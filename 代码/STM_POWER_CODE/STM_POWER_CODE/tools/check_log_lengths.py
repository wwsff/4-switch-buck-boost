"""Static worst-case length check for every log line the firmware formats.

Why this exists: 0.3.22 shipped two 1 Hz diagnostic lines (code 17 and code 9)
that were LONGER than the 96-byte log text field of the protocol frame, so they
were truncated silently on the wire - the operator saw a cut-off line and had no
way to know a field was missing. Hand-counting characters is exactly the kind of
check that rots; this tool computes the worst case from the source instead.

How it works: for every snprintf() that formats into a local char buffer it reads
the format string and substitutes each conversion with the widest value that
argument can produce, then compares the result against PD_LOG_TEXT_MAX (95: a log
text field is 96 bytes including the NUL) and against the buffer it is written
into. It also measures PD_Log()/PD_TryLog() calls that pass a string literal.

Two kinds of argument bound are used, and neither is a guess:
  * TYPE width - the widest text a 32-bit (un)signed int/long or a %.3g float can
    produce. Used for everything that feeds on a counter or a timer.
  * PROJECT bound - for values the firmware cannot print wider than physically
    possible (a measured millivolt reading, a 16-bit compare count, an ADC code
    passed through PD_LOG_VALUE). These are listed in PROJECT_BOUNDS below with
    the reason, and any conversion that is neither a type width nor a listed
    project bound is reported as unresolved - never silently accepted.

Boundaries, stated honestly:
  * A format string passed as a variable (the FSTOP/why macro payload) cannot be
    bounded here; it is reported as unresolved. Its own format string is checked
    at the definition site instead.
  * A 2 A / 24 V build would need PROJECT_BOUNDS reviewed; the checker is tied to
    the declared 2..18 V, 0..2 A configuration on purpose.

Usage: python tools/check_log_lengths.py [--verbose]
Exit code 0 = every resolvable line fits. 1 = at least one does not.
"""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[1]
# Every translation unit that formats a log line. pd_diag.c holds the two rotating
# diagnostic frames, so it belongs here: leaving a module out is how a line escapes
# the gate entirely.
SOURCES = ['App/pd_board.c', 'App/pd_core.c', 'App/pd_power.c', 'App/pd_diag.c']

# Widest textual form of each conversion, for the argument types this project
# uses: 32-bit int/long/unsigned, 16-bit fields, and the float formats that
# actually appear.
TYPE_WIDTHS = {
    'd': 11, 'i': 11, 'u': 10, 'x': 8, 'X': 8, 'o': 11, 'c': 1,
    'ld': 11, 'li': 11, 'lu': 10, 'lx': 8, 'lX': 8, 'lo': 11,
    'lld': 20, 'llu': 20,
    'hd': 6, 'hu': 5, 'hx': 4, 'hhx': 2, 'hhu': 3,
    'zu': 10, 'zd': 11,
    'g': 10, 'f': 14, 'e': 13,
}

# Project bounds: conversion -> (width, why). Matched against the printed
# argument expression, because the width depends on WHICH value is printed, not
# only on its type. Anything not listed falls back to the type width.
PROJECT_BOUNDS = [
    # A signed millivolt/current reading, range-checked before it is logged:
    # "-2000" is the widest realistic form (the ADC path cannot return more).
    (r'(vin_mv|vout_mv|iout_ma|iin_ma|vref_mv|duty_in|duty_out|_mv\b)', 5,
     'measured value in engineering units, guarded by the 2..18 V / 0..2 A range'),
    # A 16-bit HRTIM compare count, printed as raw/32 (see PD_LOG_VALUE_MAX).
    (r'PD_LOG_VALUE\(', 4, 'bounded by PD_LOG_VALUE: three digits or 1000="+" marker'),
    # Two fields of the 1 Hz control trace (0.3.28) that are bounded by construction: the
    # modulation limit mask has seven defined bits, and sat is one of 0/1/2/3. Without these
    # entries the checker assumed a 32-bit width (8 and 10 characters), which both hid the real
    # headroom and pushed the line over the limit it was measuring.
    (r'mod_limit', 4, 'PD_MOD_LIMIT_* mask, 0x00..0x7F (four characters as 0x%02X)'),
    (r'\bsat\b', 1, 'saturation reason, one of 0/1/2/3'),
    # Small enumerated fields: state, fault code, flags, sensor index.
    (r'(state|fault|reason|sensor|samples|stage|LastReason|phase|pin_at_event|en\b|gates)',
     3, 'enum/flag field (0..255)'),
    # 16-bit ADC codes.
    (r'(raw\[|il_raw\[|input_raw\[|fault_|oc_cmp|oc_mcntr)', 4, '16-bit ADC code or compare count'),
    # Timers in milliseconds since boot, LOGGED in fault paths only.
    (r'(dt_ms|now - )', 8, 'millisecond counter in a fault-path line'),
    # Calibration coefficients (1000..9999) and the measured reference (3285).
    (r'(CAL_GAIN|CAL_OFFSET|SCALE_REF)', 4, 'calibration coefficient 0..9999'),
    # HRTIM compare values: 0..PD_SW_PERIOD = 21760, i.e. five digits. These are NOT
    # put through PD_LOG_VALUE: its 999 cap would print "1000" for every realistic
    # duty and the line would stop explaining anything.
    (r'(duty_in|duty_out|cmp_in|cmp_out|snapshot\.cmp|last_counter|CmpLastCounter|CmpCrit|'
     r'_cnt\b|\bcnt\b)',
     5, 'HRTIM compare count / control term in counts (|value| <= PD_SW_PERIOD = 21760)'),
    # The ADC scheduler rate in kHz (100 or 125) and the injected-rank sample count.
    (r'(PD_FAST_HZ / 1000|PD_FAST_HZ/1000)', 3, 'sampling rate in kHz (1..999)'),
]

# A %s width is not knowable from the format alone, so every string that can reach
# a log line is audited separately (audit_string_arguments below): only bounded
# literals are allowed, and none may exceed this many characters. The longest one
# in the firmware today is "COMPARE COMMIT FAILED" (20 characters).
MAX_STRING_ARGUMENT = 22


def spec_split(spec):
    """('flags', 'field', 'precision', 'length', 'conversion') of one % conversion."""
    match = re.match(r'%([-+ #0]*)(\d*)(?:\.(\d+))?(hh|h|ll|l|z|j|t|L)?([diuxXocgfeEs])', spec)
    return match.groups() if match else None


def bound_for(spec, argument, limit):
    """Widest text for this conversion given the printed argument expression."""
    parts = spec_split(spec)
    if parts is None:
        return None, 'unparsable conversion'
    flags, field, precision, length, conversion = parts
    if conversion == 's':
        return 0, 'bounded-string-argument'     # width audited separately, see below
    key = (length or '') + conversion
    width = TYPE_WIDTHS.get(key)
    if width is None:
        return None, f'unknown conversion %{spec}'
    if precision is not None and conversion in 'gfeE':
        width = max(width, 2 + int(precision) + 6)
    if field:
        width = max(width, int(field))
    if argument is not None:
        for pattern, bound, _why in PROJECT_BOUNDS:
            if re.search(pattern, argument):
                if bound < width:
                    width = bound
                break
    return width, None


def literal_length(format_string, arguments=None):
    """Worst-case length of a format string plus its argument expressions.

    Returns (length, unresolved) where unresolved lists conversions whose width
    could not be derived.
    """
    pieces = []
    index = 0
    while index < len(format_string):
        char = format_string[index]
        if char == '%':
            if format_string[index + 1:index + 2] == '%':
                pieces.append(('text', '%'))
                index += 2
                continue
            match = re.match(r'%[-+ #0]*\d*(?:\.\d+)?(?:hh|h|ll|l|z|j|t|L)?[diuxXocgfeEs]',
                             format_string[index:])
            if not match:
                pieces.append(('unknown', format_string[index:index + 8]))
                index += 1
                continue
            pieces.append(('spec', match.group(0)))
            index += len(match.group(0))
            continue
        if char == '\\':
            escape = format_string[index + 1:index + 2]
            if escape in ('n', 't', 'r', '"', '\\', '0'):
                pieces.append(('text', '\n' if escape == 'n' else escape))
                index += 2
                continue
            if escape == 'x':
                hex_digits = re.match(r'[0-9A-Fa-f]{1,2}', format_string[index + 2:])
                pieces.append(('text', 'x'))
                index += 2 + (len(hex_digits.group(0)) if hex_digits else 0)
                continue
        pieces.append(('text', char))
        index += 1
    total = sum(len(piece) for _, piece in pieces if _ == 'text')
    total += sum(len(piece) for kind, piece in pieces if kind == 'unknown')
    unresolved = [piece for kind, piece in pieces if kind == 'unknown']
    spec_index = 0
    for kind, piece in pieces:
        if kind != 'spec':
            continue
        argument = None
        if arguments is not None and spec_index < len(arguments):
            argument = arguments[spec_index]
        width, problem = bound_for(piece, argument, None)
        spec_index += 1
        if problem == 'bounded-string-argument':
            # The %s payload width is not in the format, so the reservation is the
            # audited allowance: every string that can reach a log line must be a
            # bounded literal no longer than MAX_STRING_ARGUMENT (second pass in
            # main()). A %s whose argument the audit cannot reach becomes a finding
            # there, not an unresolved entry here.
            total += MAX_STRING_ARGUMENT
        elif problem:
            unresolved.append(piece)
            total += len(piece)
        else:
            total += width
    return total, unresolved


def blank_comments(text):
    """Replace comment bytes with spaces, preserving offsets and newlines.

    The argument scanner is line-oriented only for whitespace; without this a comment
    inside an argument list became part of the argument it preceded, and an apostrophe in
    that comment could glue a following string literal into it.
    """
    result = list(text)
    index = 0
    while index < len(text):
        if text.startswith('/*', index):
            end = text.find('*/', index + 2)
            end = len(text) - 2 if end < 0 else end
            for position in range(index, end + 2):
                if result[position] != '\n':
                    result[position] = ' '
            index = end + 2
            continue
        if text.startswith('//', index):
            end = text.find('\n', index)
            end = len(text) if end < 0 else end
            for position in range(index, end):
                result[position] = ' '
            index = end
            continue
        index += 1
    return ''.join(result)


def string_literals(text, start):
    """Concatenated C string literals starting at `start` (index of the opening quote).

    Returns (value, end_index) with escape sequences resolved. Concatenation only
    continues across whitespace: a literal followed by ',' or ')' is complete.
    """
    value = []
    index = start
    while True:
        if text[index] != '"':
            break
        index += 1
        while index < len(text) and text[index] != '"':
            if text[index] == '\\':
                escape = text[index + 1]
                mapped = {'n': '\n', 't': '\t', 'r': '\r', '0': '\0', '"': '"', '\\': '\\'}.get(escape)
                if mapped is None and escape == 'x':
                    hex_digits = re.match(r'[0-9A-Fa-f]{1,2}', text[index + 2:]).group(0)
                    value.append(chr(int(hex_digits, 16)))
                    index += 2 + len(hex_digits)
                    continue
                value.append(mapped if mapped is not None else escape)
                index += 2
                continue
            value.append(text[index])
            index += 1
        index += 1
        probe = index
        while probe < len(text) and text[probe] in ' \t\r\n':
            probe += 1
        if probe < len(text) and text[probe] == '"':
            index = probe
            continue
        break
    return ''.join(value), index


def call_arguments(text, open_paren):
    """Arguments of a call, with literal arguments marked.

    Returns (arguments, pieces) where pieces[i] lists the literal pieces of
    argument i (a format string written across several lines yields one entry per
    piece, in source order). Rebuilt on string_literals so that a call passing
    several literals (buffer name, size, format) cannot glue them together and a
    concatenated format cannot swallow the next argument.
    """
    depth = 0
    index = open_paren
    arguments = []
    pieces = []
    current = []
    current_pieces = []
    while index < len(text):
        char = text[index]
        if char == '(':
            depth += 1
            if depth == 1:
                index += 1
                continue
        elif char == ')':
            depth -= 1
            if depth == 0:
                arguments.append(''.join(current))
                pieces.append(current_pieces)
                break
        elif char == '"' and depth == 1:
            literal, end = string_literals(text, index)
            current.append('\x00LITERAL\x00' + literal)
            current_pieces.append(literal)
            index = end
            continue
        elif char == ',' and depth == 1:
            arguments.append(''.join(current))
            pieces.append(current_pieces)
            current = []
            current_pieces = []
            index += 1
            continue
        if depth >= 1:
            current.append(char)
        index += 1
    return arguments, pieces


def buffer_sizes(text):
    """Local char arrays: name -> size, for the sized ones only."""
    sizes = {}
    for match in re.finditer(r'char\s+(\w+)\s*\[\s*([^\]]*)\s*\]', text):
        name, extent = match.group(1), match.group(2).strip()
        if extent.isdigit():
            sizes[name] = int(extent)
    return sizes


def main():
    verbose = '--verbose' in sys.argv[1:]
    limit = int(re.search(r'#define\s+PD_LOG_TEXT_MAX\s+(\d+)u?',
                          (root / 'App/pd_config.h').read_text(encoding='utf-8')).group(1))
    findings = []
    unresolved = []
    checked = 0
    for relative in SOURCES:
        text = blank_comments((root / relative).read_text(encoding='utf-8'))
        sizes = buffer_sizes(text)
        for match in re.finditer(r'\bsnprintf\s*\(', text):
            line = text[:match.start()].count('\n') + 1
            arguments, pieces = call_arguments(text, match.end() - 1)
            if len(arguments) < 3:
                unresolved.append((relative, line, 'unparsable snprintf'))
                continue
            target = arguments[0].strip()
            format_pieces = pieces[2]
            if not format_pieces:
                unresolved.append((relative, line,
                                   f'snprintf into {target}: format is not a literal'))
                continue
            # A format written as several adjacent literals may be followed by
            # more (the FRAW line prints two labels), so only the leading run of
            # pieces that contain a conversion or a letter is treated as format.
            format_string = ''.join(format_pieces)
            length, unknown = literal_length(format_string, arguments[3:])
            buffer = sizes.get(target)
            checked += 1
            problems = []
            if length > limit:
                problems.append(f'{length} > PD_LOG_TEXT_MAX ({limit})')
            if isinstance(buffer, int) and length + 1 > buffer:
                problems.append(f'{length + 1} bytes incl. NUL > buffer {buffer}')
            if buffer is None:
                unresolved.append((relative, line, f'snprintf into {target}: buffer size unknown'))
            if unknown:
                unresolved.append((relative, line, f'unresolved conversion(s) {unknown}'))
            if problems:
                findings.append((relative, line, target, length, '; '.join(problems)))
    for relative in SOURCES:
        text = (root / relative).read_text(encoding='utf-8')
        for match in re.finditer(r'\bPD_(Try)?Log\s*\(', text):
            line = text[:match.start()].count('\n') + 1
            arguments, pieces = call_arguments(text, match.end() - 1)
            literals = [index for index, argument in enumerate(arguments)
                        if argument.strip().startswith('\x00LITERAL\x00')]
            if not literals or len(literals) > 1:
                continue                       # a formatted buffer, or a macro wrapper
            index = literals[0]
            value = ''.join(pieces[index])
            length, unknown = literal_length(value, arguments[index + 1:])
            checked += 1
            if length > limit:
                findings.append((relative, line, 'PD_Log literal', length,
                                 f'{length} > PD_LOG_TEXT_MAX ({limit})'))
            if unknown:
                unresolved.append((relative, line, f'unresolved conversion(s) {unknown}'))
    # A %s has no derivable width, so instead of accepting the residue blindly the
    # tool audits every string that can reach a log line: each one must be a
    # bounded literal within MAX_STRING_ARGUMENT. Together the two passes cover the
    # whole line, which is the point - a hand count cannot.
    for relative in SOURCES:
        text = blank_comments((root / relative).read_text(encoding='utf-8'))
        for match in re.finditer(r'\b(PD_Log|PD_TryLog|commit_failed)\s*\(', text):
            line = text[:match.start()].count('\n') + 1
            arguments, pieces = call_arguments(text, match.end() - 1)
            groups = [pieces[i] for i in range(len(arguments)) if pieces[i]]
            if match.group(1) != 'commit_failed':
                groups = groups[1:]            # the first literal is the format itself
            for group in groups:
                for piece in group:
                    if len(piece) > MAX_STRING_ARGUMENT:
                        findings.append((relative, line, 'string argument', len(piece),
                                         f'{piece!r} exceeds the {MAX_STRING_ARGUMENT}-character '
                                         f'allowance for a %s payload'))
    if verbose:
        for item in unresolved:
            print('unresolved:', item)
    for relative, line, target, length, why in findings:
        print(f'{relative}:{line}: {target}: {why}')
    print(f'checked {checked} formatted lines; {len(findings)} over the limit; '
          f'{len(unresolved)} unresolved')
    return 1 if findings else 0


if __name__ == '__main__':
    raise SystemExit(main())
