"""Report the worst-case length of every formatted log line in the firmware.

check_log_lengths.py is the GATE (it exits non-zero when a line can overflow the
96-byte log field); this is the REPORT you read when it fails, and the way to see
the margin on the lines that are close. Run it after any change to a diagnostic
line:

    python tools/dump_log_lines.py          # only the lines over the field
    python tools/dump_log_lines.py --all    # every line, widest first

Both tools share one parser, so the numbers cannot disagree.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_log_lengths as C

root = Path(__file__).resolve().parents[1]
show_all = '--all' in sys.argv[1:]
for relative in C.SOURCES:
    text = (root / relative).read_text(encoding='utf-8')
    rows = []
    for match in re.finditer(r'snprintf\s*\(', text):
        line = text[:match.start()].count('\n') + 1
        arguments, pieces = C.call_arguments(text, match.end() - 1)
        if len(arguments) < 3:
            continue
        literal = ''.join(pieces[2])
        length, unknown = C.literal_length(literal, arguments[3:])
        rows.append((length, line, unknown, literal))
    print('==', relative)
    for length, line, unknown, literal in sorted(rows, reverse=True):
        if not show_all and length <= 95 and not unknown:
            continue
        note = f' unresolved={unknown}' if unknown else ''
        print(f'  {line:5d} {"OVER" if length > 95 else "ok  "} {length:3d}{note}  {literal[:110]}')
