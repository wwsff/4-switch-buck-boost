"""Detect if the H750 .ioc has been silently reverted.

WHY THIS EXISTS. CubeMX holds an in-memory copy of the .ioc. If it is open when
the file is changed on disk - by this session, or by a hand edit - then the next
save writes the STALE in-memory copy back and the configuration silently reverts.
That has already happened twice: once reverting the HSE PLL source (which
produced a 576 MHz clock) and once reverting the 100 kHz I2C timing and the whole
reserved-QSPI pin set.

A reverted .ioc does not look broken. CubeMX opens it happily and generates code
from it. This compares the file against a frozen snapshot of the configuration
INTENT - the keys a human chose - so a revert is reported instead of discovered
later as a wrong clock.

Derived keys are deliberately NOT frozen: CubeMX recomputes every *_Freq_Value and
Calculate* key whenever it touches the file, and pinning those would report a
benign recompute as a fault. RCC.HSE_VALUE IS frozen: it is an input (the crystal
frequency), not a result.

    python tools/check_ioc_intent.py             # compare against the snapshot
    python tools/check_ioc_intent.py --freeze     # re-record the snapshot
"""
from pathlib import Path
import argparse
import json
import re
import sys

proj = Path(__file__).resolve().parents[1]
IOC = proj / 'STM_POWER_LCD' / 'STM_POWER_LCD.ioc'
SNAP = Path(__file__).resolve().parent / 'ioc_intent.json'

# CubeMX recomputes these on every save; freezing them would cry wolf.
RECOMPUTED = re.compile(r'(Freq_Value$|Calculate[A-Za-z]*$|SYSCLKFreq_VALUE$)')


def intent_of(text):
    out = {}
    pins = []
    for line in text.splitlines():
        if line.startswith('#') or '=' not in line:
            continue
        k, v = line.split('=', 1)
        if re.fullmatch(r'Mcu\.Pin\d+', k):
            pins.append(v)  # CubeMX reorders this inventory on regeneration
            continue
        if RECOMPUTED.search(k):
            continue
        out[k] = ','.join(sorted(v.split(','))) if k.endswith('.IPParameters') else v
    out['Mcu.PinInventory'] = ','.join(sorted(pins))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--freeze', action='store_true',
                    help='re-record the snapshot from the current .ioc')
    args = ap.parse_args()

    if not IOC.exists():
        print(f'ERROR: {IOC} not found')
        return 2
    current = intent_of(IOC.read_text(encoding='utf-8', errors='replace'))

    if args.freeze:
        SNAP.write_text(json.dumps(current, indent=1, sort_keys=True, ensure_ascii=False),
                        encoding='utf-8', newline='\n')
        print(f'frozen {len(current)} configuration keys to {SNAP.name}')
        return 0

    if not SNAP.exists():
        print(f'ERROR: no snapshot at {SNAP}. Run with --freeze first.')
        return 2
    frozen = json.loads(SNAP.read_text(encoding='utf-8'))

    changed = []
    for k in sorted(set(frozen) & set(current)):
        if frozen[k] != current[k]:
            changed.append((k, frozen[k], current[k]))
    lost = sorted(set(frozen) - set(current))
    added = sorted(set(current) - set(frozen))

    if not (changed or lost or added):
        print(f'.ioc matches the frozen intent ({len(frozen)} keys).')
        return 0

    print('The .ioc no longer matches the configuration this project was built for.')
    print('This is the signature of CubeMX saving a stale in-memory copy.\n')

    if changed:
        print(f'{len(changed)} key(s) changed:')
        for k, was, now in changed:
            print(f'  {k}')
            print(f'      was {was!r}')
            print(f'      now {now!r}')
    if lost:
        print(f'\n{len(lost)} key(s) REMOVED (a whole peripheral or pin group can vanish this way):')
        for k in lost[:40]:
            print(f'  {k}')
        if len(lost) > 40:
            print(f'  ... and {len(lost) - 40} more')
    if added:
        print(f'\n{len(added)} key(s) added:')
        for k in added[:40]:
            print(f'  {k}')
        if len(added) > 40:
            print(f'  ... and {len(added) - 40} more')

    print('\nRecovery: close CubeMX WITHOUT saving, then restore the intended .ioc.')
    print('A good copy is kept as STM_POWER_LCD.ioc.bak only if it predates these')
    print('changes - otherwise fix the listed keys in CubeMX and re-freeze.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
