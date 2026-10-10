"""Re-attach the hand-written User/ code to the Keil project after CubeMX regenerates.

WHY THIS EXISTS
CubeMX rewrites the whole .uvprojx on every "Generate Code". It preserves USER CODE
blocks inside generated .c files, but it does NOT preserve edits to the project file.
So these reappear every time and have to be added back by hand:

    groups      Application/User/Bsp   (lcd.c, touch.c, bsp_log.c)
                Application/User/Ui    (gfx_text.c, ui_model.c, ui_main.c)
    files       Application/User/App   (app_bringup.c, app_input.c, can_if.c, can_proto.c)
    include     ../User/Bsp/Inc ../User/App/Inc ../User/Ui/Inc ../User/Shared

Forgetting any of them gives a confusing failure: either "cannot open source file
app_bringup.h", or - worse - a build that SUCCEEDS but silently leaves the new code
out of the image, so the firmware behaves like the old one.

    python tools/fix_keil_project.py            # repair, report what changed
    python tools/fix_keil_project.py --check    # report only, exit 1 if repair needed

Safe to run repeatedly: it only adds what is missing and never removes anything.
"""
from pathlib import Path
import argparse
import re
import shutil
import sys

PROJ = Path(__file__).resolve().parents[1] / 'STM_POWER_LCD' / 'MDK-ARM' / 'STM_POWER_LCD.uvprojx'

# group name -> list of (file name, path relative to MDK-ARM/)
GROUPS = {
    'Application/User/Bsp': [
        ('lcd.c', '../User/Bsp/Src/lcd.c'),
        ('touch.c', '../User/Bsp/Src/touch.c'),
        ('bsp_log.c', '../User/Bsp/Src/bsp_log.c'),
    ],
    'Application/User/App': [
        ('app_bringup.c', '../User/App/Src/app_bringup.c'),
        ('app_input.c', '../User/App/Src/app_input.c'),
        ('can_if.c', '../User/App/Src/can_if.c'),
        ('can_proto.c', '../User/App/Src/can_proto.c'),
    ],
    'Application/User/Ui': [
        ('gfx_text.c', '../User/Ui/Src/gfx_text.c'),
        ('ui_model.c', '../User/Ui/Src/ui_model.c'),
        ('ui_main.c', '../User/Ui/Src/ui_main.c'),
    ],
}

INCLUDE_DIRS = ['../User/Bsp/Inc', '../User/App/Inc', '../User/Ui/Inc', '../User/Shared']

# the header-only shared protocol copy must exist for the build to work
REQUIRED_FILES = [
    '../User/Shared/can_protocol.h',
    '../User/Bsp/Inc/bsp_config.h',
    '../User/Ui/Inc/ui_main.h',
]

FILE_ENTRY = """        <File>
          <FileName>{name}</FileName>
          <FileType>1</FileType>
          <FilePath>{path}</FilePath>
        </File>
"""

GROUP_TEMPLATE = """      <Group>
        <GroupName>{group}</GroupName>
        <Files>
{files}        </Files>
      </Group>
"""


def file_entry(name, path):
    return FILE_ENTRY.format(name=name, path=path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='report only; exit 1 when a repair is needed')
    args = ap.parse_args()

    if not PROJ.exists():
        print(f'ERROR: {PROJ} not found')
        return 2

    text = PROJ.read_text(encoding='utf-8', errors='replace')
    original = text
    actions = []

    # CMSIS templates default to 25MHz and CubeMX replaces that file. A target
    # define applies to every translation unit, including SystemCoreClockUpdate.
    defines = re.search(r'<Define>([^<]*\bSTM32H750xx\b[^<]*)</Define>', text)
    if defines is None:
        print('ERROR: could not locate the H750 target compiler definitions')
        return 2
    parts = [p for p in defines.group(1).split(',') if p and not p.startswith('HSE_VALUE=')]
    parts.append('HSE_VALUE=8000000')
    required = ','.join(parts)
    if required != defines.group(1):
        text = text[:defines.start()] + '<Define>' + required + '</Define>' + text[defines.end():]
        actions.append('target HSE_VALUE=8000000 added/corrected for CMSIS and HAL')

    # ---- 1. missing source files on disk (generation does not delete User/) ----
    mdk = PROJ.parent
    for rel in REQUIRED_FILES:
        if not (mdk / rel).exists():
            print(f'ERROR: {rel} is missing on disk - the User/ tree is incomplete')
            return 2

    # ---- 2. include paths ------------------------------------------------------
    m = re.search(r'<IncludePath>(\.\./Core/Inc;[^<]*)</IncludePath>', text)
    if not m:
        print('ERROR: could not find the application IncludePath line')
        return 2
    current = m.group(1)
    parts = current.split(';')
    added_inc = [d for d in INCLUDE_DIRS if d not in parts]
    if added_inc:
        # insert right after ../Core/Inc so the list stays tidy
        new_parts = []
        for p in parts:
            new_parts.append(p)
            if p == '../Core/Inc':
                new_parts.extend(added_inc)
        text = text[:m.start()] + '<IncludePath>' + ';'.join(new_parts) + '</IncludePath>' + text[m.end():]
        actions.append(f'include paths added: {", ".join(added_inc)}')

    # ---- 3. file groups --------------------------------------------------------
    existing_groups = set(re.findall(r'<GroupName>([^<]+)</GroupName>', text))
    for group, files in GROUPS.items():
        present = [f for f, _ in files if f in text]
        missing = [(f, p) for f, p in files if f not in text]

        if group in existing_groups and not missing:
            continue

        if group in existing_groups:
            # group exists but is missing entries: insert before its </Files>
            gstart = text.index(f'<GroupName>{group}</GroupName>')
            fend = text.index('</Files>', gstart)
            block = ''.join(file_entry(f, p) for f, p in missing)
            text = text[:fend] + block + '      ' + text[fend:]
            actions.append(f'{group}: added {", ".join(f for f, _ in missing)}')
        else:
            block = GROUP_TEMPLATE.format(
                group=group, files=''.join(file_entry(f, p) for f, p in files))
            idx = text.rfind('</Groups>')
            if idx < 0:
                print('ERROR: </Groups> not found; the project file looks wrong')
                return 2
            text = text[:idx] + block + '    ' + text[idx:]
            actions.append(f'{group}: created with {len(files)} files')

        if present and missing:
            pass

    # ---- 4. sanity: enumerate what the project will compile -------------------
    compiled = sorted(set(re.findall(r'<FilePath>(\.\./User/[^<]+)</FilePath>', text)))
    expected = sorted({p for files in GROUPS.values() for _, p in files})

    if not actions:
        print('Keil project already has every User/ group and include path.')
    else:
        print(f'{len(actions)} repair(s) needed:' if args.check else 'applying repairs:')
        for a in actions:
            print(f'  - {a}')

    if args.check:
        return 1 if actions else 0

    if text != original:
        backup = PROJ.with_suffix('.uvprojx.bak')
        shutil.copyfile(PROJ, backup)
        # keep the file byte-identical apart from our edits: write as UTF-8, no BOM
        PROJ.write_text(text, encoding='utf-8', newline='')
        print(f'written (backup: {backup.name})')

    print(f'\nUser/ sources the project will compile ({len(compiled)}):')
    for c in compiled:
        print(f'  {c}')
    missing_files = [p for p in expected if p not in compiled]
    if missing_files:
        print('STILL MISSING: ' + ', '.join(missing_files))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
