"""Check the H750 .ioc against the settings this project depends on.

WHY: the .ioc has been silently reverted twice, because CubeMX holds an in-memory
copy and overwrites the file on save. A wrong .ioc is not obvious: CubeMX still
generates code, it just generates it from stale values. This catches that before
a build, and it also answers "is the project still configured the way the plan
says" in one command.

It re-derives the timing numbers from the raw register/divider values rather than
trusting the derived fields, because those derived fields are exactly what goes
stale.

    python tools/check_ioc.py            # check
    python tools/check_ioc.py --quiet    # only print problems

Exit codes:
    0  .ioc correct AND the generated code matches it - safe to build
    1  .ioc correct, but the generated code is stale - press Generate (no .ioc edit)
    2  the .ioc itself is wrong - fix it in CubeMX

The split matters: "press Generate" and "edit the .ioc" are different actions, and
reporting a stale build as a configuration error sends you to the wrong one.
"""
from pathlib import Path
import argparse
import re
import sys

IOC = Path(__file__).resolve().parents[1] / 'STM_POWER_LCD' / 'STM_POWER_LCD.ioc'

# These are the values the project actually needs. Each entry: (key, expected, why)
REQUIRED = [
    ('RCC.PLLSourceVirtual', 'RCC_PLLSOURCE_HSE',
     'PLL must run from the actual 8 MHz crystal on ATK-M100Z-M7'),
    ('RCC.HSE_VALUE', '8000000', 'schematic X1 and operator both confirm 8MHz'),
    ('RCC.DIVM1', '4', 'PLL1 input divider: 8 MHz / 4 = 2 MHz VCO input'),
    ('RCC.DIVN1', '225', 'PLL1 multiplier: 2 MHz * 225 = 450 MHz VCO'),
    ('RCC.SYSCLKSource', 'RCC_SYSCLKSOURCE_PLLCLK', 'SYSCLK from PLL'),
    ('RCC.PLLFRACN', '0', 'no fractional part needed with these integers'),
    ('RCC.FDCANCLockSelection', 'RCC_FDCANCLKSOURCE_HSE',
     'PLL1Q would be 56.25 MHz, above the FDCAN 50 MHz ceiling'),
    ('RCC.SPI123CLockSelection', 'RCC_SPI123CLKSOURCE_CLKP',
     'default PLL1Q = 225 MHz trips the SPI over-clock warning'),
    ('SYS.TimeBaseIP', 'TIM6', 'HAL timebase must not be SysTick under FreeRTOS'),
    ('NVIC.TimeBaseIP', 'TIM6', 'same'),
    ('SPI1.NSS', 'SPI_NSS_SOFT', 'panel chip select is driven by software'),
    ('SPI1.NSSPMode', 'SPI_NSS_PULSE_DISABLE', 'no NSS pulse with a software CS'),
    ('SPI1.Direction', 'SPI_DIRECTION_2LINES', 'full duplex'),
    ('TIM3.EncoderMode', 'TIM_ENCODERMODE_TI12', 'quadrature encoder, x4 decoding'),
    ('TIM3.Period', '65535', '16-bit full range so the counter wraps cleanly'),
    ('USART2.BaudRate', '115200', 'log link'),
    ('USART2.WordLength', 'WORDLENGTH_8B', '8N1 (CubeMX stores the bare constant)'),
    ('USART2.Parity', 'PARITY_NONE', '8N1 (CubeMX stores the bare constant)'),
    ('USART2.StopBits', 'STOPBITS_1', '8N1 (CubeMX stores the bare constant)'),
    ('FDCAN2.FrameFormat', 'FDCAN_FRAME_CLASSIC', 'matching the G474 power board'),
    ('FDCAN2.Mode', 'FDCAN_MODE_NORMAL', 'normal bus mode'),
    ('FDCAN2.RxFifo0ElmtsNbr', '16', 'the ISR needs actual RX FIFO0 RAM entries'),
    ('PE5.GPIO_Label', 'KEY2', 'corrected physical board marking'),
    ('PE6.GPIO_Label', 'KEY1', 'corrected physical board marking'),
]

# Pins that must exist, and the signal each must carry.
REQUIRED_PINS = {
    'PA5': 'SPI1_SCK', 'PA7': 'SPI1_MOSI', 'PA6': 'SPI1_MISO',
    'PA4': 'GPIO_Output', 'PC1': 'GPIO_Output', 'PC0': 'GPIO_Output',
    'PB8': 'I2C1_SCL', 'PB9': 'I2C1_SDA',
    'PE0': 'GPXTI0', 'PE1': 'GPIO_Output',
    'PC6': 'S_TIM3_CH1', 'PC7': 'S_TIM3_CH2', 'PC4': 'GPIO_Input',
    'PE5': 'GPIO_Input', 'PE6': 'GPIO_Input',
    'PB13': 'FDCAN2_TX', 'PB12': 'FDCAN2_RX',
    'PA2': 'USART2_TX', 'PA3': 'USART2_RX',
}

# QSPI flash lines the plan says to keep reserved but NOT activate.
RESERVED_QSPI = ['PB2', 'PB6', 'PD11', 'PD12', 'PD13', 'PE2']

# GPIOs that must carry GPIOParameters, otherwise MX_GPIO_Init() is generated empty.
# This was a real bug: the pins had Signal=GPIO_Output but no parameters, so the
# generated init configured nothing at all.
OUTPUTS_NEEDING_PARAMS = ['PA4', 'PC0', 'PC1', 'PE1']

IP_LIST = ['CORTEX_M7', 'DEBUG', 'FDCAN2', 'FREERTOS', 'I2C1', 'MEMORYMAP', 'NVIC',
           'RCC', 'SPI1', 'SYS', 'TIM3', 'USART2']

failures = []
warnings = []


def load():
    if not IOC.exists():
        print(f'ERROR: {IOC} not found')
        sys.exit(2)
    text = IOC.read_text(encoding='utf-8', errors='replace')
    kv = {}
    for line in text.splitlines():
        if line.startswith('#') or '=' not in line:
            continue
        k, v = line.split('=', 1)
        kv[k] = v
    return text, kv


def check_float(text, kv, problems, notes):
    """Re-derive the clock tree from the raw values; the derived fields go stale."""

    def num(k, default=None):
        v = kv.get(k)
        try:
            return int(v)
        except (TypeError, ValueError):
            return default

    hse = num('RCC.HSE_VALUE', 8000000)
    m, n, p = num('RCC.DIVM1'), num('RCC.DIVN1'), 2
    if not all([hse, m, n]):
        problems.append('clock: HSE_VALUE / DIVM1 / DIVN1 missing, cannot derive SYSCLK')
        return
    vco_in = hse / m
    vco = vco_in * n
    sysclk = vco / p
    apb = sysclk / 2

    notes.append(f'  SYSCLK   {sysclk/1e6:8.3f} MHz   (VCO {vco/1e6:.0f} MHz, VCO-in {vco_in/1e6:.3f} MHz)')
    notes.append(f'  APB1/2   {apb/1e6:8.3f} MHz')

    if abs(sysclk - 225e6) > 1:
        problems.append(f'clock: SYSCLK is {sysclk/1e6:.3f} MHz, expected 225 MHz')
    if not (2e6 <= vco_in <= 4e6):
        problems.append(f'clock: VCO input {vco_in/1e6:.3f} MHz is outside the 2-4 MHz band '
                        f'(PLL1VCIRANGE_1)')
    if not (192e6 <= vco <= 836e6):
        problems.append(f'clock: VCO {vco/1e6:.0f} MHz is outside 192-836 MHz')
    if sysclk > 480e6:
        problems.append(f'clock: SYSCLK {sysclk/1e6:.0f} MHz exceeds the H750 maximum of 480 MHz')

    # --- I2C1: decode TIMINGR and recompute fSCL -----------------------------
    t = kv.get('I2C1.Timing')
    if not t:
        problems.append('I2C1.Timing missing')
    else:
        try:
            reg = int(t, 16)
        except ValueError:
            problems.append(f'I2C1.Timing is not hex: {t}')
            reg = None
        if reg is not None:
            presc = (reg >> 28) & 0xF
            scldel = (reg >> 20) & 0xF
            sdadel = (reg >> 16) & 0xF
            sclh = (reg >> 8) & 0xFF
            scll = reg & 0xFF
            tp = (presc + 1) / apb
            fscl = 1.0 / (((scll + 1) + (sclh + 1) + 2) * tp)
            notes.append(f'  I2C1 SCL {fscl/1000:8.1f} kHz   (PRESC={presc} SCLH={sclh} SCLL={scll}, '
                         f'setup {scldel*tp*1e9:.0f} ns, hold {sdadel*tp*1e9:.0f} ns)')
            if not (95e3 <= fscl <= 105e3):
                problems.append(f'I2C1: {fscl/1000:.1f} kHz, expected ~100 kHz for FT6336 bring-up')
            if scldel * tp * 1e9 < 100:
                problems.append('I2C1: data setup time below the 100 ns minimum')
            if sdadel * tp * 1e9 < 250:
                problems.append('I2C1: data hold time below the 250 ns standard-mode minimum')

    # --- SPI1 ---------------------------------------------------------------
    pre = kv.get('SPI1.BaudRatePrescaler', '')
    mm = re.search(r'SPI_BAUDRATEPRESCALER_(\d+)', pre)
    if not mm:
        problems.append(f'SPI1.BaudRatePrescaler not recognised: {pre!r}')
    else:
        div = int(mm.group(1))
        # SPI123 uses CKPER, NOT APB2. Check the actual generated mux and
        # oscillator divider instead of trusting CubeMX's derived frequency.
        main_src = (IOC.parent / 'Core/Src/main.c').read_text(encoding='utf-8')
        spi_src = (IOC.parent / 'Core/Src/spi.c').read_text(encoding='utf-8')
        if ('Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP' not in spi_src or
                'CkperClockSelection = RCC_CLKPSOURCE_HSI' not in main_src):
            problems.append('SPI1: generated CKPER/HSI route changed; recheck kernel clock')
        hsi_div = re.search(r'HSIState\s*=\s*RCC_HSI_DIV(\d+)', main_src)
        if hsi_div is None:
            problems.append('SPI1: cannot derive HSI divider from generated main.c')
            return
        kernel = 64000000 / int(hsi_div.group(1))
        sclk = kernel / div
        notes.append(f'  SPI1 SCLK {sclk/1e6:7.3f} Mbps  (HSI / {hsi_div.group(1)} '
                     f'-> CKPER -> SPI123 / {div})')
        if num('RCC.SPI123Freq_Value') != kernel or num('RCC.CKPERFreq_Value') != kernel:
            problems.append('SPI1: .ioc derived kernel clock differs from generated HSI route')
        if sclk > 62.5e6:
            problems.append(f'SPI1: {sclk/1e6:.2f} Mbps exceeds the ST7789 write-cycle limit of 62.5 MHz')

    # --- FDCAN2: bit timing from the raw prescaler and segments --------------
    fcan = hse if kv.get('RCC.FDCANCLockSelection') == 'RCC_FDCANCLKSOURCE_HSE' else None
    if fcan:
        try:
            ps = int(kv['FDCAN2.NominalPrescaler'])
            s1 = int(kv['FDCAN2.NominalTimeSeg1'])
            s2 = int(kv['FDCAN2.NominalTimeSeg2'])
            if int(kv['FDCAN2.NominalSyncJumpWidth']) > s2:
                problems.append('FDCAN2: SJW must not exceed Seg2')
            bitq = s1 + s2 + 1
            tq = ps / fcan
            rate = 1.0 / (bitq * tq)
            sp = (1 + s1) / bitq * 100.0
            notes.append(f'  FDCAN2   {rate/1000:8.1f} kbit/s (tq {tq*1e9:.0f} ns, {bitq} tq, '
                         f'sample point {sp:.1f} %)')
            if abs(rate - 500e3) > 1:
                problems.append(f'FDCAN2: {rate:.0f} bit/s, expected exactly 500000')
            if not (75.0 <= sp <= 90.0):
                warnings.append(f'FDCAN2: sample point {sp:.1f} % is outside the usual 75-90 %')
        except (KeyError, ValueError):
            problems.append('FDCAN2: nominal timing fields missing')



# ---------------------------------------------------------------------------
# Generated-code cross-check
#
# The failure mode that actually bit this project was not a wrong .ioc: it was a
# CORRECT .ioc whose generated code had never been regenerated. Checking only the
# .ioc cannot see that. These functions compare what CubeMX actually wrote into
# Core/Src against what the .ioc asks for.
# ---------------------------------------------------------------------------
GEN_DIR = IOC.parent / 'Core' / 'Src'
GENERATED_EXPECT = [
    ('main.c', r'RCC_OscInitStruct\.PLL\.PLLM\s*=\s*(\d+)', 'RCC.DIVM1', 'PLL1 input divider'),
    ('main.c', r'RCC_OscInitStruct\.PLL\.PLLN\s*=\s*(\d+)', 'RCC.DIVN1', 'PLL1 multiplier'),
    ('i2c.c', r'hi2c1\.Init\.Timing\s*=\s*(0x[0-9A-Fa-f]+)', 'I2C1.Timing',
     'I2C1 timing has not been regenerated'),
    ('usart.c', r'huart2\.Init\.BaudRate\s*=\s*(\d+)', 'USART2.BaudRate',
     'USART2 baud rate has not been regenerated'),
    ('usart.c', r'huart2\.Init\.WordLength\s*=\s*(\S+);', None,
     'USART2 word length'),
    ('usart.c', r'huart2\.Init\.Parity\s*=\s*(\S+);', None, 'USART2 parity'),
    ('usart.c', r'huart2\.Init\.StopBits\s*=\s*(\S+);', None, 'USART2 stop bits'),
    ('fdcan.c', r'hfdcan2\.Init\.NominalPrescaler\s*=\s*(\d+)', 'FDCAN2.NominalPrescaler',
     'FDCAN2 prescaler not regenerated (bit rate would be wrong)'),
    ('fdcan.c', r'hfdcan2\.Init\.NominalTimeSeg1\s*=\s*(\d+)', 'FDCAN2.NominalTimeSeg1',
     'FDCAN2 Seg1 not regenerated'),
    ('fdcan.c', r'hfdcan2\.Init\.NominalTimeSeg2\s*=\s*(\d+)', 'FDCAN2.NominalTimeSeg2',
     'FDCAN2 Seg2 not regenerated'),
    ('fdcan.c', r'hfdcan2\.Init\.NominalSyncJumpWidth\s*=\s*(\d+)', 'FDCAN2.NominalSyncJumpWidth',
     'FDCAN2 SJW not regenerated'),
    ('fdcan.c', r'hfdcan2\.Init\.RxFifo0ElmtsNbr\s*=\s*(\d+)', 'FDCAN2.RxFifo0ElmtsNbr',
     'FDCAN2 RX FIFO0 not allocated'),
    ('tim.c', r'htim3\.Init\.Prescaler\s*=\s*(\d+)', 'TIM3.Prescaler',
     'TIM3 prescaler not regenerated'),
    ('tim.c', r'htim3\.Init\.Period\s*=\s*(\d+)', 'TIM3.Period',
     'TIM3 period not regenerated'),
    ('spi.c', r'hspi1\.Init\.BaudRatePrescaler\s*=\s*(\S+);', None,
     'SPI1 prescaler not regenerated'),
    ('gpio.c', r'GPIO_InitStruct\.Pin\s*=\s*(\S+)', None,
     'GPIO init not regenerated (the empty MX_GPIO_Init bug)'),
]


def check_generated(kv, problems, stale, notes):
    if not GEN_DIR.is_dir():
        notes.append('  (no generated Core/Src yet: cannot cross-check)')
        return
    seen = {}
    main = (GEN_DIR / 'main.c').read_text(encoding='utf-8', errors='replace')
    if 'RCC_PLL1VCIRANGE_1' not in main:
        problems.append('main.c: PLL1 input range must match the 2MHz input')
    project = (IOC.parent / 'MDK-ARM/STM_POWER_LCD.uvprojx').read_text(encoding='utf-8')
    defines = re.search(r'<Define>([^<]*\bSTM32H750xx\b[^<]*)</Define>', project)
    target_hse = re.search(r'(?:^|,)HSE_VALUE=(\d+)(?:,|$)', defines.group(1)) if defines else None
    if target_hse is None or int(target_hse.group(1)) != int(kv['RCC.HSE_VALUE']):
        problems.append('Keil target: HSE_VALUE must be defined as the physical 8MHz crystal')
    for rel in ['Core/Inc/stm32h7xx_hal_conf.h']:
        body = (IOC.parent / rel).read_text(encoding='utf-8', errors='replace')
        hse = re.search(r'#define\s+HSE_VALUE[^\n]*?(\d{7,})', body)
        if hse is None or int(hse.group(1)) != int(kv['RCC.HSE_VALUE']):
            problems.append(f'{rel}: HSE_VALUE disagrees with the physical crystal / .ioc')
    cmsis = (IOC.parent / 'Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c').read_text(encoding='utf-8')
    if not re.search(r'#if\s+!defined\s*\(HSE_VALUE\)', cmsis):
        problems.append('CMSIS: its fallback HSE_VALUE is no longer protected by the target define')
    notes.append('  HSE 8MHz checked in IOC, HAL and Keil target (overrides CMSIS template default)')
    header = (IOC.parent / 'Core/Inc/main.h').read_text(encoding='utf-8', errors='replace')
    for key, pin in [('KEY1', 6), ('KEY2', 5)]:
        if not re.search(rf'#define\s+{key}_Pin\s+GPIO_PIN_{pin}\b', header):
            problems.append(f'main.h: {key} must be on PE{pin}')
    for fname, pattern, ioc_key, why in GENERATED_EXPECT:
        path = GEN_DIR / fname
        if not path.exists():
            problems.append(f'{fname} is missing from {GEN_DIR}')
            continue
        body = path.read_text(encoding='utf-8', errors='replace')
        m = re.search(pattern, body)
        if not m:
            problems.append(f'{fname}: pattern not found for {ioc_key or why} - {why}')
            continue
        got = m.group(1)
        if ioc_key:
            want = kv.get(ioc_key)
            if want is None:
                continue
            if got.lower() != want.lower():
                # The .ioc is the source of truth and it is correct; the GENERATED
                # code is one revision old. That is a pending action, not a
                # configuration fault - the distinction matters because the fix is
                # completely different (press Generate, do not edit the .ioc).
                stale.append(f'{fname}: {ioc_key} is {got}, the .ioc says {want} ({why})')
            else:
                seen[fname] = True
        else:
            seen[fname] = True
    # the empty-MX_GPIO_Init bug: gpio.c must actually configure pins
    gpio = (GEN_DIR / 'gpio.c')
    if gpio.exists():
        body = gpio.read_text(encoding='utf-8', errors='replace')
        if 'HAL_GPIO_Init' not in body:
            problems.append('gpio.c contains no HAL_GPIO_Init call: every GPIO output '
                            'and input is unconfigured (the original bug)')
    if seen:
        notes.append(f'  generated code cross-checked against the .ioc: '
                     f'{len(seen)} files agree')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true', help='only print problems')
    args = ap.parse_args()

    text, kv = load()
    notes = []

    def say(s):
        if not args.quiet:
            print(s)

    say(f'checking {IOC.name}')

    # --- required key/value pairs -------------------------------------------
    for key, want, why in REQUIRED:
        got = kv.get(key)
        if got is None:
            failures.append(f'{key} is MISSING ({why})')
        elif got != want:
            failures.append(f'{key} = {got!r}, expected {want!r} ({why})')

    # --- pins ---------------------------------------------------------------
    pins = {}
    for line in text.splitlines():
        mm = re.match(r'^(P[A-Z]\d+[^=]*)\.Signal=(.*)$', line)
        if mm:
            pins[mm.group(1).split('.')[0]] = mm.group(2)
    for pin, signal in REQUIRED_PINS.items():
        got = pins.get(pin)
        if got is None:
            failures.append(f'{pin}: no Signal defined (expected {signal})')
        elif got != signal:
            failures.append(f'{pin}: Signal is {got!r}, expected {signal!r}')

    # --- the empty-MX_GPIO_Init bug -----------------------------------------
    for pin in OUTPUTS_NEEDING_PARAMS:
        if not kv.get(f'{pin}.GPIOParameters'):
            failures.append(f'{pin}: Signal is set but GPIOParameters is missing - '
                            f'MX_GPIO_Init() will not configure this pin at all')

    # --- reserved QSPI pins, present but not activated -----------------------
    for pin in RESERVED_QSPI:
        if pin not in pins:
            failures.append(f'{pin}: QSPI flash line is not reserved '
                            f'(expected GPIO_Input with no pull)')
        elif pins[pin] != 'GPIO_Input':
            failures.append(f'{pin}: is {pins[pin]!r}, expected GPIO_Input (reserved, not driven)')
    if 'QUADSPI' in text or 'OCTOSPI' in text:
        failures.append('QUADSPI/OCTOSPI is activated; the plan says keep the lines '
                        'reserved and do NOT depend on external flash for first bring-up')

    # --- IP list ------------------------------------------------------------
    ips = []
    for i in range(int(kv.get('Mcu.IPNb', '0') or 0)):
        v = kv.get(f'Mcu.IP{i}')
        if v:
            ips.append(v)
    missing_ips = [x for x in IP_LIST if x not in ips]
    extra_ips = [x for x in ips if x not in IP_LIST]
    if missing_ips:
        failures.append(f'peripherals missing from the IP list: {", ".join(missing_ips)}')
    if extra_ips:
        warnings.append(f'unexpected peripherals present: {", ".join(extra_ips)}')

    # --- structure ----------------------------------------------------------
    pins_nb = int(kv.get('Mcu.PinsNb', '0') or 0)
    declared = len(re.findall(r'^Mcu\.Pin\d+=', text, re.M))
    if pins_nb != declared:
        failures.append(f'Mcu.PinsNb is {pins_nb} but {declared} Mcu.PinN entries exist')

    lines = [l for l in text.splitlines() if '=' in l and not l.startswith('#')]
    keys = [l.split('=', 1)[0] for l in lines]
    dups = sorted({k for k in keys if keys.count(k) > 1})
    if dups:
        failures.append(f'duplicate keys: {", ".join(dups)}')

    # --- derived values -----------------------------------------------------
    stale = []
    check_float(text, kv, failures, notes)
    check_generated(kv, failures, stale, notes)

    if notes:
        say('derived from the raw values:')
        for n in notes:
            say(n)

    if warnings:
        print('warnings:')
        for w in warnings:
            print(f'  ! {w}')

    if stale:
        print(f'\n{len(stale)} item(s) pending regeneration (NOT configuration faults):')
        for f in stale:
            print(f'  ~ {f}')
        print('  -> run CubeMX: close it WITHOUT saving, reopen the .ioc, GENERATE CODE.')
        print('     Do not edit the .ioc for these: the .ioc is already correct.')

    if failures:
        print(f'\n{len(failures)} CONFIGURATION problem(s):')
        for f in failures:
            print(f'  x {f}')
        print('\nThe .ioc itself is wrong. Fix these in CubeMX before generating.')
        return 2

    if stale:
        print('\nThe .ioc is correct; the generated code is behind it.')
        return 1

    print('\n.ioc is consistent with the plan, and the generated code matches it.')
    print('Remember: close CubeMX WITHOUT saving before regenerating, or it will '
          'overwrite this file with its stale in-memory copy.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
