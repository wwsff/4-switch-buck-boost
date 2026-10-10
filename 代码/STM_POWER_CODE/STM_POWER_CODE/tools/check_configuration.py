"""Check the agreed sampling configuration in BOTH CubeMX and generated C.

Read-only: a mismatch fails instead of rewriting generated settings.
"""
from pathlib import Path
import re
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
# Every source is UTF-8; read it as UTF-8 explicitly so the checker does not
# depend on the Windows locale code page (a GBK default made this fail on the
# firmware's own comments instead of on a real configuration mismatch).
def read_source(relative):
    return (root / relative).read_text(encoding='utf-8')

ioc_text = read_source('STM_POWER_CODE.ioc')
ioc = dict(line.replace('\\#', '#').split('=', 1) for line in
           ioc_text.splitlines()
           if '=' in line and not line.startswith('#'))
errors = []

def check(ok, what):
    if not ok:
        errors.append(what)

adc = read_source('Core/Src/adc.c')
for number, channels, times in (
        (1, [6, 7, 8, 9, 3], [47, 47, 24, 92, 92]),
        (2, [5, 11, 17, 13, 3], [47, 47, 24, 92, 92]),
        (3, [12, 1], [247, 247])):
    prefix = f'ADC{number}.'
    function = re.search(
        rf'void MX_ADC{number}_Init\(void\)(.*?)(?=\nvoid |\n/\* ADC|\nvoid HAL_ADC|\Z)', adc, re.S)[1]
    # ADC settings precede MSP implementation, and records are made at HAL calls.
    expected = {'ClockPrescaler': 'ADC_CLOCK_ASYNC_DIV6',
                'NbrOfConversion': str(len(channels)),
                'DiscontinuousConvMode': 'DISABLE' if number == 3 else 'ENABLE',
                'ExternalTrigConv': 'ADC_EXTERNALTRIG_T7_TRGO' if number == 3 else 'ADC_EXTERNALTRIG_HRTIM_TRG1'}
    for name, value in expected.items():
        check(ioc.get(prefix + name) == value, prefix + name + ' in IOC')
        check(re.search(rf'hadc{number}\.Init\.{name}\s*=\s*{value};', function), prefix + name + ' in C')
    keys = sorted((k for k in ioc if k.startswith(prefix + 'Channel-')), key=lambda k: int(k.split('-')[1].split('#')[0]))
    check([ioc[k] for k in keys] == [f'ADC_CHANNEL_{ch}' for ch in channels], prefix + 'regular ranks in IOC')
    for key, timing in zip(keys, times):
        check(ioc.get(key.replace('.Channel-', '.SamplingTime-')) == f'ADC_SAMPLETIME_{timing}CYCLES_5', key + ' sample time')
    state, records = {}, []
    for m in re.finditer(r'sConfig\.(Channel|Rank|SamplingTime)\s*=\s*(\w+);|HAL_ADC_ConfigChannel\(&hadc\d,\s*&sConfig\)', function):
        if m[1]:
            state[m[1]] = m[2]
        else:
            records.append(dict(state))
    check(records == [dict(Channel=f'ADC_CHANNEL_{ch}', Rank=f'ADC_REGULAR_RANK_{rank}',
                           SamplingTime=f'ADC_SAMPLETIME_{t}CYCLES_5') for rank, (ch, t) in enumerate(zip(channels, times), 1)],
          prefix + 'regular sequence in generated C')
    if number < 3:
        for name, value in {'InjectedChannel': f'ADC_CHANNEL_{number}', 'InjectedNbrOfConversion': '1',
                'InjectedSamplingTime': 'ADC_SAMPLETIME_24CYCLES_5',
                'ExternalTrigInjecConv': 'ADC_EXTERNALTRIGINJEC_HRTIM_TRG2',
                'AutoInjectedConv': 'DISABLE', 'QueueInjectedContext': 'DISABLE'}.items():
            check(re.search(rf'sConfigInjected\.{name}\s*=\s*{value};', function), prefix + name)

hrtim = read_source('Core/Src/hrtim.c')
for number, event in ((1, 'HRTIM_ADCTRIGGEREVENT13_TIMERE_CMP3'), (2, 'HRTIM_ADCTRIGGEREVENT24_TIMERE_CMP2')):
    check(ioc.get(f'HRTIM1.ADCTrigger{number}_Source1') == event, f'HRTIM ADC{number} source in IOC')
    check(ioc.get(f'HRTIM1.ORingUpdateSource_ADCTrigger{number}') == 'HRTIM_ADCTRIGGERUPDATE_TIMER_E', f'HRTIM ADC{number} update source')
    check(event in hrtim and f'HRTIM_ADCTRIGGER_{number}, &pADCTriggerCfg' in hrtim, f'HRTIM ADC{number} in C')
check('FaultBlankingConfigAndEnable' not in hrtim, 'Fault blanking still enabled')
# The fault inputs MUST keep a digital filter. The .ioc cannot express this setting (it has no fault
# filter IPParameter), so a CubeMX regeneration silently reverts hrtim.c to HRTIM_FAULTFILTER_NONE -
# and with fFLTS = fHRTIM = 5.442 GHz that validates a fault on a single 0.18 ns sample, which is how
# ALERT_N came to false-trip at 33 V. This check is the only thing standing between the bench and
# that regression, so it pins the exact assignment rather than merely the absence of NONE.
# Every place that can (re-)arm an HRTIM fault interrupt must be gated on the channel actually
# having fault hardware. 0.3.44 armed FLT2/FLT4 for a channel with no power board, whose pins float
# low - so the flag re-asserted as fast as PD_BoardFaultIRQ() cleared it and the 1 kHz re-arm put the
# enable straight back. 986 identical log lines in two minutes starved telemetry and PowerDesk
# dropped the session one second after the handshake. Hand-checking the call sites is what let that
# through, so the invariant is mechanical now.
_board = read_source('App/pd_board.c').splitlines()
for _n, _line in enumerate(_board, 1):
    if 'sCommonRegs.IER |=' in _line:
        _ctx = '\n'.join(_board[max(0, _n - 12):_n + 1])
        check('channel_fault_hw' in _ctx or 'monitored_fault_flags' in _ctx,
              f'App/pd_board.c:{_n} arms an HRTIM fault interrupt with no channel_fault_hw() / '
              f'monitored_fault_flags() guard within 12 lines. An unguarded arm against a channel '
              f'whose fault pins float low re-asserts the flag as fast as it is cleared - that is '
              f'the 0.3.44 telemetry flood')
check('pFaultCfg.Filter = HRTIM_FAULTFILTER_15;' in hrtim,
      'pFaultCfg.Filter must be HRTIM_FAULTFILTER_15 in Core/Src/hrtim.c (47.1 ns). The .ioc has no '
      'filter parameter, so a CubeMX regeneration drops it back to HRTIM_FAULTFILTER_NONE and the '
      'fault inputs then accept a single 0.18 ns sample - see the ALERT_N false-trip at 33 V')
for irq, priority in (('ADC1_2_IRQn', 4), ('HRTIM1_FLT_IRQn', 0)):
    check(ioc.get('NVIC.' + irq, '').split('\\:')[1:3] == [str(priority), '0'], irq + ' in IOC')
    check(f'HAL_NVIC_SetPriority({irq}, {priority}, 0)' in adc + hrtim, irq + ' in C')
check(ioc.get('ProjectManager.StackSize') == '0x1000', 'MSP stack in IOC')
check(ioc.get('FREERTOS.Tasks01', '').split(',')[2:3] == ['1024'], 'task stack in IOC')
check('.stack_size = 1024 * 4' in read_source('Core/Src/app_freertos.c'), 'task stack in C')
board = read_source('App/pd_board.c')
for forbidden in ('HAL_ADC_Init(', 'HAL_ADC_ConfigChannel(', 'HAL_HRTIM_TimeBaseConfig(', 'HAL_NVIC_SetPriority(', 'pc0_poll_probe('):
    check(forbidden not in board, 'application rewrites fixed configuration: ' + forbidden)

# Switching frequency and ADC scheduling (0.3.21). Every number is taken from
# pd_config.h and compared with the .ioc, so a CubeMX regeneration and the
# firmware constants can never drift apart silently. The two Timer E compares
# must stay 50 % into their own PWM period and far enough apart that each ADC
# conversion is finished before the next trigger arrives.
config = (root / 'App/pd_config.h').read_text(encoding='utf-8')

def cfg_int(name):
    """Integer value of a #define, negatives included.

    Reads the value from the SAME line as the directive (a '-' inside a neighbouring
    comment must not be picked up) and returns the value itself, or None when the name is
    absent. It deliberately does NOT fall back to 0: a silent 0 turned the negative
    PD_VOUT_CAL_OFFSET_MV into "nominal", which made the calibration coherence gate report
    the opposite of the truth.
    """
    line = re.search(rf'^[ \t]*#define[ \t]+{name}[ \t]+(-?[0-9][0-9_]*)', config, re.M)
    check(line is not None, f'{name} missing from pd_config.h')
    if line is None:
        return None
    return int(line.group(1).replace('_', ''))

sw = cfg_int('PD_SW_PERIOD')
te = cfg_int('PD_TE_PERIOD')
ratio = cfg_int('PD_TE_PWM_RATIO')
c2 = cfg_int('PD_TE_CMP2')
c3 = cfg_int('PD_TE_CMP3')
fast = cfg_int('PD_FAST_HZ')
bridge = cfg_int('PD_OUT_BRIDGE_CMP')
for timer in ('MASTER', 'TA', 'TB', 'TC', 'TD'):
    check(ioc.get(f'HRTIM1.Periode_{timer}', '').upper() == hex(sw).upper(),
          f'HRTIM1.Periode_{timer} must equal PD_SW_PERIOD (250 kHz master period)')
check(ioc.get('HRTIM1.Periode_TE', '').upper() == hex(te).upper(),
      'HRTIM1.Periode_TE must equal PD_TE_PERIOD (integer multiple of the PWM period)')
check(ioc.get('HRTIM1.CompareValue2-No_output_TE', '').upper() == hex(c2).upper(),
      'Timer E compare 2 (injected IL trigger) must equal PD_TE_CMP2')
check(ioc.get('HRTIM1.CompareValue3-No_output_TE', '').upper() == hex(c3).upper(),
      'Timer E compare 3 (regular-group trigger) must equal PD_TE_CMP3')
check(te == ratio * sw, 'PD_TE_PERIOD must be an integer number of PWM periods')
check(ratio == 2 and fast * ratio == 250000, 'PD_FAST_HZ must be the PWM rate / ratio (integer Hz)')
for dec in (1, 10, 100, 1000):
    check(fast % dec == 0, f'PD_FAST_HZ must be divisible by the host decimation {dec}')
check(c3 == sw // 2 and c2 == te - sw // 2,
      'both Timer E triggers must sit 50 % into their own PWM period')
check(c2 - c3 > cfg_int('PD_TE_REG_RANK_COUNTS') + cfg_int('PD_TE_TRIG_LATENCY_COUNTS'),
      'the injected IL trigger must not wait for a regular conversion')
check(c3 + te - c2 > cfg_int('PD_TE_INJ_RANK_COUNTS') + cfg_int('PD_TE_TRIG_LATENCY_COUNTS'),
      'the regular trigger must not wait for the injected conversion')
check(sw * 3 // 4 == bridge, 'PD_OUT_BRIDGE_CMP must stay 75 % of PD_SW_PERIOD')
check('PD_TE_GAP_REG_TO_INJ' in config and 'PD_TE_GAP_INJ_TO_REG' in config,
      'the compile-time ADC budget gates must stay in pd_config.h')
check('sMasterRegs.MPER == PD_SW_PERIOD' in board, 'pd_board.c must verify the master period at boot')
check('CMP2xR == PD_TE_CMP2' in board, 'pd_board.c must verify the Timer E compares at boot')

# One source of truth for the reported version, and a loud reminder that the
# diagnostic build must never be shipped.
config = (root / 'App/pd_config.h').read_text(encoding='utf-8')
version = re.search(r'#define\s+PD_FW_VERSION\s+"([^"]+)"', config)
check(version is not None, 'PD_FW_VERSION missing from pd_config.h')
for source in ('App/pd_core.c', 'App/pd_board.c'):
    text = (root / source).read_text(encoding='utf-8')
    check('PD_FW_VERSION' in text, source + ' does not use PD_FW_VERSION')
    check('G474-BUCK-' not in text, source + ' hardcodes a version string')
# ... and the reported version must be the release being packaged. Passing the release name
# (--release v0.3.28-linked, which package_release.py does) turns "did you remember to bump
# it?" into a gate: 0.3.28 was first packaged with PD_FW_VERSION still reading 0.3.27-CALFIX,
# so the HEX would have flashed the new modulator and REPORTED the old version - the operator
# would have had no way to tell from telemetry which firmware was running.
_args = sys.argv[1:]
release_name = None
if '--release' in _args:
    _i = _args.index('--release')
    # The name is optional on purpose: tests and dry runs call --release alone, and only the
    # packager knows which directory the artifacts are about to land in.
    if _i + 1 < len(_args) and _args[_i + 1].startswith('v'):
        release_name = _args[_i + 1]
if release_name and version is not None:
    expected = 'G474-BUCK-' + release_name[1:]
    check(version.group(1).upper() == expected.upper(),
          f'PD_FW_VERSION is {version.group(1)!r} but the release being packaged is '
          f'{release_name!r}: expected {expected!r}. Bump PD_FW_VERSION in pd_config.h - a '
          f'HEX that reports the old version cannot be told apart from the old firmware.')
    check(len(version.group(1)) == len(expected),
          f'the version string is {len(version.group(1))} characters, which is not the '
          f'{len(expected)} the INFO reply field was sized for')
diag = re.search(r'#define\s+PD_DIAG_MASK_OC\s+(\d+)', config)
check(diag is not None, 'PD_DIAG_MASK_OC missing from pd_config.h')
if diag and diag.group(1) == '1':
    check(re.search(r'#define\s+PD_DIAG_OC_LOW_SAMPLES\s+\d+', config) is not None,
          'PD_DIAG_OC_LOW_SAMPLES missing while the OC mask is on')
    print('WARNING: PD_DIAG_MASK_OC=1 - HRTIM FLT1 output shutdown is masked; '
          'this build is for bench diagnosis only and must not be released.')
override = re.search(r'#define\s+PD_OVERRIDE_NO_LATCH\s+(\d+)', config)
check(override is not None, 'PD_OVERRIDE_NO_LATCH missing from pd_config.h')
if override and override.group(1) == '1':
    check(re.search(r'#define\s+PD_OVERRIDE_OVP_MV\s+\d+', config) is not None,
          'PD_OVERRIDE_OVP_MV missing while the latch override is on')
    print('WARNING: PD_OVERRIDE_NO_LATCH=1 - latching protections are log-only; '
          'bench override build only, never release it.')
probe = re.search(r'#define\s+PD_RANGE_PROBE\s+(\d+)', config)
check(probe is not None, 'PD_RANGE_PROBE missing from pd_config.h')
if probe and probe.group(1) == '1':
    print('WARNING: PD_RANGE_PROBE=1 - the reachability model does not refuse setpoints and the '
          'runtime range servo is bypassed; a target the input cannot reach ends in REGULATION '
          'FAILED / PD_FAULTS_INTERNAL instead of a pre-emptive refusal. Bench measurement only.')
il_sensor = re.search(r'#define\s+PD_IL_SENSOR_FITTED\s+(\d+)', config)
check(il_sensor is not None, 'PD_IL_SENSOR_FITTED missing from pd_config.h')
if il_sensor and il_sensor.group(1) == '0':
    print('WARNING: PD_IL_SENSOR_FITTED=0 - the TMCS1133 is removed, so CH1 has NO hardware '
          'overcurrent trip (IL, OC_N/FLT1 and ALERT_N/FLT3 all withdrawn); only the IOUT software '
          'limit, OVP, the input window and the supply limit remain. Bench configuration only.')
ch2_fitted = re.search(r'#define\s+PD_CH2_FITTED\s+(\d+)', config)
check(ch2_fitted is not None, 'PD_CH2_FITTED missing from pd_config.h')
if ch2_fitted and ch2_fitted.group(1) == '0':
    print('WARNING: PD_CH2_FITTED=0 - CH2 has no power board, so its fault pins are not polled and '
          'no CH2 fault is latched; bench configuration only and must not be released.')
if '--release' in sys.argv[1:]:
    check(diag is not None and diag.group(1) == '0',
          'RELEASE BUILD MUST HAVE PD_DIAG_MASK_OC = 0 (hardware OC shutdown enabled)')
    check(override is not None and override.group(1) == '0',
          'RELEASE BUILD MUST HAVE PD_OVERRIDE_NO_LATCH = 0 (latching protections active)')
    # Masking one hardware fault input is narrower than the two above, but it is still a protection
    # withdrawn, so it may not ship either. The bench build sets it; a release must not.
    alert = re.search(r'#define\s+PD_MASK_ALERT_NOUT\s+(\d+)', config)
    check(alert is not None and alert.group(1) == '0',
          'RELEASE BUILD MUST HAVE PD_MASK_ALERT_NOUT = 0 (ALERT_N output shutdown enabled): '
          'bench monitor builds use 1 and must be labelled as such')
    # CH2 fault monitoring withdrawn. Same rule: a release ships a board that monitors every channel
    # it has, so a release must declare the second board fitted.
    check(probe is not None and probe.group(1) == '0',
          'RELEASE BUILD MUST HAVE PD_RANGE_PROBE = 0: with it set the predictive range model does '
          'not refuse setpoints and an unreachable target is only caught by REGULATION FAILED')
    check(il_sensor is not None and il_sensor.group(1) == '1',
          'RELEASE BUILD MUST HAVE PD_IL_SENSOR_FITTED = 1: with the TMCS1133 removed CH1 has no '
          'hardware overcurrent trip at all (IL + OC_N/FLT1 + ALERT_N/FLT3 withdrawn)')
    ch2 = re.search(r'#define\s+PD_CH2_FITTED\s+(\d+)', config)
    check(ch2 is not None and ch2.group(1) == '1',
          'RELEASE BUILD MUST HAVE PD_CH2_FITTED = 1 (both channels monitored): this board has no '
          'CH2 power board, which is a bench configuration and not a shippable one')

# HRTIM high-resolution floor: LL_HRTIM_TIM_SetCompare1 documents 0x60 = 96 with
# CKPSC = 0 (HRTIM_PRESCALERRATIO_MUL32 = 0). A value below it may not generate a
# compare event, which leaves the high side set for the whole period.
cmpmin = re.search(r'#define\s+PD_CMP_MIN\s+(\d+)u?', config)
check(cmpmin is not None and int(cmpmin.group(1)) >= 96,
      'PD_CMP_MIN must be >= 96 (0x60) for HRTIM high-resolution mode')
check(re.search(r'#define\s+PD_CMP_FORCE_OFF', config) is None and
      re.search(r'#define\s+PD_CMP_FORCE_OFF', (root / 'App/pd_power.c').read_text(encoding='utf-8')) is None,
      'the old PD_CMP_FORCE_OFF (=1, below the HRTIM minimum) must not come back')
check('HRTIM_PRESCALERRATIO_MUL32' in read_source('Core/Src/hrtim.c'),
      'HRTIM prescaler changed: PD_CMP_MIN was derived for MUL32 (CKPSC = 0)')

# Compare-commit window (0.3.19). The guarantee that TIMER A and TIMER B load
# their CMP1xR on the same master update event rests on these HRTIM settings,
# so they are checked here instead of being assumed.
period = int(re.search(r'#define\s+PD_SW_PERIOD\s+(\d+)u?', config).group(1))
margin = re.search(r'#define\s+PD_CMP_COMMIT_MARGIN\s+(\d+)u?', config)
retries = re.search(r'#define\s+PD_CMP_MAX_RETRIES\s+(\d+)u?', config)
guard = re.search(r'#define\s+PD_CMP_WAIT_GUARD\s+(\d+)u?', config)
check(margin is not None and int(margin.group(1)) >= 40 * 32,
      'PD_CMP_COMMIT_MARGIN must cover >= 40 core cycles (40*32 HRTIM counts)')
check(margin is not None and 0 < int(margin.group(1)) < period,
      'PD_CMP_COMMIT_MARGIN must be smaller than the switching period')
check(retries is not None and 1 <= int(retries.group(1)) <= 1000,
      'PD_CMP_MAX_RETRIES must be a small bounded number')
check(guard is not None and int(guard.group(1)) >= 1000,
      'PD_CMP_WAIT_GUARD must be large enough to cross at least one period')
# The guarantee that TIMER A and TIMER B load their CMP1xR on the SAME master
# update event rests on the generated HRTIM configuration, not on the critical
# section. CubeMX reuses one pTimerCfg struct for several timers, so the
# effective settings are replayed here exactly as the generated code sets them.
hrtim_src = read_source('Core/Src/hrtim.c')
state, timer_cfg = {}, {}
for m in re.finditer(r'pTimerCfg\.(\w+)\s*=\s*(\w+);|'
                     r'HAL_HRTIM_WaveformTimerConfig\(&hhrtim1,\s*(HRTIM_TIMERINDEX_\w+),\s*&pTimerCfg\)',
                     hrtim_src):
    if m[1]:
        state[m[1]] = m[2]
    else:
        timer_cfg[m[3]] = dict(state)
for timer in ('HRTIM_TIMERINDEX_TIMER_A', 'HRTIM_TIMERINDEX_TIMER_B'):
    cfg = timer_cfg.get(timer, {})
    check(cfg.get('PreloadEnable') == 'HRTIM_PRELOAD_ENABLED',
          timer + ': CMP1xR must stay preloaded for the coherent pair commit')
    check(cfg.get('UpdateTrigger') == 'HRTIM_TIMUPDATETRIGGER_MASTER',
          timer + ': preload must transfer on the MASTER event (same boundary)')
    check(cfg.get('ResetUpdate') == 'HRTIM_TIMUPDATEONRESET_DISABLED',
          timer + ': must not update on its own reset (would split the pair)')
for timer in ('TA1TA2', 'TB1TB2'):
    check(ioc.get(f'HRTIM1.UpdateTrigger1-Output_{timer}') == 'HRTIM_TIMUPDATETRIGGER_MASTER',
          f'IOC UpdateTrigger1-Output_{timer} must stay MASTER')
check('PD_CMP_SAFE_WINDOW' not in config and 'PD_CMP_SAFE_WINDOW' not in
      (root / 'App/pd_board.c').read_text(encoding='utf-8'),
      'the old 0.3.18 window constant must not come back')

# Reference tracking / OVP evolution (0.3.20). The three quantities (accepted
# target, executed ramp reference, protection threshold in force) must stay
# separate, the threshold must follow max(target, ramp) and must be capped by a
# ceiling that was NOT raised, and SET_TARGET must refuse anything the power
# stage cannot produce.
power = (root / 'App/pd_power.c').read_text(encoding='utf-8')
core = (root / 'App/pd_core.c').read_text(encoding='utf-8')
power_h = (root / 'App/pd_power.h').read_text(encoding='utf-8')

def number(name):
    m = re.search(rf'#define\s+{name}\s+\(?\s*[0-9_]+', config)
    return int(re.search(r'([0-9_]+)$', m.group(0)).group(1).replace('_', '')) if m else None

buck_vmin, buck_vmax = number('PD_BUCK_VMIN_MV'), number('PD_BUCK_VMAX_MV')
ovp_num, ovp_den, ovp_margin = number('PD_OVP_RATIO_NUM'), number('PD_OVP_RATIO_DEN'), number('PD_OVP_MARGIN_MV')
run_down, stop_down = number('PD_RUN_RAMP_DOWN_MV_MS'), number('PD_STOP_RAMP_MV_MS')
band, band_samples = number('PD_DOWN_BAND_MV'), number('PD_DOWN_BAND_SAMPLES')
settle, timeout_min = number('PD_DOWN_SETTLE_MS'), number('PD_DOWN_TIMEOUT_MIN_MS')
check(None not in (buck_vmin, buck_vmax, ovp_num, ovp_den, ovp_margin, run_down, stop_down,
                   band, band_samples, settle, timeout_min),
      '0.3.20 reference/down-transition constants missing from pd_config.h')
# The ceiling must stay DERIVED from the declared maximum output and the ratio.
# 0.3.22 raises the declared range (15 V -> 18 V), so the ceiling rises with it
# (17250 -> 20700 mV) BY DESIGN: this check pins the derivation, and the release
# notes must record the raise instead of repeating the old wording.
ceiling = re.search(r'#define\s+PD_OVP_ABS_MAX_MV\s+(.+)', config)
expected_ceiling = '(PD_BUCK_VMAX_MV * PD_OVP_RATIO_NUM / PD_OVP_RATIO_DEN)'
check(ceiling is not None and ceiling.group(1).strip() == expected_ceiling,
      'PD_OVP_ABS_MAX_MV must stay PD_BUCK_VMAX_MV * PD_OVP_RATIO_NUM / PD_OVP_RATIO_DEN '
      '(derived from the declared range, never an independent constant)')
if None not in (buck_vmax, ovp_num, ovp_den, ovp_margin):
    check(buck_vmax * ovp_num // ovp_den >= buck_vmax + ovp_margin,
          'the OVP ceiling must stay above the maximum specified output + margin')
# The threshold must come from the reference layer, not from the raw target.
check('c->vref_mv * PD_OVP_RATIO_NUM' not in power,
      'pd_power.c still derives the OVP threshold from the raw user target')
check('ovp_for(' in power and 'PD_OVP_ABS_MAX_MV' in power and 'p.ovp_limit_mv' in power,
      'pd_power.c must compute the threshold with ovp_for() and cap it at the absolute ceiling')
check('ramp_int()' in power, 'the threshold must be derived from the executed ramp reference')
# Command-driven ramp only: a measured output may seed it once, clamped.
check('p.ramp_mv = p.vout_f' not in power,
      'the reference must not be assigned straight from the measured output')
check('p.ramp_mv = seed;' in power and 'seed > (float)p.target_mv' in power,
      'the precharge seed must stay clamped to the accepted target')
# Two independent down rates.
check('PD_RUN_RAMP_DOWN_MV_MS' in power and 'PD_STOP_RAMP_MV_MS' in power,
      'the regulation and shutdown down-rates must stay separate parameters')
check(run_down is not None and run_down > 0 and stop_down is not None and stop_down > 0,
      'both down-rates must be positive')
# Down-transition supervision bounds.
check(band is not None and 0 < band < (buck_vmin if buck_vmin else 2000),
      'PD_DOWN_BAND_MV must be positive and smaller than the minimum target')
check(band_samples is not None and 1 <= band_samples <= 255,
      'PD_DOWN_BAND_SAMPLES must fit the 8-bit consecutive-sample counter')
check(settle is not None and settle > 0, 'PD_DOWN_SETTLE_MS must be positive')
check(timeout_min is not None and settle is not None and timeout_min >= settle,
      'PD_DOWN_TIMEOUT_MIN_MS must be at least the settling allowance')
check('DOWN timeout' in power and 'PD_DOWN_BAND_SAMPLES' in power and 'down_budget_ms(' in power,
      'the down-transition supervisor must exist (budget, band, timeout stop)')
# SET_TARGET must use the power-stage window, not the generic protocol window.
check('target_vmax_mv(ch)' in core and 'target_vmin_mv(ch)' in core,
      'SET_TARGET must be limited to the window the power stage can produce')
check('v > PD_VMAX_MV' not in core,
      'SET_TARGET still compares against the generic 30 V protocol window')

# ===================== 0.3.22: 2..18 V BUCK18 candidate ====================
# The qualified output range moved from 15 V to 18 V on purpose, so the OVP
# ceiling derived from it moves from 17250 mV to 20700 mV. That is a RAISE and
# the release notes must say so; the check below only pins the DERIVATION, it no
# longer forbids the value from changing with the specified range.
a_max = cfg_int('PD_CMP_A_MAX')
dt_counts = cfg_int('PD_CMP_DT_COUNTS')
model_dt = cfg_int('PD_MODEL_DT_COUNTS')
mid_max = cfg_int('PD_CMP_MID_CHANNEL_MAX')
edge_on = cfg_int('PD_EDGE_ON_NS')
edge_off = cfg_int('PD_EDGE_OFF_NS')
vscale = cfg_int('PD_VOLTAGE_SCALE_REF_MV')
vref_cur = cfg_int('PD_VREF_MV')
range_hold = cfg_int('PD_VIN_RANGE_HOLD_MS')
check(int(float(re.search(r'#define\s+PD_DUTY_MAX\s+([0-9.]+)', config).group(1)) * sw + 0.5) == a_max,
      'PD_CMP_A_MAX must be the integer form of PD_DUTY_MAX * PD_SW_PERIOD')
check(a_max < bridge, 'the A-bridge ceiling must stay below the output-bridge duty')
check(a_max + dt_counts < bridge,
      'A-bridge ceiling + dead time must leave an A/B edge separation')
check(model_dt == dt_counts + round((edge_on - edge_off) * 5.44),
      'PD_MODEL_DT_COUNTS must be the configured dead time plus the measured gate-edge skew')
# The configured dead time must be the ONE the registers actually get: 26 fHRTIM clocks at
# DIV1 = 832 core counts (32 counts per clock), so the CubeMX source of truth (.ioc), the
# generated code (hrtim.c) and the model constant have to agree. 0.3.30 changed all three
# together from 26 clocks / 832 counts to 20 clocks / 640 counts; without this check a
# mismatch would run one dead time while every model, gate and log claimed another - the
# "old firmware with a new CubeMX" failure this project has already been bitten by.
ioc_rise = re.findall(r'HRTIM1\.RisingValue-Output_(T[ABCD])1T[ABCD]2=(0x[0-9A-Fa-f]+)', ioc_text)
ioc_fall = re.findall(r'HRTIM1\.FallingValue-Output_(T[ABCD])1T[ABCD]2=(0x[0-9A-Fa-f]+)', ioc_text)
hrtim_src = read_source('Core/Src/hrtim.c')
clocks = {int(v, 16) for _, v in ioc_rise} | {int(v, 16) for _, v in ioc_fall} | \
         {int(v, 16) for v in
          re.findall(r'pDeadTimeCfg\.RisingValue\s*=\s*(0x[0-9A-Fa-f]+);', hrtim_src)} | \
         {int(v, 16) for v in
          re.findall(r'pDeadTimeCfg\.FallingValue\s*=\s*(0x[0-9A-Fa-f]+);', hrtim_src)}
check(len(ioc_rise) == 4 and len(ioc_fall) == 4,
      f'the .ioc must carry a rising and a falling dead time for all four timers '
      f'(found {len(ioc_rise)} rising, {len(ioc_fall)} falling)')
check(len(clocks) == 1,
      f'the dead time must be the same for all four timers in both files; the sources say '
      f'{[hex(c) for c in sorted(clocks)]}')
only = next(iter(clocks)) if len(clocks) == 1 else 0
check(only * 32 == dt_counts,
      f'the registers get {only} fHRTIM clocks = {only * 32} core counts, but '
      f'PD_CMP_DT_COUNTS says {dt_counts}: the firmware would model a dead time the hardware '
      f'does not have')
check(mid_max < a_max, 'PD_CMP_MID_CHANNEL_MAX must sit below the A-bridge ceiling')
check('PD_CMP_A_MAX' in power and 'PD_MODEL_DT_COUNTS' in power,
      'pd_power.c must use the integer ceiling and the measured-edge model')
check('PD_PowerAllowedVmaxMv' in power and 'PD_PowerAllowedVmaxMv' in core,
      'ONE available-range function must gate START and the runtime SET_TARGET check')
check('vmax = (c->vin_mv * 9) / 10' not in power,
      'the old fixed 0.9*VIN headroom rule must be gone (it blocked 18 V at 19.8 V in)')
check('PD_VIN_RANGE_HOLD_MS' in power and range_hold > 0,
      'the input-sag supervisor and its hold time must exist')
check('PD_FAULTS_INPUT' in power, 'an input sag must latch the INPUT fault, not a generic one')
# ---- dual-bridge distributor (0.3.28) ------------------------------------------------------
# The five bound families of the plan are deliberately separate; what the gate enforces is that
# they stay CONSISTENT with each other and with the topology, in whichever mode is selected.
modulation = (root / 'App/pd_modulation.c').read_text(encoding='utf-8')
modulation_h = (root / 'App/pd_modulation.h').read_text(encoding='utf-8')
variants = (root / 'tools/compile_variants.py').read_text(encoding='utf-8')
uvprojx = (root / 'MDK-ARM/STM_POWER_CODE.uvprojx').read_text(encoding='utf-8')
sw = cfg_int('PD_SW_PERIOD')
q_max_c = cfg_int('PD_MOD_Q_MAX_COUNTS')
q_min_c = cfg_int('PD_MOD_Q_MIN_COUNTS')
q_low_c = cfg_int('PD_MOD_Q_LOW_COUNTS')
q_high_c = cfg_int('PD_MOD_Q_HIGH_COUNTS')
b_min_c = cfg_int('PD_MOD_B_MIN_COUNTS')
a_min_c = cfg_int('PD_MOD_A_MIN_COUNTS')
check(q_min_c < q_low_c < q_high_c < q_max_c,
      'the q range and the zone boundaries must be strictly ordered')
# The q ceiling is set by the B floor through q = 0.75*A_max/B, so assert THAT relation rather
# than a fixed duty: 0.3.28 shipped 0.875 (B 60 %), 0.3.33 opens 0.9545 (B 55 %).
check(abs(q_max_c * b_min_c - 3 * a_max * sw // 4) <= sw,
      f'PD_MOD_Q_MAX_COUNTS ({q_max_c}) and PD_MOD_B_MIN_COUNTS ({b_min_c}) must describe the '
      f'same ceiling: their product should be 0.75 * PD_CMP_A_MAX * PD_SW_PERIOD '
      f'({3 * a_max * sw // 4}) within one period count')
check(q_low_c * 20 == sw * 13, 'PD_MOD_Q_LOW_COUNTS must be 0.65 of PD_SW_PERIOD')
check(q_high_c * 4 == sw * 3, 'PD_MOD_Q_HIGH_COUNTS must be 0.75 of PD_SW_PERIOD')
check((q_high_c - q_low_c) * 10 == sw,
      'the transition zone must be 0.10 of the period: the DA polynomial uses it as its normaliser')
check(a_min_c <= q_min_c,
      'the A floor must not sit above the q floor, or the A clamp would command MORE than asked')
check(b_min_c < bridge, 'the experimental B floor must sit below the B ceiling')
check(q_max_c * b_min_c * 2 <= sw * sw * 3,
      'the released q range is not executable with B at its floor (q_max*B_min > 0.525): the '
      'distributor would have to clamp B, and inventing a lower floor is what the plan forbids')
check(a_max < q_max_c, 'the A ceiling must stay below the q ceiling, or linked mode is a no-op')
for name in ('PD_MOD_LINKED', 'PD_MOD_Q_MAX_COUNTS', 'PD_MOD_B_MIN_COUNTS'):
    check(name in config, f'{name} must be a configuration macro')
check('PD_ModDistribute' in modulation and 'PD_ModAppliedQ' in modulation,
      'pd_modulation.c must own the q -> (A, B) mapping and its inverse')
check('PD_MOD_ZONE_NONE' in modulation_h,
      'the distributor must report a NONE zone, not only the three regions')
check('pd_modulation.c' in uvprojx and 'pd_modulation.c' in variants,
      'the distributor must be listed in the Keil project AND in the variant compiler, '
      'otherwise one build silently keeps the old modulator')
check('legacy-modulator' in variants and 'PD_MOD_LINKED=0' in variants,
      'the legacy modulator must stay COMPILED as the rollback: a fallback that no longer '
      'builds is not a fallback')
# ADC edge-proximity watch (0.3.32): the predicate is only meaningful if the sampling instant
# it compares against is the one Timer E really produces, and if the window is small enough to
# mean something. Assert both, and assert that the flag is a REPORT (nothing reads it back).
watch = cfg_int('PD_ADC_EDGE_WATCH_COUNTS')
watch_ms = cfg_int('PD_ADC_EDGE_WATCH_MS')
te_cmp3 = cfg_int('PD_TE_CMP3')
sw = cfg_int('PD_SW_PERIOD')
check(te_cmp3 == sw // 2,
      f'PD_TE_CMP3 ({te_cmp3}) must be half a switching period ({sw // 2}): the ADC edge watch '
      f'compares the committed compares against exactly that instant')
check(0 < watch <= 1200,
      f'the edge-proximity window ({watch} counts) must be a real window: above zero and at most '
      f'~220 ns, or it would fire everywhere and stop meaning anything')
check(watch_ms >= 1000,
      'the edge-proximity line must be rate limited to at least a second: it shares the 1 Hz '
      'frame budget with six other frames')
check('PD_PowerAdcEdgeNear' in power and 'PD_PowerAdcEdgeNear' in power_h,
      'the edge-proximity predicate must live in pd_power.c so the host suite can test it')
diag_c = read_source('App/pd_diag.c')
diag_h = read_source('App/pd_diag.h')
check('PD_DiagSamplingLine' in diag_c and 'PD_DiagSamplingLine' in diag_h,
      'the edge-proximity line must be produced by App/pd_diag.c like every other log line')
check('PD_PowerAdcEdgeNear' in board,
      'pd_board.c must report the condition; without a caller the predicate is dead code')
check('PD_ModDistribute' in power and 'mod_limit' in power,
      'the saturation reason must come from the distributor limit bits (0.3.32), not from a '
      'difference between delivered and requested q')
check('PD_CTRL_LIMIT_CNT' in power,
      'the regulation-failure backstop must judge the END OF RANGE in the active coordinate')
check('PD_MOD_Q_MIN_COUNTS' in power,
      'the precharge hand-off must ask the distributor for the smallest executable q')
check('commit_duties' in power,
      'the legacy register path must stay in the tree for the fallback build')
# Voltage calibration is per-channel on purpose.
check(vscale > 0 and vscale < vref_cur,
      'PD_VOLTAGE_SCALE_REF_MV must be the measured reference for VIN/VOUT only')
check('PD_ConvertVinMv' in board and 'PD_ConvertVoutMv' in board,
      'pd_board.c must use the shared voltage conversion for VIN/VOUT')
check('PD_VoltageScale' not in config, 'unexpected global voltage-scale override')
check('current(raw, 5)' in board or 'PD_ZeroCurrent(1' in board,
      'current channels must keep their own conversion basis')
# The declared output range must be reachable inside the input window. Two
# independent guards, in this order of authority:
#   1. the MEASURED transfer line (0.3.23) minus its margin - what the bench
#      actually delivered, expressed in the ACTIVE modulation coordinate:
#      VOUT_max = VIN * PD_RANGE_GAIN_ACTIVE_NUM/PD_RANGE_GAIN_DEN - PD_RANGE_MARGIN_MV.
#      This is the limit the operator is held to.
#   2. the topology ceiling VIN*(ceiling-Ddt)/(B-Ddt) - an UPPER BOUND only, since
#      the real stage loses ~8.3 % to unmodelled timing. Keeping it as a check
#      makes sure the measured line is never more permissive than physics allows.
# Declaring 2..24 V without widening the input window or scheduling d_B fails
# here instead of on the bench.
#
# 0.3.28 made both the gain and the ceiling mode-dependent: with PD_MOD_LINKED the loop
# drives q (ceiling 0.875) instead of the A bridge alone (ceiling 0.70), so the same
# measurement is 1068/1000 rather than 856/1000. The two are asserted to be one
# measurement seen from two coordinates, because a build that promised the linked range
# while running the old modulator is exactly the 0.3.22 failure.
gain_num = cfg_int('PD_RANGE_GAIN_NUM')
gain_den = cfg_int('PD_RANGE_GAIN_DEN')
gain_linked = cfg_int('PD_RANGE_GAIN_LINKED_NUM')
gain_active = cfg_int('PD_RANGE_GAIN_ACTIVE_NUM')
range_margin = cfg_int('PD_RANGE_MARGIN_MV')
vin_max = cfg_int('PD_VIN_MAX_MV')
linked = cfg_int('PD_MOD_LINKED')
q_max = cfg_int('PD_MOD_Q_MAX_COUNTS')
check(linked in (0, 1), 'PD_MOD_LINKED must be 0 (legacy modulator) or 1 (dual-bridge linked)')
ceiling = q_max if linked else a_max
check(gain_active == (gain_linked if linked else gain_num),
      f'PD_RANGE_GAIN_ACTIVE_NUM ({gain_active}) does not match PD_MOD_LINKED ({linked}): the range '
      f'model and the modulator would disagree about the ceiling')
# One measurement, two coordinates: gain_linked = gain_legacy * (q_max/A_max). 4 per mille of
# slack covers the choice of the lower measured k_v (0.916 vs 0.917); anything more is a typo.
scaled_legacy = round(gain_num * q_max / a_max)
# The linked coefficient is either the legacy fit scaled to the new ceiling (when it is an
# extrapolation) or a MEASURED value at that ceiling - and the measurement may only be MORE
# conservative, never more optimistic: k_v falls with duty, and an over-optimistic range model is
# what let 0.3.22 promise 18 V it could not hold. 0.3.34 measured 1151 against a scaled 1167.
check(gain_linked <= scaled_legacy + 4,
      f'PD_RANGE_GAIN_LINKED_NUM ({gain_linked}) is more optimistic than the legacy fit scaled to '
      f'the q ceiling ({scaled_legacy} = {gain_num} * {q_max}/{a_max}): the range model would '
      f'promise output the measured line does not support')
check(gain_linked >= scaled_legacy * 9 // 10,
      f'PD_RANGE_GAIN_LINKED_NUM ({gain_linked}) is more than 10 % below the scaled legacy fit '
      f'({scaled_legacy}): that is a different transfer line, not a re-expression - re-fit it and '
      f'say so in the release notes')
reachable = vin_max * (ceiling - model_dt) // (bridge - model_dt)
measured = vin_max * gain_active // gain_den - range_margin
measured = min(measured, reachable)
check(measured >= buck_vmax,
      f'declared max output {buck_vmax} mV is not reachable at VIN={vin_max} mV under the MEASURED '
      f'transfer line (limit {measured} mV): raise the input window, widen the duty ceiling or '
      f'schedule the output-bridge duty')
# The check above uses vin_max, the OPTIMISTIC end of the input window. The declared range is
# advertised over the whole window, so print what the 19 V floor supports as well: a declared max can
# pass the gate and still be refused at the bottom of the window it is declared over. This reports,
# it does not fail - choosing which end the declaration must hold at is the operator's call, and the
# 2026-09-19 measurement is what made the distinction worth seeing.
_vin_min = number('PD_VIN_MIN_MV')
if probe is not None and probe.group(1) == '1':
    # Same trap as everywhere else in this project: reporting a figure that the running build no
    # longer acts on. With the probe set the model does not answer, so quoting what it WOULD have
    # answered would describe a refusal that cannot happen.
    print(f'declared range {buck_vmin}..{buck_vmax} mV over an input window of {_vin_min}..{vin_max} mV: '
          f'PD_RANGE_PROBE=1, so the declared range is what answers at BOTH ends - the measured line '
          f'is not consulted and the low-end figure below does not apply')
elif None not in (_vin_min, gain_active, gain_den, range_margin, reachable):
    _low = min(_vin_min * gain_active // gain_den - range_margin, reachable)
    print(f'declared range {buck_vmin}..{buck_vmax} mV over an input window of {_vin_min}..{vin_max} mV: '
          f'at the LOW end the measured line supports {_low} mV'
          + ('' if _low >= buck_vmax else
             f' - i.e. the top {buck_vmax - _low} mV of the declared range is refused at '
             f'{_vin_min} mV input'))
# The measured line may not exceed the topology ceiling. It is clamped before the comparison
# because the measured line is allowed to reach past PD_BUCK_VMAX_MV (it is capped by it).
check(vin_max * gain_active // gain_den - range_margin <= reachable,
      f'the measured range may not exceed the topology ceiling ({reachable} mV) - '
      f'the 0.3.23 gain/offset constants look wrong')
check(gain_active > gain_den * 3 // 4,
      'PD_RANGE_GAIN_ACTIVE_NUM/PD_RANGE_GAIN_DEN must be the measured gain (above 0.75 of ideal)')
check(gain_active <= ceiling * gain_den // bridge,
      f'the active gain may not exceed the ideal transfer at the active ceiling '
      f'({ceiling}/{bridge} = {ceiling * gain_den // bridge} per mille)')
for name in ('PD_RANGE_GAIN_ACTIVE_NUM', 'PD_RANGE_GAIN_DEN', 'PD_RANGE_MARGIN_MV'):
    check(name in config and name in power,
          f'{name} must gate the available-range function in pd_power.c')
check('PD_PowerRequiredVinMv' in power and 'PD_PowerRequiredVinMv' in power_h,
      'the inverse of the measured line must exist so START/telemetry can report the required input')
# Voltage calibration has exactly TWO coherent states, and everything else is a
# build error:
#   NOT CALIBRATED  gain 1000/1000, offset 0, PD_CALIBRATED = 0   (0.3.23 ships here)
#   CALIBRATED      measured coefficients, PD_CALIBRATED = 1      (the revision)
# The earlier version of this check demanded the NOT CALIBRATED state outright, which
# would have refused to package the calibrated revision the same file documents - a
# dry run of the three-point procedure caught that before it cost bench time. What
# must never ship is the MIXED state: coefficients applied while the flag still says
# "nominal" (or the flag set with nominal coefficients), because then every document
# and every telemetry line lies about which numbers are in force.
cal_gain = cfg_int('PD_VOUT_CAL_GAIN_NUM')
cal_den = cfg_int('PD_VOUT_CAL_GAIN_DEN')
cal_offset = cfg_int('PD_VOUT_CAL_OFFSET_MV')
vin_offset = cfg_int('PD_VIN_CAL_OFFSET_MV')
calibrated = cfg_int('PD_CALIBRATED')
check(None not in (cal_gain, cal_den, cal_offset, vin_offset, calibrated),
      'a calibration macro could not be read - including a negative offset (see cfg_int)')
nominal = (cal_gain == 1000 and cal_den == 1000 and cal_offset == 0 and vin_offset == 0)
check(cal_den > 0 and cal_gain > 0, 'the calibration gain must be a positive ratio')
check(calibrated in (0, 1), 'PD_CALIBRATED must be 0 (nominal) or 1 (three-point fit applied)')
if calibrated == 0:
    check(nominal,
          'PD_CALIBRATED is 0 but a calibration coefficient is not nominal '
          f'(gain {cal_gain}/{cal_den}, offset {cal_offset}): either finish the '
          'three-point fit and set PD_CALIBRATED = 1, or restore the nominal values')
else:
    check(not nominal,
          'PD_CALIBRATED is 1 but every coefficient is still nominal: the '
          'three-point fit has not actually been applied')
    check(cal_gain * 1000 // cal_den >= 900 and cal_gain * 1000 // cal_den <= 1100,
          'a gain outside 0.90..1.10 is not a calibration, it is a wiring fault - '
          'check the divider and the reference before trusting these numbers')
# And the ceiling must still be derived, with a margin above the declared range.
check(buck_vmax * ovp_num // ovp_den == buck_vmax * ovp_num // ovp_den and
      buck_vmax * ovp_num // ovp_den >= buck_vmax + ovp_margin,
      'the OVP ceiling must stay above the declared maximum output + margin')
# ...and it must stay BELOW the components that see VOUT. The declared range and the OVP
# ceiling are one knob (OVP = 1.15 x declared), so raising the range raises the protection
# threshold too; without this gate nothing tied that number to the hardware. The rating comes
# from the power-board netlist:
#   U5/U6/U7  220 uF ERJ1KM221W20OT   rated 100 V
#   C25-C28   10 uF FS32X106K101EGG  rated 100 V (1210 X7R)
#   Q1-Q4     CSD19531Q5A             Vdss 100 V
# 85 % is the usual derating headroom for a protection level.
cap_rated = cfg_int('PD_OUT_CAP_RATED_MV')
check(cap_rated is not None and cap_rated > 0,
      'PD_OUT_CAP_RATED_MV must record the rated voltage of the components across VOUT')
if cap_rated:
    ovp_abs_mv = buck_vmax * ovp_num // ovp_den
    check(ovp_abs_mv * 100 <= cap_rated * 85,
          f'the OVP ceiling ({ovp_abs_mv} mV) must stay below 85 % of the rated voltage of the '
          f'output components ({cap_rated} mV, netlist): a declared range of {buck_vmax} mV '
          f'needs either lower-voltage protection or better parts')
    check(ovp_abs_mv > buck_vmax,
          'the OVP ceiling must stay above the declared maximum output')

# Every log line must fit the 96-byte text field of the protocol frame, or it is
# truncated silently on the wire - exactly what happened to the 0.3.22 code-17 and
# code-9 lines, which is why the operator's diagnostic log contained no usable
# range/saturation data. tools/check_log_lengths.py computes the worst case for
# every formatted line from the source, so this cannot rot the way a hand count
# does.
log_check = subprocess.run([sys.executable, str(root / 'tools/check_log_lengths.py')],
                           capture_output=True, text=True)
check(log_check.returncode == 0,
      'a log line can exceed the 96-byte field:\n' + log_check.stdout.strip())

# The voltage calibration and the range model are COUPLED: PD_RANGE_GAIN_NUM/DEN was
# fitted from uncalibrated VOUT telemetry, so enabling the calibration shifts the
# transfer line by the same factor and keeping the old ratio would apply the correction
# twice (180-360 mV at 18 V). pd_power.c has the matching compile-time precondition;
# this reports it in the same words while the configuration is still being edited.
range_measured_cal = cfg_int('PD_RANGE_MEASURED_CAL')
check(range_measured_cal in (0, 1, 2),
      'PD_RANGE_MEASURED_CAL must be 0 (range ratio fitted from uncalibrated VOUT), '
      '1 (re-measured with the calibrated build) or 2 (recomputed from the fit through '
      'the calibration, effect quantified as negligible)')
if cfg_int('PD_CALIBRATED') == 1:
    check(range_measured_cal >= 1,
          'PD_CALIBRATED = 1 with PD_RANGE_MEASURED_CAL = 0: PD_RANGE_GAIN_NUM/DEN was '
          'fitted from UNCALIBRATED VOUT telemetry and would be applied twice. Either '
          're-measure the transfer line with the calibrated firmware, or recompute it '
          'from the fit (tools/apply_voltage_calibration.py prints the effect) and set '
          'PD_RANGE_MEASURED_CAL = 2')

# Both existing regular NTC ranks must agree with regenerated GPIO labels.
for pin, signal, label in [('PC3', 'ADC1_IN9', 'CH1_TMOS'), ('PA2', 'ADC1_IN3', 'CH1_TL'),
                            ('PA5', 'ADC2_IN13', 'CH2_TMOS'), ('PA6', 'ADC2_IN3', 'CH2_TL')]:
    check(ioc.get(pin + '.Signal') == signal and ioc.get(pin + '.GPIO_Label') == label,
          f'{pin}: NTC signal/label disagrees with the netlist mapping')

if errors:
    raise SystemExit('\n'.join(errors))
print('CubeMX / generated C sampling, triggers, priorities and stacks agree; no application overrides.')
print('every formatted log line fits the 96-byte field (worst case computed, not counted)')
