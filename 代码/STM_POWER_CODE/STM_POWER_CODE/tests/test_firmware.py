"""Exercise the ACTUAL firmware C codec/command/PID code using PowerDesk codec."""
import ctypes as C
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
CONFIG_TEXT=(ROOT/'App/pd_config.h').read_text(encoding='utf-8')
PC=ROOT.parents[1]/'PowerDesk'
sys.path.insert(0,str(PC))
from protocol import Frame,Parser,KIND,CMD,SCHEMA_CRC,ERR,pack,unpack,LAYOUTS
from device import Device

def firmware_version():
    """Read PD_FW_VERSION from the single header that defines it."""
    text=(ROOT/'App/pd_config.h').read_text(encoding='utf-8')
    m=re.search(r'#define\s+PD_FW_VERSION\s+"([^"]+)"',text)
    assert m,'PD_FW_VERSION not found in pd_config.h'
    return m.group(1)

FW_VERSION=firmware_version()

def config_int(name):
    """Integer value of a #define, negatives included.

    Same-line match (a '-' in a comment must not be picked up) and no silent 0 fallback:
    the calibration offset is negative, and the old r'(\d+)' regex could not see it.
    """
    line=re.search(rf'^[ \t]*#define[ \t]+{name}[ \t]+(-?[0-9][0-9_]*)', CONFIG_TEXT, re.M)
    if line is None:
        raise AssertionError(f'{name} not found in pd_config.h')
    return int(line.group(1).replace('_',''))

def config_float(name):
    text=(ROOT/'App/pd_config.h').read_text(encoding='utf-8')
    m=re.search(rf'#define\s+{name}\s+([0-9.]+)f?',text)
    assert m,f'{name} not found in pd_config.h'
    return float(m.group(1))

STALE_MS=config_int('PD_LOOP_STALE_MS')
OVP_HOLD_MS=config_int('PD_OVP_HOLD_MS')
INPUT_HOLD_MS=config_int('PD_INPUT_HOLD_MS')
INPUT_OC_MS=config_int('PD_INPUT_OC_MS')
PD_CMP_COMMIT_MARGIN=config_int('PD_CMP_COMMIT_MARGIN')
PD_CMP_MAX_RETRIES=config_int('PD_CMP_MAX_RETRIES')
PD_CMP_LATEST=config_int('PD_SW_PERIOD')-PD_CMP_COMMIT_MARGIN
# Reference/protection-tracking constants (0.3.20).
BUCK_VMIN=config_int('PD_BUCK_VMIN_MV')
BUCK_VMAX=config_int('PD_BUCK_VMAX_MV')
OVP_HOLD_MS=config_int('PD_OVP_HOLD_MS')
OVP_MARGIN=config_int('PD_OVP_MARGIN_MV')
OVP_NUM=config_int('PD_OVP_RATIO_NUM')
OVP_DEN=config_int('PD_OVP_RATIO_DEN')
OVP_ABS_MAX=BUCK_VMAX*OVP_NUM//OVP_DEN
PD_OUT_CAP_RATED_MV=config_int('PD_OUT_CAP_RATED_MV')
DOWN_BAND=config_int('PD_DOWN_BAND_MV')
DOWN_BAND_SAMPLES=config_int('PD_DOWN_BAND_SAMPLES')
DOWN_SETTLE_MS=config_int('PD_DOWN_SETTLE_MS')
DOWN_TIMEOUT_MIN_MS=config_int('PD_DOWN_TIMEOUT_MIN_MS')
RUN_RAMP_DOWN=config_int('PD_RUN_RAMP_DOWN_MV_MS')
# Modulation / range constants: 0.3.22 added the 2..18 V candidate, 0.3.23
# replaced the topology range model with the MEASURED transfer line.
PER=config_int('PD_SW_PERIOD')
D_B=config_float('PD_OUT_BRIDGE_DUTY')                       # output-bridge duty, fixed
B_CMP=config_int('PD_OUT_BRIDGE_CMP')
A_MAX=config_int('PD_CMP_A_MAX')                             # integer A-bridge ceiling
DUTY_MAX=config_float('PD_DUTY_MAX')
DT_COUNTS=config_int('PD_CMP_DT_COUNTS')                     # configured dead time (register model)
MODEL_DT_COUNTS=config_int('PD_MODEL_DT_COUNTS')             # dead time + measured gate skew
MID_CHANNEL_MAX=config_int('PD_CMP_MID_CHANNEL_MAX')
D_MODEL=MODEL_DT_COUNTS/PER
EDGE_ON_NS=config_int('PD_EDGE_ON_NS')
EDGE_OFF_NS=config_int('PD_EDGE_OFF_NS')
D_MODEL=config_int("PD_MODEL_DT_COUNTS")/float(config_int("PD_SW_PERIOD"))
PD_CMP_MIN_FOR_TEST=config_int("PD_CMP_MIN")
VREF_VOLT=config_int('PD_VOLTAGE_SCALE_REF_MV')
VREF_CURRENT=config_int('PD_VREF_MV')
VIN_RANGE_HOLD_MS=config_int('PD_VIN_RANGE_HOLD_MS')
VIN_MIN_MV=config_int('PD_VIN_MIN_MV')
VIN_MAX_MV=config_int('PD_VIN_MAX_MV')
RANGE_RESERVE=config_int('PD_RANGE_RESERVE_COUNTS')
RANGE_LOSS_MV=config_int('PD_RANGE_LOSS_MARGIN_MV')
GAIN_NUM=config_int('PD_RANGE_GAIN_ACTIVE_NUM')              # measured VOUT_max/VIN, ACTIVE coord
GAIN_DEN=config_int('PD_RANGE_GAIN_DEN')
# The range model now carries the ceiling of the coordinate the loop drives (0.3.28): 856 for
# the legacy A-only modulator (ceiling 0.70) and 1068 for the linked one (ceiling q = 0.875).
# Both are the SAME measurement re-expressed, and the test suite must follow the build it is
# testing - the host DLL is linked, and PD_MOD_LINKED is 1 in pd_config.h.
LINKED=config_int('PD_MOD_LINKED')==1
GAIN_LEGACY=config_int('PD_RANGE_GAIN_NUM')
GAIN_LINKED=config_int('PD_RANGE_GAIN_LINKED_NUM')
RANGE_CEILING=config_int('PD_MOD_Q_MAX_COUNTS') if LINKED else config_int('PD_CMP_A_MAX')
Q_MAX_CMP=config_int('PD_MOD_Q_MAX_COUNTS')
Q_HIGH_CMP=config_int('PD_MOD_Q_HIGH_COUNTS')
Q_LOW_CMP=config_int('PD_MOD_Q_LOW_COUNTS')
RANGE_MARGIN_MV=config_int('PD_RANGE_MARGIN_MV')

def structural_vmax(vin_mv):
    """The topology/duty-ceiling model: an UPPER BOUND, not a promise. It is what
    0.3.22 used and overpredicted the bench by ~8 %."""
    span=B_CMP-MODEL_DT_COUNTS
    return (vin_mv*(RANGE_CEILING-RANGE_RESERVE-MODEL_DT_COUNTS))//span-RANGE_LOSS_MV

def allowed_vmax(vin_mv):
    """Python model of PD_PowerAllowedVmaxMv() (0.3.23, measured line + margin)."""
    if vin_mv<VIN_MIN_MV or vin_mv>VIN_MAX_MV:
        return 0
    measured=vin_mv*GAIN_NUM//GAIN_DEN-RANGE_MARGIN_MV
    measured=min(measured,structural_vmax(vin_mv))            # never above the topology bound
    if measured<=0:
        return 0
    return min(BUCK_VMAX,measured)

def required_vin(vout_mv):
    """Smallest VIN at which that target becomes settable (inverse of the line)."""
    return -(-(vout_mv+RANGE_MARGIN_MV)*GAIN_DEN//GAIN_NUM)

def needed_da_counts(vout_mv,vin_mv):
    """Commanded A-bridge compare value the TOPOLOGY model needs (upper bound)."""
    return int(vout_mv*(B_CMP-MODEL_DT_COUNTS)/vin_mv)+MODEL_DT_COUNTS

# Measured transfer line (bench, 2026-09-11, 204 unsaturated samples at VIN=19827):
#   VOUT = 1.1141 mV * count + 1.9 mV  ->  VOUT_max = 0.856 * VIN
MEAS_SLOPE_MV_PER_COUNT=1.1141
MEAS_INTERCEPT_MV=1.9
MEAS_VIN_REF_MV=19827

def measured_duty_for(vout_mv,vin_mv):
    """Duty the real stage would need for that output (bench-fitted line, scaled
    with VIN). Used to state facts like "18 V is not reachable at 20 V input"."""
    return (vout_mv-MEAS_INTERCEPT_MV)/(MEAS_SLOPE_MV_PER_COUNT*vin_mv/MEAS_VIN_REF_MV)

lib=C.CDLL(str(ROOT/'tests/build/firmware_core.dll'))
lib.PD_Feed.argtypes=[C.c_void_p,C.c_uint16]
lib.Test_Time.argtypes=[C.c_uint32]
lib.Test_Pins.argtypes=[C.c_uint8,C.c_uint32]
lib.Test_Sample.argtypes=[C.c_uint8,C.c_int32]
lib.Test_Read.argtypes=[C.c_void_p,C.c_uint32]
lib.Test_Read.restype=C.c_uint32
lib.PD_LatchFault.argtypes=[C.c_uint8,C.c_uint32]
lib.Test_Sensors.argtypes=[C.c_int32,C.c_int32,C.c_int32]
lib.Test_Advance.argtypes=[C.c_uint32]
lib.Test_ZeroApply.argtypes=[C.c_uint8,C.c_uint16]
lib.Test_Compare.argtypes=[C.POINTER(C.c_uint16),C.POINTER(C.c_uint16)]
lib.Test_PowerEnable.restype=C.c_uint32
lib.Test_Gates.restype=C.c_uint32
lib.Test_GateTestIndex.restype=C.c_uint32
lib.Test_GateTestCompare.restype=C.c_uint32
lib.Test_GateTestRunning.restype=C.c_uint32
lib.Test_WaitPeriodsCalls.restype=C.c_uint32
lib.Test_BridgeStatic.argtypes=[C.c_uint8]
lib.Test_BridgeStatic.restype=C.c_uint32
lib.Test_BridgeHighOut2.argtypes=[C.c_uint8]
lib.Test_BridgeHighOut2.restype=C.c_uint32
lib.Test_CompareIllegal.restype=C.c_uint32
lib.Test_CompareWrites.restype=C.c_uint32
lib.Test_CompareMin.restype=C.c_uint32
lib.Test_CompareMax.restype=C.c_uint32
lib.Test_FailCompare.argtypes=[C.c_uint32]
lib.Test_FailWait.argtypes=[C.c_uint32]
lib.Test_CmpMode.argtypes=[C.c_uint32]
lib.Test_CmpCounter.argtypes=[C.c_uint16,C.c_uint16]
lib.Test_Input.argtypes=[C.c_int32]
lib.PD_CmpInit.argtypes=[]
lib.PD_CmpWindowAccepts.argtypes=[C.c_uint16]
lib.PD_CmpWindowAccepts.restype=C.c_uint8
lib.PD_CmpRetryAllowed.argtypes=[C.c_uint32]
lib.PD_CmpRetryAllowed.restype=C.c_uint8
lib.PD_CmpRetries.restype=C.c_uint32
lib.PD_CmpTimeouts.restype=C.c_uint32
lib.PD_CmpStopped.restype=C.c_uint32
lib.PD_CmpGuardExpired.restype=C.c_uint32
lib.PD_CmpCommits.restype=C.c_uint32
lib.PD_CmpCritMax.restype=C.c_uint32
lib.PD_CmpStage.restype=C.c_uint8
lib.PD_CmpLastReason.restype=C.c_uint8
lib.PD_CmpLastCounter.restype=C.c_uint16
lib.PD_OcInit.argtypes=[]
lib.PD_OcArmBlank.argtypes=[C.c_uint32,C.c_uint32]
lib.PD_OcFeed.argtypes=[C.c_uint8,C.c_uint32]
lib.PD_OcFeed.restype=C.c_uint8
lib.PD_OcTrips.restype=C.c_uint32
lib.PD_OcMaxRun.restype=C.c_uint32
lib.PD_OcEvents.restype=C.c_uint32
lib.PD_OcBlankedSamples.restype=C.c_uint32
lib.Test_Ref.argtypes=[C.POINTER(C.c_int32),C.POINTER(C.c_int32),C.POINTER(C.c_int32),C.POINTER(C.c_uint32)]

def tracked_ovp(target_mv,ramp_mv):
    """Python model of the 0.3.20 threshold rule, so the tests can assert that the
    C code tracks the REFERENCE rather than the raw user target."""
    guard=max(target_mv,ramp_mv)
    return min(max(guard*OVP_NUM//OVP_DEN, guard+OVP_MARGIN), OVP_ABS_MAX)

def down_budget(span_mv):
    """Ramp time for the span plus settling, floored - never a flat 10 ms."""
    return max(span_mv//RUN_RAMP_DOWN+DOWN_SETTLE_MS, DOWN_TIMEOUT_MIN_MS)

def read():
    out=C.create_string_buffer(65536)
    length=lib.Test_Read(out,len(out))
    return out.raw[:length]

def command(op,ch=0,payload=b'',seq=1):
    wire=Frame(KIND['REQUEST'],seq,CMD[op],ch,payload).encode()
    lib.PD_Feed(wire,len(wire))
    frames=Parser().feed(read())
    return next((f for f in frames if f.seq==seq and f.opcode==CMD[op]),None)

def command_frames(op,ch=0,payload=b'',seq=1):
    """Like command(), but returns EVERY frame the reply produced (the ACK/NACK
    plus any LOG lines emitted while the command was processed), so a test can
    assert on the reason text as well."""
    wire=Frame(KIND['REQUEST'],seq,CMD[op],ch,payload).encode()
    lib.PD_Feed(wire,len(wire))
    return Parser().feed(read())

def log_messages(frames):
    return ' | '.join(unpack('log',f.payload)['message'] for f in frames if f.opcode==CMD['LOG'])

class CoreTests(unittest.TestCase):
    def setUp(self):lib.Test_Reset()
    def test_shared_header_and_info(self):
        self.assertEqual((ROOT/'App/powerdesk_protocol.h').read_bytes(),(PC/'protocol/powerdesk_protocol.h').read_bytes())
        f=command('INFO');self.assertEqual(f.kind,KIND['ACK'])
        info=unpack('info',f.payload)
        self.assertEqual(info['schema_crc'],SCHEMA_CRC)
        self.assertEqual(info['watchdog_ms'],1000)
    def test_target_and_readback(self):
        # 0.3.20: SET_TARGET is limited to the window the restricted buck can
        # actually produce (2..15 V). The old generic 1..30 V window accepted a
        # 30 V buck target that the stage can never reach.
        target=dict(vref_mv=12000,iref_ma=2000)
        f=command('SET_TARGET',1,pack('target',target))
        self.assertEqual(unpack('target',f.payload),target)
        status=unpack('telemetry',command('STATUS',1).payload)
        self.assertEqual(status['vref_mv'],12000)
        self.assertEqual(status['iref_ma'],2000)
        self.assertEqual(status['enabled'],0)
        # One mV above the DECLARED top. It used to be the literal 30000 while the top was 20 V;
        # it is derived now, because the top keeps moving and a literal silently becomes a legal
        # target - which is exactly how this test started failing with an empty NACK payload
        # (the firmware ACCEPTED 30000 once the top reached 30 V, so there was no error to unpack).
        f=command('SET_TARGET',1,pack('target',dict(vref_mv=BUCK_VMAX+1,iref_ma=1000)))
        self.assertEqual((f.kind,unpack('error',f.payload)['code']),(KIND['NACK'],ERR['RANGE']))
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],12000)
    def test_target_range_and_disabled_second_channel(self):
        for v,i,ch,code in [(999,1000,1,'RANGE'),(30001,1000,1,'RANGE'),(5000,2001,1,'RANGE'),
                            (1999,1000,1,'RANGE'),(20001,1000,1,'RANGE'),(45000,1000,1,'RANGE'),
                            (5000,1000,2,'UNSUPPORTED')]:
            f=command('SET_TARGET',ch,pack('target',dict(vref_mv=v,iref_ma=i)))
            self.assertEqual(f.kind,KIND['NACK']);self.assertEqual(unpack('error',f.payload)['code'],ERR[code])
        # The window edges themselves are accepted, and the declared ceiling must be
        # storable while the output is OFF (staging before VIN is up). 0.3.22 raised it
        # to 18 V, 0.3.31 to 20 V; 18000 stays in the list because it is still a legal
        # target and the test would catch a regression that refused it.
        for v in (BUCK_VMIN,BUCK_VMAX,18000):
            self.assertEqual(command('SET_TARGET',1,pack('target',dict(vref_mv=v,iref_ma=100))).kind,KIND['ACK'])
    def test_start_requires_gate_map_and_zero(self):
        # CH1 refuses to start while the gate mapping is unverified; CH2 has no power stage.
        f=command('OUTPUT',1,pack('output',dict(enabled=1)))
        self.assertEqual(unpack('error',f.payload)['code'],ERR['NOT_READY'])
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['enabled'],0)
        f=command('OUTPUT',2,pack('output',dict(enabled=1)))
        self.assertEqual(unpack('error',f.payload)['code'],ERR['UNSUPPORTED'])
    def test_pid_float32_and_invalid_payload(self):
        p=dict(loop=2,kp=.25,ki=1.2,kd=.001,tau=.01,out_min=0,out_max=.5)
        raw=pack('pid',p)
        self.assertEqual(command('SET_PID',1,raw).payload,raw)
        self.assertEqual(command('GET_PID',1,pack('loop',{'loop':2})).payload,raw)
        invalid=LAYOUTS['pid'].pack(0,float('nan'),1,0,0,0,1)
        self.assertEqual(command('SET_PID',1,invalid).kind,KIND['NACK'])
        self.assertEqual(command('SET_PID',1,b'x').kind,KIND['NACK'])
    def test_fault_clear_requires_released_pin(self):
        lib.Test_Pins(1,1);lib.PD_LatchFault(1,1)
        self.assertEqual(command('CLEAR_FAULT',1).kind,KIND['NACK'])
        lib.Test_Pins(1,0)
        self.assertEqual(command('CLEAR_FAULT',1).kind,KIND['ACK'])
        status=unpack('telemetry',command('STATUS',1).payload)
        self.assertEqual((status['fault'],status['enabled'],status['state']),(0,0,2))
    def test_heartbeat_timeout_latches(self):
        command('HEARTBEAT',seq=0);lib.Test_Time(1001);read()
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['fault']&16,16)
    def test_capture_exact_samples_decimation_and_chunks(self):
        meta=unpack('trace_info',command('TRACE_ARM',1,pack('trace_arm',{'points':16,'decimation':10})).payload)
        for i in range(160):lib.Test_Sample(1,i)
        meta=unpack('trace_info',command('TRACE_INFO',1).payload)
        self.assertEqual((meta['state'],meta['sample_hz']),(2,config_int('PD_FAST_HZ')//10))
        raw=command('TRACE_READ',1,pack('trace_read',{'capture_id':meta['capture_id'],'offset':0,'count':16})).payload
        values=[unpack('trace_point',raw[8+i*20:28+i*20])['il_ma'] for i in range(16)]
        self.assertEqual(values,list(range(0,160,10)))
    def test_bad_crc_fragmentation_resync(self):
        raw=Frame(KIND['REQUEST'],123,CMD['INFO']).encode()
        bad=bytearray(raw);bad[-1]^=1
        noise=b'noise'+bytes(bad)
        lib.PD_Feed(noise,len(noise));self.assertEqual(read(),b'')
        for byte in raw:lib.PD_Feed(bytes([byte]),1)
        self.assertEqual(Parser().feed(read())[0].seq,123)
    def test_zero_sequence_cannot_change_target(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=20000,iref_ma=1000)),seq=0)
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],5000)
    def test_log_frame_can_decode(self):
        lib.Test_Time(5000)
        logs=[unpack('log',f.payload) for f in Parser().feed(read()) if f.opcode==CMD['LOG']]
        # Matching content the health line still guarantees, not a phrase it used to carry: the
        # literal 'restricted buck' was dropped in 0.3.40 to fit the 95-byte log field (pd_core.c).
        self.assertTrue(any('ref ' in p['message'] and 'cal=' in p['message'] for p in logs),
                        f'no 5 s health line among {len(logs)} log frames')
    def test_first_fault_log_reports_queue_failure_and_uses_priority(self):
        lib.PD_TryLog.argtypes=[C.c_uint8,C.c_uint8,C.c_uint16,C.c_char_p]
        lib.Test_TxBusy(1)
        self.assertEqual(lib.PD_TryLog(1,2,22,b'FSTOP test'),0)
        self.assertEqual(read(),b'')
        lib.Test_TxBusy(0)
        self.assertEqual(lib.PD_TryLog(1,2,22,b'FSTOP test'),1)
        self.assertEqual(lib.Test_TxPriority(),1)
        frame=Parser().feed(read())[0]
        self.assertEqual(unpack('log',frame.payload)['message'],'FSTOP test')
    def test_shadow_terms_are_visible_without_enabling_outputs(self):
        command('SET_MODE',1,b'\x01')
        command('SET_PID',1,pack('pid',dict(loop=0,kp=2,ki=0,kd=0,tau=0,out_min=-1,out_max=1)))
        command('DEBUG_REF',1,pack('debug_ref',dict(ilref_ma=100,duration_ms=1000)))
        lib.Test_Sample(1,50);lib.Test_Time(500)
        logs=[unpack('log',f.payload) for f in Parser().feed(read()) if f.opcode==CMD['LOG']]
        entry=next(p for p in logs if p['code']==12)
        self.assertIn('P=0.1',entry['message']);self.assertIn('U=0.1',entry['message'])
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['enabled'],0)
        lib.PD_LatchFault(1,1);lib.Test_Sample(1,50);lib.Test_Time(1000)
        logs=[unpack('log',f.payload) for f in Parser().feed(read()) if f.opcode==CMD['LOG']]
        self.assertIn('U=0',next(p['message'] for p in logs if p['code']==12))
    def test_uninstalled_channel_has_no_scheduled_pid_frequency(self):
        self.assertEqual(unpack('caps',command('CAPS',2).payload)['current_hz'],0)

class Config(C.Structure):_fields_=[(n,C.c_float) for n in ('kp','ki','kd','tau','out_min','out_max')]
class PIDState(C.Structure):_fields_=[('integral',C.c_float),('derivative',C.c_float),('previous_y',C.c_float),('initialized',C.c_uint8)]
lib.PD_PIDStep.argtypes=[C.POINTER(Config),C.POINTER(PIDState),C.c_float,C.c_float,C.c_float]
lib.PD_PIDStep.restype=C.c_float
class PIDTests(unittest.TestCase):
    def test_integral_uses_seconds(self):
        p=Config(0,2,0,0,-10,10);s=PIDState()
        for _ in range(100):y=lib.PD_PIDStep(C.byref(p),C.byref(s),1,0,.001)
        self.assertAlmostEqual(y,.2,places=5)
    def test_saturation_does_not_wind_up(self):
        p=Config(0,10,0,0,-1,1);s=PIDState()
        for _ in range(1000):lib.PD_PIDStep(C.byref(p),C.byref(s),100,0,.001)
        self.assertLessEqual(s.integral,1)
        value=lib.PD_PIDStep(C.byref(p),C.byref(s),-100,0,.001)
        self.assertLess(value,1)
    def test_derivative_has_no_setpoint_kick(self):
        p=Config(0,0,1,.1,-100,100);s=PIDState()
        lib.PD_PIDStep(C.byref(p),C.byref(s),0,0,.01)
        self.assertEqual(lib.PD_PIDStep(C.byref(p),C.byref(s),10,0,.01),0)

class NativeSerial:
    """Byte transport to actual MCU core C; no physical ADC/gate simulation."""
    def __init__(self):
        self.lock=threading.Lock();self.started=time.monotonic();self.closed=False;lib.Test_Reset()
    def write(self,data):
        with self.lock:lib.PD_Feed(data,len(data))
        return len(data)
    def read(self,n=4096):
        time.sleep(.01)
        with self.lock:
            lib.Test_Time(int((time.monotonic()-self.started)*1000))
            return read()
    def close(self):self.closed=True
class EndToEndTests(unittest.TestCase):
    def test_real_powerdesk_controller_against_c_firmware(self):
        d=Device()
        with patch('device.DemoSerial',NativeSerial):
            try:
                d.connect('',demo=True)
                self.assertEqual(d.info['firmware'],FW_VERSION)
                self.assertEqual(d.caps[1]['vmin_mv'],2000)
                # 0.3.22 widens the qualified CH1 range from 15 V to 18 V. The
                # host reads this from CAPS, so no host-side constant can drift.
                self.assertEqual(d.caps[1]['vmax_mv'],BUCK_VMAX)
                self.assertEqual(d.caps[1]['vmax_mv'],20000, 'CAPS moved to 20 V in 0.3.31')
                self.assertEqual(OVP_ABS_MAX,23000,
                                 'the OVP ceiling follows the declared range')
                self.assertEqual(d.caps[1]['voltage_hz'],1000)
                self.assertEqual(d.caps[1]['cc_hz'],1000)
                self.assertEqual(d.action('target',1,{'vref_mv':12000,'iref_ma':1500})['vref_mv'],12000)
                p=d.action('get_pid',1,{'loop':0});p['kp']=.25
                self.assertEqual(d.action('set_pid',1,p)['kp'],.25)
                self.assertEqual(d.action('zero',1,{'action':0})['applied'],0)
                self.assertEqual(d.action('zero',1,{'action':1})['state'],1)
                d.action('stop_all')
                self.assertEqual(d.action('zero',1,{'action':0})['reason'],5)
            finally:d.disconnect()

class ZeroTests(unittest.TestCase):
    def setUp(self):
        lib.Test_Reset()
        lib.PD_ZeroFeed.argtypes=[C.c_uint32,C.c_uint8,C.c_uint8,C.c_uint16]
        lib.PD_ZeroCurrent.argtypes=[C.c_uint8,C.c_uint16]
        lib.PD_ZeroCurrent.restype=C.c_int32
    def status(self):return unpack('zero_status',command('ZERO',1,b'\0').payload)
    def start(self,sensor=1):
        self.assertEqual(command('ZERO',1,bytes([sensor])).kind,KIND['ACK'])
    def feed(self,value=2040):
        for i in range(4096):lib.PD_ZeroFeed(500+i,1,1,value(i) if callable(value) else value)
    def test_nominal_then_independent_measured_zeros_and_reset(self):
        self.assertEqual(self.status()['iout_uv'],1650000)
        self.start();self.feed()
        self.assertEqual(self.status()['applied'],1)
        self.assertEqual(self.status()['samples'],4096)
        self.assertEqual(self.status()['state'],3)
        self.assertEqual(lib.PD_ZeroCurrent(1,2040),0)
        self.assertEqual(self.status()['il_uv'],1650000)
        self.start(2);self.feed(2078)
        self.assertEqual(self.status()['applied'],3)
        self.assertEqual(lib.PD_ZeroCurrent(2,2078),0)
        self.assertGreater(lib.PD_ZeroCurrent(2,2128),390) # noise/signal not clamped away
        command('ZERO',1,b'\x03')
        self.assertEqual(self.status()['applied'],2)
        self.assertEqual(self.status()['iout_uv'],1650000)
        lib.Test_Reset();self.assertEqual(self.status()['applied'],0)
    def test_failed_recalibration_keeps_old_zero(self):
        self.start();self.feed();old=self.status()['iout_uv']
        self.start();self.feed(lambda i:1900 if i%2 else 2200)
        self.assertEqual(self.status()['state'],4);self.assertEqual(self.status()['reason'],3)
        self.assertEqual(self.status()['iout_uv'],old);self.assertEqual(self.status()['applied'],1)
    def test_drift_is_rejected_even_with_small_peak_to_peak(self):
        self.start();self.feed(lambda i:2040+(i//512)*2)
        self.assertEqual(self.status()['reason'],3)
        self.assertGreater(self.status()['span_uv'],5000)
        self.assertEqual(self.status()['applied'],0)
    def test_rail_and_bad_mean_are_rejected(self):
        for raw in (0,4095,1000):
            lib.Test_Reset();self.start();self.feed(raw)
            self.assertEqual(self.status()['reason'],2);self.assertEqual(self.status()['applied'],0)
    def test_settling_freshness_timeout_and_bad_conditions(self):
        self.start();lib.PD_ZeroFeed(499,1,1,2040)
        self.assertEqual(self.status()['samples'],0)
        lib.PD_ZeroFeed(500,1,0,2040);self.assertEqual(self.status()['samples'],0)
        lib.PD_ZeroFeed(500,1,1,2040);lib.PD_ZeroFeed(500,1,1,2040)
        self.assertEqual(self.status()['samples'],1)
        lib.PD_ZeroFeed(8001,1,0,2040);self.assertEqual(self.status()['reason'],4)
        lib.Test_Reset();self.start();lib.PD_ZeroFeed(501,0,1,2040)
        self.assertEqual(self.status()['reason'],1)
    def test_stop_fault_and_mode_interlock(self):
        self.start()
        for op,p in [('ZERO',b'\x02'),('SET_MODE',b'\x01'),('TRACE_ARM',pack('trace_arm',dict(points=256,decimation=1)))]:
            self.assertEqual(unpack('error',command(op,1,p).payload)['code'],ERR['BUSY'])
        command('STOP_ALL');self.feed()
        self.assertEqual(self.status()['reason'],5);self.assertEqual(self.status()['applied'],0)
        self.start();lib.PD_LatchFault(1,1);self.feed()
        self.assertEqual(self.status()['reason'],5);self.assertEqual(self.status()['applied'],0)
    def test_contract_rejects_invalid_channel_action_length(self):
        for ch,p,err in [(2,b'\x01','UNSUPPORTED'),(1,b'\x05','RANGE'),(1,b'','BAD_LENGTH')]:
            self.assertEqual(unpack('error',command('ZERO',ch,p).payload)['code'],ERR[err])

class PowerTests(unittest.TestCase):
    """CH1 restricted buck state machine, modulation and protection limits.

    The plant is an ideal buck (VOUT = VIN * d / 0.75) closed in Python, so the
    C code under test is the real 1 kHz controller. No gate or ADC hardware.
    """
    PER=config_int('PD_SW_PERIOD')
    def setUp(self):
        lib.Test_Reset()
        lib.Test_ZeroApply(1,2040)          # measured IOUT zero unlocks the start
        lib.Test_Sensors(20000,0,0)
        self.mapping=0
    def confirm(self,mapping=0):
        self.mapping=mapping
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=2,index=mapping,duration_ms=0)))
        self.assertEqual(f.kind,KIND['ACK'])
    def start(self):
        return command('OUTPUT',1,pack('output',dict(enabled=1)))
    def status(self):
        return unpack('telemetry',command('STATUS',1).payload)
    def compare(self):
        a=C.c_uint16();b=C.c_uint16();lib.Test_Compare(C.byref(a),C.byref(b))
        return a.value,b.value
    def hs_duty(self):
        # The port maps the physical high-side duty onto OUT1/OUT2, so the
        # compare value is the high-side duty regardless of the gate mapping.
        a,_=self.compare()
        return a/self.PER
    def q_duty(self):
        """The transfer coordinate q = 0.75 * A / B, from the COMMITTED pair.

        0.3.28 drives both bridges, so the A compare value alone no longer determines the
        output: the plant sees q, and A is only what the distributor chose to realise it with.
        In legacy mode B is pinned at 0.75 * period, so this reduces to A/period exactly - which
        is why the whole suite kept passing when the modulator changed, and why the plant has to
        read the pair rather than one bridge."""
        a,b=self.compare()
        return (a/self.PER)*D_B/(b/self.PER)
    def ref(self):
        """(target, ramp reference, OVP threshold in force, descent supervised)."""
        t=C.c_int32();r=C.c_int32();o=C.c_int32();d=C.c_uint32()
        lib.Test_Ref(C.byref(t),C.byref(r),C.byref(o),C.byref(d))
        return t.value,r.value,o.value,d.value
    def log_text(self,code):
        """All log messages with that code, drained since the last read()."""
        return ' | '.join(unpack('log',f.payload)['message']
                          for f in Parser().feed(read())
                          if f.opcode==CMD['LOG'] and unpack('log',f.payload)['code']==code)
    def run_plant(self,ticks,r_load=20.0,vin=20000,ideal=False,plant_scale=1.0):
        """Approximate plant.

        Default = the model the bench supports: the high time loses one dead time
        AND the measured gate-edge skew, so
            VOUT = VIN * (d - d_model) / (0.75 - d_model)
        with d_model = PD_MODEL_DT_COUNTS/PD_SW_PERIOD. This is still only a
        model: no diode drops, no reverse conduction, no losses, no loop delay.
        ideal=True keeps the old VOUT = VIN*d/0.75 for logic regression and to
        show how much the dead-time/skew correction moves the operating point.
        plant_scale makes the REAL plant gain differ from the model the feed-forward uses,
        which is how a feed-forward error becomes observable at all: while both sides shared
        one formula, a wrong feed-forward was invisible to every test. plant_scale > 1 means
        the plant delivers more per duty count than the model assumes (the bench case, where
        the loop ended up with the integral pinned at zero and a structural offset).
        Passing this plant is NOT evidence that 18 V is stable on hardware.

        The duty used below is q = 0.75 * A / B, read from the committed PAIR, because q is the
        quantity the transfer model is defined on. Reading only the A compare value would make
        the plant disagree with the firmware by up to 0.7 V once the distributor starts moving B
        (q > 0.65) - i.e. exactly in the region phase B of the plan is about.
        """
        for _ in range(ticks):
            lib.Test_Advance(1)
            d=self.q_duty()
            if ideal:
                vout=vin*d/D_B
            else:
                vout=vin*(d*plant_scale-D_MODEL)/(D_B-D_MODEL)
            vout=max(0.0,min(30000.0,vout))
            lib.Test_Sensors(vin,int(round(vout)),int(round(vout/r_load)))  # mV/ohm = mA
    def designed_duty(self,vout_mv,vin_mv=20000.0,ideal=False):
        """Commanded duty the loop must settle on for that output."""
        if ideal:
            return vout_mv*D_B/vin_mv
        return vout_mv*(D_B-D_MODEL)/vin_mv+D_MODEL
    def test_a_bridge_ceiling_applies_to_modulation_but_not_to_the_gate_test(self):
        """Plan 4.2 / 9.1-4: the 70 % ceiling (= PD_CMP_A_MAX counts) constrains the
        NORMAL restricted-buck modulation only. The gate test deliberately drives a
        20 %/80 % pair to refresh the bootstrap, so the clamp must sit in
        commit_duties() and never in the port layer - otherwise the gate test (and
        the static precharge hand-off) would be silently altered."""
        lib.Test_CompareMaxA.restype=C.c_uint32
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1000)))
        self.confirm()
        for _ in range(5):                                     # 18 V needs >= 21.26 V (0.3.23)
            lib.Test_Sensors(21500,0,0)
            lib.Test_Advance(1)
        self.assertEqual(lib.Test_CompareMaxA(),0)
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500,vin=21500)
        self.assertGreater(lib.Test_CompareWrites(),10)
        self.assertLessEqual(lib.Test_CompareMaxA(),A_MAX)       # normal modulation capped
        self.assertGreater(lib.Test_CompareMaxA(),0)
        self.assertEqual(lib.Test_CompareIllegal(),0)
        # Now the gate test: VIN/VOUT discharged, output off, drive PA9 (20 %) whose
        # mate runs 80 % = 0.8*PER = 17408 counts, i.e. ABOVE the A-bridge ceiling.
        command('OUTPUT',1,pack('output',dict(enabled=0)))
        for _ in range(800):                                   # 18 V stop ramp is 450 ms
            lib.Test_Advance(1)
        lib.Test_Sensors(0,0,0)
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=1,index=2,duration_ms=300)))
        self.assertEqual(f.kind,KIND['ACK'])
        self.assertEqual(lib.Test_GateTestRunning(),1)
        self.assertEqual(lib.Test_GateTestCompare(),int(PER*0.8))
        self.assertGreater(lib.Test_GateTestCompare(),A_MAX)     # NOT clamped by the ceiling
        lib.Test_Advance(400)
        self.assertEqual(lib.Test_GateTestRunning(),0)
        self.assertEqual(lib.Test_Gates(),0)
    def test_start_sequence_reaches_run_cv_at_the_designed_duty(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        # Wiring is fixed; a measured IOUT zero and the other interlocks suffice.
        self.assertEqual(lib.Test_WaitPeriodsCalls(),0)
        self.assertEqual(self.start().kind,KIND['ACK'])
        # The preloaded compare values must be in effect before the gates turn on.
        self.assertGreaterEqual(lib.Test_WaitPeriodsCalls(),1)
        self.assertEqual(lib.Test_PowerEnable(),1)                 # PA3 high
        self.assertEqual(lib.Test_Gates(),0x0f)                    # all four outputs enabled
        # Precharge is a STATIC low-side state, not a compare value: no illegal
        # HRTIM compare may ever be programmed (PD_CMP_MIN = 0x60).
        self.assertEqual(lib.Test_BridgeStatic(1),1)
        self.assertEqual(lib.Test_BridgeStatic(2),1)
        self.assertEqual(self.compare(),(96,config_int('PD_OUT_BRIDGE_CMP')))
        self.assertEqual(self.status()['state'],3)
        self.run_plant(20)
        self.assertEqual(self.status()['state'],4)                 # soft start
        self.assertEqual(lib.Test_BridgeStatic(1),0)               # PWM restored
        self.assertEqual(lib.Test_BridgeStatic(2),0)
        self.run_plant(1500)
        st=self.status()
        self.assertEqual((st['state'],st['enabled']),(5,1))
        self.assertAlmostEqual(self.hs_duty(),self.designed_duty(5000),delta=0.01)
        self.assertEqual(self.compare()[1],config_int('PD_OUT_BRIDGE_CMP'))  # output bridge 75 %
        self.assertLess(self.compare()[0],self.compare()[1])       # refresh window is clear
        self.assertEqual(lib.Test_CompareIllegal(),0)              # never below 0x60
    def test_every_compare_value_written_stays_legal(self):
        """0x60 = 96 is the documented HRTIM high-resolution floor; a value below
        it may not generate a compare event, leaving the high side on for the
        whole period - the 0.3.16 bench over-current.

        The current limit is 300 mA, not 100 mA: with a 20 ohm load at 2 V the
        load current IS 100 mA, so a 100 mA limit puts the operating point exactly
        on the CC boundary and the state legitimately reads RUN_CC (see
        test_cc_deadband_sits_below_the_limit_at_the_boundary_operating_point)."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600)
        self.assertEqual(self.status()['state'],5)
        command('OUTPUT',1,pack('output',dict(enabled=0)))
        for _ in range(400):                                       # ramp down to stop
            lib.Test_Advance(1)
            lib.Test_Sensors(20000,0,0)
        self.assertGreater(lib.Test_CompareWrites(),10)
        self.assertEqual(lib.Test_CompareIllegal(),0)
        self.assertGreaterEqual(lib.Test_CompareMin(),96)
        self.assertLessEqual(lib.Test_CompareMax(),config_int('PD_SW_PERIOD')+1)
    def test_start_aborts_when_the_compare_commit_fails(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        lib.Test_FailCompare(1)
        self.assertNotEqual(self.start().kind,KIND['ACK'])
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(self.status()['enabled'],0)
        lib.Test_FailCompare(0)
    def test_start_aborts_when_the_period_wait_times_out(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        lib.Test_FailWait(1)
        self.assertNotEqual(self.start().kind,KIND['ACK'])
        self.assertEqual(lib.Test_Gates(),0)
        lib.Test_FailWait(0)
    def test_compare_window_missed_inside_masked_section_retries_next_period(self):
        """The 0.3.18 defect: the window closed between the unlocked check and the
        masked re-check, and one miss stopped the stage. It must retry instead."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        before=lib.Test_CompareWrites()
        lib.Test_CmpMode(1)                                        # close the window once
        self.run_plant(1500)                                       # soft start -> RUN_CV
        st=self.status()
        self.assertEqual(st['state'],5)                            # kept running
        self.assertEqual(st['fault'],0)
        self.assertEqual(lib.Test_Gates(),0x0f)
        self.assertGreaterEqual(lib.PD_CmpRetries(),1)
        self.assertEqual(lib.PD_CmpTimeouts(),0)
        self.assertGreater(lib.Test_CompareWrites(),before)        # the retry wrote
        lib.Test_CmpMode(0)
    def test_compare_retry_limit_latches_internal_and_stops(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        lib.Test_CmpMode(4)                                        # masked check always fails
        self.run_plant(5)
        st=self.status()
        self.assertEqual(st['fault'],128)                          # PD_FAULTS_INTERNAL
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.PD_CmpTimeouts(),1)
        self.assertGreaterEqual(lib.PD_CmpRetries(),PD_CMP_MAX_RETRIES)
        lib.Test_CmpMode(0)
    def test_compare_counter_stopped_times_out_via_wait_guard(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        lib.Test_CmpMode(2)                                        # counter never in the window
        self.run_plant(5)
        st=self.status()
        self.assertEqual(st['fault'],128)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.PD_CmpGuardExpired(),1)               # guard, not retry limit
        lib.Test_CmpMode(0)
    def test_fault_during_compare_wait_is_not_ours_and_never_reenables(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        lib.Test_CmpMode(3)                                        # fault lands while waiting
        self.run_plant(3)
        st=self.status()
        self.assertEqual(st['fault'],1)                            # the injected OC, not internal
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertGreaterEqual(lib.PD_CmpStopped(),1)
        lib.Test_CmpMode(0)
    def test_compare_commit_reports_its_stage(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        self.confirm()
        lib.Test_CmpMode(4)
        self.assertNotEqual(self.start().kind,KIND['ACK'])         # stage 1: power start
        self.assertEqual(lib.PD_CmpStage(),1)
        self.assertEqual(lib.PD_CmpLastReason(),3)                 # PD_CMP_ERR_TIMEOUT
        lib.Test_CmpMode(0)
        command('CLEAR_FAULT',1)                                   # stage 1 left a fault
        self.assertEqual(self.start().kind,KIND['ACK'])
        stages=set()
        for _ in range(6):
            self.run_plant(1)
            stages.add(lib.PD_CmpStage())
        self.assertIn(2,stages)                                    # stage 2: precharge -> PWM
        self.assertEqual(lib.PD_CmpStage(),3)                      # stage 3: 1 kHz loop
        self.assertEqual(lib.PD_CmpLastReason(),0)
    def test_compare_window_policy_has_margin_before_the_update_event(self):
        """Both writes must finish before the master update event; the accepted
        counter range must therefore stop PD_CMP_COMMIT_MARGIN counts short of it."""
        self.assertEqual(lib.PD_CmpWindowAccepts(0),1)
        self.assertEqual(lib.PD_CmpWindowAccepts(PD_CMP_LATEST),1)
        self.assertEqual(lib.PD_CmpWindowAccepts(PD_CMP_LATEST+1),0)
        self.assertEqual(lib.PD_CmpWindowAccepts(config_int('PD_SW_PERIOD')),0)
        self.assertGreaterEqual(PD_CMP_COMMIT_MARGIN,40*32)         # >= 40 core cycles
        retries=0
        while lib.PD_CmpRetryAllowed(retries): retries+=1
        self.assertEqual(retries,PD_CMP_MAX_RETRIES)
    def test_oc_backstop_ignores_the_blank_window_but_trips_after_it(self):
        """A 160 us gate-enable transient must not stop the stage; a held OC must."""
        lib.PD_OcInit()
        lib.PD_OcArmBlank(0,50)
        for i in range(16):                                        # 160 us inside blank
            self.assertEqual(lib.PD_OcFeed(1,0),0)
        self.assertEqual(lib.PD_OcTrips(),0)
        for i in range(20):                                        # released
            self.assertEqual(lib.PD_OcFeed(0,0),0)
        # A run that starts INSIDE the blank window and keeps going must trip as
        # soon as the window closes (the old "== threshold" test missed this).
        lib.PD_OcInit()
        lib.PD_OcArmBlank(0,1)
        tripped=0
        for i in range(300):                                       # 3 ms held low
            tripped|=lib.PD_OcFeed(1,i//100)
        self.assertEqual(tripped,1)
        self.assertEqual(lib.PD_OcTrips(),1)
        self.assertEqual(lib.PD_OcMaxRun()>=100,True)
    def test_non_designed_gate_mapping_cannot_change_fixed_wiring(self):
        """Legacy inversion commands are rejected before changing the mapping."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        for mapping in (1,2,3):
            f=command('GATE_TEST',1,pack('gate_test_request',dict(action=2,index=mapping,duration_ms=0)))
            self.assertEqual(f.kind,KIND['NACK'])
            self.assertEqual(unpack('error',f.payload)['code'],ERR['UNSUPPORTED'])
            self.assertEqual(lib.Test_PowerEnable(),0)
            self.assertEqual(lib.Test_Gates(),0)
            self.assertEqual(self.compare(),(0,0))
            self.assertEqual(self.status()['enabled'],0)
        self.confirm(0)
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.assertEqual(lib.Test_BridgeHighOut2(1),0)             # design mapping
        self.assertEqual(lib.Test_BridgeHighOut2(2),0)
    def test_stale_sampling_latches_and_never_resumes_automatically(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600)
        held=self.compare()
        lib.Test_FastAge(STALE_MS+1)
        # While the 100 kHz pair is stale a lying sensor reads 0 V; the stage must
        # stop instead of letting the integrator wind the duty up to the clamp.
        for _ in range(500):
            lib.Test_Advance(1)
            lib.Test_Sensors(20000,0,0)
        self.assertEqual(self.compare(),held)
        st=self.status()
        self.assertEqual((st['state'],st['fault']),(8,32))
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.Test_PowerEnable(),0)
        lib.Test_FastAge(0)
        self.run_plant(1)
        self.assertEqual(self.compare(),held)
        self.assertEqual(self.status()['enabled'],0)
    def test_fault_during_start_wait_never_reenables_outputs(self):
        self.confirm()
        lib.Test_FaultInWait(1)
        self.assertEqual(self.start().kind,KIND['NACK'])
        self.assertEqual(lib.Test_UnsafeEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(self.status()['state'],8)
    def test_fault_during_precharge_exit_does_not_restore_pwm(self):
        self.confirm();self.start()
        lib.Test_FaultInWait(1)
        lib.Test_Advance(3)
        self.assertEqual(lib.Test_BridgeStatic(1),1)
        self.assertEqual(lib.Test_BridgeStatic(2),1)
        self.assertEqual(self.status()['state'],8)
        self.assertEqual(lib.Test_Gates(),0)
    def test_trace_and_first_fault_state_exist_at_gate_enable(self):
        self.confirm()
        lib.Test_FaultOnGates(1)
        self.assertEqual(self.start().kind,KIND['NACK'])
        meta=unpack('trace_info',command('TRACE_INFO',1).payload)
        self.assertEqual(meta['state'],3)  # capture failed on fault, not missed
        self.assertEqual(lib.Test_FirstState(),3)
        self.assertEqual(lib.Test_FirstEnable(),1)
        self.assertEqual(self.status()['enabled'],0)
        lib.Test_Time(1)
        logs=[unpack('log',f.payload)['message'] for f in Parser().feed(read()) if f.opcode==CMD['LOG']]
        # 1234 mA is above PD_LOG_VALUE_MAX, so the line prints the bounded marker
        # 1000 ("at or beyond the cap"): the line has to fit the 96-byte log field,
        # which test_log_lines_fit_the_protocol_field enforces for every line.
        self.assertTrue(any('n=1/256' in x and '1st=1000' in x for x in logs))
        lib.PD_LatchFault(1,32)
        self.assertEqual(lib.Test_FirstState(),3)  # secondary fault cannot overwrite
    def test_invalid_feedback_stops_even_during_precharge(self):
        for invalid in (1,2,4,8):  # VIN, IIN, VOUT, IOUT; IL not used by CV/slow CC
            with self.subTest(invalid=invalid):
                self.setUp();self.confirm();self.start()
                lib.Test_Invalid(invalid)
                lib.Test_Advance(1)
                self.assertEqual(self.status()['fault'] & 32,32)
                self.assertEqual(lib.Test_Gates(),0)
    def test_unhealthy_sampler_stops_active_power(self):
        self.confirm();self.start()
        lib.Test_Unhealthy(1);lib.Test_Advance(1)
        self.assertEqual(self.status()['fault'] & 32,32)
        self.assertEqual(lib.Test_Gates(),0)
    def test_periodic_status_and_target_frames_carry_the_right_fields(self):
        """The 1 Hz diagnostics must be rendered from the same code the firmware uses, with
        the operator-facing values in the right fields.

        Why this test: splitting the code-17 line to fit the 96-byte field dropped the
        periodic target/reference pair, and the length gate cannot see a MISSING line. The
        frames now live in App/pd_diag.c, which this DLL links, so they are exercised here
        instead of only at the bench. It also pins the raw-duty decision: a bounded 999-cap
        would print 1000 for every realistic duty and the line would explain nothing."""
        lib.Test_DiagStatus.argtypes=[C.c_char_p,C.c_uint32,C.c_uint8,C.c_uint32,C.c_uint16,
                                      C.c_uint16,C.c_uint8,C.c_int32,C.c_int32,C.c_int32,C.c_int32]
        lib.Test_DiagStatus.restype=C.c_uint32
        lib.Test_DiagTarget.argtypes=[C.c_char_p,C.c_uint32,C.c_int32,C.c_int32,C.c_int32,C.c_uint8]
        lib.Test_DiagTarget.restype=C.c_uint32
        buffer=C.create_string_buffer(96)      # the firmware log field is 96 bytes incl. NUL
        # code 17 at the 18 V operating point measured on the bench: the A bridge sits
        # just under the ceiling, 21.5 V in, and the measured line allows 18 V.
        length=lib.Test_DiagStatus(buffer,len(buffer),5,0x00,14897,B_CMP,0,900,17891,21500,18000)
        status=buffer.value.decode()
        self.assertEqual(length,len(status))
        self.assertRegex(status,r'^s=5 f=00 D=\d+/\d+ S=[01] ')
        self.assertIn(f'D=14897/{B_CMP}',status)              # RAW count, not capped
        self.assertIn('S=0',status)
        self.assertIn('IO=900',status)
        self.assertIn('VO=17891',status)
        self.assertIn('VI=21500',status)
        self.assertIn('vm=18000',status)
        # Saturation is reported by the flag, and the duty is still the real count.
        lib.Test_DiagStatus(buffer,len(buffer),5,0x00,A_MAX,B_CMP,1,900,16960,20000,16920)
        saturated=buffer.value.decode()
        self.assertIn(f'D={A_MAX}/{B_CMP}',saturated)
        self.assertIn('S=1',saturated)
        self.assertIn('vm=16920',saturated)                    # the 20 V input maximum
        # code 25: what the loop is being told.
        length=lib.Test_DiagTarget(buffer,len(buffer),18000,18000,20700,0)
        target=buffer.value.decode()
        self.assertEqual(length,len(target))
        self.assertEqual(target,'tgt=18000 ref=18000 ovp=20700 dn=0')
        lib.Test_DiagTarget(buffer,len(buffer),2000,4400,6500,1)
        self.assertEqual(buffer.value.decode(),'tgt=2000 ref=4400 ovp=6500 dn=1')

    def test_feed_forward_error_is_absorbed_in_both_directions(self):
        """The review asked for feed-forward OVER- and UNDER-estimate scenarios separately, and
        for the plant model to be decoupled from the feed-forward model so a mismatch is even
        observable. Test_FeedPlant scales the real duty->VOUT gain, so the loop's feed-forward
        can be wrong on purpose. Both directions must converge: the integral carries the
        correction now, in either sign."""
        for factor,label in ((1.30,'over-asks 30 %'),(1.15,'over-asks 15 %'),
                             (0.90,'under-asks 10 %'),(0.80,'under-asks 20 %')):
            with self.subTest(feed_forward=label):
                lib.Test_Reset()
                lib.Test_ZeroApply(1,2040)
                lib.Test_Sensors(21500,0,0)
                self.confirm()
                command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=1000)))
                self.assertEqual(self.start().kind,KIND['ACK'])
                self.run_plant(3000,vin=21500,plant_scale=factor)
                vout=unpack('telemetry',command('STATUS',1).payload)['vout_mv']
                self.assertLessEqual(abs(vout-5000),60,
                                     f'{label}: VOUT {vout} mV did not converge to 5000 mV')

    def test_control_term_snapshot_describes_the_tick_that_committed(self):
        """The voltage loop now exports its internals (0.3.25 step 1): executed reference,
        filtered feedback, feed-forward, P, I, filtered output, committed compare and a
        saturation reason. This test pins the snapshot to the control law so the frame cannot
        drift away from what the loop actually did - the reason step 1 exists is that we had
        to REVERSE-ENGINEER the integral from ref/vout/duty before."""
        lib.Test_CtrlTerms.argtypes=[C.POINTER(C.c_int32)]*9+[C.POINTER(C.c_uint32)]*3
        lib.Test_CtrlTerms.restype=None
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600,vin=21500)
        # Read the compare FIRST, then the snapshot, with no command in between: command()
        # drains the frame queue and PD_Poll can advance the loop, so interleaving the two
        # reads would compare values from different ticks.
        committed=self.compare()[0]
        values=[C.c_int32() for _ in range(9)]      # element byref does not work on a ctypes
        mods=[C.c_uint32() for _ in range(3)]       # array here: indexing yields a plain int
        lib.Test_CtrlTerms(*[C.byref(v) for v in values],*[C.byref(m) for m in mods])
        (ref_mv,vout_f_mv,ff_cnt,p_cnt,i_cnt,q_cnt,cmd_cnt,q_app_cnt,cmp_out_cnt)=(
            v.value for v in values)
        mod_limit,mod_zone,sat=(m.value for m in mods)
        # the committed compare is the one in the snapshot
        self.assertEqual(cmd_cnt,committed)
        self.assertLessEqual(cmd_cnt,A_MAX)
        self.assertGreaterEqual(cmd_cnt,PD_CMP_MIN_FOR_TEST)
        # 0.3.28: this build drives BOTH bridges, so the snapshot must also publish the B compare
        # and the q the committed PAIR represents. At this operating point (5 V out of 21.5 V in)
        # q is far below the 0.65 zone boundary, so B must still be the fixed 75 % and q must
        # equal A - i.e. the linked modulator has to reproduce the legacy pair exactly here.
        self.assertEqual(mod_limit,0,f'lim=0x{mod_limit:02X} at an unrestricted point')
        self.assertEqual(cmp_out_cnt,B_CMP, 'B should still be 75 % at 5 V out')
        self.assertEqual(q_app_cnt,cmd_cnt,
                         'below the transition zone q is exactly the A-bridge compare value')
        # the reference the loop executed is near the target once settled
        self.assertAlmostEqual(ref_mv,5000,delta=120)
        self.assertAlmostEqual(vout_f_mv,5000,delta=200)
        # Feed-forward must match duty_ff(ref) recomputed here, within rounding. duty_ff
        # returns a fraction of the period, so the count form is the fraction times the
        # period - not the fraction times the period divided by vin (that mistake made this
        # assertion expect 5971 where the firmware correctly produced 4706).
        # duty_ff uses the OUTPUT-BRIDGE duty (0.75), not unity: the fraction is
        # ref/vin*(0.75 - d_model) + d_model, checked against the firmware at three points
        # (21500/5000 -> 4706, 20000/5000 -> 4970, 21500/3000 -> 3298).
        expected_ff=ref_mv/21500.0*(0.75-D_MODEL)+D_MODEL
        expected_cnt=round(expected_ff*self.PER)
        self.assertAlmostEqual(ff_cnt,expected_cnt,delta=2)
        # the three contributions account for the committed duty (filter lag aside)
        total=ff_cnt+p_cnt+i_cnt
        self.assertAlmostEqual(cmd_cnt,total,delta=40,
                               msg=f'cmd {cmd_cnt} vs ff+p+i {total}')
        # q_cnt is what the controller asked the modulator for, so it must be the same sum
        # rounded to counts; q_app_cnt above is what the pair actually delivers.
        self.assertAlmostEqual(q_cnt,total,delta=40,msg=f'q {q_cnt} vs ff+p+i {total}')
        # saturation reason: this operating point is inside the range, so not limited
        self.assertIn(sat,(0,3))
        # OUTPUT ABOVE THE REFERENCE (0.3.26). This is the bench scenario: a feed-forward that
        # over-asks drives the measured output high, the error goes negative, and before
        # 0.3.26 the integral could only fall to zero - so it pinned there and left a
        # structural offset (measured: +466..+561 mV with i=0, sat=3 at every point below
        # 10 V). The integral is now bidirectional, so the loop must REMOVE the error.
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=1000)))
        self.run_plant(3000,vin=21500,plant_scale=1.30)   # plant 30 % stronger than modelled
        values=[C.c_int32() for _ in range(9)]
        mods=[C.c_uint32() for _ in range(3)]
        lib.Test_CtrlTerms(*[C.byref(v) for v in values],*[C.byref(m) for m in mods])
        ref2,vout_f2,ff2,p2,i2,q2,cmd2,qapp2,b2=(v.value for v in values)
        settled=unpack('telemetry',command('STATUS',1).payload)['vout_mv']
        # The property that matters: the loop converges onto the reference despite a badly
        # wrong feed-forward. 40 mV is generous on purpose - the assertion is that the error
        # is gone, not that the transient is pretty.
        self.assertLessEqual(abs(settled-2000),40,
                             f'VOUT {settled} mV did not converge to 2000 mV with a '
                             f'feed-forward that over-asks by 30 %')
        # ... and it does so by moving the integral NEGATIVE, which the old code forbade.
        self.assertLess(i2,-50,
                        f'integral {i2} counts should have gone negative to remove the '
                        f'over-asked duty')
        # sat semantics: 3 means the integrator itself is on its clamp - with the bidirectional
        # integral and 0.3.28's real clamp test that is a genuine finding, not the normal state
        # of a negative integral (0.3.26 reported 3 for any i <= 0).
        self.assertIn(sat,(0,3))
        # and the frame fits the 96-byte field, with the saturation reason visible
        lib.Test_DiagControl.argtypes=[C.c_char_p,C.c_uint32]+[C.c_int32]*9+[C.c_uint8]*2
        lib.Test_DiagControl.restype=C.c_uint32
        buffer=C.create_string_buffer(96)
        length=lib.Test_DiagControl(buffer,len(buffer),ref_mv,vout_f_mv,ff_cnt,p_cnt,i_cnt,
                                   q_cnt,cmd_cnt,q_app_cnt,cmp_out_cnt,mod_limit,sat)
        line=buffer.value.decode()
        self.assertEqual(length,len(line))
        self.assertLessEqual(length,95)
        self.assertTrue(line.startswith('CT '),line)
        self.assertIn('s=',line)
        self.assertIn('lim=0x',line)
        # the B compare and both q values must be readable back out of the line
        fields=dict((token.split('=',1)[0],token.split('=',1)[1]) for token in line.split()[1:])
        self.assertEqual(int(fields['a']),cmd_cnt)
        self.assertEqual(int(fields['b']),cmp_out_cnt)
        self.assertEqual(int(fields['qr']),q_cnt)
        self.assertEqual(int(fields['q']),q_app_cnt)

    def test_gate_test_wait_failure_returns_nack_with_outputs_off(self):
        lib.Test_Sensors(0,0,0)
        lib.Test_FailWait(1)
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=1,index=1,duration_ms=600)))
        self.assertEqual(f.kind,KIND['NACK'])
        self.assertEqual(lib.Test_GateTestRunning(),0)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.Test_PowerEnable(),0)
    def test_slow_current_limit_folds_the_reference_back(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=10000,iref_ma=200)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(2500)
        st=self.status()
        self.assertEqual(st['state'],6)                            # RUN_CC
        self.assertLess(st['iout_ma'],270)
        self.assertGreater(st['iout_ma'],130)
    def test_overvoltage_latches_and_disables_everything(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=1000)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(800)
        for _ in range(OVP_HOLD_MS+2):                             # must persist
            lib.Test_Sensors(20000,7000,350)
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual(st['state'],8)
        self.assertEqual(st['fault']&4,4)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
    def test_input_overcurrent_is_adaptive_and_floored(self):
        """A 500 mA-limited supply can never reach a fixed 600 mA threshold."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600)
        # 5 V / 0.25 A -> expected input ~78 mA -> limit ~306 mA. 400 mA is a
        # real fault there and must trip; the run must survive 200 mA.
        for _ in range(INPUT_OC_MS+2):
            lib.Test_Sensors(20000,5000,250)
            lib.Test_Input(200)
            lib.Test_Advance(1)
        self.assertEqual(self.status()['fault'],0)
        for _ in range(INPUT_OC_MS+2):
            lib.Test_Sensors(20000,5000,250)
            lib.Test_Input(400)
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual(st['fault']&1,1)
        self.assertEqual(lib.Test_Gates(),0)
    def test_input_overcurrent_floor_catches_a_startup_fault(self):
        """With VOUT ~ 0 the power-balance estimate is 0, so the floor applies."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        for _ in range(INPUT_OC_MS+2):
            lib.Test_Sensors(20000,0,0)
            lib.Test_Input(400)                                    # > 250 mA floor
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual(st['fault']&1,1)
        self.assertEqual(lib.Test_Gates(),0)
    def test_single_sample_excursion_does_not_latch(self):
        """The gate-enable transient disturbs every channel for ~160 us."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600)
        lib.Test_Sensors(20000,9000,0)                             # one bad VOUT sample
        lib.Test_Advance(1)
        self.assertEqual(self.status()['fault'],0)
        lib.Test_Sensors(11400,50,0)                               # one bad VIN sample
        lib.Test_Advance(1)
        st=self.status()
        self.assertEqual((st['fault'],st['state']),(0,5))
    def test_input_window_latches(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=1000)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(300)
        for _ in range(INPUT_HOLD_MS+2):
            lib.Test_Sensors(15000,1000,50)
            lib.Test_Advance(1)
        self.assertEqual(self.status()['fault']&64,64)
        self.assertEqual(lib.Test_Gates(),0)
    def test_sustained_overcurrent_latches_when_cc_cannot_hold(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=200)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(600)
        for _ in range(60):
            lib.Test_Sensors(20000,5000,900)                       # 900 mA vs 600 mA latch limit
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual(st['fault']&1,1)
        self.assertEqual(st['state'],8)
    def test_gate_test_interlock_and_confirmation(self):
        lib.Test_Sensors(20000,0,0)
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=1,index=1,duration_ms=600)))
        self.assertEqual(unpack('error',f.payload)['code'],ERR['NOT_READY'])
        lib.Test_Sensors(0,0,0)                                    # VIN discharged
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=1,index=1,duration_ms=600)))
        self.assertEqual(f.kind,KIND['ACK'])
        self.assertEqual(lib.Test_GateTestRunning(),1)
        self.assertEqual(lib.Test_Gates(),3)                       # whole input half-bridge
        self.assertEqual(lib.Test_PowerEnable(),1)
        lib.Test_Advance(700)
        self.assertEqual(lib.Test_GateTestRunning(),0)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
        f=command('GATE_TEST',1,pack('gate_test_request',dict(action=2,index=3,duration_ms=0)))
        self.assertEqual(unpack('error',f.payload)['code'],ERR['UNSUPPORTED'])
        st=unpack('gate_test_status',command('GATE_TEST',1,pack('gate_test_request',dict(action=0,index=0,duration_ms=0))).payload)
        self.assertEqual((st['confirmed'],st['mapping']),(1,0))
    def test_gate_test_drives_whole_bridge_with_a_bootstrap_window(self):
        """The selected output gets 20 %, its bridge mate 80 % -> switch node is
        pulled low every cycle so the high-side bootstrap can charge."""
        lib.Test_Sensors(0,0,0)
        narrow,wide=self.PER//5,self.PER-self.PER//5
        for index,gates,expect in ((1,3,narrow),(2,3,wide),(3,12,narrow),(4,12,wide)):
            f=command('GATE_TEST',1,pack('gate_test_request',dict(action=1,index=index,duration_ms=300)))
            self.assertEqual(f.kind,KIND['ACK'])
            self.assertEqual(lib.Test_Gates(),gates)               # whole half-bridge
            self.assertEqual(lib.Test_GateTestCompare(),expect)    # narrow pulse on the selected output
            lib.Test_Advance(400)
            self.assertEqual(lib.Test_GateTestRunning(),0)
            self.assertEqual(lib.Test_Gates(),0)
    def test_target_above_buck_headroom_is_refused(self):
        """A target above the DECLARED top is refused by SET_TARGET itself, so nothing is stored,
        no ramp starts and the protection threshold cannot move.

        The value is DERIVED, not a literal. It has been 19000, 21000 and 30001 as the declared top
        moved 15 -> 18 -> 20 -> 30 V, and each time the old literal silently became a legal target -
        which shows up as this test blowing up while unpacking an error out of an ACK payload."""
        lib.Test_Sensors(20000,0,0)
        f=command('SET_TARGET',1,pack('target',dict(vref_mv=BUCK_VMAX+1000,iref_ma=300)))
        self.assertEqual((f.kind,unpack('error',f.payload)['code']),(KIND['NACK'],ERR['RANGE']))
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],5000)  # boot default
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])       # the stored 5 V target is fine
        self.run_plant(1500)
        self.assertEqual(self.status()['state'],5)
        self.assertEqual(self.ref(),(5000,5000,6500,0))
    def test_set_target_out_of_window_cannot_move_reference_or_threshold(self):
        """Item-5 regression: an out-of-window command must not raise the ramp or
        the OVP threshold - it must not do ANYTHING except return RANGE.

        0.3.22: the CONFIGURED window became 2..18 V, so 15001 mV turned legal;
        0.3.31 moved the top to 2..20 V, so 18001 and 19000 turned legal too;
        0.3.34/0.3.37 moved it to 30 V, so 20001/21000/30000 turned legal as well.
        The refused values are the ones outside the window IN FORCE, which is why they are derived
        here instead of being a literal list that goes stale every time the top moves."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        before=self.ref()
        self.assertEqual((before[0],before[2]),(5000,6500))
        for v in (1000,1999,BUCK_VMAX+1,BUCK_VMAX+1000,0,-5000):
            f=command('SET_TARGET',1,pack('target',dict(vref_mv=v,iref_ma=300)))
            self.assertEqual((f.kind,unpack('error',f.payload)['code']),(KIND['NACK'],ERR['RANGE']),v)
            for _ in range(5):
                lib.Test_Sensors(20000,5000,250)
                lib.Test_Advance(1)
            self.assertEqual(self.ref(),before,v)             # nothing moved
            self.assertEqual(self.status()['fault'],0,v)
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],5000)
    def test_runtime_target_above_the_measured_range_is_refused_without_side_effects(self):
        """0.3.22/0.3.23: a value inside CAPS (2..18 V) can still be unreachable at
        the measured VIN. The running stage must refuse it, keep the previous
        target, the reference ramp and the descent deadline untouched, and log why.
        0.3.23 uses the MEASURED transfer line. 0.3.28 keeps that rule but the
        requirement itself moved: with the linked modulator the ceiling is q = 0.875,
        so 18 V needs 17042 mV instead of 21262 mV.

        The sweep is built from the model rather than hard-coded, so the test asserts the
        PRINCIPLE (refused below the requirement, accepted at and above it, no side effects)
        in whichever coordinate the build drives, and pins the requirement literally."""
        need=required_vin(18000)
        self.assertEqual(need,17042 if LINKED else 21262)
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        self.assertEqual(self.ref(),(5000,5000,6500,0))
        refused=0
        accepted=0
        sweep=sorted({VIN_MIN_MV,need-2,need-1,need,need+1,20000,21000,21261,21262,21500,
                      VIN_MAX_MV})
        for vin in sweep:
            if vin<VIN_MIN_MV or vin>VIN_MAX_MV:
                continue
            for _ in range(30):                                # settle the new VIN
                lib.Test_Sensors(vin,5000,min(250,vin//80))
                lib.Test_Advance(1)
            vmax=allowed_vmax(vin)
            frames=command_frames('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=300)))
            f=next(fr for fr in frames if fr.opcode==CMD['SET_TARGET']
                   and fr.kind in (KIND['ACK'],KIND['NACK']))
            if vmax>=18000:
                accepted+=1
                self.assertEqual(f.kind,KIND['ACK'],vin)
                command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
                self.run_plant(200)
            else:
                refused+=1
                self.assertEqual((f.kind,unpack('error',f.payload)['code']),(KIND['NACK'],ERR['RANGE']),vin)
                message = log_messages(frames)
                self.assertIn('not reachable',message,vin)
                # The message has to carry the numbers, not just the verdict: the
                # target, the measured VIN and the measured-line maximum.
                self.assertIn('SET_TARGET 18000 mV not reachable',message,vin)
                self.assertIn(f'at VIN={vin}',message,vin)
                self.assertIn(f'(allowed {vmax} mV)',message,vin)
                self.assertEqual(self.ref(),(5000,5000,6500,0),vin)
                self.assertEqual(self.status()['fault'],0,vin)
                self.assertEqual(lib.Test_Gates(),0x0f,vin)
        # Both outcomes must actually occur in the sweep, or the loop below would pass
        # vacuously; and the split must be exactly at the requirement.
        self.assertGreater(refused,0)
        self.assertGreater(accepted,0)
        expected_refused=sum(1 for vin in sweep
                             if VIN_MIN_MV<=vin<=VIN_MAX_MV and allowed_vmax(vin)<18000)
        self.assertEqual(refused,expected_refused)
        self.assertEqual(required_vin(18000),need)
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],5000)
    def test_input_sag_outside_the_available_range_latches_after_the_hold_time(self):
        """0.3.22/0.3.23: a target that WAS reachable when accepted must stop the
        stage if the input then sags below the range the measured VIN allows. Own
        timer, INPUT fault, and no automatic resume when VIN recovers.

        Order matters and is stated honestly: the target is accepted while the supply is high,
        the START is then blocked because the supply has sagged, and the same sag later latches
        the INPUT fault while running. The thresholds come from the model, so the test states
        the same physics in either modulation coordinate."""
        need=required_vin(18000)                               # 17042 linked / 21262 legacy
        low=need-1                                             # 1 mV short of the requirement
        high=min(need+200,VIN_MAX_MV)                          # comfortably inside it
        self.assertLess(allowed_vmax(low),18000)
        self.assertGreaterEqual(allowed_vmax(high),18000)
        # The target is only settable while the supply is high enough.
        self.confirm()                                         # mapping verified, VIN high
        for _ in range(30):
            lib.Test_Sensors(high,0,0)
            lib.Test_Advance(1)
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1000)))
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],18000)
        # Now sag the supply to 1 mV short: START must be refused, and the log must say what
        # the input allows AND what the target needs, so the operator knows how short it is.
        for _ in range(30):
            lib.Test_Sensors(low,0,0)
            lib.Test_Advance(1)
        frames = command_frames('OUTPUT',1,pack('output',dict(enabled=1)))
        blocked = log_messages(frames)
        self.assertEqual(self.start().kind,KIND['NACK'])
        self.assertIn('START BLOCKED',blocked)
        self.assertIn(f'tgt 18000 > max {allowed_vmax(low)} mV at VIN {low}',blocked)
        self.assertIn(f'needs {need}',blocked)
        for _ in range(5):
            lib.Test_Sensors(high,0,0)
            lib.Test_Advance(1)
        self.assertEqual(self.start().kind,KIND['ACK'])        # reachable again at `high`
        self.run_plant(1500,vin=high)
        self.assertEqual(self.status()['vref_mv'],18000)
        self.assertEqual(self.status()['fault'],0)
        # VIN sags to `low`: the allowed range falls below the accepted target.
        self.assertLess(allowed_vmax(low),18000)
        for _ in range(VIN_RANGE_HOLD_MS-1):                   # shorter than the hold time
            lib.Test_Sensors(low,17500,875)
            lib.Test_Advance(1)
        self.assertEqual(self.status()['fault'],0)             # not latched yet
        for _ in range(3):
            lib.Test_Sensors(low,17500,875)
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual(st['fault']&64,64)                    # INPUT fault latched
        self.assertEqual(st['state'],8)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
        for _ in range(2000):                                  # recovery must NOT restart
            lib.Test_Sensors(21500,17500,875)
            lib.Test_Advance(1)
        self.assertEqual(self.status()['state'],8)
        self.assertEqual(lib.Test_Gates(),0)
    def test_ideal_plant_and_measured_model_differ_by_the_dead_time_and_skew(self):
        """The old VOUT = VIN*d/0.75 model is NOT what the bench shows. Both are
        approximations (no losses, no reverse conduction, no loop delay), but the
        difference is exactly why the feed-forward and the available-range model
        use PD_MODEL_DT_COUNTS - so pin it instead of letting it drift. Passing
        this test is NOT evidence that 18 V is stable on hardware."""
        self.assertAlmostEqual(self.designed_duty(5000,ideal=True),0.1875,delta=0.001)
        self.assertAlmostEqual(self.designed_duty(5000),D_MODEL+0.25*(D_B-D_MODEL),delta=0.001)
        self.assertGreater(self.designed_duty(5000),self.designed_duty(5000,ideal=True))
        # 18 V at 19.8 V in - the worst input the model still allows - needs a duty
        # below the ceiling but above the ideal-model figure.
        self.assertLess(needed_da_counts(18000,19800),A_MAX)
        self.assertGreater(needed_da_counts(18000,19800),int(18000*D_B/19800))
        # ... and the same output through the IDEAL plant would need less duty,
        # which is precisely the trap: sizing the ceiling from the ideal model
        # would leave no margin at all.
        self.assertGreater(needed_da_counts(18000,19800),int(18000*D_B/19800)+MODEL_DT_COUNTS)
    def test_available_range_matches_the_bench_measurement(self):
        """PD_PowerAllowedVmaxMv() is the single source of truth for START, the
        runtime SET_TARGET check and the sag supervisor. 0.3.23 replaced the
        topology model with the MEASURED transfer line (VOUT_max = 0.856*VIN,
        200 mV margin); the topology model stays as an upper bound only. It must
        return 0 (not a clamp) when there is no usable range.

        0.3.28 re-expressed the same line in the coordinate the loop drives. With the linked
        modulator the ceiling is q = 0.875 instead of A = 0.70, so the identical measurement
        predicts 1.068*VIN - which is what makes 18 V reachable from a ~17.0 V supply instead of
        the 21.26 V the A-only ceiling needed. Both coefficients are asserted here, together with
        the derivation between them, so neither can drift away from the fit it came from."""
        lib.PD_PowerAllowedVmaxMv.argtypes=[C.c_int32]
        lib.PD_PowerAllowedVmaxMv.restype=C.c_int32
        lib.PD_PowerRequiredVinMv.argtypes=[C.c_int32]
        lib.PD_PowerRequiredVinMv.restype=C.c_int32
        # The two coefficients are one measurement seen from two coordinates:
        #   gain_linked = gain_legacy * q_max/A_max = gain_legacy * 0.875/0.70
        # 856 * 1.25 = 1070; the shipped 1068 uses the LOWER of the two measured k_v values
        # (0.916 at 16 V rather than 0.917 at the ceiling), which is the conservative choice.
        self.assertEqual(GAIN_LEGACY,856)
        self.assertEqual(GAIN_LINKED,1068)
        self.assertLessEqual(abs(GAIN_LINKED-round(GAIN_LEGACY*0.875/0.70)),4)
        self.assertEqual(GAIN_NUM,GAIN_LINKED if LINKED else GAIN_LEGACY)
        # PD_BUCK_VMAX_MV is 20000 since 0.3.31, so the MEASURED line does the clipping.
        table_linked=((16999,None),(17000,17956),(18000,19024),(19000,20000),(19800,20000),
                      (20000,20000),(23001,None))
        table_legacy=((16999,None),(17000,14352),(19000,16064),(19500,16492),(19800,16748),
                      (20000,16920),(20200,17091),(21000,17776),(21261,17999),
                      (21262,18000),(21500,18000),(23000,18000),(23001,None))
        for vin,expect in (table_linked if LINKED else table_legacy):
            if expect is None:
                self.assertEqual(lib.PD_PowerAllowedVmaxMv(vin),0,vin)
            else:
                self.assertEqual(lib.PD_PowerAllowedVmaxMv(vin),expect,vin)
            self.assertEqual(allowed_vmax(vin),lib.PD_PowerAllowedVmaxMv(vin),vin)
        self.assertEqual(lib.PD_PowerAllowedVmaxMv(0),0)
        self.assertEqual(lib.PD_PowerAllowedVmaxMv(-5000),0)
        # Required input per target - the table the operator needs. The boundary
        # check only has meaning for targets whose requirement falls INSIDE the input window
        # (PD_VIN_MIN_MV..PD_VIN_MAX_MV); below the window the target is reachable everywhere, and
        # PD_PowerAllowedVmaxMv() answers 0 outside the window rather than a clamp.
        required_linked={16000:15169,17000:16105,18000:17042}
        required_legacy={16000:18926,17000:20094,18000:21262}
        for vout,vin in (required_linked if LINKED else required_legacy).items():
            self.assertEqual(lib.PD_PowerRequiredVinMv(vout),vin,vout)
            self.assertEqual(required_vin(vout),vin,vout)
            if vin<VIN_MIN_MV:
                self.assertGreaterEqual(lib.PD_PowerAllowedVmaxMv(VIN_MIN_MV),vout,vout)
                self.assertEqual(lib.PD_PowerAllowedVmaxMv(vin),0,
                                 'outside the input window the answer is 0, never a clamp')
                continue
            self.assertGreaterEqual(lib.PD_PowerAllowedVmaxMv(vin),vout,vout)
            self.assertLess(lib.PD_PowerAllowedVmaxMv(vin-1),vout,vout)
        # 24 V is refused because it is outside the DECLARED range (PD_BUCK_VMAX_MV = 18 V),
        # which SET_TARGET checks before the transfer model is consulted. In the linked
        # coordinate the transfer line would allow 24 V inside the input window, so the binding
        # limit has moved from the stage to the declared range - see
        # test_declared_output_range_is_reachable_inside_the_input_window for what that means
        # for the 24 V milestone (a ROADMAP decision, not taken here).
        # The measured line may never be more permissive than the duty ceiling.
        for vin in range(VIN_MIN_MV,VIN_MAX_MV+1,97):
            self.assertLessEqual(allowed_vmax(vin),max(structural_vmax(vin),0),vin)
            # ... and the two models must not drift apart without bound: the topology bound is
            # allowed to be optimistic (it is what 0.3.22 over-promised by 8 %), but a large gap
            # means the ceiling or the gain is wrong. Compared against the UNCAPPED measured line,
            # because PD_BUCK_VMAX_MV clips it at 20 V by design. The old form of this check
            # pinned the structural bound itself below 23000 mV, which stopped being meaningful
            # the moment the ceiling became q_max: the stage can now reach past the declared
            # range, and the declared range - not the topology - is what refuses it.
            uncapped=vin*GAIN_NUM//GAIN_DEN-RANGE_MARGIN_MV
            self.assertLessEqual(structural_vmax(vin)-uncapped,6000,vin)
    def test_linked_modulator_regulates_18v_from_a_20v_supply(self):
        """THE headline of 0.3.28, as a regression.

        The bench constraint is a supply that must stay at or below ~20.0 V, and the target is
        18 V. The old modulator could not do it: the A ceiling is 0.70 and the measured line
        needs ~0.742 at 19.83 V, which is why 18 V used to latch PD_FAULTS_INTERNAL at 20 V and
        only regulated from ~21.3 V. With both bridges schedulable the same 18 V lands in the
        transition zone, B comes off its 75 % stop, and the stage regulates with the supply at
        19.9 V - inside the constraint.

        What this test CANNOT show: whether the near-equal-duty pair is safe on hardware. It
        exercises the control law and the distributor against the transfer model, nothing else.
        The waveform questions (edge separation at A ~ B, body-diode freewheeling, the ADC
        sampling instant versus the moving B edge) belong to the scope."""
        if not LINKED:
            self.skipTest('legacy modulator: this operating point is out of reach by design')
        vin=19900                                                # <= 20.0 V, as the bench allows
        lib.Test_Sensors(vin,0,0)
        # 1500 mA of limit into 20 ohm: 18 V draws 900 mA, so the CC fold-back stays out of the
        # way. With the 500 mA this test first used the CC loop correctly held the output at
        # 10 V (500 mA * 20 ohm), i.e. the assertion below would have measured the current
        # limit instead of the modulator.
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1500)))
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],18000)
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(4000,vin=vin,r_load=20.0)
        settled=self.status()['vout_mv']
        self.assertLessEqual(abs(settled-18000),120,
                             f'VOUT {settled} mV did not settle on 18000 mV at VIN={vin} mV')
        self.assertEqual(self.status()['fault'],0)
        a,b=self.compare()
        # A stays inside the A-bridge ceiling, and B is off its 75 % stop but above the
        # experimental floor - i.e. the whole point of the release.
        self.assertLessEqual(a,A_MAX)
        self.assertLess(b,B_CMP,f'B {b} is still pinned at 75 %: the transition zone was not used')
        self.assertGreaterEqual(b,config_int('PD_MOD_B_MIN_COUNTS'))
        self.assertGreater(a,config_int('PD_MOD_Q_LOW_COUNTS'),
                           'A below the 0.65 zone end means q never reached the transition zone')
        # The pair must satisfy the ratio relation the whole scheme rests on.
        q_applied=lib.PD_ModAppliedQ(a,b)
        self.assertAlmostEqual(q_applied,0.75*a/(b/self.PER),delta=1)
        # ... and the trace must say the same thing, with nothing limited.
        lib.Test_CtrlTerms.argtypes=[C.POINTER(C.c_int32)]*9+[C.POINTER(C.c_uint32)]*3
        lib.Test_CtrlTerms.restype=None
        values=[C.c_int32() for _ in range(9)]
        mods=[C.c_uint32() for _ in range(3)]
        lib.Test_CtrlTerms(*[C.byref(v) for v in values],*[C.byref(m) for m in mods])
        q_cnt=values[5].value
        cmd_cnt=values[6].value
        q_app_cnt=values[7].value
        cmp_out_cnt=values[8].value
        mod_limit=mods[0].value
        self.assertEqual((cmd_cnt,cmp_out_cnt),(a,b),'trace and registers disagree')
        self.assertEqual(q_app_cnt,q_applied)
        self.assertEqual(mod_limit,0,f'lim=0x{mod_limit:02X} at a point inside the range')
        self.assertGreater(q_cnt,Q_LOW_CMP,'q should be in the transition zone here')
        self.assertLess(q_cnt,Q_MAX_CMP)
        self.assertAlmostEqual(q_cnt,q_app_cnt,delta=2,msg='requested and applied q disagree')
    def test_saturation_reason_agrees_with_the_distributor_limit_bits(self):
        """THE 0.3.32 regression, from the 20 V bench log.

        That run reported `a=15232 qr=17795 q=17796 b=13969 lim=0x00 s=2`: nothing was clamped,
        yet the saturation reason said "floor engaged", because the delivered q came back one
        count ABOVE the request from PD_ModAppliedQ()'s rounding - and the old flag read any
        difference as a bound. A flag that is permanently lit cannot report anything.

        Two halves, and both are needed: sat must be 0 at a point inside the range even though
        the one-count difference is still there, and sat must still become 1 when the q ceiling
        really is reached.
        """
        if not LINKED:
            self.skipTest('legacy modulator has no distributor')
        lib.Test_CtrlTerms.argtypes=[C.POINTER(C.c_int32)]*9+[C.POINTER(C.c_uint32)]*3
        lib.Test_CtrlTerms.restype=None
        lib.PD_PowerAtRangeEnd.argtypes=[]
        lib.PD_PowerAtRangeEnd.restype=C.c_uint8
        lib.Test_Sensors(19800,0,0)
        # VIN = 19.8 V for an 18 V target sits in the transition zone with everything in range
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1500)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(3000,vin=19800,r_load=20.0)
        values=[C.c_int32() for _ in range(9)]
        mods=[C.c_uint32() for _ in range(3)]
        lib.Test_CtrlTerms(*[C.byref(v) for v in values],*[C.byref(m) for m in mods])
        q_cnt=values[5].value
        q_app=values[7].value
        mod_limit=mods[0].value
        sat=mods[2].value
        self.assertEqual(mod_limit,0,f'lim=0x{mod_limit:02X}: this point must be in range')
        self.assertEqual(sat,0,
                         f'sat={sat} with lim=0x00 (qr={q_cnt} q={q_app}): the flag must not '
                         f'report a bound the distributor did not hit')
        # ... and the other direction: the same weak plant against a target that needs the top
        # of the range, so q really does run out.
        command('SET_TARGET',1,pack('target',dict(vref_mv=20000,iref_ma=1500)))
        self.run_plant(5000,vin=19800,r_load=20.0,plant_scale=0.85)
        values=[C.c_int32() for _ in range(9)]
        mods=[C.c_uint32() for _ in range(3)]
        lib.Test_CtrlTerms(*[C.byref(v) for v in values],*[C.byref(m) for m in mods])
        sat=mods[2].value
        self.assertEqual(lib.PD_PowerAtRangeEnd(),1,
                         'the plant is too weak for this target: q must be exhausted')
        self.assertEqual(sat,1,
                         f'sat={sat}: lim says the ceiling is engaged, so sat must be 1')
    def test_adc_edge_proximity_is_reported_not_acted_on(self):
        """Timer E samples both ADC groups at 50 % of every switching period, and the A bridge's
        falling edge sweeps through that instant as the target moves - A = 50 % lands on it
        exactly. Nothing in the firmware noticed before 0.3.32; this pins what the predicate
        counts as "near" and, just as importantly, that it is purely a report.

        The window is +/-PD_ADC_EDGE_WATCH_COUNTS around PD_TE_CMP3 % PD_SW_PERIOD.
        """
        lib.PD_PowerAdcEdgeNear.argtypes=[C.c_uint16,C.c_uint16,C.POINTER(C.c_int32)]
        lib.PD_PowerAdcEdgeNear.restype=C.c_uint8
        sample=config_int('PD_TE_CMP3') % config_int('PD_SW_PERIOD')
        watch=config_int('PD_ADC_EDGE_WATCH_COUNTS')
        dist=C.c_int32()
        # A sitting exactly on the sampling instant is the worst case: both A and B reported
        # near only if B is also close, so keep B far away to isolate it.
        far_b=config_int('PD_OUT_BRIDGE_CMP')
        self.assertEqual(lib.PD_PowerAdcEdgeNear(sample,far_b,C.byref(dist)),0x1)
        self.assertEqual(dist.value,0)
        # just outside the window on the late side, and well clear on the early side
        self.assertEqual(lib.PD_PowerAdcEdgeNear(sample+watch+40,far_b,C.byref(dist)),0)
        self.assertEqual(lib.PD_PowerAdcEdgeNear(sample-watch-40,far_b,C.byref(dist)),0)
        # a B edge inside the window is reported as B (0x2), and both together as 0x3
        self.assertEqual(lib.PD_PowerAdcEdgeNear(far_b,sample,C.byref(dist)),0x2)
        self.assertEqual(lib.PD_PowerAdcEdgeNear(sample,sample,C.byref(dist)),0x3)
        # the signed distance describes the closest edge
        lib.PD_PowerAdcEdgeNear(sample+100,far_b,C.byref(dist))
        self.assertEqual(dist.value,100)
        lib.PD_PowerAdcEdgeNear(sample-100,far_b,C.byref(dist))
        self.assertEqual(dist.value,-100)
        # the operating point the bench actually runs must NOT be reported as near, or the
        # line would be permanent and useless
        for a_cmp,b_cmp in ((4423,16320),(13416,16320),(15003,15980),(15232,15247)):
            self.assertEqual(lib.PD_PowerAdcEdgeNear(a_cmp,b_cmp,C.byref(dist)),0,
                             f'A={a_cmp} B={b_cmp} is a verified bench point and must be clear')
    def test_end_of_range_is_not_the_a_ceiling_in_linked_mode(self):
        """THE 0.3.29 regression, taken straight from the 2026-09-13 bench run.

        At 18 V out of 19.82 V in the registers held A = 15232 (the A-bridge ceiling exactly)
        with B = 15247: the loop was NOT out of range - it still had q = 16311 of a possible
        19040, i.e. 2186 counts of B travel. The old status flag called that "saturated"
        because it only looked at the A compare value, so the operator would have concluded the
        new modulator had not helped. PD_PowerAtRangeEnd() is now the single definition and it
        must stay FALSE here.

        The second half shows the other side: with the plant weaker than the loop believes, q
        really does run out and the flag must then be TRUE. Without that direction the test
        would pass on a flag that is simply hard-wired to zero."""
        if not LINKED:
            self.skipTest('legacy modulator: A at the ceiling IS the end of range')
        lib.PD_PowerAtRangeEnd.argtypes=[]
        lib.PD_PowerAtRangeEnd.restype=C.c_uint8
        # VIN 18.03 V for an 18 V target puts q at the very top of the transition zone, where
        # the polynomial's own endpoint is the A ceiling - no clamping involved.
        lib.Test_Sensors(18030,0,0)
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1500)))
        self.assertEqual(unpack('telemetry',command('STATUS',1).payload)['vref_mv'],18000)
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(4000,vin=18030,r_load=20.0)
        a,b=self.compare()
        self.assertEqual(a,A_MAX,'this point should sit exactly on the A-bridge ceiling')
        self.assertGreater(b,config_int('PD_MOD_B_MIN_COUNTS'),
                           'B must still have travel left, or the test proves nothing')
        self.assertEqual(lib.PD_PowerAtRangeEnd(),0,
                         f'A = {a} (ceiling) and B = {b}: the pair is NOT at the end of its '
                         f'released range, so the flag must be 0')
        self.assertEqual(self.status()['fault'],0)
        # ... and the opposite direction: q exhausted against a plant that delivers less than
        # the model assumes. VOUT settles inside the regulation-failure margin, so this is a
        # limit, not a fault.
        lib.Test_Sensors(18030,0,0)
        self.run_plant(4000,vin=18030,r_load=20.0,plant_scale=0.85)
        self.assertEqual(lib.PD_PowerAtRangeEnd(),1,
                         'q ran out against a weak plant: the flag must be 1')
    def test_the_measured_transfer_line_explains_the_bench_18v_trip(self):
        """The bench evidence this version is built on (do not delete as
        'unnecessary'): with the supply at 20 V the 70 % ceiling cannot deliver
        18 V - the loop saturated the A-bridge at 15232, VOUT stopped at 16.96 V
        and the regulation-failure backstop latched PD_FAULTS_INTERNAL after
        500 ms. Raising the input to 21 V made 18 V regulate (17.891 V measured,
        DMM 17.82 V)."""
        # At 19.83 V input the measured line needs ~74.2 % duty for 18 V.
        need_20v=measured_duty_for(18000,19827)
        self.assertGreater(need_20v,A_MAX)
        self.assertAlmostEqual(need_20v,16155,delta=60)
        self.assertGreater(need_20v/PER,0.73)
        # At 21.0 V the same target is still MARGINAL: it needs 15252 counts against
        # the 15232 ceiling, i.e. about 20 counts too many. The bench agrees - 21 V
        # gave 17.891 V (DMM 17.82 V), 109 mV short of the setpoint.
        need_21v=measured_duty_for(18000,21000)
        self.assertGreater(need_21v,A_MAX-60)
        self.assertAlmostEqual(need_21v,15253,delta=80)
        # 21.5 V is the first supply with real headroom under the 70 % ceiling.
        need_215v=measured_duty_for(18000,21500)
        self.assertLess(need_215v,A_MAX-200)
        self.assertAlmostEqual(need_215v,14897,delta=80)
        # The linked modulator turns those counts into a reachability statement: the 19.83 V
        # point needed 16155 counts against a 15232 ceiling, and against the 19040 q ceiling it
        # fits - inside the TRANSITION zone, i.e. exactly where the distributor starts moving B.
        # Realised there: A = 15225 (69.97 %), B = 15381 (70.74 %), so the two bridges land
        # within 1 % of each other. That is the waveform case phase B must inspect on the scope
        # (near-equal duties, both bridges switching almost together) before this counts as
        # verified - the host tests cannot see shoot-through or edge separation.
        self.assertGreater(need_20v,A_MAX)
        self.assertLess(need_20v,Q_MAX_CMP)
        self.assertLess(Q_LOW_CMP,need_20v)                    # transition zone, not the low one
        self.assertLess(need_20v,Q_HIGH_CMP)
        # ... and the measured maximum at 21 V explains the achieved 17.891 V (legacy build).
        vmax_21v=21000*GAIN_LEGACY//GAIN_DEN
        self.assertAlmostEqual(vmax_21v,17976,delta=40)
        self.assertLessEqual(17891,vmax_21v+60)
        # The declared range must stay reachable at the top of the input window.
        self.assertGreaterEqual(VIN_MAX_MV*GAIN_NUM//GAIN_DEN,BUCK_VMAX)
    def test_declared_output_range_is_reachable_inside_the_input_window(self):
        """Guards the ROADMAP, not just this version: the declared maximum output
        must be reachable with the best input voltage the window allows, under the
        MEASURED transfer and the ACTIVE modulation ceiling. Declaring 2..24 V with
        a 17..23 V input window would FAIL here (and in the configuration checker)
        until the input window widens or the B duty is scheduled further."""
        reachable=VIN_MAX_MV*GAIN_NUM//GAIN_DEN
        self.assertGreaterEqual(reachable,BUCK_VMAX,
                                f'declared {BUCK_VMAX} mV not reachable at VIN={VIN_MAX_MV} mV '
                                f'(measured limit {reachable} mV)')
        # What the linked modulator DID change: 24 V is no longer out of reach for the STAGE.
        # At the top of the input window the transfer line allows it (q = 0.78, i.e. B at 67 %,
        # inside the phase-B floor), so the limit on the 24 V milestone has moved from the
        # converter to the DECLARED range and the OVP ceiling. Widening PD_BUCK_VMAX_MV is a
        # ROADMAP decision that needs its own bench evidence and is NOT taken here; the build
        # 0.3.31 took the first step of that (18 -> 20 V) to open the boost region; 24 V
        # would be the next one, with its own evidence.
        need_vin=required_vin(24000)
        if LINKED:
            self.assertLess(need_vin,VIN_MAX_MV,
                            'with the q ceiling the transfer line reaches 24 V inside the input '
                            'window - the declared range is what refuses it now')
        else:
            self.assertGreater(need_vin,VIN_MAX_MV)
        self.assertEqual(BUCK_VMAX,20000, 'the declared top moved to 20 V in 0.3.31')
    def test_ovp_threshold_follows_the_reference_during_a_down_transition(self):
        """While the reference is still coming down, the guard is max(target,ramp)
        = the RAMP, so the threshold descends with the reference instead of
        collapsing to the new target's threshold one tick after the command."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        self.assertEqual(self.ref(),(5000,5000,6500,0))
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        lib.Test_Advance(1)                                   # one control tick
        tgt,ramp,ovp,down=self.ref()
        self.assertEqual((tgt,down),(2000,1))
        self.assertEqual(ramp,5000-RUN_RAMP_DOWN)
        self.assertEqual(ovp,tracked_ovp(2000,ramp))
        self.assertGreater(ovp,2000+OVP_MARGIN)               # NOT the new target's threshold
        self.assertGreater(ovp,3500)
        # Let the real plant follow the reference down with a 20 ohm load.
        self.run_plant(600)
        text=self.log_text(24)                                 # drain before any command
        self.assertIn('DOWN done',text)
        self.assertEqual(self.status()['fault'],0)
        tgt,ramp,ovp,down=self.ref()
        self.assertEqual((tgt,ramp,ovp,down),(2000,2000,3500,0))
        # The CC fold-back may still be active after the descent (the loop lags
        # the ramp, so the load current exceeded the limit on the way down);
        # RUN_CV and RUN_CC are both "running".
        self.assertIn(self.status()['state'],(5,6))
    def test_ovp_during_a_down_transition_is_logged_with_its_context(self):
        """A descent the output cannot follow is stopped by the shrinking
        threshold first; the log must say that this happened DURING a descent and
        must name target, reference and the threshold in force."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        for _ in range(200):                                   # output stuck at 5 V
            lib.Test_Sensors(20000,5000,250)
            lib.Test_Advance(1)
        text=self.log_text(16)                                 # drain before any command
        st=self.status()
        self.assertEqual(st['fault']&4,4)
        self.assertEqual(st['state'],8)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertIn('OVP:',text)
        self.assertIn('dn=1',text)
        self.assertIn('tgt=2000',text)
    def test_down_transition_timeout_stops_and_reports_residual_voltage(self):
        """The output sits inside the OVP band but outside the completion band:
        no protection trips, the reference cannot be reached, so the descent
        supervisor stops the stage after its budget and says so."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        budget=down_budget(5000-2000)
        self.assertGreaterEqual(budget,DOWN_TIMEOUT_MIN_MS)    # budget, not a flat 10 ms
        for _ in range(budget//2):                             # stuck 1 V above the target
            lib.Test_Sensors(20000,3000,0)
            lib.Test_Advance(1)
        self.assertEqual(self.status()['fault'],0)             # still supervised, nothing tripped
        self.assertEqual(self.ref()[3],1)
        for _ in range(budget//2+200):
            lib.Test_Sensors(20000,3000,0)
            lib.Test_Advance(1)
        text=self.log_text(24)                                 # drain before any command
        st=self.status()
        self.assertEqual(st['fault']&128,128)                  # internal: not a false OVP
        self.assertEqual(st['state'],8)
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
        self.assertIn('DOWN timeout',text)
        self.assertIn('residual V stays',text)
    def test_repeating_the_same_target_does_not_extend_the_down_deadline(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        budget=down_budget(5000-2000)
        for t in range(budget+400):
            if t%500==0:                                       # same value over and over
                command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
            lib.Test_Sensors(20000,3000,0)
            lib.Test_Advance(1)
        text=self.log_text(24)
        self.assertEqual(self.status()['fault']&128,128)       # deadline was NOT pushed out
        self.assertIn('DOWN timeout',text)
    def test_measured_output_never_raises_the_reference(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(800)
        self.assertEqual(self.ref(),(2000,2000,3500,0))
        for vout in (2200,2400,2000,1500,1700):                # measured excursions
            for _ in range(20):
                lib.Test_Sensors(20000,vout,100)
                lib.Test_Advance(1)
            self.assertEqual(self.ref(),(2000,2000,3500,0),vout)
            self.assertEqual(self.status()['fault'],0,vout)
    def test_ovp_absolute_ceiling_is_never_exceeded(self):
        """The tracking threshold may not go above the highest value this firmware
        ever used, whatever the reference does."""
        # 0.3.23: 18 V needs >= 21262 mV of input, so the supply must be at 21.5 V
        # before SET_TARGET/START or the new range check refuses the point.
        for _ in range(5):
            lib.Test_Sensors(21500,0,0)
            lib.Test_Advance(1)
        command('SET_TARGET',1,pack('target',dict(vref_mv=BUCK_VMAX,iref_ma=1000)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        for _ in range(500):                                   # whole soft-start ramp
            self.run_plant(1,r_load=20.0,vin=21500)
            tgt,ramp,ovp,down=self.ref()
            self.assertEqual(tgt,BUCK_VMAX)
            self.assertEqual(ovp,tracked_ovp(tgt,ramp))
            self.assertLessEqual(ovp,OVP_ABS_MAX)
        self.assertEqual(self.ref()[2],OVP_ABS_MAX)
        self.assertEqual(self.status()['state'],5)
        self.assertEqual(self.status()['fault'],0)
    def test_eighteen_volt_operating_point_and_descent(self):
        """The point of 0.3.22: 18 V must regulate inside the new ceiling, and the
        descent 18 -> 2 V must not trip OVP on the NEW target's threshold (the
        0.3.20 tracking rule, which matters much more over a 9:1 ratio)."""
        self.assertEqual(BUCK_VMAX,20000, 'the declared top moved to 20 V in 0.3.31')
        self.assertEqual(OVP_ABS_MAX,23000)   # 20 V declared x 1.15, raised with the range
        # ...and still far below the components across VOUT (100 V, see pd_config.h).
        # The bench operating point this version certifies: 18 V needs >= 21262 mV
        # of input (measured line + 200 mV). At 20 V the loop saturates against the
        # 70 % ceiling; at 21.5 V it keeps ~330 counts of headroom.
        for _ in range(5):
            lib.Test_Sensors(21500,0,0)
            lib.Test_Advance(1)
        command('SET_TARGET',1,pack('target',dict(vref_mv=18000,iref_ma=1000)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500,vin=21500)
        st=self.status()
        self.assertIn(st['state'],(5,6))
        self.assertEqual(st['fault'],0)
        self.assertAlmostEqual(st['vout_mv']/1000.0,18.0,delta=0.1)
        self.assertEqual(self.compare()[1],B_CMP)              # output bridge still 75 %
        self.assertLessEqual(self.compare()[0],A_MAX)          # A bridge inside the ceiling
        tgt,ramp,ovp,down=self.ref()
        self.assertEqual((tgt,ramp),(18000,18000))
        # The threshold IN FORCE is the tracking value, not the absolute ceiling: at an
        # 18 V target that is max(18000*1.15, 18000+1500) = 20700. The two coincided while
        # the declared top was 18 V (ceiling 18000*1.15 = 20700); 0.3.31 separated them,
        # and this test now pins both.
        self.assertEqual(ovp,tracked_ovp(18000,18000))
        self.assertLessEqual(ovp,OVP_ABS_MAX)
        self.assertEqual(OVP_ABS_MAX,23000)                    # 20000*1.15
        self.assertLess(OVP_ABS_MAX,PD_OUT_CAP_RATED_MV * 85 // 100)
        # Descent to 2 V: the threshold must follow the executed ramp.
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=1000)))
        lib.Test_Advance(1)
        tgt,ramp,ovp,down=self.ref()
        self.assertEqual((tgt,down),(2000,1))
        self.assertEqual(ovp,tracked_ovp(2000,ramp))
        self.assertGreater(ovp,18000)                          # still tracking the ramp
        self.run_plant(1200,vin=21500)
        st=self.status()
        self.assertEqual(st['fault'],0)
        self.assertAlmostEqual(st['vout_mv']/1000.0,2.0,delta=0.1)
        self.assertEqual(lib.Test_CompareIllegal(),0)
    def test_stop_ramp_keeps_the_target_threshold_and_never_writes_illegal_compares(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        self.assertEqual(command('OUTPUT',1,pack('output',dict(enabled=0))).kind,KIND['ACK'])
        for _ in range(40):                                    # during the stop ramp
            lib.Test_Advance(1)
            tgt,ramp,ovp,down=self.ref()
            self.assertEqual((tgt,down),(5000,0))              # descent supervision is off
            self.assertEqual(ovp,tracked_ovp(tgt,ramp))        # guard floor is the target
            self.assertGreaterEqual(ovp,5000+OVP_MARGIN)
        for _ in range(300):
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual((st['state'],st['enabled'],st['fault']),(2,0,0))
        self.assertEqual(lib.Test_Gates(),0)
        self.assertEqual(lib.Test_CompareIllegal(),0)
    def test_stop_ramps_down_then_disables(self):
        command('SET_TARGET',1,pack('target',dict(vref_mv=5000,iref_ma=300)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        self.assertEqual(self.status()['state'],5)
        self.assertEqual(command('OUTPUT',1,pack('output',dict(enabled=0))).kind,KIND['ACK'])
        self.assertEqual(self.status()['state'],7)                 # STOPPING
        for _ in range(200):
            lib.Test_Advance(1)
        st=self.status()
        self.assertEqual((st['state'],st['enabled']),(2,0))
        self.assertEqual(lib.Test_PowerEnable(),0)
        self.assertEqual(lib.Test_Gates(),0)
    def test_cc_deadband_sits_below_the_limit_at_the_boundary_operating_point(self):
        """Documents a behaviour the bench operator WILL meet: with a 20 ohm load
        at 2 V the load current is exactly 100 mA, so a 100 mA limit puts the
        operating point on the CC boundary. The symmetric 10 mA dead band then
        holds the output about 10 mA - 0.2 V into 20 ohm - low and the state reads
        CC. Setting the limit to 300 mA gives a clean CV run, which is why the
        bench procedure sets 0.1 A as the LIMIT but expects the current the load
        actually draws."""
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=100)))
        self.confirm()
        self.assertEqual(self.start().kind,KIND['ACK'])
        self.run_plant(1500)
        st=self.status()
        self.assertIn(st['state'],(5,6))
        if st['state']==6:
            self.assertLess(st['vout_mv'],2000)
            self.assertGreater(st['vout_mv'],1780)
        command('SET_TARGET',1,pack('target',dict(vref_mv=2000,iref_ma=300)))
        self.run_plant(900)
        self.assertEqual(self.status()['state'],5)
        self.assertAlmostEqual(self.status()['vout_mv']/1000.0,2.0,delta=0.05)

class SchedulingTests(unittest.TestCase):
    """0.3.21: 250 kHz PWM, unchanged duty ceilings, and the Timer E / ADC
    sample-phase invariants.

    The C header enforces these with #error gates; the Python side states the
    same intent so a refactor cannot delete the gates, and a CubeMX change
    without a matching code change fails here too (check_configuration.py
    cross-checks .ioc and hrtim.c against the same numbers).
    """
    def setUp(self):
        self.cfg=(ROOT/'App/pd_config.h').read_text(encoding='utf-8')
    def test_switch_period_is_250khz_and_the_duty_ceiling_relations_are_kept(self):
        period=config_int('PD_SW_PERIOD')
        self.assertEqual(period,21760)                             # 4 us at 170 MHz MUL32
        self.assertEqual(config_float('PD_OUT_BRIDGE_DUTY'),0.75)  # output bridge 75 %
        self.assertEqual(config_float('PD_DUTY_MAX'),0.70)         # input bridge ceiling (was 0.65)
        self.assertEqual(config_float('PD_DUTY_MIN'),0.05)         # unchanged on request
        self.assertEqual(config_int('PD_OUT_BRIDGE_CMP'),
                         int(period*config_float('PD_OUT_BRIDGE_DUTY')+0.5))
        # The integer boundary and the float ceiling must describe the same limit
        # (ratio_cmp rounds with +0.5 before truncating), and the A-bridge edge
        # must stay clear of the output bridge by more than one dead time (the
        # register-level structural constraint).
        self.assertEqual(config_int('PD_CMP_A_MAX'),int(period*config_float('PD_DUTY_MAX')+0.5))
        self.assertLess(config_int('PD_CMP_A_MAX')+config_int('PD_CMP_DT_COUNTS'),
                        config_int('PD_OUT_BRIDGE_CMP'))
        self.assertLessEqual(config_int('PD_CMP_MID_CHANNEL_MAX'),config_int('PD_CMP_A_MAX'))
        # Measured gate edges: the model uses the skew, the register checks do not.
        self.assertEqual(config_int('PD_MODEL_DT_COUNTS'),
                         config_int('PD_CMP_DT_COUNTS')
                         + round((config_int('PD_EDGE_ON_NS')-config_int('PD_EDGE_OFF_NS'))*5.44))
        # The commit margin is a TIME budget (552 ns), so it must not be scaled
        # with the period; the compare floor is set by the HRTIM clock, not by T.
        self.assertEqual(config_int('PD_CMP_COMMIT_MARGIN'),3000)
        self.assertGreaterEqual(config_int('PD_CMP_MIN'),96)
    def test_timer_e_keeps_the_sample_phase_locked_to_the_pwm(self):
        sw,te=config_int('PD_SW_PERIOD'),config_int('PD_TE_PERIOD')
        c2,c3=config_int('PD_TE_CMP2'),config_int('PD_TE_CMP3')
        ratio=config_int('PD_TE_PWM_RATIO')
        self.assertEqual(te,ratio*sw)          # integer ratio: no phase walk
        self.assertEqual(ratio,2)
        self.assertEqual(c3,sw//2)             # regular group: 50 % of a PWM period
        self.assertEqual(c2,te-sw//2)          # injected IL: 50 % of a PWM period
        self.assertEqual(config_int('PD_FAST_HZ'),250000//ratio)   # 125 kHz, integer
        self.assertEqual(250000//ratio%1000,0)                     # host decimation 1000
    def test_adc_conversion_budget_gates_exist_and_hold(self):
        cyc=config_int('PD_ADC_CYCLE_COUNTS')
        reg=config_int('PD_TE_REG_RANK_COUNTS')
        inj=config_int('PD_TE_INJ_RANK_COUNTS')
        lat=config_int('PD_TE_TRIG_LATENCY_COUNTS')
        self.assertEqual(reg,105*cyc)          # NTC rank: 92.5 sample + 12.5 SAR
        self.assertEqual(inj,37*cyc)           # injected IL: 24.5 sample + 12.5 SAR
        sw,te=config_int('PD_SW_PERIOD'),config_int('PD_TE_PERIOD')
        c2,c3=config_int('PD_TE_CMP2'),config_int('PD_TE_CMP3')
        # Both triggers must find the ADC idle, otherwise the sample moves to
        # another point of the ripple.
        self.assertGreater(c2-c3,reg+lat)      # injected waits for no regular rank
        self.assertGreater(c3+te-c2,inj+lat)   # regular waits for no injected rank
        for guard in ('PD_TE_PERIOD != PD_TE_PWM_RATIO * PD_SW_PERIOD',
                      'PD_TE_CMP3 != PD_SW_PERIOD / 2u',
                      'PD_TE_CMP2 != PD_TE_PERIOD - PD_SW_PERIOD / 2u',
                      'PD_TE_REG_RANK_COUNTS != 105u * PD_ADC_CYCLE_COUNTS',
                      'PD_TE_INJ_RANK_COUNTS != 37u * PD_ADC_CYCLE_COUNTS',
                      'PD_TE_GAP_REG_TO_INJ < (PD_TE_REG_RANK_COUNTS + PD_TE_TRIG_LATENCY_COUNTS)',
                      'PD_TE_GAP_INJ_TO_REG < (PD_TE_INJ_RANK_COUNTS + PD_TE_TRIG_LATENCY_COUNTS)'):
            self.assertIn(guard,self.cfg)      # the #error gate must exist
    def test_applied_voltage_calibration_reproduces_the_bench_measurement(self):
        """The VOUT calibration in force must be the fit of the reported bench points.

        Bench record (same session, same meter, input 21.5 V, load 20 ohm):
            set 5.000 V  -> firmware telemetry 5.385 V, meter 5.370 V
            set 10.000 V -> firmware telemetry 10.080 V, meter 10.079 V
            set 18.000 V -> firmware telemetry 18.000 V, meter 17.990 V
        The fit over those three points is gain 1000/1000 with a -11 mV offset (the 3285 mV
        reference correction already removes the gain error). The 2 V point is deliberately
        NOT in the fit: its output sits 547 mV above the setpoint (+27 %), a closed-loop
        deviation at minimum duty that no gain/offset can express. It is kept here as an
        out-of-model check instead, which is why it is allowed a larger residual.
        """
        bench=[(5385,5370),(10080,10079),(18000,17990)]      # (firmware, meter) mV, fitted
        out_of_model=(2572,2547)                             # 2 V point, NOT fitted
        gain_num=config_int('PD_VOUT_CAL_GAIN_NUM')
        gain_den=config_int('PD_VOUT_CAL_GAIN_DEN')
        offset=config_int('PD_VOUT_CAL_OFFSET_MV')
        self.assertEqual(config_int('PD_CALIBRATED'),1,
                         'this test certifies the CALIBRATED build')
        self.assertEqual((gain_num,gain_den),(1000,1000),
                         'the bench fit says the gain is unity; a different gain means the '
                         'coefficients were not taken from this record')
        self.assertEqual(offset,-11)
        def correct(firmware):
            return (firmware*gain_num+(gain_den//2))//gain_den+offset
        worst=max(abs(correct(f)-m) for f,m in bench)
        self.assertLessEqual(worst,15,f'worst fitted residual {worst} mV exceeds 15 mV')
        for firmware,meter in bench:
            self.assertLessEqual(abs(correct(firmware)-meter),11,
                                 f'firmware {firmware} -> {correct(firmware)} vs meter {meter}')
        # Out-of-model check: the calibration must still IMPROVE the 2 V point, not break it.
        firmware,meter=out_of_model
        before=abs(firmware-meter)
        after=abs(correct(firmware)-meter)
        self.assertLess(after,before,f'the calibration made 2 V worse: {before} -> {after} mV')
        self.assertLessEqual(after,15,f'2 V residual {after} mV')
        # And the range line must carry a provenance that is consistent with a calibration.
        state=config_int('PD_RANGE_MEASURED_CAL')
        self.assertIn(state,(0,1,2))
        self.assertGreaterEqual(state,1,
                                'PD_CALIBRATED=1 requires the range ratio provenance to be '
                                '1 (re-measured) or 2 (recomputed from the fit)')

    def test_bounded_log_values_are_not_capped_below_their_real_range(self):
        """PD_LOG_VALUE caps a diagnostic value at 999 and prints 1000 for "at or beyond
        the cap". That is right for a retry/timeout/ADC-code counter and WRONG for a
        register count whose real range is larger: the duty pair is 0..PD_SW_PERIOD
        (21760), so wrapping it made every realistic duty print as 1000 and the two
        lines that exist to explain a fault said nothing. The log-length gate cannot
        see this (1000 fits the field perfectly), so the rule is pinned here.

        Both mistakes are rejected: wrapping a duty field, and dividing it to fit the cap
        (a reader should not have to decode a scaling factor to read a duty count)."""
        sys.path.insert(0, str(ROOT / 'tools'))
        import check_log_lengths as checker
        checked = 0
        for relative in checker.SOURCES:
            source = checker.blank_comments((ROOT / relative).read_text(encoding='utf-8'))
            for match in re.finditer(r'snprintf\s*\(', source):
                arguments, pieces = checker.call_arguments(source, match.end() - 1)
                if len(arguments) < 3:
                    continue
                format_string = ''.join(pieces[2])
                # Formats whose duty field is a register count, not a counter.
                if 'D=%u/%u' not in format_string and 'duty=%u/%u' not in format_string:
                    continue
                checked += 1
                duty_arguments = [a.strip() for a in arguments[3:]
                                  if 'duty_in' in a or 'duty_out' in a
                                  or 'cmp_in' in a or 'cmp_out' in a]
                self.assertTrue(duty_arguments, format_string)
                for argument in duty_arguments:
                    self.assertNotIn('PD_LOG_VALUE', argument,
                                     f'{argument} is capped at 999 below its real range')
                    self.assertNotIn('/ 32', argument,
                                     f'{argument} is scaled; a duty count must be printed '
                                     f'as it is')
        self.assertGreaterEqual(checked, 2, 'the duty lines disappeared from the scan')

    def test_health_line_reports_the_real_calibration_state(self):
        """The 5 s health line must state the calibration state the firmware is actually
        compiled with. It used to carry a literal "cal=0", which stayed correct until the
        calibration was applied and then contradicted the CONFIG line printed next to it in the
        same log - the log an operator reads to decide whether the numbers can be trusted."""
        lib.Test_Reset()
        lib.Test_Time(6000)                      # crosses the 5 s health-log interval
        logs=[unpack('log',f.payload)['message'] for f in Parser().feed(read())
              if f.opcode==CMD['LOG']]
        # The 5 s health line, found by content it guarantees. 'restricted buck' was dropped from
        # this line in 0.3.40 for width; the BOOT banner still carries those words.
        health=[m for m in logs if 'ref ' in m and 'cal=' in m]
        self.assertTrue(health,f'no health line in {len(logs)} frames')
        expected=f'cal={config_int("PD_CALIBRATED")}'
        self.assertIn(expected,health[0],
                      f'health line says something else: {health[0]}')
        # and it must not contain a stale literal for any other calibration value
        for stale in ('cal=0','cal=1'):
            if stale!=expected:
                self.assertNotIn(stale,health[0],
                                 f'health line still carries the literal {stale}')

    def test_log_lines_fit_the_protocol_field(self):
        """0.3.22 shipped a code-17 status line and a code-9 CONFIG line LONGER than
        the 96-byte log text field, so they were truncated silently on the wire and
        the operator's log contained no usable range/saturation data. 0.3.23 bounds
        every formatted line; tools/check_log_lengths.py recomputes the worst case
        from the source and this test runs it, so the promise cannot rot the way a
        hand count does."""
        checker = ROOT / 'tools/check_log_lengths.py'
        result = subprocess.run([sys.executable, str(checker)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('0 over the limit', result.stdout)
    def test_info_reports_the_header_version(self):
        lib.Test_Reset()
        f=command('INFO')
        self.assertEqual(f.kind,KIND['ACK'])
        self.assertEqual(unpack('info',f.payload)['firmware'].split('\0')[0],FW_VERSION)
        # Lineage-agnostic on purpose: every release would otherwise need this line edited.
        # What matters is that the reported string is well formed and carries a revision tag.
        self.assertRegex(FW_VERSION,r'^G474-BUCK-\d+\.\d+\.\d+-[A-Z0-9]+$')
    def test_voltage_conversion_is_pure_monotonic_and_channel_separated(self):
        """0.3.22 voltage calibration: VIN/VOUT use the measured 3.285 V reference
        with 64-bit intermediates and round-to-nearest; the CURRENT channels keep
        the old 3.300 V basis and the pd_zero.c zero store. Control, telemetry and
        OVP all consume this one result, so the numbers are pinned here."""
        lib.PD_ConvertVinMv.argtypes=[C.c_uint16]
        lib.PD_ConvertVinMv.restype=C.c_int32
        lib.PD_ConvertVoutMv.argtypes=[C.c_uint16]
        lib.PD_ConvertVoutMv.restype=C.c_int32
        gain_num=config_int('PD_VOUT_CAL_GAIN_NUM')
        gain_den=config_int('PD_VOUT_CAL_GAIN_DEN')
        offset_mv=config_int('PD_VOUT_CAL_OFFSET_MV')
        vin_off=config_int('PD_VIN_CAL_OFFSET_MV')
        def vin(raw): return (raw*VREF_VOLT*1098+(4095*100)//2)//(4095*100)
        def vout(raw): return (raw*VREF_VOLT*99990+(4095*4990)//2)//(4095*4990)
        def calibrated(mv,num,den,off): return (mv*num+(den//2))//den+off
        # The reference correction is applied to BOTH channels, then the configured
        # per-channel gain/offset - the documented order. With nominal coefficients
        # this reduces to the plain reference expression, so the same assertions
        # certify the uncalibrated build and the calibrated revision.
        for raw in (0,1,100,2000,2048,2260,4094,4095):
            self.assertEqual(lib.PD_ConvertVinMv(raw),calibrated(vin(raw),1000,1000,vin_off),raw)
            self.assertEqual(lib.PD_ConvertVoutMv(raw),
                             calibrated(vout(raw),gain_num,gain_den,offset_mv),raw)
        self.assertEqual(lib.PD_ConvertVinMv(0),vin_off)
        self.assertEqual(lib.PD_ConvertVoutMv(0),offset_mv)
        # Round-to-nearest really rounds: find a code whose exact value has a
        # fractional part above 0.5 and check the C result against the reference
        # expression (truncation would be one count lower).
        probe=None
        for raw in range(1,4096):
            exact=(raw*VREF_VOLT*1098)/(4095*100)
            if exact-int(exact)>0.55:
                probe=raw; break
        self.assertIsNotNone(probe,'no code exercises the rounding')
        self.assertEqual(lib.PD_ConvertVinMv(probe),calibrated(vin(probe),1000,1000,vin_off))
        self.assertEqual(lib.PD_ConvertVinMv(probe),
                         int((probe*VREF_VOLT*1098)/(4095*100))+1+vin_off)
        last=-1
        for raw in range(0,4096,17):
            v=lib.PD_ConvertVinMv(raw)
            self.assertGreaterEqual(v,last); last=v
        # Full scale follows the divider, not the output range: VIN is a ~1:11
        # divider (1098/100) so a full-scale code is ~36.1 V, and VOUT is a
        # ~1:20 divider (99990/4990) so it is ~65.8 V. Both are well inside int32.
        # The divider scales are the reference correction's own claim, so allow for
        # whatever gain/offset is configured on top of them.
        self.assertAlmostEqual(lib.PD_ConvertVinMv(4095)/1000.0,
                               36.07+vin_off/1000.0,delta=0.05)
        self.assertAlmostEqual(lib.PD_ConvertVoutMv(4095)/1000.0,
                               65.83*gain_num/gain_den+offset_mv/1000.0,delta=0.06)
        self.assertLess(lib.PD_ConvertVoutMv(4095),70000)
        self.assertTrue([r for r in range(1000,1200) if abs(lib.PD_ConvertVoutMv(r)-18000)<20],
                        '18 V must be representable')
        # Current channels: unchanged basis, and the voltage reference must NOT be
        # reused for them (that would shift IIN by about -75 mA at mid scale).
        self.assertEqual(VREF_CURRENT,3300)
        self.assertNotEqual(VREF_CURRENT,VREF_VOLT)
        lib.PD_ZeroCurrent.argtypes=[C.c_uint8,C.c_uint16]
        lib.PD_ZeroCurrent.restype=C.c_int32
        raw_zero=int(1.645962/3.3*4095)
        self.assertAlmostEqual(lib.PD_ZeroCurrent(1,raw_zero),0,delta=30)
        self.assertGreater(lib.PD_ZeroCurrent(1,raw_zero+250),900)

if __name__=='__main__':unittest.main(verbosity=2)
