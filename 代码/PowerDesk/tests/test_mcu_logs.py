import time
import unittest
from device import Device
from protocol import Frame, KIND, CMD, pack

class LogTests(unittest.TestCase):
    def test_shadow_log_is_exposed_with_age_and_cleared_on_reconnect(self):
        d=Device()
        try:
            d.connect('',demo=True)
            payload=pack('log',dict(uptime_ms=1500,level=0,code=12,message='SHADOW ref=0.1 y=0.05 P=0.1 I=0 D=0 U=0.1'))
            with d.serial.lock:
                d.serial._emit(Frame(KIND['EVENT'],0,CMD['LOG'],1,payload))
            time.sleep(.05)
            entry=d.snapshot()['pid_shadow'][1]
            self.assertIn('U=0.1',entry['message']);self.assertLess(entry['age'],1)
            d.disconnect();d.connect('',demo=True)
            self.assertEqual(d.snapshot()['pid_shadow'],{})
        finally:d.disconnect()
    def test_binary_mcu_log_reaches_event_view(self):
        d=Device()
        try:
            d.connect('',demo=True)
            payload=pack('log',dict(uptime_ms=1200,level=2,code=7,message='ADC timing error'))
            with d.serial.lock:
                d.serial._emit(Frame(KIND['EVENT'],0,CMD['LOG'],1,payload))
            time.sleep(.05)
            self.assertTrue(any('ADC timing error' in e['message'] and e['level']=='error' for e in d.events))
        finally:d.disconnect()

if __name__=='__main__':unittest.main()
