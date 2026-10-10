import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import profiles
from device import Device, DeviceError
from protocol import CMD, SCHEMA_CRC, Frame, KIND, pack, unpack
from simulator import DemoSerial


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.profile_patch = patch.object(profiles, "PATH", Path(self.tmp.name) / "profiles.json")
        self.profile_patch.start()
        self.d = Device()
        self.d.connect("", demo=True, auto_restore=False)

    def tearDown(self):
        self.d.disconnect()
        self.profile_patch.stop()
        self.tmp.cleanup()

    def refresh(self):
        time.sleep(0.07)
        return self.d.snapshot()

    def test_handshake_capabilities_and_default_off(self):
        self.assertTrue(self.d.connected)
        self.assertEqual(self.d.info["schema_crc"], SCHEMA_CRC)
        self.assertEqual(set(self.d.caps), {1, 2})
        self.assertTrue(all(t["enabled"] == 0 for t in self.d.telemetry.values()))

    def test_host_range_comes_from_caps_not_from_a_host_constant(self):
        """0.3.22 widened CH1 from 2..15 V to 2..18 V. The host must follow the
        CAPS the device reports in both cases: an old firmware must still show
        2..15 V, a new one 2..18 V, and the range must never be a host constant."""
        self.assertEqual((self.d.caps[1]["vmin_mv"], self.d.caps[1]["vmax_mv"]), (2000, 18000))
        self.assertEqual(self.d.action("target", 1, {"vref_mv": 18000, "iref_ma": 1000})["vref_mv"], 18000)
        with self.assertRaisesRegex(DeviceError, "范围"):
            self.d.action("target", 1, {"vref_mv": 18001, "iref_ma": 1000})
        old = dict(self.d.caps[1])
        self.d.caps[1]["vmax_mv"] = 15000                    # simulate an old firmware
        with self.assertRaisesRegex(DeviceError, "范围"):
            self.d.action("target", 1, {"vref_mv": 18000, "iref_ma": 1000})
        self.assertEqual(self.d.action("target", 1, {"vref_mv": 15000, "iref_ma": 1000})["vref_mv"], 15000)
        self.d.caps[1].update(old)

    def test_zero_not_exposed_by_unsupported_demo(self):
        with self.assertRaisesRegex(DeviceError,'未开放'):
            self.d.action('zero',1,{'action':1})

    def test_zero_ack_is_pending_and_query_is_read_only(self):
        self.d.caps[1]['features'] |= 64
        pending=dict(state=1,sensor=1,applied=0,reason=0,samples=0,reserved=0,
                     iout_uv=1650000,il_uv=1650000,pp_uv=0,span_uv=0)
        with patch.object(self.d,'request',return_value=pending) as req:
            self.assertEqual(self.d.action('zero',1,{'action':1})['state'],1)
            req.assert_called_once_with('ZERO',1,'zero_request',{'action':1},'zero_status')
            req.reset_mock()
            self.d.action('zero',1,{'action':0})
            req.assert_called_once_with('ZERO',1,'zero_request',{'action':0},'zero_status')

    def test_zero_rejects_invalid_action_and_running_mode(self):
        self.d.caps[1]['features'] |= 64
        with self.assertRaises(DeviceError):self.d.action('zero',1,{'action':5})
        self.d.telemetry[1]['mode']=1
        with self.assertRaisesRegex(DeviceError,'正常'):
            self.d.action('zero',1,{'action':1})

    def test_target_limits_and_readback(self):
        with self.assertRaises(DeviceError):
            self.d.action("target", 1, dict(vref_mv=999999, iref_ma=1000))
        result = self.d.action("target", 1, dict(vref_mv=7000, iref_ma=500))
        self.assertEqual(result, dict(vref_mv=7000, iref_ma=500))
        self.assertEqual(self.refresh()["telemetry"][1]["vref_mv"], 7000)

    def test_start_fault_latch_clear_does_not_restart(self):
        self.d.action("output", 1, {"enabled": 1})
        self.d.serial.fault(1)
        self.assertEqual(self.refresh()["telemetry"][1]["state"], 8)
        with self.assertRaises(DeviceError):
            self.d.action("output", 1, {"enabled": 1})
        self.d.action("clear_fault", 1)
        self.assertEqual(self.refresh()["telemetry"][1]["enabled"], 0)

    def test_pid_requires_read_and_off_and_matches_float32(self):
        p = dict(loop=0, kp=0.4, ki=2.0, kd=0.0, tau=0.0001, out_min=-1, out_max=1)
        with self.assertRaises(DeviceError):
            self.d.action("set_pid", 1, p)
        self.d.action("get_pid", 1, {"loop": 0})
        self.assertEqual(self.d.action("set_pid", 1, p), unpack("pid", pack("pid", p)))
        self.d.action("output", 1, {"enabled": 1})
        self.refresh()
        with self.assertRaises(DeviceError):
            self.d.action("set_pid", 1, p)
        with self.assertRaises(DeviceError):
            self.d.action("mode", 1, {"mode": 1})

    def test_device_nack_is_reported(self):
        with self.assertRaisesRegex(DeviceError, "范围"):
            self.d.request("SET_TARGET", 1, "target", dict(vref_mv=-1, iref_ma=1), "target")

    def test_no_ack_is_timeout_not_success_or_retry(self):
        original = self.d.serial._handle
        calls = []
        def handler(f):
            if f.opcode == CMD["SET_TARGET"]:
                calls.append(f)
                # Apply the target but deliberately lose the ACK.
                result = original(f)
                fseq = f.seq
                self.d.serial._emit = lambda response: None if response.seq == fseq else emit(response)
                return result
            return original(f)
        emit = self.d.serial._emit
        self.d.serial._handle = handler
        with self.assertRaisesRegex(DeviceError, "结果未知"):
            self.d.request("SET_TARGET", 1, "target", dict(vref_mv=6000, iref_ma=500), "target", timeout=0.12)
        self.assertEqual(len(calls), 1)

    def test_debug_timer_stops_independently(self):
        self.d.action("mode", 1, {"mode": 1})
        self.refresh()
        self.d.action("debug_ref", 1, dict(ilref_ma=100, duration_ms=100))
        self.d.action("output", 1, {"enabled": 1})
        time.sleep(0.2)
        self.assertEqual(self.d.snapshot()["telemetry"][1]["enabled"], 0)

    def test_debug_cannot_start_without_explicit_reference(self):
        self.d.action("mode", 1, {"mode": 1})
        self.refresh()
        with self.assertRaisesRegex(DeviceError, "先明确设置"):
            self.d.action("output", 1, {"enabled": 1})

    def test_one_way_telemetry_loss_revokes_heartbeat(self):
        self.d.action("output", 1, {"enabled": 1})
        emit = self.d.serial._emit
        self.d.serial._emit = lambda f: None if f.kind == KIND["EVENT"] else emit(f)
        self.d.received[1] = time.monotonic() - 2
        time.sleep(0.06)
        self.assertTrue(self.d.lease_expired)
        self.assertEqual(self.d.serial.channels[1]["enabled"], 0)

    def test_seq_zero_cannot_start_output(self):
        self.d.serial.write(Frame(KIND["REQUEST"], 0, CMD["OUTPUT"], 1, pack("output", {"enabled": 1})).encode())
        self.assertEqual(self.d.serial.channels[1]["enabled"], 0)

    def test_clear_then_write_pid_requires_off_state(self):
        self.d.action("get_pid", 1, {"loop": 0})
        self.d.serial.fault(1)
        self.refresh()
        with self.assertRaisesRegex(DeviceError, "解除故障"):
            self.d.action("set_pid", 1, dict(loop=0,kp=.1,ki=1,kd=0,tau=0,out_min=0,out_max=1))

    def test_stale_data_rejects_control_but_stop_is_available(self):
        with self.d.lock:
            self.d.received[1] = time.monotonic() - 10
        with self.assertRaisesRegex(DeviceError, "过期"):
            self.d.action("output", 1, {"enabled": 1})
        self.d.action("stop_all")

    def test_ui_lease_expiry_stops_and_requires_reconnect(self):
        self.d.action("output", 1, {"enabled": 1})
        self.d.last_ui = time.monotonic() - 10
        time.sleep(0.1)
        self.assertTrue(self.d.lease_expired)
        self.assertEqual(self.d.snapshot(touch=False)["telemetry"][1]["enabled"], 0)
        self.d.snapshot()
        with self.assertRaisesRegex(DeviceError, "重新连接"):
            self.d.action("output", 1, {"enabled": 1})

    def test_complete_trace_download_and_channel(self):
        result = self.d.capture(2, 256, 10)
        # The demo device now reports the real CH1 capabilities (0.3.22 firmware:
        # 125 kHz injected sampling), so the rate follows CAPS instead of a host
        # constant.
        self.assertEqual(result["sample_hz"], self.d.caps[2]["trace_base_hz"] // 10)
        self.assertEqual(result["sample_hz"], 12500)
        self.assertEqual(result["channel"], 2)
        self.assertEqual(len(result["samples"]), 256)

    def test_pid_readback_mismatch_rejected(self):
        self.d.action("get_pid", 1, {"loop": 0})
        original = self.d.serial._handle
        def handler(f):
            result = original(f)
            if f.opcode == CMD["SET_PID"]:
                self.d.serial.channels[1]["pids"][0]["kp"] += 1
            return result
        self.d.serial._handle = handler
        p = dict(loop=0, kp=.2, ki=1, kd=0, tau=0, out_min=0, out_max=1)
        with self.assertRaisesRegex(DeviceError, "回读"):
            self.d.action("set_pid", 1, p)

    def test_trace_corruption_rejected(self):
        original = self.d.serial._handle
        def handler(f):
            result = original(f)
            if f.opcode == CMD["TRACE_READ"]:
                return result[:-1]
            return result
        self.d.serial._handle = handler
        with self.assertRaisesRegex(DeviceError, "长度"):
            self.d.capture(1, 64, 1)


class AutoRestoreTests(unittest.TestCase):
    """The MCU forgets the gate map and the zeros on every reset; the host
    replays the mapping and re-measures the zeros automatically."""
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.profile_patch = patch.object(profiles, "PATH", Path(self.tmp.name) / "profiles.json")
        self.profile_patch.start()
        self.d = Device()

    def tearDown(self):
        self.d.disconnect()
        self.profile_patch.stop()
        self.tmp.cleanup()

    def wait(self, timeout=15):
        self.d._restore_thread.join(timeout)
        self.assertFalse(self.d._restore_thread.is_alive(), "auto-restore did not finish")
        return self.d.snapshot()["restore"]

    def connect(self, auto_restore=True):
        self.d.connect("", demo=True, auto_restore=auto_restore)

    def test_first_boot_records_nothing_and_asks_for_manual_steps(self):
        self.connect()
        report = self.wait()
        self.assertEqual(report["known"], False)
        self.assertEqual(report["gate"], "manual")
        # The zeros ARE measured automatically even on the first connect: the MCU
        # validates the conditions, and measuring is always better than a stale value.
        self.assertEqual(report["zero"], "remeasured")
        self.assertEqual(report["state"], "done")
        entry = profiles.get(self.d.info["device_id"])
        self.assertEqual(entry["iout_uv"], 1646136)
        self.assertEqual(entry["applied"], 3)

    def test_second_boot_restores_the_gate_map_and_the_zeros(self):
        self.connect()
        self.wait()
        self.d.disconnect()
        # Simulate a MCU reset: the simulator keeps its state, so clear it by hand.
        self.d = Device()
        with patch.object(DemoSerial, "__init__", DemoSerial.__init__):
            self.connect()
        # Force the fresh device state (a real reset clears RAM).
        self.d.serial.gate["confirmed"] = 0
        self.d.serial.zero["applied"] = 0
        self.d.serial.zero["iout_uv"] = 1650000
        self.d.serial.zero["il_uv"] = 1650000
        report = self.wait()
        self.assertEqual(report["known"], True)
        self.assertEqual(report["gate"], "restored")
        self.assertEqual(self.d.serial.gate["confirmed"], 1)
        self.assertEqual(self.d.serial.gate["mapping"], 0)
        self.assertEqual(report["zero"], "remeasured")
        self.assertEqual(self.d.serial.zero["applied"], 3)

    def test_already_confirmed_and_calibrated_needs_no_action(self):
        self.connect()
        self.wait()
        self.d.disconnect()
        self.d = Device()
        self.connect()
        self.d.serial.gate.update(confirmed=1, mapping=0)
        self.d.serial.zero.update(applied=3, state=3)
        report = self.wait()
        self.assertEqual((report["gate"], report["zero"]), ("already", "already"))

    def test_auto_restore_can_be_disabled(self):
        self.connect(auto_restore=False)
        time.sleep(0.3)
        self.assertIsNone(self.d._restore_thread)
        self.assertEqual(self.d.snapshot()["restore"], {})

    def test_manual_confirmation_is_remembered(self):
        self.connect(auto_restore=False)
        self.d.action("gate_test", 1, {"action": 2, "mapping": 2})
        entry = profiles.get(self.d.info["device_id"])
        self.assertEqual(entry["gate_map"], 2)
        self.d.action("gate_test", 1, {"action": 3})
        self.assertEqual(profiles.get(self.d.info["device_id"])["gate_confirmed"], False)


class IsolationTests(unittest.TestCase):
    def test_schema_mismatch_prevents_connection(self):
        class WrongSchema(DemoSerial):
            def _handle(self, f):
                result = super()._handle(f)
                if f.opcode == CMD["INFO"]:
                    d = unpack("info", result)
                    d["schema_crc"] ^= 1
                    return pack("info", d)
                return result
        d = Device()
        with patch("device.DemoSerial", WrongSchema):
            with self.assertRaisesRegex(DeviceError, "协议定义不一致"):
                d.connect("", True)
        self.assertFalse(d.connected)
        self.assertIsNone(d.serial)

    def test_mcu_watchdog_is_independent_of_host(self):
        s = DemoSerial()
        s.write(Frame(KIND["REQUEST"], 1, CMD["OUTPUT"], 1, pack("output", {"enabled": 1})).encode())
        s.beat = time.monotonic() - 2
        s.read()
        self.assertEqual(s.channels[1]["enabled"], 0)
        self.assertEqual(s.channels[1]["fault"], 16)
        s.close()


if __name__ == "__main__":
    unittest.main()
