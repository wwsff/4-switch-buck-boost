"""Windows COM-name conflict recovery: fall back to the kernel device path.

Regression for the 2026-09-09 bench failure where the CH340K was reported as
"USB-SERIAL CH340K (COM1)" and started, yet CreateFile("\\\\.\\COM1") returned
ERROR_FILE_NOT_FOUND. The kernel object path still reaches the same device.
"""
import unittest
from unittest.mock import patch

import serial

from device import Device
from simulator import DemoSerial

MISSING = "could not open port {!r}: FileNotFoundError(2, '系统找不到指定的文件。', None, 2)"
NO_SUCH_DEVICE = "could not open port {!r}: OSError(22, '指定的设备不存在。', None, 433)"


class KernelSerial(DemoSerial):
    """Emulates the driver: COM1 name broken, \\Device\\Serial19 reachable."""

    responder = 19
    message = MISSING

    def __init__(self, **kwargs):
        super().__init__()
        self.options = kwargs

    def open(self):
        if self.port == 'COM1':
            raise serial.SerialException(self.message.format('COM1'))
        if self.port != r'\\.\GLOBALROOT\Device\Serial%d' % self.responder:
            raise serial.SerialException(self.message.format(self.port))

    def reset_input_buffer(self):
        with self.lock:
            self.rx.clear()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def read(self, size):
        return super().read(min(size, 4096))


class KernelSerial433(KernelSerial):
    """Windows also reports ERROR_NO_SUCH_DEVICE (433) for the same broken name."""

    message = NO_SUCH_DEVICE


class SerialFallbackTests(unittest.TestCase):
    def test_missing_com_name_falls_back_to_kernel_path(self):
        d = Device()
        with patch('serial.Serial', KernelSerial), patch.object(
                Device, 'ports', return_value=[dict(port='COM1', description='USB-SERIAL CH340K (COM1)')]):
            try:
                d.connect('COM1')
                self.assertTrue(d.connected)
                self.assertEqual(d.port_path, r'\\.\GLOBALROOT\Device\Serial19')
                self.assertEqual(d.port, 'COM1')
                messages = [e['message'] for e in d.events]
                self.assertTrue(any('内核设备路径' in m for m in messages), messages)
                self.assertTrue(any('改到空闲 COM 号' in m for m in messages), messages)
                self.assertTrue(d.disconnect()['stop_ack'])
            finally:
                if d.serial is not None:
                    d.disconnect()

    def test_no_such_device_winerror_433_also_falls_back(self):
        d = Device()
        with patch('serial.Serial', KernelSerial433), patch.object(
                Device, 'ports', return_value=[dict(port='COM1', description='USB-SERIAL CH340K (COM1)')]):
            try:
                d.connect('COM1')
                self.assertTrue(d.connected)
                self.assertEqual(d.port_path, r'\\.\GLOBALROOT\Device\Serial19')
            finally:
                if d.serial is not None:
                    d.disconnect()

    def test_other_serial_devices_are_skipped_until_the_mcu_answers(self):
        class TwoDevices(KernelSerial):
            responder = 7

            def open(self):
                # Device 3 exists but is not our MCU: it accepts the open and stays silent.
                if self.port == r'\\.\GLOBALROOT\Device\Serial3':
                    return
                return super().open()

            def read(self, size):
                if self.port == r'\\.\GLOBALROOT\Device\Serial3':
                    return b''
                return super().read(size)

        d = Device()
        with patch('serial.Serial', TwoDevices), patch.object(
                Device, 'ports', return_value=[dict(port='COM1', description='USB-SERIAL CH340K (COM1)')]):
            try:
                d.connect('COM1')
                self.assertEqual(d.port_path, r'\\.\GLOBALROOT\Device\Serial7')
                self.assertTrue(d.connected)
            finally:
                if d.serial is not None:
                    d.disconnect()

    def test_no_mcu_found_reports_actionable_error(self):
        class NothingThere(KernelSerial):
            responder = -1

        d = Device()
        with patch('serial.Serial', NothingThere), patch.object(
                Device, 'ports', return_value=[dict(port='COM1', description='USB-SERIAL CH340K (COM1)')]):
            with self.assertRaises(Exception) as ctx:
                d.connect('COM1')
            self.assertIn('COM 号冲突', str(ctx.exception))
            self.assertIsNone(d.serial)
            self.assertFalse(d.connected)

    def test_busy_port_is_reported_without_kernel_scan(self):
        class Busy(DemoSerial):
            def __init__(self, **kwargs):
                super().__init__()

            def open(self):
                raise serial.SerialException("could not open port 'COM1': PermissionError(13, '拒绝访问。', None, 5)")

        d = Device()
        with patch('serial.Serial', Busy), patch.object(
                Device, 'ports', return_value=[dict(port='COM1', description='USB-SERIAL CH340K (COM1)')]):
            with self.assertRaises(Exception) as ctx:
                d.connect('COM1')
            self.assertIn('被其他程序占用', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
