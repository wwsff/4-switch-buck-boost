"""Exercise the real-port reader branch with fragmented, bounded availability."""
import time
import unittest
from unittest.mock import patch

from device import Device
from protocol import CMD, Parser
from simulator import DemoSerial


class AvailableSerial(DemoSerial):
    def __init__(self, **kwargs):
        super().__init__()
        self.options = kwargs
        self.heartbeat_times = []

    def open(self):
        pass

    def reset_input_buffer(self):
        with self.lock:
            self.rx.clear()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def write(self, raw):
        for frame in Parser().feed(raw):
            if frame.opcode == CMD['HEARTBEAT']:
                self.heartbeat_times.append(time.monotonic())
        return super().write(raw)

    def read(self, size):
        if size > max(1, self.in_waiting):
            raise AssertionError('reader requested bytes that have not arrived')
        # Even an availability snapshot need not be returned in one chunk.
        return super().read(min(size, 47))


class SerialReceiveTests(unittest.TestCase):
    def test_real_port_fragmented_reads_keep_handshake_and_heartbeat_alive(self):
        d = Device()
        with patch('serial.Serial', AvailableSerial), patch.object(
                Device, 'ports', return_value=[dict(port='TEST', description='test only')]):
            try:
                d.connect('TEST')
                transport = d.serial
                self.assertEqual(transport.options['timeout'], 0)
                until = time.monotonic() + 1.1
                while time.monotonic() < until:
                    d.snapshot()
                    time.sleep(.05)
                self.assertTrue(d.connected)
                self.assertFalse(d.lease_expired)
                self.assertGreaterEqual(d.ack_frames, 5)
                beats = transport.heartbeat_times
                self.assertGreaterEqual(len(beats), 4)
                self.assertLess(max(b-a for a, b in zip(beats, beats[1:])), .6)
                self.assertTrue(d.disconnect()['stop_ack'])
            finally:
                if d.serial is not None:
                    d.disconnect()
