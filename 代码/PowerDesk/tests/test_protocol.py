import json
import struct
import unittest
from pathlib import Path
from protocol import Frame, Parser, CMD, KIND, LAYOUTS, SCHEMA_CRC, crc16, pack, unpack


class ProtocolTests(unittest.TestCase):
    def test_reference_crc(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_vectors_are_frozen_and_match_contract(self):
        data = json.loads(Path("protocol/test_vectors.json").read_text())
        self.assertEqual(data["schema_crc32"], f"{SCHEMA_CRC:08X}")
        for vector in data["frames"]:
            raw = bytes.fromhex(vector["hex"])
            frame, = Parser().feed(raw)
            self.assertEqual(frame.opcode, vector["opcode"])
            self.assertEqual(frame.encode(), raw)

    def test_fragmentation_at_every_boundary(self):
        f = Frame(1, 123, CMD["SET_TARGET"], 2, pack("target", dict(vref_mv=12000, iref_ma=1000)))
        raw = f.encode()
        for i in range(1, len(raw)):
            parser = Parser()
            self.assertEqual(parser.feed(raw[:i]), [])
            self.assertEqual(parser.feed(raw[i:]), [f])

    def test_crc_damage_noise_and_coalescing(self):
        f = Frame(1, 5, CMD["STOP_ALL"])
        bad = bytearray(f.encode())
        bad[-1] ^= 0x80
        parser = Parser()
        self.assertEqual(parser.feed(b"noise\xa5" + bad + f.encode() * 2), [f, f])
        self.assertGreater(parser.bad_frames, 0)

    def test_invalid_version_and_oversize_recovery(self):
        f = Frame(1, 5, CMD["STOP_ALL"])
        invalid = b"\xa5\x5a" + struct.pack("<BBHBBH", 1, 1, 2, 1, 0, 65535)
        wrong_version = bytearray(f.encode())
        wrong_version[2] = 9
        self.assertEqual(Parser().feed(invalid + wrong_version + f.encode()), [f])

    def test_partial_frame_expires(self):
        f = Frame(1, 3, CMD["INFO"])
        p = Parser()
        p.feed(f.encode()[:10], now=10)
        self.assertEqual(p.feed(f.encode(), now=10.3), [f])

    def test_buffer_bound_under_noise(self):
        p = Parser()
        p.feed(b"a" * 1000000)
        self.assertLessEqual(len(p.buffer), 1)

    def test_nonfinite_pid_rejected(self):
        d = dict(loop=0, kp=1, ki=2, kd=0, tau=0, out_min=0, out_max=float("nan"))
        with self.assertRaises(ValueError):
            pack("pid", d)
        with self.assertRaises(ValueError):
            unpack("pid", LAYOUTS["pid"].pack(0, 1, 1, 0, 0, 0, float("inf")))

    def test_serial_driver_byte_roundtrip(self):
        import serial
        with serial.serial_for_url("loop://", timeout=0.2) as port:
            raw = Frame(KIND["REQUEST"], 1, CMD["INFO"]).encode()
            port.write(raw)
            self.assertEqual(port.read(len(raw)), raw)


if __name__ == "__main__":
    unittest.main()
