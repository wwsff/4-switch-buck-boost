"""PowerDesk wire codec. schema.json is the only source of wire constants/layouts."""
from dataclasses import dataclass
from pathlib import Path
import json
import math
import struct
import time
import zlib

ROOT = Path(__file__).resolve().parent
SCHEMA = json.loads((ROOT / "protocol/schema.json").read_text(encoding="utf-8"))
SCHEMA_CRC = zlib.crc32(json.dumps(SCHEMA, sort_keys=True, separators=(",", ":")).encode())
CMD, KIND, ERR = (SCHEMA[k] for k in ("commands", "kinds", "errors"))
VERSION = SCHEMA["version"]
MAX_PAYLOAD = SCHEMA["max_payload"]
LAYOUTS = {k: struct.Struct("<" + "".join(t for _, t in v)) for k, v in SCHEMA["layouts"].items()}
HEADER = struct.Struct("<BBHBBH")
SYNC = b"\xa5\x5a"


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(name, values):
    args = []
    for field, typ in SCHEMA["layouts"][name]:
        value = values[field]
        if typ.endswith("s"):
            value = value.encode("ascii") if isinstance(value, str) else value
            if len(value) > int(typ[:-1]):
                raise ValueError(f"{field}: string too long")
        elif typ == "f" and not math.isfinite(value):
            raise ValueError(f"{field}: finite number required")
        args.append(value)
    return LAYOUTS[name].pack(*args)


def unpack(name, payload):
    values = LAYOUTS[name].unpack(payload)
    result = {}
    for (field, typ), value in zip(SCHEMA["layouts"][name], values):
        if typ.endswith("s"):
            value = value.split(b"\0", 1)[0].decode("ascii", errors="replace")
        if typ == "f" and not math.isfinite(value):
            raise ValueError("non-finite PID received")
        result[field] = value
    return result


@dataclass(frozen=True)
class Frame:
    kind: int
    seq: int
    opcode: int
    channel: int = 0
    payload: bytes = b""

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload too large")
        body = HEADER.pack(VERSION, self.kind, self.seq, self.opcode, self.channel, len(self.payload)) + self.payload
        return SYNC + body + struct.pack("<H", crc16(body))


class Parser:
    """Bounded streaming parser; supports split/coalesced frames and CRC recovery."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0
        self.last_feed = 0.0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.buffer and now - self.last_feed > 0.25:
            self.buffer.clear()
            self.bad_frames += 1
        self.last_feed = now
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(SYNC[:1]) else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            version, kind, seq, op, ch, length = HEADER.unpack_from(self.buffer, 2)
            if version != VERSION or kind not in KIND.values() or ch > 2 or length > MAX_PAYLOAD:
                del self.buffer[0]
                self.bad_frames += 1
                continue
            size = 12 + length
            if len(self.buffer) < size:
                break
            if crc16(self.buffer[2:size-2]) != struct.unpack_from("<H", self.buffer, size-2)[0]:
                del self.buffer[0]
                self.bad_frames += 1
                continue
            frames.append(Frame(kind, seq, op, ch, bytes(self.buffer[10:size-2])))
            del self.buffer[:size]
        return frames
