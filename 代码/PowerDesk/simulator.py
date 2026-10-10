"""Wire-level demonstration device. Synthetic curves, NOT a power-stage model."""
import math
import struct
import threading
import time
from protocol import CMD, KIND, ERR, SCHEMA_CRC, Frame, Parser, pack, unpack


class Rejected(Exception):
    def __init__(self, code):
        self.code = ERR[code]


class DemoSerial:
    def __init__(self):
        self.lock = threading.RLock()
        self.parser = Parser()
        self.rx = bytearray()
        self.closed = False
        self.boot = self.beat = self.last_tick = time.monotonic()
        self.last_report = 0
        self.capture_id = 0
        self.traces = {}
        self.channels = {}
        # Demo device reports the REAL CH1 capabilities (0.3.22: 2..18 V), so the
        # demo UI cannot advertise a range the firmware would refuse.
        self.caps = dict(features=15, vmin_mv=2000, vmax_mv=18000, imax_ma=2000, ilmax_ma=500,
                         current_hz=125000, voltage_hz=1000, cc_hz=1000,
                         max_debug_ms=5000, trace_max_points=256, trace_base_hz=125000)
        for ch in (1, 2):
            self.channels[ch] = dict(uptime_ms=0, state=2, mode=0, enabled=0, ready=1,
                fault=0, valid=127, vin_mv=20000, iin_ma=0, vout_mv=0, iout_ma=0,
                il_ma=0, vref_mv=5000, iref_ma=500, ilref_ma=0, tmos_cdeg=2600,
                tl_cdeg=2500, duty_in=0, duty_out=0, duration_ms=1000, deadline=0,
                debug_configured=False, pids={i: dict(loop=i, kp=0.1, ki=1.0, kd=0.0, tau=0.0001,
                      out_min=0.0, out_max=2.0 if i == 0 else 1.0) for i in range(3)})
        # Gate mapping + zero calibration state, mirroring the MCU (RAM only).
        self.gate = dict(confirmed=0, mapping=0, active_index=0, until=0)
        self.zero = dict(state=0, sensor=0, applied=0, reason=0, samples=0, reserved=0,
                         iout_uv=1650000, il_uv=1650000, pp_uv=0, span_uv=0,
                         busy_until=0.0, target=0)

    def _emit(self, frame):
        self.rx.extend(frame.encode())

    def _stop(self, c):
        c["enabled"] = 0
        c["state"] = 8 if c["fault"] else 2
        c["duty_in"] = c["duty_out"] = 0

    def fault(self, ch):
        with self.lock:
            c = self.channels[ch]
            c["fault"] |= 1
            self._stop(c)

    def write(self, data):
        with self.lock:
            if self.closed:
                raise OSError("demo disconnected")
            for f in self.parser.feed(data):
                if f.kind != KIND["REQUEST"]:
                    continue
                if f.seq == 0 and f.opcode not in (CMD["HEARTBEAT"], CMD["STOP_ALL"]):
                    continue
                try:
                    payload = self._handle(f)
                    if f.seq:
                        self._emit(Frame(KIND["ACK"], f.seq, f.opcode, f.channel, payload))
                except Rejected as e:
                    if f.seq:
                        self._emit(Frame(KIND["NACK"], f.seq, f.opcode, f.channel, pack("error", {"code": e.code})))
                except (ValueError, KeyError, TypeError, struct.error):
                    if f.seq:
                        self._emit(Frame(KIND["NACK"], f.seq, f.opcode, f.channel, pack("error", {"code": ERR["BAD_LENGTH"]})))
            return len(data)

    def _handle(self, f):
        op = f.opcode
        if op in (CMD["INFO"], CMD["HEARTBEAT"], CMD["STOP_ALL"]):
            if f.channel != 0:
                raise Rejected("BAD_CHANNEL")
            if f.payload:
                raise Rejected("BAD_LENGTH")
            if op == CMD["INFO"]:
                return pack("info", dict(schema_crc=SCHEMA_CRC, firmware="DEMO-1.0.0", device_id="SIMULATED-NOT-HARDWARE", watchdog_ms=1000))
            if op == CMD["HEARTBEAT"]:
                self.beat = time.monotonic()
            else:
                for c in self.channels.values():
                    self._stop(c)
            return b""
        if f.channel not in self.channels:
            raise Rejected("BAD_CHANNEL")
        c = self.channels[f.channel]
        empty = (CMD["CAPS"], CMD["STATUS"], CMD["CLEAR_FAULT"], CMD["TRACE_INFO"])
        if op in empty and f.payload:
            raise Rejected("BAD_LENGTH")
        if op == CMD["CAPS"]:
            return pack("caps", self.caps)
        if op == CMD["STATUS"]:
            return pack("telemetry", c)
        if op == CMD["SET_TARGET"]:
            d = unpack("target", f.payload)
            if not (0 <= d["vref_mv"] <= self.caps["vmax_mv"] and 0 <= d["iref_ma"] <= self.caps["imax_ma"]):
                raise Rejected("RANGE")
            c.update(d)
            return pack("target", d)
        if op == CMD["SET_MODE"]:
            if c["enabled"] or c["state"] != 2:
                raise Rejected("NOT_OFF")
            d = unpack("mode", f.payload)
            if d["mode"] not in (0, 1):
                raise Rejected("RANGE")
            c.update(d)
            c["debug_configured"] = False
            return pack("mode", d)
        if op == CMD["OUTPUT"]:
            d = unpack("output", f.payload)
            if d["enabled"] not in (0, 1):
                raise Rejected("RANGE")
            if d["enabled"]:
                if c["fault"]:
                    raise Rejected("FAULT")
                if not c["ready"]:
                    raise Rejected("NOT_READY")
                if c["mode"] == 1 and not c["debug_configured"]:
                    raise Rejected("NOT_READY")
                if not c["enabled"]:
                    c["deadline"] = time.monotonic() + c["duration_ms"] / 1000
                c["enabled"] = 1
                c["state"] = 9 if c["mode"] else 5
            else:
                self._stop(c)
            return pack("output", d)
        if op == CMD["CLEAR_FAULT"]:
            if c["enabled"]:
                raise Rejected("NOT_OFF")
            c["fault"] = 0
            self._stop(c)
            return b""
        if op == CMD["GATE_TEST"]:
            d = unpack("gate_test_request", f.payload)
            if d["action"] == 0:
                pass
            elif d["action"] == 1:
                if not 1 <= d["index"] <= 4 or not 100 <= d["duration_ms"] <= 2000:
                    raise Rejected("RANGE")
                if c["enabled"] or c["state"] != 2 or self.gate["active_index"]:
                    raise Rejected("NOT_OFF")
                if not (c["vin_mv"] <= 5000 and c["vout_mv"] <= 1000):
                    raise Rejected("NOT_READY")
                self.gate["active_index"] = d["index"]
                self.gate["until"] = time.monotonic() + d["duration_ms"] / 1000
            elif d["action"] == 2:
                if d["index"] > 3:
                    raise Rejected("RANGE")
                if self.gate["active_index"]:
                    raise Rejected("BUSY")
                self.gate["mapping"] = d["index"]
                self.gate["confirmed"] = 1
            elif d["action"] == 3:
                if self.gate["active_index"]:
                    raise Rejected("BUSY")
                self.gate["confirmed"] = 0
            else:
                raise Rejected("RANGE")
            return pack("gate_test_status", dict(confirmed=self.gate["confirmed"],
                        mapping=self.gate["mapping"], active_index=self.gate["active_index"],
                        running=1 if self.gate["active_index"] else 0,
                        remaining_ms=max(0, int((self.gate["until"] - time.monotonic()) * 1000))))
        if op == CMD["ZERO"]:
            d = unpack("zero_request", f.payload)
            a = d["action"]
            if a > 4:
                raise Rejected("RANGE")
            if a == 0:
                pass
            elif a in (1, 2):
                if c["enabled"] or c["state"] != 2 or c["mode"] != 0:
                    raise Rejected("NOT_OFF")
                if c["vout_mv"] > 200 or not 17000 <= c["vin_mv"] <= 23000:
                    raise Rejected("NOT_READY")
                self.zero.update(state=2, sensor=a, reason=0, samples=0,
                                 busy_until=time.monotonic() + 0.2, target=a)
            elif a in (3, 4):
                sensor = a - 2
                self.zero["applied"] &= ~(1 << (sensor - 1))
                self.zero["state"] = 0
                self.zero["sensor"] = sensor
                if sensor == 1:
                    self.zero["iout_uv"] = 1650000
                else:
                    self.zero["il_uv"] = 1650000
            if self.zero["state"] in (1, 2) and time.monotonic() >= self.zero["busy_until"]:
                self.zero["state"] = 3
                self.zero["samples"] = 4096
                self.zero["applied"] |= 1 << (self.zero["target"] - 1)
                if self.zero["target"] == 1:
                    self.zero["iout_uv"] = 1646136
                else:
                    self.zero["il_uv"] = 1650000
            out = {k: self.zero[k] for k in ("state", "sensor", "applied", "reason", "samples",
                                             "reserved", "iout_uv", "il_uv", "pp_uv", "span_uv")}
            return pack("zero_status", out)
        if op in (CMD["GET_PID"], CMD["SET_PID"]):
            d = unpack("loop" if op == CMD["GET_PID"] else "pid", f.payload)
            if d["loop"] not in c["pids"]:
                raise Rejected("RANGE")
            if op == CMD["SET_PID"]:
                if c["enabled"] or c["state"] != 2:
                    raise Rejected("NOT_OFF")
                if min(d[k] for k in ("kp", "ki", "kd", "tau")) < 0 or d["out_min"] >= d["out_max"] or (d["kd"] > 0 and d["tau"] <= 0):
                    raise Rejected("RANGE")
                c["pids"][d["loop"]] = d
            return pack("pid", c["pids"][d["loop"]])
        if op == CMD["DEBUG_REF"]:
            if c["enabled"] or c["state"] != 2 or c["mode"] != 1:
                raise Rejected("NOT_OFF")
            d = unpack("debug_ref", f.payload)
            if not (0 <= d["ilref_ma"] <= self.caps["ilmax_ma"] and 10 <= d["duration_ms"] <= self.caps["max_debug_ms"]):
                raise Rejected("RANGE")
            c.update(d)
            c["debug_configured"] = True
            return pack("debug_ref", d)
        if op == CMD["TRACE_ARM"]:
            d = unpack("trace_arm", f.payload)
            if not (16 <= d["points"] <= 256 and 1 <= d["decimation"] <= 1000):
                raise Rejected("RANGE")
            self.capture_id += 1
            hz = self.caps["trace_base_hz"] // d["decimation"]
            points = []
            for i in range(d["points"]):
                ripple = round(20 * math.sin(i * 0.29)) if c["enabled"] else 0
                points.append(dict(il_ma=c["il_ma"] + ripple, ilref_ma=c["ilref_ma"],
                    vout_mv=c["vout_mv"], vref_mv=c["vref_mv"], duty_in=c["duty_in"], duty_out=c["duty_out"]))
            meta = dict(capture_id=self.capture_id, state=2, points=len(points), sample_hz=hz)
            self.traces[f.channel] = (meta, points)
            return pack("trace_info", meta)
        if op in (CMD["TRACE_INFO"], CMD["TRACE_READ"]):
            if f.channel not in self.traces:
                raise Rejected("NOT_FOUND")
            meta, points = self.traces[f.channel]
            if op == CMD["TRACE_INFO"]:
                return pack("trace_info", meta)
            d = unpack("trace_read", f.payload)
            if d["capture_id"] != meta["capture_id"]:
                raise Rejected("NOT_FOUND")
            if not (1 <= d["count"] <= 24 and 0 <= d["offset"] <= len(points) - d["count"]):
                raise Rejected("RANGE")
            return pack("trace_chunk", d) + b"".join(pack("trace_point", p) for p in points[d["offset"]:d["offset"]+d["count"]])
        raise Rejected("UNSUPPORTED")

    def read(self, size=4096):
        time.sleep(0.01)
        with self.lock:
            if self.closed:
                raise OSError("demo disconnected")
            now = time.monotonic()
            dt = min(now - self.last_tick, 0.1)
            self.last_tick = now
            for ch, c in self.channels.items():
                if now - self.beat > 1.0 and c["enabled"]:
                    c["fault"] |= 16
                    self._stop(c)
                if c["enabled"] and c["mode"] == 1 and now >= c["deadline"]:
                    self._stop(c)
                target = min(c["vref_mv"], c["iref_ma"] * 10) if c["enabled"] else 0
                if c["enabled"] and c["mode"] == 1:
                    target = min(c["ilref_ma"] * 10, c["vref_mv"])
                c["vout_mv"] = round(c["vout_mv"] + (target - c["vout_mv"]) * min(1, dt * 7))
                c["iout_ma"] = c["vout_mv"] // 10
                c["il_ma"] = c["iout_ma"] if c["enabled"] else 0
                if c["mode"] == 0:
                    c["ilref_ma"] = c["iout_ma"]
                    if c["enabled"]:
                        c["state"] = 6 if c["vref_mv"] > c["iref_ma"] * 10 else 5
                c["iin_ma"] = round(c["vout_mv"] * c["iout_ma"] / 20000 / 0.92)
                c["duty_in"] = round(c["vout_mv"] / 20000 * 9500) if c["enabled"] else 0
                c["duty_out"] = 9500 if c["enabled"] else 0
                c["uptime_ms"] = int((now - self.boot) * 1000) & 0xFFFFFFFF
            if now - self.last_report >= 0.05:
                self.last_report = now
                for ch, c in self.channels.items():
                    self._emit(Frame(KIND["EVENT"], 0, CMD["TELEMETRY"], ch, pack("telemetry", c)))
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

    def close(self):
        with self.lock:
            for c in self.channels.values():
                self._stop(c)
            self.closed = True
