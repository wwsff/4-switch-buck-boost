"""Serial transport, command acknowledgements and authoritative device state."""
from collections import deque
import copy
import math
import re
import threading
import time
import struct
from protocol import CMD, KIND, ERR, SCHEMA, SCHEMA_CRC, LAYOUTS, Frame, Parser, pack, unpack
from simulator import DemoSerial
import profiles

ERROR_TEXT = {1: "帧长度错误", 2: "通道错误", 3: "固件不支持此功能", 4: "参数超出设备允许范围", 5: "请先停止输出", 6: "硬件尚未就绪", 7: "故障尚未解除", 8: "设备忙", 9: "记录不存在", 10: "协议不匹配"}
GATE_MAP_TEXT = ["设计意图（PA8→输入高侧、PA10→输出高侧）", "输入桥反相（PA8→输入低侧）",
                 "输出桥反相（PA10→输出低侧）", "两桥都反相"]
ZERO_REASON_TEXT = ["", "停机、供电、残压或采样条件不满足", "采样越界或零点超出允许窗口",
                    "波动或分段均值变化过大", "采样超时", "已被停止或故障取消"]

# Windows: a USB-serial driver names its functional device object \Device\SerialNN and
# maps it to a DOS name (COMx). When that COMx name is stale or taken, the driver still
# reports "started" and SetupAPI still lists the port, but CreateFile("\\.\COMx") fails.
# Observed Win32 codes for that state: 2 (FILE_NOT_FOUND) and 433 (NO_SUCH_DEVICE).
# The kernel object path below still reaches the device.
KERNEL_SERIAL_PATH = r"\\.\GLOBALROOT\Device\Serial%d"
KERNEL_SERIAL_SCAN = 64
# Codes that mean "this port name does not resolve to a live device" -> try the kernel path.
NAME_BROKEN_WINERRORS = (2, 433, 1167)  # FILE_NOT_FOUND, NO_SUCH_DEVICE, DEVICE_NOT_CONNECTED


def open_error_winerror(exc):
    """pyserial embeds the original WindowsError repr, e.g. OSError(22, '...', None, 433)."""
    match = re.search(r"None,\s*(\d+)\)\s*$", str(exc))
    if match:
        return int(match.group(1))
    for attr in ("winerror", "errno"):
        value = getattr(exc, attr, None)
        if isinstance(value, int):
            return value
    return None


def open_error_kind(exc):
    """Classify a pyserial open failure. pyserial only embeds the Win32 text."""
    text = str(exc)
    if open_error_winerror(exc) in NAME_BROKEN_WINERRORS:
        return "missing"
    if re.search(r"FileNotFoundError|ERROR_FILE_NOT_FOUND|系统找不到|指定的设备不存在|NO_SUCH_DEVICE", text):
        return "missing"
    if re.search(r"PermissionError|Access is denied|拒绝访问", text):
        return "busy"
    if re.search(r"信号灯超时|semaphore|timeout", text, re.IGNORECASE):
        return "unresponsive"
    return None


def open_error_text(port, exc):
    kind = open_error_kind(exc)
    if kind == "missing":
        return (f"{port} 打不开：Windows 找不到该端口名（COM 号冲突或残留）。"
                f"请重新插拔 USB 后刷新列表，或在设备管理器把该串口改到空闲 COM 号")
    if kind == "busy":
        return f"{port} 被其他程序占用，请关闭占用该串口的软件后重试"
    if kind == "unresponsive":
        return f"{port} 设备无响应（蓝牙虚拟串口常见），请确认线缆与设备已连接"
    return f"{port} 打开失败：{exc}"


class DeviceError(Exception):
    pass


class Device:
    def __init__(self):
        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.pending = {}
        self.seq = 0
        self.serial = None
        self.thread = None
        self.stop_event = threading.Event()
        self.parser = Parser()
        self.connected = False
        self.demo = False
        self.info = {}
        self.caps = {}
        self.telemetry = {}
        self.received = {}
        self.events = deque(maxlen=1000)
        self.samples = deque(maxlen=12000)
        self.sample_id = 0
        self.session_id = 0
        self.trace = None
        self.pid_shadow = {}
        self.pid_read = set()
        self.debug_configured = set()
        self.last_ui = time.monotonic()
        self.lease_expired = False
        self.last_error = ""
        self.port = ""
        self.port_path = ""
        self.rx_bytes = self.tx_bytes = self.ack_frames = 0
        self.auto_restore = True
        self.restore = {}
        self._restore_thread = None

    def log(self, message, level="info"):
        with self.lock:
            self.events.append(dict(time=time.strftime("%H:%M:%S"), date_time=time.strftime("%Y-%m-%d %H:%M:%S"), message=message, level=level))

    @staticmethod
    def ports():
        from serial.tools import list_ports
        return [dict(port=p.device, description=p.description, vid=p.vid, pid=p.pid)
                for p in list_ports.comports()]

    def _open_transport(self, path):
        """Open one serial device path. Callers own the returned transport."""
        import serial
        transport = serial.Serial(port=None, baudrate=115200, bytesize=8,
            parity="N", stopbits=1, timeout=0, write_timeout=0.2,
            xonxoff=False, rtscts=False, dsrdtr=False)
        # Keep DTR/RTS inactive before opening: many boards reset on DTR.
        transport.dtr = False
        transport.rts = False
        transport.port = path
        try:
            transport.open()
            transport.reset_input_buffer()
        except Exception:
            transport.close()
            raise
        return transport

    def _probe_mcu(self, transport, timeout=0.8):
        """Synchronous INFO handshake used to identify the MCU on an unknown path."""
        seq = 0x4000
        parser = Parser()
        try:
            transport.reset_input_buffer()
            transport.write(Frame(KIND["REQUEST"], seq, CMD["INFO"], 0, b"").encode())
        except Exception:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                size = min(4096, max(1, getattr(transport, "in_waiting", 1)))
                raw = transport.read(size)
            except Exception:
                return None
            if not raw:
                time.sleep(0.005)
                continue
            for f in parser.feed(raw):
                if f.seq != seq or f.opcode != CMD["INFO"] or f.kind not in (KIND["ACK"], KIND["NACK"]):
                    continue
                if f.kind != KIND["ACK"]:
                    return None
                try:
                    info = unpack("info", f.payload)
                except (ValueError, struct.error):
                    return None
                return info if info["schema_crc"] == SCHEMA_CRC else None
        return None

    def _open_selected(self, port):
        """Open the selected port, falling back to the kernel device path when the
        Windows COM name itself is broken (ERROR_FILE_NOT_FOUND on a started device)."""
        try:
            return self._open_transport(port), port
        except Exception as exc:
            if open_error_kind(exc) != "missing":
                raise DeviceError(open_error_text(port, exc)) from exc
            self.log(f"{port} 打不开：Windows 找不到该端口名；正在通过内核设备路径搜索同一串口设备", "error")
        for index in range(KERNEL_SERIAL_SCAN):
            candidate = KERNEL_SERIAL_PATH % index
            try:
                transport = self._open_transport(candidate)
            except Exception:
                continue
            if self._probe_mcu(transport):
                self.log(f"已改用内核设备路径 {candidate} 连接；{port} 的 COM 名字冲突或残留，"
                         f"建议在设备管理器把该串口改到空闲 COM 号", "error")
                return transport, candidate
            transport.close()
        raise DeviceError(
            f"{port} 打不开：Windows 找不到该端口名（COM 号冲突或残留），"
            f"且已扫描 {KERNEL_SERIAL_SCAN} 个内核串口对象仍未找到本机 MCU。"
            f"请重新插拔 USB、刷新列表，或在设备管理器把该串口改到空闲 COM 号")

    def connect(self, port, demo=False, auto_restore=None):
        if auto_restore is not None:
            self.auto_restore = bool(auto_restore)
        with self.operation_lock:
            if self.serial is not None:
                raise DeviceError("请先断开当前连接")
            self.demo = demo
            self.port = "演示设备" if demo else port
            self.port_path = ""
            self.log(f"开始连接 · {self.port} · 接收方式 available/nonblocking V1.1.1")
            self.rx_bytes = self.tx_bytes = self.ack_frames = 0
            self.parser = Parser()
            self.stop_event = threading.Event()
            with self.lock:
                self.session_id += 1
                self.info, self.caps, self.telemetry, self.received = {}, {}, {}, {}
                self.samples.clear()
                self.trace = None
                self.pid_shadow.clear()
                self.pid_read.clear()
                self.debug_configured.clear()
                self.last_error = ""
                self.last_ui = time.monotonic()
                self.lease_expired = False
            if demo:
                self.serial = DemoSerial()
                self.port_path = ""
            else:
                if port not in [p["port"] for p in self.ports()]:
                    raise DeviceError("串口不存在，请刷新列表")
                self.serial, self.port_path = self._open_selected(port)
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            try:
                self.info = self.request("INFO", response="info")
                if self.info["schema_crc"] != SCHEMA_CRC:
                    raise DeviceError("协议定义不一致，已禁止控制；请使用配套 MCU 协议")
                if not 500 <= self.info["watchdog_ms"] <= 1500:
                    raise DeviceError("MCU 通信看门狗不符合协议要求")
                for ch in (1, 2):
                    caps = self.request("CAPS", ch, response="caps")
                    if any(caps[k] < 0 for k in ("vmax_mv", "imax_ma", "ilmax_ma")):
                        raise DeviceError("设备量程无效")
                    self.caps[ch] = caps
                    status = self.request("STATUS", ch, response="telemetry")
                    self._telemetry(ch, status)
                self.connected = True
                route = "" if self.port_path in ("", self.port) else f"（内核路径 {self.port_path}）"
                self.log(f"{'演示连接' if demo else '串口握手成功'} · {self.port}{route} · {self.info['firmware']}")
            except Exception:
                self.disconnect()
                raise
        if self.auto_restore:
            self.start_auto_restore()

    # ---- per-device restore after a MCU reset -------------------------------
    # The MCU keeps the gate-map confirmation and the current zeros in RAM, so a
    # reset clears them. Replaying the stored gate mapping is one command; the
    # zeros cannot be written from the host (the protocol only starts a
    # measurement), so they are re-measured automatically instead. The MCU still
    # validates every condition, so a refused measurement is reported, not forced.
    def start_auto_restore(self):
        if self._restore_thread and self._restore_thread.is_alive():
            return
        self._restore_thread = threading.Thread(target=self._auto_restore, daemon=True)
        self._restore_thread.start()

    def _zero_conditions_ok(self):
        status = self.telemetry.get(1)
        if not status or time.monotonic() - self.received.get(1, 0) > 0.5:
            return False
        return (status["state"] == 2 and not status["enabled"] and not status["fault"]
                and status["mode"] == 0 and status["vout_mv"] < 200
                and abs(status["iout_ma"]) < 50 and 17000 <= status["vin_mv"] <= 23000)

    def _auto_zero(self, sensor):
        name = "输出电流" if sensor == 1 else "电感电流"
        self.request("ZERO", 1, "zero_request", {"action": sensor}, "zero_status")
        deadline = time.monotonic() + 12
        while self.connected and time.monotonic() < deadline:
            time.sleep(0.5)
            z = self.request("ZERO", 1, "zero_request", {"action": 0}, "zero_status")
            if z["state"] in (1, 2):
                continue
            if z["state"] == 3:
                self.log(f"已自动重新校准{name}零点：{z['iout_uv'] / 1e6 if sensor == 1 else z['il_uv'] / 1e6:.6f} V"
                         f"（本次实测 {z['samples']}/4096 点）")
                return True
            self.log(f"自动校准{name}零点失败：{ZERO_REASON_TEXT[z['reason']] if z['reason'] < len(ZERO_REASON_TEXT) else z['reason']}", "error")
            return False
        self.log(f"自动校准{name}零点超时，未应用；可手动点“开始校准”", "error")
        return False

    def _auto_restore(self):
        session = self.session_id
        uid = (self.info or {}).get("device_id", "")
        report = dict(state="running", uid=uid, known=False, gate="pending", zero="pending",
                      gate_map=None, zero_values={}, message="")
        with self.lock:
            self.restore = report
        try:
            prof = profiles.get(uid)
            report["known"] = bool(prof)
            if not prof:
                self.log(f"该设备（UID {uid or '未知'}）没有历史记录：请手动做一次栅极实测与电流校零，"
                         f"之后每次上电都会自动恢复")
            g = self.request("GATE_TEST", 1, "gate_test_request",
                             {"action": 0, "index": 0, "duration_ms": 0}, "gate_test_status")
            report["gate_map"] = g["mapping"]
            if g["confirmed"]:
                report["gate"] = "already"
            elif prof and prof.get("gate_map") is not None:
                g = self.request("GATE_TEST", 1, "gate_test_request",
                                 {"action": 2, "index": int(prof["gate_map"]), "duration_ms": 0},
                                 "gate_test_status")
                report["gate_map"] = g["mapping"]
                report["gate"] = "restored" if g["confirmed"] else "failed"
                self.log(f"已按上次实测自动确认栅极映射：{GATE_MAP_TEXT[g['mapping']]}（UID {uid}）；"
                         f"接线有变请重做栅极实测", "info" if g["confirmed"] else "error")
            else:
                report["gate"] = "manual"
                self.log("栅极映射未确认，且没有该设备的记忆记录：请先做一次栅极实测并确认", "error")
            z = self.request("ZERO", 1, "zero_request", {"action": 0}, "zero_status")
            missing = [s for s, bit in ((1, 1), (2, 2)) if not z["applied"] & bit]
            if not missing:
                report["zero"] = "already"
            elif not self._zero_conditions_ok():
                report["zero"] = "unsafe"
                self.log("未自动校零：需要先停机、切回正常模式，且 VIN 在 17–23 V、VOUT < 200 mV、无电流", "error")
            else:
                ok = all(self._auto_zero(s) for s in missing)
                report["zero"] = "remeasured" if ok else "failed"
            z = self.request("ZERO", 1, "zero_request", {"action": 0}, "zero_status")
            report["zero_values"] = dict(iout_uv=z["iout_uv"], il_uv=z["il_uv"], applied=z["applied"])
            report["state"] = "done"
            profiles.update(uid, gate_map=g["mapping"], gate_confirmed=bool(g["confirmed"]),
                            iout_uv=z["iout_uv"], il_uv=z["il_uv"], applied=z["applied"],
                            firmware=(self.info or {}).get("firmware"))
        except Exception as exc:
            report["state"] = "error"
            report["message"] = str(exc)
            if session == self.session_id:
                self.log(f"自动恢复未完成：{exc}", "error")
        finally:
            with self.lock:
                if session == self.session_id:
                    self.restore = report

    def disconnect(self):
        stop_confirmed = False
        if self.serial and not self.stop_event.is_set():
            try:
                self.request("STOP_ALL", timeout=0.4)
                stop_confirmed = True
            except Exception:
                self.log("停机命令未确认，实际输出状态未知；依赖 MCU 通信超时停机", "error")
        self.connected = False
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=1)
        transport, self.serial = self.serial, None
        if transport:
            transport.close()
        with self.lock:
            for pending in self.pending.values():
                pending["error"] = "连接已关闭"
                pending["event"].set()
            self.pending.clear()
        self.log("已断开连接" + (" · 停机命令已确认" if stop_confirmed else ""))
        return {"stop_ack": stop_confirmed}

    def _send(self, frame):
        with self.write_lock:
            if self.serial is None or self.stop_event.is_set():
                raise DeviceError("串口未连接")
            if self.lease_expired and frame.opcode == CMD["OUTPUT"] and frame.payload == b"\x01":
                raise DeviceError("会话已失效，启动命令未发送")
            raw = frame.encode()
            if self.serial.write(raw) != len(raw):
                raise DeviceError("串口未完整写入，命令状态未知")
            self.tx_bytes += len(raw)

    def request(self, name, ch=0, layout=None, data=None, response=None, timeout=0.8):
        with self.lock:
            self.seq = self.seq % 65535 + 1
            seq = self.seq
            pending = {"event": threading.Event(), "opcode": CMD[name], "channel": ch}
            self.pending[seq] = pending
        try:
            self._send(Frame(KIND["REQUEST"], seq, CMD[name], ch, pack(layout, data) if layout else b""))
            if not pending["event"].wait(timeout):
                raise DeviceError(f"{name} 响应超时；命令结果未知，未自动重试 "
                                  f"[port={self.port} seq={seq} TX={self.tx_bytes} RX={self.rx_bytes} "
                                  f"ACK={self.ack_frames} CRC={self.parser.bad_frames}]")
            if "error" in pending:
                raise DeviceError(pending["error"])
            f = pending["frame"]
            if f.kind == KIND["NACK"]:
                code = unpack("error", f.payload)["code"]
                raise DeviceError(ERROR_TEXT.get(code, f"设备错误 {code}"))
            return unpack(response, f.payload) if response else f.payload
        finally:
            with self.lock:
                self.pending.pop(seq, None)

    def _telemetry(self, ch, data):
        if ch not in (1, 2):
            return
        with self.lock:
            old = self.telemetry.get(ch)
            if old and old["fault"] != data["fault"]:
                self.log(f"CH{ch} 故障位：0x{data['fault']:08X}", "error" if data["fault"] else "info")
            self.telemetry[ch] = data
            self.received[ch] = time.monotonic()
            self.sample_id += 1
            self.samples.append(dict(id=self.sample_id, channel=ch, host_time=time.time(), **data))

    def _reader(self):
        heartbeat = 0
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                # A frozen/closed browser must not be kept alive by a background server.
                telemetry_lost = self.connected and any(now - self.received.get(ch, 0) > 0.5 for ch in (1, 2))
                if now - self.last_ui > 1.5 or telemetry_lost:
                    if not self.lease_expired:
                        self.lease_expired = True
                        self._send(Frame(KIND["REQUEST"], 0, CMD["STOP_ALL"]))
                        reason = "设备遥测超时" if telemetry_lost else "界面心跳中断"
                        self.log(f"{reason}：已发送停机，停止续期设备心跳", "error")
                elif not self.lease_expired and now - heartbeat >= 0.25:
                    self._send(Frame(KIND["REQUEST"], 0, CMD["HEARTBEAT"]))
                    heartbeat = now
                # Real serial reads must not wait for a large requested block.
                # timeout=0 plus a bounded available-byte read keeps heartbeats
                # and ACK processing on this reader responsive between packets.
                size = 4096 if self.demo else min(4096, max(1, getattr(self.serial, "in_waiting", 1)))
                raw = self.serial.read(size)
                if not raw:
                    self.stop_event.wait(0.002)
                    continue
                self.rx_bytes += len(raw)
                for f in self.parser.feed(raw):
                    if f.kind == KIND["EVENT"] and f.opcode == CMD["TELEMETRY"] and f.seq == 0:
                        try:
                            self._telemetry(f.channel, unpack("telemetry", f.payload))
                        except (ValueError, struct.error):
                            self.log("丢弃格式错误的遥测", "error")
                    elif f.kind == KIND["EVENT"] and f.opcode == CMD["LOG"] and f.seq == 0:
                        try:
                            entry = unpack("log", f.payload)
                            if entry['code'] == 12 and f.channel in (1, 2) and entry['message'].startswith('SHADOW '):
                                with self.lock:
                                    self.pid_shadow[f.channel] = dict(message=entry['message'],
                                        uptime_ms=entry['uptime_ms'], received=time.monotonic())
                            self.log(f"MCU [{entry['uptime_ms']} ms / CH{f.channel} / {entry['code']}] {entry['message']}", "error" if entry['level'] >= 2 else "info")
                        except (ValueError, struct.error):
                            self.log("丢弃格式错误的设备日志", "error")
                    elif f.kind in (KIND["ACK"], KIND["NACK"]):
                        self.ack_frames += 1
                        with self.lock:
                            p = self.pending.get(f.seq)
                            if p and p["opcode"] == f.opcode and p["channel"] == f.channel:
                                p["frame"] = f
                                p["event"].set()
        except Exception as exc:
            if not self.stop_event.is_set():
                self.last_error = str(exc)
                self.connected = False
                self.log(f"通信中断：{exc}；输出状态未知", "error")
                with self.lock:
                    for p in self.pending.values():
                        p["error"] = "串口连接中断，命令结果未知"
                        p["event"].set()
                self.stop_event.set()

    def snapshot(self, since=0, touch=True):
        with self.lock:
            if touch:
                self.last_ui = time.monotonic()
            age = {ch: round(time.monotonic() - t, 3) for ch, t in self.received.items()}
            return copy.deepcopy(dict(connected=self.connected, opened=self.serial is not None,
                demo=self.demo, port=self.port, port_path=self.port_path, info=self.info, caps=self.caps,
                telemetry=self.telemetry, age=age, lease_expired=self.lease_expired,
                events=list(self.events), samples=[s for s in self.samples if s["id"] > since][-400:],
                cursor=self.sample_id, session_id=self.session_id, crc_errors=self.parser.bad_frames, last_error=self.last_error,
                schema_crc=f"{SCHEMA_CRC:08X}", trace=self.trace,
                auto_restore=bool(self.auto_restore), restore=dict(self.restore),
                pid_shadow={ch: dict(message=d['message'], uptime_ms=d['uptime_ms'],
                    age=time.monotonic()-d['received']) for ch,d in self.pid_shadow.items()}))

    def _check(self, ch, feature=0, off=False, allow_fault=False):
        if not self.connected or self.lease_expired:
            raise DeviceError("连接未就绪或界面心跳已过期，请重新连接")
        if ch not in (1, 2) or ch not in self.caps:
            raise DeviceError("通道无效")
        if time.monotonic() - self.received.get(ch, 0) > 0.5:
            raise DeviceError("测量数据已过期，禁止操作")
        if feature and not self.caps[ch]["features"] & feature:
            raise DeviceError("当前固件未开放此功能")
        if off and (self.telemetry[ch]["enabled"] or self.telemetry[ch]["state"] not in ((2, 8) if allow_fault else (2,))):
            raise DeviceError("请先停机并解除故障，等待设备回报 OFF")

    @staticmethod
    def integer(value, low, high):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or int(value) != value or not low <= value <= high:
            raise DeviceError(f"数值必须是 {low} 至 {high} 范围内的整数")
        return int(value)

    def action(self, name, ch=0, values=None):
        values = values or {}
        if name == "stop_all":
            self.request("STOP_ALL")
            self.log("全部停机命令已确认，等待状态回报")
            return {}
        if name == "output" and values.get("enabled") == 0:
            self.request("OUTPUT", ch, "output", {"enabled": 0}, "output")
            self.log(f"CH{ch} 停机命令已确认，等待状态回报")
            return {}
        with self.operation_lock:
            self._check(ch)
            caps = self.caps[ch]
            if name == "target":
                self._check(ch, 1)
                d = dict(vref_mv=self.integer(values.get("vref_mv"), caps["vmin_mv"], caps["vmax_mv"]),
                         iref_ma=self.integer(values.get("iref_ma"), 0, caps["imax_ma"]))
                result = self.request("SET_TARGET", ch, "target", d, "target")
            elif name == "mode":
                mode = self.integer(values.get("mode"), 0, 1)
                self._check(ch, 2 if mode else 1, off=True)
                result = self.request("SET_MODE", ch, "mode", {"mode": mode}, "mode")
                self.debug_configured.discard(ch)
            elif name == "output":
                self.integer(values.get("enabled"), 1, 1)
                status = self.telemetry[ch]
                self._check(ch, 2 if status["mode"] else 1)
                if status["fault"] or not status["ready"]:
                    raise DeviceError("设备有故障或硬件验收未完成，无法启动")
                if status["mode"] == 1 and ch not in self.debug_configured:
                    raise DeviceError("本次会话请先明确设置受限电流目标和运行时长")
                result = self.request("OUTPUT", ch, "output", {"enabled": 1}, "output")
            elif name == "clear_fault":
                self._check(ch, off=True, allow_fault=True)
                self.request("CLEAR_FAULT", ch)
                result = {}
            elif name == "zero":
                self._check(ch, SCHEMA['features']['ZERO_CALIBRATION'])
                action = self.integer(values.get('action'), 0, 4)
                if action:
                    self._check(ch, off=True)
                    if self.telemetry[ch]['mode'] != 0:
                        raise DeviceError('校零前请切回正常稳压 / 限流模式')
                result = self.request('ZERO', ch, 'zero_request', {'action': action}, 'zero_status')
                if result['reserved'] != 0 or result['state'] > 4 or result['sensor'] > 2 or result['applied'] > 3 or result['reason'] > 5:
                    raise DeviceError('设备返回的校零状态无效')
                if action:
                    self.log(f"CH{ch} 校零操作 {action} 已确认；以校零完成状态为准，ACK 不代表已应用")
                if action == 0 and result['applied'] and self.info.get('device_id'):
                    # Keep the per-device memory current so the next reset can restore.
                    profiles.update(self.info['device_id'], iout_uv=result['iout_uv'],
                                    il_uv=result['il_uv'], applied=result['applied'],
                                    firmware=self.info.get('firmware', ''))
                return result
            elif name == "gate_test":
                self._check(ch)
                action = self.integer(values.get("action"), 0, 3)
                if action == 1:
                    index = self.integer(values.get("index"), 1, 4)
                    duration = self.integer(values.get("duration_ms"), 100, 2000)
                    self._check(ch, off=True)
                    result = self.request("GATE_TEST", ch, "gate_test_request",
                                          {"action": action, "index": index, "duration_ms": duration},
                                          "gate_test_status")
                    self.log(f"CH{ch} 栅极测试：正在驱动 PA{[8, 9, 10, 11][index - 1]}，"
                             f"{duration} ms 后自动关闭；请勿接通 20 V 输入")
                elif action == 2:
                    mapping = self.integer(values.get("mapping"), 0, 3)
                    self._check(ch, off=True)
                    result = self.request("GATE_TEST", ch, "gate_test_request",
                                          {"action": action, "index": mapping, "duration_ms": 0},
                                          "gate_test_status")
                    self.log(f"CH{ch} 栅极映射已确认：{['设计意图', '输入桥反相', '输出桥反相', '两桥都反相'][mapping]}")
                    if self.info.get('device_id'):
                        profiles.update(self.info['device_id'], gate_map=mapping, gate_confirmed=True,
                                        firmware=self.info.get('firmware', ''))
                else:
                    result = self.request("GATE_TEST", ch, "gate_test_request",
                                          {"action": action, "index": 0, "duration_ms": 0},
                                          "gate_test_status")
                    if action == 3 and self.info.get('device_id'):
                        profiles.update(self.info['device_id'], gate_confirmed=False)
                if result["confirmed"] > 1 or result["mapping"] > 3 or result["active_index"] > 4:
                    raise DeviceError("设备返回的栅极状态无效")
                return result
            elif name == "get_pid":
                self._check(ch, 4)
                loop = self.integer(values.get("loop"), 0, 2)
                result = self.request("GET_PID", ch, "loop", {"loop": loop}, "pid")
                self.pid_read.add((ch, loop))
            elif name == "set_pid":
                self._check(ch, 4, off=True)
                loop = self.integer(values.get("loop"), 0, 2)
                if (ch, loop) not in self.pid_read:
                    raise DeviceError("请先读取此通道、此环路的设备 PID 参数")
                d = {k: float(values[k]) for k in ("kp", "ki", "kd", "tau", "out_min", "out_max")}
                if not all(math.isfinite(v) and abs(v) <= 1e9 for v in d.values()):
                    raise DeviceError("PID 参数必须为有限数值且绝对值不超过 1e9")
                if min(d[k] for k in ("kp", "ki", "kd", "tau")) < 0 or d["out_min"] >= d["out_max"] or (d["kd"] > 0 and d["tau"] <= 0):
                    raise DeviceError("增益与滤波时间须非负、下限小于上限；启用 D 时滤波时间必须大于零")
                d["loop"] = loop
                self.request("SET_PID", ch, "pid", d, "pid")
                result = self.request("GET_PID", ch, "loop", {"loop": loop}, "pid")
                expected = unpack("pid", pack("pid", d))
                if result != expected:
                    raise DeviceError("PID 回读与下发值不同，参数未验证；请重新读取")
            elif name == "debug_ref":
                self._check(ch, 2, off=True)
                if self.telemetry[ch]["mode"] != 1:
                    raise DeviceError("请先切换到电感电流调试模式")
                d = dict(ilref_ma=self.integer(values.get("ilref_ma"), 0, caps["ilmax_ma"]),
                         duration_ms=self.integer(values.get("duration_ms"), 10, caps["max_debug_ms"]))
                result = self.request("DEBUG_REF", ch, "debug_ref", d, "debug_ref")
                self.debug_configured.add(ch)
            elif name == "demo_fault":
                if not self.demo:
                    raise DeviceError("仅演示设备可注入故障")
                self.serial.fault(ch)
                result = {}
            else:
                raise DeviceError("未知操作")
            self.log(f"CH{ch} · {name} · 已确认" + ("并回读验证" if name == "set_pid" else ""))
            return result

    def capture(self, ch, points, decimation):
        self._check(ch, 8)
        caps = self.caps[ch]
        d = dict(points=self.integer(points, 16, min(256, caps["trace_max_points"])), decimation=self.integer(decimation, 1, 1000))
        if caps["trace_base_hz"] <= 0 or caps["trace_base_hz"] % d["decimation"]:
            raise DeviceError("抽取系数必须整除设备基础采样率")
        meta = self.request("TRACE_ARM", ch, "trace_arm", d, "trace_info")
        capture_id = meta["capture_id"]
        deadline = time.monotonic() + 4
        while meta["state"] != 2:
            if meta["state"] != 1 or time.monotonic() > deadline:
                raise DeviceError("采集失败或超时")
            time.sleep(0.05)
            meta = self.request("TRACE_INFO", ch, response="trace_info")
            if meta["capture_id"] != capture_id:
                raise DeviceError("波形采集编号发生变化")
        if meta["points"] != d["points"] or meta["sample_hz"] != caps["trace_base_hz"] // d["decimation"]:
            raise DeviceError("设备返回的波形长度或采样率不符合请求")
        points_out = []
        while len(points_out) < meta["points"]:
            offset = len(points_out)
            query = dict(capture_id=capture_id, offset=offset, count=min(24, meta["points"]-offset))
            payload = self.request("TRACE_READ", ch, "trace_read", query)
            size = LAYOUTS["trace_chunk"].size
            header = unpack("trace_chunk", payload[:size])
            if header != query or len(payload) != size + query["count"] * LAYOUTS["trace_point"].size:
                raise DeviceError("波形分片编号或长度不符")
            for i in range(query["count"]):
                start = size + i * LAYOUTS["trace_point"].size
                points_out.append(unpack("trace_point", payload[start:start+LAYOUTS["trace_point"].size]))
        with self.lock:
            self.trace = dict(channel=ch, demo=self.demo, **meta, samples=points_out)
        self.log(f"CH{ch} 快照完成 · {meta['points']} 点 / {meta['sample_hz']} Hz")
        return self.trace
