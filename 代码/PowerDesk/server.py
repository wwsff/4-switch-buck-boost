"""Local-only web application. No cloud services, external assets or accounts."""
import argparse
import csv
import io
import json
import secrets
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit
from device import Device, DeviceError

ROOT = Path(__file__).resolve().parent


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, device=None):
        super().__init__(address, Handler)
        self.device = device or Device()
        self.token = secrets.token_urlsafe(32)
        self.capture_lock = threading.Lock()

    @property
    def origin(self):
        return f"http://127.0.0.1:{self.server_port}"


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def send(self, data, status=200, mime="application/json; charset=utf-8", filename=None):
        if isinstance(data, (dict, list)):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        elif isinstance(data, str):
            data = data.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def authorized(self):
        host = self.headers.get("Host", "")
        if host != f"127.0.0.1:{self.server.server_port}":
            self.send({"error": "Host rejected"}, 403)
            return False
        origin = self.headers.get("Origin")
        if origin and origin != self.server.origin:
            self.send({"error": "Origin rejected"}, 403)
            return False
        if self.headers.get("X-PowerDesk-Token") != self.server.token:
            self.send({"error": "请刷新本机页面后重试"}, 403)
            return False
        return True

    def do_GET(self):
        path = urlsplit(self.path).path
        if self.headers.get("Host") != f"127.0.0.1:{self.server.server_port}":
            self.send({"error": "Host rejected"}, 403)
            return
        if path in ("/", "/app.js", "/style.css", "/favicon.svg"):
            filename = "index.html" if path == "/" else path[1:]
            data = (ROOT / "web" / filename).read_bytes()
            if path == "/":
                data = data.replace(b"__TOKEN__", self.server.token.encode())
            mime = {"/": "text/html; charset=utf-8", "/app.js": "text/javascript; charset=utf-8", "/style.css": "text/css; charset=utf-8", "/favicon.svg": "image/svg+xml"}[path]
            self.send(data, mime=mime)
            return
        if not self.authorized():
            return
        try:
            if path == "/api/state":
                query = parse_qs(urlsplit(self.path).query)
                self.send(self.server.device.snapshot(int(query.get("since", [0])[0])))
            elif path == "/api/ports":
                self.send(self.server.device.ports())
            elif path == "/api/logs.csv":
                with self.server.device.lock:
                    rows = list(self.server.device.events)
                output = io.StringIO(newline="")
                writer = csv.DictWriter(output, fieldnames=["date_time", "time", "level", "message"])
                writer.writeheader()
                writer.writerows(rows)
                self.send(b"\xef\xbb\xbf" + output.getvalue().encode(), mime="text/csv; charset=utf-8", filename="powerdesk_logs.csv")
            elif path == "/api/export":
                with self.server.device.lock:
                    rows = list(self.server.device.samples)
                    demo = self.server.device.demo
                output = io.StringIO(newline="")
                if rows:
                    writer = csv.DictWriter(output, fieldnames=["source"] + list(rows[0]))
                    writer.writeheader()
                    writer.writerows(dict(source="DEMO" if demo else "MCU", **r) for r in rows)
                self.send(b"\xef\xbb\xbf" + output.getvalue().encode(), mime="text/csv; charset=utf-8", filename="powerdesk_telemetry.csv")
            elif path == "/api/trace.csv":
                trace = self.server.device.trace
                if not trace:
                    raise DeviceError("尚无波形快照")
                output = io.StringIO(newline="")
                writer = csv.writer(output)
                fields = ["il_ma", "ilref_ma", "vout_mv", "vref_mv", "duty_in", "duty_out"]
                writer.writerow(["source", "channel", "capture_id", "time_s"] + fields)
                for i, p in enumerate(trace["samples"]):
                    writer.writerow(["DEMO" if trace["demo"] else "MCU", trace["channel"], trace["capture_id"], i / trace["sample_hz"]] + [p[k] for k in fields])
                self.send(b"\xef\xbb\xbf" + output.getvalue().encode(), mime="text/csv; charset=utf-8", filename="powerdesk_capture.csv")
            else:
                self.send({"error": "Not found"}, 404)
        except Exception as exc:
            self.send({"error": str(exc)}, 400)

    def do_POST(self):
        if not self.authorized():
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise DeviceError("请求长度无效")
            body = json.loads(self.rfile.read(length), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            if not isinstance(body, dict):
                raise DeviceError("请求必须是对象")
            path = urlsplit(self.path).path
            device = self.server.device
            if path == "/api/connect":
                device.connect(str(body.get("port", "")), body.get("demo") is True,
                               body.get("auto_restore"))
                result = {"ok": True}
            elif path == "/api/disconnect":
                result = device.disconnect()
            elif path == "/api/action":
                ch = Device.integer(body.get("channel", 0), 0, 2)
                result = device.action(body["name"], ch, body.get("values"))
            elif path == "/api/capture":
                if not self.server.capture_lock.acquire(blocking=False):
                    raise DeviceError("已有波形采集正在进行")
                try:
                    result = device.capture(Device.integer(body.get("channel"), 1, 2), body.get("points"), body.get("decimation"))
                finally:
                    self.server.capture_lock.release()
            elif path == "/api/quit":
                result = device.disconnect()
                threading.Thread(target=self.server.shutdown, daemon=True).start()
            else:
                self.send({"error": "Not found"}, 404)
                return
            self.send({"ok": True, "result": result})
        except Exception as exc:
            self.server.device.log(str(exc), "error")
            self.send({"error": str(exc)}, 400)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=0, help="0 chooses a free local port")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    try:
        import serial  # dependency check before opening the UI
    except ImportError:
        print("Missing pyserial. Run: python -m pip install -r requirements.txt")
        return 1
    server = Server(("127.0.0.1", args.port))
    print(f"PowerDesk: {server.origin}", flush=True)
    if not args.no_browser:
        webbrowser.open(server.origin)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.device.disconnect()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
