import http.client
import json
import threading
import unittest
from server import Server


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.server = Server(("127.0.0.1", 0))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.device.disconnect()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, path, data=None, token=True, origin=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        headers = {"Content-Type": "application/json"}
        if token:
            headers["X-PowerDesk-Token"] = self.server.token
        if origin:
            headers["Origin"] = origin
        conn.request("POST" if data is not None else "GET", path, json.dumps(data) if data is not None else None, headers)
        response = conn.getresponse()
        status, body = response.status, response.read()
        conn.close()
        return status, body

    def test_cross_origin_and_missing_token_denied(self):
        self.assertEqual(self.request("/api/connect", {"demo": True}, token=False)[0], 403)
        self.assertEqual(self.request("/api/connect", {"demo": True}, origin="https://example.org")[0], 403)
        self.assertFalse(self.server.device.connected)

    def test_index_and_assets(self):
        status, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(self.server.token.encode(), body)
        self.assertNotIn(b"__TOKEN__", body)
        for path in ("/style.css", "/app.js", "/favicon.svg"):
            self.assertEqual(self.request(path)[0], 200)

    def test_full_api_lifecycle_and_exports(self):
        self.assertEqual(self.request("/api/connect", {"demo": True})[0], 200)
        status, body = self.request("/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["connected"])
        self.assertEqual(self.request("/api/action", {"name": "target", "channel": 1, "values": {"vref_mv": 5000, "iref_ma": 500}})[0], 200)
        self.assertEqual(self.request("/api/capture", {"channel": 1, "points": 64, "decimation": 1})[0], 200)
        status, csv = self.request("/api/trace.csv")
        self.assertEqual(status, 200)
        self.assertIn(b"DEMO", csv)
        self.assertEqual(len(csv.decode("utf-8-sig").splitlines()), 65)
        self.assertIn(b"vout_mv", self.request("/api/export")[1])
        status, logs = self.request("/api/logs.csv")
        self.assertEqual(status, 200)
        self.assertIn(b"date_time,time,level,message", logs)
        self.assertEqual(self.request("/api/logs.csv", token=False)[0], 403)
        self.assertEqual(self.request("/api/disconnect", {})[0], 200)
        self.assertFalse(self.server.device.connected)

    def test_invalid_request_cannot_start_output(self):
        self.request("/api/connect", {"demo": True})
        self.assertEqual(self.request("/api/action", {"name": "output", "channel": 1, "values": {"enabled": 999}})[0], 400)
        self.assertEqual(self.server.device.telemetry[1]["enabled"], 0)


if __name__ == "__main__":
    unittest.main()
