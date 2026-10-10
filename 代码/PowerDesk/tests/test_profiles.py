import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import profiles


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "profiles.json"
        self.patch = patch.object(profiles, "PATH", self.path)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.tmp.cleanup()

    def test_missing_file_and_unknown_device_return_none(self):
        self.assertIsNone(profiles.get("ABC"))
        self.assertIsNone(profiles.get(""))

    def test_update_merges_and_persists(self):
        profiles.update("ABC", gate_map=0, gate_confirmed=True)
        profiles.update("ABC", iout_uv=1646136, il_uv=1650000, applied=3)
        entry = profiles.get("ABC")
        self.assertEqual(entry["gate_map"], 0)
        self.assertEqual(entry["iout_uv"], 1646136)
        self.assertIn("updated", entry)
        raw = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(raw["version"], profiles.VERSION)
        self.assertEqual(raw["devices"]["ABC"]["applied"], 3)

    def test_devices_are_isolated_and_clear_removes_one(self):
        profiles.update("A", gate_map=1)
        profiles.update("B", gate_map=2)
        self.assertEqual(profiles.get("A")["gate_map"], 1)
        profiles.clear("A")
        self.assertIsNone(profiles.get("A"))
        self.assertEqual(profiles.get("B")["gate_map"], 2)

    def test_corrupt_file_is_treated_as_empty(self):
        self.path.write_text("{not json", encoding="utf-8")
        self.assertIsNone(profiles.get("A"))
        profiles.update("A", gate_map=0)
        self.assertEqual(profiles.get("A")["gate_map"], 0)

    def test_write_failure_does_not_raise(self):
        with patch.object(Path, "write_text", side_effect=OSError("read-only")):
            self.assertEqual(profiles.update("A", gate_map=0)["gate_map"], 0)
