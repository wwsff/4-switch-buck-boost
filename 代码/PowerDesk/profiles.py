"""Per-device bench profile: what was measured for this exact MCU.

The firmware keeps the gate-map confirmation and the current zeros in RAM, so
every MCU reset (flash download, power cycle, USB re-plug) clears them and the
operator had to redo both by hand. PowerDesk remembers the result per device
(INFO device_id) and replays it on connect:

  * the gate mapping is re-confirmed with one GATE_TEST command, because the
    mapping is a property of the wiring, not of the session;
  * the current zeros cannot be written from the host - the protocol only starts
    a measurement - so they are re-measured automatically instead. Same result,
    no clicks, and the MCU still validates the conditions (no current, VIN
    window, sampling healthy).

The file lives next to this module (override with POWERDESK_PROFILE). A missing
or corrupt file is treated as "no profile yet": nothing is restored and the
operator is told to do the manual steps once.
"""
import json
import os
import time
from pathlib import Path

PATH = Path(os.environ.get("POWERDESK_PROFILE", Path(__file__).resolve().parent / "profiles.json"))
VERSION = 1


def _read():
    try:
        data = json.loads(PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"version": VERSION, "devices": {}}
    if not isinstance(data, dict) or not isinstance(data.get("devices"), dict):
        return {"version": VERSION, "devices": {}}
    data.setdefault("version", VERSION)
    return data


def get(uid):
    """Return the stored profile for a device id, or None."""
    if not uid:
        return None
    entry = _read()["devices"].get(uid)
    return entry if isinstance(entry, dict) else None


def update(uid, **fields):
    """Merge fields into the profile for uid and persist it. Returns the entry."""
    if not uid:
        return None
    data = _read()
    entry = data["devices"].setdefault(uid, {})
    if not isinstance(entry, dict):
        entry = {}
        data["devices"][uid] = entry
    entry.update(fields)
    entry["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp = PATH.with_suffix(PATH.suffix + ".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        tmp.replace(PATH)
    except OSError:
        # A read-only install must not break the session: the profile is a
        # convenience, the MCU remains the authority.
        return entry
    return entry


def clear(uid):
    if not uid:
        return
    data = _read()
    if data["devices"].pop(uid, None) is not None:
        try:
            PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except OSError:
            pass
