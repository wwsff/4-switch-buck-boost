"""Generate MCU contract artifacts; never edits the MCU project."""
from pathlib import Path
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from protocol import SCHEMA, SCHEMA_CRC, LAYOUTS, Frame, KIND, CMD, pack, crc16


def generate():
    out = Path(__file__).resolve().parents[1] / "protocol"
    lines = ["/* Generated from schema.json. Do not edit. See PROTOCOL.md. */", "#ifndef POWERDESK_PROTOCOL_H", "#define POWERDESK_PROTOCOL_H", "#include <stdint.h>",
             f"#define PD_PROTOCOL_VERSION {SCHEMA['version']}u", f"#define PD_SCHEMA_CRC32 0x{SCHEMA_CRC:08X}u", "#define PD_MAX_PAYLOAD 512u", "#define PD_BAUD 115200u"]
    for group in ("kinds", "commands", "errors", "states", "features", "faults", "valid_bits", "modes", "loops", "trace_states"):
        for key, val in SCHEMA[group].items():
            lines.append(f"#define PD_{group.upper()}_{key} {val}u")
    lines += ["/* Explicit byte offsets; do not cast unaligned UART bytes to C structs. */"]
    sizes = {"B": 1, "H": 2, "h": 2, "I": 4, "i": 4, "f": 4}
    for name, fields in SCHEMA["layouts"].items():
        lines.append(f"#define PD_{name.upper()}_SIZE {LAYOUTS[name].size}u")
        offset = 0
        for key, typ in fields:
            lines.append(f"#define PD_{name.upper()}_{key.upper()}_OFFSET {offset}u /* {typ} */")
            offset += int(typ[:-1]) if typ.endswith("s") else sizes[typ]
    lines += ["#endif", ""]
    (out / "powerdesk_protocol.h").write_text("\n".join(lines), encoding="utf-8")
    examples = [Frame(KIND["REQUEST"], 1, CMD["INFO"]), Frame(KIND["REQUEST"], 2, CMD["SET_TARGET"], 1, pack("target", {"vref_mv": 5000, "iref_ma": 1000})), Frame(KIND["REQUEST"], 3, CMD["STOP_ALL"])]
    (out / "test_vectors.json").write_text(json.dumps({"schema_crc32": f"{SCHEMA_CRC:08X}", "crc_check_123456789": f"{crc16(b'123456789'):04X}", "frames": [{"opcode": f.opcode, "channel": f.channel, "seq": f.seq, "hex": f.encode().hex(" ")} for f in examples]}, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    generate()
