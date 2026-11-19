#!/usr/bin/env python3
"""Embed quadruped actions (from twist_gazebo hardware) and the room occupancy map."""
from __future__ import annotations

import csv
import shutil
import struct
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GAZEBO = Path(r"D:\Pony\twist_gazebo")
OUT = ROOT / "main" / "embed"
ACTIONS_AUTO = GAZEBO / "hardware" / "actions auto"
TICKS = GAZEBO / "hardware" / "motor_angles" / "ticks"
DEG = GAZEBO / "hardware" / "motor_angles" / "deg"
MAP_NPZ = GAZEBO / "logs" / "room_r2.npz"

ACTIONS = [
    ("neutral", "neutral.txt", "stand.csv", True),
    ("walk_forward", "walk forward.txt", "walk_forward.csv", True),
    ("walk_backward", "walk backward.txt", "walk_backward.csv", True),
    ("turn_left", "turn left.txt", "turn_left.csv", False),
    ("turn_right", "turn right.txt", "turn_right.csv", False),
]


def deg_to_ticks(deg: float) -> int:
    t = int(round(deg * 1023.0 / 300.0))
    return max(0, min(1023, t))


def load_csv(path: Path, is_ticks: bool) -> list[list[int]]:
    frames: list[list[int]] = []
    with path.open(newline="") as f:
        rows = csv.reader(f)
        header = next(rows)
        for row in rows:
            if not row or row[0].startswith("#"):
                continue
            vals = [float(x) for x in row[1:13]]
            if len(vals) != 12:
                continue
            if is_ticks:
                frames.append([max(0, min(1023, int(round(v)))) for v in vals])
            else:
                frames.append([deg_to_ticks(v) for v in vals])
    if not frames:
        raise SystemExit(f"no frames in {path}")
    return frames


def write_bin(name: str, frames: list[list[int]]) -> None:
    raw = bytearray()
    for fr in frames:
        for t in fr:
            raw += struct.pack("<H", t)
    (OUT / f"{name}.bin").write_bytes(raw)


def npy_payload(blob: bytes) -> bytes:
    if blob[:6] != b"\x93NUMPY":
        raise SystemExit("not an npy")
    major = blob[6]
    if major == 1:
        hlen = struct.unpack_from("<H", blob, 8)[0]
        return blob[10 + hlen :]
    if major == 2:
        hlen = struct.unpack_from("<I", blob, 8)[0]
        return blob[12 + hlen :]
    raise SystemExit(f"npy version {major}")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    docs = OUT / "docs"
    docs.mkdir(exist_ok=True)

    lines = [
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "#define ACTION_SERVOS  12",
        "#define ACTION_DT_MS   30",
        "",
        "typedef struct {",
        "    const char *name;",
        "    const uint16_t *ticks; /* frames * 12, little-endian in flash */",
        "    size_t frames;",
        "} action_t;",
        "",
        "const action_t *action_by_name(const char *name);",
        "size_t action_count(void);",
        "const action_t *action_at(size_t i);",
        "",
        "#define TWIST_MAP_W    400",
        "#define TWIST_MAP_H    400",
        "const int8_t *twist_map_cells(void);",
        "extern const float twist_map_res;",
        "extern const float twist_map_ox;",
        "extern const float twist_map_oy;",
        "",
    ]
    (OUT / "embed.h").write_text("\n".join(lines), encoding="utf-8")

    c_lines = [
        '#include "embed.h"',
        "#include <string.h>",
        "",
    ]
    names = []
    for name, txt, csv_name, is_ticks in ACTIONS:
        src_txt = ACTIONS_AUTO / txt
        if src_txt.exists():
            shutil.copy2(src_txt, docs / f"{name}.txt")
        csv_path = (TICKS if is_ticks else DEG) / csv_name
        frames = load_csv(csv_path, is_ticks)
        write_bin(name, frames)
        sym = name
        c_lines.append(
            f"extern const uint8_t {sym}_bin_start[] asm(\"_binary_{sym}_bin_start\");"
        )
        names.append((name, frames, sym))
        print(f"{name}: {len(frames)} frames from {csv_path.name}")

    c_lines.append("")
    c_lines.append("static action_t s_actions[] = {")
    for name, frames, sym in names:
        c_lines.append(
            f'    {{ "{name}", (const uint16_t *){sym}_bin_start, {len(frames)} }},'
        )
    c_lines.append("};")
    c_lines.append("")
    c_lines.append("size_t action_count(void) { return sizeof(s_actions) / sizeof(s_actions[0]); }")
    c_lines.append("const action_t *action_at(size_t i) {")
    c_lines.append("    return i < action_count() ? &s_actions[i] : NULL;")
    c_lines.append("}")
    c_lines.append("const action_t *action_by_name(const char *name) {")
    c_lines.append("    if (!name) return NULL;")
    c_lines.append("    for (size_t i = 0; i < action_count(); i++) {")
    c_lines.append("        if (strcmp(s_actions[i].name, name) == 0) return &s_actions[i];")
    c_lines.append("    }")
    c_lines.append("    return NULL;")
    c_lines.append("}")
    c_lines.append("")

    with zipfile.ZipFile(MAP_NPZ) as z:
        logodds = npy_payload(z.read("logodds.npy"))
        res = struct.unpack("<d", npy_payload(z.read("resolution.npy")))[0]
        ox, oy = struct.unpack("<dd", npy_payload(z.read("origin.npy")))
    if len(logodds) != 400 * 400:
        raise SystemExit(f"map size {len(logodds)}, expected 160000")
    (OUT / "map_cells.bin").write_bytes(logodds)
    c_lines.append(
        'extern const uint8_t map_cells_bin_start[] asm("_binary_map_cells_bin_start");'
    )
    c_lines.append("const int8_t *twist_map_cells(void)")
    c_lines.append("{")
    c_lines.append("    return (const int8_t *)map_cells_bin_start;")
    c_lines.append("}")
    def c_float(v: float) -> str:
        s = f"{v:.9g}"
        if "." not in s and "e" not in s and "E" not in s:
            s += ".0"
        return s + "f"

    c_lines.append(f"const float twist_map_res = {c_float(res)};")
    c_lines.append(f"const float twist_map_ox = {c_float(ox)};")
    c_lines.append(f"const float twist_map_oy = {c_float(oy)};")
    print(f"map: 400x400 res={res} origin=({ox},{oy}) bytes={len(logodds)}")

    (OUT / "embed.c").write_text("\n".join(c_lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
