#!/usr/bin/env python3
"""Merge per-package compile_commands.json files into one at the workspace root.

colcon builds each package in its own build/<pkg>/ directory and (when
CMAKE_EXPORT_COMPILE_COMMANDS=ON) emits a compile_commands.json there.
clangd only searches upward from a source file, so we concatenate all
per-package databases into a single file at the workspace root.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
OUT = ROOT / "compile_commands.json"

entries = []
for cdb in sorted(BUILD.glob("*/compile_commands.json")):
    with cdb.open() as f:
        entries.extend(json.load(f))

OUT.write_text(json.dumps(entries, indent=2))
print(f"merge_compile_commands: wrote {len(entries)} entries to {OUT.relative_to(ROOT)}")
