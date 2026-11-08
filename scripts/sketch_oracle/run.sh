#!/bin/sh
# Runs PlaneGCS over <dir>/cases.jsonl through the neighbouring FreeCAD build, which is only read.
# All of its settings and caches go in <dir>. Usage: run.sh <dir> [timeout seconds]
root="$(cd "$(dirname "$0")/../.." && pwd)"
exec "$root/scripts/occt.sh" "$1" "$root/scripts/sketch_oracle/planegcs.py" "${2:-1800}"
