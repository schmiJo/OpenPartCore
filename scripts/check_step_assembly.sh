#!/bin/sh
# Has OpenCASCADE read a STEP assembly written here, and checks what it finds against formulas.
# Usage: scripts/check_step_assembly.sh <scratch folder>. Needs the neighbouring FreeCAD build that occt.sh uses.
set -e
work="$1"
mkdir -p "$work"
here=$(cd "$(dirname "$0")" && pwd)
cargo run -q --release -p coreshape-assembly --example write_step_assembly -- "$work"
rm -f "$work/assembly.json"
sh "$here/occt.sh" "$work" "$here/check_step_assembly.py" 200 || true
python3 -I - "$work/assembly.json" <<'PY'
import json, math, sys
got = json.load(open(sys.argv[1]))
pi = math.pi
root3 = 1 / math.sqrt(3)
pin = pi * 9 * 10
want = {
    "Plate": (40 * 20 * 5, (20, 10, 2.5)),
    "Pin 1": (pin, (10, 10, 10)),
    "Pin 2": (pin, (30, 10, 10)),
    "Pin 3": (pin, (100 + 5 * root3, 20 + 5 * root3, 30 + 5 * root3)),
    "Ball": (4 / 3 * pi * 64, (-50, 0, 0)),
}
print("valid:", got["valid"], " solids:", len(got["solids"]))
bad = 0
for name, (volume, center) in want.items():
    found = [s for s in got["solids"] if abs(s[0] - volume) < 1e-6 and math.dist(s[1:], center) < 1e-6]
    print(("ok " if len(found) == 1 else "WRONG "), name, volume, center)
    bad += len(found) != 1
if not got["valid"] or len(got["solids"]) != len(want):
    bad += 1
print("structure:", got.get("objects", got.get("objects_error")))
sys.exit(1 if bad else 0)
PY
