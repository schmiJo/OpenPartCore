#!/bin/sh
# Tries the fillets of crates/kernel/examples/fillet_cases.rs in OpenCASCADE too, and shows them side by side. Also has
# OpenCASCADE read back what this kernel made, which has to be valid there with the same volume.
# Usage: scripts/check_fillets.sh <scratch folder>. Needs the neighbouring FreeCAD build that occt.sh uses.
work="$1"
mkdir -p "$work"
here=$(cd "$(dirname "$0")" && pwd)
cargo run -q --release -p coreshape --example fillet_cases -- "$work"
rm -f "$work/occt_fillets.json"
sh "$here/occt.sh" "$work" "$here/check_fillets.py" 900
python3 -I - "$work/occt_fillets.json" <<'PY'
import json, sys
# OpenCASCADE's integrator is wrong on B-spline faces with many knots, and its fine mesh is right to about 1e-5 at worst.
def best(volume, mesh):
    if volume is None:
        return None
    if mesh is not None and abs(volume - mesh) > 2e-5 * max(abs(mesh), 1.0):
        return mesh
    return volume
print()
print("%-52s %14s %14s  %s" % ("case", "this kernel", "OpenCASCADE", "difference"))
bad_back = 0
for c in json.load(open(sys.argv[1])):
    mine = c["mine"]
    if mine is not None:
        back = best(c.get("back_volume"), c.get("back_mesh"))
        if not c.get("back_valid") or back is None or abs(back - mine) > 2e-5 * max(abs(mine), 1.0):
            bad_back += 1
            print("READ BACK WRONG: %s: %s %s %s %s" % (c["name"], c.get("back_volume"), c.get("back_mesh"), c.get("back_valid"), c.get("back_error", "")))
    other = best(c.get("volume"), c.get("mesh"))
    a = "refused" if mine is None else "%.4f" % mine
    if other is None:
        b = "refused"
        note = c.get("error", "")[:48]
    else:
        b = "%.4f" % other
        note = "" if c.get("valid") else "INVALID in OpenCASCADE"
        if other != c.get("volume"):
            note += " (its mesh: its integrator says %.4f)" % c["volume"]
        if mine is not None:
            note = ("%.1e " % (abs(mine - other) / max(abs(other), 1.0))) + note
    print("%-52s %14s %14s  %s" % (c["name"], a, b, note))
print("read back wrong in OpenCASCADE:", bad_back)
PY
