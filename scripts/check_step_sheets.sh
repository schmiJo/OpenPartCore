#!/bin/sh
# Has OpenCASCADE read sheets written here and checks them against formulas, then has this kernel read sheets OpenCASCADE wrote.
# Usage: scripts/check_step_sheets.sh <scratch folder>. Needs the neighbouring FreeCAD build that occt.sh uses.
set -e
work="$1"
mkdir -p "$work"
here=$(cd "$(dirname "$0")" && pwd)
rm -f "$work"/*.step "$work/sheets.json"
cargo run -q --release -p coreshape --example sheets_to_step -- write "$work"
sh "$here/occt.sh" "$work" "$here/check_step_sheets.py" 200 || true
cargo run -q --release -p coreshape --example sheets_to_step -- read "$work" > "$work/read_back.txt"
python3 -I - "$work" <<'PY'
import json, sys
work = sys.argv[1]
got = json.load(open(work + "/sheets.json"))
bad = 0
print("Written here, read by OpenCASCADE:")
for line in open(work + "/expected.txt"):
    name, area, boundary, shells = line.split()
    r = got["read"].get(name)
    if r is None:
        print("  WRONG", name, "not read")
        bad += 1
        continue
    # A curved edge's area is integrated by each kernel in its own way, so to 1 part in a million.
    fine = abs(r["area"] - float(area)) < 1e-6 * max(float(area), 1.0) and r["valid"] and r["shells"] == int(shells)
    print("  " + ("ok   " if fine else "WRONG"), name, "area", r["area"], "of", area, "valid", r["valid"], "shells", r["shells"], "faces", r["faces"], "free edges", r["free"], "of", boundary)
    bad += not fine
print("Written by OpenCASCADE, read here:")
for line in open(work + "/read_back.txt"):
    parts = line.split(None, 2)
    name = parts[0][len("occt_"):-len(".step")]
    want = got["wrote"][name]
    if parts[1] != "ok":
        print("  REFUSED", name, parts[2].strip())
        bad += 1
        continue
    area, sheets, count, valid, boundary = line.split()[2:]
    fine = abs(float(area) - want["area"]) < 1e-6 * max(want["area"], 1.0) and valid == "true"
    print("  " + ("ok   " if fine else "WRONG"), name, "area", area, "of", want["area"], "bodies", count, "sheets", sheets, "boundary edges", boundary)
    bad += not fine
print("wrong:", bad)
sys.exit(1 if bad else 0)
PY
