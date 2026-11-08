#!/bin/sh
# Reads a folder of STEP files with this kernel and OpenCASCADE and compares them.
# Usage: check_nist.sh <folder of .stp files> <scratch directory>
# It's for NIST's public test parts (NIST-PMI-STEP-Files.zip from nist.gov), which aren't kept in this repository.
set -e
files="$1"; work="$2"
root="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$work"
(cd "$root" && cargo build --release -q -p coreshape --example step_import)
bin="${CARGO_TARGET_DIR:-$root/target}/release/examples/step_import"
: > "$work/mine.csv"
for f in "$files"/*.stp; do timeout 110 "$bin" "$f" >> "$work/mine.csv" 2>/dev/null || echo "$(basename "$f" .stp),refused,timed out" >> "$work/mine.csv"; done
cat > "$work/read.py" <<PY
import os, Part, glob
out = open("$work/occt.csv", "w")
for path in sorted(glob.glob("$files/*.stp")):
    s = Part.read(path)
    out.write(f"{os.path.basename(path)[:-4]},{len(s.Solids)},{s.Volume},{s.Area},{len(s.Faces)},{s.isValid()}\n")
    out.flush()
out.close()
PY
sh "$root/scripts/occt.sh" "$work" "$work/read.py" 300
python3 - "$work" <<'PY'
import csv, sys
work = sys.argv[1]
occt = {r[0]: r for r in csv.reader(open(work + "/occt.csv"))}
read = same = 0
rows = list(csv.reader(open(work + "/mine.csv")))
for r in rows:
    if len(r) < 4 or r[1] != "ok":
        print(f"{r[0]:<52} refused: {','.join(r[2:])[:90]}")
        continue
    read += 1
    o = occt.get(r[0])
    dv = abs(float(r[3]) - float(o[2])) / abs(float(o[2])) if o else -1
    da = abs(float(r[4]) - float(o[3])) / abs(float(o[3])) if o else -1
    same += dv < 1e-3 and da < 1e-3
    print(f"{r[0]:<52} V {float(r[3]):>14.2f} faces {r[5]:>4} against OpenCASCADE: V {dv:.1e} A {da:.1e}")
print(f"{read} of {len(rows)} read, {same} of those within 1e-3 of OpenCASCADE on the same file")
PY
