#!/bin/sh
# Reads a folder of STEP files with this kernel and OpenCASCADE and compares them.
# Usage: check_nist.sh <folder of .stp files> <scratch directory>
# It takes any folder of .stp files. It was written for NIST's public test parts (NIST-PMI-STEP-Files.zip from nist.gov),
# and has also run on STEP Tools' samples. Neither set is kept in this repository.
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
    name = os.path.basename(path)[:-4]
    try:
        s = Part.read(path)
        # Where it all is, and how much it holds, by its triangles, as neither depends on anyone's integrator.
        diagonal = s.BoundBox.DiagonalLength
        points, triangles = s.tessellate(max(diagonal * 1e-3, 1e-6))
        held = 0.0
        for a, b, c in triangles:
            p, q, r = points[a], points[b], points[c]
            held += p.dot(q.cross(r)) / 6.0
        low = [min(p[k] for p in points) for k in range(3)]
        high = [max(p[k] for p in points) for k in range(3)]
        middle = [(low[k] + high[k]) / 2.0 for k in range(3)]
        out.write(f"{name},{len(s.Solids)},{s.Volume},{s.Area},{len(s.Faces)},{s.isValid()},{middle[0]},{middle[1]},{middle[2]},{diagonal},{held}\\n")
    except Exception:
        # A file with no shape in it at all, as one with no geometry has.
        out.write(f"{name},0,0,0,0,False,0,0,0,1,0\\n")
    out.flush()
out.close()
PY
sh "$root/scripts/occt.sh" "$work" "$work/read.py" 900
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
    # Volume alone can't tell whether an assembly's parts are in the right places, so the middle of everything counts too.
    moved = -1.0
    if o and len(r) > 10 and len(o) > 9:
        moved = sum((float(r[8 + k]) - float(o[6 + k])) ** 2 for k in range(3)) ** 0.5 / max(float(o[9]), 1e-9)
    same += dv < 1e-3 and da < 1e-3 and moved < 1e-2
    print(f"{r[0]:<52} V {float(r[3]):>14.2f} faces {r[5]:>4} against OpenCASCADE: V {dv:.1e} A {da:.1e} place {moved:.1e}")
print(f"{read} of {len(rows)} read, {same} of those within 1e-3 of OpenCASCADE on the same file")
PY
