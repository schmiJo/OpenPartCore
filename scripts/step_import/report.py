#!/usr/bin/env python3
"""Compares this kernel's reading of each file with OpenCASCADE's numbers, and its result back the other way."""
import csv, sys

work = sys.argv[1]
occt = {r[0]: r for r in csv.reader(open(work + "/occt.csv"))}
back = {r[0]: r for r in csv.reader(open(work + "/back.csv"))}
good = total = refused = 0
print(f"{'part':<18} {'solids':>6} {'faces':>5} {'mesh':>5} {'V relative':>11} {'A relative':>11}   after a Boolean, in OpenCASCADE")
for r in csv.reader(open(work + "/mine.csv")):
    name = r[0]
    total += 1
    if r[1] != "ok":
        refused += 1
        print(f"{name:<18} refused: {','.join(r[2:])[:110]}")
        continue
    v, a = float(r[3]), float(r[4])
    ov, oa = float(occt[name][1]), float(occt[name][2])
    (dv, da) = (abs(v - ov) / abs(ov), abs(a - oa) / abs(oa))
    after = "no result"
    ok = dv < 1e-5 and r[6] == "true"
    if r[7].startswith("refused"):
        after = r[7][:60]
    elif name in back:
        cut, valid, vb = float(r[7]), back[name][1], float(back[name][2])
        after = f"valid={valid} V relative {abs(cut - vb) / abs(cut):.1e}"
        ok = ok and valid == "True" and abs(cut - vb) / abs(cut) < 1e-3
    good += ok
    print(f"{name:<18} {r[2]:>6} {r[5]:>5} {r[6]:>5} {dv:>11.1e} {da:>11.1e}   {after}")
print(f"{good} of {total - refused} read parts matched and survived a Boolean; {refused} refused.")
