#!/usr/bin/env python3
# The table of real parts: what the tools did with each, what a formula said, and what OpenCASCADE found in its STEP file.
# Usage: real_parts_table.py <folder> [--expected <file to write>]
# Writes <folder>/table.md and prints it. A part is whole only if every step went through and every check agrees, including
# OpenCASCADE's if it ran. --expected writes the regression test's expectations, with a volume from OpenCASCADE if it read
# the file, as this kernel's own answer is no check of itself.
import glob, json, os, sys

folder = sys.argv[1]
results = {os.path.basename(p).replace(".result.json", ""): json.load(open(p)) for p in sorted(glob.glob(os.path.join(folder, "*.result.json")))}
occt_path = os.path.join(folder, "occt.json")
occt = json.load(open(occt_path)) if os.path.exists(occt_path) else {}

# How far OpenCASCADE's volume can be from this kernel's for a file with no free-form surfaces. See docs/KNOWN_LIMITS.md for
# where its integrator is the one that's off, by this much and more, on helical and lofted solids.
AGREE = 1e-4

rows = []
expected = {}
counts = {"whole": 0, "refused": 0, "wrong": 0, "steps": 0, "failed_steps": 0, "followed": 0}
for stem, r in results.items():
    trouble = []
    for s in r["steps"]:
        counts["steps"] += 1
        if s["state"] == "ok":
            continue
        if s["state"] == "followed":
            counts["followed"] += 1
        else:
            counts["failed_steps"] += 1
        trouble.append(s)
    occt_notes = []
    occt_volume = None
    for w in r["written"]:
        got = occt.get(w["file"])
        if got is None:
            continue
        if "error" in got:
            occt_notes.append(f"OCCT can't read {w['file']}: {got['error']}")
            continue
        if not w["file"].count("__change"):
            occt_volume = got["volume"]
        if not got["valid"]:
            occt_notes.append(f"OCCT says {w['file']} isn't valid")
        if abs(got["volume"] - w["volume"]) > AGREE * max(abs(w["volume"]), 1e-9) and not r.get("occt_note"):
            occt_notes.append(f"{w['file']}: volume {w['volume']:.4f} here and {got['volume']:.4f} in OCCT")
        if got["solids"] != w["solids"]:
            occt_notes.append(f"{w['file']}: {w['solids']} solids here and {got['solids']} in OCCT")
    state = "whole"
    if any(s["state"] == "wrong" for s in trouble) or occt_notes:
        state = "wrong"
    elif trouble:
        state = "refused"
    counts[state] += 1
    first = trouble[0] if trouble else None
    why = f"{first['label']}: {first['why'][:150]}" if first else "; ".join(occt_notes)[:190]
    if first and occt_notes:
        why += " / " + occt_notes[0][:80]
    rows.append((stem, r["kind"], r["features"], state, r["volume"], occt_volume, r["seconds"], why))
    expected[stem] = {
        "state": state,
        "volume": r["volume"] if r.get("occt_note") or occt_volume is None else occt_volume,
        "volume_from": "formula" if r.get("occt_note") else "occt" if occt_volume is not None else "kernel",
        "trouble": [s["label"] for s in trouble if s["state"] != "followed"],
    }

lines = ["| Part | Kind | Features | Result | Volume | OCCT | Seconds | First trouble |", "|---|---|---|---|---|---|---|---|"]
for stem, kind, n, state, v, o, sec, why in rows:
    vs = "" if v is None else f"{v:.2f}"
    os_ = "" if o is None else f"{o:.2f}"
    lines.append(f"| {stem} | {kind} | {n} | {state} | {vs} | {os_} | {sec:.1f} | {why.replace('|', '/')} |")
lines.append("")
n = len(rows)
lines.append(f"{counts['whole']} of {n} whole, {counts['refused']} with a step refused, {counts['wrong']} with something wrong. Of {counts['steps']} steps and checks, {counts['failed_steps']} failed on their own, and {counts['followed']} followed from those.")
scans = sorted(glob.glob(os.path.join(folder, "*.scan.json")))
if scans:
    rows_ok = rows_refused = rows_wrong = 0
    failures = []
    for path in scans:
        stem = os.path.basename(path).replace(".scan.json", "")
        for row in json.load(open(path)):
            if row["state"] == "ok":
                rows_ok += 1
            elif row["state"] == "refused":
                rows_refused += 1
                failures.append(f"- {stem} refused: {row['dims'][:120]}: {row['why'][:200]}")
            else:
                rows_wrong += 1
                failures.append(f"- {stem} WRONG: {row['dims'][:120]}: {row['why'][:200]}")
    lines.append("")
    lines.append(f"Dimensions moved by up to 5% at random, all at once: {rows_ok} rebuilds fine, {rows_refused} refused, {rows_wrong} wrong.")
    lines.extend(failures)
text = "\n".join(lines)
open(os.path.join(folder, "table.md"), "w").write(text + "\n")
print(text)
if "--expected" in sys.argv:
    target = sys.argv[sys.argv.index("--expected") + 1]
    with open(target, "w") as f:
        f.write("{\n")
        items = list(expected.items())
        for k, (stem, e) in enumerate(items):
            f.write(f"  {json.dumps(stem)}: {json.dumps(e)}{',' if k + 1 < len(items) else ''}\n")
        f.write("}\n")
