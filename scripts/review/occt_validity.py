# Reads every STEP file of a `review chain` run in OCCT and reports validity, comparing
# volumes with the kernel's log. Run under FreeCADCmd with KR set to the parent directory.
import json, os, Part
S = os.environ["KR"]; D = os.path.join(S, os.environ.get("KR_CHAIN", "chain"))
logs = {}
for line in open(os.path.join(D, "chain.jsonl")):
    r = json.loads(line)
    if r["s"] == "ok":
        logs[(r["seq"], r["i"])] = r
bad = []; n = 0; worst = 0.0
for (seq, i), r in sorted(logs.items()):
    s = Part.read(f"{D}/s{seq}_{i}.step")
    n += 1
    ok = bool(s.isValid())
    dv = abs(s.Volume - r["v"]) / max(abs(r["v"]), 1e-9)
    worst = max(worst, dv)
    if not ok or dv > 1e-5:
        bad.append({"seq": seq, "i": i, "op": r["op"], "valid": ok, "dv": dv, "faces": len(s.Faces), "kernel_faces": r["faces"]})
json.dump({"checked": n, "bad": bad, "worst_dv": worst}, open(os.path.join(S, "occt_validity.json"), "w"), indent=0)
