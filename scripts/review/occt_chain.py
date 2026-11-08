import json, os, sys
import FreeCAD as App, Part
S = os.environ["KR"]; D = os.path.join(S, "chain")
def plc(p):
    return App.Placement(App.Matrix(p["x"][0], p["y"][0], p["z"][0], p["o"][0], p["x"][1], p["y"][1], p["z"][1], p["o"][1], p["x"][2], p["y"][2], p["z"][2], p["o"][2], 0, 0, 0, 1))
def make(p):
    a = p["p"]
    s = Part.makeBox(a[0], a[1], a[2]) if p["t"] == "box" else Part.makeCylinder(a[0], a[1])
    s.Placement = plc(p)
    return s
res = open(os.path.join(S, "occt_chain.jsonl"), "w")
for line in open(os.path.join(D, "chain.jsonl")):
    r = json.loads(line)
    out = {k: r.get(k) for k in ("seq", "i", "op", "s", "v", "faces", "ms", "open", "msg", "shells")}
    path = os.path.join(D, f"s{r['seq']}_{r['i']}.step")
    prev_i = r["i"] - 1 if r["s"] == "ok" else r["i"]
    prev_path = os.path.join(D, f"s{r['seq']}_{prev_i}.step")
    try:
        if r["s"] == "ok":
            s = Part.read(path)
            out["step_valid"] = bool(s.isValid()); out["step_v"] = s.Volume; out["step_solids"] = len(s.Solids)
        if r.get("prim") and prev_i >= 0 and os.path.exists(prev_path):
            base = Part.read(prev_path); tool = make(r["prim"])
            o = base.cut(tool) if r["bool"] == 1 else base.fuse(tool)
            out["occt_v"] = o.Volume; out["occt_valid"] = bool(o.isValid())
    except Exception as e:
        out["occt_err"] = str(e)[:90]
    res.write(json.dumps(out) + "\n"); res.flush()
res.close()
