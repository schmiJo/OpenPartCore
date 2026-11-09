# Runs under OpenCASCADE, by way of occt.sh: reads each part fillet_cases wrote, makes the same fillets in the same order,
# and notes the volume of each, or why it couldn't. Results go in occt_fillets.json beside the parts.
import json, os, time
import Part, FreeCAD
work = os.environ["KR"]
out = []
for case in json.load(open(os.path.join(work, "cases.json"))):
    entry = {"name": case["name"], "mine": case["mine"]}
    # What this kernel made, as OpenCASCADE reads it, even where it can't make it itself.
    if case["result"]:
        try:
            back = Part.Shape()
            back.read(os.path.join(work, case["result"]))
            entry["back_volume"] = back.Volume
            entry["back_valid"] = back.isValid()
        except Exception as why:
            entry["back_error"] = str(why)[:160]
    try:
        shape = Part.Shape()
        shape.read(os.path.join(work, case["step"]))
        solid = shape.Solids[0]
        for s in case["steps"]:
            edges = []
            for p in s["points"]:
                v = Part.Vertex(FreeCAD.Vector(*p))
                hits = [e for e in solid.Edges if e.distToShape(v)[0] < 1e-6]
                if not hits:
                    raise RuntimeError("no edge at %s" % p)
                if not any(h.isSame(e) for h in hits[:1] for e in edges):
                    edges.append(hits[0])
            solid = solid.makeFillet(s["radius"], edges)
        entry["volume"] = solid.Volume
        entry["valid"] = solid.isValid()
    except Exception as why:
        entry["error"] = str(why)[:160]
    out.append(entry)
with open(os.path.join(work, "occt_fillets.json"), "w") as f:
    json.dump(out, f)
