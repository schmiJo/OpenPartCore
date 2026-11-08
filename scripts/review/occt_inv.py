import json, os
import Part
S = os.environ["KR"]; D = S + "/chain"; out = {}
for seq, i in ((119, 7), (143, 4)):
    prev = Part.read(f"{D}/s{seq}_{i-1}.step"); s = Part.read(f"{D}/s{seq}_{i}.step")
    r = {"prev_valid": bool(prev.isValid()), "valid": bool(s.isValid()), "solids": len(s.Solids), "closed": [bool(x.isClosed()) for x in s.Shells], "faces": len(s.Faces), "prev_faces": len(prev.Faces)}
    try:
        s.check(True); r["check"] = "passes"
    except Exception as e:
        r["check"] = str(e).strip().replace("\n", " | ")[:400]
    badf = []
    for k, f in enumerate(s.Faces):
        if not f.isValid():
            n = f.Surface.__class__.__name__
            detail = {"type": n, "area": round(f.Area, 3), "wires": len(f.Wires), "wire_edges": [len(w.Edges) for w in f.Wires], "tol": max(e.Tolerance for e in f.Edges), "bbox": [round(x, 2) for x in (f.BoundBox.XMin, f.BoundBox.XMax, f.BoundBox.YMin, f.BoundBox.YMax, f.BoundBox.ZMin, f.BoundBox.ZMax)]}
            try: f.check(False)
            except Exception as e: detail["why"] = str(e).strip()[:200]
            badf.append(detail)
    r["bad_faces"] = badf
    r["bad_edges"] = sum(1 for e in s.Edges if not e.isValid())
    f2 = s.copy()
    try:
        f2.fix(1e-3, 1e-3, 1e-3); r["after_fix"] = bool(f2.isValid())
    except Exception as e:
        r["after_fix"] = str(e)[:80]
    out[f"{seq}_{i}"] = r
json.dump(out, open(S + "/occt_inv.json", "w"), indent=1)
