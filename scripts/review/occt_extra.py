import json, math, os
import FreeCAD as App, Part
V = App.Vector
S = os.environ["KR"]; out = {}
a = Part.makeBox(10, 10, 10); b = Part.makeBox(10, 10, 5)
for ang in (1e-4, 1e-7):
    b.Placement = App.Placement(V(0, 0, 10), App.Rotation(V(1, 0, 0), math.degrees(ang)))
    u = a.fuse(b); out[f"tilted {ang}"] = [u.Volume, len(u.Faces), bool(u.isValid())]
# seed 7000138
cone = Part.makeCone(2, 0, 4, V(0, 1, 2), V(0, 0, 1)); cyl = Part.makeCylinder(1, 4, V(0, 2, 2), V(0, 0, 1))
out["138"] = {"va": cone.Volume, "vb": cyl.Volume, "union": cone.fuse(cyl).Volume, "cut": cone.cut(cyl).Volume, "common": cone.common(cyl).Volume}
# does OCCT's fillet follow tangent edges? (r=2 on the top front edge, then r=4 on the top left edge)
bx = Part.makeBox(40, 30, 20)
f = bx.makeFillet(2, [e for e in bx.Edges if abs(e.CenterOfMass.z - 20) < 1e-6 and abs(e.CenterOfMass.y) < 1e-6])
sel = [e for e in f.Edges if abs(e.CenterOfMass.z - 20) < 1e-6 and abs(e.CenterOfMass.x) < 1e-6]
g = f.makeFillet(4, sel)
out["propagation"] = {"selected edges": len(sel), "volume": g.Volume, "vertical edge (x=0,y=0) rounded": not g.isInside(V(0.3, 0.3, 5), 1e-6, False),
  "before": f.isInside(V(0.3, 0.3, 5), 1e-6, False)}
# what the kernel's meaning gives: subtract only the swept corner along that edge
corner = Part.makeBox(4, 34, 4, V(0, -2, 16)).cut(Part.makeCylinder(4, 34, V(4, -2, 16), V(0, 1, 0)))
out["no-propagation volume"] = f.cut(corner).Volume
json.dump(out, open(os.path.join(S, "occt_extra.json"), "w"), indent=0)
