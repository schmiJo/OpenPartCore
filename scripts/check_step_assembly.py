# Runs under OpenCASCADE, by way of occt.sh: reads the STEP assembly written by the write_step_assembly example, and
# compares each solid's volume and center of mass with what a formula says, and the structure with what was written.
import os, json, math
import Part, FreeCAD
work = os.environ["KR"]
path = os.path.join(work, "assembly.step")
out = {}
shape = Part.Shape()
shape.read(path)
out["valid"] = shape.isValid()
solids = []
for s in shape.Solids:
    c = s.CenterOfMass
    solids.append([s.Volume, c.x, c.y, c.z])
out["solids"] = solids
try:
    import Import
    doc = FreeCAD.newDocument("structure")
    Import.insert(path, doc.Name)
    out["objects"] = [[o.TypeId, o.Label] for o in doc.Objects]
except Exception as why:
    out["objects_error"] = str(why)
with open(os.path.join(work, "assembly.json"), "w") as f:
    json.dump(out, f)
