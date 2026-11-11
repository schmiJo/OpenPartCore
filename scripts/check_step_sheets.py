# Runs under OpenCASCADE, by way of occt.sh. Reads the sheets written here and says what it finds, and writes some of its own
# for this kernel to read back.
import glob, json, math, os
import Part, FreeCAD
work = os.environ["KR"]
out = {"read": {}, "wrote": {}}
for path in sorted(glob.glob(os.path.join(work, "*.step"))):
    name = os.path.basename(path)[:-5]
    if name.startswith("occt_"):
        continue
    shape = Part.Shape()
    shape.read(path)
    free = 0
    for s in shape.Shells:
        free += len(s.getFreeEdges().Edges)
    out["read"][name] = {"area": shape.Area, "valid": shape.isValid(), "shells": len(shape.Shells), "solids": len(shape.Solids), "faces": len(shape.Faces), "free": free}

v = FreeCAD.Vector
def wrote(name, shape):
    shape.exportStep(os.path.join(work, "occt_%s.step" % name))
    out["wrote"][name] = {"area": shape.Area, "kind": shape.ShapeType, "faces": len(shape.Faces)}
wrote("plane", Part.makePlane(10, 20))
wrote("cylinder_side", Part.makeCylinder(5, 8).Faces[0])
wrote("sphere", Part.makeSphere(5).Faces[0])
box = Part.makeBox(10, 20, 30)
wrote("box_five_sides", Part.Shell(box.Faces[:5]))
wrote("two_apart", Part.makeCompound([Part.makePlane(10, 20), Part.makePlane(5, 5, v(50, 0, 0))]))
curve = Part.BSplineCurve()
curve.interpolate([v(0, 0, 0), v(5, 3, 0), v(10, 0, 0), v(15, 3, 0)])
wrote("extruded_spline", curve.toShape().extrude(v(0, 0, 4)))
points = [[v(i * 5, j * 5, math.sin(i) * math.cos(j)) for j in range(5)] for i in range(5)]
surface = Part.BSplineSurface()
surface.interpolate(points)
wrote("free_form", surface.toShape())
with open(os.path.join(work, "sheets.json"), "w") as f:
    json.dump(out, f)
