# Writes STEP files of shapes OpenCASCADE makes, each with what it says their volume and area are. Runs under FreeCADCmd.
import os, Part, FreeCAD
from FreeCAD import Vector as V

out = os.environ["KR"]
report = open(out + "/occt.csv", "w")


def save(name, shape):
    path = out + "/in/" + name + ".step"
    shape.exportStep(path)
    back = Part.read(path)
    report.write("%s,%.9f,%.9f,%s\n" % (name, back.Volume, back.Area, back.isValid()))
    report.flush()


box = Part.makeBox(30, 20, 10)
cyl = Part.makeCylinder(8, 25)
cone = Part.makeCone(10, 3, 20)
tip = Part.makeCone(10, 0, 20)
sph = Part.makeSphere(12)
tor = Part.makeTorus(20, 5)
save("box", box)
save("cylinder", cyl)
save("cone", cone)
save("cone_tip", tip)
save("sphere", sph)
save("torus", tor)
save("box_hole", box.cut(Part.makeCylinder(4, 40, V(15, 10, -5))))
save("tee", cyl.fuse(Part.makeCylinder(5, 40, V(-20, 0, 12), V(1, 0, 0))))
save("cross_drilled", cyl.cut(Part.makeCylinder(3, 40, V(-20, 1, 12), V(1, 0, 0))))
save("cut_sphere", box.cut(Part.makeSphere(6, V(15, 10, 10))))
save("cone_cut", cone.cut(Part.makeBox(30, 30, 30, V(0, -15, -5))))
save("fillet", box.makeFillet(2, box.Edges))
save("chamfer", box.makeChamfer(2, box.Edges))
save("rounded_cylinder", cyl.makeFillet(1.5, cyl.Edges))
save("half_sphere", sph.common(Part.makeBox(40, 40, 40, V(-20, -20, 0))))
save("torus_cut", tor.cut(Part.makeBox(60, 60, 60, V(0, 0, -30))))
save("ring", Part.makeCylinder(20, 5).cut(Part.makeCylinder(10, 5)))
save("void", Part.makeBox(30, 30, 30).cut(Part.makeSphere(5, V(15, 15, 15))))
ellipse = Part.Wire(Part.Ellipse(V(10, 0, 0), V(0, 5, 0), V(0, 0, 0)).toShape())
save("ellipse_prism", Part.Face(ellipse).extrude(V(0, 0, 7)))
curve = Part.BSplineCurve()
curve.interpolate([V(0, 0, 0), V(10, 5, 0), V(20, -3, 0), V(30, 2, 0)])
outline = Part.Wire([curve.toShape(), Part.makeLine(V(30, 2, 0), V(30, 20, 0)), Part.makeLine(V(30, 20, 0), V(0, 20, 0)), Part.makeLine(V(0, 20, 0), V(0, 0, 0))])
save("spline_prism", Part.Face(outline).extrude(V(0, 0, 5)))
# Between two circles a loft is a true cone, which imports.
save("loft_cone", Part.makeLoft([Part.Wire(Part.makeCircle(5)), Part.Wire(Part.makeCircle(9, V(0, 0, 12)))], True))
# Through three it's free-form, which has to be refused by name.
save("loft_free", Part.makeLoft([Part.Wire(Part.makeCircle(5)), Part.Wire(Part.makeCircle(9, V(0, 0, 12))), Part.Wire(Part.makeCircle(4, V(0, 0, 25)))], True, False))
report.close()
