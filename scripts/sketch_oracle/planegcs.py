# Solves each case in $KR/cases.jsonl with FreeCAD's PlaneGCS and writes $KR/planegcs.jsonl. It also
# hands PlaneGCS this solver's answer, to see whether it accepts it without moving it, and repeats
# this solver's drag. Runs under FreeCADCmd. Reads the FreeCAD build and writes only under $KR.
import json, math, os, time, traceback
import FreeCAD as App
from FreeCAD import Vector as V
import Part, Sketcher

KR = os.environ["KR"]
POS = {"start": 1, "end": 2, "center": 3}
AXIS = {"origin": (-1, 1), "x_axis": (-1, 0), "y_axis": (-2, 0)}
doc = App.newDocument("oracle")


def geometry_of(e):
    t = e["type"]
    if t == "point":
        return Part.Point(V(e["at"][0], e["at"][1], 0))
    if t == "line":
        return Part.LineSegment(V(e["start"][0], e["start"][1], 0), V(e["end"][0], e["end"][1], 0))
    c = V(e["center"][0], e["center"][1], 0)
    if t in ("ellipse", "elliptical_arc"):
        tip = V(e["major"][0], e["major"][1], 0)
        d = tip - c
        along = V(d.x, d.y, 0)
        along.normalize()
        side = V(-along.y, along.x, 0)
        ellipse = Part.Ellipse(tip, c + side * e["minor"], c)
        if t == "ellipse":
            return ellipse
        def angle(p):
            q = V(p[0], p[1], 0) - c
            return math.atan2(q.dot(side) / e["minor"], q.dot(along) / d.Length)
        a0, a1 = angle(e["start"]), angle(e["end"])
        while a1 <= a0:
            a1 += 2 * math.pi
        return Part.ArcOfEllipse(ellipse, a0, a1)
    circle = Part.Circle(c, V(0, 0, 1), e["radius"])
    if t == "circle":
        return circle
    a0 = math.atan2(e["start"][1] - c.y, e["start"][0] - c.x)
    a1 = math.atan2(e["end"][1] - c.y, e["end"][0] - c.x)
    while a1 <= a0:
        a1 += 2 * math.pi
    return Part.ArcOfCircle(circle, a0, a1)


def build_case(sketch):
    """Returns the geometry, then the constraint arguments with the id each came from."""
    index = {e["id"]: i for i, e in enumerate(sketch["entities"])}
    kinds = {e["id"]: e["type"] for e in sketch["entities"]}
    # Each ellipse's axes are extra construction lines, placed after everything else so that the
    # indices above stay as they are. The major axis's first point is the end this solver calls `major`.
    axes = {}
    next_geometry = len(sketch["entities"])
    for e in sketch["entities"]:
        if e["type"] in ("ellipse", "elliptical_arc"):
            axes[e["id"]] = (next_geometry, next_geometry + 1)
            next_geometry += 2

    def ref(text):
        text = str(text)
        base, _, part = text.partition(".")
        if base in AXIS:
            g, p = AXIS[base]
            return g, (POS[part] if part else p)
        i = int(base)
        if part == "major":
            return axes[i][0], 1
        if not part:
            return index[i], 1 if kinds[i] == "point" else 0
        return index[i], POS[part]

    def is_point(text):
        g, p = ref(text)
        return p != 0

    def entity(text):
        return ref(text)[0]

    # A reference dimension takes no part in solving.
    cs = [c for c in sketch["constraints"] if not c.get("reference")]
    # A tangency with a shared end replaces that end's coincidence, as FreeCAD's own tools do.
    used = set()
    tangent_at = {}
    for c in cs:
        if c["type"] != "tangent":
            continue
        for k, other in enumerate(cs):
            if other["type"] != "coincident" or k in used:
                continue
            a, b = str(other["a"]), str(other["b"])
            ab, bb = a.partition(".")[0], b.partition(".")[0]
            if {ab, bb} == {str(c["a"]), str(c["b"])} and "." in a and "." in b:
                first, second = (a, b) if ab == str(c["a"]) else (b, a)
                used.add(k)
                tangent_at[c["id"]] = (other["id"], first, second)
                break
            # An end of one placed on the other: FreeCAD's tangency at a point takes both at once. With
            # them as separate constraints, its Jacobian loses rank at the solution, and it says redundant.
            if {ab, bb} == {str(c["a"]), str(c["b"])} and ("." in a) != ("." in b):
                point, whole = (a, b) if "." in a else (b, a)
                used.add(k)
                tangent_at[c["id"]] = (other["id"], point, whole)
                break
    out = []
    for k, c in enumerate(cs):
        t, cid = c["type"], c["id"]
        if t == "coincident":
            if k in used:
                continue
            if is_point(c["a"]) and is_point(c["b"]):
                out.append((("Coincident",) + ref(c["a"]) + ref(c["b"]), cid))
            else:
                p, e = (c["a"], c["b"]) if is_point(c["a"]) else (c["b"], c["a"])
                out.append((("PointOnObject",) + ref(p) + (entity(e),), cid))
        elif t in ("horizontal", "vertical"):
            name = t.capitalize()
            if "b" in c:
                out.append(((name,) + ref(c["a"]) + ref(c["b"]), cid))
            else:
                out.append(((name, entity(c["a"])), cid))
        elif t == "parallel":
            out.append((("Parallel", entity(c["a"]), entity(c["b"])), cid))
        elif t == "perpendicular":
            out.append((("Perpendicular", entity(c["a"]), entity(c["b"])), cid))
        elif t == "tangent":
            if cid in tangent_at:
                _, first, second = tangent_at[cid]
                if "." in second:
                    out.append((("Tangent",) + ref(first) + ref(second), cid))
                else:
                    out.append((("Tangent",) + ref(first) + (entity(second),), cid))
            else:
                out.append((("Tangent", entity(c["a"]), entity(c["b"])), cid))
        elif t == "equal":
            out.append((("Equal", entity(c["a"]), entity(c["b"])), cid))
        elif t == "midpoint":
            line = entity(c["line"])
            out.append((("Symmetric", line, 1, line, 2) + ref(c["point"]), cid))
        elif t == "concentric":
            out.append((("Coincident", entity(c["a"]), 3, entity(c["b"]), 3), cid))
        elif t == "symmetric":
            about = ref(c["about"])
            if about[1] == 0:
                out.append((("Symmetric",) + ref(c["a"]) + ref(c["b"]) + (about[0],), cid))
            else:
                out.append((("Symmetric",) + ref(c["a"]) + ref(c["b"]) + about, cid))
        elif t == "normal":
            # FreeCAD has no edge-to-edge perpendicular for two curves that share no point, so that
            # one constraint is left out here and counted, and the case is compared without it.
            if kinds[int(str(c["a"]))] != "line" and kinds[int(str(c["b"]))] != "line":
                continue
            out.append((("Perpendicular", entity(c["a"]), entity(c["b"])), cid))
        elif t == "fix":
            g, p = ref(c["what"])
            if p == 0:
                out.append((("Block", g), cid))
            else:
                out.append((("DistanceX", -1, 1, g, p, c["at"][0]), cid))
                out.append((("DistanceY", -1, 1, g, p, c["at"][1]), cid))
        elif t == "distance":
            if "b" not in c:
                out.append((("Distance", entity(c["a"]), c["value"]), cid))
            elif is_point(c["a"]) and is_point(c["b"]):
                out.append((("Distance",) + ref(c["a"]) + ref(c["b"]) + (c["value"],), cid))
            elif is_point(c["a"]) or is_point(c["b"]):
                p, e = (c["a"], c["b"]) if is_point(c["a"]) else (c["b"], c["a"])
                out.append((("Distance",) + ref(p) + (entity(e), c["value"]), cid))
            elif kinds[int(str(c["a"]))] == "line" and kinds[int(str(c["b"]))] == "line":
                # The lines are parallel in these sketches, so any point of one will do. FreeCAD has no
                # form of this without picking a point.
                out.append((("Distance", entity(c["a"]), 1, entity(c["b"]), c["value"]), cid))
            else:
                # Between a line and a circle, or two circles. FreeCAD needs the circle first.
                a, b = (c["b"], c["a"]) if kinds[int(str(c["a"]))] == "line" else (c["a"], c["b"])
                out.append((("Distance", entity(a), entity(b), c["value"]), cid))
        elif t == "horizontal_distance":
            out.append((("DistanceX",) + ref(c["a"]) + ref(c["b"]) + (c["value"],), cid))
        elif t == "vertical_distance":
            out.append((("DistanceY",) + ref(c["a"]) + ref(c["b"]) + (c["value"],), cid))
        elif t == "radius":
            out.append((("Radius", entity(c["of"]), c["value"]), cid))
        elif t == "diameter":
            out.append((("Diameter", entity(c["of"]), c["value"]), cid))
        elif t == "major_radius":
            out.append((("Distance", axes[int(str(c["of"]))][0], 2 * c["value"]), cid))
        elif t == "minor_radius":
            out.append((("Distance", axes[int(str(c["of"]))][1], 2 * c["value"]), cid))
        elif t == "angle":
            rad = math.radians(c["value"])
            if "b" in c:
                out.append((("Angle", entity(c["a"]), entity(c["b"]), rad), cid))
            else:
                out.append((("Angle", entity(c["a"]), rad), cid))
        else:
            raise ValueError("no mapping for " + t)
    return out


def unmapped_count(sketch):
    kinds = {e["id"]: e["type"] for e in sketch["entities"]}
    return sum(
        1
        for c in sketch["constraints"]
        if c["type"] == "normal" and kinds[int(str(c["a"]))] != "line" and kinds[int(str(c["b"]))] != "line"
    )


def make_object(sketch):
    sk = doc.addObject("Sketcher::SketchObject", "S")
    for e in sketch["entities"]:
        sk.addGeometry(geometry_of(e), bool(e.get("construction", False)))
    # FreeCAD reaches an ellipse's axes through construction lines tied to it, which add no freedom.
    index = {e["id"]: i for i, e in enumerate(sketch["entities"])}
    alignment = []
    for e in sketch["entities"]:
        if e["type"] in ("ellipse", "elliptical_arc"):
            c, tip = V(e["center"][0], e["center"][1], 0), V(e["major"][0], e["major"][1], 0)
            d = tip - c
            side = V(-d.y, d.x, 0)
            side.normalize()
            side = side * e["minor"]
            major = sk.addGeometry(Part.LineSegment(tip, c - d), True)
            minor = sk.addGeometry(Part.LineSegment(c + side, c - side), True)
            alignment.append(Sketcher.Constraint("InternalAlignment:EllipseMajorDiameter", major, index[e["id"]]))
            alignment.append(Sketcher.Constraint("InternalAlignment:EllipseMinorDiameter", minor, index[e["id"]]))
    if alignment:
        sk.addConstraint(alignment)
    mapped = build_case(sketch)
    if mapped:
        sk.addConstraint([Sketcher.Constraint(*args) for args, _ in mapped])
    return sk, [cid for _, cid in mapped]


def flat(sk, sketch):
    out = []
    for e, g in zip(sketch["entities"], sk.Geometry):
        t = e["type"]
        if t == "point":
            out.append([g.X, g.Y])
        elif t == "line":
            out.append([g.StartPoint.x, g.StartPoint.y, g.EndPoint.x, g.EndPoint.y])
        elif t == "circle":
            out.append([g.Center.x, g.Center.y, g.Radius])
        elif t in ("ellipse", "elliptical_arc"):
            tip = g.Center + g.XAxis * g.MajorRadius
            shape = [g.Center.x, g.Center.y, tip.x, tip.y, g.MinorRadius]
            if t == "elliptical_arc":
                shape += [g.StartPoint.x, g.StartPoint.y, g.EndPoint.x, g.EndPoint.y]
            out.append(shape)
        else:
            out.append([g.Center.x, g.Center.y, g.Radius, g.StartPoint.x, g.StartPoint.y, g.EndPoint.x, g.EndPoint.y])
    return out


def names(sk, ids, attribute):
    # These lists count constraints from 1.
    return sorted({ids[i - 1] for i in getattr(sk, attribute) if 0 < i <= len(ids)})


def largest_change(a, b):
    worst = 0.0
    for x, y in zip(a, b):
        for p, q in zip(x, y):
            worst = max(worst, abs(p - q))
    return worst


def solve_case(case):
    out = {}
    start = case["start"]
    out["unmapped"] = unmapped_count(start)
    sk, ids = make_object(start)
    t = time.perf_counter()
    status = sk.solve()
    out["micros"] = (time.perf_counter() - t) * 1e6
    out["status"] = status
    out["dof"] = sk.DoF
    out["conflicting"] = names(sk, ids, "ConflictingConstraints")
    out["redundant"] = names(sk, ids, "RedundantConstraints")
    out["partial"] = names(sk, ids, "PartiallyRedundantConstraints")
    out["malformed"] = names(sk, ids, "MalformedConstraints")
    out["geometry"] = flat(sk, start)
    doc.removeObject(sk.Name)
    mine = case["mine"]
    if mine.get("ok"):
        solved = mine["solved"]
        sk, ids = make_object(solved)
        before = flat(sk, solved)
        status = sk.solve()
        accept = {"status": status, "dof": sk.DoF}
        accept["moved"] = largest_change(before, flat(sk, solved))
        accept["redundant"] = names(sk, ids, "RedundantConstraints")
        accept["conflicting"] = names(sk, ids, "ConflictingConstraints")
        accept["partial"] = names(sk, ids, "PartiallyRedundantConstraints")
        out["accept"] = accept
        drag = case.get("drag")
        if drag and status == 0:
            base, _, part = drag["grab"].partition(".")
            g = {e["id"]: i for i, e in enumerate(solved["entities"])}[int(base)]
            p = POS.get(part, 1) if part else 1
            if part == "major":
                raise_no_drag = True
                out["drag"] = {"error": "an ellipse's axis end can't be grabbed directly here"}
                doc.removeObject(sk.Name)
                return out
            try:
                t = time.perf_counter()
                sk.moveGeometry(g, p, V(drag["to"][0], drag["to"][1], 0), False)
                micros = (time.perf_counter() - t) * 1e6
                point = sk.getPoint(g, p)
                out["drag"] = {"geometry": flat(sk, solved), "at": [point.x, point.y], "micros": micros}
            except Exception as e:
                out["drag"] = {"error": str(e)}
        doc.removeObject(sk.Name)
    return out


with open(KR + "/cases.jsonl") as source, open(KR + "/planegcs.jsonl", "w") as target:
    for line in source:
        case = json.loads(line)
        try:
            answer = solve_case(case)
        except Exception as e:
            answer = {"status": -99, "error": traceback.format_exc()}
            for o in list(doc.Objects):
                doc.removeObject(o.Name)
        target.write(json.dumps(answer) + "\n")
        target.flush()
open(KR + "/planegcs.done", "w").write("done")
