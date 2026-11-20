//! Planes, and the references that place things.
//!
//! A plane feature makes a plane, not a part. It's worked out again at every rebuild from what it names, so it follows
//! them. Sketches can sit on one, and features that take a plane, a point or a line name them the same way everywhere:
//! see `PlaneRef`, `PointRef` and `LineRef`.

use super::*;
use coreshape_sketch::{EntityId, Geometry, Part, Ref};
use std::cell::OnceCell;

/// A plane, as a feature names one.
#[derive(Clone, Debug, PartialEq)]
pub enum PlaneRef {
    /// Onshape's Top: the XY plane, facing +Z, with x along X.
    Top,
    /// Onshape's Front: the XZ plane, facing -Y, with x along X and y along Z.
    Front,
    /// Onshape's Right: the YZ plane, facing +X, with x along Y and y along Z.
    Right,
    /// A plane feature.
    Feature(FeatureId),
    /// A flat face's plane, facing out of the part. Its origin is where the world's origin is seen along its normal, and
    /// its x is as `default_x` gives it, so that neither moves when the face grows or gets a hole.
    Face(FaceKey),
    /// Through a point, square to a direction, as numbers. Its x is as `default_x` gives it.
    Given { point: Point3, normal: Point3 },
}

/// A point, as a feature names one.
#[derive(Clone, Debug, PartialEq)]
pub enum PointRef {
    /// The world's origin.
    Origin,
    /// A place, as numbers.
    At(Point3),
    /// A part's corner: the vertex these faces share, the one nearest `near` if they share several. vertices_list and
    /// edges_list give these.
    Vertex { faces: Vec<FaceKey>, near: Option<Point3> },
    /// A point of a constrained sketch: a point entity, or the start, end or center of a curve, as constraints name
    /// them: `7`, `7.start`, `7.end`, `7.center`, `7.major` or `origin`.
    Sketch { sketch: FeatureId, point: Ref },
}

/// A line, as a feature names one: a point on it and a direction along it.
#[derive(Clone, Debug, PartialEq)]
pub enum LineRef {
    /// As numbers.
    Given { point: Point3, direction: Point3 },
    /// A straight edge, which the selection has to pick alone. It runs from its start to its end, as edges_list gives them.
    Edge(EdgeSelect),
    /// A line of a constrained sketch, from its start to its end.
    Sketch { sketch: FeatureId, line: EntityId },
    /// The axis of a cylinder, cone, torus or turned face, pointing the way faces_list gives it.
    AxisOfFace(FaceKey),
    /// The axis of a circle, arc or ellipse that an edge selection picks alone.
    AxisOfEdge(EdgeSelect),
}

/// How a plane feature places its plane. Each type is Onshape's.
#[derive(Clone, Debug, PartialEq)]
pub enum PlaneDef {
    /// Parallel to a plane, `distance` along its normal, with its x.
    Offset { plane: PlaneRef, distance: Expr },
    /// Parallel to a plane, through a point, with its x. The point is its origin.
    PlanePoint { plane: PlaneRef, point: PointRef },
    /// Through a line, turned about it by `degrees`, right-handed about the line's direction. At 0 it's the plane through
    /// the line that comes nearest to being parallel to `reference`. Without one, that's whichever of Top, Front and Right
    /// the line is most nearly parallel to, in that order when it's equally so. Its origin is the line's point, and its x
    /// runs along the line.
    LineAngle { line: LineRef, degrees: Expr, reference: Option<PlaneRef> },
    /// Through a point, square to a line, facing along it. The point is its origin.
    PointNormal { point: PointRef, line: LineRef },
    /// Through three points. Its origin is the first, its x points toward the second, and it faces the side from which
    /// the three go round counter-clockwise.
    ThreePoint { points: [PointRef; 3] },
    /// Halfway between two planes, facing as the first does, or between two that meet, the plane through where they meet
    /// that bisects the angle between them: the one facing between their normals, once the second is turned to face the
    /// same side as the first. Its x is the first's, made square to its normal.
    MidPlanes { planes: [PlaneRef; 2] },
    /// Square to the line between two points, at its middle, facing from the first toward the second.
    MidPoints { points: [PointRef; 2] },
    /// Square to an edge, facing along it, at the point on it nearest a point or a fraction of the way along it from its
    /// start. That point is its origin.
    CurvePoint { edge: EdgeSelect, at: AlongEdge },
    /// Touching a cylinder, cone or sphere and facing out of the part, at the point on it nearest a point, or on a cylinder
    /// or cone at an angle around its axis. That point is its origin.
    Tangent { face: FaceKey, at: TangentAt },
}

/// Where on an edge a curve-point plane is.
#[derive(Clone, Debug, PartialEq)]
pub enum AlongEdge {
    /// The point on the edge nearest this one.
    Point(PointRef),
    /// A fraction of the edge's length from its start, from 0 to 1.
    Fraction(Expr),
}

/// Where on a round face a tangent plane touches.
#[derive(Clone, Debug, PartialEq)]
pub enum TangentAt {
    /// The point on its surface nearest this one.
    Point(PointRef),
    /// An angle around a cylinder's or cone's axis, right-handed about the axis's direction. 0 is the direction
    /// `default_x` gives a plane facing along the axis, and the height is the middle of the face.
    Degrees(Expr),
}

impl PlaneRef {
    pub fn through(point: Point3, normal: Point3) -> PlaneRef {
        PlaneRef::Given { point, normal }
    }

    /// Where Top, Front and Right are. Every other plane depends on the model.
    pub fn standard_frame(&self) -> Option<Frame> {
        match self {
            PlaneRef::Top => Some(Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X)),
            PlaneRef::Front => Some(Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X)),
            PlaneRef::Right => Some(Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y)),
            _ => None,
        }
    }

    /// What it is, in a message.
    pub fn describe(&self) -> String {
        match self {
            PlaneRef::Top => "the Top plane".into(),
            PlaneRef::Front => "the Front plane".into(),
            PlaneRef::Right => "the Right plane".into(),
            PlaneRef::Feature(id) => format!("plane f{}", id.0),
            PlaneRef::Face(key) => format!("the face {}", key.text()),
            PlaneRef::Given { .. } => "the plane given".into(),
        }
    }

    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            PlaneRef::Feature(id) => out.push(*id),
            PlaneRef::Face(key) => key.references(out),
            _ => {}
        }
    }

    /// The plane features it names.
    pub(crate) fn planes(&self, out: &mut Vec<FeatureId>) {
        if let PlaneRef::Feature(id) = self {
            out.push(*id);
        }
    }

    pub(crate) fn exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        if let PlaneRef::Given { point, normal } = self {
            out.extend(point.iter());
            out.extend(normal.iter());
        }
    }
}

impl PointRef {
    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            PointRef::Vertex { faces, .. } => {
                for key in faces {
                    key.references(out);
                }
            }
            PointRef::Sketch { sketch, .. } => out.push(*sketch),
            PointRef::Origin | PointRef::At(_) => {}
        }
    }

    /// The sketches it takes a point from.
    pub(crate) fn sketches(&self, out: &mut Vec<FeatureId>) {
        if let PointRef::Sketch { sketch, .. } = self {
            out.push(*sketch);
        }
    }

    pub(crate) fn exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        match self {
            PointRef::At(p) | PointRef::Vertex { near: Some(p), .. } => out.extend(p.iter()),
            _ => {}
        }
    }
}

impl LineRef {
    pub fn through(point: Point3, direction: Point3) -> LineRef {
        LineRef::Given { point, direction }
    }

    /// A direction, as a linear pattern takes one: a line through the origin.
    pub fn along(direction: Point3) -> LineRef {
        LineRef::Given { point: point3(0.0, 0.0, 0.0), direction }
    }

    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            LineRef::Edge(edges) | LineRef::AxisOfEdge(edges) => edges.references(out),
            LineRef::Sketch { sketch, .. } => out.push(*sketch),
            LineRef::AxisOfFace(key) => key.references(out),
            LineRef::Given { .. } => {}
        }
    }

    pub(crate) fn sketches(&self, out: &mut Vec<FeatureId>) {
        if let LineRef::Sketch { sketch, .. } = self {
            out.push(*sketch);
        }
    }

    pub(crate) fn exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        match self {
            LineRef::Given { point, direction } => {
                out.extend(point.iter());
                out.extend(direction.iter());
            }
            LineRef::Edge(edges) | LineRef::AxisOfEdge(edges) => edges.collect_exprs(out),
            LineRef::Sketch { .. } | LineRef::AxisOfFace(_) => {}
        }
    }
}

impl PlaneDef {
    /// Its name in files.
    pub fn kind(&self) -> &'static str {
        match self {
            PlaneDef::Offset { .. } => "offset",
            PlaneDef::PlanePoint { .. } => "plane_point",
            PlaneDef::LineAngle { .. } => "line_angle",
            PlaneDef::PointNormal { .. } => "point_normal",
            PlaneDef::ThreePoint { .. } => "three_point",
            PlaneDef::MidPlanes { .. } | PlaneDef::MidPoints { .. } => "mid_plane",
            PlaneDef::CurvePoint { .. } => "curve_point",
            PlaneDef::Tangent { .. } => "tangent",
        }
    }

    fn parts(&self) -> (Vec<&PlaneRef>, Vec<&PointRef>, Vec<&LineRef>) {
        let (mut planes, mut points, mut lines) = (Vec::new(), Vec::new(), Vec::new());
        match self {
            PlaneDef::Offset { plane, .. } => planes.push(plane),
            PlaneDef::PlanePoint { plane, point } => {
                planes.push(plane);
                points.push(point);
            }
            PlaneDef::LineAngle { line, reference, .. } => {
                lines.push(line);
                planes.extend(reference);
            }
            PlaneDef::PointNormal { point, line } => {
                points.push(point);
                lines.push(line);
            }
            PlaneDef::ThreePoint { points: list } => points.extend(list),
            PlaneDef::MidPlanes { planes: list } => planes.extend(list),
            PlaneDef::MidPoints { points: list } => points.extend(list),
            PlaneDef::CurvePoint { at, .. } => {
                if let AlongEdge::Point(p) = at {
                    points.push(p);
                }
            }
            PlaneDef::Tangent { at, .. } => {
                if let TangentAt::Point(p) = at {
                    points.push(p);
                }
            }
        }
        (planes, points, lines)
    }

    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        let (planes, points, lines) = self.parts();
        planes.iter().for_each(|p| p.references(out));
        points.iter().for_each(|p| p.references(out));
        lines.iter().for_each(|l| l.references(out));
        match self {
            PlaneDef::CurvePoint { edge, .. } => edge.references(out),
            PlaneDef::Tangent { face, .. } => face.references(out),
            _ => {}
        }
    }

    pub(crate) fn planes(&self, out: &mut Vec<FeatureId>) {
        self.parts().0.iter().for_each(|p| p.planes(out));
    }

    pub(crate) fn sketches(&self, out: &mut Vec<FeatureId>) {
        let (_, points, lines) = self.parts();
        points.iter().for_each(|p| p.sketches(out));
        lines.iter().for_each(|l| l.sketches(out));
    }

    pub(crate) fn exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        match self {
            PlaneDef::Offset { plane, distance } => {
                plane.exprs(out);
                out.push(distance);
            }
            PlaneDef::PlanePoint { plane, point } => {
                plane.exprs(out);
                point.exprs(out);
            }
            PlaneDef::LineAngle { line, degrees, reference } => {
                line.exprs(out);
                out.push(degrees);
                if let Some(r) = reference {
                    r.exprs(out);
                }
            }
            PlaneDef::PointNormal { point, line } => {
                point.exprs(out);
                line.exprs(out);
            }
            PlaneDef::ThreePoint { points } => points.iter().for_each(|p| p.exprs(out)),
            PlaneDef::MidPlanes { planes } => planes.iter().for_each(|p| p.exprs(out)),
            PlaneDef::MidPoints { points } => points.iter().for_each(|p| p.exprs(out)),
            PlaneDef::CurvePoint { edge, at } => {
                edge.collect_exprs(out);
                match at {
                    AlongEdge::Point(p) => p.exprs(out),
                    AlongEdge::Fraction(f) => out.push(f),
                }
            }
            PlaneDef::Tangent { at, .. } => match at {
                TangentAt::Point(p) => p.exprs(out),
                TangentAt::Degrees(d) => out.push(d),
            },
        }
    }
}

/// Something a feature places things by, which a rebuild works out before running it, so that the feature's key covers
/// where it is.
pub(crate) enum Geo<'a> {
    Place(&'a Placement),
    Plane(&'a PlaneRef),
    Line(&'a LineRef),
    Def(&'a PlaneDef, bool),
}

impl<'a> Geo<'a> {
    pub(crate) fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            Geo::Place(p) => {
                if let Some(key) = &p.on_face {
                    key.references(out);
                }
                if let Some(plane) = &p.on_plane {
                    plane.references(out);
                }
            }
            Geo::Plane(p) => p.references(out),
            Geo::Line(l) => l.references(out),
            Geo::Def(d, _) => d.references(out),
        }
    }

    pub(crate) fn planes(&self, out: &mut Vec<FeatureId>) {
        match self {
            Geo::Place(p) => p.on_plane.iter().for_each(|r| r.planes(out)),
            Geo::Plane(p) => p.planes(out),
            Geo::Def(d, _) => d.planes(out),
            Geo::Line(_) => {}
        }
    }

    pub(crate) fn sketches(&self, out: &mut Vec<FeatureId>) {
        match self {
            Geo::Line(l) => l.sketches(out),
            Geo::Def(d, _) => d.sketches(out),
            Geo::Place(_) | Geo::Plane(_) => {}
        }
    }

    /// The expressions in it, other than a placement's own numbers, which its feature already counts.
    pub(crate) fn exprs(&self, out: &mut Vec<&'a Expr>) {
        match *self {
            Geo::Place(p) => {
                if let Some(plane) = &p.on_plane {
                    plane.exprs(out);
                }
            }
            Geo::Plane(p) => p.exprs(out),
            Geo::Line(l) => l.exprs(out),
            Geo::Def(d, _) => d.exprs(out),
        }
    }
}

/// The x direction a plane gets when nothing else gives it one. On a plane nearer level than upright it runs along X, and
/// on any other it's level, to the right as you face the plane, so Top, Front and Right get Onshape's.
pub fn default_x(normal: Vec3) -> Vec3 {
    let n = normal.normalized();
    let level = v3(n.x, n.y, 0.0).norm();
    if n.z.abs() > level {
        Vec3::X - n * n.x
    } else {
        Vec3::Z.cross(n)
    }
}

/// Where planes, points and lines are at one point in a rebuild: among the bodies made so far, and the sketches and planes
/// built so far.
pub(crate) struct Looker<'a> {
    doc: &'a Document,
    env: &'a Env,
    all: &'a [Held],
    whole: OnceCell<Option<Arc<Body>>>,
}

fn bad(why: String) -> Error {
    Error::InvalidInput(why)
}

fn place_of(p: Vec3) -> String {
    format!("({:.3}, {:.3}, {:.3})", p.x, p.y, p.z)
}

fn curve_kind(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "a straight line",
        Curve::Circle { .. } => "a circle or an arc",
        Curve::Ellipse { .. } => "an ellipse",
        Curve::Spline(_) => "a spline",
        Curve::Sampled(_) => "an intersection curve",
    }
}

fn surface_kind(s: &Surface) -> &'static str {
    match s {
        Surface::Plane(_) => "flat",
        Surface::Cylinder { .. } => "a cylinder",
        Surface::Cone { .. } => "a cone",
        Surface::Sphere { .. } => "a sphere",
        Surface::Torus { .. } => "a torus",
        Surface::Extruded { .. } => "an extruded curve",
        Surface::Revolved { .. } => "a turned curve",
        Surface::Free { .. } => "a free-form surface",
    }
}

impl<'a> Looker<'a> {
    pub(crate) fn new(doc: &'a Document, env: &'a Env, all: &'a [Held]) -> Looker<'a> {
        Looker { doc, env, all, whole: OnceCell::new() }
    }

    fn at(&self, p: &Point3) -> Result<Vec3> {
        Ok(v3(num(&p[0], self.env)?, num(&p[1], self.env)?, num(&p[2], self.env)?))
    }

    fn direction(&self, p: &Point3, what: &str) -> Result<Vec3> {
        let d = self.at(p)?;
        // Written so that a length that overflowed fails too.
        if !(d.norm() >= 1e-12 && d.norm() <= 1e150) {
            return Err(bad(format!("{what} must have a sensible, non-zero length")));
        }
        Ok(d.normalized())
    }

    fn whole(&self, what: &str) -> Result<&Arc<Body>> {
        self.whole.get_or_init(|| whole_of(self.all)).as_ref().ok_or_else(|| bad(format!("{what} needs a part, and there's none before it")))
    }

    /// The faces a key picks, which have to be there.
    fn faces(&self, key: &FaceKey) -> Result<(&Arc<Body>, Vec<usize>)> {
        let gone = || bad(format!("the face {} no longer exists in the model", key.text()));
        let body = self.whole.get_or_init(|| whole_of(self.all)).as_ref().ok_or_else(gone)?;
        Ok((body, crate::keys::find_one(body, key)?))
    }

    /// The one face a key picks, or the first of several that are all on one surface, as a circle's two halves are.
    fn one_surface(&self, key: &FaceKey) -> Result<(&Arc<Body>, usize)> {
        let (body, found) = self.faces(key)?;
        let first = &body.faces[found[0]];
        let size = body.aabb().diagonal().max(1.0);
        for &f in &found[1..] {
            let other = &body.faces[f];
            let same = std::mem::discriminant(&other.surface) == std::mem::discriminant(&first.surface)
                && other.flipped == first.flipped
                && other.loops.iter().flatten().all(|c| first.surface.sdf(body.coedge_point(*c, 0.5)).abs() <= 1e-6 * size);
            if !same {
                return Err(bad(format!("{} covers {} faces that aren't on one surface: name one of them by its key, as faces_list gives it", key.text(), found.len())));
            }
        }
        Ok((body, found[0]))
    }

    /// The one edge a selection picks.
    fn one_edge(&self, query: &EdgeSelect, what: &str) -> Result<(&Arc<Body>, usize)> {
        let body = self.whole(what)?;
        let picked = select_edges(query, body, self.env)?;
        match picked.as_slice() {
            [one] => Ok((body, *one)),
            many => Err(bad(format!("that edge selection picks {} edges, and {what} takes one", many.len()))),
        }
    }

    /// Where a placement puts things: where its numbers say, or following a face or a plane.
    pub(crate) fn placement(&self, p: &Placement) -> Result<Frame> {
        if let Some(on) = &p.on_plane {
            return self.plane(on);
        }
        let frame = p.resolve(self.env)?;
        let Some(key) = &p.on_face else { return Ok(frame) };
        let (whole, found) = self.faces(key)?;
        let Surface::Plane(on) = &whole.faces[found[0]].surface else {
            return Err(bad(format!("a sketch can only follow a flat face, and {} isn't flat", key.text())));
        };
        let normal = if on.z.dot(frame.z) >= 0.0 { on.z } else { -on.z };
        let origin = frame.origin - on.z * (frame.origin - on.origin).dot(on.z);
        // Tipped so far that its x axis now runs along the normal, any direction in the plane will do.
        let x = if normal.cross(frame.x).norm() > 1e-9 { frame.x } else { normal.any_perpendicular() };
        Ok(Frame::new(origin, normal, x))
    }

    pub(crate) fn plane(&self, r: &PlaneRef) -> Result<Frame> {
        match r {
            PlaneRef::Top | PlaneRef::Front | PlaneRef::Right => Ok(r.standard_frame().expect("a standard plane")),
            PlaneRef::Feature(id) => self.doc.plane_output(*id),
            PlaneRef::Given { point, normal } => {
                let n = self.direction(normal, "a plane's normal")?;
                Ok(Frame::new(self.at(point)?, n, default_x(n)))
            }
            PlaneRef::Face(key) => {
                let (body, f) = self.one_surface(key)?;
                let face = &body.faces[f];
                let Surface::Plane(on) = &face.surface else {
                    return Err(bad(format!("the face {} is {}, not flat, so it has no plane", key.text(), surface_kind(&face.surface))));
                };
                let n = if face.flipped { -on.z } else { on.z };
                Ok(Frame::new(n * on.origin.dot(n), n, default_x(n)))
            }
        }
    }

    pub(crate) fn point(&self, r: &PointRef) -> Result<Vec3> {
        match r {
            PointRef::Origin => Ok(Vec3::ZERO),
            PointRef::At(p) => self.at(p),
            PointRef::Vertex { faces, near } => up_to::vertex_at(&VertexRef::Faces { faces: faces.clone(), near: near.clone() }, self.all, self.env),
            PointRef::Sketch { sketch, point } => {
                let (frame, solved) = self.doc.solved_sketch(*sketch)?;
                let q = sketch_point(&solved, *point).map_err(|why| bad(format!("sketch f{}'s point {}: {why}", sketch.0, point.name())))?;
                Ok(frame.point(q[0], q[1], 0.0))
            }
        }
    }

    /// A point on a line, and its direction.
    pub(crate) fn line(&self, r: &LineRef) -> Result<(Vec3, Vec3)> {
        match r {
            LineRef::Given { point, direction } => Ok((self.at(point)?, self.direction(direction, "a line's direction")?)),
            LineRef::Edge(query) => {
                let (body, e) = self.one_edge(query, "a line")?;
                let edge = &body.edges[e];
                let (a, b) = (body.vertices[edge.v0], body.vertices[edge.v1]);
                if !matches!(edge.curve, Curve::Line { .. }) {
                    return Err(bad(format!("the edge through {} is {}, where a line has to be straight", place_of(edge.eval_mid()), curve_kind(&edge.curve))));
                }
                Ok((a, (b - a).normalized()))
            }
            LineRef::Sketch { sketch, line } => {
                let (frame, solved) = self.doc.solved_sketch(*sketch)?;
                match solved.entity(*line).map(|e| &e.geometry) {
                    Some(Geometry::Line { start, end }) => {
                        let (a, b) = (frame.point(start[0], start[1], 0.0), frame.point(end[0], end[1], 0.0));
                        if a.dist(b) <= TOL {
                            return Err(bad(format!("line {} of sketch f{} has no length, so it has no direction", line.0, sketch.0)));
                        }
                        Ok((a, (b - a).normalized()))
                    }
                    Some(other) => Err(bad(format!("entity {} of sketch f{} is a {}, not a line", line.0, sketch.0, other.kind()))),
                    None => Err(bad(format!("sketch f{} has no entity {}", sketch.0, line.0))),
                }
            }
            LineRef::AxisOfFace(key) => {
                let (body, f) = self.one_surface(key)?;
                let surface = &body.faces[f].surface;
                surface.axis().ok_or_else(|| bad(format!("the face {} is {}, which has no axis", key.text(), surface_kind(surface))))
            }
            LineRef::AxisOfEdge(query) => {
                let (body, e) = self.one_edge(query, "an axis")?;
                let edge = &body.edges[e];
                match &edge.curve {
                    Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => Ok((frame.origin, frame.z)),
                    other => Err(bad(format!("the edge through {} is {}, which has no axis", place_of(edge.eval_mid()), curve_kind(other)))),
                }
            }
        }
    }

    /// A plane feature's plane.
    pub(crate) fn def(&self, def: &PlaneDef, flip: bool) -> Result<Frame> {
        let f = self.unflipped(def)?;
        Ok(if flip { Frame::new(f.origin, -f.z, f.x) } else { f })
    }

    fn unflipped(&self, def: &PlaneDef) -> Result<Frame> {
        match def {
            PlaneDef::Offset { plane, distance } => {
                let f = self.plane(plane)?;
                Ok(Frame { origin: f.origin + f.z * num(distance, self.env)?, ..f })
            }
            PlaneDef::PlanePoint { plane, point } => Ok(Frame { origin: self.point(point)?, ..self.plane(plane)? }),
            PlaneDef::LineAngle { line, degrees, reference } => {
                let (q, d) = self.line(line)?;
                let r = match reference {
                    Some(plane) => {
                        let r = self.plane(plane)?.z;
                        if (r - d * r.dot(d)).norm() < 1e-9 {
                            return Err(bad(format!("the line runs square to {}, so no plane through it comes nearer to being parallel to it than another, and the angle has nothing to turn from", plane.describe())));
                        }
                        r
                    }
                    // The default plane the line is most nearly parallel to.
                    None => [Vec3::Z, -Vec3::Y, Vec3::X].into_iter().fold((f64::MAX, Vec3::Z), |(best, n), m| if m.dot(d).abs() < best - 1e-9 { (m.dot(d).abs(), m) } else { (best, n) }).1,
                };
                let n0 = (r - d * r.dot(d)).normalized();
                let (s, c) = num(degrees, self.env)?.to_radians().sin_cos();
                Ok(Frame::new(q, n0 * c + d.cross(n0) * s, d))
            }
            PlaneDef::PointNormal { point, line } => {
                let (_, d) = self.line(line)?;
                Ok(Frame::new(self.point(point)?, d, default_x(d)))
            }
            PlaneDef::ThreePoint { points } => {
                let p = [self.point(&points[0])?, self.point(&points[1])?, self.point(&points[2])?];
                const WORDS: [&str; 3] = ["first", "second", "third"];
                for (i, j) in [(0, 1), (0, 2), (1, 2)] {
                    if p[i].dist(p[j]) <= TOL {
                        return Err(bad(format!("the {} and {} points are the same point, {}, so three points don't fix a plane", WORDS[i], WORDS[j], place_of(p[i]))));
                    }
                }
                let (u, v) = (p[1] - p[0], p[2] - p[0]);
                let n = u.cross(v);
                if n.norm() <= 1e-9 * u.norm() * v.norm() {
                    return Err(bad(format!("the three points {}, {} and {} are in a line, so they don't fix a plane", place_of(p[0]), place_of(p[1]), place_of(p[2]))));
                }
                Ok(Frame::new(p[0], n, u))
            }
            PlaneDef::MidPlanes { planes } => {
                let (a, b) = (self.plane(&planes[0])?, self.plane(&planes[1])?);
                // The second turned to face the same side as the first, and each point's distances from them adding to nothing.
                let nb = if a.z.dot(b.z) < 0.0 { -b.z } else { b.z };
                let sum = a.z + nb;
                let level = a.origin.dot(a.z) + b.origin.dot(nb);
                let middle = (a.origin + b.origin) * 0.5;
                let origin = middle - sum * ((middle.dot(sum) - level) / sum.dot(sum));
                Ok(Frame::new(origin, sum, a.x))
            }
            PlaneDef::MidPoints { points } => {
                let (a, b) = (self.point(&points[0])?, self.point(&points[1])?);
                if a.dist(b) <= TOL {
                    return Err(bad(format!("the two points are the same point, {}, so there's no line between them", place_of(a))));
                }
                Ok(Frame::new((a + b) * 0.5, b - a, default_x(b - a)))
            }
            PlaneDef::CurvePoint { edge, at } => {
                let (body, e) = self.one_edge(edge, "a curve-point plane")?;
                let edge = &body.edges[e];
                let t = match at {
                    AlongEdge::Point(p) => edge.curve.closest(self.point(p)?, edge.t0, edge.t1).0,
                    AlongEdge::Fraction(f) => {
                        let f = num(f, self.env)?;
                        if !(0.0..=1.0).contains(&f) {
                            return Err(bad(format!("a fraction along an edge runs from 0 to 1, and this is {f}")));
                        }
                        at_length(&edge.curve, edge.t0, edge.t1, f)
                    }
                };
                let d = edge.curve.derivative(t);
                if !(d.norm() > 1e-12) {
                    return Err(bad(format!("the edge has no direction at {}, so no plane is square to it there", place_of(edge.curve.eval(t)))));
                }
                let d = d.normalized();
                Ok(Frame::new(edge.curve.eval(t), d, default_x(d)))
            }
            PlaneDef::Tangent { face: key, at } => {
                let (body, f) = self.one_surface(key)?;
                let face = &body.faces[f];
                let surface = &face.surface;
                if !matches!(surface, Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. }) {
                    return Err(bad(format!("the face {} is {}, and a tangent plane touches a cylinder, cone or sphere", key.text(), surface_kind(surface))));
                }
                let touch = match at {
                    TangentAt::Point(p) => {
                        let p = self.point(p)?;
                        let off = match surface {
                            Surface::Sphere { frame, .. } => p.dist(frame.origin),
                            _ => {
                                let (o, a) = surface.axis().expect("round");
                                (p - o - a * (p - o).dot(a)).norm()
                            }
                        };
                        if off <= TOL {
                            let middle = if matches!(surface, Surface::Sphere { .. }) { "center" } else { "axis" };
                            return Err(bad(format!("the point {} is on the {middle} of {}, so it's as near every side of it", place_of(p), key.text())));
                        }
                        surface.project(p)
                    }
                    TangentAt::Degrees(degrees) => {
                        let (o, a) = surface.axis().ok_or_else(|| bad(format!("the face {} is a sphere, which has no axis to go round: give a tangent plane on it a point", key.text())))?;
                        let zero = default_x(a);
                        let zero = (zero - a * zero.dot(a)).normalized();
                        let (s, c) = num(degrees, self.env)?.to_radians().sin_cos();
                        let out = zero * c + a.cross(zero) * s;
                        // At the height of the middle of the face.
                        let (mut sum, mut n) = (Vec3::ZERO, 0.0f64);
                        for coedge in face.loops.iter().flatten() {
                            sum = sum + body.coedge_point(*coedge, 0.5);
                            n += 1.0;
                        }
                        let height = (sum * (1.0 / n.max(1.0)) - o).dot(a);
                        let radius = match *surface {
                            Surface::Cylinder { radius, .. } => radius,
                            Surface::Cone { frame, radius, half_angle } => radius + height * a.dot(frame.z) * half_angle.tan(),
                            _ => unreachable!("only cylinders and cones have an axis here"),
                        };
                        if radius <= TOL {
                            return Err(bad(format!("the face {} comes to a point at the height of its middle", key.text())));
                        }
                        o + a * height + out * radius
                    }
                };
                let n = face.outward_normal(touch);
                let x = match surface.axis() {
                    Some((_, a)) if n.cross(a).norm() > 1e-9 => a,
                    _ => default_x(n),
                };
                Ok(Frame::new(touch, n, x))
            }
        }
    }
}

/// A point of a solved sketch, in its own coordinates.
fn sketch_point(sketch: &coreshape_sketch::Sketch, r: Ref) -> std::result::Result<[f64; 2], String> {
    if r.entity == coreshape_sketch::model::ORIGIN && r.part == Part::Whole {
        return Ok([0.0, 0.0]);
    }
    let e = sketch.entity(r.entity).ok_or_else(|| format!("it has no entity {}", r.entity.0))?;
    let found = match (&e.geometry, r.part) {
        (Geometry::Point { at }, Part::Whole) => Some(*at),
        (Geometry::Line { start, .. } | Geometry::Arc { start, .. } | Geometry::EllipticalArc { start, .. }, Part::Start) => Some(*start),
        (Geometry::Line { end, .. } | Geometry::Arc { end, .. } | Geometry::EllipticalArc { end, .. }, Part::End) => Some(*end),
        (Geometry::Circle { center, .. } | Geometry::Arc { center, .. } | Geometry::Ellipse { center, .. } | Geometry::EllipticalArc { center, .. }, Part::Center) => Some(*center),
        (Geometry::Ellipse { major, .. } | Geometry::EllipticalArc { major, .. }, Part::Major) => Some(*major),
        (Geometry::Spline(s), Part::Start) => Some(s.eval(s.domain().0).0),
        (Geometry::Spline(s), Part::End) => Some(s.eval(s.domain().1).0),
        (Geometry::Spline(s), Part::Control(i)) => s.points.get(i as usize).copied(),
        _ => None,
    };
    found.ok_or_else(|| {
        let what = if r.part == Part::Whole { "a point".to_string() } else { format!("a {}", r.name().rsplit('.').next().unwrap_or("")) };
        format!("entity {} is a {}, which has no {what}", r.entity.0, e.geometry.kind())
    })
}

/// The parameter a fraction of a curve's length along from `t0`.
fn at_length(curve: &Curve, t0: f64, t1: f64, fraction: f64) -> f64 {
    if matches!(curve, Curve::Line { .. } | Curve::Circle { .. }) {
        return t0 + (t1 - t0) * fraction;
    }
    // Gauss-Legendre with five points on each of many panels, then the panel the length falls in split in half until it's
    // found.
    const X: [f64; 5] = [0.0, -0.538_469_310_105_683_1, 0.538_469_310_105_683_1, -0.906_179_845_938_664, 0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
    let length = |a: f64, b: f64| -> f64 {
        let (mid, half) = ((a + b) / 2.0, (b - a) / 2.0);
        X.iter().zip(W).map(|(x, w)| w * curve.derivative(mid + half * x).norm()).sum::<f64>() * half
    };
    const PANELS: usize = 256;
    let step = (t1 - t0) / PANELS as f64;
    let mut sums = vec![0.0; PANELS + 1];
    for k in 0..PANELS {
        sums[k + 1] = sums[k] + length(t0 + step * k as f64, t0 + step * (k + 1) as f64);
    }
    let want = fraction * sums[PANELS];
    let k = sums.partition_point(|s| *s < want).saturating_sub(1).min(PANELS - 1);
    let (mut lo, mut hi) = (t0 + step * k as f64, t0 + step * (k + 1) as f64);
    let start = lo;
    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        if sums[k] + length(start, mid) < want {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

// ----- Reading and writing -----

fn feature_id(v: &Value, what: &str) -> Result<FeatureId> {
    v.as_f64()
        .filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < u32::MAX as f64)
        .map(|n| FeatureId(n as u32))
        .ok_or_else(|| bad(format!("{what} must be a feature id")))
}

const PLANE_FORMS: &str = "a plane is \"top\", \"front\", \"right\", a plane feature's id, a flat face's name or key, or {\"point\": [x, y, z], \"normal\": [x, y, z]}";

/// A plane: `"top"`, `"front"`, `"right"`, a plane feature's id, a flat face's name or key, or
/// `{"point": [x, y, z], "normal": [x, y, z]}`.
pub fn read_plane_ref(v: &Value) -> Result<PlaneRef> {
    match v {
        Value::Num(_) => Ok(PlaneRef::Feature(feature_id(v, "a plane named by number")?)),
        Value::Str(s) => Ok(match s.as_str() {
            "top" => PlaneRef::Top,
            "front" => PlaneRef::Front,
            "right" => PlaneRef::Right,
            other => PlaneRef::Face(FaceKey::parse(other).map_err(|_| bad(format!("\"{other}\" isn't a plane: {PLANE_FORMS}")))?),
        }),
        Value::Obj(_) => {
            known_keys(v, &["point", "normal"])?;
            Ok(PlaneRef::Given {
                point: v.get("point").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
                normal: read_point3(v.get("normal").ok_or_else(|| bad(PLANE_FORMS.into()))?)?,
            })
        }
        _ => Err(bad(PLANE_FORMS.into())),
    }
}

pub fn plane_ref_value(r: &PlaneRef) -> Value {
    match r {
        PlaneRef::Top => Value::str("top"),
        PlaneRef::Front => Value::str("front"),
        PlaneRef::Right => Value::str("right"),
        PlaneRef::Feature(id) => Value::Num(id.0 as f64),
        PlaneRef::Face(key) => Value::str(key.text()),
        PlaneRef::Given { point, normal } => Value::obj(vec![("point", point3_value(point)), ("normal", point3_value(normal))]),
    }
}

const POINT_FORMS: &str = "a point is \"origin\", [x, y, z], a corner {\"faces\": [...], \"near\": [x, y, z]} as vertices_list gives it, or a sketch's point {\"sketch\": 4, \"point\": \"7.start\"}";

/// A point: `"origin"`, `[x, y, z]`, a corner `{"faces": [...], "near": [x, y, z]}`, or a sketch's point
/// `{"sketch": 4, "point": "7.start"}`.
pub fn read_point_ref(v: &Value) -> Result<PointRef> {
    match v {
        Value::Str(s) if s == "origin" => Ok(PointRef::Origin),
        Value::Arr(_) => Ok(PointRef::At(read_point3(v)?)),
        Value::Obj(_) if v.get("sketch").is_some() => {
            known_keys(v, &["sketch", "point"])?;
            let text = v.get("point").and_then(Value::as_str).ok_or_else(|| bad("a sketch's point names it as constraints do, like \"7.start\"".into()))?;
            let point = Ref::parse(text).map_err(bad)?;
            if matches!(point.part, Part::Radius) || point.entity == coreshape_sketch::model::X_AXIS || point.entity == coreshape_sketch::model::Y_AXIS {
                return Err(bad(format!("\"{text}\" isn't a point of a sketch")));
            }
            Ok(PointRef::Sketch { sketch: feature_id(v.get("sketch").expect("checked"), "\"sketch\"")?, point })
        }
        Value::Obj(_) if v.get("faces").is_some() => match read_vertex_ref(v)? {
            VertexRef::Faces { faces, near } => Ok(PointRef::Vertex { faces, near }),
            VertexRef::Point(p) => Ok(PointRef::At(p)),
        },
        Value::Obj(_) if v.get("point").is_some() => {
            known_keys(v, &["point"])?;
            Ok(PointRef::At(read_point3(v.get("point").expect("checked"))?))
        }
        _ => Err(bad(POINT_FORMS.into())),
    }
}

pub fn point_ref_value(r: &PointRef) -> Value {
    match r {
        PointRef::Origin => Value::str("origin"),
        PointRef::At(p) => point3_value(p),
        PointRef::Vertex { faces, near } => {
            let mut items = vec![("faces", key_list_value(faces))];
            if let Some(p) = near {
                items.push(("near", point3_value(p)));
            }
            Value::obj(items)
        }
        PointRef::Sketch { sketch, point } => Value::obj(vec![("sketch", Value::Num(sketch.0 as f64)), ("point", Value::str(point.name()))]),
    }
}

const LINE_FORMS: &str = "a line is a straight edge's selection, {\"sketch\": 4, \"line\": 9}, {\"axis_of\": face or edge selection}, or {\"point\": [x, y, z], \"direction\": [x, y, z]}";

/// A line: a straight edge's selection, `{"sketch": 4, "line": 9}`, `{"axis_of": "f3:side.0"}` or of an edge selection, or
/// `{"point": [x, y, z], "direction": [x, y, z]}`.
pub fn read_line_ref(v: &Value) -> Result<LineRef> {
    if v.get("sketch").is_some() {
        known_keys(v, &["sketch", "line"])?;
        let line = v.get("line").and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < (u32::MAX - 1) as f64);
        let line = line.ok_or_else(|| bad("a sketch's line is {\"sketch\": 4, \"line\": 9}, with the line entity's id".into()))?;
        return Ok(LineRef::Sketch { sketch: feature_id(v.get("sketch").expect("checked"), "\"sketch\"")?, line: EntityId(line as u32) });
    }
    if let Some(of) = v.get("axis_of") {
        known_keys(v, &["axis_of"])?;
        return Ok(match of.as_str() {
            Some(name) => LineRef::AxisOfFace(FaceKey::parse(name)?),
            None => LineRef::AxisOfEdge(read_edge_select(of)?),
        });
    }
    if v.get("direction").is_some() {
        known_keys(v, &["point", "direction"])?;
        return Ok(LineRef::Given {
            point: v.get("point").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
            direction: read_point3(v.get("direction").expect("checked"))?,
        });
    }
    if let Some(edge) = v.get("edge") {
        known_keys(v, &["edge"])?;
        return Ok(LineRef::Edge(read_edge_select(edge)?));
    }
    read_edge_select(v).map(LineRef::Edge).map_err(|_| bad(LINE_FORMS.into()))
}

pub fn line_ref_value(r: &LineRef) -> Value {
    match r {
        LineRef::Given { point, direction } => Value::obj(vec![("point", point3_value(point)), ("direction", point3_value(direction))]),
        LineRef::Edge(edges) => edge_select_value(edges),
        LineRef::Sketch { sketch, line } => Value::obj(vec![("sketch", Value::Num(sketch.0 as f64)), ("line", Value::Num(line.0 as f64))]),
        LineRef::AxisOfFace(key) => Value::obj(vec![("axis_of", Value::str(key.text()))]),
        LineRef::AxisOfEdge(edges) => Value::obj(vec![("axis_of", edge_select_value(edges))]),
    }
}

/// A plane feature's fields, after `"type": "plane"`.
pub(crate) fn plane_value(def: &PlaneDef, flip: bool) -> Value {
    let mut items = vec![("type", Value::str("plane")), ("plane_type", Value::str(def.kind()))];
    match def {
        PlaneDef::Offset { plane, distance } => {
            items.push(("plane", plane_ref_value(plane)));
            items.push(("distance", expr_value(distance)));
        }
        PlaneDef::PlanePoint { plane, point } => {
            items.push(("plane", plane_ref_value(plane)));
            items.push(("point", point_ref_value(point)));
        }
        PlaneDef::LineAngle { line, degrees, reference } => {
            items.push(("line", line_ref_value(line)));
            items.push(("degrees", expr_value(degrees)));
            if let Some(r) = reference {
                items.push(("reference", plane_ref_value(r)));
            }
        }
        PlaneDef::PointNormal { point, line } => {
            items.push(("point", point_ref_value(point)));
            items.push(("line", line_ref_value(line)));
        }
        PlaneDef::ThreePoint { points } => items.push(("points", Value::Arr(points.iter().map(point_ref_value).collect()))),
        PlaneDef::MidPlanes { planes } => items.push(("planes", Value::Arr(planes.iter().map(plane_ref_value).collect()))),
        PlaneDef::MidPoints { points } => items.push(("points", Value::Arr(points.iter().map(point_ref_value).collect()))),
        PlaneDef::CurvePoint { edge, at } => {
            items.push(("edge", edge_select_value(edge)));
            match at {
                AlongEdge::Point(p) => items.push(("point", point_ref_value(p))),
                AlongEdge::Fraction(f) => items.push(("fraction", expr_value(f))),
            }
        }
        PlaneDef::Tangent { face, at } => {
            items.push(("face", Value::str(face.text())));
            match at {
                TangentAt::Point(p) => items.push(("point", point_ref_value(p))),
                TangentAt::Degrees(d) => items.push(("degrees", expr_value(d))),
            }
        }
    }
    if flip {
        items.push(("flip", Value::Bool(true)));
    }
    Value::obj(items)
}

/// A plane feature from JSON. `with` checks its fields, adding those the caller owns.
pub(crate) fn read_plane(v: &Value, with: &dyn Fn(&[&str]) -> Result<()>) -> Result<Op> {
    let how = v.get("plane_type").and_then(Value::as_str).unwrap_or("");
    let need = |key: &str| v.get(key).ok_or_else(|| bad(format!("a {how} plane needs \"{key}\"")));
    let fields = |own: &[&str]| -> Result<()> {
        let mut all = vec!["type", "plane_type", "flip"];
        all.extend_from_slice(own);
        with(&all)
    };
    let list = |key: &str, n: usize| -> Result<&[Value]> {
        need(key)?.as_array().filter(|l| l.len() == n).ok_or_else(|| bad(format!("a {how} plane's \"{key}\" is a list of {n}")))
    };
    let def = match how {
        "offset" => {
            fields(&["plane", "distance"])?;
            PlaneDef::Offset { plane: read_plane_ref(need("plane")?)?, distance: read_expr(need("distance")?)? }
        }
        "plane_point" => {
            fields(&["plane", "point"])?;
            PlaneDef::PlanePoint { plane: read_plane_ref(need("plane")?)?, point: read_point_ref(need("point")?)? }
        }
        "line_angle" => {
            fields(&["line", "degrees", "reference"])?;
            PlaneDef::LineAngle {
                line: read_line_ref(need("line")?)?,
                degrees: v.get("degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(0.0)),
                reference: v.get("reference").map(read_plane_ref).transpose()?,
            }
        }
        "point_normal" => {
            fields(&["point", "line"])?;
            PlaneDef::PointNormal { point: read_point_ref(need("point")?)?, line: read_line_ref(need("line")?)? }
        }
        "three_point" => {
            fields(&["points"])?;
            let l = list("points", 3)?;
            PlaneDef::ThreePoint { points: [read_point_ref(&l[0])?, read_point_ref(&l[1])?, read_point_ref(&l[2])?] }
        }
        "mid_plane" => match (v.get("planes"), v.get("points")) {
            (Some(_), None) => {
                fields(&["planes"])?;
                let l = list("planes", 2)?;
                PlaneDef::MidPlanes { planes: [read_plane_ref(&l[0])?, read_plane_ref(&l[1])?] }
            }
            (None, Some(_)) => {
                fields(&["points"])?;
                let l = list("points", 2)?;
                PlaneDef::MidPoints { points: [read_point_ref(&l[0])?, read_point_ref(&l[1])?] }
            }
            _ => return Err(bad("a mid_plane plane takes \"planes\", two of them, or \"points\", two of them".into())),
        },
        "curve_point" => {
            fields(&["edge", "point", "fraction"])?;
            let at = match (v.get("point"), v.get("fraction")) {
                (Some(p), None) => AlongEdge::Point(read_point_ref(p)?),
                (None, Some(f)) => AlongEdge::Fraction(read_expr(f)?),
                _ => return Err(bad("a curve_point plane takes \"point\", a point to be nearest, or \"fraction\", how far along the edge from its start".into())),
            };
            PlaneDef::CurvePoint { edge: read_edge_select(need("edge")?)?, at }
        }
        "tangent" => {
            fields(&["face", "point", "degrees"])?;
            let face = FaceKey::parse(need("face")?.as_str().ok_or_else(|| bad("a tangent plane's \"face\" is a face's name or key".into()))?)?;
            let at = match (v.get("point"), v.get("degrees")) {
                (Some(p), None) => TangentAt::Point(read_point_ref(p)?),
                (None, Some(d)) => TangentAt::Degrees(read_expr(d)?),
                _ => return Err(bad("a tangent plane takes \"point\", a point to be nearest, or \"degrees\", an angle around a cylinder's or cone's axis".into())),
            };
            PlaneDef::Tangent { face, at }
        }
        "" => return Err(bad("a plane needs \"plane_type\": offset, plane_point, line_angle, point_normal, three_point, mid_plane, curve_point or tangent".into())),
        other => return Err(bad(format!("unknown plane_type \"{other}\"; expected offset, plane_point, line_angle, point_normal, three_point, mid_plane, curve_point or tangent"))),
    };
    let flip = match v.get("flip") {
        None => false,
        Some(b) => b.as_bool().ok_or_else(|| bad("a plane's \"flip\" must be true or false".into()))?,
    };
    Ok(Op::Plane { def, flip })
}
