//! Where a mate connector sits on a part, by what it's found from, so that it follows when the part changes. These are the
//! places Onshape offers when you hover over a face or an edge.

use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::topo::Body;
use coreshape::{Curve, Surface};
use coreshape_document::json::Value;
use coreshape_document::{find_faces, FaceKey};

/// A place on a face.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FacePoint {
    /// The middle of its area, on the face's plane, or on the axis for a round face.
    Centroid,
    /// On a round face's axis, at the middle of its length, and at each end.
    AxisMiddle,
    AxisStart,
    AxisEnd,
    /// A cone's tip.
    Apex,
    /// The nearest of the face's corners, edge middles and arc centers to this point, in the part's coordinates.
    Near(Vec3),
}

/// A place on an edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EdgePoint {
    Middle,
    Start,
    End,
    /// An arc's center, on its axis.
    Center,
}

/// What a connector is found from.
#[derive(Clone, Debug, PartialEq)]
pub enum Anchor {
    /// Just a place, in the part's own coordinates. It doesn't follow anything.
    Frame(Frame),
    Face { key: FaceKey, point: FacePoint },
    /// The edge between two faces.
    Edge { faces: [FaceKey; 2], point: EdgePoint },
}

/// A place on offer, ready to use.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    pub label: String,
    pub anchor: Anchor,
    pub frame: Frame,
}

/// A frame with this z, keeping as much of this x as it can.
fn frame_along(origin: Vec3, z: Vec3, x: Vec3) -> Frame {
    if z.cross(x).norm() > 1e-9 {
        Frame::new(origin, z, x)
    } else {
        Frame::new(origin, z, z.any_perpendicular())
    }
}

fn face_normal(body: &Body, face: usize, at: Vec3) -> Vec3 {
    let f = &body.faces[face];
    let n = f.surface.normal(at);
    if f.flipped {
        -n
    } else {
        n
    }
}

/// Points around a face's loops, for measuring it.
fn boundary_samples(body: &Body, face: usize) -> Vec<Vec3> {
    let mut out = Vec::new();
    for c in body.faces[face].loops.iter().flatten() {
        for k in 0..32 {
            out.push(body.coedge_point(*c, k as f64 / 32.0));
        }
    }
    out
}

/// The middle of a flat face's area, holes and all, and curved edges closely enough for a place to put something.
fn plane_centroid(body: &Body, face: usize, frame: &Frame) -> Vec3 {
    let (mut area, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for lp in &body.faces[face].loops {
        let mut points = Vec::new();
        for c in lp {
            for k in 0..64 {
                let l = frame.to_local(body.coedge_point(*c, k as f64 / 64.0));
                points.push((l.x, l.y));
            }
        }
        for i in 0..points.len() {
            let (p, q) = (points[i], points[(i + 1) % points.len()]);
            let cross = p.0 * q.1 - q.0 * p.1;
            area += cross;
            cx += (p.0 + q.0) * cross;
            cy += (p.1 + q.1) * cross;
        }
    }
    if area.abs() < 1e-300 {
        return frame.origin;
    }
    frame.to_world(v3(cx / (3.0 * area), cy / (3.0 * area), 0.0))
}

/// How far along an axis a face reaches, as its lowest and highest.
fn along_axis(body: &Body, face: usize, frame: &Frame) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in boundary_samples(body, face) {
        let h = frame.to_local(p).z;
        lo = lo.min(h);
        hi = hi.max(h);
    }
    (lo, hi)
}

fn face_frame(body: &Body, face: usize, point: FacePoint) -> Result<Frame> {
    let f = &body.faces[face];
    let refused = |what: &str| Err(Error::InvalidInput(format!("a {} face has no {what}", f.surface.name())));
    let on_axis = |frame: &Frame, h: f64| Frame::new(frame.to_world(v3(0.0, 0.0, h)), frame.z, frame.x);
    Ok(match (point, &f.surface) {
        (FacePoint::Near(want), _) => {
            let mut best: Option<(f64, Vec3)> = None;
            let mut consider = |p: Vec3| {
                let d = p.dist(want);
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, p));
                }
            };
            for c in f.loops.iter().flatten() {
                let e = &body.edges[c.edge];
                consider(body.vertices[e.v0]);
                consider(body.vertices[e.v1]);
                consider(e.eval_mid());
                if let Curve::Circle { frame, .. } = &e.curve {
                    consider(frame.origin);
                }
            }
            let at = best.map_or(want, |b| b.1);
            let on = f.surface.project(at);
            match &f.surface {
                Surface::Plane(frame) => frame_along(at, face_normal(body, face, on), frame.x),
                Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => frame_along(at, frame.z, frame.x),
                _ => frame_along(at, face_normal(body, face, on), Vec3::X),
            }
        }
        (FacePoint::Centroid, Surface::Plane(frame)) => {
            let at = plane_centroid(body, face, frame);
            frame_along(at, face_normal(body, face, at), frame.x)
        }
        (FacePoint::Centroid | FacePoint::AxisMiddle, Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. }) => {
            let (lo, hi) = along_axis(body, face, frame);
            on_axis(frame, (lo + hi) / 2.0)
        }
        (FacePoint::AxisStart, Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. }) => on_axis(frame, along_axis(body, face, frame).0),
        (FacePoint::AxisEnd, Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. }) => on_axis(frame, along_axis(body, face, frame).1),
        (FacePoint::Apex, Surface::Cone { frame, .. }) => {
            let tip = f.surface.apex().ok_or_else(|| Error::InvalidInput("this cone has no tip".into()))?;
            Frame::new(tip, frame.z, frame.x)
        }
        (FacePoint::Centroid | FacePoint::AxisMiddle, Surface::Sphere { frame, .. } | Surface::Torus { frame, .. }) => *frame,
        (FacePoint::Centroid, _) => {
            // No formula for a free-form face. The middle of its boundary, brought onto it, does for a place.
            let samples = boundary_samples(body, face);
            let sum = samples.iter().fold(Vec3::ZERO, |s, p| s + *p);
            let on = f.surface.project(sum * (1.0 / samples.len().max(1) as f64));
            frame_along(on, face_normal(body, face, on), Vec3::X)
        }
        (FacePoint::AxisMiddle | FacePoint::AxisStart | FacePoint::AxisEnd, _) => return refused("axis"),
        (FacePoint::Apex, _) => return refused("tip"),
    })
}

fn edge_frame(body: &Body, edge: usize, point: EdgePoint) -> Result<Frame> {
    let e = &body.edges[edge];
    let t = match point {
        EdgePoint::Start => e.t0,
        EdgePoint::End => e.t1,
        _ => (e.t0 + e.t1) / 2.0,
    };
    Ok(match (point, &e.curve) {
        (EdgePoint::Center, Curve::Circle { frame, .. }) => *frame,
        (EdgePoint::Center, Curve::Ellipse { frame, .. }) => *frame,
        (EdgePoint::Center, _) => return Err(Error::InvalidInput("only a round edge has a center".into())),
        // A round edge's frame stays square to its circle, wherever along it this is.
        (_, Curve::Circle { frame, .. }) => Frame::new(e.curve.eval(t), frame.z, frame.x),
        // Otherwise, along the edge there.
        _ => {
            let along = e.curve.derivative(t);
            frame_along(e.curve.eval(t), along.normalized(), along.normalized().any_perpendicular())
        }
    })
}

/// The edge between two faces, by their keys.
pub fn edge_between(body: &Body, faces: &[FaceKey; 2]) -> Option<usize> {
    let (a, b) = (find_faces(body, &faces[0]), find_faces(body, &faces[1]));
    let uses = body.edge_uses();
    (0..body.edges.len()).find(|&e| {
        let on = |list: &[usize]| uses[e].iter().any(|u| list.contains(&u.0));
        on(&a) && on(&b)
    })
}

impl Anchor {
    /// Where this is on a part, in the part's own coordinates. It's an error if what it's found from has gone.
    pub fn resolve(&self, body: &Body) -> Result<Frame> {
        match self {
            Anchor::Frame(frame) => Ok(*frame),
            Anchor::Face { key, point } => {
                let found = find_faces(body, key);
                let face = found.first().ok_or_else(|| Error::InvalidInput(format!("the face {} isn't on this part", key.text())))?;
                face_frame(body, *face, *point)
            }
            Anchor::Edge { faces, point } => {
                let edge = edge_between(body, faces);
                let edge = edge.ok_or_else(|| Error::InvalidInput(format!("there's no edge between {} and {} on this part", faces[0].text(), faces[1].text())))?;
                edge_frame(body, edge, *point)
            }
        }
    }

    pub fn to_json(&self) -> Value {
        match self {
            Anchor::Frame(frame) => Value::obj(vec![("frame", crate::frame_value(frame))]),
            Anchor::Face { key, point } => {
                let mut items = vec![("face", Value::str(key.text()))];
                match point {
                    FacePoint::Near(p) => items.push(("near", Value::Arr(vec![Value::Num(p.x), Value::Num(p.y), Value::Num(p.z)]))),
                    other => items.push((
                        "point",
                        Value::str(match other {
                            FacePoint::Centroid => "centroid",
                            FacePoint::AxisMiddle => "axis_middle",
                            FacePoint::AxisStart => "axis_start",
                            FacePoint::AxisEnd => "axis_end",
                            _ => "apex",
                        }),
                    )),
                }
                Value::obj(items)
            }
            Anchor::Edge { faces, point } => Value::obj(vec![
                ("edge", Value::Arr(vec![Value::str(faces[0].text()), Value::str(faces[1].text())])),
                (
                    "point",
                    Value::str(match point {
                        EdgePoint::Middle => "middle",
                        EdgePoint::Start => "start",
                        EdgePoint::End => "end",
                        EdgePoint::Center => "center",
                    }),
                ),
            ]),
        }
    }

    pub fn from_json(v: &Value) -> Result<Anchor> {
        if let Some(frame) = v.get("frame") {
            return Ok(Anchor::Frame(crate::read_frame(frame)?));
        }
        let point_name = v.get("point").and_then(Value::as_str);
        if let Some(face) = v.get("face") {
            let key = FaceKey::parse(face.as_str().ok_or_else(|| Error::InvalidInput("\"face\" must be the name of a face".into()))?)?;
            if let Some(near) = v.get("near") {
                return Ok(Anchor::Face { key, point: FacePoint::Near(crate::read_vec3(near)?) });
            }
            let point = match point_name.unwrap_or("centroid") {
                "centroid" => FacePoint::Centroid,
                "axis_middle" => FacePoint::AxisMiddle,
                "axis_start" => FacePoint::AxisStart,
                "axis_end" => FacePoint::AxisEnd,
                "apex" => FacePoint::Apex,
                other => return Err(Error::InvalidInput(format!("unknown place on a face \"{other}\"; expected centroid, axis_middle, axis_start, axis_end or apex"))),
            };
            return Ok(Anchor::Face { key, point });
        }
        if let Some(edge) = v.get("edge") {
            let names = edge.as_array().filter(|l| l.len() == 2).ok_or_else(|| Error::InvalidInput("\"edge\" must be the names of its two faces".into()))?;
            let key = |n: &Value| FaceKey::parse(n.as_str().ok_or_else(|| Error::InvalidInput("\"edge\" must be the names of its two faces".into()))?);
            let point = match point_name.unwrap_or("middle") {
                "middle" => EdgePoint::Middle,
                "start" => EdgePoint::Start,
                "end" => EdgePoint::End,
                "center" => EdgePoint::Center,
                other => return Err(Error::InvalidInput(format!("unknown place on an edge \"{other}\"; expected middle, start, end or center"))),
            };
            return Ok(Anchor::Edge { faces: [key(&names[0])?, key(&names[1])?], point });
        }
        Err(Error::InvalidInput("a connector's place needs \"frame\", \"face\" or \"edge\"".into()))
    }
}

/// The places to offer on a face, as Onshape does when you hover over one: its middle, its axis, and its corners, edge
/// middles and arc centers. `key` is what they'll be found from from now on.
pub fn candidates(body: &Body, face: usize, key: &FaceKey) -> Vec<Candidate> {
    let mut out = Vec::new();
    let mut add = |label: String, point: FacePoint| {
        if let Ok(frame) = face_frame(body, face, point) {
            out.push(Candidate { label, anchor: Anchor::Face { key: key.clone(), point }, frame });
        }
    };
    let surface = &body.faces[face].surface;
    match surface {
        Surface::Cylinder { .. } | Surface::Cone { .. } => {
            add("middle of the axis".into(), FacePoint::AxisMiddle);
            add("start of the axis".into(), FacePoint::AxisStart);
            add("end of the axis".into(), FacePoint::AxisEnd);
            if matches!(surface, Surface::Cone { .. }) {
                add("tip".into(), FacePoint::Apex);
            }
        }
        _ => add("centroid".into(), FacePoint::Centroid),
    }
    let mut seen: Vec<Vec3> = Vec::new();
    for c in body.faces[face].loops.iter().flatten() {
        let e = &body.edges[c.edge];
        let mut places = vec![("corner", body.vertices[e.v0]), ("corner", body.vertices[e.v1]), ("middle of an edge", e.eval_mid())];
        if let Curve::Circle { frame, .. } = &e.curve {
            places.push(("center of an arc", frame.origin));
        }
        for (label, p) in places {
            if !seen.iter().any(|q| q.dist(p) < 1e-6) {
                seen.push(p);
                add(label.to_string(), FacePoint::Near(p));
            }
        }
    }
    out
}

/// The places to offer on an edge: its middle and ends, and its center if it's round. `faces` are the keys of the two
/// faces it's between, which is what they'll be found from from now on.
pub fn edge_candidates(body: &Body, edge: usize, faces: &[FaceKey; 2]) -> Vec<Candidate> {
    let mut out = Vec::new();
    let round = matches!(body.edges[edge].curve, Curve::Circle { .. } | Curve::Ellipse { .. });
    let list: &[(&str, EdgePoint)] = if round {
        &[("center", EdgePoint::Center), ("middle", EdgePoint::Middle), ("start", EdgePoint::Start), ("end", EdgePoint::End)]
    } else {
        &[("middle", EdgePoint::Middle), ("start", EdgePoint::Start), ("end", EdgePoint::End)]
    };
    for (label, point) in list {
        if let Ok(frame) = edge_frame(body, edge, *point) {
            out.push(Candidate { label: label.to_string(), anchor: Anchor::Edge { faces: faces.clone(), point: *point }, frame });
        }
    }
    out
}
