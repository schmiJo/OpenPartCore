//! Constant-radius fillets and chamfers on straight and circular edges.
//!
//! Both reduce to a 2D problem. Cut the two faces across the edge: each becomes a line
//! or an arc through the corner point. A fillet is the circle tangent to both, and a
//! chamfer is the line joining two points. The sliver between that and the corner,
//! swept along the edge, is a tool solid. Subtract it from a convex edge, or add it to
//! a concave one, and the Boolean does all the trimming against neighbouring faces.
//!
//! That keeps this module small and means it can't produce an invalid body without
//! the Boolean's checks refusing it. The price is that it only handles what a swept
//! tool can express, as listed in `docs/KNOWN_LIMITS.md`.

use crate::boolean::classify::{BodyLoc, FaceLoc, Index};
use crate::boolean::ssi::coincident;
use crate::boolean::{intersect, subtract, union};
use crate::build::{box_solid, extrude, revolve, sphere};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{PathBuilder, Profile};
use crate::surface::Surface;
use crate::topo::{Body, Edge};

#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Fillet(f64),
    Chamfer(f64),
}

/// A face's trace in the cross-section plane, near the corner.
#[derive(Clone, Copy, Debug)]
enum Trace {
    Line,
    Arc { centre: Vec2, radius: f64 },
}

#[derive(Clone, Copy, Debug)]
struct Side {
    trace: Trace,
    /// Unit direction from the corner into the face.
    into: Vec2,
    /// Outward normal of the face at the corner.
    normal: Vec2,
    face: usize,
}

/// One logical edge: body edges on the same curve, end to end.
#[derive(Clone, Debug)]
struct Run {
    curve: Curve,
    t0: f64,
    t1: f64,
    v0: usize,
    v1: usize,
    closed: bool,
    faces: [usize; 2],
    /// Body edges making up the run.
    members: Vec<usize>,
}

/// How a swept tool should finish at one end of its edge.
#[derive(Clone, Copy, Debug)]
enum End {
    /// Stop exactly at the end of the edge.
    Flush,
    /// Run past the end by this far: there is nothing there to damage.
    Extend(f64),
    /// Run past the end and then trim to this plane, keeping the side `keep` is on.
    Clip { plane: Frame, keep: Vec3, reach: f64 },
}

/// Rounds the given edges to a radius.
pub fn fillet(body: &Body, edges: &[usize], radius: f64, feature: u32) -> Result<Body> {
    if !(radius.is_finite() && radius >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("fillet radius {radius} is too small")));
    }
    blend(body, edges, Kind::Fillet(radius), feature)
}

/// Bevels the given edges, cutting back by a distance along each face.
pub fn chamfer(body: &Body, edges: &[usize], distance: f64, feature: u32) -> Result<Body> {
    if !(distance.is_finite() && distance >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("chamfer distance {distance} is too small")));
    }
    blend(body, edges, Kind::Chamfer(distance), feature)
}

fn same_curve(a: &Curve, b: &Curve) -> bool {
    match (a, b) {
        (Curve::Line { origin, dir }, Curve::Line { origin: o, dir: d }) => {
            let g = *o - *origin;
            dir.cross(*d).norm() < 1e-9 && (g - *dir * g.dot(*dir)).norm() <= TOL
        }
        (Curve::Circle { frame: f, radius: r }, Curve::Circle { frame: g, radius: s }) => {
            (r - s).abs() <= TOL && f.origin.dist(g.origin) <= TOL && f.z.cross(g.z).norm() < 1e-9
        }
        _ => false,
    }
}

/// Groups selected edges into runs and adds the rest of each logical edge: a circle
/// is split at seams, and picking one piece of a rim means picking the whole rim.
fn build_runs(body: &Body, selected: &[usize]) -> Result<Vec<Run>> {
    let uses = body.edge_uses();
    let size = body.aabb().diagonal();
    let faces_of = |e: usize| -> Result<[usize; 2]> {
        match uses[e].as_slice() {
            [a, b] => Ok([a.0, b.0]),
            _ => Err(Error::InvalidBody(vec![format!("edge {e} is not used by two faces")])),
        }
    };
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for (i, e) in body.edges.iter().enumerate() {
        at_vertex[e.v0].push(i);
        at_vertex[e.v1].push(i);
    }
    // Whether edge `f` continues edge `e`: same curve, between the same two surfaces.
    let continues = |e: usize, f: usize| -> bool {
        let (a, b) = (&body.edges[e], &body.edges[f]);
        if !same_curve(&a.curve, &b.curve) {
            return false;
        }
        let (Ok(fa), Ok(fb)) = (faces_of(e), faces_of(f)) else {
            return false;
        };
        let surf = |i: usize| &body.faces[i].surface;
        let same = |x: usize, y: usize| coincident(surf(x), surf(y), size).is_some();
        (same(fa[0], fb[0]) && same(fa[1], fb[1])) || (same(fa[0], fb[1]) && same(fa[1], fb[0]))
    };

    let mut done = vec![false; body.edges.len()];
    let mut runs = Vec::new();
    for &start in selected {
        if start >= body.edges.len() {
            return Err(Error::InvalidInput(format!("there is no edge {start}")));
        }
        if done[start] {
            continue;
        }
        let e = &body.edges[start];
        if !matches!(e.curve, Curve::Line { .. } | Curve::Circle { .. }) {
            return Err(Error::Unsupported(
                "fillets and chamfers work on straight and circular edges only".into(),
            ));
        }
        done[start] = true;
        let mut members = vec![start];
        let (mut t0, mut t1, mut v0, mut v1) = (e.t0, e.t1, e.v0, e.v1);
        // Grow in both directions along the curve.
        for forward in [true, false] {
            let mut guard = 0;
            loop {
                guard += 1;
                if guard > 64 {
                    break;
                }
                let v = if forward { v1 } else { v0 };
                let next = at_vertex[v]
                    .iter()
                    .copied()
                    .find(|&f| !done[f] && continues(start, f));
                let Some(f) = next else {
                    break;
                };
                let g = &body.edges[f];
                done[f] = true;
                members.push(f);
                let span = g.t1 - g.t0;
                let far = if g.v0 == v { g.v1 } else { g.v0 };
                if forward {
                    t1 += span;
                    v1 = far;
                } else {
                    t0 -= span;
                    v0 = far;
                }
            }
        }
        let closed = v0 == v1;
        runs.push(Run {
            curve: e.curve.clone(),
            t0,
            t1,
            v0,
            v1,
            closed,
            faces: faces_of(start)?,
            members,
        });
    }
    Ok(runs)
}

/// Maps between 3D and the cross-section plane of a run.
struct Section {
    /// Sweep direction is `frame.z`. In 2D, x and y map to `frame.x` and `frame.y` for
    /// a straight run, or to radius and height about `frame.z` for a circular one.
    frame: Frame,
    circular: bool,
    /// Station of the cross-section where the corner was measured.
    station: f64,
}

impl Section {
    /// 3D point of a 2D section point at a station (distance or angle along the run).
    fn lift(&self, q: Vec2, station: f64) -> Vec3 {
        if self.circular {
            let (s, c) = station.sin_cos();
            self.frame.origin + (self.frame.x * c + self.frame.y * s) * q.x + self.frame.z * q.y
        } else {
            self.frame.to_world(v3(q.x, q.y, station))
        }
    }

    fn project(&self, p: Vec3) -> Vec2 {
        let l = self.frame.to_local(p);
        if self.circular {
            v2(l.x.hypot(l.y), l.z)
        } else {
            v2(l.x, l.y)
        }
    }

    /// 2D form of a 3D direction at the measuring station.
    fn project_dir(&self, d: Vec3) -> Vec2 {
        if self.circular {
            let (s, c) = self.station.sin_cos();
            let r = self.frame.x * c + self.frame.y * s;
            v2(d.dot(r), d.dot(self.frame.z))
        } else {
            v2(d.dot(self.frame.x), d.dot(self.frame.y))
        }
    }
}

fn section_of(run: &Run, size: f64) -> Result<Section> {
    match &run.curve {
        Curve::Line { origin, dir } => {
            let frame = Frame::new(*origin + *dir * run.t0, *dir, dir.any_perpendicular());
            Ok(Section {
                frame,
                circular: false,
                station: (run.t1 - run.t0) / 2.0,
            })
        }
        Curve::Circle { frame, .. } => {
            let _ = size;
            Ok(Section {
                frame: *frame,
                circular: true,
                station: (run.t0 + run.t1) / 2.0,
            })
        }
        _ => Err(Error::Unsupported("only straight and circular edges".into())),
    }
}

/// How a face appears in the cross-section, or an error if it can't be swept.
fn trace_of(surface: &Surface, section: &Section, size: f64) -> Result<Trace> {
    let f = &section.frame;
    let unsupported = || {
        Error::Unsupported(format!(
            "this edge lies on a {} that doesn't sweep along the edge",
            surface.name()
        ))
    };
    let parallel = |a: Vec3, b: Vec3| a.cross(b).norm() * size.max(1.0) <= 1e-5;
    let on_axis = |q: Vec3| {
        let d = q - f.origin;
        (d - f.z * d.dot(f.z)).norm() <= 1e-5
    };
    if section.circular {
        match *surface {
            Surface::Plane(p) => {
                if parallel(p.z, f.z) {
                    Ok(Trace::Line)
                } else {
                    Err(unsupported())
                }
            }
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => {
                if parallel(frame.z, f.z) && on_axis(frame.origin) {
                    Ok(Trace::Line)
                } else {
                    Err(unsupported())
                }
            }
            Surface::Sphere { frame, radius } => {
                if on_axis(frame.origin) {
                    Ok(Trace::Arc {
                        centre: section.project(frame.origin),
                        radius,
                    })
                } else {
                    Err(unsupported())
                }
            }
            Surface::Torus { frame, major, minor } => {
                if parallel(frame.z, f.z) && on_axis(frame.origin) {
                    Ok(Trace::Arc {
                        centre: v2(major, (frame.origin - f.origin).dot(f.z)),
                        radius: minor,
                    })
                } else {
                    Err(unsupported())
                }
            }
        }
    } else {
        match *surface {
            Surface::Plane(_) => Ok(Trace::Line),
            Surface::Cylinder { frame, radius } => {
                if parallel(frame.z, f.z) {
                    Ok(Trace::Arc {
                        centre: section.project(frame.origin),
                        radius,
                    })
                } else {
                    Err(unsupported())
                }
            }
            _ => Err(unsupported()),
        }
    }
}

/// Cross-section geometry of one run.
struct Corner {
    p: Vec2,
    sides: [Side; 2],
    convex: bool,
}

fn corner_of(body: &Body, run: &Run, section: &Section, size: f64) -> Result<Corner> {
    let station = section.station;
    let (p3, tangent) = if section.circular {
        let t = station;
        (run.curve.eval(t), run.curve.derivative(t).normalized())
    } else {
        let t = run.t0 + station;
        (run.curve.eval(t), run.curve.derivative(t))
    };
    // Coedge direction on each face tells which side of the edge the face lies on.
    let edge = run.members[0];
    let mut sides = Vec::with_capacity(2);
    for &f in &run.faces {
        let face = &body.faces[f];
        let reversed = face
            .loops
            .iter()
            .flatten()
            .find(|c| c.edge == edge)
            .map(|c| c.reversed)
            .ok_or_else(|| Error::Internal("edge is missing from its own face".into()))?;
        let n3 = face.outward_normal(p3);
        let dir = if reversed { -tangent } else { tangent };
        let into3 = n3.cross(dir);
        sides.push(Side {
            trace: trace_of(&face.surface, section, size)?,
            into: section.project_dir(into3).normalized(),
            normal: section.project_dir(n3).normalized(),
            face: f,
        });
    }
    let sides = [sides[0], sides[1]];
    let cross = sides[0].normal.cross(sides[1].normal).abs();
    if cross < 1e-6 {
        return Err(Error::InvalidInput(
            "this edge is smooth: its two faces are already tangent, so there is no corner to blend".into(),
        ));
    }
    // Convex if going into one face takes you below the other.
    let convex = sides[1].into.dot(sides[0].normal) < 0.0;
    Ok(Corner {
        p: section.project(p3),
        sides,
        convex,
    })
}

/// The swept profile for a corner, and the points used to judge its reach.
fn profile_of(corner: &Corner, kind: Kind) -> Result<(Profile, Vec<Vec2>)> {
    let p = corner.p;
    let [a, b] = corner.sides;
    match kind {
        Kind::Fillet(r) => {
            let (c, t1, t2) = fillet_circle(p, &a, &b, r, corner.convex)?;
            let arc_ccw = (t1 - c).cross(t2 - c) > 0.0;
            let mut path = PathBuilder::new(p);
            path = match a.trace {
                Trace::Line => path.line_to(t1),
                Trace::Arc { centre, .. } => path.arc_to(t1, centre, (p - centre).cross(t1 - centre) > 0.0),
            };
            path = path.arc_to(t2, c, arc_ccw);
            path = match b.trace {
                Trace::Line => path.line_to(p),
                Trace::Arc { centre, .. } => path.arc_to(p, centre, (t2 - centre).cross(p - centre) > 0.0),
            };
            let mid = c + (((t1 - c) + (t2 - c)).normalized()) * r;
            Ok((Profile::new(path.close()), vec![p, t1, t2, mid]))
        }
        Kind::Chamfer(d) => {
            let walk = |side: &Side| -> Vec2 {
                match side.trace {
                    Trace::Line => p + side.into * d,
                    Trace::Arc { centre, radius } => {
                        let angle = d / radius;
                        let u = p - centre;
                        // Turn the way `into` points.
                        let sign = if u.cross(side.into) > 0.0 { 1.0 } else { -1.0 };
                        let (s, co) = (angle * sign).sin_cos();
                        centre + v2(u.x * co - u.y * s, u.x * s + u.y * co)
                    }
                }
            };
            let (t1, t2) = (walk(&a), walk(&b));
            let mut path = PathBuilder::new(p);
            path = match a.trace {
                Trace::Line => path.line_to(t1),
                Trace::Arc { centre, .. } => path.arc_to(t1, centre, (p - centre).cross(t1 - centre) > 0.0),
            };
            path = path.line_to(t2);
            path = match b.trace {
                Trace::Line => path.line_to(p),
                Trace::Arc { centre, .. } => path.arc_to(p, centre, (t2 - centre).cross(p - centre) > 0.0),
            };
            Ok((Profile::new(path.close()), vec![p, t1, t2]))
        }
    }
}

/// The circle of radius `r` tangent to both traces: the ball rolling along the edge.
/// It sits inside the material on a convex corner, and outside on a concave one.
fn fillet_circle(p: Vec2, a: &Side, b: &Side, r: f64, convex: bool) -> Result<(Vec2, Vec2, Vec2)> {
    let side = if convex { -1.0 } else { 1.0 };
    // Each trace's offset curve, as a line or a circle.
    enum Off {
        Line { q: Vec2, d: Vec2 },
        Circle { c: Vec2, r: f64 },
    }
    let offset = |s: &Side| -> Option<Off> {
        match s.trace {
            Trace::Line => Some(Off::Line {
                q: p + s.normal * (side * r),
                d: s.into,
            }),
            Trace::Arc { centre, radius } => {
                // The normal points away from the arc's centre on a convex surface.
                let bulging = s.normal.dot(p - centre) > 0.0;
                let new = if bulging { radius + side * r } else { radius - side * r };
                (new > TOL).then_some(Off::Circle { c: centre, r: new })
            }
        }
    };
    let too_big = || Error::InvalidInput(format!("a fillet of radius {r} does not fit this corner"));
    let (oa, ob) = (offset(a).ok_or_else(too_big)?, offset(b).ok_or_else(too_big)?);
    let line_circle = |q: Vec2, d: Vec2, c: Vec2, rr: f64| -> Vec<Vec2> {
        let foot = q + d * (c - q).dot(d);
        let dist = foot.dist(c);
        if dist > rr + TOL {
            return Vec::new();
        }
        let h = (rr * rr - dist * dist).max(0.0).sqrt();
        vec![foot + d * h, foot - d * h]
    };
    let solutions: Vec<Vec2> = match (&oa, &ob) {
        (Off::Line { q, d }, Off::Line { q: q2, d: d2 }) => {
            let det = d.cross(*d2);
            if det.abs() < 1e-12 {
                Vec::new()
            } else {
                vec![*q + *d * ((*q2 - *q).cross(*d2) / det)]
            }
        }
        (Off::Line { q, d }, Off::Circle { c, r: rr }) | (Off::Circle { c, r: rr }, Off::Line { q, d }) => {
            line_circle(*q, *d, *c, *rr)
        }
        (Off::Circle { c, r: rr }, Off::Circle { c: c2, r: rs }) => {
            let dist = c.dist(*c2);
            if dist > rr + rs + TOL || dist < (rr - rs).abs() - TOL || dist < 1e-12 {
                Vec::new()
            } else {
                let u = (*c2 - *c) / dist;
                let x = (dist * dist + rr * rr - rs * rs) / (2.0 * dist);
                let h = (rr * rr - x * x).max(0.0).sqrt();
                let w = v2(-u.y, u.x);
                vec![*c + u * x + w * h, *c + u * x - w * h]
            }
        }
    };
    let contact = |s: &Side, c: Vec2| -> Vec2 {
        match s.trace {
            Trace::Line => p + s.into * (c - p).dot(s.into),
            Trace::Arc { centre, radius } => centre + (c - centre).normalized() * radius,
        }
    };
    // The right solution is near the corner, with both contacts ahead along the faces.
    let mut best: Option<(f64, Vec2, Vec2, Vec2)> = None;
    for c in solutions {
        let (t1, t2) = (contact(a, c), contact(b, c));
        let ahead = (t1 - p).dot(a.into) > 0.0 && (t2 - p).dot(b.into) > 0.0;
        let score = c.dist(p);
        if ahead && best.is_none_or(|x| score < x.0) {
            best = Some((score, c, t1, t2));
        }
    }
    let (_, c, t1, t2) = best.ok_or_else(too_big)?;
    Ok((c, t1, t2))
}

/// Region centre for probing what lies past the end of an edge.
fn centroid(points: &[Vec2]) -> Vec2 {
    let mut c = Vec2::default();
    for &q in points {
        c += q;
    }
    c / points.len() as f64
}

/// Decides how a tool should finish at vertex `v`, going in direction `dir`.
#[allow(clippy::too_many_arguments)]
fn end_condition(
    body: &Body,
    index: &Index,
    run: &Run,
    runs: &[Run],
    section: &Section,
    points: &[Vec2],
    convex: bool,
    v: usize,
    dir: Vec3,
    station: f64,
) -> Result<End> {
    let reach_section = points
        .iter()
        .flat_map(|a| points.iter().map(move |b| a.dist(*b)))
        .fold(0.0, f64::max);
    let margin = (reach_section * 0.05).max(1e-3);

    // Continuing smoothly into another blended edge: both tools meet at this section.
    let vertex = body.vertices[v];
    for other in runs {
        if std::ptr::eq(other, run) || (other.v0 != v && other.v1 != v) {
            continue;
        }
        let (t, sign) = if other.v0 == v { (other.t0, 1.0) } else { (other.t1, -1.0) };
        let away = other.curve.derivative(t).normalized() * sign;
        if away.dot(dir) > 1.0 - 1e-9 {
            return Ok(End::Flush);
        }
    }

    // What lies just past the end of the edge, in the middle of the section?
    let c = centroid(points);
    let step = (reach_section * 0.02).max(1e-3);
    let probe = if section.circular {
        let radius = c.x.max(1e-6);
        let sense = if (station - run.t0).abs() < (station - run.t1).abs() { -1.0 } else { 1.0 };
        section.lift(c, station + sense * step / radius)
    } else {
        section.lift(c, station) + dir * step
    };
    let harmless = match index.locate_in_body(probe) {
        BodyLoc::Inside => !convex,
        BodyLoc::Outside => convex,
        BodyLoc::OnFace(_) => false,
    };

    // The faces that bound the edge's end, other than the two it runs between.
    let mut blockers: Vec<usize> = Vec::new();
    for (f, face) in body.faces.iter().enumerate() {
        if run.faces.contains(&f) {
            continue;
        }
        let touches = face
            .loops
            .iter()
            .flatten()
            .any(|c| body.start_vertex(*c) == v);
        if touches {
            blockers.push(f);
        }
    }
    let size = index.bounds.diagonal();
    blockers.dedup_by(|a, b| coincident(&body.faces[*a].surface, &body.faces[*b].surface, size).is_some());

    let plane_of = |f: usize| match body.faces[f].surface {
        Surface::Plane(frame) => Some(frame),
        _ => None,
    };
    if harmless {
        // Far enough to clear the end face across the whole section.
        let mut ext = reach_section * 1.5;
        if let [only] = blockers.as_slice() {
            if let Some(plane) = plane_of(*only) {
                let denom = dir.dot(plane.z);
                if denom.abs() > 1e-6 {
                    ext = points
                        .iter()
                        .map(|&q| (vertex - section.lift(q, station)).dot(plane.z) / denom)
                        .fold(0.0, f64::max);
                }
            }
        }
        return Ok(End::Extend(ext + margin));
    }
    if let [only] = blockers.as_slice() {
        if let Some(plane) = plane_of(*only) {
            if plane.z.cross(dir).norm() > 1e-6 {
                // An oblique wall: the tool has to end along it, not square across.
                let inward = section.lift(c, station) - dir * step;
                return Ok(End::Clip {
                    plane,
                    keep: inward,
                    reach: reach_section * 3.0 + margin,
                });
            }
        }
    }
    Ok(End::Flush)
}

/// Large box covering one side of a plane, around a region.
fn half_space(plane: &Frame, keep: Vec3, centre: Vec3, reach: f64, feature: u32) -> Result<Body> {
    let up = if (keep - plane.origin).dot(plane.z) >= 0.0 { plane.z } else { -plane.z };
    let f = Frame::new(plane.origin + plane.z * (centre - plane.origin).dot(plane.z) * 0.0, up, plane.x);
    let c = f.to_local(centre);
    let origin = f.to_world(v3(c.x - reach, c.y - reach, 0.0));
    box_solid(&Frame { origin, ..f }, 2.0 * reach, 2.0 * reach, reach, feature)
}

/// Sweeps a profile along a run to make the tool solid.
fn sweep(profile: &Profile, run: &Run, section: &Section, ends: [End; 2], feature: u32) -> Result<Body> {
    let extension = |e: &End| match e {
        End::Flush => 0.0,
        End::Extend(x) => *x,
        End::Clip { reach, .. } => *reach,
    };
    let (e0, e1) = (extension(&ends[0]), extension(&ends[1]));
    let mut tool = if section.circular {
        let f = section.frame;
        if run.closed {
            revolve(profile, &f, TAU, feature)?
        } else {
            // Convert distance past each end into angle at the section's outer radius.
            let radius = match run.curve {
                Curve::Circle { radius, .. } => radius,
                _ => 1.0,
            };
            let (a0, a1) = (e0 / radius, e1 / radius);
            let start = run.t0 - a0;
            let sweep_angle = ((run.t1 + a1) - start).min(TAU - 1e-3);
            let (s, c) = start.sin_cos();
            let x = f.x * c + f.y * s;
            let frame = Frame {
                origin: f.origin,
                x,
                y: f.z.cross(x),
                z: f.z,
            };
            revolve(profile, &frame, sweep_angle, feature)?
        }
    } else {
        let length = run.t1 - run.t0;
        let f = section.frame;
        let frame = Frame {
            origin: f.origin - f.z * e0,
            ..f
        };
        extrude(profile, &frame, length + e0 + e1, feature)?
    };
    for end in &ends {
        if let End::Clip { plane, keep, reach } = end {
            let centre = *keep;
            let keep_side = half_space(plane, *keep, centre, reach * 4.0, feature)?;
            tool = intersect(&tool, &keep_side)?;
        }
    }
    Ok(tool)
}

fn blend(body: &Body, edges: &[usize], kind: Kind, feature: u32) -> Result<Body> {
    if edges.is_empty() {
        return Ok(body.clone());
    }
    let index = Index::new(body);
    let size = index.bounds.diagonal();
    let runs = build_runs(body, edges)?;

    let mut cuts = Vec::new();
    let mut adds = Vec::new();
    let mut corners: Vec<(usize, Vec<Vec2>, Corner, Section)> = Vec::new();
    for (i, run) in runs.iter().enumerate() {
        let section = section_of(run, size)?;
        let corner = corner_of(body, run, &section, size)?;
        let (profile, points) = profile_of(&corner, kind)?;
        // Both contacts must land on their faces, or the blend overruns them.
        for (s, &q) in corner.sides.iter().zip(&points[1..3]) {
            let p3 = section.lift(q, section.station);
            if index.locate_on_face(s.face, p3, 2.0 * TOL) == FaceLoc::Outside {
                return Err(Error::InvalidInput(format!(
                    "a {} of this size runs off the neighbouring {}",
                    match kind {
                        Kind::Fillet(_) => "fillet",
                        Kind::Chamfer(_) => "chamfer",
                    },
                    body.faces[s.face].surface.name()
                )));
            }
        }
        if section.circular && points.iter().any(|q| q.x < TOL) {
            return Err(Error::InvalidInput("the blend reaches the axis of this circular edge".into()));
        }
        let ends = if run.closed {
            [End::Flush, End::Flush]
        } else {
            let (d0, d1, s0, s1) = if section.circular {
                (
                    -run.curve.derivative(run.t0).normalized(),
                    run.curve.derivative(run.t1).normalized(),
                    run.t0,
                    run.t1,
                )
            } else {
                let d = section.frame.z;
                (-d, d, 0.0, run.t1 - run.t0)
            };
            [
                end_condition(body, &index, run, &runs, &section, &points, corner.convex, run.v0, d0, s0)?,
                end_condition(body, &index, run, &runs, &section, &points, corner.convex, run.v1, d1, s1)?,
            ]
        };
        let tool = sweep(&profile, run, &section, ends, feature)?;
        if corner.convex {
            cuts.push(tool);
        } else {
            adds.push(tool);
        }
        corners.push((i, points, corner, section));
    }

    // Corners where three blended convex edges meet get a sphere, the shape a rolling
    // ball actually makes. The tools alone would leave a sharp peak there.
    if let Kind::Fillet(r) = kind {
        for v in 0..body.vertices.len() {
            if let Some(tool) = sphere_corner(body, &runs, &corners, v, r, feature)? {
                cuts.push(tool);
            }
        }
    }

    let mut out = body.clone();
    for t in &cuts {
        out = subtract(&out, t)?;
    }
    for t in &adds {
        out = union(&out, t)?;
    }
    Ok(out)
}

/// The piece of corner outside the rolling ball, where three convex blended straight
/// edges between planes meet at a vertex.
fn sphere_corner(
    body: &Body,
    runs: &[Run],
    corners: &[(usize, Vec<Vec2>, Corner, Section)],
    v: usize,
    r: f64,
    feature: u32,
) -> Result<Option<Body>> {
    let here: Vec<usize> = runs
        .iter()
        .enumerate()
        .filter(|(_, run)| !run.closed && (run.v0 == v || run.v1 == v))
        .map(|(i, _)| i)
        .collect();
    let [a, b, c] = here.as_slice() else {
        return Ok(None);
    };
    let mut planes: Vec<Frame> = Vec::new();
    for &i in &[*a, *b, *c] {
        let (_, _, corner, section) = &corners[i];
        if !corner.convex || section.circular {
            return Ok(None);
        }
        for s in &corner.sides {
            let Surface::Plane(mut frame) = body.faces[s.face].surface else {
                return Ok(None);
            };
            if body.faces[s.face].flipped {
                frame.z = -frame.z;
                frame.y = -frame.y;
            }
            if !planes.iter().any(|p| p.z.dot(frame.z) > 1.0 - 1e-9) {
                planes.push(frame);
            }
        }
    }
    let [p1, p2, p3] = planes.as_slice() else {
        return Ok(None);
    };
    // The ball's centre is `r` under each of the three planes.
    let (n1, n2, n3) = (p1.z, p2.z, p3.z);
    let det = n1.dot(n2.cross(n3));
    if det.abs() < 1e-9 {
        return Ok(None);
    }
    let (d1, d2, d3) = (n1.dot(p1.origin) - r, n2.dot(p2.origin) - r, n3.dot(p3.origin) - r);
    let centre = (n2.cross(n3) * d1 + n3.cross(n1) * d2 + n1.cross(n2) * d3) / det;
    let corner_point = body.vertices[v];

    // A block between the three face planes and the three planes through the centre
    // perpendicular to each edge. Outside the ball within it is what must go.
    let mut block: Option<Body> = None;
    let span = corner_point.dist(centre) * 2.0 + r;
    let mut clip = Vec::new();
    for p in [p1, p2, p3] {
        clip.push((*p, centre));
    }
    for &i in &[*a, *b, *c] {
        let run = &runs[i];
        let Curve::Line { dir, .. } = run.curve else {
            return Ok(None);
        };
        // Keep the side of the plane through the centre that holds the vertex.
        let toward = (corner_point - centre).dot(dir);
        let normal = if toward >= 0.0 { dir } else { -dir };
        let plane = Frame::new(centre, normal, normal.any_perpendicular());
        clip.push((plane, corner_point));
    }
    for (plane, keep) in clip {
        let piece = half_space(&plane, keep, centre, span, feature)?;
        block = Some(match block {
            None => piece,
            Some(b) => intersect(&b, &piece)?,
        });
    }
    let block = block.unwrap();
    let ball = sphere(&Frame::new(centre, (corner_point - centre).normalized(), Vec3::X), r, feature)?;
    Ok(Some(subtract(&block, &ball)?))
}

// ----- Picking edges -----

impl Body {
    /// Indices of edges that satisfy a predicate.
    pub fn edges_where(&self, mut pred: impl FnMut(usize, &Edge) -> bool) -> Vec<usize> {
        self.edges
            .iter()
            .enumerate()
            .filter(|(i, e)| pred(*i, e))
            .map(|(i, _)| i)
            .collect()
    }

    /// Whether the edge is a straight line parallel to a direction.
    pub fn edge_is_parallel(&self, e: usize, dir: Vec3) -> bool {
        matches!(self.edges[e].curve, Curve::Line { dir: d, .. } if d.cross(dir).norm() < 1e-9)
    }

    /// Whether the edge is a circular arc of this radius, within tolerance.
    pub fn edge_is_arc_of_radius(&self, e: usize, radius: f64) -> bool {
        matches!(self.edges[e].curve, Curve::Circle { radius: r, .. } if (r - radius).abs() <= TOL)
    }

    /// Midpoint of an edge.
    pub fn edge_midpoint(&self, e: usize) -> Vec3 {
        self.edges[e].eval_mid()
    }

    /// Edges lying between faces with these two tags.
    pub fn edges_between_tags(&self, a: crate::topo::FaceTag, b: crate::topo::FaceTag) -> Vec<usize> {
        let uses = self.edge_uses();
        let mut out = Vec::new();
        for (e, list) in uses.iter().enumerate() {
            if let [x, y] = list.as_slice() {
                let (tx, ty) = (self.faces[x.0].tag, self.faces[y.0].tag);
                if (tx == a && ty == b) || (tx == b && ty == a) {
                    out.push(e);
                }
            }
        }
        out
    }
}

