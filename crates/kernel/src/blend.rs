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
use crate::boolean::{boolean_with, BoolOp, Checks};
use crate::multi::boolean_many_with;
use crate::build::{box_solid, extrude, revolve, sphere};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{PathBuilder, Profile};
use crate::surface::Surface;
use crate::topo::{Body, Edge, FaceOrigin, FaceTag};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Kind {
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

/// How a blend treats the edges it was given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlendOptions {
    /// Follow tangent neighbours of the selected edges. Without this, a blend on one
    /// edge of a smooth chain stops flush at the end of that edge and leaves a step.
    pub propagate: bool,
}

/// Rounds the given edges to a radius, following them around tangent corners.
pub fn fillet(body: &Body, edges: &[usize], radius: f64, feature: u32) -> Result<Body> {
    fillet_with(body, edges, radius, BlendOptions { propagate: true }, feature)
}

/// Rounds edges to a radius, with control over tangent propagation.
pub fn fillet_with(body: &Body, edges: &[usize], radius: f64, options: BlendOptions, feature: u32) -> Result<Body> {
    body.needs_solid("a fillet")?;
    if !(radius.is_finite() && radius >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("fillet radius {radius} is too small")));
    }
    let selected = if options.propagate { propagate_tangents(body, edges)? } else { edges.to_vec() };
    // A corner where convex edges and concave ones meet has no ball that touches them all. It does once the odd one out
    // is done first, as the others then run smoothly around it.
    let first = odd_ones_at_corners(body, &selected, radius);
    if !first.is_empty() && first.len() < selected.len() {
        let rest: Vec<usize> = selected.iter().copied().filter(|e| !first.contains(e)).collect();
        let done = fillet_route(body, &first, &first, radius, feature)?;
        let left = same_edges_in(body, &rest, &done);
        if left.is_empty() {
            return Ok(done);
        }
        // They now run around what was just done, whether or not this was asked to follow tangents.
        let chain = propagate_tangents(&done, &left)?;
        return fillet_route(&done, &chain, &left, radius, feature);
    }
    fillet_route(body, &selected, edges, radius, feature)
}

/// An exact surface where there is one. Where there isn't, a fitted one, held to a stated tolerance.
pub(crate) fn fillet_route(body: &Body, selected: &[usize], asked: &[usize], radius: f64, feature: u32) -> Result<Body> {
    match blend(body, selected, Kind::Fillet(radius), feature) {
        Err(Error::Unsupported(_)) => crate::rolling::blend(body, asked, Kind::Fillet(radius), feature),
        other => other,
    }
}

/// Whether an edge is convex, with the material inside the angle its faces make.
fn edge_is_convex(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], e: usize) -> Option<bool> {
    let [a, b] = uses[e].as_slice() else {
        return None;
    };
    let edge = &body.edges[e];
    let p = edge.eval_mid();
    let t = edge.curve.derivative((edge.t0 + edge.t1) / 2.0).normalized();
    let (na, nb) = (body.faces[a.0].outward_normal(p), body.faces[b.0].outward_normal(p));
    if na.cross(nb).norm() < 1e-6 {
        return None;
    }
    // Going into the second face takes you below the first, if it's convex.
    let along = if b.3 { -t } else { t };
    Some(nb.cross(along).dot(na) < 0.0)
}

/// At each vertex where three selected edges meet that aren't all convex or all concave, the one that's different. The same
/// goes for one too short to be a wall for the others, as on a thin plate: rounded first, the others run around it.
pub(crate) fn odd_ones_at_corners(body: &Body, selected: &[usize], radius: f64) -> Vec<usize> {
    let uses = body.edge_uses();
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for &e in selected {
        if e < body.edges.len() {
            at_vertex[body.edges[e].v0].push(e);
            at_vertex[body.edges[e].v1].push(e);
        }
    }
    let mut odd = Vec::new();
    for list in &at_vertex {
        let [x, y, z] = list.as_slice() else {
            continue;
        };
        let (Some(a), Some(b), Some(c)) = (edge_is_convex(body, &uses, *x), edge_is_convex(body, &uses, *y), edge_is_convex(body, &uses, *z)) else {
            continue;
        };
        let length = |e: usize| -> f64 {
            let edge = &body.edges[e];
            edge.curve.eval(edge.t0).dist(edge.curve.eval(edge.t1))
        };
        let shortest = [*x, *y, *z].into_iter().min_by(|p, q| length(*p).total_cmp(&length(*q))).unwrap_or(*x);
        let pick = if a == b && b != c {
            *z
        } else if a == c && b != c {
            *y
        } else if b == c && a != b {
            *x
        } else if length(shortest) < radius * (1.0 - 1e-9) {
            shortest
        } else {
            continue;
        };
        if !odd.contains(&pick) {
            odd.push(pick);
        }
    }
    odd
}

/// Where edges of one body have got to in another made from it: whatever lies along them, between the same faces.
pub(crate) fn same_edges_in(old: &Body, edges: &[usize], new: &Body) -> Vec<usize> {
    let (old_uses, new_uses) = (old.edge_uses(), new.edge_uses());
    let tags = |b: &Body, list: &[(usize, usize, usize, bool)]| -> Vec<FaceTag> {
        let mut out: Vec<FaceTag> = list.iter().map(|u| b.faces[u.0].tag).collect();
        out.sort_unstable();
        out
    };
    let mut found = Vec::new();
    for (g, list) in new_uses.iter().enumerate() {
        let edge = &new.edges[g];
        let m = edge.eval_mid();
        for &e in edges {
            let o = &old.edges[e];
            if o.curve.closest(m, o.t0, o.t1).1 < 1e-7 && tags(new, list) == tags(old, &old_uses[e]) {
                found.push(g);
                break;
            }
        }
    }
    found
}

/// Adds every edge reachable from the selection through tangent continuation. An edge
/// that is itself smooth, or that can't be blended, ends the chain.
pub(crate) fn propagate_tangents(body: &Body, selected: &[usize]) -> Result<Vec<usize>> {
    let uses = body.edge_uses();
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for (i, e) in body.edges.iter().enumerate() {
        at_vertex[e.v0].push(i);
        at_vertex[e.v1].push(i);
    }
    let is_corner = |e: usize| -> bool {
        let edge = &body.edges[e];
        let [a, b] = uses[e].as_slice() else {
            return false;
        };
        let mid = edge.eval_mid();
        let (fa, fb) = (&body.faces[a.0], &body.faces[b.0]);
        fa.outward_normal(mid).cross(fb.outward_normal(mid)).norm() > 1e-6
    };
    // Unit direction leaving vertex `v` along edge `e`.
    let leaving = |e: usize, v: usize| -> Vec3 {
        let edge = &body.edges[e];
        if edge.v0 == v {
            edge.curve.derivative(edge.t0).normalized()
        } else {
            -edge.curve.derivative(edge.t1).normalized()
        }
    };
    let mut seen = vec![false; body.edges.len()];
    let mut stack = Vec::new();
    for &e in selected {
        if e >= body.edges.len() {
            return Err(Error::InvalidInput(format!("there is no edge {e}")));
        }
        if !seen[e] {
            seen[e] = true;
            stack.push(e);
        }
    }
    let mut out = selected.to_vec();
    while let Some(e) = stack.pop() {
        let edge = &body.edges[e];
        for v in [edge.v0, edge.v1] {
            let out_dir = -leaving(e, v);
            for &g in &at_vertex[v] {
                // Continuing means leaving the vertex in the direction we arrived.
                if !seen[g] && leaving(g, v).dot(out_dir) > 1.0 - 1e-9 && is_corner(g) {
                    seen[g] = true;
                    out.push(g);
                    stack.push(g);
                }
            }
        }
    }
    Ok(out)
}

/// Bevels the given edges, cutting back by a distance along each face, following them around
/// tangent corners.
pub fn chamfer(body: &Body, edges: &[usize], distance: f64, feature: u32) -> Result<Body> {
    chamfer_with(body, edges, distance, BlendOptions { propagate: true }, feature)
}

/// Bevels edges, with control over tangent propagation.
pub fn chamfer_with(body: &Body, edges: &[usize], distance: f64, options: BlendOptions, feature: u32) -> Result<Body> {
    body.needs_solid("a chamfer")?;
    if !(distance.is_finite() && distance >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("chamfer distance {distance} is too small")));
    }
    let selected = if options.propagate { propagate_tangents(body, edges)? } else { edges.to_vec() };
    match blend(body, &selected, Kind::Chamfer(distance), feature) {
        Err(Error::Unsupported(_)) => crate::rolling::blend(body, edges, Kind::Chamfer(distance), feature),
        other => other,
    }
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
            return Err(Error::Unsupported("fillets and chamfers work on straight and circular edges only".into()));
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
                let next = at_vertex[v].iter().copied().find(|&f| !done[f] && continues(start, f));
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
        runs.push(Run { curve: e.curve.clone(), t0, t1, v0, v1, closed, faces: faces_of(start)?, members });
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
            Ok(Section { frame, circular: false, station: (run.t1 - run.t0) / 2.0 })
        }
        Curve::Circle { frame, .. } => {
            let _ = size;
            Ok(Section { frame: *frame, circular: true, station: (run.t0 + run.t1) / 2.0 })
        }
        _ => Err(Error::Unsupported("only straight and circular edges".into())),
    }
}

/// How a face appears in the cross-section, or an error if it can't be swept.
fn trace_of(surface: &Surface, section: &Section, size: f64) -> Result<Trace> {
    let f = &section.frame;
    let unsupported =
        || Error::Unsupported(format!("this edge lies on a {} that doesn't sweep along the edge", surface.name()));
    let parallel = |a: Vec3, b: Vec3| a.cross(b).norm() * size.max(1.0) <= 1e-5;
    let on_axis = |q: Vec3| {
        let d = q - f.origin;
        (d - f.z * d.dot(f.z)).norm() <= 1e-5
    };
    if matches!(surface, Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. }) {
        return Err(Error::Unsupported("fillets and chamfers on spline faces aren't available yet: their exact shape isn't a spline, and this kernel gives an exact answer or none".into()));
    }
    if section.circular {
        match *surface {
            Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. } => unreachable!(),
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
                    Ok(Trace::Arc { centre: section.project(frame.origin), radius })
                } else {
                    Err(unsupported())
                }
            }
            Surface::Torus { frame, major, minor } => {
                if parallel(frame.z, f.z) && on_axis(frame.origin) {
                    Ok(Trace::Arc { centre: v2(major, (frame.origin - f.origin).dot(f.z)), radius: minor })
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
                    Ok(Trace::Arc { centre: section.project(frame.origin), radius })
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
    /// One side's wall is too short for the ball, which takes this blend through its far edge instead.
    through_edge: bool,
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
    Ok(Corner { p: section.project(p3), sides, convex, through_edge: false })
}

/// The swept profile for a corner, and the points used to judge its reach.
fn profile_of(corner: &Corner, kind: Kind, circular: bool, far: [Option<Vec2>; 2]) -> Result<(Profile, Vec<Vec2>)> {
    let p = corner.p;
    let [a, b] = corner.sides;
    match kind {
        Kind::Fillet(r) => {
            let (c, t1, t2) = fillet_circle(p, &a, &b, r, corner.convex)?;
            let (c, t1, t2) = match far {
                [None, Some(e)] => {
                    let (c, t) = circle_through(p, &a, e, r, corner.convex, c)?;
                    (c, t, e)
                }
                [Some(e), None] => {
                    let (c, t) = circle_through(p, &b, e, r, corner.convex, c)?;
                    (c, e, t)
                }
                _ => (c, t1, t2),
            };
            let arc_ccw = (t1 - c).cross(t2 - c) > 0.0;
            if circular && c.x < -TOL {
                if far != [None, None] {
                    return Err(Error::InvalidInput(format!("a fillet of radius {r} going through the far edge of its wall crosses this edge's axis")));
                }
                return clipped_at_axis(p, &a, &b, c, r, t1, t2);
            }
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
            let (t1, t2) = (far[0].unwrap_or_else(|| walk(&a)), far[1].unwrap_or_else(|| walk(&b)));
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

/// The section of a circular blend whose radius exceeds the rim's. The rolling ball's
/// centre lies across the axis, and a revolved section can't cross its axis, so the section
/// stops there: from the corner along one trace to the axis, up the axis to where the arc
/// reaches it, then along the arc to the other contact. The arc sweeps out a lemon.
fn clipped_at_axis(p: Vec2, a: &Side, b: &Side, c: Vec2, r: f64, t1: Vec2, t2: Vec2) -> Result<(Profile, Vec<Vec2>)> {
    // Which trace runs through the axis? It must be straight to be swept this way.
    let (first, far, near) = if t1.x < t2.x { (a, t1, t2) } else { (b, t2, t1) };
    if !matches!(first.trace, Trace::Line) {
        return Err(Error::Unsupported(
            "a blend larger than its rim is only supported where one face is flat across the rim".into(),
        ));
    }
    if first.into.x >= -1e-12 {
        return Err(Error::InvalidInput(format!("a fillet of radius {r} does not fit this corner")));
    }
    let on_trace = p + first.into * (-p.x / first.into.x);
    let on_trace = v2(0.0, on_trace.y);
    // Where the arc meets the axis, on the side the arc actually runs.
    let h = (r * r - c.x * c.x).sqrt();
    let candidates = [v2(0.0, c.y + h), v2(0.0, c.y - h)];
    let toward = (far - c) + (near - c);
    let on_arc =
        if (candidates[0] - c).dot(toward) >= (candidates[1] - c).dot(toward) { candidates[0] } else { candidates[1] };
    let other = if std::ptr::eq(first, a) { b } else { a };
    let arc_ccw = (on_arc - c).cross(near - c) > 0.0;
    let mut path = PathBuilder::new(p).line_to(on_trace);
    if on_trace.dist(on_arc) > MIN_FEATURE {
        path = path.line_to(on_arc);
    }
    path = path.arc_to(near, c, arc_ccw);
    path = match other.trace {
        Trace::Line => path.line_to(p),
        Trace::Arc { centre, .. } => path.arc_to(p, centre, (near - centre).cross(p - centre) > 0.0),
    };
    // Contacts in the same order as the corner's sides.
    let contacts = if std::ptr::eq(first, a) { [on_trace, near] } else { [near, on_trace] };
    Ok((Profile::new(path.close()), vec![p, contacts[0], contacts[1], on_arc]))
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
            Trace::Line => Some(Off::Line { q: p + s.normal * (side * r), d: s.into }),
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

/// The circle of radius `r` tangent to one trace and through a point, for a wall too short for the ball to touch: the ball
/// rides on the wall's far edge instead. Of the two, it's the one nearer where the ball would have been. Gives its center,
/// and where it touches.
fn circle_through(p: Vec2, s: &Side, e: Vec2, r: f64, convex: bool, usual: Vec2) -> Result<(Vec2, Vec2)> {
    let side = if convex { -1.0 } else { 1.0 };
    let too_big = || Error::InvalidInput(format!("a fillet of radius {r} doesn't fit between its face and the far edge of its wall"));
    let centers: Vec<Vec2> = match s.trace {
        Trace::Line => {
            let (q, d) = (p + s.normal * (side * r), s.into);
            let foot = q + d * (e - q).dot(d);
            let off = foot.dist(e);
            if off > r + TOL {
                return Err(too_big());
            }
            let h = (r * r - off * off).max(0.0).sqrt();
            vec![foot + d * h, foot - d * h]
        }
        Trace::Arc { centre, radius } => {
            let bulging = s.normal.dot(p - centre) > 0.0;
            let around = if bulging { radius + side * r } else { radius - side * r };
            let dist = centre.dist(e);
            if around <= TOL || dist < 1e-12 || dist > around + r + TOL || dist < (around - r).abs() - TOL {
                return Err(too_big());
            }
            let u = (e - centre) / dist;
            let x = (dist * dist + around * around - r * r) / (2.0 * dist);
            let h = (around * around - x * x).max(0.0).sqrt();
            let w = v2(-u.y, u.x);
            vec![centre + u * x + w * h, centre + u * x - w * h]
        }
    };
    let touching = |c: Vec2| -> Vec2 {
        match s.trace {
            Trace::Line => p + s.into * (c - p).dot(s.into),
            Trace::Arc { centre, radius } => centre + (c - centre).normalized() * radius,
        }
    };
    let c = centers.into_iter().filter(|c| (touching(*c) - p).dot(s.into) > 0.0).min_by(|x, y| x.dist(usual).total_cmp(&y.dist(usual))).ok_or_else(too_big)?;
    Ok((c, touching(c)))
}

/// Where along a run to look: between its ends, since those are shared with whatever is there.
fn stations_of(run: &Run, section: &Section) -> Vec<f64> {
    let (from, to) = if section.circular { (run.t0, run.t1) } else { (0.0, run.t1 - run.t0) };
    [0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.99].iter().map(|k| from + (to - from) * k).collect()
}

/// The stretch of a run where a contact lands on its face, if it runs off only near the ends, found to well within what
/// matters. Gives where it starts and stops, in the same terms as the stations.
fn on_face_span(stations: &[f64], clear: f64, landed: impl Fn(f64) -> bool) -> Option<(f64, f64)> {
    let (lo, hi) = (*stations.first()?, *stations.last()?);
    let first = stations.iter().position(|st| landed(*st))?;
    let last = stations.iter().rposition(|st| landed(*st))?;
    if first >= last || (first..=last).any(|i| !landed(stations[i])) {
        return None;
    }
    let settle = |mut off: f64, mut on: f64| -> f64 {
        for _ in 0..40 {
            let m = (off + on) / 2.0;
            if landed(m) {
                on = m;
            } else {
                off = m;
            }
        }
        on
    };
    let a = if first == 0 { lo } else { settle(stations[first - 1], stations[first]) + clear };
    let b = if last + 1 == stations.len() { hi } else { settle(stations[last + 1], stations[last]) - clear };
    (b > a).then_some((a, b))
}

/// How far along a side's trace from the corner a point is, if it's on the trace at all.
fn along_trace(s: &Side, p: Vec2, q: Vec2) -> Option<f64> {
    match s.trace {
        Trace::Line => {
            let d = q - p;
            (d.cross(s.into).abs() < 1e-6).then(|| d.dot(s.into))
        }
        Trace::Arc { centre, radius } => {
            if (q.dist(centre) - radius).abs() > 1e-6 {
                return None;
            }
            let (u, w) = (p - centre, q - centre);
            let sense = if u.cross(s.into) > 0.0 { 1.0 } else { -1.0 };
            let turn = (u.cross(w) * sense).atan2(u.dot(w));
            Some(turn * radius)
        }
    }
}

/// The faces along one side of a run: a rim's wall can be two halves.
fn faces_along(body: &Body, run: &Run, side: &Side, size: f64) -> Vec<usize> {
    let uses = body.edge_uses();
    let mut out = vec![side.face];
    for &m in &run.members {
        for u in &uses[m] {
            if !out.contains(&u.0) && coincident(&body.faces[u.0].surface, &body.faces[side.face].surface, size).is_some() {
                out.push(u.0);
            }
        }
    }
    out
}

/// The far edge of a side's faces, if they're too short for what's asked of them all along this run: the nearest edge
/// that's one point in section, on the trace before the contact, and that's there at every station.
#[allow(clippy::too_many_arguments)]
fn far_edge_of(body: &Body, run: &Run, section: &Section, s: &Side, faces: &[usize], p: Vec2, contact: Vec2, stations: &[f64]) -> Option<Vec2> {
    let limit = along_trace(s, p, contact)?;
    let mut found: Vec<(f64, Vec2)> = Vec::new();
    for c in faces.iter().flat_map(|f| body.faces[*f].loops.iter().flatten()) {
        if run.members.contains(&c.edge) {
            continue;
        }
        let e = &body.edges[c.edge];
        let q = section.project(e.curve.eval(e.t0));
        let one_point = [0.25, 0.5, 0.75, 1.0].iter().all(|k| section.project(e.curve.eval(e.t0 + (e.t1 - e.t0) * k)).dist(q) < 1e-7);
        if !one_point {
            continue;
        }
        let Some(d) = along_trace(s, p, q) else {
            continue;
        };
        if d > MIN_FEATURE && d < limit - TOL {
            found.push((d, q));
        }
    }
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    // All along, on that same line: a window in the wall has edges like this too, for part of the way.
    let there = |q: Vec2, st: f64| -> bool {
        let at = section.lift(q, st);
        faces.iter().flat_map(|f| body.faces[*f].loops.iter().flatten()).any(|c| {
            let e = &body.edges[c.edge];
            !run.members.contains(&c.edge) && e.curve.closest(at, e.t0, e.t1).1 < 1e-7
        })
    };
    found.into_iter().map(|x| x.1).find(|q| stations.iter().all(|st| there(*q, *st)))
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
    let reach_section = points.iter().flat_map(|a| points.iter().map(move |b| a.dist(*b))).fold(0.0, f64::max);
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
        let touches = face.loops.iter().flatten().any(|c| body.start_vertex(*c) == v);
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
                // An oblique wall: the tool has to end along it, not square across. Which side of it to keep is the side the
                // middle of the run is on. Right at the end, the section can already be on the other side of a slanted wall.
                let inward = section.lift(c, section.station);
                return Ok(End::Clip { plane, keep: inward, reach: reach_section * 3.0 + margin });
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
            let frame = Frame { origin: f.origin, x, y: f.z.cross(x), z: f.z };
            revolve(profile, &frame, sweep_angle, feature)?
        }
    } else {
        let length = run.t1 - run.t0;
        let f = section.frame;
        let frame = Frame { origin: f.origin - f.z * e0, ..f };
        extrude(profile, &frame, length + e0 + e1, feature)?
    };
    for end in &ends {
        if let End::Clip { plane, keep, reach } = end {
            let centre = *keep;
            // Big enough for all of the tool, however long its run.
            let keep_side = half_space(plane, *keep, centre, reach * 4.0 + tool.aabb().diagonal() * 2.0, feature)?;
            tool = boolean_with(&tool, &keep_side, BoolOp::Intersect, Checks::Interim)?;
        }
    }
    Ok(tool)
}

/// Records which faces a blend's tool stands in for, and which copy it's in. A neighbour merged across a
/// mirror plane has lost its copy step, so the one with the most specific steps says.
fn mark_source(tool: &mut Body, body: &Body, faces: &[usize]) {
    let mut between: Vec<FaceTag> = faces.iter().map(|&f| body.faces[f].tag).collect();
    between.sort_unstable();
    between.dedup();
    let copies = faces.iter().map(|&f| &body.faces[f].origin.copies).max().cloned().unwrap_or_default();
    for f in &mut tool.faces {
        f.origin = FaceOrigin { copies: copies.clone(), between: between.clone() };
    }
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
    // What each run's tool was made from, to make it again running further.
    let mut made: Vec<(Profile, [End; 2], f64)> = Vec::new();
    // For each run, at each of its stations, whether the ball has its faces to stand on there, so that the edge certainly
    // has to go. Where it runs off a face it may not, near the ends.
    let mut certain: Vec<Vec<bool>> = Vec::new();
    let name = match kind {
        Kind::Fillet(_) => "fillet",
        Kind::Chamfer(_) => "chamfer",
    };
    for (i, run) in runs.iter().enumerate() {
        let section = section_of(run, size)?;
        let mut corner = corner_of(body, run, &section, size)?;
        let (mut profile, mut points) = profile_of(&corner, kind, section.circular, [None, None])?;
        let stations = stations_of(run, &section);
        let along = [faces_along(body, run, &corner.sides[0], size), faces_along(body, run, &corner.sides[1], size)];
        let landed = |s: usize, q: Vec2, st: f64| along[s].iter().any(|f| index.locate_on_face(*f, section.lift(q, st), 2.0 * TOL) != FaceLoc::Outside);
        // A wall too short for the ball all along the run: it rides on the wall's far edge.
        let far_of = |s: usize, points: &[Vec2]| -> Option<Vec2> {
            let q = points[s + 1];
            if stations.iter().any(|st| landed(s, q, *st)) {
                return None;
            }
            far_edge_of(body, run, &section, &corner.sides[s], &along[s], corner.p, q, &stations)
        };
        let far = [far_of(0, &points), far_of(1, &points)];
        if far[0].is_some() && far[1].is_some() {
            return Err(Error::InvalidInput(format!("a {name} of this size doesn't fit: both of the faces beside this edge are too short for it")));
        }
        let through_edge = far != [None, None];
        if through_edge {
            (profile, points) = profile_of(&corner, kind, section.circular, far)?;
        }
        corner.through_edge = through_edge;
        if section.circular && points.iter().any(|q| q.x < -TOL) {
            return Err(Error::InvalidInput("the blend crosses the axis of this circular edge".into()));
        }
        let ends = if run.closed {
            [End::Flush, End::Flush]
        } else {
            let (d0, d1, s0, s1) = if section.circular {
                (-run.curve.derivative(run.t0).normalized(), run.curve.derivative(run.t1).normalized(), run.t0, run.t1)
            } else {
                let d = section.frame.z;
                (-d, d, 0.0, run.t1 - run.t0)
            };
            [
                end_condition(body, &index, run, &runs, &section, &points, corner.convex, run.v0, d0, s0)?,
                end_condition(body, &index, run, &runs, &section, &points, corner.convex, run.v1, d1, s1)?,
            ]
        };
        // Where a contact has nothing to land on, part of the way along, it's only wrong if the tool would then do damage:
        // cut into material beyond the wall, or stick out with nothing under it. Otherwise there's a hole there, say, and
        // the Boolean finishes the job.
        let mut hold_to: Vec<usize> = Vec::new();
        for s in 0..2 {
            if far[s].is_some() {
                continue;
            }
            let q = points[s + 1];
            for st in &stations {
                if landed(s, q, *st) {
                    continue;
                }
                let at = section.lift(q, *st);
                let n = body.faces[corner.sides[s].face].outward_normal(at);
                let damage = if corner.convex { index.locate_in_body(at) == BodyLoc::Inside } else { index.locate_in_body(at - n * (1e-3 * size.min(1.0))) == BodyLoc::Outside };
                if damage {
                    hold_to.push(s);
                    break;
                }
            }
        }
        // What it does past the end of a face is kept to what's over or under that face.
        let reach_section = points.iter().flat_map(|a| points.iter().map(move |b| a.dist(*b))).fold(0.0, f64::max);
        // The face's edge can be exactly where it can't be cut, as where a web runs tangent into a boss. If it's only near the
        // ends that it runs off, the blend can stop where it does, a little short of that, or it ends right on the face's edge,
        // which is just as hard to cut.
        let stopped_short = |sides: &[usize]| -> Option<Result<Body>> {
            let (mut from, mut to) = (f64::MIN, f64::MAX);
            for &s in sides {
                let q = points[s + 1];
                let clear = 0.02 * reach_section / if section.circular { q.x.max(1e-9) } else { 1.0 };
                let (a, b) = on_face_span(&stations, clear, |st| landed(s, q, st))?;
                from = from.max(a);
                to = to.min(b);
            }
            if !(to > from) {
                return None;
            }
            let (from, to) = if section.circular { (from, to) } else { (run.t0 + from, run.t0 + to) };
            let short = Run { t0: from, t1: to, ..run.clone() };
            Some(section_of(&short, size).and_then(|place| sweep(&profile, &short, &place, [End::Flush, End::Flush], feature)))
        };
        let swept = sweep(&profile, run, &section, ends, feature);
        let mut short = false;
        let mut tool = match swept {
            Ok(tool) => tool,
            Err(why) => {
                let off: Vec<usize> = (0..2).filter(|s| far[*s].is_none() && stations.iter().any(|st| !landed(*s, points[*s + 1], *st))).collect();
                if off.is_empty() {
                    return Err(why);
                }
                match stopped_short(&off) {
                    Some(Ok(tool)) => {
                        short = true;
                        tool
                    }
                    _ => return Err(why),
                }
            }
        };
        for s in hold_to {
            let thickness = 2.0 * reach_section * if corner.convex { -1.0 } else { 1.0 };
            let kept = crate::thicken::thicken(body, &along[s], thickness, feature).and_then(|over| boolean_with(&tool, &over, BoolOp::Intersect, Checks::Interim));
            tool = match kept {
                Ok(kept) => kept,
                Err(why) => match stopped_short(&[s]) {
                    Some(Ok(stopped)) => {
                        short = true;
                        stopped
                    }
                    _ => return Err(Error::InvalidInput(format!("a {name} of this size runs off the neighbouring {}, part of the way along, and can't be kept to it: {why}", body.faces[corner.sides[s].face].surface.name()))),
                },
            };
        }
        mark_source(&mut tool, body, &run.faces);
        if corner.convex {
            cuts.push(tool);
        } else {
            adds.push(tool);
        }
        certain.push(stations.iter().map(|st| !short && (0..2).all(|s| far[s].is_some() || landed(s, points[s + 1], *st))).collect());
        made.push((profile, ends, reach_section));
        corners.push((i, points, corner, section));
    }

    // Two blended edges that meet at an angle with their tools stopped short: in a reflex corner, say, where going on would
    // cut what the other doesn't. Where the tools would both have gone, they meet in a mitre.
    for v in 0..body.vertices.len() {
        let here: Vec<(usize, usize)> = runs.iter().enumerate().filter(|(_, run)| !run.closed).flat_map(|(i, run)| [(i, 0, run.v0), (i, 1, run.v1)]).filter(|x| x.2 == v).map(|x| (x.0, x.1)).collect();
        let [(a, ea), (b, eb)] = here.as_slice() else {
            continue;
        };
        let valence = body.edges.iter().filter(|e| e.v0 == v || e.v1 == v).count();
        if a == b || valence != 3 || corners[*a].2.convex != corners[*b].2.convex || !matches!(made[*a].1[*ea], End::Flush) || !matches!(made[*b].1[*eb], End::Flush) {
            continue;
        }
        let leaving = |i: usize, end: usize| -> Vec3 {
            let run = &runs[i];
            if end == 0 {
                run.curve.derivative(run.t0).normalized()
            } else {
                -run.curve.derivative(run.t1).normalized()
            }
        };
        let sine = leaving(*a, *ea).cross(leaving(*b, *eb)).norm();
        if sine < 1e-6 {
            continue;
        }
        let on_past = |i: usize, end: usize| -> Result<Body> {
            let (profile, ends, reach) = &made[i];
            let mut ends = *ends;
            ends[end] = End::Extend(3.0 * reach / sine);
            sweep(profile, &runs[i], &corners[i].3, ends, feature)
        };
        let piece = on_past(*a, *ea).and_then(|x| on_past(*b, *eb).and_then(|y| boolean_with(&x, &y, BoolOp::Intersect, Checks::Interim)));
        if let Ok(mut piece) = piece {
            if !piece.faces.is_empty() {
                let meeting: Vec<usize> = [*a, *b].iter().flat_map(|i| runs[*i].faces).collect();
                mark_source(&mut piece, body, &meeting);
                if corners[*a].2.convex {
                    cuts.push(piece);
                } else {
                    adds.push(piece);
                }
            }
        }
    }

    // Corners where three blended edges meet, all convex or all concave, get a sphere, the shape a
    // rolling ball actually makes. The tools alone would leave a sharp peak there.
    if let Kind::Fillet(r) = kind {
        for v in 0..body.vertices.len() {
            if let Some((mut tool, convex)) = sphere_corner(body, &runs, &corners, v, r, feature)? {
                let meeting: Vec<usize> =
                    runs.iter().filter(|run| !run.closed && (run.v0 == v || run.v1 == v)).flat_map(|run| run.faces).collect();
                mark_source(&mut tool, body, &meeting);
                if convex {
                    cuts.push(tool);
                } else {
                    adds.push(tool);
                }
            }
        }
    }

    // Tools that don't touch can go in together, which is far cheaper than cutting a growing body again
    // for each. A blend's tools can meet each other at corners, so if that refuses, one at a time still
    // gets its chance.
    let at_once = (|| -> Result<Body> {
        let then = if adds.is_empty() { Checks::Full } else { Checks::Interim };
        let cut = boolean_many_with(body, &cuts, BoolOp::Subtract, then)?;
        boolean_many_with(&cut, &adds, BoolOp::Union, Checks::Full)
    })();
    let out = match at_once {
        Ok(out) => out,
        Err(_) => {
            // Only the last step needs the full checks, and they apply to the final result.
            let total = cuts.len() + adds.len();
            let mut out = body.clone();
            let steps = cuts.iter().map(|t| (t, BoolOp::Subtract)).chain(adds.iter().map(|t| (t, BoolOp::Union)));
            for (i, (tool, op)) in steps.enumerate() {
                let checks = if i + 1 == total { Checks::Full } else { Checks::Interim };
                out = boolean_with(&out, tool, op, checks)?;
            }
            out
        }
    };
    let mut along = Vec::new();
    for ((run, (_, _, corner, section)), sure) in runs.iter().zip(&corners).zip(&certain) {
        for (st, sure) in stations_of(run, section).into_iter().zip(sure) {
            let t = if section.circular { st } else { run.t0 + st };
            along.push((run.curve.eval(t), corner.convex, *sure));
        }
    }
    check_blend(body, &out, &along, name)?;
    Ok(out)
}

/// Checks a blend's result in ways that don't depend on how it was made. A fillet or chamfer only ever lies between points
/// on the part's own faces, so it can't reach past the part's bounds. And where it was asked for, the edge has to go:
/// taken off a convex edge, and filled in on a concave one. `along` is points along the edges asked for, each with
/// whether its edge is convex, and whether the ball certainly has its faces to stand on there. Where it does, the edge has
/// to go. A blend that runs off a face can leave a little of an edge sharp near its ends, so elsewhere it's most of them
/// that have to go. None can go the wrong way.
pub(crate) fn check_blend(original: &Body, result: &Body, along: &[(Vec3, bool, bool)], name: &str) -> Result<()> {
    let bounds = original.aabb();
    let slack = 10.0 * TOL + 1e-9 * bounds.diagonal();
    let inside = |p: Vec3| {
        p.x >= bounds.min.x - slack && p.y >= bounds.min.y - slack && p.z >= bounds.min.z - slack && p.x <= bounds.max.x + slack && p.y <= bounds.max.y + slack && p.z <= bounds.max.z + slack
    };
    for e in &result.edges {
        for k in 0..=8 {
            let p = e.curve.eval(e.t0 + (e.t1 - e.t0) * k as f64 / 8.0);
            if !inside(p) {
                return Err(Error::NearDegenerate(format!(
                    "a {name} came out reaching past the part itself, to ({:.4}, {:.4}, {:.4}), which a blend can't do",
                    p.x, p.y, p.z
                )));
            }
        }
    }
    if along.is_empty() {
        return Ok(());
    }
    let index = Index::new(result);
    let mut gone = 0;
    for &(p, convex, sure) in along {
        match (index.locate_in_body(p), convex) {
            (BodyLoc::Outside, true) | (BodyLoc::Inside, false) => gone += 1,
            (BodyLoc::Inside, true) | (BodyLoc::Outside, false) => {
                return Err(Error::NearDegenerate(format!(
                    "a {name} came out with material on the wrong side of its edge, at ({:.4}, {:.4}, {:.4})",
                    p.x, p.y, p.z
                )))
            }
            (BodyLoc::OnFace(_), _) if sure => {
                return Err(Error::NearDegenerate(format!(
                    "a {name} left its edge as it was at ({:.4}, {:.4}, {:.4}), where it had room",
                    p.x, p.y, p.z
                )))
            }
            (BodyLoc::OnFace(_), _) => {}
        }
    }
    if 2 * gone < along.len() {
        return Err(Error::NearDegenerate(format!("a {name} left most of its edges as they were, which means it didn't happen")));
    }
    Ok(())
}

/// The piece of corner outside the rolling ball, where three blended straight edges between planes meet at a vertex, all
/// convex or all concave. Which of those it is comes with it: it's cut from a convex corner and added to a concave one.
fn sphere_corner(
    body: &Body,
    runs: &[Run],
    corners: &[(usize, Vec<Vec2>, Corner, Section)],
    v: usize,
    r: f64,
    feature: u32,
) -> Result<Option<(Body, bool)>> {
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
    let convex = corners[*a].2.convex;
    for &i in &[*a, *b, *c] {
        let (_, _, corner, section) = &corners[i];
        if corner.convex != convex || corner.through_edge || section.circular {
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
    // The ball's centre is `r` under each of the three planes, or over them in a concave corner.
    let (n1, n2, n3) = (p1.z, p2.z, p3.z);
    let det = n1.dot(n2.cross(n3));
    if det.abs() < 1e-9 {
        return Ok(None);
    }
    let off = if convex { -r } else { r };
    let (d1, d2, d3) = (n1.dot(p1.origin) + off, n2.dot(p2.origin) + off, n3.dot(p3.origin) + off);
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
            Some(b) => boolean_with(&b, &piece, BoolOp::Intersect, Checks::Interim)?,
        });
    }
    let block = block.unwrap();
    let ball = sphere(&Frame::new(centre, (corner_point - centre).normalized(), Vec3::X), r, feature)?;
    Ok(Some((boolean_with(&block, &ball, BoolOp::Subtract, Checks::Interim)?, convex)))
}

// ----- Picking edges -----

impl Body {
    /// Indices of edges that satisfy a predicate.
    pub fn edges_where(&self, mut pred: impl FnMut(usize, &Edge) -> bool) -> Vec<usize> {
        self.edges.iter().enumerate().filter(|(i, e)| pred(*i, e)).map(|(i, _)| i).collect()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::union;

    fn block() -> Body {
        box_solid(&Frame::WORLD, 20.0, 10.0, 6.0, 1).unwrap()
    }

    /// Points along the top edge at y = 0, which is convex.
    fn along_top(certain: bool) -> Vec<(Vec3, bool, bool)> {
        (1..10).map(|k| (v3(2.0 * k as f64, 0.0, 6.0), true, certain)).collect()
    }

    fn rounded() -> (Body, Body) {
        let b = block();
        let e = b.edges_where(|i, _| b.edge_midpoint(i).dist(v3(10.0, 0.0, 6.0)) < 1e-9)[0];
        let f = fillet(&b, &[e], 2.0, 9).unwrap();
        (b, f)
    }

    #[test]
    fn a_blend_that_did_nothing_is_refused() {
        let b = block();
        // Its edge is just where it was: where it certainly had room, and even where it might have run off.
        let why = check_blend(&b, &b, &along_top(true), "fillet").unwrap_err().to_string();
        assert!(why.contains("left its edge as it was"), "{why}");
        let why = check_blend(&b, &b, &along_top(false), "fillet").unwrap_err().to_string();
        assert!(why.contains("didn't happen"), "{why}");
        // Done, it passes.
        let (b, f) = rounded();
        check_blend(&b, &f, &along_top(true), "fillet").unwrap();
    }

    #[test]
    fn a_blend_stopped_short_where_it_had_room_is_refused() {
        // Rounded for only the first 15 of its 20.
        let b = block();
        let short = Run { t0: 0.0, t1: 15.0, ..build_runs(&b, &b.edges_where(|i, _| b.edge_midpoint(i).dist(v3(10.0, 0.0, 6.0)) < 1e-9)).unwrap().remove(0) };
        let section = section_of(&short, 20.0).unwrap();
        let corner = corner_of(&b, &short, &section, 20.0).unwrap();
        let (profile, _) = profile_of(&corner, Kind::Fillet(2.0), false, [None, None]).unwrap();
        let tool = sweep(&profile, &short, &section, [End::Flush, End::Flush], 9).unwrap();
        let stopped = boolean_with(&b, &tool, BoolOp::Subtract, Checks::Full).unwrap();
        let why = check_blend(&b, &stopped, &along_top(true), "fillet").unwrap_err().to_string();
        assert!(why.contains("left its edge as it was"), "{why}");
        // Where it might have run off, that's let through.
        check_blend(&b, &stopped, &along_top(false), "fillet").unwrap();
    }

    #[test]
    fn a_blend_that_reaches_past_the_part_is_refused() {
        let (b, f) = rounded();
        // A strip sticking out past the end, as once came out of a rib's fillet.
        let strip = box_solid(&Frame::translation(v3(19.0, 3.0, 3.0)), 3.0, 1.0, 1.0, 2).unwrap();
        let wrong = union(&f, &strip).unwrap();
        let why = check_blend(&b, &wrong, &along_top(true), "fillet").unwrap_err().to_string();
        assert!(why.contains("reaching past the part"), "{why}");
    }

    #[test]
    fn material_on_the_wrong_side_of_an_edge_is_refused() {
        let b = block();
        let lump = box_solid(&Frame::translation(v3(-1.0, -1.0, 5.0)), 22.0, 2.0, 2.0, 2).unwrap();
        let wrong = union(&b, &lump).unwrap();
        let bigger = box_solid(&Frame::translation(v3(-2.0, -2.0, -2.0)), 30.0, 20.0, 20.0, 3).unwrap();
        let why = check_blend(&bigger, &wrong, &along_top(false), "fillet").unwrap_err().to_string();
        assert!(why.contains("wrong side"), "{why}");
    }
}
