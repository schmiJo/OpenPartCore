//! Offset, shell and draft, which all change a body's surfaces.
//!
//! Each face gets a new surface. Then every vertex is found again where its new surfaces meet, and
//! every edge is rebuilt between its new vertices. All five surface types offset to their own kind, and
//! a drafted cylinder is a cone, so the result is as exact as the original.
//!
//! A change can be too big for a face to survive: a fillet smaller than the distance, a hole that closes, a chamfer that
//! gets used up. Such a face is redundant by then, as its neighbours have moved past it, so it's deleted, with `delete`,
//! and the change is tried again without it, which comes to the same result. That finds a chain of them in turn.
//!
//! Walls can also cross each other, where the part would split in two or two parts of it would join. A surface crossing
//! itself isn't untangled here. It's avoided, by taking the part apart: moving in, a part with a pocket is the pocket's
//! surroundings moved in, less the pocket moved out, since a point is that far inside the first if and only if it's that far
//! inside the block and that far from the pocket. Moving out, a part with a boss is the part without it moved out, together
//! with the boss moved out. Those pieces don't cross themselves, and a Boolean settles what they do to each other, including
//! any splitting or joining. `delete` finds the pieces, so this takes what it takes: pockets, bosses, slots and the like.

use crate::boolean::classify::{BodyLoc, FaceLoc, Index};
use crate::boolean::simplify::simplify;
use crate::boolean::ssi::{self, Ssi};
use crate::boolean::{subtract};
use crate::delete::delete_faces_mapped;
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::Patch;
use crate::spline::Spline;
use crate::surface::{refine_on_two, Surface};
use crate::topo::{Body, Edge, FaceTag};
use crate::validate::validate;
use std::sync::Arc;

/// How much two faces' normals can differ along an edge and still count as tangent.
const TANGENT: f64 = 1e-7;

fn refuse(what: String) -> Error {
    Error::InvalidInput(what)
}

/// What stopped a change, and which faces or edges it came down to, if any.
struct Stuck {
    why: Error,
    faces: Vec<usize>,
    edges: Vec<usize>,
}

impl From<Error> for Stuck {
    fn from(why: Error) -> Stuck {
        Stuck { why, faces: Vec::new(), edges: Vec::new() }
    }
}

/// Whether two faces meet convex, concave or smoothly along an edge, by its middle.
fn edge_kind(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], e: usize) -> Option<bool> {
    let [a, b] = uses[e].as_slice() else { return None };
    let p = body.edges[e].eval_mid();
    let (fa, fb) = (&body.faces[a.0], &body.faces[b.0]);
    let (na, nb) = (fa.outward_normal(p), fb.outward_normal(p));
    if na.cross(nb).norm() < TANGENT * 10.0 {
        return None;
    }
    let edge = &body.edges[e];
    let t = edge.curve.derivative((edge.t0 + edge.t1) / 2.0) * if a.3 { -1.0 } else { 1.0 };
    // Away from the first face, across the edge. A convex edge has the second face's normal leaning that way.
    Some(t.cross(na).dot(nb) > 0.0)
}

/// Deletes the faces that something has gone wrong for, from those named, or those with the most of their edges turned
/// round. If they can't go alone, it takes those joined to them by the kind of edge that closes up under this change.
fn heal(body: &Body, stuck: &Stuck, closes: &dyn Fn(usize, bool) -> bool) -> Option<(Body, Vec<usize>)> {
    let uses = body.edge_uses();
    let mut group: Vec<usize> = stuck.faces.clone();
    let mut alone: Option<usize> = None;
    if group.is_empty() {
        if stuck.edges.is_empty() {
            return None;
        }
        let mut count = vec![0usize; body.faces.len()];
        for &e in &stuck.edges {
            for u in &uses[e] {
                count[u.0] += 1;
            }
        }
        let score = |f: usize| count[f] as f64 / body.faces[f].loops.iter().map(Vec::len).sum::<usize>().max(1) as f64;
        let best = (0..body.faces.len()).map(score).fold(0.0f64, f64::max);
        group = (0..body.faces.len()).filter(|&f| score(f) >= best - 1e-9).collect();
        alone = group.iter().copied().min_by(|&a, &b| body.face_aabb(a).diagonal().total_cmp(&body.face_aabb(b).diagonal()));
    }
    // Separate groups go one at a time, as one of them failing says nothing about the others, and the next try finds
    // what's left.
    let joined = |a: usize, b: usize| {
        body.faces[a].loops.iter().flatten().any(|c| uses[c.edge].iter().any(|u| u.0 == b))
    };
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for &f in &group {
        let (mut touching, rest): (Vec<Vec<usize>>, Vec<Vec<usize>>) = groups.into_iter().partition(|g| g.iter().any(|&h| joined(f, h)));
        let mut merged: Vec<usize> = touching.drain(..).flatten().collect();
        merged.push(f);
        groups = rest;
        groups.push(merged);
    }
    let size = ssi_size(body);
    for start in groups {
        let mut group = start;
        for _ in 0..8 {
            if group.len() >= body.faces.len() {
                break;
            }
            if let Ok(done) = delete_faces_mapped(body, &group) {
                return Some(done);
            }
            if let Some(f) = alone.take() {
                if group.len() > 1 {
                    if let Ok(done) = delete_faces_mapped(body, &[f]) {
                        return Some(done);
                    }
                }
            }
            let before = group.len();
            for f in group.clone() {
                for c in body.faces[f].loops.iter().flatten() {
                    for u in &uses[c.edge] {
                        if u.0 == f || group.contains(&u.0) {
                            continue;
                        }
                        // Pieces of one surface go together. Otherwise it's the kind of edge that closes up.
                        let seam = ssi::coincident(&body.faces[f].surface, &body.faces[u.0].surface, size).is_some();
                        if seam || edge_kind(body, &uses, c.edge).is_some_and(|convex| closes(f, convex)) {
                            group.push(u.0);
                        }
                    }
                }
            }
            if group.len() == before {
                break;
            }
        }
    }
    None
}

fn ssi_size(body: &Body) -> f64 {
    body.aabb().diagonal().max(1.0)
}

/// Tries a change, deleting what doesn't survive it and trying again. `attempt` gets the body as it now stands, with the
/// face each of its faces started as. `closes` says which edges close up, and `went` checks the result against points of
/// the faces that went, by the face each started as, since a face that only partly gave way mustn't go at all.
fn surviving(
    body: &Body,
    attempt: &mut dyn FnMut(&Body, &[usize]) -> std::result::Result<Body, Stuck>,
    closes: &dyn Fn(usize, bool) -> bool,
    went: &dyn Fn(&Body, usize, &[Vec3]) -> Result<()>,
) -> Result<Body> {
    let mut current = body.clone();
    let mut from: Vec<usize> = (0..body.faces.len()).collect();
    let mut healed = false;
    let mut removed: Vec<(usize, Vec<Vec3>)> = Vec::new();
    for _ in 0..=body.faces.len() {
        match attempt(&current, &from) {
            Ok(done) => {
                if !healed {
                    return Ok(done);
                }
                for (f, points) in &removed {
                    went(&done, *f, points)?;
                }
                let tidy = simplify(done)?;
                validate(&tidy).map_err(|defects| refuse(format!("the result isn't a valid solid: {}", defects.join("; "))))?;
                return Ok(tidy);
            }
            Err(stuck) => {
                let closes_here = |f: usize, convex: bool| closes(from[f], convex);
                let Some((next, kept)) = heal(&current, &stuck, &closes_here) else {
                    return Err(stuck.why);
                };
                let index = Index::new(&current);
                for f in 0..current.faces.len() {
                    if !kept.contains(&f) {
                        removed.push((from[f], samples_across(&current, &index, f)));
                    }
                }
                from = kept.iter().map(|&k| from[k]).collect();
                current = next;
                healed = true;
            }
        }
    }
    Err(refuse("too many faces gave way one after the other".to_string()))
}

/// Refuses walls that have crossed: a point of a moved face that has ended up nearer another face of the part than that
/// face has moved. A real offset can't do that, since the moved copy of the other face would be in its way. It's checked
/// around each face's edges, which works when crossed walls won't mesh, and at up to 24 points of it when they will. So a
/// crossing smaller than those could get through.
fn walls_clear(base: &Body, moved: &Body, distance: &dyn Fn(usize) -> f64) -> Result<()> {
    let mut points: Vec<Vec<Vec3>> = vec![Vec::new(); moved.faces.len()];
    for (f, face) in moved.faces.iter().enumerate() {
        for c in face.loops.iter().flatten() {
            points[f].extend([0.0, 0.25, 0.5, 0.75].iter().map(|s| moved.coedge_point(*c, *s)));
        }
    }
    if let Ok(mesh) = crate::tess::tessellate(moved, &crate::tess::TessParams::draft_for_size(moved.aabb().diagonal())) {
        let mut on_face: Vec<Vec<u32>> = vec![Vec::new(); moved.faces.len()];
        for (t, tri) in mesh.triangles.iter().enumerate() {
            on_face[mesh.face_of_triangle[t] as usize].extend_from_slice(tri);
        }
        for (f, list) in on_face.iter_mut().enumerate() {
            list.sort_unstable();
            list.dedup();
            let stride = (list.len() / 24).max(1);
            points[f].extend(list.iter().step_by(stride).map(|v| mesh.positions[*v as usize]));
        }
    }
    let index = Index::new(base);
    let boxes: Vec<Aabb> = (0..base.faces.len()).map(|f| base.face_aabb(f)).collect();
    for (f, list) in points.iter().enumerate() {
        for &q in list {
            for (g, face) in base.faces.iter().enumerate() {
                // A wall moving in passes a face moving out quite properly, as a shell's do at an open face.
                let reach = distance(g).abs();
                if g == f || reach == 0.0 || distance(g) * distance(f) < 0.0 || !boxes[g].inflated(reach).contains(q) {
                    continue;
                }
                if face.surface.sdf(q).abs() >= reach - 1e-6 {
                    continue;
                }
                let foot = face.surface.project(q);
                if index.locate_on_face(g, foot, 1e-6) == FaceLoc::Inside {
                    return Err(refuse(format!(
                        "walls cross near ({:.3}, {:.3}, {:.3}): at this distance the part would split in two or two parts of it would join, which this can't do yet",
                        q.x, q.y, q.z
                    )));
                }
            }
        }
    }
    Ok(())
}

/// Points across a face, and not just around it: every point of a face's edge is on a neighbour too, which says nothing
/// about the face itself.
fn samples_across(body: &Body, index: &Index, f: usize) -> Vec<Vec3> {
    let face = &body.faces[f];
    let (lo, hi) = body.face_chart_bounds(f);
    let mut out = boundary_samples(body, f);
    const N: usize = 12;
    for i in 1..N {
        for j in 1..N {
            let uv = v2(lo.x + (hi.x - lo.x) * i as f64 / N as f64, lo.y + (hi.y - lo.y) * j as f64 / N as f64);
            let p = face.surface.from_chart(&face.chart, uv);
            if index.locate_on_face(f, p, 1e-6) == FaceLoc::Inside {
                out.push(p);
            }
        }
    }
    out
}

/// The distance from a point to the surface of a body, faces and edges both.
fn distance_to_surface(body: &Body, index: &Index, p: Vec3) -> f64 {
    let mut best = f64::INFINITY;
    for f in 0..body.faces.len() {
        let s = &body.faces[f].surface;
        let d = s.sdf(p).abs();
        if d < best && index.locate_on_face(f, s.project(p), 1e-6) != FaceLoc::Outside {
            best = d;
        }
    }
    for e in &body.edges {
        best = best.min(e.curve.closest(p, e.t0, e.t1).1);
    }
    best
}

/// That a face which went was really redundant, as its neighbours had moved past it: every point of it has to be as far
/// from the result as it moved, on the right side. If some of it should still be there, as when a tapered chamfer has only
/// used up its narrow end, then this isn't the right shape, and it says so.
fn redundant(result: &Body, d: f64, points: &[Vec3]) -> Result<()> {
    let index = Index::new(result);
    for &p in points {
        let inside = index.locate_in_body(p) == BodyLoc::Inside;
        if inside != (d > 0.0) || distance_to_surface(result, &index, p) < d.abs() - 1e-6 {
            return Err(refuse(format!(
                "a face only partly gives way near ({:.3}, {:.3}, {:.3}): what's left of it would need the part's faces rearranged, which this can't do yet",
                p.x, p.y, p.z
            )));
        }
    }
    Ok(())
}

/// A surface moved `d` along its own normal: the one it defines, not the face's outward one. `reach` is where the
/// surface has to go on existing, which only a swept spline needs to know.
pub(crate) fn moved_surface(s: &Surface, d: f64, reach: &[Vec3]) -> Result<Surface> {
    let positive = |r: f64| -> Result<f64> {
        if r > MIN_FEATURE {
            Ok(r)
        } else {
            Err(refuse(format!("offsetting by {d} shrinks a curved face away to nothing")))
        }
    };
    Ok(match *s {
        // A swept spline's offset is no spline, so it becomes the free-form surface it already is, moved.
        Surface::Extruded { .. } | Surface::Revolved { .. } => {
            let (frame, patch) = swept_as_patch(s, reach)?;
            Surface::Free { frame, patch: Arc::new(patch.moved_by(d)?) }
        }
        Surface::Free { frame, ref patch } => Surface::Free { frame, patch: Arc::new(patch.moved_by(d)?) },
        Surface::Plane(f) => Surface::Plane(Frame { origin: f.origin + f.z * d, ..f }),
        Surface::Cylinder { frame, radius } => Surface::Cylinder { frame, radius: positive(radius + d)? },
        // Moving along a cone's slanted normal shifts it sideways by more than d.
        Surface::Cone { frame, radius, half_angle } => Surface::Cone { frame, radius: radius + d / half_angle.cos(), half_angle },
        Surface::Sphere { frame, radius } => Surface::Sphere { frame, radius: positive(radius + d)? },
        Surface::Torus { frame, major, minor } => Surface::Torus { frame, major, minor: positive(minor + d)? },
    })
}

/// An extruded or revolved spline as the rational patch it exactly is, over as much of it as `reach` calls for, with
/// the normal it had.
fn swept_as_patch(s: &Surface, reach: &[Vec3]) -> Result<(Frame, Patch)> {
    match s {
        Surface::Extruded { frame, curve } => {
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for p in reach {
                let z = frame.to_local(*p).z;
                lo = lo.min(z);
                hi = hi.max(z);
            }
            let room = (hi - lo).max(1.0);
            let (z0, z1) = (lo - room, hi + room);
            let c = &curve.spline;
            let (mut points, mut weights) = (Vec::with_capacity(c.points().len() * 2), Vec::with_capacity(c.points().len() * 2));
            for (q, w) in c.points().iter().zip(c.weights()) {
                points.push(v3(q.x, q.y, z0));
                points.push(v3(q.x, q.y, z1));
                weights.push(*w);
                weights.push(*w);
            }
            let patch = Patch::new((c.degree(), 1), (c.knots().to_vec(), vec![z0, z0, z1, z1]), (c.points().len(), 2), points, weights)?;
            Ok((*frame, patch))
        }
        Surface::Revolved { frame, curve } => {
            // The angles it covers, measured from the middle of them, as a face is at most half a turn and a measure from
            // one of its ends could land on either side of the half turn away.
            let locals: Vec<Vec3> = reach.iter().map(|p| frame.to_local(*p)).filter(|l| l.x.hypot(l.y) > 1e-9).collect();
            let sum = locals.iter().fold((0.0f64, 0.0f64), |m, l| {
                let r = l.x.hypot(l.y);
                (m.0 + l.y / r, m.1 + l.x / r)
            });
            let middle = sum.0.atan2(sum.1);
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for l in &locals {
                let mut r = l.y.atan2(l.x) - middle;
                while r > PI {
                    r -= TAU;
                }
                while r <= -PI {
                    r += TAU;
                }
                lo = lo.min(r + middle);
                hi = hi.max(r + middle);
            }
            if !(hi > lo) {
                return Err(Error::Unsupported("a revolved face with no width can't be offset".into()));
            }
            let room = 0.05;
            let (a0, a1) = (lo - room, (hi + room).min(lo - room + TAU));
            let spans = ((a1 - a0) / (PI / 2.0) - 1e-9).ceil().max(1.0) as usize;
            let step = (a1 - a0) / spans as f64;
            let half = (step / 2.0).cos();
            let mut around: Vec<(f64, f64, f64)> = Vec::with_capacity(2 * spans + 1);
            for k in 0..spans {
                let a = a0 + step * k as f64;
                around.push((a.cos(), a.sin(), 1.0));
                around.push(((a + step / 2.0).cos() / half, (a + step / 2.0).sin() / half, half));
            }
            around.push((a1.cos(), a1.sin(), 1.0));
            let mut knots_u = vec![a0; 3];
            for k in 1..spans {
                knots_u.extend(vec![a0 + step * k as f64; 2]);
            }
            knots_u.extend(vec![a1; 3]);
            let c = &curve.spline;
            let (mut points, mut weights) = (Vec::with_capacity(around.len() * c.points().len()), Vec::with_capacity(around.len() * c.points().len()));
            for (x, y, w) in &around {
                for (q, cw) in c.points().iter().zip(c.weights()) {
                    points.push(v3(q.x * x, q.x * y, q.y));
                    weights.push(cw * w);
                }
            }
            let patch = Patch::new((2, c.degree()), (knots_u, c.knots().to_vec()), (around.len(), c.points().len()), points, weights)?;
            Ok((*frame, patch))
        }
        _ => Err(Error::Internal("only a swept surface is converted".into())),
    }
}

/// Points on a face's boundary.
pub(crate) fn boundary_samples(body: &Body, f: usize) -> Vec<Vec3> {
    let mut out = Vec::new();
    for c in body.faces[f].loops.iter().flatten() {
        for i in 0..=8 {
            out.push(body.coedge_point(*c, i as f64 / 8.0));
        }
    }
    out
}

/// The new position of a vertex: nearest to where it was, on all the surfaces given. Each pass
/// takes the smallest step that meets all of them to first order, skipping any whose normal adds nothing,
/// as a tangent neighbour's doesn't.
pub(crate) fn point_on_surfaces(surfaces: &[Surface], start: Vec3) -> Option<Vec3> {
    let mut p = start;
    for _ in 0..40 {
        // Gram-Schmidt on the normals, carrying the right-hand sides along.
        let mut basis: Vec<(Vec3, f64)> = Vec::with_capacity(3);
        let mut largest = 0.0f64;
        for s in surfaces {
            let f = s.sdf(p);
            largest = largest.max(f.abs());
            let (mut n, mut rhs) = (s.normal(p), -f);
            for (b, r) in &basis {
                let c = n.dot(*b);
                n = n - *b * c;
                rhs -= c * r;
            }
            let len = n.norm();
            if len > 1e-5 {
                basis.push((n * (1.0 / len), rhs / len));
            }
        }
        if largest < 1e-12 {
            return Some(p);
        }
        let step = basis.iter().fold(Vec3::ZERO, |sum, (b, r)| sum + *b * *r);
        p = p + step;
    }
    let worst = surfaces.iter().fold(0.0f64, |m, s| m.max(s.sdf(p).abs()));
    (worst < TOL * 0.1).then_some(p)
}

/// Where a curve meets the surfaces given, starting near `start`. A tangent edge fixes a vertex to its curve, leaving
/// only where along it, which the other surfaces settle. A spline carries on past its ends along its tangent there, as a
/// free-form surface does, for when a neighbour has moved away from where the curve stops.
pub(crate) fn point_along(c: &Curve, start: Vec3, others: &[Surface]) -> Vec3 {
    let domain = if let Curve::Spline(s) = c { Some(s.domain()) } else { None };
    let eval = |t: f64| -> (Vec3, Vec3) {
        if let Some((d0, d1)) = domain {
            let inside = t.clamp(d0, d1);
            let d = c.derivative(inside);
            return (c.eval(inside) + d * (t - inside), d);
        }
        (c.eval(t), c.derivative(t))
    };
    let (mut t, _) = match domain {
        Some((d0, d1)) => c.closest(start, d0, d1),
        None => c.closest(start, -1e9f64.min(0.0), 1e9),
    };
    if matches!(c, Curve::Circle { .. }) {
        t = c.closest(start, 0.0, TAU).0;
    }
    let (mut p, _) = eval(t);
    for _ in 0..40 {
        let (_, d) = eval(t);
        let Some(s) = others.iter().find(|s| s.normal(p).dot(d).abs() > 1e-6 * d.norm()) else {
            break;
        };
        let f = s.sdf(p);
        if f.abs() < 1e-13 {
            break;
        }
        t -= f / s.normal(p).dot(d);
        p = eval(t).0;
    }
    p
}

/// A spline from `a` to `b` along another one, carrying on straight past its ends if they're further. Those straight pieces
/// are joined at a knot and not smoothed in, since that's the shape of the surface there, which carries on along its tangent.
fn spliced(c: &Curve, a: Vec3, b: Vec3) -> Option<(Curve, f64, f64)> {
    let Curve::Spline(old) = c else { return None };
    if old.degree() != 3 || !old.is_polynomial() {
        return None;
    }
    let (d0, d1) = old.domain();
    let reach = |p: Vec3| {
        let (t, off) = c.closest(p, d0, d1);
        (off <= 2.0 * TOL).then_some(t)
    };
    let (mut at_a, mut at_b) = (reach(a), reach(b));
    // An end this close is the same end, and it's the fit that's a little out, as it can be at a pole.
    let mut own = old.points().to_vec();
    if at_a.is_none() && a.dist(own[0]) <= 1e-5 {
        own[0] = a;
        at_a = Some(d0);
    }
    let last_own = own.len() - 1;
    if at_b.is_none() && b.dist(own[last_own]) <= 1e-5 {
        own[last_own] = b;
        at_b = Some(d1);
    }
    let mut points: Vec<Vec3> = Vec::new();
    let mut knots: Vec<f64> = Vec::new();
    if at_a.is_none() {
        let first = own[0];
        let length = a.dist(first);
        if !(length > MIN_FEATURE) {
            return None;
        }
        points.extend([a, a + (first - a) / 3.0, a + (first - a) * (2.0 / 3.0)]);
        knots.extend(vec![d0 - length; 4]);
        knots.extend(vec![d0; 3]);
        knots.extend_from_slice(&old.knots()[4..]);
    } else {
        knots.extend_from_slice(old.knots());
    }
    points.extend_from_slice(&own);
    if at_b.is_none() {
        let last = own[last_own];
        let length = b.dist(last);
        if !(length > MIN_FEATURE) {
            return None;
        }
        points.extend([last + (b - last) / 3.0, last + (b - last) * (2.0 / 3.0), b]);
        knots.truncate(knots.len() - 1);
        knots.extend(vec![d1 + length; 4]);
    }
    let weights = vec![1.0; points.len()];
    let joined = Spline::new(3, knots, points, weights).ok()?;
    let (e0, e1) = joined.domain();
    let (t0, t1) = (at_a.unwrap_or(e0), at_b.unwrap_or(e1));
    (t1 > t0).then(|| (Curve::Spline(Arc::new(joined)), t0, t1))
}

/// The parameter range taking a curve from `a` to `b` by the way nearest `middle`, with the curve
/// turned round if need be so that the parameter increases along it.
pub(crate) fn fitted(curve: &Curve, a: Vec3, b: Vec3, middle: Vec3) -> Option<(Curve, f64, f64)> {
    let tol = TOL * 10.0;
    match curve {
        Curve::Line { origin, dir } => {
            let (t0, t1) = ((a - *origin).dot(*dir), (b - *origin).dot(*dir));
            let (c, s0, s1) = if t1 > t0 { (curve.clone(), t0, t1) } else { (Curve::Line { origin: *origin, dir: -*dir }, -t0, -t1) };
            (c.eval(s0).dist(a) < tol && c.eval(s1).dist(b) < tol).then_some((c, s0, s1))
        }
        Curve::Circle { .. } | Curve::Ellipse { .. } => {
            let frame = match curve {
                Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => *frame,
                _ => unreachable!(),
            };
            let flipped_frame = Frame { z: -frame.z, y: -frame.y, ..frame };
            let with = |f: Frame| match *curve {
                Curve::Circle { radius, .. } => Curve::Circle { frame: f, radius },
                Curve::Ellipse { major, minor, .. } => Curve::Ellipse { frame: f, major, minor },
                _ => unreachable!(),
            };
            let mut best: Option<(f64, (Curve, f64, f64))> = None;
            for f in [frame, flipped_frame] {
                let c = with(f);
                let (t0, _) = c.closest(a, 0.0, TAU);
                let (t1, _) = c.closest(b, 0.0, TAU);
                let mut end = t1;
                while end <= t0 {
                    end += TAU;
                }
                if c.eval(t0).dist(a) > tol || c.eval(end).dist(b) > tol {
                    continue;
                }
                let off = c.eval((t0 + end) / 2.0).dist(middle);
                if best.as_ref().is_none_or(|x| off < x.0) {
                    best = Some((off, (c, t0, end)));
                }
            }
            best.map(|b| b.1)
        }
        // A curve with ends of its own has to reach its corners well inside what the validator allows.
        Curve::Spline(c) => {
            let (d0, d1) = c.domain();
            let ((t0, off0), (t1, off1)) = (curve.closest(a, d0, d1), curve.closest(b, d0, d1));
            (off0 < 2.0 * TOL && off1 < 2.0 * TOL && t1 > t0).then(|| (curve.clone(), t0, t1))
        }
        Curve::Sampled(c) => {
            let last = (c.points.len() - 1) as f64;
            let ((t0, off0), (t1, off1)) = (curve.closest(a, 0.0, last), curve.closest(b, 0.0, last));
            (off0 < 2.0 * TOL && off1 < 2.0 * TOL && t1 > t0).then(|| (curve.clone(), t0, t1))
        }
    }
}

/// The plane a seam lies in. Seams run along a surface's axis or around it, so a change that keeps the
/// axis keeps this plane too, and it stays true of the new surface.
fn seam_plane(edge: &Edge, on: &Surface) -> Surface {
    let middle = edge.eval_mid();
    let n = match &edge.curve {
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => frame.z,
        _ => (edge.curve.eval(edge.t1) - edge.curve.eval(edge.t0)).cross(on.normal(on.project(middle))).normalized(),
    };
    Surface::Plane(Frame::from_z(middle, n))
}

/// The new curve for an edge between surfaces `s` and `t`, from `a` to `b`.
fn rebuilt_curve(edge: &Edge, s: &Surface, t: &Surface, same_surface: bool, a: Vec3, b: Vec3) -> Result<(Curve, f64, f64)> {
    let n = 24;
    let samples: Vec<Vec3> = (1..n).map(|i| edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64)).collect();
    let cutter = same_surface.then(|| seam_plane(edge, s));
    curve_between(s, cutter.as_ref().unwrap_or(t), a, b, edge.eval_mid(), matches!(edge.curve, Curve::Line { .. }), &samples)
}

/// A curve where two surfaces meet, from `a` to `b`, by the way nearest `middle`. It's exact if the two have a closed form
/// there, and otherwise it follows `samples`, moved onto both surfaces.
pub(crate) fn curve_between(s: &Surface, other: &Surface, a: Vec3, b: Vec3, middle: Vec3, try_straight: bool, samples: &[Vec3]) -> Result<(Curve, f64, f64)> {
    let on_both = |p: Vec3, tol: f64| s.sdf(p).abs() < tol && other.sdf(p).abs() < tol;
    // A straight edge often stays straight: rulings, and edges between planes. Checking is cheaper
    // than intersecting, and works where the surfaces are tangent, which intersection doesn't.
    if try_straight && a.dist(b) > MIN_FEATURE {
        let straight = Curve::line_through(a, b);
        if [0.25, 0.5, 0.75].iter().all(|f| on_both(a + (b - a) * *f, TOL)) {
            return fitted(&straight, a, b, middle).ok_or_else(|| Error::Internal("a straight edge didn't fit its ends".into()));
        }
    }
    let mut region = Aabb::EMPTY;
    region.add(a);
    region.add(b);
    region.add(middle);
    let region = region.inflated(a.dist(b).max(a.dist(middle)) + 1.0);
    if let Ssi::Curves { curves, .. } = ssi::intersect(s, other, &region) {
        let mut best: Option<(f64, (Curve, f64, f64))> = None;
        for c in &curves {
            if let Some(found) = fitted(&c.curve, a, b, middle) {
                let off = found.0.eval((found.1 + found.2) / 2.0).dist(middle);
                if best.as_ref().is_none_or(|x| off < x.0) {
                    best = Some((off, found));
                }
            }
        }
        if let Some((_, found)) = best {
            return Ok(found);
        }
    }
    // No closed form: follow the samples, moving them onto both new surfaces.
    let mut points = Vec::with_capacity(samples.len() + 2);
    points.push(a);
    for p in samples {
        points.push(refine_on_two(s, other, *p).ok_or_else(|| refuse("an edge can't be followed onto the new surfaces: they may have stopped meeting there".to_string()))?);
    }
    points.push(b);
    let curve = ssi::sampled_from_points(points, s, other)?;
    let last = match &curve {
        Curve::Sampled(c) => (c.points.len() - 1) as f64,
        _ => unreachable!(),
    };
    Ok((curve, 0.0, last))
}

/// A tangent edge's new curve, from what happens to its points.
fn mapped_tangent_edge(edge: &Edge, map: &mut dyn FnMut(Vec3) -> Vec3) -> Option<Curve> {
    match &edge.curve {
        Curve::Line { origin, dir } => {
            let a = map(*origin + *dir * edge.t0);
            let b = map(*origin + *dir * edge.t1);
            (a.dist(b) > MIN_FEATURE).then(|| Curve::line_through(a, b))
        }
        Curve::Circle { frame, radius } => {
            // The map keeps a rim round its axis, so one point tells where the rim went.
            let p = map(frame.origin + frame.x * *radius);
            let local = p - frame.origin;
            let r = local.dot(frame.x);
            (r > MIN_FEATURE).then(|| Curve::Circle { frame: Frame { origin: frame.origin + frame.z * local.dot(frame.z), ..*frame }, radius: r })
        }
        _ => {
            // Anything else goes through points that follow the old curve, as many as it takes to be within `FIT`.
            let at = |t: f64, map: &mut dyn FnMut(Vec3) -> Vec3| map(edge.curve.eval(t));
            let mut n = 16usize;
            loop {
                let points: Vec<Vec3> = (0..=n).map(|i| at(edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64, map)).collect();
                let spline = Spline::through(&points).ok()?;
                let (d0, d1) = spline.domain();
                let worst = (0..n).map(|i| {
                    let t = edge.t0 + (edge.t1 - edge.t0) * (i as f64 + 0.5) / n as f64;
                    let want = at(t, map);
                    Curve::Spline(Arc::new(spline.clone())).closest(want, d0, d1).1
                }).fold(0.0f64, f64::max);
                if worst <= crate::rolling::FIT || n >= 1024 {
                    return (worst <= crate::rolling::FIT).then(|| Curve::Spline(Arc::new(spline)));
                }
                n *= 2;
            }
        }
    }
}

/// A traced curve as a spline, within `FIT`.
fn sampled_as_spline(curve: &Curve, t0: f64, t1: f64) -> Option<(Curve, f64, f64)> {
    if !matches!(curve, Curve::Sampled(_)) {
        return None;
    }
    let mut n = 16usize;
    while n <= 512 {
        let points: Vec<Vec3> = (0..=n).map(|i| curve.eval(t0 + (t1 - t0) * i as f64 / n as f64)).collect();
        let fitted = Spline::through(&points).ok()?;
        let (d0, d1) = fitted.domain();
        let new_curve = Curve::Spline(Arc::new(fitted));
        let worst = (0..n).map(|i| new_curve.closest(curve.eval(t0 + (t1 - t0) * (i as f64 + 0.5) / n as f64), d0, d1).1).fold(0.0f64, f64::max);
        if worst <= crate::rolling::FIT {
            return Some((new_curve, d0, d1));
        }
        n *= 2;
    }
    None
}

/// How a change treats a tangent edge: its faces stay tangent to each other, so their intersection is
/// ill-conditioned, and the new edge has to come from where the old one's points go.
type TangentMap<'a> = &'a dyn Fn(usize, Vec3) -> Result<Vec3>;

/// The body with each face on a new surface and the same topology.
fn with_surfaces(body: &Body, surfaces: &[Surface], tangent_map: TangentMap<'_>) -> std::result::Result<Body, Stuck> {
    let uses = body.edge_uses();
    // What each edge is between.
    let mut tangent = vec![false; body.edges.len()];
    let mut same = vec![false; body.edges.len()];
    let size = body.aabb().diagonal().max(1.0);
    for (e, edge) in body.edges.iter().enumerate() {
        if uses[e].len() != 2 {
            return Err(Error::Unsupported("an edge used by other than two faces can't be moved".into()).into());
        }
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        same[e] = ssi::coincident(&body.faces[f].surface, &body.faces[g].surface, size).is_some();
        if !same[e] {
            let p = edge.eval_mid();
            let (a, b) = (body.faces[f].surface.normal(p), body.faces[g].surface.normal(p));
            tangent[e] = a.cross(b).norm() < TANGENT;
        } else if matches!(surfaces[f], Surface::Free { .. }) && !matches!(body.faces[f].surface, Surface::Plane(_)) {
            // A seam on a moved free-form surface goes where its points go, which is much cheaper to use from then on
            // than a curve found by solving onto that surface at every point.
            tangent[e] = true;
        }
    }
    let curve_for_tangent = |e: usize| -> Result<Curve> {
        let edge = &body.edges[e];
        let mut failed = None;
        let mut map = |p: Vec3| match tangent_map(e, p) {
            Ok(q) => q,
            Err(why) => {
                failed.get_or_insert(why);
                p
            }
        };
        let c = mapped_tangent_edge(edge, &mut map);
        if let Some(why) = failed {
            return Err(why);
        }
        c.ok_or_else(|| Error::Unsupported("a smooth edge of this kind can't be moved to within the tolerance here".into()))
    };
    let mut tangent_curves: Vec<Option<Curve>> = vec![None; body.edges.len()];
    for e in 0..body.edges.len() {
        if tangent[e] {
            tangent_curves[e] = Some(curve_for_tangent(e)?);
        }
    }
    // Vertices.
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for (e, edge) in body.edges.iter().enumerate() {
        at_vertex[edge.v0].push(e);
        at_vertex[edge.v1].push(e);
    }
    let mut vertices = body.vertices.clone();
    for (v, edges) in at_vertex.iter().enumerate() {
        if edges.is_empty() {
            continue;
        }
        let mut faces: Vec<usize> = edges.iter().flat_map(|&e| uses[e].iter().map(|u| u.0)).collect();
        faces.sort_unstable();
        faces.dedup();
        let mut list: Vec<Surface> = faces.iter().map(|&f| surfaces[f].clone()).collect();
        // A vertex on a seam has to stay in the seam's plane, or the seam couldn't reach it.
        for &e in edges {
            if same[e] {
                list.push(seam_plane(&body.edges[e], &body.faces[uses[e][0].0].surface));
            }
        }
        // A tangent edge fixes the vertex to a curve, leaving only where along it, which the other
        // surfaces settle. Tangent surfaces alone would pin it only to about the square root of the tolerance.
        let along = edges.iter().find_map(|&e| tangent_curves[e].as_ref().map(|c| (e, c)));
        let start = body.vertices[v];
        vertices[v] = match along {
            Some((e, c)) => {
                let others: Vec<Surface> = {
                    let tangent_pair = (uses[e][0].0, uses[e][1].0);
                    faces.iter().filter(|&&f| f != tangent_pair.0 && f != tangent_pair.1).map(|&f| surfaces[f].clone()).collect()
                };
                let p = point_along(c, tangent_map(e, start)?, &others);
                // A fitted curve's end tangent is only so good, which shows over a long stretch past its end. The surfaces
                // settle it from there.
                if matches!(c, Curve::Spline(_)) {
                    point_on_surfaces(&list, p).filter(|q| q.dist(p) < 1e-3).unwrap_or(p)
                } else {
                    p
                }
            }
            None => match point_on_surfaces(&list, start) {
                Some(p) => p,
                None => return Err(Stuck { why: refuse("the new surfaces no longer meet at a corner".to_string()), faces: Vec::new(), edges: edges.clone() }),
            },
        };
        if list.iter().any(|s| s.sdf(vertices[v]).abs() > 10.0 * TOL) {
            return Err(Stuck { why: refuse("the new surfaces no longer meet at a corner".to_string()), faces: Vec::new(), edges: edges.clone() });
        }
    }
    // An edge that has shrunk away or turned round means a face has gone.
    let turned: Vec<usize> = (0..body.edges.len())
        .filter(|&e| {
            let edge = &body.edges[e];
            let (old, new) = (body.vertices[edge.v1] - body.vertices[edge.v0], vertices[edge.v1] - vertices[edge.v0]);
            !uses[e].is_empty() && (new.norm() < MIN_FEATURE || (old.norm() > 1e-9 && old.dot(new) <= 0.0))
        })
        .collect();
    if !turned.is_empty() {
        return Err(Stuck { why: refuse("an edge shrinks away or turns round: the change is too big for the part's shape".to_string()), faces: Vec::new(), edges: turned });
    }
    // Edges.
    let mut out = Body { vertices, ..Body::default() };
    for (e, edge) in body.edges.iter().enumerate() {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (a, b) = (out.vertices[edge.v0], out.vertices[edge.v1]);
        let (curve, t0, t1) = match &tangent_curves[e] {
            Some(c) => fitted(c, a, b, tangent_map(e, edge.eval_mid())?).filter(|f| f.0.eval(f.1).dist(a) <= TOL && f.0.eval(f.2).dist(b) <= TOL).or_else(|| spliced(c, a, b)).ok_or_else(|| refuse("a smooth edge didn't fit its new corners".to_string()))?,
            None => {
                let (c, t0, t1) = rebuilt_curve(edge, &surfaces[f], &surfaces[g], same[e], a, b)?;
                // The same goes for a curve traced on a moved surface: fitted once, within the tolerance, if it can be.
                let moved = [f, g].iter().any(|&h| matches!(&surfaces[h], Surface::Free { patch, .. } if patch.offset() != 0.0));
                if moved { sampled_as_spline(&c, t0, t1).unwrap_or((c, t0, t1)) } else { (c, t0, t1) }
            }
        };
        out.edges.push(Edge { curve, t0, t1, v0: edge.v0, v1: edge.v1 });
    }
    for (f, face) in body.faces.iter().enumerate() {
        let i = out.add_face(surfaces[f].clone(), face.flipped, face.loops.clone(), face.tag);
        out.faces[i].origin = face.origin.clone();
    }
    out.shells = body.shells.clone();
    validate(&out).map_err(|defects| refuse(format!("the result isn't a valid solid: {}", defects.join("; "))))?;
    Ok(out)
}

/// Whether a result meshes into a closed surface, which a change too big for the part's shape often doesn't.
fn meshes_closed(body: &Body) -> Result<()> {
    let check = crate::tess::tessellate(body, &crate::tess::TessParams::for_size(body.aabb().diagonal()))
        .map_err(|why| refuse(format!("the result doesn't mesh, as a face's edges cross each other, so the change is too big for the part's shape: {why}")))?;
    if check.open_edge_count() > 0 {
        return Err(refuse("the result doesn't mesh into a closed surface: the change is too big for the part's shape".to_string()));
    }
    Ok(())
}

/// Moves each face outward by its own distance, negative for inward. Sharp edges stay sharp: neighbouring
/// faces simply meet further out or in. Faces that share a smooth edge must move by the same distance.
pub fn offset_faces(body: &Body, distances: &[f64]) -> Result<Body> {
    body.needs_solid("an offset")?;
    if distances.len() != body.faces.len() {
        return Err(refuse(format!("{} distances for {} faces", distances.len(), body.faces.len())));
    }
    if distances.iter().any(|d| !d.is_finite() || d.abs() > MAX_COORD) {
        return Err(refuse("an offset distance must be a sensible number".to_string()));
    }
    offset_within(body, distances, 0)
}

/// How many times over a part gets taken apart, and how many ways of doing that get tried each time.
const MOST_DEPTH: usize = 6;
const MOST_FEATURES: usize = 6;

fn offset_within(body: &Body, distances: &[f64], depth: usize) -> Result<Body> {
    match offset_directly(body, distances) {
        Ok(done) => Ok(done),
        Err(why) => {
            if depth >= MOST_DEPTH {
                return Err(why);
            }
            taken_apart(body, distances, depth).map_err(|_| why)
        }
    }
}

/// The faces joined to one by edges of one kind, convex or concave, and by seams and smooth edges: a pocket's faces by the
/// concave edges among them, and a boss's by the convex ones.
fn feature_from(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], start: usize, convex: bool) -> Vec<usize> {
    let mut group = vec![start];
    let mut k = 0;
    while k < group.len() {
        let f = group[k];
        k += 1;
        for c in body.faces[f].loops.iter().flatten() {
            for u in &uses[c.edge] {
                if u.0 != f && !group.contains(&u.0) && edge_kind(body, uses, c.edge).is_none_or(|kind| kind == convex) {
                    group.push(u.0);
                }
            }
        }
    }
    group.sort_unstable();
    group
}

/// Offsets a part by offsetting two simpler parts and combining them, as the module's notes describe.
fn taken_apart(body: &Body, distances: &[f64], depth: usize) -> Result<Body> {
    let uses = body.edge_uses();
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for f in 0..body.faces.len() {
        for convex in [false, true] {
            let group = feature_from(body, &uses, f, convex);
            if group.len() < body.faces.len() && !groups.contains(&group) {
                groups.push(group);
            }
        }
    }
    // The smallest first, as a small feature is the likeliest to be what's crossing, and the cheapest to try.
    let size = |g: &Vec<usize>| g.iter().map(|&f| body.face_aabb(f).diagonal()).fold(0.0f64, f64::max);
    groups.sort_by(|a, b| size(a).total_cmp(&size(b)));
    let mut tried = 0;
    let mut last = None;
    for group in &groups {
        // Whether this is a feature at all is for delete face to say.
        let Ok((plain, from)) = delete_faces_mapped(body, group) else { continue };
        tried += 1;
        if tried > MOST_FEATURES {
            break;
        }
        match with_feature_apart(body, distances, depth, group, &plain, &from) {
            Ok(done) => return Ok(done),
            Err(why) => last = Some(why),
        }
    }
    Err(last.unwrap_or_else(|| refuse("nothing in this part can be taken apart to get past where its walls cross".to_string())))
}

fn with_feature_apart(body: &Body, distances: &[f64], depth: usize, group: &[usize], plain: &Body, from: &[usize]) -> Result<Body> {
    let (with, without) = (crate::mass::mass_properties(body).volume, crate::mass::mass_properties(plain).volume);
    // Filling it in made the part bigger, so it's a pocket. Otherwise it's a boss.
    let pocket = without > with;
    let feature = if pocket { subtract(plain, body)? } else { subtract(body, plain)? };
    let index = Index::new(body);
    let plain_index = Index::new(plain);
    let feature_index = Index::new(&feature);
    let tol = 1e-6;
    // Each face of the feature is one of the faces that went, or a cap across where it meets the rest of the part.
    let mut moves = Vec::with_capacity(feature.faces.len());
    for f in 0..feature.faces.len() {
        let samples = samples_across(&feature, &feature_index, f);
        let middle = samples.iter().rev().copied().find(|q| feature_index.locate_on_face(f, *q, tol) == FaceLoc::Inside).or(samples.first().copied());
        let Some(q) = middle else {
            return Err(refuse("a feature's face has no points to tell it by".to_string()));
        };
        let wall = group.iter().copied().find(|&g| index.locate_on_face(g, q, tol) != FaceLoc::Outside);
        moves.push(match wall {
            // It faces the other way from the part's, so what moves the part's out moves this in, for a pocket.
            Some(g) => {
                if pocket {
                    -distances[g]
                } else {
                    distances[g]
                }
            }
            // A cap stays out of the way of the plain part as that moves, flush with it if it comes that way.
            None => {
                let on = (0..plain.faces.len()).find(|&h| plain_index.locate_on_face(h, q, tol) != FaceLoc::Outside);
                let d = on.map_or(0.0, |h| distances[from[h]]);
                if pocket {
                    d.max(0.0)
                } else {
                    (-d).max(0.0)
                }
            }
        });
    }
    let plain_moves: Vec<f64> = from.iter().map(|&f| distances[f]).collect();
    let moved_plain = offset_within(plain, &plain_moves, depth + 1)?;
    let moved_feature = offset_within(&feature, &moves, depth + 1)?;
    let result = if pocket { subtract(&moved_plain, &moved_feature)? } else { crate::boolean::union(&moved_plain, &moved_feature)? };
    if result.faces.is_empty() {
        return Err(refuse("moving this far leaves nothing of the part".to_string()));
    }
    validate(&result).map_err(|defects| refuse(format!("the result isn't a valid solid: {}", defects.join("; "))))?;
    meshes_closed(&result)?;
    // Every point of the result's surface is still at least the distance from the part's, if everything moved one way.
    let d0 = distances[0];
    if distances.iter().all(|d| *d == d0) && d0 != 0.0 {
        stays_clear(body, &result, d0.abs())?;
    }
    Ok(result)
}

/// That a result is nowhere nearer a part's surface than this.
fn stays_clear(base: &Body, result: &Body, reach: f64) -> Result<()> {
    let Ok(mesh) = crate::tess::tessellate(result, &crate::tess::TessParams::draft_for_size(result.aabb().diagonal())) else {
        return Ok(());
    };
    let index = Index::new(base);
    let stride = (mesh.positions.len() / 400).max(1);
    for q in mesh.positions.iter().step_by(stride) {
        if distance_to_surface(base, &index, *q) < reach - 1e-5 {
            return Err(refuse(format!("taking it apart left part of the result too near ({:.3}, {:.3}, {:.3})", q.x, q.y, q.z)));
        }
    }
    Ok(())
}

/// One way through, moving faces where they are, which is exact and keeps the most names.
fn offset_directly(body: &Body, distances: &[f64]) -> Result<Body> {
    // Moving out closes up what's concave, and moving in closes up what's convex.
    let closes = |f: usize, convex: bool| convex == (distances[f] < 0.0);
    surviving(
        body,
        &mut |current: &Body, from: &[usize]| {
            let distance = |f: usize| distances[from[f]];
            let mut surfaces = Vec::with_capacity(current.faces.len());
            let (mut gone, mut first) = (Vec::new(), None);
            for (i, f) in current.faces.iter().enumerate() {
                let d = distance(i);
                let reach = if matches!(f.surface, Surface::Extruded { .. } | Surface::Revolved { .. }) { boundary_samples(current, i) } else { Vec::new() };
                match moved_surface(&f.surface, if f.flipped { -d } else { d }, &reach) {
                    Ok(s) => surfaces.push(s),
                    Err(why) => {
                        gone.push(i);
                        first.get_or_insert(why);
                        surfaces.push(f.surface.clone());
                    }
                }
            }
            if let Some(why) = first {
                return Err(Stuck { why, faces: gone, edges: Vec::new() });
            }
            let uses = current.edge_uses();
            let map = |e: usize, p: Vec3| -> Result<Vec3> {
                let (f, g) = (uses[e][0].0, uses[e][1].0);
                if (distance(f) - distance(g)).abs() > TOL {
                    return Err(refuse("two faces that meet smoothly can't move by different distances".to_string()));
                }
                Ok(p + current.faces[f].outward_normal(p) * distance(f))
            };
            let done = with_surfaces(current, &surfaces, &map)?;
            walls_clear(current, &done, &distance)?;
            meshes_closed(&done)?;
            Ok(done)
        },
        &closes,
        &|result: &Body, f: usize, points: &[Vec3]| redundant(result, distances[f], points),
    )
}

/// Moves every face outward by `distance`, or inward if negative.
pub fn offset(body: &Body, distance: f64) -> Result<Body> {
    if distance.abs() < MIN_FEATURE {
        return Err(refuse(format!("an offset of {distance} is too small")));
    }
    offset_faces(body, &vec![distance; body.faces.len()])
}

/// Hollows a body to walls of `thickness`, inward from every face, leaving the listed faces open.
/// Inner faces are named for feature `feature`, with the slot of the face they follow.
pub fn shell(body: &Body, open: &[usize], thickness: f64, feature: u32) -> Result<Body> {
    body.needs_solid("a shell")?;
    if !(thickness >= MIN_FEATURE) {
        return Err(refuse(format!("a wall thickness of {thickness} is too small")));
    }
    if let Some(&bad) = open.iter().find(|&&f| f >= body.faces.len()) {
        return Err(refuse(format!("there is no face {bad}")));
    }
    // Open faces move out, so that the cavity passes cleanly through them.
    let distances: Vec<f64> = (0..body.faces.len()).map(|f| if open.contains(&f) { thickness } else { -thickness }).collect();
    let mut inner = offset_faces(body, &distances)?;
    // The cavity has to be inside the part wherever it's closed. Faces can have gone along the way, so it's by name.
    let open_names: Vec<FaceTag> = open.iter().map(|&f| body.faces[f].tag).collect();
    let index = Index::new(body);
    for (v, p) in inner.vertices.iter().enumerate() {
        let touches_open = inner.faces.iter().any(|face| open_names.contains(&face.tag) && face.loops.iter().flatten().any(|c| {
            let e = &inner.edges[c.edge];
            e.v0 == v || e.v1 == v
        }));
        if !touches_open && index.locate_in_body(*p) != BodyLoc::Inside {
            return Err(refuse(format!("walls of {thickness} don't fit: the cavity would reach outside the part")));
        }
    }
    for f in &mut inner.faces {
        // The slot alone can't say which face this follows, since two features can use the same slot.
        if f.origin.between.is_empty() {
            f.origin.between = vec![f.tag];
        }
        f.tag.feature = feature;
    }
    let result = subtract(body, &inner)?;
    // A closed shell's volume has to add up, which is worth checking on what's quick to check. Free-form faces take
    // integrating three times over for it, and those have the Boolean's own checks behind them.
    let quick = !body.faces.iter().any(|f| matches!(f.surface, Surface::Free { .. } | Surface::Extruded { .. } | Surface::Revolved { .. }));
    if open.is_empty() && quick {
        let (whole, cavity, left) = (crate::mass::mass_properties(body).volume, crate::mass::mass_properties(&inner).volume, crate::mass::mass_properties(&result).volume);
        if (whole - cavity - left).abs() > 1e-6 * whole.abs().max(1.0) {
            return Err(Error::Internal(format!("the shell's volume doesn't add up: {whole} - {cavity} isn't {left}")));
        }
    }
    Ok(result)
}

/// Tapers faces along a pull direction: planes parallel to it tilt about their line on the neutral plane,
/// and cylinders along it become cones. A positive angle makes the part narrower going along the pull, and
/// a hole wider, so that it releases from a mould drawn that way.
pub fn draft(body: &Body, faces: &[usize], pull: Vec3, neutral: Vec3, angle: f64) -> Result<Body> {
    body.needs_solid("a draft")?;
    if !(pull.norm() > 1e-12) {
        return Err(refuse("a pull direction must have a non-zero length".to_string()));
    }
    if !(angle.abs() > 1e-9 && angle.abs() < 80f64.to_radians()) {
        return Err(refuse(format!("a draft of {:.3} degrees is out of range", angle.to_degrees())));
    }
    let pull = pull.normalized();
    for &i in faces {
        if i >= body.faces.len() {
            return Err(refuse(format!("there is no face {i}")));
        }
    }
    // A fillet touching a drafted face smoothly can't stay as it is, as its radius would have to change along it. So it
    // comes off, the draft goes ahead, and it goes back on at the same radius between the same two faces.
    let uses = body.edge_uses();
    let mut in_the_way: Vec<usize> = Vec::new();
    for &f in faces {
        for c in body.faces[f].loops.iter().flatten() {
            let [a, b] = uses[c.edge].as_slice() else { continue };
            let other = if a.0 == f { b.0 } else { a.0 };
            let g = &body.faces[other];
            if faces.contains(&other) || g.origin.between.len() != 2 || blend_radius(&g.surface).is_none() || edge_kind(body, &uses, c.edge).is_some() {
                continue;
            }
            // All of that blend.
            for (h, face) in body.faces.iter().enumerate() {
                if face.tag.feature == g.tag.feature && !faces.contains(&h) && !in_the_way.contains(&h) && blend_radius(&face.surface).is_some() {
                    in_the_way.push(h);
                }
            }
        }
    }
    if in_the_way.is_empty() {
        let from: Vec<usize> = (0..body.faces.len()).collect();
        return draft_once(body, &from, faces, pull, neutral, angle).map_err(|stuck| stuck.why);
    }
    let blends: Vec<(u32, f64, Vec<FaceTag>)> = in_the_way.iter().map(|&h| (body.faces[h].tag.feature, blend_radius(&body.faces[h].surface).unwrap_or(0.0), body.faces[h].origin.between.clone())).collect();
    let (bare, from) = delete_faces_mapped(body, &in_the_way)?;
    let drafted = draft_once(&bare, &from, faces, pull, neutral, angle).map_err(|stuck| stuck.why)?;
    let mut done = drafted;
    let mut todo = blends;
    todo.sort_by(|x, y| (x.0, x.1).partial_cmp(&(y.0, y.1)).unwrap_or(std::cmp::Ordering::Equal));
    let mut k = 0;
    while k < todo.len() {
        let (feature, radius) = (todo[k].0, todo[k].1);
        let uses = done.edge_uses();
        let pairs: Vec<&Vec<FaceTag>> = todo.iter().filter(|t| t.0 == feature && (t.1 - radius).abs() < 1e-9 && t.2.len() == 2).map(|t| &t.2).collect();
        let edges: Vec<usize> = (0..done.edges.len())
            .filter(|&e| {
                let [a, b] = uses[e].as_slice() else { return false };
                let (x, y) = (done.faces[a.0].tag, done.faces[b.0].tag);
                pairs.iter().any(|p| (p[0] == x && p[1] == y) || (p[0] == y && p[1] == x))
            })
            .collect();
        if !edges.is_empty() {
            done = crate::blend::fillet_with(&done, &edges, radius, crate::blend::BlendOptions { propagate: false }, feature)?;
        }
        while k < todo.len() && todo[k].0 == feature && (todo[k].1 - radius).abs() < 1e-9 {
            k += 1;
        }
    }
    Ok(done)
}

/// A blend's radius, if this is a surface a fillet makes with one.
fn blend_radius(s: &Surface) -> Option<f64> {
    match *s {
        Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => Some(radius),
        Surface::Torus { minor, .. } => Some(minor),
        _ => None,
    }
}

/// One try at a draft, with `from` saying which face each of these started as.
fn draft_once(body: &Body, from: &[usize], faces: &[usize], pull: Vec3, neutral: Vec3, angle: f64) -> std::result::Result<Body, Stuck> {
    let drafted: Vec<bool> = from.iter().map(|f| faces.contains(f)).collect();
    let mut surfaces: Vec<Surface> = body.faces.iter().map(|f| f.surface.clone()).collect();
    for (i, face) in body.faces.iter().enumerate() {
        if !drafted[i] {
            continue;
        }
        surfaces[i] = match face.surface {
            Surface::Plane(frame) => {
                let out = if face.flipped { -frame.z } else { frame.z };
                if out.dot(pull).abs() > 1e-9 {
                    return Err(refuse("a planar face can only be drafted if it runs along the pull direction".to_string()).into());
                }
                // On the neutral plane the face stays put: it turns about the line it meets that plane in.
                let on_neutral = frame.origin - pull * (frame.origin - neutral).dot(pull);
                let axis = pull.cross(out);
                // Going along the pull, a positive angle moves the outward-facing plane inward.
                let spin = Frame::rotation(on_neutral, axis, -angle);
                let z = spin.dir_to_world(frame.z);
                Surface::Plane(Frame::new(on_neutral, z, frame.x))
            }
            Surface::Cylinder { frame, radius } => {
                if frame.z.cross(pull).norm() > 1e-9 {
                    return Err(refuse("a cylindrical face can only be drafted along its axis".to_string()).into());
                }
                let origin = frame.origin + frame.z * (neutral - frame.origin).dot(frame.z);
                // A cone widens along its z, so a part's wall narrowing along the pull has z against it.
                let z = if face.flipped == (angle > 0.0) { pull } else { -pull };
                let half_angle = angle.abs();
                Surface::Cone { frame: Frame::new(origin, z, frame.x), radius, half_angle }
            }
            _ => return Err(Error::Unsupported("only planes and cylinders can be drafted".into()).into()),
        };
    }
    // A smooth edge between drafted faces is a straight ruling. It keeps its point on the neutral plane and
    // tilts the way its faces do.
    let uses = body.edge_uses();
    let map = |e: usize, p: Vec3| -> Result<Vec3> {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (fd, gd) = (drafted[f], drafted[g]);
        if fd != gd {
            return Err(refuse("a face can't be drafted while a face it meets smoothly isn't: select them both".to_string()));
        }
        if !fd {
            return Ok(p);
        }
        if !matches!(body.edges[e].curve, Curve::Line { .. }) {
            return Err(Error::Unsupported("a smooth edge between drafted faces must be straight".into()));
        }
        let out = body.faces[f].outward_normal(p);
        let along = (p - neutral).dot(pull);
        Ok(p - out * (along * angle.tan()))
    };
    let done = with_surfaces(body, &surfaces, &map)?;
    meshes_closed(&done)?;
    Ok(done)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn moved_surfaces_are_distance_d_away() {
        let f = Frame::new(v3(1.0, 2.0, 3.0), v3(0.3, 0.2, 1.0), Vec3::X);
        let list = [
            Surface::Plane(f),
            Surface::Cylinder { frame: f, radius: 7.0 },
            Surface::Cone { frame: f, radius: 7.0, half_angle: 0.4 },
            Surface::Sphere { frame: f, radius: 7.0 },
            Surface::Torus { frame: f, major: 12.0, minor: 3.0 },
        ];
        let mut rng = Rng::new(5);
        for s in &list {
            for d in [1.5, -1.5] {
                let t = moved_surface(s, d, &[]).unwrap();
                for _ in 0..200 {
                    let raw = f.to_world(v3(rng.range(-9.0, 9.0), rng.range(-9.0, 9.0), rng.range(1.0, 9.0)));
                    let p = s.project(raw);
                    if s.sdf(p).abs() > 1e-9 {
                        continue;
                    }
                    let moved = p + s.normal(p) * d;
                    assert!(t.sdf(moved).abs() < 1e-9, "{s:?} by {d}: {}", t.sdf(moved));
                }
            }
        }
    }
}
