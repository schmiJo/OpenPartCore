//! Offset, shell and draft, which all change a body's surfaces and keep its topology.
//!
//! Each face gets a new surface. Then every vertex is found again where its new surfaces meet, and
//! every edge is rebuilt between its new vertices. All five surface types offset to their own kind, and
//! a drafted cylinder is a cone, so the result is as exact as the original. If a change is big enough to
//! alter the topology, as when an offset makes a face vanish, it is refused: the checks below catch an
//! edge that has turned round or a face that no longer closes.

use crate::boolean::classify::{BodyLoc, Index};
use crate::boolean::ssi::{self, Ssi};
use crate::boolean::{subtract};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::Patch;
use crate::spline::Spline;
use crate::surface::{refine_on_two, Surface};
use crate::topo::{Body, Edge};
use crate::validate::validate;
use std::sync::Arc;

/// How much two faces' normals can differ along an edge and still count as tangent.
const TANGENT: f64 = 1e-7;

fn refuse(what: String) -> Error {
    Error::InvalidInput(what)
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
fn with_surfaces(body: &Body, surfaces: &[Surface], tangent_map: TangentMap<'_>) -> Result<Body> {
    let uses = body.edge_uses();
    // What each edge is between.
    let mut tangent = vec![false; body.edges.len()];
    let mut same = vec![false; body.edges.len()];
    let size = body.aabb().diagonal().max(1.0);
    for (e, edge) in body.edges.iter().enumerate() {
        if uses[e].len() != 2 {
            return Err(Error::Unsupported("an edge used by other than two faces can't be moved".into()));
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
            None => point_on_surfaces(&list, start).ok_or_else(|| refuse("the new surfaces no longer meet at a corner".to_string()))?,
        };
        if list.iter().any(|s| s.sdf(vertices[v]).abs() > 10.0 * TOL) {
            return Err(refuse("the new surfaces no longer meet at a corner".to_string()));
        }
    }
    // Edges.
    let mut out = Body { vertices, ..Body::default() };
    for (e, edge) in body.edges.iter().enumerate() {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (a, b) = (out.vertices[edge.v0], out.vertices[edge.v1]);
        if a.dist(b) < MIN_FEATURE {
            return Err(refuse("an edge shrinks away to nothing: the change is too big for the part's shape".to_string()));
        }
        let (curve, t0, t1) = match &tangent_curves[e] {
            Some(c) => fitted(c, a, b, tangent_map(e, edge.eval_mid())?).filter(|f| f.0.eval(f.1).dist(a) <= TOL && f.0.eval(f.2).dist(b) <= TOL).or_else(|| spliced(c, a, b)).ok_or_else(|| refuse("a smooth edge didn't fit its new corners".to_string()))?,
            None => {
                let (c, t0, t1) = rebuilt_curve(edge, &surfaces[f], &surfaces[g], same[e], a, b)?;
                // The same goes for a curve traced on a moved surface: fitted once, within the tolerance, if it can be.
                let moved = [f, g].iter().any(|&h| matches!(&surfaces[h], Surface::Free { patch, .. } if patch.offset() != 0.0));
                if moved { sampled_as_spline(&c, t0, t1).unwrap_or((c, t0, t1)) } else { (c, t0, t1) }
            }
        };
        // An edge that has turned round means a face has turned inside out.
        let (old, new) = (edge.curve.eval(edge.t1) - edge.curve.eval(edge.t0), curve.eval(t1) - curve.eval(t0));
        if matches!(edge.curve, Curve::Line { .. }) && old.dot(new) <= 0.0 {
            return Err(refuse("an edge turns round: the change is too big for the part's shape".to_string()));
        }
        out.edges.push(Edge { curve, t0, t1, v0: edge.v0, v1: edge.v1 });
    }
    for (f, face) in body.faces.iter().enumerate() {
        let i = out.add_face(surfaces[f].clone(), face.flipped, face.loops.clone(), face.tag);
        out.faces[i].origin = face.origin.clone();
    }
    out.shells = body.shells.clone();
    validate(&out).map_err(|defects| refuse(format!("the result isn't a valid solid: {}", defects.join("; "))))?;
    let check = crate::tess::tessellate(&out, &crate::tess::TessParams::for_size(out.aabb().diagonal()))?;
    if check.open_edge_count() > 0 {
        return Err(refuse("the result doesn't mesh into a closed surface: the change is too big for the part's shape".to_string()));
    }
    Ok(out)
}

/// Moves each face outward by its own distance, negative for inward. Sharp edges stay sharp: neighbouring
/// faces simply meet further out or in. Faces that share a smooth edge must move by the same distance.
pub fn offset_faces(body: &Body, distances: &[f64]) -> Result<Body> {
    if distances.len() != body.faces.len() {
        return Err(refuse(format!("{} distances for {} faces", distances.len(), body.faces.len())));
    }
    if distances.iter().any(|d| !d.is_finite() || d.abs() > MAX_COORD) {
        return Err(refuse("an offset distance must be a sensible number".to_string()));
    }
    let surfaces = body
        .faces
        .iter()
        .zip(distances)
        .enumerate()
        .map(|(i, (f, d))| {
            let reach = if matches!(f.surface, Surface::Extruded { .. } | Surface::Revolved { .. }) { boundary_samples(body, i) } else { Vec::new() };
            moved_surface(&f.surface, if f.flipped { -*d } else { *d }, &reach)
        })
        .collect::<Result<Vec<Surface>>>()?;
    let uses = body.edge_uses();
    let map = |e: usize, p: Vec3| -> Result<Vec3> {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        if (distances[f] - distances[g]).abs() > TOL {
            return Err(refuse("two faces that meet smoothly can't move by different distances".to_string()));
        }
        Ok(p + body.faces[f].outward_normal(p) * distances[f])
    };
    with_surfaces(body, &surfaces, &map)
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
    if !(thickness >= MIN_FEATURE) {
        return Err(refuse(format!("a wall thickness of {thickness} is too small")));
    }
    if let Some(&bad) = open.iter().find(|&&f| f >= body.faces.len()) {
        return Err(refuse(format!("there is no face {bad}")));
    }
    // Open faces move out, so that the cavity passes cleanly through them.
    let distances: Vec<f64> = (0..body.faces.len()).map(|f| if open.contains(&f) { thickness } else { -thickness }).collect();
    let mut inner = offset_faces(body, &distances)?;
    // The cavity has to be inside the part wherever it's closed.
    let index = Index::new(body);
    for (v, p) in inner.vertices.iter().enumerate() {
        let touches_open = inner.faces.iter().enumerate().any(|(f, face)| open.contains(&f) && face.loops.iter().flatten().any(|c| {
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
    let (whole, cavity, left) = (crate::mass::mass_properties(body).volume, crate::mass::mass_properties(&inner).volume, crate::mass::mass_properties(&result).volume);
    if open.is_empty() && (whole - cavity - left).abs() > 1e-6 * whole.abs().max(1.0) {
        return Err(Error::Internal(format!("the shell's volume doesn't add up: {whole} - {cavity} isn't {left}")));
    }
    Ok(result)
}

/// Tapers faces along a pull direction: planes parallel to it tilt about their line on the neutral plane,
/// and cylinders along it become cones. A positive angle makes the part narrower going along the pull, and
/// a hole wider, so that it releases from a mould drawn that way.
pub fn draft(body: &Body, faces: &[usize], pull: Vec3, neutral: Vec3, angle: f64) -> Result<Body> {
    if !(pull.norm() > 1e-12) {
        return Err(refuse("a pull direction must have a non-zero length".to_string()));
    }
    if !(angle.abs() > 1e-9 && angle.abs() < 80f64.to_radians()) {
        return Err(refuse(format!("a draft of {:.3} degrees is out of range", angle.to_degrees())));
    }
    let pull = pull.normalized();
    let mut surfaces: Vec<Surface> = body.faces.iter().map(|f| f.surface.clone()).collect();
    for &i in faces {
        let face = body.faces.get(i).ok_or_else(|| refuse(format!("there is no face {i}")))?;
        surfaces[i] = match face.surface {
            Surface::Plane(frame) => {
                let out = if face.flipped { -frame.z } else { frame.z };
                if out.dot(pull).abs() > 1e-9 {
                    return Err(refuse("a planar face can only be drafted if it runs along the pull direction".to_string()));
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
                    return Err(refuse("a cylindrical face can only be drafted along its axis".to_string()));
                }
                let origin = frame.origin + frame.z * (neutral - frame.origin).dot(frame.z);
                // A cone widens along its z, so a part's wall narrowing along the pull has z against it.
                let z = if face.flipped == (angle > 0.0) { pull } else { -pull };
                let half_angle = angle.abs();
                Surface::Cone { frame: Frame::new(origin, z, frame.x), radius, half_angle }
            }
            _ => return Err(Error::Unsupported("only planes and cylinders can be drafted".into())),
        };
    }
    // A smooth edge between drafted faces is a straight ruling. It keeps its point on the neutral plane and
    // tilts the way its faces do.
    let uses = body.edge_uses();
    let map = |e: usize, p: Vec3| -> Result<Vec3> {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (fd, gd) = (faces.contains(&f), faces.contains(&g));
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
    with_surfaces(body, &surfaces, &map)
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
