//! Point queries: where is a point relative to a face, and relative to a body?

use super::roots::{curve_surface_contact, Contact};
use crate::curve::Curve;
use crate::math::*;
use crate::surface::{Chart, Surface};
use crate::topo::{Body, Edge};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceLoc {
    Inside,
    Boundary,
    Outside,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyLoc {
    Inside,
    Outside,
    /// On the boundary, on this face.
    OnFace(usize),
}

/// A body with its bounding boxes cached.
pub struct Index<'a> {
    pub body: &'a Body,
    pub edge_boxes: Vec<Aabb>,
    pub face_boxes: Vec<Aabb>,
    /// Chart bounds of each loop of each face. A face can have hundreds of holes, and a
    /// point outside a closed loop's bounds gets no winding from it and is near none of
    /// its edges, so most loops can be skipped without touching their edges.
    pub loop_boxes: Vec<Vec<(Vec2, Vec2)>>,
    pub bounds: Aabb,
}

/// Bounds of a loop in its face's chart, with a margin for curve between samples.
fn loop_bounds(body: &Body, f: usize, l: usize) -> (Vec2, Vec2) {
    let face = &body.faces[f];
    let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for &c in &face.loops[l] {
        let e = &body.edges[c.edge];
        let samples = if matches!(e.curve, Curve::Line { .. }) { 1 } else { 8 };
        for k in 0..=samples {
            let t = e.t0 + (e.t1 - e.t0) * k as f64 / samples as f64;
            let uv = face.surface.to_chart(&face.chart, e.curve.eval(t));
            lo = v2(lo.x.min(uv.x), lo.y.min(uv.y));
            hi = v2(hi.x.max(uv.x), hi.y.max(uv.y));
        }
    }
    let margin = (hi - lo).norm() * 0.15;
    (v2(lo.x - margin, lo.y - margin), v2(hi.x + margin, hi.y + margin))
}

/// Winding angle of an edge around a chart point, refined near the point so that
/// the polygon approximation can never put the point on the wrong side.
pub(crate) fn edge_winding(surface: &Surface, chart: &Chart, e: &Edge, reversed: bool, target: Vec2) -> f64 {
    let at = |t: f64| surface.to_chart(chart, e.curve.eval(t));
    fn piece(at: &impl Fn(f64) -> Vec2, ta: f64, tb: f64, a: Vec2, b: Vec2, target: Vec2, depth: u32) -> f64 {
        let tm = (ta + tb) / 2.0;
        let m = at(tm);
        let reach = a.dist(m).max(b.dist(m));
        if target.dist(m) > 1.5 * reach || depth >= 44 {
            let (u, v) = (a - target, b - target);
            u.cross(v).atan2(u.dot(v))
        } else {
            piece(at, ta, tm, a, m, target, depth + 1) + piece(at, tm, tb, m, b, target, depth + 1)
        }
    }
    let straight = matches!(e.curve, Curve::Line { .. }) && matches!(surface, Surface::Plane(_));
    let pieces = if straight {
        1
    } else {
        match &e.curve {
            Curve::Sampled(_) => ((e.t1 - e.t0).ceil() as usize).clamp(1, 256),
            _ => 8,
        }
    };
    let dt = (e.t1 - e.t0) / pieces as f64;
    let mut total = 0.0;
    for i in 0..pieces {
        let (ta, tb) = (e.t0 + dt * i as f64, e.t0 + dt * (i + 1) as f64);
        total += if straight {
            let (u, v) = (at(ta) - target, at(tb) - target);
            u.cross(v).atan2(u.dot(v))
        } else {
            piece(&at, ta, tb, at(ta), at(tb), target, 0)
        };
    }
    if reversed {
        -total
    } else {
        total
    }
}

impl<'a> Index<'a> {
    pub fn new(body: &'a Body) -> Index<'a> {
        let edge_boxes: Vec<Aabb> = body.edges.iter().map(|e| e.aabb(&body.vertices)).collect();
        let face_boxes: Vec<Aabb> = (0..body.faces.len()).map(|f| body.face_aabb(f)).collect();
        let mut bounds = Aabb::EMPTY;
        for b in &face_boxes {
            bounds.union(b);
        }
        let loop_boxes = (0..body.faces.len())
            .map(|f| (0..body.faces[f].loops.len()).map(|l| loop_bounds(body, f, l)).collect())
            .collect();
        Index { body, edge_boxes, face_boxes, loop_boxes, bounds }
    }

    /// Locates a point relative to a face. A point farther than `tol` from the
    /// surface is outside.
    pub fn locate_on_face(&self, f: usize, p: Vec3, tol: f64) -> FaceLoc {
        let face = &self.body.faces[f];
        if face.surface.sdf(p).abs() > tol || !self.face_boxes[f].inflated(tol).contains(p) {
            return FaceLoc::Outside;
        }
        let target = face.surface.to_chart(&face.chart, p);
        let near = 16.0 * tol;
        let in_box = |l: usize| {
            let (lo, hi) = self.loop_boxes[f][l];
            target.x >= lo.x - near && target.x <= hi.x + near && target.y >= lo.y - near && target.y <= hi.y + near
        };
        // Outside the outer loop's bounds means outside the face.
        if !in_box(0) {
            return FaceLoc::Outside;
        }
        let mut winding = 0.0;
        for (l, lp) in face.loops.iter().enumerate() {
            if l > 0 && !in_box(l) {
                continue;
            }
            for c in lp {
                if self.edge_boxes[c.edge].inflated(tol).contains(p) {
                    let e = &self.body.edges[c.edge];
                    if e.curve.closest(p, e.t0, e.t1).1 <= tol {
                        return FaceLoc::Boundary;
                    }
                }
            }
            for c in lp {
                winding += edge_winding(&face.surface, &face.chart, &self.body.edges[c.edge], c.reversed, target);
            }
        }
        let turns = winding / TAU * if face.flipped { -1.0 } else { 1.0 };
        if turns > 0.5 {
            FaceLoc::Inside
        } else {
            FaceLoc::Outside
        }
    }

    /// Locates a point relative to the whole body by casting rays. A ray is only
    /// trusted when it crosses faces cleanly. If it grazes a surface or hits an
    /// edge it is thrown away and another direction is tried.
    pub fn locate_in_body(&self, p: Vec3) -> BodyLoc {
        for f in 0..self.body.faces.len() {
            if self.face_boxes[f].inflated(TOL * 2.0).contains(p)
                && self.body.faces[f].surface.sdf(p).abs() <= TOL * 1.5
                && self.locate_on_face(f, p, TOL * 1.5) != FaceLoc::Outside
            {
                return BodyLoc::OnFace(f);
            }
        }
        if !self.bounds.inflated(TOL * 2.0).contains(p) {
            return BodyLoc::Outside;
        }
        let mut rng = Rng::new(0x5EED);
        let reach = self.bounds.diagonal() + 1.0;
        for _ in 0..64 {
            let d = rng.unit_vector();
            if let Some(inside) = self.cast_ray(p, d, reach) {
                return if inside { BodyLoc::Inside } else { BodyLoc::Outside };
            }
        }
        // Extremely unlikely: every random direction hit an edge or grazed a face.
        BodyLoc::Outside
    }

    fn cast_ray(&self, p: Vec3, d: Vec3, reach: f64) -> Option<bool> {
        let mut inside = false;
        for f in 0..self.body.faces.len() {
            if !ray_hits_box(p, d, &self.face_boxes[f].inflated(TOL * 2.0)) {
                continue;
            }
            let surface = &self.body.faces[f].surface;
            for t in ray_surface(surface, p, d, reach) {
                if t <= 0.0 {
                    continue;
                }
                let q = p + d * t;
                match self.locate_on_face(f, q, TOL * 2.0) {
                    FaceLoc::Outside => {}
                    FaceLoc::Boundary => return None,
                    FaceLoc::Inside => {
                        if surface.normal(q).dot(d).abs() < 1e-4 {
                            return None;
                        }
                        inside = !inside;
                    }
                }
            }
        }
        Some(inside)
    }
}

fn ray_hits_box(p: Vec3, d: Vec3, b: &Aabb) -> bool {
    let (mut lo, mut hi) = (0.0f64, f64::INFINITY);
    for (o, v, min, max) in [(p.x, d.x, b.min.x, b.max.x), (p.y, d.y, b.min.y, b.max.y), (p.z, d.z, b.min.z, b.max.z)] {
        if v.abs() < 1e-300 {
            if o < min || o > max {
                return false;
            }
        } else {
            let (mut a, mut c) = ((min - o) / v, (max - o) / v);
            if a > c {
                std::mem::swap(&mut a, &mut c);
            }
            lo = lo.max(a);
            hi = hi.min(c);
            if lo > hi {
                return false;
            }
        }
    }
    true
}

/// Distances along a ray to a surface. Closed form where that is simple.
fn ray_surface(s: &Surface, p: Vec3, d: Vec3, reach: f64) -> Vec<f64> {
    let quadratic = |a: f64, b: f64, c: f64| -> Vec<f64> {
        if a.abs() < 1e-300 {
            return Vec::new();
        }
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return Vec::new();
        }
        let root = disc.sqrt();
        let q = -0.5 * (b + b.signum() * root);
        let mut out = vec![q / a];
        if q != 0.0 {
            out.push(c / q);
        }
        out
    };
    match *s {
        Surface::Plane(f) => {
            let denom = d.dot(f.z);
            if denom == 0.0 {
                Vec::new()
            } else {
                vec![(f.origin - p).dot(f.z) / denom]
            }
        }
        Surface::Cylinder { frame, radius } => {
            let l = frame.to_local(p);
            let v = v3(d.dot(frame.x), d.dot(frame.y), 0.0);
            quadratic(v.x * v.x + v.y * v.y, 2.0 * (l.x * v.x + l.y * v.y), l.x * l.x + l.y * l.y - radius * radius)
        }
        Surface::Sphere { frame, radius } => {
            let l = p - frame.origin;
            quadratic(1.0, 2.0 * l.dot(d), l.norm2() - radius * radius)
        }
        _ => {
            let line = Curve::Line { origin: p, dir: d };
            match curve_surface_contact(&line, 0.0, reach, s, 0.0) {
                Contact::Roots(r) => r,
                Contact::OnSurface => Vec::new(),
            }
        }
    }
}
