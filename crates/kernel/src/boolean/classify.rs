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
    /// Chart positions already worked out along edges, by face, edge and parameter.
    chart_points: std::cell::RefCell<crate::hash::HashMap<(u32, u32, u64), Vec2>>,
    /// Whether each free-form face stays on its patch, so that the patch's own boxes can rule things out. One that runs off
    /// the end of it along its tangent, as a moved one can, doesn't.
    on_patch: Vec<bool>,
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
pub(crate) fn edge_winding(surface: &Surface, chart: &Chart, e: &Edge, reversed: bool, target: Vec2, memo: &mut dyn FnMut(f64, &dyn Fn(f64) -> Vec2) -> Vec2) -> f64 {
    let exact = |t: f64| surface.to_chart(chart, e.curve.eval(t));
    let memo = std::cell::RefCell::new(memo);
    let at = |t: f64| (memo.borrow_mut())(t, &exact);
    fn piece(at: &impl Fn(f64) -> Vec2, ta: f64, tb: f64, a: Vec2, b: Vec2, target: Vec2, depth: u32) -> f64 {
        let tm = (ta + tb) / 2.0;
        let m = at(tm);
        let reach = a.dist(m).max(b.dist(m));
        // By 20 halvings a curve is straight to far less than rounding, so a piece still reaching this far is a jump in the
        // chart and not a curve. A jump is one straight step, and refining it further would just cost time.
        if target.dist(m) > 1.5 * reach || depth >= 20 {
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
    // Where an edge runs into a pole of a free-form surface, its chart position jumps along the side that shrank to a point,
    // however close it gets. No amount of refining can close that up, so that last sliver is one straight step in the chart.
    let at_pole = |t: f64| match surface {
        Surface::Free { frame, patch } => patch.at_pole(frame.to_local(e.curve.eval(t))),
        _ => false,
    };
    let sliver = 1e-6 * (e.t1 - e.t0);
    let (from, to) = (if at_pole(e.t0) { e.t0 + sliver } else { e.t0 }, if at_pole(e.t1) { e.t1 - sliver } else { e.t1 });
    let dt = (to - from) / pieces as f64;
    let chord = |p: Vec2, q: Vec2| {
        let (u, v) = (p - target, q - target);
        u.cross(v).atan2(u.dot(v))
    };
    let mut total = 0.0;
    if from > e.t0 {
        total += chord(at(e.t0), at(from));
    }
    if to < e.t1 {
        total += chord(at(to), at(e.t1));
    }
    for i in 0..pieces {
        let (ta, tb) = (from + dt * i as f64, from + dt * (i + 1) as f64);
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
        let on_patch = body
            .faces
            .iter()
            .zip(&face_boxes)
            .map(|(face, b)| match &face.surface {
                Surface::Free { frame, patch } => {
                    let room = 1e-6 * b.diagonal().max(1.0);
                    let inside = |q: Vec3| {
                        let l = frame.to_local(q);
                        patch.boxes_hold(l, room)
                    };
                    face.loops.iter().flatten().all(|c| (0..=4).all(|k| inside(body.coedge_point(*c, k as f64 / 4.0))))
                }
                _ => false,
            })
            .collect();
        Index { body, edge_boxes, face_boxes, loop_boxes, bounds, chart_points: Default::default(), on_patch }
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
                    if e.curve.could_be_within(p, e.t0, e.t1, tol) && e.curve.closest(p, e.t0, e.t1).1 <= tol {
                        return FaceLoc::Boundary;
                    }
                }
            }
            for c in lp {
                // Each point of an edge is worth working out once. A curve on a free-form surface costs a search for each, and
                // the next point looked up will want the same ones again.
                let mut known = self.chart_points.borrow_mut();
                let mut memo = |t: f64, exact: &dyn Fn(f64) -> Vec2| *known.entry((f as u32, c.edge as u32, t.to_bits())).or_insert_with(|| exact(t));
                winding += edge_winding(&face.surface, &face.chart, &self.body.edges[c.edge], c.reversed, target, &mut memo);
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
    /// Where a curve meets a face's surface, looking only where a free-form face's own boxes say that it could. A long face's
    /// one bounding box says very little, and each distance on it costs a search.
    pub(crate) fn edge_meets_face(&self, f: usize, curve: &Curve, t0: f64, t1: f64, tol: f64) -> Contact {
        let surface = &self.body.faces[f].surface;
        if let (Surface::Free { frame, patch }, true, false) = (surface, self.on_patch[f], matches!(curve, Curve::Sampled(_))) {
            let spans = patch.curve_spans(&|t| frame.to_local(curve.eval(t)), t0, t1, curve.speed_bound(), tol + TOL * 4.0);
            let mut found = Vec::new();
            for (a, b) in spans {
                match curve_surface_contact(curve, a, b, surface, tol) {
                    Contact::Roots(list) => found.extend(list),
                    // Lying on it is for the whole curve to say.
                    Contact::OnSurface => return curve_surface_contact(curve, t0, t1, surface, tol),
                }
            }
            found.sort_by(f64::total_cmp);
            return Contact::Roots(found);
        }
        curve_surface_contact(curve, t0, t1, surface, tol)
    }

    /// False if a free-form face's own boxes show that a point isn't on it, which is much quicker than finding out how far
    /// off it is. A long face's one bounding box is too big to rule out much.
    fn could_hold(&self, f: usize, p: Vec3) -> bool {
        match &self.body.faces[f].surface {
            Surface::Free { frame, patch } if self.on_patch[f] => patch.boxes_hold(frame.to_local(p), TOL * 4.0),
            _ => true,
        }
    }

    pub fn locate_in_body(&self, p: Vec3) -> BodyLoc {
        for f in 0..self.body.faces.len() {
            if self.face_boxes[f].inflated(TOL * 2.0).contains(p)
                && self.could_hold(f, p)
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
            // A free-form face that stays on its patch has its crossings solved for. If they can't be told from here, as when
            // a ray runs along it, a different ray will do. A moved one is traced, only where its cells' boxes say.
            let hits: Vec<f64> = match surface {
                Surface::Free { frame, patch } if self.on_patch[f] => {
                    let local = v3(d.dot(frame.x), d.dot(frame.y), d.dot(frame.z));
                    if patch.offset() == 0.0 {
                        patch.ray_crossings(frame.to_local(p), local, reach)?
                    } else {
                        let mut found = Vec::new();
                        for (a, b) in patch.ray_spans(frame.to_local(p), local, TOL * 4.0) {
                            let (a, b) = (a.max(0.0), b.min(reach));
                            if a <= b {
                                found.extend(ray_surface_between(surface, p, d, (a, b)));
                            }
                        }
                        found
                    }
                }
                _ => ray_surface(surface, p, d, reach),
            };
            for t in hits {
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
/// Where a ray from `p` along `d` meets a free-form surface, looking only between these distances.
fn ray_surface_between(s: &Surface, p: Vec3, d: Vec3, span: (f64, f64)) -> Vec<f64> {
    let ray = Curve::Line { origin: p, dir: d };
    match curve_surface_contact(&ray, span.0, span.1, s, 0.0) {
        Contact::Roots(list) => list,
        Contact::OnSurface => Vec::new(),
    }
}

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
