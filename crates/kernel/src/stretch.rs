//! Stretching a body along one direction only.
//!
//! Every point moves along a unit direction `d` by `k - 1` times its height above the plane through `q` square to `d`, so
//! that plane stays put and everything else is `k` times as far from it. That's an affine map, and an affine map takes
//! any rational spline or patch exactly to the one with its control points mapped and its weights kept. So lines stay
//! lines, planes planes, and splines, extruded curves and free-form faces stay exact. A circle or ellipse becomes an
//! ellipse, or a circle again in a plane square to `d`. A cylinder, cone or turned curve whose axis is along `d` stays
//! one. One turned about any other axis, a sphere or a torus is no longer round, and becomes the free-form patch it
//! exactly is: its surface of revolution as a rational patch over the face's own range, mapped. A face moved by an
//! offset, whose stretch is no longer a surface moved by a distance, is refused.

use crate::curve::{Curve, SampledCurve};
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::Patch;
use crate::spline::{PlaneCurve, Spline};
use crate::surface::Surface;
use crate::topo::{Body, Edge};
use crate::validate::validate;
use std::sync::Arc;

fn refuse(why: String) -> Error {
    Error::InvalidInput(why)
}

/// `p + (k - 1) (d . (p - q)) d`, for a unit `d`.
#[derive(Clone, Copy, Debug)]
pub struct Stretch {
    pub q: Vec3,
    pub d: Vec3,
    pub k: f64,
}

impl Stretch {
    pub fn new(q: Vec3, d: Vec3, k: f64) -> Result<Stretch> {
        if !(k >= 1e-6 && k <= 1e6) {
            return Err(refuse(format!("a stretch has to be by a number above 0, from 1e-6 to 1e6, and {k} isn't")));
        }
        if !(d.norm() > 1e-12 && d.norm() < 1e150) {
            return Err(refuse("a stretch needs a direction to go along".into()));
        }
        Ok(Stretch { q, d: d.normalized(), k })
    }

    pub fn point(&self, p: Vec3) -> Vec3 {
        p + self.d * ((self.k - 1.0) * self.d.dot(p - self.q))
    }

    pub fn vector(&self, v: Vec3) -> Vec3 {
        v + self.d * ((self.k - 1.0) * self.d.dot(v))
    }

    /// Where a surface's normal goes: the inverse transpose, which for this symmetric map is the inverse.
    pub fn normal(&self, n: Vec3) -> Vec3 {
        (n + self.d * ((1.0 / self.k - 1.0) * self.d.dot(n))).normalized()
    }

    fn along(&self, z: Vec3) -> bool {
        z.dot(self.d).abs() >= 1.0 - 1e-12
    }

    /// The image of an edge's curve, with its new range. A sampled curve's surfaces come from `on`, given each and the
    /// curve's points.
    fn curve(&self, c: &Curve, t0: f64, t1: f64, on: &dyn Fn(&Surface, &[Vec3]) -> Result<Surface>) -> Result<(Curve, f64, f64)> {
        Ok(match c {
            Curve::Line { origin, dir } => {
                let w = self.vector(*dir);
                let s = w.norm();
                (Curve::Line { origin: self.point(*origin), dir: w / s }, t0 * s, t1 * s)
            }
            Curve::Circle { frame, radius } => {
                let (curve, shift) = conic(self.point(frame.origin), self.vector(frame.x) * *radius, self.vector(frame.y) * *radius);
                (curve, t0 + shift, t1 + shift)
            }
            Curve::Ellipse { frame, major, minor } => {
                let (curve, shift) = conic(self.point(frame.origin), self.vector(frame.x) * *major, self.vector(frame.y) * *minor);
                (curve, t0 + shift, t1 + shift)
            }
            Curve::Spline(s) => (Curve::Spline(Arc::new(s.mapped(|p| self.point(p)))), t0, t1),
            Curve::Sampled(s) => {
                let surfaces = (on(&s.surfaces.0, &s.points)?, on(&s.surfaces.1, &s.points)?);
                let points = s.points.iter().map(|p| self.point(*p)).collect();
                let tangents = s.tangents.iter().map(|v| self.vector(*v).normalized()).collect();
                (Curve::Sampled(Arc::new(SampledCurve::new(points, tangents, surfaces))), t0, t1)
            }
        })
    }

    /// The image of a surface. `near` are points of the face or curve on it, which say how much of a round surface a patch
    /// has to cover once it's no longer round.
    fn surface(&self, s: &Surface, near: &[Vec3]) -> Result<Surface> {
        Ok(match s {
            Surface::Plane(f) => {
                let z = self.normal(f.z);
                let w = self.vector(f.x);
                let x = (w - z * z.dot(w)).normalized();
                Surface::Plane(Frame { origin: self.point(f.origin), x, y: z.cross(x), z })
            }
            Surface::Cylinder { frame, radius } if self.along(frame.z) => Surface::Cylinder { frame: Frame { origin: self.point(frame.origin), ..*frame }, radius: *radius },
            // Heights along the axis grow k times, and the radius at the frame's origin stays.
            Surface::Cone { frame, radius, half_angle } if self.along(frame.z) => {
                Surface::Cone { frame: Frame { origin: self.point(frame.origin), ..*frame }, radius: *radius, half_angle: (half_angle.tan() / self.k).atan() }
            }
            Surface::Revolved { frame, curve } if self.along(frame.z) => Surface::Revolved {
                frame: Frame { origin: self.point(frame.origin), ..*frame },
                curve: Arc::new(PlaneCurve::new(curve.spline.mapped(|p| v3(p.x, p.y * self.k, 0.0)))),
            },
            Surface::Extruded { frame, curve } => {
                // Swept along the image of its direction, the curve seen square to that.
                let z = self.vector(frame.z).normalized();
                let origin = self.point(frame.origin);
                let w = self.vector(frame.x);
                let x = (w - z * z.dot(w)).normalized();
                let y = z.cross(x);
                let to = |p: Vec3| {
                    let r = self.point(frame.to_world(v3(p.x, p.y, 0.0))) - origin;
                    v3(r.dot(x), r.dot(y), 0.0)
                };
                Surface::Extruded { frame: Frame { origin, x, y, z }, curve: Arc::new(PlaneCurve::new(curve.spline.mapped(to))) }
            }
            Surface::Free { frame, patch } => {
                if patch.offset() != 0.0 {
                    return Err(refuse(
                        "a free-form face moved by an offset, as an offset or a shell leaves one, can't be stretched: stretched, it's no longer a surface moved by a distance".into(),
                    ));
                }
                Surface::Free { frame: Frame::WORLD, patch: Arc::new(mapped(patch, |p| self.point(frame.to_world(p)))?) }
            }
            round => Surface::Free { frame: Frame::WORLD, patch: Arc::new(mapped(&revolution_patch(round, near)?, |p| self.point(p))?) },
        })
    }
}

/// A patch with its control points moved, which for an affine map is exactly the patch it maps to.
fn mapped(patch: &Patch, f: impl Fn(Vec3) -> Vec3) -> Result<Patch> {
    let (ku, kv) = patch.knots();
    Patch::new(patch.degree(), (ku.to_vec(), kv.to_vec()), patch.count(), patch.points().iter().map(|p| f(*p)).collect(), patch.weights().to_vec())
}

/// The circle or ellipse `c + cos(t) u + sin(t) v`, and what to add to `t` for the same point on it.
fn conic(c: Vec3, u: Vec3, v: Vec3) -> (Curve, f64) {
    // Turned by theta, the two half-diameters are square to each other: those are its axes. The first is the major one, as
    // its square is half their squares' sum plus half the root of (|u|^2 - |v|^2)^2 + (2 u.v)^2, and the second's is less.
    let theta = 0.5 * (2.0 * u.dot(v)).atan2(u.norm2() - v.norm2());
    let (s, co) = theta.sin_cos();
    let (first, second) = (u * co + v * s, v * co - u * s);
    let z = u.cross(v).normalized();
    let x = first.normalized();
    let y = z.cross(x);
    let (a, b) = (first.norm(), second.dot(y));
    if (a - b).abs() <= 1e-12 * a.max(b) {
        return (Curve::Circle { frame: Frame { origin: c, x, y, z }, radius: (a + b) / 2.0 }, -theta);
    }
    (Curve::Ellipse { frame: Frame { origin: c, x, y, z }, major: a, minor: b }, -theta)
}

/// The shortest arc of a turn that holds all these angles, as its two ends, the second the larger: everything but the
/// widest gap between them. That doesn't depend on the order they come in, which an edge through a pole muddles.
fn arc_of(angles: impl Iterator<Item = f64>) -> Option<(f64, f64)> {
    let mut list: Vec<f64> = angles.map(|a| a.rem_euclid(TAU)).collect();
    if list.is_empty() {
        return None;
    }
    list.sort_by(f64::total_cmp);
    let n = list.len();
    // The gap after each angle, the last one's round past a whole turn to the first.
    let gap = |i: usize| if i + 1 < n { list[i + 1] - list[i] } else { list[0] + TAU - list[i] };
    let widest = (0..n).max_by(|&a, &b| gap(a).total_cmp(&gap(b))).unwrap();
    let start = list[(widest + 1) % n];
    Some((start, start + TAU - gap(widest)))
}

/// The range of some values, widened a little each way, but by no more than leaves `room` free of a whole turn.
fn widened(values: &[f64], at_least: f64, room: Option<f64>) -> (f64, f64) {
    let (lo, hi) = values.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
    let mut m = (0.02 * (hi - lo)).max(at_least);
    if let Some(turn) = room {
        m = m.min((turn - (hi - lo)) / 4.0);
    }
    (lo - m, hi + m)
}

/// A surface of revolution as the rational patch it exactly is, over the angles and the stretch of its meridian that
/// `near` reaches, and a little more: a circle's arc in one direction, the meridian in the other.
fn revolution_patch(s: &Surface, near: &[Vec3]) -> Result<Patch> {
    let frame = s.frame();
    let local: Vec<Vec3> = near.iter().map(|p| frame.to_local(*p)).collect();
    let size = near.iter().fold(Aabb::EMPTY, |mut b, p| {
        b.union(&Aabb { min: *p, max: *p });
        b
    });
    let tiny = 1e-9 * size.diagonal().max(1.0);
    let (lo, hi) = arc_of(local.iter().filter(|l| l.x.hypot(l.y) > tiny).map(|l| l.y.atan2(l.x)))
        .ok_or_else(|| refuse("a face on its axis has no way round it to be stretched by".into()))?;
    let round = [lo, hi];
    let span = hi - lo;
    if span > TAU - 0.1 {
        return Err(refuse(format!(
            "a {} face that goes all the way round its axis can't be stretched across it: only faces of up to most of a turn, as this kernel's own are",
            s.name()
        )));
    }
    let (a0, a1) = widened(&round, 1e-3, Some(TAU));
    let flat = |l: &Vec3| v2(l.x.hypot(l.y), l.z);
    let meridian = match *s {
        Surface::Cylinder { radius, .. } => {
            let (h0, h1) = widened(&local.iter().map(|l| l.z).collect::<Vec<_>>(), tiny, None);
            Spline::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v3(radius, h0, 0.0), v3(radius, h1, 0.0)], vec![1.0, 1.0])?
        }
        Surface::Cone { radius, half_angle, .. } => {
            let t = half_angle.tan();
            let (h0, h1) = widened(&local.iter().map(|l| l.z).collect::<Vec<_>>(), tiny, None);
            // Not past the apex.
            let h0 = h0.max(-radius / t);
            Spline::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![v3(radius + h0 * t, h0, 0.0), v3(radius + h1 * t, h1, 0.0)], vec![1.0, 1.0])?
        }
        Surface::Sphere { radius, .. } => {
            let up: Vec<f64> = local.iter().map(|l| l.z.atan2(flat(l).x)).collect();
            let (b0, b1) = widened(&up, 1e-3, None);
            Spline::ellipse_arc(Vec3::ZERO, v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), radius, radius, b0.max(-PI / 2.0), b1.min(PI / 2.0))?
        }
        Surface::Torus { major, minor, .. } => {
            if major <= 0.0 {
                return Err(refuse("a torus whose tube crosses its axis can't be stretched across it".into()));
            }
            let (lo, hi) = arc_of(local.iter().map(|l| l.z.atan2(flat(l).x - major))).expect("points");
            let up = [lo, hi];
            if hi - lo > TAU - 0.1 {
                return Err(refuse("a torus face that goes all the way round its tube can't be stretched across its axis".into()));
            }
            let (b0, b1) = widened(&up, 1e-3, Some(TAU));
            Spline::ellipse_arc(v3(major, 0.0, 0.0), v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), minor, minor, b0, b1)?
        }
        Surface::Revolved { ref curve, .. } => {
            let along: Vec<f64> = local.iter().map(|l| curve.foot(flat(l)).t).collect();
            let (d0, d1) = curve.spline.domain();
            let (lo, hi) = widened(&along, 1e-6 * (d1 - d0), None);
            let (lo, hi) = (lo.max(d0), hi.min(d1));
            if along.iter().any(|t| *t < d0 - 1e-9 * (d1 - d0) || *t > d1 + 1e-9 * (d1 - d0)) {
                return Err(refuse("a face reaching past the end of the curve it's turned from can't be stretched across its axis".into()));
            }
            curve.spline.sub(lo, hi)?
        }
        _ => unreachable!("only round surfaces"),
    };
    let circle = Spline::ellipse_arc(Vec3::ZERO, v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), 1.0, 1.0, a0, a1)?;
    let (cu, wu) = (circle.points(), circle.weights());
    let (cm, wm) = (meridian.points(), meridian.weights());
    let mut points = Vec::with_capacity(cu.len() * cm.len());
    let mut weights = Vec::with_capacity(cu.len() * cm.len());
    for i in 0..cu.len() {
        for j in 0..cm.len() {
            let (r, h) = (cm[j].x, cm[j].y);
            points.push(frame.to_world(v3(r * cu[i].x, r * cu[i].y, h)));
            weights.push(wu[i] * wm[j]);
        }
    }
    Patch::new((2, meridian.degree()), (circle.knots().to_vec(), meridian.knots().to_vec()), (cu.len(), cm.len()), points, weights)
}

impl Body {
    /// This body stretched along `d` by `k`, about the plane through `q` square to it: see the module's notes. Every face
    /// keeps its name. Its volume has to come to `k` times what it was, or it's refused.
    pub fn stretched(&self, q: Vec3, d: Vec3, k: f64) -> Result<Body> {
        let s = Stretch::new(q, d, k)?;
        let vertices: Vec<Vec3> = self.vertices.iter().map(|p| s.point(*p)).collect();
        if let Some(p) = vertices.iter().find(|p| !(p.x.abs().max(p.y.abs()).max(p.z.abs()) <= MAX_COORD)) {
            return Err(refuse(format!("stretched by {k}, the body reaches ({:.3e}, {:.3e}, {:.3e}), beyond the {MAX_COORD} mm this kernel works within", p.x, p.y, p.z)));
        }
        // Points round each face's boundary and inside it, which say how much of a round surface it covers. The inside
        // says which way round it goes where its boundary runs through a pole.
        let boundary = |f: usize| -> Vec<Vec3> {
            let face = &self.faces[f];
            let mut out = Vec::new();
            for c in face.loops.iter().flatten() {
                out.extend((0..=16).map(|i| self.coedge_point(*c, i as f64 / 16.0)));
            }
            out.extend(crate::boolean::trace::interior_points(&face.surface, &face.chart, &self.edges, &face.loops, 8, false).into_iter().map(|uv| face.surface.from_chart(&face.chart, uv)));
            out
        };
        let surfaces = (0..self.faces.len())
            .map(|f| s.surface(&self.faces[f].surface, &boundary(f)).map_err(|e| refuse(format!("{e} (face {f}, a {})", self.faces[f].surface.name()))))
            .collect::<Result<Vec<Surface>>>()?;
        let uses = self.edge_uses();
        let mut edges = Vec::with_capacity(self.edges.len());
        for (e, edge) in self.edges.iter().enumerate() {
            // A traced edge lies on its faces' surfaces, which it takes as they now are.
            let on = |surface: &Surface, points: &[Vec3]| -> Result<Surface> {
                match uses[e].iter().find(|u| self.faces[u.0].surface == *surface) {
                    Some(u) => Ok(surfaces[u.0].clone()),
                    None => s.surface(surface, points),
                }
            };
            let (curve, t0, t1) = s.curve(&edge.curve, edge.t0, edge.t1, &on)?;
            let (a, m, b) = (vertices[edge.v0], curve.eval((t0 + t1) / 2.0), vertices[edge.v1]);
            if a.dist(m) + m.dist(b) < MIN_FEATURE {
                return Err(refuse(format!(
                    "stretched by {k}, the edge through ({:.6}, {:.6}, {:.6}) would be too short for this kernel's tolerance of {TOL}",
                    m.x, m.y, m.z
                )));
            }
            edges.push(Edge { curve, t0, t1, v0: edge.v0, v1: edge.v1 });
        }
        let mut out = Body { vertices, edges, tolerance: self.tolerance * k.max(1.0), sheet: self.sheet, ..Body::default() };
        for (f, face) in self.faces.iter().enumerate() {
            // The outward side stays outward. A surface that became a patch can have its own normal the other way. That's
            // asked at points along its edges, as one at a corner can be a patch's pole, where its normal says little.
            let agree: f64 = face
                .loops
                .iter()
                .flatten()
                .flat_map(|c| [0.3, 0.5, 0.7].map(|t| self.coedge_point(*c, t)))
                .map(|p| s.normal(face.surface.normal(p)).dot(surfaces[f].normal(s.point(p))))
                .sum();
            let flipped = if agree < 0.0 { !face.flipped } else { face.flipped };
            let i = out.add_face(surfaces[f].clone(), flipped, face.loops.clone(), face.tag);
            out.faces[i].origin = face.origin.clone();
        }
        out.shells = self.shells.clone();
        validate(&out).map_err(|defects| refuse(format!("stretched, the body isn't valid: {}", defects.join("; "))))?;
        if !self.sheet {
            let (before, after) = (crate::mass::mass_properties(self).volume, crate::mass::mass_properties(&out).volume);
            if (after - before * k).abs() > 1e-7 * (before * k).abs().max(1e-9) {
                return Err(Error::Internal(format!("stretched by {k}, the body's volume is {after}, not {k} times {before}")));
            }
        }
        Ok(out)
    }
}
