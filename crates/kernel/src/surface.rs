//! The surfaces: five analytic ones, and curves swept along a line or around an axis. Each has three
//! things every algorithm relies on:
//!
//! * an exact **signed distance** (`sdf`). It is 1-Lipschitz in 3D, so a value
//!   of `d` at a point proves there is no surface within distance `d`. Root
//!   finders use this as a certificate instead of hoping sampling was enough.
//! * a closest-point **projection**.
//! * a per-face **chart**: a map from the face to the plane that is single
//!   valued and free of singularities on the face. All 2D topology runs in
//!   charts, so no algorithm ever sees a seam or a pole.
//!
//! Charts are right handed with respect to the surface normal, and 3D distance
//! never exceeds chart distance. Algorithms depend on both properties.
//!
//! A swept curve keeps all three because its distance is a distance in the plane: to the curve itself,
//! which `crate::spline` finds with certainty. See that module for why it's a true distance.

use crate::math::*;
use crate::patch::Patch;
use crate::spline::PlaneCurve;
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq)]
pub enum Surface {
    /// Normal is `frame.z`.
    Plane(Frame),
    /// Axis is `frame.z`. Normal points away from the axis.
    Cylinder { frame: Frame, radius: f64 },
    /// `radius` is measured at `frame.origin`. Widens along `+frame.z`, with
    /// `0 < half_angle < PI/2`. Only the nappe above the apex exists. Normal
    /// points away from the axis.
    Cone { frame: Frame, radius: f64, half_angle: f64 },
    /// Normal points outward.
    Sphere { frame: Frame, radius: f64 },
    /// `frame.z` is the axis. Normal points away from the tube centre.
    ///
    /// `major` may be negative, down to just above `-minor`. That is the lens-shaped part
    /// of a self-intersecting torus, the "lemon": its tube centre sits across the axis, and
    /// the surface ends in a tip on the axis at each end. A rolling ball makes one when
    /// its radius exceeds the radius of the rim it follows.
    Torus { frame: Frame, major: f64, minor: f64 },
    /// A curve in the frame's XY plane, swept along `frame.z`. Normal is to the right of the curve's
    /// direction of travel.
    Extruded { frame: Frame, curve: Arc<PlaneCurve> },
    /// A curve swept around `frame.z`, its x being the distance from the axis and its y the height along
    /// it. Normal is to the right of the curve's direction of travel in that half-plane. The curve may end
    /// on the axis, and only there.
    Revolved { frame: Frame, curve: Arc<PlaneCurve> },
    /// A free-form patch, in the frame's coordinates. Normal is along `S_u x S_v`.
    Free { frame: Frame, patch: Arc<Patch> },
}

/// Per-face chart parameters. Only some fields apply to each surface: `u0` and
/// `v0` are the centres of the angular ranges, and `frame` is the stereographic
/// frame on a sphere. On a revolved curve, `v0` says which end of it is on the axis: 1 for its start,
/// -1 for its end, 0 for neither.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Chart {
    pub u0: f64,
    pub v0: f64,
    pub frame: Frame,
}

impl Default for Chart {
    fn default() -> Chart {
        Chart { u0: 0.0, v0: 0.0, frame: Frame::WORLD }
    }
}

impl Surface {
    pub fn frame(&self) -> &Frame {
        match self {
            Surface::Plane(frame)
            | Surface::Cylinder { frame, .. }
            | Surface::Cone { frame, .. }
            | Surface::Sphere { frame, .. }
            | Surface::Torus { frame, .. }
            | Surface::Extruded { frame, .. }
            | Surface::Revolved { frame, .. }
            | Surface::Free { frame, .. } => frame,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Surface::Plane(_) => "plane",
            Surface::Cylinder { .. } => "cylinder",
            Surface::Cone { .. } => "cone",
            Surface::Sphere { .. } => "sphere",
            Surface::Torus { .. } => "torus",
            Surface::Extruded { .. } => "extrusion",
            Surface::Revolved { .. } => "revolution",
            Surface::Free { .. } => "free-form surface",
        }
    }

    /// Signed distance, positive on the side the normal points to.
    pub fn sdf(&self, p: Vec3) -> f64 {
        let l = self.frame().to_local(p);
        match self {
            Surface::Extruded { curve, .. } => return curve.signed_distance(v2(l.x, l.y)),
            Surface::Revolved { curve, .. } => return curve.signed_distance(v2(l.x.hypot(l.y), l.z)),
            Surface::Free { patch, .. } => return patch.signed_distance(l),
            _ => {}
        }
        match *self {
            Surface::Plane(_) => l.z,
            Surface::Cylinder { radius, .. } => l.x.hypot(l.y) - radius,
            Surface::Cone { radius, half_angle, .. } => {
                let (s, c) = half_angle.sin_cos();
                let rho = l.x.hypot(l.y);
                // Height above the apex is z + radius / tan(a). Multiplying
                // through by cos(a) keeps this exact and avoids dividing by a
                // small tangent.
                (rho - radius) * c - l.z * s
            }
            Surface::Sphere { radius, .. } => l.norm() - radius,
            Surface::Torus { major, minor, .. } => (l.x.hypot(l.y) - major).hypot(l.z) - minor,
            Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. } => unreachable!(),
        }
    }

    /// Unit gradient of `sdf`, which is the surface normal at the projection of
    /// `p`. On an axis or centre, where the gradient is undefined, returns one
    /// of the valid directions.
    pub fn normal(&self, p: Vec3) -> Vec3 {
        let f = *self.frame();
        let l = f.to_local(p);
        let radial = |x: f64, y: f64| {
            let r = x.hypot(y);
            if r < 1e-300 {
                f.x
            } else {
                f.x * (x / r) + f.y * (y / r)
            }
        };
        match *self {
            Surface::Plane(_) => f.z,
            Surface::Cylinder { .. } => radial(l.x, l.y),
            Surface::Cone { half_angle, .. } => {
                let (s, c) = half_angle.sin_cos();
                radial(l.x, l.y) * c - f.z * s
            }
            Surface::Sphere { .. } => {
                let n = l.norm();
                if n < 1e-300 {
                    f.z
                } else {
                    f.dir_to_world(l / n)
                }
            }
            Surface::Torus { major, .. } => {
                let rho = l.x.hypot(l.y);
                let (a, b) = (rho - major, l.z);
                let n = a.hypot(b);
                if n < 1e-300 {
                    f.z
                } else {
                    radial(l.x, l.y) * (a / n) + f.z * (b / n)
                }
            }
            Surface::Extruded { ref curve, .. } => {
                let n = curve.foot(v2(l.x, l.y)).tangent.right();
                f.x * n.x + f.y * n.y
            }
            Surface::Revolved { ref curve, .. } => {
                let n = curve.foot(v2(l.x.hypot(l.y), l.z)).tangent.right();
                radial(l.x, l.y) * n.x + f.z * n.y
            }
            Surface::Free { ref patch, .. } => f.dir_to_world(patch.normal(l)),
        }
    }

    /// Closest point on the surface, for points near it.
    pub fn project(&self, p: Vec3) -> Vec3 {
        let f = self.frame();
        let l = f.to_local(p);
        match self {
            Surface::Extruded { curve, .. } => {
                let at = curve.foot(v2(l.x, l.y)).at;
                return f.point(at.x, at.y, l.z);
            }
            Surface::Revolved { curve, .. } => {
                let rho = l.x.hypot(l.y);
                let at = curve.foot(v2(rho, l.z)).at;
                let radial = if rho > 1e-300 { f.x * (l.x / rho) + f.y * (l.y / rho) } else { f.x };
                return f.origin + radial * at.x + f.z * at.y;
            }
            _ => {}
        }
        p - self.normal(p) * self.sdf(p)
    }

    /// The axis of revolution, if the surface has a unique one. A sphere is
    /// symmetric about every line through its centre and returns `None`.
    pub fn axis(&self) -> Option<(Vec3, Vec3)> {
        match self {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Torus { frame, .. } | Surface::Revolved { frame, .. } => {
                Some((frame.origin, frame.z))
            }
            _ => None,
        }
    }

    /// Location of the apex of a cone.
    pub fn apex(&self) -> Option<Vec3> {
        match *self {
            Surface::Cone { frame, radius, half_angle } => Some(frame.origin - frame.z * (radius / half_angle.tan())),
            _ => None,
        }
    }

    /// Builds a chart that keeps all `points` (points of one face) well inside
    /// its valid range.
    pub fn fit_chart(&self, points: &[Vec3]) -> Chart {
        let f = *self.frame();
        let mut chart = Chart::default();
        match *self {
            Surface::Plane(_) => {}
            Surface::Cylinder { .. } | Surface::Cone { .. } => {
                let us: Vec<f64> = points
                    .iter()
                    .filter_map(|&p| {
                        let l = f.to_local(p);
                        (l.x.hypot(l.y) > 1e-9).then(|| l.y.atan2(l.x))
                    })
                    .collect();
                chart.u0 = angular_center(&us);
            }
            Surface::Torus { major, minor, .. } if major < 0.0 => {
                // Polar coordinates about whichever tip the face is at, so the tip is
                // one ordinary point of the chart and not a whole line of them.
                let us: Vec<f64> = points
                    .iter()
                    .filter_map(|&p| {
                        let l = f.to_local(p);
                        (l.x.hypot(l.y) > 1e-9).then(|| l.y.atan2(l.x))
                    })
                    .collect();
                chart.u0 = angular_center(&us);
                let height: f64 = points.iter().map(|&p| f.to_local(p).z).sum();
                let tip = (-major / minor).acos();
                chart.v0 = if height >= 0.0 { tip } else { -tip };
            }
            Surface::Torus { major, .. } => {
                let us: Vec<f64> = points
                    .iter()
                    .map(|&p| {
                        let l = f.to_local(p);
                        l.y.atan2(l.x)
                    })
                    .collect();
                let vs: Vec<f64> = points
                    .iter()
                    .map(|&p| {
                        let l = f.to_local(p);
                        l.z.atan2(l.x.hypot(l.y) - major)
                    })
                    .collect();
                chart.u0 = angular_center(&us);
                chart.v0 = angular_center(&vs);
            }
            Surface::Sphere { .. } => {
                let mut mean = Vec3::ZERO;
                for &p in points {
                    mean += (p - f.origin).normalized();
                }
                let c = if mean.norm() > 1e-6 {
                    mean.normalized()
                } else if let Some(&p) = points.first() {
                    (p - f.origin).normalized()
                } else {
                    f.z
                };
                chart.frame = Frame::new(f.origin, c, c.any_perpendicular());
            }
            Surface::Extruded { .. } | Surface::Free { .. } => {}
            Surface::Revolved { ref curve, .. } => {
                let us: Vec<f64> = points
                    .iter()
                    .filter_map(|&p| {
                        let l = f.to_local(p);
                        (l.x.hypot(l.y) > 1e-9).then(|| l.y.atan2(l.x))
                    })
                    .collect();
                chart.u0 = angular_center(&us);
                let (t0, t1) = curve.spline.domain();
                chart.v0 = if curve.eval(t0).x.abs() <= TOL {
                    1.0
                } else if curve.eval(t1).x.abs() <= TOL {
                    -1.0
                } else {
                    0.0
                };
            }
        }
        chart
    }

    /// Maps a point on (or near) the surface into chart coordinates.
    pub fn to_chart(&self, ch: &Chart, p: Vec3) -> Vec2 {
        let f = self.frame();
        match *self {
            Surface::Plane(_) => {
                let l = f.to_local(p);
                v2(l.x, l.y)
            }
            Surface::Cylinder { radius, .. } => {
                let l = f.to_local(p);
                v2(radius * wrap_pi(l.y.atan2(l.x) - ch.u0), l.z)
            }
            Surface::Cone { radius, half_angle, .. } => {
                let l = f.to_local(p);
                let (s, c) = half_angle.sin_cos();
                let rho = l.x.hypot(l.y);
                // Slant distance from the apex, then the unrolled angle.
                let slant = (rho * s + (l.z + radius * c / s) * c).max(0.0);
                let phi = if rho > 1e-300 { wrap_pi(l.y.atan2(l.x) - ch.u0) * s } else { 0.0 };
                v2(slant * phi.cos(), -slant * phi.sin())
            }
            Surface::Sphere { radius, .. } => {
                let d = p - ch.frame.origin;
                let (x, y, z) = (d.dot(ch.frame.x) / radius, d.dot(ch.frame.y) / radius, d.dot(ch.frame.z) / radius);
                let w = 2.0 * radius / (1.0 + z).max(1e-12);
                v2(x * w, y * w)
            }
            Surface::Torus { major, minor, .. } if major < 0.0 => {
                let l = f.to_local(p);
                let rho = l.x.hypot(l.y);
                let v = l.z.atan2(rho - major);
                let (distance, mirror) =
                    if ch.v0 >= 0.0 { ((ch.v0 - v) * minor, 1.0) } else { ((v - ch.v0) * minor, -1.0) };
                let distance = distance.max(0.0);
                let phi = if rho > 1e-300 { wrap_pi(l.y.atan2(l.x) - ch.u0) } else { 0.0 };
                v2(distance * phi.cos(), mirror * distance * phi.sin())
            }
            Surface::Torus { major, minor, .. } => {
                let l = f.to_local(p);
                v2(
                    (major + minor) * wrap_pi(l.y.atan2(l.x) - ch.u0),
                    minor * wrap_pi(l.z.atan2(l.x.hypot(l.y) - major) - ch.v0),
                )
            }
            Surface::Extruded { ref curve, .. } => {
                let l = f.to_local(p);
                v2(curve.foot(v2(l.x, l.y)).t * curve.speed, l.z)
            }
            Surface::Free { ref patch, .. } => patch.to_chart(f.to_local(p)),
            Surface::Revolved { ref curve, .. } => {
                let l = f.to_local(p);
                let rho = l.x.hypot(l.y);
                let t = curve.foot(v2(rho, l.z)).t;
                let phi = if rho > 1e-300 { wrap_pi(l.y.atan2(l.x) - ch.u0) } else { 0.0 };
                if ch.v0 == 0.0 {
                    return v2(curve.reach * phi, curve.speed * t);
                }
                // Polar about the tip, so that it's one ordinary point of the chart. The chart stays
                // right handed by mirroring when the curve runs away from the tip.
                let (t0, t1) = curve.spline.domain();
                let distance = (curve.speed * if ch.v0 > 0.0 { t - t0 } else { t1 - t }).max(0.0);
                v2(distance * phi.cos(), -ch.v0 * distance * phi.sin())
            }
        }
    }

    /// Maps chart coordinates to the surface.
    pub fn from_chart(&self, ch: &Chart, uv: Vec2) -> Vec3 {
        self.chart_derivs(ch, uv).0
    }

    /// Surface point and its partial derivatives with respect to chart x and y.
    /// `dx.cross(dy)` points along the surface normal.
    pub fn chart_derivs(&self, ch: &Chart, uv: Vec2) -> (Vec3, Vec3, Vec3) {
        let f = *self.frame();
        match *self {
            Surface::Plane(_) => (f.to_world(v3(uv.x, uv.y, 0.0)), f.x, f.y),
            Surface::Cylinder { radius, .. } => {
                let u = ch.u0 + uv.x / radius;
                let (s, c) = u.sin_cos();
                let radial = f.x * c + f.y * s;
                (f.origin + radial * radius + f.z * uv.y, f.y * c - f.x * s, f.z)
            }
            Surface::Cone { radius, half_angle, .. } => {
                let (sa, ca) = half_angle.sin_cos();
                let slant = uv.x.hypot(uv.y);
                let phi = if slant > 1e-300 { (-uv.y).atan2(uv.x) } else { 0.0 };
                let u = ch.u0 + phi / sa;
                let (su, cu) = u.sin_cos();
                let radial = f.x * cu + f.y * su;
                let theta = f.y * cu - f.x * su;
                let apex = f.origin - f.z * (radius / half_angle.tan());
                let along = radial * sa + f.z * ca;
                let p = apex + along * slant;
                let (ux, uy) = if slant > 1e-300 { (uv.x / slant, uv.y / slant) } else { (1.0, 0.0) };
                (p, along * ux + theta * uy, along * uy - theta * ux)
            }
            Surface::Sphere { radius, .. } => {
                let c = ch.frame;
                let a = uv.x / (2.0 * radius);
                let b = uv.y / (2.0 * radius);
                let s = a * a + b * b;
                let k = 1.0 / (1.0 + s);
                let q = c.x * (2.0 * a * k) + c.y * (2.0 * b * k) + c.z * ((1.0 - s) * k);
                let k2 = k * k;
                // d(q)/da and d(q)/db, halved because chart x is 2R * a.
                let da = c.x * ((1.0 + s - 2.0 * a * a) * k2) + c.y * (-2.0 * a * b * k2) + c.z * (-2.0 * a * k2);
                let db = c.x * (-2.0 * a * b * k2) + c.y * ((1.0 + s - 2.0 * b * b) * k2) + c.z * (-2.0 * b * k2);
                (c.origin + q * radius, da, db)
            }
            Surface::Torus { major, minor, .. } if major < 0.0 => {
                let upper = ch.v0 >= 0.0;
                let mirror = if upper { 1.0 } else { -1.0 };
                let distance = uv.x.hypot(uv.y);
                let phi = if distance > 1e-300 { (mirror * uv.y).atan2(uv.x) } else { 0.0 };
                let v = if upper { ch.v0 - distance / minor } else { ch.v0 + distance / minor };
                let (su, cu) = (ch.u0 + phi).sin_cos();
                let (sv, cv) = v.sin_cos();
                let radial = f.x * cu + f.y * su;
                let theta = f.y * cu - f.x * su;
                let rho = major + minor * cv;
                let p = f.origin + radial * rho + f.z * (minor * sv);
                // Moving away from the tip along a meridian, and turning about the axis.
                let meridian = (radial * -sv + f.z * cv) * -mirror;
                let (ux, uy, spread) = if distance > 1e-12 {
                    (uv.x / distance, uv.y / distance, rho / distance)
                } else {
                    (1.0, 0.0, ch.v0.abs().sin())
                };
                (p, meridian * ux + theta * (-mirror * uy * spread), meridian * uy + theta * (mirror * ux * spread))
            }
            Surface::Torus { major, minor, .. } => {
                let u = ch.u0 + uv.x / (major + minor);
                let v = ch.v0 + uv.y / minor;
                let (su, cu) = u.sin_cos();
                let (sv, cv) = v.sin_cos();
                let radial = f.x * cu + f.y * su;
                let theta = f.y * cu - f.x * su;
                let rho = major + minor * cv;
                (f.origin + radial * rho + f.z * (minor * sv), theta * (rho / (major + minor)), radial * -sv + f.z * cv)
            }
            Surface::Free { ref patch, .. } => {
                let (q, dx, dy) = patch.from_chart(uv);
                (f.to_world(q), f.dir_to_world(dx), f.dir_to_world(dy))
            }
            Surface::Extruded { ref curve, .. } => {
                let t = uv.x / curve.speed;
                let (at, d) = (curve.eval(t), curve.derivative(t) / curve.speed);
                (f.point(at.x, at.y, uv.y), f.x * d.x + f.y * d.y, f.z)
            }
            Surface::Revolved { ref curve, .. } if ch.v0 == 0.0 => {
                let t = uv.y / curve.speed;
                let (su, cu) = (ch.u0 + uv.x / curve.reach).sin_cos();
                let radial = f.x * cu + f.y * su;
                let theta = f.y * cu - f.x * su;
                let (at, d) = (curve.eval(t), curve.derivative(t) / curve.speed);
                (f.origin + radial * at.x + f.z * at.y, theta * (at.x / curve.reach), radial * d.x + f.z * d.y)
            }
            Surface::Revolved { ref curve, .. } => {
                let (t0, t1) = curve.spline.domain();
                let distance = uv.x.hypot(uv.y);
                let mirror = -ch.v0;
                let phi = if distance > 1e-300 { (mirror * uv.y).atan2(uv.x) } else { 0.0 };
                let t = if ch.v0 > 0.0 { t0 + distance / curve.speed } else { t1 - distance / curve.speed };
                let (su, cu) = (ch.u0 + phi).sin_cos();
                let radial = f.x * cu + f.y * su;
                let theta = f.y * cu - f.x * su;
                let (at, d) = (curve.eval(t), curve.derivative(t) / curve.speed);
                // Moving away from the tip along the curve, and turning about the axis.
                let meridian = (radial * d.x + f.z * d.y) * ch.v0;
                let (ux, uy, spread) = if distance > 1e-12 {
                    (uv.x / distance, uv.y / distance, at.x / distance)
                } else {
                    (1.0, 0.0, d.x.abs())
                };
                (f.origin + radial * at.x + f.z * at.y, meridian * ux + theta * (-mirror * uy * spread), meridian * uy + theta * (mirror * ux * spread))
            }
        }
    }

    /// Rigidly transforms the surface.
    /// The image under a reflection. A surface's normal is set by its shape and its z axis, so the
    /// image's normal is the reflection of the original's.
    pub fn mirrored(&self, m: &Mirror) -> Surface {
        match *self {
            // The sweep direction turns over, which keeps the normal on the right of the mirrored curve
            // the reflection of the original normal. x and y reflect, so the curve keeps its coordinates.
            Surface::Extruded { frame, ref curve } => Surface::Extruded {
                frame: Frame { origin: m.of_point(frame.origin), x: m.of_dir(frame.x), y: m.of_dir(frame.y), z: -m.of_dir(frame.z) },
                curve: curve.clone(),
            },
            Surface::Revolved { frame, ref curve } => Surface::Revolved { frame: m.of_surface_frame(&frame), curve: curve.clone() },
            // The frame's y turns over in the reflection, so the patch's own y does too. Its data is the same
            // kind of surface, so this can't fail where the original didn't.
            Surface::Free { frame, ref patch } => {
                Surface::Free { frame: m.of_surface_frame(&frame), patch: Arc::new(patch.reflected_y().expect("the reflection of a valid patch is valid")) }
            }
            Surface::Plane(f) => Surface::Plane(m.of_surface_frame(&f)),
            Surface::Cylinder { frame, radius } => Surface::Cylinder { frame: m.of_surface_frame(&frame), radius },
            Surface::Cone { frame, radius, half_angle } => Surface::Cone { frame: m.of_surface_frame(&frame), radius, half_angle },
            Surface::Sphere { frame, radius } => Surface::Sphere { frame: m.of_surface_frame(&frame), radius },
            Surface::Torus { frame, major, minor } => Surface::Torus { frame: m.of_surface_frame(&frame), major, minor },
        }
    }

    pub fn transformed(&self, t: &Frame) -> Surface {
        match *self {
            Surface::Plane(f) => Surface::Plane(t.compose(&f)),
            Surface::Cylinder { frame, radius } => Surface::Cylinder { frame: t.compose(&frame), radius },
            Surface::Cone { frame, radius, half_angle } => {
                Surface::Cone { frame: t.compose(&frame), radius, half_angle }
            }
            Surface::Sphere { frame, radius } => Surface::Sphere { frame: t.compose(&frame), radius },
            Surface::Torus { frame, major, minor } => Surface::Torus { frame: t.compose(&frame), major, minor },
            Surface::Extruded { frame, ref curve } => Surface::Extruded { frame: t.compose(&frame), curve: curve.clone() },
            Surface::Revolved { frame, ref curve } => Surface::Revolved { frame: t.compose(&frame), curve: curve.clone() },
            Surface::Free { frame, ref patch } => Surface::Free { frame: t.compose(&frame), patch: patch.clone() },
        }
    }
}

/// Moves `p` to the intersection of two surfaces with a few Newton steps,
/// taking the smallest displacement at each step. Fails where the surfaces are
/// tangent, because the intersection is then ill conditioned.
pub fn refine_on_two(a: &Surface, b: &Surface, p: Vec3) -> Option<Vec3> {
    let mut p = p;
    for _ in 0..8 {
        let (da, db) = (a.sdf(p), b.sdf(p));
        if da.abs() < 1e-13 && db.abs() < 1e-13 {
            return Some(p);
        }
        let (na, nb) = (a.normal(p), b.normal(p));
        let g = na.dot(nb);
        let det = 1.0 - g * g;
        if det < 1e-14 {
            return None;
        }
        let ca = (da - db * g) / det;
        let cb = (db - da * g) / det;
        p -= na * ca + nb * cb;
    }
    let ok = a.sdf(p).abs() < TOL * 0.1 && b.sdf(p).abs() < TOL * 0.1;
    ok.then_some(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_surfaces() -> Vec<Surface> {
        let f = Frame::new(v3(1.0, -2.0, 3.0), v3(0.3, 0.5, 1.0), v3(1.0, 0.0, 0.0));
        vec![
            Surface::Plane(f),
            Surface::Cylinder { frame: f, radius: 4.0 },
            Surface::Cone { frame: f, radius: 4.0, half_angle: 0.5 },
            Surface::Sphere { frame: f, radius: 4.0 },
            Surface::Torus { frame: f, major: 6.0, minor: 2.0 },
            Surface::Extruded { frame: f, curve: swept_curve(0.0) },
            Surface::Revolved { frame: f, curve: swept_curve(9.0) },
        ]
    }

    /// A curve around x = 4 + `shift`, running up the y axis direction, for sweeping.
    fn swept_curve(shift: f64) -> Arc<PlaneCurve> {
        let pts = [(4.5, -9.0), (3.6, -3.0), (4.4, 3.0), (3.8, 9.0)];
        let spline = crate::spline::Spline::uniform(3, pts.iter().map(|p| v3(p.0 + shift, p.1, 0.0)).collect()).unwrap();
        Arc::new(PlaneCurve::new(spline))
    }

    /// The lemon's chart is polar about its tip, in both the upper and lower lobe.
    #[test]
    fn lemon_chart_is_consistent() {
        let f = Frame::new(v3(1.0, -2.0, 3.0), v3(0.3, 0.5, 1.0), Vec3::X);
        let s = Surface::Torus { frame: f, major: -1.0, minor: 9.0 };
        for height in [1.0, -1.0] {
            let tip = (1.0f64 / 9.0).acos();
            let probe = f.point(5.0, 1.0, height * 6.0);
            let ch = s.fit_chart(&[s.project(probe)]);
            assert!((ch.v0.abs() - tip).abs() < 1e-12);
            for &(x, y) in &[(0.4, 0.1), (1.0, -0.5), (2.5, 1.5), (0.05, 0.02), (4.0, 0.0)] {
                let uv = v2(x, y);
                let (p, dx, dy) = s.chart_derivs(&ch, uv);
                assert!(s.sdf(p).abs() < 1e-12, "on surface {uv:?} {}", s.sdf(p));
                assert!(s.to_chart(&ch, p).dist(uv) < 1e-9, "round trip {uv:?} {:?}", s.to_chart(&ch, p));
                let n = dx.cross(dy).normalized();
                assert!((n - s.normal(p)).norm() < 1e-8, "handedness at {uv:?}: {n:?} vs {:?}", s.normal(p));
                assert!(dx.norm() <= 1.0 + 1e-9 && dy.norm() <= 1.0 + 1e-9, "stretch {uv:?}");
                let h = 1e-5;
                let fd = (s.from_chart(&ch, v2(x + h, y)) - s.from_chart(&ch, v2(x - h, y))) / (2.0 * h);
                assert!((dx - fd).norm() < 1e-6, "dx at {uv:?}");
            }
            // The tip itself is one point of the chart.
            let tip_point = s.from_chart(&ch, v2(0.0, 0.0));
            assert!(f.to_local(tip_point).x.hypot(f.to_local(tip_point).y) < 1e-9);
        }
    }

    fn sample_chart_points(s: &Surface, ch: &Chart) -> Vec<Vec2> {
        let mut pts = Vec::new();
        for i in -3..=3 {
            for j in -3..=3 {
                pts.push(v2(i as f64 * 0.7 + 0.13, j as f64 * 0.7 + 0.11));
            }
        }
        if let Surface::Cone { .. } = s {
            // Keep to the positive side of the apex, and inside the half-angle.
            pts = pts.iter().map(|p| v2(p.x + 6.0, p.y)).collect();
        }
        // On a swept curve, the chart runs along the curve's own parameter.
        if let Surface::Extruded { curve, .. } = s {
            let (t0, t1) = curve.spline.domain();
            let mid = (t0 + t1) / 2.0 * curve.speed;
            pts = pts.iter().map(|p| v2(mid + p.x, p.y)).collect();
        }
        if let Surface::Revolved { curve, .. } = s {
            let (t0, t1) = curve.spline.domain();
            let mid = (t0 + t1) / 2.0 * curve.speed;
            pts = pts.iter().map(|p| v2(p.x, mid + p.y)).collect();
        }
        let _ = ch;
        pts
    }

    #[test]
    fn chart_points_lie_on_surface_and_round_trip() {
        for s in all_surfaces() {
            let probe = s.frame().point(4.0, 0.0, 0.0);
            let ch = s.fit_chart(&[s.project(probe)]);
            for uv in sample_chart_points(&s, &ch) {
                let p = s.from_chart(&ch, uv);
                assert!(s.sdf(p).abs() < 1e-10, "{} {:?} sdf={}", s.name(), uv, s.sdf(p));
                let back = s.to_chart(&ch, p);
                assert!(back.dist(uv) < 1e-9, "{} {:?} -> {:?}", s.name(), uv, back);
            }
        }
    }

    #[test]
    fn chart_derivatives_match_finite_differences_and_normal() {
        for s in all_surfaces() {
            let probe = s.frame().point(4.0, 0.0, 0.0);
            let ch = s.fit_chart(&[s.project(probe)]);
            for uv in sample_chart_points(&s, &ch) {
                let (p, dx, dy) = s.chart_derivs(&ch, uv);
                let h = 1e-5;
                let fdx = (s.from_chart(&ch, v2(uv.x + h, uv.y)) - s.from_chart(&ch, v2(uv.x - h, uv.y))) / (2.0 * h);
                let fdy = (s.from_chart(&ch, v2(uv.x, uv.y + h)) - s.from_chart(&ch, v2(uv.x, uv.y - h))) / (2.0 * h);
                assert!((dx - fdx).norm() < 1e-8, "{} dx {:?}", s.name(), uv);
                assert!((dy - fdy).norm() < 1e-8, "{} dy {:?}", s.name(), uv);
                let n = dx.cross(dy).normalized();
                assert!(
                    (n - s.normal(p)).norm() < 1e-8,
                    "{} chart is not right handed at {:?}: {:?} vs {:?}",
                    s.name(),
                    uv,
                    n,
                    s.normal(p)
                );
                // 3D distance must not exceed chart distance.
                assert!(dx.norm() <= 1.0 + 1e-9 && dy.norm() <= 1.0 + 1e-9, "{}", s.name());
            }
        }
    }

    #[test]
    fn sdf_is_one_lipschitz_with_matching_gradient() {
        let mut rng = Rng::new(7);
        for s in all_surfaces() {
            for _ in 0..500 {
                let p = v3(rng.range(-20.0, 20.0), rng.range(-20.0, 20.0), rng.range(-20.0, 20.0));
                let q = p + rng.unit_vector() * rng.range(0.0, 5.0);
                assert!((s.sdf(p) - s.sdf(q)).abs() <= p.dist(q) + 1e-9, "{} not Lipschitz", s.name());
                let h = 1e-6;
                let dir = rng.unit_vector();
                let fd = (s.sdf(p + dir * h) - s.sdf(p - dir * h)) / (2.0 * h);
                assert!((fd - s.normal(p).dot(dir)).abs() < 1e-6, "{} gradient", s.name());
            }
        }
    }

    #[test]
    fn projection_lands_on_surface() {
        let mut rng = Rng::new(11);
        for s in all_surfaces() {
            for _ in 0..200 {
                let p = v3(rng.range(-20.0, 20.0), rng.range(-20.0, 20.0), rng.range(-20.0, 20.0));
                // The cone's sdf is only an exact distance on the apex side of
                // its normal; use chart points to probe it near the surface.
                let q = s.project(p);
                if let Surface::Cone { .. } = s {
                    continue;
                }
                assert!(s.sdf(q).abs() < 1e-9, "{}", s.name());
            }
        }
    }
}
