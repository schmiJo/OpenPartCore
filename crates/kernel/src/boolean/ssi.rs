//! Surface-surface intersection.
//!
//! Most pairs that occur in real parts have closed-form answers, and those matter
//! most: exact curves, and exact tangency, which numerics handle worst. Two
//! surfaces of revolution about one axis (and a plane perpendicular to it, and a
//! sphere centred on it) reduce to intersecting their meridians in a half-plane,
//! so one routine covers that whole family.
//!
//! Everything else goes through a general contour tracer over the chart of one of
//! the faces. Its output is refined onto both surfaces, and it gives up with an
//! error at tangencies rather than returning a wrong topology.

use super::roots::{curve_surface_contact, Contact};
use crate::curve::{Curve, SampledCurve};
use crate::error::{Error, Result};
use crate::math::*;
use crate::spline::PlaneCurve;
use crate::surface::{refine_on_two, Chart, Surface};
use crate::hash::HashMap;
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct SsiCurve {
    pub curve: Curve,
    /// Closed curves have no ends. Circles and ellipses repeat every 2*PI in `t`.
    /// A closed sampled curve has its first and last sample at the same point.
    pub closed: bool,
    pub t0: f64,
    pub t1: f64,
}

#[derive(Clone, Debug)]
pub enum Ssi {
    /// The same surface. `same_sense` tells whether their normals agree.
    Coincident { same_sense: bool },
    /// Intersection curves, plus points where those curves cross each other.
    Curves { curves: Vec<SsiCurve>, crossings: Vec<Vec3> },
    /// No closed form applies. Use the contour tracer.
    General,
    /// Two surfaces run together over a stretch but aren't the same surface, as two splines sharing
    /// part of their length do. This point is on that stretch.
    PartlyCoincident { at: Vec3 },
}

fn empty() -> Ssi {
    Ssi::Curves { curves: Vec::new(), crossings: Vec::new() }
}

fn closed_circle(frame: Frame, radius: f64) -> SsiCurve {
    SsiCurve { curve: Curve::Circle { frame, radius }, closed: true, t0: 0.0, t1: TAU }
}

/// An infinite line clipped to a range covering the region of interest.
fn line_in_region(point: Vec3, dir: Vec3, region: &Aabb) -> SsiCurve {
    let centre = region.center();
    let origin = point + dir * (centre - point).dot(dir);
    let reach = region.diagonal() + 1.0;
    SsiCurve { curve: Curve::Line { origin, dir }, closed: false, t0: -reach, t1: reach }
}

fn parallel(a: Vec3, b: Vec3, size: f64) -> bool {
    a.cross(b).norm() * size.max(1.0) <= TOL
}

/// Whether a swept curve's whole length lies on a surface, and if so whether their normals agree there.
fn swept_on(lift: &dyn Fn(Vec2) -> Vec3, curve: &PlaneCurve, own: &Surface, other: &Surface) -> Option<bool> {
    let samples = curve.samples();
    if !samples.iter().all(|&q| other.sdf(lift(q)).abs() <= TOL) {
        return None;
    }
    let p = lift(samples[samples.len() / 2]);
    Some(own.normal(p).dot(other.normal(p)) > 0.0)
}

/// Whether two surfaces are the same within tolerance over a region of this size.
pub fn coincident(a: &Surface, b: &Surface, size: f64) -> Option<bool> {
    let close = |x: f64, y: f64| (x - y).abs() <= TOL;
    match (a, b) {
        (&Surface::Plane(f), &Surface::Plane(g)) => {
            if parallel(f.z, g.z, size)
                && (f.origin - g.origin).dot(g.z).abs() <= TOL
                && (g.origin - f.origin).dot(f.z).abs() <= TOL
            {
                Some(f.z.dot(g.z) > 0.0)
            } else {
                None
            }
        }
        (&Surface::Cylinder { frame: f, radius: r }, &Surface::Cylinder { frame: g, radius: s }) => {
            let d = g.origin - f.origin;
            (close(r, s) && parallel(f.z, g.z, size) && (d - f.z * d.dot(f.z)).norm() <= TOL).then_some(true)
        }
        (&Surface::Cone { frame: f, half_angle: x, .. }, &Surface::Cone { frame: g, half_angle: y, .. }) => {
            let same = f.z.dot(g.z) > 0.0
                && parallel(f.z, g.z, size)
                && (x - y).abs() * size.max(1.0) <= TOL
                && a.apex().unwrap().dist(b.apex().unwrap()) <= TOL;
            same.then_some(true)
        }
        (&Surface::Sphere { frame: f, radius: r }, &Surface::Sphere { frame: g, radius: s }) => {
            (close(r, s) && f.origin.dist(g.origin) <= TOL).then_some(true)
        }
        (&Surface::Torus { frame: f, major: m, minor: r }, &Surface::Torus { frame: g, major: n, minor: s }) => {
            (close(m, n) && close(r, s) && parallel(f.z, g.z, size) && f.origin.dist(g.origin) <= TOL).then_some(true)
        }
        // A swept curve is the same surface as another if each one's curve lies on the other, where the
        // curves exist. That covers a rational circle swept into a true cylinder too.
        // Two different patches can share part of their extent, but only the same patch in the same place is
        // recognised. Anything else near it refuses as a tangency.
        (Surface::Free { frame: f, patch: p }, Surface::Free { frame: g, patch: q }) => {
            (f.origin.dist(g.origin) <= TOL && f.z.dot(g.z) > 1.0 - 1e-12 && f.x.dot(g.x) > 1.0 - 1e-12 && p == q).then_some(true)
        }
        (Surface::Extruded { frame: f, curve: c }, Surface::Extruded { frame: g, curve: d }) => {
            if !parallel(f.z, g.z, size) {
                return None;
            }
            let (on_b, on_a): (&dyn Fn(Vec2) -> Vec3, &dyn Fn(Vec2) -> Vec3) = (&|q| f.point(q.x, q.y, 0.0), &|q| g.point(q.x, q.y, 0.0));
            swept_on(on_a, d, b, a)?;
            swept_on(on_b, c, a, b)
        }
        (Surface::Extruded { frame: f, curve: c }, &Surface::Cylinder { frame: g, .. }) => {
            parallel(f.z, g.z, size).then(|| swept_on(&|q| f.point(q.x, q.y, 0.0), c, a, b)).flatten()
        }
        (&Surface::Cylinder { frame: g, .. }, Surface::Extruded { frame: f, curve: c }) => {
            parallel(f.z, g.z, size).then(|| swept_on(&|q| f.point(q.x, q.y, 0.0), c, b, a)).flatten()
        }
        (Surface::Revolved { frame: f, curve: c }, Surface::Revolved { frame: g, curve: d }) => {
            if !common_axis(a, b, size).is_some() {
                return None;
            }
            swept_on(&|q| g.origin + g.x * q.x + g.z * q.y, d, b, a)?;
            swept_on(&|q| f.origin + f.x * q.x + f.z * q.y, c, a, b)
        }
        (Surface::Revolved { frame: f, curve: c }, Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }) => {
            common_axis(a, b, size)?;
            swept_on(&|q| f.origin + f.x * q.x + f.z * q.y, c, a, b)
        }
        (Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }, Surface::Revolved { frame: f, curve: c }) => {
            common_axis(a, b, size)?;
            swept_on(&|q| f.origin + f.x * q.x + f.z * q.y, c, b, a)
        }
        _ => None,
    }
}

// ----- The family of surfaces sharing an axis -----

#[derive(Clone, Copy, Debug)]
enum Meridian {
    Line { p: Vec2, d: Vec2 },
    Circle { c: Vec2, r: f64 },
}

/// A common axis for the pair, if one exists.
fn common_axis(a: &Surface, b: &Surface, size: f64) -> Option<(Vec3, Vec3)> {
    let on_axis = |o: Vec3, z: Vec3, q: Vec3| {
        let d = q - o;
        (d - z * d.dot(z)).norm() <= TOL
    };
    match (a, b) {
        (Surface::Plane(_), Surface::Plane(_)) => None,
        (Surface::Plane(f), Surface::Sphere { frame, .. }) | (Surface::Sphere { frame, .. }, Surface::Plane(f)) => {
            Some((frame.origin, f.z))
        }
        (Surface::Plane(f), other) | (other, Surface::Plane(f)) => {
            let (o, z) = other.axis()?;
            parallel(z, f.z, size).then_some((o, z))
        }
        (Surface::Sphere { frame: f, .. }, Surface::Sphere { frame: g, .. }) => {
            let d = g.origin - f.origin;
            (d.norm() > TOL).then(|| (f.origin, d.normalized()))
        }
        (Surface::Sphere { frame, .. }, other) | (other, Surface::Sphere { frame, .. }) => {
            let (o, z) = other.axis()?;
            on_axis(o, z, frame.origin).then_some((o, z))
        }
        (x, y) => {
            let (o, z) = x.axis()?;
            let (p, w) = y.axis()?;
            (parallel(z, w, size) && on_axis(o, z, p)).then_some((o, z))
        }
    }
}

fn meridian_of(s: &Surface, o: Vec3, z: Vec3) -> Option<Meridian> {
    let height = |q: Vec3| (q - o).dot(z);
    match *s {
        Surface::Plane(f) => Some(Meridian::Line { p: v2(0.0, height(f.origin)), d: v2(1.0, 0.0) }),
        Surface::Cylinder { frame, radius } => {
            Some(Meridian::Line { p: v2(radius, height(frame.origin)), d: v2(0.0, 1.0) })
        }
        Surface::Cone { frame, radius, half_angle } => {
            let sense = frame.z.dot(z).signum();
            Some(Meridian::Line {
                p: v2(radius, height(frame.origin)),
                d: v2(half_angle.sin(), half_angle.cos() * sense),
            })
        }
        Surface::Sphere { frame, radius } => Some(Meridian::Circle { c: v2(0.0, height(frame.origin)), r: radius }),
        Surface::Torus { frame, major, minor } => {
            Some(Meridian::Circle { c: v2(major, height(frame.origin)), r: minor })
        }
        Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. } => None,
    }
}

fn meridian_points(a: Meridian, b: Meridian) -> Vec<Vec2> {
    match (a, b) {
        (Meridian::Line { p, d }, Meridian::Line { p: q, d: e }) => {
            let det = d.cross(e);
            if det.abs() < 1e-14 {
                return Vec::new();
            }
            let t = (q - p).cross(e) / det;
            vec![p + d * t]
        }
        (Meridian::Line { p, d }, Meridian::Circle { c, r }) | (Meridian::Circle { c, r }, Meridian::Line { p, d }) => {
            let foot = p + d * (c - p).dot(d);
            let dist = foot.dist(c);
            if dist > r + TOL {
                Vec::new()
            } else if dist >= r - TOL {
                // Tangent. Put the point exactly on the circle.
                let n = if dist > 1e-300 { (foot - c) / dist } else { d };
                vec![c + n * r]
            } else {
                let h = (r * r - dist * dist).sqrt();
                vec![foot - d * h, foot + d * h]
            }
        }
        (Meridian::Circle { c, r }, Meridian::Circle { c: e, r: s }) => {
            let dist = c.dist(e);
            if dist < 1e-300 || dist > r + s + TOL || dist < (r - s).abs() - TOL {
                return Vec::new();
            }
            let u = (e - c) / dist;
            if dist >= r + s - TOL {
                return vec![c + u * r];
            }
            if dist <= (r - s).abs() + TOL {
                return vec![if r > s { c + u * r } else { c - u * r }];
            }
            let x = (dist * dist + r * r - s * s) / (2.0 * dist);
            let h = (r * r - x * x).max(0.0).sqrt();
            let w = v2(-u.y, u.x);
            vec![c + u * x + w * h, c + u * x - w * h]
        }
    }
}

/// Two surfaces about one axis, one of them a revolved curve: they meet in a circle wherever the curve
/// meets the other surface in any one half-plane through the axis. That's a curve against a true distance,
/// which finds every contact, including touching ones.
fn coaxial_with_curve(frame: &Frame, curve: &PlaneCurve, other: &Surface, o: Vec3, z: Vec3) -> Ssi {
    let reference = z.any_perpendicular();
    let lifted = Curve::Spline(Arc::new(curve.spline.mapped(|q| frame.origin + reference * q.x + frame.z * q.y)));
    let (t0, t1) = curve.spline.domain();
    let Contact::Roots(found) = curve_surface_contact(&lifted, t0, t1, other, TOL) else {
        return empty();
    };
    let mut curves = Vec::new();
    for t in found {
        let p = lifted.eval(t);
        let radius = (p - o).dot(reference);
        if radius <= TOL {
            continue; // On the axis: a point of contact at most, never an edge.
        }
        curves.push(closed_circle(Frame::new(o + z * (p - o).dot(z), z, reference), radius));
    }
    Ssi::Curves { curves, crossings: Vec::new() }
}

fn coaxial(a: &Surface, b: &Surface, o: Vec3, z: Vec3) -> Ssi {
    if let Surface::Revolved { frame, curve } = a {
        return coaxial_with_curve(frame, curve, b, o, z);
    }
    if let Surface::Revolved { frame, curve } = b {
        return coaxial_with_curve(frame, curve, a, o, z);
    }
    let (Some(ma), Some(mb)) = (meridian_of(a, o, z), meridian_of(b, o, z)) else {
        return empty();
    };
    let reference = z.any_perpendicular();
    let mut curves = Vec::new();
    for p in meridian_points(ma, mb) {
        if p.x <= TOL {
            continue; // On the axis: a point of contact at most, never an edge.
        }
        let frame = Frame::new(o + z * p.y, z, reference);
        let circle = closed_circle(frame, p.x);
        // Cones only exist on one side of their apex.
        let mid = circle.curve.eval(0.3);
        if a.sdf(mid).abs() <= 2.0 * TOL && b.sdf(mid).abs() <= 2.0 * TOL {
            curves.push(circle);
        }
    }
    Ssi::Curves { curves, crossings: Vec::new() }
}

// ----- Closed forms for the remaining common pairs -----

fn plane_plane(f: &Frame, g: &Frame, region: &Aabb) -> Ssi {
    let dir = f.z.cross(g.z);
    if dir.norm() * region.diagonal().max(1.0) <= TOL {
        return empty();
    }
    let n2 = dir.norm2();
    let (d1, d2) = (f.z.dot(f.origin), g.z.dot(g.origin));
    let point = (g.z.cross(dir) * d1 + dir.cross(f.z) * d2) / n2;
    Ssi::Curves { curves: vec![line_in_region(point, dir.normalized(), region)], crossings: Vec::new() }
}

fn plane_cylinder_ellipse(f: &Frame, c: &Frame, radius: f64) -> SsiCurve {
    let (a, n) = (c.z, f.z);
    let dot = a.dot(n);
    let centre = c.origin + a * ((f.origin - c.origin).dot(n) / dot);
    let x = (a - n * dot).normalized();
    SsiCurve {
        curve: Curve::Ellipse {
            frame: Frame { origin: centre, x, y: n.cross(x), z: n },
            major: radius / dot.abs(),
            minor: radius,
        },
        closed: true,
        t0: 0.0,
        t1: TAU,
    }
}

fn plane_cylinder(f: &Frame, c: &Frame, radius: f64, region: &Aabb) -> Ssi {
    let (a, n) = (c.z, f.z);
    if a.dot(n).abs() * region.diagonal().max(1.0) <= TOL {
        // Axis parallel to the plane: zero, one (tangent) or two rulings.
        let signed = (c.origin - f.origin).dot(n);
        let foot = c.origin - n * signed;
        let gap = radius - signed.abs();
        let curves = if gap < -TOL {
            Vec::new()
        } else if gap <= TOL {
            vec![line_in_region(foot, a, region)]
        } else {
            let half = (radius * radius - signed * signed).sqrt();
            let e = n.cross(a).normalized();
            vec![line_in_region(foot + e * half, a, region), line_in_region(foot - e * half, a, region)]
        };
        return Ssi::Curves { curves, crossings: Vec::new() };
    }
    Ssi::Curves { curves: vec![plane_cylinder_ellipse(f, c, radius)], crossings: Vec::new() }
}

fn plane_cone(f: &Frame, cone: &Surface, region: &Aabb) -> Ssi {
    let Surface::Cone { frame, half_angle, .. } = *cone else {
        return Ssi::General;
    };
    let apex = cone.apex().unwrap();
    let (a, n) = (frame.z, f.z);
    let nz = n.dot(a);
    let nr_vec = n - a * nz;
    let nr = nr_vec.norm();
    let e = nr_vec / nr;
    let (sa, ca) = half_angle.sin_cos();
    let offset = (f.origin - apex).dot(n);

    if offset.abs() <= TOL {
        // Through the apex: zero, one or two rulings.
        let q = -ca * nz / (sa * nr);
        if q.abs() > 1.0 + 1e-12 {
            return empty();
        }
        let w = a.cross(e);
        let theta = q.clamp(-1.0, 1.0).acos();
        let ruling = |angle: f64| (e * angle.cos() + w * angle.sin()) * sa + a * ca;
        // Far enough to reach any point of the region from the apex.
        let reach = apex.dist(region.center()) + region.diagonal() + 1.0;
        let line = |dir: Vec3| SsiCurve { curve: Curve::Line { origin: apex, dir }, closed: false, t0: 0.0, t1: reach };
        let mut curves = vec![line(ruling(theta))];
        if theta > 1e-7 && theta < PI - 1e-7 {
            curves.push(line(ruling(-theta)));
        }
        return Ssi::Curves { curves, crossings: Vec::new() };
    }

    // Dandelin: both ends of the major axis lie on rulings in the meridian plane
    // containing the plane's tilt. If both are on the real nappe it's an ellipse.
    let gp = e * sa + a * ca;
    let gm = a * ca - e * sa;
    let (dp, dm) = (n.dot(gp), n.dot(gm));
    if dp.abs() < 1e-12 || dm.abs() < 1e-12 {
        return Ssi::General; // Parabola.
    }
    let (sp, sm) = (offset / dp, offset / dm);
    if sp <= 0.0 || sm <= 0.0 {
        return Ssi::General; // Hyperbola.
    }
    let (vp, vm) = (apex + gp * sp, apex + gm * sm);
    let centre = (vp + vm) * 0.5;
    let major = vp.dist(vm) / 2.0;
    let x = (vp - vm).normalized();
    let h = (centre - apex).dot(a);
    let rho = h * half_angle.tan();
    let d = ((centre - apex) - a * h).norm();
    let minor = (rho * rho - d * d).max(0.0).sqrt();
    Ssi::Curves {
        curves: vec![SsiCurve {
            curve: Curve::Ellipse { frame: Frame { origin: centre, x, y: n.cross(x), z: n }, major, minor },
            closed: true,
            t0: 0.0,
            t1: TAU,
        }],
        crossings: Vec::new(),
    }
}

fn plane_torus(f: &Frame, frame: &Frame, major: f64, minor: f64, size: f64) -> Ssi {
    let (a, n) = (frame.z, f.z);
    let contains_axis = a.dot(n).abs() * size.max(1.0) <= TOL && (frame.origin - f.origin).dot(n).abs() <= TOL;
    if !contains_axis {
        return Ssi::General;
    }
    let e = a.cross(n).normalized();
    let circle = |c: Vec3| closed_circle(Frame::new(c, n, e), minor);
    Ssi::Curves {
        curves: vec![circle(frame.origin + e * major), circle(frame.origin - e * major)],
        crossings: Vec::new(),
    }
}

fn cylinder_cylinder(f: &Frame, r: f64, g: &Frame, s: f64, region: &Aabb) -> Ssi {
    let size = region.diagonal();
    if parallel(f.z, g.z, size) {
        let d3 = g.origin - f.origin;
        let offset = d3 - f.z * d3.dot(f.z);
        let d = offset.norm();
        if d > r + s + TOL || d < (r - s).abs() - TOL || d < 1e-300 {
            return empty();
        }
        let u = offset / d;
        let lines = |pts: Vec<Vec3>| Ssi::Curves {
            curves: pts.into_iter().map(|p| line_in_region(p, f.z, region)).collect(),
            crossings: Vec::new(),
        };
        if d >= r + s - TOL {
            return lines(vec![f.origin + u * r]);
        }
        if d <= (r - s).abs() + TOL {
            return lines(vec![if r > s { f.origin + u * r } else { f.origin - u * r }]);
        }
        let x = (d * d + r * r - s * s) / (2.0 * d);
        let h = (r * r - x * x).max(0.0).sqrt();
        let w = f.z.cross(u);
        return lines(vec![f.origin + u * x + w * h, f.origin + u * x - w * h]);
    }
    if (r - s).abs() > TOL {
        return Ssi::General;
    }
    // Equal radii with crossing axes: two ellipses in the bisecting planes.
    let n = f.z.cross(g.z);
    let w = g.origin - f.origin;
    let miss = w.dot(n) / n.norm();
    if miss.abs() > TOL {
        return Ssi::General;
    }
    let t = w.cross(g.z).dot(n) / n.norm2();
    let centre = f.origin + f.z * t;
    let plane = |normal: Vec3| Frame::new(centre, normal, normal.any_perpendicular());
    let (p1, p2) = (plane((f.z - g.z).normalized()), plane((f.z + g.z).normalized()));
    let side = n.normalized() * r;
    Ssi::Curves {
        curves: vec![plane_cylinder_ellipse(&p1, f, r), plane_cylinder_ellipse(&p2, f, r)],
        crossings: vec![centre + side, centre - side],
    }
}

/// A plane cutting across the sweep of a swept curve. Every point moves along the sweep direction by an
/// amount that's linear in its position, so the section is an affine image of the curve, which a rational
/// spline represents exactly.
fn plane_across_extrusion(f: &Frame, frame: &Frame, curve: &PlaneCurve) -> Ssi {
    let (a, n) = (frame.z, f.z);
    let dot = a.dot(n);
    let section = curve.spline.mapped(|q| {
        let base = frame.point(q.x, q.y, 0.0);
        base + a * ((f.origin - base).dot(n) / dot)
    });
    let (t0, t1) = curve.spline.domain();
    Ssi::Curves { curves: vec![SsiCurve { curve: Curve::Spline(Arc::new(section)), closed: false, t0, t1 }], crossings: Vec::new() }
}

/// A swept curve and a surface that doesn't change along the sweep either. They meet in straight rulings,
/// at the points where the curve meets the other surface.
fn along_rulings(frame: &Frame, curve: &PlaneCurve, other: &Surface, region: &Aabb) -> Ssi {
    let height = (region.center() - frame.origin).dot(frame.z);
    let lifted = Curve::Spline(Arc::new(curve.spline.mapped(|q| frame.point(q.x, q.y, height))));
    let (t0, t1) = curve.spline.domain();
    let Contact::Roots(found) = curve_surface_contact(&lifted, t0, t1, other, TOL) else {
        return empty();
    };
    // Between two contacts, the curve lying on the other surface means a stretch they share.
    for w in found.windows(2) {
        let probes = [0.25, 0.5, 0.75].map(|f| lifted.eval(w[0] + (w[1] - w[0]) * f));
        if lifted.eval(w[0]).dist(lifted.eval(w[1])) > 10.0 * TOL && probes.iter().all(|&p| other.sdf(p).abs() <= TOL) {
            return Ssi::PartlyCoincident { at: probes[1] };
        }
    }
    // A single contact can stand for such a stretch too, since close contacts are merged into one.
    for &t in &found {
        let step = (t1 - t0) * 1e-3;
        let p = lifted.eval(t);
        let flat = [t - step, t + step].iter().any(|&u| {
            let q = lifted.eval(u.clamp(t0, t1));
            q.dist(p) > 10.0 * TOL && other.sdf(q).abs() <= TOL * 0.1
        });
        if flat {
            return Ssi::PartlyCoincident { at: p };
        }
    }
    let curves = found.iter().map(|&t| line_in_region(lifted.eval(t), frame.z, region)).collect();
    Ssi::Curves { curves, crossings: Vec::new() }
}

/// Whether a surface stays the same along a direction.
fn unchanged_along(s: &Surface, d: Vec3, size: f64) -> bool {
    match s {
        Surface::Plane(f) => f.z.dot(d).abs() * size.max(1.0) <= TOL,
        Surface::Cylinder { frame, .. } | Surface::Extruded { frame, .. } => parallel(frame.z, d, size),
        _ => false,
    }
}

/// A plane through the axis of a revolved curve: the curve itself, on both sides of the axis.
fn plane_through_revolution(f: &Frame, frame: &Frame, curve: &PlaneCurve, size: f64) -> Ssi {
    let (a, n) = (frame.z, f.z);
    if a.dot(n).abs() * size.max(1.0) > TOL || (frame.origin - f.origin).dot(n).abs() > TOL {
        return Ssi::General;
    }
    let e = a.cross(n).normalized();
    let (t0, t1) = curve.spline.domain();
    let side = |r: Vec3| SsiCurve {
        curve: Curve::Spline(Arc::new(curve.spline.mapped(|q| frame.origin + r * q.x + a * q.y))),
        closed: false,
        t0,
        t1,
    };
    Ssi::Curves { curves: vec![side(e), side(-e)], crossings: Vec::new() }
}

/// Closed-form intersection where one exists.
pub fn intersect(a: &Surface, b: &Surface, region: &Aabb) -> Ssi {
    let size = region.diagonal();
    if let Some(same_sense) = coincident(a, b, size) {
        return Ssi::Coincident { same_sense };
    }
    if let Some((o, z)) = common_axis(a, b, size) {
        return coaxial(a, b, o, z);
    }
    match (a, b) {
        (Surface::Plane(f), Surface::Plane(g)) => plane_plane(f, g, region),
        (Surface::Plane(f), Surface::Cylinder { frame, radius })
        | (Surface::Cylinder { frame, radius }, Surface::Plane(f)) => plane_cylinder(f, frame, *radius, region),
        (Surface::Plane(f), Surface::Cone { .. }) => plane_cone(f, b, region),
        (Surface::Cone { .. }, Surface::Plane(f)) => plane_cone(f, a, region),
        (Surface::Plane(f), Surface::Torus { frame, major, minor })
        | (Surface::Torus { frame, major, minor }, Surface::Plane(f)) => plane_torus(f, frame, *major, *minor, size),
        (Surface::Cylinder { frame: f, radius: r }, Surface::Cylinder { frame: g, radius: s }) => {
            cylinder_cylinder(f, *r, g, *s, region)
        }
        (Surface::Extruded { frame, curve }, other) | (other, Surface::Extruded { frame, curve }) if unchanged_along(other, frame.z, size) => {
            along_rulings(frame, curve, other, region)
        }
        (Surface::Plane(f), Surface::Extruded { frame, curve }) | (Surface::Extruded { frame, curve }, Surface::Plane(f)) => {
            plane_across_extrusion(f, frame, curve)
        }
        (Surface::Plane(f), Surface::Revolved { frame, curve }) | (Surface::Revolved { frame, curve }, Surface::Plane(f)) => {
            plane_through_revolution(f, frame, curve, size)
        }
        _ => Ssi::General,
    }
}

// ----- General contour tracer -----

fn surface_feature(s: &Surface) -> f64 {
    match *s {
        Surface::Plane(_) => f64::INFINITY,
        Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } | Surface::Cone { radius, .. } => radius,
        Surface::Torus { minor, .. } => minor,
        Surface::Extruded { ref curve, .. } => curve.min_radius(),
        Surface::Revolved { ref curve, .. } => curve.min_radius().min(curve.reach),
        Surface::Free { ref patch, .. } => patch.min_radius(),
    }
}

/// Why a trace at one resolution could not be trusted.
enum Trouble {
    /// Something finer than the grid is going on. A finer grid may resolve it.
    TooCoarse(String),
    Fatal(Error),
}

/// Newton's method on the gradient of `f = b.sdf(a(uv))`, looking for a critical
/// point of the distance. Where `f` is also zero, the surfaces are tangent.
fn critical_point(a: &Surface, chart: &Chart, b: &Surface, start: Vec2, limit: f64) -> Option<(Vec2, f64)> {
    let grad = |uv: Vec2| {
        let (p, sx, sy) = a.chart_derivs(chart, uv);
        let n = b.normal(p);
        v2(n.dot(sx), n.dot(sy))
    };
    let mut uv = start;
    for _ in 0..12 {
        let g = grad(uv);
        if g.norm() < 1e-9 {
            return Some((uv, b.sdf(a.from_chart(chart, uv))));
        }
        let h = limit.max(1e-6) * 1e-5;
        let (gx, gy) = (grad(v2(uv.x + h, uv.y)), grad(v2(uv.x, uv.y + h)));
        let (xx, xy, yx, yy) = ((gx.x - g.x) / h, (gx.y - g.y) / h, (gy.x - g.x) / h, (gy.y - g.y) / h);
        let det = xx * yy - xy * yx;
        if det.abs() < 1e-12 {
            return None;
        }
        let step = v2(-(yy * g.x - yx * g.y) / det, -(xx * g.y - xy * g.x) / det);
        uv += step;
        if uv.dist(start) > limit {
            return None;
        }
    }
    None
}

/// Traces the zero contour of `b`'s distance over the chart window `lo..hi` of
/// surface `a`, and returns the curves refined onto both surfaces.
///
/// `relevant` says whether a point matters to the faces being intersected, so that
/// a tangency somewhere else on the surfaces is no reason to give up.
pub fn contour(
    a: &Surface,
    chart: &Chart,
    lo: Vec2,
    hi: Vec2,
    b: &Surface,
    region: &Aabb,
    relevant: &dyn Fn(Vec3) -> bool,
) -> Result<Vec<SsiCurve>> {
    let mut scale = 1.0;
    for attempt in 0..3 {
        match contour_at(a, chart, lo, hi, b, region, relevant, scale) {
            Ok(curves) => return Ok(curves),
            Err(Trouble::Fatal(e)) => return Err(e),
            Err(Trouble::TooCoarse(why)) => {
                if attempt == 2 {
                    return Err(Error::NearDegenerate(why));
                }
                scale *= 0.25;
            }
        }
    }
    unreachable!()
}

#[allow(clippy::too_many_arguments)]
fn contour_at(
    a: &Surface,
    chart: &Chart,
    lo: Vec2,
    hi: Vec2,
    b: &Surface,
    region: &Aabb,
    relevant: &dyn Fn(Vec3) -> bool,
    scale: f64,
) -> std::result::Result<Vec<SsiCurve>, Trouble> {
    let feature = surface_feature(a).min(surface_feature(b)).max(1e-3);
    let span = (hi.x - lo.x).max(hi.y - lo.y).max(1e-9);
    // Cells small enough that the contour is nearly straight across each one.
    let cell = (feature / 6.0).min(span / 8.0).max(span / 512.0) * scale;
    // Two branches near a tangency sit about sqrt(2 * gap * radius) apart. If that
    // is under a cell, this grid can't tell how they connect.
    let resolvable_gap = cell * cell / feature;
    let margin = cell * 1.5;
    let (lo, hi) = (v2(lo.x - margin, lo.y - margin), v2(hi.x + margin, hi.y + margin));
    let nx = (((hi.x - lo.x) / cell).ceil() as i64).max(1);
    let ny = (((hi.y - lo.y) / cell).ceil() as i64).max(1);
    let (dx, dy) = ((hi.x - lo.x) / nx as f64, (hi.y - lo.y) / ny as f64);
    let corner = |i: i64, j: i64| v2(lo.x + dx * i as f64, lo.y + dy * j as f64);

    // A quadtree over the grid. The certificate discards cells the contour cannot
    // enter, since 3D distance never exceeds chart distance.
    let region = region.inflated(dx.hypot(dy));
    let mut active: Vec<(i64, i64)> = Vec::new();
    let levels = (nx.max(ny) as f64).log2().ceil() as u32;
    let top = 1i64 << levels;
    let mut stack = vec![(0i64, 0i64, top)];
    while let Some((i, j, n)) = stack.pop() {
        if i >= nx || j >= ny {
            continue;
        }
        let centre = corner(i, j) + v2(dx * n as f64 / 2.0, dy * n as f64 / 2.0);
        let radius = (dx * n as f64).hypot(dy * n as f64) / 2.0;
        let q = a.from_chart(chart, centre);
        if b.sdf(q).abs() > radius || !region.inflated(radius).contains(q) {
            continue;
        }
        if n == 1 {
            active.push((i, j));
        } else {
            let h = n / 2;
            for (di, dj) in [(0, 0), (h, 0), (0, h), (h, h)] {
                stack.push((i + di, j + dj, h));
            }
        }
    }

    let mut values: HashMap<(i64, i64), f64> = HashMap::default();
    let mut value = |i: i64, j: i64| -> f64 {
        *values.entry((i, j)).or_insert_with(|| {
            // A zero would sit on a grid corner and confuse the sign tests.
            let v = b.sdf(a.from_chart(chart, corner(i, j)));
            if v == 0.0 {
                1e-300
            } else {
                v
            }
        })
    };

    // Marching squares. Grid edges are named by their lower corner and direction,
    // so neighbouring cells agree on each crossing.
    type GridEdge = (i64, i64, bool);
    let mut links: HashMap<GridEdge, Vec<GridEdge>> = HashMap::default();
    let mut quiet_cells: Vec<((i64, i64), f64)> = Vec::new();
    let mut crossing: HashMap<GridEdge, Vec2> = HashMap::default();
    for &(i, j) in &active {
        let (v00, v10, v11, v01) = (value(i, j), value(i + 1, j), value(i + 1, j + 1), value(i, j + 1));
        let mut found: Vec<GridEdge> = Vec::new();
        let mut note = |edge: GridEdge, p: Vec2, q: Vec2, vp: f64, vq: f64| {
            if vp * vq < 0.0 {
                found.push(edge);
                crossing.entry(edge).or_insert_with(|| p + (q - p) * (vp / (vp - vq)));
            }
        };
        note((i, j, true), corner(i, j), corner(i + 1, j), v00, v10);
        note((i + 1, j, false), corner(i + 1, j), corner(i + 1, j + 1), v10, v11);
        note((i, j + 1, true), corner(i, j + 1), corner(i + 1, j + 1), v01, v11);
        note((i, j, false), corner(i, j), corner(i, j + 1), v00, v01);
        match found.len() {
            0 => quiet_cells.push(((i, j), v00)),
            2 => {
                links.entry(found[0]).or_default().push(found[1]);
                links.entry(found[1]).or_default().push(found[0]);
            }
            4 => {
                // A saddle: two branches pass through one cell. That only matters if
                // the point is on the faces being intersected. Elsewhere, any pairing
                // will do, because those pieces get discarded.
                let centre = corner(i, j) + v2(dx / 2.0, dy / 2.0);
                let at = critical_point(a, chart, b, centre, 2.0 * dx.hypot(dy)).map_or(centre, |(uv, _)| uv);
                if relevant(b.project(a.from_chart(chart, at))) {
                    return Err(Trouble::TooCoarse(format!("{} and {} intersect near a tangency", a.name(), b.name())));
                }
                for pair in found.as_chunks::<2>().0 {
                    links.entry(pair[0]).or_default().push(pair[1]);
                    links.entry(pair[1]).or_default().push(pair[0]);
                }
            }
            _ => {
                return Err(Trouble::Fatal(Error::Internal("odd crossing count in a contour cell".into())));
            }
        }
    }

    // A cell can hide a small closed loop: no sign change at its corners, but the
    // distance dips through zero inside. Look for the dip directly.
    for &((i, j), corner_value) in &quiet_cells {
        let centre = corner(i, j) + v2(dx / 2.0, dy / 2.0);
        if let Some((uv, f0)) = critical_point(a, chart, b, centre, 2.0 * dx.hypot(dy)) {
            // Within tolerance of zero it is a touching point, with no loop to find.
            if f0 * corner_value < 0.0 && f0.abs() > TOL && relevant(b.project(a.from_chart(chart, uv))) {
                return Err(Trouble::TooCoarse(format!(
                    "{} and {} meet in a loop too small to trace",
                    a.name(),
                    b.name()
                )));
            }
        }
    }

    // Walk the links into chains and loops.
    let mut used: HashMap<GridEdge, bool> = HashMap::default();
    let mut chains: Vec<(Vec<GridEdge>, bool)> = Vec::new();
    let mut starts: Vec<GridEdge> = links.iter().filter(|(_, v)| v.len() == 1).map(|(k, _)| *k).collect();
    starts.sort();
    let mut all: Vec<GridEdge> = links.keys().copied().collect();
    all.sort();
    starts.extend(all);
    for s in starts {
        if used.contains_key(&s) {
            continue;
        }
        let mut chain = vec![s];
        used.insert(s, true);
        let mut cur = s;
        while let Some(n) = links[&cur].iter().find(|e| !used.contains_key(*e)).copied() {
            used.insert(n, true);
            chain.push(n);
            cur = n;
        }
        let closed = chain.len() > 2 && links[&cur].contains(&s);
        chains.push((chain, closed));
    }

    let mut out = Vec::new();
    for (chain, closed) in chains {
        let mut points: Vec<Vec3> = Vec::with_capacity(chain.len() + 1);
        for edge in &chain {
            let guess = a.from_chart(chart, crossing[edge]);
            let Some(p) = refine_on_two(a, b, guess) else {
                if relevant(b.project(guess)) {
                    return Err(Trouble::TooCoarse(format!(
                        "{} and {} are tangent along their intersection",
                        a.name(),
                        b.name()
                    )));
                }
                continue;
            };
            if points.last().is_none_or(|q: &Vec3| q.dist(p) > 1e-9) {
                points.push(p);
            }
        }
        if closed {
            let first = points[0];
            if points.last().unwrap().dist(first) > 1e-9 {
                points.push(first);
            } else {
                let n = points.len();
                points[n - 1] = first;
            }
        }
        if points.len() < 2 {
            continue;
        }
        // Where the surfaces come close to tangent along this curve, look for the
        // exact point of tangency. A curve through such a point crosses another
        // branch there, and only a grid fine enough to separate them is trustworthy.
        let sine = |p: Vec3| a.normal(p).cross(b.normal(p)).norm();
        let sines: Vec<f64> = points.iter().map(|&p| sine(p)).collect();
        for i in 0..points.len() {
            let before = if i > 0 { sines[i - 1] } else { f64::INFINITY };
            let after = if i + 1 < points.len() { sines[i + 1] } else { f64::INFINITY };
            if sines[i] < 0.6 && sines[i] <= before && sines[i] <= after {
                let start = a.to_chart(chart, points[i]);
                if let Some((uv, f0)) = critical_point(a, chart, b, start, 4.0 * dx.hypot(dy)) {
                    if f0.abs() <= resolvable_gap && relevant(b.project(a.from_chart(chart, uv))) {
                        return Err(Trouble::TooCoarse(format!(
                            "{} and {} are tangent at a point on their intersection",
                            a.name(),
                            b.name()
                        )));
                    }
                }
            }
        }
        let curve = sampled_from_points(points, a, b).map_err(Trouble::Fatal)?;
        let n = curve_len(&curve);
        out.push(SsiCurve { curve, closed, t0: 0.0, t1: (n - 1) as f64 });
    }
    Ok(out)
}

fn curve_len(c: &Curve) -> usize {
    match c {
        Curve::Sampled(s) => s.points.len(),
        _ => 0,
    }
}

/// Largest turn between neighbouring samples. Everything that uses a sampled curve,
/// from volume integrals to mesh edges, needs samples dense enough to follow it.
const MAX_SAMPLE_TURN: f64 = 12.0 * PI / 180.0;

/// Tangents along a sampled curve. The cross product of the two normals is continuous
/// along a smooth curve and never vanishes, so one sign fits the whole curve.
fn curve_tangents(points: &[Vec3], a: &Surface, b: &Surface) -> Result<Vec<Vec3>> {
    let mut tangents = Vec::with_capacity(points.len());
    for &p in points {
        let t = a.normal(p).cross(b.normal(p));
        if t.norm() < 1e-7 {
            return Err(Error::NearDegenerate(format!(
                "{} and {} are tangent along their intersection",
                a.name(),
                b.name()
            )));
        }
        tangents.push(t.normalized());
    }
    let mut agreement = 0.0;
    for (w, t) in points.windows(2).zip(&tangents) {
        agreement += (w[1] - w[0]).dot(*t);
    }
    if agreement < 0.0 {
        for t in &mut tangents {
            *t = -*t;
        }
    }
    Ok(tangents)
}

/// Builds a sampled curve through points on both surfaces, adding points wherever the
/// curve turns too fast between samples. Tight spots such as the tip of a hyperbola on
/// a slender cone can be far smaller than anything the tracer's grid sees.
pub fn sampled_from_points(points: Vec<Vec3>, a: &Surface, b: &Surface) -> Result<Curve> {
    let mut points = points;
    let limit = MAX_SAMPLE_TURN.cos();
    for _pass in 0..8 {
        let tangents = curve_tangents(&points, a, b)?;
        let mut denser = Vec::with_capacity(points.len());
        let mut changed = false;
        for i in 0..points.len() - 1 {
            denser.push(points[i]);
            let (p, q) = (points[i], points[i + 1]);
            let chord = p.dist(q);
            if tangents[i].dot(tangents[i + 1]) < limit && chord > 1e-5 {
                // Cubic Hermite midpoint, snapped back onto both surfaces.
                let mid = (p + q) * 0.5 + (tangents[i] - tangents[i + 1]) * (chord / 8.0);
                if let Some(m) = refine_on_two(a, b, mid) {
                    if m.dist(p) > 1e-6 && m.dist(q) > 1e-6 {
                        denser.push(m);
                        changed = true;
                    }
                }
            }
        }
        denser.push(points[points.len() - 1]);
        points = denser;
        if !changed {
            break;
        }
    }
    let tangents = curve_tangents(&points, a, b)?;
    Ok(Curve::Sampled(Arc::new(SampledCurve { points, tangents, surfaces: (a.clone(), b.clone()) })))
}
