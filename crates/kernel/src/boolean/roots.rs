//! Where does a curve meet a surface?
//!
//! The surface's distance function is 1-Lipschitz, so an interval of the curve
//! whose endpoints are both farther from the surface than the interval is long
//! cannot touch it. That certificate discards most of the curve cheaply and with
//! no risk of missing anything. What remains is split at the extrema of the
//! distance along the curve, and each monotone piece has at most one root.

use crate::curve::Curve;
use crate::math::*;
use crate::surface::Surface;

#[derive(Clone, Debug, PartialEq)]
pub enum Contact {
    /// The whole curve piece lies on the surface.
    OnSurface,
    /// Isolated contacts: crossings and touching points.
    Roots(Vec<f64>),
}

fn curve_feature(curve: &Curve) -> f64 {
    match curve {
        Curve::Line { .. } => f64::INFINITY,
        Curve::Circle { radius, .. } => *radius,
        Curve::Ellipse { major, minor, .. } => minor * minor / major,
        Curve::Sampled(c) => c.speed_bound() * 2.5,
    }
}

fn surface_feature(s: &Surface, p: Vec3) -> f64 {
    match *s {
        Surface::Plane(_) => f64::INFINITY,
        Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => radius,
        Surface::Cone { frame, .. } => {
            let l = frame.to_local(p);
            l.x.hypot(l.y).max(1e-3)
        }
        Surface::Torus { minor, .. } => minor,
    }
}

/// Finds the contacts of a curve piece with a surface. A contact is any point
/// where the distance drops to `tol` or crosses zero.
pub fn curve_surface_contact(curve: &Curve, t0: f64, t1: f64, surf: &Surface, tol: f64) -> Contact {
    let dist = |t: f64| surf.sdf(curve.eval(t));
    let slope = |t: f64| {
        let p = curve.eval(t);
        surf.normal(p).dot(curve.derivative(t))
    };

    // Is the whole piece on the surface? Fractions are irregular so that a curve
    // crossing at a symmetric spot cannot look like contact everywhere.
    let probes = [0.0, 0.07, 0.19, 0.31, 0.43, 0.5, 0.61, 0.73, 0.87, 1.0];
    if probes.iter().all(|f| dist(t0 + (t1 - t0) * f).abs() <= tol) {
        return Contact::OnSurface;
    }

    // A line and a plane need no search.
    if let (Curve::Line { origin, dir }, Surface::Plane(frame)) = (curve, surf) {
        let denom = dir.dot(frame.z);
        let mut roots = Vec::new();
        if denom != 0.0 {
            let t = (frame.origin - *origin).dot(frame.z) / denom;
            if t >= t0 && t <= t1 {
                roots.push(t);
            }
        }
        if roots.is_empty() {
            // The piece can still end within tolerance of the plane.
            for t in [t0, t1] {
                if dist(t).abs() <= tol {
                    roots.push(t);
                }
            }
        }
        return Contact::Roots(roots);
    }

    let speed = curve.speed_bound();
    let cf = curve_feature(curve);
    let floor = ((t1 - t0) * speed / 4096.0).max(1e-6);

    let mut found: Vec<(f64, bool)> = Vec::new(); // (parameter, is a touching extremum)
    let mut stack = vec![(t0, t1, dist(t0), dist(t1))];
    let mut budget = 200_000usize;
    while let Some((a, b, sa, sb)) = stack.pop() {
        if budget == 0 {
            break;
        }
        budget -= 1;
        let length = (b - a) * speed;
        if sa * sb > 0.0 && (sa.abs() + sb.abs() - length) / 2.0 > tol {
            continue;
        }
        let m = (a + b) / 2.0;
        let leaf = (0.4 * cf.min(surface_feature(surf, curve.eval(m)))).max(floor);
        if length > leaf {
            let sm = dist(m);
            stack.push((a, m, sa, sm));
            stack.push((m, b, sm, sb));
            continue;
        }
        // Leaf: break at extrema so each piece is monotone.
        let mut breaks = vec![a];
        let probes = [a, m, b];
        let slopes = [slope(a), slope(m), slope(b)];
        for i in 0..2 {
            let (x, y) = (probes[i], probes[i + 1]);
            let (dx, dy) = (slopes[i], slopes[i + 1]);
            if dx == 0.0 {
                breaks.push(x);
            } else if dx * dy < 0.0 {
                let e = bisect(&slope, x, y, dx, speed);
                breaks.push(e);
                if dist(e).abs() <= tol {
                    found.push((e, true));
                }
            }
        }
        if slopes[2] == 0.0 {
            breaks.push(b);
        }
        breaks.push(b);
        breaks.dedup();
        for w in breaks.windows(2) {
            let (x, y) = (w[0], w[1]);
            let (dx, dy) = (dist(x), dist(y));
            if dx == 0.0 {
                found.push((x, false));
            } else if dx * dy < 0.0 {
                found.push((bisect(&dist, x, y, dx, speed), false));
            }
        }
        if sb == 0.0 {
            found.push((b, false));
        }
    }

    // Near a tangency two crossings sit a hair apart with the curve within
    // tolerance between them. That is one contact, located at the extremum.
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, bool)> = Vec::new();
    for &(t, touch) in &found {
        if let Some(last) = merged.last_mut() {
            let mid = (last.0 + t) / 2.0;
            let close = (t - last.0) * speed <= 2.0 * tol;
            if close || (dist(mid).abs() <= tol && dist(t).abs() <= tol && dist(last.0).abs() <= tol) {
                if touch && !last.1 {
                    *last = (t, true);
                }
                continue;
            }
        }
        merged.push((t, touch));
    }
    Contact::Roots(merged.into_iter().map(|r| r.0).collect())
}

/// Finds a sign change of `f` in `[a, b]`, given `f(a)`.
fn bisect(f: &impl Fn(f64) -> f64, a: f64, b: f64, fa: f64, speed: f64) -> f64 {
    let (mut lo, mut hi) = (a, b);
    let lo_positive = fa > 0.0;
    for _ in 0..80 {
        let m = (lo + hi) / 2.0;
        if (hi - lo) * speed < 1e-14 {
            return m;
        }
        let v = f(m);
        if v == 0.0 {
            return m;
        }
        if (v > 0.0) == lo_positive {
            lo = m;
        } else {
            hi = m;
        }
    }
    (lo + hi) / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roots(c: Contact) -> Vec<f64> {
        match c {
            Contact::Roots(r) => r,
            Contact::OnSurface => panic!("unexpected on-surface"),
        }
    }

    #[test]
    fn line_and_cylinder_crossing_tangent_and_miss() {
        let cyl = Surface::Cylinder {
            frame: Frame::WORLD,
            radius: 2.0,
        };
        let through = Curve::line_through(v3(-5.0, 0.0, 0.0), v3(5.0, 0.0, 0.0));
        let r = roots(curve_surface_contact(&through, 0.0, 10.0, &cyl, TOL));
        assert_eq!(r.len(), 2);
        assert!((r[0] - 3.0).abs() < 1e-10 && (r[1] - 7.0).abs() < 1e-10);

        let tangent = Curve::line_through(v3(-5.0, 2.0, 0.0), v3(5.0, 2.0, 0.0));
        let r = roots(curve_surface_contact(&tangent, 0.0, 10.0, &cyl, TOL));
        assert_eq!(r.len(), 1, "{r:?}");
        assert!((r[0] - 5.0).abs() < 1e-9, "{r:?}");

        let miss = Curve::line_through(v3(-5.0, 2.1, 0.0), v3(5.0, 2.1, 0.0));
        assert!(roots(curve_surface_contact(&miss, 0.0, 10.0, &cyl, TOL)).is_empty());
    }

    #[test]
    fn curves_lying_on_surfaces_are_detected() {
        let cyl = Surface::Cylinder {
            frame: Frame::WORLD,
            radius: 2.0,
        };
        let ruling = Curve::line_through(v3(2.0, 0.0, -1.0), v3(2.0, 0.0, 1.0));
        assert_eq!(curve_surface_contact(&ruling, 0.0, 2.0, &cyl, TOL), Contact::OnSurface);
        let rim = Curve::Circle {
            frame: Frame::translation(v3(0.0, 0.0, 4.0)),
            radius: 2.0,
        };
        assert_eq!(curve_surface_contact(&rim, 0.0, TAU, &cyl, TOL), Contact::OnSurface);
    }

    #[test]
    fn circle_and_every_surface() {
        let circle = Curve::Circle {
            frame: Frame::new(v3(0.0, 0.0, 0.0), Vec3::X, Vec3::Y),
            radius: 5.0,
        };
        // The circle lies in the YZ plane. A plane through its centre cuts it twice.
        let plane = Surface::Plane(Frame::WORLD);
        assert_eq!(roots(curve_surface_contact(&circle, 0.0, TAU, &plane, TOL)).len(), 2);
        // A torus about X with the circle as its core misses it entirely.
        let core = Surface::Torus {
            frame: Frame::new(v3(0.0, 0.0, 0.0), Vec3::X, Vec3::Y),
            major: 5.0,
            minor: 1.0,
        };
        assert!(roots(curve_surface_contact(&circle, 0.0, TAU, &core, TOL)).is_empty());
        // A torus about Z of major 5, minor 1 meets the circle where it passes
        // through the tube: four crossings.
        let crossing = Surface::Torus {
            frame: Frame::WORLD,
            major: 5.0,
            minor: 1.0,
        };
        assert_eq!(roots(curve_surface_contact(&circle, 0.0, TAU, &crossing, TOL)).len(), 4);
    }
}
