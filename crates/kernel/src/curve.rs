//! Edge geometry. Lines, circles and ellipses are exact. General surface-surface
//! intersection curves have no closed form, so they are stored as refined
//! samples that are snapped back onto both defining surfaces on evaluation.

use crate::math::*;
use crate::surface::{refine_on_two, Surface};
use std::sync::Arc;

#[derive(Clone, Debug)]
pub enum Curve {
    /// `t` is arc length; `dir` is a unit vector.
    Line { origin: Vec3, dir: Vec3 },
    /// `t` is the angle in the frame's XY plane.
    Circle { frame: Frame, radius: f64 },
    /// `t` is the eccentric angle.
    Ellipse { frame: Frame, major: f64, minor: f64 },
    /// `t` runs over `[0, n-1]`, one unit per sample.
    Sampled(Arc<SampledCurve>),
}

/// An intersection curve between two surfaces, held as samples on both.
#[derive(Clone, Debug)]
pub struct SampledCurve {
    pub points: Vec<Vec3>,
    /// Unit tangents in the direction of increasing `t`.
    pub tangents: Vec<Vec3>,
    pub surfaces: (Surface, Surface),
}

impl SampledCurve {
    /// Cubic Hermite through segment `i`, at local coordinate `s`. It is evaluated
    /// without clamping, so it can be used a little past its ends.
    fn hermite_local(&self, i: usize, s: f64) -> Vec3 {
        let (a, b) = (self.points[i], self.points[i + 1]);
        let chord = a.dist(b);
        let (ta, tb) = (self.tangents[i] * chord, self.tangents[i + 1] * chord);
        let (s2, s3) = (s * s, s * s * s);
        a * (2.0 * s3 - 3.0 * s2 + 1.0) + ta * (s3 - 2.0 * s2 + s) + b * (-2.0 * s3 + 3.0 * s2) + tb * (s3 - s2)
    }

    fn segment(&self, t: f64) -> (usize, f64) {
        let n = self.points.len();
        let t = t.clamp(0.0, (n - 1) as f64);
        let i = (t.floor() as usize).min(n - 2);
        (i, t - i as f64)
    }

    fn snapped(&self, i: usize, s: f64) -> Vec3 {
        let p = self.hermite_local(i, s);
        refine_on_two(&self.surfaces.0, &self.surfaces.1, p).unwrap_or(p)
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        let (i, s) = self.segment(t);
        self.snapped(i, s)
    }

    /// Derivative of the snapped curve. The stencil stays on one segment's
    /// polynomial, which is smooth, so it never sees the kinks between segments.
    pub fn derivative(&self, t: f64) -> Vec3 {
        let (i, s) = self.segment(t);
        let h = 1e-3;
        (self.snapped(i, s - 2.0 * h) - self.snapped(i, s + 2.0 * h)
            + (self.snapped(i, s + h) - self.snapped(i, s - h)) * 8.0)
            / (12.0 * h)
    }

    /// Upper bound on the parametric speed.
    pub fn speed_bound(&self) -> f64 {
        // A cubic Hermite segment with tangent lengths equal to its chord has
        // speed at most 1.5 * chord, plus a margin for the snap to the surfaces.
        let chord = self.points.windows(2).map(|w| w[0].dist(w[1])).fold(0.0, f64::max);
        chord * 1.6
    }
}

impl Curve {
    pub fn line_through(a: Vec3, b: Vec3) -> Curve {
        Curve::Line { origin: a, dir: (b - a).normalized() }
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        match self {
            Curve::Line { origin, dir } => *origin + *dir * t,
            Curve::Circle { frame, radius } => {
                let (s, c) = t.sin_cos();
                frame.origin + frame.x * (radius * c) + frame.y * (radius * s)
            }
            Curve::Ellipse { frame, major, minor } => {
                let (s, c) = t.sin_cos();
                frame.origin + frame.x * (major * c) + frame.y * (minor * s)
            }
            Curve::Sampled(c) => c.eval(t),
        }
    }

    /// Derivative with respect to `t`.
    pub fn derivative(&self, t: f64) -> Vec3 {
        match self {
            Curve::Line { dir, .. } => *dir,
            Curve::Circle { frame, radius } => {
                let (s, c) = t.sin_cos();
                frame.y * (radius * c) - frame.x * (radius * s)
            }
            Curve::Ellipse { frame, major, minor } => {
                let (s, c) = t.sin_cos();
                frame.y * (minor * c) - frame.x * (major * s)
            }
            Curve::Sampled(c) => c.derivative(t),
        }
    }

    /// Upper bound on `|d(eval)/dt|`.
    pub fn speed_bound(&self) -> f64 {
        match self {
            Curve::Line { .. } => 1.0,
            Curve::Circle { radius, .. } => *radius,
            Curve::Ellipse { major, .. } => *major,
            Curve::Sampled(c) => c.speed_bound(),
        }
    }

    /// The parameter step that corresponds to at most `dist` of travel.
    pub fn param_step_for_distance(&self, dist: f64) -> f64 {
        dist / self.speed_bound()
    }

    /// Whether the curve is periodic in `t` with period 2*PI.
    pub fn is_periodic(&self) -> bool {
        matches!(self, Curve::Circle { .. } | Curve::Ellipse { .. })
    }

    /// Parameter of the closest point within `[t0, t1]`, and the distance to it.
    pub fn closest(&self, p: Vec3, t0: f64, t1: f64) -> (f64, f64) {
        let best = |t: f64| {
            let t = t.clamp(t0, t1);
            (t, self.eval(t).dist(p))
        };
        let pick = |a: (f64, f64), b: (f64, f64)| if b.1 < a.1 { b } else { a };
        match self {
            Curve::Line { origin, dir } => best((p - *origin).dot(*dir)),
            Curve::Circle { frame, .. } => {
                let l = frame.to_local(p);
                let a = l.y.atan2(l.x);
                // Bring the angle into the interval if any period lands there.
                let k = ((t0 + t1) / 2.0 - a) / TAU;
                let t = a + k.round() * TAU;
                pick(pick(best(t), best(t0)), best(t1))
            }
            Curve::Ellipse { .. } => {
                let mut result = best(t0);
                let steps = 64;
                for i in 0..=steps {
                    let t = t0 + (t1 - t0) * i as f64 / steps as f64;
                    result = pick(result, best(t));
                }
                let mut h = (t1 - t0) / steps as f64;
                for _ in 0..50 {
                    for t in [result.0 - h, result.0 + h] {
                        result = pick(result, best(t));
                    }
                    h *= 0.6;
                }
                result
            }
            Curve::Sampled(c) => {
                // Find the best chord, then polish on the true curve.
                let (lo, hi) = (t0.max(0.0), t1.min((c.points.len() - 1) as f64));
                let mut result = best(lo);
                let first = lo.floor() as usize;
                let last = (hi.ceil() as usize).min(c.points.len() - 1);
                for i in first..last {
                    let (a, b) = (c.points[i], c.points[i + 1]);
                    let d = b - a;
                    let s = ((p - a).dot(d) / d.norm2()).clamp(0.0, 1.0);
                    let t = (i as f64 + s).clamp(lo, hi);
                    if (a + d * s).dist(p) < result.1 + d.norm() {
                        result = pick(result, best(t));
                    }
                }
                let mut h = 0.25;
                for _ in 0..40 {
                    for t in [result.0 - h, result.0 + h] {
                        result = pick(result, best(t));
                    }
                    h *= 0.6;
                }
                result
            }
        }
    }

    /// Rigidly transforms the curve.
    /// The image under a reflection. A parameter still means the same point.
    pub fn mirrored(&self, m: &Mirror) -> Curve {
        match self {
            Curve::Line { origin, dir } => Curve::Line { origin: m.of_point(*origin), dir: m.of_dir(*dir) },
            Curve::Circle { frame, radius } => Curve::Circle { frame: m.of_curve_frame(frame), radius: *radius },
            Curve::Ellipse { frame, major, minor } => Curve::Ellipse { frame: m.of_curve_frame(frame), major: *major, minor: *minor },
            Curve::Sampled(c) => Curve::Sampled(Arc::new(SampledCurve {
                points: c.points.iter().map(|p| m.of_point(*p)).collect(),
                tangents: c.tangents.iter().map(|d| m.of_dir(*d)).collect(),
                surfaces: (c.surfaces.0.mirrored(m), c.surfaces.1.mirrored(m)),
            })),
        }
    }

    pub fn transformed(&self, t: &Frame) -> Curve {
        match self {
            Curve::Line { origin, dir } => Curve::Line { origin: t.to_world(*origin), dir: t.dir_to_world(*dir) },
            Curve::Circle { frame, radius } => Curve::Circle { frame: t.compose(frame), radius: *radius },
            Curve::Ellipse { frame, major, minor } => {
                Curve::Ellipse { frame: t.compose(frame), major: *major, minor: *minor }
            }
            Curve::Sampled(c) => Curve::Sampled(Arc::new(SampledCurve {
                points: c.points.iter().map(|p| t.to_world(*p)).collect(),
                tangents: c.tangents.iter().map(|d| t.dir_to_world(*d)).collect(),
                surfaces: (c.surfaces.0.transformed(t), c.surfaces.1.transformed(t)),
            })),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_derivatives_match_differences() {
        let f = Frame::new(v3(1.0, 2.0, 3.0), v3(0.2, 0.3, 1.0), Vec3::X);
        let curves = [
            Curve::line_through(v3(0.0, 0.0, 0.0), v3(1.0, 2.0, 3.0)),
            Curve::Circle { frame: f, radius: 3.0 },
            Curve::Ellipse { frame: f, major: 5.0, minor: 2.0 },
        ];
        for c in &curves {
            for i in 0..20 {
                let t = i as f64 * 0.3;
                let h = 1e-6;
                let fd = (c.eval(t + h) - c.eval(t - h)) / (2.0 * h);
                assert!((fd - c.derivative(t)).norm() < 1e-7);
                assert!(c.derivative(t).norm() <= c.speed_bound() + 1e-12);
            }
        }
    }

    #[test]
    fn closest_point_on_arc_respects_the_interval() {
        let c = Curve::Circle { frame: Frame::WORLD, radius: 2.0 };
        let (t, d) = c.closest(v3(0.0, 3.0, 0.0), 0.0, PI);
        assert!((t - PI / 2.0).abs() < 1e-12 && (d - 1.0).abs() < 1e-12);
        // The point lies beyond the end of the arc, so an endpoint must win.
        let (t, _) = c.closest(v3(0.0, -3.0, 0.0), 0.0, 1.0);
        assert!(t.abs() < 1e-12);
        // Interval above 2*PI: the angle has to wrap into it.
        let (t, d) = c.closest(v3(2.0, 0.0, 0.0), TAU - 0.5, TAU + 0.5);
        assert!((t - TAU).abs() < 1e-12 && d < 1e-12);
    }
}
