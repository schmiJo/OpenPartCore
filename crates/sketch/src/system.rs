//! Turns a sketch into variables and equations, and evaluates them.
//!
//! Every variable is a length: coordinates and radii, never angles. So "move as little as
//! possible" has one meaning across the whole sketch. Every equation's value is a length too,
//! measured in millimetres of error at the size of the geometry involved, so one tolerance
//! applies everywhere.

use crate::dual::{D, V2};
use crate::expr::{Env, Expr};
use crate::model::*;
use crate::solve::{ErrorKind, SketchError};
use std::collections::HashMap;

/// Most variables one equation uses.
pub(crate) const MAX_VARS: usize = 12;

/// What one equation computes. The comments list its variables, in order, as points where
/// that applies.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Eq {
    /// Up to four variables: `sum(coef * v) - value`.
    Linear { coef: [f64; 4], value: f64 },
    /// a, b: `|b - a| - value`.
    Dist { value: f64 },
    /// a, b, r: `|b - a| - r - gap`.
    DistToRadius { gap: f64 },
    /// a0, a1, b0, b1: `|a1 - a0| - |b1 - b0|`.
    EqualLength,
    /// p, a, b: the signed distance from p to line ab, or its size, minus `value`.
    PointLine { value: f64, unsigned: bool },
    /// p, a, b, r: `|distance from p to line ab| - r - gap`.
    PointLineRadius { gap: f64 },
    /// a0, a1, b0, b1: the sine of the angle between the directions, at `scale`.
    Parallel { scale: f64 },
    /// a0, a1, b0, b1: the cosine of the angle between the directions, at `scale`.
    Perpendicular { scale: f64 },
    /// a0, a1, b0, b1: the angle from a to b minus `theta`, wrapped, at `scale`.
    Angle { theta: f64, scale: f64 },
    /// c, s, e: the angle from cs to ce minus `theta`, wrapped, at `scale`.
    ArcSpan { theta: f64, scale: f64 },
    /// c0, c1, r0, r1: `|c1 - c0| - s0 r0 - s1 r1 - gap`.
    CircleTangent { s0: f64, s1: f64, gap: f64 },
    /// c0, c1, r0, r1: `|c1 - c0| - sqrt(r0^2 + r1^2)`.
    CircleNormal,
    /// p, q, a, b: the distance from the midpoint of pq to line ab, signed or its size, minus `value`.
    MidpointLine { value: f64, unsigned: bool },
    /// p, q, a, b: `(q - p) · direction of ab`.
    PerpendicularToLine,
    /// p, then an ellipse's center, major end and minor radius: how far p is off the ellipse, to first
    /// order and at `scale`, since an ellipse's true distance has no closed form.
    PointEllipse { scale: f64 },
    /// a, b, then an ellipse: the line through ab tangent to it. The distance from the center to the line,
    /// against how far the ellipse reaches in that direction, which is exact.
    LineTangentEllipse,
    /// p, then an ellipse, then a, b: at a point on both, the line through ab square to the ellipse's normal.
    LineTangentEllipseAt { scale: f64 },
    /// p, then an ellipse, then a circle's center: at a point on both, the radius along the ellipse's normal.
    CircleTangentEllipseAt { scale: f64 },
    /// p, then two ellipses: at a point on both, their normals in line.
    EllipsesTangentAt { scale: f64 },
}

/// One scalar equation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Row {
    /// Index of the constraint this came from, or `None` for an arc's own geometry.
    pub owner: Option<usize>,
    pub eq: Eq,
    pub vars: [u32; MAX_VARS],
    pub count: u8,
}

fn wrap(a: f64) -> f64 {
    let turn = std::f64::consts::TAU;
    a - turn * (a / turn).round()
}

/// Below this length a direction is taken to be undefined.
const TINY: f64 = 1e-14;

impl Row {
    /// The equation's value, with its gradient written into `g` in the order of `vars`.
    pub fn eval(&self, x: &[f64], g: &mut [f64; MAX_VARS]) -> f64 {
        let v = |i: usize| x[self.vars[i] as usize];
        let p = |i: usize| [v(i), v(i + 1)];
        match self.eq {
            Eq::Linear { coef, value } => {
                let mut sum = -value;
                for i in 0..self.count as usize {
                    sum += coef[i] * v(i);
                    g[i] = coef[i];
                }
                sum
            }
            Eq::Dist { value } => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                n - value
            }
            Eq::DistToRadius { gap } => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                g[4] = -1.0;
                n - v(4) - gap
            }
            Eq::EqualLength => {
                let (n0, u0) = length_and_unit(p(0), p(2));
                let (n1, u1) = length_and_unit(p(4), p(6));
                set(g, 0, [-u0[0], -u0[1]]);
                set(g, 2, u0);
                set(g, 4, u1);
                set(g, 6, [-u1[0], -u1[1]]);
                n0 - n1
            }
            Eq::PointLine { value, unsigned } => {
                let s = point_line(p(0), p(2), p(4), g, 0, 2, 4, 1.0);
                if unsigned && s < 0.0 {
                    for gi in g.iter_mut().take(6) {
                        *gi = -*gi;
                    }
                    -s - value
                } else {
                    s - value
                }
            }
            Eq::PointLineRadius { gap } => {
                let s = point_line(p(0), p(2), p(4), g, 0, 2, 4, 1.0);
                g[6] = -1.0;
                if s < 0.0 {
                    for gi in g.iter_mut().take(6) {
                        *gi = -*gi;
                    }
                    -s - v(6) - gap
                } else {
                    s - v(6) - gap
                }
            }
            Eq::Parallel { scale } => {
                let (u, v2) = (diff(p(0), p(2)), diff(p(4), p(6)));
                let (nu, nv) = (norm2(u), norm2(v2));
                if nu < TINY * TINY || nv < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let m = (nu * nv).sqrt();
                let s = (u[0] * v2[1] - u[1] * v2[0]) / m;
                let du = [(v2[1] / m - s * u[0] / nu) * scale, (-v2[0] / m - s * u[1] / nu) * scale];
                let dv = [(-u[1] / m - s * v2[0] / nv) * scale, (u[0] / m - s * v2[1] / nv) * scale];
                segments(g, du, dv);
                s * scale
            }
            Eq::Perpendicular { scale } => {
                let (u, v2) = (diff(p(0), p(2)), diff(p(4), p(6)));
                let (nu, nv) = (norm2(u), norm2(v2));
                if nu < TINY * TINY || nv < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let m = (nu * nv).sqrt();
                let t = (u[0] * v2[0] + u[1] * v2[1]) / m;
                let du = [(v2[0] / m - t * u[0] / nu) * scale, (v2[1] / m - t * u[1] / nu) * scale];
                let dv = [(u[0] / m - t * v2[0] / nv) * scale, (u[1] / m - t * v2[1] / nv) * scale];
                segments(g, du, dv);
                t * scale
            }
            Eq::Angle { theta, scale } => {
                let (u, v2) = (diff(p(0), p(2)), diff(p(4), p(6)));
                let (nu, nv) = (norm2(u), norm2(v2));
                if nu < TINY * TINY || nv < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let angle = (u[0] * v2[1] - u[1] * v2[0]).atan2(u[0] * v2[0] + u[1] * v2[1]);
                let du = [u[1] / nu * scale, -u[0] / nu * scale];
                let dv = [-v2[1] / nv * scale, v2[0] / nv * scale];
                segments(g, du, dv);
                wrap(angle - theta) * scale
            }
            Eq::ArcSpan { theta, scale } => {
                let (u, v2) = (diff(p(0), p(2)), diff(p(0), p(4)));
                let (nu, nv) = (norm2(u), norm2(v2));
                if nu < TINY * TINY || nv < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let angle = (u[0] * v2[1] - u[1] * v2[0]).atan2(u[0] * v2[0] + u[1] * v2[1]);
                let du = [u[1] / nu * scale, -u[0] / nu * scale];
                let dv = [-v2[1] / nv * scale, v2[0] / nv * scale];
                set(g, 0, [-du[0] - dv[0], -du[1] - dv[1]]);
                set(g, 2, du);
                set(g, 4, dv);
                wrap(angle - theta) * scale
            }
            Eq::CircleTangent { s0, s1, gap } => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                g[4] = -s0;
                g[5] = -s1;
                n - s0 * v(4) - s1 * v(5) - gap
            }
            Eq::CircleNormal => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                let (r0, r1) = (v(4), v(5));
                let h = r0.hypot(r1).max(TINY);
                g[4] = -r0 / h;
                g[5] = -r1 / h;
                n - h
            }
            Eq::MidpointLine { value, unsigned } => {
                let (a, b) = (p(4), p(6));
                let m = [(v(0) + v(2)) / 2.0, (v(1) + v(3)) / 2.0];
                let mut h = [0.0; MAX_VARS];
                let s = point_line(m, a, b, &mut h, 0, 2, 4, 1.0);
                let sign = if unsigned && s < 0.0 { -1.0 } else { 1.0 };
                g[0] = sign * h[0] / 2.0;
                g[1] = sign * h[1] / 2.0;
                g[2] = sign * h[0] / 2.0;
                g[3] = sign * h[1] / 2.0;
                g[4] = sign * h[2];
                g[5] = sign * h[3];
                g[6] = sign * h[4];
                g[7] = sign * h[5];
                sign * s - value
            }
            Eq::PerpendicularToLine => {
                let w = diff(p(0), p(2));
                let u = diff(p(4), p(6));
                let nu = norm2(u);
                if nu < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let l = nu.sqrt();
                let f = (w[0] * u[0] + w[1] * u[1]) / l;
                set(g, 0, [-u[0] / l, -u[1] / l]);
                set(g, 2, [u[0] / l, u[1] / l]);
                let db = [w[0] / l - f * u[0] / nu, w[1] / l - f * u[1] / nu];
                set(g, 4, [-db[0], -db[1]]);
                set(g, 6, db);
                f
            }
            Eq::PointEllipse { scale } => {
                let (q, c, a, b) = (V2::vars(v(0), v(1), 0), V2::vars(v(2), v(3), 2), V2::vars(v(4), v(5), 4), D::var(v(6), 6));
                let (w, d) = (a.minus(c), q.minus(c));
                if norm2([w.x.v, w.y.v]) < TINY * TINY || b.v.abs() < TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let s = w.dot(w);
                let (along, across) = (w.dot(d), w.cross(d));
                let level = (along * along / (s * s) + across * across / (s * b * b)).sqrt();
                if level.v < 1e-12 {
                    // At the center, the direction to the curve is undefined. Head along the major axis.
                    let n = s.v.sqrt();
                    *g = [0.0; MAX_VARS];
                    g[0] = w.x.v / s.v * scale;
                    g[1] = w.y.v / s.v * scale;
                    g[2] = -g[0];
                    g[3] = -g[1];
                    return (level.v - 1.0) * scale + 0.0 * n;
                }
                finish(g, (level - one()) * scale)
            }
            Eq::LineTangentEllipse => {
                let (la, lb, c, a, b) = (V2::vars(v(0), v(1), 0), V2::vars(v(2), v(3), 2), V2::vars(v(4), v(5), 4), V2::vars(v(6), v(7), 6), D::var(v(8), 8));
                let u = lb.minus(la);
                let w = a.minus(c);
                if norm2([u.x.v, u.y.v]) < TINY * TINY || norm2([w.x.v, w.y.v]) < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                let n = u.perp().unit();
                let s = n.dot(c.minus(la));
                let (n1, n2) = (n.dot(w), n.dot(w.perp()));
                let reach = (n1 * n1 + b * b * n2 * n2 / w.dot(w)).sqrt();
                let side = if s.v < 0.0 { -1.0 } else { 1.0 };
                finish(g, s * side - reach)
            }
            Eq::LineTangentEllipseAt { scale } => {
                let n = ellipse_normal(p(0), p(2), p(4), v(6), 0, 2, 4, 6);
                let (la, lb) = (V2::vars(v(7), v(8), 7), V2::vars(v(9), v(10), 9));
                let u = lb.minus(la);
                if norm2([u.x.v, u.y.v]) < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                finish(g, u.unit().dot(n) * scale)
            }
            Eq::CircleTangentEllipseAt { scale } => {
                let n = ellipse_normal(p(0), p(2), p(4), v(6), 0, 2, 4, 6);
                let (at, center) = (V2::vars(v(0), v(1), 0), V2::vars(v(7), v(8), 7));
                let r = at.minus(center);
                if norm2([r.x.v, r.y.v]) < TINY * TINY {
                    *g = [0.0; MAX_VARS];
                    return 0.0;
                }
                finish(g, r.unit().cross(n) * scale)
            }
            Eq::EllipsesTangentAt { scale } => {
                let n0 = ellipse_normal(p(0), p(2), p(4), v(6), 0, 2, 4, 6);
                // The second ellipse's variables, with the shared point at the front of the same list.
                let n1 = ellipse_normal(p(0), p(7), p(9), v(11), 0, 7, 9, 11);
                finish(g, n0.cross(n1) * scale)
            }
        }
    }
}

fn one() -> D {
    D { v: 1.0, g: [0.0; MAX_VARS] }
}

fn finish(g: &mut [f64; MAX_VARS], value: D) -> f64 {
    *g = value.g;
    value.v
}

/// The unit normal of an ellipse at a point on it, for a center, major end and minor radius given with the
/// slots of their variables.
#[allow(clippy::too_many_arguments)]
fn ellipse_normal(at: P2, center: P2, major: P2, minor: f64, s_at: usize, s_center: usize, s_major: usize, s_minor: usize) -> V2 {
    let (q, c, a, b) = (V2::vars(at[0], at[1], s_at), V2::vars(center[0], center[1], s_center), V2::vars(major[0], major[1], s_major), D::var(minor, s_minor));
    let (w, d) = (a.minus(c), q.minus(c));
    let s = w.dot(w);
    // The gradient of (along / a^2)^2 + (across / (a b))^2, with denominators cleared.
    let along = w.dot(d);
    let across = w.cross(d);
    let normal = w.scaled(along * b * b).plus(w.perp().scaled(across * s));
    if norm2([normal.x.v, normal.y.v]) < 1e-300 {
        return normal;
    }
    normal.unit()
}

fn diff(a: P2, b: P2) -> P2 {
    [b[0] - a[0], b[1] - a[1]]
}

fn norm2(a: P2) -> f64 {
    a[0] * a[0] + a[1] * a[1]
}

fn set(g: &mut [f64; MAX_VARS], at: usize, v: P2) {
    g[at] = v[0];
    g[at + 1] = v[1];
}

/// Gradient for an expression of the two segments' directions, from the derivative with
/// respect to each direction.
fn segments(g: &mut [f64; MAX_VARS], du: P2, dv: P2) {
    set(g, 0, [-du[0], -du[1]]);
    set(g, 2, du);
    set(g, 4, [-dv[0], -dv[1]]);
    set(g, 6, dv);
}

fn length_and_unit(a: P2, b: P2) -> (f64, P2) {
    let d = diff(a, b);
    let n = norm2(d).sqrt();
    if n < TINY {
        // Two points in one place have no direction, so pick one that lets them separate.
        (n, [1.0, 0.0])
    } else {
        (n, [d[0] / n, d[1] / n])
    }
}

/// The signed distance from p to line ab, positive on the left, with its gradient.
#[allow(clippy::too_many_arguments)]
fn point_line(p: P2, a: P2, b: P2, g: &mut [f64; MAX_VARS], gp: usize, ga: usize, gb: usize, k: f64) -> f64 {
    let u = diff(a, b);
    let w = diff(a, p);
    let nu = norm2(u);
    if nu < TINY * TINY {
        // A line of no length has no direction. Use the distance to the point it collapsed to,
        // which gives the solver a way out.
        let (n, unit) = length_and_unit(a, p);
        set(g, gp, [unit[0] * k, unit[1] * k]);
        set(g, ga, [-unit[0] * k, -unit[1] * k]);
        set(g, gb, [0.0, 0.0]);
        return n * k;
    }
    let l = nu.sqrt();
    let s = (u[0] * w[1] - u[1] * w[0]) / l;
    let dp = [-u[1] / l * k, u[0] / l * k];
    let db = [(w[1] / l - s * u[0] / nu) * k, (-w[0] / l - s * u[1] / nu) * k];
    set(g, gp, dp);
    set(g, gb, db);
    set(g, ga, [-dp[0] - db[0], -dp[1] - db[1]]);
    s * k
}

/// A connected part of the system, which can be solved on its own.
#[derive(Clone, Debug)]
pub(crate) struct Component {
    pub vars: Vec<u32>,
    pub rows: Vec<usize>,
}

/// A sketch as variables and equations.
pub(crate) struct System {
    pub x0: Vec<f64>,
    /// Reference geometry, which never moves.
    pub frozen: Vec<bool>,
    pub rows: Vec<Row>,
    /// First variable of each entity.
    pub base: HashMap<EntityId, u32>,
    /// Variables that must stay positive.
    pub radii: Vec<u32>,
    /// Both ends of each line and arc, which must not collapse together.
    pub spans: Vec<[u32; 4]>,
    pub size: f64,
    pub constraint_ids: Vec<ConstraintId>,
}

fn invalid(c: Option<&Constraint>, message: String) -> SketchError {
    let (text, constraints) = match c {
        Some(c) => (format!("c{} ({}): {message}", c.id.0, c.rule.kind()), vec![c.id]),
        None => (message, Vec::new()),
    };
    SketchError { kind: ErrorKind::Invalid, message: text, constraints, entities: Vec::new() }
}

struct Builder<'a> {
    sketch: &'a Sketch,
    env: &'a Env,
    sys: System,
    /// Union-find over x variables of points, from coincident constraints.
    same: HashMap<u32, u32>,
    /// Pairs of (point's x variable, curve) from point-on-curve constraints.
    on_curve: Vec<(u32, EntityId)>,
}

impl System {
    pub fn build(sketch: &Sketch, env: &Env) -> Result<System, SketchError> {
        let mut sys = System {
            x0: Vec::new(),
            frozen: Vec::new(),
            rows: Vec::new(),
            base: HashMap::new(),
            radii: Vec::new(),
            spans: Vec::new(),
            size: sketch.size(),
            constraint_ids: sketch.constraints.iter().map(|c| c.id).collect(),
        };
        let mut seen = std::collections::HashSet::new();
        for e in &sketch.entities {
            if !seen.insert(e.id) {
                return Err(invalid(None, format!("entity id {} appears twice", e.id.0)));
            }
            let base = sys.x0.len() as u32;
            sys.base.insert(e.id, base);
            match e.geometry {
                Geometry::Spline(ref shape) => {
                    let last = base + 2 * (shape.points.len() as u32 - 1);
                    sys.spans.push([base, base + 1, last, last + 1]);
                }
                Geometry::Line { .. } => sys.spans.push([base, base + 1, base + 2, base + 3]),
                Geometry::Arc { .. } => sys.spans.push([base + 3, base + 4, base + 5, base + 6]),
                Geometry::Ellipse { .. } => sys.spans.push([base, base + 1, base + 2, base + 3]),
                Geometry::EllipticalArc { .. } => {
                    sys.spans.push([base, base + 1, base + 2, base + 3]);
                    sys.spans.push([base + 5, base + 6, base + 7, base + 8]);
                }
                _ => {}
            }
            let values: Vec<f64> = match e.geometry {
                Geometry::Spline(ref shape) => {
                    shape.check().map_err(|why| invalid(None, format!("entity {}: {why}", e.id.0)))?;
                    shape.points.iter().flatten().copied().collect()
                }
                Geometry::Point { at } => at.to_vec(),
                Geometry::Line { start, end } => vec![start[0], start[1], end[0], end[1]],
                Geometry::Circle { center, radius } => {
                    if !(radius > 0.0) {
                        return Err(invalid(None, format!("circle {} needs a positive radius, not {radius}", e.id.0)));
                    }
                    sys.radii.push(base + 2);
                    vec![center[0], center[1], radius]
                }
                Geometry::Arc { center, radius, start, end } => {
                    if !(radius > 0.0) {
                        return Err(invalid(None, format!("arc {} needs a positive radius, not {radius}", e.id.0)));
                    }
                    sys.radii.push(base + 2);
                    vec![center[0], center[1], radius, start[0], start[1], end[0], end[1]]
                }
                Geometry::Ellipse { center, major, minor } => {
                    if !(minor > 0.0) {
                        return Err(invalid(None, format!("ellipse {} needs a positive minor radius, not {minor}", e.id.0)));
                    }
                    sys.radii.push(base + 4);
                    vec![center[0], center[1], major[0], major[1], minor]
                }
                Geometry::EllipticalArc { center, major, minor, start, end } => {
                    if !(minor > 0.0) {
                        return Err(invalid(None, format!("elliptical arc {} needs a positive minor radius, not {minor}", e.id.0)));
                    }
                    sys.radii.push(base + 4);
                    vec![center[0], center[1], major[0], major[1], minor, start[0], start[1], end[0], end[1]]
                }
            };
            if values.iter().any(|v| !v.is_finite() || v.abs() > 1e12) {
                return Err(invalid(None, format!("entity {} has a coordinate that isn't a sensible number", e.id.0)));
            }
            sys.x0.extend_from_slice(&values);
            sys.frozen.resize(sys.x0.len(), false);
        }
        // Reference geometry, after everything else.
        for (id, values) in [
            (ORIGIN, vec![0.0, 0.0]),
            (X_AXIS, vec![0.0, 0.0, 1.0, 0.0]),
            (Y_AXIS, vec![0.0, 0.0, 0.0, 1.0]),
        ] {
            sys.base.insert(id, sys.x0.len() as u32);
            sys.x0.extend_from_slice(&values);
            sys.frozen.resize(sys.x0.len(), true);
        }
        // An arc's own geometry: both ends on its circle. These come first, so that a constraint
        // is what gets called redundant, never the arc itself.
        for e in &sketch.entities {
            if let Geometry::Arc { .. } = e.geometry {
                let b = sys.base[&e.id];
                for point in [b + 3, b + 5] {
                    sys.rows.push(row(None, Eq::DistToRadius { gap: 0.0 }, &[b, b + 1, point, point + 1, b + 2]));
                }
            }
            if let Geometry::EllipticalArc { center, major, minor, .. } = e.geometry {
                let b = sys.base[&e.id];
                let scale = minor.min((major[0] - center[0]).hypot(major[1] - center[1])).max(1e-3);
                for point in [b + 5, b + 7] {
                    sys.rows.push(row(None, Eq::PointEllipse { scale }, &[point, point + 1, b, b + 1, b + 2, b + 3, b + 4]));
                }
            }
        }
        let mut builder = Builder { sketch, env, sys, same: HashMap::new(), on_curve: Vec::new() };
        for c in &sketch.constraints {
            builder.note_contacts(c)?;
        }
        for (i, c) in sketch.constraints.iter().enumerate() {
            // A reference dimension reports. It takes no part in solving.
            if !c.reference {
                builder.add(i, c).map_err(|m| invalid(Some(c), m))?;
            }
        }
        Ok(builder.sys)
    }

    /// Splits into independent parts. Rows with no moving variable are left out.
    pub fn components(&self) -> Vec<Component> {
        let n = self.x0.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(parent: &mut [u32], mut v: u32) -> u32 {
            while parent[v as usize] != v {
                parent[v as usize] = parent[parent[v as usize] as usize];
                v = parent[v as usize];
            }
            v
        }
        for r in &self.rows {
            let mut first = None;
            for &v in &r.vars[..r.count as usize] {
                if self.frozen[v as usize] {
                    continue;
                }
                match first {
                    None => first = Some(v),
                    Some(f) => {
                        let (a, b) = (find(&mut parent, f), find(&mut parent, v));
                        parent[a as usize] = b;
                    }
                }
            }
        }
        // Both ends of a line belong together even if no equation links x to y: a horizontal and a
        // vertical on one line are separate equations, and only together do they collapse it.
        for span in &self.spans {
            for &v in &span[1..] {
                let (a, b) = (find(&mut parent, span[0]), find(&mut parent, v));
                parent[a as usize] = b;
            }
        }
        let mut index: HashMap<u32, usize> = HashMap::new();
        let mut out: Vec<Component> = Vec::new();
        for (i, r) in self.rows.iter().enumerate() {
            let Some(&v) = r.vars[..r.count as usize].iter().find(|&&v| !self.frozen[v as usize]) else {
                continue;
            };
            let root = find(&mut parent, v);
            let at = *index.entry(root).or_insert_with(|| {
                out.push(Component { vars: Vec::new(), rows: Vec::new() });
                out.len() - 1
            });
            out[at].rows.push(i);
        }
        let mut touched = vec![false; n];
        for c in &mut out {
            for &i in &c.rows {
                let r = &self.rows[i];
                for &v in &r.vars[..r.count as usize] {
                    if !self.frozen[v as usize] && !touched[v as usize] {
                        touched[v as usize] = true;
                        c.vars.push(v);
                    }
                }
            }
            c.vars.sort_unstable();
        }
        out
    }

    /// Rows that touch only reference geometry, such as a constraint between two axes.
    pub fn constant_rows(&self) -> Vec<usize> {
        (0..self.rows.len())
            .filter(|&i| {
                let r = &self.rows[i];
                r.vars[..r.count as usize].iter().all(|&v| self.frozen[v as usize])
            })
            .collect()
    }

    /// The part of an entity that a variable belongs to.
    pub fn locate(&self, sketch: &Sketch) -> Vec<Option<Ref>> {
        let mut out = vec![None; self.x0.len()];
        for e in &sketch.entities {
            let b = self.base[&e.id] as usize;
            if let Geometry::Spline(ref shape) = e.geometry {
                for i in 0..shape.points.len() {
                    let part = Ref { entity: e.id, part: Part::Control(i as u32) };
                    out[b + 2 * i] = Some(part);
                    out[b + 2 * i + 1] = Some(part);
                }
                continue;
            }
            let parts: &[(usize, Part)] = match e.geometry {
                Geometry::Spline(_) => &[],
                Geometry::Point { .. } => &[(0, Part::Whole), (1, Part::Whole)],
                Geometry::Line { .. } => &[(0, Part::Start), (1, Part::Start), (2, Part::End), (3, Part::End)],
                Geometry::Circle { .. } => &[(0, Part::Center), (1, Part::Center), (2, Part::Radius)],
                Geometry::Arc { .. } => &[
                    (0, Part::Center),
                    (1, Part::Center),
                    (2, Part::Radius),
                    (3, Part::Start),
                    (4, Part::Start),
                    (5, Part::End),
                    (6, Part::End),
                ],
                Geometry::Ellipse { .. } => &[(0, Part::Center), (1, Part::Center), (2, Part::Major), (3, Part::Major), (4, Part::Radius)],
                Geometry::EllipticalArc { .. } => &[
                    (0, Part::Center),
                    (1, Part::Center),
                    (2, Part::Major),
                    (3, Part::Major),
                    (4, Part::Radius),
                    (5, Part::Start),
                    (6, Part::Start),
                    (7, Part::End),
                    (8, Part::End),
                ],
            };
            for &(offset, part) in parts {
                out[b + offset] = Some(Ref { entity: e.id, part });
            }
        }
        out
    }

    /// Writes solved values into a copy of the sketch.
    pub fn write_back(&self, sketch: &Sketch, x: &[f64]) -> Sketch {
        let mut out = sketch.clone();
        for e in &mut out.entities {
            let b = self.base[&e.id] as usize;
            let p = |i: usize| [x[b + i], x[b + i + 1]];
            e.geometry = match e.geometry {
                Geometry::Spline(ref shape) => {
                    let mut moved = (**shape).clone();
                    for (i, q) in moved.points.iter_mut().enumerate() {
                        *q = p(2 * i);
                    }
                    Geometry::Spline(std::sync::Arc::new(moved))
                }
                Geometry::Point { .. } => Geometry::Point { at: p(0) },
                Geometry::Line { .. } => Geometry::Line { start: p(0), end: p(2) },
                Geometry::Circle { .. } => Geometry::Circle { center: p(0), radius: x[b + 2] },
                Geometry::Arc { .. } => Geometry::Arc { center: p(0), radius: x[b + 2], start: p(3), end: p(5) },
                Geometry::Ellipse { .. } => Geometry::Ellipse { center: p(0), major: p(2), minor: x[b + 4] },
                Geometry::EllipticalArc { .. } => Geometry::EllipticalArc { center: p(0), major: p(2), minor: x[b + 4], start: p(5), end: p(7) },
            };
        }
        out.refresh_references();
        out
    }
}

fn row(owner: Option<usize>, eq: Eq, vars: &[u32]) -> Row {
    let mut all = [0u32; MAX_VARS];
    all[..vars.len()].copy_from_slice(vars);
    Row { owner, eq, vars: all, count: vars.len() as u8 }
}

fn linear(owner: usize, value: f64, terms: &[(u32, f64)]) -> Row {
    let mut coef = [0.0; 4];
    let mut vars = Vec::with_capacity(terms.len());
    for (i, &(v, c)) in terms.iter().enumerate() {
        coef[i] = c;
        vars.push(v);
    }
    row(Some(owner), Eq::Linear { coef, value }, &vars)
}

impl Builder<'_> {
    fn geometry(&self, id: EntityId) -> Result<Option<Geometry>, String> {
        if id.is_reference() {
            return Ok(None);
        }
        match self.sketch.entity(id) {
            Some(e) => Ok(Some(e.geometry.clone())),
            None => Err(format!("there is no entity {}", id.0)),
        }
    }

    /// The variables of a point.
    fn point(&self, r: Ref) -> Result<[u32; 2], String> {
        let bad = || format!("{} is not a point", r.name());
        let b = *self.sys.base.get(&r.entity).ok_or_else(|| format!("there is no entity {}", r.name()))?;
        let offset = match (r.entity, r.part) {
            (ORIGIN, Part::Whole) => 0,
            (X_AXIS | Y_AXIS, Part::Start) => 0,
            (X_AXIS | Y_AXIS, Part::End) => 2,
            (ORIGIN | X_AXIS | Y_AXIS, _) => return Err(bad()),
            _ => match (self.geometry(r.entity)?.ok_or_else(bad)?, r.part) {
                (Geometry::Spline(_), Part::Start) => 0,
                (Geometry::Spline(shape), Part::End) => 2 * (shape.points.len() as u32 - 1),
                (Geometry::Spline(shape), Part::Control(i)) if (i as usize) < shape.points.len() => 2 * i,
                (Geometry::Point { .. }, Part::Whole) => 0,
                (Geometry::Line { .. }, Part::Start) => 0,
                (Geometry::Line { .. }, Part::End) => 2,
                (Geometry::Circle { .. }, Part::Center) => 0,
                (Geometry::Arc { .. }, Part::Center) => 0,
                (Geometry::Arc { .. }, Part::Start) => 3,
                (Geometry::Arc { .. }, Part::End) => 5,
                (Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. }, Part::Center) => 0,
                (Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. }, Part::Major) => 2,
                (Geometry::EllipticalArc { .. }, Part::Start) => 5,
                (Geometry::EllipticalArc { .. }, Part::End) => 7,
                _ => return Err(bad()),
            },
        };
        Ok([b + offset, b + offset + 1])
    }

    fn is_point(&self, r: Ref) -> bool {
        self.point(r).is_ok()
    }

    fn is_line(&self, id: EntityId) -> bool {
        id == X_AXIS || id == Y_AXIS || matches!(self.geometry(id), Ok(Some(Geometry::Line { .. })))
    }

    fn is_curve(&self, id: EntityId) -> bool {
        matches!(self.geometry(id), Ok(Some(Geometry::Circle { .. } | Geometry::Arc { .. })))
    }

    fn is_spline(&self, id: EntityId) -> bool {
        matches!(self.geometry(id), Ok(Some(Geometry::Spline(_))))
    }

    /// The line a spline leaves or arrives along at this end: its first or last control leg, in which
    /// direction doesn't matter. A clamped spline is tangent to that leg at the end, whatever its weights.
    fn end_leg(&self, id: EntityId, at: [u32; 2]) -> Result<[u32; 4], String> {
        let Ok(Some(Geometry::Spline(shape))) = self.geometry(id) else {
            return Err(format!("{} is not a spline", id.whole().name()));
        };
        let b = self.sys.base[&id];
        let last = shape.points.len() as u32 - 1;
        let (start, end) = ([b, b + 1], [b + 2 * last, b + 2 * last + 1]);
        if self.root(at[0]) == self.root(start[0]) {
            Ok([start[0], start[1], b + 2, b + 3])
        } else if self.root(at[0]) == self.root(end[0]) {
            Ok([end[0] - 2, end[1] - 2, end[0], end[1]])
        } else {
            Err(format!("a tangency with spline {} has to be at one of its ends for now", id.0))
        }
    }

    fn is_ellipse(&self, id: EntityId) -> bool {
        matches!(self.geometry(id), Ok(Some(Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. })))
    }

    /// An ellipse's center, major end and minor radius variables.
    fn ellipse(&self, id: EntityId) -> Result<([u32; 2], [u32; 2], u32), String> {
        if !self.is_ellipse(id) {
            return Err(format!("{} is not an ellipse or elliptical arc", id.whole().name()));
        }
        let b = self.sys.base[&id];
        Ok(([b, b + 1], [b + 2, b + 3], b + 4))
    }

    /// An error scale for an ellipse: its smaller radius, so that its equations read in millimetres.
    fn ellipse_scale(&self, e: ([u32; 2], [u32; 2], u32)) -> f64 {
        self.at(e.2).min(self.length(e.0, e.1)).max(1e-3)
    }

    /// The center of a circle, arc or ellipse.
    fn center_of(&self, id: EntityId) -> Result<[u32; 2], String> {
        if self.is_ellipse(id) {
            Ok(self.ellipse(id)?.0)
        } else {
            Ok(self.curve(id)?.0)
        }
    }

    /// A line's variables: both ends.
    fn line(&self, id: EntityId) -> Result<[u32; 4], String> {
        if !self.is_line(id) {
            return Err(format!("{} is not a line", id.whole().name()));
        }
        let b = self.sys.base[&id];
        Ok([b, b + 1, b + 2, b + 3])
    }

    /// A circle's or arc's center and radius variables.
    fn curve(&self, id: EntityId) -> Result<([u32; 2], u32), String> {
        if !self.is_curve(id) {
            return Err(format!("{} is not a circle or arc", id.whole().name()));
        }
        let b = self.sys.base[&id];
        Ok(([b, b + 1], b + 2))
    }

    fn root(&self, mut v: u32) -> u32 {
        while let Some(&p) = self.same.get(&v) {
            if p == v {
                break;
            }
            v = p;
        }
        v
    }

    /// Records which points are joined, so that a tangency can tell if it has a shared point.
    fn note_contacts(&mut self, c: &Constraint) -> Result<(), SketchError> {
        if let Rule::Coincident { a, b } = c.rule {
            let (pa, pb) = (self.point(a), self.point(b));
            match (pa, pb) {
                (Ok(pa), Ok(pb)) => {
                    let (ra, rb) = (self.root(pa[0]), self.root(pb[0]));
                    if ra != rb {
                        self.same.insert(ra, rb);
                    }
                }
                (Ok(pa), Err(_)) if self.is_curve(b.entity) || self.is_ellipse(b.entity) => self.on_curve.push((pa[0], b.entity)),
                (Err(_), Ok(pb)) if self.is_curve(a.entity) || self.is_ellipse(a.entity) => self.on_curve.push((pb[0], a.entity)),
                _ => {}
            }
        }
        Ok(())
    }

    fn endpoints(&self, id: EntityId) -> Vec<[u32; 2]> {
        match self.geometry(id) {
            Ok(Some(Geometry::Line { .. } | Geometry::Arc { .. } | Geometry::EllipticalArc { .. } | Geometry::Spline(_))) => {
                [id.start(), id.end()].iter().filter_map(|&r| self.point(r).ok()).collect()
            }
            _ => Vec::new(),
        }
    }

    /// A point the two entities share through coincident constraints, if any.
    fn shared_point(&self, a: EntityId, b: EntityId) -> Option<[u32; 2]> {
        let (ea, eb) = (self.endpoints(a), self.endpoints(b));
        for p in &ea {
            for q in &eb {
                if self.root(p[0]) == self.root(q[0]) {
                    return Some(*p);
                }
            }
        }
        // An endpoint of one placed on the other.
        for (points, curve) in [(&ea, b), (&eb, a)] {
            for p in points.iter() {
                if self.on_curve.iter().any(|&(v, c)| c == curve && self.root(v) == self.root(p[0])) {
                    return Some(*p);
                }
            }
        }
        None
    }

    fn at(&self, v: u32) -> f64 {
        self.sys.x0[v as usize]
    }

    fn length(&self, p: [u32; 2], q: [u32; 2]) -> f64 {
        (self.at(q[0]) - self.at(p[0])).hypot(self.at(q[1]) - self.at(p[1]))
    }

    fn scale_of(&self, a: f64, b: f64) -> f64 {
        a.max(b).max(1e-3)
    }

    fn value(&self, e: &Expr) -> Result<f64, String> {
        e.eval(self.env)
    }

    fn positive(&self, e: &Expr, what: &str) -> Result<f64, String> {
        let v = self.value(e)?;
        if v > 0.0 {
            Ok(v)
        } else {
            Err(format!("{what} must be positive, not {v}"))
        }
    }

    fn push(&mut self, r: Row) {
        self.sys.rows.push(r);
    }

    fn add(&mut self, i: usize, c: &Constraint) -> Result<(), String> {
        match &c.rule {
            Rule::Coincident { a, b } => {
                let (a, b) = if self.is_point(*a) { (*a, *b) } else { (*b, *a) };
                let p = self.point(a).map_err(|_| "coincident needs at least one point".to_string())?;
                if self.is_point(b) {
                    let q = self.point(b)?;
                    self.push(linear(i, 0.0, &[(p[0], 1.0), (q[0], -1.0)]));
                    self.push(linear(i, 0.0, &[(p[1], 1.0), (q[1], -1.0)]));
                } else if self.is_line(b.entity) && b.part == Part::Whole {
                    let l = self.line(b.entity)?;
                    self.push(row(Some(i), Eq::PointLine { value: 0.0, unsigned: false }, &[p[0], p[1], l[0], l[1], l[2], l[3]]));
                } else if self.is_curve(b.entity) && b.part == Part::Whole {
                    let (center, r) = self.curve(b.entity)?;
                    self.push(row(Some(i), Eq::DistToRadius { gap: 0.0 }, &[center[0], center[1], p[0], p[1], r]));
                } else if self.is_ellipse(b.entity) && b.part == Part::Whole {
                    let e = self.ellipse(b.entity)?;
                    let scale = self.ellipse_scale(e);
                    self.push(row(Some(i), Eq::PointEllipse { scale }, &[p[0], p[1], e.0[0], e.0[1], e.1[0], e.1[1], e.2]));
                } else if self.is_spline(b.entity) && b.part == Part::Whole {
                    return Err(format!("a point on spline {} isn't available yet: join to its ends or control points", b.entity.0));
                } else {
                    return Err(format!("{} is not a point, line, circle, arc or ellipse", b.name()));
                }
            }
            Rule::Horizontal { a, b } | Rule::Vertical { a, b } => {
                let axis = usize::from(matches!(c.rule, Rule::Vertical { .. }) == false);
                let (p, q) = match b {
                    Some(b) => (self.point(*a)?, self.point(*b)?),
                    None => {
                        let l = self.line(a.entity)?;
                        ([l[0], l[1]], [l[2], l[3]])
                    }
                };
                self.push(linear(i, 0.0, &[(p[axis], 1.0), (q[axis], -1.0)]));
            }
            Rule::Parallel { a, b } | Rule::Perpendicular { a, b } => {
                let (la, lb) = (self.line(*a)?, self.line(*b)?);
                let scale = self.scale_of(self.length([la[0], la[1]], [la[2], la[3]]), self.length([lb[0], lb[1]], [lb[2], lb[3]]));
                let eq = if matches!(c.rule, Rule::Parallel { .. }) { Eq::Parallel { scale } } else { Eq::Perpendicular { scale } };
                self.push(row(Some(i), eq, &[la[0], la[1], la[2], la[3], lb[0], lb[1], lb[2], lb[3]]));
            }
            Rule::Tangent { a, b } => self.tangent(i, *a, *b)?,
            Rule::Equal { a, b } => {
                if self.is_line(*a) && self.is_line(*b) {
                    let (la, lb) = (self.line(*a)?, self.line(*b)?);
                    self.push(row(Some(i), Eq::EqualLength, &[la[0], la[1], la[2], la[3], lb[0], lb[1], lb[2], lb[3]]));
                } else if self.is_curve(*a) && self.is_curve(*b) {
                    let (ra, rb) = (self.curve(*a)?.1, self.curve(*b)?.1);
                    self.push(linear(i, 0.0, &[(ra, 1.0), (rb, -1.0)]));
                } else if self.is_ellipse(*a) && self.is_ellipse(*b) {
                    let (ea, eb) = (self.ellipse(*a)?, self.ellipse(*b)?);
                    self.push(row(Some(i), Eq::EqualLength, &[ea.0[0], ea.0[1], ea.1[0], ea.1[1], eb.0[0], eb.0[1], eb.1[0], eb.1[1]]));
                    self.push(linear(i, 0.0, &[(ea.2, 1.0), (eb.2, -1.0)]));
                } else {
                    return Err("equal needs two lines, two circles or arcs, or two ellipses".to_string());
                }
            }
            Rule::Midpoint { point, line } => {
                let (p, l) = (self.point(*point)?, self.line(*line)?);
                self.push(linear(i, 0.0, &[(p[0], 1.0), (l[0], -0.5), (l[2], -0.5)]));
                self.push(linear(i, 0.0, &[(p[1], 1.0), (l[1], -0.5), (l[3], -0.5)]));
            }
            Rule::Concentric { a, b } => {
                let (ca, cb) = (self.center_of(*a)?, self.center_of(*b)?);
                self.push(linear(i, 0.0, &[(ca[0], 1.0), (cb[0], -1.0)]));
                self.push(linear(i, 0.0, &[(ca[1], 1.0), (cb[1], -1.0)]));
            }
            Rule::Symmetric { a, b, about } => {
                let (p, q) = (self.point(*a)?, self.point(*b)?);
                if about.part == Part::Whole && self.is_line(about.entity) {
                    let l = self.line(about.entity)?;
                    self.push(row(Some(i), Eq::MidpointLine { value: 0.0, unsigned: false }, &[p[0], p[1], q[0], q[1], l[0], l[1], l[2], l[3]]));
                    self.push(row(Some(i), Eq::PerpendicularToLine, &[p[0], p[1], q[0], q[1], l[0], l[1], l[2], l[3]]));
                } else {
                    let m = self.point(*about)?;
                    self.push(linear(i, 0.0, &[(p[0], 1.0), (q[0], 1.0), (m[0], -2.0)]));
                    self.push(linear(i, 0.0, &[(p[1], 1.0), (q[1], 1.0), (m[1], -2.0)]));
                }
            }
            Rule::Normal { a, b } => self.normal(i, *a, *b)?,
            Rule::Fix { what, at } => self.fix(i, *what, at)?,
            Rule::Distance { a, b, value } => {
                let v = self.positive(value, "a distance")?;
                let (first, second) = (*a, *b);
                match second {
                    None => {
                        let l = self.line(first.entity)?;
                        self.push(row(Some(i), Eq::Dist { value: v }, &[l[0], l[1], l[2], l[3]]));
                    }
                    Some(s) => self.distance(i, first, s, v)?,
                }
            }
            Rule::HorizontalDistance { a, b, value } | Rule::VerticalDistance { a, b, value } => {
                let axis = usize::from(matches!(c.rule, Rule::VerticalDistance { .. }));
                let v = self.value(value)?;
                let (p, q) = (self.point(*a)?, self.point(*b)?);
                self.push(linear(i, v, &[(q[axis], 1.0), (p[axis], -1.0)]));
            }
            Rule::Radius { of, value } => {
                let v = self.positive(value, "a radius")?;
                let r = self.curve(*of)?.1;
                self.push(linear(i, v, &[(r, 1.0)]));
            }
            Rule::Diameter { of, value } => {
                let v = self.positive(value, "a diameter")?;
                let r = self.curve(*of)?.1;
                self.push(linear(i, v, &[(r, 2.0)]));
            }
            Rule::MajorRadius { of, value } => {
                let v = self.positive(value, "a major radius")?;
                let e = self.ellipse(*of)?;
                self.push(row(Some(i), Eq::Dist { value: v }, &[e.0[0], e.0[1], e.1[0], e.1[1]]));
            }
            Rule::MinorRadius { of, value } => {
                let v = self.positive(value, "a minor radius")?;
                let e = self.ellipse(*of)?;
                self.push(linear(i, v, &[(e.2, 1.0)]));
            }
            Rule::Angle { a, b, value } => {
                let theta = self.value(value)?.to_radians();
                match b {
                    Some(b) => {
                        let (la, lb) = (self.line(*a)?, self.line(*b)?);
                        let scale = self.scale_of(self.length([la[0], la[1]], [la[2], la[3]]), self.length([lb[0], lb[1]], [lb[2], lb[3]]));
                        self.push(row(Some(i), Eq::Angle { theta, scale }, &[la[0], la[1], la[2], la[3], lb[0], lb[1], lb[2], lb[3]]));
                    }
                    None if self.is_line(*a) => {
                        let (axis, l) = (self.line(X_AXIS)?, self.line(*a)?);
                        let scale = self.scale_of(self.length([l[0], l[1]], [l[2], l[3]]), 0.0);
                        self.push(row(Some(i), Eq::Angle { theta, scale }, &[axis[0], axis[1], axis[2], axis[3], l[0], l[1], l[2], l[3]]));
                    }
                    None => {
                        let Some(Geometry::Arc { .. }) = self.geometry(*a)? else {
                            return Err(format!("{} has no angle of its own: use a line or an arc", a.whole().name()));
                        };
                        let bv = self.sys.base[a];
                        let scale = self.at(bv + 2).max(1e-3);
                        self.push(row(Some(i), Eq::ArcSpan { theta, scale }, &[bv, bv + 1, bv + 3, bv + 4, bv + 5, bv + 6]));
                    }
                }
            }
        }
        Ok(())
    }

    /// A distance between any two of a point, a line and a circle or arc. To a circle or arc, it's
    /// the gap to the curve, on whichever side things are now. An arc counts as its full circle, and a
    /// line as infinite, as they do for other constraints.
    fn distance(&mut self, i: usize, a: Ref, b: Ref, v: f64) -> Result<(), String> {
        enum Thing {
            Point([u32; 2]),
            Line([u32; 4]),
            Curve([u32; 2], u32),
        }
        let thing = |this: &Self, r: Ref| -> Result<Thing, String> {
            if let Ok(p) = this.point(r) {
                Ok(Thing::Point(p))
            } else if r.part == Part::Whole && this.is_line(r.entity) {
                Ok(Thing::Line(this.line(r.entity)?))
            } else if r.part == Part::Whole && this.is_curve(r.entity) {
                let (c, radius) = this.curve(r.entity)?;
                Ok(Thing::Curve(c, radius))
            } else if r.part == Part::Whole && this.is_spline(r.entity) {
                Err(format!("a distance to the spline {} isn't supported yet: use its control points, such as {}", r.name(), r.entity.start().name()))
            } else if r.part == Part::Whole && this.is_ellipse(r.entity) {
                Err(format!("a distance to the ellipse {} isn't supported yet: use its center or {}", r.name(), r.entity.major().name()))
            } else {
                Err(format!("a distance to {} isn't supported: use points, lines, circles or arcs", r.name()))
            }
        };
        let (ta, tb) = (thing(self, a)?, thing(self, b)?);
        match (ta, tb) {
            (Thing::Point(p), Thing::Point(q)) => {
                self.push(row(Some(i), Eq::Dist { value: v }, &[p[0], p[1], q[0], q[1]]));
            }
            (Thing::Point(p), Thing::Line(l)) | (Thing::Line(l), Thing::Point(p)) => {
                self.push(row(Some(i), Eq::PointLine { value: v, unsigned: true }, &[p[0], p[1], l[0], l[1], l[2], l[3]]));
            }
            (Thing::Point(p), Thing::Curve(c, r)) | (Thing::Curve(c, r), Thing::Point(p)) => {
                // Stay on the side it's on, unless the circle is too small for that.
                let inside = self.length(p, c) < self.at(r) && v < self.at(r);
                let gap = if inside { -v } else { v };
                self.push(row(Some(i), Eq::DistToRadius { gap }, &[c[0], c[1], p[0], p[1], r]));
            }
            (Thing::Line(l), Thing::Curve(c, r)) | (Thing::Curve(c, r), Thing::Line(l)) => {
                let (la, lb, center) = ([self.at(l[0]), self.at(l[1])], [self.at(l[2]), self.at(l[3])], [self.at(c[0]), self.at(c[1])]);
                let length = diff(la, lb);
                let from = diff(la, center);
                let across = (length[0] * from[1] - length[1] * from[0]).abs() / norm2(length).sqrt().max(TINY);
                let inside = across < self.at(r) && v < self.at(r);
                let gap = if inside { -v } else { v };
                self.push(row(Some(i), Eq::PointLineRadius { gap }, &[c[0], c[1], l[0], l[1], l[2], l[3], r]));
            }
            (Thing::Curve(c0, r0), Thing::Curve(c1, r1)) => {
                let (d, v0, v1) = (self.length(c0, c1), self.at(r0), self.at(r1));
                let (s0, s1, gap) = if d <= (v0 - v1).abs() && v < v0.max(v1) {
                    // One inside the other.
                    if v0 >= v1 {
                        (1.0, -1.0, -v)
                    } else {
                        (-1.0, 1.0, -v)
                    }
                } else {
                    (1.0, 1.0, v)
                };
                self.push(row(Some(i), Eq::CircleTangent { s0, s1, gap }, &[c0[0], c0[1], c1[0], c1[1], r0, r1]));
            }
            (Thing::Line(la), Thing::Line(lb)) => {
                // From the middle of one line to the other, which is the distance between them when
                // they're parallel. A reference axis has no middle worth measuring from.
                let (from, to) = if a.entity.is_reference() { (lb, la) } else { (la, lb) };
                self.push(row(Some(i), Eq::MidpointLine { value: v, unsigned: true }, &[from[0], from[1], from[2], from[3], to[0], to[1], to[2], to[3]]));
            }
        }
        Ok(())
    }

    /// A tangency with at least one spline, at an end of it, since that's where the direction is known
    /// without a search: along the first or last control leg. The leg then behaves as a line.
    fn tangent_spline(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
        let (spline, other) = if self.is_spline(a) { (a, b) } else { (b, a) };
        let Some(p) = self.shared_point(a, b) else {
            return Err(format!(
                "a tangency with spline {} needs an end of it joined to {}: make them coincident first",
                spline.0,
                other.whole().name()
            ));
        };
        let leg = self.end_leg(spline, p)?;
        if self.is_spline(other) {
            let leg2 = self.end_leg(other, p)?;
            let scale = self.length([leg[0], leg[1]], [leg[2], leg[3]]).max(self.length([leg2[0], leg2[1]], [leg2[2], leg2[3]])).max(1e-3);
            self.push(row(Some(i), Eq::Parallel { scale }, &[leg[0], leg[1], leg[2], leg[3], leg2[0], leg2[1], leg2[2], leg2[3]]));
        } else if self.is_line(other) {
            let l = self.line(other)?;
            let scale = self.length([leg[0], leg[1]], [leg[2], leg[3]]).max(self.length([l[0], l[1]], [l[2], l[3]])).max(1e-3);
            self.push(row(Some(i), Eq::Parallel { scale }, &[leg[0], leg[1], leg[2], leg[3], l[0], l[1], l[2], l[3]]));
        } else if self.is_curve(other) {
            let center = self.curve(other)?.0;
            self.push(row(Some(i), Eq::PerpendicularToLine, &[center[0], center[1], p[0], p[1], leg[0], leg[1], leg[2], leg[3]]));
        } else if self.is_ellipse(other) {
            let e = self.ellipse(other)?;
            let scale = self.ellipse_scale(e);
            self.push(row(Some(i), Eq::LineTangentEllipseAt { scale }, &[p[0], p[1], e.0[0], e.0[1], e.1[0], e.1[1], e.2, leg[0], leg[1], leg[2], leg[3]]));
        } else {
            return Err(format!("{} can't be tangent to a spline", other.whole().name()));
        }
        Ok(())
    }

    /// A tangency with at least one ellipse.
    fn tangent_ellipse(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
        let shared = self.shared_point(a, b);
        let (first, other) = if self.is_ellipse(a) { (a, b) } else { (b, a) };
        let e = self.ellipse(first)?;
        let scale = self.ellipse_scale(e);
        let own = [e.0[0], e.0[1], e.1[0], e.1[1], e.2];
        if self.is_line(other) {
            let l = self.line(other)?;
            match shared {
                Some(p) => self.push(row(Some(i), Eq::LineTangentEllipseAt { scale }, &[p[0], p[1], own[0], own[1], own[2], own[3], own[4], l[0], l[1], l[2], l[3]])),
                None => self.push(row(Some(i), Eq::LineTangentEllipse, &[l[0], l[1], l[2], l[3], own[0], own[1], own[2], own[3], own[4]])),
            }
            return Ok(());
        }
        let Some(p) = shared else {
            return Err(format!(
                "a tangency between ellipse {} and {} needs a point they share: make an end of one coincident with the other",
                first.0,
                other.whole().name()
            ));
        };
        if self.is_curve(other) {
            let c = self.curve(other)?.0;
            self.push(row(Some(i), Eq::CircleTangentEllipseAt { scale }, &[p[0], p[1], own[0], own[1], own[2], own[3], own[4], c[0], c[1]]));
        } else {
            let f = self.ellipse(other)?;
            self.push(row(Some(i), Eq::EllipsesTangentAt { scale }, &[p[0], p[1], own[0], own[1], own[2], own[3], own[4], f.0[0], f.0[1], f.1[0], f.1[1], f.2]));
        }
        Ok(())
    }

    fn tangent(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
        if self.is_spline(a) || self.is_spline(b) {
            return self.tangent_spline(i, a, b);
        }
        if self.is_ellipse(a) || self.is_ellipse(b) {
            return self.tangent_ellipse(i, a, b);
        }
        let (al, bl) = (self.is_line(a), self.is_line(b));
        if al && bl {
            return Err("two lines can't be tangent: use parallel".to_string());
        }
        if al || bl {
            let (line, curve) = if al { (a, b) } else { (b, a) };
            let l = self.line(line)?;
            let (center, r) = self.curve(curve)?;
            if let Some(p) = self.shared_point(line, curve) {
                // Tangent at a shared point: the radius there is square to the line. Using the
                // line's distance from the center instead would make this singular at the
                // solution, because a point on both already fixes that distance to first order.
                self.push(row(Some(i), Eq::PerpendicularToLine, &[center[0], center[1], p[0], p[1], l[0], l[1], l[2], l[3]]));
            } else {
                self.push(row(Some(i), Eq::PointLineRadius { gap: 0.0 }, &[center[0], center[1], l[0], l[1], l[2], l[3], r]));
            }
            return Ok(());
        }
        let ((c0, r0), (c1, r1)) = (self.curve(a)?, self.curve(b)?);
        if let Some(p) = self.shared_point(a, b) {
            let scale = self.scale_of(self.at(r0), self.at(r1));
            self.push(row(Some(i), Eq::Parallel { scale }, &[c0[0], c0[1], p[0], p[1], c1[0], c1[1], p[0], p[1]]));
        } else {
            // Outside or inside each other, whichever the geometry is nearer to now.
            let (d, v0, v1) = (self.length(c0, c1), self.at(r0), self.at(r1));
            let (s0, s1) = if (d - (v0 + v1)).abs() <= (d - (v0 - v1).abs()).abs() {
                (1.0, 1.0)
            } else if v0 >= v1 {
                (1.0, -1.0)
            } else {
                (-1.0, 1.0)
            };
            self.push(row(Some(i), Eq::CircleTangent { s0, s1, gap: 0.0 }, &[c0[0], c0[1], c1[0], c1[1], r0, r1]));
        }
        Ok(())
    }

    fn normal(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
        if self.is_spline(a) || self.is_spline(b) {
            return Err("normal isn't available for a spline yet".to_string());
        }
        if self.is_ellipse(a) || self.is_ellipse(b) {
            return Err("normal isn't available for an ellipse yet: use tangent, or constrain its center and major axis".to_string());
        }
        let (al, bl) = (self.is_line(a), self.is_line(b));
        if al && bl {
            return Err("normal is for a line and a curve, or two curves: for two lines, use perpendicular".to_string());
        }
        if al || bl {
            let (line, curve) = if al { (a, b) } else { (b, a) };
            let l = self.line(line)?;
            let center = self.curve(curve)?.0;
            self.push(row(Some(i), Eq::PointLine { value: 0.0, unsigned: false }, &[center[0], center[1], l[0], l[1], l[2], l[3]]));
            return Ok(());
        }
        let ((c0, r0), (c1, r1)) = (self.curve(a)?, self.curve(b)?);
        if let Some(p) = self.shared_point(a, b) {
            let scale = self.scale_of(self.at(r0), self.at(r1));
            self.push(row(Some(i), Eq::Perpendicular { scale }, &[c0[0], c0[1], p[0], p[1], c1[0], c1[1], p[0], p[1]]));
        } else {
            self.push(row(Some(i), Eq::CircleNormal, &[c0[0], c0[1], c1[0], c1[1], r0, r1]));
        }
        Ok(())
    }

    fn fix(&mut self, i: usize, what: Ref, at: &[f64]) -> Result<(), String> {
        let need = |n: usize| {
            if at.len() == n {
                Ok(())
            } else {
                Err(format!("fixing {} takes {n} values, not {}", what.name(), at.len()))
            }
        };
        if at.iter().any(|v| !v.is_finite()) {
            return Err("a fixed value isn't a finite number".to_string());
        }
        if let Ok(p) = self.point(what) {
            need(2)?;
            self.push(linear(i, at[0], &[(p[0], 1.0)]));
            self.push(linear(i, at[1], &[(p[1], 1.0)]));
            return Ok(());
        }
        match self.geometry(what.entity)? {
            Some(Geometry::Line { .. }) => {
                need(4)?;
                let l = self.line(what.entity)?;
                for k in 0..4 {
                    self.push(linear(i, at[k], &[(l[k], 1.0)]));
                }
            }
            Some(Geometry::Circle { .. }) => {
                need(3)?;
                let (c, r) = self.curve(what.entity)?;
                self.push(linear(i, at[0], &[(c[0], 1.0)]));
                self.push(linear(i, at[1], &[(c[1], 1.0)]));
                self.push(linear(i, at[2], &[(r, 1.0)]));
            }
            Some(Geometry::Arc { .. }) => {
                need(7)?;
                let (c, r) = self.curve(what.entity)?;
                self.push(linear(i, at[0], &[(c[0], 1.0)]));
                self.push(linear(i, at[1], &[(c[1], 1.0)]));
                self.push(linear(i, at[2], &[(r, 1.0)]));
                // Each end has one freedom left on its circle: where it sits along it.
                let b = self.sys.base[&what.entity];
                for (point, offset) in [(b + 3, 3), (b + 5, 5)] {
                    let (dx, dy) = (at[offset] - at[0], at[offset + 1] - at[1]);
                    let n = dx.hypot(dy).max(1e-12);
                    let (ux, uy) = (dx / n, dy / n);
                    // Stays on the ray from the center through the fixed point.
                    self.push(linear(i, 0.0, &[(point, uy), (point + 1, -ux), (c[0], -uy), (c[1], ux)]));
                }
            }
            Some(Geometry::Ellipse { .. }) => {
                need(5)?;
                let e = self.ellipse(what.entity)?;
                for (k, v) in [e.0[0], e.0[1], e.1[0], e.1[1], e.2].into_iter().enumerate() {
                    self.push(linear(i, at[k], &[(v, 1.0)]));
                }
            }
            Some(Geometry::EllipticalArc { .. }) => {
                need(9)?;
                let e = self.ellipse(what.entity)?;
                for (k, v) in [e.0[0], e.0[1], e.1[0], e.1[1], e.2].into_iter().enumerate() {
                    self.push(linear(i, at[k], &[(v, 1.0)]));
                }
                // As on an arc: each end stays on the ray from the center through its fixed point.
                let b = self.sys.base[&what.entity];
                for (point, offset) in [(b + 5, 5), (b + 7, 7)] {
                    let (dx, dy) = (at[offset] - at[0], at[offset + 1] - at[1]);
                    let n = dx.hypot(dy).max(1e-12);
                    let (ux, uy) = (dx / n, dy / n);
                    self.push(linear(i, 0.0, &[(point, uy), (point + 1, -ux), (e.0[0], -uy), (e.0[1], ux)]));
                }
            }
            Some(Geometry::Spline(ref shape)) => {
                need(2 * shape.points.len())?;
                let b = self.sys.base[&what.entity];
                for (k, v) in at.iter().enumerate() {
                    self.push(linear(i, *v, &[(b + k as u32, 1.0)]));
                }
            }
            _ => return Err(format!("{} can't be fixed", what.name())),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    /// Every equation's gradient must match its value. This is where a sign slip would show.
    #[test]
    fn gradients_match_central_differences() {
        let mut rng = Rng::new(21);
        let cases: Vec<(Eq, usize)> = vec![
            (Eq::Linear { coef: [1.5, -2.0, 0.5, 3.0], value: 7.0 }, 4),
            (Eq::Dist { value: 3.0 }, 4),
            (Eq::DistToRadius { gap: 0.0 }, 5),
            (Eq::DistToRadius { gap: 1.5 }, 5),
            (Eq::EqualLength, 8),
            (Eq::PointLine { value: 2.0, unsigned: false }, 6),
            (Eq::PointLine { value: 2.0, unsigned: true }, 6),
            (Eq::PointLineRadius { gap: 0.0 }, 7),
            (Eq::PointLineRadius { gap: -1.5 }, 7),
            (Eq::Parallel { scale: 12.0 }, 8),
            (Eq::Perpendicular { scale: 12.0 }, 8),
            (Eq::Angle { theta: 0.7, scale: 9.0 }, 8),
            (Eq::ArcSpan { theta: 1.3, scale: 5.0 }, 6),
            (Eq::CircleTangent { s0: 1.0, s1: -1.0, gap: 0.0 }, 6),
            (Eq::CircleTangent { s0: 1.0, s1: 1.0, gap: 2.0 }, 6),
            (Eq::CircleNormal, 6),
            (Eq::MidpointLine { value: 0.0, unsigned: false }, 8),
            (Eq::MidpointLine { value: 3.0, unsigned: true }, 8),
            (Eq::PerpendicularToLine, 8),
            (Eq::PointEllipse { scale: 4.0 }, 7),
            (Eq::LineTangentEllipse, 9),
            (Eq::LineTangentEllipseAt { scale: 4.0 }, 11),
            (Eq::CircleTangentEllipseAt { scale: 4.0 }, 9),
            (Eq::EllipsesTangentAt { scale: 4.0 }, 12),
        ];
        for (eq, count) in cases {
            for _ in 0..200 {
                let mut x: Vec<f64> = (0..MAX_VARS).map(|_| rng.range(-20.0, 20.0)).collect();
                let vars: Vec<u32> = (0..count as u32).collect();
                let r = row(None, eq, &vars);
                let mut g = [0.0; MAX_VARS];
                let value = r.eval(&x, &mut g);
                let mut scratch = [0.0; MAX_VARS];
                for i in 0..count {
                    let h = 1e-6;
                    let keep = x[i];
                    x[i] = keep + h;
                    let up = r.eval(&x, &mut scratch);
                    x[i] = keep - h;
                    let down = r.eval(&x, &mut scratch);
                    x[i] = keep;
                    let mut slope = (up - down) / (2.0 * h);
                    // An angle that wrapped between the two samples has jumped by a full turn.
                    if matches!(eq, Eq::Angle { .. } | Eq::ArcSpan { .. }) && slope.abs() > 1e5 {
                        continue;
                    }
                    // So has an unsigned distance that crossed zero.
                    if (up - value).abs() > 1e3 * h * (1.0 + g[i].abs()) {
                        slope = g[i];
                    }
                    assert!((slope - g[i]).abs() < 1e-5 * (1.0 + g[i].abs()), "{eq:?} variable {i}: analytic {} numeric {slope}", g[i]);
                }
            }
        }
    }
}
