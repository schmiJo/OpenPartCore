//! Turns a sketch into variables and equations, and evaluates them.
//!
//! Every variable is a length: coordinates and radii, never angles. So "move as little as
//! possible" has one meaning across the whole sketch. Every equation's value is a length too,
//! measured in millimetres of error at the size of the geometry involved, so one tolerance
//! applies everywhere.

use crate::expr::{Env, Expr};
use crate::model::*;
use crate::solve::{ErrorKind, SketchError};
use std::collections::HashMap;

/// Most variables one equation uses.
pub(crate) const MAX_VARS: usize = 8;

/// What one equation computes. The comments list its variables, in order, as points where
/// that applies.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Eq {
    /// Up to four variables: `sum(coef * v) - value`.
    Linear { coef: [f64; 4], value: f64 },
    /// a, b: `|b - a| - value`.
    Dist { value: f64 },
    /// a, b, r: `|b - a| - r`.
    DistToRadius,
    /// a0, a1, b0, b1: `|a1 - a0| - |b1 - b0|`.
    EqualLength,
    /// p, a, b: the signed distance from p to line ab, or its size, minus `value`.
    PointLine { value: f64, unsigned: bool },
    /// p, a, b, r: `|distance from p to line ab| - r`.
    PointLineRadius,
    /// a0, a1, b0, b1: the sine of the angle between the directions, at `scale`.
    Parallel { scale: f64 },
    /// a0, a1, b0, b1: the cosine of the angle between the directions, at `scale`.
    Perpendicular { scale: f64 },
    /// a0, a1, b0, b1: the angle from a to b minus `theta`, wrapped, at `scale`.
    Angle { theta: f64, scale: f64 },
    /// c, s, e: the angle from cs to ce minus `theta`, wrapped, at `scale`.
    ArcSpan { theta: f64, scale: f64 },
    /// c0, c1, r0, r1: `|c1 - c0| - s0 r0 - s1 r1`.
    CircleTangent { s0: f64, s1: f64 },
    /// c0, c1, r0, r1: `|c1 - c0| - sqrt(r0^2 + r1^2)`.
    CircleNormal,
    /// p, q, a, b: the signed distance from the midpoint of pq to line ab.
    MidpointOnLine,
    /// p, q, a, b: `(q - p) · direction of ab`.
    PerpendicularToLine,
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
            Eq::DistToRadius => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                g[4] = -1.0;
                n - v(4)
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
            Eq::PointLineRadius => {
                let s = point_line(p(0), p(2), p(4), g, 0, 2, 4, 1.0);
                g[6] = -1.0;
                if s < 0.0 {
                    for gi in g.iter_mut().take(6) {
                        *gi = -*gi;
                    }
                    -s - v(6)
                } else {
                    s - v(6)
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
            Eq::CircleTangent { s0, s1 } => {
                let (n, u) = length_and_unit(p(0), p(2));
                set(g, 0, [-u[0], -u[1]]);
                set(g, 2, u);
                g[4] = -s0;
                g[5] = -s1;
                n - s0 * v(4) - s1 * v(5)
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
            Eq::MidpointOnLine => {
                let (a, b) = (p(4), p(6));
                let m = [(v(0) + v(2)) / 2.0, (v(1) + v(3)) / 2.0];
                let mut h = [0.0; MAX_VARS];
                let s = point_line(m, a, b, &mut h, 0, 2, 4, 1.0);
                g[0] = h[0] / 2.0;
                g[1] = h[1] / 2.0;
                g[2] = h[0] / 2.0;
                g[3] = h[1] / 2.0;
                g[4] = h[2];
                g[5] = h[3];
                g[6] = h[4];
                g[7] = h[5];
                s
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
        }
    }
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
                Geometry::Line { .. } => sys.spans.push([base, base + 1, base + 2, base + 3]),
                Geometry::Arc { .. } => sys.spans.push([base + 3, base + 4, base + 5, base + 6]),
                _ => {}
            }
            let values: Vec<f64> = match e.geometry {
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
                    sys.rows.push(row(None, Eq::DistToRadius, &[b, b + 1, point, point + 1, b + 2]));
                }
            }
        }
        let mut builder = Builder { sketch, env, sys, same: HashMap::new(), on_curve: Vec::new() };
        for c in &sketch.constraints {
            builder.note_contacts(c)?;
        }
        for (i, c) in sketch.constraints.iter().enumerate() {
            builder.add(i, c).map_err(|m| invalid(Some(c), m))?;
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
            let parts: &[(usize, Part)] = match e.geometry {
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
                Geometry::Point { .. } => Geometry::Point { at: p(0) },
                Geometry::Line { .. } => Geometry::Line { start: p(0), end: p(2) },
                Geometry::Circle { .. } => Geometry::Circle { center: p(0), radius: x[b + 2] },
                Geometry::Arc { .. } => Geometry::Arc { center: p(0), radius: x[b + 2], start: p(3), end: p(5) },
            };
        }
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
            Some(e) => Ok(Some(e.geometry)),
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
                (Geometry::Point { .. }, Part::Whole) => 0,
                (Geometry::Line { .. }, Part::Start) => 0,
                (Geometry::Line { .. }, Part::End) => 2,
                (Geometry::Circle { .. }, Part::Center) => 0,
                (Geometry::Arc { .. }, Part::Center) => 0,
                (Geometry::Arc { .. }, Part::Start) => 3,
                (Geometry::Arc { .. }, Part::End) => 5,
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
                (Ok(pa), Err(_)) if self.is_curve(b.entity) => self.on_curve.push((pa[0], b.entity)),
                (Err(_), Ok(pb)) if self.is_curve(a.entity) => self.on_curve.push((pb[0], a.entity)),
                _ => {}
            }
        }
        Ok(())
    }

    fn endpoints(&self, id: EntityId) -> Vec<[u32; 2]> {
        match self.geometry(id) {
            Ok(Some(Geometry::Line { .. } | Geometry::Arc { .. })) => {
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
                    self.push(row(Some(i), Eq::DistToRadius, &[center[0], center[1], p[0], p[1], r]));
                } else {
                    return Err(format!("{} is not a point, line, circle or arc", b.name()));
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
                } else {
                    return Err("equal needs two lines, or two circles or arcs".to_string());
                }
            }
            Rule::Midpoint { point, line } => {
                let (p, l) = (self.point(*point)?, self.line(*line)?);
                self.push(linear(i, 0.0, &[(p[0], 1.0), (l[0], -0.5), (l[2], -0.5)]));
                self.push(linear(i, 0.0, &[(p[1], 1.0), (l[1], -0.5), (l[3], -0.5)]));
            }
            Rule::Concentric { a, b } => {
                let (ca, cb) = (self.curve(*a)?.0, self.curve(*b)?.0);
                self.push(linear(i, 0.0, &[(ca[0], 1.0), (cb[0], -1.0)]));
                self.push(linear(i, 0.0, &[(ca[1], 1.0), (cb[1], -1.0)]));
            }
            Rule::Symmetric { a, b, about } => {
                let (p, q) = (self.point(*a)?, self.point(*b)?);
                if about.part == Part::Whole && self.is_line(about.entity) {
                    let l = self.line(about.entity)?;
                    self.push(row(Some(i), Eq::MidpointOnLine, &[p[0], p[1], q[0], q[1], l[0], l[1], l[2], l[3]]));
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
                    Some(s) => {
                        let (pr, lr) = if self.is_point(first) { (first, s) } else { (s, first) };
                        let p = self.point(pr).map_err(|_| "a distance needs a point or a line".to_string())?;
                        if self.is_point(lr) {
                            let q = self.point(lr)?;
                            self.push(row(Some(i), Eq::Dist { value: v }, &[p[0], p[1], q[0], q[1]]));
                        } else if lr.part == Part::Whole && self.is_line(lr.entity) {
                            let l = self.line(lr.entity)?;
                            self.push(row(Some(i), Eq::PointLine { value: v, unsigned: true }, &[p[0], p[1], l[0], l[1], l[2], l[3]]));
                        } else {
                            return Err(format!("a distance to {} isn't supported: use two points, or a point and a line", lr.name()));
                        }
                    }
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

    fn tangent(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
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
                self.push(row(Some(i), Eq::PointLineRadius, &[center[0], center[1], l[0], l[1], l[2], l[3], r]));
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
            self.push(row(Some(i), Eq::CircleTangent { s0, s1 }, &[c0[0], c0[1], c1[0], c1[1], r0, r1]));
        }
        Ok(())
    }

    fn normal(&mut self, i: usize, a: EntityId, b: EntityId) -> Result<(), String> {
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
            (Eq::DistToRadius, 5),
            (Eq::EqualLength, 8),
            (Eq::PointLine { value: 2.0, unsigned: false }, 6),
            (Eq::PointLine { value: 2.0, unsigned: true }, 6),
            (Eq::PointLineRadius, 7),
            (Eq::Parallel { scale: 12.0 }, 8),
            (Eq::Perpendicular { scale: 12.0 }, 8),
            (Eq::Angle { theta: 0.7, scale: 9.0 }, 8),
            (Eq::ArcSpan { theta: 1.3, scale: 5.0 }, 6),
            (Eq::CircleTangent { s0: 1.0, s1: -1.0 }, 6),
            (Eq::CircleNormal, 6),
            (Eq::MidpointOnLine, 8),
            (Eq::PerpendicularToLine, 8),
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
