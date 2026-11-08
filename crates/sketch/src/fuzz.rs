//! Randomized sketches with known answers, shared by the tests, `examples/fuzz.rs` and the check
//! against FreeCAD's PlaneGCS.
//!
//! Each sketch is built from an exact true configuration, so its constraints are known to have a
//! solution, and then its geometry is nudged away from it for the solver to find its way back. Every
//! case replays from `(category, seed)`.
//!
//! `violations` measures constraints directly from geometry with formulas of its own, separate from
//! the solver's equations, so a mistake in those can't hide from it.

use crate::model::*;
use crate::rng::Rng;
use crate::solve::{ErrorKind, SketchError};
use std::f64::consts::{PI, TAU};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Category {
    Rectilinear,
    Polygons,
    Rectangles,
    Slots,
    RoundedCorners,
    Circles,
    Symmetry,
    Mixed,
    Large,
    Redundant,
    Conflict,
    Impossible,
    /// Last, because a category's number seeds its cases and the others must keep theirs.
    Distances,
}

pub const ALL_CATEGORIES: [Category; 13] = [
    Category::Rectilinear,
    Category::Polygons,
    Category::Rectangles,
    Category::Slots,
    Category::RoundedCorners,
    Category::Circles,
    Category::Symmetry,
    Category::Mixed,
    Category::Distances,
    Category::Large,
    Category::Redundant,
    Category::Conflict,
    Category::Impossible,
];

impl Category {
    pub fn name(self) -> &'static str {
        match self {
            Category::Rectilinear => "rectilinear",
            Category::Polygons => "polygons",
            Category::Rectangles => "rectangles",
            Category::Slots => "slots",
            Category::RoundedCorners => "rounded",
            Category::Circles => "circles",
            Category::Symmetry => "symmetry",
            Category::Mixed => "mixed",
            Category::Distances => "distances",
            Category::Large => "large",
            Category::Redundant => "redundant",
            Category::Conflict => "conflict",
            Category::Impossible => "impossible",
        }
    }

    pub fn parse(text: &str) -> Option<Category> {
        ALL_CATEGORIES.iter().copied().find(|c| c.name() == text)
    }
}

/// What the right answer is.
#[derive(Clone, Debug, PartialEq)]
pub enum Expect {
    Solvable,
    /// Solvable, and the constraints added last are implied by the others.
    Redundant(Vec<ConstraintId>),
    /// These constraints can't all hold.
    Conflict(Vec<ConstraintId>),
    /// No solution exists, though no two constraints contradict directly.
    Impossible,
}

pub struct Case {
    pub category: Category,
    pub seed: u64,
    /// Exact geometry that meets every constraint, when one exists.
    pub truth: Sketch,
    /// The same sketch with its geometry nudged, for the solver to start from.
    pub start: Sketch,
    pub expect: Expect,
}

fn polar(c: P2, r: f64, a: f64) -> P2 {
    [c[0] + r * a.cos(), c[1] + r * a.sin()]
}

fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn signed_distance(p: P2, a: P2, b: P2) -> f64 {
    let u = sub(b, a);
    cross(u, sub(p, a)) / u[0].hypot(u[1]).max(1e-300)
}

fn wrap(a: f64) -> f64 {
    a - TAU * (a / TAU).round()
}

fn line_of(s: &Sketch, id: EntityId) -> Option<(P2, P2)> {
    match id {
        X_AXIS => return Some(([0.0, 0.0], [1.0, 0.0])),
        Y_AXIS => return Some(([0.0, 0.0], [0.0, 1.0])),
        _ => {}
    }
    match s.entity(id)?.geometry {
        Geometry::Line { start, end } => Some((start, end)),
        _ => None,
    }
}

fn curve_of(s: &Sketch, id: EntityId) -> Option<(P2, f64)> {
    match s.entity(id)?.geometry {
        Geometry::Circle { center, radius } | Geometry::Arc { center, radius, .. } => Some((center, radius)),
        _ => None,
    }
}

fn point_of(s: &Sketch, r: Ref) -> Option<P2> {
    match (r.entity, r.part) {
        (ORIGIN, Part::Whole) => return Some([0.0, 0.0]),
        (X_AXIS | Y_AXIS, Part::Start) => return Some([0.0, 0.0]),
        (X_AXIS, Part::End) => return Some([1.0, 0.0]),
        (Y_AXIS, Part::End) => return Some([0.0, 1.0]),
        _ => {}
    }
    match (s.entity(r.entity)?.geometry, r.part) {
        (Geometry::Point { at }, Part::Whole) => Some(at),
        (Geometry::Line { start, .. }, Part::Start) => Some(start),
        (Geometry::Line { end, .. }, Part::End) => Some(end),
        (Geometry::Circle { center, .. } | Geometry::Arc { center, .. }, Part::Center) => Some(center),
        (Geometry::Arc { start, .. }, Part::Start) => Some(start),
        (Geometry::Arc { end, .. }, Part::End) => Some(end),
        _ => None,
    }
}

fn shared_end(s: &Sketch, a: EntityId, b: EntityId) -> Option<P2> {
    let ends = |e: EntityId| [point_of(s, e.start()), point_of(s, e.end())];
    for p in ends(a).into_iter().flatten() {
        for q in ends(b).into_iter().flatten() {
            if dist(p, q) < 1e-7 {
                return Some(p);
            }
        }
    }
    None
}

/// How far a sketch's geometry is from meeting one constraint, in millimetres. `None` if the
/// constraint doesn't make sense for that geometry.
pub fn violation(s: &Sketch, rule: &Rule) -> Option<f64> {
    let num = |e: &crate::expr::Expr| e.as_number();
    Some(match rule {
        Rule::Coincident { a, b } => {
            let (first, second) = (point_of(s, *a), point_of(s, *b));
            match (first, second) {
                (Some(p), Some(q)) => dist(p, q),
                (Some(p), None) | (None, Some(p)) => {
                    let edge = if first.is_some() { b.entity } else { a.entity };
                    if let Some((c, r)) = curve_of(s, edge) {
                        (dist(p, c) - r).abs()
                    } else {
                        let (u, v) = line_of(s, edge)?;
                        signed_distance(p, u, v).abs()
                    }
                }
                _ => return None,
            }
        }
        Rule::Horizontal { a, b } | Rule::Vertical { a, b } => {
            let (p, q) = match b {
                Some(b) => (point_of(s, *a)?, point_of(s, *b)?),
                None => line_of(s, a.entity)?,
            };
            if matches!(rule, Rule::Horizontal { .. }) {
                (p[1] - q[1]).abs()
            } else {
                (p[0] - q[0]).abs()
            }
        }
        Rule::Parallel { a, b } | Rule::Perpendicular { a, b } => {
            let ((a0, a1), (b0, b1)) = (line_of(s, *a)?, line_of(s, *b)?);
            let (u, v) = (sub(a1, a0), sub(b1, b0));
            let (nu, nv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
            let measure = if matches!(rule, Rule::Parallel { .. }) { cross(u, v) } else { dot(u, v) };
            (measure / (nu * nv).max(1e-300)).abs() * nu.max(nv)
        }
        Rule::Tangent { a, b } => {
            if let (Some((c, r)), Some((p, q))) = (curve_of(s, *a), line_of(s, *b)) {
                (signed_distance(c, p, q).abs() - r).abs()
            } else if let (Some((p, q)), Some((c, r))) = (line_of(s, *a), curve_of(s, *b)) {
                (signed_distance(c, p, q).abs() - r).abs()
            } else {
                let ((c0, r0), (c1, r1)) = (curve_of(s, *a)?, curve_of(s, *b)?);
                let d = dist(c0, c1);
                (d - (r0 + r1)).abs().min((d - (r0 - r1).abs()).abs())
            }
        }
        Rule::Equal { a, b } => {
            if let (Some((a0, a1)), Some((b0, b1))) = (line_of(s, *a), line_of(s, *b)) {
                (dist(a0, a1) - dist(b0, b1)).abs()
            } else {
                (curve_of(s, *a)?.1 - curve_of(s, *b)?.1).abs()
            }
        }
        Rule::Midpoint { point, line } => {
            let p = point_of(s, *point)?;
            let (a, b) = line_of(s, *line)?;
            dist(p, [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0])
        }
        Rule::Concentric { a, b } => dist(curve_of(s, *a)?.0, curve_of(s, *b)?.0),
        Rule::Symmetric { a, b, about } => {
            let (p, q) = (point_of(s, *a)?, point_of(s, *b)?);
            if let Some(m) = point_of(s, *about) {
                dist([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0], m)
            } else {
                let (u, v) = line_of(s, about.entity)?;
                // The mirror image of p must land on q.
                let d = sub(v, u);
                let unit = [d[0] / d[0].hypot(d[1]), d[1] / d[0].hypot(d[1])];
                let w = sub(p, u);
                let along = dot(w, unit);
                let image = [u[0] + 2.0 * along * unit[0] - w[0], u[1] + 2.0 * along * unit[1] - w[1]];
                dist(image, q)
            }
        }
        Rule::Normal { a, b } => {
            if let (Some((c, _)), Some((p, q))) = (curve_of(s, *a), line_of(s, *b)) {
                signed_distance(c, p, q).abs()
            } else if let (Some((p, q)), Some((c, _))) = (line_of(s, *a), curve_of(s, *b)) {
                signed_distance(c, p, q).abs()
            } else {
                let ((c0, r0), (c1, r1)) = (curve_of(s, *a)?, curve_of(s, *b)?);
                (dist(c0, c1) - r0.hypot(r1)).abs()
            }
        }
        Rule::Fix { what, at } => {
            if let Some(p) = point_of(s, *what) {
                dist(p, [*at.first()?, *at.get(1)?])
            } else {
                match s.entity(what.entity)?.geometry {
                    Geometry::Line { start, end } => dist(start, [at[0], at[1]]).max(dist(end, [at[2], at[3]])),
                    Geometry::Circle { center, radius } => dist(center, [at[0], at[1]]).max((radius - at[2]).abs()),
                    Geometry::Arc { center, radius, start, end } => dist(center, [at[0], at[1]])
                        .max((radius - at[2]).abs())
                        .max(dist(start, [at[3], at[4]]))
                        .max(dist(end, [at[5], at[6]])),
                    Geometry::Point { .. } => return None,
                }
            }
        }
        Rule::Distance { a, b, value } => {
            let v = num(value)?;
            match b {
                None => {
                    let (p, q) = line_of(s, a.entity)?;
                    (dist(p, q) - v).abs()
                }
                Some(b) => {
                    let (first, second) = (point_of(s, *a), point_of(s, *b));
                    let curve = |r: Ref| curve_of(s, r.entity);
                    let line = |r: Ref| line_of(s, r.entity);
                    let gap = match (first, second) {
                        (Some(p), Some(q)) => dist(p, q),
                        (Some(p), None) | (None, Some(p)) => {
                            let other = if first.is_some() { *b } else { *a };
                            match (line(other), curve(other)) {
                                (Some((u, w)), _) => signed_distance(p, u, w).abs(),
                                (_, Some((c, r))) => (dist(p, c) - r).abs(),
                                _ => return None,
                            }
                        }
                        _ => match (line(*a), curve(*a), line(*b), curve(*b)) {
                            (Some((u, w)), _, _, Some((c, r))) | (_, Some((c, r)), Some((u, w)), _) => (signed_distance(c, u, w).abs() - r).abs(),
                            (_, Some((c0, r0)), _, Some((c1, r1))) => {
                                let d = dist(c0, c1);
                                if d >= r0 + r1 {
                                    d - r0 - r1
                                } else if d <= (r0 - r1).abs() {
                                    (r0 - r1).abs() - d
                                } else {
                                    0.0
                                }
                            }
                            (Some((p, q)), _, Some((u, w)), _) => signed_distance([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0], u, w).abs(),
                            _ => return None,
                        },
                    };
                    (gap - v).abs()
                }
            }
        }
        Rule::HorizontalDistance { a, b, value } => (point_of(s, *b)?[0] - point_of(s, *a)?[0] - num(value)?).abs(),
        Rule::VerticalDistance { a, b, value } => (point_of(s, *b)?[1] - point_of(s, *a)?[1] - num(value)?).abs(),
        Rule::Radius { of, value } => (curve_of(s, *of)?.1 - num(value)?).abs(),
        Rule::Diameter { of, value } => (2.0 * curve_of(s, *of)?.1 - num(value)?).abs(),
        Rule::Angle { a, b, value } => {
            let theta = num(value)?.to_radians();
            match b {
                Some(b) => {
                    let ((a0, a1), (b0, b1)) = (line_of(s, *a)?, line_of(s, *b)?);
                    let (u, v) = (sub(a1, a0), sub(b1, b0));
                    wrap(v[1].atan2(v[0]) - u[1].atan2(u[0]) - theta).abs() * u[0].hypot(u[1]).max(v[0].hypot(v[1]))
                }
                None => {
                    if let Some((p, q)) = line_of(s, *a) {
                        let u = sub(q, p);
                        wrap(u[1].atan2(u[0]) - theta).abs() * u[0].hypot(u[1])
                    } else if let Geometry::Arc { center, radius, start, end } = s.entity(*a)?.geometry {
                        let (u, v) = (sub(start, center), sub(end, center));
                        wrap(v[1].atan2(v[0]) - u[1].atan2(u[0]) - theta).abs() * radius
                    } else {
                        return None;
                    }
                }
            }
        }
    })
}

/// Every constraint a sketch's geometry fails by more than `tol`, and any arc whose ends are off
/// its circle.
pub fn violations(s: &Sketch, tol: f64) -> Vec<String> {
    let mut out = Vec::new();
    for c in s.constraints.iter().filter(|c| !c.reference) {
        // An endpoint tangency says more than the distance check above: the radius at the shared
        // point must be square to the line. It is checked here since it needs the shared point.
        let extra = match c.rule {
            Rule::Tangent { a, b } => shared_end(s, a, b).and_then(|p| match (line_of(s, a), line_of(s, b)) {
                (Some((u, v)), None) | (None, Some((u, v))) => {
                    let center = if line_of(s, a).is_some() { curve_of(s, b)?.0 } else { curve_of(s, a)?.0 };
                    let d = sub(v, u);
                    Some(dot(sub(p, center), d).abs() / d[0].hypot(d[1]).max(1e-300))
                }
                _ => None,
            }),
            _ => None,
        };
        match violation(s, &c.rule) {
            Some(v) if v.max(extra.unwrap_or(0.0)) <= tol => {}
            Some(v) => out.push(format!("c{} ({}) is off by {v:.3e}", c.id.0, c.rule.kind())),
            None => out.push(format!("c{} ({}) doesn't fit its geometry", c.id.0, c.rule.kind())),
        }
    }
    for e in &s.entities {
        if let Geometry::Arc { center, radius, start, end } = e.geometry {
            let off = (dist(center, start) - radius).abs().max((dist(center, end) - radius).abs());
            if off > tol {
                out.push(format!("arc {} has an end off its circle by {off:.3e}", e.id.0));
            }
        }
    }
    out
}

// ----- Building blocks. Each adds entities and constraints that hold exactly as built. -----

struct Gen<'a> {
    s: Sketch,
    rng: &'a mut Rng,
}

impl Gen<'_> {
    fn on(&mut self, rule: Rule) -> ConstraintId {
        self.s.constrain(rule)
    }

    fn join(&mut self, a: Ref, b: Ref) {
        self.on(Rule::Coincident { a, b });
    }

    fn length_of(&mut self, line: EntityId) {
        let (p, q) = line_of(&self.s, line).unwrap();
        self.on(Rule::Distance { a: line.whole(), b: None, value: dist(p, q).into() });
    }

    fn measure_points(&mut self, a: Ref, b: Ref) {
        let (p, q) = (point_of(&self.s, a).unwrap(), point_of(&self.s, b).unwrap());
        match self.rng.int(0, 2) {
            0 if dist(p, q) > 1e-3 => {
                self.on(Rule::Distance { a, b: Some(b), value: dist(p, q).into() });
            }
            1 => {
                self.on(Rule::HorizontalDistance { a, b, value: (q[0] - p[0]).into() });
            }
            _ => {
                self.on(Rule::VerticalDistance { a, b, value: (q[1] - p[1]).into() });
            }
        }
    }

    fn angle_between(&mut self, a: EntityId, b: EntityId) {
        let ((a0, a1), (b0, b1)) = (line_of(&self.s, a).unwrap(), line_of(&self.s, b).unwrap());
        let (u, v) = (sub(a1, a0), sub(b1, b0));
        let value = wrap(v[1].atan2(v[0]) - u[1].atan2(u[0])).to_degrees();
        self.on(Rule::Angle { a, b: Some(b), value: value.into() });
    }

    fn pin_somewhere(&mut self, r: Ref) {
        let p = point_of(&self.s, r).unwrap();
        self.on(Rule::Fix { what: r, at: p.to_vec() });
    }

    /// A closed chain of lines through these points, with its corners joined.
    fn chain(&mut self, points: &[P2]) -> Vec<EntityId> {
        let n = points.len();
        let lines: Vec<EntityId> = (0..n).map(|i| self.s.line(points[i], points[(i + 1) % n])).collect();
        for i in 0..n {
            self.join(lines[i].end(), lines[(i + 1) % n].start());
        }
        lines
    }

    fn rectilinear(&mut self, c: P2) -> Vec<EntityId> {
        // A staircase outline, always a simple polygon.
        let steps = self.rng.int(1, 4) as usize;
        let mut points = vec![[c[0], c[1]]];
        let mut x = c[0];
        let mut y = c[1];
        for _ in 0..steps {
            x += self.rng.range(5.0, 25.0);
            points.push([x, y]);
            y += self.rng.range(5.0, 25.0);
            points.push([x, y]);
        }
        points.push([c[0], y]);
        let lines = self.chain(&points);
        for (i, &l) in lines.iter().enumerate() {
            let rule = if i % 2 == 0 { Rule::Horizontal { a: l.whole(), b: None } } else { Rule::Vertical { a: l.whole(), b: None } };
            self.on(rule);
        }
        for &l in &lines {
            if self.rng.chance(0.6) {
                self.length_of(l);
            }
        }
        lines
    }

    fn polygon(&mut self, c: P2) -> Vec<EntityId> {
        let n = self.rng.int(3, 8) as usize;
        let start = self.rng.range(0.0, TAU);
        let points: Vec<P2> = (0..n)
            .map(|i| {
                let a = start + TAU * (i as f64 + self.rng.range(-0.2, 0.2)) / n as f64;
                polar(c, self.rng.range(12.0, 30.0), a)
            })
            .collect();
        let lines = self.chain(&points);
        for i in 0..n {
            match self.rng.int(0, 3) {
                0 => self.length_of(lines[i]),
                1 => self.angle_between(lines[i], lines[(i + 1) % n]),
                2 => {
                    let a = lines[i];
                    let b = lines[(i + 2) % n];
                    if self.rng.chance(0.5) {
                        self.measure_points(a.start(), b.start());
                    }
                }
                _ => {}
            }
        }
        if self.rng.chance(0.5) {
            self.pin_somewhere(lines[0].start());
        }
        lines
    }

    fn rectangle(&mut self, c: P2) -> Vec<EntityId> {
        let (w, h) = (self.rng.range(8.0, 40.0), self.rng.range(8.0, 40.0));
        let a = self.rng.range(0.0, TAU);
        let (u, v) = ([a.cos(), a.sin()], [-a.sin(), a.cos()]);
        let at = |x: f64, y: f64| [c[0] + u[0] * x + v[0] * y, c[1] + u[1] * x + v[1] * y];
        let points = [at(0.0, 0.0), at(w, 0.0), at(w, h), at(0.0, h)];
        let lines = self.chain(&points);
        self.on(Rule::Parallel { a: lines[0], b: lines[2] });
        self.on(Rule::Parallel { a: lines[1], b: lines[3] });
        self.on(Rule::Perpendicular { a: lines[0], b: lines[1] });
        match self.rng.int(0, 3) {
            0 => {
                self.length_of(lines[0]);
                self.length_of(lines[1]);
            }
            1 => {
                self.length_of(lines[2]);
                let (p, q) = (point_of(&self.s, lines[0].start()).unwrap(), point_of(&self.s, lines[2].start()).unwrap());
                self.on(Rule::Distance { a: lines[0].start(), b: Some(lines[2].start()), value: dist(p, q).into() });
            }
            2 => {
                let m = self.s.point((points[0][0] + points[1][0]) / 2.0, (points[0][1] + points[1][1]) / 2.0);
                self.on(Rule::Midpoint { point: m.whole(), line: lines[0] });
                let d = signed_distance(point_of(&self.s, m.whole()).unwrap(), points[2], points[3]).abs();
                self.on(Rule::Distance { a: m.whole(), b: Some(lines[2].whole()), value: d.into() });
            }
            _ => {}
        }
        if self.rng.chance(0.5) {
            self.pin_somewhere(lines[0].start());
            let (p, q) = (points[0], points[1]);
            let angle = (q[1] - p[1]).atan2(q[0] - p[0]).to_degrees();
            self.on(Rule::Angle { a: lines[0], b: None, value: angle.into() });
        }
        lines
    }

    fn slot(&mut self, c: P2) -> Vec<EntityId> {
        let (len, r) = (self.rng.range(10.0, 40.0), self.rng.range(3.0, 10.0));
        let a = self.rng.range(0.0, TAU);
        let (u, v) = ([a.cos(), a.sin()], [-a.sin(), a.cos()]);
        let at = |x: f64, y: f64| [c[0] + u[0] * x + v[0] * y, c[1] + u[1] * x + v[1] * y];
        let deg = a.to_degrees();
        let right = self.s.arc(at(len, 0.0), r, deg - 90.0, deg + 90.0);
        let left = self.s.arc(at(0.0, 0.0), r, deg + 90.0, deg + 270.0);
        let bottom = self.s.line(at(0.0, -r), at(len, -r));
        let top = self.s.line(at(len, r), at(0.0, r));
        self.join(bottom.end(), right.start());
        self.join(right.end(), top.start());
        self.join(top.end(), left.start());
        self.join(left.end(), bottom.start());
        for (l, curve) in [(bottom, right), (top, right), (top, left), (bottom, left)] {
            self.on(Rule::Tangent { a: l, b: curve });
        }
        self.on(Rule::Equal { a: left, b: right });
        match self.rng.int(0, 2) {
            0 => {
                self.on(Rule::Radius { of: left, value: r.into() });
                self.length_of(bottom);
            }
            1 => {
                self.on(Rule::Diameter { of: right, value: (2.0 * r).into() });
                self.on(Rule::Distance { a: left.center(), b: Some(right.center()), value: len.into() });
            }
            _ => {}
        }
        if self.rng.chance(0.4) {
            self.pin_somewhere(left.center());
        }
        vec![bottom, right, top, left]
    }

    fn rounded(&mut self, c: P2) -> Vec<EntityId> {
        let (w, h) = (self.rng.range(20.0, 50.0), self.rng.range(20.0, 50.0));
        let r = self.rng.range(2.0, 8.0);
        let (x0, y0, x1, y1) = (c[0], c[1], c[0] + w, c[1] + h);
        let bottom = self.s.line([x0 + r, y0], [x1 - r, y0]);
        let corner0 = self.s.arc([x1 - r, y0 + r], r, -90.0, 0.0);
        let right = self.s.line([x1, y0 + r], [x1, y1 - r]);
        let corner1 = self.s.arc([x1 - r, y1 - r], r, 0.0, 90.0);
        let top = self.s.line([x1 - r, y1], [x0 + r, y1]);
        let corner2 = self.s.arc([x0 + r, y1 - r], r, 90.0, 180.0);
        let left = self.s.line([x0, y1 - r], [x0, y0 + r]);
        let corner3 = self.s.arc([x0 + r, y0 + r], r, 180.0, 270.0);
        let order = [(bottom, corner0), (right, corner1), (top, corner2), (left, corner3)];
        let lines = [bottom, right, top, left];
        for (i, &(line, arc)) in order.iter().enumerate() {
            self.join(line.end(), arc.start());
            self.join(arc.end(), lines[(i + 1) % 4].start());
            self.on(Rule::Tangent { a: line, b: arc });
            self.on(Rule::Tangent { a: arc, b: lines[(i + 1) % 4] });
        }
        self.on(Rule::Horizontal { a: bottom.whole(), b: None });
        self.on(Rule::Vertical { a: right.whole(), b: None });
        self.on(Rule::Horizontal { a: top.whole(), b: None });
        self.on(Rule::Vertical { a: left.whole(), b: None });
        for a in [corner1, corner2, corner3] {
            self.on(Rule::Equal { a: corner0, b: a });
        }
        if self.rng.chance(0.7) {
            self.on(Rule::Radius { of: corner0, value: r.into() });
            self.length_of(bottom);
            self.length_of(right);
        }
        if self.rng.chance(0.4) {
            self.pin_somewhere(bottom.start());
        }
        vec![bottom, right, top, left]
    }

    /// Dimensions to circles and arcs, between parallel lines, and reference dimensions.
    fn distances(&mut self, c: P2) -> Vec<EntityId> {
        let r0 = self.rng.range(8.0, 16.0);
        let a = self.s.circle(c, r0);
        self.on(Rule::Radius { of: a, value: r0.into() });
        let mut made = vec![a];
        // A point outside the circle, and one inside it.
        if self.rng.chance(0.7) {
            let gap = self.rng.range(1.0, 10.0);
            let p = polar(c, r0 + gap, self.rng.range(0.0, TAU));
            let e = self.s.point(p[0], p[1]);
            self.on(Rule::Distance { a: e.whole(), b: Some(a.whole()), value: gap.into() });
        }
        if self.rng.chance(0.5) {
            let gap = self.rng.range(1.0, r0 * 0.7);
            let p = polar(c, r0 - gap, self.rng.range(0.0, TAU));
            let e = self.s.point(p[0], p[1]);
            self.on(Rule::Distance { a: a.whole(), b: Some(e.whole()), value: gap.into() });
        }
        // A line clear of the circle.
        if self.rng.chance(0.7) {
            let (gap, t) = (self.rng.range(1.0, 10.0), self.rng.range(0.0, TAU));
            let foot = polar(c, r0 + gap, t);
            let dir = [-t.sin(), t.cos()];
            let (m, n) = (self.rng.range(2.0, 20.0), self.rng.range(2.0, 20.0));
            let l = self.s.line([foot[0] - dir[0] * m, foot[1] - dir[1] * m], [foot[0] + dir[0] * n, foot[1] + dir[1] * n]);
            self.on(Rule::Distance { a: l.whole(), b: Some(a.whole()), value: gap.into() });
            made.push(l);
        }
        // A circle clear of it, and one inside it.
        if self.rng.chance(0.7) {
            let (r1, gap, t) = (self.rng.range(2.0, 8.0), self.rng.range(1.0, 10.0), self.rng.range(0.0, TAU));
            let b = self.s.circle(polar(c, r0 + r1 + gap, t), r1);
            self.on(Rule::Distance { a: a.whole(), b: Some(b.whole()), value: gap.into() });
            made.push(b);
        }
        if self.rng.chance(0.5) {
            let r1 = self.rng.range(1.0, r0 * 0.4);
            let gap = self.rng.range(0.5, r0 - r1 - 0.5);
            let b = self.s.circle(polar(c, r0 - r1 - gap, self.rng.range(0.0, TAU)), r1);
            self.on(Rule::Distance { a: b.whole(), b: Some(a.whole()), value: gap.into() });
            made.push(b);
        }
        // An arc, which counts as its full circle.
        if self.rng.chance(0.6) {
            let far = polar(c, r0 + 45.0, self.rng.range(0.0, TAU));
            let r1 = self.rng.range(5.0, 12.0);
            let d0 = self.rng.range(0.0, 200.0);
            let arc = self.s.arc(far, r1, d0, d0 + self.rng.range(40.0, 140.0));
            self.on(Rule::Radius { of: arc, value: r1.into() });
            let (gap, t) = (self.rng.range(1.0, 6.0), self.rng.range(0.0, TAU));
            let p = polar(far, r1 + gap, t);
            let e = self.s.point(p[0], p[1]);
            self.on(Rule::Distance { a: e.whole(), b: Some(arc.whole()), value: gap.into() });
            made.push(arc);
        }
        // Two parallel lines, with no point picked. The length keeps the first from collapsing.
        let t = self.rng.range(0.0, TAU);
        let (dir, side) = ([t.cos(), t.sin()], [-t.sin(), t.cos()]);
        let at = polar(c, r0 + 70.0, self.rng.range(0.0, TAU));
        let (len, gap, slide) = (self.rng.range(5.0, 30.0), self.rng.range(1.0, 15.0), self.rng.range(-10.0, 10.0));
        let (s0, e0) = (at, [at[0] + dir[0] * len, at[1] + dir[1] * len]);
        let s1 = [at[0] + side[0] * gap + dir[0] * slide, at[1] + side[1] * gap + dir[1] * slide];
        let e1 = [s1[0] + dir[0] * len * 0.7, s1[1] + dir[1] * len * 0.7];
        let (l0, l1) = (self.s.line(s0, e0), self.s.line(s1, e1));
        self.on(Rule::Parallel { a: l0, b: l1 });
        self.on(Rule::Distance { a: l0.whole(), b: Some(l1.whole()), value: gap.into() });
        self.length_of(l0);
        self.length_of(l1);
        made.push(l0);
        made.push(l1);
        // Reference dimensions, whose values mean nothing until they're measured.
        if self.rng.chance(0.7) {
            let _ = self.s.add_reference(Rule::Distance { a: l0.start(), b: Some(l1.end()), value: 1234.5.into() });
            let _ = self.s.add_reference(Rule::Diameter { of: a, value: 1234.5.into() });
            let _ = self.s.add_reference(Rule::Angle { a: l0, b: None, value: 1234.5.into() });
        }
        made
    }

    fn circles(&mut self, c: P2) -> Vec<EntityId> {
        let r0 = self.rng.range(6.0, 16.0);
        let a = self.s.circle(c, r0);
        let mut made = vec![a];
        if self.rng.chance(0.7) {
            self.on(Rule::Radius { of: a, value: r0.into() });
        }
        // Tangent from outside.
        let r1 = self.rng.range(3.0, 10.0);
        let t = self.rng.range(0.0, TAU);
        let b = self.s.circle(polar(c, r0 + r1, t), r1);
        self.on(Rule::Tangent { a, b });
        made.push(b);
        // Tangent from inside.
        if self.rng.chance(0.5) {
            let r2 = self.rng.range(1.0, r0 * 0.7);
            let t2 = self.rng.range(0.0, TAU);
            let d = self.s.circle(polar(c, r0 - r2, t2), r2);
            self.on(Rule::Tangent { a, b: d });
            made.push(d);
        }
        // Concentric and equal.
        if self.rng.chance(0.5) {
            let r3 = self.rng.range(r0 + 1.0, r0 + 6.0);
            let e = self.s.circle(c, r3);
            self.on(Rule::Concentric { a, b: e });
            self.on(Rule::Diameter { of: e, value: (2.0 * r3).into() });
        }
        if self.rng.chance(0.5) {
            let far = polar(c, r0 + 40.0, self.rng.range(0.0, TAU));
            let f = self.s.circle(far, r1);
            self.on(Rule::Equal { a: b, b: f });
        }
        // A line tangent to the circle, sharing no point with it.
        if self.rng.chance(0.7) {
            let t3 = self.rng.range(0.0, TAU);
            let touch = polar(c, r0, t3);
            let dir = [-t3.sin(), t3.cos()];
            let (m, n) = (self.rng.range(2.0, 20.0), self.rng.range(2.0, 20.0));
            let l = self.s.line([touch[0] - dir[0] * m, touch[1] - dir[1] * m], [touch[0] + dir[0] * n, touch[1] + dir[1] * n]);
            self.on(Rule::Tangent { a: l, b: a });
            made.push(l);
        }
        // A line through the center.
        if self.rng.chance(0.6) {
            let t4 = self.rng.range(0.0, TAU);
            let (m, n) = (self.rng.range(1.0, 20.0), self.rng.range(1.0, 20.0));
            let l = self.s.line(polar(c, -m, t4), polar(c, n, t4));
            self.on(Rule::Normal { a: l, b: a });
            made.push(l);
        }
        // Points on things.
        if self.rng.chance(0.6) {
            let p = polar(c, r0, self.rng.range(0.0, TAU));
            let e = self.s.point(p[0], p[1]);
            self.on(Rule::Coincident { a: e.whole(), b: a.whole() });
        }
        // Two circles crossing square.
        if self.rng.chance(0.4) {
            let r5 = self.rng.range(3.0, 12.0);
            let f = self.s.circle(polar(c, r0.hypot(r5), self.rng.range(0.0, TAU)), r5);
            self.on(Rule::Normal { a, b: f });
        }
        // An arc with its span set.
        if self.rng.chance(0.4) {
            let start = self.rng.range(0.0, 360.0);
            let span = self.rng.range(20.0, 300.0);
            let center = polar(c, 45.0, self.rng.range(0.0, TAU));
            let radius = self.rng.range(4.0, 12.0);
            let arc = self.s.arc(center, radius, start, start + span);
            self.on(Rule::Angle { a: arc, b: None, value: span.into() });
            self.on(Rule::Radius { of: arc, value: radius.into() });
        }
        if self.rng.chance(0.4) {
            self.pin_somewhere(a.center());
        }
        made
    }

    fn symmetry(&mut self, c: P2) -> Vec<EntityId> {
        let t = self.rng.range(0.0, PI);
        let unit = [t.cos(), t.sin()];
        let span = self.rng.range(15.0, 40.0);
        let axis = self.s.line([c[0] - unit[0] * span, c[1] - unit[1] * span], [c[0] + unit[0] * span, c[1] + unit[1] * span]);
        self.s.set_construction(axis, true).unwrap();
        let mut made = vec![axis];
        let mirror = |p: P2| {
            let w = sub(p, c);
            let along = dot(w, unit);
            [c[0] + 2.0 * along * unit[0] - w[0], c[1] + 2.0 * along * unit[1] - w[1]]
        };
        for _ in 0..self.rng.int(1, 4) {
            let p = [c[0] + self.rng.range(-25.0, 25.0), c[1] + self.rng.range(-25.0, 25.0)];
            let q = mirror(p);
            if dist(p, q) < 1.0 {
                continue;
            }
            if self.rng.chance(0.5) {
                let (a, b) = (self.s.point(p[0], p[1]), self.s.point(q[0], q[1]));
                self.on(Rule::Symmetric { a: a.whole(), b: b.whole(), about: axis.whole() });
            } else {
                let l = self.s.line(p, q);
                self.on(Rule::Symmetric { a: l.start(), b: l.end(), about: axis.whole() });
                made.push(l);
            }
        }
        // Mirrored through a point.
        if self.rng.chance(0.6) {
            let center = self.s.point(c[0] + 3.0, c[1] - 2.0);
            let m = [c[0] + 3.0, c[1] - 2.0];
            let p = [m[0] + self.rng.range(4.0, 20.0), m[1] + self.rng.range(4.0, 20.0)];
            let q = [2.0 * m[0] - p[0], 2.0 * m[1] - p[1]];
            let (a, b) = (self.s.point(p[0], p[1]), self.s.point(q[0], q[1]));
            self.on(Rule::Symmetric { a: a.whole(), b: b.whole(), about: center.whole() });
        }
        // A midpoint.
        if self.rng.chance(0.6) {
            let (p, q) = ([c[0] + self.rng.range(-20.0, 20.0), c[1] + 30.0], [c[0] + self.rng.range(-20.0, 20.0), c[1] + 45.0]);
            let l = self.s.line(p, q);
            let m = self.s.point((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0);
            self.on(Rule::Midpoint { point: m.whole(), line: l });
            made.push(l);
        }
        if self.rng.chance(0.5) {
            self.pin_somewhere(axis.start());
            self.pin_somewhere(axis.end());
        }
        made
    }

    fn block(&mut self, c: P2) -> Vec<EntityId> {
        match self.rng.int(0, 6) {
            0 => self.rectilinear(c),
            1 => self.polygon(c),
            2 => self.rectangle(c),
            3 => self.slot(c),
            4 => self.rounded(c),
            5 => self.circles(c),
            _ => self.symmetry(c),
        }
    }
}

fn nudge(p: P2, rng: &mut Rng, m: f64) -> P2 {
    [p[0] + rng.range(-m, m), p[1] + rng.range(-m, m)]
}

/// Moves each entity's geometry by up to `m`, keeping arcs' ends on their circles.
pub fn perturb(s: &Sketch, rng: &mut Rng, m: f64) -> Sketch {
    let mut out = s.clone();
    for e in &mut out.entities {
        e.geometry = match e.geometry {
            Geometry::Point { at } => Geometry::Point { at: nudge(at, rng, m) },
            Geometry::Line { start, end } => Geometry::Line { start: nudge(start, rng, m), end: nudge(end, rng, m) },
            Geometry::Circle { center, radius } => {
                Geometry::Circle { center: nudge(center, rng, m), radius: (radius + rng.range(-m, m)).max(radius * 0.3) }
            }
            Geometry::Arc { center, radius, start, end } => {
                let c = nudge(center, rng, m);
                let r = (radius + rng.range(-m, m)).max(radius * 0.3);
                let shift = m / radius;
                let a0 = (start[1] - center[1]).atan2(start[0] - center[0]) + rng.range(-shift, shift);
                let a1 = (end[1] - center[1]).atan2(end[0] - center[0]) + rng.range(-shift, shift);
                Geometry::Arc { center: c, radius: r, start: polar(c, r, a0), end: polar(c, r, a1) }
            }
        };
    }
    out
}

/// Builds one case from its category and seed.
pub fn generate(category: Category, seed: u64) -> Case {
    generate_sized(category, seed, 0)
}

/// As `generate`, with a large sketch built up to about `entities` entities when that isn't zero.
pub fn generate_sized(category: Category, seed: u64, entities: usize) -> Case {
    let mut rng = Rng::new(seed.wrapping_mul(1009).wrapping_add(category as u64));
    let magnitude = [0.0, 1e-3, 1e-2, 0.1, 0.5, 1.5, 3.0, 5.0][rng.int(0, 7) as usize];
    let mut gen = Gen { s: Sketch::new(), rng: &mut rng };
    let mut expect = Expect::Solvable;
    match category {
        Category::Rectilinear => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            let lines = gen.rectilinear(c);
            if gen.rng.chance(0.5) {
                gen.join(lines[0].start(), ORIGIN.whole());
                // Move the truth so that this holds: see the shift below.
            }
        }
        Category::Polygons => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.polygon(c);
        }
        Category::Rectangles => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.rectangle(c);
        }
        Category::Slots => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.slot(c);
        }
        Category::RoundedCorners => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.rounded(c);
        }
        Category::Circles => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.circles(c);
        }
        Category::Symmetry => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.symmetry(c);
        }
        Category::Distances => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            gen.distances(c);
        }
        Category::Mixed => {
            let blocks = gen.rng.int(2, 5);
            let mut anchors: Vec<EntityId> = Vec::new();
            for b in 0..blocks {
                let c = [b as f64 * 120.0, gen.rng.range(-30.0, 30.0)];
                let made = gen.block(c);
                if let Some(&first) = made.iter().find(|&&e| matches!(gen.s.entity(e).map(|x| x.geometry), Some(Geometry::Line { .. }))) {
                    anchors.push(first);
                }
            }
            // Dimensions between blocks.
            for w in anchors.windows(2) {
                let (a, b) = (w[0], w[1]);
                let (p, q) = (point_of(&gen.s, a.start()).unwrap(), point_of(&gen.s, b.start()).unwrap());
                gen.on(Rule::HorizontalDistance { a: a.start(), b: b.start(), value: (q[0] - p[0]).into() });
                if gen.rng.chance(0.6) {
                    gen.on(Rule::VerticalDistance { a: a.start(), b: b.start(), value: (q[1] - p[1]).into() });
                }
            }
        }
        Category::Large => {
            // Separate blocks joined by dimensions into one connected sketch.
            let blocks = if entities > 0 { i64::MAX } else { gen.rng.int(15, 50) };
            let mut last: Option<EntityId> = None;
            for b in 0..blocks {
                if entities > 0 && gen.s.entities.len() >= entities {
                    break;
                }
                let c = [(b % 8) as f64 * 130.0, (b / 8) as f64 * 130.0];
                let made = gen.block(c);
                let line = made.iter().copied().find(|&e| matches!(gen.s.entity(e).map(|x| x.geometry), Some(Geometry::Line { .. })));
                if let (Some(a), Some(l)) = (last, line) {
                    let (p, q) = (point_of(&gen.s, a.start()).unwrap(), point_of(&gen.s, l.start()).unwrap());
                    gen.on(Rule::HorizontalDistance { a: a.start(), b: l.start(), value: (q[0] - p[0]).into() });
                }
                last = line.or(last);
            }
        }
        Category::Redundant => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            let added = inject_redundant(&mut gen, c);
            expect = Expect::Redundant(added);
        }
        Category::Conflict => {
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            let added = inject_conflict(&mut gen, c);
            expect = Expect::Conflict(added);
        }
        Category::Impossible => {
            // Three sides that can't make a triangle, the smallest case with no contradiction
            // visible in the equations' linear algebra at a generic point.
            let c = [gen.rng.range(-20.0, 20.0), gen.rng.range(-20.0, 20.0)];
            let (a, b) = (gen.rng.range(5.0, 20.0), gen.rng.range(5.0, 20.0));
            let points = [c, [c[0] + a, c[1]], [c[0] + a * 0.7, c[1] + b]];
            let lines = gen.chain(&points);
            let long = (a + b) * gen.rng.range(1.2, 3.0);
            gen.on(Rule::Distance { a: lines[0].whole(), b: None, value: a.into() });
            gen.on(Rule::Distance { a: lines[1].whole(), b: None, value: b.into() });
            gen.on(Rule::Distance { a: lines[2].whole(), b: None, value: long.into() });
            expect = Expect::Impossible;
        }
    }
    let mut truth = gen.s;
    // A rectilinear outline joined to the origin is only true once it is moved there.
    if category == Category::Rectilinear {
        let joined = truth.constraints.iter().find_map(|c| match c.rule {
            Rule::Coincident { a, b } if b.entity == ORIGIN => Some(a),
            _ => None,
        });
        if let Some(a) = joined {
            let p = point_of(&truth, a).unwrap();
            for e in &mut truth.entities {
                if let Geometry::Line { start, end } = e.geometry {
                    e.geometry = Geometry::Line { start: sub(start, p), end: sub(end, p) };
                }
            }
        }
    }
    let start = perturb(&truth, &mut rng, magnitude);
    Case { category, seed, truth, start, expect }
}

fn inject_redundant(gen: &mut Gen<'_>, c: P2) -> Vec<ConstraintId> {
    let which = gen.rng.int(0, 4);
    let mut added = Vec::new();
    match which {
        0 => {
            // Opposite sides of an axis-aligned rectangle are already parallel.
            let (w, h) = (gen.rng.range(10.0, 40.0), gen.rng.range(10.0, 40.0));
            let p = [c, [c[0] + w, c[1]], [c[0] + w, c[1] + h], [c[0], c[1] + h]];
            let l = gen.chain(&p);
            gen.on(Rule::Horizontal { a: l[0].whole(), b: None });
            gen.on(Rule::Vertical { a: l[1].whole(), b: None });
            gen.on(Rule::Horizontal { a: l[2].whole(), b: None });
            gen.on(Rule::Vertical { a: l[3].whole(), b: None });
            added.push(gen.on(Rule::Parallel { a: l[0], b: l[2] }));
        }
        1 => {
            // A level side and an upright one are already square to each other.
            let (w, h) = (gen.rng.range(10.0, 40.0), gen.rng.range(10.0, 40.0));
            let p = [c, [c[0] + w, c[1]], [c[0] + w, c[1] + h], [c[0], c[1] + h]];
            let l = gen.chain(&p);
            gen.on(Rule::Horizontal { a: l[0].whole(), b: None });
            gen.on(Rule::Vertical { a: l[1].whole(), b: None });
            gen.on(Rule::Horizontal { a: l[2].whole(), b: None });
            added.push(gen.on(Rule::Perpendicular { a: l[0], b: l[1] }));
        }
        2 => {
            // The same dimension twice.
            let (w, h) = (gen.rng.range(10.0, 40.0), gen.rng.range(10.0, 40.0));
            let l = gen.s.line(c, [c[0] + w, c[1] + h]);
            let length = w.hypot(h);
            gen.on(Rule::Distance { a: l.whole(), b: None, value: length.into() });
            added.push(gen.on(Rule::Distance { a: l.start(), b: Some(l.end()), value: length.into() }));
        }
        3 => {
            // Three points coincident in a cycle.
            let a = gen.s.point(c[0], c[1]);
            let b = gen.s.point(c[0], c[1]);
            let d = gen.s.point(c[0], c[1]);
            gen.join(a.whole(), b.whole());
            gen.join(b.whole(), d.whole());
            added.push(gen.on(Rule::Coincident { a: d.whole(), b: a.whole() }));
        }
        _ => {
            // Equal radii around a triangle.
            let (r, far) = (gen.rng.range(3.0, 9.0), 30.0);
            let a = gen.s.circle(c, r);
            let b = gen.s.circle([c[0] + far, c[1]], r);
            let d = gen.s.circle([c[0], c[1] + far], r);
            gen.on(Rule::Equal { a, b });
            gen.on(Rule::Equal { a: b, b: d });
            added.push(gen.on(Rule::Equal { a: d, b: a }));
        }
    }
    added
}

fn inject_conflict(gen: &mut Gen<'_>, c: P2) -> Vec<ConstraintId> {
    let which = gen.rng.int(0, 4);
    let mut pair = Vec::new();
    match which {
        0 => {
            // Two lengths for one line.
            let (w, h) = (gen.rng.range(10.0, 40.0), gen.rng.range(10.0, 40.0));
            let l = gen.s.line(c, [c[0] + w, c[1] + h]);
            let length = w.hypot(h);
            pair.push(gen.on(Rule::Distance { a: l.whole(), b: None, value: length.into() }));
            let other = length * gen.rng.range(1.2, 2.0);
            pair.push(gen.on(Rule::Distance { a: l.start(), b: Some(l.end()), value: other.into() }));
        }
        1 => {
            // Two places for one point.
            let a = gen.s.point(c[0], c[1]);
            pair.push(gen.on(Rule::Fix { what: a.whole(), at: c.to_vec() }));
            pair.push(gen.on(Rule::Fix { what: a.whole(), at: vec![c[0] + 5.0, c[1]] }));
        }
        2 => {
            // A level line given a slope.
            let l = gen.s.line(c, [c[0] + 20.0, c[1]]);
            pair.push(gen.on(Rule::Horizontal { a: l.whole(), b: None }));
            let slope = gen.rng.range(20.0, 70.0);
            pair.push(gen.on(Rule::Angle { a: l, b: None, value: slope.into() }));
        }
        3 => {
            // Two radii for one circle.
            let e = gen.s.circle(c, 10.0);
            pair.push(gen.on(Rule::Radius { of: e, value: 10.0.into() }));
            let other = gen.rng.range(25.0, 60.0);
            pair.push(gen.on(Rule::Diameter { of: e, value: other.into() }));
        }
        _ => {
            // A rectangle whose diagonal doesn't fit its sides.
            let (w, h) = (gen.rng.range(10.0, 40.0), gen.rng.range(10.0, 40.0));
            let p = [c, [c[0] + w, c[1]], [c[0] + w, c[1] + h], [c[0], c[1] + h]];
            let l = gen.chain(&p);
            gen.on(Rule::Horizontal { a: l[0].whole(), b: None });
            gen.on(Rule::Vertical { a: l[1].whole(), b: None });
            gen.on(Rule::Horizontal { a: l[2].whole(), b: None });
            gen.on(Rule::Vertical { a: l[3].whole(), b: None });
            pair.push(gen.on(Rule::Distance { a: l[0].whole(), b: None, value: w.into() }));
            pair.push(gen.on(Rule::Distance { a: l[1].whole(), b: None, value: h.into() }));
            let diagonal = w.hypot(h) * gen.rng.range(1.2, 2.0);
            pair.push(gen.on(Rule::Distance { a: l[0].start(), b: Some(l[1].end()), value: diagonal.into() }));
        }
    }
    pair
}

/// The result of one case: refusals and wrong answers, in words.
#[derive(Clone, Debug, Default)]
pub struct Outcome {
    pub refusals: Vec<String>,
    pub wrong: Vec<String>,
}

/// How far from a constraint a solved sketch may be, as measured independently.
pub const CHECK_TOLERANCE: f64 = 1e-7;

/// Whether removing any one of the named constraints makes the sketch solvable.
fn is_smallest(start: &Sketch, named: &[ConstraintId]) -> bool {
    named.iter().all(|&id| {
        let mut s = start.clone();
        s.remove_constraint(id).is_ok() && s.solve().is_ok()
    })
}

pub fn check_case(case: &Case) -> Outcome {
    let mut out = Outcome::default();
    let problems = |list: Vec<String>| list.join("; ");
    if !matches!(case.expect, Expect::Impossible | Expect::Conflict(_)) {
        let bad = violations(&case.truth, 1e-9);
        if !bad.is_empty() {
            out.wrong.push(format!("the generator's own truth is off: {}", problems(bad)));
            return out;
        }
    }
    let result = case.start.solve();
    let before = case.start.clone();
    match (&result, &case.expect) {
        (Ok(s), Expect::Conflict(_) | Expect::Impossible) => {
            let bad = violations(&s.sketch, CHECK_TOLERANCE);
            out.wrong.push(format!("solved a sketch with no solution, leaving: {}", problems(bad)));
        }
        (Ok(s), expect) => {
            let bad = violations(&s.sketch, CHECK_TOLERANCE);
            if !bad.is_empty() {
                out.wrong.push(format!("said solved but: {}", problems(bad)));
            }
            if let Expect::Redundant(ids) = expect {
                for id in ids {
                    let named = s.analysis.redundant.iter().any(|d| d.constraint == *id || d.with.contains(id));
                    if !named {
                        out.wrong.push(format!("c{} is implied by others but wasn't named", id.0));
                    }
                }
            }
            // Solving what's solved must leave it alone.
            match s.sketch.solve() {
                Ok(again) => {
                    if again.analysis.dof != s.analysis.dof {
                        out.wrong.push(format!("solving twice changed the freedom from {} to {}", s.analysis.dof, again.analysis.dof));
                    }
                    let moved = largest_move(&s.sketch, &again.sketch);
                    if moved > 1e-7 {
                        out.wrong.push(format!("solving again moved geometry by {moved:.3e}"));
                    }
                }
                Err(e) => out.wrong.push(format!("the solved sketch doesn't solve again: {}", e.message)),
            }
        }
        (Err(e), Expect::Conflict(ids)) => {
            if e.kind != ErrorKind::Conflict && e.kind != ErrorKind::Collapsed {
                out.wrong.push(format!("reported {:?}, not a conflict: {}", e.kind, e.message));
            }
            for id in ids {
                if !e.constraints.contains(id) {
                    out.wrong.push(format!("the conflict doesn't name c{}: {}", id.0, e.message));
                }
            }
            if !is_smallest(&case.start, &e.constraints) {
                out.wrong.push(format!("removing a named constraint doesn't resolve it: {}", e.message));
            }
        }
        (Err(e), Expect::Impossible) => {
            if e.constraints.is_empty() {
                out.wrong.push(format!("no constraints named: {}", e.message));
            }
        }
        (Err(e), _) => out.refusals.push(describe_failure(e)),
    }
    if before != case.start {
        out.wrong.push("solving changed the sketch it was given".to_string());
    }
    out
}

fn describe_failure(e: &SketchError) -> String {
    format!("{:?}: {}", e.kind, e.message)
}

/// The largest difference in any coordinate between two sketches with the same entities.
pub fn largest_move(a: &Sketch, b: &Sketch) -> f64 {
    let flat = |g: &Geometry| -> Vec<f64> {
        match *g {
            Geometry::Point { at } => at.to_vec(),
            Geometry::Line { start, end } => vec![start[0], start[1], end[0], end[1]],
            Geometry::Circle { center, radius } => vec![center[0], center[1], radius],
            Geometry::Arc { center, radius, start, end } => vec![center[0], center[1], radius, start[0], start[1], end[0], end[1]],
        }
    };
    let mut largest = 0.0f64;
    for (x, y) in a.entities.iter().zip(&b.entities) {
        for (p, q) in flat(&x.geometry).iter().zip(flat(&y.geometry)) {
            largest = largest.max((p - q).abs());
        }
    }
    largest
}
