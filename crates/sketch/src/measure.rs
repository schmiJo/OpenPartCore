//! What a dimension measures on the geometry as it stands, worked out directly from the shapes. The
//! solver's equations don't come into it, so this also checks them from the outside.

use crate::model::*;

fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn across(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    (dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / dx.hypot(dy)
}

/// The middle of a line.
fn middle(a: P2, b: P2) -> P2 {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

/// Wraps to (-180, 180] degrees.
fn signed_degrees(radians: f64) -> f64 {
    let d = radians.to_degrees().rem_euclid(360.0);
    if d > 180.0 {
        d - 360.0
    } else {
        d
    }
}

impl Sketch {
    pub(crate) fn line_ends(&self, id: EntityId) -> Option<(P2, P2)> {
        match id {
            X_AXIS => Some(([0.0, 0.0], [1.0, 0.0])),
            Y_AXIS => Some(([0.0, 0.0], [0.0, 1.0])),
            _ => match self.entity(id)?.geometry {
                Geometry::Line { start, end } => Some((start, end)),
                _ => None,
            },
        }
    }

    pub(crate) fn circle_of(&self, id: EntityId) -> Option<(P2, f64)> {
        match self.entity(id)?.geometry {
            Geometry::Circle { center, radius } | Geometry::Arc { center, radius, .. } => Some((center, radius)),
            _ => None,
        }
    }

    pub(crate) fn point_at(&self, r: Ref) -> Option<P2> {
        match (r.entity, r.part) {
            (ORIGIN, Part::Whole) => return Some([0.0, 0.0]),
            (X_AXIS | Y_AXIS, Part::Start) => return Some([0.0, 0.0]),
            (X_AXIS, Part::End) => return Some([1.0, 0.0]),
            (Y_AXIS, Part::End) => return Some([0.0, 1.0]),
            _ => {}
        }
        match (self.entity(r.entity)?.geometry, r.part) {
            (Geometry::Point { at }, Part::Whole) => Some(at),
            (Geometry::Line { start, .. } | Geometry::Arc { start, .. }, Part::Start) => Some(start),
            (Geometry::Line { end, .. } | Geometry::Arc { end, .. }, Part::End) => Some(end),
            (Geometry::Circle { center, .. } | Geometry::Arc { center, .. }, Part::Center) => Some(center),
            _ => None,
        }
    }

    /// The gap between two things, as `Rule::Distance` means it.
    fn gap(&self, a: Ref, b: Ref) -> Option<f64> {
        let whole_line = |r: Ref| if r.part == Part::Whole { self.line_ends(r.entity) } else { None };
        let whole_circle = |r: Ref| if r.part == Part::Whole { self.circle_of(r.entity) } else { None };
        let point = |r: Ref| self.point_at(r);
        if let (Some(p), Some(q)) = (point(a), point(b)) {
            return Some(dist(p, q));
        }
        for (x, y) in [(a, b), (b, a)] {
            if let (Some(p), Some((u, v))) = (point(x), whole_line(y)) {
                return Some(across(p, u, v).abs());
            }
            if let (Some(p), Some((c, r))) = (point(x), whole_circle(y)) {
                return Some((dist(p, c) - r).abs());
            }
            if let (Some((u, v)), Some((c, r))) = (whole_line(x), whole_circle(y)) {
                return Some((across(c, u, v).abs() - r).abs());
            }
        }
        if let (Some((c0, r0)), Some((c1, r1))) = (whole_circle(a), whole_circle(b)) {
            let d = dist(c0, c1);
            // Crossing circles touch, so there's no gap to report.
            return Some(if d >= r0 + r1 {
                d - r0 - r1
            } else if d <= (r0 - r1).abs() {
                (r0 - r1).abs() - d
            } else {
                0.0
            });
        }
        if let (Some((a0, a1)), Some((b0, b1))) = (whole_line(a), whole_line(b)) {
            let (from, to) = if a.entity.is_reference() { ((b0, b1), (a0, a1)) } else { ((a0, a1), (b0, b1)) };
            return Some(across(middle(from.0, from.1), to.0, to.1).abs());
        }
        None
    }

    /// What a dimension's rule comes to on the geometry as it stands, in millimetres or degrees.
    pub fn measure_rule(&self, rule: &Rule) -> Result<f64, String> {
        let unfit = || format!("a {} dimension doesn't fit the geometry it refers to", rule.kind());
        let value = match rule {
            Rule::Distance { a, b: None, .. } => self.line_ends(a.entity).map(|(p, q)| dist(p, q)),
            Rule::Distance { a, b: Some(b), .. } => self.gap(*a, *b),
            Rule::HorizontalDistance { a, b, .. } => self.point_at(*a).zip(self.point_at(*b)).map(|(p, q)| q[0] - p[0]),
            Rule::VerticalDistance { a, b, .. } => self.point_at(*a).zip(self.point_at(*b)).map(|(p, q)| q[1] - p[1]),
            Rule::Radius { of, .. } => self.circle_of(*of).map(|c| c.1),
            Rule::Diameter { of, .. } => self.circle_of(*of).map(|c| c.1 * 2.0),
            Rule::Angle { a, b: Some(b), .. } => self.line_ends(*a).zip(self.line_ends(*b)).map(|((a0, a1), (b0, b1))| {
                let (u, v) = ([a1[0] - a0[0], a1[1] - a0[1]], [b1[0] - b0[0], b1[1] - b0[1]]);
                signed_degrees((u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]))
            }),
            Rule::Angle { a, b: None, .. } => match self.entity(*a).map(|e| e.geometry) {
                Some(Geometry::Arc { center, start, end, .. }) => {
                    let (u, v) = ([start[0] - center[0], start[1] - center[1]], [end[0] - center[0], end[1] - center[1]]);
                    Some((u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).to_degrees().rem_euclid(360.0))
                }
                _ => self.line_ends(*a).map(|(p, q)| signed_degrees((q[1] - p[1]).atan2(q[0] - p[0]))),
            },
            other => return Err(format!("a {} constraint isn't a dimension", other.kind())),
        };
        value.ok_or_else(unfit)
    }

    /// What a dimension comes to on the geometry as it stands, whether or not it drives anything.
    pub fn measure(&self, id: ConstraintId) -> Result<f64, String> {
        let c = self.constraint(id).ok_or_else(|| format!("there is no constraint c{}", id.0))?;
        self.measure_rule(&c.rule).map_err(|why| format!("c{}: {why}", id.0))
    }

    /// Every reference dimension and what it measures now.
    pub fn reference_values(&self) -> Vec<(ConstraintId, f64)> {
        self.constraints.iter().filter(|c| c.reference).filter_map(|c| self.measure_rule(&c.rule).ok().map(|v| (c.id, v))).collect()
    }
}
