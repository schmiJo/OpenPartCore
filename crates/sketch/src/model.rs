//! What a sketch is: entities, constraints and dimensions, all plain data.
//!
//! Ids are permanent and never reused, so a constraint or a kernel face made from an entity
//! can refer to it through any edit.

use crate::expr::Expr;

/// An entity's permanent id. Sketches hand out ids from 1. The reference geometry has
/// reserved ids and names of its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntityId(pub u32);

/// A constraint's permanent id, written `c12` in messages.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ConstraintId(pub u32);

/// The sketch origin, a fixed point.
pub const ORIGIN: EntityId = EntityId(0);
/// The X axis, a fixed line from the origin along +x.
pub const X_AXIS: EntityId = EntityId(u32::MAX);
/// The Y axis, a fixed line from the origin along +y.
pub const Y_AXIS: EntityId = EntityId(u32::MAX - 1);

impl EntityId {
    pub fn is_reference(self) -> bool {
        self == ORIGIN || self == X_AXIS || self == Y_AXIS
    }

    pub fn whole(self) -> Ref {
        Ref { entity: self, part: Part::Whole }
    }

    pub fn start(self) -> Ref {
        Ref { entity: self, part: Part::Start }
    }

    pub fn end(self) -> Ref {
        Ref { entity: self, part: Part::End }
    }

    pub fn center(self) -> Ref {
        Ref { entity: self, part: Part::Center }
    }
}

/// Which part of an entity a constraint refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Part {
    /// The whole entity: a point itself, or an edge.
    Whole,
    Start,
    End,
    Center,
    /// A circle's or arc's radius. Only used when reporting what is still free.
    Radius,
}

/// A reference to an entity or a part of one, written `7`, `7.start`, `7.end`, `7.center`,
/// `origin`, `x_axis` or `y_axis`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Ref {
    pub entity: EntityId,
    pub part: Part,
}

impl Ref {
    pub fn name(self) -> String {
        let base = match self.entity {
            ORIGIN => "origin".to_string(),
            X_AXIS => "x_axis".to_string(),
            Y_AXIS => "y_axis".to_string(),
            EntityId(n) => n.to_string(),
        };
        match self.part {
            Part::Whole => base,
            Part::Start => format!("{base}.start"),
            Part::End => format!("{base}.end"),
            Part::Center => format!("{base}.center"),
            Part::Radius => format!("{base}.radius"),
        }
    }

    pub fn parse(text: &str) -> Result<Ref, String> {
        let bad = || format!("\"{text}\" is not a reference; they look like 7, 7.start, 7.end, 7.center, origin, x_axis or y_axis");
        let (base, part) = match text.split_once('.') {
            None => (text, Part::Whole),
            Some((b, "start")) => (b, Part::Start),
            Some((b, "end")) => (b, Part::End),
            Some((b, "center")) => (b, Part::Center),
            Some(_) => return Err(bad()),
        };
        let entity = match base {
            "origin" => ORIGIN,
            "x_axis" => X_AXIS,
            "y_axis" => Y_AXIS,
            n => {
                let id: u32 = n.parse().map_err(|_| bad())?;
                if id == 0 || id >= u32::MAX - 1 {
                    return Err(bad());
                }
                EntityId(id)
            }
        };
        Ok(Ref { entity, part })
    }
}

impl From<EntityId> for Ref {
    fn from(e: EntityId) -> Ref {
        e.whole()
    }
}

pub type P2 = [f64; 2];

/// The shape of an entity.
///
/// Every number here is a length, which keeps the solver's "move as little as possible"
/// meaningful. A later ellipse (center, major axis end, minor radius) or spline (control
/// points) fits the same pattern: add a variant here and its variable layout in `system.rs`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Geometry {
    Point { at: P2 },
    Line { start: P2, end: P2 },
    Circle { center: P2, radius: f64 },
    /// Runs counter-clockwise from `start` to `end`.
    Arc { center: P2, radius: f64, start: P2, end: P2 },
}

impl Geometry {
    pub fn kind(&self) -> &'static str {
        match self {
            Geometry::Point { .. } => "point",
            Geometry::Line { .. } => "line",
            Geometry::Circle { .. } => "circle",
            Geometry::Arc { .. } => "arc",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Entity {
    pub id: EntityId,
    pub geometry: Geometry,
    /// Construction geometry helps constrain a sketch but is left out of its regions.
    pub construction: bool,
}

/// What a constraint requires. Dimensions hold expressions, which may name parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum Rule {
    /// Two points in one place, or a point on a line, circle or arc (`b` is the whole edge).
    /// A line counts as infinite here and an arc as its full circle.
    Coincident { a: Ref, b: Ref },
    /// A line (`b` absent), or the two points `a` and `b`, level with each other.
    Horizontal { a: Ref, b: Option<Ref> },
    Vertical { a: Ref, b: Option<Ref> },
    Parallel { a: EntityId, b: EntityId },
    Perpendicular { a: EntityId, b: EntityId },
    /// A line and a circle or arc, or two circles or arcs. If the two share an endpoint
    /// through coincident constraints, they are tangent at that point.
    Tangent { a: EntityId, b: EntityId },
    /// Two lines of equal length, or two circles or arcs of equal radius.
    Equal { a: EntityId, b: EntityId },
    /// A point at the middle of a line.
    Midpoint { point: Ref, line: EntityId },
    /// Two circles or arcs with one center.
    Concentric { a: EntityId, b: EntityId },
    /// Points mirrored about a line (the whole line) or a point.
    Symmetric { a: Ref, b: Ref, about: Ref },
    /// A line through a circle's or arc's center, or two circles or arcs crossing at right
    /// angles.
    Normal { a: EntityId, b: EntityId },
    /// Holds a point or a whole entity in place. `at` holds the values to keep, in order:
    /// x, y for a point; both ends for a line; x, y, radius for a circle; those then the
    /// start and end points for an arc.
    Fix { what: Ref, at: Vec<f64> },
    /// A line's length (`b` absent), or between any two of a point, a line and a circle or arc.
    /// To a circle or arc, it's the gap to the curve, on whichever side things are now, with an arc
    /// counting as its full circle. Between two lines, it's from the middle of `a` to `b`, which is
    /// the distance between them when they're parallel. It doesn't make them parallel.
    Distance { a: Ref, b: Option<Ref>, value: Expr },
    /// `b.x - a.x`, which may be negative.
    HorizontalDistance { a: Ref, b: Ref, value: Expr },
    /// `b.y - a.y`, which may be negative.
    VerticalDistance { a: Ref, b: Ref, value: Expr },
    Radius { of: EntityId, value: Expr },
    Diameter { of: EntityId, value: Expr },
    /// In degrees, counter-clockwise from line `a`'s direction to line `b`'s. With `b` absent:
    /// from +x to line `a`, or an arc's span.
    Angle { a: EntityId, b: Option<EntityId>, value: Expr },
}

impl Rule {
    pub fn kind(&self) -> &'static str {
        match self {
            Rule::Coincident { .. } => "coincident",
            Rule::Horizontal { .. } => "horizontal",
            Rule::Vertical { .. } => "vertical",
            Rule::Parallel { .. } => "parallel",
            Rule::Perpendicular { .. } => "perpendicular",
            Rule::Tangent { .. } => "tangent",
            Rule::Equal { .. } => "equal",
            Rule::Midpoint { .. } => "midpoint",
            Rule::Concentric { .. } => "concentric",
            Rule::Symmetric { .. } => "symmetric",
            Rule::Normal { .. } => "normal",
            Rule::Fix { .. } => "fix",
            Rule::Distance { .. } => "distance",
            Rule::HorizontalDistance { .. } => "horizontal_distance",
            Rule::VerticalDistance { .. } => "vertical_distance",
            Rule::Radius { .. } => "radius",
            Rule::Diameter { .. } => "diameter",
            Rule::Angle { .. } => "angle",
        }
    }

    /// Every entity this rule refers to.
    pub fn entities(&self) -> Vec<EntityId> {
        match self {
            Rule::Coincident { a, b } => vec![a.entity, b.entity],
            Rule::Horizontal { a, b } | Rule::Vertical { a, b } => {
                let mut v = vec![a.entity];
                v.extend(b.map(|r| r.entity));
                v
            }
            Rule::Parallel { a, b }
            | Rule::Perpendicular { a, b }
            | Rule::Tangent { a, b }
            | Rule::Equal { a, b }
            | Rule::Concentric { a, b }
            | Rule::Normal { a, b } => vec![*a, *b],
            Rule::Midpoint { point, line } => vec![point.entity, *line],
            Rule::Symmetric { a, b, about } => vec![a.entity, b.entity, about.entity],
            Rule::Fix { what, .. } => vec![what.entity],
            Rule::Distance { a, b, .. } => {
                let mut v = vec![a.entity];
                v.extend(b.map(|r| r.entity));
                v
            }
            Rule::HorizontalDistance { a, b, .. } | Rule::VerticalDistance { a, b, .. } => vec![a.entity, b.entity],
            Rule::Radius { of, .. } | Rule::Diameter { of, .. } => vec![*of],
            Rule::Angle { a, b, .. } => {
                let mut v = vec![*a];
                v.extend(*b);
                v
            }
        }
    }

    /// The expression a dimension is driven by.
    pub fn value(&self) -> Option<&Expr> {
        match self {
            Rule::Distance { value, .. }
            | Rule::HorizontalDistance { value, .. }
            | Rule::VerticalDistance { value, .. }
            | Rule::Radius { value, .. }
            | Rule::Diameter { value, .. }
            | Rule::Angle { value, .. } => Some(value),
            _ => None,
        }
    }

    pub(crate) fn value_mut(&mut self) -> Option<&mut Expr> {
        match self {
            Rule::Distance { value, .. }
            | Rule::HorizontalDistance { value, .. }
            | Rule::VerticalDistance { value, .. }
            | Rule::Radius { value, .. }
            | Rule::Diameter { value, .. }
            | Rule::Angle { value, .. } => Some(value),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Constraint {
    pub id: ConstraintId,
    pub rule: Rule,
    /// A reference dimension reports what the geometry measures and doesn't drive it. Solving ignores
    /// it, then writes the measured value into it.
    pub reference: bool,
}

/// A sketch: plain data, and a value. Solving returns a new sketch and never changes this one,
/// so a failure can't leave anything half done.
#[derive(Clone, Debug, PartialEq)]
pub struct Sketch {
    pub entities: Vec<Entity>,
    pub constraints: Vec<Constraint>,
    pub(crate) next_entity: u32,
    pub(crate) next_constraint: u32,
    /// Regions chosen by id. Empty means the default: see `Sketch::regions`.
    pub(crate) picked_regions: Vec<crate::regions::RegionId>,
}

impl Default for Sketch {
    fn default() -> Sketch {
        Sketch::new()
    }
}

impl Sketch {
    pub fn new() -> Sketch {
        Sketch { entities: Vec::new(), constraints: Vec::new(), next_entity: 1, next_constraint: 1, picked_regions: Vec::new() }
    }

    pub fn entity(&self, id: EntityId) -> Option<&Entity> {
        self.entities.iter().find(|e| e.id == id)
    }

    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint> {
        self.constraints.iter().find(|c| c.id == id)
    }

    pub fn add(&mut self, geometry: Geometry) -> EntityId {
        let id = EntityId(self.next_entity);
        self.next_entity += 1;
        self.entities.push(Entity { id, geometry, construction: false });
        id
    }

    pub fn point(&mut self, x: f64, y: f64) -> EntityId {
        self.add(Geometry::Point { at: [x, y] })
    }

    pub fn line(&mut self, start: P2, end: P2) -> EntityId {
        self.add(Geometry::Line { start, end })
    }

    pub fn circle(&mut self, center: P2, radius: f64) -> EntityId {
        self.add(Geometry::Circle { center, radius })
    }

    /// An arc from `degrees0` to `degrees1`, counter-clockwise.
    pub fn arc(&mut self, center: P2, radius: f64, degrees0: f64, degrees1: f64) -> EntityId {
        let at = |d: f64| {
            let (s, c) = d.to_radians().sin_cos();
            [center[0] + radius * c, center[1] + radius * s]
        };
        self.add(Geometry::Arc { center, radius, start: at(degrees0), end: at(degrees1) })
    }

    /// Marks an entity as construction geometry, or not.
    pub fn set_construction(&mut self, id: EntityId, construction: bool) -> Result<(), String> {
        match self.entities.iter_mut().find(|e| e.id == id) {
            Some(e) => {
                e.construction = construction;
                Ok(())
            }
            None => Err(format!("there is no entity {}", id.0)),
        }
    }

    pub fn constrain(&mut self, rule: Rule) -> ConstraintId {
        let id = ConstraintId(self.next_constraint);
        self.next_constraint += 1;
        self.constraints.push(Constraint { id, rule, reference: false });
        id
    }

    /// Adds a reference dimension: one that reports what the geometry measures and doesn't drive it.
    /// The rule's value is replaced with the measurement, so any number will do there.
    pub fn add_reference(&mut self, rule: Rule) -> Result<ConstraintId, String> {
        if rule.value().is_none() {
            return Err(format!("a {} constraint has no value, so it can't be a reference dimension", rule.kind()));
        }
        let id = self.constrain(rule);
        self.set_reference(id, true)?;
        Ok(id)
    }

    /// Makes a dimension a reference, or back into one that drives. A reference made driving starts
    /// from the value it last measured.
    pub fn set_reference(&mut self, id: ConstraintId, reference: bool) -> Result<(), String> {
        let c = self.constraints.iter_mut().find(|c| c.id == id).ok_or_else(|| format!("there is no constraint c{}", id.0))?;
        if c.rule.value().is_none() {
            return Err(format!("c{} is a {} constraint, which has no value", id.0, c.rule.kind()));
        }
        c.reference = reference;
        if reference {
            self.refresh_references();
        }
        Ok(())
    }

    /// Writes what the geometry measures into every reference dimension. One that doesn't fit its
    /// geometry keeps its old value; solving reports the same problem for a driving one.
    pub(crate) fn refresh_references(&mut self) {
        let measured: Vec<(usize, f64)> = self
            .constraints
            .iter()
            .enumerate()
            .filter(|(_, c)| c.reference)
            .filter_map(|(i, c)| self.measure_rule(&c.rule).ok().map(|v| (i, v)))
            .collect();
        for (i, v) in measured {
            if let Some(value) = self.constraints[i].rule.value_mut() {
                *value = v.into();
            }
        }
    }

    /// Fixes a point or a whole entity where it is now.
    pub fn fix(&mut self, what: impl Into<Ref>) -> Result<ConstraintId, String> {
        let what = what.into();
        let at = self.current_values(what)?;
        Ok(self.constrain(Rule::Fix { what, at }))
    }

    fn current_values(&self, what: Ref) -> Result<Vec<f64>, String> {
        let e = self.entity(what.entity).ok_or_else(|| format!("there is no entity {}", what.name()))?;
        let bad = || format!("{} is not something that can be fixed", what.name());
        Ok(match (e.geometry, what.part) {
            (Geometry::Point { at }, Part::Whole) => at.to_vec(),
            (Geometry::Line { start, .. }, Part::Start) => start.to_vec(),
            (Geometry::Line { end, .. }, Part::End) => end.to_vec(),
            (Geometry::Line { start, end }, Part::Whole) => vec![start[0], start[1], end[0], end[1]],
            (Geometry::Circle { center, .. }, Part::Center) => center.to_vec(),
            (Geometry::Circle { center, radius }, Part::Whole) => vec![center[0], center[1], radius],
            (Geometry::Arc { center, .. }, Part::Center) => center.to_vec(),
            (Geometry::Arc { start, .. }, Part::Start) => start.to_vec(),
            (Geometry::Arc { end, .. }, Part::End) => end.to_vec(),
            (Geometry::Arc { center, radius, start, end }, Part::Whole) => {
                vec![center[0], center[1], radius, start[0], start[1], end[0], end[1]]
            }
            _ => return Err(bad()),
        })
    }

    /// Changes a dimension's value. The value is a number or an expression.
    pub fn set_value(&mut self, id: ConstraintId, value: impl Into<Expr>) -> Result<(), String> {
        let c = self.constraints.iter_mut().find(|c| c.id == id).ok_or_else(|| format!("there is no constraint c{}", id.0))?;
        let kind = c.rule.kind();
        if c.reference {
            return Err(format!("c{} is a reference dimension: it reports the geometry and can't be set", id.0));
        }
        match c.rule.value_mut() {
            Some(v) => {
                *v = value.into();
                Ok(())
            }
            None => Err(format!("c{} is a {kind} constraint, which has no value", id.0)),
        }
    }

    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<(), String> {
        let at = self.constraints.iter().position(|c| c.id == id).ok_or_else(|| format!("there is no constraint c{}", id.0))?;
        self.constraints.remove(at);
        Ok(())
    }

    /// Removes an entity and every constraint that refers to it, returning those constraints.
    pub fn remove_entity(&mut self, id: EntityId) -> Result<Vec<ConstraintId>, String> {
        let at = self.entities.iter().position(|e| e.id == id).ok_or_else(|| format!("there is no entity {}", id.0))?;
        self.entities.remove(at);
        let gone: Vec<ConstraintId> =
            self.constraints.iter().filter(|c| c.rule.entities().contains(&id)).map(|c| c.id).collect();
        self.constraints.retain(|c| !gone.contains(&c.id));
        Ok(gone)
    }

    /// The largest extent of the geometry, used to scale tolerances and restarts.
    pub fn size(&self) -> f64 {
        let mut largest: f64 = 1.0;
        let mut see = |p: P2| largest = largest.max(p[0].abs()).max(p[1].abs());
        for e in &self.entities {
            match e.geometry {
                Geometry::Point { at } => see(at),
                Geometry::Line { start, end } => {
                    see(start);
                    see(end);
                }
                Geometry::Circle { center, radius } => {
                    see(center);
                    see([radius, 0.0]);
                }
                Geometry::Arc { center, radius, start, end } => {
                    see(center);
                    see([radius, 0.0]);
                    see(start);
                    see(end);
                }
            }
        }
        largest
    }
}
