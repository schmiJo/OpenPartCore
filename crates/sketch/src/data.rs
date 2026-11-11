//! Sketches and results as JSON, for files, the saveable history and tools.
//!
//! Readers refuse what they don't understand, in the same way the document format does: a sketch
//! that silently lost a constraint would be worse than an error.

use crate::expr::Expr;
use crate::json::{self, Value};
use crate::model::*;
use crate::spline::SplineShape;
use crate::regions::{Arrangement, Ignored, Loop, Region, RegionId, Seg, SegKind, WhyIgnored};
use crate::solve::*;

pub const SKETCH_FORMAT: &str = "coreshape-sketch";
pub const SKETCH_VERSION: f64 = 1.0;

type R<T> = Result<T, String>;

fn known(v: &Value, allowed: &[&str]) -> R<()> {
    match v {
        Value::Obj(items) => {
            for (k, _) in items {
                if !allowed.contains(&k.as_str()) {
                    return Err(format!("unknown field \"{k}\"; expected one of: {}", allowed.join(", ")));
                }
            }
            Ok(())
        }
        _ => Err(format!("expected an object with fields: {}", allowed.join(", "))),
    }
}

fn field<'a>(v: &'a Value, key: &str, what: &str) -> R<&'a Value> {
    v.get(key).ok_or_else(|| format!("{what} needs a \"{key}\""))
}

fn point(v: &Value) -> R<P2> {
    match v.as_array() {
        Some([a, b]) => match (a.as_f64(), b.as_f64()) {
            (Some(a), Some(b)) => Ok([a, b]),
            _ => Err("a point is written [x, y], with numbers".to_string()),
        },
        _ => Err("a point is written [x, y]".to_string()),
    }
}

fn number(v: &Value, key: &str, what: &str) -> R<f64> {
    field(v, key, what)?.as_f64().ok_or_else(|| format!("\"{key}\" must be a number"))
}

fn p2_value(p: P2) -> Value {
    Value::Arr(vec![Value::Num(p[0]), Value::Num(p[1])])
}

fn reference(v: &Value) -> R<Ref> {
    match v {
        Value::Str(s) => Ref::parse(s),
        Value::Num(n) if n.fract() == 0.0 && *n >= 1.0 && *n < 4e9 => Ok(EntityId(*n as u32).whole()),
        _ => Err("a reference is written like \"7\", \"7.start\" or \"origin\"".to_string()),
    }
}

fn ref_field(v: &Value, key: &str, what: &str) -> R<Ref> {
    reference(field(v, key, what)?)
}

fn opt_ref(v: &Value, key: &str) -> R<Option<Ref>> {
    match v.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(r) => reference(r).map(Some),
    }
}

fn entity_field(v: &Value, key: &str, what: &str) -> R<EntityId> {
    let r = ref_field(v, key, what)?;
    if r.part != Part::Whole {
        return Err(format!("\"{key}\" must name a whole entity, not {}", r.name()));
    }
    Ok(r.entity)
}

fn expr_value(e: &Expr) -> Value {
    match e.as_number() {
        Some(n) => Value::Num(n),
        None => Value::str(e.text()),
    }
}

fn read_expr(v: &Value) -> R<Expr> {
    match v {
        Value::Num(n) => Ok(Expr::number(*n)),
        Value::Str(s) => Expr::parse(s),
        _ => Err("a dimension must be a number or an expression in quotes".to_string()),
    }
}

fn value_field(v: &Value, what: &str) -> R<Expr> {
    read_expr(field(v, "value", what)?)
}

fn r(x: Ref) -> Value {
    Value::str(x.name())
}

fn e(x: EntityId) -> Value {
    Value::str(x.whole().name())
}

/// A constraint's fields as JSON, without its id.
pub fn rule_value(rule: &Rule) -> Vec<(&'static str, Value)> {
    let mut items = vec![("type", Value::str(rule.kind()))];
    match rule {
        Rule::Coincident { a, b } => {
            items.push(("a", self::r(*a)));
            items.push(("b", self::r(*b)));
        }
        Rule::Horizontal { a, b } | Rule::Vertical { a, b } => {
            items.push(("a", self::r(*a)));
            if let Some(b) = b {
                items.push(("b", self::r(*b)));
            }
        }
        Rule::Parallel { a, b }
        | Rule::Perpendicular { a, b }
        | Rule::Tangent { a, b }
        | Rule::Equal { a, b }
        | Rule::Concentric { a, b }
        | Rule::Normal { a, b } => {
            items.push(("a", self::e(*a)));
            items.push(("b", self::e(*b)));
        }
        Rule::Midpoint { point, line } => {
            items.push(("point", self::r(*point)));
            items.push(("line", self::e(*line)));
        }
        Rule::Symmetric { a, b, about } => {
            items.push(("a", self::r(*a)));
            items.push(("b", self::r(*b)));
            items.push(("about", self::r(*about)));
        }
        Rule::Fix { what, at } => {
            items.push(("what", self::r(*what)));
            items.push(("at", Value::Arr(at.iter().map(|v| Value::Num(*v)).collect())));
        }
        Rule::Distance { a, b, value } => {
            items.push(("a", self::r(*a)));
            if let Some(b) = b {
                items.push(("b", self::r(*b)));
            }
            items.push(("value", expr_value(value)));
        }
        Rule::HorizontalDistance { a, b, value } | Rule::VerticalDistance { a, b, value } => {
            items.push(("a", self::r(*a)));
            items.push(("b", self::r(*b)));
            items.push(("value", expr_value(value)));
        }
        Rule::Radius { of, value } | Rule::MajorRadius { of, value } | Rule::MinorRadius { of, value } | Rule::Diameter { of, value } => {
            items.push(("of", self::e(*of)));
            items.push(("value", expr_value(value)));
        }
        Rule::Angle { a, b, value } => {
            items.push(("a", self::e(*a)));
            if let Some(b) = b {
                items.push(("b", self::e(*b)));
            }
            items.push(("value", expr_value(value)));
        }
    }
    items
}

/// Reads a constraint's fields. `extra` lists fields that belong to the caller.
pub fn read_rule(v: &Value, extra: &[&str]) -> R<Rule> {
    let check = |own: &[&str]| -> R<()> {
        let mut all = vec!["type"];
        all.extend_from_slice(own);
        all.extend_from_slice(extra);
        known(v, &all)
    };
    let kind = field(v, "type", "a constraint")?.as_str().unwrap_or("");
    let what = format!("a {kind} constraint");
    let w = what.as_str();
    Ok(match kind {
        "coincident" => {
            check(&["a", "b"])?;
            Rule::Coincident { a: ref_field(v, "a", w)?, b: ref_field(v, "b", w)? }
        }
        "horizontal" | "vertical" => {
            check(&["a", "b"])?;
            let (a, b) = (ref_field(v, "a", w)?, opt_ref(v, "b")?);
            if kind == "horizontal" {
                Rule::Horizontal { a, b }
            } else {
                Rule::Vertical { a, b }
            }
        }
        "parallel" | "perpendicular" | "tangent" | "equal" | "concentric" | "normal" => {
            check(&["a", "b"])?;
            let (a, b) = (entity_field(v, "a", w)?, entity_field(v, "b", w)?);
            match kind {
                "parallel" => Rule::Parallel { a, b },
                "perpendicular" => Rule::Perpendicular { a, b },
                "tangent" => Rule::Tangent { a, b },
                "equal" => Rule::Equal { a, b },
                "concentric" => Rule::Concentric { a, b },
                _ => Rule::Normal { a, b },
            }
        }
        "midpoint" => {
            check(&["point", "line"])?;
            Rule::Midpoint { point: ref_field(v, "point", w)?, line: entity_field(v, "line", w)? }
        }
        "symmetric" => {
            check(&["a", "b", "about"])?;
            Rule::Symmetric { a: ref_field(v, "a", w)?, b: ref_field(v, "b", w)?, about: ref_field(v, "about", w)? }
        }
        "fix" => {
            check(&["what", "at"])?;
            let at = field(v, "at", w)?
                .as_array()
                .ok_or_else(|| "\"at\" must be a list of numbers".to_string())?
                .iter()
                .map(|n| n.as_f64().ok_or_else(|| "\"at\" must be a list of numbers".to_string()))
                .collect::<R<Vec<f64>>>()?;
            Rule::Fix { what: ref_field(v, "what", w)?, at }
        }
        "distance" => {
            check(&["a", "b", "value"])?;
            Rule::Distance { a: ref_field(v, "a", w)?, b: opt_ref(v, "b")?, value: value_field(v, w)? }
        }
        "horizontal_distance" | "vertical_distance" => {
            check(&["a", "b", "value"])?;
            let (a, b, value) = (ref_field(v, "a", w)?, ref_field(v, "b", w)?, value_field(v, w)?);
            if kind == "horizontal_distance" {
                Rule::HorizontalDistance { a, b, value }
            } else {
                Rule::VerticalDistance { a, b, value }
            }
        }
        "radius" | "diameter" | "major_radius" | "minor_radius" => {
            check(&["of", "value"])?;
            let (of, value) = (entity_field(v, "of", w)?, value_field(v, w)?);
            match kind {
                "radius" => Rule::Radius { of, value },
                "diameter" => Rule::Diameter { of, value },
                "major_radius" => Rule::MajorRadius { of, value },
                _ => Rule::MinorRadius { of, value },
            }
        }
        "angle" => {
            check(&["a", "b", "value"])?;
            let b = match opt_ref(v, "b")? {
                Some(x) if x.part != Part::Whole => return Err("an angle's \"b\" must name a whole line".to_string()),
                Some(x) => Some(x.entity),
                None => None,
            };
            Rule::Angle { a: entity_field(v, "a", w)?, b, value: value_field(v, w)? }
        }
        other => {
            return Err(format!(
                "unknown constraint \"{other}\"; expected coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, midpoint, concentric, symmetric, normal, fix, distance, horizontal_distance, vertical_distance, radius, diameter, major_radius, minor_radius or angle"
            ))
        }
    })
}

/// An entity's shape as JSON fields.
pub fn geometry_value(g: &Geometry) -> Vec<(&'static str, Value)> {
    let mut items = vec![("type", Value::str(g.kind()))];
    match *g {
        Geometry::Spline(ref shape) => {
            items.push(("degree", Value::Num(shape.degree as f64)));
            items.push(("points", Value::Arr(shape.points.iter().map(|q| p2_value(*q)).collect())));
            items.push(("knots", Value::Arr(shape.knots.iter().map(|k| Value::Num(*k)).collect())));
            if shape.weights.iter().any(|w| *w != 1.0) {
                items.push(("weights", Value::Arr(shape.weights.iter().map(|w| Value::Num(*w)).collect())));
            }
        }
        Geometry::Point { at } => items.push(("at", p2_value(at))),
        Geometry::Line { start, end } => {
            items.push(("start", p2_value(start)));
            items.push(("end", p2_value(end)));
        }
        Geometry::Circle { center, radius } => {
            items.push(("center", p2_value(center)));
            items.push(("radius", Value::Num(radius)));
        }
        Geometry::Arc { center, radius, start, end } => {
            items.push(("center", p2_value(center)));
            items.push(("radius", Value::Num(radius)));
            items.push(("start", p2_value(start)));
            items.push(("end", p2_value(end)));
        }
        Geometry::Ellipse { center, major, minor } => {
            items.push(("center", p2_value(center)));
            items.push(("major", p2_value(major)));
            items.push(("minor", Value::Num(minor)));
        }
        Geometry::EllipticalArc { center, major, minor, start, end } => {
            items.push(("center", p2_value(center)));
            items.push(("major", p2_value(major)));
            items.push(("minor", Value::Num(minor)));
            items.push(("start", p2_value(start)));
            items.push(("end", p2_value(end)));
        }
    }
    items
}

/// Reads an entity's shape. `extra` lists fields that belong to the caller.
pub fn read_geometry(v: &Value, extra: &[&str]) -> R<Geometry> {
    let check = |own: &[&str]| -> R<()> {
        let mut all = vec!["type"];
        all.extend_from_slice(own);
        all.extend_from_slice(extra);
        known(v, &all)
    };
    let kind = field(v, "type", "an entity")?.as_str().unwrap_or("");
    let what = format!("a {kind}");
    let w = what.as_str();
    Ok(match kind {
        "point" => {
            check(&["at"])?;
            Geometry::Point { at: point(field(v, "at", w)?)? }
        }
        "line" => {
            check(&["start", "end"])?;
            Geometry::Line { start: point(field(v, "start", w)?)?, end: point(field(v, "end", w)?)? }
        }
        "circle" => {
            check(&["center", "radius"])?;
            Geometry::Circle { center: point(field(v, "center", w)?)?, radius: number(v, "radius", w)? }
        }
        "arc" => {
            check(&["center", "radius", "start", "end"])?;
            Geometry::Arc {
                center: point(field(v, "center", w)?)?,
                radius: number(v, "radius", w)?,
                start: point(field(v, "start", w)?)?,
                end: point(field(v, "end", w)?)?,
            }
        }
        "ellipse" => {
            check(&["center", "major", "minor"])?;
            Geometry::Ellipse { center: point(field(v, "center", w)?)?, major: point(field(v, "major", w)?)?, minor: number(v, "minor", w)? }
        }
        "elliptical_arc" => {
            check(&["center", "major", "minor", "start", "end"])?;
            Geometry::EllipticalArc {
                center: point(field(v, "center", w)?)?,
                major: point(field(v, "major", w)?)?,
                minor: number(v, "minor", w)?,
                start: point(field(v, "start", w)?)?,
                end: point(field(v, "end", w)?)?,
            }
        }
        "spline" => {
            check(&["degree", "points", "knots", "weights", "through"])?;
            let numbers = |key: &str| -> R<Option<Vec<f64>>> {
                match v.get(key) {
                    None => Ok(None),
                    Some(list) => list
                        .as_array()
                        .ok_or_else(|| format!("a spline's \"{key}\" must be a list of numbers"))?
                        .iter()
                        .map(|n| n.as_f64().ok_or_else(|| format!("a spline's \"{key}\" must be a list of numbers")))
                        .collect::<R<Vec<f64>>>()
                        .map(Some),
                }
            };
            let points_of = |key: &str| -> R<Option<Vec<P2>>> {
                match v.get(key) {
                    None => Ok(None),
                    Some(list) => list
                        .as_array()
                        .ok_or_else(|| format!("a spline's \"{key}\" must be a list of points"))?
                        .iter()
                        .map(point)
                        .collect::<R<Vec<P2>>>()
                        .map(Some),
                }
            };
            // Hand-written files can just say what it goes through.
            let shape = if let Some(through) = points_of("through")? {
                if v.get("points").is_some() || v.get("knots").is_some() {
                    return Err("a spline has either \"through\" or \"points\", not both".to_string());
                }
                SplineShape::through(&through)?
            } else {
                let points = points_of("points")?.ok_or("a spline needs \"points\" or \"through\"")?;
                let degree = match v.get("degree") {
                    Some(d) => d.as_f64().filter(|d| d.fract() == 0.0 && *d >= 0.0 && *d < 100.0).ok_or("a spline's degree must be a whole number")? as usize,
                    None => 3.min(points.len().saturating_sub(1)),
                };
                let mut shape = match numbers("knots")? {
                    Some(knots) => {
                        let n = points.len();
                        SplineShape::new(degree, knots, points, vec![1.0; n])?
                    }
                    None => SplineShape::uniform(degree, points)?,
                };
                if let Some(weights) = numbers("weights")? {
                    shape.weights = weights;
                    shape.check()?;
                }
                shape
            };
            Geometry::Spline(std::sync::Arc::new(shape))
        }
        other => return Err(format!("unknown entity \"{other}\"; expected point, line, circle, arc, ellipse, elliptical_arc or spline")),
    })
}

fn id_field(v: &Value, what: &str) -> R<u32> {
    let n = field(v, "id", what)?.as_f64().ok_or_else(|| format!("{what}'s id must be a whole number"))?;
    if n.fract() != 0.0 || !(1.0..4e9).contains(&n) {
        return Err(format!("{what}'s id must be a whole number from 1"));
    }
    Ok(n as u32)
}

impl Sketch {
    pub fn to_json(&self) -> Value {
        let entities = self
            .entities
            .iter()
            .map(|e| {
                let mut items = vec![("id", Value::Num(e.id.0 as f64))];
                match &e.projection {
                    // Projected geometry is construction unless it says it isn't, and keeps where it last was.
                    Some(p) => {
                        items.push(("type", Value::str("projected")));
                        items.push(("from", p.from.clone()));
                        if p.placed {
                            items.push(("geometry", Value::obj(geometry_value(&e.geometry))));
                        }
                        if !e.construction {
                            items.push(("construction", Value::Bool(false)));
                        }
                    }
                    None => {
                        items.extend(geometry_value(&e.geometry));
                        if e.construction {
                            items.push(("construction", Value::Bool(true)));
                        }
                    }
                }
                Value::obj(items)
            })
            .collect();
        let constraints = self
            .constraints
            .iter()
            .map(|c| {
                let mut items = vec![("id", Value::Num(c.id.0 as f64))];
                items.extend(rule_value(&c.rule));
                if c.reference {
                    items.push(("reference", Value::Bool(true)));
                }
                Value::obj(items)
            })
            .collect();
        let mut items = vec![
            ("format", Value::str(SKETCH_FORMAT)),
            ("version", Value::Num(SKETCH_VERSION)),
            ("next_entity", Value::Num(self.next_entity as f64)),
            ("next_constraint", Value::Num(self.next_constraint as f64)),
            ("entities", Value::Arr(entities)),
            ("constraints", Value::Arr(constraints)),
        ];
        if !self.picked_regions.is_empty() {
            items.push(("picked_regions", Value::Arr(self.picked_regions.iter().map(|r| Value::str(r.name())).collect())));
        }
        Value::obj(items)
    }

    pub fn save_text(&self) -> String {
        self.to_json().to_pretty()
    }

    pub fn load_text(text: &str) -> Result<Sketch, SketchError> {
        let v = json::parse(text).map_err(|m| invalid(format!("not valid JSON: {m}")))?;
        Sketch::from_json(&v)
    }

    pub fn from_json(v: &Value) -> Result<Sketch, SketchError> {
        Sketch::read(v).map_err(invalid)
    }

    fn read(v: &Value) -> R<Sketch> {
        known(v, &["format", "version", "next_entity", "next_constraint", "entities", "constraints", "picked_regions"])?;
        if v.get("format").and_then(Value::as_str) != Some(SKETCH_FORMAT) {
            return Err("this is not a CoreShape sketch".to_string());
        }
        let version = v.get("version").and_then(Value::as_f64).ok_or("the sketch has no version")?;
        if version != SKETCH_VERSION {
            return Err(format!("this sketch is format version {version}, and this solver reads version {SKETCH_VERSION}"));
        }
        let mut s = Sketch::new();
        for item in v.get("entities").and_then(Value::as_array).unwrap_or(&[]) {
            let id = EntityId(id_field(item, "an entity")?);
            if id.is_reference() {
                return Err(format!("entity id {} is reserved", id.0));
            }
            if s.entities.iter().any(|e| e.id == id) {
                return Err(format!("entity id {} appears twice", id.0));
            }
            let projected = item.get("type").and_then(Value::as_str) == Some("projected");
            let (geometry, projection) = if projected {
                known(item, &["id", "type", "from", "geometry", "construction"])?;
                let from = field(item, "from", "a projected entity")?;
                if !matches!(from, Value::Obj(_)) {
                    return Err(format!("projected entity {}'s \"from\" is {{\"vertex\": ...}} or {{\"edge\": ...}}", id.0));
                }
                match item.get("geometry") {
                    Some(g) => (read_geometry(g, &[])?, Some(Projection { from: from.clone(), placed: true })),
                    None => (Geometry::Point { at: [0.0, 0.0] }, Some(Projection { from: from.clone(), placed: false })),
                }
            } else {
                (read_geometry(item, &["id", "construction"])?, None)
            };
            let construction = match item.get("construction") {
                None => projected,
                Some(c) => c.as_bool().ok_or("\"construction\" must be true or false")?,
            };
            s.next_entity = s.next_entity.max(id.0 + 1);
            s.entities.push(Entity { id, geometry, construction, projection });
        }
        for item in v.get("constraints").and_then(Value::as_array).unwrap_or(&[]) {
            let id = ConstraintId(id_field(item, "a constraint")?);
            if s.constraints.iter().any(|c| c.id == id) {
                return Err(format!("constraint id {} appears twice", id.0));
            }
            let reference = match item.get("reference") {
                None => false,
                Some(r) => r.as_bool().ok_or("\"reference\" must be true or false")?,
            };
            // A reference dimension is measured, so a file needn't say what it comes to.
            let rule = match item {
                Value::Obj(fields) if reference && item.get("value").is_none() => {
                    let mut with_value = fields.clone();
                    with_value.push(("value".to_string(), Value::Num(0.0)));
                    read_rule(&Value::Obj(with_value), &["id", "reference"])?
                }
                _ => read_rule(item, &["id", "reference"])?,
            };
            if reference && rule.value().is_none() {
                return Err(format!("c{} is a {} constraint, which has no value, so it can't be a reference dimension", id.0, rule.kind()));
            }
            for e in rule.entities() {
                if !e.is_reference() && !s.entities.iter().any(|x| x.id == e) {
                    return Err(format!("c{} refers to entity {}, which doesn't exist", id.0, e.0));
                }
            }
            s.next_constraint = s.next_constraint.max(id.0 + 1);
            s.constraints.push(Constraint { id, rule, reference });
        }
        // Ids are never reused, so these must stay above every id ever handed out.
        let keep = |key: &str, floor: u32| {
            let n = v.get(key).and_then(Value::as_f64).unwrap_or(0.0);
            if n.is_finite() && n >= 0.0 && n < 4e9 {
                floor.max(n as u32)
            } else {
                floor
            }
        };
        s.next_entity = keep("next_entity", s.next_entity);
        s.next_constraint = keep("next_constraint", s.next_constraint);
        s.refresh_references();
        if let Some(list) = v.get("picked_regions") {
            let list = list.as_array().ok_or("\"picked_regions\" must be a list of region ids")?;
            for item in list {
                let name = item.as_str().ok_or("\"picked_regions\" must be a list of region ids")?;
                s.picked_regions.push(RegionId::parse(name)?);
            }
        }
        Ok(s)
    }
}

fn invalid(message: String) -> SketchError {
    SketchError { kind: ErrorKind::Invalid, message, constraints: Vec::new(), entities: Vec::new() }
}

impl Analysis {
    pub fn to_json(&self) -> Value {
        Value::obj(vec![
            ("dof", Value::Num(self.dof as f64)),
            ("free", Value::Arr(self.free.iter().map(|r| Value::str(r.name())).collect())),
            (
                "redundant",
                Value::Arr(
                    self.redundant
                        .iter()
                        .map(|d| {
                            Value::obj(vec![
                                ("constraint", Value::Num(d.constraint.0 as f64)),
                                ("partial", Value::Bool(d.partial)),
                                ("with", Value::Arr(d.with.iter().map(|c| Value::Num(c.0 as f64)).collect())),
                            ])
                        })
                        .collect(),
                ),
            ),
        ])
    }
}

impl SketchError {
    pub fn to_json(&self) -> Value {
        let kind = match self.kind {
            ErrorKind::Invalid => "invalid",
            ErrorKind::Conflict => "conflict",
            ErrorKind::NoSolution => "no_solution",
            ErrorKind::Collapsed => "collapsed",
            ErrorKind::Regions => "regions",
        };
        Value::obj(vec![
            ("kind", Value::str(kind)),
            ("message", Value::str(&self.message)),
            ("constraints", Value::Arr(self.constraints.iter().map(|c| Value::Num(c.0 as f64)).collect())),
            ("entities", Value::Arr(self.entities.iter().map(|c| Value::Num(c.0 as f64)).collect())),
        ])
    }
}

impl Solved {
    pub fn to_json(&self) -> Value {
        Value::obj(vec![
            ("sketch", self.sketch.to_json()),
            ("analysis", self.analysis.to_json()),
            ("iterations", Value::Num(self.iterations as f64)),
        ])
    }
}

fn seg_value(s: &Seg) -> Value {
    let mut items = vec![("entity", Value::Num(s.entity.0 as f64))];
    match s.kind {
        SegKind::Line { a, b } => {
            items.push(("type", Value::str("line")));
            items.push(("a", p2_value(a)));
            items.push(("b", p2_value(b)));
        }
        SegKind::Arc { center, radius, a0, a1 } => {
            items.push(("type", Value::str("arc")));
            items.push(("center", p2_value(center)));
            items.push(("radius", Value::Num(radius)));
            items.push(("a0", Value::Num(a0)));
            items.push(("a1", Value::Num(a1)));
        }
        SegKind::Ellipse { center, x, major, minor, a0, a1 } => {
            items.push(("type", Value::str("ellipse")));
            items.push(("center", p2_value(center)));
            items.push(("x", p2_value(x)));
            items.push(("major", Value::Num(major)));
            items.push(("minor", Value::Num(minor)));
            items.push(("a0", Value::Num(a0)));
            items.push(("a1", Value::Num(a1)));
        }
        SegKind::Spline { ref shape, t0, t1 } => {
            items.push(("type", Value::str("spline")));
            items.push(("degree", Value::Num(shape.degree as f64)));
            items.push(("points", Value::Arr(shape.points.iter().map(|q| p2_value(*q)).collect())));
            items.push(("knots", Value::Arr(shape.knots.iter().map(|k| Value::Num(*k)).collect())));
            items.push(("weights", Value::Arr(shape.weights.iter().map(|w| Value::Num(*w)).collect())));
            items.push(("t0", Value::Num(t0)));
            items.push(("t1", Value::Num(t1)));
        }
    }
    Value::obj(items)
}

fn loop_value(l: &Loop) -> Value {
    Value::Arr(l.segs.iter().map(seg_value).collect())
}

impl Region {
    pub fn to_json(&self) -> Value {
        Value::obj(vec![
            ("id", Value::str(self.id.name())),
            ("depth", Value::Num(self.depth as f64)),
            ("area", Value::Num(self.area())),
            ("outer", loop_value(&self.outer)),
            ("holes", Value::Arr(self.holes.iter().map(loop_value).collect())),
        ])
    }
}

impl Ignored {
    pub fn to_json(&self) -> Value {
        let mut items = vec![("entity", Value::Num(self.entity.0 as f64)), ("whole", Value::Bool(self.whole))];
        match self.why {
            WhyIgnored::FreeEnd { at } => {
                items.push(("why", Value::str("free_end")));
                items.push(("at", p2_value(at)));
            }
            WhyIgnored::Bridge => items.push(("why", Value::str("bridge"))),
            WhyIgnored::Overlap { with } => {
                items.push(("why", Value::str("overlap")));
                items.push(("with", Value::Num(with.0 as f64)));
            }
            WhyIgnored::NoLength => items.push(("why", Value::str("no_length"))),
            WhyIgnored::NoArea => items.push(("why", Value::str("no_area"))),
        }
        items.push(("pieces", Value::Arr(self.pieces.iter().map(seg_value).collect())));
        Value::obj(items)
    }
}

impl Arrangement {
    pub fn to_json(&self) -> Value {
        Value::obj(vec![
            ("regions", Value::Arr(self.regions.iter().map(Region::to_json).collect())),
            ("ignored", Value::Arr(self.ignored.iter().map(Ignored::to_json).collect())),
        ])
    }
}
