//! Assemblies as Onshape has them: instances of parts, placed by mates between mate connectors, with the parts themselves
//! untouched. An instance refers to a part in a Part Studio, so a change there comes through. Moving an instance changes
//! where it is and nothing else, and isn't a feature in any history. See `docs/FILE_FORMAT.md`.
//!
//! All of it is plain data, and a `Project` holds the tabs: Part Studios, which are `Document`s, and assemblies.

pub mod connector;
pub mod project;
pub mod solver;

pub use connector::{candidates, edge_candidates, Anchor, Candidate, EdgePoint, FacePoint};
pub use project::{Bom, BomRow, Interference, Placed, Project, Solved, Tab, TabContent, TabId};
pub use solver::{Dof, Goal, Limit, MateKind};

use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape_document::json::Value;
use coreshape_document::BodyId;
use std::collections::BTreeMap;

/// What an instance is an instance of.
#[derive(Clone, Debug, PartialEq)]
pub enum Source {
    /// A part in a Part Studio.
    Part { studio: TabId, body: BodyId },
    /// A whole assembly in another tab.
    Assembly { tab: TabId },
}

/// A use of a part or an assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct Instance {
    pub id: u32,
    pub name: String,
    pub source: Source,
    /// Where it is, in this assembly's coordinates.
    pub at: Frame,
    /// Held where it is. As in Onshape, this counts only in the assembly it's set in.
    pub fixed: bool,
    pub suppressed: bool,
    /// For an assembly: keeps its parts as they are in their own tab, moving as one. Otherwise its mates carry on working
    /// here, and its parts can move.
    pub locked: bool,
}

/// A place on an instance, which mates join. Its z is the primary axis.
#[derive(Clone, Debug, PartialEq)]
pub struct Connector {
    pub id: u32,
    /// The instance it's on, by id, and on into a sub-assembly by more ids.
    pub instance: Vec<u32>,
    pub anchor: Anchor,
    /// Moved from there, along its own axes.
    pub offset: [f64; 3],
    /// Turned about its own z, in degrees.
    pub rotation: f64,
    /// Primary axis the other way.
    pub flip: bool,
    /// Turned about its z by this many quarter turns.
    pub quarter_turns: i32,
    /// Made for one mate, which takes it along when it goes.
    pub implicit: bool,
}

/// How a mate's limit reads: which motion, and between what. Lengths in millimetres, and angles in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MateLimit {
    pub dof: Dof,
    pub min: f64,
    pub max: f64,
}

/// Two connectors held in some relation to each other: their z axes end up pointing the same way, which `flip` reverses.
#[derive(Clone, Debug, PartialEq)]
pub struct Mate {
    pub id: u32,
    pub name: String,
    pub kind: MateKind,
    pub a: u32,
    pub b: u32,
    /// Spacing from `a`'s frame to `b`'s, along `a`'s axes.
    pub offset: [f64; 3],
    /// Turned about `a`'s z, in degrees.
    pub rotation: f64,
    pub flip: bool,
    pub quarter_turns: i32,
    pub limits: Vec<MateLimit>,
    pub suppressed: bool,
    /// How far it's turned, in radians, with its turns, kept so that the next turn is told from this one.
    pub turned: f64,
}

/// Things that move as one, without any mates.
#[derive(Clone, Debug, PartialEq)]
pub struct Group {
    pub id: u32,
    pub name: String,
    pub instances: Vec<Vec<u32>>,
    pub suppressed: bool,
}

/// How a relation ties one mate's motion to another's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RelationKind {
    /// Two turns: `b` turns `ratio` times as far as `a`.
    Gear { ratio: f64 },
    /// A turn and a slide: `distance` millimetres for each whole turn.
    RackAndPinion { distance: f64 },
    /// A cylindrical mate's own turn and slide: `lead` millimetres for each whole turn.
    Screw { lead: f64 },
    /// Two slides: `b` goes `ratio` times as far as `a`.
    Linear { ratio: f64 },
}

/// One mate's motion tied to another's. A screw names one mate twice.
#[derive(Clone, Debug, PartialEq)]
pub struct Relation {
    pub id: u32,
    pub kind: RelationKind,
    pub a: u32,
    pub b: u32,
    /// The other way round.
    pub reverse: bool,
    /// What `b` is when `a` is zero, as they were when this was set up, so that adding it moves nothing.
    pub offset: f64,
    pub suppressed: bool,
}

/// An assembly: all plain data.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Assembly {
    pub next_id: u32,
    pub instances: Vec<Instance>,
    pub connectors: Vec<Connector>,
    pub mates: Vec<Mate>,
    pub groups: Vec<Group>,
    pub relations: Vec<Relation>,
    /// Where things inside a sub-assembly have got to, when they've moved here: its parts when its mates let them move, or
    /// a locked sub-assembly nested in another. By their paths, relative to the top-level instance they're in, so that
    /// moving that takes them along.
    pub moved: BTreeMap<Vec<u32>, Frame>,
    /// Whatever an app wants kept with it.
    pub app: Option<Value>,
}

impl Assembly {
    pub fn new() -> Assembly {
        Assembly { next_id: 1, ..Assembly::default() }
    }

    fn fresh_id(&mut self) -> u32 {
        let id = self.next_id.max(1);
        self.next_id = id + 1;
        id
    }

    pub fn instance(&self, id: u32) -> Option<&Instance> {
        self.instances.iter().find(|i| i.id == id)
    }

    pub fn instance_mut(&mut self, id: u32) -> Result<&mut Instance> {
        self.instances.iter_mut().find(|i| i.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no instance {id}")))
    }

    pub fn mate(&self, id: u32) -> Option<&Mate> {
        self.mates.iter().find(|m| m.id == id)
    }

    pub fn mate_mut(&mut self, id: u32) -> Result<&mut Mate> {
        self.mates.iter_mut().find(|m| m.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no mate {id}")))
    }

    pub fn connector(&self, id: u32) -> Option<&Connector> {
        self.connectors.iter().find(|c| c.id == id)
    }

    pub fn connector_mut(&mut self, id: u32) -> Result<&mut Connector> {
        self.connectors.iter_mut().find(|c| c.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no connector {id}")))
    }

    /// Adds an instance. A new one sits where its source is, as the part would in its own Part Studio.
    pub fn add_instance(&mut self, source: Source, name: String, at: Frame) -> u32 {
        let id = self.fresh_id();
        self.instances.push(Instance { id, name, source, at, fixed: false, suppressed: false, locked: false });
        id
    }

    /// Adds a connector on an instance.
    pub fn add_connector(&mut self, instance: Vec<u32>, anchor: Anchor) -> Result<u32> {
        match instance.first() {
            Some(first) if self.instance(*first).is_some() => {}
            _ => return Err(Error::InvalidInput(format!("a connector has to be on an instance, and there's no instance {instance:?}"))),
        }
        let id = self.fresh_id();
        self.connectors.push(Connector { id, instance, anchor, offset: [0.0; 3], rotation: 0.0, flip: false, quarter_turns: 0, implicit: false });
        Ok(id)
    }

    /// Adds a mate between two connectors.
    pub fn add_mate(&mut self, kind: MateKind, a: u32, b: u32) -> Result<u32> {
        for c in [a, b] {
            if self.connector(c).is_none() {
                return Err(Error::InvalidInput(format!("there's no connector {c}")));
            }
        }
        if a == b || self.connector(a).map(|c| &c.instance) == self.connector(b).map(|c| &c.instance) {
            return Err(Error::InvalidInput("a mate has to be between two different instances".into()));
        }
        let id = self.fresh_id();
        let n = self.mates.iter().filter(|m| m.kind == kind).count() + 1;
        self.mates.push(Mate { id, name: format!("{} {n}", kind_label(kind)), kind, a, b, offset: [0.0; 3], rotation: 0.0, flip: false, quarter_turns: 0, limits: Vec::new(), suppressed: false, turned: 0.0 });
        Ok(id)
    }

    /// Adds a group of instances that move as one.
    pub fn add_group(&mut self, instances: Vec<Vec<u32>>) -> Result<u32> {
        if instances.len() < 2 {
            return Err(Error::InvalidInput("a group needs at least two instances".into()));
        }
        for path in &instances {
            if !path.first().is_some_and(|p| self.instance(*p).is_some()) {
                return Err(Error::InvalidInput(format!("there's no instance {path:?}")));
            }
        }
        let id = self.fresh_id();
        let n = self.groups.len() + 1;
        self.groups.push(Group { id, name: format!("Group {n}"), instances, suppressed: false });
        Ok(id)
    }

    /// Adds a relation between mates, or one mate's two motions for a screw. What it needs of them is checked here.
    pub fn add_relation(&mut self, kind: RelationKind, a: u32, b: u32, reverse: bool) -> Result<u32> {
        let (ma, mb) = (self.mate(a).ok_or_else(|| Error::InvalidInput(format!("there's no mate {a}")))?, self.mate(b).ok_or_else(|| Error::InvalidInput(format!("there's no mate {b}")))?);
        let turns = |m: &Mate| m.kind.values().contains(&Dof::Angle);
        let slides = |m: &Mate| m.kind.values().contains(&Dof::Z) || m.kind.values().contains(&Dof::X);
        let fits = match kind {
            RelationKind::Gear { .. } => turns(ma) && turns(mb),
            RelationKind::RackAndPinion { .. } => turns(ma) && slides(mb),
            RelationKind::Screw { .. } => a == b && ma.kind == MateKind::Cylindrical,
            RelationKind::Linear { .. } => slides(ma) && slides(mb),
        };
        if !fits {
            return Err(Error::InvalidInput(match kind {
                RelationKind::Gear { .. } => "a gear ties two mates that turn".to_string(),
                RelationKind::RackAndPinion { .. } => "a rack and pinion ties a mate that turns to a mate that slides".to_string(),
                RelationKind::Screw { .. } => "a screw is on one cylindrical mate".to_string(),
                RelationKind::Linear { .. } => "a linear relation ties two mates that slide".to_string(),
            }));
        }
        let id = self.fresh_id();
        self.relations.push(Relation { id, kind, a, b, reverse, offset: 0.0, suppressed: false });
        Ok(id)
    }

    /// Deletes a mate, with any relations on it and its implicit connectors, if nothing else uses them.
    pub fn delete_mate(&mut self, id: u32) -> Result<()> {
        let at = self.mates.iter().position(|m| m.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no mate {id}")))?;
        let mate = self.mates.remove(at);
        self.relations.retain(|r| r.a != id && r.b != id);
        for c in [mate.a, mate.b] {
            let used = self.mates.iter().any(|m| m.a == c || m.b == c);
            if !used && self.connector(c).is_some_and(|k| k.implicit) {
                self.connectors.retain(|k| k.id != c);
            }
        }
        Ok(())
    }

    /// Deletes an instance, with its connectors and whatever mates and groups use it.
    pub fn delete_instance(&mut self, id: u32) -> Result<()> {
        let at = self.instances.iter().position(|i| i.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no instance {id}")))?;
        self.instances.remove(at);
        let on_it: Vec<u32> = self.connectors.iter().filter(|c| c.instance.first() == Some(&id)).map(|c| c.id).collect();
        let mates: Vec<u32> = self.mates.iter().filter(|m| on_it.contains(&m.a) || on_it.contains(&m.b)).map(|m| m.id).collect();
        for m in mates {
            self.delete_mate(m)?;
        }
        self.connectors.retain(|c| !on_it.contains(&c.id));
        for g in &mut self.groups {
            g.instances.retain(|p| p.first() != Some(&id));
        }
        self.groups.retain(|g| g.instances.len() >= 2);
        self.moved.retain(|p, _| p.first() != Some(&id));
        Ok(())
    }

}

pub fn kind_label(kind: MateKind) -> &'static str {
    match kind {
        MateKind::Fastened => "Fastened",
        MateKind::Revolute => "Revolute",
        MateKind::Slider => "Slider",
        MateKind::Cylindrical => "Cylindrical",
        MateKind::PinSlot => "Pin slot",
        MateKind::Planar => "Planar",
        MateKind::Ball => "Ball",
        MateKind::Parallel => "Parallel",
    }
}

pub fn kind_name(kind: MateKind) -> &'static str {
    match kind {
        MateKind::Fastened => "fastened",
        MateKind::Revolute => "revolute",
        MateKind::Slider => "slider",
        MateKind::Cylindrical => "cylindrical",
        MateKind::PinSlot => "pin_slot",
        MateKind::Planar => "planar",
        MateKind::Ball => "ball",
        MateKind::Parallel => "parallel",
    }
}

pub fn read_kind(name: &str) -> Result<MateKind> {
    Ok(match name {
        "fastened" => MateKind::Fastened,
        "revolute" => MateKind::Revolute,
        "slider" => MateKind::Slider,
        "cylindrical" => MateKind::Cylindrical,
        "pin_slot" => MateKind::PinSlot,
        "planar" => MateKind::Planar,
        "ball" => MateKind::Ball,
        "parallel" => MateKind::Parallel,
        other => return Err(Error::InvalidInput(format!("unknown mate \"{other}\"; expected fastened, revolute, slider, cylindrical, pin_slot, planar, ball or parallel"))),
    })
}

pub fn dof_name(dof: Dof) -> &'static str {
    match dof {
        Dof::X => "x",
        Dof::Y => "y",
        Dof::Z => "z",
        Dof::Angle => "angle",
    }
}

pub fn read_dof(name: &str) -> Result<Dof> {
    Ok(match name {
        "x" => Dof::X,
        "y" => Dof::Y,
        "z" => Dof::Z,
        "angle" => Dof::Angle,
        other => return Err(Error::InvalidInput(format!("unknown motion \"{other}\"; expected x, y, z or angle"))),
    })
}

// ----- Saved form -----

fn num(x: f64) -> Value {
    Value::Num(x)
}

fn numbers(v: &[f64]) -> Value {
    Value::Arr(v.iter().map(|x| Value::Num(*x)).collect())
}

fn vec3_value(v: Vec3) -> Value {
    numbers(&[v.x, v.y, v.z])
}

pub fn frame_value(f: &Frame) -> Value {
    Value::obj(vec![("origin", vec3_value(f.origin)), ("z", vec3_value(f.z)), ("x", vec3_value(f.x))])
}

fn path_value(p: &[u32]) -> Value {
    Value::Arr(p.iter().map(|x| Value::Num(*x as f64)).collect())
}

fn bad(what: &str) -> Error {
    Error::InvalidInput(format!("{what} isn't right in this file"))
}

fn known(v: &Value, allowed: &[&str]) -> Result<()> {
    if let Value::Obj(items) = v {
        for (k, _) in items {
            if !allowed.contains(&k.as_str()) {
                return Err(Error::InvalidInput(format!("unknown field \"{k}\": expected one of {}", allowed.join(", "))));
            }
        }
        Ok(())
    } else {
        Err(Error::InvalidInput("expected an object".into()))
    }
}

pub fn read_vec3(v: &Value) -> Result<Vec3> {
    let list = v.as_array().filter(|l| l.len() == 3).ok_or_else(|| bad("a point"))?;
    let at = |i: usize| list[i].as_f64().filter(|x| x.is_finite()).ok_or_else(|| bad("a point"));
    Ok(v3(at(0)?, at(1)?, at(2)?))
}

pub fn read_frame(v: &Value) -> Result<Frame> {
    known(v, &["origin", "z", "x"])?;
    let get = |key: &str, default: Vec3| -> Result<Vec3> { v.get(key).map_or(Ok(default), read_vec3) };
    let (origin, z, x) = (get("origin", Vec3::ZERO)?, get("z", Vec3::Z)?, get("x", Vec3::X)?);
    if z.norm() < 1e-12 || z.cross(x).norm() < 1e-9 * z.norm() * x.norm().max(1e-300) {
        return Err(Error::InvalidInput("a placement's z and x directions must have length and not be parallel".into()));
    }
    Ok(Frame::new(origin, z, x))
}

fn read_id(v: Option<&Value>, what: &str) -> Result<u32> {
    v.and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).map(|n| n as u32).ok_or_else(|| bad(what))
}

fn read_path(v: Option<&Value>, what: &str) -> Result<Vec<u32>> {
    match v {
        Some(Value::Num(_)) => Ok(vec![read_id(v, what)?]),
        Some(list) => list.as_array().ok_or_else(|| bad(what))?.iter().map(|x| read_id(Some(x), what)).collect(),
        None => Err(bad(what)),
    }
}

fn read_flag(v: &Value, key: &str) -> Result<bool> {
    match v.get(key) {
        None => Ok(false),
        Some(b) => b.as_bool().ok_or_else(|| Error::InvalidInput(format!("\"{key}\" must be true or false"))),
    }
}

fn read_number(v: &Value, key: &str, default: f64) -> Result<f64> {
    match v.get(key) {
        None => Ok(default),
        Some(n) => n.as_f64().filter(|x| x.is_finite()).ok_or_else(|| Error::InvalidInput(format!("\"{key}\" must be a number"))),
    }
}

fn read_offset(v: &Value) -> Result<[f64; 3]> {
    match v.get("offset") {
        None => Ok([0.0; 3]),
        Some(o) => {
            let p = read_vec3(o)?;
            Ok([p.x, p.y, p.z])
        }
    }
}

fn with_common(items: &mut Vec<(&str, Value)>, offset: &[f64; 3], rotation: f64, flip: bool, quarter_turns: i32) {
    if *offset != [0.0; 3] {
        items.push(("offset", numbers(offset)));
    }
    if rotation != 0.0 {
        items.push(("rotation", num(rotation)));
    }
    if flip {
        items.push(("flip", Value::Bool(true)));
    }
    if quarter_turns != 0 {
        items.push(("quarter_turns", num(quarter_turns as f64)));
    }
}

fn relation_kind_value(kind: RelationKind) -> Vec<(&'static str, Value)> {
    match kind {
        RelationKind::Gear { ratio } => vec![("type", Value::str("gear")), ("ratio", num(ratio))],
        RelationKind::RackAndPinion { distance } => vec![("type", Value::str("rack_and_pinion")), ("distance", num(distance))],
        RelationKind::Screw { lead } => vec![("type", Value::str("screw")), ("lead", num(lead))],
        RelationKind::Linear { ratio } => vec![("type", Value::str("linear")), ("ratio", num(ratio))],
    }
}

pub fn read_relation_kind(v: &Value) -> Result<RelationKind> {
    let need = |key: &str| read_number(v, key, f64::NAN).and_then(|x| if x.is_nan() { Err(Error::InvalidInput(format!("a relation needs \"{key}\""))) } else { Ok(x) });
    Ok(match v.get("type").and_then(Value::as_str).unwrap_or("") {
        "gear" => RelationKind::Gear { ratio: need("ratio")? },
        "rack_and_pinion" => RelationKind::RackAndPinion { distance: need("distance")? },
        "screw" => RelationKind::Screw { lead: need("lead")? },
        "linear" => RelationKind::Linear { ratio: need("ratio")? },
        other => return Err(Error::InvalidInput(format!("unknown relation \"{other}\"; expected gear, rack_and_pinion, screw or linear"))),
    })
}

impl Assembly {
    pub fn to_json(&self) -> Value {
        let instances = self.instances.iter().map(|i| {
            let source = match &i.source {
                Source::Part { studio, body } => Value::obj(vec![("tab", num(studio.0 as f64)), ("body", Value::str(body.text()))]),
                Source::Assembly { tab } => Value::obj(vec![("assembly", num(tab.0 as f64))]),
            };
            let mut items = vec![("id", num(i.id as f64)), ("name", Value::str(&i.name)), ("source", source), ("at", frame_value(&i.at))];
            if i.fixed {
                items.push(("fixed", Value::Bool(true)));
            }
            if i.suppressed {
                items.push(("suppressed", Value::Bool(true)));
            }
            if i.locked {
                items.push(("locked", Value::Bool(true)));
            }
            Value::obj(items)
        });
        let connectors = self.connectors.iter().map(|c| {
            let mut items = vec![("id", num(c.id as f64)), ("instance", path_value(&c.instance)), ("on", c.anchor.to_json())];
            with_common(&mut items, &c.offset, c.rotation, c.flip, c.quarter_turns);
            if c.implicit {
                items.push(("implicit", Value::Bool(true)));
            }
            Value::obj(items)
        });
        let mates = self.mates.iter().map(|m| {
            let mut items = vec![("id", num(m.id as f64)), ("name", Value::str(&m.name)), ("type", Value::str(kind_name(m.kind))), ("a", num(m.a as f64)), ("b", num(m.b as f64))];
            with_common(&mut items, &m.offset, m.rotation, m.flip, m.quarter_turns);
            if !m.limits.is_empty() {
                let limits = m.limits.iter().map(|l| Value::obj(vec![("dof", Value::str(dof_name(l.dof))), ("min", num(l.min)), ("max", num(l.max))])).collect();
                items.push(("limits", Value::Arr(limits)));
            }
            if m.suppressed {
                items.push(("suppressed", Value::Bool(true)));
            }
            if m.turned != 0.0 {
                items.push(("turned", num(m.turned)));
            }
            Value::obj(items)
        });
        let groups = self.groups.iter().map(|g| {
            let mut items = vec![("id", num(g.id as f64)), ("name", Value::str(&g.name)), ("instances", Value::Arr(g.instances.iter().map(|p| path_value(p)).collect()))];
            if g.suppressed {
                items.push(("suppressed", Value::Bool(true)));
            }
            Value::obj(items)
        });
        let relations = self.relations.iter().map(|r| {
            let mut items = relation_kind_value(r.kind);
            items.extend([("id", num(r.id as f64)), ("a", num(r.a as f64)), ("b", num(r.b as f64))]);
            if r.reverse {
                items.push(("reverse", Value::Bool(true)));
            }
            if r.offset != 0.0 {
                items.push(("offset", num(r.offset)));
            }
            if r.suppressed {
                items.push(("suppressed", Value::Bool(true)));
            }
            Value::obj(items)
        });
        let mut items = vec![
            ("next_id", num(self.next_id as f64)),
            ("instances", Value::Arr(instances.collect())),
            ("connectors", Value::Arr(connectors.collect())),
            ("mates", Value::Arr(mates.collect())),
            ("groups", Value::Arr(groups.collect())),
            ("relations", Value::Arr(relations.collect())),
        ];
        if !self.moved.is_empty() {
            let moved = self.moved.iter().map(|(p, f)| Value::obj(vec![("path", path_value(p)), ("at", frame_value(f))])).collect();
            items.push(("moved", Value::Arr(moved)));
        }
        if let Some(app) = &self.app {
            items.push(("app", app.clone()));
        }
        Value::obj(items)
    }

    pub fn from_json(v: &Value) -> Result<Assembly> {
        known(v, &["next_id", "instances", "connectors", "mates", "groups", "relations", "moved", "app"])?;
        let mut out = Assembly::new();
        out.app = v.get("app").cloned();
        let list = |key: &str| v.get(key).and_then(Value::as_array).unwrap_or(&[]);
        for i in list("instances") {
            known(i, &["id", "name", "source", "at", "fixed", "suppressed", "locked"])?;
            let s = i.get("source").ok_or_else(|| bad("an instance's source"))?;
            let source = if s.get("assembly").is_some() {
                known(s, &["assembly"])?;
                Source::Assembly { tab: TabId(read_id(s.get("assembly"), "an instance's assembly")?) }
            } else {
                known(s, &["tab", "body"])?;
                let body = s.get("body").and_then(Value::as_str).ok_or_else(|| bad("an instance's part"))?;
                Source::Part { studio: TabId(read_id(s.get("tab"), "an instance's tab")?), body: BodyId::parse(body)? }
            };
            out.instances.push(Instance {
                id: read_id(i.get("id"), "an instance's id")?,
                name: i.get("name").and_then(Value::as_str).unwrap_or("").to_string(),
                source,
                at: i.get("at").map_or(Ok(Frame::WORLD), read_frame)?,
                fixed: read_flag(i, "fixed")?,
                suppressed: read_flag(i, "suppressed")?,
                locked: read_flag(i, "locked")?,
            });
        }
        for c in list("connectors") {
            known(c, &["id", "instance", "on", "offset", "rotation", "flip", "quarter_turns", "implicit"])?;
            out.connectors.push(Connector {
                id: read_id(c.get("id"), "a connector's id")?,
                instance: read_path(c.get("instance"), "a connector's instance")?,
                anchor: Anchor::from_json(c.get("on").ok_or_else(|| bad("a connector's place"))?)?,
                offset: read_offset(c)?,
                rotation: read_number(c, "rotation", 0.0)?,
                flip: read_flag(c, "flip")?,
                quarter_turns: read_number(c, "quarter_turns", 0.0)? as i32,
                implicit: read_flag(c, "implicit")?,
            });
        }
        for m in list("mates") {
            known(m, &["id", "name", "type", "a", "b", "offset", "rotation", "flip", "quarter_turns", "limits", "suppressed", "turned"])?;
            let mut limits = Vec::new();
            for l in m.get("limits").and_then(Value::as_array).unwrap_or(&[]) {
                known(l, &["dof", "min", "max"])?;
                limits.push(MateLimit { dof: read_dof(l.get("dof").and_then(Value::as_str).unwrap_or(""))?, min: read_number(l, "min", f64::NEG_INFINITY)?, max: read_number(l, "max", f64::INFINITY)? });
            }
            out.mates.push(Mate {
                id: read_id(m.get("id"), "a mate's id")?,
                name: m.get("name").and_then(Value::as_str).unwrap_or("").to_string(),
                kind: read_kind(m.get("type").and_then(Value::as_str).unwrap_or(""))?,
                a: read_id(m.get("a"), "a mate's connector")?,
                b: read_id(m.get("b"), "a mate's connector")?,
                offset: read_offset(m)?,
                rotation: read_number(m, "rotation", 0.0)?,
                flip: read_flag(m, "flip")?,
                quarter_turns: read_number(m, "quarter_turns", 0.0)? as i32,
                limits,
                suppressed: read_flag(m, "suppressed")?,
                turned: read_number(m, "turned", 0.0)?,
            });
        }
        for g in list("groups") {
            known(g, &["id", "name", "instances", "suppressed"])?;
            let paths = g.get("instances").and_then(Value::as_array).unwrap_or(&[]).iter().map(|p| read_path(Some(p), "a group's instance")).collect::<Result<Vec<_>>>()?;
            out.groups.push(Group { id: read_id(g.get("id"), "a group's id")?, name: g.get("name").and_then(Value::as_str).unwrap_or("").to_string(), instances: paths, suppressed: read_flag(g, "suppressed")? });
        }
        for r in list("relations") {
            known(r, &["id", "type", "ratio", "distance", "lead", "a", "b", "reverse", "offset", "suppressed"])?;
            out.relations.push(Relation {
                id: read_id(r.get("id"), "a relation's id")?,
                kind: read_relation_kind(r)?,
                a: read_id(r.get("a"), "a relation's mate")?,
                b: read_id(r.get("b"), "a relation's mate")?,
                reverse: read_flag(r, "reverse")?,
                offset: read_number(r, "offset", 0.0)?,
                suppressed: read_flag(r, "suppressed")?,
            });
        }
        for entry in list("moved") {
            known(entry, &["path", "at"])?;
            out.moved.insert(read_path(entry.get("path"), "a moved part's path")?, read_frame(entry.get("at").ok_or_else(|| bad("a moved part's place"))?)?);
        }
        // Every reference has to be to something here, and ids stay above all those ever handed out.
        let mut ids: Vec<u32> = out.instances.iter().map(|i| i.id).chain(out.connectors.iter().map(|c| c.id)).chain(out.mates.iter().map(|m| m.id)).chain(out.groups.iter().map(|g| g.id)).chain(out.relations.iter().map(|r| r.id)).collect();
        ids.sort_unstable();
        if ids.windows(2).any(|w| w[0] == w[1]) {
            return Err(Error::InvalidInput("an id appears twice in this assembly".into()));
        }
        for c in &out.connectors {
            if !c.instance.first().is_some_and(|i| out.instance(*i).is_some()) {
                return Err(Error::InvalidInput(format!("connector {} is on instance {:?}, which isn't here", c.id, c.instance)));
            }
        }
        for m in &out.mates {
            for c in [m.a, m.b] {
                if out.connector(c).is_none() {
                    return Err(Error::InvalidInput(format!("mate {} uses connector {c}, which isn't here", m.id)));
                }
            }
        }
        for r in &out.relations {
            for m in [r.a, r.b] {
                if out.mate(m).is_none() {
                    return Err(Error::InvalidInput(format!("relation {} uses mate {m}, which isn't here", r.id)));
                }
            }
        }
        let top = ids.last().copied().unwrap_or(0);
        let saved = read_number(v, "next_id", 0.0)?;
        out.next_id = (top + 1).max(if saved >= 0.0 && saved < 4e9 { saved as u32 } else { 0 });
        Ok(out)
    }
}
