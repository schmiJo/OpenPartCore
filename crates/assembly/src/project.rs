//! A project: tabs of Part Studios and assemblies, as an Onshape document has. This is where an assembly meets its parts.

use crate::solver::{self, Contact, Coupling, Dof, Drive, Goal, Joint, Limit, MateKind, Problem, Report, Touching};
use crate::{Anchor, Assembly, Connector, Mate, PatternAxis, PatternKind, Position, RelationKind, Source};
use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::topo::Body;
use coreshape_document::json::{self, Value};
use coreshape_document::{BodyId, Document, Op};
use std::collections::BTreeMap;
use std::sync::Arc;

const FORMAT: &str = "coreshape-project";
const FORMAT_VERSION: f64 = 1.0;

/// A tab's permanent id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TabId(pub u32);

#[derive(Clone)]
pub enum TabContent {
    PartStudio(Document),
    Assembly(Assembly),
    Drawing(DrawingTab),
}

/// What a drawing shows: a Part Studio's parts, one or all, or an assembly as it solves.
#[derive(Clone, Debug, PartialEq)]
pub enum DrawingSource {
    Part { studio: TabId, body: Option<BodyId> },
    Assembly { tab: TabId },
}

/// A drawing tab: what it shows and the drawing itself, which is made again from it each time, so it follows the model.
#[derive(Clone, Debug, PartialEq)]
pub struct DrawingTab {
    pub source: DrawingSource,
    pub drawing: coreshape_drawing::Drawing,
}

#[derive(Clone)]
pub struct Tab {
    pub id: TabId,
    pub name: String,
    pub content: TabContent,
}

/// All the tabs.
#[derive(Clone)]
pub struct Project {
    pub tabs: Vec<Tab>,
    next_id: u32,
}

/// A part where an assembly has it.
#[derive(Clone, Debug)]
pub struct Placed {
    /// The instances down to it, by id.
    pub path: Vec<u32>,
    pub name: String,
    pub studio: TabId,
    pub part: BodyId,
    pub body: Arc<Body>,
    pub at: Frame,
}

/// How solving an assembly went.
#[derive(Clone, Debug, Default)]
pub struct Solved {
    pub placed: Vec<Placed>,
    /// Mates that couldn't be met, with why. The rest are met.
    pub failing: Vec<(u32, String)>,
    /// Relations that couldn't be met.
    pub failing_relations: Vec<u32>,
    /// What's wrong that isn't about a mate, such as an instance whose part has gone.
    pub problems: Vec<String>,
    /// How many ways it can still move.
    pub freedom: usize,
    /// The instances that can still move, by their paths.
    pub free: Vec<Vec<u32>>,
    /// Each top-level mate's value for each motion it has: millimetres, or degrees with their turns.
    pub values: Vec<(u32, Dof, f64)>,
    /// How far a drag still was from where it was wanted, if this was a drag.
    pub remaining: Option<f64>,
}

impl Solved {
    /// All the parts in place, side by side, for showing and measuring.
    pub fn whole(&self) -> Option<Body> {
        if self.placed.is_empty() {
            return None;
        }
        let moved: Vec<Body> = self.placed.iter().map(|p| p.body.transformed(&p.at)).collect();
        let list: Vec<&Body> = moved.iter().collect();
        Some(Body::side_by_side(&list))
    }
}

/// Two parts in the same place.
#[derive(Clone, Debug, PartialEq)]
pub struct Interference {
    pub a: Vec<u32>,
    pub b: Vec<u32>,
    pub volume: f64,
}

/// One line of a bill of materials.
#[derive(Clone, Debug, PartialEq)]
pub struct BomRow {
    pub name: String,
    pub studio: TabId,
    pub part: BodyId,
    pub count: usize,
    pub volume: f64,
    pub area: f64,
}

/// A bill of materials.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Bom {
    pub rows: Vec<BomRow>,
}

/// A connector's frame, in its part's own coordinates, with its own adjustments.
pub fn connector_frame(c: &Connector, body: &Body) -> Result<Frame> {
    let base = c.anchor.resolve(body)?;
    let moved = Frame::translation(v3(c.offset[0], c.offset[1], c.offset[2]));
    let turned = Frame::rotation(Vec3::ZERO, Vec3::Z, c.rotation.to_radians() + c.quarter_turns as f64 * FRAC_PI_2);
    let flipped = if c.flip { Frame::rotation(Vec3::ZERO, Vec3::X, PI) } else { Frame::WORLD };
    Ok(base.compose(&moved).compose(&turned).compose(&flipped))
}

const FRAC_PI_2: f64 = std::f64::consts::FRAC_PI_2;
/// How many times a drive halves the last step, to find where something stops it: a 15 degree step comes to under 0.004.
const STOP_HALVINGS: usize = 12;

/// What came out of flattening an assembly into things that move.
#[derive(Default)]
struct Flat {
    placed: Vec<Placed>,
    fixed: Vec<bool>,
    /// The pose of the top-level instance each is in.
    top_at: Vec<Frame>,
    /// For a part in a locked sub-assembly, that sub-assembly's path and where it is.
    locked: Vec<Option<(Vec<u32>, Frame)>>,
    /// Things that move as one.
    groups: Vec<Vec<usize>>,
    joints: Vec<Joint>,
    /// For each joint, its mate's id and the path of the assembly that has it, which is empty for the top one.
    joint_mates: Vec<(u32, Vec<u32>)>,
    couplings: Vec<Coupling>,
    coupling_relations: Vec<u32>,
    failing: Vec<(u32, String)>,
    problems: Vec<String>,
    /// The tabs of locked sub-assemblies, solved once each.
    solved_tabs: Vec<TabId>,
    /// Copies from patterns and replicates, placed once the rest is solved.
    copies: Vec<CopyPlan>,
}

/// A copy of a part from a pattern or a replicate: the part, and how it's placed from what it copies. Its place is
/// `local`, or with `thing` the move `local` makes in that thing's own coordinates, done to where `seed` is.
#[derive(Clone)]
struct CopyPlan {
    template: Placed,
    seed: usize,
    thing: Option<usize>,
    local: Frame,
}

/// Where each copy goes, given where everything else is.
fn place_copies(copies: &[CopyPlan], poses: &[Frame]) -> Vec<Placed> {
    copies
        .iter()
        .map(|c| {
            let step = match c.thing {
                None => c.local,
                Some(j) => poses[j].compose(&c.local).compose(&poses[j].inverse()),
            };
            Placed { at: step.compose(&poses[c.seed]), ..c.template.clone() }
        })
        .collect()
}

/// The faces of a part like the one an anchor is on, as replicate looks for them: a cylinder of the same radius with its
/// axis the same way, a ball of the same radius, or a flat face facing the same way. Each once, by its key, with the
/// anchor's own place on it, and not the anchor's own face.
fn like_faces(body: &Body, anchor: &Anchor) -> Result<Vec<Anchor>> {
    let Anchor::Face { key, point } = anchor else {
        return Err(Error::InvalidInput("a replicate that finds its own places needs its connector on a face".into()));
    };
    let own = coreshape_document::find_one(body, key)?[0];
    let keys = coreshape_document::face_keys(body);
    let same = |a: f64, b: f64| (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0);
    let normal = |f: usize, s: &coreshape::Surface| match s {
        coreshape::Surface::Plane(frame) => {
            if body.faces[f].flipped {
                -frame.z
            } else {
                frame.z
            }
        }
        _ => Vec3::ZERO,
    };
    let base = &body.faces[own].surface;
    let mut out: Vec<Anchor> = Vec::new();
    let mut seen = vec![keys[own].text()];
    for (f, face) in body.faces.iter().enumerate() {
        let alike = match (base, &face.surface) {
            (coreshape::Surface::Cylinder { frame: a, radius: ra }, coreshape::Surface::Cylinder { frame: b, radius: rb }) => same(*ra, *rb) && a.z.cross(b.z).norm() < 1e-9,
            (coreshape::Surface::Sphere { radius: ra, .. }, coreshape::Surface::Sphere { radius: rb, .. }) => same(*ra, *rb),
            (coreshape::Surface::Plane(_), coreshape::Surface::Plane(_)) => normal(own, base).dot(normal(f, &face.surface)) > 1.0 - 1e-9,
            _ => false,
        };
        let text = keys[f].text();
        if alike && !seen.contains(&text) {
            seen.push(text);
            out.push(Anchor::Face { key: keys[f].clone(), point: *point });
        }
    }
    Ok(out)
}

/// Where a walk down the instances has got to.
struct Walk<'a> {
    top: &'a Assembly,
    prefix: Vec<u32>,
    parent: Frame,
    top_at: Frame,
    /// The locked sub-assembly this is inside, if any, and where it is.
    locked: Option<(Vec<u32>, Frame)>,
}

/// Whether a frame has barely moved from another.
fn same_place(a: &Frame, b: &Frame) -> bool {
    a.origin.dist(b.origin) < 1e-11 && a.x.dist(b.x) < 1e-12 && a.y.dist(b.y) < 1e-12
}

impl Project {
    /// One empty Part Studio, as a plain document has.
    pub fn new() -> Project {
        Project::from_document(Document::new())
    }

    pub fn from_document(doc: Document) -> Project {
        Project { tabs: vec![Tab { id: TabId(1), name: "Part Studio 1".to_string(), content: TabContent::PartStudio(doc) }], next_id: 2 }
    }

    pub fn tab(&self, id: TabId) -> Result<&Tab> {
        self.tabs.iter().find(|t| t.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no tab {}", id.0)))
    }

    pub fn tab_mut(&mut self, id: TabId) -> Result<&mut Tab> {
        self.tabs.iter_mut().find(|t| t.id == id).ok_or_else(|| Error::InvalidInput(format!("there's no tab {}", id.0)))
    }

    pub fn studio(&self, id: TabId) -> Result<&Document> {
        match &self.tab(id)?.content {
            TabContent::PartStudio(doc) => Ok(doc),
            TabContent::Assembly(_) => Err(Error::InvalidInput(format!("tab {} is an assembly, not a Part Studio", id.0))),
            TabContent::Drawing(_) => Err(Error::InvalidInput(format!("tab {} is a drawing, not a Part Studio", id.0))),
        }
    }

    pub fn studio_mut(&mut self, id: TabId) -> Result<&mut Document> {
        match &mut self.tab_mut(id)?.content {
            TabContent::PartStudio(doc) => Ok(doc),
            TabContent::Assembly(_) => Err(Error::InvalidInput(format!("tab {} is an assembly, not a Part Studio", id.0))),
            TabContent::Drawing(_) => Err(Error::InvalidInput(format!("tab {} is a drawing, not a Part Studio", id.0))),
        }
    }

    pub fn assembly(&self, id: TabId) -> Result<&Assembly> {
        match &self.tab(id)?.content {
            TabContent::Assembly(a) => Ok(a),
            TabContent::PartStudio(_) => Err(Error::InvalidInput(format!("tab {} is a Part Studio, not an assembly", id.0))),
            TabContent::Drawing(_) => Err(Error::InvalidInput(format!("tab {} is a drawing, not an assembly", id.0))),
        }
    }

    pub fn assembly_mut(&mut self, id: TabId) -> Result<&mut Assembly> {
        match &mut self.tab_mut(id)?.content {
            TabContent::Assembly(a) => Ok(a),
            TabContent::PartStudio(_) => Err(Error::InvalidInput(format!("tab {} is a Part Studio, not an assembly", id.0))),
            TabContent::Drawing(_) => Err(Error::InvalidInput(format!("tab {} is a drawing, not an assembly", id.0))),
        }
    }

    pub fn drawing(&self, id: TabId) -> Result<&DrawingTab> {
        match &self.tab(id)?.content {
            TabContent::Drawing(d) => Ok(d),
            _ => Err(Error::InvalidInput(format!("tab {} isn't a drawing", id.0))),
        }
    }

    pub fn drawing_mut(&mut self, id: TabId) -> Result<&mut DrawingTab> {
        match &mut self.tab_mut(id)?.content {
            TabContent::Drawing(d) => Ok(d),
            _ => Err(Error::InvalidInput(format!("tab {} isn't a drawing", id.0))),
        }
    }

    /// Adds a drawing of a Part Studio's parts or an assembly. What it shows has to be here, as the right kind of tab.
    pub fn add_drawing(&mut self, name: &str, source: DrawingSource, drawing: coreshape_drawing::Drawing) -> Result<TabId> {
        match &source {
            DrawingSource::Part { studio, .. } => {
                self.studio(*studio)?;
            }
            DrawingSource::Assembly { tab } => {
                self.assembly(*tab)?;
            }
        }
        drawing.size()?;
        Ok(self.add_tab(name, TabContent::Drawing(DrawingTab { source, drawing })))
    }

    /// What a drawing shows, as one body, made now.
    pub fn drawing_body(&mut self, id: TabId) -> Result<Body> {
        let source = self.drawing(id)?.source.clone();
        match source {
            DrawingSource::Part { studio, body } => {
                let outcome = self.studio_mut(studio)?.rebuild(None);
                match body {
                    Some(b) => Ok(outcome.find_body(b).ok_or_else(|| Error::InvalidInput(format!("the drawing's part {} isn't in tab {} now", b.text(), studio.0)))?.body.as_ref().clone()),
                    None => outcome.body.map(|b| b.as_ref().clone()).ok_or_else(|| Error::InvalidInput(format!("tab {} has no parts to draw", studio.0))),
                }
            }
            DrawingSource::Assembly { tab } => self.solve(tab)?.whole().ok_or_else(|| Error::InvalidInput(format!("assembly {} has no parts to draw", tab.0))),
        }
    }

    /// A drawing laid out on its sheet, from its source as it is now. A dimension's point given as a reference is looked up
    /// in its Part Studio; a drawing of an assembly takes only places.
    pub fn drawing_page(&mut self, id: TabId) -> Result<coreshape_drawing::Page> {
        let body = self.drawing_body(id)?;
        let tab = self.drawing(id)?.clone();
        let studio = match tab.source {
            DrawingSource::Part { studio, .. } => Some(studio),
            DrawingSource::Assembly { .. } => None,
        };
        let cell = std::cell::RefCell::new(&mut *self);
        let locate = |v: &Value| -> Result<Vec3> {
            let Some(studio) = studio else {
                return Err(Error::InvalidInput("a drawing of an assembly takes its dimensions' points as places, [x, y, z]".into()));
            };
            let r = coreshape_document::read_point_ref(v)?;
            cell.borrow_mut().studio_mut(studio)?.locate_point(&r)
        };
        coreshape_drawing::page(&tab.drawing, &body, &locate)
    }

    pub fn add_studio(&mut self, name: &str) -> TabId {
        self.add_tab(name, TabContent::PartStudio(Document::new()))
    }

    pub fn add_assembly(&mut self, name: &str) -> TabId {
        self.add_tab(name, TabContent::Assembly(Assembly::new()))
    }

    fn add_tab(&mut self, name: &str, content: TabContent) -> TabId {
        let id = TabId(self.next_id);
        self.next_id += 1;
        self.tabs.push(Tab { id, name: name.to_string(), content });
        id
    }

    /// Deletes a tab. Refused while an instance anywhere uses it.
    pub fn delete_tab(&mut self, id: TabId) -> Result<()> {
        self.tab(id)?;
        for t in &self.tabs {
            if let TabContent::Assembly(a) = &t.content {
                let uses = a.instances.iter().any(|i| match &i.source {
                    Source::Part { studio, .. } => *studio == id,
                    Source::Assembly { tab } => *tab == id,
                });
                if uses {
                    return Err(Error::InvalidInput(format!("tab {} can't be deleted while \"{}\" has instances from it", id.0, t.name)));
                }
            }
        }
        for t in &self.tabs {
            if let TabContent::Drawing(d) = &t.content {
                let uses = match &d.source {
                    DrawingSource::Part { studio, .. } => *studio == id,
                    DrawingSource::Assembly { tab } => *tab == id,
                };
                if uses {
                    return Err(Error::InvalidInput(format!("tab {} can't be deleted while the drawing \"{}\" shows it", id.0, t.name)));
                }
            }
        }
        if self.tabs.len() == 1 {
            return Err(Error::InvalidInput("a project needs at least one tab".into()));
        }
        self.tabs.retain(|t| t.id != id);
        Ok(())
    }

    // ----- Solving -----

    /// Everything an assembly brings together, flattened.
    fn flatten(&mut self, id: TabId) -> Result<Flat> {
        self.check_no_loops(id)?;
        let mut flat = Flat::default();
        let top = self.assembly(id)?.clone();
        let mut outcomes = BTreeMap::new();
        let walk = Walk { top: &top, prefix: Vec::new(), parent: Frame::WORLD, top_at: Frame::WORLD, locked: None };
        self.flatten_into(&top, &walk, &mut flat, &mut outcomes)?;
        plan_copies(&top, &mut flat);
        Ok(flat)
    }

    /// Checks that no assembly ends up inside itself, by way of this one.
    pub fn check_no_loops(&self, id: TabId) -> Result<()> {
        fn walk(p: &Project, id: TabId, path: &mut Vec<TabId>) -> Result<()> {
            if path.contains(&id) {
                return Err(Error::InvalidInput(format!("tab {} ends up inside itself", id.0)));
            }
            path.push(id);
            for i in &p.assembly(id)?.instances {
                if let Source::Assembly { tab } = &i.source {
                    walk(p, *tab, path)?;
                }
            }
            path.pop();
            Ok(())
        }
        walk(self, id, &mut Vec::new())
    }

    fn flatten_into(&mut self, here: &Assembly, walk: &Walk, flat: &mut Flat, outcomes: &mut BTreeMap<TabId, coreshape_document::Outcome>) -> Result<()> {
        let is_top = walk.prefix.is_empty();
        for instance in &here.instances {
            if instance.suppressed {
                continue;
            }
            let path: Vec<u32> = walk.prefix.iter().copied().chain([instance.id]).collect();
            let top_at = if is_top { instance.at } else { walk.top_at };
            // Where its parent has it, or where it's got to here. Nothing inside a locked sub-assembly goes anywhere alone.
            let at = match (is_top || walk.locked.is_some(), walk.top.moved.get(&path)) {
                (false, Some(moved)) => top_at.compose(moved),
                _ => walk.parent.compose(&instance.at),
            };
            match &instance.source {
                Source::Part { studio, body } => {
                    if !outcomes.contains_key(studio) {
                        let rebuilt = self.studio_mut(*studio).map_err(|_| Error::InvalidInput(format!("instance \"{}\" comes from tab {}, which isn't a Part Studio", instance.name, studio.0)))?.rebuild(None);
                        if rebuilt.failed.is_some() {
                            flat.problems.push(format!("a feature fails in tab {}, so its parts are as they are up to there", studio.0));
                        }
                        outcomes.insert(*studio, rebuilt);
                    }
                    let found = outcomes[studio].find_body(*body).map(|b| b.body.clone());
                    let Some(found) = found else {
                        flat.problems.push(format!("instance \"{}\" is of part {}, which isn't in its Part Studio now", instance.name, body.text()));
                        continue;
                    };
                    flat.placed.push(Placed { path, name: instance.name.clone(), studio: *studio, part: *body, body: found, at });
                    flat.fixed.push(is_top && instance.fixed);
                    flat.top_at.push(top_at);
                    flat.locked.push(walk.locked.clone());
                }
                Source::Assembly { tab } => {
                    let start = flat.placed.len();
                    // Locked, it's as its own tab has it, whatever has changed since that tab was last solved.
                    if instance.locked && walk.locked.is_none() && !flat.solved_tabs.contains(tab) {
                        self.solve(*tab)?;
                        flat.solved_tabs.push(*tab);
                    }
                    let inner = self.assembly(*tab)?.clone();
                    let locked = walk.locked.clone().or_else(|| instance.locked.then(|| (path.clone(), at)));
                    let next = Walk { top: walk.top, prefix: path, parent: at, top_at, locked };
                    self.flatten_into(&inner, &next, flat, outcomes)?;
                    // Locked, its parts stay as they are in it and move as one.
                    if instance.locked && walk.locked.is_none() && flat.placed.len() - start > 1 {
                        flat.groups.push((start..flat.placed.len()).collect());
                    }
                    if is_top && instance.fixed {
                        for f in &mut flat.fixed[start..] {
                            *f = true;
                        }
                    }
                }
            }
        }
        // Its mates and groups, unless it's inside a locked sub-assembly, where they've had their say already.
        if walk.locked.is_none() {
            self.add_joints(here, walk, is_top, flat);
        }
        Ok(())
    }

    fn add_joints(&self, here: &Assembly, walk: &Walk, is_top: bool, flat: &mut Flat) {
        let prefix = walk.prefix.as_slice();
        let find = |flat: &Flat, inner: &[u32]| -> Option<usize> {
            let path: Vec<u32> = prefix.iter().chain(inner).copied().collect();
            flat.placed.iter().position(|p| p.path == path)
        };
        for group in &here.groups {
            if group.suppressed {
                continue;
            }
            let members: Vec<usize> = group.instances.iter().filter_map(|p| find(flat, p)).collect();
            if members.len() >= 2 {
                flat.groups.push(members);
            }
        }
        // A mate's index among the joints, for relations.
        let mut joint_of: BTreeMap<u32, usize> = BTreeMap::new();
        for mate in &here.mates {
            if mate.suppressed {
                continue;
            }
            let (Some(ca), Some(cb)) = (here.connector(mate.a), here.connector(mate.b)) else { continue };
            let (a, b) = (find(flat, &ca.instance), find(flat, &cb.instance));
            let (Some(a), Some(b)) = (a, b) else {
                if is_top {
                    flat.failing.push((mate.id, "one of its instances isn't there".to_string()));
                }
                continue;
            };
            let frames = connector_frame(ca, &flat.placed[a].body).and_then(|fa| connector_frame(cb, &flat.placed[b].body).map(|fb| (fa, fb)));
            let (fa, fb) = match frames {
                Ok(pair) => pair,
                Err(why) => {
                    if is_top {
                        flat.failing.push((mate.id, why.to_string()));
                    } else {
                        flat.problems.push(format!("mate \"{}\" in a sub-assembly: {why}", mate.name));
                    }
                    continue;
                }
            };
            let mut joint = mate_joint(mate, a, b, fa, fb);
            if mate.kind == MateKind::Tangent {
                match tangent_joint(mate, a, b, ca, cb, flat) {
                    Ok(j) => joint = j,
                    Err(why) => {
                        if is_top {
                            flat.failing.push((mate.id, why.to_string()));
                        } else {
                            flat.problems.push(format!("mate \"{}\" in a sub-assembly: {why}", mate.name));
                        }
                        continue;
                    }
                }
            }
            // How far a sub-assembly's mate has turned here is kept here, and its own tab's says nothing about that.
            if let Some(turned) = walk.top.moved_turns.get(&(prefix.to_vec(), mate.id)) {
                joint.turned = *turned;
            }
            joint_of.insert(mate.id, flat.joints.len());
            flat.joints.push(joint);
            flat.joint_mates.push((mate.id, prefix.to_vec()));
        }
        for relation in &here.relations {
            if relation.suppressed {
                continue;
            }
            let (Some(&ja), Some(&jb)) = (joint_of.get(&relation.a), joint_of.get(&relation.b)) else { continue };
            let slide = |j: usize| if flat.joints[j].kind.values().contains(&Dof::Z) { Dof::Z } else { Dof::X };
            let sign = if relation.reverse { -1.0 } else { 1.0 };
            let (a, b, ratio) = match relation.kind {
                RelationKind::Gear { ratio } => ((ja, Dof::Angle), (jb, Dof::Angle), sign * ratio),
                RelationKind::RackAndPinion { distance } => ((ja, Dof::Angle), (jb, slide(jb)), sign * distance / TAU),
                RelationKind::Screw { lead } => ((ja, Dof::Angle), (ja, Dof::Z), sign * lead / TAU),
                RelationKind::Linear { ratio } => ((ja, slide(ja)), (jb, slide(jb)), sign * ratio),
            };
            flat.couplings.push(Coupling { a, b, ratio, offset: relation.offset });
            flat.coupling_relations.push(relation.id);
        }
    }

    /// Adds what a solve found's copies to it, once the rest is where it goes.
    fn with_copies(flat: &mut Flat) {
        let poses: Vec<Frame> = flat.placed.iter().map(|p| p.at).collect();
        let copies = place_copies(&flat.copies, &poses);
        flat.placed.extend(copies);
    }

    fn problem_of(flat: &Flat) -> Problem {
        Problem {
            poses: flat.placed.iter().map(|p| p.at).collect(),
            fixed: flat.fixed.clone(),
            groups: flat.groups.clone(),
            joints: flat.joints.clone(),
            couplings: flat.couplings.clone(),
            drives: Vec::new(),
            prefer: Vec::new(),
            extent: flat.placed.iter().map(|p| p.body.aabb().diagonal()).fold(0.0, f64::max),
        }
    }

    /// Saves what a solve found back into the assembly: where each instance is, and how far each mate has turned.
    fn write_back(&mut self, id: TabId, flat: &mut Flat, problem: &Problem) -> Result<()> {
        let a = self.assembly_mut(id)?;
        let mut keep = |path: &[u32], place: Frame, top_at: Frame| {
            if let [only] = path {
                if let Some(i) = a.instances.iter_mut().find(|i| i.id == *only) {
                    i.at = place;
                }
            } else {
                a.moved.insert(path.to_vec(), top_at.inverse().compose(&place));
            }
        };
        let mut locked_done: Vec<Vec<u32>> = Vec::new();
        for i in 0..flat.placed.len() {
            let (was, now) = (flat.placed[i].at, problem.poses[i]);
            flat.placed[i].at = now;
            if same_place(&was, &now) {
                continue;
            }
            match &flat.locked[i] {
                // A locked sub-assembly moves as one, so it's that which is kept, and not the parts in it.
                Some((path, place)) => {
                    if !locked_done.contains(path) {
                        locked_done.push(path.clone());
                        keep(path, now.compose(&was.inverse()).compose(place), flat.top_at[i]);
                    }
                }
                None => keep(&flat.placed[i].path, now, flat.top_at[i]),
            }
        }
        for ((joint, was), (mate, prefix)) in problem.joints.iter().zip(&flat.joints).zip(&flat.joint_mates) {
            if prefix.is_empty() {
                if let Ok(m) = a.mate_mut(*mate) {
                    m.turned = joint.turned;
                }
            } else if joint.turned != was.turned {
                a.moved_turns.insert((prefix.clone(), *mate), joint.turned);
            }
        }
        Ok(())
    }

    fn finish(&mut self, id: TabId, mut flat: Flat, problem: &Problem, report: Report, remaining: Option<f64>) -> Result<Solved> {
        self.write_back(id, &mut flat, problem)?;
        Self::with_copies(&mut flat);
        let mut failing = std::mem::take(&mut flat.failing);
        for j in &report.failing {
            let (mate, prefix) = &flat.joint_mates[*j];
            let why = if prefix.is_empty() { "it can't be met along with the others".to_string() } else { format!("it can't be met along with the others, in the sub-assembly at {prefix:?}") };
            failing.push((*mate, why));
        }
        let mut values = Vec::new();
        for (j, joint) in problem.joints.iter().enumerate() {
            if !flat.joint_mates[j].1.is_empty() {
                continue;
            }
            for dof in joint.kind.values() {
                let v = solver::joint_value(problem, j, *dof);
                values.push((flat.joint_mates[j].0, *dof, if *dof == Dof::Angle { v.to_degrees() } else { v }));
            }
        }
        let free = flat.placed.iter().zip(&report.free).filter(|(_, f)| **f).map(|(p, _)| p.path.clone()).collect();
        Ok(Solved {
            failing,
            failing_relations: report.failing_couplings.iter().map(|c| flat.coupling_relations[*c]).collect(),
            problems: std::mem::take(&mut flat.problems),
            freedom: report.freedom,
            free,
            values,
            remaining,
            placed: flat.placed,
        })
    }

    /// The parts of an assembly and where they are now, without solving anything.
    pub fn placed(&mut self, id: TabId) -> Result<Vec<Placed>> {
        let mut flat = self.flatten(id)?;
        Self::with_copies(&mut flat);
        Ok(flat.placed)
    }

    /// The assembly solved, and shown exploded: each of the view's steps moving what it moves, in turn, and `amount`, from
    /// 0 to 1, how far through them. Only what's shown moves: the assembly keeps its places.
    pub fn exploded(&mut self, id: TabId, view: u32, amount: f64) -> Result<Solved> {
        let x = self.assembly(id)?.explodes.iter().find(|x| x.id == view).cloned().ok_or_else(|| Error::InvalidInput(format!("there's no exploded view {view}")))?;
        if !amount.is_finite() {
            return Err(Error::InvalidInput("how far to explode is a number from 0 to 1".into()));
        }
        let mut solved = self.solve(id)?;
        let n = x.steps.len() as f64;
        for (k, step) in x.steps.iter().enumerate() {
            let share = (amount.clamp(0.0, 1.0) * n - k as f64).clamp(0.0, 1.0);
            if share == 0.0 {
                continue;
            }
            let mut moved = Frame::translation(step.shift * share);
            if let Some((point, axis, degrees)) = step.turn {
                moved = moved.compose(&Frame::rotation(point, axis.normalized(), (degrees * share).to_radians()));
            }
            for p in &mut solved.placed {
                if step.instances.iter().any(|path| p.path.starts_with(path)) {
                    p.at = moved.compose(&p.at);
                }
            }
        }
        Ok(solved)
    }

    /// Keeps where every top-level mate is now as a named position, to go back to.
    pub fn save_position(&mut self, id: TabId, name: Option<String>) -> Result<u32> {
        let solved = self.solve(id)?;
        let a = self.assembly_mut(id)?;
        let pid = a.fresh_id();
        let n = a.positions.len() + 1;
        a.positions.push(Position { id: pid, name: name.unwrap_or_else(|| format!("Position {n}")), values: solved.values.clone() });
        Ok(pid)
    }

    /// Takes the mates back to a named position, all together, a little at a time so that a mechanism stays on its branch.
    pub fn go_to_position(&mut self, id: TabId, position: u32) -> Result<Solved> {
        let p = self.assembly(id)?.positions.iter().find(|p| p.id == position).cloned().ok_or_else(|| Error::InvalidInput(format!("there's no named position {position}")))?;
        self.set_mate_values(id, &p.values)
    }

    /// Takes several mates' motions to values together, as `set_mate_value` takes one: in steps, stopping where the other
    /// mates stop them. A value for a mate that isn't working here is left out, and said.
    pub fn set_mate_values(&mut self, id: TabId, wanted: &[(u32, Dof, f64)]) -> Result<Solved> {
        let mut flat = self.flatten(id)?;
        let mut targets: Vec<(usize, Dof, f64)> = Vec::new();
        for &(mate, dof, value) in wanted {
            let joint = flat.joint_mates.iter().position(|(m, p)| *m == mate && p.is_empty());
            match joint {
                Some(j) if flat.joints[j].kind.values().contains(&dof) => targets.push((j, dof, if dof == Dof::Angle { value.to_radians() } else { value })),
                _ => flat.problems.push(format!("mate {mate} isn't here to take along {} any more", crate::dof_name(dof))),
            }
        }
        let mut problem = Self::problem_of(&flat);
        let first = solver::solve(&mut problem);
        if targets.iter().any(|t| first.failing.contains(&t.0)) || targets.is_empty() {
            return self.finish(id, flat, &problem, first, None);
        }
        let starts: Vec<f64> = targets.iter().map(|t| solver::joint_value(&problem, t.0, t.1)).collect();
        let size = solver::problem_size(&problem);
        let count = targets
            .iter()
            .zip(&starts)
            .map(|(t, s)| {
                let span = (t.2 - s).abs();
                if t.1 == Dof::Angle {
                    (span / (TAU / 24.0)).ceil()
                } else {
                    (span / (size / 24.0).max(1e-9)).ceil()
                }
            })
            .fold(1.0, f64::max)
            .clamp(1.0, 2000.0) as usize;
        let (known, known_relations) = (first.failing.clone(), first.failing_couplings.clone());
        let holds = |step: &Report| step.failing.iter().all(|f| known.contains(f)) && step.failing_couplings.iter().all(|f| known_relations.contains(f));
        let drives_at = |share: f64| -> Vec<Drive> { targets.iter().zip(&starts).map(|(t, s)| Drive { joint: t.0, dof: t.1, value: s + (t.2 - s) * share }).collect() };
        let mut report = first;
        let mut reached = 0.0;
        for k in 1..=count {
            let share = k as f64 / count as f64;
            let before = problem.clone();
            problem.drives = drives_at(share);
            let step = solver::solve(&mut problem);
            problem.drives.clear();
            if holds(&step) {
                report = step;
                reached = share;
                continue;
            }
            problem = before;
            let (mut good, mut bad) = (reached, share);
            for _ in 0..STOP_HALVINGS {
                let middle = (good + bad) / 2.0;
                let before = problem.clone();
                problem.drives = drives_at(middle);
                let step = solver::solve(&mut problem);
                problem.drives.clear();
                if holds(&step) {
                    report = step;
                    good = middle;
                } else {
                    problem = before;
                    bad = middle;
                }
            }
            break;
        }
        problem.drives.clear();
        self.finish(id, flat, &problem, report, None)
    }

    /// Solves an assembly, moving instances as little as it can to meet the mates, and saving where they end up.
    pub fn solve(&mut self, id: TabId) -> Result<Solved> {
        self.solve_preferring(id, &[])
    }

    /// As `solve`, with these instances doing the moving if there's a choice, as the first end of a new mate does.
    pub fn solve_preferring(&mut self, id: TabId, prefer: &[u32]) -> Result<Solved> {
        let flat = self.flatten(id)?;
        let mut problem = Self::problem_of(&flat);
        problem.prefer = flat.placed.iter().enumerate().filter(|(_, p)| prefer.contains(&p.path[0])).map(|(i, _)| i).collect();
        let report = solver::solve(&mut problem);
        self.finish(id, flat, &problem, report, None)
    }

    /// Solves after adding a mate. Its first end does the moving if there's a choice. A mate that leaves a turn free doesn't
    /// say where it should stop, so a part that this is the only mate of comes to rest with the mate at zero, its connectors
    /// lined up, and not at some angle left over from how it got there. Anything else stays where solving puts it.
    pub fn solve_new_mate(&mut self, id: TabId, mate: u32, prefer: &[u32]) -> Result<Solved> {
        let solved = self.solve_preferring(id, prefer)?;
        let flat = self.flatten(id)?;
        let Some(joint) = flat.joint_mates.iter().position(|(m, p)| *m == mate && p.is_empty()) else { return Ok(solved) };
        let (a, b) = (flat.joints[joint].a, flat.joints[joint].b);
        if !flat.joints[joint].kind.values().contains(&Dof::Angle) {
            return Ok(solved);
        }
        // Only the end that moves, with nothing else holding it, and nothing moving with it that isn't free to.
        let mover = if prefer.contains(&flat.placed[a].path[0]) { a } else { b };
        let alone = flat.joints.iter().filter(|j| j.a == mover || j.b == mover).count() == 1 && !flat.groups.iter().any(|g| g.contains(&mover)) && !flat.fixed[mover];
        if !alone {
            return Ok(solved);
        }
        let mut problem = Self::problem_of(&flat);
        problem.prefer = vec![mover];
        problem.drives = vec![Drive { joint, dof: Dof::Angle, value: 0.0 }];
        let report = solver::solve(&mut problem);
        if !report.failing.is_empty() {
            return Ok(solved);
        }
        problem.drives.clear();
        problem.joints[joint].turned = 0.0;
        let report = solver::solve(&mut problem);
        self.finish(id, flat, &problem, report, None)
    }

    /// Moves an instance's part toward where it's wanted, as far as the mates let it. `path` says which, and a point on
    /// it, in its part's coordinates, is what goes to `to`.
    pub fn drag_point(&mut self, id: TabId, path: &[u32], local: Vec3, to: Vec3) -> Result<Solved> {
        let flat = self.flatten(id)?;
        let thing = flat.placed.iter().position(|p| p.path == path).ok_or_else(|| Error::InvalidInput(format!("there's no part at {path:?}")))?;
        self.drag(id, flat, Goal::Point { thing, local, to })
    }

    /// Moves an instance's part toward a whole new place, as far as the mates let it.
    pub fn drag_pose(&mut self, id: TabId, path: &[u32], to: Frame) -> Result<Solved> {
        let flat = self.flatten(id)?;
        let thing = flat.placed.iter().position(|p| p.path == path).ok_or_else(|| Error::InvalidInput(format!("there's no part at {path:?}")))?;
        self.drag(id, flat, Goal::Pose { thing, to })
    }

    fn drag(&mut self, id: TabId, flat: Flat, goal: Goal) -> Result<Solved> {
        let mut problem = Self::problem_of(&flat);
        let dragged = solver::drag(&mut problem, &goal);
        self.finish(id, flat, &problem, dragged.report, Some(dragged.remaining))
    }

    /// Takes a mate's motion to a value, a little at a time so that a mechanism stays on its branch, stopping where the
    /// other mates stop it. Lengths in millimetres, and angles in degrees, with their turns. Nothing holds it there
    /// afterward: a mate's value is only where it is, as in Onshape.
    pub fn set_mate_value(&mut self, id: TabId, mate: u32, dof: Dof, value: f64) -> Result<Solved> {
        let flat = self.flatten(id)?;
        let joint = flat.joint_mates.iter().position(|(m, p)| *m == mate && p.is_empty()).ok_or_else(|| Error::InvalidInput(format!("there's no mate {mate} working here")))?;
        if !flat.joints[joint].kind.values().contains(&dof) {
            return Err(Error::InvalidInput(format!("mate {mate} can't move along {}", crate::dof_name(dof))));
        }
        let mut problem = Self::problem_of(&flat);
        let first = solver::solve(&mut problem);
        if first.failing.contains(&joint) {
            return self.finish(id, flat, &problem, first, None);
        }
        let want = if dof == Dof::Angle { value.to_radians() } else { value };
        let start = solver::joint_value(&problem, joint, dof);
        // Steps of a twelfth of a turn, or the same share of the distance's own size.
        let span = (want - start).abs();
        let count = if dof == Dof::Angle { (span / (TAU / 24.0)).ceil() } else { (span / (solver::problem_size(&problem) / 24.0).max(1e-9)).ceil() }.clamp(1.0, 2000.0) as usize;
        // A mate that was failing already, elsewhere, says nothing about whether this one can go on. One that starts to, or a
        // relation that does, is what stops it: left to itself the solver would drop a gear to get where it's told.
        let (known, known_relations) = (first.failing.clone(), first.failing_couplings.clone());
        let holds = |step: &Report| step.failing.iter().all(|f| known.contains(f)) && step.failing_couplings.iter().all(|f| known_relations.contains(f));
        let mut report = first;
        let mut reached = start;
        for k in 1..=count {
            let at = start + (want - start) * k as f64 / count as f64;
            let before = problem.clone();
            problem.drives = vec![Drive { joint, dof, value: at }];
            let step = solver::solve(&mut problem);
            problem.drives.clear();
            if holds(&step) {
                report = step;
                reached = at;
                continue;
            }
            // As far as it goes, which is somewhere between the last that worked and this.
            problem = before;
            let (mut good, mut bad) = (reached, at);
            for _ in 0..STOP_HALVINGS {
                let middle = (good + bad) / 2.0;
                let before = problem.clone();
                problem.drives = vec![Drive { joint, dof, value: middle }];
                let step = solver::solve(&mut problem);
                problem.drives.clear();
                if holds(&step) {
                    report = step;
                    good = middle;
                } else {
                    problem = before;
                    bad = middle;
                }
            }
            break;
        }
        problem.drives.clear();
        self.finish(id, flat, &problem, report, None)
    }

    /// Ties two mates together as they are now: `offset` is set so that nothing moves when the relation is added.
    pub fn settle_relation(&mut self, id: TabId, relation: u32) -> Result<()> {
        let flat = self.flatten(id)?;
        let mut problem = Self::problem_of(&flat);
        problem.couplings.clear();
        solver::solve(&mut problem);
        let r = self.assembly(id)?.relations.iter().find(|r| r.id == relation).cloned().ok_or_else(|| Error::InvalidInput(format!("there's no relation {relation}")))?;
        let joint_of = |m: u32| flat.joint_mates.iter().position(|(mate, p)| *mate == m && p.is_empty());
        let (Some(ja), Some(jb)) = (joint_of(r.a), joint_of(r.b)) else { return Ok(()) };
        let slide = |j: usize| if flat.joints[j].kind.values().contains(&Dof::Z) { Dof::Z } else { Dof::X };
        let sign = if r.reverse { -1.0 } else { 1.0 };
        let (da, db, ratio) = match r.kind {
            RelationKind::Gear { ratio } => (Dof::Angle, Dof::Angle, sign * ratio),
            RelationKind::RackAndPinion { distance } => (Dof::Angle, slide(jb), sign * distance / TAU),
            RelationKind::Screw { lead } => (Dof::Angle, Dof::Z, sign * lead / TAU),
            RelationKind::Linear { ratio } => (slide(ja), slide(jb), sign * ratio),
        };
        let offset = solver::joint_value(&problem, jb, db) - ratio * solver::joint_value(&problem, ja, da);
        if let Some(entry) = self.assembly_mut(id)?.relations.iter_mut().find(|x| x.id == relation) {
            entry.offset = offset;
        }
        Ok(())
    }

    /// Which parts are in the same place as each other, and any pairs it couldn't tell about. Parts that only touch don't count.
    pub fn interferences(&mut self, id: TabId) -> Result<(Vec<Interference>, Vec<String>)> {
        let solved = self.solve(id)?;
        let bodies: Vec<Body> = solved.placed.iter().map(|p| p.body.transformed(&p.at)).collect();
        let boxes: Vec<Aabb> = bodies.iter().map(Body::aabb).collect();
        // A sheet has no inside, so nothing can be in the same place as one. It's not a question for it.
        let meet = |a: &Aabb, b: &Aabb| a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z;
        let (mut found, mut unknown) = (Vec::new(), Vec::new());
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                if !meet(&boxes[i], &boxes[j]) || bodies[i].sheet || bodies[j].sheet {
                    continue;
                }
                match coreshape::boolean::intersect(&bodies[i], &bodies[j]) {
                    Ok(common) => {
                        let volume = if common.faces.is_empty() { 0.0 } else { coreshape::mass_properties(&common).volume };
                        if volume > 1e-9 {
                            found.push(Interference { a: solved.placed[i].path.clone(), b: solved.placed[j].path.clone(), volume });
                        }
                    }
                    Err(why) => unknown.push(format!("\"{}\" and \"{}\": {why}", solved.placed[i].name, solved.placed[j].name)),
                }
            }
        }
        Ok((found, unknown))
    }

    /// How many of each part there are.
    pub fn bom(&mut self, id: TabId) -> Result<Bom> {
        let solved = self.solve(id)?;
        let mut rows: Vec<BomRow> = Vec::new();
        for p in &solved.placed {
            if let Some(row) = rows.iter_mut().find(|r| r.studio == p.studio && r.part == p.part) {
                row.count += 1;
                continue;
            }
            let props = coreshape::mass_properties(&p.body);
            let name = p.name.split(" <").next().unwrap_or(&p.name).to_string();
            rows.push(BomRow { name, studio: p.studio, part: p.part, count: 1, volume: props.volume, area: props.area });
        }
        Ok(Bom { rows })
    }

    /// The assembly as a STEP assembly: each distinct part once, with a use of it for each instance.
    pub fn step_text(&mut self, id: TabId, name: &str) -> Result<String> {
        let solved = self.solve(id)?;
        let mut parts: Vec<(TabId, BodyId, String, Arc<Body>)> = Vec::new();
        let mut uses: Vec<(usize, String, Frame)> = Vec::new();
        for p in &solved.placed {
            let at = match parts.iter().position(|q| q.0 == p.studio && q.1 == p.part) {
                Some(at) => at,
                None => {
                    parts.push((p.studio, p.part, p.name.split(" <").next().unwrap_or(&p.name).to_string(), p.body.clone()));
                    parts.len() - 1
                }
            };
            uses.push((at, p.name.clone(), p.at));
        }
        let list: Vec<(&str, &Body)> = parts.iter().map(|q| (q.2.as_str(), q.3.as_ref())).collect();
        let instances: Vec<(usize, &str, Frame)> = uses.iter().map(|u| (u.0, u.1.as_str(), u.2)).collect();
        Ok(coreshape::export::step::write_step_assembly(&list, &instances, name))
    }

    /// Brings a STEP file in as a Part Studio of its distinct parts and an assembly of their uses, each held where the
    /// file has it. A file with one part used once comes in the same way.
    pub fn import_step(&mut self, text: &str, name: &str) -> Result<(TabId, TabId)> {
        let structure = coreshape::import::step::read_step_structure(text)?;
        let studio = self.add_studio(name);
        let feature = self.studio_mut(studio)?.add(Op::ImportStepParts { text: text.to_string() })?;
        let outcome = self.studio_mut(studio)?.rebuild(None);
        if let Some(i) = outcome.failed {
            let why = match &outcome.steps[i].state {
                coreshape_document::StepState::Failed(w) => w.clone(),
                _ => "it couldn't be read".to_string(),
            };
            self.tabs.retain(|t| t.id != studio);
            return Err(Error::InvalidInput(why));
        }
        let assembly = self.add_assembly(&format!("{name} assembly"));
        let a = self.assembly_mut(assembly)?;
        let mut counts = vec![0usize; structure.names.len()];
        for (part, at) in &structure.instances {
            counts[*part] += 1;
            let source = Source::Part { studio, body: BodyId { feature, index: *part as u32 } };
            let id = a.add_instance(source, format!("{} <{}>", structure.names[*part], counts[*part]), *at);
            a.instance_mut(id)?.fixed = true;
        }
        Ok((studio, assembly))
    }

    // ----- Saved form -----

    pub fn to_json(&self) -> Value {
        let tabs = self.tabs.iter().map(|t| {
            let mut items = vec![("id", Value::Num(t.id.0 as f64)), ("name", Value::str(&t.name))];
            match &t.content {
                TabContent::PartStudio(doc) => {
                    items.push(("type", Value::str("part_studio")));
                    items.push(("document", doc.to_json()));
                }
                TabContent::Assembly(a) => {
                    items.push(("type", Value::str("assembly")));
                    items.push(("assembly", a.to_json()));
                }
                TabContent::Drawing(d) => {
                    items.push(("type", Value::str("drawing")));
                    let source = match &d.source {
                        DrawingSource::Part { studio, body } => {
                            let mut s = vec![("tab", Value::Num(studio.0 as f64))];
                            if let Some(b) = body {
                                s.push(("body", Value::str(b.text())));
                            }
                            Value::obj(s)
                        }
                        DrawingSource::Assembly { tab } => Value::obj(vec![("assembly", Value::Num(tab.0 as f64))]),
                    };
                    items.push(("source", source));
                    items.push(("drawing", d.drawing.to_json()));
                }
            }
            Value::obj(items)
        });
        Value::obj(vec![("format", Value::str(FORMAT)), ("version", Value::Num(FORMAT_VERSION)), ("next_id", Value::Num(self.next_id as f64)), ("tabs", Value::Arr(tabs.collect()))])
    }

    /// The project as the text of a file. One lone Part Studio is written as a plain document, as it always was.
    pub fn save_text(&self) -> String {
        match self.tabs.as_slice() {
            [Tab { content: TabContent::PartStudio(doc), .. }] => doc.save_text(),
            _ => self.to_json().to_pretty(),
        }
    }

    /// Reads a project, or a plain document as a project of one Part Studio.
    pub fn load_text(text: &str) -> Result<Project> {
        let v = json::parse(text).map_err(|e| Error::InvalidInput(format!("not valid JSON: {e}")))?;
        if v.get("format").and_then(Value::as_str) != Some(FORMAT) {
            return Ok(Project::from_document(Document::from_json(&v)?));
        }
        let version = v.get("version").and_then(Value::as_f64).ok_or_else(|| Error::InvalidInput("the project has no version".into()))?;
        if version != FORMAT_VERSION {
            return Err(Error::InvalidInput(format!("this project is format version {version}, and this kernel reads version {FORMAT_VERSION}")));
        }
        let mut tabs = Vec::new();
        for t in v.get("tabs").and_then(Value::as_array).unwrap_or(&[]) {
            let id = t.get("id").and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).ok_or_else(|| Error::InvalidInput("a tab's id must be a whole number".into()))? as u32;
            let name = t.get("name").and_then(Value::as_str).unwrap_or("").to_string();
            let content = match t.get("type").and_then(Value::as_str).unwrap_or("") {
                "part_studio" => TabContent::PartStudio(Document::from_json(t.get("document").ok_or_else(|| Error::InvalidInput("a Part Studio needs a document".into()))?)?),
                "assembly" => TabContent::Assembly(Assembly::from_json(t.get("assembly").ok_or_else(|| Error::InvalidInput("an assembly tab needs an assembly".into()))?)?),
                "drawing" => {
                    let s = t.get("source").ok_or_else(|| Error::InvalidInput("a drawing tab needs a source".into()))?;
                    let tab_id = |x: Option<&Value>| x.and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).map(|n| TabId(n as u32)).ok_or_else(|| Error::InvalidInput("a drawing's source names a tab by its id".into()));
                    let source = if s.get("assembly").is_some() {
                        DrawingSource::Assembly { tab: tab_id(s.get("assembly"))? }
                    } else {
                        let body = s.get("body").and_then(Value::as_str).map(BodyId::parse).transpose()?;
                        DrawingSource::Part { studio: tab_id(s.get("tab"))?, body }
                    };
                    let drawing = coreshape_drawing::Drawing::from_json(t.get("drawing").ok_or_else(|| Error::InvalidInput("a drawing tab needs a drawing".into()))?)?;
                    TabContent::Drawing(DrawingTab { source, drawing })
                }
                other => return Err(Error::InvalidInput(format!("unknown kind of tab \"{other}\"; expected part_studio, assembly or drawing"))),
            };
            if tabs.iter().any(|x: &Tab| x.id == TabId(id)) {
                return Err(Error::InvalidInput(format!("tab {id} appears twice")));
            }
            tabs.push(Tab { id: TabId(id), name, content });
        }
        if tabs.is_empty() {
            return Err(Error::InvalidInput("a project needs at least one tab".into()));
        }
        // Every tab an instance uses has to be here, as the right kind.
        for t in &tabs {
            if let TabContent::Assembly(a) = &t.content {
                for i in &a.instances {
                    let (want, is_right): (TabId, fn(&TabContent) -> bool) = match &i.source {
                        Source::Part { studio, .. } => (*studio, |c| matches!(c, TabContent::PartStudio(_))),
                        Source::Assembly { tab } => (*tab, |c| matches!(c, TabContent::Assembly(_))),
                    };
                    if !tabs.iter().any(|x| x.id == want && is_right(&x.content)) {
                        return Err(Error::InvalidInput(format!("instance \"{}\" comes from tab {}, which isn't here as that kind of tab", i.name, want.0)));
                    }
                }
            }
        }
        for t in &tabs {
            if let TabContent::Drawing(d) = &t.content {
                let fine = match &d.source {
                    DrawingSource::Part { studio, .. } => tabs.iter().any(|x| x.id == *studio && matches!(x.content, TabContent::PartStudio(_))),
                    DrawingSource::Assembly { tab } => tabs.iter().any(|x| x.id == *tab && matches!(x.content, TabContent::Assembly(_))),
                };
                if !fine {
                    return Err(Error::InvalidInput(format!("the drawing \"{}\" shows a tab that isn't here as that kind of tab", t.name)));
                }
            }
        }
        let top = tabs.iter().map(|t| t.id.0).max().unwrap_or(0);
        let saved = v.get("next_id").and_then(Value::as_f64).filter(|n| *n >= 0.0 && *n < 4e9).unwrap_or(0.0) as u32;
        Ok(Project { tabs, next_id: (top + 1).max(saved) })
    }
}

impl Default for Project {
    fn default() -> Project {
        Project::new()
    }
}

/// A mate as a joint between two things: its frames in their own coordinates, with its own adjustments.
fn mate_joint(mate: &Mate, a: usize, b: usize, fa: Frame, fb: Frame) -> Joint {
    let shift = Frame::translation(v3(mate.offset[0], mate.offset[1], mate.offset[2]));
    let turn = Frame::rotation(Vec3::ZERO, Vec3::Z, mate.rotation.to_radians());
    let on_a = fa.compose(&shift).compose(&turn);
    let quarter = Frame::rotation(Vec3::ZERO, Vec3::Z, mate.quarter_turns as f64 * FRAC_PI_2);
    let flipped = if mate.flip { Frame::rotation(Vec3::ZERO, Vec3::X, PI) } else { Frame::WORLD };
    let on_b = fb.compose(&quarter).compose(&flipped);
    let limits = mate
        .limits
        .iter()
        .map(|l| {
            let k = if l.dof == Dof::Angle { 1f64.to_radians() } else { 1.0 };
            Limit { dof: l.dof, min: l.min * k, max: l.max * k }
        })
        .collect();
    Joint { a, b, on_a, on_b, kind: mate.kind, limits, turned: mate.turned, contact: None }
}

/// A tangent mate as a joint: the shapes of the faces its connectors are on, put in order, planes first. The other part
/// goes on the outside of a flat face, the side its normal points to, and round faces go outside each other; `flip`
/// takes the other side of the plane, or one inside the other. That's fixed by the mate, and not by where things are,
/// so that solving again never changes it.
fn tangent_joint(mate: &Mate, a: usize, b: usize, ca: &Connector, cb: &Connector, flat: &Flat) -> Result<Joint> {
    let (fa, ta) = ca.anchor.touching(&flat.placed[a].body)?;
    let (fb, tb) = cb.anchor.touching(&flat.placed[b].body)?;
    let (a, b, fa, fb, ta, tb) = if ta.rank() > tb.rank() { (b, a, fb, fa, tb, ta) } else { (a, b, fa, fb, ta, tb) };
    let side = if mate.flip { -1.0 } else { 1.0 };
    // Two planes touch facing each other: the second turned over, so that lining up their z axes does that.
    let fb = if matches!((ta, tb), (Touching::Plane, Touching::Plane)) { fb.compose(&Frame::rotation(Vec3::ZERO, Vec3::X, PI)) } else { fb };
    Ok(Joint { a, b, on_a: fa, on_b: fb, kind: MateKind::Tangent, limits: Vec::new(), turned: 0.0, contact: Some(Contact { a: ta, b: tb, side }) })
}

/// The copies an assembly's patterns and replicates make, of what's flattened, to place once it's solved.
fn plan_copies(top: &Assembly, flat: &mut Flat) {
    let index_of = |flat: &Flat, path: &[u32]| flat.placed.iter().position(|p| p.path == path);
    let seeds = |flat: &Flat, of: &[u32]| -> Vec<usize> { (0..flat.placed.len()).filter(|&i| of.contains(&flat.placed[i].path[0])).collect() };
    let mut plans = Vec::new();
    for p in top.patterns.iter().filter(|p| !p.suppressed) {
        let axis = match &p.axis {
            PatternAxis::Given { point, direction } => Some((Frame::new(*point, direction.normalized(), direction.normalized().any_perpendicular()), None)),
            PatternAxis::Connector(c) => top.connector(*c).and_then(|c| {
                let j = index_of(flat, &c.instance)?;
                connector_frame(c, &flat.placed[j].body).ok().map(|f| (f, Some(j)))
            }),
        };
        let Some((frame, thing)) = axis else {
            flat.problems.push(format!("pattern \"{}\": its axis isn't there", p.name));
            continue;
        };
        for k in 1..p.count {
            let step = match p.kind {
                PatternKind::Linear { spacing } => Frame::translation(v3(0.0, 0.0, spacing * k as f64)),
                PatternKind::Circular { degrees } => Frame::rotation(Vec3::ZERO, Vec3::Z, (degrees * k as f64).to_radians()),
            };
            let local = frame.compose(&step).compose(&frame.inverse());
            for s in seeds(flat, &p.of) {
                plans.push(copy_plan(&flat.placed[s], s, p.id, k, &p.name, thing, local));
            }
        }
    }
    for r in top.replicates.iter().filter(|r| !r.suppressed) {
        let Some(c) = top.connector(r.connector) else { continue };
        let Some(j) = index_of(flat, &c.instance) else {
            flat.problems.push(format!("replicate \"{}\": the instance its connector is on isn't there", r.name));
            continue;
        };
        let body = flat.placed[j].body.clone();
        let from = match connector_frame(c, &body) {
            Ok(f) => f,
            Err(why) => {
                flat.problems.push(format!("replicate \"{}\": {why}", r.name));
                continue;
            }
        };
        let anchors = if r.at.is_empty() {
            match like_faces(&body, &c.anchor) {
                Ok(list) => list,
                Err(why) => {
                    flat.problems.push(format!("replicate \"{}\": {why}", r.name));
                    continue;
                }
            }
        } else {
            r.at.clone()
        };
        let mut k = 0;
        for anchor in anchors {
            let place = Connector { anchor, ..c.clone() };
            let to = match connector_frame(&place, &body) {
                Ok(f) => f,
                Err(why) => {
                    flat.problems.push(format!("replicate \"{}\": {why}", r.name));
                    continue;
                }
            };
            // The same place as the connector's own isn't another.
            if to.origin.dist(from.origin) < 1e-9 * body.aabb().diagonal().max(1.0) {
                continue;
            }
            // Pointing the way the connector does, as a hole found from the other end would point the other way.
            let to = if to.z.dot(from.z) < 0.0 { to.compose(&Frame::rotation(Vec3::ZERO, Vec3::X, PI)) } else { to };
            let local = to.compose(&from.inverse());
            k += 1;
            for s in seeds(flat, &r.of) {
                plans.push(copy_plan(&flat.placed[s], s, r.id, k, &r.name, Some(j), local));
            }
        }
    }
    flat.copies = plans;
}

fn copy_plan(seed: &Placed, at: usize, record: u32, k: u32, label: &str, thing: Option<usize>, local: Frame) -> CopyPlan {
    let mut template = seed.clone();
    template.path = [record, k].into_iter().chain(seed.path.iter().copied()).collect();
    template.name = format!("{} <{label} {}>", seed.name.split(" <").next().unwrap_or(&seed.name), k + 1);
    CopyPlan { template, seed: at, thing, local }
}
