//! The tools for tabs and assemblies. The Part Studio being worked on stays in the session's own document, as it always was,
//! so the tools for features don't change, and it goes back among the tabs for whatever needs them all.

use super::*;
use coreshape::{Frame, Vec3};
use coreshape_assembly::connector::edge_between;
use coreshape_assembly::*;

/// Every tool here, in the same form as the others.
pub(super) const TOOLS: &[(&str, &str, &str, &str)] = &[
    ("tab_list", "The project's tabs, Part Studios and assemblies, with their ids, and which is selected.", "{}", "[]"),
    ("tab_new", "Adds a Part Studio or an assembly, and selects it.", r#"{"type": {"type": "string", "enum": ["part_studio", "assembly"]}, "name": {"type": "string"}}"#, r#"["type"]"#),
    ("tab_select", "Selects a tab. The tools for features work on the last Part Studio selected, and the tools for assemblies, render and export on the assembly if one is selected.", r#"{"tab": {"type": "integer"}}"#, r#"["tab"]"#),
    ("tab_rename", "Renames a tab.", r#"{"tab": {"type": "integer"}, "name": {"type": "string"}}"#, r#"["tab", "name"]"#),
    ("tab_delete", "Deletes a tab. Refused while an instance uses it, or if it's the last Part Studio.", r#"{"tab": {"type": "integer"}}"#, r#"["tab"]"#),
    ("project_show", "The whole project as JSON, in the file format.", "{}", "[]"),
    ("instance_add", "Adds a part or a whole assembly to the selected assembly, as an instance of it, so that it changes when its source does. It starts where its source is, unless told otherwise.", r#"{"part": {"type": "string", "description": "A part, like f3 or f3.2, from bodies_list."}, "studio": {"type": "integer", "description": "The Part Studio's tab. By default the one being worked on."}, "assembly": {"type": "integer", "description": "An assembly's tab, in place of a part."}, "name": {"type": "string"}, "at": {"type": "object", "description": "origin, z and x, as lists of three numbers."}, "fixed": {"type": "boolean"}}"#, "[]"),
    ("instance_edit", "Changes an instance: its name, where it is, whether it's fixed or suppressed, and for a sub-assembly whether it's locked, which keeps its parts as they are in its own tab. Then solves.", r#"{"id": {"type": "integer"}, "name": {"type": "string"}, "at": {"type": "object"}, "fixed": {"type": "boolean"}, "suppressed": {"type": "boolean"}, "locked": {"type": "boolean"}}"#, r#"["id"]"#),
    ("instance_delete", "Deletes an instance, with its connectors and the mates and groups that use it. Then solves.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("connector_candidates", "The places to put a mate connector on a face or an edge of an instance: a face's centroid or axis and its corners, edge middles and arc centers. Each comes with an \"on\" to pass to connector_add or mate_add. Face names are in faces_list.", r#"{"instance": {"description": "An instance's id, or a list of ids to reach into a sub-assembly."}, "face": {"type": "string"}, "edge": {"type": "array", "items": {"type": "string"}, "description": "The two faces it's between."}}"#, r#"["instance"]"#),
    ("connector_add", "Adds a mate connector to an instance. Found from a face or an edge, it follows when the part changes. Its z is the primary axis: out of a flat face, or along an axis.", r#"{"instance": {"description": "An instance's id, or a list of ids to reach into a sub-assembly."}, "on": {"type": "object", "description": "From connector_candidates, or {\"frame\": {origin, z, x}} in the part's coordinates."}, "offset": {"type": "array"}, "rotation": {"type": "number", "description": "Degrees about its z."}, "flip": {"type": "boolean"}, "quarter_turns": {"type": "integer"}}"#, r#"["instance", "on"]"#),
    ("connector_edit", "Changes a mate connector, then solves.", r#"{"id": {"type": "integer"}, "on": {"type": "object"}, "offset": {"type": "array"}, "rotation": {"type": "number"}, "flip": {"type": "boolean"}, "quarter_turns": {"type": "integer"}}"#, r#"["id"]"#),
    ("connector_delete", "Deletes a mate connector no mate is using.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("mate_add", "Mates two connectors, and solves, with the first end doing the moving if there's a choice. Their z axes end up pointing the same way, so two faces, whose z axes both point out, need flip to lie against each other. An end can be a connector's id, or {instance, on} to make one for this mate alone. fastened leaves nothing free, revolute a turn about z, slider a slide along z, cylindrical both, pin_slot a turn and a slide along x, planar a turn and slides in x and y, ball any turn, parallel a turn about z and any slide.", r#"{"type": {"type": "string", "enum": ["fastened", "revolute", "slider", "cylindrical", "pin_slot", "planar", "ball", "parallel"]}, "a": {}, "b": {}, "name": {"type": "string"}, "offset": {"type": "array", "description": "From a to b, along a's axes."}, "rotation": {"type": "number", "description": "Degrees about a's z."}, "flip": {"type": "boolean"}, "quarter_turns": {"type": "integer"}, "limits": {"type": "array", "description": "Each {dof, min, max}. dof is x, y, z or angle. Millimetres, and degrees."}}"#, r#"["type", "a", "b"]"#),
    ("mate_edit", "Changes a mate, then solves.", r#"{"id": {"type": "integer"}, "type": {"type": "string"}, "name": {"type": "string"}, "offset": {"type": "array"}, "rotation": {"type": "number"}, "flip": {"type": "boolean"}, "quarter_turns": {"type": "integer"}, "limits": {"type": "array"}, "suppressed": {"type": "boolean"}}"#, r#"["id"]"#),
    ("mate_delete", "Deletes a mate, with any relations on it and the connectors made only for it. Then solves.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("mate_set_value", "Takes a mate to a value, a little at a time so that a mechanism stays on its branch, stopping where other mates stop it. Nothing holds it there afterward.", r#"{"id": {"type": "integer"}, "dof": {"type": "string", "enum": ["x", "y", "z", "angle"]}, "value": {"type": "number", "description": "Millimetres, or degrees with their turns."}}"#, r#"["id", "dof", "value"]"#),
    ("group_add", "Makes instances move as one, without any mates. Then solves.", r#"{"instances": {"type": "array", "description": "Ids, or lists of ids to reach into a sub-assembly."}}"#, r#"["instances"]"#),
    ("group_delete", "Deletes a group. Then solves.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("relation_add", "Ties one mate's motion to another's as they are now, so adding it moves nothing. gear: two mates that turn, with a ratio. rack_and_pinion: one that turns and one that slides, with a distance for each whole turn. screw: one cylindrical mate, as both, with a lead for each whole turn. linear: two that slide, with a ratio.", r#"{"type": {"type": "string", "enum": ["gear", "rack_and_pinion", "screw", "linear"]}, "a": {"type": "integer"}, "b": {"type": "integer", "description": "Leave out for a screw."}, "ratio": {"type": "number"}, "distance": {"type": "number"}, "lead": {"type": "number"}, "reverse": {"type": "boolean"}}"#, r#"["type", "a"]"#),
    ("relation_delete", "Deletes a relation. Then solves.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("assembly_solve", "Solves the selected assembly, or another by its tab, and gives where every part is, how many ways it can still move, each mate's values, and which mates can't be met and why. A part that has changed brings what's mated to it along.", r#"{"tab": {"type": "integer"}}"#, "[]"),
    ("assembly_drag", "Moves an instance toward where it's wanted, as far as its mates let it. Either a point on it, in its part's coordinates, to a place in the assembly, or its whole pose.", r#"{"instance": {"description": "An instance's id, or a list of ids to reach into a sub-assembly."}, "point": {"type": "array"}, "to": {"type": "array"}, "pose": {"type": "object"}}"#, r#"["instance"]"#),
    ("interference_check", "Which parts in the assembly are in the same place as each other, with the volume they share, and any pair it couldn't tell about. Parts that only touch don't count.", r#"{"tab": {"type": "integer"}}"#, "[]"),
    ("bom", "A bill of materials: each part, how many times it's used however deep, and its volume and area.", r#"{"tab": {"type": "integer"}}"#, "[]"),
    ("assembly_import_step", "Brings in a STEP file as a new Part Studio of its distinct parts and a new assembly of their uses, each fixed where the file has it, and selects the assembly.", r#"{"path": {"type": "string"}, "text": {"type": "string"}, "name": {"type": "string"}}"#, "[]"),
];

fn e(x: coreshape::Error) -> String {
    x.to_string()
}

fn tab_of(v: &Value, key: &str) -> R<TabId> {
    v.as_f64().filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).map(|n| TabId(n as u32)).ok_or_else(|| format!("\"{key}\" must be a tab's id"))
}

fn number_id(args: &Value, key: &str) -> R<u32> {
    args.get(key).and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).map(|n| n as u32).ok_or_else(|| format!("\"{key}\" must be an id"))
}

fn path_of(v: Option<&Value>, key: &str) -> R<Vec<u32>> {
    let one = |x: &Value| x.as_f64().filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).map(|n| n as u32);
    let wrong = || format!("\"{key}\" must be an instance's id, or a list of them");
    match v {
        Some(Value::Num(_)) => Ok(vec![v.and_then(one).ok_or_else(wrong)?]),
        Some(Value::Arr(list)) if !list.is_empty() => list.iter().map(|x| one(x).ok_or_else(wrong)).collect(),
        _ => Err(wrong()),
    }
}

fn exact_frame(f: &Frame) -> Value {
    frame_value(f)
}

/// The adjustments a connector and a mate share.
#[derive(Default)]
struct Adjust {
    offset: Option<[f64; 3]>,
    rotation: Option<f64>,
    flip: Option<bool>,
    quarter_turns: Option<i32>,
}

fn read_adjust(args: &Value) -> R<Adjust> {
    let number = |key: &str| -> R<Option<f64>> {
        match args.get(key) {
            None => Ok(None),
            Some(v) => v.as_f64().filter(|n| n.is_finite()).map(Some).ok_or_else(|| format!("\"{key}\" must be a number")),
        }
    };
    let offset = match args.get("offset") {
        None => None,
        Some(v) => {
            let p = read_vec3(v).map_err(|_| "\"offset\" must be three numbers".to_string())?;
            Some([p.x, p.y, p.z])
        }
    };
    let flip = match args.get("flip") {
        None => None,
        Some(v) => Some(v.as_bool().ok_or("\"flip\" must be true or false")?),
    };
    let quarter_turns = match number("quarter_turns")? {
        None => None,
        Some(n) if n.fract() == 0.0 && n.abs() < 1e6 => Some(n as i32),
        Some(_) => return Err("\"quarter_turns\" must be a whole number".to_string()),
    };
    Ok(Adjust { offset, rotation: number("rotation")?, flip, quarter_turns })
}

fn read_limits(args: &Value) -> R<Option<Vec<MateLimit>>> {
    let Some(list) = args.get("limits") else { return Ok(None) };
    let list = list.as_array().ok_or("\"limits\" must be a list")?;
    let mut out = Vec::new();
    for l in list {
        let dof = read_dof(l.get("dof").and_then(Value::as_str).unwrap_or("")).map_err(e)?;
        let bound = |key: &str, default: f64| -> R<f64> {
            match l.get(key) {
                None => Ok(default),
                Some(v) => v.as_f64().filter(|n| n.is_finite()).ok_or_else(|| format!("a limit's \"{key}\" must be a number")),
            }
        };
        let (min, max) = (bound("min", f64::NEG_INFINITY)?, bound("max", f64::INFINITY)?);
        if min > max {
            return Err(format!("a limit's min of {min} is more than its max of {max}"));
        }
        out.push(MateLimit { dof, min, max });
    }
    Ok(Some(out))
}

fn set_flag(args: &Value, key: &str, slot: &mut bool) -> R<()> {
    if let Some(v) = args.get(key) {
        *slot = v.as_bool().ok_or_else(|| format!("\"{key}\" must be true or false"))?;
    }
    Ok(())
}

impl Session {
    /// The project with all its tabs in place. Whatever `work` does to the Part Studio being worked on is kept.
    pub fn with_project<T>(&mut self, work: impl FnOnce(&mut Project) -> T) -> T {
        self.swap_studio();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| work(&mut self.project)));
        self.swap_studio();
        match result {
            Ok(value) => value,
            Err(cause) => std::panic::resume_unwind(cause),
        }
    }

    fn swap_studio(&mut self) {
        let studio = self.studio;
        if let Ok(slot) = self.project.studio_mut(studio) {
            std::mem::swap(&mut self.doc, slot);
        }
    }

    /// Makes this the Part Studio that the tools for features work on.
    fn work_on_studio(&mut self, id: TabId) -> R<()> {
        self.project.studio(id).map_err(e)?;
        if id != self.studio {
            self.swap_studio();
            self.studio = id;
            self.swap_studio();
            self.outcome = None;
        }
        Ok(())
    }

    /// Replaces everything with this project.
    pub(super) fn replace_project(&mut self, mut project: Project) {
        // There's always a Part Studio to work on.
        let first = project.tabs.iter().find(|t| matches!(t.content, TabContent::PartStudio(_))).map(|t| t.id);
        let studio = first.unwrap_or_else(|| project.add_studio("Part Studio 1"));
        self.project = project;
        self.doc = Document::new();
        self.studio = studio;
        self.active = studio;
        self.outcome = None;
        self.swap_studio();
    }

    /// The tab that's selected.
    pub fn active_tab(&self) -> TabId {
        self.active
    }

    /// Whether what's selected is an assembly.
    pub fn active_tab_is_assembly(&self) -> bool {
        self.active_assembly().is_some()
    }

    /// The selected assembly, if an assembly is what's selected.
    pub(super) fn active_assembly(&self) -> Option<TabId> {
        self.project.assembly(self.active).is_ok().then_some(self.active)
    }

    /// The selected assembly as it solves now, with each part and where it goes, for an app to show.
    pub fn solved(&mut self) -> R<Solved> {
        let tab = self.active_assembly().ok_or("no assembly is selected")?;
        self.with_project(|p| p.solve(tab)).map_err(e)
    }

    fn tab_arg(&self, args: &Value) -> R<TabId> {
        match args.get("tab") {
            Some(v) => tab_of(v, "tab"),
            None => self.active_assembly().ok_or_else(|| "no assembly is selected: tab_new or tab_select one, or give a \"tab\"".to_string()),
        }
    }

    /// What's in a tool's assembly, for changing, and the tab it's in.
    fn asm(&mut self, args: &Value) -> R<(TabId, &mut Assembly)> {
        let tab = self.tab_arg(args)?;
        Ok((tab, self.project.assembly_mut(tab).map_err(e)?))
    }

    /// The state of an assembly, after whatever a tool did to it, as its answer.
    fn state(&self, tab: TabId, solved: &Solved) -> Value {
        let Ok(assembly) = self.project.assembly(tab) else { return Value::Null };
        let instances = solved.placed.iter().map(|p| {
            let free = solved.free.contains(&p.path);
            Value::obj(vec![
                ("instance", Value::Arr(p.path.iter().map(|i| Value::Num(*i as f64)).collect())),
                ("name", Value::str(&p.name)),
                ("tab", Value::Num(p.studio.0 as f64)),
                ("part", Value::str(p.part.text())),
                ("at", exact_frame(&p.at)),
                ("free", Value::Bool(free)),
            ])
        });
        let mates = assembly.mates.iter().map(|m| {
            let mut items = vec![("id", Value::Num(m.id as f64)), ("name", Value::str(&m.name)), ("type", Value::str(kind_name(m.kind)))];
            let values: Vec<(&str, Value)> = solved.values.iter().filter(|v| v.0 == m.id).map(|v| (dof_name(v.1), num(v.2))).collect();
            if !values.is_empty() {
                items.push(("values", Value::obj(values)));
            }
            if m.suppressed {
                items.push(("suppressed", Value::Bool(true)));
            }
            if let Some(f) = solved.failing.iter().find(|f| f.0 == m.id) {
                items.push(("failing", Value::str(&f.1)));
            }
            Value::obj(items)
        });
        let mut items = vec![
            ("tab", Value::Num(tab.0 as f64)),
            ("freedom", Value::Num(solved.freedom as f64)),
            ("instances", Value::Arr(instances.collect())),
            ("mates", Value::Arr(mates.collect())),
        ];
        // A failing mate of a sub-assembly isn't among this one's, and has to be seen as well.
        let others: Vec<Value> = solved.failing.iter().filter(|f| assembly.mate(f.0).is_none()).map(|f| Value::obj(vec![("mate", Value::Num(f.0 as f64)), ("failing", Value::str(&f.1))])).collect();
        if !others.is_empty() {
            items.push(("failing_in_sub_assemblies", Value::Arr(others)));
        }
        if !solved.failing_relations.is_empty() {
            items.push(("failing_relations", Value::Arr(solved.failing_relations.iter().map(|r| Value::Num(*r as f64)).collect())));
        }
        if !solved.problems.is_empty() {
            items.push(("problems", Value::Arr(solved.problems.iter().map(Value::str).collect())));
        }
        if let Some(remaining) = solved.remaining {
            items.push(("remaining", num(remaining)));
        }
        Value::obj(items)
    }

    fn solved_answer(&mut self, tab: TabId, prefer: &[u32], lead: Option<Vec<(&str, Value)>>) -> R<Vec<Value>> {
        let solved = self.with_project(|p| p.solve_preferring(tab, prefer)).map_err(e)?;
        let state = self.state(tab, &solved);
        Ok(vec![text(answer_with(lead, state).to_pretty())])
    }

    /// A part where the assembly has it now, and its body.
    fn part_at(&mut self, tab: TabId, path: &[u32]) -> R<Placed> {
        let placed = self.with_project(|p| p.placed(tab)).map_err(e)?;
        placed.into_iter().find(|q| q.path == path).ok_or_else(|| {
            format!("there's no part at {path:?} here: a mate connector is on a part, so a path into a sub-assembly has to go all the way down to one")
        })
    }

    /// A new connector from a tool's arguments.
    fn new_connector(&mut self, tab: TabId, args: &Value, implicit: bool) -> R<u32> {
        let path = path_of(args.get("instance"), "instance")?;
        let on = Anchor::from_json(args.get("on").ok_or("a connector needs \"on\": connector_candidates gives the places there are")?).map_err(e)?;
        let part = self.part_at(tab, &path)?;
        // One that can't be found isn't worth keeping. Later on it's a mate that says so, as the part changes.
        on.resolve(&part.body).map_err(e)?;
        let adjust = read_adjust(args)?;
        let a = self.project.assembly_mut(tab).map_err(e)?;
        let id = a.add_connector(path, on).map_err(e)?;
        let c = a.connector_mut(id).map_err(e)?;
        apply_to_connector(c, &adjust);
        c.implicit = implicit;
        Ok(id)
    }

    /// An end of a new mate: a connector's id, or what to make one from. It says if it made one.
    fn mate_end(&mut self, tab: TabId, v: &Value) -> R<(u32, bool)> {
        match v {
            Value::Num(_) => {
                let id = v.as_f64().filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).ok_or("a mate's end must be a connector's id, or {instance, on}")? as u32;
                Ok((id, false))
            }
            _ => Ok((self.new_connector(tab, v, true)?, true)),
        }
    }

    pub(super) fn call_assembly(&mut self, name: &str, args: &Value) -> Option<R<Vec<Value>>> {
        if !TOOLS.iter().any(|t| t.0 == name) {
            return None;
        }
        Some(self.assembly_tool(name, args))
    }

    fn assembly_tool(&mut self, name: &str, args: &Value) -> R<Vec<Value>> {
        match name {
            "tab_list" => Ok(vec![text(self.tabs_value().to_pretty())]),
            "tab_new" => {
                let kind = string_arg(args, "type")?;
                let given = args.get("name").and_then(Value::as_str);
                let count = self.project.tabs.len() + 1;
                let id = match kind {
                    "part_studio" => {
                        let id = self.project.add_studio(given.unwrap_or(&format!("Part Studio {count}")));
                        self.work_on_studio(id)?;
                        id
                    }
                    "assembly" => self.project.add_assembly(given.unwrap_or(&format!("Assembly {count}"))),
                    other => return Err(format!("unknown kind of tab \"{other}\"; expected part_studio or assembly")),
                };
                self.active = id;
                Ok(vec![text(self.tabs_value().to_pretty())])
            }
            "tab_select" => {
                let id = tab_of(args.get("tab").ok_or("\"tab\" is needed")?, "tab")?;
                self.project.tab(id).map_err(e)?;
                if self.project.studio(id).is_ok() {
                    self.work_on_studio(id)?;
                }
                self.active = id;
                Ok(vec![text(self.tabs_value().to_pretty())])
            }
            "tab_rename" => {
                let id = tab_of(args.get("tab").ok_or("\"tab\" is needed")?, "tab")?;
                let new = string_arg(args, "name")?.to_string();
                self.project.tab_mut(id).map_err(e)?.name = new;
                Ok(vec![text(self.tabs_value().to_pretty())])
            }
            "tab_delete" => {
                let id = tab_of(args.get("tab").ok_or("\"tab\" is needed")?, "tab")?;
                let is_studio = self.project.studio(id).is_ok();
                let studios = self.project.tabs.iter().filter(|t| matches!(t.content, TabContent::PartStudio(_))).count();
                if is_studio && studios == 1 {
                    return Err("a project needs a Part Studio, and this is the last one".to_string());
                }
                // In use or not, this says so before anything moves.
                let mut trial = self.project.clone();
                trial.delete_tab(id).map_err(e)?;
                if id == self.studio {
                    let other = self.project.tabs.iter().find(|t| t.id != id && matches!(t.content, TabContent::PartStudio(_))).map(|t| t.id).ok_or("there's no other Part Studio to work on")?;
                    self.work_on_studio(other)?;
                }
                self.project.delete_tab(id).map_err(e)?;
                if self.active == id {
                    self.active = self.studio;
                }
                Ok(vec![text(self.tabs_value().to_pretty())])
            }
            "project_show" => Ok(vec![text(self.with_project(|p| p.to_json().to_pretty()))]),
            "instance_add" => self.instance_add(args),
            "instance_edit" => {
                let (tab, a) = self.asm(args)?;
                let id = number_id(args, "id")?;
                let at = args.get("at").map(read_frame).transpose().map_err(e)?;
                let instance = a.instance_mut(id).map_err(e)?;
                if let Some(new) = args.get("name") {
                    instance.name = new.as_str().ok_or("\"name\" must be text")?.to_string();
                }
                if let Some(place) = at {
                    instance.at = place;
                }
                set_flag(args, "fixed", &mut instance.fixed)?;
                set_flag(args, "suppressed", &mut instance.suppressed)?;
                set_flag(args, "locked", &mut instance.locked)?;
                if instance.locked && matches!(instance.source, Source::Part { .. }) {
                    instance.locked = false;
                    return Err("only a sub-assembly can be locked: a part always moves as one".to_string());
                }
                self.solved_answer(tab, &[], None)
            }
            "instance_delete" => {
                let (tab, a) = self.asm(args)?;
                a.delete_instance(number_id(args, "id")?).map_err(e)?;
                self.solved_answer(tab, &[], None)
            }
            "connector_candidates" => self.connector_candidates(args),
            "connector_add" => {
                let tab = self.tab_arg(args)?;
                let id = self.new_connector(tab, args, false)?;
                let at = self.connector_place(tab, id)?;
                Ok(vec![text(Value::obj(vec![("id", Value::Num(id as f64)), ("at", exact_frame(&at))]).to_pretty())])
            }
            "connector_edit" => {
                let tab = self.tab_arg(args)?;
                let id = number_id(args, "id")?;
                let adjust = read_adjust(args)?;
                let on = args.get("on").map(Anchor::from_json).transpose().map_err(e)?;
                let path = self.project.assembly(tab).map_err(e)?.connector(id).ok_or_else(|| format!("there's no connector {id}"))?.instance.clone();
                if let Some(anchor) = &on {
                    anchor.resolve(&self.part_at(tab, &path)?.body).map_err(e)?;
                }
                let c = self.project.assembly_mut(tab).map_err(e)?.connector_mut(id).map_err(e)?;
                apply_to_connector(c, &adjust);
                if let Some(anchor) = on {
                    c.anchor = anchor;
                }
                self.solved_answer(tab, &[], None)
            }
            "connector_delete" => {
                let (_, a) = self.asm(args)?;
                let id = number_id(args, "id")?;
                let using: Vec<String> = a.mates.iter().filter(|m| m.a == id || m.b == id).map(|m| m.name.clone()).collect();
                if !using.is_empty() {
                    return Err(format!("connector {id} is used by {}", using.join(", ")));
                }
                a.connector(id).ok_or_else(|| format!("there's no connector {id}"))?;
                a.connectors.retain(|c| c.id != id);
                Ok(vec![text("Deleted.")])
            }
            "mate_add" => self.mate_add(args),
            "mate_edit" => {
                let (tab, a) = self.asm(args)?;
                let adjust = read_adjust(args)?;
                let limits = read_limits(args)?;
                let m = a.mate_mut(number_id(args, "id")?).map_err(e)?;
                if let Some(kind) = args.get("type") {
                    m.kind = read_kind(kind.as_str().ok_or("\"type\" must be text")?).map_err(e)?;
                }
                if let Some(new) = args.get("name") {
                    m.name = new.as_str().ok_or("\"name\" must be text")?.to_string();
                }
                apply_to_mate(m, &adjust);
                if let Some(list) = limits {
                    m.limits = list;
                }
                set_flag(args, "suppressed", &mut m.suppressed)?;
                self.solved_answer(tab, &[], None)
            }
            "mate_delete" => {
                let (tab, a) = self.asm(args)?;
                a.delete_mate(number_id(args, "id")?).map_err(e)?;
                self.solved_answer(tab, &[], None)
            }
            "mate_set_value" => {
                let tab = self.tab_arg(args)?;
                let id = number_id(args, "id")?;
                let dof = read_dof(string_arg(args, "dof")?).map_err(e)?;
                let value = args.get("value").and_then(Value::as_f64).filter(|v| v.is_finite()).ok_or("\"value\" must be a number")?;
                let solved = self.with_project(|p| p.set_mate_value(tab, id, dof, value)).map_err(e)?;
                let reached = solved.values.iter().find(|v| v.0 == id && v.1 == dof).map_or(f64::NAN, |v| v.2);
                let state = self.state(tab, &solved);
                Ok(vec![text(answer_with(Some(vec![("reached", num(reached))]), state).to_pretty())])
            }
            "group_add" => {
                let (tab, a) = self.asm(args)?;
                let list = args.get("instances").and_then(Value::as_array).ok_or("\"instances\" must be a list")?;
                let paths = list.iter().map(|p| path_of(Some(p), "instances")).collect::<R<Vec<_>>>()?;
                let id = a.add_group(paths).map_err(e)?;
                self.solved_answer(tab, &[], Some(vec![("id", Value::Num(id as f64))]))
            }
            "group_delete" => {
                let (tab, a) = self.asm(args)?;
                let id = number_id(args, "id")?;
                if !a.groups.iter().any(|g| g.id == id) {
                    return Err(format!("there's no group {id}"));
                }
                a.groups.retain(|g| g.id != id);
                self.solved_answer(tab, &[], None)
            }
            "relation_add" => {
                let (tab, a) = self.asm(args)?;
                let kind = read_relation_kind(args).map_err(e)?;
                let first = number_id(args, "a")?;
                let second = if args.get("b").is_some() { number_id(args, "b")? } else { first };
                let mut reverse = false;
                set_flag(args, "reverse", &mut reverse)?;
                let id = a.add_relation(kind, first, second, reverse).map_err(e)?;
                let settled = self.with_project(|p| p.settle_relation(tab, id));
                if let Err(why) = settled {
                    if let Ok(a) = self.project.assembly_mut(tab) {
                        a.relations.retain(|r| r.id != id);
                    }
                    return Err(e(why));
                }
                self.solved_answer(tab, &[], Some(vec![("id", Value::Num(id as f64))]))
            }
            "relation_delete" => {
                let (tab, a) = self.asm(args)?;
                let id = number_id(args, "id")?;
                if !a.relations.iter().any(|r| r.id == id) {
                    return Err(format!("there's no relation {id}"));
                }
                a.relations.retain(|r| r.id != id);
                self.solved_answer(tab, &[], None)
            }
            "assembly_solve" => {
                let tab = self.tab_arg(args)?;
                self.solved_answer(tab, &[], None)
            }
            "assembly_drag" => {
                let tab = self.tab_arg(args)?;
                let path = path_of(args.get("instance"), "instance")?;
                let solved = match (args.get("pose"), args.get("to")) {
                    (Some(pose), None) => {
                        let to = read_frame(pose).map_err(e)?;
                        self.with_project(|p| p.drag_pose(tab, &path, to)).map_err(e)?
                    }
                    (None, Some(to)) => {
                        let to = read_vec3(to).map_err(|_| "\"to\" must be three numbers".to_string())?;
                        let local = args.get("point").map_or(Ok(Vec3::ZERO), |v| read_vec3(v).map_err(|_| "\"point\" must be three numbers".to_string()))?;
                        self.with_project(|p| p.drag_point(tab, &path, local, to)).map_err(e)?
                    }
                    _ => return Err("give either \"to\", with a \"point\" on it, or a whole \"pose\"".to_string()),
                };
                let state = self.state(tab, &solved);
                Ok(vec![text(state.to_pretty())])
            }
            "interference_check" => {
                let tab = self.tab_arg(args)?;
                let (found, unknown) = self.with_project(|p| p.interferences(tab)).map_err(e)?;
                let path = |p: &[u32]| Value::Arr(p.iter().map(|i| Value::Num(*i as f64)).collect());
                let list = found.iter().map(|f| Value::obj(vec![("a", path(&f.a)), ("b", path(&f.b)), ("volume", num(f.volume))])).collect();
                let unknown = unknown.iter().map(Value::str).collect();
                Ok(vec![text(Value::obj(vec![("interferences", Value::Arr(list)), ("couldnt_tell", Value::Arr(unknown))]).to_pretty())])
            }
            "bom" => {
                let tab = self.tab_arg(args)?;
                let bom = self.with_project(|p| p.bom(tab)).map_err(e)?;
                let rows = bom.rows.iter().map(|r| {
                    Value::obj(vec![
                        ("name", Value::str(&r.name)),
                        ("tab", Value::Num(r.studio.0 as f64)),
                        ("part", Value::str(r.part.text())),
                        ("count", Value::Num(r.count as f64)),
                        ("volume", num(r.volume)),
                        ("area", num(r.area)),
                    ])
                });
                Ok(vec![text(Value::Arr(rows.collect()).to_pretty())])
            }
            "assembly_import_step" => {
                let contents = match (args.get("path"), args.get("text")) {
                    (Some(_), Some(_)) => return Err("give a path or text, not both".to_string()),
                    (Some(p), None) => {
                        let path = self.inside_root(p.as_str().ok_or("\"path\" must be text")?)?;
                        std::fs::read_to_string(&path).map_err(|why| format!("can't read {}: {why}", path.display()))?
                    }
                    (None, Some(t)) => t.as_str().ok_or("\"text\" must be text")?.to_string(),
                    (None, None) => return Err("give a path or text".to_string()),
                };
                let label = args.get("name").and_then(Value::as_str).unwrap_or("Imported").to_string();
                let (_, assembly) = self.with_project(|p| p.import_step(&contents, &label)).map_err(e)?;
                self.active = assembly;
                self.solved_answer(assembly, &[], None)
            }
            other => Err(format!("there is no tool named \"{other}\"")),
        }
    }

    fn tabs_value(&self) -> Value {
        Value::Arr(
            self.project
                .tabs
                .iter()
                .map(|t| {
                    let kind = if matches!(t.content, TabContent::PartStudio(_)) { "part_studio" } else { "assembly" };
                    let mut items = vec![("id", Value::Num(t.id.0 as f64)), ("name", Value::str(&t.name)), ("type", Value::str(kind))];
                    if t.id == self.active {
                        items.push(("selected", Value::Bool(true)));
                    }
                    Value::obj(items)
                })
                .collect(),
        )
    }

    fn instance_add(&mut self, args: &Value) -> R<Vec<Value>> {
        let tab = self.tab_arg(args)?;
        self.project.assembly(tab).map_err(e)?;
        let at = args.get("at").map(read_frame).transpose().map_err(e)?.unwrap_or(Frame::WORLD);
        let (source, base) = match (args.get("assembly"), args.get("part")) {
            (Some(_), Some(_)) => return Err("give a part or an assembly, not both".to_string()),
            (Some(inner), None) => {
                let inner = tab_of(inner, "assembly")?;
                self.project.assembly(inner).map_err(e)?;
                (Source::Assembly { tab: inner }, self.project.tab(inner).map_err(e)?.name.clone())
            }
            (None, Some(part)) => {
                let id = BodyId::parse(part.as_str().ok_or("\"part\" must be the name of a part, like f3 or f3.2")?).map_err(e)?;
                let studio = match args.get("studio") {
                    Some(v) => tab_of(v, "studio")?,
                    None => self.studio,
                };
                let maker = self.with_project(|p| -> R<String> {
                    let doc = p.studio_mut(studio).map_err(e)?;
                    let o = doc.rebuild(None);
                    if o.find_body(id).is_none() {
                        let there: Vec<String> = o.bodies.iter().map(|b| b.id.text()).collect();
                        return Err(format!("tab {} has no part {}: there's {}", studio.0, id.text(), if there.is_empty() { "none yet".to_string() } else { there.join(", ") }));
                    }
                    Ok(doc.feature(id.feature).map_or_else(|| "Part".to_string(), |f| f.name.clone()))
                })?;
                (Source::Part { studio, body: id }, maker)
            }
            (None, None) => return Err("give a \"part\", or an \"assembly\"".to_string()),
        };
        let a = self.project.assembly_mut(tab).map_err(e)?;
        let count = a.instances.iter().filter(|i| i.source == source).count() + 1;
        let label = args.get("name").and_then(Value::as_str).map_or_else(|| format!("{base} <{count}>"), str::to_string);
        let id = a.add_instance(source, label, at);
        let mut fixed = false;
        set_flag(args, "fixed", &mut fixed)?;
        a.instance_mut(id).map_err(e)?.fixed = fixed;
        if let Err(why) = self.project.check_no_loops(tab) {
            if let Ok(a) = self.project.assembly_mut(tab) {
                a.instances.retain(|i| i.id != id);
            }
            return Err(e(why));
        }
        self.solved_answer(tab, &[], Some(vec![("id", Value::Num(id as f64))]))
    }

    fn connector_place(&mut self, tab: TabId, id: u32) -> R<Frame> {
        let c = self.project.assembly(tab).map_err(e)?.connector(id).ok_or_else(|| format!("there's no connector {id}"))?.clone();
        let part = self.part_at(tab, &c.instance)?;
        Ok(part.at.compose(&coreshape_assembly::project::connector_frame(&c, &part.body).map_err(e)?))
    }

    fn connector_candidates(&mut self, args: &Value) -> R<Vec<Value>> {
        let tab = self.tab_arg(args)?;
        let path = path_of(args.get("instance"), "instance")?;
        let part = self.part_at(tab, &path)?;
        let list = match (args.get("face"), args.get("edge")) {
            (Some(face), None) => {
                let name = face.as_str().ok_or("\"face\" must be a face's name")?;
                let key = FaceKey::parse(name).map_err(e)?;
                let found = find_faces(&part.body, &key);
                let first = found.first().ok_or_else(|| format!("this part has no face {name}: faces_list gives the names"))?;
                candidates(&part.body, *first, &key)
            }
            (None, Some(edge)) => {
                let names = edge.as_array().filter(|l| l.len() == 2).ok_or("\"edge\" must be the names of its two faces")?;
                let key = |v: &Value| FaceKey::parse(v.as_str().ok_or("\"edge\" must be the names of its two faces")?).map_err(e);
                let faces = [key(&names[0])?, key(&names[1])?];
                let found = edge_between(&part.body, &faces).ok_or_else(|| format!("this part has no edge between {} and {}", faces[0].text(), faces[1].text()))?;
                edge_candidates(&part.body, found, &faces)
            }
            _ => return Err("give a \"face\" or an \"edge\"".to_string()),
        };
        let rows = list.iter().map(|c| Value::obj(vec![("label", Value::str(&c.label)), ("on", c.anchor.to_json()), ("at", exact_frame(&part.at.compose(&c.frame)))]));
        Ok(vec![text(Value::Arr(rows.collect()).to_pretty())])
    }

    fn mate_add(&mut self, args: &Value) -> R<Vec<Value>> {
        let tab = self.tab_arg(args)?;
        self.project.assembly(tab).map_err(e)?;
        let kind = read_kind(string_arg(args, "type")?).map_err(e)?;
        let adjust = read_adjust(args)?;
        let limits = read_limits(args)?;
        for limit in limits.iter().flatten() {
            if !kind.values().contains(&limit.dof) {
                return Err(format!("a {} mate has no motion along {} to limit", kind_name(kind), dof_name(limit.dof)));
            }
        }
        let (a, a_new) = self.mate_end(tab, args.get("a").ok_or("\"a\" is needed")?)?;
        // Whatever goes wrong from here takes back any connector that was made for this.
        let tidy = |s: &mut Session, list: &[u32]| {
            if let Ok(asm) = s.project.assembly_mut(tab) {
                asm.connectors.retain(|c| !list.contains(&c.id));
            }
        };
        let made_a: Vec<u32> = if a_new { vec![a] } else { Vec::new() };
        let (b, b_new) = match args.get("b").ok_or_else(|| "\"b\" is needed".to_string()).and_then(|v| self.mate_end(tab, v)) {
            Ok(end) => end,
            Err(why) => {
                tidy(self, &made_a);
                return Err(why);
            }
        };
        let mut made = made_a;
        if b_new {
            made.push(b);
        }
        let asm = self.project.assembly_mut(tab).map_err(e)?;
        let id = match asm.add_mate(kind, a, b) {
            Ok(id) => id,
            Err(why) => {
                tidy(self, &made);
                return Err(e(why));
            }
        };
        let first = asm.connector(a).and_then(|c| c.instance.first().copied()).into_iter().collect::<Vec<u32>>();
        let m = asm.mate_mut(id).map_err(e)?;
        apply_to_mate(m, &adjust);
        if let Some(list) = limits {
            m.limits = list;
        }
        if let Some(new) = args.get("name").and_then(Value::as_str) {
            m.name = new.to_string();
        }
        let solved = self.with_project(|p| p.solve_new_mate(tab, id, &first)).map_err(e)?;
        let state = self.state(tab, &solved);
        Ok(vec![text(answer_with(Some(vec![("id", Value::Num(id as f64))]), state).to_pretty())])
    }
}

fn apply_to_connector(c: &mut Connector, adjust: &Adjust) {
    if let Some(v) = adjust.offset {
        c.offset = v;
    }
    if let Some(v) = adjust.rotation {
        c.rotation = v;
    }
    if let Some(v) = adjust.flip {
        c.flip = v;
    }
    if let Some(v) = adjust.quarter_turns {
        c.quarter_turns = v;
    }
}

fn apply_to_mate(m: &mut Mate, adjust: &Adjust) {
    if let Some(v) = adjust.offset {
        m.offset = v;
    }
    if let Some(v) = adjust.rotation {
        m.rotation = v;
    }
    if let Some(v) = adjust.flip {
        m.flip = v;
    }
    if let Some(v) = adjust.quarter_turns {
        m.quarter_turns = v;
    }
}

/// An assembly's state, with something said first.
fn answer_with(lead: Option<Vec<(&str, Value)>>, state: Value) -> Value {
    let Some(mut items) = lead else { return state };
    if let Value::Obj(rest) = &state {
        let rest: Vec<(&str, Value)> = rest.iter().map(|(k, v)| (k.as_str(), v.clone())).collect();
        items.extend(rest);
    }
    Value::obj(items)
}
