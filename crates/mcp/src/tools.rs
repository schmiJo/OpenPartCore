//! The tools themselves, and the state they work on.

mod assembly;
mod mesh;

pub use mesh::{FlatMesh, InstanceMesh};

use coreshape::{mass_properties, tessellate, validate, Body, Curve, Surface, TessParams};
use coreshape_assembly::{Project, TabId};
use coreshape_document::json::{self, Value};
use coreshape_document::*;
use coreshape_render::{render, render_four, Colors, Options, View};
use coreshape_sketch as sk;
use std::path::{Component, Path, PathBuf};
use std::sync::Arc;

const GUIDE: &str = include_str!("../../../docs/FILE_FORMAT.md");

type R<T> = Result<T, String>;

/// Every tool: name, description, and its arguments as a JSON Schema's properties.
const TOOLS: &[(&str, &str, &str, &str)] = &[
    ("format_guide", "The fields of every feature, constraint and edge selection. Read this once before adding features.", "{}", "[]"),
    ("document_new", "Starts an empty document, replacing the one open now.", "{}", "[]"),
    ("document_open", "Opens a .coreshape file under the server's root folder, or from text.", r#"{"path": {"type": "string"}, "text": {"type": "string", "description": "The file's contents, in place of a path."}}"#, "[]"),
    ("document_save", "Saves the document as a .coreshape file under the server's root folder.", r#"{"path": {"type": "string"}}"#, r#"["path"]"#),
    ("document_show", "The whole document as JSON, in the file format.", "{}", "[]"),
    ("parameter_set", "Creates or changes a named parameter, then rebuilds. The value is a number or an expression over other parameters.", r#"{"name": {"type": "string"}, "value": {"type": ["number", "string"]}}"#, r#"["name", "value"]"#),
    ("parameter_remove", "Removes a parameter nothing uses.", r#"{"name": {"type": "string"}}"#, r#"["name"]"#),
    ("feature_add", "Adds a feature, then rebuilds. Gives its permanent id and what the rebuild did. See format_guide for the fields. With several bodies it acts on all of them unless the feature lists \"bodies\", like [\"f3.2\"], and it leaves alone any it doesn't reach. The mode new_body makes bodies of their own, so that parts can touch or overlap.", r#"{"feature": {"type": "object", "description": "A feature, without id: {\"type\": \"extrude\", \"sketch\": 1, \"length\": 10}."}, "name": {"type": "string"}, "position": {"type": "integer", "description": "Where in the history. The end, if left out."}}"#, r#"["feature"]"#),
    ("feature_edit", "Replaces what a feature does, keeping its id, name and position, then rebuilds. It keeps the bodies it acts on unless the feature lists \"bodies\", and an empty list means all of them.", r#"{"id": {"type": "integer"}, "feature": {"type": "object"}}"#, r#"["id", "feature"]"#),
    ("feature_rename", "Renames a feature.", r#"{"id": {"type": "integer"}, "name": {"type": "string"}}"#, r#"["id", "name"]"#),
    ("feature_suppress", "Turns a feature off or on, then rebuilds.", r#"{"id": {"type": "integer"}, "suppressed": {"type": "boolean"}}"#, r#"["id", "suppressed"]"#),
    ("feature_delete", "Deletes a feature nothing refers to, then rebuilds.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("feature_move", "Moves a feature in the history, if that keeps every reference pointing backward, then rebuilds.", r#"{"id": {"type": "integer"}, "position": {"type": "integer"}}"#, r#"["id", "position"]"#),
    ("features_list", "The history: each feature's id, name, type, state and any error.", "{}", "[]"),
    ("rebuild", "Rebuilds, reusing whatever didn't change. Gives each feature's state and the solid's volume, area, size and validity.", r#"{"upto": {"type": "integer", "description": "Stop after the feature at this position, to see an earlier state."}}"#, "[]"),
    ("bodies_list", "The document's bodies: each one's name, the feature that made it, volume, area, faces and size. Separate bodies can touch or overlap, as an assembly's parts do.", "{}", "[]"),
    ("app_set", "Keeps data for an app with a feature, or with the document if no id is given. It's stored as it is and changes nothing in the model, so it's for things like where a dimension's label sits.", r#"{"id": {"type": "integer"}, "app": {"description": "Anything. Leave it out to clear what's there."}}"#, "[]"),
    ("faces_list", "The solid's faces with their stable names, surface type, area, center and outward normal, and the axis of a round one. One name can cover several faces, such as a pattern's copies. Each face's key is its own, and an edge selection takes a key wherever it takes a face name.", r#"{"body": {"type": "string", "description": "One body, like f3 or f3.2. bodies_list gives the names. Leave it out for all of them."}}"#, "[]"),
    ("edges_list", "The solid's edges with their stable names, curve type, length and ends, a selection for each, ready to use in a fillet or chamfer, and each end named as an extrude's up_to_vertex takes it.", r#"{"body": {"type": "string", "description": "One body, like f3 or f3.2. bodies_list gives the names. Leave it out for all of them."}}"#, "[]"),
    ("export", "Exports the solid. STEP and the native format can come back as text; STL needs a path. Paths are under the server's root folder.", r#"{"body": {"type": "string", "description": "One body, like f3 or f3.2. bodies_list gives the names. Leave it out for all of them."}, "format": {"type": "string", "enum": ["step", "stl", "coreshape"]}, "path": {"type": "string"}}"#, r#"["format"]"#),
    ("render", "Draws the solid and returns a PNG. Views: iso, front, back, left, right, top, bottom or four (top, iso, front and right together), or an azimuth and elevation in degrees.", r#"{"body": {"type": "string", "description": "One body, like f3 or f3.2. bodies_list gives the names. Leave it out for all of them."}, "view": {"type": "string"}, "azimuth": {"type": "number"}, "elevation": {"type": "number"}, "width": {"type": "integer"}, "height": {"type": "integer"}, "labels": {"type": "boolean", "description": "Write each visible face's name on it."}, "colors": {"type": "string", "enum": ["plain", "faces"], "description": "faces gives each face its own tint."}, "highlight_faces": {"type": "array", "items": {"type": "string"}, "description": "Face names to pick out."}, "highlight_edges": {"description": "An edge selection to pick out, as in a fillet."}, "path": {"type": "string", "description": "Also save the PNG here, under the server's root folder."}}"#, "[]"),
    ("sketch_solve", "Solves a sketch without changing the document. Gives the solved sketch, remaining freedom, what's still free and what's redundant, or the constraints that can't all hold.", r#"{"sketch": {"type": "object", "description": "A sketch in the format in format_guide."}, "parameters": {"type": "object", "description": "Values for names used in its dimensions."}}"#, r#"["sketch"]"#),
    ("sketch_drag", "Moves a point or edge of a sketch toward a target, disturbing the rest as little as possible, and gives the new sketch.", r#"{"sketch": {"type": "object"}, "grab": {"description": "A point, such as \"7.end\", or an edge and where it was grabbed: {\"edge\": 7, \"at\": [x, y]}."}, "to": {"type": "array", "items": {"type": "number"}}, "parameters": {"type": "object"}}"#, r#"["sketch", "grab", "to"]"#),
    ("sketch_regions", "The closed regions a sketch would use: those picked, or by default every other level of nesting, so a loop in a loop is a hole. Says which entity leaves a gap if there aren't any.", r#"{"sketch": {"type": "object"}}"#, r#"["sketch"]"#),
    ("sketch_arrangement", "Every closed region a sketch's curves make, split at crossings and T-junctions, each with a stable id, plus the curves that make none and why.", r#"{"sketch": {"type": "object"}}"#, r#"["sketch"]"#),
];

pub fn exists(name: &str) -> bool {
    TOOLS.iter().chain(assembly::TOOLS).any(|t| t.0 == name)
}

pub fn list() -> Value {
    Value::Arr(
        TOOLS
            .iter()
            .chain(assembly::TOOLS)
            .map(|(name, description, properties, required)| {
                Value::obj(vec![
                    ("name", Value::str(*name)),
                    ("description", Value::str(*description)),
                    (
                        "inputSchema",
                        Value::obj(vec![
                            ("type", Value::str("object")),
                            ("properties", json::parse(properties).expect("tool schemas are written correctly")),
                            ("required", json::parse(required).expect("tool schemas are written correctly")),
                        ]),
                    ),
                ])
            })
            .collect(),
    )
}

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub fn base64(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let n = (chunk[0] as u32) << 16 | (*chunk.get(1).unwrap_or(&0) as u32) << 8 | *chunk.get(2).unwrap_or(&0) as u32;
        out.push(BASE64[(n >> 18) as usize & 63] as char);
        out.push(BASE64[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 { BASE64[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if chunk.len() > 2 { BASE64[n as usize & 63] as char } else { '=' });
    }
    out
}

fn text(t: impl Into<String>) -> Value {
    Value::obj(vec![("type", Value::str("text")), ("text", Value::str(t.into()))])
}

fn num(v: f64) -> Value {
    // Rounded to a nanometre: enough for any decision, and far shorter to read.
    Value::Num((v * 1e6).round() / 1e6)
}

fn vec3(p: coreshape::Vec3) -> Value {
    Value::Arr(vec![num(p.x), num(p.y), num(p.z)])
}

fn string_arg<'a>(args: &'a Value, key: &str) -> R<&'a str> {
    args.get(key).and_then(Value::as_str).ok_or_else(|| format!("\"{key}\" must be text"))
}

fn id_arg(args: &Value) -> R<FeatureId> {
    args.get("id")
        .and_then(Value::as_f64)
        .filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9)
        .map(|n| FeatureId(n as u32))
        .ok_or_else(|| "\"id\" must be a feature id".to_string())
}

fn position_arg(args: &Value, key: &str) -> R<Option<usize>> {
    match args.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v
            .as_f64()
            .filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 1e9)
            .map(|n| Some(n as usize))
            .ok_or_else(|| format!("\"{key}\" must be a position, from 0")),
    }
}

/// The document being worked on, and the result of its last rebuild.
pub struct Session {
    /// The Part Studio being worked on, kept here rather than among the tabs while it is.
    doc: Document,
    outcome: Option<Outcome>,
    root: PathBuf,
    /// All the tabs, with an empty stand-in for `doc`.
    project: Project,
    /// Which tab `doc` belongs to.
    studio: TabId,
    /// The tab that's selected, which can be an assembly.
    active: TabId,
    /// Meshes worked out already, by body and whether they're draft.
    meshes: Vec<(Arc<Body>, bool, Arc<FlatMesh>)>,
}

impl Session {
    pub fn new(root: PathBuf) -> Session {
        let project = Project::new();
        let studio = project.tabs[0].id;
        Session { doc: Document::new(), outcome: None, root, project, studio, active: studio, meshes: Vec::new() }
    }

    /// The document being worked on.
    pub fn document(&self) -> &Document {
        &self.doc
    }

    /// The result of the last rebuild, rebuilding if there hasn't been one. For showing what the tools have made, without
    /// keeping a second copy of the document. Nothing that's unchanged gets rebuilt.
    pub fn outcome(&mut self) -> &Outcome {
        self.outcome.get_or_insert_with(|| self.doc.rebuild(None))
    }

    /// The document as it was up to a feature, for rolling back a view. That's only for looking, and tools still see all of it.
    pub fn outcome_upto(&mut self, upto: Option<usize>) -> Outcome {
        self.doc.rebuild(upto)
    }

    /// Calls a tool. Whatever happens, the server carries on: a refusal or even a kernel bug comes
    /// back as a failed result, not a dead connection.
    pub fn call_safely(&mut self, name: &str, args: &Value) -> Value {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.call(name, args)));
        let (content, failed) = match result {
            Ok(Ok(content)) => (content, false),
            Ok(Err(why)) => (vec![text(why)], true),
            Err(cause) => {
                let what = cause.downcast_ref::<String>().cloned().or_else(|| cause.downcast_ref::<&str>().map(|s| s.to_string()));
                (vec![text(format!("the kernel hit an internal error, which is a bug: {}", what.unwrap_or_default()))], true)
            }
        };
        Value::obj(vec![("content", Value::Arr(content)), ("isError", Value::Bool(failed))])
    }

    fn call(&mut self, name: &str, args: &Value) -> R<Vec<Value>> {
        let e = |x: coreshape::Error| x.to_string();
        match name {
            "format_guide" => Ok(vec![text(GUIDE)]),
            "document_new" => {
                self.replace_project(Project::new());
                Ok(vec![text("A new, empty document is open.")])
            }
            "document_open" => {
                let contents = match (args.get("path"), args.get("text")) {
                    (Some(_), Some(_)) => return Err("give a path or text, not both".to_string()),
                    (Some(p), None) => {
                        let path = self.inside_root(p.as_str().ok_or("\"path\" must be text")?)?;
                        std::fs::read_to_string(&path).map_err(|why| format!("can't read {}: {why}", path.display()))?
                    }
                    (None, Some(t)) => t.as_str().ok_or("\"text\" must be text")?.to_string(),
                    (None, None) => return Err("give a path or text".to_string()),
                };
                self.replace_project(Project::load_text(&contents).map_err(e)?);
                self.rebuilt(None)
            }
            "document_save" => {
                let path = self.writable(string_arg(args, "path")?, &["coreshape", "json"])?;
                let contents = self.with_project(|p| p.save_text());
                std::fs::write(&path, &contents).map_err(|why| format!("can't write {}: {why}", path.display()))?;
                Ok(vec![text(format!("Saved {} bytes to {}.", contents.len(), path.display()))])
            }
            "document_show" => Ok(vec![text(self.doc.save_text())]),
            "parameter_set" => {
                let value = match args.get("value") {
                    Some(Value::Num(n)) => Expr::number(*n),
                    Some(Value::Str(s)) => Expr::parse(s)?,
                    _ => return Err("\"value\" must be a number or an expression".to_string()),
                };
                self.doc.set_parameter(string_arg(args, "name")?, value).map_err(e)?;
                self.rebuilt(None)
            }
            "parameter_remove" => {
                self.doc.remove_parameter(string_arg(args, "name")?).map_err(e)?;
                Ok(vec![text("Removed.")])
            }
            "feature_add" => {
                let spec = args.get("feature").ok_or("\"feature\" is needed")?;
                let op = read_op(spec, &["bodies", "app"]).map_err(e)?;
                let scope = read_body_list(spec, "bodies").map_err(e)?;
                let at = position_arg(args, "position")?.unwrap_or(self.doc.features().len());
                let name = args.get("name").and_then(Value::as_str).map(str::to_string);
                let id = self.doc.insert(op, name, at).map_err(e)?;
                // It's no use with a body it can't have.
                if let Err(why) = self.doc.set_scope(id, scope) {
                    self.doc.delete(id).map_err(e)?;
                    return Err(why.to_string());
                }
                if let Some(app) = spec.get("app") {
                    self.doc.set_app(Some(id), Some(app.clone())).map_err(e)?;
                }
                let mut out = vec![text(format!("Added feature f{} (id {}).", id.0, id.0))];
                out.extend(self.rebuilt(None)?);
                Ok(out)
            }
            "feature_edit" => {
                let spec = args.get("feature").ok_or("\"feature\" is needed")?;
                let op = read_op(spec, &["bodies", "app"]).map_err(e)?;
                let id = id_arg(args)?;
                let before = self.doc.feature(id).cloned();
                self.doc.edit(id, op).map_err(e)?;
                // What it says about its bodies and its app data changes; what it doesn't say stays.
                if spec.get("bodies").is_some() {
                    let scope = read_body_list(spec, "bodies").map_err(e);
                    if let Err(why) = scope.and_then(|list| self.doc.set_scope(id, list).map_err(e)) {
                        // All of it or none of it.
                        if let Some(old) = before {
                            self.doc.edit(id, old.op).map_err(e)?;
                        }
                        return Err(why);
                    }
                }
                if let Some(app) = spec.get("app") {
                    self.doc.set_app(Some(id), Some(app.clone())).map_err(e)?;
                }
                self.rebuilt(None)
            }
            "feature_rename" => {
                self.doc.rename(id_arg(args)?, string_arg(args, "name")?).map_err(e)?;
                self.outcome = None;
                Ok(vec![text("Renamed.")])
            }
            "feature_suppress" => {
                let on = args.get("suppressed").and_then(Value::as_bool).ok_or("\"suppressed\" must be true or false")?;
                self.doc.suppress(id_arg(args)?, on).map_err(e)?;
                self.rebuilt(None)
            }
            "feature_delete" => {
                self.doc.delete(id_arg(args)?).map_err(e)?;
                self.rebuilt(None)
            }
            "feature_move" => {
                let to = position_arg(args, "position")?.ok_or("\"position\" is needed")?;
                self.doc.reorder(id_arg(args)?, to).map_err(e)?;
                self.rebuilt(None)
            }
            "features_list" => {
                if self.outcome.is_none() {
                    self.outcome = Some(self.doc.rebuild(None));
                }
                Ok(vec![text(self.steps_value().to_pretty())])
            }
            "rebuild" => self.rebuilt(position_arg(args, "upto")?),
            "app_set" => {
                let id = if args.get("id").is_some() { Some(id_arg(args)?) } else { None };
                self.doc.set_app(id, args.get("app").cloned()).map_err(e)?;
                Ok(vec![text("Kept.")])
            }
            "bodies_list" => {
                self.body()?;
                Ok(vec![text(self.bodies_value().to_pretty())])
            }
            "faces_list" => {
                let scope = self.scope(args)?;
                Ok(vec![text(faces_value(&scope.body, &scope.of_face)?.to_pretty())])
            }
            "edges_list" => {
                let scope = self.scope(args)?;
                Ok(vec![text(edges_value(&scope.body, &scope.of_edge)?.to_pretty())])
            }
            "export" => self.export(args),
            "render" => self.render(args),
            "sketch_solve" => {
                let sketch = read_sketch(args)?;
                let env = read_env(args)?;
                match sketch.solve_with(&env) {
                    Ok(solved) => Ok(vec![text(solved.to_json().to_text())]),
                    Err(why) => Err(why.to_json().to_text()),
                }
            }
            "sketch_drag" => {
                let sketch = read_sketch(args)?;
                let env = read_env(args)?;
                let grab = read_grab(args.get("grab").ok_or("\"grab\" is needed")?)?;
                let to = read_point(args.get("to").ok_or("\"to\" is needed")?)?;
                match sketch.drag_with(grab, to, &env) {
                    Ok((after, report)) => Ok(vec![text(
                        Value::obj(vec![
                            ("sketch", after.to_json()),
                            ("reached", Value::Bool(report.reached)),
                            ("distance_from_target", Value::Num(report.distance)),
                        ])
                        .to_text(),
                    )]),
                    Err(why) => Err(why.to_json().to_text()),
                }
            }
            "sketch_regions" => match read_sketch(args)?.regions() {
                Ok(regions) => Ok(vec![text(Value::Arr(regions.iter().map(sk::Region::to_json).collect()).to_text())]),
                Err(why) => Err(why.to_json().to_text()),
            },
            "sketch_arrangement" => Ok(vec![text(read_sketch(args)?.arrangement().to_json().to_text())]),
            other => self.call_assembly(other, args).unwrap_or_else(|| Err(format!("there is no tool named \"{other}\""))),
        }
    }

    /// The current body, rebuilding if nothing has yet.
    fn body(&mut self) -> R<Arc<Body>> {
        if self.outcome.is_none() {
            self.outcome = Some(self.doc.rebuild(None));
        }
        self.outcome.as_ref().and_then(|o| o.body.clone()).ok_or_else(|| "there is no solid yet: add a feature that makes one".to_string())
    }

    /// What a tool that looks at the solid is looking at: everything, or the one body it asked for. With several bodies,
    /// whose each face and edge is.
    fn scope(&mut self, args: &Value) -> R<Scope> {
        let whole = self.body()?;
        let o = self.outcome.as_ref().ok_or("nothing has been built")?;
        if let Some(asked) = args.get("body") {
            let name = asked.as_str().ok_or("\"body\" must be the name of a body, like f3 or f3.2")?;
            let id = BodyId::parse(name).map_err(|x| x.to_string())?;
            let Some(part) = o.find_body(id) else {
                let there: Vec<String> = o.bodies.iter().map(|b| b.id.text()).collect();
                return Err(format!("there is no body {name}: there's {}", there.join(", ")));
            };
            return Ok(Scope { body: part.body.clone(), of_face: vec![name.to_string(); part.body.faces.len()], of_edge: vec![name.to_string(); part.body.edges.len()] });
        }
        if o.bodies.len() < 2 {
            return Ok(Scope { body: whole, of_face: Vec::new(), of_edge: Vec::new() });
        }
        let (mut of_face, mut of_edge) = (Vec::with_capacity(whole.faces.len()), Vec::with_capacity(whole.edges.len()));
        for b in &o.bodies {
            let name = b.id.text();
            of_face.extend(std::iter::repeat_n(name.clone(), b.body.faces.len()));
            of_edge.extend(std::iter::repeat_n(name, b.body.edges.len()));
        }
        Ok(Scope { body: whole, of_face, of_edge })
    }

    /// Each body: its name, what made it, and how big it is.
    fn bodies_value(&self) -> Value {
        let Some(o) = &self.outcome else { return Value::Arr(Vec::new()) };
        Value::Arr(
            o.bodies
                .iter()
                .map(|b| {
                    let props = mass_properties(&b.body);
                    let bounds = b.body.aabb();
                    let maker = self.doc.feature(b.id.feature).map_or(String::new(), |f| f.name.clone());
                    Value::obj(vec![
                        ("body", Value::str(b.id.text())),
                        ("made_by", Value::str(maker)),
                        ("volume", num(props.volume)),
                        ("area", num(props.area)),
                        ("faces", Value::Num(b.body.faces.len() as f64)),
                        ("box_min", vec3(bounds.min)),
                        ("box_max", vec3(bounds.max)),
                    ])
                })
                .collect(),
        )
    }

    fn rebuilt(&mut self, upto: Option<usize>) -> R<Vec<Value>> {
        self.outcome = Some(self.doc.rebuild(upto));
        Ok(vec![text(self.summary().to_pretty())])
    }

    fn steps_value(&self) -> Value {
        let Some(o) = &self.outcome else { return Value::Arr(Vec::new()) };
        Value::Arr(
            o.steps
                .iter()
                .map(|s| {
                    let mut items = vec![
                        ("id", Value::Num(s.id.0 as f64)),
                        ("name", Value::str(&s.name)),
                        ("type", Value::str(s.kind)),
                        ("state", Value::str(s.state.word())),
                    ];
                    if let StepState::Failed(why) = &s.state {
                        items.push(("error", Value::str(why)));
                    }
                    Value::obj(items)
                })
                .collect(),
        )
    }

    /// What a rebuild did, and what the solid is now.
    fn summary(&self) -> Value {
        let Some(o) = &self.outcome else { return Value::Null };
        let mut items = vec![("features", self.steps_value())];
        if let Some(i) = o.failed {
            let s = &o.steps[i];
            let why = match &s.state {
                StepState::Failed(w) => w.as_str(),
                _ => "",
            };
            items.push(("failed", Value::str(format!("f{} ({}): {why}. The solid shown is from before it.", s.id.0, s.name))));
        }
        match &o.body {
            Some(body) => {
                // A cut in the wrong place changes nothing and isn't an error, which is easy to
                // miss without looking. A feature that left no faces is worth pointing out.
                let mut notes = Vec::new();
                for s in &o.steps {
                    let built = matches!(s.state, StepState::Rebuilt | StepState::Reused);
                    // Copies keep the names of what they copied, so these have no faces of their own.
                    let names_none = matches!(s.kind, "transform" | "delete_bodies" | "boolean" | "sketch" | "constrained_sketch" | "pattern" | "mirror" | "offset" | "draft" | "delete_face");
                    if built && !names_none && !body.faces.iter().any(|f| f.tag.feature == s.id.0) {
                        notes.push(Value::str(format!("f{} ({}) left no faces on the solid, so it may not have touched it", s.id.0, s.name)));
                    }
                }
                // A curve that closes nothing no longer stops a sketch, so say which ones those were.
                for s in o.steps.iter().filter(|s| s.kind == "constrained_sketch") {
                    if let Ok(found) = self.doc.sketch_arrangement(s.id) {
                        let mut who: Vec<u32> = found.ignored.iter().map(|i| i.entity.0).collect();
                        who.sort_unstable();
                        who.dedup();
                        if !who.is_empty() {
                            let list = who.iter().map(u32::to_string).collect::<Vec<_>>().join(", ");
                            notes.push(Value::str(format!("f{} ({}): all or part of entities {list} enclose nothing and were left out", s.id.0, s.name)));
                        }
                    }
                }
                if !notes.is_empty() {
                    items.push(("notes", Value::Arr(notes)));
                }
                let props = mass_properties(body);
                // The mesh's bounds, since the B-rep's box is a safe overestimate around curves.
                let bounds = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).map_or_else(|_| body.aabb(), |m| m.bounds());
                let mut solid = vec![
                    ("volume", num(props.volume)),
                    ("area", num(props.area)),
                    ("bounds_min", vec3(bounds.min)),
                    ("bounds_max", vec3(bounds.max)),
                    ("faces", Value::Num(body.faces.len() as f64)),
                    ("edges", Value::Num(body.edges.len() as f64)),
                ];
                match validate(body) {
                    Ok(report) => {
                        solid.push(("valid", Value::Bool(true)));
                        solid.push(("shells", Value::Num(report.shells as f64)));
                        solid.push(("holes_through", Value::Arr(report.genus.iter().map(|g| Value::Num(*g as f64)).collect())));
                    }
                    Err(defects) => {
                        solid.push(("valid", Value::Bool(false)));
                        solid.push(("defects", Value::Arr(defects.iter().map(Value::str).collect())));
                    }
                }
                let touching = body.contact_vertices();
                if touching > 0 {
                    solid.push(("touching_points", Value::Num(touching as f64)));
                }
                if o.bodies.len() > 1 {
                    solid.push(("volume_note", Value::str("the bodies are separate, so this counts twice wherever they overlap")));
                    items.push(("bodies", self.bodies_value()));
                }
                items.push(("solid", Value::obj(solid)));
            }
            None => items.push(("solid", Value::Null)),
        }
        Value::obj(items)
    }

    /// Resolves a path under the root, refusing anything that would leave it.
    fn inside_root(&self, path: &str) -> R<PathBuf> {
        let refuse = || format!("\"{path}\" is outside the folder this server may use, {}", self.root.display());
        let p = Path::new(path);
        let root = self.root.canonicalize().map_err(|why| format!("the root folder can't be used: {why}"))?;
        let joined = if p.is_absolute() { p.to_path_buf() } else { root.join(p) };
        if joined.components().any(|c| matches!(c, Component::ParentDir)) {
            return Err(refuse());
        }
        // A link inside the root can still point out of it, so what counts is where it really leads.
        let check = if joined.exists() { joined.clone() } else { joined.parent().map(Path::to_path_buf).ok_or_else(refuse)? };
        let real = check.canonicalize().map_err(|why| format!("can't use {}: {why}", check.display()))?;
        if !real.starts_with(&root) {
            return Err(refuse());
        }
        Ok(joined)
    }

    fn writable(&self, path: &str, kinds: &[&str]) -> R<PathBuf> {
        let resolved = self.inside_root(path)?;
        let extension = resolved.extension().and_then(|x| x.to_str()).unwrap_or("").to_lowercase();
        if !kinds.contains(&extension.as_str()) {
            return Err(format!("\"{path}\" should end in .{}", kinds.join(" or .")));
        }
        Ok(resolved)
    }

    fn export(&mut self, args: &Value) -> R<Vec<Value>> {
        let format = string_arg(args, "format")?;
        let path = args.get("path").and_then(Value::as_str);
        let (bytes, kinds): (Vec<u8>, &[&str]) = match format {
            "coreshape" => (self.with_project(|p| p.save_text()).into_bytes(), &["coreshape", "json"]),
            "step" if args.get("body").is_none() && self.active_assembly().is_some() => {
                let tab = self.active;
                let label = self.project.tab(tab).map_or_else(|_| "model".to_string(), |t| t.name.clone());
                let text = self.with_project(|p| p.step_text(tab, &label)).map_err(|x| x.to_string())?;
                (text.into_bytes(), &["step", "stp"])
            }
            "step" => {
                let scope = self.scope(args)?;
                // Each body as its own solids, unless one was asked for.
                let all = self.outcome.as_ref().filter(|_| args.get("body").is_none()).and_then(|o| o.step_text("model"));
                let text = all.unwrap_or_else(|| coreshape::export::step::write_step(&scope.body, "model"));
                (text.into_bytes(), &["step", "stp"])
            }
            "stl" => {
                let body = self.view_scope(args)?.body;
                let mesh = tessellate(&body, &TessParams::for_size(body.aabb().diagonal())).map_err(|x| x.to_string())?;
                (coreshape::export::stl::write_binary(&mesh, "model"), &["stl"])
            }
            other => return Err(format!("unknown format \"{other}\"; expected step, stl or coreshape")),
        };
        match path {
            Some(p) => {
                let target = self.writable(p, kinds)?;
                std::fs::write(&target, &bytes).map_err(|why| format!("can't write {}: {why}", target.display()))?;
                Ok(vec![text(format!("Wrote {} bytes to {}.", bytes.len(), target.display()))])
            }
            None if format == "stl" => Err("an STL is binary, so it needs a \"path\" to go to".to_string()),
            None => Ok(vec![text(String::from_utf8(bytes).map_err(|_| "the export isn't text".to_string())?)]),
        }
    }

    /// What render and STL look at: the selected assembly, with each instance as a body of its own, or the Part Studio.
    fn view_scope(&mut self, args: &Value) -> R<Scope> {
        if args.get("body").is_some() || self.active_assembly().is_none() {
            return self.scope(args);
        }
        let solved = self.solved()?;
        let whole = solved.whole().ok_or("there are no parts in this assembly yet: instance_add puts some in")?;
        let (mut of_face, mut of_edge) = (Vec::new(), Vec::new());
        for p in &solved.placed {
            let name = format!("{:?}", p.path);
            of_face.extend(std::iter::repeat_n(name.clone(), p.body.faces.len()));
            of_edge.extend(std::iter::repeat_n(name, p.body.edges.len()));
        }
        Ok(Scope { body: Arc::new(whole), of_face, of_edge })
    }

    fn render(&mut self, args: &Value) -> R<Vec<Value>> {
        let scope = self.view_scope(args)?;
        let body = scope.body.clone();
        let size = |key: &str, default: u32| -> R<u32> {
            match args.get(key).and_then(Value::as_f64) {
                None => Ok(default),
                Some(n) if (16.0..=3000.0).contains(&n) => Ok(n as u32),
                Some(n) => Err(format!("\"{key}\" of {n} is out of range: it must be from 16 to 3000")),
            }
        };
        let (width, height) = (size("width", 800)?, size("height", 600)?);
        let names: Vec<String> = body.faces.iter().map(|f| FaceRef::from_tag(f.tag).name()).collect();
        let view_name = args.get("view").and_then(Value::as_str).unwrap_or("iso");
        let four = view_name == "four";
        let view = match (args.get("azimuth").and_then(Value::as_f64), args.get("elevation").and_then(Value::as_f64)) {
            (None, None) => {
                if four {
                    View::Iso
                } else {
                    View::parse(view_name).ok_or_else(|| format!("unknown view \"{view_name}\"; expected iso, front, back, left, right, top, bottom or four"))?
                }
            }
            (a, e) => View::Orbit { azimuth: a.unwrap_or(0.0), elevation: e.unwrap_or(0.0) },
        };
        let mut highlight_faces = Vec::new();
        if let Some(list) = args.get("highlight_faces") {
            for item in list.as_array().ok_or("\"highlight_faces\" must be a list of face names")? {
                let name = item.as_str().ok_or("\"highlight_faces\" must be a list of face names")?;
                let key = FaceKey::parse(name).map_err(|x| x.to_string())?;
                let before = highlight_faces.len();
                highlight_faces.extend(find_faces(body.as_ref(), &key));
                if highlight_faces.len() == before {
                    return Err(format!("no face is named {name}. faces_list gives the names there are."));
                }
            }
        }
        let mut highlight_edges = Vec::new();
        if let Some(selection) = args.get("highlight_edges") {
            let query = read_edge_select(selection).map_err(|x| x.to_string())?;
            let env = self.doc.environment().map_err(|x| x.to_string())?;
            highlight_edges = select_edges(&query, &body, &env).map_err(|x| x.to_string())?;
        }
        let labelled = args.get("labels").and_then(Value::as_bool).unwrap_or(false);
        // Separate bodies are hard to tell apart in one colour, so that's how they're shown unless asked otherwise.
        let groups: Vec<u32> = {
            let mut seen: Vec<&String> = Vec::new();
            scope
                .of_face
                .iter()
                .map(|name| match seen.iter().position(|n| *n == name) {
                    Some(g) => g as u32,
                    None => {
                        seen.push(name);
                        seen.len() as u32 - 1
                    }
                })
                .collect()
        };
        let several = groups.iter().any(|g| *g > 0);
        let colors = match args.get("colors").and_then(Value::as_str) {
            None if several => Colors::ByGroup,
            None | Some("plain") => Colors::Plain,
            Some("faces") => Colors::ByFace,
            Some("bodies") => Colors::ByGroup,
            Some(other) => return Err(format!("unknown colors \"{other}\"; expected plain, faces or bodies")),
        };
        let options = Options {
            width,
            height,
            view,
            colors,
            highlight_faces,
            highlight_edges,
            face_names: if labelled { names.clone() } else { Vec::new() },
            face_groups: groups,
        };
        let image = if four { render_four(&body, &options) } else { render(&body, &options) }.map_err(|x| x.to_string())?;
        let png = image.to_png();
        if let Some(p) = args.get("path").and_then(Value::as_str) {
            let target = self.writable(p, &["png"])?;
            std::fs::write(&target, &png).map_err(|why| format!("can't write {}: {why}", target.display()))?;
        }
        let mut seen: Vec<(String, usize)> = Vec::new();
        for &(f, count) in &image.visible_faces {
            match seen.iter_mut().find(|s| s.0 == names[f]) {
                Some(s) => s.1 += count,
                None => seen.push((names[f].clone(), count)),
            }
        }
        seen.sort_by_key(|s| std::cmp::Reverse(s.1));
        let mut info = vec![
            ("mm_per_pixel", Value::Num(image.mm_per_pixel)),
            ("visible_faces", Value::Arr(seen.iter().map(|s| Value::str(&s.0)).collect())),
        ];
        if let Some(o) = &self.outcome {
            if let Some(i) = o.failed {
                info.push(("note", Value::str(format!("f{} failed, so this shows the solid from before it", o.steps[i].id.0))));
            }
        }
        Ok(vec![
            Value::obj(vec![("type", Value::str("image")), ("data", Value::str(base64(&png))), ("mimeType", Value::str("image/png"))]),
            text(Value::obj(info).to_text()),
        ])
    }
}

fn surface_name(s: &Surface) -> &'static str {
    match s {
        Surface::Plane(_) => "plane",
        Surface::Cylinder { .. } => "cylinder",
        Surface::Cone { .. } => "cone",
        Surface::Sphere { .. } => "sphere",
        Surface::Torus { .. } => "torus",
        Surface::Extruded { .. } => "extrusion",
        Surface::Revolved { .. } => "revolution",
        Surface::Free { .. } => "free-form",
    }
}

/// What a tool is looking at. The names of the bodies are empty when there's only one to speak of.
struct Scope {
    body: Arc<Body>,
    of_face: Vec<String>,
    of_edge: Vec<String>,
}

/// A round face's axis: a point on it, which is where its radius is measured, and which way it goes.
fn axis_value(frame: &coreshape::Frame) -> Value {
    Value::obj(vec![("point", vec3(frame.origin)), ("direction", vec3(frame.z))])
}

/// Each vertex as an extrude's end names it: by the keys of the faces it's on, with a place if those fit several vertices.
fn vertices_value(body: &Body, keys: &[FaceKey]) -> Vec<Value> {
    let mut on: Vec<Vec<String>> = vec![Vec::new(); body.vertices.len()];
    for (f, face) in body.faces.iter().enumerate() {
        for c in face.loops.iter().flatten() {
            let list = &mut on[body.start_vertex(*c)];
            let key = keys[f].text();
            if !list.contains(&key) {
                list.push(key);
            }
        }
    }
    for list in &mut on {
        list.sort();
    }
    (0..body.vertices.len())
        .map(|v| {
            let mut items = vec![("faces", Value::Arr(on[v].iter().map(Value::str).collect()))];
            let fits = (0..on.len()).filter(|w| on[*w].iter().all(|k| on[v].contains(k)) && on[v].iter().all(|k| on[*w].contains(k))).count();
            if fits > 1 {
                items.push(("near", vec3(body.vertices[v])));
            }
            Value::obj(items)
        })
        .collect()
}

fn faces_value(body: &Body, of_face: &[String]) -> R<Value> {
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).map_err(|x| x.to_string())?;
    let n = body.faces.len();
    let mut area = vec![0.0; n];
    let mut sum = vec![coreshape::Vec3::ZERO; n];
    for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
        let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
        let w = (b - a).cross(c - a).norm() / 2.0;
        area[f as usize] += w;
        sum[f as usize] += (a + b + c) * (w / 3.0);
    }
    let mut out = Vec::with_capacity(n);
    let keys = face_keys(body);
    for (i, face) in body.faces.iter().enumerate() {
        let name = FaceRef::from_tag(face.tag).name();
        let center = sum[i] * (1.0 / area[i].max(1e-300));
        let on = face.surface.project(center);
        let normal = face.surface.normal(on) * if face.flipped { -1.0 } else { 1.0 };
        let mut items = vec![
            ("name", Value::str(name)),
            ("key", Value::str(keys[i].text())),
            ("surface", Value::str(surface_name(&face.surface))),
            ("area", num(area[i])),
            ("center", vec3(on)),
            ("normal", vec3(normal)),
        ];
        if let Some(owner) = of_face.get(i) {
            items.push(("body", Value::str(owner)));
        }
        match face.surface {
            Surface::Cylinder { radius, frame } => {
                items.push(("radius", num(radius)));
                items.push(("axis", axis_value(&frame)));
            }
            Surface::Sphere { radius, .. } => items.push(("radius", num(radius))),
            Surface::Cone { radius, frame, half_angle } => {
                items.push(("radius", num(radius)));
                items.push(("half_angle", num(half_angle.to_degrees())));
                items.push(("axis", axis_value(&frame)));
            }
            Surface::Torus { major, minor, frame } => {
                items.push(("major", num(major)));
                items.push(("minor", num(minor)));
                items.push(("axis", axis_value(&frame)));
            }
            Surface::Plane(_) | Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. } => {}
        }
        out.push(Value::obj(items));
    }
    Ok(Value::Arr(out))
}

fn edges_value(body: &Body, of_edge: &[String]) -> R<Value> {
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).map_err(|x| x.to_string())?;
    let uses = body.edge_uses();
    let mut out = Vec::with_capacity(body.edges.len());
    let names: Vec<String> = (0..body.edges.len()).map(|e| edge_name(body, e, &uses)).collect();
    let keys = face_keys(body);
    let vertices = vertices_value(body, &keys);
    let key_pairs: Vec<Vec<String>> = (0..body.edges.len())
        .map(|e| {
            let mut pair: Vec<String> = uses[e].iter().map(|u| keys[u.0].text()).collect();
            pair.sort();
            pair
        })
        .collect();
    for (e, edge) in body.edges.iter().enumerate() {
        let faces: Vec<Value> = names[e].split('|').map(Value::str).collect();
        let midpoint = body.edge_midpoint(e);
        let length: f64 = mesh.edge_lines.get(e).map_or(0.0, |l| l.windows(2).map(|w| (w[1] - w[0]).norm()).sum());
        // A name that fits more than one edge needs the faces' keys, and a place if even those fit several,
        // since two faces can meet more than once.
        let shared = names.iter().filter(|n| **n == names[e]).count() > 1;
        let between = if shared {
            Value::obj(vec![("between", Value::Arr(key_pairs[e].iter().map(Value::str).collect()))])
        } else {
            Value::obj(vec![("between", Value::Arr(faces.clone()))])
        };
        let select = if shared && key_pairs.iter().filter(|p| **p == key_pairs[e]).count() > 1 {
            let near = Value::obj(vec![("near", vec3(midpoint)), ("tolerance", Value::Num(1e-3))]);
            Value::obj(vec![("both", Value::Arr(vec![between, near]))])
        } else {
            between
        };
        let mut items = vec![
            ("name", Value::str(&names[e])),
            ("select", select),
            ("length", num(length)),
            ("midpoint", vec3(midpoint)),
            ("start", vec3(body.vertices[edge.v0])),
            ("end", vec3(body.vertices[edge.v1])),
            ("start_vertex", vertices[edge.v0].clone()),
            ("end_vertex", vertices[edge.v1].clone()),
        ];
        if let Some(owner) = of_edge.get(e) {
            items.push(("body", Value::str(owner)));
        }
        match &edge.curve {
            Curve::Line { dir, .. } => {
                items.push(("curve", Value::str("line")));
                items.push(("direction", vec3(*dir)));
            }
            Curve::Circle { frame, radius } => {
                items.push(("curve", Value::str("circle")));
                items.push(("radius", num(*radius)));
                items.push(("center", vec3(frame.origin)));
            }
            Curve::Ellipse { .. } => items.push(("curve", Value::str("ellipse"))),
            Curve::Sampled(_) => items.push(("curve", Value::str("traced"))),
            Curve::Spline(_) => items.push(("curve", Value::str("spline"))),
        }
        out.push(Value::obj(items));
    }
    Ok(Value::Arr(out))
}

fn read_sketch(args: &Value) -> R<sk::Sketch> {
    let v = args.get("sketch").ok_or("\"sketch\" is needed")?;
    // Agents shouldn't have to repeat the header, so it is added when left out.
    let with_header = match v {
        Value::Obj(items) if v.get("format").is_none() => {
            let mut all = vec![("format".to_string(), Value::str(sk::SKETCH_FORMAT)), ("version".to_string(), Value::Num(sk::SKETCH_VERSION))];
            all.extend(items.iter().cloned());
            Value::Obj(all)
        }
        other => other.clone(),
    };
    sk::Sketch::from_json(&with_header).map_err(|x| x.message)
}

fn read_env(args: &Value) -> R<Env> {
    let mut env = Env::new();
    if let Some(Value::Obj(items)) = args.get("parameters") {
        for (k, v) in items {
            env.insert(k.clone(), v.as_f64().ok_or_else(|| format!("parameter \"{k}\" must be a number"))?);
        }
    }
    Ok(env)
}

fn read_point(v: &Value) -> R<[f64; 2]> {
    match v.as_array() {
        Some([a, b]) => match (a.as_f64(), b.as_f64()) {
            (Some(a), Some(b)) => Ok([a, b]),
            _ => Err("a point is written [x, y]".to_string()),
        },
        _ => Err("a point is written [x, y]".to_string()),
    }
}

fn read_grab(v: &Value) -> R<sk::Grab> {
    if let Some(s) = v.as_str() {
        return Ok(sk::Grab::Point(sk::Ref::parse(s)?));
    }
    let edge = v.get("edge").and_then(Value::as_f64).ok_or("\"grab\" is a point such as \"7.end\", or {\"edge\": 7, \"at\": [x, y]}")?;
    let at = read_point(v.get("at").ok_or("grabbing an edge needs \"at\", where it was grabbed")?)?;
    Ok(sk::Grab::Edge { entity: sk::EntityId(edge as u32), at })
}
