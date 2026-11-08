//! The tools themselves, and the state they work on.

use coreshape::{mass_properties, tessellate, validate, Body, Curve, Surface, TessParams};
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
    ("feature_add", "Adds a feature, then rebuilds. Gives its permanent id and what the rebuild did. See format_guide for the fields.", r#"{"feature": {"type": "object", "description": "A feature, without id: {\"type\": \"extrude\", \"sketch\": 1, \"length\": 10}."}, "name": {"type": "string"}, "position": {"type": "integer", "description": "Where in the history. The end, if left out."}}"#, r#"["feature"]"#),
    ("feature_edit", "Replaces what a feature does, keeping its id, name and position, then rebuilds.", r#"{"id": {"type": "integer"}, "feature": {"type": "object"}}"#, r#"["id", "feature"]"#),
    ("feature_rename", "Renames a feature.", r#"{"id": {"type": "integer"}, "name": {"type": "string"}}"#, r#"["id", "name"]"#),
    ("feature_suppress", "Turns a feature off or on, then rebuilds.", r#"{"id": {"type": "integer"}, "suppressed": {"type": "boolean"}}"#, r#"["id", "suppressed"]"#),
    ("feature_delete", "Deletes a feature nothing refers to, then rebuilds.", r#"{"id": {"type": "integer"}}"#, r#"["id"]"#),
    ("feature_move", "Moves a feature in the history, if that keeps every reference pointing backward, then rebuilds.", r#"{"id": {"type": "integer"}, "position": {"type": "integer"}}"#, r#"["id", "position"]"#),
    ("features_list", "The history: each feature's id, name, type, state and any error.", "{}", "[]"),
    ("rebuild", "Rebuilds, reusing whatever didn't change. Gives each feature's state and the solid's volume, area, size and validity.", r#"{"upto": {"type": "integer", "description": "Stop after the feature at this position, to see an earlier state."}}"#, "[]"),
    ("faces_list", "The solid's faces with their stable names, surface type, area, center and outward normal. One name can cover several pieces.", "{}", "[]"),
    ("edges_list", "The solid's edges with their stable names, curve type, length and ends, and a selection for each, ready to use in a fillet or chamfer.", "{}", "[]"),
    ("export", "Exports the solid. STEP and the native format can come back as text; STL needs a path. Paths are under the server's root folder.", r#"{"format": {"type": "string", "enum": ["step", "stl", "coreshape"]}, "path": {"type": "string"}}"#, r#"["format"]"#),
    ("render", "Draws the solid and returns a PNG. Views: iso, front, back, left, right, top, bottom or four (top, iso, front and right together), or an azimuth and elevation in degrees.", r#"{"view": {"type": "string"}, "azimuth": {"type": "number"}, "elevation": {"type": "number"}, "width": {"type": "integer"}, "height": {"type": "integer"}, "labels": {"type": "boolean", "description": "Write each visible face's name on it."}, "colors": {"type": "string", "enum": ["plain", "faces"], "description": "faces gives each face its own tint."}, "highlight_faces": {"type": "array", "items": {"type": "string"}, "description": "Face names to pick out."}, "highlight_edges": {"description": "An edge selection to pick out, as in a fillet."}, "path": {"type": "string", "description": "Also save the PNG here, under the server's root folder."}}"#, "[]"),
    ("sketch_solve", "Solves a sketch without changing the document. Gives the solved sketch, remaining freedom, what's still free and what's redundant, or the constraints that can't all hold.", r#"{"sketch": {"type": "object", "description": "A sketch in the format in format_guide."}, "parameters": {"type": "object", "description": "Values for names used in its dimensions."}}"#, r#"["sketch"]"#),
    ("sketch_drag", "Moves a point or edge of a sketch toward a target, disturbing the rest as little as possible, and gives the new sketch.", r#"{"sketch": {"type": "object"}, "grab": {"description": "A point, such as \"7.end\", or an edge and where it was grabbed: {\"edge\": 7, \"at\": [x, y]}."}, "to": {"type": "array", "items": {"type": "number"}}, "parameters": {"type": "object"}}"#, r#"["sketch", "grab", "to"]"#),
    ("sketch_regions", "The closed regions a sketch's geometry makes: outer loops and holes. Says which entity leaves a gap if there aren't any.", r#"{"sketch": {"type": "object"}}"#, r#"["sketch"]"#),
];

pub fn exists(name: &str) -> bool {
    TOOLS.iter().any(|t| t.0 == name)
}

pub fn list() -> Value {
    Value::Arr(
        TOOLS
            .iter()
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
    doc: Document,
    outcome: Option<Outcome>,
    root: PathBuf,
}

impl Session {
    pub fn new(root: PathBuf) -> Session {
        Session { doc: Document::new(), outcome: None, root }
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
                self.doc = Document::new();
                self.outcome = None;
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
                self.doc = Document::load_text(&contents).map_err(e)?;
                self.outcome = None;
                self.rebuilt(None)
            }
            "document_save" => {
                let path = self.writable(string_arg(args, "path")?, &["coreshape", "json"])?;
                let contents = self.doc.save_text();
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
                let op = read_op(spec, &[]).map_err(e)?;
                let at = position_arg(args, "position")?.unwrap_or(self.doc.features().len());
                let name = args.get("name").and_then(Value::as_str).map(str::to_string);
                let id = self.doc.insert(op, name, at).map_err(e)?;
                let mut out = vec![text(format!("Added feature f{} (id {}).", id.0, id.0))];
                out.extend(self.rebuilt(None)?);
                Ok(out)
            }
            "feature_edit" => {
                let op = read_op(args.get("feature").ok_or("\"feature\" is needed")?, &[]).map_err(e)?;
                self.doc.edit(id_arg(args)?, op).map_err(e)?;
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
            "faces_list" => Ok(vec![text(faces_value(self.body()?.as_ref())?.to_pretty())]),
            "edges_list" => Ok(vec![text(edges_value(self.body()?.as_ref())?.to_pretty())]),
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
            other => Err(format!("there is no tool named \"{other}\"")),
        }
    }

    /// The current body, rebuilding if nothing has yet.
    fn body(&mut self) -> R<Arc<Body>> {
        if self.outcome.is_none() {
            self.outcome = Some(self.doc.rebuild(None));
        }
        self.outcome.as_ref().and_then(|o| o.body.clone()).ok_or_else(|| "there is no solid yet: add a feature that makes one".to_string())
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
                    let names_none = matches!(s.kind, "sketch" | "constrained_sketch" | "pattern" | "mirror");
                    if built && !names_none && !body.faces.iter().any(|f| f.tag.feature == s.id.0) {
                        notes.push(Value::str(format!("f{} ({}) left no faces on the solid, so it may not have touched it", s.id.0, s.name)));
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
            "coreshape" => (self.doc.save_text().into_bytes(), &["coreshape", "json"]),
            "step" => {
                let body = self.body()?;
                (coreshape::export::step::write_step(&body, "model").into_bytes(), &["step", "stp"])
            }
            "stl" => {
                let body = self.body()?;
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

    fn render(&mut self, args: &Value) -> R<Vec<Value>> {
        let body = self.body()?;
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
                FaceRef::parse(name).map_err(|x| x.to_string())?;
                let before = highlight_faces.len();
                highlight_faces.extend(names.iter().enumerate().filter(|(_, n)| n.as_str() == name).map(|(i, _)| i));
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
        let colors = match args.get("colors").and_then(Value::as_str) {
            None | Some("plain") => Colors::Plain,
            Some("faces") => Colors::ByFace,
            Some(other) => return Err(format!("unknown colors \"{other}\"; expected plain or faces")),
        };
        let options = Options {
            width,
            height,
            view,
            colors,
            highlight_faces,
            highlight_edges,
            face_names: if labelled { names.clone() } else { Vec::new() },
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
    }
}

fn faces_value(body: &Body) -> R<Value> {
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
    for (i, face) in body.faces.iter().enumerate() {
        let name = FaceRef::from_tag(face.tag).name();
        let center = sum[i] * (1.0 / area[i].max(1e-300));
        let on = face.surface.project(center);
        let normal = face.surface.normal(on) * if face.flipped { -1.0 } else { 1.0 };
        let mut items = vec![
            ("name", Value::str(name)),
            ("surface", Value::str(surface_name(&face.surface))),
            ("area", num(area[i])),
            ("center", vec3(on)),
            ("normal", vec3(normal)),
        ];
        match face.surface {
            Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } | Surface::Cone { radius, .. } => items.push(("radius", num(radius))),
            Surface::Torus { major, minor, .. } => {
                items.push(("major", num(major)));
                items.push(("minor", num(minor)));
            }
            Surface::Plane(_) => {}
        }
        out.push(Value::obj(items));
    }
    Ok(Value::Arr(out))
}

fn edges_value(body: &Body) -> R<Value> {
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).map_err(|x| x.to_string())?;
    let uses = body.edge_uses();
    let mut out = Vec::with_capacity(body.edges.len());
    let names: Vec<String> = (0..body.edges.len()).map(|e| edge_name(body, e, &uses)).collect();
    for (e, edge) in body.edges.iter().enumerate() {
        let faces: Vec<Value> = names[e].split('|').map(Value::str).collect();
        let midpoint = body.edge_midpoint(e);
        let length: f64 = mesh.edge_lines.get(e).map_or(0.0, |l| l.windows(2).map(|w| (w[1] - w[0]).norm()).sum());
        // A name that fits more than one edge needs a place to pick out this one.
        let shared = names.iter().filter(|n| **n == names[e]).count() > 1;
        let between = Value::obj(vec![("between", Value::Arr(faces.clone()))]);
        let select = if shared {
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
        ];
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
