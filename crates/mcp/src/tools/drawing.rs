//! The tools for drawings: a drawing tab of a part or an assembly, its views and dimensions, and writing it out. Every tool
//! that changes a drawing answers with a picture of it, so the change can be seen.

use super::*;
use coreshape_assembly::DrawingSource;
use coreshape_drawing::{out, read_dim_kind, read_view_kind, Drawing, Page, Projection, Title};

pub(super) const TOOLS: &[(&str, &str, &str, &str)] = &[
    ("drawing_new", "Adds a drawing tab, an engineering drawing of the Part Studio being worked on, one of its parts, or an assembly, and selects it. By default the front, top and right views and an iso one, laid out by first-angle projection as in Europe, on an A3 sheet at the largest standard scale that fits, with overall sizes and hole diameters dimensioned. Answers with a picture of it. It follows the model: change the part and draw it again.", r#"{"name": {"type": "string"}, "part": {"type": "string", "description": "One part, like f3, from bodies_list. Leave out for all of the Part Studio's parts."}, "assembly": {"type": "integer", "description": "An assembly tab to draw, instead."}, "sheet": {"type": "string", "enum": ["A4", "A3", "A2", "A1", "A0", "ANSI A", "ANSI B", "ANSI C", "ANSI D", "ANSI E"]}, "projection": {"type": "string", "enum": ["first_angle", "third_angle"]}, "scale": {"type": "number", "description": "0.5 for 1:2. Leave out to fit."}, "views": {"type": "array", "description": "Each {type: front, back, top, bottom, left, right, iso or section}. A section has point [x, y, z], normal, the half it points to taken away, and letter."}, "auto_dimensions": {"type": "boolean"}, "title": {"type": "object", "description": "title, number, part, material, author, date, company, revision."}}"#, "[]"),
    ("drawing_view_add", "Adds a view to the selected drawing: front, back, top, bottom, left, right, iso, or a section, cut by the plane through point square to normal with the half normal points to taken away, seen from that side, hatched where it's cut and labelled with its letter, with its cutting line drawn on a view that sees the plane edge on. Answers with a picture.", r#"{"tab": {"type": "integer"}, "type": {"type": "string", "enum": ["front", "back", "top", "bottom", "left", "right", "iso", "section"]}, "point": {"type": "array"}, "normal": {"type": "array"}, "letter": {"type": "string"}, "at": {"type": "array", "description": "Its middle on the sheet, [x, y] in millimetres from the bottom left. Leave out for the layout."}, "scale": {"type": "number", "description": "Times the sheet's scale."}, "hidden": {"type": "boolean", "description": "Hidden lines, dashed: on for the six views by default."}}"#, r#"["type"]"#),
    ("drawing_dimension_add", "Adds a dimension to a view of the selected drawing. distance: between points a and b, horizontal, vertical or aligned on the sheet, as the view shows them. diameter or radius: of the circle seen face on in the view nearest the point near. A point is [x, y, z], or for a drawing of a part, a corner {\"faces\": [...], \"near\": [...]} from vertices_list or a sketch's point {\"sketch\": 4, \"point\": \"7\"}, which follow the part. Answers with a picture.", r#"{"tab": {"type": "integer"}, "view": {"type": "integer"}, "type": {"type": "string", "enum": ["distance", "diameter", "radius"]}, "a": {}, "b": {}, "near": {}, "along": {"type": "string", "enum": ["horizontal", "vertical", "aligned"]}, "offset": {"type": "number", "description": "Millimetres on the sheet from what it measures to its line, 10 by default."}, "text": {"type": "string", "description": "Text in place of the value; <> in it stands for the value."}}"#, r#"["view", "type"]"#),
    ("drawing_edit", "Changes the selected drawing: its sheet, projection, scale (null to fit), automatic dimensions, title block, and views or dimensions to take out by id. Answers with a picture.", r#"{"tab": {"type": "integer"}, "sheet": {"type": "string"}, "projection": {"type": "string", "enum": ["first_angle", "third_angle"]}, "scale": {}, "auto_dimensions": {"type": "boolean"}, "title": {"type": "object"}, "remove": {"type": "array", "description": "Ids of views or dimensions."}}"#, "[]"),
    ("drawing_export", "Writes the selected drawing as svg, pdf or dxf, or draws it as png. With a path, to that file; without, the SVG or DXF text, or the PNG as a picture.", r#"{"tab": {"type": "integer"}, "format": {"type": "string", "enum": ["svg", "pdf", "dxf", "png"]}, "path": {"type": "string"}, "dpi": {"type": "number", "description": "For a PNG: dots per inch, 100 by default."}}"#, r#"["format"]"#),
];

fn e(x: coreshape::Error) -> String {
    x.to_string()
}

fn read_title(v: &Value, into: &mut Title) -> R<()> {
    let set = |key: &str, slot: &mut String| -> R<()> {
        if let Some(x) = v.get(key) {
            *slot = x.as_str().ok_or_else(|| format!("the title's \"{key}\" must be text"))?.to_string();
        }
        Ok(())
    };
    set("title", &mut into.title)?;
    set("number", &mut into.number)?;
    set("part", &mut into.part)?;
    set("material", &mut into.material)?;
    set("author", &mut into.author)?;
    set("date", &mut into.date)?;
    set("company", &mut into.company)?;
    set("revision", &mut into.revision)?;
    Ok(())
}

fn read_projection(v: &Value) -> R<Projection> {
    match v.as_str() {
        Some("first_angle" | "first") => Ok(Projection::FirstAngle),
        Some("third_angle" | "third") => Ok(Projection::ThirdAngle),
        _ => Err("\"projection\" is first_angle or third_angle".to_string()),
    }
}

impl Session {
    pub(super) fn call_drawing(&mut self, name: &str, args: &Value) -> Option<R<Vec<Value>>> {
        if !TOOLS.iter().any(|t| t.0 == name) {
            return None;
        }
        Some(self.drawing_tool(name, args))
    }

    /// The drawing a tool works on: the one given, or the one selected.
    fn drawing_tab(&self, args: &Value) -> R<TabId> {
        let tab = match args.get("tab") {
            Some(v) => TabId(v.as_f64().filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < 4e9).ok_or("\"tab\" must be a tab's id")? as u32),
            None => self.active_tab(),
        };
        self.project.drawing(tab).map_err(|_| format!("tab {} isn't a drawing: drawing_new makes one, or give a drawing's \"tab\"", tab.0))?;
        Ok(tab)
    }

    fn drawing_page(&mut self, tab: TabId) -> R<Page> {
        self.with_project(|p| p.drawing_page(tab)).map_err(e)
    }

    /// The drawing as it is now: a picture, and what's worth saying about it.
    fn drawing_answer(&mut self, tab: TabId, lead: Vec<(&str, Value)>) -> R<Vec<Value>> {
        let page = self.drawing_page(tab)?;
        let mut items = lead;
        items.push(("tab", Value::Num(tab.0 as f64)));
        items.push(("scale", Value::str(coreshape_drawing::format_scale(page.scale))));
        let d = self.project.drawing(tab).map_err(e)?;
        let views = d.drawing.views.iter().map(|v| Value::obj(vec![("id", Value::Num(v.id as f64)), ("type", Value::str(v.kind.name()))])).collect();
        items.push(("views", Value::Arr(views)));
        if !page.notes.is_empty() {
            items.push(("notes", Value::Arr(page.notes.iter().map(Value::str).collect())));
        }
        // A page's width at 100 dots per inch is under 1700 pixels even for A3.
        let png = out::png(&page, 100.0 / 25.4);
        Ok(vec![text(Value::obj(items).to_pretty()), Value::obj(vec![("type", Value::str("image")), ("data", Value::str(base64(&png))), ("mimeType", Value::str("image/png"))])])
    }

    fn drawing_tool(&mut self, name: &str, args: &Value) -> R<Vec<Value>> {
        match name {
            "drawing_new" => {
                let source = match (args.get("assembly"), args.get("part")) {
                    (Some(a), None) => DrawingSource::Assembly { tab: TabId(a.as_f64().filter(|n| n.fract() == 0.0 && *n >= 0.0).ok_or("\"assembly\" must be a tab's id")? as u32) },
                    (None, part) => {
                        let body = part.map(|p| p.as_str().ok_or("\"part\" must be a part's name, like f3").and_then(|s| BodyId::parse(s).map_err(|_| "\"part\" must be a part's name, like f3"))).transpose()?;
                        DrawingSource::Part { studio: self.studio, body }
                    }
                    (Some(_), Some(_)) => return Err("a drawing is of a \"part\" or of an \"assembly\", not both".to_string()),
                };
                let mut drawing = if args.get("views").is_some() { Drawing::default() } else { Drawing::standard() };
                if let Some(list) = args.get("views") {
                    for v in list.as_array().ok_or("\"views\" must be a list")? {
                        let kind = read_view_kind(v).map_err(e)?;
                        drawing.add_view(kind, None);
                    }
                }
                if let Some(s) = args.get("sheet") {
                    drawing.sheet = s.as_str().ok_or("\"sheet\" must be text")?.to_string();
                    drawing.size().map_err(e)?;
                }
                if let Some(p) = args.get("projection") {
                    drawing.projection = read_projection(p)?;
                }
                if let Some(s) = args.get("scale") {
                    drawing.scale = Some(s.as_f64().filter(|x| x.is_finite() && *x > 0.0).ok_or("\"scale\" must be a number above 0")?);
                }
                if let Some(a) = args.get("auto_dimensions") {
                    drawing.auto_dimensions = a.as_bool().ok_or("\"auto_dimensions\" is true or false")?;
                }
                if let Some(t) = args.get("title") {
                    read_title(t, &mut drawing.title)?;
                }
                if drawing.title.part.is_empty() {
                    drawing.title.part = match &source {
                        DrawingSource::Part { body: Some(b), .. } => b.text(),
                        DrawingSource::Part { studio, .. } | DrawingSource::Assembly { tab: studio } => self.project.tab(*studio).map(|t| t.name.clone()).unwrap_or_default(),
                    };
                }
                let count = self.project.tabs.len() + 1;
                let given = args.get("name").and_then(Value::as_str).map(str::to_string).unwrap_or_else(|| format!("Drawing {count}"));
                let tab = self.project.add_drawing(&given, source, drawing).map_err(e)?;
                self.active = tab;
                match self.drawing_answer(tab, Vec::new()) {
                    Ok(answer) => Ok(answer),
                    Err(why) => {
                        // A drawing that can't be made isn't kept.
                        let _ = self.project.delete_tab(tab);
                        self.active = self.studio;
                        Err(why)
                    }
                }
            }
            "drawing_view_add" => {
                let tab = self.drawing_tab(args)?;
                let kind = read_view_kind(args).map_err(e)?;
                let at = match args.get("at").and_then(Value::as_array) {
                    Some([x, y]) => Some([x.as_f64().ok_or("\"at\" is [x, y]")?, y.as_f64().ok_or("\"at\" is [x, y]")?]),
                    Some(_) => return Err("\"at\" is [x, y]".to_string()),
                    None => None,
                };
                let d = &mut self.project.drawing_mut(tab).map_err(e)?.drawing;
                let id = d.add_view(kind, at);
                if let Some(view) = d.views.iter_mut().find(|v| v.id == id) {
                    view.scale = args.get("scale").and_then(Value::as_f64).filter(|s| s.is_finite() && *s > 0.0);
                    view.hidden = args.get("hidden").and_then(Value::as_bool);
                }
                self.drawing_answer(tab, vec![("id", Value::Num(id as f64))])
            }
            "drawing_dimension_add" => {
                let tab = self.drawing_tab(args)?;
                let view = args.get("view").and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 1.0).ok_or("\"view\" must be a view's id")? as u32;
                let kind = read_dim_kind(args).map_err(e)?;
                let offset = args.get("offset").and_then(Value::as_f64).filter(|x| x.is_finite()).unwrap_or(10.0);
                let label = args.get("text").map(|t| t.as_str().map(str::to_string).ok_or("\"text\" must be text")).transpose()?;
                let id = self.project.drawing_mut(tab).map_err(e)?.drawing.add_dimension(view, kind, offset, label).map_err(e)?;
                let answer = self.drawing_answer(tab, vec![("id", Value::Num(id as f64))])?;
                // A dimension that can't be drawn says why, and isn't kept.
                let page = self.drawing_page(tab)?;
                if let Some(why) = page.notes.iter().find(|n| n.starts_with(&format!("dimension {id}:"))) {
                    self.project.drawing_mut(tab).map_err(e)?.drawing.dimensions.retain(|d| d.id != id);
                    return Err(why.clone());
                }
                Ok(answer)
            }
            "drawing_edit" => {
                let tab = self.drawing_tab(args)?;
                let mut d = self.project.drawing(tab).map_err(e)?.drawing.clone();
                if let Some(s) = args.get("sheet") {
                    d.sheet = s.as_str().ok_or("\"sheet\" must be text")?.to_string();
                    d.size().map_err(e)?;
                }
                if let Some(p) = args.get("projection") {
                    d.projection = read_projection(p)?;
                }
                match args.get("scale") {
                    None => {}
                    Some(Value::Null) => d.scale = None,
                    Some(s) => d.scale = Some(s.as_f64().filter(|x| x.is_finite() && *x > 0.0).ok_or("\"scale\" must be a number above 0, or null to fit")?),
                }
                if let Some(a) = args.get("auto_dimensions") {
                    d.auto_dimensions = a.as_bool().ok_or("\"auto_dimensions\" is true or false")?;
                }
                if let Some(t) = args.get("title") {
                    read_title(t, &mut d.title)?;
                }
                if let Some(list) = args.get("remove") {
                    for id in list.as_array().ok_or("\"remove\" must be a list of ids")? {
                        let id = id.as_f64().filter(|n| n.fract() == 0.0 && *n >= 1.0).ok_or("\"remove\" must be a list of ids")? as u32;
                        let before = d.views.len() + d.dimensions.len();
                        d.views.retain(|v| v.id != id);
                        d.dimensions.retain(|x| x.id != id && x.view != id);
                        if d.views.len() + d.dimensions.len() == before {
                            return Err(format!("this drawing has no view or dimension {id}"));
                        }
                    }
                    if d.views.is_empty() {
                        return Err("a drawing needs at least one view".to_string());
                    }
                }
                self.project.drawing_mut(tab).map_err(e)?.drawing = d;
                self.drawing_answer(tab, Vec::new())
            }
            "drawing_export" => {
                let tab = self.drawing_tab(args)?;
                let format = string_arg(args, "format")?;
                let page = self.drawing_page(tab)?;
                let dpi = args.get("dpi").and_then(Value::as_f64).filter(|d| (10.0..=1200.0).contains(d)).unwrap_or(100.0);
                let (bytes, kinds): (Vec<u8>, &[&str]) = match format {
                    "svg" => (out::svg(&page).into_bytes(), &["svg"]),
                    "pdf" => (out::pdf(&page), &["pdf"]),
                    "dxf" => (out::dxf(&page).into_bytes(), &["dxf"]),
                    "png" => (out::png(&page, dpi / 25.4), &["png"]),
                    other => return Err(format!("unknown format \"{other}\"; expected svg, pdf, dxf or png")),
                };
                match args.get("path").and_then(Value::as_str) {
                    Some(p) => {
                        let target = self.writable(p, kinds)?;
                        std::fs::write(&target, &bytes).map_err(|why| format!("can't write {}: {why}", target.display()))?;
                        Ok(vec![text(format!("Wrote {} bytes to {}.", bytes.len(), target.display()))])
                    }
                    None if format == "png" => Ok(vec![Value::obj(vec![("type", Value::str("image")), ("data", Value::str(base64(&bytes))), ("mimeType", Value::str("image/png"))])]),
                    None if format == "pdf" => Err("a PDF needs a \"path\" to go to".to_string()),
                    None => Ok(vec![text(String::from_utf8(bytes).map_err(|_| "the export isn't text".to_string())?)]),
                }
            }
            other => Err(format!("there is no tool named \"{other}\"")),
        }
    }
}
