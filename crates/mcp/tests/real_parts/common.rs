//! Runs a part's script through the same tools an app uses, feature by feature, the way a person works: each thing is
//! added, and if it's refused that's written down and the feature is turned off, so the rest of the part still gets tried.
//! Only the checks look past the tools, as a measurement of what they made has to come from somewhere else.
//!
//! A script is JSON: `name`, `kind`, `parameters`, `steps`, `always` and `changes`. A step is a feature, or
//! `{"tool": ..., "args": ..., "as": "name"}`, whose answer's `id` later steps can use as `"$name"`, or `{"expect": ...}`.
//! See `check` for what can be expected. `always` is expected at the end and again after each change of dimensions.

use coreshape::{mass_properties, tessellate, TessParams};
use coreshape_document::json::{self, Value};
use coreshape_mcp::Session;
use coreshape_sketch::expr::{Env, Expr};
use std::path::Path;
use std::time::Instant;

/// What became of one step.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Ok,
    /// Turned down. The text is why.
    Refused(String),
    /// Refused, after an earlier step in the same part was, so it may only be following from that.
    Followed(String),
    /// It went through, and a check found what it made is wrong.
    Wrong(String),
}

#[derive(Clone, Debug)]
pub struct Step {
    pub label: String,
    pub outcome: Outcome,
    pub seconds: f64,
    /// What the tool had to say about it, such as a cut that touched nothing.
    pub notes: Vec<String>,
}

/// A STEP file written for another system to check.
#[derive(Clone, Debug)]
pub struct Written {
    pub file: String,
    pub volume: f64,
    pub area: f64,
    pub solids: usize,
}

#[derive(Clone, Debug, Default)]
pub struct PartResult {
    pub name: String,
    pub kind: String,
    pub features: usize,
    pub steps: Vec<Step>,
    pub volume: Option<f64>,
    pub area: Option<f64>,
    pub faces: usize,
    pub seconds: f64,
    pub written: Vec<Written>,
    /// Why OpenCASCADE's volume for this part isn't to be believed, where a formula has shown that.
    pub occt_note: Option<String>,
}

impl PartResult {
    pub fn whole(&self) -> bool {
        self.steps.iter().all(|s| s.outcome == Outcome::Ok)
    }

    /// The first thing that went wrong, for the table.
    pub fn first_trouble(&self) -> Option<(String, String)> {
        self.steps.iter().find_map(|s| match &s.outcome {
            Outcome::Ok => None,
            Outcome::Refused(w) | Outcome::Followed(w) | Outcome::Wrong(w) => Some((s.label.clone(), w.clone())),
        })
    }

    pub fn to_json(&self) -> Value {
        let steps = self.steps.iter().map(|s| {
            let (state, why) = match &s.outcome {
                Outcome::Ok => ("ok", String::new()),
                Outcome::Refused(w) => ("refused", w.clone()),
                Outcome::Followed(w) => ("followed", w.clone()),
                Outcome::Wrong(w) => ("wrong", w.clone()),
            };
            Value::obj(vec![
                ("label", Value::str(&s.label)),
                ("state", Value::str(state)),
                ("why", Value::str(why)),
                ("seconds", Value::Num(s.seconds)),
                ("notes", Value::Arr(s.notes.iter().map(Value::str).collect())),
            ])
        });
        let written = self.written.iter().map(|w| {
            Value::obj(vec![("file", Value::str(&w.file)), ("volume", Value::Num(w.volume)), ("area", Value::Num(w.area)), ("solids", Value::Num(w.solids as f64))])
        });
        Value::obj(vec![
            ("name", Value::str(&self.name)),
            ("kind", Value::str(&self.kind)),
            ("features", Value::Num(self.features as f64)),
            ("volume", self.volume.map_or(Value::Null, Value::Num)),
            ("area", self.area.map_or(Value::Null, Value::Num)),
            ("faces", Value::Num(self.faces as f64)),
            ("seconds", Value::Num(self.seconds)),
            ("whole", Value::Bool(self.whole())),
            ("steps", Value::Arr(steps.collect())),
            ("written", Value::Arr(written.collect())),
            ("occt_note", self.occt_note.as_deref().map_or(Value::Null, Value::str)),
        ])
    }
}

/// Replaces `"$name"` with what it was captured as.
fn with_captures(v: &Value, captured: &[(String, f64)]) -> Value {
    let mut text = v.to_text();
    for (name, id) in captured {
        text = text.replace(&format!("\"${name}\""), &id.to_string());
    }
    json::parse(&text).unwrap_or_else(|why| panic!("a script's arguments stopped being JSON: {why}: {text}"))
}

fn answer_of(result: &Value) -> (bool, Value, String) {
    let failed = result.get("isError").and_then(Value::as_bool).unwrap_or(false);
    let texts: Vec<String> = result.get("content").and_then(Value::as_array).unwrap_or(&[]).iter().filter_map(|c| c.get("text").and_then(Value::as_str)).map(str::to_string).collect();
    let parsed = texts.iter().rev().find_map(|t| t.find(['{', '[']).and_then(|at| json::parse(&t[at..]).ok())).unwrap_or(Value::Null);
    (failed, parsed, texts.join(" "))
}

fn number(v: &Value, env: &Env, what: &str) -> f64 {
    match v {
        Value::Num(n) => *n,
        Value::Str(s) => {
            let e = Expr::parse(s).unwrap_or_else(|why| panic!("{what}: {why}"));
            e.eval(env).unwrap_or_else(|why| panic!("{what}: {why}"))
        }
        _ => panic!("{what} should be a number or an expression"),
    }
}

fn point(v: &Value, env: &Env, what: &str) -> [f64; 3] {
    let list = v.as_array().unwrap_or_else(|| panic!("{what} should be three numbers"));
    [number(&list[0], env, what), number(&list[1], env, what), number(&list[2], env, what)]
}

fn parameters_of(session: &Session) -> Env {
    session.document().environment().unwrap_or_default()
}

/// Everything now in the selected tab: the assembly if one is, and otherwise the Part Studio.
struct Now {
    body: Option<std::sync::Arc<coreshape::Body>>,
    parts: usize,
}

fn now_of(session: &mut Session) -> Now {
    if session.active_tab_is_assembly() {
        let solved = session.solved().ok();
        let parts = solved.as_ref().map_or(0, |s| s.placed.len());
        Now { body: solved.and_then(|s| s.whole()).map(std::sync::Arc::new), parts }
    } else {
        let o = session.outcome();
        Now { body: o.body.clone(), parts: o.bodies.len() }
    }
}

/// What a script can expect. Every number can be an expression over the parameters.
///
/// `volume` with `volume_tolerance`, relative and 1e-6 by default. `area` the same. `genus`: the through-holes of each shell, in
/// order. `parts`. `bounds` as `[min, max]`, with `bounds_tolerance`. For an assembly, whose last answer is `state`: `freedom`,
/// `failing` (how many mates are), `origin` as `{"name": [x, y, z]}`, `value` as `{"$mate": {"angle": d}}`, `interferences`
/// (how many pairs), and `bom` as `{"name": count}`.
fn check(session: &mut Session, want: &Value, state: &Value, captured: &[(String, f64)]) -> Vec<String> {
    let want = with_captures(want, captured);
    let env = parameters_of(session);
    let mut wrong = Vec::new();
    let now = now_of(session);
    let is_assembly = session.active_tab_is_assembly();
    let Some(body) = now.body else {
        wrong.push("there's no solid".to_string());
        return wrong;
    };
    // Whatever else, it has to be a sound solid.
    if !is_assembly {
        if let Err(defects) = coreshape::validate(&body) {
            wrong.push(format!("not valid: {}", defects.join("; ")));
        }
    }
    let mesh = tessellate(&body, &TessParams::for_size(body.aabb().diagonal()));
    match &mesh {
        Err(why) => wrong.push(format!("doesn't mesh: {why}")),
        Ok(m) if m.open_edge_count() > 0 => wrong.push(format!("the mesh has {} open edges", m.open_edge_count())),
        Ok(_) => {}
    }
    let props = mass_properties(&body);
    let close = |got: f64, wanted: f64, tolerance: f64| (got - wanted).abs() <= tolerance * wanted.abs().max(1e-9);
    if let Some(v) = want.get("volume") {
        let wanted = number(v, &env, "volume");
        let tolerance = want.get("volume_tolerance").and_then(Value::as_f64).unwrap_or(1e-6);
        if !close(props.volume, wanted, tolerance) {
            wrong.push(format!("volume is {:.6} and should be {wanted:.6}", props.volume));
        }
    }
    if let Some(v) = want.get("area") {
        let wanted = number(v, &env, "area");
        let tolerance = want.get("area_tolerance").and_then(Value::as_f64).unwrap_or(1e-6);
        if !close(props.area, wanted, tolerance) {
            wrong.push(format!("area is {:.6} and should be {wanted:.6}", props.area));
        }
    }
    if let Some(g) = want.get("genus") {
        let wanted: Vec<usize> = g.as_array().expect("genus is a list").iter().map(|n| number(n, &env, "genus") as usize).collect();
        match coreshape::validate(&body) {
            Ok(report) if report.genus == wanted => {}
            Ok(report) => wrong.push(format!("holes through it are {:?} and should be {wanted:?}", report.genus)),
            Err(_) => {}
        }
    }
    if let Some(p) = want.get("parts") {
        let wanted = number(p, &env, "parts") as usize;
        if now.parts != wanted {
            wrong.push(format!("there are {} parts and should be {wanted}", now.parts));
        }
    }
    if let (Some(b), Ok(m)) = (want.get("bounds"), &mesh) {
        let list = b.as_array().expect("bounds are [min, max]");
        let (lo, hi) = (point(&list[0], &env, "bounds"), point(&list[1], &env, "bounds"));
        let (a, z) = (m.bounds().min, m.bounds().max);
        let size = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
        let tolerance = want.get("bounds_tolerance").and_then(Value::as_f64).unwrap_or(0.003 * size);
        let (got, wanted) = ([a.x, a.y, a.z, z.x, z.y, z.z], [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]]);
        if got.iter().zip(&wanted).any(|(g, w)| (g - w).abs() > tolerance) {
            wrong.push(format!("bounds are {got:.3?} and should be {wanted:.3?}"));
        }
    }
    // For an assembly.
    if let Some(n) = want.get("freedom") {
        let wanted = number(n, &env, "freedom");
        let got = state.get("freedom").and_then(Value::as_f64).unwrap_or(-1.0);
        if got != wanted {
            wrong.push(format!("it can move {got} ways and should be {wanted}"));
        }
    }
    if let Some(n) = want.get("failing") {
        let wanted = number(n, &env, "failing") as usize;
        let list = state.get("mates").and_then(Value::as_array).unwrap_or(&[]);
        let failing: Vec<&Value> = list.iter().filter(|m| m.get("failing").is_some()).collect();
        if failing.len() != wanted {
            let what: Vec<String> = failing.iter().map(|m| m.to_text()).collect();
            wrong.push(format!("{} mates fail and should be {wanted}: {}", failing.len(), what.join(", ")));
        }
    }
    if let Some(Value::Obj(items)) = want.get("origin") {
        let list = state.get("instances").and_then(Value::as_array).unwrap_or(&[]);
        for (name, at) in items {
            let wanted = point(at, &env, "origin");
            let found = list.iter().find(|i| i.get("name").and_then(Value::as_str) == Some(name)).and_then(|i| i.get("at")).and_then(|a| a.get("origin")).and_then(Value::as_array);
            match found {
                Some(o) if o.iter().zip(&wanted).all(|(g, w)| (g.as_f64().unwrap_or(f64::NAN) - w).abs() < 1e-5) => {}
                Some(o) => wrong.push(format!("{name} is at {} and should be at {wanted:?}", Value::Arr(o.to_vec()).to_text())),
                None => wrong.push(format!("no instance is called {name}")),
            }
        }
    }
    if let Some(Value::Arr(items)) = want.get("value") {
        // Each is [mate, dof, value], with the mate as "$name".
        let list = state.get("mates").and_then(Value::as_array).unwrap_or(&[]);
        for item in items {
            let parts = item.as_array().expect("a value is [mate, dof, value]");
            let Some(id) = parts[0].as_f64() else {
                wrong.push(format!("mate {} was never made", parts[0].to_text()));
                continue;
            };
            let dof = parts[1].as_str().expect("a dof");
            let wanted = number(&parts[2], &env, "a mate's value");
            let mate = list.iter().find(|m| m.get("id").and_then(Value::as_f64) == Some(id));
            let got = mate.and_then(|m| m.get("values")).and_then(|v| v.get(dof)).and_then(Value::as_f64);
            if got.is_none_or(|g| (g - wanted).abs() > 1e-4) {
                wrong.push(format!("mate {id}'s {dof} is {got:?} and should be {wanted}"));
            }
        }
    }
    if let Some(n) = want.get("interferences") {
        let wanted = number(n, &env, "interferences") as usize;
        let (_, answer, text) = answer_of(&session.call_safely("interference_check", &Value::obj(vec![])));
        let found = answer.get("interferences").and_then(Value::as_array).map_or(usize::MAX, <[Value]>::len);
        let unknown = answer.get("couldnt_tell").and_then(Value::as_array).map_or(0, <[Value]>::len);
        if found != wanted || unknown > 0 {
            wrong.push(format!("interference should be {wanted} pairs: {text}"));
        }
    }
    if let Some(Value::Obj(items)) = want.get("bom") {
        let (_, answer, _) = answer_of(&session.call_safely("bom", &Value::obj(vec![])));
        let rows = answer.as_array().unwrap_or(&[]);
        for (name, count) in items {
            let wanted = number(count, &env, "a parts list");
            let got = rows.iter().find(|r| r.get("name").and_then(Value::as_str) == Some(name)).and_then(|r| r.get("count")).and_then(Value::as_f64);
            if got != Some(wanted) {
                wrong.push(format!("{name}: the parts list says {got:?} and should be {wanted}"));
            }
        }
    }
    wrong
}

fn without(v: &Value, keys: &[&str]) -> Value {
    match v {
        Value::Obj(items) => Value::Obj(items.iter().filter(|(k, _)| !keys.contains(&k.as_str())).cloned().collect()),
        other => other.clone(),
    }
}

fn trouble(label: String, why: String) -> Step {
    Step { label, outcome: Outcome::Wrong(why), seconds: 0.0, notes: Vec::new() }
}

/// Exports and draws what's there now, under the session's root.
fn write_out(session: &mut Session, stem: &str, out: &mut PartResult, draw: bool) {
    let path = format!("{stem}.step");
    let args = Value::obj(vec![("format", Value::str("step")), ("path", Value::str(&path))]);
    let (failed, _, text) = answer_of(&session.call_safely("export", &args));
    if failed {
        out.steps.push(trouble(format!("export {stem}"), text));
        return;
    }
    let now = now_of(session);
    if let Some(body) = &now.body {
        let props = mass_properties(body);
        out.written.push(Written { file: path, volume: props.volume, area: props.area, solids: body.solid_count() });
    }
    if draw {
        let args = Value::obj(vec![("path", Value::str(format!("{stem}.png"))), ("width", Value::Num(480.0)), ("height", Value::Num(360.0))]);
        let (failed, _, text) = answer_of(&session.call_safely("render", &args));
        if failed {
            out.steps.push(trouble(format!("render {stem}"), text));
        }
    }
}

/// Runs one script. With `write`, each state's STEP file and picture go to the session's root under `stem`.
pub fn run(script: &Value, root: &Path, stem: &str, write: bool) -> PartResult {
    let started = Instant::now();
    let mut out = PartResult {
        name: script.get("name").and_then(Value::as_str).unwrap_or(stem).to_string(),
        kind: script.get("kind").and_then(Value::as_str).unwrap_or("").to_string(),
        occt_note: script.get("occt_note").and_then(Value::as_str).map(str::to_string),
        ..PartResult::default()
    };
    let mut session = Session::new(root.to_path_buf());
    let mut troubled = false;
    if let Some(Value::Obj(items)) = script.get("parameters") {
        for (name, value) in items {
            let args = Value::obj(vec![("name", Value::str(name)), ("value", value.clone())]);
            let (failed, _, text) = answer_of(&session.call_safely("parameter_set", &args));
            if failed {
                out.steps.push(Step { label: format!("parameter {name}"), outcome: Outcome::Refused(text), seconds: 0.0, notes: Vec::new() });
                troubled = true;
            }
        }
    }
    let mut captured: Vec<(String, f64)> = Vec::new();
    let mut state = Value::Null;
    let mut next_feature = 1.0;
    for step in script.get("steps").and_then(Value::as_array).expect("a script has steps") {
        let t = Instant::now();
        if let Some(want) = step.get("expect") {
            let wrong = check(&mut session, want, &state, &captured);
            let label = step.get("label").and_then(Value::as_str).unwrap_or("a check along the way").to_string();
            // After something's been turned off, what's expected along the way can't be told from what follows from that.
            if !wrong.is_empty() && troubled {
                continue;
            }
            let outcome = if wrong.is_empty() { Outcome::Ok } else { Outcome::Wrong(wrong.join("; ")) };
            out.steps.push(Step { label, outcome, seconds: t.elapsed().as_secs_f64(), notes: Vec::new() });
            continue;
        }
        let (tool, args, label) = if let Some(tool) = step.get("tool").and_then(Value::as_str) {
            let args = with_captures(step.get("args").unwrap_or(&Value::obj(vec![])), &captured);
            (tool.to_string(), args, step.get("label").and_then(Value::as_str).map_or_else(|| tool.to_string(), str::to_string))
        } else {
            // A feature.
            if let Some(id) = step.get("id").and_then(Value::as_f64) {
                assert_eq!(id, next_feature, "{}: a script's feature ids run from 1 in order", out.name);
            }
            let name = step.get("name").and_then(Value::as_str).unwrap_or("");
            let kind = step.get("type").and_then(Value::as_str).unwrap_or("?");
            let label = format!("f{next_feature} {kind}{}", if name.is_empty() { String::new() } else { format!(" ({name})") });
            let spec = without(step, &["id", "name", "note"]);
            let mut items = vec![("feature", with_captures(&spec, &captured))];
            if !name.is_empty() {
                items.push(("name", Value::str(name)));
            }
            ("feature_add".to_string(), Value::obj(items), label)
        };
        let result = session.call_safely(&tool, &args);
        let (failed, answer, text) = answer_of(&result);
        let notes: Vec<String> = answer.get("notes").and_then(Value::as_array).unwrap_or(&[]).iter().filter_map(Value::as_str).map(str::to_string).collect();
        let why = if failed { Some(text.clone()) } else { answer.get("failed").and_then(Value::as_str).map(str::to_string) };
        let seconds = t.elapsed().as_secs_f64();
        if tool == "feature_add" {
            if failed {
                // Never added, and later steps count on its id. A stand-in that does nothing keeps them right.
                let stand_in = json::parse(r#"{"feature":{"type":"sketch","regions":[{"outer":{"type":"rect","w":1,"h":1}}]}}"#).unwrap();
                session.call_safely("feature_add", &stand_in);
                session.call_safely("feature_suppress", &Value::obj(vec![("id", Value::Num(next_feature)), ("suppressed", Value::Bool(true))]));
            } else if why.is_some() {
                // Turned off, so that the rest can go on, as a person would have to.
                session.call_safely("feature_suppress", &Value::obj(vec![("id", Value::Num(next_feature)), ("suppressed", Value::Bool(true))]));
            }
            next_feature += 1.0;
            out.features += 1;
        } else if !failed {
            state = answer.clone();
            if let (Some(name), Some(id)) = (step.get("as").and_then(Value::as_str), answer.get("id").and_then(Value::as_f64)) {
                captured.push((name.to_string(), id));
            }
        }
        let outcome = match why {
            None => Outcome::Ok,
            Some(w) if troubled => Outcome::Followed(w),
            Some(w) => {
                troubled = true;
                Outcome::Refused(w)
            }
        };
        out.steps.push(Step { label, outcome, seconds, notes });
    }

    // The end of the history, as it's supposed to be.
    let always = script.get("always").cloned().unwrap_or_else(|| Value::obj(vec![]));
    if !troubled {
        let wrong = check(&mut session, &always, &state, &captured);
        if !wrong.is_empty() {
            out.steps.push(trouble("the finished part".to_string(), wrong.join("; ")));
        }
    }
    {
        let now = now_of(&mut session);
        if let Some(body) = &now.body {
            let props = mass_properties(body);
            out.volume = Some(props.volume);
            out.area = Some(props.area);
            out.faces = body.faces.len();
        }
    }
    if write {
        write_out(&mut session, stem, &mut out, true);
    }

    // Comes back the same from its file.
    if !session.active_tab_is_assembly() {
        let (_, _, text) = answer_of(&session.call_safely("document_show", &Value::obj(vec![])));
        let mut again = Session::new(root.to_path_buf());
        let (failed, _, why) = answer_of(&again.call_safely("document_open", &Value::obj(vec![("text", Value::str(text))])));
        let there = again.outcome().body.as_ref().map(|b| mass_properties(b).volume);
        if failed || there.zip(out.volume).is_some_and(|(a, b)| (a - b).abs() > 1e-9 * b.abs().max(1.0)) || there.is_none() != out.volume.is_none() {
            out.steps.push(trouble("saved and opened again".to_string(), format!("{there:?} after opening, {:?} before. {why}", out.volume)));
        }
    }

    // Changes of dimensions, one at a time, as a user would make them.
    let original: Vec<(String, Value)> = match script.get("parameters") {
        Some(Value::Obj(items)) => items.clone(),
        _ => Vec::new(),
    };
    for (k, change) in script.get("changes").and_then(Value::as_array).unwrap_or(&[]).iter().enumerate() {
        let t = Instant::now();
        let mut said = Vec::new();
        let mut problem = None;
        if let Some(Value::Obj(items)) = change.get("set") {
            for (name, value) in items {
                said.push(format!("{name} = {}", value.to_text()));
                let args = Value::obj(vec![("name", Value::str(name)), ("value", value.clone())]);
                let (failed, answer, text) = answer_of(&session.call_safely("parameter_set", &args));
                problem = if failed { Some(text) } else { answer.get("failed").and_then(Value::as_str).map(str::to_string) };
            }
        }
        // An assembly's answer goes stale when a part changes, and asking again is what solving it is.
        if session.active_tab_is_assembly() {
            let (failed, answer, text) = answer_of(&session.call_safely("assembly_solve", &Value::obj(vec![])));
            if failed {
                problem = Some(text);
            } else {
                state = answer;
            }
        }
        // What a person would do next, such as taking a mechanism back to where it was.
        for next in change.get("then").and_then(Value::as_array).unwrap_or(&[]) {
            let args = with_captures(next.get("args").unwrap_or(&Value::obj(vec![])), &captured);
            let (failed, answer, text) = answer_of(&session.call_safely(next.get("tool").and_then(Value::as_str).expect("a step's tool"), &args));
            if failed {
                problem = Some(text);
            } else {
                state = answer;
            }
        }
        let label = format!("change {}: {}", k + 1, said.join(", "));
        let seconds = t.elapsed().as_secs_f64();
        if let Some(why) = problem {
            out.steps.push(Step { label, outcome: if troubled { Outcome::Followed(why) } else { Outcome::Refused(why) }, seconds, notes: Vec::new() });
        } else if !troubled {
            let mut wrong = check(&mut session, &always, &state, &captured);
            if let Some(want) = change.get("expect") {
                wrong.extend(check(&mut session, want, &state, &captured));
            }
            let outcome = if wrong.is_empty() { Outcome::Ok } else { Outcome::Wrong(wrong.join("; ")) };
            out.steps.push(Step { label, outcome, seconds, notes: Vec::new() });
            if write {
                write_out(&mut session, &format!("{stem}__change{}", k + 1), &mut out, false);
            }
        }
        // Back to how it was, which has to come back exactly.
        for (name, value) in &original {
            if change.get("set").and_then(|s| s.get(name)).is_some() {
                session.call_safely("parameter_set", &Value::obj(vec![("name", Value::str(name)), ("value", value.clone())]));
            }
        }
        let now = now_of(&mut session);
        let there = now.body.as_ref().map(|b| mass_properties(b).volume);
        if there.zip(out.volume).is_some_and(|(a, b)| (a - b).abs() > 1e-9 * b.abs().max(1.0)) {
            out.steps.push(trouble(format!("back after change {}", k + 1), format!("{there:?} and it was {:?}", out.volume)));
        }
    }
    out.seconds = started.elapsed().as_secs_f64();
    out
}

/// One set of dimensions a part was rebuilt with: what they were, `ok`, `refused` or `wrong`, and why.
pub type ScanRow = (String, &'static str, String);

/// Builds a part, then rebuilds it with its dimensions moved, all at once, by up to 5% either way, as someone dragging
/// sliders would. Counts stay as they are. What's expected has to go on holding, and that's a stiffer test than it sounds: a
/// hole that's near an edge can run into it, and then what's expected no longer holds, for a good reason.
pub fn scan(script: &Value, root: &Path, tries: usize) -> Vec<ScanRow> {
    let mut session = Session::new(root.to_path_buf());
    let items: Vec<(String, Value)> = match script.get("parameters") {
        Some(Value::Obj(items)) => items.clone(),
        _ => Vec::new(),
    };
    for (name, value) in &items {
        session.call_safely("parameter_set", &Value::obj(vec![("name", Value::str(name)), ("value", value.clone())]));
    }
    let mut captured: Vec<(String, f64)> = Vec::new();
    let mut state = Value::Null;
    for step in script.get("steps").and_then(Value::as_array).expect("a script has steps") {
        if step.get("expect").is_some() {
            continue;
        }
        let (tool, args) = match step.get("tool").and_then(Value::as_str) {
            Some(tool) => (tool.to_string(), with_captures(step.get("args").unwrap_or(&Value::obj(vec![])), &captured)),
            None => ("feature_add".to_string(), Value::obj(vec![("feature", without(step, &["id", "name", "note"]))])),
        };
        let (failed, answer, text) = answer_of(&session.call_safely(&tool, &args));
        if failed || answer.get("failed").is_some() {
            return vec![("as drawn".to_string(), "refused", text.chars().take(300).collect())];
        }
        if tool != "feature_add" {
            state = answer.clone();
            if let (Some(name), Some(id)) = (step.get("as").and_then(Value::as_str), answer.get("id").and_then(Value::as_f64)) {
                captured.push((name.to_string(), id));
            }
        }
    }
    let always = script.get("always").cloned().unwrap_or_else(|| Value::obj(vec![]));
    let moving: Vec<(String, f64)> = items
        .iter()
        .filter(|(k, _)| !["n", "z", "nx", "ny"].contains(&k.as_str()))
        .filter(|(k, _)| !script.get("scan_fixed").and_then(Value::as_array).unwrap_or(&[]).iter().any(|f| f.as_str() == Some(k)))
        .filter_map(|(k, v)| v.as_f64().map(|n| (k.clone(), n)))
        .collect();
    let mut rng = coreshape::Rng::new(0x5ca1ab1e);
    let mut out = Vec::new();
    for _ in 0..tries {
        let mut said = Vec::new();
        let mut problem = None;
        for (name, base) in &moving {
            let share = (rng.next_u64() % 1_000_000) as f64 / 1_000_000.0;
            let value = ((base * (0.95 + 0.10 * share)) * 100.0).round() / 100.0;
            said.push(format!("{name} = {value}"));
            let args = Value::obj(vec![("name", Value::str(name)), ("value", Value::Num(value))]);
            let (failed, answer, text) = answer_of(&session.call_safely("parameter_set", &args));
            problem = if failed { Some(text) } else { answer.get("failed").and_then(Value::as_str).map(str::to_string) };
        }
        if session.active_tab_is_assembly() {
            let (failed, answer, text) = answer_of(&session.call_safely("assembly_solve", &Value::obj(vec![])));
            if failed {
                problem = Some(text);
            } else {
                state = answer;
            }
        }
        let dims = said.join(", ");
        match problem {
            Some(why) => out.push((dims, "refused", why)),
            None => {
                let wrong = check(&mut session, &always, &state, &captured);
                if wrong.is_empty() {
                    out.push((dims, "ok", String::new()));
                } else {
                    out.push((dims, "wrong", wrong.join("; ")));
                }
            }
        }
    }
    out
}

/// The scripts, in the order of their names.
pub fn scripts(folder: &Path) -> Vec<(String, Value)> {
    let mut names: Vec<_> = std::fs::read_dir(folder)
        .expect("the scripts' folder")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "json") && p.file_stem().is_some_and(|s| s != "expected"))
        .collect();
    names.sort();
    names
        .into_iter()
        .map(|p| {
            let text = std::fs::read_to_string(&p).unwrap();
            let v = json::parse(&text).unwrap_or_else(|why| panic!("{}: {why}", p.display()));
            (p.file_stem().unwrap().to_string_lossy().to_string(), v)
        })
        .collect()
}
