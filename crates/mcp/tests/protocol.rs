//! Drives the real binary over its pipes, the way an MCP client does.

use coreshape_document::json::{self, Value};
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

struct Client {
    child: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
    next: f64,
}

fn folder(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("coreshape-mcp-test-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

impl Client {
    fn start(root: &PathBuf) -> Client {
        let mut child = Command::new(env!("CARGO_BIN_EXE_coreshape-mcp"))
            .arg("--root")
            .arg(root)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let input = child.stdin.take().unwrap();
        let output = BufReader::new(child.stdout.take().unwrap());
        Client { child, input, output, next: 1.0 }
    }

    fn send_line(&mut self, line: &str) {
        writeln!(self.input, "{line}").unwrap();
        self.input.flush().unwrap();
    }

    fn read(&mut self) -> Value {
        let mut line = String::new();
        self.output.read_line(&mut line).unwrap();
        assert!(!line.is_empty(), "the server closed the connection");
        assert!(line.ends_with('\n') && !line.trim_end().contains('\n'), "one message per line");
        json::parse(&line).unwrap_or_else(|e| panic!("bad JSON from the server: {e}: {line}"))
    }

    fn request(&mut self, method: &str, params: &str) -> Value {
        let id = self.next;
        self.next += 1.0;
        self.send_line(&format!("{{\"jsonrpc\":\"2.0\",\"id\":{id},\"method\":\"{method}\",\"params\":{params}}}"));
        let answer = self.read();
        assert_eq!(answer.get("jsonrpc").and_then(Value::as_str), Some("2.0"));
        assert_eq!(answer.get("id").and_then(Value::as_f64), Some(id));
        answer
    }

    /// Calls a tool, and gives its content and whether it failed.
    fn call(&mut self, tool: &str, args: &str) -> (Vec<Value>, bool) {
        let answer = self.request("tools/call", &format!("{{\"name\":\"{tool}\",\"arguments\":{args}}}"));
        let result = answer.get("result").unwrap_or_else(|| panic!("{tool}: {}", answer.to_text()));
        let content = result.get("content").and_then(Value::as_array).expect("content").to_vec();
        (content, result.get("isError").and_then(Value::as_bool).unwrap_or(false))
    }

    /// Calls a tool that must work, and gives its text.
    fn ok(&mut self, tool: &str, args: &str) -> String {
        let (content, failed) = self.call(tool, args);
        let body = content.iter().filter_map(|c| c.get("text").and_then(Value::as_str)).collect::<Vec<_>>().join("\n");
        assert!(!failed, "{tool} {args} failed: {body}");
        body
    }

    fn refused(&mut self, tool: &str, args: &str) -> String {
        let (content, failed) = self.call(tool, args);
        let body = content.iter().filter_map(|c| c.get("text").and_then(Value::as_str)).collect::<Vec<_>>().join("\n");
        assert!(failed, "{tool} {args} should have been refused: {body}");
        body
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn volume(summary: &str) -> f64 {
    // A feature_add answer holds a sentence, then the summary.
    let at = summary.find('{').unwrap();
    let v = json::parse(&summary[at..]).unwrap_or_else(|e| panic!("{e}: {summary}"));
    v.get("solid").and_then(|s| s.get("volume")).and_then(Value::as_f64).expect("a volume")
}

fn decode_base64(s: &str) -> Vec<u8> {
    let val = |c: u8| match c {
        b'A'..=b'Z' => c - b'A',
        b'a'..=b'z' => c - b'a' + 26,
        b'0'..=b'9' => c - b'0' + 52,
        b'+' => 62,
        b'/' => 63,
        other => panic!("bad base64 byte {other}"),
    } as u32;
    let mut out = Vec::new();
    for chunk in s.as_bytes().chunks(4) {
        assert_eq!(chunk.len(), 4);
        let n = val(chunk[0]) << 18 | val(chunk[1]) << 12 | if chunk[2] != b'=' { val(chunk[2]) << 6 } else { 0 } | if chunk[3] != b'=' { val(chunk[3]) } else { 0 };
        out.push((n >> 16) as u8);
        if chunk[2] != b'=' {
            out.push((n >> 8) as u8);
        }
        if chunk[3] != b'=' {
            out.push(n as u8);
        }
    }
    out
}

#[test]
fn the_handshake_follows_the_protocol() {
    let dir = folder("handshake");
    let mut c = Client::start(&dir);
    let a = c.request("initialize", r#"{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"test","version":"1"}}"#);
    let r = a.get("result").unwrap();
    assert_eq!(r.get("protocolVersion").and_then(Value::as_str), Some("2025-06-18"), "it agrees to a version it knows");
    assert!(r.get("capabilities").and_then(|x| x.get("tools")).is_some());
    assert_eq!(r.get("serverInfo").and_then(|x| x.get("name")).and_then(Value::as_str), Some("coreshape"));
    assert!(r.get("instructions").and_then(Value::as_str).unwrap().contains("feature_add"));
    // An unknown version gets its newest, and a client decides from there.
    let a = c.request("initialize", r#"{"protocolVersion":"1999-01-01"}"#);
    assert_eq!(a.get("result").and_then(|x| x.get("protocolVersion")).and_then(Value::as_str), Some("2025-11-25"));
    // A notification gets no answer: the next thing to come back is the ping's.
    c.send_line(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#);
    let a = c.request("ping", "{}");
    assert!(a.get("result").is_some());
}

#[test]
fn every_tool_is_listed_with_a_schema() {
    let dir = folder("list");
    let mut c = Client::start(&dir);
    let a = c.request("tools/list", "{}");
    let tools = a.get("result").and_then(|r| r.get("tools")).and_then(Value::as_array).unwrap().to_vec();
    assert!(tools.len() >= 20);
    for t in &tools {
        let name = t.get("name").and_then(Value::as_str).unwrap();
        assert!(!t.get("description").and_then(Value::as_str).unwrap().is_empty(), "{name}");
        let schema = t.get("inputSchema").unwrap();
        assert_eq!(schema.get("type").and_then(Value::as_str), Some("object"), "{name}");
        // Everything required is described.
        let props = schema.get("properties").unwrap();
        for r in schema.get("required").and_then(Value::as_array).unwrap() {
            assert!(props.get(r.as_str().unwrap()).is_some(), "{name} requires an undescribed {r:?}");
        }
    }
    for needed in ["feature_add", "feature_edit", "faces_list", "edges_list", "rebuild", "export", "render", "sketch_solve"] {
        assert!(tools.iter().any(|t| t.get("name").and_then(Value::as_str) == Some(needed)), "{needed}");
    }
}

#[test]
fn protocol_errors_get_protocol_answers() {
    let dir = folder("errors");
    let mut c = Client::start(&dir);
    c.send_line("this isn't json");
    let a = c.read();
    assert_eq!(a.get("error").and_then(|e| e.get("code")).and_then(Value::as_f64), Some(-32700.0));
    assert_eq!(a.get("id"), Some(&Value::Null));
    c.send_line(r#"{"id":5,"method":"ping"}"#);
    assert_eq!(c.read().get("error").and_then(|e| e.get("code")).and_then(Value::as_f64), Some(-32600.0));
    let a = c.request("no/such/method", "{}");
    assert_eq!(a.get("error").and_then(|e| e.get("code")).and_then(Value::as_f64), Some(-32601.0));
    let a = c.request("tools/call", r#"{"name":"nope","arguments":{}}"#);
    assert_eq!(a.get("error").and_then(|e| e.get("code")).and_then(Value::as_f64), Some(-32602.0));
    // It's still alive after all that.
    assert!(c.request("ping", "{}").get("result").is_some());
}

#[test]
fn an_agent_can_build_edit_and_inspect_a_part() {
    let dir = folder("build");
    let mut c = Client::start(&dir);
    assert!(c.ok("format_guide", "{}").contains("Edge selections"));
    c.ok("parameter_set", r#"{"name":"w","value":40}"#);
    c.ok("parameter_set", r#"{"name":"half","value":"w / 2"}"#);
    let text = c.ok("feature_add", r#"{"feature":{"type":"sketch","regions":[{"outer":{"type":"rect","w":"w","h":20}}]}}"#);
    assert!(text.contains("f1"), "{text}");
    let text = c.ok("feature_add", r#"{"feature":{"type":"extrude","sketch":1,"length":10},"name":"Plate"}"#);
    assert!((volume(&text) - 8000.0).abs() < 1e-6, "{text}");
    // Names come back, and they work in a fillet.
    let faces = c.ok("faces_list", "{}");
    assert!(faces.contains("f2:end") && faces.contains("f2:side.0"), "{faces}");
    let edges = json::parse(&c.ok("edges_list", "{}")).unwrap();
    let list = edges.as_array().unwrap();
    assert_eq!(list.len(), 12);
    let top_front = list.iter().find(|e| e.get("name").and_then(Value::as_str) == Some("f2:end|f2:side.0")).expect("that edge");
    let select = top_front.get("select").unwrap().to_text();
    let text = c.ok("feature_add", &format!(r#"{{"feature":{{"type":"fillet","edges":{select},"radius":2}}}}"#));
    let corner = 4.0 * (1.0 - std::f64::consts::PI / 4.0);
    assert!((volume(&text) - (8000.0 - corner * 40.0)).abs() < 1e-5, "{text}");
    // Change a dimension: the name still finds the edge.
    let text = c.ok("parameter_set", r#"{"name":"w","value":60}"#);
    assert!((volume(&text) - (12000.0 - corner * 60.0)).abs() < 1e-5, "{text}");
    // Suppress, move, delete.
    let text = c.ok("feature_suppress", r#"{"id":3,"suppressed":true}"#);
    assert!((volume(&text) - 12000.0).abs() < 1e-5, "{text}");
    c.ok("feature_suppress", r#"{"id":3,"suppressed":false}"#);
    assert!(c.refused("feature_delete", r#"{"id":2}"#).contains("refer to it"));
    assert!(c.refused("feature_move", r#"{"id":1,"position":2}"#).contains("before"));
    c.ok("feature_rename", r#"{"id":3,"name":"Round"}"#);
    let steps = c.ok("features_list", "{}");
    assert!(steps.contains("Round") && steps.contains("Plate"), "{steps}");
}

#[test]
fn a_failing_feature_reports_why_and_keeps_the_last_good_solid() {
    let dir = folder("failing");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":10,"dz":10}}"#);
    let text = c.ok("feature_add", r#"{"feature":{"type":"fillet","edges":{"on_face":"f1:end"},"radius":50}}"#);
    assert!(text.contains("\"failed\""), "{text}");
    assert!(text.contains("before it"), "{text}");
    assert!((volume(&text) - 1000.0).abs() < 1e-9, "{text}");
    let (content, failed) = c.call("render", "{}");
    assert!(!failed);
    assert!(content.iter().any(|x| x.get("text").and_then(Value::as_str).is_some_and(|t| t.contains("from before"))));
    // A cut that misses isn't an error, and the answer points it out.
    c.ok("feature_delete", r#"{"id":2}"#);
    let text = c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"sphere","radius":1,"at":{"origin":[100,0,0]},"mode":"remove"}}"#);
    assert!(text.contains("left no faces"), "{text}");
    c.ok("feature_delete", r#"{"id":3}"#);
    c.ok("feature_add", r#"{"feature":{"type":"fillet","edges":{"on_face":"f1:end"},"radius":50}}"#);
    // Bad input is refused and leaves the document alone.
    assert!(c.refused("feature_add", r#"{"feature":{"type":"warp"}}"#).contains("unknown feature type"));
    assert!(c.refused("feature_add", r#"{"feature":{"type":"extrude","sketch":99,"length":1}}"#).contains("before"));
    assert!(c.refused("feature_edit", r#"{"id":77,"feature":{"type":"solid","shape":"sphere","radius":1}}"#).contains("no feature"));
    assert!(c.refused("parameter_set", r#"{"name":"bad name","value":1}"#).contains("parameter name"));
    assert!(c.refused("parameter_set", r#"{"name":"a","value":"1 +"}"#).contains("expression"));
    assert_eq!(json::parse(&c.ok("document_show", "{}")).unwrap().get("features").and_then(Value::as_array).map(<[Value]>::len), Some(2));
}

#[test]
fn render_returns_a_real_png() {
    let dir = folder("render");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":30,"dy":20,"dz":10}}"#);
    for args in [
        r#"{}"#,
        r#"{"view":"top","width":320,"height":240}"#,
        r#"{"view":"four","width":640,"height":480,"labels":true}"#,
        r#"{"azimuth":120,"elevation":-20,"colors":"faces"}"#,
        r#"{"highlight_faces":["f1:end"],"highlight_edges":{"on_face":"f1:start"}}"#,
    ] {
        let (content, failed) = c.call("render", args);
        assert!(!failed, "{args}: {content:?}");
        let image = content.iter().find(|x| x.get("type").and_then(Value::as_str) == Some("image")).expect("an image");
        assert_eq!(image.get("mimeType").and_then(Value::as_str), Some("image/png"));
        let bytes = decode_base64(image.get("data").and_then(Value::as_str).unwrap());
        assert_eq!(&bytes[..8], &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]);
        assert!(bytes.len() > 500, "{args}: {} bytes", bytes.len());
        assert!(content.iter().any(|x| x.get("text").and_then(Value::as_str).is_some_and(|t| t.contains("visible_faces"))));
    }
    assert!(c.refused("render", r#"{"view":"sideways"}"#).contains("unknown view"));
    assert!(c.refused("render", r#"{"width":1}"#).contains("out of range"));
    assert!(c.refused("render", r#"{"highlight_faces":["f9:end"]}"#).contains("no face"));
    c.ok("render", r#"{"path":"picture.png"}"#);
    assert!(std::fs::metadata(dir.join("picture.png")).unwrap().len() > 500);
}

#[test]
fn render_with_no_solid_says_so() {
    let dir = folder("empty");
    let mut c = Client::start(&dir);
    assert!(c.refused("render", "{}").contains("no solid"));
    assert!(c.refused("faces_list", "{}").contains("no solid"));
    assert!(c.refused("export", r#"{"format":"step"}"#).contains("no solid"));
}

#[test]
fn files_stay_inside_the_root() {
    let dir = folder("root");
    let outside = folder("outside");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":3,"dy":2,"dz":1}}"#);
    c.ok("document_save", r#"{"path":"part.coreshape"}"#);
    assert!(dir.join("part.coreshape").exists());
    let step = c.ok("export", r#"{"format":"step"}"#);
    assert!(step.starts_with("ISO-10303-21;"));
    c.ok("export", r#"{"format":"step","path":"part.step"}"#);
    c.ok("export", r#"{"format":"stl","path":"part.stl"}"#);
    assert!(std::fs::read_to_string(dir.join("part.step")).unwrap().starts_with("ISO-10303-21;"));
    assert!(c.refused("export", r#"{"format":"stl"}"#).contains("needs a"));
    let escape = outside.join("x.step");
    for path in ["../x.step", "a/../../x.step", escape.to_str().unwrap(), "/etc/passwd"] {
        let args = format!(r#"{{"format":"step","path":"{path}"}}"#);
        let why = c.refused("export", &args);
        assert!(why.contains("outside") || why.contains("should end"), "{path}: {why}");
    }
    assert!(!escape.exists());
    assert!(c.refused("document_open", r#"{"path":"/etc/passwd"}"#).contains("outside"));
    assert!(c.refused("document_save", r#"{"path":"part.exe"}"#).contains("should end"));
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(&outside, dir.join("link")).unwrap();
        let why = c.refused("export", r#"{"format":"step","path":"link/x.step"}"#);
        assert!(why.contains("outside"), "{why}");
        assert!(!escape.exists());
    }
    // What was saved opens again as the same part.
    c.ok("document_new", "{}");
    let text = c.ok("document_open", r#"{"path":"part.coreshape"}"#);
    assert!((volume(&text) - 6.0).abs() < 1e-9, "{text}");
    assert!(c.refused("document_open", r#"{"text":"{}"}"#).contains("not a CoreShape document"));
}

#[test]
fn sketches_solve_drag_and_become_solids_through_the_tools() {
    let dir = folder("sketch");
    let mut c = Client::start(&dir);
    // A sketch with its header left out is fine here.
    let sketch = r#"{"entities":[{"id":1,"type":"line","start":[0,1],"end":[10,0]}],"constraints":[
        {"id":1,"type":"horizontal","a":"1"},{"id":2,"type":"coincident","a":"1.start","b":"origin"}]}"#
        .replace('\n', "");
    let solved = json::parse(&c.ok("sketch_solve", &format!(r#"{{"sketch":{sketch}}}"#))).unwrap();
    assert_eq!(solved.get("analysis").and_then(|a| a.get("dof")).and_then(Value::as_f64), Some(1.0));
    let dragged = json::parse(&c.ok("sketch_drag", &format!(r#"{{"sketch":{sketch},"grab":"1.end","to":[25,7]}}"#))).unwrap();
    assert_eq!(dragged.get("reached").and_then(Value::as_bool), Some(false), "it can only slide along x");
    // Conflicts name the constraints.
    let bad = r#"{"entities":[{"id":1,"type":"line","start":[0,0],"end":[10,0]}],"constraints":[{"id":1,"type":"distance","a":"1","value":5},{"id":2,"type":"distance","a":"1","value":9}]}"#;
    let why = c.refused("sketch_solve", &format!(r#"{{"sketch":{bad}}}"#));
    let v = json::parse(&why).unwrap();
    assert_eq!(v.get("kind").and_then(Value::as_str), Some("conflict"));
    assert_eq!(v.get("constraints").and_then(Value::as_array).map(<[Value]>::len), Some(2));
    // Regions say what's wrong.
    assert!(c.refused("sketch_regions", &format!(r#"{{"sketch":{sketch}}}"#)).contains("isn't closed"));
    // The arrangement never refuses: it says what it left out.
    let arrangement = json::parse(&c.ok("sketch_arrangement", &format!(r#"{{"sketch":{sketch}}}"#))).unwrap();
    assert_eq!(arrangement.get("regions").and_then(Value::as_array).map(<[Value]>::len), Some(0));
    assert_eq!(arrangement.get("ignored").and_then(Value::as_array).map(<[Value]>::len), Some(1));
    // Parameters drive dimensions.
    let param = r#"{"entities":[{"id":1,"type":"circle","center":[0,0],"radius":2}],"constraints":[{"id":1,"type":"radius","of":"1","value":"r * 2"}]}"#;
    let s = c.ok("sketch_solve", &format!(r#"{{"sketch":{param},"parameters":{{"r":5}}}}"#));
    assert!(s.contains("\"radius\":10"), "{s}");
    assert!(c.refused("sketch_solve", &format!(r#"{{"sketch":{param}}}"#)).contains("no parameter"));
    // And a constrained sketch as a feature.
    c.ok("parameter_set", r#"{"name":"size","value":10}"#);
    let square = r#"{"format":"coreshape-sketch","version":1,"entities":[
        {"id":1,"type":"line","start":[0,0],"end":[9,0]},{"id":2,"type":"line","start":[9,0],"end":[9,9]},
        {"id":3,"type":"line","start":[9,9],"end":[0,9]},{"id":4,"type":"line","start":[0,9],"end":[0,0]}],
        "constraints":[{"id":1,"type":"coincident","a":"1.end","b":"2.start"},{"id":2,"type":"coincident","a":"2.end","b":"3.start"},
        {"id":3,"type":"coincident","a":"3.end","b":"4.start"},{"id":4,"type":"coincident","a":"4.end","b":"1.start"},
        {"id":5,"type":"horizontal","a":"1"},{"id":6,"type":"vertical","a":"2"},{"id":7,"type":"horizontal","a":"3"},{"id":8,"type":"vertical","a":"4"},
        {"id":9,"type":"distance","a":"1","value":"size"},{"id":10,"type":"distance","a":"2","value":"size"}]}"#
        .replace('\n', "");
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"constrained_sketch","sketch":{square}}}}}"#));
    let text = c.ok("feature_add", r#"{"feature":{"type":"extrude","sketch":1,"length":3}}"#);
    assert!((volume(&text) - 300.0).abs() < 1e-6, "{text}");
    assert!(c.ok("faces_list", "{}").contains("f2:side.3"));
}

#[test]
fn several_bodies_can_be_made_chosen_listed_and_exported() {
    let dir = folder("bodies");
    let mut c = Client::start(&dir);
    let block = |x: f64, extra: &str| format!(r#"{{"feature":{{"type":"solid","shape":"box","dx":10,"dy":10,"dz":10,"at":{{"origin":[{x},0,0]}},"mode":"new_body"{extra}}}}}"#);
    c.ok("feature_add", &block(0.0, ""));
    c.ok("feature_add", &block(50.0, ""));

    let bodies = json::parse(&c.ok("bodies_list", "{}")).unwrap();
    let list = bodies.as_array().unwrap();
    assert_eq!(list.len(), 2);
    assert_eq!(list[0].get("body").unwrap().as_str(), Some("f1"));
    assert_eq!(list[1].get("body").unwrap().as_str(), Some("f2"));
    assert_eq!(list[1].get("volume").unwrap().as_f64(), Some(1000.0));

    // A feature acts on all it reaches, and it can list them.
    let cut = |x: f64, extra: &str| format!(r#"{{"feature":{{"type":"solid","shape":"cylinder","radius":1,"height":20,"at":{{"origin":[{x},5,-5]}},"mode":"remove"{extra}}}}}"#);
    c.ok("feature_add", &cut(55.0, ""));
    c.ok("feature_add", &cut(5.0, r#","bodies":["f1"]"#));
    let bodies = json::parse(&c.ok("bodies_list", "{}")).unwrap();
    let hole = 10.0 * std::f64::consts::PI;
    for b in bodies.as_array().unwrap() {
        assert!((b.get("volume").unwrap().as_f64().unwrap() - (1000.0 - hole)).abs() < 1e-6);
    }
    // A body it can't have leaves nothing behind.
    let before = c.ok("features_list", "{}");
    assert!(c.refused("feature_add", &cut(5.0, r#","bodies":["f9"]"#)).contains("f9"));
    assert!(c.refused("feature_add", &cut(5.0, r#","bodies":["nine"]"#)).contains("nine"));
    assert_eq!(c.ok("features_list", "{}"), before);
    // Editing keeps the choice unless it says otherwise.
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.5, "")[1..]));
    assert!(c.ok("document_show", "{}").contains(r#""bodies""#));
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","bodies":[]"#)[1..]));
    assert!(!c.ok("document_show", "{}").contains(r#""bodies""#));
    assert!(c.refused("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","bodies":["f9"]"#)[1..])).contains("f9"));
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","bodies":["f1"]"#)[1..]));

    // Moving, joining and deleting parts, through the same tools.
    c.ok("feature_add", r#"{"feature":{"type":"transform","bodies":["f2"],"motion":{"type":"translate","by":[1,0,0]}}}"#);
    let moved = json::parse(&c.ok("bodies_list", "{}")).unwrap();
    assert!((moved.as_array().unwrap()[1].get("box_min").unwrap().as_array().unwrap()[0].as_f64().unwrap() - 51.0).abs() < 1e-3);
    c.ok("feature_add", r#"{"feature":{"type":"transform","bodies":["f2"],"motion":{"type":"translate","by":[-1,0,0]},"copy":false}}"#);
    // An app's own data, kept as it is.
    c.ok("app_set", r#"{"id":1,"app":{"color":"amber"}}"#);
    assert!(c.ok("document_show", "{}").contains("amber"));

    // The lists say whose each thing is, and can stick to one.
    let faces = json::parse(&c.ok("faces_list", "{}")).unwrap();
    let all = faces.as_array().unwrap();
    assert!(all.iter().any(|f| f.get("body").and_then(Value::as_str) == Some("f1")));
    assert!(all.iter().any(|f| f.get("body").and_then(Value::as_str) == Some("f2")));
    let one = json::parse(&c.ok("faces_list", r#"{"body":"f2"}"#)).unwrap();
    assert!(!one.as_array().unwrap().is_empty() && one.as_array().unwrap().len() < all.len());
    assert!(one.as_array().unwrap().iter().all(|f| f.get("body").and_then(Value::as_str) == Some("f2")));
    let edges = json::parse(&c.ok("edges_list", r#"{"body":"f1"}"#)).unwrap();
    assert!(edges.as_array().unwrap().iter().all(|e| e.get("body").and_then(Value::as_str) == Some("f1")));
    assert!(c.refused("faces_list", r#"{"body":"f7"}"#).contains("f1, f2"));

    // A rebuild's summary tells them apart.
    let summary = json::parse(&c.ok("rebuild", "{}")).unwrap();
    assert_eq!(summary.get("bodies").unwrap().as_array().unwrap().len(), 2);

    // A STEP file has both, or the one asked for, and pictures show them.
    let both = c.ok("export", r#"{"format":"step"}"#);
    assert_eq!(both.matches("MANIFOLD_SOLID_BREP").count(), 2);
    let single = c.ok("export", r#"{"format":"step","body":"f2"}"#);
    assert_eq!(single.matches("MANIFOLD_SOLID_BREP").count(), 1);
    for args in [r#"{}"#, r#"{"body":"f2"}"#, r#"{"colors":"bodies"}"#, r#"{"colors":"plain"}"#] {
        let (_, failed) = c.call("render", args);
        assert!(!failed, "{args}");
    }
    assert!(c.refused("render", r#"{"colors":"tartan"}"#).contains("bodies"));
}

#[test]
fn a_document_with_one_body_lists_as_it_did() {
    let dir = folder("one_body");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":10,"dz":10}}"#);
    let faces = c.ok("faces_list", "{}");
    assert!(!faces.contains("\"body\""), "{faces}");
    let summary = json::parse(&c.ok("rebuild", "{}")).unwrap();
    assert!(summary.get("bodies").is_none());
    let bodies = json::parse(&c.ok("bodies_list", "{}")).unwrap();
    assert_eq!(bodies.as_array().unwrap().len(), 1);
}

// ----- Assemblies -----

fn parsed(text: &str) -> Value {
    json::parse(text).unwrap_or_else(|e| panic!("{e}: {text}"))
}

fn numbers(v: &Value) -> Vec<f64> {
    v.as_array().unwrap().iter().map(|n| n.as_f64().unwrap()).collect()
}

/// Where an instance's frame is, from an assembly's state.
fn origin_of(state: &Value, name: &str) -> Vec<f64> {
    let list = state.get("instances").and_then(Value::as_array).unwrap();
    let it = list.iter().find(|i| i.get("name").and_then(Value::as_str) == Some(name)).unwrap_or_else(|| panic!("no {name} in {}", state.to_text()));
    numbers(it.get("at").unwrap().get("origin").unwrap())
}

fn near(got: Vec<f64>, want: [f64; 3]) {
    assert!(got.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-6), "{got:?} isn't {want:?}");
}

fn mate_value(state: &Value, mate: f64, dof: &str) -> f64 {
    let list = state.get("mates").and_then(Value::as_array).unwrap();
    let it = list.iter().find(|m| m.get("id").and_then(Value::as_f64) == Some(mate)).unwrap();
    it.get("values").unwrap().get(dof).and_then(Value::as_f64).unwrap()
}

fn id_of(state: &Value) -> f64 {
    state.get("id").and_then(Value::as_f64).unwrap()
}

/// A plate of height `h`, and a pin, and an assembly with one of each mated by a revolute on the plate's top face.
fn plate_and_pin(c: &mut Client) -> (f64, f64) {
    c.ok("parameter_set", r#"{"name":"h","value":10}"#);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":40,"dy":40,"dz":"h","mode":"new_body"}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"cylinder","radius":3,"height":20,"mode":"new_body"}}"#);
    c.ok("tab_new", r#"{"type":"assembly"}"#);
    let plate = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f1","fixed":true,"name":"Plate"}"#)));
    let pin = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f2","name":"Pin"}"#)));
    (plate, pin)
}

#[test]
fn an_assembly_can_be_built_moved_and_kept_through_the_tools() {
    let dir = folder("assembly");
    let mut c = Client::start(&dir);
    let (plate, pin) = plate_and_pin(&mut c);
    let tabs = parsed(&c.ok("tab_list", "{}"));
    let tabs = tabs.as_array().unwrap();
    assert_eq!(tabs.len(), 2);
    assert_eq!(tabs[1].get("selected").and_then(Value::as_bool), Some(true));

    // The places on offer on the plate's top, ready to use.
    let places = parsed(&c.ok("connector_candidates", &format!(r#"{{"instance":{plate},"face":"f1:end"}}"#)));
    let places = places.as_array().unwrap();
    assert_eq!(places.len(), 9);
    assert_eq!(numbers(places[0].get("at").unwrap().get("origin").unwrap()), vec![20.0, 20.0, 10.0]);
    let on_plate = places[0].get("on").unwrap().to_text();

    // The pin's end, on the plate. The first end moves, and two faces need flip to lie against each other.
    let answer = parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{{"instance":{pin},"on":{{"face":"f2:start"}}}},"b":{{"instance":{plate},"on":{on_plate}}},"flip":true}}"#)));
    let mate = id_of(&answer);
    near(origin_of(&answer, "Pin"), [20.0, 20.0, 10.0]);
    assert_eq!(answer.get("freedom").and_then(Value::as_f64), Some(1.0));

    // It turns, and keeps count.
    let turned = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{mate},"dof":"angle","value":400}}"#)));
    assert!((turned.get("reached").and_then(Value::as_f64).unwrap() - 400.0).abs() < 1e-5, "{}", turned.to_text());
    assert!((mate_value(&turned, mate, "angle") - 400.0).abs() < 1e-5);
    let dragged = parsed(&c.ok("assembly_drag", &format!(r#"{{"instance":{pin},"point":[3,0,0],"to":[20,23,10]}}"#)));
    assert!(dragged.get("remaining").and_then(Value::as_f64).unwrap() < 1e-5, "{}", dragged.to_text());

    // A taller plate, in the Part Studio, brings the pin up with it.
    c.ok("parameter_set", r#"{"name":"h","value":25}"#);
    let state = parsed(&c.ok("assembly_solve", "{}"));
    near(origin_of(&state, "Pin"), [20.0, 20.0, 25.0]);
    assert!(state.get("problems").is_none());

    // Touching isn't interference, and two of the same part are one line of a bill of materials.
    let check = parsed(&c.ok("interference_check", "{}"));
    assert!(check.get("interferences").unwrap().as_array().unwrap().is_empty(), "{}", check.to_text());
    c.ok("instance_add", r#"{"part":"f2","name":"Spare","at":{"origin":[5,5,20]}}"#);
    let bom = parsed(&c.ok("bom", "{}"));
    let rows = bom.as_array().unwrap();
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[1].get("count").and_then(Value::as_f64), Some(2.0));
    let check = parsed(&c.ok("interference_check", "{}"));
    let found = check.get("interferences").unwrap().as_array().unwrap();
    assert_eq!(found.len(), 1, "{}", check.to_text());
    // The spare is in the plate for 5 of its height.
    let want = std::f64::consts::PI * 9.0 * 5.0;
    assert!((found[0].get("volume").and_then(Value::as_f64).unwrap() - want).abs() < 1e-5);

    // Drawn, and written as an assembly, with each part once.
    let (content, failed) = c.call("render", r#"{"width":200,"height":150}"#);
    assert!(!failed);
    assert!(content.iter().any(|x| x.get("type").and_then(Value::as_str) == Some("image")));
    let step = c.ok("export", r#"{"format":"step"}"#);
    assert_eq!(step.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE").count(), 3);
    assert_eq!(step.matches("MANIFOLD_SOLID_BREP").count(), 2);

    // Saved and opened again, everything's there and the same.
    c.ok("document_save", r#"{"path":"joint.coreshape"}"#);
    c.ok("document_new", "{}");
    assert_eq!(parsed(&c.ok("tab_list", "{}")).as_array().unwrap().len(), 1);
    c.ok("document_open", r#"{"path":"joint.coreshape"}"#);
    assert_eq!(parsed(&c.ok("tab_list", "{}")).as_array().unwrap().len(), 2);
    let state = parsed(&c.ok("assembly_solve", r#"{"tab":2}"#));
    near(origin_of(&state, "Pin"), [20.0, 20.0, 25.0]);
    // Dragged from 400 degrees to the nearest place that gets there, a turn and a quarter.
    assert!((mate_value(&state, mate, "angle") - 450.0).abs() < 1e-5, "{}", state.to_text());
    let shown = c.ok("project_show", "{}");
    assert!(shown.contains("coreshape-project"));
}

#[test]
fn the_part_studio_tools_still_work_with_an_assembly_selected() {
    let dir = folder("assembly_studio");
    let mut c = Client::start(&dir);
    plate_and_pin(&mut c);
    // Features go to the Part Studio, and bodies are the Part Studio's.
    let added = c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":1,"dy":1,"dz":1,"at":{"origin":[100,0,0]},"mode":"new_body"}}"#);
    assert!(added.contains("\"id\": 3"), "{added}");
    assert_eq!(parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().len(), 3);
    assert!(parsed(&c.ok("faces_list", r#"{"body":"f3"}"#)).as_array().is_some());
    // A second Part Studio has its own features, and the first is still there when it comes back.
    c.ok("tab_new", r#"{"type":"part_studio","name":"Other"}"#);
    assert!(c.refused("bodies_list", "{}").contains("no solid"));
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":2,"dy":2,"dz":2,"mode":"new_body"}}"#);
    assert_eq!(parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().len(), 1);
    c.ok("tab_select", r#"{"tab":1}"#);
    assert_eq!(parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().len(), 3);
    // An instance from the other Part Studio, by its tab, whichever is being worked on.
    c.ok("tab_select", r#"{"tab":2}"#);
    let state = parsed(&c.ok("instance_add", r#"{"part":"f1","studio":3,"name":"From the other"}"#));
    assert!(state.get("instances").unwrap().as_array().unwrap().iter().any(|i| i.get("tab").and_then(Value::as_f64) == Some(3.0)));
    // The file goes on being a plain document only as long as that's all there is.
    let saved = c.ok("export", r#"{"format":"coreshape"}"#);
    assert!(saved.contains("coreshape-project"));
}

#[test]
fn a_project_of_one_part_studio_stays_a_plain_document_through_the_tools() {
    let dir = folder("assembly_plain");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":1,"dy":1,"dz":1}}"#);
    let shown = c.ok("export", r#"{"format":"coreshape"}"#);
    assert!(shown.contains("coreshape-document") && !shown.contains("coreshape-project"));
    assert_eq!(shown, c.ok("document_show", "{}"));
    c.ok("document_save", r#"{"path":"plain.coreshape"}"#);
    assert_eq!(std::fs::read_to_string(dir.join("plain.coreshape")).unwrap(), shown);
    c.ok("document_open", r#"{"path":"plain.coreshape"}"#);
    assert_eq!(c.ok("document_show", "{}"), shown);
}

#[test]
fn assembly_tools_refuse_what_cant_be_done_and_say_why() {
    let dir = folder("assembly_refused");
    let mut c = Client::start(&dir);
    assert!(c.refused("assembly_solve", "{}").contains("no assembly is selected"));
    assert!(c.refused("mate_add", r#"{"type":"revolute","a":1,"b":2}"#).contains("no assembly is selected"));
    let (plate, pin) = plate_and_pin(&mut c);
    assert!(c.refused("instance_add", r#"{"part":"f9"}"#).contains("no part f9"));
    assert!(c.refused("instance_add", "{}").contains("part"));
    assert!(c.refused("instance_add", r#"{"assembly":1}"#).contains("Part Studio"));
    // It can't go in itself, and it's not kept for trying.
    assert!(c.refused("instance_add", r#"{"assembly":2}"#).contains("inside itself"));
    assert_eq!(parsed(&c.ok("assembly_solve", "{}")).get("instances").unwrap().as_array().unwrap().len(), 2);

    assert!(c.refused("connector_add", &format!(r#"{{"instance":{pin},"on":{{"face":"f9:end"}}}}"#)).contains("f9:end"));
    assert!(c.refused("connector_add", r#"{"instance":77,"on":{"face":"f1:end"}}"#).contains("77"));
    assert!(c.refused("connector_candidates", &format!(r#"{{"instance":{pin},"face":"nonsense"}}"#)).contains("nonsense"));
    assert!(c.refused("connector_candidates", &format!(r#"{{"instance":{pin}}}"#)).contains("face"));

    let a = format!(r#"{{"instance":{plate},"on":{{"face":"f1:end"}}}}"#);
    let b = format!(r#"{{"instance":{pin},"on":{{"face":"f2:start"}}}}"#);
    assert!(c.refused("mate_add", &format!(r#"{{"type":"welded","a":{a},"b":{b}}}"#)).contains("welded"));
    assert!(c.refused("mate_add", &format!(r#"{{"type":"revolute","a":{a}}}"#)).contains("\"b\""));
    assert!(c.refused("mate_add", &format!(r#"{{"type":"revolute","a":{a},"b":{a}}}"#)).contains("different instances"));
    assert!(c.refused("mate_add", &format!(r#"{{"type":"revolute","a":{a},"b":{b},"limits":[{{"dof":"z","min":0,"max":1}}]}}"#)).contains("no motion along z"));
    assert!(c.refused("mate_add", &format!(r#"{{"type":"revolute","a":{a},"b":{b},"limits":[{{"dof":"angle","min":5,"max":1}}]}}"#)).contains("more than"));
    // None of those left a connector behind.
    let shown = parsed(&c.ok("project_show", "{}"));
    let tabs = shown.get("tabs").unwrap().as_array().unwrap();
    assert!(tabs[1].get("assembly").unwrap().get("connectors").unwrap().as_array().unwrap().is_empty());

    let made = parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{b},"b":{a},"flip":true}}"#)));
    let mate = id_of(&made);
    assert!(c.refused("mate_set_value", &format!(r#"{{"id":{mate},"dof":"z","value":1}}"#)).contains("can't move along z"));
    assert!(c.refused("mate_set_value", r#"{"id":99,"dof":"angle","value":1}"#).contains("99"));
    assert!(c.refused("relation_add", &format!(r#"{{"type":"gear","ratio":1,"a":{mate},"b":99}}"#)).contains("99"));
    assert!(c.refused("relation_add", &format!(r#"{{"type":"gear","a":{mate},"b":{mate}}}"#)).contains("ratio"));
    assert!(c.refused("group_add", &format!(r#"{{"instances":[{plate}]}}"#)).contains("at least two"));
    assert!(c.refused("instance_edit", &format!(r#"{{"id":{pin},"locked":true}}"#)).contains("only a sub-assembly"));
    assert!(c.refused("assembly_drag", &format!(r#"{{"instance":{pin}}}"#)).contains("to"));
    assert!(c.refused("assembly_drag", r#"{"instance":77,"to":[1,2,3]}"#).contains("77"));
    let connector = shown_connector(&mut c);
    assert!(c.refused("connector_delete", &format!(r#"{{"id":{connector}}}"#)).contains("used by"));

    // A Part Studio in use stays, and the last one has to stay anyway.
    assert!(c.refused("tab_delete", r#"{"tab":1}"#).contains("Part Studio"));
    c.ok("tab_new", r#"{"type":"part_studio"}"#);
    assert!(c.refused("tab_delete", r#"{"tab":1}"#).contains("instances from it"));
    assert!(c.refused("tab_select", r#"{"tab":99}"#).contains("99"));

    // A bad file brings nothing in.
    let before = parsed(&c.ok("tab_list", "{}")).as_array().unwrap().len();
    assert!(!c.refused("assembly_import_step", r#"{"text":"not a file"}"#).is_empty());
    assert_eq!(parsed(&c.ok("tab_list", "{}")).as_array().unwrap().len(), before);
}

fn shown_connector(c: &mut Client) -> f64 {
    let shown = parsed(&c.ok("project_show", "{}"));
    let tab = shown.get("tabs").unwrap().as_array().unwrap().iter().find(|t| t.get("assembly").is_some()).unwrap().clone();
    tab.get("assembly").unwrap().get("connectors").unwrap().as_array().unwrap()[0].get("id").and_then(Value::as_f64).unwrap()
}

#[test]
fn a_failing_mate_and_a_missing_part_come_back_as_state_and_not_as_errors() {
    let dir = folder("assembly_failing");
    let mut c = Client::start(&dir);
    let (plate, pin) = plate_and_pin(&mut c);
    c.ok("instance_edit", &format!(r#"{{"id":{pin},"fixed":true,"at":{{"origin":[100,0,0]}}}}"#));
    let a = format!(r#"{{"instance":{plate},"on":{{"face":"f1:end"}}}}"#);
    let b = format!(r#"{{"instance":{pin},"on":{{"face":"f2:start"}}}}"#);
    // Both fixed, and nowhere near: it's kept, and it's named.
    let answer = parsed(&c.ok("mate_add", &format!(r#"{{"type":"fastened","a":{a},"b":{b}}}"#)));
    let mate = answer.get("mates").unwrap().as_array().unwrap()[0].clone();
    assert!(mate.get("failing").and_then(Value::as_str).is_some(), "{}", answer.to_text());
    // Let go, it's met.
    let answer = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{pin},"fixed":false}}"#)));
    assert!(answer.get("mates").unwrap().as_array().unwrap()[0].get("failing").is_none(), "{}", answer.to_text());
    // Its part suppressed in the Part Studio, what's left still comes back, saying what's wrong.
    c.ok("feature_suppress", r#"{"id":2,"suppressed":true}"#);
    let answer = parsed(&c.ok("assembly_solve", "{}"));
    assert_eq!(answer.get("instances").unwrap().as_array().unwrap().len(), 1);
    assert!(answer.get("problems").unwrap().as_array().unwrap()[0].as_str().unwrap().contains("Pin"));
    assert!(answer.get("mates").unwrap().as_array().unwrap()[0].get("failing").is_some());
}

#[test]
fn a_step_assembly_comes_in_as_tabs_through_the_tools() {
    let dir = folder("assembly_import");
    let mut c = Client::start(&dir);
    let (_, pin) = plate_and_pin(&mut c);
    c.ok("instance_edit", &format!(r#"{{"id":{pin},"at":{{"origin":[1,2,3]}}}}"#));
    c.ok("instance_add", r#"{"part":"f2","at":{"origin":[20,0,0]}}"#);
    c.ok("export", r#"{"format":"step","path":"three.step"}"#);
    c.ok("document_new", "{}");
    let state = parsed(&c.ok("assembly_import_step", r#"{"path":"three.step","name":"Three"}"#));
    let list = state.get("instances").unwrap().as_array().unwrap();
    assert_eq!(list.len(), 3);
    assert_eq!(state.get("freedom").and_then(Value::as_f64), Some(0.0));
    near(numbers(list[1].get("at").unwrap().get("origin").unwrap()), [1.0, 2.0, 3.0]);
    let tabs = parsed(&c.ok("tab_list", "{}"));
    let tabs = tabs.as_array().unwrap();
    assert_eq!(tabs.len(), 3);
    assert_eq!(tabs[1].get("name").and_then(Value::as_str), Some("Three"));
    assert_eq!(tabs[2].get("selected").and_then(Value::as_bool), Some(true));
    // Its parts are there to work on, once.
    c.ok("tab_select", r#"{"tab":2}"#);
    assert_eq!(parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().len(), 2);
}

#[test]
fn a_sub_assembly_can_be_added_locked_and_reached_into_through_the_tools() {
    let dir = folder("assembly_sub");
    let mut c = Client::start(&dir);
    let (plate, pin) = plate_and_pin(&mut c);
    c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{{"instance":{pin},"on":{{"face":"f2:start"}}}},"b":{{"instance":{plate},"on":{{"face":"f1:end"}}}},"flip":true}}"#));
    c.ok("tab_new", r#"{"type":"assembly","name":"Top"}"#);
    let sub = id_of(&parsed(&c.ok("instance_add", r#"{"assembly":2,"at":{"origin":[0,0,100]}}"#)));
    let state = parsed(&c.ok("assembly_solve", "{}"));
    assert_eq!(state.get("instances").unwrap().as_array().unwrap().len(), 2);
    // Six for the base, which isn't fixed up here, and one for the turn.
    assert_eq!(state.get("freedom").and_then(Value::as_f64), Some(7.0));
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{sub},"locked":true}}"#)));
    assert_eq!(state.get("freedom").and_then(Value::as_f64), Some(6.0));
    near(origin_of(&state, "Pin"), [20.0, 20.0, 110.0]);
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{sub},"locked":false,"fixed":true}}"#)));
    assert_eq!(state.get("freedom").and_then(Value::as_f64), Some(0.0));
    // A connector reaching in has to get as far as a part.
    assert!(c.refused("connector_add", &format!(r#"{{"instance":{sub},"on":{{"face":"f1:end"}}}}"#)).contains("go all the way down"));
    c.ok("connector_add", &format!(r#"{{"instance":[{sub},{pin}],"on":{{"face":"f2:end"}}}}"#));
}

// ----- Sheets -----

/// What `bodies_list` says, as a list.
fn bodies(c: &mut Client) -> Vec<Value> {
    parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().to_vec()
}

#[test]
fn sheets_can_be_made_cut_thickened_and_sewn_through_the_tools() {
    let dir = folder("sheets");
    let mut c = Client::start(&dir);
    // A flat panel, from a region.
    c.ok("feature_add", r#"{"feature":{"type":"sketch","regions":[{"outer":{"type":"rect","x":0,"y":0,"w":40,"h":40}}]}}"#);
    let answer = c.ok("feature_add", r#"{"feature":{"type":"fill","sketch":1}}"#);
    assert!(answer.contains("sheet_note"), "{answer}");
    let list = bodies(&mut c);
    assert_eq!(list.len(), 1);
    assert_eq!(list[0].get("sheet").and_then(Value::as_bool), Some(true));
    assert_eq!(list[0].get("area").and_then(Value::as_f64), Some(1600.0));
    assert_eq!(list[0].get("volume").and_then(Value::as_f64), Some(0.0));
    assert_eq!(list[0].get("boundary_edges").and_then(Value::as_f64), Some(4.0));

    // Its faces and edges are listed like any others, and its boundary says it's one.
    let faces = parsed(&c.ok("faces_list", "{}"));
    assert_eq!(faces.as_array().unwrap().len(), 1);
    assert_eq!(faces.as_array().unwrap()[0].get("name").and_then(Value::as_str), Some("f2:end"));
    let edges = parsed(&c.ok("edges_list", "{}"));
    let edges = edges.as_array().unwrap();
    assert_eq!(edges.len(), 4);
    assert!(edges.iter().all(|e| e.get("boundary").and_then(Value::as_bool) == Some(true)));
    assert!(edges[0].get("select").unwrap().get("near").is_some());

    // Drawn and exported.
    let (content, failed) = c.call("render", r#"{"width":200,"height":150}"#);
    assert!(!failed);
    assert!(content.iter().any(|x| x.get("type").and_then(Value::as_str) == Some("image")));
    let step = c.ok("export", r#"{"format":"step"}"#);
    assert!(step.contains("SHELL_BASED_SURFACE_MODEL") && step.contains("OPEN_SHELL") && !step.contains("MANIFOLD_SOLID_BREP"));
    c.ok("export", r#"{"format":"stl","path":"panel.stl"}"#);

    // Cut with a plane, keeping what's on the side the normal points to.
    let answer = c.ok("feature_add", r#"{"feature":{"type":"trim","point":[10,0,0],"normal":[1,0,0]}}"#);
    assert!(answer.contains("\"id\": 3"), "{answer}");
    assert_eq!(bodies(&mut c)[0].get("area").and_then(Value::as_f64), Some(1200.0));

    // What needs a solid says so.
    let (content, _) = c.call("feature_add", r#"{"feature":{"type":"fillet","edges":"all","radius":1}}"#);
    let said = content.iter().filter_map(|x| x.get("text").and_then(Value::as_str)).collect::<Vec<_>>().join(" ");
    assert!(said.contains("needs a solid"), "{said}");
    c.ok("feature_delete", r#"{"id":4}"#);

    // Into a solid, by name, with no faces picked.
    c.ok("feature_add", r#"{"feature":{"type":"thicken","bodies":["f2"],"faces":[],"thickness":2,"mode":"new_body"}}"#);
    let list = bodies(&mut c);
    assert_eq!(list.len(), 2);
    let solid = list.iter().find(|b| b.get("sheet").is_none()).unwrap();
    assert_eq!(solid.get("volume").and_then(Value::as_f64), Some(2400.0));

    // It all comes back from a file.
    c.ok("document_save", r#"{"path":"sheets.coreshape"}"#);
    c.ok("document_new", "{}");
    c.ok("document_open", r#"{"path":"sheets.coreshape"}"#);
    assert_eq!(bodies(&mut c).len(), 2);
}

#[test]
fn an_open_curve_and_a_sew_through_the_tools() {
    let dir = folder("sheets_open");
    let mut c = Client::start(&dir);
    // Four walls of a box from one open path, with a floor and a lid sewn on.
    c.ok("feature_add", r#"{"feature":{"type":"sketch","regions":[{"outer":{"type":"open_path","start":[0,0],"moves":[{"line_to":[10,0]},{"line_to":[10,10]},{"line_to":[0,10]},{"line_to":[0,0]}]}}]}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"extrude","sketch":1,"start":0,"length":5,"mode":"surface"}}"#);
    let list = bodies(&mut c);
    assert_eq!(list[0].get("sheet").and_then(Value::as_bool), Some(true));
    assert_eq!(list[0].get("area").and_then(Value::as_f64), Some(200.0));
    c.ok("feature_add", r#"{"feature":{"type":"sketch","regions":[{"outer":{"type":"rect","x":0,"y":0,"w":10,"h":10}}]}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"fill","sketch":3}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"sketch","plane":{"origin":[0,0,5]},"regions":[{"outer":{"type":"rect","x":0,"y":0,"w":10,"h":10}}]}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"fill","sketch":5}}"#);
    assert_eq!(bodies(&mut c).len(), 3);
    c.ok("feature_add", r#"{"feature":{"type":"sew"}}"#);
    let list = bodies(&mut c);
    assert_eq!(list.len(), 1);
    assert!(list[0].get("sheet").is_none(), "it closes, so it's a solid: {}", list[0].to_text());
    assert_eq!(list[0].get("volume").and_then(Value::as_f64), Some(500.0));
    // A solid does what a solid does.
    c.ok("feature_add", r#"{"feature":{"type":"fillet","edges":"all","radius":0.5}}"#);

    // Bad requests are refused, saying what's wrong.
    let (content, failed) = c.call("feature_add", r#"{"feature":{"type":"trim","tool":"f2","keep":"middle"}}"#);
    assert!(failed);
    assert!(content[0].get("text").and_then(Value::as_str).unwrap().contains("middle"));
}
