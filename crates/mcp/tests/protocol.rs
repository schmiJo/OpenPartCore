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

// ----- Extrude ends, and what an app shows -----

#[test]
fn an_extrude_takes_regions_and_ends_through_the_tools() {
    let dir = folder("extrude_ends");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":100,"dy":100,"dz":2,"at":{"origin":[0,0,30]},"mode":"new_body"}}"#);
    let sketch = r#"{"format":"coreshape-sketch","version":1,"next_entity":3,"next_constraint":1,"entities":[{"id":1,"type":"circle","center":[20,20],"radius":5},{"id":2,"type":"circle","center":[60,20],"radius":3}],"constraints":[]}"#;
    let regions = parsed(&c.ok("sketch_arrangement", &format!(r#"{{"sketch":{sketch}}}"#)));
    let regions = regions.get("regions").unwrap().as_array().unwrap().to_vec();
    assert_eq!(regions.len(), 2);
    let id = |r: &Value| r.get("id").and_then(Value::as_str).unwrap().to_string();
    let area = |r: &Value| r.get("area").and_then(Value::as_f64).unwrap();
    let big = regions.iter().find(|r| area(r) > 50.0).map(id).unwrap();
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"constrained_sketch","plane":{{"origin":[0,0,0]}},"sketch":{sketch}}}}}"#));
    // One region, to the plate above, without a copy of the sketch.
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"extrude","sketch":2,"regions":["{big}"],"end":{{"type":"up_to_face","face":"f1:start"}},"mode":"new_body"}}}}"#));
    let bodies = parsed(&c.ok("bodies_list", "{}"));
    let want = std::f64::consts::PI * 25.0 * 30.0;
    assert!((bodies.as_array().unwrap()[1].get("volume").and_then(Value::as_f64).unwrap() - want).abs() < 1e-5, "{}", bodies.to_text());
    // A region that isn't there comes back as that feature failing, saying what is.
    let added = c.ok("feature_add", r#"{"feature":{"type":"extrude","sketch":2,"regions":["r0000000000ab"],"length":1,"mode":"new_body"}}"#);
    assert!(added.contains("r0000000000ab") && added.contains(&big), "{added}");
    // A bad end is refused outright, as it's not something a rebuild would ever get past.
    assert!(c.refused("feature_add", r#"{"feature":{"type":"extrude","sketch":2,"end":"sideways","mode":"new_body"}}"#).contains("sideways"));
}

#[test]
fn faces_and_edges_say_what_a_tool_needs_to_pick_them() {
    let dir = folder("axis");
    let mut c = Client::start(&dir);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"cylinder","radius":5,"height":20,"at":{"origin":[1,2,3],"z":[0,0,1]},"mode":"new_body"}}"#);
    let faces = parsed(&c.ok("faces_list", "{}"));
    let list = faces.as_array().unwrap();
    let round = list.iter().find(|f| f.get("surface").and_then(Value::as_str) == Some("cylinder")).unwrap();
    let axis = round.get("axis").unwrap_or_else(|| panic!("{}", round.to_text()));
    near(numbers(axis.get("direction").unwrap()), [0.0, 0.0, 1.0]);
    near(numbers(axis.get("point").unwrap()), [1.0, 2.0, 3.0]);
    assert!(list.iter().find(|f| f.get("surface").and_then(Value::as_str) == Some("plane")).unwrap().get("axis").is_none());

    c.ok("document_new", "{}");
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"cone","r0":10,"r1":5,"height":10,"mode":"new_body"}}"#);
    let faces = parsed(&c.ok("faces_list", "{}"));
    let cone = faces.as_array().unwrap().iter().find(|f| f.get("surface").and_then(Value::as_str) == Some("cone")).unwrap().clone();
    let axis = cone.get("axis").unwrap();
    assert!((numbers(axis.get("direction").unwrap())[2].abs() - 1.0).abs() < 1e-9);
    assert!((cone.get("half_angle").and_then(Value::as_f64).unwrap() - 26.565051).abs() < 1e-5, "{}", cone.to_text());

    // A cylinder's rim has two vertices on the same faces, so naming one of them takes a place too, and that place works.
    c.ok("document_new", "{}");
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"cylinder","radius":5,"height":20,"mode":"new_body"}}"#);
    let edges = parsed(&c.ok("edges_list", "{}"));
    let on_rim = edges.as_array().unwrap().iter().find(|e| numbers(e.get("start").unwrap())[2] == 20.0 && numbers(e.get("end").unwrap())[2] == 20.0).unwrap().clone();
    let vertex = on_rim.get("end_vertex").unwrap().clone();
    assert!(vertex.get("near").is_some(), "{}", vertex.to_text());
    let sketch = r#"{"format":"coreshape-sketch","version":1,"next_entity":2,"next_constraint":1,"entities":[{"id":1,"type":"circle","center":[100,0],"radius":5}],"constraints":[]}"#;
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"constrained_sketch","plane":{{"origin":[0,0,0]}},"sketch":{sketch}}}}}"#));
    let answer = c.ok("feature_add", &format!(r#"{{"feature":{{"type":"extrude","sketch":2,"end":{{"type":"up_to_vertex","vertex":{}}},"mode":"new_body"}}}}"#, vertex.to_text()));
    assert!(!answer.contains("failed"), "{answer}");

    // An edge's ends are named so that an extrude can go up to them.
    c.ok("document_new", "{}");
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":20,"dz":30,"mode":"new_body"}}"#);
    let edges = parsed(&c.ok("edges_list", "{}"));
    let edge = edges.as_array().unwrap().iter().find(|e| numbers(e.get("end").unwrap())[2] == 30.0 && numbers(e.get("start").unwrap())[2] == 30.0).unwrap().clone();
    let vertex = edge.get("end_vertex").unwrap().to_text();
    let height = numbers(edge.get("end").unwrap())[2];
    assert_eq!(edge.get("end_vertex").unwrap().get("faces").unwrap().as_array().unwrap().len(), 3);
    let sketch = r#"{"format":"coreshape-sketch","version":1,"next_entity":2,"next_constraint":1,"entities":[{"id":1,"type":"circle","center":[100,0],"radius":5}],"constraints":[]}"#;
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"constrained_sketch","plane":{{"origin":[0,0,0]}},"sketch":{sketch}}}}}"#));
    c.ok("feature_add", &format!(r#"{{"feature":{{"type":"extrude","sketch":2,"end":{{"type":"up_to_vertex","vertex":{vertex}}},"mode":"new_body"}}}}"#));
    let bodies = parsed(&c.ok("bodies_list", "{}"));
    let want = std::f64::consts::PI * 25.0 * height;
    assert!((bodies.as_array().unwrap()[1].get("volume").and_then(Value::as_f64).unwrap() - want).abs() < 1e-5, "{}", bodies.to_text());
}

/// Two free blocks' worth of an assembly: the first fixed, and connectors at each one's corner.
fn two_blocks(c: &mut Client) -> (f64, f64, f64, f64) {
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":10,"dz":10,"mode":"new_body"}}"#);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":10,"dy":10,"dz":10,"at":{"origin":[50,0,0]},"mode":"new_body"}}"#);
    c.ok("tab_new", r#"{"type":"assembly"}"#);
    let a = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f1","fixed":true,"name":"A"}"#)));
    let b = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f2","name":"B"}"#)));
    let frame = r#"{"frame":{"origin":[0,0,0]}}"#;
    let ca = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{frame}}}"#))));
    let frame_b = r#"{"frame":{"origin":[50,0,0]}}"#;
    let cb = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{b},"on":{frame_b}}}"#))));
    (a, b, ca, cb)
}

/// An instance's frame, by name, as origin, z and x.
fn frame_of(state: &Value, name: &str) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let list = state.get("instances").and_then(Value::as_array).unwrap();
    let it = list.iter().find(|i| i.get("name").and_then(Value::as_str) == Some(name)).unwrap_or_else(|| panic!("no {name} in {}", state.to_text()));
    let at = it.get("at").unwrap();
    (numbers(at.get("origin").unwrap()), numbers(at.get("z").unwrap()), numbers(at.get("x").unwrap()))
}

fn freedom_of(state: &Value) -> f64 {
    state.get("freedom").and_then(Value::as_f64).unwrap()
}

fn shown_assembly(c: &mut Client) -> Value {
    let shown = parsed(&c.ok("project_show", "{}"));
    shown.get("tabs").unwrap().as_array().unwrap().iter().find(|t| t.get("assembly").is_some()).unwrap().get("assembly").unwrap().clone()
}

#[test]
fn a_connectors_adjustments_do_what_they_say_through_the_tools() {
    let dir = folder("connector_adjust");
    let mut c = Client::start(&dir);
    let (a, _, ca, _) = two_blocks(&mut c);
    // Moved along its own axes, turned by degrees and by quarter turns, then flipped.
    let made = parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{{"frame":{{"origin":[1,2,3]}}}},"offset":[1,0,0]}}"#)));
    near(numbers(made.get("at").unwrap().get("origin").unwrap()), [2.0, 2.0, 3.0]);
    let made = parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{{"frame":{{"origin":[0,0,0]}}}},"rotation":90}}"#)));
    near(numbers(made.get("at").unwrap().get("x").unwrap()), [0.0, 1.0, 0.0]);
    let made = parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{{"frame":{{"origin":[0,0,0]}}}},"quarter_turns":2}}"#)));
    near(numbers(made.get("at").unwrap().get("x").unwrap()), [-1.0, 0.0, 0.0]);
    let made = parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{{"frame":{{"origin":[0,0,0]}}}},"flip":true}}"#)));
    near(numbers(made.get("at").unwrap().get("z").unwrap()), [0.0, 0.0, -1.0]);
    let spare = id_of(&made);

    // Edited, the saved form has each of them.
    c.ok("connector_edit", &format!(r#"{{"id":{ca},"offset":[4,5,6],"rotation":30,"flip":true,"quarter_turns":1}}"#));
    let shown = shown_assembly(&mut c);
    let list = shown.get("connectors").unwrap().as_array().unwrap();
    let it = list.iter().find(|k| k.get("id").and_then(Value::as_f64) == Some(ca)).unwrap();
    assert_eq!(numbers(it.get("offset").unwrap()), vec![4.0, 5.0, 6.0]);
    assert_eq!(it.get("rotation").and_then(Value::as_f64), Some(30.0));
    assert_eq!(it.get("flip").and_then(Value::as_bool), Some(true));
    assert_eq!(it.get("quarter_turns").and_then(Value::as_f64), Some(1.0));
    // Edited to somewhere else altogether, by a place it can check.
    c.ok("connector_edit", &format!(r#"{{"id":{ca},"on":{{"face":"f1:end"}}}}"#));
    let shown = shown_assembly(&mut c);
    let list = shown.get("connectors").unwrap().as_array().unwrap();
    let it = list.iter().find(|k| k.get("id").and_then(Value::as_f64) == Some(ca)).unwrap();
    assert_eq!(it.get("on").unwrap().get("face").and_then(Value::as_str), Some("f1:end"));
    assert!(c.refused("connector_edit", &format!(r#"{{"id":{ca},"on":{{"face":"f9:end"}}}}"#)).contains("f9:end"));
    assert!(c.refused("connector_edit", r#"{"id":999,"rotation":1}"#).contains("999"));
    // One that nothing uses can go, and then it's gone.
    c.ok("connector_delete", &format!(r#"{{"id":{spare}}}"#));
    assert!(c.refused("connector_delete", &format!(r#"{{"id":{spare}}}"#)).contains("no connector"));
}

#[test]
fn a_mates_adjustments_and_edits_do_what_they_say_through_the_tools() {
    let dir = folder("mate_adjust");
    let mut c = Client::start(&dir);
    let (a, b, ca, cb) = two_blocks(&mut c);
    let made = parsed(&c.ok("mate_add", &format!(r#"{{"type":"fastened","a":{ca},"b":{cb},"name":"Held","offset":[1,2,3]}}"#)));
    let m = id_of(&made);
    assert_eq!(freedom_of(&made), 0.0);
    // Its connector was at x = 50 on the part, so that's the instance's frame less 50.
    near(frame_of(&made, "B").0, [-49.0, 2.0, 3.0]);
    let list = made.get("mates").unwrap().as_array().unwrap();
    assert_eq!(list[0].get("name").and_then(Value::as_str), Some("Held"));

    let edit = |c: &mut Client, fields: &str| parsed(&c.ok("mate_edit", &format!(r#"{{"id":{m},{fields}}}"#)));
    let state = edit(&mut c, r#""offset":[0,0,0]"#);
    near(frame_of(&state, "B").0, [-50.0, 0.0, 0.0]);
    let state = edit(&mut c, r#""rotation":90"#);
    near(frame_of(&state, "B").2, [0.0, 1.0, 0.0]);
    // A quarter turn the other way, on the second end, undoes that.
    let state = edit(&mut c, r#""quarter_turns":1"#);
    near(frame_of(&state, "B").2, [1.0, 0.0, 0.0]);
    let state = edit(&mut c, r#""rotation":0,"quarter_turns":0,"flip":true"#);
    near(frame_of(&state, "B").1, [0.0, 0.0, -1.0]);
    let state = edit(&mut c, r#""flip":false,"name":"Renamed""#);
    let list = state.get("mates").unwrap().as_array().unwrap();
    assert_eq!(list[0].get("name").and_then(Value::as_str), Some("Renamed"));

    // Another kind of mate, and limits on it.
    let state = edit(&mut c, r#""type":"revolute","limits":[{"dof":"angle","min":-10,"max":20}]"#);
    assert_eq!(freedom_of(&state), 1.0);
    let state = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{m},"dof":"angle","value":90}}"#)));
    assert!((state.get("reached").and_then(Value::as_f64).unwrap() - 20.0).abs() < 0.01, "{}", state.to_text());
    assert!(c.refused("mate_edit", &format!(r#"{{"id":{m},"type":"welded"}}"#)).contains("welded"));

    // Suppressed, it says nothing about where the part goes, and the instance is as free as any.
    let state = edit(&mut c, r#""suppressed":true"#);
    assert_eq!(freedom_of(&state), 6.0);
    assert_eq!(state.get("mates").unwrap().as_array().unwrap()[0].get("suppressed").and_then(Value::as_bool), Some(true));
    let state = edit(&mut c, r#""suppressed":false"#);
    assert_eq!(freedom_of(&state), 1.0);

    // Deleted, it takes a connector made only for it, and leaves those made on their own.
    let inline = parsed(&c.ok("mate_add", &format!(r#"{{"type":"planar","a":{{"instance":{b},"on":{{"face":"f2:start"}}}},"b":{{"instance":{a},"on":{{"face":"f1:end"}}}},"flip":true}}"#)));
    let second = id_of(&inline);
    assert_eq!(shown_assembly(&mut c).get("connectors").unwrap().as_array().unwrap().len(), 4);
    let state = parsed(&c.ok("mate_delete", &format!(r#"{{"id":{second}}}"#)));
    assert_eq!(state.get("mates").unwrap().as_array().unwrap().len(), 1);
    assert_eq!(shown_assembly(&mut c).get("connectors").unwrap().as_array().unwrap().len(), 2);
    assert!(c.refused("mate_delete", &format!(r#"{{"id":{second}}}"#)).contains("no mate"));
}

#[test]
fn instances_can_be_edited_and_deleted_through_the_tools() {
    let dir = folder("instance_edit");
    let mut c = Client::start(&dir);
    let (a, b, ca, cb) = two_blocks(&mut c);
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{b},"name":"Other","at":{{"origin":[7,8,9]}}}}"#)));
    near(frame_of(&state, "Other").0, [7.0, 8.0, 9.0]);
    // Fixed, it doesn't go anywhere when a mate pulls on it, and that's what fails.
    c.ok("instance_edit", &format!(r#"{{"id":{b},"fixed":true}}"#));
    let state = parsed(&c.ok("mate_add", &format!(r#"{{"type":"fastened","a":{cb},"b":{ca}}}"#)));
    near(frame_of(&state, "Other").0, [7.0, 8.0, 9.0]);
    assert!(state.get("mates").unwrap().as_array().unwrap()[0].get("failing").is_some(), "{}", state.to_text());
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{b},"fixed":false}}"#)));
    assert!(state.get("mates").unwrap().as_array().unwrap()[0].get("failing").is_none());
    // Suppressed, it isn't there, and its mate says so.
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{b},"suppressed":true}}"#)));
    assert_eq!(state.get("instances").unwrap().as_array().unwrap().len(), 1);
    assert!(state.get("mates").unwrap().as_array().unwrap()[0].get("failing").is_some());
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{b},"suppressed":false}}"#)));
    assert_eq!(state.get("instances").unwrap().as_array().unwrap().len(), 2);
    // A name for one from a part's maker, numbered among its copies.
    let state = parsed(&c.ok("instance_add", r#"{"part":"f2"}"#));
    let names: Vec<String> = state.get("instances").unwrap().as_array().unwrap().iter().map(|i| i.get("name").and_then(Value::as_str).unwrap().to_string()).collect();
    assert!(names.iter().any(|n| n.ends_with("<2>")), "{names:?}");
    // Deleted, its mates go too.
    let state = parsed(&c.ok("instance_delete", &format!(r#"{{"id":{b}}}"#)));
    assert!(state.get("mates").unwrap().as_array().unwrap().is_empty());
    assert!(c.refused("instance_delete", &format!(r#"{{"id":{b}}}"#)).contains("no instance"));
    assert!(c.refused("instance_edit", &format!(r#"{{"id":{b},"fixed":true}}"#)).contains("no instance"));
    let _ = a;
}

#[test]
fn groups_and_relations_go_through_the_tools() {
    let dir = folder("groups");
    let mut c = Client::start(&dir);
    let (a, b, ca, cb) = two_blocks(&mut c);
    let third = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f2","name":"C","at":{"origin":[0,50,0]}}"#)));
    // Grouped, two free parts count as one, in six ways rather than twelve.
    let state = parsed(&c.ok("assembly_solve", "{}"));
    assert_eq!(freedom_of(&state), 12.0);
    let group = parsed(&c.ok("group_add", &format!(r#"{{"instances":[{b},[{third}]]}}"#)));
    assert_eq!(freedom_of(&group), 6.0);
    let state = parsed(&c.ok("group_delete", &format!(r#"{{"id":{}}}"#, id_of(&group))));
    assert_eq!(freedom_of(&state), 12.0);
    assert!(c.refused("group_delete", &format!(r#"{{"id":{}}}"#, id_of(&group))).contains("no group"));

    // Two turning parts, geared. Adding it moves nothing, and from then on they go by the ratio.
    let ground = format!(r#"{{"frame":{{"origin":[100,0,0]}}}}"#);
    let second = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{ground}}}"#))));
    let cc = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{third},"on":{{"frame":{{"origin":[0,0,0]}}}}}}"#))));
    let m1 = id_of(&parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{cb},"b":{ca}}}"#))));
    let m2 = id_of(&parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{cc},"b":{second}}}"#))));
    c.ok("mate_set_value", &format!(r#"{{"id":{m1},"dof":"angle","value":10}}"#));
    let geared = parsed(&c.ok("relation_add", &format!(r#"{{"type":"gear","ratio":3,"a":{m1},"b":{m2}}}"#)));
    assert!((mate_value(&geared, m1, "angle") - 10.0).abs() < 1e-5, "{}", geared.to_text());
    assert!(mate_value(&geared, m2, "angle").abs() < 1e-5, "{}", geared.to_text());
    assert_eq!(freedom_of(&geared), 1.0);
    let state = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{m1},"dof":"angle","value":20}}"#)));
    assert!((mate_value(&state, m2, "angle") - 30.0).abs() < 1e-4, "{}", state.to_text());
    // Taken off, they're free of each other.
    let state = parsed(&c.ok("relation_delete", &format!(r#"{{"id":{}}}"#, id_of(&geared))));
    assert_eq!(freedom_of(&state), 2.0);
    assert!(c.refused("relation_delete", &format!(r#"{{"id":{}}}"#, id_of(&geared))).contains("no relation"));
    // The other way round.
    let backward = parsed(&c.ok("relation_add", &format!(r#"{{"type":"gear","ratio":3,"a":{m1},"b":{m2},"reverse":true}}"#)));
    let was = mate_value(&backward, m2, "angle");
    let state = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{m1},"dof":"angle","value":30}}"#)));
    assert!((mate_value(&state, m2, "angle") - (was - 30.0)).abs() < 1e-4, "{}", state.to_text());
    // A screw is on one mate, named once.
    c.ok("mate_delete", &format!(r#"{{"id":{m2}}}"#));
    c.ok("mate_edit", &format!(r#"{{"id":{m1},"type":"cylindrical"}}"#));
    let screw = parsed(&c.ok("relation_add", &format!(r#"{{"type":"screw","lead":2,"a":{m1}}}"#)));
    let start = mate_value(&screw, m1, "z");
    let state = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{m1},"dof":"angle","value":390}}"#)));
    let turned = mate_value(&state, m1, "angle") - mate_value(&screw, m1, "angle");
    assert!((mate_value(&state, m1, "z") - start - turned / 360.0 * 2.0).abs() < 1e-4, "{}", state.to_text());
}

#[test]
fn tabs_can_be_renamed_deleted_and_moved_between_through_the_tools() {
    let dir = folder("tabs");
    let mut c = Client::start(&dir);
    let shown = parsed(&c.ok("tab_rename", r#"{"tab":1,"name":"Parts"}"#));
    assert_eq!(shown.as_array().unwrap()[0].get("name").and_then(Value::as_str), Some("Parts"));
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":1,"dy":1,"dz":1,"mode":"new_body"}}"#);
    // Deleting the Part Studio being worked on puts it on another, whose features are its own.
    c.ok("tab_new", r#"{"type":"part_studio","name":"Second"}"#);
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":2,"dy":2,"dz":2,"mode":"new_body"},"name":"Mine"}"#);
    let shown = parsed(&c.ok("tab_delete", r#"{"tab":2}"#));
    let list = shown.as_array().unwrap();
    assert_eq!(list.len(), 1);
    assert_eq!(list[0].get("selected").and_then(Value::as_bool), Some(true));
    let bodies = parsed(&c.ok("bodies_list", "{}"));
    assert_eq!(bodies.as_array().unwrap()[0].get("volume").and_then(Value::as_f64), Some(1.0));
    // Deleting the selected assembly goes back to the Part Studio.
    c.ok("tab_new", r#"{"type":"assembly"}"#);
    let shown = parsed(&c.ok("tab_delete", r#"{"tab":3}"#));
    assert_eq!(shown.as_array().unwrap()[0].get("selected").and_then(Value::as_bool), Some(true));
    assert!(c.refused("assembly_solve", "{}").contains("no assembly is selected"));
    assert!(c.refused("tab_new", r#"{"type":"drawing"}"#).contains("drawing"));
    assert!(c.refused("tab_rename", r#"{"tab":9,"name":"x"}"#).contains("9"));
    assert!(c.refused("tab_delete", r#"{"tab":9}"#).contains("9"));
    // The tools take an assembly by its tab, whichever is selected.
    c.ok("tab_new", r#"{"type":"assembly","name":"Named"}"#);
    c.ok("tab_select", r#"{"tab":1}"#);
    assert_eq!(parsed(&c.ok("assembly_solve", r#"{"tab":4}"#)).get("tab").and_then(Value::as_f64), Some(4.0));
    assert_eq!(parsed(&c.ok("bom", r#"{"tab":4}"#)).as_array().unwrap().len(), 0);
    assert!(parsed(&c.ok("interference_check", r#"{"tab":4}"#)).get("interferences").unwrap().as_array().unwrap().is_empty());
    assert!(c.refused("assembly_solve", r#"{"tab":1}"#).contains("Part Studio"));
    let shown = parsed(&c.ok("tab_list", "{}"));
    let names: Vec<&str> = shown.as_array().unwrap().iter().map(|t| t.get("name").and_then(Value::as_str).unwrap()).collect();
    assert_eq!(names, vec!["Parts", "Named"]);
}

#[test]
fn a_whole_pose_an_edge_and_a_view_of_one_part_go_through_the_tools() {
    let dir = folder("assembly_more");
    let mut c = Client::start(&dir);
    let (_, b, _, _) = two_blocks(&mut c);
    // A whole new pose.
    let state = parsed(&c.ok("assembly_drag", &format!(r#"{{"instance":{b},"pose":{{"origin":[1,2,3],"z":[1,0,0],"x":[0,1,0]}}}}"#)));
    let (origin, z, x) = frame_of(&state, "B");
    near(origin, [1.0, 2.0, 3.0]);
    near(z, [1.0, 0.0, 0.0]);
    near(x, [0.0, 1.0, 0.0]);
    // With no point given, it's the part's own origin that goes there.
    let state = parsed(&c.ok("assembly_drag", &format!(r#"{{"instance":{b},"to":[4,5,6]}}"#)));
    near(frame_of(&state, "B").0, [4.0, 5.0, 6.0]);
    assert!(c.refused("assembly_drag", &format!(r#"{{"instance":{b},"to":[1,2]}}"#)).contains("three numbers"));
    assert!(c.refused("assembly_drag", &format!(r#"{{"instance":{b},"to":[1,2,3],"pose":{{}}}}"#)).contains("either"));

    // The places on an edge, in where the assembly has the part.
    let edges = parsed(&c.ok("edges_list", r#"{"body":"f2"}"#));
    assert!(!edges.as_array().unwrap().is_empty());
    let faces = parsed(&c.ok("faces_list", r#"{"body":"f2"}"#));
    let find = |wanted: &[f64; 3]| {
        let list = faces.as_array().unwrap();
        let hit = list.iter().find(|f| numbers(f.get("normal").unwrap()).iter().zip(wanted).all(|(a, b)| (a - b).abs() < 1e-9)).expect("a face that way");
        hit.get("key").and_then(Value::as_str).unwrap().to_string()
    };
    let (top, side) = (find(&[0.0, 0.0, 1.0]), find(&[1.0, 0.0, 0.0]));
    let places = parsed(&c.ok("connector_candidates", &format!(r#"{{"instance":{b},"edge":["{top}","{side}"]}}"#)));
    let list = places.as_array().unwrap();
    assert_eq!(list.len(), 3, "{}", places.to_text());
    assert_eq!(list[0].get("label").and_then(Value::as_str), Some("middle"));
    // Its middle: at (60, 5, 10) on the part, whose x, y and z axes now point along Y, Z and X, from (4, 5, 6).
    near(numbers(list[0].get("at").unwrap().get("origin").unwrap()), [4.0 + 10.0, 5.0 + 60.0, 6.0 + 5.0]);
    let on = list[0].get("on").unwrap().to_text();
    c.ok("connector_add", &format!(r#"{{"instance":{b},"on":{on}}}"#));
    assert!(c.refused("connector_candidates", &format!(r#"{{"instance":{b},"edge":["{top}","{top}x"]}}"#)).contains("x"));

    // An STL of it all, and one part on its own, by name, though an assembly's selected.
    c.ok("export", r#"{"format":"stl","path":"all.stl"}"#);
    c.ok("export", r#"{"format":"stl","path":"one.stl","body":"f2"}"#);
    let (all, one) = (std::fs::metadata(dir.join("all.stl")).unwrap().len(), std::fs::metadata(dir.join("one.stl")).unwrap().len());
    assert_eq!(one, 84 + 12 * 50);
    assert_eq!(all, 84 + 24 * 50);
    // A part's STEP on its own is the plain kind, and the whole is an assembly.
    assert!(!c.ok("export", r#"{"format":"step","body":"f2"}"#).contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE"));
    assert!(c.ok("export", r#"{"format":"step"}"#).contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE"));
    // With nothing in it, it says so.
    c.ok("tab_new", r#"{"type":"assembly"}"#);
    assert!(c.refused("render", "{}").contains("no parts in this assembly"));
    assert!(c.refused("export", r#"{"format":"stl","path":"none.stl"}"#).contains("no parts in this assembly"));
}

#[test]
fn a_session_gives_an_app_the_parts_and_where_they_go() {
    let dir = folder("assembly_session");
    let mut s = coreshape_mcp::Session::new(dir.clone());
    let call = |s: &mut coreshape_mcp::Session, name: &str, args: &str| {
        let answer = s.call_safely(name, &json::parse(args).unwrap());
        assert_eq!(answer.get("isError").and_then(Value::as_bool), Some(false), "{name}: {}", answer.to_text());
    };
    assert!(s.solved().unwrap_err().contains("no assembly is selected"));
    call(&mut s, "feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":2,"dy":3,"dz":4,"mode":"new_body"}}"#);
    let studio = s.active_tab();
    call(&mut s, "tab_new", r#"{"type":"assembly"}"#);
    assert_ne!(s.active_tab(), studio);
    call(&mut s, "instance_add", r#"{"part":"f1","at":{"origin":[10,0,0]}}"#);
    let solved = s.solved().unwrap();
    assert_eq!(solved.placed.len(), 1);
    assert_eq!(solved.placed[0].at.origin.x, 10.0);
    assert_eq!(solved.placed[0].body.faces.len(), 6);
    // Everything's there to be got at, and the Part Studio being worked on is in its place among the tabs for that.
    let bodies = s.with_project(|p| p.studio_mut(studio).unwrap().rebuild(None).bodies.len());
    assert_eq!(bodies, 1);
    assert_eq!(s.document().features().len(), 1);
    // A panic inside doesn't lose it.
    let lost = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| s.with_project(|_| panic!("on purpose"))));
    assert!(lost.is_err());
    assert_eq!(s.document().features().len(), 1);
    assert_eq!(s.with_project(|p| p.studio_mut(studio).unwrap().features().len()), 1);
}

#[test]
fn a_file_of_nothing_but_assemblies_still_has_a_part_studio_to_work_on() {
    let dir = folder("assembly_only");
    let mut c = Client::start(&dir);
    let text = r#"{"format":"coreshape-project","version":1,"next_id":5,"tabs":[{"id":4,"name":"Alone","type":"assembly","assembly":{"instances":[],"connectors":[],"mates":[]}}]}"#;
    c.ok("document_open", &format!(r#"{{"text":{}}}"#, Value::str(text).to_text()));
    let tabs = parsed(&c.ok("tab_list", "{}"));
    let list = tabs.as_array().unwrap();
    assert_eq!(list.len(), 2);
    assert!(list.iter().any(|t| t.get("type").and_then(Value::as_str) == Some("part_studio")));
    c.ok("feature_add", r#"{"feature":{"type":"solid","shape":"box","dx":1,"dy":1,"dz":1,"mode":"new_body"}}"#);
    assert_eq!(parsed(&c.ok("bodies_list", "{}")).as_array().unwrap().len(), 1);
}

#[test]
fn what_fails_in_a_sub_assembly_shows_above_it() {
    let dir = folder("assembly_sub_failing");
    let mut c = Client::start(&dir);
    let (_, b, ca, cb) = two_blocks(&mut c);
    // Two fixed parts in a tab, with a mate that can't be met there.
    c.ok("instance_edit", &format!(r#"{{"id":{b},"fixed":true,"at":{{"origin":[300,0,0]}}}}"#));
    c.ok("mate_add", &format!(r#"{{"type":"fastened","a":{ca},"b":{cb}}}"#));
    c.ok("tab_new", r#"{"type":"assembly","name":"Top"}"#);
    let sub = id_of(&parsed(&c.ok("instance_add", r#"{"assembly":2,"fixed":true}"#)));
    let state = parsed(&c.ok("assembly_solve", "{}"));
    let failing = state.get("failing_in_sub_assemblies").unwrap_or_else(|| panic!("{}", state.to_text())).as_array().unwrap();
    assert_eq!(failing.len(), 1, "{}", state.to_text());
    assert!(failing[0].get("failing").and_then(Value::as_str).unwrap().contains("sub-assembly"));
    // Locked, it's that tab's own business, and nothing's wrong up here.
    let state = parsed(&c.ok("instance_edit", &format!(r#"{{"id":{sub},"locked":true}}"#)));
    assert!(state.get("failing_in_sub_assemblies").is_none(), "{}", state.to_text());
}

#[test]
fn a_relation_that_cant_be_met_is_named() {
    let dir = folder("assembly_relation_failing");
    let mut c = Client::start(&dir);
    let (a, b, ca, cb) = two_blocks(&mut c);
    let cc = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{a},"on":{{"frame":{{"origin":[0,0,0]}}}}}}"#))));
    let m1 = id_of(&parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{cb},"b":{ca},"limits":[{{"dof":"angle","min":-5,"max":5}}]}}"#))));
    let third = id_of(&parsed(&c.ok("instance_add", r#"{"part":"f2","name":"C"}"#)));
    let cd = id_of(&parsed(&c.ok("connector_add", &format!(r#"{{"instance":{third},"on":{{"frame":{{"origin":[50,0,0]}}}}}}"#))));
    let m2 = id_of(&parsed(&c.ok("mate_add", &format!(r#"{{"type":"revolute","a":{cd},"b":{cc}}}"#))));
    let tied = parsed(&c.ok("relation_add", &format!(r#"{{"type":"gear","ratio":1,"a":{m1},"b":{m2}}}"#)));
    let r = id_of(&tied);
    // Its limit stops the first, so through the gear, the second can't get where it's told.
    let state = parsed(&c.ok("mate_set_value", &format!(r#"{{"id":{m2},"dof":"angle","value":90}}"#)));
    assert!(state.get("reached").and_then(Value::as_f64).unwrap() <= 5.0 + 1e-3, "{}", state.to_text());
    // Fixing the second where it isn't what the gear says makes it impossible, and that's what's named.
    c.ok("instance_edit", &format!(r#"{{"id":{third},"at":{{"origin":[0,-50,0],"z":[0,0,1],"x":[0,1,0]}},"fixed":true}}"#));
    c.ok("instance_edit", &format!(r#"{{"id":{b},"fixed":true}}"#));
    let state = parsed(&c.ok("assembly_solve", "{}"));
    let named = state.get("failing_relations").unwrap_or_else(|| panic!("{}", state.to_text())).as_array().unwrap();
    assert_eq!(named[0].as_f64(), Some(r));
}

#[test]
fn the_first_end_of_a_new_mate_is_what_moves_through_the_tools() {
    let dir = folder("assembly_first_end");
    let mut c = Client::start(&dir);
    let (a, b, ca, cb) = two_blocks(&mut c);
    // Neither is held, so it could be either that moves.
    c.ok("instance_edit", &format!(r#"{{"id":{a},"fixed":false,"at":{{"origin":[7,0,0]}}}}"#));
    c.ok("instance_edit", &format!(r#"{{"id":{b},"at":{{"origin":[0,30,0]}}}}"#));
    let state = parsed(&c.ok("mate_add", &format!(r#"{{"type":"fastened","a":{cb},"b":{ca}}}"#)));
    let (fa, fb) = (frame_of(&state, "A").0, frame_of(&state, "B").0);
    // The second stays, to within what a preference leaves, and the first has gone to it.
    assert!((fa[0] - 7.0).abs() < 1e-2 && fa[1].abs() < 1e-2, "{fa:?}");
    assert!((fb[0] + 43.0).abs() < 1e-2 && fb[1].abs() < 1e-2, "{fb:?}");
}
