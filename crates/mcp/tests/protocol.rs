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

    // By default a feature acts on the last one made, and it can say otherwise.
    let cut = |x: f64, extra: &str| format!(r#"{{"feature":{{"type":"solid","shape":"cylinder","radius":1,"height":20,"at":{{"origin":[{x},5,-5]}},"mode":"remove"{extra}}}}}"#);
    c.ok("feature_add", &cut(55.0, ""));
    c.ok("feature_add", &cut(5.0, r#","body":"f1""#));
    let bodies = json::parse(&c.ok("bodies_list", "{}")).unwrap();
    let hole = 10.0 * std::f64::consts::PI;
    for b in bodies.as_array().unwrap() {
        assert!((b.get("volume").unwrap().as_f64().unwrap() - (1000.0 - hole)).abs() < 1e-6);
    }
    // A body it can't have leaves nothing behind.
    let before = c.ok("features_list", "{}");
    assert!(c.refused("feature_add", &cut(5.0, r#","body":"f9""#)).contains("f9"));
    assert!(c.refused("feature_add", &cut(5.0, r#","body":"nine""#)).contains("nine"));
    assert_eq!(c.ok("features_list", "{}"), before);
    // Editing keeps the choice unless it says otherwise.
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.5, "")[1..]));
    assert!(c.ok("document_show", "{}").contains(r#""body": "f1""#));
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","body":null"#)[1..]));
    assert!(!c.ok("document_show", "{}").contains(r#""body": "f1""#));
    assert!(c.refused("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","body":"f9""#)[1..])).contains("f9"));
    c.ok("feature_edit", &format!(r#"{{"id":4,{}"#, &cut(5.0, r#","body":"f1""#)[1..]));

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
