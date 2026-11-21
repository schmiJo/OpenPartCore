//! Bringing in a STEP file through the tools as an app does: what the feature left out and why, each body's name, a large
//! file's text kept out of the document shown and given back to open it, and progress while it works.

use coreshape_document::json::{self, Value};
use coreshape_mcp::Server;
use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

/// Three tetrahedra, each a faceted solid of its own with its name, the second with a face missing.
fn file() -> String {
    let mut lines = vec!["#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));".to_string()];
    let add = |lines: &mut Vec<String>, body: String| {
        let id = lines.len() + 1;
        lines.push(format!("#{id}={body};"));
        id
    };
    let solid = |lines: &mut Vec<String>, name: &str, x: f64, size: f64, faces: usize| {
        let at = [[x, 0.0, 0.0], [x + size, 0.0, 0.0], [x, size, 0.0], [x, 0.0, size]];
        let p: Vec<usize> = at.iter().map(|q| add(lines, format!("CARTESIAN_POINT('',({:.6},{:.6},{:.6}))", q[0], q[1], q[2]))).collect();
        let mut list = Vec::new();
        for (tri, n) in [([0, 2, 1], [0.0, 0.0, -1.0]), ([0, 1, 3], [0.0, -1.0, 0.0]), ([0, 3, 2], [-1.0, 0.0, 0.0]), ([1, 2, 3], [1.0f64, 1.0, 1.0])].iter().take(faces) {
            let lp = add(lines, format!("POLY_LOOP('',(#{},#{},#{}))", p[tri[0]], p[tri[1]], p[tri[2]]));
            let bound = add(lines, format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
            let o = add(lines, format!("CARTESIAN_POINT('',({:.6},{:.6},{:.6}))", at[tri[0]][0], at[tri[0]][1], at[tri[0]][2]));
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            let z = add(lines, format!("DIRECTION('',({:.9},{:.9},{:.9}))", n[0] / len, n[1] / len, n[2] / len));
            let a = add(lines, format!("AXIS2_PLACEMENT_3D('',#{o},#{z},$)"));
            let plane = add(lines, format!("PLANE('',#{a})"));
            list.push(format!("#{}", add(lines, format!("FACE_SURFACE('',(#{bound}),#{plane},.T.)"))));
        }
        let shell = add(lines, format!("CLOSED_SHELL('',({}))", list.join(",")));
        add(lines, format!("FACETED_BREP('{name}',#{shell})"))
    };
    solid(&mut lines, "wheel hub", 0.0, 10.0, 4);
    solid(&mut lines, "bent bracket", 50.0, 10.0, 3);
    solid(&mut lines, "Zahnr\\X\\E4der", 100.0, 20.0, 4);
    format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n", lines.join("\n"))
}

/// Sends one request and gives its answer, and any notifications sent before it.
fn ask(server: &mut Server, heard: &Rc<RefCell<Vec<String>>>, id: u32, method: &str, params: Value) -> (Value, Vec<Value>) {
    heard.borrow_mut().clear();
    let line = Value::obj(vec![("jsonrpc", Value::str("2.0")), ("id", Value::Num(id as f64)), ("method", Value::str(method)), ("params", params)]).to_text();
    let answer = json::parse(&server.handle_line(&line).unwrap()).unwrap();
    let notes = heard.borrow().iter().map(|l| json::parse(l).unwrap()).collect();
    (answer, notes)
}

fn call(name: &str, arguments: Value) -> Value {
    Value::obj(vec![("name", Value::str(name)), ("arguments", arguments)])
}

/// The last text a tool answered with, which is the rebuild's summary when it gives one, and whether it was refused.
fn text_of(answer: &Value) -> (String, bool) {
    let result = answer.get("result").unwrap();
    let t = result.get("content").and_then(Value::as_array).and_then(|c| c.last()).and_then(|c| c.get("text")).and_then(Value::as_str).unwrap().to_string();
    (t, result.get("isError") == Some(&Value::Bool(true)))
}

fn server() -> (Server, Rc<RefCell<Vec<String>>>) {
    let mut server = Server::new(PathBuf::from("/"));
    let heard = Rc::new(RefCell::new(Vec::new()));
    let h = heard.clone();
    server.set_notifier(move |line| h.borrow_mut().push(line));
    (server, heard)
}

#[test]
fn an_import_says_what_it_left_out_and_names_its_parts() {
    let (mut server, heard) = server();
    let feature = Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(file())), ("mode", Value::str("new_body"))]);
    let (answer, progress) = ask(&mut server, &heard, 1, "tools/call", call("feature_add", Value::obj(vec![("feature", feature)])));
    let (summary, refused) = text_of(&answer);
    assert!(!refused, "{summary}");
    // Nothing asked how it was going, so nothing said.
    assert!(progress.is_empty());
    let summary = json::parse(&summary).unwrap();
    let step = &summary.get("features").and_then(Value::as_array).unwrap()[0];
    assert_eq!(step.get("state").and_then(Value::as_str), Some("rebuilt"));
    let said = step.get("notes").and_then(Value::as_array).unwrap();
    assert_eq!(said.len(), 1);
    assert!(said[0].as_str().unwrap().contains("left out solid 2 of 3, \"bent bracket\""), "{:?}", said[0]);
    let notes = summary.get("notes").and_then(Value::as_array).unwrap();
    assert!(notes.iter().any(|n| n.as_str().unwrap().starts_with("f1 (") && n.as_str().unwrap().contains("bent bracket")), "{notes:?}");
    let solid = summary.get("solid").unwrap();
    assert_eq!(solid.get("valid"), Some(&Value::Bool(true)));
    let volume = solid.get("volume").and_then(Value::as_f64).unwrap();
    assert!((volume - 9000.0 / 6.0).abs() < 1e-6, "{volume}");
    let max = solid.get("bounds_max").and_then(Value::as_array).unwrap();
    assert!((max[0].as_f64().unwrap() - 120.0).abs() < 1e-6);
    // The bodies go by their parts' names, as the file writes them.
    let (listed, _) = text_of(&ask(&mut server, &heard, 2, "tools/call", call("bodies_list", Value::obj(vec![]))).0);
    let listed = json::parse(&listed).unwrap();
    let names: Vec<&str> = listed.as_array().unwrap().iter().map(|b| b.get("name").and_then(Value::as_str).unwrap()).collect();
    assert_eq!(names, ["wheel hub", "Zahnr\u{e4}der"]);
}

#[test]
fn a_large_text_can_be_kept_out_of_the_document_and_given_back() {
    let (mut server, heard) = server();
    let text = file();
    let feature = Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(&text)), ("mode", Value::str("new_body"))]);
    ask(&mut server, &heard, 1, "tools/call", call("feature_add", Value::obj(vec![("feature", feature)])));
    let (whole, _) = text_of(&ask(&mut server, &heard, 2, "tools/call", call("document_show", Value::obj(vec![]))).0);
    assert!(whole.len() > text.len());
    let (small, refused) = text_of(&ask(&mut server, &heard, 3, "tools/call", call("document_show", Value::obj(vec![("omit_texts_over", Value::Num(1000.0))]))).0);
    assert!(!refused, "{small}");
    assert!(small.len() < 2000, "{small}");
    let doc = json::parse(&small).unwrap();
    let left = doc.get("features").and_then(Value::as_array).unwrap()[0].get("text").unwrap().clone();
    assert_eq!(left.get("bytes").and_then(Value::as_f64), Some(text.len() as f64));
    let hash = left.get("sha256").and_then(Value::as_str).unwrap().to_string();
    assert_eq!(hash.len(), 64);
    // Opened again with the text given back, it's the document it was.
    let blobs = Value::Obj(vec![(hash.clone(), Value::str(&text))]);
    let (opened, refused) = text_of(&ask(&mut server, &heard, 4, "tools/call", call("document_open", Value::obj(vec![("text", Value::str(&small)), ("blobs", blobs)]))).0);
    assert!(!refused, "{opened}");
    let (again, _) = text_of(&ask(&mut server, &heard, 5, "tools/call", call("document_show", Value::obj(vec![]))).0);
    assert_eq!(again, whole);
    // Without it, or with some other text under its name, it's refused, naming it.
    let (why, refused) = text_of(&ask(&mut server, &heard, 6, "tools/call", call("document_open", Value::obj(vec![("text", Value::str(&small))]))).0);
    assert!(refused, "{why}");
    let wrong = Value::Obj(vec![(hash.clone(), Value::str(text.replace("wheel hub", "wheel nut")))]);
    let (why, refused) = text_of(&ask(&mut server, &heard, 7, "tools/call", call("document_open", Value::obj(vec![("text", Value::str(&small)), ("blobs", wrong)]))).0);
    assert!(refused && why.contains(&hash), "{why}");
    let (why, refused) = text_of(&ask(&mut server, &heard, 8, "tools/call", call("document_open", Value::obj(vec![("text", Value::str(&small)), ("blobs", Value::obj(vec![]))]))).0);
    assert!(refused && why.contains(&hash), "{why}");
}

#[test]
fn a_call_that_asks_hears_how_it_is_going() {
    let (mut server, heard) = server();
    let feature = Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(file())), ("mode", Value::str("new_body"))]);
    let params = Value::obj(vec![
        ("name", Value::str("feature_add")),
        ("arguments", Value::obj(vec![("feature", feature)])),
        ("_meta", Value::obj(vec![("progressToken", Value::str("import-1"))])),
    ]);
    let (answer, progress) = ask(&mut server, &heard, 1, "tools/call", params);
    assert!(!text_of(&answer).1);
    assert!(!progress.is_empty());
    let mut last = 0.0;
    let mut stages = Vec::new();
    for p in &progress {
        assert_eq!(p.get("method").and_then(Value::as_str), Some("notifications/progress"));
        assert!(p.get("id").is_none());
        let params = p.get("params").unwrap();
        assert_eq!(params.get("progressToken").and_then(Value::as_str), Some("import-1"));
        let n = params.get("progress").and_then(Value::as_f64).unwrap();
        assert!(n > last, "progress has to go up");
        last = n;
        let message = params.get("message").and_then(Value::as_str).unwrap();
        stages.push(message.split(' ').take(2).collect::<Vec<_>>().join(" "));
    }
    // Two of the three parts came in, so there are two to measure.
    for stage in ["f1: parsing", "f1: solids", "rebuild 1", "measuring 2"] {
        assert!(stages.iter().any(|s| s.starts_with(stage)), "{stage} in {stages:?}");
    }
    // After the call, nothing's listening.
    let (_, after) = ask(&mut server, &heard, 2, "tools/call", call("parameter_set", Value::obj(vec![("name", Value::str("x")), ("value", Value::Num(2.0))])));
    assert!(after.is_empty());
}

#[test]
fn copies_are_measured_once_and_bounded_where_each_one_is() {
    use coreshape::{box_solid, v3, Frame, Vec3};
    let plate = box_solid(&Frame::WORLD, 10.0, 20.0, 5.0, 1).unwrap();
    let moved = Frame::translation(v3(100.0, 0.0, 0.0));
    let turned = Frame::rotation(Vec3::ZERO, Vec3::Z, std::f64::consts::FRAC_PI_2);
    let text = coreshape::export::step::write_step_assembly(&[("plate", &plate)], &[(0, "first", moved), (0, "second", turned)], "kit");
    let (mut server, heard) = server();
    let feature = Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(text)), ("mode", Value::str("new_body"))]);
    let (summary, refused) = text_of(&ask(&mut server, &heard, 1, "tools/call", call("feature_add", Value::obj(vec![("feature", feature)]))).0);
    assert!(!refused, "{summary}");
    let solid = json::parse(&summary).unwrap().get("solid").unwrap().clone();
    let v = |k: &str| solid.get(k).and_then(Value::as_array).unwrap().iter().map(|x| x.as_f64().unwrap()).collect::<Vec<_>>();
    // The first is at x 100 to 110, and the second, turned, at x -20 to 0: together they span both.
    for (got, want) in v("bounds_min").iter().zip([-20.0, 0.0, 0.0]).chain(v("bounds_max").iter().zip([110.0, 20.0, 5.0])) {
        assert!((got - want).abs() < 1e-6, "{got} for {want}");
    }
    assert!((solid.get("volume").and_then(Value::as_f64).unwrap() - 2000.0).abs() < 1e-6);
    let (listed, _) = text_of(&ask(&mut server, &heard, 2, "tools/call", call("bodies_list", Value::obj(vec![]))).0);
    let listed = json::parse(&listed).unwrap();
    let second = &listed.as_array().unwrap()[1];
    assert_eq!(second.get("name").and_then(Value::as_str), Some("plate"));
    let max: Vec<f64> = second.get("box_max").and_then(Value::as_array).unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
    assert!((max[0] - 0.0).abs() < 1e-6 && (max[1] - 10.0).abs() < 1e-6, "{max:?}");
}
