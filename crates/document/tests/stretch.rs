//! Stretching parts along one direction: by numbers or along a line it follows, as a copy or in place, with every face
//! keeping its name, each volume held to the factor times what it was.

use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    for b in &o.bodies {
        validate(&b.body).unwrap_or_else(|e| panic!("{e:?}"));
    }
    o
}

fn failure(doc: &mut Document) -> String {
    let o = doc.rebuild(None);
    let i = o.failed.unwrap_or_else(|| panic!("it should have failed: {:?}", o.steps));
    match &o.steps[i].state {
        StepState::Failed(why) => why.clone(),
        other => panic!("{other:?}"),
    }
}

fn op(text: &str) -> Op {
    read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))
}

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(op(text)).unwrap()
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-8 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

/// A plate 20 by 12 by 4 with a hole of radius 3 at (6, 6), its top edges round the hole filleted by 1.
fn plate() -> (Document, FeatureId) {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 20.0, 12.0)).hole(Shape::circle(6.0, 6.0, 3.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    (doc, e)
}

fn names(o: &Outcome) -> Vec<String> {
    let mut out: Vec<String> = o.bodies.iter().flat_map(|b| b.body.faces.iter()).map(|f| FaceRef::from_tag(f.tag).name()).collect();
    out.sort();
    out
}

#[test]
fn a_part_stretched_one_way_keeps_its_names() {
    let (mut doc, _) = plate();
    let before = ok(&mut doc);
    let volume = (240.0 - PI * 9.0) * 4.0;
    near(mass_properties(&before.bodies[0].body).volume, volume, "the plate");
    // Twice as long in x, about x = 0: the hole an ellipse 12 by 6, its center at x = 12.
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "stretch", "axis": [1, 0, 0], "point": [0, 0, 0], "factor": 2}}"#);
    let o = ok(&mut doc);
    near(mass_properties(&o.bodies[0].body).volume, 2.0 * volume, "stretched");
    assert_eq!(names(&o), names(&before), "every face keeps its name");
    let b = &o.bodies[0].body;
    let xs = b.vertices.iter().map(|p| p.x).fold(f64::MIN, f64::max);
    near(xs, 40.0, "its far end");
    // About x = 10, the middle of the plate as it was drawn: it reaches from -10 to 30.
    let (mut other, _) = plate();
    feature(&mut other, r#"{"type": "transform", "motion": {"type": "stretch", "axis": [1, 0, 0], "point": [10, 0, 0], "factor": 2}}"#);
    let o2 = ok(&mut other);
    let xs = o2.bodies[0].body.vertices.iter().map(|p| p.x).fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(x), b.max(x)));
    assert!((xs.0 + 10.0).abs() < 1e-9 && (xs.1 - 30.0).abs() < 1e-9, "{xs:?}");
    // A later feature still finds the hole's wall by name: a fillet round its top.
    feature(&mut doc, r#"{"type": "fillet", "edges": {"between": ["f2:end", "f2:hole.0.side.0"]}, "radius": 0.5}"#);
    let o = ok(&mut doc);
    assert!(mass_properties(&o.bodies[0].body).volume < 2.0 * volume);
}

#[test]
fn a_stretch_follows_its_line_and_can_copy() {
    // Along a sketch's line, which moves with a parameter: its direction is the stretch's, and the plane through its start
    // square to it stays put.
    let (mut doc, _) = plate();
    doc.set_parameter("lean", 0.0).unwrap();
    let mut s = coreshape_sketch::Sketch::new();
    let line = s.line([0.0, 0.0], [0.0, 10.0]);
    s.constrain(coreshape_sketch::Rule::Coincident { a: line.start(), b: coreshape_sketch::model::ORIGIN.whole() });
    s.constrain(coreshape_sketch::Rule::Angle { a: line, b: None, value: "90 - lean".into() });
    s.constrain(coreshape_sketch::Rule::Distance { a: line.start(), b: Some(line.end()), value: 10.0.into() });
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let t = feature(&mut doc, &format!(r#"{{"type": "transform", "motion": {{"type": "stretch", "axis": {{"sketch": {}, "line": {}}}, "factor": 1.5}}, "copy": true}}"#, sk.0, line.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2, "the plate and its copy");
    let copy = o.bodies.iter().find(|b| b.id.text() != "f2").unwrap();
    near(mass_properties(&copy.body).volume, 1.5 * (240.0 - PI * 9.0) * 4.0, "the copy");
    let ys = copy.body.vertices.iter().map(|p| p.y).fold(f64::MIN, f64::max);
    near(ys, 18.0, "the copy's far side, along y");
    // Leaning the line 30 degrees, the copy stretches that way.
    doc.set_parameter("lean", 30.0).unwrap();
    let o = ok(&mut doc);
    let copy = o.bodies.iter().find(|b| b.id.text() != "f2").unwrap();
    let ys = copy.body.vertices.iter().map(|p| p.y).fold(f64::MIN, f64::max);
    assert!((ys - 18.0).abs() > 0.1, "it moved with the line: {ys}");
    let _ = t;
}

#[test]
fn what_a_stretch_refuses_and_how_its_written() {
    let (mut doc, _) = plate();
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "stretch", "axis": [1, 0, 0], "point": [0, 0, 0], "factor": 0}}"#);
    assert!(failure(&mut doc).contains("above 0"));
    for text in [
        r#"{"type": "transform", "motion": {"type": "stretch", "point": [0, 0, 0], "axis": [1, 0, 0], "factor": 2}, "copy": false}"#,
        r#"{"type": "transform", "motion": {"type": "stretch", "axis": {"axis_of": "f2:hole.0.side.0"}, "factor": "k"}, "copy": true}"#,
    ] {
        let v = json::parse(text).unwrap();
        assert_eq!(op_value(&read_op(&v, &[]).unwrap()).to_text(), v.to_text(), "{text}");
    }
    let why = read_op(&json::parse(r#"{"type": "transform", "motion": {"type": "stretch", "axis": [1, 0, 0], "factor": 2}}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("point"), "{why}");
}
