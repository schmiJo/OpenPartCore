//! Ribs: a gusset in an L bracket, sketched on its middle plane and running across from its line to the corner, and ribs
//! along an open box's floor, sketched on its rim and running down. Each volume is worked out by hand.

use coreshape::*;
use coreshape_document::*;

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

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn p(x: impl Into<Expr>, y: impl Into<Expr>) -> Point2 {
    [x.into(), y.into()]
}

/// An L bracket 10 deep in y: a plate 20 along x and 2 thick, and an upright 2 thick and 20 high at x = 0. Its volume
/// is 760. And a sketch on the plane through its middle, y = 5, whose x is X and whose y is Z, holding `line`.
fn bracket(line: Shape) -> (Document, FeatureId) {
    let mut doc = Document::new();
    doc.set_parameter("g", 12.0).unwrap();
    let front = |y: f64| Placement::new(point3(0.0, y, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0));
    let l = Shape::Polygon { points: vec![p(0.0, 0.0), p(20.0, 0.0), p(20.0, 2.0), p(2.0, 2.0), p(2.0, 20.0), p(0.0, 20.0)] };
    let s = doc.with(Op::sketch(front(0.0), vec![Region::new(l)]));
    doc.with(Op::Extrude { sketch: Some(s), start: 0.0.into(), length: 10.0.into(), mode: Mode::Add, regions: Vec::new(), faces: Vec::new(), end: ExtrudeEnd::Blind, opposite: true, thin: None });
    let rib = doc.with(Op::sketch(front(5.0), vec![Region::new(line)]));
    (doc, rib)
}

fn gusset_line() -> Shape {
    Shape::OpenPath { start: p(2.0, "g"), moves: vec![Move::Line { to: p("g", 2.0) }] }
}

#[test]
fn a_gusset_fills_the_corner_of_a_bracket() {
    let (mut doc, s) = bracket(gusset_line());
    assert!(ok(&mut doc).bodies[0].body.aabb().max.y > 9.0, "the bracket runs along +y");
    near(vol(&ok(&mut doc)), 760.0, "the bracket");
    // Across from the line, toward the corner: its left as drawn is away from it, so it's flipped.
    let r = feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2, "flip": true}}"#, s.0));
    // The triangle between the line and the bracket's inside faces, 2 thick.
    for g in [12.0, 8.0] {
        doc.set_parameter("g", g).unwrap();
        let o = ok(&mut doc);
        assert_eq!(o.bodies.len(), 1, "one part");
        near(vol(&o), 760.0 + (g - 2.0) * (g - 2.0) / 2.0 * 2.0, &format!("with a gusset at {g}"));
        assert!(o.bodies[0].body.faces.iter().any(|f| f.tag.feature == r.0), "the rib's faces are named for it");
    }
    assert_eq!(read_op(&op_value(&doc.feature(r).unwrap().op), &[]).unwrap(), doc.feature(r).unwrap().op);
    // The other way it runs off past the bracket's ends, and is refused.
    let (mut doc, s) = bracket(gusset_line());
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("runs on past the parts") && why.contains("flip it"), "{why}");
}

/// An open box, 30 by 20 outside and 10 high, its walls and floor 2 thick: 6000 less a cavity of 26 by 16 by 8.
fn open_box() -> Document {
    let mut doc = Document::new();
    let outside = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 20.0))]));
    doc.with(Op::extrude(outside, 0.0, 10.0, Mode::Add));
    let inside = doc.with(Op::sketch(Placement::xy(2.0), vec![Region::new(Shape::rect(2.0, 2.0, 26.0, 16.0))]));
    doc.with(Op::extrude(inside, 0.0, 8.0, Mode::Remove));
    doc
}

#[test]
fn ribs_run_down_to_a_box_floor_and_out_to_its_walls() {
    // A line across the middle, short of the walls, on the rim, down to the floor.
    let mut doc = open_box();
    near(vol(&ok(&mut doc)), 6000.0 - 26.0 * 16.0 * 8.0, "the box");
    let s = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::OpenPath { start: p(5.0, 10.0), moves: vec![Move::Line { to: p(25.0, 10.0) }] })]));
    let r = feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2, "direction": "normal", "flip": true}}"#, s.0));
    let o = ok(&mut doc);
    // Carried out to the walls, from x 2 to 28, 2 thick, from the floor at 2 up to the rim at 10.
    near(vol(&o), 6000.0 - 3328.0 + 26.0 * 2.0 * 8.0, "a rib wall to wall");
    // Without carrying its ends on, it stops where the line does.
    let mut edited = doc.feature(r).unwrap().op.clone();
    if let Op::Rib { extend, .. } = &mut edited {
        *extend = false;
    }
    doc.edit(r, edited).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), 6000.0 - 3328.0 + 20.0 * 2.0 * 8.0, "a rib as long as its line");

    // A cross of two lines, each its own curve, which meet and join.
    let mut doc = open_box();
    let cross = vec![
        Region::new(Shape::OpenPath { start: p(5.0, 10.0), moves: vec![Move::Line { to: p(25.0, 10.0) }] }),
        Region::new(Shape::OpenPath { start: p(15.0, 5.0), moves: vec![Move::Line { to: p(15.0, 15.0) }] }),
    ];
    let s = doc.with(Op::sketch(Placement::xy(10.0), cross));
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2, "direction": "normal", "flip": true}}"#, s.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    near(vol(&o), 6000.0 - 3328.0 + (26.0 * 2.0 + 16.0 * 2.0 - 4.0) * 8.0, "two ribs across");
    // The box's faces keep their names, though the second rib met them.
    let names: Vec<String> = o.bodies[0].body.faces.iter().filter(|f| f.tag.feature == 2 || f.tag.feature == 4).map(|f| FaceRef::from_tag(f.tag).name()).collect();
    assert!(names.iter().all(|n| !n.contains("region")), "{names:?}");
    feature(&mut doc, r#"{"type": "fillet", "edges": {"between": ["f4:start", "f4:side.0"]}, "radius": 0.5}"#);
    ok(&mut doc);

    // Up from the rim there's nothing, so it runs off.
    let mut doc = open_box();
    let s = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::OpenPath { start: p(5.0, 10.0), moves: vec![Move::Line { to: p(25.0, 10.0) }] })]));
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2, "direction": "normal"}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("runs on past the parts"), "{why}");
}

#[test]
fn what_a_rib_refuses() {
    // Across from an arc.
    let arc = Shape::OpenPath { start: p(2.0, 12.0), moves: vec![Move::Arc { to: p(12.0, 2.0), center: p(12.0, 12.0), ccw: true }] };
    let (mut doc, s) = bracket(arc);
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 2, "flip": true}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("straight lines"), "{why}");
    // A sketch with no open curves.
    let (mut doc, _) = bracket(gusset_line());
    feature(&mut doc, r#"{"type": "rib", "sketch": 1, "thickness": 2}"#);
    let why = failure(&mut doc);
    assert!(why.contains("open curves"), "{why}");
    // A curve inside the part.
    let (mut doc, s) = bracket(Shape::OpenPath { start: p(5.0, 1.0), moves: vec![Move::Line { to: p(15.0, 1.0) }] });
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 0.5, "extend": false}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("nothing between it and the parts"), "{why}");
    for text in [
        r#"{"type": "rib", "sketch": 3, "thickness": 2, "direction": "parallel", "flip": false, "extend": true}"#,
        r#"{"type": "rib", "sketch": 3, "thickness": "t", "direction": "normal", "flip": true, "extend": false}"#,
    ] {
        let v = json::parse(text).unwrap();
        assert_eq!(op_value(&read_op(&v, &[]).unwrap()).to_text(), v.to_text(), "{text}");
    }
    let why = read_op(&json::parse(r#"{"type": "rib", "sketch": 3, "thickness": 2, "direction": "up"}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("parallel or normal"), "{why}");
}

#[test]
fn a_rib_meets_a_thin_wall() {
    // A tray: walls 2 thick inside a rounded outline 60 by 40 with corners of 8, 18 high, and a floor 2 thick. A rib along
    // its middle, sketched at 12, runs down to the floor and out to the walls.
    let mut doc = Document::new();
    let outline = || Region::new(Shape::RoundedRect { cx: 0.0.into(), cy: 0.0.into(), w: 60.0.into(), h: 40.0.into(), r: 8.0.into() });
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![outline()]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 18, "thin": {{"thickness": 2}}}}"#, s.0));
    let floor = doc.with(Op::sketch(Placement::xy(0.0), vec![outline()]));
    doc.with(Op::extrude(floor, 0.0, 2.0, Mode::Add));
    let r = doc.with(Op::sketch(Placement::xy(12.0), vec![Region::new(Shape::OpenPath { start: p(-10.0, 0.0), moves: vec![Move::Line { to: p(10.0, 0.0) }] })]));
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 1.5, "direction": "normal", "flip": true}}"#, r.0));
    let o = ok(&mut doc);
    let area = |w: f64, h: f64, r: f64| w * h - (4.0 - std::f64::consts::PI) * r * r;
    let ring = area(60.0, 40.0, 8.0) - area(56.0, 36.0, 6.0);
    near(vol(&o), ring * 16.0 + area(60.0, 40.0, 8.0) * 2.0 + 56.0 * 1.5 * 10.0, "walls, floor and rib");
    // The walls keep their names.
    let names: Vec<String> = o.bodies[0].body.faces.iter().filter(|f| f.tag.feature == 2).map(|f| FaceRef::from_tag(f.tag).name()).collect();
    assert!(names.iter().any(|n| n == "f2:inner.side.1"), "{names:?}");
}
