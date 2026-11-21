//! A revolve, a helix and a rotation about a line named as other features name one: a sketch's line, a circle's axis, a
//! straight edge. Each follows the line when it moves, and the number forms read and write as they did.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::model::ORIGIN;
use coreshape_sketch::{Rule, Sketch};
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
    assert!((a - b).abs() < 1e-7 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn extent(o: &Outcome) -> Aabb {
    let mut out = Aabb::EMPTY;
    for b in &o.bodies {
        out.union(&b.body.aabb());
    }
    out
}

/// A rectangle from x 10 to 14 and y 0 to 6, held where it is, and an upright construction line at x = `a`.
fn rectangle_and_axis() -> (Sketch, coreshape_sketch::EntityId) {
    let mut s = Sketch::new();
    let corners = [[10.0, 0.0], [14.0, 0.0], [14.0, 6.0], [10.0, 6.0]];
    for k in 0..4 {
        let l = s.line(corners[k], corners[(k + 1) % 4]);
        s.fix(l).unwrap();
    }
    let axis = s.line([1.0, -2.0], [1.0, 8.0]);
    s.set_construction(axis, true).unwrap();
    s.constrain(Rule::HorizontalDistance { a: ORIGIN.whole(), b: axis.start(), value: "a".into() });
    s.constrain(Rule::HorizontalDistance { a: ORIGIN.whole(), b: axis.end(), value: "a".into() });
    (s, axis)
}

#[test]
fn a_revolve_goes_round_a_line_of_its_sketch_and_follows_it() {
    let mut doc = Document::new();
    doc.set_parameter("a", 2.0).unwrap();
    let (s, axis) = rectangle_and_axis();
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let r = feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "axis": {{"sketch": {}, "line": {}}}}}"#, sk.0, sk.0, axis.0));
    // Pappus: the rectangle's 24 of area goes round at its centroid's distance from the axis, 12 - a.
    for a in [2.0, 5.0, 9.0] {
        doc.set_parameter("a", a).unwrap();
        let o = ok(&mut doc);
        assert_eq!(o.steps.iter().find(|x| x.id == r).unwrap().state, StepState::Rebuilt);
        near(vol(&o), 2.0 * PI * (12.0 - a) * 24.0, &format!("round x = {a}"));
    }
    // Half a turn, the same way it went as numbers.
    let half = |axis: String| {
        let mut doc = Document::new();
        doc.set_parameter("a", 2.0).unwrap();
        let (s, _) = rectangle_and_axis();
        let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
        feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "degrees": 180, "axis": {axis}}}"#, sk.0));
        extent(&ok(&mut doc))
    };
    let named = half(format!(r#"{{"sketch": 1, "line": {}}}"#, axis.0));
    let numbers = half(r#"{"point": [2, 0], "direction": [0, 1]}"#.into());
    assert!(named.min.dist(numbers.min) < 1e-9 && named.max.dist(numbers.max) < 1e-9, "{named:?} isn't {numbers:?}");
    // A line in the world that lies in the sketch's plane does as well, and one that doesn't is refused.
    let world = half(r#"{"point": [2, 0, 0], "direction": [0, 1, 0]}"#.into());
    assert!(world.min.dist(numbers.min) < 1e-9 && world.max.dist(numbers.max) < 1e-9, "{world:?} isn't {numbers:?}");
    for (axis, says) in [(r#"{"point": [2, 0, 1], "direction": [0, 1, 0]}"#, "is 1.000000 off"), (r#"{"point": [2, 0, 0], "direction": [0, 1, 1]}"#, "runs out of")] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(10.0, 0.0, 4.0, 6.0))]));
        feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "axis": {axis}}}"#, s.0));
        let why = failure(&mut doc);
        assert!(why.contains("has to lie in its sketch's plane") && why.contains(says), "{why}");
    }
}

#[test]
fn a_helix_goes_round_a_circle_of_a_sketch_or_a_boss_and_follows_it() {
    // A square wire 1 across, 3 out from the axis, two turns of pitch 2, round a circle at (cx, 5) in another sketch.
    let spring = |axis: &str, numbers: bool| {
        let mut doc = Document::new();
        doc.set_parameter("cx", 4.0).unwrap();
        let mut s = Sketch::new();
        let c = s.circle([4.0, 5.0], 2.0);
        s.constrain(Rule::HorizontalDistance { a: ORIGIN.whole(), b: c.center(), value: "cx".into() });
        s.constrain(Rule::VerticalDistance { a: ORIGIN.whole(), b: c.center(), value: 5.0.into() });
        let guide = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
        // The wire on the plane through the axis facing -Y, at y = 5, and 3 out from it in x.
        let wire = doc.with(Op::sketch(Placement::new(point3(0.0, 5.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::rect("cx + 3", 0.0, 1.0, 1.0))]));
        let axis = axis.replace("GUIDE", &guide.0.to_string());
        let h = feature(&mut doc, &format!(r#"{{"type": "helix", "sketch": {}, {axis}, "pitch": 2, "turns": 2}}"#, wire.0));
        let mut boxes = Vec::new();
        for cx in [4.0, 10.0] {
            doc.set_parameter("cx", cx).unwrap();
            let o = ok(&mut doc);
            near(vol(&o), 2.0 * 2.0 * PI * 3.5 * 1.0, "Pappus along the helix, whose turns don't change its volume");
            boxes.push(extent(&o));
            if numbers {
                break;
            }
        }
        (boxes, doc.feature(h).unwrap().op.clone())
    };
    let (by_circle, op) = spring(r#""axis": {"sketch": GUIDE, "line": 1}"#, false);
    assert_eq!(read_op(&op_value(&op), &[]).unwrap(), op);
    let (by_numbers, _) = spring(r#""point": [4, 5, 0], "axis": [0, 0, 1]"#, true);
    assert!(by_circle[0].min.dist(by_numbers[0].min) < 1e-9 && by_circle[0].max.dist(by_numbers[0].max) < 1e-9, "{:?} isn't {:?}", by_circle[0], by_numbers[0]);
    // Moved 6 along x with the circle.
    near(by_circle[1].min.x - by_circle[0].min.x, 6.0, "the spring moved with the circle");
    near(by_circle[1].min.y, by_circle[0].min.y, "and only along x");

    // Round a boss's axis, by its face.
    let mut doc = Document::new();
    doc.set_parameter("cx", 0.0).unwrap();
    let b = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle("cx", 0.0, 2.0))]));
    let boss = doc.with(Op::extrude(b, 0.0, 10.0, Mode::NewBody));
    let wire = doc.with(Op::sketch(Placement::new(point3(0.0, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::rect("cx + 3", 0.0, 1.0, 1.0))]));
    feature(&mut doc, &format!(r#"{{"type": "helix", "sketch": {}, "axis": {{"axis_of": "f{}:side.0"}}, "pitch": 2, "turns": 2, "mode": "new_body"}}"#, wire.0, boss.0));
    for cx in [0.0, 7.0] {
        doc.set_parameter("cx", cx).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), PI * 4.0 * 10.0 + 2.0 * 2.0 * PI * 3.5, "the boss and the spring round it");
        let spring = o.bodies.iter().map(|b| b.body.aabb()).find(|bx| bx.max.x > cx + 3.9).unwrap();
        near((spring.min.x + spring.max.x) / 2.0, cx, "the spring is round the boss");
    }
    // Only the circle moves, and the wire stays: the helix still follows, its wire now nearer the axis.
    let mut doc = Document::new();
    doc.set_parameter("cx", 0.0).unwrap();
    let mut s = Sketch::new();
    let c = s.circle([0.0, 0.0], 2.0);
    s.constrain(Rule::HorizontalDistance { a: ORIGIN.whole(), b: c.center(), value: "cx".into() });
    s.constrain(Rule::VerticalDistance { a: ORIGIN.whole(), b: c.center(), value: 0.0.into() });
    let guide = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let wire = doc.with(Op::sketch(Placement::new(point3(0.0, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::rect(10.0, 0.0, 1.0, 1.0))]));
    let h = feature(&mut doc, &format!(r#"{{"type": "helix", "sketch": {}, "axis": {{"sketch": {}, "line": 1}}, "pitch": 2, "turns": 2}}"#, wire.0, guide.0));
    for cx in [0.0, 4.0] {
        doc.set_parameter("cx", cx).unwrap();
        let o = ok(&mut doc);
        assert_eq!(o.steps.iter().find(|x| x.id == h).unwrap().state, StepState::Rebuilt);
        near(vol(&o), 2.0 * 2.0 * PI * (10.5 - cx), &format!("the wire {} from the axis", 10.5 - cx));
    }

    // A line has its own point.
    let why = read_op(&json::parse(r#"{"type": "helix", "sketch": 1, "point": [0, 0, 0], "axis": {"axis_of": "f2:side.0"}, "pitch": 2, "turns": 2}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("takes no \"point\""), "{why}");
}

#[test]
fn a_rotation_turns_about_a_straight_edge_and_follows_it() {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 6.0))]));
    doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    // The upright edge at (w, 0), where the side facing -Y meets the one facing +X.
    let t = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "rotate", "axis": {"between": ["f2:side.0", "f2:side.1"]}, "degrees": 90}}"#);
    for w in [10.0, 13.0] {
        doc.set_parameter("w", w).unwrap();
        let o = ok(&mut doc);
        let b = extent(&o);
        near(b.max.x - b.min.x, 6.0, "its depth is now across");
        near(b.max.y - b.min.y, w, "and its width along y");
        assert!((b.min.x - w).abs() < 1e-9 || (b.max.x - w).abs() < 1e-9, "it turned about x = {w}: {b:?}");
        assert!(b.min.y.abs() < 1e-9 || b.max.y.abs() < 1e-9, "and y = 0: {b:?}");
        near(vol(&o), w * 24.0, "the same block");
    }
    assert_eq!(read_op(&op_value(&doc.feature(t).unwrap().op), &[]).unwrap(), doc.feature(t).unwrap().op);
    // A rotation as numbers still needs its point.
    let why = read_op(&json::parse(r#"{"type": "transform", "motion": {"type": "rotate", "axis": [0, 0, 1], "degrees": 90}}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("needs a \"point\""), "{why}");
}

#[test]
fn the_number_forms_read_and_write_as_they_did() {
    for text in [
        r#"{"type": "revolve", "sketch": 1, "degrees": 360, "mode": "add", "axis": {"point": [0, 0], "direction": [0, 1]}}"#,
        r#"{"type": "helix", "sketch": 1, "point": [0, 0, 0], "axis": [0, 0, 1], "pitch": 2, "turns": 3, "left_handed": false, "mode": "add"}"#,
        r#"{"type": "transform", "motion": {"type": "rotate", "point": [1, 2, 3], "axis": [0, 0, 1], "degrees": 90}, "copy": false}"#,
        r#"{"type": "revolve", "sketch": 1, "degrees": 360, "mode": "add", "axis": {"sketch": 1, "line": 5}}"#,
        r#"{"type": "revolve", "sketch": 1, "degrees": 360, "mode": "add", "axis": {"point": [0, 0, 0], "direction": [0, 1, 0]}}"#,
        r#"{"type": "helix", "sketch": 1, "axis": {"axis_of": "f2:side.0"}, "pitch": 2, "turns": 3, "left_handed": false, "mode": "add"}"#,
        r#"{"type": "transform", "motion": {"type": "rotate", "axis": {"between": ["f2:side.0", "f2:side.1"]}, "degrees": 90}, "copy": true}"#,
    ] {
        let v = json::parse(text).unwrap();
        let read = read_op(&v, &[]).unwrap();
        assert_eq!(read_op(&op_value(&read), &[]).unwrap(), read, "{text}");
        assert_eq!(op_value(&read).to_text(), v.to_text(), "{text}");
    }
    // A helix with nothing said about its axis goes up Z through the origin, as it did.
    let read = op(r#"{"type": "helix", "sketch": 1, "pitch": 2, "turns": 3}"#);
    let Op::Helix { axis, .. } = read else { panic!() };
    assert_eq!(axis, LineRef::along(point3(0.0, 0.0, 1.0)));
}
