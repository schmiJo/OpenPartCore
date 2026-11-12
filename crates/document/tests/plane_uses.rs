//! Features that take a plane or a line naming one, and following it: mirror, trim, draft, an extrude's end, patterns and a
//! transform's frames. Each result is held against what it should be by hand, and the number forms work as they did.

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
    assert!((a - b).abs() < 1e-9 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn extent(o: &Outcome) -> Aabb {
    let mut out = Aabb::EMPTY;
    for b in &o.bodies {
        for p in &b.body.vertices {
            out.add(*p);
        }
    }
    out
}

/// A block from the origin, `w` by 6 by 4: `f2:start` is its bottom, `f2:end` its top, and `f2:side.0` to `side.3` face -Y,
/// +X, +Y and -X.
fn block() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 6.0))]));
    doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    doc
}

#[test]
fn a_mirror_across_a_plane_follows_it() {
    for across in [r#""plane": "f2:side.1""#.to_string(), r#""plane": 3"#.to_string()] {
        let mut doc = block();
        feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:side.1", "distance": 0}"#);
        feature(&mut doc, &format!(r#"{{"type": "mirror", {across}}}"#));
        for w in [10.0, 17.0] {
            doc.set_parameter("w", w).unwrap();
            let o = ok(&mut doc);
            near(vol(&o), 2.0 * w * 24.0, &across);
            near(extent(&o).max.x, 2.0 * w, &across);
        }
    }
    // Across Right, the YZ plane.
    let mut doc = block();
    feature(&mut doc, r#"{"type": "mirror", "plane": "right"}"#);
    let o = ok(&mut doc);
    near(extent(&o).min.x, -10.0, "across Right");
    near(vol(&o), 480.0, "across Right");
    // Not both ways at once.
    let why = read_op(&json::parse(r#"{"type": "mirror", "plane": "right", "normal": [1, 0, 0]}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("not both"), "{why}");
}

#[test]
fn a_trim_by_a_plane_follows_it() {
    // A square sheet on Front, 10 by 10, cut by a plane h above Top: what's above it stays.
    let mut doc = Document::new();
    doc.set_parameter("h", 3.0).unwrap();
    let s = doc.with(Op::sketch(Placement::on_plane(PlaneRef::Front), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    doc.with(Op::Fill { sketch: s });
    feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": "h"}"#);
    let t = feature(&mut doc, r#"{"type": "trim", "plane": 3, "keep": "outside"}"#);
    for h in [3.0, 7.5] {
        doc.set_parameter("h", h).unwrap();
        let o = ok(&mut doc);
        near(mass_properties(&o.bodies[0].body).area, 10.0 * (10.0 - h), "what's left");
        near(extent(&o).min.z, h, "where it was cut");
    }
    assert_eq!(read_op(&op_value(&doc.feature(t).unwrap().op), &[]).unwrap(), doc.feature(t).unwrap().op);
}

#[test]
fn a_draft_on_a_neutral_plane_is_the_draft_along_its_normal() {
    let sides = r#"["f2:side.0", "f2:side.1", "f2:side.2", "f2:side.3"]"#;
    let drafted = |neutral: &str| {
        let mut doc = block();
        feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 2}"#);
        feature(&mut doc, &format!(r#"{{"type": "draft", "faces": {sides}, {neutral}, "degrees": 5}}"#));
        vol(&ok(&mut doc))
    };
    // On the bottom face, which faces down, the pull is down, as numbers say it.
    let on_face = drafted(r#""neutral": "f2:start""#);
    let as_numbers = drafted(r#""pull": [0, 0, -1], "neutral": [0, 0, 0]"#);
    near(on_face, as_numbers, "on the bottom face");
    // On a plane halfway up, facing up.
    let on_plane = drafted(r#""neutral": 3"#);
    let up = drafted(r#""pull": [0, 0, 1], "neutral": [0, 0, 2]"#);
    near(on_plane, up, "on a plane");
    // By hand: the block's sides lean out by tan 5 degrees for every unit away from the neutral plane against the pull,
    // so its cross-section at height z is (10 + 2zt) by (6 + 2zt), drafted from the bottom, and integrated over 0 to 4.
    let t = 5f64.to_radians().tan();
    near(on_face, 240.0 + 256.0 * t + 256.0 * t * t / 3.0, "drafted from the bottom, pulling down");
    // Halfway up, pulling up, it widens below and narrows above, by the same.
    near(on_plane, 240.0 + 64.0 * t * t / 3.0, "drafted from halfway up, pulling up");
    // A neutral plane gives the pull, so it can't be given too.
    let why = read_op(&json::parse(&format!(r#"{{"type": "draft", "faces": {sides}, "neutral": "top", "pull": [0, 0, 1], "degrees": 5}}"#)).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("the pull is its normal"), "{why}");
}

#[test]
fn an_extrude_up_to_a_plane_follows_it() {
    let mut doc = Document::new();
    doc.set_parameter("d", 6.0).unwrap();
    let p = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": "d"}"#);
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 2.0))]));
    let e = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "end": {{"type": "up_to_face", "face": {}}}}}"#, s.0, p.0));
    for d in [6.0, 9.25] {
        doc.set_parameter("d", d).unwrap();
        near(vol(&ok(&mut doc)), PI * 4.0 * d, "up to the plane");
    }
    assert_eq!(read_op(&op_value(&doc.feature(e).unwrap().op), &[]).unwrap(), doc.feature(e).unwrap().op);
    // Up to Top from below it.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(-5.0), vec![Region::new(Shape::circle(0.0, 0.0, 2.0))]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "end": {{"type": "up_to_face", "face": "top"}}}}"#, s.0));
    near(vol(&ok(&mut doc)), PI * 4.0 * 5.0, "up to Top");
    // A plane the extrude runs along is never reached.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 2.0))]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "end": {{"type": "up_to_face", "face": "front"}}}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("the Front plane runs along the direction"), "{why}");
}

#[test]
fn patterns_along_a_line_and_round_an_axis_follow_them() {
    // Along a sketch's line, at 45 degrees: three blocks 1 across, 5 apart along it.
    let mut doc = Document::new();
    let mut s = coreshape_sketch::Sketch::new();
    s.line([0.0, 0.0], [3.0, 3.0]);
    s.circle([100.0, 100.0], 1.0);
    let guide = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let cube = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::NewBody));
    feature(&mut doc, &format!(r#"{{"type": "pattern", "of": [{}], "direction": {{"sketch": {}, "line": 1}}, "count": 3, "spacing": 5}}"#, cube.0, guide.0));
    let o = ok(&mut doc);
    near(vol(&o), 3.0, "three cubes");
    near(extent(&o).max.x, 1.0 + 10.0 / 2f64.sqrt(), "the last cube");

    // A line that turns with a parameter, the cube staying put: the copies turn with it.
    let mut doc = Document::new();
    doc.set_parameter("rise", 3.0).unwrap();
    let mut s = coreshape_sketch::Sketch::new();
    let l = s.line([0.0, 0.0], [3.0, 3.0]);
    s.constrain(coreshape_sketch::Rule::Coincident { a: l.start(), b: coreshape_sketch::model::ORIGIN.whole() });
    s.constrain(coreshape_sketch::Rule::HorizontalDistance { a: l.start(), b: l.end(), value: 4.0.into() });
    s.constrain(coreshape_sketch::Rule::VerticalDistance { a: l.start(), b: l.end(), value: "rise".into() });
    s.circle([100.0, 100.0], 1.0);
    let guide = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let cube = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::NewBody));
    let p = feature(&mut doc, &format!(r#"{{"type": "pattern", "of": [{}], "direction": {{"sketch": {}, "line": {}}}, "count": 2, "spacing": 5}}"#, cube.0, guide.0, l.0));
    near(extent(&ok(&mut doc)).max.y, 1.0 + 3.0, "along 4 by 3");
    doc.set_parameter("rise", 0.0).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.steps.iter().find(|x| x.id == p).unwrap().state, StepState::Rebuilt);
    near(extent(&o).max.y, 1.0, "along 4 by 0");
    near(extent(&o).max.x, 6.0, "along 4 by 0");

    // Four holes round a boss's axis, which moves with a parameter: the holes go with it.
    let mut doc = Document::new();
    doc.set_parameter("cx", 20.0).unwrap();
    let plate = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(plate, 0.0, 4.0, Mode::Add));
    let b = doc.with(Op::sketch(Placement::xy(4.0), vec![Region::new(Shape::circle("cx", 20.0, 3.0))]));
    let boss = doc.with(Op::extrude(b, 0.0, 5.0, Mode::Add));
    let h = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle("cx + 10", 20.0, 1.0))]));
    let hole = doc.with(Op::extrude(h, 0.0, 4.0, Mode::Remove));
    feature(&mut doc, &format!(r#"{{"type": "pattern", "of": [{}], "axis": {{"axis_of": "f{}:side.0"}}, "count": 4}}"#, hole.0, boss.0));
    for cx in [20.0, 31.0] {
        doc.set_parameter("cx", cx).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), 60.0 * 40.0 * 4.0 + PI * 9.0 * 5.0 - 4.0 * PI * 4.0, "plate, boss and four holes");
        let body = o.body.as_ref().unwrap();
        let mut centers: Vec<(f64, f64)> = body
            .faces
            .iter()
            .filter_map(|f| match f.surface {
                Surface::Cylinder { frame, radius } if (radius - 1.0).abs() < 1e-9 => Some(((frame.origin.x * 1e6).round() / 1e6, (frame.origin.y * 1e6).round() / 1e6)),
                _ => None,
            })
            .collect();
        centers.sort_by(|a, b| a.partial_cmp(b).unwrap());
        centers.dedup();
        assert_eq!(centers, vec![(cx - 10.0, 20.0), (cx, 10.0), (cx, 30.0), (cx + 10.0, 20.0)]);
    }
    // A line's center comes from it.
    let why = read_op(&json::parse(r#"{"type": "pattern", "axis": {"axis_of": "f4:side.0"}, "center": [0, 0, 0], "count": 4}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("center"), "{why}");
}

#[test]
fn a_transform_carries_one_plane_onto_another() {
    // A box 2 by 3 by 1 on Top, carried onto a plane 10 in front of Front, which faces -Y with x along X and y up.
    let mut doc = Document::new();
    doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 3.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::Add));
    feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "front", "distance": 10}"#);
    let t = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "frames", "from": {"on_plane": "top"}, "to": {"on_plane": 2}}}"#);
    let b = extent(&ok(&mut doc));
    for (got, want) in [(b.min, [0.0, -11.0, 0.0]), (b.max, [2.0, -10.0, 3.0])] {
        assert!(got.dist(v3(want[0], want[1], want[2])) < 1e-9, "{got:?} isn't {want:?}");
    }
    assert_eq!(read_op(&op_value(&doc.feature(t).unwrap().op), &[]).unwrap(), doc.feature(t).unwrap().op);
    // A solid placed on a plane follows it too.
    let mut doc = Document::new();
    doc.set_parameter("d", 5.0).unwrap();
    feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": "d"}"#);
    feature(&mut doc, r#"{"type": "solid", "shape": "box", "dx": 1, "dy": 1, "dz": 1, "at": {"on_plane": 1}}"#);
    near(extent(&ok(&mut doc)).min.z, 5.0, "on the plane");
    doc.set_parameter("d", 8.0).unwrap();
    near(extent(&ok(&mut doc)).min.z, 8.0, "on the plane once it's moved");
}

#[test]
fn the_number_forms_read_and_write_as_they_did() {
    for text in [
        r#"{"type": "mirror", "of": [], "point": [1, 0, 0], "normal": [1, 0, 0]}"#,
        r#"{"type": "trim", "point": [0, 0, 1], "normal": [0, 0, 1], "keep": "outside"}"#,
        r#"{"type": "draft", "faces": ["f2:side.0"], "pull": [0, 0, 1], "neutral": [0, 0, 0], "degrees": 5}"#,
        r#"{"type": "pattern", "of": [], "direction": [1, 0, 0], "count": 3, "spacing": 5}"#,
        r#"{"type": "pattern", "of": [], "center": [1, 2, 0], "axis": [0, 0, 1], "count": 3, "degrees": 360}"#,
        r#"{"type": "extrude", "sketch": 1, "start": 0, "mode": "add", "end": {"type": "up_to_face", "face": "f2:end"}}"#,
    ] {
        let v = json::parse(text).unwrap();
        let written = op_value(&read_op(&v, &[]).unwrap());
        assert_eq!(written.to_text(), v.to_text(), "{text}");
    }
    for text in [
        r#"{"type": "mirror", "of": [], "plane": "front"}"#,
        r#"{"type": "trim", "plane": "f2:end", "keep": "inside"}"#,
        r#"{"type": "draft", "faces": ["f2:side.0"], "neutral": 3, "degrees": 5}"#,
        r#"{"type": "pattern", "of": [], "direction": {"between": ["f2:start", "f2:side.0"]}, "count": 3, "spacing": 5}"#,
        r#"{"type": "pattern", "of": [], "axis": {"axis_of": "f4:side.0"}, "count": 3, "degrees": 360}"#,
        r#"{"type": "extrude", "sketch": 1, "start": 0, "mode": "add", "end": {"type": "up_to_face", "face": "top"}}"#,
        r#"{"type": "extrude", "sketch": 1, "start": 0, "mode": "add", "end": {"type": "up_to_face", "face": {"point": [0, 0, 3], "normal": [0, 0, 1]}}}"#,
    ] {
        let v = json::parse(text).unwrap();
        let read = read_op(&v, &[]).unwrap();
        assert_eq!(read_op(&op_value(&read), &[]).unwrap(), read, "{text}");
        assert_eq!(op_value(&read).to_text(), v.to_text(), "{text}");
    }
}
