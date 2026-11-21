//! Thin extrudes: a wall along a sketch's outlines and open curves, inside, outside or both, each held against its area
//! worked out by hand, with its faces named after the curves they follow.

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
    assert!((a - b).abs() < 1e-9 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

/// The names of a feature's faces on the result.
fn names(o: &Outcome, f: FeatureId) -> Vec<String> {
    let mut out: Vec<String> = o.bodies.iter().flat_map(|b| b.body.faces.iter()).filter(|x| x.tag.feature == f.0).map(|x| FaceRef::from_tag(x.tag).name()).collect();
    out.sort();
    out.dedup();
    out
}

/// A thin extrude 5 high of a sketch holding `shape`.
fn walled(shape: Shape, thickness: f64, side: &str) -> std::result::Result<(Outcome, FeatureId), String> {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(shape)]));
    let e = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 5, "thin": {{"thickness": {thickness}, "side": "{side}"}}}}"#, s.0));
    let o = doc.rebuild(None);
    match o.failed {
        None => Ok((o, e)),
        Some(i) => match &o.steps[i].state {
            StepState::Failed(why) => Err(why.clone()),
            other => panic!("{other:?}"),
        },
    }
}

#[test]
fn a_wall_round_an_outline_goes_inside_outside_or_both() {
    // A square 10 across, a wall 1 thick.
    for (side, area) in [("inside", 100.0 - 64.0), ("outside", 144.0 - 100.0), ("both", 121.0 - 81.0)] {
        let (o, _) = walled(Shape::rect(0.0, 0.0, 10.0, 10.0), 1.0, side).unwrap();
        near(vol(&o), area * 5.0, &format!("square, {side}"));
    }
    // A circle of radius 5.
    for (side, area) in [("inside", 25.0 - 16.0), ("outside", 36.0 - 25.0), ("both", 5.5 * 5.5 - 4.5 * 4.5)] {
        let (o, _) = walled(Shape::circle(0.0, 0.0, 5.0), 1.0, side).unwrap();
        near(vol(&o), PI * area * 5.0, &format!("circle, {side}"));
    }
    // A rounded rectangle 20 by 12 with corners of radius 3: inside, the corners round at 2 about the same centers.
    let area = |w: f64, h: f64, r: f64| w * h - (4.0 - PI) * r * r;
    let (o, e) = walled(Shape::RoundedRect { cx: 0.0.into(), cy: 0.0.into(), w: 20.0.into(), h: 12.0.into(), r: 3.0.into() }, 1.0, "inside").unwrap();
    near(vol(&o), (area(20.0, 12.0, 3.0) - area(18.0, 10.0, 2.0)) * 5.0, "rounded rectangle");
    // Its faces: the outside follows the outline, named as an extrude names it, and the inside is its inner side.
    let n = names(&o, e);
    for want in ["side.0", "side.1", "inner.side.0", "inner.side.7"] {
        assert!(n.iter().any(|x| x.ends_with(&format!(":{want}"))), "{want} in {n:?}");
    }
    // A region with a hole: a wall round each, into the material between them.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 20.0, 20.0)).hole(Shape::circle(10.0, 10.0, 4.0))]));
    let e = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 5, "thin": {{"thickness": 1}}}}"#, s.0));
    let o = ok(&mut doc);
    near(vol(&o), ((400.0 - 324.0) + PI * (25.0 - 16.0)) * 5.0, "square with a hole");
    let n = names(&o, e);
    assert!(n.iter().any(|x| x.ends_with(":hole.0.side.0")) && n.iter().any(|x| x.ends_with(":inner.hole.0.side.0")), "{n:?}");
}

#[test]
fn a_wall_along_an_open_curve_has_square_ends() {
    // An L, 10 along x then 8 up, a wall 2 thick about it: its sharp outside corner and cut-back inside corner make an
    // area of exactly its length times its thickness.
    let l = Shape::OpenPath { start: [0.0.into(), 0.0.into()], moves: vec![Move::Line { to: [10.0.into(), 0.0.into()] }, Move::Line { to: [10.0.into(), 8.0.into()] }] };
    let (o, e) = walled(l.clone(), 2.0, "both").unwrap();
    near(vol(&o), 18.0 * 2.0 * 5.0, "an L, both sides");
    let b = o.bodies[0].body.aabb();
    assert!(b.min.dist(v3(0.0, -1.0, 0.0)) < 1e-9 && b.max.dist(v3(11.0, 8.0, 5.0)) < 1e-9, "{b:?}");
    let n = names(&o, e);
    for want in ["side.0", "side.1", "inner.side.0", "inner.side.1", "start_of.side.0", "end_of.side.1", "start", "end"] {
        assert!(n.iter().any(|x| x.ends_with(&format!(":{want}"))), "{want} in {n:?}");
    }
    // On its left, inside the corner: the first leg is 8 long on that side, the second 8, less the corner square.
    let (o, _) = walled(l, 2.0, "inside").unwrap();
    near(vol(&o), (10.0 * 2.0 + 8.0 * 2.0 - 4.0) * 5.0, "an L, on its left");
    // An arc of radius 6 through a quarter turn, counter-clockwise, its wall on the left, toward its center.
    let arc = Shape::OpenPath { start: [6.0.into(), 0.0.into()], moves: vec![Move::Arc { to: [0.0.into(), 6.0.into()], center: [0.0.into(), 0.0.into()], ccw: true }] };
    let (o, _) = walled(arc, 1.5, "inside").unwrap();
    near(vol(&o), PI / 4.0 * (36.0 - 4.5 * 4.5) * 5.0, "an arc");
}

#[test]
fn later_features_find_a_wall_by_its_names() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    let e = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 5, "thin": {{"thickness": 1}}}}"#, s.0));
    let f = feature(&mut doc, &format!(r#"{{"type": "fillet", "edges": {{"between": ["f{0}:end", "f{0}:inner.side.1"]}}, "radius": 0.3}}"#, e.0));
    let o = ok(&mut doc);
    assert!(o.steps.iter().any(|x| x.id == f && x.state == StepState::Rebuilt));
    assert!(vol(&o) < 36.0 * 5.0);
    for name in ["f2:inner.side.1", "f2:region.3.inner.hole.2.side.4", "f2:start_of.side.0", "f2:region.1.end_of.side.7"] {
        assert_eq!(FaceRef::parse(name).unwrap().name(), name);
    }
    for name in ["f2:inner.inner.side.1", "f2:start_of.hole.0.side.1", "f2:inner.region.300.side.0", "f2:inner"] {
        assert!(FaceRef::parse(name).is_err(), "{name}");
    }
}

#[test]
fn a_thin_cut_and_ends_up_to_something() {
    // A ring groove 1 wide, round a circle of radius 5, cut right through a block.
    let mut doc = Document::new();
    let b = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-10.0, -10.0, 20.0, 20.0))]));
    doc.with(Op::extrude(b, 0.0, 10.0, Mode::Add));
    let s = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "mode": "remove", "end": {{"type": "through_all"}}, "opposite": true, "thin": {{"thickness": 1}}}}"#, s.0));
    let o = ok(&mut doc);
    // The core comes away as a part of its own.
    near(vol(&o), 4000.0 - PI * 9.0 * 10.0, "the block less the groove");
    assert_eq!(o.bodies.len(), 2);
}

#[test]
fn a_constrained_sketch_names_walls_by_entity_and_keeps_them() {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let b = s.line([10.0, 0.0], [10.0, 6.0]);
    s.constrain(Rule::Coincident { a: a.start(), b: ORIGIN.whole() });
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: b.whole(), b: None });
    s.constrain(Rule::Distance { a: a.start(), b: Some(a.end()), value: "w".into() });
    s.constrain(Rule::Distance { a: b.start(), b: Some(b.end()), value: 6.0.into() });
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let e = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 3, "thin": {{"thickness": 1, "side": "outside"}}}}"#, sk.0));
    for w in [10.0, 15.0] {
        doc.set_parameter("w", w).unwrap();
        let o = ok(&mut doc);
        // On its right: the first leg, w + 1 with the corner, and the second, 6.
        near(vol(&o), ((w + 1.0) + 6.0) * 3.0, "an L on its right");
        let n = names(&o, e);
        for want in [format!("side.{}", a.0), format!("inner.side.{}", b.0), format!("start_of.side.{}", a.0), format!("end_of.side.{}", b.0)] {
            assert!(n.iter().any(|x| x.ends_with(&format!(":{want}"))), "{want} in {n:?}");
        }
    }
}

#[test]
fn what_a_thin_extrude_refuses() {
    // Thicker than a circle is round.
    let why = walled(Shape::circle(0.0, 0.0, 1.0), 2.0, "inside").unwrap_err();
    assert!(why.contains("thicker than the arc is round"), "{why}");
    // A spline.
    let wavy = Shape::Path { start: [0.0.into(), 0.0.into()], moves: vec![Move::Line { to: [10.0.into(), 0.0.into()] }, Move::Spline { through: vec![[12.0.into(), 5.0.into()], [0.0.into(), 6.0.into()]] }, Move::Line { to: [0.0.into(), 0.0.into()] }] };
    let why = walled(wavy, 1.0, "inside").unwrap_err();
    assert!(why.contains("spline"), "{why}");
    // A corner so sharp that its outside goes a long way out.
    let v = Shape::OpenPath { start: [0.0.into(), 0.0.into()], moves: vec![Move::Line { to: [20.0.into(), 0.0.into()] }, Move::Line { to: [0.0.into(), 0.1.into()] }] };
    let why = walled(v, 1.0, "both").unwrap_err();
    assert!(why.contains("too sharp") || why.contains("too thick") || why.contains("crosses itself"), "{why}");
    // A hairpin 1 across, with a wall 2 thick inside it, whose turn would run backwards.
    let hairpin = Shape::OpenPath { start: [0.0.into(), 0.0.into()], moves: vec![Move::Line { to: [10.0.into(), 0.0.into()] }, Move::Line { to: [10.0.into(), 1.0.into()] }, Move::Line { to: [0.0.into(), 1.0.into()] }] };
    let why = walled(hairpin.clone(), 2.0, "inside").unwrap_err();
    assert!(why.contains("too thick for the segment from (10.0000, 0.0000) to (10.0000, 1.0000)"), "{why}");
    // Outside it, the same wall goes round the turn.
    let (o, _) = walled(hairpin, 2.0, "outside").unwrap();
    near(vol(&o), (12.0 * 5.0 - 10.0 * 1.0) * 5.0, "a hairpin, outside");
    // As a surface.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 5, "mode": "surface", "thin": {{"thickness": 1}}}}"#, s.0));
    assert!(failure(&mut doc).contains("a surface has none"));
    // Written and read back.
    for text in [
        r#"{"type": "extrude", "sketch": 1, "start": 0, "length": 5, "mode": "add", "thin": {"thickness": 1, "side": "inside"}}"#,
        r#"{"type": "extrude", "sketch": 1, "start": 0, "mode": "remove", "end": "through_all", "thin": {"thickness": "t", "side": "both"}}"#,
    ] {
        let v = json::parse(text).unwrap();
        assert_eq!(op_value(&read_op(&v, &[]).unwrap()).to_text(), v.to_text(), "{text}");
    }
    let why = read_op(&json::parse(r#"{"type": "extrude", "sketch": 1, "length": 5, "thin": {"thickness": 1, "side": "left"}}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("inside, outside or both"), "{why}");
}
