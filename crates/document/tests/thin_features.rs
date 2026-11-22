//! Thin revolves, sweeps and lofts: a wall along a sketch's outlines and open curves taken round, along or through, each held
//! against its volume worked out by hand, with its faces named as a thin extrude's are.

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

fn has(n: &[String], want: &str) {
    assert!(n.iter().any(|x| x.ends_with(&format!(":{want}"))), "{want} in {n:?}");
}

fn open(points: &[(f64, f64)]) -> Shape {
    Shape::OpenPath {
        start: [points[0].0.into(), points[0].1.into()],
        moves: points[1..].iter().map(|p| Move::Line { to: [p.0.into(), p.1.into()] }).collect(),
    }
}

#[test]
fn a_thin_revolve_turns_a_wall_round_an_outline() {
    // A rectangle 10 out from the axis, 10 wide and 30 high, its wall 1 thick inside it. With no axis, profile x is the
    // distance from the sketch's Z axis. By Pappus, each rectangle turned through an angle a makes a * its middle's
    // distance from the axis * its area, and both have their middles 15 out.
    for degrees in [360.0, 90.0] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(10.0, 0.0, 10.0, 30.0))]));
        let r = feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "degrees": {degrees}, "thin": {{"thickness": 1}}}}"#, s.0));
        let o = ok(&mut doc);
        near(vol(&o), f64::to_radians(degrees) * 15.0 * (300.0 - 8.0 * 28.0), &format!("a ring turned {degrees} degrees"));
        let n = names(&o, r);
        for want in ["side.0", "side.3", "inner.side.0", "inner.side.3"] {
            has(&n, want);
        }
        if degrees < 360.0 {
            has(&n, "start");
            has(&n, "end");
        }
    }
    // Outside a circle of radius 2, 10 out: an annulus from 2 to 3 turned 360 degrees, its middle 10 out.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(10.0, 5.0, 2.0))]));
    feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "thin": {{"thickness": 1, "side": "outside"}}}}"#, s.0));
    near(vol(&ok(&mut doc)), 2.0 * PI * 10.0 * PI * (9.0 - 4.0), "a hollow ring");
}

#[test]
fn a_thin_revolve_of_an_open_curve_makes_a_cup() {
    // From the axis out to 20, then up 30, a wall 2 thick about it. Its start lies on the axis, square to it, so its end
    // there is on the axis and turns to nothing: a cup with a bottom 2 thick, from -1 to 1. Outside it's 21 round and 31
    // high, from -1 to 30, and inside 19 round and 29 high, from 1 to 30.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open(&[(0.0, 0.0), (20.0, 0.0), (20.0, 30.0)]))]));
    let r = feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "thin": {{"thickness": 2, "side": "both"}}}}"#, s.0));
    let o = ok(&mut doc);
    near(vol(&o), PI * (21.0 * 21.0 * 31.0 - 19.0 * 19.0 * 29.0), "a cup");
    let n = names(&o, r);
    for want in ["side.0", "side.1", "inner.side.0", "inner.side.1", "end_of.side.1"] {
        has(&n, want);
    }
    // Its rim is a flat ring, which a fillet can round.
    let f = feature(&mut doc, &format!(r#"{{"type": "fillet", "edges": {{"between": ["f{0}:end_of.side.1", "f{0}:side.1"]}}, "radius": 0.5}}"#, r.0));
    let o = ok(&mut doc);
    assert!(o.steps.iter().any(|x| x.id == f && x.state == StepState::Rebuilt));
    assert!(vol(&o) < PI * (21.0 * 21.0 * 31.0 - 19.0 * 19.0 * 29.0));

    // About a line drawn in the sketch, here the sketch's y axis, half way round: a half cup on its side.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open(&[(0.0, 0.0), (20.0, 0.0), (20.0, 30.0)]))]));
    feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "degrees": 180, "axis": {{"point": [0, 0], "direction": [0, 1]}}, "thin": {{"thickness": 2, "side": "both"}}}}"#, s.0));
    let o = ok(&mut doc);
    near(vol(&o), PI * (21.0 * 21.0 * 31.0 - 19.0 * 19.0 * 29.0) / 2.0, "half a cup");
    // It lies along world y, the line it turned about: its corners, where it starts and ends in the sketch's plane, run from
    // its bottom at -1 to its rim at 30, and 21 either way in x.
    let v = &o.bodies[0].body.vertices;
    let extent = |f: fn(&Vec3) -> f64| (v.iter().map(f).fold(f64::MAX, f64::min), v.iter().map(f).fold(f64::MIN, f64::max));
    let (x, y, z) = (extent(|p| p.x), extent(|p| p.y), extent(|p| p.z));
    assert!((x.0 + 21.0).abs() < 1e-9 && (x.1 - 21.0).abs() < 1e-9 && (y.0 + 1.0).abs() < 1e-9 && (y.1 - 30.0).abs() < 1e-9, "{x:?} {y:?}");
    assert!(z.0.abs() < 1e-9 && z.1.abs() < 1e-9, "{z:?}");
}

#[test]
fn a_thin_revolve_refuses_a_wall_across_its_axis() {
    // From the axis at a slant: the wall's end there reaches across it.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open(&[(0.0, 0.0), (10.0, 10.0)]))]));
    feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "thin": {{"thickness": 2, "side": "both"}}}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("the thin wall can't revolve") && why.contains("axis"), "{why}");
    // As a surface.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(10.0, 0.0, 10.0, 30.0))]));
    feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "mode": "surface", "thin": {{"thickness": 1}}}}"#, s.0));
    assert!(failure(&mut doc).contains("a surface has none"));
}

#[test]
fn a_thin_sweep_makes_a_tube() {
    // A circle of radius 5 with a wall 1 thick inside it, 20 along, a quarter bend of radius 15, then 10 along. The ring's
    // middle is on the path, so by Pappus the volume is its area times the path's length.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let w = feature(
        &mut doc,
        &format!(r#"{{"type": "sweep", "sketch": {}, "path": [{{"line": 20}}, {{"arc_radius": 15, "degrees": 90, "toward_degrees": 0}}, {{"line": 10}}], "thin": {{"thickness": 1}}}}"#, s.0),
    );
    let o = ok(&mut doc);
    near(vol(&o), PI * 9.0 * (30.0 + 15.0 * PI / 2.0), "a bent tube");
    let n = names(&o, w);
    for want in ["side.0", "inner.side.0", "start", "end"] {
        has(&n, want);
    }

    // Along a path sketch's curves: a strip 10 wide and 2 thick, 20 along x, then round a quarter of radius 10. The strip's
    // middle is on the path, so again its area times the path's length.
    let mut doc = Document::new();
    let profile = doc.with(Op::sketch(Placement::new([0.0.into(), 0.0.into(), 0.0.into()], [1.0.into(), 0.0.into(), 0.0.into()], [0.0.into(), 1.0.into(), 0.0.into()]), vec![Region::new(open(&[(-5.0, 0.0), (5.0, 0.0)]))]));
    let path = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::OpenPath {
            start: [0.0.into(), 0.0.into()],
            moves: vec![Move::Line { to: [20.0.into(), 0.0.into()] }, Move::Arc { to: [30.0.into(), 10.0.into()], center: [20.0.into(), 10.0.into()], ccw: true }],
        })],
    ));
    let w = feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": {{"sketch": {}}}, "thin": {{"thickness": 2, "side": "both"}}}}"#, profile.0, path.0));
    let o = ok(&mut doc);
    near(vol(&o), 10.0 * 2.0 * (20.0 + 10.0 * PI / 2.0), "a bent strip");
    let n = names(&o, w);
    for want in ["side.0", "inner.side.0", "start_of.side.0", "end_of.side.0"] {
        has(&n, want);
    }
    // A bend tighter than the wall reaches is refused, saying it's the wall.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": [{{"arc_radius": 5.5, "degrees": 90, "toward_degrees": 0}}], "thin": {{"thickness": 1, "side": "outside"}}}}"#, s.0));
    let why = failure(&mut doc);
    assert!(why.contains("the thin wall can't sweep"), "{why}");
}

#[test]
fn a_thin_loft_goes_through_each_sections_wall() {
    // Squares 20 across at height 0 and 10 across at 30, walls 1 thick inside them. Straight between two sections, the
    // outside is a frustum, h/3 (A + B + sqrt(AB)), and so is the inside, of squares 18 and 8 across.
    let frustum = |a: f64, b: f64, h: f64| h / 3.0 * (a * a + b * b + a * b);
    let mut doc = Document::new();
    let low = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-10.0, -10.0, 20.0, 20.0))]));
    let high = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::rect(-5.0, -5.0, 10.0, 10.0))]));
    let l = feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 1}}}}"#, low.0, high.0));
    let o = ok(&mut doc);
    near(vol(&o), frustum(20.0, 10.0, 30.0) - frustum(18.0, 8.0, 30.0), "a hollow frustum");
    let n = names(&o, l);
    for want in ["side.0", "side.3", "inner.side.0", "inner.side.3", "start", "end"] {
        has(&n, want);
    }

    // Open curves: a line 20 long at height 0 and one 10 long at 20, walls 2 thick about them. Each level is a rectangle
    // 2 thick whose length goes straight from 20 to 10.
    let mut doc = Document::new();
    let low = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open(&[(-10.0, 0.0), (10.0, 0.0)]))]));
    let high = doc.with(Op::sketch(Placement::xy(20.0), vec![Region::new(open(&[(-5.0, 0.0), (5.0, 0.0)]))]));
    let l = feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 2, "side": "both"}}}}"#, low.0, high.0));
    let o = ok(&mut doc);
    near(vol(&o), 2.0 * (20.0 + 10.0) / 2.0 * 20.0, "a tapering wall");
    let n = names(&o, l);
    for want in ["side.0", "inner.side.0", "start_of.side.0", "end_of.side.0"] {
        has(&n, want);
    }

    // A region with a hole: a wall round each, lofted to the same in the next section. A square 20 across with a circle of
    // radius 4 in it, to the same at 10 up: straight up, so walls 1 thick, inside, make 10 times their area.
    let mut doc = Document::new();
    let holed = |z: f64| Op::sketch(Placement::xy(z), vec![Region::new(Shape::rect(-10.0, -10.0, 20.0, 20.0)).hole(Shape::circle(0.0, 0.0, 4.0))]);
    let low = doc.with(holed(0.0));
    let high = doc.with(holed(10.0));
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 1}}}}"#, low.0, high.0));
    near(vol(&ok(&mut doc)), 10.0 * ((400.0 - 324.0) + PI * (25.0 - 16.0)), "two walls");
}

#[test]
fn a_thin_loft_needs_its_sections_to_match() {
    // An outline to an open curve.
    let mut doc = Document::new();
    let low = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-10.0, -10.0, 20.0, 20.0))]));
    let high = doc.with(Op::sketch(Placement::xy(20.0), vec![Region::new(open(&[(-5.0, 0.0), (5.0, 0.0)]))]));
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 1}}}}"#, low.0, high.0));
    let why = failure(&mut doc);
    assert!(why.contains("section 1") && why.contains("an open curve") && why.contains("an outline"), "{why}");
    // As a surface.
    let mut doc = Document::new();
    let low = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-10.0, -10.0, 20.0, 20.0))]));
    let high = doc.with(Op::sketch(Placement::xy(20.0), vec![Region::new(Shape::rect(-5.0, -5.0, 10.0, 10.0))]));
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "mode": "surface", "thin": {{"thickness": 1}}}}"#, low.0, high.0));
    assert!(failure(&mut doc).contains("a surface has none"));
}

#[test]
fn thin_features_are_written_and_read_back() {
    for text in [
        r#"{"type": "revolve", "sketch": 1, "degrees": 360, "mode": "add", "thin": {"thickness": 1, "side": "inside"}}"#,
        r#"{"type": "revolve", "sketch": 1, "degrees": 90, "mode": "remove", "axis": {"point": [0, 0], "direction": [0, 1]}, "thin": {"thickness": "t", "side": "both"}}"#,
        r#"{"type": "loft", "sections": [1, {"sketch": 2, "region": "r0123456789ab"}], "mode": "add", "thin": {"thickness": 2, "side": "outside"}}"#,
        r#"{"type": "sweep", "sketch": 1, "path": {"sketch": 2}, "mode": "add", "thin": {"thickness": 1, "side": "inside"}}"#,
        r#"{"type": "sweep", "sketch": 1, "path": [{"line": 10}], "mode": "new_body", "thin": {"thickness": 1, "side": "both"}}"#,
    ] {
        let v = json::parse(text).unwrap();
        let o = read_op(&v, &[]).unwrap_or_else(|e| panic!("{text}: {e}"));
        assert_eq!(op_value(&o).to_text(), v.to_text(), "{text}");
    }
    let why = read_op(&json::parse(r#"{"type": "sweep", "sketch": 1, "path": [{"line": 10}], "thin": {"thickness": 1, "depth": 2}}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("depth"), "{why}");
}

#[test]
fn a_constrained_sketchs_curves_drive_a_thin_revolve_sweep_and_loft() {
    use coreshape_sketch::model::ORIGIN;
    use coreshape_sketch::{Rule, Sketch};
    // A cup's profile as entities: out along x by "r", then up by 30. A parameter "t" sets every wall's thickness.
    let mut doc = Document::new();
    doc.set_parameter("r", 20.0).unwrap();
    doc.set_parameter("t", 2.0).unwrap();
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [20.0, 0.0]);
    let b = s.line([20.0, 0.0], [20.0, 30.0]);
    s.constrain(Rule::Coincident { a: a.start(), b: ORIGIN.whole() });
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: b.whole(), b: None });
    s.constrain(Rule::Distance { a: a.start(), b: Some(a.end()), value: "r".into() });
    s.constrain(Rule::Distance { a: b.start(), b: Some(b.end()), value: 30.0.into() });
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let r = feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "thin": {{"thickness": "t", "side": "both"}}}}"#, sk.0));
    let cup = |r: f64, t: f64| PI * ((r + t / 2.0).powi(2) * (30.0 + t / 2.0) - (r - t / 2.0).powi(2) * (30.0 - t / 2.0));
    for (radius, t) in [(20.0, 2.0), (25.0, 2.0), (25.0, 3.0)] {
        doc.set_parameter("r", radius).unwrap();
        doc.set_parameter("t", t).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), cup(radius, t), &format!("a cup {radius} round, {t} thick"));
        let n = names(&o, r);
        for want in [format!("side.{}", a.0), format!("inner.side.{}", b.0), format!("end_of.side.{}", b.0)] {
            has(&n, &want);
        }
    }
    doc.delete(r).unwrap();
    // The same curves swept 10 along their sketch's normal, and lofted to a copy 10 up: both a wall 30 + r long, as each
    // corner's outside reaches as far as its inside falls short.
    let w = feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": [{{"line": 10}}], "thin": {{"thickness": "t", "side": "both"}}}}"#, sk.0));
    let o = ok(&mut doc);
    near(vol(&o), (25.0 + 30.0) * 3.0 * 10.0, "a swept wall");
    has(&names(&o, w), &format!("start_of.side.{}", a.0));
    doc.delete(w).unwrap();
    let mut s = Sketch::new();
    let c = s.line([0.0, 0.0], [25.0, 0.0]);
    let d = s.line([25.0, 0.0], [25.0, 30.0]);
    s.constrain(Rule::Coincident { a: c.end(), b: d.start() });
    let up = doc.add(Op::ConstrainedSketch { plane: Placement::xy(10.0), sketch: s }).unwrap();
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": "t", "side": "both"}}}}"#, sk.0, up.0));
    near(vol(&ok(&mut doc)), (25.0 + 30.0) * 3.0 * 10.0, "a lofted wall");
    doc.set_parameter("t", 1.0).unwrap();
    near(vol(&ok(&mut doc)), (25.0 + 30.0) * 1.0 * 10.0, "a lofted wall, thinner");
}

#[test]
fn a_thin_loft_from_a_rectangle_to_a_circle_twists_both_its_sides_alike() {
    // A rectangle's corners fit a circle's quarters as well one way round as the other, so a loft twists one way or the other
    // by choice. A thin loft's two sides have to make the same one, or its wall would cross itself: this was refused, about
    // 21% of the way up. Its wall is what's between the loft of its outside and the loft of its inside.
    let loft = |low: Shape, high: Shape, thin: &str| -> f64 {
        let mut doc = Document::new();
        let a = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(low)]));
        let b = doc.with(Op::sketch(Placement::xy(35.0), vec![Region::new(high)]));
        feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}]{thin}}}"#, a.0, b.0));
        vol(&ok(&mut doc))
    };
    for t in [1.5, 0.2] {
        let wall = loft(Shape::rect(-20.0, -12.0, 40.0, 24.0), Shape::circle(0.0, 0.0, 10.0), &format!(r#", "thin": {{"thickness": {t}}}"#));
        let outside = loft(Shape::rect(-20.0, -12.0, 40.0, 24.0), Shape::circle(0.0, 0.0, 10.0), "");
        let inside = loft(Shape::rect(-20.0 + t, -12.0 + t, 40.0 - 2.0 * t, 24.0 - 2.0 * t), Shape::circle(0.0, 0.0, 10.0 - t), "");
        near(wall, outside - inside, &format!("a duct {t} thick"));
    }
}
