//! The hole feature: simple, counterbored, countersunk and tapped holes at sketch points or on a plane, blind, through
//! everything or through the first wall, each held against its volume worked out by hand, with its faces named for its
//! hole and its callout saying what it is.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::Sketch;
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

/// A block 60 by 40 by 20, from the origin up, and a sketch of points on its top.
fn block_with_points(points: &[(f64, f64)]) -> (Document, FeatureId, Vec<u32>) {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let mut sketch = Sketch::new();
    let ids = points.iter().map(|(x, y)| sketch.point(*x, *y).0).collect();
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch }).unwrap();
    (doc, at, ids)
}

const BLOCK: f64 = 60.0 * 40.0 * 20.0;

/// A cone's volume, from its base's radius and its height.
fn cone(r: f64, h: f64) -> f64 {
    PI * r * r * h / 3.0
}

/// A drill point's height, for a radius and an included angle in degrees.
fn point_height(r: f64, degrees: f64) -> f64 {
    r / (degrees / 2.0).to_radians().tan()
}

#[test]
fn a_blind_hole_has_a_drill_point() {
    let (mut doc, at, ids) = block_with_points(&[(15.0, 20.0), (45.0, 20.0)]);
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, at.0));
    let o = ok(&mut doc);
    let one = PI * 9.0 * 10.0 + cone(3.0, point_height(3.0, 118.0));
    near(vol(&o), BLOCK - 2.0 * one, "two blind holes");
    let n = names(&o, h);
    for id in &ids {
        assert!(n.contains(&format!("f{}:bore.{id}", h.0)), "{n:?}");
        assert!(n.contains(&format!("f{}:tip.{id}", h.0)), "{n:?}");
    }
    assert_eq!(o.holes.len(), 2);
    assert_eq!(o.holes[0].text, "⌀6 10 deep, 118° point");
    // A flat bottom.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10, "tip_angle": 180}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - 2.0 * PI * 9.0 * 10.0, "two flat-bottomed holes");
    assert!(names(&o, h).contains(&format!("f{}:floor.{}", h.0, ids[0])));
}

#[test]
fn clearance_holes_take_their_sizes_from_the_tables() {
    let (mut doc, at, ids) = block_with_points(&[(20.0, 20.0)]);
    let id = ids[0];
    // M6 at a normal fit is 6.6 across. Through all, it goes through the 20.
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "end": "through_all"}}"#, at.0));
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 3.3f64.powi(2) * 20.0, "an M6 clearance hole");
    assert_eq!(o.holes[0].text, "M6 clearance, ⌀6.6 through");
    // Close and loose fits are 6.4 and 7.
    for (fit, d) in [("close", 6.4), ("loose", 7.0)] {
        doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "fit": "{fit}", "end": "through_all"}}"#, at.0))).unwrap();
        near(vol(&ok(&mut doc)), BLOCK - PI * (d / 2.0f64).powi(2) * 20.0, fit);
    }
    // A counterbore for an M6 socket head: 11 across, DIN 974-1's series 1, and as deep as the head is high, 6, and
    // DIN 974-1's allowance of 0.4.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "style": "counterbore", "end": "through_all"}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * (5.5f64.powi(2) * 6.4 + 3.3f64.powi(2) * 13.6), "an M6 counterbored hole");
    assert_eq!(o.holes[0].text, "M6 clearance, ⌀6.6 through, counterbore ⌀11 6.4 deep");
    let n = names(&o, h);
    for part in ["cbore", "cbore_floor", "bore"] {
        assert!(n.contains(&format!("f{}:{part}.{id}", h.0)), "{part} in {n:?}");
    }
    // A countersink for an M6 countersunk head, DIN 74's form F: 13.71 across at 90 degrees, so 3.555 deep to the
    // hole's 6.6.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "style": "countersink", "end": "through_all"}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    let (big, small) = (6.855, 3.3);
    let sink = big - small;
    let frustum = PI * sink * (big * big + big * small + small * small) / 3.0;
    near(vol(&o), BLOCK - frustum - PI * small * small * (20.0 - sink), "an M6 countersunk hole");
    assert_eq!(o.holes[0].text, "M6 clearance, ⌀6.6 through, countersink ⌀13.71 × 90°");
    assert!(names(&o, h).contains(&format!("f{}:csink.{id}", h.0)));
    // Dimensions given win over the tables.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "style": "counterbore", "cbore_diameter": 12, "cbore_depth": 4, "end": "through_all"}}"#, at.0))).unwrap();
    near(vol(&ok(&mut doc)), BLOCK - PI * (36.0 * 4.0 + 3.3f64.powi(2) * 16.0), "a counterbore of its own");
    assert_eq!(hole_sizes().first(), Some(&"M1.6"));
}

#[test]
fn a_tapped_hole_is_drilled_at_the_tap_drill_and_notes_its_thread() {
    let (mut doc, at, _) = block_with_points(&[(20.0, 20.0)]);
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "tapped": true, "depth": 15}}"#, at.0));
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 6.25 * 15.0 - cone(2.5, point_height(2.5, 118.0)), "an M6 tapped hole, drilled 5");
    let hole = &o.holes[0];
    assert_eq!(hole.text, "M6×1 tapped 15 deep, drill ⌀5 15 deep, 118° point");
    assert_eq!(hole.thread, Some(ThreadCallout { size: "M6".into(), major: 6.0, pitch: 1.0, depth: Some(15.0) }));
    assert!((hole.start - v3(20.0, 20.0, 20.0)).norm() < 1e-9 && (hole.direction - v3(0.0, 0.0, -1.0)).norm() < 1e-12, "{hole:?}");
    // A fine pitch takes a bigger drill, and the thread can stop short of the bottom.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M8", "tapped": {{"pitch": 1, "depth": 10}}, "depth": 15}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 3.5f64.powi(2) * 15.0 - cone(3.5, point_height(3.5, 118.0)), "an M8×1 tapped hole, drilled 7");
    assert_eq!(o.holes[0].text, "M8×1 tapped 10 deep, drill ⌀7 15 deep, 118° point");
    // ISO 2306 rounds some fine pitches' drills to one that's made: M10×1.25 takes 8.8, not 8.75.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M10", "tapped": {{"pitch": 1.25}}, "depth": 15}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 4.4f64.powi(2) * 15.0 - cone(4.4, point_height(4.4, 118.0)), "an M10×1.25 tapped hole, drilled 8.8");
    assert!(o.steps.iter().find(|s| s.id == h).unwrap().notes.is_empty());
    // A pitch it doesn't list for the size is drilled at the major diameter less the pitch, and the step says so.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "tapped": {{"pitch": 0.5}}, "depth": 15}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 2.75f64.powi(2) * 15.0 - cone(2.75, point_height(2.75, 118.0)), "an M6×0.5 tapped hole, drilled 5.5");
    assert_eq!(o.steps.iter().find(|s| s.id == h).unwrap().notes, vec!["ISO 2306 lists no tap drill for M6×0.5: it's drilled at the major diameter less the pitch, 5.5".to_string()]);
    // Through, the thread goes all the way.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M10", "tapped": true, "end": "through_all"}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 4.25f64.powi(2) * 20.0, "an M10 tapped hole through");
    assert_eq!(o.holes[0].thread.as_ref().unwrap().depth, None);
    assert_eq!(o.holes[0].text, "M10×1.5 tapped through, drill ⌀8.5 through");
}

#[test]
fn a_blind_hole_deeper_than_the_part_says_it_goes_through() {
    let (mut doc, at, _) = block_with_points(&[(20.0, 20.0)]);
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 25}}"#, at.0));
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 9.0 * 20.0, "a blind hole right through");
    assert_eq!(o.steps.iter().find(|s| s.id == h).unwrap().notes, vec!["hole 1 at (20, 20, 20) is 25 deep, and the part ends 20 in along its axis, so it goes right through".to_string()]);
    // As deep as the part, with a flat bottom, it goes through too; one short of it doesn't, though its point does.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 20, "tip_angle": 180}}"#, at.0))).unwrap();
    assert_eq!(ok(&mut doc).steps.iter().find(|s| s.id == h).unwrap().notes.len(), 1);
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 19}}"#, at.0))).unwrap();
    assert!(ok(&mut doc).steps.iter().find(|s| s.id == h).unwrap().notes.is_empty());
}

#[test]
fn a_hole_starts_where_it_meets_the_part() {
    // Points on a sketch 10 above the block's top, and on one half way down inside it: both holes start at the top.
    let one = PI * 9.0 * 10.0 + cone(3.0, point_height(3.0, 118.0));
    for z in [30.0, 10.0, 20.0] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
        doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
        let mut sketch = Sketch::new();
        sketch.point(30.0, 20.0);
        let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(z), sketch }).unwrap();
        feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, at.0));
        let o = ok(&mut doc);
        near(vol(&o), BLOCK - one, &format!("a hole from a sketch at {z}"));
        assert!((o.holes[0].start.z - 20.0).abs() < 1e-9, "{:?}", o.holes[0]);
    }
    // From the sketch, 10 up, a hole 15 deep goes only 5 into the part, and its point below that.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let mut sketch = Sketch::new();
    sketch.point(30.0, 20.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(30.0), sketch }).unwrap();
    feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 15, "from_sketch": true}}"#, at.0));
    near(vol(&ok(&mut doc)), BLOCK - PI * 9.0 * 5.0 - cone(3.0, point_height(3.0, 118.0)), "a hole from the sketch");
}

#[test]
fn opposite_drills_along_the_sketchs_normal() {
    // A sketch on the bottom face's plane: against its normal, down, is away from the block.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let mut sketch = Sketch::new();
    sketch.point(30.0, 20.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, at.0));
    let why = failure(&mut doc);
    assert!(why.contains("never meets the part going that way"), "{why}");
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10, "opposite": true}}"#, at.0))).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - PI * 9.0 * 10.0 - cone(3.0, point_height(3.0, 118.0)), "a hole up from the bottom");
    assert!(o.holes[0].start.z.abs() < 1e-9 && o.holes[0].direction.z > 0.999);
}

#[test]
fn holes_on_a_face_follow_it() {
    // The block's top face's plane: its x is along X and its origin above the world's, so (30, 20) is the middle.
    let mut doc = Document::new();
    doc.set_parameter("h", 20.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, "h", Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [[30, 20], ["10 + 5", 10]], "size": "M5", "style": "counterbore", "end": "through_all"}}"#, e.0));
    // An M5 counterbore is 10 across and 5 and 0.4 deep.
    let one = |h: f64| PI * (25.0 * 5.4 + 2.75f64.powi(2) * (h - 5.4));
    for h in [20.0, 30.0] {
        doc.set_parameter("h", h).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), 60.0 * 40.0 * h - 2.0 * one(h), &format!("two M5 counterbored holes in a block {h} high"));
        assert!((o.holes[0].start - v3(30.0, 20.0, h)).norm() < 1e-9 && (o.holes[1].start - v3(15.0, 10.0, h)).norm() < 1e-9, "{:?}", o.holes);
    }
    // A point's dimensions are parameters too, and the parameter can't go while the hole uses it.
    let mut doc = Document::new();
    doc.set_parameter("x", 30.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [["x", 20]], "diameter": 6, "end": "through_all"}}"#, e.0));
    assert!((ok(&mut doc).holes[0].start.x - 30.0).abs() < 1e-9);
    doc.set_parameter("x", 40.0).unwrap();
    assert!((ok(&mut doc).holes[0].start.x - 40.0).abs() < 1e-9);
    assert!(doc.remove_parameter("x").is_err());
}

#[test]
fn up_to_next_goes_through_the_first_wall_only() {
    // Two plates, 0 to 10 and 15 to 25, as parts of their own. A hole from the top goes through the upper one.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 10.0, Mode::NewBody));
    doc.with(Op::extrude(s, 15.0, 10.0, Mode::NewBody));
    let mut sketch = Sketch::new();
    sketch.point(20.0, 20.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(25.0), sketch }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "size": "M8", "style": "counterbore", "cbore_depth": 3, "end": "up_to_next"}}"#, at.0));
    let o = ok(&mut doc);
    let plate = 40.0 * 40.0 * 10.0;
    near(mass_properties(&o.bodies[0].body).volume, plate, "the lower plate");
    near(mass_properties(&o.bodies[1].body).volume, plate - PI * (7.5f64.powi(2) * 3.0 + 4.5f64.powi(2) * 7.0), "the upper plate");
    assert_eq!(o.holes[0].text, "M8 clearance, ⌀9 through, counterbore ⌀15 3 deep");
    // Through all goes through both.
    doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, "size": "M8", "end": "through_all"}}"#, at.0))).unwrap();
    near(vol(&ok(&mut doc)), 2.0 * plate - 2.0 * PI * 4.5f64.powi(2) * 10.0, "through both plates");
    // A post 2 across standing 4 up from the lower plate, half way out under a hole 12 across: between the rays down the
    // middle and round the edge, but the ring half way out finds it, and the hole stops half way to its top, clear of it.
    let post = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 4.0.into() }, Placement::at([23.0.into(), 20.0.into(), 10.0.into()]), Mode::NewBody));
    let wide = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 12, "end": "up_to_next"}}"#, at.0));
    doc.delete(h).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), 2.0 * plate + PI * 4.0 - PI * 36.0 * 10.0, "a hole through the upper plate, clear of the post");
    assert_eq!(o.holes.iter().find(|x| x.feature == wide).unwrap().text, "⌀12 through");
    let _ = post;
}

#[test]
fn a_hole_opens_cleanly_on_a_round_face() {
    // A bar 20 across lying along x, and a counterbored hole straight down into it from a sketch above. The hole's top
    // carries on back past where its axis meets the bar, so the counterbore is a full circle at its floor; its walls are
    // what a cylinder cut from above would leave, and so is the volume.
    let mut doc = Document::new();
    doc.with(Op::solid(Primitive::Cylinder { radius: 10.0.into(), height: 60.0.into() }, Placement::new([0.0.into(), 0.0.into(), 0.0.into()], [1.0.into(), 0.0.into(), 0.0.into()], [0.0.into(), 1.0.into(), 0.0.into()]), Mode::Add));
    let mut sketch = Sketch::new();
    sketch.point(30.0, 0.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(15.0), sketch }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "style": "counterbore", "cbore_diameter": 12, "cbore_depth": 4, "end": "through_all"}}"#, at.0));
    let o = ok(&mut doc);
    assert!((o.holes[0].start - v3(30.0, 0.0, 10.0)).norm() < 1e-9, "{:?}", o.holes[0]);
    let n = names(&o, h);
    for part in ["cbore", "cbore_floor", "bore"] {
        assert!(n.iter().any(|x| x.starts_with(&format!("f{}:{part}.", h.0))), "{part} in {n:?}");
    }
    // The same cut made from two solids: the counterbore from 10 + 6 down to 6, and the hole all the way.
    let mut check = Document::new();
    check.with(Op::solid(Primitive::Cylinder { radius: 10.0.into(), height: 60.0.into() }, Placement::new([0.0.into(), 0.0.into(), 0.0.into()], [1.0.into(), 0.0.into(), 0.0.into()], [0.0.into(), 1.0.into(), 0.0.into()]), Mode::Add));
    let up = |z: f64| Placement::new([30.0.into(), 0.0.into(), z.into()], [0.0.into(), 0.0.into(), 1.0.into()], [1.0.into(), 0.0.into(), 0.0.into()]);
    check.with(Op::solid(Primitive::Cylinder { radius: 6.0.into(), height: 10.0.into() }, up(6.0), Mode::Remove));
    check.with(Op::solid(Primitive::Cylinder { radius: 3.0.into(), height: 30.0.into() }, up(-15.0), Mode::Remove));
    near(vol(&o), vol(&ok(&mut check)), "a counterbored hole into a bar");
}

/// A wedge 40 along x and 40 deep, from y = 0 to -40, 5 high at x = 0 and rising at `degrees` to x = 40, turned `turn`
/// degrees about the upright through its middle, (20, -20). Gives the height of its top in the middle.
fn wedge(doc: &mut Document, degrees: f64, turn: f64) -> f64 {
    let rise = 40.0 * degrees.to_radians().tan();
    let (s, c) = turn.to_radians().sin_cos();
    let origin = [(20.0 - 20.0 * c - 20.0 * s).into(), (-20.0 - 20.0 * s + 20.0 * c).into(), 0.0.into()];
    let side = Placement::new(origin, [s.into(), (-c).into(), 0.0.into()], [c.into(), s.into(), 0.0.into()]);
    let outline = vec![[0.0.into(), 0.0.into()], [40.0.into(), 0.0.into()], [40.0.into(), (5.0 + rise).into()], [0.0.into(), 5.0.into()]];
    let s = doc.with(Op::sketch(side, vec![Region::new(Shape::polygon(outline))]));
    doc.with(Op::extrude(s, 0.0, 40.0, Mode::Add));
    5.0 + rise / 2.0
}

/// Upright, at (20, -20) and a height.
fn upright(z: f64) -> Placement {
    Placement::new([20.0.into(), (-20.0).into(), z.into()], [0.0.into(), 0.0.into(), 1.0.into()], [1.0.into(), 0.0.into(), 0.0.into()])
}

#[test]
fn a_hole_clears_a_steep_face_it_goes_in_through() {
    // A counterbore 12 across and 4 deep, and a countersink 14 across at 90 degrees, down into a face at 30, 60 and 75
    // degrees: within its top's radius the face rises 3.5, 10.4 and 22.4 above where the axis meets it. All of that
    // within the top comes away, as cylinders cut from the floor up and a cone below show. The wedge is turned so the
    // way up the face is half way between two of the rays that find it, 15 degrees apart, where they find it lowest.
    for degrees in [30.0, 60.0, 75.0] {
        for countersink in [false, true] {
            let mut doc = Document::new();
            let z = wedge(&mut doc, degrees, 7.5);
            let mut sketch = Sketch::new();
            sketch.point(20.0, -20.0);
            let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(300.0), sketch }).unwrap();
            let style = if countersink { r#""style": "countersink", "csink_diameter": 14"# } else { r#""style": "counterbore", "cbore_diameter": 12, "cbore_depth": 4"# };
            let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, {style}, "end": "through_all"}}"#, at.0));
            let o = ok(&mut doc);
            assert!((o.holes[0].start - v3(20.0, -20.0, z)).norm() < 1e-9, "{:?}", o.holes[0]);
            let mut check = Document::new();
            wedge(&mut check, degrees, 7.5);
            if countersink {
                check.with(Op::solid(Primitive::Cylinder { radius: 7.0.into(), height: 400.0.into() }, upright(z), Mode::Remove));
                check.with(Op::solid(Primitive::Cone { r0: 3.0.into(), r1: 7.0.into(), height: 4.0.into() }, upright(z - 4.0), Mode::Remove));
            } else {
                check.with(Op::solid(Primitive::Cylinder { radius: 6.0.into(), height: 400.0.into() }, upright(z - 4.0), Mode::Remove));
            }
            check.with(Op::solid(Primitive::Cylinder { radius: 3.0.into(), height: 400.0.into() }, upright(-10.0), Mode::Remove));
            let want = vol(&ok(&mut check));
            assert!((vol(&o) - want).abs() < 1e-7 * want, "{degrees} degrees, countersink {countersink}: {} against {want}", vol(&o));
            // The top's wall is named for the hole, and nothing's left of the face inside it, so its end, carried back
            // clear of the face, is nowhere on the part.
            let part = if countersink { "csink" } else { "cbore" };
            let n = names(&o, h);
            assert!(n.contains(&format!("f{}:{part}.{}", h.0, o.holes[0].number)), "{n:?}");
            assert!(!n.iter().any(|x| x.starts_with(&format!("f{}:top.", h.0))), "{degrees} degrees, countersink {countersink}: {n:?}");
        }
    }
}

#[test]
fn a_counterbore_leaves_a_wall_beside_it_alone() {
    // A plate 10 thick and a wall 20 high on it from x = 40. An M6 counterbore at x = 36 reaches 1.5 under the wall: it
    // takes what's under the wall, below the plate's top, and nothing of the wall above it.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let w = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::rect(40.0, 0.0, 20.0, 40.0))]));
    doc.with(Op::extrude(w, 0.0, 20.0, Mode::Add));
    let whole = 60.0 * 40.0 * 10.0 + 20.0 * 40.0 * 20.0;
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [[36, 20]], "size": "M6", "style": "counterbore", "end": "through_all"}}"#, e.0));
    let o = ok(&mut doc);
    near(vol(&o), whole - PI * (5.5f64.powi(2) * 6.4 + 3.3f64.powi(2) * 3.6), "a counterbore under a wall's foot");
    // Nothing's carried back over a flat face: the hole's top is level with it, and shows only under the wall, as the
    // ceiling of what the counterbore takes from under it.
    // The volume says the wall above the plate is whole.
    assert!(names(&o, h).contains(&format!("f{}:top.0", h.0)), "{:?}", names(&o, h));
    // The same against a wall with a draft, leaning back at 85 degrees to the plate: its foot is over the counterbore too,
    // and it's still left alone. A face at 80 degrees or less is one the hole goes in through, as a wedge's is.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let side = Placement::new([0.0.into(), 0.0.into(), 0.0.into()], [0.0.into(), (-1.0).into(), 0.0.into()], [1.0.into(), 0.0.into(), 0.0.into()]);
    let run = 20.0 / 85f64.to_radians().tan();
    let lean = vec![[40.0.into(), 10.0.into()], [60.0.into(), 10.0.into()], [60.0.into(), 30.0.into()], [(40.0 + run).into(), 30.0.into()]];
    let l = doc.with(Op::sketch(side, vec![Region::new(Shape::polygon(lean))]));
    doc.with(Op::extrude(l, 0.0, 40.0, Mode::Add));
    let whole = 60.0 * 40.0 * 10.0 + (20.0 + 20.0 - run) / 2.0 * 20.0 * 40.0;
    feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [[36, 20]], "size": "M6", "style": "counterbore", "end": "through_all"}}"#, e.0));
    near(vol(&ok(&mut doc)), whole - PI * (5.5f64.powi(2) * 6.4 + 3.3f64.powi(2) * 3.6), "a counterbore under a leaning wall's foot");
    // A round pocket 10 deep with a 5 degree draft, 9 across at its floor, and an M6 counterbore 11 across down its
    // middle: all round, the rays a little way out from the pocket's foot meet the drafted wall only just above the
    // floor, and are told from the floor by how steep the wall is. The counterbore undercuts the wall and leaves it be.
    let (mut doc, at, _) = block_with_points(&[(30.0, 20.0)]);
    let draft = 5f64.to_radians().tan();
    doc.add(Op::solid(Primitive::Cone { r0: 4.5.into(), r1: (4.5 + 12.0 * draft).into(), height: 12.0.into() }, Placement::at([30.0.into(), 20.0.into(), 10.0.into()]), Mode::Remove)).unwrap();
    let at = {
        let mut sketch = Sketch::new();
        sketch.point(30.0, 20.0);
        doc.delete(at).unwrap();
        doc.add(Op::ConstrainedSketch { plane: Placement::xy(40.0), sketch }).unwrap()
    };
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "size": "M6", "style": "counterbore", "end": "through_all"}}"#, at.0));
    let o = ok(&mut doc);
    let top = 4.5 + 10.0 * draft;
    let pocket = PI * 10.0 * (4.5 * 4.5 + 4.5 * top + top * top) / 3.0;
    near(vol(&o), BLOCK - pocket - PI * (5.5f64.powi(2) * 6.4 + 3.3f64.powi(2) * 3.6), "a counterbore under a drafted pocket's wall");
    assert!((o.holes[0].start.z - 10.0).abs() < 1e-9, "{:?}", o.holes[0]);
    assert!(names(&o, h).contains(&format!("f{}:top.1", h.0)), "{:?}", names(&o, h));
}

#[test]
fn a_holes_names_stay_with_its_point() {
    // Three points, every one taken: holes are numbered by their points' entity ids, so the middle one going leaves the
    // others' names alone, and a chamfer on the last hole's rim keeps to it.
    let mut doc = Document::new();
    doc.set_parameter("d", 6.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let mut sketch = Sketch::new();
    let ids: Vec<_> = [(10.0, 20.0), (30.0, 20.0), (50.0, 20.0)].iter().map(|(x, y)| sketch.point(*x, *y)).collect();
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch: sketch.clone() }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": "d", "end": "through_all"}}"#, at.0));
    let last = h.hole_face(ids[2].0, HoleFace::Bore);
    assert_eq!(last.name(), format!("f{}:bore.{}", h.0, ids[2].0));
    assert_eq!(FaceRef::parse(&last.name()).unwrap(), last);
    doc.add(Op::chamfer(EdgeSelect::Between(e.end_cap(), last), 1.0)).unwrap();
    // A 45 degree chamfer 1 deep round a hole r across takes a ring of triangles: pi * ((r + 1)^2 - r^2) / 2 * ... worked
    // out as the cone frustum it is, less the hole.
    let rim = |r: f64| PI * 1.0 * ((r + 1.0).powi(2) + (r + 1.0) * r + r * r) / 3.0 - PI * r * r;
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - 3.0 * PI * 9.0 * 20.0 - rim(3.0), "three holes and a chamfer");
    doc.set_parameter("d", 8.0).unwrap();
    near(vol(&ok(&mut doc)), BLOCK - 3.0 * PI * 16.0 * 20.0 - rim(4.0), "wider holes, and the chamfer still on the last");
    sketch.remove_entity(ids[1]).unwrap();
    doc.edit(at, Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch }).unwrap();
    let o = ok(&mut doc);
    near(vol(&o), BLOCK - 2.0 * PI * 16.0 * 20.0 - rim(4.0), "the middle hole gone, and the chamfer still on the last");
    let n = names(&o, h);
    assert_eq!(n, vec![format!("f{}:bore.{}", h.0, ids[0].0), format!("f{}:bore.{}", h.0, ids[2].0)]);
    // The sketch can't go, or move after the hole, while the hole uses it.
    assert!(doc.delete(at).unwrap_err().to_string().contains(&format!("f{} refer to it", h.0)));
    assert!(doc.reorder(at, 10).is_err());
}

#[test]
fn points_named_in_a_sketch_can_be_ends_and_centers() {
    // A circle's center and a line's end, and the same place twice, which makes one hole and says so.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let mut sketch = Sketch::new();
    let c = sketch.circle([15.0, 20.0], 5.0);
    let l = sketch.line([40.0, 10.0], [45.0, 30.0]);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch }).unwrap();
    let h = feature(
        &mut doc,
        &format!(r#"{{"type": "hole", "sketch": {}, "points": ["{}.center", "{}.end", "{}.center"], "diameter": 4, "end": "through_all"}}"#, at.0, c.0, l.0, c.0),
    );
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    near(vol(&o), BLOCK - 2.0 * PI * 4.0 * 20.0, "two holes");
    let step = o.steps.iter().find(|s| s.id == h).unwrap();
    assert_eq!(step.notes, vec!["holes 0 and 2 are at the same place, (15, 20, 20), so there's one hole there".to_string()]);
    assert_eq!(o.holes.iter().map(|x| x.number).collect::<Vec<_>>(), vec![0, 1]);
    assert!((o.holes[1].start - v3(45.0, 30.0, 20.0)).norm() < 1e-9);
}

#[test]
fn what_a_hole_cant_be_is_refused_saying_why() {
    let (mut doc, at, _) = block_with_points(&[(30.0, 20.0)]);
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, at.0));
    ok(&mut doc);
    let cases = [
        (r#""depth": 10"#, "a hole needs a diameter or a size"),
        (r#""size": "M7", "depth": 10"#, "isn't a size the hole tables know"),
        (r#""diameter": 6, "tapped": true, "depth": 10"#, "a tapped hole needs a size"),
        (r#""diameter": 6, "style": "counterbore", "depth": 10"#, "a counterbore needs cbore_diameter and cbore_depth"),
        (r#""diameter": 6, "style": "counterbore", "cbore_diameter": 5, "cbore_depth": 2, "depth": 10"#, "a counterbore has to be wider than its hole"),
        (r#""diameter": 6, "cbore_diameter": 9, "depth": 10"#, "are for a counterbored hole"),
        (r#""diameter": 6, "style": "countersink", "csink_diameter": 30, "depth": 10"#, "countersink goes 12, as deep as the hole or deeper"),
        (r#""size": "M2", "style": "countersink", "depth": 10"#, "the tables have no countersink for M2"),
        (r#""size": "M6", "tapped": {"depth": 12}, "depth": 10"#, "thread goes 12, deeper than the hole's 10"),
        (r#""diameter": 6, "depth": 0"#, "a hole's depth has to be at least"),
        (r#""diameter": 6, "depth": 10, "tip_angle": 200"#, "tip angle is an angle in degrees"),
    ];
    for (fields, want) in cases {
        doc.edit(h, op(&format!(r#"{{"type": "hole", "sketch": {}, {fields}}}"#, at.0))).unwrap();
        let why = failure(&mut doc);
        assert!(why.contains(want), "{fields}: {why}");
    }
    // A point off the part, and from the sketch a hole that stops short of the part. Each is a new feature, after
    // the sketch it uses.
    doc.delete(h).unwrap();
    let refused = |doc: &mut Document, text: String| -> String {
        let h = feature(doc, &text);
        let why = failure(doc);
        doc.delete(h).unwrap();
        why
    };
    let mut far = Sketch::new();
    far.point(100.0, 100.0);
    let off = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch: far }).unwrap();
    let why = refused(&mut doc, format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, off.0));
    assert!(why.contains("never meets the part going that way"), "{why}");
    let mut above = Sketch::new();
    above.point(30.0, 20.0);
    let high = doc.add(Op::ConstrainedSketch { plane: Placement::xy(40.0), sketch: above }).unwrap();
    let why = refused(&mut doc, format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10, "from_sketch": true}}"#, high.0));
    assert!(why.contains("doesn't reach the part from the sketch"), "{why}");
    // From a sketch half way down inside the part, a short hole would be a hollow inside it.
    let mut inside = Sketch::new();
    inside.point(30.0, 20.0);
    let mid = doc.add(Op::ConstrainedSketch { plane: Placement::xy(10.0), sketch: inside }).unwrap();
    let why = refused(&mut doc, format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 3, "from_sketch": true}}"#, mid.0));
    assert!(why.contains("would leave a hollow inside part"), "{why}");
    // A sketch of shapes, and one with no points.
    let s = doc.with(Op::sketch(Placement::xy(20.0), vec![Region::new(Shape::circle(5.0, 5.0, 1.0))]));
    let why = refused(&mut doc, format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, s.0));
    assert!(why.contains("constrained sketch"), "{why}");
    let empty = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch: Sketch::new() }).unwrap();
    let why = refused(&mut doc, format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "depth": 10}}"#, empty.0));
    assert!(why.contains("has no points for holes to go at"), "{why}");
    // Through the first wall, where half the hole is over a step and meets the wall below first: there's no depth that
    // stops between the walls all round. Parts that touch are one wall.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 10.0, Mode::NewBody));
    let half = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 20.0, 40.0))]));
    doc.with(Op::extrude(half, 15.0, 10.0, Mode::NewBody));
    let mut sketch = Sketch::new();
    sketch.point(19.0, 20.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(25.0), sketch }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "end": "up_to_next"}}"#, at.0));
    let why = failure(&mut doc);
    assert!(why.contains("can't stop between the first wall it goes through and what's behind it"), "{why}");
    doc.delete(h).unwrap();
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 10.0, Mode::NewBody));
    doc.with(Op::extrude(s, 10.0, 10.0, Mode::NewBody));
    let mut sketch = Sketch::new();
    sketch.point(20.0, 20.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(20.0), sketch }).unwrap();
    feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 6, "end": "up_to_next"}}"#, at.0));
    near(vol(&ok(&mut doc)), 2.0 * 16000.0 - PI * 9.0 * 20.0, "through two plates that touch");
    // JSON that doesn't fit.
    for (text, want) in [
        (r#"{"type": "hole", "diameter": 6, "depth": 1}"#, "from a \"sketch\" or a \"plane\""),
        (r#"{"type": "hole", "sketch": 2, "diameter": 6}"#, "\"depth\""),
        (r#"{"type": "hole", "sketch": 2, "diameter": 6, "end": "through_all", "depth": 3}"#, "is for a blind hole"),
        (r#"{"type": "hole", "sketch": 2, "diameter": 6, "depth": 3, "style": "spotface"}"#, "unknown hole style"),
        (r#"{"type": "hole", "sketch": 2, "diameter": 6, "depth": 3, "tapped": {"lead": 2}}"#, "lead"),
    ] {
        let why = read_op(&json::parse(text).unwrap(), &[]).unwrap_err().to_string();
        assert!(why.contains(want), "{text}: {why}");
    }
}

#[test]
fn a_hole_saves_and_loads() {
    let (mut doc, at, ids) = block_with_points(&[(20.0, 20.0)]);
    let texts = [
        format!(r#"{{"type": "hole", "sketch": {}, "style": "counterbore", "size": "M8", "fit": "loose", "tapped": {{"pitch": 1, "depth": 9}}, "cbore_diameter": 16, "cbore_depth": 2, "tip_angle": 90, "end": "blind", "depth": 12, "from_sketch": true}}"#, at.0),
        format!(r#"{{"type": "hole", "sketch": {}, "points": ["{}"], "style": "countersink", "diameter": 5, "csink_diameter": 10, "csink_angle": 82, "end": "up_to_next", "opposite": true}}"#, at.0, ids[0]),
        r#"{"type": "hole", "plane": "top", "points": [[1, "2 * 3"]], "style": "simple", "size": "M3", "tapped": true, "end": "through_all"}"#.to_string(),
    ];
    for text in texts {
        let h = feature(&mut doc, &text);
        let back = Document::from_json(&doc.to_json()).unwrap();
        assert_eq!(back.feature(h).unwrap().op, doc.feature(h).unwrap().op, "{text}");
        doc.delete(h).unwrap();
    }
}

#[test]
fn a_pattern_or_a_mirror_repeats_holes() {
    // A disc 100 across and 10 thick, one M8 counterbored hole 35 out, and a pattern making six round. A mirror of the
    // pattern across the YZ plane adds nothing new, as six round are already symmetric.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 50.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [[35, 0]], "size": "M8", "style": "counterbore", "cbore_depth": 4, "end": "through_all"}}"#, e.0));
    let one = PI * (7.5f64.powi(2) * 4.0 + 4.5f64.powi(2) * 6.0);
    let disc = PI * 2500.0 * 10.0;
    near(vol(&ok(&mut doc)), disc - one, "one hole");
    let p = feature(&mut doc, &format!(r#"{{"type": "pattern", "of": [{}], "count": 6}}"#, h.0));
    let o = ok(&mut doc);
    near(vol(&o), disc - 6.0 * one, "six holes round");
    // Copies keep the hole's names.
    let bores = o.bodies[0].body.faces.iter().filter(|f| FaceRef::from_tag(f.tag) == h.hole_face(0, HoleFace::Bore)).count();
    assert_eq!(bores, 12, "six bores, each in two halves");
    feature(&mut doc, &format!(r#"{{"type": "mirror", "of": [{}, {}], "plane": "right"}}"#, h.0, p.0));
    near(vol(&ok(&mut doc)), disc - 6.0 * one, "mirrored onto themselves");
}

#[test]
fn a_hole_goes_into_a_free_form_face() {
    // A loft from a square 40 across to a circle 15 round, 30 up, and a hole straight down its middle from a sketch above
    // it: it starts on the circle's flat top, so it's as a hole in a flat face is.
    let mut doc = Document::new();
    let base = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-20.0, -20.0, 40.0, 40.0))]));
    let top = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::circle(0.0, 0.0, 15.0))]));
    doc.with(Op::Loft { sections: vec![base.into(), top.into()], mode: Mode::Add, thin: None, shape: LoftShape::default() });
    let before = vol(&ok(&mut doc));
    let mut sketch = Sketch::new();
    sketch.point(0.0, 0.0);
    let at = doc.add(Op::ConstrainedSketch { plane: Placement::xy(40.0), sketch }).unwrap();
    let h = feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 10, "depth": 20}}"#, at.0));
    near(vol(&ok(&mut doc)), before - PI * 25.0 * 20.0 - cone(5.0, point_height(5.0, 118.0)), "a hole down a loft's middle");
    // From the side, into the lofted wall, which leans back: the hole starts where its axis meets the free-form
    // face, and its counterbore opens cleanly round it.
    let side = |x: f64| Placement::new([x.into(), 0.0.into(), 0.0.into()], [1.0.into(), 0.0.into(), 0.0.into()], [0.0.into(), 1.0.into(), 0.0.into()]);
    let mut points = Sketch::new();
    points.point(0.0, 15.0);
    let off = doc.add(Op::ConstrainedSketch { plane: side(40.0), sketch: points }).unwrap();
    doc.delete(h).unwrap();
    feature(&mut doc, &format!(r#"{{"type": "hole", "sketch": {}, "diameter": 4, "style": "counterbore", "cbore_diameter": 8, "cbore_depth": 3, "end": "through_all"}}"#, off.0));
    let o = ok(&mut doc);
    let x = o.holes[0].start.x;
    assert!(x > 15.0 && x < 20.0 && (o.holes[0].start - v3(x, 0.0, 15.0)).norm() < 1e-9, "{:?}", o.holes[0]);
    // The same counterbore and hole as two cylinders cut by extrudes along -x, from where it starts.
    let mut check = Document::new();
    let base = check.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-20.0, -20.0, 40.0, 40.0))]));
    let top = check.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::circle(0.0, 0.0, 15.0))]));
    check.with(Op::Loft { sections: vec![base.into(), top.into()], mode: Mode::Add, thin: None, shape: LoftShape::default() });
    let c = check.with(Op::sketch(side(x - 3.0), vec![Region::new(Shape::circle(0.0, 15.0, 4.0))]));
    check.with(Op::extrude(c, 0.0, 30.0, Mode::Remove));
    let d = check.with(Op::sketch(side(-30.0), vec![Region::new(Shape::circle(0.0, 15.0, 2.0))]));
    check.with(Op::extrude(d, 0.0, 70.0, Mode::Remove));
    let want = vol(&ok(&mut check));
    assert!((vol(&o) - want).abs() < 1e-6 * want, "{} against {want}", vol(&o));
}
