//! Splitting parts: along a plane, which it follows; along the whole surface a face is on; by another part; and a sheet.
//! Splitting along sheets is in split_sheets.rs.
//! The piece in front of the plane, or inside, keeps the part's name, and each piece is a part of its own.

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

/// Each body's name and volume, or area for a sheet.
fn sizes(o: &Outcome) -> Vec<(String, f64)> {
    o.bodies
        .iter()
        .map(|b| {
            let m = mass_properties(&b.body);
            (b.id.text(), if b.body.sheet { m.area } else { m.volume })
        })
        .collect()
}

fn size_of(o: &Outcome, name: &str) -> f64 {
    sizes(o).into_iter().find(|(n, _)| n == name).unwrap_or_else(|| panic!("no body {name}: {:?}", sizes(o))).1
}

/// A block `w` by 6 by 4 from the origin, with a hole of radius 1 through it at (3, 3).
fn block() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 6.0)).hole(Shape::circle(3.0, 3.0, 1.0))]));
    doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    doc
}

#[test]
fn a_plane_cuts_a_part_in_two_and_follows_it() {
    let mut doc = block();
    doc.set_parameter("c", 6.0).unwrap();
    feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "right", "distance": "c"}"#);
    let split = feature(&mut doc, r#"{"type": "split", "plane": 3}"#);
    for (w, c) in [(10.0, 6.0), (12.0, 7.5)] {
        doc.set_parameter("w", w).unwrap();
        doc.set_parameter("c", c).unwrap();
        let o = ok(&mut doc);
        assert_eq!(o.bodies.len(), 2, "{:?}", sizes(&o));
        // In front of the plane, facing +X, keeps the name, and behind it is a new part named for the split.
        near(size_of(&o, "f2"), (w - c) * 24.0, "in front");
        near(size_of(&o, "f4"), c * 24.0 - PI * 4.0, "behind, with the hole");
        // Both cut faces are the split's start.
        for b in &o.bodies {
            assert!(b.body.faces.iter().any(|f| f.tag.feature == split.0 && f.tag.slot == coreshape::build::SLOT_START_CAP), "{}", b.id.text());
        }
    }
    // Later features find the cut faces and the new part by name.
    feature(&mut doc, r#"{"type": "fillet", "edges": {"between": ["f4:start", "f2:end"]}, "radius": 0.5}"#);
    let moved = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "translate", "by": [0, 0, 10]}}"#);
    doc.set_scope(moved, vec![BodyId::parse("f4").unwrap()]).unwrap();
    let o = ok(&mut doc);
    let lifted = o.bodies.iter().find(|b| b.id.text() == "f4").unwrap();
    assert!(lifted.body.aabb().min.z > 9.0);
    assert_eq!(read_op(&op_value(&doc.feature(split).unwrap().op), &[]).unwrap(), doc.feature(split).unwrap().op);
}

#[test]
fn a_plane_through_a_hole_or_a_flat_face_of_another_part() {
    // Through the middle of the hole, on Front moved 3 back: two halves, each with half the hole.
    let mut doc = block();
    feature(&mut doc, r#"{"type": "split", "plane": {"point": [0, 3, 0], "normal": [0, -1, 0]}}"#);
    let o = ok(&mut doc);
    near(size_of(&o, "f2"), 120.0 - PI * 2.0, "the near half");
    near(size_of(&o, "f3"), 120.0 - PI * 2.0, "the far half");
    // Along the top of a slab beside it, facing up: what's above keeps the name.
    let mut doc = block();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(20.0, 0.0, 5.0, 5.0))]));
    doc.with(Op::extrude(s, 0.0, 1.5, Mode::NewBody));
    let split = feature(&mut doc, r#"{"type": "split", "plane": "f4:end"}"#);
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    near(size_of(&o, "f2"), 2.5 * (60.0 - PI), "above the slab's top");
    near(size_of(&o, "f5"), 1.5 * (60.0 - PI), "below it");
    near(size_of(&o, "f4"), 37.5, "the slab, which isn't split");
}

#[test]
fn a_face_cuts_along_its_whole_surface() {
    // A shaft, a collar of radius 10 and 10 long under a stem of radius 5 up to 30, split along the stem's surface.
    let mut doc = Document::new();
    let collar = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 10.0))]));
    doc.with(Op::extrude(collar, 0.0, 10.0, Mode::Add));
    let stem = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let up = doc.with(Op::extrude(stem, 0.0, 20.0, Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.0"}}"#, up.0));
    let o = doc.rebuild(None);
    match o.failed {
        None => {
            near(size_of(&o, "f2"), PI * 25.0 * 30.0, "the core, inside the stem's surface");
            near(size_of(&o, "f5"), PI * 75.0 * 10.0, "the ring round it");
        }
        Some(i) => panic!("{:?}", o.steps[i]),
    }

    // A block cut by a ball's surface, the ball a part of its own centred on the block's corner.
    let mut doc = block();
    doc.with(Op::solid(Primitive::Sphere { radius: 2.0.into() }, Placement::xy(0.0), Mode::NewBody));
    let split = feature(&mut doc, r#"{"type": "split", "face": "f3:side.0"}"#);
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    let eighth = 4.0 / 3.0 * PI * 8.0 / 8.0;
    near(size_of(&o, "f2"), eighth, "the corner inside the ball");
    near(size_of(&o, "f4"), 240.0 - PI * 4.0 - eighth, "the rest");

    // A face on a turned or extruded curve, or a free-form one, cuts along a sheet: see split_sheets.rs.
}

#[test]
fn a_part_cuts_others_into_inside_and_outside() {
    // A rod of radius 0.8 along x through the block, at y 5 and z 2, clear of the hole.
    let rod = |doc: &mut Document| {
        let s = doc.with(Op::sketch(Placement::new(point3(-5.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![Region::new(Shape::circle(5.0, 2.0, 0.8))]));
        doc.with(Op::extrude(s, 0.0, 20.0, Mode::NewBody));
    };
    for keep in [true, false] {
        let mut doc = block();
        rod(&mut doc);
        let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f4", "keep_tool": {keep}}}"#));
        doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
        let o = ok(&mut doc);
        // The rod's 10 across the block is inside.
        near(size_of(&o, "f2"), PI * 6.4, "inside the rod");
        near(size_of(&o, "f5"), 240.0 - PI * 4.0 - PI * 6.4, "outside it");
        assert_eq!(o.bodies.iter().any(|b| b.id.text() == "f4"), keep, "the rod");
    }
    // The rod moves, and the split follows it.
    let mut doc = block();
    doc.set_parameter("ry", 5.0).unwrap();
    let s = doc.with(Op::sketch(Placement::new(point3(-5.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![Region::new(Shape::circle("ry", 2.0, 0.8))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::NewBody));
    let split = feature(&mut doc, r#"{"type": "split", "tool": "f4"}"#);
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    for ry in [5.0, 5.1] {
        doc.set_parameter("ry", ry).unwrap();
        let o = ok(&mut doc);
        let inside = o.bodies.iter().find(|b| b.id.text() == "f2").unwrap().body.aabb();
        assert!(((inside.min.y + inside.max.y) / 2.0 - ry).abs() < 1e-6, "the piece inside the rod is where it is: {inside:?}");
    }
    // A flat sheet right across it, half way up, splits it too: the half above, the way the sheet faces, keeps the name.
    let mut doc = block();
    let s = doc.with(Op::sketch(Placement::xy(2.0), vec![Region::new(Shape::rect(-1.0, -1.0, 20.0, 20.0))]));
    doc.with(Op::Fill { sketch: s });
    let split = feature(&mut doc, r#"{"type": "split", "tool": "f4"}"#);
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    for (name, low) in [("f2", 2.0), ("f5", 0.0)] {
        near(size_of(&o, name), (240.0 - PI * 4.0) / 2.0, "half the block");
        let b = o.bodies.iter().find(|b| b.id.text() == name).unwrap();
        let lowest = b.body.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);
        assert!((lowest - low).abs() < 1e-9, "{name} starts at {lowest}");
    }
}

#[test]
fn a_sheet_splits_too() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 6.0))]));
    doc.with(Op::Fill { sketch: s });
    feature(&mut doc, r#"{"type": "split", "plane": {"point": [4, 0, 0], "normal": [1, 0, 0]}}"#);
    let o = ok(&mut doc);
    near(size_of(&o, "f2"), 36.0, "in front");
    near(size_of(&o, "f3"), 24.0, "behind");
}

#[test]
fn what_a_split_refuses() {
    // A plane beyond the part, which it names.
    let mut doc = block();
    let split = feature(&mut doc, r#"{"type": "split", "plane": {"point": [50, 0, 0], "normal": [1, 0, 0]}}"#);
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("doesn't cut through body f2"), "{why}");
    // Or naming none, along its own top face.
    let mut doc = block();
    feature(&mut doc, r#"{"type": "split", "plane": "f2:end"}"#);
    let why = failure(&mut doc);
    assert!(why.contains("doesn't cut through any part"), "{why}");
    for text in [r#"{"type": "split"}"#, r#"{"type": "split", "plane": "top", "tool": "f2"}"#] {
        let why = read_op(&json::parse(text).unwrap(), &[]).unwrap_err().to_string();
        assert!(why.contains("takes one of"), "{why}");
    }
    for text in [r#"{"type": "split", "plane": "top"}"#, r#"{"type": "split", "face": "f2:side.0"}"#, r#"{"type": "split", "tool": "f2.1", "keep_tool": false}"#] {
        let v = json::parse(text).unwrap();
        let read = read_op(&v, &[]).unwrap();
        assert_eq!(op_value(&read).to_text(), v.to_text(), "{text}");
    }
}
