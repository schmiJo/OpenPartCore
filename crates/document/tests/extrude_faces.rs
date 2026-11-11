//! Extruding a part's flat faces, as Onshape's extrude takes faces as well as sketch regions. Each face goes along its own
//! outward normal, holes and all, and every volume is held against a closed formula.

use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn near(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol, "{what}: {a} isn't {b}");
}

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

fn total(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| vol(&b.body)).sum()
}

fn key(text: &str) -> FaceKey {
    FaceKey::parse(text).unwrap()
}

fn keys(list: &[&str]) -> Vec<FaceKey> {
    list.iter().map(|t| key(t)).collect()
}

/// How many faces of the outcome a key picks.
fn count(o: &Outcome, text: &str) -> usize {
    find_faces(o.body.as_ref().unwrap(), &key(text)).len()
}

fn block(doc: &mut Document, dx: impl Into<Expr>, dy: impl Into<Expr>, dz: impl Into<Expr>, at: [f64; 3], mode: Mode) -> FeatureId {
    doc.add(Op::solid(Primitive::Box { dx: dx.into(), dy: dy.into(), dz: dz.into() }, Placement::at(point3(at[0], at[1], at[2])), mode)).unwrap()
}

fn extrude_faces(list: &[&str], length: f64, mode: Mode, end: ExtrudeEnd, opposite: bool) -> Op {
    Op::Extrude { sketch: None, start: 0.0.into(), length: length.into(), mode, regions: Vec::new(), faces: keys(list), end, opposite }
}

/// The span of a body's mesh along a direction.
fn span(b: &Body, along: Vec3) -> (f64, f64) {
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    mesh.positions.iter().map(|p| p.dot(along)).fold((f64::MAX, f64::MIN), |(lo, hi), h| (lo.min(h), hi.max(h)))
}

#[test]
fn a_top_face_grows_up_and_its_new_faces_are_named_for_it() {
    let mut doc = Document::new();
    let b = block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    // As a part of its own, so that its walls stay faces of their own.
    let e = doc.add(Op::extrude_faces(keys(&["f1:end"]), 5.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(total(&o), 10.0 * 6.0 * 9.0, 1e-9, "volume");
    let body = o.body.as_ref().unwrap();
    // The far face, keyed by the face it came from.
    let far = find_faces(body, &FaceKey::plain(e.end_cap()));
    assert_eq!(far.len(), 1);
    assert_eq!(face_keys(body)[far[0]].text(), "f2:end[f1:end]");
    // A wall for each edge of the face, each keyed by the face across that edge.
    assert_eq!(count(&o, "f2:wall.0"), 4);
    for side in 0..4 {
        let k = format!("f2:wall.0[{}]", b.side(side).name());
        assert_eq!(count(&o, &k), 1, "{k}");
    }
    let right = find_faces(body, &key("f2:wall.0[f1:side.1]"))[0];
    let box_ = body.face_aabb(right);
    near(box_.min.x, 10.0, 1e-9, "the wall over the x = 10 side");
    near(box_.max.x, 10.0, 1e-9, "the wall over the x = 10 side");
    near(box_.min.z, 4.0, 1e-9, "it starts at the top");
    // Every key it gives picks its face, and only that face.
    for (f, k) in face_keys(body).iter().enumerate() {
        assert_eq!(find_faces(body, &FaceKey::parse(&k.text()).unwrap()), vec![f], "{}", k.text());
    }
    assert_eq!(FeatureId(2).wall(0).name(), "f2:wall.0");
    assert_eq!(FaceRef::parse("f2:wall.7").unwrap(), FeatureId(2).wall(7));
}

#[test]
fn added_walls_flush_with_the_part_become_part_of_its_faces() {
    // As with a sketch's region, a side that carries straight on is one face, which keeps the name it had.
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:end"]), 5.0, Mode::Add)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    near(total(&o), 10.0 * 6.0 * 9.0, 1e-9, "volume");
    assert_eq!(count(&o, "f2:wall.0"), 0);
    assert_eq!(count(&o, "f2:end[f1:end]"), 1);
    let body = o.body.as_ref().unwrap();
    let side = find_faces(body, &key("f1:side.1"));
    assert_eq!(side.len(), 1);
    let reach = body.face_aabb(side[0]);
    near(reach.max.z - reach.min.z, 9.0, 1e-9, "the side face now goes all the way up");
}

#[test]
fn each_face_goes_along_its_own_normal() {
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    // The top up and the bottom down, and then the x = 10 side out along x.
    doc.add(Op::extrude_faces(keys(&["f1:end", "f1:start"]), 5.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 10.0 * 6.0 * 14.0, 1e-9, "volume");
    let (lo, hi) = span(o.body.as_ref().unwrap(), Vec3::Z);
    near(lo, -5.0, 1e-9, "bottom");
    near(hi, 9.0, 1e-9, "top");
    // Named for which entry picked them.
    assert_eq!(count(&o, "f2:wall.0"), 4);
    assert_eq!(count(&o, "f2:wall.1"), 4);

    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:side.1"]), 3.0, Mode::Add)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 10.0 * 6.0 * 4.0 + 6.0 * 4.0 * 3.0, 1e-9, "volume");
    near(span(o.body.as_ref().unwrap(), Vec3::X).1, 13.0, 1e-9, "x reach");
}

#[test]
fn holes_go_along_too() {
    let mut doc = Document::new();
    let s = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::Rect { x: 0.0.into(), y: 0.0.into(), w: 20.0.into(), h: 20.0.into() }).hole(Shape::Circle { cx: 10.0.into(), cy: 10.0.into(), r: 3.0.into() })])).unwrap();
    doc.add(Op::extrude(s, 0.0, 2.0, Mode::Add)).unwrap();
    doc.add(Op::extrude_faces(keys(&["f2:end"]), 3.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), (400.0 - 9.0 * PI) * 5.0, 1e-6, "volume");
    // The hole's wall is two halves, as a whole circle always is, across from the hole's own halves.
    assert_eq!(count(&o, "f3:wall.0[f2:hole.0.side.0]"), 2);
    assert_eq!(count(&o, "f3:wall.0[f2:side.0]"), 1);
}

#[test]
fn opposite_and_remove_cut_into_the_part() {
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(extrude_faces(&["f1:end"], 1.5, Mode::Remove, ExtrudeEnd::Blind, true)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 10.0 * 6.0 * 2.5, 1e-9, "volume");
}

#[test]
fn ends_work_as_they_do_for_a_sketch() {
    // Two blocks 10 apart, and the near side of the first taken across to the second.
    let ends = [
        ExtrudeEnd::UpToFace(key("f2:side.3")),
        ExtrudeEnd::UpToNext,
        ExtrudeEnd::UpToPart(BodyId { feature: FeatureId(2), index: 0 }),
        ExtrudeEnd::UpToVertex(VertexRef::Point(point3(20.0, 3.0, 2.0))),
    ];
    for end in ends {
        let mut doc = Document::new();
        block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::NewBody);
        block(&mut doc, 10.0, 6.0, 4.0, [20.0, 0.0, 0.0], Mode::NewBody);
        doc.add(extrude_faces(&["f1:side.1"], 0.0, Mode::Add, end.clone(), false)).unwrap();
        let o = ok(&mut doc);
        near(total(&o), 240.0 * 2.0 + 6.0 * 4.0 * 10.0, 1e-9, &format!("{end:?}"));
        assert_eq!(o.bodies.len(), 1, "{end:?}: it joins them");
    }
}

#[test]
fn faces_and_regions_together_keep_their_names_apart() {
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    let s = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::Circle { cx: 30.0.into(), cy: 0.0.into(), r: 2.0.into() })])).unwrap();
    doc.add(Op::Extrude { sketch: Some(s), start: 0.0.into(), length: 3.0.into(), mode: Mode::NewBody, regions: Vec::new(), faces: keys(&["f1:end"]), end: ExtrudeEnd::Blind, opposite: false }).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 240.0 + 60.0 * 3.0 + PI * 4.0 * 3.0, 1e-6, "volume");
    assert_eq!(o.bodies.len(), 3);
    assert_eq!(count(&o, "f3:side.0"), 2);
    assert_eq!(count(&o, "f3:wall.0"), 4);
    assert_eq!(count(&o, "f3:end"), 2);
    assert_eq!(count(&o, "f3:end[f1:end]"), 1);
}

#[test]
fn a_name_that_picks_several_faces_extrudes_them_all() {
    let mut doc = Document::new();
    block(&mut doc, 30.0, 10.0, 2.0, [0.0; 3], Mode::Add);
    let boss = doc.add(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 3.0.into() }, Placement::at(point3(5.0, 5.0, 2.0)), Mode::Add)).unwrap();
    doc.add(Op::Pattern { of: vec![boss], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 10.0.into() } }).unwrap();
    // Twice, which takes each face once.
    doc.add(Op::extrude_faces(keys(&["f2:end", "f2:end@f3.1"]), 2.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 600.0 + 3.0 * PI * 5.0, 1e-6, "volume");
    assert_eq!(o.bodies.len(), 4);
    assert_eq!(count(&o, "f4:wall.0"), 6);
    assert_eq!(count(&o, "f4:wall.1"), 0);
}

#[test]
fn it_follows_the_part_when_the_part_changes() {
    let mut doc = Document::new();
    doc.set_parameter("h", 4.0).unwrap();
    block(&mut doc, 10.0, 6.0, "h", [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:end"]), 5.0, Mode::Add)).unwrap();
    near(total(&ok(&mut doc)), 60.0 * 9.0, 1e-9, "at first");
    doc.set_parameter("h", 7.0).unwrap();
    near(total(&ok(&mut doc)), 60.0 * 12.0, 1e-9, "after the change");
}

#[test]
fn as_a_surface_it_is_the_walls() {
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:end"]), 5.0, Mode::Surface)).unwrap();
    let o = ok(&mut doc);
    let sheets: Vec<&NamedBody> = o.bodies.iter().filter(|b| b.body.sheet).collect();
    assert_eq!(sheets.len(), 1);
    near(mass_properties(&sheets[0].body).area, 32.0 * 5.0, 1e-9, "area");
    assert!(sheets[0].body.faces.iter().all(|f| FaceRef::from_tag(f.tag).name() == "f2:wall.0"));
}

#[test]
fn what_cant_be_extruded_says_which_face() {
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 3.0.into() }, Placement::xy(0.0), Mode::Add)).unwrap();
    doc.add(Op::extrude_faces(keys(&["f1:side.0"]), 2.0, Mode::Add)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("f1:side.0") && why.contains("isn't flat"), "{why}");

    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:side.7"]), 2.0, Mode::Add)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("f1:side.7") && why.contains("no longer exists"), "{why}");
}

#[test]
fn json_takes_faces_with_or_without_a_sketch() {
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]);
    let op = read(r#"{"type": "extrude", "faces": ["f1:end", "f2:side.1[f1:end|f1:side.0]"], "length": 5}"#).unwrap();
    let Op::Extrude { sketch, faces, .. } = &op else { panic!() };
    assert_eq!(*sketch, None);
    assert_eq!(faces.len(), 2);
    let text = op_value(&op).to_text();
    assert!(text.contains("\"faces\"") && !text.contains("\"sketch\""), "{text}");
    assert_eq!(read(&text).unwrap(), op);

    let op = read(r#"{"type": "extrude", "sketch": 3, "faces": ["f1:end"], "length": 5}"#).unwrap();
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);

    let why = read(r#"{"type": "extrude", "length": 5}"#).unwrap_err().to_string();
    assert!(why.contains("\"faces\""), "{why}");
}

#[test]
fn a_document_with_one_saves_and_loads() {
    let mut doc = Document::new();
    block(&mut doc, 10.0, 6.0, 4.0, [0.0; 3], Mode::Add);
    doc.add(Op::extrude_faces(keys(&["f1:end"]), 5.0, Mode::Add)).unwrap();
    let mut back = Document::from_json(&json::parse(&doc.to_json().to_text()).unwrap()).unwrap();
    near(total(&ok(&mut back)), 540.0, 1e-9, "volume");
    // And a face that's gone stops it from being deleted.
    assert!(back.delete(FeatureId(1)).is_err());
}

/// A part's top face, extruded as a new part, against the area its first extrude says it has.
fn top_again(region: Region) -> (f64, f64) {
    let mut doc = Document::new();
    let s = doc.add(Op::sketch(Placement::new(point3(1.0, 2.0, 3.0), point3(0.2, -0.3, 1.0), point3(1.0, 0.0, 0.0)), vec![region])).unwrap();
    doc.add(Op::extrude(s, 0.0, 2.0, Mode::Add)).unwrap();
    doc.add(Op::extrude_faces(keys(&["f2:end"]), 3.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    (vol(&o.bodies[0].body) / 2.0, vol(&o.bodies[1].body) / 3.0)
}

#[test]
fn ellipses_and_splines_go_exactly() {
    let (area, again) = top_again(Region::new(Shape::Ellipse { cx: 1.0.into(), cy: 2.0.into(), rx: 6.0.into(), ry: 2.5.into(), degrees: 30.0.into() }));
    near(area, PI * 15.0, 1e-9, "the ellipse");
    near(again, PI * 15.0, 1e-9, "the ellipse again");
    let path = Shape::Path {
        start: point(0.0, 0.0),
        moves: vec![Move::Line { to: point(10.0, 0.0) }, Move::Spline { through: vec![point(12.0, 5.0), point(8.0, 10.0)] }, Move::Line { to: point(0.0, 10.0) }],
    };
    let (area, again) = top_again(Region::new(path));
    assert!(area > 90.0, "{area}");
    near(again, area, 1e-9 * area, "the spline");
}

#[test]
fn a_face_bounded_by_a_sampled_curve_is_refused_saying_which() {
    // A loft from a circle to a square has free-form sides, and a tilted cut through them leaves a flat face whose edge is
    // where a plane meets a free-form surface.
    let mut doc = Document::new();
    let a = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::Circle { cx: 0.0.into(), cy: 0.0.into(), r: 10.0.into() })])).unwrap();
    let b = doc.add(Op::sketch(Placement::xy(20.0), vec![Region::new(Shape::Rect { x: (-6.0).into(), y: (-6.0).into(), w: 12.0.into(), h: 12.0.into() })])).unwrap();
    doc.add(Op::Loft { sections: vec![a, b], mode: Mode::Add }).unwrap();
    let cut = doc.add(Op::sketch(Placement::new(point3(0.0, 0.0, 10.0), point3(0.3, 0.0, 1.0), point3(1.0, 0.0, -0.3)), vec![Region::new(Shape::Rect { x: (-30.0).into(), y: (-30.0).into(), w: 60.0.into(), h: 60.0.into() })])).unwrap();
    doc.add(Op::extrude(cut, 0.0, 30.0, Mode::Remove)).unwrap();
    let o = ok(&mut doc);
    let body = o.body.as_ref().unwrap();
    let flat = find_faces(body, &key("f5:start"));
    assert_eq!(flat.len(), 1, "{:?}", face_keys(body).iter().map(|k| k.text()).collect::<Vec<_>>());
    let sampled = body.faces[flat[0]].loops.iter().flatten().any(|c| matches!(body.edges[c.edge].curve, Curve::Sampled(_)));
    doc.add(Op::extrude_faces(keys(&["f5:start"]), 2.0, Mode::NewBody)).unwrap();
    if sampled {
        let why = failure(&mut doc);
        assert!(why.contains("f5:start") && why.contains("samples"), "{why}");
    } else {
        panic!("the cut face's edges all have an exact form, so this test no longer tests the refusal");
    }
}
