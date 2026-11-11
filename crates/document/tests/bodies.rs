//! A Part Studio with several parts, as Onshape has them. A body is one connected solid. A feature acts on all of them
//! unless it lists some, which Onshape calls its merge scope, and leaves alone any it doesn't reach. A document with one
//! body has to work exactly as it did.

use coreshape::export::step::{write_step, write_step_bodies};
use coreshape::import::step::read_step;
use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;
use std::sync::Arc;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn rebuilt(o: &Outcome) -> Vec<String> {
    o.steps.iter().filter(|s| s.state == StepState::Rebuilt).map(|s| s.name.clone()).collect()
}

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    o
}

fn block(dx: f64, dy: f64, dz: f64, x: f64) -> Op {
    Op::solid(Primitive::Box { dx: dx.into(), dy: dy.into(), dz: dz.into() }, Placement::at(point3(x, 0.0, 0.0)), Mode::NewBody)
}

fn named(doc: &mut Document, name: &str, op: Op) -> FeatureId {
    doc.insert(op, Some(name.to_string()), usize::MAX).unwrap()
}

/// A plate with a hole, and a bolt through it exactly the hole's size: parts that touch all around.
fn plate_and_bolt() -> (Body, Body) {
    let plate = box_solid(&Frame::WORLD, 40.0, 40.0, 10.0, 1).unwrap();
    let hole = cylinder(&Frame::translation(v3(20.0, 20.0, -1.0)), 5.0, 12.0, 2).unwrap();
    let plate = subtract(&plate, &hole).unwrap();
    let bolt = cylinder(&Frame::translation(v3(20.0, 20.0, -5.0)), 5.0, 20.0, 3).unwrap();
    (plate, bolt)
}

#[test]
fn parts_that_touch_all_around_stay_apart() {
    let (plate, bolt) = plate_and_bolt();
    let text = write_step_bodies(&[&plate, &bolt], "joint");
    let mut doc = Document::new();
    let f = doc.with(Op::ImportStep { text: text.clone(), mode: Mode::NewBody });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    // Joined, they'd be one shell.
    assert_eq!(union(&plate, &bolt).unwrap().shells.len(), 1);
    assert_eq!(o.bodies[0].id, BodyId { feature: f, index: 0 });
    assert_eq!(o.bodies[1].id, BodyId { feature: f, index: 1 });
    assert!((vol(&o.bodies[0].body) - vol(&plate)).abs() < 1e-6);
    assert!((vol(&o.bodies[1].body) - vol(&bolt)).abs() < 1e-6);
    // Each is whole, and valid on its own.
    for b in &o.bodies {
        validate(&b.body).unwrap();
    }
}

#[test]
fn merging_a_files_parts_says_how_to_keep_them_apart_when_it_fails() {
    // A cylinder just touching the inside of a sphere, where they meet in a figure of eight, which a Boolean refuses.
    let a = sphere(&Frame::WORLD, 30.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(12.0, 0.0, -40.0)), 18.0, 80.0, 2).unwrap();
    let text = write_step_bodies(&[&a, &b], "too close");
    let mut doc = Document::new();
    let f = doc.with(Op::ImportStep { text: text.clone(), mode: Mode::Add });
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(0), "this merged, so it doesn't test the message");
    let StepState::Failed(why) = &o.steps[0].state else { panic!() };
    assert!(why.contains("2 solids") && why.contains("new_body"), "{why}");
    // And that does what it says.
    doc.edit(f, Op::ImportStep { text, mode: Mode::NewBody }).unwrap();
    assert_eq!(ok(&mut doc).bodies.len(), 2);
}

#[test]
fn the_whole_body_of_an_outcome_holds_them_all_and_says_whose_each_face_is() {
    let mut doc = Document::new();
    let (a, b, c) = (doc.with(block(10.0, 10.0, 10.0, 0.0)), doc.with(block(20.0, 10.0, 10.0, 5.0)), doc.with(block(5.0, 5.0, 5.0, 100.0)));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|b| b.id.feature).collect::<Vec<_>>(), vec![a, b, c]);
    let all = o.body.as_ref().unwrap();
    assert_eq!(all.faces.len(), 18);
    assert_eq!(all.shells.len(), 3);
    validate(all).unwrap();
    // The overlap between the first two counts twice, as each is whole.
    assert!((mass_properties(all).volume - (1000.0 + 2000.0 + 125.0)).abs() < 1e-9);
    let mesh = tessellate(all, &TessParams::for_size(all.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    for f in 0..18 {
        let id = o.body_of_face(f).unwrap().id;
        assert_eq!(id.feature, [a, b, c][f / 6], "face {f}");
        assert_eq!(all.faces[f].tag.feature, id.feature.0);
    }
    assert!(o.body_of_face(18).is_none());
    let e = all.edges.len() - 1;
    assert_eq!(o.body_of_edge(e).unwrap().id.feature, c);
    assert_eq!(o.body_of_edge(0).unwrap().id.feature, a);
    assert!(o.body_of_edge(all.edges.len()).is_none());
    assert_eq!(o.find_body(BodyId { feature: b, index: 0 }).unwrap().first_face, 6);
}

#[test]
fn a_document_with_one_body_is_as_it_was() {
    let mut doc = Document::new();
    let first = doc.with(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 10.0.into(), dz: 10.0.into() }, Placement::xy(0.0), Mode::Add));
    doc.with(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 20.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert_eq!(o.bodies[0].id, BodyId { feature: first, index: 0 });
    // The very same body, not a copy.
    assert!(Arc::ptr_eq(o.body.as_ref().unwrap(), &o.bodies[0].body));
    assert!((vol(o.body.as_ref().unwrap()) - (1000.0 - PI * 4.0 * 10.0)).abs() < 1e-9);
    // Nothing new in its file, which older readers can take.
    let text = doc.save_text();
    assert!(!text.contains("new_body") && !text.contains("\"body\""), "{text}");
    assert_eq!(o.step_text("m").unwrap(), write_step(o.body.as_ref().unwrap(), "m"));
}

/// Where the corners are, as a body's own box is a safe overestimate around curves.
fn exact_bounds(b: &Body) -> Aabb {
    let mut out = Aabb::EMPTY;
    for p in &b.vertices {
        out.add(*p);
    }
    out
}

fn one(feature: FeatureId) -> Vec<BodyId> {
    vec![BodyId { feature, index: 0 }]
}

fn drill(x: f64, mode: Mode) -> Op {
    Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(x, 5.0, -5.0)), mode)
}

const HOLE: f64 = 10.0 * PI;

#[test]
fn a_feature_acts_on_every_body_it_reaches_unless_it_lists_them() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 6.0));
    let far = doc.with(block(10.0, 10.0, 10.0, 100.0));
    // Through where the first two overlap: it cuts them both, and the third is left as it is.
    doc.with(drill(8.0, Mode::Remove));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert!((vol(&o.bodies[0].body) - (1000.0 - HOLE)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - HOLE)).abs() < 1e-9);
    assert!((vol(&o.bodies[2].body) - 1000.0).abs() < 1e-9);
    // Listed, it cuts only those.
    let only_a = doc.with(drill(4.0, Mode::Remove));
    doc.set_scope(only_a, one(a)).unwrap();
    let only_b = doc.with(drill(15.0, Mode::Remove));
    doc.set_scope(only_b, one(far)).unwrap();
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[0].body) - (1000.0 - 2.0 * HOLE)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - HOLE)).abs() < 1e-9);
    // That one misses what it's told to act on, and isn't an error.
    assert!((vol(&o.bodies[2].body) - 1000.0).abs() < 1e-9);
    // A body keeps its name and place however much is done to it.
    assert_eq!(o.bodies.iter().map(|b| b.id.feature).collect::<Vec<_>>(), vec![a, b, far]);
    // A blend goes where its edges are, wherever that is in the list.
    doc.with(Op::fillet(EdgeSelect::Radius(1.0.into()), 0.25));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert!(vol(&o.bodies[0].body) < 1000.0 - 2.0 * HOLE);
    assert!((vol(&o.bodies[2].body) - 1000.0).abs() < 1e-9);
}

#[test]
fn adding_joins_whatever_it_touches_and_only_that() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 20.0));
    let c = doc.with(block(10.0, 10.0, 10.0, 100.0));
    // A bridge between the first two, in the default scope.
    let bridge = Op::solid(Primitive::Box { dx: 14.0.into(), dy: 2.0.into(), dz: 2.0.into() }, Placement::at(point3(8.0, 4.0, 4.0)), Mode::Add);
    doc.with(bridge);
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|x| x.id.feature).collect::<Vec<_>>(), vec![a, c]);
    assert!((vol(&o.bodies[0].body) - (2000.0 + 10.0 * 2.0 * 2.0)).abs() < 1e-9);
    // Touching nothing, it's a new part.
    let lone = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::at(point3(0.0, 500.0, 0.0)), Mode::Add));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert_eq!(o.bodies[2].id, BodyId { feature: lone, index: 0 });
    // A scope keeps it away from what it would otherwise have joined.
    let apart = doc.with(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 10.0.into(), dz: 10.0.into() }, Placement::at(point3(105.0, 0.0, 0.0)), Mode::Add));
    doc.set_scope(apart, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 4, "a part of its own, overlapping the one it wasn't told to join");
    let _ = b;
}

#[test]
fn a_cut_through_a_part_makes_two_and_the_name_stays_with_what_was_there() {
    let mut doc = Document::new();
    let a = doc.with(block(30.0, 10.0, 10.0, 0.0));
    let cutter = doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 20.0.into(), dz: 20.0.into() }, Placement::at(point3(19.0, -5.0, -5.0)), Mode::Remove));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    // The larger piece keeps the name, as both have as many faces from before.
    assert_eq!(o.bodies[0].id, BodyId { feature: a, index: 0 });
    assert_eq!(o.bodies[1].id, BodyId { feature: cutter, index: 0 });
    assert!((vol(&o.bodies[0].body) - 1900.0).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 900.0).abs() < 1e-9);
    for b in &o.bodies {
        validate(&b.body).unwrap();
        assert_eq!(b.body.shells.len(), 1);
    }
    // The piece can then be got rid of like any other part.
    let gone = doc.with(Op::DeleteBodies);
    doc.set_scope(gone, one(cutter)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!((vol(&o.bodies[0].body) - 1900.0).abs() < 1e-9);
}

#[test]
fn cutting_all_of_a_part_away_takes_the_part() {
    let mut doc = Document::new();
    doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let all_of_it = Op::solid(Primitive::Box { dx: 30.0.into(), dy: 30.0.into(), dz: 30.0.into() }, Placement::at(point3(-10.0, -10.0, -10.0)), Mode::Remove);
    doc.with(all_of_it);
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|x| x.id.feature).collect::<Vec<_>>(), vec![b]);
}

#[test]
fn a_transform_moves_parts_and_their_names_go_with_them() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let m = doc.with(Op::Transform { motion: Motion::Translate { by: point3(0.0, 50.0, 0.0) }, copy: false });
    doc.set_scope(m, one(a)).unwrap();
    let o = ok(&mut doc);
    assert!(o.bodies[0].body.aabb().center().dist(v3(5.0, 55.0, 5.0)) < 1e-9);
    assert!(o.bodies[1].body.aabb().center().dist(v3(105.0, 5.0, 5.0)) < 1e-9);
    assert_eq!(o.bodies[0].id.feature, a);
    // Names still find it where it's gone.
    assert_eq!(find_faces(&o.bodies[0].body, &FaceKey::plain(a.end_cap())).len(), 1);

    // Around an axis, a quarter turn about the z axis through the origin.
    let r = doc.with(Op::Transform { motion: Motion::Rotate { point: point3(0.0, 0.0, 0.0), axis: point3(0.0, 0.0, 1.0), degrees: 90.0.into() }, copy: false });
    doc.set_scope(r, one(b)).unwrap();
    let o = ok(&mut doc);
    assert!(o.bodies[1].body.aabb().center().dist(v3(-5.0, 105.0, 5.0)) < 1e-9, "{:?}", o.bodies[1].body.aabb().center());

    // From one frame to another: its corner onto a point, its z along x.
    let from = Placement::at(point3(0.0, 50.0, 0.0));
    let to = Placement::new(point3(200.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0));
    let f = doc.with(Op::Transform { motion: Motion::Frames { from, to }, copy: false });
    doc.set_scope(f, one(a)).unwrap();
    let o = ok(&mut doc);
    assert!(o.bodies[0].body.aabb().center().dist(v3(205.0, 5.0, 5.0)) < 1e-9, "{:?}", o.bodies[0].body.aabb().center());
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
    validate(&o.bodies[0].body).unwrap();

    // A parameter moves it, and the other part isn't touched.
    let mut doc = Document::new();
    doc.set_parameter("x", 5.0).unwrap();
    let a = named(&mut doc, "A", block(10.0, 10.0, 10.0, 0.0));
    let move_a = named(&mut doc, "Move A", Op::Transform { motion: Motion::Translate { by: point3("x", 0.0, 0.0) }, copy: false });
    doc.set_scope(move_a, one(a)).unwrap();
    let b = named(&mut doc, "B", block(10.0, 10.0, 10.0, 100.0));
    let move_b = named(&mut doc, "Move B", Op::Transform { motion: Motion::Translate { by: point3(1.0, 0.0, 0.0) }, copy: false });
    doc.set_scope(move_b, one(b)).unwrap();
    ok(&mut doc);
    doc.set_parameter("x", 6.0).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["Move A"]);
    assert!(o.bodies[0].body.aabb().center().dist(v3(11.0, 5.0, 5.0)) < 1e-9);
}

#[test]
fn a_transform_with_copy_leaves_the_original_and_adds_a_part() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let m = doc.with(Op::Transform { motion: Motion::Translate { by: point3(0.0, 50.0, 0.0) }, copy: true });
    doc.set_scope(m, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!(o.bodies[0].body.aabb().center().dist(v3(5.0, 5.0, 5.0)) < 1e-9);
    assert!(o.bodies[1].body.aabb().center().dist(v3(5.0, 55.0, 5.0)) < 1e-9);
    assert_eq!(o.bodies[1].id, BodyId { feature: m, index: 0 });
    // Keys tell the copy from the original.
    let keys = face_keys(o.body.as_ref().unwrap());
    let texts: std::collections::HashSet<String> = keys.iter().map(FaceKey::text).collect();
    assert_eq!(texts.len(), keys.len());
    assert!(texts.iter().any(|k| k.contains(&format!("@f{}.1", m.0))));
    // Copying in place is a zero move, and it's a part like any other.
    let again = doc.with(Op::Transform { motion: Motion::Translate { by: point3(0.0, 0.0, 0.0) }, copy: true });
    doc.set_scope(again, one(a)).unwrap();
    assert_eq!(ok(&mut doc).bodies.len(), 3);
}

#[test]
fn delete_bodies_has_to_name_them_and_can_be_undone_by_suppressing_it() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let d = doc.with(Op::DeleteBodies);
    // Named nothing: it doesn't take all of them.
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(2));
    assert_eq!(o.bodies.len(), 2);
    doc.set_scope(d, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|x| x.id.feature).collect::<Vec<_>>(), vec![b]);
    // What comes after can't use it.
    let after = doc.with(drill(5.0, Mode::Remove));
    doc.set_scope(after, one(a)).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(3));
    let StepState::Failed(why) = &o.steps[3].state else { panic!() };
    assert!(why.contains(&format!("f{}", a.0)), "{why}");
    doc.delete(after).unwrap();
    // Off, it's as if it were never there.
    doc.suppress(d, true).unwrap();
    assert_eq!(ok(&mut doc).bodies.len(), 2);
    // Deleting everything leaves nothing to show, and that's fine.
    doc.suppress(d, false).unwrap();
    doc.set_scope(d, vec![BodyId { feature: a, index: 0 }, BodyId { feature: b, index: 0 }]).unwrap();
    let o = ok(&mut doc);
    assert!(o.bodies.is_empty() && o.body.is_none());
}

fn boolean(operation: BodyBoolean, tools: Vec<BodyId>, keep_tools: bool) -> Op {
    Op::BooleanBodies { operation, tools, keep_tools }
}

#[test]
fn a_union_of_bodies_joins_them_or_says_they_dont_touch() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 5.0));
    let far = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let u = doc.with(boolean(BodyBoolean::Union, one(b), false));
    doc.set_scope(u, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|x| x.id.feature).collect::<Vec<_>>(), vec![a, far]);
    assert!((vol(&o.bodies[0].body) - 1500.0).abs() < 1e-9);

    // Onshape refuses two that don't meet, and so does this.
    let apart = doc.with(boolean(BodyBoolean::Union, one(far), false));
    doc.set_scope(apart, one(a)).unwrap();
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("don't touch"), "{why}");
    doc.delete(apart).unwrap();

    // Keeping the tools keeps them all, and the result is a part of its own.
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 5.0));
    let u = doc.with(boolean(BodyBoolean::Union, one(b), true));
    doc.set_scope(u, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert!((vol(&o.bodies[2].body) - 1500.0).abs() < 1e-9);
    assert_eq!(o.bodies[2].id, BodyId { feature: u, index: 0 });
    // One body is no Boolean.
    let lonely = doc.with(boolean(BodyBoolean::Union, Vec::new(), false));
    doc.set_scope(lonely, one(a)).unwrap();
    assert!(doc.rebuild(None).failed.is_some());
}

#[test]
fn subtracting_bodies_needs_a_target_and_takes_the_tool_unless_told_to_keep_it() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let tool = doc.with(block(10.0, 10.0, 10.0, 5.0));
    let far = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let s = doc.with(boolean(BodyBoolean::Subtract, one(tool), false));
    // Which one to cut has to be said: it's not all of them.
    assert!(doc.rebuild(None).failed.is_some());
    doc.set_scope(s, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|x| x.id.feature).collect::<Vec<_>>(), vec![a, far]);
    assert!((vol(&o.bodies[0].body) - 500.0).abs() < 1e-9);

    // Kept.
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let tool = doc.with(block(10.0, 10.0, 10.0, 5.0));
    let s = doc.with(boolean(BodyBoolean::Subtract, one(tool), true));
    doc.set_scope(s, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!((vol(&o.bodies[0].body) - 500.0).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 1000.0).abs() < 1e-9);

    // A tool that misses is refused, as nothing would happen, whatever was hoped for.
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let tool = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let s = doc.with(boolean(BodyBoolean::Subtract, one(tool), false));
    doc.set_scope(s, one(a)).unwrap();
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("reach"), "{why}");

    // A tool through the middle leaves two parts.
    let mut doc = Document::new();
    let a = doc.with(block(30.0, 10.0, 10.0, 0.0));
    let tool = doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 20.0.into(), dz: 20.0.into() }, Placement::at(point3(19.0, -5.0, -5.0)), Mode::NewBody));
    let s = doc.with(boolean(BodyBoolean::Subtract, one(tool), false));
    doc.set_scope(s, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert_eq!(o.bodies[0].id, BodyId { feature: a, index: 0 });
    assert_eq!(o.bodies[1].id, BodyId { feature: s, index: 0 });
}

#[test]
fn intersecting_bodies_keeps_where_they_overlap() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 5.0));
    let i = doc.with(boolean(BodyBoolean::Intersect, one(b), false));
    doc.set_scope(i, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert_eq!(o.bodies[0].id.feature, a);
    assert!((vol(&o.bodies[0].body) - 500.0).abs() < 1e-9);

    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 100.0));
    let i = doc.with(boolean(BodyBoolean::Intersect, one(b), false));
    doc.set_scope(i, one(a)).unwrap();
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("overlap"), "{why}");
}

/// A rectangle at x from 10 to 14 and y from 0 to 6, whose centroid is at (12, 3), so Pappus's theorem gives the volume.
fn revolved(point: (f64, f64), direction: (f64, f64), degrees: f64) -> std::result::Result<Outcome, String> {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(10.0, 0.0, 4.0, 6.0))]));
    let axis = RevolveAxis { point: coreshape_document::point(point.0, point.1), direction: coreshape_document::point(direction.0, direction.1) };
    doc.with(Op::Revolve { sketch: s, degrees: degrees.into(), mode: Mode::Add, axis: Some(axis), regions: Vec::new() });
    let o = doc.rebuild(None);
    match o.failed {
        Some(i) => match &o.steps[i].state {
            StepState::Failed(why) => Err(why.clone()),
            _ => Err("failed".into()),
        },
        None => Ok(o),
    }
}

#[test]
fn a_revolve_can_go_around_any_line_in_its_sketch() {
    let area = 4.0 * 6.0;
    // Around the sketch's own y axis, with the profile to its right, a distance of 12 to the centroid.
    let check = |o: &Outcome, want: f64, what: &str| {
        let got = mass_properties(o.body.as_ref().unwrap()).volume;
        assert!((got - want).abs() < 1e-6, "{what}: {got} against {want}");
        validate(o.body.as_ref().unwrap()).unwrap();
    };
    check(&revolved((0.0, 0.0), (0.0, 1.0), 360.0).unwrap(), 2.0 * PI * 12.0 * area, "y axis");
    // The other way along it, so the profile's to its left. The same for a whole turn.
    check(&revolved((0.0, 0.0), (0.0, -1.0), 360.0).unwrap(), 2.0 * PI * 12.0 * area, "y axis, reversed");
    // Around a horizontal line below it, 3 from the centroid.
    check(&revolved((0.0, 0.0), (1.0, 0.0), 360.0).unwrap(), 2.0 * PI * 3.0 * area, "x axis");
    // A line offset to the right of it, with the profile on its left.
    check(&revolved((20.0, 0.0), (0.0, 1.0), 360.0).unwrap(), 2.0 * PI * 8.0 * area, "x = 20");
    // A slanted one, along y = x - 30, whose distance to the centroid is |12 - 3 - 30| / sqrt 2.
    let d = 21.0 * std::f64::consts::FRAC_1_SQRT_2;
    check(&revolved((30.0, 0.0), (1.0, 1.0), 360.0).unwrap(), 2.0 * PI * d * area, "slanted");
    // Part of a turn.
    check(&revolved((0.0, 0.0), (0.0, 1.0), 90.0).unwrap(), PI / 2.0 * 12.0 * area, "a quarter");
    // It starts from where it's drawn, and goes right-handed around the axis. Around +y from +x, that's toward -z.
    let o = revolved((0.0, 0.0), (0.0, 1.0), 90.0).unwrap();
    let bounds = exact_bounds(o.body.as_ref().unwrap());
    assert!(bounds.min.z < -13.9 && bounds.max.z.abs() < 1e-6, "{bounds:?}");
    // And the same with the axis reversed, which has to turn the other way from the same start: toward +z.
    let o = revolved((0.0, 0.0), (0.0, -1.0), 90.0).unwrap();
    check(&o, PI / 2.0 * 12.0 * area, "a quarter, reversed");
    let bounds = exact_bounds(o.body.as_ref().unwrap());
    assert!(bounds.max.z > 13.9 && bounds.min.z.abs() < 1e-6, "{bounds:?}");
    // Through the profile is refused.
    assert!(revolved((12.0, 0.0), (0.0, 1.0), 360.0).unwrap_err().contains("crosses"));
    // And a direction of nothing.
    assert!(revolved((0.0, 0.0), (0.0, 0.0), 360.0).is_err());
}

#[test]
fn a_revolves_axis_is_saved_and_a_plain_one_is_as_it_was() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(10.0, 0.0, 4.0, 6.0))]));
    doc.with(Op::Revolve { sketch: s, degrees: 360.0.into(), mode: Mode::Add, axis: Some(RevolveAxis { point: point(0.0, 0.0), direction: point(0.0, 1.0) }), regions: Vec::new() });
    let text = doc.save_text();
    assert!(text.contains("\"axis\""), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    assert!((vol(ok(&mut back).body.as_ref().unwrap()) - vol(ok(&mut doc).body.as_ref().unwrap())).abs() < 1e-9);
}

#[test]
fn a_sketch_on_a_face_follows_it() {
    // A plate, whose height is a parameter, with a boss on its top face.
    let mut doc = Document::new();
    doc.set_parameter("h", 10.0).unwrap();
    let outline = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 40.0))]));
    let plate = doc.with(Op::extrude(outline, 0.0, "h", Mode::Add));
    let on_top = Placement::at(point3(5.0, 0.0, 10.0)).following(FaceKey::plain(plate.end_cap()));
    let circle = doc.with(Op::sketch(on_top, vec![Region::new(Shape::circle(0.0, 0.0, 3.0))]));
    doc.with(Op::extrude(circle, 0.0, 7.0, Mode::Add));
    let top = |o: &Outcome| exact_bounds(o.body.as_ref().unwrap()).max.z;
    let o = ok(&mut doc);
    assert!((top(&o) - 17.0).abs() < 1e-9);
    // It goes up with the face, and keeps its place in the plane.
    doc.set_parameter("h", 25.0).unwrap();
    let o = ok(&mut doc);
    assert!((top(&o) - 32.0).abs() < 1e-9, "{}", top(&o));
    assert!((vol(o.body.as_ref().unwrap()) - (40.0 * 40.0 * 25.0 + PI * 9.0 * 7.0)).abs() < 1e-9);
    // Nothing else makes the sketch rebuild.
    let text = doc.save_text();
    assert!(text.contains("\"on_face\": \"f2:end\""), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert!((top(&ok(&mut back)) - 32.0).abs() < 1e-9);
    // A face that's gone stops it, and says which.
    doc.suppress(plate, true).unwrap();
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("f2:end"), "{why}");
    // A curved face isn't one it can follow.
    let mut doc = Document::new();
    let c = doc.with(Op::solid(Primitive::Cylinder { radius: 5.0.into(), height: 5.0.into() }, Placement::xy(0.0), Mode::Add));
    let sketch = doc.with(Op::sketch(Placement::xy(0.0).following(FaceKey::plain(c.side(0))), vec![Region::new(Shape::circle(0.0, 0.0, 1.0))]));
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("flat"), "{why}");
    let _ = sketch;
}

#[test]
fn a_sketch_on_a_face_rebuilds_only_when_the_face_moves() {
    let mut doc = Document::new();
    doc.set_parameter("h", 10.0).unwrap();
    doc.set_parameter("r", 1.0).unwrap();
    let outline = named(&mut doc, "Outline", Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 40.0))]));
    let plate = named(&mut doc, "Plate", Op::extrude(outline, 0.0, "h", Mode::Add));
    named(&mut doc, "Round", Op::fillet(EdgeSelect::Between(plate.start_cap(), plate.side(0)), "r"));
    let on_top = Placement::at(point3(0.0, 0.0, 0.0)).following(FaceKey::plain(plate.end_cap()));
    named(&mut doc, "On top", Op::sketch(on_top, vec![Region::new(Shape::circle(0.0, 0.0, 3.0))]));
    ok(&mut doc);
    // A fillet at the bottom changes the body, not the top face's plane.
    doc.set_parameter("r", 2.0).unwrap();
    assert_eq!(rebuilt(&ok(&mut doc)), vec!["Round"]);
    doc.set_parameter("h", 20.0).unwrap();
    assert_eq!(rebuilt(&ok(&mut doc)), vec!["Plate", "Round", "On top"]);
}

#[test]
fn an_app_can_keep_what_it_likes_in_the_file() {
    let mut doc = Document::new();
    let a = named(&mut doc, "A", block(10.0, 10.0, 10.0, 0.0));
    ok(&mut doc);
    let labels = json::parse(r#"{"labels": {"7": [1.5, 2]}, "note": "anything"}"#).unwrap();
    doc.set_app(Some(a), Some(labels.clone())).unwrap();
    doc.set_app(None, Some(json::parse(r#"{"camera": [1, 2, 3]}"#).unwrap())).unwrap();
    // It's not part of the model, so nothing rebuilds.
    assert_eq!(rebuilt(&ok(&mut doc)), Vec::<String>::new());
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    assert_eq!(back.features()[0].app.as_ref(), Some(&labels));
    assert!(back.app().is_some());
    assert_eq!(ok(&mut back).bodies.len(), 1);
    doc.set_app(Some(a), None).unwrap();
    assert!(!doc.save_text().contains("labels"));
}

#[test]
fn a_change_to_one_body_leaves_the_others_alone() {
    let mut doc = Document::new();
    doc.set_parameter("wide", 10.0).unwrap();
    doc.set_parameter("tall", 10.0).unwrap();
    let box_of = |dx: &str, dz: &str, x: f64| Op::solid(Primitive::Box { dx: dx.into(), dy: 10.0.into(), dz: dz.into() }, Placement::at(point3(x, 0.0, 0.0)), Mode::NewBody);
    let a = named(&mut doc, "A", box_of("wide", "10", 0.0));
    let round_a = named(&mut doc, "Round A", Op::fillet(EdgeSelect::OnFace(a.end_cap()), 1.0));
    doc.set_scope(round_a, one(a)).unwrap();
    let b = named(&mut doc, "B", box_of("10", "tall", 50.0));
    let round_b = named(&mut doc, "Round B", Op::fillet(EdgeSelect::OnFace(b.end_cap()), 1.0));
    doc.set_scope(round_b, one(b)).unwrap();
    ok(&mut doc);
    assert_eq!(rebuilt(&doc.rebuild(None)), Vec::<String>::new());

    // Only A, though B comes after it in the history.
    doc.set_parameter("wide", 12.0).unwrap();
    assert_eq!(rebuilt(&ok(&mut doc)), vec!["A", "Round A"]);
    // Only B.
    doc.set_parameter("tall", 12.0).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["B", "Round B"]);
    // And the results are what a rebuild from nothing gives.
    let mut fresh = Document::load_text(&doc.save_text()).unwrap();
    let f = ok(&mut fresh);
    for (x, y) in o.bodies.iter().zip(&f.bodies) {
        assert_eq!(x.id, y.id);
        assert!((vol(&x.body) - vol(&y.body)).abs() < 1e-9);
    }
}

#[test]
fn work_on_two_bodies_can_be_interleaved_and_stay_apart() {
    let mut doc = Document::new();
    doc.set_parameter("r", 1.0).unwrap();
    let a = named(&mut doc, "A", block(10.0, 10.0, 10.0, 0.0));
    let b = named(&mut doc, "B", block(10.0, 10.0, 10.0, 50.0));
    let on = |doc: &mut Document, name: &str, feature: FeatureId, x: f64, radius: &str| {
        let f = named(doc, name, Op::solid(Primitive::Cylinder { radius: radius.into(), height: 20.0.into() }, Placement::at(point3(x, 5.0, -5.0)), Mode::Remove));
        doc.set_scope(f, one(feature)).unwrap();
    };
    on(&mut doc, "Hole in A", a, 5.0, "r");
    on(&mut doc, "Hole in B", b, 55.0, "1");
    on(&mut doc, "Second in A", a, 2.0, "0.5");
    on(&mut doc, "Second in B", b, 52.0, "0.5");
    ok(&mut doc);
    // What came after on the same body rebuilds, and what came between on the other doesn't.
    doc.set_parameter("r", 1.5).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["Hole in A", "Second in A"]);
    let area = |r: f64| PI * r * r * 10.0;
    assert!((vol(&o.bodies[0].body) - (1000.0 - area(1.5) - area(0.5))).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - area(1.0) - area(0.5))).abs() < 1e-9);
}

#[test]
fn choosing_another_body_rebuilds_what_that_changes() {
    let mut doc = Document::new();
    let a = named(&mut doc, "A", block(10.0, 10.0, 10.0, 0.0));
    let b = named(&mut doc, "B", block(10.0, 10.0, 10.0, 0.0));
    let cut = named(&mut doc, "Cut", Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    doc.set_scope(cut, one(b)).unwrap();
    let hole = PI * 10.0;
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    // To the other one.
    doc.set_scope(cut, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["Cut"]);
    assert!((vol(&o.bodies[0].body) - (1000.0 - hole)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 1000.0).abs() < 1e-9);
    // And to the first again.
    doc.set_scope(cut, one(b)).unwrap();
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
}

#[test]
fn copies_follow_what_they_copy_whichever_body_that_changed() {
    let mut doc = Document::new();
    doc.set_parameter("r", 2.0).unwrap();
    let bolt = named(&mut doc, "Bolt", Op::solid(Primitive::Cylinder { radius: "r".into(), height: 20.0.into() }, Placement::xy(0.0), Mode::NewBody));
    // The pattern's own body is this one, which has nothing to do with the bolt.
    named(&mut doc, "Plate", block(100.0, 100.0, 5.0, 50.0));
    named(&mut doc, "Row", Op::Pattern { of: vec![bolt], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    let one = |r: f64| PI * r * r * 20.0;
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[2].body) - one(2.0)).abs() < 1e-9);
    doc.set_parameter("r", 3.0).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["Bolt", "Row"]);
    for b in [&o.bodies[0], &o.bodies[2], &o.bodies[3]] {
        assert!((vol(&b.body) - one(3.0)).abs() < 1e-9);
    }
    assert!((vol(&o.bodies[1].body) - 50_000.0).abs() < 1e-9);
}

#[test]
fn its_all_saved_and_comes_back() {
    let (plate, bolt) = plate_and_bolt();
    let mut doc = Document::new();
    let f = doc.with(Op::ImportStep { text: write_step_bodies(&[&plate, &bolt], "joint"), mode: Mode::NewBody });
    let cut = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 50.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    let on = BodyId { feature: f, index: 0 };
    doc.set_scope(cut, vec![on]).unwrap();
    let text = doc.save_text();
    assert!(text.contains("\"new_body\"") && text.contains("\"bodies\""), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    assert_eq!(back.features()[1].scope, vec![on]);
    let (a, b) = (ok(&mut doc), ok(&mut back));
    assert_eq!(a.bodies.len(), 2);
    assert_eq!(b.bodies.len(), 2);
    for (x, y) in a.bodies.iter().zip(&b.bodies) {
        assert!((vol(&x.body) - vol(&y.body)).abs() < 1e-9);
    }
}

#[test]
fn a_file_is_refused_if_it_names_a_body_it_cant_have() {
    let good = |body: &str| {
        format!(
            r#"{{"format": "coreshape-document", "version": 1, "units": "mm", "next_id": 3, "parameters": [], "features": [
            {{"id": 1, "name": "A", "type": "solid", "shape": "box", "dx": 1, "dy": 1, "dz": 1, "mode": "new_body"}},
            {{"id": 2, "name": "B", "type": "solid", "shape": "box", "dx": 2, "dy": 1, "dz": 1, "mode": "remove", "bodies": ["{body}"]}}]}}"#
        )
    };
    assert!(Document::load_text(&good("f1")).is_ok());
    // Not a name.
    for bad in ["1", "body", "f", "fx", "f1.x", "f1.", "f.1", "f1.2.3"] {
        assert!(Document::load_text(&good(bad)).is_err(), "{bad}");
    }
    // A feature that doesn't come before it.
    assert!(Document::load_text(&good("f2")).is_err());
    assert!(Document::load_text(&good("f9.2")).is_err());
    // A sketch has nothing to do with a body.
    let sketch = r#"{"format": "coreshape-document", "version": 1, "units": "mm", "next_id": 3, "parameters": [], "features": [
        {"id": 1, "name": "A", "type": "solid", "shape": "box", "dx": 1, "dy": 1, "dz": 1, "mode": "new_body"},
        {"id": 2, "name": "S", "type": "sketch", "bodies": ["f1"], "regions": []}]}"#;
    assert!(Document::load_text(sketch).is_err());
}

#[test]
fn names_of_bodies_read_and_write() {
    let id = |f: u32, index: u32| BodyId { feature: FeatureId(f), index };
    assert_eq!(id(3, 0).text(), "f3");
    assert_eq!(id(3, 2).text(), "f3.2");
    assert_eq!(BodyId::parse("f3").unwrap(), id(3, 0));
    assert_eq!(BodyId::parse("f3.0").unwrap(), id(3, 0));
    assert_eq!(BodyId::parse("f3.2").unwrap(), id(3, 2));
    assert!(BodyId::parse("f99999999999").is_err());
    assert!(BodyId::parse("g3").is_err());
}

#[test]
fn a_body_that_isnt_there_fails_that_feature_and_says_which() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let cut = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    // A second body of a feature that makes only one.
    doc.set_scope(cut, vec![BodyId { feature: a, index: 1 }]).unwrap();
    let after = doc.with(block(5.0, 5.0, 5.0, 50.0));
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(1));
    let StepState::Failed(why) = &o.steps[1].state else { panic!() };
    assert!(why.contains("f1.1"), "{why}");
    assert_eq!(o.steps[2].state, StepState::Blocked);
    let _ = after;
    // What it had so far is still there.
    assert_eq!(o.bodies.len(), 1);
    // Likewise once the feature that made it is suppressed.
    doc.set_scope(cut, one(a)).unwrap();
    assert_eq!(doc.rebuild(None).failed, None);
    doc.suppress(a, true).unwrap();
    assert_eq!(doc.rebuild(None).failed, Some(1));
}

#[test]
fn a_failure_on_one_of_several_bodies_says_which_it_was() {
    let mut doc = Document::new();
    doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 50.0));
    // Far too big a fillet for the part.
    let round = doc.with(Op::fillet(EdgeSelect::OnFace(b.end_cap()), 40.0));
    let o = doc.rebuild(None);
    let _ = round;
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains(&format!("body f{}", b.0)), "{why}");
}

#[test]
fn a_reference_to_a_body_holds_its_feature_in_place() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let cut = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    doc.set_scope(cut, one(a)).unwrap();
    assert_eq!(doc.dependents(a), vec![cut]);
    assert!(doc.delete(a).is_err());
    assert!(doc.reorder(cut, 0).is_err());
    assert!(doc.reorder(a, 1).is_err());
    // A feature can only act on what comes before it.
    assert!(doc.set_scope(a, one(cut)).is_err());
    doc.set_scope(cut, Vec::new()).unwrap();
    assert!(doc.dependents(a).is_empty());
    // A sketch acts on nothing.
    let sketch = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 1.0, 1.0))]));
    assert!(doc.set_scope(sketch, one(a)).is_err());
}

#[test]
fn separate_regions_are_separate_parts_and_touching_ones_are_one() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0)), Region::new(Shape::rect(20.0, 0.0, 4.0, 4.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 1.0, Mode::NewBody));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.iter().map(|b| b.id).collect::<Vec<_>>(), vec![BodyId { feature: e, index: 0 }, BodyId { feature: e, index: 1 }]);
    assert!((vol(&o.bodies[0].body) - 16.0).abs() < 1e-9);
    // The same with plain adding, as there's nothing yet to add to.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0)), Region::new(Shape::rect(20.0, 0.0, 4.0, 4.0))]));
    doc.with(Op::extrude(s, 0.0, 1.0, Mode::Add));
    assert_eq!(ok(&mut doc).bodies.len(), 2);
    // Touching, they're one.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0)), Region::new(Shape::rect(4.0, 0.0, 4.0, 4.0))]));
    doc.with(Op::extrude(s, 0.0, 1.0, Mode::NewBody));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!((vol(&o.bodies[0].body) - 32.0).abs() < 1e-9);
}

#[test]
fn a_part_keeps_the_voids_inside_it_and_not_those_inside_another() {
    // A hollow block, with a smaller hollow block floating in its hollow: two parts, each with its own void.
    let mut doc = Document::new();
    let cube = |s: f64, at: f64, mode: Mode| Op::solid(Primitive::Box { dx: s.into(), dy: s.into(), dz: s.into() }, Placement::at(point3(at, at, at)), mode);
    doc.with(cube(100.0, 0.0, Mode::Add));
    doc.with(cube(80.0, 10.0, Mode::Remove));
    doc.with(cube(40.0, 30.0, Mode::Add));
    doc.with(cube(20.0, 40.0, Mode::Remove));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let want = |outer: f64, inner: f64| outer * outer * outer - inner * inner * inner;
    let mut got: Vec<f64> = o.bodies.iter().map(|b| vol(&b.body)).collect();
    got.sort_by(f64::total_cmp);
    assert!((got[0] - want(40.0, 20.0)).abs() < 1e-9, "{got:?}");
    assert!((got[1] - want(100.0, 80.0)).abs() < 1e-9, "{got:?}");
    for b in &o.bodies {
        assert_eq!(b.body.shells.len(), 2);
        validate(&b.body).unwrap();
    }
}

#[test]
fn copies_of_a_body_are_bodies_with_names_of_their_own() {
    let mut doc = Document::new();
    let plate = doc.with(block(100.0, 100.0, 5.0, 0.0));
    let bolt = doc.with(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 20.0.into() }, Placement::at(point3(10.0, 10.0, -5.0)), Mode::NewBody));
    let row = doc.with(Op::Pattern { of: vec![bolt], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 4.0.into(), spacing: 20.0.into() } });
    let o = ok(&mut doc);
    let names: Vec<String> = o.bodies.iter().map(|b| b.id.text()).collect();
    assert_eq!(names, vec![format!("f{}", plate.0), format!("f{}", bolt.0), format!("f{}", row.0), format!("f{}.1", row.0), format!("f{}.2", row.0)]);
    let one = PI * 4.0 * 20.0;
    for b in &o.bodies[1..] {
        assert!((vol(&b.body) - one).abs() < 1e-9);
    }
    assert!((vol(&o.bodies[0].body) - 50_000.0).abs() < 1e-9);
    assert!((o.bodies[4].body.aabb().center().x - 70.0).abs() < 1e-3);
    // Every face of the whole has a key of its own.
    let keys = face_keys(o.body.as_ref().unwrap());
    let texts: std::collections::HashSet<String> = keys.iter().map(FaceKey::text).collect();
    assert_eq!(texts.len(), keys.len());
    // A mirror is the same.
    doc.with(Op::Mirror { of: vec![bolt], point: point3(50.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 6);
    assert!((o.bodies[5].body.aabb().center().x - 90.0).abs() < 1e-3);
}

#[test]
fn copies_of_what_a_feature_added_join_what_they_land_on() {
    let mut doc = Document::new();
    let a = doc.with(block(100.0, 10.0, 10.0, 0.0));
    let boss = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 3.0.into() }, Placement::at(point3(5.0, 5.0, 10.0)), Mode::Add));
    doc.set_scope(boss, one(a)).unwrap();
    // A later body, which they don't reach.
    doc.with(block(10.0, 10.0, 10.0, 500.0));
    doc.with(Op::Pattern { of: vec![boss], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let one = PI * 3.0;
    assert!((vol(&o.bodies[0].body) - (10_000.0 + 3.0 * one)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 1000.0).abs() < 1e-9);
}

#[test]
fn a_pattern_of_a_whole_part_makes_parts() {
    let mut doc = Document::new();
    doc.with(block(5.0, 5.0, 5.0, 0.0));
    let b = doc.with(block(5.0, 5.0, 5.0, 100.0));
    let p = doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    doc.set_scope(p, one(b)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 4);
    assert_eq!(o.bodies[1].id.feature, b);
    assert_eq!(o.bodies[2].id, BodyId { feature: p, index: 0 });
    assert!((vol(&o.bodies[0].body) - 125.0).abs() < 1e-9);
    assert!(o.bodies[3].body.aabb().center().dist(v3(102.5, 42.5, 2.5)) < 1e-9);
    // Close enough to touch, they're one.
    let mut doc = Document::new();
    doc.with(block(5.0, 5.0, 5.0, 0.0));
    doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 5.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!((vol(&o.bodies[0].body) - 375.0).abs() < 1e-9);
}

#[test]
fn thickening_faces_of_one_body_can_make_another() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    doc.with(block(10.0, 10.0, 10.0, 50.0));
    let t = doc.with(Op::Thicken { faces: vec![FaceKey::plain(a.end_cap())], thickness: 2.0.into(), mode: Mode::NewBody });
    doc.set_scope(t, one(a)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
    assert!((vol(&o.bodies[2].body) - 200.0).abs() < 1e-9);
    assert_eq!(o.bodies[2].id, BodyId { feature: t, index: 0 });
}

#[test]
fn going_back_in_the_history_gives_the_bodies_there_were_then() {
    let mut doc = Document::new();
    doc.with(block(5.0, 5.0, 5.0, 0.0));
    doc.with(block(5.0, 5.0, 5.0, 50.0));
    doc.with(block(5.0, 5.0, 5.0, 100.0));
    assert_eq!(doc.rebuild(None).bodies.len(), 3);
    assert_eq!(doc.rebuild(Some(0)).bodies.len(), 1);
    assert_eq!(doc.rebuild(Some(1)).bodies.len(), 2);
    let o = doc.rebuild(None);
    assert_eq!(o.bodies.len(), 3);
    assert_eq!(rebuilt(&o), Vec::<String>::new());
}

#[test]
fn every_body_goes_to_a_step_file_and_comes_back_where_it_was() {
    // One part with a void, whose box holds another part that has a void of its own: a single body's voids have to stay
    // with its own solids, however the boxes overlap.
    let outer = box_solid(&Frame::WORLD, 100.0, 100.0, 100.0, 1).unwrap();
    let void = sphere(&Frame::translation(v3(20.0, 20.0, 20.0)), 5.0, 2).unwrap();
    let hollow = subtract(&outer, &void).unwrap();
    let inner = box_solid(&Frame::translation(v3(60.0, 60.0, 60.0)), 30.0, 30.0, 30.0, 3).unwrap();
    let small = sphere(&Frame::translation(v3(75.0, 75.0, 75.0)), 5.0, 4).unwrap();
    let nested = subtract(&inner, &small).unwrap();
    let text = write_step_bodies(&[&hollow, &nested], "nested");
    let back = read_step(&text, 1).unwrap();
    assert_eq!(back.len(), 2);
    for (a, b) in [(&hollow, &back[0]), (&nested, &back[1])] {
        assert_eq!(a.shells.len(), b.shells.len());
        assert!((vol(a) - vol(b)).abs() < 1e-6, "{} against {}", vol(a), vol(b));
        assert!(a.aabb().center().dist(b.aabb().center()) < 1e-9);
    }

    // And the same through a document.
    let mut doc = Document::new();
    doc.with(Op::ImportStep { text, mode: Mode::NewBody });
    doc.with(block(5.0, 5.0, 5.0, 300.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    let again = read_step(&o.step_text("m").unwrap(), 1).unwrap();
    assert_eq!(again.len(), 3);
    for (a, b) in o.bodies.iter().zip(&again) {
        assert!((vol(&a.body) - vol(b)).abs() < 1e-6);
        assert_eq!(a.body.shells.len(), b.shells.len());
    }
}

#[test]
fn nothing_to_show_is_nothing_at_all() {
    let mut doc = Document::new();
    let o = doc.rebuild(None);
    assert!(o.body.is_none() && o.bodies.is_empty() && o.step_text("m").is_none());
    assert!(o.body_of_face(0).is_none() && o.body_of_edge(0).is_none());
}

#[test]
fn removing_with_no_body_at_all_is_still_refused() {
    let mut doc = Document::new();
    doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 2.0.into() }, Placement::xy(0.0), Mode::Remove));
    assert_eq!(doc.rebuild(None).failed, Some(0));
}
