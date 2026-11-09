//! A document with several bodies: parts that are kept apart, so that they can touch, overlap and be worked on one at a
//! time. A document with one body has to work exactly as it did.

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
    // A cylinder just inside another, along the same axis, which a Boolean refuses as too close to call.
    let a = cylinder(&Frame::WORLD, 5.0, 10.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(1e-5, 0.0, 2.0)), 5.0, 10.0, 2).unwrap();
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

#[test]
fn a_feature_acts_on_the_last_body_made_unless_it_says_otherwise() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let b = doc.with(block(10.0, 10.0, 10.0, 100.0));
    // Say nothing, and it's the second.
    let cut = |x: f64| Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(x, 5.0, -5.0)), Mode::Remove);
    let on_last = doc.with(cut(105.0));
    let o = ok(&mut doc);
    let hole = PI * 10.0;
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    // Say the first.
    let on_first = doc.with(cut(5.0));
    doc.set_body(on_first, Some(BodyId { feature: a, index: 0 })).unwrap();
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[0].body) - (1000.0 - hole)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    // A body keeps its name and place however much is done to it.
    assert_eq!(o.bodies[0].id.feature, a);
    assert_eq!(o.bodies[1].id.feature, b);
    // Blends, like anything else, go where they're told.
    let round = doc.with(Op::fillet(EdgeSelect::OnFace(a.end_cap()), 1.0));
    doc.set_body(round, Some(BodyId { feature: a, index: 0 })).unwrap();
    let o = ok(&mut doc);
    assert!(vol(&o.bodies[0].body) < 1000.0 - hole);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    assert_eq!(o.bodies.len(), 2);
    let _ = on_last;
}

#[test]
fn a_change_to_one_body_leaves_the_others_alone() {
    let mut doc = Document::new();
    doc.set_parameter("wide", 10.0).unwrap();
    doc.set_parameter("tall", 10.0).unwrap();
    let box_of = |dx: &str, dz: &str, x: f64| Op::solid(Primitive::Box { dx: dx.into(), dy: 10.0.into(), dz: dz.into() }, Placement::at(point3(x, 0.0, 0.0)), Mode::NewBody);
    let a = named(&mut doc, "A", box_of("wide", "10", 0.0));
    let round_a = named(&mut doc, "Round A", Op::fillet(EdgeSelect::OnFace(a.end_cap()), 1.0));
    doc.set_body(round_a, Some(BodyId { feature: a, index: 0 })).unwrap();
    let b = named(&mut doc, "B", box_of("10", "tall", 50.0));
    let round_b = named(&mut doc, "Round B", Op::fillet(EdgeSelect::OnFace(b.end_cap()), 1.0));
    doc.set_body(round_b, Some(BodyId { feature: b, index: 0 })).unwrap();
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
        doc.set_body(f, Some(BodyId { feature, index: 0 })).unwrap();
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
    let hole = PI * 10.0;
    let o = ok(&mut doc);
    assert!((vol(&o.bodies[0].body) - 1000.0).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - (1000.0 - hole)).abs() < 1e-9);
    // To the other one.
    doc.set_body(cut, Some(BodyId { feature: a, index: 0 })).unwrap();
    let o = ok(&mut doc);
    assert_eq!(rebuilt(&o), vec!["Cut"]);
    assert!((vol(&o.bodies[0].body) - (1000.0 - hole)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 1000.0).abs() < 1e-9);
    // And saying the one it would have had anyway gives the same answer.
    doc.set_body(cut, Some(BodyId { feature: b, index: 0 })).unwrap();
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
    doc.set_body(cut, Some(on)).unwrap();
    let text = doc.save_text();
    assert!(text.contains("\"new_body\"") && text.contains("\"body\": \"f1\""), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    assert_eq!(back.features()[1].body, Some(on));
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
            {{"id": 2, "name": "B", "type": "solid", "shape": "box", "dx": 2, "dy": 1, "dz": 1, "mode": "remove", "body": "{body}"}}]}}"#
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
        {"id": 2, "name": "S", "type": "sketch", "body": "f1", "regions": []}]}"#;
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
    doc.set_body(cut, Some(BodyId { feature: a, index: 1 })).unwrap();
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
    doc.set_body(cut, Some(BodyId { feature: a, index: 0 })).unwrap();
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
    doc.with(Op::fillet(EdgeSelect::OnFace(b.end_cap()), 40.0));
    let o = doc.rebuild(None);
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains(&format!("body f{}", b.0)), "{why}");
}

#[test]
fn a_reference_to_a_body_holds_its_feature_in_place() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    let cut = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 20.0.into() }, Placement::at(point3(5.0, 5.0, -5.0)), Mode::Remove));
    doc.set_body(cut, Some(BodyId { feature: a, index: 0 })).unwrap();
    assert_eq!(doc.dependents(a), vec![cut]);
    assert!(doc.delete(a).is_err());
    assert!(doc.reorder(cut, 0).is_err());
    assert!(doc.reorder(a, 1).is_err());
    // A feature can only act on what comes before it.
    assert!(doc.set_body(a, Some(BodyId { feature: cut, index: 0 })).is_err());
    doc.set_body(cut, None).unwrap();
    assert!(doc.dependents(a).is_empty());
    // A sketch acts on nothing.
    let sketch = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 1.0, 1.0))]));
    assert!(doc.set_body(sketch, Some(BodyId { feature: a, index: 0 })).is_err());
}

#[test]
fn a_new_body_has_to_come_from_something() {
    // A sketch's several regions make one body, joined as they are anywhere else.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0)), Region::new(Shape::rect(20.0, 0.0, 4.0, 4.0))]));
    doc.with(Op::extrude(s, 0.0, 1.0, Mode::NewBody));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert_eq!(o.bodies[0].body.shells.len(), 2);
    assert!((vol(&o.bodies[0].body) - 32.0).abs() < 1e-9);
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
fn copies_of_what_a_feature_added_go_to_the_body_it_added_them_to() {
    let mut doc = Document::new();
    let a = doc.with(block(100.0, 10.0, 10.0, 0.0));
    let boss = doc.with(Op::solid(Primitive::Cylinder { radius: 1.0.into(), height: 3.0.into() }, Placement::at(point3(5.0, 5.0, 10.0)), Mode::Add));
    doc.set_body(boss, Some(BodyId { feature: a, index: 0 })).unwrap();
    // A later body, which isn't the one the copies are of.
    doc.with(block(10.0, 10.0, 10.0, 500.0));
    doc.with(Op::Pattern { of: vec![boss], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let one = PI * 3.0;
    assert!((vol(&o.bodies[0].body) - (10_000.0 + 3.0 * one)).abs() < 1e-9);
    assert!((vol(&o.bodies[1].body) - 1000.0).abs() < 1e-9);
}

#[test]
fn a_pattern_of_a_whole_body_stays_in_that_body() {
    let mut doc = Document::new();
    doc.with(block(5.0, 5.0, 5.0, 0.0));
    let b = doc.with(block(5.0, 5.0, 5.0, 100.0));
    let p = doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert_eq!(o.bodies[1].id.feature, b);
    assert_eq!(o.bodies[1].body.shells.len(), 3);
    assert!((vol(&o.bodies[0].body) - 125.0).abs() < 1e-9);
    let _ = p;
}

#[test]
fn thickening_faces_of_one_body_can_make_another() {
    let mut doc = Document::new();
    let a = doc.with(block(10.0, 10.0, 10.0, 0.0));
    doc.with(block(10.0, 10.0, 10.0, 50.0));
    let t = doc.with(Op::Thicken { faces: vec![FaceKey::plain(a.end_cap())], thickness: 2.0.into(), mode: Mode::NewBody });
    doc.set_body(t, Some(BodyId { feature: a, index: 0 })).unwrap();
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
