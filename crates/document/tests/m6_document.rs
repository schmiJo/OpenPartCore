//! Milestone 6: the document rebuilds correctly, keeps its references, and is plain data
//! that can be saved, edited, suppressed and reordered.

use coreshape::*;
use coreshape_document::*;

fn corner_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

fn vol(o: &Outcome) -> f64 {
    mass_properties(o.body.as_ref().expect("a body")).volume
}

fn rebuilt(o: &Outcome) -> Vec<&str> {
    o.steps.iter().filter(|s| s.state == StepState::Rebuilt).map(|s| s.name.as_str()).collect()
}

fn set(doc: &mut Document, values: [f64; 5]) {
    for (name, v) in ["w", "d", "t", "r", "hole"].iter().zip(values) {
        doc.set_parameter(name, v).unwrap();
    }
}

const BASE: [f64; 5] = [40.0, 20.0, 6.0, 2.0, 3.0];

/// A plate: w, d, t, fillet radius r on its long top edge, hole radius.
fn plate() -> (Document, FeatureId) {
    let mut doc = Document::new();
    set(&mut doc, BASE);
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", "d"))]));
    let e = doc.with(Op::extrude(s, 0.0, "t", Mode::Add));
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), "r"));
    let s2 = doc.with(Op::sketch(
        Placement::at(point3("w / 2", "d / 2", 0.0)),
        vec![Region::new(Shape::circle(0.0, 0.0, "hole"))],
    ));
    doc.with(Op::extrude(s2, -1.0, "t + 2", Mode::Remove));
    (doc, e)
}

fn plate_volume(p: [f64; 5]) -> f64 {
    p[0] * p[1] * p[2] - corner_area(p[3]) * p[0] - PI * p[4] * p[4] * p[2]
}

#[test]
fn a_document_builds_the_expected_solid() {
    let (mut doc, _) = plate();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - plate_volume(BASE)).abs() < 1e-8);
    let names: Vec<&str> = o.steps.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Sketch 1", "Extrude 1", "Fillet 1", "Sketch 2", "Extrude 2"]);
    validate(o.body.as_ref().unwrap()).unwrap();
}

#[test]
fn changing_any_dimension_rebuilds_correctly() {
    let (mut doc, _) = plate();
    doc.rebuild(None);
    for changed in [
        [55.0, 20.0, 6.0, 2.0, 3.0],
        [40.0, 33.0, 6.0, 2.0, 3.0],
        [40.0, 20.0, 9.0, 2.0, 3.0],
        [40.0, 20.0, 6.0, 4.5, 3.0],
        [40.0, 20.0, 6.0, 2.0, 5.5],
    ] {
        set(&mut doc, changed);
        let o = doc.rebuild(None);
        assert!(o.failed.is_none(), "{changed:?}: {:?}", o.steps);
        assert!((vol(&o) - plate_volume(changed)).abs() < 1e-8, "{changed:?}");
        set(&mut doc, BASE);
        doc.rebuild(None);
    }
}

#[test]
fn unchanged_steps_are_reused() {
    let (mut doc, _) = plate();
    let first = doc.rebuild(None);
    assert_eq!(rebuilt(&first).len(), 5);
    // Nothing changed.
    assert!(rebuilt(&doc.rebuild(None)).is_empty());
    // Only the hole radius: its sketch and everything after it.
    doc.set_parameter("hole", 4.0).unwrap();
    assert_eq!(rebuilt(&doc.rebuild(None)), ["Sketch 2", "Extrude 2"]);
    // The fillet radius: the fillet and everything that depends on the body. The hole's
    // sketch is unaffected and stays cached.
    doc.set_parameter("r", 3.0).unwrap();
    assert_eq!(rebuilt(&doc.rebuild(None)), ["Fillet 1", "Extrude 2"]);
    // The first dimension affects everything.
    doc.set_parameter("w", 50.0).unwrap();
    assert_eq!(rebuilt(&doc.rebuild(None)).len(), 5);
    let o = doc.rebuild(None);
    assert!((vol(&o) - plate_volume([50.0, 20.0, 6.0, 3.0, 4.0])).abs() < 1e-8);
}

#[test]
fn rewriting_a_dimension_with_the_same_value_rebuilds_nothing_downstream() {
    let (mut doc, e) = plate();
    doc.rebuild(None);
    // The text changes and the value doesn't: only the extrude's own text differs, so it
    // recomputes. A parameter written differently with the same value changes nothing at
    // all, because features refer to the name.
    doc.set_parameter("t", "2 * 3").unwrap();
    assert!(rebuilt(&doc.rebuild(None)).is_empty());
    let f = doc.feature(e).unwrap().op.clone();
    doc.edit(e, f).unwrap();
    assert!(rebuilt(&doc.rebuild(None)).is_empty());
}

#[test]
fn rollback_shows_earlier_states_without_recomputing() {
    let (mut doc, _) = plate();
    doc.rebuild(None);
    let at_extrude = doc.rebuild(Some(1));
    assert!((vol(&at_extrude) - 40.0 * 20.0 * 6.0).abs() < 1e-9);
    assert!(rebuilt(&at_extrude).is_empty());
    assert_eq!(at_extrude.steps[4].state, StepState::RolledBack);
    let at_fillet = doc.rebuild(Some(2));
    assert!((vol(&at_fillet) - (4800.0 - corner_area(2.0) * 40.0)).abs() < 1e-9);
    let end = doc.rebuild(None);
    assert!((vol(&end) - plate_volume(BASE)).abs() < 1e-8);
    // A rollback before a change, then a roll forward, still ends up right.
    doc.set_parameter("hole", 4.0).unwrap();
    doc.rebuild(Some(1));
    let mut q = BASE;
    q[4] = 4.0;
    assert!((vol(&doc.rebuild(None)) - plate_volume(q)).abs() < 1e-8);
}

#[test]
fn a_failing_feature_keeps_the_last_good_body_and_recovers() {
    let (mut doc, _) = plate();
    doc.rebuild(None);
    // A fillet bigger than the 6 mm face it runs along.
    doc.set_parameter("r", 7.0).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(2));
    assert!(matches!(o.steps[2].state, StepState::Failed(_)));
    assert_eq!(o.steps[4].state, StepState::Blocked);
    assert!((vol(&o) - 4800.0).abs() < 1e-9, "should show the body before the failure");
    // Back to a good value: everything works again, and nothing needs recomputing.
    doc.set_parameter("r", 2.0).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none());
    assert!((vol(&o) - plate_volume(BASE)).abs() < 1e-8);
    assert!(rebuilt(&o).is_empty(), "{:?}", rebuilt(&o));
}

#[test]
fn names_survive_a_feature_that_splits_the_edge() {
    // A slot cuts across the front edge before the fillet is applied. The edge between
    // the same two named faces is now two pieces, and the fillet must find both.
    let mut doc = Document::new();
    doc.set_parameter("t", 6.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 20.0))]));
    let e = doc.with(Op::extrude(s, 0.0, "t", Mode::Add));
    let slot = doc.with(Op::sketch(Placement::xy("t - 2"), vec![Region::new(Shape::rect(12.0, -1.0, 6.0, 8.0))]));
    doc.with(Op::extrude(slot, 0.0, 5.0, Mode::Remove));
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), 1.0));
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let expected = 30.0 * 20.0 * 6.0 - 6.0 * 7.0 * 2.0 - corner_area(1.0) * 24.0;
    assert!((vol(&o) - expected).abs() < 1e-8, "got {} want {expected}", vol(&o));
    // Thicker plate: same names, same result shape.
    doc.set_parameter("t", 9.0).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let expected = 30.0 * 20.0 * 9.0 - 6.0 * 7.0 * 2.0 - corner_area(1.0) * 24.0;
    assert!((vol(&o) - expected).abs() < 1e-8);
}

#[test]
fn a_reference_that_stops_existing_is_reported() {
    // Cutting all the way across removes the edge the fillet refers to.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let cut = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-1.0, -1.0, 12.0, 3.0))]));
    doc.with(Op::extrude(cut, -1.0, 7.0, Mode::Remove));
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), 1.0));
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(4));
    match &o.steps[4].state {
        StepState::Failed(why) => assert!(why.contains("no longer exist"), "{why}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn sketches_with_repeated_regions() {
    let mut doc = Document::new();
    doc.set_parameter("n", 3.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("50", "50"))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let holes = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::circle(0.0, 0.0, 3.0)).repeated(Repeat::Circular {
            count: "n".into(),
            radius: 15.0.into(),
            start_degrees: 0.0.into(),
        })],
    ));
    doc.with(Op::extrude(holes, -1.0, 7.0, Mode::Remove));
    for n in [3.0, 5.0, 8.0] {
        doc.set_parameter("n", n).unwrap();
        let o = doc.rebuild(None);
        assert!(o.failed.is_none(), "{:?}", o.steps);
        assert!((vol(&o) - (12500.0 - n * PI * 9.0 * 5.0)).abs() < 1e-8);
        assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![n as usize]);
    }
}

#[test]
fn a_grid_of_holes_and_each_region_keeps_its_name() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let holes = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::circle(10.0, 10.0, 2.0)).repeated(Repeat::Grid {
            nx: 3.0.into(),
            ny: 2.0.into(),
            dx: 20.0.into(),
            dy: 20.0.into(),
        })],
    ));
    let drill = doc.with(Op::extrude(holes, -1.0, 7.0, Mode::Remove));
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - (12000.0 - 6.0 * PI * 4.0 * 5.0)).abs() < 1e-8);
    let body = o.body.unwrap();
    let names = face_names(&body);
    for region in 0..6 {
        let name = drill.region_side(region, 0).name();
        assert!(names.contains(&name), "{name} not in {names:?}");
    }
}

#[test]
fn primitives_combine_as_features() {
    let mut doc = Document::new();
    doc.set_parameter("size", 20.0).unwrap();
    doc.with(Op::solid(
        Primitive::Box { dx: "size".into(), dy: "size".into(), dz: "size".into() },
        Placement::xy(0.0),
        Mode::Add,
    ));
    doc.with(Op::solid(
        Primitive::Cylinder { radius: 4.0.into(), height: "size + 2".into() },
        Placement::at(point3("size / 2", "size / 2", -1.0)),
        Mode::Remove,
    ));
    doc.with(Op::solid(Primitive::Sphere { radius: 3.0.into() }, Placement::at(point3("size / 2", 0.0, "size / 2")), Mode::Remove));
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let half_ball = 2.0 / 3.0 * PI * 27.0;
    assert!((vol(&o) - (8000.0 - PI * 16.0 * 20.0 - half_ball)).abs() < 1e-7, "{}", vol(&o));
}

// ----- Editing -----

#[test]
fn suppressing_a_feature_rebuilds_as_if_it_were_absent() {
    let (mut doc, _) = plate();
    doc.rebuild(None);
    let fillet = doc.features()[2].id;
    doc.suppress(fillet, true).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert_eq!(o.steps[2].state, StepState::Suppressed);
    assert!((vol(&o) - (4800.0 - PI * 9.0 * 6.0)).abs() < 1e-8);
    // Turning it back on reuses the fillet. Each step keeps one result, and the hole's
    // was replaced while the fillet was off, so that one step runs again.
    doc.suppress(fillet, false).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(rebuilt(&o), ["Extrude 2"]);
    assert!((vol(&o) - plate_volume(BASE)).abs() < 1e-8);
}

#[test]
fn suppressing_a_sketch_fails_what_needs_it_with_a_clear_message() {
    let (mut doc, _) = plate();
    let hole_sketch = doc.features()[3].id;
    doc.suppress(hole_sketch, true).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(4));
    match &o.steps[4].state {
        StepState::Failed(why) => assert!(why.contains("suppressed"), "{why}"),
        other => panic!("{other:?}"),
    }
    doc.suppress(hole_sketch, false).unwrap();
    assert!(doc.rebuild(None).failed.is_none());
}

#[test]
fn deleting_is_refused_while_something_depends_on_it() {
    let (mut doc, e) = plate();
    let first_sketch = doc.features()[0].id;
    let err = doc.delete(first_sketch).unwrap_err().to_string();
    assert!(err.contains("refer to it"), "{err}");
    // The fillet refers to the extrude's faces.
    assert!(doc.delete(e).is_err());
    let fillet = doc.features()[2].id;
    assert_eq!(doc.dependents(e), vec![fillet]);
    doc.delete(fillet).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none());
    assert!((vol(&o) - (4800.0 - PI * 9.0 * 6.0)).abs() < 1e-8);
    assert_eq!(doc.features().len(), 4);
}

#[test]
fn ids_are_never_reused_even_across_a_save() {
    let (mut doc, _) = plate();
    let last = doc.features()[4].id;
    doc.delete(last).unwrap();
    let saved = doc.save_text();
    let mut loaded = Document::load_text(&saved).unwrap();
    let new = loaded
        .add(Op::solid(Primitive::Sphere { radius: 1.0.into() }, Placement::xy(0.0), Mode::Add))
        .unwrap();
    assert!(new.0 > last.0, "{new:?} reuses or precedes the deleted {last:?}");
}

#[test]
fn reordering_changes_the_result_order_and_refuses_broken_order() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 20.0, 20.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let cut = doc.with(Op::solid(
        Primitive::Box { dx: 40.0.into(), dy: 40.0.into(), dz: 5.0.into() },
        Placement::at(point3(-10.0, -10.0, 8.0)),
        Mode::Remove,
    ));
    let boss = doc.with(Op::solid(
        Primitive::Cylinder { radius: 3.0.into(), height: 12.0.into() },
        Placement::at(point3(10.0, 10.0, 0.0)),
        Mode::Add,
    ));
    let boss_tip = PI * 9.0 * 4.0;
    // Cut after the boss exists: the boss is trimmed at z = 8.
    doc.reorder(boss, 2).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 3200.0).abs() < 1e-8, "{}", vol(&o));
    // Cut first: the boss then stands above the cut plate.
    doc.reorder(cut, 2).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - (3200.0 + boss_tip)).abs() < 1e-8, "{}", vol(&o));
    assert_eq!(doc.features()[2].id, cut);
    assert_eq!(doc.features()[3].id, boss);
    // A feature can't go before what it uses, and a used one can't go after its user.
    assert!(doc.reorder(e, 0).is_err());
    assert!(doc.reorder(s, 1).is_err());
    assert_eq!(doc.features()[0].id, s, "a refused move changes nothing");
}

#[test]
fn inserting_in_the_middle_rebuilds_only_what_follows() {
    let (mut doc, e) = plate();
    doc.rebuild(None);
    let at = 3;
    let dent = doc
        .insert(
            Op::solid(
                Primitive::Box { dx: 4.0.into(), dy: 4.0.into(), dz: 2.0.into() },
                Placement::at(point3(2.0, 10.0, 0.0)),
                Mode::Remove,
            ),
            Some("Dent".to_string()),
            at,
        )
        .unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert_eq!(o.steps[0].state, StepState::Reused);
    assert_eq!(o.steps[2].state, StepState::Reused);
    assert_eq!(o.steps[3].state, StepState::Rebuilt);
    assert_eq!(o.steps[3].id, dent);
    assert_eq!(o.steps[3].name, "Dent");
    assert_eq!(o.steps[4].state, StepState::Reused, "the hole's sketch doesn't depend on the body");
    assert_eq!(o.steps[5].state, StepState::Rebuilt);
    assert!((vol(&o) - (plate_volume(BASE) - 32.0)).abs() < 1e-8);
    // A feature can't be inserted before something it refers to.
    let ahead = Op::fillet(EdgeSelect::OnFace(e.end_cap()), 1.0);
    assert!(doc.insert(ahead, None, 0).is_err());
}

#[test]
fn editing_a_feature_keeps_its_id_and_downstream_references() {
    let (mut doc, e) = plate();
    doc.rebuild(None);
    doc.edit(e, Op::extrude(doc.features()[0].id, 0.0, "t + 1", Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    // The fillet still finds its edge, and the hole still goes through 6 of the 7 mm.
    let expected = 40.0 * 20.0 * 7.0 - corner_area(2.0) * 40.0 - PI * 9.0 * 7.0;
    // The hole's extrude is t + 2 long from z = -1, which ends at z = 7: exactly flush.
    assert!((vol(&o) - expected).abs() < 1e-8, "{} vs {expected}", vol(&o));
    assert_eq!(doc.feature(e).unwrap().name, "Extrude 1");
    doc.rename(e, "Base").unwrap();
    assert_eq!(doc.feature(e).unwrap().name, "Base");
    assert_eq!(doc.rebuild(None).steps[1].name, "Base");
    // An edit can't introduce a forward reference.
    let later = doc.features()[3].id;
    assert!(doc.edit(e, Op::extrude(later, 0.0, 1.0, Mode::Add)).is_err());
    // An extrude must name a sketch.
    let err = doc.edit(doc.features()[2].id, Op::extrude(e, 0.0, 1.0, Mode::Add)).unwrap_err().to_string();
    assert!(err.contains("not a sketch"), "{err}");
}

// ----- Parameters -----

#[test]
fn parameters_can_depend_on_each_other_but_not_in_a_cycle() {
    let mut doc = Document::new();
    doc.set_parameter("a", 10.0).unwrap();
    doc.set_parameter("b", "a * 2").unwrap();
    doc.set_parameter("c", "b + a").unwrap();
    assert_eq!(doc.environment().unwrap()["c"], 30.0);
    let err = doc.set_parameter("a", "c - 1").unwrap_err().to_string();
    assert!(err.contains("cycle"), "{err}");
    assert_eq!(doc.environment().unwrap()["a"], 10.0, "a refused value changes nothing");
    let err = doc.set_parameter("d", "nope + 1").unwrap_err().to_string();
    assert!(err.contains("doesn't exist"), "{err}");
    assert!(doc.set_parameter("bad name", 1.0).is_err());
    assert!(doc.set_parameter("1x", 1.0).is_err());
    assert!(doc.remove_parameter("a").is_err(), "b uses a");
    doc.remove_parameter("c").unwrap();
    doc.remove_parameter("b").unwrap();
    doc.remove_parameter("a").unwrap();
    assert!(doc.remove_parameter("a").is_err());
}

#[test]
fn a_feature_using_a_parameter_blocks_its_removal() {
    let (mut doc, _) = plate();
    let err = doc.remove_parameter("hole").unwrap_err().to_string();
    assert!(err.contains("uses"), "{err}");
}

#[test]
fn a_bad_expression_fails_the_feature_with_its_message() {
    let (mut doc, e) = plate();
    doc.edit(e, Op::extrude(doc.features()[0].id, 0.0, "t / (w - w)", Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(1));
    match &o.steps[1].state {
        StepState::Failed(why) => assert!(why.contains("division by zero"), "{why}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn degenerate_dimensions_are_refused_not_panics() {
    for length in ["0", "-1e-12", "1e308 * 10", "sqrt(-1)"] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
        doc.with(Op::extrude(s, 0.0, length, Mode::Add));
        let o = doc.rebuild(None);
        assert_eq!(o.failed, Some(1), "length {length}: {:?}", o.steps);
    }
    // Zero-size wires, coincident points, an empty path.
    for shape in [
        Shape::rect(0.0, 0.0, 0.0, 10.0),
        Shape::circle(0.0, 0.0, 0.0),
        Shape::circle(0.0, 0.0, -1.0),
        Shape::polygon(vec![point(0.0, 0.0), point(0.0, 0.0), point(0.0, 0.0)]),
        Shape::polygon(vec![point(0.0, 0.0), point(1.0, 1.0)]),
        Shape::Path { start: point(0.0, 0.0), moves: vec![] },
        Shape::rounded(0.0, 0.0, 10.0, 10.0, 6.0),
        Shape::Path {
            start: point(0.0, 0.0),
            moves: vec![Move::Arc { to: point(10.0, 0.0), center: point(0.0, 3.0), ccw: true }, Move::Line { to: point(5.0, 5.0) }],
        },
    ] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(shape.clone())]));
        doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
        let o = doc.rebuild(None);
        assert_eq!(o.failed, Some(0), "{shape:?}: {:?}", o.steps);
    }
}

// ----- Saving and loading -----

fn every_kind_of_feature() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 50.0).unwrap();
    doc.set_parameter("half", "w / 2").unwrap();
    let plane = Placement::new(point3(0.0, 0.0, 1.0), point3(0.0, 0.0, 1.0), point3(1.0, 0.0, 0.0));
    let s = doc.with(Op::sketch(
        plane,
        vec![
            Region::new(Shape::rect("-half", "-half", "w", "w"))
                .hole(Shape::circle(0.0, 0.0, 5.0))
                .hole(Shape::polygon(vec![point(10.0, 10.0), point(15.0, 10.0), point(15.0, 15.0)])),
            Region::new(Shape::rounded(100.0, 0.0, 20.0, 20.0, 3.0)),
        ],
    ));
    let e = doc.with(Op::extrude(s, 0.0, 8.0, Mode::Add));
    let path = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::Path {
            start: point(30.0, 0.0),
            moves: vec![
                Move::Line { to: point(35.0, 0.0) },
                Move::Arc { to: point(35.0, 6.0), center: point(35.0, 3.0), ccw: true },
                Move::Line { to: point(30.0, 6.0) },
            ],
        })],
    ));
    doc.with(Op::revolve(path, "270", Mode::Add));
    let bolts = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::circle(0.0, 0.0, 1.0)).repeated(Repeat::Circular {
            count: 4.0.into(),
            radius: 15.0.into(),
            start_degrees: 45.0.into(),
        })],
    ));
    doc.with(Op::extrude(bolts, -1.0, 12.0, Mode::Remove));
    let grid = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::circle(93.0, -7.0, 1.0)).repeated(Repeat::Grid {
            nx: 2.0.into(),
            ny: 2.0.into(),
            dx: 14.0.into(),
            dy: 14.0.into(),
        })],
    ));
    doc.with(Op::extrude(grid, -1.0, 12.0, Mode::Remove));
    doc.with(Op::fillet(
        EdgeSelect::Any(vec![
            EdgeSelect::Between(e.end_cap(), e.region_side(1, 0)),
            EdgeSelect::Both(vec![EdgeSelect::Parallel(point3(1.0, 0.0, 0.0)), EdgeSelect::OnFace(e.end_cap())]),
        ]),
        0.5,
    ));
    doc.with(Op::Fillet {
        // The corner post above the ring, from z = 5.98 to the fillet at z = 8.5.
        edges: EdgeSelect::Near { point: point3("half", "half", 7.24), tolerance: 0.5.into() },
        radius: 1.0.into(),
        propagate: false,
    });
    doc.with(Op::chamfer(
        EdgeSelect::Any(vec![
            // The bottom edge of the second region, a sharp corner well away from the ring.
            EdgeSelect::InBox { min: point3(95.0, -11.0, 0.5), max: point3(105.0, -9.0, 1.5) },
            EdgeSelect::Radius(1000.0.into()),
        ]),
        0.25,
    ));
    let extra = doc.with(Op::solid(
        Primitive::Torus { major: 200.0.into(), minor: 3.0.into() },
        Placement::new(point3(0.0, 0.0, 50.0), point3(0.0, 0.0, 1.0), point3(1.0, 0.0, 0.0)),
        Mode::Add,
    ));
    doc.with(Op::solid(Primitive::Cone { r0: 4.0.into(), r1: 2.0.into(), height: 5.0.into() }, Placement::at(point3(0.0, 300.0, 0.0)), Mode::Add));
    doc.with(Op::solid(Primitive::Sphere { radius: 2.0.into() }, Placement::at(point3(0.0, 400.0, 0.0)), Mode::Add));
    doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 2.0.into() }, Placement::at(point3(0.0, 500.0, 0.0)), Mode::Add));
    doc.with(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 2.0.into() }, Placement::at(point3(0.0, 600.0, 0.0)), Mode::Add));
    doc.rename(extra, "Ring \"quoted\" \u{e9}").unwrap();
    doc.suppress(extra, true).unwrap();
    doc
}

#[test]
fn save_and_load_round_trips_everything() {
    let mut doc = every_kind_of_feature();
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.parameters(), doc.parameters());
    assert_eq!(back.features(), doc.features());
    assert_eq!(back.save_text(), text, "saving is stable");
    // Same geometry, to the bit.
    let (a, b) = (doc.rebuild(None), back.rebuild(None));
    assert!(a.failed.is_none(), "{:?}", a.steps);
    assert!(b.failed.is_none(), "{:?}", b.steps);
    let (a, b) = (a.body.unwrap(), b.body.unwrap());
    assert_eq!(mass_properties(&a).volume.to_bits(), mass_properties(&b).volume.to_bits());
    assert_eq!(a.faces.len(), b.faces.len());
    assert_eq!(face_names(&a), face_names(&b));
}

#[test]
fn numbers_in_a_saved_file_keep_their_exact_value() {
    let tricky = [0.1 + 0.2, 1.0 / 3.0, 1e-9, 123456.7890123, f64::MIN_POSITIVE, 1e300];
    let mut doc = Document::new();
    for (i, v) in tricky.iter().enumerate() {
        doc.set_parameter(&format!("p{i}"), *v).unwrap();
    }
    let back = Document::load_text(&doc.save_text()).unwrap();
    let env = back.environment().unwrap();
    for (i, v) in tricky.iter().enumerate() {
        assert_eq!(env[&format!("p{i}")].to_bits(), v.to_bits(), "{v}");
    }
}

#[test]
fn a_hand_written_file_with_defaults_loads() {
    let text = r#"{
        "format": "coreshape-document",
        "version": 1,
        "parameters": [{"name": "size", "value": 30}, {"name": "h", "value": "size / 3"}],
        "features": [
            {"id": 1, "name": "Outline", "type": "sketch",
             "regions": [{"outer": {"type": "rect", "w": "size", "h": "size"}}]},
            {"id": 2, "name": "Plate", "type": "extrude", "sketch": 1, "length": "h"},
            {"id": 7, "name": "Round", "type": "fillet", "edges": {"on_face": "f2:end"}, "radius": 1}
        ]
    }"#;
    let mut doc = Document::load_text(text).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!(vol(&o) < 9000.0 && vol(&o) > 8800.0, "{}", vol(&o));
    // The next id goes after the highest one seen.
    let id = doc.add(Op::solid(Primitive::Sphere { radius: 1.0.into() }, Placement::xy(100.0), Mode::Add)).unwrap();
    assert_eq!(id.0, 8);
}

#[test]
fn bad_files_are_refused_with_a_reason() {
    let base = |features: &str| format!("{{\"format\":\"coreshape-document\",\"version\":1,\"features\":[{features}]}}");
    let sketch = r#"{"id":1,"name":"S","type":"sketch","regions":[{"outer":{"type":"circle","r":1}}]}"#;
    let cases: Vec<(String, &str)> = vec![
        ("{}".into(), "not a CoreShape document"),
        ("not json".into(), "not valid JSON"),
        ("{\"format\":\"coreshape-document\",\"version\":2}".into(), "version"),
        ("{\"format\":\"coreshape-document\",\"version\":1,\"units\":\"in\"}".into(), "units"),
        ("{\"format\":\"coreshape-document\",\"version\":1,\"extra\":1}".into(), "unknown field"),
        (base(r#"{"id":1,"name":"X","type":"warp"}"#), "unknown feature type"),
        (base(r#"{"id":1,"name":"X","type":"extrude","sketch":1,"length":1}"#), "doesn't come before"),
        (base(&format!(r#"{sketch},{{"id":1,"name":"Y","type":"solid","shape":"sphere","radius":1}}"#)), "appears twice"),
        (base(&format!(r#"{sketch},{{"id":2,"name":"E","type":"extrude","sketch":1,"length":1,"mode":"sideways"}}"#)), "unknown mode"),
        (base(&format!(r#"{sketch},{{"id":2,"name":"E","type":"extrude","sketch":1,"length":"1 +"}}"#)), "expression"),
        (base(&format!(r#"{sketch},{{"id":2,"name":"E","type":"extrude","sketch":1,"length":1,"typo":3}}"#)), "unknown field"),
        (base(&format!(r#"{sketch},{{"id":2,"name":"F","type":"fillet","edges":{{"on_face":"nonsense"}},"radius":1}}"#)), "not a face name"),
        (base(r#"{"id":1,"name":"F","type":"fillet","edges":{"on_face":"f9:end"},"radius":1}"#), "doesn't come before"),
        (base(r#"{"id":0,"name":"S","type":"sketch","regions":[]}"#), "ids start at 1"),
        (base(r#"{"id":1.5,"name":"S","type":"sketch","regions":[]}"#), "whole number"),
        ("{\"format\":\"coreshape-document\",\"version\":1,\"parameters\":[{\"name\":\"a\",\"value\":\"a\"}]}".into(), "cycle"),
    ];
    for (text, expected) in cases {
        match Document::load_text(&text) {
            Ok(_) => panic!("should have refused {text}"),
            Err(e) => assert!(e.to_string().contains(expected), "{text}\n  said: {e}\n  wanted: {expected}"),
        }
    }
}

#[test]
fn face_names_round_trip() {
    let id = FeatureId(12);
    for r in [
        id.start_cap(),
        id.end_cap(),
        id.side(0),
        id.side(999),
        id.hole_side(0, 3),
        id.hole_side(998, 999),
        id.region_side(1, 0),
        id.region_side(3999, 999),
        FaceRef { feature: id, slot: id.region_side(5, 7).slot + id.hole_side(2, 0).slot - id.side(0).slot },
    ] {
        let name = r.name();
        assert_eq!(FaceRef::parse(&name).unwrap(), r, "{name}");
    }
    assert_eq!(id.end_cap().name(), "f12:end");
    assert_eq!(id.side(3).name(), "f12:side.3");
    assert_eq!(id.hole_side(1, 2).name(), "f12:hole.1.side.2");
    assert_eq!(id.region_side(2, 1).name(), "f12:region.2.side.1");
    for bad in ["", "f12", "12:end", "f12:", "f12:middle", "f12:side", "f12:side.x", "fx:end", "f12:side.1000", "f12:hole.1", "f12:side.1.side", "f12:region.9999.side.0", "f99999999999:end"] {
        assert!(FaceRef::parse(bad).is_err(), "{bad:?}");
    }
}

#[test]
fn every_selection_works_through_json() {
    let list = vec![
        EdgeSelect::All,
        EdgeSelect::Between(FeatureId(2).end_cap(), FeatureId(2).side(1)),
        EdgeSelect::OnFace(FeatureId(3).hole_side(0, 0)),
        EdgeSelect::Near { point: point3(1.0, "w", 0.5), tolerance: 0.25.into() },
        EdgeSelect::InBox { min: point3(0.0, 0.0, 0.0), max: point3(1.0, 1.0, "h") },
        EdgeSelect::Parallel(point3(0.0, 1.0, 0.0)),
        EdgeSelect::Radius("r / 2".into()),
        EdgeSelect::Any(vec![EdgeSelect::All, EdgeSelect::Both(vec![EdgeSelect::Radius(1.0.into())])]),
    ];
    for q in list {
        let text = edge_select_value(&q).to_text();
        let back = read_edge_select(&json::parse(&text).unwrap()).unwrap();
        assert_eq!(back, q, "{text}");
    }
}

/// Random damage to a valid file must never panic, whether it loads or rebuilds or not.
#[test]
fn damaged_files_never_panic() {
    let text = every_kind_of_feature().to_json().to_text();
    let mut rng = Rng::new(404);
    let numbers = ["0", "-1", "1e308", "1e-308", "0.5", "1000000", "3", "\"w\"", "\"1/0\"", "null", "[]", "{}", "true"];
    let mut loaded = 0;
    let mut built = 0;
    for _ in 0..400 {
        let mut s = text.clone().into_bytes();
        match rng.int(0, 2) {
            // A byte changed.
            0 => {
                for _ in 0..rng.int(1, 4) {
                    let at = rng.int(0, s.len() as i64 - 1) as usize;
                    s[at] = rng.int(32, 126) as u8;
                }
            }
            // A chunk removed.
            1 => {
                let at = rng.int(0, s.len() as i64 - 2) as usize;
                let n = rng.int(1, 40) as usize;
                s.drain(at..(at + n).min(s.len()));
            }
            // One number replaced by something awkward.
            _ => {
                let t = String::from_utf8_lossy(&s).to_string();
                let spots: Vec<usize> = t
                    .char_indices()
                    .filter(|(i, c)| c.is_ascii_digit() && t.as_bytes()[*i - 1] == b':')
                    .map(|(i, _)| i)
                    .collect();
                let at = spots[rng.int(0, spots.len() as i64 - 1) as usize];
                let end = at + t[at..].find(|c: char| !(c.is_ascii_digit() || c == '.' || c == 'e' || c == '-')).unwrap_or(0);
                let new = numbers[rng.int(0, numbers.len() as i64 - 1) as usize];
                s = format!("{}{}{}", &t[..at], new, &t[end..]).into_bytes();
            }
        }
        let damaged = String::from_utf8_lossy(&s);
        if let Ok(mut doc) = Document::load_text(&damaged) {
            loaded += 1;
            let o = doc.rebuild(None);
            if let Some(b) = &o.body {
                built += 1;
                validate(b).unwrap_or_else(|e| panic!("an invalid body came back: {e:?}\nfile: {damaged}"));
            }
            let _ = doc.save_text();
        }
    }
    assert!(loaded > 20, "only {loaded} damaged files loaded, so this tested little");
    assert!(built > 10, "only {built} damaged files built a body");
}

// ----- Sweeps, patterns and mirrors -----

fn plate_with_hole() -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 100.0, 40.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let hole = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(10.0, 20.0, 3.0))]));
    let drill = doc.with(Op::extrude(hole, -1.0, 7.0, Mode::Remove));
    (doc, e, drill)
}

#[test]
fn a_linear_pattern_of_a_cut_makes_a_row_of_holes() {
    let (mut doc, _, drill) = plate_with_hole();
    doc.set_parameter("n", 5.0).unwrap();
    doc.with(Op::Pattern {
        of: vec![drill],
        along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: "n".into(), spacing: 20.0.into() },
    });
    for n in [5.0, 2.0, 4.0] {
        doc.set_parameter("n", n).unwrap();
        let o = doc.rebuild(None);
        assert!(o.failed.is_none(), "{:?}", o.steps);
        assert!((vol(&o) - (20000.0 - n * PI * 9.0 * 5.0)).abs() < 1e-7, "{n}: {}", vol(&o));
        assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![n as usize]);
    }
}

#[test]
fn a_circular_pattern_goes_around() {
    let mut doc = Document::new();
    let d = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 30.0))]));
    doc.with(Op::extrude(d, 0.0, 4.0, Mode::Add));
    let h = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(20.0, 0.0, 2.0))]));
    let drill = doc.with(Op::extrude(h, -1.0, 6.0, Mode::Remove));
    doc.with(Op::Pattern {
        of: vec![drill],
        along: PatternPath::Circular { center: point3(0.0, 0.0, 0.0), axis: point3(0.0, 0.0, 1.0), count: 8.0.into(), degrees: 360.0.into() },
    });
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - (PI * 900.0 * 4.0 - 8.0 * PI * 4.0 * 4.0)).abs() < 1e-6, "{}", vol(&o));
    assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![8]);
}

#[test]
fn a_partial_circular_pattern_ends_at_its_angle() {
    let mut doc = Document::new();
    let d = doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 2.0.into() }, Placement::at(point3(20.0, -1.0, 0.0)), Mode::Add));
    doc.with(Op::Pattern {
        of: vec![d],
        along: PatternPath::Circular { center: point3(0.0, 0.0, 0.0), axis: point3(0.0, 0.0, 1.0), count: 3.0.into(), degrees: 90.0.into() },
    });
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 24.0).abs() < 1e-9);
    // The last copy is a quarter turn round, so it reaches along +y.
    let b = o.body.unwrap().aabb();
    assert!((b.max.y - 22.0).abs() < 1e-6, "{b:?}");
}

#[test]
fn a_whole_body_can_be_patterned_and_copies_keep_their_names() {
    let (mut doc, e, _) = plate_with_hole();
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), 1.0));
    let corner = 1.0 - PI / 4.0;
    let one = 20000.0 - PI * 9.0 * 5.0 - corner * 100.0;
    doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 3.0.into(), spacing: 50.0.into() } });
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 3.0 * one).abs() < 1e-6, "{}", vol(&o));
    let body = o.body.unwrap();
    assert_eq!(validate(&body).unwrap().shells, 3);
    // One name covers the side in all three copies, so a selection by name reaches them all.
    let named = body.faces.iter().filter(|f| f.tag == e.end_cap().tag()).count();
    assert_eq!(named, 3);
    let env = doc.environment().unwrap();
    let edges = select_edges(&EdgeSelect::Between(e.end_cap(), e.side(1)), &body, &env).unwrap();
    assert_eq!(edges.len(), 3);
}

#[test]
fn a_mirror_doubles_a_half_and_a_listed_feature_alone() {
    // A half plate mirrored whole: the shared face goes, leaving one solid.
    let (mut doc, _, _) = plate_with_hole();
    doc.with(Op::Mirror { of: vec![], point: point3(0.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 2.0 * (20000.0 - PI * 9.0 * 5.0)).abs() < 1e-6, "{}", vol(&o));
    let body = o.body.unwrap();
    assert_eq!(validate(&body).unwrap().shells, 1);
    assert_eq!(validate(&body).unwrap().genus, vec![2]);
    // Only the hole, mirrored about the plate's middle.
    let (mut doc, _, drill) = plate_with_hole();
    doc.with(Op::Mirror { of: vec![drill], point: point3(50.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - (20000.0 - 2.0 * PI * 9.0 * 5.0)).abs() < 1e-7, "{}", vol(&o));
}

#[test]
fn a_sweep_feature_follows_its_path_and_saves() {
    let mut doc = Document::new();
    doc.set_parameter("bend", 12.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 3.0).clone()).hole(Shape::circle(0.0, 0.0, 2.0))]));
    let sw = doc.with(Op::Sweep {
        sketch: s,
        path: vec![
            SweepStep::Line { length: 10.0.into() },
            SweepStep::Arc { radius: "bend".into(), degrees: 90.0.into(), toward_degrees: 0.0.into() },
            SweepStep::Line { length: 10.0.into() },
        ],
        mode: Mode::Add,
    });
    let want = |bend: f64| PI * 5.0 * (20.0 + bend * PI / 2.0);
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - want(12.0)).abs() < 1e-6, "{}", vol(&o));
    let names = face_names(o.body.as_ref().unwrap());
    assert!(names.contains(&sw.side(0).name()) && names.contains(&sw.start_cap().name()) && names.contains(&sw.end_cap().name()), "{names:?}");
    doc.set_parameter("bend", 20.0).unwrap();
    let o = doc.rebuild(None);
    assert!((vol(&o) - want(20.0)).abs() < 1e-6, "{}", vol(&o));
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.features(), doc.features());
    assert_eq!(back.save_text(), text);
    assert_eq!(vol(&back.rebuild(None)).to_bits(), vol(&doc.rebuild(None)).to_bits());
}

#[test]
fn patterns_and_mirrors_save_and_refuse_with_reasons() {
    let (mut doc, _, drill) = plate_with_hole();
    doc.with(Op::Pattern { of: vec![drill], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 2.0.into(), spacing: 20.0.into() } });
    doc.with(Op::Pattern { of: vec![drill], along: PatternPath::Circular { center: point3(50.0, 20.0, 0.0), axis: point3(0.0, 0.0, 1.0), count: 2.0.into(), degrees: 360.0.into() } });
    doc.with(Op::Mirror { of: vec![], point: point3(0.0, 0.0, 5.0), normal: point3(0.0, 0.0, 1.0) });
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.features(), doc.features());
    assert_eq!(back.save_text(), text);
    let o = back.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!(back.delete(drill).is_err(), "a pattern refers to it");
    // A copied feature has to make a solid, come first and be switched on.
    let mut d = Document::new();
    let s = d.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 5.0, 5.0))]));
    let x = d.with(Op::extrude(s, 0.0, 1.0, Mode::Add));
    let f = d.with(Op::fillet(EdgeSelect::OnFace(x.end_cap()), 0.2));
    let p = d.with(Op::Pattern { of: vec![f], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 2.0.into(), spacing: 9.0.into() } });
    let o = d.rebuild(None);
    match &o.steps[3].state {
        StepState::Failed(why) => assert!(why.contains("no solid to copy"), "{why}"),
        other => panic!("{other:?}"),
    }
    d.edit(p, Op::Pattern { of: vec![x], along: PatternPath::Linear { direction: point3(0.0, 0.0, 0.0), count: 2.0.into(), spacing: 9.0.into() } }).unwrap();
    match &d.rebuild(None).steps[3].state {
        StepState::Failed(why) => assert!(why.contains("non-zero"), "{why}"),
        other => panic!("{other:?}"),
    }
    d.edit(p, Op::Pattern { of: vec![x], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 0.0.into(), spacing: 9.0.into() } }).unwrap();
    assert!(matches!(d.rebuild(None).steps[3].state, StepState::Failed(_)));
    d.suppress(x, true).unwrap();
    d.edit(p, Op::Pattern { of: vec![x], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 2.0.into(), spacing: 9.0.into() } }).unwrap();
    assert!(d.rebuild(None).failed.is_some());
    let mut late = Document::new();
    assert!(late.add(Op::Mirror { of: vec![FeatureId(5)], point: point3(0.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) }).is_err());
    let m = late.with(Op::Mirror { of: vec![], point: point3(0.0, 0.0, 0.0), normal: point3(0.0, 0.0, 0.0) });
    let _ = m;
    assert!(late.rebuild(None).failed.is_some());
}

// ----- Offset, shell and draft -----

#[test]
fn offset_shell_and_draft_work_as_features_and_follow_dimensions() {
    let mut doc = Document::new();
    doc.set_parameter("w", 40.0).unwrap();
    doc.set_parameter("wall", 2.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("w", "30"))]));
    let e = doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
    let tray = doc.with(Op::Shell { open: vec![e.end_cap()], thickness: "wall".into() });
    let want = |w: f64, t: f64| w * 30.0 * 20.0 - (w - 2.0 * t) * (30.0 - 2.0 * t) * (20.0 - t);
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - want(40.0, 2.0)).abs() < 1e-8, "{}", vol(&o));
    // The inner floor has a name of its own, and a fillet can find it.
    let body = o.body.unwrap();
    assert!(body.faces.iter().any(|f| f.tag == tray.start_cap().tag()), "{:?}", face_names(&body));
    // The opened face remains as the rim around the opening, one loop inside another.
    let rim = body.faces.iter().find(|f| f.tag == e.end_cap().tag()).expect("the rim");
    assert_eq!(rim.loops.len(), 2);
    for (w, t) in [(60.0, 2.0), (60.0, 3.5)] {
        doc.set_parameter("w", w).unwrap();
        doc.set_parameter("wall", t).unwrap();
        let o = doc.rebuild(None);
        assert!(o.failed.is_none(), "{:?}", o.steps);
        assert!((vol(&o) - want(w, t)).abs() < 1e-8, "{}", vol(&o));
    }
    // Walls too thick for the part are refused, with the last good solid kept.
    doc.set_parameter("wall", 16.0).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(2), "{:?}", o.steps);
    assert!(o.body.is_some());
    doc.set_parameter("wall", 2.0).unwrap();

    let mut d = Document::new();
    let s = d.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("20", "20"))]));
    let e = d.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let sides: Vec<FaceRef> = (0..4).map(|i| e.side(i)).collect();
    d.with(Op::Draft { faces: sides, pull: point3(0.0, 0.0, 1.0), neutral: point3(0.0, 0.0, 0.0), degrees: 5.0.into() });
    d.with(Op::Offset { distance: 1.0.into() });
    let o = d.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let text = d.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    assert_eq!(vol(&back.rebuild(None)).to_bits(), vol(&d.rebuild(None)).to_bits());
    // A name that's gone says so.
    let mut d = Document::new();
    let s = d.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("20", "20"))]));
    let e = d.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    d.with(Op::Shell { open: vec![e.side(99)], thickness: 1.0.into() });
    match &d.rebuild(None).steps[2].state {
        StepState::Failed(why) => assert!(why.contains("no longer exists"), "{why}"),
        other => panic!("{other:?}"),
    }
}
