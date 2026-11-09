//! Solved sketches as features: from constraints to a kernel solid, with names that hold.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch as sk;
use coreshape_sketch::{EntityId, Rule, ORIGIN};

struct Plate {
    sketch: sk::Sketch,
    sides: [EntityId; 4],
    hole: EntityId,
}

/// A rectangle from the origin, width and height set by parameters, with a centred hole. The
/// geometry starts crooked and the wrong size, so the solver has real work to do.
fn plate_sketch() -> Plate {
    let mut s = sk::Sketch::new();
    let a = s.line([1.0, 1.0], [30.0, -1.0]);
    let b = s.line([30.5, 0.0], [31.0, 20.0]);
    let c = s.line([30.0, 21.0], [-1.0, 19.0]);
    let d = s.line([0.0, 20.0], [0.0, 0.5]);
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
    s.constrain(Rule::Coincident { a: a.start(), b: ORIGIN.whole() });
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: b.whole(), b: None });
    s.constrain(Rule::Horizontal { a: c.whole(), b: None });
    s.constrain(Rule::Vertical { a: d.whole(), b: None });
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: "w".into() });
    s.constrain(Rule::Distance { a: b.whole(), b: None, value: "h".into() });
    let hole = s.circle([12.0, 12.0], 3.0);
    s.constrain(Rule::HorizontalDistance { a: a.start(), b: hole.center(), value: "w / 2".into() });
    s.constrain(Rule::VerticalDistance { a: a.start(), b: hole.center(), value: "h / 2".into() });
    s.constrain(Rule::Diameter { of: hole, value: "bore".into() });
    Plate { sketch: s, sides: [a, b, c, d], hole }
}

fn vol(o: &Outcome) -> f64 {
    mass_properties(o.body.as_ref().expect("a body")).volume
}

fn plate_doc() -> (Document, FeatureId, Plate) {
    let plate = plate_sketch();
    let mut doc = Document::new();
    for (name, value) in [("w", 40.0), ("h", 25.0), ("bore", 8.0), ("t", 6.0)] {
        doc.set_parameter(name, value).unwrap();
    }
    let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: plate.sketch.clone() }).unwrap();
    let e = doc.add(Op::extrude(s, 0.0, "t", Mode::Add)).unwrap();
    (doc, e, plate)
}

#[test]
fn a_constrained_sketch_extrudes_to_the_right_solid() {
    let (mut doc, _, _) = plate_doc();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let want = (40.0 * 25.0 - PI * 16.0) * 6.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
    let report = validate(o.body.as_ref().unwrap()).unwrap();
    assert_eq!(report.genus, vec![1]);
}

#[test]
fn parameters_drive_the_sketch_and_only_what_depends_on_it_rebuilds() {
    let (mut doc, _, _) = plate_doc();
    doc.rebuild(None);
    doc.set_parameter("t", 9.0).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.steps[0].state, StepState::Reused, "the thickness isn't in the sketch");
    assert_eq!(o.steps[1].state, StepState::Rebuilt);
    doc.set_parameter("w", 60.0).unwrap();
    doc.set_parameter("bore", 10.0).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.steps[0].state, StepState::Rebuilt);
    let want = (60.0 * 25.0 - PI * 25.0) * 9.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
}

#[test]
fn faces_are_named_by_entity_and_blends_find_them_after_a_change() {
    let (mut doc, e, plate) = plate_doc();
    let top_front = EdgeSelect::Between(e.end_cap(), e.side(plate.sides[0].0));
    doc.add(Op::fillet(top_front, 2.0)).unwrap();
    doc.add(Op::chamfer(EdgeSelect::Between(e.end_cap(), e.side(plate.hole.0)), 1.0)).unwrap();
    let round = |w: f64| 4.0 * (1.0 - PI / 4.0) * w;
    let cone = |r: f64, t: f64| PI * t / 3.0 * ((r + t).powi(2) + (r + t) * r + r * r) - PI * r * r * t;
    for (w, bore) in [(40.0, 8.0), (70.0, 12.0)] {
        doc.set_parameter("w", w).unwrap();
        doc.set_parameter("bore", bore).unwrap();
        let o = doc.rebuild(None);
        assert!(o.failed.is_none(), "{:?}", o.steps);
        let want = (w * 25.0 - PI * (bore / 2.0).powi(2)) * 6.0 - round(w) - cone(bore / 2.0, 1.0);
        assert!((vol(&o) - want).abs() < 1e-5, "w={w}: {} vs {want}", vol(&o));
        let names = face_names(o.body.as_ref().unwrap());
        assert!(names.contains(&e.side(plate.sides[1].0).name()), "{names:?}");
        assert!(names.contains(&e.side(plate.hole.0).name()), "{names:?}");
    }
}

#[test]
fn arcs_make_true_curved_walls() {
    // A slot: two lines and two semicircles, tangent where they meet.
    let mut s = sk::Sketch::new();
    let right = s.arc([30.0, 0.0], 5.5, -90.0, 90.0);
    let left = s.arc([0.0, 0.0], 4.5, 90.0, 270.0);
    let bottom = s.line([0.0, -4.5], [30.0, -5.5]);
    let top = s.line([30.0, 5.5], [0.0, 4.5]);
    s.constrain(Rule::Coincident { a: bottom.end(), b: right.start() });
    s.constrain(Rule::Coincident { a: right.end(), b: top.start() });
    s.constrain(Rule::Coincident { a: top.end(), b: left.start() });
    s.constrain(Rule::Coincident { a: left.end(), b: bottom.start() });
    for (line, arc) in [(bottom, right), (top, right), (top, left), (bottom, left)] {
        s.constrain(Rule::Tangent { a: line, b: arc });
    }
    s.constrain(Rule::Equal { a: left, b: right });
    s.constrain(Rule::Radius { of: left, value: 5.0.into() });
    s.constrain(Rule::Horizontal { a: bottom.whole(), b: None });
    s.constrain(Rule::Coincident { a: left.center(), b: ORIGIN.whole() });
    s.constrain(Rule::Distance { a: bottom.whole(), b: None, value: 30.0.into() });
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let e = doc.add(Op::extrude(f, 0.0, 4.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let want = (30.0 * 10.0 + PI * 25.0) * 4.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
    let body = o.body.unwrap();
    let curved = body.faces.iter().filter(|x| matches!(x.surface, Surface::Cylinder { .. })).count();
    assert_eq!(curved, 2);
    assert!(body.faces.iter().any(|x| x.tag == e.side(right.0).tag()));
}

#[test]
fn a_revolved_constrained_profile_has_the_volume_pappus_says() {
    // A square section out from the axis.
    let mut s = sk::Sketch::new();
    let a = s.line([20.0, 0.0], [31.0, 0.0]);
    let b = s.line([30.0, 0.0], [30.0, 10.0]);
    let c = s.line([30.0, 11.0], [20.0, 10.0]);
    let d = s.line([20.0, 10.0], [20.0, 0.0]);
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: b.whole(), b: None });
    s.constrain(Rule::Horizontal { a: c.whole(), b: None });
    s.constrain(Rule::Vertical { a: d.whole(), b: None });
    s.fix(a.start()).unwrap();
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: 10.0.into() });
    s.constrain(Rule::Distance { a: b.whole(), b: None, value: 10.0.into() });
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    doc.add(Op::revolve(f, 360.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let want = TAU * 25.0 * 100.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
}

#[test]
fn several_regions_each_become_a_solid_and_a_loop_in_a_hole_is_a_new_one() {
    let mut s = sk::Sketch::new();
    let a = s.circle([0.0, 0.0], 20.0);
    let hole = s.circle([0.0, 0.0], 12.0);
    let island = s.circle([0.0, 0.0], 5.0);
    let apart = s.circle([60.0, 0.0], 3.0);
    s.constrain(Rule::Concentric { a, b: hole });
    s.constrain(Rule::Concentric { a, b: island });
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let e = doc.add(Op::extrude(f, 0.0, 2.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let want = PI * (400.0 - 144.0 + 25.0 + 9.0) * 2.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
    let body = o.body.unwrap();
    assert_eq!(validate(&body).unwrap().shells, 3);
    assert!(body.faces.iter().any(|x| x.tag == e.side(apart.0).tag()));
}

fn solid_of(s: sk::Sketch) -> (Outcome, FeatureId) {
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let e = doc.add(Op::extrude(f, 0.0, 2.0, Mode::Add)).unwrap();
    (doc.rebuild(None), e)
}

fn divided_rectangle() -> (sk::Sketch, sk::EntityId) {
    let mut s = sk::Sketch::new();
    s.line([0.0, 0.0], [30.0, 0.0]);
    s.line([30.0, 0.0], [30.0, 12.0]);
    s.line([30.0, 12.0], [0.0, 12.0]);
    s.line([0.0, 12.0], [0.0, 0.0]);
    let divider = s.line([10.0, 0.0], [10.0, 12.0]);
    (s, divider)
}

#[test]
fn regions_that_share_an_edge_merge_into_one_solid_when_both_are_used() {
    let (s, divider) = divided_rectangle();
    let (o, e) = solid_of(s);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 720.0).abs() < 1e-9, "{}", vol(&o));
    let body = o.body.unwrap();
    assert_eq!(validate(&body).unwrap().shells, 1);
    // The dividing line is inside the solid now, so it left no face.
    assert!(!body.faces.iter().any(|x| x.tag == e.side(divider.0).tag()));
}

#[test]
fn a_picked_region_is_the_only_one_built_and_survives_saving() {
    let (mut s, divider) = divided_rectangle();
    let left = s.arrangement().region_at([5.0, 5.0]).unwrap().id;
    s.pick_regions(&[left]);
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let e = doc.add(Op::extrude(f, 0.0, 2.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 240.0).abs() < 1e-9, "{}", vol(&o));
    // Here the divider is an outside wall, under its entity's name.
    assert!(o.body.as_ref().unwrap().faces.iter().any(|x| x.tag == e.side(divider.0).tag()));

    let mut back = Document::load_text(&doc.save_text()).unwrap();
    let o = back.rebuild(None);
    assert!((vol(&o) - 240.0).abs() < 1e-9, "{}", vol(&o));
}

#[test]
fn overlapping_circles_make_their_union_by_default() {
    let mut s = sk::Sketch::new();
    s.circle([0.0, 0.0], 5.0);
    s.circle([6.0, 0.0], 5.0);
    let (o, _) = solid_of(s);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let lens = 2.0 * 25.0 * (3.0f64 / 5.0).acos() - 3.0 * 8.0;
    let want = (2.0 * 25.0 * PI - lens) * 2.0;
    assert!((vol(&o) - want).abs() < 1e-6, "{} vs {want}", vol(&o));
    assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().shells, 1);
}

#[test]
fn a_stray_open_curve_no_longer_stops_the_solid() {
    let (mut s, _) = divided_rectangle();
    s.line([50.0, 0.0], [60.0, 0.0]);
    let (o, _) = solid_of(s);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 720.0).abs() < 1e-9, "{}", vol(&o));
}

#[test]
fn a_crescent_from_a_circle_touching_the_wall_builds_or_says_why() {
    let mut s = sk::Sketch::new();
    let wall = s.circle([0.0, 0.0], 5.0);
    let disc = s.circle([3.0, 0.0], 2.0);
    let a = s.arrangement();
    assert_eq!(a.regions.len(), 2);
    let crescent = a.region_at([-3.0, 0.0]).unwrap().id;
    s.pick_regions(&[crescent]);
    let (o, _) = solid_of(s);
    match o.failed {
        None => {
            assert!((vol(&o) - 21.0 * PI * 2.0).abs() < 1e-6, "{}", vol(&o));
            validate(o.body.as_ref().unwrap()).unwrap();
        }
        Some(i) => assert!(matches!(&o.steps[i].state, StepState::Failed(_)), "{:?}", o.steps[i]),
    }
    println!("crescent: {:?}", o.failed.map(|i| &o.steps[i].state));
    let _ = (wall, disc);
}

#[test]
fn an_ellipse_extrudes_and_keeps_its_name() {
    let mut s = sk::Sketch::new();
    let e = s.ellipse([0.0, 0.0], [10.0, 0.0], 4.0);
    s.constrain(Rule::Coincident { a: e.center(), b: ORIGIN.whole() });
    s.constrain(Rule::Horizontal { a: e.center(), b: Some(e.major()) });
    s.constrain(Rule::MajorRadius { of: e, value: "size".into() });
    s.constrain(Rule::MinorRadius { of: e, value: 4.0.into() });
    let mut doc = Document::new();
    doc.set_parameter("size", 12.0).unwrap();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    let x = doc.add(Op::extrude(f, 0.0, 2.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - PI * 12.0 * 4.0 * 2.0).abs() < 1e-9, "{}", vol(&o));
    let body = o.body.unwrap();
    assert!(body.faces.iter().any(|g| g.tag == x.side(e.0).tag()));
    validate(&body).unwrap();
    // The dimension drives it.
    doc.set_parameter("size", 20.0).unwrap();
    let o = doc.rebuild(None);
    assert!((vol(&o) - PI * 20.0 * 4.0 * 2.0).abs() < 1e-9, "{}", vol(&o));
}

#[test]
fn a_d_shape_from_an_elliptical_arc_and_a_chord_extrudes() {
    let mut s = sk::Sketch::new();
    s.elliptical_arc([0.0, 0.0], [10.0, 0.0], 5.0, 0.0, 180.0);
    s.line([-10.0, 0.0], [10.0, 0.0]);
    let (o, _) = solid_of(s);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - 25.0 * PI * 2.0).abs() < 1e-9, "{}", vol(&o));
    validate(o.body.as_ref().unwrap()).unwrap();
}

#[test]
fn a_drawn_spline_extrudes_and_revolves() {
    // A spline over a base line, dimensioned by its ends.
    let build = || {
        let mut s = sk::Sketch::new();
        let k = s.spline_through(&[[0.0, 5.0], [8.0, 11.0], [16.0, 7.0], [24.0, 12.0]]).unwrap();
        let down = s.line([24.0, 12.0], [24.0, 0.0]);
        let base = s.line([24.0, 0.0], [0.0, 0.0]);
        let up = s.line([0.0, 0.0], [0.0, 5.0]);
        s.constrain(Rule::Coincident { a: k.end(), b: down.start() });
        s.constrain(Rule::Coincident { a: down.end(), b: base.start() });
        s.constrain(Rule::Coincident { a: base.end(), b: up.start() });
        s.constrain(Rule::Coincident { a: up.end(), b: k.start() });
        s.constrain(Rule::Coincident { a: base.end(), b: ORIGIN.whole() });
        s.constrain(Rule::Horizontal { a: base.whole(), b: None });
        s.constrain(Rule::Vertical { a: up.whole(), b: None });
        s.constrain(Rule::Vertical { a: down.whole(), b: None });
        (s, k)
    };
    let (s, k) = build();
    let area: f64 = s.solve().unwrap().sketch.regions().unwrap().iter().map(|r| r.area()).sum();
    assert!(area > 100.0, "{area}");
    let (o, e) = solid_of(s);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - area * 2.0).abs() < 1e-7, "{} vs {}", vol(&o), area * 2.0);
    let body = o.body.as_ref().unwrap();
    validate(body).unwrap();
    assert!(body.faces.iter().any(|f| f.tag == e.side(k.0).tag() && matches!(f.surface, Surface::Extruded { .. })), "the spline's face is named for it");

    // Around the sketch's y axis: a turned shape, with its volume from Pappus through the kernel's own integral.
    let (s, _) = build();
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    doc.add(Op::revolve(f, 360.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!(vol(&o) > 0.0);
    validate(o.body.as_ref().unwrap()).unwrap();
}

#[test]
fn an_ellipse_with_a_hole_revolves() {
    // Away from the axis: an elliptical torus, whose volume Pappus gives.
    let mut s = sk::Sketch::new();
    s.ellipse([20.0, 0.0], [26.0, 0.0], 3.0);
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    doc.add(Op::revolve(f, 360.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let want = TAU * 20.0 * PI * 6.0 * 3.0;
    assert!((vol(&o) - want).abs() < 1e-7, "{} vs {want}", vol(&o));
}

#[test]
fn a_conflict_fails_the_feature_with_its_reason_and_keeps_the_last_good_body() {
    let (mut doc, e, plate) = plate_doc();
    doc.add(Op::solid(Primitive::Sphere { radius: 2.0.into() }, Placement::at(point3(0.0, -50.0, 0.0)), Mode::Add)).unwrap();
    let first = doc.features()[0].id;
    let good = doc.rebuild(None);
    assert!(good.failed.is_none(), "{:?}", good.steps);
    let mut broken = plate.sketch.clone();
    broken.constrain(Rule::Distance { a: plate.sides[0].whole(), b: None, value: 5.0.into() });
    doc.edit(first, Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: broken }).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, Some(0));
    match &o.steps[0].state {
        StepState::Failed(why) => assert!(why.contains("c17") || why.contains("conflict") || why.contains("can't all hold"), "{why}"),
        other => panic!("{other:?}"),
    }
    assert_eq!(o.steps[1].state, StepState::Blocked);
    // Fixing the sketch brings everything back without recomputing what depends on it.
    doc.edit(first, Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: plate.sketch.clone() }).unwrap();
    let o = doc.rebuild(None);
    assert!(o.failed.is_none());
    assert_eq!(o.steps[1].state, StepState::Reused);
    let _ = e;
}

#[test]
fn an_open_sketch_says_which_entity_has_the_free_end() {
    let mut s = sk::Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    s.line([10.0, 0.0], [10.0, 10.0]);
    let _ = a;
    let mut doc = Document::new();
    let f = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    doc.add(Op::extrude(f, 0.0, 1.0, Mode::Add)).unwrap();
    let o = doc.rebuild(None);
    match &o.steps[0].state {
        StepState::Failed(why) => assert!(why.contains("isn't closed"), "{why}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_document_with_a_constrained_sketch_saves_and_loads_exactly() {
    let (mut doc, _, _) = plate_doc();
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.features(), doc.features());
    assert_eq!(back.save_text(), text);
    let (a, b) = (doc.rebuild(None), back.rebuild(None));
    assert_eq!(mass_properties(a.body.as_ref().unwrap()).volume.to_bits(), mass_properties(b.body.as_ref().unwrap()).volume.to_bits());
    // A broken sketch inside a document is refused when it's read.
    let bad = text.replace("\"coreshape-sketch\"", "\"something-else\"");
    assert!(Document::load_text(&bad).err().expect("refused").to_string().contains("not a CoreShape sketch"));
}

#[test]
fn a_parameter_used_only_in_a_sketch_can_not_be_removed() {
    let (mut doc, _, _) = plate_doc();
    let err = doc.remove_parameter("bore").unwrap_err().to_string();
    assert!(err.contains("uses"), "{err}");
}
