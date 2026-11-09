//! Things that modelling real parts turned up, each pinned down on its own. `crates/mcp/tests/real_parts` has the parts.

use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    o
}

fn vol(o: &Outcome) -> f64 {
    mass_properties(o.body.as_ref().unwrap()).volume
}

#[test]
fn a_slot_is_a_rounded_rectangle_whose_corners_meet() {
    let (w, h, t) = (8.0, 24.0, 3.0);
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rounded(0.0, 0.0, w, h, w / 2.0))]));
    doc.with(Op::extrude(s, 0.0, t, Mode::Add));
    let o = ok(&mut doc);
    let area = w * (h - w) + PI * (w / 2.0).powi(2);
    assert!((vol(&o) - area * t).abs() < 1e-9, "{}", vol(&o));
    validate(o.body.as_ref().unwrap()).unwrap();
    // Two straight walls, a quarter circle for each corner, and the caps.
    assert_eq!(o.body.as_ref().unwrap().faces.len(), 8);
}

#[test]
fn a_slot_the_other_way_and_a_square_with_round_corners_all_the_way_are_fine_too() {
    for (w, h, r, area) in [(24.0, 8.0, 4.0, 16.0 * 8.0 + PI * 16.0), (10.0, 10.0, 5.0, PI * 25.0)] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rounded(0.0, 0.0, w, h, r))]));
        doc.with(Op::extrude(s, 0.0, 2.0, Mode::Add));
        let o = ok(&mut doc);
        assert!((vol(&o) - area * 2.0).abs() < 1e-9, "{w} by {h}: {}", vol(&o));
        validate(o.body.as_ref().unwrap()).unwrap();
    }
}

#[test]
fn a_rounded_rectangles_sides_keep_their_names_as_the_radius_comes_up_to_a_slot() {
    let mut doc = Document::new();
    doc.set_parameter("r", 3.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rounded(0.0, 0.0, 8.0, 24.0, "r"))]));
    let e = doc.with(Op::extrude(s, 0.0, 3.0, Mode::Add));
    // A fillet by name on the side that stays, which is number 2 of bottom, corner, right, corner, top, corner, left and corner.
    let f = doc.with(Op::chamfer(EdgeSelect::Between(e.end_cap(), e.side(2)), 0.5));
    let long_side = |o: &Outcome| -> f64 {
        let b = o.body.as_ref().unwrap();
        let found = b.faces.iter().position(|x| x.tag == e.side(2).tag()).expect("the side's there");
        let a = coreshape::mass_properties(b).area;
        assert!(a > 0.0);
        let (lo, hi) = (b.face_aabb(found).min, b.face_aabb(found).max);
        hi.y - lo.y
    };
    let o = ok(&mut doc);
    assert!((long_side(&o) - 18.0).abs() < 1e-6, "{}", long_side(&o));
    assert!(o.body.as_ref().unwrap().faces.iter().any(|x| x.tag.feature == f.0));
    doc.set_parameter("r", 4.0).unwrap();
    let o = ok(&mut doc);
    assert!((long_side(&o) - 16.0).abs() < 1e-6, "{}", long_side(&o));
    assert!(o.body.as_ref().unwrap().faces.iter().any(|x| x.tag.feature == f.0));
}

/// A plate, with a hole through it, patterned along X and then that pattern and the hole along Y.
fn grid_of_holes(nx: f64, ny: f64) -> (Document, FeatureId, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 100.0, 100.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let c = doc.with(Op::sketch(Placement::xy(-1.0), vec![Region::new(Shape::circle(10.0, 10.0, 2.0))]));
    let hole = doc.with(Op::extrude(c, 0.0, 7.0, Mode::Remove));
    let across = doc.with(Op::Pattern { of: vec![hole], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: nx.into(), spacing: 20.0.into() } });
    let up = doc.with(Op::Pattern { of: vec![hole, across], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: ny.into(), spacing: 20.0.into() } });
    (doc, hole, across, up)
}

#[test]
fn a_pattern_can_copy_a_pattern() {
    let (mut doc, ..) = grid_of_holes(3.0, 2.0);
    let o = ok(&mut doc);
    let holes = 6.0;
    assert!((vol(&o) - (100.0 * 100.0 * 5.0 - holes * PI * 4.0 * 5.0)).abs() < 1e-8, "{}", vol(&o));
    assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![6]);
    // And its numbers can change.
    let (mut doc, ..) = grid_of_holes(4.0, 3.0);
    let o = ok(&mut doc);
    assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![12]);
}

#[test]
fn a_pattern_can_copy_a_mirror_and_one_of_new_bodies_makes_new_bodies() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(5.0, 0.0, 2.0, 2.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 2.0, Mode::NewBody));
    let m = doc.with(Op::Mirror { of: vec![e], point: point3(0.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    doc.with(Op::Pattern { of: vec![e, m], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 3.0.into(), spacing: 10.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 6);
    assert!((vol(&o) - 6.0 * 8.0).abs() < 1e-9, "{}", vol(&o));
}

#[test]
fn a_pattern_of_a_cut_and_an_add_together_can_be_made_but_not_copied_again() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 100.0, 100.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let c = doc.with(Op::sketch(Placement::xy(-1.0), vec![Region::new(Shape::circle(10.0, 10.0, 2.0))]));
    let hole = doc.with(Op::extrude(c, 0.0, 7.0, Mode::Remove));
    let b = doc.with(Op::sketch(Placement::xy(5.0), vec![Region::new(Shape::circle(30.0, 10.0, 2.0))]));
    let boss = doc.with(Op::extrude(b, 0.0, 3.0, Mode::Add));
    let both = doc.with(Op::Pattern { of: vec![hole, boss], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 2.0.into(), spacing: 20.0.into() } });
    ok(&mut doc);
    doc.with(Op::Pattern { of: vec![both], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 2.0.into(), spacing: 50.0.into() } });
    let o = doc.rebuild(None);
    assert!(o.failed.is_some());
    let StepState::Failed(why) = &o.steps[o.failed.unwrap()].state else { panic!() };
    assert!(why.contains("no solid to copy"), "{why}");
}

#[test]
fn a_radius_a_hair_from_half_the_width_is_a_slot_too() {
    // As an expression can leave it, by rounding, or a little over by a slider.
    for r in [4.0 + 2e-7, 4.0 - 2e-7, 4.0 + 1e-13, 4.0 - 1e-13] {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rounded(0.0, 0.0, 8.0, 24.0, r))]));
        doc.with(Op::extrude(s, 0.0, 3.0, Mode::Add));
        let o = ok(&mut doc);
        let want = (8.0 * 16.0 + PI * 16.0) * 3.0;
        assert!((vol(&o) - want).abs() < 1e-5, "{r}: {}", vol(&o));
        assert_eq!(o.body.as_ref().unwrap().faces.len(), 8, "{r}");
    }
}
