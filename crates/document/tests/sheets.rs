//! Sheets in a document: made by features, saved and read back, acted on when they're named, and left out of the way of
//! everything that needs a solid.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::{Geometry, Sketch};
use std::f64::consts::{PI, TAU};

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    o
}

fn fails(doc: &mut Document) -> String {
    let o = doc.rebuild(None);
    let i = o.failed.unwrap_or_else(|| panic!("this should have failed: {:?}", o.steps));
    match &o.steps[i].state {
        StepState::Failed(why) => why.clone(),
        other => panic!("{other:?}"),
    }
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn area_of(o: &Outcome, i: usize) -> f64 {
    mass_properties(&o.bodies[i].body).area
}

/// A sketch on the XY plane with one rectangle.
fn rect_sketch(doc: &mut Document, w: f64, h: f64) -> FeatureId {
    doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, w, h))]))
}

/// A sketch holding one open L, 10 along x and then 7 up y.
fn open_l(doc: &mut Document) -> FeatureId {
    let path = Shape::OpenPath { start: point(0.0, 0.0), moves: vec![Move::Line { to: point(10.0, 0.0) }, Move::Line { to: point(10.0, 7.0) }] };
    doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(path)]))
}

fn surface_extrude(sketch: FeatureId, length: impl Into<Expr>) -> Op {
    Op::extrude(sketch, 0.0, length, Mode::Surface)
}

// ----- Making them -----

#[test]
fn a_fill_is_a_flat_sheet_named_as_its_end() {
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 20.0);
    let f = doc.with(Op::Fill { sketch: s });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!(o.bodies[0].body.sheet);
    near(area_of(&o, 0), 200.0, "area");
    assert_eq!(mass_properties(&o.bodies[0].body).volume, 0.0);
    assert_eq!(find_faces(&o.bodies[0].body, &FaceKey::plain(f.end_cap())).len(), 1);
    // It follows its parameters.
    doc.set_parameter("w", 30.0).unwrap();
    let edited = rect_sketch_with(&mut doc);
    doc.edit(f, Op::Fill { sketch: edited }).unwrap_err();
}

fn rect_sketch_with(doc: &mut Document) -> FeatureId {
    doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 5.0))]))
}

#[test]
fn a_fill_follows_a_parameter() {
    let mut doc = Document::new();
    doc.set_parameter("w", 30.0).unwrap();
    let s = rect_sketch_with(&mut doc);
    doc.with(Op::Fill { sketch: s });
    near(area_of(&ok(&mut doc), 0), 150.0, "first");
    doc.set_parameter("w", 40.0).unwrap();
    near(area_of(&ok(&mut doc), 0), 200.0, "second");
}

#[test]
fn a_surface_extrude_of_an_open_curve_is_panels_named_for_its_segments() {
    let mut doc = Document::new();
    let s = open_l(&mut doc);
    let f = doc.with(surface_extrude(s, 5.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    near(area_of(&o, 0), 5.0 * 17.0, "area");
    let body = &o.bodies[0].body;
    assert_eq!(body.faces.len(), 2);
    assert_eq!(find_faces(body, &FaceKey::plain(f.side(0))).len(), 1);
    assert_eq!(find_faces(body, &FaceKey::plain(f.side(1))).len(), 1);
}

#[test]
fn a_surface_extrude_of_a_closed_region_is_its_tube() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0)).hole(Shape::circle(0.0, 0.0, 2.0))]));
    doc.with(surface_extrude(s, 8.0));
    let o = ok(&mut doc);
    // A tube for the outline and one for the hole: each a body of its own, as they don't touch.
    assert_eq!(o.bodies.len(), 2);
    near(area_of(&o, 0) + area_of(&o, 1), TAU * 7.0 * 8.0, "area");
    assert!(o.bodies.iter().all(|b| b.body.sheet));
}

#[test]
fn two_open_curves_in_a_sketch_have_faces_named_apart() {
    let path = |y: f64| Shape::OpenPath { start: point(0.0, y), moves: vec![Move::Line { to: point(10.0, y) }] };
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(path(0.0)), Region::new(path(5.0)), Region::new(Shape::rect(20.0, 0.0, 5.0, 5.0))]));
    doc.with(surface_extrude(s, 3.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    let all: Vec<FaceTag> = o.bodies.iter().flat_map(|b| b.body.faces.iter().map(|f| f.tag)).collect();
    let mut unique = all.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(all.len(), unique.len(), "{all:?}");
}

#[test]
fn a_surface_revolve_sweeps_open_curves_around_either_kind_of_axis() {
    // Up the side at radius 5, around the sketch's z.
    let mut doc = Document::new();
    let path = Shape::OpenPath { start: point(5.0, 0.0), moves: vec![Move::Line { to: point(5.0, 8.0) }] };
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(path)]));
    doc.with(Op::revolve(s, 360.0, Mode::Surface));
    near(area_of(&ok(&mut doc), 0), TAU * 5.0 * 8.0, "around z");

    // The same line, around a line drawn in the sketch: x = 2 running up y, so radius 3.
    let mut doc = Document::new();
    let path = Shape::OpenPath { start: point(5.0, 0.0), moves: vec![Move::Line { to: point(5.0, 8.0) }] };
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(path)]));
    doc.with(Op::Revolve { sketch: s, degrees: 90.0.into(), mode: Mode::Surface, axis: Some(RevolveAxis { point: point(2.0, 0.0), direction: point(0.0, 1.0) }), regions: Vec::new() });
    near(area_of(&ok(&mut doc), 0), (PI / 2.0) * 3.0 * 8.0, "around a line");

    // And a closed region's outline: a rectangle gives a surface with no end caps, which a full turn doesn't have anyway.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(2.0, 0.0, 3.0, 4.0))]));
    doc.with(Op::revolve(s, 360.0, Mode::Surface));
    let o = ok(&mut doc);
    // By the rectangle's own solid: two rings and two cylinders.
    let want = PI * (25.0 - 4.0) * 2.0 + TAU * 2.0 * 4.0 + TAU * 5.0 * 4.0;
    near(area_of(&o, 0), want, "a full turn of a rectangle");
}

#[test]
fn a_sweep_a_loft_and_a_helix_make_sides_without_ends() {
    // A circle along a straight path is a tube.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 3.0))]));
    doc.with(Op::Sweep { sketch: s, path: vec![SweepStep::Line { length: 10.0.into() }], mode: Mode::Surface });
    let o = ok(&mut doc);
    assert!(o.bodies[0].body.sheet);
    near(area_of(&o, 0), TAU * 3.0 * 10.0, "a sweep");

    // Between two squares of one size: four walls.
    let mut doc = Document::new();
    let a = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0))]));
    let b = doc.with(Op::sketch(Placement::xy(6.0), vec![Region::new(Shape::rect(0.0, 0.0, 4.0, 4.0))]));
    doc.with(Op::Loft { sections: vec![a.into(), b.into()], mode: Mode::Surface });
    let o = ok(&mut doc);
    assert!(o.bodies[0].body.sheet);
    assert!(!o.bodies[0].body.boundary_edges().is_empty());
    assert!((area_of(&o, 0) - 4.0 * 4.0 * 6.0).abs() < 1e-3, "{}", area_of(&o, 0));

    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::new(point3(20.0, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::circle(0.0, 0.0, 1.0))]));
    doc.with(Op::Helix { sketch: s, point: point3(0.0, 0.0, 0.0), axis: point3(0.0, 0.0, 1.0), pitch: 5.0.into(), turns: 1.0.into(), left_handed: false, mode: Mode::Surface });
    let o = ok(&mut doc);
    assert!(o.bodies[0].body.sheet);
    validate(&o.bodies[0].body).unwrap();
}

#[test]
fn surface_is_refused_where_nothing_sweeps() {
    let mut doc = Document::new();
    doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::Surface));
    assert!(fails(&mut doc).contains("mode surface"));
}

#[test]
fn a_constrained_sketchs_open_curves_can_sweep() {
    // A polyline of two lines and an arc, in a constrained sketch.
    let mut sketch = Sketch::new();
    let a = sketch.add(Geometry::Line { start: [0.0, 0.0], end: [10.0, 0.0] });
    let b = sketch.add(Geometry::Line { start: [10.0, 0.0], end: [10.0, 5.0] });
    let lone = |doc: &mut Document, sketch: &Sketch| doc.with(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: sketch.clone() });

    // Used for a solid, it's refused as it always was.
    let mut doc = Document::new();
    let s = lone(&mut doc, &sketch);
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    assert!(fails(&mut doc).contains("isn't closed"));

    // Used for a surface it's fine, and named for its entities.
    let mut doc = Document::new();
    let s = lone(&mut doc, &sketch);
    let f = doc.with(surface_extrude(s, 4.0));
    let o = ok(&mut doc);
    near(area_of(&o, 0), 15.0 * 4.0, "area");
    let body = &o.bodies[0].body;
    assert_eq!(find_faces(body, &FaceKey::plain(f.side(a.0))).len(), 1);
    assert_eq!(find_faces(body, &FaceKey::plain(f.side(b.0))).len(), 1);
    assert_eq!(body.shells.len(), 1);
    assert_eq!(body.boundary_edges().len(), 6);

    // Take the surface away again and the sketch goes back to being refused, with what it always said.
    doc.delete(f).unwrap();
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    assert!(fails(&mut doc).contains("isn't closed"), "{}", fails(&mut doc));

    // A line drawn the other way joins up all the same.
    let mut other = Sketch::new();
    other.add(Geometry::Line { start: [0.0, 0.0], end: [10.0, 0.0] });
    other.add(Geometry::Line { start: [10.0, 5.0], end: [10.0, 0.0] });
    let mut doc = Document::new();
    let s = lone(&mut doc, &other);
    doc.with(surface_extrude(s, 4.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert_eq!(o.bodies[0].body.shells.len(), 1);

    // Where three meet, each is on its own, so that no edge has three faces.
    let mut tee = Sketch::new();
    tee.add(Geometry::Line { start: [0.0, 0.0], end: [10.0, 0.0] });
    tee.add(Geometry::Line { start: [10.0, 0.0], end: [20.0, 0.0] });
    tee.add(Geometry::Line { start: [10.0, 0.0], end: [10.0, 5.0] });
    let mut doc = Document::new();
    let s = lone(&mut doc, &tee);
    doc.with(surface_extrude(s, 4.0));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    let total: f64 = (0..3).map(|i| area_of(&o, i)).sum();
    near(total, 25.0 * 4.0, "all three");
    validate(o.body.as_ref().unwrap()).unwrap();

    // An arc in a chain.
    let mut curved = Sketch::new();
    curved.add(Geometry::Line { start: [-5.0, 0.0], end: [0.0, 0.0] });
    curved.add(Geometry::Arc { center: [0.0, 5.0], radius: 5.0, start: [0.0, 0.0], end: [5.0, 5.0] });
    let mut doc = Document::new();
    let s = lone(&mut doc, &curved);
    doc.with(surface_extrude(s, 2.0));
    let o = ok(&mut doc);
    near(area_of(&o, 0), (5.0 + 5.0 * PI / 2.0) * 2.0, "a line and an arc");

    // A chain that starts with an arc and ends with a line: the line's far end is where it says, and not back at the arc's start.
    let mut hook = Sketch::new();
    hook.add(Geometry::Arc { center: [0.0, 5.0], radius: 5.0, start: [0.0, 0.0], end: [5.0, 5.0] });
    hook.add(Geometry::Line { start: [5.0, 5.0], end: [5.0, 20.0] });
    let mut doc = Document::new();
    let s = lone(&mut doc, &hook);
    doc.with(surface_extrude(s, 2.0));
    let o = ok(&mut doc);
    near(area_of(&o, 0), (5.0 * PI / 2.0 + 15.0) * 2.0, "an arc and a line");
    assert!(o.bodies[0].body.vertices.iter().any(|p| p.y == 20.0));
}

#[test]
fn copy_faces_leaves_the_solid_and_makes_a_sheet() {
    let mut doc = Document::new();
    let block = doc.with(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 20.0.into(), dz: 30.0.into() }, Placement::xy(0.0), Mode::Add));
    let copy = doc.with(Op::CopyFaces { faces: vec![FaceKey::plain(block.end_cap())] });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!(!o.bodies[0].body.sheet);
    assert!(o.bodies[1].body.sheet);
    assert_eq!(o.bodies[1].id, BodyId { feature: copy, index: 0 });
    near(area_of(&o, 1), 200.0, "area");
    near(mass_properties(&o.bodies[0].body).volume, 6000.0, "the solid is whole");
    // Its faces can be told apart from the original's, by key, though a name covers both.
    let keys = face_keys(&o.bodies[1].body);
    assert!(keys[0].text().contains('@'), "{}", keys[0].text());
    // A face that has gone is said so.
    let mut doc = Document::new();
    assert!(doc.add(Op::CopyFaces { faces: vec![FaceKey::parse("f9:end").unwrap()] }).unwrap_err().to_string().contains("f9"));
    let mut doc = Document::new();
    let gone = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::Add));
    doc.with(Op::CopyFaces { faces: vec![FaceKey::plain(gone.side(99))] });
    assert!(fails(&mut doc).contains("no longer exists"));
}

// ----- Keeping out of the way -----

#[test]
fn a_sheet_lying_around_doesnt_get_in_a_solids_way() {
    let mut doc = Document::new();
    let block = doc.with(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 10.0.into(), dz: 10.0.into() }, Placement::xy(0.0), Mode::Add));
    let s = rect_sketch(&mut doc, 10.0, 10.0);
    doc.with(Op::Fill { sketch: s });
    // A cut through both, with no list, cuts the solid and doesn't try the sheet.
    let hole = doc.with(Op::sketch(Placement::xy(-1.0), vec![Region::new(Shape::circle(5.0, 5.0, 1.0))]));
    doc.with(Op::extrude(hole, 0.0, 20.0, Mode::Remove));
    // So does a fillet that picks everything, and an add.
    doc.with(Op::fillet(EdgeSelect::OnFace(block.end_cap()), 0.5));
    doc.with(Op::Offset { distance: 0.1.into() });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let sheet = o.bodies.iter().find(|b| b.body.sheet).unwrap();
    near(mass_properties(&sheet.body).area, 100.0, "the sheet is as it was");
    assert!(o.bodies.iter().any(|b| !b.body.sheet && mass_properties(&b.body).volume > 900.0));
    // The whole outcome holds both, and it's valid and has the solid's volume.
    let whole = o.body.as_ref().unwrap();
    assert!(whole.sheet);
    validate(whole).unwrap();
    let solid = o.bodies.iter().find(|b| !b.body.sheet).unwrap();
    near(mass_properties(whole).volume, mass_properties(&solid.body).volume, "volume");
    // Adding something with no list keeps away from it too.
    doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 20.0.into() }, Placement::at(point3(2.0, 2.0, -5.0)), Mode::Add));
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
}

#[test]
fn what_needs_a_solid_refuses_a_sheet_it_names() {
    let setup = || {
        let mut doc = Document::new();
        let s = rect_sketch(&mut doc, 10.0, 10.0);
        let f = doc.with(Op::Fill { sketch: s });
        (doc, f)
    };
    let id = |f: FeatureId| BodyId { feature: f, index: 0 };

    // By name, in the feature's list.
    let (mut doc, f) = setup();
    let cut = doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 20.0.into() }, Placement::at(point3(2.0, 2.0, -5.0)), Mode::Remove));
    doc.set_scope(cut, vec![id(f)]).unwrap();
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    let off = doc.with(Op::Offset { distance: 1.0.into() });
    doc.set_scope(off, vec![id(f)]).unwrap();
    assert!(fails(&mut doc).contains("needs a solid"));

    // By picking what's on it, with nothing to leave out. A sheet alone is no body to work on.
    let (mut doc, f) = setup();
    doc.with(Op::fillet(EdgeSelect::OnFace(f.end_cap()), 1.0));
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    let set = |r: f64| RadiusSet { edges: EdgeSelect::OnFace(f.end_cap()), radius: RadiusLaw::Constant(r.into()) };
    doc.with(Op::FilletRadii { sets: vec![set(1.0)] });
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    doc.with(Op::chamfer(EdgeSelect::OnFace(f.end_cap()), 1.0));
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    doc.with(Op::DeleteFace { faces: vec![FaceKey::plain(f.end_cap())] });
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    doc.with(Op::Draft { faces: vec![f.end_cap()], pull: point3(0.0, 0.0, 1.0), neutral: point3(0.0, 0.0, 0.0), degrees: 5.0.into() });
    assert!(fails(&mut doc).contains("needs a solid"));

    let (mut doc, f) = setup();
    doc.with(Op::Shell { open: vec![f.end_cap()], thickness: 1.0.into() });
    assert!(fails(&mut doc).contains("needs a solid"));

    // A Boolean between bodies.
    let (mut doc, f) = setup();
    let block = doc.with(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 2.0.into() }, Placement::xy(0.0), Mode::NewBody));
    let b = doc.with(Op::BooleanBodies { operation: BodyBoolean::Union, tools: vec![id(block)], keep_tools: false });
    doc.set_scope(b, vec![id(f)]).unwrap();
    assert!(fails(&mut doc).contains("needs a solid"));
}

#[test]
fn a_sheet_moves_copies_and_goes_away_as_any_body_does() {
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 10.0);
    let f = doc.with(Op::Fill { sketch: s });
    // With no list a move takes it along, as it does anything.
    doc.with(Op::Transform { motion: Motion::Translate { by: point3(0.0, 0.0, 5.0) }, copy: false });
    let copied = doc.with(Op::Transform { motion: Motion::Translate { by: point3(0.0, 0.0, 5.0) }, copy: true });
    doc.set_scope(copied, vec![BodyId { feature: f, index: 0 }]).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!(o.bodies.iter().all(|b| b.body.sheet));
    assert!(o.bodies[0].body.vertices.iter().all(|p| p.z == 5.0));
    assert!(o.bodies[1].body.vertices.iter().all(|p| p.z == 10.0));

    // A pattern of the feature that made it.
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 10.0);
    let f = doc.with(Op::Fill { sketch: s });
    doc.with(Op::Pattern { of: vec![f], along: PatternPath::Linear { direction: point3(0.0, 0.0, 1.0), count: 3.0.into(), spacing: 4.0.into() } });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    assert!(o.bodies.iter().all(|b| b.body.sheet));

    // A mirror of it.
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 10.0);
    let f = doc.with(Op::Fill { sketch: s });
    doc.with(Op::Mirror { of: vec![f], point: point3(0.0, 0.0, 3.0), normal: point3(0.0, 0.0, 1.0) });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!(o.bodies[1].body.vertices.iter().all(|p| p.z == 6.0));

    // Gone by name.
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 10.0);
    let f = doc.with(Op::Fill { sketch: s });
    let d = doc.with(Op::DeleteBodies);
    doc.set_scope(d, vec![BodyId { feature: f, index: 0 }]).unwrap();
    assert!(ok(&mut doc).bodies.is_empty());
}

// ----- To a solid -----

#[test]
fn a_sheet_thickens_by_its_faces_or_whole() {
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 20.0);
    let f = doc.with(Op::Fill { sketch: s });
    doc.with(Op::Thicken { faces: vec![FaceKey::plain(f.end_cap())], thickness: 2.0.into(), mode: Mode::NewBody });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let solid = o.bodies.iter().find(|b| !b.body.sheet).unwrap();
    near(mass_properties(&solid.body).volume, 400.0, "by its face");

    // With no faces, every face of the sheet it names.
    let mut doc = Document::new();
    let path = Shape::OpenPath { start: point(5.0, 0.0), moves: vec![Move::Arc { to: point(-5.0, 0.0), center: point(0.0, 0.0), ccw: true }] };
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(path)]));
    let f = doc.with(surface_extrude(s, 10.0));
    let t = doc.with(Op::Thicken { faces: Vec::new(), thickness: 1.0.into(), mode: Mode::NewBody });
    doc.set_scope(t, vec![BodyId { feature: f, index: 0 }]).unwrap();
    let o = ok(&mut doc);
    let solid = o.bodies.iter().find(|b| !b.body.sheet).unwrap();
    // Half an annulus, with the normal to the right of travel, which runs counter-clockwise, so outward.
    near(mass_properties(&solid.body).volume, (PI / 2.0) * (36.0 - 25.0) * 10.0, "whole");

    // With no faces and no names, it can't guess.
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 20.0);
    doc.with(Op::Fill { sketch: s });
    doc.with(Op::Thicken { faces: Vec::new(), thickness: 1.0.into(), mode: Mode::NewBody });
    assert!(fails(&mut doc).contains("at least one face"));

    // A solid can't be thickened whole.
    let mut doc = Document::new();
    let b = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::Add));
    let t = doc.with(Op::Thicken { faces: Vec::new(), thickness: 1.0.into(), mode: Mode::NewBody });
    doc.set_scope(t, vec![BodyId { feature: b, index: 0 }]).unwrap();
    assert!(fails(&mut doc).contains("all the faces of a sheet"));
}

#[test]
fn sewing_makes_a_box_from_a_tube_and_two_ends() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let top = doc.with(Op::sketch(Placement::xy(8.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let tube = doc.with(surface_extrude(s, 8.0));
    let bottom = doc.with(Op::Fill { sketch: s });
    let lid = doc.with(Op::Fill { sketch: top });
    let sew = doc.with(Op::Sew { tolerance: None });
    let o = ok(&mut doc);
    // With no list it takes all of them, and it's a solid now, under the first one's name.
    assert_eq!(o.bodies.len(), 1);
    assert_eq!(o.bodies[0].id, BodyId { feature: tube, index: 0 });
    let solid = &o.bodies[0].body;
    assert!(!solid.sheet);
    validate(solid).unwrap();
    near(mass_properties(solid).volume, PI * 25.0 * 8.0, "volume");
    // And a solid does what solids do.
    doc.with(Op::fillet(EdgeSelect::OnFace(lid.end_cap()), 1.0));
    let o = ok(&mut doc);
    assert!(!o.bodies[0].body.sheet);
    assert!(mass_properties(&o.bodies[0].body).volume < PI * 25.0 * 8.0);

    // With a list it takes only those, and what doesn't close stays a sheet.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let top = doc.with(Op::sketch(Placement::xy(8.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let tube = doc.with(surface_extrude(s, 8.0));
    let bottom2 = doc.with(Op::Fill { sketch: s });
    let lid2 = doc.with(Op::Fill { sketch: top });
    let sewn = doc.with(Op::Sew { tolerance: None });
    doc.set_scope(sewn, vec![BodyId { feature: tube, index: 0 }, BodyId { feature: bottom2, index: 0 }]).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    assert!(o.bodies.iter().all(|b| b.body.sheet));
    assert_eq!(o.bodies[0].id, BodyId { feature: tube, index: 0 });
    assert_eq!(o.bodies[1].id, BodyId { feature: lid2, index: 0 });
    let _ = (bottom, sew);
}

#[test]
fn sewing_says_what_it_cant_do() {
    let mut doc = Document::new();
    doc.with(Op::Sew { tolerance: None });
    assert!(fails(&mut doc).contains("no sheet"));

    let mut doc = Document::new();
    let a = rect_sketch(&mut doc, 10.0, 10.0);
    doc.with(Op::Fill { sketch: a });
    let b = doc.with(Op::sketch(Placement::xy(5.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    doc.with(Op::Fill { sketch: b });
    doc.with(Op::Sew { tolerance: None });
    assert!(fails(&mut doc).contains("nothing to sew"));
}

// ----- Cutting -----

/// A 40 by 40 sheet, as the first thing in a document.
fn wide(doc: &mut Document) -> FeatureId {
    let s = rect_sketch(doc, 40.0, 40.0);
    doc.with(Op::Fill { sketch: s })
}

#[test]
fn a_trim_by_a_solid_keeps_either_part_or_both() {
    for (keep, count, total) in [(TrimKeep::Outside, 1, 1600.0 - PI * 25.0), (TrimKeep::Inside, 1, PI * 25.0), (TrimKeep::Both, 2, 1600.0)] {
        let mut doc = Document::new();
        let f = wide(&mut doc);
        let post = doc.with(Op::solid(Primitive::Cylinder { radius: 5.0.into(), height: 10.0.into() }, Placement::at(point3(20.0, 20.0, -5.0)), Mode::NewBody));
        doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: post, index: 0 }), keep });
        let o = ok(&mut doc);
        let sheets: Vec<&NamedBody> = o.bodies.iter().filter(|b| b.body.sheet).collect();
        assert_eq!(sheets.len(), count, "{keep:?}");
        assert_eq!(sheets[0].id, BodyId { feature: f, index: 0 }, "it stays under its own name");
        let got: f64 = sheets.iter().map(|b| mass_properties(&b.body).area).sum();
        near(got, total, "area");
        // The tool stays.
        assert!(o.bodies.iter().any(|b| !b.body.sheet));
    }
}

#[test]
fn a_trim_that_splits_a_sheet_makes_pieces_of_it() {
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 40.0, 10.0);
    let f = doc.with(Op::Fill { sketch: s });
    let bar = doc.with(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 20.0.into(), dz: 10.0.into() }, Placement::at(point3(15.0, -5.0, -5.0)), Mode::NewBody));
    doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: bar, index: 0 }), keep: TrimKeep::Outside });
    let o = ok(&mut doc);
    let sheets: Vec<&NamedBody> = o.bodies.iter().filter(|b| b.body.sheet).collect();
    assert_eq!(sheets.len(), 2);
    assert!(sheets.iter().any(|b| b.id == BodyId { feature: f, index: 0 }));
    for b in sheets {
        near(mass_properties(&b.body).area, 150.0, "each piece");
    }
}

#[test]
fn a_trim_by_a_plane_keeps_the_side_its_normal_points_to() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    doc.with(surface_extrude(s, 10.0));
    doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 3.0), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Outside });
    near(area_of(&ok(&mut doc), 0), TAU * 5.0 * 7.0, "above");

    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    doc.with(surface_extrude(s, 10.0));
    doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 3.0), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Inside });
    near(area_of(&ok(&mut doc), 0), TAU * 5.0 * 3.0, "below");

    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    doc.with(surface_extrude(s, 10.0));
    doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 3.0), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Both });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(area_of(&o, 0) + area_of(&o, 1), TAU * 5.0 * 10.0, "both");
}

#[test]
fn a_trim_by_a_sketch_cuts_like_a_cookie_cutter() {
    // A hole through a panel, from a sketch off to one side of it along the normal.
    let mut doc = Document::new();
    wide(&mut doc);
    let circle = doc.with(Op::sketch(Placement::xy(50.0), vec![Region::new(Shape::circle(20.0, 20.0, 5.0))]));
    doc.with(Op::Trim { tool: TrimTool::Sketch(circle), keep: TrimKeep::Outside });
    near(area_of(&ok(&mut doc), 0), 1600.0 - PI * 25.0, "a hole");

    // Keeping what's inside two regions gives two pieces.
    let mut doc = Document::new();
    wide(&mut doc);
    let two = doc.with(Op::sketch(Placement::xy(-30.0), vec![Region::new(Shape::circle(10.0, 10.0, 5.0)), Region::new(Shape::circle(30.0, 30.0, 3.0))]));
    doc.with(Op::Trim { tool: TrimTool::Sketch(two), keep: TrimKeep::Inside });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(area_of(&o, 0) + area_of(&o, 1), PI * (25.0 + 9.0), "two");

    // And outside both takes both holes.
    let mut doc = Document::new();
    wide(&mut doc);
    let two = doc.with(Op::sketch(Placement::xy(-30.0), vec![Region::new(Shape::circle(10.0, 10.0, 5.0)), Region::new(Shape::circle(30.0, 30.0, 3.0))]));
    doc.with(Op::Trim { tool: TrimTool::Sketch(two), keep: TrimKeep::Outside });
    near(area_of(&ok(&mut doc), 0), 1600.0 - PI * (25.0 + 9.0), "both holes");
}

#[test]
fn a_trim_follows_its_tool() {
    let mut doc = Document::new();
    doc.set_parameter("r", 5.0).unwrap();
    wide(&mut doc);
    let post = doc.with(Op::solid(Primitive::Cylinder { radius: "r".into(), height: 10.0.into() }, Placement::at(point3(20.0, 20.0, -5.0)), Mode::NewBody));
    doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: post, index: 0 }), keep: TrimKeep::Outside });
    let first = ok(&mut doc);
    near(mass_properties(&first.bodies[0].body).area, 1600.0 - PI * 25.0, "first");
    doc.set_parameter("r", 8.0).unwrap();
    let second = ok(&mut doc);
    near(mass_properties(&second.bodies[0].body).area, 1600.0 - PI * 64.0, "second");
    // Nothing changed, nothing rebuilt.
    let third = ok(&mut doc);
    assert_eq!(third.rebuilt_count(), 0);
}

#[test]
fn a_trim_of_a_named_sheet_follows_a_tool_it_doesnt_name() {
    // The tool isn't among the bodies it acts on, so it's only by being the tool that a change to it matters.
    let mut doc = Document::new();
    doc.set_parameter("r", 5.0).unwrap();
    let f = wide(&mut doc);
    let post = doc.with(Op::solid(Primitive::Cylinder { radius: "r".into(), height: 10.0.into() }, Placement::at(point3(20.0, 20.0, -5.0)), Mode::NewBody));
    let t = doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: post, index: 0 }), keep: TrimKeep::Outside });
    doc.set_scope(t, vec![BodyId { feature: f, index: 0 }]).unwrap();
    near(mass_properties(&ok(&mut doc).bodies[0].body).area, 1600.0 - PI * 25.0, "first");
    doc.set_parameter("r", 8.0).unwrap();
    near(mass_properties(&ok(&mut doc).bodies[0].body).area, 1600.0 - PI * 64.0, "second");
}

#[test]
fn a_file_of_surfaces_comes_in_as_sheets_whatever_the_mode() {
    let panel = fill(&Profile::new(Wire::rect(0.0, 0.0, 10.0, 10.0)), &Frame::WORLD, 1).unwrap();
    let solid = box_solid(&Frame::translation(v3(100.0, 0.0, 0.0)), 2.0, 3.0, 4.0, 2).unwrap();
    let surfaces = coreshape::export::step::write_step(&panel, "panel");
    let both = coreshape::export::step::write_step_bodies(&[&solid, &panel], "both");

    // Asked to cut, there's nothing in a surface to cut with, and it comes in beside what's there.
    for mode in [Mode::Add, Mode::Remove, Mode::Intersect, Mode::NewBody, Mode::Surface] {
        let mut doc = Document::new();
        doc.with(Op::solid(Primitive::Box { dx: 20.0.into(), dy: 20.0.into(), dz: 20.0.into() }, Placement::at(point3(-5.0, -5.0, -5.0)), Mode::Add));
        doc.with(Op::ImportStep { text: surfaces.clone(), mode });
        let o = ok(&mut doc);
        assert_eq!(o.bodies.len(), 2, "{mode:?}");
        assert!(o.bodies[1].body.sheet);
        near(mass_properties(&o.bodies[0].body).volume, 8000.0, "the solid is as it was");
        near(area_of(&o, 1), 100.0, "the sheet");
    }

    // With solids in the file, an import means those, as it always has. The mode surface means what's beside them.
    let mut doc = Document::new();
    doc.with(Op::ImportStep { text: both.clone(), mode: Mode::NewBody });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!(!o.bodies[0].body.sheet);
    let mut doc = Document::new();
    doc.with(Op::ImportStep { text: both, mode: Mode::Surface });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!(o.bodies[0].body.sheet);
    // And with none to bring in, it says so.
    let mut doc = Document::new();
    doc.with(Op::ImportStep { text: coreshape::export::step::write_step(&solid, "solid"), mode: Mode::Surface });
    assert!(fails(&mut doc).contains("no surfaces"));

    // A file's parts can be its sheets.
    let mut doc = Document::new();
    doc.with(Op::ImportStepParts { text: surfaces });
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 1);
    assert!(o.bodies[0].body.sheet);
}

#[test]
fn a_trim_says_what_it_cant_do() {
    // No sheet to cut.
    let mut doc = Document::new();
    doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 0.0), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Outside });
    assert!(fails(&mut doc).contains("no sheet"));

    // A solid, by name.
    let mut doc = Document::new();
    let b = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(0.0), Mode::Add));
    let t = doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 0.5), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Outside });
    doc.set_scope(t, vec![BodyId { feature: b, index: 0 }]).unwrap();
    assert!(fails(&mut doc).contains("this is a solid"));

    // A sheet as the tool, which is what isn't built.
    let mut doc = Document::new();
    let f = wide(&mut doc);
    let other = doc.with(Op::sketch(Placement::new(point3(20.0, -5.0, -5.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![Region::new(Shape::rect(0.0, 0.0, 50.0, 10.0))]));
    let tool = doc.with(Op::Fill { sketch: other });
    let t = doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: tool, index: 0 }), keep: TrimKeep::Outside });
    doc.set_scope(t, vec![BodyId { feature: f, index: 0 }]).unwrap();
    let why = fails(&mut doc);
    assert!(why.contains("another sheet") && why.contains("no inside"), "{why}");

    // A tool that isn't there.
    let mut doc = Document::new();
    wide(&mut doc);
    doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: FeatureId(1), index: 7 }), keep: TrimKeep::Outside });
    assert!(fails(&mut doc).contains("isn't there"));

    // A plane with no normal.
    let mut doc = Document::new();
    wide(&mut doc);
    doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 0.0), normal: point3(0.0, 0.0, 0.0) }, keep: TrimKeep::Outside });
    assert!(fails(&mut doc).contains("normal"));

    // A sketch with no regions, only an open curve.
    let mut doc = Document::new();
    wide(&mut doc);
    let open = open_l(&mut doc);
    doc.with(Op::Trim { tool: TrimTool::Sketch(open), keep: TrimKeep::Outside });
    assert!(fails(&mut doc).contains("closed regions"));

    // A sketch that doesn't come before it, or isn't a sketch, is refused at once.
    let mut doc = Document::new();
    let f = wide(&mut doc);
    assert!(doc.add(Op::Trim { tool: TrimTool::Sketch(f), keep: TrimKeep::Outside }).unwrap_err().to_string().contains("not a sketch"));
    assert!(doc.add(Op::Fill { sketch: f }).unwrap_err().to_string().contains("not a sketch"));
    assert!(doc.add(Op::Fill { sketch: FeatureId(99) }).is_err());
}

// ----- Saving -----

#[test]
fn every_new_thing_comes_back_from_the_file_the_same() {
    let mut doc = Document::new();
    let s = rect_sketch(&mut doc, 10.0, 20.0);
    let open = open_l(&mut doc);
    let circle = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(3.0, 3.0, 1.0))]));
    let fill = doc.with(Op::Fill { sketch: s });
    let extruded = doc.with(surface_extrude(open, 5.0));
    let cut = doc.with(Op::Trim { tool: TrimTool::Sketch(circle), keep: TrimKeep::Outside });
    doc.set_scope(cut, vec![BodyId { feature: fill, index: 0 }]).unwrap();
    let plane = doc.with(Op::Trim { tool: TrimTool::Plane { point: point3(0.0, 0.0, 2.0), normal: point3(0.0, 0.0, 1.0) }, keep: TrimKeep::Both });
    doc.set_scope(plane, vec![BodyId { feature: extruded, index: 0 }]).unwrap();
    let block = doc.with(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::at(point3(100.0, 0.0, 0.0)), Mode::NewBody));
    doc.with(Op::CopyFaces { faces: vec![FaceKey::plain(block.end_cap())] });
    let again = doc.with(Op::Fill { sketch: circle });
    let by_body = doc.with(Op::Trim { tool: TrimTool::Body(BodyId { feature: block, index: 0 }), keep: TrimKeep::Inside });
    doc.set_scope(by_body, vec![BodyId { feature: again, index: 0 }]).unwrap();
    let o = doc.rebuild(None);
    let text = doc.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.save_text(), text);
    let p = back.rebuild(None);
    assert_eq!(p.failed, o.failed);
    assert_eq!(o.bodies.len(), p.bodies.len());
    for (a, b) in o.bodies.iter().zip(&p.bodies) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.body.sheet, b.body.sheet);
        assert_eq!(mass_properties(&a.body).area.to_bits(), mass_properties(&b.body).area.to_bits());
    }
    // A sew with a tolerance, and one without, read as they were written.
    for op in [Op::Sew { tolerance: None }, Op::Sew { tolerance: Some(1e-4.into()) }, Op::Sew { tolerance: Some("g".into()) }] {
        let v = op_value(&op);
        assert_eq!(read_op(&v, &[]).unwrap(), op);
    }
    for keep in ["outside", "inside", "both"] {
        let v = json::parse(&format!("{{\"type\":\"trim\",\"tool\":\"f2.1\",\"keep\":\"{keep}\"}}")).unwrap();
        assert_eq!(op_value(&read_op(&v, &[]).unwrap()).to_text(), v.to_text());
    }
}

#[test]
fn a_file_asking_for_the_impossible_is_refused() {
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]).unwrap_err().to_string();
    assert!(read(r#"{"type":"trim"}"#).contains("takes one of"));
    assert!(read(r#"{"type":"trim","tool":"f1","point":[0,0,0],"normal":[0,0,1]}"#).contains("takes one of"));
    assert!(read(r#"{"type":"trim","tool":"nonsense"}"#).contains("name of a body"));
    assert!(read(r#"{"type":"trim","tool":"f1","keep":"middle"}"#).contains("middle"));
    assert!(read(r#"{"type":"trim","point":[0,0,0]}"#).contains("normal"));
    assert!(read(r#"{"type":"fill"}"#).contains("sketch"));
    assert!(read(r#"{"type":"fill","sketch":1,"extra":2}"#).contains("extra"));
    assert!(read(r#"{"type":"copy_faces","sketch":1}"#).contains("sketch"));
    assert!(read(r#"{"type":"sew","faces":[]}"#).contains("faces"));
    assert!(read(r#"{"type":"extrude","sketch":1,"start":0,"length":1,"mode":"sheet"}"#).contains("surface"));
}

#[test]
fn an_open_path_is_refused_where_it_cant_be() {
    let mut doc = Document::new();
    let open = Shape::OpenPath { start: point(0.0, 0.0), moves: vec![Move::Line { to: point(1.0, 0.0) }] };
    doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open.clone()).hole(Shape::circle(0.0, 0.0, 1.0))]));
    assert!(fails(&mut doc).contains("can't have holes"));

    let mut doc = Document::new();
    let crossing = Shape::OpenPath { start: point(0.0, 0.0), moves: vec![Move::Line { to: point(10.0, 10.0) }, Move::Line { to: point(10.0, 0.0) }, Move::Line { to: point(0.0, 10.0) }] };
    doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(crossing)]));
    assert!(fails(&mut doc).contains("crosses itself"));

    // For a solid it has no closed regions, as it says.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open)]));
    doc.with(Op::extrude(s, 0.0, 1.0, Mode::Add));
    assert!(fails(&mut doc).contains("no closed regions"));
}

#[test]
fn solids_come_out_the_same_with_sheets_about() {
    // The same part, with and without sheets in the document, to the bit.
    let build = |with_sheets: bool| {
        let mut doc = Document::new();
        if with_sheets {
            let s = rect_sketch(&mut doc, 10.0, 10.0);
            doc.with(Op::Fill { sketch: s });
        }
        let base = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 20.0)).hole(Shape::circle(15.0, 10.0, 4.0))]));
        let block = doc.with(Op::extrude(base, 0.0, 10.0, Mode::Add));
        doc.with(Op::fillet(EdgeSelect::OnFace(block.end_cap()), 1.0));
        let o = ok(&mut doc);
        let solid = o.bodies.iter().find(|b| !b.body.sheet).unwrap().body.clone();
        solid
    };
    let (a, b) = (build(false), build(true));
    assert_eq!(mass_properties(&a).volume.to_bits(), mass_properties(&b).volume.to_bits());
    assert_eq!(a.faces.len(), b.faces.len());
    assert_eq!(a.edges.len(), b.edges.len());
}
