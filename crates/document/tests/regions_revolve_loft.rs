//! Regions on revolve and loft, as extrude has them: picked by id from a constrained sketch, the revolve's own or another,
//! and a loft's sections as regions of sketches with several outlines. Volumes are held against closed formulas: Pappus for
//! a revolve, and the frustum and cylinder for a loft.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::{RegionId, Rule, Sketch};
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

/// A rectangle from corner to corner, its corners joined.
fn rect(s: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) {
    let (a, b, c, d) = (s.line([x0, y0], [x1, y0]), s.line([x1, y0], [x1, y1]), s.line([x1, y1], [x0, y1]), s.line([x0, y1], [x0, y0]));
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
}

fn rects(list: &[[f64; 4]]) -> Sketch {
    let mut s = Sketch::new();
    for r in list {
        rect(&mut s, r[0], r[1], r[2], r[3]);
    }
    s
}

fn circles(list: &[(f64, f64, f64)]) -> Sketch {
    let mut s = Sketch::new();
    for &(x, y, r) in list {
        s.circle([x, y], r);
    }
    s
}

fn sketch_on(doc: &mut Document, plane: Placement, sketch: Sketch) -> FeatureId {
    doc.add(Op::ConstrainedSketch { plane, sketch }).unwrap()
}

/// Region ids by area, as a test can't know them otherwise.
fn region_of_area(doc: &Document, sketch: FeatureId, area: f64) -> RegionId {
    let found = doc.sketch_arrangement(sketch).unwrap();
    found.regions.iter().find(|r| (r.area() - area).abs() < 1e-3).unwrap_or_else(|| panic!("no region of area {area}")).id
}

fn pick(sketch: FeatureId, region: RegionId) -> RegionPick {
    RegionPick { sketch, region }
}

/// About the sketch's y axis.
fn y_axis() -> Option<RevolveAxis> {
    Some(RevolveAxis { point: point(0.0, 0.0), direction: point(0.0, 1.0) })
}

fn revolve(sketch: FeatureId, degrees: f64, mode: Mode, axis: Option<RevolveAxis>, regions: Vec<RegionPick>) -> Op {
    Op::Revolve { sketch, degrees: degrees.into(), mode, axis, regions }
}

// A rectangle 2 wide and 3 high centred 3 from the axis, and one 1 wide and 2 high centred 6.5 from it.
const A: f64 = 2.0 * PI * 3.0 * 6.0;
const B: f64 = 2.0 * PI * 6.5 * 2.0;

#[test]
fn a_revolve_takes_only_the_regions_it_picks() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0], [6.0, 0.0, 7.0, 2.0]]));
    let (a, b) = (region_of_area(&doc, s, 6.0), region_of_area(&doc, s, 2.0));
    let r = doc.add(revolve(s, 360.0, Mode::NewBody, y_axis(), vec![pick(s, a)])).unwrap();
    near(total(&ok(&mut doc)), A, 1e-6, "one");
    doc.edit(r, revolve(s, 360.0, Mode::NewBody, y_axis(), vec![pick(s, a), pick(s, b)])).unwrap();
    near(total(&ok(&mut doc)), A + B, 1e-6, "both");
    doc.edit(r, revolve(s, 360.0, Mode::NewBody, y_axis(), Vec::new())).unwrap();
    near(total(&ok(&mut doc)), A + B, 1e-6, "the whole sketch, as before");
    // Without an axis, profile x is the distance from the sketch's Z axis, as it always was.
    doc.edit(r, revolve(s, 360.0, Mode::NewBody, None, vec![pick(s, b)])).unwrap();
    near(total(&ok(&mut doc)), 2.0 * PI * 6.5 * 2.0, 1e-6, "no axis");
}

#[test]
fn a_region_from_another_sketch_in_the_plane_turns_around_this_ones_axis() {
    // The same rectangle drawn in a sketch whose origin and x axis are elsewhere in the plane, and in one facing the other way.
    let others = [
        // (u, v) here is (1 - v, u) in the world.
        (Placement::new(point3(1.0, 0.0, 0.0), point3(0.0, 0.0, 1.0), point3(0.0, 1.0, 0.0)), [0.0, -6.0, 2.0, -5.0]),
        // (u, v) here is (u, -v) in the world.
        (Placement::new(point3(0.0, 0.0, 0.0), point3(0.0, 0.0, -1.0), point3(1.0, 0.0, 0.0)), [6.0, -2.0, 7.0, 0.0]),
    ];
    for (plane, corners) in others {
        let mut doc = Document::new();
        let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0]]));
        let t = sketch_on(&mut doc, plane, rects(&[corners]));
        let (a, b) = (region_of_area(&doc, s, 6.0), region_of_area(&doc, t, 2.0));
        doc.add(revolve(s, 360.0, Mode::NewBody, y_axis(), vec![pick(s, a), pick(t, b)])).unwrap();
        let o = ok(&mut doc);
        near(total(&o), A + B, 1e-6, "both");
        // Faces of the second sketch are told apart from the first's, as an extrude's are.
        let names: Vec<String> = face_keys(o.body.as_ref().unwrap()).iter().map(|k| k.face.name()).collect();
        assert!(names.iter().any(|n| n.starts_with("f3:region.1.side.")) && names.iter().any(|n| n.starts_with("f3:side.")), "{names:?}");
    }
}

#[test]
fn without_an_axis_each_region_turns_around_its_own_sketch() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0]]));
    let t = sketch_on(&mut doc, Placement::xy(10.0), rects(&[[1.0, 0.0, 2.0, 1.0]]));
    let (a, c) = (region_of_area(&doc, s, 6.0), region_of_area(&doc, t, 1.0));
    doc.add(revolve(s, 360.0, Mode::Add, None, vec![pick(s, a), pick(t, c)])).unwrap();
    let o = ok(&mut doc);
    near(total(&o), A + 2.0 * PI * 1.5 * 1.0, 1e-6, "both");
    assert_eq!(o.bodies.len(), 2);
}

#[test]
fn with_an_axis_a_region_off_its_plane_is_refused() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0]]));
    let t = sketch_on(&mut doc, Placement::xy(10.0), rects(&[[1.0, 0.0, 2.0, 1.0]]));
    let c = region_of_area(&doc, t, 1.0);
    doc.add(revolve(s, 360.0, Mode::Add, y_axis(), vec![pick(t, c)])).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("f2") && why.contains("isn't in the plane") && why.contains("f1"), "{why}");
}

#[test]
fn a_region_that_has_gone_fails_the_revolve_saying_whats_there() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0]]));
    let a = region_of_area(&doc, s, 6.0);
    let gone = RegionId::parse("r0123456789ab").unwrap();
    doc.add(revolve(s, 360.0, Mode::Add, y_axis(), vec![pick(s, gone)])).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("r0123456789ab") && why.contains(&a.name()), "{why}");
}

#[test]
fn a_surface_revolve_takes_the_regions_it_picks() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), rects(&[[2.0, 0.0, 4.0, 3.0], [6.0, 0.0, 7.0, 2.0]]));
    let a = region_of_area(&doc, s, 6.0);
    doc.add(revolve(s, 90.0, Mode::Surface, y_axis(), vec![pick(s, a)])).unwrap();
    let o = ok(&mut doc);
    assert!(o.bodies.iter().all(|b| b.body.sheet));
    // The two cylinders and the two flat rings a quarter of the way round, without the ends.
    let quarter = PI / 2.0;
    let area: f64 = o.bodies.iter().map(|b| mass_properties(&b.body).area).sum();
    near(area, quarter * (4.0 * 3.0 + 2.0 * 3.0 + (16.0 - 4.0)), 1e-6, "area");
}

#[test]
fn a_loft_takes_regions_from_sketches_with_several_outlines() {
    let mut doc = Document::new();
    let low = sketch_on(&mut doc, Placement::xy(0.0), circles(&[(0.0, 0.0, 10.0), (30.0, 0.0, 4.0)]));
    let high = sketch_on(&mut doc, Placement::xy(20.0), circles(&[(0.0, 0.0, 5.0), (30.0, 0.0, 4.0)]));
    let (big, small) = (region_of_area(&doc, low, PI * 100.0), region_of_area(&doc, high, PI * 25.0));
    let (side_low, side_high) = (region_of_area(&doc, low, PI * 16.0), region_of_area(&doc, high, PI * 16.0));
    doc.add(Op::Loft { sections: vec![pick(low, big).into(), pick(high, small).into()], mode: Mode::Add }).unwrap();
    doc.add(Op::Loft { sections: vec![pick(low, side_low).into(), pick(high, side_high).into()], mode: Mode::Add }).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(total(&o), PI * 20.0 * (100.0 + 50.0 + 25.0) / 3.0 + PI * 16.0 * 20.0, 1e-6, "a frustum and a cylinder");
}

#[test]
fn a_loft_section_with_several_outlines_says_to_pick_one_and_a_gone_one_says_whats_there() {
    let mut doc = Document::new();
    let low = sketch_on(&mut doc, Placement::xy(0.0), circles(&[(0.0, 0.0, 10.0), (30.0, 0.0, 4.0)]));
    let high = sketch_on(&mut doc, Placement::xy(20.0), circles(&[(0.0, 0.0, 5.0)]));
    let l = doc.add(Op::Loft { sections: vec![low.into(), high.into()], mode: Mode::Add }).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("not 2") && why.contains("\"region\""), "{why}");
    let gone = RegionId::parse("r0123456789ab").unwrap();
    doc.edit(l, Op::Loft { sections: vec![pick(low, gone).into(), high.into()], mode: Mode::Add }).unwrap();
    let why = failure(&mut doc);
    let big = region_of_area(&doc, low, PI * 100.0);
    assert!(why.contains("r0123456789ab") && why.contains(&big.name()), "{why}");
}

#[test]
fn json_takes_regions_on_revolve_and_loft() {
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]);
    let op = read(r#"{"type": "revolve", "sketch": 2, "regions": ["r0123456789ab", {"sketch": 3, "region": "r00000000abcd"}], "axis": {"point": [0, 0], "direction": [0, 1]}}"#).unwrap();
    let Op::Revolve { sketch, regions, .. } = &op else { panic!() };
    assert_eq!((*sketch, regions.len(), regions[1].sketch), (FeatureId(2), 2, FeatureId(3)));
    let text = op_value(&op).to_text();
    assert!(text.contains("\"r0123456789ab\"") && text.contains("\"r00000000abcd\""), "{text}");
    assert_eq!(read(&text).unwrap(), op);
    // With no sketch, the first region's.
    let op = read(r#"{"type": "revolve", "regions": [{"sketch": 3, "region": "r00000000abcd"}]}"#).unwrap();
    assert!(matches!(&op, Op::Revolve { sketch: FeatureId(3), .. }));
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);
    // A plain id needs the sketch.
    let why = read(r#"{"type": "revolve", "regions": ["r0123456789ab"]}"#).unwrap_err().to_string();
    assert!(why.contains("a revolve's \"sketch\""), "{why}");

    let op = read(r#"{"type": "loft", "sections": [2, {"sketch": 3, "region": "r00000000abcd"}]}"#).unwrap();
    let Op::Loft { sections, .. } = &op else { panic!() };
    assert_eq!(sections[0], LoftSection::from(FeatureId(2)));
    assert_eq!(sections[1].sketch, FeatureId(3));
    assert!(sections[1].region.is_some());
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);
    assert!(read(r#"{"type": "loft", "sections": [2, {"sketch": 3}]}"#).is_err());
}
