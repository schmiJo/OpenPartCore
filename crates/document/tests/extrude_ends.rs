//! An extrude that picks regions from sketches of its own choosing, and ends somewhere other than at a length, as Onshape's
//! does. Each result is held against a closed formula, so nothing here is the kernel marking its own work.

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

fn lowest_highest(b: &Body, along: Vec3) -> (f64, f64) {
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    let list = mesh.positions.iter().map(|p| p.dot(along));
    list.fold((f64::MAX, f64::MIN), |(lo, hi), h| (lo.min(h), hi.max(h)))
}

/// A sketch of a circle and a rectangle apart, and a second circle inside the rectangle.
fn three_regions() -> Sketch {
    let mut s = Sketch::new();
    s.circle([50.0, 0.0], 5.0);
    let (a, b, c, d) = (s.line([0.0, 0.0], [20.0, 0.0]), s.line([20.0, 0.0], [20.0, 20.0]), s.line([20.0, 20.0], [0.0, 20.0]), s.line([0.0, 20.0], [0.0, 0.0]));
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
    s.circle([10.0, 10.0], 3.0);
    s
}

fn sketch_on(doc: &mut Document, plane: Placement, sketch: Sketch) -> FeatureId {
    doc.add(Op::ConstrainedSketch { plane, sketch }).unwrap()
}

fn circle_sketch(x: f64, y: f64, r: f64) -> Sketch {
    let mut s = Sketch::new();
    s.circle([x, y], r);
    s
}

/// Region ids by area, as a test can't know them otherwise.
fn region_of_area(doc: &Document, sketch: FeatureId, area: f64) -> RegionId {
    let found = doc.sketch_arrangement(sketch).unwrap();
    found.regions.iter().find(|r| (r.area() - area).abs() < 1e-3).unwrap_or_else(|| panic!("no region of area {area}")).id
}

fn extrude_regions(sketch: FeatureId, picks: &[(FeatureId, RegionId)], length: f64, mode: Mode) -> Op {
    let regions = picks.iter().map(|(sketch, region)| RegionPick { sketch: *sketch, region: *region }).collect();
    Op::Extrude { sketch, start: 0.0.into(), length: length.into(), mode, regions, end: ExtrudeEnd::Blind, opposite: false }
}

fn extrude_to(sketch: FeatureId, end: ExtrudeEnd, mode: Mode) -> Op {
    Op::Extrude { sketch, start: 0.0.into(), length: 0.0.into(), mode, regions: Vec::new(), end, opposite: false }
}

fn backward(op: Op) -> Op {
    match op {
        Op::Extrude { sketch, start, length, mode, regions, end, .. } => Op::Extrude { sketch, start, length, mode, regions, end, opposite: true },
        other => other,
    }
}

fn solid(dx: f64, dy: f64, dz: f64, at: [f64; 3]) -> Op {
    Op::solid(Primitive::Box { dx: dx.into(), dy: dy.into(), dz: dz.into() }, Placement::at(point3(at[0], at[1], at[2])), Mode::NewBody)
}

fn key(text: &str) -> FaceKey {
    FaceKey::parse(text).unwrap()
}

// ----- Regions -----

#[test]
fn two_extrudes_can_take_different_regions_of_one_sketch() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let circle = region_of_area(&doc, s, PI * 25.0);
    let frame = region_of_area(&doc, s, 400.0 - PI * 9.0);
    let inner = region_of_area(&doc, s, PI * 9.0);
    doc.add(extrude_regions(s, &[(s, circle)], 10.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), PI * 25.0 * 10.0, 1e-6, "the circle alone");
    doc.add(extrude_regions(s, &[(s, frame), (s, inner)], 7.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    // The square with its inner circle filled, as they share an edge and so merge into one part, as in Onshape.
    assert_eq!(o.bodies.len(), 2);
    near(vol(&o.bodies[1].body), 400.0 * 7.0, 1e-6, "the whole square");
}

#[test]
fn an_inner_region_alone_is_a_boss_and_not_a_hole() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let inner = region_of_area(&doc, s, PI * 9.0);
    doc.add(extrude_regions(s, &[(s, inner)], 4.0, Mode::NewBody)).unwrap();
    near(total(&ok(&mut doc)), PI * 9.0 * 4.0, 1e-6, "the inner circle");
}

#[test]
fn one_extrude_can_take_regions_from_several_sketches() {
    let mut doc = Document::new();
    let low = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let high = sketch_on(&mut doc, Placement::xy(30.0), circle_sketch(0.0, 0.0, 4.0));
    let side = Placement::new(point3(100.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0));
    let tilted = sketch_on(&mut doc, side, circle_sketch(0.0, 0.0, 2.0));
    let (a, b, c) = (region_of_area(&doc, low, PI * 25.0), region_of_area(&doc, high, PI * 16.0), region_of_area(&doc, tilted, PI * 4.0));
    doc.add(extrude_regions(low, &[(low, a), (high, b), (tilted, c)], 5.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 3);
    near(total(&o), PI * (25.0 + 16.0 + 4.0) * 5.0, 1e-6, "all three");
    // Each goes along its own sketch's normal.
    let (lo, hi) = lowest_highest(&o.bodies.iter().find(|b| (vol(&b.body) - PI * 80.0).abs() < 1e-6).unwrap().body, Vec3::Z);
    near(lo, 30.0, 1e-9, "from the high sketch");
    near(hi, 35.0, 1e-9, "along its normal");
    let (lo, hi) = lowest_highest(&o.bodies.iter().find(|b| (vol(&b.body) - PI * 20.0).abs() < 1e-6).unwrap().body, Vec3::X);
    near(lo, 100.0, 1e-9, "from the tilted sketch");
    near(hi, 105.0, 1e-9, "along its normal");
    // All three are from circles that are entity 1 of their sketches, and their walls still have different names.
    let names: Vec<String> = o.bodies.iter().flat_map(|b| b.body.faces.iter().filter(|f| !matches!(f.surface, Surface::Plane(_))).map(|f| FaceRef::from_tag(f.tag).name())).collect();
    let mut distinct = names.clone();
    distinct.sort();
    distinct.dedup();
    let by_body = |i: usize| o.bodies[i].body.faces.iter().filter(|f| !matches!(f.surface, Surface::Plane(_))).map(|f| f.tag).collect::<Vec<_>>();
    for (i, j) in [(0, 1), (0, 2), (1, 2)] {
        assert!(by_body(i).iter().all(|t| !by_body(j).contains(t)), "{names:?}");
    }
    assert!(distinct.len() >= 3, "{names:?}");
}

#[test]
fn a_change_to_any_of_its_sketches_comes_through_and_its_ids_last() {
    let mut doc = Document::new();
    doc.set_parameter("w", 20.0).unwrap();
    let first = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    let mut second_sketch = Sketch::new();
    let c = second_sketch.circle([100.0, 0.0], 8.0);
    second_sketch.constrain(Rule::Diameter { of: c, value: "w".into() });
    let second = sketch_on(&mut doc, Placement::xy(0.0), second_sketch);
    let (a, b) = (region_of_area(&doc, first, PI * 25.0), region_of_area(&doc, second, PI * 100.0));
    doc.add(extrude_regions(first, &[(first, a), (second, b)], 10.0, Mode::NewBody)).unwrap();
    near(total(&ok(&mut doc)), PI * (25.0 + 100.0) * 10.0, 1e-6, "as drawn");
    // The second sketch isn't the one the extrude is filed under, and it has to be rebuilt all the same.
    doc.set_parameter("w", 30.0).unwrap();
    near(total(&ok(&mut doc)), PI * (25.0 + 225.0) * 10.0, 1e-6, "the second circle bigger");
    assert_eq!(region_of_area(&doc, second, PI * 225.0), b);
}

#[test]
fn a_region_thats_gone_is_refused_and_says_whats_there() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let circle = region_of_area(&doc, s, PI * 25.0);
    let gone = RegionId(0xabc);
    doc.add(extrude_regions(s, &[(s, circle), (s, gone)], 10.0, Mode::NewBody)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&gone.name()) && why.contains(&circle.name()), "{why}");
    assert!(why.contains(&format!("f{}", s.0)), "{why}");
}

#[test]
fn regions_need_a_constrained_sketch_and_it_has_to_come_first() {
    let mut doc = Document::new();
    let plain = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))])).unwrap();
    doc.add(extrude_regions(plain, &[(plain, RegionId(1))], 10.0, Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("constrained"));

    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let solid_feature = doc.add(solid(1.0, 1.0, 1.0, [0.0, 0.0, 0.0])).unwrap();
    let circle = region_of_area(&doc, s, PI * 25.0);
    assert!(doc.add(extrude_regions(s, &[(solid_feature, circle)], 1.0, Mode::NewBody)).is_err());
    assert!(doc.add(extrude_regions(s, &[(FeatureId(99), circle)], 1.0, Mode::NewBody)).is_err());
    // It's in use by the extrude, whichever sketch that's filed under, so it can't go.
    let other = sketch_on(&mut doc, Placement::xy(5.0), circle_sketch(0.0, 0.0, 1.0));
    let r = region_of_area(&doc, other, PI);
    doc.add(extrude_regions(s, &[(other, r)], 1.0, Mode::NewBody)).unwrap();
    assert!(doc.delete(other).is_err());
}

#[test]
fn a_suppressed_sketch_among_several_says_so() {
    let mut doc = Document::new();
    let (a, b) = (sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 1.0)), sketch_on(&mut doc, Placement::xy(5.0), circle_sketch(0.0, 0.0, 2.0)));
    let (ra, rb) = (region_of_area(&doc, a, PI), region_of_area(&doc, b, PI * 4.0));
    doc.add(extrude_regions(a, &[(a, ra), (b, rb)], 1.0, Mode::NewBody)).unwrap();
    ok(&mut doc);
    doc.suppress(b, true).unwrap();
    assert!(failure(&mut doc).contains(&format!("f{}", b.0)));
}

// ----- Through all -----

#[test]
fn a_cut_through_all_goes_right_through_and_only_ever_ahead() {
    for (backwards, z) in [(false, -50.0), (true, 50.0)] {
        let mut doc = Document::new();
        doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap();
        let s = sketch_on(&mut doc, Placement::xy(z), circle_sketch(20.0, 20.0, 5.0));
        let op = extrude_to(s, ExtrudeEnd::ThroughAll, Mode::Remove);
        doc.add(if backwards { backward(op) } else { op }).unwrap();
        near(total(&ok(&mut doc)), 40.0 * 40.0 * 10.0 - PI * 25.0 * 10.0, 1e-6, "a hole right through");
    }
    // The other way there's nothing to go through, and it says so rather than doing nothing.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(50.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::Remove)).unwrap();
    assert!(failure(&mut doc).contains("opposite"));
}

#[test]
fn through_all_goes_through_all_of_them_and_keeps_to_its_scope() {
    let mut doc = Document::new();
    let parts = [doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap(), doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 30.0])).unwrap(), doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 60.0])).unwrap()];
    let s = sketch_on(&mut doc, Placement::xy(-5.0), circle_sketch(20.0, 20.0, 5.0));
    let cut = doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::Remove)).unwrap();
    let hole = PI * 25.0 * 10.0;
    let o = ok(&mut doc);
    for b in &o.bodies {
        near(vol(&b.body), 16000.0 - hole, 1e-6, "each has a hole");
    }
    doc.set_scope(cut, vec![BodyId { feature: parts[1], index: 0 }]).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[0].body), 16000.0, 1e-6, "not this one");
    near(vol(&o.bodies[1].body), 16000.0 - hole, 1e-6, "this one");
    near(vol(&o.bodies[2].body), 16000.0, 1e-6, "not this one");
}

#[test]
fn a_new_solid_through_all_ends_flush_with_the_furthest_part() {
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 20.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(100.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[1].body), PI * 25.0 * 30.0, 1e-6, "up to z = 30");

    // The far side of a round part is on no vertex, and a mesh of it has to do.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Sphere { radius: 10.0.into() }, Placement::at(point3(0.0, 0.0, 20.0)), Mode::NewBody)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(100.0, 0.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[1].body), PI * 25.0 * 30.0, 1e-2, "up to the top of the sphere");
}

#[test]
fn through_all_needs_something_to_go_through() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("through"));
}

// ----- Up to a face -----

#[test]
fn up_to_a_flat_face_is_exactly_that_far_whatever_the_face_covers() {
    let mut doc = Document::new();
    let plate = doc.add(solid(10.0, 10.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    // Well off to one side of the plate. The face is extended as far as it needs to be, as Onshape does.
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(200.0, 0.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(FaceKey::plain(plate.start_cap())), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[1].body), PI * 25.0 * 30.0, 1e-6, "to the plate's underside");
    // Its far end is its end cap, as with a length.
    assert!(o.bodies[1].body.faces.iter().any(|f| f.tag == FaceTag { feature: 3, slot: 1 }));
}

#[test]
fn up_to_a_face_follows_the_face() {
    let mut doc = Document::new();
    doc.set_parameter("gap", 30.0).unwrap();
    let plate = doc.add(Op::solid(Primitive::Box { dx: 40.0.into(), dy: 40.0.into(), dz: 2.0.into() }, Placement::xy("gap"), Mode::NewBody)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(FaceKey::plain(plate.start_cap())), Mode::NewBody)).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 30.0, 1e-6, "at 30");
    doc.set_parameter("gap", 44.0).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 44.0, 1e-6, "at 44, though a new body needs nothing else as a rule");
}

#[test]
fn up_to_a_slanted_plane_has_a_slanted_end() {
    // A wedge's sloped top: z = 40 - x / 2 over x from 0 to 40. A prism's volume under a plane is its area times the height
    // at its centroid, which is what this comes to.
    let mut doc = Document::new();
    let wedge = doc.add(Op::sketch(Placement::new(point3(0.0, 100.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::polygon(vec![point(0.0, 0.0), point(40.0, 0.0), point(40.0, 20.0), point(0.0, 40.0)]))])).unwrap();
    let solid_wedge = doc.add(Op::extrude(wedge, 0.0, 10.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    let body = &o.bodies[0].body;
    let slope = body.faces.iter().find(|f| matches!(&f.surface, Surface::Plane(p) if p.z.z.abs() > 0.1 && p.z.z.abs() < 0.99)).unwrap();
    let name = face_keys(body)[body.faces.iter().position(|f| std::ptr::eq(f, slope)).unwrap()].text();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    let e = doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    let height_at_centroid = 40.0 - 20.0 / 2.0;
    let got = vol(&o.bodies[1].body);
    near(got, PI * 25.0 * height_at_centroid, 1e-6, "a circle up to the slope");
    // Named as a cap, like any end.
    let end = FaceTag { feature: e.0, slot: 1 };
    let cap = o.bodies[1].body.faces.iter().find(|f| f.tag == end).expect("an end cap");
    assert!(matches!(cap.surface, Surface::Plane(_)));
    let _ = solid_wedge;
}

/// The volume of a rectangular prism from z = 0 up to the underside of a cylinder along y, centred at height `c`.
fn under_cylinder(half_width: f64, depth: f64, r: f64, c: f64) -> f64 {
    let x = half_width;
    // The integral of c - sqrt(r^2 - x^2), from -x to x.
    depth * (2.0 * x * c - (x * (r * r - x * x).sqrt() + r * r * (x / r).asin()))
}

#[test]
fn up_to_a_cylinder_takes_its_curve() {
    // A rod along y, r = 20, centred at z = 50, and a block coming up from below to its underside.
    let frame_z = point3(0.0, 1.0, 0.0);
    let mut doc = Document::new();
    let rod = doc.add(Op::solid(Primitive::Cylinder { radius: 20.0.into(), height: 200.0.into() }, Placement::new(point3(0.0, -100.0, 50.0), frame_z, point3(1.0, 0.0, 0.0)), Mode::NewBody)).unwrap();
    let side = {
        let o = ok(&mut doc);
        let b = &o.bodies[0].body;
        let f = b.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder { .. })).unwrap();
        face_keys(b)[f].text().split('#').next().unwrap().to_string()
    };
    let mut rect = Sketch::new();
    let (a, b, c, d) = (rect.line([-5.0, -8.0], [5.0, -8.0]), rect.line([5.0, -8.0], [5.0, 8.0]), rect.line([5.0, 8.0], [-5.0, 8.0]), rect.line([-5.0, 8.0], [-5.0, -8.0]));
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        rect.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
    let s = sketch_on(&mut doc, Placement::xy(0.0), rect);
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&side)), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[1].body), under_cylinder(5.0, 16.0, 20.0, 50.0), 1e-5, "up to the underside of the rod");
    // Nothing beyond it survives: the far part of the long tool isn't kept.
    let (_, hi) = lowest_highest(&o.bodies[1].body, Vec3::Z);
    assert!(hi < 30.7, "{hi}");
    let _ = rod;
}

#[test]
fn up_to_a_cylinder_from_inside_it() {
    // A tube's bore, r = 20 around z = 50 along y, and a block going up from its center to the wall.
    let mut doc = Document::new();
    let rod = doc.add(Op::solid(Primitive::Cylinder { radius: 20.0.into(), height: 200.0.into() }, Placement::new(point3(0.0, -100.0, 50.0), point3(0.0, 1.0, 0.0), point3(1.0, 0.0, 0.0)), Mode::NewBody)).unwrap();
    let side = {
        let o = ok(&mut doc);
        let b = &o.bodies[0].body;
        face_keys(b)[b.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder { .. })).unwrap()].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(50.0), {
        let mut rect = Sketch::new();
        let (a, b, c, d) = (rect.line([-5.0, -8.0], [5.0, -8.0]), rect.line([5.0, -8.0], [5.0, 8.0]), rect.line([5.0, 8.0], [-5.0, 8.0]), rect.line([-5.0, 8.0], [-5.0, -8.0]));
        for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
            rect.constrain(Rule::Coincident { a: p.end(), b: q.start() });
        }
        rect
    });
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&side)), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    let x: f64 = 5.0;
    let r: f64 = 20.0;
    // The integral of sqrt(r^2 - x^2) from -x to x.
    let want = 16.0 * (x * (r * r - x * x).sqrt() + r * r * (x / r).asin());
    near(vol(&o.bodies[1].body), want, 1e-5, "from the axis out to the wall");
    let _ = rod;
}

#[test]
fn up_to_a_sphere_takes_its_cap_off() {
    // A sphere r = 20 at z = 50, and a circle r = 5 going up to its underside.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Sphere { radius: 20.0.into() }, Placement::at(point3(0.0, 0.0, 50.0)), Mode::NewBody)).unwrap();
    let name = {
        let o = ok(&mut doc);
        face_keys(&o.bodies[0].body)[0].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    // Its underside is at height 50 - sqrt(400 - p^2), at p from the axis. Integrated over the circle:
    let want = PI * 25.0 * 50.0 - 2.0 * PI * (400f64.powf(1.5) - 375f64.powf(1.5)) / 3.0;
    near(vol(&o.bodies[1].body), want, 1e-5, "a cylinder with a dished top");
}

#[test]
fn up_to_a_cone_comes_down_onto_its_slope() {
    // A cone narrowing upward, with a tip at z = 80 and its height at radius p being 80 - 2p. A circle off to one side comes
    // down from z = 75, below the tip. Integrated here, without the kernel: the height of the tool over the circle.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Cone { r0: 30.0.into(), r1: 10.0.into(), height: 40.0.into() }, Placement::xy(20.0), Mode::NewBody)).unwrap();
    let name = {
        let o = ok(&mut doc);
        let b = &o.bodies[0].body;
        face_keys(b)[b.faces.iter().position(|f| matches!(f.surface, Surface::Cone { .. })).unwrap()].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(75.0), circle_sketch(20.0, 0.0, 5.0));
    let e = doc.add(backward(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody))).unwrap();
    let o = ok(&mut doc);
    let (n, mut sum) = (800, 0.0);
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-5.0 + 10.0 * (i as f64 + 0.5) / n as f64, -5.0 + 10.0 * (j as f64 + 0.5) / n as f64);
            if x * x + y * y < 25.0 {
                sum += 75.0 - (80.0 - 2.0 * ((20.0 + x) * (20.0 + x) + y * y).sqrt());
            }
        }
    }
    let want = sum * (10.0 / n as f64) * (10.0 / n as f64);
    near(vol(&o.bodies[1].body), want, 0.1, "down onto the slope");
    // From above the tip, it would have to go around the tip, which isn't built.
    doc.edit(s, Op::ConstrainedSketch { plane: Placement::xy(100.0), sketch: circle_sketch(0.0, 0.0, 5.0) }).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("tip"), "{why}");
    let _ = e;
}

#[test]
fn up_to_a_torus_is_valid_and_ends_on_it() {
    // A ring around z at height 50, major 40, minor 10, and a small circle under the lowest line of its tube. Its volume is
    // integrated here as the height of the tube's underside over the circle, which the kernel has no part in.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Torus { major: 40.0.into(), minor: 10.0.into() }, Placement::at(point3(0.0, 0.0, 50.0)), Mode::NewBody)).unwrap();
    let name = {
        let o = ok(&mut doc);
        face_keys(&o.bodies[0].body)[0].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(40.0, 0.0, 3.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    let (n, mut sum) = (800, 0.0);
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-3.0 + 6.0 * (i as f64 + 0.5) / n as f64, -3.0 + 6.0 * (j as f64 + 0.5) / n as f64);
            if x * x + y * y < 9.0 {
                let from_axis = ((40.0 + x) * (40.0 + x) + y * y).sqrt();
                sum += 50.0 - (100.0 - (from_axis - 40.0) * (from_axis - 40.0)).sqrt();
            }
        }
    }
    let want = sum * (6.0 / n as f64) * (6.0 / n as f64);
    near(vol(&o.bodies[1].body), want, 1e-1, "under the ring");
}

#[test]
fn up_to_a_face_refuses_what_it_cant_do_and_says_why() {
    let with_plate = |doc: &mut Document| doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    // Behind the sketch.
    let mut doc = Document::new();
    let plate = with_plate(&mut doc);
    let s = sketch_on(&mut doc, Placement::xy(100.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(FaceKey::plain(plate.start_cap())), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("opposite"));
    // There's no such face.
    let mut doc = Document::new();
    with_plate(&mut doc);
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key("f1:side.77")), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("f1:side.77"));
    // A wall running along the direction.
    let mut doc = Document::new();
    let plate = with_plate(&mut doc);
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(100.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(FaceKey::plain(plate.side(0))), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("runs along"));
    // A surface the sketch sits across.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Sphere { radius: 20.0.into() }, Placement::at(point3(0.0, 0.0, 0.0)), Mode::NewBody)).unwrap();
    let name = {
        let o = ok(&mut doc);
        face_keys(&o.bodies[0].body)[0].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 0.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("crosses"));
    // A sketch wider than the sphere it's going up to: part of it would go on past, and doesn't.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Sphere { radius: 10.0.into() }, Placement::at(point3(0.0, 0.0, 50.0)), Mode::NewBody)).unwrap();
    let name = {
        let o = ok(&mut doc);
        face_keys(&o.bodies[0].body)[0].text().split('#').next().unwrap().to_string()
    };
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 25.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToFace(key(&name)), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("goes past"));
}

// ----- Up to next -----

#[test]
fn up_to_next_stops_at_the_first_thing_in_the_way() {
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 60.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[2].body), PI * 25.0 * 30.0, 1e-6, "to the nearer plate");
}

#[test]
fn up_to_next_merges_with_what_it_reaches_and_each_region_goes_its_own_way() {
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    doc.add(solid(40.0, 40.0, 2.0, [100.0, 0.0, 50.0])).unwrap();
    let mut two = Sketch::new();
    two.circle([20.0, 20.0], 5.0);
    two.circle([120.0, 20.0], 4.0);
    let s = sketch_on(&mut doc, Placement::xy(0.0), two);
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::Add)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    let plate = 40.0 * 40.0 * 2.0;
    near(total(&o), 2.0 * plate + PI * 25.0 * 30.0 + PI * 16.0 * 50.0, 1e-6, "each up to its own plate");
}

#[test]
fn up_to_next_follows_what_moves() {
    let mut doc = Document::new();
    doc.set_parameter("gap", 30.0).unwrap();
    doc.add(Op::solid(Primitive::Box { dx: 40.0.into(), dy: 40.0.into(), dz: 2.0.into() }, Placement::xy("gap"), Mode::NewBody)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 30.0, 1e-6, "at 30");
    doc.set_parameter("gap", 41.0).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 41.0, 1e-6, "at 41");
}

#[test]
fn up_to_next_from_a_face_doesnt_stop_where_it_starts() {
    // Sketched on the top of a block, going up to a plate above.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(10.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[2].body), PI * 25.0 * 20.0, 1e-6, "from 10 to 30");
}

#[test]
fn a_cut_up_to_next_goes_through_to_the_far_side_of_the_material() {
    // A block with a cavity: 0 to 10 solid, 10 to 20 hollow, 20 to 30 solid. Cut from the top, it goes through the roof and
    // stops there, leaving the floor.
    let mut doc = Document::new();
    let block = doc.add(solid(40.0, 40.0, 30.0, [0.0, 0.0, 0.0])).unwrap();
    doc.add(Op::solid(Primitive::Box { dx: 20.0.into(), dy: 20.0.into(), dz: 10.0.into() }, Placement::at(point3(10.0, 10.0, 10.0)), Mode::Remove)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(30.0), circle_sketch(20.0, 20.0, 5.0));
    let cut = extrude_to(s, ExtrudeEnd::UpToNext, Mode::Remove);
    doc.add(backward(cut)).unwrap();
    let o = ok(&mut doc);
    let whole = 40.0 * 40.0 * 30.0 - 20.0 * 20.0 * 10.0;
    near(total(&o), whole - PI * 25.0 * 10.0, 1e-6, "through the roof only");
    let _ = block;
}

#[test]
fn a_cut_wider_than_a_cavity_stops_at_the_far_side_of_the_roof_and_not_at_the_floor() {
    // The same block, and a cut wider than the cavity, so that where it stops shows. Past the roof it would take material
    // from around the cavity as well.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 30.0, [0.0, 0.0, 0.0])).unwrap();
    doc.add(Op::solid(Primitive::Box { dx: 20.0.into(), dy: 20.0.into(), dz: 10.0.into() }, Placement::at(point3(10.0, 10.0, 10.0)), Mode::Remove)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(30.0), circle_sketch(20.0, 20.0, 15.0));
    doc.add(backward(extrude_to(s, ExtrudeEnd::UpToNext, Mode::Remove))).unwrap();
    let whole = 40.0 * 40.0 * 30.0 - 20.0 * 20.0 * 10.0;
    near(total(&ok(&mut doc)), whole - PI * 225.0 * 10.0, 1e-6, "the roof's thickness and no more");
}

#[test]
fn a_sketch_a_hair_off_a_face_doesnt_stop_there() {
    // Above the block's top by less than can be told apart from on it, going down through it to a plate below.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, -30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(10.0 + 5e-8), circle_sketch(20.0, 20.0, 5.0));
    doc.add(backward(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody))).unwrap();
    near(vol(&ok(&mut doc).bodies[2].body), PI * 25.0 * 38.0, 1e-4, "from the top down to the plate at -28");
}

#[test]
fn a_cut_through_all_leaves_no_sliver_past_a_curved_part() {
    // A rod along y, r = 10, centred at z = 10, cut from below by a rectangle across its top line, which is on no vertex.
    // Ending where the rod's mesh seems to end could leave a sliver along the top.
    for turned in [0.0f64, 30.0, 47.0, 73.0] {
        let x = point3(turned.to_radians().cos(), 0.0, turned.to_radians().sin());
        let rod = Placement::new(point3(0.0, -100.0, 10.0), point3(0.0, 1.0, 0.0), x);
        let mut doc = Document::new();
        doc.add(Op::solid(Primitive::Cylinder { radius: 10.0.into(), height: 200.0.into() }, rod, Mode::NewBody)).unwrap();
        let mut rect = Sketch::new();
        let (a, b, c, d) = (rect.line([-5.0, -20.0], [5.0, -20.0]), rect.line([5.0, -20.0], [5.0, 20.0]), rect.line([5.0, 20.0], [-5.0, 20.0]), rect.line([-5.0, 20.0], [-5.0, -20.0]));
        for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
            rect.constrain(Rule::Coincident { a: p.end(), b: q.start() });
        }
        let s = sketch_on(&mut doc, Placement::xy(-5.0), rect);
        doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::Remove)).unwrap();
        let o = ok(&mut doc);
        // Between x = -5 and 5 the rod's section has an area of 2 * integral of sqrt(100 - x^2).
        let (w, r): (f64, f64) = (5.0, 10.0);
        let section = 2.0 * (w * (r * r - w * w).sqrt() + r * r * (w / r).asin());
        near(total(&o), PI * 100.0 * 200.0 - section * 40.0, 1e-5, "with the middle gone");
        // The two ends joined by what's left of the rod either side of the cut: one part, and nothing more.
        assert_eq!(o.bodies.len(), 1, "at {turned}: {:?}", o.bodies.iter().map(|b| vol(&b.body)).collect::<Vec<_>>());
    }
}

#[test]
fn up_to_next_with_a_hole_in_the_way_stops_at_the_surface_of_what_it_met() {
    // The plate has a hole in it, and some of the sketch's rays go through that to a further plate. It ends on the nearer.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    doc.add(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 10.0.into() }, Placement::at(point3(20.0, 20.0, 25.0)), Mode::Remove)).unwrap();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 60.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies.last().unwrap().body), PI * 25.0 * 30.0, 1e-6, "up to the nearer plate");
}

#[test]
fn up_to_next_that_would_run_into_something_else_is_refused() {
    // A slope that the sketch meets first low down, and a part that the slope, extended, would run the tool into.
    let mut doc = Document::new();
    let wedge = doc.add(Op::sketch(Placement::new(point3(-50.0, 100.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![Region::new(Shape::polygon(vec![point(0.0, 30.0), point(100.0, 80.0), point(100.0, 100.0), point(0.0, 100.0)]))])).unwrap();
    doc.add(Op::extrude(wedge, 0.0, 200.0, Mode::NewBody)).unwrap();
    // The slope over this sketch is first met at z = 45 and rises to 65. A peg comes down to z = 50 at a place where the
    // slope is at 60, so it isn't the nearest thing, and the slope, as the end, would run the tool up into it.
    doc.add(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 40.0.into() }, Placement::at(point3(10.0, 0.0, 50.0)), Mode::NewBody)).unwrap();
    let mut wide = Sketch::new();
    wide.circle([0.0, 0.0], 20.0);
    let s = sketch_on(&mut doc, Placement::xy(0.0), wide);
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("run into"), "{why}");
}

#[test]
fn up_to_next_with_nothing_ahead_says_so() {
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(50.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("opposite"));
    // Backward, there it is, and it comes down onto the plate's top.
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(50.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(backward(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody))).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 18.0, 1e-6, "from 50 down to 32");
}

// ----- Up to a part and a vertex -----

#[test]
fn up_to_a_part_goes_on_past_whatever_else_is_in_the_way() {
    let mut doc = Document::new();
    doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let far = doc.add(solid(40.0, 40.0, 2.0, [0.0, 0.0, 60.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToPart(BodyId { feature: far, index: 0 }), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[2].body), PI * 25.0 * 60.0, 1e-6, "to the far plate");
}

#[test]
fn a_cut_up_to_a_part_stops_short_of_it() {
    let mut doc = Document::new();
    let near_block = doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 0.0])).unwrap();
    let far = doc.add(solid(40.0, 40.0, 10.0, [0.0, 0.0, 20.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(-5.0), circle_sketch(20.0, 20.0, 5.0));
    doc.add(extrude_to(s, ExtrudeEnd::UpToPart(BodyId { feature: far, index: 0 }), Mode::Remove)).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[0].body), 16000.0 - PI * 25.0 * 10.0, 1e-6, "cut right through");
    near(vol(&o.bodies[1].body), 16000.0, 1e-6, "left alone");
    let _ = near_block;
}

#[test]
fn up_to_a_part_refuses_one_that_isnt_there_or_in_the_way() {
    let mut doc = Document::new();
    let part = doc.add(solid(10.0, 10.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(500.0, 0.0, 5.0));
    let e = doc.add(extrude_to(s, ExtrudeEnd::UpToPart(BodyId { feature: part, index: 0 }), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("never meets"));
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToPart(BodyId { feature: part, index: 7 }), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("isn't there"));
}

#[test]
fn up_to_a_vertex_ends_on_a_plane_through_it() {
    let mut doc = Document::new();
    doc.set_parameter("tall", 30.0).unwrap();
    let block = doc.add(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 20.0.into(), dz: "tall".into() }, Placement::xy(0.0), Mode::NewBody)).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(100.0, 0.0, 5.0));
    let corner = VertexRef::Faces { faces: vec![FaceKey::plain(block.end_cap()), FaceKey::plain(block.side(0)), FaceKey::plain(block.side(1))], near: None };
    let e = doc.add(extrude_to(s, ExtrudeEnd::UpToVertex(corner), Mode::NewBody)).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 30.0, 1e-6, "as tall as the block");
    // It follows.
    doc.set_parameter("tall", 44.0).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 44.0, 1e-6, "as tall again");
    // Two faces meet at two vertices, so it takes something to say which.
    let two = vec![FaceKey::plain(block.side(0)), FaceKey::plain(block.side(1))];
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Faces { faces: two.clone(), near: None }), Mode::NewBody)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("near"), "{why}");
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Faces { faces: two, near: Some(point3(10.0, 0.0, 40.0)) }), Mode::NewBody)).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 44.0, 1e-6, "the one up there");
    // Faces that don't meet.
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Faces { faces: vec![FaceKey::plain(block.end_cap()), FaceKey::plain(block.start_cap())], near: None }), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("no vertex"));
    // Just a place.
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Point(point3(1.0, 2.0, 12.5))), Mode::NewBody)).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 12.5, 1e-6, "to a place");
    // Behind it.
    doc.edit(e, extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Point(point3(1.0, 2.0, -12.5))), Mode::NewBody)).unwrap();
    assert!(failure(&mut doc).contains("opposite"));
    doc.edit(e, backward(extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Point(point3(1.0, 2.0, -12.5))), Mode::NewBody))).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 12.5, 1e-6, "to a place below");
}

// ----- With the rest of it -----

#[test]
fn regions_and_an_end_work_together_and_start_counts() {
    let mut doc = Document::new();
    let plate = doc.add(solid(100.0, 100.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = sketch_on(&mut doc, Placement::xy(0.0), three_regions());
    let circle = region_of_area(&doc, s, PI * 25.0);
    let op = Op::Extrude { sketch: s, start: 10.0.into(), length: 0.0.into(), mode: Mode::NewBody, regions: vec![RegionPick { sketch: s, region: circle }], end: ExtrudeEnd::UpToFace(FaceKey::plain(plate.start_cap())), opposite: false };
    doc.add(op).unwrap();
    near(vol(&ok(&mut doc).bodies[1].body), PI * 25.0 * 20.0, 1e-6, "from 10 up to 30");
}

#[test]
fn a_length_can_be_backward_too() {
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    doc.add(backward(Op::extrude(s, 0.0, 10.0, Mode::NewBody))).unwrap();
    let o = ok(&mut doc);
    let (lo, hi) = lowest_highest(&o.bodies[0].body, Vec3::Z);
    near(lo, -10.0, 1e-9, "below");
    near(hi, 0.0, 1e-9, "from the sketch");
}

/// The surface of an extrude's cap, by its name.
fn cap(o: &Outcome, body: usize, slot: u32, feature: FeatureId) -> Surface {
    o.bodies[body].body.faces.iter().find(|f| f.tag == FaceTag { feature: feature.0, slot }).unwrap_or_else(|| panic!("no cap {slot}")).surface.clone()
}

#[test]
fn the_far_end_is_named_as_a_blind_extrudes_is() {
    // Going forward, the far end is the end cap. Going the other way it's the start cap, as caps go by where they are.
    let sphere = |z: f64| Op::solid(Primitive::Sphere { radius: 20.0.into() }, Placement::at(point3(0.0, 0.0, z)), Mode::NewBody);
    let sphere_name = |doc: &mut Document| {
        let o = ok(doc);
        key(face_keys(&o.bodies[0].body)[0].text().split('#').next().unwrap())
    };
    let mut doc = Document::new();
    doc.add(sphere(50.0)).unwrap();
    let k = sphere_name(&mut doc);
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    let e = doc.add(extrude_to(s, ExtrudeEnd::UpToFace(k), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert!(matches!(cap(&o, 1, 1, e), Surface::Sphere { .. }));
    assert!(matches!(cap(&o, 1, 0, e), Surface::Plane(_)));

    let mut doc = Document::new();
    doc.add(sphere(-50.0)).unwrap();
    let k = sphere_name(&mut doc);
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    let e = doc.add(backward(extrude_to(s, ExtrudeEnd::UpToFace(k), Mode::NewBody))).unwrap();
    let o = ok(&mut doc);
    assert!(matches!(cap(&o, 1, 0, e), Surface::Sphere { .. }));
    assert!(matches!(cap(&o, 1, 1, e), Surface::Plane(_)));
    // The same as a flat end going that way, or a length.
    let mut doc = Document::new();
    let s = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(0.0, 0.0, 5.0));
    let e = doc.add(backward(extrude_to(s, ExtrudeEnd::UpToVertex(VertexRef::Point(point3(0.0, 0.0, -9.0))), Mode::NewBody))).unwrap();
    let o = ok(&mut doc);
    let Surface::Plane(far) = cap(&o, 0, 0, e) else { panic!() };
    near(far.origin.z, -9.0, 1e-9, "the start cap is down there");
}

#[test]
fn a_plain_sketch_can_end_anywhere_too() {
    let mut doc = Document::new();
    doc.add(solid(100.0, 100.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let s = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0)), Region::new(Shape::rect(50.0, 0.0, 10.0, 20.0))])).unwrap();
    doc.add(extrude_to(s, ExtrudeEnd::UpToNext, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    near(total(&o) - 100.0 * 100.0 * 2.0, (100.0 + 200.0) * 30.0, 1e-6, "both, up to the plate");
    assert_eq!(o.bodies.len(), 3);
}

#[test]
fn every_kind_of_end_comes_back_from_a_file() {
    let mut doc = Document::new();
    let plate = doc.add(solid(100.0, 100.0, 2.0, [0.0, 0.0, 30.0])).unwrap();
    let a = sketch_on(&mut doc, Placement::xy(0.0), circle_sketch(20.0, 20.0, 5.0));
    let b = sketch_on(&mut doc, Placement::xy(1.0), circle_sketch(60.0, 20.0, 5.0));
    let (ra, rb) = (region_of_area(&doc, a, PI * 25.0), region_of_area(&doc, b, PI * 25.0));
    let ends = vec![
        ExtrudeEnd::ThroughAll,
        ExtrudeEnd::UpToNext,
        ExtrudeEnd::UpToFace(FaceKey::plain(plate.start_cap())),
        ExtrudeEnd::UpToPart(BodyId { feature: plate, index: 0 }),
        ExtrudeEnd::UpToVertex(VertexRef::Point(point3(0.0, 0.0, 20.0))),
        ExtrudeEnd::UpToVertex(VertexRef::Faces { faces: vec![FaceKey::plain(plate.side(0)), FaceKey::plain(plate.side(1))], near: Some(point3(0.0, 0.0, "1 + 2")) }),
    ];
    for end in ends {
        let mut d = doc.clone();
        let regions = vec![RegionPick { sketch: a, region: ra }, RegionPick { sketch: b, region: rb }];
        d.add(Op::Extrude { sketch: a, start: 0.0.into(), length: 5.0.into(), mode: Mode::NewBody, regions, end: end.clone(), opposite: true }).unwrap();
        let text = d.save_text();
        let back = Document::load_text(&text).unwrap_or_else(|e| panic!("{end:?}: {e}\n{text}"));
        assert_eq!(back.features().last().unwrap().op, d.features().last().unwrap().op, "{end:?}");
        assert_eq!(back.save_text(), text);
    }
}

#[test]
fn a_plain_extrude_is_written_as_it_always_was() {
    let mut doc = Document::new();
    let s = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))])).unwrap();
    doc.add(Op::extrude(s, 0.0, 5.0, Mode::Add)).unwrap();
    let text = doc.save_text();
    let at = text.find("\"extrude\"").unwrap();
    let extrude = &text[at..];
    assert!(!extrude.contains("\"regions\"") && !extrude.contains("\"end\"") && !extrude.contains("\"opposite\""), "{text}");
    assert!(extrude.contains("\"length\""));
    // And one that doesn't use a length doesn't write one, unless there's one to keep for going back to it.
    let mut doc = Document::new();
    let s = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))])).unwrap();
    let e = doc.add(extrude_to(s, ExtrudeEnd::ThroughAll, Mode::Add)).unwrap();
    let text = doc.save_text();
    assert!(!text[text.find("\"extrude\"").unwrap()..].contains("\"length\""), "{text}");
    let Op::Extrude { sketch, start, mode, regions, end, opposite, .. } = doc.feature(e).unwrap().op.clone() else { panic!() };
    doc.edit(e, Op::Extrude { sketch, start, length: 12.0.into(), mode, regions, end, opposite }).unwrap();
    let text = doc.save_text();
    let back = Document::load_text(&text).unwrap();
    assert!(matches!(&back.feature(e).unwrap().op, Op::Extrude { length, .. } if *length == Expr::number(12.0)), "{text}");
}

#[test]
fn a_file_with_a_bad_end_or_pick_is_refused_and_says_why() {
    let doc_with = |extrude: &str| format!("{{\"format\": \"coreshape-document\", \"version\": 1, \"units\": \"mm\", \"next_id\": 9, \"parameters\": [], \"features\": [{{\"id\": 1, \"name\": \"Sketch\", \"type\": \"sketch\", \"plane\": {{\"origin\": [0, 0, 0]}}, \"regions\": [{{\"outer\": {{\"type\": \"circle\", \"cx\": 0, \"cy\": 0, \"r\": 1}}}}]}}, {{\"id\": 2, \"name\": \"E\", {extrude}}}]}}");
    let refused = |extrude: &str, says: &str| {
        let why = Document::load_text(&doc_with(extrude)).err().unwrap_or_else(|| panic!("{extrude} was taken")).to_string();
        assert!(why.contains(says), "{extrude}: {why}");
    };
    Document::load_text(&doc_with(r#""type": "extrude", "sketch": 1, "length": 1"#)).unwrap();
    Document::load_text(&doc_with(r#""type": "extrude", "sketch": 1, "end": "through_all""#)).unwrap();
    Document::load_text(&doc_with(r#""type": "extrude", "regions": [{"sketch": 1, "region": "r0123456789ab"}], "length": 1"#)).unwrap();
    refused(r#""type": "extrude", "sketch": 1"#, "length");
    refused(r#""type": "extrude", "sketch": 1, "end": "sideways""#, "sideways");
    refused(r#""type": "extrude", "sketch": 1, "end": {"type": "up_to_face"}"#, "face");
    refused(r#""type": "extrude", "sketch": 1, "end": {"type": "up_to_face", "face": "nonsense"}"#, "nonsense");
    refused(r#""type": "extrude", "sketch": 1, "end": {"type": "up_to_part", "part": "x"}"#, "body");
    refused(r#""type": "extrude", "sketch": 1, "end": {"type": "up_to_vertex"}"#, "vertex");
    refused(r#""type": "extrude", "sketch": 1, "end": {"type": "up_to_vertex", "vertex": {"faces": []}}"#, "faces");
    refused(r#""type": "extrude", "sketch": 1, "length": 1, "regions": ["nope"]"#, "nope");
    refused(r#""type": "extrude", "length": 1, "regions": ["r0123456789ab"]"#, "sketch");
    refused(r#""type": "extrude", "sketch": 1, "length": 1, "regions": [{"sketch": 1}]"#, "region");
    refused(r#""type": "extrude", "sketch": 1, "length": 1, "opposite": 1"#, "opposite");
    refused(r#""type": "extrude", "sketch": 1, "length": 1, "regions": 3"#, "regions");
}
