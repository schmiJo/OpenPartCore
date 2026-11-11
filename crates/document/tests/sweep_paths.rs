//! Sweeping along curves, as Onshape's sweep does: a sketch's curves or a part's edges, in any order and either way round.
//! The profile goes from the end of the path in its plane and stays square to the path. Volumes are held against Pappus:
//! a profile's area times how far its centroid travels.

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

fn span(b: &Body, along: Vec3) -> (f64, f64) {
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    mesh.positions.iter().map(|p| p.dot(along)).fold((f64::MAX, f64::MIN), |(lo, hi), h| (lo.min(h), hi.max(h)))
}

fn circle(r: f64) -> Region {
    Region::new(Shape::Circle { cx: 0.0.into(), cy: 0.0.into(), r: r.into() })
}

/// The XZ plane, with sketch (u, v) at world (u, 0, v).
fn xz() -> Placement {
    Placement::new(point3(0.0, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0))
}

fn open(start: [f64; 2], moves: Vec<Move>) -> Region {
    Region::new(Shape::OpenPath { start: point(start[0], start[1]), moves })
}

fn line(x: f64, y: f64) -> Move {
    Move::Line { to: point(x, y) }
}

fn arc(x: f64, y: f64, cx: f64, cy: f64, ccw: bool) -> Move {
    Move::Arc { to: point(x, y), center: point(cx, cy), ccw }
}

fn sweep(profile: FeatureId, path: SweepPath, mode: Mode) -> Op {
    Op::Sweep { sketch: profile, path, mode }
}

/// Up 10, a quarter bend of radius 5 toward +x, and on 8, in the XZ plane.
fn bent_path() -> Vec<Region> {
    vec![open([0.0, 0.0], vec![line(0.0, 10.0), arc(5.0, 15.0, 5.0, 10.0, false), line(13.0, 15.0)])]
}

#[test]
fn along_a_sketchs_curves() {
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(1.0)])).unwrap();
    let path = doc.add(Op::sketch(xz(), bent_path())).unwrap();
    doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), PI * (10.0 + 2.5 * PI + 8.0), 1e-6, "volume");
    let body = o.body.as_ref().unwrap();
    near(span(body, Vec3::X).1, 13.0, 1e-9, "x reach");
    near(span(body, Vec3::Z).1, 16.0, 1e-9, "z reach");
    // Named from the profile, as a sweep by steps is.
    assert!(!find_faces(body, &FaceKey::parse("f3:side.0").unwrap()).is_empty());
    assert_eq!(find_faces(body, &FaceKey::parse("f3:start").unwrap()).len(), 1);
    assert_eq!(find_faces(body, &FaceKey::parse("f3:end").unwrap()).len(), 1);
}

#[test]
fn the_profile_keeps_its_place_across_the_path() {
    // A 2 by 2 square whose middle is 1 toward the bend's center: its middle goes round at radius 4.
    let mut doc = Document::new();
    let square = Region::new(Shape::Rect { x: 0.0.into(), y: (-1.0).into(), w: 2.0.into(), h: 2.0.into() });
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![square])).unwrap();
    let path = doc.add(Op::sketch(xz(), bent_path())).unwrap();
    doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    near(total(&ok(&mut doc)), 4.0 * (10.0 + 4.0 * PI / 2.0 + 8.0), 1e-6, "volume");
}

#[test]
fn it_starts_from_whichever_end_is_at_the_profile_and_goes_either_way() {
    // The same path drawn the other way round, from its far end.
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(1.0)])).unwrap();
    let path = doc.add(Op::sketch(xz(), vec![open([13.0, 15.0], vec![line(5.0, 15.0), arc(0.0, 10.0, 5.0, 10.0, true), line(0.0, 0.0)])])).unwrap();
    doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    near(total(&ok(&mut doc)), PI * (10.0 + 2.5 * PI + 8.0), 1e-6, "volume");

    // Down, behind the profile's plane.
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(1.0)])).unwrap();
    let path = doc.add(Op::sketch(xz(), vec![open([0.0, 0.0], vec![line(0.0, -10.0)])])).unwrap();
    doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), PI * 10.0, 1e-9, "volume");
    let (lo, hi) = span(o.body.as_ref().unwrap(), Vec3::Z);
    near(lo, -10.0, 1e-9, "bottom");
    near(hi, 0.0, 1e-9, "top");
}

#[test]
fn along_a_closed_curve_it_closes() {
    // A ring: a circle of radius 1 around a circle of radius 10, starting where that crosses the profile's plane.
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::new(point3(10.0, 0.0, 0.0), point3(0.0, 1.0, 0.0), point3(1.0, 0.0, 0.0)), vec![circle(1.0)])).unwrap();
    let path = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(10.0)])).unwrap();
    doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    let o = ok(&mut doc);
    near(total(&o), 2.0 * PI * PI * 10.0, 1e-6, "volume");
    let body = o.body.as_ref().unwrap();
    assert!(find_faces(body, &FaceKey::parse("f3:start").unwrap()).is_empty(), "a closed ring has no ends");
}

#[test]
fn along_a_parts_edges() {
    // A rounded block's top outline is lines and quarter circles, smooth all round. A ring around it, as a part of its own.
    let mut doc = Document::new();
    let plate = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::RoundedRect { cx: 0.0.into(), cy: 0.0.into(), w: 40.0.into(), h: 20.0.into(), r: 5.0.into() })])).unwrap();
    let block = doc.add(Op::extrude(plate, 0.0, 5.0, Mode::Add)).unwrap();
    // Square to the outline's straight side along x, through its middle.
    let profile = doc.add(Op::sketch(Placement::new(point3(0.0, -10.0, 5.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![circle(1.0)])).unwrap();
    doc.add(sweep(profile, SweepPath::Edges(EdgeSelect::OnFace(block.end_cap())), Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(vol(&o.bodies[1].body), PI * (2.0 * 30.0 + 2.0 * 10.0 + 2.0 * PI * 5.0), 1e-6, "ring");
    // It follows the block when the block changes.
    let Op::Sketch { plane, regions } = doc.feature(plate).unwrap().op.clone() else { panic!() };
    let wider = regions.into_iter().map(|_| Region::new(Shape::RoundedRect { cx: 0.0.into(), cy: 0.0.into(), w: 50.0.into(), h: 20.0.into(), r: 5.0.into() })).collect();
    doc.edit(plate, Op::Sketch { plane, regions: wider }).unwrap();
    let o = ok(&mut doc);
    near(vol(&o.bodies[1].body), PI * (2.0 * 40.0 + 2.0 * 10.0 + 2.0 * PI * 5.0), 1e-6, "wider ring");
}

#[test]
fn what_it_cant_follow_it_says() {
    let attempt = |profile_plane: Placement, path: Vec<Region>| -> String {
        let mut doc = Document::new();
        let profile = doc.add(Op::sketch(profile_plane, vec![circle(1.0)])).unwrap();
        let path = doc.add(Op::sketch(xz(), path)).unwrap();
        doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
        failure(&mut doc)
    };
    // Starting away from the profile's plane.
    let why = attempt(Placement::xy(0.0), vec![open([0.0, 2.0], vec![line(0.0, 10.0)])]);
    assert!(why.contains("starts 2.0000 from the profile's plane"), "{why}");
    // Leaving it at a slant.
    let why = attempt(Placement::xy(0.0), vec![open([0.0, 0.0], vec![line(5.0, 5.0)])]);
    assert!(why.contains("45.000 degrees from square"), "{why}");
    // A corner.
    let why = attempt(Placement::xy(0.0), vec![open([0.0, 0.0], vec![line(0.0, 10.0), line(5.0, 10.0)])]);
    assert!(why.contains("corner of 90.000 degrees at (0.0000, 0.0000, 10.0000)"), "{why}");
    // A spline.
    let why = attempt(Placement::xy(0.0), vec![open([0.0, 0.0], vec![Move::Spline { through: vec![point(1.0, 5.0), point(0.0, 10.0)] }])]);
    assert!(why.contains("lines and arcs for now"), "{why}");
    // Two pieces apart.
    let why = attempt(Placement::xy(0.0), vec![open([0.0, 0.0], vec![line(0.0, 10.0)]), open([5.0, 20.0], vec![line(5.0, 30.0)])]);
    assert!(why.contains("don't join"), "{why}");

    // A part's edges with corners, as a block's top outline has.
    let mut doc = Document::new();
    let block = doc.add(Op::solid(Primitive::Box { dx: 10.0.into(), dy: 6.0.into(), dz: 4.0.into() }, Placement::xy(0.0), Mode::Add)).unwrap();
    let profile = doc.add(Op::sketch(Placement::new(point3(5.0, 0.0, 4.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![circle(0.5)])).unwrap();
    doc.add(sweep(profile, SweepPath::Edges(EdgeSelect::OnFace(block.end_cap())), Mode::NewBody)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("corner of 90.000 degrees"), "{why}");
}

#[test]
fn json_takes_each_kind_of_path() {
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]);
    for text in [
        r#"{"type": "sweep", "sketch": 1, "path": [{"line": 10}, {"arc_radius": 5, "degrees": 90, "toward_degrees": 0}], "mode": "add"}"#,
        r#"{"type": "sweep", "sketch": 1, "path": {"sketch": 2}, "mode": "add"}"#,
        r#"{"type": "sweep", "sketch": 1, "path": {"edges": {"on_face": "f3:end"}}, "mode": "new_body"}"#,
    ] {
        let op = read(text).unwrap();
        assert_eq!(read(&op_value(&op).to_text()).unwrap(), op, "{text}");
    }
    assert!(matches!(read(r#"{"type": "sweep", "sketch": 1, "path": {"sketch": 2}}"#).unwrap(), Op::Sweep { path: SweepPath::Sketch(FeatureId(2)), .. }));
    let why = read(r#"{"type": "sweep", "sketch": 1, "path": {"curve": 2}}"#).unwrap_err().to_string();
    assert!(why.contains("{\"sketch\": 5}"), "{why}");
}

#[test]
fn a_path_sketch_is_needed_before_it_and_is_held_on_to() {
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(1.0)])).unwrap();
    let path = doc.add(Op::sketch(xz(), bent_path())).unwrap();
    let s = doc.add(sweep(profile, SweepPath::Sketch(path), Mode::Add)).unwrap();
    assert!(doc.delete(path).is_err());
    assert_eq!(doc.dependents(path), vec![s]);
    // A path that isn't a sketch is refused when it's added.
    let mut doc = Document::new();
    let profile = doc.add(Op::sketch(Placement::xy(0.0), vec![circle(1.0)])).unwrap();
    let solid = doc.add(Op::solid(Primitive::Box { dx: 1.0.into(), dy: 1.0.into(), dz: 1.0.into() }, Placement::xy(5.0), Mode::Add)).unwrap();
    assert!(doc.add(sweep(profile, SweepPath::Sketch(solid), Mode::Add)).is_err());
}
