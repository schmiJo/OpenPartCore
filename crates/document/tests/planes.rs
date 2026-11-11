//! Plane features, as Onshape's: each type placed from what it names, checked against where it should be by hand, and
//! following what it's made from when that changes. Sketches sit on planes and follow them too.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::{Rule, Sketch};

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
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

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    let op = read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"));
    doc.add(op).unwrap()
}

fn close(a: Vec3, b: [f64; 3], what: &str) {
    assert!(a.dist(v3(b[0], b[1], b[2])) < 1e-9, "{what}: {a:?} isn't {b:?}");
}

/// A plane's frame after a rebuild, checked against its origin, normal and x.
fn is(doc: &mut Document, id: FeatureId, origin: [f64; 3], z: [f64; 3], x: [f64; 3]) {
    let o = ok(doc);
    let f = doc.frame_of(id).unwrap();
    close(f.origin, origin, "origin");
    close(f.z, z, "normal");
    close(f.x, x, "x");
    close(f.y, [f.z.cross(f.x).x, f.z.cross(f.x).y, f.z.cross(f.x).z], "y");
    // The outcome says the same of a plane feature.
    if let Some(at) = o.planes.iter().find(|p| p.id == id) {
        assert_eq!(at.frame, f);
    }
}

/// A block from the origin, `w` by 6 by 4, made by extruding a rectangle: `f2:start` is its bottom, `f2:end` its top, and
/// `f2:side.0` to `side.3` face -Y, +X, +Y and -X.
fn block() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 6.0))]));
    doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    doc
}

fn extent(b: &Body) -> Aabb {
    let mut out = Aabb::EMPTY;
    for p in &b.vertices {
        out.add(*p);
    }
    out
}

fn last_extent(o: &Outcome) -> Aabb {
    extent(&o.bodies.last().unwrap().body)
}

/// A sketch feature needs a closed region of its own, so ones that are only for their points get a small circle well away.
fn with_region(mut s: Sketch) -> Sketch {
    s.circle([100.0, 100.0], 1.0);
    s
}

#[test]
fn sketches_sit_on_the_standard_planes_as_onshape_has_them() {
    for (name, z, x, low, high) in [
        ("top", [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [10.0, 5.0, 3.0]),
        ("front", [0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, -3.0, 0.0], [10.0, 0.0, 5.0]),
        ("right", [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [3.0, 10.0, 5.0]),
    ] {
        let mut doc = Document::new();
        let plane = read_placement(&json::parse(&format!(r#"{{"on_plane": "{name}"}}"#)).unwrap()).unwrap();
        let s = doc.with(Op::sketch(plane, vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 5.0))]));
        doc.with(Op::extrude(s, 0.0, 3.0, Mode::Add));
        is(&mut doc, s, [0.0; 3], z, x);
        let b = last_extent(&ok(&mut doc));
        close(b.min, low, name);
        close(b.max, high, name);
        // Saved as it was written.
        let text = doc.save_text();
        assert!(text.contains(&format!("\"on_plane\": \"{name}\"")), "{text}");
        assert_eq!(Document::load_text(&text).unwrap().features(), doc.features());
    }
    // A placement on a plane takes everything from it.
    let why = read_placement(&json::parse(r#"{"on_plane": "top", "origin": [0, 0, 5]}"#).unwrap()).unwrap_err().to_string();
    assert!(why.contains("on_plane"), "{why}");
}

#[test]
fn an_offset_plane_follows_the_face_it_comes_from_and_a_sketch_on_it_follows_it() {
    let mut doc = block();
    // From the +X face, 5 out: its origin is the world's origin seen along its normal, and its x is level.
    let p = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:side.1", "distance": 5}"#);
    is(&mut doc, p, [15.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let s = doc.with(Op::sketch(Placement::on_plane(PlaneRef::Feature(p)), vec![Region::new(Shape::rect(1.0, 1.0, 2.0, 2.0))]));
    doc.with(Op::extrude(s, 0.0, 1.0, Mode::NewBody));
    let b = last_extent(&ok(&mut doc));
    close(b.min, [15.0, 1.0, 1.0], "the boss on the plane");
    close(b.max, [16.0, 3.0, 3.0], "the boss on the plane");
    // From Top, by a parameter: nothing about the part moves it.
    doc.set_parameter("d", 7.0).unwrap();
    let up = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": "d"}"#);
    is(&mut doc, up, [0.0, 0.0, 7.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    // The block widens: the face's plane goes with it, and the sketch with the plane, but the plane from Top is reused.
    doc.set_parameter("w", 20.0).unwrap();
    let o = ok(&mut doc);
    let state = |id: FeatureId| o.steps.iter().find(|s| s.id == id).unwrap().state.clone();
    assert_eq!(state(p), StepState::Rebuilt);
    assert_eq!(state(s), StepState::Rebuilt);
    assert_eq!(state(up), StepState::Reused);
    is(&mut doc, p, [25.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let b = last_extent(&ok(&mut doc));
    close(b.min, [25.0, 1.0, 1.0], "the boss after the change");
    // Flipped, it faces the other way from the same origin, with the same x.
    let back = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 2, "flip": true}"#);
    is(&mut doc, back, [0.0, 0.0, 2.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]);
    // An offset from an offset.
    let twice = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "offset", "plane": {}, "distance": -3}}"#, back.0));
    is(&mut doc, twice, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]);
}

#[test]
fn a_face_away_from_the_origin_gives_a_plane_where_the_origin_is_seen_along_its_normal() {
    // A block whose sketch sits at (20, 30, 5): its faces' own surfaces start there, and its planes don't.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::at(point3(20.0, 30.0, 5.0)), vec![Region::new(Shape::rect(1.0, 2.0, 4.0, 4.0))]));
    doc.with(Op::extrude(s, 0.0, 3.0, Mode::Add));
    let top = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:end", "distance": 0}"#);
    is(&mut doc, top, [0.0, 0.0, 8.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let side = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:side.1", "distance": 0}"#);
    is(&mut doc, side, [25.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let back = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:side.2", "distance": 0}"#);
    is(&mut doc, back, [0.0, 36.0, 0.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]);
}

#[test]
fn a_plane_through_a_point_and_one_through_three_corners() {
    let mut doc = block();
    let corner = |faces: &str| format!(r#"{{"faces": [{faces}]}}"#);
    let top_corner = corner(r#""f2:end", "f2:side.1", "f2:side.2""#);
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "plane_point", "plane": "top", "point": {top_corner}}}"#));
    is(&mut doc, p, [10.0, 6.0, 4.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    // A slant through three corners: (w, 0, 0), (0, 6, 0) and (0, 0, 4). Its normal is (b - a) x (c - a) = (24, 4w, 6w).
    let (a, b, c) = (corner(r#""f2:start", "f2:side.0", "f2:side.1""#), corner(r#""f2:start", "f2:side.2", "f2:side.3""#), corner(r#""f2:end", "f2:side.0", "f2:side.3""#));
    let slant = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "three_point", "points": [{a}, {b}, {c}]}}"#));
    let expect = |w: f64| {
        let n = v3(24.0, 4.0 * w, 6.0 * w).normalized();
        let x = v3(-w, 6.0, 0.0).normalized();
        ([w, 0.0, 0.0], [n.x, n.y, n.z], [x.x, x.y, x.z])
    };
    let (o, n, x) = expect(10.0);
    is(&mut doc, slant, o, n, x);
    doc.set_parameter("w", 30.0).unwrap();
    let (o, n, x) = expect(30.0);
    is(&mut doc, slant, o, n, x);
    is(&mut doc, p, [30.0, 6.0, 4.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
}

#[test]
fn three_points_from_a_sketch_follow_it_and_bad_ones_are_named() {
    // Points on a sketch on a plane above Top: the three-point plane follows the plane through the sketch.
    let mut doc = Document::new();
    doc.set_parameter("d", 3.0).unwrap();
    let up = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": "d"}"#);
    let mut s = Sketch::new();
    let (p, q) = (s.point(1.0, 1.0), s.point(1.0, 4.0));
    let l = s.line([5.0, 1.0], [8.0, 1.0]);
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::on_plane(PlaneRef::Feature(up)), sketch: with_region(s) }).unwrap();
    let three = |a: &str, b: &str, c: &str| format!(r#"{{"type": "plane", "plane_type": "three_point", "points": [{{"sketch": {0}, "point": "{a}"}}, {{"sketch": {0}, "point": "{b}"}}, {{"sketch": {0}, "point": "{c}"}}]}}"#, sk.0);
    let plane = feature(&mut doc, &three(&p.0.to_string(), &format!("{}.start", l.0), &q.0.to_string()));
    is(&mut doc, plane, [1.0, 1.0, 3.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    doc.set_parameter("d", 9.5).unwrap();
    is(&mut doc, plane, [1.0, 1.0, 9.5], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    // Three in a line.
    let op = read_op(&json::parse(&three(&p.0.to_string(), &format!("{}.start", l.0), &format!("{}.end", l.0))).unwrap(), &[]).unwrap();
    doc.edit(plane, op).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("in a line"), "{why}");
    // Two the same.
    let op = read_op(&json::parse(&three(&p.0.to_string(), &p.0.to_string(), &q.0.to_string())).unwrap(), &[]).unwrap();
    doc.edit(plane, op).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("first and second points are the same point"), "{why}");
    // A point the sketch doesn't have, and a part a line doesn't have.
    let op = read_op(&json::parse(&three(&p.0.to_string(), "77", &q.0.to_string())).unwrap(), &[]).unwrap();
    doc.edit(plane, op).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("no entity 77"), "{why}");
    let op = read_op(&json::parse(&three(&p.0.to_string(), &format!("{}.center", l.0), &q.0.to_string())).unwrap(), &[]).unwrap();
    doc.edit(plane, op).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("is a line, which has no a center") || why.contains("has no"), "{why}");
}

#[test]
fn mid_planes_between_planes_and_between_points() {
    let mut doc = block();
    // Between the bottom, facing down, and the top, facing up: halfway, facing as the first does.
    let mid = feature(&mut doc, r#"{"type": "plane", "plane_type": "mid_plane", "planes": ["f2:start", "f2:end"]}"#);
    is(&mut doc, mid, [0.0, 0.0, 2.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]);
    // Between two planes facing the same way.
    let (a, b) = (
        feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 2}"#),
        feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 8}"#),
    );
    let between = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "mid_plane", "planes": [{}, {}]}}"#, a.0, b.0));
    is(&mut doc, between, [0.0, 0.0, 5.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    // Top and Front meet along X: the plane bisecting them faces between +Z and -Y.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let bisect = feature(&mut doc, r#"{"type": "plane", "plane_type": "mid_plane", "planes": ["top", "front"]}"#);
    is(&mut doc, bisect, [0.0, 0.0, 0.0], [0.0, -h, h], [1.0, 0.0, 0.0]);
    // A plane tilted 60 degrees about X through (0, 0, 4), and the block's top: the bisector is tilted 30, through the line
    // where they meet.
    let (s, c) = (60f64.to_radians().sin(), 60f64.to_radians().cos());
    let tilted = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "offset", "plane": {{"point": [0, 0, 4], "normal": [0, {}, {c}]}}, "distance": 0}}"#, -s));
    let half = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "mid_plane", "planes": ["f2:end", {}]}}"#, tilted.0));
    let (s30, c30) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    is(&mut doc, half, [0.0, 0.0, 4.0], [0.0, -s30, c30], [1.0, 0.0, 0.0]);
    // Square to the line between two points, at its middle.
    let pts = feature(&mut doc, r#"{"type": "plane", "plane_type": "mid_plane", "points": ["origin", {"faces": ["f2:end", "f2:side.1", "f2:side.2"]}]}"#);
    let n = v3(10.0, 6.0, 4.0).normalized();
    let x = default_x(n);
    let x = (x - n * x.dot(n)).normalized();
    is(&mut doc, pts, [5.0, 3.0, 2.0], [n.x, n.y, n.z], [x.x, x.y, x.z]);
    let same = feature(&mut doc, r#"{"type": "plane", "plane_type": "mid_plane", "points": ["origin", [0, 0, 0]]}"#);
    let why = failure(&mut doc);
    assert!(why.contains("same point"), "{why}");
    let _ = same;
}

/// A sketch on Top with a line along X from (0, 2) to (10, 2), and one on Front going up, from (0, 0) to (0, 5).
fn with_lines() -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let mut s = Sketch::new();
    let along = s.line([0.0, 2.0], [10.0, 2.0]);
    let flat = doc.add(Op::ConstrainedSketch { plane: Placement::on_plane(PlaneRef::Top), sketch: with_region(s) }).unwrap();
    let mut s = Sketch::new();
    let up = s.line([0.0, 0.0], [0.0, 5.0]);
    let upright = doc.add(Op::ConstrainedSketch { plane: Placement::on_plane(PlaneRef::Front), sketch: with_region(s) }).unwrap();
    assert_eq!((along.0, up.0), (1, 1));
    (doc, flat, upright)
}

#[test]
fn a_plane_turned_about_a_line() {
    let (mut doc, flat, upright) = with_lines();
    let line = format!(r#"{{"sketch": {}, "line": 1}}"#, flat.0);
    // From Top, 30 degrees about +X: the normal goes from +Z toward -Y.
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "line_angle", "line": {line}, "degrees": 30}}"#));
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    is(&mut doc, p, [0.0, 2.0, 0.0], [0.0, -s, c], [1.0, 0.0, 0.0]);
    // From Front instead.
    let q = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "line_angle", "line": {line}, "degrees": 0, "reference": "front"}}"#));
    is(&mut doc, q, [0.0, 2.0, 0.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]);
    // A line going up is most nearly parallel to Front, so that's where it starts; 90 degrees about +Z turns -Y to +X.
    let up = format!(r#"{{"sketch": {}, "line": 1}}"#, upright.0);
    let r = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "line_angle", "line": {up}, "degrees": 90}}"#));
    is(&mut doc, r, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    // But no plane through it is nearer parallel to Top than another.
    let bad = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "line_angle", "line": {up}, "degrees": 10, "reference": "top"}}"#));
    let why = failure(&mut doc);
    assert!(why.contains("square to the Top plane"), "{why}");
    doc.delete(bad).unwrap();
    // A point and a line: square to the line.
    let corner = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": [3, 4, 5], "line": {line}}}"#));
    is(&mut doc, corner, [3.0, 4.0, 5.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    // A line has to be a line.
    let mut s = Sketch::new();
    s.circle([0.0, 0.0], 2.0);
    let round = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: s }).unwrap();
    feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {{"sketch": {}, "line": 1}}}}"#, round.0));
    let why = failure(&mut doc);
    assert!(why.contains("is a circle, not a line"), "{why}");
}

#[test]
fn lines_from_edges_and_axes() {
    let mut doc = block();
    // A round boss on the block, 3 in radius about (5, 3).
    let c = doc.with(Op::sketch(Placement::xy(4.0), vec![Region::new(Shape::circle(5.0, 3.0, 3.0))]));
    let boss = doc.with(Op::extrude(c, 0.0, 2.0, Mode::Add));
    let o = ok(&mut doc);
    // Square to the boss's axis through a corner: level, facing along the axis as faces_list gives it.
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": [1, 1, 1], "line": {{"axis_of": "f{}:side.0"}}}}"#, boss.0));
    let body = o.body.as_ref().unwrap();
    let f = find_faces(body, &FaceKey::parse(&format!("f{}:side.0", boss.0)).unwrap())[0];
    let (_, a) = body.faces[f].surface.axis().unwrap();
    assert!((a.z.abs() - 1.0).abs() < 1e-12);
    is(&mut doc, p, [1.0, 1.0, 1.0], [0.0, 0.0, a.z], [1.0, 0.0, 0.0]);
    // A straight edge, the one between the bottom and the -Y face, running as edges_list has it.
    let edge = r#"{"between": ["f2:start", "f2:side.0"]}"#;
    let e = select_edges(&read_edge_select(&json::parse(edge).unwrap()).unwrap(), body, &Env::new()).unwrap()[0];
    let (s, t) = (body.vertices[body.edges[e].v0], body.vertices[body.edges[e].v1]);
    let d = (t - s).normalized();
    let q = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {edge}}}"#));
    is(&mut doc, q, [0.0; 3], [d.x, d.y, d.z], [0.0, d.x, 0.0]);
    // A curved edge isn't a line, and a selection of several isn't one.
    feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {{"on_face": "f{}:end"}}}}"#, boss.0));
    let why = failure(&mut doc);
    assert!(why.contains("edges, and a line takes one") || why.contains("has to be straight"), "{why}");
}

#[test]
fn square_to_an_edge_at_a_point_or_a_fraction_of_its_length() {
    let mut doc = block();
    let edge = r#"{"between": ["f2:start", "f2:side.0"]}"#;
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "curve_point", "edge": {edge}, "fraction": 0.3}}"#));
    let o = ok(&mut doc);
    let body = o.body.as_ref().unwrap();
    let e = select_edges(&read_edge_select(&json::parse(edge).unwrap()).unwrap(), body, &Env::new()).unwrap()[0];
    let (s, t) = (body.vertices[body.edges[e].v0], body.vertices[body.edges[e].v1]);
    let at = s + (t - s) * 0.3;
    let d = (t - s).normalized();
    is(&mut doc, p, [at.x, at.y, at.z], [d.x, d.y, d.z], [0.0, d.x, 0.0]);
    let q = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "curve_point", "edge": {edge}, "point": [7, -3, 9]}}"#));
    is(&mut doc, q, [7.0, 0.0, 0.0], [d.x, d.y, d.z], [0.0, d.x, 0.0]);
    let bad = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "curve_point", "edge": {edge}, "fraction": 1.5}}"#));
    assert!(failure(&mut doc).contains("from 0 to 1"));
    doc.delete(bad).unwrap();

    // On an ellipse's rim, a fraction is of its length: held against the length of a fine polyline along it.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::Ellipse { cx: 0.0.into(), cy: 0.0.into(), rx: 10.0.into(), ry: 4.0.into(), degrees: 0.0.into() })]));
    doc.with(Op::extrude(s, 0.0, 2.0, Mode::Add));
    let o = ok(&mut doc);
    let body = o.body.as_ref().unwrap().clone();
    // The top's rim is four quarters: this is the one from +X round to +Y.
    let rim = r#"{"both": [{"on_face": "f2:end"}, {"in_box": [[0, 0, 1], [20, 20, 3]]}]}"#;
    let e = select_edges(&read_edge_select(&json::parse(rim).unwrap()).unwrap(), &body, &Env::new()).unwrap();
    assert_eq!(e.len(), 1, "one quarter of the rim");
    let edge = &body.edges[e[0]];
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "curve_point", "edge": {rim}, "fraction": 0.3}}"#));
    ok(&mut doc);
    let f = doc.frame_of(p).unwrap();
    let n = 200_000;
    let points: Vec<Vec3> = (0..=n).map(|i| edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64)).collect();
    let lengths: Vec<f64> = points.windows(2).map(|w| w[0].dist(w[1])).collect();
    let total: f64 = lengths.iter().sum();
    let (t, gap) = edge.curve.closest(f.origin, edge.t0, edge.t1);
    assert!(gap < 1e-9, "the plane's origin is on the edge");
    let upto = ((t - edge.t0) / (edge.t1 - edge.t0) * n as f64).round() as usize;
    let before: f64 = lengths[..upto].iter().sum();
    assert!((before / total - 0.3).abs() < 1e-5, "{} of the way along", before / total);
    let d = edge.curve.derivative(t).normalized();
    assert!(f.z.dist(d) < 1e-9, "square to the edge there");
}

#[test]
fn tangent_planes_on_a_cylinder_a_cone_and_a_sphere() {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 5.0))]));
    let rod = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    // Nearest a point: the cylinder's side facing it.
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.0", "point": [20, 0, 3]}}"#, rod.0));
    let o = ok(&mut doc);
    let body = o.body.as_ref().unwrap();
    let f = find_faces(body, &FaceKey::parse(&format!("f{}:side.0", rod.0)).unwrap())[0];
    let (_, a) = body.faces[f].surface.axis().unwrap();
    is(&mut doc, p, [5.0, 0.0, 3.0], [1.0, 0.0, 0.0], [0.0, 0.0, a.z]);
    // At an angle round its axis: 0 is the way a plane facing along the axis gets its x, and 90 a quarter turn on,
    // right-handed about the axis, at the height of the middle of the face.
    let q = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.0", "degrees": 90}}"#, rod.0));
    let zero = default_x(a);
    let out = a.cross((zero - a * zero.dot(a)).normalized());
    is(&mut doc, q, [out.x * 5.0, out.y * 5.0, 5.0], [out.x, out.y, out.z], [0.0, 0.0, a.z]);
    // Not on the axis, where it could touch anywhere.
    let bad = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.0", "point": [0, 0, 7]}}"#, rod.0));
    assert!(failure(&mut doc).contains("on the axis"));
    doc.delete(bad).unwrap();
    // A flat face isn't round.
    let bad = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:end", "point": [20, 0, 3]}}"#, rod.0));
    let why = failure(&mut doc);
    assert!(why.contains("is flat, and a tangent plane touches a cylinder, cone or sphere"), "{why}");
    doc.delete(bad).unwrap();

    // A cone 6 in radius at the bottom and 2 at the top, 8 high, whose slanted face is side.1: the plane touches it along
    // the line to its tip.
    let mut doc = Document::new();
    let cone = doc.with(Op::solid(Primitive::Cone { r0: 6.0.into(), r1: 2.0.into(), height: 8.0.into() }, Placement::xy(0.0), Mode::Add));
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.1", "point": [0, 30, 4]}}"#, cone.0));
    ok(&mut doc);
    let f = doc.frame_of(p).unwrap();
    let tip = v3(0.0, 0.0, 12.0);
    assert!(f.origin.x.abs() < 1e-9 && f.origin.y > 0.0, "{:?}", f.origin);
    assert!(f.z.dot(tip - f.origin).abs() < 1e-9, "the tip is in the plane");
    assert!(f.z.dot(v3(0.0, 6.0, 0.0) - f.origin).abs() < 1e-9, "so is the bottom rim's point below");
    assert!(f.z.y > 0.0 && f.z.x.abs() < 1e-12);
    assert!(f.z.dot(v3(0.0, 0.0, 4.0) - f.origin) < 0.0, "facing away from the axis");

    // A sphere: at a point, and not at an angle.
    let mut doc = Document::new();
    let ball = doc.with(Op::solid(Primitive::Sphere { radius: 5.0.into() }, Placement::at(point3(1.0, 2.0, 3.0)), Mode::Add));
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.0", "point": [1, 2, 30]}}"#, ball.0));
    is(&mut doc, p, [1.0, 2.0, 8.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let bad = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "tangent", "face": "f{}:side.0", "degrees": 30}}"#, ball.0));
    assert!(failure(&mut doc).contains("sphere, which has no axis"));
    let _ = bad;
}

#[test]
fn what_a_plane_names_has_to_be_there_and_fit() {
    let mut doc = block();
    // A face that isn't flat has no plane.
    let c = doc.with(Op::sketch(Placement::xy(4.0), vec![Region::new(Shape::circle(5.0, 3.0, 2.0))]));
    let boss = doc.with(Op::extrude(c, 0.0, 2.0, Mode::Add));
    let p = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "offset", "plane": "f{}:side.0", "distance": 1}}"#, boss.0));
    let why = failure(&mut doc);
    assert!(why.contains("is a cylinder, not flat"), "{why}");
    // A face that's gone.
    doc.edit(p, read_op(&json::parse(&format!(r#"{{"type": "plane", "plane_type": "offset", "plane": "f{}:end", "distance": 1}}"#, boss.0)).unwrap(), &[]).unwrap()).unwrap();
    is(&mut doc, p, [0.0, 0.0, 7.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    doc.suppress(boss, true).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&format!("the face f{}:end no longer exists", boss.0)), "{why}");
    doc.suppress(boss, false).unwrap();
    // A number names a plane feature, and nothing else.
    let op = read_op(&json::parse(&format!(r#"{{"type": "plane", "plane_type": "offset", "plane": {}, "distance": 1}}"#, boss.0)).unwrap(), &[]).unwrap();
    let why = doc.add(op).unwrap_err().to_string();
    assert!(why.contains("is a extrude, not a plane") || why.contains("not a plane"), "{why}");
    // A sketch's points come from a constrained sketch.
    let op = read_op(&json::parse(&format!(r#"{{"type": "plane", "plane_type": "plane_point", "plane": "top", "point": {{"sketch": {}, "point": "1"}}}}"#, c.0)).unwrap(), &[]).unwrap();
    let why = doc.add(op).unwrap_err().to_string();
    assert!(why.contains("constrained sketch"), "{why}");
    // A sketch on a plane needs it there: it can't be deleted while it's used, and suppressed, it stops what's on it.
    let s = doc.with(Op::sketch(Placement::on_plane(PlaneRef::Feature(p)), vec![Region::new(Shape::circle(0.0, 0.0, 1.0))]));
    assert!(doc.delete(p).unwrap_err().to_string().contains(&format!("f{}", s.0)));
    doc.suppress(p, true).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&format!("plane f{} is suppressed", p.0)), "{why}");
    doc.suppress(p, false).unwrap();
    ok(&mut doc);
    // A plane acts on no body.
    assert!(doc.set_scope(p, vec![BodyId { feature: FeatureId(2), index: 0 }]).is_err());
    // And a sketch can't come before the plane it's on.
    assert!(doc.reorder(s, 0).is_err());
    // What it's made of has to make sense as it's read.
    for (text, says) in [
        (r#"{"type": "plane", "plane_type": "sideways", "plane": "top"}"#, "unknown plane_type"),
        (r#"{"type": "plane", "plane": "top"}"#, "plane_type"),
        (r#"{"type": "plane", "plane_type": "offset", "plane": "top"}"#, "distance"),
        (r#"{"type": "plane", "plane_type": "offset", "plane": "sideways", "distance": 1}"#, "isn't a plane"),
        (r#"{"type": "plane", "plane_type": "three_point", "points": ["origin", [1, 0, 0]]}"#, "list of 3"),
        (r#"{"type": "plane", "plane_type": "mid_plane"}"#, "two of them"),
        (r#"{"type": "plane", "plane_type": "curve_point", "edge": "all"}"#, "fraction"),
        (r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 1, "flip": 1}"#, "flip"),
        (r#"{"type": "plane", "plane_type": "offset", "plane": "top", "distance": 1, "extra": 1}"#, "unknown field"),
    ] {
        let why = read_op(&json::parse(text).unwrap(), &[]).unwrap_err().to_string();
        assert!(why.contains(says), "{text}: {why}");
    }
}

#[test]
fn every_type_saves_and_reads_back() {
    let (mut doc, flat, _) = with_lines();
    let edge = r#"{"between": ["f2:start", "f2:side.0"]}"#;
    let list = [
        r#"{"type": "plane", "plane_type": "offset", "plane": "f2:end", "distance": "1 + 2", "flip": true}"#.to_string(),
        r#"{"type": "plane", "plane_type": "plane_point", "plane": {"point": [0, 0, 1], "normal": [0, 1, 1]}, "point": {"faces": ["f2:end", "f2:side.1"], "near": [1, 2, 3]}}"#.to_string(),
        format!(r#"{{"type": "plane", "plane_type": "line_angle", "line": {{"sketch": {}, "line": 1}}, "degrees": 45, "reference": "right"}}"#, flat.0),
        format!(r#"{{"type": "plane", "plane_type": "point_normal", "point": {{"sketch": {}, "point": "1.end"}}, "line": {edge}}}"#, flat.0),
        r#"{"type": "plane", "plane_type": "three_point", "points": ["origin", [1, 0, 0], [0, 1, 0]]}"#.to_string(),
        r#"{"type": "plane", "plane_type": "mid_plane", "planes": ["top", "front"]}"#.to_string(),
        r#"{"type": "plane", "plane_type": "mid_plane", "points": ["origin", [0, 0, 4]]}"#.to_string(),
        format!(r#"{{"type": "plane", "plane_type": "curve_point", "edge": {edge}, "fraction": 0.5}}"#),
        r#"{"type": "plane", "plane_type": "curve_point", "edge": {"near": [1, 2, 3], "tolerance": 0.5}, "point": [1, 2, 3]}"#.to_string(),
        r#"{"type": "plane", "plane_type": "tangent", "face": "f2:side.0", "degrees": 30}"#.to_string(),
        r#"{"type": "plane", "plane_type": "tangent", "face": "f2:side.0", "point": [0, 0, 1]}"#.to_string(),
        r#"{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {"axis_of": {"radius": 3}}}"#.to_string(),
        r#"{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {"point": [0, 0, 1], "direction": [0, 0, 1]}}"#.to_string(),
        r#"{"type": "plane", "plane_type": "point_normal", "point": "origin", "line": {"edge": {"parallel": [0, 0, 1]}}}"#.to_string(),
    ];
    for text in &list {
        let op = read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"));
        assert_eq!(read_op(&op_value(&op), &[]).unwrap(), op, "{text}");
        doc.add(op).unwrap();
    }
    let back = Document::load_text(&doc.save_text()).unwrap();
    assert_eq!(back.features(), doc.features());
}

#[test]
fn the_outcome_has_each_plane_as_the_history_stood() {
    let mut doc = block();
    let p = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:end", "distance": 1}"#);
    let q = feature(&mut doc, &format!(r#"{{"type": "plane", "plane_type": "offset", "plane": {}, "distance": 1}}"#, p.0));
    let o = ok(&mut doc);
    assert_eq!(o.planes.iter().map(|x| x.id).collect::<Vec<_>>(), vec![p, q]);
    close(o.planes[1].frame.origin, [0.0, 0.0, 6.0], "q");
    // Rolled back before the second, only the first is there.
    let o = doc.rebuild(Some(2));
    assert_eq!(o.planes.iter().map(|x| x.id).collect::<Vec<_>>(), vec![p]);
}

#[test]
fn a_sketch_tool_can_solve_on_a_plane() {
    // A projected corner, seen from a plane that's tilted: the plane's frame is what it's seen in.
    let mut doc = block();
    let p = feature(&mut doc, r#"{"type": "plane", "plane_type": "offset", "plane": "f2:side.1", "distance": 2}"#);
    ok(&mut doc);
    let mut s = Sketch::new();
    let corner = s.project(json::parse(r#"{"vertex": {"faces": ["f2:end", "f2:side.1", "f2:side.2"]}}"#).unwrap());
    let pt = s.point(1.0, 1.0);
    s.constrain(Rule::Coincident { a: pt.whole(), b: corner.whole() });
    let placed = doc.sketch_in_context(&s, &Placement::on_plane(PlaneRef::Feature(p)), doc.features().len()).unwrap();
    // The plane's x is +Y and its y is +Z, so the corner (10, 6, 4) is at (6, 4) in it.
    let solved = placed.solve().unwrap();
    let coreshape_sketch::Geometry::Point { at } = solved.sketch.entity(pt).unwrap().geometry else { panic!() };
    assert!((at[0] - 6.0).abs() < 1e-9 && (at[1] - 4.0).abs() < 1e-9, "{at:?}");
}
