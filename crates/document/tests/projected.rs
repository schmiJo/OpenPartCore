//! Links from a sketch to part geometry, as Onshape's are: a point on a part's corner or edge that stays there when the part
//! changes, and Use, which brings a part's edge into the sketch as a curve. Each placement is checked against where the
//! part's geometry is, worked out by hand.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::{EntityId, Geometry, Rule, Sketch};

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

fn near(a: [f64; 2], b: [f64; 2], what: &str) {
    assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-7, "{what}: {a:?} isn't {b:?}");
}

fn from(text: &str) -> json::Value {
    json::parse(text).unwrap()
}

/// A block from the origin, `w` by 6 by 4, with `w` a parameter.
fn block() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    doc.add(Op::solid(Primitive::Box { dx: "w".into(), dy: 6.0.into(), dz: 4.0.into() }, Placement::xy(0.0), Mode::Add)).unwrap();
    doc
}

/// Where the last rebuild put a sketch's entity.
fn placed(doc: &Document, sketch: FeatureId, id: EntityId) -> Geometry {
    let Op::ConstrainedSketch { plane, sketch: s } = doc.feature(sketch).unwrap().op.clone() else { panic!() };
    let mut doc = doc.clone();
    let at = doc.features().iter().position(|f| f.id == sketch).unwrap();
    doc.sketch_in_context(&s, &plane, at).unwrap().entity(id).unwrap().geometry.clone()
}

/// The extent of a body's mesh: its box isn't tight round curves.
fn bounds(o: &Outcome, body: usize) -> Aabb {
    let b = &o.bodies[body].body;
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    let mut out = Aabb::EMPTY;
    for p in &mesh.positions {
        out.add(*p);
    }
    out
}

/// A sketch feature needs a closed region of its own, so ones that only look at projections get a small circle well away.
fn with_region(mut s: Sketch) -> Sketch {
    s.circle([100.0, 100.0], 1.0);
    s
}

#[test]
fn a_circle_on_a_corner_follows_the_corner() {
    let mut doc = block();
    let mut s = Sketch::new();
    let corner = s.project(from(r#"{"vertex": {"faces": ["f1:end", "f1:side.1", "f1:side.2"]}}"#));
    let c = s.circle([8.0, 5.0], 2.0);
    s.constrain(Rule::Coincident { a: c.center(), b: corner.whole() });
    s.constrain(Rule::Radius { of: c, value: 2.0.into() });
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(4.0), sketch: s }).unwrap();
    doc.add(Op::extrude(sk, 0.0, 3.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    let b = bounds(&o, 1);
    near([(b.min.x + b.max.x) / 2.0, (b.min.y + b.max.y) / 2.0], [10.0, 6.0], "on the corner");
    assert!((b.min.z - 4.0).abs() < 1e-9);
    doc.set_parameter("w", 17.0).unwrap();
    let o = ok(&mut doc);
    let b = bounds(&o, 1);
    near([(b.min.x + b.max.x) / 2.0, (b.min.y + b.max.y) / 2.0], [17.0, 6.0], "still on the corner");
}

#[test]
fn use_brings_an_edge_in_to_bound_a_region() {
    // A rectangle off the block's front, closed by the block's top front edge, as Onshape's Use does.
    let mut doc = block();
    let mut s = Sketch::new();
    let edge = s.project(from(r#"{"edge": {"between": ["f1:end", "f1:side.0"]}}"#));
    s.set_construction(edge, false).unwrap();
    let plane = Placement::xy(4.0);
    let there = doc.sketch_in_context(&s, &plane, 1).unwrap();
    let Geometry::Line { start, end } = there.entity(edge).unwrap().geometry else { panic!("{:?}", there.entity(edge)) };
    // Whichever way the edge runs, it's the front edge of the top.
    assert!((start[1]).abs() < 1e-9 && (end[1]).abs() < 1e-9 && (start[0] - end[0]).abs() > 9.999, "{start:?} {end:?}");
    let (a, b, c) = (s.line(end, [end[0], -5.0]), s.line([end[0], -5.0], [start[0], -5.0]), s.line([start[0], -5.0], start));
    s.constrain(Rule::Coincident { a: edge.end(), b: a.start() });
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Coincident { a: b.end(), b: c.start() });
    s.constrain(Rule::Coincident { a: c.end(), b: edge.start() });
    s.constrain(Rule::Vertical { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: c.whole(), b: None });
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: 5.0.into() });
    let sk = doc.add(Op::ConstrainedSketch { plane, sketch: s }).unwrap();
    let e = doc.add(Op::extrude(sk, 0.0, 2.0, Mode::NewBody)).unwrap();
    let o = ok(&mut doc);
    assert!((mass_properties(&o.bodies[1].body).volume - 10.0 * 5.0 * 2.0).abs() < 1e-9);
    // The side swept from the part's edge is named for the projected entity.
    let names: Vec<String> = face_keys(&o.bodies[1].body).iter().map(|k| k.face.name()).collect();
    assert!(names.contains(&e.side(edge.0).name()), "{names:?}");
    // It widens with the block.
    doc.set_parameter("w", 14.0).unwrap();
    let o = ok(&mut doc);
    assert!((mass_properties(&o.bodies[1].body).volume - 14.0 * 5.0 * 2.0).abs() < 1e-9);
}

#[test]
fn an_edge_seen_end_on_is_a_point_and_a_circle_edge_on_is_a_line() {
    let mut doc = block();
    // A cylinder lying along x, behind the block.
    doc.add(Op::solid(Primitive::Cylinder { radius: 2.0.into(), height: 8.0.into() }, Placement::new(point3(20.0, 3.0, 2.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), Mode::NewBody)).unwrap();
    let mut s = Sketch::new();
    let upright = s.project(from(r#"{"edge": {"between": ["f1:side.0", "f1:side.1"]}}"#));
    let rims = s.project(from(r#"{"edge": {"near": [20, 3, 4], "tolerance": 0.001}}"#));
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(10.0), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    match placed(&doc, sk, upright) {
        Geometry::Point { at } => near(at, [10.0, 0.0], "the block's upright edge, end on"),
        other => panic!("{other:?}"),
    }
    match placed(&doc, sk, rims) {
        Geometry::Line { start, end } => {
            let (lo, hi) = (start[1].min(end[1]), start[1].max(end[1]));
            assert!((start[0] - 20.0).abs() < 1e-9 && (end[0] - 20.0).abs() < 1e-9, "{start:?} {end:?}");
            assert!((hi - lo - 4.0).abs() < 1e-9 || (hi - lo - 2.0).abs() < 1e-9, "half a rim seen edge on spans its width: {lo} to {hi}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_rim_seen_square_on_is_an_arc_and_at_a_slant_an_elliptical_arc() {
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Cylinder { radius: 3.0.into(), height: 5.0.into() }, Placement::xy(0.0), Mode::Add)).unwrap();
    let mut s = Sketch::new();
    let rim = s.project(from(r#"{"edge": {"near": [0, 3, 5], "tolerance": 0.001}}"#));
    let line = s.line([4.0, -2.0], [4.0, 2.0]);
    s.set_construction(line, true).unwrap();
    s.constrain(Rule::Tangent { a: line, b: rim });
    s.constrain(Rule::Vertical { a: line.whole(), b: None });
    let square = doc.add(Op::ConstrainedSketch { plane: Placement::xy(5.0), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    match placed(&doc, square, rim) {
        Geometry::Arc { center, radius, start, end } => {
            near(center, [0.0, 0.0], "center");
            assert!((radius - 3.0).abs() < 1e-9);
            // Half the rim, counter-clockwise from start to end, through (0, 3).
            let mid = |p: [f64; 2]| p[1].atan2(p[0]);
            let (a0, a1) = (mid(start), mid(end));
            let sweep = (a1 - a0).rem_euclid(std::f64::consts::TAU);
            assert!((sweep - std::f64::consts::PI).abs() < 1e-9, "{start:?} {end:?}");
            let through = a0 + sweep / 2.0;
            assert!((through.sin() - 1.0).abs() < 1e-9, "{start:?} {end:?}");
        }
        other => panic!("{other:?}"),
    }
    // The line is tangent on the rim's right.
    let Op::ConstrainedSketch { plane, sketch } = doc.feature(square).unwrap().op.clone() else { panic!() };
    let solved = doc.clone().sketch_in_context(&sketch, &plane, 1).unwrap().solve().unwrap().sketch;
    let Geometry::Line { start, .. } = solved.entity(line).unwrap().geometry else { panic!() };
    assert!((start[0] - 3.0).abs() < 1e-7, "{start:?}");
    assert!(doc.sketch_arrangement(square).is_ok());

    // Tipped 60 degrees about x: the rim's y shrinks by cos 60.
    let mut s = Sketch::new();
    let rim = s.project(from(r#"{"edge": {"near": [0, 3, 5], "tolerance": 0.001}}"#));
    let tipped = doc.add(Op::ConstrainedSketch { plane: Placement::new(point3(0.0, 0.0, 5.0), point3(0.0, -(60f64.to_radians().sin()), 60f64.to_radians().cos()), point3(1.0, 0.0, 0.0)), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    match placed(&doc, tipped, rim) {
        Geometry::EllipticalArc { center, major, minor, .. } => {
            near(center, [0.0, 0.0], "center");
            assert!(((major[0].hypot(major[1])) - 3.0).abs() < 1e-9, "{major:?}");
            assert!((major[1]).abs() < 1e-9, "the major axis along x: {major:?}");
            assert!((minor - 1.5).abs() < 1e-9, "{minor}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_spline_edge_comes_in_as_a_spline() {
    let mut doc = Document::new();
    let path = Shape::Path {
        start: point(0.0, 0.0),
        moves: vec![Move::Line { to: point(10.0, 0.0) }, Move::Spline { through: vec![point(12.0, 5.0), point(8.0, 10.0)] }, Move::Line { to: point(0.0, 10.0) }],
    };
    let plate = doc.add(Op::sketch(Placement::xy(0.0), vec![Region::new(path)])).unwrap();
    doc.add(Op::extrude(plate, 0.0, 3.0, Mode::Add)).unwrap();
    let mut s = Sketch::new();
    let curve = s.project(from(r#"{"edge": {"between": ["f2:end", "f2:side.1"]}}"#));
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(3.0), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    match placed(&doc, sk, curve) {
        Geometry::Spline(shape) => {
            let (first, last) = (shape.points[0], shape.points[shape.points.len() - 1]);
            let ends = [first, last];
            assert!(ends.iter().any(|p| (p[0] - 10.0).abs() < 1e-9 && p[1].abs() < 1e-9), "{ends:?}");
            assert!(ends.iter().any(|p| (p[0] - 8.0).abs() < 1e-9 && (p[1] - 10.0).abs() < 1e-9), "{ends:?}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn what_has_gone_fails_the_sketch_saying_which_entity() {
    let mut doc = block();
    let mut s = Sketch::new();
    s.project(from(r#"{"edge": {"between": ["f1:end", "f1:side.0"]}}"#));
    let gone = s.project(from(r#"{"vertex": {"faces": ["f1:end", "f1:start"]}}"#));
    doc.add(Op::ConstrainedSketch { plane: Placement::xy(4.0), sketch: s }).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&format!("entity {} is projected from", gone.0)) && why.contains("no vertex is on all of f1:end, f1:start"), "{why}");

    let mut doc = block();
    let mut s = Sketch::new();
    let many = s.project(from(r#"{"edge": {"on_face": "f1:end"}}"#));
    doc.add(Op::ConstrainedSketch { plane: Placement::xy(4.0), sketch: s }).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&format!("entity {}", many.0)) && why.contains("picks 4 edges"), "{why}");

    // An edge on a face that a later edit takes away.
    let mut doc = block();
    let mut s = Sketch::new();
    let edge = s.project(from(r#"{"edge": {"between": ["f1:end", "f1:side.0"]}}"#));
    doc.add(Op::ConstrainedSketch { plane: Placement::xy(4.0), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    doc.edit(FeatureId(1), Op::solid(Primitive::Cylinder { radius: 3.0.into(), height: 4.0.into() }, Placement::xy(0.0), Mode::Add)).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains(&format!("entity {} is projected from", edge.0)), "{why}");
}

#[test]
fn it_saves_where_it_last_was_depends_on_what_it_projects_and_reads_back() {
    let mut doc = block();
    let mut s = Sketch::new();
    let corner = s.project(from(r#"{"vertex": {"faces": ["f1:end", "f1:side.1", "f1:side.2"]}}"#));
    let p = s.point(1.0, 1.0);
    s.constrain(Rule::Coincident { a: p.whole(), b: corner.whole() });
    let sk = doc.add(Op::ConstrainedSketch { plane: Placement::xy(4.0), sketch: with_region(s) }).unwrap();
    ok(&mut doc);
    assert!(doc.delete(FeatureId(1)).is_err(), "the sketch needs the block");
    assert_eq!(doc.dependents(FeatureId(1)), vec![sk]);
    let text = doc.save_text();
    let back = Document::load_text(&text).unwrap();
    let Op::ConstrainedSketch { sketch, .. } = &back.feature(sk).unwrap().op else { panic!() };
    match &sketch.entity(corner).unwrap().geometry {
        Geometry::Point { at } => near(*at, [10.0, 6.0], "saved where it last was"),
        other => panic!("{other:?}"),
    }
    let mut back = back;
    ok(&mut back);
    // A "from" the document can't read is refused when the sketch is read.
    let bad = r#"{"type": "constrained_sketch", "sketch": {"format": "coreshape-sketch", "version": 1, "entities": [{"id": 1, "type": "projected", "from": {"face": "f1:end"}}], "constraints": []}}"#;
    assert!(read_op(&json::parse(bad).unwrap(), &[]).is_err());
}

#[test]
fn the_sketch_tools_place_it_from_where_the_sketch_goes() {
    let mut doc = block();
    doc.add(Op::solid(Primitive::Box { dx: 2.0.into(), dy: 2.0.into(), dz: 2.0.into() }, Placement::at(point3(30.0, 0.0, 0.0)), Mode::NewBody)).unwrap();
    let mut s = Sketch::new();
    let corner = s.project(from(r#"{"vertex": {"faces": ["f2:end", "f2:side.1", "f2:side.2"]}}"#));
    // At position 1 the second box isn't there yet, and at 2 it is.
    assert!(doc.sketch_in_context(&s, &Placement::xy(0.0), 1).is_err());
    let there = doc.sketch_in_context(&s, &Placement::xy(0.0), 2).unwrap();
    match there.entity(corner).unwrap().geometry {
        Geometry::Point { at } => near(at, [32.0, 2.0], "the second box's corner"),
        ref other => panic!("{other:?}"),
    }
    assert!(there.solve().is_ok());
    // On a plane that follows a face, as a sketch on a face does.
    let on_top = Placement::xy(0.0).following(FaceKey::parse("f1:end").unwrap());
    let there = doc.sketch_in_context(&s, &on_top, 2).unwrap();
    match there.entity(corner).unwrap().geometry {
        Geometry::Point { at } => near(at, [32.0, 2.0], "seen from the top face's plane"),
        ref other => panic!("{other:?}"),
    }
}
