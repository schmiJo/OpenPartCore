//! Lofts to a point, round in a ring, along guides and with their ends set, as features: they build, save, load and
//! rebuild when what they hang on moves.

use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    for b in &o.bodies {
        validate(&b.body).unwrap_or_else(|e| panic!("{e:?}"));
    }
    o
}

fn volume(doc: &mut Document) -> f64 {
    ok(doc).bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn failure(doc: &mut Document) -> String {
    let o = doc.rebuild(None);
    let i = o.failed.unwrap_or_else(|| panic!("it should have failed: {:?}", o.steps));
    match &o.steps[i].state {
        StepState::Failed(why) => why.clone(),
        other => panic!("{other:?}"),
    }
}

fn circle(r: &str) -> Region {
    Region::new(Shape::Circle { cx: 0.0.into(), cy: 0.0.into(), r: Expr::parse(r).unwrap() })
}

fn rect(w: f64, h: f64) -> Region {
    Region::new(Shape::Rect { x: (-w / 2.0).into(), y: (-h / 2.0).into(), w: w.into(), h: h.into() })
}

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "{what}: {a} against {b}");
}

/// Saved, loaded back the same, and building to the same volume.
fn round_trip(doc: &mut Document) {
    let before = volume(doc);
    let text = doc.to_json().to_pretty();
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.to_json().to_pretty(), text);
    close(volume(&mut back), before, 1e-12, "loaded back");
}

fn loft(sections: Vec<LoftSection>, shape: LoftShape) -> Op {
    Op::Loft { sections, mode: Mode::Add, thin: None, shape }
}

#[test]
fn a_loft_to_a_point_is_a_cone_and_follows_its_point() {
    let mut doc = Document::new();
    doc.set_parameter("r", 5.0).unwrap();
    doc.set_parameter("h", 12.0).unwrap();
    let base = doc.with(Op::sketch(Placement::xy(0.0), vec![circle("r")]));
    let tip = PointRef::At(point3(0.0, 0.0, "h"));
    doc.with(loft(vec![base.into()], LoftShape { to: Some(tip), ..Default::default() }));
    close(volume(&mut doc), PI * 25.0 * 12.0 / 3.0, 1e-9, "cone");
    doc.set_parameter("h", 20.0).unwrap();
    close(volume(&mut doc), PI * 25.0 * 20.0 / 3.0, 1e-9, "taller");
    round_trip(&mut doc);
    let text = doc.to_json().to_text();
    assert!(text.contains("\"to\""), "{text}");
}

#[test]
fn a_loft_can_start_at_a_corner_of_a_part() {
    let mut doc = Document::new();
    doc.with(Op::Solid { primitive: Primitive::Box { dx: 10.0.into(), dy: 10.0.into(), dz: 10.0.into() }, at: Placement::xy(0.0), mode: Mode::Add });
    let top = doc.with(Op::sketch(Placement::xy(30.0), vec![rect(10.0, 10.0)]));
    // From the box's top corner at (10, 10, 10) up to the square at 30: a pyramid, standing apart from the box.
    let body = ok(&mut doc).body.unwrap();
    let v = body.vertices.iter().position(|p| p.dist(v3(10.0, 10.0, 10.0)) < 1e-9).expect("the corner");
    let VertexRef::Faces { faces, near } = vertex_picks(&body, &face_keys(&body)).remove(v) else { panic!() };
    let from = PointRef::Vertex { faces, near };
    doc.with(Op::Loft { sections: vec![top.into()], mode: Mode::NewBody, thin: None, shape: LoftShape { from: Some(from), ..Default::default() } });
    let o = ok(&mut doc);
    close(o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum::<f64>(), 1000.0 + 100.0 * 20.0 / 3.0, 1e-9, "box and pyramid");
    round_trip(&mut doc);
}

#[test]
fn a_closed_loft_saves_and_rebuilds() {
    let mut doc = Document::new();
    let mut sections = Vec::new();
    for k in 0..8 {
        let a = 2.0 * PI * k as f64 / 8.0;
        let (c, s) = (a.cos(), a.sin());
        let place = Placement::new(point3(10.0 * c, 10.0 * s, 0.0), point3(-s, c, 0.0), point3(c, s, 0.0));
        sections.push(doc.with(Op::sketch(place, vec![circle("2")])).into());
    }
    doc.with(loft(sections, LoftShape { closed: true, ..Default::default() }));
    close(volume(&mut doc), 2.0 * PI * PI * 10.0 * 4.0, 3e-3, "ring");
    round_trip(&mut doc);
}

#[test]
fn ends_can_be_square_and_their_pull_a_parameter() {
    let mut doc = Document::new();
    doc.set_parameter("pull", 1.0).unwrap();
    let a = doc.with(Op::sketch(Placement::xy(0.0), vec![circle("5")]));
    let b = doc.with(Op::sketch(Placement::new(point3(8.0, 0.0, 20.0), point3(0.0, 0.0, 1.0), point3(1.0, 0.0, 0.0)), vec![circle("5")]));
    let pull = LoftEndSpec::Normal(Expr::parse("pull").unwrap());
    doc.with(loft(vec![a.into(), b.into()], LoftShape { start: pull.clone(), end: pull, ..Default::default() }));
    // The circle carried along one curve: its area times its height, whatever the pull.
    close(volume(&mut doc), PI * 25.0 * 20.0, 1e-9, "moved over");
    doc.set_parameter("pull", 2.0).unwrap();
    close(volume(&mut doc), PI * 25.0 * 20.0, 1e-9, "pulled harder");
    round_trip(&mut doc);
    doc.set_parameter("pull", 0.0).unwrap();
    assert!(failure(&mut doc).contains("pull"));
}

#[test]
fn guides_come_from_sketches_and_follow_them() {
    let mut doc = Document::new();
    doc.set_parameter("out", 16.0).unwrap();
    let a = doc.with(Op::sketch(Placement::xy(0.0), vec![rect(20.0, 10.0)]));
    let b = doc.with(Op::sketch(Placement::xy(40.0), vec![rect(20.0, 10.0)]));
    // On the plane y = 5, (u, v) at world (u, 5, v): out from the corner to x = out halfway up, and back.
    let plane = Placement::new(point3(0.0, 5.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0));
    let path = Region::new(Shape::OpenPath {
        start: point(10.0, 0.0),
        moves: vec![Move::Line { to: point("out", 20.0) }, Move::Line { to: point(10.0, 40.0) }],
    });
    let g = doc.with(Op::sketch(plane, vec![path]));
    doc.with(loft(vec![a.into(), b.into()], LoftShape { guides: vec![g], ..Default::default() }));
    // One guide moves every level alike, so the volume stays the box's.
    close(volume(&mut doc), 8000.0, 1e-9, "guided");
    let body = ok(&mut doc).body.unwrap();
    let mesh = tessellate(&body, &TessParams { chord: 1e-3, angle: 0.2 }).unwrap();
    let far = mesh.positions.iter().map(|p| p.x).fold(f64::MIN, f64::max);
    close(far, 16.0, 1e-6, "out to the guide");
    doc.set_parameter("out", 13.0).unwrap();
    let body = ok(&mut doc).body.unwrap();
    let mesh = tessellate(&body, &TessParams { chord: 1e-3, angle: 0.2 }).unwrap();
    close(mesh.positions.iter().map(|p| p.x).fold(f64::MIN, f64::max), 13.0, 1e-6, "the guide moved");
    round_trip(&mut doc);
    // A guide sketch with nothing open in it is refused, saying so.
    let c = doc.with(Op::sketch(Placement::xy(5.0), vec![circle("1")]));
    doc.add(loft(vec![a.into(), b.into()], LoftShape { guides: vec![c], ..Default::default() })).unwrap();
    assert!(failure(&mut doc).contains("open curve"));
}

#[test]
fn the_json_reads_and_writes_every_option() {
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]);
    let op = read(r#"{"type": "loft", "sections": [2, 3], "to": [0, 0, 30], "start": "normal", "guides": [4, 5]}"#).unwrap();
    let Op::Loft { shape, .. } = &op else { panic!() };
    assert_eq!(shape.guides, vec![FeatureId(4), FeatureId(5)]);
    assert!(matches!(shape.start, LoftEndSpec::Normal(_)));
    assert!(shape.to.is_some() && shape.from.is_none() && !shape.closed);
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);
    let op = read(r#"{"type": "loft", "sections": [2, 3, 4], "closed": true}"#).unwrap();
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);
    let op = read(r#"{"type": "loft", "sections": [2], "from": {"sketch": 6, "point": "3"}, "end": {"round": "1.5 * k"}}"#).unwrap();
    assert_eq!(read(&op_value(&op).to_text()).unwrap(), op);
    // A plain loft writes nothing new.
    let plain = read(r#"{"type": "loft", "sections": [2, 3]}"#).unwrap();
    let text = op_value(&plain).to_text();
    for key in ["from", "to", "closed", "guides", "start", "end"] {
        assert!(!text.contains(&format!("\"{key}\"")), "{text}");
    }
    assert!(read(r#"{"type": "loft", "sections": [2, 3], "start": "sideways"}"#).is_err());
    assert!(read(r#"{"type": "loft", "sections": [2, 3], "closed": 1}"#).is_err());
}
