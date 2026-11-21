//! A transform that scales: about a point, or a part's corner, which it follows; as a copy; and on a part brought in from
//! a STEP file drawn in inches, whose faces keep their names, so later features still find them.

use coreshape::export::step::write_step;
use coreshape::*;
use coreshape_document::*;

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

fn op(text: &str) -> Op {
    read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))
}

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(op(text)).unwrap()
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-9 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn corners(o: &Outcome) -> Aabb {
    let mut out = Aabb::EMPTY;
    for b in &o.bodies {
        for p in &b.body.vertices {
            out.add(*p);
        }
    }
    out
}

/// A block `w` by 6 by 4 from the origin, with a hole of radius 1 through it at (3, 3).
fn block() -> Document {
    let mut doc = Document::new();
    doc.set_parameter("w", 10.0).unwrap();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, "w", 6.0)).hole(Shape::circle(3.0, 3.0, 1.0))]));
    doc.with(Op::extrude(s, 0.0, 4.0, Mode::Add));
    doc
}

#[test]
fn a_part_scales_about_a_point_or_a_corner_it_follows() {
    let solid = |w: f64| w * 24.0 - std::f64::consts::PI * 4.0;
    // About the origin, by a parameter.
    let mut doc = block();
    doc.set_parameter("k", 2.0).unwrap();
    let t = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "scale", "factor": "k"}}"#);
    for k in [2.0, 0.5] {
        doc.set_parameter("k", k).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), solid(10.0) * k * k * k, "scaled about the origin");
        let b = corners(&o);
        assert!(b.min.dist(Vec3::ZERO) < 1e-12 && b.max.dist(v3(10.0 * k, 6.0 * k, 4.0 * k)) < 1e-9, "{b:?}");
    }
    assert_eq!(read_op(&op_value(&doc.feature(t).unwrap().op), &[]).unwrap(), doc.feature(t).unwrap().op);

    // About the block's far top corner, where the sides facing +X and +Y meet the top, which moves with w: it stays put.
    let mut doc = block();
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "scale", "point": {"faces": ["f2:side.1", "f2:side.2", "f2:end"]}, "factor": 3}}"#);
    for w in [10.0, 14.0] {
        doc.set_parameter("w", w).unwrap();
        let o = ok(&mut doc);
        near(vol(&o), solid(w) * 27.0, "scaled about a corner");
        let b = corners(&o);
        assert!(b.max.dist(v3(w, 6.0, 4.0)) < 1e-9, "the corner stays where it is: {b:?}");
        assert!(b.min.dist(v3(w - 3.0 * w, 6.0 - 18.0, 4.0 - 12.0)) < 1e-9, "{b:?}");
        // The hole is a cylinder three times the size, and the faces keep their names, so a later fillet finds them.
        let body = &o.bodies[0].body;
        assert!(body.faces.iter().any(|f| matches!(f.surface, Surface::Cylinder { radius, .. } if (radius - 3.0).abs() < 1e-12)));
    }
    feature(&mut doc, r#"{"type": "fillet", "edges": {"between": ["f2:end", "f2:side.1"]}, "radius": 0.5}"#);
    ok(&mut doc);

    // As a copy, leaving the original.
    let mut doc = block();
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "scale", "point": [0, 0, 0], "factor": 2}, "copy": true}"#);
    let o = ok(&mut doc);
    assert_eq!(o.bodies.len(), 2);
    near(vol(&o), solid(10.0) * 9.0, "the original and a copy twice the size");
}

#[test]
fn a_part_from_a_file_in_inches_comes_to_millimetres() {
    // A plate 4 by 2 by 0.25 inches, with a hole of radius 0.25, as a file written in inches would have it.
    let plate = extrude(&Profile::new(Wire::rect_centered(4.0, 2.0)).with_hole(Wire::circle(v2(1.0, 0.0), 0.25)), &Frame::WORLD, 0.25, 1).unwrap();
    let text = write_step(&plate, "plate");
    let mut doc = Document::new();
    doc.with(Op::ImportStep { text, mode: Mode::NewBody });
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "scale", "factor": 25.4}}"#);
    let o = ok(&mut doc);
    near(vol(&o), mass_properties(&plate).volume * 25.4f64.powi(3), "in millimetres");
    let b = corners(&o);
    assert!(b.max.dist(v3(50.8, 25.4, 6.35)) < 1e-9, "{b:?}");
}

#[test]
fn what_a_scale_refuses() {
    for (factor, says) in [("0", "above 0"), ("-2", "above 0"), ("1e7", "above 0")] {
        let mut doc = block();
        feature(&mut doc, &format!(r#"{{"type": "transform", "motion": {{"type": "scale", "factor": {factor}}}}}"#));
        let why = failure(&mut doc);
        assert!(why.contains(says), "{factor}: {why}");
    }
    let why = read_op(&json::parse(r#"{"type": "transform", "motion": {"type": "scale"}}"#).unwrap(), &[]).unwrap_err().to_string();
    assert!(why.contains("factor"), "{why}");
}
