//! Helix, delete face and thicken as features: saved, reloaded, driven by parameters, and telling you when what they
//! refer to has gone.

use coreshape_document::*;
use std::f64::consts::{PI, TAU};

fn volume(doc: &mut Document) -> f64 {
    let out = doc.rebuild(None);
    assert_eq!(out.failed, None, "{:?}", out.steps);
    coreshape::mass_properties(out.body.as_ref().unwrap()).volume
}

fn upright(x: f64) -> Placement {
    Placement::new(point3(x, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0))
}

fn spring(doc: &mut Document) -> FeatureId {
    let wire = doc.with(Op::sketch(upright(0.0), vec![Region::new(Shape::circle("coil", 0.0, "wire"))]));
    doc.with(Op::Helix {
        sketch: wire,
        point: point3(0.0, 0.0, 0.0),
        axis: point3(0.0, 0.0, 1.0),
        pitch: "pitch".into(),
        turns: "turns".into(),
        left_handed: false,
        mode: Mode::Add,
    })
}

#[test]
fn a_spring_follows_its_parameters_and_survives_saving() {
    let mut doc = Document::new();
    for (name, value) in [("coil", 12.0), ("wire", 1.5), ("pitch", 5.0), ("turns", 3.0)] {
        doc.set_parameter(name, value).unwrap();
    }
    spring(&mut doc);
    let expect = |turns: f64| turns * TAU * PI * 1.5 * 1.5 * 12.0;
    let v = volume(&mut doc);
    assert!((v - expect(3.0)).abs() < 1e-6 * v, "{v}");
    doc.set_parameter("turns", 5.5).unwrap();
    let v = volume(&mut doc);
    assert!((v - expect(5.5)).abs() < 1e-6 * v, "{v}");
    let text = doc.save_text();
    assert!(text.contains("\"helix\""));
    let mut back = Document::load_text(&text).unwrap();
    assert!((volume(&mut back) - v).abs() < 1e-9);
    assert_eq!(back.save_text(), text);
    // A wire too thick for its pitch says so, in the feature.
    doc.set_parameter("wire", 3.0).unwrap();
    let out = doc.rebuild(None);
    assert!(out.failed.is_some());
    assert!(format!("{:?}", out.steps).contains("run into each other"), "{:?}", out.steps);
}

#[test]
fn deleting_a_hole_by_name_and_one_of_a_pattern_by_key() {
    let mut doc = Document::new();
    let plate = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 20.0))]));
    doc.with(Op::extrude(plate, 0.0, 5.0, Mode::Add));
    let circle = doc.with(Op::sketch(Placement::xy(-1.0), vec![Region::new(Shape::circle(10.0, 10.0, 3.0))]));
    let hole = doc.with(Op::extrude(circle, 0.0, 7.0, Mode::Remove));
    let pattern = doc.with(Op::Pattern { of: vec![hole], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 20.0.into() } });
    let all = volume(&mut doc);
    let one_hole = PI * 9.0 * 5.0;
    assert!((all - (6000.0 - 3.0 * one_hole)).abs() < 1e-9, "{all}");

    let mut keyed = doc.clone();
    // The middle hole only: the first copy made by the pattern.
    let key = FaceKey { face: hole.side(0), between: Vec::new(), copies: vec![coreshape::topo::CopyStep { feature: pattern.0, index: 1 }], piece: None };
    keyed.with(Op::DeleteFace { faces: vec![key] });
    let v = volume(&mut keyed);
    assert!((v - (6000.0 - 2.0 * one_hole)).abs() < 1e-9, "{v}");
    let text = keyed.save_text();
    assert!(text.contains("@f"), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert!((volume(&mut back) - v).abs() < 1e-9);

    // By its plain name, it means all of them.
    let mut named = doc.clone();
    named.with(Op::DeleteFace { faces: vec![FaceKey::plain(hole.side(0))] });
    assert!((volume(&mut named) - 6000.0).abs() < 1e-9);

    // Something not there is an error in the feature, not a wrong solid.
    let mut gone = doc.clone();
    gone.with(Op::DeleteFace { faces: vec![FaceKey::plain(FaceRef { feature: plate, slot: 987 })] });
    let out = gone.rebuild(None);
    assert!(out.failed.is_some());
    assert!(format!("{:?}", out.steps).contains("no longer exists"), "{:?}", out.steps);
}

#[test]
fn deleting_a_fillet_gives_the_part_back_and_a_later_feature_rebuilds() {
    let mut doc = Document::new();
    doc.set_parameter("r", 2.0).unwrap();
    let plate = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 20.0))]));
    let e = doc.with(Op::extrude(plate, 0.0, 10.0, Mode::Add));
    let round = doc.with(Op::Fillet { edges: EdgeSelect::OnFace(e.end_cap()), radius: "r".into(), propagate: true });
    let rounded = volume(&mut doc);
    assert!(rounded < 6000.0);
    // Every face of the fillet by what it's named, whichever edges those came from.
    let out = doc.rebuild(None);
    let body = out.body.unwrap();
    let keys: Vec<FaceKey> = face_keys(&body).into_iter().filter(|k| k.face.feature == round).collect();
    assert!(!keys.is_empty());
    doc.with(Op::DeleteFace { faces: keys });
    assert!((volume(&mut doc) - 6000.0).abs() < 1e-9);
    // And it still follows the parameter it no longer shows.
    doc.set_parameter("r", 3.0).unwrap();
    assert!((volume(&mut doc) - 6000.0).abs() < 1e-9);
}

#[test]
fn thickening_a_face_adds_to_the_part_and_can_cut_it() {
    let mut doc = Document::new();
    doc.set_parameter("t", 3.0).unwrap();
    let plate = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 20.0))]));
    let e = doc.with(Op::extrude(plate, 0.0, 10.0, Mode::Add));
    let base = volume(&mut doc);

    let mut up = doc.clone();
    let t = up.with(Op::Thicken { faces: vec![FaceKey::plain(e.end_cap())], thickness: "t".into(), mode: Mode::Add });
    assert!((volume(&mut up) - (base + 1800.0)).abs() < 1e-9);
    up.set_parameter("t", 5.0).unwrap();
    assert!((volume(&mut up) - (base + 3000.0)).abs() < 1e-9);
    let text = up.save_text();
    let mut back = Document::load_text(&text).unwrap();
    assert!((volume(&mut back) - (base + 3000.0)).abs() < 1e-9);
    // The far face has a name to build on.
    let out = up.rebuild(None);
    let body = out.body.as_ref().unwrap();
    assert!(body.faces.iter().any(|f| f.tag == t.end_cap().tag()), "{:?}", body.faces.iter().map(|f| f.tag).collect::<Vec<_>>());

    let mut down = doc.clone();
    down.with(Op::Thicken { faces: vec![FaceKey::plain(e.end_cap())], thickness: (-2.0).into(), mode: Mode::Remove });
    assert!((volume(&mut down) - (base - 1200.0)).abs() < 1e-9);
}
