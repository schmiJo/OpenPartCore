//! A loft as a feature: names, saving, editing and the cache.

use coreshape_document::*;
use std::f64::consts::PI;

/// Two circles of parameter radii, 20 apart.
fn circles() -> (Document, FeatureId, FeatureId, FeatureId) {
    let mut doc = Document::new();
    doc.set_parameter("bottom", 10.0).unwrap();
    doc.set_parameter("top", 5.0).unwrap();
    let a = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, "bottom"))]));
    let b = doc.with(Op::sketch(Placement::xy(20.0), vec![Region::new(Shape::circle(0.0, 0.0, "top"))]));
    let l = doc.with(Op::Loft { sections: vec![a, b], mode: Mode::Add });
    (doc, a, b, l)
}

fn volume(doc: &mut Document) -> f64 {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    coreshape::mass_properties(&o.body.unwrap()).volume
}

fn frustum(a: f64, b: f64) -> f64 {
    PI * 20.0 * (a * a + a * b + b * b) / 3.0
}

#[test]
fn it_builds_and_a_dimension_in_any_section_rebuilds_it() {
    let (mut doc, ..) = circles();
    assert!((volume(&mut doc) - frustum(10.0, 5.0)).abs() < 1e-8);
    doc.set_parameter("top", 7.0).unwrap();
    assert!((volume(&mut doc) - frustum(10.0, 7.0)).abs() < 1e-8);
    doc.set_parameter("bottom", 12.0).unwrap();
    assert!((volume(&mut doc) - frustum(12.0, 7.0)).abs() < 1e-8);
}

#[test]
fn it_saves_and_loads() {
    let (mut doc, ..) = circles();
    let before = volume(&mut doc);
    let text = doc.to_json().to_pretty();
    assert!(text.contains("\"loft\""), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.to_json().to_pretty(), text);
    assert!((volume(&mut back) - before).abs() < 1e-9);
}

#[test]
fn its_faces_have_names_and_keys() {
    let (mut doc, _, _, l) = circles();
    let body = doc.rebuild(None).body.unwrap();
    let names = face_names(&body);
    for want in [format!("f{}:start", l.0), format!("f{}:end", l.0), format!("f{}:side.0", l.0)] {
        assert!(names.contains(&want), "{want} in {names:?}");
    }
    for (i, key) in face_keys(&body).iter().enumerate() {
        assert_eq!(find_faces(&body, key), vec![i], "{}", key.text());
    }
}

#[test]
fn it_counts_as_using_its_sections_and_can_be_cut_and_copied() {
    let (mut doc, a, b, l) = circles();
    assert!(doc.dependents(a).contains(&l) && doc.dependents(b).contains(&l));
    let one = volume(&mut doc);
    doc.with(Op::Mirror { of: vec![], point: point3(30.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    assert!((volume(&mut doc) - 2.0 * one).abs() < 1e-6);
    let hole = doc.with(Op::sketch(Placement::new(point3(-20.0, 0.0, 10.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)), vec![Region::new(Shape::circle(0.0, 0.0, 2.0))]));
    doc.with(Op::extrude(hole, 0.0, 40.0, Mode::Remove));
    let cut = volume(&mut doc);
    assert!(cut < 2.0 * one && cut > 1.8 * one, "{cut}");
}

#[test]
fn a_section_has_to_be_one_outline_of_a_sketch() {
    let (mut doc, _, _, l) = circles();
    assert!(doc.add(Op::Loft { sections: vec![l, l], mode: Mode::Add }).is_err());

    let mut doc = Document::new();
    let two = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(-9.0, 0.0, 3.0)), Region::new(Shape::circle(9.0, 0.0, 3.0))]));
    let one = doc.with(Op::sketch(Placement::xy(9.0), vec![Region::new(Shape::circle(0.0, 0.0, 3.0))]));
    doc.with(Op::Loft { sections: vec![two, one], mode: Mode::Add });
    let o = doc.rebuild(None);
    let failed = o.failed.expect("two outlines in one section");
    assert!(matches!(&o.steps[failed].state, StepState::Failed(why) if why.contains("one outline")), "{:?}", o.steps[failed]);
}
