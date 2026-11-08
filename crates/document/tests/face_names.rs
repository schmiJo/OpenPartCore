//! What a face name promises, feature by feature. A name is stable: no feature renames a face that
//! already has one. It is not unique: a name means every face with that name, and these tests pin
//! down when a name covers more than one face. Anything attaching to a face by name needs this.

use coreshape_document::*;
use std::collections::BTreeMap;

/// How many faces carry each name.
fn counts(doc: &mut Document) -> BTreeMap<String, usize> {
    let outcome = doc.rebuild(None);
    assert_eq!(outcome.failed, None, "{:?}", outcome.steps);
    let body = outcome.body.expect("a body");
    let mut out = BTreeMap::new();
    for f in &body.faces {
        *out.entry(FaceRef::from_tag(f.tag).name()).or_insert(0) += 1;
    }
    out
}

fn list(items: &[(&str, usize)]) -> BTreeMap<String, usize> {
    items.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

/// A 40 by 30 by 10 block, named f2.
fn block() -> (Document, FeatureId) {
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 30.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    (doc, e)
}

const BLOCK: [(&str, usize); 6] =
    [("f2:start", 1), ("f2:end", 1), ("f2:side.0", 1), ("f2:side.1", 1), ("f2:side.2", 1), ("f2:side.3", 1)];

fn block_and(extra: &[(&str, usize)]) -> BTreeMap<String, usize> {
    let mut out = list(&BLOCK);
    out.extend(list(extra));
    out
}

#[test]
fn a_fillet_keeps_every_name_and_gives_all_its_faces_one_new_name() {
    let (mut doc, e) = block();
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), 2.0));
    assert_eq!(counts(&mut doc), block_and(&[("f3:side.1", 1)]));

    // Four rounded edges are four faces under a single name, so one can't be picked out by name.
    let (mut doc, e) = block();
    doc.with(Op::fillet(EdgeSelect::OnFace(e.end_cap()), 2.0));
    assert_eq!(counts(&mut doc), block_and(&[("f3:side.1", 4)]));

    let (mut doc, e) = block();
    doc.with(Op::chamfer(EdgeSelect::OnFace(e.end_cap()), 2.0));
    assert_eq!(counts(&mut doc), block_and(&[("f3:side.1", 4)]));
}

#[test]
fn a_shell_keeps_every_name_and_names_each_inner_wall_after_the_face_it_follows() {
    let (mut doc, e) = block();
    doc.with(Op::Shell { open: vec![e.end_cap()], thickness: 2.0.into() });
    assert_eq!(
        counts(&mut doc),
        block_and(&[("f3:start", 1), ("f3:side.0", 1), ("f3:side.1", 1), ("f3:side.2", 1), ("f3:side.3", 1)])
    );
}

#[test]
fn offset_and_draft_keep_every_name() {
    let (mut doc, _) = block();
    doc.with(Op::Offset { distance: 1.0.into() });
    assert_eq!(counts(&mut doc), list(&BLOCK));

    let (mut doc, e) = block();
    doc.with(Op::Draft { faces: vec![e.side(0)], pull: point3(0.0, 0.0, 1.0), neutral: point3(0.0, 0.0, 0.0), degrees: 3.0.into() });
    assert_eq!(counts(&mut doc), list(&BLOCK));
}

#[test]
fn a_mirror_or_pattern_copy_has_the_same_name_as_its_original() {
    // Nothing tells the copy from the original by name, and they aren't in the same plane.
    let (mut doc, _) = block();
    doc.with(Op::Mirror { of: vec![], point: point3(50.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    assert_eq!(
        counts(&mut doc),
        list(&[("f2:start", 2), ("f2:end", 2), ("f2:side.0", 2), ("f2:side.1", 2), ("f2:side.2", 2), ("f2:side.3", 2)])
    );

    let (mut doc, _) = block();
    doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 50.0.into() } });
    assert_eq!(
        counts(&mut doc),
        list(&[("f2:start", 3), ("f2:end", 3), ("f2:side.0", 3), ("f2:side.1", 3), ("f2:side.2", 3), ("f2:side.3", 3)])
    );
}

#[test]
fn a_name_can_disappear_when_its_face_does() {
    // The mirror plane lies on f2:side.1, so that wall ends up inside the merged solid.
    let (mut doc, _) = block();
    doc.with(Op::Mirror { of: vec![], point: point3(40.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    assert_eq!(
        counts(&mut doc),
        list(&[("f2:start", 1), ("f2:end", 1), ("f2:side.0", 1), ("f2:side.2", 1), ("f2:side.3", 2)])
    );
}

#[test]
fn a_cut_splits_a_face_into_pieces_under_one_name() {
    let (mut doc, _) = block();
    let slot = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(18.0, -1.0, 4.0, 32.0))]));
    doc.with(Op::extrude(slot, 0.0, 10.0, Mode::Remove));
    assert_eq!(
        counts(&mut doc),
        list(&[
            ("f2:start", 2),
            ("f2:end", 2),
            ("f2:side.0", 2),
            ("f2:side.1", 1),
            ("f2:side.2", 2),
            ("f2:side.3", 1),
            ("f4:side.1", 1),
            ("f4:side.3", 1),
        ])
    );
}

#[test]
fn a_whole_circle_is_two_faces_under_one_name() {
    let (mut doc, _) = block();
    let hole = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(6.0, 6.0, 2.0))]));
    let cut = doc.with(Op::extrude(hole, 0.0, 10.0, Mode::Remove));
    assert_eq!(counts(&mut doc), block_and(&[("f4:side.0", 2)]));

    doc.with(Op::Pattern { of: vec![cut], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 10.0.into() } });
    assert_eq!(counts(&mut doc), block_and(&[("f4:side.0", 6)]));
}

#[test]
fn names_do_not_change_with_a_dimension() {
    let build = |radius: f64, thickness: f64| {
        let (mut doc, e) = block();
        doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), radius));
        doc.with(Op::Shell { open: vec![e.start_cap()], thickness: thickness.into() });
        doc.with(Op::Mirror { of: vec![], point: point3(60.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
        counts(&mut doc)
    };
    assert_eq!(build(2.0, 1.0), build(3.5, 1.5));
}

// ----- Keys: telling apart faces with one name -----

/// Each face's key, after checking that it picks out that face and no other.
fn keys(doc: &mut Document) -> Vec<String> {
    let outcome = doc.rebuild(None);
    assert_eq!(outcome.failed, None, "{:?}", outcome.steps);
    let body = outcome.body.expect("a body");
    let list = face_keys(&body);
    assert_eq!(list.len(), body.faces.len());
    for (i, key) in list.iter().enumerate() {
        assert_eq!(find_faces(&body, key), vec![i], "{}", key.text());
        // The text is the whole key: it reads back as the same thing.
        let back = FaceKey::parse(&key.text()).unwrap();
        assert_eq!(&back, key, "{}", key.text());
    }
    let mut out: Vec<String> = list.iter().map(|k| k.text()).collect();
    out.sort();
    out
}

fn volume(doc: &mut Document) -> f64 {
    let outcome = doc.rebuild(None);
    assert_eq!(outcome.failed, None, "{:?}", outcome.steps);
    coreshape::mass_properties(&outcome.body.expect("a body")).volume
}

fn mirrored() -> (Document, FeatureId, FeatureId) {
    let (mut doc, e) = block();
    let m = doc.with(Op::Mirror { of: vec![], point: point3(50.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    (doc, e, m)
}

#[test]
fn a_face_that_has_its_name_to_itself_has_it_as_its_key() {
    let (mut doc, _) = block();
    assert_eq!(keys(&mut doc), ["f2:end", "f2:side.0", "f2:side.1", "f2:side.2", "f2:side.3", "f2:start"]);
}

#[test]
fn copies_are_numbered_and_the_original_is_copy_zero() {
    let (mut doc, _, _) = mirrored();
    let list = keys(&mut doc);
    assert_eq!(list.len(), 12);
    assert!(list.contains(&"f2:end@f3.0".to_string()) && list.contains(&"f2:end@f3.1".to_string()), "{list:?}");

    let (mut doc, _) = block();
    doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 50.0.into() } });
    let list = keys(&mut doc);
    for i in 0..3 {
        assert!(list.contains(&format!("f2:start@f3.{i}")), "{list:?}");
    }
}

#[test]
fn a_copy_of_a_copy_says_both() {
    let (mut doc, _, _) = mirrored();
    doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(0.0, 1.0, 0.0), count: 2.0.into(), spacing: 80.0.into() } });
    let list = keys(&mut doc);
    assert_eq!(list.len(), 24);
    for key in ["f2:end@f3.0@f4.0", "f2:end@f3.1@f4.0", "f2:end@f3.0@f4.1", "f2:end@f3.1@f4.1"] {
        assert!(list.contains(&key.to_string()), "{key} in {list:?}");
    }
}

#[test]
fn each_face_of_a_fillet_or_chamfer_says_which_edge_it_replaced() {
    for chamfer in [false, true] {
        let (mut doc, e) = block();
        doc.with(if chamfer { Op::chamfer(EdgeSelect::OnFace(e.end_cap()), 2.0) } else { Op::fillet(EdgeSelect::OnFace(e.end_cap()), 2.0) });
        let list = keys(&mut doc);
        for side in 0..4 {
            let key = format!("f3:side.1[f2:end|f2:side.{side}]");
            assert!(list.contains(&key), "{key} in {list:?}");
        }
    }
}

#[test]
fn a_fillet_on_copies_says_which_copy_too() {
    let (mut doc, e, _) = mirrored();
    doc.with(Op::fillet(EdgeSelect::Between(e.end_cap(), e.side(0)), 2.0));
    let list = keys(&mut doc);
    for copy in 0..2 {
        let key = format!("f4:side.1[f2:end|f2:side.0]@f3.{copy}");
        assert!(list.contains(&key), "{key} in {list:?}");
    }
}

#[test]
fn a_shells_inner_walls_keep_the_copy_of_the_face_they_follow() {
    let (mut doc, e, _) = mirrored();
    doc.with(Op::Shell { open: vec![e.end_cap()], thickness: 2.0.into() });
    let list = keys(&mut doc);
    for copy in 0..2 {
        let key = format!("f4:side.0[f2:side.0]@f3.{copy}");
        assert!(list.contains(&key), "{key} in {list:?}");
    }
}

#[test]
fn pieces_are_numbered_when_nothing_else_tells_them_apart() {
    // The two halves of a whole circle.
    let (mut doc, _) = block();
    let hole = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(6.0, 6.0, 2.0))]));
    let cut = doc.with(Op::extrude(hole, 0.0, 10.0, Mode::Remove));
    let list = keys(&mut doc);
    assert!(list.contains(&"f4:side.0#0".to_string()) && list.contains(&"f4:side.0#1".to_string()), "{list:?}");

    // A pattern of that cut: two halves in each of three copies.
    doc.with(Op::Pattern { of: vec![cut], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: 10.0.into() } });
    let list = keys(&mut doc);
    for copy in 0..3 {
        for piece in 0..2 {
            let key = format!("f4:side.0@f5.{copy}#{piece}");
            assert!(list.contains(&key), "{key} in {list:?}");
        }
    }

    // A slot cut across: each wall in two pieces.
    let (mut doc, _) = block();
    let slot = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(18.0, -1.0, 4.0, 32.0))]));
    doc.with(Op::extrude(slot, 0.0, 10.0, Mode::Remove));
    let list = keys(&mut doc);
    assert!(list.contains(&"f2:end#0".to_string()) && list.contains(&"f2:end#1".to_string()), "{list:?}");
}

#[test]
fn piece_numbers_follow_position() {
    let (mut doc, _) = block();
    let slot = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(18.0, -1.0, 4.0, 32.0))]));
    doc.with(Op::extrude(slot, 0.0, 10.0, Mode::Remove));
    let body = doc.rebuild(None).body.unwrap();
    let at = |text: &str| {
        let found = find_faces(&body, &FaceKey::parse(text).unwrap());
        assert_eq!(found.len(), 1);
        body.face_aabb(found[0]).center()
    };
    // Whichever way the surface's own axes run, the two pieces are the ones on either side of the slot.
    assert!((at("f2:end#0").x < 18.0) != (at("f2:end#1").x < 18.0));
}

#[test]
fn a_plain_name_still_means_every_face_with_that_name() {
    let (mut doc, _, _) = mirrored();
    let body = doc.rebuild(None).body.unwrap();
    assert_eq!(find_faces(&body, &FaceKey::parse("f2:end").unwrap()).len(), 2);
    assert_eq!(find_faces(&body, &FaceKey::parse("f2:end@f3.1").unwrap()).len(), 1);
    // A part of a key narrows on its own. The only piece of each copy is piece 0, and there's no piece 1.
    assert_eq!(find_faces(&body, &FaceKey::parse("f2:end#0").unwrap()).len(), 2);
    assert_eq!(find_faces(&body, &FaceKey::parse("f2:end#1").unwrap()).len(), 0);
}

#[test]
fn a_key_can_pick_one_copy_for_a_fillet() {
    let (mut one, e) = block();
    one.with(Op::fillet(EdgeSelect::OnFace(e.end_cap()), 2.0));
    let rounded = volume(&mut one);

    let (mut doc, _, _) = mirrored();
    doc.with(Op::fillet(EdgeSelect::OnFaceKey(FaceKey::parse("f2:end@f3.1").unwrap()), 2.0));
    let got = volume(&mut doc);
    assert!((got - (12000.0 + rounded)).abs() < 1e-6, "{got} against {}", 12000.0 + rounded);

    // Between two keys: one edge of one copy.
    let (mut doc, _, _) = mirrored();
    let a = FaceKey::parse("f2:end@f3.0").unwrap();
    let b = FaceKey::parse("f2:side.0@f3.0").unwrap();
    doc.with(Op::fillet(EdgeSelect::BetweenKeys(a, b), 2.0));
    let list = keys(&mut doc);
    assert!(list.iter().any(|k| k.starts_with("f4:side.1[f2:end|f2:side.0]")), "{list:?}");
    assert_eq!(list.iter().filter(|k| k.starts_with("f4:")).count(), 1, "{list:?}");
}

#[test]
fn a_key_saves_and_loads_in_the_file() {
    let (mut doc, _, _) = mirrored();
    doc.with(Op::fillet(EdgeSelect::OnFaceKey(FaceKey::parse("f2:end@f3.1").unwrap()), 2.0));
    let before = volume(&mut doc);
    let text = doc.to_json().to_pretty();
    assert!(text.contains("f2:end@f3.1"), "{text}");
    let mut back = Document::load_text(&text).unwrap();
    assert_eq!(back.to_json().to_pretty(), text);
    assert_eq!(volume(&mut back).to_bits(), before.to_bits());
}

#[test]
fn a_key_that_no_longer_picks_anything_says_so() {
    let (mut doc, _, _) = mirrored();
    doc.with(Op::fillet(EdgeSelect::OnFaceKey(FaceKey::parse("f2:end@f3.7").unwrap()), 2.0));
    let outcome = doc.rebuild(None);
    let failed = outcome.failed.expect("copy 7 doesn't exist");
    assert!(matches!(&outcome.steps[failed].state, StepState::Failed(why) if why.contains("no longer exist")), "{:?}", outcome.steps[failed]);
}

#[test]
fn a_feature_with_a_key_counts_as_using_every_feature_in_it() {
    let (mut doc, _, m) = mirrored();
    let f = doc.with(Op::fillet(EdgeSelect::OnFaceKey(FaceKey::parse("f2:end@f3.1").unwrap()), 2.0));
    assert!(doc.dependents(m).contains(&f));
}

#[test]
fn copy_zero_still_means_the_original_when_there_are_no_copies() {
    let (mut doc, _) = block();
    let p = doc.with(Op::Pattern { of: vec![], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 1.0.into(), spacing: 50.0.into() } });
    let body = doc.rebuild(None).body.unwrap();
    let key = FaceKey::parse(&format!("f2:end@f{}.0", p.0)).unwrap();
    assert_eq!(find_faces(&body, &key).len(), 1);
    assert_eq!(FaceKey::plain(FeatureId(2).end_cap()).text(), "f2:end");
}

#[test]
fn a_merged_face_keeps_the_originals_key() {
    // The mirror plane lies on the block's wall, so its top and its copy's top become one face.
    let (mut doc, _) = block();
    doc.with(Op::Mirror { of: vec![], point: point3(40.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
    let list = keys(&mut doc);
    assert!(list.contains(&"f2:end".to_string()), "{list:?}");
}

#[test]
fn bad_keys_are_refused_with_what_a_key_looks_like() {
    for text in ["f2:end@", "f2:end@f3", "f2:end@3.1", "f2:end#", "f2:end#x", "f3:side.1[f2:end", "f3:side.1[]", "f2:nope"] {
        assert!(FaceKey::parse(text).is_err(), "{text}");
    }
}

#[test]
fn shell_and_draft_refuse_a_key_instead_of_ignoring_part_of_it() {
    let text = r#"{"format":"coreshape-document","version":1,"units":"mm","next_id":4,"parameters":[],"features":[
        {"id":1,"name":"S","type":"sketch","regions":[{"outer":{"type":"rect","w":40,"h":30}}]},
        {"id":2,"name":"E","type":"extrude","sketch":1,"length":10},
        {"id":3,"name":"Sh","type":"shell","open":["f2:end@f9.1"],"thickness":2}]}"#;
    let error = Document::load_text(text).err().expect("refused").to_string();
    assert!(error.contains("plain face names"), "{error}");
}

#[test]
fn keys_do_not_change_with_a_dimension() {
    let build = |radius: f64, thickness: f64, height: f64| {
        let mut doc = Document::new();
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 40.0, 30.0))]));
        let e = doc.with(Op::extrude(s, 0.0, height, Mode::Add));
        doc.with(Op::fillet(EdgeSelect::OnFace(e.end_cap()), radius));
        doc.with(Op::Shell { open: vec![e.start_cap()], thickness: thickness.into() });
        doc.with(Op::Mirror { of: vec![], point: point3(60.0, 0.0, 0.0), normal: point3(1.0, 0.0, 0.0) });
        keys(&mut doc)
    };
    assert_eq!(build(2.0, 1.0, 10.0), build(3.5, 1.5, 17.0));
}
