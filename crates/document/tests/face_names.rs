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
