//! What an app saves when a face or an edge is clicked has to keep meaning the same face or edge as dimensions change,
//! or else say it can't. Each of these is made of picks as an app makes them (`follow::as_clicked`), and followed by
//! continuity through a range of one dimension (`follow::follow`): a pick that resolves to anything other than where
//! its face or edge has got to, while its feature still builds, is a failure.

use coreshape_document::follow::{as_clicked, follow, seen, Tally};
use coreshape_document::json::{self, Value};
use coreshape_document::*;

/// A document from its parameters and its features, as JSON text.
fn part(params: &[(&str, f64)], features: &str) -> Document {
    let list = json::parse(features).unwrap();
    let n = list.as_array().unwrap().len();
    let params: Vec<Value> = params.iter().map(|(k, v)| Value::obj(vec![("name", Value::str(*k)), ("value", Value::Num(*v))])).collect();
    let v = Value::obj(vec![
        ("format", Value::str(FORMAT)),
        ("version", Value::Num(FORMAT_VERSION)),
        ("units", Value::str("mm")),
        ("next_id", Value::Num(n as f64 + 1.0)),
        ("parameters", Value::Arr(params)),
        ("features", list),
    ]);
    Document::from_json(&v).unwrap_or_else(|e| panic!("{e}"))
}

/// The document as clicked, which has to build as the document does.
fn clicked(doc: &Document) -> Document {
    let outcome = doc.clone_fresh().rebuild(None);
    assert_eq!(outcome.failed, None, "{:?}", outcome.steps);
    let out = as_clicked(doc).unwrap().0;
    let outcome = out.clone_fresh().rebuild(None);
    assert_eq!(outcome.failed, None, "as clicked: {:?}", outcome.steps);
    out
}

/// What feature `id` holds, as saved.
fn field(doc: &Document, id: u32, name: &str) -> String {
    let v = doc.to_json();
    let f = v.get("features").and_then(Value::as_array).unwrap().iter().find(|f| f.get("id").and_then(Value::as_f64) == Some(id as f64)).unwrap().clone();
    f.get(name).unwrap().to_text()
}

fn range(from: f64, to: f64, steps: usize) -> Vec<f64> {
    (1..=steps).map(|k| from + (to - from) * k as f64 / steps as f64).collect()
}

fn none_lost(t: &Tally, what: &str) {
    eprintln!("{what}: {}", t.summary());
    for line in &t.lines {
        eprintln!("  {line}");
    }
    assert!(t.moved == 0 && t.dropped == 0, "{what}: {}", t.summary());
    assert!(t.kept > 0, "{what}: nothing was followed");
}

trait Fresh {
    fn clone_fresh(&self) -> Document;
}

impl Fresh for Document {
    fn clone_fresh(&self) -> Document {
        Document::from_json(&self.to_json()).unwrap()
    }
}

const SLOTTED: &str = r#"[
  {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 40, "h": 30}}]},
  {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 10},
  {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "x": "s", "y": -1, "w": 4, "h": 32}}]},
  {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 10, "mode": "remove"},
  {"id": 5, "name": "", "type": "fillet", "edges": {"between": ["f2:end", "f2:side.0"]}, "radius": 1}
]"#;

#[test]
fn pieces_keep_their_picks_as_a_slot_moves_across() {
    let doc = clicked(&part(&[("s", 18.0)], SLOTTED));
    // The top is in two pieces, each picked by the side of the block it reaches.
    let edges = field(&doc, 5, "edges");
    assert!(edges.contains("f2:end{f2:side.1}") && edges.contains("f2:end{f2:side.3}"), "{edges}");
    none_lost(&follow(&doc, "s", &range(18.0, 4.0, 14), &range(18.0, 32.0, 14), "slot"), "a slot moving across");
}

#[test]
fn a_piece_coming_between_others_leaves_their_picks_alone() {
    // A second slot comes in from beyond the end, and makes a third piece of the top between the other two.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 60, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 10},
      {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "x": 10, "y": -1, "w": 4, "h": 32}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 10, "mode": "remove"},
      {"id": 5, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "x": "s", "y": -1, "w": 4, "h": 32}}]},
      {"id": 6, "name": "", "type": "extrude", "sketch": 5, "length": 10, "mode": "remove"},
      {"id": 7, "name": "", "type": "fillet", "edges": {"between": ["f2:end", "f2:side.0"]}, "radius": 1}
    ]"#;
    let doc = clicked(&part(&[("s", 58.0)], features));
    let at = seen(&doc, 58.0);
    assert_eq!(at.failed, None, "{}", at.why);
    none_lost(&follow(&doc, "s", &[], &range(58.0, 20.0, 19), "second slot"), "a piece coming between");
}

#[test]
fn a_whole_rim_is_one_edge() {
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 40, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 10},
      {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "cx": 20, "cy": 15, "r": "r"}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 10, "mode": "remove"},
      {"id": 5, "name": "", "type": "fillet", "edges": {"between": ["f2:end", "f4:side.0"]}, "radius": 1}
    ]"#;
    let doc = clicked(&part(&[("r", 6.0)], features));
    // Both halves, by the two faces alone: no piece numbers to swap.
    assert_eq!(field(&doc, 5, "edges"), r#"{"between":["f2:end","f4:side.0"]}"#);
    none_lost(&follow(&doc, "r", &range(6.0, 2.0, 8), &range(6.0, 12.0, 12), "hole"), "a whole rim");
}

#[test]
fn a_pick_that_has_gone_is_refused_and_not_left_out() {
    // Two holes, the second cut from below to a depth, so that it stops reaching the top.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 60, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 5},
      {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "cx": 15, "cy": 15, "r": 3}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "start": -1, "length": 7, "mode": "remove"},
      {"id": 5, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "cx": 45, "cy": 15, "r": 3}}]},
      {"id": 6, "name": "", "type": "extrude", "sketch": 5, "start": -1, "length": "d", "mode": "remove"},
      {"id": 7, "name": "", "type": "fillet", "edges": {"any": [{"between": ["f2:end", "f4:side.0"]}, {"between": ["f2:end", "f6:side.0"]}]}, "radius": 0.5}
    ]"#;
    let mut doc = clicked(&part(&[("d", 7.0)], features));
    assert_eq!(seen(&doc, 7.0).failed, None);
    doc.set_parameter("d", 4.0).unwrap();
    let outcome = doc.rebuild(None);
    let Some(i) = outcome.failed else { panic!("the fillet built without the rim it was given") };
    let StepState::Failed(why) = &outcome.steps[i].state else { unreachable!() };
    assert_eq!(outcome.steps[i].id, FeatureId(7));
    assert!(why.contains("f6:side.0") && why.contains("no longer exists") && why.contains("pick it again"), "{why}");
}

#[test]
fn a_key_for_one_piece_that_now_fits_two_is_refused() {
    // A slot splits the top. A second slot, cut in from the left, gets longer until it splits the left piece in two,
    // and both of those reach the left side.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 40, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 10},
      {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "x": 18, "y": -1, "w": 4, "h": 32}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 10, "mode": "remove"},
      {"id": 5, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "x": -1, "y": 13, "w": "L", "h": 4}}]},
      {"id": 6, "name": "", "type": "extrude", "sketch": 5, "start": 6, "length": 5, "mode": "remove"},
      {"id": 7, "name": "", "type": "fillet", "edges": {"between": ["f2:end", "f2:side.0"]}, "radius": 1}
    ]"#;
    let mut doc = clicked(&part(&[("L", 10.0)], features));
    let edges = field(&doc, 7, "edges");
    assert!(edges.contains("f2:end{f2:side.3}"), "{edges}");
    doc.set_parameter("L", 21.0).unwrap();
    let outcome = doc.rebuild(None);
    let Some(i) = outcome.failed else { panic!("a key for one piece picked two") };
    let StepState::Failed(why) = &outcome.steps[i].state else { unreachable!() };
    assert!(why.contains("f2:end{f2:side.3}") && why.contains("can't be told"), "{why}");
}

#[test]
fn pieces_nothing_tells_apart_keep_their_order() {
    // A tube with a slot across its top: the bore cuts each wall of the slot in two, and both halves of a wall touch
    // the same faces, so where they are tells them apart.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "r": "R"}, "holes": [{"type": "circle", "r": 6}]}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 20},
      {"id": 3, "name": "", "type": "sketch", "plane": {"origin": [0, 0, 15]}, "regions": [{"outer": {"type": "rect", "x": -1.5, "y": -20, "w": 3, "h": 40}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 6, "mode": "remove"},
      {"id": 5, "name": "", "type": "chamfer", "edges": {"between": ["f2:end", "f4:side.1"]}, "distance": 0.3}
    ]"#;
    let doc = clicked(&part(&[("R", 10.0)], features));
    let edges = field(&doc, 5, "edges");
    assert!(edges.contains("f4:side.1~0") && edges.contains("f4:side.1~1"), "{edges}");
    none_lost(&follow(&doc, "R", &range(10.0, 8.0, 6), &range(10.0, 16.0, 12), "tube"), "pieces told apart by where they are");
}

#[test]
fn places_where_two_faces_meet_twice_keep_their_order() {
    // A round rail half sunk in a plate: its round face meets the plate's top along two lines, which both end at the
    // rail's ends, so only where they are tells them apart.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 60, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 5},
      {"id": 3, "name": "", "type": "solid", "shape": "cylinder", "radius": "rr", "height": 40, "at": {"origin": [10, 15, 5], "z": [1, 0, 0], "x": [0, 1, 0]}, "mode": "add"},
      {"id": 4, "name": "", "type": "chamfer", "edges": {"both": [{"on_face": "f2:end"}, {"parallel": [1, 0, 0]}, {"in_box": [[0, 5, 0], [60, 25, 10]]}]}, "distance": 0.3}
    ]"#;
    let doc = clicked(&part(&[("rr", 4.0)], features));
    let edges = field(&doc, 4, "edges");
    assert!(edges.contains(r#""nth":0"#) && edges.contains(r#""nth":1"#), "{edges}");
    none_lost(&follow(&doc, "rr", &range(4.0, 2.0, 8), &range(4.0, 7.0, 12), "rail"), "two places where faces meet");
}

#[test]
fn pieces_between_round_cuts_are_told_apart_by_both() {
    // Round seats cut into a disc's rim, as on a sprocket: each piece of the rim touches two seats, and each seat two
    // pieces, so it takes both seats to say which piece.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "r": 20}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 5},
      {"id": 3, "name": "", "type": "sketch", "plane": {"origin": [0, 0, -1]}, "regions": [{"outer": {"type": "circle", "r": 3}, "repeat": {"type": "circular", "count": 6, "radius": "p"}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "length": 7, "mode": "remove"},
      {"id": 5, "name": "", "type": "chamfer", "edges": {"between": ["f2:end", "f2:side.0"]}, "distance": 0.3}
    ]"#;
    let doc = clicked(&part(&[("p", 21.0)], features));
    let edges = field(&doc, 5, "edges");
    assert!(edges.matches("f2:side.0{").count() == 6 && edges.contains(','), "{edges}");
    let t = follow(&doc, "p", &range(21.0, 19.0, 8), &range(21.0, 22.5, 6), "seats");
    none_lost(&t, "pieces between round cuts");
}

#[test]
fn places_where_two_faces_meet_twice_are_told_apart_by_their_ends() {
    // A round rail half sunk in a plate, notched at its far end on one side: of the two lines where it meets the top,
    // one ends at the notch and the other at the rail's end.
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 60, "h": 30}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 5},
      {"id": 3, "name": "", "type": "solid", "shape": "cylinder", "radius": "rr", "height": 40, "at": {"origin": [10, 15, 5], "z": [1, 0, 0], "x": [0, 1, 0]}, "mode": "add"},
      {"id": 4, "name": "", "type": "solid", "shape": "box", "dx": 20, "dy": 15, "dz": 20, "at": {"origin": [46, 15, 4]}, "mode": "remove"},
      {"id": 5, "name": "", "type": "chamfer", "edges": {"both": [{"on_face": "f2:end"}, {"parallel": [1, 0, 0]}, {"in_box": [[0, 5, 0], [60, 25, 10]]}]}, "distance": 0.3}
    ]"#;
    let doc = clicked(&part(&[("rr", 4.0)], features));
    let edges = field(&doc, 5, "edges");
    assert!(edges.matches(r#""ends""#).count() == 2, "{edges}");
    let t = follow(&doc, "rr", &range(4.0, 3.0, 5), &range(4.0, 5.0, 5), "notched rail");
    none_lost(&t, "two places told apart by their ends");
    assert_eq!(t.refused, 0, "each end picks its own line");
}

#[test]
fn a_pattern_count_keeps_each_copy_and_refuses_one_that_goes() {
    let features = r#"[
      {"id": 1, "name": "", "type": "sketch", "regions": [{"outer": {"type": "rect", "w": 100, "h": 20}}]},
      {"id": 2, "name": "", "type": "extrude", "sketch": 1, "length": 5},
      {"id": 3, "name": "", "type": "sketch", "regions": [{"outer": {"type": "circle", "cx": 10, "cy": 10, "r": 2}}]},
      {"id": 4, "name": "", "type": "extrude", "sketch": 3, "start": -1, "length": 7, "mode": "remove"},
      {"id": 5, "name": "", "type": "pattern", "of": [4], "direction": [1, 0, 0], "count": "n", "spacing": 15},
      {"id": 6, "name": "", "type": "fillet", "edges": {"between": ["f2:end", "f4:side.0@f5.2"]}, "radius": 0.5}
    ]"#;
    let mut doc = clicked(&part(&[("n", 4.0)], features));
    assert_eq!(field(&doc, 6, "edges"), r#"{"between":["f2:end","f4:side.0@f5.2"]}"#);
    let rim_at = |doc: &mut Document| {
        let outcome = doc.rebuild(None);
        assert_eq!(outcome.failed, None, "{:?}", outcome.steps);
        let body = outcome.body.unwrap();
        let rounded: Vec<usize> = (0..body.faces.len()).filter(|&f| FaceRef::from_tag(body.faces[f].tag).feature == FeatureId(6)).collect();
        assert!(!rounded.is_empty());
        rounded.iter().map(|&f| body.face_aabb(f).center().x).sum::<f64>() / rounded.len() as f64
    };
    assert!((rim_at(&mut doc) - 40.0).abs() < 1e-6);
    doc.set_parameter("n", 6.0).unwrap();
    assert!((rim_at(&mut doc) - 40.0).abs() < 1e-6, "copy 2 is still the one 30 along");
    doc.set_parameter("n", 2.0).unwrap();
    let outcome = doc.rebuild(None);
    assert_eq!(outcome.failed.map(|i| outcome.steps[i].id), Some(FeatureId(6)), "copy 2 has gone, so its rim can't be rounded");
}

#[test]
fn keys_read_back() {
    for text in ["f2:end{f6:side.1}", "f2:end{f2:side.3,f6:side.1@f7.2}", "f4:side.0~1", "f3:side.1[f2:end|f2:side.0]@f5.1{f2:side.3}~0", "f4:side.0#1"] {
        let key = FaceKey::parse(text).unwrap();
        assert_eq!(key.text(), text);
    }
    for bad in ["f2:end{f6:side.1#0}", "f2:end{}", "f2:end{f6:side.1", "f2:end~x", "f2:end#1{f6:side.1}"] {
        assert!(FaceKey::parse(bad).is_err(), "{bad}");
    }
    let pick = read_edge_select(&json::parse(r#"{"between": ["f2:end{f4:side.1}", "f4:side.1"], "ends": ["f1:side.0"], "nth": 1}"#).unwrap()).unwrap();
    assert_eq!(edge_select_value(&pick).to_text(), r#"{"between":["f2:end{f4:side.1}","f4:side.1"],"ends":["f1:side.0"],"nth":1}"#);
}
