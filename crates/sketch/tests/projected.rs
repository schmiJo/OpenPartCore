//! Geometry projected from a part, as the solver sees it: fixed where its document put it, usable by every constraint and
//! dimension, construction unless it says otherwise, and kept in files where it last was.

use coreshape_sketch::json::{self, Value};
use coreshape_sketch::*;

fn from_edge() -> Value {
    json::parse(r#"{"edge": {"between": ["f2:end", "f2:side.0"]}}"#).unwrap()
}

/// A projected line along x from (0, 0) to (10, 0), and a projected point at (10, 5).
fn with_part() -> (Sketch, EntityId, EntityId) {
    let mut s = Sketch::new();
    let edge = s.project(from_edge());
    s.place_projected(edge, Geometry::Line { start: [0.0, 0.0], end: [10.0, 0.0] }).unwrap();
    let corner = s.project(json::parse(r#"{"vertex": {"faces": ["f2:end", "f2:side.1"], "near": [10, 5, 3]}}"#).unwrap());
    s.place_projected(corner, Geometry::Point { at: [10.0, 5.0] }).unwrap();
    (s, edge, corner)
}

fn near(a: [f64; 2], b: [f64; 2], what: &str) {
    assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-7, "{what}: {a:?} isn't {b:?}");
}

fn point_of(s: &Sketch, r: Ref) -> [f64; 2] {
    match (&s.entity(r.entity).unwrap().geometry, r.part) {
        (Geometry::Point { at }, Part::Whole) => *at,
        (Geometry::Line { start, .. }, Part::Start) => *start,
        (Geometry::Line { end, .. }, Part::End) => *end,
        (Geometry::Circle { center, .. } | Geometry::Arc { center, .. }, Part::Center) => *center,
        (Geometry::Arc { start, .. }, Part::Start) => *start,
        (Geometry::Arc { end, .. }, Part::End) => *end,
        other => panic!("{other:?}"),
    }
}

#[test]
fn it_saves_where_it_last_was_and_reads_back() {
    let (mut s, edge, corner) = with_part();
    s.set_construction(edge, false).unwrap();
    let text = s.save_text();
    assert!(text.contains("\"projected\"") && text.contains("\"from\"") && text.contains("\"geometry\""), "{text}");
    let back = Sketch::load_text(&text).unwrap();
    assert_eq!(back, s);
    assert!(!back.entity(edge).unwrap().construction && back.entity(corner).unwrap().construction);
    // As an app writes one: no geometry yet, and construction by default.
    let v = json::parse(r#"{"format": "coreshape-sketch", "version": 1, "entities": [{"id": 9, "type": "projected", "from": {"vertex": {"faces": ["f2:end"], "near": [0, 0, 0]}}}], "constraints": []}"#).unwrap();
    let fresh = Sketch::from_json(&v).unwrap();
    let e = fresh.entity(EntityId(9)).unwrap();
    assert!(e.construction && !e.projection.as_ref().unwrap().placed);
    assert_eq!(Sketch::from_json(&fresh.to_json()).unwrap(), fresh);
    // Until it's placed it can't be solved, and says how to place it.
    let why = fresh.solve().unwrap_err().message;
    assert!(why.contains("entity 9") && why.contains("\"context\""), "{why}");
    // "from" has to say something.
    let bad = json::parse(r#"{"format": "coreshape-sketch", "version": 1, "entities": [{"id": 9, "type": "projected", "from": 3}], "constraints": []}"#).unwrap();
    assert!(Sketch::from_json(&bad).is_err());
}

#[test]
fn constraints_reach_it_and_it_doesnt_move() {
    let (mut s, edge, corner) = with_part();
    let line = s.line([1.0, 1.0], [7.0, 4.0]);
    // One end on the part's corner, the other on its edge, 4 along from the edge's start.
    s.constrain(Rule::Coincident { a: line.start(), b: corner.whole() });
    s.constrain(Rule::Coincident { a: line.end(), b: edge.whole() });
    s.constrain(Rule::Distance { a: line.end(), b: Some(edge.start()), value: 4.0.into() });
    let solved = s.solve().unwrap();
    near(point_of(&solved.sketch, line.start()), [10.0, 5.0], "on the corner");
    near(point_of(&solved.sketch, line.end()), [4.0, 0.0], "on the edge");
    assert_eq!(solved.analysis.dof, 0, "{:?}", solved.analysis.free);
    // The part's geometry is where it was.
    assert_eq!(solved.sketch.entity(edge), s.entity(edge));
    assert_eq!(solved.sketch.entity(corner), s.entity(corner));
}

#[test]
fn it_takes_no_freedom_of_its_own() {
    let (s, ..) = with_part();
    assert_eq!(s.solve().unwrap().analysis.dof, 0);
    let (mut s, ..) = with_part();
    s.point(3.0, 3.0);
    assert_eq!(s.solve().unwrap().analysis.dof, 2);
}

#[test]
fn tangent_to_a_projected_arc() {
    let mut s = Sketch::new();
    let rim = s.project(from_edge());
    s.place_projected(rim, Geometry::Arc { center: [0.0, 0.0], radius: 5.0, start: [5.0, 0.0], end: [0.0, 5.0] }).unwrap();
    let line = s.line([6.0, -3.0], [6.5, 4.0]);
    s.constrain(Rule::Tangent { a: line, b: rim });
    s.constrain(Rule::Vertical { a: line.whole(), b: None });
    let solved = s.solve().unwrap();
    let Geometry::Line { start, end } = solved.sketch.entity(line).unwrap().geometry else { panic!() };
    assert!((start[0] - 5.0).abs() < 1e-7 && (end[0] - 5.0).abs() < 1e-7, "{start:?} {end:?}");
    assert_eq!(solved.sketch.entity(rim), s.entity(rim));
}

#[test]
fn dragging_it_is_refused_and_dragging_what_holds_to_it_leaves_it() {
    let (mut s, edge, corner) = with_part();
    let why = s.drag(Grab::Point(corner.whole()), [3.0, 3.0]).unwrap_err().message;
    assert!(why.contains("entity 2") && why.contains("projected"), "{why}");
    assert!(s.drag(Grab::Edge { entity: edge, at: [5.0, 0.0] }, [5.0, 3.0]).is_err());
    let p = s.point(2.0, 2.0);
    s.constrain(Rule::Coincident { a: p.whole(), b: edge.whole() });
    let (moved, _) = s.drag(Grab::Point(p.whole()), [7.0, 3.0]).unwrap();
    near(point_of(&moved, p.whole()), [7.0, 0.0], "slid along the edge");
    assert_eq!(moved.entity(edge), s.entity(edge));
}

#[test]
fn constraints_between_two_projections_are_checked() {
    let (mut s, edge, corner) = with_part();
    s.constrain(Rule::Coincident { a: corner.whole(), b: edge.whole() });
    let why = s.solve().unwrap_err().message;
    assert!(why.contains("can't be met"), "{why}");
}

#[test]
fn a_projected_curve_bounds_regions_only_when_it_isnt_construction() {
    // Three lines closing a rectangle on a projected edge.
    let (mut s, edge, _) = with_part();
    let (a, b, c) = (s.line([10.0, 0.0], [10.0, 6.0]), s.line([10.0, 6.0], [0.0, 6.0]), s.line([0.0, 6.0], [0.0, 0.0]));
    s.constrain(Rule::Coincident { a: edge.end(), b: a.start() });
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Coincident { a: b.end(), b: c.start() });
    s.constrain(Rule::Coincident { a: c.end(), b: edge.start() });
    let solved = s.solve().unwrap().sketch;
    assert!(solved.regions().is_err(), "as construction, it closes nothing");
    s.set_construction(edge, false).unwrap();
    let solved = s.solve().unwrap().sketch;
    let regions = solved.regions().unwrap();
    assert_eq!(regions.len(), 1);
    assert!((regions[0].area() - 60.0).abs() < 1e-9);
    assert!(regions[0].outer.segs.iter().any(|seg| seg.entity == edge), "the projected edge is one of its sides");
}
