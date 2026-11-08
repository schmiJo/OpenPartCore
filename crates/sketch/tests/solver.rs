//! What the solver promises, case by case. Random sketches are in `fuzz.rs`.

use coreshape_sketch::fuzz::{violations, CHECK_TOLERANCE};
use coreshape_sketch::*;

fn rectangle(s: &mut Sketch, w: f64, h: f64) -> [EntityId; 4] {
    let a = s.line([0.0, 0.0], [w, 0.0]);
    let b = s.line([w, 0.0], [w, h]);
    let c = s.line([w, h], [0.0, h]);
    let d = s.line([0.0, h], [0.0, 0.0]);
    for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
        s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
    }
    [a, b, c, d]
}

fn level(s: &mut Sketch, l: [EntityId; 4]) {
    s.constrain(Rule::Horizontal { a: l[0].whole(), b: None });
    s.constrain(Rule::Vertical { a: l[1].whole(), b: None });
    s.constrain(Rule::Horizontal { a: l[2].whole(), b: None });
    s.constrain(Rule::Vertical { a: l[3].whole(), b: None });
}

fn line_ends(s: &Sketch, id: EntityId) -> (P2, P2) {
    match s.entity(id).unwrap().geometry {
        Geometry::Line { start, end } => (start, end),
        _ => panic!("not a line"),
    }
}

fn near(a: P2, b: P2) -> bool {
    (a[0] - b[0]).abs() < 1e-7 && (a[1] - b[1]).abs() < 1e-7
}

// ----- Degrees of freedom -----

#[test]
fn free_geometry_counts_its_own_freedom() {
    let mut s = Sketch::new();
    s.point(1.0, 2.0);
    assert_eq!(s.analyze().unwrap().dof, 2);
    s.line([0.0, 0.0], [1.0, 0.0]);
    assert_eq!(s.analyze().unwrap().dof, 6);
    s.circle([0.0, 0.0], 3.0);
    assert_eq!(s.analyze().unwrap().dof, 9);
    s.arc([0.0, 0.0], 3.0, 0.0, 90.0);
    assert_eq!(s.analyze().unwrap().dof, 14);
}

#[derive(Clone, Copy)]
struct Ids {
    a: EntityId,
    b: EntityId,
    p: EntityId,
    c: EntityId,
    d: EntityId,
    arc: EntityId,
}

#[test]
#[allow(unused_variables)]
fn each_constraint_takes_the_freedom_it_should() {
    // The freedom of two lines, a point and a circle, and what each rule takes from it.
    let build = || {
        let mut s = Sketch::new();
        let a = s.line([0.0, 0.0], [10.0, 1.0]);
        let b = s.line([3.0, 8.0], [14.0, 6.0]);
        let p = s.point(30.0, 30.0);
        let c = s.circle([50.0, 0.0], 5.0);
        let d = s.circle([70.0, 3.0], 4.0);
        let arc = s.arc([90.0, 0.0], 6.0, 10.0, 80.0);
        (s, a, b, p, c, d, arc)
    };
    let (base, ..) = build();
    let all = base.analyze().unwrap().dof;
    assert_eq!(all, 4 + 4 + 2 + 3 + 3 + 5);
    type Maker = fn(Ids) -> Rule;
    let cases: Vec<(&str, usize, Maker)> = vec![
        ("coincident", 2, |Ids { a, b, p, c, d, arc }| Rule::Coincident { a: a.end(), b: b.start() }),
        ("point on line", 1, |Ids { a, b, p, c, d, arc }| Rule::Coincident { a: p.whole(), b: a.whole() }),
        ("point on circle", 1, |Ids { a, b, p, c, d, arc }| Rule::Coincident { a: p.whole(), b: c.whole() }),
        ("point on arc", 1, |Ids { a, b, p, c, d, arc }| Rule::Coincident { a: p.whole(), b: arc.whole() }),
        ("horizontal", 1, |Ids { a, b, p, c, d, arc }| Rule::Horizontal { a: a.whole(), b: None }),
        ("horizontal points", 1, |Ids { a, b, p, c, d, arc }| Rule::Horizontal { a: a.start(), b: Some(b.end()) }),
        ("vertical", 1, |Ids { a, b, p, c, d, arc }| Rule::Vertical { a: a.whole(), b: None }),
        ("parallel", 1, |Ids { a, b, p, c, d, arc }| Rule::Parallel { a, b }),
        ("perpendicular", 1, |Ids { a, b, p, c, d, arc }| Rule::Perpendicular { a, b }),
        ("tangent line and circle", 1, |Ids { a, b, p, c, d, arc }| Rule::Tangent { a, b: c }),
        ("tangent line and arc", 1, |Ids { a, b, p, c, d, arc }| Rule::Tangent { a, b: arc }),
        ("tangent circles", 1, |Ids { a, b, p, c, d, arc }| Rule::Tangent { a: c, b: d }),
        ("equal lines", 1, |Ids { a, b, p, c, d, arc }| Rule::Equal { a, b }),
        ("equal circles", 1, |Ids { a, b, p, c, d, arc }| Rule::Equal { a: c, b: d }),
        ("midpoint", 2, |Ids { a, b, p, c, d, arc }| Rule::Midpoint { point: p.whole(), line: a }),
        ("concentric", 2, |Ids { a, b, p, c, d, arc }| Rule::Concentric { a: c, b: d }),
        ("symmetric about a line", 2, |Ids { a, b, p, c, d, arc }| Rule::Symmetric { a: p.whole(), b: b.start(), about: a.whole() }),
        ("symmetric about a point", 2, |Ids { a, b, p, c, d, arc }| Rule::Symmetric { a: a.start(), b: b.start(), about: p.whole() }),
        ("normal to a circle", 1, |Ids { a, b, p, c, d, arc }| Rule::Normal { a, b: c }),
        ("normal circles", 1, |Ids { a, b, p, c, d, arc }| Rule::Normal { a: c, b: d }),
        ("distance", 1, |Ids { a, b, p, c, d, arc }| Rule::Distance { a: a.start(), b: Some(b.start()), value: 12.0.into() }),
        ("length", 1, |Ids { a, b, p, c, d, arc }| Rule::Distance { a: a.whole(), b: None, value: 12.0.into() }),
        ("point to line", 1, |Ids { a, b, p, c, d, arc }| Rule::Distance { a: p.whole(), b: Some(a.whole()), value: 12.0.into() }),
        ("horizontal distance", 1, |Ids { a, b, p, c, d, arc }| Rule::HorizontalDistance { a: a.start(), b: b.start(), value: (-2.0).into() }),
        ("vertical distance", 1, |Ids { a, b, p, c, d, arc }| Rule::VerticalDistance { a: a.start(), b: b.start(), value: 5.0.into() }),
        ("radius", 1, |Ids { a, b, p, c, d, arc }| Rule::Radius { of: c, value: 8.0.into() }),
        ("diameter", 1, |Ids { a, b, p, c, d, arc }| Rule::Diameter { of: c, value: 8.0.into() }),
        ("angle", 1, |Ids { a, b, p, c, d, arc }| Rule::Angle { a, b: Some(b), value: 30.0.into() }),
        ("angle from x", 1, |Ids { a, b, p, c, d, arc }| Rule::Angle { a, b: None, value: 30.0.into() }),
        ("arc span", 1, |Ids { a, b, p, c, d, arc }| Rule::Angle { a: arc, b: None, value: 120.0.into() }),
    ];
    for (name, taken, make) in cases {
        let (mut s, a, b, p, c, d, arc) = build();
        s.constrain(make(Ids { a, b, p, c, d, arc }));
        let solved = s.solve().unwrap_or_else(|e| panic!("{name}: {}", e.message));
        assert_eq!(all - solved.analysis.dof, taken, "{name}");
        assert!(solved.analysis.redundant.is_empty(), "{name}: {:?}", solved.analysis.redundant);
        let bad = violations(&solved.sketch, CHECK_TOLERANCE);
        assert!(bad.is_empty(), "{name}: {bad:?}");
    }
}

#[test]
fn fixing_takes_all_of_an_entitys_freedom() {
    for (build, freedom) in [
        ((|s: &mut Sketch| s.point(1.0, 2.0)) as fn(&mut Sketch) -> EntityId, 2),
        (|s: &mut Sketch| s.line([0.0, 0.0], [4.0, 1.0]), 4),
        (|s: &mut Sketch| s.circle([0.0, 0.0], 4.0), 3),
        (|s: &mut Sketch| s.arc([0.0, 0.0], 4.0, 15.0, 200.0), 5),
    ] {
        let mut s = Sketch::new();
        let e = build(&mut s);
        assert_eq!(s.analyze().unwrap().dof, freedom);
        s.fix(e).unwrap();
        let solved = s.solve().unwrap();
        assert_eq!(solved.analysis.dof, 0, "{:?}", s.entity(e));
        assert!(solved.analysis.free.is_empty());
        assert!(solved.analysis.redundant.is_empty(), "{:?}", solved.analysis.redundant);
    }
}

#[test]
fn a_fully_dimensioned_rectangle_has_no_freedom_and_lands_where_it_should() {
    let mut s = Sketch::new();
    let l = rectangle(&mut s, 30.0, 12.0);
    // Start it crooked, and let the constraints straighten it.
    s.entities[0].geometry = Geometry::Line { start: [1.0, 1.0], end: [28.0, 3.0] };
    level(&mut s, l);
    s.constrain(Rule::Coincident { a: l[0].start(), b: ORIGIN.whole() });
    s.constrain(Rule::Distance { a: l[0].whole(), b: None, value: 40.0.into() });
    s.constrain(Rule::Distance { a: l[1].whole(), b: None, value: 25.0.into() });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 0);
    assert!(solved.analysis.redundant.is_empty(), "{:?}", solved.analysis.redundant);
    let (a, b) = line_ends(&solved.sketch, l[1]);
    assert!(near(a, [40.0, 0.0]) && near(b, [40.0, 25.0]), "{a:?} {b:?}");
}

#[test]
fn what_is_still_free_gets_named() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let p = s.point(20.0, 20.0);
    s.fix(a.start()).unwrap();
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 3);
    // The fixed end isn't free, and the other can slide. So can the point.
    assert!(solved.analysis.free.contains(&a.end()));
    assert!(solved.analysis.free.contains(&p.whole()));
    assert!(!solved.analysis.free.contains(&a.start()));
}

// ----- Redundant and conflicting -----

#[test]
fn the_constraint_that_adds_nothing_is_named() {
    let mut s = Sketch::new();
    let l = rectangle(&mut s, 30.0, 12.0);
    level(&mut s, l);
    let extra = s.constrain(Rule::Parallel { a: l[0], b: l[2] });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.redundant.len(), 1, "{:?}", solved.analysis.redundant);
    let d = &solved.analysis.redundant[0];
    assert_eq!(d.constraint, extra);
    assert!(!d.partial);
    assert_eq!(d.with.len(), 2, "two horizontals imply it: {:?}", d.with);
}

#[test]
fn a_duplicate_is_redundant_and_keeps_the_same_freedom() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 3.0]);
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let before = s.solve().unwrap().analysis.dof;
    let again = s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, before);
    assert_eq!(solved.analysis.redundant[0].constraint, again);
}

#[test]
fn half_of_a_coincidence_can_be_redundant() {
    let mut s = Sketch::new();
    let (p, q) = (s.point(0.0, 0.0), s.point(5.0, 0.0));
    s.constrain(Rule::Horizontal { a: p.whole(), b: Some(q.whole()) });
    let c = s.constrain(Rule::Coincident { a: p.whole(), b: q.whole() });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 2);
    assert_eq!(solved.analysis.redundant.len(), 1);
    assert_eq!(solved.analysis.redundant[0].constraint, c);
    assert!(solved.analysis.redundant[0].partial);
}

#[test]
fn two_lengths_for_one_line_are_named_and_the_sketch_is_left_alone() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let keep = s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let one = s.constrain(Rule::Distance { a: a.whole(), b: None, value: 10.0.into() });
    let two = s.constrain(Rule::Distance { a: a.whole(), b: None, value: 20.0.into() });
    let before = s.clone();
    let e = s.solve().unwrap_err();
    assert_eq!(e.kind, ErrorKind::Conflict);
    assert_eq!(e.constraints, vec![one, two], "{}", e.message);
    assert!(!e.constraints.contains(&keep), "the horizontal has nothing to do with it");
    assert!(e.message.contains("c2") && e.message.contains("c3"), "{}", e.message);
    assert_eq!(s, before);
}

#[test]
fn a_triangle_that_can_not_close_names_its_three_sides() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [5.0, 0.0]);
    let b = s.line([5.0, 0.0], [5.0, 5.0]);
    let c = s.line([5.0, 5.0], [0.0, 0.0]);
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Coincident { a: b.end(), b: c.start() });
    s.constrain(Rule::Coincident { a: c.end(), b: a.start() });
    let free = s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let sides = [
        s.constrain(Rule::Distance { a: a.whole(), b: None, value: 5.0.into() }),
        s.constrain(Rule::Distance { a: b.whole(), b: None, value: 5.0.into() }),
        s.constrain(Rule::Distance { a: c.whole(), b: None, value: 50.0.into() }),
    ];
    let e = s.solve().unwrap_err();
    for side in sides {
        assert!(e.constraints.contains(&side), "{}", e.message);
    }
    assert!(!e.constraints.contains(&free), "{}", e.message);
    // Any one side removed, and it solves.
    for side in sides {
        let mut t = s.clone();
        t.remove_constraint(side).unwrap();
        t.solve().unwrap_or_else(|e| panic!("{}", e.message));
    }
}

#[test]
fn horizontal_and_vertical_on_one_line_collapses_it_and_says_so() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 3.0]);
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.constrain(Rule::Vertical { a: a.whole(), b: None });
    let e = s.solve().unwrap_err();
    assert_eq!(e.kind, ErrorKind::Collapsed, "{}", e.message);
    assert_eq!(e.entities, vec![a]);
    assert_eq!(e.constraints.len(), 2, "{}", e.message);
}

#[test]
fn bad_input_is_refused_with_a_reason() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let c = s.circle([0.0, 0.0], 5.0);
    let missing = EntityId(99);
    let rules: Vec<(Rule, &str)> = vec![
        (Rule::Parallel { a, b: c }, "not a line"),
        (Rule::Parallel { a, b: missing }, "not a line"),
        (Rule::Tangent { a, b: a }, "can't be tangent"),
        (Rule::Equal { a, b: c }, "equal needs"),
        (Rule::Radius { of: a, value: 1.0.into() }, "not a circle"),
        (Rule::Radius { of: c, value: (-1.0).into() }, "must be positive"),
        (Rule::Radius { of: c, value: 0.0.into() }, "must be positive"),
        (Rule::Distance { a: a.whole(), b: None, value: (-5.0).into() }, "must be positive"),
        (Rule::Distance { a: c.whole(), b: None, value: 5.0.into() }, "not a line"),
        (Rule::Coincident { a: a.whole(), b: c.whole() }, "at least one point"),
        (Rule::Coincident { a: c.start(), b: a.start() }, "not a point"),
        (Rule::Midpoint { point: a.start(), line: c }, "not a line"),
        (Rule::Angle { a: c, b: None, value: 3.0.into() }, "no angle"),
        (Rule::Fix { what: a.start(), at: vec![1.0] }, "takes 2 values"),
        (Rule::Distance { a: a.whole(), b: None, value: "nope * 2".into() }, "no parameter"),
        (Rule::Normal { a, b: a }, "for two lines"),
    ];
    for (rule, expected) in rules {
        let mut t = s.clone();
        let id = t.constrain(rule.clone());
        let e = t.solve().unwrap_err();
        assert_eq!(e.kind, ErrorKind::Invalid, "{rule:?}");
        assert!(e.message.contains(expected), "{rule:?}\n  said: {}\n  wanted: {expected}", e.message);
        assert_eq!(e.constraints, vec![id]);
    }
}

// ----- Moving as little as possible -----

#[test]
fn solving_leaves_alone_whatever_the_constraints_do_not_involve() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 3.0]);
    let other = s.line([50.0, 50.0], [60.0, 53.0]);
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let solved = s.solve().unwrap();
    assert_eq!(line_ends(&solved.sketch, other), line_ends(&s, other), "to the bit");
}

#[test]
fn a_new_constraint_moves_things_only_as_far_as_it_must() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 2.0]);
    s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    let solved = s.solve().unwrap();
    let (p, q) = line_ends(&solved.sketch, a);
    // The smallest move shares the difference, and x doesn't change at all.
    assert!(near(p, [0.0, 1.0]) && near(q, [10.0, 1.0]), "{p:?} {q:?}");
}

#[test]
fn already_solved_sketches_do_not_move_at_all() {
    let mut s = Sketch::new();
    let l = rectangle(&mut s, 30.0, 12.0);
    level(&mut s, l);
    let solved = s.solve().unwrap();
    assert_eq!(solved.sketch, s);
    assert_eq!(solved.iterations, 0);
}

// ----- Dragging -----

fn free_rectangle() -> (Sketch, [EntityId; 4]) {
    let mut s = Sketch::new();
    let l = rectangle(&mut s, 30.0, 12.0);
    level(&mut s, l);
    (s, l)
}

#[test]
fn a_dragged_corner_follows_and_the_rectangle_stays_a_rectangle() {
    let (s, l) = free_rectangle();
    let (after, report) = s.drag(Grab::Point(l[1].end()), [50.0, 40.0]).unwrap();
    assert!(report.reached, "{report:?}");
    let (_, corner) = line_ends(&after, l[1]);
    assert!((corner[0] - 50.0).abs() < 1e-6 && (corner[1] - 40.0).abs() < 1e-6, "{corner:?}");
    assert!(violations(&after, CHECK_TOLERANCE).is_empty());
}

#[test]
fn dragging_does_not_touch_what_is_not_connected() {
    let (mut s, l) = free_rectangle();
    let other = s.circle([200.0, 200.0], 7.0);
    let (after, _) = s.drag(Grab::Point(l[1].end()), [50.0, 40.0]).unwrap();
    assert_eq!(after.entity(other), s.entity(other), "to the bit");
}

#[test]
fn dragging_stops_at_what_the_constraints_allow() {
    // A fixed-length arm on a fixed pivot can't reach past its length.
    let mut s = Sketch::new();
    let arm = s.line([0.0, 0.0], [10.0, 0.0]);
    s.constrain(Rule::Coincident { a: arm.start(), b: ORIGIN.whole() });
    s.constrain(Rule::Distance { a: arm.whole(), b: None, value: 10.0.into() });
    let (after, report) = s.drag(Grab::Point(arm.end()), [30.0, 30.0]).unwrap();
    assert!(!report.reached);
    assert!(violations(&after, CHECK_TOLERANCE).is_empty(), "{:?}", violations(&after, CHECK_TOLERANCE));
    let (_, tip) = line_ends(&after, arm);
    let expected = 10.0 / 2.0f64.sqrt();
    assert!((tip[0] - expected).abs() < 1e-4 && (tip[1] - expected).abs() < 1e-4, "{tip:?}");
}

#[test]
fn a_far_away_pointer_is_still_stable() {
    // The pull is much stronger than the arm's length, which breaks an iteration with no line search.
    let mut s = Sketch::new();
    let arm = s.line([0.0, 0.0], [10.0, 0.0]);
    s.constrain(Rule::Coincident { a: arm.start(), b: ORIGIN.whole() });
    s.constrain(Rule::Distance { a: arm.whole(), b: None, value: 10.0.into() });
    let (after, _) = s.drag(Grab::Point(arm.end()), [3000.0, 4000.0]).unwrap();
    assert!(violations(&after, CHECK_TOLERANCE).is_empty());
    let (_, tip) = line_ends(&after, arm);
    assert!((tip[0] - 6.0).abs() < 1e-3 && (tip[1] - 8.0).abs() < 1e-3, "{tip:?}");
}

#[test]
fn going_back_to_the_start_restores_the_sketch() {
    let (s, l) = free_rectangle();
    let start = s.solve().unwrap().sketch;
    let (_, home) = line_ends(&start, l[1]);
    let mut session = DragSession::begin(&s, Grab::Point(l[1].end())).unwrap();
    for step in 1..=20 {
        let t = step as f64 / 20.0 * std::f64::consts::TAU;
        session.move_to([home[0] + 15.0 * t.sin(), home[1] + 10.0 * (1.0 - t.cos())]);
    }
    let report = session.move_to(home);
    assert!(report.reached);
    let moved = coreshape_sketch::fuzz::largest_move(&session.sketch(), &start);
    assert!(moved < 1e-3, "moved {moved}");
}

#[test]
fn a_chain_shares_the_motion_and_leaves_the_far_end_put() {
    // Two equal-length links with a fixed end. Dragging the far end can't move the fixed one.
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let b = s.line([10.0, 0.0], [20.0, 0.0]);
    s.fix(a.start()).unwrap();
    s.constrain(Rule::Coincident { a: a.end(), b: b.start() });
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: 10.0.into() });
    s.constrain(Rule::Distance { a: b.whole(), b: None, value: 10.0.into() });
    let (after, report) = s.drag(Grab::Point(b.end()), [12.0, 9.0]).unwrap();
    assert!(report.reached, "{report:?}");
    assert!(violations(&after, CHECK_TOLERANCE).is_empty());
    let (start, _) = line_ends(&after, a);
    assert!(start[0].abs() < 1e-10 && start[1].abs() < 1e-10, "{start:?}");
}

#[test]
fn grabbing_a_line_moves_it_along() {
    let (s, l) = free_rectangle();
    let (after, _) = s.drag(Grab::Edge { entity: l[2], at: [15.0, 12.0] }, [15.0, 30.0]).unwrap();
    let (p, q) = line_ends(&after, l[2]);
    assert!((p[1] - 30.0).abs() < 1e-3 && (q[1] - 30.0).abs() < 1e-3, "{p:?} {q:?}");
    assert!(violations(&after, CHECK_TOLERANCE).is_empty());
}

#[test]
fn grabbing_a_circle_changes_its_radius() {
    let mut s = Sketch::new();
    let c = s.circle([0.0, 0.0], 5.0);
    s.fix(c.center()).unwrap();
    let (after, report) = s.drag(Grab::Edge { entity: c, at: [5.0, 0.0] }, [0.0, 12.0]).unwrap();
    assert!(report.reached, "{report:?}");
    match after.entity(c).unwrap().geometry {
        Geometry::Circle { radius, .. } => assert!((radius - 12.0).abs() < 1e-3, "{radius}"),
        _ => panic!(),
    }
}

#[test]
fn a_fixed_point_does_not_budge_when_grabbed() {
    let mut s = Sketch::new();
    let p = s.point(3.0, 4.0);
    s.fix(p).unwrap();
    let (after, report) = s.drag(Grab::Point(p.whole()), [30.0, 40.0]).unwrap();
    assert!(!report.reached);
    match after.entity(p).unwrap().geometry {
        Geometry::Point { at } => assert!((at[0] - 3.0).abs() < 1e-10 && (at[1] - 4.0).abs() < 1e-10, "{at:?}"),
        _ => panic!(),
    }
}

#[test]
fn a_drag_never_starts_from_a_broken_sketch() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: 10.0.into() });
    s.constrain(Rule::Distance { a: a.whole(), b: None, value: 20.0.into() });
    let error = s.drag(Grab::Point(a.end()), [1.0, 1.0]).err().expect("a conflict");
    assert_eq!(error.kind, ErrorKind::Conflict);
}

// ----- Regions -----

#[test]
fn a_rectangle_is_one_region() {
    let (s, _) = free_rectangle();
    let regions = s.regions().unwrap();
    assert_eq!(regions.len(), 1);
    assert!((regions[0].outer.signed_area() - 360.0).abs() < 1e-9);
    assert!(regions[0].holes.is_empty());
}

#[test]
fn a_loop_inside_is_a_hole_and_a_loop_inside_that_is_a_new_region() {
    let (mut s, _) = free_rectangle();
    let hole = s.circle([15.0, 6.0], 5.0);
    let island = s.circle([15.0, 6.0], 2.0);
    let beside = s.circle([60.0, 6.0], 3.0);
    let regions = s.regions().unwrap();
    assert_eq!(regions.len(), 3);
    let by_area = |a: f64| regions.iter().find(|r| (r.outer.signed_area() - a).abs() < 1e-6);
    let plate = by_area(360.0).expect("the plate");
    assert_eq!(plate.holes.len(), 1);
    assert_eq!(plate.holes[0].segs[0].entity, hole);
    assert!(plate.holes[0].signed_area() < 0.0, "holes run clockwise");
    let middle = by_area(4.0 * std::f64::consts::PI).expect("the island");
    assert_eq!(middle.outer.segs[0].entity, island);
    let side = by_area(9.0 * std::f64::consts::PI).expect("the circle beside");
    assert_eq!(side.outer.segs[0].entity, beside);
}

#[test]
fn an_arc_outline_has_the_right_area_in_either_winding() {
    // A slot: two lines and two semicircles, drawn both ways.
    let build = |flip: bool| {
        let mut s = Sketch::new();
        let right = s.arc([40.0, 0.0], 5.0, -90.0, 90.0);
        let left = s.arc([0.0, 0.0], 5.0, 90.0, 270.0);
        let (bottom, top) = if flip {
            (s.line([40.0, -5.0], [0.0, -5.0]), s.line([0.0, 5.0], [40.0, 5.0]))
        } else {
            (s.line([0.0, -5.0], [40.0, -5.0]), s.line([40.0, 5.0], [0.0, 5.0]))
        };
        let _ = (right, left, bottom, top);
        s
    };
    for flip in [false, true] {
        let regions = build(flip).regions().unwrap();
        assert_eq!(regions.len(), 1);
        let want = 40.0 * 10.0 + std::f64::consts::PI * 25.0;
        assert!((regions[0].outer.signed_area() - want).abs() < 1e-9, "{}", regions[0].outer.signed_area());
        // Each segment starts where the last one ended.
        let segs = &regions[0].outer.segs;
        for i in 0..segs.len() {
            let (a, b) = (segs[i].end(), segs[(i + 1) % segs.len()].start());
            assert!(near(a, b), "{a:?} {b:?}");
        }
    }
}

#[test]
fn a_point_between_an_arc_and_its_chord_counts_as_inside() {
    // The hole sits in the bulge of a big arc, outside the polygon of its ends.
    let mut s = Sketch::new();
    let arc = s.arc([0.0, 0.0], 20.0, 0.0, 180.0);
    let base = s.line([-20.0, 0.0], [20.0, 0.0]);
    let _ = (arc, base);
    let hole = s.circle([0.0, 15.0], 2.0);
    let regions = s.regions().unwrap();
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].holes[0].segs[0].entity, hole);
}

#[test]
fn open_outlines_are_refused_by_name() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [10.0, 0.0]);
    let b = s.line([10.0, 0.0], [10.0, 10.0]);
    let e = s.regions().unwrap_err();
    assert_eq!(e.kind, ErrorKind::Regions);
    assert!(e.message.contains("isn't closed"), "{}", e.message);
    assert!(e.entities == vec![a] || e.entities == vec![b], "{:?}", e.entities);
}

#[test]
fn an_empty_sketch_has_no_regions_and_says_so() {
    let mut s = Sketch::new();
    s.point(0.0, 0.0);
    assert!(s.regions().unwrap_err().message.contains("no closed regions"));
}

// ----- As data -----

#[test]
fn sketches_round_trip_through_json_with_expressions_intact() {
    let mut s = Sketch::new();
    let a = s.line([0.25, 0.1 + 0.2], [10.0, 1.0 / 3.0]);
    let b = s.line([1.0, 5.0], [9.0, 5.0]);
    let c = s.circle([0.0, 0.0], 5.0);
    let d = s.circle([20.0, 0.0], 5.0);
    let arc = s.arc([40.0, 0.0], 6.0, 10.0, 80.0);
    let p = s.point(3.0, 4.0);
    s.set_construction(b, true).unwrap();
    let rules = vec![
        Rule::Coincident { a: a.end(), b: arc.start() },
        Rule::Coincident { a: p.whole(), b: a.whole() },
        Rule::Horizontal { a: a.whole(), b: None },
        Rule::Horizontal { a: a.start(), b: Some(b.end()) },
        Rule::Vertical { a: b.start(), b: Some(ORIGIN.whole()) },
        Rule::Parallel { a, b },
        Rule::Perpendicular { a, b: Y_AXIS },
        Rule::Tangent { a: c, b: d },
        Rule::Equal { a: c, b: d },
        Rule::Midpoint { point: p.whole(), line: a },
        Rule::Concentric { a: c, b: arc },
        Rule::Symmetric { a: a.start(), b: a.end(), about: X_AXIS.whole() },
        Rule::Symmetric { a: a.start(), b: a.end(), about: p.whole() },
        Rule::Normal { a, b: c },
        Rule::Fix { what: p.whole(), at: vec![3.0, 4.0] },
        Rule::Distance { a: a.whole(), b: None, value: "width / 2".into() },
        Rule::Distance { a: p.whole(), b: Some(a.whole()), value: 1.5.into() },
        Rule::HorizontalDistance { a: a.start(), b: b.start(), value: (-2.5).into() },
        Rule::VerticalDistance { a: a.start(), b: b.start(), value: "h + 1".into() },
        Rule::Radius { of: c, value: 5.0.into() },
        Rule::Diameter { of: d, value: "2 * r".into() },
        Rule::Angle { a, b: Some(b), value: 30.0.into() },
        Rule::Angle { a: arc, b: None, value: 70.0.into() },
    ];
    for r in rules {
        s.constrain(r);
    }
    let text = s.save_text();
    let back = Sketch::load_text(&text).unwrap();
    assert_eq!(back, s);
    assert_eq!(back.save_text(), text);
}

#[test]
fn ids_are_never_reused_even_across_a_save() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [1.0, 0.0]);
    let c = s.constrain(Rule::Horizontal { a: a.whole(), b: None });
    s.remove_entity(a).unwrap();
    assert!(s.constraints.is_empty(), "the constraint goes with its entity");
    let mut back = Sketch::load_text(&s.save_text()).unwrap();
    let e = back.point(0.0, 0.0);
    let k = back.constrain(Rule::Fix { what: e.whole(), at: vec![0.0, 0.0] });
    assert!(e.0 > a.0 && k.0 > c.0, "{e:?} {k:?}");
}

#[test]
fn a_hand_written_sketch_loads_and_solves() {
    let text = r#"{
        "format": "coreshape-sketch", "version": 1,
        "entities": [
            {"id": 1, "type": "line", "start": [0, 0], "end": [10, 1]},
            {"id": 2, "type": "circle", "center": [20, 0], "radius": 4}
        ],
        "constraints": [
            {"id": 1, "type": "horizontal", "a": 1},
            {"id": 2, "type": "coincident", "a": "1.start", "b": "origin"},
            {"id": 3, "type": "distance", "a": "1", "value": 12},
            {"id": 4, "type": "tangent", "a": "1", "b": "2"},
            {"id": 5, "type": "radius", "of": "2", "value": 4},
            {"id": 6, "type": "horizontal_distance", "a": "1.end", "b": "2.center", "value": 3}
        ]
    }"#;
    let s = Sketch::load_text(text).unwrap();
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 0);
    let (center, radius) = match solved.sketch.entity(EntityId(2)).unwrap().geometry {
        Geometry::Circle { center, radius } => (center, radius),
        _ => panic!(),
    };
    // Both sides of the line are solutions. It starts tilted above the center, so it stays there.
    assert!(near(center, [15.0, -4.0]), "{center:?}");
    assert_eq!(radius, 4.0);
}

#[test]
fn bad_files_are_refused_with_a_reason() {
    let wrap = |entities: &str, constraints: &str| {
        format!("{{\"format\":\"coreshape-sketch\",\"version\":1,\"entities\":[{entities}],\"constraints\":[{constraints}]}}")
    };
    let line = r#"{"id":1,"type":"line","start":[0,0],"end":[1,0]}"#;
    let cases: Vec<(String, &str)> = vec![
        ("{}".into(), "not a CoreShape sketch"),
        ("nonsense".into(), "not valid JSON"),
        ("{\"format\":\"coreshape-sketch\",\"version\":9}".into(), "version"),
        ("{\"format\":\"coreshape-sketch\",\"version\":1,\"more\":1}".into(), "unknown field"),
        (wrap(r#"{"id":1,"type":"spline"}"#, ""), "unknown entity"),
        (wrap(r#"{"id":1,"type":"line","start":[0,0]}"#, ""), "needs a \"end\""),
        (wrap(r#"{"id":1,"type":"line","start":[0],"end":[1,0]}"#, ""), "written [x, y]"),
        (wrap(&format!("{line},{line}"), ""), "appears twice"),
        (wrap(r#"{"id":0,"type":"point","at":[0,0]}"#, ""), "from 1"),
        (wrap(line, r#"{"id":1,"type":"warp","a":"1"}"#), "unknown constraint"),
        (wrap(line, r#"{"id":1,"type":"horizontal","a":"1","c":2}"#), "unknown field"),
        (wrap(line, r#"{"id":1,"type":"horizontal","a":"2"}"#), "doesn't exist"),
        (wrap(line, r#"{"id":1,"type":"horizontal","a":"1.middle"}"#), "not a reference"),
        (wrap(line, r#"{"id":1,"type":"parallel","a":"1.start","b":"1"}"#), "whole entity"),
        (wrap(line, r#"{"id":1,"type":"distance","a":"1","value":"3 +"}"#), "expression"),
    ];
    for (text, expected) in cases {
        match Sketch::load_text(&text) {
            Ok(_) => panic!("should have refused {text}"),
            Err(e) => assert!(e.message.contains(expected), "{text}\n  said: {}\n  wanted: {expected}", e.message),
        }
    }
}

#[test]
fn the_solved_result_serializes() {
    let (s, _) = free_rectangle();
    let solved = s.solve().unwrap();
    let text = solved.to_json().to_text();
    let v = json::parse(&text).unwrap();
    assert_eq!(v.get("analysis").and_then(|a| a.get("dof")).and_then(json::Value::as_f64), Some(4.0));
    let regions = solved.sketch.regions().unwrap();
    assert!(json::parse(&regions[0].to_json().to_text()).is_ok());
    let e = Sketch::load_text("{}").unwrap_err().to_json().to_text();
    assert!(e.contains("invalid"), "{e}");
}

// ----- Distances to curves, between lines, and reference dimensions -----

#[test]
fn a_point_keeps_its_side_of_a_circle() {
    let mut s = Sketch::new();
    let c = s.circle([0.0, 0.0], 10.0);
    s.fix(c).unwrap();
    let outside = s.point(15.0, 0.0);
    let inside = s.point(4.0, 3.0);
    s.constrain(Rule::Horizontal { a: outside.whole(), b: Some(c.center()) });
    s.constrain(Rule::Horizontal { a: inside.whole(), b: Some(c.center()) });
    let d0 = s.constrain(Rule::Distance { a: outside.whole(), b: Some(c.whole()), value: 7.0.into() });
    let d1 = s.constrain(Rule::Distance { a: c.whole(), b: Some(inside.whole()), value: 2.0.into() });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 0);
    let at = |id: EntityId| match solved.sketch.entity(id).unwrap().geometry {
        Geometry::Point { at } => at,
        _ => panic!(),
    };
    assert!((at(outside)[0] - 17.0).abs() < 1e-9, "{:?}", at(outside));
    assert!((at(inside)[0] - 8.0).abs() < 1e-9, "{:?}", at(inside));
    assert!((solved.sketch.measure(d0).unwrap() - 7.0).abs() < 1e-9);
    assert!((solved.sketch.measure(d1).unwrap() - 2.0).abs() < 1e-9);
    // Asking for more than the radius from inside can only be met from outside.
    let mut far = s.clone();
    far.set_value(d1, 12.0).unwrap();
    let solved = far.solve().unwrap();
    assert!((solved.sketch.measure(d1).unwrap() - 12.0).abs() < 1e-9);
}

#[test]
fn a_line_and_a_circle_and_two_circles() {
    let mut s = Sketch::new();
    let c = s.circle([0.0, 0.0], 10.0);
    s.fix(c).unwrap();
    let l = s.line([-20.0, 14.0], [20.0, 14.0]);
    s.constrain(Rule::Horizontal { a: l.whole(), b: None });
    s.constrain(Rule::Distance { a: l.whole(), b: Some(c.whole()), value: 6.0.into() });
    let outside = s.circle([20.0, 0.0], 3.0);
    s.constrain(Rule::Radius { of: outside, value: 3.0.into() });
    s.constrain(Rule::Horizontal { a: outside.center(), b: Some(c.center()) });
    s.constrain(Rule::Distance { a: c.whole(), b: Some(outside.whole()), value: 5.0.into() });
    let nested = s.circle([1.0, 0.0], 2.0);
    s.constrain(Rule::Radius { of: nested, value: 2.0.into() });
    s.constrain(Rule::Horizontal { a: nested.center(), b: Some(c.center()) });
    s.constrain(Rule::Distance { a: nested.whole(), b: Some(c.whole()), value: 3.0.into() });
    let solved = s.solve().unwrap();
    let k = &solved.sketch;
    match k.entity(l).unwrap().geometry {
        Geometry::Line { start, .. } => assert!((start[1] - 16.0).abs() < 1e-9, "{start:?}"),
        _ => panic!(),
    }
    match k.entity(outside).unwrap().geometry {
        Geometry::Circle { center, .. } => assert!((center[0] - 18.0).abs() < 1e-9, "{center:?}"),
        _ => panic!(),
    }
    match k.entity(nested).unwrap().geometry {
        // 10 - 2 - 3 from the center.
        Geometry::Circle { center, .. } => assert!((center[0] - 5.0).abs() < 1e-9, "{center:?}"),
        _ => panic!(),
    }
    assert!(fuzz::violations(k, 1e-9).is_empty(), "{:?}", fuzz::violations(k, 1e-9));
}

#[test]
fn a_distance_to_an_arc_measures_to_its_full_circle() {
    let mut s = Sketch::new();
    let a = s.arc([0.0, 0.0], 10.0, 0.0, 90.0);
    s.fix(a).unwrap();
    // Opposite the arc, off its ends: it's the circle that counts, as everywhere else.
    let p = s.point(-15.0, 0.0);
    s.constrain(Rule::Horizontal { a: p.whole(), b: Some(a.center()) });
    s.constrain(Rule::Distance { a: p.whole(), b: Some(a.whole()), value: 3.0.into() });
    let solved = s.solve().unwrap();
    match solved.sketch.entity(p).unwrap().geometry {
        Geometry::Point { at } => assert!((at[0] + 13.0).abs() < 1e-9, "{at:?}"),
        _ => panic!(),
    }
}

#[test]
fn parallel_lines_have_a_distance_with_no_point_picked() {
    let mut s = Sketch::new();
    let a = s.line([0.0, 0.0], [20.0, 0.0]);
    s.fix(a).unwrap();
    let b = s.line([3.0, 8.0], [15.0, 9.0]);
    s.constrain(Rule::Parallel { a, b });
    s.constrain(Rule::Distance { a: b.whole(), b: Some(a.whole()), value: 12.0.into() });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 2, "b can still slide along and change length");
    match solved.sketch.entity(b).unwrap().geometry {
        Geometry::Line { start, end } => {
            assert!((start[1] - 12.0).abs() < 1e-9 && (end[1] - 12.0).abs() < 1e-9, "{start:?} {end:?}");
        }
        _ => panic!(),
    }
    assert!(solved.analysis.redundant.is_empty());
    // Against an axis too.
    let mut t = Sketch::new();
    let l = t.line([0.0, 5.0], [10.0, 6.0]);
    t.constrain(Rule::Parallel { a: l, b: X_AXIS });
    t.constrain(Rule::Distance { a: X_AXIS.whole(), b: Some(l.whole()), value: 4.0.into() });
    let solved = t.solve().unwrap();
    match solved.sketch.entity(l).unwrap().geometry {
        Geometry::Line { start, end } => assert!((start[1] - 4.0).abs() < 1e-9 && (end[1] - 4.0).abs() < 1e-9),
        _ => panic!(),
    }
}

#[test]
fn a_reference_dimension_reports_and_drives_nothing() {
    let mut s = Sketch::new();
    let l = s.line([0.0, 0.0], [30.0, 40.0]);
    let free = s.solve().unwrap().analysis.dof;
    let length = s.add_reference(Rule::Distance { a: l.whole(), b: None, value: 1.0.into() }).unwrap();
    assert_eq!(s.measure(length).unwrap(), 50.0);
    assert_eq!(s.constraint(length).unwrap().rule.value().unwrap().as_number(), Some(50.0), "measured as soon as it's added");
    assert_eq!(s.solve().unwrap().analysis.dof, free, "it takes no freedom away");

    // It follows the geometry through a solve and a drag.
    s.constrain(Rule::Coincident { a: l.start(), b: ORIGIN.whole() });
    let h = s.constrain(Rule::Horizontal { a: l.whole(), b: None });
    let span = s.constrain(Rule::HorizontalDistance { a: l.start(), b: l.end(), value: 25.0.into() });
    let solved = s.solve().unwrap();
    assert_eq!(solved.analysis.dof, 0);
    assert!((solved.sketch.reference_values()[0].1 - 25.0).abs() < 1e-9);
    assert_eq!(solved.sketch.constraint(length).unwrap().rule.value().unwrap().as_number().map(|v| (v - 25.0).abs() < 1e-9), Some(true));

    // A reference can't conflict with anything, unlike the same dimension driving.
    let mut driving = s.clone();
    driving.set_reference(length, false).unwrap();
    driving.set_value(length, 99.0).unwrap();
    assert_eq!(driving.solve().unwrap_err().kind, ErrorKind::Conflict);
    assert!(s.solve().is_ok());

    // Its value can't be set, and only dimensions can be references.
    assert!(s.set_value(length, 3.0).is_err());
    assert!(s.add_reference(Rule::Vertical { a: l.whole(), b: None }).is_err());
    assert!(s.set_reference(h, true).is_err());
    let _ = span;
}

#[test]
fn a_reference_dimension_is_saved_and_needs_no_value_in_a_file() {
    let mut s = Sketch::new();
    let l = s.line([0.0, 0.0], [3.0, 4.0]);
    let c = s.add_reference(Rule::Distance { a: l.whole(), b: None, value: 0.0.into() }).unwrap();
    let back = Sketch::load_text(&s.save_text()).unwrap();
    assert_eq!(back, s);
    assert!(back.constraint(c).unwrap().reference);
    // Hand-written, with no value.
    let text = r#"{"format":"coreshape-sketch","version":1,"entities":[{"id":1,"type":"line","start":[0,0],"end":[3,4]}],
        "constraints":[{"id":1,"type":"distance","a":"1","reference":true}]}"#;
    let k = Sketch::load_text(text).unwrap();
    assert_eq!(k.measure(ConstraintId(1)).unwrap(), 5.0);
    assert_eq!(k.reference_values(), vec![(ConstraintId(1), 5.0)]);
    // A reference must be a dimension, and a driving dimension still needs a value.
    assert!(Sketch::load_text(&text.replace("distance", "horizontal")).is_err());
    assert!(Sketch::load_text(&text.replace(r#","reference":true"#, "")).is_err());
}

#[test]
fn measuring_every_kind_of_dimension() {
    let mut s = Sketch::new();
    let l = s.line([0.0, 0.0], [10.0, 0.0]);
    let m = s.line([0.0, 5.0], [0.0, 15.0]);
    let c = s.circle([20.0, 0.0], 4.0);
    let d = s.circle([40.0, 0.0], 6.0);
    let a = s.arc([0.0, 0.0], 2.0, 0.0, 90.0);
    let p = s.point(20.0, 10.0);
    let near = |rule: Rule, want: f64| {
        let got = s.measure_rule(&rule).unwrap();
        assert!((got - want).abs() < 1e-9, "{rule:?}: {got} vs {want}");
    };
    let zero = || 0.0.into();
    near(Rule::Distance { a: l.whole(), b: None, value: zero() }, 10.0);
    near(Rule::Distance { a: l.start(), b: Some(l.end()), value: zero() }, 10.0);
    near(Rule::Distance { a: p.whole(), b: Some(l.whole()), value: zero() }, 10.0);
    near(Rule::Distance { a: p.whole(), b: Some(c.whole()), value: zero() }, 6.0);
    near(Rule::Distance { a: c.center(), b: Some(d.center()), value: zero() }, 20.0);
    near(Rule::Distance { a: c.whole(), b: Some(d.whole()), value: zero() }, 10.0);
    near(Rule::Distance { a: l.whole(), b: Some(c.whole()), value: zero() }, 4.0);
    near(Rule::Distance { a: l.whole(), b: Some(X_AXIS.whole()), value: zero() }, 0.0);
    near(Rule::HorizontalDistance { a: l.start(), b: c.center(), value: zero() }, 20.0);
    near(Rule::VerticalDistance { a: p.whole(), b: l.end(), value: zero() }, -10.0);
    near(Rule::Radius { of: d, value: zero() }, 6.0);
    near(Rule::Diameter { of: a, value: zero() }, 4.0);
    near(Rule::Angle { a: l, b: Some(m), value: zero() }, 90.0);
    near(Rule::Angle { a: m, b: Some(l), value: zero() }, -90.0);
    near(Rule::Angle { a: m, b: None, value: zero() }, 90.0);
    near(Rule::Angle { a, b: None, value: zero() }, 90.0);
    assert!(s.measure_rule(&Rule::Vertical { a: l.whole(), b: None }).is_err());
    assert!(s.measure_rule(&Rule::Radius { of: l, value: zero() }).is_err());
}
