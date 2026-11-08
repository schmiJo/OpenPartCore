//! Milestone 6: the feature history rebuilds correctly and keeps its references.

use coreshape::history::*;
use coreshape::*;

fn corner_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

fn vol(o: &Outcome) -> f64 {
    mass_properties(o.body.as_ref().expect("a body")).volume
}

fn rebuilt(o: &Outcome) -> Vec<&str> {
    o.steps
        .iter()
        .filter(|s| s.state == StepState::Rebuilt)
        .map(|s| s.name.as_str())
        .collect()
}

/// A plate: w, d, t, fillet radius on its long top edge, hole radius.
fn plate_history() -> (History, FeatureId) {
    let mut h = History::new();
    let s = h.sketch(
        |_| Frame::WORLD,
        |p| Ok(vec![Profile::new(Wire::rect(0.0, 0.0, p[0], p[1]))]),
    );
    let e = h.extrude(s, |_| 0.0, |p| p[2], Mode::Add);
    h.fillet(EdgeQuery::Between(e.end_cap(), e.side(0)), |p| p[3]);
    let s2 = h.sketch(
        |p| Frame::translation(v3(p[0] / 2.0, p[1] / 2.0, 0.0)),
        |p| Ok(vec![Profile::new(Wire::circle(v2(0.0, 0.0), p[4]))]),
    );
    h.extrude(s2, |_| -1.0, |p| p[2] + 2.0, Mode::Remove);
    (h, e)
}

fn plate_volume(p: &[f64]) -> f64 {
    p[0] * p[1] * p[2] - corner_area(p[3]) * p[0] - PI * p[4] * p[4] * p[2]
}

#[test]
fn history_builds_the_expected_solid() {
    let (mut h, _) = plate_history();
    let p = [40.0, 20.0, 6.0, 2.0, 3.0];
    let o = h.rebuild(&p, None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    assert!((vol(&o) - plate_volume(&p)).abs() < 1e-8);
    let names: Vec<&str> = o.steps.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["Sketch 1", "Extrude 1", "Fillet 1", "Sketch 2", "Extrude 2"]);
    validate(o.body.as_ref().unwrap()).unwrap();
}

#[test]
fn changing_any_dimension_rebuilds_correctly() {
    let (mut h, _) = plate_history();
    let base = [40.0, 20.0, 6.0, 2.0, 3.0];
    h.rebuild(&base, None);
    for changed in [
        [55.0, 20.0, 6.0, 2.0, 3.0],
        [40.0, 33.0, 6.0, 2.0, 3.0],
        [40.0, 20.0, 9.0, 2.0, 3.0],
        [40.0, 20.0, 6.0, 4.5, 3.0],
        [40.0, 20.0, 6.0, 2.0, 5.5],
    ] {
        let o = h.rebuild(&changed, None);
        assert!(o.failed.is_none(), "{changed:?}: {:?}", o.steps);
        assert!((vol(&o) - plate_volume(&changed)).abs() < 1e-8, "{changed:?}");
        h.rebuild(&base, None);
    }
}

#[test]
fn unchanged_steps_are_reused() {
    let (mut h, _) = plate_history();
    let mut p = [40.0, 20.0, 6.0, 2.0, 3.0];
    let first = h.rebuild(&p, None);
    assert_eq!(rebuilt(&first).len(), 5);
    // Nothing changed.
    assert!(rebuilt(&h.rebuild(&p, None)).is_empty());
    // Only the hole radius: its sketch and everything after it.
    p[4] = 4.0;
    assert_eq!(rebuilt(&h.rebuild(&p, None)), ["Sketch 2", "Extrude 2"]);
    // The fillet radius: the fillet and everything that depends on the body. The
    // hole's sketch is unaffected and stays cached.
    p[3] = 3.0;
    assert_eq!(rebuilt(&h.rebuild(&p, None)), ["Fillet 1", "Extrude 2"]);
    // The first dimension affects everything.
    p[0] = 50.0;
    assert_eq!(rebuilt(&h.rebuild(&p, None)).len(), 5);
    let o = h.rebuild(&p, None);
    assert!((vol(&o) - plate_volume(&p)).abs() < 1e-8);
}

#[test]
fn rollback_shows_earlier_states_without_recomputing() {
    let (mut h, _) = plate_history();
    let p = [40.0, 20.0, 6.0, 2.0, 3.0];
    h.rebuild(&p, None);
    let at_extrude = h.rebuild(&p, Some(1));
    assert!((vol(&at_extrude) - 40.0 * 20.0 * 6.0).abs() < 1e-9);
    assert!(rebuilt(&at_extrude).is_empty());
    assert_eq!(at_extrude.steps[4].state, StepState::RolledBack);
    let at_fillet = h.rebuild(&p, Some(2));
    assert!((vol(&at_fillet) - (4800.0 - corner_area(2.0) * 40.0)).abs() < 1e-9);
    let end = h.rebuild(&p, None);
    assert!((vol(&end) - plate_volume(&p)).abs() < 1e-8);
    // A rollback before a change, then a roll forward, still ends up right.
    let q = [40.0, 20.0, 6.0, 2.0, 4.0];
    h.rebuild(&q, Some(1));
    assert!((vol(&h.rebuild(&q, None)) - plate_volume(&q)).abs() < 1e-8);
}

#[test]
fn a_failing_feature_keeps_the_last_good_body_and_recovers() {
    let (mut h, _) = plate_history();
    let good = [40.0, 20.0, 6.0, 2.0, 3.0];
    h.rebuild(&good, None);
    // A fillet bigger than the 6 mm face it runs along.
    let bad = [40.0, 20.0, 6.0, 7.0, 3.0];
    let o = h.rebuild(&bad, None);
    assert_eq!(o.failed, Some(2));
    assert!(matches!(o.steps[2].state, StepState::Failed(_)));
    assert_eq!(o.steps[4].state, StepState::Blocked);
    assert!((vol(&o) - 4800.0).abs() < 1e-9, "should show the body before the failure");
    // Back to a good value: everything works again.
    let o = h.rebuild(&good, None);
    assert!(o.failed.is_none());
    assert!((vol(&o) - plate_volume(&good)).abs() < 1e-8);
}

#[test]
fn names_survive_a_feature_that_splits_the_edge() {
    // A slot cuts across the front edge before the fillet is applied. The edge between
    // the same two named faces is now two pieces, and the fillet must find both.
    let mut h = History::new();
    let s = h.sketch(|_| Frame::WORLD, |_| Ok(vec![Profile::new(Wire::rect(0.0, 0.0, 30.0, 20.0))]));
    let e = h.extrude(s, |_| 0.0, |p| p[0], Mode::Add);
    let slot = h.sketch(
        |p| Frame::translation(v3(0.0, 0.0, p[0] - 2.0)),
        |_| Ok(vec![Profile::new(Wire::rect(12.0, -1.0, 6.0, 8.0))]),
    );
    h.extrude(slot, |_| 0.0, |_| 5.0, Mode::Remove);
    h.fillet(EdgeQuery::Between(e.end_cap(), e.side(0)), |_| 1.0);
    let o = h.rebuild(&[6.0], None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let expected = 30.0 * 20.0 * 6.0 - 6.0 * 7.0 * 2.0 - corner_area(1.0) * 24.0;
    assert!((vol(&o) - expected).abs() < 1e-8, "got {} want {expected}", vol(&o));
    // Thicker plate: same names, same result shape.
    let o = h.rebuild(&[9.0], None);
    assert!(o.failed.is_none(), "{:?}", o.steps);
    let expected = 30.0 * 20.0 * 9.0 - 6.0 * 7.0 * 2.0 - corner_area(1.0) * 24.0;
    assert!((vol(&o) - expected).abs() < 1e-8);
}

#[test]
fn a_reference_that_stops_existing_is_reported() {
    // Cutting all the way across removes the edge the fillet refers to.
    let mut h = History::new();
    let s = h.sketch(|_| Frame::WORLD, |_| Ok(vec![Profile::new(Wire::rect(0.0, 0.0, 10.0, 10.0))]));
    let e = h.extrude(s, |_| 0.0, |_| 5.0, Mode::Add);
    let cut = h.sketch(
        |_| Frame::WORLD,
        |p| Ok(vec![Profile::new(Wire::rect(-1.0, -1.0, 12.0, p[0]))]),
    );
    h.extrude(cut, |_| -1.0, |_| 7.0, Mode::Remove);
    h.fillet(EdgeQuery::Between(e.end_cap(), e.side(0)), |_| 1.0);
    let o = h.rebuild(&[3.0], None);
    assert_eq!(o.failed, Some(4));
    match &o.steps[4].state {
        StepState::Failed(why) => assert!(why.contains("no longer exist"), "{why}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn sketches_with_several_regions() {
    let mut h = History::new();
    let s = h.sketch(|_| Frame::WORLD, |_| Ok(vec![Profile::new(Wire::rect_centered(50.0, 50.0))]));
    h.extrude(s, |_| 0.0, |_| 5.0, Mode::Add);
    let holes = h.sketch(
        |_| Frame::WORLD,
        |p| {
            Ok((0..p[0] as usize)
                .map(|i| {
                    let a = TAU * i as f64 / p[0];
                    Profile::new(Wire::circle(v2(15.0 * a.cos(), 15.0 * a.sin()), 3.0))
                })
                .collect())
        },
    );
    h.extrude(holes, |_| -1.0, |_| 7.0, Mode::Remove);
    for n in [3.0, 5.0, 8.0] {
        let o = h.rebuild(&[n], None);
        assert!(o.failed.is_none(), "{:?}", o.steps);
        assert!((vol(&o) - (12500.0 - n * PI * 9.0 * 5.0)).abs() < 1e-8);
        assert_eq!(validate(o.body.as_ref().unwrap()).unwrap().genus, vec![n as usize]);
    }
}
