//! Sweeping along a chain of lines and arcs in space, given in any order and either way round. Volumes against Pappus: a
//! profile centred on the path, of area A, makes A times the path's length.

use coreshape::*;

fn circle(r: f64) -> Profile {
    Profile::new(Wire::circle(v2(0.0, 0.0), r))
}

/// Up 10, a quarter bend of radius 5 toward +x, a quarter bend of radius 3 toward +y, which is in another plane, and on 7.
fn path() -> Vec<PathCurve> {
    vec![
        PathCurve::Line { from: v3(0.0, 0.0, 0.0), to: v3(0.0, 0.0, 10.0) },
        PathCurve::Arc { center: v3(5.0, 0.0, 10.0), axis: v3(0.0, 1.0, 0.0), from: v3(0.0, 0.0, 10.0), angle: PI / 2.0 },
        PathCurve::Arc { center: v3(5.0, 3.0, 15.0), axis: v3(0.0, 0.0, 1.0), from: v3(5.0, 0.0, 15.0), angle: PI / 2.0 },
        PathCurve::Line { from: v3(8.0, 3.0, 15.0), to: v3(8.0, 10.0, 15.0) },
    ]
}

fn length() -> f64 {
    10.0 + 5.0 * PI / 2.0 + 3.0 * PI / 2.0 + 7.0
}

#[test]
fn a_path_bending_in_two_planes() {
    let pieces = path();
    for (i, p) in pieces.iter().enumerate() {
        if i + 1 < pieces.len() {
            assert!(p.end().dist(pieces[i + 1].start()) < 1e-12, "piece {i} doesn't reach the next");
        }
    }
    let body = sweep_along(&circle(1.0), &Frame::WORLD, &pieces, 1).unwrap();
    validate(&body).unwrap();
    assert!((mass_properties(&body).volume - PI * length()).abs() < 1e-6, "{}", mass_properties(&body).volume);
    // It ends where the path does, facing along it.
    let end = body.faces.iter().find(|f| f.tag.slot == coreshape::build::SLOT_END_CAP).unwrap();
    let Surface::Plane(cap) = &end.surface else { panic!() };
    assert!(cap.origin.dist(v3(8.0, 10.0, 15.0)) < 1e-9 || (cap.origin - v3(8.0, 10.0, 15.0)).dot(v3(0.0, 1.0, 0.0)).abs() < 1e-9);
    assert!(end.outward_normal(cap.origin).dist(v3(0.0, 1.0, 0.0)) < 1e-9);
}

#[test]
fn in_any_order_and_either_way_round() {
    let mut pieces = path();
    pieces.swap(0, 3);
    pieces.swap(1, 2);
    pieces[1] = pieces[1].reversed();
    pieces[3] = pieces[3].reversed();
    let body = sweep_along(&circle(1.0), &Frame::WORLD, &pieces, 1).unwrap();
    assert!((mass_properties(&body).volume - PI * length()).abs() < 1e-6);
    // And from the far end, with the profile there, square to the path.
    let far = Frame::new(v3(8.0, 10.0, 15.0), v3(0.0, 1.0, 0.0), v3(1.0, 0.0, 0.0));
    let body = sweep_along(&circle(1.0), &far, &pieces, 1).unwrap();
    assert!((mass_properties(&body).volume - PI * length()).abs() < 1e-6);
}

#[test]
fn a_reversed_arc_is_the_same_arc() {
    let arc = path()[2];
    let back = arc.reversed();
    assert!(back.start().dist(arc.end()) < 1e-12 && back.end().dist(arc.start()) < 1e-12);
    assert!(back.tangent(0.0).dist(-arc.tangent(1.0)) < 1e-12);
    assert!((back.length() - arc.length()).abs() < 1e-12);
}
