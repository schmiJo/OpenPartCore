//! Thin walls beside large circles. An arc's chord strays by more than such a wall is thick, so nothing that decides whether
//! the wall is there can go by chords. The answers come from a formula, and a hole that really does cross has to stay refused.

use coreshape::*;
use std::f64::consts::{PI, TAU};

/// A regular polygon around the origin, with its corners on this radius.
fn polygon(n: usize, radius: f64) -> Wire {
    let at = |k: usize| {
        let a = TAU * k as f64 / n as f64;
        v2(radius * a.cos(), radius * a.sin())
    };
    let mut path = PathBuilder::new(at(0));
    for k in 1..n {
        path = path.line_to(at(k));
    }
    path.close()
}

fn polygon_area(n: usize, radius: f64) -> f64 {
    0.5 * n as f64 * radius * radius * (TAU / n as f64).sin()
}

#[test]
fn a_hole_just_inside_a_large_circle_is_a_valid_profile() {
    for wall in [0.5, 0.3, 0.2, 0.1, 0.01, 0.001] {
        let profile = Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)).with_hole(polygon(128, 100.0 - wall));
        profile.validate().unwrap_or_else(|why| panic!("wall {wall}: {why}"));
        let body = extrude(&profile, &Frame::WORLD, 2.0, 1).unwrap_or_else(|why| panic!("wall {wall}: {why}"));
        let want = (PI * 100.0 * 100.0 - polygon_area(128, 100.0 - wall)) * 2.0;
        let got = mass_properties(&body).volume;
        assert!((got - want).abs() < 1e-7 * want, "wall {wall}: {got} and {want}");
    }
}

#[test]
fn a_hole_that_reaches_past_a_large_circle_is_still_refused() {
    // Its corners are inside the circle's chords, by the sag, but a corner past the true circle crosses it.
    let profile = Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)).with_hole(polygon(128, 100.05));
    assert!(profile.validate().is_err());
    // Only one corner sticking out, by very little, and off the chords' corners.
    let a = |r: f64, k: usize| {
        let t = TAU * k as f64 / 128.0 + 0.003;
        v2(r * t.cos(), r * t.sin())
    };
    let mut path = PathBuilder::new(a(99.5, 0));
    for k in 1..128 {
        path = path.line_to(a(if k == 40 { 100.001 } else { 99.5 }, k));
    }
    let profile = Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)).with_hole(path.close());
    let why = profile.validate().unwrap_err().to_string();
    assert!(why.contains("crosses the outer wire"), "{why}");
}

#[test]
fn a_hole_past_the_chord_but_inside_the_arc_is_inside() {
    // A small hole in the bulge of an arc, right up against it, where a chord would have it outside the wire.
    let a = 24f64.to_radians() / 2.0;
    let at = v2(100.0 * a.cos(), 100.0 * a.sin()) * (99.8 / 100.0);
    let profile = Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)).with_hole(Wire::circle(at, 0.1));
    profile.validate().unwrap();
    let body = extrude(&profile, &Frame::WORLD, 1.0, 1).unwrap();
    let want = PI * (100.0 * 100.0 - 0.1 * 0.1);
    assert!((mass_properties(&body).volume - want).abs() < 1e-7 * want);
    // And just outside the arc, it's outside, however near.
    let at = v2(100.0 * a.cos(), 100.0 * a.sin()) * (100.2 / 100.0);
    let profile = Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)).with_hole(Wire::circle(at, 0.1));
    assert!(profile.validate().is_err());
}

#[test]
fn holes_side_by_side_in_a_thin_web_are_fine_and_crossing_ones_arent() {
    // Two large circles with a web of 0.05 between them.
    let web = |gap: f64| Profile::new(Wire::rect_centered(500.0, 300.0)).with_hole(Wire::circle(v2(-100.0 - gap / 2.0, 0.0), 100.0)).with_hole(Wire::circle(v2(100.0 + gap / 2.0, 0.0), 100.0));
    web(0.05).validate().unwrap();
    let why = web(-0.05).validate().unwrap_err().to_string();
    assert!(why.contains("cross"), "{why}");
}

#[test]
fn an_outer_wire_with_arcs_that_nearly_touch_is_not_crossing_itself() {
    // A thin crescent: an outer arc and an inner one, 0.05 apart at the middle. Chords would cross. The true arcs don't.
    let (r, s) = (100.0, 100.5);
    let half = 20f64.to_radians();
    let (x, y) = (r * half.sin(), r * half.cos());
    // The inner arc's center is below, so that it passes 0.05 under the outer one's top.
    let cy = r - 0.05 - s;
    let wire = PathBuilder::new(v2(-x, y)).arc_to(v2(x, y), v2(0.0, cy), false).arc_to(v2(-x, y), v2(0.0, 0.0), true).close();
    let profile = Profile::new(wire);
    profile.validate().unwrap_or_else(|why| panic!("{why}"));
}

#[test]
fn a_wire_whose_arcs_really_cross_is_refused() {
    // A figure of eight, from two arcs.
    let wire = PathBuilder::new(v2(-10.0, 0.0)).arc_to(v2(10.0, 0.0), v2(0.0, -5.0), false).line_to(v2(10.0, 8.0)).arc_to(v2(-10.0, 8.0), v2(0.0, 13.0), true).close();
    // Arcs bulging into each other across the middle.
    let crossing = PathBuilder::new(v2(-10.0, 0.0)).arc_to(v2(10.0, 0.0), v2(0.0, -30.0), false).line_to(v2(10.0, 1.0)).arc_to(v2(-10.0, 1.0), v2(0.0, 31.0), false).close();
    Profile::new(wire).validate().unwrap();
    let why = Profile::new(crossing).validate().unwrap_err().to_string();
    assert!(why.contains("crosses itself"), "{why}");
}

#[test]
fn a_thin_wall_beside_a_large_circle_can_be_cut() {
    for wall in [0.5, 0.3, 0.2, 0.1] {
        let solid = extrude(&Profile::new(Wire::circle(v2(0.0, 0.0), 100.0)), &Frame::WORLD, 2.0, 1).unwrap();
        let tool = extrude(&Profile::new(polygon(128, 100.0 - wall)), &Frame::translation(v3(0.0, 0.0, -1.0)), 4.0, 2).unwrap();
        let cut = subtract(&solid, &tool).unwrap_or_else(|why| panic!("wall {wall}: {why}"));
        validate(&cut).unwrap();
        let want = (PI * 100.0 * 100.0 - polygon_area(128, 100.0 - wall)) * 2.0;
        let got = mass_properties(&cut).volume;
        assert!((got - want).abs() < 1e-7 * want, "wall {wall}: {got} and {want}");
    }
}

/// The volume two cylinders share, a bar of radius `s` along X from x = 0 whose axis is `d` from a post of radius `r` along Z, by
/// integrating across the bar: a line along the bar at height y is inside the post for as far as sqrt(r^2 - y^2).
fn shared_with_post(r: f64, s: f64, d: f64) -> f64 {
    let n = 200_000;
    let mut sum = 0.0;
    for k in 0..n {
        // Midpoints, which are good enough with this many, as both square roots go to zero smoothly in area terms.
        let y = d - s + 2.0 * s * (k as f64 + 0.5) / n as f64;
        sum += 2.0 * (s * s - (y - d) * (y - d)).max(0.0).sqrt() * (r * r - y * y).max(0.0).sqrt();
    }
    sum * 2.0 * s / n as f64
}

fn post_and_bar(r: f64, s: f64) -> (Body, Body) {
    let post = cylinder(&Frame::WORLD, r, 120.0, 1).unwrap();
    // The bar's axis is r - s from the post's, so that its wall just touches the post's from within.
    let bar = cylinder(&Frame::new(v3(0.0, r - s, 90.0), Vec3::X, Vec3::Y), s, 100.0, 2).unwrap();
    (post, bar)
}

#[test]
fn a_cylinder_tangent_inside_another_joins_it() {
    let (r, s) = (50.0, 15.0);
    let (post, bar) = post_and_bar(r, s);
    let (vp, vb) = (PI * r * r * 120.0, PI * s * s * 100.0);
    let shared = shared_with_post(r, s, r - s);
    let joined = union(&post, &bar).unwrap();
    validate(&joined).unwrap();
    let want = vp + vb - shared;
    let got = mass_properties(&joined).volume;
    assert!((got - want).abs() < 1e-3, "{got} and {want}");
    // The same part from the other two.
    let common = intersect(&post, &bar).unwrap();
    assert!((mass_properties(&common).volume - shared).abs() < 1e-3, "{} and {shared}", mass_properties(&common).volume);
    let cut = subtract(&post, &bar).unwrap();
    assert!((mass_properties(&cut).volume - (vp - shared)).abs() < 1e-3, "{}", mass_properties(&cut).volume);
    let other_cut = subtract(&bar, &post).unwrap();
    assert!((mass_properties(&other_cut).volume - (vb - shared)).abs() < 1e-3, "{}", mass_properties(&other_cut).volume);
}

#[test]
fn a_cylinder_a_little_off_tangent_is_still_right_or_refused() {
    let (r, s) = (50.0, 15.0);
    for off in [1e-3, 1e-2, 0.1, -1e-3, -1e-2, -0.1] {
        let post = cylinder(&Frame::WORLD, r, 120.0, 1).unwrap();
        let bar = cylinder(&Frame::new(v3(0.0, r - s + off, 90.0), Vec3::X, Vec3::Y), s, 100.0, 2).unwrap();
        if let Ok(joined) = union(&post, &bar) {
            let want = PI * r * r * 120.0 + PI * s * s * 100.0 - shared_with_post(r, s, r - s + off);
            let got = mass_properties(&joined).volume;
            assert!((got - want).abs() < 1e-2, "off by {off}: {got} and {want}");
        }
    }
}

#[test]
fn cylinders_that_only_seem_tangent_arent_treated_as_it() {
    // Their axes are the right distance apart, but it's the outside of the smaller that touches the outside of the larger, so
    // they share nothing but a point.
    let post = cylinder(&Frame::WORLD, 50.0, 120.0, 1).unwrap();
    let bar = cylinder(&Frame::new(v3(-50.0, 65.0, 90.0), Vec3::X, Vec3::Y), 15.0, 100.0, 2).unwrap();
    if let Ok(common) = intersect(&post, &bar) {
        assert!(mass_properties(&common).volume.abs() < 1e-6);
    }
}

#[test]
fn a_tilted_cylinder_tangent_inside_another_adds_up() {
    // Not square to the post. The two Booleans have to account for all of both, and there's no formula needed for that.
    let (r, s) = (50.0, 12.0);
    let z = v3(1.0, 0.0, 0.4).normalized();
    let post = cylinder(&Frame::translation(v3(0.0, 0.0, -100.0)), r, 300.0, 1).unwrap();
    let bar = cylinder(&Frame::new(v3(0.0, r - s, 30.0) - z * 10.0, z, Vec3::Y), s, 90.0, 2).unwrap();
    let (a, b) = (mass_properties(&post).volume, mass_properties(&bar).volume);
    let joined = union(&post, &bar).unwrap();
    let common = intersect(&post, &bar).unwrap();
    let cut = subtract(&bar, &post).unwrap();
    validate(&joined).unwrap();
    validate(&common).unwrap();
    validate(&cut).unwrap();
    let (j, c, d) = (mass_properties(&joined).volume, mass_properties(&common).volume, mass_properties(&cut).volume);
    assert!((j + c - a - b).abs() < 1e-3, "{j} + {c} and {a} + {b}");
    assert!((d + c - b).abs() < 1e-3, "{d} + {c} and {b}");
    assert!(c > 0.0 && d > 0.0);
}

/// A torus's volume on the far side of a plane along its axis, at distance `d`, by rings: the part of a ring of radius rho on
/// that side takes the angle whose cosine is d / rho.
fn torus_beyond(major: f64, minor: f64, d: f64) -> f64 {
    let n = 400_000;
    let mut sum = 0.0;
    for k in 0..n {
        let rho = major - minor + 2.0 * minor * (k as f64 + 0.5) / n as f64;
        let height = 2.0 * (minor * minor - (rho - major) * (rho - major)).max(0.0).sqrt();
        if rho > d {
            sum += height * 2.0 * (d / rho).acos() * rho;
        }
    }
    sum * 2.0 * minor / n as f64
}

#[test]
fn a_plane_touching_a_torus_at_its_waist_cuts_it() {
    let (major, minor) = (30.0, 10.0);
    let ring = torus(&Frame::WORLD, major, minor, 1).unwrap();
    // A box whose side is the plane x = 20, which touches the inside of the waist.
    let block = box_solid(&Frame::translation(v3(major - minor, -60.0, -20.0)), 60.0, 120.0, 40.0, 2).unwrap();
    let beyond = torus_beyond(major, minor, major - minor);
    let whole = 2.0 * PI * PI * major * minor * minor;
    let kept = intersect(&ring, &block).unwrap();
    validate(&kept).unwrap();
    let got = mass_properties(&kept).volume;
    assert!((got - beyond).abs() < 1e-3, "{got} and {beyond}");
    let rest = subtract(&ring, &block).unwrap();
    validate(&rest).unwrap();
    let got = mass_properties(&rest).volume;
    assert!((got - (whole - beyond)).abs() < 1e-3, "{got} and {}", whole - beyond);
    let joined = union(&ring, &block).unwrap();
    validate(&joined).unwrap();
    let want = mass_properties(&block).volume + whole - beyond;
    let got = mass_properties(&joined).volume;
    assert!((got - want).abs() < 1e-3, "{got} and {want}");
}

#[test]
fn a_plane_on_the_other_side_of_a_torus_waist_too() {
    let (major, minor) = (30.0, 10.0);
    let ring = torus(&Frame::WORLD, major, minor, 1).unwrap();
    let block = box_solid(&Frame::translation(v3(-60.0 - (major - minor), -60.0, -20.0)), 60.0, 120.0, 40.0, 2).unwrap();
    let kept = intersect(&ring, &block).unwrap();
    let got = mass_properties(&kept).volume;
    let beyond = torus_beyond(major, minor, major - minor);
    assert!((got - beyond).abs() < 1e-3, "{got} and {beyond}");
}

#[test]
fn a_tangent_cylinder_on_the_other_side_joins_it_too() {
    let (r, s) = (50.0, 15.0);
    let post = cylinder(&Frame::WORLD, r, 120.0, 1).unwrap();
    let bar = cylinder(&Frame::new(v3(0.0, s - r, 90.0), Vec3::X, Vec3::Y), s, 100.0, 2).unwrap();
    let joined = union(&post, &bar).unwrap();
    validate(&joined).unwrap();
    let want = PI * r * r * 120.0 + PI * s * s * 100.0 - shared_with_post(r, s, r - s);
    let got = mass_properties(&joined).volume;
    assert!((got - want).abs() < 1e-3, "{got} and {want}");
}

#[test]
fn cylinders_that_arent_tangent_go_on_as_they_did() {
    // Well inside the larger one's reach, by nothing like the difference of their radii.
    let (r, s, d) = (50.0, 15.0, 20.0);
    let post = cylinder(&Frame::WORLD, r, 120.0, 1).unwrap();
    let bar = cylinder(&Frame::new(v3(0.0, d, 90.0), Vec3::X, Vec3::Y), s, 100.0, 2).unwrap();
    let joined = union(&post, &bar).unwrap();
    let want = PI * r * r * 120.0 + PI * s * s * 100.0 - shared_with_post(r, s, d);
    let got = mass_properties(&joined).volume;
    assert!((got - want).abs() < 1e-3, "{got} and {want}");
}

#[test]
fn a_tangent_cylinder_that_starts_past_where_it_touches_is_fine() {
    // The bar begins at x = 20, so the place where the two surfaces touch isn't on it, and neither loop gets there.
    let (r, s, from) = (50.0, 15.0, 20.0);
    let post = cylinder(&Frame::WORLD, r, 120.0, 1).unwrap();
    let bar = cylinder(&Frame::new(v3(from, r - s, 90.0), Vec3::X, Vec3::Y), s, 80.0, 2).unwrap();
    let joined = union(&post, &bar).unwrap();
    validate(&joined).unwrap();
    // Across the bar, a line along it is inside the post from its start to sqrt(r^2 - y^2), where that's past its start.
    let n = 200_000;
    let d = r - s;
    let mut shared = 0.0;
    for k in 0..n {
        let y = d - s + 2.0 * s * (k as f64 + 0.5) / n as f64;
        let reach = (r * r - y * y).max(0.0).sqrt() - from;
        if reach > 0.0 {
            shared += 2.0 * (s * s - (y - d) * (y - d)).max(0.0).sqrt() * reach;
        }
    }
    shared *= 2.0 * s / n as f64;
    let want = PI * r * r * 120.0 + PI * s * s * 80.0 - shared;
    let got = mass_properties(&joined).volume;
    assert!((got - want).abs() < 1e-3, "{got} and {want}");
}

#[test]
fn a_plane_on_a_torus_waist_can_touch_it_somewhere_off_the_face() {
    // The block's side is on the plane that touches the waist, at x = 0, but the block only starts at x = 5.
    let (major, minor) = (30.0, 10.0);
    let ring = torus(&Frame::WORLD, major, minor, 1).unwrap();
    let block = box_solid(&Frame::translation(v3(5.0, major - minor, -20.0)), 60.0, 60.0, 40.0, 2).unwrap();
    let (a, b) = (mass_properties(&ring).volume, mass_properties(&block).volume);
    let joined = union(&ring, &block).unwrap();
    let common = intersect(&ring, &block).unwrap();
    let cut = subtract(&ring, &block).unwrap();
    validate(&joined).unwrap();
    validate(&common).unwrap();
    validate(&cut).unwrap();
    let (j, c, d) = (mass_properties(&joined).volume, mass_properties(&common).volume, mass_properties(&cut).volume);
    assert!((j + c - a - b).abs() < 1e-3, "{j} + {c} and {a} + {b}");
    assert!((d + c - a).abs() < 1e-3, "{d} + {c} and {a}");
    assert!(c > 1.0);
}

#[test]
fn cylinders_a_hair_apart_come_out_right() {
    // A hair off the same axis, overlapping along part of it. What's left of one is a crescent as thin as that, which
    // the check on where a region's sample points are is what gets right.
    let (r, d) = (5.0, 1e-5);
    let a = cylinder(&Frame::WORLD, r, 10.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(d, 0.0, 2.0)), r, 10.0, 2).unwrap();
    let whole = PI * r * r * 10.0;
    // Two circles that far apart overlap by all but 2 r d.
    let shared = (PI * r * r - 2.0 * r * d) * 8.0;
    let joined = union(&a, &b).unwrap();
    validate(&joined).unwrap();
    let got = mass_properties(&joined).volume;
    assert!((got - (2.0 * whole - shared)).abs() < 1e-6, "{got} and {}", 2.0 * whole - shared);
}

#[test]
fn the_slivers_that_used_to_be_refused_come_out_right() {
    // A skin 1e-5 thick around a cylinder, and a crescent that thin beside one. OpenCASCADE does both, and so does this now.
    let skin = subtract(&cylinder(&Frame::WORLD, 5.00001, 10.0, 1).unwrap(), &cylinder(&Frame::translation(v3(0.0, 0.0, -1.0)), 5.0, 12.0, 2).unwrap()).unwrap();
    validate(&skin).unwrap();
    let want = PI * (5.00001f64 * 5.00001 - 25.0) * 10.0;
    let got = mass_properties(&skin).volume;
    assert!((got - want).abs() < 1e-8, "{got} and {want}");
    let crescent = subtract(&cylinder(&Frame::WORLD, 5.0, 10.0, 1).unwrap(), &cylinder(&Frame::translation(v3(1e-5, 0.0, -1.0)), 5.0, 12.0, 2).unwrap()).unwrap();
    validate(&crescent).unwrap();
    let got = mass_properties(&crescent).volume;
    assert!((got - 2.0 * 5.0 * 1e-5 * 10.0).abs() < 1e-8, "{got}");
}
