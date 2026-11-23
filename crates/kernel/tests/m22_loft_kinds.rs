//! Lofts to a point, round in a ring, along guide curves, and leaving their ends square or rounded, against shapes whose
//! answers are known exactly where there are any, and otherwise against their own meshes and the properties asked of them.

use coreshape::boolean::{subtract, union};
use coreshape::loft::{loft_through, LoftEnd, LoftOptions, Section};
use coreshape::profile::{PathBuilder, Profile, Wire};
use coreshape::surface::Surface;
use coreshape::*;
use std::f64::consts::PI;

fn at(z: f64) -> Frame {
    Frame { origin: v3(0.0, 0.0, z), ..Frame::WORLD }
}

fn circle(r: f64) -> Profile {
    Profile::new(Wire::circle(v2(0.0, 0.0), r))
}

fn rect(w: f64, h: f64) -> Profile {
    let (a, b) = (w / 2.0, h / 2.0);
    Profile::new(Wire::polygon(&[v2(-a, -b), v2(a, -b), v2(a, b), v2(-a, b)]))
}

/// Valid, watertight, and its mesh agrees with its integrated volume.
fn check(b: &Body) -> f64 {
    validate(b).unwrap();
    let props = mass_properties(b);
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    let by_mesh = mesh.volume();
    assert!((by_mesh - props.volume).abs() < 0.01 * props.volume.abs(), "mesh {by_mesh}, integrated {}", props.volume);
    assert!(props.volume_error < 1e-6 * props.volume.abs(), "{}", props.volume_error);
    props.volume
}

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "{what}: {a} against {b}");
}

fn outline<'a>(p: &'a Profile, f: Frame) -> Section<'a> {
    Section::Outline(p, f)
}

/// The closest any edge of a body comes to a point.
fn edge_distance(b: &Body, p: Vec3) -> f64 {
    let mut best = f64::INFINITY;
    for e in &b.edges {
        let n = 400;
        let at = |s: f64| e.curve.eval(e.t0 + (e.t1 - e.t0) * s).dist(p);
        let mut k = 0;
        for i in 0..=n {
            if at(i as f64 / n as f64) < at(k as f64 / n as f64) {
                k = i;
            }
        }
        let (mut lo, mut hi) = (((k as f64) - 1.0).max(0.0) / n as f64, ((k as f64) + 1.0).min(n as f64) / n as f64);
        for _ in 0..100 {
            let (a, c) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if at(a) < at(c) {
                hi = c;
            } else {
                lo = a;
            }
        }
        best = best.min(at((lo + hi) / 2.0));
    }
    best
}

#[test]
fn a_circle_to_a_point_is_a_cone() {
    let c = circle(5.0);
    let b = loft_through(&[outline(&c, at(0.0)), Section::Point(v3(0.0, 0.0, 12.0))], &LoftOptions::default(), 1).unwrap();
    close(check(&b), PI * 25.0 * 12.0 / 3.0, 1e-9, "cone");
    // One cap, and the sides meet at one vertex.
    assert_eq!(b.faces.iter().filter(|f| matches!(f.surface, Surface::Plane(_))).count(), 1);
    // Leaning over, it's still a cone of that height.
    let b = loft_through(&[outline(&c, at(0.0)), Section::Point(v3(3.0, -1.0, 10.0))], &LoftOptions::default(), 1).unwrap();
    close(check(&b), PI * 25.0 * 10.0 / 3.0, 1e-9, "leaning cone");
    // From the point instead.
    let b = loft_through(&[Section::Point(v3(0.0, 0.0, -8.0)), outline(&c, at(0.0))], &LoftOptions::default(), 1).unwrap();
    close(check(&b), PI * 25.0 * 8.0 / 3.0, 1e-9, "cone from its point");
}

#[test]
fn a_rectangle_to_a_point_is_a_pyramid() {
    let r = rect(10.0, 6.0);
    let b = loft_through(&[outline(&r, at(0.0)), Section::Point(v3(1.0, 2.0, 9.0))], &LoftOptions::default(), 1).unwrap();
    close(check(&b), 60.0 * 9.0 / 3.0, 1e-9, "pyramid");
    // Its four sides are planes.
    assert!(b.faces.iter().all(|f| matches!(f.surface, Surface::Plane(_))));
}

#[test]
fn a_point_through_a_circle_to_a_point() {
    // Each line across it is a parabola through the point, the circle and the point, so its radius at a share v of the
    // way is 4 r v (1 - v) and its height goes evenly: the volume is 16 pi r^2 h / 30.
    let c = circle(6.0);
    let b = loft_through(&[Section::Point(Vec3::ZERO), outline(&c, at(5.0)), Section::Point(v3(0.0, 0.0, 10.0))], &LoftOptions::default(), 1).unwrap();
    close(check(&b), 16.0 * PI * 36.0 * 10.0 / 30.0, 1e-9, "lens");
    assert!(b.faces.iter().all(|f| !matches!(f.surface, Surface::Plane(_))));
}

#[test]
fn a_point_can_be_rounded_over() {
    let c = circle(10.0);
    let options = LoftOptions { end: LoftEnd::Round(1.0), ..Default::default() };
    let b = loft_through(&[outline(&c, at(0.0)), Section::Point(v3(0.0, 0.0, 10.0))], &options, 1).unwrap();
    let v = check(&b);
    // More than the cone, less than the cylinder.
    assert!(v > PI * 100.0 * 10.0 / 3.0 * 1.2 && v < PI * 100.0 * 10.0, "{v}");
    // Flat at the top: just below it, the side is nearly level, where a cone's falls one for one.
    for f in &b.faces {
        let Surface::Free { patch, .. } = &f.surface else { continue };
        let ((u0, u1), (_, v1)) = patch.domain();
        for i in 0..5 {
            let u = u0 + (u1 - u0) * (i as f64 + 0.5) / 5.0;
            let q = patch.eval(u, v1 - 1e-3);
            let drop = 10.0 - q.z;
            let out = (q.x * q.x + q.y * q.y).sqrt();
            assert!(out > 0.0 && drop < 0.01 * out, "near the top, {drop} down at {out} out");
        }
    }
    // Rounded at a point that isn't an end, or a point made square, are refused.
    let wrong = LoftOptions { end: LoftEnd::Normal(1.0), ..Default::default() };
    assert!(loft_through(&[outline(&c, at(0.0)), Section::Point(v3(0.0, 0.0, 10.0))], &wrong, 1).is_err());
    let wrong = LoftOptions { start: LoftEnd::Round(1.0), ..Default::default() };
    assert!(loft_through(&[outline(&c, at(0.0)), Section::Point(v3(0.0, 0.0, 10.0))], &wrong, 1).is_err());
}

#[test]
fn square_ends_on_a_straight_loft_change_nothing() {
    let c = circle(5.0);
    for m in [1.0, 2.5] {
        let options = LoftOptions { start: LoftEnd::Normal(m), end: LoftEnd::Normal(m), ..Default::default() };
        let b = loft_through(&[outline(&c, at(0.0)), outline(&c, at(20.0))], &options, 1).unwrap();
        close(check(&b), PI * 25.0 * 20.0, 1e-9, "cylinder");
    }
}

#[test]
fn square_ends_leave_square_on_a_loft_that_moves_over() {
    // Each line across it is the same curve, so it's the circle moved along that curve, and every level cut is the
    // circle: the volume is its area times the height, whatever the curve.
    let c = circle(5.0);
    let options = LoftOptions { start: LoftEnd::Normal(1.0), end: LoftEnd::Normal(1.0), ..Default::default() };
    let top = Frame { origin: v3(8.0, 0.0, 20.0), ..Frame::WORLD };
    let b = loft_through(&[outline(&c, at(0.0)), outline(&c, top)], &options, 1).unwrap();
    close(check(&b), PI * 25.0 * 20.0, 1e-9, "moved over");
    // At both ends, the sides go straight up.
    for f in &b.faces {
        let Surface::Free { patch, .. } = &f.surface else { continue };
        let ((u0, u1), (v0, v1)) = patch.domain();
        for i in 0..5 {
            let u = u0 + (u1 - u0) * (i as f64 + 0.5) / 5.0;
            for v in [v0, v1] {
                let (_, _, dv) = patch.derivs(u, v);
                let d = dv.normalized();
                assert!(d.z.abs() > 1.0 - 1e-12, "{d:?} at v = {v}");
            }
        }
    }
    // Without them, they don't.
    let plain = loft_through(&[outline(&c, at(0.0)), outline(&c, top)], &LoftOptions::default(), 1).unwrap();
    close(check(&plain), PI * 25.0 * 20.0, 1e-9, "moved over, plain");
}

/// Sections round a circle of radius `big`, each a circle of radius `r` square to it.
fn ring(count: usize, big: f64, r: f64) -> (Profile, Vec<Frame>) {
    let frames = (0..count)
        .map(|k| {
            let a = 2.0 * PI * k as f64 / count as f64;
            let (c, s) = (a.cos(), a.sin());
            Frame::new(v3(big * c, big * s, 0.0), v3(-s, c, 0.0), v3(c, s, 0.0))
        })
        .collect();
    (circle(r), frames)
}

#[test]
fn a_closed_loft_is_a_ring() {
    let torus = |big: f64, r: f64| 2.0 * PI * PI * big * r * r;
    let mut last_error = f64::INFINITY;
    for count in [4, 8, 16] {
        let (c, frames) = ring(count, 10.0, 2.0);
        let sections: Vec<Section> = frames.iter().map(|f| outline(&c, *f)).collect();
        let b = loft_through(&sections, &LoftOptions { closed: true, ..Default::default() }, 1).unwrap();
        let v = check(&b);
        // No caps, and each piece of the outline in two bands.
        assert!(b.faces.iter().all(|f| !matches!(f.surface, Surface::Plane(_))));
        let error = (v - torus(10.0, 2.0)).abs() / torus(10.0, 2.0);
        // A cubic through more sections comes closer to the true ring.
        assert!(error < last_error && error < 0.03, "{count} sections: {v} against {}, {error}", torus(10.0, 2.0));
        last_error = error;
    }
    assert!(last_error < 2e-4, "{last_error}");
    // Squares round a ring make one too, and it can be cut.
    let (_, frames) = ring(6, 12.0, 1.0);
    let s = rect(3.0, 2.0);
    let sections: Vec<Section> = frames.iter().map(|f| outline(&s, *f)).collect();
    let b = loft_through(&sections, &LoftOptions { closed: true, ..Default::default() }, 1).unwrap();
    let v = check(&b);
    let cut = subtract(&b, &box_solid(&Frame { origin: v3(0.0, -20.0, -5.0), ..Frame::WORLD }, 20.0, 40.0, 10.0, 2).unwrap()).unwrap();
    let half = check(&cut);
    close(half, v / 2.0, 2e-3, "half the ring");
    // Too few sections, a point, or ends to set are refused.
    let two: Vec<Section> = frames[..2].iter().map(|f| outline(&s, *f)).collect();
    assert!(loft_through(&two, &LoftOptions { closed: true, ..Default::default() }, 1).is_err());
}

/// An open chain on the plane square to y at `y`, its local x along x and local y along z.
fn guide_frame(y: f64) -> Frame {
    Frame::new(v3(0.0, y, 0.0), v3(0.0, -1.0, 0.0), Vec3::X)
}

#[test]
fn guides_bend_a_loft_and_keep_its_sections() {
    let r = rect(20.0, 10.0);
    // Out from the corners at x = 10 to 16 halfway up, and back.
    let bend = PathBuilder::new(v2(10.0, 0.0)).line_to(v2(16.0, 20.0)).line_to(v2(10.0, 40.0)).open();
    let (front, back) = (guide_frame(5.0), guide_frame(-5.0));
    assert!(front.point(16.0, 20.0, 0.0).dist(v3(16.0, 5.0, 20.0)) < 1e-12);
    let sections = [outline(&r, at(0.0)), outline(&r, at(40.0))];
    // Both right-hand corners bent alike move every level the same way, so every level is still the rectangle: the
    // volume is unchanged.
    let both = LoftOptions { guides: vec![(&bend, front), (&bend, back)], ..Default::default() };
    let b = loft_through(&sections, &both, 1).unwrap();
    close(check(&b), 200.0 * 40.0, 1e-9, "two guides");
    for p in [v3(16.0, 5.0, 20.0), v3(16.0, -5.0, 20.0), v3(13.0, 5.0, 10.0), v3(13.0, -5.0, 30.0)] {
        assert!(edge_distance(&b, p) < 1e-9, "{p:?}: {}", edge_distance(&b, p));
    }
    // As far out as the guides, and the other side with them, halfway up, by a fine mesh's points.
    let mesh = tessellate(&b, &TessParams { chord: 1e-4, angle: 0.1 }).unwrap();
    let middle = mesh.positions.iter().filter(|p| (p.z - 20.0).abs() < 0.05);
    let (lo, hi) = middle.fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), p| (a.min(p.x), b.max(p.x)));
    // Within the band, a guide that turns at 20 is up to 0.05 * 6 / 20 lower.
    close(hi, 16.0, 1e-3, "as far out as the guides");
    close(lo, -4.0, 5e-3, "and the other side with them");
    // One guide alone does the same.
    let one = LoftOptions { guides: vec![(&bend, front)], ..Default::default() };
    close(check(&loft_through(&sections, &one, 1).unwrap()), 8000.0, 1e-9, "one guide");
    // Drawn the other way round, the same.
    let back_down = PathBuilder::new(v2(10.0, 40.0)).line_to(v2(16.0, 20.0)).line_to(v2(10.0, 0.0)).open();
    let reversed = LoftOptions { guides: vec![(&back_down, front)], ..Default::default() };
    close(check(&loft_through(&sections, &reversed, 1).unwrap()), 8000.0, 1e-9, "guide drawn downwards");
    // A guide on one corner and a straight one on the next. Every point moves along x by the bend times its share,
    // which goes evenly from one guide to the other: from 1 to 0 over the top, left and bottom sides, and back from 0 to
    // 1 up the right. A level's area changes by the integral of that move along y round the outline, and the left side's
    // share averages a half going down as the right's does going up: nothing.
    let straight = PathBuilder::new(v2(10.0, 0.0)).line_to(v2(10.0, 40.0)).open();
    let leaning = LoftOptions { guides: vec![(&bend, front), (&straight, back)], ..Default::default() };
    let b = loft_through(&sections, &leaning, 1).unwrap();
    close(check(&b), 8000.0, 1e-9, "one corner bent");
    assert!(edge_distance(&b, v3(16.0, 5.0, 20.0)) < 1e-9);
    assert!(edge_distance(&b, v3(10.0, -5.0, 20.0)) < 1e-9);
}

#[test]
fn a_guide_can_be_an_arc_and_meet_a_side_between_corners() {
    let r = rect(20.0, 10.0);
    // An arc from (10, 0) up to (10, 40), out to x = 16 halfway, on the plane y = 0, which crosses the right-hand side
    // halfway along it.
    let (cx, cy) = (-122.0 / 6.0, 20.0);
    let radius = 16.0 - cx;
    let arc = PathBuilder::new(v2(10.0, 0.0)).arc_to(v2(10.0, 40.0), v2(cx, cy), true).open();
    let sections = [outline(&r, at(0.0)), outline(&r, at(40.0))];
    let options = LoftOptions { guides: vec![(&arc, guide_frame(0.0))], ..Default::default() };
    let b = loft_through(&sections, &options, 1).unwrap();
    // A single guide moves every level alike, by an exact arc, so the volume is unchanged.
    close(check(&b), 8000.0, 1e-9, "arc guide");
    for t in [0.1, 0.3, 0.5, 0.7, 0.9] {
        let a0 = (0.0f64 - cy).atan2(10.0 - cx);
        let a = a0 + (2.0 * -a0) * t;
        let p = v3(cx + radius * a.cos(), 0.0, cy + radius * a.sin());
        assert!(edge_distance(&b, p) < 1e-9, "{t}: {}", edge_distance(&b, p));
    }
    // The right-hand side was cut where the guide meets it: five sides and two caps.
    assert_eq!(b.faces.len(), 7);
}

#[test]
fn guides_through_circles_and_to_a_point() {
    // A circle to a smaller one, with an arc guide pushing one side out, and a guide to a point.
    let (c1, c2) = (circle(10.0), circle(6.0));
    let bulge = PathBuilder::new(v2(10.0, 0.0)).spline_to(
        coreshape::spline::Spline::through(&[v3(10.0, 0.0, 0.0), v3(15.0, 15.0, 0.0), v3(6.0, 30.0, 0.0)]).unwrap(),
    );
    let bulge = bulge.open();
    let f = guide_frame(0.0);
    let options = LoftOptions { guides: vec![(&bulge, f)], ..Default::default() };
    let b = loft_through(&[outline(&c1, at(0.0)), outline(&c2, at(30.0))], &options, 1).unwrap();
    check(&b);
    assert!(edge_distance(&b, v3(15.0, 0.0, 15.0)) < 1e-9);
    // To a point, with the guide ending there.
    let to_tip = PathBuilder::new(v2(10.0, 0.0)).line_to(v2(14.0, 10.0)).line_to(v2(0.0, 25.0)).open();
    let options = LoftOptions { guides: vec![(&to_tip, f)], ..Default::default() };
    let b = loft_through(&[outline(&c1, at(0.0)), Section::Point(v3(0.0, 0.0, 25.0))], &options, 1).unwrap();
    check(&b);
    assert!(edge_distance(&b, v3(14.0, 0.0, 10.0)) < 1e-9);
    // And it joins up with other solids.
    let post = Frame { origin: v3(-2.0, -1.5, -5.0), ..Frame::WORLD };
    let tall = union(&b, &box_solid(&post, 4.0, 3.0, 40.0, 2).unwrap()).unwrap();
    check(&tall);
}

#[test]
fn guides_that_miss_are_refused_saying_how() {
    let r = rect(20.0, 10.0);
    let sections = [outline(&r, at(0.0)), outline(&r, at(40.0))];
    // Off the corner by a millimetre.
    let off = PathBuilder::new(v2(11.0, 0.0)).line_to(v2(11.0, 40.0)).open();
    let e = loft_through(&sections, &LoftOptions { guides: vec![(&off, guide_frame(5.0))], ..Default::default() }, 1).unwrap_err();
    assert!(e.to_string().contains("doesn't meet section 0"), "{e}");
    // Too short.
    let short = PathBuilder::new(v2(10.0, 0.0)).line_to(v2(12.0, 20.0)).open();
    let e = loft_through(&sections, &LoftOptions { guides: vec![(&short, guide_frame(5.0))], ..Default::default() }, 1).unwrap_err();
    assert!(e.to_string().contains("section 1"), "{e}");
}
