//! Fillets and chamfers with no exact surface. Where an exact one exists, the fitted route has to agree with it, which
//! is the strongest check available. Beyond that, OpenCASCADE's numbers for the same parts.

use coreshape::boolean::{subtract, union};
use coreshape::profile::{PathBuilder, Profile, Wire};
use coreshape::rolling::{chamfer_fitted, fillet_fitted, FIT};
use coreshape::spline::Spline;
use coreshape::surface::Surface;
use coreshape::*;
use std::f64::consts::PI;

fn volume(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn check(b: &Body) -> f64 {
    validate(b).unwrap();
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    let v = volume(b);
    assert!((mesh.volume() - v).abs() < 0.01 * v.abs(), "mesh {} against {v}", mesh.volume());
    v
}

/// The edges of a body at a height, or anything else a test wants to pick.
fn edges_where(b: &Body, f: impl Fn(&Body, usize) -> bool) -> Vec<usize> {
    (0..b.edges.len()).filter(|&e| f(b, e)).collect()
}

fn at_height(z: f64) -> impl Fn(&Body, usize) -> bool {
    move |b, e| {
        let edge = &b.edges[e];
        (edge.curve.eval(edge.t0).z - z).abs() < 1e-9 && (edge.curve.eval(edge.t1).z - z).abs() < 1e-9 && (b.edge_midpoint(e).z - z).abs() < 1e-9
    }
}

// ----- Against exact answers -----

#[test]
fn a_straight_edge_matches_the_exact_fillet() {
    let b = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
    let edge = edges_where(&b, |b, e| {
        let m = b.edge_midpoint(e);
        (m.z - 20.0).abs() < 1e-9 && m.y.abs() < 1e-9
    });
    assert_eq!(edge.len(), 1);
    let exact = fillet(&b, &edge, 3.0, 2).unwrap();
    let fitted = fillet_fitted(&b, &edge, 3.0, 2).unwrap();
    let (x, y) = (check(&exact), check(&fitted));
    assert!((x - y).abs() < 1e-7 * x, "exact {x}, fitted {y}");
    // On a true cylinder it's a section of, the fitted surface is exact too, and has one face for its one edge.
    assert_eq!(fitted.faces.len(), 7);
}

#[test]
fn a_whole_rim_matches_the_exact_fillet() {
    let b = cylinder(&Frame::WORLD, 10.0, 25.0, 1).unwrap();
    let rim = edges_where(&b, at_height(25.0));
    for r in [1.0, 2.5] {
        let exact = fillet(&b, &rim, r, 2).unwrap();
        let fitted = fillet_fitted(&b, &rim, r, 2).unwrap();
        let (x, y) = (check(&exact), check(&fitted));
        assert!((x - y).abs() < 1e-7 * x, "radius {r}: exact {x}, fitted {y}");
    }
}

#[test]
fn a_concave_edge_matches_the_exact_fillet() {
    // A step, whose inside corner gets filled.
    let wire = Wire::polygon(&[v2(0.0, 0.0), v2(30.0, 0.0), v2(30.0, 10.0), v2(15.0, 10.0), v2(15.0, 25.0), v2(0.0, 25.0)]);
    let b = extrude(&Profile::new(wire), &Frame::new(v3(0.0, 0.0, 0.0), Vec3::Z, Vec3::X), 20.0, 1).unwrap();
    let edge = edges_where(&b, |b, e| {
        let m = b.edge_midpoint(e);
        (m.x - 15.0).abs() < 1e-9 && (m.y - 10.0).abs() < 1e-9
    });
    assert_eq!(edge.len(), 1);
    let exact = fillet(&b, &edge, 2.0, 2).unwrap();
    let fitted = fillet_fitted(&b, &edge, 2.0, 2).unwrap();
    let (x, y) = (check(&exact), check(&fitted));
    assert!(x > volume(&b), "a concave fillet adds material");
    assert!((x - y).abs() < 1e-7 * x, "exact {x}, fitted {y}");
}

#[test]
fn chamfers_match_the_exact_ones() {
    let b = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
    let edge = edges_where(&b, |b, e| {
        let m = b.edge_midpoint(e);
        (m.z - 20.0).abs() < 1e-9 && m.y.abs() < 1e-9
    });
    let (x, y) = (check(&chamfer(&b, &edge, 2.0, 2).unwrap()), check(&chamfer_fitted(&b, &edge, 2.0, 2).unwrap()));
    assert!((x - y).abs() < 1e-7 * x, "exact {x}, fitted {y}");

    let c = cylinder(&Frame::WORLD, 10.0, 25.0, 1).unwrap();
    let rim = edges_where(&c, at_height(25.0));
    let (x, y) = (check(&chamfer(&c, &rim, 1.5, 2).unwrap()), check(&chamfer_fitted(&c, &rim, 1.5, 2).unwrap()));
    assert!((x - y).abs() < 1e-6 * x, "exact {x}, fitted {y}");
}

// ----- What it's for -----

/// A cylinder cut by a tilted plane through a point on its axis, so a rim that's an ellipse.
fn elliptical_rim() -> (Body, Vec<usize>) {
    let cyl = cylinder(&Frame::WORLD, 10.0, 40.0, 1).unwrap();
    let f = Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&Frame::translation(v3(-50.0, -50.0, 25.0)));
    let cutter = box_solid(&f, 100.0, 100.0, 50.0, 2).unwrap();
    let b = subtract(&cyl, &cutter).unwrap();
    assert!((volume(&b) - 7853.9816).abs() < 1e-3, "{}", volume(&b));
    let e = edges_where(&b, |b, e| !matches!(b.edges[e].curve, Curve::Circle { .. } | Curve::Line { .. }));
    (b, e)
}

fn tee() -> (Body, Vec<usize>) {
    let main = cylinder(&Frame::new(v3(-30.0, 0.0, 0.0), Vec3::X, Vec3::Y), 10.0, 60.0, 1).unwrap();
    let branch = cylinder(&Frame::WORLD, 5.0, 30.0, 2).unwrap();
    let u = union(&main, &branch).unwrap();
    assert!((volume(&u) - 20445.7252).abs() < 1e-2, "{}", volume(&u));
    let e = edges_where(&u, |b, e| !matches!(b.edges[e].curve, Curve::Circle { .. } | Curve::Line { .. }));
    (u, e)
}

// OpenCASCADE's own fillets, from a script run against it. They're fits as well, so these compare what each one removes
// or adds, within what the two can be expected to differ by. Where the true answer is known, below, this kernel is closer.
#[test]
fn an_elliptical_rim_is_near_opencascades() {
    let (b, e) = elliptical_rim();
    assert!(!e.is_empty());
    for (r, removed) in [(1.0, 7853.9816 - 7835.6284), (2.0, 7853.9816 - 7782.2136)] {
        let got = volume(&b) - check(&fillet(&b, &e, r, 3).unwrap());
        assert!((got - removed).abs() < 0.15, "radius {r}: removed {got}, and OpenCASCADE {removed}");
    }
    let got = volume(&b) - check(&chamfer(&b, &e, 1.0, 3).unwrap());
    assert!(got > 0.0 && got < 60.0, "{got}");
}

/// The volume taken off the elliptical rim by the true rolling ball, from its definition and with none of the kernel.
/// This solid is convex, so a fillet of radius `r` on it leaves what's within `r` of the solid shrunk by `r`: a point is
/// removed if it's in the solid and further than `r` from the shrunken one. Counted in slices around the axis.
fn removed_by_definition(r: f64) -> f64 {
    let angle = 0.5f64;
    // Local coordinates of the cutting plane: it holds the x axis through (0, 0, 25) and tilts about x.
    let n = v3(0.0, -angle.sin(), angle.cos());
    let height = |p: Vec3| n.dot(p - v3(0.0, 0.0, 25.0));
    let shrunk = 10.0 - r;
    // The edge of the shrunken solid: where its smaller cylinder meets its lowered plane.
    let edge_at = |phi: f64| {
        let (x, y) = (shrunk * phi.cos(), shrunk * phi.sin());
        // n . (x, y, z - 25) = -r
        let z = 25.0 + (-r - n.y * y) / n.z;
        v3(x, y, z)
    };
    let distance_to_shrunk = |p: Vec3| -> f64 {
        let rho = p.x.hypot(p.y);
        let above = height(p) + r;
        if above <= 0.0 && rho <= shrunk {
            return 0.0;
        }
        // Straight down onto the lowered plane, if that lands inside the small cylinder.
        if above > 0.0 {
            let q = p - n * above;
            if q.x.hypot(q.y) <= shrunk {
                return above;
            }
        }
        // Straight in to the small cylinder, if that lands under the lowered plane.
        if rho > shrunk {
            let q = v3(p.x * shrunk / rho, p.y * shrunk / rho, p.z);
            if height(q) + r <= 0.0 {
                return rho - shrunk;
            }
        }
        // Otherwise it's on the edge. Newton from this direction, on the one angle.
        let mut phi = p.y.atan2(p.x);
        let h = 1e-6;
        let f = |a: f64| edge_at(a).dist(p).powi(2);
        for _ in 0..12 {
            let (a, b, c) = (f(phi - h), f(phi), f(phi + h));
            let (d1, d2) = ((c - a) / (2.0 * h), (c - 2.0 * b + a) / (h * h));
            if d2 > 0.0 {
                phi -= d1 / d2;
            }
        }
        edge_at(phi).dist(p)
    };
    // Around the axis, then along the radius, with the height of what's removed found exactly on each line upward: above
    // some height a point is further than r from the shrunken solid, and it's removed from there to the top.
    let (around, along) = (720, 500);
    let span = 2.6 * r;
    let mut sum = 0.0;
    for i in 0..around {
        let phi = (i as f64 + 0.5) / around as f64 * 2.0 * PI;
        let radial = v3(phi.sin(), phi.cos(), 0.0);
        let top = |rho: f64| 25.0 + (-n.dot(radial * rho)) / n.z;
        for a in 0..along {
            let rho = 10.0 - span * (a as f64 + 0.5) / along as f64;
            let z_top = top(rho);
            let beyond = |z: f64| distance_to_shrunk(radial * rho + v3(0.0, 0.0, z)) > r;
            if !beyond(z_top) {
                continue;
            }
            let (mut hi, mut lo) = (z_top, z_top - 6.0 * r);
            for _ in 0..40 {
                let m = (hi + lo) / 2.0;
                if beyond(m) {
                    hi = m;
                } else {
                    lo = m;
                }
            }
            sum += rho * (z_top - hi) * (span / along as f64) * (2.0 * PI / around as f64);
        }
    }
    sum
}

#[test]
fn an_elliptical_rim_is_the_true_rolling_ball() {
    let (b, e) = elliptical_rim();
    for r in [1.0, 2.0] {
        let got = volume(&b) - volume(&fillet(&b, &e, r, 3).unwrap());
        let want = removed_by_definition(r);
        assert!((got - want).abs() < 3e-4 * want, "radius {r}: this kernel removes {got}, and the definition says {want}");
    }
}

#[test]
fn a_tee_saddle_is_near_opencascades() {
    let (u, e) = tee();
    assert!(!e.is_empty());
    for (r, added) in [(1.0, 20449.6868 - 20445.7252), (2.0, 20461.4940 - 20445.7252)] {
        let got = check(&fillet(&u, &e, r, 3).unwrap()) - volume(&u);
        assert!((got - added).abs() < 0.1, "radius {r}: added {got}, and OpenCASCADE {added}");
    }
}

#[test]
fn the_rim_of_a_hole_through_a_curved_wall() {
    let wall = cylinder(&Frame::WORLD, 20.0, 40.0, 1).unwrap();
    let hole = cylinder(&Frame::new(v3(0.0, 0.0, 20.0), Vec3::X, Vec3::Y), 4.0, 40.0, 2).unwrap();
    let b = subtract(&wall, &hole).unwrap();
    let e = edges_where(&b, |b, e| !matches!(b.edges[e].curve, Curve::Circle { .. } | Curve::Line { .. }));
    let got = check(&fillet(&b, &e, 1.0, 3).unwrap());
    let removed = volume(&b) - got;
    assert!((removed - (49265.1912 - 49257.4414)).abs() < 0.05, "removed {removed}, and OpenCASCADE {}", 49265.1912 - 49257.4414);
    let chamfered = check(&chamfer(&b, &e, 1.0, 3).unwrap());
    assert!(chamfered < volume(&b));
}

#[test]
fn a_spline_extrusion_can_be_filleted() {
    let points: Vec<Vec3> = [(30.0, 0.0), (20.0, 14.0), (5.0, 9.0), (-10.0, 18.0), (-30.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let wire = PathBuilder::new(v2(30.0, 0.0)).spline_to(Spline::through(&points).unwrap()).close();
    let b = extrude(&Profile::new(wire), &Frame::WORLD, 10.0, 1).unwrap();
    let top = edges_where(&b, |b, e| at_height(10.0)(b, e) && matches!(b.edges[e].curve, Curve::Spline(_)));
    assert!(!top.is_empty());
    let got = check(&fillet(&b, &top, 1.0, 2).unwrap());
    assert!(got < volume(&b));
}

#[test]
fn the_rim_of_a_loft_can_be_filleted() {
    let at = |z: f64| Frame::translation(v3(0.0, 0.0, z));
    let round = |r: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r));
    let b = loft(&[(&round(10.0), at(0.0)), (&round(13.0), at(10.0)), (&round(7.0), at(20.0))], 1).unwrap();
    assert!(b.faces.iter().any(|f| matches!(f.surface, Surface::Free { .. })));
    let rim = edges_where(&b, at_height(20.0));
    let got = check(&fillet(&b, &rim, 1.0, 2).unwrap());
    assert!(got < volume(&b));
}

#[test]
fn a_ball_rolling_over_a_crease_is_refused_by_name() {
    // The sides of a square-to-circle loft are smooth where they meet at the circle, but only exactly there.
    let at = |z: f64| Frame::translation(v3(0.0, 0.0, z));
    let square = Profile::new(Wire::polygon(&[v2(-10.0, -10.0), v2(10.0, -10.0), v2(10.0, 10.0), v2(-10.0, 10.0)]));
    let circle = Profile::new(Wire::circle(v2(0.0, 0.0), 7.0));
    let b = loft(&[(&square, at(0.0)), (&circle, at(20.0))], 1).unwrap();
    let why = fillet(&b, &edges_where(&b, at_height(20.0)), 1.0, 2).err().expect("refused").to_string();
    assert!(why.contains("corner blend"), "{why}");
}

// ----- What holds true of every result -----

#[test]
fn its_long_sides_are_within_the_fit_of_the_faces_they_lie_on() {
    let (b, e) = elliptical_rim();
    let r = fillet(&b, &e, 1.0, 3).unwrap();
    let uses = r.edge_uses();
    let mut looked = 0;
    for (i, edge) in r.edges.iter().enumerate() {
        for u in &uses[i] {
            for k in 0..=40 {
                let p = edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 40.0);
                let d = r.faces[u.0].surface.sdf(p).abs();
                assert!(d < 2.0 * FIT, "{d} off at {k}/40 of edge {i}");
                looked += 1;
            }
        }
    }
    assert!(looked > 100);
}

#[test]
fn it_can_be_cut_afterwards_and_comes_back_from_step() {
    let (b, e) = elliptical_rim();
    let r = fillet(&b, &e, 1.0, 3).unwrap();
    let tool = box_solid(&Frame::translation(v3(-30.0, -30.0, 5.0)), 60.0, 60.0, 4.0, 4).unwrap();
    let cut = subtract(&r, &tool).unwrap();
    check(&cut);
    let across = cylinder(&Frame::new(v3(-30.0, 0.0, 25.0), Vec3::X, Vec3::Y), 3.0, 60.0, 5).unwrap();
    assert!(check(&subtract(&r, &across).unwrap()) < volume(&r));
    let text = export::step::write_step(&r, "fillet");
    let back = import::step::read_step(&text, 1).unwrap();
    assert!((check(&back[0]) - volume(&r)).abs() < 1e-5 * volume(&r));
}

#[test]
fn a_smooth_edge_has_nothing_to_blend() {
    // The two halves of a prism's elliptical wall meet smoothly, at a rail between them.
    let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 9.0, 4.0, 0.0, 2.0 * PI).unwrap();
    let b = extrude(&Profile::new(PathBuilder::new(v2(9.0, 0.0)).spline_to(c).close()), &Frame::WORLD, 3.0, 1).unwrap();
    let rail = edges_where(&b, |b, e| matches!(b.edges[e].curve, Curve::Line { .. }) && (b.edge_midpoint(e).z - 1.5).abs() < 1e-9);
    assert!(!rail.is_empty());
    let why = fillet(&b, &rail[..1], 0.5, 2).err().expect("refused").to_string();
    assert!(why.contains("smooth"), "{why}");
    // Its rim isn't smooth, and that's fine.
    let rim = edges_where(&b, at_height(3.0));
    assert!(check(&fillet(&b, &rim, 0.5, 2).unwrap()) < volume(&b));
}

#[test]
fn a_radius_too_big_is_refused_and_not_guessed_at() {
    let (b, e) = elliptical_rim();
    let why = fillet(&b, &e, 15.0, 3).err().expect("too big").to_string();
    assert!(why.contains("doesn't fit") || why.contains("near-degenerate") || why.contains("valid"), "{why}");
}
