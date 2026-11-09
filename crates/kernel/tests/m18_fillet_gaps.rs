//! Fillets that used to be refused or that left the wrong shape: walls too short for the ball, mitres at reflex corners,
//! balls in concave corners, corners where convex and concave edges meet, and fillets that run off part of a face. Each is
//! checked against a formula, worked out here with none of the kernel, or against OpenCASCADE's volume for the same fillets
//! (`scripts/check_fillets.sh`).

use coreshape::*;
use std::f64::consts::PI;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn sound(b: &Body, what: &str) -> f64 {
    validate(b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let m = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(m.open_edge_count(), 0, "{what}: open mesh");
    let v = vol(b);
    assert!((m.volume() - v).abs() < 0.01 * v.abs(), "{what}: mesh {} against {v}", m.volume());
    v
}

fn close(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs().max(1.0), "{what}: got {got}, expected {want}");
}

/// An edge of a body running through this point.
fn edge_at(b: &Body, p: Vec3) -> usize {
    (0..b.edges.len())
        .find(|&i| {
            let e = &b.edges[i];
            e.curve.closest(p, e.t0, e.t1).1 < 1e-6
        })
        .unwrap_or_else(|| panic!("no edge at {p:?}"))
}

fn edges_at(b: &Body, points: &[Vec3]) -> Vec<usize> {
    points.iter().map(|p| edge_at(b, *p)).collect()
}

fn bx(x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64, tag: u32) -> Body {
    box_solid(&Frame::translation(v3(x, y, z)), dx, dy, dz, tag).unwrap()
}

fn cyl(x: f64, y: f64, z: f64, r: f64, h: f64, tag: u32) -> Body {
    cylinder(&Frame::translation(v3(x, y, z)), r, h, tag).unwrap()
}

fn prism(points: &[(f64, f64)], h: f64) -> Body {
    let list: Vec<Vec2> = points.iter().map(|p| v2(p.0, p.1)).collect();
    extrude(&Profile::new(Wire::polygon(&list)), &Frame::WORLD, h, 1).unwrap()
}

fn simpson(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let n = 20000;
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 0 { 2.0 } else { 4.0 };
    }
    sum * h / 3.0
}

/// What a plain fillet takes from a right-angle corner, per unit of length.
fn k(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

/// What two of those have in common where they cross at a right angle.
fn mitre(r: f64) -> f64 {
    r * r * r * (5.0 / 3.0 - PI / 2.0)
}

/// How far back along the face it's tangent to a fillet touches, when its wall is too short: tangent there, and through
/// the wall's far edge.
fn reach(r: f64, h: f64) -> f64 {
    (2.0 * r * h - h * h).sqrt()
}

/// How deep the cut is at a distance `x` back from the corner.
fn depth(r: f64, h: f64, x: f64) -> f64 {
    let a = reach(r, h);
    r - (r * r - (a - x) * (a - x)).max(0.0).sqrt()
}

fn cliff_area(r: f64, h: f64) -> f64 {
    simpson(0.0, reach(r, h), |x| depth(r, h, x))
}

/// The same around an axis, with the tangent face running away from a wall at this radius, or in from it.
fn cliff_ring(r: f64, h: f64, wall: f64, outward: bool) -> f64 {
    simpson(0.0, reach(r, h), |x| 2.0 * PI * (if outward { wall + x } else { wall - x }) * depth(r, h, x))
}

// ----- A wall too short for the ball -----

#[test]
fn a_plate_thinner_than_the_fillet_keeps_its_far_edge() {
    let (r, h) = (5.0, 2.0);
    let plate = bx(0.0, 0.0, 0.0, 40.0, 30.0, h, 1);
    let e = edge_at(&plate, v3(20.0, 0.0, h));
    let f = fillet(&plate, &[e], r, 2).unwrap();
    close(sound(&f, "thin plate"), 2400.0 - cliff_area(r, h) * 40.0, 1e-9, "thin plate");
    // The wall has gone, and the arc runs through what was its bottom edge, which is still where it was.
    assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Cylinder { .. })).count(), 1);
    assert_eq!(f.faces.len(), 6);
    let bottom = edge_at(&f, v3(20.0, 0.0, 0.0));
    assert!(f.edge_midpoint(bottom).dist(v3(20.0, 0.0, 0.0)) < 1e-9);
    // It touches the top this far back, and no further.
    let top = edge_at(&f, v3(20.0, reach(r, h), h));
    assert!(matches!(f.edges[top].curve, Curve::Line { .. }));
}

#[test]
fn a_chamfer_too_big_for_its_wall_runs_to_its_far_edge() {
    let (d, h) = (5.0, 2.0);
    let plate = bx(0.0, 0.0, 0.0, 40.0, 30.0, h, 1);
    let e = edge_at(&plate, v3(20.0, 0.0, h));
    let f = chamfer(&plate, &[e], d, 2).unwrap();
    close(sound(&f, "thin plate chamfer"), 2400.0 - 0.5 * d * h * 40.0, 1e-9, "thin plate chamfer");
}

#[test]
fn the_rim_of_a_hole_in_a_thin_plate() {
    let (r, h) = (3.0, 2.0);
    let drilled = subtract(&bx(0.0, 0.0, 0.0, 40.0, 40.0, h, 1), &cyl(20.0, 20.0, -1.0, 5.0, h + 2.0, 2)).unwrap();
    let f = fillet(&drilled, &[edge_at(&drilled, v3(25.0, 20.0, h))], r, 3).unwrap();
    close(sound(&f, "hole rim"), vol(&drilled) - cliff_ring(r, h, 5.0, true), 1e-9, "hole rim");
}

#[test]
fn a_low_boss_at_its_base_and_its_top() {
    let (r, h) = (4.0, 2.0);
    let boss = union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 5.0, 1), &cyl(0.0, 0.0, 5.0, 10.0, h, 2)).unwrap();
    let base = fillet(&boss, &[edge_at(&boss, v3(10.0, 0.0, 5.0))], r, 3).unwrap();
    close(sound(&base, "boss base"), vol(&boss) + cliff_ring(r, h, 10.0, true), 1e-9, "boss base");
    let top = fillet(&boss, &[edge_at(&boss, v3(10.0, 0.0, 7.0))], r, 3).unwrap();
    close(sound(&top, "boss top"), vol(&boss) - cliff_ring(r, h, 10.0, false), 1e-9, "boss top");
}

#[test]
fn a_low_rib_and_a_narrow_one() {
    let ribbed = |w: f64, h: f64| union(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 5.0, 1), &bx(30.0 - w / 2.0, 0.0, 5.0, w, 40.0, h, 2)).unwrap();
    let low = ribbed(10.0, 2.0);
    let f = fillet(&low, &[edge_at(&low, v3(25.0, 20.0, 5.0))], 4.0, 3).unwrap();
    close(sound(&f, "low rib"), vol(&low) + cliff_area(4.0, 2.0) * 40.0, 1e-9, "low rib");
    // Its top is narrower than the fillet, so it's the top that's too short, going the other way.
    let thin = ribbed(4.0, 20.0);
    let f = fillet(&thin, &[edge_at(&thin, v3(28.0, 20.0, 25.0))], 6.0, 3).unwrap();
    close(sound(&f, "narrow rib"), vol(&thin) - cliff_area(6.0, 4.0) * 40.0, 1e-9, "narrow rib");
}

#[test]
fn a_step_at_its_top_and_bottom() {
    let (r, h) = (4.0, 2.0);
    let stepped = prism(&[(0.0, 0.0), (60.0, 0.0), (60.0, 5.0), (30.0, 5.0), (30.0, 7.0), (0.0, 7.0)], 40.0);
    let top = fillet(&stepped, &[edge_at(&stepped, v3(30.0, 7.0, 20.0))], r, 3).unwrap();
    close(sound(&top, "step top"), vol(&stepped) - cliff_area(r, h) * 40.0, 1e-9, "step top");
    let bottom = fillet(&stepped, &[edge_at(&stepped, v3(30.0, 5.0, 20.0))], r, 3).unwrap();
    close(sound(&bottom, "step bottom"), vol(&stepped) + cliff_area(r, h) * 40.0, 1e-9, "step bottom");
}

#[test]
fn a_shaft_shoulder_lower_than_the_fillet() {
    let (r, h) = (4.0, 2.0);
    let list: Vec<Vec2> = [(0.0, 0.0), (20.0, 0.0), (20.0, 30.0), (22.0, 30.0), (22.0, 50.0), (0.0, 50.0)].iter().map(|p| v2(p.0, p.1)).collect();
    let shaft = revolve(&Profile::new(Wire::polygon(&list)), &Frame::WORLD, TAU, 1).unwrap();
    let a = reach(r, h);
    // Under the collar: tangent to the thin part, through the collar's outer edge. Fills in a ring.
    let inside = fillet(&shaft, &[edge_at(&shaft, v3(20.0, 0.0, 30.0))], r, 2).unwrap();
    let filled = simpson(30.0 - a, 30.0, |z| {
        let rho = 24.0 - (r * r - (z - (30.0 - a)) * (z - (30.0 - a))).max(0.0).sqrt();
        PI * (rho * rho - 400.0)
    });
    close(sound(&inside, "shoulder inside"), vol(&shaft) + filled, 1e-8, "shoulder inside");
    // Over it: tangent to the thick part, through the same ring's inner edge. Takes one off.
    let outside = fillet(&shaft, &[edge_at(&shaft, v3(22.0, 0.0, 30.0))], r, 2).unwrap();
    let taken = simpson(30.0, 30.0 + a, |z| {
        let rho = 18.0 + (r * r - (z - (30.0 + a)) * (z - (30.0 + a))).max(0.0).sqrt();
        PI * (484.0 - rho * rho)
    });
    close(sound(&outside, "shoulder outside"), vol(&shaft) - taken, 1e-8, "shoulder outside");
}

#[test]
fn a_sharp_edge_whose_slanted_wall_is_too_short() {
    // At 45 degrees a ball needs far more wall than it has. There are two circles tangent to the top and through the wall's
    // far edge here, both touching the top ahead of the corner, and the right one is near where the ball would have been.
    let (r, wall) = (2.0f64, 4.0f64);
    let e = wall * (PI / 4.0).cos();
    let b = prism(&[(0.0, 0.0), (e, -e), (30.0, -e), (30.0, 0.0)], 40.0);
    let f = fillet(&b, &[edge_at(&b, v3(0.0, 0.0, 20.0))], r, 2).unwrap();
    let touches = e + (2.0 * r * e - e * e).sqrt();
    // The triangle of the corner, where it touches and the far edge, less the bit of circle bulging into it. The arc goes
    // back on itself on the way down, so this can't be summed across as a depth.
    let chord = ((touches - e).powi(2) + e * e).sqrt();
    let turn = 2.0 * (chord / (2.0 * r)).asin();
    let area = touches * e / 2.0 - r * r / 2.0 * (turn - turn.sin());
    close(sound(&f, "slanted wall"), vol(&b) - area * 40.0, 1e-8, "slanted wall");
}

#[test]
fn a_wall_with_steps_along_it_has_no_far_edge_to_ride_on() {
    // 3 tall for half its length and 2 for the rest, under a fillet of 5. So the usual arc, cut off by what's there.
    let r = 5.0f64;
    let b = subtract(&bx(0.0, 0.0, 0.0, 60.0, 30.0, 3.0, 1), &bx(30.0, -1.0, -1.0, 31.0, 32.0, 2.0, 2)).unwrap();
    let f = fillet(&b, &[edge_at(&b, v3(10.0, 0.0, 3.0))], r, 3).unwrap();
    let back = |z: f64| r - (r * r - (z + 2.0) * (z + 2.0)).max(0.0).sqrt();
    let taken = 30.0 * simpson(0.0, 3.0, back) + 30.0 * simpson(1.0, 3.0, back);
    close(sound(&f, "stepped wall"), vol(&b) - taken, 1e-8, "stepped wall");
}

#[test]
fn a_window_in_a_short_wall_doesnt_stop_it_riding_on_the_far_edge() {
    let (r, h) = (5.0f64, 2.0f64);
    let b = subtract(&bx(0.0, 0.0, 0.0, 40.0, 30.0, h, 1), &bx(10.0, -1.0, 0.5, 20.0, 6.0, 1.0, 2)).unwrap();
    let f = fillet(&b, &[edge_at(&b, v3(5.0, 0.0, h))], r, 3).unwrap();
    let a = reach(r, h);
    // Where the window already took what the cut would have.
    let back = |z: f64| a - (r * r - (z + 3.0) * (z + 3.0)).max(0.0).sqrt();
    let shared = 20.0 * simpson(0.5, 1.5, back);
    close(sound(&f, "window"), vol(&b) - cliff_area(r, h) * 40.0 + shared, 1e-8, "window");
}

#[test]
fn both_sides_too_short_is_refused_by_name() {
    let rib = union(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 5.0, 1), &bx(29.5, 0.0, 5.0, 1.0, 40.0, 1.0, 2)).unwrap();
    let why = fillet(&rib, &[edge_at(&rib, v3(29.5, 20.0, 6.0))], 5.0, 3).unwrap_err().to_string();
    assert!(why.contains("both of the faces"), "{why}");
}

#[test]
fn a_rim_of_a_thin_plate_mitres_with_its_far_edges() {
    let (r, h) = (5.0, 2.0);
    let plate = bx(0.0, 0.0, 0.0, 40.0, 30.0, h, 1);
    let rim = edges_at(&plate, &[v3(20.0, 0.0, h), v3(20.0, 30.0, h), v3(0.0, 15.0, h), v3(40.0, 15.0, h)]);
    let f = fillet(&plate, &rim, r, 2).unwrap();
    // How far back it cuts at each height, squared, is what two have in common at a corner.
    let a = reach(r, h);
    let width_at = |below_top: f64| a - (r * r - (r - below_top) * (r - below_top)).max(0.0).sqrt();
    let overlap = simpson(0.0, h, |z| width_at(z) * width_at(z));
    close(sound(&f, "thin rim"), 2400.0 - cliff_area(r, h) * 140.0 + 4.0 * overlap, 1e-8, "thin rim");
}

#[test]
fn a_thin_plates_rim_and_uprights_at_once() {
    // Its uprights are shorter than the radius, so they go first, and the rim runs around them through its far edge.
    let h = 2.0f64;
    let plate = bx(0.0, 0.0, 0.0, 40.0, 30.0, h, 1);
    let list = edges_at(
        &plate,
        &[v3(20.0, 0.0, h), v3(20.0, 30.0, h), v3(0.0, 15.0, h), v3(40.0, 15.0, h), v3(0.0, 0.0, 1.0), v3(40.0, 0.0, 1.0), v3(0.0, 30.0, 1.0), v3(40.0, 30.0, 1.0)],
    );
    // At some of these, the tracer lands right on where the rim's straight and round parts touch.
    for r in [3.0f64, 4.0, 5.0, 8.0] {
        let f = fillet(&plate, &list, r, 2).unwrap();
        let rounded = h * (1200.0 - 4.0 * k(r));
        let straight = 2.0 * (40.0 - 2.0 * r + 30.0 - 2.0 * r);
        close(sound(&f, "thin plate all round"), rounded - cliff_area(r, h) * straight - cliff_ring(r, h, r, false), 1e-8, "thin plate all round");
        assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Sphere { .. })).count(), 0);
    }
}

// ----- Corners -----

#[test]
fn a_reflex_corner_gets_a_mitre_and_not_a_step() {
    let r = 2.0;
    let ell = prism(&[(0.0, 0.0), (40.0, 0.0), (40.0, 15.0), (15.0, 15.0), (15.0, 40.0), (0.0, 40.0)], 20.0);
    let rim = edges_at(&ell, &[v3(20.0, 0.0, 20.0), v3(40.0, 7.0, 20.0), v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0), v3(7.0, 40.0, 20.0), v3(0.0, 20.0, 20.0)]);
    let f = fillet(&ell, &rim, r, 2).unwrap();
    // Five corners share their overlap. At the sixth, each tool stops at the other's wall, and the two meet beyond them.
    close(sound(&f, "L rim"), vol(&ell) - k(r) * 160.0 + 5.0 * mitre(r) - mitre(r), 1e-9, "L rim");
    // With no step in it, nothing near the corner is left standing above the fillets.
    let top = f.vertices.iter().filter(|p| (p.z - 20.0).abs() < 1e-9).map(|p| p.dist(v3(15.0, 15.0, 20.0))).fold(f64::MAX, f64::min);
    assert!(top > 1.0, "the top comes within {top} of the corner");
}

#[test]
fn a_square_bosss_base_mitres_around_its_corners() {
    let r = 2.0;
    let b = union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 5.0, 1), &bx(-10.0, -10.0, 5.0, 20.0, 20.0, 10.0, 2)).unwrap();
    let base = edges_at(&b, &[v3(0.0, -10.0, 5.0), v3(0.0, 10.0, 5.0), v3(-10.0, 0.0, 5.0), v3(10.0, 0.0, 5.0)]);
    let f = fillet(&b, &base, r, 3).unwrap();
    close(sound(&f, "square boss"), vol(&b) + k(r) * 80.0 + 4.0 * mitre(r), 1e-9, "square boss");
}

#[test]
fn a_concave_corner_gets_a_ball() {
    let r = 2.0;
    let p = subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 20.0, 1), &bx(10.0, 10.0, 8.0, 40.0, 20.0, 20.0, 2)).unwrap();
    let list = edges_at(&p, &[v3(30.0, 10.0, 8.0), v3(30.0, 30.0, 8.0), v3(10.0, 20.0, 8.0), v3(50.0, 20.0, 8.0), v3(10.0, 10.0, 14.0), v3(50.0, 10.0, 14.0), v3(10.0, 30.0, 14.0), v3(50.0, 30.0, 14.0)]);
    let f = fillet(&p, &list, r, 3).unwrap();
    // Each edge's run less the corners', and each corner a cube less an eighth of a ball.
    let want = vol(&p) + k(r) * (120.0 + 48.0 - 24.0) + 4.0 * r * r * r * (1.0 - PI / 6.0);
    close(sound(&f, "pocket"), want, 1e-9, "pocket");
    assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Sphere { .. })).count(), 4);
}

/// OpenCASCADE's volumes for these.
const L_RIM_AND_UPRIGHTS: f64 = 19300.81290033929;
const L_UPRIGHT_AND_RIMS_BESIDE_IT: f64 = 19474.382271669183;
const POCKET_ALL: f64 = 38436.38458374067;

#[test]
fn where_convex_and_concave_edges_meet_the_odd_one_goes_first() {
    let r = 2.0;
    let ell = prism(&[(0.0, 0.0), (40.0, 0.0), (40.0, 15.0), (15.0, 15.0), (15.0, 40.0), (0.0, 40.0)], 20.0);
    let three = edges_at(&ell, &[v3(15.0, 15.0, 10.0), v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0)]);
    let f = fillet(&ell, &three, r, 2).unwrap();
    close(sound(&f, "L corner"), L_UPRIGHT_AND_RIMS_BESIDE_IT, 1e-9, "L corner");
    // Done in that order by hand, it's the same.
    let up = fillet(&ell, &[edge_at(&ell, v3(15.0, 15.0, 10.0))], r, 2).unwrap();
    let by_hand = fillet(&up, &edges_at(&up, &[v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0)]), r, 3).unwrap();
    close(vol(&by_hand), vol(&f), 1e-10, "by hand");

    let mut all = edges_at(&ell, &[v3(20.0, 0.0, 20.0), v3(40.0, 7.0, 20.0), v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0), v3(7.0, 40.0, 20.0), v3(0.0, 20.0, 20.0)]);
    all.extend(edges_at(&ell, &[v3(40.0, 0.0, 10.0), v3(40.0, 15.0, 10.0), v3(15.0, 15.0, 10.0), v3(15.0, 40.0, 10.0), v3(0.0, 40.0, 10.0), v3(0.0, 0.0, 10.0)]));
    let f = fillet(&ell, &all, r, 2).unwrap();
    close(sound(&f, "L all"), L_RIM_AND_UPRIGHTS, 1e-9, "L all");

    let pocket = subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 20.0, 1), &bx(10.0, 10.0, 8.0, 40.0, 20.0, 20.0, 2)).unwrap();
    let list = edges_at(
        &pocket,
        &[
            v3(30.0, 10.0, 8.0),
            v3(30.0, 30.0, 8.0),
            v3(10.0, 20.0, 8.0),
            v3(50.0, 20.0, 8.0),
            v3(10.0, 10.0, 14.0),
            v3(50.0, 10.0, 14.0),
            v3(10.0, 30.0, 14.0),
            v3(50.0, 30.0, 14.0),
            v3(30.0, 10.0, 20.0),
            v3(30.0, 30.0, 20.0),
            v3(10.0, 20.0, 20.0),
            v3(50.0, 20.0, 20.0),
        ],
    );
    let f = fillet(&pocket, &list, r, 3).unwrap();
    close(sound(&f, "pocket all"), POCKET_ALL, 1e-9, "pocket all");
}

// ----- Running off part of the way along -----

#[test]
fn a_hole_across_the_contact_line_just_cuts_the_fillet() {
    let (r, big) = (5.0, 3.0);
    let b = subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 10.0, 1), &cyl(30.0, 5.0, -1.0, big, 12.0, 2)).unwrap();
    let f = fillet(&b, &[edge_at(&b, v3(10.0, 0.0, 10.0))], r, 3).unwrap();
    // What the tool takes, less where the hole already had it, by summing across the section.
    let width_at = |d: f64| r - (r * r - (r - d) * (r - d)).max(0.0).sqrt();
    let taken = simpson(0.0, r, |d| {
        let w = width_at(d);
        simpson_coarse(0.0, w, |y| {
            let off = big * big - (y - 5.0) * (y - 5.0);
            60.0 - if off > 0.0 { 2.0 * off.sqrt() } else { 0.0 }
        })
    });
    close(sound(&f, "hole across"), 24000.0 - PI * big * big * 10.0 - taken, 1e-4, "hole across");
}

fn simpson_coarse(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let n = 400;
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 0 { 2.0 } else { 4.0 };
    }
    sum * h / 3.0
}

#[test]
fn a_boss_near_the_plates_edge_keeps_its_fillet_over_the_plate() {
    let (r, big, center, plate_edge) = (5.0, 10.0, -18.0, -30.0);
    let b = union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 10.0, 1), &cyl(center, 0.0, 10.0, big, 10.0, 2)).unwrap();
    let f = fillet(&b, &[edge_at(&b, v3(center + big, 0.0, 10.0))], r, 3).unwrap();
    // Each ring of the fillet, around the boss, stops where it would leave the plate.
    let added = simpson(big, big + r, |rho| {
        let height = r - (r * r - (rho - big - r) * (rho - big - r)).max(0.0).sqrt();
        let room = center - plate_edge;
        let turn = if rho <= room { TAU } else { TAU - 2.0 * (room / rho).acos() };
        height * rho * turn
    });
    close(sound(&f, "boss near edge"), vol(&b) + added, 1e-8, "boss near edge");
    let bounds_x = f.vertices.iter().map(|p| p.x).fold(f64::MAX, f64::min);
    assert!(bounds_x >= plate_edge - 1e-9, "it sticks out to {bounds_x}");
}

/// OpenCASCADE's volume for this.
const WALL_WITH_A_LEDGE: f64 = 44195.265811278296;

#[test]
fn a_wall_that_gets_too_short_doesnt_cut_a_notch_in_what_is_beyond_it() {
    let along_y = |points: &[(f64, f64)], y0: f64, dy: f64, tag: u32| -> Body {
        let list: Vec<Vec2> = points.iter().map(|p| v2(p.0, p.1)).collect();
        extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, y0 + dy, 0.0), -Vec3::Y, Vec3::X), dy, tag).unwrap()
    };
    let ledged = union(&bx(0.0, 0.0, 0.0, 60.0, 30.0, 20.0, 1), &along_y(&[(0.0, 0.0), (60.0, 0.0), (60.0, 10.0), (0.0, 18.0)], -10.0, 10.0, 2)).unwrap();
    let f = fillet(&ledged, &[edge_at(&ledged, v3(30.0, 0.0, 20.0))], 4.0, 3).unwrap();
    close(sound(&f, "ledge"), WALL_WITH_A_LEDGE, 1e-8, "ledge");
    // Under the ledge's surface, at the shallow end, it's all still there.
    let index = coreshape::boolean::classify::Index::new(&f);
    use coreshape::boolean::classify::BodyLoc;
    assert_eq!(index.locate_in_body(v3(5.0, 0.0, 16.0)), BodyLoc::Inside);
}

/// A web with a sloped back, between two flanges.
fn shelf() -> Body {
    let list = [v2(0.0, 0.0), v2(120.0, 0.0), v2(0.0, 120.0)];
    let web = extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, 2.5, 0.0), -Vec3::Y, Vec3::X), 5.0, 1).unwrap();
    union(&union(&web, &bx(0.0, -15.0, 0.0, 120.0, 30.0, 5.0, 2)).unwrap(), &bx(0.0, -15.0, 0.0, 5.0, 30.0, 120.0, 3)).unwrap()
}

#[test]
fn blends_along_a_sloped_back_end_on_the_flanges_it_runs_down_to() {
    // Right at its end the section is already below the flange, which once kept the wrong side of it, and left next to nothing.
    let b = shelf();
    let edges = edges_at(&b, &[v3(60.0, 2.5, 60.0), v3(60.0, -2.5, 60.0)]);
    // From flange to flange along the back, and the deeper into the web the shorter that is.
    let length = 110.0 * 2f64.sqrt();
    let c = chamfer(&b, &edges, 1.0, 9).unwrap();
    close(sound(&c, "chamfered back"), vol(&b) - 2.0 * 0.5 * (length - 2.0 / 3.0), 1e-9, "chamfered back");
    let r = 1.0;
    let off_wall = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let f = fillet(&b, &edges, r, 9).unwrap();
    close(sound(&f, "filleted back"), vol(&b) - 2.0 * k(r) * (length - 2.0 * off_wall), 1e-9, "filleted back");
}

#[test]
fn a_ribs_fillet_stops_at_the_wedge_it_runs_down_to() {
    // A triangular rib whose slope comes down to the plate. The fillets along its foot once ran on past it into the air.
    let plate = bx(0.0, -30.0, 0.0, 80.0, 60.0, 10.0, 1);
    let upright = bx(0.0, -30.0, 0.0, 10.0, 60.0, 70.0, 2);
    let list = [v2(10.0, 10.0), v2(80.0, 10.0), v2(10.0, 70.0)];
    let rib = extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, 4.0, 0.0), -Vec3::Y, Vec3::X), 8.0, 3).unwrap();
    let b = union(&union(&plate, &upright).unwrap(), &rib).unwrap();
    let r = 3.0;
    let f = fillet(&b, &edges_at(&b, &[v3(45.0, 4.0, 10.0), v3(45.0, -4.0, 10.0)]), r, 9).unwrap();
    let off_wall = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    close(sound(&f, "rib's foot"), vol(&b) + 2.0 * k(r) * (70.0 - 7.0 / 6.0 * off_wall), 1e-9, "rib's foot");
    let furthest = f.vertices.iter().map(|p| p.x).fold(f64::MIN, f64::max);
    assert!(furthest <= 80.0 + 1e-9, "it reaches x = {furthest}");
}

#[test]
fn a_web_tangent_to_its_boss_stops_its_fillet_where_it_runs_off() {
    // The web's side runs tangent into the boss, so nothing can be cut along it near there: a plane touches the fillet's
    // surface at a single point. So the fillet stops where it would leave the web, and doesn't hang out past its side.
    let (big, small, length, r) = (22.0f64, 13.0f64, 100.0f64, 2.0f64);
    let lean = ((big - small) / length).asin();
    let (c, s) = (lean.cos(), lean.sin());
    let (a, b, d, e) = (v2(big * c, big * s), v2(small * c, length + small * s), v2(-small * c, length + small * s), v2(-big * c, big * s));
    let wire = PathBuilder::new(e).arc_to(a, v2(0.0, 0.0), true).line_to(b).arc_to(d, v2(0.0, length), true).close();
    let web = extrude(&Profile::new(wire), &Frame::translation(v3(0.0, 0.0, -4.0)), 8.0, 1).unwrap();
    let bosses = union(&cyl(0.0, 0.0, -8.0, big, 16.0, 2), &cyl(0.0, length, -8.0, small, 16.0, 3)).unwrap();
    let body = union(&web, &bosses).unwrap();
    let f = fillet(&body, &[edge_at(&body, v3(0.0, big, 4.0))], r, 9).unwrap();
    let v = sound(&f, "tangent web");
    // Around the boss between where the contact, 2 further out, crosses the web's sides.
    let off_wall = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let turn = PI - 2.0 * lean - 2.0 * (big / (big + r)).acos();
    let most = k(r) * (big + off_wall) * turn;
    let added = v - vol(&body);
    assert!(added > 0.98 * most && added < most, "it added {added}, and all of that span would be {most}");
    // Past the web's side there's nothing over the plane of its top.
    let index = coreshape::boolean::classify::Index::new(&f);
    use coreshape::boolean::classify::BodyLoc;
    let side = v2(c, s);
    let along = v2(-s, c);
    for t in [1.0, 3.0, 5.0, 7.0] {
        let p = side * (big + 0.02) + along * t;
        assert_eq!(index.locate_in_body(v3(p.x, p.y, 4.05)), BodyLoc::Outside, "at {t} along the side");
    }
}

#[test]
fn a_fillet_that_ends_on_a_slanted_face_goes_all_the_way_along() {
    // A long edge ending at a chamfer. The tool it's cut with was once too short for the edge.
    let block = bx(0.0, 0.0, 0.0, 40.0, 30.0, 20.0, 1);
    let ch = chamfer(&block, &[edge_at(&block, v3(20.0, 0.0, 20.0))], 3.0, 2).unwrap();
    let f = fillet(&ch, &[edge_at(&ch, v3(40.0, 15.0, 20.0))], 1.0, 3).unwrap();
    let r = 1.0;
    let off_wall = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let want = 24000.0 - 4.5 * 40.0 - (k(r) * 30.0 - k(r) * (3.0 - off_wall));
    close(sound(&f, "ends on a chamfer"), want, 1e-9, "ends on a chamfer");
}
