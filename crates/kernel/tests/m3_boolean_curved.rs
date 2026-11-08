//! Milestone 3: Booleans on curved surfaces, checked against closed-form volumes.

use coreshape::boolean::classify::{BodyLoc, Index};
use coreshape::*;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!(
        (a - b).abs() <= rel * b.abs().max(1.0),
        "{what}: got {a}, expected {b} (rel {:e})",
        (a - b).abs() / b.abs()
    );
}

fn z_at(z: f64) -> Frame {
    Frame::translation(v3(0.0, 0.0, z))
}

fn along_x(x: f64) -> Frame {
    Frame::new(v3(x, 0.0, 0.0), Vec3::X, Vec3::Y)
}

fn along_y(y: f64) -> Frame {
    Frame::new(v3(0.0, y, 0.0), Vec3::Y, Vec3::Z)
}

fn all_ops(a: &Body, b: &Body, name: &str) -> (Body, Body, Body) {
    let u = union(a, b).unwrap_or_else(|e| panic!("{name} union: {e}"));
    let d = subtract(a, b).unwrap_or_else(|e| panic!("{name} subtract: {e}"));
    let i = intersect(a, b).unwrap_or_else(|e| panic!("{name} intersect: {e}"));
    let (va, vb) = (vol(a), vol(b));
    close(vol(&u) + vol(&i), va + vb, 1e-7, &format!("{name} union + intersect"));
    close(vol(&d) + vol(&i), va, 1e-7, &format!("{name} subtract + intersect"));
    (u, d, i)
}

#[test]
fn hole_through_a_plate() {
    let plate = box_solid(&Frame::translation(v3(-10.0, -10.0, 0.0)), 20.0, 20.0, 4.0, 1).unwrap();
    let pin = cylinder(&z_at(-5.0), 3.0, 15.0, 2).unwrap();
    let (u, d, i) = all_ops(&plate, &pin, "plate and pin");
    close(vol(&d), 1600.0 - PI * 9.0 * 4.0, 1e-10, "holed plate");
    close(vol(&i), PI * 9.0 * 4.0, 1e-10, "plug");
    close(vol(&u), 1600.0 + PI * 9.0 * 11.0, 1e-10, "pin through plate");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
    assert_eq!(validate(&u).unwrap().genus, vec![0]);
}

#[test]
fn hole_exactly_as_long_as_the_plate_is_thick() {
    // Both end faces of the tool are coplanar with the plate.
    let plate = box_solid(&Frame::translation(v3(-10.0, -10.0, 0.0)), 20.0, 20.0, 4.0, 1).unwrap();
    let pin = cylinder(&z_at(0.0), 3.0, 4.0, 2).unwrap();
    let (_, d, i) = all_ops(&plate, &pin, "flush pin");
    close(vol(&d), 1600.0 - PI * 9.0 * 4.0, 1e-10, "flush hole");
    close(vol(&i), PI * 9.0 * 4.0, 1e-10, "flush plug");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
}

#[test]
fn blind_hole_flush_with_one_face() {
    let plate = box_solid(&Frame::translation(v3(-10.0, -10.0, 0.0)), 20.0, 20.0, 4.0, 1).unwrap();
    let pin = cylinder(&z_at(2.0), 3.0, 2.0, 2).unwrap();
    let (_, d, _) = all_ops(&plate, &pin, "blind hole");
    close(vol(&d), 1600.0 - PI * 9.0 * 2.0, 1e-10, "blind hole volume");
    assert_eq!(validate(&d).unwrap().genus, vec![0]);
}

#[test]
fn steinmetz_solid_equal_cylinders() {
    let r = 3.0;
    let a = cylinder(&z_at(-10.0), r, 20.0, 1).unwrap();
    let b = cylinder(&along_x(-10.0), r, 20.0, 2).unwrap();
    let (_, _, i) = all_ops(&a, &b, "steinmetz");
    close(vol(&i), 16.0 * r * r * r / 3.0, 1e-10, "Steinmetz volume");
    // Its surface is 16 r^2.
    close(mass_properties(&i).area, 16.0 * r * r, 1e-9, "Steinmetz area");
}

#[test]
fn tricylinder_chain() {
    let r = 2.0;
    let a = cylinder(&z_at(-10.0), r, 20.0, 1).unwrap();
    let b = cylinder(&along_x(-10.0), r, 20.0, 2).unwrap();
    let c = cylinder(&along_y(-10.0), r, 20.0, 3).unwrap();
    let two = intersect(&a, &b).unwrap();
    let three = intersect(&two, &c).unwrap();
    close(vol(&three), 8.0 * (2.0 - 2f64.sqrt()) * r * r * r, 1e-9, "tricylinder");
}

/// Numeric reference for two unequal cylinders crossing at right angles.
fn bicylinder_reference(r1: f64, r2: f64) -> f64 {
    // V = 8 * integral_0^s sqrt(r1^2 - x^2) sqrt(r2^2 - x^2) dx for s = min(r1, r2).
    // The substitution x = s sin(u) removes the end-point singularity.
    let s = r1.min(r2);
    let n = 20000;
    let mut total = 0.0;
    for i in 0..n {
        let u = (i as f64 + 0.5) / n as f64 * PI / 2.0;
        let x = s * u.sin();
        let dx = s * u.cos() * (PI / 2.0 / n as f64);
        total += (r1 * r1 - x * x).max(0.0).sqrt() * (r2 * r2 - x * x).max(0.0).sqrt() * dx;
    }
    8.0 * total
}

#[test]
fn unequal_cylinders_need_the_general_tracer() {
    let (r1, r2) = (4.0, 2.5);
    let a = cylinder(&z_at(-10.0), r1, 20.0, 1).unwrap();
    let b = cylinder(&along_x(-10.0), r2, 20.0, 2).unwrap();
    let (_, _, i) = all_ops(&a, &b, "unequal cylinders");
    close(vol(&i), bicylinder_reference(r1, r2), 1e-6, "bicylinder");
    let quartic = i.edges.iter().any(|e| matches!(e.curve, Curve::Sampled(_)));
    assert!(quartic, "expected sampled intersection curves");
}

#[test]
fn skew_cylinders_with_offset_axes() {
    let a = cylinder(&z_at(-10.0), 4.0, 20.0, 1).unwrap();
    let f = Frame::new(v3(-10.0, 1.2, 0.3), v3(1.0, 0.25, 0.1), Vec3::Y);
    let b = cylinder(&f, 1.7, 22.0, 2).unwrap();
    all_ops(&a, &b, "skew cylinders");
}

#[test]
fn coaxial_cylinders_make_a_tube() {
    let outer = cylinder(&z_at(0.0), 5.0, 10.0, 1).unwrap();
    let inner = cylinder(&z_at(-1.0), 3.0, 12.0, 2).unwrap();
    let (_, d, _) = all_ops(&outer, &inner, "tube");
    close(vol(&d), PI * (25.0 - 9.0) * 10.0, 1e-10, "tube volume");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
    // Same length, so both ends are flush.
    let flush = cylinder(&z_at(0.0), 3.0, 10.0, 2).unwrap();
    let (_, d, _) = all_ops(&outer, &flush, "flush tube");
    close(vol(&d), PI * (25.0 - 9.0) * 10.0, 1e-10, "flush tube volume");
}

#[test]
fn identical_and_same_axis_cylinders() {
    let a = cylinder(&z_at(0.0), 5.0, 10.0, 1).unwrap();
    let b = cylinder(&z_at(5.0), 5.0, 10.0, 2).unwrap();
    let (u, d, i) = all_ops(&a, &b, "stacked overlap");
    close(vol(&u), PI * 25.0 * 15.0, 1e-10, "union");
    close(vol(&d), PI * 25.0 * 5.0, 1e-10, "difference");
    close(vol(&i), PI * 25.0 * 5.0, 1e-10, "overlap");
    assert_eq!(validate(&u).unwrap().shells, 1);
    let (u, _, i) = all_ops(&a, &a, "identical");
    close(vol(&u), PI * 25.0 * 10.0, 1e-10, "union with itself");
    close(vol(&i), PI * 25.0 * 10.0, 1e-10, "intersect with itself");
}

#[test]
fn cylinders_stacked_end_to_end_merge() {
    let a = cylinder(&z_at(0.0), 5.0, 10.0, 1).unwrap();
    let b = cylinder(&z_at(10.0), 5.0, 10.0, 2).unwrap();
    let (u, _, _) = all_ops(&a, &b, "end to end");
    close(vol(&u), PI * 25.0 * 20.0, 1e-10, "stack");
    let r = validate(&u).unwrap();
    assert_eq!(r.shells, 1);
    // The shared disc is gone and the side walls have merged.
    assert_eq!(r.faces, 4);
}

#[test]
fn tangent_cylinders_side_by_side() {
    let a = cylinder(&z_at(0.0), 2.0, 5.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(4.0, 0.0, 0.0)), 2.0, 5.0, 2).unwrap();
    let u = union(&a, &b).unwrap();
    close(vol(&u), 2.0 * PI * 4.0 * 5.0, 1e-10, "tangent union");
    validate(&u).unwrap();
    let d = subtract(&a, &b).unwrap();
    close(vol(&d), PI * 4.0 * 5.0, 1e-10, "tangent difference");
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
}

#[test]
fn cylinder_tangent_to_box_wall() {
    let bx = box_solid(&Frame::translation(v3(-10.0, -10.0, 0.0)), 20.0, 10.0, 5.0, 1).unwrap();
    // Outside the wall y = 0, touching it along a line.
    let c = cylinder(&Frame::translation(v3(0.0, 2.0, 0.0)), 2.0, 5.0, 2).unwrap();
    let u = union(&bx, &c).unwrap();
    close(vol(&u), 1000.0 + PI * 4.0 * 5.0, 1e-10, "outside tangent");
    // Inside the wall, touching it along a line: the hole breaks nothing.
    let c = cylinder(&Frame::translation(v3(0.0, -2.0, 1.0)), 2.0, 2.0, 3).unwrap();
    let d = subtract(&bx, &c).unwrap();
    close(vol(&d), 1000.0 - PI * 4.0 * 2.0, 1e-10, "inside tangent");
    validate(&d).unwrap();
}

#[test]
fn sphere_and_plane_cut() {
    let s = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let half_space = box_solid(&Frame::translation(v3(-10.0, -10.0, 0.0)), 20.0, 20.0, 10.0, 2).unwrap();
    let (_, d, i) = all_ops(&s, &half_space, "hemisphere");
    let half = 2.0 / 3.0 * PI * 125.0;
    close(vol(&i), half, 1e-10, "upper hemisphere");
    close(vol(&d), half, 1e-10, "lower hemisphere");
    // A cap cut off away from the equator.
    let top = box_solid(&Frame::translation(v3(-10.0, -10.0, 3.0)), 20.0, 20.0, 10.0, 2).unwrap();
    let cap = intersect(&s, &top).unwrap();
    let h = 2.0;
    close(vol(&cap), PI * h * h * (3.0 * 5.0 - h) / 3.0, 1e-10, "spherical cap");
}

#[test]
fn sphere_cut_off_axis_keeps_the_cap_formula() {
    // The cut is not aligned with the sphere's construction axis, so the cap's
    // circle crosses the seam edges and misses the poles.
    let f = Frame::new(Vec3::ZERO, v3(1.0, 2.0, 0.5), Vec3::X);
    let s = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let cut = box_solid(&Frame { origin: f.to_world(v3(-20.0, -20.0, 3.0)), ..f }, 40.0, 40.0, 20.0, 2).unwrap();
    let cap = intersect(&s, &cut).unwrap();
    let h = 2.0;
    close(vol(&cap), PI * h * h * (3.0 * 5.0 - h) / 3.0, 1e-9, "tilted cap");
}

#[test]
fn two_spheres_make_a_lens() {
    let (r1, r2, d) = (5.0, 4.0, 6.0);
    let a = sphere(&Frame::WORLD, r1, 1).unwrap();
    let b = sphere(&Frame::translation(v3(d, 0.0, 0.0)), r2, 2).unwrap();
    let (_, _, i) = all_ops(&a, &b, "lens");
    let expected = PI * (r1 + r2 - d).powi(2) * (d * d + 2.0 * d * (r1 + r2) - 3.0 * (r1 - r2).powi(2)) / (12.0 * d);
    close(vol(&i), expected, 1e-9, "lens volume");
}

#[test]
fn spheres_touching_at_a_point() {
    let a = sphere(&Frame::WORLD, 2.0, 1).unwrap();
    let b = sphere(&Frame::translation(v3(5.0, 0.0, 0.0)), 3.0, 2).unwrap();
    let u = union(&a, &b).unwrap();
    close(vol(&u), 4.0 / 3.0 * PI * (8.0 + 27.0), 1e-9, "touching spheres");
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
}

#[test]
fn sphere_through_cylinder_on_its_axis() {
    // Bore through a ball: the napkin ring depends only on its height.
    let (rs, rc) = (5.0, 3.0);
    let s = sphere(&Frame::WORLD, rs, 1).unwrap();
    let c = cylinder(&z_at(-10.0), rc, 20.0, 2).unwrap();
    let (_, d, _) = all_ops(&s, &c, "napkin ring");
    let h = 2.0 * (rs * rs - rc * rc).sqrt();
    close(vol(&d), PI * h * h * h / 6.0, 1e-10, "napkin ring volume");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
}

#[test]
fn sphere_and_cylinder_off_axis() {
    let s = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let c = cylinder(&Frame::translation(v3(1.3, 0.4, -9.0)), 2.0, 18.0, 2).unwrap();
    all_ops(&s, &c, "off-axis bore");
}

#[test]
fn cone_cut_by_planes() {
    let cone = cone_frustum(&Frame::WORLD, 4.0, 0.0, 8.0, 1).unwrap();
    // A horizontal cut gives a frustum and a smaller cone.
    let top = box_solid(&Frame::translation(v3(-10.0, -10.0, 4.0)), 20.0, 20.0, 10.0, 2).unwrap();
    let (_, d, i) = all_ops(&cone, &top, "horizontal cut");
    close(vol(&i), PI * 4.0 * 4.0 / 3.0, 1e-10, "tip");
    close(vol(&d), PI * 16.0 * 8.0 / 3.0 - PI * 4.0 * 4.0 / 3.0, 1e-10, "frustum");
    // A cut through the apex along the axis gives exactly half, and the section is
    // bounded by two straight rulings.
    let half = box_solid(&Frame::translation(v3(0.0, -10.0, -1.0)), 20.0, 20.0, 12.0, 3).unwrap();
    let (_, d, i) = all_ops(&cone, &half, "axial cut");
    close(vol(&i), PI * 16.0 * 8.0 / 6.0, 1e-10, "half cone");
    close(vol(&d), PI * 16.0 * 8.0 / 6.0, 1e-10, "other half cone");
}

/// Regression: the ruling from the apex was clipped short at larger scales, leaving
/// the cone uncut. Found by the browser self test at r = 25, h = 45.
#[test]
fn axial_cone_cut_works_at_any_scale() {
    for (r, h) in [(0.5, 0.9), (4.0, 8.0), (25.0, 45.0), (25.0, 50.0), (300.0, 700.0)] {
        let c = cone_frustum(&Frame::WORLD, r, 0.0, h, 1).unwrap();
        let half = box_solid(&Frame::translation(v3(0.0, -2.0 * r, -1.0)), 2.0 * r, 4.0 * r, h + 2.0, 2).unwrap();
        let d = subtract(&c, &half).unwrap_or_else(|e| panic!("r={r} h={h}: {e}"));
        close(vol(&d), PI * r * r * h / 6.0, 1e-10, "half cone");
    }
}

#[test]
fn oblique_plane_cut_makes_an_ellipse_on_cylinders_and_cones() {
    let c = cylinder(&z_at(-10.0), 2.0, 20.0, 1).unwrap();
    let f = Frame::new(Vec3::ZERO, v3(0.0, 0.5, 1.0), Vec3::X);
    let upper = box_solid(&Frame { origin: f.to_world(v3(-30.0, -30.0, 0.0)), ..f }, 60.0, 60.0, 40.0, 2).unwrap();
    let i = intersect(&c, &upper).unwrap();
    // The cut passes through the centre of the cylinder, so it halves it.
    close(vol(&i), PI * 4.0 * 10.0, 1e-9, "oblique cylinder cut");
    assert!(i.edges.iter().any(|e| matches!(e.curve, Curve::Ellipse { .. })));

    let cone = cone_frustum(&z_at(0.0), 6.0, 0.0, 12.0, 3).unwrap();
    let f = Frame::new(v3(0.0, 0.0, 5.0), v3(0.0, 0.4, 1.0), Vec3::X);
    let upper = box_solid(&Frame { origin: f.to_world(v3(-30.0, -30.0, 0.0)), ..f }, 60.0, 60.0, 40.0, 4).unwrap();
    let (_, _, i) = all_ops(&cone, &upper, "oblique cone cut");
    assert!(i.edges.iter().any(|e| matches!(e.curve, Curve::Ellipse { .. })));
}

#[test]
fn torus_cut_in_half_two_ways() {
    let t = torus(&Frame::WORLD, 6.0, 2.0, 1).unwrap();
    let full = 2.0 * PI * PI * 6.0 * 4.0;
    let across = box_solid(&Frame::translation(v3(-20.0, -20.0, 0.0)), 40.0, 40.0, 10.0, 2).unwrap();
    let (_, _, i) = all_ops(&t, &across, "equatorial cut");
    close(vol(&i), full / 2.0, 1e-10, "half torus");
    let along = box_solid(&Frame::translation(v3(0.0, -20.0, -10.0)), 40.0, 40.0, 20.0, 3).unwrap();
    let (_, _, i) = all_ops(&t, &along, "axial cut");
    close(vol(&i), full / 2.0, 1e-10, "half torus");
}

#[test]
fn coaxial_family() {
    let f = Frame::WORLD;
    let sph = sphere(&f, 5.0, 1).unwrap();
    let cone = cone_frustum(&z_at(-6.0), 5.0, 0.0, 12.0, 2).unwrap();
    all_ops(&sph, &cone, "sphere and cone");
    let t = torus(&f, 6.0, 2.0, 3).unwrap();
    let cyl = cylinder(&z_at(-5.0), 6.0, 10.0, 4).unwrap();
    let (_, _, i) = all_ops(&t, &cyl, "torus and cylinder");
    // The cylinder through the tube's centre circle splits off the inner half.
    // Pappus: half-disc area times the path of its centroid.
    let inner = TAU * (6.0 - 4.0 * 2.0 / (3.0 * PI)) * (PI * 4.0 / 2.0);
    close(vol(&i), inner, 1e-9, "inner torus half");
    let wide = cone_frustum(&z_at(-5.0), 2.0, 9.0, 10.0, 5).unwrap();
    all_ops(&t, &wide, "torus and cone");
    let s2 = sphere(&f, 7.0, 6).unwrap();
    all_ops(&t, &s2, "torus and sphere");
}

#[test]
fn general_curved_pairs() {
    let t = torus(&Frame::WORLD, 6.0, 2.0, 1).unwrap();
    let c = cylinder(&Frame::translation(v3(6.0, 0.5, -5.0)), 1.0, 10.0, 2).unwrap();
    all_ops(&t, &c, "torus pierced by a cylinder");
    let s = sphere(&Frame::translation(v3(6.0, 0.0, 1.0)), 2.0, 3).unwrap();
    all_ops(&t, &s, "torus and a ball");
    let k = cone_frustum(&Frame::new(v3(-8.0, 1.0, 0.5), Vec3::X, Vec3::Y), 3.0, 1.0, 16.0, 4).unwrap();
    let y = cylinder(&z_at(-6.0), 3.5, 12.0, 5).unwrap();
    all_ops(&k, &y, "cone through a cylinder");
}

#[test]
fn flange_with_bolt_circle() {
    let w =
        Wire::polygon(&[v2(13.0, 0.0), v2(50.0, 0.0), v2(50.0, 12.0), v2(25.0, 12.0), v2(25.0, 40.0), v2(13.0, 40.0)]);
    let mut flange = revolve(&Profile::new(w), &Frame::WORLD, TAU, 1).unwrap();
    let start = vol(&flange);
    let n = 6;
    for i in 0..n {
        let a = TAU * i as f64 / n as f64;
        let bolt = cylinder(&Frame::translation(v3(38.0 * a.cos(), 38.0 * a.sin(), -1.0)), 4.5, 14.0, 2 + i).unwrap();
        flange = subtract(&flange, &bolt).unwrap();
    }
    close(vol(&flange), start - n as f64 * PI * 4.5 * 4.5 * 12.0, 1e-9, "bolted flange");
    assert_eq!(validate(&flange).unwrap().genus, vec![1 + n as usize]);
}

#[test]
fn result_can_be_used_again_many_times() {
    // Each result feeds the next operation, so errors must not accumulate.
    let mut body = box_solid(&Frame::translation(v3(-30.0, -30.0, 0.0)), 60.0, 60.0, 10.0, 1).unwrap();
    let mut expected = 36000.0;
    for i in 0..5 {
        for j in 0..5 {
            let c = v3(-20.0 + 10.0 * i as f64, -20.0 + 10.0 * j as f64, -1.0);
            let hole = cylinder(&Frame::translation(c), 3.0, 12.0, 2).unwrap();
            body = subtract(&body, &hole).unwrap();
            expected -= PI * 9.0 * 10.0;
        }
    }
    close(vol(&body), expected, 1e-9, "perforated plate");
    assert_eq!(validate(&body).unwrap().genus, vec![25]);
}

#[test]
fn membership_points_agree_on_a_curved_result() {
    let a = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(1.5, 0.2, -9.0)), 3.0, 18.0, 2).unwrap();
    let d = subtract(&a, &b).unwrap();
    let index = Index::new(&d);
    let mut rng = Rng::new(5);
    for _ in 0..2000 {
        let p = v3(rng.range(-6.0, 6.0), rng.range(-6.0, 6.0), rng.range(-6.0, 6.0));
        let in_sphere = p.norm() < 5.0;
        let in_cylinder = (p.x - 1.5).hypot(p.y - 0.2) < 3.0;
        let want = in_sphere && !in_cylinder;
        match index.locate_in_body(p) {
            BodyLoc::Inside => assert!(want, "{p:?}"),
            BodyLoc::Outside => assert!(!want, "{p:?}"),
            BodyLoc::OnFace(_) => {}
        }
    }
}

/// Known limit, pinned as a test. A cylinder internally tangent to a sphere meets it
/// in a figure-eight curve with a double point at the contact. The tracer can't
/// resolve that, so the kernel must refuse, and must never return a wrong solid.
#[test]
fn internally_tangent_sphere_and_cylinder_refuses_rather_than_lies() {
    let a = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(2.0, 0.0, -9.0)), 3.0, 18.0, 2).unwrap();
    match subtract(&a, &b) {
        Err(_) => {}
        Ok(d) => {
            // If this ever starts working, it has to be right.
            let v = mass_properties(&d).volume;
            let i = intersect(&a, &b).unwrap();
            close(v + mass_properties(&i).volume, mass_properties(&a).volume, 1e-7, "tangent bore");
        }
    }
}

/// Regression from fuzzing (GeneralPrimitives seed 4564): a plane cuts a slender cone
/// near its apex in a tight hyperbola. Sampling it at the grid's spacing put a wrong
/// value of about 4e-4 relative into the volume, which the identities caught.
#[test]
fn tight_hyperbola_on_a_slender_cone_keeps_exact_volumes() {
    use coreshape::fuzz::{make_shape, Category};
    let category = Category::GeneralPrimitives;
    let mut rng = Rng::new(4564 ^ ((category as u64 + 1) << 40));
    let a = make_shape(category, &mut rng).unwrap();
    let b = make_shape(category, &mut rng).unwrap();
    all_ops(&a, &b, "tight hyperbola");
    let d = subtract(&a, &b).unwrap();
    for e in &d.edges {
        if let Curve::Sampled(c) = &e.curve {
            for w in c.tangents.windows(2) {
                assert!(w[0].dot(w[1]) > 0.97, "samples turn too far");
            }
        }
    }
}

/// Exact reference for review seed 7000138, where a cone's apex lies on a cylinder and
/// their bases touch internally. The common region is a stack of circle-circle lenses, so
/// its volume comes from direct integration. OpenCASCADE gives 5.45529 here, 6.5 ppm high.
#[test]
fn cone_apex_on_a_cylinder_matches_exact_integration() {
    let a = cone_frustum(&Frame::new(v3(0.0, 1.0, 2.0), Vec3::Z, Vec3::X), 2.0, 0.0, 4.0, 1).unwrap();
    let b = cylinder(&Frame::translation(v3(0.0, 2.0, 2.0)), 1.0, 4.0, 2).unwrap();
    let common = 5.455259503248;
    let (u, d, i) = all_ops(&a, &b, "apex on cylinder");
    close(vol(&i), common, 1e-10, "common");
    close(vol(&d), PI * 16.0 / 3.0 - common, 1e-10, "cut");
    close(vol(&u), PI * 16.0 / 3.0 + PI * 4.0 - common, 1e-10, "union");
}

/// Review seeds that once failed the identities or were refused for grid-level reasons.
#[test]
fn review_seeds_satisfy_the_identities() {
    use coreshape::fuzz::{run_case, Category};
    for seed in [7000138, 7000418, 3833] {
        let out = run_case(Category::AlignedPrimitives, seed, 100);
        assert!(out.wrong.is_empty(), "seed {seed}: {:?}", out.wrong);
        assert!(out.refusals.is_empty(), "seed {seed}: {:?}", out.refusals);
    }
}

/// Holes in one profile used to take seconds through pairwise validation.
#[test]
fn many_holes_in_one_profile() {
    let n = 20;
    let mut p = Profile::new(Wire::rect(0.0, 0.0, 400.0, 400.0));
    for i in 0..n {
        for j in 0..n {
            let s = 380.0 / n as f64;
            p = p.with_hole(Wire::circle(v2(10.0 + s * (i as f64 + 0.5), 10.0 + s * (j as f64 + 0.5)), s * 0.3));
        }
    }
    let b = extrude(&p, &Frame::WORLD, 10.0, 1).unwrap();
    assert_eq!(validate(&b).unwrap().genus, vec![400]);
    let r = 380.0 / n as f64 * 0.3;
    close(vol(&b), 1600000.0 - 400.0 * PI * r * r * 10.0, 1e-10, "perforated plate");
}

/// Regression from the review's chain test: a traced edge split by a later operation has
/// a range starting mid-segment, and quadrature pieces straddled the kinks between
/// segments. Volume and area were off by up to 0.4% on the faces it bounded.
#[test]
fn split_traced_edges_keep_exact_mass_properties() {
    let a = cylinder(&z_at(-20.0), 8.0, 40.0, 1).unwrap();
    let b = cylinder(&along_x(-20.0), 5.0, 40.0, 2).unwrap();
    let u = union(&a, &b).unwrap();
    // Cuts across the traced curves, leaving them with fractional parameter ranges.
    let cut = box_solid(&Frame::translation(v3(-30.0, 1.3, -30.0)), 60.0, 60.0, 60.0, 3).unwrap();
    let (x, y, _) = all_ops(&u, &cut, "cut through traced curves");
    for body in [&x, &y] {
        let fractional = body.edges.iter().any(|e| {
            matches!(e.curve, Curve::Sampled(_)) && e.t0.fract() != 0.0
                || matches!(e.curve, Curve::Sampled(_)) && e.t1.fract() != 0.0
        });
        assert!(fractional, "expected split traced edges");
        for f in &body.faces {
            for lp in &f.loops {
                let closure = coreshape::mass::loop_closure_error(body, lp);
                assert!(closure < 1e-7, "loop misses closing by {closure:e} on a {}", f.surface.name());
            }
        }
    }
}
