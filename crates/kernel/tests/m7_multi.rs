//! Many tools at once, reflections and sweeps, checked against closed forms and one-at-a-time results.

use coreshape::boolean::boolean;
use coreshape::*;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol, "{what}: got {a}, want {b}");
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::translation(v3(x, y, z))
}

#[test]
fn separate_tools_go_in_together() {
    let plate = box_solid(&Frame::WORLD, 100.0, 100.0, 5.0, 1).unwrap();
    let tools: Vec<Body> =
        (0..25).map(|i| cylinder(&at(10.0 + (i % 5) as f64 * 20.0, 10.0 + (i / 5) as f64 * 20.0, -1.0), 3.0, 7.0, 2).unwrap()).collect();
    let r = boolean_many(&plate, &tools, BoolOp::Subtract).unwrap();
    close(vol(&r), 50000.0 - 25.0 * PI * 9.0 * 5.0, 1e-7, "plate");
    assert_eq!(validate(&r).unwrap().genus, vec![25]);
}

#[test]
fn many_tools_at_once_match_one_at_a_time() {
    let mut rng = Rng::new(31);
    for round in 0..60 {
        let target = box_solid(&Frame::WORLD, 60.0, 60.0, 20.0, 1).unwrap();
        let tools: Vec<Body> = (0..rng.int(2, 12))
            .map(|i| {
                let (x, y) = (rng.int(0, 60) as f64, rng.int(0, 60) as f64);
                match rng.int(0, 2) {
                    0 => cylinder(&at(x, y, -2.0), rng.int(2, 8) as f64, 26.0, 2 + i as u32).unwrap(),
                    1 => box_solid(&at(x, y, rng.int(0, 15) as f64), rng.int(3, 15) as f64, rng.int(3, 15) as f64, rng.int(3, 15) as f64, 2 + i as u32).unwrap(),
                    _ => sphere(&at(x, y, rng.int(0, 20) as f64), rng.int(2, 9) as f64, 2 + i as u32).unwrap(),
                }
            })
            .collect();
        for op in [BoolOp::Subtract, BoolOp::Union] {
            let together = boolean_many(&target, &tools, op);
            let mut single = Ok(target.clone());
            for t in &tools {
                single = single.and_then(|s| boolean(&s, t, op));
            }
            match (together, single) {
                (Ok(a), Ok(b)) => {
                    let (va, vb) = (vol(&a), vol(&b));
                    assert!((va - vb).abs() < 1e-5 * vb.max(1.0), "round {round} {op:?}: {va} against {vb}");
                    validate(&a).unwrap();
                }
                // Tangencies refuse. What matters is that nothing comes back wrong.
                (Err(_), _) | (_, Err(_)) => {}
            }
        }
    }
}

#[test]
fn overlapping_tools_are_joined_first() {
    let target = box_solid(&Frame::WORLD, 40.0, 10.0, 10.0, 1).unwrap();
    // Two overlapping slots and one apart.
    let tools = [
        box_solid(&at(5.0, 2.0, -1.0), 6.0, 6.0, 12.0, 2).unwrap(),
        box_solid(&at(8.0, 2.0, -1.0), 6.0, 6.0, 12.0, 3).unwrap(),
        box_solid(&at(30.0, 2.0, -1.0), 4.0, 6.0, 12.0, 4).unwrap(),
    ];
    let r = boolean_many(&target, &tools, BoolOp::Subtract).unwrap();
    close(vol(&r), 4000.0 - (9.0 + 4.0) * 6.0 * 10.0, 1e-9, "slots");
    // Intersecting keeps what lies inside any of them.
    let r = boolean_many(&target, &tools, BoolOp::Intersect).unwrap();
    close(vol(&r), (9.0 + 4.0) * 6.0 * 10.0, 1e-9, "kept");
    // No tools leaves the target alone.
    close(vol(&boolean_many(&target, &[], BoolOp::Subtract).unwrap()), 4000.0, 1e-9, "none");
    assert!(merged_tools(&[]).is_err());
}

#[test]
fn a_compound_is_several_shells() {
    let parts = [box_solid(&Frame::WORLD, 1.0, 1.0, 1.0, 1).unwrap(), sphere(&at(10.0, 0.0, 0.0), 2.0, 2).unwrap()];
    let c = compound(&parts);
    assert_eq!(validate(&c).unwrap().shells, 2);
    close(vol(&c), 1.0 + 4.0 / 3.0 * PI * 8.0, 1e-9, "both");
}

// ----- Mirror -----

fn check_mirror(body: &Body, m: &Mirror, what: &str) {
    let image = body.mirrored(m);
    validate(&image).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    close(vol(&image), vol(body), 1e-6 * vol(body).max(1.0), what);
    close(mass_properties(&image).area, mass_properties(body).area, 1e-6 * vol(body).max(1.0), what);
    // Reflecting twice comes back.
    let back = image.mirrored(m);
    close(vol(&back), vol(body), 1e-6 * vol(body).max(1.0), what);
    // It meshes, into a closed surface of the same volume.
    let mesh = tessellate(&image, &TessParams::for_size(image.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(mesh.open_edge_count(), 0, "{what}");
    let (a, b) = (mesh.volume(), vol(&image));
    assert!((a - b).abs() < 0.05 * b, "{what}: mesh {a} against {b}");
}

#[test]
fn every_surface_reflects_correctly() {
    let plane = Mirror::new(v3(3.0, 1.0, 2.0), v3(1.0, 2.0, -0.5));
    let bodies: Vec<(&str, Body)> = vec![
        ("box", box_solid(&at(5.0, 5.0, 5.0), 3.0, 4.0, 5.0, 1).unwrap()),
        ("cylinder", cylinder(&at(8.0, 1.0, 0.0), 2.0, 6.0, 1).unwrap()),
        ("cone", cone_frustum(&at(8.0, 1.0, 0.0), 4.0, 1.0, 6.0, 1).unwrap()),
        ("sphere", sphere(&at(8.0, 1.0, 0.0), 3.0, 1).unwrap()),
        ("torus", torus(&at(12.0, 1.0, 0.0), 5.0, 1.5, 1).unwrap()),
        ("traced curve", union(&cylinder(&at(8.0, 0.0, -8.0), 3.0, 16.0, 1).unwrap(), &cylinder(&Frame::new(v3(0.0, 0.5, 0.0), Vec3::X, Vec3::Y), 2.0, 16.0, 2).unwrap()).unwrap()),
        ("sphere cut by a plane", subtract(&sphere(&at(8.0, 1.0, 0.0), 3.0, 1).unwrap(), &box_solid(&at(8.0, -5.0, -5.0), 10.0, 10.0, 10.0, 2).unwrap()).unwrap()),
        ("rounded", fillet(&box_solid(&at(9.0, 9.0, 9.0), 10.0, 10.0, 10.0, 1).unwrap(), &(0..12).collect::<Vec<_>>(), 2.0, 2).unwrap()),
    ];
    for (name, b) in &bodies {
        check_mirror(b, &plane, name);
    }
}

#[test]
fn a_half_and_its_mirror_make_the_whole() {
    let half = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let image = half.mirrored(&Mirror::new(Vec3::ZERO, Vec3::X));
    let whole = union(&half, &image).unwrap();
    close(vol(&whole), 12000.0, 1e-9, "whole");
    assert_eq!(whole.faces.len(), 6, "the shared face is gone, and the coplanar ones merged");
    // A hemisphere and its image make a sphere.
    let cut = box_solid(&at(0.0, -20.0, -20.0), 20.0, 40.0, 40.0, 2).unwrap();
    let dome = intersect(&sphere(&Frame::WORLD, 10.0, 1).unwrap(), &cut).unwrap();
    let ball = union(&dome, &dome.mirrored(&Mirror::new(Vec3::ZERO, Vec3::X))).unwrap();
    close(vol(&ball), 4.0 / 3.0 * PI * 1000.0, 1e-6, "ball");
}

// ----- Sweep -----

fn disc(r: f64) -> Profile {
    Profile::new(Wire::circle(v2(0.0, 0.0), r))
}

#[test]
fn straight_steps_add_up() {
    let b = sweep(&disc(2.0), &Frame::WORLD, &[PathStep::Line { length: 5.0 }, PathStep::Line { length: 7.0 }], 1).unwrap();
    close(vol(&b), PI * 4.0 * 12.0, 1e-9, "tube");
    assert_eq!(validate(&b).unwrap().shells, 1);
}

#[test]
fn a_bend_has_the_volume_pappus_says() {
    for toward in [0.0, 90.0, 200.0, 305.0] {
        let (r, bend, length) = (2.0, 10.0, 8.0);
        let path = [
            PathStep::Line { length },
            PathStep::Arc { radius: bend, angle: 90f64.to_radians(), toward: f64::to_radians(toward) },
            PathStep::Line { length },
        ];
        let b = sweep(&disc(r), &Frame::WORLD, &path, 1).unwrap();
        // A circle's centre travels along the path, so the volume is its area times the path's length.
        let want = PI * r * r * (2.0 * length + bend * PI / 2.0);
        close(vol(&b), want, 1e-7, "pipe");
        assert_eq!(validate(&b).unwrap().genus, vec![0]);
        let mesh = tessellate(&b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
        assert_eq!(mesh.open_edge_count(), 0);
        // It ends where the path says, heading the new way.
        let (s, c) = f64::to_radians(toward).sin_cos();
        let end = v3(c * bend, s * bend, length + bend) + v3(c, s, 0.0) * length;
        let bounds = b.aabb();
        assert!(bounds.inflated(1e-6).contains(end), "{end:?} in {bounds:?}");
    }
}

#[test]
fn a_rectangular_section_off_center_has_the_right_volume() {
    // Off centre, so the section's own centroid decides the bend's volume.
    let p = Profile::new(Wire::rect(1.0, -2.0, 3.0, 4.0));
    let (bend, angle) = (20.0, 75f64.to_radians());
    let b = sweep(&p, &Frame::WORLD, &[PathStep::Arc { radius: bend, angle, toward: 0.0 }], 1).unwrap();
    // Bending toward +x, the centroid at x = 2.5 is 17.5 from the center.
    close(vol(&b), 12.0 * 17.5 * angle, 1e-7, "section");
    let flipped = sweep(&p, &Frame::WORLD, &[PathStep::Arc { radius: bend, angle, toward: PI }], 1).unwrap();
    close(vol(&flipped), 12.0 * 22.5 * angle, 1e-7, "other way");
}

#[test]
fn a_section_with_a_hole_stays_hollow() {
    let p = Profile::new(Wire::circle(v2(0.0, 0.0), 4.0)).with_hole(Wire::circle(v2(0.0, 0.0), 3.0));
    let path = [PathStep::Line { length: 10.0 }, PathStep::Arc { radius: 12.0, angle: PI / 2.0, toward: PI / 2.0 }, PathStep::Line { length: 10.0 }];
    let b = sweep(&p, &Frame::WORLD, &path, 1).unwrap();
    close(vol(&b), PI * 7.0 * (20.0 + 12.0 * PI / 2.0), 1e-6, "hollow");
    assert_eq!(validate(&b).unwrap().genus, vec![1]);
}

#[test]
fn an_s_bend_and_a_full_turn_work() {
    let path = [
        PathStep::Arc { radius: 10.0, angle: PI / 2.0, toward: 0.0 },
        PathStep::Arc { radius: 10.0, angle: PI / 2.0, toward: PI },
    ];
    let b = sweep(&disc(1.0), &Frame::WORLD, &path, 1).unwrap();
    close(vol(&b), PI * 2.0 * 10.0 * PI / 2.0, 1e-7, "S bend");
    // Up, around in a full ring, and on.
    let ring = sweep(&disc(1.0), &at(0.0, 0.0, 0.0), &[PathStep::Arc { radius: 10.0, angle: TAU, toward: 0.0 }], 1).unwrap();
    close(vol(&ring), PI * TAU * 10.0, 1e-7, "ring");
}

#[test]
fn a_bad_sweep_says_why() {
    let cases: Vec<(Vec<PathStep>, &str)> = vec![
        (vec![], "at least one"),
        (vec![PathStep::Line { length: 0.0 }], "positive"),
        (vec![PathStep::Line { length: f64::NAN }], "positive"),
        (vec![PathStep::Arc { radius: -1.0, angle: 1.0, toward: 0.0 }], "positive"),
        (vec![PathStep::Arc { radius: 5.0, angle: 0.0, toward: 0.0 }], "angle"),
        (vec![PathStep::Arc { radius: 1.0, angle: 1.0, toward: 0.0 }], "too tight"),
    ];
    for (path, expected) in cases {
        let e = sweep(&disc(2.0), &Frame::WORLD, &path, 1).unwrap_err().to_string();
        assert!(e.contains(expected), "{path:?}: {e}");
    }
}
