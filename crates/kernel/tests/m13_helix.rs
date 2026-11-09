//! Helixes, against what's known exactly. A section holding the axis sweeps out `angle * integral of r dA`, whatever the
//! pitch, and a section across it sweeps out its area times the height, since a solid is its sections stacked up.

use coreshape::boolean::{subtract, union};
use coreshape::profile::{Profile, Wire};
use coreshape::surface::Surface;
use coreshape::topo::Face;
use coreshape::*;
use std::f64::consts::{PI, TAU};

/// A frame whose XY plane holds the Z axis: profile x is the distance from the axis and profile y the height.
fn upright() -> Frame {
    Frame { origin: Vec3::ZERO, x: Vec3::X, y: Vec3::Z, z: -Vec3::Y }
}

fn along_z(pitch: f64, turns: f64) -> Helix {
    Helix { origin: Vec3::ZERO, axis: Vec3::Z, pitch, turns, left_handed: false }
}

fn rect(r0: f64, r1: f64, z0: f64, z1: f64) -> Profile {
    Profile::new(Wire::polygon(&[v2(r0, z0), v2(r1, z0), v2(r1, z1), v2(r0, z1)]))
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} against {b}");
}

#[test]
fn a_square_spring_has_the_volume_and_area_worked_out_by_hand() {
    let (r0, r1, tall, pitch, turns): (f64, f64, f64, f64, f64) = (8.0, 10.0, 1.5, 3.0, 2.5);
    let b = helix(&rect(r0, r1, 0.0, tall), &upright(), &along_z(pitch, turns), 1).unwrap();
    validate(&b).unwrap();
    let angle = turns * TAU;
    let volume = angle * tall * (r1 * r1 - r0 * r0) / 2.0;
    let m = mass_properties(&b);
    close(m.volume, volume, 1e-6, "volume");
    // The inner and outer walls are strips of cylinders, and the top and bottom are helicoids.
    let rate = pitch / TAU;
    let helicoid = |r: f64| (r * (r * r + rate * rate).sqrt() + rate * rate * (r + (r * r + rate * rate).sqrt()).ln()) / 2.0;
    let area = angle * tall * (r0 + r1) + 2.0 * angle * (helicoid(r1) - helicoid(r0)) + 2.0 * tall * (r1 - r0);
    close(m.area, area, 1e-6, "area");
    // Four sides for each half turn, and two caps.
    assert_eq!(b.faces.len(), 4 * 5 + 2);
}

#[test]
fn more_turns_leave_the_earlier_faces_where_they_were() {
    let section = rect(8.0, 10.0, 0.0, 1.5);
    let (short, long) = (helix(&section, &upright(), &along_z(3.0, 2.2), 1).unwrap(), helix(&section, &upright(), &along_z(3.0, 3.1), 1).unwrap());
    let half_turn = |f: &Face| f.origin.copies.first().map_or(0, |s| s.index);
    let middle = |b: &Body, f: usize| b.coedge_point(b.faces[f].loops[0][1], 0.5);
    // Those before the last of the short one are the same in both, by their half turn.
    let last = (0..short.faces.len()).map(|f| half_turn(&short.faces[f])).max().unwrap();
    assert_eq!(last, 4);
    for f in 0..short.faces.len() {
        let face = &short.faces[f];
        if !matches!(face.surface, Surface::Free { .. }) || half_turn(face) == last {
            continue;
        }
        let same = (0..long.faces.len()).find(|&g| long.faces[g].tag == face.tag && half_turn(&long.faces[g]) == half_turn(face)).unwrap();
        assert!(middle(&short, f).dist(middle(&long, same)) < 1e-9, "half turn {}", half_turn(face));
    }
    // A sliver at the end goes with the one before it.
    let just_past = helix(&section, &upright(), &along_z(3.0, 2.51), 1).unwrap();
    assert_eq!(just_past.faces.len(), 4 * 5 + 2);
}

#[test]
fn the_fit_is_as_close_to_the_true_helix_as_stated() {
    for (r, pitch, turns) in [(1.0, 0.5, 3.0), (10.0, 3.0, 2.0), (100.0, 40.0, 1.5), (1000.0, 300.0, 1.0)] {
        let wall = r * 0.1;
        let b = helix(&rect(r, r + wall, 0.0, pitch * 0.4), &upright(), &along_z(pitch, turns), 1).unwrap();
        let mut worst: f64 = 0.0;
        for f in &b.faces {
            let Surface::Free { patch, .. } = &f.surface else { continue };
            let ((u0, u1), (v0, v1)) = patch.domain();
            for i in 0..=8 {
                let u = u0 + (u1 - u0) * i as f64 / 8.0;
                // The face's first row is exact, as a section is carried rigidly. Take a point there along by the true motion.
                let start = patch.eval(u, v0);
                for j in 0..=200 {
                    let v = v0 + (v1 - v0) * j as f64 / 200.0;
                    let (s, c) = (v - v0).sin_cos();
                    let truth = v3(start.x * c - start.y * s, start.x * s + start.y * c, start.z + pitch * (v - v0) / TAU);
                    worst = worst.max(patch.eval(u, v).dist(truth));
                }
            }
        }
        assert!(worst < coreshape::helix::FIT * 1.2, "r {r}: {worst}");
        eprintln!("r {r}: worst distance from the true helix {worst:.2e}");
    }
}

#[test]
fn a_round_wire_spring_and_a_hollow_one() {
    let (coil, wire, pitch, turns): (f64, f64, f64, f64) = (12.0, 1.5, 5.0, 4.0);
    let round = Profile::new(Wire::circle(v2(coil, 0.0), wire));
    let b = helix(&round, &upright(), &along_z(pitch, turns), 1).unwrap();
    validate(&b).unwrap();
    close(mass_properties(&b).volume, turns * TAU * PI * wire * wire * coil, 1e-6, "round");
    let hollow = Profile::new(Wire::circle(v2(coil, 0.0), wire)).with_hole(Wire::circle(v2(coil, 0.0), wire / 2.0));
    let tube = helix(&hollow, &upright(), &along_z(pitch, turns), 1).unwrap();
    validate(&tube).unwrap();
    close(mass_properties(&tube).volume, turns * TAU * PI * coil * (wire * wire - wire * wire / 4.0), 1e-6, "hollow");
}

#[test]
fn a_section_across_the_axis_makes_a_twisted_bar() {
    // Around the axis, with the axis inside the outline. Its volume is its area times its height at any twist.
    let square = Profile::new(Wire::polygon(&[v2(-5.0, -3.0), v2(5.0, -3.0), v2(5.0, 3.0), v2(-5.0, 3.0)]));
    for (pitch, turns) in [(40.0, 0.25), (40.0, 1.0), (20.0, 2.0)] {
        let b = helix(&square, &Frame::WORLD, &along_z(pitch, turns), 1).unwrap();
        validate(&b).unwrap();
        close(mass_properties(&b).volume, 60.0 * pitch * turns, 1e-6, "twisted bar");
    }
    // Off to one side of the axis, it's a stack too.
    let off = Profile::new(Wire::circle(v2(9.0, 0.0), 2.0));
    let b = helix(&off, &Frame::WORLD, &along_z(30.0, 1.5), 1).unwrap();
    close(mass_properties(&b).volume, PI * 4.0 * 45.0, 1e-6, "off to one side");
}

#[test]
fn left_handed_is_the_mirror_image() {
    let p = rect(5.0, 7.0, 0.0, 1.0);
    let right = helix(&p, &upright(), &along_z(2.0, 1.75), 1).unwrap();
    let left = helix(&p, &upright(), &Helix { left_handed: true, ..along_z(2.0, 1.75) }, 1).unwrap();
    validate(&left).unwrap();
    let (a, b) = (mass_properties(&right), mass_properties(&left));
    close(a.volume, b.volume, 1e-9, "volume");
    close(a.area, b.area, 1e-9, "area");
    // The section lies in the plane y = 0, so a reflection in it takes one to the other, point for point.
    assert_eq!(right.vertices.len(), left.vertices.len());
    for p in &right.vertices {
        let nearest = left.vertices.iter().map(|q| p.dist(v3(q.x, -q.y, q.z))).fold(f64::INFINITY, f64::min);
        assert!(nearest < 1e-9, "{p:?}: {nearest}");
    }
}

#[test]
fn turns_that_would_run_into_each_other_are_refused() {
    let tall = helix(&rect(5.0, 7.0, 0.0, 3.0), &upright(), &along_z(2.0, 3.0), 1).unwrap_err().to_string();
    assert!(tall.contains("run into each other"), "{tall}");
    // Exactly touching would be a zero-thickness contact.
    let touching = helix(&rect(5.0, 7.0, 0.0, 2.0), &upright(), &along_z(2.0, 3.0), 1).unwrap_err().to_string();
    assert!(touching.contains("run into each other"), "{touching}");
    // A short sweep doesn't get as far as its own start.
    assert!(helix(&rect(5.0, 7.0, 0.0, 3.0), &upright(), &along_z(2.0, 0.75), 1).is_ok());
    // Holding the axis, and touching or crossing it.
    assert!(helix(&rect(0.0, 7.0, 0.0, 1.0), &upright(), &along_z(3.0, 1.0), 1).is_err());
    assert!(helix(&rect(-1.0, 7.0, 0.0, 1.0), &upright(), &along_z(3.0, 1.0), 1).is_err());
    // A section lying along the path, as a tube along its own length would.
    let flat = Frame { origin: v3(0.0, 0.0, 0.0), x: Vec3::Z, y: Vec3::Y, z: Vec3::X };
    assert!(helix(&rect(-1.0, 1.0, -1.0, 1.0), &flat, &along_z(3.0, 1.0), 1).is_err());
    // Numbers out of range.
    assert!(helix(&rect(5.0, 7.0, 0.0, 1.0), &upright(), &along_z(0.0, 1.0), 1).is_err());
    assert!(helix(&rect(5.0, 7.0, 0.0, 1.0), &upright(), &along_z(3.0, 0.0), 1).is_err());
    assert!(helix(&rect(5.0, 7.0, 0.0, 1.0), &upright(), &along_z(3.0, 1000.0), 1).is_err());
}

#[test]
fn a_thread_cut_and_added_to_a_shaft() {
    let shaft = cylinder(&Frame { origin: v3(0.0, 0.0, -2.0), ..Frame::WORLD }, 6.0, 20.0, 1).unwrap();
    let start = shaft.faces.len();
    // A triangular groove, its tip inside the shaft and its base outside, running past both ends.
    let groove = Profile::new(Wire::polygon(&[v2(5.2, -0.6), v2(6.5, -0.6), v2(6.5, 0.6), v2(5.2, 0.6)]));
    let tool = helix(&groove, &Frame { origin: v3(0.0, 0.0, -4.0), ..upright() }, &along_z(2.5, 10.0), 2).unwrap();
    let cut = subtract(&shaft, &tool).unwrap();
    validate(&cut).unwrap();
    let before = mass_properties(&shaft).volume;
    let after = mass_properties(&cut).volume;
    assert!(after < before && after > before * 0.5, "{before} {after}");
    assert!(cut.faces.len() > start);
    // A ridge added, with the same section moved out a little, buried in the shaft.
    let ridge = Profile::new(Wire::polygon(&[v2(5.0, -0.6), v2(6.8, -0.6), v2(6.8, 0.6), v2(5.0, 0.6)]));
    let tool = helix(&ridge, &Frame { origin: v3(0.0, 0.0, 2.0), ..upright() }, &along_z(2.5, 4.0), 2).unwrap();
    let added = union(&shaft, &tool).unwrap();
    validate(&added).unwrap();
    let swept_out = 4.0 * TAU * 1.2 * (6.8f64 * 6.8 - 36.0) / 2.0;
    close(mass_properties(&added).volume, before + swept_out, 1e-5, "ridge");
}

#[test]
fn a_helix_goes_through_a_step_file() {
    let b = helix(&Profile::new(Wire::circle(v2(10.0, 0.0), 1.0)), &upright(), &along_z(4.0, 2.0), 1).unwrap();
    let text = export::step::write_step(&b, "spring");
    let back = import::step::read_step(&text, 1).unwrap();
    validate(&back[0]).unwrap();
    close(mass_properties(&back[0]).volume, mass_properties(&b).volume, 1e-9, "volume after a round trip");
}
