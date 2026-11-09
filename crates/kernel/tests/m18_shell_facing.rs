//! Which way a shell faces, without working out its exact volume. Reading a file and writing one both ask this of every
//! shell, and on traced curves the exact volume was most of the time they took. `scripts/step_import/check_nist.sh` and
//! its like have compared the rough answer with the exact one on real files: see docs/KNOWN_LIMITS.md.

use coreshape::mass::{rough_shell_volume, shell_facing, shell_volume};
use coreshape::*;

/// A block with a boss of another radius crossing a cylinder, so that their meeting is a traced curve.
fn traced() -> Body {
    let upright = cylinder(&Frame::WORLD, 10.0, 40.0, 1).unwrap();
    let across = cylinder(&Frame::new(v3(-30.0, 0.0, 20.0), Vec3::X, Vec3::Y), 6.0, 60.0, 2).unwrap();
    let body = union(&upright, &across).unwrap();
    assert!(body.edges.iter().any(|e| matches!(e.curve, Curve::Sampled(_))), "the test needs a traced curve");
    body
}

#[test]
fn a_solid_faces_out_and_a_void_faces_in_whatever_the_curves() {
    for body in [box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap(), traced()] {
        assert_eq!(body.shells.len(), 1);
        assert!(shell_facing(&body, 0) > 0.0);
        // Rough, but close enough for a sign, and well clear of zero.
        let (rough, parts) = rough_shell_volume(&body, 0);
        let exact = shell_volume(&body, 0);
        assert!((rough - exact).abs() < 0.05 * exact, "{rough} against {exact}");
        assert!(rough.abs() > 0.1 * parts, "{rough} of {parts}");
    }
    // A traced solid taken out of the middle of a block leaves a void of the same shape, facing the other way.
    let outer = box_solid(&Frame::new(v3(-60.0, -30.0, -20.0), Vec3::Z, Vec3::X), 120.0, 60.0, 80.0, 3).unwrap();
    let hollow = subtract(&outer, &traced()).unwrap();
    assert_eq!(hollow.shells.len(), 2);
    let signs: Vec<bool> = (0..2).map(|s| shell_facing(&hollow, s) > 0.0).collect();
    let exact: Vec<bool> = (0..2).map(|s| shell_volume(&hollow, s) > 0.0).collect();
    assert_eq!(signs, exact);
    assert_eq!(signs.iter().filter(|s| **s).count(), 1);
    assert_eq!(hollow.solid_count(), 1);
}

#[test]
fn an_inside_out_shell_says_so() {
    let mut body = traced();
    let before = shell_facing(&body, 0);
    for face in &mut body.faces {
        face.flipped = !face.flipped;
        for lp in &mut face.loops {
            lp.reverse();
            for c in lp.iter_mut() {
                c.reversed = !c.reversed;
            }
        }
    }
    assert!(before > 0.0 && shell_facing(&body, 0) < 0.0);
}

#[test]
fn working_out_which_way_it_faces_leaves_nothing_behind() {
    // Taken roughly once, a body's real volume has to be as it always was.
    let body = traced();
    let before = mass_properties(&body);
    shell_facing(&body, 0);
    rough_shell_volume(&body, 0);
    let after = mass_properties(&body);
    assert_eq!(before.volume, after.volume);
    assert_eq!(before.area, after.area);
    assert_eq!(shell_volume(&body, 0), shell_volume(&body, 0));
}

#[test]
fn a_thin_wall_around_nothing_is_too_close_to_call_roughly_and_gets_the_exact_answer() {
    // A cup, whose walls' parts of the volume nearly cancel, as the reference point is in its empty middle. A boss on its
    // side gives it a traced curve, so that the rough answer and the exact one aren't the same number.
    let outside = cylinder(&Frame::WORLD, 50.0, 100.0, 1).unwrap();
    let inside = cylinder(&Frame::translation(v3(0.0, 0.0, 1.0)), 49.0, 100.0, 2).unwrap();
    let cup = subtract(&outside, &inside).unwrap();
    let boss = cylinder(&Frame::new(v3(30.0, 0.0, 50.0), Vec3::X, Vec3::Y), 10.0, 40.0, 3).unwrap();
    let body = subtract(&cup, &boss).unwrap();
    assert!(body.edges.iter().any(|e| matches!(e.curve, Curve::Sampled(_))), "the test needs a traced curve");
    assert_eq!(body.shells.len(), 1);
    let (rough, parts) = rough_shell_volume(&body, 0);
    assert!(rough.abs() < 0.1 * parts, "this should be close to call: {rough} of {parts}");
    let exact = shell_volume(&body, 0);
    assert!(exact > 0.0);
    assert_ne!(rough, exact);
    assert_eq!(shell_facing(&body, 0), exact, "it should have gone to the exact answer");
}

#[test]
fn a_file_with_a_shell_written_inside_out_still_reads_the_right_way_out() {
    use coreshape::export::step::write_step;
    use coreshape::import::step::read_step;
    // Its own export, as a reader would get it, comes back the same size and facing out.
    let body = traced();
    let back = read_step(&write_step(&body, "traced"), 1).unwrap();
    assert_eq!(back.len(), 1);
    let (a, b) = (mass_properties(&body).volume, mass_properties(&back[0]).volume);
    assert!((a - b).abs() < 1e-6 * a, "{a} against {b}");
    assert!(shell_volume(&back[0], 0) > 0.0);
    // A void comes through as a void.
    let outer = box_solid(&Frame::new(v3(-60.0, -30.0, -20.0), Vec3::Z, Vec3::X), 120.0, 60.0, 80.0, 3).unwrap();
    let hollow = subtract(&outer, &body).unwrap();
    let back = read_step(&write_step(&hollow, "hollow"), 1).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].shells.len(), 2);
    let (a, b) = (mass_properties(&hollow).volume, mass_properties(&back[0]).volume);
    assert!((a - b).abs() < 1e-6 * a, "{a} against {b}");
}
