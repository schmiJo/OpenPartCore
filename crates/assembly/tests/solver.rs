//! The mate solver, checked against what each kind of mate is defined to leave free, and against mechanisms with known
//! closed-form answers.

use coreshape::math::*;
use coreshape_assembly::solver::*;
use std::f64::consts::FRAC_PI_2;

fn joint(a: usize, b: usize, kind: MateKind) -> Joint {
    Joint { a, b, on_a: Frame::WORLD, on_b: Frame::WORLD, kind, limits: Vec::new(), turned: 0.0 }
}

/// Something fixed at the origin, and something free, a long way off and turned.
fn pair(kind: MateKind) -> Problem {
    let away = Frame::new(v3(30.0, -20.0, 45.0), v3(0.3, 0.8, 0.5), v3(1.0, 0.0, 0.2));
    Problem { poses: vec![Frame::WORLD, away], fixed: vec![true, false], joints: vec![joint(0, 1, kind)], ..Problem::default() }
}

const ALL: [MateKind; 8] = [MateKind::Fastened, MateKind::Revolute, MateKind::Slider, MateKind::Cylindrical, MateKind::PinSlot, MateKind::Planar, MateKind::Ball, MateKind::Parallel];

#[test]
fn each_mate_leaves_the_freedom_onshape_says_it_does() {
    for kind in ALL {
        let mut p = pair(kind);
        let r = solve(&mut p);
        assert!(r.failing.is_empty(), "{kind:?}: {r:?}");
        assert!(r.residual <= 1e-8, "{kind:?}: {r:?}");
        assert_eq!(r.freedom, kind.freedom(), "{kind:?}");
        assert_eq!(r.free, vec![false, kind.freedom() > 0], "{kind:?}");
    }
}

/// The free thing's frame, as seen from the fixed one.
fn relative(p: &Problem) -> Frame {
    p.poses[1]
}

#[test]
fn each_mate_brings_together_what_it_should() {
    let tol = 1e-7;
    let (close, flat) = (|a: f64, b: f64| (a - b).abs() < tol, |f: &Frame| f.z.dot(Vec3::Z) > 1.0 - 1e-12);
    for kind in ALL {
        let mut p = pair(kind);
        solve(&mut p);
        let f = relative(&p);
        match kind {
            MateKind::Fastened => {
                assert!(f.origin.norm() < tol && f.x.dist(Vec3::X) < tol && f.z.dist(Vec3::Z) < tol, "{f:?}");
            }
            MateKind::Revolute => assert!(f.origin.norm() < tol && flat(&f), "{f:?}"),
            MateKind::Slider => {
                assert!(close(f.origin.x, 0.0) && close(f.origin.y, 0.0) && f.x.dist(Vec3::X) < tol && flat(&f), "{f:?}");
            }
            MateKind::Cylindrical => assert!(close(f.origin.x, 0.0) && close(f.origin.y, 0.0) && flat(&f), "{f:?}"),
            MateKind::PinSlot => assert!(close(f.origin.y, 0.0) && close(f.origin.z, 0.0) && flat(&f), "{f:?}"),
            MateKind::Planar => assert!(close(f.origin.z, 0.0) && flat(&f), "{f:?}"),
            MateKind::Ball => assert!(f.origin.norm() < tol, "{f:?}"),
            MateKind::Parallel => assert!(flat(&f), "{f:?}"),
        }
    }
}

#[test]
fn a_mate_moves_a_thing_as_little_as_it_has_to() {
    // Already on the z axis and already upright, a slider has nothing to do but square up its turn.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::translation(v3(0.0, 0.0, 17.0))], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Slider)], ..Problem::default() };
    solve(&mut p);
    assert!(p.poses[1].origin.dist(v3(0.0, 0.0, 17.0)) < 1e-12);
    // A planar mate keeps where it is in the plane.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::translation(v3(3.0, 4.0, 5.0))], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Planar)], ..Problem::default() };
    solve(&mut p);
    assert!(p.poses[1].origin.dist(v3(3.0, 4.0, 0.0)) < 1e-9, "{:?}", p.poses[1].origin);
    // And something with no mates doesn't move at all.
    let far = Frame::new(v3(1.0, 2.0, 3.0), v3(1.0, 1.0, 1.0), Vec3::X);
    let mut p = Problem { poses: vec![far], fixed: vec![false], ..Problem::default() };
    let r = solve(&mut p);
    assert_eq!(p.poses[0], far);
    assert_eq!(r.freedom, 6);
}

#[test]
fn something_with_two_free_ends_splits_the_move_unless_told_otherwise() {
    let start = vec![Frame::WORLD, Frame::translation(v3(10.0, 0.0, 0.0))];
    let mut p = Problem { poses: start.clone(), fixed: vec![false, false], joints: vec![joint(0, 1, MateKind::Ball)], ..Problem::default() };
    solve(&mut p);
    assert!(p.poses[0].origin.dist(v3(5.0, 0.0, 0.0)) < 1e-6, "{:?}", p.poses[0].origin);
    // As when a mate's added: the first one comes to the second, which barely moves.
    let mut p = Problem { poses: start, fixed: vec![false, false], joints: vec![joint(0, 1, MateKind::Ball)], prefer: vec![0], ..Problem::default() };
    solve(&mut p);
    assert!(p.poses[0].origin.dist(v3(10.0, 0.0, 0.0)) < 1e-3, "{:?}", p.poses[0].origin);
    assert!(p.poses[1].origin.dist(v3(10.0, 0.0, 0.0)) < 1e-3);
}

/// Crank of length r about the origin in the XY plane, with a rod of length l to a piston sliding along x.
fn slider_crank(r: f64, l: f64, angle: f64) -> Problem {
    let (s, c) = angle.sin_cos();
    let pin = v3(r * c, r * s, 0.0);
    let piston = v3(r * c + (l * l - r * r * s * s).sqrt(), 0.0, 0.0);
    let rod = (piston - pin).normalized();
    let turn = Frame::new(pin, Vec3::Z, rod);
    // Thing 0 is the frame, 1 the crank, 2 the rod and 3 the piston, a little off where they belong.
    let mut poses = vec![Frame::WORLD, Frame::new(Vec3::ZERO, Vec3::Z, v3(c, s, 0.0)), turn, Frame::translation(piston)];
    for p in poses.iter_mut().skip(1) {
        p.origin = p.origin + v3(0.5, -0.5, 0.25);
    }
    let crank_end = Frame::translation(v3(r, 0.0, 0.0));
    let rod_end = Frame::translation(v3(l, 0.0, 0.0));
    let along_x = Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y);
    Problem {
        poses,
        fixed: vec![true, false, false, false],
        joints: vec![
            joint(0, 1, MateKind::Revolute),
            Joint { on_a: crank_end, ..joint(1, 2, MateKind::Revolute) },
            Joint { on_a: rod_end, ..joint(2, 3, MateKind::Revolute) },
            Joint { on_a: along_x, on_b: along_x, ..joint(0, 3, MateKind::Slider) },
        ],
        ..Problem::default()
    }
}

fn piston_at(r: f64, l: f64, angle: f64) -> f64 {
    r * angle.cos() + (l * l - r * r * angle.sin() * angle.sin()).sqrt()
}

#[test]
fn a_slider_crank_follows_its_formula_around_a_whole_turn() {
    let (r, l) = (10.0, 35.0);
    let mut p = slider_crank(r, l, 0.3);
    let report = solve(&mut p);
    assert!(report.failing.is_empty() && report.residual <= 1e-8, "{report:?}");
    // One closed loop in a plane, whose mates overlap so that the one freedom left is the crank's.
    assert_eq!(report.freedom, 1, "{report:?}");
    let mut last = joint_value(&p, 0, Dof::Angle);
    // Driven all the way round by the crank's mate, and past a full turn.
    for k in 1..=60 {
        let angle = 0.3 + TAU * 1.5 * k as f64 / 60.0;
        p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: angle }];
        let report = solve(&mut p);
        assert!(report.failing.is_empty(), "at {angle}: {report:?}");
        let x = p.poses[3].origin.x;
        assert!((x - piston_at(r, l, angle)).abs() < 1e-6, "at {angle}: {x} against {}", piston_at(r, l, angle));
        let now = joint_value(&p, 0, Dof::Angle);
        assert!((now - angle).abs() < 1e-7 && now > last, "{now} after {last}, for {angle}");
        last = now;
    }
}

#[test]
fn a_four_bar_linkage_follows_freudensteins_equation() {
    // Ground from (0, 0) to (d, 0), crank a, coupler b, rocker c.
    let (a, b, c, d) = (10.0f64, 30.0f64, 25.0f64, 28.0f64);
    let solve_rocker = |theta: f64| -> f64 {
        // Rocker angle by circle intersection, on the branch with the coupler above the ground.
        let (px, py) = (a * theta.cos(), a * theta.sin());
        let (dx, dy) = (d - px, -py);
        let e = dx.hypot(dy);
        let along = (b * b - c * c + e * e) / (2.0 * e);
        let up = (b * b - along * along).sqrt();
        let (qx, qy) = (px + along * dx / e - up * dy / e, py + along * dy / e + up * dx / e);
        (qy).atan2(qx - d)
    };
    let theta = 0.8;
    let phi = solve_rocker(theta);
    let q = v3(d + c * phi.cos(), c * phi.sin(), 0.0);
    let pin = v3(a * theta.cos(), a * theta.sin(), 0.0);
    let poses = vec![
        Frame::WORLD,
        Frame::new(Vec3::ZERO, Vec3::Z, pin),
        Frame::new(pin, Vec3::Z, q - pin),
        Frame::new(v3(d, 0.0, 0.0), Vec3::Z, q - v3(d, 0.0, 0.0)),
    ];
    let along = |x: f64| Frame::translation(v3(x, 0.0, 0.0));
    let mut p = Problem {
        poses,
        fixed: vec![true, false, false, false],
        joints: vec![
            joint(0, 1, MateKind::Revolute),
            Joint { on_a: along(a), ..joint(1, 2, MateKind::Revolute) },
            Joint { on_a: along(b), on_b: along(c), ..joint(2, 3, MateKind::Revolute) },
            Joint { on_a: along(d), ..joint(0, 3, MateKind::Revolute) },
        ],
        ..Problem::default()
    };
    let report = solve(&mut p);
    assert!(report.failing.is_empty(), "{report:?}");
    assert_eq!(report.freedom, 1);
    for k in 1..=40 {
        let t = theta + 1.5 * k as f64 / 40.0;
        p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: t }];
        let report = solve(&mut p);
        assert!(report.failing.is_empty(), "at {t}: {report:?}");
        let rocker = p.poses[3].x;
        let got = rocker.y.atan2(rocker.x);
        let want = solve_rocker(t);
        assert!(wrap_pi(got - want).abs() < 1e-6, "at {t}: {got} against {want}");
        // Freudenstein's equation itself, which doesn't depend on how the answer above was worked out.
        let k1 = d / a;
        let k2 = d / c;
        let k3 = (a * a - b * b + c * c + d * d) / (2.0 * a * c);
        let left = k1 * got.cos() - k2 * t.cos() + k3;
        assert!((left - (t - got).cos()).abs() < 1e-6, "at {t}: {left} against {}", (t - got).cos());
    }
}

#[test]
fn a_fixed_thing_doesnt_move_and_a_conflict_is_named() {
    // Two fixed things a mate can't bring together.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::translation(v3(5.0, 0.0, 0.0))], fixed: vec![true, true], joints: vec![joint(0, 1, MateKind::Fastened)], ..Problem::default() };
    let r = solve(&mut p);
    assert_eq!(r.failing, vec![0]);
    assert_eq!(p.poses[1].origin, v3(5.0, 0.0, 0.0));

    // A thing held at two places at once. The first is met, and the second named.
    let there = Frame::translation(v3(50.0, 0.0, 0.0));
    let mut p = Problem {
        poses: vec![Frame::WORLD, there, Frame::translation(v3(20.0, 20.0, 0.0))],
        fixed: vec![true, true, false],
        joints: vec![joint(0, 2, MateKind::Fastened), joint(1, 2, MateKind::Fastened)],
        ..Problem::default()
    };
    let r = solve(&mut p);
    assert_eq!(r.failing, vec![1]);
    assert!(p.poses[2].origin.norm() < 1e-7, "{:?}", p.poses[2].origin);
    assert!(r.residual <= 1e-8);
}

#[test]
fn redundant_mates_are_fine_and_take_no_more_freedom() {
    // Two sliders along the same line between the same two things say the same thing.
    let mut p = pair(MateKind::Slider);
    p.joints.push(joint(0, 1, MateKind::Slider));
    let r = solve(&mut p);
    assert!(r.failing.is_empty() && r.residual <= 1e-8, "{r:?}");
    assert_eq!(r.freedom, 1);
    // A revolute and a slider along one axis leave nothing that both allow.
    let mut p = pair(MateKind::Slider);
    p.joints.push(joint(0, 1, MateKind::Revolute));
    let r = solve(&mut p);
    assert!(r.failing.is_empty(), "{r:?}");
    assert_eq!(r.freedom, 0);
    assert_eq!(r.free, vec![false, false]);
}

#[test]
fn things_in_a_group_move_as_one() {
    let (a, b) = (Frame::translation(v3(10.0, 0.0, 0.0)), Frame::new(v3(20.0, 5.0, 0.0), Vec3::X, Vec3::Y));
    let mut p = Problem {
        poses: vec![Frame::WORLD, a, b],
        fixed: vec![true, false, false],
        groups: vec![vec![1, 2]],
        joints: vec![joint(0, 1, MateKind::Fastened)],
        ..Problem::default()
    };
    let r = solve(&mut p);
    assert_eq!(r.freedom, 0);
    assert_eq!(r.free, vec![false, false, false]);
    // The first came 10 back along x, and the other with it, keeping how they were to each other.
    assert!(p.poses[1].origin.norm() < 1e-7);
    assert!(p.poses[2].origin.dist(v3(10.0, 5.0, 0.0)) < 1e-7, "{:?}", p.poses[2].origin);
    assert!(p.poses[2].z.dist(Vec3::X) < 1e-7);
    // Something fixed in a group fixes the group.
    let mut p = Problem { poses: vec![a, b], fixed: vec![true, false], groups: vec![vec![0, 1]], ..Problem::default() };
    let r = solve(&mut p);
    assert_eq!(r.freedom, 0);
    assert_eq!(r.free, vec![false, false]);
}

#[test]
fn a_limit_holds_a_mate_where_it_says() {
    let along_limit = |min: f64, max: f64| vec![Limit { dof: Dof::Z, min, max }];
    let mut p = pair(MateKind::Slider);
    p.joints[0].limits = along_limit(-5.0, 5.0);
    solve(&mut p);
    assert!((p.poses[1].origin.z - 5.0).abs() < 1e-6, "{:?}", p.poses[1].origin);
    // Inside, it's where it was.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::translation(v3(0.0, 0.0, 2.0))], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Slider)], ..Problem::default() };
    p.joints[0].limits = along_limit(-5.0, 5.0);
    let r = solve(&mut p);
    assert!((p.poses[1].origin.z - 2.0).abs() < 1e-9);
    // A limit doesn't take a freedom.
    assert_eq!(r.freedom, 1);
    // And an angle.
    let turned = Frame::new(Vec3::ZERO, Vec3::Z, v3(0.0, 1.0, 0.0));
    let mut p = Problem { poses: vec![Frame::WORLD, turned], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Revolute)], ..Problem::default() };
    p.joints[0].limits = vec![Limit { dof: Dof::Angle, min: -0.5, max: 0.5 }];
    p.joints[0].turned = FRAC_PI_2;
    solve(&mut p);
    let x = p.poses[1].x;
    assert!((x.y.atan2(x.x) - 0.5).abs() < 1e-6, "{x:?}");
}

#[test]
fn a_gear_ties_two_turns_together_past_a_whole_turn() {
    let at = |x: f64| Frame::translation(v3(x, 0.0, 0.0));
    let mut p = Problem {
        poses: vec![Frame::WORLD, Frame::WORLD, at(30.0)],
        fixed: vec![true, false, false],
        joints: vec![joint(0, 1, MateKind::Revolute), Joint { on_a: at(30.0), ..joint(0, 2, MateKind::Revolute) }],
        // Two teeth to one, the other way round.
        couplings: vec![Coupling { a: (0, Dof::Angle), b: (1, Dof::Angle), ratio: -0.5, offset: 0.0 }],
        ..Problem::default()
    };
    let r = solve(&mut p);
    assert!(r.failing.is_empty(), "{r:?}");
    assert_eq!(r.freedom, 1);
    for k in 1..=36 {
        let angle = 3.0 * TAU * k as f64 / 36.0;
        p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: angle }];
        let r = solve(&mut p);
        assert!(r.failing.is_empty() && r.failing_couplings.is_empty(), "{r:?}");
        assert!((joint_value(&p, 1, Dof::Angle) + 0.5 * angle).abs() < 1e-6, "{} at {angle}", joint_value(&p, 1, Dof::Angle));
    }
}

#[test]
fn a_rack_and_a_screw_tie_a_turn_to_a_slide() {
    // Rack and pinion: 10 mm for each turn of a pinion.
    let per_radian = 10.0 / TAU;
    let mut p = Problem {
        poses: vec![Frame::WORLD, Frame::WORLD, Frame::translation(v3(30.0, 0.0, 0.0))],
        fixed: vec![true, false, false],
        joints: vec![joint(0, 1, MateKind::Revolute), Joint { on_a: Frame::translation(v3(30.0, 0.0, 0.0)), ..joint(0, 2, MateKind::Slider) }],
        couplings: vec![Coupling { a: (0, Dof::Angle), b: (1, Dof::Z), ratio: per_radian, offset: 0.0 }],
        ..Problem::default()
    };
    assert_eq!(solve(&mut p).freedom, 1);
    p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: TAU * 2.5 }];
    for k in 1..=10 {
        p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: TAU * 2.5 * k as f64 / 10.0 }];
        assert!(solve(&mut p).failing.is_empty());
    }
    assert!((p.poses[2].origin.z - 25.0).abs() < 1e-6, "{:?}", p.poses[2].origin);

    // A screw on one cylindrical mate: 2 mm a turn, which leaves one freedom of the two.
    let mut p = pair(MateKind::Cylindrical);
    p.poses[1] = Frame::WORLD;
    p.couplings = vec![Coupling { a: (0, Dof::Angle), b: (0, Dof::Z), ratio: 2.0 / TAU, offset: 0.0 }];
    assert_eq!(solve(&mut p).freedom, 1);
    for k in 1..=8 {
        p.drives = vec![Drive { joint: 0, dof: Dof::Angle, value: TAU * k as f64 / 4.0 }];
        assert!(solve(&mut p).failing.is_empty());
    }
    assert!((p.poses[1].origin.z - 4.0).abs() < 1e-6, "{:?}", p.poses[1].origin);
}

#[test]
fn a_drag_goes_as_far_as_the_mates_let_it() {
    // Free: exactly there, turned or not.
    let target = Frame::new(v3(10.0, 20.0, 30.0), v3(1.0, 0.0, 1.0), Vec3::Y);
    let mut p = Problem { poses: vec![Frame::WORLD], fixed: vec![false], ..Problem::default() };
    let d = drag(&mut p, &Goal::Pose { thing: 0, to: target });
    assert!(d.remaining < 1e-6, "{d:?}");
    assert!(p.poses[0].origin.dist(target.origin) < 1e-6 && p.poses[0].z.dist(target.z) < 1e-6);

    // A slider: along it, and no further off it than it was.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::WORLD], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Slider)], ..Problem::default() };
    let d = drag(&mut p, &Goal::Point { thing: 1, local: Vec3::ZERO, to: v3(5.0, 0.0, 12.0) });
    assert!(p.poses[1].origin.dist(v3(0.0, 0.0, 12.0)) < 1e-6, "{:?}", p.poses[1].origin);
    assert!((d.remaining - 5.0).abs() < 1e-6, "{d:?}");

    // A limit stops it.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::WORLD], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Slider)], ..Problem::default() };
    p.joints[0].limits = vec![Limit { dof: Dof::Z, min: -1.0, max: 8.0 }];
    drag(&mut p, &Goal::Point { thing: 1, local: Vec3::ZERO, to: v3(0.0, 0.0, 20.0) });
    assert!((p.poses[1].origin.z - 8.0).abs() < 1e-6, "{:?}", p.poses[1].origin);

    // On a revolute, a point on an arm swings round to where it can get closest.
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::WORLD], fixed: vec![true, false], joints: vec![joint(0, 1, MateKind::Revolute)], ..Problem::default() };
    drag(&mut p, &Goal::Point { thing: 1, local: v3(10.0, 0.0, 0.0), to: v3(0.0, 50.0, 0.0) });
    let tip = p.poses[1].to_world(v3(10.0, 0.0, 0.0));
    assert!(tip.dist(v3(0.0, 10.0, 0.0)) < 1e-4, "{tip:?}");

    // Fixed, it doesn't go anywhere.
    let mut p = Problem { poses: vec![Frame::WORLD], fixed: vec![true], ..Problem::default() };
    let d = drag(&mut p, &Goal::Pose { thing: 0, to: target });
    assert_eq!(p.poses[0], Frame::WORLD);
    assert!(d.remaining > 1.0);
}

#[test]
fn dragging_a_mechanism_moves_what_it_has_to_and_nothing_else() {
    let (r, l) = (10.0, 35.0);
    let mut p = slider_crank(r, l, 0.3);
    p.poses.push(Frame::translation(v3(500.0, 500.0, 0.0)));
    p.fixed.push(false);
    solve(&mut p);
    let bystander = p.poses[4];
    // Pulling the piston along its slide drives everything through the rod, within reach of the crank.
    let want = 40.0;
    let d = drag(&mut p, &Goal::Point { thing: 3, local: Vec3::ZERO, to: v3(want, 0.0, 0.0) });
    assert!(d.remaining < 1e-5, "{d:?}");
    assert_eq!(p.poses[4], bystander);
    assert!(d.report.residual <= 1e-7, "{d:?}");
    // The crank's where the formula says, for that piston position.
    let x = p.poses[1].x;
    let angle = x.y.atan2(x.x);
    assert!((piston_at(r, l, angle) - want).abs() < 1e-5, "{angle}");
    // Asked past the reach of crank and rod together, it stops at the end of its travel, without coming apart.
    let d = drag(&mut p, &Goal::Point { thing: 3, local: Vec3::ZERO, to: v3(100.0, 0.0, 0.0) });
    assert!(d.report.residual <= 1e-7, "{d:?}");
    assert!((p.poses[3].origin.x - (r + l)).abs() < 1e-2, "{:?}", p.poses[3].origin);
}

#[test]
fn a_flipped_frame_mates_facing_the_other_way() {
    // Frames facing each other, as two faces in contact are.
    let flipped = Frame::new(Vec3::ZERO, -Vec3::Z, Vec3::X);
    let mut p = pair(MateKind::Fastened);
    p.joints[0].on_b = flipped;
    solve(&mut p);
    assert!(p.poses[1].z.dist(-Vec3::Z) < 1e-7, "{:?}", p.poses[1]);
    // Starting exactly backward, with nothing to say which way to turn, it still comes round.
    let backward = Frame::new(Vec3::ZERO, -Vec3::Z, Vec3::X);
    for kind in [MateKind::Fastened, MateKind::Revolute, MateKind::Slider, MateKind::Planar, MateKind::Parallel] {
        let mut p = Problem { poses: vec![Frame::WORLD, backward], fixed: vec![true, false], joints: vec![joint(0, 1, kind)], ..Problem::default() };
        let r = solve(&mut p);
        assert!(r.failing.is_empty(), "{kind:?}: {r:?}");
        assert!(p.poses[1].z.dist(Vec3::Z) < 1e-7, "{kind:?}: {:?}", p.poses[1]);
    }
}

#[test]
fn a_long_chain_comes_together() {
    // 30 links with a revolute between each, thrown about at the start.
    let n = 30;
    let mut rng = 1u64;
    let mut next = || {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5
    };
    let mut poses = vec![Frame::WORLD];
    for _ in 0..n {
        poses.push(Frame::new(v3(next() * 100.0, next() * 100.0, next() * 100.0), v3(next(), next(), next() + 1.0), Vec3::X));
    }
    let link = Frame::translation(v3(10.0, 0.0, 0.0));
    let mut joints = vec![joint(0, 1, MateKind::Revolute)];
    for i in 1..n {
        joints.push(Joint { on_a: link, ..joint(i, i + 1, MateKind::Revolute) });
    }
    let mut p = Problem { poses, fixed: { let mut f = vec![false; n + 1]; f[0] = true; f }, joints, ..Problem::default() };
    let r = solve(&mut p);
    assert!(r.failing.is_empty(), "{r:?}");
    assert!(r.residual <= 1e-8, "{r:?}");
    assert_eq!(r.freedom, n);
    // Every link hangs from the one before, whatever way they ended up.
    for i in 2..=n {
        let end = p.poses[i - 1].to_world(v3(10.0, 0.0, 0.0));
        assert!(end.dist(p.poses[i].origin) < 1e-6);
    }
}

/// An arm on a revolute, with this many more parts fastened along it, each where its mate says.
fn arm_with_parts(parts: usize) -> Problem {
    let n = parts + 2;
    let mut p = Problem { poses: (0..n).map(|i| Frame::translation(v3(i as f64, 0.0, 0.0))).collect(), fixed: vec![false; n], ..Problem::default() };
    p.fixed[0] = true;
    p.joints.push(Joint { on_a: Frame::translation(v3(1.0, 0.0, 0.0)), ..joint(0, 1, MateKind::Revolute) });
    for i in 2..n {
        p.joints.push(Joint { on_a: Frame::translation(v3(i as f64 - 1.0, 0.0, 0.0)), ..joint(1, i, MateKind::Fastened) });
    }
    p
}

#[test]
fn parts_fastened_together_move_exactly_together() {
    let mut p = arm_with_parts(5);
    assert_eq!(solve(&mut p).freedom, 1);
    let before: Vec<Frame> = (2..7).map(|i| p.poses[1].inverse().compose(&p.poses[i])).collect();
    for k in 1..=8 {
        let a = k as f64 * 0.7;
        drag(&mut p, &Goal::Point { thing: 6, local: Vec3::ZERO, to: v3(1.0 + 5.0 * a.cos(), 5.0 * a.sin(), 0.0) });
    }
    for (i, was) in (2..7).zip(&before) {
        let now = p.poses[1].inverse().compose(&p.poses[i]);
        assert!(now.origin.dist(was.origin) < 1e-12 && now.x.dist(was.x) < 1e-12 && now.z.dist(was.z) < 1e-12, "{now:?}");
    }
    // The arm did go somewhere.
    assert!(p.poses[6].origin.dist(v3(6.0, 0.0, 0.0)) > 1.0);
}

#[test]
fn a_fastened_mate_that_stops_holding_is_brought_back() {
    let mut p = arm_with_parts(2);
    solve(&mut p);
    // Its connector has moved, as when a part changes, so it isn't one of those that simply move as one.
    p.joints[2].on_a = Frame::translation(v3(50.0, 0.0, 0.0));
    let r = solve(&mut p);
    assert!(r.failing.is_empty(), "{r:?}");
    assert!(p.poses[3].origin.dist(p.poses[1].to_world(v3(50.0, 0.0, 0.0))) < 1e-7, "{:?}", p.poses[3].origin);
    // Its neighbour, whose mate still holds, stays with the arm.
    assert!(p.poses[2].origin.dist(p.poses[1].to_world(v3(1.0, 0.0, 0.0))) < 1e-9);
    assert_eq!(r.freedom, 1);
}

#[test]
fn a_conflict_among_fastened_parts_is_still_named() {
    // A is fixed, B is fixed elsewhere, and C is on A already. C can't also be on B.
    let mut p = Problem {
        poses: vec![Frame::WORLD, Frame::translation(v3(100.0, 0.0, 0.0)), Frame::WORLD],
        fixed: vec![true, true, false],
        joints: vec![joint(0, 2, MateKind::Fastened), joint(1, 2, MateKind::Fastened)],
        ..Problem::default()
    };
    let r = solve(&mut p);
    assert_eq!(r.failing, vec![1], "{r:?}");
    assert!(p.poses[2].origin.norm() < 1e-9);
}

#[test]
fn a_thousand_fastened_parts_are_no_trouble() {
    let mut p = arm_with_parts(1000);
    let t = std::time::Instant::now();
    let r = solve(&mut p);
    assert_eq!(r.freedom, 1);
    let d = drag(&mut p, &Goal::Point { thing: 1001, local: Vec3::ZERO, to: v3(1.0, 1000.0, 0.0) });
    // Far more than it takes: solved one by one, this would take minutes.
    assert!(t.elapsed().as_secs_f64() < 5.0, "{:?}", t.elapsed());
    assert!(d.remaining < 1e-6, "{d:?}");
}

#[test]
fn a_part_going_to_a_mate_doesnt_turn_however_far_its_origin_is_from_the_mate() {
    // Modelled off to one side, as parts in one Part Studio are. Its connector is far from its own origin, and the revolute
    // leaves it free to turn, so nothing but this says it shouldn't. It's already square, so it has no need to.
    let on_b = Frame::translation(v3(100.0, 32.0, 4.0));
    let on_a = Frame::translation(v3(0.0, 16.0, 4.0));
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::WORLD], fixed: vec![true, false], joints: vec![Joint { on_a, on_b, ..joint(0, 1, MateKind::Revolute) }], ..Problem::default() };
    let r = solve(&mut p);
    assert!(r.failing.is_empty());
    assert!(p.poses[1].x.dist(Vec3::X) < 1e-9 && p.poses[1].z.dist(Vec3::Z) < 1e-9, "{:?}", p.poses[1]);
    assert!(p.poses[1].origin.dist(v3(-100.0, -16.0, 0.0)) < 1e-7, "{:?}", p.poses[1].origin);
    assert!(joint_value(&p, 0, Dof::Angle).abs() < 1e-9);
}

#[test]
fn a_free_part_dragged_by_its_origin_has_nothing_to_turn_for() {
    let mut p = Problem { poses: vec![Frame::translation(v3(10.0, 0.0, 0.0))], fixed: vec![false], ..Problem::default() };
    let d = drag(&mut p, &Goal::Point { thing: 0, local: Vec3::ZERO, to: v3(10.0, 50.0, 0.0) });
    assert!(d.remaining < 1e-7);
    assert!(p.poses[0].x.dist(Vec3::X) < 1e-9, "{:?}", p.poses[0]);
}

#[test]
fn a_part_that_has_to_turn_turns_about_where_its_held() {
    // A parallel mate only asks for the axes to line up, and its connector is far from the part's origin. So all that has to
    // happen is a turn, and the connector stays where it is.
    let on_b = Frame::new(v3(100.0, 50.0, 0.0), v3(0.3, 0.8, 0.5), Vec3::X);
    let mut p = Problem { poses: vec![Frame::WORLD, Frame::WORLD], fixed: vec![true, false], joints: vec![Joint { on_b, ..joint(0, 1, MateKind::Parallel) }], ..Problem::default() };
    let r = solve(&mut p);
    assert!(r.failing.is_empty());
    let held = p.poses[1].to_world(v3(100.0, 50.0, 0.0));
    assert!(held.dist(v3(100.0, 50.0, 0.0)) < 1e-6, "{held:?}");
    assert!(p.poses[1].compose(&on_b).z.dist(Vec3::Z) < 1e-7);
}
