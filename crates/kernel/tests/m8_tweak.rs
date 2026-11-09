//! Offset, shell and draft, checked against closed forms.

use coreshape::*;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol, "{what}: got {a}, want {b}");
}

fn closed(b: &Body) {
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    validate(b).unwrap();
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::translation(v3(x, y, z))
}

fn face_where(b: &Body, slot: u32) -> usize {
    b.faces.iter().position(|f| f.tag.slot == slot).unwrap()
}

// ----- Offset -----

#[test]
fn a_box_offsets_both_ways() {
    let b = box_solid(&Frame::WORLD, 20.0, 30.0, 40.0, 1).unwrap();
    let out = offset(&b, 2.0).unwrap();
    close(vol(&out), 24.0 * 34.0 * 44.0, 1e-9, "out");
    let inn = offset(&b, -2.0).unwrap();
    close(vol(&inn), 16.0 * 26.0 * 36.0, 1e-9, "in");
    closed(&out);
    closed(&inn);
}

#[test]
fn curved_primitives_offset_exactly() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    close(vol(&offset(&c, 2.0).unwrap()), PI * 144.0 * 24.0, 1e-8, "cylinder out");
    close(vol(&offset(&c, -2.0).unwrap()), PI * 64.0 * 16.0, 1e-8, "cylinder in");
    let s = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    close(vol(&offset(&s, 3.0).unwrap()), 4.0 / 3.0 * PI * 2197.0, 1e-7, "sphere out");
    close(vol(&offset(&s, -3.0).unwrap()), 4.0 / 3.0 * PI * 343.0, 1e-7, "sphere in");
    let t = torus(&Frame::WORLD, 20.0, 5.0, 1).unwrap();
    close(vol(&offset(&t, 1.0).unwrap()), 2.0 * PI * PI * 20.0 * 36.0, 1e-7, "torus out");
    close(vol(&offset(&t, -1.0).unwrap()), 2.0 * PI * PI * 20.0 * 16.0, 1e-7, "torus in");
    // A cone's walls move square to themselves, and its caps along the axis.
    let cone = cone_frustum(&Frame::WORLD, 10.0, 5.0, 20.0, 1).unwrap();
    let d = 1.0;
    let slope = (5.0f64 / 20.0).atan();
    let shift = d / slope.cos();
    let (r0, r1) = (10.0 + shift + d * slope.tan(), 5.0 + shift - d * slope.tan());
    let want = PI * 22.0 / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
    close(vol(&offset(&cone, d).unwrap()), want, 1e-7, "cone out");
}

#[test]
fn a_hole_shrinks_when_the_part_grows() {
    let plate = extrude(&Profile::new(Wire::rect(0.0, 0.0, 40.0, 40.0)).with_hole(Wire::circle(v2(20.0, 20.0), 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let grown = offset(&plate, 1.0).unwrap();
    close(vol(&grown), (42.0 * 42.0 - PI * 49.0) * 12.0, 1e-8, "plate");
    assert_eq!(validate(&grown).unwrap().genus, vec![1]);
    closed(&grown);
}

#[test]
fn a_filleted_box_keeps_its_smooth_edges() {
    // Rounded on every edge: planes, cylinders and sphere patches, all tangent.
    let r = 3.0;
    let b = box_solid(&Frame::WORLD, 20.0, 20.0, 20.0, 1).unwrap();
    let all: Vec<usize> = (0..b.edges.len()).collect();
    let round = fillet(&b, &all, r, 2).unwrap();
    let rounded = |side: f64, radius: f64| {
        let flat = side - 2.0 * radius;
        flat.powi(3) + 6.0 * flat * flat * radius + 3.0 * flat * PI * radius * radius + 4.0 / 3.0 * PI * radius.powi(3)
    };
    close(vol(&round), rounded(20.0, r), 1e-6, "start");
    let out = offset(&round, 1.0).unwrap();
    close(vol(&out), rounded(22.0, r + 1.0), 1e-6, "grown");
    closed(&out);
    let inn = offset(&round, -1.0).unwrap();
    close(vol(&inn), rounded(18.0, r - 1.0), 1e-6, "shrunk");
    closed(&inn);
    // Going in further than the radius, the rounds vanish, and what's left is a plain box.
    let past = offset(&round, -3.5).unwrap();
    close(vol(&past), 13.0 * 13.0 * 13.0, 1e-6, "past the rounds");
    closed(&past);
}

#[test]
fn a_rounded_prism_offsets_along_its_tangent_edges() {
    let p = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let area = |w: f64, h: f64, r: f64| w * h - (4.0 - PI) * r * r;
    close(vol(&offset(&p, 2.0).unwrap()), area(64.0, 44.0, 10.0) * 14.0, 1e-7, "out");
    close(vol(&offset(&p, -2.0).unwrap()), area(56.0, 36.0, 6.0) * 6.0, 1e-7, "in");
}

#[test]
fn traced_edges_follow_along() {
    // A hole through a cylinder's wall: its edge has no closed form.
    let a = cylinder(&at(0.0, 0.0, -20.0), 8.0, 40.0, 1).unwrap();
    let b = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
    let joined = union(&a, &b).unwrap();
    let d = 0.5;
    let grown = offset(&joined, d).unwrap();
    closed(&grown);
    // Against a union built directly from the grown pieces.
    let a2 = cylinder(&at(0.0, 0.0, -20.0 - d), 8.0 + d, 40.0 + 2.0 * d, 1).unwrap();
    let b2 = cylinder(&Frame::new(v3(-20.0 - d, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0 + d, 40.0 + 2.0 * d, 2).unwrap();
    close(vol(&grown), vol(&union(&a2, &b2).unwrap()), 1e-4, "traced");
}

#[test]
fn faces_can_move_by_different_distances() {
    let b = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    let mut d = vec![0.0; 6];
    d[face_where(&b, 1)] = 5.0; // the top
    let r = offset_faces(&b, &d).unwrap();
    close(vol(&r), 1500.0, 1e-9, "taller");
    assert!(offset_faces(&b, &[1.0]).is_err());
    assert!(offset_faces(&b, &[f64::NAN; 6]).is_err());
    assert!(offset(&b, 1e-9).is_err());
}

#[test]
fn too_big_an_offset_is_refused() {
    let b = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    assert!(offset(&b, -6.0).is_err());
    let s = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    assert!(offset(&s, -11.0).is_err());
    let plate = extrude(&Profile::new(Wire::rect(0.0, 0.0, 40.0, 40.0)).with_hole(Wire::circle(v2(20.0, 20.0), 3.0)), &Frame::WORLD, 10.0, 1).unwrap();
    // The hole closes, which is what moving out by more than its radius does.
    let past = offset(&plate, 3.5).unwrap();
    assert_eq!(past.faces.len(), 6);
    let plain = extrude(&Profile::new(Wire::rect(0.0, 0.0, 40.0, 40.0)), &Frame::WORLD, 10.0, 1).unwrap();
    close(vol(&past), vol(&offset(&plain, 3.5).unwrap()), 1e-9, "the hole closed");
}

// ----- Shell -----

#[test]
fn a_closed_shell_is_a_box_with_a_void() {
    let b = box_solid(&Frame::WORLD, 20.0, 30.0, 40.0, 1).unwrap();
    let s = shell(&b, &[], 2.0, 9).unwrap();
    close(vol(&s), 24000.0 - 16.0 * 26.0 * 36.0, 1e-8, "walls");
    assert_eq!(validate(&s).unwrap().shells, 2);
    assert_eq!(s.faces.iter().filter(|f| f.tag.feature == 9).count(), 6, "the inner faces are named");
    closed(&s);
}

#[test]
fn an_open_top_makes_a_tray() {
    let b = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
    let s = shell(&b, &[face_where(&b, 1)], 2.0, 9).unwrap();
    close(vol(&s), 24000.0 - 36.0 * 26.0 * 18.0, 1e-8, "tray");
    assert_eq!(validate(&s).unwrap().shells, 1);
    closed(&s);
    // Two faces open: a sleeve.
    let t = shell(&b, &[face_where(&b, 0), face_where(&b, 1)], 2.0, 9).unwrap();
    close(vol(&t), 24000.0 - 36.0 * 26.0 * 20.0, 1e-8, "sleeve");
    assert_eq!(validate(&t).unwrap().genus, vec![1]);
}

#[test]
fn curved_parts_shell_too() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    let cup = shell(&c, &[face_where(&c, 1)], 1.5, 9).unwrap();
    close(vol(&cup), PI * 100.0 * 20.0 - PI * 8.5 * 8.5 * 18.5, 1e-7, "cup");
    closed(&cup);
    let ball = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    let hollow = shell(&ball, &[], 2.0, 9).unwrap();
    close(vol(&hollow), 4.0 / 3.0 * PI * (1000.0 - 512.0), 1e-7, "ball");
    // A rounded tray, open on top: its corners are tangent all the way round.
    let tray = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 15.0, 1).unwrap();
    let s = shell(&tray, &[face_where(&tray, 1)], 2.0, 9).unwrap();
    let area = |w: f64, h: f64, r: f64| w * h - (4.0 - PI) * r * r;
    close(vol(&s), area(60.0, 40.0, 8.0) * 15.0 - area(56.0, 36.0, 6.0) * 13.0, 1e-6, "rounded tray");
    closed(&s);
}

#[test]
fn walls_that_do_not_fit_are_refused() {
    let thin = box_solid(&Frame::WORLD, 40.0, 40.0, 3.0, 1).unwrap();
    assert!(shell(&thin, &[], 2.0, 9).is_err());
    assert!(shell(&thin, &[99], 0.5, 9).is_err());
    assert!(shell(&thin, &[], 0.0, 9).is_err());
    assert!(shell(&thin, &[], f64::NAN, 9).is_err());
}

// ----- Draft -----

fn frustum(a0: f64, a1: f64, h: f64) -> f64 {
    // Prismatoid with square-root mean, for similar sections.
    h / 3.0 * (a0 + a1 + (a0 * a1).sqrt())
}

#[test]
fn a_box_tapers_from_its_neutral_plane() {
    let b = box_solid(&at(-10.0, -10.0, 0.0), 20.0, 20.0, 15.0, 1).unwrap();
    let sides: Vec<usize> = (0..b.faces.len()).filter(|&f| b.faces[f].tag.slot >= 2).collect();
    let angle = 5f64.to_radians();
    let t = draft(&b, &sides, Vec3::Z, Vec3::ZERO, angle).unwrap();
    closed(&t);
    let top = 20.0 - 2.0 * 15.0 * angle.tan();
    close(vol(&t), frustum(400.0, top * top, 15.0), 1e-8, "narrower going up");
    // The other way round it widens, and the neutral plane stays put.
    let w = draft(&b, &sides, Vec3::Z, Vec3::ZERO, -angle).unwrap();
    let top = 20.0 + 2.0 * 15.0 * angle.tan();
    close(vol(&w), frustum(400.0, top * top, 15.0), 1e-8, "wider going up");
    let bounds = t.aabb();
    assert!((bounds.max.x - 10.0).abs() < 1e-6, "{bounds:?}");
    // With the neutral plane at the top, the base grows.
    let n = draft(&b, &sides, Vec3::Z, v3(0.0, 0.0, 15.0), angle).unwrap();
    let base = 20.0 + 2.0 * 15.0 * angle.tan();
    close(vol(&n), frustum(base * base, 400.0, 15.0), 1e-8, "neutral on top");
}

#[test]
fn one_face_can_be_drafted_alone() {
    let b = box_solid(&Frame::WORLD, 20.0, 10.0, 15.0, 1).unwrap();
    // The face at x = 20 is swept from the rectangle's second side.
    let side = face_where(&b, 3);
    let angle = 10f64.to_radians();
    let t = draft(&b, &[side], Vec3::Z, Vec3::ZERO, angle).unwrap();
    close(vol(&t), 3000.0 - 10.0 * 0.5 * 15.0 * 15.0 * angle.tan(), 1e-8, "wedge");
    closed(&t);
}

#[test]
fn a_cylinder_becomes_a_cone_and_a_hole_widens_the_other_way() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    let wall: Vec<usize> = (0..c.faces.len()).filter(|&f| matches!(c.faces[f].surface, Surface::Cylinder { .. })).collect();
    let angle = 3f64.to_radians();
    let t = draft(&c, &wall, Vec3::Z, Vec3::ZERO, angle).unwrap();
    let r1 = 10.0 - 20.0 * angle.tan();
    close(vol(&t), PI * 20.0 / 3.0 * (100.0 + 10.0 * r1 + r1 * r1), 1e-7, "cone");
    assert!(t.faces.iter().any(|f| matches!(f.surface, Surface::Cone { .. })));
    closed(&t);
    let plate = extrude(&Profile::new(Wire::rect(-20.0, -20.0, 40.0, 40.0)).with_hole(Wire::circle(v2(0.0, 0.0), 6.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let hole: Vec<usize> = (0..plate.faces.len()).filter(|&f| matches!(plate.faces[f].surface, Surface::Cylinder { .. })).collect();
    let t = draft(&plate, &hole, Vec3::Z, Vec3::ZERO, angle).unwrap();
    let r1 = 6.0 + 10.0 * angle.tan();
    close(vol(&t), 16000.0 - PI * 10.0 / 3.0 * (36.0 + 6.0 * r1 + r1 * r1), 1e-7, "hole");
    closed(&t);
}

#[test]
fn a_rounded_prism_drafts_with_tangent_corners() {
    let p = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let walls: Vec<usize> = (0..p.faces.len()).filter(|&f| p.faces[f].tag.slot >= 2).collect();
    let angle = 4f64.to_radians();
    let t = draft(&p, &walls, Vec3::Z, Vec3::ZERO, angle).unwrap();
    closed(&t);
    // A section at height z is the same rounded rectangle, inset by z tan(angle). Integrate it.
    let area = |d: f64| (60.0 - 2.0 * d) * (40.0 - 2.0 * d) - (4.0 - PI) * (8.0 - d).powi(2);
    let n = 2000;
    let mut want = 0.0;
    for i in 0..n {
        let z = (i as f64 + 0.5) * 10.0 / n as f64;
        want += area(z * angle.tan()) * 10.0 / n as f64;
    }
    close(vol(&t), want, 1e-4, "rounded taper");
}

#[test]
fn bad_drafts_say_why() {
    let b = box_solid(&Frame::WORLD, 20.0, 20.0, 15.0, 1).unwrap();
    let side = face_where(&b, 3);
    let top = face_where(&b, 1);
    let cases: Vec<(Vec<usize>, Vec3, f64, &str)> = vec![
        (vec![top], Vec3::Z, 0.1, "along the pull"),
        (vec![side], Vec3::X, 0.1, "along the pull"),
        (vec![side], Vec3::ZERO, 0.1, "non-zero"),
        (vec![side], Vec3::Z, 0.0, "out of range"),
        (vec![side], Vec3::Z, 1.5, "out of range"),
        (vec![99], Vec3::Z, 0.1, "no face"),
    ];
    for (faces, pull, angle, expected) in cases {
        let e = draft(&b, &faces, pull, Vec3::ZERO, angle).unwrap_err().to_string();
        assert!(e.contains(expected), "{faces:?}: {e}");
    }
    let s = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    assert!(draft(&s, &[0], Vec3::Z, Vec3::ZERO, 0.1).is_err());
    // Half of a smooth pair.
    let p = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let plane = (0..p.faces.len()).find(|&f| p.faces[f].tag.slot >= 2 && matches!(p.faces[f].surface, Surface::Plane(_))).unwrap();
    let e = draft(&p, &[plane], Vec3::Z, Vec3::ZERO, 0.05).unwrap_err().to_string();
    assert!(e.contains("smoothly"), "{e}");
}
