//! Milestone 2: Booleans on planar solids, including the degenerate contacts.

use coreshape::*;

fn boxed(x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64) -> Body {
    box_solid(&Frame::translation(v3(x, y, z)), dx, dy, dz, 1).unwrap()
}

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-8 * b.abs().max(1.0), "{what}: got {a}, expected {b}");
}

/// Checks all three operations against the inclusion-exclusion identities.
fn check_identities(a: &Body, b: &Body, name: &str) {
    let u = union(a, b).unwrap_or_else(|e| panic!("{name} union: {e}"));
    let i = intersect(a, b).unwrap_or_else(|e| panic!("{name} intersect: {e}"));
    let d = subtract(a, b).unwrap_or_else(|e| panic!("{name} subtract: {e}"));
    let (va, vb, vu, vi, vd) = (vol(a), vol(b), vol(&u), vol(&i), vol(&d));
    assert!((vu + vi - va - vb).abs() < 1e-7, "{name}: union {vu} + intersect {vi} != {va} + {vb}");
    assert!((vd + vi - va).abs() < 1e-7, "{name}: subtract {vd} + intersect {vi} != {va}");
    for (label, body) in [("union", &u), ("intersect", &i), ("subtract", &d)] {
        if !body.faces.is_empty() {
            let m = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).unwrap_or_else(|e| panic!("{name} {label}: {e}"));
            assert_eq!(m.open_edge_count(), 0, "{name} {label}: open mesh");
        }
    }
}

#[test]
fn overlapping_boxes_general_position() {
    let a = boxed(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    let b = boxed(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 15.0, "union");
    let r = validate(&u).unwrap();
    assert_eq!(r.shells, 1);
    assert_eq!(r.genus, vec![0]);
    near(vol(&intersect(&a, &b).unwrap()), 1.0, "intersect");
    near(vol(&subtract(&a, &b).unwrap()), 7.0, "subtract");
    check_identities(&a, &b, "general");
}

#[test]
fn disjoint_boxes() {
    let a = boxed(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let b = boxed(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 2.0, "union");
    assert_eq!(validate(&u).unwrap().shells, 2);
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
    near(vol(&subtract(&a, &b).unwrap()), 1.0, "subtract");
}

#[test]
fn nested_box_makes_a_void() {
    let a = boxed(0.0, 0.0, 0.0, 4.0, 4.0, 4.0);
    let b = boxed(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    let d = subtract(&a, &b).unwrap();
    near(vol(&d), 63.0, "void");
    assert_eq!(validate(&d).unwrap().shells, 2);
    near(vol(&union(&a, &b).unwrap()), 64.0, "union");
    near(vol(&intersect(&a, &b).unwrap()), 1.0, "intersect");
    assert!(subtract(&b, &a).unwrap().faces.is_empty());
}

#[test]
fn identical_boxes() {
    let a = boxed(0.0, 0.0, 0.0, 2.0, 3.0, 4.0);
    near(vol(&union(&a, &a).unwrap()), 24.0, "union");
    near(vol(&intersect(&a, &a).unwrap()), 24.0, "intersect");
    assert!(subtract(&a, &a).unwrap().faces.is_empty());
    let r = validate(&union(&a, &a).unwrap()).unwrap();
    assert_eq!((r.vertices, r.edges, r.faces), (8, 12, 6));
}

#[test]
fn boxes_sharing_a_whole_face() {
    let a = boxed(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let b = boxed(1.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 2.0, "union");
    // The shared wall vanishes and the flush sides merge back into one box.
    let r = validate(&u).unwrap();
    assert_eq!((r.vertices, r.edges, r.faces), (8, 12, 6));
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
    near(vol(&subtract(&a, &b).unwrap()), 1.0, "subtract");
}

#[test]
fn boxes_sharing_part_of_a_face() {
    let a = boxed(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    let b = boxed(2.0, 0.5, 0.5, 2.0, 1.0, 1.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 10.0, "union");
    assert_eq!(validate(&u).unwrap().genus, vec![0]);
    check_identities(&a, &b, "partial face");
}

#[test]
fn flush_coplanar_overlap() {
    // Tops are coplanar, and the bodies overlap.
    let a = boxed(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    let b = boxed(1.0, 1.0, 1.0, 2.0, 2.0, 1.0);
    near(vol(&union(&a, &b).unwrap()), 8.0 + 4.0 - 1.0, "union");
    check_identities(&a, &b, "flush tops");
}

#[test]
fn inset_pocket_flush_with_the_top() {
    let a = boxed(0.0, 0.0, 0.0, 4.0, 4.0, 2.0);
    let b = boxed(1.0, 1.0, 1.0, 2.0, 2.0, 1.0);
    let d = subtract(&a, &b).unwrap();
    near(vol(&d), 28.0, "pocket");
    assert_eq!(validate(&d).unwrap().genus, vec![0]);
    check_identities(&a, &b, "pocket");
}

#[test]
fn through_hole_makes_genus_one() {
    let a = boxed(0.0, 0.0, 0.0, 4.0, 4.0, 2.0);
    let b = boxed(1.0, 1.0, -1.0, 2.0, 2.0, 4.0);
    let d = subtract(&a, &b).unwrap();
    near(vol(&d), 24.0, "hole");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
    check_identities(&a, &b, "through hole");
}

#[test]
fn through_hole_with_flush_ends() {
    let a = boxed(0.0, 0.0, 0.0, 4.0, 4.0, 2.0);
    let b = boxed(1.0, 1.0, 0.0, 2.0, 2.0, 2.0);
    let d = subtract(&a, &b).unwrap();
    near(vol(&d), 24.0, "hole");
    assert_eq!(validate(&d).unwrap().genus, vec![1]);
    check_identities(&a, &b, "flush through hole");
}

#[test]
fn cut_in_two_gives_two_shells() {
    let a = boxed(0.0, 0.0, 0.0, 6.0, 2.0, 2.0);
    let b = boxed(2.0, -1.0, -1.0, 2.0, 4.0, 4.0);
    let d = subtract(&a, &b).unwrap();
    near(vol(&d), 16.0, "split");
    assert_eq!(validate(&d).unwrap().shells, 2);
    check_identities(&a, &b, "split");
}

#[test]
fn boxes_touching_along_an_edge() {
    let a = boxed(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let b = boxed(1.0, 1.0, 0.0, 1.0, 1.0, 1.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 2.0, "edge touch");
    // Non-manifold contact becomes a manifold body with a duplicated edge.
    let r = validate(&u).unwrap();
    assert_eq!(r.shells, 2);
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
    near(vol(&subtract(&a, &b).unwrap()), 1.0, "subtract");
}

#[test]
fn boxes_touching_at_a_vertex() {
    let a = boxed(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let b = boxed(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    let u = union(&a, &b).unwrap();
    near(vol(&u), 2.0, "vertex touch");
    assert_eq!(validate(&u).unwrap().shells, 2);
    assert!(intersect(&a, &b).unwrap().faces.is_empty());
    near(vol(&subtract(&a, &b).unwrap()), 1.0, "subtract");
}

#[test]
fn edge_touching_a_face_interior() {
    // A diamond prism lying along Y with its lower edge resting on the box's top.
    let a = boxed(0.0, 0.0, 0.0, 4.0, 4.0, 1.0);
    let sketch = Frame {
        origin: v3(0.0, 1.0, 0.0),
        x: Vec3::X,
        y: Vec3::Z,
        z: -Vec3::Y,
    };
    let w = Wire::polygon(&[v2(2.0, 1.0), v2(3.0, 2.0), v2(2.0, 3.0), v2(1.0, 2.0)]);
    let b = extrude(&Profile::new(w), &sketch, -2.0, 2).unwrap();
    near(vol(&b), 4.0, "prism");
    check_identities(&a, &b, "edge on face");
    near(vol(&union(&a, &b).unwrap()), 20.0, "union");
}

#[test]
fn vertex_resting_on_a_face_interior() {
    // A unit box standing on one corner, with that corner resting on a plate.
    let a = boxed(-5.0, -5.0, -1.0, 10.0, 10.0, 1.0);
    let r = Frame::rotation(Vec3::ZERO, v3(1.0, -1.0, 0.0), (1.0f64 / 3.0f64.sqrt()).acos());
    let b = box_solid(&r, 1.0, 1.0, 1.0, 2).unwrap();
    let lowest = b.vertices.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    assert!(lowest.abs() < 1e-12, "box should stand on its corner, lowest {lowest}");
    check_identities(&a, &b, "vertex on face");
}

#[test]
fn staircase_chain() {
    let mut s = boxed(0.0, 0.0, 0.0, 1.0, 5.0, 1.0);
    for i in 1..8 {
        let step = boxed(i as f64, 0.0, 0.0, 1.0, 5.0, 1.0 + i as f64);
        s = union(&s, &step).unwrap();
        validate(&s).unwrap();
    }
    let expected: f64 = (0..8).map(|i| 5.0 * (1.0 + i as f64)).sum();
    near(vol(&s), expected, "staircase");
    assert_eq!(validate(&s).unwrap().shells, 1);
}

#[test]
fn comb_of_slots() {
    let mut s = boxed(0.0, 0.0, 0.0, 22.0, 10.0, 5.0);
    for i in 0..5 {
        let slot = boxed(2.0 + 4.0 * i as f64, -1.0, 2.0, 2.0, 12.0, 4.0);
        s = subtract(&s, &slot).unwrap();
    }
    near(vol(&s), 22.0 * 10.0 * 5.0 - 5.0 * 2.0 * 10.0 * 3.0, "comb");
    assert_eq!(validate(&s).unwrap().genus, vec![0]);
}
