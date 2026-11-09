//! Sheet bodies: open surfaces that are bodies of their own. Each result is checked by a formula, and anything that holds for
//! solids has to go on holding for them.

use coreshape::boolean::{trim_sheet, trim_sheet_by_plane, Keep};
use coreshape::tess::open_only_at_boundary;
use coreshape::topo::Coedge;
use coreshape::*;
use std::f64::consts::{PI, TAU};

fn area(b: &Body) -> f64 {
    mass_properties(b).area
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn mesh(b: &Body) -> Mesh {
    tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap()
}

/// A sheet's mesh matches the true area to within what chords lose, and is torn only at the boundary.
fn check_sheet(b: &Body, want_area: f64) {
    assert!(b.sheet, "this should be a sheet");
    validate(b).unwrap();
    near(area(b), want_area, "area");
    let m = mesh(b);
    assert!(open_only_at_boundary(b, &m), "{} open mesh edges away from the boundary", m.open_edge_count());
    assert!((m.area() - want_area).abs() < 2e-2 * want_area.max(1.0), "the mesh's area {} isn't {want_area}", m.area());
}

fn rect(w: f64, h: f64) -> Profile {
    Profile::new(Wire::rect(0.0, 0.0, w, h))
}

fn line(a: Vec2, b: Vec2, id: u32) -> Seg {
    Seg { geom: SegGeom::Line { a, b }, id }
}

/// A flat 10 by 20 sheet on the XY plane, facing up.
fn plate() -> Body {
    fill(&rect(10.0, 20.0), &Frame::WORLD, 1).unwrap()
}

// ----- What a sheet is -----

#[test]
fn a_filled_region_is_a_flat_sheet_facing_along_its_plane() {
    let s = plate();
    check_sheet(&s, 200.0);
    assert_eq!(s.faces.len(), 1);
    assert_eq!(s.boundary_edges().len(), 4);
    // Up, and with no volume.
    assert!(s.faces[0].outward_normal(v3(1.0, 1.0, 0.0)).dist(Vec3::Z) < 1e-12);
    assert_eq!(mass_properties(&s).volume, 0.0);
    // It's one surface with one boundary loop, so a disc: genus 0.
    assert_eq!(validate(&s).unwrap().genus, vec![0]);
    assert!(s.vertices.iter().all(|p| p.z == 0.0));
}

#[test]
fn a_filled_region_keeps_its_holes_and_curves() {
    let p = Profile::new(Wire::circle(v2(0.0, 0.0), 10.0)).with_hole(Wire::circle(v2(2.0, 0.0), 3.0).reversed());
    let frame = Frame::new(v3(1.0, 2.0, 3.0), v3(0.0, 0.0, 1.0), v3(1.0, 0.0, 0.0));
    let s = fill(&p, &frame, 1).unwrap();
    check_sheet(&s, PI * (100.0 - 9.0));
    assert_eq!(s.faces[0].loops.len(), 2);
    assert!(s.faces[0].outward_normal(v3(0.0, 0.0, 3.0)).dist(Vec3::Z) < 1e-12);
}

#[test]
fn the_strict_check_still_catches_a_solid_with_a_hole_in_it() {
    // The one thing that must not change: a body not declared a sheet that has an edge used once fails, as it always has.
    let solid = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    let mut broken = solid.clone();
    broken.faces.pop();
    broken.compute_shells();
    let why = validate(&broken).unwrap_err().join("; ");
    assert!(why.contains("used by 1 coedges"), "{why}");
    // Declared as one, the same faces are fine, as a box with no lid.
    broken.sheet = true;
    validate(&broken).unwrap();
    assert_eq!(broken.boundary_edges().len(), 4);
    assert!(validate(&solid).is_ok());
    assert!(solid.boundary_edges().is_empty());
    assert!(!solid.sheet);
}

#[test]
fn a_sheet_with_something_wrong_with_it_is_refused() {
    // An edge used twice the same way.
    let mut s = Body { sheet: true, ..Body::default() };
    let (a, b, c, d) = (s.add_vertex(v3(0.0, 0.0, 0.0)), s.add_vertex(v3(1.0, 0.0, 0.0)), s.add_vertex(v3(1.0, 1.0, 0.0)), s.add_vertex(v3(0.0, -1.0, 0.0)));
    let line = |s: &mut Body, p: usize, q: usize| {
        let (u, v) = (s.vertices[p], s.vertices[q]);
        s.add_edge(Curve::line_through(u, v), 0.0, u.dist(v), p, q)
    };
    let (ab, bc, ca, bd, da) = (line(&mut s, a, b), line(&mut s, b, c), line(&mut s, c, a), line(&mut s, b, d), line(&mut s, d, a));
    let plane = Surface::Plane(Frame::WORLD);
    s.add_face(plane.clone(), false, vec![vec![ab, bc, ca]], FaceTag::default());
    s.add_face(plane.clone(), false, vec![vec![ab, bd, da]], FaceTag::default());
    s.compute_shells();
    let why = validate(&s).unwrap_err().join("; ");
    assert!(why.contains("same direction"), "{why}");

    // A pinch: two triangles that share only a point. They're one vertex here, and each has its own fan.
    let mut p = Body { sheet: true, ..Body::default() };
    let (o, a1, a2, b1, b2) = (p.add_vertex(v3(0.0, 0.0, 0.0)), p.add_vertex(v3(1.0, 0.0, 0.0)), p.add_vertex(v3(1.0, 1.0, 0.0)), p.add_vertex(v3(-1.0, 0.0, 0.0)), p.add_vertex(v3(-1.0, -1.0, 0.0)));
    let (e1, e2, e3) = (line(&mut p, o, a1), line(&mut p, a1, a2), line(&mut p, a2, o));
    let (e4, e5, e6) = (line(&mut p, o, b1), line(&mut p, b1, b2), line(&mut p, b2, o));
    p.add_face(plane.clone(), false, vec![vec![e1, e2, e3]], FaceTag::default());
    p.add_face(plane.clone(), false, vec![vec![e4, e5, e6]], FaceTag::default());
    p.compute_shells();
    let why = validate(&p).unwrap_err().join("; ");
    assert!(why.contains("pinch"), "{why}");

    // A face with three edges used three times.
    let mut t = plate();
    let extra = t.faces[0].clone();
    t.faces.push(extra.clone());
    t.faces.push(extra);
    t.compute_shells();
    assert!(validate(&t).unwrap_err().join("; ").contains("not 1 or 2"));

    // A list of shells that leaves a face out.
    let mut l = plate();
    l.shells.clear();
    assert!(validate(&l).unwrap_err().join("; ").contains("shells list 0 faces"));
}

#[test]
fn a_shell_miscounted_by_euler_is_refused() {
    // Two separate squares called one shell: V=8, E=8, F=2, so V-E+F=2, with two boundary loops makes an impossible surface.
    let a = fill(&rect(1.0, 1.0), &Frame::WORLD, 1).unwrap();
    let b = fill(&rect(1.0, 1.0), &Frame::translation(v3(5.0, 0.0, 0.0)), 2).unwrap();
    let mut both = Body::side_by_side(&[&a, &b]);
    validate(&both).unwrap();
    both.shells = vec![vec![0, 1]];
    let why = validate(&both).unwrap_err().join("; ");
    assert!(why.contains("Euler"), "{why}");
}

// ----- Solids and sheets side by side -----

#[test]
fn side_by_side_with_a_solid_counts_area_for_all_and_volume_for_the_solid() {
    let s = plate();
    let solid = box_solid(&Frame::translation(v3(100.0, 0.0, 0.0)), 2.0, 3.0, 4.0, 2).unwrap();
    let both = Body::side_by_side(&[&solid, &s]);
    assert!(both.sheet);
    validate(&both).unwrap();
    let m = mass_properties(&both);
    near(m.volume, 24.0, "volume");
    near(m.area, 200.0 + 2.0 * (6.0 + 8.0 + 12.0), "area");
    // A solid alone is untouched.
    let alone = Body::side_by_side(&[&solid]);
    assert!(!alone.sheet);
    assert_eq!(mass_properties(&alone).volume.to_bits(), mass_properties(&solid).volume.to_bits());
}

#[test]
fn a_sheet_comes_apart_by_its_shells() {
    let (a, b) = (plate(), fill(&rect(3.0, 3.0), &Frame::translation(v3(50.0, 0.0, 0.0)), 2).unwrap());
    let both = Body::side_by_side(&[&a, &b]);
    assert_eq!(both.solid_count(), 2);
    let pieces = both.solids();
    assert_eq!(pieces.len(), 2);
    assert!(pieces.iter().all(|p| p.sheet));
    near(area(&pieces[0]), 200.0, "first");
    near(area(&pieces[1]), 9.0, "second");
}

#[test]
fn a_sheet_moves_and_reflects_as_any_body_does() {
    let s = plate();
    let moved = s.transformed(&Frame::rotation(Vec3::ZERO, Vec3::X, PI / 2.0));
    check_sheet(&moved, 200.0);
    assert!(moved.faces[0].outward_normal(v3(1.0, 0.0, 1.0)).dist(-Vec3::Y) < 1e-12);
    let m = s.mirrored(&Mirror::new(Vec3::ZERO, Vec3::Z));
    check_sheet(&m, 200.0);
    assert!(m.faces[0].outward_normal(v3(1.0, 1.0, 0.0)).dist(-Vec3::Z) < 1e-12);
}

// ----- What takes a sheet, and what refuses one -----

#[test]
fn what_needs_a_solid_says_so() {
    let s = plate();
    let solid = box_solid(&Frame::WORLD, 1.0, 1.0, 1.0, 2).unwrap();
    let mut said: Vec<String> = Vec::new();
    said.push(union(&s, &solid).unwrap_err().to_string());
    said.push(subtract(&solid, &s).unwrap_err().to_string());
    said.push(intersect(&s, &s).unwrap_err().to_string());
    said.push(fillet(&s, &[0], 1.0, 3).unwrap_err().to_string());
    said.push(chamfer(&s, &[0], 1.0, 3).unwrap_err().to_string());
    said.push(offset(&s, 1.0).unwrap_err().to_string());
    said.push(shell(&s, &[], 1.0, 3).unwrap_err().to_string());
    said.push(draft(&s, &[0], Vec3::Z, Vec3::ZERO, 0.1).unwrap_err().to_string());
    said.push(delete_faces(&s, &[0]).unwrap_err().to_string());
    for why in said {
        assert!(why.contains("needs a solid") && why.contains("sheet"), "{why}");
    }
}

// ----- Copying faces -----

#[test]
fn faces_copied_out_of_a_solid_leave_it_alone() {
    let solid = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let top = (0..solid.faces.len()).find(|&f| solid.faces[f].tag.slot == 1).unwrap();
    let side = (0..solid.faces.len()).find(|&f| solid.faces[f].tag.slot == 2).unwrap();
    let copy = sheet_from_faces(&solid, &[top]).unwrap();
    check_sheet(&copy, 200.0);
    assert_eq!(copy.faces[0].tag, solid.faces[top].tag);
    assert!(!solid.sheet);
    validate(&solid).unwrap();
    // Two that meet share their edge, as one sheet.
    let two = sheet_from_faces(&solid, &[top, side]).unwrap();
    check_sheet(&two, 200.0 + 300.0);
    assert_eq!(two.shells.len(), 1);
    assert_eq!(two.boundary_edges().len(), 6);
    // All of them make a closed sheet, which is still a sheet, and does enclose something.
    let all: Vec<usize> = (0..solid.faces.len()).collect();
    let closed = sheet_from_faces(&solid, &all).unwrap();
    assert!(closed.sheet);
    assert!(closed.boundary_edges().is_empty());
    near(mass_properties(&closed).volume, 6000.0, "a closed sheet's volume");
    // Asking wrongly.
    assert!(sheet_from_faces(&solid, &[]).is_err());
    assert!(sheet_from_faces(&solid, &[99]).is_err());
}

/// A flat triangle as a sheet, facing the way its corners wind.
fn triangle(a: Vec3, b: Vec3, c: Vec3, feature: u32) -> Body {
    let mut s = Body { sheet: true, ..Body::default() };
    let (i, j, k) = (s.add_vertex(a), s.add_vertex(b), s.add_vertex(c));
    let side = |s: &mut Body, p: usize, q: usize| {
        let (u, v) = (s.vertices[p], s.vertices[q]);
        s.add_edge(Curve::line_through(u, v), 0.0, u.dist(v), p, q)
    };
    let (ab, bc, ca) = (side(&mut s, i, j), side(&mut s, j, k), side(&mut s, k, i));
    let plane = Surface::Plane(Frame::new(a, (b - a).cross(c - a), b - a));
    s.add_face(plane, false, vec![vec![ab, bc, ca]], FaceTag { feature, slot: 2 });
    s.compute_shells();
    validate(&s).unwrap();
    s
}

/// A square pyramid sewn from five flat sheets, each facing whichever way it was drawn.
fn pyramid() -> Body {
    let top = v3(5.0, 5.0, 6.0);
    let c = [v3(0.0, 0.0, 0.0), v3(10.0, 0.0, 0.0), v3(10.0, 10.0, 0.0), v3(0.0, 10.0, 0.0)];
    let walls: Vec<Body> = (0..4).map(|i| triangle(c[i], c[(i + 1) % 4], top, i as u32 + 1)).collect();
    let base = fill(&Profile::new(Wire::rect(0.0, 0.0, 10.0, 10.0)), &Frame::WORLD, 9).unwrap();
    let mut list: Vec<&Body> = walls.iter().collect();
    list.push(&base);
    sew(&list, 1e-6).unwrap()
}

#[test]
fn a_pyramid_sews_from_triangles_and_its_volume_is_a_third() {
    let p = pyramid();
    assert!(!p.sheet);
    validate(&p).unwrap();
    near(mass_properties(&p).volume, 100.0 * 6.0 / 3.0, "volume");
}

#[test]
fn faces_that_only_meet_at_a_point_come_out_apart() {
    // Two opposite walls of a pyramid share nothing but its tip. They're not a pinch: each piece gets a vertex there of its own.
    let p = pyramid();
    let wall = |n: u32| (0..p.faces.len()).find(|&f| p.faces[f].tag.feature == n).unwrap();
    let s = sheet_from_faces(&p, &[wall(1), wall(3)]).unwrap();
    validate(&s).unwrap();
    assert_eq!(s.shells.len(), 2);
    assert_eq!(s.vertices.len(), 6, "three corners each, with the tip twice");
    assert_eq!(s.solids().len(), 2);
    // Neighbours do share an edge, and are one sheet with their tip once.
    let s = sheet_from_faces(&p, &[wall(1), wall(2)]).unwrap();
    assert_eq!(s.shells.len(), 1);
    assert_eq!(s.vertices.len(), 4);
}

// ----- Sweeping open curves -----

#[test]
fn a_straight_open_curve_extrudes_to_flat_panels() {
    // An L, up by 5.
    let path = Wire { segs: vec![line(v2(0.0, 0.0), v2(10.0, 0.0), 0), line(v2(10.0, 0.0), v2(10.0, 7.0), 1)] };
    let s = extrude_open(&path, &Frame::WORLD, 5.0, 3).unwrap();
    check_sheet(&s, 5.0 * 17.0);
    assert_eq!(s.faces.len(), 2);
    assert_eq!(s.shells.len(), 1);
    assert_eq!(s.boundary_edges().len(), 6);
    // Named for its segments, as on a solid, and facing to the right of travel.
    let first = s.faces.iter().find(|f| f.tag == FaceTag { feature: 3, slot: 2 }).unwrap();
    assert!(first.outward_normal(v3(5.0, 0.0, 2.0)).dist(-Vec3::Y) < 1e-12);
    let second = s.faces.iter().find(|f| f.tag == FaceTag { feature: 3, slot: 3 }).unwrap();
    assert!(second.outward_normal(v3(10.0, 3.0, 2.0)).dist(Vec3::X) < 1e-12);
    // Downward too.
    let down = extrude_open(&path, &Frame::WORLD, -5.0, 3).unwrap();
    check_sheet(&down, 5.0 * 17.0);
    assert!(down.aabb().min.z < -4.9);
}

#[test]
fn drawn_either_way_it_faces_to_the_right_and_the_chord_has_no_say() {
    // A chain whose closing line would cross it, and one that winds clockwise, which a solid's profile wouldn't allow.
    let zig = Wire { segs: vec![line(v2(0.0, 0.0), v2(10.0, 10.0), 0), line(v2(10.0, 10.0), v2(10.0, 0.0), 1), line(v2(10.0, 0.0), v2(0.0, 10.0), 2)] };
    assert!(extrude_open(&zig, &Frame::WORLD, 1.0, 1).is_err(), "that one crosses itself");
    let hook = Wire {
        segs: vec![line(v2(0.0, 0.0), v2(10.0, 0.0), 0), line(v2(10.0, 0.0), v2(10.0, 10.0), 1), line(v2(10.0, 10.0), v2(5.0, 10.0), 2), line(v2(5.0, 10.0), v2(5.0, -5.0), 3)],
    };
    // Its closing line runs from (5, -5) to the start, and doesn't matter. The chain itself crosses at (5, 0), so it's refused.
    assert!(extrude_open(&hook, &Frame::WORLD, 1.0, 1).is_err());
    let spiral = Wire {
        segs: vec![line(v2(0.0, 0.0), v2(10.0, 0.0), 0), line(v2(10.0, 0.0), v2(10.0, 10.0), 1), line(v2(10.0, 10.0), v2(5.0, 10.0), 2), line(v2(5.0, 10.0), v2(5.0, 5.0), 3)],
    };
    // Clockwise from the other end.
    let back = Wire { segs: spiral.segs.iter().rev().map(Seg::reversed).collect() };
    let (a, b) = (extrude_open(&spiral, &Frame::WORLD, 2.0, 1).unwrap(), extrude_open(&back, &Frame::WORLD, 2.0, 1).unwrap());
    check_sheet(&a, 2.0 * 30.0);
    check_sheet(&b, 2.0 * 30.0);
    let p = v3(5.0, 0.0, 1.0);
    let normal = |s: &Body| s.faces.iter().find(|f| f.tag.slot == 2).unwrap().outward_normal(p);
    assert!(normal(&a).dist(-Vec3::Y) < 1e-12);
    assert!(normal(&b).dist(Vec3::Y) < 1e-12, "{:?}", normal(&b));
}

#[test]
fn an_arc_extrudes_to_a_cylinders_side() {
    // A quarter of radius 5, up by 8, and more than half a turn, which is cut in two as a solid's would be.
    let quarter = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: 0.0, a1: PI / 2.0 }, id: 0 }] };
    check_sheet(&extrude_open(&quarter, &Frame::WORLD, 8.0, 1).unwrap(), 5.0 * (PI / 2.0) * 8.0);
    let most = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: 0.0, a1: 3.0 * PI / 2.0 }, id: 0 }] };
    let s = extrude_open(&most, &Frame::WORLD, 8.0, 1).unwrap();
    check_sheet(&s, 5.0 * (3.0 * PI / 2.0) * 8.0);
    assert_eq!(s.shells.len(), 1);
    assert!(s.faces.len() >= 2);
    // A whole circle drawn as a chain is a tube, with two boundary loops: genus 0, and no ends.
    let circle = Wire::circle(v2(0.0, 0.0), 5.0);
    let tube = extrude_open(&circle, &Frame::WORLD, 8.0, 1).unwrap();
    check_sheet(&tube, TAU * 5.0 * 8.0);
    assert_eq!(validate(&tube).unwrap().genus, vec![0]);
}

#[test]
fn a_spline_extrudes_too() {
    let curve = Spline::through(&[v3(0.0, 0.0, 0.0), v3(5.0, 3.0, 0.0), v3(10.0, 0.0, 0.0), v3(15.0, 3.0, 0.0)]).unwrap();
    let length = curve.length(curve.domain().0, curve.domain().1);
    let path = Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(curve)), id: 0 }] };
    check_sheet(&extrude_open(&path, &Frame::WORLD, 4.0, 1).unwrap(), length * 4.0);
}

#[test]
fn a_line_revolved_is_a_cylinder_a_cone_or_a_ring() {
    // Up the side at radius 5.
    let s = revolve_open(&Wire { segs: vec![line(v2(5.0, 0.0), v2(5.0, 8.0), 0)] }, &Frame::WORLD, TAU, 1).unwrap();
    check_sheet(&s, TAU * 5.0 * 8.0);
    assert_eq!(validate(&s).unwrap().genus, vec![0]);
    assert_eq!(s.boundary_edges().len(), 4);
    // Part of a turn.
    let s = revolve_open(&Wire { segs: vec![line(v2(5.0, 0.0), v2(5.0, 8.0), 0)] }, &Frame::WORLD, PI / 3.0, 1).unwrap();
    check_sheet(&s, 5.0 * (PI / 3.0) * 8.0);
    // A cone from a point on the axis, of slant 5 and radius 3.
    let s = revolve_open(&Wire { segs: vec![line(v2(0.0, 0.0), v2(3.0, 4.0), 0)] }, &Frame::WORLD, TAU, 1).unwrap();
    check_sheet(&s, PI * 3.0 * 5.0);
    // A flat ring.
    let s = revolve_open(&Wire { segs: vec![line(v2(2.0, 0.0), v2(5.0, 0.0), 0)] }, &Frame::WORLD, TAU, 1).unwrap();
    check_sheet(&s, PI * (25.0 - 4.0));
    assert_eq!(s.faces.len(), 1);
    assert_eq!(s.faces[0].loops.len(), 2);
    // Lying along the axis, it sweeps nothing.
    assert!(revolve_open(&Wire { segs: vec![line(v2(0.0, 0.0), v2(0.0, 5.0), 0)] }, &Frame::WORLD, TAU, 1).is_err());
    // Across the axis.
    assert!(revolve_open(&Wire { segs: vec![line(v2(-1.0, 0.0), v2(5.0, 0.0), 0)] }, &Frame::WORLD, TAU, 1).is_err());
}

#[test]
fn a_half_circle_revolved_is_a_sphere_and_closes() {
    let half = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 }] };
    let s = revolve_open(&half, &Frame::WORLD, TAU, 1).unwrap();
    check_sheet(&s, 4.0 * PI * 25.0);
    assert!(s.boundary_edges().is_empty());
    // It closes, so it has a volume, and it's still a sheet until it's sewn.
    near(mass_properties(&s).volume, 4.0 / 3.0 * PI * 125.0, "volume");
    // Part of a sphere from part of the arc: a cap from the pole down 60 degrees.
    let cap = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: PI / 6.0, a1: PI / 2.0 }, id: 0 }] };
    let s = revolve_open(&cap, &Frame::WORLD, TAU, 1).unwrap();
    // 2 pi r h, with h = r (1 - sin 30).
    check_sheet(&s, TAU * 5.0 * (5.0 * 0.5));
}

#[test]
fn a_torus_from_an_arc_and_a_panel_from_a_spline() {
    // Half a tube around the axis.
    let arc = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(10.0, 0.0), radius: 2.0, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 }] };
    let s = revolve_open(&arc, &Frame::WORLD, TAU, 1).unwrap();
    // By Pappus: the arc's length, 2 pi, times how far its centroid goes. Its centroid is at 10 + 2 * (2 / pi).
    check_sheet(&s, (PI * 2.0) * TAU * (10.0 + 4.0 / PI));
    let curve = Spline::through(&[v3(2.0, 0.0, 0.0), v3(4.0, 2.0, 0.0), v3(3.0, 5.0, 0.0)]).unwrap();
    let path = Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(curve)), id: 0 }] };
    let s = revolve_open(&path, &Frame::WORLD, PI, 1).unwrap();
    validate(&s).unwrap();
    assert!(s.sheet);
    assert_eq!(s.boundary_edges().len() >= 4, true);
}

#[test]
fn the_sides_of_a_solid_are_a_tube() {
    let solid = extrude(&Profile::new(Wire::circle(v2(0.0, 0.0), 5.0)), &Frame::WORLD, 8.0, 1).unwrap();
    let tube = sides_of(&solid).unwrap();
    check_sheet(&tube, TAU * 5.0 * 8.0);
    // Holes give tubes of their own.
    let ring = extrude(&Profile::new(Wire::circle(v2(0.0, 0.0), 5.0)).with_hole(Wire::circle(v2(0.0, 0.0), 2.0).reversed()), &Frame::WORLD, 8.0, 1).unwrap();
    let both = sides_of(&ring).unwrap();
    check_sheet(&both, TAU * (5.0 + 2.0) * 8.0);
    assert_eq!(both.shells.len(), 2);
    assert_eq!(both.solids().len(), 2);
}

#[test]
fn curves_that_cant_be_swept_are_refused() {
    assert!(extrude_open(&Wire { segs: Vec::new() }, &Frame::WORLD, 1.0, 1).is_err());
    assert!(extrude_open(&Wire { segs: vec![line(v2(0.0, 0.0), v2(1e-9, 0.0), 0)] }, &Frame::WORLD, 1.0, 1).is_err());
    let gap = Wire { segs: vec![line(v2(0.0, 0.0), v2(1.0, 0.0), 0), line(v2(1.5, 0.0), v2(2.0, 0.0), 1)] };
    assert!(extrude_open(&gap, &Frame::WORLD, 1.0, 1).unwrap_err().to_string().contains("gap"));
    assert!(extrude_open(&Wire { segs: vec![line(v2(0.0, 0.0), v2(1.0, 0.0), 0)] }, &Frame::WORLD, 0.0, 1).is_err());
    assert!(extrude_open(&Wire { segs: vec![line(v2(0.0, 0.0), v2(1.0, 0.0), 0)] }, &Frame::WORLD, f64::NAN, 1).is_err());
    assert!(fill(&Profile::new(Wire { segs: vec![line(v2(0.0, 0.0), v2(1.0, 0.0), 0)] }), &Frame::WORLD, 1).is_err());
}

// ----- From a sheet to a solid: thicken -----

#[test]
fn a_sheet_thickens_into_a_solid() {
    // Flat: the area times the thickness.
    let s = plate();
    let t = thicken(&s, &[0], 2.0, 9).unwrap();
    assert!(!t.sheet);
    validate(&t).unwrap();
    near(mass_properties(&t).volume, 400.0, "up");
    assert_eq!(mesh(&t).open_edge_count(), 0);
    let t = thicken(&s, &[0], -2.0, 9).unwrap();
    near(mass_properties(&t).volume, 400.0, "down");
    assert!(t.aabb().min.z < -1.9);

    // A tube, outward: an annulus's area by its height.
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 5.0), &Frame::WORLD, 8.0, 1).unwrap();
    let all: Vec<usize> = (0..tube.faces.len()).collect();
    let t = thicken(&tube, &all, 1.0, 9).unwrap();
    validate(&t).unwrap();
    near(mass_properties(&t).volume, PI * (36.0 - 25.0) * 8.0, "a tube");
    assert_eq!(t.shells.len(), 1);

    // A cap of a sphere, inward: by the difference of two caps.
    let cap = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: PI / 6.0, a1: PI / 2.0 }, id: 0 }] };
    let s = revolve_open(&cap, &Frame::WORLD, TAU, 1).unwrap();
    let all: Vec<usize> = (0..s.faces.len()).collect();
    let t = thicken(&s, &all, -1.0, 9).unwrap();
    validate(&t).unwrap();
    // The solid angle of the cap is 2 pi (1 - cos 60), and the shell between radii 4 and 5 takes that share of (5^3 - 4^3) / 3.
    near(mass_properties(&t).volume, TAU * 0.5 * (125.0 - 64.0) / 3.0, "a sphere's cap");

    // A closed sheet has no boundary to wall, and thickened outward it makes a hollow ball.
    let half = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 }] };
    let s = revolve_open(&half, &Frame::WORLD, TAU, 1).unwrap();
    let all: Vec<usize> = (0..s.faces.len()).collect();
    let t = thicken(&s, &all, 1.0, 9).unwrap();
    near(mass_properties(&t).volume, 4.0 / 3.0 * PI * (216.0 - 125.0), "a hollow ball");
    assert_eq!(t.shells.len(), 2);

    // Panels at a corner can't be thickened as one, and it says so.
    let path = Wire { segs: vec![line(v2(0.0, 0.0), v2(10.0, 0.0), 0), line(v2(10.0, 0.0), v2(10.0, 7.0), 1)] };
    let s = extrude_open(&path, &Frame::WORLD, 5.0, 3).unwrap();
    let all: Vec<usize> = (0..s.faces.len()).collect();
    assert!(thicken(&s, &all, 1.0, 9).unwrap_err().to_string().contains("sharp"));
}

// ----- From sheets to a solid: sewing -----

/// The six sides of a box, as separate sheets.
fn six_sides(x: f64, y: f64, z: f64) -> Vec<Body> {
    let solid = box_solid(&Frame::WORLD, x, y, z, 1).unwrap();
    (0..solid.faces.len()).map(|f| sheet_from_faces(&solid, &[f]).unwrap()).collect()
}

#[test]
fn six_sheets_sew_into_a_box() {
    let sides = six_sides(10.0, 20.0, 30.0);
    let list: Vec<&Body> = sides.iter().collect();
    let solid = sew(&list, 1e-6).unwrap();
    assert!(!solid.sheet, "it closes, so it's a solid");
    validate(&solid).unwrap();
    near(mass_properties(&solid).volume, 6000.0, "volume");
    assert_eq!(solid.faces.len(), 6);
    assert_eq!(solid.edges.len(), 12);
    assert_eq!(solid.vertices.len(), 8);
    assert_eq!(mesh(&solid).open_edge_count(), 0);
    // It's a solid in every way: a Boolean takes it.
    let cut = subtract(&solid, &box_solid(&Frame::translation(v3(5.0, 5.0, 5.0)), 20.0, 5.0, 5.0, 2).unwrap()).unwrap();
    near(mass_properties(&cut).volume, 6000.0 - 5.0 * 5.0 * 5.0, "cut");
}

#[test]
fn five_sheets_sew_into_a_box_with_no_lid() {
    let sides = six_sides(10.0, 20.0, 30.0);
    let list: Vec<&Body> = sides[..5].iter().collect();
    let open = sew(&list, 1e-6).unwrap();
    assert!(open.sheet);
    validate(&open).unwrap();
    assert_eq!(open.shells.len(), 1);
    assert_eq!(open.boundary_edges().len(), 4);
    assert_eq!(mass_properties(&open).volume, 0.0);
    let total: f64 = sides[..5].iter().map(area).sum();
    near(area(&open), total, "area");
    // A sixth sewn on closes it, with a sheet that has already been sewn.
    let solid = sew(&[&open, &sides[5]], 1e-6).unwrap();
    assert!(!solid.sheet);
    near(mass_properties(&solid).volume, 6000.0, "volume");
}

#[test]
fn sewing_turns_over_whatever_faces_the_wrong_way() {
    let mut sides = six_sides(10.0, 20.0, 30.0);
    // Two of them made facing in.
    for k in [1usize, 4] {
        let f = &mut sides[k].faces[0];
        f.flipped = !f.flipped;
        for lp in &mut f.loops {
            lp.reverse();
            for c in lp.iter_mut() {
                *c = Coedge { edge: c.edge, reversed: !c.reversed };
            }
        }
        validate(&sides[k]).unwrap();
    }
    let list: Vec<&Body> = sides.iter().collect();
    let solid = sew(&list, 1e-6).unwrap();
    validate(&solid).unwrap();
    near(mass_properties(&solid).volume, 6000.0, "volume");
    // All inside out to begin with, it still comes out with its outside out.
    let mut sides = six_sides(10.0, 20.0, 30.0);
    for side in &mut sides {
        let f = &mut side.faces[0];
        f.flipped = !f.flipped;
        for lp in &mut f.loops {
            lp.reverse();
            for c in lp.iter_mut() {
                *c = Coedge { edge: c.edge, reversed: !c.reversed };
            }
        }
    }
    let list: Vec<&Body> = sides.iter().collect();
    let solid = sew(&list, 1e-6).unwrap();
    near(mass_properties(&solid).volume, 6000.0, "volume");
}

#[test]
fn a_cylinder_sews_from_a_tube_and_two_discs_cut_differently() {
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 5.0), &Frame::WORLD, 8.0, 1).unwrap();
    // A disc with its circle cut at a different place, so that the boundaries don't share their ends. A disc turned by 30 degrees.
    let bottom = fill(&Profile::new(Wire::circle(v2(0.0, 0.0), 5.0)), &Frame::rotation(Vec3::ZERO, Vec3::Z, PI / 6.0), 2).unwrap();
    let top = fill(&Profile::new(Wire::circle(v2(0.0, 0.0), 5.0)), &Frame::translation(v3(0.0, 0.0, 8.0)), 3).unwrap();
    let solid = sew(&[&tube, &bottom, &top], 1e-6).unwrap();
    assert!(!solid.sheet);
    validate(&solid).unwrap();
    near(mass_properties(&solid).volume, PI * 25.0 * 8.0, "volume");
    assert_eq!(mesh(&solid).open_edge_count(), 0);
}

#[test]
fn a_long_side_sews_to_two_short_ones() {
    // One panel along the whole edge, and two along parts of it: the long edge is cut to fit.
    let long = fill(&rect(10.0, 5.0), &Frame::WORLD, 1).unwrap();
    let up = Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X);
    let left = fill(&rect(4.0, 5.0), &up, 2).unwrap();
    let right = fill(&rect(6.0, 5.0), &Frame { origin: v3(4.0, 0.0, 0.0), ..up }, 3).unwrap();
    let s = sew(&[&long, &left, &right], 1e-6).unwrap();
    validate(&s).unwrap();
    assert!(s.sheet);
    assert_eq!(s.shells.len(), 1);
    near(area(&s), 50.0 + 20.0 + 30.0, "area");
    // The two meet each other as well, so left and right are joined up the middle and merge into one face if they can.
    assert!(s.faces.len() <= 3);
}

#[test]
fn sewing_within_a_tolerance_records_it() {
    let a = fill(&rect(10.0, 5.0), &Frame::WORLD, 1).unwrap();
    // A gap of 1e-5 along the edge they share.
    let b = fill(&rect(10.0, 5.0), &Frame::translation(v3(10.0 + 1e-5, 0.0, 0.0)), 2).unwrap();
    assert!(sew(&[&a, &b], 1e-6).unwrap_err().to_string().contains("nothing to sew"));
    let s = sew(&[&a, &b], 1e-4).unwrap();
    assert!(s.sheet);
    validate(&s).unwrap();
    assert_eq!(s.shells.len(), 1);
    assert!(s.tolerance >= 0.99e-5 && s.tolerance <= 1e-4, "{}", s.tolerance);
}

#[test]
fn what_cant_be_sewn_is_refused() {
    let a = plate();
    assert!(sew(&[], 1e-6).is_err());
    assert!(sew(&[&a], 1e-12).is_err());
    assert!(sew(&[&a], 2.0).is_err());
    let solid = box_solid(&Frame::WORLD, 1.0, 1.0, 1.0, 2).unwrap();
    assert!(sew(&[&a, &solid], 1e-6).unwrap_err().to_string().contains("solid"));
    let far = plate().transformed(&Frame::translation(v3(100.0, 0.0, 0.0)));
    assert!(sew(&[&a, &far], 1e-6).unwrap_err().to_string().contains("nothing to sew"));

    // Three sheets along one edge.
    let p = fill(&rect(5.0, 5.0), &Frame::WORLD, 1).unwrap();
    let q = fill(&rect(5.0, 5.0), &Frame::new(Vec3::ZERO, -Vec3::Y, Vec3::X), 2).unwrap();
    let r = fill(&rect(5.0, 5.0), &Frame::new(Vec3::ZERO, Vec3::Y, Vec3::X), 3).unwrap();
    assert!(sew(&[&p, &q, &r], 1e-6).unwrap_err().to_string().contains("at most two"));

}

#[test]
fn a_band_with_a_twist_has_one_side_and_is_refused() {
    // The smallest Moebius band: five points, and the triangles i, i+1, i+2. Each neighbour runs the same way along the edge they
    // share, so every other triangle has to turn over, and around five of them that can't come out even.
    let at = |i: usize| {
        let a = TAU * i as f64 / 5.0;
        v3(10.0 * a.cos(), 10.0 * a.sin(), 3.0 * (2.0 * a).sin())
    };
    let parts: Vec<Body> = (0..5).map(|i| triangle(at(i), at((i + 1) % 5), at((i + 2) % 5), i as u32 + 1)).collect();
    let list: Vec<&Body> = parts.iter().collect();
    let why = sew(&list, 1e-6).unwrap_err().to_string();
    assert!(why.contains("only one side"), "{why}");
    // Four of them are fine: that's just a strip.
    let four: Vec<&Body> = parts[..4].iter().collect();
    let s = sew(&four, 1e-6).unwrap();
    assert!(s.sheet);
    validate(&s).unwrap();
    assert_eq!(s.shells.len(), 1);
}

// ----- Cutting a sheet -----

#[test]
fn a_solid_cuts_a_hole_in_a_sheet_and_the_two_parts_add_up() {
    let s = fill(&rect(40.0, 40.0), &Frame::WORLD, 1).unwrap();
    let post = cylinder(&Frame::translation(v3(20.0, 20.0, -5.0)), 5.0, 10.0, 2).unwrap();
    let outside = trim_sheet(&s, &post, Keep::Outside).unwrap();
    let inside = trim_sheet(&s, &post, Keep::Inside).unwrap();
    check_sheet(&outside, 1600.0 - PI * 25.0);
    check_sheet(&inside, PI * 25.0);
    assert_eq!(outside.faces[0].loops.len(), 2);
    assert_eq!(outside.faces[0].tag, s.faces[0].tag, "a cut keeps the face's name");
    near(area(&outside) + area(&inside), area(&s), "the identity");
    // Still facing the way it did.
    assert!(inside.faces[0].outward_normal(v3(20.0, 20.0, 0.0)).dist(Vec3::Z) < 1e-12);
}

#[test]
fn a_solid_across_a_sheet_splits_it_into_pieces() {
    let s = fill(&rect(40.0, 10.0), &Frame::WORLD, 1).unwrap();
    let bar = box_solid(&Frame::translation(v3(15.0, -5.0, -5.0)), 10.0, 20.0, 10.0, 2).unwrap();
    let outside = trim_sheet(&s, &bar, Keep::Outside).unwrap();
    check_sheet(&outside, 300.0);
    assert_eq!(outside.shells.len(), 2);
    assert_eq!(outside.solids().len(), 2);
    let inside = trim_sheet(&s, &bar, Keep::Inside).unwrap();
    check_sheet(&inside, 100.0);
}

#[test]
fn a_tube_cut_by_a_plane_is_as_long_as_what_is_left() {
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 5.0), &Frame::WORLD, 10.0, 1).unwrap();
    let up = trim_sheet_by_plane(&tube, v3(0.0, 0.0, 3.0), Vec3::Z, Keep::Outside).unwrap();
    check_sheet(&up, TAU * 5.0 * 7.0);
    let down = trim_sheet_by_plane(&tube, v3(0.0, 0.0, 3.0), Vec3::Z, Keep::Inside).unwrap();
    check_sheet(&down, TAU * 5.0 * 3.0);
    // A slanted plane through the middle takes half, whatever the slant, by symmetry.
    let slant = trim_sheet_by_plane(&tube, v3(0.0, 0.0, 5.0), v3(0.0, 0.5, 1.0), Keep::Outside).unwrap();
    check_sheet(&slant, TAU * 5.0 * 5.0);
    // A plane that misses it leaves it whole on one side and takes it all on the other.
    let all = trim_sheet_by_plane(&tube, v3(0.0, 0.0, -1.0), Vec3::Z, Keep::Outside).unwrap();
    check_sheet(&all, TAU * 5.0 * 10.0);
    let none = trim_sheet_by_plane(&tube, v3(0.0, 0.0, -1.0), Vec3::Z, Keep::Inside).unwrap();
    assert!(none.faces.is_empty());
    assert!(trim_sheet_by_plane(&tube, Vec3::ZERO, Vec3::ZERO, Keep::Outside).is_err());
}

#[test]
fn a_spheres_cap_comes_from_a_plane() {
    let half = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 }] };
    let ball = revolve_open(&half, &Frame::WORLD, TAU, 1).unwrap();
    // 2 pi r h for each side, with h = 2 on top.
    check_sheet(&trim_sheet_by_plane(&ball, v3(0.0, 0.0, 3.0), Vec3::Z, Keep::Outside).unwrap(), TAU * 5.0 * 2.0);
    check_sheet(&trim_sheet_by_plane(&ball, v3(0.0, 0.0, 3.0), Vec3::Z, Keep::Inside).unwrap(), TAU * 5.0 * 8.0);
}

#[test]
fn a_sheet_lying_on_a_solids_face_isnt_inside_it() {
    // So the two parts always make up the sheet, and nothing is counted twice.
    let s = fill(&rect(10.0, 10.0), &Frame::translation(v3(0.0, 0.0, 5.0)), 1).unwrap();
    let block = box_solid(&Frame::WORLD, 10.0, 10.0, 5.0, 2).unwrap();
    near(area(&trim_sheet(&s, &block, Keep::Outside).unwrap()), 100.0, "outside");
    assert!(trim_sheet(&s, &block, Keep::Inside).unwrap().faces.is_empty());
    // Half on it and half off it, which is all outside.
    let wider = fill(&rect(20.0, 10.0), &Frame::translation(v3(0.0, 0.0, 5.0)), 1).unwrap();
    near(area(&trim_sheet(&wider, &block, Keep::Outside).unwrap()), 200.0, "outside");
    assert!(trim_sheet(&wider, &block, Keep::Inside).unwrap().faces.is_empty());
    // Facing the other way makes no difference.
    let under = wider.mirrored(&Mirror::new(v3(0.0, 0.0, 5.0), Vec3::Z));
    near(area(&trim_sheet(&under, &block, Keep::Outside).unwrap()), 200.0, "outside, facing in");
    // On a plane it's on the side of the normal, whichever way the sheet faces.
    near(area(&trim_sheet_by_plane(&wider, v3(0.0, 0.0, 5.0), Vec3::Z, Keep::Outside).unwrap()), 200.0, "on a plane");
    assert!(trim_sheet_by_plane(&wider, v3(0.0, 0.0, 5.0), Vec3::Z, Keep::Inside).unwrap().faces.is_empty());
    // Part of a panel on a plane, and the rest across it.
    let path = Wire { segs: vec![line(v2(0.0, 0.0), v2(10.0, 0.0), 0), line(v2(10.0, 0.0), v2(10.0, 7.0), 1)] };
    let panels = extrude_open(&path, &Frame::WORLD, 5.0, 3).unwrap();
    let (a, b) = (trim_sheet_by_plane(&panels, Vec3::ZERO, -Vec3::Y, Keep::Outside).unwrap(), trim_sheet_by_plane(&panels, Vec3::ZERO, -Vec3::Y, Keep::Inside).unwrap());
    near(area(&a), 50.0, "the panel on it");
    near(area(&b), 35.0, "the panel beyond it");
}

#[test]
fn a_sheet_clear_of_the_solid_is_all_outside() {
    let s = plate();
    let far = box_solid(&Frame::translation(v3(100.0, 100.0, 100.0)), 1.0, 1.0, 1.0, 2).unwrap();
    near(area(&trim_sheet(&s, &far, Keep::Outside).unwrap()), 200.0, "outside");
    assert!(trim_sheet(&s, &far, Keep::Inside).unwrap().faces.is_empty());
}

#[test]
fn a_curved_sheet_is_cut_by_a_curved_solid() {
    // A tube cut by a sphere on its axis: the band within it goes. Each end keeps the tube's side beyond where they cross.
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 3.0), &Frame::WORLD, 20.0, 1).unwrap();
    let ball = sphere(&Frame::translation(v3(0.0, 0.0, 10.0)), 5.0, 2).unwrap();
    let outside = trim_sheet(&tube, &ball, Keep::Outside).unwrap();
    let inside = trim_sheet(&tube, &ball, Keep::Inside).unwrap();
    // They cross at height 4 above and below the middle, by 3-4-5.
    check_sheet(&inside, TAU * 3.0 * 8.0);
    check_sheet(&outside, TAU * 3.0 * 12.0);
    assert_eq!(outside.shells.len(), 2);
}

#[test]
fn trimming_asks_for_a_sheet_and_a_solid() {
    let s = plate();
    let solid = box_solid(&Frame::WORLD, 1.0, 1.0, 1.0, 2).unwrap();
    assert!(trim_sheet(&solid, &solid, Keep::Outside).unwrap_err().to_string().contains("this is a solid"));
    let why = trim_sheet(&s, &s, Keep::Outside).unwrap_err().to_string();
    assert!(why.contains("another sheet") && why.contains("no inside"), "{why}");
}

#[test]
fn a_cut_sheet_thickens_and_sews() {
    // Through to a solid: a plate with a hole, by the area times the thickness.
    let s = fill(&rect(40.0, 40.0), &Frame::WORLD, 1).unwrap();
    let post = cylinder(&Frame::translation(v3(20.0, 20.0, -5.0)), 5.0, 10.0, 2).unwrap();
    let holed = trim_sheet(&s, &post, Keep::Outside).unwrap();
    let t = thicken(&holed, &[0], 3.0, 9).unwrap();
    validate(&t).unwrap();
    near(mass_properties(&t).volume, 3.0 * (1600.0 - PI * 25.0), "volume");
    // And the disc it cut out goes back in.
    let disc = trim_sheet(&s, &post, Keep::Inside).unwrap();
    let whole = sew(&[&holed, &disc], 1e-6).unwrap();
    assert!(whole.sheet);
    near(area(&whole), 1600.0, "area");
    assert_eq!(whole.boundary_edges().len(), 4, "the seam has gone from the boundary");
}
