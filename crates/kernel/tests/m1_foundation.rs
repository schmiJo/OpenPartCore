//! Milestone 1: every builder must give a valid body whose volume and area match
//! the closed-form answer, measured on the B-rep itself, and a watertight mesh.

use coreshape::*;

fn check(name: &str, b: &Body, volume: f64, area: f64) -> Report {
    let report = validate(b).unwrap_or_else(|e| panic!("{name}: {e:#?}"));
    let m = mass_properties(b);
    let (ev, ea) = ((m.volume - volume).abs() / volume, (m.area - area).abs() / area);
    assert!(ev < 1e-10, "{name}: volume {} expected {volume} (rel err {ev:e})", m.volume);
    assert!(ea < 1e-10, "{name}: area {} expected {area} (rel err {ea:e})", m.area);

    let size = b.aabb().diagonal();
    let mesh = tessellate(b, &TessParams::for_size(size)).unwrap_or_else(|e| panic!("{name}: {e}"));
    assert_eq!(mesh.open_edge_count(), 0, "{name}: mesh is not watertight");
    let mv = mesh.volume();
    assert!((mv - volume).abs() / volume < 3e-2, "{name}: mesh volume {mv} vs {volume}");
    report
}

fn off_world() -> Frame {
    Frame::new(v3(13.0, -7.0, 5.0), v3(0.3, -0.8, 0.5), v3(1.0, 0.2, 0.0))
}

#[test]
fn box_matches() {
    for frame in [Frame::WORLD, off_world()] {
        let b = box_solid(&frame, 3.0, 4.0, 5.0, 1).unwrap();
        let r = check("box", &b, 60.0, 94.0);
        assert_eq!((r.vertices, r.edges, r.faces, r.shells), (8, 12, 6, 1));
        assert_eq!(r.genus, vec![0]);
    }
}

#[test]
fn cylinder_matches() {
    for frame in [Frame::WORLD, off_world()] {
        let (r, h) = (2.5, 7.0);
        let b = cylinder(&frame, r, h, 1).unwrap();
        let report = check("cylinder", &b, PI * r * r * h, TAU * r * h + TAU * r * r);
        assert_eq!(report.genus, vec![0]);
    }
}

#[test]
fn plate_with_holes_has_genus_equal_to_hole_count() {
    let p = Profile::new(Wire::rect_centered(40.0, 20.0))
        .with_hole(Wire::circle(v2(-10.0, 0.0), 3.0))
        .with_hole(Wire::circle(v2(10.0, 0.0), 4.0));
    let b = extrude(&p, &Frame::WORLD, 5.0, 1).unwrap();
    let hole_area = PI * 9.0 + PI * 16.0;
    let volume = (800.0 - hole_area) * 5.0;
    let area = 2.0 * (800.0 - hole_area) + 5.0 * (120.0 + TAU * 3.0 + TAU * 4.0);
    let r = check("plate", &b, volume, area);
    assert_eq!(r.genus, vec![2]);
}

#[test]
fn rounded_plate_matches() {
    let p = Profile::new(Wire::rounded_rect(30.0, 20.0, 3.0));
    let b = extrude(&p, &off_world(), 4.0, 1).unwrap();
    let a = 600.0 - 36.0 + PI * 9.0;
    let perimeter = 2.0 * (24.0 + 14.0) + TAU * 3.0;
    check("rounded plate", &b, a * 4.0, 2.0 * a + perimeter * 4.0);
}

#[test]
fn arc_over_180_degrees_is_split_into_valid_faces() {
    let w = PathBuilder::new(v2(0.0, 0.0)).line_to(v2(2.0, 0.0)).arc_to(v2(0.0, 2.0), v2(0.0, 0.0), true).close();
    let b = extrude(&Profile::new(w), &Frame::WORLD, 1.0, 1).unwrap();
    check("quarter disc", &b, PI, 2.0 * PI + (4.0 + PI));

    // Three quarters of a disc: the arc spans 270 degrees.
    let w = PathBuilder::new(v2(0.0, 0.0)).line_to(v2(2.0, 0.0)).arc_to(v2(0.0, -2.0), v2(0.0, 0.0), true).close();
    let b = extrude(&Profile::new(w), &Frame::WORLD, 1.0, 1).unwrap();
    let area = 3.0 * PI;
    check("three quarter disc", &b, area, 2.0 * area + 4.0 + 3.0 * PI);
}

#[test]
fn sphere_matches() {
    for frame in [Frame::WORLD, off_world()] {
        let r = 3.0;
        let b = sphere(&frame, r, 1).unwrap();
        let report = check("sphere", &b, 4.0 / 3.0 * PI * r * r * r, 4.0 * PI * r * r);
        assert_eq!((report.vertices, report.edges, report.faces), (2, 2, 2));
        assert_eq!(report.genus, vec![0]);
    }
}

#[test]
fn torus_matches() {
    for frame in [Frame::WORLD, off_world()] {
        let (rr, r) = (5.0, 1.5);
        let b = torus(&frame, rr, r, 1).unwrap();
        let report = check("torus", &b, 2.0 * PI * PI * rr * r * r, 4.0 * PI * PI * rr * r);
        assert_eq!(report.genus, vec![1]);
    }
}

#[test]
fn cones_match() {
    for frame in [Frame::WORLD, off_world()] {
        let (r0, r1, h) = (4.0, 2.0, 6.0);
        let slant = ((r0 - r1) * (r0 - r1) + h * h as f64).sqrt();
        let lateral = PI * (r0 + r1) * slant;
        let volume = PI * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
        let b = cone_frustum(&frame, r0, r1, h, 1).unwrap();
        check("narrowing frustum", &b, volume, lateral + PI * (r0 * r0 + r1 * r1));
        let b = cone_frustum(&frame, r1, r0, h, 1).unwrap();
        check("widening frustum", &b, volume, lateral + PI * (r0 * r0 + r1 * r1));

        // A true cone with its apex at the top, then at the bottom.
        let slant = (r0 * r0 + h * h as f64).sqrt();
        let v = PI * r0 * r0 * h / 3.0;
        let b = cone_frustum(&frame, r0, 0.0, h, 1).unwrap();
        check("cone apex up", &b, v, PI * r0 * slant + PI * r0 * r0);
        let b = cone_frustum(&frame, 0.0, r0, h, 1).unwrap();
        check("cone apex down", &b, v, PI * r0 * slant + PI * r0 * r0);
    }
}

#[test]
fn flange_blank_revolve_matches_pappus_terms() {
    // A stepped ring: hub r 10..20 height 30, flange r 10..40 height 8.
    let w =
        Wire::polygon(&[v2(10.0, 0.0), v2(40.0, 0.0), v2(40.0, 8.0), v2(20.0, 8.0), v2(20.0, 30.0), v2(10.0, 30.0)]);
    let b = revolve(&Profile::new(w), &off_world(), TAU, 1).unwrap();
    let volume = PI * (20.0f64 * 20.0 - 100.0) * 30.0 + PI * (1600.0f64 - 400.0) * 8.0;
    let area = PI * (1600.0 - 100.0)  // bottom
        + PI * (1600.0 - 400.0)       // flange top
        + PI * (400.0 - 100.0)        // hub top
        + TAU * 10.0 * 30.0           // bore
        + TAU * 40.0 * 8.0            // flange rim
        + TAU * 20.0 * 22.0; // hub wall
    let r = check("flange blank", &b, volume, area);
    assert_eq!(r.genus, vec![1]);
}

#[test]
fn partial_revolve_matches() {
    let w = Wire::rect(5.0, 0.0, 2.0, 3.0);
    let p = Profile::new(w);
    // Under and over 180 degrees exercise the one-step and two-step sweeps.
    for angle in [1.0, PI, 4.5] {
        let b = revolve(&p, &off_world(), angle, 1).unwrap();
        let v = angle / 2.0 * (49.0 - 25.0) * 3.0;
        let a = 2.0 * (2.0 * 3.0) // end caps
            + angle * (49.0 - 25.0) // top and bottom rings
            + angle * 7.0 * 3.0
            + angle * 5.0 * 3.0;
        check("partial ring", &b, v, a);
    }
}

#[test]
fn partial_revolve_touching_the_axis() {
    let w = Wire::polygon(&[v2(0.0, 0.0), v2(4.0, 0.0), v2(4.0, 3.0), v2(0.0, 3.0)]);
    let b = revolve(&Profile::new(w), &Frame::WORLD, PI / 2.0, 1).unwrap();
    let v = PI * 16.0 * 3.0 / 4.0;
    let a = 2.0 * (PI * 16.0 / 4.0) + 2.0 * 12.0 + (TAU * 4.0 / 4.0) * 3.0;
    check("quarter cylinder", &b, v, a);
}

#[test]
fn revolved_profile_with_a_hole_has_a_void_shell() {
    let p = Profile::new(Wire::rect(5.0, -5.0, 10.0, 10.0)).with_hole(Wire::circle(v2(10.0, 0.0), 2.0));
    let b = revolve(&p, &Frame::WORLD, TAU, 1).unwrap();
    let outer = PI * (225.0f64 - 25.0) * 10.0;
    let void = 2.0 * PI * PI * 10.0 * 4.0;
    let outer_area = 2.0 * PI * (225.0 - 25.0) + TAU * 15.0 * 10.0 + TAU * 5.0 * 10.0;
    let void_area = 4.0 * PI * PI * 10.0 * 2.0;
    let r = check("tube with void", &b, outer - void, outer_area + void_area);
    assert_eq!(r.shells, 2);
    assert_eq!(r.genus, vec![1, 1]);
}

#[test]
fn spheroid_cap_and_lens_revolves_match() {
    // A dome: quarter arc plus a flat base. Half a sphere of radius 3.
    let w = PathBuilder::new(v2(0.0, 0.0)).line_to(v2(3.0, 0.0)).arc_to(v2(0.0, 3.0), v2(0.0, 0.0), true).close();
    let b = revolve(&Profile::new(w), &Frame::WORLD, TAU, 1).unwrap();
    check("dome", &b, 2.0 / 3.0 * PI * 27.0, 2.0 * PI * 9.0 + PI * 9.0);
}

#[test]
fn bad_inputs_return_errors() {
    assert!(extrude(&Profile::new(Wire::rect_centered(1.0, 1.0)), &Frame::WORLD, 0.0, 1).is_err());
    assert!(revolve(&Profile::new(Wire::rect(-1.0, 0.0, 2.0, 1.0)), &Frame::WORLD, TAU, 1).is_err());
    assert!(torus(&Frame::WORLD, 1.0, 2.0, 1).is_err());
    assert!(sphere(&Frame::WORLD, 0.0, 1).is_err());
}

#[test]
fn validator_catches_damage() {
    let good = box_solid(&Frame::WORLD, 1.0, 1.0, 1.0, 1).unwrap();
    // Remove a face: the shell is open.
    let mut open = good.clone();
    open.faces.pop();
    open.compute_shells();
    assert!(validate(&open).is_err());
    // Reverse an edge use: the loop is broken and an edge is used one way twice.
    let mut bad = good.clone();
    bad.faces[0].loops[0][0].reversed ^= true;
    assert!(validate(&bad).is_err());
    // Move a vertex: geometry no longer matches topology.
    let mut moved = good.clone();
    moved.vertices[0].x += 0.1;
    assert!(validate(&moved).is_err());
    // Flip a whole loop: Euler still holds but edges pair the wrong way round.
    let mut flipped = good.clone();
    let lp = &mut flipped.faces[2].loops[0];
    lp.reverse();
    for c in lp.iter_mut() {
        *c = c.flipped();
    }
    assert!(validate(&flipped).is_err());
}

#[test]
fn stl_output_has_the_right_size() {
    let b = cylinder(&Frame::WORLD, 2.0, 3.0, 1).unwrap();
    let mesh = tessellate(&b, &TessParams::for_size(5.0)).unwrap();
    let bin = coreshape::export::stl::write_binary(&mesh, "cyl");
    assert_eq!(bin.len(), 84 + 50 * mesh.triangles.len());
    let text = coreshape::export::stl::write_ascii(&mesh, "cyl");
    assert_eq!(text.matches("facet normal").count(), mesh.triangles.len());
}
