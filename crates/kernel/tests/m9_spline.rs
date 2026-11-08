//! Spline profiles, extruded and revolved. The strongest check available costs almost nothing: an
//! ellipse or circle written as a rational spline is the exact shape, and the answers are known in
//! closed form, or can be had from the same part built with true cylinders and cones.

use coreshape::math::gauss_legendre;
use coreshape::*;
use std::sync::Arc;

fn circle_wire(center: Vec2, radius: f64) -> Wire {
    let c = v3(center.x, center.y, 0.0);
    let spline = Spline::ellipse_arc(c, Vec3::X, Vec3::Y, radius, radius, 0.0, TAU).unwrap();
    Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(spline)), id: 0 }] }
}

fn ellipse_wire(major: f64, minor: f64) -> Wire {
    let spline = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, major, minor, 0.0, TAU).unwrap();
    Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(spline)), id: 0 }] }
}

/// The volume has to be right, and the reported error has to cover the gap.
fn assert_volume(body: &Body, want: f64, what: &str) {
    let m = mass_properties(body);
    let gap = (m.volume - want).abs();
    assert!(gap < 1e-6, "{what}: volume {} vs {want}", m.volume);
    assert!(gap <= m.volume_error + 1e-9, "{what}: off by {gap:e}, reported error {:e}", m.volume_error);
    assert!(m.volume_error < 1e-5, "{what}: error estimate {:e} is too loose to be useful", m.volume_error);
}

fn valid_and_watertight(body: &Body, what: &str) {
    validate(body).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(mesh.open_edge_count(), 0, "{what}: the mesh has gaps");
    let (m, v) = (mass_properties(body).volume, mesh.volume());
    assert!((m - v).abs() < 0.02 * m.abs().max(1.0), "{what}: mesh volume {v} vs {m}");
}

#[test]
fn an_extruded_rational_circle_is_a_cylinder() {
    let body = extrude(&Profile::new(circle_wire(v2(3.0, -2.0), 5.0)), &Frame::WORLD, 7.0, 1).unwrap();
    assert!(body.faces.iter().any(|f| matches!(f.surface, Surface::Extruded { .. })));
    valid_and_watertight(&body, "circle");
    assert_volume(&body, PI * 25.0 * 7.0, "circle");
    let area = mass_properties(&body).area;
    let want = 2.0 * PI * 25.0 + TAU * 5.0 * 7.0;
    assert!((area - want).abs() < 1e-7, "{area} vs {want}");
}

#[test]
fn an_extruded_ellipse_has_the_ellipse_area() {
    let body = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &Frame::WORLD, 3.0, 1).unwrap();
    valid_and_watertight(&body, "ellipse");
    assert_volume(&body, PI * 9.0 * 4.0 * 3.0, "ellipse");
    // In any placement.
    let f = Frame::new(v3(5.0, -3.0, 2.0), v3(0.3, 0.2, 1.0), Vec3::X);
    let moved = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &f, 3.0, 1).unwrap();
    valid_and_watertight(&moved, "placed");
    assert_volume(&moved, PI * 9.0 * 4.0 * 3.0, "placed");
    // Downward, and with a transformed copy.
    let down = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &f, -3.0, 1).unwrap();
    assert_volume(&down, PI * 9.0 * 4.0 * 3.0, "downward");
    let copy = moved.transformed(&Frame::rotation(Vec3::ZERO, v3(1.0, 1.0, 0.0), 1.1));
    valid_and_watertight(&copy, "transformed");
    assert_volume(&copy, PI * 9.0 * 4.0 * 3.0, "transformed");
    let mirrored = moved.mirrored(&Mirror::new(v3(40.0, 0.0, 0.0), v3(1.0, 0.2, 0.0)));
    valid_and_watertight(&mirrored, "mirrored");
    assert_volume(&mirrored, PI * 9.0 * 4.0 * 3.0, "mirrored");
}

#[test]
fn a_hole_of_an_ellipse_in_a_plate() {
    let profile = Profile::new(Wire::rect_centered(40.0, 30.0)).with_hole(ellipse_wire(9.0, 4.0));
    let body = extrude(&profile, &Frame::WORLD, 2.0, 1).unwrap();
    valid_and_watertight(&body, "plate");
    assert_volume(&body, (1200.0 - PI * 36.0) * 2.0, "plate");
}

#[test]
fn a_free_form_outline_has_the_area_green_gives_it() {
    let points: Vec<Vec3> = [(0.0, 0.0), (10.0, 4.0), (22.0, -2.0), (30.0, 5.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let curve = Spline::through(&points).unwrap();
    let wire = PathBuilder::new(v2(0.0, 0.0)).spline_to(curve).line_to(v2(30.0, 20.0)).line_to(v2(0.0, 20.0)).close();
    let profile = Profile::new(wire.clone());
    let area = profile.area();
    assert!(area > 400.0 && area < 700.0, "{area}");
    let body = extrude(&profile, &Frame::WORLD, 5.0, 1).unwrap();
    valid_and_watertight(&body, "outline");
    assert_volume(&body, area * 5.0, "outline");
    // Names survive: segment 0 is the spline's side.
    assert!(body.faces.iter().any(|f| f.tag.slot == 2 && matches!(f.surface, Surface::Extruded { .. })));
}

#[test]
fn a_spline_that_crosses_itself_or_the_outline_is_refused() {
    let looped: Vec<Vec3> = [(0.0, 0.0), (20.0, 0.0), (20.0, 15.0), (5.0, 15.0), (10.0, -8.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let s = Spline::through(&looped).unwrap();
    let wire = PathBuilder::new(v2(0.0, 0.0)).spline_to(s).line_to(v2(-10.0, -10.0)).close();
    let e = extrude(&Profile::new(wire), &Frame::WORLD, 1.0, 1).unwrap_err();
    assert!(e.to_string().contains("crosses itself"), "{e}");
    // A spline with a control point out of the plane.
    let bent = Spline::uniform(2, vec![Vec3::ZERO, v3(5.0, 5.0, 1.0), v3(10.0, 0.0, 0.0)]).unwrap();
    let wire = PathBuilder::new(v2(0.0, 0.0)).spline_to(bent).line_to(v2(5.0, -5.0)).close();
    assert!(extrude(&Profile::new(wire), &Frame::WORLD, 1.0, 1).is_err());
    // One that stops dead.
    let dead = Spline::uniform(3, vec![Vec3::ZERO, Vec3::ZERO, v3(5.0, 5.0, 0.0), v3(10.0, 0.0, 0.0)]).unwrap();
    let wire = PathBuilder::new(v2(0.0, 0.0)).spline_to(dead).line_to(v2(5.0, -5.0)).close();
    let e = extrude(&Profile::new(wire), &Frame::WORLD, 1.0, 1).unwrap_err();
    assert!(e.to_string().contains("stops dead"), "{e}");
}

// ----- Revolved -----

fn quarter(center: (f64, f64), radius: f64, a0: f64, a1: f64) -> Spline {
    Spline::ellipse_arc(v3(center.0, center.1, 0.0), Vec3::X, Vec3::Y, radius, radius, a0, a1).unwrap()
}

#[test]
fn a_rational_arc_revolved_is_a_torus() {
    // A full circle of radius 2 at distance 6 from the axis.
    let wire = Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(quarter((6.0, 0.0), 2.0, 0.0, TAU))), id: 0 }] };
    let body = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    assert!(body.faces.iter().all(|f| matches!(f.surface, Surface::Revolved { .. })));
    valid_and_watertight(&body, "torus");
    assert_volume(&body, 2.0 * PI * PI * 6.0 * 4.0, "torus");
    let area = mass_properties(&body).area;
    let want = 4.0 * PI * PI * 6.0 * 2.0;
    assert!((area - want).abs() < 1e-7, "{area} vs {want}");
}

#[test]
fn a_part_revolve_has_flat_caps() {
    let wire = Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(quarter((6.0, 0.0), 2.0, 0.0, TAU))), id: 0 }] };
    for degrees in [30.0f64, 90.0, 180.0, 250.0] {
        let body = revolve(&Profile::new(wire.clone()), &Frame::WORLD, degrees.to_radians(), 1).unwrap();
        valid_and_watertight(&body, &format!("{degrees} degrees"));
        assert_volume(&body, 2.0 * PI * PI * 6.0 * 4.0 * degrees / 360.0, &format!("{degrees} degrees"));
    }
}

#[test]
fn a_dome_with_a_tip_on_the_axis_is_a_sphere() {
    // A half circle from the south pole to the north pole, as a spline, closed along the axis.
    let arc = quarter((0.0, 0.0), 5.0, -PI / 2.0, PI / 2.0);
    let wire = PathBuilder::new(v2(0.0, -5.0)).spline_to(arc).line_to(v2(0.0, -5.0)).close();
    let body = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    valid_and_watertight(&body, "sphere");
    assert_volume(&body, 4.0 / 3.0 * PI * 125.0, "sphere");
    let area = mass_properties(&body).area;
    assert!((area - 4.0 * PI * 25.0).abs() < 1e-7, "{area}");

    // A spheroid, from an elliptical arc.
    let half = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 3.0, 8.0, -PI / 2.0, PI / 2.0).unwrap();
    let wire = PathBuilder::new(v2(0.0, -8.0)).spline_to(half).line_to(v2(0.0, -8.0)).close();
    let body = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    valid_and_watertight(&body, "spheroid");
    assert_volume(&body, 4.0 / 3.0 * PI * 9.0 * 8.0, "spheroid");
}

#[test]
fn a_straight_stretch_of_spline_revolves_into_a_cone() {
    let line = Spline::uniform(3, vec![v3(2.0, 0.0, 0.0), v3(3.0, 2.0, 0.0), v3(4.0, 4.0, 0.0), v3(5.0, 6.0, 0.0)]).unwrap();
    let wire = PathBuilder::new(v2(2.0, 0.0)).spline_to(line).line_to(v2(0.0, 6.0)).line_to(v2(0.0, 0.0)).close();
    let body = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    // Exactly: a straight spline becomes a true cone, since analytic surfaces stay analytic.
    assert!(body.faces.iter().any(|f| matches!(f.surface, Surface::Cone { .. })));
    assert!(!body.faces.iter().any(|f| matches!(f.surface, Surface::Revolved { .. })));
    let want = PI * 6.0 / 3.0 * (4.0 + 10.0 + 25.0);
    assert!((mass_properties(&body).volume - want).abs() < 1e-9);
}

#[test]
fn a_spline_touching_the_axis_midway_is_refused() {
    let s = Spline::through(&[v3(4.0, 0.0, 0.0), v3(0.0, 5.0, 0.0), v3(4.0, 10.0, 0.0)]).unwrap();
    let wire = PathBuilder::new(v2(4.0, 0.0)).spline_to(s).line_to(v2(8.0, 5.0)).close();
    assert!(revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).is_err());
}

#[test]
fn surfaces_of_a_free_form_revolve_match_a_mesh() {
    let points: Vec<Vec3> = [(5.0, 0.0), (7.0, 3.0), (4.0, 7.0), (6.0, 12.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let curve = Spline::through(&points).unwrap();
    let wire = PathBuilder::new(v2(5.0, 0.0)).spline_to(curve).line_to(v2(0.0, 12.0)).line_to(v2(0.0, 0.0)).close();
    let body = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    valid_and_watertight(&body, "vase");
    let m = mass_properties(&body);
    // Volume by the disc method, directly from the curve.
    let c = Spline::through(&points).unwrap();
    let (u0, u1) = c.domain();
    let mut breaks = vec![u0];
    breaks.extend(c.breaks_between(u0, u1));
    breaks.push(u1);
    let rule = gauss_legendre(40);
    let mut want = 0.0;
    for w in breaks.windows(2) {
        let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
        for &(x, weight) in &rule {
            let (p, d, _) = c.derivs(mid + half * x);
            want += PI * p.x * p.x * d.y * weight * half;
        }
    }
    assert!((m.volume - want).abs() < 1e-9, "{} vs {want}", m.volume);
    assert!((m.volume - want).abs() <= m.volume_error + 1e-9);
}

// ----- Refusals -----

#[test]
fn offsets_on_spline_faces_are_refused_by_name() {
    let body = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &Frame::WORLD, 3.0, 1).unwrap();
    let mentions = |e: Error| {
        let text = e.to_string();
        assert!(text.contains("spline") || text.contains("straight and circular"), "{text}");
    };
    mentions(offset(&body, 0.5).unwrap_err());
    mentions(shell(&body, &[], 0.5, 2).unwrap_err());
    let side = body.faces.iter().position(|f| matches!(f.surface, Surface::Extruded { .. })).unwrap();
    assert!(draft(&body, &[side], Vec3::Z, Vec3::ZERO, 0.05).is_err());
}

// ----- Booleans against known answers -----

fn cylinder_of_spline(radius: f64, height: f64, at: Vec2) -> Body {
    extrude(&Profile::new(circle_wire(at, radius)), &Frame::WORLD, height, 1).unwrap()
}

#[test]
fn a_rational_cylinder_cut_by_planes() {
    let c = cylinder_of_spline(5.0, 10.0, v2(0.0, 0.0));
    // A box across the top half: planes square to the sweep, and along it.
    let b = box_solid(&Frame::translation(v3(-10.0, -10.0, 4.0)), 20.0, 20.0, 20.0, 2).unwrap();
    let cut = subtract(&c, &b).unwrap();
    valid_and_watertight(&cut, "lower half");
    assert_volume(&cut, PI * 25.0 * 4.0, "lower half");
    // A half space by a box along the axis.
    let side = box_solid(&Frame::translation(v3(2.0, -10.0, -5.0)), 20.0, 20.0, 30.0, 2).unwrap();
    let cut = intersect(&c, &side).unwrap();
    valid_and_watertight(&cut, "cap");
    // A circular segment with chord at x = 2.
    let d = 2.0f64;
    let segment = 25.0 * (d / 5.0).acos() - d * (25.0 - d * d).sqrt();
    assert_volume(&cut, segment * 10.0, "cap");
}

#[test]
fn a_rational_cylinder_against_a_true_one_agrees_with_true_against_true() {
    let frame = Frame::WORLD;
    let spline_body = cylinder_of_spline(5.0, 10.0, v2(0.0, 0.0));
    let true_body = cylinder(&frame, 5.0, 10.0, 1).unwrap();
    // Parallel, overlapping: rulings from one curve meeting the other.
    let tool = cylinder(&Frame::translation(v3(6.0, 0.0, -2.0)), 4.0, 14.0, 2).unwrap();
    for (name, op) in [("union", BoolOp::Union), ("subtract", BoolOp::Subtract), ("intersect", BoolOp::Intersect)] {
        let (a, b) = (boolean::boolean(&spline_body, &tool, op).unwrap(), boolean::boolean(&true_body, &tool, op).unwrap());
        valid_and_watertight(&a, name);
        let (va, vb) = (mass_properties(&a).volume, mass_properties(&b).volume);
        assert!((va - vb).abs() < 1e-6, "{name}: {va} vs {vb}");
    }
    // Crossing at right angles, which has to be traced.
    let across = cylinder(&Frame::new(v3(-20.0, 0.0, 5.0), Vec3::X, Vec3::Y), 2.0, 40.0, 2).unwrap();
    for (name, op) in [("union", BoolOp::Union), ("subtract", BoolOp::Subtract), ("intersect", BoolOp::Intersect)] {
        let (a, b) = (boolean::boolean(&spline_body, &across, op).unwrap(), boolean::boolean(&true_body, &across, op).unwrap());
        valid_and_watertight(&a, name);
        let (va, vb) = (mass_properties(&a).volume, mass_properties(&b).volume);
        assert!((va - vb).abs() < 1e-4, "{name}: {va} vs {vb}");
    }
}

#[test]
fn a_rational_cylinder_coincident_with_a_true_one() {
    let a = cylinder_of_spline(5.0, 10.0, v2(0.0, 0.0));
    let b = cylinder(&Frame::translation(v3(0.0, 0.0, 6.0)), 5.0, 10.0, 2).unwrap();
    let u = union(&a, &b).unwrap();
    valid_and_watertight(&u, "stacked");
    assert_volume(&u, PI * 25.0 * 16.0, "stacked");
    let i = intersect(&a, &b).unwrap();
    assert_volume(&i, PI * 25.0 * 4.0, "overlap");
}

#[test]
fn two_extrusions_of_one_profile_overlap_cleanly() {
    let a = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let b = extrude(&Profile::new(ellipse_wire(9.0, 4.0)), &Frame::translation(v3(0.0, 0.0, 6.0)), 10.0, 2).unwrap();
    let u = union(&a, &b).unwrap();
    valid_and_watertight(&u, "stacked ellipses");
    assert_volume(&u, PI * 36.0 * 16.0, "stacked ellipses");
    let i = intersect(&a, &b).unwrap();
    assert_volume(&i, PI * 36.0 * 4.0, "overlapping ellipses");
}

#[test]
fn holes_along_and_across_an_elliptical_plate() {
    let plate = extrude(&Profile::new(ellipse_wire(20.0, 12.0)), &Frame::WORLD, 4.0, 1).unwrap();
    // Through it, along the sweep, and away from the wall.
    let hole = cylinder(&Frame::translation(v3(5.0, 0.0, -1.0)), 2.0, 6.0, 2).unwrap();
    let cut = subtract(&plate, &hole).unwrap();
    valid_and_watertight(&cut, "hole");
    assert_volume(&cut, PI * 240.0 * 4.0 - PI * 4.0 * 4.0, "hole");
    // Breaking through the wall: a notch.
    let notch = cylinder(&Frame::translation(v3(20.0, 0.0, -1.0)), 3.0, 6.0, 2).unwrap();
    let cut = subtract(&plate, &notch).unwrap();
    valid_and_watertight(&cut, "notch");
    let v = mass_properties(&cut).volume;
    assert!(v < PI * 240.0 * 4.0 && v > PI * 240.0 * 4.0 - PI * 9.0 * 4.0, "{v}");
    // A slanted cut across the sweep: the section is an exact spline.
    let n = v3(0.3, 0.1, 1.0).normalized();
    let slab = box_solid(&Frame::new(v3(0.0, 0.0, 2.0), n, Vec3::X), 200.0, 200.0, 100.0, 3).unwrap().transformed(&Frame::translation(Vec3::ZERO));
    let moved = slab.transformed(&Frame::translation(Frame::new(Vec3::ZERO, n, Vec3::X).point(-100.0, -100.0, 0.0)));
    let top = subtract(&plate, &moved).unwrap();
    valid_and_watertight(&top, "slanted cut");
    assert!(top.edges.iter().any(|e| matches!(e.curve, Curve::Spline(_))));
    // Everything below the slanted plane: integrate its height over the ellipse in closed form. The
    // plane's height is linear, so the mean height over the ellipse is its height at the center.
    assert_volume(&top, PI * 240.0 * 2.0, "slanted cut");
}

#[test]
fn a_revolved_spline_against_planes_and_coaxial_shapes() {
    let sphere_of = |r: f64| {
        let arc = quarter((0.0, 0.0), r, -PI / 2.0, PI / 2.0);
        let wire = PathBuilder::new(v2(0.0, -r)).spline_to(arc).line_to(v2(0.0, -r)).close();
        revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap()
    };
    let s = sphere_of(5.0);
    let cap = |h: f64| PI * h * h * (15.0 - h) / 3.0;
    // A plane square to the axis: circles.
    let above = box_solid(&Frame::translation(v3(-10.0, -10.0, 2.0)), 20.0, 20.0, 20.0, 2).unwrap();
    let top = intersect(&s, &above).unwrap();
    valid_and_watertight(&top, "cap");
    assert_volume(&top, cap(3.0), "cap");
    // A plane through the axis: the curve itself, on both sides.
    let half = box_solid(&Frame::translation(v3(0.0, -10.0, -10.0)), 20.0, 20.0, 20.0, 2).unwrap();
    let side = intersect(&s, &half).unwrap();
    valid_and_watertight(&side, "half");
    assert_volume(&side, 2.0 / 3.0 * PI * 125.0, "half");
    // A coaxial cylinder: a drilled bead.
    let drill = cylinder(&Frame::translation(v3(0.0, 0.0, -10.0)), 3.0, 20.0, 2).unwrap();
    let bead = subtract(&s, &drill).unwrap();
    valid_and_watertight(&bead, "bead");
    // A napkin ring: height 8, so volume pi h^3 / 6.
    assert_volume(&bead, PI * 512.0 / 6.0, "bead");
    // Off-axis, which has to be traced.
    let off = cylinder(&Frame::translation(v3(2.0, 0.0, -10.0)), 1.0, 20.0, 2).unwrap();
    let (a, b) = (subtract(&s, &off).unwrap(), subtract(&sphere(&Frame::WORLD, 5.0, 1).unwrap(), &off).unwrap());
    valid_and_watertight(&a, "off axis");
    let (va, vb) = (mass_properties(&a).volume, mass_properties(&b).volume);
    assert!((va - vb).abs() < 1e-4, "{va} vs {vb}");
}

#[test]
fn two_revolved_curves_about_one_axis() {
    // Two spheres as splines, about one axis: they meet in a circle.
    let ball = |r: f64, z: f64| {
        let arc = quarter((0.0, z), r, -PI / 2.0, PI / 2.0);
        let wire = PathBuilder::new(v2(0.0, z - r)).spline_to(arc).line_to(v2(0.0, z - r)).close();
        revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap()
    };
    let (a, b) = (ball(5.0, 0.0), ball(5.0, 6.0));
    let i = intersect(&a, &b).unwrap();
    valid_and_watertight(&i, "lens");
    // Two caps of height 2.
    let cap = PI * 4.0 * (15.0 - 2.0) / 3.0;
    assert_volume(&i, 2.0 * cap, "lens");
}

#[test]
fn stretches_that_two_splines_share_are_refused_not_guessed() {
    // Two profiles with one spline in common over part of its length, and overlapping otherwise.
    let shared: Vec<Vec3> = [(0.0, 0.0), (5.0, 3.0), (10.0, 1.0), (15.0, 4.0), (20.0, 2.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let whole = Spline::through(&shared).unwrap();
    let (u0, u1) = whole.domain();
    let part = whole.sub(u0 + (u1 - u0) * 0.2, u0 + (u1 - u0) * 0.8).unwrap();
    let (a0, a1) = (whole.start(), whole.end());
    let (p0, p1) = (part.start(), part.end());
    let wire_a = PathBuilder::new(v2(a0.x, a0.y)).spline_to(whole.clone()).line_to(v2(a1.x, 20.0)).line_to(v2(a0.x, 20.0)).close();
    let wire_b = PathBuilder::new(v2(p0.x, p0.y)).spline_to(part).line_to(v2(p1.x, 15.0)).line_to(v2(p0.x, 15.0)).close();
    let a = extrude(&Profile::new(wire_a), &Frame::WORLD, 5.0, 1).unwrap();
    let b = extrude(&Profile::new(wire_b), &Frame::translation(v3(0.0, 0.0, 1.0)), 2.0, 2).unwrap();
    match union(&a, &b) {
        Err(e) => println!("refused: {e}"),
        Ok(u) => {
            // If it answers, the answer has to be right: B sits entirely inside A.
            valid_and_watertight(&u, "shared stretch");
            let (want, got) = (mass_properties(&a).volume, mass_properties(&u).volume);
            assert!((want - got).abs() < 1e-6, "{want} vs {got}");
        }
    }
}
