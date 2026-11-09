//! Writes every demo model as STEP and STL, with its exact volume and area, for
//! checking in another CAD system. Usage: gallery <output directory>
use coreshape::*;
use coreshape_wasm::models;
use std::fmt::Write;

fn spline_extras() -> Vec<(&'static str, Result<Body>)> {
    let ellipse = |a: f64, b: f64| {
        let s = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, a, b, 0.0, TAU).unwrap();
        Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(s)), id: 0 }] }
    };
    let wavy = || {
        let points: Vec<Vec3> = [(30.0, 0.0), (20.0, 14.0), (5.0, 9.0), (-10.0, 18.0), (-30.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        PathBuilder::new(v2(30.0, 0.0)).spline_to(Spline::through(&points).unwrap()).close()
    };
    let vase_profile = || {
        let points: Vec<Vec3> = [(14.0, 0.0), (20.0, 12.0), (9.0, 28.0), (13.0, 44.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        PathBuilder::new(v2(14.0, 0.0)).spline_to(Spline::through(&points).unwrap()).line_to(v2(0.0, 44.0)).line_to(v2(0.0, 0.0)).close()
    };
    let dome = || {
        let arc = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 12.0, 20.0, -PI / 2.0, PI / 2.0).unwrap();
        PathBuilder::new(v2(0.0, -20.0)).spline_to(arc).close()
    };
    let slab_above = |height: f64, tilt: Vec3| {
        let f = Frame::new(v3(0.0, 0.0, height), tilt, Vec3::X);
        box_solid(&Frame { origin: f.point(-200.0, -200.0, 0.0), ..f }, 400.0, 400.0, 200.0, 9).unwrap()
    };
    vec![
        ("spline_ellipse_rod", extrude(&Profile::new(ellipse(20.0, 9.0)), &Frame::WORLD, 15.0, 1)),
        ("spline_ellipse_ring", extrude(&Profile::new(ellipse(20.0, 12.0)).with_hole(ellipse(9.0, 5.0)), &Frame::WORLD, 6.0, 1)),
        ("spline_outline", extrude(&Profile::new(wavy()), &Frame::WORLD, 10.0, 1)),
        ("spline_slanted_cut", subtract(&extrude(&Profile::new(ellipse(20.0, 12.0)), &Frame::WORLD, 20.0, 1).unwrap(), &slab_above(10.0, v3(0.3, 0.1, 1.0)))),
        ("spline_drilled", subtract(&extrude(&Profile::new(wavy()), &Frame::WORLD, 10.0, 1).unwrap(), &cylinder(&Frame::translation(v3(5.0, 5.0, -1.0)), 3.0, 12.0, 2).unwrap())),
        ("spline_notched", subtract(&extrude(&Profile::new(wavy()), &Frame::WORLD, 10.0, 1).unwrap(), &cylinder(&Frame::translation(v3(5.0, 9.0, -1.0)), 4.0, 12.0, 2).unwrap())),
        ("spline_cross_drilled", subtract(&extrude(&Profile::new(ellipse(20.0, 12.0)), &Frame::WORLD, 20.0, 1).unwrap(), &cylinder(&Frame::new(v3(-30.0, 0.0, 10.0), Vec3::X, Vec3::Y), 4.0, 60.0, 2).unwrap())),
        ("spline_vase", revolve(&Profile::new(vase_profile()), &Frame::WORLD, TAU, 1)),
        ("spline_vase_part", revolve(&Profile::new(vase_profile()), &Frame::WORLD, 2.2, 1)),
        ("spline_dome", revolve(&Profile::new(dome()), &Frame::WORLD, TAU, 1)),
        ("spline_vase_hollow", subtract(&revolve(&Profile::new(vase_profile()), &Frame::WORLD, TAU, 1).unwrap(), &cylinder(&Frame::translation(v3(0.0, 0.0, 4.0)), 6.0, 50.0, 2).unwrap())),
        ("spline_vase_halved", subtract(&revolve(&Profile::new(vase_profile()), &Frame::WORLD, TAU, 1).unwrap(), &box_solid(&Frame::translation(v3(0.0, -50.0, -10.0)), 100.0, 100.0, 80.0, 2).unwrap())),
        ("spline_vase_side_hole", subtract(&revolve(&Profile::new(vase_profile()), &Frame::WORLD, TAU, 1).unwrap(), &cylinder(&Frame::new(v3(-40.0, 0.0, 20.0), Vec3::X, Vec3::Y), 3.0, 80.0, 2).unwrap())),
    ]
}

fn loft_extras() -> Vec<(&'static str, Result<Body>)> {
    let at = |z: f64| Frame::translation(v3(0.0, 0.0, z));
    let circle = |r: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r));
    let square = |s: f64| Profile::new(Wire::polygon(&[v2(-s / 2.0, -s / 2.0), v2(s / 2.0, -s / 2.0), v2(s / 2.0, s / 2.0), v2(-s / 2.0, s / 2.0)]));
    let ring = |a: f64, b: f64| Profile::new(Wire::circle(v2(0.0, 0.0), a)).with_hole(Wire::circle(v2(0.0, 0.0), b));
    let wavy = || {
        let points: Vec<Vec3> = [(18.0, 0.0), (9.0, 13.0), (-7.0, 10.0), (-16.0, -4.0), (-3.0, -15.0), (12.0, -11.0), (18.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        Profile::new(PathBuilder::new(v2(18.0, 0.0)).spline_to(Spline::through(&points).unwrap()).close())
    };
    let tilted = Frame::new(v3(4.0, 0.0, 30.0), v3(0.25, 0.1, 1.0), Vec3::X);
    let nozzle = || loft(&[(&square(40.0), at(0.0)), (&circle(13.0), at(25.0)), (&square(18.0), at(50.0))], 1);
    let free = || loft(&[(&wavy(), at(0.0)), (&square(22.0), at(20.0)), (&circle(10.0), tilted)], 1);
    let ball = |x: f64, y: f64, z: f64, r: f64| sphere(&Frame::translation(v3(x, y, z)), r, 2).unwrap();
    let across = Frame::new(v3(-40.0, 1.0, 25.0), Vec3::X, Vec3::Y);
    vec![
        ("loft_cylinder", loft(&[(&circle(10.0), at(0.0)), (&circle(10.0), at(30.0))], 1)),
        ("loft_frustum", loft(&[(&circle(14.0), at(0.0)), (&circle(6.0), at(25.0))], 1)),
        ("loft_square_circle", loft(&[(&square(30.0), at(0.0)), (&circle(10.0), at(30.0))], 1)),
        ("loft_ring", loft(&[(&ring(16.0, 8.0), at(0.0)), (&ring(10.0, 3.0), at(20.0))], 1)),
        ("loft_nozzle", nozzle()),
        ("loft_free", free()),
        ("loft_nozzle_ball", subtract(&nozzle().unwrap(), &ball(10.0, 8.0, 26.0, 9.0))),
        ("loft_nozzle_bored", subtract(&nozzle().unwrap(), &cylinder(&across, 4.0, 80.0, 2).unwrap())),
        ("loft_nozzle_joined", union(&nozzle().unwrap(), &cylinder(&across, 4.0, 80.0, 2).unwrap())),
        ("loft_free_halved", intersect(&free().unwrap(), &box_solid(&Frame::translation(v3(-40.0, -40.0, 8.0)), 80.0, 80.0, 40.0, 2).unwrap())),
        ("loft_free_ball", subtract(&free().unwrap(), &ball(8.0, 5.0, 12.0, 7.0))),
        ("loft_mirrored", nozzle().map(|b| b.mirrored(&Mirror::new(v3(30.0, 0.0, 0.0), v3(1.0, 0.1, 0.0))))),
    ]
}

/// Fillets and chamfers with no exact surface, which OpenCASCADE then has to read.
fn helix_extras() -> Vec<(&'static str, Result<Body>)> {
    let upright = Frame { origin: Vec3::ZERO, x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
    let along = |pitch: f64, turns: f64| Helix { origin: Vec3::ZERO, axis: Vec3::Z, pitch, turns, left_handed: false };
    let rect = |a: f64, b: f64, c: f64, d: f64| Profile::new(Wire::polygon(&[v2(a, c), v2(b, c), v2(b, d), v2(a, d)]));
    let circle = |x: f64, r: f64| Profile::new(Wire::circle(v2(x, 0.0), r));
    let at = |z: f64| Frame::translation(v3(0.0, 0.0, z));
    let shaft = || cylinder(&at(-2.0), 6.0, 20.0, 1).unwrap();
    let groove = rect(5.2, 6.5, -0.6, 0.6);
    let groove_at = Frame { origin: v3(0.0, 0.0, -4.0), ..upright };
    let vase = || {
        let round = |r: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r));
        loft(&[(&round(10.0), at(0.0)), (&round(7.0), at(12.0)), (&round(9.0), at(24.0))], 1).unwrap()
    };
    let sides = |b: &Body| -> Vec<usize> { (0..b.faces.len()).filter(|&f| !matches!(b.faces[f].surface, Surface::Plane(_))).collect() };
    let v = vase();
    let block = box_solid(&Frame::WORLD, 30.0, 20.0, 10.0, 1).unwrap();
    let top = (0..6).find(|&f| block.faces[f].surface.sdf(v3(1.0, 1.0, 10.0)).abs() < 1e-9).unwrap();
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    let s = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    let k = cone_frustum(&Frame::WORLD, 10.0, 5.0, 12.0, 1).unwrap();
    let t = torus(&Frame::WORLD, 20.0, 5.0, 1).unwrap();
    let plate = subtract(&block, &box_solid(&Frame::translation(v3(12.0, -1.0, 7.0)), 6.0, 22.0, 4.0, 2).unwrap()).unwrap();
    let plate_faces: Vec<usize> = (0..plate.faces.len()).filter(|&f| plate.faces[f].tag.feature == 2).collect();
    let round_edges = fillet(&block, &(0..block.edges.len()).collect::<Vec<_>>(), 2.0, 2).unwrap();
    let round_faces: Vec<usize> = (0..round_edges.faces.len()).filter(|&f| round_edges.faces[f].tag.feature == 2).collect();
    let lean = {
        let f = Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&Frame::translation(v3(-50.0, -50.0, 25.0)));
        subtract(&cylinder(&Frame::WORLD, 10.0, 40.0, 1).unwrap(), &box_solid(&f, 100.0, 100.0, 50.0, 2).unwrap()).unwrap()
    };
    let lean_sides: Vec<usize> = (0..lean.faces.len()).filter(|&f| matches!(lean.faces[f].surface, Surface::Cylinder { .. })).collect();
    vec![
        ("helix_square_spring", helix(&rect(8.0, 10.0, 0.0, 1.5), &upright, &along(3.0, 2.5), 1)),
        ("helix_round_spring", helix(&circle(12.0, 1.5), &upright, &along(5.0, 4.0), 1)),
        ("helix_hollow_spring", helix(&Profile::new(Wire::circle(v2(12.0, 0.0), 1.5)).with_hole(Wire::circle(v2(12.0, 0.0), 0.75)), &upright, &along(5.0, 2.0), 1)),
        ("helix_twisted_bar", helix(&rect(-5.0, 5.0, -3.0, 3.0), &Frame::WORLD, &along(40.0, 1.0), 1)),
        ("helix_thread_cut", subtract(&shaft(), &helix(&groove, &groove_at, &along(2.5, 10.0), 2).unwrap())),
        ("helix_ridge_added", union(&shaft(), &helix(&rect(5.0, 6.8, -0.6, 0.6), &Frame { origin: v3(0.0, 0.0, 2.0), ..upright }, &along(2.5, 4.0), 2).unwrap())),
        ("thick_flat", thicken(&block, &[top], 3.0, 5)),
        ("thick_cylinder", thicken(&c, &sides(&c), 2.0, 5)),
        ("thick_cylinder_in", thicken(&c, &sides(&c), -2.0, 5)),
        ("thick_sphere", thicken(&s, &sides(&s), 1.0, 5)),
        ("thick_cone", thicken(&k, &sides(&k), 1.0, 5)),
        ("thick_torus", thicken(&t, &sides(&t), 1.0, 5)),
        ("thick_vase", thicken(&v, &sides(&v), 0.5, 5)),
        ("thick_lean", thicken(&lean, &lean_sides, 2.0, 5)),
        ("vase_offset", offset(&v, 0.5)),
        ("vase_offset_in", offset(&v, -0.5)),
        ("vase_hollow", shell(&v, &[], 0.5, 5)),
        ("healed_slot", delete_faces(&plate, &plate_faces)),
        ("healed_rounding", delete_faces(&round_edges, &round_faces)),
    ]
}

fn blend_extras() -> Vec<(&'static str, Result<Body>)> {
    let curved = |b: &Body| -> Vec<usize> { (0..b.edges.len()).filter(|&e| !matches!(b.edges[e].curve, Curve::Circle { .. } | Curve::Line { .. })).collect() };
    let ellipse = || {
        let cyl = cylinder(&Frame::WORLD, 10.0, 40.0, 1).unwrap();
        let f = Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&Frame::translation(v3(-50.0, -50.0, 25.0)));
        subtract(&cyl, &box_solid(&f, 100.0, 100.0, 50.0, 2).unwrap()).unwrap()
    };
    let tee = || union(&cylinder(&Frame::new(v3(-30.0, 0.0, 0.0), Vec3::X, Vec3::Y), 10.0, 60.0, 1).unwrap(), &cylinder(&Frame::WORLD, 5.0, 30.0, 2).unwrap()).unwrap();
    let drilled = || subtract(&cylinder(&Frame::WORLD, 20.0, 40.0, 1).unwrap(), &cylinder(&Frame::new(v3(0.0, 0.0, 20.0), Vec3::X, Vec3::Y), 4.0, 40.0, 2).unwrap()).unwrap();
    let points: Vec<Vec3> = [(30.0, 0.0), (20.0, 14.0), (5.0, 9.0), (-10.0, 18.0), (-30.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let wavy = extrude(&Profile::new(PathBuilder::new(v2(30.0, 0.0)).spline_to(Spline::through(&points).unwrap()).close()), &Frame::WORLD, 10.0, 1).unwrap();
    let wavy_top: Vec<usize> = (0..wavy.edges.len()).filter(|&e| matches!(wavy.edges[e].curve, Curve::Spline(_)) && (wavy.edge_midpoint(e).z - 10.0).abs() < 1e-9).collect();
    let (e, t, d) = (ellipse(), tee(), drilled());
    vec![
        ("blend_ellipse_fillet", fillet(&e, &curved(&e), 2.0, 3)),
        ("blend_ellipse_chamfer", chamfer(&e, &curved(&e), 1.0, 3)),
        ("blend_tee_fillet", fillet(&t, &curved(&t), 2.0, 3)),
        ("blend_drilled_fillet", fillet(&d, &curved(&d), 1.0, 3)),
        ("blend_drilled_chamfer", chamfer(&d, &curved(&d), 1.0, 3)),
        ("blend_spline_fillet", fillet(&wavy, &wavy_top, 1.0, 2)),
    ]
}

fn extras() -> Vec<(&'static str, Result<Body>)> {
    let void = {
        let outer = box_solid(&Frame::translation(v3(0.0, 0.0, 0.0)), 40.0, 40.0, 40.0, 1).unwrap();
        let inner = sphere(&Frame::translation(v3(20.0, 20.0, 20.0)), 10.0, 2).unwrap();
        subtract(&outer, &inner)
    };
    let quartic = {
        let a = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
        let b = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
        union(&a, &b)
    };
    let two_lumps = {
        let a = box_solid(&Frame::translation(v3(0.0, 0.0, 0.0)), 30.0, 10.0, 10.0, 1).unwrap();
        let b = box_solid(&Frame::translation(v3(10.0, -5.0, -5.0)), 10.0, 20.0, 20.0, 2).unwrap();
        subtract(&a, &b)
    };
    let steinmetz = {
        let a = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
        let b = cylinder(&Frame::new(v3(-20.0, 0.0, 0.0), Vec3::X, Vec3::Y), 8.0, 40.0, 2).unwrap();
        intersect(&a, &b)
    };
    let cone_cut = {
        let a = cone_frustum(&Frame::WORLD, 12.0, 0.0, 24.0, 1).unwrap();
        let f = Frame::new(v3(0.0, 0.0, 8.0), v3(0.0, 0.5, 1.0), Vec3::X);
        let b = box_solid(&Frame { origin: f.to_world(v3(-50.0, -50.0, 0.0)), ..f }, 100.0, 100.0, 50.0, 2).unwrap();
        subtract(&a, &b)
    };
    let rounded_cube = {
        let b = box_solid(&Frame::WORLD, 20.0, 20.0, 20.0, 1).unwrap();
        let all: Vec<usize> = (0..b.edges.len()).collect();
        fillet(&b, &all, 4.0, 2)
    };
    let boss_flare = {
        let plate = box_solid(&Frame::translation(v3(-20.0, -20.0, 0.0)), 40.0, 40.0, 5.0, 1).unwrap();
        let boss = cylinder(&Frame::translation(v3(0.0, 0.0, 5.0)), 6.0, 10.0, 2).unwrap();
        let t = union(&plate, &boss).unwrap();
        let foot =
            t.edges_where(|i, _| (t.edge_midpoint(i).z - 5.0).abs() < 1e-9 && t.edge_is_arc_of_radius(i, 6.0))[0];
        fillet(&t, &[foot], 2.0, 3)
    };
    let chamfered_rim = {
        let p = Profile::new(Wire::rounded_rect(40.0, 30.0, 5.0));
        let plate = extrude(&p, &Frame::WORLD, 8.0, 1).unwrap();
        let rim = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - 8.0).abs() < 1e-9);
        chamfer(&plate, &rim, 1.5, 2)
    };
    let filleted_rim = {
        let p = Profile::new(Wire::rounded_rect(40.0, 30.0, 5.0));
        let plate = extrude(&p, &Frame::WORLD, 8.0, 1).unwrap();
        let rim = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - 8.0).abs() < 1e-9);
        fillet(&plate, &rim, 2.0, 2)
    };
    let rim = |corner: f64, height: f64, radius: f64| {
        let plate = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, corner)), &Frame::WORLD, height, 1).unwrap();
        let top = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - height).abs() < 1e-9);
        fillet(&plate, &top, radius, 2)
    };
    let chain = {
        let b = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
        let front = b.edges_where(|i, _| {
            let m = b.edge_midpoint(i);
            (m.z - 20.0).abs() < 1e-6 && m.y.abs() < 1e-6
        });
        let f = fillet(&b, &front, 4.0, 2).unwrap();
        let side = f.edges_where(|i, _| {
            let m = f.edge_midpoint(i);
            (m.z - 20.0).abs() < 1e-6 && m.x.abs() < 1e-6
        });
        fillet(&f, &side, 2.0, 3)
    };
    let many_holes = {
        let plate = box_solid(&Frame::WORLD, 100.0, 60.0, 6.0, 1).unwrap();
        let tools: Vec<Body> = (0..40)
            .map(|i| cylinder(&Frame::translation(v3(6.0 + (i % 8) as f64 * 12.0, 6.0 + (i / 8) as f64 * 12.0, -1.0)), 3.0, 8.0, 2).unwrap())
            .collect();
        boolean_many(&plate, &tools, BoolOp::Subtract)
    };
    let mirrored_part = {
        let part = union(
            &box_solid(&Frame::translation(v3(2.0, 0.0, 0.0)), 20.0, 10.0, 8.0, 1).unwrap(),
            &cylinder(&Frame::translation(v3(12.0, 5.0, 8.0)), 4.0, 6.0, 2).unwrap(),
        )
        .unwrap();
        Ok(part.mirrored(&Mirror::new(v3(1.0, 2.0, 3.0), v3(1.0, 0.5, 0.25))))
    };
    let mirror_of_curves = {
        let a = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
        let b = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
        Ok(union(&a, &b).unwrap().mirrored(&Mirror::new(v3(30.0, 0.0, 0.0), v3(1.0, 0.0, 0.0))))
    };
    let pipe = {
        let section = Profile::new(Wire::circle(v2(0.0, 0.0), 4.0)).with_hole(Wire::circle(v2(0.0, 0.0), 3.0));
        let path = [
            PathStep::Line { length: 20.0 },
            PathStep::Arc { radius: 15.0, angle: PI / 2.0, toward: 0.3 },
            PathStep::Line { length: 25.0 },
            PathStep::Arc { radius: 10.0, angle: 2.0, toward: 2.0 },
        ];
        sweep(&section, &Frame::WORLD, &path, 1)
    };
    let rounded = {
        let b = box_solid(&Frame::WORLD, 30.0, 20.0, 15.0, 1).unwrap();
        let all: Vec<usize> = (0..b.edges.len()).collect();
        fillet(&b, &all, 3.0, 2).unwrap()
    };
    let top_of = |b: &Body| b.faces.iter().position(|f| f.tag.slot == 1).unwrap();
    let tray = {
        let b = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 15.0, 1).unwrap();
        let top = top_of(&b);
        shell(&b, &[top], 2.0, 9)
    };
    let tapered = {
        let b = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)).with_hole(Wire::circle(v2(0.0, 0.0), 6.0)), &Frame::WORLD, 10.0, 1).unwrap();
        let walls: Vec<usize> = (0..b.faces.len()).filter(|&f| b.faces[f].tag.slot >= 2).collect();
        draft(&b, &walls, Vec3::Z, Vec3::ZERO, 4f64.to_radians())
    };
    let offset_traced = {
        let a = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
        let b = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
        offset(&union(&a, &b).unwrap(), 0.5)
    };
    vec![
        ("offset_out", offset(&rounded, 1.5)),
        ("offset_in", offset(&rounded, -1.5)),
        ("offset_traced", offset_traced),
        ("shell_closed", shell(&rounded, &[], 1.5, 9)),
        ("shell_tray", tray),
        ("draft_rounded", tapered),
        ("many_holes", many_holes),
        ("mirrored_part", mirrored_part),
        ("mirror_of_curves", mirror_of_curves),
        ("swept_pipe", pipe),
        ("rim_r_equals_corner", rim(8.0, 12.0, 8.0)),
        ("rim_r_above_corner", rim(8.0, 20.0, 9.0)),
        ("propagated_chain", chain),
        ("rounded_cube", rounded_cube),
        ("boss_flare", boss_flare),
        ("chamfered_rim", chamfered_rim),
        ("filleted_rim", filleted_rim),
        ("box", box_solid(&Frame::WORLD, 20.0, 30.0, 10.0, 1)),
        ("sphere", sphere(&Frame::WORLD, 15.0, 1)),
        ("torus", torus(&Frame::WORLD, 20.0, 5.0, 1)),
        ("cone", cone_frustum(&Frame::WORLD, 10.0, 0.0, 20.0, 1)),
        ("void_in_box", void),
        ("quartic_union", quartic),
        ("two_lumps", two_lumps),
        ("steinmetz", steinmetz),
        ("cone_oblique_cut", cone_cut),
    ]
}

fn main() {
    let dir = std::env::args().nth(1).expect("output directory");
    std::fs::create_dir_all(&dir).unwrap();
    let mut manifest = String::from("name,volume,area,faces,edges,vertices,shells\n");
    let mut bodies: Vec<(String, Body)> = Vec::new();
    for m in models::catalog() {
        let mut doc = m.document(0);
        let outcome = doc.rebuild(None);
        assert!(outcome.failed.is_none(), "{}: {:?}", m.id, outcome.steps);
        bodies.push((m.id.to_string(), (*outcome.body.unwrap()).clone()));
    }
    for (name, r) in extras().into_iter().chain(spline_extras()).chain(loft_extras()).chain(blend_extras()).chain(helix_extras()) {
        bodies.push((name.to_string(), r.unwrap()));
    }
    for (name, body) in &bodies {
        let props = mass_properties(body);
        let report = validate(body).unwrap();
        std::fs::write(format!("{dir}/{name}.step"), coreshape::export::step::write_step(body, name)).unwrap();
        let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
        std::fs::write(format!("{dir}/{name}.stl"), coreshape::export::stl::write_binary(&mesh, name)).unwrap();
        let _ = writeln!(
            manifest,
            "{name},{},{},{},{},{},{}",
            props.volume, props.area, report.faces, report.edges, report.vertices, report.shells
        );
    }
    for m in models::catalog() {
        std::fs::write(format!("{dir}/{}.coreshape", m.id), m.document(0).save_text()).unwrap();
    }
    std::fs::write(format!("{dir}/manifest.csv"), manifest).unwrap();
    println!("wrote {} models to {dir}", bodies.len());
}
