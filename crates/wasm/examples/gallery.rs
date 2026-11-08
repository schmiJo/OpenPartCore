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
    for (name, r) in extras().into_iter().chain(spline_extras()) {
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
