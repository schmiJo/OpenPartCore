//! Writes every demo model as STEP and STL, with its exact volume and area, for
//! checking in another CAD system. Usage: gallery <output directory>
use coreshape::*;
use coreshape_wasm::models;
use std::fmt::Write;

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
    vec![
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
        let values: Vec<f64> = m.params.iter().map(|p| p.default).collect();
        let mut h = (m.history)();
        let outcome = h.rebuild(&values, None);
        assert!(outcome.failed.is_none(), "{}: {:?}", m.id, outcome.steps);
        bodies.push((m.id.to_string(), (*outcome.body.unwrap()).clone()));
    }
    for (name, r) in extras() {
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
    std::fs::write(format!("{dir}/manifest.csv"), manifest).unwrap();
    println!("wrote {} models to {dir}", bodies.len());
}
