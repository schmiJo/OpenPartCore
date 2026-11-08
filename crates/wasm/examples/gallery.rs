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
    vec![
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
        bodies.push((m.id.to_string(), (m.build)(&values).unwrap()));
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
        let _ = writeln!(manifest, "{name},{},{},{},{},{},{}", props.volume, props.area, report.faces, report.edges, report.vertices, report.shells);
    }
    std::fs::write(format!("{dir}/manifest.csv"), manifest).unwrap();
    println!("wrote {} models to {dir}", bodies.len());
}
