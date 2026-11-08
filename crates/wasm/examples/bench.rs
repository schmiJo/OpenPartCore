//! Times each stage of a rebuild for every demo model.
use coreshape::*;
use coreshape_wasm::models;
use std::time::Instant;

fn main() {
    println!("{:<18} {:>8} {:>8} {:>8} {:>8} {:>9}", "model", "build", "validate", "mass", "tess", "triangles");
    for m in models::catalog() {
        let values: Vec<f64> = m.params.iter().map(|p| p.default).collect();
        let t = Instant::now();
        let body = (m.build)(&values).unwrap();
        let build = t.elapsed();
        let t = Instant::now();
        validate(&body).unwrap();
        let val = t.elapsed();
        let t = Instant::now();
        let _ = mass_properties(&body);
        let mass = t.elapsed();
        let t = Instant::now();
        let mesh = tessellate(&body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
        let tess = t.elapsed();
        let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;
        println!(
            "{:<18} {:>6.2}ms {:>6.2}ms {:>6.2}ms {:>6.2}ms {:>9}",
            m.id, ms(build), ms(val), ms(mass), ms(tess), mesh.triangles.len()
        );
    }
}
