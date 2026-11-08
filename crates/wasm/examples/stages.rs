//! Where a slider update's time goes, for every demo: the rebuild, meshing at each density,
//! validation and mass properties. Usage: stages
use coreshape::*;
use coreshape_wasm::models;
use std::time::Instant;

fn timed<T>(f: impl FnOnce() -> T) -> (T, f64) {
    let t = Instant::now();
    let v = f();
    (v, t.elapsed().as_secs_f64() * 1e3)
}

fn main() {
    println!("{:<8} {:<7} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>7} {:>7}", "model", "slider", "rebuild", "", "tess", "tess30", "valid", "mass", "tris", "tris30");
    for m in models::catalog() {
        if m.params.iter().any(|p| !p.choices.is_empty()) {
            continue;
        }
        let values = m.defaults();
        for (name, which) in [("first", 0), ("last", values.len() - 1)] {
            let mut nudged = values.clone();
            let p = &m.params[which];
            nudged[which] += if values[which] + p.step <= p.max { p.step } else { -p.step };
            // Each measured from a settled document, as a drag's next step would be.
            let mut doc = m.document(0);
            m.apply(&mut doc, &values);
            doc.rebuild(None);
            m.apply(&mut doc, &nudged);
            let (o, full) = timed(|| doc.rebuild(None));
            let body = o.body;
            let Some(body) = body else {
                println!("{:<8} {:<7} refused", m.id, name);
                continue;
            };
            let size = body.aabb().diagonal();
            let (mesh, tess) = timed(|| tessellate(&body, &TessParams::for_size(size)).unwrap());
            let (coarse, tess30) = timed(|| tessellate(&body, &TessParams { chord: size * 1.2e-3, angle: 30f64.to_radians() }).unwrap());
            let (_, valid) = timed(|| validate(&body).unwrap());
            let (_, mass) = timed(|| mass_properties(&body));
            println!(
                "{:<8} {:<7} {:>6.1}ms {:>8} {:>6.1}ms {:>6.1}ms {:>6.1}ms {:>6.1}ms {:>7} {:>7}",
                m.id, name, full, "", tess, tess30, valid, mass, mesh.triangles.len(), coarse.triangles.len()
            );
        }
    }
}
