//! Times each stage of a rebuild for every demo model.
use coreshape::*;
use coreshape_wasm::models;
use std::time::Instant;

fn main() {
    println!(
        "{:<12} {:>9} {:>9} {:>8} {:>9} {:>6} {:>10}",
        "model", "first", "last dim", "mass", "tess", "faces", "triangles"
    );
    for m in models::catalog() {
        let mut values: Vec<f64> = m.params.iter().map(|p| p.default).collect();
        let mut doc = m.document(0);
        let t = Instant::now();
        let o = doc.rebuild(None);
        let first = t.elapsed();
        assert!(o.failed.is_none(), "{}: {:?}", m.id, o.steps);
        let body = o.body.unwrap();
        // Nudge the last parameter, as dragging its slider would.
        let last = values.len() - 1;
        values[last] += m.params[last].step;
        let t = Instant::now();
        // A list-valued parameter picks a document instead of changing a dimension.
        m.apply(&mut doc, &values);
        let _ = doc.rebuild(None);
        let again = t.elapsed();
        let t = Instant::now();
        let _ = mass_properties(&body);
        let mass = t.elapsed();
        let t = Instant::now();
        let mesh = tessellate(&body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
        let tess = t.elapsed();
        let ms = |d: std::time::Duration| d.as_secs_f64() * 1e3;
        println!(
            "{:<12} {:>7.1}ms {:>7.1}ms {:>6.1}ms {:>7.1}ms {:>6} {:>10}",
            m.id,
            ms(first),
            ms(again),
            ms(mass),
            ms(tess),
            body.faces.len(),
            mesh.triangles.len()
        );
    }
}
