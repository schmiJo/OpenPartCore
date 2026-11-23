//! Times offsetting a demo out and in, as the demo tests do: offset_time [demo id]
use coreshape::*;
fn main() {
    let want = std::env::args().nth(1).unwrap_or_else(|| "text".into());
    for m in coreshape_wasm::models::catalog() {
        if m.id != want { continue; }
        let body = m.document(0).rebuild(None).body.unwrap();
        println!("{} faces", body.faces.len());
        for d in [0.25, -0.25] {
            let t = std::time::Instant::now();
            let r = offset(&body, d);
            println!("offset {d}: {:?} in {:.1} s", r.as_ref().map(|b| b.faces.len()).map_err(|e| e.to_string()), t.elapsed().as_secs_f64());
        }
    }
}
