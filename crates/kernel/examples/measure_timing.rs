//! Times measuring each distinct solid of a STEP file: its exact volume, its check and its display mesh, slowest first.
//!
//! cargo run --release -p coreshape --example measure_timing -- <file.step> [how many to list] [only the slowest: --one]

use coreshape::*;
use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).expect("a STEP file");
    let list: usize = std::env::args().nth(2).and_then(|a| a.parse().ok()).unwrap_or(8);
    let one = std::env::args().any(|a| a == "--one");
    let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
    let read = coreshape::import::step::read_step_each(&text, 1).unwrap();
    let mut distinct: Vec<std::sync::Arc<Body>> = Vec::new();
    for b in read.parts.iter().flatten() {
        if !distinct.iter().any(|d| std::sync::Arc::ptr_eq(d, b)) {
            distinct.push(b.clone());
        }
    }
    println!("{} distinct solids", distinct.len());
    // Just one, by its number in this list.
    if let Some(k) = std::env::args().find_map(|a| a.strip_prefix("--solid=").and_then(|n| n.parse::<usize>().ok())) {
        let t = Instant::now();
        let v = mass_properties(&distinct[k]).volume;
        println!("solid {k}: volume {v} in {:.2} s", t.elapsed().as_secs_f64());
        let t = Instant::now();
        let m = tessellate(&distinct[k], &TessParams::for_size(distinct[k].aabb().diagonal())).map(|m| m.triangles.len());
        println!("solid {k}: mesh {m:?} in {:.2} s", t.elapsed().as_secs_f64());
        return;
    }
    let mut times: Vec<(usize, f64, f64, f64)> = Vec::new();
    for (k, b) in distinct.iter().enumerate() {
        let t = Instant::now();
        let v = mass_properties(b).volume;
        let volume = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let _ = validate(b);
        let check = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let _ = tessellate(b, &TessParams::for_size(b.aabb().diagonal()));
        let mesh = t.elapsed().as_secs_f64();
        let _ = v;
        times.push((k, volume, check, mesh));
    }
    times.sort_by(|a, b| (b.1 + b.2 + b.3).total_cmp(&(a.1 + a.2 + a.3)));
    let total: f64 = times.iter().map(|t| t.1 + t.2 + t.3).sum();
    println!("all: {total:.1} s");
    for (k, volume, check, mesh) in times.iter().take(list) {
        let b = &distinct[*k];
        let free = b.faces.iter().filter(|f| matches!(f.surface, Surface::Free { .. })).count();
        let sampled = b.edges.iter().filter(|e| matches!(e.curve, Curve::Sampled(_))).count();
        println!("solid {k}: {} faces ({free} free-form), {} edges ({sampled} traced): volume {volume:.2} s, check {check:.2} s, mesh {mesh:.2} s", b.faces.len(), b.edges.len());
    }
    if one {
        let b = &distinct[times[0].0];
        let t = Instant::now();
        for _ in 0..3 {
            mass_properties(b);
        }
        println!("the slowest's volume three times: {:.2} s", t.elapsed().as_secs_f64());
    }
}
