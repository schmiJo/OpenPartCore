//! Each solid a STEP file brings in, where the file puts it, as CSV: valid, volume, area, the box of its mesh, and faces,
//! for comparing one by one with another reader's. Parts used many times are measured once and moved.
//!
//! cargo run --release -p coreshape --example step_solids -- <file.step> > ours.csv

use coreshape::import::step::read_step_each;
use coreshape::*;
use std::collections::HashMap;
use std::sync::Arc;

fn main() {
    let path = std::env::args().nth(1).expect("a STEP file");
    let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
    let read = read_step_each(&text, 1).unwrap();
    // With STEP_LIST set, just who each part is.
    if std::env::var("STEP_LIST").is_ok() {
        for k in 0..read.parts.len() {
            println!("{}", read.who(k));
        }
        return;
    }
    let mut known: HashMap<usize, (bool, MassProps, Vec<Vec3>)> = HashMap::new();
    for (k, part, at) in read.uses() {
        let key = Arc::as_ptr(&part) as usize;
        let (ok, m, points) = known.entry(key).or_insert_with(|| {
            let points = tessellate(&part, &TessParams::for_size(part.aabb().diagonal())).map(|m| m.positions).unwrap_or_default();
            (validate(&part).is_ok(), mass_properties(&part), points)
        });
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        for p in points.iter() {
            let q = at.to_world(*p);
            lo = lo.min(q);
            hi = hi.max(q);
        }
        let name = read.structure.names[read.structure.instances[k].0].replace([',', '\n', '\r'], " ");
        println!("{ok},{},{},{},{},{},{},{},{},{},{name}", m.volume, m.area, lo.x, lo.y, lo.z, hi.x, hi.y, hi.z, part.faces.len());
    }
    for note in read.notes() {
        eprintln!("{note}");
    }
}
