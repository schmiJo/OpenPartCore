//! Fitted fillets and chamfers on random curved edges. Usage: fuzz_blends [cases] [first seed]
//! Refusing is allowed. A result has to be valid, mesh closed, change the volume the right way by a believable amount,
//! keep every point away from the edge the same as before, and survive a Boolean.
use coreshape::boolean::classify::{BodyLoc, Index};
use coreshape::fuzz::*;
use coreshape::rolling::{chamfer_fitted, fillet_fitted};
use coreshape::*;
use std::collections::BTreeMap;
use std::time::Instant;

struct Tried {
    wrong: Vec<String>,
    refused: Option<String>,
    done: bool,
}

fn run(seed: u64) -> Tried {
    let mut out = Tried { wrong: Vec::new(), refused: None, done: false };
    let category = [Category::GeneralPrimitives, Category::SplineExtrusions, Category::Lofts][seed as usize % 3];
    let (Some(a), Some(b), mut rng) = make_pair(category, seed) else { return out };
    let op = [BoolOp::Union, BoolOp::Subtract, BoolOp::Intersect][rng.int(0, 2) as usize];
    let Ok(body) = boolean::boolean(&a, &b, op) else { return out };
    if body.faces.is_empty() {
        return out;
    }
    let uses = body.edge_uses();
    let sharp = |e: usize| -> bool {
        let [x, y] = uses[e].as_slice() else { return false };
        let p = body.edge_midpoint(e);
        let n = |f: usize| body.faces[f].surface.normal(p) * if body.faces[f].flipped { -1.0 } else { 1.0 };
        n(x.0).cross(n(y.0)).norm() > 0.05
    };
    let curved: Vec<usize> = (0..body.edges.len()).filter(|&e| !matches!(body.edges[e].curve, Curve::Line { .. } | Curve::Circle { .. }) && sharp(e)).collect();
    if curved.is_empty() {
        return out;
    }
    let edge = curved[rng.int(0, curved.len() as i64 - 1) as usize];
    let size = body.aabb().diagonal();
    let r = size * rng.range(0.004, 0.03);
    let is_fillet = rng.int(0, 1) == 0;
    let result = if is_fillet { fillet_fitted(&body, &[edge], r, 9) } else { chamfer_fitted(&body, &[edge], r, 9) };
    let result = match result {
        Ok(x) => x,
        Err(e) => {
            out.refused = Some(e.to_string());
            return out;
        }
    };
    out.done = true;
    let name = if is_fillet { "fillet" } else { "chamfer" };
    if let Err(e) = validate(&result) {
        out.wrong.push(format!("{name}: invalid: {:?}", e.first()));
        return out;
    }
    let mesh = match tessellate(&result, &TessParams::for_size(size)) {
        Ok(m) => m,
        Err(e) => {
            out.wrong.push(format!("{name}: no mesh: {e}"));
            return out;
        }
    };
    if mesh.open_edge_count() != 0 {
        out.wrong.push(format!("{name}: mesh not closed"));
    }
    let (before, after) = (mass_properties(&body), mass_properties(&result));
    if (mesh.volume() - after.volume).abs() > 0.01 * after.volume.abs().max(1.0) {
        out.wrong.push(format!("{name}: mesh {} against integrated {}", mesh.volume(), after.volume));
    }
    // It can only change material within reach of the edges it touched, whatever chain it grew into.
    let length: f64 = body.edges.iter().map(|e| (0..16).map(|k| { let f = |k: f64| e.curve.eval(e.t0 + (e.t1 - e.t0) * k / 16.0); f(k as f64).dist(f(k as f64 + 1.0)) }).sum::<f64>()).sum();
    if (after.volume - before.volume).abs() > 8.0 * r * r * length {
        out.wrong.push(format!("{name}: volume went from {} to {} with radius {r}", before.volume, after.volume));
    }
    // Far from every edge, membership doesn't change.
    let (old, new) = (Index::new(&body), Index::new(&result));
    let bb = body.aabb();
    let reach = 4.0 * r;
    let mut sampled = 0;
    for _ in 0..4000 {
        let p = v3(rng.range(bb.min.x, bb.max.x), rng.range(bb.min.y, bb.max.y), rng.range(bb.min.z, bb.max.z));
        let close = body.edges.iter().any(|e| (0..=48).any(|k| e.curve.eval(e.t0 + (e.t1 - e.t0) * k as f64 / 48.0).dist(p) < reach + (e.t1 - e.t0) * e.curve.speed_bound() / 48.0));
        if close {
            continue;
        }
        sampled += 1;
        if sampled > 60 {
            break;
        }
        let (x, y) = (old.locate_in_body(p), new.locate_in_body(p));
        if matches!(x, BodyLoc::OnFace(_)) || matches!(y, BodyLoc::OnFace(_)) {
            continue;
        }
        if x != y {
            out.wrong.push(format!("{name}: {p:?} far from the edge went from {x:?} to {y:?}"));
            break;
        }
    }
    // And it's still something the rest of the kernel can work with.
    let tool = sphere(&Frame::translation(bb.center() + rng.unit_vector() * (0.1 * size)), 0.17 * size, 77).unwrap();
    if let Ok(cut) = boolean::boolean(&result, &tool, BoolOp::Subtract) {
        if let Err(e) = validate(&cut) {
            out.wrong.push(format!("{name}: a cut after it is invalid: {:?}", e.first()));
        }
    }
    out
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("one") {
        let seed = args[2].parse().unwrap();
        let r = run(seed);
        println!("seed {seed}: done {} refused {:?} wrong {:?}", r.done, r.refused, r.wrong);
        return;
    }
    let cases: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(120);
    let first: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()) as u64;
    let t = Instant::now();
    let mut handles = Vec::new();
    for w in 0..threads {
        handles.push(std::thread::spawn(move || {
            let mut results = Vec::new();
            let mut seed = first + w;
            while seed < first + cases {
                let tried = std::panic::catch_unwind(|| run(seed)).unwrap_or_else(|_| Tried { wrong: vec!["PANIC".to_string()], refused: None, done: false });
                results.push((seed, tried));
                seed += threads;
            }
            results
        }));
    }
    let mut all: Vec<(u64, Tried)> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
    all.sort_by_key(|r| r.0);
    let (mut done, mut refused, mut wrong) = (0, 0, 0);
    let mut reasons: BTreeMap<String, (usize, u64)> = BTreeMap::new();
    for (seed, r) in &all {
        done += usize::from(r.done);
        if let Some(why) = &r.refused {
            refused += 1;
            let key: String = why.chars().take(58).collect();
            reasons.entry(key).or_insert((0, *seed)).0 += 1;
        }
        if !r.wrong.is_empty() {
            wrong += 1;
            println!("WRONG seed {seed}: {}", r.wrong.join(" | "));
        }
    }
    println!("{} seeds, {done} blended, {refused} refused, {wrong} wrong, {:.0} s", all.len(), t.elapsed().as_secs_f64());
    let mut list: Vec<_> = reasons.into_iter().collect();
    list.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
    for (why, (n, seed)) in list {
        println!("  {n:>4} x {why} (first seed {seed})");
    }
}
