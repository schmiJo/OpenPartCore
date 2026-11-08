//! Scans random slider positions on every demo and reports the refusals.
//! Usage: scan [positions per model]
use coreshape::*;
use coreshape_wasm::models;
use coreshape_document::Document;
use std::collections::{BTreeMap, HashMap};

fn main() {
    let runs: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(200);
    let mut rng = Rng::new(2026);
    println!("{:<10} {:>9} {:>8} {:>8}", "model", "positions", "refused", "invalid");
    for m in models::catalog() {
        let mut docs: HashMap<usize, Document> = HashMap::new();
        let mut refused = 0;
        let mut invalid = 0;
        let mut reasons: BTreeMap<String, (usize, String, Vec<f64>)> = BTreeMap::new();
        for _ in 0..runs {
            let values: Vec<f64> = m
                .params
                .iter()
                .map(|p| {
                    let steps = ((p.max - p.min) / p.step).round();
                    p.min + rng.int(0, steps as i64) as f64 * p.step
                })
                .collect();
            let variant = m.variant(&values);
            let doc = docs.entry(variant).or_insert_with(|| m.document(variant));
            m.apply(doc, &values);
            let o = doc.rebuild(None);
            if let Some(b) = &o.body {
                if validate(b).is_err() {
                    invalid += 1;
                }
            }
            if let Some(i) = o.failed {
                refused += 1;
                if let coreshape_document::StepState::Failed(why) = &o.steps[i].state {
                    let key: String = why.chars().take(70).collect();
                    let e = reasons.entry(key).or_insert((0, o.steps[i].name.clone(), values.clone()));
                    e.0 += 1;
                }
            }
        }
        println!("{:<10} {:>9} {:>8} {:>8}", m.id, runs, refused, invalid);
        let mut list: Vec<_> = reasons.into_iter().collect();
        list.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
        for (why, (n, feature, values)) in list.into_iter().take(4) {
            println!("    {n:>4} x {feature}: {why}\n           e.g. p={values:?}");
        }
    }
}
