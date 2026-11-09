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
        let mut mechanism = m.mechanism();
        let (mut stopped, mut slowest) = (0, 0.0f64);
        for _ in 0..runs {
            let values: Vec<f64> = m
                .params
                .iter()
                .map(|p| {
                    let steps = ((p.max - p.min) / p.step).round();
                    p.min + rng.int(0, steps as i64) as f64 * p.step
                })
                .collect();
            // A mechanism counts as refused if a mate fails. One the others stop short of where it's asked isn't refused: that's
            // its range of motion, and it says so.
            if let Some(moving) = &mut mechanism {
                let start = std::time::Instant::now();
                let (solved, asked) = m.move_mechanism(moving, &values).expect("the tabs are there");
                slowest = slowest.max(start.elapsed().as_secs_f64());
                let reached = solved.values.iter().find(|v| v.0 == moving.drive && v.1 == coreshape_assembly::Dof::Angle).map_or(f64::NAN, |v| v.2);
                if (reached - asked).abs() > 1e-5 {
                    stopped += 1;
                }
                let wrong: Vec<String> = solved.failing.iter().map(|f| f.1.clone()).chain(solved.problems.iter().cloned()).collect();
                if let Some(why) = wrong.first() {
                    refused += 1;
                    let key: String = why.chars().take(70).collect();
                    reasons.entry(key).or_insert((0, "assembly".to_string(), values.clone())).0 += 1;
                }
                if solved.whole().is_some_and(|b| validate(&b).is_err()) {
                    invalid += 1;
                }
                continue;
            }
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
        if mechanism.is_some() {
            println!("    {stopped} stopped short of what was asked, and the slowest solve took {:.1} ms", slowest * 1e3);
        }
        let mut list: Vec<_> = reasons.into_iter().collect();
        list.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
        for (why, (n, feature, values)) in list.into_iter().take(4) {
            println!("    {n:>4} x {feature}: {why}\n           e.g. p={values:?}");
        }
    }
}
