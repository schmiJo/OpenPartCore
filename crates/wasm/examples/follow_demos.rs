//! Follows every pick in every demo as each slider moves across its whole range, with the demos' references
//! rewritten first into what an app saves when each one is clicked, and says which picks moved to something else
//! without the feature saying so. Counts are left out, as every copy moves at once: those have tests of their own.
//!
//! cargo run --release -p coreshape-wasm --example follow_demos -- [model id] [steps each way]

use coreshape_document::follow::{as_clicked, counts, follow, Tally};
use coreshape_wasm::models;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let only = args.get(1).cloned().unwrap_or_default();
    let steps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(12);
    let mut all = Tally::default();
    for m in models::catalog() {
        if !only.is_empty() && m.id != only {
            continue;
        }
        let defaults: Vec<f64> = m.params.iter().map(|p| p.default).collect();
        let variant = m.variant(&defaults);
        let mut doc = m.document(variant);
        m.apply(&mut doc, &defaults);
        let (doc, rewritten) = match as_clicked(&doc) {
            Ok(x) => x,
            Err(e) => {
                println!("{}: can't rewrite: {e}", m.id);
                continue;
            }
        };
        let counting = counts(&doc);
        let mut model = Tally::default();
        for p in m.params.iter().filter(|p| p.choices.is_empty() && !counting.iter().any(|c| c == p.key)) {
            let side = |to: f64| -> Vec<f64> {
                let n = (((to - p.default).abs() / p.step).round() as usize).clamp(1, steps);
                (1..=n).map(|k| p.default + (to - p.default) * k as f64 / n as f64).filter(|v| (*v - p.default).abs() > 1e-12).collect()
            };
            model.add(&follow(&doc, p.key, &side(p.min), &side(p.max), m.id));
        }
        println!("{}: {rewritten} references rewritten; {}", m.id, model.summary());
        for line in &model.lines {
            println!("    {line}");
        }
        all.add(&model);
    }
    println!("all: {}", all.summary());
}
