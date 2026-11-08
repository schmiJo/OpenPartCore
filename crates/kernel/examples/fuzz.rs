//! Fuzz run with a printed report. Usage: fuzz [cases per category] [first seed]
use coreshape::fuzz::*;
use std::collections::BTreeMap;
use std::time::Instant;

fn main() {
    std::panic::set_hook(Box::new(|_| {}));
    let args: Vec<String> = std::env::args().collect();
    let cases: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(500);
    let first: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());

    println!("{:<20} {:>6} {:>8} {:>8} {:>10}", "category", "cases", "refused", "wrong", "time");
    for category in ALL_CATEGORIES {
        let t = Instant::now();
        let mut handles = Vec::new();
        for w in 0..threads as u64 {
            handles.push(std::thread::spawn(move || {
                let mut results = Vec::new();
                let mut seed = first + w;
                while seed < first + cases {
                    // The kernel must never panic. Report it as a wrong result.
                    let out = std::panic::catch_unwind(|| run_case(category, seed, 150)).unwrap_or_else(|_| Outcome {
                        refusals: Vec::new(),
                        wrong: vec!["PANIC: the kernel panicked".to_string()],
                    });
                    if !out.refusals.is_empty() || !out.wrong.is_empty() {
                        results.push((seed, out));
                    }
                    seed += threads as u64;
                }
                results
            }));
        }
        let mut results: Vec<(u64, Outcome)> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
        results.sort_by_key(|r| r.0);
        let refused = results.iter().filter(|r| !r.1.refusals.is_empty()).count();
        let wrong = results.iter().filter(|r| !r.1.wrong.is_empty()).count();
        println!("{:<20} {:>6} {:>8} {:>8} {:>9.1}s", format!("{category:?}"), cases, refused, wrong, t.elapsed().as_secs_f64());

        let mut reasons: BTreeMap<String, (usize, u64)> = BTreeMap::new();
        for (seed, out) in &results {
            for (_, why) in &out.refusals {
                let key: String = why.chars().take(90).collect();
                let e = reasons.entry(format!("refused: {key}")).or_insert((0, *seed));
                e.0 += 1;
            }
            for why in &out.wrong {
                let key: String = why.split(':').take(2).collect::<Vec<_>>().join(":").chars().take(90).collect();
                let e = reasons.entry(format!("WRONG:   {key}")).or_insert((0, *seed));
                e.0 += 1;
            }
        }
        let mut sorted: Vec<_> = reasons.into_iter().collect();
        sorted.sort_by_key(|r| (!r.0.starts_with("WRONG"), std::cmp::Reverse(r.1 .0)));
        for (why, (n, seed)) in sorted.into_iter().take(16) {
            println!("    {n:>5} x {why}  (first seed {seed})");
        }
    }
}
