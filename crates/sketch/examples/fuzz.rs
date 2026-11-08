//! Fuzz run with a printed report. Usage: fuzz [cases per category] [first seed]
use coreshape_sketch::fuzz::*;
use std::collections::BTreeMap;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let cases: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(500);
    let first: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()) as u64;
    std::panic::set_hook(Box::new(|_| {}));
    println!("{:<13} {:>6} {:>8} {:>6} {:>10}", "category", "cases", "refused", "wrong", "time");
    let (mut total_refused, mut total_wrong, mut total_cases) = (0, 0, 0);
    // ONLY_CATEGORY=name runs just that one.
    let only = std::env::var("ONLY_CATEGORY").ok();
    for category in ALL_CATEGORIES {
        if only.as_deref().is_some_and(|o| o != category.name()) {
            continue;
        }
        let t = Instant::now();
        let handles: Vec<_> = (0..threads)
            .map(|w| {
                std::thread::spawn(move || {
                    let mut results = Vec::new();
                    let mut seed = first + w;
                    while seed < first + cases {
                        let out = std::panic::catch_unwind(|| check_case(&generate(category, seed)))
                            .unwrap_or_else(|_| Outcome { refusals: Vec::new(), wrong: vec!["PANIC".to_string()] });
                        if !out.refusals.is_empty() || !out.wrong.is_empty() {
                            results.push((seed, out));
                        }
                        seed += threads;
                    }
                    results
                })
            })
            .collect();
        let mut results: Vec<(u64, Outcome)> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
        results.sort_by_key(|r| r.0);
        let refused = results.iter().filter(|r| !r.1.refusals.is_empty()).count();
        let wrong = results.iter().filter(|r| !r.1.wrong.is_empty()).count();
        println!("{:<13} {:>6} {:>8} {:>6} {:>9.2}s", category.name(), cases, refused, wrong, t.elapsed().as_secs_f64());
        total_refused += refused;
        total_wrong += wrong;
        total_cases += cases;
        let mut reasons: BTreeMap<String, (usize, u64)> = BTreeMap::new();
        for (seed, out) in &results {
            for m in out.wrong.iter().map(|w| format!("WRONG {w}")).chain(out.refusals.iter().cloned()) {
                let key: String = m.split(|c: char| c.is_ascii_digit()).collect::<Vec<_>>().join("#").chars().take(90).collect();
                let e = reasons.entry(key).or_insert((0, *seed));
                e.0 += 1;
            }
        }
        let mut list: Vec<_> = reasons.into_iter().collect();
        list.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
        for (why, (n, seed)) in list.into_iter().take(5) {
            println!("    {n:>5} x {why}   (seed {seed})");
        }
    }
    println!("total: {total_cases} cases, {total_refused} refused, {total_wrong} wrong");
}
