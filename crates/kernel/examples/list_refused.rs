//! Lists every refused case in a category. Usage: list_refused <gen|al> <cases>
use coreshape::fuzz::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let category = if args[1] == "gen" { Category::GeneralPrimitives } else { Category::AlignedPrimitives };
    let cases: u64 = args[2].parse().unwrap();
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()) as u64;
    let mut handles = Vec::new();
    for w in 0..threads {
        handles.push(std::thread::spawn(move || {
            let mut found = Vec::new();
            let mut seed = w;
            while seed < cases {
                let out = run_case(category, seed, 0);
                if let Some((_, why)) = out.refusals.first() {
                    found.push((seed, why.clone()));
                }
                seed += threads;
            }
            found
        }));
    }
    let mut all: Vec<(u64, String)> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
    all.sort();
    for (seed, why) in all {
        println!("{seed}: {}", why.chars().take(110).collect::<String>());
    }
}
