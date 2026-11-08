//! Replays one random case and explains the result. Usage: replay <category> <seed> [--json]
use coreshape_sketch::fuzz::*;
use coreshape_sketch::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let category = Category::parse(&args[1]).expect("a category name");
    let seed: u64 = args[2].parse().unwrap();
    let case = generate(category, seed);
    println!("{} entities, {} constraints; expect {:?}", case.start.entities.len(), case.start.constraints.len(), case.expect);
    println!("starts up to {:.3} from a true arrangement", largest_move(&case.start, &case.truth));
    let out = check_case(&case);
    println!("refusals: {:?}\nwrong: {:?}", out.refusals, out.wrong);
    match case.start.solve() {
        Ok(s) => println!("solved: {} free, {} redundant, {} iterations", s.analysis.dof, s.analysis.redundant.len(), s.iterations),
        Err(e) => {
            println!("failed: {:?}: {}", e.kind, e.message);
            // Just the named constraints, from where the case started and from the truth.
            let keep = |s: &Sketch| {
                let mut t = s.clone();
                t.constraints.retain(|c| e.constraints.contains(&c.id));
                t
            };
            println!("the named constraints alone, from the start: {:?}", keep(&case.start).solve().map(|s| s.analysis.dof).map_err(|x| x.message));
            println!("the named constraints alone, from the truth: {:?}", keep(&case.truth).solve().map(|s| s.analysis.dof).map_err(|x| x.message));
            let touched: Vec<EntityId> = case.start.constraints.iter().filter(|c| e.constraints.contains(&c.id)).flat_map(|c| c.rule.entities()).collect();
            for id in touched.iter().collect::<std::collections::BTreeSet<_>>() {
                if let (Some(a), Some(b)) = (case.start.entity(*id), case.truth.entity(*id)) {
                    println!("  {:>4} start {:?}\n       truth {:?}", id.0, a.geometry, b.geometry);
                }
            }
            for c in case.start.constraints.iter().filter(|c| e.constraints.contains(&c.id)) {
                println!("  c{} {:?}", c.id.0, c.rule);
            }
        }
    }
    if args.get(3).map(String::as_str) == Some("--json") {
        println!("{}", case.start.save_text());
    }
}
