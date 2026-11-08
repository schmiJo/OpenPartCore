//! Times solving and dragging on sketches up to a thousand entities. Usage: bench
use coreshape_sketch::workloads::*;
use std::time::Instant;

fn main() {
    let start = Instant::now();
    let clock = move || start.elapsed().as_secs_f64() * 1e3;
    println!("{:<14} {:>8} {:>11} {:>5} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}", "workload", "entities", "constraints", "free", "solve", "analyze", "begin", "move med", "move p95", "worst");
    for w in ALL_WORKLOADS {
        for n in [50, 100, 200, 300, 500, 1000] {
            let t = measure(w, n, &clock);
            println!(
                "{:<14} {:>8} {:>11} {:>5} {:>7.2}ms {:>7.2}ms {:>7.2}ms {:>7.3}ms {:>7.3}ms {:>7.3}ms",
                t.workload, t.entities, t.constraints, t.dof, t.solve, t.analyze, t.begin, t.move_median, t.move_p95, t.move_worst
            );
        }
    }
}
