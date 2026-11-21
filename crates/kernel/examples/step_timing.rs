//! How long each stage of reading a STEP file takes, as its progress reports it, and what was left out or repaired.
//! Usage: step_timing <file.step>
use coreshape::import::step::read_step_each;
use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).expect("a STEP file");
    let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
    let start = Instant::now();
    let mut stage = ("", Instant::now());
    coreshape::progress::set_hook(move |p| {
        if p.stage != stage.0 {
            if !stage.0.is_empty() {
                println!("{}: {:.1} s", stage.0, stage.1.elapsed().as_secs_f64());
            }
            stage = (p.stage, Instant::now());
        }
        if p.done == p.total {
            println!("{}: {:.1} s", p.stage, stage.1.elapsed().as_secs_f64());
        }
    });
    let read = read_step_each(&text, 1).unwrap();
    coreshape::progress::clear();
    let distinct: std::collections::HashSet<usize> = read.parts.iter().filter_map(|p| p.as_ref().ok()).map(|b| std::sync::Arc::as_ptr(b) as usize).collect();
    println!("all: {:.1} s; {} parts, {} distinct, {} uses", start.elapsed().as_secs_f64(), read.parts.len(), distinct.len(), read.structure.instances.len());
    for note in read.notes() {
        println!("  {note}");
    }
}
