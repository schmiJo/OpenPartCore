//! Real parts, modelled the way a person does, through the tools an app uses. Each script in `real_parts/` is checked on its
//! own terms, by formulas and what it should look like, and against `real_parts/expected.json`, whose volumes are
//! OpenCASCADE's reading of the STEP file where it read it, and which says what's known not to work. So this fails if a part
//! that worked stops working or changes, and also if one that didn't starts to, which is the time to update the file:
//! `scripts/check_real_parts.sh <folder>` makes the table, and `real_parts_table.py <folder> --expected <file>` the file.

#[path = "real_parts/common.rs"]
#[allow(dead_code)]
mod common;

use coreshape_document::json::{self, Value};
use std::path::Path;
use std::sync::Mutex;

const FOLDER: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/real_parts");

/// How far a volume can be from the recorded one. OpenCASCADE and this kernel integrate differently, and the table holds them
/// to the same when it's made.
const TOLERANCE: f64 = 1e-4;

/// The parts that take minutes, as cutting a thread does. `cargo test -- --ignored` runs them.
fn slow(stem: &str) -> bool {
    ["p25_", "p26_", "p29_", "p41_"].iter().any(|p| stem.starts_with(p))
}

fn differences(stem: &str, result: &common::PartResult, expected: Option<&Value>) -> Vec<String> {
    let mut out = Vec::new();
    let Some(expected) = expected else {
        return vec![format!("{stem} isn't in expected.json")];
    };
    let trouble: Vec<String> = result
        .steps
        .iter()
        .filter(|s| !matches!(s.outcome, common::Outcome::Ok | common::Outcome::Followed(_)))
        .map(|s| s.label.clone())
        .collect();
    let known: Vec<String> = expected.get("trouble").and_then(Value::as_array).unwrap_or(&[]).iter().filter_map(Value::as_str).map(str::to_string).collect();
    for t in &trouble {
        if !known.contains(t) {
            let why = result.steps.iter().find(|s| &s.label == t).map_or(String::new(), |s| format!("{:?}", s.outcome));
            out.push(format!("{stem}: {t} has started to fail: {why}"));
        }
    }
    for k in &known {
        if !trouble.contains(k) {
            out.push(format!("{stem}: {k} used to fail and now works, so expected.json should say so"));
        }
    }
    if trouble.is_empty() && known.is_empty() {
        if let (Some(got), Some(want)) = (result.volume, expected.get("volume").and_then(Value::as_f64)) {
            if (got - want).abs() > TOLERANCE * want.abs().max(1e-9) {
                out.push(format!("{stem}: the volume is {got} and was {want}"));
            }
        }
    }
    out
}

fn run_all(which: impl Fn(&str) -> bool) {
    let expected = json::parse(&std::fs::read_to_string(Path::new(FOLDER).join("expected.json")).expect("expected.json")).expect("expected.json is JSON");
    let work: Vec<(String, Value)> = common::scripts(Path::new(FOLDER)).into_iter().filter(|(s, _)| which(s)).collect();
    assert!(!work.is_empty());
    let root = std::env::temp_dir().join(format!("coreshape-real-parts-{}", std::process::id()));
    std::fs::create_dir_all(&root).unwrap();
    let next = Mutex::new(0usize);
    let found: Mutex<Vec<String>> = Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..3 {
            scope.spawn(|| loop {
                let at = {
                    let mut n = next.lock().unwrap();
                    *n += 1;
                    *n - 1
                };
                let Some((stem, script)) = work.get(at) else { break };
                let result = common::run(script, &root, stem, false);
                found.lock().unwrap().extend(differences(stem, &result, expected.get(stem)));
            });
        }
    });
    let found = found.into_inner().unwrap();
    assert!(found.is_empty(), "{}", found.join("\n"));
}

#[test]
fn real_parts_do_what_they_did() {
    run_all(|stem| !slow(stem));
}

#[test]
#[ignore = "takes minutes: threads and springs"]
fn real_parts_that_take_minutes_do_what_they_did() {
    run_all(slow);
}
