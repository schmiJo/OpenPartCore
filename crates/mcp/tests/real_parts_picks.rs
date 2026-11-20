//! The real parts' references, rewritten into what an app saves when each face, edge and corner is clicked, followed by
//! continuity as their dimensions move: none may move to something else, or be left out, while its feature builds. See
//! `coreshape_document::follow`. The whole set takes a while, so the test that runs every time takes the parts that
//! went wrong before picks followed what they touch (2026-11-20), and `cargo test -- --ignored` takes them all.
//! `cargo run --release -p coreshape-document --example follow_picks` prints the whole table.

use coreshape_document::follow::{as_clicked, counts, follow, without_refusals, Tally};
use coreshape_document::json::{self, Value};
use coreshape_document::{Document, FORMAT, FORMAT_VERSION};
use std::path::Path;

const FOLDER: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/real_parts");

/// A real part's script as a document: its parameters and its features, as they're saved.
fn document_of(script: &Value) -> Document {
    let params: Vec<Value> = match script.get("parameters") {
        Some(Value::Obj(items)) => items.iter().map(|(k, v)| Value::obj(vec![("name", Value::str(k)), ("value", v.clone())])).collect(),
        _ => Vec::new(),
    };
    let mut features = Vec::new();
    for step in script.get("steps").and_then(Value::as_array).unwrap_or(&[]) {
        if step.get("expect").is_some() || step.get("tool").is_some() {
            continue;
        }
        let Value::Obj(items) = step else { continue };
        let mut items: Vec<(String, Value)> = items.iter().filter(|(k, _)| k != "note").cloned().collect();
        if !items.iter().any(|(k, _)| k == "name") {
            items.push(("name".into(), Value::str("")));
        }
        features.push(Value::Obj(items));
    }
    let n = features.len();
    Document::from_json(&Value::obj(vec![
        ("format", Value::str(FORMAT)),
        ("version", Value::Num(FORMAT_VERSION)),
        ("units", Value::str("mm")),
        ("next_id", Value::Num(n as f64 + 1.0)),
        ("parameters", Value::Arr(params)),
        ("features", Value::Arr(features)),
    ]))
    .unwrap()
}

/// Follows every pick of the parts whose names start with one of `which`, each dimension moving a quarter either way in
/// `steps` steps.
fn follow_parts(which: &[&str], steps: usize) -> Tally {
    let mut names: Vec<_> = std::fs::read_dir(Path::new(FOLDER)).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).collect();
    names.sort();
    let mut all = Tally::default();
    for path in names {
        let stem = path.file_stem().unwrap().to_string_lossy().to_string();
        if !stem.starts_with('p') || path.extension().is_none_or(|x| x != "json") || !which.iter().any(|w| stem.starts_with(w)) {
            continue;
        }
        let script = json::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let mut doc = document_of(&script);
        without_refusals(&mut doc);
        let (doc, _) = as_clicked(&doc).unwrap();
        let counting = counts(&doc);
        let Some(Value::Obj(params)) = script.get("parameters") else { continue };
        for (name, value) in params {
            let Some(base) = value.as_f64() else { continue };
            if counting.contains(name) {
                continue;
            }
            let step = base.abs() * 0.25 / steps as f64;
            let down: Vec<f64> = (1..=steps).map(|k| base - step * k as f64).collect();
            let up: Vec<f64> = (1..=steps).map(|k| base + step * k as f64).collect();
            all.add(&follow(&doc, name, &down, &up, &stem));
        }
    }
    all
}

fn none_lost(t: &Tally) {
    eprintln!("{}", t.summary());
    for line in &t.lines {
        eprintln!("  {line}");
    }
    assert!(t.moved == 0 && t.dropped == 0, "{}", t.summary());
}

#[test]
fn picks_in_the_parts_that_lost_them_follow() {
    // A weld neck flange and a bushing, both turned, whose whole round faces were two halves that could swap, and a
    // sprocket whose tooth tips run together into one face as the seats move in.
    none_lost(&follow_parts(&["p08_", "p21_", "p47_"], 4));
}

#[test]
#[ignore = "takes many minutes: every real part"]
fn picks_in_every_real_part_follow() {
    none_lost(&follow_parts(&["p"], 10));
}
