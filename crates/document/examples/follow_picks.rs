//! Follows every pick in the real parts as each of their dimensions moves, and says which picks moved to something
//! else without the feature saying so. The parts' references are rewritten first into what an app saves when each one
//! is clicked.
//!
//! cargo run --release -p coreshape-document --example follow_picks -- [part name filter] [steps each way] [share each way]

use coreshape_document::follow::{as_clicked, counts, follow, without_refusals, Tally};
use coreshape_document::json::{self, Value};
use coreshape_document::{Document, FORMAT, FORMAT_VERSION};
use std::path::Path;

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
    let v = Value::obj(vec![
        ("format", Value::str(FORMAT)),
        ("version", Value::Num(FORMAT_VERSION)),
        ("units", Value::str("mm")),
        ("next_id", Value::Num(n as f64 + 1.0)),
        ("parameters", Value::Arr(params)),
        ("features", Value::Arr(features)),
    ]);
    Document::from_json(&v).unwrap_or_else(|e| panic!("{e}"))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let filter = args.get(1).cloned().unwrap_or_default();
    let steps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(6);
    let range: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0.1);
    let folder = Path::new(env!("CARGO_MANIFEST_DIR")).join("../mcp/tests/real_parts");
    let mut names: Vec<_> = std::fs::read_dir(&folder).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).collect();
    names.sort();
    let mut all = Tally::default();
    for path in names {
        let stem = path.file_stem().unwrap().to_string_lossy().to_string();
        if !stem.starts_with('p') || !stem.contains(&filter) || path.extension().is_none_or(|x| x != "json") {
            continue;
        }
        let script = json::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let mut doc = document_of(&script);
        let off = without_refusals(&mut doc);
        let (doc, rewritten) = match as_clicked(&doc) {
            Ok(x) => x,
            Err(e) => {
                println!("{stem}: can't rewrite: {e}");
                continue;
            }
        };
        let mut part = Tally::default();
        let params: Vec<(String, f64)> = match script.get("parameters") {
            Some(Value::Obj(items)) => items.iter().filter_map(|(k, v)| v.as_f64().map(|x| (k.clone(), x))).collect(),
            _ => Vec::new(),
        };
        let counting = counts(&doc);
        for (name, base) in &params {
            // A count moves every copy at once, which continuity can't follow: those have tests of their own.
            if counting.contains(name) {
                continue;
            }
            let step = base.abs() * range / steps as f64;
            let (down, up): (Vec<f64>, Vec<f64>) =
                ((1..=steps).map(|k| base - step * k as f64).collect(), (1..=steps).map(|k| base + step * k as f64).collect());
            let t = follow(&doc, name, &down, &up, &stem);
            part.add(&t);
        }
        println!("{stem}: {rewritten} references rewritten, {} turned off; {}", off.len(), part.summary());
        for line in &part.lines {
            println!("    {line}");
        }
        all.add(&part);
    }
    println!("all: {}", all.summary());
}
