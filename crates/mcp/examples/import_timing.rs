//! Times bringing a STEP file into a document the way an app does: through the tools, then a display mesh, as
//! SchmiShape's engine makes one. Prints each call's time, what the feature said, and the progress it reported.
//!
//! cargo run --release -p coreshape-mcp --example import_timing -- <file.step> [mode]
//!
//! The mode is new_body if left out.

use coreshape::{tessellate, TessParams};
use coreshape_document::json::Value;
use coreshape_mcp::Session;
use std::path::PathBuf;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("a STEP file");
    let mode = args.get(2).cloned().unwrap_or_else(|| "new_body".to_string());
    let text = String::from_utf8_lossy(&std::fs::read(path).unwrap()).into_owned();
    let mut session = Session::new(PathBuf::from("/"));
    let start = Instant::now();
    let mut heard = 0usize;
    coreshape::progress::set_hook(move |p| {
        heard += 1;
        if p.done == p.total || heard % 25 == 0 {
            eprintln!("  progress {} at {:.1} s", p.json(), start.elapsed().as_secs_f64());
        }
    });
    let time = |session: &mut Session, what: &str, name: &str, arguments: Value| -> Value {
        let t = Instant::now();
        let answer = session.call_safely(name, &arguments);
        let words = answer.get("content").and_then(|c| c.as_array()).and_then(|c| c.first()).and_then(|c| c.get("text")).and_then(Value::as_str).unwrap_or("").to_string();
        println!("{what}: {:.1} s, {} bytes back{}", t.elapsed().as_secs_f64(), words.len(), if answer.get("isError") == Some(&Value::Bool(true)) { ", refused" } else { "" });
        answer
    };
    let feature = Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(&text)), ("mode", Value::str(&mode))]);
    let added = time(&mut session, "feature_add", "feature_add", Value::obj(vec![("feature", feature)]));
    let words = added.get("content").and_then(|c| c.as_array()).and_then(|c| c.first()).and_then(|c| c.get("text")).and_then(Value::as_str).unwrap_or("");
    for line in words.lines().filter(|l| l.contains("left out") || l.contains("repaired") || l.contains("\"failed\"") || l.contains("error")) {
        println!("  {}", line.trim());
    }
    if std::env::var("ONLY_ADD").is_ok() {
        return;
    }
    time(&mut session, "bodies_list", "bodies_list", Value::obj(vec![]));
    time(&mut session, "document_show, whole", "document_show", Value::obj(vec![]));
    time(&mut session, "document_show, big texts left out", "document_show", Value::obj(vec![("omit_texts_over", Value::Num(100_000.0))]));
    let t = Instant::now();
    let outcome = session.outcome_upto(None);
    println!("rebuild with nothing changed: {:.1} s", t.elapsed().as_secs_f64());
    if let Some(body) = outcome.body {
        let t = Instant::now();
        let size = body.aabb().diagonal();
        let params = TessParams { chord: (size * 5e-4).max(1e-4), angle: 11f64.to_radians() };
        match tessellate(&body, &params) {
            Ok(mesh) => println!("display mesh: {:.1} s, {} triangles, {} bodies", t.elapsed().as_secs_f64(), mesh.triangles.len(), outcome.bodies.len()),
            Err(e) => println!("display mesh refused after {:.1} s: {e}", t.elapsed().as_secs_f64()),
        }
    }
    time(&mut session, "parameter_set, then rebuild", "parameter_set", Value::obj(vec![("name", Value::str("x")), ("value", Value::Num(1.0))]));
    coreshape::progress::clear();
}
