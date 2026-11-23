//! Times a part of about 200 features, as a person would model a big machined plate: a base, then 28 cells, each a
//! pocket, a boss with a tapped hole, a round on the boss and a chamfer round the pocket, all on the base's top, and
//! rounds on the base's corners last. Prints the time to add each feature as it's modelled, the slowest of them, a full
//! rebuild after the base's thickness changes, a rebuild after the last feature changes, and a display mesh.
//!
//! cargo run --release -p coreshape-mcp --example big_part -- [cells, default 28] [--each]

use coreshape::*;
use coreshape_document::*;
use std::time::Instant;

fn add(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))).unwrap()
}

fn main() {
    let cells: usize = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(28);
    let each = std::env::args().any(|a| a == "--each");
    let cols = 7usize;
    let rows = cells.div_ceil(cols);
    let (w, d) = (cols as f64 * 40.0, rows as f64 * 40.0);
    let mut doc = Document::new();
    for (k, v) in [("t", 20.0), ("pd", 6.0), ("bh", 8.0), ("r", 1.0)] {
        doc.set_parameter(k, v).unwrap();
    }
    let mut times: Vec<(String, f64)> = Vec::new();
    let mut timed = |doc: &mut Document, label: String, text: String| -> FeatureId {
        let t = Instant::now();
        let id = add(doc, &text);
        let o = doc.rebuild(None);
        if let Some(i) = o.failed {
            panic!("{label}: {:?}", o.steps[i]);
        }
        let s = t.elapsed().as_secs_f64();
        if each {
            println!("{:>4} {label:<28} {s:.3} s", id.0);
        }
        times.push((label, s));
        id
    };
    let start = Instant::now();
    let base_sketch = timed(&mut doc, "base sketch".into(), format!(r#"{{"type": "sketch", "regions": [{{"outer": {{"type": "rect", "x": 0, "y": 0, "w": {w}, "h": {d}}}}}]}}"#));
    let base = timed(&mut doc, "base".into(), format!(r#"{{"type": "extrude", "sketch": {}, "length": "t"}}"#, base_sketch.0));
    for c in 0..cells {
        let (x, y) = ((c % cols) as f64 * 40.0 + 20.0, (c / cols) as f64 * 40.0 + 20.0);
        let ps = timed(&mut doc, format!("cell {c} pocket sketch"), format!(r#"{{"type": "sketch", "plane": {{"origin": [0, 0, "t"], "z": [0, 0, 1], "x": [1, 0, 0]}}, "regions": [{{"outer": {{"type": "rect", "x": {}, "y": {}, "w": 14, "h": 14}}}}]}}"#, x - 16.0, y - 16.0));
        let p = timed(&mut doc, format!("cell {c} pocket"), format!(r#"{{"type": "extrude", "sketch": {}, "start": "-pd", "length": "pd", "mode": "remove"}}"#, ps.0));
        let bs = timed(&mut doc, format!("cell {c} boss sketch"), format!(r#"{{"type": "sketch", "plane": {{"origin": [0, 0, "t"], "z": [0, 0, 1], "x": [1, 0, 0]}}, "regions": [{{"outer": {{"type": "circle", "cx": {}, "cy": {}, "r": 6}}}}]}}"#, x + 8.0, y + 8.0));
        let b = timed(&mut doc, format!("cell {c} boss"), format!(r#"{{"type": "extrude", "sketch": {}, "length": "bh"}}"#, bs.0));
        timed(&mut doc, format!("cell {c} hole"), format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [[{}, {}]], "size": "M5", "tapped": true, "depth": 10}}"#, b.0, x + 8.0, y + 8.0));
        timed(&mut doc, format!("cell {c} boss round"), format!(r#"{{"type": "fillet", "edges": {{"between": ["f{}:end", "f{}:side.0"]}}, "radius": "r"}}"#, b.0, b.0));
        let sides: Vec<String> = (0..4).map(|k| format!(r#"{{"between": ["f{}:end", "f{}:side.{k}"]}}"#, base.0, p.0)).collect();
        timed(&mut doc, format!("cell {c} pocket chamfer"), format!(r#"{{"type": "chamfer", "edges": {{"any": [{}]}}, "distance": 0.5}}"#, sides.join(", ")));
    }
    timed(&mut doc, "corner rounds".into(), format!(r#"{{"type": "fillet", "edges": {{"any": [{}]}}, "radius": 5}}"#, (0..4).map(|k| format!(r#"{{"between": ["f{}:side.{k}", "f{}:side.{}"]}}"#, base.0, base.0, (k + 1) % 4)).collect::<Vec<_>>().join(", ")));
    let modelled = start.elapsed().as_secs_f64();
    let o = doc.rebuild(None);
    let faces: usize = o.bodies.iter().map(|b| b.body.faces.len()).sum();
    println!("{} features, {} faces, modelled in {modelled:.2} s", doc.features().len(), faces);
    let mut sorted = times.clone();
    sorted.sort_by(|a, b| b.1.total_cmp(&a.1));
    println!("slowest to add:");
    for (label, s) in sorted.iter().take(8) {
        println!("  {label:<28} {s:.3} s");
    }
    let sum = |what: &str| times.iter().filter(|(l, _)| l.ends_with(what)).map(|(_, s)| s).sum::<f64>();
    for what in ["sketch", "pocket", "boss", "hole", "round", "chamfer"] {
        println!("  all '{what}': {:.2} s", sum(what));
    }
    let first_half: f64 = times[..times.len() / 2].iter().map(|(_, s)| s).sum();
    let second_half: f64 = times[times.len() / 2..].iter().map(|(_, s)| s).sum();
    println!("first half of the features {first_half:.2} s, second half {second_half:.2} s");

    // The base's thickness: everything after it builds again.
    let t = Instant::now();
    doc.set_parameter("t", 22.0).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps.iter().find(|s| matches!(s.state, StepState::Failed(_))));
    println!("full rebuild after the base's thickness changes: {:.2} s", t.elapsed().as_secs_f64());
    // The rounds' radius: from the first boss's round on.
    let t = Instant::now();
    doc.set_parameter("r", 1.5).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None);
    println!("rebuild after the rounds' radius changes: {:.2} s", t.elapsed().as_secs_f64());
    // Nothing changed.
    let t = Instant::now();
    doc.rebuild(None);
    println!("rebuild with nothing changed: {:.3} s", t.elapsed().as_secs_f64());
    // The display mesh.
    let t = Instant::now();
    let mut triangles = 0;
    for b in &o.bodies {
        triangles += tessellate(&b.body, &TessParams::display_for_size(w.max(d))).unwrap().triangles.len();
    }
    println!("display mesh: {triangles} triangles in {:.2} s", t.elapsed().as_secs_f64());
    let t = Instant::now();
    for b in &o.bodies {
        validate(&b.body).unwrap();
    }
    println!("validate: {:.2} s", t.elapsed().as_secs_f64());
    let t = Instant::now();
    let v: f64 = o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum();
    println!("volume {v:.3} in {:.2} s", t.elapsed().as_secs_f64());
}
