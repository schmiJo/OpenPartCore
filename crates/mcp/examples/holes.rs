//! A picture of the hole feature: the demo's hole plate five ways, M6 counterbored, M6 countersunk, M8 clearance, M8
//! tapped 15 deep and M5 tapped through, each with two M5 tapped holes in its front face; or with `cut`, three of them cut
//! through their front row of holes, so their profiles show. Prints each hole's callout. Usage: holes <out.png> [whole|cut] [azimuth elevation]
use coreshape::*;
use coreshape_document::*;
use coreshape_render::{render, Colors, Options, View};

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))).unwrap()
}

fn plate(variant: usize, cut: bool) -> Vec<Body> {
    let mut doc = Document::new();
    for (k, v) in [("width", 120.0), ("depth", 80.0), ("thick", 16.0), ("across", 70.0), ("along", 40.0)] {
        doc.set_parameter(k, v).unwrap();
    }
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::centered("width", "depth"))]));
    let e = doc.with(Op::extrude(s, 0.0, "thick", Mode::Add));
    let top = match variant {
        0 => r#""style": "counterbore", "size": "M6", "end": "through_all""#,
        1 => r#""style": "countersink", "size": "M6", "end": "through_all""#,
        2 => r#""size": "M8", "end": "through_all""#,
        3 => r#""size": "M8", "tapped": true, "depth": "thick - 5""#,
        _ => r#""size": "M5", "tapped": true, "end": "through_all""#,
    };
    feature(
        &mut doc,
        &format!(r#"{{"type": "hole", "plane": "f{}:end", "points": [["-across / 2", "-along / 2"], ["across / 2", "-along / 2"], ["across / 2", "along / 2"], ["-across / 2", "along / 2"]], {top}}}"#, e.0),
    );
    feature(&mut doc, &format!(r#"{{"type": "hole", "plane": "f{}:side.0", "points": [["-across / 4", "thick / 2"], ["across / 4", "thick / 2"]], "size": "M5", "tapped": {{"depth": 10}}, "depth": 13}}"#, e.0));
    if cut {
        let split = feature(&mut doc, r#"{"type": "split", "plane": {"point": [0, -20, 0], "normal": [0, 1, 0]}}"#);
        let gone = feature(&mut doc, r#"{"type": "delete_bodies"}"#);
        doc.set_scope(gone, vec![BodyId::parse(&format!("f{}", split.0)).unwrap()]).unwrap();
    }
    let o = doc.rebuild(None);
    if let Some(i) = o.failed {
        panic!("variant {variant}: {:?}", o.steps[i]);
    }
    if !cut {
        for h in &o.holes {
            println!("variant {variant}, f{} hole {}: {}", h.feature.0, h.number, h.text);
        }
    }
    o.bodies.iter().map(|b| (*b.body).clone()).inspect(|b| { validate(b).unwrap(); }).collect()
}

fn main() {
    let out = std::env::args().nth(1).expect("an output file");
    // Whole, or cut through the front row of holes.
    let cut = std::env::args().nth(2).as_deref() == Some("cut");
    let arg = |k: usize, default: f64| std::env::args().nth(k).and_then(|a| a.parse().ok()).unwrap_or(default);
    let mut placed: Vec<Body> = Vec::new();
    let mut groups = Vec::new();
    let variants: Vec<usize> = if cut { vec![0, 1, 3] } else { (0..5).collect() };
    for (k, variant) in variants.into_iter().enumerate() {
        let (x, y, z) = if cut { (0.0, 0.0, k as f64 * -40.0) } else { ((k % 3) as f64 * 135.0, (k / 3) as f64 * 95.0, 0.0) };
        for body in plate(variant, cut) {
            let moved = body.transformed(&Frame::translation(v3(x, y, z)));
            groups.extend(std::iter::repeat_n(variant as u32, moved.faces.len()));
            placed.push(moved);
        }
    }
    let list: Vec<&Body> = placed.iter().collect();
    let all = Body::side_by_side(&list);
    let (azimuth, elevation) = if cut { (0.0, 10.0) } else { (-60.0, 40.0) };
    let (width, height) = if cut { (1200, 1100) } else { (1600, 1000) };
    let options = Options { width, height, view: View::Orbit { azimuth: arg(3, azimuth), elevation: arg(4, elevation) }, colors: Colors::ByGroup, highlight_faces: Vec::new(), highlight_edges: Vec::new(), face_names: Vec::new(), face_groups: groups };
    std::fs::write(&out, render(&all, &options).unwrap().to_png()).unwrap();
    println!("wrote {out}");
}
