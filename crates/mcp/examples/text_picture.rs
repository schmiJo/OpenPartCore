//! A picture of text in sketches: name plates with text cut in and raised, as the demo makes them, one above another.
//! Usage: text_picture <out.png> [azimuth elevation]
use coreshape::*;
use coreshape_document::*;
use coreshape_render::{render, Colors, Options, View};
use coreshape_sketch::{Part, Ref, Rule, Sketch, ORIGIN};

fn plate(words: &str, raised: bool, height: f64) -> Vec<Body> {
    let mut doc = Document::new();
    for (k, v) in [("height", height), ("depth", 1.0), ("thick", 4.0), ("margin", 6.0)] {
        doc.set_parameter(k, v).unwrap();
    }
    let font = coreshape_sketch::font::Font::builtin();
    let k = font.advance(words) / font.cap_height;
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rounded(0.0, 0.0, format!("{k} * height + 2 * margin").as_str(), "2.4 * height + 2 * margin", "margin"))]));
    doc.with(Op::extrude(s, 0.0, "thick", Mode::Add));
    let mut sketch = Sketch::new();
    let t = sketch.text(words, [0.0, 0.0], 1.0);
    let (at, up) = (sketch.text_of(t).unwrap().at, sketch.text_of(t).unwrap().up);
    let whole = |entity| Ref { entity, part: Part::Whole };
    sketch.constrain(Rule::HorizontalDistance { a: whole(ORIGIN), b: whole(at), value: format!("-{k} * height / 2").as_str().into() });
    sketch.constrain(Rule::VerticalDistance { a: whole(ORIGIN), b: whole(at), value: "-height / 2".into() });
    sketch.constrain(Rule::Vertical { a: whole(at), b: Some(whole(up)) });
    sketch.constrain(Rule::VerticalDistance { a: whole(at), b: whole(up), value: "height".into() });
    let w = doc.with(Op::ConstrainedSketch { plane: Placement::xy("thick"), sketch });
    if raised {
        doc.with(Op::extrude(w, 0.0, "depth", Mode::Add));
    } else {
        doc.with(Op::extrude(w, "-depth", "depth", Mode::Remove));
    }
    let o = doc.rebuild(None);
    if let Some(i) = o.failed {
        panic!("{words}: {:?}", o.steps[i]);
    }
    o.bodies.iter().map(|b| (*b.body).clone()).inspect(|b| {
        validate(b).unwrap();
    }).collect()
}

fn main() {
    let out = std::env::args().nth(1).expect("an output file");
    let arg = |k: usize, default: f64| std::env::args().nth(k).and_then(|a| a.parse().ok()).unwrap_or(default);
    let plates = [("CoreShape", false, 10.0), ("CoreShape", true, 10.0), ("\u{D8} 12 \u{B1}0.05", false, 10.0), ("\u{C5}ngstr\u{F6}m \u{C7}a va", true, 8.0)];
    let mut placed: Vec<Body> = Vec::new();
    let mut groups = Vec::new();
    for (k, (words, raised, height)) in plates.iter().enumerate() {
        for body in plate(words, *raised, *height) {
            let moved = body.transformed(&Frame::translation(v3(0.0, k as f64 * -45.0, 0.0)));
            groups.extend(std::iter::repeat_n(k as u32, moved.faces.len()));
            placed.push(moved);
        }
    }
    let list: Vec<&Body> = placed.iter().collect();
    let all = Body::side_by_side(&list);
    let options = Options { width: 1200, height: 1400, view: View::Orbit { azimuth: arg(2, -20.0), elevation: arg(3, 55.0) }, colors: Colors::ByGroup, highlight_faces: Vec::new(), highlight_edges: Vec::new(), face_names: Vec::new(), face_groups: groups };
    std::fs::write(&out, render(&all, &options).unwrap().to_png()).unwrap();
    println!("wrote {out}");
}
