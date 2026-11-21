//! A picture of the small features at work: a bracket with gussets (rib across its sketch's plane), a thin-walled tray with
//! ribs down to its floor (thin extrude, rib along the normal), a shaft split along its stem and pulled apart, and a part
//! with a copy scaled up about its corner. Each part in a tint of its own. Usage: small_features <out.png> [azimuth elevation]
use coreshape::*;
use coreshape_document::*;
use coreshape_render::{render, Colors, Options, View};

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))).unwrap()
}

fn built(doc: &mut Document, what: &str) -> Vec<Body> {
    let o = doc.rebuild(None);
    if let Some(i) = o.failed {
        panic!("{what}: {:?}", o.steps[i]);
    }
    let bodies: Vec<Body> = o.bodies.iter().map(|b| (*b.body).clone()).collect();
    for (b, n) in bodies.iter().zip(&o.bodies) {
        validate(b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        println!("{what} {}: volume {:.3}", n.id.text(), mass_properties(b).volume);
    }
    bodies
}

fn front(y: f64) -> Placement {
    Placement::new(point3(0.0, y, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0))
}

fn p(x: f64, y: f64) -> Point2 {
    [x.into(), y.into()]
}

fn main() {
    let out = std::env::args().nth(1).expect("an output file");
    let mut parts: Vec<Vec<Body>> = Vec::new();

    // A bracket 30 deep, with a gusset 3 thick at each side.
    let mut doc = Document::new();
    let l = Shape::OpenPath { start: p(0.0, 40.0), moves: vec![Move::Line { to: p(0.0, 0.0) }, Move::Line { to: p(40.0, 0.0) }] };
    let s = doc.with(Op::sketch(front(0.0), vec![Region::new(l)]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 30, "opposite": true, "thin": {{"thickness": 4, "side": "outside"}}}}"#, s.0));
    for y in [5.0, 25.0] {
        let g = doc.with(Op::sketch(front(y), vec![Region::new(Shape::OpenPath { start: p(0.0, 28.0), moves: vec![Move::Line { to: p(28.0, 0.0) }] })]));
        feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 3, "flip": true}}"#, g.0));
    }
    parts.push(built(&mut doc, "bracket"));

    // A tray: walls 2 thick inside a rounded outline, a floor, and ribs crossing it, sketched on the rim and run down.
    let mut doc = Document::new();
    let outline = || Region::new(Shape::RoundedRect { cx: 0.0.into(), cy: 0.0.into(), w: 60.0.into(), h: 40.0.into(), r: 8.0.into() });
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![outline()]));
    feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 18, "thin": {{"thickness": 2}}}}"#, s.0));
    let floor = doc.with(Op::sketch(Placement::xy(0.0), vec![outline()]));
    doc.with(Op::extrude(floor, 0.0, 2.0, Mode::Add));
    let cross = vec![
        Region::new(Shape::OpenPath { start: p(-10.0, 0.0), moves: vec![Move::Line { to: p(10.0, 0.0) }] }),
        Region::new(Shape::OpenPath { start: p(-10.0, -5.0), moves: vec![Move::Line { to: p(-10.0, 5.0) }] }),
        Region::new(Shape::OpenPath { start: p(10.0, -5.0), moves: vec![Move::Line { to: p(10.0, 5.0) }] }),
    ];
    let r = doc.with(Op::sketch(Placement::xy(12.0), cross));
    feature(&mut doc, &format!(r#"{{"type": "rib", "sketch": {}, "thickness": 1.5, "direction": "normal", "flip": true}}"#, r.0));
    parts.push(built(&mut doc, "tray"));

    // A shaft, a collar under a stem, split along the stem's surface, and the ring pulled down off it.
    let mut doc = Document::new();
    let c = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 14.0))]));
    doc.with(Op::extrude(c, 0.0, 10.0, Mode::Add));
    let st = doc.with(Op::sketch(Placement::xy(10.0), vec![Region::new(Shape::circle(0.0, 0.0, 7.0))]));
    let stem = doc.with(Op::extrude(st, 0.0, 30.0, Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.0"}}"#, stem.0));
    let pull = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "translate", "by": [0, 0, -16]}}"#);
    doc.set_scope(pull, vec![BodyId::parse("f5").unwrap()]).unwrap();
    parts.push(built(&mut doc, "shaft"));

    // A knob, and a copy one and a half times its size, scaled about its corner.
    let mut doc = Document::new();
    let k = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 16.0, 16.0)).hole(Shape::circle(8.0, 8.0, 3.0))]));
    let e = doc.with(Op::extrude(k, 0.0, 8.0, Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "fillet", "edges": {{"between": ["f{0}:end", "f{0}:side.1"]}}, "radius": 3}}"#, e.0));
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "scale", "point": [0, -40, 0], "factor": 1.5}, "copy": true}"#);
    parts.push(built(&mut doc, "scaled"));

    // Side by side along x, each part its own tint.
    let mut placed: Vec<Body> = Vec::new();
    let mut groups = Vec::new();
    let mut x = 0.0;
    for list in &parts {
        let mut b = Aabb::EMPTY;
        for body in list {
            b.union(&body.aabb());
        }
        for body in list {
            let moved = body.transformed(&Frame::translation(v3(x - b.min.x, -b.center().y, 0.0)));
            groups.extend(std::iter::repeat_n(placed.len() as u32, moved.faces.len()));
            placed.push(moved);
        }
        x += b.max.x - b.min.x + 35.0;
    }
    let list: Vec<&Body> = placed.iter().collect();
    let all = Body::side_by_side(&list);
    let arg = |k: usize, default: f64| std::env::args().nth(k).and_then(|a| a.parse().ok()).unwrap_or(default);
    let options = Options { width: 1600, height: 900, view: View::Orbit { azimuth: arg(2, 30.0), elevation: arg(3, 30.0) }, colors: Colors::ByGroup, highlight_faces: Vec::new(), highlight_edges: Vec::new(), face_names: Vec::new(), face_groups: groups };
    std::fs::write(&out, render(&all, &options).unwrap().to_png()).unwrap();
    println!("wrote {out}");
}
