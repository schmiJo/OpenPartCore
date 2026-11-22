//! A picture of splitting along sheets and stretching: a block split by a wavy spline sheet, its pieces pulled apart; a bar
//! split along a vase's turned spline face; a plate with a hole stretched across the hole, beside the plate as it was; and a
//! thin duct like the demo's, stretched. Each part in a tint of its own. Usage: split_stretch <out.png> [azimuth elevation]
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
    let bodies: Vec<Body> = o.bodies.iter().filter(|b| !b.body.sheet).map(|b| (*b.body).clone()).collect();
    for (b, n) in bodies.iter().zip(o.bodies.iter().filter(|b| !b.body.sheet)) {
        validate(b).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        println!("{what} {}: volume {:.3}", n.id.text(), mass_properties(b).volume);
    }
    bodies
}

fn p(x: f64, y: f64) -> Point2 {
    [x.into(), y.into()]
}

fn main() {
    let out = std::env::args().nth(1).expect("an output file");
    let mut parts: Vec<Vec<Body>> = Vec::new();

    // A block split by a wall through a spline, the piece above it lifted off.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 30.0, 24.0))]));
    doc.with(Op::extrude(s, 0.0, 16.0, Mode::Add));
    let wave = vec![Move::Line { to: p(0.0, 10.0) }, Move::Spline { through: vec![p(9.0, 17.0), p(20.0, 6.0), p(30.0, 13.0)] }, Move::Line { to: p(36.0, 12.0) }];
    let w = doc.with(Op::sketch(Placement::xy(-5.0), vec![Region::new(Shape::OpenPath { start: p(-6.0, 10.0), moves: wave })]));
    let wall = feature(&mut doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 26, "mode": "surface"}}"#, w.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}", "keep_tool": false}}"#, wall.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let lift = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "translate", "by": [0, 10, 8]}}"#);
    doc.set_scope(lift, vec![BodyId::parse(&format!("f{}", split.0)).unwrap()]).unwrap();
    parts.push(built(&mut doc, "wavy split"));

    // A vase turned from a spline, and a bar through it split along the vase's curved face, the vase taken away.
    let mut doc = Document::new();
    let outline = Shape::Path { start: p(0.0, 0.0), moves: vec![Move::Line { to: p(9.0, 0.0) }, Move::Spline { through: vec![p(12.0, 6.0), p(7.0, 14.0)] }, Move::Line { to: p(0.0, 14.0) }, Move::Line { to: p(0.0, 0.0) }] };
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(outline)]));
    let vase = doc.with(Op::revolve(s, 360.0, Mode::Add));
    let s = doc.with(Op::sketch(Placement::xy(3.0), vec![Region::new(Shape::rect(-3.0, -20.0, 6.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::NewBody));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.1"}}"#, vase.0));
    doc.set_scope(split, vec![BodyId::parse(&format!("f{}", vase.0 + 2)).unwrap()]).unwrap();
    let gone = feature(&mut doc, r#"{"type": "delete_bodies"}"#);
    doc.set_scope(gone, vec![BodyId::parse(&format!("f{}", vase.0)).unwrap()]).unwrap();
    parts.push(built(&mut doc, "bar split along a vase"));

    // A plate with a hole, and a copy stretched across the hole, twice as wide, beside it.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 16.0, 16.0)).hole(Shape::circle(8.0, 8.0, 4.0))]));
    let e = doc.with(Op::extrude(s, 0.0, 6.0, Mode::Add));
    feature(&mut doc, &format!(r#"{{"type": "fillet", "edges": {{"between": ["f{0}:end", "f{0}:hole.0.side.0"]}}, "radius": 1.5}}"#, e.0));
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "stretch", "axis": [1, 0, 0], "point": [24, 0, 0], "factor": 2}, "copy": true}"#);
    let mv = feature(&mut doc, r#"{"type": "transform", "motion": {"type": "translate", "by": [-12, -24, 0]}}"#);
    doc.set_scope(mv, vec![BodyId::parse("f4").unwrap()]).unwrap();
    parts.push(built(&mut doc, "plate stretched"));

    // A duct as the demo builds it: a thin loft from a rectangle up to a circle, a thin sweep round a bend, stretched across.
    let mut doc = Document::new();
    let base = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-40.0, -30.0, 80.0, 60.0))]));
    let top = doc.with(Op::sketch(Placement::xy(45.0), vec![Region::new(Shape::circle(0.0, 0.0, 12.0))]));
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 2}}}}"#, base.0, top.0));
    let ring = doc.with(Op::sketch(Placement::xy(45.0), vec![Region::new(Shape::circle(0.0, 0.0, 12.0))]));
    feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": [{{"arc_radius": 35, "degrees": 90, "toward_degrees": 0}}, {{"line": 12}}], "thin": {{"thickness": 2}}}}"#, ring.0));
    feature(&mut doc, r#"{"type": "transform", "motion": {"type": "stretch", "axis": [0, 1, 0], "point": [0, 0, 0], "factor": 1.5}}"#);
    parts.push(built(&mut doc, "duct"));

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
        x += b.max.x - b.min.x + 25.0;
    }
    let list: Vec<&Body> = placed.iter().collect();
    let all = Body::side_by_side(&list);
    let arg = |k: usize, default: f64| std::env::args().nth(k).and_then(|a| a.parse().ok()).unwrap_or(default);
    let options = Options { width: 1600, height: 900, view: View::Orbit { azimuth: arg(2, 30.0), elevation: arg(3, 30.0) }, colors: Colors::ByGroup, highlight_faces: Vec::new(), highlight_edges: Vec::new(), face_names: Vec::new(), face_groups: groups };
    std::fs::write(&out, render(&all, &options).unwrap().to_png()).unwrap();
    println!("wrote {out}");
}
