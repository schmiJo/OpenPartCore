//! A picture of thin revolves, sweeps and lofts: a tapered cup turned three quarters of the way round so its wall shows, a
//! bent tube and a bent gutter swept from a circle and an open U, and a duct lofted from a rectangle to a circle. Each part in a
//! tint of its own. Usage: thin_features <out.png> [azimuth elevation]
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

fn p(x: f64, y: f64) -> Point2 {
    [x.into(), y.into()]
}

fn open(points: &[(f64, f64)]) -> Shape {
    Shape::OpenPath { start: p(points[0].0, points[0].1), moves: points[1..].iter().map(|q| Move::Line { to: p(q.0, q.1) }).collect() }
}

/// A sketch on the plane through the origin square to x, its own x along world y and its y along world z.
fn across_x() -> Placement {
    Placement::new(point3(0.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0))
}

fn main() {
    let out = std::env::args().nth(1).expect("an output file");
    let mut parts: Vec<Vec<Body>> = Vec::new();

    // A cup: a floor out from the axis, then a side leaning out, a wall 2.5 thick about them, turned 270 degrees.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(open(&[(0.0, 0.0), (16.0, 0.0), (22.0, 34.0)]))]));
    feature(&mut doc, &format!(r#"{{"type": "revolve", "sketch": {}, "degrees": 270, "thin": {{"thickness": 2.5, "side": "both"}}}}"#, s.0));
    parts.push(built(&mut doc, "cup"));

    // A tube: a circle of radius 6 with a wall 1.2 thick inside it, 25 along, a quarter bend of radius 18, then 15 along.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(across_x(), vec![Region::new(Shape::circle(0.0, 6.0, 6.0))]));
    feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": [{{"line": 25}}, {{"arc_radius": 18, "degrees": 90, "toward_degrees": 0}}, {{"line": 15}}], "thin": {{"thickness": 1.2}}}}"#, s.0));
    parts.push(built(&mut doc, "tube"));

    // A gutter: an open U, 14 wide and 8 deep, with a wall 1.5 thick outside it, along a path sketch: 30 along x, then round.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(across_x(), vec![Region::new(open(&[(-7.0, 8.0), (-7.0, 0.0), (7.0, 0.0), (7.0, 8.0)]))]));
    let path = doc.with(Op::sketch(
        Placement::xy(0.0),
        vec![Region::new(Shape::OpenPath { start: p(0.0, 0.0), moves: vec![Move::Line { to: p(30.0, 0.0) }, Move::Arc { to: p(55.0, 25.0), center: p(30.0, 25.0), ccw: true }] })],
    ));
    feature(&mut doc, &format!(r#"{{"type": "sweep", "sketch": {}, "path": {{"sketch": {}}}, "thin": {{"thickness": 1.5, "side": "outside"}}}}"#, s.0, path.0));
    parts.push(built(&mut doc, "gutter"));

    // A duct from a rectangle 40 by 24 to a circle of radius 10, 35 up, its wall 1.5 thick inside each.
    let mut doc = Document::new();
    let low = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(-20.0, -12.0, 40.0, 24.0))]));
    let high = doc.with(Op::sketch(Placement::xy(35.0), vec![Region::new(Shape::circle(0.0, 0.0, 10.0))]));
    feature(&mut doc, &format!(r#"{{"type": "loft", "sections": [{}, {}], "thin": {{"thickness": 1.5}}}}"#, low.0, high.0));
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
