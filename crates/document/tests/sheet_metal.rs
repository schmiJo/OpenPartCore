//! Sheet metal: plates and chains of walls from sketches, flanges, and flat patterns, each checked against volumes and
//! lengths worked out by hand.

use coreshape::*;
use coreshape_document::*;
use std::f64::consts::PI;

fn add(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(read_op(&json::parse(text).unwrap_or_else(|e| panic!("{text}: {e:?}")), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))).unwrap()
}

fn built(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    o
}

fn failure(doc: &mut Document) -> String {
    let o = doc.rebuild(None);
    let at = o.failed.unwrap_or_else(|| panic!("expected a failure: {:?}", o.steps));
    match &o.steps[at].state {
        StepState::Failed(why) => why.clone(),
        other => panic!("{other:?}"),
    }
}

fn volume(o: &Outcome) -> f64 {
    let b = o.body.as_ref().expect("a body");
    validate(b).unwrap_or_else(|e| panic!("{e:?}"));
    mass_properties(b).volume
}

fn close(got: f64, want: f64, what: &str) {
    assert!((got - want).abs() <= 1e-7 * want.abs().max(1.0), "{what}: got {got}, expected {want}");
}

/// The flat's outline's extents in its own x and y, smaller first.
fn extents(flat: &FlatPattern) -> [f64; 2] {
    let (mut lo, mut hi) = (v2(f64::MAX, f64::MAX), v2(f64::MIN, f64::MIN));
    for e in &flat.outline {
        let points = match e {
            FlatEdge::Line { a, b } | FlatEdge::Arc { a, b, .. } => vec![*a, *b],
            FlatEdge::Points(p) => p.clone(),
        };
        for p in points {
            lo = v2(lo.x.min(p.x), lo.y.min(p.y));
            hi = v2(hi.x.max(p.x), hi.y.max(p.y));
        }
    }
    let (w, h) = (hi.x - lo.x, hi.y - lo.y);
    [w.min(h), w.max(h)]
}

fn flat_volume(flat: &FlatPattern) -> f64 {
    validate(&flat.body).unwrap_or_else(|e| panic!("{e:?}"));
    mass_properties(&flat.body).volume
}

const PLATE: &str = r#"{"type": "sketch", "regions": [{"outer": {"type": "rect", "x": 0, "y": 0, "w": "w", "h": 60}}]}"#;

fn plate(doc: &mut Document) {
    doc.set_parameter("w", 100.0).unwrap();
    add(doc, PLATE);
    add(doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2}"#);
}

#[test]
fn a_plate_from_a_region_is_its_own_flat() {
    let mut doc = Document::new();
    plate(&mut doc);
    let o = built(&mut doc);
    close(volume(&o), 100.0 * 60.0 * 2.0, "the plate");
    let flat = doc.flat_pattern(&o, None).unwrap();
    close(flat_volume(&flat), 12000.0, "laid flat");
    assert!(flat.bends.is_empty());
    assert_eq!(flat.outline.len(), 4);
    assert_eq!(extents(&flat), [60.0, 100.0]);
    assert_eq!((flat.thickness, flat.k_factor), (2.0, 0.45));
}

/// An L along lines (0, 0) to (50, 0) to (50, 40), 30 deep, 2 thick, bent at radius 3. With the sheet on the lines' left,
/// inside the corner, the lines are the bend's outside, so each wall's straight part stops r + t short of the corner;
/// flipped, the sheet is outside the corner and they stop r short.
#[test]
fn an_l_from_a_chain_of_lines_both_ways_round() {
    let (t, r, k, depth) = (2.0, 3.0, 0.45, 30.0);
    for flip in [false, true] {
        let mut doc = Document::new();
        add(&mut doc, r#"{"type": "sketch", "regions": [{"outer": {"type": "open_path", "start": [0, 0], "moves": [{"line_to": [50, 0]}, {"line_to": [50, 40]}]}}]}"#);
        add(&mut doc, &format!(r#"{{"type": "sheet_metal", "sketch": 1, "thickness": {t}, "bend_radius": {r}, "depth": {depth}, "flip": {flip}}}"#));
        let o = built(&mut doc);
        let back = if flip { r } else { r + t };
        let walls = (50.0 - back) + (40.0 - back);
        close(volume(&o), (walls * t + PI / 2.0 * t * (r + t / 2.0)) * depth, "the L");
        let flat = doc.flat_pattern(&o, None).unwrap();
        let length = walls + PI / 2.0 * (r + k * t);
        close(flat_volume(&flat), length * t * depth, "the L laid flat");
        let [a, b] = extents(&flat);
        close(a, depth, "the flat's depth");
        close(b, length, "the flat's length");
        let [bend] = flat.bends.as_slice() else { panic!("{:?}", flat.bends) };
        close(bend.degrees, 90.0, "the bend's angle");
        close(bend.radius, r, "the bend's inner radius");
        close(bend.width, PI / 2.0 * (r + k * t), "the bend's strip");
        close(bend.a.dist(bend.b), depth, "the bend line's length");
        // Seen from the side the flat is unfolded from, the lines' side, the other wall goes up when it's the outside.
        assert_eq!(bend.up, !flip);
    }
}

/// A flange 20 long at 90 degrees on the edge at y = 60 of a plate 100 by 60 by 2, with the part's radius, 2. Its bend
/// is a quarter of a ring 2 thick, 3 pi in section, and its wall 20 - (r + t) = 16 long. The wall is cut back by
/// (r + t) for `outer`, by r for `inner`, and not at all for `bend`, which moves the flange out by as much.
#[test]
fn a_flange_on_a_plate_each_way_it_can_sit() {
    let ring = PI / 2.0 * 2.0 * (2.0 + 1.0) * 100.0;
    let wall = 16.0 * 2.0 * 100.0;
    for (alignment, cut, outside) in [("outer", 4.0, 60.0), ("inner", 2.0, 62.0), ("bend", 0.0, 64.0)] {
        for flip in [false, true] {
            let mut doc = Document::new();
            plate(&mut doc);
            add(&mut doc, &format!(r#"{{"type": "flange", "edges": {{"near": [50, 60, 2]}}, "length": 20, "alignment": "{alignment}", "flip": {flip}}}"#));
            let o = built(&mut doc);
            let what = format!("{alignment}, flipped {flip}");
            close(volume(&o), 12000.0 - cut * 2.0 * 100.0 + ring + wall, &what);
            // Its far faces are flat, so its corners say how far it goes.
            let body = o.body.as_ref().unwrap();
            let most = |d: Vec3| body.vertices.iter().map(|p| p.dot(d)).fold(f64::MIN, f64::max);
            close(most(Vec3::Y), outside, &format!("{what}: its outside"));
            // 20 from the outer virtual sharp, on the plate's far face from the way it bends.
            let (low, high) = if flip { (-18.0, 2.0) } else { (0.0, 20.0) };
            close(-most(-Vec3::Z), low, &format!("{what}: its bottom"));
            close(most(Vec3::Z), high, &format!("{what}: its top"));
            let flat = doc.flat_pattern(&o, None).unwrap();
            let strip = PI / 2.0 * (2.0 + 0.45 * 2.0);
            let across = 60.0 - cut + strip + 16.0;
            close(flat_volume(&flat), across * 100.0 * 2.0, &format!("{what}: laid flat"));
            let [a, b] = extents(&flat);
            close(a, across.min(100.0), &format!("{what}: the flat's width"));
            close(b, across.max(100.0), &format!("{what}: the flat's length"));
            let [bend] = flat.bends.as_slice() else { panic!("{:?}", flat.bends) };
            // Laid flat from the plate's bottom face: bending up, toward its top, is up.
            assert_eq!(bend.up, !flip, "{what}");
            close(bend.a.dist(bend.b), 100.0, &format!("{what}: its bend line"));
        }
    }
}

/// Flanges on all four top edges of the plate, in one feature. Aligned at the bend they meet at the corners; aligned
/// outside, each stops (r + t) short of each corner, leaving a square of that size cut out of it.
#[test]
fn a_tray_from_four_flanges() {
    let section = 16.0 * 2.0 + PI / 2.0 * 2.0 * 3.0;
    let laid = 16.0 + PI / 2.0 * (2.0 + 0.45 * 2.0);
    let edges = r#"{"any": [{"near": [50, 0, 2]}, {"near": [50, 60, 2]}, {"near": [0, 30, 2]}, {"near": [100, 30, 2]}]}"#;
    for (alignment, cut) in [("bend", 0.0), ("outer", 4.0)] {
        let mut doc = Document::new();
        plate(&mut doc);
        add(&mut doc, &format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "alignment": "{alignment}"}}"#));
        let o = built(&mut doc);
        let (w, h) = (100.0 - 2.0 * cut, 60.0 - 2.0 * cut);
        close(volume(&o), w * h * 2.0 + 2.0 * (w + h) * section, alignment);
        let flat = doc.flat_pattern(&o, None).unwrap();
        close(flat_volume(&flat), (w * h + 2.0 * (w + h) * laid) * 2.0, &format!("{alignment}: laid flat"));
        assert_eq!(flat.bends.len(), 4);
        assert!(flat.bends.iter().all(|b| b.up));
        // A cross: the plate with a strip on each side.
        assert_eq!(flat.outline.len(), 12, "{alignment}: {:?}", flat.outline);
        let [a, b] = extents(&flat);
        close(a, h + 2.0 * laid, &format!("{alignment}: across"));
        close(b, w + 2.0 * laid, &format!("{alignment}: along"));
    }
}

/// The flange follows the part: its default radius is the part's, and it moves with the plate's width.
#[test]
fn a_flange_follows_the_part() {
    let mut doc = Document::new();
    doc.set_parameter("w", 100.0).unwrap();
    doc.set_parameter("br", 2.0).unwrap();
    add(&mut doc, PLATE);
    add(&mut doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2, "bend_radius": "br"}"#);
    add(&mut doc, r#"{"type": "flange", "edges": {"between": ["f2:end", "f2:side.2"]}, "length": 20}"#);
    let flanged = |w: f64, r: f64| w * 60.0 * 2.0 - (r + 2.0) * 2.0 * w + PI / 2.0 * 2.0 * (r + 1.0) * w + (20.0 - (r + 2.0)) * 2.0 * w;
    let o = built(&mut doc);
    close(volume(&o), flanged(100.0, 2.0), "at first");
    // The plate itself doesn't change with the bend radius, but the flange does.
    doc.set_parameter("br", 3.0).unwrap();
    let o = built(&mut doc);
    close(volume(&o), flanged(100.0, 3.0), "with a bigger radius");
    doc.set_parameter("w", 80.0).unwrap();
    let o = built(&mut doc);
    close(volume(&o), flanged(80.0, 3.0), "on a narrower plate");
    // Saved and read back, it's the same.
    let again = Document::load_text(&doc.save_text()).unwrap();
    assert_eq!(again.to_json(), doc.to_json());
}

#[test]
fn a_hole_through_a_wall_comes_out_in_the_flat() {
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#);
    // A hole through the flange's wall, radius 4, 10 up.
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 70, 0], "z": [0, -1, 0], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "circle", "cx": 50, "cy": 10, "r": 4}}]}"#);
    add(&mut doc, r#"{"type": "extrude", "sketch": 4, "length": 20, "mode": "remove"}"#);
    let o = built(&mut doc);
    let full = 14400.0 + 300.0 * PI;
    close(volume(&o), full - 16.0 * PI * 2.0, "with the hole");
    let flat = doc.flat_pattern(&o, None).unwrap();
    let across = 56.0 + PI / 2.0 * 2.9 + 16.0;
    close(flat_volume(&flat), across * 100.0 * 2.0 - 16.0 * PI * 2.0, "laid flat with the hole");
    let arcs: Vec<f64> = flat.outline.iter().filter_map(|e| if let FlatEdge::Arc { radius, .. } = e { Some(*radius) } else { None }).collect();
    assert!(!arcs.is_empty() && arcs.iter().all(|r| (r - 4.0).abs() < 1e-9), "{:?}", flat.outline);
}

#[test]
fn what_sheet_metal_refuses_says_why() {
    // A chain with an arc in it.
    let mut doc = Document::new();
    add(&mut doc, r#"{"type": "sketch", "regions": [{"outer": {"type": "open_path", "start": [0, 0], "moves": [{"line_to": [50, 0]}, {"arc_to": [60, 10], "center": [50, 10], "ccw": true}]}}]}"#);
    add(&mut doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2, "depth": 20}"#);
    assert!(failure(&mut doc).contains("lines only"));
    // A chain without a depth.
    let mut doc = Document::new();
    add(&mut doc, r#"{"type": "sketch", "regions": [{"outer": {"type": "open_path", "start": [0, 0], "moves": [{"line_to": [50, 0]}, {"line_to": [50, 40]}]}}]}"#);
    add(&mut doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2}"#);
    assert!(failure(&mut doc).contains("needs a depth"));
    // Lines too short for their bends.
    let mut doc = Document::new();
    add(&mut doc, r#"{"type": "sketch", "regions": [{"outer": {"type": "open_path", "start": [0, 0], "moves": [{"line_to": [3, 0]}, {"line_to": [3, 40]}]}}]}"#);
    add(&mut doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2, "depth": 20}"#);
    assert!(failure(&mut doc).contains("too short for its bends"));
    // A flange beside another's corner, from a second feature: the side face now runs on into the first flange's end.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [0, 28, 2]}, "length": 20}"#);
    assert!(failure(&mut doc).contains("flanges that meet at a corner go in one feature"));
    // A flange shorter than its bend.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 3}"#);
    assert!(failure(&mut doc).contains("ends inside its own bend"));
    // A flange on an edge that isn't along a wall: an upright corner of the plate.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [0, 0, 1]}, "length": 20}"#);
    assert!(failure(&mut doc).contains("isn't along the side of a wall"));
    // A flange at 180 degrees, which is a hem.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20, "degrees": 180}"#);
    assert!(failure(&mut doc).contains("less than 180"));
    // A flange on a part that isn't sheet metal.
    let mut doc = Document::new();
    add(&mut doc, PLATE.replace("\"w\": \"w\"", "\"w\": 100").as_str());
    add(&mut doc, r#"{"type": "extrude", "sketch": 1, "length": 2}"#);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#);
    assert!(failure(&mut doc).contains("isn't sheet metal"));
    // A flange on the top of another, bent back down across the plate, runs into it.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20, "alignment": "bend"}"#);
    built(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 62, 20]}, "length": 60, "degrees": 150}"#);
    let why = failure(&mut doc);
    assert!(why.contains("runs into the part"), "{why}");
}

#[test]
fn a_flat_pattern_refuses_what_it_cant_lay_flat() {
    // Nothing that's sheet metal.
    let mut doc = Document::new();
    add(&mut doc, PLATE.replace("\"w\": \"w\"", "\"w\": 100").as_str());
    add(&mut doc, r#"{"type": "extrude", "sketch": 1, "length": 2}"#);
    let o = built(&mut doc);
    assert!(doc.flat_pattern(&o, None).unwrap_err().to_string().contains("no sheet metal part"));
    // A hole across a bend.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#);
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 0, 10], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "circle", "cx": 50, "cy": 58, "r": 3}}]}"#);
    add(&mut doc, r#"{"type": "extrude", "sketch": 4, "length": -20, "mode": "remove"}"#);
    let o = built(&mut doc);
    let why = doc.flat_pattern(&o, None).unwrap_err().to_string();
    assert!(why.contains("cut across it"), "{why}");
    // A rounded wall edge isn't a bend.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "fillet", "edges": {"near": [50, 0, 0]}, "radius": 1}"#);
    let o = built(&mut doc);
    let why = doc.flat_pattern(&o, None).unwrap_err().to_string();
    assert!(why.contains("isn't a bend"), "{why}");
    // One on the other side, which the flat never reaches, comes out in its volume.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "fillet", "edges": {"near": [50, 0, 2]}, "radius": 1}"#);
    let o = built(&mut doc);
    let why = doc.flat_pattern(&o, None).unwrap_err().to_string();
    assert!(why.contains("laid flat this part comes to"), "{why}");
}
