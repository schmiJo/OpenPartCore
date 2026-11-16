//! Sheet metal, stage 2: hems, closed corners, tabs, bends along a line, and the defaults a part sets, each checked
//! against volumes and lengths worked out by hand, and each laid flat.
//!
//! Most start from a plate 100 by 60 by 2 on the XY plane, with the part's defaults: bend radius 2, k-factor 0.45, and a
//! minimal gap of a tenth of the thickness, 0.2.

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

/// How far the part reaches along `d`: at its vertices, or partway round an arc, which can bulge past its ends.
fn most(o: &Outcome, d: Vec3) -> f64 {
    let body = o.body.as_ref().unwrap();
    let mut best = body.vertices.iter().map(|p| p.dot(d)).fold(f64::MIN, f64::max);
    for edge in &body.edges {
        if let Curve::Circle { frame, radius } = &edge.curve {
            let at = d.dot(frame.y).atan2(d.dot(frame.x));
            let (lo, hi) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
            for k in -2..=2 {
                let t = at + k as f64 * 2.0 * PI;
                if t >= lo && t <= hi {
                    best = best.max(edge.curve.eval(t).dot(d));
                }
            }
            let _ = radius;
        }
    }
    best
}

fn outline_points(flat: &FlatPattern) -> Vec<Vec2> {
    flat.outline
        .iter()
        .flat_map(|e| match e {
            FlatEdge::Line { a, b } | FlatEdge::Arc { a, b, .. } => vec![*a, *b],
            FlatEdge::Points(p) => p.clone(),
        })
        .collect()
}

/// The flat's outline's extents in its own x and y, smaller first.
fn extents(flat: &FlatPattern) -> [f64; 2] {
    let points = outline_points(flat);
    let (lo, hi) = points.iter().fold((v2(f64::MAX, f64::MAX), v2(f64::MIN, f64::MIN)), |(lo, hi), p| {
        (v2(lo.x.min(p.x), lo.y.min(p.y)), v2(hi.x.max(p.x), hi.y.max(p.y)))
    });
    let (w, h) = (hi.x - lo.x, hi.y - lo.y);
    [w.min(h), w.max(h)]
}

fn flat_volume(flat: &FlatPattern) -> f64 {
    validate(&flat.body).unwrap_or_else(|e| panic!("{e:?}"));
    mass_properties(&flat.body).volume
}

const PLATE: &str = r#"{"type": "sketch", "regions": [{"outer": {"type": "rect", "x": 0, "y": 0, "w": 100, "h": 60}}]}"#;

fn plate(doc: &mut Document) {
    add(doc, PLATE);
    add(doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2}"#);
}

/// A fold's section: a bend of angle `a` at inner radius `r`, `t` thick, then a wall `wall` long.
fn section(a: f64, r: f64, t: f64, wall: f64) -> f64 {
    a * t * (r + t / 2.0) + wall * t
}

/// The same laid flat: the neutral line's length, then the wall.
fn laid(a: f64, r: f64, t: f64, wall: f64) -> f64 {
    (a * (r + 0.45 * t) + wall) * t
}

/// Hems on the plate's edge at y = 60, each kind, 100 long. A straight hem's fold has half the gap as its inner radius; with
/// `outer`, the fold's outside is where the edge was, so the wall is cut back by r + t.
#[test]
fn a_hem_of_each_kind() {
    // kind, its fields, inner radius, turn, straight part after the fold, how far the wall is cut back, how far out it
    // reaches past y = 60, and how high it comes.
    let cases = [
        ("straight", r#""length": 10"#, 0.1, PI, 10.0 - 2.1, 2.1, 0.0, 2.0 + 0.2 + 2.0),
        ("rounded", r#""length": 12, "bend_radius": 3, "alignment": "inner""#, 3.0, PI, 12.0 - 5.0, 3.0, 2.0, 2.0 + 6.0 + 2.0),
        ("teardrop", r#""length": 3, "alignment": "bend""#, 2.0, 1.25 * PI, 3.0, 0.0, 4.0, 2.0 + 2.0 + 4.0),
        ("rolled", "", 2.0, 1.5 * PI, 0.0, 4.0, 0.0, 2.0 + 2.0 + 4.0),
    ];
    for (kind, fields, r, a, wall, cut, out, top) in cases {
        let mut doc = Document::new();
        plate(&mut doc);
        let comma = if fields.is_empty() { "" } else { ", " };
        add(&mut doc, &format!(r#"{{"type": "hem", "edges": {{"near": [50, 60, 2]}}, "kind": "{kind}"{comma}{fields}}}"#));
        let o = built(&mut doc);
        close(volume(&o), 12000.0 - cut * 2.0 * 100.0 + section(a, r, 2.0, wall) * 100.0, kind);
        close(most(&o, Vec3::Y), 60.0 + out, &format!("{kind}: how far out"));
        close(most(&o, Vec3::Z), top, &format!("{kind}: how high"));
        close(-most(&o, -Vec3::Z), 0.0, &format!("{kind}: its bottom"));
        let flat = doc.flat_pattern(&o, None).unwrap();
        let across = 60.0 - cut;
        close(flat_volume(&flat), (across * 2.0 + laid(a, r, 2.0, wall)) * 100.0, &format!("{kind}: laid flat"));
        let [bend] = flat.bends.as_slice() else { panic!("{kind}: {:?}", flat.bends) };
        close(bend.degrees, a.to_degrees(), &format!("{kind}: the bend's angle"));
        close(bend.radius, r, &format!("{kind}: the bend's radius"));
        close(bend.a.dist(bend.b), 100.0, &format!("{kind}: the bend line"));
        assert!(bend.up, "{kind}");
        let length = across + laid(a, r, 2.0, wall) / 2.0;
        let [short, long] = extents(&flat);
        close(short, length.min(100.0), &format!("{kind}: the flat's width"));
        close(long, length.max(100.0), &format!("{kind}: the flat's length"));
    }
}

/// A straight hem flipped goes under the plate, the gap below it.
#[test]
fn a_hem_flipped_goes_under() {
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "straight", "length": 10, "gap": 1, "flip": true}"#);
    let o = built(&mut doc);
    close(volume(&o), 12000.0 - 2.5 * 200.0 + section(PI, 0.5, 2.0, 7.5) * 100.0, "flipped");
    close(-most(&o, -Vec3::Z), -3.0, "its bottom, the gap and a thickness under the plate");
    close(most(&o, Vec3::Z), 2.0, "its top");
    let flat = doc.flat_pattern(&o, None).unwrap();
    assert!(!flat.bends[0].up);
}

/// Hems on two edges that meet at a corner, in one feature, each stop short of the corner by as far as they reach back
/// over the wall, and the minimal gap more, so they don't touch: a rounded hem 10 long reaches 10 back.
#[test]
fn hems_meeting_at_a_corner_leave_it_open() {
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "hem", "edges": {"any": [{"near": [50, 60, 2]}, {"near": [100, 30, 2]}]}, "kind": "rounded", "length": 10}"#);
    let o = built(&mut doc);
    let trimmed = 12000.0 - 2.0 * (4.0 * 100.0 + 4.0 * 60.0 - 16.0);
    let runs = 89.8 + 49.8;
    close(volume(&o), trimmed + section(PI, 2.0, 2.0, 6.0) * runs, "two hems");
    let flat = doc.flat_pattern(&o, None).unwrap();
    close(flat_volume(&flat), trimmed + laid(PI, 2.0, 2.0, 6.0) * runs, "laid flat");
    assert_eq!(flat.bends.len(), 2);
}

/// A tray of four flanges at 120 degrees, leaning back over the plate. Each reaches 10 + sqrt 3 back from the edge, so
/// at the corners each stops that and the minimal gap short, and none touches another.
#[test]
fn flanges_leaning_back_meet_at_open_corners() {
    let (t, r, a) = (2.0, 2.0, 2.0 * PI / 3.0);
    let s3 = 3.0f64.sqrt();
    let (start, wall) = (-(r + t) * s3, 20.0 - (r + t) * s3);
    let stop = 10.0 + s3 + 0.2;
    let edges = r#"{"any": [{"near": [50, 0, 2]}, {"near": [50, 60, 2]}, {"near": [0, 30, 2]}, {"near": [100, 30, 2]}]}"#;
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, &format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "degrees": 120}}"#));
    let o = built(&mut doc);
    let plate_left = (100.0 + 2.0 * start) * (60.0 + 2.0 * start) * t;
    let runs = 2.0 * (100.0 - 2.0 * stop) + 2.0 * (60.0 - 2.0 * stop);
    close(volume(&o), plate_left + runs * section(a, r, t, wall), "four flanges at 120 degrees");
    assert_eq!(o.body.as_ref().unwrap().solids().len(), 1);
    let flat = doc.flat_pattern(&o, None).unwrap();
    close(flat_volume(&flat), plate_left + runs * laid(a, r, t, wall), "laid flat");
    assert!(flat.bends.iter().all(|b| (b.degrees - 120.0).abs() < 1e-9));
}

/// A tray of four flanges 20 long, with its corners closed. Outside alignment cuts the plate back by r + t = 4 and stops
/// each flange 4 short of each corner, as an open corner does; closing it, at each corner one flange's wall runs on 4 to
/// the corner, out to the other's outside, and the other's runs on to within the gap of the first one's inside, 4 - 2 -
/// gap. Each wall is 16 high above its bend, and 2 thick.
#[test]
fn a_tray_with_closed_corners() {
    let edges = r#"{"any": [{"near": [50, 0, 2]}, {"near": [50, 60, 2]}, {"near": [0, 30, 2]}, {"near": [100, 30, 2]}]}"#;
    let (w, h) = (92.0, 52.0);
    let open = w * h * 2.0 + 2.0 * (w + h) * section(PI / 2.0, 2.0, 2.0, 16.0);
    let open_flat = w * h * 2.0 + 2.0 * (w + h) * laid(PI / 2.0, 2.0, 2.0, 16.0);
    for (gap, field) in [(0.2, ""), (0.5, r#", "gap": 0.5"#)] {
        let mut doc = Document::new();
        plate(&mut doc);
        add(&mut doc, &format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "corner": "closed"{field}}}"#));
        let o = built(&mut doc);
        let more = 4.0 * (4.0 + (2.0 - gap)) * 16.0 * 2.0;
        close(volume(&o), open + more, &format!("gap {gap}"));
        // Its outside is the plate's outline, all the way up, at the corners too.
        close(most(&o, Vec3::X), 100.0, "its outside");
        let corner_top = o.body.as_ref().unwrap().vertices.iter().any(|p| (p.x - 100.0).abs() < 1e-9 && (p.y - 60.0).abs() < 1e-9 && (p.z - 20.0).abs() < 1e-9);
        assert!(corner_top, "gap {gap}: a wall reaches the outside corner at the top");
        // Going round anticlockwise, seen from above, each flange runs over the next one's end: at (100, 60), the one on
        // x = 100 runs on to y = 60, inside face and all, and the one on y = 60 stops the gap short of its inside face.
        let has = |p: Vec3| o.body.as_ref().unwrap().vertices.iter().any(|q| q.dist(p) < 1e-9);
        assert!(has(v3(98.0, 60.0, 20.0)), "gap {gap}: the flange on x = 100 runs over");
        assert!(has(v3(100.0 - 2.0 - gap, 58.0, 20.0)), "gap {gap}: the flange on y = 60 stops short");
        // Its pieces are named as the flanges' own faces, with the underside of each as side.5.
        let names = face_names(o.body.as_ref().unwrap());
        for region in ["", "region.1.", "region.2.", "region.3."] {
            assert_eq!(names.iter().filter(|n| n.starts_with(&format!("f3:{region}side.0"))).count(), 1, "{region}: {names:?}");
            assert!(names.iter().any(|n| n.starts_with(&format!("f3:{region}side.5"))), "{region}: {names:?}");
        }
        let flat = doc.flat_pattern(&o, None).unwrap();
        close(flat_volume(&flat), open_flat + more, &format!("gap {gap}: laid flat"));
        assert_eq!(flat.bends.len(), 4);
    }
}

/// The part's defaults: a closed corner and its gap for flanges, and the gap of a straight hem. A flange can still say
/// otherwise.
#[test]
fn a_part_sets_defaults_for_what_comes_after() {
    let edges = r#"{"any": [{"near": [50, 0, 2]}, {"near": [50, 60, 2]}, {"near": [0, 30, 2]}, {"near": [100, 30, 2]}]}"#;
    let (w, h) = (92.0, 52.0);
    let open = w * h * 2.0 + 2.0 * (w + h) * section(PI / 2.0, 2.0, 2.0, 16.0);
    let part = r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2, "minimal_gap": 0.5, "corner": "closed"}"#;
    let mut doc = Document::new();
    add(&mut doc, PLATE);
    add(&mut doc, part);
    let flange = add(&mut doc, &format!(r#"{{"type": "flange", "edges": {edges}, "length": 20}}"#));
    let o = built(&mut doc);
    close(volume(&o), open + 4.0 * (4.0 + 1.5) * 32.0, "closed by default, with the part's gap");
    let saved = doc.save_text();
    let again = Document::load_text(&saved).unwrap();
    assert_eq!(again.to_json(), doc.to_json());
    doc.edit(flange, read_op(&json::parse(&format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "corner": "open"}}"#)).unwrap(), &[]).unwrap()).unwrap();
    let o = built(&mut doc);
    close(volume(&o), open, "open when the flange says so");
    // A straight hem takes the part's gap: radius 0.25.
    let mut doc = Document::new();
    add(&mut doc, PLATE);
    add(&mut doc, part);
    add(&mut doc, r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "straight", "length": 10}"#);
    let o = built(&mut doc);
    close(volume(&o), 12000.0 - 2.25 * 200.0 + section(PI, 0.25, 2.0, 10.0 - 2.25) * 100.0, "the hem's gap");
}

/// Tabs drawn on the plate's plane, beside it and overlapping it, from either side, and one on a flange's wall.
#[test]
fn tabs_join_a_wall_in_its_plane() {
    // A tab 20 by 15 beside the edge at y = 60, on the plate's bottom plane, then overlapping the plate by 10 on its top.
    for (plane, y0) in [(r#"{"origin": [0, 0, 0], "z": [0, 0, 1], "x": [1, 0, 0]}"#, 60.0), (r#"{"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}"#, 50.0)] {
        let mut doc = Document::new();
        plate(&mut doc);
        add(&mut doc, &format!(r#"{{"type": "sketch", "plane": {plane}, "regions": [{{"outer": {{"type": "rect", "x": 40, "y": {y0}, "w": 20, "h": {}}}}}]}}"#, 75.0 - y0));
        add(&mut doc, r#"{"type": "tab", "sketch": 3}"#);
        let o = built(&mut doc);
        close(volume(&o), 12600.0, &format!("a tab from y = {y0}"));
        close(most(&o, Vec3::Y), 75.0, "it reaches out");
        close(most(&o, Vec3::Z), 2.0, "it's in the plate's plane");
        let flat = doc.flat_pattern(&o, None).unwrap();
        close(flat_volume(&flat), 12600.0, "laid flat");
        assert_eq!(extents(&flat), [75.0, 100.0]);
    }
    // On the outside of a flange's wall, at y = 60, standing on its end at z = 20: 20 by 10.
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#);
    let before = volume(&built(&mut doc));
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 60, 0], "z": [0, -1, 0], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "rect", "x": 40, "y": 20, "w": 20, "h": 10}}]}"#);
    add(&mut doc, r#"{"type": "tab", "sketch": 4}"#);
    let o = built(&mut doc);
    close(volume(&o), before + 400.0, "a tab on the flange");
    close(most(&o, Vec3::Z), 30.0, "it stands on the flange");
    let flat = doc.flat_pattern(&o, None).unwrap();
    let across = 56.0 + PI / 2.0 * 2.9 + 16.0;
    close(flat_volume(&flat), across * 200.0 + 400.0, "laid flat");
}

/// Bent along a line across the plate at x = 50, by 90 degrees at radius 2: the strip the bend's neutral line takes,
/// N = pi/2 (2 + 0.45 * 2), becomes the bend, and the part comes to 12000 - 2 * 60 N + 60 * 3 pi = 12000 + 6 pi. Laid
/// flat it's the plate again, with the bend line where the line was.
#[test]
fn a_bend_along_a_line_keeps_the_flat() {
    let n = PI / 2.0 * 2.9;
    for (fields, top, low, centre) in [
        ("", 4.0 + 50.0 - n / 2.0, 0.0, 0.0),
        (r#", "flip": true"#, 2.0, -(2.0 + 50.0 - n / 2.0), 0.0),
        (r#", "moves": "right""#, 4.0 + 50.0 - n / 2.0, 0.0, 0.0),
        (r#", "position": "start""#, 4.0 + 50.0 - n, 0.0, n / 2.0),
    ] {
        let mut doc = Document::new();
        plate(&mut doc);
        add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "open_path", "start": [50, -10], "moves": [{"line_to": [50, 70]}]}}]}"#);
        add(&mut doc, &format!(r#"{{"type": "bend", "sketch": 3{fields}}}"#));
        let o = built(&mut doc);
        let what = if fields.is_empty() { "as drawn" } else { fields };
        close(volume(&o), 12000.0 + 6.0 * PI, what);
        close(most(&o, Vec3::Z), top, &format!("{what}: its top"));
        close(-most(&o, -Vec3::Z), low, &format!("{what}: its bottom"));
        let flat = doc.flat_pattern(&o, None).unwrap();
        close(flat_volume(&flat), 12000.0, &format!("{what}: laid flat"));
        let [short, long] = extents(&flat);
        close(short, 60.0, &format!("{what}: the flat's width"));
        close(long, 100.0, &format!("{what}: the flat's length"));
        let [bend] = flat.bends.as_slice() else { panic!("{:?}", flat.bends) };
        close(bend.degrees, 90.0, what);
        close(bend.a.dist(bend.b), 60.0, &format!("{what}: the bend line"));
        // How far the plate's ends are from the bend line, either side.
        let across = v2(bend.b.y - bend.a.y, bend.a.x - bend.b.x).normalized();
        let at = outline_points(&flat).iter().map(|p| (*p - bend.a).dot(across)).fold((f64::MAX, f64::MIN), |(lo, hi), d| (lo.min(d), hi.max(d)));
        close(at.0.abs().min(at.1.abs()), 50.0 - centre, &format!("{what}: the nearer end"));
        close(at.0.abs().max(at.1.abs()), 50.0 + centre, &format!("{what}: the further end"));
    }
}

/// What's on the side that moves goes with it: a flange on the plate's edge at x = 0, then a bend along x = 50.
#[test]
fn a_bend_carries_what_is_on_the_side_that_moves() {
    let mut doc = Document::new();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [0, 30, 2]}, "length": 20}"#);
    let before = built(&mut doc);
    let (v0, f0) = (volume(&before), flat_volume(&doc.flat_pattern(&before, None).unwrap()));
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "open_path", "start": [50, -10], "moves": [{"line_to": [50, 70]}]}}]}"#);
    add(&mut doc, r#"{"type": "bend", "sketch": 4}"#);
    let o = built(&mut doc);
    close(volume(&o), v0 + 6.0 * PI, "bent, with its flange");
    let flat = doc.flat_pattern(&o, None).unwrap();
    close(flat_volume(&flat), f0, "the same flat");
    assert_eq!(flat.bends.len(), 2);
    // The flange, which stood up at x = 0, now lies level at the top: its outside, where the plate ended, is 50 - N/2 from
    // where the bend starts, above the bend's axis at z = 4.
    close(most(&o, Vec3::Z), 4.0 + 50.0 - PI / 2.0 * 2.9 / 2.0, "the flange's outside is the top");
}

/// Moving the line moves the bend, and moving a tab's sketch moves the tab, though the part before them stays the same.
#[test]
fn a_bend_and_a_tab_follow_their_sketches() {
    let n = PI / 2.0 * 2.9;
    let mut doc = Document::new();
    doc.set_parameter("bx", 50.0).unwrap();
    doc.set_parameter("ty", 60.0).unwrap();
    plate(&mut doc);
    add(&mut doc, r#"{"type": "sketch", "regions": [{"outer": {"type": "rect", "x": 40, "y": "ty", "w": 20, "h": 15}}]}"#);
    add(&mut doc, r#"{"type": "tab", "sketch": 3}"#);
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "open_path", "start": ["bx", -10], "moves": [{"line_to": ["bx", 90]}]}}]}"#);
    add(&mut doc, r#"{"type": "bend", "sketch": 5}"#);
    let o = built(&mut doc);
    // At first the line crosses the tab too, so the bend is 75 wide.
    close(volume(&o), 12600.0 + PI / 2.0 * 75.0 * 4.0 * 0.05, "bent across the plate and the tab");
    close(most(&o, Vec3::Z), 4.0 + 50.0 - n / 2.0, "bent at x = 50");
    close(most(&o, Vec3::Y), 75.0, "the tab out to y = 75");
    // The line alone, and then the tab alone.
    doc.set_parameter("bx", 30.0).unwrap();
    let o = built(&mut doc);
    close(most(&o, Vec3::Z), 4.0 + 30.0 - n / 2.0, "bent at x = 30");
    doc.set_parameter("ty", 55.0).unwrap();
    let o = built(&mut doc);
    close(most(&o, Vec3::Y), 70.0, "the tab out to y = 70");
    close(volume(&o), 12000.0 + 6.0 * PI + 20.0 * 10.0 * 2.0, "with the tab overlapping by 5");
}

#[test]
fn what_stage_2_refuses_says_why() {
    let line = |x0: f64, y0: f64, x1: f64, y1: f64, z: f64| {
        format!(
            r#"{{"type": "sketch", "plane": {{"origin": [0, 0, {z}], "z": [0, 0, 1], "x": [1, 0, 0]}}, "regions": [{{"outer": {{"type": "open_path", "start": [{x0}, {y0}], "moves": [{{"line_to": [{x1}, {y1}]}}]}}}}]}}"#
        )
    };
    let refused = |extra: &[&str], want: &str| {
        let mut doc = Document::new();
        plate(&mut doc);
        for e in extra {
            add(&mut doc, e);
        }
        let why = failure(&mut doc);
        assert!(why.contains(want), "{want}: {why}");
    };
    // A teardrop turned back onto its wall.
    refused(&[r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "teardrop", "length": 6, "alignment": "bend"}"#], "comes back down onto its own wall");
    // A straight hem shorter than its fold.
    refused(&[r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "straight", "length": 2}"#], "ends inside its own fold");
    // A roll of a full turn.
    refused(&[r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "rolled", "degrees": 360}"#], "less than 360");
    // Closing the corners of flanges that aren't at 90 degrees, and with a gap as wide as the bend radius.
    let edges = r#"{"any": [{"near": [50, 60, 2]}, {"near": [100, 30, 2]}]}"#;
    refused(&[&format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "degrees": 80, "corner": "closed"}}"#)], "flanges of 90 degrees");
    refused(&[&format!(r#"{{"type": "flange", "edges": {edges}, "length": 20, "corner": "closed", "gap": 2}}"#)], "less than the bend radius");
    // A tab that doesn't reach the plate, and one in a plane with no wall.
    let tab = |plane: &str, y: f64| format!(r#"{{"type": "sketch", "plane": {plane}, "regions": [{{"outer": {{"type": "rect", "x": 40, "y": {y}, "w": 20, "h": 10}}}}]}}"#);
    let on = r#"{"origin": [0, 0, 0], "z": [0, 0, 1], "x": [1, 0, 0]}"#;
    refused(&[&tab(on, 70.0), r#"{"type": "tab", "sketch": 3}"#], "doesn't reach a sheet metal wall");
    refused(&[&tab(r#"{"origin": [0, 0, 1], "z": [0, 0, 1], "x": [1, 0, 0]}"#, 50.0), r#"{"type": "tab", "sketch": 3}"#], "doesn't reach a sheet metal wall");
    // A tab over a flange's bend.
    refused(
        &[r#"{"type": "flange", "edges": {"near": [50, 60, 2]}, "length": 20}"#, &tab(r#"{"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}"#, 50.0), r#"{"type": "tab", "sketch": 4}"#],
        "isn't flat wall square through the sheet",
    );
    // A bend along a line that stops halfway across, one on no wall's plane, one across a hole, and one that folds the
    // plate back onto itself.
    refused(&[&line(50.0, 10.0, 50.0, 40.0, 2.0), r#"{"type": "bend", "sketch": 3}"#], "doesn't cut the part in two");
    refused(&[&line(50.0, -10.0, 50.0, 70.0, 5.0), r#"{"type": "bend", "sketch": 3}"#], "isn't on the plane of a sheet metal wall");
    refused(
        &[
            r#"{"type": "sketch", "plane": {"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "circle", "cx": 50, "cy": 30, "r": 3}}]}"#,
            r#"{"type": "extrude", "sketch": 3, "length": -2, "mode": "remove"}"#,
            &line(50.0, -10.0, 50.0, 70.0, 2.0),
            r#"{"type": "bend", "sketch": 5}"#,
        ],
        "plain flat wall right across",
    );
    // Folded back by 170 degrees, the plate's right side rises slowly over its left, through a flange standing at x = 0.
    refused(
        &[r#"{"type": "flange", "edges": {"near": [0, 30, 2]}, "length": 20}"#, &line(40.0, -10.0, 40.0, 70.0, 2.0), r#"{"type": "bend", "sketch": 4, "degrees": 170, "moves": "right"}"#],
        "runs into the rest of the part",
    );
    // A sketch with more than a line.
    refused(&[PLATE, r#"{"type": "bend", "sketch": 3}"#], "the one line in its sketch");
    // Fields a hem of that kind doesn't take, and one it needs.
    let read = |text: &str| read_op(&json::parse(text).unwrap(), &[]).map(|_| ()).unwrap_err().to_string();
    assert!(read(r#"{"type": "hem", "edges": {"near": [0, 0, 0]}, "kind": "rounded", "length": 5, "gap": 1}"#).contains("has no \"gap\""));
    assert!(read(r#"{"type": "hem", "edges": {"near": [0, 0, 0]}, "kind": "rolled", "length": 5}"#).contains("has no \"length\""));
    assert!(read(r#"{"type": "hem", "edges": {"near": [0, 0, 0]}, "kind": "straight"}"#).contains("needs a \"length\""));
    assert!(read(r#"{"type": "hem", "edges": {"near": [0, 0, 0]}, "kind": "folded", "length": 5}"#).contains("\"kind\" is"));
    assert!(read(r#"{"type": "bend", "sketch": 1, "moves": "up"}"#).contains("\"left\" or \"right\""));
}

/// Everything here saves and reads back as it was.
#[test]
fn stage_2_features_save_and_read_back() {
    let mut doc = Document::new();
    add(&mut doc, PLATE);
    add(&mut doc, r#"{"type": "sheet_metal", "sketch": 1, "thickness": 2, "minimal_gap": 0.3, "corner": "closed"}"#);
    add(&mut doc, r#"{"type": "flange", "edges": {"near": [50, 0, 2]}, "length": 20, "corner": "open", "gap": 0.4}"#);
    add(&mut doc, r#"{"type": "hem", "edges": {"near": [50, 60, 2]}, "kind": "teardrop", "length": 3, "bend_radius": 2, "degrees": 220, "alignment": "bend", "flip": true}"#);
    add(&mut doc, r#"{"type": "sketch", "plane": {"origin": [0, 0, 2], "z": [0, 0, 1], "x": [1, 0, 0]}, "regions": [{"outer": {"type": "open_path", "start": [50, -10], "moves": [{"line_to": [50, 70]}]}}]}"#);
    add(&mut doc, r#"{"type": "bend", "sketch": 5, "degrees": 45, "bend_radius": 3, "position": "start", "flip": true, "moves": "right"}"#);
    add(&mut doc, r#"{"type": "tab", "sketch": 5}"#);
    let again = Document::load_text(&doc.save_text()).unwrap();
    assert_eq!(again.to_json(), doc.to_json());
    let text = doc.save_text();
    for field in ["\"minimal_gap\"", "\"corner\": \"closed\"", "\"kind\": \"teardrop\"", "\"moves\": \"right\"", "\"position\": \"start\"", "\"type\": \"tab\""] {
        assert!(text.contains(field), "{field} in {text}");
    }
}
