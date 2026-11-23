//! Splitting parts along sheets: a sheet part, and the surfaces of extruded and turned curves and of free-form faces. Each is
//! held against the same cut made another way, by a solid Boolean with the solid on one side of the surface, or by hand.

use coreshape::*;
use coreshape_document::*;

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    for b in &o.bodies {
        validate(&b.body).unwrap_or_else(|e| panic!("{e:?}"));
    }
    o
}

fn failure(doc: &mut Document) -> String {
    let o = doc.rebuild(None);
    let i = o.failed.unwrap_or_else(|| panic!("it should have failed: {:?}", o.steps));
    match &o.steps[i].state {
        StepState::Failed(why) => why.clone(),
        other => panic!("{other:?}"),
    }
}

fn op(text: &str) -> Op {
    read_op(&json::parse(text).unwrap(), &[]).unwrap_or_else(|e| panic!("{text}: {e}"))
}

fn feature(doc: &mut Document, text: &str) -> FeatureId {
    doc.add(op(text)).unwrap()
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-7 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

/// Each body's name and volume, or area for a sheet.
fn sizes(o: &Outcome) -> Vec<(String, f64)> {
    o.bodies
        .iter()
        .map(|b| {
            let m = mass_properties(&b.body);
            (b.id.text(), if b.body.sheet { m.area } else { m.volume })
        })
        .collect()
}

fn size_of(o: &Outcome, name: &str) -> f64 {
    sizes(o).into_iter().find(|(n, _)| n == name).unwrap_or_else(|| panic!("no body {name}: {:?}", sizes(o))).1
}

fn p(x: f64, y: f64) -> Point2 {
    [x.into(), y.into()]
}

/// A block 20 on each side from the origin.
fn block(doc: &mut Document) {
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 20.0, 20.0))]));
    doc.with(Op::extrude(s, 0.0, 20.0, Mode::Add));
}

/// The wavy curve the tests cut along: across x from -5 to 25, through a spline.
fn wave() -> Vec<Move> {
    vec![Move::Line { to: p(0.0, 8.0) }, Move::Spline { through: vec![p(6.0, 14.0), p(13.0, 5.0), p(20.0, 11.0)] }, Move::Line { to: p(25.0, 9.0) }]
}

/// A surface 40 high, from z = -10, along a sketch's open curve: a sheet part.
fn wall(doc: &mut Document, moves: Vec<Move>, start: Point2) -> FeatureId {
    let s = doc.with(Op::sketch(Placement::xy(-10.0), vec![Region::new(Shape::OpenPath { start, moves })]));
    feature(doc, &format!(r#"{{"type": "extrude", "sketch": {}, "length": 40, "mode": "surface"}}"#, s.0))
}

/// The block's volume below the wave, by a solid Boolean: the block intersected with a solid whose top is the wave.
fn below_the_wave() -> f64 {
    let mut doc = Document::new();
    block(&mut doc);
    let mut moves = wave();
    moves.extend([Move::Line { to: p(25.0, -5.0) }, Move::Line { to: p(-5.0, -5.0) }, Move::Line { to: p(-5.0, 8.0) }]);
    let s = doc.with(Op::sketch(Placement::xy(-10.0), vec![Region::new(Shape::Path { start: p(-5.0, 8.0), moves })]));
    doc.with(Op::extrude(s, 0.0, 40.0, Mode::Intersect));
    let o = ok(&mut doc);
    sizes(&o)[0].1
}

#[test]
fn a_sheet_part_splits_a_solid() {
    // A V-shaped wall, from (-5, 5) up to (10, 15) and down to (25, 5): below it the block's floor plan has an area of
    // 2 * (25/3 * 10 + 10 * 10 / 3) = 700 / 3, so 14000 / 3 of its 8000.
    let mut doc = Document::new();
    block(&mut doc);
    let v = wall(&mut doc, vec![Move::Line { to: p(10.0, 15.0) }, Move::Line { to: p(25.0, 5.0) }], p(-5.0, 5.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}"}}"#, v.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    // The wall faces to the right of the way it was drawn, down toward -y: the piece below it is in front and keeps the name.
    near(size_of(&o, "f2"), 14000.0 / 3.0, "below the V");
    near(size_of(&o, &format!("f{}", split.0)), 10000.0 / 3.0, "above it");
    // The tool is kept, as a sheet of 2 * sqrt(15^2 + 10^2) * 40.
    near(size_of(&o, &format!("f{}", v.0)), 2.0 * 325f64.sqrt() * 40.0, "the wall");
    // And the cut faces are the wall's, on both pieces.
    for b in &o.bodies {
        if !b.body.sheet {
            assert!(b.body.faces.iter().any(|f| f.tag.feature == v.0), "{} has the wall's faces", b.id.text());
        }
    }

    // A wavy wall through a spline: the piece below it is what a solid Boolean leaves below it.
    let mut doc = Document::new();
    block(&mut doc);
    let w = wall(&mut doc, wave(), p(-5.0, 8.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}", "keep_tool": false}}"#, w.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    let below = below_the_wave();
    near(size_of(&o, "f2"), below, "below the wave");
    near(size_of(&o, &format!("f{}", split.0)), 8000.0 - below, "above it");
    assert_eq!(o.bodies.len(), 2, "the wall is gone: {:?}", sizes(&o));
}

#[test]
fn a_face_on_an_extruded_curve_cuts_along_its_surface() {
    // A part whose side is the wave, as a part of its own beside the block, short of it at both ends: the split carries the
    // wave on past the part's own face, up and down and along the curve's ends, through the whole block.
    let mut doc = Document::new();
    block(&mut doc);
    let mut moves = wave();
    moves.extend([Move::Line { to: p(25.0, 30.0) }, Move::Line { to: p(-5.0, 30.0) }, Move::Line { to: p(-5.0, 8.0) }]);
    let s = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::Path { start: p(-5.0, 8.0), moves })]));
    let lid = doc.with(Op::extrude(s, 0.0, 5.0, Mode::NewBody));
    // Its wave's spline is segment 1: its face faces out of the lid, down toward -y, so the piece below it is in front.
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.1"}}"#, lid.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    let below = below_the_wave();
    near(size_of(&o, "f2"), below, "below the wave");
    near(size_of(&o, &format!("f{}", split.0)), 8000.0 - below, "above it");

    // The wave as the wall of a cut: a plate above the block with the region below the wave cut out of it. The wall is the
    // cut's face turned over, still facing out of the plate, down toward -y, so the piece below is again in front.
    let mut doc = Document::new();
    block(&mut doc);
    let s = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::rect(-5.0, -5.0, 30.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::NewBody));
    let mut moves = wave();
    moves.extend([Move::Line { to: p(25.0, -5.0) }, Move::Line { to: p(-5.0, -5.0) }, Move::Line { to: p(-5.0, 8.0) }]);
    let s = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::Path { start: p(-5.0, 8.0), moves })]));
    let cut = doc.with(Op::extrude(s, 0.0, 5.0, Mode::Remove));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.1"}}"#, cut.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    near(size_of(&o, "f2"), below, "below the wave, by a cut's wall");

    // A spline that stops short of the block's sides, from (4, 10) up through (10, 14) and down to (16, 10): its surface
    // carries on straight along its end tangents, through the block's sides. The tangents are the surface's own.
    let arch = || vec![Move::Spline { through: vec![p(10.0, 14.0), p(16.0, 10.0)] }];
    let mut doc = Document::new();
    block(&mut doc);
    let mut moves = arch();
    moves.extend([Move::Line { to: p(16.0, 30.0) }, Move::Line { to: p(4.0, 30.0) }, Move::Line { to: p(4.0, 10.0) }]);
    let s = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::Path { start: p(4.0, 10.0), moves })]));
    let lid = doc.with(Op::extrude(s, 0.0, 5.0, Mode::NewBody));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.0"}}"#, lid.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let o = ok(&mut doc);
    let lid_body = o.bodies.iter().find(|b| b.id.text() == format!("f{}", lid.0)).unwrap();
    let curve = lid_body
        .body
        .faces
        .iter()
        .find_map(|f| match &f.surface {
            Surface::Extruded { curve, .. } if f.tag.slot == 2 => Some(curve.clone()),
            _ => None,
        })
        .expect("the arch's face");
    let (t0, t1) = curve.spline.domain();
    let (d0, d1) = (curve.derivative(t0).normalized(), curve.derivative(t1).normalized());
    // The block below the arch and its tangents, by a solid Boolean.
    let (a, b) = (v2(4.0, 10.0) - d0 * 40.0, v2(16.0, 10.0) + d1 * 40.0);
    let mut check = Document::new();
    block(&mut check);
    let mut moves = vec![Move::Line { to: p(4.0, 10.0) }];
    moves.extend(arch());
    moves.extend([Move::Line { to: p(b.x, b.y) }, Move::Line { to: p(b.x, -60.0) }, Move::Line { to: p(a.x, -60.0) }, Move::Line { to: p(a.x, a.y) }]);
    let s = check.with(Op::sketch(Placement::xy(-10.0), vec![Region::new(Shape::Path { start: p(a.x, a.y), moves })]));
    check.with(Op::extrude(s, 0.0, 40.0, Mode::Intersect));
    let below = sizes(&ok(&mut check))[0].1;
    assert!(below > 1000.0 && below < 7000.0, "the arch and its tangents cut the block: {below}");
    near(size_of(&o, "f2"), below, "below the arch, carried on");
    near(size_of(&o, &format!("f{}", split.0)), 8000.0 - below, "above it");
}

#[test]
fn a_face_on_a_turned_curve_or_a_free_form_face_cuts_along_its_surface() {
    // A vase turned from a spline, and a bar through it: the bar's piece inside the vase is what an intersect leaves.
    let vase = |doc: &mut Document| {
        let through = vec![p(8.0, 3.0), p(5.0, 6.0)];
        let outline = Shape::Path { start: p(0.0, 0.0), moves: vec![Move::Line { to: p(6.0, 0.0) }, Move::Spline { through }, Move::Line { to: p(0.0, 6.0) }, Move::Line { to: p(0.0, 0.0) }] };
        let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(outline)]));
        doc.with(Op::revolve(s, 360.0, Mode::Add))
    };
    let bar = |doc: &mut Document, mode: Mode| {
        let s = doc.with(Op::sketch(Placement::xy(1.0), vec![Region::new(Shape::rect(-1.0, -12.0, 2.0, 24.0))]));
        doc.with(Op::extrude(s, 0.0, 2.0, mode));
    };
    let mut doc = Document::new();
    let v = vase(&mut doc);
    bar(&mut doc, Mode::NewBody);
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.1"}}"#, v.0));
    doc.set_scope(split, vec![BodyId::parse(&format!("f{}", v.0 + 2)).unwrap()]).unwrap();
    let o = ok(&mut doc);
    let mut check = Document::new();
    vase(&mut check);
    bar(&mut check, Mode::Intersect);
    let inside = sizes(&ok(&mut check))[0].1;
    let all: f64 = sizes(&o).iter().filter(|(n, _)| n != &format!("f{}", v.0)).map(|x| x.1).sum();
    near(all, 2.0 * 24.0 * 2.0, "the bar's pieces");
    // Out of the vase's face is out of the vase: the bar's two ends are in front, one under the bar's name and the other
    // named for the split, and the piece inside is behind, named for the split too.
    let bar_id = format!("f{}", v.0 + 2);
    let end = size_of(&o, &bar_id);
    near(end, (2.0 * 24.0 * 2.0 - inside) / 2.0, "an end of the bar");
    let rest: Vec<f64> = sizes(&o).into_iter().filter(|(n, _)| n.starts_with(&format!("f{}", split.0))).map(|x| x.1).collect();
    assert_eq!(rest.len(), 2, "{:?}", sizes(&o));
    assert!(rest.iter().any(|x| (x - inside).abs() < 1e-7 * inside) && rest.iter().any(|x| (x - end).abs() < 1e-7 * end), "{:?} with {inside} inside", sizes(&o));

    // A loft's side, free-form, from a circle of radius 10 down at 0 to one of radius 6 at 30: a block across it, inside
    // its height, is cut into what's inside the loft, as an intersect leaves, and the rest.
    let loft = |doc: &mut Document| {
        let a = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 10.0))]));
        let b = doc.with(Op::sketch(Placement::xy(30.0), vec![Region::new(Shape::circle(0.0, 0.0, 6.0))]));
        doc.add(Op::Loft { sections: vec![a.into(), b.into()], mode: Mode::Add, thin: None, shape: LoftShape::default() }).unwrap()
    };
    let slab = |doc: &mut Document, mode: Mode| {
        let s = doc.with(Op::sketch(Placement::xy(5.0), vec![Region::new(Shape::rect(-3.0, -15.0, 6.0, 30.0))]));
        doc.with(Op::extrude(s, 0.0, 20.0, mode));
    };
    let mut doc = Document::new();
    let l = loft(&mut doc);
    slab(&mut doc, Mode::NewBody);
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "face": "f{}:side.0"}}"#, l.0));
    doc.set_scope(split, vec![BodyId::parse(&format!("f{}", l.0 + 2)).unwrap()]).unwrap();
    let o = ok(&mut doc);
    let mut check = Document::new();
    loft(&mut check);
    slab(&mut check, Mode::Intersect);
    let inside = sizes(&ok(&mut check))[0].1;
    let rest: Vec<f64> = sizes(&o).into_iter().filter(|(n, _)| n.starts_with(&format!("f{}", split.0))).map(|x| x.1).collect();
    assert!(rest.iter().any(|x| (x - inside).abs() < 1e-7 * inside), "{:?} with {inside} inside the loft", sizes(&o));
    let all: f64 = sizes(&o).iter().filter(|(n, _)| n != &format!("f{}", l.0)).map(|x| x.1).sum();
    near(all, 6.0 * 30.0 * 20.0, "the slab's pieces");
    // The cut faces are named for the split, not for the loft.
    for b in o.bodies.iter().filter(|b| b.id.text() != format!("f{}", l.0)) {
        assert!(b.body.faces.iter().any(|f| f.tag.feature == split.0) && b.body.faces.iter().all(|f| f.tag.feature != l.0), "{}", b.id.text());
    }
}

#[test]
fn what_a_split_along_a_sheet_refuses() {
    // A wall that stops inside the block would only slit it.
    let mut doc = Document::new();
    block(&mut doc);
    let w = wall(&mut doc, vec![Move::Line { to: p(10.0, 10.0) }], p(-5.0, 10.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}"}}"#, w.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("ends inside"), "{why}");
    // One that misses it.
    let mut doc = Document::new();
    block(&mut doc);
    let w = wall(&mut doc, vec![Move::Line { to: p(25.0, 30.0) }], p(-5.0, 30.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}"}}"#, w.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("doesn't cut through body f2"), "{why}");
    // A ring cut across once stays in one piece.
    let mut doc = Document::new();
    let s = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::circle(0.0, 0.0, 10.0)).hole(Shape::circle(0.0, 0.0, 6.0))]));
    doc.with(Op::extrude(s, 0.0, 5.0, Mode::Add));
    let w = wall(&mut doc, vec![Move::Line { to: p(15.0, 0.0) }], p(3.0, 0.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}"}}"#, w.0));
    doc.set_scope(split, vec![BodyId::parse("f2").unwrap()]).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("one piece"), "{why}");
    // A sheet split by a sheet.
    let mut doc = Document::new();
    let a = wall(&mut doc, vec![Move::Line { to: p(25.0, 30.0) }], p(-5.0, 0.0));
    let b = wall(&mut doc, vec![Move::Line { to: p(25.0, 0.0) }], p(-5.0, 30.0));
    let split = feature(&mut doc, &format!(r#"{{"type": "split", "tool": "f{}"}}"#, b.0));
    doc.set_scope(split, vec![BodyId::parse(&format!("f{}", a.0)).unwrap()]).unwrap();
    let why = failure(&mut doc);
    assert!(why.contains("sheet with another sheet"), "{why}");
}
