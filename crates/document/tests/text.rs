//! Text in a sketch, raised from nothing or cut into a part, held against the area the sketch's regions give, which its own
//! tests hold against the font's outlines.

use coreshape::*;
use coreshape_document::*;
use coreshape_sketch::{Part, Ref, Region, Rule, Sketch, ORIGIN};

fn ok(doc: &mut Document) -> Outcome {
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None, "{:?}", o.steps);
    for b in &o.bodies {
        validate(&b.body).unwrap_or_else(|e| panic!("{e:?}"));
    }
    o
}

fn vol(o: &Outcome) -> f64 {
    o.bodies.iter().map(|b| mass_properties(&b.body).volume).sum()
}

fn area(sketch: &Sketch) -> f64 {
    sketch.regions().unwrap().iter().map(Region::area).sum::<f64>().abs()
}

fn whole(id: coreshape_sketch::EntityId) -> Ref {
    Ref { entity: id, part: Part::Whole }
}

#[test]
fn text_raised_and_cut() {
    // "CoreShape 2026" 8 high, raised 2 from the XY plane.
    let mut sketch = Sketch::new();
    let t = sketch.text("CoreShape 2026", [0.0, 0.0], 8.0);
    let a = area(&sketch);
    let mut doc = Document::new();
    let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: sketch.clone() }).unwrap();
    let e = doc.add(Op::extrude(s, 0.0, 2.0, Mode::Add)).unwrap();
    let o = ok(&mut doc);
    let v = vol(&o);
    assert!((v - 2.0 * a).abs() < 1e-8 * v, "{v} against {}", 2.0 * a);
    // Its sides are named by the text's id.
    let side = format!("f{}:side.{}", e.0, t.0);
    assert!(o.bodies.iter().flat_map(|b| b.body.faces.iter()).any(|f| FaceRef::from_tag(f.tag).name() == side), "{side}");
    // Cut 1 deep into a plate 90 by 14 by 5 under it.
    let mut doc = Document::new();
    doc.add(Op::solid(Primitive::Box { dx: 90.0.into(), dy: 14.0.into(), dz: 5.0.into() }, Placement::at([(-2.0).into(), (-3.0).into(), (-5.0).into()]), Mode::Add)).unwrap();
    let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch }).unwrap();
    doc.add(Op::extrude(s, -1.0, 1.0, Mode::Remove)).unwrap();
    let v = vol(&ok(&mut doc));
    let want = 90.0 * 14.0 * 5.0 - a;
    assert!((v - want).abs() < 1e-8 * want, "{v} against {want}");
}

#[test]
fn text_sized_by_a_dimension() {
    // At the origin, upright by a vertical constraint, as high as the parameter h says.
    let mut sketch = Sketch::new();
    let t = sketch.text("Hi", [1.0, 1.0], 5.0);
    let (at, up) = (sketch.text_of(t).unwrap().at, sketch.text_of(t).unwrap().up);
    sketch.constrain(Rule::Coincident { a: whole(at), b: whole(ORIGIN) });
    sketch.constrain(Rule::Vertical { a: whole(at), b: Some(whole(up)) });
    sketch.constrain(Rule::Distance { a: whole(at), b: Some(whole(up)), value: "h".into() });
    let mut doc = Document::new();
    doc.set_parameter("h", 10.0).unwrap();
    let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: sketch.clone() }).unwrap();
    doc.add(Op::extrude(s, 0.0, 1.0, Mode::Add)).unwrap();
    let ten = vol(&ok(&mut doc));
    doc.set_parameter("h", 20.0).unwrap();
    let twenty = vol(&ok(&mut doc));
    assert!((twenty - 4.0 * ten).abs() < 1e-8 * twenty, "{twenty} against 4 times {ten}");
    // The same as the sketch at that size gives.
    let mut env = Env::new();
    env.insert("h".into(), 20.0);
    let solved = sketch.solve_with(&env).unwrap();
    assert!((twenty - area(&solved.sketch)).abs() < 1e-8 * twenty);
}

#[test]
fn a_character_the_font_hasnt_got_is_noted() {
    let mut sketch = Sketch::new();
    sketch.text("a\u{2603}b", [0.0, 0.0], 5.0);
    let mut doc = Document::new();
    let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch }).unwrap();
    let o = doc.rebuild(None);
    assert_eq!(o.failed, None);
    let step = o.steps.iter().find(|x| x.id == s).unwrap();
    assert_eq!(step.notes, vec!["text 3 has characters its font hasn't got, left out with a space's room: \u{2603}".to_string()]);
}

#[test]
fn accented_letters_that_overlap_close_up() {
    for h in [9.0, 8.0, 4.0, 14.0] {
        let mut sketch = Sketch::new();
        sketch.text("\u{C5}ngstr\u{F6}m \u{C7}a va", [0.0, 0.0], h);
        let mut doc = Document::new();
        let s = doc.add(Op::ConstrainedSketch { plane: Placement::xy(0.0), sketch: sketch.clone() }).unwrap();
        doc.add(Op::extrude(s, 0.0, 1.0, Mode::Add)).unwrap();
        let v = vol(&ok(&mut doc));
        assert!((v - area(&sketch)).abs() < 1e-8 * v, "height {h}: {v} against {}", area(&sketch));
    }
}
