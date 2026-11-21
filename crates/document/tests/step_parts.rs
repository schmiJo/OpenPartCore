//! A STEP import whose file has a part that can't come in: the feature brings in the rest, says what it left out and why,
//! and names each body after its part.

use coreshape::mass_properties;
use coreshape_document::*;
use std::sync::Arc;

/// Three or four tetrahedra, each a faceted solid of its own with its name, and the second with a face missing.
fn file(copies: bool) -> String {
    let mut lines = vec!["#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));".to_string()];
    let add = |lines: &mut Vec<String>, body: String| {
        let id = lines.len() + 1;
        lines.push(format!("#{id}={body};"));
        id
    };
    let solid = |lines: &mut Vec<String>, name: &str, x: f64, size: f64, faces: usize| {
        let at = [[x, 0.0, 0.0], [x + size, 0.0, 0.0], [x, size, 0.0], [x, 0.0, size]];
        let p: Vec<usize> = at.iter().map(|q| add(lines, format!("CARTESIAN_POINT('',({:.6},{:.6},{:.6}))", q[0], q[1], q[2]))).collect();
        let mut list = Vec::new();
        for (tri, n) in [([0, 2, 1], [0.0, 0.0, -1.0]), ([0, 1, 3], [0.0, -1.0, 0.0]), ([0, 3, 2], [-1.0, 0.0, 0.0]), ([1, 2, 3], [1.0, 1.0, 1.0])].iter().take(faces) {
            let lp = add(lines, format!("POLY_LOOP('',(#{},#{},#{}))", p[tri[0]], p[tri[1]], p[tri[2]]));
            let bound = add(lines, format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
            let o = add(lines, format!("CARTESIAN_POINT('',({:.6},{:.6},{:.6}))", at[tri[0]][0], at[tri[0]][1], at[tri[0]][2]));
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2] as f64).sqrt();
            let z = add(lines, format!("DIRECTION('',({:.9},{:.9},{:.9}))", n[0] / len, n[1] / len, n[2] / len));
            let a = add(lines, format!("AXIS2_PLACEMENT_3D('',#{o},#{z},$)"));
            let plane = add(lines, format!("PLANE('',#{a})"));
            list.push(format!("#{}", add(lines, format!("FACE_SURFACE('',(#{bound}),#{plane},.T.)"))));
        }
        let shell = add(lines, format!("CLOSED_SHELL('',({}))", list.join(",")));
        add(lines, format!("FACETED_BREP('{name}',#{shell})"))
    };
    solid(&mut lines, "wheel hub", 0.0, 10.0, 4);
    solid(&mut lines, "bent bracket", 50.0, 10.0, 3);
    solid(&mut lines, "M3x8 screw", 100.0, 20.0, 4);
    if copies {
        solid(&mut lines, "M3x8 screw", 100.0, 20.0, 4);
    }
    format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n", lines.join("\n"))
}

fn feature(doc: &mut Document, op: Op) -> Outcome {
    doc.add(op).unwrap();
    doc.rebuild(None)
}

#[test]
fn an_import_brings_in_what_it_can_and_says_what_it_left_out() {
    for parts in [false, true] {
        let mut doc = Document::new();
        let text = file(false);
        let op = if parts { Op::ImportStepParts { text } } else { Op::ImportStep { text, mode: Mode::NewBody } };
        let o = feature(&mut doc, op);
        assert_eq!(o.failed, None, "{:?}", o.steps);
        let step = &o.steps[0];
        assert_eq!(step.state, StepState::Rebuilt);
        assert_eq!(step.notes.len(), 1, "{:?}", step.notes);
        assert!(step.notes[0].contains("left out solid 2 of 3, \"bent bracket\"") && step.notes[0].contains("isn't closed"), "{}", step.notes[0]);
        let names: Vec<_> = o.bodies.iter().map(|b| b.name.clone().unwrap_or_default()).collect();
        assert_eq!(names, ["wheel hub", "M3x8 screw"]);
        let volumes: Vec<f64> = o.bodies.iter().map(|b| mass_properties(&b.body).volume).collect();
        assert!((volumes[0] - 1000.0 / 6.0).abs() < 1e-9 && (volumes[1] - 8000.0 / 6.0).abs() < 1e-9, "{volumes:?}");
        // A rebuild with nothing changed says it again.
        doc.set_parameter("x", 1.0).unwrap();
        let again = doc.rebuild(None);
        assert_eq!(again.steps[0].state, StepState::Reused);
        assert_eq!(again.steps[0].notes, step.notes);
        assert_eq!(again.bodies.iter().map(|b| b.name.clone()).collect::<Vec<_>>(), o.bodies.iter().map(|b| b.name.clone()).collect::<Vec<_>>());
    }
}

#[test]
fn copies_of_a_part_say_what_they_are_copies_of() {
    let mut doc = Document::new();
    let o = feature(&mut doc, Op::ImportStep { text: file(true), mode: Mode::NewBody });
    assert_eq!(o.bodies.len(), 3);
    let of = |k: usize| o.bodies[k].copy_of.as_ref().unwrap().body.clone();
    // The two screws are the same in the file, entity for entity, so they're copies of one body. The hub isn't.
    assert!(Arc::ptr_eq(&of(1), &of(2)));
    assert!(!Arc::ptr_eq(&of(0), &of(1)));
    // A copy is its part moved, with faces named apart.
    for b in &o.bodies {
        let c = b.copy_of.as_ref().unwrap();
        let moved = c.body.transformed(&c.at);
        assert_eq!(moved.vertices, b.body.vertices);
    }
    assert_ne!(o.bodies[1].body.faces[0].origin, o.bodies[2].body.faces[0].origin);
}

#[test]
fn an_import_with_nothing_that_can_come_in_fails_saying_why() {
    let bad = file(false).replace("FACETED_BREP('wheel hub'", "FACETED_BREPX('wheel hub'");
    let only_bad = {
        // Just the one with a face missing.
        let mut doc = Document::new();
        let text = bad.replace("FACETED_BREP('M3x8 screw'", "FACETED_BREPX('M3x8 screw'");
        let o = feature(&mut doc, Op::ImportStep { text, mode: Mode::NewBody });
        o.steps[0].state.clone()
    };
    match only_bad {
        StepState::Failed(why) => assert!(why.contains("nothing in this file could be brought in") && why.contains("bent bracket"), "{why}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn copies_placed_by_an_assembly_are_their_part_moved() {
    use coreshape::{box_solid, v3, Frame, Vec3};
    // One plate, used twice: moved along x, and turned a quarter about z.
    let plate = box_solid(&Frame::WORLD, 10.0, 20.0, 5.0, 1).unwrap();
    let moved = Frame::translation(v3(100.0, 0.0, 0.0));
    let turned = Frame::rotation(Vec3::ZERO, Vec3::Z, std::f64::consts::FRAC_PI_2);
    let text = coreshape::export::step::write_step_assembly(&[("plate", &plate)], &[(0, "first", moved), (0, "second", turned)], "kit");
    let mut doc = Document::new();
    let o = feature(&mut doc, Op::ImportStep { text, mode: Mode::NewBody });
    assert_eq!(o.bodies.len(), 2);
    let (a, b) = (o.bodies[0].copy_of.as_ref().unwrap(), o.bodies[1].copy_of.as_ref().unwrap());
    assert!(Arc::ptr_eq(&a.body, &b.body));
    for (body, at, lo, hi) in [(&o.bodies[0], moved, v3(100.0, 0.0, 0.0), v3(110.0, 20.0, 5.0)), (&o.bodies[1], turned, v3(-20.0, 0.0, 0.0), v3(0.0, 10.0, 5.0))] {
        let c = body.copy_of.as_ref().unwrap();
        assert!(c.at.origin.dist(at.origin) < 1e-9 && c.at.x.dist(at.x) < 1e-9 && c.at.y.dist(at.y) < 1e-9, "{:?}", c.at);
        let b = body.body.aabb();
        assert!(b.min.dist(lo) < 1e-9 && b.max.dist(hi) < 1e-9, "{b:?}");
        assert_eq!(body.name.as_deref(), Some("plate"));
    }
}
