//! A sheet is a part like any other in an assembly: it can be placed, mated by its faces, and exported, and there's nothing for it
//! to interfere with, as it has no inside.

use coreshape::*;
use coreshape_assembly::*;
use coreshape_document::*;

fn solid_block() -> Op {
    Op::solid(Primitive::Box { dx: 20.0.into(), dy: 20.0.into(), dz: 5.0.into() }, Placement::xy(0.0), Mode::NewBody)
}

/// A project with a block and a flat sheet in a Part Studio, and an assembly holding one of each.
fn set_up() -> (Project, TabId, u32, u32, BodyId, BodyId) {
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let doc = p.studio_mut(studio).unwrap();
    let block = BodyId { feature: doc.with(solid_block()), index: 0 };
    let sketch = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    let panel = BodyId { feature: doc.with(Op::Fill { sketch }), index: 0 };
    let tab = p.add_assembly("Assembly 1");
    let a = p.assembly_mut(tab).unwrap();
    let (i, j) = (a.add_instance(Source::Part { studio, body: block }, "Block".into(), Frame::WORLD), a.add_instance(Source::Part { studio, body: panel }, "Panel".into(), Frame::translation(v3(0.0, 0.0, 50.0))));
    a.instance_mut(i).unwrap().fixed = true;
    (p, tab, i, j, block, panel)
}

#[test]
fn a_sheet_is_placed_and_mated_by_its_face() {
    let (mut p, tab, i, j, block, panel) = set_up();
    let solved = p.solve(tab).unwrap();
    assert_eq!(solved.placed.len(), 2);
    assert!(solved.placed[1].body.sheet);
    let a = p.assembly_mut(tab).unwrap();
    let (ca, cb) = (a.add_connector(vec![i], Anchor::Face { key: FaceKey::plain(block.feature.end_cap()), point: FacePoint::Centroid }).unwrap(), a.add_connector(vec![j], Anchor::Face { key: FaceKey::plain(panel.feature.end_cap()), point: FacePoint::Centroid }).unwrap());
    a.add_mate(MateKind::Fastened, ca, cb).unwrap();
    let solved = p.solve(tab).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    assert_eq!(solved.freedom, 0);
    // Its centre is on the block's top.
    let at = solved.placed[1].at.to_world(v3(5.0, 5.0, 0.0));
    assert!(at.dist(v3(10.0, 10.0, 5.0)) < 1e-6, "{at:?}");
}

#[test]
fn a_sheet_doesnt_interfere_or_have_a_volume() {
    let (mut p, tab, _, j, _, _) = set_up();
    // Right through the block.
    p.assembly_mut(tab).unwrap().instance_mut(j).unwrap().at = Frame::translation(v3(5.0, 5.0, 2.0));
    let (found, unknown) = p.interferences(tab).unwrap();
    assert!(found.is_empty() && unknown.is_empty(), "{found:?} {unknown:?}");
    let bom = p.bom(tab).unwrap();
    assert_eq!(bom.rows.len(), 2);
    assert_eq!(bom.rows[1].volume, 0.0);
    assert!((bom.rows[1].area - 100.0).abs() < 1e-9);
}

#[test]
fn a_sheet_goes_out_in_an_assembly_and_comes_back_as_tabs() {
    let (mut p, tab, _, _, _, _) = set_up();
    let text = p.step_text(tab, "Mixed").unwrap();
    assert!(text.contains("MANIFOLD_SOLID_BREP") && text.contains("OPEN_SHELL"));
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE").count(), 2);
    // With solids in it, the solids are its parts, as a file's surfaces have to be asked for.
    let mut q = Project::new();
    let (_, back) = q.import_step(&text, "Mixed").unwrap();
    assert_eq!(q.solve(back).unwrap().placed.len(), 1);

    // A file of nothing but sheets brings them in as its parts.
    let mut only = Project::new();
    let studio = only.tabs[0].id;
    let doc = only.studio_mut(studio).unwrap();
    let sketch = doc.with(Op::sketch(Placement::xy(0.0), vec![Region::new(Shape::rect(0.0, 0.0, 10.0, 10.0))]));
    let panel = BodyId { feature: doc.with(Op::Fill { sketch }), index: 0 };
    let t = only.add_assembly("Panels");
    let a = only.assembly_mut(t).unwrap();
    a.add_instance(Source::Part { studio, body: panel }, "Panel <1>".into(), Frame::translation(v3(0.0, 0.0, 3.0)));
    a.add_instance(Source::Part { studio, body: panel }, "Panel <2>".into(), Frame::translation(v3(0.0, 0.0, 7.0)));
    let text = only.step_text(t, "Panels").unwrap();
    let mut q = Project::new();
    let (s, back) = q.import_step(&text, "Panels").unwrap();
    let o = q.studio_mut(s).unwrap().rebuild(None);
    assert_eq!(o.bodies.len(), 1, "{:?}", o.steps);
    assert!(o.bodies[0].body.sheet);
    let solved = q.solve(back).unwrap();
    assert!(solved.problems.is_empty(), "{:?}", solved.problems);
    assert_eq!(solved.placed.len(), 2);
    let heights: Vec<f64> = solved.placed.iter().map(|x| x.at.origin.z).collect();
    assert_eq!(heights, vec![3.0, 7.0]);
    assert!(solved.placed.iter().all(|x| x.body.sheet));
}
