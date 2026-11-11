//! Assemblies as Onshape has them, from parts in a Part Studio. Each result is checked against what the mates say by a
//! formula, or by where the connectors end up. A part that changes has to bring what's mated to it along.

use coreshape::export::step::write_step_bodies;
use coreshape::import::step::{read_step, read_step_parts, read_step_structure};
use coreshape::*;
use coreshape_assembly::*;
use coreshape_document::*;

const NEAR: f64 = 1e-6;

fn block(x: &str, y: &str, z: &str) -> Op {
    Op::solid(Primitive::Box { dx: x.into(), dy: y.into(), dz: z.into() }, Placement::xy(0.0), Mode::NewBody)
}

fn pin(r: &str, h: &str) -> Op {
    Op::solid(Primitive::Cylinder { radius: r.into(), height: h.into() }, Placement::xy(0.0), Mode::NewBody)
}

/// A project with one Part Studio holding these parts, and an empty assembly.
fn project(parts: Vec<Op>) -> (Project, TabId, TabId, Vec<BodyId>) {
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let ids = parts.into_iter().map(|op| BodyId { feature: p.studio_mut(studio).unwrap().with(op), index: 0 }).collect();
    let assembly = p.add_assembly("Assembly 1");
    (p, studio, assembly, ids)
}

fn instance(p: &mut Project, assembly: TabId, studio: TabId, body: BodyId, name: &str) -> u32 {
    p.assembly_mut(assembly).unwrap().add_instance(Source::Part { studio, body }, name.to_string(), Frame::WORLD)
}

fn end_of(body: BodyId) -> Anchor {
    Anchor::Face { key: FaceKey::plain(body.feature.end_cap()), point: FacePoint::Centroid }
}

fn start_of(body: BodyId) -> Anchor {
    Anchor::Face { key: FaceKey::plain(body.feature.start_cap()), point: FacePoint::Centroid }
}

/// A connector at a place on an instance, given in its part's coordinates.
fn at_point(p: &mut Project, assembly: TabId, on: u32, origin: Vec3, z: Vec3, x: Vec3) -> u32 {
    p.assembly_mut(assembly).unwrap().add_connector(vec![on], Anchor::Frame(Frame::new(origin, z, x))).unwrap()
}

fn mate(p: &mut Project, assembly: TabId, kind: MateKind, a: u32, b: u32) -> u32 {
    p.assembly_mut(assembly).unwrap().add_mate(kind, a, b).unwrap()
}

fn pose(solved: &Solved, id: u32) -> Frame {
    solved.placed.iter().find(|q| q.path == [id]).unwrap_or_else(|| panic!("instance {id} isn't placed")).at
}

fn value(solved: &Solved, mate: u32, dof: Dof) -> f64 {
    solved.values.iter().find(|v| v.0 == mate && v.1 == dof).unwrap().2
}

fn close(a: Vec3, b: Vec3) {
    assert!(a.dist(b) < NEAR, "{a:?} isn't {b:?}");
}

/// The crank-and-slider of the demo: a crank turning about the origin in the XY plane, a rod, and a slider held to X.
struct SliderCrank {
    p: Project,
    assembly: TabId,
    crank_joint: u32,
    slider_joint: u32,
    slider: u32,
    crank: f64,
    rod: f64,
}

fn slider_crank() -> SliderCrank {
    let (crank, rod) = (30.0, 100.0);
    let (mut p, studio, assembly, parts) = project(vec![block("40", "10", "5"), block("120", "10", "5"), block("20", "20", "20"), block("200", "40", "5")]);
    let ground = instance(&mut p, assembly, studio, parts[3], "Ground");
    let c = instance(&mut p, assembly, studio, parts[0], "Crank");
    let r = instance(&mut p, assembly, studio, parts[1], "Rod");
    let s = instance(&mut p, assembly, studio, parts[2], "Slider");
    p.assembly_mut(assembly).unwrap().instance_mut(ground).unwrap().fixed = true;
    let z = Vec3::Z;
    let x = Vec3::X;
    let g1 = at_point(&mut p, assembly, ground, v3(0.0, 0.0, 0.0), z, x);
    let c1 = at_point(&mut p, assembly, c, v3(5.0, 5.0, 0.0), z, x);
    let c2 = at_point(&mut p, assembly, c, v3(5.0 + crank, 5.0, 0.0), z, x);
    let r1 = at_point(&mut p, assembly, r, v3(5.0, 5.0, 0.0), z, x);
    let r2 = at_point(&mut p, assembly, r, v3(5.0 + rod, 5.0, 0.0), z, x);
    let s1 = at_point(&mut p, assembly, s, v3(10.0, 10.0, 0.0), z, x);
    let s2 = at_point(&mut p, assembly, s, v3(10.0, 10.0, 0.0), x, Vec3::Y);
    let g2 = at_point(&mut p, assembly, ground, v3(0.0, 0.0, 0.0), x, Vec3::Y);
    let crank_joint = mate(&mut p, assembly, MateKind::Revolute, g1, c1);
    mate(&mut p, assembly, MateKind::Revolute, c2, r1);
    mate(&mut p, assembly, MateKind::Revolute, r2, s1);
    let slider_joint = mate(&mut p, assembly, MateKind::Slider, g2, s2);
    // Start it off in a sensible place, on the branch with the slider out along +X.
    p.assembly_mut(assembly).unwrap().instance_mut(s).unwrap().at = Frame::translation(v3(crank + rod - 10.0, -10.0, 0.0));
    p.assembly_mut(assembly).unwrap().instance_mut(c).unwrap().at = Frame::translation(v3(-5.0, -5.0, 0.0));
    p.assembly_mut(assembly).unwrap().instance_mut(r).unwrap().at = Frame::translation(v3(crank - 5.0, -5.0, 0.0));
    SliderCrank { p, assembly, crank_joint, slider_joint, slider: s, crank, rod }
}

/// How far a slider is from the crank's axis, by the formula for a crank and rod.
fn slider_at(crank: f64, rod: f64, angle: f64) -> f64 {
    crank * angle.cos() + (rod * rod - (crank * angle.sin()).powi(2)).sqrt()
}

// ----- Connectors -----

#[test]
fn a_connector_on_a_face_is_where_the_face_is() {
    let body = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let f = |name: &str| FaceKey::parse(name).unwrap();
    let top = Anchor::Face { key: f("f1:end"), point: FacePoint::Centroid }.resolve(&body).unwrap();
    close(top.origin, v3(5.0, 10.0, 30.0));
    // Out of the face: that's its primary axis.
    close(top.z, Vec3::Z);
    let bottom = Anchor::Face { key: f("f1:start"), point: FacePoint::Centroid }.resolve(&body).unwrap();
    close(bottom.origin, v3(5.0, 10.0, 0.0));
    close(bottom.z, -Vec3::Z);
    // A corner of it, picked by being near.
    let corner = Anchor::Face { key: f("f1:end"), point: FacePoint::Near(v3(11.0, 21.0, 30.0)) }.resolve(&body).unwrap();
    close(corner.origin, v3(10.0, 20.0, 30.0));
    close(corner.z, Vec3::Z);
    // The middle of an edge of it.
    let middle = Anchor::Face { key: f("f1:end"), point: FacePoint::Near(v3(5.0, 21.0, 30.0)) }.resolve(&body).unwrap();
    close(middle.origin, v3(5.0, 20.0, 30.0));
}

#[test]
fn a_flat_face_with_a_hole_has_its_centroid_where_the_area_says() {
    // A 40 by 40 plate with a hole of radius 5 at (30, 20). The centroid moves away from the hole by a formula.
    let plate = box_solid(&Frame::WORLD, 40.0, 40.0, 10.0, 1).unwrap();
    let hole = cylinder(&Frame::translation(v3(30.0, 20.0, -1.0)), 5.0, 12.0, 2).unwrap();
    let plate = subtract(&plate, &hole).unwrap();
    let at = Anchor::Face { key: FaceKey::parse("f1:end").unwrap(), point: FacePoint::Centroid }.resolve(&plate).unwrap();
    let (whole, gone) = (1600.0, std::f64::consts::PI * 25.0);
    let want = (20.0 * whole - 30.0 * gone) / (whole - gone);
    // The hole is sampled with 64 straight pieces, which is close enough to put something on, and no more.
    assert!((at.origin.x - want).abs() < 1e-2, "{} isn't {want}", at.origin.x);
    assert!((at.origin.y - 20.0).abs() < 1e-9);
}

#[test]
fn a_round_face_gives_its_axis() {
    let frame = Frame::new(v3(1.0, 2.0, 3.0), Vec3::X, Vec3::Y);
    let body = cylinder(&frame, 5.0, 20.0, 1).unwrap();
    let side = FaceKey::parse("f1:side.0").unwrap();
    let get = |point| Anchor::Face { key: side.clone(), point }.resolve(&body).unwrap();
    close(get(FacePoint::AxisStart).origin, v3(1.0, 2.0, 3.0));
    close(get(FacePoint::AxisEnd).origin, v3(21.0, 2.0, 3.0));
    close(get(FacePoint::AxisMiddle).origin, v3(11.0, 2.0, 3.0));
    close(get(FacePoint::AxisMiddle).z, Vec3::X);
    // There's no tip on a cylinder, and it says so.
    assert!(Anchor::Face { key: side.clone(), point: FacePoint::Apex }.resolve(&body).is_err());
    // A cone has one, on its axis at the height the slope says.
    let cone = cone_frustum(&Frame::WORLD, 10.0, 5.0, 10.0, 2).unwrap();
    let slope = cone.faces.iter().position(|f| matches!(f.surface, Surface::Cone { .. })).unwrap();
    let tip = Anchor::Face { key: face_keys(&cone)[slope].clone(), point: FacePoint::Apex }.resolve(&cone).unwrap();
    close(tip.origin, v3(0.0, 0.0, 20.0));
}

#[test]
fn a_connector_on_an_edge_goes_along_it_or_at_its_center() {
    let body = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let keys = face_keys(&body);
    // The edge along X at the top of the wall at y = 0.
    let edge = (0..body.edges.len())
        .find(|&e| {
            let (a, b) = (body.vertices[body.edges[e].v0], body.vertices[body.edges[e].v1]);
            a.y.abs() < 1e-9 && b.y.abs() < 1e-9 && a.z == 30.0 && b.z == 30.0
        })
        .unwrap();
    let uses = body.edge_uses();
    let faces = [keys[uses[edge][0].0].clone(), keys[uses[edge][1].0].clone()];
    let got = Anchor::Edge { faces: faces.clone(), point: EdgePoint::Middle }.resolve(&body).unwrap();
    close(got.origin, v3(5.0, 0.0, 30.0));
    assert!(got.z.cross(Vec3::X).norm() < 1e-9);
    assert!(Anchor::Edge { faces: faces.clone(), point: EdgePoint::Center }.resolve(&body).is_err());
    let list = edge_candidates(&body, edge, &faces);
    assert_eq!(list.len(), 3);

    // A round one has a center, on its axis.
    let round = cylinder(&Frame::translation(v3(1.0, 2.0, 3.0)), 5.0, 20.0, 2).unwrap();
    let keys = face_keys(&round);
    let uses = round.edge_uses();
    let e = (0..round.edges.len()).find(|&e| matches!(round.edges[e].curve, Curve::Circle { .. }) && round.vertices[round.edges[e].v0].z > 10.0).unwrap();
    let faces = [keys[uses[e][0].0].clone(), keys[uses[e][1].0].clone()];
    let center = Anchor::Edge { faces, point: EdgePoint::Center }.resolve(&round).unwrap();
    close(center.origin, v3(1.0, 2.0, 23.0));
    assert!(center.z.cross(Vec3::Z).norm() < 1e-9);
}

#[test]
fn candidates_offer_what_a_face_has() {
    let body = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let key = FaceKey::parse("f1:end").unwrap();
    let face = find_faces(&body, &key)[0];
    let list = candidates(&body, face, &key);
    // The centroid, four corners, four edge middles.
    assert_eq!(list.len(), 9);
    assert!(list[0].label.contains("centroid"));
    // Every one of them comes back to the same place from its anchor.
    for c in &list {
        close(c.anchor.resolve(&body).unwrap().origin, c.frame.origin);
    }
    let round = cylinder(&Frame::WORLD, 5.0, 20.0, 2).unwrap();
    let key = FaceKey::parse("f2:side.0").unwrap();
    let list = candidates(&round, find_faces(&round, &key)[0], &key);
    assert!(list.iter().any(|c| c.label.contains("middle of the axis")));
    // And on the flat end of it, the center of the circle round it.
    let key = FaceKey::parse("f2:end").unwrap();
    let list = candidates(&round, find_faces(&round, &key)[0], &key);
    assert!(list.iter().any(|c| c.label.contains("center") && c.frame.origin.dist(v3(0.0, 0.0, 20.0)) < 1e-9));
}

// ----- Mates between instances -----

#[test]
fn a_fastened_mate_brings_one_face_onto_another() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "20", "30"), block("4", "4", "6")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "Base"), instance(&mut p, assembly, studio, parts[1], "Top"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (asm.add_connector(vec![a], end_of(parts[0])).unwrap(), asm.add_connector(vec![b], start_of(parts[1])).unwrap());
    let m = asm.add_mate(MateKind::Fastened, ca, cb).unwrap();
    // Their primary axes end up the same way, and both point out of their faces. Flipped, the faces lie against each other.
    asm.mate_mut(m).unwrap().flip = true;
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    assert_eq!(solved.freedom, 0);
    let top = pose(&solved, b);
    close(top.to_world(v3(2.0, 2.0, 0.0)), v3(5.0, 10.0, 30.0));
    close(top.z, Vec3::Z);
    // Sitting on it, rather than sunk into it.
    close(top.to_world(v3(2.0, 2.0, 6.0)), v3(5.0, 10.0, 36.0));
    assert_eq!(p.interferences(assembly).unwrap().0, vec![]);

    // Not flipped, it's on the same side, as z has to agree. That's inside the other part.
    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().flip = false;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).to_world(v3(2.0, 2.0, 0.0)), v3(5.0, 10.0, 30.0));
    close(pose(&solved, b).to_world(v3(2.0, 2.0, 6.0)), v3(5.0, 10.0, 24.0));
}

#[test]
fn a_mates_offset_and_turns_do_what_they_say() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let ca = at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X);
    let cb = at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X);
    let m = mate(&mut p, assembly, MateKind::Fastened, ca, cb);
    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().offset = [1.0, 2.0, 3.0];
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).origin, v3(1.0, 2.0, 3.0));

    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().rotation = 90.0;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).x, Vec3::Y);
    close(pose(&solved, b).origin, v3(1.0, 2.0, 3.0));

    // A quarter turn more, of the second connector.
    let asm = p.assembly_mut(assembly).unwrap();
    asm.mate_mut(m).unwrap().rotation = 0.0;
    asm.mate_mut(m).unwrap().quarter_turns = 1;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).x, -Vec3::Y);

    // A connector's own adjustments come first, along its own axes.
    let asm = p.assembly_mut(assembly).unwrap();
    asm.mate_mut(m).unwrap().quarter_turns = 0;
    asm.mate_mut(m).unwrap().offset = [0.0; 3];
    asm.connector_mut(ca).unwrap().offset = [5.0, 0.0, 0.0];
    asm.connector_mut(ca).unwrap().rotation = 90.0;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).origin, v3(5.0, 0.0, 0.0));
    close(pose(&solved, b).x, Vec3::Y);

    let asm = p.assembly_mut(assembly).unwrap();
    asm.connector_mut(ca).unwrap().rotation = 0.0;
    asm.connector_mut(ca).unwrap().quarter_turns = 2;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).x, -Vec3::X);

    let asm = p.assembly_mut(assembly).unwrap();
    asm.connector_mut(ca).unwrap().quarter_turns = 0;
    asm.connector_mut(ca).unwrap().flip = true;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).z, -Vec3::Z);
}

#[test]
fn each_kind_of_mate_leaves_its_freedom() {
    for kind in [MateKind::Fastened, MateKind::Revolute, MateKind::Slider, MateKind::Cylindrical, MateKind::PinSlot, MateKind::Planar, MateKind::Ball, MateKind::Parallel] {
        let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("5", "5", "5")]);
        let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
        p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
        let (ca, cb) = (at_point(&mut p, assembly, a, v3(10.0, 0.0, 0.0), Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
        mate(&mut p, assembly, kind, ca, cb);
        let solved = p.solve(assembly).unwrap();
        assert!(solved.failing.is_empty());
        assert_eq!(solved.freedom, kind.freedom(), "{kind:?}");
        assert_eq!(solved.free.is_empty(), kind.freedom() == 0, "{kind:?}");
    }
}

#[test]
fn the_first_end_of_a_new_mate_does_the_moving() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(b).unwrap().at = Frame::translation(v3(50.0, 0.0, 0.0));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    mate(&mut p, assembly, MateKind::Fastened, ca, cb);
    let solved = p.solve_preferring(assembly, &[a]).unwrap();
    // Neither is fixed, and yet it's the one asked that moves. A thousandth of it goes to the other.
    assert!(pose(&solved, b).origin.dist(v3(50.0, 0.0, 0.0)) < 1e-3, "{:?}", pose(&solved, b).origin);
    close(pose(&solved, a).origin, pose(&solved, b).origin);
}

#[test]
fn a_new_mate_that_leaves_a_turn_free_comes_to_rest_at_zero() {
    // The shaft has to turn over to get to its bearing, and how it gets there leaves it at some angle or other.
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "90")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "Bearing"), instance(&mut p, assembly, studio, parts[1], "Shaft"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let on_shaft = Anchor::Frame(Frame::new(v3(5.0, 5.0, 45.0), Vec3::Z, Vec3::X));
    let on_bearing = Anchor::Frame(Frame::new(v3(5.0, 5.0, 5.0), v3(0.3, 0.8, 0.5), Vec3::X));
    let asm = p.assembly_mut(assembly).unwrap();
    let (ca, cb) = (asm.add_connector(vec![b], on_shaft).unwrap(), asm.add_connector(vec![a], on_bearing).unwrap());
    let m = asm.add_mate(MateKind::Revolute, ca, cb).unwrap();
    let solved = p.solve_new_mate(assembly, m, &[b]).unwrap();
    assert!(solved.failing.is_empty());
    assert!(value(&solved, m, Dof::Angle).abs() < 1e-6, "{}", value(&solved, m, Dof::Angle));
    assert_eq!(solved.freedom, 1);
}

#[test]
fn a_new_mate_doesnt_swing_what_something_else_holds() {
    // The crank press's last mate closes a loop, and the mechanism stays where it was rather than going to dead center.
    let mut s = slider_crank();
    s.p.solve(s.assembly).unwrap();
    s.p.set_mate_value(s.assembly, s.crank_joint, Dof::Angle, 60.0).unwrap();
    let last = s.p.assembly(s.assembly).unwrap().mates.iter().find(|m| m.id != s.crank_joint && m.kind == MateKind::Revolute).unwrap().id;
    let first = s.p.assembly(s.assembly).unwrap().instances[2].id;
    let solved = s.p.solve_new_mate(s.assembly, last, &[first]).unwrap();
    assert!((value(&solved, s.crank_joint, Dof::Angle) - 60.0).abs() < 1e-6, "{}", value(&solved, s.crank_joint, Dof::Angle));
}

#[test]
fn what_isnt_mated_stays_where_it_is() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    asm.instance_mut(c).unwrap().at = Frame::translation(v3(70.0, 80.0, 90.0));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, c).origin, v3(70.0, 80.0, 90.0));
    // C is free, in six ways, and B in one.
    assert_eq!(solved.freedom, 7);
    assert_eq!(solved.free.len(), 2);
}

#[test]
fn a_mate_that_cant_be_met_is_named_and_the_rest_still_are() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    asm.instance_mut(c).unwrap().fixed = true;
    asm.instance_mut(c).unwrap().at = Frame::translation(v3(100.0, 0.0, 0.0));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (cc, cd) = (at_point(&mut p, assembly, c, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let good = mate(&mut p, assembly, MateKind::Fastened, ca, cb);
    let bad = mate(&mut p, assembly, MateKind::Fastened, cc, cd);
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.failing.iter().map(|f| f.0).collect::<Vec<_>>(), vec![bad], "{:?}, and {good} is good", solved.failing);
    close(pose(&solved, b).origin, Vec3::ZERO);
}

// ----- A part changes -----

#[test]
fn a_part_that_changes_brings_what_is_mated_to_it() {
    // A pin standing on the top face of a block, whose height is a parameter.
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let doc = p.studio_mut(studio).unwrap();
    doc.set_parameter("h", "30").unwrap();
    let base = BodyId { feature: doc.with(block("40", "40", "h")), index: 0 };
    let post = BodyId { feature: doc.with(pin("5", "20")), index: 0 };
    let assembly = p.add_assembly("Assembly 1");
    let (a, b) = (instance(&mut p, assembly, studio, base, "Base"), instance(&mut p, assembly, studio, post, "Post"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (asm.add_connector(vec![a], end_of(base)).unwrap(), asm.add_connector(vec![b], start_of(post)).unwrap());
    let m = asm.add_mate(MateKind::Fastened, ca, cb).unwrap();
    asm.mate_mut(m).unwrap().flip = true;
    let solved = p.solve(assembly).unwrap();
    close(pose(&solved, b).origin, v3(20.0, 20.0, 30.0));

    // Taller, and wider so the centroid moves too.
    let doc = p.studio_mut(studio).unwrap();
    doc.set_parameter("h", "55").unwrap();
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty());
    close(pose(&solved, b).origin, v3(20.0, 20.0, 55.0));
    // The instance's own volume changed with it, as it's that very part.
    assert!((mass_properties(&solved.placed[0].body).volume - 40.0 * 40.0 * 55.0).abs() < 1e-6);
}

#[test]
fn a_part_that_goes_away_is_reported_and_its_mates_named() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (cc, cd) = (at_point(&mut p, assembly, c, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (m1, m2) = (mate(&mut p, assembly, MateKind::Fastened, ca, cb), mate(&mut p, assembly, MateKind::Fastened, cc, cd));
    p.studio_mut(studio).unwrap().suppress(parts[0].feature, true).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.placed.len(), 2);
    assert_eq!(solved.problems.len(), 1, "{:?}", solved.problems);
    assert!(solved.problems[0].contains('A'));
    assert_eq!(solved.failing.iter().map(|f| f.0).collect::<Vec<_>>(), vec![m1]);
    assert!(solved.failing[0].1.contains("isn't there"));
    let _ = m2;

    // It comes back when the part does.
    p.studio_mut(studio).unwrap().suppress(parts[0].feature, false).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.placed.len(), 3);
    assert!(solved.problems.is_empty());
    assert!(solved.failing.is_empty());
}

#[test]
fn a_face_that_goes_away_fails_only_its_mate() {
    // A connector on the top of a block. A fillet-free change: a new extrude that merges over the top renames nothing,
    // but pointing it at a face that was never there says which and why.
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    let asm = p.assembly_mut(assembly).unwrap();
    let gone = Anchor::Face { key: FaceKey::parse("f99:end").unwrap(), point: FacePoint::Centroid };
    let (ca, cb) = (asm.add_connector(vec![a], gone).unwrap(), asm.add_connector(vec![b], start_of(parts[1])).unwrap());
    let m = asm.add_mate(MateKind::Fastened, ca, cb).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.failing.len(), 1);
    assert_eq!(solved.failing[0].0, m);
    assert!(solved.failing[0].1.contains("f99:end"), "{}", solved.failing[0].1);
    assert_eq!(solved.placed.len(), 2);
    let _ = studio;
}

// ----- Moving things -----

#[test]
fn a_slider_crank_follows_its_formula_over_two_turns() {
    let mut s = slider_crank();
    let first = s.p.solve(s.assembly).unwrap();
    assert!(first.failing.is_empty(), "{:?}", first.failing);
    assert_eq!(first.freedom, 1);
    let mut worst = 0.0f64;
    for k in 0..=48 {
        let degrees = k as f64 * 15.0;
        let solved = s.p.set_mate_value(s.assembly, s.crank_joint, Dof::Angle, degrees).unwrap();
        assert!(solved.failing.is_empty());
        let got = value(&solved, s.crank_joint, Dof::Angle);
        assert!((got - degrees).abs() < 1e-5, "asked for {degrees} and got {got}");
        let want = slider_at(s.crank, s.rod, degrees.to_radians());
        // The slider's point goes along X from the origin.
        let at = pose(&solved, s.slider).to_world(v3(10.0, 10.0, 0.0));
        worst = worst.max((at.x - want).abs());
        assert!(at.y.abs() < 1e-6 && at.z.abs() < 1e-6, "{at:?}");
    }
    assert!(worst < 1e-5, "off by {worst}");
    // And it keeps count of its turns.
    assert!((value(&s.p.solve(s.assembly).unwrap(), s.crank_joint, Dof::Angle) - 720.0).abs() < 1e-5);
}

#[test]
fn a_mates_value_saved_and_read_back_is_still_on_its_turn() {
    let mut s = slider_crank();
    s.p.solve(s.assembly).unwrap();
    s.p.set_mate_value(s.assembly, s.crank_joint, Dof::Angle, 400.0).unwrap();
    let mut again = Project::load_text(&s.p.save_text()).unwrap();
    let solved = again.solve(s.assembly).unwrap();
    assert!((value(&solved, s.crank_joint, Dof::Angle) - 400.0).abs() < 1e-5);
}

#[test]
fn a_slider_driven_past_where_it_can_go_stops_at_the_end() {
    let mut s = slider_crank();
    s.p.solve(s.assembly).unwrap();
    // At most crank + rod from the origin: the point of it starts at x = 130, and that's zero on this mate.
    let solved = s.p.set_mate_value(s.assembly, s.slider_joint, Dof::Z, 20.0).unwrap();
    assert!(solved.failing.is_empty());
    let at = pose(&solved, s.slider).to_world(v3(10.0, 10.0, 0.0));
    assert!(at.x <= s.crank + s.rod + 1e-6, "{at:?}");
    // It starts at the very end of its travel, so there's nowhere further to go.
    assert!(at.x > s.crank + s.rod - 1e-3, "{at:?}");
    assert!(solved.remaining.is_none());
}

#[test]
fn a_mate_driven_past_a_stop_ends_up_at_the_stop() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("100", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "Base"), instance(&mut p, assembly, studio, parts[1], "Arm"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let m = mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().limits = vec![MateLimit { dof: Dof::Angle, min: -30.0, max: 47.0 }];
    p.solve(assembly).unwrap();
    // Steps of 15 degrees would stop short of it at 45, if that was all there was to it.
    let got = value(&p.set_mate_value(assembly, m, Dof::Angle, 180.0).unwrap(), m, Dof::Angle);
    assert!((got - 47.0).abs() < 0.01 && got <= 47.0 + 1e-6, "stopped at {got}");
    let got = value(&p.set_mate_value(assembly, m, Dof::Angle, -180.0).unwrap(), m, Dof::Angle);
    assert!((got + 30.0).abs() < 0.01 && got >= -30.0 - 1e-6, "stopped at {got}");
}

#[test]
fn driving_a_mate_doesnt_break_a_gear_to_get_there() {
    let (mut p, studio, assembly, parts) = project(vec![block("200", "10", "1"), block("10", "10", "10"), block("10", "10", "10")]);
    let (g, a, b) = (instance(&mut p, assembly, studio, parts[0], "Ground"), instance(&mut p, assembly, studio, parts[1], "A"), instance(&mut p, assembly, studio, parts[2], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(g).unwrap().fixed = true;
    let g1 = at_point(&mut p, assembly, g, Vec3::ZERO, Vec3::Z, Vec3::X);
    let g2 = at_point(&mut p, assembly, g, v3(100.0, 0.0, 0.0), Vec3::Z, Vec3::X);
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (ma, mb) = (mate(&mut p, assembly, MateKind::Revolute, g1, ca), mate(&mut p, assembly, MateKind::Revolute, g2, cb));
    p.solve(assembly).unwrap();
    let r = p.assembly_mut(assembly).unwrap().add_relation(RelationKind::Gear { ratio: 2.0 }, ma, mb, false).unwrap();
    p.settle_relation(assembly, r).unwrap();
    // The first can only go 30 degrees, so through the gear the second can only go 60, however far it's asked to.
    p.assembly_mut(assembly).unwrap().mate_mut(ma).unwrap().limits = vec![MateLimit { dof: Dof::Angle, min: -30.0, max: 30.0 }];
    let solved = p.set_mate_value(assembly, mb, Dof::Angle, 90.0).unwrap();
    assert!(solved.failing_relations.is_empty(), "{:?}", solved.failing_relations);
    let (va, vb) = (value(&solved, ma, Dof::Angle), value(&solved, mb, Dof::Angle));
    assert!((vb - 60.0).abs() < 0.01, "the second got to {vb}");
    assert!((vb - 2.0 * va).abs() < 1e-4, "{va} and {vb}");
}

#[test]
fn a_mate_failing_elsewhere_doesnt_stop_one_being_driven() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("100", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c, d) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"), instance(&mut p, assembly, studio, parts[3], "D"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    // C and D are both fixed, apart, with a mate that can't be met between them.
    asm.instance_mut(c).unwrap().fixed = true;
    asm.instance_mut(d).unwrap().fixed = true;
    asm.instance_mut(d).unwrap().at = Frame::translation(v3(500.0, 0.0, 0.0));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (cc, cd) = (at_point(&mut p, assembly, c, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, d, Vec3::ZERO, Vec3::Z, Vec3::X));
    let m = mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    let bad = mate(&mut p, assembly, MateKind::Fastened, cc, cd);
    let solved = p.set_mate_value(assembly, m, Dof::Angle, 90.0).unwrap();
    assert!((value(&solved, m, Dof::Angle) - 90.0).abs() < 1e-5, "{}", value(&solved, m, Dof::Angle));
    assert_eq!(solved.failing.iter().map(|f| f.0).collect::<Vec<_>>(), vec![bad]);
}

#[test]
fn a_part_dragged_stays_on_its_mate() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("100", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "Base"), instance(&mut p, assembly, studio, parts[1], "Arm"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let m = mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    p.solve(assembly).unwrap();
    // The arm's tip, toward a point up the Y axis. It can get there.
    let solved = p.drag_point(assembly, &[b], v3(100.0, 0.0, 0.0), v3(0.0, 100.0, 0.0)).unwrap();
    close(pose(&solved, b).to_world(v3(100.0, 0.0, 0.0)), v3(0.0, 100.0, 0.0));
    assert!(solved.remaining.unwrap() < 1e-6);
    assert!((value(&solved, m, Dof::Angle) - 90.0).abs() < 1e-5);
    // Further than it reaches, it goes as near as it can.
    let solved = p.drag_point(assembly, &[b], v3(100.0, 0.0, 0.0), v3(0.0, 400.0, 0.0)).unwrap();
    close(pose(&solved, b).to_world(v3(100.0, 0.0, 0.0)), v3(0.0, 100.0, 0.0));
    assert!((solved.remaining.unwrap() - 300.0).abs() < 1e-5);
    // A fixed one doesn't go at all.
    let solved = p.drag_point(assembly, &[a], Vec3::ZERO, v3(5.0, 5.0, 5.0)).unwrap();
    close(pose(&solved, a).origin, Vec3::ZERO);
    assert!(p.drag_point(assembly, &[999], Vec3::ZERO, Vec3::ZERO).is_err());
}

#[test]
fn a_part_dragged_to_a_whole_new_place_gets_there_if_nothings_stopping_it() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10")]);
    let a = instance(&mut p, assembly, studio, parts[0], "A");
    let to = Frame::new(v3(5.0, 6.0, 7.0), Vec3::X, Vec3::Y);
    let solved = p.drag_pose(assembly, &[a], to).unwrap();
    close(pose(&solved, a).origin, to.origin);
    close(pose(&solved, a).z, to.z);
    close(pose(&solved, a).x, to.x);
}

#[test]
fn something_small_can_be_dragged_a_long_way_in_one_go() {
    // A millimetre across, with nothing holding it, and ten metres to go.
    let (mut p, studio, assembly, parts) = project(vec![block("1", "1", "1")]);
    let a = instance(&mut p, assembly, studio, parts[0], "A");
    let solved = p.drag_point(assembly, &[a], Vec3::ZERO, v3(10000.0, 0.0, 0.0)).unwrap();
    close(pose(&solved, a).origin, v3(10000.0, 0.0, 0.0));
}

#[test]
fn a_large_part_moves_in_steps_of_its_own_size_even_if_its_mate_is_small() {
    // The connectors are at one point, so only the part itself says how big this is.
    let (mut p, studio, assembly, parts) = project(vec![block("1000", "1000", "1000"), block("1000", "1000", "1000")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::X, Vec3::Y), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::X, Vec3::Y));
    mate(&mut p, assembly, MateKind::Slider, ca, cb);
    p.solve(assembly).unwrap();
    let solved = p.drag_point(assembly, &[b], Vec3::ZERO, v3(20000.0, 0.0, 0.0)).unwrap();
    close(pose(&solved, b).origin, v3(20000.0, 0.0, 0.0));
}

#[test]
fn a_limit_is_in_the_units_it_says() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("100", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "Base"), instance(&mut p, assembly, studio, parts[1], "Arm"));
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let m = mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().limits = vec![MateLimit { dof: Dof::Angle, min: -30.0, max: 45.0 }];
    p.solve(assembly).unwrap();
    let solved = p.set_mate_value(assembly, m, Dof::Angle, 40.0).unwrap();
    assert!((value(&solved, m, Dof::Angle) - 40.0).abs() < 1e-5);
    // Asked for more than the limit, it goes as far as it can.
    let solved = p.set_mate_value(assembly, m, Dof::Angle, 80.0).unwrap();
    assert!(value(&solved, m, Dof::Angle) <= 45.0 + 1e-6 && value(&solved, m, Dof::Angle) > 44.99, "{}", value(&solved, m, Dof::Angle));
    // Dragging stops at it too.
    let solved = p.drag_point(assembly, &[b], v3(100.0, 0.0, 0.0), v3(0.0, -100.0, 0.0)).unwrap();
    assert!((value(&solved, m, Dof::Angle) + 30.0).abs() < 1e-5, "{}", value(&solved, m, Dof::Angle));
}

// ----- Groups and relations -----

#[test]
fn a_group_moves_as_one() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    asm.instance_mut(c).unwrap().at = Frame::translation(v3(30.0, 0.0, 0.0));
    asm.add_group(vec![vec![b], vec![c]]).unwrap();
    assert!(asm.add_group(vec![vec![b]]).is_err());
    assert!(asm.add_group(vec![vec![b], vec![77]]).is_err());
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, v3(0.0, 0.0, 0.0), Vec3::Z, Vec3::X));
    let m = mate(&mut p, assembly, MateKind::Revolute, ca, cb);
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.freedom, 1);
    let solved = p.set_mate_value(assembly, m, Dof::Angle, 90.0).unwrap();
    close(pose(&solved, b).x, Vec3::Y);
    // C went along, keeping its place from B.
    close(pose(&solved, c).origin, v3(0.0, 30.0, 0.0));
    close(pose(&solved, c).x, Vec3::Y);
}

#[test]
fn a_gear_turns_by_its_ratio_and_adding_it_moves_nothing() {
    let (mut p, studio, assembly, parts) = project(vec![block("200", "10", "1"), block("10", "10", "10"), block("10", "10", "10")]);
    let (g, a, b) = (instance(&mut p, assembly, studio, parts[0], "Ground"), instance(&mut p, assembly, studio, parts[1], "A"), instance(&mut p, assembly, studio, parts[2], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(g).unwrap().fixed = true;
    let g1 = at_point(&mut p, assembly, g, Vec3::ZERO, Vec3::Z, Vec3::X);
    let g2 = at_point(&mut p, assembly, g, v3(100.0, 0.0, 0.0), Vec3::Z, Vec3::X);
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (ma, mb) = (mate(&mut p, assembly, MateKind::Revolute, g1, ca), mate(&mut p, assembly, MateKind::Revolute, g2, cb));
    p.solve(assembly).unwrap();
    p.set_mate_value(assembly, ma, Dof::Angle, 20.0).unwrap();
    p.set_mate_value(assembly, mb, Dof::Angle, 70.0).unwrap();
    let r = p.assembly_mut(assembly).unwrap().add_relation(RelationKind::Gear { ratio: -2.0 }, ma, mb, false).unwrap();
    p.settle_relation(assembly, r).unwrap();
    // Nothing jumps.
    let solved = p.solve(assembly).unwrap();
    assert!((value(&solved, ma, Dof::Angle) - 20.0).abs() < 1e-5 && (value(&solved, mb, Dof::Angle) - 70.0).abs() < 1e-5);
    assert_eq!(solved.freedom, 1);
    // Thereafter, -2 for each one.
    let solved = p.set_mate_value(assembly, ma, Dof::Angle, 50.0).unwrap();
    assert!((value(&solved, ma, Dof::Angle) - 50.0).abs() < 1e-4, "{}", value(&solved, ma, Dof::Angle));
    assert!((value(&solved, mb, Dof::Angle) - 10.0).abs() < 1e-4, "{}", value(&solved, mb, Dof::Angle));
    // Past a whole turn of the second, which is where a wrapped angle would go wrong.
    let solved = p.set_mate_value(assembly, ma, Dof::Angle, 500.0).unwrap();
    assert!((value(&solved, mb, Dof::Angle) - (70.0 - 2.0 * 480.0)).abs() < 1e-3, "{}", value(&solved, mb, Dof::Angle));
}

#[test]
fn a_rack_and_a_screw_go_as_far_per_turn_as_they_say() {
    let (mut p, studio, assembly, parts) = project(vec![block("200", "10", "1"), block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (g, a, b, c) = (instance(&mut p, assembly, studio, parts[0], "Ground"), instance(&mut p, assembly, studio, parts[1], "Pinion"), instance(&mut p, assembly, studio, parts[2], "Rack"), instance(&mut p, assembly, studio, parts[3], "Nut"));
    p.assembly_mut(assembly).unwrap().instance_mut(g).unwrap().fixed = true;
    let g1 = at_point(&mut p, assembly, g, Vec3::ZERO, Vec3::Z, Vec3::X);
    let g2 = at_point(&mut p, assembly, g, v3(50.0, 0.0, 0.0), Vec3::X, Vec3::Y);
    let g3 = at_point(&mut p, assembly, g, v3(150.0, 0.0, 0.0), Vec3::Z, Vec3::X);
    let (ca, cb, cc) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::X, Vec3::Y), at_point(&mut p, assembly, c, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (ma, mb, mc) = (mate(&mut p, assembly, MateKind::Revolute, g1, ca), mate(&mut p, assembly, MateKind::Slider, g2, cb), mate(&mut p, assembly, MateKind::Cylindrical, g3, cc));
    p.solve(assembly).unwrap();
    let asm = p.assembly_mut(assembly).unwrap();
    let rack = asm.add_relation(RelationKind::RackAndPinion { distance: 40.0 }, ma, mb, false).unwrap();
    let screw = asm.add_relation(RelationKind::Screw { lead: 2.0 }, mc, mc, false).unwrap();
    p.settle_relation(assembly, rack).unwrap();
    p.settle_relation(assembly, screw).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.freedom, 2);
    let (rack_start, nut_start) = (value(&solved, mb, Dof::Z), value(&solved, mc, Dof::Z));
    let solved = p.set_mate_value(assembly, ma, Dof::Angle, 720.0).unwrap();
    assert!((value(&solved, mb, Dof::Z) - rack_start - 80.0).abs() < 1e-4, "{}", value(&solved, mb, Dof::Z));
    let solved = p.set_mate_value(assembly, mc, Dof::Angle, 1080.0).unwrap();
    assert!((value(&solved, mc, Dof::Z) - nut_start - 6.0).abs() < 1e-4, "{}", value(&solved, mc, Dof::Z));
    // What each needs of its mates is checked.
    let asm = p.assembly_mut(assembly).unwrap();
    assert!(asm.add_relation(RelationKind::Gear { ratio: 1.0 }, ma, mb, false).is_err());
    assert!(asm.add_relation(RelationKind::Linear { ratio: 1.0 }, ma, mb, false).is_err());
    assert!(asm.add_relation(RelationKind::RackAndPinion { distance: 1.0 }, mb, ma, false).is_err());
    assert!(asm.add_relation(RelationKind::Screw { lead: 1.0 }, ma, ma, false).is_err());
    assert!(asm.add_relation(RelationKind::Gear { ratio: 1.0 }, ma, 999, false).is_err());
}

#[test]
fn a_linear_relation_ties_two_slides_and_can_go_the_other_way() {
    let (mut p, studio, assembly, parts) = project(vec![block("200", "10", "1"), block("10", "10", "10"), block("10", "10", "10")]);
    let (g, a, b) = (instance(&mut p, assembly, studio, parts[0], "Ground"), instance(&mut p, assembly, studio, parts[1], "A"), instance(&mut p, assembly, studio, parts[2], "B"));
    p.assembly_mut(assembly).unwrap().instance_mut(g).unwrap().fixed = true;
    let g1 = at_point(&mut p, assembly, g, Vec3::ZERO, Vec3::X, Vec3::Y);
    let g2 = at_point(&mut p, assembly, g, v3(0.0, 50.0, 0.0), Vec3::X, Vec3::Y);
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::X, Vec3::Y), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::X, Vec3::Y));
    let (ma, mb) = (mate(&mut p, assembly, MateKind::Slider, g1, ca), mate(&mut p, assembly, MateKind::Slider, g2, cb));
    p.solve(assembly).unwrap();
    let r = p.assembly_mut(assembly).unwrap().add_relation(RelationKind::Linear { ratio: 3.0 }, ma, mb, true).unwrap();
    p.settle_relation(assembly, r).unwrap();
    let solved = p.set_mate_value(assembly, ma, Dof::Z, 10.0).unwrap();
    assert!((value(&solved, mb, Dof::Z) + 30.0).abs() < 1e-5, "{}", value(&solved, mb, Dof::Z));
}

// ----- Sub-assemblies -----

/// A tab holding a base and an arm on a revolute, and a top assembly with an instance of it.
fn nested() -> (Project, TabId, u32, u32, u32, BodyId) {
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let doc = p.studio_mut(studio).unwrap();
    let (base, arm) = (BodyId { feature: doc.with(block("20", "20", "20")), index: 0 }, BodyId { feature: doc.with(block("100", "10", "10")), index: 0 });
    let inner = p.add_assembly("Arm");
    let (ib, ia) = (instance(&mut p, inner, studio, base, "Base"), instance(&mut p, inner, studio, arm, "Arm"));
    p.assembly_mut(inner).unwrap().instance_mut(ib).unwrap().fixed = true;
    let (c1, c2) = (at_point(&mut p, inner, ib, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, inner, ia, Vec3::ZERO, Vec3::Z, Vec3::X));
    mate(&mut p, inner, MateKind::Revolute, c1, c2);
    let top = p.add_assembly("Top");
    let sub = p.assembly_mut(top).unwrap().add_instance(Source::Assembly { tab: inner }, "Arm <1>".into(), Frame::translation(v3(500.0, 0.0, 0.0)));
    (p, top, sub, ib, ia, base)
}

fn at_path(solved: &Solved, path: &[u32]) -> Frame {
    solved.placed.iter().find(|q| q.path == path).unwrap().at
}

#[test]
fn a_sub_assemblys_mates_still_work_above_it() {
    let (mut p, top, sub, ib, ia, _) = nested();
    let solved = p.solve(top).unwrap();
    assert_eq!(solved.placed.len(), 2);
    close(at_path(&solved, &[sub, ib]).origin, v3(500.0, 0.0, 0.0));
    // Six for the base, which isn't fixed up here though it is in its own tab, and one for the arm.
    assert_eq!(solved.freedom, 7);
    // Dragging the arm swings it, keeping to its mate, and the base can go along if it's easier.
    let solved = p.drag_point(top, &[sub, ia], v3(100.0, 0.0, 0.0), v3(500.0, 100.0, 0.0)).unwrap();
    let (b, a) = (at_path(&solved, &[sub, ib]), at_path(&solved, &[sub, ia]));
    close(a.to_world(v3(100.0, 0.0, 0.0)), v3(500.0, 100.0, 0.0));
    close(a.origin, b.origin);
    assert!(a.z.dist(b.z) < 1e-6);
    // Fixed up here too, only the arm is left, and it's on its mate.
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().fixed = true;
    let solved = p.solve(top).unwrap();
    assert_eq!(solved.freedom, 0, "both of its parts are held once the instance is");
}

#[test]
fn moving_a_sub_assembly_takes_its_moved_parts_along() {
    let (mut p, top, sub, ib, ia, _) = nested();
    let solved = p.drag_point(top, &[sub, ia], v3(100.0, 0.0, 0.0), v3(500.0, 100.0, 0.0)).unwrap();
    let (was_base, was_arm) = (at_path(&solved, &[sub, ib]), at_path(&solved, &[sub, ia]));
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().at = Frame::translation(v3(800.0, 0.0, 0.0));
    let solved = p.solve(top).unwrap();
    let by = v3(300.0, 0.0, 0.0);
    close(at_path(&solved, &[sub, ib]).origin, was_base.origin + by);
    close(at_path(&solved, &[sub, ia]).origin, was_arm.origin + by);
    close(at_path(&solved, &[sub, ia]).x, was_arm.x);
    // And that's in the file.
    let saved = p.save_text();
    let mut back = Project::load_text(&saved).unwrap();
    let again = back.solve(top).unwrap();
    close(at_path(&again, &[sub, ia]).x, was_arm.x);
    close(at_path(&again, &[sub, ia]).origin, was_arm.origin + by);
}

#[test]
fn a_locked_sub_assembly_moves_as_one_and_is_as_its_own_tab_has_it() {
    let (mut p, top, sub, ib, ia, _) = nested();
    // Swing the arm here first, so there's something to forget.
    p.drag_point(top, &[sub, ia], v3(100.0, 0.0, 0.0), v3(500.0, 100.0, 0.0)).unwrap();
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().locked = true;
    let solved = p.solve(top).unwrap();
    // As in its own tab, where the arm hasn't moved, at the instance's place.
    close(at_path(&solved, &[sub, ia]).x, Vec3::X);
    close(at_path(&solved, &[sub, ia]).origin, v3(500.0, 0.0, 0.0));
    assert_eq!(solved.freedom, 6);
    // Dragged, it goes together, and the instance is what remembers it.
    let solved = p.drag_pose(top, &[sub, ib], Frame::new(v3(0.0, 300.0, 0.0), Vec3::X, Vec3::Y)).unwrap();
    close(at_path(&solved, &[sub, ib]).origin, v3(0.0, 300.0, 0.0));
    close(at_path(&solved, &[sub, ia]).origin, v3(0.0, 300.0, 0.0));
    close(at_path(&solved, &[sub, ia]).z, Vec3::X);
    close(p.assembly(top).unwrap().instance(sub).unwrap().at.origin, v3(0.0, 300.0, 0.0));
    close(p.assembly(top).unwrap().instance(sub).unwrap().at.z, Vec3::X);
}

#[test]
fn a_locked_sub_assembly_dragged_by_any_part_keeps_its_shape() {
    let (mut p, top, sub, ib, ia, _) = nested();
    let inner = p.tabs[1].id;
    // Its arm is at a quarter turn in its own tab, and its base off to one side, so no part is at the instance's place.
    p.assembly_mut(inner).unwrap().instance_mut(ib).unwrap().at = Frame::translation(v3(7.0, 8.0, 9.0));
    let mate = p.assembly(inner).unwrap().mates[0].id;
    p.set_mate_value(inner, mate, Dof::Angle, 90.0).unwrap();
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().locked = true;
    let solved = p.solve(top).unwrap();
    close(at_path(&solved, &[sub, ia]).x, Vec3::Y);
    // Dragged by that arm, the instance itself ends up moved by the same motion.
    let to = Frame::new(v3(100.0, 200.0, 300.0), Vec3::Z, Vec3::X);
    let solved = p.drag_pose(top, &[sub, ia], to).unwrap();
    close(at_path(&solved, &[sub, ia]).origin, to.origin);
    close(at_path(&solved, &[sub, ia]).x, Vec3::X);
    let place = p.assembly(top).unwrap().instance(sub).unwrap().at;
    close(place.origin, v3(92.0, 207.0, 291.0));
    close(place.x, -Vec3::Y);
    // And it reads the same from the instance alone.
    let again = p.solve(top).unwrap();
    close(at_path(&again, &[sub, ia]).origin, to.origin);
    close(at_path(&again, &[sub, ia]).x, Vec3::X);
}

#[test]
fn a_locked_sub_assembly_is_as_its_tab_solves_even_if_nobody_has_solved_it_lately() {
    let (mut p, top, sub, ib, ia, _) = nested();
    let inner = p.tabs[1].id;
    // Its arm is nowhere near its mate, and its tab hasn't been looked at since.
    p.assembly_mut(inner).unwrap().instance_mut(ia).unwrap().at = Frame::translation(v3(70.0, 80.0, 90.0));
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().locked = true;
    let solved = p.solve(top).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    close(at_path(&solved, &[sub, ia]).origin, at_path(&solved, &[sub, ib]).origin);
}

#[test]
fn a_mate_that_fails_inside_a_locked_sub_assembly_is_its_own_tabs_business() {
    let (mut p, top, sub, ib, ia, _) = nested();
    let inner = p.tabs[1].id;
    // Two fixed parts, and a mate between them that can't be met.
    let asm = p.assembly_mut(inner).unwrap();
    asm.instance_mut(ia).unwrap().fixed = true;
    asm.instance_mut(ia).unwrap().at = Frame::translation(v3(70.0, 80.0, 90.0));
    assert_eq!(asm.instance(ib).unwrap().fixed, true);
    let solved = p.solve(inner).unwrap();
    assert_eq!(solved.failing.len(), 1);
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().locked = true;
    let solved = p.solve(top).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    close(at_path(&solved, &[sub, ia]).origin, v3(570.0, 80.0, 90.0));
}

#[test]
fn a_mate_can_reach_into_a_sub_assembly() {
    let (mut p, top, sub, ib, _, base) = nested();
    let studio = p.tabs[0].id;
    let other = instance(&mut p, top, studio, base, "Other");
    p.assembly_mut(top).unwrap().instance_mut(other).unwrap().fixed = true;
    let asm = p.assembly_mut(top).unwrap();
    let c1 = asm.add_connector(vec![other], end_of(base)).unwrap();
    let c2 = asm.add_connector(vec![sub, ib], start_of(base)).unwrap();
    let m = asm.add_mate(MateKind::Fastened, c1, c2).unwrap();
    asm.mate_mut(m).unwrap().flip = true;
    let solved = p.solve(top).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    // The inner base sits on the top of the other.
    close(at_path(&solved, &[sub, ib]).origin, v3(0.0, 0.0, 20.0));
    // A mate needs two ends that aren't the same.
    let asm = p.assembly_mut(top).unwrap();
    let c3 = asm.add_connector(vec![sub, ib], end_of(base)).unwrap();
    assert!(asm.add_mate(MateKind::Fastened, c2, c3).is_err());
    assert!(asm.add_mate(MateKind::Fastened, c2, 999).is_err());
    assert!(asm.add_connector(vec![999], end_of(base)).is_err());
}

#[test]
fn a_nested_locked_assembly_keeps_its_shape_two_levels_up() {
    let (mut p, inner_top, sub, _, ia, _) = nested();
    // The assembly with the instance in it becomes a sub-assembly itself, and its instance is locked there.
    p.assembly_mut(inner_top).unwrap().instance_mut(sub).unwrap().locked = true;
    let outer = p.add_assembly("Outer");
    let sub2 = p.assembly_mut(outer).unwrap().add_instance(Source::Assembly { tab: inner_top }, "Top <1>".into(), Frame::translation(v3(0.0, 1000.0, 0.0)));
    let solved = p.solve(outer).unwrap();
    assert_eq!(solved.placed.len(), 2);
    assert_eq!(solved.freedom, 6);
    let path = [sub2, sub, ia];
    close(at_path(&solved, &path).origin, v3(500.0, 1000.0, 0.0));
    // Moved as one, it's saved for the locked instance, by its path, and not for each part.
    let solved = p.drag_pose(outer, &path, Frame::new(v3(500.0, 2000.0, 0.0), Vec3::Z, Vec3::X)).unwrap();
    close(at_path(&solved, &path).origin, v3(500.0, 2000.0, 0.0));
    let moved = &p.assembly(outer).unwrap().moved;
    assert!(moved.contains_key(&vec![sub2, sub]), "{:?}", moved.keys());
    assert_eq!(moved.len(), 1);
    close(p.solve(outer).unwrap().placed[1].at.origin, v3(500.0, 2000.0, 0.0));
}

/// A sub-assembly holding two parts on revolutes in a frame, geared together, held still at the top by its frame.
fn geared_sub_assembly(ratio: f64) -> (Project, TabId, u32, u32, u32, u32) {
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let doc = p.studio_mut(studio).unwrap();
    let frame = BodyId { feature: doc.with(block("200", "10", "1")), index: 0 };
    let arm = BodyId { feature: doc.with(block("30", "10", "10")), index: 0 };
    let inner = p.add_assembly("Gears");
    let (g, a, b) = (instance(&mut p, inner, studio, frame, "Frame"), instance(&mut p, inner, studio, arm, "A"), instance(&mut p, inner, studio, arm, "B"));
    p.assembly_mut(inner).unwrap().instance_mut(g).unwrap().fixed = true;
    p.assembly_mut(inner).unwrap().instance_mut(b).unwrap().at = Frame::translation(v3(100.0, 0.0, 0.0));
    let (g1, g2) = (at_point(&mut p, inner, g, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, inner, g, v3(100.0, 0.0, 0.0), Vec3::Z, Vec3::X));
    let (ca, cb) = (at_point(&mut p, inner, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, inner, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    let (ma, mb) = (mate(&mut p, inner, MateKind::Revolute, g1, ca), mate(&mut p, inner, MateKind::Revolute, g2, cb));
    p.solve(inner).unwrap();
    let r = p.assembly_mut(inner).unwrap().add_relation(RelationKind::Gear { ratio }, ma, mb, false).unwrap();
    p.settle_relation(inner, r).unwrap();
    assert_eq!(p.solve(inner).unwrap().freedom, 1);
    // Up here its frame isn't fixed, so a fixed base holds it.
    let top = p.add_assembly("Top");
    let base = instance(&mut p, top, studio, frame, "Base");
    p.assembly_mut(top).unwrap().instance_mut(base).unwrap().fixed = true;
    let sub = p.assembly_mut(top).unwrap().add_instance(Source::Assembly { tab: inner }, "Gears <1>".into(), Frame::WORLD);
    let c1 = at_point(&mut p, top, base, Vec3::ZERO, Vec3::Z, Vec3::X);
    let c2 = p.assembly_mut(top).unwrap().add_connector(vec![sub, g], Anchor::Frame(Frame::WORLD)).unwrap();
    mate(&mut p, top, MateKind::Fastened, c1, c2);
    (p, top, sub, g, a, b)
}

/// How far a part has turned about Z, from X, between -180 and 180.
fn turned_to(f: &Frame) -> f64 {
    f.x.y.atan2(f.x.x).to_degrees()
}

fn same_angle(a: f64, b: f64) -> bool {
    let d = (a - b).rem_euclid(360.0);
    d.min(360.0 - d) < 1e-3
}

#[test]
fn a_gear_in_a_sub_assembly_holds_from_above_past_a_whole_turn() {
    let (mut p, top, sub, _, a, b) = geared_sub_assembly(2.0);
    let solved = p.solve(top).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    assert_eq!(solved.freedom, 1, "the gear leaves one between them");
    // Round by hand, a little at a time, for a turn and a half of A, so three of B.
    for k in 1..=18 {
        let angle = k as f64 * 30.0;
        let to = v3(30.0 * angle.to_radians().cos(), 30.0 * angle.to_radians().sin(), 0.0);
        let solved = p.drag_point(top, &[sub, a], v3(30.0, 0.0, 0.0), to).unwrap();
        assert!(solved.failing.is_empty() && solved.failing_relations.is_empty(), "at {angle}: {:?} {:?}", solved.failing, solved.failing_relations);
        assert!(solved.remaining.unwrap() < 1e-5, "at {angle}: {:?}", solved.remaining);
        let (ta, tb) = (turned_to(&at_path(&solved, &[sub, a])), turned_to(&at_path(&solved, &[sub, b])));
        assert!(same_angle(ta, angle), "at {angle}, A is at {ta}");
        assert!(same_angle(tb, 2.0 * angle), "at {angle}, B is at {tb}");
        // And a solve on its own leaves it there, as it's the same state.
        let again = p.solve(top).unwrap();
        assert!(same_angle(turned_to(&at_path(&again, &[sub, b])), 2.0 * angle), "at {angle}, solved again");
    }
    // It's all in the file: where it's got to, and how many turns that is.
    let mut back = Project::load_text(&p.save_text()).unwrap();
    let solved = back.solve(top).unwrap();
    assert!(solved.failing.is_empty() && solved.failing_relations.is_empty());
    assert!(same_angle(turned_to(&at_path(&solved, &[sub, b])), 2.0 * 540.0));
    let to = v3(30.0 * 570f64.to_radians().cos(), 30.0 * 570f64.to_radians().sin(), 0.0);
    let solved = back.drag_point(top, &[sub, a], v3(30.0, 0.0, 0.0), to).unwrap();
    assert!(same_angle(turned_to(&at_path(&solved, &[sub, b])), 2.0 * 570.0));
}

#[test]
fn deleting_a_sub_assembly_forgets_where_its_mates_had_got_to() {
    let (mut p, top, sub, _, a, _) = geared_sub_assembly(2.0);
    p.drag_point(top, &[sub, a], v3(30.0, 0.0, 0.0), v3(0.0, 30.0, 0.0)).unwrap();
    assert!(!p.assembly(top).unwrap().moved_turns.is_empty());
    let asm = p.assembly_mut(top).unwrap();
    asm.delete_instance(sub).unwrap();
    assert!(asm.moved.is_empty() && asm.moved_turns.is_empty());
}

#[test]
fn a_locked_sub_assembly_with_a_gear_in_it_moves_as_one() {
    let (mut p, top, sub, _, a, b) = geared_sub_assembly(2.0);
    p.drag_point(top, &[sub, a], v3(30.0, 0.0, 0.0), v3(0.0, 30.0, 0.0)).unwrap();
    p.assembly_mut(top).unwrap().instance_mut(sub).unwrap().locked = true;
    // As its own tab has it, where neither has been turned.
    let solved = p.solve(top).unwrap();
    assert!(solved.failing.is_empty() && solved.failing_relations.is_empty());
    assert!(same_angle(turned_to(&at_path(&solved, &[sub, a])), 0.0));
    assert!(same_angle(turned_to(&at_path(&solved, &[sub, b])), 0.0));
    assert_eq!(solved.freedom, 0);
}

#[test]
fn an_assembly_cant_end_up_inside_itself() {
    let mut p = Project::new();
    let (a, b) = (p.add_assembly("A"), p.add_assembly("B"));
    p.assembly_mut(a).unwrap().add_instance(Source::Assembly { tab: b }, "B".into(), Frame::WORLD);
    p.assembly_mut(b).unwrap().add_instance(Source::Assembly { tab: a }, "A".into(), Frame::WORLD);
    let why = p.solve(a).unwrap_err().to_string();
    assert!(why.contains("inside itself"), "{why}");
}

// ----- Deleting -----

#[test]
fn deleting_an_instance_takes_what_depends_on_it() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b, c) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"), instance(&mut p, assembly, studio, parts[2], "C"));
    let asm = p.assembly_mut(assembly).unwrap();
    let (ca, cb) = (asm.add_connector(vec![a], end_of(parts[0])).unwrap(), asm.add_connector(vec![b], end_of(parts[1])).unwrap());
    let (cc, cd) = (asm.add_connector(vec![b], end_of(parts[1])).unwrap(), asm.add_connector(vec![c], end_of(parts[2])).unwrap());
    let (m1, m2) = (asm.add_mate(MateKind::Revolute, ca, cb).unwrap(), asm.add_mate(MateKind::Revolute, cc, cd).unwrap());
    let r = asm.add_relation(RelationKind::Gear { ratio: 1.0 }, m1, m2, false).unwrap();
    asm.add_group(vec![vec![a], vec![b]]).unwrap();
    asm.delete_instance(a).unwrap();
    assert!(asm.instance(a).is_none());
    assert!(asm.connector(ca).is_none());
    assert!(asm.mate(m1).is_none());
    assert!(asm.mate(m2).is_some());
    assert!(asm.relations.iter().all(|x| x.id != r));
    assert!(asm.groups.is_empty());
    assert!(asm.delete_instance(a).is_err());
    // What's left still solves.
    assert!(p.solve(assembly).unwrap().failing.is_empty());
}

#[test]
fn an_implicit_connector_goes_with_its_mate_and_an_explicit_one_stays() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    let asm = p.assembly_mut(assembly).unwrap();
    let (ca, cb) = (asm.add_connector(vec![a], end_of(parts[0])).unwrap(), asm.add_connector(vec![b], end_of(parts[1])).unwrap());
    asm.connector_mut(cb).unwrap().implicit = true;
    let m = asm.add_mate(MateKind::Fastened, ca, cb).unwrap();
    asm.delete_mate(m).unwrap();
    assert!(asm.connector(ca).is_some());
    assert!(asm.connector(cb).is_none());
    assert!(asm.delete_mate(m).is_err());
    let _ = studio;
}

#[test]
fn a_tab_in_use_cant_be_deleted() {
    let (mut p, top, _, _, _, _) = nested();
    let studio = p.tabs[0].id;
    let inner = p.tabs[1].id;
    assert!(p.delete_tab(studio).is_err());
    assert!(p.delete_tab(inner).is_err());
    p.delete_tab(top).unwrap();
    p.delete_tab(inner).unwrap();
    // The last one has to stay.
    assert!(p.delete_tab(studio).is_err());
    assert!(p.delete_tab(TabId(99)).is_err());
}

#[test]
fn asking_for_the_wrong_kind_of_tab_says_so() {
    let (mut p, studio, assembly, _) = project(vec![block("1", "1", "1")]);
    assert!(p.assembly(studio).is_err());
    assert!(p.studio(assembly).is_err());
    assert!(p.solve(studio).is_err());
    // An instance pointing at an assembly as its Part Studio.
    p.assembly_mut(assembly).unwrap().add_instance(Source::Part { studio: assembly, body: BodyId { feature: FeatureId(1), index: 0 } }, "X".into(), Frame::WORLD);
    assert!(p.solve(assembly).unwrap_err().to_string().contains("isn't a Part Studio"));
}

// ----- Interference and BOM -----

#[test]
fn interference_is_the_volume_they_share() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10"), block("10", "10", "10")]);
    let ids: Vec<u32> = parts.iter().map(|b| instance(&mut p, assembly, studio, *b, "Block")).collect();
    let asm = p.assembly_mut(assembly).unwrap();
    // Overlaps the first by 5 x 10 x 10.
    asm.instance_mut(ids[1]).unwrap().at = Frame::translation(v3(5.0, 0.0, 0.0));
    // Touches the second, and doesn't overlap the first.
    asm.instance_mut(ids[2]).unwrap().at = Frame::translation(v3(15.0, 0.0, 0.0));
    // Nowhere near.
    asm.instance_mut(ids[3]).unwrap().at = Frame::translation(v3(500.0, 0.0, 0.0));
    let (found, unknown) = p.interferences(assembly).unwrap();
    assert!(unknown.is_empty(), "{unknown:?}");
    assert_eq!(found.len(), 1, "{found:?}");
    assert_eq!((found[0].a[0], found[0].b[0]), (ids[0], ids[1]));
    assert!((found[0].volume - 500.0).abs() < 1e-6, "{}", found[0].volume);
}

#[test]
fn a_bom_counts_each_part_however_it_is_used() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("2", "3", "4")]);
    for k in 0..3 {
        let i = instance(&mut p, assembly, studio, parts[0], &format!("Plate <{}>", k + 1));
        p.assembly_mut(assembly).unwrap().instance_mut(i).unwrap().at = Frame::translation(v3(k as f64 * 20.0, 0.0, 0.0));
    }
    instance(&mut p, assembly, studio, parts[1], "Key <1>");
    let bom = p.bom(assembly).unwrap();
    assert_eq!(bom.rows.len(), 2);
    assert_eq!((bom.rows[0].name.as_str(), bom.rows[0].count), ("Plate", 3));
    assert_eq!((bom.rows[1].name.as_str(), bom.rows[1].count), ("Key", 1));
    assert!((bom.rows[0].volume - 1000.0).abs() < 1e-6);
    assert!((bom.rows[1].area - 2.0 * (6.0 + 8.0 + 12.0)).abs() < 1e-6);
}

// ----- Saving -----

#[test]
fn a_whole_project_comes_back_from_its_file() {
    let mut s = slider_crank();
    let asm = s.p.assembly_mut(s.assembly).unwrap();
    asm.mate_mut(s.crank_joint).unwrap().limits = vec![MateLimit { dof: Dof::Angle, min: -720.0, max: 720.0 }];
    asm.app = Some(coreshape_document::json::parse("{\"view\":[1,2,3]}").unwrap());
    let g = asm.add_group(vec![vec![asm.instances[1].id], vec![asm.instances[2].id]]).unwrap();
    asm.groups.iter_mut().find(|x| x.id == g).unwrap().suppressed = true;
    let text = s.p.save_text();
    assert!(text.contains("coreshape-project"));
    let back = Project::load_text(&text).unwrap();
    assert_eq!(back.tabs.len(), 2);
    assert_eq!(back.assembly(s.assembly).unwrap(), s.p.assembly(s.assembly).unwrap());
    // And its Part Studio is whole.
    let (mut a, mut b) = (s.p.clone(), back);
    let (oa, ob) = (a.studio_mut(a.tabs[0].id).unwrap().rebuild(None), b.studio_mut(b.tabs[0].id).unwrap().rebuild(None));
    assert_eq!(oa.bodies.len(), ob.bodies.len());
    // It says the same again when saved again.
    assert_eq!(b.save_text(), text);
    // New ids don't reuse old ones.
    let top = b.assembly(s.assembly).unwrap().next_id;
    assert!(top > b.assembly(s.assembly).unwrap().instances.iter().map(|i| i.id).max().unwrap());
    assert_eq!(b.add_studio("Another").0 as usize, 3);
}

#[test]
fn every_kind_of_connector_and_mate_comes_back() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A \"quoted\""), instance(&mut p, assembly, studio, parts[1], "B"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(a).unwrap().fixed = true;
    asm.instance_mut(b).unwrap().suppressed = true;
    let k = |t: &str| FaceKey::parse(t).unwrap();
    let anchors = vec![
        Anchor::Frame(Frame::new(v3(1.0, 2.0, 3.0), Vec3::X, Vec3::Z)),
        Anchor::Face { key: k("f1:end"), point: FacePoint::Centroid },
        Anchor::Face { key: k("f1:side.0"), point: FacePoint::AxisMiddle },
        Anchor::Face { key: k("f1:side.0"), point: FacePoint::AxisStart },
        Anchor::Face { key: k("f1:side.0"), point: FacePoint::AxisEnd },
        Anchor::Face { key: k("f1:side.0"), point: FacePoint::Apex },
        Anchor::Face { key: k("f1:end@f5.1#2"), point: FacePoint::Near(v3(1.0, 1.0, 1.0)) },
        Anchor::Edge { faces: [k("f1:end"), k("f1:side.0")], point: EdgePoint::Middle },
        Anchor::Edge { faces: [k("f1:end"), k("f1:side.0")], point: EdgePoint::Start },
        Anchor::Edge { faces: [k("f1:end"), k("f1:side.0")], point: EdgePoint::End },
        Anchor::Edge { faces: [k("f1:end"), k("f1:side.0")], point: EdgePoint::Center },
    ];
    let mut connectors = Vec::new();
    for anchor in anchors {
        let c = asm.add_connector(vec![a], anchor).unwrap();
        let entry = asm.connector_mut(c).unwrap();
        entry.offset = [1.0, -2.0, 3.5];
        entry.rotation = 12.5;
        entry.flip = true;
        entry.quarter_turns = 3;
        entry.implicit = c % 2 == 0;
        connectors.push(c);
    }
    let kinds = [MateKind::Fastened, MateKind::Revolute, MateKind::Slider, MateKind::Cylindrical, MateKind::PinSlot, MateKind::Planar, MateKind::Ball, MateKind::Parallel];
    let other = asm.add_connector(vec![b], Anchor::Frame(Frame::WORLD)).unwrap();
    let mut mates = Vec::new();
    for (i, kind) in kinds.iter().enumerate() {
        let m = asm.add_mate(*kind, connectors[i], other).unwrap();
        let entry = asm.mate_mut(m).unwrap();
        entry.offset = [0.5, 0.25, 0.125];
        entry.rotation = 33.0;
        entry.flip = true;
        entry.quarter_turns = 2;
        entry.limits = vec![MateLimit { dof: Dof::Z, min: -1.0, max: 1.0 }];
        entry.suppressed = true;
        entry.turned = 7.5;
        mates.push(m);
    }
    let kinds = [RelationKind::Gear { ratio: 2.0 }, RelationKind::RackAndPinion { distance: 3.0 }, RelationKind::Linear { ratio: -4.0 }];
    asm.relations.push(Relation { id: 900, kind: kinds[0], a: mates[1], b: mates[1], reverse: true, offset: 1.5, suppressed: true });
    asm.relations.push(Relation { id: 901, kind: kinds[1], a: mates[1], b: mates[2], reverse: false, offset: 0.0, suppressed: false });
    asm.relations.push(Relation { id: 902, kind: kinds[2], a: mates[2], b: mates[2], reverse: false, offset: 0.0, suppressed: false });
    asm.relations.push(Relation { id: 903, kind: RelationKind::Screw { lead: 5.0 }, a: mates[3], b: mates[3], reverse: false, offset: 0.0, suppressed: false });
    asm.moved.insert(vec![b, 7], Frame::translation(v3(1.0, 0.0, 0.0)));
    asm.moved_turns.insert((vec![b], 7), -12.5);
    asm.next_id = 904;
    let want = asm.clone();
    let back = Project::load_text(&p.save_text()).unwrap();
    assert_eq!(*back.assembly(assembly).unwrap(), want);
}

#[test]
fn a_plain_document_is_a_project_of_one_and_stays_plain() {
    let mut doc = Document::new();
    doc.with(block("10", "20", "30"));
    let text = doc.save_text();
    let p = Project::load_text(&text).unwrap();
    assert_eq!(p.tabs.len(), 1);
    assert!(matches!(p.tabs[0].content, TabContent::PartStudio(_)));
    // Written back just as it was.
    assert_eq!(p.save_text(), text);
    assert!(!p.save_text().contains("coreshape-project"));
}

#[test]
fn a_file_that_doesnt_add_up_is_refused_and_says_why() {
    let (mut p, studio, assembly, parts) = project(vec![block("10", "10", "10"), block("10", "10", "10")]);
    let (a, b) = (instance(&mut p, assembly, studio, parts[0], "A"), instance(&mut p, assembly, studio, parts[1], "B"));
    let (ca, cb) = (at_point(&mut p, assembly, a, Vec3::ZERO, Vec3::Z, Vec3::X), at_point(&mut p, assembly, b, Vec3::ZERO, Vec3::Z, Vec3::X));
    mate(&mut p, assembly, MateKind::Fastened, ca, cb);
    let text = p.save_text();
    let refused = |from: &str, to: &str, says: &str| {
        let changed = text.replacen(from, to, 1);
        assert_ne!(changed, text, "{from} isn't in it");
        let why = Project::load_text(&changed).err().unwrap_or_else(|| panic!("{from} -> {to} was taken")).to_string();
        assert!(why.contains(says), "{why}");
    };
    refused("\"version\": 1", "\"version\": 99", "version");
    refused("\"type\": \"assembly\"", "\"type\": \"drawing\"", "drawing");
    refused("\"type\": \"fastened\"", "\"type\": \"welded\"", "welded");
    refused("\"instance\": [", "\"instance\": [99,", "99");
    refused("\"a\": 3", "\"a\": 77", "connector 77");
    refused("\"relations\": [", "\"gears\": 1, \"relations\": [", "gears");
    assert!(Project::load_text("not json").is_err());
    assert!(Project::load_text("{\"format\": \"coreshape-project\", \"version\": 1, \"tabs\": []}").is_err());
}

// ----- STEP -----

fn plate_with_two_pins() -> (Project, TabId, TabId, [u32; 3]) {
    let (mut p, studio, assembly, parts) = project(vec![block("40", "20", "5"), pin("3", "10")]);
    let plate = instance(&mut p, assembly, studio, parts[0], "Plate <1>");
    let (a, b) = (instance(&mut p, assembly, studio, parts[1], "Pin <1>"), instance(&mut p, assembly, studio, parts[1], "Pin <2>"));
    let asm = p.assembly_mut(assembly).unwrap();
    asm.instance_mut(plate).unwrap().fixed = true;
    asm.instance_mut(a).unwrap().at = Frame::translation(v3(10.0, 10.0, 5.0));
    asm.instance_mut(b).unwrap().at = Frame::new(v3(30.0, 10.0, 15.0), -Vec3::Z, Vec3::Y);
    (p, studio, assembly, [plate, a, b])
}

#[test]
fn an_assembly_comes_out_as_one_copy_of_each_part_and_a_use_of_each() {
    let (mut p, _, assembly, _) = plate_with_two_pins();
    let text = p.step_text(assembly, "Plate and pins").unwrap();
    // Two parts, three uses, and the geometry of each part once.
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP").count(), 2);
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE").count(), 3);
    let one_pin = cylinder(&Frame::WORLD, 3.0, 10.0, 2).unwrap();
    assert_eq!(text.matches("ADVANCED_FACE").count(), 6 + one_pin.faces.len());
    let structure = read_step_structure(&text).unwrap();
    assert_eq!(structure.names, vec!["Plate", "Pin"]);
    assert_eq!(structure.instances.len(), 3);
    assert_eq!(structure.instances.iter().filter(|x| x.0 == 1).count(), 2);
    // Read the plain way, every use is a body where it should be.
    let bodies = read_step(&text, 1).unwrap();
    assert_eq!(bodies.len(), 3);
    let total: f64 = bodies.iter().map(|b| mass_properties(b).volume).sum();
    let want = 40.0 * 20.0 * 5.0 + 2.0 * std::f64::consts::PI * 9.0 * 10.0;
    assert!((total - want).abs() < 1e-6, "{total}");
    // A pin the other way up is still in the same place, standing on the plate. Exactly: by its vertices.
    let high = |b: &Body| b.vertices.iter().fold(f64::MIN, |m, v| m.max(v.z));
    let low = |b: &Body| b.vertices.iter().fold(f64::MAX, |m, v| m.min(v.z));
    assert!((low(&bodies[2]) - 5.0).abs() < 1e-9 && (high(&bodies[2]) - 15.0).abs() < 1e-9);
    let centre = bodies[2].vertices[0];
    assert!((v3(centre.x - 30.0, centre.y - 10.0, 0.0).norm() - 3.0).abs() < 1e-9);
    // The parts on their own come back in their own coordinates.
    let parts = read_step_parts(&text, 1).unwrap();
    assert_eq!(parts.len(), 2);
    assert!((low(&parts[1])).abs() < 1e-9 && (high(&parts[1]) - 10.0).abs() < 1e-9);
    // And no two have a face of the same name.
    let (first, second) = (face_keys(&parts[0]), face_keys(&parts[1]));
    assert!(first.iter().all(|k| !second.contains(k)), "{first:?} and {second:?}");
    // The standard has a motion's first placement in the part and its second in the whole. Other readers go by that.
    let line_of = |id: &str| text.lines().find(|l| l.starts_with(&format!("{id}="))).unwrap_or_else(|| panic!("no {id}"));
    let whole = text.lines().find(|l| l.contains("=SHAPE_REPRESENTATION(")).unwrap();
    for motion in text.lines().filter(|l| l.contains("ITEM_DEFINED_TRANSFORMATION")) {
        let inner = motion.split("'','',").nth(1).unwrap().trim_end_matches(");");
        let (from, to) = inner.split_once(',').unwrap();
        assert!(whole.contains(&format!("{to},")) || whole.contains(&format!("{to})")), "{to} isn't in {whole}");
        assert!(!whole.contains(&format!("{from},")) && !whole.contains(&format!("{from})")), "{from} is in {whole}");
        assert!(line_of(from).contains("AXIS2_PLACEMENT_3D"));
    }
}

#[test]
fn a_step_assembly_comes_in_as_a_studio_and_an_assembly() {
    let (mut p, _, assembly, _) = plate_with_two_pins();
    let text = p.step_text(assembly, "Plate and pins").unwrap();
    let mut q = Project::new();
    let (studio, tab) = q.import_step(&text, "Imported").unwrap();
    assert_eq!(q.tabs.len(), 3);
    let o = q.studio_mut(studio).unwrap().rebuild(None);
    assert_eq!(o.bodies.len(), 2);
    let solved = q.solve(tab).unwrap();
    assert!(solved.problems.is_empty(), "{:?}", solved.problems);
    assert_eq!(solved.placed.len(), 3);
    let names: Vec<&str> = solved.placed.iter().map(|x| x.name.as_str()).collect();
    assert_eq!(names, vec!["Plate <1>", "Pin <1>", "Pin <2>"]);
    // Held where the file has them, and the same as what went out.
    assert_eq!(solved.freedom, 0);
    let was = p.solve(assembly).unwrap();
    for (a, b) in was.placed.iter().zip(&solved.placed) {
        close(a.at.origin, b.at.origin);
        close(a.at.z, b.at.z);
        close(a.at.x, b.at.x);
    }
    // Out again, it's the same structure.
    let again = q.step_text(tab, "Again").unwrap();
    assert_eq!(read_step_structure(&again).unwrap().instances.len(), 3);
    // A bad file brings nothing in.
    let tabs = q.tabs.len();
    assert!(q.import_step("not a file", "Bad").is_err());
    assert_eq!(q.tabs.len(), tabs);
}

#[test]
fn a_file_of_plain_solids_comes_in_as_parts_too() {
    let (a, b) = (box_solid(&Frame::WORLD, 1.0, 2.0, 3.0, 1).unwrap(), box_solid(&Frame::translation(v3(10.0, 0.0, 0.0)), 1.0, 2.0, 3.0, 2).unwrap());
    let text = write_step_bodies(&[&a, &b], "Two");
    let mut q = Project::new();
    let (_, tab) = q.import_step(&text, "Two").unwrap();
    let solved = q.solve(tab).unwrap();
    assert_eq!(solved.placed.len(), 2);
    // Named for the file, whatever each solid's called, and where it was.
    assert!(solved.placed[0].name.starts_with("Two"), "{}", solved.placed[0].name);
    assert!((solved.placed[1].body.vertices.iter().fold(f64::MAX, |m, v| m.min(v.x)) - 10.0).abs() < 1e-9);
}

