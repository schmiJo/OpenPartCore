//! The rest of Onshape's assemblies: tangent and width mates, patterns of instances, replicate, exploded views and named
//! positions. Each checked against where things have to be by a formula.

use coreshape::*;
use coreshape_assembly::*;
use coreshape_document::*;

const NEAR: f64 = 1e-6;

fn solid(primitive: Primitive, at: Placement, mode: Mode) -> Op {
    Op::solid(primitive, at, mode)
}

fn block(x: f64, y: f64, z: f64) -> Op {
    solid(Primitive::Box { dx: x.into(), dy: y.into(), dz: z.into() }, Placement::xy(0.0), Mode::NewBody)
}

fn pin(r: f64, h: f64) -> Op {
    solid(Primitive::Cylinder { radius: r.into(), height: h.into() }, Placement::xy(0.0), Mode::NewBody)
}

fn ball(r: f64) -> Op {
    solid(Primitive::Sphere { radius: r.into() }, Placement::xy(0.0), Mode::NewBody)
}

/// A project with one Part Studio holding these parts, made one after another, and an empty assembly. Each part is the
/// body its feature made; extra features act on the last.
fn project(parts: Vec<Op>) -> (Project, TabId, TabId, Vec<BodyId>) {
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let ids = parts.into_iter().map(|op| BodyId { feature: p.studio_mut(studio).unwrap().with(op), index: 0 }).collect();
    let assembly = p.add_assembly("Assembly 1");
    (p, studio, assembly, ids)
}

fn body_of(p: &mut Project, studio: TabId, id: BodyId) -> std::sync::Arc<Body> {
    p.studio_mut(studio).unwrap().rebuild(None).find_body(id).unwrap().body.clone()
}

/// The key of the face of a part whose surface passes this test.
fn key_where(body: &Body, test: impl Fn(&Surface, Vec3) -> bool) -> FaceKey {
    let keys = face_keys(body);
    let found: Vec<usize> = (0..body.faces.len())
        .filter(|&f| {
            let face = &body.faces[f];
            let mid = face.surface.project(body.vertices[body.edges[face.loops[0][0].edge].v0]);
            let n = if face.flipped { -face.surface.normal(mid) } else { face.surface.normal(mid) };
            test(&face.surface, n)
        })
        .collect();
    assert!(!found.is_empty(), "no face fits");
    keys[found[0]].clone()
}

fn on(key: FaceKey) -> Anchor {
    Anchor::Face { key, point: FacePoint::Centroid }
}

fn add(p: &mut Project, assembly: TabId, studio: TabId, body: BodyId, name: &str, at: Frame) -> u32 {
    p.assembly_mut(assembly).unwrap().add_instance(Source::Part { studio, body }, name.to_string(), at)
}

fn connect(p: &mut Project, assembly: TabId, instance: u32, anchor: Anchor) -> u32 {
    p.assembly_mut(assembly).unwrap().add_connector(vec![instance], anchor).unwrap()
}

fn pose(solved: &Solved, path: &[u32]) -> Frame {
    solved.placed.iter().find(|q| q.path == path).unwrap_or_else(|| panic!("{path:?} isn't placed")).at
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < NEAR, "{what}: {a} isn't {b}");
}

fn top(v: Vec3) -> bool {
    v.z > 1.0 - 1e-9
}

/// A 100 x 60 x 10 plate, fixed, and one other part, tilted and lifted, as given.
fn on_a_plate(other: Op, at: Frame) -> (Project, TabId, u32, u32, std::sync::Arc<Body>, std::sync::Arc<Body>) {
    let (mut p, studio, assembly, parts) = project(vec![block(100.0, 60.0, 10.0), other]);
    let plate = add(&mut p, assembly, studio, parts[0], "Plate", Frame::WORLD);
    p.assembly_mut(assembly).unwrap().instance_mut(plate).unwrap().fixed = true;
    let thing = add(&mut p, assembly, studio, parts[1], "Thing", at);
    let (pb, tb) = (body_of(&mut p, studio, parts[0]), body_of(&mut p, studio, parts[1]));
    (p, assembly, plate, thing, pb, tb)
}

fn tilted(origin: Vec3) -> Frame {
    Frame::rotation(origin, v3(1.0, 1.0, 0.0).normalized(), 0.3).compose(&Frame::translation(origin))
}

#[test]
fn a_cylinder_tangent_to_a_plane_lies_on_it() {
    let (mut p, assembly, plate, thing, pb, tb) = on_a_plate(pin(5.0, 40.0), tilted(v3(30.0, 20.0, 25.0)));
    let face = key_where(&pb, |s, n| matches!(s, Surface::Plane(_)) && top(n));
    let side = key_where(&tb, |s, _| matches!(s, Surface::Cylinder { .. }));
    let (a, b) = (connect(&mut p, assembly, plate, on(face)), connect(&mut p, assembly, thing, on(side)));
    p.assembly_mut(assembly).unwrap().add_mate(MateKind::Tangent, a, b).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    // The pin's axis is level, and 5 above the plate's top at 10, all along it.
    let at = pose(&solved, &[thing]);
    near(at.z.z, 0.0, "axis level");
    near(at.origin.z, 15.0, "axis height");
    near(at.to_world(v3(0.0, 0.0, 40.0)).z, 15.0, "far end height");
    // It can still roll and slide: two moves along the plate, a turn about the normal, and a roll about its axis.
    assert_eq!(solved.freedom, 4);
}

#[test]
fn a_ball_tangent_to_a_plane_rests_on_it_or_under_it_flipped() {
    let (mut p, assembly, plate, thing, pb, tb) = on_a_plate(ball(4.0), Frame::translation(v3(20.0, 30.0, 40.0)));
    let face = key_where(&pb, |s, n| matches!(s, Surface::Plane(_)) && top(n));
    let round = key_where(&tb, |s, _| matches!(s, Surface::Sphere { .. }));
    let (a, b) = (connect(&mut p, assembly, plate, on(face.clone())), connect(&mut p, assembly, thing, on(round)));
    let m = p.assembly_mut(assembly).unwrap().add_mate(MateKind::Tangent, b, a).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    near(pose(&solved, &[thing]).origin.z, 14.0, "on top");
    assert_eq!(solved.freedom, 5);
    // Flipped, under the plate's top plane instead.
    p.assembly_mut(assembly).unwrap().mate_mut(m).unwrap().flip = true;
    let solved = p.solve(assembly).unwrap();
    near(pose(&solved, &[thing]).origin.z, 6.0, "under the top's plane");
}

#[test]
fn round_things_tangent_to_each_other() {
    // Two pins side by side, axes parallel and 3 + 5 apart; a ball against a pin, 4 + 3 from its axis; two balls 4 + 2 apart.
    let (mut p, studio, assembly, parts) = project(vec![pin(3.0, 30.0), pin(5.0, 20.0), ball(4.0), ball(2.0)]);
    let a = add(&mut p, assembly, studio, parts[0], "A", Frame::WORLD);
    p.assembly_mut(assembly).unwrap().instance_mut(a).unwrap().fixed = true;
    let b = add(&mut p, assembly, studio, parts[1], "B", tilted(v3(20.0, 5.0, 3.0)));
    let c = add(&mut p, assembly, studio, parts[2], "C", Frame::translation(v3(-12.0, 4.0, 15.0)));
    let d = add(&mut p, assembly, studio, parts[3], "D", Frame::translation(v3(-15.0, -10.0, 20.0)));
    let bodies: Vec<_> = parts.iter().map(|id| body_of(&mut p, studio, *id)).collect();
    let curved = |b: &Body| key_where(b, |s, _| matches!(s, Surface::Cylinder { .. } | Surface::Sphere { .. }));
    let ka = connect(&mut p, assembly, a, on(curved(&bodies[0])));
    let kb = connect(&mut p, assembly, b, on(curved(&bodies[1])));
    let kc = connect(&mut p, assembly, c, on(curved(&bodies[2])));
    let kc2 = connect(&mut p, assembly, c, on(curved(&bodies[2])));
    let kd = connect(&mut p, assembly, d, on(curved(&bodies[3])));
    for (x, y) in [(ka, kb), (ka, kc), (kc2, kd)] {
        p.assembly_mut(assembly).unwrap().add_mate(MateKind::Tangent, x, y).unwrap();
    }
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    let (pb, pc, pd) = (pose(&solved, &[b]), pose(&solved, &[c]), pose(&solved, &[d]));
    near(pb.z.cross(Vec3::Z).norm(), 0.0, "axes parallel");
    near((pb.origin.x.powi(2) + pb.origin.y.powi(2)).sqrt(), 8.0, "axes apart");
    near((pc.origin.x.powi(2) + pc.origin.y.powi(2)).sqrt(), 7.0, "ball from axis");
    near(pc.origin.dist(pd.origin), 6.0, "balls apart");
}

#[test]
fn a_width_mate_centers_a_tab_in_a_slot() {
    // A slot 30 wide in x, from x = 20 to 50, made by two blocks either side; a tab 12 wide sits midway, at x = 35 - 6.
    let (mut p, studio, assembly, parts) = project(vec![block(100.0, 20.0, 10.0), block(12.0, 20.0, 30.0)]);
    let base = add(&mut p, assembly, studio, parts[0], "Base", Frame::WORLD);
    p.assembly_mut(assembly).unwrap().instance_mut(base).unwrap().fixed = true;
    let tab = add(&mut p, assembly, studio, parts[1], "Tab", tilted(v3(70.0, 4.0, 15.0)));
    let (bb, tb) = (body_of(&mut p, studio, parts[0]), body_of(&mut p, studio, parts[1]));
    // The base's own end faces stand in for the slot's walls, x = 0 and x = 100: midway is 50.
    let left = |b: &Body| key_where(b, |s, n| matches!(s, Surface::Plane(_)) && n.x < -1.0 + 1e-9);
    let right = |b: &Body| key_where(b, |s, n| matches!(s, Surface::Plane(_)) && n.x > 1.0 - 1e-9);
    let slot = connect(&mut p, assembly, base, Anchor::Between { faces: [left(&bb), right(&bb)] });
    let width = connect(&mut p, assembly, tab, Anchor::Between { faces: [left(&tb), right(&tb)] });
    p.assembly_mut(assembly).unwrap().add_mate(MateKind::Width, slot, width).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty(), "{:?}", solved.failing);
    // The tab's own middle, at x = 6 in it, is at x = 50.
    near(pose(&solved, &[tab]).to_world(v3(6.0, 10.0, 15.0)).x, 50.0, "centered");
    assert_eq!(solved.freedom, 5);
    // Faces that aren't parallel are refused, saying so.
    let top_face = key_where(&tb, |s, n| matches!(s, Surface::Plane(_)) && top(n));
    let wrong = Anchor::Between { faces: [left(&tb), top_face] };
    assert!(wrong.resolve(&tb).unwrap_err().to_string().contains("parallel"));
}

#[test]
fn patterns_of_instances_follow_their_first() {
    let (mut p, studio, assembly, parts) = project(vec![pin(2.0, 10.0)]);
    let first = add(&mut p, assembly, studio, parts[0], "Pin", Frame::translation(v3(5.0, 0.0, 0.0)));
    let a = p.assembly_mut(assembly).unwrap();
    let row = a.add_pattern(vec![first], PatternAxis::Given { point: Vec3::ZERO, direction: Vec3::X }, PatternKind::Linear { spacing: 12.0 }, 4).unwrap();
    let round = a.add_pattern(vec![first], PatternAxis::Given { point: Vec3::ZERO, direction: Vec3::Z }, PatternKind::Circular { degrees: 90.0 }, 4).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert_eq!(solved.placed.len(), 1 + 3 + 3);
    for k in 1..4u32 {
        near(pose(&solved, &[row, k, first]).origin.dist(v3(5.0 + 12.0 * k as f64, 0.0, 0.0)), 0.0, "in a row");
        let angle = (90.0 * k as f64).to_radians();
        near(pose(&solved, &[round, k, first]).origin.dist(v3(5.0 * angle.cos(), 5.0 * angle.sin(), 0.0)), 0.0, "round");
    }
    // Moving the first moves them all.
    p.assembly_mut(assembly).unwrap().instance_mut(first).unwrap().at = Frame::translation(v3(5.0, 0.0, 7.0));
    let solved = p.solve(assembly).unwrap();
    near(pose(&solved, &[row, 3, first]).origin.z, 7.0, "followed");
    // Counted in the bill of materials.
    assert_eq!(p.bom(assembly).unwrap().rows[0].count, 7);
    // Saved and read back the same.
    let a = p.assembly(assembly).unwrap().clone();
    assert_eq!(Assembly::from_json(&a.to_json()).unwrap(), a);
}

#[test]
fn a_pattern_along_a_connector_turns_with_its_part() {
    let (mut p, studio, assembly, parts) = project(vec![block(60.0, 10.0, 4.0), pin(2.0, 10.0)]);
    let bar = add(&mut p, assembly, studio, parts[0], "Bar", Frame::WORLD);
    let first = add(&mut p, assembly, studio, parts[1], "Pin", Frame::translation(v3(5.0, 5.0, 4.0)));
    let along = connect(&mut p, assembly, bar, Anchor::Frame(Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y)));
    let row = p.assembly_mut(assembly).unwrap().add_pattern(vec![first], PatternAxis::Connector(along), PatternKind::Linear { spacing: 10.0 }, 3).unwrap();
    let solved = p.solve(assembly).unwrap();
    near(pose(&solved, &[row, 2, first]).origin.dist(v3(25.0, 5.0, 4.0)), 0.0, "along the bar");
    // The bar turned a quarter about z: the pattern goes along y now.
    p.assembly_mut(assembly).unwrap().instance_mut(bar).unwrap().at = Frame::rotation(Vec3::ZERO, Vec3::Z, PI / 2.0);
    let solved = p.solve(assembly).unwrap();
    near(pose(&solved, &[row, 2, first]).origin.dist(v3(5.0, 25.0, 4.0)), 0.0, "along the turned bar");
}

#[test]
fn replicate_puts_a_copy_in_every_hole_of_one_size() {
    // A plate with three holes of radius 3 and one of radius 5, and a bolt of radius 3 in the first hole.
    let hole = |x: f64, y: f64, r: f64| solid(Primitive::Cylinder { radius: r.into(), height: 20.0.into() }, Placement::at(point3(x, y, -5.0)), Mode::Remove);
    let (mut p, studio, assembly, parts) = project(vec![block(100.0, 60.0, 10.0), hole(20.0, 20.0, 3.0), hole(50.0, 40.0, 3.0), hole(80.0, 20.0, 3.0), hole(50.0, 15.0, 5.0), pin(3.0, 25.0)]);
    let plate_id = parts[0];
    let plate = add(&mut p, assembly, studio, plate_id, "Plate", Frame::WORLD);
    p.assembly_mut(assembly).unwrap().instance_mut(plate).unwrap().fixed = true;
    let bolt = add(&mut p, assembly, studio, parts[5], "Bolt", Frame::translation(v3(-30.0, 0.0, 0.0)));
    let plate_body = body_of(&mut p, studio, plate_id);
    let bolt_body = body_of(&mut p, studio, parts[5]);
    let first_hole = key_where(&plate_body, |s, _| matches!(s, Surface::Cylinder { frame, radius } if (*radius - 3.0).abs() < 1e-9 && frame.origin.x.abs() < 1e-9 + 20.0 && frame.origin.x > 19.0));
    let shank = key_where(&bolt_body, |s, _| matches!(s, Surface::Cylinder { .. }));
    let at_hole = connect(&mut p, assembly, plate, on(first_hole));
    let on_bolt = connect(&mut p, assembly, bolt, on(shank));
    p.assembly_mut(assembly).unwrap().add_mate(MateKind::Fastened, at_hole, on_bolt).unwrap();
    let r = p.assembly_mut(assembly).unwrap().add_replicate(vec![bolt], at_hole, Vec::new()).unwrap();
    let solved = p.solve(assembly).unwrap();
    assert!(solved.failing.is_empty() && solved.problems.is_empty(), "{:?} {:?}", solved.failing, solved.problems);
    let bolts: Vec<&Placed> = solved.placed.iter().filter(|q| q.path.last() == Some(&bolt)).collect();
    assert_eq!(bolts.len(), 3, "one bolt and two copies, and none in the bigger hole");
    let mut xy: Vec<(f64, f64)> = bolts.iter().map(|q| { let o = q.at.to_world(v3(0.0, 0.0, 12.5)); ((o.x * 1e6).round() / 1e6, (o.y * 1e6).round() / 1e6) }).collect();
    xy.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(xy, vec![(20.0, 20.0), (50.0, 40.0), (80.0, 20.0)]);
    // Saved and read back the same.
    let a = p.assembly(assembly).unwrap().clone();
    assert_eq!(Assembly::from_json(&a.to_json()).unwrap(), a);
    // Deleting the bolt takes the replicate with it.
    p.assembly_mut(assembly).unwrap().delete_instance(bolt).unwrap();
    assert!(p.assembly(assembly).unwrap().replicates.iter().all(|x| x.id != r));
}

#[test]
fn an_exploded_view_moves_only_what_is_shown() {
    let (mut p, studio, assembly, parts) = project(vec![block(10.0, 10.0, 10.0), block(10.0, 10.0, 10.0)]);
    let low = add(&mut p, assembly, studio, parts[0], "Low", Frame::WORLD);
    let high = add(&mut p, assembly, studio, parts[1], "High", Frame::translation(v3(0.0, 0.0, 10.0)));
    let steps = vec![
        ExplodeStep { instances: vec![vec![high]], shift: v3(0.0, 0.0, 20.0), turn: None },
        ExplodeStep { instances: vec![vec![low], vec![high]], shift: v3(30.0, 0.0, 0.0), turn: Some((Vec3::ZERO, Vec3::Z, 90.0)) },
    ];
    let view = p.assembly_mut(assembly).unwrap().add_explode(Some("Apart".into()), steps).unwrap();
    let at = |p: &mut Project, amount: f64, id: u32| pose(&p.exploded(assembly, view, amount).unwrap(), &[id]).origin;
    near(at(&mut p, 0.0, high).dist(v3(0.0, 0.0, 10.0)), 0.0, "none");
    near(at(&mut p, 0.25, high).dist(v3(0.0, 0.0, 20.0)), 0.0, "half the first step");
    near(at(&mut p, 0.5, high).dist(v3(0.0, 0.0, 30.0)), 0.0, "the first step");
    // The second turns a quarter about z through the origin, then shifts 30 in x.
    near(at(&mut p, 1.0, high).dist(v3(30.0, 0.0, 30.0)), 0.0, "both");
    let turned = pose(&p.exploded(assembly, view, 1.0).unwrap(), &[low]);
    near(turned.x.dist(Vec3::Y), 0.0, "turned");
    // The assembly itself hasn't moved.
    near(pose(&p.solve(assembly).unwrap(), &[high]).origin.dist(v3(0.0, 0.0, 10.0)), 0.0, "kept");
    let a = p.assembly(assembly).unwrap().clone();
    assert_eq!(Assembly::from_json(&a.to_json()).unwrap(), a);
}

#[test]
fn named_positions_go_back_to_where_the_mates_were() {
    // A lid on a revolute about x, and a slider in z: two mates, moved and brought back together.
    let (mut p, studio, assembly, parts) = project(vec![block(40.0, 40.0, 5.0), block(40.0, 40.0, 2.0), block(5.0, 5.0, 5.0)]);
    let base = add(&mut p, assembly, studio, parts[0], "Base", Frame::WORLD);
    p.assembly_mut(assembly).unwrap().instance_mut(base).unwrap().fixed = true;
    let lid = add(&mut p, assembly, studio, parts[1], "Lid", Frame::translation(v3(0.0, 0.0, 5.0)));
    let knob = add(&mut p, assembly, studio, parts[2], "Knob", Frame::translation(v3(10.0, 10.0, 7.0)));
    let hinge_a = connect(&mut p, assembly, base, Anchor::Frame(Frame::new(v3(0.0, 0.0, 5.0), Vec3::X, Vec3::Y)));
    let hinge_b = connect(&mut p, assembly, lid, Anchor::Frame(Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y)));
    let rail_a = connect(&mut p, assembly, lid, Anchor::Frame(Frame::new(v3(10.0, 10.0, 2.0), Vec3::Z, Vec3::X)));
    let rail_b = connect(&mut p, assembly, knob, Anchor::Frame(Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X)));
    let hinge = p.assembly_mut(assembly).unwrap().add_mate(MateKind::Revolute, hinge_a, hinge_b).unwrap();
    let rail = p.assembly_mut(assembly).unwrap().add_mate(MateKind::Slider, rail_a, rail_b).unwrap();
    p.solve(assembly).unwrap();
    let shut = p.save_position(assembly, Some("Shut".into())).unwrap();
    p.set_mate_value(assembly, hinge, Dof::Angle, 70.0).unwrap();
    p.set_mate_value(assembly, rail, Dof::Z, 8.0).unwrap();
    let open = p.save_position(assembly, None).unwrap();
    let value = |s: &Solved, m: u32, d: Dof| s.values.iter().find(|v| v.0 == m && v.1 == d).unwrap().2;
    // Where they were when it was saved: the slider's drive may have turned the lid a little too, as nothing holds it.
    let kept = p.assembly(assembly).unwrap().positions.iter().find(|x| x.id == open).unwrap().values.clone();
    let saved = |m: u32, d: Dof| kept.iter().find(|v| v.0 == m && v.1 == d).unwrap().2;
    let back = p.go_to_position(assembly, shut).unwrap();
    near(value(&back, hinge, Dof::Angle), 0.0, "hinge shut");
    near(value(&back, rail, Dof::Z), 0.0, "knob down");
    let again = p.go_to_position(assembly, open).unwrap();
    near(value(&again, hinge, Dof::Angle), saved(hinge, Dof::Angle), "hinge open");
    near(value(&again, rail, Dof::Z), saved(rail, Dof::Z), "knob up");
    near(saved(rail, Dof::Z), 8.0, "the knob's value as set");
    // Saved and read back the same. Frames read back are made square again, which can move a last digit, so it's the
    // positions that are compared.
    let a = p.assembly(assembly).unwrap().clone();
    assert_eq!(Assembly::from_json(&a.to_json()).unwrap().positions, a.positions);
    assert_eq!(a.positions.iter().map(|x| x.name.as_str()).collect::<Vec<_>>(), vec!["Shut", "Position 2"]);
}
