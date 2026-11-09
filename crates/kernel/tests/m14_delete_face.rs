//! Deleting faces, with the best oracle there is: taking away what a feature added has to give back the part as it was
//! before that feature, in volume and in area.

use coreshape::boolean::{subtract, union};
use coreshape::profile::{Profile, Wire};
use coreshape::*;

const BASE: u32 = 1;
const TOOL: u32 = 2;
const BLEND: u32 = 3;

fn block() -> Body {
    box_solid(&Frame::WORLD, 30.0, 20.0, 10.0, BASE).unwrap()
}

fn z_at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: v3(x, y, z), ..Frame::WORLD }
}

fn from(b: &Body, feature: u32) -> Vec<usize> {
    (0..b.faces.len()).filter(|&f| b.faces[f].tag.feature == feature).collect()
}

/// Deleting all of a feature's faces has to give `want` back.
fn gives_back(b: &Body, feature: u32, want: &Body) {
    let faces = from(b, feature);
    assert!(!faces.is_empty());
    let healed = delete_faces(b, &faces).unwrap_or_else(|e| panic!("{e}"));
    validate(&healed).unwrap();
    let (m, w) = (mass_properties(&healed), mass_properties(want));
    assert!((m.volume - w.volume).abs() < 1e-5 * w.volume, "volume {} against {}", m.volume, w.volume);
    assert!((m.area - w.area).abs() < 1e-5 * w.area, "area {} against {}", m.area, w.area);
    assert!(from(&healed, feature).is_empty());
    // The faces that were left keep their names.
    for f in &healed.faces {
        assert!(want.faces.iter().any(|w| w.tag == f.tag) || f.tag.feature == TOOL, "{:?}", f.tag);
    }
}

#[test]
fn a_through_hole_a_blind_hole_a_pocket_and_a_boss() {
    let b = block();
    let through = subtract(&b, &cylinder(&z_at(15.0, 10.0, -1.0), 4.0, 12.0, TOOL).unwrap()).unwrap();
    gives_back(&through, TOOL, &b);
    let blind = subtract(&b, &cylinder(&z_at(15.0, 10.0, 4.0), 4.0, 12.0, TOOL).unwrap()).unwrap();
    gives_back(&blind, TOOL, &b);
    let pocket = subtract(&b, &box_solid(&z_at(5.0, 5.0, 6.0), 8.0, 6.0, 10.0, TOOL).unwrap()).unwrap();
    gives_back(&pocket, TOOL, &b);
    let boss = union(&b, &cylinder(&z_at(15.0, 10.0, 5.0), 3.0, 10.0, TOOL).unwrap()).unwrap();
    gives_back(&boss, TOOL, &b);
    // A cone and a sphere, on a side face as well as on top.
    let dimple = subtract(&b, &sphere(&z_at(15.0, 10.0, 10.0), 3.0, TOOL).unwrap()).unwrap();
    gives_back(&dimple, TOOL, &b);
    let side = Frame::new(v3(30.0, 10.0, 5.0), Vec3::X, Vec3::Y);
    let cross = subtract(&b, &cylinder(&Frame { origin: side.origin - Vec3::X * 2.0, ..side }, 2.0, 5.0, TOOL).unwrap()).unwrap();
    gives_back(&cross, TOOL, &b);
}

#[test]
fn only_the_faces_chosen_go() {
    // Two holes, of which one goes, and then both at once.
    let b = block();
    let one = subtract(&b, &cylinder(&z_at(8.0, 10.0, -1.0), 3.0, 12.0, TOOL).unwrap()).unwrap();
    let two = subtract(&one, &cylinder(&z_at(22.0, 10.0, -1.0), 3.0, 12.0, 4).unwrap()).unwrap();
    gives_back(&two, 4, &one);
    let both: Vec<usize> = (0..two.faces.len()).filter(|&f| two.faces[f].tag.feature != BASE).collect();
    let healed = delete_faces(&two, &both).unwrap();
    assert!((mass_properties(&healed).volume - 6000.0).abs() < 1e-9);
    assert_eq!(healed.faces.len(), 6);
}

#[test]
fn a_fillet_and_a_chamfer_on_one_edge() {
    let b = block();
    let edge = (0..b.edges.len()).find(|&e| b.edges[e].eval_mid().dist(v3(15.0, 0.0, 10.0)) < 1e-9).unwrap();
    gives_back(&fillet(&b, &[edge], 2.0, BLEND).unwrap(), BLEND, &b);
    gives_back(&chamfer(&b, &[edge], 2.0, BLEND).unwrap(), BLEND, &b);
}

#[test]
fn fillets_all_around_a_box_with_their_corners() {
    let b = block();
    let top: Vec<usize> = (0..b.edges.len()).filter(|&e| (b.edges[e].eval_mid().z - 10.0).abs() < 1e-9).collect();
    gives_back(&fillet(&b, &top, 2.0, BLEND).unwrap(), BLEND, &b);
    gives_back(&chamfer(&b, &top, 2.0, BLEND).unwrap(), BLEND, &b);
    let all: Vec<usize> = (0..b.edges.len()).collect();
    let rounded = fillet(&b, &all, 2.0, BLEND).unwrap();
    assert!(rounded.faces.len() > 20);
    gives_back(&rounded, BLEND, &b);
}

#[test]
fn a_fillet_around_a_round_rim_and_a_hole() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, BASE).unwrap();
    let rim: Vec<usize> = (0..c.edges.len()).filter(|&e| (c.edges[e].eval_mid().z - 20.0).abs() < 1e-9 && matches!(c.edges[e].curve, Curve::Circle { .. })).collect();
    gives_back(&fillet(&c, &rim, 2.0, BLEND).unwrap(), BLEND, &c);
    let plate = subtract(&block(), &cylinder(&z_at(15.0, 10.0, -1.0), 4.0, 12.0, TOOL).unwrap()).unwrap();
    let hole_rim: Vec<usize> = (0..plate.edges.len()).filter(|&e| (plate.edges[e].eval_mid().z - 10.0).abs() < 1e-9 && matches!(plate.edges[e].curve, Curve::Circle { .. })).collect();
    gives_back(&chamfer(&plate, &hole_rim, 1.0, BLEND).unwrap(), BLEND, &plate);
    gives_back(&fillet(&plate, &hole_rim, 1.0, BLEND).unwrap(), BLEND, &plate);
}

#[test]
fn a_rim_over_smooth_corners() {
    // Rounded upright corners first, then the rim around the top, which meets them smoothly.
    let b = block();
    let upright: Vec<usize> = (0..b.edges.len()).filter(|&e| {
        let m = b.edges[e].eval_mid();
        (m.z - 5.0).abs() < 1e-9
    }).collect();
    let rounded = fillet(&b, &upright, 3.0, 2).unwrap();
    let rim: Vec<usize> = (0..rounded.edges.len()).filter(|&e| (rounded.edges[e].eval_mid().z - 10.0).abs() < 1e-9).collect();
    let both = fillet(&rounded, &rim, 1.5, BLEND).unwrap();
    gives_back(&both, BLEND, &rounded);
}

#[test]
fn a_rabbet_a_slot_and_a_corner_notch() {
    let b = block();
    // A rabbet along the top edge: a step of two faces, on the whole length.
    let rabbet = subtract(&b, &box_solid(&z_at(-1.0, -1.0, 8.0), 32.0, 4.0, 3.0, TOOL).unwrap()).unwrap();
    gives_back(&rabbet, TOOL, &b);
    // A slot right across the top, with the top plane on both sides of it.
    let slot = subtract(&b, &box_solid(&z_at(12.0, -1.0, 7.0), 6.0, 22.0, 4.0, TOOL).unwrap()).unwrap();
    assert!(slot.faces.len() > 6);
    gives_back(&slot, TOOL, &b);
    // A notch out of one corner: three faces, with three neighbours.
    let notch = subtract(&b, &box_solid(&z_at(25.0, 15.0, 6.0), 6.0, 6.0, 5.0, TOOL).unwrap()).unwrap();
    gives_back(&notch, TOOL, &b);
    // A corner cut off by one plane, with three sides.
    let cut_tool = box_solid(&Frame::new(v3(30.0, 20.0, 7.0), v3(1.0, 1.0, 1.0), Vec3::X), 20.0, 20.0, 5.0, TOOL);
    let corner = subtract(&b, &cut_tool.unwrap().transformed(&Frame { origin: v3(-3.0, -3.0, -3.0), ..Frame::WORLD })).unwrap();
    assert!(corner.faces.len() >= 7, "{}", corner.faces.len());
    gives_back(&corner, TOOL, &b);
}

#[test]
fn a_void_and_a_separate_piece() {
    let b = block();
    let void_tool = box_solid(&z_at(10.0, 5.0, 3.0), 5.0, 5.0, 4.0, TOOL).unwrap();
    let hollow = subtract(&b, &void_tool).unwrap();
    assert_eq!(hollow.shells.len(), 2);
    gives_back(&hollow, TOOL, &b);
    let two = coreshape::compound(&[b.clone(), box_solid(&z_at(50.0, 0.0, 0.0), 5.0, 5.0, 5.0, TOOL).unwrap()]);
    gives_back(&two, TOOL, &b);
}

#[test]
fn blends_with_no_exact_form_come_off_too() {
    // A rim cut at a slant, whose edge is an ellipse, and a fitted fillet on it.
    let c = cylinder(&Frame::WORLD, 10.0, 40.0, BASE).unwrap();
    let tilt = Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&Frame::translation(v3(-50.0, -50.0, 25.0)));
    let tool = box_solid(&tilt, 100.0, 100.0, 50.0, TOOL).unwrap();
    let cut = subtract(&c, &tool).unwrap();
    let rim: Vec<usize> = (0..cut.edges.len()).filter(|&e| matches!(cut.edges[e].curve, Curve::Ellipse { .. } | Curve::Spline(_) | Curve::Sampled(_))).collect();
    assert!(!rim.is_empty());
    gives_back(&fillet(&cut, &rim, 1.0, BLEND).unwrap(), BLEND, &cut);
    gives_back(&chamfer(&cut, &rim, 1.0, BLEND).unwrap(), BLEND, &cut);
}

#[test]
fn a_fillet_on_a_free_form_rim() {
    let sections: Vec<(Profile, Frame)> = vec![
        (Profile::new(Wire::circle(v2(0.0, 0.0), 10.0)), z_at(0.0, 0.0, 0.0)),
        (Profile::new(Wire::circle(v2(0.0, 0.0), 7.0)), z_at(0.0, 0.0, 12.0)),
        (Profile::new(Wire::circle(v2(0.0, 0.0), 9.0)), z_at(0.0, 0.0, 24.0)),
    ];
    let list: Vec<(&Profile, Frame)> = sections.iter().map(|(p, f)| (p, *f)).collect();
    let vase = loft(&list, BASE).unwrap();
    let rim: Vec<usize> = (0..vase.edges.len()).filter(|&e| (vase.edges[e].eval_mid().z - 24.0).abs() < 1e-6).collect();
    let blended = fillet(&vase, &rim, 1.0, BLEND).unwrap();
    gives_back(&blended, BLEND, &vase);
}

#[test]
fn what_can_t_be_closed_is_refused_by_name() {
    let b = block();
    // One side of the box alone: its four neighbours don't close over it.
    let why = delete_faces(&b, &[0]).unwrap_err().to_string();
    assert!(why.contains("deleting these faces"), "{why}");
    // Everything, and nothing, and something that isn't there.
    assert!(delete_faces(&b, &[0, 1, 2, 3, 4, 5]).is_err());
    assert!(delete_faces(&b, &[]).is_err());
    assert!(delete_faces(&b, &[99]).is_err());
    // Three faces of a box leave a gap with three neighbours, that meet at the far corner: a tetrahedron-like wedge
    // can't come of it, so this has to be refused, or be right.
    let three: Vec<usize> = (0..6).filter(|&f| b.faces[f].surface.normal(Vec3::ZERO).dot(v3(1.0, 1.0, 1.0)) > 0.0).collect();
    if let Ok(result) = delete_faces(&b, &three) {
        validate(&result).unwrap();
        assert!(mass_properties(&result).volume > 0.0);
    }
    // Part of a hole's wall, with the other half left behind.
    let holed = subtract(&b, &cylinder(&z_at(15.0, 10.0, -1.0), 4.0, 12.0, TOOL).unwrap()).unwrap();
    let half = from(&holed, TOOL)[0];
    let result = delete_faces(&holed, &[half]);
    assert!(result.is_err(), "half a hole's wall went, leaving {:?}", result.map(|r| mass_properties(&r).volume));
}
