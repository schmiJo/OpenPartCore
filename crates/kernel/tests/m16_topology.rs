//! Offset and shell where the topology changes: a face that doesn't survive goes, its neighbours meet, and the result is
//! what the plain shape would have given. So the oracle is the part without that feature, moved the same way.

use coreshape::boolean::{subtract, union};
use coreshape::*;
use std::f64::consts::PI;

const BASE: u32 = 1;
const TOOL: u32 = 2;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: v3(x, y, z), ..Frame::WORLD }
}

fn block(w: f64, d: f64, h: f64) -> Body {
    box_solid(&Frame::WORLD, w, d, h, BASE).unwrap()
}

fn volume(b: &Body) -> f64 {
    validate(b).unwrap();
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    let v = mass_properties(b).volume;
    assert!((mesh.volume() - v).abs() < 0.02 * v.abs(), "mesh {} against {v}", mesh.volume());
    v
}

fn close(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() <= 1e-7 * b.abs().max(1.0), "{what}: {a} against {b}");
}

fn all_edges(b: &Body) -> Vec<usize> {
    (0..b.edges.len()).collect()
}

#[test]
fn a_fillet_smaller_than_the_wall_is_gone_from_the_cavity() {
    // The commonest failure there is: a rounded part, with walls thicker than the rounding.
    let plain = block(40.0, 30.0, 20.0);
    let rounded = fillet(&plain, &all_edges(&plain), 1.0, TOOL).unwrap();
    let whole = mass_properties(&rounded).volume;
    for wall in [0.5, 0.9, 1.5, 2.0, 3.0] {
        let hollow = shell(&rounded, &[], wall, 5).unwrap();
        let cavity = if wall < 1.0 {
            // The rounding survives, smaller. By the same formula as a rounded block.
            let r = 1.0 - wall;
            let (x, y, z) = (40.0 - 2.0 * wall, 30.0 - 2.0 * wall, 20.0 - 2.0 * wall);
            x * y * z - (4.0 - PI) * r * r * (x + y + z - 6.0 * r) - 8.0 * r * r * r + 4.0 / 3.0 * PI * r * r * r
        } else {
            (40.0 - 2.0 * wall) * (30.0 - 2.0 * wall) * (20.0 - 2.0 * wall)
        };
        close(volume(&hollow), whole - cavity, &format!("wall {wall}"));
    }
}

#[test]
fn inward_past_a_chamfer_and_a_fillet_on_one_edge() {
    let plain = block(40.0, 30.0, 20.0);
    let edge = (0..plain.edges.len()).find(|&e| plain.edges[e].eval_mid().dist(v3(20.0, 0.0, 20.0)) < 1e-9).unwrap();
    let cut = chamfer(&plain, &[edge], 1.0, TOOL).unwrap();
    // A 45 degree chamfer of 1 is used up at 1 / (1 - sqrt(1/2)) * ... the point where the moved planes meet past it.
    let moved = offset(&cut, -5.0).unwrap();
    close(volume(&moved), 30.0 * 20.0 * 10.0, "past a chamfer");
    let small = offset(&cut, -0.5).unwrap();
    assert_eq!(small.faces.len(), 7, "a small move keeps the chamfer");
    let round = fillet(&plain, &[edge], 1.0, TOOL).unwrap();
    let moved = offset(&round, -2.0).unwrap();
    close(volume(&moved), 36.0 * 26.0 * 16.0, "past a fillet");
    assert_eq!(moved.faces.len(), 6);
}

#[test]
fn outward_a_hole_closes() {
    let plate = subtract(&block(40.0, 30.0, 10.0), &cylinder(&at(20.0, 15.0, -1.0), 2.0, 12.0, TOOL).unwrap()).unwrap();
    // Still open.
    close(volume(&offset(&plate, 1.5).unwrap()), 43.0 * 33.0 * 13.0 - PI * 0.25 * 13.0, "still open");
    // Closed.
    let closed = offset(&plate, 3.0).unwrap();
    close(volume(&closed), 46.0 * 36.0 * 16.0, "closed");
    assert_eq!(closed.faces.len(), 6);
}

#[test]
fn outward_a_pocket_and_a_slot_close() {
    let b = block(40.0, 30.0, 10.0);
    let pocket = subtract(&b, &box_solid(&at(16.0, 12.0, 6.0), 8.0, 6.0, 10.0, TOOL).unwrap()).unwrap();
    close(volume(&offset(&pocket, 2.0).unwrap()), 44.0 * 34.0 * 14.0 - 4.0 * 2.0 * 4.0, "pocket still open");
    close(volume(&offset(&pocket, 4.0).unwrap()), 48.0 * 38.0 * 18.0, "pocket closed");
    let slot = subtract(&b, &box_solid(&at(18.0, -1.0, 6.0), 4.0, 32.0, 10.0, TOOL).unwrap()).unwrap();
    close(volume(&offset(&slot, 1.0).unwrap()), 42.0 * 32.0 * 12.0 - 2.0 * 4.0 * 32.0, "slot still open");
    close(volume(&offset(&slot, 3.0).unwrap()), 46.0 * 36.0 * 16.0, "slot closed");
}

#[test]
fn inward_a_boss_goes() {
    let b = block(40.0, 30.0, 10.0);
    let boss = union(&b, &cylinder(&at(20.0, 15.0, 5.0), 2.0, 10.0, TOOL).unwrap()).unwrap();
    // Still there, thinner.
    close(volume(&offset(&boss, -1.0).unwrap()), 38.0 * 28.0 * 8.0 + PI * 1.0 * 5.0, "boss thinner");
    // Gone.
    let gone = offset(&boss, -3.0).unwrap();
    close(volume(&gone), 34.0 * 24.0 * 4.0, "boss gone");
    assert_eq!(gone.faces.len(), 6);
    let square = union(&b, &box_solid(&at(18.0, 13.0, 5.0), 4.0, 4.0, 10.0, TOOL).unwrap()).unwrap();
    close(volume(&offset(&square, -3.0).unwrap()), 34.0 * 24.0 * 4.0, "square boss gone");
}

#[test]
fn outward_past_a_fillet_in_a_corner() {
    // An L, whose inside corner is filleted: a concave fillet is what closes moving out.
    let l = union(&block(40.0, 10.0, 30.0), &block(10.0, 40.0, 30.0)).unwrap();
    let inside = (0..l.edges.len()).find(|&e| l.edges[e].eval_mid().dist(v3(10.0, 10.0, 15.0)) < 1e-9).unwrap();
    let rounded = fillet(&l, &[inside], 1.0, TOOL).unwrap();
    let moved = offset(&rounded, 2.0).unwrap();
    // Moved out by 2, it spans -2 to 42 both ways, less the square from its inside corner at (12, 12) out to (42, 42).
    close(volume(&moved), (44.0 * 44.0 - 30.0 * 30.0) * 34.0, "past a concave fillet");
}

#[test]
fn a_round_rim_and_a_free_form_rim() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, BASE).unwrap();
    let rim: Vec<usize> = (0..c.edges.len()).filter(|&e| (c.edges[e].eval_mid().z - 20.0).abs() < 1e-9 && matches!(c.edges[e].curve, Curve::Circle { .. })).collect();
    let rounded = fillet(&c, &rim, 1.0, TOOL).unwrap();
    close(volume(&offset(&rounded, -2.0).unwrap()), PI * 64.0 * 16.0, "round rim");
    // A fitted fillet on a free-form cup, past its radius.
    let circle = |r: f64| profile::Profile::new(profile::Wire::circle(v2(0.0, 0.0), r));
    let (a, b, d) = (circle(10.0), circle(7.0), circle(9.0));
    let vase = loft(&[(&a, at(0.0, 0.0, 0.0)), (&b, at(0.0, 0.0, 12.0)), (&d, at(0.0, 0.0, 24.0))], BASE).unwrap();
    let top: Vec<usize> = (0..vase.edges.len()).filter(|&e| (vase.edges[e].eval_mid().z - 24.0).abs() < 1e-6).collect();
    let rounded = fillet(&vase, &top, 1.0, TOOL).unwrap();
    let plain = offset(&vase, -2.0).unwrap();
    let moved = offset(&rounded, -2.0).unwrap();
    close(volume(&moved), mass_properties(&plain).volume, "free-form rim");
}

#[test]
fn walls_that_would_cross_are_refused_and_say_where() {
    // A thin wall between a pocket and the outside: moving in by more than half of it would split the part.
    let b = block(40.0, 30.0, 20.0);
    let pocket = subtract(&b, &box_solid(&at(1.0, 5.0, 5.0), 10.0, 20.0, 20.0, TOOL).unwrap()).unwrap();
    let why = offset(&pocket, -2.0).unwrap_err().to_string();
    assert!(why.contains("walls cross near"), "{why}");
    // Two bosses that would grow into each other.
    let one = union(&b, &box_solid(&at(5.0, 5.0, 15.0), 4.0, 4.0, 15.0, TOOL).unwrap()).unwrap();
    let two = union(&one, &box_solid(&at(11.0, 5.0, 15.0), 4.0, 4.0, 15.0, 3).unwrap()).unwrap();
    let why = offset(&two, 2.0).unwrap_err().to_string();
    assert!(why.contains("walls cross near"), "{why}");
    // And a shell whose cavity would come through a thin neck says the same, or that it doesn't fit.
    let why = shell(&pocket, &[], 2.0, 5).unwrap_err().to_string();
    assert!(why.contains("walls cross near") || why.contains("don't fit"), "{why}");
}

#[test]
fn a_shell_of_a_part_with_small_features() {
    // A plate with a rounded edge, a small boss and a shallow hole, shelled thicker than all of them.
    let plate = block(60.0, 40.0, 20.0);
    let edge = (0..plate.edges.len()).find(|&e| plate.edges[e].eval_mid().dist(v3(30.0, 0.0, 20.0)) < 1e-9).unwrap();
    let rounded = fillet(&plate, &[edge], 1.0, 3).unwrap();
    let with_boss = union(&rounded, &cylinder(&at(30.0, 20.0, 15.0), 1.5, 7.0, 4).unwrap()).unwrap();
    let whole = mass_properties(&with_boss).volume;
    let hollow = shell(&with_boss, &[], 2.5, 5).unwrap();
    // The cavity is the plain block's, as the rounding and the boss are both narrower than the walls.
    close(volume(&hollow), whole - 55.0 * 35.0 * 15.0, "cavity");
}

#[test]
fn a_face_that_only_partly_gives_way_is_refused_not_deleted() {
    // A chamfer that tapers: wide at one end of the edge and narrow at the other. Moving in uses up its narrow end, and its
    // wide end should still be there. Deleting it would be the wrong answer, with nothing else to show for it.
    let b = block(40.0, 30.0, 20.0);
    let (p1, p2, p3) = (v3(-1.0, 0.0, 16.8), v3(-1.0, 3.2, 20.0), v3(41.0, 0.45, 20.0));
    let n = (p2 - p1).cross(p3 - p1).normalized();
    let outward = if n.dot(v3(0.0, -1.0, 1.0)) > 0.0 { n } else { -n };
    let frame = Frame::new(p1, outward, (p3 - p1).normalized());
    let tool = box_solid(&Frame { origin: frame.origin - frame.x * 10.0 - frame.y * 100.0, ..frame }, 100.0, 200.0, 50.0, TOOL).unwrap();
    let tapered = subtract(&b, &tool).unwrap();
    assert_eq!(tapered.faces.len(), 7);
    let result = offset(&tapered, -1.2);
    if let Ok(moved) = &result {
        // If it's built at all, it has to keep the chamfer, as the plain box's corner would stick out past it.
        let plain = 37.6 * 27.6 * 17.6;
        assert!(mass_properties(moved).volume < plain - 1e-3, "the chamfer went: {}", mass_properties(moved).volume);
    } else {
        let why = result.err().unwrap().to_string();
        assert!(why.contains("partly gives way") || why.contains("turns round"), "{why}");
    }
    // Further in, past all of it, it's redundant, and the plain box comes out.
    close(volume(&offset(&tapered, -6.0).unwrap()), 28.0 * 18.0 * 8.0, "past the whole chamfer");
}

#[test]
fn a_draft_takes_the_fillet_off_and_puts_it_back() {
    // Rounding around the top used to stop the walls being drafted.
    let plain = block(40.0, 30.0, 20.0);
    let top: Vec<usize> = (0..plain.edges.len()).filter(|&e| (plain.edges[e].eval_mid().z - 20.0).abs() < 1e-9).collect();
    let rounded = fillet(&plain, &top, 1.5, TOOL).unwrap();
    let walls = |b: &Body| -> Vec<usize> {
        (0..b.faces.len()).filter(|&f| matches!(b.faces[f].surface, surface::Surface::Plane(_)) && b.faces[f].tag.feature == BASE && b.faces[f].outward_normal(v3(0.0, 0.0, 0.0)).z.abs() < 1e-9).collect()
    };
    let angle = 3f64.to_radians();
    let done = draft(&rounded, &walls(&rounded), Vec3::Z, Vec3::ZERO, angle).unwrap();
    // The same thing done by hand: draft the plain block, then round its top.
    let drafted = draft(&plain, &walls(&plain), Vec3::Z, Vec3::ZERO, angle).unwrap();
    let top: Vec<usize> = (0..drafted.edges.len()).filter(|&e| (drafted.edges[e].eval_mid().z - 20.0).abs() < 1e-9).collect();
    let by_hand = fillet(&drafted, &top, 1.5, TOOL).unwrap();
    close(volume(&done), volume(&by_hand), "draft past a fillet");
    assert!(done.faces.iter().any(|f| f.tag.feature == TOOL && !f.origin.between.is_empty()));
}
