//! Fillets on realistic parts, each checked for validity, a closed mesh, and against a formula where there is one.
//! Usage: fillet_cases [folder]. With a folder, it also writes each part as STEP with what was asked of it, for
//! `scripts/check_fillets.sh` to try the same fillets in OpenCASCADE. Refusing is reported and counted, not an error.
use coreshape::export::step::write_step;
use coreshape::*;
use std::f64::consts::PI;
use std::time::Instant;

const TAG: u32 = 1;

/// A cut from a right-angle corner by a fillet of this radius, per unit of length.
fn k(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

/// Where the middle of that cut is, from each wall.
fn centroid_off_wall(r: f64) -> f64 {
    r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI)
}

/// What two such cuts have in common where they meet at a right angle with the third edge left sharp.
fn mitre_overlap(r: f64) -> f64 {
    simpson(0.0, r, |d| {
        let a = r - (r * r - (r - d) * (r - d)).max(0.0).sqrt();
        a * a
    })
}

fn simpson(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let n = 4000;
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 0 { 2.0 } else { 4.0 };
    }
    sum * h / 3.0
}

/// A fillet too big for a wall of height `h` at a right angle: tangent to one face, and through the wall's far edge. How
/// far along the face it touches.
fn cliff_reach(r: f64, h: f64) -> f64 {
    (2.0 * r * h - h * h).sqrt()
}

/// The cut in section, between the corner and the arc, per unit of length.
fn cliff_area(r: f64, h: f64) -> f64 {
    let a = cliff_reach(r, h);
    simpson(0.0, a, |x| cliff_depth(r, h, x))
}

/// How deep the cut is at a distance `x` back from the corner, along the face it's tangent to. Measured from the plane of
/// the tangent face, at `x = a` it's nothing, and at `x = 0` it's the wall's height.
fn cliff_depth(r: f64, h: f64, x: f64) -> f64 {
    let a = cliff_reach(r, h);
    r - (r * r - (a - x) * (a - x)).max(0.0).sqrt()
}

/// The same cut swept around an axis. `wall` is the wall's radius, and `outward` says the tangent face runs away from the
/// axis from it.
fn cliff_ring(r: f64, h: f64, wall: f64, outward: bool) -> f64 {
    let a = cliff_reach(r, h);
    simpson(0.0, a, |x| {
        let rho = if outward { wall + x } else { wall - x };
        2.0 * PI * rho * cliff_depth(r, h, x)
    })
}

/// What a straight-walled ring loses, or gains, to a plain fillet.
fn ring_cut(r: f64, wall: f64, outward: bool) -> f64 {
    let d = centroid_off_wall(r);
    2.0 * PI * (if outward { wall + d } else { wall - d }) * k(r)
}

#[derive(Clone)]
struct Step {
    points: Vec<Vec3>,
    radius: f64,
    propagate: bool,
}

struct Case {
    name: &'static str,
    body: Body,
    steps: Vec<Step>,
    expect: Option<f64>,
    /// Asking for what no fillet could be.
    impossible: bool,
}

fn step(points: &[Vec3], radius: f64) -> Step {
    Step { points: points.to_vec(), radius, propagate: true }
}

fn alone(points: &[Vec3], radius: f64) -> Step {
    Step { points: points.to_vec(), radius, propagate: false }
}

fn case(name: &'static str, body: Body, steps: Vec<Step>, expect: Option<f64>) -> Case {
    Case { name, body, steps, expect, impossible: false }
}

fn bx(x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64, tag: u32) -> Body {
    box_solid(&Frame::translation(v3(x, y, z)), dx, dy, dz, tag).unwrap()
}

fn cyl(x: f64, y: f64, z: f64, r: f64, h: f64, tag: u32) -> Body {
    cylinder(&Frame::translation(v3(x, y, z)), r, h, tag).unwrap()
}

fn prism(points: &[(f64, f64)], h: f64) -> Body {
    let list: Vec<Vec2> = points.iter().map(|p| v2(p.0, p.1)).collect();
    extrude(&Profile::new(Wire::polygon(&list)), &Frame::WORLD, h, TAG).unwrap()
}

fn volume(b: &Body) -> f64 {
    mass_properties(b).volume
}

/// A part turned about z, from its half section in radius and height.
fn turned(points: &[(f64, f64)]) -> Body {
    let list: Vec<Vec2> = points.iter().map(|p| v2(p.0, p.1)).collect();
    // A revolve's section sits in its frame's xy plane, around y... so go by what `revolve` asks: x is radius, y is height.
    revolve(&Profile::new(Wire::polygon(&list)), &Frame::WORLD, TAU, TAG).unwrap()
}

fn cases() -> Vec<Case> {
    let mut out = Vec::new();
    let (a, b, c) = (40.0, 30.0, 20.0);
    let whole = a * b * c;
    let block = || bx(0.0, 0.0, 0.0, a, b, c, TAG);

    // ----- Plain ones, which have to keep working. -----
    let r = 3.0;
    out.push(case("box one edge", block(), vec![step(&[v3(20.0, 0.0, c)], r)], Some(whole - k(r) * a)));
    let rim = [v3(20.0, 0.0, c), v3(20.0, b, c), v3(0.0, 15.0, c), v3(a, 15.0, c)];
    out.push(case("box top rim, mitred", block(), vec![step(&rim, r)], Some(whole - k(r) * 2.0 * (a + b) + 4.0 * mitre_overlap(r))));
    let corner = [v3(20.0, b, c), v3(a, 15.0, c), v3(a, b, 10.0)];
    out.push(case("box corner, three edges", block(), vec![step(&corner, r)], Some(whole - k(r) * (a + b + c - 3.0 * r) - r * r * r * (1.0 - PI / 6.0))));
    let every = [
        v3(20.0, 0.0, c),
        v3(20.0, b, c),
        v3(0.0, 15.0, c),
        v3(a, 15.0, c),
        v3(20.0, 0.0, 0.0),
        v3(20.0, b, 0.0),
        v3(0.0, 15.0, 0.0),
        v3(a, 15.0, 0.0),
        v3(0.0, 0.0, 10.0),
        v3(a, 0.0, 10.0),
        v3(0.0, b, 10.0),
        v3(a, b, 10.0),
    ];
    let r2 = 2.0;
    let (p, q, s) = (a - 2.0 * r2, b - 2.0 * r2, c - 2.0 * r2);
    out.push(case("box all twelve", block(), vec![step(&every, r2)], Some(p * q * s + 2.0 * r2 * (p * q + p * s + q * s) + PI * r2 * r2 * (p + q + s) + 4.0 / 3.0 * PI * r2.powi(3))));

    // Upright edges first, then the top as a smooth chain: Pappus, around whichever radius.
    let uprights = [v3(0.0, 0.0, 10.0), v3(a, 0.0, 10.0), v3(0.0, b, 10.0), v3(a, b, 10.0)];
    let rounded_rim = |big: f64, small: f64| -> f64 {
        let base = whole - c * 4.0 * k(big);
        base - k(small) * (2.0 * (a + b - 4.0 * big) + 2.0 * PI * (big - centroid_off_wall(small)))
    };
    out.push(case("uprights r5, then rim r2", block(), vec![step(&uprights, 5.0), step(&[v3(20.0, 0.0, c)], 2.0)], Some(rounded_rim(5.0, 2.0))));
    out.push(case("uprights r2, then rim r2", block(), vec![step(&uprights, 2.0), step(&[v3(20.0, 0.0, c)], 2.0)], Some(rounded_rim(2.0, 2.0))));
    out.push(case("uprights r2, then rim r5", block(), vec![step(&uprights, 2.0), step(&[v3(20.0, 0.0, c)], 5.0)], None));

    // One after another, as people do, sharing a corner.
    out.push(case("one edge, then the next along", block(), vec![step(&[v3(20.0, 0.0, c)], 2.0), step(&[v3(a, 15.0, c)], 2.0)], None));
    out.push(case("one edge, then the next, different radii", block(), vec![step(&[v3(20.0, 0.0, c)], 4.0), step(&[v3(a, 15.0, c)], 2.0)], None));
    out.push(case(
        "three in turn at a corner, same radius",
        block(),
        vec![alone(&[v3(20.0, b, c)], 2.0), alone(&[v3(a, 15.0, c)], 2.0), alone(&[v3(a, b, 10.0)], 2.0)],
        None,
    ));
    out.push(case(
        "three in turn at a corner, r5 r3 r2",
        block(),
        vec![alone(&[v3(a, b, 10.0)], 5.0), alone(&[v3(20.0, b, c)], 3.0), alone(&[v3(a, 15.0, c)], 2.0)],
        None,
    ));

    // Non-right angles.
    let wedge = prism(&[(0.0, 0.0), (40.0, 0.0), (0.0, 30.0)], 20.0);
    out.push(case("wedge's sharp upright edge", wedge, vec![step(&[v3(40.0, 0.0, 10.0)], 2.0)], None));
    let hex: Vec<(f64, f64)> = (0..6).map(|i| { let t = i as f64 * PI / 3.0; (20.0 * t.cos(), 20.0 * t.sin()) }).collect();
    let mid = |i: usize, z: f64| -> Vec3 { let (p, q) = (hex[i], hex[(i + 1) % 6]); v3((p.0 + q.0) / 2.0, (p.1 + q.1) / 2.0, z) };
    let hex_rim: Vec<Vec3> = (0..6).map(|i| mid(i, 15.0)).collect();
    out.push(case("hex top rim, mitred", prism(&hex, 15.0), vec![step(&hex_rim, 2.0)], None));
    let hex_up: Vec<Vec3> = hex.iter().map(|p| v3(p.0, p.1, 7.0)).collect();
    out.push(case("hex uprights r3, then rim r1", prism(&hex, 15.0), vec![step(&hex_up, 3.0), step(&[mid(0, 15.0)], 1.0)], None));

    // ----- Running off a face: a wall too short for the ball. -----
    let (r, h) = (5.0, 2.0);
    let plate = bx(0.0, 0.0, 0.0, 40.0, 30.0, h, TAG);
    out.push(case("thin plate edge, r above thickness", plate.clone(), vec![step(&[v3(20.0, 0.0, h)], r)], Some(volume(&plate) - cliff_area(r, h) * 40.0)));
    let thin_rim = [v3(20.0, 0.0, h), v3(20.0, 30.0, h), v3(0.0, 15.0, h), v3(40.0, 15.0, h)];
    out.push(case("thin plate rim, r above thickness", plate.clone(), vec![step(&thin_rim, r)], None));

    let mut thin_all = thin_rim.to_vec();
    thin_all.extend([v3(0.0, 0.0, 1.0), v3(40.0, 0.0, 1.0), v3(0.0, 30.0, 1.0), v3(40.0, 30.0, 1.0)]);
    out.push(case(
        "thin plate rim and uprights, r above thickness",
        plate.clone(),
        vec![step(&thin_all, r)],
        Some(h * (1200.0 - 4.0 * k(r)) - cliff_area(r, h) * 2.0 * (30.0 + 20.0) - cliff_ring(r, h, r, false)),
    ));
    let drilled = subtract(&bx(0.0, 0.0, 0.0, 40.0, 40.0, h, TAG), &cyl(20.0, 20.0, -1.0, 5.0, h + 2.0, 2)).unwrap();
    let rim_r = 3.0;
    out.push(case("hole rim in thin plate", drilled.clone(), vec![step(&[v3(25.0, 20.0, h)], rim_r)], Some(volume(&drilled) - cliff_ring(rim_r, h, 5.0, true))));

    let bossed = |bh: f64| union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 5.0, TAG), &cyl(0.0, 0.0, 5.0, 10.0, bh, 2)).unwrap();
    let boss = bossed(2.0);
    out.push(case("boss base, r above its height", boss.clone(), vec![step(&[v3(10.0, 0.0, 5.0)], 4.0)], Some(volume(&boss) + cliff_ring(4.0, 2.0, 10.0, true))));
    out.push(case("boss top rim, r above its height", boss.clone(), vec![step(&[v3(10.0, 0.0, 7.0)], 4.0)], Some(volume(&boss) - cliff_ring(4.0, 2.0, 10.0, false))));
    let tall = bossed(10.0);
    out.push(case("boss base, plain", tall.clone(), vec![step(&[v3(10.0, 0.0, 5.0)], 2.0)], Some(volume(&tall) + ring_cut(2.0, 10.0, true))));
    out.push(case("boss top rim, plain", tall.clone(), vec![step(&[v3(10.0, 0.0, 15.0)], 2.0)], Some(volume(&tall) - ring_cut(2.0, 10.0, false))));

    let square = union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 5.0, TAG), &bx(-10.0, -10.0, 5.0, 20.0, 20.0, 10.0, 2)).unwrap();
    let base = [v3(0.0, -10.0, 5.0), v3(0.0, 10.0, 5.0), v3(-10.0, 0.0, 5.0), v3(10.0, 0.0, 5.0)];
    out.push(case("square boss base, mitred", square.clone(), vec![step(&base, 2.0)], Some(volume(&square) + k(2.0) * 80.0 + 4.0 * mitre_overlap(2.0))));
    let ribbed = |rw: f64, rh: f64| union(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 5.0, TAG), &bx(30.0 - rw / 2.0, 0.0, 5.0, rw, 40.0, rh, 2)).unwrap();
    let low = ribbed(10.0, 2.0);
    out.push(case("rib base, r above its height", low.clone(), vec![step(&[v3(25.0, 20.0, 5.0)], 4.0)], Some(volume(&low) + cliff_area(4.0, 2.0) * 40.0)));
    let thin = ribbed(4.0, 20.0);
    out.push(case("rib top edge, r above its width", thin.clone(), vec![step(&[v3(28.0, 20.0, 25.0)], 6.0)], Some(volume(&thin) - cliff_area(6.0, 4.0) * 40.0)));
    let stepped = prism(&[(0.0, 0.0), (60.0, 0.0), (60.0, 5.0), (30.0, 5.0), (30.0, 7.0), (0.0, 7.0)], 40.0);
    out.push(case("step's top edge, r above its height", stepped.clone(), vec![step(&[v3(30.0, 7.0, 20.0)], 4.0)], Some(volume(&stepped) - cliff_area(4.0, 2.0) * 40.0)));
    out.push(case("step's bottom edge, r above its height", stepped.clone(), vec![step(&[v3(30.0, 5.0, 20.0)], 4.0)], Some(volume(&stepped) + cliff_area(4.0, 2.0) * 40.0)));
    let shaft = turned(&[(0.0, 0.0), (20.0, 0.0), (20.0, 30.0), (22.0, 30.0), (22.0, 50.0), (0.0, 50.0)]);
    out.push(case("shaft shoulder, r above its height, inside", shaft.clone(), vec![step(&[v3(20.0, 0.0, 30.0)], 4.0)], None));
    out.push(case("shaft shoulder, r above its height, outside", shaft.clone(), vec![step(&[v3(22.0, 0.0, 30.0)], 4.0)], None));

    let sliver = ribbed(1.0, 1.0);
    out.push(Case { name: "rib too small either way", body: sliver, steps: vec![step(&[v3(29.5, 20.0, 6.0)], 5.0)], expect: None, impossible: true });

    // Running off in only part of its length.
    let near_edge = union(&bx(-30.0, -30.0, 0.0, 60.0, 60.0, 10.0, TAG), &cyl(-18.0, 0.0, 10.0, 10.0, 10.0, 2)).unwrap();
    out.push(case("boss near the plate's edge, rim runs off part way", near_edge, vec![step(&[v3(-8.0, 0.0, 10.0)], 5.0)], None));
    let varying = prism(&[(0.0, 0.0), (60.0, 0.0), (60.0, 2.0), (0.0, 8.0)], 30.0);
    out.push(case("slanted top's edge, plain", varying, vec![step(&[v3(30.0, 5.0, 30.0)], 4.0)], None));
    // Stood up: x across, and what's z to the prism is y here. The wall at y = 0 runs from 2 tall to 10 tall under the top.
    let along_y = |points: &[(f64, f64)], y0: f64, dy: f64, tag: u32| -> Body {
        let list: Vec<Vec2> = points.iter().map(|p| v2(p.0, p.1)).collect();
        extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, y0 + dy, 0.0), -Vec3::Y, Vec3::X), dy, tag).unwrap()
    };
    let falling = along_y(&[(0.0, 18.0), (60.0, 10.0), (60.0, 20.0), (0.0, 20.0)], 0.0, 30.0, TAG);
    out.push(case("wall getting shorter, nothing beyond it", falling, vec![step(&[v3(30.0, 0.0, 20.0)], 4.0)], None));
    let ledged = union(&bx(0.0, 0.0, 0.0, 60.0, 30.0, 20.0, TAG), &along_y(&[(0.0, 0.0), (60.0, 0.0), (60.0, 10.0), (0.0, 18.0)], -10.0, 10.0, 2)).unwrap();
    out.push(case("wall getting shorter, a ledge beyond it", ledged, vec![step(&[v3(30.0, 0.0, 20.0)], 4.0)], None));

    // ----- Things in the way. -----
    let with_hole = subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 10.0, TAG), &cyl(30.0, 5.0, -1.0, 3.0, 12.0, 2)).unwrap();
    out.push(case("hole across a fillet's contact line", with_hole, vec![step(&[v3(10.0, 0.0, 10.0)], 5.0)], None));
    let notched = subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 10.0, TAG), &bx(25.0, -1.0, 5.0, 10.0, 11.0, 6.0, 2)).unwrap();
    out.push(case("edge with a notch in it, both pieces", notched.clone(), vec![step(&[v3(10.0, 0.0, 10.0), v3(50.0, 0.0, 10.0)], 2.0)], Some(volume(&notched) - k(2.0) * 50.0)));
    let keyed = subtract(&cyl(0.0, 0.0, 0.0, 10.0, 40.0, TAG), &bx(-2.0, 7.0, 10.0, 4.0, 10.0, 20.0, 2)).unwrap();
    out.push(case("keyway's long edge on a shaft", keyed, vec![step(&[v3(2.0, 96f64.sqrt(), 20.0)], 0.5)], None));

    // ----- Re-entrant corners. -----
    let ell = || prism(&[(0.0, 0.0), (40.0, 0.0), (40.0, 15.0), (15.0, 15.0), (15.0, 40.0), (0.0, 40.0)], 20.0);
    let ell_rim = [v3(20.0, 0.0, 20.0), v3(40.0, 7.0, 20.0), v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0), v3(7.0, 40.0, 20.0), v3(0.0, 20.0, 20.0)];
    out.push(case("L's top rim", ell(), vec![step(&ell_rim, 2.0)], None));
    out.push(case("L's inside upright r3, then rim r1", ell(), vec![step(&[v3(15.0, 15.0, 10.0)], 3.0), step(&[v3(20.0, 0.0, 20.0)], 1.0)], None));
    let mut all_of_ell = ell_rim.to_vec();
    all_of_ell.extend([v3(40.0, 0.0, 10.0), v3(40.0, 15.0, 10.0), v3(15.0, 15.0, 10.0), v3(15.0, 40.0, 10.0), v3(0.0, 40.0, 10.0), v3(0.0, 0.0, 10.0)]);
    out.push(case("L's rim and uprights at once", ell(), vec![step(&all_of_ell, 2.0)], None));
    out.push(case("L's inside upright r2, then rim r2", ell(), vec![step(&[v3(15.0, 15.0, 10.0)], 2.0), step(&[v3(20.0, 0.0, 20.0)], 2.0)], None));
    out.push(case("L's inside upright and the rim edges beside it", ell(), vec![step(&[v3(15.0, 15.0, 10.0), v3(27.0, 15.0, 20.0), v3(15.0, 27.0, 20.0)], 2.0)], None));

    // ----- Pockets. -----
    let pocket = || subtract(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 20.0, TAG), &bx(10.0, 10.0, 8.0, 40.0, 20.0, 20.0, 2)).unwrap();
    let floor = [v3(30.0, 10.0, 8.0), v3(30.0, 30.0, 8.0), v3(10.0, 20.0, 8.0), v3(50.0, 20.0, 8.0)];
    let walls = [v3(10.0, 10.0, 14.0), v3(50.0, 10.0, 14.0), v3(10.0, 30.0, 14.0), v3(50.0, 30.0, 14.0)];
    let mouth = [v3(30.0, 10.0, 20.0), v3(30.0, 30.0, 20.0), v3(10.0, 20.0, 20.0), v3(50.0, 20.0, 20.0)];
    let v0 = volume(&pocket());
    // Filling a corner from inside: the mitres add what each adds, less what they share.
    out.push(case("pocket floor, mitred", pocket(), vec![step(&floor, 2.0)], Some(v0 + k(2.0) * 120.0 - 4.0 * mitre_overlap(2.0))));
    let mut floor_walls = floor.to_vec();
    floor_walls.extend(walls);
    // A ball in each corner, and the same sums as a box's corner, from the other side.
    out.push(case("pocket floor and walls at once", pocket(), vec![step(&floor_walls, 2.0)], Some(v0 + k(2.0) * (120.0 + 4.0 * 12.0 - 12.0 * 2.0) + 4.0 * 8.0 * (1.0 - PI / 6.0))));
    out.push(case("pocket walls r5, then floor r2, then mouth r1", pocket(), vec![step(&walls, 5.0), step(&[floor[0]], 2.0), step(&[mouth[0]], 1.0)], None));
    out.push(case("pocket walls r2, then floor r2, then mouth r2", pocket(), vec![step(&walls, 2.0), step(&[floor[0]], 2.0), step(&[mouth[0]], 2.0)], None));
    let mut every_edge = floor_walls.clone();
    every_edge.extend(mouth);
    out.push(case("pocket floor, walls and mouth at once", pocket(), vec![step(&every_edge, 2.0)], None));

    // ----- Curved neighbours and fitted edges. -----
    let on_plate = bx(-30.0, -30.0, -5.0, 60.0, 60.0, 5.0, TAG);
    let coned = union(&on_plate, &cone_frustum(&Frame::WORLD, 15.0, 8.0, 20.0, 2).unwrap()).unwrap();
    out.push(case("cone on a plate, base", coned, vec![step(&[v3(15.0, 0.0, 0.0)], 2.0)], None));
    let balled = union(&on_plate, &sphere(&Frame::translation(v3(0.0, 0.0, 5.0)), 12.0, 2).unwrap()).unwrap();
    let foot = (144.0f64 - 25.0).sqrt();
    out.push(case("ball on a plate, base", balled, vec![step(&[v3(foot, 0.0, 0.0)], 2.0)], None));
    let tee = union(&cylinder(&Frame::new(v3(-40.0, 0.0, 0.0), Vec3::X, Vec3::Y), 15.0, 80.0, TAG).unwrap(), &cyl(0.0, 0.0, 0.0, 8.0, 30.0, 2)).unwrap();
    let saddle = tee.edges_where(|i, e| !matches!(e.curve, Curve::Line { .. } | Curve::Circle { .. }) && tee.edge_midpoint(i).z > 5.0);
    if let Some(&first) = saddle.first() {
        let on = tee.edge_midpoint(first);
        out.push(case("tee saddle", tee.clone(), vec![step(&[on], 2.0)], None));
    }
    let chamfered = chamfer(&block(), &[block().edges_where(|i, _| block().edge_midpoint(i).dist(v3(20.0, 0.0, c)) < 1e-6)[0]], 3.0, 2).unwrap();
    out.push(case("chamfer's long edge", chamfered.clone(), vec![step(&[v3(20.0, 3.0, c)], 1.0)], None));
    // The chamfer takes the tool off at a slant: from y = 3 at the top to y = 2 at the fillet's lower contact.
    let beside = |r: f64, dist: f64| whole - dist * dist / 2.0 * a - (k(r) * b - k(r) * (dist - centroid_off_wall(r)));
    out.push(case("edge beside a chamfer, ending on it", chamfered, vec![step(&[v3(a, 15.0, c)], 1.0)], Some(beside(1.0, 3.0))));

    // From real parts, as they were found to go wrong: a triangular rib running down to a wedge on a plate.
    let gusset = {
        let plate = bx(0.0, -30.0, 0.0, 80.0, 60.0, 10.0, TAG);
        let upright = bx(0.0, -30.0, 0.0, 10.0, 60.0, 70.0, 2);
        let list = [v2(10.0, 10.0), v2(80.0, 10.0), v2(10.0, 70.0)];
        let rib = extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, 4.0, 0.0), -Vec3::Y, Vec3::X), 8.0, 3).unwrap();
        union(&union(&plate, &upright).unwrap(), &rib).unwrap()
    };
    // Each side's fill runs from the upright to where the slope comes down through it.
    let fill = |r: f64| k(r) * (70.0 - 7.0 / 6.0 * centroid_off_wall(r));
    out.push(case("gusset rib's base, down to its wedge", gusset.clone(), vec![step(&[v3(45.0, 4.0, 10.0), v3(45.0, -4.0, 10.0)], 3.0)], Some(volume(&gusset) + 2.0 * fill(3.0))));
    out.push(case("gusset rib's base and joint", gusset.clone(), vec![step(&[v3(45.0, 4.0, 10.0), v3(45.0, -4.0, 10.0)], 3.0), step(&[v3(10.0, 4.0, 40.0), v3(10.0, -4.0, 40.0)], 3.0)], None));
    // A web with a sloped back, between two flanges, chamfered along that back.
    let shelf = {
        let list = [v2(0.0, 0.0), v2(120.0, 0.0), v2(0.0, 120.0)];
        let web = extrude(&Profile::new(Wire::polygon(&list)), &Frame::new(v3(0.0, 2.5, 0.0), -Vec3::Y, Vec3::X), 5.0, TAG).unwrap();
        union(&union(&web, &bx(0.0, -15.0, 0.0, 120.0, 30.0, 5.0, 2)).unwrap(), &bx(0.0, -15.0, 0.0, 5.0, 30.0, 120.0, 3)).unwrap()
    };
    // Along the web's back from flange to flange, and the deeper into the web, the shorter that is.
    let back = 110.0 * 2f64.sqrt();
    out.push(case(
        "shelf web's sloped back, ending on flanges",
        shelf.clone(),
        vec![step(&[v3(60.0, 2.5, 60.0), v3(60.0, -2.5, 60.0)], 1.0)],
        Some(volume(&shelf) - 2.0 * k(1.0) * (back - 2.0 * centroid_off_wall(1.0))),
    ));
    // A connecting rod, whose web runs tangent into its bosses.
    let rod = |length: f64| {
        let (big, small) = (22.0f64, 13.0f64);
        let lean = ((big - small) / length).asin();
        let (c, s) = (lean.cos(), lean.sin());
        let (a, b, d, e) = (v2(big * c, big * s), v2(small * c, length + small * s), v2(-small * c, length + small * s), v2(-big * c, big * s));
        let wire = PathBuilder::new(e).arc_to(a, v2(0.0, 0.0), true).line_to(b).arc_to(d, v2(0.0, length), true).close();
        let web = extrude(&Profile::new(wire), &Frame::translation(v3(0.0, 0.0, -4.0)), 8.0, TAG).unwrap();
        let bosses = union(&cyl(0.0, 0.0, -8.0, big, 16.0, 2), &cyl(0.0, length, -8.0, small, 16.0, 3)).unwrap();
        union(&web, &bosses).unwrap()
    };
    for length in [100.0, 120.0] {
        let name: &'static str = if length == 100.0 { "connecting rod's web into its bosses, L 100" } else { "connecting rod's web into its bosses, L 120" };
        let body = rod(length);
        let on = |z: f64| vec![v3(0.0, 22.0, z), v3(0.0, length - 13.0, z)];
        out.push(case(name, body, vec![step(&[on(4.0), on(-4.0)].concat(), 2.0)], None));
    }

    // Ribs meeting walls, as on a moulded part.
    let walled = union(&union(&bx(0.0, 0.0, 0.0, 60.0, 40.0, 5.0, TAG), &bx(0.0, 0.0, 5.0, 60.0, 5.0, 20.0, 2)).unwrap(), &bx(28.0, 5.0, 5.0, 4.0, 35.0, 15.0, 3)).unwrap();
    out.push(case("rib against a wall, its base", walled.clone(), vec![step(&[v3(28.0, 20.0, 5.0), v3(32.0, 20.0, 5.0)], 1.0)], None));
    out.push(case("rib against a wall, its joint to the wall", walled.clone(), vec![step(&[v3(28.0, 5.0, 12.0), v3(32.0, 5.0, 12.0)], 1.0)], None));
    out.push(case("rib against a wall, base and joint at once", walled, vec![step(&[v3(28.0, 20.0, 5.0), v3(32.0, 20.0, 5.0), v3(28.0, 5.0, 12.0), v3(32.0, 5.0, 12.0)], 1.0)], None));
    let crossed = union(&union(&bx(0.0, 0.0, 0.0, 60.0, 60.0, 5.0, TAG), &bx(28.0, 0.0, 5.0, 4.0, 60.0, 10.0, 2)).unwrap(), &bx(0.0, 28.0, 5.0, 60.0, 4.0, 10.0, 3)).unwrap();
    let cross_up = [v3(28.0, 28.0, 10.0), v3(32.0, 28.0, 10.0), v3(28.0, 32.0, 10.0), v3(32.0, 32.0, 10.0)];
    out.push(case("crossed ribs, their joints", crossed.clone(), vec![step(&cross_up, 1.0)], None));
    out.push(case("crossed ribs, their joints then bases", crossed, vec![step(&cross_up, 1.0), step(&[v3(14.0, 28.0, 5.0), v3(14.0, 32.0, 5.0), v3(46.0, 28.0, 5.0), v3(46.0, 32.0, 5.0)], 1.0)], None));
    out
}

fn find_edges(b: &Body, points: &[Vec3]) -> Result<Vec<usize>> {
    let mut out = Vec::new();
    for p in points {
        let hit = (0..b.edges.len()).find(|&i| {
            let e = &b.edges[i];
            e.curve.closest(*p, e.t0, e.t1).1 < 1e-6
        });
        match hit {
            Some(i) if !out.contains(&i) => out.push(i),
            Some(_) => {}
            None => return Err(Error::InvalidInput(format!("there's no edge at ({:.3}, {:.3}, {:.3}) any more", p.x, p.y, p.z))),
        }
    }
    Ok(out)
}

fn run(c: &Case) -> Result<Body> {
    let mut body = c.body.clone();
    for (i, s) in c.steps.iter().enumerate() {
        let edges = find_edges(&body, &s.points)?;
        body = fillet_with(&body, &edges, s.radius, BlendOptions { propagate: s.propagate }, 200 + i as u32)?;
    }
    Ok(body)
}

fn check(c: &Case, b: &Body) -> std::result::Result<f64, String> {
    validate(b).map_err(|e| format!("invalid: {:?}", e.first()))?;
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).map_err(|e| format!("no mesh: {e}"))?;
    if mesh.open_edge_count() != 0 {
        return Err("mesh not closed".to_string());
    }
    let v = volume(b);
    if (mesh.volume() - v).abs() > 0.01 * v.abs().max(1.0) {
        return Err(format!("mesh volume {} against {v}", mesh.volume()));
    }
    if let Some(want) = c.expect {
        if (v - want).abs() > 1e-6 * want.abs().max(1.0) {
            return Err(format!("volume {v}, and the formula says {want}"));
        }
    }
    Ok(v)
}

fn json_points(s: &Step) -> String {
    let list: Vec<String> = s.points.iter().map(|p| format!("[{:.15e},{:.15e},{:.15e}]", p.x, p.y, p.z)).collect();
    format!("{{\"points\":[{}],\"radius\":{},\"propagate\":{}}}", list.join(","), s.radius, s.propagate)
}

fn main() {
    let folder = std::env::args().nth(1);
    let only = std::env::args().nth(2);
    let list = cases();
    let (mut done, mut refused, mut wrong, mut refused_right) = (0, 0, 0, 0);
    let mut index = Vec::new();
    for (i, c) in list.iter().enumerate() {
        if only.as_deref().is_some_and(|o| !c.name.contains(o)) {
            continue;
        }
        let t = Instant::now();
        let got = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(c)));
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let mut mine = "null".to_string();
        let mut result = "null".to_string();
        match got {
            Err(_) => {
                wrong += 1;
                println!("WRONG   {:<52} panicked", c.name);
            }
            Ok(Err(why)) => {
                if c.impossible {
                    refused_right += 1;
                    println!("rightly {:<52} {why}", c.name);
                } else {
                    refused += 1;
                    println!("REFUSED {:<52} {why}", c.name);
                }
            }
            Ok(Ok(body)) => match check(c, &body) {
                Ok(v) => {
                    done += 1;
                    mine = format!("{v}");
                    if let Some(dir) = &folder {
                        std::fs::write(format!("{dir}/result{i}.step"), write_step(&body, c.name)).unwrap();
                        result = format!("\"result{i}.step\"");
                    }
                    let formula = c.expect.map_or(String::new(), |_| " matches its formula".to_string());
                    println!("ok      {:<52} {v:.4} in {ms:.0} ms{formula}", c.name);
                }
                Err(why) => {
                    wrong += 1;
                    println!("WRONG   {:<52} {why}", c.name);
                }
            },
        }
        if let Some(dir) = &folder {
            std::fs::write(format!("{dir}/case{i}.step"), write_step(&c.body, c.name)).unwrap();
            let steps: Vec<String> = c.steps.iter().map(json_points).collect();
            index.push(format!("{{\"name\":\"{}\",\"step\":\"case{i}.step\",\"steps\":[{}],\"mine\":{mine},\"result\":{result}}}", c.name, steps.join(",")));
        }
    }
    println!("{} cases: {done} done, {refused} refused, {refused_right} rightly refused, {wrong} wrong", done + refused + refused_right + wrong);
    if let Some(dir) = &folder {
        std::fs::write(format!("{dir}/cases.json"), format!("[{}]", index.join(",\n"))).unwrap();
    }
}
