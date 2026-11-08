//! Lofts and free-form surfaces, against shapes whose answers are known exactly and against the same shapes
//! built from analytic surfaces. A loft between equal circles is a true cylinder written as a rational patch,
//! so anything done to it has to come out the same as on a real cylinder.

use coreshape::boolean::{intersect, subtract, union};
use coreshape::surface::Surface;
use coreshape::*;
use coreshape::profile::{PathBuilder, Profile, Wire};
use coreshape::spline::Spline;
use std::f64::consts::PI;

fn at(z: f64) -> Frame {
    Frame { origin: v3(0.0, 0.0, z), ..Frame::WORLD }
}

fn circle(r: f64) -> Profile {
    Profile::new(Wire::circle(v2(0.0, 0.0), r))
}

fn circle_with_hole(r: f64, hole: f64) -> Profile {
    Profile::new(Wire::circle(v2(0.0, 0.0), r)).with_hole(Wire::circle(v2(0.0, 0.0), hole))
}

fn ellipse(major: f64, minor: f64) -> Profile {
    let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, major, minor, 0.0, 2.0 * PI).unwrap();
    Profile::new(PathBuilder::new(v2(major, 0.0)).spline_to(c).close())
}

fn square(side: f64) -> Profile {
    let h = side / 2.0;
    Profile::new(Wire::polygon(&[v2(-h, -h), v2(h, -h), v2(h, h), v2(-h, h)]))
}

fn volume(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn free_faces(b: &Body) -> usize {
    b.faces.iter().filter(|f| matches!(f.surface, Surface::Free { .. })).count()
}

/// Valid, watertight, and its mesh agrees with its integrated volume.
fn check(b: &Body) -> f64 {
    validate(b).unwrap();
    let props = mass_properties(b);
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    let by_mesh = mesh.volume();
    assert!((by_mesh - props.volume).abs() < 0.01 * props.volume.abs(), "mesh {by_mesh}, integrated {}", props.volume);
    assert!(props.volume_error < 1e-6 * props.volume.abs(), "{}", props.volume_error);
    props.volume
}

#[test]
fn squares_of_one_size_make_a_box_of_planes() {
    let b = loft(&[(&square(10.0), at(0.0)), (&square(10.0), at(7.0))], 1).unwrap();
    assert_eq!(free_faces(&b), 0);
    assert!((check(&b) - 700.0).abs() < 1e-9);
}

#[test]
fn squares_of_two_sizes_make_a_frustum_of_planes() {
    let b = loft(&[(&square(10.0), at(0.0)), (&square(4.0), at(9.0))], 1).unwrap();
    assert_eq!(free_faces(&b), 0);
    let want = 9.0 * (100.0 + 40.0 + 16.0) / 3.0;
    assert!((check(&b) - want).abs() < 1e-9);
}

#[test]
fn circles_of_one_size_make_a_true_cylinder() {
    let (r, h) = (6.0, 11.0);
    let b = loft(&[(&circle(r), at(0.0)), (&circle(r), at(h))], 1).unwrap();
    assert!(free_faces(&b) > 0);
    let want = PI * r * r * h;
    assert!((check(&b) - want).abs() < 1e-9 * want, "{} against {want}", volume(&b));
    // Every point of it is on the true cylinder.
    let real = Surface::Cylinder { frame: Frame::WORLD, radius: r };
    let mesh = tessellate(&b, &TessParams::for_size(5.0)).unwrap();
    for p in &mesh.positions {
        if p.z.abs() > 1e-9 && (p.z - h).abs() > 1e-9 {
            assert!(real.sdf(*p).abs() < 1e-9, "{p:?}");
        }
    }
}

#[test]
fn circles_of_two_sizes_make_a_true_frustum() {
    let (a, b, h) = (6.0, 2.5, 8.0);
    let body = loft(&[(&circle(a), at(0.0)), (&circle(b), at(h))], 1).unwrap();
    let want = PI * h * (a * a + a * b + b * b) / 3.0;
    assert!((check(&body) - want).abs() < 1e-9 * want, "{} against {want}", volume(&body));
}

#[test]
fn a_square_goes_to_a_circle() {
    // At height fraction s, each section is the blend of a square's and a circle's boundary point for point, and
    // the volume by slices comes from the shoelace formula over those points.
    let (side, r, h) = (10.0, 4.0, 6.0);
    let b = loft(&[(&square(side), at(0.0)), (&circle(r), at(h))], 1).unwrap();
    assert_eq!(free_faces(&b), 4);
    let got = check(&b);
    // Bounds: it holds at least the smaller end's prism, and is under the big end's.
    assert!(got > PI * r * r * h && got < side * side * h, "{got}");
    assert!(mass_properties(&b).area > 0.0);
    // And by slicing the mesh-independent surface: its own cross sections, integrated.
    let by_slices = slices(&b, h);
    assert!((got - by_slices).abs() < 1e-3 * got, "{got} against {by_slices}");
}

/// Volume by slices: intersect with thin layers, an independent route through the Boolean.
fn slices(b: &Body, h: f64) -> f64 {
    let n = 24;
    let mut sum = 0.0;
    let f = Frame { origin: v3(-50.0, -50.0, 0.0), ..Frame::WORLD };
    for k in 0..n {
        let (z0, z1) = (h * k as f64 / n as f64, h * (k + 1) as f64 / n as f64);
        let layer = box_solid(&Frame { origin: f.origin + v3(0.0, 0.0, z0), ..f }, 100.0, 100.0, z1 - z0, 9).unwrap();
        sum += volume(&intersect(b, &layer).unwrap());
    }
    sum
}

#[test]
fn three_circles_make_a_surface_of_revolution() {
    // Through three coaxial circles, the radius and height run on the same interpolating cubic, so this is a
    // revolved curve: volume is the integral of pi r^2 dz along it.
    let (r0, r1, r2) = (5.0, 8.0, 3.0);
    let (z1, z2) = (6.0, 13.0);
    let b = loft(&[(&circle(r0), at(0.0)), (&circle(r1), at(z1)), (&circle(r2), at(z2))], 1).unwrap();
    let got = check(&b);
    // Two sections' worth of parameters by height, so radius is a quadratic in height through the three circles.
    let lagrange = |z: f64| {
        r0 * (z - z1) * (z - z2) / ((0.0 - z1) * (0.0 - z2))
            + r1 * (z - 0.0) * (z - z2) / ((z1 - 0.0) * (z1 - z2))
            + r2 * (z - 0.0) * (z - z1) / ((z2 - 0.0) * (z2 - z1))
    };
    let n = 4000;
    let want: f64 = (0..n).map(|k| { let z = z2 * (k as f64 + 0.5) / n as f64; PI * lagrange(z).powi(2) * z2 / n as f64 }).sum();
    assert!((got - want).abs() < 1e-5 * want, "{got} against {want}");
}

#[test]
fn a_ring_with_a_hole() {
    let b = loft(&[(&circle_with_hole(9.0, 4.0), at(0.0)), (&circle_with_hole(7.0, 2.0), at(10.0))], 1).unwrap();
    let frustum = |a: f64, b: f64| PI * 10.0 * (a * a + a * b + b * b) / 3.0;
    let want = frustum(9.0, 7.0) - frustum(4.0, 2.0);
    assert!((check(&b) - want).abs() < 1e-9 * want);
}

#[test]
fn a_tilted_section() {
    // A circle's frustum cut on a slant has a known volume when it's a true cylinder: the mean height.
    let tilt = 0.3f64;
    let top = Frame::new(v3(0.0, 0.0, 10.0), v3(tilt.sin(), 0.0, tilt.cos()), Vec3::X);
    // The top section's circle, as an ellipse in the tilted plane, keeps the cylinder.
    let slanted = ellipse(5.0 / tilt.cos(), 5.0);
    let b = loft(&[(&circle(5.0), at(0.0)), (&slanted, top)], 1).unwrap();
    let want = PI * 25.0 * 10.0;
    assert!((check(&b) - want).abs() < 1e-8 * want, "{} against {want}", volume(&b));
}

// ----- Booleans, against the same shape with analytic surfaces -----

fn same_volume(a: &Body, b: &Body, what: &str) {
    let (x, y) = (check(a), check(b));
    assert!((x - y).abs() < 1e-6 * y.abs().max(1.0), "{what}: on the patch {x}, on the real surface {y}");
}

#[test]
fn a_patch_cylinder_cuts_like_a_real_one() {
    let (r, h) = (6.0, 20.0);
    let patch = loft(&[(&circle(r), at(0.0)), (&circle(r), at(h))], 1).unwrap();
    let real = cylinder(&Frame::WORLD, r, h, 1).unwrap();
    for (name, tool) in [
        ("box across one side", box_solid(&Frame { origin: v3(2.0, -10.0, 5.0), ..Frame::WORLD }, 20.0, 20.0, 8.0, 2).unwrap()),
        ("box on a slant", box_solid(&Frame::new(v3(0.0, 0.0, 10.0), v3(0.4, 0.3, 1.0), v3(1.0, 0.0, -0.2)), 8.0, 7.0, 6.0, 2).unwrap()),
        ("sphere through the wall", sphere(&Frame { origin: v3(5.0, 1.0, 9.0), ..Frame::WORLD }, 3.5, 2).unwrap()),
        ("cylinder across", cylinder(&Frame::new(v3(-15.0, 0.5, 10.0), Vec3::X, Vec3::Y), 2.0, 30.0, 2).unwrap()),
        ("torus around it", torus(&Frame { origin: v3(0.0, 0.0, 10.0), ..Frame::WORLD }, 7.0, 2.0, 2).unwrap()),
    ] {
        same_volume(&subtract(&patch, &tool).unwrap(), &subtract(&real, &tool).unwrap(), &format!("subtract {name}"));
        same_volume(&union(&patch, &tool).unwrap(), &union(&real, &tool).unwrap(), &format!("union {name}"));
        same_volume(&intersect(&patch, &tool).unwrap(), &intersect(&real, &tool).unwrap(), &format!("intersect {name}"));
    }
}

#[test]
fn a_tool_can_be_the_patch() {
    let (r, h) = (3.0, 30.0);
    let patch = loft(&[(&circle(r), at(-5.0)), (&circle(r), at(h))], 2).unwrap();
    let real = cylinder(&Frame { origin: v3(0.0, 0.0, -5.0), ..Frame::WORLD }, r, h + 5.0, 2).unwrap();
    let block = box_solid(&Frame { origin: v3(-10.0, -10.0, 0.0), ..Frame::WORLD }, 20.0, 20.0, 12.0, 1).unwrap();
    same_volume(&subtract(&block, &patch).unwrap(), &subtract(&block, &real).unwrap(), "a hole by a patch");
}

#[test]
fn two_patches_meet_like_two_real_surfaces() {
    let (r, h) = (6.0, 24.0);
    let a = loft(&[(&circle(r), at(0.0)), (&circle(r), at(h))], 1).unwrap();
    let across = Frame::new(v3(-15.0, 1.0, 12.0), Vec3::X, Vec3::Y);
    let top = Frame { origin: across.origin + across.z * 30.0, ..across };
    let b = loft(&[(&circle(3.0), across), (&circle(3.0), top)], 2).unwrap();
    let (ra, rb) = (cylinder(&Frame::WORLD, r, h, 1).unwrap(), cylinder(&across, 3.0, 30.0, 2).unwrap());
    same_volume(&subtract(&a, &b).unwrap(), &subtract(&ra, &rb).unwrap(), "subtract");
    same_volume(&union(&a, &b).unwrap(), &union(&ra, &rb).unwrap(), "union");
    same_volume(&intersect(&a, &b).unwrap(), &intersect(&ra, &rb).unwrap(), "intersect");
}

#[test]
fn random_cuts_on_exact_patches_match_the_real_surfaces() {
    // Every patch here is exactly a real cylinder or cone, written as rational splines, in any position. So any
    // tool has to give the same volume through either one, which needs no other system to check against.
    let mut rng = Rng::new(2026);
    let (mut compared, mut refused) = (0, 0);
    for _ in 0..24 {
        let f = Frame::new(v3(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0), rng.range(-2.0, 2.0)), rng.unit_vector(), rng.unit_vector());
        let (r0, r1, h) = (rng.range(3.0, 7.0), rng.range(3.0, 7.0), rng.range(6.0, 14.0));
        let top = Frame { origin: f.origin + f.z * h, ..f };
        let patch = loft(&[(&circle(r0), f), (&circle(r1), top)], 1).unwrap();
        let real = cone_frustum(&f, r0, r1, h, 1).or_else(|_| cylinder(&f, r0, h, 1)).unwrap();
        let center = f.origin + f.z * (h / 2.0) + rng.unit_vector() * rng.range(2.0, 6.0);
        let frame = Frame::new(center, rng.unit_vector(), rng.unit_vector());
        let tool = match rng.int(0, 3) {
            0 => box_solid(&frame, rng.range(2.0, 9.0), rng.range(2.0, 9.0), rng.range(2.0, 9.0), 2),
            1 => sphere(&frame, rng.range(1.5, 5.0), 2),
            2 => cylinder(&frame, rng.range(1.0, 4.0), rng.range(4.0, 16.0), 2),
            _ => torus(&frame, rng.range(3.0, 6.0), rng.range(0.7, 2.0), 2),
        }
        .unwrap();
        for op in [BoolOp::Union, BoolOp::Subtract, BoolOp::Intersect] {
            let (a, b) = (boolean::boolean(&patch, &tool, op), boolean::boolean(&real, &tool, op));
            match (a, b) {
                (Ok(a), Ok(b)) => {
                    let (x, y) = (volume(&a), volume(&b));
                    assert!((x - y).abs() < 1e-5 * y.abs().max(1.0), "{op:?}: on the patch {x}, on the real surface {y}");
                    compared += 1;
                }
                // Tangencies are refused, and the two routes don't have to refuse the same ones.
                _ => refused += 1,
            }
        }
    }
    assert!(compared >= 60, "{compared} compared, {refused} refused");
}

#[test]
fn volumes_add_up_on_a_truly_free_form_shape() {
    // No analytic twin here, so the identities stand in: union + intersection = A + B, and A - B = A - (A and B).
    let a = loft(&[(&square(12.0), at(0.0)), (&circle(5.0), at(7.0)), (&square(7.0), at(14.0))], 1).unwrap();
    let tool = sphere(&Frame { origin: v3(4.0, 3.0, 8.0), ..Frame::WORLD }, 5.0, 2).unwrap();
    let (u, i, s) = (union(&a, &tool).unwrap(), intersect(&a, &tool).unwrap(), subtract(&a, &tool).unwrap());
    let (va, vb) = (check(&a), check(&tool));
    let (vu, vi, vs) = (check(&u), check(&i), check(&s));
    assert!((vu + vi - va - vb).abs() < 1e-5 * va, "{vu} + {vi} against {va} + {vb}");
    assert!((vs + vi - va).abs() < 1e-5 * va, "{vs} + {vi} against {va}");
    assert!(vi > 0.0 && vs > 0.0);
}

#[test]
fn a_face_that_bulges_past_its_edges_is_inside_its_box() {
    // Through a wide middle section, a side bulges well past both of its ends. Everything of the shape has to
    // be inside the body's box, and a point on that bulge has to count as on the face.
    let b = loft(&[(&circle(3.0), at(0.0)), (&circle(12.0), at(10.0)), (&circle(3.0), at(20.0))], 1).unwrap();
    let bounds = b.aabb();
    let index = coreshape::boolean::classify::Index::new(&b);
    let mesh = tessellate(&b, &TessParams::for_size(10.0)).unwrap();
    for q in &mesh.positions {
        assert!(bounds.inflated(1e-9).contains(*q), "{q:?} is outside {bounds:?}");
    }
    let on = v3(12.0, 0.0, 10.0);
    assert!(index.locate_in_body(v3(11.0, 0.0, 10.0)) == coreshape::boolean::classify::BodyLoc::Inside);
    assert!(index.locate_in_body(v3(13.0, 0.0, 10.0)) == coreshape::boolean::classify::BodyLoc::Outside);
    assert!(matches!(index.locate_in_body(on), coreshape::boolean::classify::BodyLoc::OnFace(_)));
}

#[test]
fn it_can_be_moved_and_mirrored() {
    let a = loft(&[(&square(10.0), at(0.0)), (&circle(4.0), at(8.0))], 1).unwrap();
    let v = check(&a);
    let moved = a.transformed(&Frame::new(v3(30.0, 5.0, -2.0), v3(0.2, 0.5, 1.0), Vec3::X));
    assert!((check(&moved) - v).abs() < 1e-8 * v);
    let m = Mirror::new(v3(20.0, 0.0, 0.0), v3(1.0, 0.2, 0.1));
    let image = a.mirrored(&m);
    assert!((check(&image) - v).abs() < 1e-8 * v, "{}", volume(&image));
    // A Boolean through the image behaves the same as through the original.
    let tool = sphere(&Frame { origin: v3(4.0, 3.0, 4.0), ..Frame::WORLD }, 3.0, 2).unwrap();
    let image_tool = tool.mirrored(&m);
    let (x, y) = (subtract(&a, &tool).unwrap(), subtract(&image, &image_tool).unwrap());
    assert!((volume(&x) - volume(&y)).abs() < 1e-7 * v, "{} {}", volume(&x), volume(&y));
}

#[test]
fn a_loft_comes_back_from_its_own_step_file() {
    let shapes = [
        loft(&[(&circle(6.0), at(0.0)), (&circle(6.0), at(12.0))], 1).unwrap(),
        loft(&[(&square(12.0), at(0.0)), (&circle(5.0), at(7.0)), (&square(7.0), at(14.0))], 1).unwrap(),
        subtract(
            &loft(&[(&square(12.0), at(0.0)), (&circle(5.0), at(7.0)), (&square(7.0), at(14.0))], 1).unwrap(),
            &sphere(&Frame { origin: v3(4.0, 3.0, 8.0), ..Frame::WORLD }, 5.0, 2).unwrap(),
        )
        .unwrap(),
    ];
    for b in shapes {
        let text = export::step::write_step(&b, "loft");
        let back = import::step::read_step(&text, 1).unwrap();
        assert_eq!(back.len(), 1);
        let (x, y) = (check(&b), check(&back[0]));
        assert!((x - y).abs() < 1e-5 * x, "{x} came back as {y}");
        assert_eq!(free_faces(&b), free_faces(&back[0]));
        // And it can still be cut.
        let tool = box_solid(&Frame { origin: v3(-1.0, -1.0, 2.0), ..Frame::WORLD }, 30.0, 30.0, 3.0, 5).unwrap();
        let (a, c) = (subtract(&b, &tool).unwrap(), subtract(&back[0], &tool).unwrap());
        assert!((volume(&a) - volume(&c)).abs() < 1e-5 * x);
    }
}

#[test]
fn a_loft_that_would_pass_through_itself_is_refused() {
    // One section through another.
    let across = Frame::new(v3(0.0, 0.0, 1.0), Vec3::X, Vec3::Y);
    let why = loft(&[(&circle(5.0), at(0.0)), (&circle(5.0), across)], 1).err().map(|e| e.to_string()).unwrap_or_default();
    assert!(why.contains("pass through each other"), "{why}");
    // A bend, whose planes meet away from the outlines, is fine.
    let bent = Frame::new(v3(20.0, 0.0, 20.0), Vec3::X, Vec3::Y);
    assert!(loft(&[(&circle(5.0), at(0.0)), (&circle(5.0), bent)], 1).is_ok());
    // A hole whose path takes it through the outside wall.
    let ring = |r: f64, x: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r)).with_hole(Wire::circle(v2(x, 0.0), 1.0));
    let why = loft(&[(&ring(8.0, 6.5), at(0.0)), (&ring(1.5, 0.0), at(5.0)), (&ring(8.0, -6.5), at(10.0))], 1)
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(why.contains("cross"), "{why}");
}

#[test]
fn a_patch_lying_on_a_different_surface_is_refused_and_not_guessed_at() {
    // The patch is exactly this cylinder, but nothing tells the kernel so, and the surfaces touch everywhere.
    let patch = loft(&[(&circle(6.0), at(0.0)), (&circle(6.0), at(20.0))], 1).unwrap();
    let real = cylinder(&Frame { origin: v3(0.0, 0.0, 5.0), ..Frame::WORLD }, 6.0, 30.0, 2).unwrap();
    for op in [BoolOp::Union, BoolOp::Subtract, BoolOp::Intersect] {
        assert!(boolean::boolean(&patch, &real, op).is_err(), "{op:?}");
    }
    // Two copies of one patch, which it can recognise, are fine.
    let again = patch.clone();
    let both = boolean::boolean(&patch, &again, BoolOp::Union).unwrap();
    assert!((volume(&both) - volume(&patch)).abs() < 1e-6);
}

#[test]
fn refusals_name_what_is_wrong() {
    let one = loft(&[(&circle(3.0), at(0.0))], 1).err().unwrap().to_string();
    assert!(one.contains("at least two"), "{one}");
    let same = loft(&[(&circle(3.0), at(0.0)), (&circle(5.0), at(0.0))], 1).err().unwrap().to_string();
    assert!(same.contains("one place"), "{same}");
    let holes = loft(&[(&circle_with_hole(5.0, 2.0), at(0.0)), (&circle(5.0), at(4.0))], 1).err().unwrap().to_string();
    assert!(holes.contains("holes"), "{holes}");
}
