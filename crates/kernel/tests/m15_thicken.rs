//! Thickening faces, and offsets of free-form faces, against shapes worked out by hand. A free-form face that's really a
//! cylinder, sphere or torus written as a patch has to come out the same as the true one, which checks the moved surface
//! against exact answers.

use coreshape::boolean::{subtract, union};
use coreshape::profile::{Profile, Wire};
use coreshape::spline::Spline;
use coreshape::surface::Surface;
use coreshape::*;
use std::f64::consts::PI;

fn z_at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: v3(x, y, z), ..Frame::WORLD }
}

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} against {b}");
}

fn volume(b: &Body) -> f64 {
    validate(b).unwrap();
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    let v = mass_properties(b).volume;
    assert!((mesh.volume() - v).abs() < 0.02 * v.abs(), "mesh {} against {v}", mesh.volume());
    v
}

fn faces_where(b: &Body, f: impl Fn(&Surface) -> bool) -> Vec<usize> {
    (0..b.faces.len()).filter(|&i| f(&b.faces[i].surface)).collect()
}

#[test]
fn a_flat_face_outward_and_inward() {
    let b = box_solid(&Frame::WORLD, 30.0, 20.0, 10.0, 1).unwrap();
    let top = (0..6).find(|&f| b.faces[f].outward_normal(v3(15.0, 10.0, 10.0)).z > 0.9 && b.faces[f].surface.sdf(v3(15.0, 10.0, 10.0)).abs() < 1e-9).unwrap();
    for d in [2.0, -2.0] {
        let t = thicken(&b, &[top], d, 5).unwrap();
        close(volume(&t), 600.0 * 2.0, 1e-9, "flat");
        assert_eq!(t.faces.len(), 6);
        let added = if d > 0.0 { union(&b, &t).unwrap() } else { subtract(&b, &t).unwrap() };
        close(volume(&added), 6000.0 + d * 600.0, 1e-9, "added to the block");
    }
    // Keeps what it came from, for telling faces apart.
    let t = thicken(&b, &[top], 2.0, 5).unwrap();
    assert!(t.faces.iter().all(|f| f.tag.feature == 5 && !f.origin.between.is_empty()));
}

#[test]
fn a_flat_face_with_a_hole_and_round_corners() {
    let plate = subtract(&box_solid(&Frame::WORLD, 30.0, 20.0, 10.0, 1).unwrap(), &cylinder(&z_at(15.0, 10.0, -1.0), 4.0, 12.0, 2).unwrap()).unwrap();
    let top: Vec<usize> = (0..plate.faces.len()).filter(|&f| matches!(plate.faces[f].surface, Surface::Plane(_)) && plate.faces[f].surface.sdf(v3(1.0, 1.0, 10.0)).abs() < 1e-9).collect();
    assert_eq!(top.len(), 1);
    let t = thicken(&plate, &top, 3.0, 5).unwrap();
    close(volume(&t), (600.0 - PI * 16.0) * 3.0, 1e-9, "flat with a hole");
    // Each side of a rounded hole wall is a cylinder's own.
    assert!(t.faces.iter().any(|f| matches!(f.surface, Surface::Cylinder { .. })));
}

#[test]
fn a_cylinder_wall_either_way() {
    let c = cylinder(&Frame::WORLD, 10.0, 20.0, 1).unwrap();
    let sides = faces_where(&c, |s| matches!(s, Surface::Cylinder { .. }));
    assert_eq!(sides.len(), 2);
    let out = thicken(&c, &sides, 2.0, 5).unwrap();
    close(volume(&out), PI * (144.0 - 100.0) * 20.0, 1e-9, "tube outward");
    let inward = thicken(&c, &sides, -2.0, 5).unwrap();
    close(volume(&inward), PI * (100.0 - 64.0) * 20.0, 1e-9, "tube inward");
    let half = thicken(&c, &sides[..1], 2.0, 5).unwrap();
    close(volume(&half), PI * (144.0 - 100.0) * 10.0, 1e-9, "half a tube");
    assert!(thicken(&c, &sides, -11.0, 5).is_err());
}

#[test]
fn a_sphere_a_cone_and_a_torus() {
    let s = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    let all: Vec<usize> = (0..s.faces.len()).collect();
    let shell = thicken(&s, &all, 1.0, 5).unwrap();
    close(volume(&shell), 4.0 / 3.0 * PI * (1331.0 - 1000.0), 1e-7, "sphere");
    assert_eq!(shell.shells.len(), 2);
    let inward = thicken(&s, &all, -1.0, 5).unwrap();
    close(volume(&inward), 4.0 / 3.0 * PI * (1000.0 - 729.0), 1e-7, "sphere inward");

    let c = cone_frustum(&Frame::WORLD, 10.0, 5.0, 12.0, 1).unwrap();
    let sides = faces_where(&c, |s| matches!(s, Surface::Cone { .. }));
    let thick = thicken(&c, &sides, 1.0, 5).unwrap();
    // A cone's normals make a frustum of a different shape, by Steiner's formula for a surface that bends one way.
    let slant = (25.0f64 + 144.0).sqrt();
    let area = PI * (10.0 + 5.0) * slant;
    let bend = PI * slant * 1.0 * (12.0 / slant) * 2.0 / 2.0;
    close(volume(&thick), area * 1.0 + bend * 1.0, 1e-6, "cone");

    let t = torus(&Frame::WORLD, 20.0, 5.0, 1).unwrap();
    let all: Vec<usize> = (0..t.faces.len()).collect();
    let tube = thicken(&t, &all, 1.0, 5).unwrap();
    close(volume(&tube), 2.0 * PI * PI * 20.0 * (36.0 - 25.0), 1e-7, "torus");
}

/// A cylinder, as a loft of circles, which is a rational free-form patch of the same shape.
fn patch_cylinder(r: f64, h: f64) -> Body {
    let p = Profile::new(Wire::circle(v2(0.0, 0.0), r));
    loft(&[(&p, z_at(0.0, 0.0, 0.0)), (&p, z_at(0.0, 0.0, h))], 1).unwrap()
}

#[test]
fn a_free_form_face_moved_is_the_true_one_moved() {
    let b = patch_cylinder(10.0, 20.0);
    let sides = faces_where(&b, |s| matches!(s, Surface::Free { .. }));
    assert!(!sides.is_empty());
    let out = thicken(&b, &sides, 2.0, 5).unwrap();
    close(volume(&out), PI * (144.0 - 100.0) * 20.0, 1e-9, "free-form tube outward");
    let inward = thicken(&b, &sides, -2.0, 5).unwrap();
    close(volume(&inward), PI * (100.0 - 64.0) * 20.0, 1e-9, "free-form tube inward");
    // Whole-body offset and shell on the same thing.
    close(volume(&offset(&b, 1.5).unwrap()), PI * 11.5 * 11.5 * 23.0, 1e-9, "free-form offset");
    close(volume(&offset(&b, -1.5).unwrap()), PI * 8.5 * 8.5 * 17.0, 1e-9, "free-form offset inward");
    let caps = faces_where(&b, |s| matches!(s, Surface::Plane(_)));
    let top = caps.iter().copied().find(|&f| b.faces[f].surface.sdf(v3(0.0, 0.0, 20.0)).abs() < 1e-9).unwrap();
    let tray = shell(&b, &[top], 1.0, 5).unwrap();
    close(volume(&tray), PI * 100.0 * 20.0 - PI * 81.0 * 19.0, 1e-9, "free-form shell");
    // Further than the radius would fold it.
    assert!(offset(&b, -10.5).is_err());
}

#[test]
fn a_surface_that_really_bends_both_ways() {
    // A vase. By Steiner's formula a thin layer's volume is its area times its thickness, to first order, and on a closed
    // surface of revolution everything through the third order is given by the integrals of curvature, so this checks
    // against a much thinner layer instead: volume over thickness settles to the area as the thickness goes to nothing.
    let circle = |r: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r));
    let (a, b, c) = (circle(10.0), circle(7.0), circle(9.0));
    let vase = loft(&[(&a, z_at(0.0, 0.0, 0.0)), (&b, z_at(0.0, 0.0, 12.0)), (&c, z_at(0.0, 0.0, 24.0))], 1).unwrap();
    let sides = faces_where(&vase, |s| matches!(s, Surface::Free { .. }));
    let area: f64 = {
        // The area of those faces alone: the whole surface less the caps.
        mass_properties(&vase).area - PI * 100.0 - PI * 81.0
    };
    let layer = thicken(&vase, &sides, 1e-3, 5).unwrap();
    close(volume(&layer) / 1e-3, area, 1e-3, "a thin layer of a vase");
    let thick = thicken(&vase, &sides, 0.5, 5).unwrap();
    let v = volume(&thick);
    assert!(v > 0.45 * area && v < 0.6 * area, "{v} against {area}");
    // Offset the vase, and the new one holds the old.
    let bigger = offset(&vase, 0.5).unwrap();
    close(volume(&bigger), mass_properties(&vase).volume + 0.5 * mass_properties(&vase).area, 0.03, "vase offset");
    let cup = shell(&vase, &[], 0.5, 5);
    assert!(cup.is_ok(), "{cup:?}");
    // A STEP file carries it as an ordinary B-spline, fitted, since that's what all readers can take.
    let text = export::step::write_step(&bigger, "vase");
    assert!(!text.contains("OFFSET_SURFACE"));
    let back = import::step::read_step(&text, 1).unwrap();
    validate(&back[0]).unwrap();
    close(mass_properties(&back[0]).volume, mass_properties(&bigger).volume, 1e-6, "after a round trip");
    // And this kernel reads OFFSET_SURFACE from anybody else's file, around a plane or a smooth B-spline.
    let plain = export::step::write_step(&vase, "vase");
    let by = 0.25;
    let mut moved = String::new();
    let mut extra = 90000;
    for line in plain.lines() {
        if line.contains("B_SPLINE_SURFACE") {
            let (id, rest) = line.split_once('=').unwrap();
            let new = format!("#{extra}");
            moved.push_str(&format!("{new}={rest}\n{id}=OFFSET_SURFACE('',{new},{by},.F.);\n"));
            extra += 1;
        } else {
            moved.push_str(line);
            moved.push('\n');
        }
    }
    assert!(moved.contains("OFFSET_SURFACE"));
    let read = import::step::read_step(&moved, 1);
    // Its edges in that file are where they were, a quarter away from what's now its surface, so this has to say so.
    assert!(read.is_err() || read.unwrap()[0].tolerance >= 0.2);
}

#[test]
fn a_swept_spline_face() {
    // A spline outline extruded: the sides become free-form surfaces moved, and the far side keeps the height.
    let curve = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 20.0, 10.0, 0.0, 2.0 * PI).unwrap();
    let p = Profile::new(profile::PathBuilder::new(v2(20.0, 0.0)).spline_to(curve).close());
    let e = extrude(&p, &Frame::WORLD, 15.0, 1).unwrap();
    let sides = faces_where(&e, |s| matches!(s, Surface::Extruded { .. }));
    assert!(!sides.is_empty());
    let t = thicken(&e, &sides, 1.0, 5).unwrap();
    // An outline's parallel curve encloses its area, plus its length times the distance, plus pi times the square of that.
    let flat_area = PI * 200.0;
    let length = mass_properties(&e).area - 2.0 * flat_area;
    let length = length / 15.0;
    close(volume(&t), 15.0 * (length * 1.0 + PI), 1e-6, "swept spline wall");
    close(volume(&offset(&e, 1.0).unwrap()), 17.0 * (flat_area + length + PI), 1e-6, "swept spline offset");
}

#[test]
fn walls_on_a_curved_edge_are_fitted_to_within_the_tolerance() {
    // A cylinder cut at a slant, whose top is flat with an elliptical edge, and whose side has it too.
    let c = cylinder(&Frame::WORLD, 10.0, 40.0, 1).unwrap();
    let f = Frame::rotation(v3(0.0, 0.0, 25.0), Vec3::X, 0.5).compose(&Frame::translation(v3(-50.0, -50.0, 25.0)));
    let b = subtract(&c, &box_solid(&f, 100.0, 100.0, 50.0, 2).unwrap()).unwrap();
    let slant = (0..b.faces.len()).find(|&i| b.faces[i].tag.feature == 2).unwrap();
    let up = thicken(&b, &[slant], 2.0, 5).unwrap();
    // The flat ellipse's area, times the height.
    close(volume(&up), PI * 100.0 / 0.5f64.cos() * 2.0, 1e-5, "ellipse lifted");
    let sides = faces_where(&b, |s| matches!(s, Surface::Cylinder { .. }));
    let out = thicken(&b, &sides, 2.0, 5).unwrap();
    // Its volume is what lies between the radii, up to the plane's height on each. The cut's height averages out to 25.
    close(volume(&out), PI * (144.0 - 100.0) * 25.0, 1e-5, "side of a slanted cylinder");
}

#[test]
fn what_it_refuses() {
    let b = box_solid(&Frame::WORLD, 30.0, 20.0, 10.0, 1).unwrap();
    assert!(thicken(&b, &[], 1.0, 5).is_err());
    assert!(thicken(&b, &[99], 1.0, 5).is_err());
    assert!(thicken(&b, &[0], 0.0, 5).is_err());
    let why = thicken(&b, &[0, 1], 1.0, 5).unwrap_err().to_string();
    assert!(why.contains("sharp"), "{why}");
}
