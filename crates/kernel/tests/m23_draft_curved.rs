//! Draft on faces extruded or turned from splines, and on free-form faces, against closed forms. A drafted wall's level
//! cut is its outline moved in by its height times the angle's tangent, and for a smooth convex outline that's an area
//! of A - P d + pi d^2 (Steiner), so a drafted prism's volume is known exactly. A turned wall drafted about its axis is a
//! cone, and so is a loft between equal circles, which is a cylinder written as a free-form patch.

use coreshape::surface::Surface;
use coreshape::*;
use coreshape::profile::{PathBuilder, Profile};
use coreshape::spline::Spline;
use coreshape::loft::{loft_through, LoftOptions, Section};
use std::f64::consts::PI;

fn check(b: &Body) -> f64 {
    validate(b).unwrap();
    let props = mass_properties(b);
    let mesh = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap();
    assert_eq!(mesh.open_edge_count(), 0);
    assert!((mesh.volume() - props.volume).abs() < 0.01 * props.volume, "mesh {}, integrated {}", mesh.volume(), props.volume);
    props.volume
}

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!((a - b).abs() <= rel * b.abs(), "{what}: {a} against {b}, off by {:e}", ((a - b) / b).abs());
}

fn ellipse(a: f64, b: f64) -> Profile {
    let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, a, b, 0.0, 2.0 * PI).unwrap();
    Profile::new(PathBuilder::new(v2(a, 0.0)).spline_to(c).close())
}

fn perimeter(a: f64, b: f64) -> f64 {
    let n = 20000;
    let f = |t: f64| (a * a * t.sin().powi(2) + b * b * t.cos().powi(2)).sqrt();
    let h = 2.0 * PI / n as f64;
    (0..n).map(|k| (f(k as f64 * h) + 4.0 * f((k as f64 + 0.5) * h) + f((k + 1) as f64 * h)) * h / 6.0).sum()
}

fn faces_where(b: &Body, test: impl Fn(&Surface) -> bool) -> Vec<usize> {
    (0..b.faces.len()).filter(|&f| test(&b.faces[f].surface)).collect()
}

#[test]
fn a_drafted_elliptical_prism_has_steiners_volume() {
    let (a, b, h) = (20.0, 12.0, 30.0);
    let prism = extrude(&ellipse(a, b), &Frame::WORLD, h, 1).unwrap();
    let walls = faces_where(&prism, |s| matches!(s, Surface::Extruded { .. }));
    assert!(!walls.is_empty());
    let p = perimeter(a, b);
    for degrees in [3.0f64, -2.0] {
        let t = degrees.to_radians().tan();
        let drafted = draft(&prism, &walls, Vec3::Z, Vec3::ZERO, degrees.to_radians()).unwrap();
        // Integral over the height of A - P (z t) + pi (z t)^2.
        let want = PI * a * b * h - p * t * h * h / 2.0 + PI * t * t * h * h * h / 3.0;
        close(check(&drafted), want, 1e-7, &format!("{degrees} degrees"));
        assert!(drafted.faces.iter().all(|f| !matches!(f.surface, Surface::Extruded { .. })));
    }
    // About a neutral plane halfway up, the bottom grows and the top shrinks by the same, and the volume barely changes:
    // only the pi d^2 term is left.
    let t = 4f64.to_radians().tan();
    let drafted = draft(&prism, &walls, Vec3::Z, v3(0.0, 0.0, h / 2.0), 4f64.to_radians()).unwrap();
    let want = PI * a * b * h + PI * t * t * 2.0 * (h / 2.0).powi(3) / 3.0;
    close(check(&drafted), want, 1e-7, "about the middle");
    // Not along the way it was extruded, it's refused, saying so.
    let why = draft(&prism, &walls, Vec3::X, Vec3::ZERO, 0.05).unwrap_err().to_string();
    assert!(why.contains("extruded"), "{why}");
}

#[test]
fn a_drafted_turned_wall_is_a_cone() {
    // A wall turned from a spline, from radius 10 at the bottom out and back to 8 at the top, 30 tall, round the z axis,
    // which the sketch's y becomes.
    let wall = Spline::through(&[v3(10.0, 0.0, 0.0), v3(11.5, 12.0, 0.0), v3(8.0, 30.0, 0.0)]).unwrap();
    let outline = Profile::new(PathBuilder::new(v2(0.0, 0.0)).line_to(v2(10.0, 0.0)).spline_to(wall).line_to(v2(0.0, 30.0)).close());
    let vase = revolve(&outline, &Frame::WORLD, 2.0 * PI, 1).unwrap();
    let walls = faces_where(&vase, |s| matches!(s, Surface::Revolved { .. }));
    assert_eq!(walls.len(), 2, "two halves");
    let angle = 5f64.to_radians();
    // About the bottom, where it's 10 round: a frustum narrowing to 10 - 30 tan 5 at the top.
    let drafted = draft(&vase, &walls, Vec3::Z, Vec3::ZERO, angle).unwrap();
    let (r0, r1) = (10.0, 10.0 - 30.0 * angle.tan());
    close(check(&drafted), PI * 30.0 * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0, 1e-7, "frustum");
    // About halfway up, where it crosses at the spline's radius there.
    let drafted = draft(&vase, &walls, Vec3::Z, v3(0.0, 0.0, 15.0), angle).unwrap();
    check(&drafted);
}

#[test]
fn a_drafted_free_form_cylinder_is_a_cone() {
    // A loft between two equal circles is a cylinder written as a free-form patch.
    let c = Profile::new(coreshape::profile::Wire::circle(v2(0.0, 0.0), 10.0));
    let top = Frame { origin: v3(0.0, 0.0, 25.0), ..Frame::WORLD };
    let tube = loft_through(&[Section::Outline(&c, Frame::WORLD), Section::Outline(&c, top)], &LoftOptions::default(), 1).unwrap();
    let sides = faces_where(&tube, |s| matches!(s, Surface::Free { .. }));
    assert!(!sides.is_empty());
    let angle = 3f64.to_radians();
    let drafted = draft(&tube, &sides, Vec3::Z, Vec3::ZERO, angle).unwrap();
    let (r0, r1) = (10.0, 10.0 - 25.0 * angle.tan());
    close(check(&drafted), PI * 25.0 * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0, 1e-7, "frustum from a patch");
    // A neutral plane the face doesn't cross is refused, saying why.
    let why = draft(&tube, &sides, Vec3::Z, v3(0.0, 0.0, 40.0), angle).unwrap_err().to_string();
    assert!(why.contains("crosses the neutral plane"), "{why}");
}
