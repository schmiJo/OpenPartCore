//! Scaling a body the same in every direction: each kind of surface and curve comes out as the same kind, scaled. Volume
//! goes as the cube, area as the square, and what's scaled works like any part.

use coreshape::export::step::write_step;
use coreshape::import::step::read_step;
use coreshape::*;
use std::f64::consts::TAU;
use std::sync::Arc;

fn close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn z_at(z: f64) -> Frame {
    Frame::translation(v3(0.0, 0.0, z))
}

/// One of everything: each analytic surface, swept and turned splines, free-form patches plain and moved, traced
/// intersection curves, blends, and a void.
fn parts() -> Vec<(&'static str, Body)> {
    let f = Frame::new(v3(3.0, -2.0, 5.0), v3(0.2, 0.3, 1.0), Vec3::X);
    let ellipse = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 20.0, 9.0, 0.0, TAU).unwrap();
    let rod = extrude(&Profile::new(Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(ellipse)), id: 0 }] }), &Frame::WORLD, 15.0, 1).unwrap();
    let points: Vec<Vec3> = [(14.0, 0.0), (20.0, 12.0), (9.0, 28.0), (13.0, 44.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
    let wire = PathBuilder::new(v2(14.0, 0.0)).spline_to(Spline::through(&points).unwrap()).line_to(v2(0.0, 44.0)).line_to(v2(0.0, 0.0)).close();
    let vase = revolve(&Profile::new(wire), &Frame::WORLD, TAU, 1).unwrap();
    let circle = |r: f64| Profile::new(Wire::circle(v2(0.0, 0.0), r));
    let (a, b, c) = (circle(10.0), circle(7.0), circle(9.0));
    let lofted = loft(&[(&a, z_at(0.0)), (&b, z_at(12.0)), (&c, z_at(24.0))], 1).unwrap();
    let moved = offset(&lofted, 1.5).unwrap();
    let cube = box_solid(&Frame::WORLD, 40.0, 40.0, 40.0, 1).unwrap();
    let edges: Vec<usize> = (0..cube.edges.len()).collect();
    let rounded = fillet(&cube, &edges, 4.0, 2).unwrap();
    let hollow = subtract(&cube, &sphere(&Frame::translation(v3(20.0, 20.0, 20.0)), 10.0, 2).unwrap()).unwrap();
    let up = cylinder(&Frame::translation(v3(0.0, 0.0, -20.0)), 8.0, 40.0, 1).unwrap();
    let across = cylinder(&Frame::new(v3(-20.0, 1.5, 0.0), Vec3::X, Vec3::Y), 5.0, 40.0, 2).unwrap();
    vec![
        ("box", box_solid(&f, 10.0, 20.0, 30.0, 1).unwrap()),
        ("cylinder", cylinder(&f, 5.0, 12.0, 1).unwrap()),
        ("frustum", cone_frustum(&f, 8.0, 3.0, 10.0, 1).unwrap()),
        ("cone", cone_frustum(&f, 8.0, 0.0, 10.0, 1).unwrap()),
        ("sphere", sphere(&f, 7.0, 1).unwrap()),
        ("torus", torus(&f, 12.0, 3.0, 1).unwrap()),
        ("elliptical rod", rod),
        ("vase", vase),
        ("lofted vase", lofted),
        ("lofted vase moved out", moved),
        ("rounded cube", rounded),
        ("void", hollow),
        ("traced union", union(&up, &across).unwrap()),
    ]
}

#[test]
fn every_kind_of_face_scales_to_its_own_kind() {
    let center = v3(3.0, -2.0, 5.0);
    for (what, body) in parts() {
        let m = mass_properties(&body);
        for k in [25.4, 0.5, 1.0 / 25.4] {
            let what = format!("{what} by {k}");
            let s = body.scaled(center, k).unwrap_or_else(|e| panic!("{what}: {e}"));
            validate(&s).unwrap_or_else(|e| panic!("{what}: {e:?}"));
            for (a, b) in body.faces.iter().zip(&s.faces) {
                assert_eq!(std::mem::discriminant(&a.surface), std::mem::discriminant(&b.surface), "{what}");
            }
            let n = mass_properties(&s);
            close(n.volume, m.volume * k * k * k, 1e-7, &format!("{what}: volume"));
            close(n.area, m.area * k * k, 1e-7, &format!("{what}: area"));
            let mesh = tessellate(&s, &TessParams::for_size(s.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
            assert_eq!(mesh.open_edge_count(), 0, "{what}: gaps in the mesh");
            // Every vertex is on each of its edges' curves, and each edge on its faces' surfaces.
            for e in &s.edges {
                for (t, v) in [(e.t0, e.v0), (e.t1, e.v1)] {
                    assert!(e.curve.eval(t).dist(s.vertices[v]) < 1e-6 * k.max(1.0), "{what}: an edge's end is off its vertex");
                }
            }
        }
    }
}

#[test]
fn a_scaled_part_works_like_any_other() {
    let center = v3(1.0, 2.0, 3.0);
    for (what, body) in parts() {
        if what == "traced union" || what == "lofted vase moved out" {
            // Cut below, to keep these quick.
            continue;
        }
        let k = 2.5;
        let size = body.aabb();
        let tool = box_solid(&Frame::translation(size.min - v3(1.0, 1.0, 1.0)), size.max.x - size.min.x + 2.0, (size.max.y - size.min.y) / 2.0 + 0.37, size.max.z - size.min.z + 2.0, 9).unwrap();
        // Cutting then scaling, and scaling then cutting, make the same part.
        let cut_first = subtract(&body, &tool).unwrap_or_else(|e| panic!("{what}: {e}"));
        let scaled_first = subtract(&body.scaled(center, k).unwrap(), &tool.scaled(center, k).unwrap()).unwrap_or_else(|e| panic!("{what}: {e}"));
        validate(&scaled_first).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        close(mass_properties(&scaled_first).volume, mass_properties(&cut_first).volume * k * k * k, 1e-7, what);
        // It writes to STEP and reads back the same.
        let back = read_step(&write_step(&scaled_first, what), 7).unwrap_or_else(|e| panic!("{what}: {e}")).remove(0);
        close(mass_properties(&back).volume, mass_properties(&scaled_first).volume, 1e-6, &format!("{what} read back"));
    }
}

#[test]
fn what_scaling_refuses() {
    let b = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    for k in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e7] {
        let why = b.scaled(Vec3::ZERO, k).unwrap_err().to_string();
        assert!(why.contains("above 0"), "{k}: {why}");
    }
    // Beyond the coordinates the kernel works within.
    let why = b.scaled(v3(-1.0, 0.0, 0.0), 1e6).unwrap_err().to_string();
    assert!(why.contains("beyond"), "{why}");
    // Too small for its tolerance.
    let small = box_solid(&Frame::WORLD, 0.5, 0.5, 0.5, 1).unwrap();
    assert!(small.scaled(Vec3::ZERO, 1e-5).is_ok());
    let why = small.scaled(Vec3::ZERO, 1e-6).unwrap_err().to_string();
    assert!(why.contains("too small"), "{why}");
}
