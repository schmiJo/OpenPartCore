//! Stretching along one direction. Each stretched body is held to the map itself, worked out here on its own: every point
//! of every face of the original, moved as the stretch moves points, has to lie on the same face's new surface, and its
//! volume has to be k times what it was, as stretching one way multiplies every volume by k.

use coreshape::*;
use std::f64::consts::PI;

fn moved(p: Vec3, q: Vec3, d: Vec3, k: f64) -> Vec3 {
    let d = d.normalized();
    p + d * ((k - 1.0) * d.dot(p - q))
}

/// Stretches `body` and checks the result against the map, point by point on every face, and by volume.
fn check(body: &Body, q: Vec3, d: Vec3, k: f64, what: &str) -> Body {
    let out = body.stretched(q, d, k).unwrap_or_else(|e| panic!("{what}: {e}"));
    validate(&out).unwrap_or_else(|e| panic!("{what}: {e:?}"));
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal())).unwrap();
    let size = body.aabb().diagonal() * k.max(1.0);
    let mut worst = 0.0f64;
    for (t, tri) in mesh.triangles.iter().enumerate() {
        let f = mesh.face_of_triangle[t] as usize;
        for &v in tri {
            // A mesh point is within the mesh's chord of the surface: its foot on the surface is on it.
            let p = body.faces[f].surface.project(mesh.positions[v as usize]);
            worst = worst.max(out.faces[f].surface.sdf(moved(p, q, d, k)).abs());
        }
    }
    assert!(worst < 1e-8 * size, "{what}: a moved point is {worst:e} off its face's new surface");
    let (before, after) = (mass_properties(body).volume, mass_properties(&out).volume);
    assert!((after - k * before).abs() < 1e-8 * k * before, "{what}: volume {after}, not {k} times {before}");
    let closed = tessellate(&out, &TessParams::for_size(out.aabb().diagonal())).unwrap();
    assert_eq!(closed.open_edge_count(), 0, "{what}: its mesh has gaps");
    out
}

fn kinds(b: &Body) -> Vec<&'static str> {
    b.faces.iter().map(|f| f.surface.name()).collect()
}

fn holed_block() -> Body {
    let p = Profile { outer: Wire::rect(0.0, 0.0, 20.0, 12.0), holes: vec![Wire::circle(v2(6.0, 6.0), 3.0).reversed()] };
    extrude(&p, &Frame::WORLD, 8.0, 1).unwrap()
}

#[test]
fn a_block_with_a_hole_stretched_along_and_across_the_hole() {
    let block = holed_block();
    // Along the hole: it stays a cylinder, and the block gets taller.
    let up = check(&block, Vec3::ZERO, v3(0.0, 0.0, 1.0), 2.5, "along the hole");
    assert!(kinds(&up).contains(&"cylinder"), "{:?}", kinds(&up));
    // Across it, along x, about the plane x = 6 through the hole's axis: the hole is an ellipse 12 across x and 6 across y.
    let wide = check(&block, v3(6.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), 2.0, "across the hole");
    assert!(!kinds(&wide).contains(&"cylinder"), "{:?}", kinds(&wide));
    let v = &wide.vertices;
    let xs = v.iter().map(|p| p.x).fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(x), b.max(x)));
    assert!((xs.0 + 6.0).abs() < 1e-12 && (xs.1 - 34.0).abs() < 1e-12, "{xs:?}");
    assert!((mass_properties(&wide).volume - 2.0 * (240.0 - PI * 9.0) * 8.0).abs() < 1e-8 * 4000.0);
    // Across it along y, where the hole's ellipse has its long axis the other way from its circle's start: its half
    // circles have to start and end where they did, a quarter of the way round the ellipse.
    let tall = check(&block, v3(0.0, 6.0, 0.0), v3(0.0, 1.0, 0.0), 3.0, "across the hole along y");
    assert!((mass_properties(&tall).volume - 3.0 * (240.0 - PI * 9.0) * 8.0).abs() < 1e-8 * 6000.0);
    // At a slant, down a diagonal, shrinking.
    check(&block, v3(3.0, 1.0, 2.0), v3(1.0, 2.0, 2.0), 0.4, "at a slant");
}

#[test]
fn round_solids_stretched() {
    // A ball into a spheroid, 1.5 times as tall: 4/3 pi r^3 * 1.5.
    let ball = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let tall = check(&ball, Vec3::ZERO, v3(0.0, 0.0, 1.0), 1.5, "a ball");
    assert!((mass_properties(&tall).volume - 4.0 / 3.0 * PI * 125.0 * 1.5).abs() < 1e-7 * 785.0);
    // A ring, along and across its axis.
    let ring = torus(&Frame::WORLD, 10.0, 3.0, 1).unwrap();
    check(&ring, Vec3::ZERO, v3(0.0, 0.0, 1.0), 2.0, "a ring along its axis");
    check(&ring, v3(1.0, 2.0, 0.0), v3(1.0, 0.0, 0.0), 0.5, "a ring across its axis");
    // A cone frustum, along its axis, where it stays a cone, and across.
    let cone = cone_frustum(&Frame::WORLD, 6.0, 2.0, 9.0, 1).unwrap();
    let longer = check(&cone, Vec3::ZERO, v3(0.0, 0.0, 1.0), 3.0, "a cone along its axis");
    assert!(kinds(&longer).contains(&"cone"), "{:?}", kinds(&longer));
    check(&cone, Vec3::ZERO, v3(0.0, 1.0, 0.0), 1.7, "a cone across its axis");
    // A cylinder across its axis.
    let can = cylinder(&Frame::WORLD, 4.0, 10.0, 1).unwrap();
    check(&can, Vec3::ZERO, v3(1.0, 1.0, 0.0), 3.0, "a can across");
}

#[test]
fn free_form_and_traced_edges_stretched() {
    // A loft from a square to a circle: free-form sides.
    let square = Profile::new(Wire::rect(-8.0, -8.0, 16.0, 16.0));
    let circle = Profile::new(Wire::circle(v2(0.0, 0.0), 6.0));
    let loft = coreshape::loft::loft(&[(&square, Frame::WORLD), (&circle, Frame::translation(v3(0.0, 0.0, 20.0)))], 1).unwrap();
    check(&loft, Vec3::ZERO, v3(1.0, 0.0, 1.0), 1.8, "a loft");
    // Two rods crossing, their meeting traced: a cylinder through a cylinder.
    let a = cylinder(&Frame::WORLD, 4.0, 20.0, 1).unwrap();
    let b = cylinder(&Frame { origin: v3(-10.0, 0.0, 10.0), x: v3(0.0, 1.0, 0.0), y: v3(0.0, 0.0, 1.0), z: v3(1.0, 0.0, 0.0) }, 3.0, 20.0, 2).unwrap();
    let cross = union(&a, &b).unwrap();
    check(&cross, v3(0.0, 0.0, 10.0), v3(0.0, 1.0, 1.0), 1.6, "crossed rods");
    // The loft turned and moved, so that its patches sit in a frame of their own.
    let turned = loft.transformed(&Frame::rotation(v3(3.0, 0.0, 0.0), v3(1.0, 1.0, 0.0).normalized(), 0.7));
    check(&turned, v3(1.0, 2.0, 3.0), v3(0.0, 0.0, 1.0), 1.3, "a loft turned");
    // A vase turned from a spline, along its axis, where it stays a turned curve, and across.
    let outline = Profile::new(Wire {
        segs: vec![
            Seg { geom: SegGeom::Line { a: v2(0.0, 0.0), b: v2(6.0, 0.0) }, id: 0 },
            Seg { geom: SegGeom::Spline(std::sync::Arc::new(Spline::through(&[v3(6.0, 0.0, 0.0), v3(8.0, 3.0, 0.0), v3(5.0, 6.0, 0.0), v3(3.0, 9.0, 0.0)]).unwrap())), id: 1 },
            Seg { geom: SegGeom::Line { a: v2(3.0, 9.0), b: v2(0.0, 9.0) }, id: 2 },
            Seg { geom: SegGeom::Line { a: v2(0.0, 9.0), b: v2(0.0, 0.0) }, id: 3 },
        ],
    });
    let vase = revolve(&outline, &Frame::WORLD, std::f64::consts::TAU, 1).unwrap();
    assert!(kinds(&vase).contains(&"revolution"), "{:?}", kinds(&vase));
    let taller = check(&vase, Vec3::ZERO, v3(0.0, 0.0, 1.0), 1.4, "a vase along its axis");
    assert!(kinds(&taller).contains(&"revolution"), "{:?}", kinds(&taller));
    check(&vase, v3(0.0, 0.0, 4.0), v3(1.0, 0.0, 0.0), 1.6, "a vase across its axis");
    // A spline extruded.
    let wave = Profile::new(Wire {
        segs: vec![
            Seg { geom: SegGeom::Line { a: v2(0.0, 0.0), b: v2(10.0, 0.0) }, id: 0 },
            Seg { geom: SegGeom::Spline(std::sync::Arc::new(Spline::through(&[v3(10.0, 0.0, 0.0), v3(12.0, 5.0, 0.0), v3(4.0, 9.0, 0.0), v3(0.0, 6.0, 0.0)]).unwrap())), id: 1 },
            Seg { geom: SegGeom::Line { a: v2(0.0, 6.0), b: v2(0.0, 0.0) }, id: 2 },
        ],
    });
    let blob = extrude(&wave, &Frame::WORLD, 5.0, 1).unwrap();
    check(&blob, Vec3::ZERO, v3(0.3, 1.0, 0.2), 2.2, "a spline extruded");
}

#[test]
fn what_a_stretch_refuses() {
    let block = holed_block();
    for k in [0.0, -1.0, 1e7, f64::NAN] {
        assert!(block.stretched(Vec3::ZERO, v3(1.0, 0.0, 0.0), k).is_err(), "{k}");
    }
    assert!(block.stretched(Vec3::ZERO, Vec3::ZERO, 2.0).is_err());
    // A face moved by an offset: a loft between two circles, grown by 0.5.
    let loft = {
        let low = Profile::new(Wire::circle(v2(0.0, 0.0), 10.0));
        let high = Profile::new(Wire::circle(v2(0.0, 0.0), 6.0));
        coreshape::loft::loft(&[(&low, Frame::WORLD), (&high, Frame::translation(v3(0.0, 0.0, 20.0)))], 1).unwrap()
    };
    let grown = offset(&loft, 0.5).unwrap();
    let why = grown.stretched(Vec3::ZERO, v3(1.0, 0.0, 0.0), 2.0).unwrap_err().to_string();
    assert!(why.contains("offset"), "{why}");
    // Along the loft's axis it's refused too, as its free-form sides are moved patches whichever way.
    assert!(grown.stretched(Vec3::ZERO, v3(0.0, 0.0, 1.0), 2.0).is_err());
}
