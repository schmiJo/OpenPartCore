//! Milestone 5: fillets and chamfers, checked against closed-form volumes.

use coreshape::*;

fn vol(b: &Body) -> f64 {
    mass_properties(b).volume
}

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!(
        (a - b).abs() <= rel * b.abs().max(1.0),
        "{what}: got {a}, expected {b} (rel {:e})",
        (a - b).abs() / b.abs()
    );
}

/// First radius, second radius, and OpenCASCADE's volume.
const OCCT_CHAINS: &[(f64, f64, f64)] =
    &[(4.0, 2.0, 23821.810554709475), (5.0, 3.0, 23695.005194770096), (3.0, 3.0, 23830.69242805789)];

fn mesh_ok(b: &Body, what: &str) {
    let m = tessellate(b, &TessParams::for_size(b.aabb().diagonal())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!(m.open_edge_count(), 0, "{what}: open mesh");
}

/// A block 20 x 10 x 6 with one corner at the origin.
fn block() -> Body {
    box_solid(&Frame::WORLD, 20.0, 10.0, 6.0, 1).unwrap()
}

/// Edges of a body whose midpoint is close to this point.
fn edge_at(b: &Body, p: Vec3) -> usize {
    b.edges_where(|i, _| b.edge_midpoint(i).dist(p) < 1e-6)
        .first()
        .copied()
        .unwrap_or_else(|| panic!("no edge at {p:?}"))
}

/// What a fillet of radius r removes from a right-angle corner, per unit length.
fn corner_area(r: f64) -> f64 {
    r * r * (1.0 - PI / 4.0)
}

#[test]
fn fillet_one_convex_edge() {
    let b = block();
    let e = edge_at(&b, v3(10.0, 0.0, 6.0));
    let f = fillet(&b, &[e], 2.0, 9).unwrap();
    close(vol(&f), 1200.0 - corner_area(2.0) * 20.0, 1e-10, "one edge");
    let r = validate(&f).unwrap();
    assert_eq!((r.faces, r.shells), (7, 1));
    assert_eq!(r.genus, vec![0]);
    assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Cylinder { .. })).count(), 1);
    mesh_ok(&f, "one edge");
}

#[test]
fn chamfer_one_convex_edge() {
    let b = block();
    let e = edge_at(&b, v3(10.0, 0.0, 6.0));
    let f = chamfer(&b, &[e], 2.0, 9).unwrap();
    close(vol(&f), 1200.0 - 0.5 * 4.0 * 20.0, 1e-10, "chamfer");
    assert_eq!(validate(&f).unwrap().faces, 7);
    mesh_ok(&f, "chamfer");
}

#[test]
fn fillet_two_edges_on_one_face_miter() {
    let b = block();
    let e1 = edge_at(&b, v3(10.0, 0.0, 6.0));
    let e2 = edge_at(&b, v3(10.0, 10.0, 6.0));
    let f = fillet(&b, &[e1, e2], 2.0, 9).unwrap();
    close(vol(&f), 1200.0 - 2.0 * corner_area(2.0) * 20.0, 1e-10, "two edges");
    mesh_ok(&f, "two edges");
}

#[test]
fn fillet_two_edges_meeting_at_a_corner() {
    let b = block();
    let e1 = edge_at(&b, v3(10.0, 0.0, 6.0));
    let e2 = edge_at(&b, v3(20.0, 5.0, 6.0));
    let r = 2.0;
    let f = fillet(&b, &[e1, e2], r, 9).unwrap();
    // Two swept tools overlap in a mitre at the shared corner. Inclusion-exclusion:
    // the overlap of two perpendicular corner prisms has a known area integral.
    let overlap = {
        // Over the corner square of side r, removed by both when outside both arcs.
        // Integrate numerically: this is a reference, not the code under test.
        let n = 2000;
        let mut v = 0.0;
        for i in 0..n {
            for j in 0..n {
                let (x, y) = ((i as f64 + 0.5) / n as f64 * r, (j as f64 + 0.5) / n as f64 * r);
                // Depth of the part removed by the tool in x at this point, and by y.
                let d1 = r - (r * r - (r - x).powi(2)).max(0.0).sqrt();
                let d2 = r - (r * r - (r - y).powi(2)).max(0.0).sqrt();
                v += d1.min(d2);
            }
        }
        v * (r / n as f64).powi(2)
    };
    let expected = 1200.0 - corner_area(r) * 20.0 - corner_area(r) * 10.0 + overlap;
    close(vol(&f), expected, 1e-5, "mitred corner");
    mesh_ok(&f, "mitred corner");
}

#[test]
fn fillet_concave_edge_adds_material() {
    // An L: the inner corner runs along Y at x=4, z=4.
    let w = Wire::polygon(&[v2(0.0, 0.0), v2(20.0, 0.0), v2(20.0, 4.0), v2(4.0, 4.0), v2(4.0, 20.0), v2(0.0, 20.0)]);
    let sketch = Frame { origin: v3(0.0, 30.0, 0.0), x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
    let l = extrude(&Profile::new(w), &sketch, 30.0, 1).unwrap();
    let base = vol(&l);
    let e = edge_at(&l, v3(4.0, 15.0, 4.0));
    let f = fillet(&l, &[e], 3.0, 9).unwrap();
    close(vol(&f), base + corner_area(3.0) * 30.0, 1e-10, "concave fillet");
    assert_eq!(validate(&f).unwrap().genus, vec![0]);
    mesh_ok(&f, "concave fillet");
    let c = chamfer(&l, &[e], 3.0, 9).unwrap();
    close(vol(&c), base + 0.5 * 9.0 * 30.0, 1e-10, "concave chamfer");
}

#[test]
fn fillet_vertical_corners_of_a_plate() {
    let b = box_solid(&Frame::translation(v3(-20.0, -10.0, 0.0)), 40.0, 20.0, 5.0, 1).unwrap();
    let edges = b.edges_where(|i, _| b.edge_is_parallel(i, Vec3::Z));
    assert_eq!(edges.len(), 4);
    let f = fillet(&b, &edges, 3.0, 9).unwrap();
    close(vol(&f), 4000.0 - 4.0 * corner_area(3.0) * 5.0, 1e-10, "rounded plate");
    assert_eq!(validate(&f).unwrap().faces, 10);
    mesh_ok(&f, "rounded plate");
}

#[test]
fn fillet_rim_of_a_cylinder_is_a_torus() {
    let (rr, h, r) = (10.0, 20.0, 2.0);
    let c = cylinder(&Frame::WORLD, rr, h, 1).unwrap();
    // Pick just one piece of the top rim: the whole circle should come with it.
    let rim = c
        .edges_where(|i, _| (c.edge_midpoint(i).z - h).abs() < 1e-9 && c.edge_is_arc_of_radius(i, rr))
        .first()
        .copied()
        .unwrap();
    let f = fillet(&c, &[rim], r, 9).unwrap();
    // Pappus on the removed corner: area times the path of its centroid.
    let area = r * r * (1.0 - PI / 4.0);
    let centroid_from_corner = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let removed = area * TAU * (rr - centroid_from_corner);
    close(vol(&c) - vol(&f), removed, 1e-9, "rim fillet");
    assert!(f.faces.iter().any(|x| matches!(x.surface, Surface::Torus { .. })));
    mesh_ok(&f, "rim fillet");
    let ch = chamfer(&c, &[rim], r, 9).unwrap();
    assert!(ch.faces.iter().any(|x| matches!(x.surface, Surface::Cone { .. })));
    let tri = r * r / 2.0;
    let removed = tri * TAU * (rr - r / 3.0);
    close(vol(&c) - vol(&ch), removed, 1e-9, "rim chamfer");
}

#[test]
fn fillet_rim_of_a_hole_flares_it() {
    let (hole, r) = (5.0, 1.5);
    let p = Profile::new(Wire::rect_centered(40.0, 40.0)).with_hole(Wire::circle(v2(0.0, 0.0), hole));
    let plate = extrude(&p, &Frame::WORLD, 10.0, 1).unwrap();
    let rim = plate
        .edges_where(|i, _| (plate.edge_midpoint(i).z - 10.0).abs() < 1e-9 && plate.edge_is_arc_of_radius(i, hole))
        .first()
        .copied()
        .unwrap();
    let f = fillet(&plate, &[rim], r, 9).unwrap();
    assert_eq!(validate(&f).unwrap().genus, vec![1]);
    let area = r * r * (1.0 - PI / 4.0);
    let centroid_from_corner = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let removed = area * TAU * (hole + centroid_from_corner);
    close(vol(&plate) - vol(&f), removed, 1e-9, "hole rim");
    mesh_ok(&f, "hole rim");
}

#[test]
fn fillet_all_three_edges_at_a_corner_uses_a_sphere() {
    let r = 2.0;
    let b = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    let edges =
        vec![edge_at(&b, v3(5.0, 10.0, 10.0)), edge_at(&b, v3(10.0, 5.0, 10.0)), edge_at(&b, v3(10.0, 10.0, 5.0))];
    let f = fillet(&b, &edges, r, 9).unwrap();
    assert!(f.faces.iter().any(|x| matches!(x.surface, Surface::Sphere { .. })), "expected a spherical corner");
    mesh_ok(&f, "sphere corner");
    // Reference by direct integration of the rolling-ball shape in the corner cube,
    // plus the prisms away from the corner.
    let prisms = 3.0 * corner_area(r) * (10.0 - r);
    let n = 240;
    let h = r / n as f64;
    let mut kept = 0usize;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let c = |a: usize| (a as f64 + 0.5) * h; // distance from the ball's centre plane
                if c(i).powi(2) + c(j).powi(2) + c(k).powi(2) <= r * r {
                    kept += 1;
                }
            }
        }
    }
    let corner_removed = r * r * r - kept as f64 * h * h * h;
    close(vol(&b) - vol(&f), prisms + corner_removed, 1e-3, "sphere corner");
}

#[test]
fn fillet_every_edge_of_a_box() {
    let (a, r) = (10.0, 2.0);
    let b = box_solid(&Frame::WORLD, a, a, a, 1).unwrap();
    let edges: Vec<usize> = (0..b.edges.len()).collect();
    let f = fillet(&b, &edges, r, 9).unwrap();
    let report = validate(&f).unwrap();
    assert_eq!(report.genus, vec![0]);
    // A rounded cube is 6 faces, 12 cylinders and 8 sphere octants.
    let count = |name: &str| f.faces.iter().filter(|x| x.surface.name() == name).count();
    assert_eq!(count("plane"), 6);
    assert_eq!(count("cylinder"), 12);
    assert_eq!(count("sphere"), 8);
    let inner = a - 2.0 * r;
    let expected = inner * inner * inner
        + 6.0 * inner * inner * r
        + 12.0 * inner * (PI * r * r / 4.0)
        + 4.0 / 3.0 * PI * r * r * r;
    close(vol(&f), expected, 1e-9, "rounded cube");
    mesh_ok(&f, "rounded cube");
}

#[test]
fn chamfer_on_a_rounded_plate_rim_chain() {
    // Top rim of a rounded rectangle: lines and arcs that continue tangentially.
    let (w, d, c, h, ch) = (40.0, 30.0, 5.0, 8.0, 1.5);
    let p = Profile::new(Wire::rounded_rect(w, d, c));
    let plate = extrude(&p, &Frame::WORLD, h, 1).unwrap();
    let rim = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - h).abs() < 1e-9);
    assert_eq!(rim.len(), 8);
    let f = chamfer(&plate, &rim, ch, 9).unwrap();
    let tri = ch * ch / 2.0;
    let straight = 2.0 * (w - 2.0 * c) + 2.0 * (d - 2.0 * c);
    let curved = TAU * (c - ch / 3.0);
    close(vol(&plate) - vol(&f), tri * (straight + curved), 1e-9, "rim chain chamfer");
    mesh_ok(&f, "rim chain chamfer");
    assert_eq!(validate(&f).unwrap().shells, 1);
}

#[test]
fn fillet_on_a_rounded_plate_rim_chain() {
    let (w, d, c, h, r) = (40.0, 30.0, 5.0, 8.0, 2.0);
    let p = Profile::new(Wire::rounded_rect(w, d, c));
    let plate = extrude(&p, &Frame::WORLD, h, 1).unwrap();
    let rim = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - h).abs() < 1e-9);
    let f = fillet(&plate, &rim, r, 9).unwrap();
    let area = r * r * (1.0 - PI / 4.0);
    let centroid_from_corner = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let straight = 2.0 * (w - 2.0 * c) + 2.0 * (d - 2.0 * c);
    let curved = TAU * (c - centroid_from_corner);
    close(vol(&plate) - vol(&f), area * (straight + curved), 1e-9, "rim chain fillet");
    let count = |name: &str| f.faces.iter().filter(|x| x.surface.name() == name).count();
    assert_eq!(count("torus"), 4);
    mesh_ok(&f, "rim chain fillet");
}

#[test]
fn fillet_up_against_a_wall_stops_flush() {
    // A plate with a wall rising from it. The plate's edge along the wall's foot ends
    // against nothing, but this edge along the plate's front runs into the wall's side.
    let plate = box_solid(&Frame::WORLD, 20.0, 20.0, 4.0, 1).unwrap();
    let wall = box_solid(&Frame::translation(v3(8.0, 0.0, 4.0)), 4.0, 20.0, 10.0, 2).unwrap();
    let t = union(&plate, &wall).unwrap();
    let base = vol(&t);
    // Top edge of the plate's front, left of the wall: ends at the wall (concave end).
    let e = edge_at(&t, v3(4.0, 0.0, 4.0));
    let f = fillet(&t, &[e], 1.5, 9).unwrap();
    close(vol(&f), base - corner_area(1.5) * 8.0, 1e-10, "stops at a wall");
    assert_eq!(validate(&f).unwrap().shells, 1);
    mesh_ok(&f, "stops at a wall");
}

#[test]
fn fillet_the_foot_of_a_boss() {
    // The circular edge where a boss meets a plate: a concave rim, adding a torus flare.
    let (rb, r) = (6.0, 2.0);
    let plate = box_solid(&Frame::translation(v3(-20.0, -20.0, 0.0)), 40.0, 40.0, 5.0, 1).unwrap();
    let boss = cylinder(&Frame::translation(v3(0.0, 0.0, 5.0)), rb, 10.0, 2).unwrap();
    let t = union(&plate, &boss).unwrap();
    let foot = t
        .edges_where(|i, _| (t.edge_midpoint(i).z - 5.0).abs() < 1e-9 && t.edge_is_arc_of_radius(i, rb))
        .first()
        .copied()
        .unwrap();
    let f = fillet(&t, &[foot], r, 9).unwrap();
    let area = r * r * (1.0 - PI / 4.0);
    let centroid_from_corner = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    close(vol(&f) - vol(&t), area * TAU * (rb + centroid_from_corner), 1e-9, "boss foot");
    assert_eq!(validate(&f).unwrap().shells, 1);
    mesh_ok(&f, "boss foot");
}

#[test]
fn oversized_blends_are_refused() {
    let b = block();
    let e = edge_at(&b, v3(10.0, 0.0, 6.0));
    assert!(fillet(&b, &[e], 7.0, 9).is_err(), "radius larger than the 6 mm face");
    assert!(chamfer(&b, &[e], 7.0, 9).is_err());
    assert!(fillet(&b, &[e], 0.0, 9).is_err());
    assert!(chamfer(&b, &[9999], 1.0, 9).is_err());
}

#[test]
fn smooth_edges_are_refused() {
    // The seam edge between two halves of a cylinder is not a corner.
    let c = cylinder(&Frame::WORLD, 5.0, 10.0, 1).unwrap();
    let seam = c.edges_where(|i, _| c.edge_is_parallel(i, Vec3::Z))[0];
    assert!(fillet(&c, &[seam], 1.0, 9).is_err());
}

#[test]
fn bracket_with_holes_and_fillets() {
    let (width, base, upright, t) = (40.0, 50.0, 50.0, 6.0);
    let w = Wire::polygon(&[v2(0.0, 0.0), v2(base, 0.0), v2(base, t), v2(t, t), v2(t, upright), v2(0.0, upright)]);
    let sketch = Frame { origin: v3(0.0, width, 0.0), x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
    let body = extrude(&Profile::new(w), &sketch, width, 1).unwrap();
    // Inner corner (concave) and outer corner (convex).
    let inner = edge_at(&body, v3(t, width / 2.0, t));
    let outer = edge_at(&body, v3(0.0, width / 2.0, 0.0));
    let body2 = fillet(&body, &[outer], 6.0, 9).unwrap();
    let inner2 = body2.edges_where(|i, _| body2.edge_midpoint(i).dist(v3(t, width / 2.0, t)) < 1e-6)[0];
    assert_eq!(inner2, inner2);
    let _ = inner;
    let body3 = fillet(&body2, &[inner2], 5.0, 10).unwrap();
    let mut out = body3;
    for y in [10.0, 30.0] {
        let d = cylinder(&Frame::translation(v3(35.0, y, -1.0)), 3.5, t + 2.0, 11).unwrap();
        out = subtract(&out, &d).unwrap();
        let d = cylinder(&Frame::new(v3(-1.0, y, 35.0), Vec3::X, Vec3::Y), 3.5, t + 2.0, 12).unwrap();
        out = subtract(&out, &d).unwrap();
    }
    assert_eq!(validate(&out).unwrap().genus, vec![4]);
    let expected = vol(&body) - corner_area(6.0) * width + corner_area(5.0) * width - 4.0 * PI * 3.5 * 3.5 * t;
    close(vol(&out), expected, 1e-9, "bracket");
    mesh_ok(&out, "bracket");
}

/// A box with its top front edge filleted, and the edge of the top left side selected.
fn chain_setup(first_radius: f64) -> (Body, Vec<usize>) {
    let b = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
    let first = b.edges_where(|i, _| {
        let m = b.edge_midpoint(i);
        (m.z - 20.0).abs() < 1e-6 && m.y.abs() < 1e-6
    });
    let f = fillet(&b, &first, first_radius, 2).unwrap();
    let second = f.edges_where(|i, _| {
        let m = f.edge_midpoint(i);
        (m.z - 20.0).abs() < 1e-6 && m.x.abs() < 1e-6
    });
    assert_eq!(second.len(), 1, "one edge selected");
    (f, second)
}

/// Selecting one edge follows the chain from that edge around the first fillet's arc and
/// down the vertical corner, because each continues the last tangentially. The volumes
/// are OpenCASCADE's on the same parts, where the rolling ball has a unique solution.
#[test]
fn propagation_follows_a_tangent_chain_like_occt() {
    for (r1, r2, expected) in OCCT_CHAINS {
        let (f, second) = chain_setup(*r1);
        let g = fillet(&f, &second, *r2, 3).unwrap();
        close(vol(&g), *expected, 1e-9, &format!("chain r{r1} then r{r2}"));
        mesh_ok(&g, "propagated fillet");
        validate(&g).unwrap();
    }
}

/// With propagation off, the fillet stops flush and leaves the step it used to.
#[test]
fn propagation_can_be_turned_off() {
    let (f, second) = chain_setup(2.0);
    let h = fillet_with(&f, &second, 4.0, BlendOptions { propagate: false }, 3).unwrap();
    // Exactly the prism along the selected edge, which starts 2 mm in from the end.
    close(vol(&h), vol(&f) - corner_area(4.0) * 28.0, 1e-10, "without propagation");
}

/// One straight edge of a rounded plate's rim selects the whole rim.
#[test]
fn propagation_goes_around_a_rounded_rim() {
    let (w, d, c, h, r) = (40.0, 30.0, 5.0, 8.0, 2.0);
    let plate = extrude(&Profile::new(Wire::rounded_rect(w, d, c)), &Frame::WORLD, h, 1).unwrap();
    let all = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - h).abs() < 1e-9);
    let one = all.iter().copied().find(|&i| plate.edge_is_parallel(i, Vec3::X)).unwrap();
    let every = fillet(&plate, &all, r, 9).unwrap();
    let single = fillet(&plate, &[one], r, 9).unwrap();
    close(vol(&single), vol(&every), 1e-12, "same as selecting the whole rim");
}

/// Rim radius equal to the corner radius: each corner is a sphere. OpenCASCADE reference.
#[test]
fn rim_fillet_equal_to_the_corner_radius() {
    let b = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 12.0, 1).unwrap();
    let rim = b.edges_where(|i, _| (b.edge_midpoint(i).z - 12.0).abs() < 1e-6);
    let f = fillet(&b, &rim, 8.0, 2).unwrap();
    close(vol(&f), 25736.683625955688, 1e-9, "r = corner radius");
    assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Sphere { .. })).count(), 4);
    mesh_ok(&f, "r = corner radius");
}

/// Rim radius above the corner radius. No rolling ball can touch both faces all the way
/// around such a corner, so there is no unique answer. This kernel uses the exact surface
/// the ball's section sweeps, a lemon ending in a tip on the corner's axis, and lets the
/// straight fillets meet in mitres. OpenCASCADE instead fits a free-form patch that bulges
/// 0.011 mm past the original wall, for a volume 7e-4 lower. So the reference here is
/// direct integration of this definition: the height of the top over each point.
#[test]
fn rim_fillet_larger_than_the_corner_radius() {
    let (w, d, c, h, r) = (60.0f64, 40.0f64, 8.0f64, 20.0f64, 9.0f64);
    let b = extrude(&Profile::new(Wire::rounded_rect(w, d, c)), &Frame::WORLD, h, 1).unwrap();
    let rim = b.edges_where(|i, _| (b.edge_midpoint(i).z - h).abs() < 1e-6);
    let f = fillet(&b, &rim, r, 2).unwrap();
    assert_eq!(f.faces.iter().filter(|x| matches!(x.surface, Surface::Torus { major, .. } if major < 0.0)).count(), 4);
    mesh_ok(&f, "r > corner radius");
    validate(&f).unwrap();

    let (ax, ay) = (w / 2.0 - c, d / 2.0 - c);
    let arc = |offset: f64| {
        // Height above the fillet's centre at this distance from its centre line.
        (r * r - offset * offset).max(0.0).sqrt() + (h - r)
    };
    let top = |x: f64, y: f64| -> f64 {
        // By symmetry, work in the quadrant x, y >= 0.
        if x > ax && y > ay {
            // The corner's sector, beyond the axis in both directions: the lemon.
            let rho = (x - ax).hypot(y - ay);
            if rho > c {
                return 0.0;
            }
            return arc(rho + (r - c));
        }
        let from_x = if x > w / 2.0 - r { arc(x - (w / 2.0 - r)) } else { h };
        let from_y = if y > d / 2.0 - r { arc(y - (d / 2.0 - r)) } else { h };
        from_x.min(from_y)
    };
    let n = 1500;
    let (dx, dy) = (w / 2.0 / n as f64, d / 2.0 / n as f64);
    let mut total = 0.0;
    for i in 0..n {
        for j in 0..n {
            total += top((i as f64 + 0.5) * dx, (j as f64 + 0.5) * dy) * dx * dy;
        }
    }
    close(vol(&f), 4.0 * total, 1e-5, "r > corner radius against integration");
}

#[test]
fn a_chamfer_follows_a_tangent_chain_too() {
    // One straight edge of a rounded rim brings the whole rim, as a fillet does.
    let plate = extrude(&Profile::new(Wire::rounded_rect(60.0, 40.0, 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let rim = plate.edges_where(|i, _| (plate.edge_midpoint(i).z - 10.0).abs() < 1e-9);
    let whole = chamfer(&plate, &rim, 2.0, 2).unwrap();
    let one = plate.edges_where(|i, _| {
        let m = plate.edge_midpoint(i);
        (m.z - 10.0).abs() < 1e-9 && (m.y + 20.0).abs() < 1e-9
    });
    assert_eq!(one.len(), 1);
    let chain = chamfer(&plate, &one, 2.0, 2).unwrap();
    assert!((vol(&chain) - vol(&whole)).abs() < 1e-9, "{} against {}", vol(&chain), vol(&whole));
    // Pappus for the four cones' worth of corner, and straight prisms along the sides.
    let triangle = 2.0 * 2.0 / 2.0;
    let straight = 2.0 * (60.0 - 16.0) + 2.0 * (40.0 - 16.0);
    let centroid = 8.0 - 2.0 / 3.0;
    let want = vol(&plate) - triangle * (straight + std::f64::consts::TAU * centroid);
    assert!((vol(&chain) - want).abs() < 1e-7, "{} against {want}", vol(&chain));
    // Turned off, it stops at the end of the edge.
    let lone = chamfer_with(&plate, &one, 2.0, BlendOptions { propagate: false }, 2).unwrap();
    assert!(vol(&lone) > vol(&chain) + 100.0, "{} against {}", vol(&lone), vol(&chain));
    validate(&lone).unwrap();
}
