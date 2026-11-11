//! Sheets through STEP. A sheet goes out as a surface model, and comes back the same. `scripts/check_step_sheets.sh` has
//! OpenCASCADE read what's written here and write what's read here.

use coreshape::export::step::{write_step, write_step_assembly, write_step_bodies};
use coreshape::import::step::{read_step, read_step_all, read_step_parts, read_step_sheets, read_step_structure};
use coreshape::*;
use std::f64::consts::{PI, TAU};

fn area(b: &Body) -> f64 {
    mass_properties(b).area
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6 * b.abs().max(1.0), "{what}: {a} isn't {b}");
}

fn panel() -> Body {
    fill(&Profile::new(Wire::rect(0.0, 0.0, 10.0, 20.0)), &Frame::WORLD, 1).unwrap()
}

fn back(b: &Body) -> Vec<Body> {
    let got = read_step(&write_step(b, "sheet"), 1).unwrap();
    for g in &got {
        assert!(g.sheet);
        validate(g).unwrap();
    }
    got
}

#[test]
fn a_flat_sheet_goes_out_as_a_surface_model_and_comes_back() {
    let s = panel();
    let text = write_step(&s, "panel");
    assert_eq!(text.matches("SHELL_BASED_SURFACE_MODEL").count(), 1);
    assert_eq!(text.matches("OPEN_SHELL").count(), 1);
    assert_eq!(text.matches("MANIFOLD_SURFACE_SHAPE_REPRESENTATION").count(), 1);
    assert!(!text.contains("MANIFOLD_SOLID_BREP") && !text.contains("CLOSED_SHELL") && !text.contains("ADVANCED_BREP"));
    let got = back(&s);
    assert_eq!(got.len(), 1);
    near(area(&got[0]), 200.0, "area");
    assert_eq!(got[0].boundary_edges().len(), 4);
    assert_eq!(mass_properties(&got[0]).volume, 0.0);
    // It still faces up.
    assert!(got[0].faces[0].outward_normal(v3(1.0, 1.0, 0.0)).dist(Vec3::Z) < 1e-9);
}

#[test]
fn it_still_faces_the_way_it_did_turned_over() {
    let s = panel().mirrored(&Mirror::new(Vec3::ZERO, Vec3::Z));
    let got = back(&s);
    assert!(got[0].faces[0].outward_normal(v3(1.0, 1.0, 0.0)).dist(-Vec3::Z) < 1e-9);
}

#[test]
fn curved_sheets_come_back_with_their_areas() {
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 5.0), &Frame::WORLD, 8.0, 1).unwrap();
    near(area(&back(&tube)[0]), TAU * 5.0 * 8.0, "a tube");
    let cap = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: PI / 6.0, a1: PI / 2.0 }, id: 0 }] };
    let dome = revolve_open(&cap, &Frame::WORLD, TAU, 1).unwrap();
    near(area(&back(&dome)[0]), TAU * 5.0 * 2.5, "a dome");
    let curve = Spline::through(&[v3(0.0, 0.0, 0.0), v3(5.0, 3.0, 0.0), v3(10.0, 0.0, 0.0), v3(15.0, 3.0, 0.0)]).unwrap();
    let length = curve.length(curve.domain().0, curve.domain().1);
    let wavy = extrude_open(&Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(curve)), id: 0 }] }, &Frame::WORLD, 4.0, 1).unwrap();
    near(area(&back(&wavy)[0]), length * 4.0, "a spline's");
    let path = Wire { segs: vec![Seg { geom: SegGeom::Line { a: v2(0.0, 0.0), b: v2(10.0, 0.0) }, id: 0 }, Seg { geom: SegGeom::Line { a: v2(10.0, 0.0), b: v2(10.0, 7.0) }, id: 1 }] };
    let panels = extrude_open(&path, &Frame::WORLD, 5.0, 1).unwrap();
    let got = back(&panels);
    near(area(&got[0]), 85.0, "panels at a corner");
    assert_eq!(got[0].shells.len(), 1, "they share their edge");
    assert_eq!(got[0].boundary_edges().len(), 6);
    let holed = fill(&Profile::new(Wire::circle(v2(0.0, 0.0), 10.0)).with_hole(Wire::circle(v2(2.0, 0.0), 3.0).reversed()), &Frame::WORLD, 1).unwrap();
    near(area(&back(&holed)[0]), PI * 91.0, "a hole");
}

#[test]
fn a_sheet_that_closes_comes_back_closed_and_still_a_sheet() {
    let half = Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius: 5.0, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 }] };
    let ball = revolve_open(&half, &Frame::WORLD, TAU, 1).unwrap();
    let text = write_step(&ball, "ball");
    assert!(text.contains("SHELL_BASED_SURFACE_MODEL") && text.contains("CLOSED_SHELL") && !text.contains("OPEN_SHELL"));
    let got = read_step(&text, 1).unwrap();
    assert!(got[0].sheet);
    near(area(&got[0]), 100.0 * PI, "area");
    near(mass_properties(&got[0]).volume, 4.0 / 3.0 * PI * 125.0, "volume");
}

#[test]
fn several_shells_come_back_as_several_bodies_named_apart() {
    let tubes = sides_of(&extrude(&Profile::new(Wire::circle(v2(0.0, 0.0), 5.0)).with_hole(Wire::circle(v2(0.0, 0.0), 2.0).reversed()), &Frame::WORLD, 8.0, 1).unwrap()).unwrap();
    assert_eq!(tubes.shells.len(), 2);
    let got = back(&tubes);
    assert_eq!(got.len(), 2);
    near(area(&got[0]) + area(&got[1]), TAU * 7.0 * 8.0, "both");
    let (a, b) = (coreshape_keys(&got[0]), coreshape_keys(&got[1]));
    assert!(a.iter().all(|k| !b.contains(k)), "{a:?} and {b:?}");
    // Two bodies written together are two.
    let (p, q) = (panel(), panel().transformed(&Frame::translation(v3(50.0, 0.0, 0.0))));
    let got = read_step(&write_step_bodies(&[&p, &q], "two"), 1).unwrap();
    assert_eq!(got.len(), 2);
    assert!(got[1].vertices.iter().all(|v| v.x >= 49.0));
}

/// What tells a face apart, as far as a kernel test can see.
fn coreshape_keys(b: &Body) -> Vec<(FaceTag, Vec<coreshape::topo::CopyStep>)> {
    b.faces.iter().map(|f| (f.tag, f.origin.copies.clone())).collect()
}

#[test]
fn beside_a_solid_a_sheet_is_only_read_when_its_asked_for() {
    let (s, solid) = (panel(), box_solid(&Frame::translation(v3(100.0, 0.0, 0.0)), 2.0, 3.0, 4.0, 2).unwrap());
    let text = write_step_bodies(&[&solid, &s], "both");
    assert!(text.contains("ADVANCED_BREP_SHAPE_REPRESENTATION") && text.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION") && text.contains("SHAPE_REPRESENTATION_RELATIONSHIP"));
    // As it always did, for a file with solids in it.
    let plain = read_step(&text, 1).unwrap();
    assert_eq!(plain.len(), 1);
    assert!(!plain[0].sheet);
    near(mass_properties(&plain[0]).volume, 24.0, "the solid");
    let all = read_step_all(&text, 1).unwrap();
    assert_eq!(all.len(), 2);
    assert!(!all[0].sheet && all[1].sheet);
    near(area(&all[1]), 200.0, "the sheet");
    // Named apart.
    assert!(coreshape_keys(&all[0]).iter().all(|k| !coreshape_keys(&all[1]).contains(k)));
    let sheets = read_step_sheets(&text, 1).unwrap();
    assert_eq!(sheets.len(), 1);
    assert!(read_step_sheets(&write_step(&solid, "solid"), 1).unwrap_err().to_string().contains("no surfaces"));
}

#[test]
fn solids_write_as_they_did() {
    let solid = box_solid(&Frame::WORLD, 2.0, 3.0, 4.0, 2).unwrap();
    let text = write_step(&solid, "solid");
    assert!(text.contains("MANIFOLD_SOLID_BREP") && text.contains("CLOSED_SHELL") && text.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));
    assert!(!text.contains("OPEN_SHELL") && !text.contains("SHELL_BASED") && !text.contains("MANIFOLD_SURFACE") && !text.contains("RELATIONSHIP"));
}

#[test]
fn a_sheet_can_be_a_part_in_an_assembly() {
    let (s, solid) = (panel(), box_solid(&Frame::WORLD, 2.0, 3.0, 4.0, 2).unwrap());
    let text = write_step_assembly(&[("Panel", &s), ("Block", &solid)], &[(0, "Panel <1>", Frame::translation(v3(0.0, 0.0, 7.0))), (1, "Block <1>", Frame::WORLD), (0, "Panel <2>", Frame::translation(v3(0.0, 0.0, 9.0)))], "Mixed");
    // Each part in the kind of representation that can hold it.
    assert_eq!(text.matches("MANIFOLD_SURFACE_SHAPE_REPRESENTATION").count(), 1);
    assert_eq!(text.matches("ADVANCED_BREP_SHAPE_REPRESENTATION").count(), 1);
    let structure = read_step_structure(&text).unwrap();
    assert_eq!(structure.names, vec!["Block"], "with solids in it, its solids are its parts, as they were");
    assert_eq!(read_step(&text, 1).unwrap().len(), 1);
    let all = read_step_all(&text, 1).unwrap();
    assert_eq!(all.len(), 3);
    let heights: Vec<f64> = all.iter().filter(|b| b.sheet).map(|b| b.vertices[0].z).collect();
    assert_eq!(heights, vec![7.0, 9.0]);
    // With nothing but sheets, they're the parts.
    let text = write_step_assembly(&[("Panel", &s)], &[(0, "Panel <1>", Frame::translation(v3(0.0, 0.0, 7.0))), (0, "Panel <2>", Frame::translation(v3(0.0, 0.0, 9.0)))], "Panels");
    let structure = read_step_structure(&text).unwrap();
    assert_eq!(structure.names, vec!["Panel"]);
    assert_eq!(structure.instances.len(), 2);
    assert_eq!(read_step_parts(&text, 1).unwrap().len(), 1);
    assert_eq!(read_step(&text, 1).unwrap().len(), 2);
}

#[test]
fn an_open_shell_in_a_solid_is_still_refused() {
    let solid = box_solid(&Frame::WORLD, 2.0, 3.0, 4.0, 2).unwrap();
    let text = write_step(&solid, "solid").replacen("CLOSED_SHELL", "OPEN_SHELL", 1);
    let why = read_step(&text, 1).unwrap_err().to_string();
    assert!(why.contains("open shell"), "{why}");
    // And a shell a solid's file calls closed, with a face gone, is still called out.
    let whole = write_step(&solid, "solid");
    let text: String = whole
        .lines()
        .map(|l| match l.find("CLOSED_SHELL('',(") {
            Some(at) => {
                let (head, list) = l.split_at(at + "CLOSED_SHELL('',(".len());
                let rest = list.split_once(',').map_or(list, |x| x.1);
                format!("{head}{rest}\n")
            }
            None => format!("{l}\n"),
        })
        .collect();
    let why = read_step(&text, 1).unwrap_err().to_string();
    assert!(why.contains("isn't closed"), "{why}");
}

#[test]
fn construction_geometry_is_left_out() {
    // As a file marks its reference planes. It has nothing else in it, so it says so.
    let text = write_step(&panel(), "plane").replace("MANIFOLD_SURFACE_SHAPE_REPRESENTATION", "CONSTRUCTIVE_GEOMETRY_REPRESENTATION");
    assert!(read_step(&text, 1).is_err());
    // Beside a real one, only the real one comes in.
    let (a, b) = (panel(), panel().transformed(&Frame::translation(v3(50.0, 0.0, 0.0))));
    let first = write_step(&a, "a");
    assert_eq!(read_step(&first, 1).unwrap().len(), 1);
    let both = write_step_bodies(&[&a, &b], "both");
    assert_eq!(read_step(&both, 1).unwrap().len(), 2);
}

#[test]
fn what_comes_back_can_be_cut_thickened_and_sewn() {
    let (s, other) = (panel(), fill(&Profile::new(Wire::rect(0.0, 0.0, 10.0, 20.0)), &Frame::translation(v3(10.0, 0.0, 0.0)), 2).unwrap());
    let got = back(&s);
    let t = thicken(&got[0], &[0], 2.0, 9).unwrap();
    near(mass_properties(&t).volume, 400.0, "thickened");
    let bar = box_solid(&Frame::translation(v3(4.0, -5.0, -5.0)), 2.0, 30.0, 10.0, 3).unwrap();
    let cut = trim_sheet(&got[0], &bar, Keep::Outside).unwrap();
    near(area(&cut), 200.0 - 40.0, "cut");
    let sewn = sew(&[&got[0], &other], 1e-6).unwrap();
    near(area(&sewn), 400.0, "sewn");
}
