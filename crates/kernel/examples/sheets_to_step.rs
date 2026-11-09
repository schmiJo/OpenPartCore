//! For `scripts/check_step_sheets.sh`. `write <folder>` writes sheets as STEP, with what each should come to. `read <folder>`
//! reads whatever STEP files OpenCASCADE wrote there, and says what it finds in each.

use coreshape::export::step::{write_step, write_step_bodies};
use coreshape::import::step::read_step_all;
use coreshape::*;
use std::f64::consts::{PI, TAU};

fn arc(radius: f64, a0: f64, a1: f64) -> Wire {
    Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius, a0, a1 }, id: 0 }] }
}

fn write(dir: &str) {
    let panel = fill(&Profile::new(Wire::rect(0.0, 0.0, 10.0, 20.0)), &Frame::WORLD, 1).unwrap();
    let holed = fill(&Profile::new(Wire::circle(v2(0.0, 0.0), 10.0)).with_hole(Wire::circle(v2(2.0, 0.0), 3.0).reversed()), &Frame::WORLD, 1).unwrap();
    let tube = extrude_open(&Wire::circle(v2(0.0, 0.0), 5.0), &Frame::WORLD, 8.0, 1).unwrap();
    let dome = revolve_open(&arc(5.0, PI / 6.0, PI / 2.0), &Frame::WORLD, TAU, 1).unwrap();
    let ball = revolve_open(&arc(5.0, -PI / 2.0, PI / 2.0), &Frame::WORLD, TAU, 1).unwrap();
    let corner = Wire { segs: vec![Seg { geom: SegGeom::Line { a: v2(0.0, 0.0), b: v2(10.0, 0.0) }, id: 0 }, Seg { geom: SegGeom::Line { a: v2(10.0, 0.0), b: v2(10.0, 7.0) }, id: 1 }] };
    let panels = extrude_open(&corner, &Frame::WORLD, 5.0, 1).unwrap();
    let curve = Spline::through(&[v3(0.0, 0.0, 0.0), v3(5.0, 3.0, 0.0), v3(10.0, 0.0, 0.0), v3(15.0, 3.0, 0.0)]).unwrap();
    let length = curve.length(curve.domain().0, curve.domain().1);
    let wavy = extrude_open(&Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(curve)), id: 0 }] }, &Frame::WORLD, 4.0, 1).unwrap();
    let wide = fill(&Profile::new(Wire::rect(0.0, 0.0, 40.0, 40.0)), &Frame::WORLD, 1).unwrap();
    let post = cylinder(&Frame::translation(v3(20.0, 20.0, -5.0)), 5.0, 10.0, 2).unwrap();
    let cut_hole = trim_sheet(&wide, &post, Keep::Outside).unwrap();
    let slanted = trim_sheet_by_plane(&tube, v3(0.0, 0.0, 4.0), v3(0.0, 0.5, 1.0), Keep::Outside).unwrap();
    let block = box_solid(&Frame::translation(v3(100.0, 0.0, 0.0)), 2.0, 3.0, 4.0, 2).unwrap();
    let list: Vec<(&str, &Body, f64)> = vec![
        ("panel", &panel, 200.0),
        ("holed", &holed, PI * 91.0),
        ("tube", &tube, TAU * 5.0 * 8.0),
        ("dome", &dome, TAU * 5.0 * 2.5),
        ("ball", &ball, 100.0 * PI),
        ("panels", &panels, 85.0),
        ("wavy", &wavy, length * 4.0),
        ("cut_hole", &cut_hole, 1600.0 - PI * 25.0),
        ("slanted", &slanted, TAU * 5.0 * 4.0),
    ];
    let mut expected = String::new();
    for (name, body, area) in list {
        std::fs::write(format!("{dir}/{name}.step"), write_step(body, name)).unwrap();
        expected.push_str(&format!("{name} {area} {} {}\n", body.boundary_edges().len(), body.shells.len()));
    }
    // A sheet beside a solid, with both areas together.
    let both = write_step_bodies(&[&block, &panel], "both");
    std::fs::write(format!("{dir}/both.step"), both).unwrap();
    expected.push_str(&format!("both {} {} {}\n", mass_properties(&block).area + 200.0, panel.boundary_edges().len(), 2));
    std::fs::write(format!("{dir}/expected.txt"), expected).unwrap();
}

fn read(dir: &str) {
    let mut names: Vec<String> = std::fs::read_dir(dir).unwrap().filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().to_string()).filter(|n| n.starts_with("occt_") && n.ends_with(".step")).collect();
    names.sort();
    for name in names {
        let text = std::fs::read_to_string(format!("{dir}/{name}")).unwrap();
        match read_step_all(&text, 1) {
            Ok(bodies) => {
                let area: f64 = bodies.iter().map(|b| mass_properties(b).area).sum();
                let sheets = bodies.iter().filter(|b| b.sheet).count();
                let valid = bodies.iter().all(|b| validate(b).is_ok());
                let boundary: usize = bodies.iter().map(|b| b.boundary_edges().len()).sum();
                println!("{name} ok {area} {sheets} {} {valid} {boundary}", bodies.len());
            }
            Err(why) => println!("{name} refused {why}"),
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match (args.get(1).map(String::as_str), args.get(2)) {
        (Some("write"), Some(dir)) => write(dir),
        (Some("read"), Some(dir)) => read(dir),
        _ => eprintln!("give write or read, and a folder"),
    }
}
