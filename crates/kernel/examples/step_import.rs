//! Imports STEP files and prints what came in, for comparing with another system.
//! Usage: step_import <file.step>...   prints name, solids, volume, area, faces, or the refusal.
use coreshape::import::step::read_step;
use coreshape::*;

fn main() {
    for path in std::env::args().skip(1) {
        let name = std::path::Path::new(&path).file_stem().map_or(path.clone(), |s| s.to_string_lossy().into_owned());
        let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
        match read_step(&text, 1) {
            Ok(bodies) => {
                let (mut v, mut a, mut f, mut watertight) = (0.0, 0.0, 0, true);
                for b in &bodies {
                    let m = mass_properties(b);
                    v += m.volume;
                    a += m.area;
                    f += b.faces.len();
                    match tessellate(b, &TessParams::for_size(b.aabb().diagonal())) {
                        Ok(mesh) => watertight &= mesh.open_edge_count() == 0,
                        Err(_) => watertight = false,
                    }
                }
                // Does it work like any other body? A box through the middle of the first solid, and back out to a file.
                let bounds = bodies[0].aabb();
                let c = bounds.center();
                let size = bounds.diagonal() * 0.1;
                let tool = box_solid(&Frame::translation(c), size, size * 1.3, size * 0.7, 2).unwrap();
                let cut = match subtract(&bodies[0], &tool) {
                    Ok(r) => {
                        if let Ok(dir) = std::env::var("STEP_OUT") {
                            std::fs::write(format!("{dir}/{name}.step"), export::step::write_step(&r, &name)).unwrap();
                        }
                        format!("{}", mass_properties(&r).volume)
                    }
                    Err(e) => format!("refused: {}", e.to_string().replace(',', ";")),
                };
                println!("{name},ok,{},{v},{a},{f},{watertight},{cut}", bodies.len());
            }
            Err(e) => println!("{name},refused,{}", e.to_string().replace(',', ";")),
        }
    }
}
