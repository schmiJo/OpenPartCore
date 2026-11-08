//! Replays one fuzz case and prints what happened. Usage: replay <category> <seed>
use coreshape::fuzz::*;
use coreshape::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let category = match args[1].as_str() {
        "grid" => Category::GridBoxes,
        "rot" => Category::RotatedBoxes,
        "prism" => Category::GridPrisms,
        "gen" => Category::GeneralPrimitives,
        _ => Category::AlignedPrimitives,
    };
    let seed: u64 = args[2].parse().unwrap();
    let mut rng = Rng::new(seed ^ ((category as u64 + 1) << 40));
    let a = make_shape(category, &mut rng).unwrap();
    let b = make_shape(category, &mut rng).unwrap();
    for (n, s) in [("A", &a), ("B", &b)] {
        let mut seen: Vec<String> = Vec::new();
        for f in &s.faces {
            let d = match f.surface {
                Surface::Plane(fr) => format!("plane n=({:.2} {:.2} {:.2}) d={:.2}", fr.z.x, fr.z.y, fr.z.z, fr.z.dot(fr.origin)),
                Surface::Cylinder { frame, radius } => format!("cyl o=({:.2} {:.2} {:.2}) z=({:.2} {:.2} {:.2}) r={radius:.3}", frame.origin.x, frame.origin.y, frame.origin.z, frame.z.x, frame.z.y, frame.z.z),
                Surface::Cone { frame, radius, half_angle } => format!("cone o=({:.2} {:.2} {:.2}) z=({:.2} {:.2} {:.2}) r={radius:.3} a={:.3}", frame.origin.x, frame.origin.y, frame.origin.z, frame.z.x, frame.z.y, frame.z.z, half_angle),
                Surface::Sphere { frame, radius } => format!("sphere o=({:.2} {:.2} {:.2}) r={radius:.3}", frame.origin.x, frame.origin.y, frame.origin.z),
                Surface::Torus { frame, major, minor } => format!("torus o=({:.2} {:.2} {:.2}) z=({:.2} {:.2} {:.2}) R={major:.3} r={minor:.3}", frame.origin.x, frame.origin.y, frame.origin.z, frame.z.x, frame.z.y, frame.z.z),
            };
            if !d.starts_with("plane") && !seen.contains(&d) {
                println!("  {n} {d}");
                seen.push(d);
            }
        }
        let bb = s.aabb();
        println!("  {n} box ({:.2} {:.2} {:.2}) ({:.2} {:.2} {:.2})", bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z);
    }
    for op in [BoolOp::Union, BoolOp::Subtract, BoolOp::Intersect] {
        match coreshape::boolean::boolean(&a, &b, op) {
            Ok(r) => {
                println!("{op:?}: ok, {} faces, volume {}", r.faces.len(), mass_properties(&r).volume);
                if !r.faces.is_empty() {
                    match tessellate(&r, &TessParams::for_size(r.aabb().diagonal())) {
                        Ok(m) => {
                            for (p, q) in m.open_edges().iter().take(12) {
                                println!("   open ({:.4} {:.4} {:.4}) - ({:.4} {:.4} {:.4})", p.x, p.y, p.z, q.x, q.y, q.z);
                            }
                            if !m.open_edges().is_empty() {
                                for (i, f) in r.faces.iter().enumerate() {
                                    let pts: Vec<Vec<String>> = f.loops.iter().map(|l| l.iter().map(|&c| { let p = r.vertices[r.start_vertex(c)]; format!("({:.2} {:.2} {:.2})", p.x, p.y, p.z) }).collect()).collect();
                                    println!("   face {i} {} flipped={}: {:?}", f.surface.name(), f.flipped, pts);
                                }
                            }
                        }
                        Err(e) => println!("   tess error: {e}"),
                    }
                }
            }
            Err(e) => println!("{op:?}: ERR {e}"),
        }
    }
    let out = check_pair(&a, &b, &mut rng, 150);
    println!("{:#?}", out.wrong);
}
