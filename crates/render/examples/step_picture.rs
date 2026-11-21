//! A picture of everything a STEP file brings in, each part in a tint of its own, where the file puts it.
//! Usage: step_picture <file.step> <out.png> [azimuth elevation]
use coreshape::import::step::read_step_each;
use coreshape::Body;
use coreshape_render::{render, Colors, Options, View};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let text = String::from_utf8_lossy(&std::fs::read(&args[1]).unwrap()).into_owned();
    let read = read_step_each(&text, 1).unwrap();
    let placed: Vec<Body> = read.placed(1).into_iter().map(|(_, b)| b).collect();
    let list: Vec<&Body> = placed.iter().collect();
    let all = Body::side_by_side(&list);
    let mut groups = Vec::new();
    for (k, b) in placed.iter().enumerate() {
        groups.extend(std::iter::repeat_n(k as u32, b.faces.len()));
    }
    let view = match (args.get(3).and_then(|a| a.parse().ok()), args.get(4).and_then(|e| e.parse().ok())) {
        (Some(azimuth), Some(elevation)) => View::Orbit { azimuth, elevation },
        _ => View::Iso,
    };
    let options = Options { width: 1600, height: 1200, view, colors: Colors::ByGroup, highlight_faces: Vec::new(), highlight_edges: Vec::new(), face_names: Vec::new(), face_groups: groups };
    let image = render(&all, &options).unwrap();
    std::fs::write(&args[2], image.to_png()).unwrap();
    println!("{} parts, {} bodies", read.parts.len(), placed.len());
}
