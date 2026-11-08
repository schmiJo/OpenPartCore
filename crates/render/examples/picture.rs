//! Renders a demo-sized part to PNG files, to look at. Usage: picture <output directory>
use coreshape::*;
use coreshape_render::*;

fn main() {
    let dir = std::env::args().nth(1).expect("output directory");
    std::fs::create_dir_all(&dir).unwrap();
    let plate = extrude(&Profile::new(Wire::rounded_rect(80.0, 50.0, 8.0)), &Frame::WORLD, 10.0, 1).unwrap();
    let boss = cylinder(&Frame::translation(v3(0.0, 0.0, 10.0)), 12.0, 14.0, 2).unwrap();
    let hole = cylinder(&Frame::translation(v3(0.0, 0.0, -1.0)), 7.0, 30.0, 3).unwrap();
    let body = subtract(&union(&plate, &boss).unwrap(), &hole).unwrap();
    let names: Vec<String> = body.faces.iter().map(|f| format!("f{}:{}", f.tag.feature, f.tag.slot)).collect();
    let save = |name: &str, image: Image| {
        let png = image.to_png();
        println!("{name}: {} bytes, {} labels", png.len(), image.labels.len());
        std::fs::write(format!("{dir}/{name}.png"), png).unwrap();
    };
    save("iso", render(&body, &Options { width: 900, height: 650, ..Options::default() }).unwrap());
    save("labels", render(&body, &Options { width: 900, height: 650, colors: Colors::ByFace, face_names: names, ..Options::default() }).unwrap());
    save("four", render_four(&body, &Options { width: 1000, height: 800, ..Options::default() }).unwrap());
}
