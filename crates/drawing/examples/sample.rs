//! Draws a bracket: a stepped block with two holes and a round boss, as SVG, PDF, DXF and PNG in the folder given.
use coreshape::boolean::{subtract, union};
use coreshape::*;
use coreshape_drawing::*;

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| ".".into());
    let at = |x: f64, y: f64, z: f64| Frame { origin: v3(x, y, z), ..Frame::WORLD };
    let base = box_solid(&at(0.0, 0.0, 0.0), 80.0, 50.0, 12.0, 1).unwrap();
    let step = box_solid(&at(0.0, 0.0, 12.0), 30.0, 50.0, 18.0, 2).unwrap();
    let boss = cylinder(&at(55.0, 25.0, 12.0), 10.0, 10.0, 3).unwrap();
    let mut part = union(&union(&base, &step).unwrap(), &boss).unwrap();
    for (x, y, r) in [(15.0, 12.0, 4.0), (15.0, 38.0, 4.0), (55.0, 25.0, 5.0)] {
        part = subtract(&part, &cylinder(&at(x, y, -5.0), r, 40.0, 4).unwrap()).unwrap();
    }
    let mut d = Drawing::standard();
    d.add_view(ViewKind::Section { point: v3(0.0, 25.0, 0.0), normal: -Vec3::Y, letter: "A".into() }, None);
    d.title = Title { title: "Bracket".into(), number: "CS-0001".into(), part: "Bracket".into(), material: "AlMg3".into(), author: "CoreShape".into(), date: "2026-11-23".into(), ..Title::default() };
    let front = d.views[0].id;
    d.add_dimension(front, DimKind::Distance { a: Spot::At(v3(0.0, 0.0, 30.0)), b: Spot::At(v3(30.0, 0.0, 30.0)), along: Along::Horizontal }, 10.0, None).unwrap();
    let top = d.views[1].id;
    d.add_dimension(top, DimKind::Diameter { near: Spot::At(v3(55.0, 25.0, 22.0)) }, 8.0, None).unwrap();
    let started = std::time::Instant::now();
    let page = page(&d, &part, &|_| Err(Error::InvalidInput("no references here".into()))).unwrap();
    println!("page in {:.2} s, scale {}, {} lines, {} texts, notes {:?}", started.elapsed().as_secs_f64(), format_scale(page.scale), page.lines.len(), page.texts.len(), page.notes);
    std::fs::write(format!("{dir}/bracket.svg"), out::svg(&page)).unwrap();
    std::fs::write(format!("{dir}/bracket.pdf"), out::pdf(&page)).unwrap();
    std::fs::write(format!("{dir}/bracket.dxf"), out::dxf(&page)).unwrap();
    std::fs::write(format!("{dir}/bracket.png"), out::png(&page, 4.0)).unwrap();
}
