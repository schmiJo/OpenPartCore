//! Writes a small STEP assembly to a folder: a plate with a pin on it and a nut at an angle, and two more of those pins.
//! `scripts/check_step_assembly.sh` has OpenCASCADE read it and compare what it finds.

use coreshape::*;
use coreshape_assembly::*;
use coreshape_document::*;

fn main() {
    let dir = std::env::args().nth(1).expect("give a folder");
    let mut p = Project::new();
    let studio = p.tabs[0].id;
    let solid = |primitive| Op::solid(primitive, Placement::xy(0.0), Mode::NewBody);
    let doc = p.studio_mut(studio).unwrap();
    let plate = doc.with(solid(Primitive::Box { dx: 40.0.into(), dy: 20.0.into(), dz: 5.0.into() }));
    let pin = doc.with(solid(Primitive::Cylinder { radius: 3.0.into(), height: 10.0.into() }));
    let ball = doc.with(solid(Primitive::Sphere { radius: 4.0.into() }));
    let body = |feature| BodyId { feature, index: 0 };
    let tab = p.add_assembly("Plate and pins");
    let a = p.assembly_mut(tab).unwrap();
    a.add_instance(Source::Part { studio, body: body(plate) }, "Plate <1>".into(), Frame::WORLD);
    a.add_instance(Source::Part { studio, body: body(pin) }, "Pin <1>".into(), Frame::translation(v3(10.0, 10.0, 5.0)));
    a.add_instance(Source::Part { studio, body: body(pin) }, "Pin <2>".into(), Frame::new(v3(30.0, 10.0, 15.0), -Vec3::Z, Vec3::Y));
    a.add_instance(Source::Part { studio, body: body(pin) }, "Pin <3>".into(), Frame::new(v3(100.0, 20.0, 30.0), v3(1.0, 1.0, 1.0), v3(1.0, -1.0, 0.0)));
    a.add_instance(Source::Part { studio, body: body(ball) }, "Ball <1>".into(), Frame::new(v3(-50.0, 0.0, 0.0), Vec3::Y, Vec3::Z));
    let text = p.step_text(tab, "Plate and pins").unwrap();
    std::fs::write(format!("{dir}/assembly.step"), &text).unwrap();
}
