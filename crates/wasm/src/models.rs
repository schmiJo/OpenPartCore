//! The demo models. Each is a document written only against the kernel's public API, so
//! a demo can't hide what the kernel can't do. A demo is plain data: it can be saved to a
//! file and opened again, or driven by the MCP server.

use coreshape::*;
use coreshape_document::*;

#[derive(Clone, Copy, Debug)]
pub struct ParamDef {
    pub key: &'static str,
    pub label: &'static str,
    pub unit: &'static str,
    pub min: f64,
    pub max: f64,
    pub step: f64,
    pub default: f64,
    /// Names for each whole-number value, when the parameter picks from a list. Such a
    /// parameter chooses which document to use instead of being a dimension in it.
    pub choices: &'static [&'static str],
}

pub struct ModelDef {
    pub id: &'static str,
    pub name: &'static str,
    pub blurb: &'static str,
    pub params: &'static [ParamDef],
    /// Adds the features. The sliders' parameters are already in the document.
    pub build: fn(&mut Document, usize),
}

impl ModelDef {
    /// Which document the slider values ask for.
    pub fn variant(&self, values: &[f64]) -> usize {
        self.params
            .iter()
            .zip(values)
            .find(|(p, _)| !p.choices.is_empty())
            .map_or(0, |(_, v)| v.round().max(0.0) as usize)
    }

    /// A new document for one variant, with its sliders at their defaults.
    pub fn document(&self, variant: usize) -> Document {
        let mut doc = Document::new();
        for p in self.params.iter().filter(|p| p.choices.is_empty()) {
            doc.set_parameter(p.key, p.default).expect("demo parameter names are valid");
        }
        (self.build)(&mut doc, variant);
        doc
    }

    /// Writes slider values into a document.
    pub fn apply(&self, doc: &mut Document, values: &[f64]) {
        for (p, v) in self.params.iter().zip(values) {
            if p.choices.is_empty() {
                doc.set_parameter(p.key, *v).expect("demo parameter names are valid");
            }
        }
    }

    pub fn defaults(&self) -> Vec<f64> {
        self.params.iter().map(|p| p.default).collect()
    }
}

const fn mm(key: &'static str, label: &'static str, min: f64, max: f64, default: f64) -> ParamDef {
    ParamDef { key, label, unit: "mm", min, max, step: 0.5, default, choices: &[] }
}

const fn deg(key: &'static str, label: &'static str, min: f64, max: f64, default: f64) -> ParamDef {
    ParamDef { key, label, unit: "\u{b0}", min, max, step: 1.0, default, choices: &[] }
}

const fn count(key: &'static str, label: &'static str, min: f64, max: f64, default: f64) -> ParamDef {
    ParamDef { key, label, unit: "", min, max, step: 1.0, default, choices: &[] }
}

fn xy() -> Placement {
    Placement::xy(0.0)
}

/// A spline revolved into a vase, with a second spline cut out of it to hollow it.
fn vase(doc: &mut Document, _: usize) {
    let outside = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::Path {
            start: point("foot", 0.0),
            moves: vec![
                Move::Spline { through: vec![point("belly", "height * 0.35"), point("waist", "height * 0.72"), point("lip", "height")] },
                Move::Line { to: point(0.0, "height") },
                Move::Line { to: point(0.0, 0.0) },
            ],
        })],
    ));
    doc.with(Op::Revolve { sketch: outside, degrees: 360.0.into(), mode: Mode::Add });
    let inside = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::Path {
            start: point("foot - wall", "wall"),
            // Through the same heights as the outside, so that the two curves stay about a wall apart. It
            // ends level with the top, and the cut's flat end lying on the top is what opens the mouth.
            moves: vec![
                Move::Spline { through: vec![point("belly - wall", "height * 0.35"), point("waist - wall", "height * 0.72"), point("lip - wall", "height")] },
                Move::Line { to: point(0.0, "height") },
                Move::Line { to: point(0.0, "wall") },
            ],
        })],
    ));
    doc.with(Op::Revolve { sketch: inside, degrees: 360.0.into(), mode: Mode::Remove });
}

/// A cam from one spline around, with a bore and an elliptical window. A hole drilled across it has to
/// follow the curved wall, which is the traced case.
fn cam(doc: &mut Document, _: usize) {
    let lobe = |angle: f64, r: &str| point(format!("({r}) * {}", angle.to_radians().cos()).as_str(), format!("({r}) * {}", angle.to_radians().sin()).as_str());
    let s = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::Path {
            start: lobe(0.0, "lift"),
            moves: vec![Move::Spline {
                through: vec![lobe(50.0, "base + (lift - base) * 0.45"), lobe(120.0, "base"), lobe(180.0, "base"), lobe(240.0, "base"), lobe(310.0, "base + (lift - base) * 0.45"), lobe(360.0, "lift")],
            }],
        })
        .hole(Shape::circle(0.0, 0.0, "bore / 2"))
        .hole(Shape::Ellipse { cx: "lift * 0.55".into(), cy: 0.0.into(), rx: "slot".into(), ry: "slot * 0.45".into(), degrees: "tilt".into() })],
    ));
    doc.with(Op::extrude(s, 0.0, "thick", Mode::Add));
    let across = doc.with(Op::sketch(
        Placement::new(point3(0.0, 0.0, "thick / 2"), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)),
        vec![Region::new(Shape::circle(0.0, 0.0, "pin / 2"))],
    ));
    doc.with(Op::extrude(across, 0.0, "lift + 5", Mode::Remove));
}

/// A nozzle from a rectangle through a circle to an ellipse, so all its sides are free-form, with a hole
/// across it that has to follow them.
fn nozzle(doc: &mut Document, _: usize) {
    let base = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect("-width / 2", "-depth / 2", "width", "depth"))]));
    let waist = doc.with(Op::sketch(Placement::xy("height * 0.55"), vec![Region::new(Shape::circle(0.0, 0.0, "waist"))]));
    let lip = doc.with(Op::sketch(
        Placement::xy("height"),
        vec![Region::new(Shape::Ellipse { cx: 0.0.into(), cy: 0.0.into(), rx: "lip".into(), ry: "lip * 0.7".into(), degrees: 0.0.into() })],
    ));
    doc.with(Op::Loft { sections: vec![base, waist, lip], mode: Mode::Add });
    let across = doc.with(Op::sketch(
        Placement::new(point3("-width", 0.0, "height * 0.3"), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)),
        vec![Region::new(Shape::circle(0.0, 0.0, "pin / 2"))],
    ));
    doc.with(Op::extrude(across, 0.0, "2 * width", Mode::Remove));
}

fn bracket(doc: &mut Document, _: usize) {
    // Sketched on XZ, with the extrusion running from y = width back toward y = 0.
    let plane = Placement::new(point3(0.0, "width", 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0));
    let s = doc.with(Op::sketch(
        plane,
        vec![Region::new(Shape::polygon(vec![
            point(0.0, 0.0),         // 0: underside
            point("base", 0.0),      // 1: end of the base
            point("base", "thick"),  // 2: top of the base
            point("thick", "thick"), // 3: inner face of the upright
            point("thick", "upright"), // 4: top of the upright
            point(0.0, "upright"),   // 5: back of the upright
        ]))],
    ));
    let e = doc.with(Op::extrude(s, 0.0, "width", Mode::Add));
    doc.with(Op::fillet(EdgeSelect::Between(e.side(5), e.side(0)), "outer"));
    doc.with(Op::fillet(EdgeSelect::Between(e.side(2), e.side(3)), "inner"));
    doc.with(Op::fillet(
        EdgeSelect::Any(vec![
            EdgeSelect::Between(e.side(1), e.start_cap()),
            EdgeSelect::Between(e.side(1), e.end_cap()),
            EdgeSelect::Between(e.side(4), e.start_cap()),
            EdgeSelect::Between(e.side(4), e.end_cap()),
        ]),
        "corner",
    ));
    let hole = |y: &str| Region::new(Shape::circle("(base + thick) / 2", y, "hole / 2"));
    let base_holes = doc.with(Op::sketch(xy(), vec![hole("width * 0.27"), hole("width * 0.73")]));
    let drilled = doc.with(Op::extrude(base_holes, -1.0, "thick + 2", Mode::Remove));
    let upright_hole = doc.with(Op::sketch(
        Placement::new(point3(-1.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)),
        vec![Region::new(Shape::circle("width / 2", "(upright + thick) / 2", "hole / 2"))],
    ));
    doc.with(Op::extrude(upright_hole, 0.0, "thick + 2", Mode::Remove));
    doc.with(Op::chamfer(
        EdgeSelect::Any(vec![
            EdgeSelect::Between(drilled.region_side(0, 0), e.side(2)),
            EdgeSelect::Between(drilled.region_side(1, 0), e.side(2)),
        ]),
        "rim",
    ));
}

fn flange(doc: &mut Document, _: usize) {
    let s = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::polygon(vec![
            point("bore / 2", 0.0),     // 0: underside
            point("outer / 2", 0.0),    // 1: rim
            point("outer / 2", "thick"), // 2: top of the flange
            point("hub / 2", "thick"),  // 3: hub wall
            point("hub / 2", "height"), // 4: hub top
            point("bore / 2", "height"), // 5: bore
        ]))],
    ));
    let r = doc.with(Op::revolve(s, 360.0, Mode::Add));
    doc.with(Op::fillet(EdgeSelect::Between(r.side(2), r.side(3)), "foot"));
    doc.with(Op::chamfer(
        EdgeSelect::Any(vec![EdgeSelect::Between(r.side(1), r.side(2)), EdgeSelect::Between(r.side(4), r.side(5))]),
        "edge",
    ));
    let bolts = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::circle(0.0, 0.0, "hole / 2")).repeated(Repeat::Circular {
            count: "bolts".into(),
            radius: "circle / 2".into(),
            start_degrees: "180 / bolts".into(),
        })],
    ));
    doc.with(Op::extrude(bolts, -1.0, "thick + 2", Mode::Remove));
}

fn rounded_block(doc: &mut Document, _: usize) {
    let s = doc.with(Op::sketch(xy(), vec![Region::new(Shape::centered("w", "d"))]));
    doc.with(Op::extrude(s, 0.0, "h", Mode::Add));
    doc.with(Op::fillet(EdgeSelect::All, "r"));
}

fn plate(doc: &mut Document, _: usize) {
    let corner = |x: &str, y: &str| Shape::circle(x, y, "hole / 2");
    let (px, py) = ("w / 2 - inset", "d / 2 - inset");
    let (nx, ny) = ("-(w / 2 - inset)", "-(d / 2 - inset)");
    let s = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::rounded(0.0, 0.0, "w", "d", "corner"))
            .hole(corner(nx, ny))
            .hole(corner(px, ny))
            .hole(corner(px, py))
            .hole(corner(nx, py))],
    ));
    let e = doc.with(Op::extrude(s, 0.0, "t", Mode::Add));
    doc.with(Op::fillet(EdgeSelect::OnFace(e.end_cap()), "top"));
    doc.with(Op::chamfer(EdgeSelect::OnFace(e.start_cap()), "bottom"));
}

fn crossed(doc: &mut Document, _: usize) {
    doc.set_parameter("reach", 60.0).expect("a valid name");
    let a = doc.with(Op::sketch(Placement::xy("-reach"), vec![Region::new(Shape::circle(0.0, 0.0, "r1"))]));
    doc.with(Op::extrude(a, 0.0, "2 * reach", Mode::Add));
    let along = point3("cos(angle)", 0.0, "sin(angle)");
    let from = point3("-reach * cos(angle)", "offset", "-reach * sin(angle)");
    let b = doc.with(Op::sketch(
        Placement::new(from, along, point3(0.0, 1.0, 0.0)),
        vec![Region::new(Shape::circle(0.0, 0.0, "r2"))],
    ));
    doc.with(Op::extrude(b, 0.0, "2 * reach", Mode::Intersect));
}

fn pockets(doc: &mut Document, _: usize) {
    let s = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect(-30.0, -20.0, 60.0, 40.0))]));
    doc.with(Op::extrude(s, 0.0, 10.0, Mode::Add));
    let square = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect("offset - 8", -16.0, 16.0, 12.0))]));
    doc.with(Op::extrude(square, "10 - depth", "depth + 5", Mode::Remove));
    let round = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle("offset", 10.0, "radius"))]));
    doc.with(Op::extrude(round, "10 - depth", "depth + 5", Mode::Remove));
}

fn knob(doc: &mut Document, _: usize) {
    for (name, value) in [
        ("base", "8"),
        ("chamfer", "2.5"),
        ("shoulder", "base + chamfer"),
        ("bead_bottom", "shoulder + 6"),
        ("bead_top", "bead_bottom + 2 * bead"),
        ("dome_z", "bead_top + max(abs(neck - dome), 1)"),
    ] {
        doc.set_parameter(name, value).expect("valid derived parameters");
    }
    let line = |x: &str, y: &str| Move::Line { to: point(x, y) };
    let s = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::Path {
            start: point(0.0, 0.0),
            moves: vec![
                line("body", "0"),                      // plane
                line("body", "base"),                   // cylinder
                line("body - chamfer", "shoulder"),     // cone
                line("neck", "shoulder"),               // plane
                line("neck", "bead_bottom"),            // cylinder
                Move::Arc { to: point("neck", "bead_top"), center: point("neck", "bead_bottom + bead"), ccw: true }, // torus
                line("dome", "dome_z"),                 // cone
                Move::Arc { to: point(0.0, "dome_z + dome"), center: point(0.0, "dome_z"), ccw: true }, // sphere
            ],
        })],
    ));
    doc.with(Op::revolve(s, 360.0, Mode::Add));
}

fn sector(doc: &mut Document, _: usize) {
    let s = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::rounded("inner + section / 2", 0.0, "section", "section", "section / 5"))],
    ));
    doc.with(Op::revolve(s, "angle", Mode::Add));
}

/// Names of the Boolean stress cases, in slider order.
const STRESS_NAMES: [&str; 14] = [
    "Pocket flush with the top",
    "Boxes sharing a whole face",
    "Boxes touching along an edge",
    "Boxes touching at one corner",
    "Box standing on its corner",
    "Cylinder tangent to a wall",
    "Bore tangent to a wall inside",
    "Coaxial cylinders, flush ends",
    "Equal cylinders crossing",
    "Cone cut through its apex",
    "Ball bored along its axis",
    "Two balls touching at a point",
    "Torus halved through its axis",
    "Known limit: tangent bore",
];

fn stress(doc: &mut Document, case: usize) {
    let at = |x: f64, y: f64, z: f64| Placement::at(point3(x, y, z));
    let block = |p: Placement, dx: f64, dy: f64, dz: f64, mode: Mode| {
        Op::solid(Primitive::Box { dx: dx.into(), dy: dy.into(), dz: dz.into() }, p, mode)
    };
    let rod = |p: Placement, r: f64, h: f64, mode: Mode| Op::solid(Primitive::Cylinder { radius: r.into(), height: h.into() }, p, mode);
    let ball = |p: Placement, r: f64, mode: Mode| Op::solid(Primitive::Sphere { radius: r.into() }, p, mode);
    let ops: Vec<Op> = match case {
        0 => vec![block(at(-30.0, -30.0, 0.0), 60.0, 60.0, 20.0, Mode::Add), block(at(-15.0, -15.0, 8.0), 30.0, 30.0, 12.0, Mode::Remove)],
        1 => vec![block(at(-30.0, -15.0, 0.0), 30.0, 30.0, 30.0, Mode::Add), block(at(0.0, -15.0, 0.0), 30.0, 30.0, 30.0, Mode::Add)],
        2 => vec![block(at(-30.0, -30.0, 0.0), 30.0, 30.0, 30.0, Mode::Add), block(at(0.0, 0.0, 0.0), 30.0, 30.0, 30.0, Mode::Add)],
        3 => vec![block(at(-30.0, -30.0, -30.0), 30.0, 30.0, 30.0, Mode::Add), block(at(0.0, 0.0, 0.0), 30.0, 30.0, 30.0, Mode::Add)],
        4 => {
            let tilt = Frame::rotation(Vec3::ZERO, v3(1.0, -1.0, 0.0), (1.0f64 / 3.0f64.sqrt()).acos());
            vec![block(at(-40.0, -40.0, -10.0), 80.0, 80.0, 10.0, Mode::Add), block(Placement::from_frame(&tilt), 30.0, 30.0, 30.0, Mode::Add)]
        }
        5 => vec![block(at(-40.0, -30.0, 0.0), 80.0, 30.0, 20.0, Mode::Add), rod(at(0.0, 12.0, 0.0), 12.0, 20.0, Mode::Add)],
        6 => vec![block(at(-40.0, -30.0, 0.0), 80.0, 30.0, 20.0, Mode::Add), rod(at(0.0, -12.0, 5.0), 12.0, 15.0, Mode::Remove)],
        7 => vec![rod(at(0.0, 0.0, 0.0), 25.0, 40.0, Mode::Add), rod(at(0.0, 0.0, 0.0), 15.0, 40.0, Mode::Remove)],
        8 => {
            let side = Placement::new(point3(-40.0, 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0));
            vec![rod(at(0.0, 0.0, -40.0), 20.0, 80.0, Mode::Add), rod(side, 20.0, 80.0, Mode::Intersect)]
        }
        9 => vec![
            Op::solid(Primitive::Cone { r0: 25.0.into(), r1: 0.0.into(), height: 45.0.into() }, xy(), Mode::Add),
            block(at(0.0, -40.0, -5.0), 40.0, 80.0, 60.0, Mode::Remove),
        ],
        10 => vec![ball(xy(), 30.0, Mode::Add), rod(at(0.0, 0.0, -40.0), 15.0, 80.0, Mode::Remove)],
        11 => vec![ball(at(-15.0, 0.0, 0.0), 15.0, Mode::Add), ball(at(20.0, 0.0, 0.0), 20.0, Mode::Add)],
        12 => vec![
            Op::solid(Primitive::Torus { major: 35.0.into(), minor: 10.0.into() }, xy(), Mode::Add),
            block(at(0.0, -60.0, -20.0), 60.0, 120.0, 40.0, Mode::Intersect),
        ],
        // A cylinder internally tangent to a sphere meets it in a figure eight with a
        // double point. The kernel refuses this one rather than guess.
        _ => vec![ball(xy(), 30.0, Mode::Add), rod(at(12.0, 0.0, -40.0), 18.0, 80.0, Mode::Remove)],
    };
    for op in ops {
        doc.with(op);
    }
}

pub fn catalog() -> &'static [ModelDef] {
    static VASE: [ParamDef; 6] = [
        mm("height", "Height", 60.0, 160.0, 110.0),
        mm("foot", "Foot radius", 18.0, 36.0, 26.0),
        mm("belly", "Belly radius", 30.0, 60.0, 44.0),
        mm("waist", "Waist radius", 14.0, 30.0, 18.0),
        mm("lip", "Lip radius", 18.0, 40.0, 26.0),
        mm("wall", "Wall thickness", 1.5, 6.0, 3.0),
    ];
    static NOZZLE: [ParamDef; 6] = [
        mm("height", "Height", 50.0, 110.0, 80.0),
        mm("width", "Base width", 40.0, 80.0, 60.0),
        mm("depth", "Base depth", 30.0, 70.0, 45.0),
        mm("waist", "Waist radius", 10.0, 18.0, 14.0),
        mm("lip", "Lip radius", 14.0, 30.0, 22.0),
        mm("pin", "Cross hole", 4.0, 12.0, 8.0),
    ];
    static CAM: [ParamDef; 7] = [
        mm("lift", "Lobe radius", 40.0, 70.0, 55.0),
        mm("base", "Base radius", 24.0, 36.0, 30.0),
        mm("bore", "Bore diameter", 8.0, 24.0, 16.0),
        mm("thick", "Thickness", 6.0, 24.0, 14.0),
        mm("slot", "Window length", 4.0, 9.0, 7.0),
        ParamDef { key: "tilt", label: "Window angle", unit: "deg", min: 0.0, max: 180.0, step: 1.0, default: 30.0, choices: &[] },
        mm("pin", "Cross hole", 2.0, 6.0, 4.0),
    ];
    static BRACKET: [ParamDef; 9] = [
        mm("width", "Width", 40.0, 120.0, 70.0),
        mm("base", "Base length", 30.0, 100.0, 60.0),
        mm("upright", "Upright height", 30.0, 100.0, 55.0),
        mm("thick", "Wall thickness", 6.0, 20.0, 8.0),
        mm("outer", "Outer bend radius", 1.0, 5.5, 5.0),
        mm("inner", "Inner bend radius", 1.0, 16.0, 6.0),
        mm("corner", "Corner radius", 1.0, 10.0, 6.0),
        mm("hole", "Hole diameter", 4.0, 16.0, 10.0),
        mm("rim", "Hole chamfer", 0.5, 2.5, 1.0),
    ];
    static FLANGE: [ParamDef; 10] = [
        mm("outer", "Outer diameter", 80.0, 160.0, 110.0),
        mm("hub", "Hub diameter", 30.0, 56.0, 46.0),
        mm("bore", "Bore diameter", 4.0, 26.0, 20.0),
        mm("thick", "Flange thickness", 8.0, 25.0, 12.0),
        mm("height", "Overall height", 30.0, 80.0, 42.0),
        mm("foot", "Hub fillet", 1.0, 5.0, 4.0),
        mm("edge", "Edge chamfer", 0.5, 3.0, 1.5),
        count("bolts", "Bolt count", 3.0, 12.0, 6.0),
        mm("hole", "Bolt hole diameter", 4.0, 10.0, 8.0),
        mm("circle", "Bolt circle diameter", 66.0, 76.0, 76.0),
    ];
    static BLOCK: [ParamDef; 4] = [
        mm("w", "Width", 20.0, 80.0, 50.0),
        mm("d", "Depth", 20.0, 80.0, 40.0),
        mm("h", "Height", 20.0, 80.0, 30.0),
        mm("r", "Fillet radius", 1.0, 9.5, 6.0),
    ];
    static PLATE: [ParamDef; 8] = [
        mm("w", "Width", 60.0, 200.0, 120.0),
        mm("d", "Depth", 60.0, 160.0, 80.0),
        mm("t", "Thickness", 6.0, 30.0, 10.0),
        mm("corner", "Corner radius", 3.0, 18.0, 10.0),
        mm("hole", "Hole diameter", 4.0, 12.0, 9.0),
        mm("inset", "Hole inset", 18.0, 28.0, 18.0),
        mm("top", "Top fillet", 0.5, 2.5, 1.5),
        mm("bottom", "Bottom chamfer", 0.5, 2.0, 1.0),
    ];
    static CROSSED: [ParamDef; 4] = [
        mm("r1", "Radius A", 5.0, 30.0, 20.0),
        mm("r2", "Radius B", 5.0, 30.0, 20.0),
        mm("offset", "Axis offset", 0.0, 8.0, 0.0),
        deg("angle", "Crossing angle", 30.0, 90.0, 90.0),
    ];
    static POCKETS: [ParamDef; 3] = [
        mm("offset", "Tool position", -40.0, 40.0, 0.0),
        mm("radius", "Round pocket radius", 2.0, 10.0, 6.0),
        mm("depth", "Pocket depth", 1.0, 15.0, 5.0),
    ];
    static KNOB: [ParamDef; 4] = [
        mm("body", "Base radius", 16.0, 40.0, 26.0),
        mm("neck", "Neck radius", 6.0, 13.0, 10.0),
        mm("bead", "Bead radius", 1.5, 6.0, 3.5),
        mm("dome", "Dome radius", 5.0, 14.0, 9.0),
    ];
    static SECTOR: [ParamDef; 3] = [
        mm("inner", "Inner radius", 2.0, 60.0, 28.0),
        mm("section", "Section size", 6.0, 30.0, 16.0),
        deg("angle", "Sweep angle", 20.0, 355.0, 250.0),
    ];
    static STRESS: [ParamDef; 1] = [ParamDef {
        key: "case",
        label: "Case",
        unit: "",
        min: 0.0,
        max: 13.0,
        step: 1.0,
        default: 0.0,
        choices: &STRESS_NAMES,
    }];
    static MODELS: [ModelDef; 12] = [
        ModelDef {
            id: "bracket",
            name: "Bracket",
            blurb: "Sketch, extrude, fillet the bends and corners, drill, chamfer the holes. Each dimension rebuilds only what depends on it.",
            params: &BRACKET,
            build: bracket,
        },
        ModelDef {
            id: "flange",
            name: "Flange",
            blurb: "A revolved flange with a filleted hub, chamfered edges and a bolt circle. The hub fillet is a true torus.",
            params: &FLANGE,
            build: flange,
        },
        ModelDef {
            id: "block",
            name: "Rounded block",
            blurb: "Every edge filleted. Twelve cylinders, six planes and eight sphere patches that meet exactly.",
            params: &BLOCK,
            build: rounded_block,
        },
        ModelDef {
            id: "plate",
            name: "Mounting plate",
            blurb: "All top edges filleted and all bottom edges chamfered, with each feature selecting edges by the face they bound.",
            params: &PLATE,
            build: plate,
        },
        ModelDef {
            id: "crossed",
            name: "Crossed cylinders",
            blurb: "Two cylinders intersected. Equal radii crossing square make a Steinmetz solid of volume 16r\u{b3}/3. Offsets or unequal radii need the general curve tracer.",
            params: &CROSSED,
            build: crossed,
        },
        ModelDef {
            id: "pockets",
            name: "Sliding pockets",
            blurb: "Slide the tools off the plate's edge. Many positions line faces up exactly. If the kernel refuses a position, the reason appears here.",
            params: &POCKETS,
            build: pockets,
        },
        ModelDef {
            id: "stress",
            name: "Stress cases",
            blurb: "The contacts Booleans find hardest: coplanar faces, edges and corners that only touch, and tangent surfaces. The last case is a known limit, where the kernel refuses rather than guess.",
            params: &STRESS,
            build: stress,
        },
        ModelDef {
            id: "vase",
            name: "Spline vase",
            blurb: "A sketched spline revolved, with a second spline cut out to hollow it. The curve stays an exact spline all the way to the STEP file.",
            params: &VASE,
            build: vase,
        },
        ModelDef {
            id: "nozzle",
            name: "Free-form nozzle",
            blurb: "A loft from a rectangle through a circle to an ellipse, so all its sides are free-form patches, with a hole across them that follows the surface. Slower than the others: it's the hardest thing the kernel does.",
            params: &NOZZLE,
            build: nozzle,
        },
        ModelDef {
            id: "cam",
            name: "Spline cam",
            blurb: "One spline around the outline, with a bore, an elliptical window and a cross hole. Slicing a spline surface by a plane gives an exact spline back, and the cross hole has to follow the curve.",
            params: &CAM,
            build: cam,
        },
        ModelDef {
            id: "knob",
            name: "Turned knob",
            blurb: "One revolve that generates all five surface types: plane, cylinder, cone, torus and sphere.",
            params: &KNOB,
            build: knob,
        },
        ModelDef {
            id: "sector",
            name: "Ring sector",
            blurb: "A partial revolve with end caps. Past 180 degrees the kernel splits each swept face in two.",
            params: &SECTOR,
            build: sector,
        },
    ];
    &MODELS
}
