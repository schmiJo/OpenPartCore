//! The demo models. Each is a document written only against the kernel's public API, so
//! a demo can't hide what the kernel can't do. A demo is plain data: it can be saved to a
//! file and opened again, or driven by the MCP server.

use coreshape::*;
use coreshape_assembly::{Anchor, Dof, FacePoint, MateKind, Project, Solved, Source, TabId};
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
    /// For a mechanism: the assembly of the parts it adds.
    pub assembly: Option<AssemblyStep>,
}

/// What makes a model a mechanism: an assembly of its parts, moved by one of its sliders.
#[derive(Clone, Copy)]
pub struct AssemblyStep {
    /// Puts the assembly together from the parts in a Part Studio, and gives the mate the slider turns.
    pub assemble: fn(&mut Project, TabId, TabId) -> u32,
    /// The slider that turns it, in degrees.
    pub drive: &'static str,
    /// What to show of where it's got to: a label, a value and a unit for each.
    pub readout: fn(&Solved) -> Vec<(&'static str, f64, &'static str)>,
}

/// A mechanism being worked on: its tabs, and the mate that moves it.
pub struct Mechanism {
    pub project: Project,
    pub studio: TabId,
    pub assembly: TabId,
    pub drive: u32,
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

    /// A new mechanism with its sliders at their defaults, if this is one.
    pub fn mechanism(&self) -> Option<Mechanism> {
        let step = self.assembly?;
        let mut project = Project::from_document(self.document(0));
        let studio = project.tabs[0].id;
        let assembly = project.add_assembly(self.name);
        let drive = (step.assemble)(&mut project, studio, assembly);
        Some(Mechanism { project, studio, assembly, drive })
    }

    /// Writes slider values into a mechanism: dimensions into its parts, and the driving slider into its mate, which goes
    /// as far as the others let it. Gives what comes of that, and the value that slider asked for.
    pub fn move_mechanism(&self, m: &mut Mechanism, values: &[f64]) -> Option<(Solved, f64)> {
        let step = self.assembly?;
        let doc = m.project.studio_mut(m.studio).ok()?;
        self.apply(doc, values);
        let at = self.params.iter().position(|p| p.key == step.drive)?;
        let asked = *values.get(at)?;
        let solved = m.project.set_mate_value(m.assembly, m.drive, Dof::Angle, asked).ok()?;
        Some((solved, asked))
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
    doc.with(Op::revolve(outside, 360.0, Mode::Add));
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
    doc.with(Op::revolve(inside, 360.0, Mode::Remove));
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

/// The upright plane through the Z axis, whose sketch x is the distance from the axis and whose y is the height.
fn upright() -> Placement {
    Placement::new(point3(0.0, 0.0, 0.0), point3(0.0, -1.0, 0.0), point3(1.0, 0.0, 0.0))
}

/// A round wire carried along a helix. Each face is fitted to within a stated tolerance of the true one.
fn spring(doc: &mut Document, _: usize) {
    let wire = doc.with(Op::sketch(upright(), vec![Region::new(Shape::circle("coil", 0.0, "wire"))]));
    doc.with(Op::Helix {
        sketch: wire,
        point: point3(0.0, 0.0, 0.0),
        axis: point3(0.0, 0.0, 1.0),
        pitch: "pitch".into(),
        turns: "turns".into(),
        left_handed: false,
        mode: Mode::Add,
    });
}

/// A shaft with a square thread cut along it, running off both ends.
fn screw(doc: &mut Document, _: usize) {
    let round = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle(0.0, 0.0, "radius"))]));
    doc.with(Op::extrude(round, 0.0, "length", Mode::Add));
    let groove = doc.with(Op::sketch(
        upright(),
        vec![Region::new(Shape::rect("radius - depth", "-pitch * 1.5", "depth + 1", "pitch * 0.5"))],
    ));
    doc.with(Op::Helix {
        sketch: groove,
        point: point3(0.0, 0.0, 0.0),
        axis: point3(0.0, 0.0, 1.0),
        pitch: "pitch".into(),
        turns: "length / pitch + 2".into(),
        left_handed: false,
        mode: Mode::Remove,
    });
}

/// A lofted cup: its wall is the free-form side moved inward by exactly the thickness, and its foot is that same
/// side's lower band thickened outward.
fn cup(doc: &mut Document, _: usize) {
    let base = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle(0.0, 0.0, "foot"))]));
    let waist = doc.with(Op::sketch(Placement::xy("height * 0.5"), vec![Region::new(Shape::circle(0.0, 0.0, "waist"))]));
    let lip = doc.with(Op::sketch(Placement::xy("height"), vec![Region::new(Shape::circle(0.0, 0.0, "lip"))]));
    let body = doc.with(Op::Loft { sections: vec![base, waist, lip], mode: Mode::Add });
    doc.with(Op::Shell { open: vec![body.end_cap()], thickness: "wall".into() });
}

/// A plate with a row of holes and a rounded top edge, then one hole and one blend taken away again by name. Everything
/// around them closes over the gap, and the rest keeps its names, so the sliders still drive it.
fn healed(doc: &mut Document, _: usize) {
    let plate = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect(0.0, 0.0, "width", "depth"))]));
    let block = doc.with(Op::extrude(plate, 0.0, "thick", Mode::Add));
    let circle = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle("width / 6", "depth / 2", "bore"))]));
    let hole = doc.with(Op::extrude(circle, -1.0, "thick + 2", Mode::Remove));
    let row = doc.with(Op::Pattern { of: vec![hole], along: PatternPath::Linear { direction: point3(1.0, 0.0, 0.0), count: 3.0.into(), spacing: "width / 3".into() } });
    let round = doc.with(Op::fillet(EdgeSelect::Between(block.end_cap(), block.side(0)), "round"));
    doc.with(Op::Chamfer { edges: EdgeSelect::Between(block.end_cap(), block.side(2)), distance: "round".into(), propagate: true });
    // The middle hole, and the fillet.
    doc.with(Op::DeleteFace {
        faces: vec![
            FaceKey { face: hole.side(0), between: Vec::new(), copies: vec![coreshape::topo::CopyStep { feature: row.0, index: 1 }], piece: None },
            FaceKey::plain(round.side(1)),
        ],
    });
}

/// A block with two pockets close together and close to the side, with every face then moved in. Past half of either wall,
/// the moved faces would cross: the pockets join, or one opens through the side.
fn walls(doc: &mut Document, _: usize) {
    let plate = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect(0.0, 0.0, 60.0, 30.0))]));
    doc.with(Op::extrude(plate, 0.0, 20.0, Mode::Add));
    let first = doc.with(Op::sketch(Placement::xy(5.0), vec![Region::new(Shape::rect("side", 6.0, 12.0, 18.0))]));
    doc.with(Op::extrude(first, 0.0, 20.0, Mode::Remove));
    let second = doc.with(Op::sketch(Placement::xy(5.0), vec![Region::new(Shape::rect("side + 12 + between", 6.0, 12.0, 18.0))]));
    doc.with(Op::extrude(second, 0.0, 20.0, Mode::Remove));
    doc.with(Op::Offset { distance: "-inward".into() });
}

/// A branch off a pipe, with the saddle between them filleted. No exact surface exists for that, so it's a fitted one.
fn tee(doc: &mut Document, _: usize) {
    let along = doc.with(Op::sketch(
        Placement::new(point3("-length / 2", 0.0, 0.0), point3(1.0, 0.0, 0.0), point3(0.0, 1.0, 0.0)),
        vec![Region::new(Shape::circle(0.0, 0.0, "main"))],
    ));
    let pipe = doc.with(Op::extrude(along, 0.0, "length", Mode::Add));
    let up = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle(0.0, 0.0, "branch"))]));
    let stub = doc.with(Op::extrude(up, 0.0, "reach", Mode::Add));
    doc.with(Op::fillet(EdgeSelect::Between(pipe.side(0), stub.side(0)), "round"));
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

/// Two flanges bolted together. Every part is a body of its own, so they stay separate parts though they touch all around,
/// and a change to one part leaves the others alone.
fn panel(doc: &mut Document, _: usize) {
    // An open curve, swept into a sheet with no thickness and no inside.
    let curve = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::OpenPath {
            start: point("-width / 2", 0.0),
            moves: vec![Move::Spline { through: vec![point("-width / 6", "wave"), point("width / 6", "-wave"), point("width / 2", 0.0)] }],
        })],
    ));
    doc.with(Op::extrude(curve, 0.0, "height", Mode::Surface));
    // A round window through it, from a sketch standing in front of it.
    let window = doc.with(Op::sketch(
        Placement::new(point3(0.0, "-2 * width", 0.0), point3(0.0, 1.0, 0.0), point3(1.0, 0.0, 0.0)),
        vec![Region::new(Shape::circle(0.0, "-height / 2", "hole"))],
    ));
    doc.with(Op::Trim { tool: TrimTool::Sketch(window), keep: TrimKeep::Outside });
}

fn joint(doc: &mut Document, _: usize) {
    for (name, value) in [("nut_h", "0.8 * bolt"), ("nut_r", "0.95 * bolt"), ("head_h", "0.6 * bolt"), ("head_r", "0.85 * bolt"), ("tail", "3")] {
        doc.set_parameter(name, value).expect("valid derived parameters");
    }
    let of = |feature: FeatureId| vec![BodyId { feature, index: 0 }];
    // One outline for both flanges, and one ring of holes through each.
    let disc = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle(0.0, 0.0, "outer / 2"))]));
    let lower = doc.with(Op::extrude(disc, 0.0, "thick", Mode::NewBody));
    let upper = doc.with(Op::extrude(disc, "thick", "thick", Mode::NewBody));
    let ring = doc.with(Op::sketch(
        xy(),
        vec![Region::new(Shape::circle(0.0, 0.0, "bolt / 2")).repeated(Repeat::Circular { count: "bolts".into(), radius: "circle / 2".into(), start_degrees: 0.0.into() })],
    ));
    let lower_holes = doc.with(Op::extrude(ring, -1.0, "2 * thick + 2", Mode::Remove));
    doc.set_scope(lower_holes, of(lower)).expect("the flange comes before it");
    let upper_holes = doc.with(Op::extrude(ring, -1.0, "2 * thick + 2", Mode::Remove));
    doc.set_scope(upper_holes, of(upper)).expect("the flange comes before it");
    // The upper one's outer rim, and not the holes.
    let rim = doc.with(Op::fillet(EdgeSelect::Both(vec![EdgeSelect::OnFace(upper.end_cap()), EdgeSelect::Radius("outer / 2".into())]), 3.0));
    doc.set_scope(rim, of(upper)).expect("the flange comes before it");
    // A bolt through the first hole.
    let screw = doc.with(Op::sketch(
        Placement::at(point3("circle / 2", 0.0, "-(nut_h + tail)")),
        vec![Region::new(Shape::polygon(vec![
            point(0.0, 0.0),
            point("bolt / 2", 0.0),
            point("bolt / 2", "nut_h + tail + 2 * thick"),
            point("head_r", "nut_h + tail + 2 * thick"),
            point("head_r", "nut_h + tail + 2 * thick + head_h"),
            point(0.0, "nut_h + tail + 2 * thick + head_h"),
        ]))],
    ));
    let bolt = doc.with(Op::revolve(screw, 360.0, Mode::NewBody));
    // And a nut under it.
    let corner = |k: f64| {
        let a = 60.0 * k;
        point(format!("circle / 2 + nut_r * cos({a})").as_str(), format!("nut_r * sin({a})").as_str())
    };
    let hex = doc.with(Op::sketch(
        Placement::xy("-nut_h"),
        vec![Region::new(Shape::polygon(vec![corner(0.0), corner(1.0), corner(2.0), corner(3.0), corner(4.0), corner(5.0)])).hole(Shape::circle("circle / 2", 0.0, "bolt / 2"))],
    ));
    let nut = doc.with(Op::extrude(hex, 0.0, "nut_h", Mode::NewBody));
    // The same in every other hole.
    doc.with(Op::Pattern {
        of: vec![bolt, nut],
        along: PatternPath::Circular { center: point3(0.0, 0.0, 0.0), axis: point3(0.0, 0.0, 1.0), count: "bolts".into(), degrees: 360.0.into() },
    });
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

/// How thick the mechanism's plates are. Its mates' offsets stack the plates by this, so it isn't a slider.
const LAYER: f64 = 8.0;

/// A crank, a rod and a piston on a frame, as plates in layers so that they can pass each other. Each is its own part, and
/// each hole is what a mate connector is found from, so changing a length carries the mates with it. The frame keeps the
/// piston between two guides, as long as its travel, from a stop past the far end of it.
fn slider_crank(doc: &mut Document, _: usize) {
    let t = LAYER;
    for (name, value) in [("far", "crank + rod + 60"), ("near", "rod - crank - 20"), ("wide", "crank + 20"), ("gap", "12.5"), ("guide", "18.5")] {
        doc.set_parameter(name, value).expect("valid derived parameters");
    }
    let named = |doc: &mut Document, name: &str, op: Op| doc.insert(op, Some(name.to_string()), usize::MAX).expect("the feature is valid");
    // The frame: a plate behind everything, with the pivot's hole in it, then a post, then the guides on top of that.
    let plate = named(
        doc,
        "Frame sketch",
        Op::sketch(Placement::xy(-t), vec![Region::new(Shape::rect("-wide", "-wide", "wide + far + 20", "2 * wide")).hole(Shape::circle(0.0, 0.0, 4.0))]),
    );
    let frame = named(doc, "Frame", Op::extrude(plate, 0.0, t, Mode::NewBody));
    let only_frame = vec![BodyId { feature: frame, index: 0 }];
    let post = named(doc, "Post sketch", Op::sketch(xy(), vec![Region::new(Shape::rect("far + 1", "-guide", 6.0, "2 * guide"))]));
    let posted = named(doc, "Post", Op::extrude(post, 0.0, 2.0 * t, Mode::Add));
    doc.set_scope(posted, only_frame.clone()).expect("the frame comes before it");
    let guides = named(
        doc,
        "Guides sketch",
        Op::sketch(
            Placement::xy(2.0 * t),
            vec![Region::new(Shape::polygon(vec![
                point("near", "-guide"),
                point("far + 8", "-guide"),
                point("far + 8", "guide"),
                point("near", "guide"),
                point("near", "gap"),
                point("far", "gap"),
                point("far", "-gap"),
                point("near", "-gap"),
            ]))],
        ),
    );
    let guided = named(doc, "Guides", Op::extrude(guides, 0.0, t, Mode::Add));
    doc.set_scope(guided, only_frame).expect("the frame comes before it");
    // A plate with a hole at each end.
    let link = |doc: &mut Document, name: &str, length: &str| {
        let outline = Shape::rounded(format!("({length}) / 2").as_str(), 0.0, format!("({length}) + 18").as_str(), 18.0, 8.0);
        let sketch = doc.with(Op::sketch(xy(), vec![Region::new(outline).hole(Shape::circle(0.0, 0.0, 4.0)).hole(Shape::circle(length, 0.0, 4.0))]));
        doc.insert(Op::extrude(sketch, 0.0, t, Mode::NewBody), Some(name.to_string()), usize::MAX).expect("the feature is valid")
    };
    link(doc, "Crank", "crank");
    link(doc, "Rod", "rod");
    let piston = doc.with(Op::sketch(xy(), vec![Region::new(Shape::rect(-15.0, -12.0, 30.0, 24.0)).hole(Shape::circle(0.0, 0.0, 4.0))]));
    named(doc, "Piston", Op::extrude(piston, 0.0, t, Mode::NewBody));
    let pin = doc.with(Op::sketch(xy(), vec![Region::new(Shape::circle(0.0, 0.0, 3.9))]));
    named(doc, "Pin", Op::extrude(pin, 0.0, 2.0 * t, Mode::NewBody));
}

fn assemble_slider_crank(p: &mut Project, studio: TabId, tab: TabId) -> u32 {
    let t = LAYER;
    let feature = |p: &Project, name: &str| p.studio(studio).expect("it's a Part Studio").features().iter().find(|f| f.name == name).expect("the model has it").id;
    let (frame, crank, rod, piston, pin) = (feature(p, "Frame"), feature(p, "Crank"), feature(p, "Rod"), feature(p, "Piston"), feature(p, "Pin"));
    let guides = feature(p, "Guides");
    let a = p.assembly_mut(tab).expect("it's an assembly");
    let part = |feature| Source::Part { studio, body: BodyId { feature, index: 0 } };
    // Roughly where they belong, at the sliders' defaults, so that it starts on the branch with the piston out along X.
    let place = |x: f64, z: f64| Frame::translation(v3(x, 0.0, z));
    let ground = a.add_instance(part(frame), "Frame".to_string(), Frame::WORLD);
    a.instance_mut(ground).expect("just added").fixed = true;
    let on_crank = a.add_instance(part(crank), "Crank".to_string(), Frame::WORLD);
    let on_rod = a.add_instance(part(rod), "Rod".to_string(), place(30.0, t));
    let on_piston = a.add_instance(part(piston), "Piston".to_string(), place(130.0, 2.0 * t));
    let pins = [
        a.add_instance(part(pin), "Pin <1>".to_string(), place(0.0, -t)),
        a.add_instance(part(pin), "Pin <2>".to_string(), place(30.0, 0.0)),
        a.add_instance(part(pin), "Pin <3>".to_string(), place(130.0, t)),
    ];
    // Each found from a hole's axis, or from a face, so that it follows when the part changes.
    let mut connect = |instance: u32, face: FaceRef, point: FacePoint| a.add_connector(vec![instance], Anchor::Face { key: FaceKey::plain(face), point }).expect("the instance is there");
    let pivot = connect(ground, frame.hole_side(0, 0), FacePoint::AxisMiddle);
    let (crank_start, crank_end) = (connect(on_crank, crank.hole_side(0, 0), FacePoint::AxisMiddle), connect(on_crank, crank.hole_side(1, 0), FacePoint::AxisMiddle));
    let (rod_start, rod_end) = (connect(on_rod, rod.hole_side(0, 0), FacePoint::AxisMiddle), connect(on_rod, rod.hole_side(1, 0), FacePoint::AxisMiddle));
    let wrist = connect(on_piston, piston.hole_side(0, 0), FacePoint::AxisMiddle);
    // The stop past the guides and the piston's front, each facing along X, so a slider between them runs along X.
    let (stop, front) = (connect(ground, guides.side(1), FacePoint::Centroid), connect(on_piston, piston.side(1), FacePoint::Centroid));
    let pin_axes = [connect(pins[0], pin.side(0), FacePoint::AxisMiddle), connect(pins[1], pin.side(0), FacePoint::AxisMiddle), connect(pins[2], pin.side(0), FacePoint::AxisMiddle)];
    let mut mate = |kind: MateKind, from: u32, to: u32, name: &str, along: f64| {
        let id = a.add_mate(kind, from, to).expect("two different instances");
        let m = a.mate_mut(id).expect("just added");
        m.name = name.to_string();
        m.offset = [0.0, 0.0, along];
        id
    };
    // The next plate in, one layer on.
    let drive = mate(MateKind::Revolute, pivot, crank_start, "Crank on frame", t);
    mate(MateKind::Revolute, crank_end, rod_start, "Rod on crank", t);
    mate(MateKind::Revolute, rod_end, wrist, "Piston on rod", t);
    mate(MateKind::Slider, stop, front, "Piston in guides", 0.0);
    // A pin reaches half way out of each of the two plates it joins.
    mate(MateKind::Fastened, crank_start, pin_axes[0], "Pin in crank", -t / 2.0);
    mate(MateKind::Fastened, crank_end, pin_axes[1], "Pin in crank", t / 2.0);
    mate(MateKind::Fastened, rod_end, pin_axes[2], "Pin in rod", t / 2.0);
    drive
}

fn slider_crank_readout(solved: &Solved) -> Vec<(&'static str, f64, &'static str)> {
    solved.placed.iter().filter(|q| q.name == "Piston").map(|q| ("Piston from the pivot", q.at.origin.x, "mm")).collect()
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
    static PANEL: [ParamDef; 4] = [
        mm("width", "Width", 80.0, 200.0, 140.0),
        mm("height", "Height", 40.0, 100.0, 70.0),
        mm("wave", "Wave", 4.0, 30.0, 16.0),
        mm("hole", "Window radius", 5.0, 18.0, 12.0),
    ];
    static JOINT: [ParamDef; 5] = [
        mm("outer", "Flange diameter", 110.0, 160.0, 130.0),
        mm("circle", "Bolt circle", 60.0, 90.0, 80.0),
        count("bolts", "Bolts", 3.0, 8.0, 6.0),
        mm("bolt", "Bolt diameter", 6.0, 12.0, 10.0),
        mm("thick", "Flange thickness", 6.0, 20.0, 12.0),
    ];
    static SPRING: [ParamDef; 4] = [
        mm("coil", "Coil radius", 6.0, 20.0, 12.0),
        mm("wire", "Wire radius", 0.5, 2.0, 1.5),
        mm("pitch", "Pitch", 4.5, 14.0, 6.0),
        count("turns", "Turns", 1.0, 8.0, 4.0),
    ];
    static SCREW: [ParamDef; 4] = [
        mm("radius", "Shaft radius", 5.0, 10.0, 8.0),
        mm("length", "Length", 15.0, 40.0, 25.0),
        mm("pitch", "Pitch", 4.0, 8.0, 5.0),
        mm("depth", "Thread depth", 0.5, 2.0, 1.0),
    ];
    static CUP: [ParamDef; 5] = [
        mm("foot", "Foot radius", 10.0, 20.0, 15.0),
        mm("waist", "Waist radius", 6.0, 12.0, 9.0),
        mm("lip", "Lip radius", 10.0, 20.0, 14.0),
        mm("height", "Height", 20.0, 50.0, 35.0),
        mm("wall", "Wall", 0.5, 2.5, 1.5),
    ];
    static HEALED: [ParamDef; 5] = [
        mm("width", "Width", 40.0, 90.0, 60.0),
        mm("depth", "Depth", 20.0, 40.0, 30.0),
        mm("thick", "Thickness", 5.0, 15.0, 8.0),
        mm("bore", "Hole radius", 2.0, 5.0, 3.0),
        mm("round", "Blend size", 0.5, 2.5, 1.5),
    ];
    static WALLS: [ParamDef; 3] = [
        mm("inward", "Moved in by", 0.25, 3.0, 0.5),
        mm("side", "Wall to the side", 1.0, 5.0, 2.0),
        mm("between", "Wall between", 1.0, 5.0, 2.0),
    ];
    static TEE: [ParamDef; 5] = [
        mm("main", "Pipe radius", 8.0, 16.0, 12.0),
        mm("branch", "Branch radius", 3.0, 7.0, 5.0),
        mm("length", "Pipe length", 50.0, 90.0, 70.0),
        mm("reach", "Branch height", 22.0, 40.0, 30.0),
        mm("round", "Fillet radius", 0.5, 2.5, 1.5),
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
    static SLIDER_CRANK: [ParamDef; 3] = [mm("crank", "Crank length", 15.0, 80.0, 30.0), mm("rod", "Rod length", 40.0, 160.0, 100.0), deg("angle", "Crank angle", 0.0, 720.0, 30.0)];
    static MODELS: [ModelDef; 21] = [
        ModelDef {
            id: "bracket",
            name: "Bracket",
            blurb: "Sketch, extrude, fillet the bends and corners, drill, chamfer the holes. Each dimension rebuilds only what depends on it.",
            params: &BRACKET,
            build: bracket,
            assembly: None,
        },
        ModelDef {
            id: "flange",
            name: "Flange",
            blurb: "A revolved flange with a filleted hub, chamfered edges and a bolt circle. The hub fillet is a true torus.",
            params: &FLANGE,
            build: flange,
            assembly: None,
        },
        ModelDef {
            id: "block",
            name: "Rounded block",
            blurb: "Every edge filleted. Twelve cylinders, six planes and eight sphere patches that meet exactly.",
            params: &BLOCK,
            build: rounded_block,
            assembly: None,
        },
        ModelDef {
            id: "plate",
            name: "Mounting plate",
            blurb: "All top edges filleted and all bottom edges chamfered, with each feature selecting edges by the face they bound.",
            params: &PLATE,
            build: plate,
            assembly: None,
        },
        ModelDef {
            id: "crossed",
            name: "Crossed cylinders",
            blurb: "Two cylinders intersected. Equal radii crossing square make a Steinmetz solid of volume 16r\u{b3}/3. Offsets or unequal radii need the general curve tracer.",
            params: &CROSSED,
            build: crossed,
            assembly: None,
        },
        ModelDef {
            id: "pockets",
            name: "Sliding pockets",
            blurb: "Slide the tools off the plate's edge. Many positions line faces up exactly. If the kernel refuses a position, the reason appears here.",
            params: &POCKETS,
            build: pockets,
            assembly: None,
        },
        ModelDef {
            id: "stress",
            name: "Stress cases",
            blurb: "The contacts Booleans find hardest: coplanar faces, edges and corners that only touch, and tangent surfaces. The last case is a known limit, where the kernel refuses rather than guess.",
            params: &STRESS,
            build: stress,
            assembly: None,
        },
        ModelDef {
            id: "vase",
            name: "Spline vase",
            blurb: "A sketched spline revolved, with a second spline cut out to hollow it. The curve stays an exact spline all the way to the STEP file.",
            params: &VASE,
            build: vase,
            assembly: None,
        },
        ModelDef {
            id: "joint",
            name: "Bolted joint",
            blurb: "Two flanges, bolts and nuts, each a body of its own, so they stay separate parts though they touch all around. Changing a flange's size leaves every bolt as it is, without rebuilding it.",
            params: &JOINT,
            build: joint,
            assembly: None,
        },
        ModelDef {
            id: "crank",
            name: "Slider-crank",
            blurb: "Five parts, three of the pin, on real mates found from the holes: revolutes and a slider, leaving one free motion. A test holds the piston to the crank-and-rod formula within a millionth of a millimetre, over two turns at five sets of lengths. Make the rod shorter than the crank and it stops where the rod runs out, and says so.",
            params: &SLIDER_CRANK,
            build: slider_crank,
            assembly: Some(AssemblyStep { assemble: assemble_slider_crank, drive: "angle", readout: slider_crank_readout }),
        },
        ModelDef {
            id: "panel",
            name: "Surface panel",
            blurb: "A sheet: an open curve swept into a surface with no thickness, with a window cut through it by a sketch. It has area and no volume. Turn it round and it's the same from behind.",
            params: &PANEL,
            build: panel,
            assembly: None,
        },
        ModelDef {
            id: "spring",
            name: "Helical spring",
            blurb: "A round wire carried along a helix. The section is exact, and the path is fitted to within 1e-7 of the true helix, as measured.",
            params: &SPRING,
            build: spring,
            assembly: None,
        },
        ModelDef {
            id: "screw",
            name: "Threaded shaft",
            blurb: "A square groove cut along a helix through a shaft, running off both ends. Every wall of the thread is a free-form face, through a Boolean.",
            params: &SCREW,
            build: screw,
            assembly: None,
        },
        ModelDef {
            id: "cup",
            name: "Shelled cup",
            blurb: "A lofted cup hollowed by a shell, whose inner wall is the free-form side moved along its own normals by exactly the wall thickness.",
            params: &CUP,
            build: cup,
            assembly: None,
        },
        ModelDef {
            id: "healed",
            name: "Delete face",
            blurb: "A plate with a row of holes, a fillet and a chamfer, with the middle hole and the fillet then deleted. Their neighbours grow to close the gap.",
            params: &HEALED,
            build: healed,
            assembly: None,
        },
        ModelDef {
            id: "walls",
            name: "Crossing walls",
            blurb: "Every face moved in. Past half of a thin wall the moved faces would cross, so the part is taken apart: its surroundings moved in, less each pocket moved out. The pockets join, then open through the side.",
            params: &WALLS,
            build: walls,
            assembly: None,
        },
        ModelDef {
            id: "tee",
            name: "Filleted tee",
            blurb: "A branch off a pipe with the saddle between them rounded. A true rolling-ball surface has no exact form here, so this one is fitted to within 1.5e-7 mm of the two pipes.",
            params: &TEE,
            build: tee,
            assembly: None,
        },
        ModelDef {
            id: "nozzle",
            name: "Free-form nozzle",
            blurb: "A loft from a rectangle through a circle to an ellipse, so all its sides are free-form patches, with a hole across them that follows the surface. Slower than the others: it's the hardest thing the kernel does.",
            params: &NOZZLE,
            build: nozzle,
            assembly: None,
        },
        ModelDef {
            id: "cam",
            name: "Spline cam",
            blurb: "One spline around the outline, with a bore, an elliptical window and a cross hole. Slicing a spline surface by a plane gives an exact spline back, and the cross hole has to follow the curve.",
            params: &CAM,
            build: cam,
            assembly: None,
        },
        ModelDef {
            id: "knob",
            name: "Turned knob",
            blurb: "One revolve that generates all five surface types: plane, cylinder, cone, torus and sphere.",
            params: &KNOB,
            build: knob,
            assembly: None,
        },
        ModelDef {
            id: "sector",
            name: "Ring sector",
            blurb: "A partial revolve with end caps. Past 180 degrees the kernel splits each swept face in two.",
            params: &SECTOR,
            build: sector,
            assembly: None,
        },
    ];
    &MODELS
}
