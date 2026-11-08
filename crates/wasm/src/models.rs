//! The demo models. Each is a feature history written only against the kernel's
//! public API, so a demo can't hide what the kernel can't do.

use coreshape::history::*;
use coreshape::*;

#[derive(Clone, Copy, Debug)]
pub struct ParamDef {
    pub key: &'static str,
    pub label: &'static str,
    pub unit: &'static str,
    pub min: f64,
    pub max: f64,
    pub step: f64,
    pub default: f64,
    /// Names for each whole-number value, when the parameter picks from a list.
    pub choices: &'static [&'static str],
}

pub struct ModelDef {
    pub id: &'static str,
    pub name: &'static str,
    pub blurb: &'static str,
    pub params: &'static [ParamDef],
    pub history: fn() -> History,
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

/// Sketch plane XZ, with extrusion running from y = `y` back toward y = 0.
fn xz_at(y: f64) -> Frame {
    Frame { origin: v3(0.0, y, 0.0), x: Vec3::X, y: Vec3::Z, z: -Vec3::Y }
}

fn bracket() -> History {
    // width, base, upright, thick, outer radius, inner radius, corner radius, hole, rim
    let mut h = History::new();
    let s = h.sketch(
        |p| xz_at(p[0]),
        |p| {
            let (base, up, t) = (p[1], p[2], p[3]);
            Ok(vec![Profile::new(Wire::polygon(&[
                v2(0.0, 0.0),  // 0: underside
                v2(base, 0.0), // 1: end of the base
                v2(base, t),   // 2: top of the base
                v2(t, t),      // 3: inner face of the upright
                v2(t, up),     // 4: top of the upright
                v2(0.0, up),   // 5: back of the upright
            ]))])
        },
    );
    let e = h.extrude(s, |_| 0.0, |p| p[0], Mode::Add);
    h.fillet(EdgeQuery::Between(e.side(5), e.side(0)), |p| p[4]);
    h.fillet(EdgeQuery::Between(e.side(2), e.side(3)), |p| p[5]);
    h.fillet(
        EdgeQuery::Any(vec![
            EdgeQuery::Between(e.side(1), e.start_cap()),
            EdgeQuery::Between(e.side(1), e.end_cap()),
            EdgeQuery::Between(e.side(4), e.start_cap()),
            EdgeQuery::Between(e.side(4), e.end_cap()),
        ]),
        |p| p[6],
    );
    let base_holes = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let x = (p[1] + p[3]) / 2.0;
            Ok([0.27, 0.73].iter().map(|f| Profile::new(Wire::circle(v2(x, p[0] * f), p[7] / 2.0))).collect())
        },
    );
    let drilled = h.extrude(base_holes, |_| -1.0, |p| p[3] + 2.0, Mode::Remove);
    let upright_hole = h.sketch(
        |_| Frame::new(v3(-1.0, 0.0, 0.0), Vec3::X, Vec3::Y),
        |p| Ok(vec![Profile::new(Wire::circle(v2(p[0] / 2.0, (p[2] + p[3]) / 2.0), p[7] / 2.0))]),
    );
    h.extrude(upright_hole, |_| 0.0, |p| p[3] + 2.0, Mode::Remove);
    h.chamfer(
        EdgeQuery::Any(vec![
            EdgeQuery::Between(drilled.side_in_region(0, 0), e.side(2)),
            EdgeQuery::Between(drilled.side_in_region(1, 0), e.side(2)),
        ]),
        |p| p[8],
    );
    h
}

fn flange() -> History {
    // outer, hub, bore, thickness, height, foot fillet, chamfer, bolts, hole, circle
    let mut h = History::new();
    let s = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let (outer, hub, bore, t, height) = (p[0] / 2.0, p[1] / 2.0, p[2] / 2.0, p[3], p[4]);
            Ok(vec![Profile::new(Wire::polygon(&[
                v2(bore, 0.0),    // 0: underside
                v2(outer, 0.0),   // 1: rim
                v2(outer, t),     // 2: top of the flange
                v2(hub, t),       // 3: hub wall
                v2(hub, height),  // 4: hub top
                v2(bore, height), // 5: bore
            ]))])
        },
    );
    let r = h.revolve(s, |_| TAU, Mode::Add);
    h.fillet(EdgeQuery::Between(r.side(2), r.side(3)), |p| p[5]);
    h.chamfer(
        EdgeQuery::Any(vec![EdgeQuery::Between(r.side(1), r.side(2)), EdgeQuery::Between(r.side(4), r.side(5))]),
        |p| p[6],
    );
    let bolts = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let n = p[7].round() as usize;
            Ok((0..n)
                .map(|i| {
                    let a = TAU * (i as f64 + 0.5) / n as f64;
                    Profile::new(Wire::circle(v2(p[9] / 2.0 * a.cos(), p[9] / 2.0 * a.sin()), p[8] / 2.0))
                })
                .collect())
        },
    );
    h.extrude(bolts, |_| -1.0, |p| p[3] + 2.0, Mode::Remove);
    h
}

fn rounded_block() -> History {
    let mut h = History::new();
    let s = h.sketch(|_| Frame::WORLD, |p| Ok(vec![Profile::new(Wire::rect_centered(p[0], p[1]))]));
    h.extrude(s, |_| 0.0, |p| p[2], Mode::Add);
    h.fillet(EdgeQuery::filter(|_, _, _| true), |p| p[3]);
    h
}

fn plate() -> History {
    // width, depth, thick, corner, hole, inset, top fillet, bottom chamfer
    let mut h = History::new();
    let s = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let (x, y) = (p[0] / 2.0 - p[5], p[1] / 2.0 - p[5]);
            let mut profile = Profile::new(Wire::rounded_rect(p[0], p[1], p[3]));
            for c in [v2(-x, -y), v2(x, -y), v2(x, y), v2(-x, y)] {
                profile = profile.with_hole(Wire::circle(c, p[4] / 2.0));
            }
            Ok(vec![profile])
        },
    );
    let e = h.extrude(s, |_| 0.0, |p| p[2], Mode::Add);
    h.fillet(EdgeQuery::OnFace(e.end_cap()), |p| p[6]);
    h.chamfer(EdgeQuery::OnFace(e.start_cap()), |p| p[7]);
    h
}

fn crossed() -> History {
    let reach = 60.0;
    let mut h = History::new();
    let a = h.sketch(
        move |_| Frame::translation(v3(0.0, 0.0, -reach)),
        |p| Ok(vec![Profile::new(Wire::circle(v2(0.0, 0.0), p[0]))]),
    );
    h.extrude(a, |_| 0.0, move |_| 2.0 * reach, Mode::Add);
    let b = h.sketch(
        move |p| {
            let angle = p[3].to_radians();
            let dir = v3(angle.cos(), 0.0, angle.sin());
            Frame::new(v3(0.0, p[2], 0.0) - dir * reach, dir, Vec3::Y)
        },
        |p| Ok(vec![Profile::new(Wire::circle(v2(0.0, 0.0), p[1]))]),
    );
    h.extrude(b, |_| 0.0, move |_| 2.0 * reach, Mode::Intersect);
    h
}

fn pockets() -> History {
    let mut h = History::new();
    let s = h.sketch(|_| Frame::WORLD, |_| Ok(vec![Profile::new(Wire::rect(-30.0, -20.0, 60.0, 40.0))]));
    h.extrude(s, |_| 0.0, |_| 10.0, Mode::Add);
    let square = h.sketch(|_| Frame::WORLD, |p| Ok(vec![Profile::new(Wire::rect(p[0] - 8.0, -16.0, 16.0, 12.0))]));
    h.extrude(square, |p| 10.0 - p[2], |p| p[2] + 5.0, Mode::Remove);
    let round = h.sketch(|_| Frame::WORLD, |p| Ok(vec![Profile::new(Wire::circle(v2(p[0], 10.0), p[1]))]));
    h.extrude(round, |p| 10.0 - p[2], |p| p[2] + 5.0, Mode::Remove);
    h
}

fn knob() -> History {
    let mut h = History::new();
    let s = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let (body, neck, bead, dome) = (p[0], p[1], p[2], p[3]);
            let base = 8.0;
            let chamfer = 2.5;
            let shoulder = base + chamfer;
            let bead_bottom = shoulder + 6.0;
            let bead_top = bead_bottom + 2.0 * bead;
            let dome_z = bead_top + (neck - dome).abs().max(1.0);
            Ok(vec![Profile::new(
                PathBuilder::new(v2(0.0, 0.0))
                    .line_to(v2(body, 0.0)) // plane
                    .line_to(v2(body, base)) // cylinder
                    .line_to(v2(body - chamfer, shoulder)) // cone
                    .line_to(v2(neck, shoulder)) // plane
                    .line_to(v2(neck, bead_bottom)) // cylinder
                    .arc_to(v2(neck, bead_top), v2(neck, bead_bottom + bead), true) // torus
                    .line_to(v2(dome, dome_z)) // cone
                    .arc_to(v2(0.0, dome_z + dome), v2(0.0, dome_z), true) // sphere
                    .close(),
            )])
        },
    );
    h.revolve(s, |_| TAU, Mode::Add);
    h
}

fn sector() -> History {
    let mut h = History::new();
    let s = h.sketch(
        |_| Frame::WORLD,
        |p| {
            let (inner, section) = (p[0], p[1]);
            let wire = Wire::rounded_rect(section, section, section / 5.0).translated(v2(inner + section / 2.0, 0.0));
            Ok(vec![Profile::new(wire)])
        },
    );
    h.revolve(s, |p| p[2].to_radians(), Mode::Add);
    h
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

fn stress_case(p: &[f64]) -> Result<Body> {
    let at = |x: f64, y: f64, z: f64| Frame::translation(v3(x, y, z));
    let z_cylinder = |x: f64, y: f64, z: f64, r: f64, h: f64| cylinder(&at(x, y, z), r, h, 2);
    match p[0].round() as i64 {
        0 => {
            let a = box_solid(&at(-30.0, -30.0, 0.0), 60.0, 60.0, 20.0, 1)?;
            subtract(&a, &box_solid(&at(-15.0, -15.0, 8.0), 30.0, 30.0, 12.0, 2)?)
        }
        1 => {
            let a = box_solid(&at(-30.0, -15.0, 0.0), 30.0, 30.0, 30.0, 1)?;
            union(&a, &box_solid(&at(0.0, -15.0, 0.0), 30.0, 30.0, 30.0, 2)?)
        }
        2 => {
            let a = box_solid(&at(-30.0, -30.0, 0.0), 30.0, 30.0, 30.0, 1)?;
            union(&a, &box_solid(&at(0.0, 0.0, 0.0), 30.0, 30.0, 30.0, 2)?)
        }
        3 => {
            let a = box_solid(&at(-30.0, -30.0, -30.0), 30.0, 30.0, 30.0, 1)?;
            union(&a, &box_solid(&at(0.0, 0.0, 0.0), 30.0, 30.0, 30.0, 2)?)
        }
        4 => {
            let plate = box_solid(&at(-40.0, -40.0, -10.0), 80.0, 80.0, 10.0, 1)?;
            let tilt = Frame::rotation(Vec3::ZERO, v3(1.0, -1.0, 0.0), (1.0f64 / 3.0f64.sqrt()).acos());
            union(&plate, &box_solid(&tilt, 30.0, 30.0, 30.0, 2)?)
        }
        5 => {
            let a = box_solid(&at(-40.0, -30.0, 0.0), 80.0, 30.0, 20.0, 1)?;
            union(&a, &z_cylinder(0.0, 12.0, 0.0, 12.0, 20.0)?)
        }
        6 => {
            let a = box_solid(&at(-40.0, -30.0, 0.0), 80.0, 30.0, 20.0, 1)?;
            subtract(&a, &z_cylinder(0.0, -12.0, 5.0, 12.0, 15.0)?)
        }
        7 => subtract(&z_cylinder(0.0, 0.0, 0.0, 25.0, 40.0)?, &z_cylinder(0.0, 0.0, 0.0, 15.0, 40.0)?),
        8 => {
            let side = Frame::new(v3(-40.0, 0.0, 0.0), Vec3::X, Vec3::Y);
            intersect(&z_cylinder(0.0, 0.0, -40.0, 20.0, 80.0)?, &cylinder(&side, 20.0, 80.0, 3)?)
        }
        9 => {
            let half = box_solid(&at(0.0, -40.0, -5.0), 40.0, 80.0, 60.0, 2)?;
            subtract(&cone_frustum(&Frame::WORLD, 25.0, 0.0, 45.0, 1)?, &half)
        }
        10 => subtract(&sphere(&Frame::WORLD, 30.0, 1)?, &z_cylinder(0.0, 0.0, -40.0, 15.0, 80.0)?),
        11 => union(&sphere(&at(-15.0, 0.0, 0.0), 15.0, 1)?, &sphere(&at(20.0, 0.0, 0.0), 20.0, 2)?),
        12 => {
            let half = box_solid(&at(0.0, -60.0, -20.0), 60.0, 120.0, 40.0, 2)?;
            intersect(&torus(&Frame::WORLD, 35.0, 10.0, 1)?, &half)
        }
        // A cylinder internally tangent to a sphere meets it in a figure eight with a
        // double point. The kernel refuses this one rather than guess.
        _ => subtract(&sphere(&Frame::WORLD, 30.0, 1)?, &z_cylinder(12.0, 0.0, -40.0, 18.0, 80.0)?),
    }
}

fn stress() -> History {
    let mut h = History::new();
    h.custom(stress_case);
    h
}

pub fn catalog() -> &'static [ModelDef] {
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
    static MODELS: [ModelDef; 9] = [
        ModelDef {
            id: "bracket",
            name: "Bracket",
            blurb: "Sketch, extrude, fillet the bends and corners, drill, chamfer the holes. Each dimension rebuilds only what depends on it.",
            params: &BRACKET,
            history: bracket,
        },
        ModelDef {
            id: "flange",
            name: "Flange",
            blurb: "A revolved flange with a filleted hub, chamfered edges and a bolt circle. The hub fillet is a true torus.",
            params: &FLANGE,
            history: flange,
        },
        ModelDef {
            id: "block",
            name: "Rounded block",
            blurb: "Every edge filleted. Twelve cylinders, six planes and eight sphere patches that meet exactly.",
            params: &BLOCK,
            history: rounded_block,
        },
        ModelDef {
            id: "plate",
            name: "Mounting plate",
            blurb: "All top edges filleted and all bottom edges chamfered, with each feature selecting edges by the face they bound.",
            params: &PLATE,
            history: plate,
        },
        ModelDef {
            id: "crossed",
            name: "Crossed cylinders",
            blurb: "Two cylinders intersected. Equal radii crossing square make a Steinmetz solid of volume 16r\u{b3}/3. Offsets or unequal radii need the general curve tracer.",
            params: &CROSSED,
            history: crossed,
        },
        ModelDef {
            id: "pockets",
            name: "Sliding pockets",
            blurb: "Slide the tools off the plate's edge. Many positions line faces up exactly. If the kernel refuses a position, the reason appears here.",
            params: &POCKETS,
            history: pockets,
        },
        ModelDef {
            id: "stress",
            name: "Stress cases",
            blurb: "The contacts Booleans find hardest: coplanar faces, edges and corners that only touch, and tangent surfaces. The last case is a known limit, where the kernel refuses rather than guess.",
            params: &STRESS,
            history: stress,
        },
        ModelDef {
            id: "knob",
            name: "Turned knob",
            blurb: "One revolve that generates all five surface types: plane, cylinder, cone, torus and sphere.",
            params: &KNOB,
            history: knob,
        },
        ModelDef {
            id: "sector",
            name: "Ring sector",
            blurb: "A partial revolve with end caps. Past 180 degrees the kernel splits each swept face in two.",
            params: &SECTOR,
            history: sector,
        },
    ];
    &MODELS
}
