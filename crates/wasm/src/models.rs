//! The demo models. Each is a function from dimensions to a body, written only
//! against the kernel's public API, so a demo cannot hide what the kernel can't do.

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
}

pub struct ModelDef {
    pub id: &'static str,
    pub name: &'static str,
    pub blurb: &'static str,
    pub params: &'static [ParamDef],
    pub build: fn(&[f64]) -> Result<Body>,
}

const fn mm(key: &'static str, label: &'static str, min: f64, max: f64, default: f64) -> ParamDef {
    ParamDef {
        key,
        label,
        unit: "mm",
        min,
        max,
        step: 0.5,
        default,
    }
}

const fn deg(key: &'static str, label: &'static str, min: f64, max: f64, default: f64) -> ParamDef {
    ParamDef {
        key,
        label,
        unit: "\u{b0}",
        min,
        max,
        step: 1.0,
        default,
    }
}

fn flange_blank(p: &[f64]) -> Result<Body> {
    let (outer, hub, bore, thickness, height) = (p[0] / 2.0, p[1] / 2.0, p[2] / 2.0, p[3], p[4]);
    let w = Wire::polygon(&[
        v2(bore, 0.0),
        v2(outer, 0.0),
        v2(outer, thickness),
        v2(hub, thickness),
        v2(hub, height),
        v2(bore, height),
    ]);
    revolve(&Profile::new(w), &Frame::WORLD, TAU, 1)
}

fn mounting_plate(p: &[f64]) -> Result<Body> {
    let (w, d, t, r, hole, inset) = (p[0], p[1], p[2], p[3], p[4] / 2.0, p[5]);
    let (x, y) = (w / 2.0 - inset, d / 2.0 - inset);
    let mut profile = Profile::new(Wire::rounded_rect(w, d, r));
    for c in [v2(-x, -y), v2(x, -y), v2(x, y), v2(-x, y)] {
        profile = profile.with_hole(Wire::circle(c, hole));
    }
    extrude(&profile, &Frame::WORLD, t, 1)
}

/// One revolve whose profile generates all five surface types.
fn turned_knob(p: &[f64]) -> Result<Body> {
    let (body, neck, bead, dome) = (p[0], p[1], p[2], p[3]);
    let base = 8.0;
    let chamfer = 2.5;
    let shoulder = base + chamfer;
    let bead_bottom = shoulder + 6.0;
    let bead_top = bead_bottom + 2.0 * bead;
    let dome_z = bead_top + (neck - dome).abs().max(1.0);
    let w = PathBuilder::new(v2(0.0, 0.0))
        .line_to(v2(body, 0.0)) // plane
        .line_to(v2(body, base)) // cylinder
        .line_to(v2(body - chamfer, shoulder)) // cone
        .line_to(v2(neck, shoulder)) // plane
        .line_to(v2(neck, bead_bottom)) // cylinder
        .arc_to(v2(neck, bead_top), v2(neck, bead_bottom + bead), true) // torus
        .line_to(v2(dome, dome_z)) // cone
        .arc_to(v2(0.0, dome_z + dome), v2(0.0, dome_z), true) // sphere
        .close();
    revolve(&Profile::new(w), &Frame::WORLD, TAU, 1)
}

fn ring_sector(p: &[f64]) -> Result<Body> {
    let (inner, section, angle) = (p[0], p[1], p[2].to_radians());
    let rr = inner + section / 2.0;
    let profile = Profile::new(Wire::rounded_rect(section, section, section / 5.0));
    // The rounded square is centred on the origin of its sketch; move it out to
    // the mean radius before revolving.
    let moved = Profile::new(Wire {
        segs: profile
            .outer
            .segs
            .iter()
            .map(|s| Seg {
                geom: match s.geom {
                    SegGeom::Line { a, b } => SegGeom::Line {
                        a: a + v2(rr, 0.0),
                        b: b + v2(rr, 0.0),
                    },
                    SegGeom::Arc {
                        center,
                        radius,
                        a0,
                        a1,
                    } => SegGeom::Arc {
                        center: center + v2(rr, 0.0),
                        radius,
                        a0,
                        a1,
                    },
                },
                id: s.id,
            })
            .collect(),
    });
    revolve(&moved, &Frame::WORLD, angle, 1)
}

/// An L-shaped bracket: extruded profile with holes drilled through each leg.
fn bracket(p: &[f64]) -> Result<Body> {
    let (width, leg_a, leg_b, t, hole) = (p[0], p[1], p[2], p[3], p[4] / 2.0);
    let side = Wire::polygon(&[v2(0.0, 0.0), v2(leg_a, 0.0), v2(leg_a, t), v2(t, t), v2(t, leg_b), v2(0.0, leg_b)]);
    // Sketch the L in the XZ plane and extrude across the width.
    let sketch = Frame {
        origin: v3(0.0, width, 0.0),
        x: Vec3::X,
        y: Vec3::Z,
        z: -Vec3::Y,
    };
    let mut body = extrude(&Profile::new(side), &sketch, width, 1)?;
    // Two holes in the base leg, drilled along Z.
    let base_mid = (leg_a + t) / 2.0;
    for y in [width * 0.25, width * 0.75] {
        let drill = cylinder(&Frame::translation(v3(base_mid, y, -1.0)), hole, t + 2.0, 2)?;
        body = subtract(&body, &drill)?;
    }
    // One hole in the upright leg, drilled along X.
    let upright = Frame::new(v3(-1.0, width / 2.0, (leg_b + t) / 2.0), Vec3::X, Vec3::Y);
    let drill = cylinder(&upright, hole, t + 2.0, 3)?;
    subtract(&body, &drill)
}

/// A revolved flange with a bolt circle drilled through it.
fn bolted_flange(p: &[f64]) -> Result<Body> {
    let (outer, hub, bore, thickness, height) = (p[0] / 2.0, p[1] / 2.0, p[2] / 2.0, p[3], p[4]);
    let (bolts, hole, circle) = (p[5].round() as usize, p[6] / 2.0, p[7] / 2.0);
    let w = Wire::polygon(&[
        v2(bore, 0.0),
        v2(outer, 0.0),
        v2(outer, thickness),
        v2(hub, thickness),
        v2(hub, height),
        v2(bore, height),
    ]);
    let mut body = revolve(&Profile::new(w), &Frame::WORLD, TAU, 1)?;
    for i in 0..bolts {
        let a = TAU * (i as f64 + 0.5) / bolts as f64;
        let f = Frame::translation(v3(circle * a.cos(), circle * a.sin(), -1.0));
        let drill = cylinder(&f, hole, thickness + 2.0, 2 + i as u32)?;
        body = subtract(&body, &drill)?;
    }
    Ok(body)
}

/// Two cylinders crossing. Equal radii meet in exact ellipses, and anything else
/// meets in a quartic that only the general tracer can follow.
fn crossed_cylinders(p: &[f64]) -> Result<Body> {
    let (r1, r2, offset, angle) = (p[0], p[1], p[2], p[3].to_radians());
    let reach = 60.0;
    let a = cylinder(&Frame::translation(v3(0.0, 0.0, -reach)), r1, 2.0 * reach, 1)?;
    let dir = v3(angle.cos(), 0.0, angle.sin());
    let start = v3(0.0, offset, 0.0) - dir * reach;
    let b = cylinder(&Frame::new(start, dir, Vec3::Y), r2, 2.0 * reach, 2)?;
    intersect(&a, &b)
}

/// A pocket sliding across a plate's edge. Whole-number positions line faces up
/// exactly, and the kernel has to get the coplanar and tangent contacts right.
fn sliding_pocket(p: &[f64]) -> Result<Body> {
    let (offset, radius, depth) = (p[0], p[1], p[2]);
    let plate = box_solid(&Frame::translation(v3(-30.0, -20.0, 0.0)), 60.0, 40.0, 10.0, 1)?;
    let square = box_solid(&Frame::translation(v3(offset - 8.0, -16.0, 10.0 - depth)), 16.0, 12.0, depth + 5.0, 2)?;
    let round = cylinder(&Frame::translation(v3(offset, 10.0, 10.0 - depth)), radius, depth + 5.0, 3)?;
    let cut = subtract(&plate, &square)?;
    subtract(&cut, &round)
}

pub fn catalog() -> &'static [ModelDef] {
    static PARAMS_FLANGE: [ParamDef; 5] = [
        mm("outer", "Outer diameter", 40.0, 160.0, 100.0),
        mm("hub", "Hub diameter", 20.0, 80.0, 50.0),
        mm("bore", "Bore diameter", 4.0, 40.0, 26.0),
        mm("thickness", "Flange thickness", 3.0, 25.0, 12.0),
        mm("height", "Overall height", 28.0, 80.0, 40.0),
    ];
    static PARAMS_PLATE: [ParamDef; 6] = [
        mm("width", "Width", 40.0, 200.0, 120.0),
        mm("depth", "Depth", 40.0, 160.0, 80.0),
        mm("thick", "Thickness", 1.0, 30.0, 8.0),
        mm("radius", "Corner radius", 1.0, 18.0, 8.0),
        mm("hole", "Hole diameter", 2.0, 16.0, 9.0),
        mm("inset", "Hole inset", 10.0, 19.0, 14.0),
    ];
    static PARAMS_KNOB: [ParamDef; 4] = [
        mm("body", "Base radius", 16.0, 40.0, 26.0),
        mm("neck", "Neck radius", 6.0, 13.0, 10.0),
        mm("bead", "Bead radius", 1.5, 6.0, 3.5),
        mm("dome", "Dome radius", 5.0, 14.0, 9.0),
    ];
    static PARAMS_SECTOR: [ParamDef; 3] = [
        mm("inner", "Inner radius", 2.0, 60.0, 28.0),
        mm("section", "Section size", 6.0, 30.0, 16.0),
        deg("angle", "Sweep angle", 20.0, 355.0, 250.0),
    ];
    static PARAMS_BRACKET: [ParamDef; 5] = [
        mm("width", "Width", 30.0, 120.0, 70.0),
        mm("base", "Base length", 30.0, 100.0, 60.0),
        mm("upright", "Upright height", 30.0, 100.0, 50.0),
        mm("thick", "Wall thickness", 4.0, 20.0, 8.0),
        mm("hole", "Hole diameter", 3.0, 16.0, 10.0),
    ];
    static PARAMS_BOLTED: [ParamDef; 8] = [
        mm("outer", "Outer diameter", 70.0, 160.0, 110.0),
        mm("hub", "Hub diameter", 20.0, 60.0, 46.0),
        mm("bore", "Bore diameter", 4.0, 18.0, 16.0),
        mm("thickness", "Flange thickness", 3.0, 25.0, 12.0),
        mm("height", "Overall height", 28.0, 80.0, 42.0),
        ParamDef {
            key: "bolts",
            label: "Bolt count",
            unit: "",
            min: 3.0,
            max: 12.0,
            step: 1.0,
            default: 6.0,
        },
        mm("hole", "Bolt hole diameter", 3.0, 12.0, 9.0),
        mm("circle", "Bolt circle diameter", 62.0, 76.0, 76.0),
    ];
    static PARAMS_CROSSED: [ParamDef; 4] = [
        mm("r1", "Radius A", 5.0, 30.0, 20.0),
        mm("r2", "Radius B", 5.0, 30.0, 20.0),
        mm("offset", "Axis offset", 0.0, 8.0, 0.0),
        deg("angle", "Crossing angle", 30.0, 90.0, 90.0),
    ];
    static PARAMS_POCKET: [ParamDef; 3] = [
        mm("offset", "Tool position", -40.0, 40.0, 0.0),
        mm("radius", "Round pocket radius", 2.0, 10.0, 6.0),
        mm("depth", "Pocket depth", 1.0, 15.0, 5.0),
    ];
    static MODELS: [ModelDef; 8] = [
        ModelDef {
            id: "bracket",
            name: "Bracket",
            blurb: "An extruded L with three drilled holes. Each hole is a real Boolean subtract on exact cylinders.",
            params: &PARAMS_BRACKET,
            build: bracket,
        },
        ModelDef {
            id: "bolted",
            name: "Bolted flange",
            blurb: "A revolved flange with a bolt circle drilled through it. Every bolt adds one to the genus.",
            params: &PARAMS_BOLTED,
            build: bolted_flange,
        },
        ModelDef {
            id: "crossed",
            name: "Crossed cylinders",
            blurb: "The intersection of two cylinders. Equal radii crossing square make a Steinmetz solid of volume 16r\u{b3}/3. Offset or unequal radii need the general curve tracer.",
            params: &PARAMS_CROSSED,
            build: crossed_cylinders,
        },
        ModelDef {
            id: "pocket",
            name: "Sliding pockets",
            blurb: "Slide the tools off the plate's edge. Many positions line faces up exactly. If the kernel refuses a position, the error appears here.",
            params: &PARAMS_POCKET,
            build: sliding_pocket,
        },
        ModelDef {
            id: "knob",
            name: "Turned knob",
            blurb: "One revolve that generates all five surface types: plane, cylinder, cone, torus and sphere.",
            params: &PARAMS_KNOB,
            build: turned_knob,
        },
        ModelDef {
            id: "flange",
            name: "Flange blank",
            blurb: "A revolved hub and flange with a through bore. Bolt holes arrive with Booleans.",
            params: &PARAMS_FLANGE,
            build: flange_blank,
        },
        ModelDef {
            id: "plate",
            name: "Mounting plate",
            blurb: "A sketch with rounded corners and four holes, extruded. Each hole adds one to the genus.",
            params: &PARAMS_PLATE,
            build: mounting_plate,
        },
        ModelDef {
            id: "sector",
            name: "Ring sector",
            blurb: "A partial revolve with end caps. Past 180 degrees the kernel splits each swept face in two.",
            params: &PARAMS_SECTOR,
            build: ring_sector,
        },
    ];
    &MODELS
}
