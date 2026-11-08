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
    static MODELS: [ModelDef; 4] = [
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
