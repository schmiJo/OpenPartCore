//! STEP export (ISO 10303-21, AP214 "automotive design" schema).
//!
//! Analytic surfaces and curves are written as what they are, so a Ø10 hole arrives
//! in another CAD system as a true Ø10 cylinder. Sampled intersection curves become
//! cubic B-splines that pass through the samples with matching tangents. Those are
//! approximations of the true curve, which is also how other kernels store them.

use crate::curve::Curve;
use crate::mass::shell_volume;
use crate::math::*;
use crate::surface::Surface;
use crate::topo::Body;
use std::fmt::Write;

struct Writer {
    out: String,
    next: usize,
}

fn real(x: f64) -> String {
    if x == 0.0 {
        "0.".to_string()
    } else {
        // Scientific notation with a mandatory decimal point is always valid here.
        let s = format!("{x:.16E}");
        if s.contains('.') {
            s
        } else {
            s.replacen('E', ".E", 1)
        }
    }
}

fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('\'');
    for c in s.chars() {
        if c == '\'' {
            out.push_str("''");
        } else if c.is_ascii() && !c.is_ascii_control() {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    out.push('\'');
    out
}

impl Writer {
    fn entity(&mut self, body: &str) -> usize {
        let id = self.next;
        self.next += 1;
        let _ = writeln!(self.out, "#{id}={body};");
        id
    }

    fn point(&mut self, p: Vec3) -> usize {
        self.entity(&format!("CARTESIAN_POINT('',({},{},{}))", real(p.x), real(p.y), real(p.z)))
    }

    fn direction(&mut self, d: Vec3) -> usize {
        self.entity(&format!("DIRECTION('',({},{},{}))", real(d.x), real(d.y), real(d.z)))
    }

    fn axis(&mut self, f: &Frame) -> usize {
        let o = self.point(f.origin);
        let z = self.direction(f.z);
        let x = self.direction(f.x);
        self.entity(&format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }

    fn curve(&mut self, c: &Curve) -> usize {
        match c {
            Curve::Line { origin, dir } => {
                let p = self.point(*origin);
                let d = self.direction(*dir);
                let v = self.entity(&format!("VECTOR('',#{d},1.)"));
                self.entity(&format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle { frame, radius } => {
                let a = self.axis(frame);
                self.entity(&format!("CIRCLE('',#{a},{})", real(*radius)))
            }
            Curve::Ellipse { frame, major, minor } => {
                let a = self.axis(frame);
                self.entity(&format!("ELLIPSE('',#{a},{},{})", real(*major), real(*minor)))
            }
            Curve::Sampled(s) => {
                // Each cubic Hermite span becomes a Bezier span. With knots spaced by
                // chord length the tangents match exactly across joins, so the curve
                // is C1 and each interior knot only needs multiplicity 2.
                let n = s.points.len();
                let mut poles = Vec::with_capacity(3 * n);
                let mut knots = vec![0.0];
                let mut u = 0.0;
                for i in 0..n - 1 {
                    let (a, b) = (s.points[i], s.points[i + 1]);
                    let l = a.dist(b);
                    if i == 0 {
                        poles.push(a);
                    }
                    poles.push(a + s.tangents[i] * (l / 3.0));
                    poles.push(b - s.tangents[i + 1] * (l / 3.0));
                    u += l;
                    knots.push(u);
                }
                poles.push(s.points[n - 1]);
                let ids: Vec<String> = poles.iter().map(|&p| format!("#{}", self.point(p))).collect();
                let mut mults = vec!["2".to_string(); knots.len()];
                mults[0] = "4".into();
                let last = mults.len() - 1;
                mults[last] = "4".into();
                let knot_text: Vec<String> = knots.iter().map(|&k| real(k)).collect();
                self.entity(&format!(
                    "B_SPLINE_CURVE_WITH_KNOTS('',3,({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                    ids.join(","),
                    mults.join(","),
                    knot_text.join(",")
                ))
            }
        }
    }

    fn surface(&mut self, s: &Surface) -> usize {
        match s {
            Surface::Plane(f) => {
                let a = self.axis(f);
                self.entity(&format!("PLANE('',#{a})"))
            }
            Surface::Cylinder { frame, radius } => {
                let a = self.axis(frame);
                self.entity(&format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Cone {
                frame,
                radius,
                half_angle,
            } => {
                let a = self.axis(frame);
                self.entity(&format!("CONICAL_SURFACE('',#{a},{},{})", real(*radius), real(*half_angle)))
            }
            Surface::Sphere { frame, radius } => {
                let a = self.axis(frame);
                self.entity(&format!("SPHERICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Torus { frame, major, minor } => {
                let a = self.axis(frame);
                self.entity(&format!("TOROIDAL_SURFACE('',#{a},{},{})", real(*major), real(*minor)))
            }
        }
    }
}

/// Writes a body as a STEP file. Dimensions are in millimetres.
pub fn write_step(body: &Body, name: &str) -> String {
    let mut w = Writer {
        out: String::new(),
        next: 1,
    };
    let product = quote(name);

    // Product structure: required by the schema even for one part.
    let app = w.entity("APPLICATION_CONTEXT('automotive design')");
    w.entity(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2001,#{app})"));
    let mech = w.entity(&format!("MECHANICAL_CONTEXT('',#{app},'mechanical')"));
    let prod = w.entity(&format!("PRODUCT({product},{product},'',(#{mech}))"));
    w.entity(&format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part','',(#{prod}))"));
    let formation = w.entity(&format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let pctx = w.entity(&format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"));
    let def = w.entity(&format!("PRODUCT_DEFINITION('design','',#{formation},#{pctx})"));
    let def_shape = w.entity(&format!("PRODUCT_DEFINITION_SHAPE('','',#{def})"));

    // Units and tolerance.
    let length = w.entity("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
    let angle = w.entity("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let solid = w.entity("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let accuracy = w.entity(&format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{length},'distance_accuracy_value','confusion accuracy')",
        real(TOL)
    ));
    let ctx = w.entity(&format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{accuracy}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solid}))REPRESENTATION_CONTEXT('Context','3D'))"
    ));

    // Geometry and topology.
    let vertex_ids: Vec<usize> = body
        .vertices
        .iter()
        .map(|&p| {
            let c = w.point(p);
            w.entity(&format!("VERTEX_POINT('',#{c})"))
        })
        .collect();
    let edge_ids: Vec<usize> = body
        .edges
        .iter()
        .map(|e| {
            let c = w.curve(&e.curve);
            w.entity(&format!(
                "EDGE_CURVE('',#{},#{},#{c},.T.)",
                vertex_ids[e.v0], vertex_ids[e.v1]
            ))
        })
        .collect();

    // Whether each shell is a void. Void shells' faces point into the void, and STEP
    // wants them written as an ordinary closed shell around the void, turned inside out.
    let is_void: Vec<bool> = (0..body.shells.len()).map(|s| shell_volume(body, s) < 0.0).collect();

    let write_shell = |w: &mut Writer, shell: usize, invert: bool| -> usize {
        let mut faces = Vec::with_capacity(body.shells[shell].len());
        for &f in &body.shells[shell] {
            let face = &body.faces[f];
            let surface = w.surface(&face.surface);
            let mut bounds = Vec::with_capacity(face.loops.len());
            for (l, lp) in face.loops.iter().enumerate() {
                let oriented: Vec<String> = lp
                    .iter()
                    .map(|c| {
                        // The second flag says whether the loop runs along the edge.
                        let sense = if c.reversed { ".F." } else { ".T." };
                        format!("#{}", w.entity(&format!("ORIENTED_EDGE('',*,*,#{},{sense})", edge_ids[c.edge])))
                    })
                    .collect();
                let edge_loop = w.entity(&format!("EDGE_LOOP('',({}))", oriented.join(",")));
                let kind = if l == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                let sense = if invert { ".F." } else { ".T." };
                bounds.push(format!("#{}", w.entity(&format!("{kind}('',#{edge_loop},{sense})"))));
            }
            let same = face.flipped == invert;
            let same = if same { ".T." } else { ".F." };
            faces.push(format!(
                "#{}",
                w.entity(&format!("ADVANCED_FACE('',({}),#{surface},{same})", bounds.join(",")))
            ));
        }
        w.entity(&format!("CLOSED_SHELL('',({}))", faces.join(",")))
    };

    // Attach each void to the outer shell containing it. With one outer shell, which
    // is by far the common case, that is trivial; otherwise test by bounding box.
    let shell_box = |s: usize| {
        let mut b = Aabb::EMPTY;
        for &f in &body.shells[s] {
            b.union(&body.face_aabb(f));
        }
        b
    };
    let outers: Vec<usize> = (0..body.shells.len()).filter(|&s| !is_void[s]).collect();
    let mut solids = Vec::new();
    for &outer in &outers {
        let voids: Vec<usize> = (0..body.shells.len())
            .filter(|&s| {
                is_void[s]
                    && (outers.len() == 1 || {
                        let (a, b) = (shell_box(outer), shell_box(s));
                        a.contains(b.min) && a.contains(b.max)
                    })
            })
            .collect();
        let shell = write_shell(&mut w, outer, false);
        if voids.is_empty() {
            solids.push(format!("#{}", w.entity(&format!("MANIFOLD_SOLID_BREP('',#{shell})"))));
        } else {
            let oriented: Vec<String> = voids
                .iter()
                .map(|&v| {
                    let s = write_shell(&mut w, v, true);
                    format!("#{}", w.entity(&format!("ORIENTED_CLOSED_SHELL('',*,#{s},.F.)")))
                })
                .collect();
            solids.push(format!(
                "#{}",
                w.entity(&format!("BREP_WITH_VOIDS('',#{shell},({}))", oriented.join(",")))
            ));
        }
    }

    let origin = w.axis(&Frame::WORLD);
    let rep = w.entity(&format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},{}),#{ctx})",
        solids.join(",")
    ));
    w.entity(&format!("SHAPE_DEFINITION_REPRESENTATION(#{def_shape},#{rep})"));

    format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('CoreShape model'),'2;1');\nFILE_NAME({product},'2026-01-01T00:00:00',(''),(''),'CoreShape','CoreShape','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
        w.out
    )
}
