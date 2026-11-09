//! STEP export (ISO 10303-21, AP214 "automotive design" schema).
//!
//! Analytic surfaces and curves are written as what they are, so a Ø10 hole arrives
//! in another CAD system as a true Ø10 cylinder. Sampled intersection curves become
//! cubic B-splines that pass through the samples with matching tangents. Those are
//! approximations of the true curve, which is also how other kernels store them.

use crate::error::Result;
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

/// A sphere for writing one face, with its poles and seam away from that face where that's possible.
///
/// STEP gives a sphere an axis, which means nothing to this kernel but a lot to a reader. A loop around a pole
/// isn't closed in the reader's parameters, and OpenCASCADE repairs that by guessing which side of the loop the
/// face is on. It guessed wrong on a trimmed sphere with an ordinary loop, reading the rest of the sphere instead.
/// With no pole inside, a loop is plainly closed and there's nothing to guess. A whole hemisphere has no such
/// axis, since every line through the center meets it, and it keeps the one it has, which reads fine.
/// How far a moved free-form face's fit in a file may be from the true one.
pub const OFFSET_FIT: f64 = 1e-5;

/// A moved free-form face as an ordinary B-spline, for a file. STEP has `OFFSET_SURFACE`, and this kernel reads it, but
/// OpenCASCADE refuses one whose basis isn't C1, as "Offset with no C1 Surface", and a circle's double knots aren't. A moved
/// face can also reach past the end of its basis, where a neighbour moved away from it, which `OFFSET_SURFACE` can't say.
/// So it's written as a cubic through the true surface, stopping at the basis's own knots and ends, within `OFFSET_FIT`,
/// which is well inside what files are read with. The kernel's own surface stays exact.
fn offset_as_spline(body: &Body, f: usize) -> Option<Result<Surface>> {
    let face = &body.faces[f];
    let Surface::Free { patch, .. } = &face.surface else { return None };
    if patch.offset() == 0.0 {
        return None;
    }
    Some((|| {
        let ((u0, u1), (v0, v1)) = patch.domain();
        let corner_lo = patch.chart_of(u0, v0);
        let corner_hi = patch.chart_of(u1, v1);
        // The whole of the surface, and further only where the face runs past its end, along its tangent.
        let (face_lo, face_hi) = body.face_chart_bounds(f);
        let room = 1e-3 + 0.01 * (face_hi - face_lo).norm();
        let past = |face: f64, end: f64, up: bool| if up == (face > end) && (face - end).abs() > 1e-7 { face + if up { room } else { -room } } else { end };
        let lo = v2(past(face_lo.x, corner_lo.x, false), past(face_lo.y, corner_lo.y, false));
        let hi = v2(past(face_hi.x, corner_hi.x, true), past(face_hi.y, corner_hi.y, true));
        let inner = patch.interior_knots();
        let stops = |a: f64, b: f64, list: Vec<f64>| -> Vec<f64> {
            let mut out = vec![a];
            out.extend(list.into_iter().filter(|x| *x > a + 1e-9 && *x < b - 1e-9));
            out.push(b);
            out.sort_by(f64::total_cmp);
            out.dedup_by(|p, q| (*p - *q).abs() < 1e-9);
            out
        };
        let (mut list_x, mut list_y) = (vec![corner_lo.x, corner_hi.x], vec![corner_lo.y, corner_hi.y]);
        list_x.extend(inner[0].iter().map(|k| patch.chart_of(*k, v0).x));
        list_y.extend(inner[1].iter().map(|k| patch.chart_of(u0, *k).y));
        let (xs, ys) = (stops(lo.x, hi.x, list_x), stops(lo.y, hi.y, list_y));
        let at = |x: f64, y: f64| face.surface.from_chart(&face.chart, v2(x, y));
        // A polynomial through well spaced points gains digits very quickly on a smooth surface, so a low degree on few
        // pieces is the aim. A file with thousands of small cubics in it is worse for everybody than one with a few of these.
        let (mut xs, mut ys) = (xs, ys);
        for _ in 0..4 {
            for degree in [3usize, 5, 7, 9] {
                let grid = BezierGrid::through(&xs, &ys, degree, &at)?;
                if grid.worst_against(&at) <= OFFSET_FIT {
                    return grid.into_patch().map(|patch| Surface::Free { frame: crate::math::Frame::WORLD, patch: std::sync::Arc::new(patch) });
                }
            }
            let halved = |list: &[f64]| -> Vec<f64> {
                let mut out = Vec::with_capacity(2 * list.len());
                for w in list.windows(2) {
                    out.push(w[0]);
                    out.push((w[0] + w[1]) / 2.0);
                }
                out.push(list[list.len() - 1]);
                out
            };
            xs = halved(&xs);
            ys = halved(&ys);
        }
        Err(crate::error::Error::Unsupported(format!("a moved free-form face can't be fitted within {OFFSET_FIT:e}, even on {} pieces", (xs.len() - 1) * (ys.len() - 1))))
    })())
}

/// Bezier pieces on a grid, of one degree, through the values of a function at Chebyshev-Lobatto points, which share their
/// end points, so that neighbouring pieces agree all along the line between them.
struct BezierGrid {
    xs: Vec<f64>,
    ys: Vec<f64>,
    degree: usize,
    /// Control points by x, then by y.
    points: Vec<Vec<Vec3>>,
}

impl BezierGrid {
    fn nodes(degree: usize) -> Vec<f64> {
        (0..=degree).map(|i| 0.5 - 0.5 * (crate::math::PI * i as f64 / degree as f64).cos()).collect()
    }

    /// The inverse of the matrix taking control points to values at the nodes. Bernstein polynomials at these nodes are
    /// well conditioned at these degrees.
    fn inverse(degree: usize) -> Vec<Vec<f64>> {
        let n = degree + 1;
        let nodes = Self::nodes(degree);
        let mut a: Vec<Vec<f64>> = nodes
            .iter()
            .enumerate()
            .map(|(row, t)| {
                let mut line = bernstein(degree, *t);
                line.extend((0..n).map(|c| if c == row { 1.0 } else { 0.0 }));
                line
            })
            .collect();
        for col in 0..n {
            let pivot = (col..n).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs())).unwrap();
            a.swap(col, pivot);
            let head = a[col][col];
            for v in a[col].iter_mut() {
                *v /= head;
            }
            for row in 0..n {
                if row != col {
                    let factor = a[row][col];
                    if factor != 0.0 {
                        let pivot_row = a[col].clone();
                        for (v, w) in a[row].iter_mut().zip(&pivot_row) {
                            *v -= factor * w;
                        }
                    }
                }
            }
        }
        a.into_iter().map(|line| line[n..].to_vec()).collect()
    }

    fn through(xs: &[f64], ys: &[f64], degree: usize, at: &dyn Fn(f64, f64) -> Vec3) -> Result<BezierGrid> {
        let nodes = Self::nodes(degree);
        let inverse = Self::inverse(degree);
        let spread = |stops: &[f64]| -> Vec<f64> {
            let mut out = Vec::with_capacity(degree * (stops.len() - 1) + 1);
            for (k, w) in stops.windows(2).enumerate() {
                for (i, t) in nodes.iter().enumerate() {
                    if i > 0 || k == 0 {
                        out.push(w[0] + (w[1] - w[0]) * t);
                    }
                }
            }
            out
        };
        let (along_x, along_y) = (spread(xs), spread(ys));
        // Turns values at the nodes of each piece of a line into that line's control points.
        let solve = |values: &[Vec3]| -> Vec<Vec3> {
            let pieces = (values.len() - 1) / degree;
            let mut out = vec![Vec3::ZERO; values.len()];
            for k in 0..pieces {
                let part = &values[k * degree..=(k + 1) * degree];
                for (i, row) in inverse.iter().enumerate() {
                    out[k * degree + i] = row.iter().zip(part).fold(Vec3::ZERO, |sum, (c, v)| sum + *v * *c);
                }
            }
            out
        };
        let lines: Vec<Vec<Vec3>> = along_x.iter().map(|x| solve(&along_y.iter().map(|y| at(*x, *y)).collect::<Vec<_>>())).collect();
        let count_y = along_y.len();
        let across: Vec<Vec<Vec3>> = (0..count_y).map(|j| solve(&lines.iter().map(|l| l[j]).collect::<Vec<_>>())).collect();
        let points: Vec<Vec<Vec3>> = (0..along_x.len()).map(|i| across.iter().map(|l| l[i]).collect()).collect();
        Ok(BezierGrid { xs: xs.to_vec(), ys: ys.to_vec(), degree, points })
    }

    fn eval(&self, x: f64, y: f64) -> Vec3 {
        let place = |stops: &[f64], v: f64| {
            let k = stops.partition_point(|s| *s <= v).clamp(1, stops.len() - 1) - 1;
            (k, (v - stops[k]) / (stops[k + 1] - stops[k]))
        };
        let ((i, s), (j, t)) = (place(&self.xs, x), place(&self.ys, y));
        let (bs, bt) = (bernstein(self.degree, s), bernstein(self.degree, t));
        let mut sum = Vec3::ZERO;
        for (a, u) in bs.iter().enumerate() {
            for (b, w) in bt.iter().enumerate() {
                sum = sum + self.points[i * self.degree + a][j * self.degree + b] * (u * w);
            }
        }
        sum
    }

    /// The largest distance from what it fits, between the points it goes through.
    fn worst_against(&self, at: &dyn Fn(f64, f64) -> Vec3) -> f64 {
        let n = 2 * self.degree + 1;
        let mut worst = 0.0f64;
        for w in self.xs.windows(2) {
            for z in self.ys.windows(2) {
                for a in 0..n {
                    for b in 0..n {
                        let x = w[0] + (w[1] - w[0]) * (a as f64 + 0.5) / n as f64;
                        let y = z[0] + (z[1] - z[0]) * (b as f64 + 0.5) / n as f64;
                        worst = worst.max(self.eval(x, y).dist(at(x, y)));
                    }
                }
            }
        }
        worst
    }

    fn into_patch(self) -> Result<crate::patch::Patch> {
        let knots = |stops: &[f64]| -> Vec<f64> {
            let mut out = vec![stops[0]; self.degree + 1];
            for s in &stops[1..stops.len() - 1] {
                out.extend(vec![*s; self.degree]);
            }
            out.extend(vec![stops[stops.len() - 1]; self.degree + 1]);
            out
        };
        let (count_x, count_y) = (self.points.len(), self.points[0].len());
        let points: Vec<Vec3> = self.points.iter().flatten().copied().collect();
        let weights = vec![1.0; points.len()];
        crate::patch::Patch::new((self.degree, self.degree), (knots(&self.xs), knots(&self.ys)), (count_x, count_y), points, weights)
    }
}

/// The Bernstein polynomials of a degree at `t`.
fn bernstein(degree: usize, t: f64) -> Vec<f64> {
    let mut out = vec![0.0; degree + 1];
    out[0] = 1.0;
    for level in 1..=degree {
        let mut carry = 0.0;
        for slot in out.iter_mut().take(level) {
            let old = *slot;
            *slot = carry + (1.0 - t) * old;
            carry = t * old;
        }
        out[level] = carry;
    }
    out
}

fn sphere_axis_off_face(body: &Body, index: &crate::boolean::classify::Index, f: usize) -> Option<Surface> {
    let face = &body.faces[f];
    let Surface::Sphere { frame, radius } = face.surface else {
        return None;
    };
    let mut boundary = Vec::new();
    for c in face.loops.iter().flatten() {
        for k in 0..12 {
            boundary.push((body.coedge_point(*c, k as f64 / 12.0) - frame.origin).normalized());
        }
    }
    let on_face = |d: Vec3| index.locate_on_face(f, frame.origin + d * radius, 1e-6) != crate::boolean::classify::FaceLoc::Outside;
    // Evenly spread directions, so the choice doesn't depend on the order anything happened in.
    const CANDIDATES: usize = 200;
    let mut best: Option<(f64, Vec3)> = None;
    for k in 0..CANDIDATES {
        let z = 1.0 - (k as f64 + 0.5) / CANDIDATES as f64;
        let (s, c) = (k as f64 * 2.399963229728653).sin_cos();
        let rho = (1.0 - z * z).max(0.0).sqrt();
        let d = frame.x * (rho * c) + frame.y * (rho * s) + frame.z * z;
        if on_face(d) || on_face(-d) {
            continue;
        }
        let clearance = boundary.iter().fold(f64::INFINITY, |m, b| m.min(1.0 - b.dot(d).abs()));
        if best.is_none_or(|b| clearance > b.0) {
            best = Some((clearance, d));
        }
    }
    let (clearance, axis) = best?;
    // A pole almost touching the boundary would be no improvement.
    if clearance < 0.005 {
        return None;
    }
    // The seam runs through the frame's x, so put that on the far side from the face.
    let middle = boundary.iter().fold(Vec3::ZERO, |m, b| m + *b);
    let across = middle - axis * middle.dot(axis);
    let x = if across.norm() > 1e-9 { -across.normalized() } else { axis.any_perpendicular() };
    Some(Surface::Sphere { frame: Frame::new(frame.origin, axis, x), radius })
}

/// A knot vector as STEP wants it: multiplicities and distinct values.
fn knot_lists(knots: &[f64]) -> (String, String) {
    let (mut values, mut mults): (Vec<String>, Vec<String>) = (Vec::new(), Vec::new());
    let mut i = 0;
    while i < knots.len() {
        let mut j = i;
        while j < knots.len() && knots[j] == knots[i] {
            j += 1;
        }
        values.push(real(knots[i]));
        mults.push((j - i).to_string());
        i = j;
    }
    (mults.join(","), values.join(","))
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

    /// A B-spline curve, with the complex form STEP uses when it has weights.
    fn patch(&mut self, frame: &crate::math::Frame, patch: &crate::patch::Patch) -> usize {
        let (nu, nv) = patch.count();
        let (du, dv) = patch.degree();
        let mut rows = Vec::with_capacity(nu);
        for i in 0..nu {
            let row: Vec<String> = (0..nv).map(|j| format!("#{}", self.point(frame.to_world(patch.points()[i * nv + j])))).collect();
            rows.push(format!("({})", row.join(",")));
        }
        let rows = rows.join(",");
        let (mu, ku) = knot_lists(patch.knots().0);
        let (mv, kv) = knot_lists(patch.knots().1);
        if patch.is_polynomial() {
            return self.entity(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{du},{dv},({rows}),.UNSPECIFIED.,.F.,.F.,.F.,({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.)"
            ));
        }
        let weights: Vec<String> = (0..nu)
            .map(|i| format!("({})", (0..nv).map(|j| real(patch.weights()[i * nv + j])).collect::<Vec<_>>().join(",")))
            .collect();
        self.entity(&format!(
            "(BOUNDED_SURFACE() B_SPLINE_SURFACE({du},{dv},({rows}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE())",
            weights.join(",")
        ))
    }

    fn spline(&mut self, c: &crate::spline::Spline) -> usize {
        let ids: Vec<String> = c.points().iter().map(|&p| format!("#{}", self.point(p))).collect();
        let (mults, values) = knot_lists(c.knots());
        let ids = ids.join(",");
        let degree = c.degree();
        if c.is_polynomial() {
            return self.entity(&format!("B_SPLINE_CURVE_WITH_KNOTS('',{degree},({ids}),.UNSPECIFIED.,.F.,.F.,({mults}),({values}),.UNSPECIFIED.)"));
        }
        let weights: Vec<String> = c.weights().iter().map(|&w| real(w)).collect();
        self.entity(&format!(
            "(BOUNDED_CURVE() B_SPLINE_CURVE({degree},({ids}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(({mults}),({values}),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM(''))",
            weights.join(",")
        ))
    }

    fn curve(&mut self, c: &Curve) -> usize {
        match c {
            Curve::Spline(s) => self.spline(s),
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
            Surface::Free { frame, patch } => self.patch(frame, patch),
            Surface::Extruded { frame, curve } => {
                let lifted = curve.spline.mapped(|q| frame.point(q.x, q.y, 0.0));
                let c = self.spline(&lifted);
                let d = self.direction(frame.z);
                let v = self.entity(&format!("VECTOR('',#{d},1.)"));
                self.entity(&format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})"))
            }
            Surface::Revolved { frame, curve } => {
                let lifted = curve.spline.mapped(|q| frame.origin + frame.x * q.x + frame.z * q.y);
                let c = self.spline(&lifted);
                let o = self.point(frame.origin);
                let d = self.direction(frame.z);
                let a = self.entity(&format!("AXIS1_PLACEMENT('',#{o},#{d})"));
                self.entity(&format!("SURFACE_OF_REVOLUTION('',#{c},#{a})"))
            }
            Surface::Plane(f) => {
                let a = self.axis(f);
                self.entity(&format!("PLANE('',#{a})"))
            }
            Surface::Cylinder { frame, radius } => {
                let a = self.axis(frame);
                self.entity(&format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Cone { frame, radius, half_angle } => {
                let a = self.axis(frame);
                self.entity(&format!("CONICAL_SURFACE('',#{a},{},{})", real(*radius), real(*half_angle)))
            }
            Surface::Sphere { frame, radius } => {
                let a = self.axis(frame);
                self.entity(&format!("SPHERICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Torus { frame, major, minor } if major.abs() < *minor => {
                // STEP has a separate entity for a torus that crosses itself, and a flag
                // for which part of it is meant: the outer apple or the inner lemon.
                let a = self.axis(frame);
                let outer = if *major >= 0.0 { ".T." } else { ".F." };
                self.entity(&format!(
                    "DEGENERATE_TOROIDAL_SURFACE('',#{a},{},{},{outer})",
                    real(major.abs()),
                    real(*minor)
                ))
            }
            Surface::Torus { frame, major, minor } => {
                let a = self.axis(frame);
                self.entity(&format!("TOROIDAL_SURFACE('',#{a},{},{})", real(*major), real(*minor)))
            }
        }
    }
}

/// The faces of a shell in depth-first order across shared edges.
///
/// A file's face order carries no meaning in STEP, but OpenCASCADE's shell checker
/// depends on it: a shell that touches itself along a line (a hole tangent to a wall)
/// can flip between valid and "Unorientable" when only the order changes. Depth-first
/// order passed on every start face tried, against about half of random orders. That is
/// an observed property of that checker, not a rule from the standard.
fn connected_order(body: &Body, shell: &[usize], uses: &[Vec<(usize, usize, usize, bool)>]) -> Vec<usize> {
    let mut seen = crate::hash::HashSet::default();
    let mut order = Vec::with_capacity(shell.len());
    for &start in shell {
        let mut stack = vec![start];
        while let Some(f) = stack.pop() {
            if !seen.insert(f) {
                continue;
            }
            order.push(f);
            let edges: Vec<usize> = body.faces[f].loops.iter().flatten().map(|c| c.edge).collect();
            for &e in edges.iter().rev() {
                for u in &uses[e] {
                    if !seen.contains(&u.0) {
                        stack.push(u.0);
                    }
                }
            }
        }
    }
    order
}

/// Writes a body as a STEP file. Dimensions are in millimetres.
pub fn write_step(body: &Body, name: &str) -> String {
    write_step_bodies(&[body], name)
}

/// Writes several bodies as one STEP file, each as its own solids where it already is, whether they touch or not. There's
/// no assembly in it: one product holding them all, which any reader comes back from with the same solids in the same places.
pub fn write_step_bodies(bodies: &[&Body], name: &str) -> String {
    let mut w = Writer { out: String::new(), next: 1 };
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
    let mut solids = Vec::new();
    for body in bodies {
        solids.extend(write_solids(&mut w, body));
    }

    let origin = w.axis(&Frame::WORLD);
    let rep = w.entity(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},{}),#{ctx})", solids.join(",")));
    w.entity(&format!("SHAPE_DEFINITION_REPRESENTATION(#{def_shape},#{rep})"));

    format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('CoreShape model'),'2;1');\nFILE_NAME({product},'2026-01-01T00:00:00',(''),(''),'CoreShape','CoreShape','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
        w.out
    )
}

/// A body's solids: each of its outer shells, with the voids inside it. Returns their references.
fn write_solids(w: &mut Writer, body: &Body) -> Vec<String> {
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
            w.entity(&format!("EDGE_CURVE('',#{},#{},#{c},.T.)", vertex_ids[e.v0], vertex_ids[e.v1]))
        })
        .collect();

    // Whether each shell is a void. Void shells' faces point into the void, and STEP
    // wants them written as an ordinary closed shell around the void, turned inside out.
    let is_void: Vec<bool> = (0..body.shells.len()).map(|s| shell_volume(body, s) < 0.0).collect();

    let uses = body.edge_uses();
    let index = crate::boolean::classify::Index::new(body);
    let write_shell = |w: &mut Writer, shell: usize, invert: bool| -> usize {
        let mut faces = Vec::with_capacity(body.shells[shell].len());
        for f in connected_order(body, &body.shells[shell], &uses) {
            let face = &body.faces[f];
            let surface = match sphere_axis_off_face(body, &index, f) {
                Some(s) => w.surface(&s),
                None => match (offset_as_spline(body, f), &face.surface) {
                    (Some(Ok(s)), _) => w.surface(&s),
                    // No fit within the tolerance: the exact form, which not every reader can take, rather than a worse fit.
                    (Some(Err(_)), Surface::Free { frame, patch }) => {
                        let base = w.patch(frame, patch);
                        w.entity(&format!("OFFSET_SURFACE('',#{base},{},.F.)", real(patch.offset())))
                    }
                    _ => w.surface(&face.surface),
                },
            };
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
            faces
                .push(format!("#{}", w.entity(&format!("ADVANCED_FACE('',({}),#{surface},{same})", bounds.join(",")))));
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
        let shell = write_shell(w, outer, false);
        if voids.is_empty() {
            solids.push(format!("#{}", w.entity(&format!("MANIFOLD_SOLID_BREP('',#{shell})"))));
        } else {
            let oriented: Vec<String> = voids
                .iter()
                .map(|&v| {
                    let s = write_shell(w, v, true);
                    format!("#{}", w.entity(&format!("ORIENTED_CLOSED_SHELL('',*,#{s},.F.)")))
                })
                .collect();
            solids.push(format!("#{}", w.entity(&format!("BREP_WITH_VOIDS('',#{shell},({}))", oriented.join(",")))));
        }
    }
    solids
}
