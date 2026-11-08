//! Reads solids from a STEP file: planes, cylinders, cones, spheres and tori, and curves swept along a
//! line or around an axis. Any other surface refuses the whole file, naming the surface and how many faces
//! use it, so the result is never a solid with faces missing.
//!
//! The file's topology isn't what this kernel uses. In a file, a full cylinder is usually one face with a
//! seam, whose circles are edges that start and end at one vertex. Here an edge has two different ends, and
//! a curved face stays within half a turn. So closed edges are cut in two, seams are dropped, and wide faces
//! are cut along walls through their axis. Each piece keeps a name from the face it came from.
//!
//! Positions in a file are only as good as what wrote them, and this kernel's tolerance may be tighter. So
//! each vertex is moved onto the surfaces around it, which refuses a file that's too far off. Straight edges
//! are rebuilt between their vertices, and a B-spline edge lying between two surfaces is resampled onto
//! both, as an intersection curve here would be.

use crate::boolean::simplify::within_half_turn;
use crate::boolean::ssi::sampled_from_points;
use crate::boolean::trace::{trace_faces, FaceChart};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::{cut_lines, Patch};
use crate::spline::{Hom, PlaneCurve, Spline};
use crate::surface::Surface;
use crate::topo::{Body, Coedge, Edge, FaceTag};
use crate::validate::validate;
use std::collections::BTreeMap;
use std::sync::Arc;

// ----- The file's syntax -----

#[derive(Clone, Debug, PartialEq)]
enum Val {
    Ref(usize),
    Num(f64),
    Str(String),
    Enum(String),
    List(Vec<Val>),
    /// `$` or `*`.
    Empty,
    /// A typed value such as `LENGTH_MEASURE(1.E-7)`.
    Typed(String, Vec<Val>),
}

/// One instance: its type names and their arguments. Most have one, and a complex instance has several.
type Instance = Vec<(String, Vec<Val>)>;

fn bad(why: impl Into<String>) -> Error {
    Error::InvalidInput(format!("this STEP file can't be read: {}", why.into()))
}

struct Scan<'a> {
    text: &'a [u8],
    at: usize,
}

impl Scan<'_> {
    fn skip(&mut self) {
        loop {
            while self.at < self.text.len() && self.text[self.at].is_ascii_whitespace() {
                self.at += 1;
            }
            if self.text[self.at..].starts_with(b"/*") {
                match self.text[self.at..].windows(2).position(|w| w == b"*/") {
                    Some(end) => self.at += end + 2,
                    None => self.at = self.text.len(),
                }
            } else {
                return;
            }
        }
    }

    fn peek(&mut self) -> Option<u8> {
        self.skip();
        self.text.get(self.at).copied()
    }

    fn eat(&mut self, c: u8) -> Result<()> {
        if self.peek() == Some(c) {
            self.at += 1;
            Ok(())
        } else {
            Err(bad(format!("expected '{}' at byte {}", c as char, self.at)))
        }
    }

    fn word(&mut self) -> String {
        self.skip();
        let start = self.at;
        while self.at < self.text.len() && (self.text[self.at].is_ascii_alphanumeric() || self.text[self.at] == b'_' || self.text[self.at] == b'-') {
            self.at += 1;
        }
        String::from_utf8_lossy(&self.text[start..self.at]).into_owned()
    }

    fn arguments(&mut self, depth: usize) -> Result<Vec<Val>> {
        if depth > 64 {
            return Err(bad("lists are nested too deeply"));
        }
        self.eat(b'(')?;
        let mut out = Vec::new();
        if self.peek() == Some(b')') {
            self.at += 1;
            return Ok(out);
        }
        loop {
            out.push(self.value(depth + 1)?);
            match self.peek() {
                Some(b',') => self.at += 1,
                Some(b')') => {
                    self.at += 1;
                    return Ok(out);
                }
                _ => return Err(bad(format!("expected ',' or ')' at byte {}", self.at))),
            }
        }
    }

    fn value(&mut self, depth: usize) -> Result<Val> {
        match self.peek().ok_or_else(|| bad("it ends in the middle of a value"))? {
            b'#' => {
                self.at += 1;
                let n = self.word();
                n.parse().map(Val::Ref).map_err(|_| bad(format!("\"#{n}\" isn't a reference")))
            }
            b'$' | b'*' => {
                self.at += 1;
                Ok(Val::Empty)
            }
            b'\'' => {
                self.at += 1;
                let mut s = Vec::new();
                loop {
                    let c = *self.text.get(self.at).ok_or_else(|| bad("a string never ends"))?;
                    self.at += 1;
                    if c == b'\'' {
                        if self.text.get(self.at) == Some(&b'\'') {
                            self.at += 1;
                        } else {
                            return Ok(Val::Str(String::from_utf8_lossy(&s).into_owned()));
                        }
                    }
                    s.push(c);
                }
            }
            b'.' => {
                self.at += 1;
                let w = self.word();
                self.eat(b'.')?;
                Ok(Val::Enum(w))
            }
            b'(' => Ok(Val::List(self.arguments(depth)?)),
            c if c == b'-' || c == b'+' || c.is_ascii_digit() => {
                let start = self.at;
                while self.at < self.text.len() && matches!(self.text[self.at], b'0'..=b'9' | b'+' | b'-' | b'.' | b'E' | b'e') {
                    self.at += 1;
                }
                let t = std::str::from_utf8(&self.text[start..self.at]).unwrap_or("");
                t.parse().map(Val::Num).map_err(|_| bad(format!("\"{t}\" isn't a number")))
            }
            c if c.is_ascii_alphabetic() => {
                let name = self.word();
                Ok(Val::Typed(name, self.arguments(depth)?))
            }
            c => Err(bad(format!("unexpected '{}' at byte {}", c as char, self.at))),
        }
    }
}

fn parse(text: &str) -> Result<BTreeMap<usize, Instance>> {
    let data = text.find("DATA;").ok_or_else(|| bad("it has no DATA section"))?;
    let mut scan = Scan { text: text.as_bytes(), at: data + 5 };
    let mut out = BTreeMap::new();
    loop {
        match scan.peek() {
            Some(b'#') => {
                scan.at += 1;
                let id: usize = scan.word().parse().map_err(|_| bad(format!("a bad instance number at byte {}", scan.at)))?;
                scan.eat(b'=')?;
                let mut parts: Instance = Vec::new();
                if scan.peek() == Some(b'(') {
                    scan.at += 1;
                    while scan.peek() != Some(b')') {
                        let name = scan.word();
                        if name.is_empty() {
                            return Err(bad(format!("a bad complex instance at byte {}", scan.at)));
                        }
                        parts.push((name, scan.arguments(0)?));
                    }
                    scan.at += 1;
                } else {
                    let name = scan.word();
                    parts.push((name, scan.arguments(0)?));
                }
                scan.eat(b';')?;
                out.insert(id, parts);
            }
            Some(_) => {
                // ENDSEC, or anything after it.
                return Ok(out);
            }
            None => return Ok(out),
        }
    }
}

// ----- Reading instances -----

struct File {
    items: BTreeMap<usize, Instance>,
    /// Millimetres per file length unit.
    length: f64,
    /// Radians per file angle unit.
    angle: f64,
}

impl File {
    fn part(&self, id: usize, name: &str) -> Result<&[Val]> {
        self.items
            .get(&id)
            .and_then(|i| i.iter().find(|(n, _)| n == name))
            .map(|(_, a)| a.as_slice())
            .ok_or_else(|| bad(format!("#{id} should be a {name}")))
    }

    fn name(&self, id: usize) -> &str {
        self.items.get(&id).and_then(|i| i.first()).map_or("nothing", |(n, _)| n.as_str())
    }

    fn has(&self, id: usize, name: &str) -> bool {
        self.items.get(&id).is_some_and(|i| i.iter().any(|(n, _)| n == name))
    }

    fn id(v: Option<&Val>) -> Result<usize> {
        match v {
            Some(Val::Ref(n)) => Ok(*n),
            _ => Err(bad("a reference is missing")),
        }
    }

    fn number(v: Option<&Val>) -> Result<f64> {
        match v {
            Some(Val::Num(n)) => Ok(*n),
            Some(Val::Typed(_, a)) => File::number(a.first()),
            _ => Err(bad("a number is missing")),
        }
    }

    fn truth(v: Option<&Val>) -> Result<bool> {
        match v {
            Some(Val::Enum(e)) if e == "T" => Ok(true),
            Some(Val::Enum(e)) if e == "F" => Ok(false),
            _ => Err(bad("a true-or-false is missing")),
        }
    }

    fn list(v: Option<&Val>) -> Result<&[Val]> {
        match v {
            Some(Val::List(l)) => Ok(l),
            _ => Err(bad("a list is missing")),
        }
    }

    fn numbers(v: Option<&Val>) -> Result<Vec<f64>> {
        File::list(v)?.iter().map(|n| File::number(Some(n))).collect()
    }

    fn point(&self, id: usize) -> Result<Vec3> {
        let c = File::numbers(self.part(id, "CARTESIAN_POINT")?.get(1))?;
        if c.len() != 3 {
            return Err(bad(format!("#{id} isn't a 3D point")));
        }
        Ok(v3(c[0], c[1], c[2]) * self.length)
    }

    fn direction(&self, id: usize) -> Result<Vec3> {
        let c = File::numbers(self.part(id, "DIRECTION")?.get(1))?;
        if c.len() != 3 || !(c[0].hypot(c[1]).hypot(c[2]) > 0.0) {
            return Err(bad(format!("#{id} isn't a 3D direction")));
        }
        Ok(v3(c[0], c[1], c[2]).normalized())
    }

    fn placement(&self, id: usize) -> Result<Frame> {
        let a = self.part(id, "AXIS2_PLACEMENT_3D")?;
        let origin = self.point(File::id(a.get(1))?)?;
        let z = match a.get(2) {
            Some(Val::Ref(d)) => self.direction(*d)?,
            _ => Vec3::Z,
        };
        let x = match a.get(3) {
            Some(Val::Ref(d)) => self.direction(*d)?,
            _ => z.any_perpendicular(),
        };
        Ok(Frame::new(origin, z, x))
    }

    fn spline(&self, id: usize) -> Result<Spline> {
        // Plain, or a complex instance with the same data spread over its parts.
        let (degree, poles) = if let Ok(a) = self.part(id, "B_SPLINE_CURVE") {
            (File::number(a.first())?, a.get(1))
        } else {
            let a = self.part(id, "B_SPLINE_CURVE_WITH_KNOTS")?;
            (File::number(a.get(1))?, a.get(2))
        };
        let knots_part = self.part(id, "B_SPLINE_CURVE_WITH_KNOTS")?;
        let offset = if self.has(id, "B_SPLINE_CURVE") { 0 } else { 6 };
        let mults = File::numbers(knots_part.get(offset))?;
        let values = File::numbers(knots_part.get(offset + 1))?;
        let mut knots = Vec::new();
        for (m, k) in mults.iter().zip(&values) {
            knots.extend(std::iter::repeat_n(*k, *m as usize));
        }
        let points: Vec<Vec3> = File::list(poles)?.iter().map(|p| self.point(File::id(Some(p))?)).collect::<Result<_>>()?;
        let weights = match self.part(id, "RATIONAL_B_SPLINE_CURVE") {
            Ok(a) => File::numbers(a.first())?,
            Err(_) => vec![1.0; points.len()],
        };
        let degree = degree as usize;
        if knots.len() == points.len() + degree + 1 && (knots[0] != knots[degree] || knots[knots.len() - 1] != knots[knots.len() - 1 - degree]) {
            // Not clamped, as some writers leave a curve: the part that's defined is the same curve, clamped.
            let mut line = vec![points.iter().zip(&weights).map(|(p, w)| Hom::of(*p, *w)).collect::<Vec<Hom>>()];
            let (clamped, (first, last)) = cut_lines(degree, &knots, (knots[degree], knots[knots.len() - 1 - degree]), &mut line);
            let kept = &line[0][first..=last];
            return Spline::new(degree, clamped, kept.iter().map(|q| q.point()).collect(), kept.iter().map(|q| q.w).collect());
        }
        Spline::new(degree, knots, points, weights)
    }
}

impl File {
    /// A B-spline surface, in its plain form or spread over a complex instance's parts.
    fn patch(&self, id: usize) -> Result<Patch> {
        let knots_part = self.part(id, "B_SPLINE_SURFACE_WITH_KNOTS")?;
        let complex = self.has(id, "B_SPLINE_SURFACE");
        let (du, dv, rows, offset) = if complex {
            let a = self.part(id, "B_SPLINE_SURFACE")?;
            (File::number(a.first())?, File::number(a.get(1))?, a.get(2), 0)
        } else {
            (File::number(knots_part.get(1))?, File::number(knots_part.get(2))?, knots_part.get(3), 8)
        };
        let expand = |mults: Vec<f64>, values: Vec<f64>| -> Vec<f64> {
            let mut out = Vec::new();
            for (m, k) in mults.iter().zip(&values) {
                out.extend(std::iter::repeat_n(*k, *m as usize));
            }
            out
        };
        let knots_u = expand(File::numbers(knots_part.get(offset))?, File::numbers(knots_part.get(offset + 2))?);
        let knots_v = expand(File::numbers(knots_part.get(offset + 1))?, File::numbers(knots_part.get(offset + 3))?);
        let rows = File::list(rows)?;
        let nu = rows.len();
        let mut points = Vec::new();
        let mut nv = 0;
        for row in rows {
            let line = File::list(Some(row))?;
            nv = line.len();
            for p in line {
                points.push(self.point(File::id(Some(p))?)?);
            }
        }
        let weights = match self.part(id, "RATIONAL_B_SPLINE_SURFACE") {
            Ok(a) => {
                let mut all = Vec::with_capacity(points.len());
                for row in File::list(a.first())? {
                    all.extend(File::numbers(Some(row))?);
                }
                all
            }
            Err(_) => vec![1.0; points.len()],
        };
        let (du, dv) = (du as usize, dv as usize);
        let clamped = |d: usize, k: &[f64]| k.len() > 2 * d + 1 && k[0] == k[d] && k[k.len() - 1] == k[k.len() - 1 - d];
        if knots_u.len() == nu + du + 1 && knots_v.len() == nv + dv + 1 && !(clamped(du, &knots_u) && clamped(dv, &knots_v)) {
            // As for a curve: the part that's defined, clamped, one direction at a time.
            let mut along_u: Vec<Vec<Hom>> = (0..nv).map(|j| (0..nu).map(|i| Hom::of(points[i * nv + j], weights[i * nv + j])).collect()).collect();
            let (ku, (u0, u1)) = cut_lines(du, &knots_u, (knots_u[du], knots_u[knots_u.len() - 1 - du]), &mut along_u);
            let mut along_v: Vec<Vec<Hom>> = (u0..=u1).map(|i| along_u.iter().map(|l| l[i]).collect()).collect();
            let (kv, (v0, v1)) = cut_lines(dv, &knots_v, (knots_v[dv], knots_v[knots_v.len() - 1 - dv]), &mut along_v);
            let (mut p, mut w) = (Vec::new(), Vec::new());
            for line in &along_v {
                for q in &line[v0..=v1] {
                    p.push(q.point());
                    w.push(q.w);
                }
            }
            return Patch::new((du, dv), (ku, kv), (along_v.len(), v1 - v0 + 1), p, w);
        }
        Patch::new((du, dv), (knots_u, knots_v), (nu, nv), points, weights)
    }

    /// The curve a swept surface is swept from, as a spline: a conic is exactly one.
    fn basis_curve(&self, id: usize) -> Result<Spline> {
        match self.name(id) {
            "CIRCLE" => {
                let a = self.part(id, "CIRCLE")?;
                let frame = self.placement(File::id(a.get(1))?)?;
                let r = File::number(a.get(2))? * self.length;
                Spline::ellipse_arc(frame.origin, frame.x, frame.y, r, r, 0.0, TAU)
            }
            "ELLIPSE" => {
                let a = self.part(id, "ELLIPSE")?;
                let frame = self.placement(File::id(a.get(1))?)?;
                let (major, minor) = (File::number(a.get(2))? * self.length, File::number(a.get(3))? * self.length);
                Spline::ellipse_arc(frame.origin, frame.x, frame.y, major, minor, 0.0, TAU)
            }
            _ => self.spline(id),
        }
    }
}

/// What one unit is worth: millimetres for a length, radians for an angle. A conversion-based unit says it in terms of
/// another unit, which may itself be anything: an inch can be 25.4 millimetres or 2.54 centimetres.
fn unit_value(items: &BTreeMap<usize, Instance>, id: usize, depth: u32) -> Option<f64> {
    let instance = items.get(&id)?;
    let part = |n: &str| instance.iter().find(|(m, _)| m == n).map(|(_, a)| a);
    if depth > 8 {
        return None;
    }
    if let Some(a) = part("SI_UNIT") {
        let prefix = match a.first() {
            Some(Val::Enum(p)) => p.as_str(),
            _ => "",
        };
        let factor = match prefix {
            "" => 1.0,
            "MILLI" => 1e-3,
            "CENTI" => 1e-2,
            "DECI" => 0.1,
            "KILO" => 1e3,
            "MICRO" => 1e-6,
            "NANO" => 1e-9,
            _ => return None,
        };
        return Some(if part("LENGTH_UNIT").is_some() { factor * 1000.0 } else { factor });
    }
    let a = part("CONVERSION_BASED_UNIT")?;
    let Some(Val::Ref(measure)) = a.get(1) else { return None };
    let (_, m) = items.get(measure)?.iter().find(|(n, _)| n.ends_with("_MEASURE_WITH_UNIT"))?;
    let value = File::number(m.first()).ok()?;
    let Some(Val::Ref(unit)) = m.get(1) else { return None };
    Some(value * unit_value(items, *unit, depth + 1)?)
}

/// The units of the geometry: the ones its own representation's context assigns. A file also mentions others, such as
/// the millimetre an inch is defined in, or a metre for a mass property, so the first one in the file proves nothing.
fn assigned_units(items: &BTreeMap<usize, Instance>) -> Option<(Option<f64>, Option<f64>)> {
    let context_of = |i: &Instance| -> Option<usize> {
        let (_, a) = i.iter().find(|(n, _)| n == "ADVANCED_BREP_SHAPE_REPRESENTATION")?;
        match a.get(2) {
            Some(Val::Ref(c)) => Some(*c),
            _ => None,
        }
    };
    let assigned = |i: &Instance| i.iter().find(|(n, _)| n == "GLOBAL_UNIT_ASSIGNED_CONTEXT").map(|(_, a)| a.clone());
    let list = items.values().find_map(context_of).and_then(|c| items.get(&c)).and_then(assigned).or_else(|| items.values().find_map(assigned))?;
    let Some(Val::List(units)) = list.first() else { return None };
    let (mut length, mut angle) = (None, None);
    for u in units {
        let Val::Ref(id) = u else { continue };
        let instance = items.get(id)?;
        let has = |n: &str| instance.iter().any(|(m, _)| m == n);
        if has("LENGTH_UNIT") {
            length = unit_value(items, *id, 0);
        } else if has("PLANE_ANGLE_UNIT") {
            angle = unit_value(items, *id, 0);
        }
    }
    Some((length, angle))
}

fn units(items: &BTreeMap<usize, Instance>) -> Result<(f64, f64)> {
    if let Some((Some(length), angle)) = assigned_units(items) {
        return Ok((length, angle.unwrap_or(1.0)));
    }
    let (mut length, mut angle) = (None, None);
    for i in items.values() {
        let has = |n: &str| i.iter().any(|(m, _)| m == n);
        let of = |n: &str| i.iter().find(|(m, _)| m == n).map(|(_, a)| a);
        if has("LENGTH_UNIT") && has("SI_UNIT") && length.is_none() {
            let prefix = match of("SI_UNIT").and_then(|a| a.first()) {
                Some(Val::Enum(p)) => p.as_str(),
                _ => "",
            };
            length = Some(match prefix {
                "MILLI" => 1.0,
                "CENTI" => 10.0,
                "DECI" => 100.0,
                "KILO" => 1e6,
                "MICRO" => 1e-3,
                "" => 1000.0,
                other => return Err(bad(format!("it uses a length unit with the prefix {other}"))),
            });
        }
        if has("LENGTH_UNIT") && has("CONVERSION_BASED_UNIT") && length.is_none() {
            let name = match of("CONVERSION_BASED_UNIT").and_then(|a| a.first()) {
                Some(Val::Str(s)) => s.to_ascii_uppercase(),
                _ => String::new(),
            };
            length = Some(if name.contains("INCH") {
                25.4
            } else if name.contains("FOOT") {
                304.8
            } else {
                return Err(bad(format!("it uses the length unit \"{name}\"")));
            });
        }
        if has("PLANE_ANGLE_UNIT") && has("CONVERSION_BASED_UNIT") && angle.is_none() {
            let name = match of("CONVERSION_BASED_UNIT").and_then(|a| a.first()) {
                Some(Val::Str(s)) => s.to_ascii_uppercase(),
                _ => String::new(),
            };
            angle = Some(if name.contains("DEG") { PI / 180.0 } else { 1.0 });
        }
    }
    Ok((length.unwrap_or(1.0), angle.unwrap_or(1.0)))
}

// ----- From a file's topology to this kernel's -----

/// A face while it's being converted.
#[derive(Clone)]
struct RawFace {
    surface: Surface,
    flipped: bool,
    loops: Vec<Vec<Coedge>>,
    /// Vertices the file bounds the face by alone: a sphere's pole, or a cone's apex.
    points: Vec<usize>,
    tag: FaceTag,
}

/// A family of cuts around a surface: a value that wraps around it, and where to cut.
#[derive(Clone, Copy, PartialEq)]
enum Family {
    /// Around the axis.
    AroundAxis,
    /// Around a torus's tube.
    AroundTube,
}

/// Whether a point has no phase, being on the axis: an apex or a pole.
fn on_axis(surface: &Surface, family: Family, p: Vec3) -> bool {
    if let Surface::Free { frame, patch } = surface {
        return patch.at_pole(frame.to_local(p));
    }
    let l = surface.frame().to_local(p);
    family == Family::AroundAxis && l.x.hypot(l.y) <= 1e-7
}

/// A closed free-form surface's direction: its parameter, as an angle, which is what its cuts work in.
fn free_phase(patch: &Patch, family: Family, l: Vec3) -> f64 {
    let foot = patch.foot(l);
    let ((u0, u1), (v0, v1)) = patch.domain();
    let (x, a, b) = if family == Family::AroundAxis { (foot.u, u0, u1) } else { (foot.v, v0, v1) };
    (x - a) / (b - a) * TAU - PI
}

/// The parameter at a free-form surface's phase.
fn free_parameter(patch: &Patch, family: Family, phase: f64) -> f64 {
    let ((u0, u1), (v0, v1)) = patch.domain();
    let (a, b) = if family == Family::AroundAxis { (u0, u1) } else { (v0, v1) };
    a + (b - a) * ((phase + PI) / TAU).clamp(0.0, 1.0)
}

fn phase(surface: &Surface, family: Family, p: Vec3) -> f64 {
    let f = surface.frame();
    let l = f.to_local(p);
    if let Surface::Free { patch, .. } = surface {
        return free_phase(patch, family, l);
    }
    match (family, surface) {
        (Family::AroundTube, Surface::Torus { major, .. }) => l.z.atan2(l.x.hypot(l.y) - major),
        _ => l.y.atan2(l.x),
    }
}

/// The surface's curve at a fixed value of the phase, with parameters running along the other direction.
fn wall_curve(surface: &Surface, family: Family, a: f64) -> Option<(Curve, f64, f64)> {
    let f = *surface.frame();
    if let Surface::Free { patch, .. } = surface {
        let at = free_parameter(patch, family, wrap_pi(a));
        let curve = patch.iso_curve(at, family == Family::AroundAxis).ok()?.mapped(|q| f.to_world(q));
        let (t0, t1) = curve.domain();
        return Some((Curve::Spline(Arc::new(curve)), t0, t1));
    }
    let radial = f.x * a.cos() + f.y * a.sin();
    match (family, surface) {
        (Family::AroundAxis, Surface::Cylinder { radius, .. }) => Some((Curve::Line { origin: f.origin + radial * *radius, dir: f.z }, -1e9, 1e9)),
        (Family::AroundAxis, Surface::Cone { half_angle, .. }) => {
            let apex = surface.apex()?;
            let dir = radial * half_angle.sin() + f.z * half_angle.cos();
            Some((Curve::Line { origin: apex, dir }, 0.0, 1e9))
        }
        (Family::AroundAxis, Surface::Sphere { radius, .. }) => {
            let frame = Frame { origin: f.origin, x: radial, y: f.z, z: radial.cross(f.z) };
            Some((Curve::Circle { frame, radius: *radius }, -PI / 2.0, PI / 2.0))
        }
        (Family::AroundAxis, Surface::Torus { major, minor, .. }) => {
            let frame = Frame { origin: f.origin + radial * *major, x: radial, y: f.z, z: radial.cross(f.z) };
            Some((Curve::Circle { frame, radius: *minor }, -PI, PI))
        }
        (Family::AroundAxis, Surface::Revolved { curve, .. }) => {
            let (t0, t1) = curve.spline.domain();
            Some((Curve::Spline(Arc::new(curve.spline.mapped(|q| f.origin + radial * q.x + f.z * q.y))), t0, t1))
        }
        (Family::AroundTube, Surface::Torus { major, minor, .. }) => {
            let frame = Frame { origin: f.origin + f.z * (*minor * a.sin()), ..f };
            Some((Curve::Circle { frame, radius: *major + *minor * a.cos() }, -PI, PI))
        }
        _ => None,
    }
}

/// Where a curve piece crosses a phase, found by stepping along it, with a tolerance for its ends.
fn crossings(curve: &Curve, t0: f64, t1: f64, surface: &Surface, family: Family, a: f64) -> Vec<f64> {
    let f = |t: f64| wrap_pi(phase(surface, family, curve.eval(t)) - a);
    let n = 128;
    let mut out = Vec::new();
    let mut before = f(t0);
    for i in 1..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let now = f(t);
        // A sign change near zero, not the jump where the phase wraps a half turn away.
        if before.signum() != now.signum() && before.abs() < 1.0 && now.abs() < 1.0 {
            let (mut lo, mut hi, mut flo) = (t - (t1 - t0) / n as f64, t, before);
            for _ in 0..60 {
                let mid = (lo + hi) / 2.0;
                let v = f(mid);
                if v.signum() == flo.signum() {
                    lo = mid;
                    flo = v;
                } else {
                    hi = mid;
                }
            }
            out.push((lo + hi) / 2.0);
        }
        before = now;
    }
    out
}

/// Builds solids from one shell's worth of faces at a time, sharing vertices and edges across a file's solid.
struct Builder {
    vertices: Vec<Vec3>,
    edges: Vec<Edge>,
    /// The widest gap between an edge and its faces, once everything has been brought as close as it will go.
    gap: f64,
}

impl Builder {
    fn add_vertex(&mut self, p: Vec3) -> usize {
        self.vertices.push(p);
        self.vertices.len() - 1
    }

    fn add_edge(&mut self, edge: Edge) -> usize {
        self.edges.push(edge);
        self.edges.len() - 1
    }

    /// Replaces edges by the pieces between the given parameters, in every loop of every face.
    fn cut(&mut self, faces: &mut [RawFace], cuts: &BTreeMap<usize, Vec<(f64, Option<usize>)>>) {
        let mut pieces: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (&e, list) in cuts {
            let edge = self.edges[e].clone();
            let speed = edge.curve.speed_bound().max(1e-9);
            let mut stops: Vec<(f64, Option<usize>)> = list.iter().copied().filter(|(t, _)| (t - edge.t0) * speed > 4.0 * TOL && (edge.t1 - t) * speed > 4.0 * TOL).collect();
            stops.sort_by(|a, b| a.0.total_cmp(&b.0));
            stops.dedup_by(|a, b| (a.0 - b.0) * speed <= 4.0 * TOL);
            if stops.is_empty() {
                continue;
            }
            let mut ids = Vec::with_capacity(stops.len() + 1);
            let (mut from_t, mut from_v) = (edge.t0, edge.v0);
            for (t, known) in stops {
                let v = known.unwrap_or_else(|| {
                    let p = edge.curve.eval(t);
                    self.add_vertex(p)
                });
                ids.push(self.add_edge(Edge { curve: edge.curve.clone(), t0: from_t, t1: t, v0: from_v, v1: v }));
                (from_t, from_v) = (t, v);
            }
            ids.push(self.add_edge(Edge { curve: edge.curve.clone(), t0: from_t, t1: edge.t1, v0: from_v, v1: edge.v1 }));
            pieces.insert(e, ids);
        }
        for face in faces {
            for lp in &mut face.loops {
                let mut out = Vec::with_capacity(lp.len());
                for c in lp.iter() {
                    match pieces.get(&c.edge) {
                        None => out.push(*c),
                        Some(ids) if !c.reversed => out.extend(ids.iter().map(|&edge| Coedge { edge, reversed: false })),
                        Some(ids) => out.extend(ids.iter().rev().map(|&edge| Coedge { edge, reversed: true })),
                    }
                }
                *lp = out;
            }
        }
    }

    fn point_on(&self, c: Coedge) -> Vec3 {
        let e = &self.edges[c.edge];
        e.curve.eval((e.t0 + e.t1) / 2.0)
    }

    /// Points around a face's boundary.
    fn boundary_points(&self, face: &RawFace) -> Vec<Vec3> {
        let mut out = Vec::new();
        for lp in &face.loops {
            for c in lp {
                let e = &self.edges[c.edge];
                for i in 0..=8 {
                    out.push(e.curve.eval(e.t0 + (e.t1 - e.t0) * i as f64 / 8.0));
                }
            }
        }
        out.extend(face.points.iter().map(|&v| self.vertices[v]));
        out
    }
}

/// Edges a face uses in both directions: its seams. They divide nothing, and walls replace them.
fn seams_of(face: &RawFace) -> Vec<usize> {
    let mut seen: BTreeMap<usize, usize> = BTreeMap::new();
    for c in face.loops.iter().flatten() {
        *seen.entry(c.edge).or_insert(0) += 1;
    }
    seen.into_iter().filter(|(_, n)| *n >= 2).map(|(e, _)| e).collect()
}

/// Whether a face goes all the way round in a family: it has to be cut that way first, since until then a chart
/// can't tell one side of the cut from the other.
fn goes_round(b: &Builder, face: &RawFace, family: Family) -> bool {
    let seam = seams_of(face).into_iter().any(|e| {
        let edge = &b.edges[e];
        let at = |f: f64| phase(&face.surface, family, edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * f));
        wrap_pi(at(0.2) - at(0.5)).abs() < 1e-6 && wrap_pi(at(0.8) - at(0.5)).abs() < 1e-6
    });
    seam || face.loops.iter().any(|lp| {
        let mut total = 0.0;
        let mut last = None;
        for c in lp {
            let e = &b.edges[c.edge];
            for i in 0..=16 {
                let f = if c.reversed { 1.0 - i as f64 / 16.0 } else { i as f64 / 16.0 };
                let q = e.curve.eval(e.t0 + (e.t1 - e.t0) * f);
                // Through a pole the phase jumps by a half turn, which isn't going round.
                if on_axis(&face.surface, family, q) {
                    last = None;
                    continue;
                }
                let a = phase(&face.surface, family, q);
                if let Some(l) = last {
                    total += wrap_pi(a - l);
                }
                last = Some(a);
            }
        }
        total.abs() > 1.5 * PI
    })
}

/// Whether a family needs cutting on this face, and at which two phases.
fn walls_for(b: &Builder, face: &RawFace, family: Family) -> Option<[f64; 2]> {
    if let Surface::Free { patch, .. } = &face.surface {
        // Only one that closes up can go round.
        if !patch.closes_up(family == Family::AroundTube) {
            return None;
        }
    } else if family == Family::AroundTube && !matches!(face.surface, Surface::Torus { .. }) {
        return None;
    }
    if matches!(face.surface, Surface::Plane(_) | Surface::Extruded { .. }) {
        return None;
    }
    let points = b.boundary_points(face);
    // A seam of this family lies at one phase all along.
    let seam = seams_of(face).into_iter().find_map(|e| {
        let edge = &b.edges[e];
        let at = |f: f64| phase(&face.surface, family, edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * f));
        let a = at(0.5);
        (wrap_pi(at(0.2) - a).abs() < 1e-6 && wrap_pi(at(0.8) - a).abs() < 1e-6).then_some(a)
    });
    if let Some(a) = seam {
        return Some([a, a + PI]);
    }
    let phases: Vec<f64> = points.iter().filter(|&&p| !on_axis(&face.surface, family, p)).map(|&p| phase(&face.surface, family, p)).collect();
    // A loop with no seam can still go all the way round: a band between two circles has two such.
    let goes_round = goes_round(b, face, family);
    let wide = phase_spread(&phases) > PI + 1e-6;
    let fits = if let Surface::Sphere { frame, .. } = &face.surface {
        // A hemisphere's boundary can be a whole great circle, whose points alone don't say which side it's on.
        // The direction of the loops does, as when fitting a chart to one.
        let mut area = Vec3::ZERO;
        for lp in &face.loops {
            let mut chain = Vec::new();
            for c in lp {
                let e = &b.edges[c.edge];
                for i in 0..16 {
                    let f = if c.reversed { 1.0 - i as f64 / 16.0 } else { i as f64 / 16.0 };
                    chain.push(e.curve.eval(e.t0 + (e.t1 - e.t0) * f) - frame.origin);
                }
            }
            for i in 0..chain.len() {
                area += chain[i].cross(chain[(i + 1) % chain.len()]) * 0.5;
            }
        }
        let center = if face.flipped { -area } else { area };
        center.norm() > 1e-9 && points.iter().all(|&p| (p - frame.origin).dot(center) >= -1e-6 * center.norm() * frame.origin.dist(p).max(1.0))
    } else {
        within_half_turn(&face.surface, &points)
    };
    if !goes_round && !wide && fits {
        return None;
    }
    // Through the middle of what it covers, on an irregular angle so as not to land on a vertex.
    let center = angular_center(&phases) + 0.137;
    Some([center, center + PI])
}

/// The surface for the half turn of phase from `start`.
fn sector_surface(surface: &Surface, family: Family, start: f64) -> Result<Surface> {
    let Surface::Free { frame, patch } = surface else {
        return Ok(surface.clone());
    };
    let mut lo = wrap_pi(start);
    if lo > PI - 1e-6 {
        lo -= TAU;
    }
    let hi = lo + PI;
    if hi > PI + 1e-6 {
        return Err(Error::Unsupported("a free-form face that runs across its surface's seam".into()));
    }
    let (u, v) = patch.domain();
    let part = (free_parameter(patch, family, lo), free_parameter(patch, family, hi));
    let own = if family == Family::AroundAxis { patch.restricted(part, v)? } else { patch.restricted(u, part)? };
    Ok(Surface::Free { frame: *frame, patch: Arc::new(own) })
}

/// A face cut from a surface that closes up gets a patch of only its part. On the whole surface, a point on the
/// seam is at both ends of the parameters, and a chart can't say which end this face means.
fn keep_own_part_of_closed_patch(b: &Builder, face: &mut RawFace) -> Result<()> {
    let Surface::Free { frame, patch } = &face.surface else {
        return Ok(());
    };
    let points = b.boundary_points(face);
    let range = |family: Family| -> Result<(f64, f64)> {
        let ((u0, u1), (v0, v1)) = patch.domain();
        if !patch.closes_up(family == Family::AroundTube) {
            return Ok(if family == Family::AroundAxis { (u0, u1) } else { (v0, v1) });
        }
        let phases: Vec<f64> = points.iter().map(|p| phase(&face.surface, family, *p)).collect();
        let center = angular_center(&phases);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for a in &phases {
            let at = center + wrap_pi(a - center);
            lo = lo.min(at);
            hi = hi.max(at);
        }
        if lo < -PI - 1e-6 || hi > PI + 1e-6 {
            return Err(Error::Unsupported("a free-form face that runs across its surface's seam".into()));
        }
        if hi - lo < 1e-9 {
            return Err(Error::Unsupported("a free-form surface closed all the way round with no edges of its own to cut it by".into()));
        }
        Ok((free_parameter(patch, family, lo), free_parameter(patch, family, hi)))
    };
    let (u, v) = (range(Family::AroundAxis)?, range(Family::AroundTube)?);
    let own = patch.restricted(u, v)?;
    face.surface = Surface::Free { frame: *frame, patch: Arc::new(own) };
    Ok(())
}

/// How wide the largest arc a set of angles covers is.
fn phase_spread(phases: &[f64]) -> f64 {
    if phases.is_empty() {
        return 0.0;
    }
    let center = angular_center(phases);
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for &p in phases {
        let d = wrap_pi(p - center);
        lo = lo.min(d);
        hi = hi.max(d);
    }
    hi - lo
}

/// Cuts one face along two walls, half a turn apart, into the sectors between them.
fn split_face(b: &mut Builder, face: &RawFace, family: Family, walls: [f64; 2]) -> Result<Vec<RawFace>> {
    let surface = &face.surface;
    let seam_edges: Vec<usize> = seams_of(face)
        .into_iter()
        .filter(|&e| {
            let edge = &b.edges[e];
            let a = phase(surface, family, edge.curve.eval((edge.t0 + edge.t1) / 2.0));
            walls.iter().any(|w| wrap_pi(a - w).abs() < 1e-6)
        })
        .collect();
    // What goes along each wall: first, where the boundary crosses it, with whether going up the wall from
    // there is going into the face.
    let mut new_faces: Vec<RawFace> = Vec::new();
    let mut half: [Vec<Coedge>; 2] = [Vec::new(), Vec::new()];
    // Poles and apexes are vertices that both walls end on. Any the face already has count.
    let mut poles: Vec<usize> = face.points.clone();
    for &e in &seam_edges {
        poles.extend([b.edges[e].v0, b.edges[e].v1]);
    }
    // So does any corner of the face that's on the axis, as a wedge of a sphere has at its point.
    for c in face.loops.iter().flatten() {
        for v in [b.edges[c.edge].v0, b.edges[c.edge].v1] {
            if on_axis(surface, family, b.vertices[v]) && !poles.contains(&v) {
                poles.push(v);
            }
        }
    }
    for c in face.loops.iter().flatten() {
        if seam_edges.contains(&c.edge) {
            continue;
        }
        let a = phase(surface, family, b.point_on(*c));
        let sector = usize::from((a - walls[0]).rem_euclid(TAU) >= PI);
        half[sector].push(*c);
    }
    for (w, &wall) in walls.iter().enumerate() {
        let (curve, lo, hi) = wall_curve(surface, family, wall)
            .ok_or_else(|| Error::Unsupported(format!("cutting a {} into pieces of half a turn isn't available", surface.name())))?;
        // Stops along the wall: a parameter, a vertex, and whether going up the wall from there enters the face
        // (or `None` if the face is on both sides of it, as at a seam of the other family).
        let mut stops: Vec<(f64, usize, Option<bool>)> = Vec::new();
        let sign = if face.flipped { -1.0 } else { 1.0 };
        for c in face.loops.iter().flatten() {
            if seam_edges.contains(&c.edge) {
                continue;
            }
            for end in [false, true] {
                let v = if end == c.reversed { b.edges[c.edge].v0 } else { b.edges[c.edge].v1 };
                let p = b.vertices[v];
                if wrap_pi(phase(surface, family, p) - wall).abs() > 1e-6 {
                    continue;
                }
                // Its phase already says it's on this wall. All this can see is how far a file's edge is off its surface,
                // which gets settled later, and has a limit of its own there.
                let (t, off) = curve.closest(p, lo, hi);
                if off > 0.2 {
                    continue;
                }
                // The direction of travel through this point, along the loop.
                let e = &b.edges[c.edge];
                let at = if end == c.reversed { e.t0 } else { e.t1 };
                let travel = e.curve.derivative(at) * if c.reversed { -1.0 } else { 1.0 };
                let up = curve.derivative(t);
                // The face is on the left of travel, seen from outside. Going up the wall is on that side?
                let n = surface.normal(p) * sign;
                let enters = n.cross(travel).dot(up) > 0.0;
                match stops.iter_mut().find(|s| s.1 == v) {
                    Some(stop) => {
                        if stop.2 != Some(enters) {
                            stop.2 = None;
                        }
                    }
                    None => stops.push((t, v, Some(enters))),
                }
            }
        }
        stops.sort_by(|x, y| x.0.total_cmp(&y.0));
        let closes_on_itself = !matches!(surface, Surface::Free { .. }) && ((matches!(surface, Surface::Torus { .. }) && family == Family::AroundAxis) || family == Family::AroundTube);
        let mut intervals: Vec<(f64, usize, f64, usize)> = Vec::new();
        let mut end_at = |b: &mut Builder, t: f64| -> usize {
            let p = curve.eval(t);
            if let Some(&v) = poles.iter().find(|&&v| b.vertices[v].dist(p) <= 8.0 * TOL) {
                return v;
            }
            let v = b.add_vertex(p);
            poles.push(v);
            v
        };
        if stops.is_empty() {
            // Nothing crosses it, so it's a pole-to-pole wall on a face wrapped all the way round, or it misses.
            if !closes_on_itself && face.loops.iter().flatten().all(|c| seam_edges.contains(&c.edge)) {
                let (first, second) = (end_at(b, lo), end_at(b, hi));
                intervals.push((lo, first, hi, second));
            }
        } else if closes_on_itself {
            // Round a closed wall, what's inside after the last stop is what's inside before the first.
            let n = stops.len();
            let mut inside = stops.iter().rev().find_map(|s| s.2).unwrap_or(true);
            for i in 0..n {
                let (t, v, change) = stops[i];
                if let Some(enters) = change {
                    inside = enters;
                }
                if inside {
                    let (mut u, w, _) = stops[(i + 1) % n];
                    while u <= t {
                        u += TAU;
                    }
                    intervals.push((t, v, u, w));
                }
            }
        } else {
            // Up an open wall, the first stop says whether it started inside, which means from a pole or apex.
            let mut inside = stops.first().and_then(|s| s.2) == Some(false);
            let mut from = if inside {
                let v = end_at(b, lo);
                (lo, v)
            } else {
                (stops[0].0, stops[0].1)
            };
            for &(t, v, change) in &stops {
                match (inside, change) {
                    (false, Some(true)) => {
                        inside = true;
                        from = (t, v);
                    }
                    (true, Some(false)) => {
                        intervals.push((from.0, from.1, t, v));
                        inside = false;
                    }
                    (true, None) => {
                        intervals.push((from.0, from.1, t, v));
                        from = (t, v);
                    }
                    (false, None) => from = (t, v),
                    _ => {
                        return Err(Error::NearDegenerate(format!(
                            "this {}'s boundary doesn't alternate in and out along a line cut through it near ({:.4}, {:.4}, {:.4})",
                            surface.name(),
                            b.vertices[v].x,
                            b.vertices[v].y,
                            b.vertices[v].z
                        )))
                    }
                }
            }
            if inside {
                let v = end_at(b, hi);
                intervals.push((from.0, from.1, hi, v));
            }
        }
        for (t0, v0, t1, v1) in intervals {
            let id = b.add_edge(Edge { curve: curve.clone(), t0, t1, v0, v1 });
            // Each sector gets it once, in the direction that keeps the sector on the left.
            let middle = curve.eval((t0 + t1) / 2.0);
            let along = curve.derivative((t0 + t1) / 2.0);
            let n = surface.normal(middle) * sign;
            // The way the phase increases at this point.
            let h = 1e-4;
            let toward = {
                let forward = surface.project(move_phase(surface, family, middle, h));
                (forward - middle).normalized()
            };
            // For the sector on the increasing side of this wall, the way in is `toward`.
            let leaves_on_left = n.cross(along).dot(toward) > 0.0;
            let (inc, dec) = if w == 0 { (0, 1) } else { (1, 0) };
            half[inc].push(Coedge { edge: id, reversed: !leaves_on_left });
            half[dec].push(Coedge { edge: id, reversed: leaves_on_left });
        }
    }
    for (sector, list) in half.iter().enumerate() {
        if list.is_empty() {
            continue;
        }
        let mid = walls[0] + PI * (sector as f64 + 0.5);
        let points: Vec<Vec3> = list
            .iter()
            .flat_map(|c| {
                let e = &b.edges[c.edge];
                (0..=8).map(move |i| (e, i)).map(|(e, i)| e.curve.eval(e.t0 + (e.t1 - e.t0) * i as f64 / 8.0))
            })
            .collect();
        // On a closed free-form surface, each sector gets a patch of its own before anything is traced, since
        // on the whole surface a point on the seam is at both ends of the parameters.
        let own = sector_surface(surface, family, walls[0] + PI * sector as f64)?;
        let surface = &own;
        let mut chart = surface.fit_chart(&points);
        if family == Family::AroundAxis {
            chart.u0 = if matches!(surface, Surface::Plane(_)) { chart.u0 } else { wrap_pi(mid) };
        }
        let fc = FaceChart { surface, chart: &chart, flipped: face.flipped };
        for loops in trace_faces(&fc, &b.edges, &b.vertices, list)? {
            new_faces.push(RawFace { surface: surface.clone(), flipped: face.flipped, loops, points: Vec::new(), tag: face.tag });
        }
    }
    if new_faces.is_empty() {
        return Err(Error::NearDegenerate(format!("a {} couldn't be cut into pieces of half a turn", surface.name())));
    }
    Ok(new_faces)
}

/// A swept face whose curve is too long for one face: cut along rulings where the curve has to be cut, and put each
/// piece on the part of the curve it covers. This runs before anything else looks at the surface's sign.
fn split_extruded(b: &mut Builder, faces: Vec<RawFace>) -> Result<Vec<RawFace>> {
    // First the edges, across all faces, since their neighbours share them.
    let mut cuts: BTreeMap<usize, Vec<(f64, Option<usize>)>> = BTreeMap::new();
    let mut plans: Vec<Option<Vec<(f64, f64, bool)>>> = Vec::with_capacity(faces.len());
    for face in &faces {
        let Surface::Extruded { frame, curve } = &face.surface else {
            plans.push(None);
            continue;
        };
        let ranges = curve.spline.face_ranges()?;
        if ranges.len() < 2 && !matches!(ranges.first(), Some((_, _, true))) {
            plans.push(None);
            continue;
        }
        let seams = seams_of(face);
        let foot = |p: Vec3| {
            let l = frame.to_local(p);
            curve.spline.closest(v3(l.x, l.y, 0.0), curve.spline.domain().0, curve.spline.domain().1).0
        };
        for c in face.loops.iter().flatten() {
            if seams.contains(&c.edge) {
                continue;
            }
            let e = &b.edges[c.edge];
            for &(start, _, _) in ranges.iter().skip(1) {
                let n = 256;
                let mut before = foot(e.curve.eval(e.t0)) - start;
                for i in 1..=n {
                    let t = e.t0 + (e.t1 - e.t0) * i as f64 / n as f64;
                    let now = foot(e.curve.eval(t)) - start;
                    // Not the jump across a closed curve's ends.
                    let span = curve.spline.domain().1 - curve.spline.domain().0;
                    if before.signum() != now.signum() && (before - now).abs() < span * 0.5 {
                        let (mut lo, mut hi, mut flo) = (t - (e.t1 - e.t0) / n as f64, t, before);
                        for _ in 0..60 {
                            let mid = (lo + hi) / 2.0;
                            let v = foot(e.curve.eval(mid)) - start;
                            if v.signum() == flo.signum() {
                                lo = mid;
                                flo = v;
                            } else {
                                hi = mid;
                            }
                        }
                        cuts.entry(c.edge).or_default().push(((lo + hi) / 2.0, None));
                    }
                    before = now;
                }
            }
        }
        plans.push(Some(ranges));
    }
    let mut faces = faces;
    b.cut(&mut faces, &cuts);
    let mut out = Vec::with_capacity(faces.len());
    for (face, plan) in faces.into_iter().zip(plans) {
        let Some(ranges) = plan else {
            out.push(face);
            continue;
        };
        let Surface::Extruded { frame, curve } = face.surface.clone() else { unreachable!() };
        let (u0, u1) = curve.spline.domain();
        let foot = |p: Vec3| {
            let l = frame.to_local(p);
            curve.spline.closest(v3(l.x, l.y, 0.0), u0, u1).0
        };
        let closed = curve.eval(u0).dist(curve.eval(u1)) <= 2.0 * TOL;
        let seams = seams_of(&face);
        let sign = if face.flipped { -1.0 } else { 1.0 };
        // What's on the boundary of each range.
        let range_of = |t: f64| ranges.iter().position(|r| t >= r.0 && t <= r.1).unwrap_or(ranges.len() - 1);
        let mut half: Vec<Vec<Coedge>> = vec![Vec::new(); ranges.len()];
        for c in face.loops.iter().flatten() {
            if seams.contains(&c.edge) {
                continue;
            }
            half[range_of(foot(b.point_on(*c)))].push(*c);
        }
        // Rulings between neighbouring ranges, including across the join of a closed curve.
        let mut joins: Vec<(f64, usize, usize)> = (1..ranges.len()).map(|i| (ranges[i].0, i - 1, i)).collect();
        if closed && !seams.is_empty() {
            joins.push((u0, ranges.len() - 1, 0));
        }
        for (t, below, above) in joins {
            let at = curve.eval(t);
            let line = Curve::Line { origin: frame.point(at.x, at.y, 0.0), dir: frame.z };
            let tangent = curve.derivative(t);
            let along = frame.x * tangent.x + frame.y * tangent.y;
            let mut stops: Vec<(f64, usize, bool)> = Vec::new();
            for c in face.loops.iter().flatten() {
                if seams.contains(&c.edge) {
                    continue;
                }
                let e = &b.edges[c.edge];
                for end in [false, true] {
                    let v = if end == c.reversed { e.v0 } else { e.v1 };
                    let p = b.vertices[v];
                    let (z, off) = line.closest(p, -1e9, 1e9);
                    if off > 8.0 * TOL || stops.iter().any(|s| s.1 == v) {
                        continue;
                    }
                    let param = if end == c.reversed { e.t0 } else { e.t1 };
                    let travel = e.curve.derivative(param) * if c.reversed { -1.0 } else { 1.0 };
                    let n = face.surface.normal(p) * sign;
                    stops.push((z, v, n.cross(travel).dot(frame.z) > 0.0));
                }
            }
            stops.sort_by(|x, y| x.0.total_cmp(&y.0));
            for w in stops.windows(2) {
                if w[0].2 {
                    let id = b.add_edge(Edge { curve: line.clone(), t0: w[0].0, t1: w[1].0, v0: w[0].1, v1: w[1].1 });
                    // Each side keeps its own piece on its left, going by which way the curve runs.
                    let n = face.surface.normal(line.eval((w[0].0 + w[1].0) / 2.0)) * sign;
                    let left_is_ahead = n.cross(frame.z).dot(along) > 0.0;
                    half[above].push(Coedge { edge: id, reversed: !left_is_ahead });
                    half[below].push(Coedge { edge: id, reversed: left_is_ahead });
                }
            }
        }
        for (r, list) in half.iter().enumerate() {
            if list.is_empty() {
                continue;
            }
            let (start, stop, straight) = ranges[r];
            if straight {
                return Err(Error::Unsupported("SURFACE_OF_LINEAR_EXTRUSION of a curve with a straight stretch".into()));
            }
            let surface = Surface::Extruded { frame, curve: Arc::new(PlaneCurve::new(curve.spline.sub(start, stop)?)) };
            let chart = surface.fit_chart(&[]);
            let fc = FaceChart { surface: &surface, chart: &chart, flipped: face.flipped };
            for loops in trace_faces(&fc, &b.edges, &b.vertices, list)? {
                out.push(RawFace { surface: surface.clone(), flipped: face.flipped, loops, points: Vec::new(), tag: face.tag });
            }
        }
    }
    Ok(out)
}

/// A point moved a little further round in phase, off the surface if need be.
fn move_phase(surface: &Surface, family: Family, p: Vec3, h: f64) -> Vec3 {
    let f = surface.frame();
    let l = f.to_local(p);
    if let Surface::Free { patch, .. } = surface {
        let foot = patch.foot(l);
        let ((u0, u1), (v0, v1)) = patch.domain();
        // Past the end of a surface that closes up is back at its start.
        let (u, v) = if family == Family::AroundAxis {
            let u = foot.u + h / TAU * (u1 - u0);
            (if u > u1 { u - (u1 - u0) } else { u }, foot.v)
        } else {
            let v = foot.v + h / TAU * (v1 - v0);
            (foot.u, if v > v1 { v - (v1 - v0) } else { v })
        };
        return f.to_world(patch.eval(u, v));
    }
    match (family, surface) {
        (Family::AroundTube, Surface::Torus { major, .. }) => {
            let rho = l.x.hypot(l.y);
            let radial = if rho > 1e-300 { v3(l.x / rho, l.y / rho, 0.0) } else { Vec3::X };
            let (a, c) = (rho - major, l.z);
            let (ca, sa) = (h.cos(), h.sin());
            let (a2, c2) = (a * ca - c * sa, a * sa + c * ca);
            f.to_world(radial * (major + a2) + Vec3::Z * c2)
        }
        _ => {
            let (s, c) = h.sin_cos();
            f.to_world(v3(l.x * c - l.y * s, l.x * s + l.y * c, l.z))
        }
    }
}

/// One solid from a file's shells.
struct Reader<'a> {
    file: &'a File,
    next_slot: u32,
    feature: u32,
}

impl Reader<'_> {
    fn surface(&self, id: usize) -> Result<Surface> {
        let f = self.file;
        if f.has(id, "B_SPLINE_SURFACE_WITH_KNOTS") {
            let patch = f.patch(id)?;
            return Ok(Surface::Free { frame: Frame::WORLD, patch: Arc::new(patch) });
        }
        Ok(match f.name(id) {
            "PLANE" => Surface::Plane(f.placement(File::id(f.part(id, "PLANE")?.get(1))?)?),
            "CYLINDRICAL_SURFACE" => {
                let a = f.part(id, "CYLINDRICAL_SURFACE")?;
                Surface::Cylinder { frame: f.placement(File::id(a.get(1))?)?, radius: File::number(a.get(2))? * f.length }
            }
            "CONICAL_SURFACE" => {
                let a = f.part(id, "CONICAL_SURFACE")?;
                Surface::Cone { frame: f.placement(File::id(a.get(1))?)?, radius: File::number(a.get(2))? * f.length, half_angle: File::number(a.get(3))? * f.angle }
            }
            "SPHERICAL_SURFACE" => {
                let a = f.part(id, "SPHERICAL_SURFACE")?;
                Surface::Sphere { frame: f.placement(File::id(a.get(1))?)?, radius: File::number(a.get(2))? * f.length }
            }
            "TOROIDAL_SURFACE" => {
                let a = f.part(id, "TOROIDAL_SURFACE")?;
                Surface::Torus { frame: f.placement(File::id(a.get(1))?)?, major: File::number(a.get(2))? * f.length, minor: File::number(a.get(3))? * f.length }
            }
            "DEGENERATE_TOROIDAL_SURFACE" => {
                let a = f.part(id, "DEGENERATE_TOROIDAL_SURFACE")?;
                let major = File::number(a.get(2))? * f.length;
                let outer = File::truth(a.get(4))?;
                Surface::Torus { frame: f.placement(File::id(a.get(1))?)?, major: if outer { major } else { -major }, minor: File::number(a.get(3))? * f.length }
            }
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let a = f.part(id, "SURFACE_OF_LINEAR_EXTRUSION")?;
                let spline = f.basis_curve(File::id(a.get(1))?)?;
                let v = f.part(File::id(a.get(2))?, "VECTOR")?;
                let dir = f.direction(File::id(v.get(1))?)?;
                swept_along(&spline, dir)?
            }
            "SURFACE_OF_REVOLUTION" => {
                let a = f.part(id, "SURFACE_OF_REVOLUTION")?;
                let spline = f.basis_curve(File::id(a.get(1))?)?;
                let axis = f.part(File::id(a.get(2))?, "AXIS1_PLACEMENT")?;
                let (o, z) = (f.point(File::id(axis.get(1))?)?, f.direction(File::id(axis.get(2))?)?);
                swept_around(&spline, o, z)?
            }
            other => return Err(Error::Unsupported(format!("a face uses {other}"))),
        })
    }

    /// The faces of a shell, reading edges and vertices into `seen` so that faces share them.
    #[allow(clippy::too_many_arguments)]
    fn shell(&mut self, id: usize, invert: bool, b: &mut Builder, vertices: &mut BTreeMap<usize, usize>, edges: &mut BTreeMap<usize, (usize, bool)>, out: &mut Vec<RawFace>) -> Result<()> {
        let f = self.file;
        let list = if f.has(id, "CLOSED_SHELL") {
            File::list(f.part(id, "CLOSED_SHELL")?.get(1))?
        } else if f.has(id, "OPEN_SHELL") {
            return Err(Error::Unsupported("an open shell: this file holds surfaces with gaps, and only closed solids can be imported".into()));
        } else {
            return Err(bad(format!("#{id} should be a closed shell, not a {}", f.name(id))));
        };
        for item in list {
            let face_id = File::id(Some(item))?;
            let parts = f.part(face_id, "ADVANCED_FACE").or_else(|_| f.part(face_id, "FACE_SURFACE"))?;
            let surface = self.surface(File::id(parts.get(2))?)?;
            let same = File::truth(parts.get(3))?;
            let tag = FaceTag { feature: self.feature, slot: 2 + self.next_slot };
            self.next_slot += 1;
            let mut loops = Vec::new();
            let mut points = Vec::new();
            for bound in File::list(parts.get(1))? {
                let bound_id = File::id(Some(bound))?;
                let a = f.part(bound_id, "FACE_OUTER_BOUND").or_else(|_| f.part(bound_id, "FACE_BOUND"))?;
                let loop_id = File::id(a.get(1))?;
                let forward = File::truth(a.get(2))?;
                if f.has(loop_id, "VERTEX_LOOP") {
                    let v = File::id(f.part(loop_id, "VERTEX_LOOP")?.get(1))?;
                    points.push(self.vertex(v, b, vertices)?);
                    continue;
                }
                let list = File::list(f.part(loop_id, "EDGE_LOOP")?.get(1))?;
                let mut lp = Vec::with_capacity(list.len());
                for oriented in list {
                    let oe = f.part(File::id(Some(oriented))?, "ORIENTED_EDGE")?;
                    let (edge, flip) = self.edge(File::id(oe.get(3))?, b, vertices, edges)?;
                    let along = File::truth(oe.get(4))?;
                    lp.push(Coedge { edge, reversed: along == flip });
                }
                if !forward {
                    lp.reverse();
                    for c in &mut lp {
                        c.reversed = !c.reversed;
                    }
                }
                loops.push(lp);
            }
            let mut flipped = !same;
            let only_seams = {
                let mut seen: BTreeMap<usize, usize> = BTreeMap::new();
                for c in loops.iter().flatten() {
                    *seen.entry(c.edge).or_insert(0) += 1;
                }
                seen.values().all(|&n| n == 2)
            };
            if only_seams && matches!(surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
                // A whole sphere or torus, bounded by nothing but a point or its own seams. There's nothing to share
                // with other faces, so it's built as this kernel always builds one.
                let frame = *surface.frame();
                let whole = match surface {
                    Surface::Sphere { radius, .. } => crate::build::sphere(&frame, radius, self.feature)?,
                    Surface::Torus { major, minor, .. } => crate::build::torus(&frame, major, minor, self.feature)?,
                    _ => return Err(bad(format!("a {} face has no edges", surface.name()))),
                };
                let inside_out = flipped != invert;
                let base_v = b.vertices.len();
                b.vertices.extend(whole.vertices.iter().copied());
                let base_e = b.edges.len();
                b.edges.extend(whole.edges.iter().map(|e| Edge { curve: e.curve.clone(), t0: e.t0, t1: e.t1, v0: e.v0 + base_v, v1: e.v1 + base_v }));
                for piece in &whole.faces {
                    let mut loops: Vec<Vec<Coedge>> = piece.loops.iter().map(|lp| lp.iter().map(|c| Coedge { edge: c.edge + base_e, reversed: c.reversed }).collect()).collect();
                    if inside_out {
                        for lp in &mut loops {
                            lp.reverse();
                            for c in lp.iter_mut() {
                                c.reversed = !c.reversed;
                            }
                        }
                    }
                    out.push(RawFace { surface: piece.surface.clone(), flipped: piece.flipped != inside_out, loops, points: Vec::new(), tag });
                }
                continue;
            }
            if invert {
                flipped = !flipped;
                for lp in &mut loops {
                    lp.reverse();
                    for c in lp.iter_mut() {
                        c.reversed = !c.reversed;
                    }
                }
            }
            out.push(RawFace { surface, flipped, loops, points, tag });
        }
        Ok(())
    }

    fn vertex(&self, id: usize, b: &mut Builder, vertices: &mut BTreeMap<usize, usize>) -> Result<usize> {
        if let Some(&v) = vertices.get(&id) {
            return Ok(v);
        }
        let p = self.file.point(File::id(self.file.part(id, "VERTEX_POINT")?.get(1))?)?;
        let v = b.add_vertex(p);
        vertices.insert(id, v);
        Ok(v)
    }

    /// An edge, with whether this kernel's direction for it is the opposite of the file's.
    fn edge(&self, id: usize, b: &mut Builder, vertices: &mut BTreeMap<usize, usize>, edges: &mut BTreeMap<usize, (usize, bool)>) -> Result<(usize, bool)> {
        if let Some(&e) = edges.get(&id) {
            return Ok(e);
        }
        let f = self.file;
        let a = f.part(id, "EDGE_CURVE")?;
        let (start, end) = (self.vertex(File::id(a.get(1))?, b, vertices)?, self.vertex(File::id(a.get(2))?, b, vertices)?);
        let same = File::truth(a.get(4))?;
        let mut geom = File::id(a.get(3))?;
        // A curve on a surface, or a seam, is its 3D curve with extras.
        while f.has(geom, "SURFACE_CURVE") || f.has(geom, "SEAM_CURVE") {
            let part = f.part(geom, "SURFACE_CURVE").or_else(|_| f.part(geom, "SEAM_CURVE"))?;
            geom = File::id(part.get(1))?;
        }
        let (p, q) = (b.vertices[start], b.vertices[end]);
        let closed = start == end || p.dist(q) <= 2.0 * TOL;
        let (curve, ta, tb): (Curve, f64, f64) = match f.name(geom) {
            "LINE" => {
                if closed {
                    return Err(bad(format!("#{id} is a straight edge that starts and ends at one point")));
                }
                (Curve::line_through(p, q), 0.0, p.dist(q))
            }
            "CIRCLE" => {
                let c = f.part(geom, "CIRCLE")?;
                let frame = f.placement(File::id(c.get(1))?)?;
                let radius = File::number(c.get(2))? * f.length;
                let angle = |r: Vec3| {
                    let l = frame.to_local(r);
                    l.y.atan2(l.x)
                };
                (Curve::Circle { frame, radius }, angle(p), angle(q))
            }
            "ELLIPSE" => {
                let c = f.part(geom, "ELLIPSE")?;
                let frame = f.placement(File::id(c.get(1))?)?;
                let (major, minor) = (File::number(c.get(2))? * f.length, File::number(c.get(3))? * f.length);
                let angle = |r: Vec3| {
                    let l = frame.to_local(r);
                    (l.y / minor).atan2(l.x / major)
                };
                (Curve::Ellipse { frame, major, minor }, angle(p), angle(q))
            }
            name if name.starts_with("B_SPLINE_CURVE") || f.has(geom, "B_SPLINE_CURVE") => {
                let s = f.spline(geom)?;
                let (u0, u1) = s.domain();
                let (ta, tb) = if closed { (u0, u1) } else { (s.closest(p, u0, u1).0, s.closest(q, u0, u1).0) };
                (Curve::Spline(Arc::new(s)), ta, tb)
            }
            other => return Err(Error::Unsupported(format!("an edge uses {other}"))),
        };
        let periodic = matches!(curve, Curve::Circle { .. } | Curve::Ellipse { .. });
        // Kernel edges run with their parameter increasing. On a curve that doesn't wrap around, the two ends settle
        // which way that is, and some writers' flag here contradicts them. On one that does, only the flag can say.
        let forward = if periodic { same } else { ta <= tb };
        let (t0, t1, v0, v1, flip) = if forward { (ta, tb, start, end, false) } else { (tb, ta, end, start, true) };
        let mut t1 = t1;
        if periodic {
            while t1 <= t0 + 1e-12 {
                t1 += TAU;
            }
        }
        if !(t1 > t0) {
            return Err(bad(format!("#{id} is an edge with no length")));
        }
        let e = b.add_edge(Edge { curve, t0, t1, v0, v1 });
        edges.insert(id, (e, flip));
        Ok((e, flip))
    }
}

/// A spline in a plane square to `dir`, as a swept surface whose coordinates are in that plane.
fn swept_along(spline: &Spline, dir: Vec3) -> Result<Surface> {
    let origin = spline.start();
    let x = dir.any_perpendicular();
    let frame = Frame::new(origin, dir, x);
    // The surface's section square to its direction, which an affine map gives exactly.
    let flat = spline.mapped(|p| {
        let l = frame.to_local(p);
        v3(l.x, l.y, 0.0)
    });
    // Too long a curve for one face is cut into pieces later, along with the faces on it.
    flat.face_ranges().map_err(|e| Error::Unsupported(format!("SURFACE_OF_LINEAR_EXTRUSION of a curve that can't be used: {e}")))?;
    Ok(Surface::Extruded { frame, curve: Arc::new(PlaneCurve::new(flat)) })
}

/// A spline in a plane through the axis, as a swept surface.
fn swept_around(spline: &Spline, origin: Vec3, z: Vec3) -> Result<Surface> {
    // The half-plane the curve's in, from a point of it off the axis.
    let (u0, u1) = spline.domain();
    let off = (0..=16)
        .map(|i| spline.eval(u0 + (u1 - u0) * i as f64 / 16.0) - origin)
        .map(|d| d - z * d.dot(z))
        .max_by(|a, b| a.norm().total_cmp(&b.norm()))
        .ok_or_else(|| bad("an empty curve"))?;
    if off.norm() <= TOL {
        return Err(Error::Unsupported("SURFACE_OF_REVOLUTION of a curve on its own axis".into()));
    }
    let x = off.normalized();
    let frame = Frame::new(origin, z, x);
    let flat = spline.mapped(|p| {
        let l = frame.to_local(p);
        v3(l.x, l.z, 0.0)
    });
    if spline.points().iter().any(|p| frame.to_local(*p).y.abs() > 1e-6) {
        return Err(Error::Unsupported("SURFACE_OF_REVOLUTION of a curve not in a plane through its axis".into()));
    }
    single_piece(&flat, "SURFACE_OF_REVOLUTION")?;
    Ok(Surface::Revolved { frame, curve: Arc::new(PlaneCurve::new(flat)) })
}

/// A swept surface here needs a curve that one face can cover. Larger ones would need cutting by the curve,
/// which isn't written yet.
fn single_piece(flat: &Spline, what: &str) -> Result<()> {
    match flat.face_pieces() {
        Ok(p) if p.len() == 1 && matches!(p[0], crate::spline::FacePiece::Curve(_)) => Ok(()),
        Ok(_) => Err(Error::Unsupported(format!("{what} of a curve that turns too far for one face, or has corners"))),
        Err(e) => Err(Error::Unsupported(format!("{what} of a curve that can't be used: {e}"))),
    }
}

/// Everything that can't be imported, named and counted.
fn unsupported_surfaces(file: &File, shells: &[usize]) -> Vec<(String, usize)> {
    let known = ["B_SPLINE_SURFACE_WITH_KNOTS", "PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE", "TOROIDAL_SURFACE", "DEGENERATE_TOROIDAL_SURFACE", "SURFACE_OF_LINEAR_EXTRUSION", "SURFACE_OF_REVOLUTION"];
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for &s in shells {
        let Ok(list) = file.part(s, "CLOSED_SHELL").and_then(|a| File::list(a.get(1))) else { continue };
        for item in list {
            let Ok(face) = File::id(Some(item)) else { continue };
            let Ok(parts) = file.part(face, "ADVANCED_FACE").or_else(|_| file.part(face, "FACE_SURFACE")) else { continue };
            let Ok(surface) = File::id(parts.get(2)) else { continue };
            // A complex instance names its parts in another order, so check them all.
            let names: Vec<&str> = file.items.get(&surface).map_or(Vec::new(), |i| i.iter().map(|(n, _)| n.as_str()).collect());
            if !names.iter().any(|n| known.contains(n)) {
                let name = names.iter().find(|n| n.contains("SURFACE") && !n.contains("REPRESENTATION") && **n != "BOUNDED_SURFACE").or(names.first()).copied().unwrap_or("nothing");
                // The plain and complex forms of a spline surface are one kind of thing.
                let name = if name.starts_with("B_SPLINE_SURFACE") { "B_SPLINE_SURFACE" } else { name };
                *counts.entry(name.to_string()).or_insert(0) += 1;
            }
        }
    }
    counts.into_iter().collect()
}

/// As close as a vertex will go to all the surfaces around it. Surfaces that are tangent there have nearly the same
/// normal, and two equations that say nearly the same thing can only send a point flying, so a normal that adds
/// nothing is left out, and whatever gap remains between tangent surfaces gets stated and not solved.
fn nearest_to_all(surfaces: &[Surface], start: Vec3, limit: f64) -> Vec3 {
    let mut p = start;
    for _ in 0..40 {
        let mut basis: Vec<(Vec3, f64)> = Vec::with_capacity(3);
        let mut largest = 0.0f64;
        for s in surfaces {
            let f = s.sdf(p);
            largest = largest.max(f.abs());
            let (mut n, mut rhs) = (s.normal(p), -f);
            for (b, r) in &basis {
                let c = n.dot(*b);
                n = n - *b * c;
                rhs -= c * r;
            }
            let len = n.norm();
            if len > 2e-3 {
                basis.push((n * (1.0 / len), rhs / len));
            }
        }
        if largest < 1e-12 {
            break;
        }
        let step = basis.iter().fold(Vec3::ZERO, |sum, (b, r)| sum + *b * *r);
        // Nothing here should go far, and if it wants to, that's for the caller to report.
        if step.norm() > 4.0 * limit {
            return p + step;
        }
        p = p + step;
        if step.norm() < 1e-13 {
            break;
        }
    }
    p
}

/// Moves each vertex onto the surfaces around it, which a file's rounding or its tolerance can leave it off. Straight
/// edges then get rebuilt between their vertices, and B-spline edges go onto both surfaces where those cross. Where
/// they're tangent, as around a fillet, nothing can bring them together, so the file's curve stays. Whatever gap is
/// left over is measured, and the body is held to that.
fn settle(b: &mut Builder, faces: &[RawFace]) -> Result<()> {
    let mut around: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut beside: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (f, face) in faces.iter().enumerate() {
        for c in face.loops.iter().flatten() {
            let e = &b.edges[c.edge];
            for v in [e.v0, e.v1] {
                let list = around.entry(v).or_default();
                if !list.contains(&f) {
                    list.push(f);
                }
            }
            let list = beside.entry(c.edge).or_default();
            if !list.contains(&f) {
                list.push(f);
            }
        }
        for &v in &face.points {
            let list = around.entry(v).or_default();
            if !list.contains(&f) {
                list.push(f);
            }
        }
    }
    // How far a vertex may move: a thousandth of the part, which is more than most files' error, up to a quarter of a
    // millimetre, which the sloppiest real file that reads correctly here needs.
    let used: Vec<usize> = around.keys().copied().collect();
    let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
    for &v in &used {
        lo = lo.min(b.vertices[v]);
        hi = hi.max(b.vertices[v]);
    }
    let limit = (lo.dist(hi) * 1e-3).clamp(1e-3, 0.25);
    for (&v, list) in &around {
        let surfaces: Vec<Surface> = list.iter().map(|&f| faces[f].surface.clone()).collect();
        let start = b.vertices[v];
        let mut moved = nearest_to_all(&surfaces, start, limit);
        // Surfaces close to tangent can send it a long way for little gain. If the file's own place is close enough to
        // all of them, that's a better answer.
        if moved.dist(start) > limit && surfaces.iter().all(|s| s.sdf(start).abs() <= limit) {
            moved = start;
        }
        if moved.dist(start) > limit {
            return Err(Error::NearDegenerate(format!(
                "the vertex at ({:.5}, {:.5}, {:.5}) is {:.2e} mm off the surfaces around it, past the {limit:.2e} mm this part can be healed by",
                start.x,
                start.y,
                start.z,
                moved.dist(start)
            )));
        }
        b.vertices[v] = moved;
    }
    for e in 0..b.edges.len() {
        let (a, c) = (b.vertices[b.edges[e].v0], b.vertices[b.edges[e].v1]);
        let list = beside.get(&e).cloned().unwrap_or_default();
        match b.edges[e].curve.clone() {
            Curve::Line { .. } if a.dist(c) > 0.0 => {
                b.edges[e].curve = Curve::line_through(a, c);
                b.edges[e].t0 = 0.0;
                b.edges[e].t1 = a.dist(c);
            }
            Curve::Spline(s) if list.len() == 2 => {
                let (t0, t1) = (b.edges[e].t0, b.edges[e].t1);
                let (sa, sb) = (&faces[list[0]].surface, &faces[list[1]].surface);
                // An exact spline stays one. Anything else lies only near its surfaces, and goes onto both if they cross.
                let samples: Vec<Vec3> = (0..=32).map(|i| s.eval(t0 + (t1 - t0) * i as f64 / 32.0)).collect();
                if samples.iter().all(|&p| sa.sdf(p).abs() <= TOL && sb.sdf(p).abs() <= TOL) {
                    continue;
                }
                let mut breaks = vec![t0];
                breaks.extend(s.breaks_between(t0, t1));
                breaks.push(t1);
                let mut points = Vec::new();
                let mut crossing = true;
                'walk: for w in breaks.windows(2) {
                    for i in 0..12 {
                        match crate::surface::refine_on_two(sa, sb, s.eval(w[0] + (w[1] - w[0]) * i as f64 / 12.0)) {
                            Some(p) => points.push(p),
                            None => {
                                crossing = false;
                                break 'walk;
                            }
                        }
                    }
                }
                if !crossing {
                    continue;
                }
                points[0] = a;
                points.push(c);
                let Ok(traced) = sampled_from_points(points, sa, sb) else { continue };
                let last = match &traced {
                    Curve::Sampled(t) => (t.points.len() - 1) as f64,
                    _ => 1.0,
                };
                b.edges[e] = Edge { curve: traced, t0: 0.0, t1: last, v0: b.edges[e].v0, v1: b.edges[e].v1 };
            }
            _ => {}
        }
    }
    // What's left: every edge against its vertices and its faces.
    let mut gap = 0.0f64;
    for (&e, list) in &beside {
        let edge = &b.edges[e];
        gap = gap.max(edge.curve.eval(edge.t0).dist(b.vertices[edge.v0])).max(edge.curve.eval(edge.t1).dist(b.vertices[edge.v1]));
        for k in 0..=8 {
            let p = edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 8.0);
            for &f in list {
                gap = gap.max(faces[f].surface.sdf(p).abs());
            }
        }
    }
    if gap > limit {
        return Err(Error::NearDegenerate(format!("this part's edges are up to {gap:.2e} mm off its faces, past the {limit:.2e} mm it can be healed by")));
    }
    // Inside the kernel's own tolerance, there's nothing to state.
    b.gap = if gap > TOL { gap } else { 0.0 };
    Ok(())
}

/// Makes a shell's loops agree: every edge used once each way. A writer's flags aren't enough to go on here. A file can
/// say a loop is reversed when it isn't, and other readers repair that by looking at the shape, so this does too. It keeps
/// the first loop as the file has it, follows the others from there, then lets the simple faces decide which way round
/// that whole group goes.
fn orient_shell(b: &Builder, faces: &mut [RawFace]) {
    let mut loops: Vec<(usize, usize)> = Vec::new();
    for (f, face) in faces.iter().enumerate() {
        for l in 0..face.loops.len() {
            loops.push((f, l));
        }
    }
    // Which loops use each edge, and which way.
    let mut uses: BTreeMap<usize, Vec<(usize, bool)>> = BTreeMap::new();
    for (i, &(f, l)) in loops.iter().enumerate() {
        for c in &faces[f].loops[l] {
            uses.entry(c.edge).or_default().push((i, c.reversed));
        }
    }
    let mut turned: Vec<Option<bool>> = vec![None; loops.len()];
    for start in 0..loops.len() {
        if turned[start].is_some() {
            continue;
        }
        turned[start] = Some(false);
        let mut stack = vec![start];
        let mut component = vec![start];
        while let Some(i) = stack.pop() {
            let me = turned[i].unwrap_or(false);
            let (f, l) = loops[i];
            for c in &faces[f].loops[l] {
                for &(j, reversed) in &uses[&c.edge] {
                    if j != i && turned[j].is_none() {
                        // Opposite directions once turning is taken into account.
                        turned[j] = Some((reversed == c.reversed) != me);
                        stack.push(j);
                        component.push(j);
                    }
                }
            }
        }
        // Which way this whole group should go is for the faces in it to say, each one simple enough to have one chart:
        // its largest loop is its outer one, whatever order the file lists them in, and the others are holes, which run
        // the other way. They vote by area, so a tiny face with an awkward shape can't decide for a big one.
        let mut votes = 0.0f64;
        for &i in &component {
            let (f, l) = loops[i];
            let face = &faces[f];
            if !seams_of(face).is_empty() {
                continue;
            }
            let everything = b.boundary_points(face);
            if !within_half_turn(&face.surface, &everything) || matches!(face.surface, Surface::Sphere { .. }) {
                continue;
            }
            let chart = face.surface.fit_chart(&everything);
            let area_of = |lp: &Vec<Coedge>| -> f64 {
                let mut points = Vec::new();
                for c in lp {
                    let e = &b.edges[c.edge];
                    for k in 0..16 {
                        let u = if c.reversed { 1.0 - k as f64 / 16.0 } else { k as f64 / 16.0 };
                        points.push(face.surface.to_chart(&chart, e.curve.eval(e.t0 + (e.t1 - e.t0) * u)));
                    }
                }
                let mut area = 0.0;
                for k in 0..points.len() {
                    let (p, q) = (points[k], points[(k + 1) % points.len()]);
                    area += p.x * q.y - p.y * q.x;
                }
                area
            };
            let area = area_of(&face.loops[l]);
            if area == 0.0 {
                continue;
            }
            let is_outer = face.loops.iter().all(|other| area_of(other).abs() <= area.abs() * (1.0 + 1e-9));
            // With its own turning applied, the outer loop of an unflipped face runs counter-clockwise, and of a
            // flipped one clockwise, and a hole the other way from that.
            let runs_ccw = (area > 0.0) != turned[i].unwrap_or(false);
            votes += if (runs_ccw != face.flipped) == is_outer { area.abs() } else { -area.abs() };
        }
        let verdict = if votes == 0.0 { None } else { Some(votes > 0.0) };
        if verdict == Some(false) {
            for &i in &component {
                turned[i] = turned[i].map(|t| !t);
            }
        }
    }
    for (i, &(f, l)) in loops.iter().enumerate() {
        if turned[i] == Some(true) {
            let lp = &mut faces[f].loops[l];
            lp.reverse();
            for c in lp.iter_mut() {
                c.reversed = !c.reversed;
            }
        }
    }
}

/// One solid: its outer shell, and any voids.
fn solid(file: &File, feature: u32, outer: usize, voids: &[(usize, bool)]) -> Result<Body> {
    let mut b = Builder { vertices: Vec::new(), edges: Vec::new(), gap: 0.0 };
    let (mut vertices, mut edges) = (BTreeMap::new(), BTreeMap::new());
    let mut faces: Vec<RawFace> = Vec::new();
    let mut reader = Reader { file, next_slot: 0, feature };
    reader.shell(outer, false, &mut b, &mut vertices, &mut edges, &mut faces)?;
    for &(v, invert) in voids {
        reader.shell(v, invert, &mut b, &mut vertices, &mut edges, &mut faces)?;
    }

    orient_shell(&b, &mut faces);
    let mut faces = split_extruded(&mut b, faces)?;
    // Cut until nothing's left to cut. A torus can go round in two directions, and a face goes first in the direction
    // it goes all the way round, since until then a chart can't tell one side of the cut from the other.
    for _pass in 0..3 {
        let mut any = false;
        // Where every edge has to be cut, across all faces, since neighbouring faces share edges.
        let walls: Vec<Option<(Family, [f64; 2])>> = faces
            .iter()
            .map(|f| {
                let around = walls_for(&b, f, Family::AroundAxis).map(|w| (Family::AroundAxis, w));
                let tube = walls_for(&b, f, Family::AroundTube).map(|w| (Family::AroundTube, w));
                match (around, tube) {
                    (Some(a), Some(t)) => Some(if goes_round(&b, f, Family::AroundTube) && !goes_round(&b, f, Family::AroundAxis) { t } else { a }),
                    (a, t) => a.or(t),
                }
            })
            .collect();
        let mut cuts: BTreeMap<usize, Vec<(f64, Option<usize>)>> = BTreeMap::new();
        for (face, wall) in faces.iter().zip(&walls) {
            let Some((family, wall)) = wall else { continue };
            let family = *family;
            any = true;
            let seams = seams_of(face);
            for c in face.loops.iter().flatten() {
                if seams.contains(&c.edge) && seam_on_wall(&b, face, family, c.edge, wall) {
                    continue;
                }
                let e = &b.edges[c.edge];
                for &a in wall {
                    for t in crossings(&e.curve, e.t0, e.t1, &face.surface, family, a) {
                        cuts.entry(c.edge).or_default().push((t, None));
                    }
                }
            }
        }
        if !any {
            break;
        }
        b.cut(&mut faces, &cuts);
        let mut next = Vec::with_capacity(faces.len());
        for (face, wall) in faces.iter().zip(&walls) {
            match wall {
                Some((family, w)) => next.extend(split_face(&mut b, face, *family, *w)?),
                None => next.push(face.clone()),
            }
        }
        faces = next;
    }

    for face in &mut faces {
        keep_own_part_of_closed_patch(&b, face)?;
    }

    // Any closed edge left, on a face that didn't need cutting: a plane's circle whose neighbour was cut
    // shares its cuts, so this is rare, but no edge here can end where it starts.
    let mut cuts: BTreeMap<usize, Vec<(f64, Option<usize>)>> = BTreeMap::new();
    for e in 0..b.edges.len() {
        if b.edges[e].v0 == b.edges[e].v1 && faces.iter().flat_map(|f| f.loops.iter().flatten()).any(|c| c.edge == e) {
            cuts.entry(e).or_default().push(((b.edges[e].t0 + b.edges[e].t1) / 2.0, None));
        }
    }
    b.cut(&mut faces, &cuts);

    settle(&mut b, &faces)?;

    // A face's outer loop comes first here, and a file can list a hole first. It's the largest one in the face's chart.
    for face in &mut faces {
        if face.loops.len() < 2 {
            continue;
        }
        let everything = b.boundary_points(face);
        let chart = face.surface.fit_chart(&everything);
        let area_of = |lp: &Vec<Coedge>| -> f64 {
            let mut points = Vec::new();
            for c in lp {
                let e = &b.edges[c.edge];
                for k in 0..16 {
                    let u = if c.reversed { 1.0 - k as f64 / 16.0 } else { k as f64 / 16.0 };
                    points.push(face.surface.to_chart(&chart, e.curve.eval(e.t0 + (e.t1 - e.t0) * u)));
                }
            }
            let mut area = 0.0;
            for k in 0..points.len() {
                let (p, q) = (points[k], points[(k + 1) % points.len()]);
                area += p.x * q.y - p.y * q.x;
            }
            area.abs()
        };
        let areas: Vec<f64> = face.loops.iter().map(area_of).collect();
        let largest = (0..areas.len()).max_by(|&a, &c| areas[a].total_cmp(&areas[c])).unwrap_or(0);
        face.loops.swap(0, largest);
    }

    // Only what the faces use goes into the body, renumbered.
    let mut body = Body { tolerance: b.gap, ..Body::default() };
    let (mut new_vertex, mut new_edge) = (BTreeMap::new(), BTreeMap::new());
    for face in &faces {
        let mut loops = Vec::with_capacity(face.loops.len());
        for lp in &face.loops {
            let mut out = Vec::with_capacity(lp.len());
            for c in lp {
                let id = *new_edge.entry(c.edge).or_insert_with(|| {
                    let e = &b.edges[c.edge];
                    let mut place = |v: usize, body: &mut Body| *new_vertex.entry(v).or_insert_with(|| body.add_vertex(b.vertices[v]));
                    let (v0, v1) = (place(e.v0, &mut body), place(e.v1, &mut body));
                    body.edges.push(Edge { curve: e.curve.clone(), t0: e.t0, t1: e.t1, v0, v1 });
                    body.edges.len() - 1
                });
                out.push(Coedge { edge: id, reversed: c.reversed });
            }
            loops.push(out);
        }
        body.add_face(face.surface.clone(), face.flipped, loops, face.tag);
    }
    body.compute_shells();
    validate(&body).map_err(|defects| Error::NearDegenerate(format!("the imported solid isn't valid: {}", defects.join("; "))))?;

    // Shells are meant to have their outsides out. If one's inside out, turn it over.
    let mut turned = false;
    for s in 0..body.shells.len() {
        let is_outer = body.shells[s].iter().any(|&f| faces[f].tag.slot < 2 + first_void_slot(file, outer));
        let v = crate::mass::shell_volume(&body, s);
        if (is_outer && v < 0.0) || (!is_outer && v > 0.0) {
            let list = body.shells[s].clone();
            for f in list {
                let face = &mut body.faces[f];
                face.flipped = !face.flipped;
                for lp in &mut face.loops {
                    lp.reverse();
                    for c in lp.iter_mut() {
                        c.reversed = !c.reversed;
                    }
                }
            }
            turned = true;
        }
    }
    if turned {
        validate(&body).map_err(|defects| Error::NearDegenerate(format!("the imported solid isn't valid: {}", defects.join("; "))))?;
    }
    Ok(body)
}

fn first_void_slot(file: &File, outer: usize) -> u32 {
    file.part(outer, "CLOSED_SHELL").and_then(|a| File::list(a.get(1))).map_or(u32::MAX, |l| l.len() as u32)
}

fn seam_on_wall(b: &Builder, face: &RawFace, family: Family, e: usize, wall: &[f64; 2]) -> bool {
    let edge = &b.edges[e];
    let a = phase(&face.surface, family, edge.curve.eval((edge.t0 + edge.t1) / 2.0));
    wall.iter().any(|w| wrap_pi(a - w).abs() < 1e-6)
}

/// Reads every solid in a STEP file, each as its own body, with faces named by `feature` and their position
/// in the file. A file holding anything this can't represent exactly is refused, naming what it found.
pub fn read_step(text: &str, feature: u32) -> Result<Vec<Body>> {
    let items = parse(text)?;
    let (length, angle) = units(&items)?;
    let file = File { items, length, angle };
    if file.items.values().any(|i| i.iter().any(|(n, _)| n == "NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
        return Err(Error::Unsupported("this file is an assembly, with parts placed by transforms, and only single parts can be imported yet".into()));
    }
    // The solids, in file order.
    let mut solids: Vec<(usize, Vec<(usize, bool)>)> = Vec::new();
    for (&id, instance) in &file.items {
        for (name, args) in instance {
            match name.as_str() {
                "MANIFOLD_SOLID_BREP" => solids.push((File::id(args.get(1))?, Vec::new())),
                "BREP_WITH_VOIDS" => {
                    let mut voids = Vec::new();
                    for v in File::list(args.get(2))? {
                        let vid = File::id(Some(v))?;
                        if file.has(vid, "ORIENTED_CLOSED_SHELL") {
                            let o = file.part(vid, "ORIENTED_CLOSED_SHELL")?;
                            voids.push((File::id(o.get(2))?, !File::truth(o.get(3))?));
                        } else {
                            voids.push((vid, false));
                        }
                    }
                    solids.push((File::id(args.get(1))?, voids));
                }
                _ => {}
            }
        }
        let _ = id;
    }
    if solids.is_empty() {
        let surface_model = file.items.values().any(|i| i.iter().any(|(n, _)| n == "SHELL_BASED_SURFACE_MODEL" || n == "GEOMETRIC_CURVE_SET"));
        return Err(if surface_model {
            Error::Unsupported("this file holds surfaces or curves, not closed solids".into())
        } else {
            bad("it has no solids in it")
        });
    }
    // Refuse everything at once, naming it all, before doing any work.
    let shells: Vec<usize> = solids.iter().flat_map(|(o, v)| std::iter::once(*o).chain(v.iter().map(|x| x.0))).collect();
    let missing = unsupported_surfaces(&file, &shells);
    if !missing.is_empty() {
        let list: Vec<String> = missing.iter().map(|(n, c)| format!("{c} on {n}")).collect();
        return Err(Error::Unsupported(format!(
            "this file has faces on surfaces that can't be imported yet: {}. Planes, cylinders, cones, spheres, tori and curves swept along a line or around an axis can",
            list.join(", ")
        )));
    }
    solids.iter().map(|(outer, voids)| solid(&file, feature, *outer, voids)).collect()
}
