//! 2D profiles: closed wires of lines, arcs and splines, one outer and any number of
//! holes. Extrude and revolve consume these.

use crate::error::{Error, Result};
use crate::math::*;
use crate::spline::{FacePiece, Spline};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq)]
pub enum SegGeom {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// Runs from angle `a0` to angle `a1`: counter-clockwise when `a1 > a0`,
    /// clockwise otherwise.
    Arc {
        center: Vec2,
        radius: f64,
        a0: f64,
        a1: f64,
    },
    /// A rational B-spline in the XY plane, which is how an ellipse comes here too. Runs in the direction
    /// of its own parameter.
    Spline(Arc<Spline>),
}

/// One profile segment. `id` names it stably: the side face it generates carries
/// this id, however the segment's dimensions change.
#[derive(Clone, Debug, PartialEq)]
pub struct Seg {
    pub geom: SegGeom,
    pub id: u32,
}

impl Seg {
    pub fn start(&self) -> Vec2 {
        self.at(0.0)
    }

    pub fn end(&self) -> Vec2 {
        self.at(1.0)
    }

    /// Point at fraction `s` along the segment.
    pub fn at(&self, s: f64) -> Vec2 {
        match self.geom {
            SegGeom::Line { a, b } => a.lerp(b, s),
            SegGeom::Arc { center, radius, a0, a1 } => {
                let t = a0 + (a1 - a0) * s;
                center + v2(t.cos(), t.sin()) * radius
            }
            SegGeom::Spline(ref c) => {
                let (u0, u1) = c.domain();
                let p = c.eval(u0 + (u1 - u0) * s);
                v2(p.x, p.y)
            }
        }
    }

    /// Unit direction of travel at fraction `s`.
    pub fn direction(&self, s: f64) -> Vec2 {
        match self.geom {
            SegGeom::Line { a, b } => (b - a).normalized(),
            SegGeom::Arc { a0, a1, .. } => {
                let t = a0 + (a1 - a0) * s;
                let d = v2(-t.sin(), t.cos());
                if a1 >= a0 {
                    d
                } else {
                    -d
                }
            }
            SegGeom::Spline(ref c) => {
                let (u0, u1) = c.domain();
                let d = c.derivative(u0 + (u1 - u0) * s);
                v2(d.x, d.y).normalized()
            }
        }
    }

    pub fn length(&self) -> f64 {
        match self.geom {
            SegGeom::Line { a, b } => a.dist(b),
            SegGeom::Arc { radius, a0, a1, .. } => radius * (a1 - a0).abs(),
            SegGeom::Spline(ref c) => {
                let (u0, u1) = c.domain();
                c.length(u0, u1)
            }
        }
    }

    pub fn reversed(&self) -> Seg {
        let geom = match self.geom {
            SegGeom::Line { a, b } => SegGeom::Line { a: b, b: a },
            SegGeom::Arc { center, radius, a0, a1 } => SegGeom::Arc { center, radius, a0: a1, a1: a0 },
            SegGeom::Spline(ref c) => SegGeom::Spline(Arc::new(c.reversed())),
        };
        Seg { geom, id: self.id }
    }

    /// Splits an arc into pieces of at most 180 degrees. Curved faces must not
    /// span more than that, so this happens before any surface is created. A spline is
    /// cut at its corners and wherever it turns too far, and its straight parts become lines.
    fn split_for_faces(&self, out: &mut Vec<Seg>) {
        match self.geom {
            SegGeom::Spline(ref c) => match c.face_pieces() {
                Ok(pieces) => {
                    for piece in pieces {
                        let geom = match piece {
                            FacePiece::Line(a, b) => SegGeom::Line { a: v2(a.x, a.y), b: v2(b.x, b.y) },
                            FacePiece::Curve(part) => SegGeom::Spline(Arc::new(part)),
                        };
                        out.push(Seg { geom, id: self.id });
                    }
                }
                // `Profile::validate` has refused this before anything gets here.
                Err(_) => out.push(self.clone()),
            },
            SegGeom::Arc { center, radius, a0, a1 } if (a1 - a0).abs() > PI + 1e-9 => {
                let n = ((a1 - a0).abs() / PI).ceil() as usize;
                for i in 0..n {
                    let s0 = a0 + (a1 - a0) * i as f64 / n as f64;
                    let s1 = a0 + (a1 - a0) * (i + 1) as f64 / n as f64;
                    out.push(Seg { geom: SegGeom::Arc { center, radius, a0: s0, a1: s1 }, id: self.id });
                }
            }
            _ => out.push(self.clone()),
        }
    }

    /// Enclosed-area contribution, `1/2 * integral(x dy - y dx)`.
    fn area_term(&self) -> f64 {
        match self.geom {
            SegGeom::Line { a, b } => 0.5 * a.cross(b),
            SegGeom::Arc { center, radius, a0, a1 } => {
                0.5 * (radius * radius * (a1 - a0) + radius * center.x * (a1.sin() - a0.sin())
                    - radius * center.y * (a1.cos() - a0.cos()))
            }
            SegGeom::Spline(ref c) => {
                let (u0, u1) = c.domain();
                let mut breaks = vec![u0];
                breaks.extend(c.breaks_between(u0, u1));
                breaks.push(u1);
                let mut sum = 0.0;
                for w in breaks.windows(2) {
                    let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
                    for &(x, weight) in gauss_24() {
                        let t = mid + half * x;
                        let (p, d, _) = c.derivs(t);
                        sum += (p.x * d.y - p.y * d.x) * weight * half;
                    }
                }
                sum * 0.5
            }
        }
    }
}

/// A closed chain of segments.
#[derive(Clone, Debug, PartialEq)]
pub struct Wire {
    pub segs: Vec<Seg>,
}

impl Wire {
    pub fn signed_area(&self) -> f64 {
        self.segs.iter().map(Seg::area_term).sum()
    }

    pub fn reversed(&self) -> Wire {
        Wire { segs: self.segs.iter().rev().map(Seg::reversed).collect() }
    }

    /// Axis-aligned rectangle with corner at `(x, y)`.
    pub fn rect(x: f64, y: f64, w: f64, h: f64) -> Wire {
        PathBuilder::new(v2(x, y)).line_to(v2(x + w, y)).line_to(v2(x + w, y + h)).line_to(v2(x, y + h)).close()
    }

    /// Rectangle centred on the origin.
    pub fn rect_centered(w: f64, h: f64) -> Wire {
        Wire::rect(-w / 2.0, -h / 2.0, w, h)
    }

    pub fn circle(center: Vec2, radius: f64) -> Wire {
        Wire {
            segs: vec![
                Seg { geom: SegGeom::Arc { center, radius, a0: 0.0, a1: PI }, id: 0 },
                Seg { geom: SegGeom::Arc { center, radius, a0: PI, a1: TAU }, id: 0 },
            ],
        }
    }

    pub fn polygon(points: &[Vec2]) -> Wire {
        let mut b = PathBuilder::new(points[0]);
        for &p in &points[1..] {
            b = b.line_to(p);
        }
        b.close()
    }

    /// Rectangle centred on the origin with rounded corners.
    pub fn rounded_rect(w: f64, h: f64, r: f64) -> Wire {
        let (x, y) = (w / 2.0, h / 2.0);
        // Corners that meet make a slot, or a circle, and there's no straight side left between them. What's left keeps its
        // number, so a side's name doesn't change as the radius comes up to it.
        let r = if (r - x).abs() < MIN_FEATURE / 2.0 {
            x
        } else if (r - y).abs() < MIN_FEATURE / 2.0 {
            y
        } else {
            r
        };
        let mut wire = Wire::rounded_rect_with_sides(x, y, r);
        wire.segs.retain(|s| s.length() >= MIN_FEATURE / 2.0);
        wire
    }

    fn rounded_rect_with_sides(x: f64, y: f64, r: f64) -> Wire {
        PathBuilder::new(v2(-x + r, -y))
            .line_to(v2(x - r, -y))
            .arc_to(v2(x, -y + r), v2(x - r, -y + r), true)
            .line_to(v2(x, y - r))
            .arc_to(v2(x - r, y), v2(x - r, y - r), true)
            .line_to(v2(-x + r, y))
            .arc_to(v2(-x, y - r), v2(-x + r, y - r), true)
            .line_to(v2(-x, -y + r))
            .arc_to(v2(-x + r, -y), v2(-x + r, -y + r), true)
            .close()
    }

    /// The same wire moved by an offset.
    /// Turned by an angle around the origin, then moved. Directions and winding don't change.
    pub fn turned_and_moved(&self, angle: f64, by: Vec2) -> Wire {
        let (s, c) = angle.sin_cos();
        let at = |p: Vec2| v2(c * p.x - s * p.y + by.x, s * p.x + c * p.y + by.y);
        Wire {
            segs: self
                .segs
                .iter()
                .map(|seg| Seg {
                    geom: match seg.geom {
                        SegGeom::Line { a, b } => SegGeom::Line { a: at(a), b: at(b) },
                        SegGeom::Arc { center, radius, a0, a1 } => SegGeom::Arc { center: at(center), radius, a0: a0 + angle, a1: a1 + angle },
                        SegGeom::Spline(ref curve) => SegGeom::Spline(Arc::new(curve.mapped(|p| {
                            let q = at(v2(p.x, p.y));
                            v3(q.x, q.y, p.z)
                        }))),
                    },
                    id: seg.id,
                })
                .collect(),
        }
    }

    pub fn translated(&self, by: Vec2) -> Wire {
        Wire {
            segs: self
                .segs
                .iter()
                .map(|s| Seg {
                    geom: match s.geom {
                        SegGeom::Line { a, b } => SegGeom::Line { a: a + by, b: b + by },
                        SegGeom::Arc { center, radius, a0, a1 } => SegGeom::Arc { center: center + by, radius, a0, a1 },
                        SegGeom::Spline(ref c) => SegGeom::Spline(Arc::new(c.mapped(|p| v3(p.x + by.x, p.y + by.y, p.z)))),
                    },
                    id: s.id,
                })
                .collect(),
        }
    }

    fn flatten(&self) -> Vec<(Vec2, Vec2, usize)> {
        let mut out = Vec::new();
        for (i, s) in self.segs.iter().enumerate() {
            if let SegGeom::Spline(ref c) = s.geom {
                for w in c.graded_polyline(FLAT_PER_SPAN).windows(2) {
                    out.push((v2(w[0].x, w[0].y), v2(w[1].x, w[1].y), i));
                }
                continue;
            }
            let n = match s.geom {
                SegGeom::Line { .. } => 1,
                SegGeom::Arc { a0, a1, .. } => (((a1 - a0).abs() * 24.0 / PI).ceil() as usize).max(4),
                SegGeom::Spline(_) => unreachable!(),
            };
            for k in 0..n {
                out.push((s.at(k as f64 / n as f64), s.at((k + 1) as f64 / n as f64), i));
            }
        }
        out
    }

    /// Winding test. Summing angles avoids the ray-through-a-vertex cases that a
    /// crossing count has to handle.
    fn contains(&self, p: Vec2) -> bool {
        let mut winding = 0.0;
        for (a, b, i) in self.flatten() {
            let (u, v) = (a - p, b - p);
            winding += u.cross(v).atan2(u.dot(v));
            // A point between an arc and its chord is on the other side of the wire from where the chord puts it.
            if let SegGeom::Arc { center, radius, a0, a1 } = self.segs[i].geom {
                let turn = if a1 > a0 { 1.0 } else { -1.0 };
                if p.dist(center) < radius && (b - a).cross(p - a) * turn < 0.0 {
                    winding += turn * TAU;
                }
            }
        }
        winding.abs() > PI
    }

    /// The wire as pieces for checking crossings: lines and arcs as they are, and splines as lines. An arc's chord can
    /// stray by more than a thin wall is thick, so it's only good for working out which way round something is.
    fn pieces(&self) -> Vec<(Piece, usize)> {
        let mut out = Vec::new();
        for (i, s) in self.segs.iter().enumerate() {
            match s.geom {
                SegGeom::Line { a, b } => out.push((Piece::Line(a, b), i)),
                SegGeom::Arc { center, radius, a0, a1 } => out.push((Piece::Arc { center, radius, a0, a1 }, i)),
                SegGeom::Spline(ref c) => {
                    for w in c.graded_polyline(FLAT_PER_SPAN).windows(2) {
                        out.push((Piece::Line(v2(w[0].x, w[0].y), v2(w[1].x, w[1].y)), i));
                    }
                }
            }
        }
        out
    }

    fn check(&self, what: &str) -> Result<()> {
        if self.segs.is_empty() {
            return Err(Error::InvalidProfile(format!("{what} wire is empty")));
        }
        for (i, s) in self.segs.iter().enumerate() {
            if !s.length().is_finite() || s.length() < MIN_FEATURE {
                return Err(Error::InvalidProfile(format!("{what} wire: segment {i} is too short ({:e})", s.length())));
            }
            let next = &self.segs[(i + 1) % self.segs.len()];
            if s.end().dist(next.start()) > TOL {
                return Err(Error::InvalidProfile(format!(
                    "{what} wire is not closed between segments {i} and the next (gap {:e})",
                    s.end().dist(next.start())
                )));
            }
        }
        if self.signed_area().abs() < MIN_FEATURE * MIN_FEATURE {
            return Err(Error::InvalidProfile(format!("{what} wire encloses no area")));
        }
        let flat = self.pieces();
        for i in 0..flat.len() {
            for j in i + 1..flat.len() {
                let (first, si) = flat[i];
                let (second, sj) = flat[j];
                // Pieces of one segment and neighbouring segments meet at their
                // shared endpoint by construction.
                let n = self.segs.len();
                let neighbours = (si + 1) % n == sj || (sj + 1) % n == si;
                let curved = |k: usize| matches!(self.segs[k].geom, SegGeom::Spline(_));
                if si == sj || neighbours {
                    // Lines and arcs can't cross themselves, and these meet at their shared end. A spline
                    // can do either, so only the pieces right beside each other are left out.
                    if !(curved(si) || curved(sj)) {
                        continue;
                    }
                    let side_by_side = (first.end() - second.start()).norm() <= TOL || (first.start() - second.end()).norm() <= TOL;
                    if j == i + 1 || side_by_side {
                        continue;
                    }
                }
                if first.crosses(&second) {
                    return Err(Error::InvalidProfile(format!(
                        "{what} wire crosses itself between segments {si} and {sj}"
                    )));
                }
            }
        }
        Ok(())
    }
}

impl Wire {
    /// Whether the end of this chain comes back to its start.
    pub fn is_closed(&self) -> bool {
        match (self.segs.first(), self.segs.last()) {
            (Some(a), Some(b)) => b.end().dist(a.start()) <= TOL,
            _ => false,
        }
    }

    /// Checks a chain that needn't close: connected, not too short, in the plane and crossing nowhere.
    pub fn check_open(&self) -> Result<()> {
        if self.segs.is_empty() {
            return Err(Error::InvalidProfile("an open curve has to have something in it".to_string()));
        }
        for (i, s) in self.segs.iter().enumerate() {
            if let SegGeom::Spline(ref c) = s.geom {
                if c.points().iter().any(|p| p.z != 0.0) {
                    return Err(Error::InvalidProfile("a spline in a sketch has to lie in the plane of the sketch".to_string()));
                }
                c.face_pieces()?;
            }
            let (a, b) = (s.start(), s.end());
            if !(a.x.abs().max(a.y.abs()).max(b.x.abs()).max(b.y.abs()) <= MAX_COORD) {
                return Err(Error::InvalidProfile(format!("a coordinate is beyond the {MAX_COORD} mm this kernel works within")));
            }
            if !s.length().is_finite() || s.length() < MIN_FEATURE {
                return Err(Error::InvalidProfile(format!("segment {i} of an open curve is too short ({:e})", s.length())));
            }
            if let Some(next) = self.segs.get(i + 1) {
                if s.end().dist(next.start()) > TOL {
                    return Err(Error::InvalidProfile(format!("an open curve has a gap of {:e} after segment {i}", s.end().dist(next.start()))));
                }
            }
        }
        let flat = self.flatten();
        for i in 0..flat.len() {
            for j in i + 1..flat.len() {
                let (a, b, si) = flat[i];
                let (c, d, sj) = flat[j];
                let curved = |k: usize| matches!(self.segs[k].geom, SegGeom::Spline(_));
                if (si == sj || si + 1 == sj) && (!(curved(si) || curved(sj)) || j == i + 1) {
                    continue;
                }
                if segments_cross(a, b, c, d) {
                    return Err(Error::InvalidProfile(format!("an open curve crosses itself between segments {si} and {sj}")));
                }
            }
        }
        Ok(())
    }
}

/// Pieces per knot interval when a spline is checked for crossings.
const FLAT_PER_SPAN: usize = 16;

fn segments_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let side = |p: Vec2, q: Vec2, r: Vec2| (q - p).cross(r - p);
    let (d1, d2) = (side(a, b, c), side(a, b, d));
    let (d3, d4) = (side(c, d, a), side(c, d, b));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

/// A piece of a wire, for telling whether two wires cross.
#[derive(Clone, Copy)]
enum Piece {
    Line(Vec2, Vec2),
    Arc { center: Vec2, radius: f64, a0: f64, a1: f64 },
}

impl Piece {
    fn start(&self) -> Vec2 {
        match *self {
            Piece::Line(a, _) => a,
            Piece::Arc { center, radius, a0, .. } => center + v2(a0.cos(), a0.sin()) * radius,
        }
    }

    fn end(&self) -> Vec2 {
        match *self {
            Piece::Line(_, b) => b,
            Piece::Arc { center, radius, a1, .. } => center + v2(a1.cos(), a1.sin()) * radius,
        }
    }

    /// Whether a point on the arc's circle is strictly along the arc.
    fn along_arc(a0: f64, a1: f64, angle: f64) -> bool {
        let (lo, hi) = (a0.min(a1), a0.max(a1));
        let mut a = angle;
        while a < lo {
            a += TAU;
        }
        while a > hi + 1e-15 && a - TAU >= lo {
            a -= TAU;
        }
        a > lo && a < hi
    }

    fn line_crosses_arc(a: Vec2, b: Vec2, center: Vec2, radius: f64, a0: f64, a1: f64) -> bool {
        let (d, f) = (b - a, a - center);
        let (qa, qb, qc) = (d.dot(d), 2.0 * f.dot(d), f.dot(f) - radius * radius);
        let disc = qb * qb - 4.0 * qa * qc;
        // Just touching isn't crossing, as with two lines.
        if qa <= 0.0 || disc <= 0.0 {
            return false;
        }
        let root = disc.sqrt();
        [(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)].iter().any(|&t| {
            if !(t > 0.0 && t < 1.0) {
                return false;
            }
            let q = a + d * t - center;
            Piece::along_arc(a0, a1, q.y.atan2(q.x))
        })
    }

    fn arcs_cross(c0: Vec2, r0: f64, a0: (f64, f64), c1: Vec2, r1: f64, a1: (f64, f64)) -> bool {
        let between = c1 - c0;
        let d = between.norm();
        if d <= 0.0 || d >= r0 + r1 || d <= (r0 - r1).abs() {
            return false;
        }
        let along = (d * d + r0 * r0 - r1 * r1) / (2.0 * d);
        let h = (r0 * r0 - along * along).max(0.0).sqrt();
        let unit = between * (1.0 / d);
        let foot = c0 + unit * along;
        let across = v2(-unit.y, unit.x) * h;
        [foot + across, foot - across].iter().any(|&q| {
            let (u, v) = (q - c0, q - c1);
            Piece::along_arc(a0.0, a0.1, u.y.atan2(u.x)) && Piece::along_arc(a1.0, a1.1, v.y.atan2(v.x))
        })
    }

    fn crosses(&self, other: &Piece) -> bool {
        match (*self, *other) {
            (Piece::Line(a, b), Piece::Line(c, d)) => segments_cross(a, b, c, d),
            (Piece::Line(a, b), Piece::Arc { center, radius, a0, a1 }) | (Piece::Arc { center, radius, a0, a1 }, Piece::Line(a, b)) => Piece::line_crosses_arc(a, b, center, radius, a0, a1),
            (Piece::Arc { center, radius, a0, a1 }, Piece::Arc { center: c2, radius: r2, a0: b0, a1: b1 }) => Piece::arcs_cross(center, radius, (a0, a1), c2, r2, (b0, b1)),
        }
    }
}

/// Builds a wire from connected line and arc moves.
pub struct PathBuilder {
    start: Vec2,
    cursor: Vec2,
    segs: Vec<Seg>,
    next_id: u32,
}

impl PathBuilder {
    pub fn new(start: Vec2) -> PathBuilder {
        PathBuilder { start, cursor: start, segs: Vec::new(), next_id: 0 }
    }

    pub fn line_to(mut self, p: Vec2) -> PathBuilder {
        self.segs.push(Seg { geom: SegGeom::Line { a: self.cursor, b: p }, id: self.next_id });
        self.next_id += 1;
        self.cursor = p;
        self
    }

    /// Arc from the cursor to `end` about `center`, the short way or the long
    /// way as needed to travel in the requested direction.
    pub fn arc_to(mut self, end: Vec2, center: Vec2, ccw: bool) -> PathBuilder {
        let radius = self.cursor.dist(center);
        let a0 = (self.cursor.y - center.y).atan2(self.cursor.x - center.x);
        let mut a1 = (end.y - center.y).atan2(end.x - center.x);
        if ccw {
            while a1 <= a0 + 1e-12 {
                a1 += TAU;
            }
        } else {
            while a1 >= a0 - 1e-12 {
                a1 -= TAU;
            }
        }
        self.segs.push(Seg { geom: SegGeom::Arc { center, radius, a0, a1 }, id: self.next_id });
        self.next_id += 1;
        self.cursor = end;
        self
    }

    /// Where the path has got to.
    pub fn cursor(&self) -> Vec2 {
        self.cursor
    }

    /// A spline from the cursor. It has to start there, and the cursor moves to its end.
    pub fn spline_to(mut self, spline: Spline) -> PathBuilder {
        let end = spline.end();
        self.segs.push(Seg { geom: SegGeom::Spline(Arc::new(spline)), id: self.next_id });
        self.next_id += 1;
        self.cursor = v2(end.x, end.y);
        self
    }

    /// Finishes the path as it is, without closing it, for a sheet to sweep.
    pub fn open(self) -> Wire {
        Wire { segs: self.segs }
    }

    /// Finishes the wire, adding a closing line if the cursor is not at the start.
    pub fn close(mut self) -> Wire {
        if self.cursor.dist(self.start) > TOL {
            let start = self.start;
            self = self.line_to(start);
        }
        Wire { segs: self.segs }
    }
}

/// An outer wire and its holes, with directions normalised: the outer wire runs
/// counter-clockwise and holes run clockwise, so material is always on the left.
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    pub outer: Wire,
    pub holes: Vec<Wire>,
}

impl Profile {
    pub fn new(outer: Wire) -> Profile {
        let outer = if outer.signed_area() < 0.0 { outer.reversed() } else { outer };
        Profile { outer, holes: Vec::new() }
    }

    pub fn with_hole(mut self, hole: Wire) -> Profile {
        let hole = if hole.signed_area() > 0.0 { hole.reversed() } else { hole };
        // Give each hole's segments ids that no other wire in the profile uses.
        let base = 1000 * (self.holes.len() as u32 + 1);
        let segs = hole.segs.iter().map(|s| Seg { geom: s.geom.clone(), id: s.id + base }).collect();
        self.holes.push(Wire { segs });
        self
    }

    /// All wires with long arcs split, ready for building faces.
    pub fn wires_for_faces(&self) -> Vec<Wire> {
        std::iter::once(&self.outer)
            .chain(self.holes.iter())
            .map(|w| {
                let mut segs = Vec::new();
                for s in &w.segs {
                    s.split_for_faces(&mut segs);
                }
                Wire { segs }
            })
            .collect()
    }

    pub fn area(&self) -> f64 {
        self.outer.signed_area() + self.holes.iter().map(Wire::signed_area).sum::<f64>()
    }

    pub fn validate(&self) -> Result<()> {
        for wire in std::iter::once(&self.outer).chain(&self.holes) {
            for seg in &wire.segs {
                let radius = match seg.geom {
                    SegGeom::Arc { center, radius, .. } => radius.max(center.x.abs()).max(center.y.abs()),
                    SegGeom::Line { .. } => 0.0,
                    SegGeom::Spline(ref c) => {
                        if c.points().iter().any(|p| p.z != 0.0) {
                            return Err(Error::InvalidProfile("a spline in a profile has to lie in the plane of the profile".to_string()));
                        }
                        c.face_pieces()?;
                        0.0
                    }
                };
                let (a, b) = (seg.start(), seg.end());
                // Written so that a NaN fails too.
                if !(a.x.abs().max(a.y.abs()).max(b.x.abs()).max(b.y.abs()).max(radius) <= MAX_COORD) {
                    return Err(Error::InvalidProfile(format!("a coordinate is beyond the {MAX_COORD} mm this kernel works within")));
                }
            }
        }
        self.outer.check("outer")?;
        let outer_pieces = self.outer.pieces();
        let bounds_of = |p: &Piece| -> (Vec2, Vec2) {
            match *p {
                Piece::Line(a, b) => (v2(a.x.min(b.x), a.y.min(b.y)), v2(a.x.max(b.x), a.y.max(b.y))),
                // The whole circle, which is enough to rule things out.
                Piece::Arc { center, radius, .. } => (center - v2(radius, radius), center + v2(radius, radius)),
            }
        };
        let bounds = |flat: &[(Vec2, Vec2, usize)]| {
            let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
            for &(a, _, _) in flat {
                lo = v2(lo.x.min(a.x), lo.y.min(a.y));
                hi = v2(hi.x.max(a.x), hi.y.max(a.y));
            }
            // Chords can fall short of an arc's reach by this share of how far across it is.
            let margin = hi - lo;
            (lo - margin * 0.01, hi + margin * 0.01)
        };
        let flats: Vec<Vec<(Vec2, Vec2, usize)>> = self.holes.iter().map(Wire::flatten).collect();
        let boxes: Vec<(Vec2, Vec2)> = flats.iter().map(|f| bounds(f)).collect();
        for (i, h) in self.holes.iter().enumerate() {
            h.check(&format!("hole {i}"))?;
            if !self.outer.contains(h.segs[0].start()) {
                return Err(Error::InvalidProfile(format!("hole {i} is outside the outer wire")));
            }
            let (lo, hi) = boxes[i];
            let hole_pieces = h.pieces();
            for (outer_piece, _) in &outer_pieces {
                let (low, high) = bounds_of(outer_piece);
                if high.x < lo.x || low.x > hi.x || high.y < lo.y || low.y > hi.y {
                    continue;
                }
                for (piece, _) in &hole_pieces {
                    if piece.crosses(outer_piece) {
                        return Err(Error::InvalidProfile(format!("hole {i} crosses the outer wire")));
                    }
                }
            }
        }
        // Sweep across x so that only holes whose bounds overlap are ever compared.
        let mut order: Vec<usize> = (0..self.holes.len()).collect();
        order.sort_by(|&a, &b| boxes[a].0.x.total_cmp(&boxes[b].0.x));
        for (k, &i) in order.iter().enumerate() {
            let (lo, hi) = boxes[i];
            for &j in &order[k + 1..] {
                let (lo2, hi2) = boxes[j];
                if lo2.x > hi.x {
                    break;
                }
                if lo2.y > hi.y || hi2.y < lo.y {
                    continue;
                }
                let (first, second) = (i.min(j), i.max(j));
                let (these, those) = (self.holes[i].pieces(), self.holes[j].pieces());
                for (a, _) in &these {
                    for (b, _) in &those {
                        if a.crosses(b) {
                            return Err(Error::InvalidProfile(format!("holes {first} and {second} cross")));
                        }
                    }
                }
                if self.holes[j].contains(self.holes[i].segs[0].start())
                    || self.holes[i].contains(self.holes[j].segs[0].start())
                {
                    return Err(Error::InvalidProfile(format!("holes {first} and {second} are nested")));
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn areas_are_exact() {
        let circle = Wire::circle(v2(3.0, -2.0), 2.0);
        assert!((circle.signed_area() - PI * 4.0).abs() < 1e-12);
        let rr = Wire::rounded_rect(10.0, 6.0, 1.0);
        assert!((rr.signed_area() - (60.0 - 4.0 + PI)).abs() < 1e-12);
        let p = Profile::new(Wire::rect_centered(10.0, 10.0)).with_hole(Wire::circle(v2(0.0, 0.0), 2.0));
        assert!((p.area() - (100.0 - PI * 4.0)).abs() < 1e-12);
        p.validate().unwrap();
    }

    #[test]
    fn directions_are_normalised_and_ids_stay_distinct() {
        let p = Profile::new(Wire::rect(0.0, 0.0, 1.0, 1.0).reversed())
            .with_hole(Wire::circle(v2(0.5, 0.5), 0.1))
            .with_hole(Wire::circle(v2(0.2, 0.2), 0.05));
        assert!(p.outer.signed_area() > 0.0);
        assert!(p.holes.iter().all(|h| h.signed_area() < 0.0));
        assert_ne!(p.holes[0].segs[0].id, p.holes[1].segs[0].id);
    }

    #[test]
    fn bad_profiles_are_rejected() {
        let bow = Wire::polygon(&[v2(0.0, 0.0), v2(1.0, 1.0), v2(1.0, 0.0), v2(0.0, 1.0)]);
        assert!(Profile::new(bow).validate().is_err());
        let outside = Profile::new(Wire::rect_centered(1.0, 1.0)).with_hole(Wire::circle(v2(5.0, 0.0), 0.1));
        assert!(outside.validate().is_err());
        let crossing = Profile::new(Wire::rect_centered(1.0, 1.0)).with_hole(Wire::circle(v2(0.5, 0.0), 0.2));
        assert!(crossing.validate().is_err());
        let open = Wire { segs: vec![Seg { geom: SegGeom::Line { a: v2(0.0, 0.0), b: v2(1.0, 0.0) }, id: 0 }] };
        assert!(Profile::new(open).validate().is_err());
    }
}
