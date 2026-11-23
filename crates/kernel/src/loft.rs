//! A solid through sections: one free-form patch for each stretch of outline, running from section to section.
//!
//! Each outline piece becomes rational Bezier pieces in 3D, and matching pieces of all the sections are brought
//! to one degree and one set of breaks, so that their control points correspond. Those then go through the
//! sections: straight between two, and as a cubic through three or more. A line stays a line and a circle stays a
//! circle, exactly, because interpolating control points in homogeneous form does that.

use crate::error::{Error, Result};
use crate::math::*;
use crate::curve::Curve;
use crate::patch::{interpolation_knots, Collocation, Patch};
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::spline::{Hom, Spline};
use crate::surface::Surface;
use crate::build::{SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::topo::{Body, Coedge, FaceTag};
use crate::validate::validate;
use std::sync::Arc;

/// One outline piece in 3D, as rational Bezier pieces over 0 to 1.
#[derive(Clone)]
pub(crate) struct Chain {
    pub(crate) spans: Vec<Vec<Hom>>,
    pub(crate) breaks: Vec<f64>,
    pub(crate) id: u32,
}

fn split_span(h: &[Hom], s: f64) -> (Vec<Hom>, Vec<Hom>) {
    let n = h.len();
    let mut work = h.to_vec();
    let (mut left, mut right) = (vec![Hom::ZERO; n], vec![Hom::ZERO; n]);
    for level in 0..n {
        left[level] = work[0];
        right[n - 1 - level] = work[n - 1 - level];
        for i in 0..n - 1 - level {
            work[i] = work[i].lerp(work[i + 1], s);
        }
    }
    (left, right)
}

fn elevate(h: &[Hom]) -> Vec<Hom> {
    let p = h.len() - 1;
    let mut out = Vec::with_capacity(p + 2);
    out.push(h[0]);
    for i in 1..=p {
        out.push(h[i - 1].lerp(h[i], 1.0 - i as f64 / (p + 1) as f64));
    }
    out.push(h[p]);
    out
}

impl Chain {
    pub(crate) fn of(seg: &Seg, frame: &Frame) -> Result<Chain> {
        let lift = |q: Vec2, w: f64| Hom::of(frame.point(q.x, q.y, 0.0), w);
        let id = seg.id;
        match seg.geom {
            SegGeom::Line { a, b } => Ok(Chain { spans: vec![vec![lift(a, 1.0), lift(b, 1.0)]], breaks: vec![0.0, 1.0], id }),
            SegGeom::Arc { center, radius, a0, a1 } => {
                let n = (((a1 - a0).abs() / (PI / 2.0)) - 1e-9).ceil().max(1.0) as usize;
                let step = (a1 - a0) / n as f64;
                let on = |a: f64, r: f64| center + v2(a.cos(), a.sin()) * r;
                let w = (step / 2.0).cos();
                let spans = (0..n)
                    .map(|i| {
                        let a = a0 + step * i as f64;
                        vec![lift(on(a, radius), 1.0), lift(on(a + step / 2.0, radius / w), w), lift(on(a + step, radius), 1.0)]
                    })
                    .collect();
                Ok(Chain { spans, breaks: (0..=n).map(|i| i as f64 / n as f64).collect(), id })
            }
            SegGeom::Spline(ref c) => {
                let lifted = c.mapped(|q| frame.point(q.x, q.y, 0.0));
                let (t0, t1) = lifted.domain();
                let pieces = lifted.bezier_spans();
                let first = pieces[0].2[0].w;
                let last = pieces[pieces.len() - 1].2.last().map_or(first, |q| q.w);
                if (first - last).abs() > 1e-9 * first {
                    return Err(Error::Unsupported("a loft can't take a rational spline whose two end weights differ yet".into()));
                }
                let mut breaks = vec![0.0];
                let mut spans = Vec::with_capacity(pieces.len());
                for (_, b, h) in pieces {
                    breaks.push((b - t0) / (t1 - t0));
                    spans.push(h.iter().map(|q| Hom { a: q.a / first, w: q.w / first }).collect());
                }
                Ok(Chain { spans, breaks, id })
            }
        }
    }

    pub(crate) fn start(&self) -> Vec3 {
        self.spans[0][0].point()
    }

    pub(crate) fn degree(&self) -> usize {
        self.spans[0].len() - 1
    }

    fn length(&self) -> f64 {
        self.spans.iter().flat_map(|h| h.windows(2)).map(|w| w[0].point().dist(w[1].point())).sum()
    }

    pub(crate) fn reversed(&self) -> Chain {
        let spans = self.spans.iter().rev().map(|h| h.iter().rev().copied().collect()).collect();
        let breaks = self.breaks.iter().rev().map(|b| 1.0 - b).collect();
        Chain { spans, breaks, id: self.id }
    }

    /// Its two halves by parameter.
    fn halves(&self) -> (Chain, Chain) {
        self.split_at(0.5)
    }

    /// The two pieces either side of a parameter strictly between 0 and 1. At a half, the arithmetic is what halving
    /// always was, exactly, as dividing by a half is.
    fn split_at(&self, x: f64) -> (Chain, Chain) {
        let k = self.breaks.partition_point(|b| *b <= x).clamp(1, self.spans.len()) - 1;
        let (a, b) = (self.breaks[k], self.breaks[k + 1]);
        let (mut first, mut second) = (self.spans[..k].to_vec(), self.spans[k + 1..].to_vec());
        let (mut stops_a, mut stops_b): (Vec<f64>, Vec<f64>) = (self.breaks[..=k].iter().map(|t| t / x).collect(), Vec::new());
        if (a - x).abs() < 1e-12 {
            second.insert(0, self.spans[k].clone());
            stops_a.truncate(k + 1);
            stops_b.extend(self.breaks[k..].iter().map(|t| (t - x) / (1.0 - x)));
        } else {
            let (l, r) = split_span(&self.spans[k], (x - a) / (b - a));
            first.push(l);
            second.insert(0, r);
            stops_a.push(1.0);
            stops_b.push(0.0);
            stops_b.extend(self.breaks[k + 1..].iter().map(|t| (t - x) / (1.0 - x)));
        }
        (Chain { spans: first, breaks: stops_a, id: self.id }, Chain { spans: second, breaks: stops_b, id: self.id })
    }

    /// The point closest to `p`: which parameter, the point, and how far.
    fn closest(&self, p: Vec3) -> (f64, Vec3, f64) {
        let mut best = (0.0, self.start(), f64::INFINITY);
        for (k, h) in self.spans.iter().enumerate() {
            let (a, b) = (self.breaks[k], self.breaks[k + 1]);
            let at = |s: f64| bezier_at(h, s).point();
            let steps = 24;
            let (mut lo, mut hi, mut d) = (0.0, 0.0, f64::INFINITY);
            for i in 0..=steps {
                let s = i as f64 / steps as f64;
                let e = at(s).dist(p);
                if e < d {
                    d = e;
                    lo = (s - 1.0 / steps as f64).max(0.0);
                    hi = (s + 1.0 / steps as f64).min(1.0);
                }
            }
            // The distance has one low point this close in, so golden section finds it.
            let g = 0.5 * (5f64.sqrt() - 1.0);
            for _ in 0..80 {
                let (m1, m2) = (hi - g * (hi - lo), lo + g * (hi - lo));
                if at(m1).dist(p) <= at(m2).dist(p) {
                    hi = m2;
                } else {
                    lo = m1;
                }
            }
            let s = (lo + hi) / 2.0;
            let q = at(s);
            if q.dist(p) < best.2 {
                best = (a + (b - a) * s, q, q.dist(p));
            }
        }
        best
    }

    /// Control points at this degree, cut at these breaks, with each shared end kept once.
    pub(crate) fn control_points(&self, degree: usize, breaks: &[f64]) -> Vec<Hom> {
        let mut out: Vec<Hom> = Vec::new();
        for (k, span) in self.spans.iter().enumerate() {
            let mut h = span.clone();
            while h.len() - 1 < degree {
                h = elevate(&h);
            }
            let (a, b) = (self.breaks[k], self.breaks[k + 1]);
            let mut from = a;
            let mut rest = h;
            let mut pieces = Vec::new();
            for &x in breaks.iter().filter(|x| **x > a + 1e-12 && **x < b - 1e-12) {
                let (l, r) = split_span(&rest, (x - from) / (b - from));
                pieces.push(l);
                rest = r;
                from = x;
            }
            pieces.push(rest);
            for piece in pieces {
                let skip = usize::from(!out.is_empty());
                out.extend_from_slice(&piece[skip..]);
            }
        }
        out
    }
}

/// A rational Bezier piece's point, by de Casteljau.
fn bezier_at(h: &[Hom], s: f64) -> Hom {
    let mut work = h.to_vec();
    for level in (1..work.len()).rev() {
        for k in 0..level {
            work[k] = work[k].lerp(work[k + 1], s);
        }
    }
    work[0]
}

fn hs(a: Hom, s: f64) -> Hom {
    Hom { a: a.a * s, w: a.w * s }
}

fn hp(a: Hom, b: Hom) -> Hom {
    Hom { a: a.a + b.a, w: a.w + b.w }
}

fn hm(a: Hom, b: Hom) -> Hom {
    Hom { a: a.a - b.a, w: a.w - b.w }
}

fn wire_centroid(chains: &[Chain]) -> Vec3 {
    let mut sum = Vec3::ZERO;
    for c in chains {
        sum += c.start();
    }
    sum / chains.len() as f64
}

/// The vector area of an outline, whose direction says which way around it runs.
pub(crate) fn area_vector(chains: &[Chain]) -> Vec3 {
    let mut points = Vec::new();
    for c in chains {
        for h in &c.spans {
            for i in 0..8 {
                let s = i as f64 / 8.0;
                let mut work = h.clone();
                for level in (1..work.len()).rev() {
                    for k in 0..level {
                        work[k] = work[k].lerp(work[k + 1], s);
                    }
                }
                points.push(work[0].point());
            }
        }
    }
    let o = points[0];
    let mut sum = Vec3::ZERO;
    for w in 1..points.len() {
        sum += (points[w - 1] - o).cross(points[w] - o) * 0.5;
    }
    sum
}

/// A curve for an edge: a line if it's straight, since that's exact and much cheaper.
pub(crate) fn edge_curve(degree: usize, knots: &[f64], line: &[Hom]) -> Result<(Curve, f64, f64)> {
    let points: Vec<Vec3> = line.iter().map(|q| q.point()).collect();
    let (a, b) = (points[0], points[points.len() - 1]);
    let length = a.dist(b);
    if length > MIN_FEATURE {
        let d = (b - a) / length;
        let mut progress = 0.0f64;
        let straight = points.iter().all(|q| {
            let along = (*q - a).dot(d);
            let on = (*q - a - d * along).norm() <= 1e-9 * length && along >= progress - 1e-12;
            progress = along;
            on
        }) && line.iter().all(|q| (q.w - line[0].w).abs() <= 1e-12);
        if straight {
            return Ok((Curve::line_through(a, b), 0.0, length));
        }
    }
    let weights: Vec<f64> = line.iter().map(|q| q.w).collect();
    let spline = Spline::new(degree, knots.to_vec(), points, weights)?;
    let (t0, t1) = spline.domain();
    Ok((Curve::Spline(Arc::new(spline)), t0, t1))
}

/// What a loft goes through: an outline on a frame's XY plane, or a point, which only its first or last can be.
#[derive(Clone, Copy)]
pub enum Section<'a> {
    Outline(&'a Profile, Frame),
    Point(Vec3),
}

/// How a loft leaves its first section, or comes into its last.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum LoftEnd {
    /// However the sections take it.
    #[default]
    Free,
    /// Square to the section's plane, with this much pull: at 1 the sides set off as fast as going straight from end to
    /// end would, and more carries the square start further along.
    Normal(f64),
    /// Rounded over a point, as a dome is, square there to the next section's plane, with this much pull.
    Round(f64),
}

/// What a loft does besides going through its sections in order.
#[derive(Clone, Default)]
pub struct LoftOptions<'a> {
    /// On from the last section back to the first, as a ring, with no caps.
    pub closed: bool,
    /// Curves its sides follow: each an open chain on a frame's XY plane that runs through every section's outline, and
    /// ends at a point section. The sides are bent to them and still go exactly through every section.
    pub guides: Vec<(&'a Wire, Frame)>,
    pub start: LoftEnd,
    pub end: LoftEnd,
}

/// Lofts through sections in order: each a profile on a frame's XY plane, each with the same number of holes.
/// The sides are named from the first section's segments, and the caps are the start and end caps.
pub fn loft(sections: &[(&Profile, Frame)], feature: u32) -> Result<Body> {
    let list: Vec<Section> = sections.iter().map(|(p, f)| Section::Outline(p, *f)).collect();
    loft_through(&list, &LoftOptions::default(), feature)
}

/// A guide, as rational Bezier pieces one after another, and where it crosses each section, as a piece's number plus how
/// far through that piece.
struct Guide {
    spans: Vec<Vec<Hom>>,
    at: Vec<f64>,
}

impl Guide {
    fn of(wire: &Wire, frame: &Frame) -> Result<Guide> {
        let mut spans = Vec::new();
        for seg in &wire.segs {
            spans.extend(Chain::of(seg, frame)?.spans);
        }
        Ok(Guide { spans, at: Vec::new() })
    }

    fn len(&self) -> f64 {
        self.spans.len() as f64
    }

    fn point(&self, g: f64) -> Vec3 {
        let k = (g.max(0.0).floor() as usize).min(self.spans.len() - 1);
        bezier_at(&self.spans[k], (g - k as f64).clamp(0.0, 1.0)).point()
    }

    fn reverse(&mut self) {
        self.spans.reverse();
        for h in &mut self.spans {
            h.reverse();
        }
        let len = self.len();
        for a in &mut self.at {
            *a = len - *a;
        }
    }

    /// Where it crosses a plane, or touches it, in order along it.
    fn crossings(&self, origin: Vec3, z: Vec3, tol: f64) -> Vec<f64> {
        let h = |g: f64| (self.point(g) - origin).dot(z);
        let steps = 48;
        let mut out: Vec<f64> = Vec::new();
        let total = self.spans.len() * steps;
        for i in 0..total {
            let (a, b) = (i as f64 / steps as f64, (i + 1) as f64 / steps as f64);
            let (ha, hb) = (h(a), h(b));
            if ha.abs() <= tol {
                out.push(a);
            } else if ha * hb < 0.0 {
                let (mut lo, mut hi, mut hlo) = (a, b, ha);
                for _ in 0..80 {
                    let mid = (lo + hi) / 2.0;
                    let hm = h(mid);
                    if hm * hlo <= 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                        hlo = hm;
                    }
                }
                out.push((lo + hi) / 2.0);
            }
        }
        if h(self.len()).abs() <= tol {
            out.push(self.len());
        }
        out.dedup_by(|a, b| (*a - *b).abs() < 1.0 / steps as f64);
        out
    }

    /// From `a` to `b` along it, as Bezier pieces, each with where it ends, as a share of the way by length.
    fn portion(&self, a: f64, b: f64) -> Vec<(Vec<Hom>, f64)> {
        let mut pieces = Vec::new();
        let first = (a.floor() as usize).min(self.spans.len() - 1);
        for k in first..self.spans.len() {
            let (lo, hi) = ((a - k as f64).max(0.0), (b - k as f64).min(1.0));
            if hi - lo < 1e-12 {
                if k as f64 >= b {
                    break;
                }
                continue;
            }
            let mut h = self.spans[k].clone();
            if hi < 1.0 {
                h = split_span(&h, hi).0;
            }
            if lo > 0.0 {
                h = split_span(&h, lo / hi).1;
            }
            pieces.push(h);
        }
        let lengths: Vec<f64> = pieces
            .iter()
            .map(|h| {
                let steps = 16;
                (0..steps).map(|i| bezier_at(h, i as f64 / steps as f64).point().dist(bezier_at(h, (i + 1) as f64 / steps as f64).point())).sum::<f64>()
            })
            .collect();
        let total: f64 = lengths.iter().sum::<f64>().max(1e-300);
        let count = pieces.len();
        let mut run = 0.0;
        pieces
            .into_iter()
            .zip(lengths)
            .enumerate()
            .map(|(i, (h, l))| {
                run += l;
                (h, if i + 1 == count { 1.0 } else { run / total })
            })
            .collect()
    }
}

/// Raises a Bezier piece to a degree.
fn raised(h: &[Hom], degree: usize) -> Vec<Hom> {
    let mut h = h.to_vec();
    while h.len() - 1 < degree {
        h = elevate(&h);
    }
    h
}

/// A rational Bezier piece reparameterised so that its end weights are `w0` and `w1`. The curve is the same.
fn reweighted(h: &[Hom], w0: f64, w1: f64) -> Vec<Hom> {
    let p = h.len() - 1;
    let c = ((w1 * h[0].w) / (w0 * h[p].w)).powf(1.0 / p as f64);
    let lambda = w0 / h[0].w;
    h.iter().enumerate().map(|(i, q)| hs(*q, lambda * c.powi(i as i32))).collect()
}

/// One line across a loft as Bezier pieces, span by span, each piece from `a` to `b` of the span's way, cut where `cuts`
/// says, so that every line cut the same way has its pieces at the same places.
fn pieces_of(spans: &[Vec<(Vec<Hom>, f64, f64)>], cuts: &[Vec<f64>]) -> Vec<Vec<Hom>> {
    let mut out = Vec::new();
    for (k, list) in spans.iter().enumerate() {
        for (h, a, b) in list {
            let mut rest = h.clone();
            let mut from = *a;
            for &x in cuts[k].iter().filter(|x| **x > a + 1e-9 && **x < b - 1e-9) {
                let (l, r) = split_span(&rest, (x - from) / (b - from));
                out.push(l);
                rest = r;
                from = x;
            }
            out.push(rest);
        }
    }
    out
}

/// Those pieces as one row of control points, each raised to `degree`, each shared end kept once.
fn flatten(pieces: &[Vec<Hom>], degree: usize) -> Vec<Hom> {
    let mut out: Vec<Hom> = Vec::new();
    for piece in pieces {
        let piece = raised(piece, degree);
        let skip = usize::from(!out.is_empty());
        out.extend_from_slice(&piece[skip..]);
    }
    out
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

/// A number that varies as a Bezier piece does, times a Bezier piece, as one piece of the two degrees added. One that
/// doesn't vary just scales it.
fn times(s: &[f64], h: &[Hom]) -> Vec<Hom> {
    if s.iter().all(|x| *x == s[0]) {
        return h.iter().map(|q| hs(*q, s[0])).collect();
    }
    let (p, q) = (s.len() - 1, h.len() - 1);
    (0..=p + q)
        .map(|k| {
            let mut acc = Hom::ZERO;
            for i in k.saturating_sub(q)..=k.min(p) {
                let j = k - i;
                acc = hp(acc, hs(h[j], s[i] * binomial(p, i) * binomial(q, j) / binomial(p + q, k)));
            }
            acc
        })
        .collect()
}

/// Two such numbers multiplied.
fn times_scalar(a: &[f64], b: &[f64]) -> Vec<f64> {
    let h: Vec<Hom> = b.iter().map(|w| Hom { a: Vec3::ZERO, w: *w }).collect();
    times(a, &h).iter().map(|q| q.w).collect()
}

/// Knots for those rows, from parameters at the sections: each section and cut a knot as many times as the degree.
fn rows_knots(t: &[f64], degree: usize, cuts: &[Vec<f64>]) -> Vec<f64> {
    let mut knots = vec![t[0]; degree + 1];
    for s in 0..cuts.len() {
        for x in &cuts[s] {
            knots.extend(vec![t[s] + (t[s + 1] - t[s]) * x; degree]);
        }
        if s + 1 < cuts.len() {
            knots.extend(vec![t[s + 1]; degree]);
        }
    }
    knots.extend(vec![t[t.len() - 1]; degree + 1]);
    knots
}

/// Solves a small dense system with points for its right-hand side, by elimination with partial pivoting.
fn solve_dense(a: &[Vec<f64>], b: &[Hom]) -> Option<Vec<Hom>> {
    let n = b.len();
    let mut m: Vec<Vec<f64>> = a.to_vec();
    let mut r: Vec<Hom> = b.to_vec();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if !(m[p][c].abs() > 1e-14) {
            return None;
        }
        m.swap(c, p);
        r.swap(c, p);
        for i in c + 1..n {
            let f = m[i][c] / m[c][c];
            if f != 0.0 {
                for j in c..n {
                    m[i][j] -= f * m[c][j];
                }
                r[i] = hm(r[i], hs(r[c], f));
            }
        }
    }
    let mut x = vec![Hom::ZERO; n];
    for c in (0..n).rev() {
        let mut v = r[c];
        for j in c + 1..n {
            v = hm(v, hs(x[j], m[c][j]));
        }
        x[c] = hs(v, 1.0 / m[c][c]);
    }
    Some(x)
}

/// The derivatives at each section of a C2 cubic through `values` at parameters `t`: a ring if `closed`, where `t` has
/// one more, back at the first, and otherwise with each end's derivative given or left free, as a natural spline's is.
fn hermite_derivatives(t: &[f64], values: &[Hom], closed: bool, start: Option<Hom>, end: Option<Hom>) -> Option<Vec<Hom>> {
    let n = values.len();
    let h = |k: usize| t[k + 1] - t[k];
    let at = |k: usize| values[k % n];
    let delta = |k: usize| hm(at(k + 1), at(k));
    let mut a = vec![vec![0.0; n]; n];
    let mut b = vec![Hom::ZERO; n];
    for k in 0..n {
        let interior = closed || (k > 0 && k + 1 < n);
        if interior {
            let before = (k + n - 1) % n;
            let (hb, ha) = (h(before), h(k));
            a[k][before] += ha;
            a[k][k] += 2.0 * (hb + ha);
            a[k][(k + 1) % n] += hb;
            b[k] = hs(hp(hs(delta(before), ha / hb), hs(delta(k), hb / ha)), 3.0);
        } else if k == 0 {
            match start {
                Some(d) => {
                    a[0][0] = 1.0;
                    b[0] = d;
                }
                None => {
                    a[0][0] = 2.0;
                    a[0][1] = 1.0;
                    b[0] = hs(delta(0), 3.0 / h(0));
                }
            }
        } else {
            match end {
                Some(d) => {
                    a[k][k] = 1.0;
                    b[k] = d;
                }
                None => {
                    a[k][k] = 2.0;
                    a[k][k - 1] = 1.0;
                    b[k] = hs(delta(k - 1), 3.0 / h(k - 1));
                }
            }
        }
    }
    solve_dense(&a, &b)
}

/// Lofts through sections in order, as `loft` does, and also to a point, round in a ring, along guide curves, or
/// leaving its ends square or rounded: see `LoftOptions`.
pub fn loft_through(sections: &[Section], options: &LoftOptions, feature: u32) -> Result<Body> {
    let bad = |why: String| Err(Error::InvalidProfile(format!("loft: {why}")));
    let n = sections.len();
    if n < 2 {
        return bad(format!("it takes at least two sections, not {n}"));
    }
    let closed = options.closed;
    let point_of = |k: usize| match sections[k] {
        Section::Point(p) => Some(p),
        Section::Outline(..) => None,
    };
    let is_point: Vec<bool> = (0..n).map(|k| point_of(k).is_some()).collect();
    if is_point.iter().all(|p| *p) {
        return bad("it needs an outline to go through, not only points".into());
    }
    if let Some(k) = (1..n - 1).find(|&k| is_point[k]) {
        return bad(format!("only its first and last sections can be points, and section {k} is one"));
    }
    let guided = !options.guides.is_empty();
    if closed {
        if n < 3 {
            return bad(format!("a closed loft goes round through at least three sections, not {n}"));
        }
        if is_point.iter().any(|p| *p) {
            return bad("a closed loft can't go through a point".into());
        }
        if options.start != LoftEnd::Free || options.end != LoftEnd::Free {
            return bad("a closed loft has no ends to make square or round".into());
        }
        if guided {
            return bad("a closed loft can't follow guides yet".into());
        }
    }
    for (end, k, name) in [(options.start, 0, "start"), (options.end, n - 1, "end")] {
        if let LoftEnd::Normal(m) | LoftEnd::Round(m) = end {
            if !(m.is_finite() && m > 0.0 && m <= 100.0) {
                return bad(format!("its {name}'s pull must be above 0 and at most 100, not {m}"));
            }
            if matches!(end, LoftEnd::Normal(_)) && is_point[k] {
                return bad(format!("its {name} is a point, which has no plane to be square to: it can be rounded over instead"));
            }
            if matches!(end, LoftEnd::Round(_)) && !is_point[k] {
                return bad(format!("only a point can be rounded over, and its {name} is an outline, which can be left square instead"));
            }
        }
    }
    if guided && (options.start != LoftEnd::Free || options.end != LoftEnd::Free) {
        return bad("a loft can't both follow guides and set how it leaves its ends yet".into());
    }
    let profile_of = |k: usize| match sections[k] {
        Section::Outline(p, _) => Some(p),
        Section::Point(_) => None,
    };
    let outlines: Vec<usize> = (0..n).filter(|&k| !is_point[k]).collect();
    let holes = profile_of(outlines[0]).map_or(0, |p| p.holes.len());
    if let Some(&k) = outlines.iter().find(|&&k| profile_of(k).map_or(0, |p| p.holes.len()) != holes) {
        return bad(format!("section {k} has a different number of holes from the first"));
    }
    if holes > 0 && is_point.iter().any(|p| *p) {
        return bad("an outline with holes in it can't go to a point".into());
    }
    for &k in &outlines {
        if let Some(p) = profile_of(k) {
            p.validate()?;
        }
    }
    // Each section's wires, as chains. A point's are made once it's known how many pieces the others have.
    let mut wires: Vec<Vec<Vec<Chain>>> = Vec::with_capacity(n);
    for s in sections {
        let mut list = Vec::with_capacity(holes + 1);
        if let Section::Outline(p, f) = s {
            for w in p.wires_for_faces() {
                list.push(w.segs.iter().map(|s| Chain::of(s, f)).collect::<Result<Vec<_>>>()?);
            }
        }
        wires.push(list);
    }

    // Parameters along the loft, by distance between sections, and back to the first for a ring.
    let centers: Vec<Vec3> = (0..n).map(|k| point_of(k).unwrap_or_else(|| wire_centroid(&wires[k][0]))).collect();
    let mut params = vec![0.0];
    let stops = if closed { n + 1 } else { n };
    for k in 1..stops {
        let gap = centers[k - 1].dist(centers[k % n]);
        if !(gap > MIN_FEATURE) {
            return bad("two neighbouring sections are in one place".into());
        }
        params.push(params[params.len() - 1] + gap);
    }
    let total = params[params.len() - 1];
    for p in &mut params {
        *p /= total;
    }
    // Plain lofts go through a single interpolating cubic, as they always have. Anything that sets derivatives goes
    // through a cubic piece between each two sections instead, with each section's row of control points its own.
    let hermite = closed || guided || options.start != LoftEnd::Free || options.end != LoftEnd::Free;
    let (degree_v, knots_v) = interpolation_knots(&params);
    let collocation = if hermite { None } else { Collocation::new(&params, degree_v, &knots_v) };
    if !hermite && collocation.is_none() {
        return bad("these sections can't be interpolated".into());
    }

    // All outlines run counter-clockwise seen from the direction of travel, so that outward is to their right.
    let direction = |k: usize| {
        if closed {
            centers[(k + 1) % n] - centers[(k + n - 1) % n]
        } else if k + 1 < centers.len() {
            centers[k + 1] - centers[k]
        } else {
            centers[k] - centers[k - 1]
        }
    };
    for (k, section) in wires.iter_mut().enumerate() {
        if is_point[k] {
            continue;
        }
        if area_vector(&section[0]).dot(direction(k)) < 0.0 {
            for wire in section.iter_mut() {
                wire.reverse();
                for c in wire.iter_mut() {
                    *c = c.reversed();
                }
            }
        }
    }

    sections_keep_apart(&wires, closed)?;

    // The guides: where each meets each section, its outline cut there so that a side's edge runs along it, and the
    // outlines turned to start at the first guide.
    let size = {
        let mut lo = centers[0];
        let mut hi = centers[0];
        for c in wires.iter().flatten().flatten().flat_map(|c| c.spans.iter().flatten()) {
            lo = lo.min(c.point());
            hi = hi.max(c.point());
        }
        lo.dist(hi).max(1.0)
    };
    let tol = 1e-6 * size;
    let mut guides: Vec<Guide> = Vec::with_capacity(options.guides.len());
    // The piece of the outline each guide starts.
    let mut guide_at: Vec<usize> = Vec::new();
    if guided {
        for (gi, (wire, frame)) in options.guides.iter().enumerate() {
            let mut guide = Guide::of(wire, frame)?;
            if guide.spans.is_empty() {
                return bad(format!("guide {gi} has nothing in it"));
            }
            for k in 0..n {
                let at = match sections[k] {
                    Section::Point(p) => {
                        if guide.point(0.0).dist(p) <= tol {
                            0.0
                        } else if guide.point(guide.len()).dist(p) <= tol {
                            guide.len()
                        } else {
                            return bad(format!("guide {gi} has to end at section {k}, which is a point, and it doesn't"));
                        }
                    }
                    Section::Outline(_, f) => {
                        let mut best = (f64::NAN, f64::INFINITY);
                        for g in guide.crossings(f.origin, f.z, tol) {
                            let q = guide.point(g);
                            let d = wires[k][0].iter().map(|c| c.closest(q).2).fold(f64::INFINITY, f64::min);
                            if d < best.1 {
                                best = (g, d);
                            }
                        }
                        if !(best.1 <= tol) {
                            return Err(Error::InvalidProfile(if best.1.is_finite() {
                                format!("loft: guide {gi} doesn't meet section {k}'s outline: where it crosses that section's plane it's {:.6} away from it", best.1)
                            } else {
                                format!("loft: guide {gi} doesn't reach section {k}'s plane")
                            }));
                        }
                        best.0
                    }
                };
                guide.at.push(at);
            }
            let rising = guide.at.windows(2).all(|w| w[1] > w[0] + 1e-9);
            let falling = guide.at.windows(2).all(|w| w[1] < w[0] - 1e-9);
            if falling {
                guide.reverse();
            } else if !rising {
                return bad(format!("guide {gi} doesn't run through the sections in order"));
            }
            guides.push(guide);
        }
        // Each outline cut where the guides meet it, unless that's a corner already.
        for &k in &outlines {
            let wire = &mut wires[k][0];
            let mut corners = Vec::with_capacity(guides.len());
            for g in &guides {
                let p = g.point(g.at[k]);
                let (c, (x, q, _)) = wire.iter().map(|c| c.closest(p)).enumerate().min_by(|a, b| a.1 .2.total_cmp(&b.1 .2)).unwrap();
                let len = wire.len();
                if q.dist(wire[c].start()) <= tol || x <= 1e-9 {
                    corners.push(wire[c].start());
                } else if q.dist(wire[(c + 1) % len].start()) <= tol || x >= 1.0 - 1e-9 {
                    corners.push(wire[(c + 1) % len].start());
                } else {
                    let (a, b) = wire[c].split_at(x);
                    corners.push(b.start());
                    wire[c] = a;
                    wire.insert(c + 1, b);
                }
            }
            let index = |wire: &[Chain], p: Vec3| (0..wire.len()).min_by(|&a, &b| wire[a].start().dist(p).total_cmp(&wire[b].start().dist(p))).unwrap();
            let first = index(wire, corners[0]);
            wire.rotate_left(first);
            let at: Vec<usize> = corners.iter().map(|c| index(wire, *c)).collect();
            if at.windows(2).any(|w| w[1] <= w[0]) {
                return bad(format!("the guides meet section {k}'s outline in a different order from the first's, or two meet it at one place"));
            }
        }
        // Between each two guides, the same number of pieces in every section, by cutting the longest.
        let gcount = guides.len();
        let groups_of = |wire: &[Chain], at: &[usize]| -> Vec<Vec<Chain>> {
            (0..at.len()).map(|g| wire[at[g]..if g + 1 < at.len() { at[g + 1] } else { wire.len() }].to_vec()).collect()
        };
        let mut split: Vec<Option<Vec<Vec<Chain>>>> = vec![None; n];
        for &k in &outlines {
            let wire = &wires[k][0];
            let corners: Vec<Vec3> = guides.iter().map(|g| g.point(g.at[k])).collect();
            let at: Vec<usize> = corners
                .iter()
                .map(|p| (0..wire.len()).min_by(|&a, &b| wire[a].start().dist(*p).total_cmp(&wire[b].start().dist(*p))).unwrap())
                .collect();
            split[k] = Some(groups_of(wire, &at));
        }
        guide_at = Vec::with_capacity(gcount);
        let mut start = 0;
        for g in 0..gcount {
            let count = outlines.iter().map(|&k| split[k].as_ref().unwrap()[g].len()).max().unwrap_or(1);
            for &k in &outlines {
                let group = &mut split[k].as_mut().unwrap()[g];
                while group.len() < count {
                    let longest = (0..group.len()).max_by(|&a, &b| group[a].length().total_cmp(&group[b].length())).unwrap();
                    let (a, b) = group[longest].halves();
                    group[longest] = a;
                    group.insert(longest + 1, b);
                }
            }
            guide_at.push(start);
            start += count;
        }
        for &k in &outlines {
            wires[k][0] = split[k].take().unwrap().into_iter().flatten().collect();
        }
    }

    let tag = |slot| FaceTag { feature, slot };
    let mut body = Body::default();
    let (mut start_loops, mut end_loops) = (Vec::new(), Vec::new());
    // For a ring, two bands, each its own faces, from the first section to the one halfway round and on back to it.
    let half = n / 2;
    let bands: Vec<(usize, usize, usize, usize)> = if closed { vec![(0, half, 0, 1), (half, n, 1, 0)] } else { vec![(0, n - 1, 0, 1)] };
    let stop_sections: Vec<usize> = if closed { vec![0, half] } else { vec![0, n - 1] };
    let mut all_patches: Vec<Vec<Vec<Arc<Patch>>>> = vec![Vec::with_capacity(holes + 1); bands.len()];
    // How far the outline turns from each section to the one before it, which its holes follow.
    let mut twists = vec![0.0; wires.len()];
    let twist_of = |last: &[Chain], this: &[Chain], axis: Vec3, r: usize| -> f64 {
        let count = last.len();
        let (ca, cb) = (wire_centroid(last), wire_centroid(this));
        let flat = |v: Vec3| v - axis * v.dot(axis);
        let sum: f64 = (0..count)
            .map(|i| {
                let (a, b) = (flat(last[i].start() - ca), flat(this[(i + r) % count].start() - cb));
                axis.dot(a.cross(b)).atan2(a.dot(b))
            })
            .sum();
        sum / count as f64
    };
    for w in 0..=holes {
        if w == 0 && guided {
            for k in 1..n {
                if !is_point[k] && !is_point[k - 1] {
                    let axis = (centers[k] - centers[k - 1]).normalized();
                    twists[k] = twist_of(&wires[k - 1][0], &wires[k][0], axis, 0);
                }
            }
        } else {
            // The same number of pieces in every section, by cutting the longest.
            let count = outlines.iter().map(|&k| wires[k][w].len()).max().unwrap_or(0);
            for &k in &outlines {
                let wire = &mut wires[k][w];
                while wire.len() < count {
                    let longest = (0..wire.len()).max_by(|&a, &b| wire[a].length().total_cmp(&wire[b].length())).unwrap();
                    let (a, b) = wire[longest].halves();
                    wire[longest] = a;
                    wire.insert(longest + 1, b);
                }
            }
            // Each section starts at the piece closest to where the last one started, so the sides don't twist.
            for k in 1..n {
                if is_point[k] || is_point[k - 1] {
                    continue;
                }
                let axis = (centers[k] - centers[k - 1]).normalized();
                let (before, after) = wires.split_at_mut(k);
                let (last, this) = (&before[k - 1][w], &mut after[0][w]);
                let (ca, cb) = (wire_centroid(last), wire_centroid(this));
                let cost = |r: usize| -> f64 { (0..count).map(|i| ((this[(i + r) % count].start() - cb) - (last[i].start() - ca)).norm2()).sum() };
                let costs: Vec<f64> = (0..count).map(cost).collect();
                let turns: Vec<f64> = (0..count).map(|r| twist_of(last, this, axis, r)).collect();
                // The smaller twist, or the clockwise one where they're the same size, as a square's going to a circle are. Either
                // would do; clockwise is what rounding happened to choose for the lofts this kernel's demos and tests have.
                let by_twist = |a: f64, b: f64| if (a.abs() - b.abs()).abs() <= 1e-9 { a.total_cmp(&b) } else { a.abs().total_cmp(&b.abs()) };
                let best = if w == 0 {
                    // Starts that fit as well as each other, as a rectangle's corners do a circle's, can't be told apart by how well
                    // they fit, and rounding would choose. They go by their twist, so that outlines lofted side by side, as a thin
                    // loft's are, twist alike.
                    let least = costs.iter().copied().fold(f64::INFINITY, f64::min);
                    let scale = costs.iter().copied().fold(0.0, f64::max);
                    (0..count).filter(|&r| costs[r] <= least + 1e-9 * scale).min_by(|&a, &b| by_twist(turns[a], turns[b])).unwrap_or(0)
                } else {
                    // A hole twists as the outline does, so that the wall between them doesn't.
                    (0..count)
                        .min_by(|&a, &b| by_twist(turns[a] - twists[k], turns[b] - twists[k]).then(costs[a].total_cmp(&costs[b])))
                        .unwrap_or(0)
                };
                if w == 0 {
                    twists[k] = turns[best];
                }
                this.rotate_left(best);
            }
        }
        let count = wires[outlines[0]][w].len();
        // A point, as that many pieces of nothing, named as its neighbour's.
        for k in 0..n {
            if let Some(p) = point_of(k) {
                let next = if k == 0 { 1 } else { n - 2 };
                let ids: Vec<u32> = wires[next][w].iter().map(|c| c.id).collect();
                let chains = ids.into_iter().map(|id| Chain { spans: vec![vec![Hom::of(p, 1.0), Hom::of(p, 1.0)]], breaks: vec![0.0, 1.0], id }).collect();
                if wires[k].is_empty() {
                    wires[k].push(chains);
                } else {
                    wires[k][w] = chains;
                }
            }
        }

        let stop_vertex: Vec<Vec<usize>> = stop_sections
            .iter()
            .map(|&k| match point_of(k) {
                Some(p) => vec![body.add_vertex(p); count],
                None => wires[k][w].iter().map(|c| body.add_vertex(c.start())).collect(),
            })
            .collect();
        // Control points of every piece, by section and then along the outline.
        struct Piece {
            degree: usize,
            knots: Vec<f64>,
            grid: Vec<Vec<Hom>>,
            id: u32,
        }
        let mut pieces = Vec::with_capacity(count);
        // The rows of control points across the loft, and their knots, when it goes section by section.
        let mut cuts: Vec<Vec<f64>> = vec![Vec::new(); params.len() - 1];
        let mut degree_rows = 3;
        let bent = guided && w == 0;
        if bent {
            // Every piece of every outline with its end weights at 1, the curve the same: then each side's edge, and
            // each guide where it meets the sections, has a weight of 1 there, as the sums below need.
            for &k in &outlines {
                for c in wires[k][0].iter_mut() {
                    for h in c.spans.iter_mut() {
                        *h = reweighted(h, 1.0, 1.0);
                    }
                }
            }
            // The guides' own pieces, with breaks between them as shares of the way between two sections.
            for g in &guides {
                for k in 0..n - 1 {
                    for (_, end) in g.portion(g.at[k], g.at[k + 1]) {
                        if end < 1.0 - 1e-9 {
                            cuts[k].push(end);
                        }
                    }
                }
            }
            for c in &mut cuts {
                c.sort_by(f64::total_cmp);
                c.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            }
        }
        // For a bent loft, each line across it as Bezier pieces, section to section and cut to cut, before the guides.
        let mut bent_columns: Vec<Vec<Vec<Vec<Hom>>>> = Vec::new();
        for i in 0..count {
            let degree = wires.iter().map(|s| s[w][i].degree()).max().unwrap();
            let mut breaks: Vec<f64> = wires.iter().flat_map(|s| s[w][i].breaks.iter().copied()).collect();
            breaks.sort_by(f64::total_cmp);
            breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            let mut lines: Vec<Vec<Hom>> = wires.iter().map(|s| s[w][i].control_points(degree, &breaks)).collect();
            // A point takes its neighbour's weights, so that a circle goes to it as a cone does.
            for k in 0..n {
                if let Some(p) = point_of(k) {
                    let next = if k == 0 { 1 } else { n - 2 };
                    for j in 0..lines[k].len() {
                        lines[k][j] = Hom::of(p, lines[next][j].w);
                    }
                }
            }
            let mut knots = vec![0.0; degree + 1];
            for b in &breaks[1..breaks.len() - 1] {
                knots.extend(vec![*b; degree]);
            }
            knots.extend(vec![1.0; degree + 1]);
            // Across the sections, a line of control points at a time.
            let mut grid = Vec::with_capacity(lines[0].len());
            let mut columns = Vec::new();
            for j in 0..lines[0].len() {
                let values: Vec<Hom> = lines.iter().map(|l| l[j]).collect();
                let column = match &collocation {
                    Some(c) => {
                        if values.len() == 2 {
                            values
                        } else {
                            c.solve(&values)
                        }
                    }
                    None => {
                        // How each end leaves, as a derivative of the homogeneous point whose weight doesn't change.
                        let given = |end: LoftEnd, k: usize| -> Option<Hom> {
                            let weight = values[k].w;
                            match end {
                                LoftEnd::Free => None,
                                LoftEnd::Normal(m) => {
                                    let f = match sections[k] {
                                        Section::Outline(_, f) => f,
                                        Section::Point(_) => unreachable!(),
                                    };
                                    let z = if f.z.dot(direction(k)) >= 0.0 { f.z } else { -f.z };
                                    Some(Hom { a: z * (m * total * weight), w: 0.0 })
                                }
                                LoftEnd::Round(m) => {
                                    let next = if k == 0 { 1 } else { n - 2 };
                                    let apex = values[k].point();
                                    let f = match sections[next] {
                                        Section::Outline(_, f) => f,
                                        Section::Point(_) => unreachable!(),
                                    };
                                    let axis = f.z;
                                    let flat = |v: Vec3| v - axis * v.dot(axis);
                                    let reach = wires[next][w].iter().map(|c| flat(c.start() - apex).norm()).sum::<f64>() / count as f64;
                                    let out = flat(values[next].point() - apex) * (m * total / reach.max(1e-300));
                                    let sign = if k == 0 { 1.0 } else { -1.0 };
                                    Some(Hom { a: out * (sign * weight), w: 0.0 })
                                }
                            }
                        };
                        let Some(d) = hermite_derivatives(&params, &values, closed, given(options.start, 0), given(options.end, n - 1)) else {
                            return bad("these sections can't be interpolated".into());
                        };
                        let spans: Vec<Vec<(Vec<Hom>, f64, f64)>> = (0..params.len() - 1)
                            .map(|k| {
                                let (a, b) = (values[k], values[(k + 1) % n]);
                                let (da, db) = (d[k], d[(k + 1) % n]);
                                let h = params[k + 1] - params[k];
                                vec![(vec![a, hp(a, hs(da, h / 3.0)), hm(b, hs(db, h / 3.0)), b], 0.0, 1.0)]
                            })
                            .collect();
                        let parts = pieces_of(&spans, &cuts);
                        let rows = flatten(&parts, 3);
                        if bent {
                            columns.push(parts);
                        }
                        rows
                    }
                };
                if column.iter().any(|q| !(q.w > 1e-6)) {
                    return bad("going through these sections would take a weight below zero, so they're too different".into());
                }
                grid.push(column);
            }
            bent_columns.push(columns);
            pieces.push(Piece { degree, knots, grid, id: wires[outlines[0]][w][i].id });
        }
        // Bent to the guides. With g a guide, r its side's edge before bending, both weighing 1 at the sections, and the
        // guides' weights multiplied together making W, a line across the loft whose control points are H, with weights h,
        // becomes H W + h (g - r) W, raised as far as these products take it. Its points move by g - r, shared out across
        // the pieces between one guide and the next by how far along they are, which is nothing at the sections, where g
        // and r are the same point, and all of it at the guide's own edge, which then is the guide. So the sections are
        // kept exactly, and the edges follow the guides exactly, arcs and all.
        if bent {
            let groups = guides.len();
            // Where each section's span starts among the pieces.
            let first_piece: Vec<usize> = {
                let mut at = vec![0];
                for c in &cuts {
                    at.push(at[at.len() - 1] + c.len() + 1);
                }
                at
            };
            // Each guide as pieces matched to the rows, its ends on its side's edge exactly.
            let mut guide_parts: Vec<Vec<Vec<Hom>>> = Vec::with_capacity(groups);
            for (g, guide) in guides.iter().enumerate() {
                let rail = &bent_columns[guide_at[g]][0];
                let at_section = |k: usize| if k + 1 < n { rail[first_piece[k]][0] } else { *rail[rail.len() - 1].last().unwrap() };
                let spans: Vec<Vec<(Vec<Hom>, f64, f64)>> = (0..n - 1)
                    .map(|k| {
                        let parts = guide.portion(guide.at[k], guide.at[k + 1]);
                        let last = parts.len() - 1;
                        let mut from = 0.0;
                        let mut list: Vec<(Vec<Hom>, f64, f64)> = Vec::with_capacity(parts.len());
                        for (i, (h, end)) in parts.into_iter().enumerate() {
                            let mut h = reweighted(&h, 1.0, 1.0);
                            if i == 0 {
                                h[0] = at_section(k);
                            } else {
                                h[0] = *list[i - 1].0.last().unwrap();
                            }
                            if i == last {
                                let p = h.len() - 1;
                                h[p] = at_section(k + 1);
                            }
                            list.push((h, from, end));
                            from = end;
                        }
                        list
                    })
                    .collect();
                let parts = pieces_of(&spans, &cuts);
                if parts.len() != rail.len() {
                    return bad(format!("guide {g} couldn't be matched to its side's rows"));
                }
                guide_parts.push(parts);
            }
            let weights = |h: &[Hom]| h.iter().map(|q| q.w).collect::<Vec<f64>>();
            let pieces_count = guide_parts[0].len();
            // W, and for each guide the product of the others' weights, piece by piece.
            let mut all_w: Vec<Vec<f64>> = Vec::with_capacity(pieces_count);
            let mut others: Vec<Vec<Vec<f64>>> = vec![Vec::with_capacity(pieces_count); groups];
            for q in 0..pieces_count {
                let ws: Vec<Vec<f64>> = guide_parts.iter().map(|parts| weights(&parts[q])).collect();
                let product = |skip: Option<usize>| -> Vec<f64> {
                    let mut acc = vec![1.0];
                    for (g, wg) in ws.iter().enumerate() {
                        if Some(g) != skip {
                            acc = times_scalar(&acc, wg);
                        }
                    }
                    acc
                };
                all_w.push(product(None));
                for g in 0..groups {
                    others[g].push(product(Some(g)));
                }
            }
            // (g - r) W for each guide, as points with no weight, all raised to one degree so they can be shared out.
            let mut moves: Vec<Vec<Vec<Hom>>> = Vec::with_capacity(groups);
            for g in 0..groups {
                let rail = &bent_columns[guide_at[g]][0];
                let list: Vec<Vec<Hom>> = (0..pieces_count)
                    .map(|q| {
                        let a = times(&others[g][q], &guide_parts[g][q]);
                        let b = times(&all_w[q], &rail[q]);
                        let d = a.len().max(b.len()) - 1;
                        raised(&a, d).iter().zip(raised(&b, d)).map(|(x, y)| Hom { a: x.a - y.a, w: 0.0 }).collect()
                    })
                    .collect();
                moves.push(list);
            }
            for q in 0..pieces_count {
                let d = (0..groups).map(|g| moves[g][q].len()).max().unwrap() - 1;
                for g in 0..groups {
                    moves[g][q] = raised(&moves[g][q], d);
                }
            }
            let mut finished: Vec<Vec<Vec<Vec<Hom>>>> = Vec::with_capacity(count);
            for (i, piece) in pieces.iter().enumerate() {
                let g = (0..groups).rev().find(|&g| guide_at[g] <= i).unwrap_or(0);
                let end = if g + 1 < groups { guide_at[g + 1] } else { count };
                let size = (end - guide_at[g]) as f64;
                let local = (i - guide_at[g]) as f64;
                let (ma, mb) = (&moves[g], &moves[(g + 1) % groups]);
                let p = piece.degree;
                let mut cols = Vec::with_capacity(piece.grid.len());
                for (j, column) in bent_columns[i].iter().enumerate() {
                    let u = piece.knots[j + 1..=j + p].iter().sum::<f64>() / p as f64;
                    let c = (local + u) / size;
                    let parts: Vec<Vec<Hom>> = (0..pieces_count)
                        .map(|q| {
                            let shared: Vec<Hom> = ma[q].iter().zip(&mb[q]).map(|(x, y)| hp(hs(*x, 1.0 - c), hs(*y, c))).collect();
                            let carried = times(&all_w[q], &column[q]);
                            let moved = times(&weights(&column[q]), &shared);
                            let d = carried.len().max(moved.len()) - 1;
                            raised(&carried, d).iter().zip(raised(&moved, d)).map(|(x, y)| hp(*x, y)).collect()
                        })
                        .collect();
                    cols.push(parts);
                }
                finished.push(cols);
            }
            degree_rows = finished.iter().flatten().flatten().map(|h| h.len() - 1).max().unwrap_or(3).max(3);
            if degree_rows > crate::spline::MAX_DEGREE {
                return bad(format!("its guides would take its sides to degree {degree_rows}, past the {} this kernel works to", crate::spline::MAX_DEGREE));
            }
            for (piece, cols) in pieces.iter_mut().zip(finished) {
                for (column, parts) in piece.grid.iter_mut().zip(cols) {
                    *column = flatten(&parts, degree_rows);
                    if column.iter().any(|q| !(q.w > 1e-6)) {
                        return bad("bending its sides to the guides would take a weight below zero".into());
                    }
                }
            }
        }
        let last_row = pieces[0].grid[0].len() - 1;
        // Where each section's row is, and the knots for the rows from one section to another.
        let section_row = |k: usize| -> usize {
            if !hermite {
                if k == 0 {
                    0
                } else {
                    last_row
                }
            } else {
                (0..k).map(|s| (cuts[s].len() + 1) * degree_rows).sum()
            }
        };
        let band_knots = |a: usize, b: usize| -> (usize, Vec<f64>) {
            if !hermite {
                (degree_v, knots_v.clone())
            } else {
                (degree_rows, rows_knots(&params[a..=b], degree_rows, &cuts[a..b]))
            }
        };
        let rails: Vec<Vec<Coedge>> = bands
            .iter()
            .map(|&(a, b, sa, sb)| {
                let (dv, kv) = band_knots(a, b);
                (0..count)
                    .map(|i| -> Result<Coedge> {
                        let column = &pieces[i].grid[0][section_row(a)..=section_row(b)];
                        let (curve, t0, t1) = edge_curve(dv, &kv, column)?;
                        Ok(body.add_edge(curve, t0, t1, stop_vertex[sa][i], stop_vertex[sb][i]))
                    })
                    .collect::<Result<Vec<_>>>()
            })
            .collect::<Result<_>>()?;
        let (mut bottom, mut top) = (Vec::with_capacity(count), Vec::with_capacity(count));
        let mut patches: Vec<Vec<Arc<Patch>>> = vec![Vec::with_capacity(count); bands.len()];
        for i in 0..count {
            let j = (i + 1) % count;
            let piece = &pieces[i];
            let at_row = |row: usize| piece.grid.iter().map(|c| c[row]).collect::<Vec<Hom>>();
            // The edges along each stop, unless it's a point.
            let mut along: Vec<Option<Coedge>> = Vec::with_capacity(stop_sections.len());
            for (s, &k) in stop_sections.iter().enumerate() {
                if is_point[k] {
                    along.push(None);
                    continue;
                }
                let (curve, a, b) = edge_curve(piece.degree, &piece.knots, &at_row(section_row(k)))?;
                along.push(Some(body.add_edge(curve, a, b, stop_vertex[s][i], stop_vertex[s][j])));
            }
            if !closed {
                bottom.push(along[0]);
                top.push(along[1]);
            }
            for (bi, &(a, b, sa, sb)) in bands.iter().enumerate() {
                let (dv, kv) = band_knots(a, b);
                let (r0, r1) = (section_row(a), section_row(b));
                let (nu, nv) = (piece.grid.len(), r1 - r0 + 1);
                let mut points = Vec::with_capacity(nu * nv);
                let mut weights = Vec::with_capacity(nu * nv);
                for column in &piece.grid {
                    for q in &column[r0..=r1] {
                        points.push(q.point());
                        weights.push(q.w);
                    }
                }
                let patch = Arc::new(Patch::new((piece.degree, dv), (piece.knots.clone(), kv), (nu, nv), points, weights)?);
                patches[bi].push(patch.clone());
                // A flat piece is a plane, which everything downstream handles better and more exactly.
                let surface = flat_frame(&patch).map_or_else(|| Surface::Free { frame: Frame::WORLD, patch }, Surface::Plane);
                let mut edges = Vec::with_capacity(4);
                edges.extend(along[sa]);
                edges.push(rails[bi][j]);
                edges.extend(along[sb].map(|e| e.flipped()));
                edges.push(rails[bi][i].flipped());
                body.add_face(surface, false, vec![edges], tag(SLOT_SEG_BASE + piece.id));
            }
        }
        if !closed {
            if !is_point[0] {
                start_loops.push(bottom.iter().rev().map(|c| c.unwrap().flipped()).collect::<Vec<_>>());
            }
            if !is_point[n - 1] {
                end_loops.push(top.into_iter().map(|c| c.unwrap()).collect::<Vec<_>>());
            }
        }
        for (bi, list) in patches.into_iter().enumerate() {
            all_patches[bi].push(list);
        }
    }
    for band in &all_patches {
        sides_keep_apart(band)?;
    }

    // The caps, with their normals along the direction of travel, then the start's flipped to face out.
    let plane_at = |k: usize| {
        let f = match sections[k] {
            Section::Outline(_, f) => f,
            Section::Point(_) => unreachable!(),
        };
        if f.z.dot(direction(k)) >= 0.0 {
            f
        } else {
            Frame { origin: f.origin, x: f.x, y: -f.y, z: -f.z }
        }
    };
    if !start_loops.is_empty() {
        body.add_face(Surface::Plane(plane_at(0)), true, start_loops, tag(SLOT_START_CAP));
    }
    if !end_loops.is_empty() {
        body.add_face(Surface::Plane(plane_at(n - 1)), false, end_loops, tag(SLOT_END_CAP));
    }
    body.compute_shells();
    validate(&body).map_err(Error::InvalidBody)?;
    Ok(body)
}

/// Whether two segments in a plane properly cross.
fn segments_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let side = |p: Vec2, q: Vec2, r: Vec2| (q - p).cross(r - p);
    let (d1, d2, d3, d4) = (side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

/// Refuses sections where one passes through another, by where a section's outline crosses the plane of its
/// neighbour. Those planes can meet away from the outlines, as in a bend, and that's fine.
fn sections_keep_apart(wires: &[Vec<Vec<Chain>>], closed: bool) -> Result<()> {
    let outline = |chains: &[Chain]| -> Vec<Vec3> {
        let mut points = Vec::new();
        for c in chains {
            for h in &c.spans {
                for i in 0..8 {
                    let s = i as f64 / 8.0;
                    let mut work = h.clone();
                    for level in (1..work.len()).rev() {
                        for k in 0..level {
                            work[k] = work[k].lerp(work[k + 1], s);
                        }
                    }
                    points.push(work[0].point());
                }
            }
        }
        points
    };
    // A point has no outline to pass through.
    let outlines: Vec<Vec<Vec3>> = wires.iter().map(|w| w.first().map_or_else(Vec::new, |o| outline(o))).collect();
    let count = outlines.len();
    for k in 0..count {
        if outlines[k].is_empty() {
            continue;
        }
        let around = if closed { [(k + count - 1) % count, (k + 1) % count] } else { [k.wrapping_sub(1), k + 1] };
        for other in around {
            let Some(them) = outlines.get(other) else { continue };
            if them.is_empty() {
                continue;
            }
            let mine = &outlines[k];
            let z = area_vector(&wires[k][0]).normalized();
            let origin = wire_centroid(&wires[k][0]);
            let x = z.any_perpendicular();
            let y = z.cross(x);
            let flat = |p: Vec3| v2((p - origin).dot(x), (p - origin).dot(y));
            let polygon: Vec<Vec2> = mine.iter().map(|p| flat(*p)).collect();
            for i in 0..them.len() {
                let (p, q) = (them[i], them[(i + 1) % them.len()]);
                let (hp, hq) = ((p - origin).dot(z), (q - origin).dot(z));
                if hp * hq >= 0.0 {
                    continue;
                }
                let at = flat(p + (q - p) * (hp / (hp - hq)));
                // Inside the neighbour's outline, by counting crossings along a ray.
                let mut inside = false;
                for j in 0..polygon.len() {
                    let (a, b) = (polygon[j], polygon[(j + 1) % polygon.len()]);
                    if (a.y > at.y) != (b.y > at.y) && at.x < a.x + (at.y - a.y) / (b.y - a.y) * (b.x - a.x) {
                        inside = !inside;
                    }
                }
                if inside {
                    return Err(Error::InvalidProfile(format!("loft: sections {k} and {other} pass through each other")));
                }
            }
        }
    }
    Ok(())
}

/// Refuses sides that cross each other, which would be a solid passing through itself. Each stage of the loft
/// has to be an outline that doesn't cross itself, looking along the direction of travel there. This goes by
/// sampling, so a crossing smaller than the samples could get through.
fn sides_keep_apart(wires: &[Vec<Arc<Patch>>]) -> Result<()> {
    const STAGES: usize = 24;
    const ALONG: usize = 12;
    let stage = |v: f64| -> Vec<Vec<Vec3>> {
        wires
            .iter()
            .map(|wire| {
                let mut points = Vec::with_capacity(wire.len() * ALONG);
                for patch in wire {
                    let ((u0, u1), (v0, v1)) = patch.domain();
                    for i in 0..ALONG {
                        points.push(patch.eval(u0 + (u1 - u0) * i as f64 / ALONG as f64, v0 + (v1 - v0) * v));
                    }
                }
                points
            })
            .collect()
    };
    let middle = |outline: &[Vec<Vec3>]| outline[0].iter().fold(Vec3::ZERO, |m, p| m + *p) / outline[0].len() as f64;
    for s in 1..STAGES {
        let v = s as f64 / STAGES as f64;
        let here = stage(v);
        let along = middle(&stage(v + 0.5 / STAGES as f64)) - middle(&stage(v - 0.5 / STAGES as f64));
        if !(along.norm() > 1e-12) {
            continue;
        }
        let z = along.normalized();
        let x = z.any_perpendicular();
        let y = z.cross(x);
        let segments: Vec<(Vec2, Vec2, usize, usize)> = here
            .iter()
            .enumerate()
            .flat_map(|(w, outline)| {
                let flat: Vec<Vec2> = outline.iter().map(|p| v2(p.dot(x), p.dot(y))).collect();
                let n = flat.len();
                (0..n).map(move |i| (flat[i], flat[(i + 1) % n], w, i)).collect::<Vec<_>>()
            })
            .collect();
        for (i, a) in segments.iter().enumerate() {
            for b in &segments[i + 1..] {
                if segments_cross(a.0, a.1, b.0, b.1) {
                    return Err(Error::InvalidProfile(format!(
                        "loft: its sides cross each other about {:.0}% of the way along, so the solid would pass through itself",
                        v * 100.0
                    )));
                }
            }
        }
    }
    Ok(())
}

/// A frame for a patch that's flat, whose normal is the patch's.
fn flat_frame(patch: &Patch) -> Option<Frame> {
    if !patch.is_polynomial() {
        return None;
    }
    let ((u0, u1), (v0, v1)) = patch.domain();
    let (origin, du, dv) = patch.derivs((u0 + u1) / 2.0, (v0 + v1) / 2.0);
    let z = du.cross(dv).normalized();
    let (lo, hi) = patch.bounds();
    let size = lo.dist(hi);
    if patch.points().iter().any(|p| (*p - origin).dot(z).abs() > 1e-10 * size) {
        return None;
    }
    let x = du.normalized();
    Some(Frame { origin, x, y: z.cross(x), z })
}
