//! Rational B-spline curves, for sketch splines and for ellipses, which are exactly rational quadratics.
//!
//! The kernel's Booleans rest on a true distance: a value of `d` proves nothing lies within `d`. A curve
//! gives that here, because the distance from a point to a rational curve has an exact polynomial
//! form. Where the distance has a stationary point, `(A - p w) . (A' w - A w') = 0`, with `A / w` the
//! curve. Written in Bernstein form, that polynomial's coefficients are linear in `p`, and the number of
//! sign changes among them bounds the number of its roots. So every local minimum is found by
//! subdividing until each piece has at most one root, which is a proof, where Newton from a sample
//! would be a guess.
//!
//! A surface needs an inside and an outside too, which a curve with ends doesn't have. So a surface uses
//! a [`PlaneCurve`]: a piece whose tangents all lie within 90 degrees of one direction, carried on past
//! both ends along its end tangents. That makes it the graph of a function over that direction, which
//! divides the plane in two, and its signed distance is then 1-Lipschitz everywhere.

use crate::error::{Error, Result};
use crate::math::*;

/// The highest degree allowed, which keeps evaluation free of allocation.
pub const MAX_DEGREE: usize = 7;
/// Coefficients of the stationary polynomial at the highest degree: its degree is `3 p - 1`.
const MAX_COEFFICIENTS: usize = 3 * MAX_DEGREE;

/// The most a face's tangents may turn. Under 180 degrees, so the piece is a graph, with room to spare.
const MAX_TURN: f64 = 150.0 * PI / 180.0;
/// Tangents this close count as continuous.
const KINK: f64 = 1e-7;

/// A point in homogeneous form.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Hom {
    a: Vec3,
    w: f64,
}

impl Hom {
    const ZERO: Hom = Hom { a: Vec3::ZERO, w: 0.0 };

    fn lerp(self, o: Hom, s: f64) -> Hom {
        Hom { a: self.a + (o.a - self.a) * s, w: self.w + (o.w - self.w) * s }
    }

    fn of(p: Vec3, w: f64) -> Hom {
        Hom { a: p * w, w }
    }

    fn point(self) -> Vec3 {
        self.a / self.w
    }
}

/// One knot interval, as a rational Bezier curve.
#[derive(Clone, Debug)]
struct Span {
    t0: f64,
    t1: f64,
    h: Vec<Hom>,
    lo: Vec3,
    hi: Vec3,
    min_w: f64,
    start: Vec3,
    end: Vec3,
    /// `A' w - A w'` in Bernstein form, in terms of the span's own parameter from 0 to 1. Its direction
    /// is the tangent's and it's the speed times `w^2`.
    n: Vec<Vec3>,
    /// With `v`, the stationary polynomial for a point `p`: coefficient `i` is `c[i] - p . v[i]`.
    c: Vec<f64>,
    v: Vec<Vec3>,
}

/// A rational B-spline with clamped ends. `t` runs over the knot range. Outside it, the curve carries on
/// along its end tangents at its end speeds, so that a surface and its section curves agree past the ends.
#[derive(Clone, Debug)]
pub struct Spline {
    degree: usize,
    knots: Vec<f64>,
    points: Vec<Vec3>,
    weights: Vec<f64>,
    spans: Vec<Span>,
    speed_bound: f64,
    /// About the smallest radius of curvature, from samples: it sizes searches but proves nothing.
    min_radius: f64,
}

impl PartialEq for Spline {
    fn eq(&self, o: &Spline) -> bool {
        self.degree == o.degree && self.knots == o.knots && self.points == o.points && self.weights == o.weights
    }
}

fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k);
    let mut out = 1.0;
    for i in 0..k {
        out = out * (n - i) as f64 / (i + 1) as f64;
    }
    out
}

/// The product of two polynomials in Bernstein form, as a third.
fn product<T: Copy>(a: &[f64], b: &[T], zero: T, add: impl Fn(T, T, f64) -> T) -> Vec<T> {
    let (m, n) = (a.len() - 1, b.len() - 1);
    let mut out = vec![zero; m + n + 1];
    for i in 0..=m {
        for j in 0..=n {
            let k = binomial(m, i) * binomial(n, j) / binomial(m + n, i + j);
            out[i + j] = add(out[i + j], b[j], a[i] * k);
        }
    }
    out
}

fn product_vec(a: &[f64], b: &[Vec3]) -> Vec<Vec3> {
    product(a, b, Vec3::ZERO, |sum, x, k| sum + x * k)
}

/// The dot product of two vector polynomials, by component.
fn dot_product(a: &[Vec3], b: &[Vec3]) -> Vec<f64> {
    let (m, n) = (a.len() - 1, b.len() - 1);
    let mut out = vec![0.0; m + n + 1];
    for i in 0..=m {
        for j in 0..=n {
            let k = binomial(m, i) * binomial(n, j) / binomial(m + n, i + j);
            out[i + j] += a[i].dot(b[j]) * k;
        }
    }
    out
}

impl Span {
    fn new(t0: f64, t1: f64, h: Vec<Hom>) -> Span {
        let p = h.len() - 1;
        let (mut lo, mut hi) = (v3(f64::INFINITY, f64::INFINITY, f64::INFINITY), v3(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        let mut min_w = f64::INFINITY;
        for b in &h {
            let q = b.point();
            lo = v3(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
            hi = v3(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
            min_w = min_w.min(b.w);
        }
        let a: Vec<Vec3> = h.iter().map(|b| b.a).collect();
        let w: Vec<f64> = h.iter().map(|b| b.w).collect();
        let da: Vec<Vec3> = a.windows(2).map(|x| (x[1] - x[0]) * p as f64).collect();
        let dw: Vec<f64> = w.windows(2).map(|x| (x[1] - x[0]) * p as f64).collect();
        let (first, second) = (product_vec(&w, &da), product_vec(&dw, &a));
        let n: Vec<Vec3> = first.iter().zip(&second).map(|(x, y)| *x - *y).collect();
        let c = dot_product(&a, &n);
        let v = product_vec(&w, &n);
        Span { t0, t1, start: h[0].point(), end: h[p].point(), h, lo, hi, min_w, n, c, v }
    }

    /// Value, first and second derivative in homogeneous form, with respect to `t`.
    fn homogeneous(&self, s: f64) -> (Hom, Hom, Hom) {
        let p = self.h.len() - 1;
        let mut work = [Hom::ZERO; MAX_DEGREE + 1];
        work[..=p].copy_from_slice(&self.h);
        let (mut d1, mut d2) = (Hom::ZERO, Hom::ZERO);
        let scale = 1.0 / (self.t1 - self.t0);
        for level in (0..p).rev() {
            // `level + 1` points remain in the last round, so read the differences before folding.
            if level == 1 {
                let f = (p * (p - 1)) as f64 * scale * scale;
                d2 = Hom {
                    a: (work[2].a - work[1].a * 2.0 + work[0].a) * f,
                    w: (work[2].w - 2.0 * work[1].w + work[0].w) * f,
                };
            }
            if level == 0 {
                let f = p as f64 * scale;
                d1 = Hom { a: (work[1].a - work[0].a) * f, w: (work[1].w - work[0].w) * f };
            }
            for i in 0..=level {
                work[i] = work[i].lerp(work[i + 1], s);
            }
        }
        (work[0], d1, d2)
    }
}

/// Finds the local minima of the distance on one span: where the stationary polynomial goes from negative
/// to positive. `flat` is how small the polynomial may be across a stretch and still count as level, which
/// happens at the center of a circular arc.
fn isolate(g: &[f64], lo: f64, hi: f64, depth: u32, flat: f64, out: &mut dyn FnMut(f64)) {
    let mut changes = 0;
    let mut last = 0.0f64;
    let mut first = 0.0f64;
    let mut biggest = 0.0f64;
    for &x in g {
        biggest = biggest.max(x.abs());
        if x != 0.0 {
            if first == 0.0 {
                first = x;
            }
            if last != 0.0 && (x > 0.0) != (last > 0.0) {
                changes += 1;
            }
            last = x;
        }
    }
    if changes == 0 {
        return;
    }
    if biggest * (hi - lo) <= flat || depth == 0 {
        out((lo + hi) / 2.0);
        return;
    }
    if changes == 1 {
        if first < 0.0 {
            out(lo + (hi - lo) * single_root(g));
        }
        return;
    }
    let n = g.len();
    let (mut left, mut right) = ([0.0; MAX_COEFFICIENTS], [0.0; MAX_COEFFICIENTS]);
    let mut work = [0.0; MAX_COEFFICIENTS];
    work[..n].copy_from_slice(g);
    for level in 0..n {
        left[level] = work[0];
        right[n - 1 - level] = work[n - 1 - level];
        for i in 0..n - 1 - level {
            work[i] = (work[i] + work[i + 1]) / 2.0;
        }
    }
    let mid = (lo + hi) / 2.0;
    isolate(&left[..n], lo, mid, depth - 1, flat, out);
    isolate(&right[..n], mid, hi, depth - 1, flat, out);
}

fn bernstein_at(g: &[f64], s: f64) -> f64 {
    let n = g.len();
    let mut work = [0.0; MAX_COEFFICIENTS];
    work[..n].copy_from_slice(g);
    for level in (1..n).rev() {
        for i in 0..level {
            work[i] += (work[i + 1] - work[i]) * s;
        }
    }
    work[0]
}

/// The one root of a polynomial whose coefficients change sign once, by the Illinois method.
fn single_root(g: &[f64]) -> f64 {
    let (mut a, mut b) = (0.0, 1.0);
    let (mut fa, mut fb) = (g[0], g[g.len() - 1]);
    if fa == 0.0 {
        return a;
    }
    if fb == 0.0 {
        return b;
    }
    for _ in 0..64 {
        let s = (a * fb - b * fa) / (fb - fa);
        let s = if s > a && s < b { s } else { (a + b) / 2.0 };
        let fs = bernstein_at(g, s);
        if fs == 0.0 {
            return s;
        }
        if (fs > 0.0) == (fb > 0.0) {
            b = s;
            fb = fs;
            fa *= 0.5;
        } else {
            a = s;
            fa = fs;
            fb *= 0.5;
        }
        if b - a < 1e-15 {
            break;
        }
    }
    (a + b) / 2.0
}

fn box_distance(p: Vec3, lo: Vec3, hi: Vec3) -> f64 {
    let d = v3((lo.x - p.x).max(p.x - hi.x).max(0.0), (lo.y - p.y).max(p.y - hi.y).max(0.0), (lo.z - p.z).max(p.z - hi.z).max(0.0));
    d.norm()
}

impl Spline {
    /// A spline from its control data. Ends must be clamped: the first and last knots repeated
    /// `degree + 1` times, so that the curve starts and ends on its end control points.
    pub fn new(degree: usize, knots: Vec<f64>, points: Vec<Vec3>, weights: Vec<f64>) -> Result<Spline> {
        let bad = |why: String| Err(Error::InvalidProfile(format!("spline: {why}")));
        if degree < 1 || degree > MAX_DEGREE {
            return bad(format!("degree {degree} isn't between 1 and {MAX_DEGREE}"));
        }
        if points.len() < degree + 1 {
            return bad(format!("degree {degree} needs at least {} control points, not {}", degree + 1, points.len()));
        }
        if weights.len() != points.len() {
            return bad(format!("{} weights for {} control points", weights.len(), points.len()));
        }
        if knots.len() != points.len() + degree + 1 {
            return bad(format!("{} knots, when {} control points of degree {degree} need {}", knots.len(), points.len(), points.len() + degree + 1));
        }
        if !points.iter().all(|p| p.is_finite() && p.x.abs().max(p.y.abs()).max(p.z.abs()) <= MAX_COORD) {
            return bad(format!("a control point is beyond the {MAX_COORD} mm this kernel works within"));
        }
        if !weights.iter().all(|w| w.is_finite() && *w > 0.0) {
            return bad("every weight must be positive".to_string());
        }
        if !knots.iter().all(|k| k.is_finite()) || knots.windows(2).any(|w| w[1] < w[0]) {
            return bad("knots must be finite and never decrease".to_string());
        }
        let (u0, u1) = (knots[0], knots[knots.len() - 1]);
        if !(u1 > u0) || knots[degree] != u0 || knots[knots.len() - 1 - degree] != u1 {
            return bad(format!("the first and last knots must each repeat {} times", degree + 1));
        }
        let mut run = 1;
        for w in knots[degree..=knots.len() - 1 - degree].windows(2) {
            run = if w[0] == w[1] { run + 1 } else { 1 };
            if run > degree && w[1] != u1 && w[0] != u0 {
                return bad(format!("an interior knot repeats more than {degree} times"));
            }
        }
        let mut out = Spline { degree, knots, points, weights, spans: Vec::new(), speed_bound: 0.0, min_radius: f64::INFINITY };
        out.build_spans();
        if !(out.speed_bound > 0.0) {
            return bad("all its control points are in one place".to_string());
        }
        Ok(out)
    }

    /// A clamped spline with evenly spaced knots, in units of its control polygon's length.
    pub fn uniform(degree: usize, points: Vec<Vec3>) -> Result<Spline> {
        let n = points.len();
        if degree < 1 || n < degree + 1 {
            return Err(Error::InvalidProfile(format!("spline: degree {degree} needs at least {} control points, not {n}", degree + 1)));
        }
        let total: f64 = points.windows(2).map(|w| w[0].dist(w[1])).sum();
        let intervals = n - degree;
        let step = total.max(1e-12) / intervals as f64;
        let mut knots = vec![0.0; degree + 1];
        for i in 1..intervals {
            knots.push(step * i as f64);
        }
        knots.extend(vec![step * intervals as f64; degree + 1]);
        let weights = vec![1.0; n];
        Spline::new(degree, knots, points, weights)
    }

    /// The smooth curve through these points: cubic if there are at least four, with `t` measured by the
    /// chords between them, so that it's close to arc length.
    pub fn through(points: &[Vec3]) -> Result<Spline> {
        let n = points.len();
        if n < 2 {
            return Err(Error::InvalidProfile("spline: a curve through points needs at least two of them".to_string()));
        }
        let degree = 3.min(n - 1);
        let mut u = vec![0.0; n];
        for i in 1..n {
            let chord = points[i - 1].dist(points[i]);
            if !(chord > MIN_FEATURE) {
                return Err(Error::InvalidProfile(format!("spline: through-points {} and {i} are in one place", i - 1)));
            }
            u[i] = u[i - 1] + chord;
        }
        // Knots by averaging, which keeps the system well behaved for any spacing.
        let mut knots = vec![0.0; degree + 1];
        for j in 1..n - degree {
            let sum: f64 = u[j..j + degree].iter().sum();
            knots.push(sum / degree as f64);
        }
        knots.extend(vec![u[n - 1]; degree + 1]);
        // The collocation matrix is banded and totally positive, so elimination needs no pivoting.
        let band = degree;
        let mut rows = vec![vec![0.0; 2 * band + 1]; n];
        for (k, row) in rows.iter_mut().enumerate() {
            let span = find_knot_span(&knots, degree, u[k], n);
            let basis = basis_functions(&knots, degree, span, u[k]);
            for (b, value) in basis.iter().enumerate() {
                let column = span - degree + b;
                row[column + band - k] = *value;
            }
        }
        let mut rhs: Vec<Vec3> = points.to_vec();
        for k in 0..n {
            let pivot = rows[k][band];
            if pivot.abs() < 1e-14 {
                return Err(Error::InvalidProfile("spline: these points can't be interpolated".to_string()));
            }
            for below in k + 1..(k + band + 1).min(n) {
                let offset = below - k;
                let f = rows[below][band - offset] / pivot;
                if f == 0.0 {
                    continue;
                }
                for c in 0..=band {
                    let value = rows[k][band + c];
                    rows[below][band - offset + c] -= f * value;
                }
                let r = rhs[k];
                rhs[below] -= r * f;
            }
        }
        let mut solved = vec![Vec3::ZERO; n];
        for k in (0..n).rev() {
            let mut sum = rhs[k];
            for c in 1..=band {
                if k + c < n {
                    sum -= solved[k + c] * rows[k][band + c];
                }
            }
            solved[k] = sum / rows[k][band];
        }
        Spline::new(degree, knots, solved, vec![1.0; n])
    }

    /// An exact elliptical arc from eccentric angle `a0` to `a1`, in the plane through `center` spanned
    /// by the unit vectors `x` and `y`. It's counter-clockwise in that plane if `a1 > a0`. Each piece of up
    /// to 90 degrees is one rational quadratic, and `t` measures roughly arc length.
    pub fn ellipse_arc(center: Vec3, x: Vec3, y: Vec3, major: f64, minor: f64, a0: f64, a1: f64) -> Result<Spline> {
        let sweep = a1 - a0;
        if !(major > 0.0 && minor > 0.0 && sweep.abs() > 0.0 && sweep.abs() <= TAU + 1e-9) {
            return Err(Error::InvalidProfile(format!("spline: bad arc, radii {major} and {minor} from {a0} to {a1}")));
        }
        let pieces = ((sweep.abs() / (PI / 2.0) - 1e-9).ceil() as usize).max(1);
        let step = sweep / pieces as f64;
        let at = |px: f64, py: f64| center + x * (major * px) + y * (minor * py);
        let scale = major.max(minor) * step.abs();
        let mut points = vec![at(a0.cos(), a0.sin())];
        let mut weights = vec![1.0];
        let mut knots = vec![0.0; 3];
        let half = step / 2.0;
        let (w, reach) = (half.cos(), 1.0 / half.cos());
        for i in 0..pieces {
            let mid = a0 + step * (i as f64 + 0.5);
            let end = a0 + step * (i as f64 + 1.0);
            points.push(at(reach * mid.cos(), reach * mid.sin()));
            weights.push(w);
            points.push(at(end.cos(), end.sin()));
            weights.push(1.0);
            let k = scale * (i as f64 + 1.0);
            knots.push(k);
            knots.push(k);
        }
        knots.push(scale * pieces as f64);
        Spline::new(2, knots, points, weights)
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    pub fn points(&self) -> &[Vec3] {
        &self.points
    }

    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Whether every weight is the same, which makes it an ordinary polynomial spline.
    pub fn is_polynomial(&self) -> bool {
        self.weights.iter().all(|w| *w == self.weights[0])
    }

    pub fn domain(&self) -> (f64, f64) {
        (self.knots[0], self.knots[self.knots.len() - 1])
    }

    pub fn start(&self) -> Vec3 {
        self.points[0]
    }

    pub fn end(&self) -> Vec3 {
        self.points[self.points.len() - 1]
    }

    /// An upper bound on the speed, from the control points of the derivative.
    pub fn speed_bound(&self) -> f64 {
        self.speed_bound
    }

    /// The parameters where the curve is less smooth than a polynomial: the knots strictly inside
    /// this range.
    pub fn breaks_between(&self, t0: f64, t1: f64) -> Vec<f64> {
        self.spans.iter().map(|s| s.t0).filter(|&k| k > t0 && k < t1).collect()
    }

    /// Replaces the spans, from the control data, as control points in Bezier form by blossoming.
    fn build_spans(&mut self) {
        let p = self.degree;
        let hom: Vec<Hom> = self.points.iter().zip(&self.weights).map(|(q, w)| Hom::of(*q, *w)).collect();
        let mut spans = Vec::new();
        for k in p..self.points.len() {
            let (t0, t1) = (self.knots[k], self.knots[k + 1]);
            if t1 <= t0 {
                continue;
            }
            let mut h = Vec::with_capacity(p + 1);
            for j in 0..=p {
                let args: Vec<f64> = (0..p).map(|i| if i < p - j { t0 } else { t1 }).collect();
                h.push(blossom(&hom, &self.knots, p, k, &args));
            }
            spans.push(Span::new(t0, t1, h));
        }
        let mut bound: f64 = 0.0;
        for s in &spans {
            let biggest = s.n.iter().fold(0.0f64, |m, v| m.max(v.norm()));
            bound = bound.max(biggest / (s.min_w * s.min_w) / (s.t1 - s.t0));
        }
        self.spans = spans;
        self.speed_bound = bound;
        let mut smallest = f64::INFINITY;
        for s in &self.spans {
            for i in 0..=16 {
                let (_, d1, d2) = self.derivs(s.t0 + (s.t1 - s.t0) * i as f64 / 16.0);
                let (speed, turn) = (d1.norm(), d1.cross(d2).norm());
                if turn > 0.0 && speed > 0.0 {
                    smallest = smallest.min(speed * speed * speed / turn);
                }
            }
        }
        self.min_radius = smallest;
    }

    /// About the smallest radius of curvature, from samples. It sizes searches but proves nothing.
    pub fn min_radius(&self) -> f64 {
        self.min_radius
    }

    fn span_at(&self, t: f64) -> &Span {
        let at = self.spans.partition_point(|s| s.t1 <= t);
        &self.spans[at.min(self.spans.len() - 1)]
    }

    /// The point, first and second derivative. Past the ends, a straight line on from the end tangent.
    pub fn derivs(&self, t: f64) -> (Vec3, Vec3, Vec3) {
        let (u0, u1) = self.domain();
        if t < u0 || t > u1 {
            let end = if t < u0 { u0 } else { u1 };
            let (q, d, _) = self.derivs(end);
            return (q + d * (t - end), d, Vec3::ZERO);
        }
        let span = self.span_at(t);
        let (b, d1, d2) = span.homogeneous((t - span.t0) / (span.t1 - span.t0));
        let c = b.point();
        let c1 = (d1.a - c * d1.w) / b.w;
        let c2 = (d2.a - c1 * (2.0 * d1.w) - c * d2.w) / b.w;
        (c, c1, c2)
    }

    pub fn eval(&self, t: f64) -> Vec3 {
        let (u0, u1) = self.domain();
        if t < u0 || t > u1 {
            return self.derivs(t).0;
        }
        let span = self.span_at(t);
        let s = (t - span.t0) / (span.t1 - span.t0);
        let p = span.h.len() - 1;
        let mut work = [Hom::ZERO; MAX_DEGREE + 1];
        work[..=p].copy_from_slice(&span.h);
        for level in (0..p).rev() {
            for i in 0..=level {
                work[i] = work[i].lerp(work[i + 1], s);
            }
        }
        work[0].point()
    }

    pub fn derivative(&self, t: f64) -> Vec3 {
        self.derivs(t).1
    }

    /// The closest point to `p` for `t` in the given range, which may run past the ends or be
    /// infinite. Returns its parameter and distance. Every local minimum is examined, so this is the
    /// true closest point, not merely a nearby one.
    pub fn closest(&self, p: Vec3, t0: f64, t1: f64) -> (f64, f64) {
        let (u0, u1) = self.domain();
        let mut best = (f64::NAN, f64::INFINITY);
        let consider = |t: f64, best: &mut (f64, f64)| {
            let d = self.eval(t).dist(p);
            if d < best.1 {
                *best = (t, d);
            }
        };
        if t0.is_finite() {
            consider(t0, &mut best);
        }
        if t1.is_finite() {
            consider(t1, &mut best);
        }
        if t0 < u0 {
            let d = self.spans[0].n[0] / (self.spans[0].h[0].w * self.spans[0].h[0].w * (self.spans[0].t1 - self.spans[0].t0));
            let along = u0 + (p - self.start()).dot(d) / d.norm2();
            if along < u0 {
                consider(along.max(t0), &mut best);
            }
        }
        if t1 > u1 {
            let last = &self.spans[self.spans.len() - 1];
            let d = last.n[last.n.len() - 1] / (last.h[last.h.len() - 1].w * last.h[last.h.len() - 1].w * (last.t1 - last.t0));
            let along = u1 + (p - self.end()).dot(d) / d.norm2();
            if along > u1 {
                consider(along.min(t1), &mut best);
            }
        }
        for s in &self.spans {
            if s.t0 >= t0 && s.t0 <= t1 {
                let d = s.start.dist(p);
                if d < best.1 {
                    best = (s.t0, d);
                }
            }
            if s.t1 >= t0 && s.t1 <= t1 {
                let d = s.end.dist(p);
                if d < best.1 {
                    best = (s.t1, d);
                }
            }
        }
        let mut g = [0.0; MAX_COEFFICIENTS];
        for s in &self.spans {
            if s.t1 < t0 || s.t0 > t1 || box_distance(p, s.lo, s.hi) >= best.1 {
                continue;
            }
            let n = s.c.len();
            for i in 0..n {
                g[i] = s.c[i] - p.dot(s.v[i]);
            }
            // Below this size the distance can't change by a useful amount across the stretch.
            let flat = 1e-14 * s.min_w * s.min_w * s.min_w * best.1.max(1e-9);
            let mut found = |x: f64| {
                let t = s.t0 + (s.t1 - s.t0) * x;
                if t >= t0 && t <= t1 {
                    let d = self.eval(t).dist(p);
                    if d < best.1 {
                        best = (t, d);
                    }
                }
            };
            isolate(&g[..n], 0.0, 1.0, 48, flat, &mut found);
        }
        best
    }

    /// The same curve with its direction reversed, over the same parameter range.
    pub fn reversed(&self) -> Spline {
        let (u0, u1) = self.domain();
        let knots = self.knots.iter().rev().map(|k| u0 + u1 - k).collect();
        let points = self.points.iter().rev().copied().collect();
        let weights = self.weights.iter().rev().copied().collect();
        let mut out = Spline { degree: self.degree, knots, points, weights, spans: Vec::new(), speed_bound: 0.0, min_radius: f64::INFINITY };
        out.build_spans();
        out
    }

    /// The image under an affine map, which a rational spline follows exactly. `t` still means the
    /// matching point, including past the ends.
    pub fn mapped(&self, f: impl Fn(Vec3) -> Vec3) -> Spline {
        let points = self.points.iter().map(|p| f(*p)).collect();
        let mut out = Spline { degree: self.degree, knots: self.knots.clone(), points, weights: self.weights.clone(), spans: Vec::new(), speed_bound: 0.0, min_radius: f64::INFINITY };
        out.build_spans();
        out
    }

    /// The part from `a` to `b`, keeping its parameters, as a spline of its own.
    pub fn sub(&self, a: f64, b: f64) -> Result<Spline> {
        let (u0, u1) = self.domain();
        if !(a >= u0 && b <= u1 && b > a) {
            return Err(Error::InvalidProfile(format!("spline: can't take the part from {a} to {b} of a curve over {u0} to {u1}")));
        }
        let p = self.degree;
        let hom: Vec<Hom> = self.points.iter().zip(&self.weights).map(|(q, w)| Hom::of(*q, *w)).collect();
        let mut knots = vec![a; p + 1];
        knots.extend(self.knots.iter().copied().filter(|&k| k > a && k < b));
        knots.extend(vec![b; p + 1]);
        let count = knots.len() - p - 1;
        let mut points = Vec::with_capacity(count);
        let mut weights = Vec::with_capacity(count);
        for i in 0..count {
            // The Oslo rule: a new control point is the blossom of the next `degree` new knots, on the
            // old span holding this one.
            let tau = knots[i].min(self.knots[self.knots.len() - 1 - p - 1].max(knots[i]));
            let mut k = find_knot_span(&self.knots, p, tau, self.points.len());
            if tau >= u1 {
                k = self.points.len() - 1;
                while self.knots[k + 1] <= self.knots[k] {
                    k -= 1;
                }
            }
            let h = blossom(&hom, &self.knots, p, k, &knots[i + 1..=i + p]);
            points.push(h.point());
            weights.push(h.w);
        }
        Spline::new(p, knots, points, weights)
    }

    /// Length over a range, numerically.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        let rule = gauss_24();
        let mut breaks = vec![t0];
        breaks.extend(self.breaks_between(t0, t1));
        breaks.push(t1);
        let mut sum = 0.0;
        for w in breaks.windows(2) {
            let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
            for &(x, weight) in rule {
                sum += self.derivative(mid + half * x).norm() * weight * half;
            }
        }
        sum
    }

    /// Cuts the curve where one face can't cover it: at sharp corners, and wherever its tangents have
    /// turned too far to stay a graph. Straight stretches come out as lines, since a plane is the
    /// exact surface for them. Refuses a curve that stops dead, since it has no tangent there.
    pub fn face_pieces(&self) -> Result<Vec<FacePiece>> {
        self.face_ranges()?
            .into_iter()
            .map(|(a, b, line)| {
                if line {
                    Ok(FacePiece::Line(self.eval(a), self.eval(b)))
                } else {
                    Ok(FacePiece::Curve(self.sub(a, b)?))
                }
            })
            .collect()
    }

    /// The parameter ranges of [`Spline::face_pieces`], with whether each is straight.
    pub fn face_ranges(&self) -> Result<Vec<(f64, f64, bool)>> {
        // Stretches over which the tangent's direction is certainly within a small cone.
        struct Stretch {
            t0: f64,
            t1: f64,
            base: f64,
            low: f64,
            high: f64,
            first: f64,
            last: f64,
        }
        fn split_vectors(n: &[Vec3]) -> (Vec<Vec3>, Vec<Vec3>) {
            let mut work = n.to_vec();
            let m = n.len();
            let (mut left, mut right) = (vec![Vec3::ZERO; m], vec![Vec3::ZERO; m]);
            for level in 0..m {
                left[level] = work[0];
                right[m - 1 - level] = work[m - 1 - level];
                for i in 0..m - 1 - level {
                    work[i] = (work[i] + work[i + 1]) * 0.5;
                }
            }
            (left, right)
        }
        fn walk(n: &[Vec3], t0: f64, t1: f64, depth: u32, out: &mut Vec<Stretch>) -> Result<()> {
            let biggest = n.iter().fold(0.0f64, |m, v| m.max(v.norm()));
            let stops = |t: f64| Err(Error::InvalidProfile(format!("spline: it stops dead at t = {t:.6}, so it has no direction there. Move control points apart")));
            if n[0].norm() <= 1e-12 * biggest || biggest == 0.0 {
                return stops(t0);
            }
            if n[n.len() - 1].norm() <= 1e-12 * biggest {
                return stops(t1);
            }
            let base = n[0].y.atan2(n[0].x);
            let (mut low, mut high) = (0.0f64, 0.0f64);
            for v in n {
                if v.norm() > 1e-12 * biggest {
                    let a = wrap_pi(v.y.atan2(v.x) - base);
                    low = low.min(a);
                    high = high.max(a);
                }
            }
            if high - low > 2.0 && depth > 0 {
                let (left, right) = split_vectors(n);
                let mid = (t0 + t1) / 2.0;
                walk(&left, t0, mid, depth - 1, out)?;
                return walk(&right, mid, t1, depth - 1, out);
            }
            if high - low > 2.0 {
                return Err(Error::InvalidProfile(format!("spline: it turns back on itself sharply near t = {:.6}", (t0 + t1) / 2.0)));
            }
            let last = wrap_pi(n[n.len() - 1].y.atan2(n[n.len() - 1].x) - base);
            out.push(Stretch { t0, t1, base, low, high, first: 0.0, last });
            Ok(())
        }
        let mut stretches = Vec::new();
        for s in &self.spans {
            walk(&s.n, s.t0, s.t1, 14, &mut stretches)?;
        }
        // Join stretches into pieces for as long as they stay smooth and don't turn too far.
        let mut pieces: Vec<(f64, f64, bool)> = Vec::new();
        let straight = |s: &Stretch| s.high - s.low < 1e-10;
        let mut start = stretches[0].t0;
        let mut base = stretches[0].base;
        let (mut low, mut high) = (stretches[0].low, stretches[0].high);
        let mut is_straight = straight(&stretches[0]);
        for pair in 1..stretches.len() {
            let (before, now) = (&stretches[pair - 1], &stretches[pair]);
            let kink = wrap_pi(now.base + now.first - (before.base + before.last)).abs() > KINK;
            let shift = wrap_pi(now.base - base);
            let (next_low, next_high) = (low.min(shift + now.low), high.max(shift + now.high));
            if kink || next_high - next_low > MAX_TURN || straight(now) != is_straight {
                pieces.push((start, now.t0, is_straight));
                start = now.t0;
                base = now.base;
                low = now.low;
                high = now.high;
                is_straight = straight(now);
            } else {
                low = next_low;
                high = next_high;
            }
        }
        pieces.push((start, stretches[stretches.len() - 1].t1, is_straight));
        Ok(pieces)
    }

    /// Points along the curve, for checking whether an outline crosses itself. They get closer and closer
    /// toward both ends, since a curve can leave a corner on the wrong side of its neighbour and cross back
    /// by a tiny amount, and the neighbour has to be tested right up to the corner.
    pub fn graded_polyline(&self, per_span: usize) -> Vec<Vec3> {
        let (u0, u1) = self.domain();
        let mut ts: Vec<f64> = Vec::with_capacity(self.spans.len() * per_span + 64);
        for s in &self.spans {
            for i in 0..per_span {
                ts.push(s.t0 + (s.t1 - s.t0) * i as f64 / per_span as f64);
            }
        }
        ts.push(u1);
        let first = self.spans[0].t1 - self.spans[0].t0;
        let last = self.spans[self.spans.len() - 1].t1 - self.spans[self.spans.len() - 1].t0;
        let mut step = 1.0 / per_span as f64;
        for _ in 0..24 {
            step *= 0.5;
            ts.push(u0 + first * step);
            ts.push(u1 - last * step);
        }
        ts.sort_by(f64::total_cmp);
        ts.iter().map(|&t| self.eval(t)).collect()
    }

    /// Evenly spaced points along the curve, by knot interval.
    pub fn polyline(&self, per_span: usize) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(self.spans.len() * per_span + 1);
        for s in &self.spans {
            for i in 0..per_span {
                out.push(self.eval(s.t0 + (s.t1 - s.t0) * i as f64 / per_span as f64));
            }
        }
        out.push(self.end());
        out
    }
}

/// What one face's worth of curve turned out to be.
#[derive(Clone, Debug)]
pub enum FacePiece {
    Line(Vec3, Vec3),
    Curve(Spline),
}

fn find_knot_span(knots: &[f64], degree: usize, t: f64, count: usize) -> usize {
    if t >= knots[count] {
        // At the very end: the last interval with any length.
        let mut k = count - 1;
        while k > degree && knots[k + 1] <= knots[k] {
            k -= 1;
        }
        return k;
    }
    let mut k = knots.partition_point(|&u| u <= t) - 1;
    k = k.clamp(degree, count - 1);
    while k > degree && knots[k + 1] <= knots[k] {
        k -= 1;
    }
    k
}

fn basis_functions(knots: &[f64], degree: usize, span: usize, t: f64) -> Vec<f64> {
    let mut out = vec![0.0; degree + 1];
    out[0] = 1.0;
    let (mut left, mut right) = (vec![0.0; degree + 1], vec![0.0; degree + 1]);
    for j in 1..=degree {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = out[r] / (right[r + 1] + left[j - r]);
            out[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        out[j] = saved;
    }
    out
}

/// The polar form of the polynomial on knot span `k`, at these arguments. De Boor's algorithm, using a
/// different argument at each level.
fn blossom(hom: &[Hom], knots: &[f64], p: usize, k: usize, args: &[f64]) -> Hom {
    let mut work = [Hom::ZERO; MAX_DEGREE + 1];
    work[..=p].copy_from_slice(&hom[k - p..=k]);
    for r in 1..=p {
        for j in (r..=p).rev() {
            let lo = knots[k - p + j];
            let hi = knots[k + 1 + j - r];
            let alpha = if hi > lo { (args[r - 1] - lo) / (hi - lo) } else { 0.0 };
            work[j] = work[j - 1].lerp(work[j], alpha);
        }
    }
    work[p]
}

/// The nearest point on a curve in the plane, and which side the query point is on.
#[derive(Clone, Copy, Debug)]
pub struct Foot {
    pub t: f64,
    pub at: Vec2,
    /// Unit direction of travel there.
    pub tangent: Vec2,
    /// Distance to the curve, positive on the right of travel, which is the side a surface's normal
    /// points to.
    pub signed: f64,
}

/// One face's worth of curve in the XY plane, carried on along its end tangents. Its tangents all lie
/// within 90 degrees of one direction, so the plane has two clear sides of it. See [`Spline::face_pieces`].
#[derive(Clone, Debug)]
pub struct PlaneCurve {
    pub spline: Spline,
    /// No speed along the curve exceeds this.
    pub speed: f64,
    /// The largest distance of the curve from the Y axis, for a surface of revolution.
    pub reach: f64,
}

impl PartialEq for PlaneCurve {
    fn eq(&self, o: &PlaneCurve) -> bool {
        self.spline == o.spline
    }
}

impl PlaneCurve {
    pub fn new(spline: Spline) -> PlaneCurve {
        let mut reach = 0.0f64;
        for s in &spline.spans {
            for b in &s.h {
                reach = reach.max(b.point().x.abs());
            }
        }
        PlaneCurve { speed: spline.speed_bound().max(1e-12), reach, spline }
    }

    pub fn min_radius(&self) -> f64 {
        self.spline.min_radius()
    }

    /// Sample points along the curve itself, without its extensions.
    pub fn samples(&self) -> Vec<Vec2> {
        self.spline.polyline(8).iter().map(|p| v2(p.x, p.y)).collect()
    }

    pub fn eval(&self, t: f64) -> Vec2 {
        let p = self.spline.eval(t);
        v2(p.x, p.y)
    }

    pub fn derivative(&self, t: f64) -> Vec2 {
        let d = self.spline.derivative(t);
        v2(d.x, d.y)
    }

    /// The nearest point, over the curve and both its extensions.
    pub fn foot(&self, q: Vec2) -> Foot {
        let (t, _) = self.spline.closest(v3(q.x, q.y, 0.0), f64::NEG_INFINITY, f64::INFINITY);
        let (p, d, _) = self.spline.derivs(t);
        let (at, tangent) = (v2(p.x, p.y), v2(d.x, d.y).normalized());
        let w = q - at;
        let distance = w.norm();
        // At a true nearest point the offset is square to the tangent, so its side is clear unless it's
        // too short to have a direction, when the value hardly matters.
        let side = w.dot(tangent.right());
        Foot { t, at, tangent, signed: if side < 0.0 { -distance } else { distance } }
    }

    /// Signed distance: positive on the right of travel.
    pub fn signed_distance(&self, q: Vec2) -> f64 {
        self.foot(q).signed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wavy() -> Spline {
        let pts = [(0.0, 0.0), (4.0, 7.0), (9.0, -3.0), (14.0, 6.0), (20.0, 1.0), (25.0, 4.0)];
        Spline::uniform(3, pts.iter().map(|p| v3(p.0, p.1, 0.0)).collect()).unwrap()
    }

    fn rational() -> Spline {
        let pts = [(0.0, 0.0), (4.0, 7.0), (9.0, -3.0), (14.0, 6.0), (20.0, 1.0), (25.0, 4.0)];
        let knots = vec![0.0, 0.0, 0.0, 0.0, 2.0, 3.5, 7.0, 7.0, 7.0, 7.0];
        Spline::new(3, knots, pts.iter().map(|p| v3(p.0, p.1, 0.0)).collect(), vec![1.0, 2.5, 0.7, 3.0, 0.4, 1.0]).unwrap()
    }

    /// De Boor's algorithm directly on the control data, in homogeneous form, written apart from the
    /// spans so that it checks them.
    fn de_boor(s: &Spline, t: f64) -> Vec3 {
        let p = s.degree;
        let k = find_knot_span(&s.knots, p, t, s.points.len());
        let mut d: Vec<Hom> = (k - p..=k).map(|i| Hom::of(s.points[i], s.weights[i])).collect();
        for r in 1..=p {
            for j in (r..=p).rev() {
                let lo = s.knots[k - p + j];
                let hi = s.knots[k + 1 + j - r];
                let alpha = (t - lo) / (hi - lo);
                d[j] = d[j - 1].lerp(d[j], alpha);
            }
        }
        d[p].point()
    }

    #[test]
    fn spans_evaluate_the_same_as_de_boor() {
        for s in [wavy(), rational()] {
            let (u0, u1) = s.domain();
            for i in 0..=200 {
                let t = u0 + (u1 - u0) * i as f64 / 200.0;
                assert!(s.eval(t).dist(de_boor(&s, t)) < 1e-11, "t={t}");
            }
        }
    }

    #[test]
    fn derivatives_match_differences_and_the_speed_bound_holds() {
        for s in [wavy(), rational()] {
            let (u0, u1) = s.domain();
            for i in 1..200 {
                let t = u0 + (u1 - u0) * (i as f64 + 0.37) / 200.0;
                let h = 1e-5;
                let (_, d1, d2) = s.derivs(t);
                let fd = (s.eval(t + h) - s.eval(t - h)) / (2.0 * h);
                let fd2 = (s.derivs(t + h).1 - s.derivs(t - h).1) / (2.0 * h);
                if (t - h).max(u0) > u0 && s.breaks_between(t - h, t + h).is_empty() {
                    assert!((fd - d1).norm() < 1e-6, "first at {t}: {fd:?} vs {d1:?}");
                    assert!((fd2 - d2).norm() < 1e-4, "second at {t}: {fd2:?} vs {d2:?}");
                }
                assert!(d1.norm() <= s.speed_bound() * (1.0 + 1e-12), "{} > {}", d1.norm(), s.speed_bound());
            }
        }
    }

    #[test]
    fn past_its_ends_it_carries_on_straight() {
        let s = rational();
        let (u0, u1) = s.domain();
        let (p, d, _) = s.derivs(u1);
        assert!(s.eval(u1 + 2.0).dist(p + d * 2.0) < 1e-12);
        let (p, d, _) = s.derivs(u0);
        assert!(s.eval(u0 - 3.0).dist(p - d * 3.0) < 1e-12);
    }

    #[test]
    fn the_closest_point_beats_dense_sampling() {
        let mut rng = Rng::new(5);
        for s in [wavy(), rational()] {
            let (u0, u1) = s.domain();
            let samples: Vec<Vec3> = (0..=4000).map(|i| s.eval(u0 + (u1 - u0) * i as f64 / 4000.0)).collect();
            for _ in 0..400 {
                let p = v3(rng.range(-10.0, 35.0), rng.range(-15.0, 15.0), 0.0);
                let (t, d) = s.closest(p, u0, u1);
                let sampled = samples.iter().map(|q| q.dist(p)).fold(f64::INFINITY, f64::min);
                assert!(d <= sampled + 1e-9, "{p:?}: {d} vs sampled {sampled}");
                assert!(d >= sampled - 0.02, "{p:?}: {d} vs sampled {sampled}");
                assert!((s.eval(t).dist(p) - d).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn the_center_of_a_circular_arc_is_handled() {
        let a = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 5.0, 5.0, 0.1, 3.0).unwrap();
        let (u0, u1) = a.domain();
        let (_, d) = a.closest(Vec3::ZERO, u0, u1);
        assert!((d - 5.0).abs() < 1e-12, "{d}");
        // Just off center, and a little off it the other way.
        for &(x, y) in &[(1e-9, 0.0), (0.0, -1e-7), (1e-4, 1e-4)] {
            let p = v3(x, y, 0.0);
            let (_, d) = a.closest(p, u0, u1);
            let sampled = (0..=20000).map(|i| a.eval(u0 + (u1 - u0) * i as f64 / 20000.0).dist(p)).fold(f64::INFINITY, f64::min);
            assert!(d <= sampled + 1e-12 && d >= sampled - 1e-6, "{d} vs {sampled}");
        }
    }

    #[test]
    fn an_ellipse_arc_is_exact() {
        let (x, y) = (v3(0.6, 0.8, 0.0), v3(-0.8, 0.6, 0.0));
        let e = Spline::ellipse_arc(v3(1.0, 2.0, 0.0), x, y, 7.0, 3.0, 0.3, 5.5).unwrap();
        let (u0, u1) = e.domain();
        for i in 0..=500 {
            let l = e.eval(u0 + (u1 - u0) * i as f64 / 500.0) - v3(1.0, 2.0, 0.0);
            let (a, b) = (l.dot(x) / 7.0, l.dot(y) / 3.0);
            assert!((a * a + b * b - 1.0).abs() < 1e-13, "{a} {b}");
        }
        assert!(e.eval(u0).dist(v3(1.0, 2.0, 0.0) + x * (7.0 * 0.3f64.cos()) + y * (3.0 * 0.3f64.sin())) < 1e-12);
        assert!(e.eval(u1).dist(v3(1.0, 2.0, 0.0) + x * (7.0 * 5.5f64.cos()) + y * (3.0 * 5.5f64.sin())) < 1e-12);
        // Clockwise, too.
        let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 2.0, 1.0, 1.0, -1.0).unwrap();
        assert!(c.eval(c.domain().0).dist(v3(2.0 * 1f64.cos(), 1f64.sin(), 0.0)) < 1e-12);
        assert!(c.derivative(c.domain().0).y < 0.0);
    }

    #[test]
    fn a_circle_has_the_length_of_a_circle() {
        let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 3.0, 3.0, 0.0, TAU).unwrap();
        let (u0, u1) = c.domain();
        assert!((c.length(u0, u1) - TAU * 3.0).abs() < 1e-10, "{}", c.length(u0, u1));
    }

    #[test]
    fn a_curve_through_points_goes_through_them() {
        let pts: Vec<Vec3> = [(0.0, 0.0), (3.0, 4.0), (7.0, 1.0), (12.0, 6.0), (13.0, -2.0), (20.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        let s = Spline::through(&pts).unwrap();
        let (u0, u1) = s.domain();
        // The closest point search finds each of them on the curve.
        for p in &pts {
            assert!(s.closest(*p, u0, u1).1 < 1e-9, "{p:?} {}", s.closest(*p, u0, u1).1);
        }
        assert_eq!(s.degree(), 3);
        assert_eq!(Spline::through(&pts[..2]).unwrap().degree(), 1);
        assert_eq!(Spline::through(&pts[..3]).unwrap().degree(), 2);
        assert!(Spline::through(&[pts[0], pts[0]]).is_err());
        assert!(Spline::through(&pts[..1]).is_err());
    }

    #[test]
    fn a_sub_curve_matches_the_whole() {
        for s in [wavy(), rational()] {
            let (u0, u1) = s.domain();
            for (a, b) in [(u0, u1), (u0, (u0 + u1) * 0.4), ((u0 + u1) * 0.3, u1), ((u0 + u1) * 0.31, (u0 + u1) * 0.77)] {
                let part = s.sub(a, b).unwrap();
                assert_eq!(part.domain(), (a, b));
                for i in 0..=100 {
                    let t = a + (b - a) * i as f64 / 100.0;
                    assert!(part.eval(t).dist(s.eval(t)) < 1e-10, "{a}..{b} at {t}: {:?} vs {:?}", part.eval(t), s.eval(t));
                }
            }
        }
    }

    #[test]
    fn reversing_and_mapping_keep_the_curve() {
        for s in [wavy(), rational()] {
            let r = s.reversed();
            let (u0, u1) = s.domain();
            assert_eq!(r.domain(), (u0, u1));
            for i in 0..=100 {
                let t = u0 + (u1 - u0) * i as f64 / 100.0;
                assert!(r.eval(u0 + u1 - t).dist(s.eval(t)) < 1e-10);
            }
            let f = Frame::new(v3(1.0, 2.0, 3.0), v3(0.2, 0.3, 1.0), Vec3::X);
            let m = s.mapped(|p| f.to_world(p));
            for i in 0..=100 {
                let t = u0 + (u1 - u0) * i as f64 / 100.0;
                assert!(m.eval(t).dist(f.to_world(s.eval(t))) < 1e-10);
            }
        }
    }

    #[test]
    fn bad_splines_are_refused() {
        let pts = |n: usize| (0..n).map(|i| v3(i as f64, 0.0, 0.0)).collect::<Vec<_>>();
        let clamped = vec![0.0, 0.0, 1.0, 1.0];
        assert!(Spline::new(1, clamped.clone(), pts(2), vec![1.0, 1.0]).is_ok());
        assert!(Spline::new(0, clamped.clone(), pts(2), vec![1.0, 1.0]).is_err());
        assert!(Spline::new(8, clamped.clone(), pts(2), vec![1.0, 1.0]).is_err());
        assert!(Spline::new(1, clamped.clone(), pts(2), vec![1.0, -1.0]).is_err(), "negative weight");
        assert!(Spline::new(1, clamped.clone(), pts(2), vec![1.0]).is_err(), "too few weights");
        assert!(Spline::new(1, vec![0.0, 1.0, 1.0, 1.0], pts(2), vec![1.0, 1.0]).is_err(), "not clamped");
        assert!(Spline::new(1, vec![0.0, 0.0, 2.0, 1.0], pts(2), vec![1.0, 1.0]).is_err(), "knots go backward");
        assert!(Spline::new(1, vec![0.0, 0.0, 0.0, 0.0], pts(2), vec![1.0, 1.0]).is_err(), "no range");
        assert!(Spline::new(1, clamped.clone(), vec![Vec3::ZERO, Vec3::ZERO], vec![1.0, 1.0]).is_err(), "one place");
        assert!(Spline::new(1, clamped.clone(), vec![Vec3::ZERO, v3(f64::NAN, 0.0, 0.0)], vec![1.0, 1.0]).is_err());
        assert!(Spline::new(1, clamped, vec![Vec3::ZERO, v3(1e300, 0.0, 0.0)], vec![1.0, 1.0]).is_err());
    }

    #[test]
    fn pieces_turn_less_than_a_half_turn_and_cover_the_curve() {
        let circle = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 3.0, 3.0, 0.0, TAU).unwrap();
        let pieces = circle.face_pieces().unwrap();
        assert!((3..=6).contains(&pieces.len()), "{}", pieces.len());
        let mut at = circle.domain().0;
        for piece in &pieces {
            let FacePiece::Curve(c) = piece else { panic!("a circle has no straight parts") };
            assert_eq!(c.domain().0, at);
            at = c.domain().1;
            let (u0, u1) = c.domain();
            let d = c.derivative((u0 + u1) / 2.0).normalized();
            for i in 0..=64 {
                let t = u0 + (u1 - u0) * i as f64 / 64.0;
                assert!(c.derivative(t).normalized().dot(d) > 0.0, "every tangent within 90 degrees of the middle one");
            }
        }
        assert_eq!(at, circle.domain().1);
    }

    #[test]
    fn straight_stretches_become_lines_and_corners_cut() {
        let pts: Vec<Vec3> = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        let line = Spline::uniform(2, pts).unwrap();
        let pieces = line.face_pieces().unwrap();
        assert!(matches!(pieces.as_slice(), [FacePiece::Line(a, b)] if a.dist(Vec3::ZERO) < 1e-12 && b.dist(v3(10.0, 0.0, 0.0)) < 1e-12));

        // A polyline as a degree-one spline is all corners.
        let poly = Spline::uniform(1, vec![Vec3::ZERO, v3(1.0, 0.0, 0.0), v3(1.0, 1.0, 0.0)]).unwrap();
        assert_eq!(poly.face_pieces().unwrap().len(), 2);

        // A corner in a quadratic, from a knot repeated twice.
        let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0];
        let p = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (3.0, 3.0), (2.0, 5.0)];
        let kinked = Spline::new(2, k, p.iter().map(|q| v3(q.0, q.1, 0.0)).collect(), vec![1.0; 5]).unwrap();
        let pieces = kinked.face_pieces().unwrap();
        assert!(pieces.len() >= 2, "{}", pieces.len());
        let FacePiece::Curve(first) = &pieces[0] else { panic!() };
        assert_eq!(first.domain().1, 1.0);
    }

    #[test]
    fn a_curve_that_stops_dead_is_refused() {
        let pts: Vec<Vec3> = [(0.0, 0.0), (0.0, 0.0), (5.0, 3.0), (9.0, 0.0)].iter().map(|p| v3(p.0, p.1, 0.0)).collect();
        let s = Spline::uniform(3, pts).unwrap();
        assert!(s.face_pieces().is_err());
    }

    #[test]
    fn the_signed_distance_is_lipschitz_and_has_a_side() {
        let mut rng = Rng::new(9);
        for s in [wavy(), rational()] {
            let pieces = s.face_pieces().unwrap();
            for piece in pieces {
                let FacePiece::Curve(c) = piece else { continue };
                let (u0, u1) = c.domain();
                let pc = PlaneCurve::new(c);
                // Each side of the curve has the sign it should, near the curve.
                for i in 0..=40 {
                    let t = u0 + (u1 - u0) * i as f64 / 40.0;
                    let (q, d) = (pc.eval(t), pc.derivative(t).normalized());
                    assert!(pc.signed_distance(q).abs() < 1e-10);
                    assert!(pc.signed_distance(q + d.right() * 0.01) > 0.0, "right of travel");
                    assert!(pc.signed_distance(q - d.right() * 0.01) < 0.0, "left of travel");
                }
                for _ in 0..800 {
                    let p = v2(rng.range(-30.0, 60.0), rng.range(-40.0, 40.0));
                    let dir = rng.unit_vector();
                    let len = rng.range(0.0, 8.0);
                    let q = p + v2(dir.x, dir.y) * len;
                    let (a, b) = (pc.signed_distance(p), pc.signed_distance(q));
                    assert!((a - b).abs() <= p.dist(q) + 1e-9, "{p:?} {q:?}: {a} {b}");
                }
                // Past the ends, the extension counts: it's straight on from the end tangent.
                let (end, d) = (pc.eval(u1), pc.derivative(u1).normalized());
                let beyond = end + d * 5.0 + d.right() * 2.0;
                assert!((pc.signed_distance(beyond) - 2.0).abs() < 1e-9, "{}", pc.signed_distance(beyond));
            }
        }
    }

    #[test]
    fn a_rational_circle_has_a_circle_distance() {
        let c = Spline::ellipse_arc(Vec3::ZERO, Vec3::X, Vec3::Y, 4.0, 4.0, 0.0, PI / 2.0).unwrap();
        let (u0, u1) = c.domain();
        let pc = PlaneCurve::new(c);
        assert!((pc.min_radius() - 4.0).abs() < 1e-9, "{}", pc.min_radius());
        assert!(pc.speed >= pc.derivative((u0 + u1) / 2.0).norm());
        let mut rng = Rng::new(3);
        for _ in 0..500 {
            let a = rng.range(0.05, PI / 2.0 - 0.05);
            let r = rng.range(0.5, 12.0);
            let q = v2(a.cos(), a.sin()) * r;
            // Counter-clockwise, so the right of travel is outward.
            assert!((pc.signed_distance(q) - (r - 4.0)).abs() < 1e-10, "{q:?}");
        }
    }
}
