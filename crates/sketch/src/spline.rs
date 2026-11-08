//! A rational B-spline in a sketch: control points the solver can move, with weights and knots that stay
//! put. This crate has no dependencies, so it evaluates these for itself, and the kernel's own spline
//! takes over once a sketch becomes a profile.

use crate::model::P2;
use std::sync::OnceLock;

/// The highest degree a sketch spline can have, which is the kernel's limit too.
pub const MAX_DEGREE: usize = 7;

#[derive(Clone, Debug, PartialEq)]
pub struct SplineShape {
    pub degree: usize,
    /// Clamped: the first and last repeat `degree + 1` times, so the curve starts and ends on its end
    /// control points. Never decreasing.
    pub knots: Vec<f64>,
    pub points: Vec<P2>,
    pub weights: Vec<f64>,
}

fn gauss_16() -> &'static [(f64, f64)] {
    static RULE: OnceLock<Vec<(f64, f64)>> = OnceLock::new();
    RULE.get_or_init(|| {
        let n = 16;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
            let mut dp = 0.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let k = k as f64;
                    let p2 = ((2.0 * k - 1.0) * x * p1 - (k - 1.0) * p0) / k;
                    p0 = p1;
                    p1 = p2;
                }
                dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
                let dx = p1 / dp;
                x -= dx;
                if dx.abs() < 1e-16 {
                    break;
                }
            }
            out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
        }
        out
    })
}

impl SplineShape {
    /// A spline from its control data, checked.
    pub fn new(degree: usize, knots: Vec<f64>, points: Vec<P2>, weights: Vec<f64>) -> Result<SplineShape, String> {
        let shape = SplineShape { degree, knots, points, weights };
        shape.check()?;
        Ok(shape)
    }

    /// Whether the numbers make a spline.
    pub fn check(&self) -> Result<(), String> {
        let (p, n) = (self.degree, self.points.len());
        if !(1..=MAX_DEGREE).contains(&p) {
            return Err(format!("a spline's degree must be from 1 to {MAX_DEGREE}, not {p}"));
        }
        if n < p + 1 {
            return Err(format!("a spline of degree {p} needs at least {} control points, not {n}", p + 1));
        }
        if self.weights.len() != n {
            return Err(format!("a spline has {} weights for {n} control points", self.weights.len()));
        }
        if self.knots.len() != n + p + 1 {
            return Err(format!("a spline with {n} control points of degree {p} needs {} knots, not {}", n + p + 1, self.knots.len()));
        }
        if !self.points.iter().flatten().all(|v| v.is_finite()) {
            return Err("a spline has a control point that isn't a finite number".to_string());
        }
        if !self.weights.iter().all(|w| w.is_finite() && *w > 0.0) {
            return Err("a spline's weights must all be positive".to_string());
        }
        if !self.knots.iter().all(|k| k.is_finite()) || self.knots.windows(2).any(|w| w[1] < w[0]) {
            return Err("a spline's knots must be finite and never decrease".to_string());
        }
        let (first, last) = (self.knots[0], self.knots[self.knots.len() - 1]);
        if !(last > first) || self.knots[p] != first || self.knots[self.knots.len() - 1 - p] != last {
            return Err(format!("a spline's first and last knots must each repeat {} times", p + 1));
        }
        let mut run = 1;
        for w in self.knots[p..=self.knots.len() - 1 - p].windows(2) {
            run = if w[0] == w[1] { run + 1 } else { 1 };
            if run > p && w[1] != last && w[0] != first {
                return Err(format!("a spline's interior knot repeats more than {p} times"));
            }
        }
        Ok(())
    }

    /// A polynomial spline with evenly spaced knots, in units of its control polygon's length.
    pub fn uniform(degree: usize, points: Vec<P2>) -> Result<SplineShape, String> {
        let n = points.len();
        if degree < 1 || n < degree + 1 {
            return Err(format!("a spline of degree {degree} needs at least {} control points, not {n}", degree + 1));
        }
        let total: f64 = points.windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).sum();
        let intervals = n - degree;
        let step = total.max(1e-12) / intervals as f64;
        let mut knots = vec![0.0; degree + 1];
        for i in 1..intervals {
            knots.push(step * i as f64);
        }
        knots.extend(vec![step * intervals as f64; degree + 1]);
        let weights = vec![1.0; n];
        SplineShape::new(degree, knots, points, weights)
    }

    /// The smooth curve through these points: cubic if there are at least four, spaced by the chords
    /// between them. Only its control points stay in the sketch, so moving one later won't keep the curve
    /// on these points.
    pub fn through(points: &[P2]) -> Result<SplineShape, String> {
        let n = points.len();
        if n < 2 {
            return Err("a curve through points needs at least two of them".to_string());
        }
        let degree = 3.min(n - 1);
        let mut u = vec![0.0; n];
        for i in 1..n {
            let chord = (points[i][0] - points[i - 1][0]).hypot(points[i][1] - points[i - 1][1]);
            if !(chord > 1e-9) {
                return Err(format!("through-points {} and {i} are in one place", i - 1));
            }
            u[i] = u[i - 1] + chord;
        }
        let mut knots = vec![0.0; degree + 1];
        for j in 1..n - degree {
            knots.push(u[j..j + degree].iter().sum::<f64>() / degree as f64);
        }
        knots.extend(vec![u[n - 1]; degree + 1]);
        // Banded and totally positive, so elimination without pivoting is stable.
        let band = degree;
        let mut rows = vec![vec![0.0; 2 * band + 1]; n];
        let shape = SplineShape { degree, knots: knots.clone(), points: vec![[0.0, 0.0]; n], weights: vec![1.0; n] };
        for (k, row) in rows.iter_mut().enumerate() {
            let span = shape.span_of(u[k]);
            let basis = shape.basis(span, u[k]);
            for (b, value) in basis.iter().enumerate() {
                row[span - degree + b + band - k] = *value;
            }
        }
        let mut rhs = points.to_vec();
        for k in 0..n {
            let pivot = rows[k][band];
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
                rhs[below] = [rhs[below][0] - r[0] * f, rhs[below][1] - r[1] * f];
            }
        }
        let mut solved = vec![[0.0; 2]; n];
        for k in (0..n).rev() {
            let mut sum = rhs[k];
            for c in 1..=band {
                if k + c < n {
                    sum = [sum[0] - solved[k + c][0] * rows[k][band + c], sum[1] - solved[k + c][1] * rows[k][band + c]];
                }
            }
            solved[k] = [sum[0] / rows[k][band], sum[1] / rows[k][band]];
        }
        SplineShape::new(degree, knots, solved, vec![1.0; n])
    }

    pub fn domain(&self) -> (f64, f64) {
        (self.knots[0], self.knots[self.knots.len() - 1])
    }

    /// The knot interval holding `t`.
    fn span_of(&self, t: f64) -> usize {
        let (p, n) = (self.degree, self.points.len());
        let mut k = if t >= self.knots[n] { n - 1 } else { self.knots.partition_point(|&u| u <= t).saturating_sub(1).clamp(p, n - 1) };
        while k > p && self.knots[k + 1] <= self.knots[k] {
            k -= 1;
        }
        k
    }

    fn basis(&self, span: usize, t: f64) -> Vec<f64> {
        let p = self.degree;
        let mut out = vec![0.0; p + 1];
        out[0] = 1.0;
        let (mut left, mut right) = (vec![0.0; p + 1], vec![0.0; p + 1]);
        for j in 1..=p {
            left[j] = t - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - t;
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

    /// The point and its derivative with respect to `t`, by de Boor's algorithm in homogeneous form.
    pub fn eval(&self, t: f64) -> (P2, P2) {
        let (u0, u1) = self.domain();
        let t = t.clamp(u0, u1);
        let (p, k) = (self.degree, self.span_of(t));
        let mut d = [[0.0f64; 3]; MAX_DEGREE + 1];
        for i in 0..=p {
            let (q, w) = (self.points[k - p + i], self.weights[k - p + i]);
            d[i] = [q[0] * w, q[1] * w, w];
        }
        let mut last_difference = [0.0; 3];
        for r in 1..=p {
            for j in (r..=p).rev() {
                let (lo, hi) = (self.knots[k - p + j], self.knots[k + 1 + j - r]);
                let alpha = (t - lo) / (hi - lo);
                if r == p {
                    let scale = p as f64 / (hi - lo);
                    last_difference = [(d[j][0] - d[j - 1][0]) * scale, (d[j][1] - d[j - 1][1]) * scale, (d[j][2] - d[j - 1][2]) * scale];
                }
                for c in 0..3 {
                    d[j][c] = d[j - 1][c] + (d[j][c] - d[j - 1][c]) * alpha;
                }
            }
        }
        let [x, y, w] = d[p];
        let at = [x / w, y / w];
        let [dx, dy, dw] = last_difference;
        (at, [(dx - at[0] * dw) / w, (dy - at[1] * dw) / w])
    }

    /// The parameters where the curve has a knot, from first to last, each once.
    pub fn breaks(&self) -> Vec<f64> {
        let mut out = self.knots.clone();
        out.dedup();
        out
    }

    /// Twice the area this piece sweeps about the origin: the integral of `x dy - y dx`. It's exact to
    /// rounding for a polynomial spline, and very nearly so for a rational one.
    pub fn twice_area(&self, t0: f64, t1: f64) -> f64 {
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let mut stops = vec![lo];
        stops.extend(self.breaks().into_iter().filter(|&k| k > lo && k < hi));
        stops.push(hi);
        let mut sum = 0.0;
        for w in stops.windows(2) {
            let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
            for &(x, weight) in gauss_16() {
                let (p, d) = self.eval(mid + half * x);
                sum += (p[0] * d[1] - p[1] * d[0]) * weight * half;
            }
        }
        if t1 >= t0 {
            sum
        } else {
            -sum
        }
    }

    /// The bounds of its control points, which hold the curve.
    pub fn bounds(&self) -> [f64; 4] {
        let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for q in &self.points {
            b = [b[0].min(q[0]), b[1].min(q[1]), b[2].max(q[0]), b[3].max(q[1])];
        }
        b
    }
}
