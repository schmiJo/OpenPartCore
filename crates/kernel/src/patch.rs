//! Free-form surfaces: a rational tensor-product B-spline, stored as a grid of Bezier cells.
//!
//! The kernel's Booleans rest on one guarantee: a surface's distance function never overstates how far away
//! the surface is, so a value of `d` proves nothing on the surface lies within `d`. A patch keeps that, and the
//! closest point is found with certainty rather than by following a guess from somewhere nearby:
//!
//! * The closest point in the interior makes `(A - p w) . (A_s w - A w_s)` and its partner in `t` both zero. In
//!   Bernstein form their coefficients are linear in `p` and bound their values, so a box where all of one
//!   polynomial's coefficients have one sign can't hold a closest point, and it's discarded.
//! * A box whose Jacobian is regular across its whole range holds at most one root, and Newton finds it.
//! * The rest of the closest points are on the four boundary curves, which `Spline::closest` handles.
//!
//! Past its boundary a patch carries on along its tangent planes, as a curve carries on along its end tangents,
//! so a section through the edge of a face continues smoothly. Out there the value is the height above the
//! tangent plane at the nearest boundary point. It's no larger than the distance to the patch, so the
//! guarantee still holds for every point of the patch, which is where faces are.

use crate::error::{Error, Result};
use crate::math::*;
use crate::spline::{basis_functions, box_distance, find_knot_span, Hom, Spline, MAX_DEGREE};
use std::cell::Cell as ThreadCell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::OnceLock;

/// How many boxes one closest-point search may open. Near the surface a search takes a handful. This only
/// matters where a great many points are almost equally close, such as at the center of a sphere's patch,
/// and there the search reports a distance it can prove instead of an exact one.
const SEARCH_BUDGET: usize = 600;
const SEARCH_DEPTH: u32 = 28;

/// What a grid of Bernstein coefficients needs for halving.
trait Midpoint: Copy {
    fn midpoint(self, o: Self) -> Self;
}

impl Midpoint for f64 {
    fn midpoint(self, o: f64) -> f64 {
        (self + o) / 2.0
    }
}

impl Midpoint for Vec3 {
    fn midpoint(self, o: Vec3) -> Vec3 {
        (self + o) * 0.5
    }
}

impl Midpoint for Hom {
    fn midpoint(self, o: Hom) -> Hom {
        self.lerp(o, 0.5)
    }
}

/// A Bernstein polynomial over a rectangle: `r` coefficients across `s` and `c` across `t`.
#[derive(Clone)]
struct Grid<T = f64> {
    r: usize,
    c: usize,
    a: Vec<T>,
}

impl<T: Midpoint> Grid<T> {
    /// Both halves in `s`, or in `t` if `along_t`.
    fn halves(&self, along_t: bool) -> (Grid<T>, Grid<T>) {
        let (r, c) = (self.r, self.c);
        let (mut left, mut right) = (self.clone(), self.clone());
        let (count, lines) = if along_t { (c, r) } else { (r, c) };
        let mut work = self.a[..count].to_vec();
        for line in 0..lines {
            let at = |k: usize| if along_t { line * c + k } else { k * c + line };
            for (k, w) in work.iter_mut().enumerate() {
                *w = self.a[at(k)];
            }
            for level in 0..count {
                left.a[at(level)] = work[0];
                right.a[at(count - 1 - level)] = work[count - 1 - level];
                for i in 0..count - 1 - level {
                    work[i] = work[i].midpoint(work[i + 1]);
                }
            }
        }
        (left, right)
    }
}

impl Grid<f64> {
    /// The value at the middle of the box.
    fn center_value(&self) -> f64 {
        let (r, c) = (self.r, self.c);
        let mut line = vec![0.0; r];
        let mut work = vec![0.0; c];
        for (i, out) in line.iter_mut().enumerate() {
            work.copy_from_slice(&self.a[i * c..(i + 1) * c]);
            for level in (1..c).rev() {
                for k in 0..level {
                    work[k] = (work[k] + work[k + 1]) / 2.0;
                }
            }
            *out = work[0];
        }
        for level in (1..r).rev() {
            for k in 0..level {
                line[k] = (line[k] + line[k + 1]) / 2.0;
            }
        }
        line[0]
    }

    fn one_sign(&self) -> bool {
        self.a.iter().all(|&x| x > 0.0) || self.a.iter().all(|&x| x < 0.0)
    }

    /// The range of the derivative along `s`, or `t`, bounded by its own Bernstein coefficients.
    fn slope(&self, along_t: bool) -> (f64, f64) {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let (r, c) = (self.r, self.c);
        let degree = if along_t { c } else { r } as f64 - 1.0;
        if degree < 1.0 {
            return (0.0, 0.0);
        }
        for i in 0..r {
            for j in 0..c {
                let next = if along_t {
                    if j + 1 >= c {
                        continue;
                    }
                    self.a[i * c + j + 1]
                } else {
                    if i + 1 >= r {
                        continue;
                    }
                    self.a[(i + 1) * c + j]
                };
                let d = (next - self.a[i * c + j]) * degree;
                lo = lo.min(d);
                hi = hi.max(d);
            }
        }
        (lo, hi)
    }
}

/// The ranges of the two polynomials' four partial derivatives over a box.
struct Slopes {
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
    d: (f64, f64),
}

impl Slopes {
    fn of(gs: &Grid, gt: &Grid) -> Slopes {
        Slopes { a: gs.slope(false), b: gs.slope(true), c: gt.slope(false), d: gt.slope(true) }
    }

    /// Whether the Jacobian can't vanish over the box, which allows at most one common root.
    fn regular(&self) -> bool {
        let smallest = |(lo, hi): (f64, f64)| if lo > 0.0 { lo } else if hi < 0.0 { -hi } else { 0.0 };
        let largest = |(lo, hi): (f64, f64)| lo.abs().max(hi.abs());
        let (a, b, c, d) = (self.a, self.b, self.c, self.d);
        smallest(a) * smallest(d) > largest(b) * largest(c) || smallest(b) * smallest(c) > largest(a) * largest(d)
    }

    /// Krawczyk's test: every common root in the box lies in an image of the box worked out from its center,
    /// so an image that misses the box proves there's no root in it. A box right beside the answer gets
    /// settled here in one step, where cutting it up would take many.
    fn no_root(&self, gs: &Grid, gt: &Grid) -> bool {
        let mid = |r: (f64, f64)| (r.0 + r.1) / 2.0;
        let rad = |r: (f64, f64)| (r.1 - r.0) / 2.0;
        let (ma, mb, mc, md) = (mid(self.a), mid(self.b), mid(self.c), mid(self.d));
        let det = ma * md - mb * mc;
        if det == 0.0 || !det.is_finite() {
            return false;
        }
        let (y11, y12, y21, y22) = (md / det, -mb / det, -mc / det, ma / det);
        let (fs, ft) = (gs.center_value(), gt.center_value());
        let (cx, cy) = (0.5 - (y11 * fs + y12 * ft), 0.5 - (y21 * fs + y22 * ft));
        let (ra, rb, rc, rd) = (rad(self.a), rad(self.b), rad(self.c), rad(self.d));
        // The box's half width is a half in each direction.
        let px = 0.5 * (y11.abs() * (ra + rb) + y12.abs() * (rc + rd));
        let py = 0.5 * (y21.abs() * (ra + rb) + y22.abs() * (rc + rd));
        const SLACK: f64 = 1e-9;
        cx - px > 1.0 + SLACK || cx + px < -SLACK || cy - py > 1.0 + SLACK || cy + py < -SLACK
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

/// Weights for multiplying Bernstein polynomials of degrees `a` and `b`.
fn product_weights(a: usize, b: usize) -> Vec<f64> {
    let mut out = vec![0.0; (a + 1) * (b + 1)];
    for i in 0..=a {
        for j in 0..=b {
            out[i * (b + 1) + j] = binomial(a, i) * binomial(b, j) / binomial(a + b, i + j);
        }
    }
    out
}

/// A product of two gridded Bernstein polynomials, with `join` multiplying their coefficients.
fn product<A: Copy, B: Copy, C: Copy + Default + std::ops::AddAssign + std::ops::Mul<f64, Output = C>>(
    a: &[A],
    (ar, ac): (usize, usize),
    b: &[B],
    (br, bc): (usize, usize),
    join: impl Fn(A, B) -> C,
) -> (Vec<C>, (usize, usize)) {
    let (r, c) = (ar + br - 1, ac + bc - 1);
    let (across, along) = (product_weights(ar - 1, br - 1), product_weights(ac - 1, bc - 1));
    let mut out = vec![C::default(); r * c];
    for i in 0..ar {
        for j in 0..ac {
            for k in 0..br {
                for l in 0..bc {
                    let weight = across[i * br + k] * along[j * bc + l];
                    out[(i + k) * c + j + l] += join(a[i * ac + j], b[k * bc + l]) * weight;
                }
            }
        }
    }
    (out, (r, c))
}

/// The two stationary polynomials of a cell, as coefficients `c[i] - p . v[i]`.
struct Stationary {
    s: (usize, usize),
    cs: Vec<f64>,
    vs: Vec<Vec3>,
    t: (usize, usize),
    ct: Vec<f64>,
    vt: Vec<Vec3>,
}

type Basis = [f64; MAX_DEGREE + 1];

/// Bernstein basis values of one degree.
fn bernstein_values(degree: usize, s: f64) -> Basis {
    let mut b = [0.0; MAX_DEGREE + 1];
    b[0] = 1.0;
    let r = 1.0 - s;
    for n in 1..=degree {
        let mut carried = 0.0;
        for x in b.iter_mut().take(n) {
            let old = *x;
            *x = carried + old * r;
            carried = old * s;
        }
        b[n] = carried;
    }
    b
}

/// Bernstein basis values and their first derivatives.
fn bernstein_first(degree: usize, s: f64) -> (Basis, Basis) {
    let mut d = [0.0; MAX_DEGREE + 1];
    let l = bernstein_values(degree - 1, s);
    let n = degree as f64;
    d[0] = -n * l[0];
    for i in 1..degree {
        d[i] = n * (l[i - 1] - l[i]);
    }
    d[degree] = n * l[degree - 1];
    (bernstein_values(degree, s), d)
}

/// Bernstein basis values and their first two derivatives.
fn bernstein(degree: usize, s: f64) -> (Basis, Basis, Basis) {
    let (mut d1, mut d2) = ([0.0; MAX_DEGREE + 1], [0.0; MAX_DEGREE + 1]);
    if degree >= 1 {
        let l = bernstein_values(degree - 1, s);
        let n = degree as f64;
        d1[0] = -n * l[0];
        for i in 1..degree {
            d1[i] = n * (l[i - 1] - l[i]);
        }
        d1[degree] = n * l[degree - 1];
    }
    if degree >= 2 {
        let l = bernstein_values(degree - 2, s);
        let n = (degree * (degree - 1)) as f64;
        for i in 0..=degree {
            let a = if i >= 2 { l[i - 2] } else { 0.0 };
            let b = if i >= 1 && i - 1 <= degree - 2 { l[i - 1] } else { 0.0 };
            let c = if i <= degree - 2 { l[i] } else { 0.0 };
            d2[i] = n * (a - 2.0 * b + c);
        }
    }
    (bernstein_values(degree, s), d1, d2)
}

/// A point of a surface with its derivatives, in the cell's own `s` and `t`.
#[derive(Clone, Copy)]
struct Local {
    p: Vec3,
    ds: Vec3,
    dt: Vec3,
    dss: Vec3,
    dst: Vec3,
    dtt: Vec3,
}

/// How many times a cell is halved each way ahead of time. Most searches then open a leaf or two and find
/// everything already small enough to decide. A surface with a great many cells gets fewer, since each cell's
/// leaves take memory and those cells are small to begin with.
fn leaf_levels(cells: usize) -> u32 {
    match cells {
        0..=64 => 3,
        65..=512 => 2,
        _ => 1,
    }
}

/// A small piece of a cell: its range, two bounding boxes, and its stationary polynomials cut down to it.
struct Leaf {
    s: (f64, f64),
    t: (f64, f64),
    lo: Vec3,
    hi: Vec3,
    center: Vec3,
    /// A box along the surface here, which fits a slanted piece far better than one along the axes: its
    /// axes, and the range of the control points along each, measured from the center.
    axes: [Vec3; 3],
    range: [(f64, f64); 3],
    cs: Grid,
    vs: Grid<Vec3>,
    ct: Grid,
    vt: Grid<Vec3>,
}

struct BezierCell {
    u: (f64, f64),
    v: (f64, f64),
    m: usize,
    n: usize,
    h: Vec<Hom>,
    lo: Vec3,
    hi: Vec3,
    /// No speed along `s`, or `t`, exceeds these.
    speed: (f64, f64),
    center: Vec3,
    levels: u32,
    leaves: OnceLock<Vec<Leaf>>,
}

impl BezierCell {
    fn new(u: (f64, f64), v: (f64, f64), m: usize, n: usize, h: Vec<Hom>) -> BezierCell {
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        for q in &h {
            let p = q.point();
            lo = lo.min(p);
            hi = hi.max(p);
        }
        let mut out = BezierCell { u, v, m, n, h, lo, hi, speed: (0.0, 0.0), center: Vec3::ZERO, levels: 3, leaves: OnceLock::new() };
        out.center = out.point(0.5, 0.5);
        let (ns, nt, _, _) = out.tangent_numerators();
        let least = out.h.iter().fold(f64::INFINITY, |m, q| m.min(q.w));
        let bound = |list: &[Vec3]| list.iter().fold(0.0f64, |m, v| m.max(v.norm())) / (least * least);
        out.speed = (bound(&ns), bound(&nt));
        out
    }

    fn polynomial(&self) -> bool {
        self.h.iter().all(|q| q.w == self.h[0].w)
    }

    /// `A_s w - A w_s` and the same in `t`, with their shapes, divided through by the weight if it's constant.
    fn tangent_numerators(&self) -> (Vec<Vec3>, Vec<Vec3>, (usize, usize), (usize, usize)) {
        let (m, n) = (self.m, self.n);
        let width = n + 1;
        let a: Vec<Vec3> = self.h.iter().map(|q| q.a).collect();
        let w: Vec<f64> = self.h.iter().map(|q| q.w).collect();
        let along_s = |x: &dyn Fn(usize) -> Vec3| -> Vec<Vec3> {
            let mut out = Vec::with_capacity(m * width);
            for i in 0..m {
                for j in 0..width {
                    out.push((x((i + 1) * width + j) - x(i * width + j)) * m as f64);
                }
            }
            out
        };
        let along_t = |x: &dyn Fn(usize) -> Vec3| -> Vec<Vec3> {
            let mut out = Vec::with_capacity((m + 1) * n);
            for i in 0..=m {
                for j in 0..n {
                    out.push((x(i * width + j + 1) - x(i * width + j)) * n as f64);
                }
            }
            out
        };
        let (as_, at) = (along_s(&|k| a[k]), along_t(&|k| a[k]));
        if self.polynomial() {
            let scale = 1.0 / w[0];
            let down = |list: Vec<Vec3>| list.into_iter().map(|v| v * scale).collect::<Vec<_>>();
            return (down(as_), down(at), (m, width), (m + 1, n));
        }
        let (ws, wt): (Vec<f64>, Vec<f64>) = {
            let ws = along_s(&|k| v3(w[k], 0.0, 0.0)).iter().map(|v| v.x).collect();
            let wt = along_t(&|k| v3(w[k], 0.0, 0.0)).iter().map(|v| v.x).collect();
            (ws, wt)
        };
        let shape = (m + 1, width);
        let minus = |x: (Vec<Vec3>, (usize, usize)), y: (Vec<Vec3>, (usize, usize))| {
            let list: Vec<Vec3> = x.0.iter().zip(&y.0).map(|(a, b)| *a - *b).collect();
            (list, x.1)
        };
        let scaled = |v: Vec3, s: f64| v * s;
        let (ns, ss) = minus(product(&as_, (m, width), &w, shape, scaled), product(&a, shape, &ws, (m, width), scaled));
        let (nt, st) = minus(product(&at, (m + 1, n), &w, shape, scaled), product(&a, shape, &wt, (m + 1, n), scaled));
        (ns, nt, ss, st)
    }

    fn stationary(&self) -> Stationary {
        let (ns, nt, ss, st) = self.tangent_numerators();
        let shape = (self.m + 1, self.n + 1);
        let polynomial = self.polynomial();
        // On a constant weight, the factor it would contribute has been divided out.
        let a: Vec<Vec3> = self.h.iter().map(|q| if polynomial { q.a / q.w } else { q.a }).collect();
        let w: Vec<f64> = self.h.iter().map(|q| if polynomial { 1.0 } else { q.w }).collect();
        let (cs, s) = product(&a, shape, &ns, ss, |x, y| x.dot(y));
        let (vs, _) = product(&w, shape, &ns, ss, |x, y| y * x);
        let (ct, t) = product(&a, shape, &nt, st, |x, y| x.dot(y));
        let (vt, _) = product(&w, shape, &nt, st, |x, y| y * x);
        Stationary { s, cs, vs, t, ct, vt }
    }

    fn leaves(&self) -> &[Leaf] {
        self.leaves.get_or_init(|| {
            let poly = self.stationary();
            let grid = |(r, c): (usize, usize), a: &[f64]| Grid { r, c, a: a.to_vec() };
            let grid3 = |(r, c): (usize, usize), a: &[Vec3]| Grid { r, c, a: a.to_vec() };
            let root = Leaf {
                s: (0.0, 1.0),
                t: (0.0, 1.0),
                lo: self.lo,
                hi: self.hi,
                center: self.center,
                axes: [Vec3::X, Vec3::Y, Vec3::Z],
                range: [(0.0, 0.0); 3],
                cs: grid(poly.s, &poly.cs),
                vs: grid3(poly.s, &poly.vs),
                ct: grid(poly.t, &poly.ct),
                vt: grid3(poly.t, &poly.vt),
            };
            let net = Grid { r: self.m + 1, c: self.n + 1, a: self.h.clone() };
            let mut level = vec![(root, net)];
            for _ in 0..2 * self.levels {
                let mut next = Vec::with_capacity(level.len() * 2);
                for (leaf, net) in level {
                    // Each way in turn.
                    let along_t = leaf_along_t(&leaf);
                    let ((n0, n1), (cs0, cs1), (vs0, vs1), (ct0, ct1), (vt0, vt1)) =
                        (net.halves(along_t), leaf.cs.halves(along_t), leaf.vs.halves(along_t), leaf.ct.halves(along_t), leaf.vt.halves(along_t));
                    let (ms, mt) = ((leaf.s.0 + leaf.s.1) / 2.0, (leaf.t.0 + leaf.t.1) / 2.0);
                    let ((s0, t0), (s1, t1)) =
                        if along_t { ((leaf.s, (leaf.t.0, mt)), (leaf.s, (mt, leaf.t.1))) } else { (((leaf.s.0, ms), leaf.t), ((ms, leaf.s.1), leaf.t)) };
                    let make = |s: (f64, f64), t: (f64, f64), net: Grid<Hom>, cs, vs, ct, vt| {
                        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
                        for q in &net.a {
                            let x = q.point();
                            lo = lo.min(x);
                            hi = hi.max(x);
                        }
                        let q = self.local((s.0 + s.1) / 2.0, (t.0 + t.1) / 2.0);
                        let (x, z) = (q.ds.normalized(), q.ds.cross(q.dt).normalized());
                        let axes = [x, z.cross(x), z];
                        let mut range = [(f64::INFINITY, f64::NEG_INFINITY); 3];
                        for c in &net.a {
                            let r = c.point() - q.p;
                            for (axis, span) in axes.iter().zip(range.iter_mut()) {
                                let along = r.dot(*axis);
                                *span = (span.0.min(along), span.1.max(along));
                            }
                        }
                        (Leaf { s, t, lo, hi, center: q.p, axes, range, cs, vs, ct, vt }, net)
                    };
                    next.push(make(s0, t0, n0, cs0, vs0, ct0, vt0));
                    next.push(make(s1, t1, n1, cs1, vs1, ct1, vt1));
                }
                level = next;
            }
            level.into_iter().map(|(leaf, _)| leaf).collect()
        })
    }

    fn local(&self, s: f64, t: f64) -> Local {
        let (bs, ds, dds) = bernstein(self.m, s);
        let (bt, dt, ddt) = bernstein(self.n, t);
        let (mut a, mut as_, mut at, mut ass, mut ast, mut att) = (Hom::ZERO, Hom::ZERO, Hom::ZERO, Hom::ZERO, Hom::ZERO, Hom::ZERO);
        let add = |sum: &mut Hom, q: Hom, f: f64| {
            sum.a += q.a * f;
            sum.w += q.w * f;
        };
        let width = self.n + 1;
        for i in 0..=self.m {
            for j in 0..=self.n {
                let q = self.h[i * width + j];
                add(&mut a, q, bs[i] * bt[j]);
                add(&mut as_, q, ds[i] * bt[j]);
                add(&mut at, q, bs[i] * dt[j]);
                add(&mut ass, q, dds[i] * bt[j]);
                add(&mut ast, q, ds[i] * dt[j]);
                add(&mut att, q, bs[i] * ddt[j]);
            }
        }
        let w = a.w;
        let p = a.a / w;
        let d_s = (as_.a - p * as_.w) / w;
        let d_t = (at.a - p * at.w) / w;
        let dss = (ass.a - d_s * (2.0 * as_.w) - p * ass.w) / w;
        let dtt = (att.a - d_t * (2.0 * at.w) - p * att.w) / w;
        let dst = (ast.a - d_s * at.w - d_t * as_.w - p * ast.w) / w;
        Local { p, ds: d_s, dt: d_t, dss, dst, dtt }
    }

    /// The point and its first derivatives alone.
    fn first(&self, s: f64, t: f64) -> (Vec3, Vec3, Vec3) {
        let (bs, ds) = bernstein_first(self.m, s);
        let (bt, dt) = bernstein_first(self.n, t);
        let (mut a, mut a_s, mut a_t) = (Hom::ZERO, Hom::ZERO, Hom::ZERO);
        let width = self.n + 1;
        for i in 0..=self.m {
            for j in 0..=self.n {
                let q = self.h[i * width + j];
                let (f, fs, ft) = (bs[i] * bt[j], ds[i] * bt[j], bs[i] * dt[j]);
                a.a += q.a * f;
                a.w += q.w * f;
                a_s.a += q.a * fs;
                a_s.w += q.w * fs;
                a_t.a += q.a * ft;
                a_t.w += q.w * ft;
            }
        }
        let p = a.a / a.w;
        (p, (a_s.a - p * a_s.w) / a.w, (a_t.a - p * a_t.w) / a.w)
    }

    fn point(&self, s: f64, t: f64) -> Vec3 {
        let (bs, bt) = (bernstein_values(self.m, s), bernstein_values(self.n, t));
        let (mut a, mut w) = (Vec3::ZERO, 0.0);
        let width = self.n + 1;
        for i in 0..=self.m {
            for j in 0..=self.n {
                let q = self.h[i * width + j];
                let f = bs[i] * bt[j];
                a += q.a * f;
                w += q.w * f;
            }
        }
        a / w
    }

    /// A stationary point of the distance, by Newton's method, in the cell's own parameters.
    /// Whether the distance to `p` is at a minimum here, rather than a saddle or a maximum.
    fn is_minimum(&self, s: f64, t: f64, p: Vec3) -> bool {
        let q = self.local(s, t);
        let r = q.p - p;
        let (a, b, d) = (q.ds.dot(q.ds) + r.dot(q.dss), q.ds.dot(q.dt) + r.dot(q.dst), q.dt.dot(q.dt) + r.dot(q.dtt));
        a > 0.0 && d > 0.0 && a * d - b * b > 0.0
    }

    fn newton(&self, mut s: f64, mut t: f64, p: Vec3) -> Option<(f64, f64)> {
        for _ in 0..16 {
            let q = self.local(s, t);
            let r = q.p - p;
            let (g1, g2) = (r.dot(q.ds), r.dot(q.dt));
            let (a, b, d) = (q.ds.dot(q.ds) + r.dot(q.dss), q.ds.dot(q.dt) + r.dot(q.dst), q.dt.dot(q.dt) + r.dot(q.dtt));
            let det = a * d - b * b;
            if det == 0.0 || !det.is_finite() {
                return None;
            }
            let (dx, dy) = ((d * g1 - b * g2) / det, (a * g2 - b * g1) / det);
            s -= dx;
            t -= dy;
            if !(-0.05..=1.05).contains(&s) || !(-0.05..=1.05).contains(&t) {
                return None;
            }
            if dx.abs() + dy.abs() < 1e-14 {
                return ((-1e-9..=1.0 + 1e-9).contains(&s) && (-1e-9..=1.0 + 1e-9).contains(&t))
                    .then_some((s.clamp(0.0, 1.0), t.clamp(0.0, 1.0)));
            }
        }
        None
    }
}

impl Leaf {
    /// A lower bound on the distance from a point to this piece of surface.
    fn distance(&self, p: Vec3) -> f64 {
        let flat = box_distance(p, self.lo, self.hi);
        let r = p - self.center;
        let mut sum = 0.0;
        for (axis, (lo, hi)) in self.axes.iter().zip(&self.range) {
            let along = r.dot(*axis);
            let gap = (lo - along).max(along - hi).max(0.0);
            sum += gap * gap;
        }
        flat.max(sum.sqrt())
    }
}

/// Which way a leaf is halved next: across whichever side is still longer in its own parameters.
fn leaf_along_t(leaf: &Leaf) -> bool {
    leaf.t.1 - leaf.t.0 > leaf.s.1 - leaf.s.0
}

/// The nearest point of a patch to some point.
#[derive(Clone, Copy, Debug)]
pub struct PatchFoot {
    pub u: f64,
    pub v: f64,
    pub at: Vec3,
    /// The unit normal there, along `S_u x S_v`.
    pub normal: Vec3,
    /// How far it is. Far from the surface, in the rare case a search ran out of budget, this is a smaller
    /// value it could prove instead.
    pub distance: f64,
    /// Whether it's on the boundary with the point off to one side, rather than straight off the surface.
    pub boundary: bool,
}

thread_local! {
    /// The last answer, since one point usually gets its distance and then its normal.
    static LAST: ThreadCell<Option<(u64, [u64; 3], PatchFoot)>> = const { ThreadCell::new(None) };
}

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

/// A rational B-spline surface with clamped ends. `points` and `weights` run over `u` first: index
/// `i * count_v + j`. Normal is along `S_u x S_v`.
pub struct Patch {
    id: u64,
    degree: (usize, usize),
    knots: (Vec<f64>, Vec<f64>),
    count: (usize, usize),
    points: Vec<Vec3>,
    weights: Vec<f64>,
    cells: Vec<BezierCell>,
    breaks: (Vec<f64>, Vec<f64>),
    /// The boundary, as `v = v0`, `u = u1`, `v = v1` and `u = u0`, with their bounding boxes. A side that has shrunk to
    /// a point is a pole, and has no curve.
    sides: Vec<Option<(Spline, Vec3, Vec3)>>,
    /// The poles, as points.
    poles: Vec<Vec3>,
    /// Chart units per unit of `u` and `v`. Large enough that 3D distance never exceeds chart distance.
    scale: (f64, f64),
    lo: Vec3,
    hi: Vec3,
    min_radius: f64,
    /// How far the surface is moved along its normal from the one the control points define. Everything about
    /// closest points still happens on those, which is why an offset costs no precision: see `moved_by`.
    offset: f64,
    /// `poles`, moved with the surface. It's `poles` itself when there's no offset.
    moved_poles: Vec<Vec3>,
    /// The normal at each of `poles`, when something needs it exactly, as a moved surface does: a moved pole has to land
    /// on one point, and the normal from a little way in is only nearly that.
    pole_normals: Vec<(Vec3, Vec3)>,
}

impl std::fmt::Debug for Patch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Patch(degree {:?}, {:?} control points, offset {})", self.degree, self.count, self.offset)
    }
}

impl PartialEq for Patch {
    fn eq(&self, o: &Patch) -> bool {
        self.id == o.id
            || (self.degree == o.degree && self.knots == o.knots && self.points == o.points && self.weights == o.weights && self.offset == o.offset)
    }
}

/// Raises every interior knot to full multiplicity, so that each knot interval is one Bezier piece.
/// `lines` holds control points in lines along this direction. Returns the distinct knots.
fn extract(degree: usize, knots: &[f64], lines: &mut [Vec<Hom>]) -> Vec<f64> {
    let (first, last) = (knots[0], knots[knots.len() - 1]);
    let mut working = knots.to_vec();
    let mut distinct = vec![first];
    let mut interior: Vec<f64> = knots.iter().copied().filter(|k| *k > first && *k < last).collect();
    interior.dedup();
    for x in interior {
        distinct.push(x);
        raise_knot(degree, &mut working, x, lines);
    }
    distinct.push(last);
    distinct
}

/// Raises a knot to full multiplicity in every line, which makes the surface pass through a control line there.
fn raise_knot(degree: usize, knots: &mut Vec<f64>, x: f64, lines: &mut [Vec<Hom>]) {
    let mut multiplicity = knots.iter().filter(|k| **k == x).count();
    while multiplicity < degree {
        let k = knots.iter().rposition(|u| *u <= x).unwrap();
        for line in lines.iter_mut() {
            let mut next = Vec::with_capacity(line.len() + 1);
            for i in 0..=line.len() {
                next.push(if i + degree <= k {
                    line[i]
                } else if i + multiplicity > k {
                    line[i - 1]
                } else {
                    let alpha = (x - knots[i]) / (knots[i + degree] - knots[i]);
                    line[i - 1].lerp(line[i], alpha)
                });
            }
            *line = next;
        }
        knots.insert(k + 1, x);
        multiplicity += 1;
    }
}

/// Knots and which control points make up the part of the lines between two parameters.
pub(crate) fn cut_lines(degree: usize, knots: &[f64], (a, b): (f64, f64), lines: &mut [Vec<Hom>]) -> (Vec<f64>, (usize, usize)) {
    let mut working = knots.to_vec();
    let (first, last) = (knots[0], knots[knots.len() - 1]);
    if a > first {
        raise_knot(degree, &mut working, a, lines);
    }
    if b < last {
        raise_knot(degree, &mut working, b, lines);
    }
    let start = if a > first { working.iter().position(|k| *k == a).unwrap() - 1 } else { 0 };
    let end = if b < last { working.iter().position(|k| *k == b).unwrap() - 1 } else { lines[0].len() - 1 };
    let mut out = vec![a; degree + 1];
    out.extend(working.iter().copied().filter(|k| *k > a && *k < b));
    out.extend(vec![b; degree + 1]);
    (out, (start, end))
}

impl Patch {
    pub fn new(degree: (usize, usize), knots: (Vec<f64>, Vec<f64>), count: (usize, usize), points: Vec<Vec3>, weights: Vec<f64>) -> Result<Patch> {
        let bad = |why: String| Err(Error::InvalidInput(format!("free-form surface: {why}")));
        let (p, q) = degree;
        let (nu, nv) = count;
        if !(1..=MAX_DEGREE).contains(&p) || !(1..=MAX_DEGREE).contains(&q) {
            return bad(format!("degree {p} by {q} isn't within 1 to {MAX_DEGREE}"));
        }
        if points.len() != nu * nv || weights.len() != nu * nv {
            return bad(format!("{nu} by {nv} control points need {} points and weights", nu * nv));
        }
        if !points.iter().all(|c| c.is_finite() && c.x.abs().max(c.y.abs()).max(c.z.abs()) <= MAX_COORD) {
            return bad(format!("a control point is beyond the {MAX_COORD} mm this kernel works within"));
        }
        // The boundary curves check the knots and weights, and they're needed anyway.
        let row = |j: usize| -> (Vec<Vec3>, Vec<f64>) { ((0..nu).map(|i| points[i * nv + j]).collect(), (0..nu).map(|i| weights[i * nv + j]).collect()) };
        let column = |i: usize| -> (Vec<Vec3>, Vec<f64>) { (points[i * nv..(i + 1) * nv].to_vec(), weights[i * nv..(i + 1) * nv].to_vec()) };
        let along_u = |(c, w): (Vec<Vec3>, Vec<f64>)| Spline::new(p, knots.0.clone(), c, w);
        let along_v = |(c, w): (Vec<Vec3>, Vec<f64>)| Spline::new(q, knots.1.clone(), c, w);
        // A side that has shrunk to a point is a pole, and has no curve to search.
        let raw = [(row(0), true), (column(nu - 1), false), (row(nv - 1), true), (column(0), false)];
        let size = points.iter().fold(0.0f64, |m, c| m.max(c.dist(points[0])));
        let mut sides = Vec::with_capacity(4);
        let mut poles = Vec::new();
        for ((c, w), is_u) in raw {
            if c.iter().all(|q| q.dist(c[0]) <= 1e-9 * size.max(1.0)) {
                poles.push(c[0]);
                sides.push(None);
                continue;
            }
            let s = if is_u { along_u((c, w)) } else { along_v((c, w)) };
            let s = match s {
                Ok(s) => s,
                Err(e) => return bad(e.to_string()),
            };
            let pts = s.points();
            let (mut lo, mut hi) = (pts[0], pts[0]);
            for q in pts {
                lo = lo.min(*q);
                hi = hi.max(*q);
            }
            sides.push(Some((s, lo, hi)));
        }

        // Bezier pieces: along u, then along v.
        let mut along: Vec<Vec<Hom>> = (0..nv).map(|j| (0..nu).map(|i| Hom::of(points[i * nv + j], weights[i * nv + j])).collect()).collect();
        let breaks_u = extract(p, &knots.0, &mut along);
        let full_u = along[0].len();
        let mut across: Vec<Vec<Hom>> = (0..full_u).map(|i| along.iter().map(|l| l[i]).collect()).collect();
        let breaks_v = extract(q, &knots.1, &mut across);
        let mut cells = Vec::with_capacity((breaks_u.len() - 1) * (breaks_v.len() - 1));
        for a in 0..breaks_u.len() - 1 {
            for b in 0..breaks_v.len() - 1 {
                let mut h = Vec::with_capacity((p + 1) * (q + 1));
                for i in 0..=p {
                    h.extend_from_slice(&across[a * p + i][b * q..=b * q + q]);
                }
                cells.push(BezierCell::new((breaks_u[a], breaks_u[a + 1]), (breaks_v[b], breaks_v[b + 1]), p, q, h));
            }
        }

        let levels = leaf_levels(cells.len());
        for c in &mut cells {
            c.levels = levels;
        }
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        let (mut speed_u, mut speed_v) = (0.0f64, 0.0f64);
        for c in &cells {
            lo = lo.min(c.lo);
            hi = hi.max(c.hi);
            speed_u = speed_u.max(c.speed.0 / (c.u.1 - c.u.0));
            speed_v = speed_v.max(c.speed.1 / (c.v.1 - c.v.0));
        }
        let scale = (std::f64::consts::SQRT_2 * speed_u, std::f64::consts::SQRT_2 * speed_v);

        // No normal anywhere means a pole or a fold, and the sign and the chart both need one. Sampled, along
        // with the tightest curve, which sets how fine a trace over this surface has to be.
        let mut min_radius = f64::INFINITY;
        // At most this many samples a side per cell. A surface with many small cells gets fewer per cell, down to a limit,
        // as they're that much closer together already: a helix would otherwise spend most of its time here.
        const MOST: usize = 12;
        let samples = ((12.0 * 12.0 * 8.0 / cells.len() as f64).sqrt() as usize).clamp(4, MOST);
        let near_pole = |q: Vec3| poles.iter().any(|p: &Vec3| p.dist(q) <= 1e-6 * size.max(1.0));
        // What a look at a cell finds at a given spacing: the tightest bend, or why it can't be used.
        let scan = |c: &BezierCell, n: usize| -> std::result::Result<f64, String> {
            let mut tightest = f64::INFINITY;
            // A fold is narrow and the normal doesn't have to come near zero at any sample, but it does turn over
            // between two close ones, which no smooth surface does at this spacing.
            let mut last_row = [Vec3::ZERO; MOST + 1];
            for a in 0..=n {
                for b in 0..=n {
                    let q = c.local(a as f64 / n as f64, b as f64 / n as f64);
                    let unit_n = q.ds.cross(q.dt);
                    let (e, f, g) = (q.ds.dot(q.ds), q.ds.dot(q.dt), q.dt.dot(q.dt));
                    let area2 = e * g - f * f;
                    // A pole has no normal, and that's all right: it's a point, and what's around it has one.
                    if near_pole(q.p) {
                        continue;
                    }
                    let (u, v) = (c.u.0 + (c.u.1 - c.u.0) * a as f64 / n as f64, c.v.0 + (c.v.1 - c.v.0) * b as f64 / n as f64);
                    if !(area2 > 1e-14 * e * g) || !(e > 0.0) || !(g > 0.0) {
                        return Err(format!("it has no normal near u = {u:.6}, v = {v:.6}, which means a pole or a fold, and those aren't supported yet"));
                    }
                    let unit = unit_n.normalized();
                    if unit.dot(last_row[b]) < -0.5 || (b > 0 && unit.dot(last_row[b - 1]) < -0.5) {
                        return Err(format!("it folds over itself near u = {u:.6}, v = {v:.6}, which means a pole or a fold, and those aren't supported yet"));
                    }
                    last_row[b] = unit;
                    let (l, m, r) = (q.dss.dot(unit), q.dst.dot(unit), q.dtt.dot(unit));
                    let mean = (e * r - 2.0 * f * m + g * l) / (2.0 * area2);
                    let gauss = (l * r - m * m) / area2;
                    let bend = mean.abs() + (mean * mean - gauss).max(0.0).sqrt();
                    if bend > 1e-12 {
                        tightest = tightest.min(1.0 / bend);
                    }
                }
            }
            Ok(tightest)
        };
        for c in &cells {
            // A cell that looks wrong from a distance gets a closer look before anything is refused, since a sharp twist looks
            // like a fold if the samples are far enough apart.
            let found = scan(c, samples).or_else(|why| if samples < MOST { scan(c, MOST) } else { Err(why) });
            match found {
                Ok(tightest) => min_radius = min_radius.min(tightest),
                Err(why) => return bad(why),
            }
        }

        Ok(Patch {
            id: NEXT_ID.fetch_add(1, Ordering::Relaxed),
            degree,
            knots,
            count,
            points,
            weights,
            cells,
            breaks: (breaks_u, breaks_v),
            sides,
            moved_poles: poles.clone(),
            pole_normals: Vec::new(),
            poles,
            scale,
            lo,
            hi,
            min_radius,
            offset: 0.0,
        })
    }

    /// This surface moved `d` along its normal, further than it already is.
    ///
    /// That's exact, and no fit: a point of the moved surface is `S + d n`. The nearest point on it to anything is straight
    /// off the nearest point of `S`, so the certified search below still does all the work, and the signed distance is the
    /// one to `S`, less `d`. It never overstates, since moving a point by some distance can't change its distance from
    /// anything by more than that. Refused where the surface would fold, past a center of curvature, or where it has a
    /// crease, whose offset would leave a gap or an overlap.
    pub fn moved_by(&self, d: f64) -> Result<Patch> {
        let bad = |why: String| Err(Error::InvalidInput(format!("moving a free-form surface by {d}: {why}")));
        let mut out = Patch::new(self.degree, self.knots.clone(), self.count, self.points.clone(), self.weights.clone())?;
        out.offset = self.offset + d;
        let total = out.offset;
        if total == 0.0 {
            return Ok(out);
        }
        const SAMPLES: usize = 9;
        // A crease at a knot.
        for (which, list) in [&self.breaks.0, &self.breaks.1].iter().enumerate() {
            let other = if which == 0 { &self.breaks.1 } else { &self.breaks.0 };
            for &k in &list[1..list.len() - 1] {
                let (a, b) = (other[0], other[other.len() - 1]);
                for i in 0..=SAMPLES {
                    let at = a + (b - a) * i as f64 / SAMPLES as f64;
                    let (lo, hi) = if which == 0 { (self.cell_normal(k, at, true, false), self.cell_normal(k, at, false, false)) } else { (self.cell_normal(at, k, false, true), self.cell_normal(at, k, false, false)) };
                    if lo.dot(hi) < 1.0 - 1e-9 {
                        return bad("it has a sharp crease, whose offset would need a gap filled".into());
                    }
                }
            }
        }
        // A fold: the moved surface's own normal turns against this one's.
        for cell in &self.cells {
            for a in 0..=SAMPLES {
                for b in 0..=SAMPLES {
                    let (u, v) = (cell.u.0 + (cell.u.1 - cell.u.0) * a as f64 / SAMPLES as f64, cell.v.0 + (cell.v.1 - cell.v.0) * b as f64 / SAMPLES as f64);
                    let (_, du, dv) = out.derivs(u, v);
                    let n = self.normal_at(u, v);
                    if self.base_at_pole(self.base_point(u, v)) {
                        continue;
                    }
                    let (bu, bv) = self.base_derivs(u, v);
                    if du.cross(dv).dot(n) <= 1e-6 * bu.cross(bv).norm() {
                        return bad(format!("that passes a center of curvature near u = {u:.4}, v = {v:.4}, where the surface would fold over"));
                    }
                }
            }
        }
        // A pole stays a point only if the normal around it is one direction.
        let ((u0, u1), (v0, v1)) = self.domain();
        let mut moved = Vec::with_capacity(self.poles.len());
        for (i, side) in self.sides.iter().enumerate() {
            if side.is_some() {
                continue;
            }
            let along = |t: f64| match i {
                0 => (u0 + (u1 - u0) * t, v0),
                1 => (u1, v0 + (v1 - v0) * t),
                2 => (u0 + (u1 - u0) * t, v1),
                _ => (u0, v0 + (v1 - v0) * t),
            };
            let (u, v) = along(0.5);
            let at = self.base_point(u, v);
            // All round it from just inside. The tilt there is in proportion to how far inside, so two distances give the
            // limit, and if it has one normal, those all agree.
            let inside = |t: f64, by: f64| -> Vec3 {
                let (u, v) = along(t);
                let (u, v) = match i {
                    0 => (u, v0 + (v1 - v0) * by),
                    1 => (u1 - (u1 - u0) * by, v),
                    2 => (u, v1 - (v1 - v0) * by),
                    _ => (u0 + (u1 - u0) * by, v),
                };
                let (a, b) = self.base_derivs(u, v);
                a.cross(b).normalized()
            };
            let around: Vec<Vec3> = (0..=8)
                .map(|k| {
                    let t = k as f64 / 8.0;
                    (inside(t, 1e-5) * 2.0 - inside(t, 2e-5)).normalized()
                })
                .collect();
            let mean = around.iter().fold(Vec3::ZERO, |sum, n| sum + *n).normalized();
            if around.iter().any(|n| n.dot(mean) < 1.0 - 1e-6) {
                return bad("it has a pole with a point to it, whose offset would need a gap filled".into());
            }
            moved.push(at + mean * total);
            out.pole_normals.push((at, mean));
        }
        out.moved_poles = moved;
        out.lo = out.lo - Vec3::splat(total.abs());
        out.hi = out.hi + Vec3::splat(total.abs());
        // Moving out along a bend stretches it by at most this.
        let stretch = 1.0 + total.abs() / self.min_radius.max(1e-9);
        out.scale = (out.scale.0 * stretch, out.scale.1 * stretch);
        out.min_radius = (self.min_radius - total.abs()).max(self.min_radius * 0.1);
        Ok(out)
    }

    /// How far it's moved along its normal from what its control points define.
    pub fn offset(&self) -> f64 {
        self.offset
    }

    /// The normal from one side of a knot, for looking for creases.
    fn cell_normal(&self, u: f64, v: f64, before_u: bool, before_v: bool) -> Vec3 {
        let find = |list: &[f64], x: f64, before: bool| {
            let k = list.partition_point(|b| *b <= x).clamp(1, list.len() - 1) - 1;
            if before && k > 0 && (x - list[k]).abs() < 1e-12 { k - 1 } else { k }
        };
        let (a, b) = (find(&self.breaks.0, u, before_u), find(&self.breaks.1, v, before_v));
        let cell = &self.cells[a * (self.breaks.1.len() - 1) + b];
        // A little way in from the knot along the other direction too, so that a pole's neighbourhood still has a normal.
        let s = ((u - cell.u.0) / (cell.u.1 - cell.u.0)).clamp(1e-6, 1.0 - 1e-6);
        let t = ((v - cell.v.0) / (cell.v.1 - cell.v.0)).clamp(1e-6, 1.0 - 1e-6);
        let (_, ds, dt) = cell.first(s, t);
        ds.cross(dt).normalized()
    }

    pub fn degree(&self) -> (usize, usize) {
        self.degree
    }

    pub fn knots(&self) -> (&[f64], &[f64]) {
        (&self.knots.0, &self.knots.1)
    }

    pub fn count(&self) -> (usize, usize) {
        self.count
    }

    pub fn points(&self) -> &[Vec3] {
        &self.points
    }

    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    pub fn is_polynomial(&self) -> bool {
        self.weights.iter().all(|w| *w == self.weights[0])
    }

    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        let (u, v) = (&self.knots.0, &self.knots.1);
        ((u[0], u[u.len() - 1]), (v[0], v[v.len() - 1]))
    }

    /// A box holding everything between these parameters, from the control points, so it's certain. A face
    /// can bulge past its own edges here, so the edges alone don't give this.
    pub fn bounds_over(&self, u: (f64, f64), v: (f64, f64)) -> (Vec3, Vec3) {
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        let room = Vec3::splat(self.offset.abs());
        for cell in &self.cells {
            if cell.u.1 < u.0 || cell.u.0 > u.1 || cell.v.1 < v.0 || cell.v.0 > v.1 {
                continue;
            }
            // All of this cell: its own control points bound it, without building anything for a search.
            if cell.u.0 >= u.0 && cell.u.1 <= u.1 && cell.v.0 >= v.0 && cell.v.1 <= v.1 {
                lo = lo.min(cell.lo - room);
                hi = hi.max(cell.hi + room);
                continue;
            }
            let (du, dv) = (cell.u.1 - cell.u.0, cell.v.1 - cell.v.0);
            for leaf in cell.leaves() {
                let (a, b) = (cell.u.0 + du * leaf.s.0, cell.u.0 + du * leaf.s.1);
                let (c, d) = (cell.v.0 + dv * leaf.t.0, cell.v.0 + dv * leaf.t.1);
                if b >= u.0 && a <= u.1 && d >= v.0 && c <= v.1 {
                    lo = lo.min(leaf.lo - room);
                    hi = hi.max(leaf.hi + room);
                }
            }
        }
        (lo, hi)
    }

    /// The tightest bend, as a radius, sampled. It guides how fine a trace is, and isn't a proof.
    pub fn min_radius(&self) -> f64 {
        self.min_radius
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        (self.lo, self.hi)
    }

    /// Whether a point is at one of this surface's poles, where a whole side has shrunk to a point.
    pub fn at_pole(&self, p: Vec3) -> bool {
        self.moved_poles.iter().any(|q| q.dist(p) <= 1e-7)
    }

    fn base_at_pole(&self, p: Vec3) -> bool {
        self.poles.iter().any(|q| q.dist(p) <= 1e-7)
    }

    /// The normal at these parameters. At a pole there isn't one, so it's the limit from a little way in.
    fn normal_at(&self, u: f64, v: f64) -> Vec3 {
        let (du, dv) = self.base_derivs(u, v);
        let n = du.cross(dv);
        let at = self.base_point(u, v);
        if n.norm() > 1e-12 * du.norm().max(dv.norm()).max(1e-300) * du.norm().max(dv.norm()) && !self.base_at_pole(at) {
            return n.normalized();
        }
        if let Some((_, known)) = self.pole_normals.iter().find(|(q, _)| q.dist(at) <= 1e-7) {
            return *known;
        }
        let ((u0, u1), (v0, v1)) = self.domain();
        let (a, b) = self.base_derivs(u + (0.5 * (u0 + u1) - u) * 1e-6, v + (0.5 * (v0 + v1) - v) * 1e-6);
        a.cross(b).normalized()
    }

    fn base_point(&self, u: f64, v: f64) -> Vec3 {
        let (cell, s, t) = self.cell_at(u, v);
        cell.point(s, t)
    }

    fn base_derivs(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        let (cell, s, t) = self.cell_at(u, v);
        let (_, ds, dt) = cell.first(s, t);
        (ds / (cell.u.1 - cell.u.0), dt / (cell.v.1 - cell.v.0))
    }

    /// The same surface reflected in local y, with `v` running the other way, which makes its normal the
    /// reflection of this one's.
    pub fn reflected_y(&self) -> Result<Patch> {
        let (nu, nv) = self.count;
        let mut points = Vec::with_capacity(self.points.len());
        let mut weights = Vec::with_capacity(self.weights.len());
        for i in 0..nu {
            for j in (0..nv).rev() {
                let c = self.points[i * nv + j];
                points.push(v3(c.x, -c.y, c.z));
                weights.push(self.weights[i * nv + j]);
            }
        }
        let (v0, v1) = self.domain().1;
        let knots_v: Vec<f64> = self.knots.1.iter().rev().map(|k| v0 + v1 - k).collect();
        let plain = Patch::new(self.degree, (self.knots.0.clone(), knots_v), self.count, points, weights)?;
        if self.offset != 0.0 {
            plain.moved_by(self.offset)
        } else {
            Ok(plain)
        }
    }

    /// Whether its two `u` sides, or `v` sides, are the same curve in space, so that it closes up.
    pub fn closes_up(&self, along_v: bool) -> bool {
        let (a, b) = if along_v { (&self.sides[0], &self.sides[2]) } else { (&self.sides[3], &self.sides[1]) };
        let (Some((a, ..)), Some((b, ..))) = (a, b) else {
            return false;
        };
        let (t0, t1) = a.domain();
        let size = self.lo.dist(self.hi);
        (0..=16).all(|k| {
            let t = t0 + (t1 - t0) * k as f64 / 16.0;
            a.eval(t).dist(b.eval(t)) <= 1e-7 * size.max(1.0)
        })
    }

    /// The curve at a fixed `u`, running along `v`, or at a fixed `v` running along `u`.
    pub fn iso_curve(&self, at: f64, along_v_at_u: bool) -> Result<Spline> {
        if self.offset != 0.0 {
            return Err(Error::Unsupported("a line of constant parameter on an offset surface isn't a spline".into()));
        }
        let (nu, nv) = self.count;
        let (degree, knots, count, other_degree, other_knots) = if along_v_at_u {
            (self.degree.0, &self.knots.0, nu, self.degree.1, &self.knots.1)
        } else {
            (self.degree.1, &self.knots.1, nv, self.degree.0, &self.knots.0)
        };
        let span = find_knot_span(knots, degree, at, count);
        let basis = basis_functions(knots, degree, span, at);
        let lines = if along_v_at_u { nv } else { nu };
        let (mut points, mut weights) = (Vec::with_capacity(lines), Vec::with_capacity(lines));
        for line in 0..lines {
            let mut sum = Hom::ZERO;
            for (b, f) in basis.iter().enumerate() {
                let k = span - degree + b;
                let index = if along_v_at_u { k * nv + line } else { line * nv + k };
                let q = Hom::of(self.points[index], self.weights[index]);
                sum.a += q.a * *f;
                sum.w += q.w * f;
            }
            points.push(sum.point());
            weights.push(sum.w);
        }
        Spline::new(other_degree, other_knots.clone(), points, weights)
    }

    /// The part of the surface between these parameters, keeping its parameters, as a patch of its own.
    pub fn restricted(&self, u: (f64, f64), v: (f64, f64)) -> Result<Patch> {
        let ((u0, u1), (v0, v1)) = self.domain();
        if !(u.0 >= u0 && u.1 <= u1 && u.1 > u.0 && v.0 >= v0 && v.1 <= v1 && v.1 > v.0) {
            return Err(Error::InvalidInput(format!("free-form surface: {u:?} by {v:?} isn't inside its range")));
        }
        let (nu, nv) = self.count;
        let mut along: Vec<Vec<Hom>> = (0..nv).map(|j| (0..nu).map(|i| Hom::of(self.points[i * nv + j], self.weights[i * nv + j])).collect()).collect();
        let (knots_u, keep_u) = cut_lines(self.degree.0, &self.knots.0, u, &mut along);
        let across_count = along[0].len();
        let mut across: Vec<Vec<Hom>> = (keep_u.0..=keep_u.1).map(|i| along.iter().map(|l| l[i]).collect()).collect();
        let _ = across_count;
        let (knots_v, keep_v) = cut_lines(self.degree.1, &self.knots.1, v, &mut across);
        let (mut points, mut weights) = (Vec::new(), Vec::new());
        for line in &across {
            for q in &line[keep_v.0..=keep_v.1] {
                points.push(q.point());
                weights.push(q.w);
            }
        }
        let plain = Patch::new(self.degree, (knots_u, knots_v), (across.len(), keep_v.1 - keep_v.0 + 1), points, weights)?;
        if self.offset != 0.0 {
            plain.moved_by(self.offset)
        } else {
            Ok(plain)
        }
    }

    fn cell_at(&self, u: f64, v: f64) -> (&BezierCell, f64, f64) {
        let find = |list: &[f64], x: f64| list.partition_point(|b| *b <= x).clamp(1, list.len() - 1) - 1;
        let (a, b) = (find(&self.breaks.0, u), find(&self.breaks.1, v));
        let cell = &self.cells[a * (self.breaks.1.len() - 1) + b];
        (cell, (u - cell.u.0) / (cell.u.1 - cell.u.0), (v - cell.v.0) / (cell.v.1 - cell.v.0))
    }

    /// The point at these parameters, which must be in the domain.
    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        let (cell, s, t) = self.cell_at(u, v);
        let p = cell.point(s, t);
        if self.offset == 0.0 {
            return p;
        }
        p + self.normal_at(u, v) * self.offset
    }

    /// The point and its derivatives with respect to `u` and `v`.
    pub fn derivs(&self, u: f64, v: f64) -> (Vec3, Vec3, Vec3) {
        let (cell, s, t) = self.cell_at(u, v);
        let (p, ds, dt) = cell.first(s, t);
        let (du, dv) = (ds / (cell.u.1 - cell.u.0), dt / (cell.v.1 - cell.v.0));
        if self.offset == 0.0 {
            return (p, du, dv);
        }
        // The normal's rate of change, by differences within this cell, which are good to about ten digits here.
        let step = 1e-5;
        let normal = |s: f64, t: f64| -> Vec3 {
            let (_, a, b) = cell.first(s, t);
            let n = a.cross(b);
            if n.norm() > 1e-12 * a.norm() * b.norm() {
                n.normalized()
            } else {
                Vec3::ZERO
            }
        };
        let (s0, s1) = ((s - step).max(0.0), (s + step).min(1.0));
        let (t0, t1) = ((t - step).max(0.0), (t + step).min(1.0));
        let (na, nb) = (normal(s0, t), normal(s1, t));
        let (nc, ne) = (normal(s, t0), normal(s, t1));
        let (nu, nv) = if na == Vec3::ZERO || nb == Vec3::ZERO || nc == Vec3::ZERO || ne == Vec3::ZERO {
            // At a pole the surface isn't regular, and it stays a point when moved.
            (Vec3::ZERO, Vec3::ZERO)
        } else {
            ((nb - na) / ((s1 - s0) * (cell.u.1 - cell.u.0)), (ne - nc) / ((t1 - t0) * (cell.v.1 - cell.v.0)))
        };
        (p + self.normal_at(u, v) * self.offset, du + nu * self.offset, dv + nv * self.offset)
    }

    /// The closest point of the patch, found with certainty as the module's notes describe.
    pub fn foot(&self, p: Vec3) -> PatchFoot {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        if let Some((id, at, foot)) = LAST.with(|c| c.get()) {
            if id == self.id && at == key {
                return foot;
            }
        }
        let foot = self.search(p);
        let foot = if self.offset == 0.0 { foot } else { self.moved_foot(p, foot) };
        LAST.with(|c| c.set(Some((self.id, key, foot))));
        foot
    }

    /// The nearest point on the moved surface, from the nearest on the one it was moved from. The distance goes by where
    /// that point is. What a search that ran out of budget could prove about the unmoved surface is no use here, as the
    /// difference between that and the offset would look like a real gap.
    fn moved_foot(&self, p: Vec3, foot: PatchFoot) -> PatchFoot {
        let at = foot.at + foot.normal * self.offset;
        PatchFoot { at, distance: p.dist(at), ..foot }
    }

    fn search(&self, p: Vec3) -> PatchFoot {
        let mut best = (f64::INFINITY, 0.0, 0.0);
        let mut budget = SEARCH_BUDGET;
        // What a search that ran out of budget can still prove.
        let mut proven = f64::INFINITY;
        // Most queries are points on the surface: along a face's own edges, or checking what was just built. So first Newton
        // from a few places in the cells that could hold it, which needs nothing built, and a helix's or a fine loft's hundreds
        // of cells then only ever build their search structures if something needs them.
        let mut root = None;
        let mut minimum = false;
        // A point on the surface is in its own cell's box, so only those are worth trying, the nearest first.
        let mut first: Option<(&BezierCell, f64)> = None;
        let mut holding: Vec<(&BezierCell, f64)> = Vec::new();
        for cell in &self.cells {
            let d = cell.center.dist(p);
            if first.is_none_or(|x| d < x.1) {
                first = Some((cell, d));
            }
            if box_distance(p, cell.lo, cell.hi) <= TOL + self.offset.abs() {
                holding.push((cell, d));
            }
        }
        holding.sort_by(|a, b| a.1.total_cmp(&b.1));
        for &(cell, _) in holding.iter().take(4) {
            let (mut near, mut at) = (f64::INFINITY, (0.5, 0.5));
            for a in 0..3 {
                for b in 0..3 {
                    let (s, t) = (0.5 * a as f64, 0.5 * b as f64);
                    let d = consider(cell, s, t, p, &mut best);
                    if d < near {
                        near = d;
                        at = (s, t);
                    }
                }
            }
            if let Some((s, t)) = cell.newton(at.0, at.1, p) {
                consider(cell, s, t, p, &mut best);
                root = Some((std::ptr::from_ref(cell), s, t));
                minimum = cell.is_minimum(s, t, p);
                if best.0 < TOL {
                    break;
                }
            }
        }
        if !(best.0 < TOL) && !(root.is_some() && self.offset != 0.0 && (best.0 - self.offset.abs()).abs() < TOL && minimum) {
            // A close first answer rules most of the patch out straight away: the nearest cell's nearest leaf, by
            // its center, and Newton from there.
            if let Some((cell, _)) = first {
                if let Some(leaf) = cell.leaves().iter().min_by(|a, b| a.center.dist(p).total_cmp(&b.center.dist(p))) {
                    let (s, t) = ((leaf.s.0 + leaf.s.1) / 2.0, (leaf.t.0 + leaf.t.1) / 2.0);
                    consider(cell, s, t, p, &mut best);
                    if let Some((s, t)) = cell.newton(s, t, p) {
                        consider(cell, s, t, p, &mut best);
                        root = Some((std::ptr::from_ref(cell), s, t));
                        minimum = cell.is_minimum(s, t, p);
                    }
                }
            }
        }
        // Within tolerance, nothing else can be closer in any way that counts, since two sheets of one surface
        // that close aren't something this kernel can have. This is how every point on the surface goes, such as
        // those along a face's own edges, and it's most queries while building something.
        if root.is_some() && best.0 < TOL {
            let at = self.base_point(best.1, best.2);
            return PatchFoot { u: best.1, v: best.2, at, normal: self.normal_at(best.1, best.2), distance: best.0, boundary: false };
        }
        // The same for a surface moved by a distance: a point on it is that far straight off the unmoved one. A nearer part
        // of the unmoved surface would mean the moved one had run into itself.
        if root.is_some() && self.offset != 0.0 && (best.0 - self.offset.abs()).abs() < TOL {
            let at = self.base_point(best.1, best.2);
            let normal = self.normal_at(best.1, best.2);
            // Close to a pole, every direction around it is about the same distance away, so only a true minimum will do.
            if minimum && (p - at).dot(normal) * self.offset > 0.0 {
                return PatchFoot { u: best.1, v: best.2, at, normal, distance: best.0, boundary: false };
            }
        }
        // A boundary point closer than the best so far is in a leaf on that side whose box is closer too.
        let ((u0, u1), (v0, v1)) = self.domain();
        let mut near_side = [false; 4];
        for cell in &self.cells {
            if box_distance(p, cell.lo, cell.hi) < best.0 {
                let known = root.filter(|r| std::ptr::eq(r.0, cell)).map(|r| (r.1, r.2));
                for leaf in cell.leaves() {
                    if leaf.distance(p) < best.0 {
                        near_side[0] |= leaf.t.0 == 0.0 && cell.v.0 == v0;
                        near_side[1] |= leaf.s.1 == 1.0 && cell.u.1 == u1;
                        near_side[2] |= leaf.t.1 == 1.0 && cell.v.1 == v1;
                        near_side[3] |= leaf.s.0 == 0.0 && cell.u.0 == u0;
                        self.search_leaf(cell, leaf, p, known, &mut best, &mut budget, &mut proven);
                    }
                }
            }
        }
        for (k, entry) in self.sides.iter().enumerate() {
            let Some((side, lo, hi)) = entry else { continue };
            if near_side[k] && box_distance(p, *lo, *hi) < best.0 {
                let (a, b) = side.domain();
                let (t, d) = side.closest(p, a, b);
                // A stationary point right on the boundary is the same point, and better conditioned.
                if d < best.0 * (1.0 - 1e-12) - 1e-15 {
                    let (u, v) = match k {
                        0 => (t, v0),
                        1 => (u1, t),
                        2 => (t, v1),
                        _ => (u0, t),
                    };
                    best = (d, u, v);
                }
            }
        }
        let at = self.base_point(best.1, best.2);
        let normal = self.normal_at(best.1, best.2);
        // Where the surface is stationary, the offset is along the normal. Anything else is a boundary point,
        // whichever way it was reached, and there the height above the tangent plane is what's used.
        let r = p - at;
        let boundary = (r - normal * r.dot(normal)).norm() > 1e-8 * (1.0 + r.norm());
        PatchFoot { u: best.1, v: best.2, at, normal, distance: best.0.min(proven.max(0.0)), boundary }
    }

    #[allow(clippy::too_many_arguments)]
    fn search_leaf(&self, cell: &BezierCell, leaf: &Leaf, p: Vec3, known: Option<(f64, f64)>, best: &mut (f64, f64, f64), budget: &mut usize, proven: &mut f64) {
        let at = |c: &Grid, v: &Grid<Vec3>| Grid { r: c.r, c: c.c, a: c.a.iter().zip(&v.a).map(|(c, v)| c - p.dot(*v)).collect() };
        let (gs, gt) = (at(&leaf.cs, &leaf.vs), at(&leaf.ct, &leaf.vt));
        if gs.one_sign() || gt.one_sign() {
            return;
        }
        let mut stack = vec![(leaf.s.0, leaf.s.1, leaf.t.0, leaf.t.1, gs, gt, 0u32)];
        while let Some((s0, s1, t0, t1, gs, gt, depth)) = stack.pop() {
            let (s, t) = ((s0 + s1) / 2.0, (t0 + t1) / 2.0);
            let d = consider(cell, s, t, p, best);
            // Nothing in the box is closer than this, given how fast the surface can move.
            let reach = cell.speed.0 * (s1 - s0) / 2.0 + cell.speed.1 * (t1 - t0) / 2.0;
            if d - reach >= best.0 || gs.one_sign() || gt.one_sign() {
                continue;
            }
            if *budget == 0 {
                *proven = proven.min(d - reach);
                continue;
            }
            *budget -= 1;
            let slopes = Slopes::of(&gs, &gt);
            if slopes.no_root(&gs, &gt) {
                continue;
            }
            // At most one root in here. If Newton finds one in the box, that's it.
            if slopes.regular() {
                if known.is_some_and(|(a, b)| a >= s0 && a <= s1 && b >= t0 && b <= t1) {
                    continue;
                }
                if let Some((a, b)) = cell.newton(s, t, p) {
                    consider(cell, a, b, p, best);
                    if a >= s0 && a <= s1 && b >= t0 && b <= t1 {
                        continue;
                    }
                }
            }
            if depth >= SEARCH_DEPTH {
                *proven = proven.min(d - reach);
                continue;
            }
            // Halve the side that moves the surface further.
            if cell.speed.0 * (s1 - s0) >= cell.speed.1 * (t1 - t0) {
                let ((sa, sb), (ta, tb)) = (gs.halves(false), gt.halves(false));
                stack.push((s0, s, t0, t1, sa, ta, depth + 1));
                stack.push((s, s1, t0, t1, sb, tb, depth + 1));
            } else {
                let ((sa, sb), (ta, tb)) = (gs.halves(true), gt.halves(true));
                stack.push((s0, s1, t0, t, sa, ta, depth + 1));
                stack.push((s0, s1, t, t1, sb, tb, depth + 1));
            }
        }
    }

    /// Signed distance, positive on the side the normal points to. Past the boundary, the height above the
    /// tangent plane at the nearest boundary point, which is what carries the surface on smoothly.
    pub fn signed_distance(&self, p: Vec3) -> f64 {
        let foot = self.foot(p);
        let height = (p - foot.at).dot(foot.normal);
        if foot.boundary {
            height
        } else if height < 0.0 {
            -foot.distance
        } else {
            foot.distance
        }
    }

    pub fn normal(&self, p: Vec3) -> Vec3 {
        self.foot(p).normal
    }

    /// How far a point near the surface is from it, by Newton's method from a chart position that's already
    /// close. No proof that it's the nearest part of the surface, which a mesh checking its own sag doesn't
    /// need, and it's a good deal faster than one.
    pub fn distance_near(&self, uv: Vec2, p: Vec3) -> f64 {
        let ((u0, u1), (v0, v1)) = self.domain();
        let (mut u, mut v) = ((uv.x / self.scale.0).clamp(u0, u1), (uv.y / self.scale.1).clamp(v0, v1));
        // Newton over the whole patch, so that a point just over a knot line carries on into the next cell.
        for _ in 0..16 {
            let (cell, s, t) = self.cell_at(u, v);
            let q = cell.local(s, t);
            let r = q.p - p;
            let (g1, g2) = (r.dot(q.ds), r.dot(q.dt));
            let (a, b, d) = (q.ds.dot(q.ds) + r.dot(q.dss), q.ds.dot(q.dt) + r.dot(q.dst), q.dt.dot(q.dt) + r.dot(q.dtt));
            let det = a * d - b * b;
            if det == 0.0 || !det.is_finite() {
                break;
            }
            let (dx, dy) = ((d * g1 - b * g2) / det, (a * g2 - b * g1) / det);
            let (nu, nv) = ((u - dx * (cell.u.1 - cell.u.0)).clamp(u0, u1), (v - dy * (cell.v.1 - cell.v.0)).clamp(v0, v1));
            let moved = (nu - u).abs() / (u1 - u0) + (nv - v).abs() / (v1 - v0);
            (u, v) = (nu, nv);
            if moved < 1e-13 {
                let (cell, s, t) = self.cell_at(u, v);
                let q = cell.point(s, t);
                if self.offset == 0.0 {
                    return q.dist(p);
                }
                let (_, ds, dt) = cell.first(s, t);
                let n = ds.cross(dt);
                if n.norm() > 1e-12 * ds.norm() * dt.norm() {
                    return ((p - q).dot(n.normalized()) - self.offset).abs();
                }
                break;
            }
        }
        self.signed_distance(p).abs()
    }

    /// The normal at a chart position in the domain.
    pub fn normal_at_chart(&self, uv: Vec2) -> Vec3 {
        if self.offset != 0.0 {
            let ((u0, u1), (v0, v1)) = self.domain();
            return self.normal_at((uv.x / self.scale.0).clamp(u0, u1), (uv.y / self.scale.1).clamp(v0, v1));
        }
        let (_, dx, dy) = self.from_chart(uv);
        dx.cross(dy).normalized()
    }

    /// How many Gauss points integrate one chart direction of a flux or area exactly, or nearly.
    pub fn quadrature_points(&self) -> usize {
        if self.is_polynomial() && self.offset == 0.0 {
            // A flux density is a polynomial of three times the degree here, and an n-point rule is exact to 2n - 1.
            (3 * self.degree.0 + 2) / 2 + 1
        } else {
            16
        }
    }

    /// Chart position: `u` and `v` in units that keep 3D distance at or below chart distance. Past the
    /// boundary, by the tangent plane.
    pub fn to_chart(&self, p: Vec3) -> Vec2 {
        let foot = self.foot(p);
        let (mut u, mut v) = (foot.u, foot.v);
        if foot.boundary {
            let (_, du, dv) = self.derivs(u, v);
            let r = p - foot.at;
            let (e, f, g) = (du.dot(du), du.dot(dv), dv.dot(dv));
            let (a, b) = (r.dot(du), r.dot(dv));
            let det = e * g - f * f;
            u += (a * g - b * f) / det;
            v += (e * b - f * a) / det;
        }
        v2(u * self.scale.0, v * self.scale.1)
    }

    /// The point at a chart position and its derivatives along chart x and y. Past the boundary the surface
    /// carries on along its tangent plane there.
    pub fn from_chart(&self, uv: Vec2) -> (Vec3, Vec3, Vec3) {
        let ((u0, u1), (v0, v1)) = self.domain();
        let (u, v) = (uv.x / self.scale.0, uv.y / self.scale.1);
        let (cu, cv) = (u.clamp(u0, u1), v.clamp(v0, v1));
        let (q, du, dv) = self.derivs(cu, cv);
        let (over_u, over_v) = (u - cu, v - cv);
        // Past one side, the tangent it carries on along turns as you go along that side, which is a real part of the
        // surface's slope there. Past a corner, the tangent plane is flat.
        let (mut along_u, mut along_v) = (du, dv);
        if over_v != 0.0 && over_u == 0.0 {
            let h = 1e-5 * (u1 - u0);
            let (a, b) = ((cu - h).max(u0), (cu + h).min(u1));
            let turn = (self.derivs(b, cv).2 - self.derivs(a, cv).2) / (b - a);
            along_u = du + turn * over_v;
        } else if over_u != 0.0 && over_v == 0.0 {
            let h = 1e-5 * (v1 - v0);
            let (a, b) = ((cv - h).max(v0), (cv + h).min(v1));
            let turn = (self.derivs(cu, b).1 - self.derivs(cu, a).1) / (b - a);
            along_v = dv + turn * over_u;
        }
        (q + du * over_u + dv * over_v, along_u / self.scale.0, along_v / self.scale.1)
    }

    /// The knots strictly inside the range, along `u` and along `v`.
    pub fn interior_knots(&self) -> [&[f64]; 2] {
        let inner = |list: &'_ Vec<f64>| -> std::ops::Range<usize> { 1..list.len() - 1 };
        [&self.breaks.0[inner(&self.breaks.0)], &self.breaks.1[inner(&self.breaks.1)]]
    }

    /// Whether a point, in the patch's own coordinates, is inside the box of any of its cells, with this much room.
    pub fn boxes_hold(&self, p: Vec3, room: f64) -> bool {
        let room = room + self.offset.abs();
        self.cells.iter().any(|c| p.x >= c.lo.x - room && p.x <= c.hi.x + room && p.y >= c.lo.y - room && p.y <= c.hi.y + room && p.z >= c.lo.z - room && p.z <= c.hi.z + room)
    }

    /// The stretch of a ray, in the patch's own coordinates, that passes through the boxes of its cells, if any does.
    pub fn ray_span(&self, p: Vec3, d: Vec3, room: f64) -> Option<(f64, f64)> {
        let room = room + self.offset.abs();
        let mut span: Option<(f64, f64)> = None;
        for c in &self.cells {
            let (mut enter, mut leave) = (f64::NEG_INFINITY, f64::INFINITY);
            let mut hit = true;
            for (o, v, lo, hi) in [(p.x, d.x, c.lo.x - room, c.hi.x + room), (p.y, d.y, c.lo.y - room, c.hi.y + room), (p.z, d.z, c.lo.z - room, c.hi.z + room)] {
                if v.abs() < 1e-300 {
                    if o < lo || o > hi {
                        hit = false;
                        break;
                    }
                } else {
                    let (a, b) = ((lo - o) / v, (hi - o) / v);
                    enter = enter.max(a.min(b));
                    leave = leave.min(a.max(b));
                }
            }
            if hit && enter <= leave {
                span = Some(span.map_or((enter, leave), |s| (s.0.min(enter), s.1.max(leave))));
            }
        }
        span
    }

    /// The chart position of these parameters.
    pub fn chart_of(&self, u: f64, v: f64) -> Vec2 {
        v2(u * self.scale.0, v * self.scale.1)
    }

    /// The parameters at a chart position.
    pub fn parameters_at(&self, uv: Vec2) -> (f64, f64) {
        (uv.x / self.scale.0, uv.y / self.scale.1)
    }

    /// The chart x positions of the knots, where a smooth rule of integration has to stop.
    pub fn chart_breaks_x(&self) -> Vec<f64> {
        self.breaks.0.iter().map(|u| u * self.scale.0).collect()
    }
}

fn consider(cell: &BezierCell, s: f64, t: f64, p: Vec3, best: &mut (f64, f64, f64)) -> f64 {
    let d = cell.point(s, t).dist(p);
    if d < best.0 {
        *best = (d, cell.u.0 + (cell.u.1 - cell.u.0) * s, cell.v.0 + (cell.v.1 - cell.v.0) * t);
    }
    d
}

/// Control points and knots for an interpolation through values at these parameters: cubic where there are
/// enough, and with knots by averaging, as `Spline::through` does.
pub(crate) fn interpolation_knots(params: &[f64]) -> (usize, Vec<f64>) {
    let n = params.len();
    let degree = 3.min(n - 1);
    let mut knots = vec![params[0]; degree + 1];
    for j in 1..n - degree {
        let sum: f64 = params[j..j + degree].iter().sum();
        knots.push(sum / degree as f64);
    }
    knots.extend(vec![params[n - 1]; degree + 1]);
    (degree, knots)
}

/// Solves for the control points of interpolations through many lines of values at one set of parameters. The
/// matrix is banded and totally positive, so elimination needs no pivoting and each line costs little.
pub(crate) struct Collocation {
    n: usize,
    band: usize,
    rows: Vec<Vec<f64>>,
    multipliers: Vec<Vec<f64>>,
}

impl Collocation {
    pub(crate) fn new(params: &[f64], degree: usize, knots: &[f64]) -> Option<Collocation> {
        let n = params.len();
        let band = degree;
        let mut rows = vec![vec![0.0; 2 * band + 1]; n];
        for (k, row) in rows.iter_mut().enumerate() {
            let span = find_knot_span(knots, degree, params[k], n);
            for (b, value) in basis_functions(knots, degree, span, params[k]).iter().enumerate() {
                let column = span - degree + b;
                row[column + band - k] = *value;
            }
        }
        let mut multipliers = vec![vec![0.0; band]; n];
        for k in 0..n {
            let pivot = rows[k][band];
            if pivot.abs() < 1e-14 {
                return None;
            }
            for below in k + 1..(k + band + 1).min(n) {
                let offset = below - k;
                let f = rows[below][band - offset] / pivot;
                multipliers[k][offset - 1] = f;
                if f != 0.0 {
                    for c in 0..=band {
                        let value = rows[k][band + c];
                        rows[below][band - offset + c] -= f * value;
                    }
                }
            }
        }
        Some(Collocation { n, band, rows, multipliers })
    }

    pub(crate) fn solve(&self, values: &[Hom]) -> Vec<Hom> {
        let (n, band) = (self.n, self.band);
        let mut rhs = values.to_vec();
        for k in 0..n {
            for below in k + 1..(k + band + 1).min(n) {
                let f = self.multipliers[k][below - k - 1];
                let q = rhs[k];
                rhs[below].a -= q.a * f;
                rhs[below].w -= q.w * f;
            }
        }
        let mut out = vec![Hom::ZERO; n];
        for k in (0..n).rev() {
            let mut sum = rhs[k];
            for c in 1..=band {
                if k + c < n {
                    sum.a -= out[k + c].a * self.rows[k][band + c];
                    sum.w -= out[k + c].w * self.rows[k][band + c];
                }
            }
            let d = self.rows[k][band];
            out[k] = Hom { a: sum.a / d, w: sum.w / d };
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bicubic B-spline over a wavy height field, with several knot intervals each way.
    fn wavy() -> Patch {
        let (nu, nv) = (6, 5);
        let mut points = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (x, y) = (i as f64 * 8.0, j as f64 * 8.0);
                points.push(v3(x, y, 3.0 * (x * 0.31).sin() + 2.0 * (y * 0.4 + x * 0.1).cos()));
            }
        }
        let weights = vec![1.0; nu * nv];
        let knots_u = vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.5, 4.0, 4.0, 4.0, 4.0];
        let knots_v = vec![0.0, 0.0, 0.0, 0.0, 1.5, 3.0, 3.0, 3.0, 3.0];
        Patch::new((3, 3), (knots_u, knots_v), (nu, nv), points, weights).unwrap()
    }

    /// The same shape with uneven weights.
    fn wavy_rational() -> Patch {
        let p = wavy();
        let weights: Vec<f64> = (0..p.points.len()).map(|k| 1.0 + 0.7 * ((k as f64) * 1.3).sin().abs()).collect();
        Patch::new(p.degree, p.knots.clone(), p.count, p.points.clone(), weights).unwrap()
    }

    /// An exact piece of a torus, as a product of two rational quadratic arcs: major radius `r` over a
    /// quarter turn around z, minor radius `a` over a quarter turn around the tube.
    fn torus_piece(r: f64, a: f64) -> Patch {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        // The tube section's control points, as distance from the axis and height.
        let tube = [(r + a, 0.0, 1.0), (r + a, a, w), (r, a, 1.0)];
        // The sweep's control points on the unit circle.
        let sweep = [(1.0, 0.0, 1.0), (1.0, 1.0, w), (0.0, 1.0, 1.0)];
        let (mut points, mut weights) = (Vec::new(), Vec::new());
        // u runs around the axis and v around the tube, which puts the normal outward from the tube.
        for &(x, y, wb) in &sweep {
            for &(rho, h, wa) in &tube {
                points.push(v3(rho * x, rho * y, h));
                weights.push(wa * wb);
            }
        }
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        Patch::new((2, 2), (knots.clone(), knots), (3, 3), points, weights).unwrap()
    }

    /// De Boor's algorithm over both directions, written apart from the Bezier cells, so that it checks them.
    fn de_boor(p: &Patch, u: f64, v: f64) -> Vec3 {
        let (nu, nv) = p.count;
        let (du, dv) = p.degree;
        let (su, sv) = (find_knot_span(&p.knots.0, du, u, nu), find_knot_span(&p.knots.1, dv, v, nv));
        let (bu, bv) = (basis_functions(&p.knots.0, du, su, u), basis_functions(&p.knots.1, dv, sv, v));
        let (mut a, mut w) = (Vec3::ZERO, 0.0);
        for (i, x) in bu.iter().enumerate() {
            for (j, y) in bv.iter().enumerate() {
                let k = (su - du + i) * nv + sv - dv + j;
                let f = x * y * p.weights[k];
                a += p.points[k] * f;
                w += f;
            }
        }
        a / w
    }

    /// The true nearest point by brute force, without any of the search: a fine grid, then a shrinking one.
    fn brute(p: &Patch, q: Vec3) -> f64 {
        let ((u0, u1), (v0, v1)) = p.domain();
        let mut best = (f64::INFINITY, u0, v0);
        let n = 240;
        for i in 0..=n {
            for j in 0..=n {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64);
                let d = de_boor(p, u, v).dist(q);
                if d < best.0 {
                    best = (d, u, v);
                }
            }
        }
        let (mut hu, mut hv) = ((u1 - u0) / n as f64, (v1 - v0) / n as f64);
        for _ in 0..40 {
            let (cu, cv) = (best.1, best.2);
            for i in -3i32..=3 {
                for j in -3i32..=3 {
                    let (u, v) = ((cu + hu * i as f64).clamp(u0, u1), (cv + hv * j as f64).clamp(v0, v1));
                    let d = de_boor(p, u, v).dist(q);
                    if d < best.0 {
                        best = (d, u, v);
                    }
                }
            }
            hu *= 0.5;
            hv *= 0.5;
        }
        best.0
    }

    #[test]
    fn the_bezier_cells_are_the_same_surface_as_the_b_spline() {
        for p in [wavy(), wavy_rational()] {
            let ((u0, u1), (v0, v1)) = p.domain();
            for i in 0..=23 {
                for j in 0..=19 {
                    let (u, v) = (u0 + (u1 - u0) * i as f64 / 23.0, v0 + (v1 - v0) * j as f64 / 19.0);
                    assert!(p.eval(u, v).dist(de_boor(&p, u, v)) < 1e-11, "{u} {v}");
                }
            }
        }
    }

    #[test]
    fn derivatives_match_differences() {
        let p = wavy_rational();
        let cell = &p.cells[3];
        let h = 1e-5;
        for (s, t) in [(0.3, 0.2), (0.77, 0.5), (0.1, 0.9)] {
            let q = cell.local(s, t);
            let a = cell.local(s + h, t);
            let b = cell.local(s, t + h);
            let (m, n) = (cell.local(s - h, t), cell.local(s, t - h));
            assert!((q.ds - (a.p - m.p) / (2.0 * h)).norm() < 1e-6);
            assert!((q.dt - (b.p - n.p) / (2.0 * h)).norm() < 1e-6);
            assert!((q.dss - (a.ds - m.ds) / (2.0 * h)).norm() < 1e-5);
            assert!((q.dst - (a.dt - m.dt) / (2.0 * h)).norm() < 1e-5);
            assert!((q.dtt - (b.dt - n.dt) / (2.0 * h)).norm() < 1e-5);
        }
    }

    #[test]
    fn the_stationary_polynomials_are_what_they_say() {
        // Evaluated directly, each should be (S - p) . S_s, up to a positive power of the weight.
        let p = wavy_rational();
        let cell = &p.cells[1];
        let poly = cell.stationary();
        let q = v3(7.0, 3.0, 5.0);
        let (s, t) = (0.37, 0.61);
        let value = |c: &[f64], v: &[Vec3], (r, n): (usize, usize)| {
            let (bs, _) = (basis(r - 1, s), 0);
            let bt = basis(n - 1, t);
            let mut sum = 0.0;
            for i in 0..r {
                for j in 0..n {
                    sum += (c[i * n + j] - q.dot(v[i * n + j])) * bs[i] * bt[j];
                }
            }
            sum
        };
        let local = cell.local(s, t);
        let w: f64 = {
            let (bs, bt) = (basis(cell.m, s), basis(cell.n, t));
            (0..=cell.m).flat_map(|i| (0..=cell.n).map(move |j| (i, j))).map(|(i, j)| cell.h[i * (cell.n + 1) + j].w * bs[i] * bt[j]).sum()
        };
        let expected_s = (local.p - q).dot(local.ds) * w * w * w;
        let expected_t = (local.p - q).dot(local.dt) * w * w * w;
        assert!((value(&poly.cs, &poly.vs, poly.s) - expected_s).abs() < 1e-8 * expected_s.abs().max(1.0));
        assert!((value(&poly.ct, &poly.vt, poly.t) - expected_t).abs() < 1e-8 * expected_t.abs().max(1.0));
    }

    fn basis(degree: usize, s: f64) -> Vec<f64> {
        (0..=degree).map(|i| binomial(degree, i) * s.powi(i as i32) * (1.0 - s).powi((degree - i) as i32)).collect()
    }

    #[test]
    fn a_grid_halves_into_the_same_polynomial() {
        let g: Grid = Grid { r: 3, c: 4, a: (0..12).map(|k| ((k * k) as f64 * 0.37).sin()).collect() };
        let value = |g: &Grid, s: f64, t: f64| {
            let (bs, bt) = (basis(g.r - 1, s), basis(g.c - 1, t));
            let mut sum = 0.0;
            for i in 0..g.r {
                for j in 0..g.c {
                    sum += g.a[i * g.c + j] * bs[i] * bt[j];
                }
            }
            sum
        };
        let (a, b) = g.halves(false);
        let (c, d) = g.halves(true);
        for (s, t) in [(0.2, 0.3), (0.7, 0.9), (0.45, 0.05)] {
            assert!((value(&a, s, t) - value(&g, s / 2.0, t)).abs() < 1e-12);
            assert!((value(&b, s, t) - value(&g, 0.5 + s / 2.0, t)).abs() < 1e-12);
            assert!((value(&c, s, t) - value(&g, s, t / 2.0)).abs() < 1e-12);
            assert!((value(&d, s, t) - value(&g, s, 0.5 + t / 2.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn a_point_on_the_surface_is_at_distance_zero_wherever_the_search_halves() {
        for p in [wavy(), wavy_rational()] {
            let ((u0, u1), (v0, v1)) = p.domain();
            for i in 0..=32 {
                for j in 0..=32 {
                    let q = p.eval(u0 + (u1 - u0) * i as f64 / 32.0, v0 + (v1 - v0) * j as f64 / 32.0);
                    let d = p.foot(q).distance;
                    assert!(d < 1e-10, "{d} off at {i}, {j}");
                    assert!(p.signed_distance(q).abs() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn the_closest_point_agrees_with_brute_force() {
        let mut rng = Rng::new(11);
        for p in [wavy(), wavy_rational()] {
            let (lo, hi) = p.bounds();
            let mid = (lo + hi) * 0.5;
            for _ in 0..50 {
                let q = mid + v3((hi.x - lo.x) * rng.range(-0.8, 0.8), (hi.y - lo.y) * rng.range(-0.8, 0.8), rng.range(-14.0, 14.0));
                let (got, want) = (p.foot(q).distance, brute(&p, q));
                assert!((got - want).abs() < 1e-6, "{q:?}: found {got}, brute force {want}");
            }
        }
    }

    #[test]
    fn the_closest_point_is_right_when_a_point_is_near_the_surface_too() {
        // These are the queries where boxes beside the answer get ruled out in one step, so they're the ones
        // that would show a box wrongly discarded.
        let mut rng = Rng::new(21);
        for p in [wavy(), wavy_rational(), torus_piece(20.0, 5.0)] {
            let ((u0, u1), (v0, v1)) = p.domain();
            for _ in 0..80 {
                let on = p.eval(rng.range(u0, u1), rng.range(v0, v1));
                let q = on + rng.unit_vector() * rng.range(0.0, 0.05) * if rng.int(0, 1) == 0 { 1.0 } else { 0.02 };
                let (got, want) = (p.foot(q).distance, brute(&p, q));
                assert!((got - want).abs() < 1e-7, "{q:?}: found {got}, brute force {want}");
            }
        }
    }

    #[test]
    fn a_torus_written_as_a_patch_has_a_torus_distance() {
        let (r, a) = (20.0, 5.0);
        let p = torus_piece(r, a);
        let torus = crate::surface::Surface::Torus { frame: Frame::WORLD, major: r, minor: a };
        let mut rng = Rng::new(5);
        let mut interior = 0;
        for _ in 0..400 {
            // Around the quarter of the torus the patch covers.
            let (phi, theta, off) = (rng.range(0.0, 1.0) * PI / 2.0, rng.range(0.0, 1.0) * PI / 2.0, rng.range(-3.0, 3.0));
            let rho = r + (a + off) * theta.cos();
            let q = v3(rho * phi.cos(), rho * phi.sin(), (a + off) * theta.sin());
            let foot = p.foot(q);
            if foot.boundary {
                continue;
            }
            interior += 1;
            assert!((p.signed_distance(q) - torus.sdf(q)).abs() < 1e-9, "{q:?}");
            assert!(p.normal(q).dist(torus.normal(q)) < 1e-9);
            let on = foot.at;
            assert!(torus.sdf(on).abs() < 1e-11);
        }
        assert!(interior > 300, "{interior}");
    }

    #[test]
    fn the_value_never_overstates_the_distance_to_the_patch() {
        let mut rng = Rng::new(3);
        for p in [wavy(), wavy_rational(), torus_piece(20.0, 5.0)] {
            let (lo, hi) = p.bounds();
            let size = lo.dist(hi);
            for _ in 0..40 {
                let q = (lo + hi) * 0.5 + rng.unit_vector() * rng.range(0.0, size);
                let g = p.signed_distance(q);
                assert!(g.abs() <= brute(&p, q) + 1e-6, "{q:?}");
            }
        }
    }

    #[test]
    fn the_value_changes_no_faster_than_the_point_moves_near_the_surface() {
        // With a gentle surface this holds everywhere. Past the boundary it's the height above a tangent plane
        // that twists along the edge, so it gets a little room, and faces are never out there.
        let mut rng = Rng::new(8);
        let p = wavy();
        let (lo, hi) = p.bounds();
        for _ in 0..3000 {
            let a = (lo + hi) * 0.5 + v3((hi.x - lo.x) * rng.range(-0.45, 0.45), (hi.y - lo.y) * rng.range(-0.45, 0.45), rng.range(-2.0, 2.0));
            let b = a + rng.unit_vector() * rng.range(0.0, 1.5);
            let (ga, gb) = (p.signed_distance(a), p.signed_distance(b));
            assert!((ga - gb).abs() <= a.dist(b) * 1.001 + 1e-9, "{a:?} {b:?}: {ga} {gb}");
        }
    }

    #[test]
    fn the_surface_carries_on_smoothly_past_its_boundary() {
        // Walking straight along a tangent plane out of the patch, the value stays near zero and has no jump.
        let p = wavy();
        let ((_, u1), (v0, v1)) = p.domain();
        let v = (v0 + v1) / 2.0;
        let (at, du, dv) = p.derivs(u1, v);
        let n = du.cross(dv).normalized();
        let conormal = dv.cross(n).normalized();
        let out = if conormal.dot(du) > 0.0 { conormal } else { -conormal };
        let at_step = |step: f64| p.signed_distance(at + out * step + n * 0.25);
        assert!((at_step(0.0) - 0.25).abs() < 1e-9);
        // Each small step changes it by no more than the step, so there's no jump where the foot reaches the edge.
        for k in 0..100 {
            let (a, b) = (0.01 * k as f64, 0.01 * (k + 1) as f64);
            assert!((at_step(b) - at_step(a)).abs() <= 0.01 * 1.01, "{a}: {} then {}", at_step(a), at_step(b));
        }
        // And it stays close to the height it started at, as the tangent plane changes along the edge.
        assert!((at_step(1.0) - 0.25).abs() < 0.1, "{}", at_step(1.0));
    }

    #[test]
    fn the_chart_never_understates_distance_and_comes_back() {
        let mut rng = Rng::new(4);
        for p in [wavy(), wavy_rational(), torus_piece(20.0, 5.0)] {
            let ((u0, u1), (v0, v1)) = p.domain();
            for _ in 0..500 {
                let a = v2(rng.range(u0, u1) * p.scale.0, rng.range(v0, v1) * p.scale.1);
                let b = v2(rng.range(u0, u1) * p.scale.0, rng.range(v0, v1) * p.scale.1);
                assert!(p.from_chart(a).0.dist(p.from_chart(b).0) <= a.dist(b) * (1.0 + 1e-12));
                let back = p.to_chart(p.from_chart(a).0);
                assert!(p.from_chart(back).0.dist(p.from_chart(a).0) < 1e-8);
            }
        }
    }

    #[test]
    fn a_reflected_patch_has_the_reflected_normal() {
        let p = wavy_rational();
        let m = p.reflected_y().unwrap();
        for (x, y, z) in [(10.0, 9.0, 1.0), (22.0, 15.0, -3.0), (31.0, 4.0, 2.0)] {
            let q = v3(x, y, z);
            let image = v3(x, -y, z);
            assert!((m.signed_distance(image) - p.signed_distance(q)).abs() < 1e-9);
            let (a, b) = (p.normal(q), m.normal(image));
            assert!(v3(a.x, -a.y, a.z).dist(b) < 1e-9);
        }
    }

    #[test]
    fn a_restricted_patch_and_an_iso_curve_are_the_same_surface() {
        for p in [wavy(), wavy_rational(), torus_piece(20.0, 5.0)] {
            let ((u0, u1), (v0, v1)) = p.domain();
            let (ua, ub) = (u0 + (u1 - u0) * 0.23, u0 + (u1 - u0) * 0.8);
            let (va, vb) = (v0 + (v1 - v0) * 0.4, v1);
            let part = p.restricted((ua, ub), (va, vb)).unwrap();
            assert_eq!(part.domain(), ((ua, ub), (va, vb)));
            for i in 0..=9 {
                for j in 0..=9 {
                    let (u, v) = (ua + (ub - ua) * i as f64 / 9.0, va + (vb - va) * j as f64 / 9.0);
                    assert!(part.eval(u, v).dist(de_boor(&p, u, v)) < 1e-10, "{u} {v}");
                }
            }
            let at_u = p.iso_curve(ua, true).unwrap();
            let at_v = p.iso_curve(va, false).unwrap();
            for k in 0..=12 {
                let f = k as f64 / 12.0;
                let (u, v) = (u0 + (u1 - u0) * f, v0 + (v1 - v0) * f);
                assert!(at_u.eval(v).dist(de_boor(&p, ua, v)) < 1e-10);
                assert!(at_v.eval(u).dist(de_boor(&p, u, va)) < 1e-10);
            }
        }
    }

    #[test]
    fn a_surface_folded_over_itself_is_refused() {
        // A sheet that runs out along x, doubles back, and runs out again, like a letter z seen from the side,
        // so its normal turns over along two narrow lines without being zero at any sample.
        let row = [(0.0, 0.0), (14.0, 0.02), (-4.0, 0.04), (10.0, 0.06)];
        let mut points = Vec::new();
        for (x, z) in row {
            for j in 0..4 {
                points.push(v3(x, j as f64 * 3.0, z));
            }
        }
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let error = Patch::new((3, 3), (knots.clone(), knots), (4, 4), points, vec![1.0; 16]).err().expect("refused").to_string();
        assert!(error.contains("fold"), "{error}");
    }

    /// A piece of a true cone with its apex as a pole: a quarter turn, as a rational patch.
    fn cone_piece(radius: f64, height: f64) -> Patch {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let around = [(1.0, 0.0, 1.0), (1.0, 1.0, w), (0.0, 1.0, 1.0)];
        let (mut points, mut weights) = (Vec::new(), Vec::new());
        // u around the axis and v from the rim up to the apex, where every control point is one point.
        for &(x, y, wa) in &around {
            points.push(v3(x * radius, y * radius, 0.0));
            weights.push(wa);
            points.push(v3(0.0, 0.0, height));
            weights.push(wa);
        }
        Patch::new((2, 1), (vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]), (3, 2), points, weights).unwrap()
    }

    #[test]
    fn a_pole_is_a_point_with_a_surface_around_it() {
        let (r, h) = (8.0f64, 6.0f64);
        let p = cone_piece(r, h);
        let apex = v3(0.0, 0.0, h);
        assert!(p.at_pole(apex));
        assert!(!p.at_pole(v3(1.0, 1.0, h - 1.0)));
        // Points on it, right up to the pole, are at no distance from it.
        for v in [0.0, 0.3, 0.9, 0.999, 1.0] {
            for u in [0.1, 0.5, 0.9] {
                let q = p.eval(u, v);
                assert!(p.foot(q).distance < 1e-8, "{u} {v}");
            }
        }
        // Off it, the distance is the true cone's, and the side it's on is right.
        let cone = crate::surface::Surface::Cone { frame: Frame { origin: apex, x: -Vec3::X, y: Vec3::Y, z: -Vec3::Z }, radius: 0.0, half_angle: (r / h).atan() };
        let _ = cone;
        let slant = r.hypot(h);
        let outward = v3(h / slant * std::f64::consts::FRAC_1_SQRT_2, h / slant * std::f64::consts::FRAC_1_SQRT_2, r / slant);
        for t in [0.2, 0.5, 0.8] {
            let on = p.eval(0.5, t);
            let q = on + outward * 0.05;
            assert!((p.foot(q).distance - 0.05).abs() < 1e-6, "{t}: {}", p.foot(q).distance);
            assert!(p.normal(q).dist(outward) < 1e-6 || p.normal(q).dist(-outward) < 1e-6);
        }
        // Right at the pole it still has a normal to give.
        assert!(p.normal(apex).is_finite() && (p.normal(apex).norm() - 1.0).abs() < 1e-9);
        assert!(!p.closes_up(false) && !p.closes_up(true));
    }

    #[test]
    fn interpolation_passes_through_its_values() {
        let params = [0.0, 1.0, 2.5, 3.0, 4.5];
        let (degree, knots) = interpolation_knots(&params);
        let values: Vec<Hom> = params.iter().map(|t| Hom { a: v3(t * t, t.sin(), 1.0 - t), w: 1.0 + 0.1 * t }).collect();
        let solved = Collocation::new(&params, degree, &knots).unwrap().solve(&values);
        for (k, t) in params.iter().enumerate() {
            let span = find_knot_span(&knots, degree, *t, params.len());
            let (mut a, mut w) = (Vec3::ZERO, 0.0);
            for (b, f) in basis_functions(&knots, degree, span, *t).iter().enumerate() {
                let q = solved[span - degree + b];
                a += q.a * *f;
                w += q.w * f;
            }
            assert!((a - values[k].a).norm() < 1e-11 && (w - values[k].w).abs() < 1e-11);
        }
    }
}
