//! Vectors, frames, bounding boxes, quadrature and the kernel's tolerance.

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};
use std::sync::OnceLock;

/// Linear tolerance in model units (millimetres by convention).
///
/// Two points closer than this are the same point. It sits about eight orders
/// of magnitude above `f64` rounding for parts up to a metre across, which
/// leaves room for accumulated error, and well below any machining tolerance.
pub const TOL: f64 = 1e-7;

/// Smallest feature the kernel will build. Anything shorter than this is
/// rejected, because features near `TOL` cannot be told apart reliably.
pub const MIN_FEATURE: f64 = 10.0 * TOL;
/// The farthest any coordinate may be from the origin: 10 km. Doubles still resolve about
/// 2e-9 mm out there, safely below `TOL`. Beyond it the tolerances stop meaning anything,
/// so operations refuse the input.
pub const MAX_COORD: f64 = 1e7;

pub const PI: f64 = std::f64::consts::PI;
pub const TAU: f64 = std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v2(x: f64, y: f64) -> Vec2 {
    Vec2 { x, y }
}

pub const fn v3(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3 { x, y, z }
}

impl Vec2 {
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }
    pub fn normalized(self) -> Vec2 {
        self / self.norm()
    }
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).norm()
    }
    /// This vector turned 90 degrees clockwise.
    pub fn right(self) -> Vec2 {
        v2(self.y, -self.x)
    }
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
}

impl Vec3 {
    pub const X: Vec3 = v3(1.0, 0.0, 0.0);
    pub const Y: Vec3 = v3(0.0, 1.0, 0.0);
    pub const Z: Vec3 = v3(0.0, 0.0, 1.0);
    pub const ZERO: Vec3 = v3(0.0, 0.0, 0.0);

    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        v3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    pub fn norm2(self) -> f64 {
        self.dot(self)
    }
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }
    pub fn normalized(self) -> Vec3 {
        self / self.norm()
    }
    pub fn dist(self, o: Vec3) -> f64 {
        (self - o).norm()
    }
    pub fn lerp(self, o: Vec3, t: f64) -> Vec3 {
        self + (o - self) * t
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Any unit vector perpendicular to this one.
    pub fn any_perpendicular(self) -> Vec3 {
        let a = if self.x.abs() < 0.6 { Vec3::X } else { Vec3::Y };
        self.cross(a).normalized()
    }
}

macro_rules! impl_ops {
    ($t:ident, $($f:ident),+) => {
        impl Add for $t { type Output = $t; fn add(self, o: $t) -> $t { $t { $($f: self.$f + o.$f),+ } } }
        impl Sub for $t { type Output = $t; fn sub(self, o: $t) -> $t { $t { $($f: self.$f - o.$f),+ } } }
        impl Neg for $t { type Output = $t; fn neg(self) -> $t { $t { $($f: -self.$f),+ } } }
        impl Mul<f64> for $t { type Output = $t; fn mul(self, s: f64) -> $t { $t { $($f: self.$f * s),+ } } }
        impl Div<f64> for $t { type Output = $t; fn div(self, s: f64) -> $t { $t { $($f: self.$f / s),+ } } }
        impl AddAssign for $t { fn add_assign(&mut self, o: $t) { $(self.$f += o.$f;)+ } }
        impl SubAssign for $t { fn sub_assign(&mut self, o: $t) { $(self.$f -= o.$f;)+ } }
    };
}
impl_ops!(Vec2, x, y);
impl_ops!(Vec3, x, y, z);

/// A right-handed orthonormal frame. Also serves as a rigid transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

/// A reflection across a plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mirror {
    pub point: Vec3,
    /// Unit normal of the plane.
    pub normal: Vec3,
}

impl Mirror {
    pub fn new(point: Vec3, normal: Vec3) -> Mirror {
        Mirror { point, normal: normal.normalized() }
    }

    pub fn of_point(&self, p: Vec3) -> Vec3 {
        p - self.normal * (2.0 * (p - self.point).dot(self.normal))
    }

    pub fn of_dir(&self, d: Vec3) -> Vec3 {
        d - self.normal * (2.0 * d.dot(self.normal))
    }

    /// The image of a surface's frame. Its z axis reflects, so an axis and the normal a surface
    /// defines by it land where they should. A reflection turns right hand into left, so y is
    /// worked out again.
    pub fn of_surface_frame(&self, f: &Frame) -> Frame {
        let (x, z) = (self.of_dir(f.x), self.of_dir(f.z));
        Frame { origin: self.of_point(f.origin), x, y: z.cross(x), z }
    }

    /// The image of a curve's frame. Its x and y axes reflect, so a parameter still means the same
    /// point. Here z is what's worked out again.
    pub fn of_curve_frame(&self, f: &Frame) -> Frame {
        let (x, y) = (self.of_dir(f.x), self.of_dir(f.y));
        Frame { origin: self.of_point(f.origin), x, y, z: x.cross(y) }
    }
}

impl Frame {
    pub const WORLD: Frame = Frame { origin: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };

    /// Frame with the given Z axis; X is `x_hint` made perpendicular to it.
    pub fn new(origin: Vec3, z: Vec3, x_hint: Vec3) -> Frame {
        let z = z.normalized();
        let mut x = x_hint - z * x_hint.dot(z);
        if x.norm() < 1e-9 {
            x = z.any_perpendicular();
        }
        let x = x.normalized();
        Frame { origin, x, y: z.cross(x), z }
    }

    /// Frame with the given Z axis and an arbitrary X axis.
    pub fn from_z(origin: Vec3, z: Vec3) -> Frame {
        Frame::new(origin, z, z.any_perpendicular())
    }

    pub fn translation(t: Vec3) -> Frame {
        Frame { origin: t, ..Frame::WORLD }
    }

    /// Rotation by `angle` radians about the line through `origin` along `axis`.
    pub fn rotation(origin: Vec3, axis: Vec3, angle: f64) -> Frame {
        let k = axis.normalized();
        let (s, c) = angle.sin_cos();
        let rot = |v: Vec3| v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c));
        Frame { origin: origin - rot(origin), x: rot(Vec3::X), y: rot(Vec3::Y), z: rot(Vec3::Z) }
    }

    pub fn to_local(&self, p: Vec3) -> Vec3 {
        let d = p - self.origin;
        v3(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    pub fn to_world(&self, l: Vec3) -> Vec3 {
        self.origin + self.x * l.x + self.y * l.y + self.z * l.z
    }

    pub fn dir_to_world(&self, d: Vec3) -> Vec3 {
        self.x * d.x + self.y * d.y + self.z * d.z
    }

    pub fn point(&self, x: f64, y: f64, z: f64) -> Vec3 {
        self.to_world(v3(x, y, z))
    }

    /// `other` expressed through this rigid transform.
    pub fn compose(&self, other: &Frame) -> Frame {
        Frame {
            origin: self.to_world(other.origin),
            x: self.dir_to_world(other.x),
            y: self.dir_to_world(other.y),
            z: self.dir_to_world(other.z),
        }
    }
}

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: v3(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: v3(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };

    pub fn add(&mut self, p: Vec3) {
        self.min = v3(self.min.x.min(p.x), self.min.y.min(p.y), self.min.z.min(p.z));
        self.max = v3(self.max.x.max(p.x), self.max.y.max(p.y), self.max.z.max(p.z));
    }

    pub fn union(&mut self, o: &Aabb) {
        if o.min.x <= o.max.x {
            self.add(o.min);
            self.add(o.max);
        }
    }

    pub fn inflated(&self, d: f64) -> Aabb {
        let e = v3(d, d, d);
        Aabb { min: self.min - e, max: self.max + e }
    }

    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }

    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn diagonal(&self) -> f64 {
        if self.min.x > self.max.x {
            0.0
        } else {
            (self.max - self.min).norm()
        }
    }
}

/// Wraps an angle into `(-PI, PI]`.
pub fn wrap_pi(a: f64) -> f64 {
    let r = a.rem_euclid(TAU);
    if r > PI {
        r - TAU
    } else {
        r
    }
}

/// Picks a reference angle such that every given angle lies strictly inside
/// `(ref - PI, ref + PI)`, by centring opposite the largest gap between them.
pub fn angular_center(angles: &[f64]) -> f64 {
    if angles.is_empty() {
        return 0.0;
    }
    let mut a: Vec<f64> = angles.iter().map(|x| x.rem_euclid(TAU)).collect();
    a.sort_by(f64::total_cmp);
    let mut best_gap = a[0] + TAU - a[a.len() - 1];
    let mut best_end = a[0]; // the angle at which the occupied arc starts
    for w in a.windows(2) {
        let gap = w[1] - w[0];
        if gap > best_gap {
            best_gap = gap;
            best_end = w[1];
        }
    }
    let span = TAU - best_gap;
    wrap_pi(best_end + span / 2.0)
}

/// Gauss-Legendre nodes and weights on `[-1, 1]`.
pub fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
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
}

/// The fixed rule used for mass properties.
pub fn gauss_24() -> &'static [(f64, f64)] {
    static RULE: OnceLock<Vec<(f64, f64)>> = OnceLock::new();
    RULE.get_or_init(|| gauss_legendre(24))
}

/// Small deterministic generator (xoshiro256**). Fuzz tests must be replayable
/// from a seed on every platform, so they do not use system randomness.
#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let mut z = seed.wrapping_add(0x9E3779B97F4A7C15);
        let mut s = [0u64; 4];
        for x in &mut s {
            z = z.wrapping_add(0x9E3779B97F4A7C15);
            let mut v = z;
            v = (v ^ (v >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
            v = (v ^ (v >> 27)).wrapping_mul(0x94D049BB133111EB);
            *x = v ^ (v >> 31);
        }
        Rng { s }
    }

    pub fn next_u64(&mut self) -> u64 {
        let r = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        r
    }

    /// Uniform in `[0, 1)`.
    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.f64()
    }

    /// Uniform integer in `[lo, hi]`.
    pub fn int(&mut self, lo: i64, hi: i64) -> i64 {
        lo + (self.next_u64() % ((hi - lo + 1) as u64)) as i64
    }

    pub fn unit_vector(&mut self) -> Vec3 {
        loop {
            let v = v3(self.range(-1.0, 1.0), self.range(-1.0, 1.0), self.range(-1.0, 1.0));
            let n = v.norm();
            if n > 0.1 && n <= 1.0 {
                return v / n;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angular_center_contains_all() {
        for &(lo, span) in &[(0.1, 1.0), (3.0, 3.0), (-3.0, 5.5), (6.0, 6.0)] {
            let pts: Vec<f64> = (0..=10).map(|i| lo + span * i as f64 / 10.0).collect();
            let c = angular_center(&pts);
            for p in &pts {
                assert!(wrap_pi(p - c).abs() < PI - 0.1, "{lo} {span} {p} {c}");
            }
        }
    }

    #[test]
    fn gauss_rule_integrates_polynomials_and_cosine() {
        let rule = gauss_24();
        let weights: f64 = rule.iter().map(|r| r.1).sum();
        assert!((weights - 2.0).abs() < 1e-14);
        let integral: f64 = rule.iter().map(|&(x, w)| w * (x * PI).cos() * PI).sum();
        assert!(integral.abs() < 1e-13);
    }

    #[test]
    fn rotation_is_rigid() {
        let r = Frame::rotation(v3(1.0, 2.0, 3.0), v3(1.0, 1.0, 0.0), 0.7);
        assert!((r.x.cross(r.y) - r.z).norm() < 1e-14);
        assert!(r.to_world(v3(1.0, 2.0, 3.0)).dist(v3(1.0, 2.0, 3.0)) < 1e-14);
    }
}
