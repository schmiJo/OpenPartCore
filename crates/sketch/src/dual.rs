//! Numbers that carry their gradient with them, for equations whose derivatives would be long to write out
//! by hand and easy to get wrong. Each variable an equation uses starts with a one in its own slot, and
//! arithmetic follows the chain rule, so a gradient is exact to rounding, with no step size to choose.

use crate::system::MAX_VARS;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug)]
pub(crate) struct D {
    pub v: f64,
    pub g: [f64; MAX_VARS],
}

impl D {
    /// The equation's variable number `i`.
    pub fn var(v: f64, i: usize) -> D {
        let mut g = [0.0; MAX_VARS];
        g[i] = 1.0;
        D { v, g }
    }

    pub fn sqrt(self) -> D {
        let r = self.v.max(0.0).sqrt();
        if r < 1e-300 {
            return D { v: 0.0, g: [0.0; MAX_VARS] };
        }
        let k = 0.5 / r;
        let mut g = self.g;
        for x in &mut g {
            *x *= k;
        }
        D { v: r, g }
    }
}

macro_rules! with_each {
    ($a:expr, $b:expr, |$x:ident, $y:ident| $f:expr) => {{
        let mut g = [0.0; MAX_VARS];
        for i in 0..MAX_VARS {
            let ($x, $y) = ($a.g[i], $b.g[i]);
            g[i] = $f;
        }
        g
    }};
}

impl Add for D {
    type Output = D;
    fn add(self, o: D) -> D {
        D { v: self.v + o.v, g: with_each!(self, o, |x, y| x + y) }
    }
}

impl Sub for D {
    type Output = D;
    fn sub(self, o: D) -> D {
        D { v: self.v - o.v, g: with_each!(self, o, |x, y| x - y) }
    }
}

impl Mul for D {
    type Output = D;
    fn mul(self, o: D) -> D {
        let (a, b) = (self.v, o.v);
        D { v: a * b, g: with_each!(self, o, |x, y| x * b + a * y) }
    }
}

impl Div for D {
    type Output = D;
    fn div(self, o: D) -> D {
        let (a, b) = (self.v, o.v);
        D { v: a / b, g: with_each!(self, o, |x, y| (x * b - a * y) / (b * b)) }
    }
}

impl Mul<f64> for D {
    type Output = D;
    fn mul(self, k: f64) -> D {
        let mut g = self.g;
        for x in &mut g {
            *x *= k;
        }
        D { v: self.v * k, g }
    }
}

impl Neg for D {
    type Output = D;
    fn neg(self) -> D {
        self * -1.0
    }
}

/// A point or direction whose coordinates carry gradients.
#[derive(Clone, Copy, Debug)]
pub(crate) struct V2 {
    pub x: D,
    pub y: D,
}

impl V2 {
    pub fn vars(x: f64, y: f64, i: usize) -> V2 {
        V2 { x: D::var(x, i), y: D::var(y, i + 1) }
    }

    pub fn minus(self, o: V2) -> V2 {
        V2 { x: self.x - o.x, y: self.y - o.y }
    }

    pub fn dot(self, o: V2) -> D {
        self.x * o.x + self.y * o.y
    }

    pub fn cross(self, o: V2) -> D {
        self.x * o.y - self.y * o.x
    }

    /// Turned a quarter turn counter-clockwise.
    pub fn perp(self) -> V2 {
        V2 { x: -self.y, y: self.x }
    }

    pub fn scaled(self, k: D) -> V2 {
        V2 { x: self.x * k, y: self.y * k }
    }

    pub fn plus(self, o: V2) -> V2 {
        V2 { x: self.x + o.x, y: self.y + o.y }
    }

    pub fn unit(self) -> V2 {
        let n = self.dot(self).sqrt();
        if n.v < 1e-300 {
            return self;
        }
        V2 { x: self.x / n, y: self.y / n }
    }
}
