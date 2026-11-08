//! Finds the closed regions of a sketch: the areas a kernel profile can be made from.
//!
//! Lines, arcs, circles, ellipses and splines that aren't construction geometry are cut wherever they cross or touch,
//! at crossings and T-junctions alike. Every area those pieces enclose is a region, with an id that
//! survives solving and dragging. Pieces that enclose nothing, such as open curves and spurs, are
//! reported as ignored rather than refused. A connected group of curves inside a region is a hole in
//! it, and the areas inside that group are regions of their own, one level deeper.
//!
//! Faces are found by the usual walk: at each junction, leave by the next piece clockwise from the
//! one you arrived on, which keeps the area on your left. A walk that runs counter-clockwise goes
//! round one region. A walk that runs clockwise goes round the outside of a connected group.

use crate::model::*;
use crate::solve::{ErrorKind, SketchError};
use crate::spline::SplineShape;
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

/// Points this close count as one junction, and a curve's end this close to another curve lands on it.
/// Within the kernel's tolerance, and far above the solver's.
const JOIN: f64 = 1e-7;

/// A walk enclosing less than this is treated as having no area.
const MIN_AREA: f64 = 1e-9;

#[derive(Clone, Debug, PartialEq)]
pub enum SegKind {
    Line { a: P2, b: P2 },
    /// Runs from angle `a0` to `a1` in radians: counter-clockwise if `a1 > a0`, clockwise if not.
    /// A whole circle runs through a full turn.
    Arc { center: P2, radius: f64, a0: f64, a1: f64 },
    /// Part or all of an ellipse, by eccentric angle measured from the unit vector `x` along the major
    /// axis, with the same directions as an arc.
    Ellipse { center: P2, x: P2, major: f64, minor: f64, a0: f64, a1: f64 },
    /// Part of a spline, from parameter `t0` to `t1`, in whichever direction.
    Spline { shape: Arc<SplineShape>, t0: f64, t1: f64 },
}

/// A piece of a loop, and the entity it came from.
#[derive(Clone, Debug, PartialEq)]
pub struct Seg {
    pub entity: EntityId,
    pub kind: SegKind,
}

impl Seg {
    pub fn start(&self) -> P2 {
        match self.kind {
            SegKind::Line { a, .. } => a,
            SegKind::Arc { center, radius, a0, .. } => [center[0] + radius * a0.cos(), center[1] + radius * a0.sin()],
            SegKind::Ellipse { a0, .. } => point_at(&self.kind, a0),
            SegKind::Spline { t0, .. } => point_at(&self.kind, t0),
        }
    }

    pub fn end(&self) -> P2 {
        match self.kind {
            SegKind::Line { b, .. } => b,
            SegKind::Arc { center, radius, a1, .. } => [center[0] + radius * a1.cos(), center[1] + radius * a1.sin()],
            SegKind::Ellipse { a1, .. } => point_at(&self.kind, a1),
            SegKind::Spline { t1, .. } => point_at(&self.kind, t1),
        }
    }

    fn reversed(&self) -> Seg {
        let kind = match self.kind {
            SegKind::Line { a, b } => SegKind::Line { a: b, b: a },
            SegKind::Arc { center, radius, a0, a1 } => SegKind::Arc { center, radius, a0: a1, a1: a0 },
            SegKind::Ellipse { center, x, major, minor, a0, a1 } => SegKind::Ellipse { center, x, major, minor, a0: a1, a1: a0 },
            SegKind::Spline { ref shape, t0, t1 } => SegKind::Spline { shape: shape.clone(), t0: t1, t1: t0 },
        };
        Seg { entity: self.entity, kind }
    }

    /// The point halfway along.
    fn middle(&self) -> P2 {
        match self.kind {
            SegKind::Line { a, b } => [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0],
            SegKind::Arc { center, radius, a0, a1 } => {
                let (s, c) = ((a0 + a1) / 2.0).sin_cos();
                [center[0] + radius * c, center[1] + radius * s]
            }
            SegKind::Ellipse { a0, a1, .. } => point_at(&self.kind, (a0 + a1) / 2.0),
            SegKind::Spline { t0, t1, .. } => point_at(&self.kind, (t0 + t1) / 2.0),
        }
    }

    /// Direction of travel at the start, in radians, and how fast it turns counter-clockwise.
    fn leaving(&self) -> (f64, f64) {
        match self.kind {
            SegKind::Line { a, b } => ((b[1] - a[1]).atan2(b[0] - a[0]), 0.0),
            SegKind::Arc { radius, a0, a1, .. } => {
                let (s, c) = a0.sin_cos();
                if a1 >= a0 {
                    (c.atan2(-s), 1.0 / radius)
                } else {
                    ((-c).atan2(s), -1.0 / radius)
                }
            }
            SegKind::Ellipse { major, minor, a0, a1, .. } => {
                let d = velocity_at(&self.kind, a0);
                let speed = d[0].hypot(d[1]);
                let bend = major * minor / (speed * speed * speed);
                if a1 >= a0 {
                    (d[1].atan2(d[0]), bend)
                } else {
                    ((-d[1]).atan2(-d[0]), -bend)
                }
            }
            SegKind::Spline { t0, t1, .. } => {
                let d = velocity_at(&self.kind, t0);
                // The bend from how the direction changes a short way along.
                let step = (t1 - t0) * 1e-4;
                let e = velocity_at(&self.kind, t0 + step);
                let (s0, s1) = (d[0].hypot(d[1]).max(1e-300), e[0].hypot(e[1]).max(1e-300));
                let turn = (d[0] * e[1] - d[1] * e[0]) / (s0 * s1);
                let moved = step.abs() * s0;
                let bend = if moved > 0.0 { turn / moved } else { 0.0 };
                let sign = if t1 >= t0 { 1.0 } else { -1.0 };
                ((d[1] * sign).atan2(d[0] * sign), bend)
            }
        }
    }
}

/// A closed chain of segments, head to tail.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    pub segs: Vec<Seg>,
}

impl Loop {
    /// Positive when the loop runs counter-clockwise.
    pub fn signed_area(&self) -> f64 {
        let mut sum = 0.0;
        for s in &self.segs {
            sum += match s.kind {
                SegKind::Line { a, b } => a[0] * b[1] - a[1] * b[0],
                SegKind::Arc { center, radius, a0, a1 } => {
                    radius * radius * (a1 - a0)
                        + radius * (center[0] * (a1.sin() - a0.sin()) - center[1] * (a1.cos() - a0.cos()))
                }
                SegKind::Ellipse { center, x, major, minor, a0, a1 } => {
                    let y = [-x[1], x[0]];
                    let (cx, cy) = (center[0] * x[1] - center[1] * x[0], center[0] * y[1] - center[1] * y[0]);
                    major * minor * (a1 - a0) + major * cx * (a1.cos() - a0.cos()) + minor * cy * (a1.sin() - a0.sin())
                }
                SegKind::Spline { ref shape, t0, t1 } => shape.twice_area(t0, t1),
            };
        }
        sum / 2.0
    }

    /// Whether a point not on the loop is inside it, by exact winding number.
    fn contains(&self, p: P2) -> bool {
        let mut winding = 0.0;
        let turn = |a: P2, b: P2| {
            let (u, v) = ([a[0] - p[0], a[1] - p[1]], [b[0] - p[0], b[1] - p[1]]);
            (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1])
        };
        for s in &self.segs {
            match s.kind {
                SegKind::Line { a, b } => winding += turn(a, b),
                SegKind::Arc { center, radius, a0, a1 } => {
                    // In pieces of under a quarter turn. The angle a piece subtends equals its chord's,
                    // unless p lies between the chord and the arc, when it differs by a full turn.
                    let pieces = ((a1 - a0).abs() / (PI / 2.0)).ceil().max(1.0) as usize;
                    for k in 0..pieces {
                        let (b0, b1) = (a0 + (a1 - a0) * k as f64 / pieces as f64, a0 + (a1 - a0) * (k + 1) as f64 / pieces as f64);
                        let at = |t: f64| [center[0] + radius * t.cos(), center[1] + radius * t.sin()];
                        let (q0, q1) = (at(b0), at(b1));
                        winding += turn(q0, q1);
                        let inside_circle = (p[0] - center[0]).hypot(p[1] - center[1]) < radius;
                        // The side of the chord the arc bulges to.
                        let mid = at((b0 + b1) / 2.0);
                        let side = |q: P2| (q1[0] - q0[0]) * (q[1] - q0[1]) - (q1[1] - q0[1]) * (q[0] - q0[0]);
                        if inside_circle && side(p) * side(mid) > 0.0 {
                            winding += if b1 > b0 { TAU } else { -TAU };
                        }
                    }
                }
                SegKind::Spline { t0: a0, t1: a1, .. } | SegKind::Ellipse { a0, a1, .. } => {
                    // Chords, split finer wherever the point is close enough to be between chord and curve.
                    fn piece(kind: &SegKind, p: P2, t0: f64, t1: f64, depth: u32, turn: &dyn Fn(P2, P2) -> f64) -> f64 {
                        let (q0, q1, m) = (point_at(kind, t0), point_at(kind, t1), point_at(kind, (t0 + t1) / 2.0));
                        let reach = dist(q0, m).max(dist(q1, m));
                        if dist(p, m) > 1.5 * reach || depth == 0 {
                            turn(q0, q1)
                        } else {
                            piece(kind, p, t0, (t0 + t1) / 2.0, depth - 1, turn) + piece(kind, p, (t0 + t1) / 2.0, t1, depth - 1, turn)
                        }
                    }
                    let pieces = 16;
                    for k in 0..pieces {
                        let (b0, b1) = (a0 + (a1 - a0) * k as f64 / pieces as f64, a0 + (a1 - a0) * (k + 1) as f64 / pieces as f64);
                        winding += piece(&s.kind, p, b0, b1, 40, &turn);
                    }
                }
            }
        }
        winding.abs() > PI
    }
}

/// A region's permanent name, written `r` and twelve hex digits.
///
/// It comes from the curves around the region's outside and the direction each is walked in, not from
/// where they are or what order they were drawn in. So it survives solving, dragging and dimension
/// changes, and edits elsewhere in the sketch: a new hole doesn't rename the region around it, and a
/// spur added from outside doesn't either. Splitting a region makes new ids for its parts.
///
/// Lobes with the same curves around them, such as the parts of an ellipse sticking out of a circle,
/// are told apart by their order along the first curve. That order can only shift if a dimension
/// change carries a lobe past where a whole circle starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RegionId(pub u64);

impl RegionId {
    pub fn name(self) -> String {
        format!("r{:012x}", self.0)
    }

    pub fn parse(text: &str) -> Result<RegionId, String> {
        let digits = text.strip_prefix('r').filter(|d| d.len() == 12 && d.bytes().all(|b| b.is_ascii_hexdigit()));
        digits
            .and_then(|d| u64::from_str_radix(d, 16).ok())
            .map(RegionId)
            .ok_or_else(|| format!("\"{text}\" is not a region id; they look like r0123456789ab"))
    }
}

/// An area of the sketch: a counter-clockwise outer loop and clockwise holes.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub id: RegionId,
    /// How many connected groups of curves surround this one. Even depths are material by default
    /// and odd ones are holes, which is what nesting loops inside each other means.
    pub depth: u32,
    pub outer: Loop,
    pub holes: Vec<Loop>,
}

impl Region {
    /// The enclosed area, without the holes.
    pub fn area(&self) -> f64 {
        self.outer.signed_area() + self.holes.iter().map(Loop::signed_area).sum::<f64>()
    }

    /// Whether a point not on the boundary lies in this region.
    pub fn contains(&self, p: P2) -> bool {
        self.outer.contains(p) && !self.holes.iter().any(|h| h.contains(p))
    }
}

/// Why a piece of a curve is in no region.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WhyIgnored {
    /// It runs to a dead end at this point, directly or through other ignored pieces.
    FreeEnd { at: P2 },
    /// It joins two closed groups, with the same area on both sides, so it encloses nothing.
    Bridge,
    /// It runs along this other entity, which was kept.
    Overlap { with: EntityId },
    /// A line with no length, or a circle or arc with no radius.
    NoLength,
    /// It's part of a loop that encloses no area.
    NoArea,
}

/// Part or all of a curve that bounds no region. Construction geometry and points aren't listed: they
/// were never meant to.
#[derive(Clone, Debug, PartialEq)]
pub struct Ignored {
    pub entity: EntityId,
    /// True if none of this entity bounds a region. False if only these pieces of it don't.
    pub whole: bool,
    pub why: WhyIgnored,
    pub pieces: Vec<Seg>,
}

/// Every closed region a sketch's curves make, and what they leave over.
#[derive(Clone, Debug, PartialEq)]
pub struct Arrangement {
    pub regions: Vec<Region>,
    pub ignored: Vec<Ignored>,
}

impl Arrangement {
    /// The regions that are material when nothing has been picked: every other level of nesting.
    pub fn default_regions(&self) -> Vec<Region> {
        self.regions.iter().filter(|r| r.depth % 2 == 0).cloned().collect()
    }

    pub fn region(&self, id: RegionId) -> Option<&Region> {
        self.regions.iter().find(|r| r.id == id)
    }

    /// The region a point falls in, for picking by clicking. A point on a boundary gets either side.
    pub fn region_at(&self, p: P2) -> Option<&Region> {
        self.regions.iter().find(|r| r.contains(p))
    }
}

fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The point at a curve's parameter: a fraction on a line, an angle on an arc or ellipse.
fn point_at(kind: &SegKind, t: f64) -> P2 {
    match *kind {
        SegKind::Spline { ref shape, .. } => shape.eval(t).0,
        SegKind::Line { a, b } => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t],
        SegKind::Arc { center, radius, .. } => [center[0] + radius * t.cos(), center[1] + radius * t.sin()],
        SegKind::Ellipse { center, x, major, minor, .. } => {
            let (u, v) = (major * t.cos(), minor * t.sin());
            [center[0] + u * x[0] - v * x[1], center[1] + u * x[1] + v * x[0]]
        }
    }
}

/// How fast `point_at` moves with its parameter, going up.
fn velocity_at(kind: &SegKind, t: f64) -> P2 {
    match *kind {
        SegKind::Spline { ref shape, .. } => shape.eval(t).1,
        SegKind::Line { a, b } => [b[0] - a[0], b[1] - a[1]],
        SegKind::Arc { radius, .. } => [-radius * t.sin(), radius * t.cos()],
        SegKind::Ellipse { x, major, minor, .. } => {
            let (u, v) = (-major * t.sin(), minor * t.cos());
            [u * x[0] - v * x[1], u * x[1] + v * x[0]]
        }
    }
}

fn line_param(a: P2, b: P2, p: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
}

/// Where along a curve a point lies, if it's within `JOIN` of the curve: a fraction on a line and an
/// angle on an arc.
fn param_on(kind: &SegKind, p: P2) -> Option<f64> {
    match *kind {
        SegKind::Spline { ref shape, t0, t1 } => {
            // A scan along each knot interval, then Newton from every dip in the distance, since two strands
            // of a curve can pass close together and the nearest sample may be on the wrong one.
            let mut stops = vec![t0];
            for k in shape.breaks() {
                if k > t0 && k < t1 {
                    stops.push(k);
                }
            }
            stops.push(t1);
            let mut samples: Vec<(f64, f64)> = Vec::new();
            for w in stops.windows(2) {
                for i in 0..48 {
                    let t = w[0] + (w[1] - w[0]) * i as f64 / 48.0;
                    samples.push((t, dist(shape.eval(t).0, p)));
                }
            }
            samples.push((t1, dist(shape.eval(t1).0, p)));
            let (mut t, mut d) = (t0, f64::INFINITY);
            for i in 0..samples.len() {
                let before = if i > 0 { samples[i - 1].1 } else { f64::INFINITY };
                let after = if i + 1 < samples.len() { samples[i + 1].1 } else { f64::INFINITY };
                if samples[i].1 > before || samples[i].1 > after {
                    continue;
                }
                let mut u = samples[i].0;
                for _ in 0..16 {
                    let (q, v) = shape.eval(u);
                    let speed = v[0] * v[0] + v[1] * v[1];
                    if speed <= 0.0 {
                        break;
                    }
                    let step = ((p[0] - q[0]) * v[0] + (p[1] - q[1]) * v[1]) / speed;
                    u = (u + step).clamp(t0, t1);
                    if step.abs() < 1e-14 * (t1 - t0) {
                        break;
                    }
                }
                let here = dist(shape.eval(u).0, p);
                if here < d {
                    (t, d) = (u, here);
                }
            }
            (d <= JOIN).then_some(t)
        }
        SegKind::Line { a, b } => {
            let f = line_param(a, b, p);
            let foot = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
            let slack = JOIN / dist(a, b);
            (dist(foot, p) <= JOIN && f >= -slack && f <= 1.0 + slack).then(|| f.clamp(0.0, 1.0))
        }
        SegKind::Arc { center, radius, a0, a1 } => {
            if (dist(center, p) - radius).abs() > JOIN {
                return None;
            }
            let slack = JOIN / radius;
            let mut t = ((p[1] - center[1]).atan2(p[0] - center[0]) - a0).rem_euclid(TAU);
            if t > a1 - a0 + slack && t > TAU - slack {
                t -= TAU;
            }
            (t <= a1 - a0 + slack).then(|| a0 + t.clamp(0.0, a1 - a0))
        }
        SegKind::Ellipse { center, x, major, minor, a0, a1 } => {
            let d = [p[0] - center[0], p[1] - center[1]];
            let (u, v) = (d[0] * x[0] + d[1] * x[1], d[1] * x[0] - d[0] * x[1]);
            // Starting from where it would be on a circle, walk to the nearest point of the ellipse.
            let mut t = (v / minor).atan2(u / major);
            for _ in 0..24 {
                let (s, c) = t.sin_cos();
                let (ex, ey) = (major * c - u, minor * s - v);
                let (dx, dy) = (-major * s, minor * c);
                let slope = ex * dx + ey * dy;
                let curve = dx * dx + dy * dy + ex * (-major * c) + ey * (-minor * s);
                let step = if curve.abs() > 1e-12 { slope / curve } else { 0.0 };
                t -= step.clamp(-0.5, 0.5);
                if step.abs() < 1e-14 {
                    break;
                }
            }
            if dist(point_at(kind, t), p) > JOIN {
                return None;
            }
            let slack = JOIN / major.min(minor);
            let mut along = (t - a0).rem_euclid(TAU);
            if along > a1 - a0 + slack && along > TAU - slack {
                along -= TAU;
            }
            (along <= a1 - a0 + slack).then(|| a0 + along.clamp(0.0, a1 - a0))
        }
    }
}

/// Where two curves meet when no closed form is at hand: wherever pieces of them come close, Newton's
/// method from there on the two parameters, in a form that still settles where the curves only touch.
fn crossings_numeric(s: &SegKind, t: &SegKind) -> Vec<P2> {
    fn stations(kind: &SegKind) -> (f64, f64, usize) {
        match *kind {
            SegKind::Spline { ref shape, t0, t1 } => (t0, t1, (shape.breaks().len() * 24).max(32)),
            SegKind::Line { .. } => (0.0, 1.0, 64),
            SegKind::Arc { a0, a1, .. } => (a0, a1, (((a1 - a0).abs() * 16.0) as usize).max(8)),
            SegKind::Ellipse { a0, a1, major, minor, .. } => {
                let shape = major.max(minor) / major.min(minor);
                (a0, a1, (((a1 - a0).abs() * 16.0 * shape.sqrt()) as usize).max(8))
            }
        }
    }
    let (s0, s1, ns) = stations(s);
    let (t0, t1, nt) = stations(t);
    let along = |lo: f64, hi: f64, n: usize, i: usize| lo + (hi - lo) * i as f64 / n as f64;
    let (ps, pt): (Vec<P2>, Vec<P2>) = (
        (0..=ns).map(|i| point_at(s, along(s0, s1, ns, i))).collect(),
        (0..=nt).map(|j| point_at(t, along(t0, t1, nt, j))).collect(),
    );
    let mut out: Vec<P2> = Vec::new();
    for i in 0..ns {
        let (a, b) = (ps[i], ps[i + 1]);
        let margin_s = dist(a, b) * 0.2 + JOIN;
        for j in 0..nt {
            let (c, d) = (pt[j], pt[j + 1]);
            let margin = margin_s + dist(c, d) * 0.2;
            if a[0].max(b[0]) + margin < c[0].min(d[0])
                || c[0].max(d[0]) + margin < a[0].min(b[0])
                || a[1].max(b[1]) + margin < c[1].min(d[1])
                || c[1].max(d[1]) + margin < a[1].min(b[1])
            {
                continue;
            }
            // Start where the chords cross, if they do, and from the middles if not. Newton stays near
            // these pieces, since a far-off root would be found from its own pieces anyway.
            let (ds, dt) = ((s1 - s0) / ns as f64, (t1 - t0) / nt as f64);
            let (ua, va) = (along(s0, s1, ns, i), along(t0, t1, nt, j));
            let (eu, ev) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
            let denominator = eu[0] * ev[1] - eu[1] * ev[0];
            let w = [c[0] - a[0], c[1] - a[1]];
            let (fu, fv) = if denominator != 0.0 {
                let (fu, fv) = ((w[0] * ev[1] - w[1] * ev[0]) / denominator, (w[0] * eu[1] - w[1] * eu[0]) / denominator);
                ((fu.clamp(0.0, 1.0)), (fv.clamp(0.0, 1.0)))
            } else {
                (0.5, 0.5)
            };
            let (mut u, mut v) = (ua + ds * fu, va + dt * fv);
            for _ in 0..40 {
                let (p, q) = (point_at(s, u), point_at(t, v));
                let r = [p[0] - q[0], p[1] - q[1]];
                let (du, dv) = (velocity_at(s, u), velocity_at(t, v));
                // Least squares on the two parameters, with a little damping to get through tangency.
                let (aa, ab, bb) = (du[0] * du[0] + du[1] * du[1], -(du[0] * dv[0] + du[1] * dv[1]), dv[0] * dv[0] + dv[1] * dv[1]);
                let damp = 1e-10 * (aa + bb);
                let (g0, g1) = (du[0] * r[0] + du[1] * r[1], -(dv[0] * r[0] + dv[1] * r[1]));
                let det = (aa + damp) * (bb + damp) - ab * ab;
                if det.abs() < 1e-300 {
                    break;
                }
                let (step_u, step_v) = (((bb + damp) * g0 - ab * g1) / det, ((aa + damp) * g1 - ab * g0) / det);
                u -= step_u.clamp(-ds.abs(), ds.abs());
                v -= step_v.clamp(-dt.abs(), dt.abs());
                if (u - ua).abs() > 3.0 * ds.abs() || (v - va).abs() > 3.0 * dt.abs() {
                    break;
                }
                if step_u.abs() * aa.sqrt() + step_v.abs() * bb.sqrt() < 1e-13 {
                    break;
                }
            }
            let (p, q) = (point_at(s, u), point_at(t, v));
            if dist(p, q) <= JOIN && !out.iter().any(|&o| dist(o, p) <= JOIN / 2.0) {
                out.push(p);
            }
        }
    }
    out
}

/// Where two curves cross or touch, leaving out where they run together: the ends of an overlap are
/// found by testing each curve's ends against the other curve.
fn crossings(s: &SegKind, t: &SegKind) -> Vec<P2> {
    match (s, t) {
        (SegKind::Ellipse { .. } | SegKind::Spline { .. }, _) | (_, SegKind::Ellipse { .. } | SegKind::Spline { .. }) => crossings_numeric(s, t),
        (&SegKind::Line { a, b }, &SegKind::Line { a: c, b: d }) => {
            let (u, v) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
            let cross = u[0] * v[1] - u[1] * v[0];
            if cross.abs() <= 1e-12 * dist(a, b) * dist(c, d) {
                return Vec::new();
            }
            let w = [c[0] - a[0], c[1] - a[1]];
            let f = (w[0] * v[1] - w[1] * v[0]) / cross;
            vec![[a[0] + u[0] * f, a[1] + u[1] * f]]
        }
        (&SegKind::Line { a, b }, &SegKind::Arc { center, radius, .. }) | (&SegKind::Arc { center, radius, .. }, &SegKind::Line { a, b }) => {
            let f = line_param(a, b, center);
            let foot = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
            let d = dist(foot, center);
            if d > radius + JOIN {
                return Vec::new();
            }
            let h = (radius * radius - d * d).max(0.0).sqrt();
            let len = dist(a, b);
            let u = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
            vec![[foot[0] + u[0] * h, foot[1] + u[1] * h], [foot[0] - u[0] * h, foot[1] - u[1] * h]]
        }
        (&SegKind::Arc { center: c0, radius: r0, .. }, &SegKind::Arc { center: c1, radius: r1, .. }) => {
            let d = dist(c0, c1);
            if d <= JOIN || d > r0 + r1 + JOIN || d < (r0 - r1).abs() - JOIN {
                return Vec::new();
            }
            let a = (d * d + r0 * r0 - r1 * r1) / (2.0 * d);
            let h = (r0 * r0 - a * a).max(0.0).sqrt();
            let u = [(c1[0] - c0[0]) / d, (c1[1] - c0[1]) / d];
            let m = [c0[0] + u[0] * a, c0[1] + u[1] * a];
            vec![[m[0] - u[1] * h, m[1] + u[0] * h], [m[0] + u[1] * h, m[1] - u[0] * h]]
        }
    }
}

/// Where a spline crosses itself, as the two parameters of each crossing. Nothing else here can.
fn self_crossings(kind: &SegKind) -> Vec<(f64, f64)> {
    let SegKind::Spline { ref shape, t0, t1 } = *kind else {
        return Vec::new();
    };
    let n = (shape.breaks().len() * 24).max(48);
    let at = |i: usize| t0 + (t1 - t0) * i as f64 / n as f64;
    let points: Vec<P2> = (0..=n).map(|i| point_at(kind, at(i))).collect();
    let mut out: Vec<(f64, f64)> = Vec::new();
    for i in 0..n {
        let (a, b) = (points[i], points[i + 1]);
        for j in i + 2..n {
            let (c, d) = (points[j], points[j + 1]);
            let margin = (dist(a, b) + dist(c, d)) * 0.2 + JOIN;
            if a[0].max(b[0]) + margin < c[0].min(d[0])
                || c[0].max(d[0]) + margin < a[0].min(b[0])
                || a[1].max(b[1]) + margin < c[1].min(d[1])
                || c[1].max(d[1]) + margin < a[1].min(b[1])
            {
                continue;
            }
            let (mut u, mut v) = ((at(i) + at(i + 1)) / 2.0, (at(j) + at(j + 1)) / 2.0);
            for _ in 0..30 {
                let (p, q) = (shape.eval(u), shape.eval(v));
                let r = [p.0[0] - q.0[0], p.0[1] - q.0[1]];
                let det = -p.1[0] * q.1[1] + p.1[1] * q.1[0];
                if det.abs() < 1e-300 {
                    break;
                }
                let (du, dv) = ((-r[0] * q.1[1] + r[1] * q.1[0]) / det, (p.1[0] * r[1] - p.1[1] * r[0]) / det);
                u = (u - du).clamp(t0, t1);
                v = (v - dv).clamp(t0, t1);
                if du.abs() + dv.abs() < 1e-14 * (t1 - t0) {
                    break;
                }
            }
            let apart = (u - v).abs() > (t1 - t0) / n as f64;
            if apart && dist(shape.eval(u).0, shape.eval(v).0) <= JOIN && !out.iter().any(|o| (o.0 - u).abs() + (o.1 - v).abs() < 1e-9 * (t1 - t0)) {
                out.push((u.min(v), u.max(v)));
            }
        }
    }
    out
}

fn bounds(kind: &SegKind) -> [f64; 4] {
    match *kind {
        SegKind::Spline { ref shape, .. } => shape.bounds(),
        SegKind::Line { a, b } => [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])],
        SegKind::Arc { center, radius, .. } => [center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius],
        SegKind::Ellipse { center, major, minor, .. } => {
            let r = major.max(minor);
            [center[0] - r, center[1] - r, center[0] + r, center[1] + r]
        }
    }
}

/// Groups points closer than `JOIN`, and gives each the lowest index in its group.
fn cluster(points: &[P2]) -> Vec<usize> {
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..points.len()).collect();
    let mut order: Vec<usize> = (0..points.len()).collect();
    order.sort_by(|&a, &b| points[a][0].total_cmp(&points[b][0]));
    for (k, &i) in order.iter().enumerate() {
        for &j in &order[k + 1..] {
            if points[j][0] - points[i][0] > JOIN {
                break;
            }
            if dist(points[i], points[j]) <= JOIN {
                let (x, y) = (find(&mut parent, i), find(&mut parent, j));
                parent[x.max(y)] = x.min(y);
            }
        }
    }
    (0..points.len()).map(|i| find(&mut parent, i)).collect()
}

/// A stretch of one curve between two junctions.
struct Piece {
    curve: usize,
    /// Where it starts along its curve, which tells apart two pieces of one curve.
    start: f64,
    seg: Seg,
    from: usize,
    to: usize,
}

/// Half-edge `h` walks piece `h / 2`, forward if `h` is even.
fn walked(pieces: &[Piece], h: usize) -> Seg {
    if h % 2 == 0 {
        pieces[h / 2].seg.clone()
    } else {
        pieces[h / 2].seg.reversed()
    }
}

/// Puts half-edges leaving one junction in counter-clockwise order. Curves that leave in one direction
/// are ordered by how they bend away from it.
fn in_angular_order(mut out: Vec<(usize, f64, f64)>) -> Vec<usize> {
    const SAME: f64 = 1e-9;
    for o in &mut out {
        o.1 = o.1.rem_euclid(TAU);
    }
    out.sort_by(|a, b| a.1.total_cmp(&b.1));
    // Start after the widest gap, so no group of equal directions straddles the start.
    let n = out.len();
    let mut start = 0;
    let mut widest = -1.0;
    for i in 0..n {
        let gap = if i == 0 { out[0].1 + TAU - out[n - 1].1 } else { out[i].1 - out[i - 1].1 };
        if gap > widest {
            widest = gap;
            start = i;
        }
    }
    out.rotate_left(start);
    let mut i = 0;
    while i < n {
        let mut j = i + 1;
        while j < n && (out[j].1 - out[j - 1].1).rem_euclid(TAU) < SAME {
            j += 1;
        }
        out[i..j].sort_by(|a, b| a.2.total_cmp(&b.2));
        i = j;
    }
    out.into_iter().map(|o| o.0).collect()
}

/// A walk's curves and directions, with runs of one curve counted once and in no particular order,
/// since a junction added on the far side of a curve shouldn't change what this walk means.
fn signature(segs: &[(u32, bool)]) -> Vec<(u32, bool)> {
    let mut out: Vec<(u32, bool)> = Vec::with_capacity(segs.len());
    for &s in segs {
        if out.last() != Some(&s) {
            out.push(s);
        }
    }
    while out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    out.sort_unstable();
    out
}

fn id_from(signature: &[(u32, bool)], ordinal: u64) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    let mut add = |x: u64| {
        for b in x.to_le_bytes() {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    };
    for &(e, forward) in signature {
        add(e as u64 * 2 + forward as u64);
    }
    add(u64::MAX);
    add(ordinal);
    // Twelve hex digits, which a JavaScript number could hold too.
    (h ^ (h >> 48)) & 0xffff_ffff_ffff
}

/// One walk around a region or a group, ready for a `Loop`.
struct Walk {
    segs: Vec<Seg>,
    /// Entity and direction of each step, in order.
    steps: Vec<(u32, bool)>,
    /// Where each step starts along its curve.
    starts: Vec<f64>,
    group: usize,
    area: f64,
}

impl Walk {
    /// The position of the first step on the lowest-numbered entity. Which piece that is doesn't
    /// depend on where the walk happened to start.
    fn lowest_step(&self) -> usize {
        (0..self.steps.len())
            .min_by(|&a, &b| self.steps[a].0.cmp(&self.steps[b].0).then(self.starts[a].total_cmp(&self.starts[b])))
            .unwrap_or(0)
    }

    /// As a `Loop` that starts from the lowest-numbered entity if it's walked forward, or ends on it
    /// if it's walked backward. Sketches without crossings have always come out this way.
    fn into_loop(self) -> Loop {
        let k = self.lowest_step();
        let mut segs = self.segs;
        let n = segs.len();
        segs.rotate_left(if self.steps[k].1 { k } else { (k + 1) % n });
        Loop { segs }
    }
}

/// A whole circle or ellipse, which nothing touches.
fn is_whole_circle(l: &Loop) -> bool {
    l.segs.len() == 1
        && matches!(l.segs[0].kind, SegKind::Arc { a0, a1, .. } | SegKind::Ellipse { a0, a1, .. } if (a1 - a0).abs() >= TAU - 1e-9)
}

/// An ellipse's unit major direction and major radius, if it's a real ellipse.
fn ellipse_axes(center: P2, major: P2, minor: f64) -> Option<(P2, f64)> {
    let a = dist(center, major);
    (a > JOIN && a.is_finite() && minor > JOIN && minor.is_finite()).then(|| ([(major[0] - center[0]) / a, (major[1] - center[1]) / a], a))
}

fn refusal(message: String, entities: Vec<EntityId>) -> SketchError {
    SketchError { kind: ErrorKind::Regions, message, constraints: Vec::new(), entities }
}

impl Sketch {
    /// Every closed region of the sketch's geometry as it stands, and the curves that make none. Solve
    /// first: gaps the constraints would close are still gaps here. This never refuses a sketch.
    pub fn arrangement(&self) -> Arrangement {
        let mut ignored: Vec<Ignored> = Vec::new();
        let mut curves: Vec<(EntityId, SegKind, bool)> = Vec::new();
        for e in &self.entities {
            if e.construction {
                continue;
            }
            let curve = match e.geometry {
                Geometry::Spline(ref shape) => {
                    let (t0, t1) = shape.domain();
                    (shape.check().is_ok() && dist(shape.eval(t0).0, shape.eval((t0 + t1) / 2.0).0).max(dist(shape.eval(t0).0, shape.eval(t1).0)) > JOIN)
                        .then(|| (SegKind::Spline { shape: shape.clone(), t0, t1 }, false))
                }
                Geometry::Point { .. } => continue,
                Geometry::Line { start, end } => {
                    (dist(start, end) > JOIN && dist(start, end).is_finite()).then_some((SegKind::Line { a: start, b: end }, false))
                }
                Geometry::Circle { center, radius } => (radius > JOIN && radius.is_finite() && center.iter().all(|x| x.is_finite()))
                    .then_some((SegKind::Arc { center, radius, a0: 0.0, a1: TAU }, true)),
                Geometry::Arc { center, radius, start, end } => {
                    let good = radius > JOIN && radius.is_finite() && center.iter().chain(&start).chain(&end).all(|x| x.is_finite());
                    good.then(|| {
                        let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
                        let mut a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
                        while a1 <= a0 {
                            a1 += TAU;
                        }
                        (SegKind::Arc { center, radius, a0, a1 }, false)
                    })
                }
                Geometry::Ellipse { center, major, minor } => ellipse_axes(center, major, minor).map(|(x, a)| (SegKind::Ellipse { center, x, major: a, minor, a0: 0.0, a1: TAU }, true)),
                Geometry::EllipticalArc { center, major, minor, start, end } => {
                    ellipse_axes(center, major, minor).filter(|_| start.iter().chain(&end).all(|v| v.is_finite())).map(|(x, a)| {
                        let angle = |p: P2| {
                            let d = [p[0] - center[0], p[1] - center[1]];
                            ((d[1] * x[0] - d[0] * x[1]) / minor).atan2((d[0] * x[0] + d[1] * x[1]) / a)
                        };
                        let a0 = angle(start);
                        let mut a1 = angle(end);
                        while a1 <= a0 {
                            a1 += TAU;
                        }
                        (SegKind::Ellipse { center, x, major: a, minor, a0, a1 }, false)
                    })
                }
            };
            match curve {
                Some((kind, closed)) => curves.push((e.id, kind, closed)),
                None => ignored.push(Ignored { entity: e.id, whole: true, why: WhyIgnored::NoLength, pieces: Vec::new() }),
            }
        }

        // Every point where something happens along a curve. Ends come first, so that a junction
        // takes its position from a curve's end when it has one.
        let mut points: Vec<P2> = Vec::new();
        let mut marks: Vec<Vec<(f64, usize)>> = vec![Vec::new(); curves.len()];
        let mut ends: Vec<Vec<usize>> = vec![Vec::new(); curves.len()];
        for (i, (entity, kind, closed)) in curves.iter().enumerate() {
            if *closed {
                continue;
            }
            let seg = Seg { entity: *entity, kind: kind.clone() };
            let (first, last) = match *kind {
                SegKind::Line { .. } => (0.0, 1.0),
                SegKind::Arc { a0, a1, .. } | SegKind::Ellipse { a0, a1, .. } => (a0, a1),
                SegKind::Spline { t0, t1, .. } => (t0, t1),
            };
            for (p, t) in [(seg.start(), first), (seg.end(), last)] {
                points.push(p);
                ends[i].push(points.len() - 1);
                marks[i].push((t, points.len() - 1));
            }
        }
        for (i, c) in curves.iter().enumerate() {
            for (u, v) in self_crossings(&c.1) {
                points.push(point_at(&c.1, u));
                marks[i].push((u, points.len() - 1));
                marks[i].push((v, points.len() - 1));
            }
        }
        let boxes: Vec<[f64; 4]> = curves.iter().map(|c| bounds(&c.1)).collect();
        for i in 0..curves.len() {
            for j in i + 1..curves.len() {
                let (a, b) = (boxes[i], boxes[j]);
                if a[2] + JOIN < b[0] || b[2] + JOIN < a[0] || a[3] + JOIN < b[1] || b[3] + JOIN < a[1] {
                    continue;
                }
                let (s, t) = (&curves[i].1, &curves[j].1);
                for p in crossings(s, t) {
                    if let (Some(u), Some(v)) = (param_on(s, p), param_on(t, p)) {
                        points.push(p);
                        marks[i].push((u, points.len() - 1));
                        marks[j].push((v, points.len() - 1));
                    }
                }
                for &end in &ends[i] {
                    if let Some(v) = param_on(t, points[end]) {
                        marks[j].push((v, end));
                    }
                }
                for &end in &ends[j] {
                    if let Some(u) = param_on(s, points[end]) {
                        marks[i].push((u, end));
                    }
                }
            }
        }
        let node_of = cluster(&points);

        // Cut each curve at its junctions.
        let mut pieces: Vec<Piece> = Vec::new();
        let mut free_circles: Vec<usize> = Vec::new();
        for (c, (entity, kind, closed)) in curves.iter().enumerate() {
            let (entity, closed) = (*entity, *closed);
            let mut list = std::mem::take(&mut marks[c]);
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            let speed = match *kind {
                // The slowest it goes, roughly: its length over its range, which a stop near another is measured by.
                SegKind::Spline { ref shape, t0, t1 } => {
                    let d = shape.eval((t0 + t1) / 2.0).1;
                    d[0].hypot(d[1]).max(1e-9)
                }
                SegKind::Line { a, b } => dist(a, b),
                SegKind::Arc { radius, .. } => radius,
                SegKind::Ellipse { major, minor, .. } => major.min(minor),
            };
            let near = 4.0 * JOIN / speed;
            let mut stops: Vec<(f64, usize)> = Vec::new();
            for (t, p) in list {
                let node = node_of[p];
                if stops.last().is_none_or(|&(q, m)| m != node || t - q > near) {
                    stops.push((t, node));
                }
            }
            if closed {
                if stops.is_empty() {
                    free_circles.push(c);
                    continue;
                }
                let (first, last) = (stops[0], stops[stops.len() - 1]);
                if stops.len() > 1 && first.1 == last.1 && first.0 + TAU - last.0 <= near {
                    stops.pop();
                }
                stops.push((stops[0].0 + TAU, stops[0].1));
            } else if let (&SegKind::Arc { a1, .. } | &SegKind::Ellipse { a1, .. } | &SegKind::Spline { t1: a1, .. }, Some(last)) = (kind, stops.last_mut()) {
                last.0 = a1;
            } else if let Some(last) = stops.last_mut() {
                last.0 = 1.0;
            }
            if stops.len() < 2 {
                // Both its ends fell into one junction.
                ignored.push(Ignored { entity, whole: true, why: WhyIgnored::NoLength, pieces: Vec::new() });
            }
            for w in stops.windows(2) {
                let ((t0, n0), (t1, n1)) = (w[0], w[1]);
                let seg_kind = match *kind {
                    SegKind::Spline { ref shape, .. } => SegKind::Spline { shape: shape.clone(), t0, t1 },
                    SegKind::Line { .. } => SegKind::Line { a: points[n0], b: points[n1] },
                    SegKind::Arc { center, radius, .. } => SegKind::Arc { center, radius, a0: t0, a1: t1 },
                    SegKind::Ellipse { center, x, major, minor, .. } => SegKind::Ellipse { center, x, major, minor, a0: t0, a1: t1 },
                };
                pieces.push(Piece { curve: c, start: t0, seg: Seg { entity, kind: seg_kind }, from: n0, to: n1 });
            }
        }

        // A stretch drawn twice counts once. The first curve to reach it keeps it.
        let mut alive = vec![true; pieces.len()];
        let mut dropped: Vec<(usize, WhyIgnored)> = Vec::new();
        let mut by_ends: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (i, p) in pieces.iter().enumerate() {
            let key = (p.from.min(p.to), p.from.max(p.to));
            let same = by_ends.entry(key).or_default();
            if let Some(&k) = same.iter().find(|&&k| dist(pieces[k].seg.middle(), p.seg.middle()) <= 10.0 * JOIN) {
                alive[i] = false;
                dropped.push((i, WhyIgnored::Overlap { with: pieces[k].seg.entity }));
            } else {
                same.push(i);
            }
        }

        // Walk the faces, taking away spurs and bridges until every piece has a different walk on each
        // side. Each pass either removes something or finishes.
        let half = pieces.len() * 2;
        let mut walks: Vec<Vec<usize>>;
        loop {
            // Dead ends first, following a spur back to where it joins.
            let mut touching: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
            for (i, p) in pieces.iter().enumerate().filter(|(i, _)| alive[*i]) {
                touching.entry(p.from).or_default().push(i);
                touching.entry(p.to).or_default().push(i);
            }
            let mut tips: Vec<usize> = touching.iter().filter(|(_, l)| l.len() == 1).map(|(n, _)| *n).collect();
            let mut next_tip = 0;
            while next_tip < tips.len() {
                let node = tips[next_tip];
                next_tip += 1;
                let list = touching.get(&node).cloned().unwrap_or_default();
                let Some(&i) = list.iter().find(|&&i| alive[i]) else { continue };
                alive[i] = false;
                dropped.push((i, WhyIgnored::FreeEnd { at: points[node] }));
                let other = if pieces[i].from == node { pieces[i].to } else { pieces[i].from };
                let rest = touching.entry(other).or_default();
                rest.retain(|&k| k != i);
                if rest.len() == 1 {
                    tips.push(other);
                }
            }
            // Leave each junction by the next piece clockwise from the one you arrived on.
            let mut leaving: BTreeMap<usize, Vec<(usize, f64, f64)>> = BTreeMap::new();
            for (i, p) in pieces.iter().enumerate().filter(|(i, _)| alive[*i]) {
                for (h, node) in [(i * 2, p.from), (i * 2 + 1, p.to)] {
                    let (angle, bend) = walked(&pieces, h).leaving();
                    leaving.entry(node).or_default().push((h, angle, bend));
                }
            }
            let mut clockwise = vec![usize::MAX; half];
            for (_, out) in leaving {
                let order = in_angular_order(out);
                for (k, &h) in order.iter().enumerate() {
                    clockwise[h] = order[(k + order.len() - 1) % order.len()];
                }
            }
            let mut seen = vec![false; half];
            let mut walk_of = vec![usize::MAX; half];
            walks = Vec::new();
            for start in 0..half {
                if !alive[start / 2] || seen[start] {
                    continue;
                }
                let mut walk = Vec::new();
                let mut h = start;
                while !seen[h] {
                    seen[h] = true;
                    walk_of[h] = walks.len();
                    walk.push(h);
                    h = clockwise[h ^ 1];
                }
                walks.push(walk);
            }
            let mut removed = false;
            for i in 0..pieces.len() {
                if alive[i] && walk_of[i * 2] == walk_of[i * 2 + 1] {
                    alive[i] = false;
                    dropped.push((i, WhyIgnored::Bridge));
                    removed = true;
                }
            }
            if !removed {
                break;
            }
        }

        // Connected groups of what's left.
        let mut group_of_node: BTreeMap<usize, usize> = BTreeMap::new();
        let mut parent: Vec<usize> = Vec::new();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for (i, p) in pieces.iter().enumerate().filter(|(i, _)| alive[*i]) {
            let _ = i;
            for node in [p.from, p.to] {
                group_of_node.entry(node).or_insert_with(|| {
                    parent.push(parent.len());
                    parent.len() - 1
                });
            }
            let (a, b) = (group_of_node[&p.from], group_of_node[&p.to]);
            let (x, y) = (root(&mut parent, a), root(&mut parent, b));
            parent[x.max(y)] = x.min(y);
        }

        let mut outers: Vec<Walk> = Vec::new();
        let mut inners: Vec<Walk> = Vec::new();
        for walk in &walks {
            let segs: Vec<Seg> = walk.iter().map(|&h| walked(&pieces, h)).collect();
            let area = Loop { segs: segs.clone() }.signed_area();
            let group = {
                let g = group_of_node[&pieces[walk[0] / 2].from];
                root(&mut parent, g)
            };
            let w = Walk {
                segs,
                steps: walk.iter().map(|&h| (pieces[h / 2].seg.entity.0, h % 2 == 0)).collect(),
                starts: walk.iter().map(|&h| pieces[h / 2].start).collect(),
                group,
                area,
            };
            if area >= MIN_AREA {
                outers.push(w);
            } else if area <= -MIN_AREA {
                inners.push(w);
            } else {
                for &h in walk {
                    if alive[h / 2] {
                        alive[h / 2] = false;
                        dropped.push((h / 2, WhyIgnored::NoArea));
                    }
                }
            }
        }
        // A circle nothing touches is a group of its own: one walk each way round.
        for &c in &free_circles {
            let (entity, kind, _) = curves[c].clone();
            let group = parent.len();
            parent.push(group);
            let seg = Seg { entity, kind };
            let area = Loop { segs: vec![seg.clone()] }.signed_area();
            if area < MIN_AREA {
                ignored.push(Ignored { entity, whole: true, why: WhyIgnored::NoArea, pieces: vec![seg] });
                continue;
            }
            outers.push(Walk { segs: vec![seg.clone()], steps: vec![(entity.0, true)], starts: vec![0.0], group, area });
            inners.push(Walk { segs: vec![seg.reversed()], steps: vec![(entity.0, false)], starts: vec![0.0], group, area: -area });
        }

        // The ids: from the outer walk alone, so holes coming and going don't rename a region.
        let mut same_signature: BTreeMap<Vec<(u32, bool)>, Vec<usize>> = BTreeMap::new();
        for (i, w) in outers.iter().enumerate() {
            same_signature.entry(signature(&w.steps)).or_default().push(i);
        }
        let mut ids = vec![RegionId(0); outers.len()];
        let mut taken: Vec<u64> = Vec::new();
        for (sig, mut members) in same_signature {
            let along = |i: usize| {
                let k = outers[i].lowest_step();
                outers[i].starts[k]
            };
            members.sort_by(|&a, &b| along(a).total_cmp(&along(b)));
            for (ordinal, &i) in members.iter().enumerate() {
                let mut salt = ordinal as u64;
                let mut id = id_from(&sig, salt);
                while taken.contains(&id) {
                    salt += 1 << 32;
                    id = id_from(&sig, salt);
                }
                taken.push(id);
                ids[i] = RegionId(id);
            }
        }

        // Each group's outside is a hole in the smallest region around it, if any.
        let inner_points: Vec<P2> = inners.iter().map(|w| w.segs[0].start()).collect();
        let outer_loops: Vec<Loop> = outers.iter().map(|w| Loop { segs: w.segs.clone() }).collect();
        let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); outers.len()];
        let mut surrounding: BTreeMap<usize, usize> = BTreeMap::new();
        for (n, inner) in inners.iter().enumerate() {
            let mut best: Option<usize> = None;
            for (o, outer) in outers.iter().enumerate() {
                if outer.group != inner.group
                    && best.is_none_or(|b| outer.area < outers[b].area)
                    && outer_loops[o].contains(inner_points[n])
                {
                    best = Some(o);
                }
            }
            if let Some(o) = best {
                holes_of[o].push(n);
                surrounding.insert(inner.group, outers[o].group);
            }
        }
        let depth_of = |group: usize| {
            let mut depth = 0;
            let mut g = group;
            while let Some(&up) = surrounding.get(&g) {
                depth += 1;
                g = up;
                if depth as usize > parent.len() {
                    break;
                }
            }
            depth
        };

        let order_of = |l: &Loop| {
            let lowest = l.segs.iter().map(|s| s.entity.0).min().unwrap_or(0);
            (!is_whole_circle(l), lowest)
        };
        let inner_loops: Vec<Option<Loop>> = inners.into_iter().map(|w| Some(w.into_loop())).collect();
        let mut inner_loops = inner_loops;
        let mut regions: Vec<Region> = Vec::with_capacity(outers.len());
        for (o, outer) in outers.into_iter().enumerate() {
            let depth = depth_of(outer.group);
            let mut holes: Vec<Loop> = holes_of[o].iter().filter_map(|&n| inner_loops[n].take()).collect();
            holes.sort_by_key(|l| order_of(l));
            regions.push(Region { id: ids[o], depth, outer: outer.into_loop(), holes });
        }
        regions.sort_by_key(|r| (order_of(&r.outer), r.id));

        // What was left over, by entity and reason.
        let remaining: Vec<usize> = curves
            .iter()
            .enumerate()
            .map(|(c, _)| pieces.iter().enumerate().filter(|(i, p)| alive[*i] && p.curve == c).count())
            .collect();
        for (i, why) in dropped {
            let p = &pieces[i];
            let whole = remaining[p.curve] == 0;
            match ignored.iter_mut().find(|g| g.entity == p.seg.entity && g.why == why) {
                Some(group) => group.pieces.push(p.seg.clone()),
                None => ignored.push(Ignored { entity: p.seg.entity, whole, why, pieces: vec![p.seg.clone()] }),
            }
        }
        Arrangement { regions, ignored }
    }

    /// The regions to use: those picked with [`Sketch::pick_regions`], or if none are picked, every
    /// other level of nesting, so a loop inside a loop is a hole. Refused only if that leaves nothing,
    /// or if a picked region no longer exists.
    pub fn regions(&self) -> Result<Vec<Region>, SketchError> {
        let found = self.arrangement();
        if !self.picked_regions.is_empty() {
            let mut out = Vec::with_capacity(self.picked_regions.len());
            for &id in &self.picked_regions {
                match found.region(id) {
                    Some(r) => out.push(r.clone()),
                    None => {
                        let there: Vec<String> = found.regions.iter().map(|r| r.id.name()).collect();
                        let list = if there.is_empty() { "none".to_string() } else { there.join(", ") };
                        return Err(refusal(format!("picked region {} isn't in the sketch any more. It has: {list}", id.name()), Vec::new()));
                    }
                }
            }
            return Ok(out);
        }
        let out = found.default_regions();
        if !out.is_empty() {
            return Ok(out);
        }
        let free = found.ignored.iter().find_map(|i| match i.why {
            WhyIgnored::FreeEnd { at } => Some((i.entity, at)),
            _ => None,
        });
        if let Some((entity, at)) = free {
            return Err(refusal(format!("the sketch isn't closed: entity {} has a free end at ({:.6}, {:.6})", entity.0, at[0], at[1]), vec![entity]));
        }
        if let Some(i) = found.ignored.first() {
            let who: Vec<EntityId> = found.ignored.iter().map(|g| g.entity).collect();
            let what = match i.why {
                WhyIgnored::NoLength => "a curve in the sketch has no length",
                _ => "a loop in the sketch encloses no area",
            };
            return Err(refusal(what.to_string(), who));
        }
        Err(refusal("the sketch has no closed regions: it has no lines, arcs or circles outside construction geometry".to_string(), Vec::new()))
    }

    /// Chooses which regions [`Sketch::regions`] returns, by id. An empty list goes back to the default.
    /// The choice is saved with the sketch.
    pub fn pick_regions(&mut self, ids: &[RegionId]) {
        self.picked_regions = ids.to_vec();
    }

    pub fn picked_regions(&self) -> &[RegionId] {
        &self.picked_regions
    }
}
