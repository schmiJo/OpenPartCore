//! Finds the closed regions of a sketch: the areas a kernel profile can be made from.
//!
//! Lines, arcs and circles that aren't construction geometry must form separate closed loops that
//! don't touch each other. A loop inside another is a hole in it, and a loop inside a hole starts a
//! new region. Anything else is refused, with the entities named.

use crate::model::*;
use crate::solve::{ErrorKind, SketchError};
use std::f64::consts::{PI, TAU};

/// Endpoints this close count as joined. Within the kernel's tolerance, and far above the solver's.
const JOIN: f64 = 1e-7;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SegKind {
    Line { a: P2, b: P2 },
    /// Runs from angle `a0` to `a1` in radians: counter-clockwise if `a1 > a0`, clockwise if not.
    /// A whole circle runs through a full turn.
    Arc { center: P2, radius: f64, a0: f64, a1: f64 },
}

/// A piece of a loop, and the entity it came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seg {
    pub entity: EntityId,
    pub kind: SegKind,
}

impl Seg {
    pub fn start(&self) -> P2 {
        match self.kind {
            SegKind::Line { a, .. } => a,
            SegKind::Arc { center, radius, a0, .. } => [center[0] + radius * a0.cos(), center[1] + radius * a0.sin()],
        }
    }

    pub fn end(&self) -> P2 {
        match self.kind {
            SegKind::Line { b, .. } => b,
            SegKind::Arc { center, radius, a1, .. } => [center[0] + radius * a1.cos(), center[1] + radius * a1.sin()],
        }
    }

    fn reversed(&self) -> Seg {
        let kind = match self.kind {
            SegKind::Line { a, b } => SegKind::Line { a: b, b: a },
            SegKind::Arc { center, radius, a0, a1 } => SegKind::Arc { center, radius, a0: a1, a1: a0 },
        };
        Seg { entity: self.entity, kind }
    }

    fn bounds(&self) -> [f64; 4] {
        match self.kind {
            SegKind::Line { a, b } => [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])],
            SegKind::Arc { center, radius, .. } => [center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius],
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
            };
        }
        sum / 2.0
    }

    fn reversed(&self) -> Loop {
        Loop { segs: self.segs.iter().rev().map(Seg::reversed).collect() }
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
            }
        }
        winding.abs() > PI
    }
}

/// An area of the sketch: a counter-clockwise outer loop and clockwise holes.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub outer: Loop,
    pub holes: Vec<Loop>,
}

fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn on_arc(angle: f64, a0: f64, a1: f64, slack: f64) -> bool {
    let (lo, hi) = if a1 >= a0 { (a0, a1) } else { (a1, a0) };
    let mut t = (angle - lo).rem_euclid(TAU);
    if t > TAU - slack {
        t -= TAU;
    }
    t >= -slack && t <= hi - lo + slack
}

fn line_param(a: P2, b: P2, p: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
}

/// Where two segments meet: up to two points, or the overlap's ends if they run together.
fn contacts(s: &Seg, t: &Seg) -> Vec<P2> {
    let eps = JOIN * 10.0;
    let on_seg = |seg: &Seg, p: P2| -> bool {
        match seg.kind {
            SegKind::Line { a, b } => {
                let f = line_param(a, b, p);
                let (fx, fy) = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f);
                let len = dist(a, b);
                dist([fx, fy], p) <= eps && f * len >= -eps && (1.0 - f) * len >= -eps
            }
            SegKind::Arc { center, radius, a0, a1 } => {
                (dist(center, p) - radius).abs() <= eps && on_arc((p[1] - center[1]).atan2(p[0] - center[0]), a0, a1, eps / radius)
            }
        }
    };
    let mut out = Vec::new();
    match (s.kind, t.kind) {
        (SegKind::Line { a, b }, SegKind::Line { a: c, b: d }) => {
            let (u, v) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
            let cross = u[0] * v[1] - u[1] * v[0];
            if cross.abs() > 1e-12 * dist(a, b) * dist(c, d) {
                let w = [c[0] - a[0], c[1] - a[1]];
                let f = (w[0] * v[1] - w[1] * v[0]) / cross;
                out.push([a[0] + u[0] * f, a[1] + u[1] * f]);
            } else {
                out.extend_from_slice(&[a, b, c, d]);
            }
        }
        (SegKind::Line { a, b }, SegKind::Arc { center, radius, .. }) | (SegKind::Arc { center, radius, .. }, SegKind::Line { a, b }) => {
            let f = line_param(a, b, center);
            let foot = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
            let d = dist(foot, center);
            if d <= radius + eps {
                let h = (radius * radius - d * d).max(0.0).sqrt();
                let len = dist(a, b);
                let u = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
                out.push([foot[0] + u[0] * h, foot[1] + u[1] * h]);
                out.push([foot[0] - u[0] * h, foot[1] - u[1] * h]);
            }
        }
        (SegKind::Arc { center: c0, radius: r0, .. }, SegKind::Arc { center: c1, radius: r1, .. }) => {
            let d = dist(c0, c1);
            if d < eps {
                if (r0 - r1).abs() < eps {
                    out.extend_from_slice(&[s.start(), s.end(), t.start(), t.end()]);
                }
            } else if d <= r0 + r1 + eps && d >= (r0 - r1).abs() - eps {
                let a = (d * d + r0 * r0 - r1 * r1) / (2.0 * d);
                let h = (r0 * r0 - a * a).max(0.0).sqrt();
                let u = [(c1[0] - c0[0]) / d, (c1[1] - c0[1]) / d];
                let m = [c0[0] + u[0] * a, c0[1] + u[1] * a];
                out.push([m[0] - u[1] * h, m[1] + u[0] * h]);
                out.push([m[0] + u[1] * h, m[1] - u[0] * h]);
            }
        }
    }
    out.retain(|&p| on_seg(s, p) && on_seg(t, p));
    out
}

fn refusal(message: String, entities: Vec<EntityId>) -> SketchError {
    SketchError { kind: ErrorKind::Regions, message, constraints: Vec::new(), entities }
}

impl Sketch {
    /// The closed regions of the sketch's geometry as it stands. Solve first: gaps the constraints
    /// would close are still gaps here.
    pub fn regions(&self) -> Result<Vec<Region>, SketchError> {
        let mut closed: Vec<Loop> = Vec::new();
        let mut open: Vec<Seg> = Vec::new();
        for e in &self.entities {
            if e.construction {
                continue;
            }
            match e.geometry {
                Geometry::Point { .. } => {}
                Geometry::Line { start, end } => open.push(Seg { entity: e.id, kind: SegKind::Line { a: start, b: end } }),
                Geometry::Circle { center, radius } => {
                    closed.push(Loop { segs: vec![Seg { entity: e.id, kind: SegKind::Arc { center, radius, a0: 0.0, a1: TAU } }] });
                }
                Geometry::Arc { center, radius, start, end } => {
                    let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
                    let mut a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
                    while a1 <= a0 {
                        a1 += TAU;
                    }
                    open.push(Seg { entity: e.id, kind: SegKind::Arc { center, radius, a0, a1 } });
                }
            }
        }
        // Join ends into nodes.
        let mut nodes: Vec<P2> = Vec::new();
        let mut node_of = |p: P2| -> usize {
            if let Some(i) = nodes.iter().position(|&q| dist(p, q) <= JOIN) {
                return i;
            }
            nodes.push(p);
            nodes.len() - 1
        };
        let ends: Vec<[usize; 2]> = open.iter().map(|s| [node_of(s.start()), node_of(s.end())]).collect();
        let mut at_node: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
        for (i, e) in ends.iter().enumerate() {
            at_node[e[0]].push(i);
            at_node[e[1]].push(i);
        }
        for (n, list) in at_node.iter().enumerate() {
            let names = |l: &[usize]| l.iter().map(|&i| open[i].entity).collect::<Vec<_>>();
            let shown = |l: &[EntityId]| l.iter().map(|e| e.0.to_string()).collect::<Vec<_>>().join(", ");
            if list.len() == 1 {
                let who = names(list);
                return Err(refusal(
                    format!("the sketch isn't closed: entity {} has a free end at ({:.6}, {:.6})", shown(&who), nodes[n][0], nodes[n][1]),
                    who,
                ));
            }
            if list.len() > 2 {
                let who = names(list);
                return Err(refusal(
                    format!("{} edges meet at ({:.6}, {:.6}), so the loops are ambiguous: entities {}. Make any that only guide the sketch construction geometry.", list.len(), nodes[n][0], nodes[n][1], shown(&who)),
                    who,
                ));
            }
        }
        // Walk the chains.
        let mut used = vec![false; open.len()];
        for first in 0..open.len() {
            if used[first] {
                continue;
            }
            used[first] = true;
            let mut segs = vec![open[first]];
            let start_node = ends[first][0];
            let mut node = ends[first][1];
            while node != start_node {
                let Some(&next) = at_node[node].iter().find(|&&i| !used[i]) else { break };
                used[next] = true;
                if ends[next][0] == node {
                    segs.push(open[next]);
                    node = ends[next][1];
                } else {
                    segs.push(open[next].reversed());
                    node = ends[next][0];
                }
            }
            closed.push(Loop { segs });
        }
        if closed.is_empty() {
            return Err(refusal("the sketch has no closed regions: it has no lines, arcs or circles outside construction geometry".to_string(), Vec::new()));
        }
        // Nothing may cross or touch, apart from neighbours at their shared ends.
        let all: Vec<(usize, Seg)> = closed.iter().enumerate().flat_map(|(l, lp)| lp.segs.iter().map(move |s| (l, *s))).collect();
        let boxes: Vec<[f64; 4]> = all.iter().map(|(_, s)| s.bounds()).collect();
        let eps = JOIN * 10.0;
        for i in 0..all.len() {
            for j in i + 1..all.len() {
                let (a, b) = (boxes[i], boxes[j]);
                if a[2] + eps < b[0] || b[2] + eps < a[0] || a[3] + eps < b[1] || b[3] + eps < a[1] {
                    continue;
                }
                let (s, t) = (&all[i].1, &all[j].1);
                let shared = [s.start(), s.end()];
                let others = [t.start(), t.end()];
                for p in contacts(s, t) {
                    let at_shared_end = all[i].0 == all[j].0
                        && shared.iter().any(|&q| dist(q, p) <= eps && others.iter().any(|&r| dist(r, p) <= eps));
                    if !at_shared_end {
                        return Err(refusal(
                            format!("entities {} and {} cross or touch at ({:.6}, {:.6})", s.entity.0, t.entity.0, p[0], p[1]),
                            vec![s.entity, t.entity],
                        ));
                    }
                }
            }
        }
        // Each loop's depth inside the others. A loop's own area decides its parent.
        let areas: Vec<f64> = closed.iter().map(|l| l.signed_area()).collect();
        for (l, a) in closed.iter().zip(&areas) {
            if a.abs() < 1e-9 {
                let who: Vec<EntityId> = l.segs.iter().map(|s| s.entity).collect();
                return Err(refusal("a loop in the sketch encloses no area".to_string(), who));
            }
        }
        let mut parents: Vec<Option<usize>> = Vec::with_capacity(closed.len());
        let mut depth = vec![0usize; closed.len()];
        for i in 0..closed.len() {
            let p = closed[i].segs[0].start();
            let mut parent = None;
            let mut smallest = f64::INFINITY;
            for j in 0..closed.len() {
                if i != j && areas[j].abs() > areas[i].abs() && closed[j].contains(p) {
                    depth[i] += 1;
                    if areas[j].abs() < smallest {
                        smallest = areas[j].abs();
                        parent = Some(j);
                    }
                }
            }
            parents.push(parent);
        }
        let ccw = |l: &Loop, a: f64| if a > 0.0 { l.clone() } else { l.reversed() };
        let cw = |l: &Loop, a: f64| if a < 0.0 { l.clone() } else { l.reversed() };
        let mut out = Vec::new();
        let mut index_of = vec![usize::MAX; closed.len()];
        for i in 0..closed.len() {
            if depth[i] % 2 == 0 {
                index_of[i] = out.len();
                out.push(Region { outer: ccw(&closed[i], areas[i]), holes: Vec::new() });
            }
        }
        for i in 0..closed.len() {
            if depth[i] % 2 == 1 {
                if let Some(p) = parents[i] {
                    out[index_of[p]].holes.push(cw(&closed[i], areas[i]));
                }
            }
        }
        Ok(out)
    }
}
