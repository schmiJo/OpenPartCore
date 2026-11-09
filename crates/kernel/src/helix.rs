//! A solid swept along a helix: springs, threads and twisted bars.
//!
//! A profile carried by a screw motion is rigid, so every section is exact. Along the helix the true surface has no
//! rational form, so it's fitted, piece by piece: each control point of the section follows its own helix, and since
//! its position and first two derivatives there are known exactly, each piece is the quintic matching them at both
//! ends. Only the rotation is approximated, and for a quintic of this kind the error is at most `r h^6 / 46080` for a
//! radius `r` and an angle `h` per piece, which picks the number of pieces to keep it under `FIT`. The tests measure it.
//!
//! Neighbouring faces share their control points along an edge, so the fit can't open a gap between them: only the
//! distance from the true helix is approximate, and that's what's stated.
//!
//! A side is cut into a face at every half turn counted from the start, so that changing the number of turns leaves the
//! earlier faces where they were. Each records which half turn it is the way a pattern's copy does, which gives it a key
//! of its own, like `f4:side.0@f4.3`, that doesn't depend on where anything else is.

use crate::build::{SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::error::{Error, Result};
use crate::loft::{area_vector, edge_curve, Chain};
use crate::math::*;
use crate::patch::Patch;
use crate::profile::Profile;
use crate::spline::Hom;
use crate::surface::Surface;
use crate::topo::{Body, Coedge, CopyStep, FaceTag};
use crate::validate::validate;
use std::sync::Arc;

/// How far a helical face may be from the true one, by the bound above.
pub const FIT: f64 = 1e-7;
/// Faces are at most this much of a turn, so that a long spring's faces each have a small bounding box.
const FACE_ANGLE: f64 = PI;
/// No face at the end of a side is shorter than this.
const SHORTEST: f64 = PI / 18.0;
/// More turns than this is many thousands of faces, which isn't what this kernel is for.
const MAX_TURNS: f64 = 200.0;

/// The motion of a helix. Sections travel around `axis` through `origin`, moving `pitch` along it for each turn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Helix {
    pub origin: Vec3,
    pub axis: Vec3,
    /// How far along the axis one turn goes. Above zero.
    pub pitch: f64,
    pub turns: f64,
    pub left_handed: bool,
}

/// A point on its way around, by angle.
#[derive(Clone, Copy)]
struct Screw {
    origin: Vec3,
    axis: Vec3,
    per_radian: f64,
    spin: f64,
}

impl Screw {
    fn of(h: &Helix) -> Screw {
        Screw { origin: h.origin, axis: h.axis.normalized(), per_radian: h.pitch / TAU, spin: if h.left_handed { -1.0 } else { 1.0 } }
    }

    fn radial(&self, p: Vec3) -> (f64, Vec3) {
        let from = p - self.origin;
        let along = from.dot(self.axis);
        (along, from - self.axis * along)
    }

    fn at(&self, p: Vec3, angle: f64) -> Vec3 {
        let (along, r) = self.radial(p);
        let side = self.axis.cross(r);
        let (s, c) = (self.spin * angle).sin_cos();
        self.origin + self.axis * (along + self.per_radian * angle) + r * c + side * s
    }

    /// The first and second derivatives by angle.
    fn rates(&self, p: Vec3, angle: f64) -> (Vec3, Vec3) {
        let (_, r) = self.radial(p);
        let side = self.axis.cross(r);
        let (s, c) = (self.spin * angle).sin_cos();
        (self.axis * self.per_radian + (side * c - r * s) * self.spin, -(r * c + side * s))
    }

    fn frame_at(&self, f: &Frame, angle: f64) -> Frame {
        let turn = |d: Vec3| self.at(self.origin + d, angle) - self.at(self.origin, angle);
        Frame { origin: self.at(f.origin, angle), x: turn(f.x), y: turn(f.y), z: turn(f.z) }
    }

    /// The six control points of the quintic for one point from `a` to `b`, without repeating the first if `skip`.
    fn quintic(&self, p: Vec3, a: f64, b: f64, out: &mut Vec<Vec3>, skip: bool) {
        let h = b - a;
        let (f0, f1) = (self.at(p, a), self.at(p, b));
        let ((d0, s0), (d1, s1)) = (self.rates(p, a), self.rates(p, b));
        if !skip {
            out.push(f0);
        }
        out.push(f0 + d0 * (h / 5.0));
        out.push(f0 + d0 * (h * 2.0 / 5.0) + s0 * (h * h / 20.0));
        out.push(f1 - d1 * (h * 2.0 / 5.0) + s1 * (h * h / 20.0));
        out.push(f1 - d1 * (h / 5.0));
        out.push(f1);
    }
}

/// Where a section would sweep through itself: whether a point on its outline comes back around to land inside it,
/// after any number of turns and in either direction.
fn keeps_clear_of_itself(screw: &Screw, frame: &Frame, outline: &[Vec3], total: f64) -> Result<()> {
    const STEPS_PER_TURN: f64 = 64.0;
    let polygon: Vec<Vec2> = outline.iter().map(|p| v2((*p - frame.origin).dot(frame.x), (*p - frame.origin).dot(frame.y))).collect();
    let near = |q: Vec2| -> f64 {
        (0..polygon.len())
            .map(|i| {
                let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
                let d = b - a;
                let t = if d.dot(d) > 0.0 { ((q - a).dot(d) / d.dot(d)).clamp(0.0, 1.0) } else { 0.0 };
                (a + d * t - q).norm()
            })
            .fold(f64::INFINITY, f64::min)
    };
    let inside = |q: Vec2| {
        let mut odd = false;
        for i in 0..polygon.len() {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            if (a.y > q.y) != (b.y > q.y) && q.x < a.x + (q.y - a.y) / (b.y - a.y) * (b.x - a.x) {
                odd = !odd;
            }
        }
        odd
    };
    let steps = (total / TAU * STEPS_PER_TURN).ceil().max(4.0) as usize;
    for way in [1.0, -1.0] {
        for &p in outline {
            let height = |angle: f64| (screw.at(p, way * angle) - frame.origin).dot(frame.z);
            let mut last = height(total / steps as f64);
            for k in 2..=steps {
                let angle = total * k as f64 / steps as f64;
                let now = height(angle);
                if last * now <= 0.0 {
                    // Between these two angles it crosses the section's plane.
                    let (mut lo, mut hi) = (total * (k - 1) as f64 / steps as f64, angle);
                    for _ in 0..40 {
                        let mid = (lo + hi) / 2.0;
                        if height(lo) * height(mid) <= 0.0 {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    let at = screw.at(p, way * lo) - frame.origin;
                    let q = v2(at.dot(frame.x), at.dot(frame.y));
                    if inside(q) || near(q) < 10.0 * MIN_FEATURE {
                        return Err(Error::InvalidProfile("helix: the turns run into each other, as the profile is too tall for the pitch. Make the pitch larger or the profile smaller".into()));
                    }
                }
                last = now;
            }
        }
    }
    Ok(())
}

/// Sweeps a profile along a helix, from `frame`. The section can hold the axis, as a spring's or a thread's does, or lie
/// across it, which makes a twisted bar. The sides are named from the profile's segments, and the ends are the start
/// and end caps. A side is cut into one face each half turn, from the start.
pub fn helix(profile: &Profile, frame: &Frame, h: &Helix, feature: u32) -> Result<Body> {
    let bad = |why: String| Err(Error::InvalidProfile(format!("helix: {why}")));
    if !(h.axis.norm() > 1e-12) {
        return bad("its axis must have a length".into());
    }
    if !(h.pitch > MIN_FEATURE && h.pitch < MAX_COORD) {
        return bad(format!("a pitch of {} is out of range", h.pitch));
    }
    if !(h.turns > 1e-3 && h.turns <= MAX_TURNS) {
        return bad(format!("{} turns is out of range: it takes above 0 and at most {MAX_TURNS}", h.turns));
    }
    profile.validate()?;
    let screw = Screw::of(h);
    let total = h.turns * TAU;

    let mut wires: Vec<Vec<Chain>> = Vec::with_capacity(profile.holes.len() + 1);
    for w in profile.wires_for_faces() {
        wires.push(w.segs.iter().map(|s| Chain::of(s, frame)).collect::<Result<Vec<_>>>()?);
    }

    // The section has to cross the path, and always the same way.
    let sample = |chain: &Chain, out: &mut Vec<Vec3>| {
        for span in &chain.spans {
            for i in 0..8 {
                let s = i as f64 / 8.0;
                let mut work = span.clone();
                for level in (1..work.len()).rev() {
                    for k in 0..level {
                        work[k] = work[k].lerp(work[k + 1], s);
                    }
                }
                out.push(work[0].point());
            }
        }
    };
    let mut outline = Vec::new();
    for c in &wires[0] {
        sample(c, &mut outline);
    }
    let crossing = |p: Vec3| screw.rates(p, 0.0).0.dot(frame.z);
    let rates: Vec<f64> = outline.iter().map(|p| crossing(*p)).collect();
    let top = rates.iter().fold(0.0f64, |m, c| m.max(c.abs()));
    if rates.iter().any(|c| c.abs() <= 1e-3 * top) || rates.iter().any(|c| c.signum() != rates[0].signum()) {
        return bad("the profile has to cross the path the same way all round: it can't touch or cross the axis while holding it, or lie along the path".into());
    }
    keeps_clear_of_itself(&screw, frame, &outline, total)?;

    // All outlines run counter-clockwise around the direction of travel, so that outward is to their right.
    if area_vector(&wires[0]).dot(frame.z) * rates[0] < 0.0 {
        for wire in wires.iter_mut() {
            wire.reverse();
            for c in wire.iter_mut() {
                *c = c.reversed();
            }
        }
    }

    // Enough pieces for the farthest control point from the axis to fit.
    let largest = wires
        .iter()
        .flatten()
        .flat_map(|c| c.spans.iter().flatten())
        .map(|q| screw.radial(q.point()).1.norm())
        .fold(0.0f64, f64::max);
    let per_piece = (46080.0 * FIT / largest.max(1e-9)).powf(1.0 / 6.0).min(PI / 3.0);
    // At every half turn from the start. A last piece that would be a sliver goes with the one before it.
    let mut angles: Vec<f64> = vec![0.0];
    while angles[angles.len() - 1] + FACE_ANGLE < total - SHORTEST {
        angles.push(angles[angles.len() - 1] + FACE_ANGLE);
    }
    angles.push(total);
    let face_count = angles.len() - 1;

    let tag = |slot| FaceTag { feature, slot };
    let mut body = Body::default();
    let (mut start_loops, mut end_loops) = (Vec::new(), Vec::new());
    const DEGREE: usize = 5;
    for wire in &wires {
        let count = wire.len();
        // Each piece's control points along the outline, and its degree and knots that way.
        let across: Vec<(usize, Vec<f64>, Vec<Hom>)> = wire
            .iter()
            .map(|c| {
                let degree = c.degree();
                let breaks = &c.breaks;
                let mut knots = vec![0.0; degree + 1];
                for b in &breaks[1..breaks.len() - 1] {
                    knots.extend(vec![*b; degree]);
                }
                knots.extend(vec![1.0; degree + 1]);
                (degree, knots, c.control_points(degree, breaks))
            })
            .collect();
        // One vertex for each piece and station, and one ring of edges at each station.
        let vertices: Vec<Vec<usize>> = angles.iter().map(|a| wire.iter().map(|c| body.add_vertex(screw.at(c.start(), *a))).collect()).collect();
        let ring_at = |piece: usize, angle: f64| -> Vec<Hom> { across[piece].2.iter().map(|q| Hom::of(screw.at(q.point(), angle), q.w)).collect() };
        let mut rings: Vec<Vec<Coedge>> = Vec::with_capacity(angles.len());
        for (k, angle) in angles.iter().enumerate() {
            let mut ring = Vec::with_capacity(count);
            for i in 0..count {
                let (curve, a, b) = edge_curve(across[i].0, &across[i].1, &ring_at(i, *angle))?;
                ring.push(body.add_edge(curve, a, b, vertices[k][i], vertices[k][(i + 1) % count]));
            }
            rings.push(ring);
        }
        for k in 0..face_count {
            let (a, b) = (angles[k], angles[k + 1]);
            let cells = ((b - a) / per_piece).ceil().max(1.0) as usize;
            let stops: Vec<f64> = (0..=cells).map(|c| a + (b - a) * c as f64 / cells as f64).collect();
            let mut knots_v = vec![a; DEGREE + 1];
            for s in &stops[1..cells] {
                knots_v.extend(vec![*s; DEGREE]);
            }
            knots_v.extend(vec![b; DEGREE + 1]);
            let along = |q: &Hom| -> Vec<Hom> {
                let mut column = Vec::with_capacity(DEGREE * cells + 1);
                for c in 0..cells {
                    screw.quintic(q.point(), stops[c], stops[c + 1], &mut column, c > 0);
                }
                column.iter().map(|p| Hom::of(*p, q.w)).collect()
            };
            let rails: Vec<Coedge> = (0..count)
                .map(|i| -> Result<Coedge> {
                    let (curve, t0, t1) = edge_curve(DEGREE, &knots_v, &along(&across[i].2[0]))?;
                    Ok(body.add_edge(curve, t0, t1, vertices[k][i], vertices[k + 1][i]))
                })
                .collect::<Result<_>>()?;
            for i in 0..count {
                let j = (i + 1) % count;
                let (degree, knots_u, line) = &across[i];
                let (mut points, mut weights) = (Vec::with_capacity(line.len() * (DEGREE * cells + 1)), Vec::with_capacity(line.len() * (DEGREE * cells + 1)));
                for q in line {
                    for c in along(q) {
                        points.push(c.point());
                        weights.push(c.w);
                    }
                }
                let patch = Arc::new(Patch::new((*degree, DEGREE), (knots_u.clone(), knots_v.clone()), (line.len(), DEGREE * cells + 1), points, weights)?);
                let face = body.add_face(
                    Surface::Free { frame: Frame::WORLD, patch },
                    false,
                    vec![vec![rings[k][i], rails[j], rings[k + 1][i].flipped(), rails[i].flipped()]],
                    tag(SLOT_SEG_BASE + wire[i].id),
                );
                if k > 0 {
                    body.faces[face].origin.copies.push(CopyStep { feature, index: k as u32 });
                }
            }
        }
        start_loops.push(rings[0].iter().rev().map(|c| c.flipped()).collect::<Vec<_>>());
        end_loops.push(rings[face_count].clone());
    }
    // Each cap's normal runs along the direction of travel, and the start's is then flipped to face out.
    let along_travel = |f: Frame| if rates[0] >= 0.0 { f } else { Frame { origin: f.origin, x: f.x, y: -f.y, z: -f.z } };
    body.add_face(Surface::Plane(along_travel(*frame)), true, start_loops, tag(SLOT_START_CAP));
    body.add_face(Surface::Plane(along_travel(screw.frame_at(frame, total))), false, end_loops, tag(SLOT_END_CAP));
    body.compute_shells();
    validate(&body).map_err(Error::InvalidBody)?;
    Ok(body)
}
