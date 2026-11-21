//! Thin walls: a sketch's outlines and open curves thickened in its plane, for a thin extrude and a rib.
//!
//! A curve is moved sideways exactly: a line along its normal, an arc to a larger or smaller radius about the same center.
//! Where two moved curves no longer meet, as on the outside of a corner, each is carried on to where they cross, so the
//! wall's corners stay sharp, as Onshape's do. Where they overlap, on the inside, each is cut back to the same point. A
//! spline or an ellipse is refused, as moving one sideways isn't a spline.
//!
//! Faces are named by the curve they follow. The side of a curve to its right, as drawn, keeps the name an extrude of it
//! would give, `side.N`; on a closed outline that's the side away from the region. The side to its left is `inner.side.N`,
//! and the ends of an open curve's wall are `start_of.side.N` and `end_of.side.N`, after its first and last segment.

use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::profile::{Profile, Seg, SegGeom, Wire};

/// Added to a segment's id for its face on the left. The extrude moves such faces into a slot range of their own.
pub(crate) const INNER: u32 = 100_000_000;
/// Added to the id of the first and last segment of an open curve, after `INNER`, for its wall's ends. They take the hole
/// numbers no region comes near.
pub(crate) const START_OF: u32 = 998_000;
pub(crate) const END_OF: u32 = 999_000;

fn bad(why: String) -> Error {
    Error::InvalidInput(why)
}

fn place(p: Vec2) -> String {
    format!("({:.4}, {:.4})", p.x, p.y)
}

/// A curve moved sideways, as far as it goes either way: a line, or a whole circle.
#[derive(Clone, Copy, Debug)]
enum Moved {
    Line { p: Vec2, d: Vec2 },
    Circle { c: Vec2, r: f64 },
}

/// The left of a direction.
fn left(d: Vec2) -> Vec2 {
    v2(-d.y, d.x)
}

fn moved(seg: &Seg, dist: f64) -> Result<Moved> {
    match seg.geom {
        SegGeom::Line { a, b } => {
            let d = (b - a).normalized();
            Ok(Moved::Line { p: a + left(d) * dist, d })
        }
        SegGeom::Arc { center, radius, a0, a1 } => {
            // The left of a counter-clockwise arc is toward its center.
            let r = if a1 > a0 { radius - dist } else { radius + dist };
            if r < MIN_FEATURE {
                return Err(bad(format!("a wall {} thick can't follow the arc of radius {radius:.4} about {}: it's thicker than the arc is round", dist.abs(), place(center))));
            }
            Ok(Moved::Circle { c: center, r })
        }
        SegGeom::Spline(_) => Err(bad("a thin wall follows lines and arcs, and a spline or an ellipse moved sideways isn't one".into())),
    }
}

/// Where two moved curves cross, nearest `near`.
fn crossing(a: Moved, b: Moved, near: Vec2) -> Option<Vec2> {
    let mut found: Vec<Vec2> = Vec::new();
    let line_circle = |p: Vec2, d: Vec2, c: Vec2, r: f64, out: &mut Vec<Vec2>| {
        let w = p - c;
        let (bq, cq) = (d.dot(w), w.dot(w) - r * r);
        let disc = bq * bq - cq;
        if disc >= -1e-12 * r * r {
            let s = disc.max(0.0).sqrt();
            out.push(p + d * (-bq - s));
            out.push(p + d * (-bq + s));
        }
    };
    match (a, b) {
        (Moved::Line { p: p1, d: d1 }, Moved::Line { p: p2, d: d2 }) => {
            let cross = d1.x * d2.y - d1.y * d2.x;
            if cross.abs() > 1e-12 {
                let w = p2 - p1;
                let s = (w.x * d2.y - w.y * d2.x) / cross;
                found.push(p1 + d1 * s);
            }
        }
        (Moved::Line { p, d }, Moved::Circle { c, r }) | (Moved::Circle { c, r }, Moved::Line { p, d }) => line_circle(p, d, c, r, &mut found),
        (Moved::Circle { c: c1, r: r1 }, Moved::Circle { c: c2, r: r2 }) => {
            let w = c2 - c1;
            let dd = w.norm();
            if dd > 1e-12 {
                let x = (dd * dd + r1 * r1 - r2 * r2) / (2.0 * dd);
                let h2 = r1 * r1 - x * x;
                if h2 >= -1e-12 * r1 * r1 {
                    let u = w / dd;
                    let h = h2.max(0.0).sqrt();
                    found.push(c1 + u * x + left(u) * h);
                    found.push(c1 + u * x - left(u) * h);
                }
            }
        }
    }
    found.into_iter().min_by(|p, q| p.dist(near).total_cmp(&q.dist(near)))
}

/// The segment `seg` moved `dist` to its left, from `from` to `to`, which are on the moved curve. Refused where it would
/// run backwards or come to nothing, as on the inside of a corner tighter than the wall is thick.
fn moved_between(seg: &Seg, m: Moved, from: Vec2, to: Vec2, id: u32, dist: f64) -> Result<Seg> {
    let too_thick = || {
        bad(format!(
            "a wall {} thick is too thick for the segment from {} to {}: its other side would run backwards past a neighbour's",
            dist.abs(),
            place(seg.start()),
            place(seg.end())
        ))
    };
    let geom = match (seg.geom.clone(), m) {
        (SegGeom::Line { a, b }, Moved::Line { .. }) => {
            if (to - from).dot((b - a).normalized()) < MIN_FEATURE {
                return Err(too_thick());
            }
            SegGeom::Line { a: from, b: to }
        }
        (SegGeom::Arc { a0, a1, .. }, Moved::Circle { c, r }) => {
            let sweep = a1 - a0;
            let start = (from - c).y.atan2((from - c).x);
            let end = (to - c).y.atan2((to - c).x);
            let mut turned = if sweep > 0.0 { (end - start).rem_euclid(TAU) } else { -(start - end).rem_euclid(TAU) };
            // A whole circle comes back to where it started.
            if (sweep.abs() - TAU).abs() < 1e-9 && turned.abs() < 1e-9 {
                turned = sweep.signum() * TAU;
            }
            if turned.abs() * r < MIN_FEATURE || turned.abs() > sweep.abs() + PI {
                return Err(too_thick());
            }
            SegGeom::Arc { center: c, radius: r, a0: start, a1: start + turned }
        }
        _ => unreachable!("a segment moves to its own kind"),
    };
    Ok(Seg { geom, id })
}

/// A chain of segments moved `dist` to their left, each given `id(seg)`. A closed chain joins its last to its first.
pub(crate) fn moved_chain(segs: &[Seg], dist: f64, closed: bool, id: impl Fn(&Seg) -> u32) -> Result<Vec<Seg>> {
    if dist == 0.0 {
        return Ok(segs.iter().map(|s| Seg { geom: s.geom.clone(), id: id(s) }).collect());
    }
    let n = segs.len();
    let curves = segs.iter().map(|s| moved(s, dist)).collect::<Result<Vec<Moved>>>()?;
    // Where moved segment k meets the one after it, as it starts and as it ends.
    let mut joints: Vec<Vec2> = Vec::with_capacity(n);
    for k in 0..n {
        let next = (k + 1) % n;
        if !closed && next == 0 {
            joints.push(segs[k].end() + left(segs[k].direction(1.0)) * dist);
            break;
        }
        let corner = segs[k].end();
        let (ends, starts) = (corner + left(segs[k].direction(1.0)) * dist, corner + left(segs[next].direction(0.0)) * dist);
        if ends.dist(starts) <= 1e-9 * (1.0 + dist.abs()) {
            joints.push(ends);
            continue;
        }
        let meet = crossing(curves[k], curves[next], (ends + starts) / 2.0).ok_or_else(|| {
            bad(format!("a wall {} thick can't turn the corner at {}: its sides there don't meet", dist.abs(), place(corner)))
        })?;
        // A corner so sharp that its sides meet far out isn't one a wall can follow.
        if meet.dist(corner) > 50.0 * dist.abs() {
            return Err(bad(format!("the corner at {} is too sharp for a wall {} thick to follow", place(corner), dist.abs())));
        }
        joints.push(meet);
    }
    let first = if closed { joints[n - 1] } else { segs[0].start() + left(segs[0].direction(0.0)) * dist };
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let from = if k == 0 { first } else { joints[k - 1] };
        out.push(moved_between(&segs[k], curves[k], from, joints[k], id(&segs[k]), dist)?);
    }
    Ok(out)
}

fn reversed(segs: Vec<Seg>) -> Vec<Seg> {
    segs.iter().rev().map(Seg::reversed).collect()
}

/// The walls along a region's outlines: one ring for each, from `right` to the right of it to `left` to its left.
pub(crate) fn rings(region: &Profile, left: f64, right: f64) -> Result<Vec<Profile>> {
    let mut out = Vec::new();
    for (k, wire) in std::iter::once(&region.outer).chain(&region.holes).enumerate() {
        let outside = moved_chain(&wire.segs, -right, true, |s| s.id)?;
        let inside = moved_chain(&wire.segs, left, true, |s| INNER + s.id)?;
        // The outer outline runs counter-clockwise, so its left is the smaller loop, and a hole's the other way.
        let (outer, hole) = if k == 0 { (outside, reversed(inside)) } else { (reversed(inside), outside) };
        let ring = Profile { outer: Wire { segs: outer }, holes: vec![Wire { segs: hole }] };
        ring.validate().map_err(|e| bad(format!("a wall {} thick along this outline crosses itself: {e}", left + right)))?;
        out.push(ring);
    }
    Ok(out)
}

/// The wall along an open curve, from `right` to the right of it to `left` to its left, with straight ends.
pub(crate) fn strip(wire: &Wire, left: f64, right: f64) -> Result<Profile> {
    let segs = &wire.segs;
    let (first, last) = (segs[0].id, segs[segs.len() - 1].id);
    let right_side = moved_chain(segs, -right, false, |s| s.id)?;
    let left_side = moved_chain(segs, left, false, |s| INNER + s.id)?;
    let (r_end, l_end) = (right_side[right_side.len() - 1].end(), left_side[left_side.len() - 1].end());
    let (l_start, r_start) = (left_side[0].start(), right_side[0].start());
    let mut outline = right_side;
    outline.push(Seg { geom: SegGeom::Line { a: r_end, b: l_end }, id: INNER + END_OF + last });
    outline.extend(reversed(left_side));
    outline.push(Seg { geom: SegGeom::Line { a: l_start, b: r_start }, id: INNER + START_OF + first });
    let profile = Profile::new(Wire { segs: outline });
    profile.validate().map_err(|e| bad(format!("a wall {} thick along this curve crosses itself: {e}", left + right)))?;
    Ok(profile)
}
