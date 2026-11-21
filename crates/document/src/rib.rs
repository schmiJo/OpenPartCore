//! Ribs: a sketch's open curves thickened into walls that run until they meet the parts, as Onshape's rib does.
//!
//! Each curve is made a slab that runs well past the parts: parallel to the sketch's plane, a strip the thickness across
//! the plane swept sideways from each line, or normal to it, a thin wall along the curve swept along the normal. The parts
//! are cut out of the slab, and the rib is what's left next to the curve: the pieces holding points just off it. With
//! `extend`, the curve's ends carry straight on first, so that the rib reaches the part at its sides too. A piece that runs
//! on past the parts never met them on that side, and is refused rather than left sticking out.

use super::thin;
use coreshape::boolean::classify::{BodyLoc, Index};
use coreshape::build::extrude;
use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::multi::boolean_many;
use coreshape::profile::{Seg, SegGeom, Wire};
use coreshape::boolean::BoolOp;
use coreshape::topo::Body;

/// Which way a rib runs from its curves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RibDirection {
    /// Across from each line, in the sketch's plane, to the line's left as drawn, or its right with `flip`. The rib is as
    /// thick as it says across the plane, half each side.
    Parallel,
    /// Along the sketch's normal, or against it with `flip`. The rib is as thick as it says across the curve, half each side.
    Normal,
}

fn bad(why: String) -> Error {
    Error::InvalidInput(why)
}

/// A chain carried straight on at both ends by `reach`, as the same segments, so its faces keep their names.
fn extended(chain: &Wire, reach: f64) -> Wire {
    let (first, last) = (&chain.segs[0], &chain.segs[chain.segs.len() - 1]);
    let (a, da) = (first.start(), first.direction(0.0));
    let (b, db) = (last.end(), last.direction(1.0));
    let mut segs = vec![Seg { geom: SegGeom::Line { a: a - da * reach, b: a }, id: first.id }];
    segs.extend(chain.segs.iter().cloned());
    segs.push(Seg { geom: SegGeom::Line { a: b, b: b + db * reach }, id: last.id });
    Wire { segs }
}

/// The ribs along a sketch's open curves, each with the index of the curve it follows, its faces named as a thin wall's.
#[allow(clippy::too_many_arguments)]
pub(crate) fn ribs(chains: &[Wire], frame: &Frame, parts: &[Body], t: f64, direction: RibDirection, flip: bool, extend: bool, tag: u32) -> Result<Vec<(Body, u32)>> {
    if chains.is_empty() {
        return Err(bad("a rib follows a sketch's open curves, and this one has none".into()));
    }
    if parts.is_empty() {
        return Err(bad("a rib runs until it meets a part, and there's none for it to meet".into()));
    }
    if !(t >= MIN_FEATURE && t <= MAX_COORD) {
        return Err(bad(format!("a rib's thickness has to be at least {MIN_FEATURE}, and {t} isn't")));
    }
    let mut bounds = Aabb::EMPTY;
    for p in parts {
        bounds.union(&p.aabb());
    }
    let reach = bounds.diagonal() + (bounds.center() - frame.origin).norm() + 10.0 * t + 10.0;
    let on_plane = |q: Vec2| frame.origin + frame.x * q.x + frame.y * q.y;
    let mut out = Vec::new();
    for (k, chain) in chains.iter().enumerate() {
        // Each slab, with where it heads and points just off the curve, a little way along that.
        let mut slabs: Vec<(Body, Vec3, Vec<Vec3>)> = Vec::new();
        let near = (t * 1e-3).min(reach * 1e-6).max(10.0 * TOL);
        match direction {
            RibDirection::Normal => {
                let along = if flip { -frame.z } else { frame.z };
                let path = if extend { extended(chain, reach) } else { chain.clone() };
                let strip = thin::strip(&path, t / 2.0, t / 2.0)?;
                let slab = extrude(&strip, frame, if flip { -reach } else { reach }, tag)?;
                let samples = chain.segs.iter().flat_map(|s| [0.1, 0.3, 0.5, 0.7, 0.9].map(|f| on_plane(s.at(f)) + along * near)).collect();
                slabs.push((slab, along, samples));
            }
            RibDirection::Parallel => {
                for seg in &chain.segs {
                    let SegGeom::Line { a, b } = seg.geom else {
                        return Err(bad("a rib parallel to its sketch's plane runs across from straight lines, and this curve has an arc or a spline: make it normal to the plane, or draw it in lines".into()));
                    };
                    let (a3, b3) = (on_plane(a), on_plane(b));
                    let d = (b3 - a3).normalized();
                    let along = if flip { d.cross(frame.z) } else { frame.z.cross(d) };
                    // In a plane square to the sketch's, through the line: x along it, z the way the rib goes.
                    let length = a3.dist(b3);
                    let (from, to) = if extend { (-reach, length + reach) } else { (0.0, length) };
                    let line = Wire { segs: vec![Seg { geom: SegGeom::Line { a: v2(from, 0.0), b: v2(to, 0.0) }, id: seg.id }] };
                    let strip = thin::strip(&line, t / 2.0, t / 2.0)?;
                    let slab = extrude(&strip, &Frame::new(a3, along, d), reach, tag)?;
                    let samples = [0.1, 0.3, 0.5, 0.7, 0.9].map(|f| a3.lerp(b3, f) + along * near).to_vec();
                    slabs.push((slab, along, samples));
                }
            }
        }
        let mut kept: Vec<Body> = Vec::new();
        for (slab, along, samples) in slabs {
            let left = boolean_many(&slab, parts, BoolOp::Subtract)?;
            let mut found = false;
            for piece in left.solids() {
                let index = Index::new(&piece);
                if !samples.iter().any(|p| index.locate_in_body(*p) == BodyLoc::Inside) {
                    continue;
                }
                found = true;
                // Within the parts' reach, and the curve's own, or it ran on without meeting them.
                let mut room = bounds;
                for p in &samples {
                    room.add(*p);
                }
                let room = room.inflated(t + 1e-6 * reach);
                let b = piece.aabb();
                if !(room.contains(b.min) && room.contains(b.max)) {
                    let way = if direction == RibDirection::Normal { "along the sketch's normal" } else { "across from its line" };
                    return Err(bad(format!(
                        "the rib along curve {k} of its sketch runs on past the parts without meeting them, going {way} ({:.3}, {:.3}, {:.3}){}: flip it, or draw it where a part closes it in",
                        along.x,
                        along.y,
                        along.z,
                        if extend { ", with its ends carried on" } else { "" }
                    )));
                }
                kept.push(piece);
            }
            if !found {
                return Err(bad(format!("the rib along curve {k} of its sketch has nothing between it and the parts: its curve is inside them, or meets them at once")));
            }
        }
        out.extend(kept.into_iter().map(|b| (b, k as u32)));
    }
    Ok(out)
}
