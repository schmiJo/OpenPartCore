//! Fillets with more than one radius: different radii on different edges, which can meet at a corner, and a radius that
//! varies along a chain of edges.
//!
//! Where edges of different radii meet at a corner, the corner comes out of doing them in the right order. The largest goes
//! first, and whichever edges then run smoothly around it become one chain. If those have different radii, the radius goes
//! smoothly from one to the other around the piece between them, so no extra patch is needed and nothing is left sharp. The
//! exception is an edge that's the odd one out at a corner, convex among concave or the other way round, which always goes
//! first, as nothing else can run around a corner like that.

use crate::blend::{fillet_route, odd_ones_at_corners, propagate_tangents, same_edges_in};
use crate::error::{Error, Result};
use crate::math::*;
use crate::rolling::{blend_varying, Varying};
use crate::topo::Body;

/// What radius a set of edges has.
#[derive(Clone, Debug, PartialEq)]
pub enum Radius {
    /// The same all along.
    Constant(f64),
    /// By how far along the chain of edges, from 0 to 1, in straight lines between these. It's measured from the end nearest
    /// `start`, or without one, from the end with the lowest x, then y, then z.
    Along { stations: Vec<(f64, f64)>, start: Option<Vec3> },
}

impl Radius {
    fn largest(&self) -> f64 {
        match self {
            Radius::Constant(r) => *r,
            Radius::Along { stations, .. } => stations.iter().map(|s| s.1).fold(0.0, f64::max),
        }
    }

    fn check(&self) -> Result<()> {
        let fine = |r: f64| r.is_finite() && r >= MIN_FEATURE;
        match self {
            Radius::Constant(r) if !fine(*r) => Err(Error::InvalidInput(format!("fillet radius {r} is too small"))),
            Radius::Constant(_) => Ok(()),
            Radius::Along { stations, .. } => {
                if stations.is_empty() {
                    return Err(Error::InvalidInput("a radius along a chain of edges needs at least one station".into()));
                }
                for (at, r) in stations {
                    if !at.is_finite() || !(0.0..=1.0).contains(at) {
                        return Err(Error::InvalidInput(format!("a radius's place along its edges has to be from 0 to 1, and this is {at}")));
                    }
                    if !fine(*r) {
                        return Err(Error::InvalidInput(format!("fillet radius {r} is too small")));
                    }
                }
                if stations.windows(2).any(|w| !(w[1].0 > w[0].0)) {
                    return Err(Error::InvalidInput("a radius's places along its edges have to go up in order".into()));
                }
                Ok(())
            }
        }
    }
}

/// A set of edges and its radius, while they're being worked through.
struct Set {
    edges: Vec<usize>,
    radius: Radius,
}

/// Fillets sets of edges, each with its own radius. Faces are named as a plain fillet's are.
pub fn fillet_radii(body: &Body, sets: &[(Vec<usize>, Radius)], feature: u32) -> Result<Body> {
    let mut seen: Vec<usize> = Vec::new();
    for (edges, radius) in sets {
        radius.check()?;
        for &e in edges {
            if e >= body.edges.len() {
                return Err(Error::InvalidInput(format!("there is no edge {e}")));
            }
            if seen.contains(&e) {
                return Err(Error::InvalidInput(format!("edge {e} has been given two radii")));
            }
            seen.push(e);
        }
    }
    let mut work = body.clone();
    let mut left: Vec<Set> = sets.iter().filter(|s| !s.0.is_empty()).map(|s| Set { edges: s.0.clone(), radius: s.1.clone() }).collect();
    while !left.is_empty() {
        let (next, this) = next_group(&work, &mut left);
        let done = match &this {
            Radius::Constant(r) => {
                let chain = propagate_tangents(&work, &next)?;
                let others: Vec<(usize, Radius)> = left.iter().flat_map(|s| s.edges.iter().map(|e| (*e, s.radius.clone()))).filter(|x| chain.contains(&x.0)).collect();
                if others.is_empty() {
                    fillet_route(&work, &chain, &next, *r, feature)?
                } else {
                    // They run into each other smoothly, so they're one chain with its radius going from one to the other.
                    let mut by_edge: Vec<(usize, f64)> = next.iter().map(|e| (*e, *r)).collect();
                    for (e, radius) in &others {
                        let Radius::Constant(other) = radius else {
                            return Err(Error::Unsupported("a radius that varies along its edges can't yet run smoothly into edges with another radius".into()));
                        };
                        by_edge.push((*e, *other));
                    }
                    let picked: Vec<usize> = by_edge.iter().map(|x| x.0).collect();
                    for s in &mut left {
                        s.edges.retain(|e| !picked.contains(e));
                    }
                    blend_varying(&work, &picked, &Varying::ByEdge(by_edge), feature)?
                }
            }
            Radius::Along { stations, start } => blend_varying(&work, &next, &Varying::Along { stations: stations.clone(), start: *start }, feature)?,
        };
        for s in &mut left {
            s.edges = same_edges_in(&work, &s.edges, &done);
        }
        left.retain(|s| !s.edges.is_empty());
        work = done;
    }
    Ok(work)
}

/// Takes the edges to do next out of what's left, with their radius: the odd ones out at corners first, then the largest.
fn next_group(work: &Body, left: &mut Vec<Set>) -> (Vec<usize>, Radius) {
    let all: Vec<usize> = left.iter().flat_map(|s| s.edges.iter().copied()).collect();
    let largest = left.iter().map(|s| s.radius.largest()).fold(0.0, f64::max);
    let odd = odd_ones_at_corners(work, &all, largest);
    let radius = left
        .iter()
        .find(|s| s.edges.iter().any(|e| odd.contains(e)))
        .or_else(|| left.iter().max_by(|a, b| a.radius.largest().total_cmp(&b.radius.largest())))
        .map_or(Radius::Constant(largest), |s| s.radius.clone());
    let mut next = Vec::new();
    for s in left.iter_mut().filter(|s| s.radius == radius) {
        let any_odd = s.edges.iter().any(|e| odd.contains(e));
        s.edges.retain(|e| {
            let take = !any_odd || odd.contains(e);
            if take {
                next.push(*e);
            }
            !take
        });
    }
    left.retain(|s| !s.edges.is_empty());
    (next, radius)
}
