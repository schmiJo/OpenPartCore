//! Booleans with many tools at once, mirroring, and sweeps along lines and arcs.
//!
//! Applying a hundred tools one by one rebuilds the target a hundred times. Tools whose boxes
//! don't touch can't interact, so they go together as separate shells of one body, and the
//! Boolean only has to run once. Tools that do overlap are joined first.

use crate::boolean::{boolean_with, BoolOp, Checks};
use crate::build::{extrude, revolve};
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::topo::Body;

/// Several bodies as one, each as its own shells. They must not touch: nothing checks that here.
pub fn compound(bodies: &[Body]) -> Body {
    let mut out = Body::default();
    for b in bodies {
        let (vs, es, fs) = (out.vertices.len(), out.edges.len(), out.faces.len());
        out.vertices.extend_from_slice(&b.vertices);
        for e in &b.edges {
            let mut e = e.clone();
            e.v0 += vs;
            e.v1 += vs;
            out.edges.push(e);
        }
        for f in &b.faces {
            let mut f = f.clone();
            for lp in &mut f.loops {
                for c in lp {
                    c.edge += es;
                }
            }
            out.faces.push(f);
        }
        out.shells.extend(b.shells.iter().map(|s| s.iter().map(|f| f + fs).collect::<Vec<usize>>()));
    }
    out
}

/// Joins tools into one body: those that may overlap by a real union, the rest side by side.
pub fn merged_tools(tools: &[Body]) -> Result<Body> {
    let n = tools.len();
    if n == 0 {
        return Err(Error::InvalidInput("there are no tools".into()));
    }
    // Clusters of boxes that overlap, by a sweep along x.
    let boxes: Vec<Aabb> = tools.iter().map(|t| t.aabb().inflated(4.0 * TOL)).collect();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| boxes[a].min.x.total_cmp(&boxes[b].min.x));
    let mut active: Vec<usize> = Vec::new();
    for &i in &order {
        active.retain(|&j| boxes[j].max.x >= boxes[i].min.x);
        for &j in &active {
            let (a, b) = (&boxes[i], &boxes[j]);
            if a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z {
                let (x, y) = (find(&mut parent, i), find(&mut parent, j));
                parent[x] = y;
            }
        }
        active.push(i);
    }
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let mut cluster_of = vec![usize::MAX; n];
    for i in 0..n {
        let root = find(&mut parent, i);
        if cluster_of[root] == usize::MAX {
            cluster_of[root] = clusters.len();
            clusters.push(Vec::new());
        }
        clusters[cluster_of[root]].push(i);
    }
    let mut joined: Vec<Body> = Vec::with_capacity(clusters.len());
    for cluster in clusters {
        let mut current = tools[cluster[0]].clone();
        for &t in &cluster[1..] {
            current = boolean_with(&current, &tools[t], BoolOp::Union, Checks::Interim)?;
        }
        joined.push(current);
    }
    if joined.len() == 1 {
        return Ok(joined.pop().unwrap());
    }
    Ok(compound(&joined))
}

/// Applies many tools to a target at once. `Union` adds them all, `Subtract` removes them all, and
/// `Intersect` keeps what lies inside any of them.
pub fn boolean_many(target: &Body, tools: &[Body], op: BoolOp) -> Result<Body> {
    boolean_many_with(target, tools, op, Checks::Full)
}

/// As `boolean_many`, with control over how much the result is checked.
pub fn boolean_many_with(target: &Body, tools: &[Body], op: BoolOp, checks: Checks) -> Result<Body> {
    if tools.is_empty() {
        return Ok(target.clone());
    }
    if op == BoolOp::Intersect {
        let tool = merged_tools(tools)?;
        return boolean_with(target, &tool, op, checks);
    }
    // Adding or removing tools can go in any order, so each round takes as many as don't touch each
    // other, as separate shells of one body. A rounded box's 20 tools need about 5 rounds this way.
    let rounds = rounds_of(tools);
    let total = rounds.len();
    let mut current = target.clone();
    for (k, round) in rounds.into_iter().enumerate() {
        let tool = if round.len() == 1 {
            tools[round[0]].clone()
        } else {
            compound(&round.iter().map(|&t| tools[t].clone()).collect::<Vec<Body>>())
        };
        let level = if k + 1 == total { checks } else { Checks::Interim };
        current = boolean_with(&current, &tool, op, level)?;
    }
    Ok(current)
}

/// Sorts tools into rounds whose members' boxes don't touch. Greedy, biggest first: any answer is
/// correct, and fewer rounds is only faster.
fn rounds_of(tools: &[Body]) -> Vec<Vec<usize>> {
    let boxes: Vec<Aabb> = tools.iter().map(|t| t.aabb().inflated(4.0 * TOL)).collect();
    let touching = |a: &Aabb, b: &Aabb| {
        a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z
    };
    let mut order: Vec<usize> = (0..tools.len()).collect();
    order.sort_by(|&a, &b| boxes[b].diagonal().total_cmp(&boxes[a].diagonal()));
    let mut rounds: Vec<Vec<usize>> = Vec::new();
    for t in order {
        match rounds.iter().position(|r| r.iter().all(|&o| !touching(&boxes[t], &boxes[o]))) {
            Some(r) => rounds[r].push(t),
            None => rounds.push(vec![t]),
        }
    }
    for r in &mut rounds {
        r.sort_unstable();
    }
    rounds
}

/// One step of a sweep. Each carries on from the direction the last one ended in, so a path is
/// always smooth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathStep {
    /// Straight ahead for a distance.
    Line { length: f64 },
    /// Bends through `angle` radians around a center `radius` away from the path. `toward` is the
    /// direction of that center, in radians counter-clockwise from the section's x axis.
    Arc { radius: f64, angle: f64, toward: f64 },
}

/// A wire under a rigid 2D map: `(u, v)` goes to `offset + m (u, v)`, with `m` orthogonal.
fn mapped(wire: &Wire, m: [[f64; 2]; 2], offset: Vec2) -> Wire {
    let at = |p: Vec2| v2(offset.x + m[0][0] * p.x + m[0][1] * p.y, offset.y + m[1][0] * p.x + m[1][1] * p.y);
    let turn = |x: f64, y: f64| (m[1][0] * x + m[1][1] * y).atan2(m[0][0] * x + m[0][1] * y);
    let sense = if m[0][0] * m[1][1] - m[0][1] * m[1][0] > 0.0 { 1.0 } else { -1.0 };
    Wire {
        segs: wire
            .segs
            .iter()
            .map(|s| {
                let geom = match s.geom {
                    SegGeom::Line { a, b } => SegGeom::Line { a: at(a), b: at(b) },
                    SegGeom::Arc { center, radius, a0, a1 } => {
                        let start = turn(a0.cos(), a0.sin());
                        SegGeom::Arc { center: at(center), radius, a0: start, a1: start + sense * (a1 - a0) }
                    }
                };
                Seg { geom, id: s.id }
            })
            .collect(),
    }
}

/// A profile under a rigid 2D map, with its wires put back in their proper directions and ids kept.
fn mapped_profile(p: &Profile, m: [[f64; 2]; 2], offset: Vec2) -> Profile {
    let outer = mapped(&p.outer, m, offset);
    let outer = if outer.signed_area() < 0.0 { outer.reversed() } else { outer };
    let holes = p
        .holes
        .iter()
        .map(|h| {
            let h = mapped(h, m, offset);
            if h.signed_area() > 0.0 {
                h.reversed()
            } else {
                h
            }
        })
        .collect();
    Profile { outer, holes }
}

/// Sweeps a profile along a smooth path of lines and arcs, starting at `frame` and heading along
/// its z axis. Straight runs are extrusions and bends are revolutions, so the surfaces stay exact:
/// planes, cylinders, cones, spheres and tori.
pub fn sweep(profile: &Profile, frame: &Frame, path: &[PathStep], feature: u32) -> Result<Body> {
    if path.is_empty() {
        return Err(Error::InvalidInput("a sweep needs at least one step".into()));
    }
    let mut section = *frame;
    let mut pieces: Vec<Body> = Vec::with_capacity(path.len());
    for (i, step) in path.iter().enumerate() {
        match *step {
            PathStep::Line { length } => {
                if !(length > 0.0) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a length must be positive, not {length}")));
                }
                pieces.push(extrude(profile, &section, length, feature)?);
                section.origin = section.origin + section.z * length;
            }
            PathStep::Arc { radius, angle, toward } => {
                if !(radius > 0.0) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a bend's radius must be positive, not {radius}")));
                }
                if !(angle > 0.0 && angle <= TAU + 1e-9) {
                    return Err(Error::InvalidInput(format!("sweep step {i}: a bend's angle must be above 0 and at most a full turn")));
                }
                let to_center = section.x * toward.cos() + section.y * toward.sin();
                let center = section.origin + to_center * radius;
                let x = -to_center;
                let axis = x.cross(section.z);
                let turning = Frame { origin: center, x, y: section.z, z: axis };
                // Distance from the axis and height along it, for a point (u, v) of the section.
                let m = [[section.x.dot(x), section.y.dot(x)], [section.x.dot(axis), section.y.dot(axis)]];
                let moved = mapped_profile(profile, m, v2(radius, 0.0));
                pieces.push(revolve(&moved, &turning, angle, feature).map_err(|e| match e {
                    Error::InvalidProfile(why) if why.contains("axis") => {
                        Error::InvalidProfile(format!("sweep step {i}: the bend is too tight, and the profile crosses its center"))
                    }
                    other => other,
                })?);
                let spin = Frame::rotation(center, axis, angle);
                section = Frame {
                    origin: spin.to_world(section.origin),
                    x: spin.dir_to_world(section.x),
                    y: spin.dir_to_world(section.y),
                    z: spin.dir_to_world(section.z),
                };
            }
        }
    }
    let total = pieces.len();
    let mut pieces = pieces.into_iter();
    let mut current = pieces.next().unwrap();
    for (k, piece) in pieces.enumerate() {
        let checks = if k + 2 == total { Checks::Full } else { Checks::Interim };
        current = boolean_with(&current, &piece, BoolOp::Union, checks)?;
    }
    Ok(current)
}
