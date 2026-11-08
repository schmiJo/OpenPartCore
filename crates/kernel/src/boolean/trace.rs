//! Turns a set of directed edges on one surface into face loops.
//!
//! Each vertex's outgoing edges are ordered by angle in the face's chart. Leaving
//! each arriving edge by its clockwise neighbour keeps the region on the left,
//! which traces every region's boundary exactly once.

use crate::error::{Error, Result};
use crate::math::*;
use crate::surface::{Chart, Surface};
use crate::topo::{Coedge, Edge};
use crate::validate::polygon_area;
use std::collections::HashMap;

/// What the tracer needs to know about the face being built.
pub struct FaceChart<'a> {
    pub surface: &'a Surface,
    pub chart: &'a Chart,
    pub flipped: bool,
}

impl FaceChart<'_> {
    /// Chart position, mirrored on flipped faces so loops always wind
    /// counter-clockwise when seen from outside the body.
    fn view(&self, p: Vec3) -> Vec2 {
        let uv = self.surface.to_chart(self.chart, p);
        if self.flipped {
            v2(uv.x, -uv.y)
        } else {
            uv
        }
    }
}

fn start_of(edges: &[Edge], c: Coedge) -> usize {
    if c.reversed {
        edges[c.edge].v1
    } else {
        edges[c.edge].v0
    }
}

fn end_of(edges: &[Edge], c: Coedge) -> usize {
    if c.reversed {
        edges[c.edge].v0
    } else {
        edges[c.edge].v1
    }
}

fn edge_length(e: &Edge) -> f64 {
    (e.t1 - e.t0) * e.curve.speed_bound()
}

/// Point on a coedge at about `dist` of travel from its start.
fn near_start(e: &Edge, c: Coedge, dist: f64) -> Vec3 {
    let dt = (e.curve.param_step_for_distance(dist)).min((e.t1 - e.t0) * 0.5);
    if c.reversed {
        e.curve.eval(e.t1 - dt)
    } else {
        e.curve.eval(e.t0 + dt)
    }
}

fn polygon_of(fc: &FaceChart, edges: &[Edge], lp: &[Coedge]) -> Vec<Vec2> {
    let mut pts = Vec::new();
    for &c in lp {
        let e = &edges[c.edge];
        let n = match &e.curve {
            crate::curve::Curve::Line { .. } if matches!(fc.surface, Surface::Plane(_)) => 1,
            crate::curve::Curve::Sampled(_) => ((e.t1 - e.t0).ceil() as usize).clamp(4, 512),
            _ => 32,
        };
        for i in 0..n {
            let s = i as f64 / n as f64;
            let s = if c.reversed { 1.0 - s } else { s };
            pts.push(fc.view(e.curve.eval(e.t0 + (e.t1 - e.t0) * s)));
        }
    }
    pts
}

fn winding_of(poly: &[Vec2], p: Vec2) -> f64 {
    let mut w = 0.0;
    for i in 0..poly.len() {
        let (u, v) = (poly[i] - p, poly[(i + 1) % poly.len()] - p);
        w += u.cross(v).atan2(u.dot(v));
    }
    w / TAU
}

/// Removes interior edges that dangle: those with an end no other edge reaches.
/// They divide nothing, so they only arise from touching contacts.
fn prune_dangling(edges: &[Edge], half: &mut Vec<Coedge>) {
    loop {
        let mut degree: HashMap<usize, usize> = HashMap::new();
        let mut seen = std::collections::HashSet::new();
        for &c in half.iter() {
            if seen.insert(c.edge) {
                *degree.entry(edges[c.edge].v0).or_insert(0) += 1;
                *degree.entry(edges[c.edge].v1).or_insert(0) += 1;
            }
        }
        let before = half.len();
        half.retain(|c| degree[&edges[c.edge].v0] > 1 && degree[&edges[c.edge].v1] > 1);
        if half.len() == before {
            return;
        }
    }
}

/// Traces directed edges into faces. Returns one list of loops per face, with the
/// outer loop first.
pub fn trace_faces(
    fc: &FaceChart,
    edges: &[Edge],
    vertices: &[Vec3],
    half_edges: &[Coedge],
) -> Result<Vec<Vec<Vec<Coedge>>>> {
    let mut half: Vec<Coedge> = half_edges.to_vec();
    prune_dangling(edges, &mut half);
    if half.is_empty() {
        return Ok(Vec::new());
    }

    // The sampling distance at each vertex is a fraction of its shortest edge, so
    // that samples land before anything else happens along any edge.
    let mut reach: HashMap<usize, f64> = HashMap::new();
    for &c in &half {
        let l = edge_length(&edges[c.edge]);
        for v in [edges[c.edge].v0, edges[c.edge].v1] {
            let r = reach.entry(v).or_insert(f64::INFINITY);
            *r = r.min(l);
        }
    }
    let angle_leaving = |c: Coedge| -> f64 {
        let v = start_of(edges, c);
        let q = near_start(&edges[c.edge], c, reach[&v] * 0.25);
        let d = fc.view(q) - fc.view(vertices[v]);
        d.y.atan2(d.x)
    };

    let mut leaving: HashMap<usize, Vec<(usize, f64)>> = HashMap::new();
    for (i, &c) in half.iter().enumerate() {
        leaving.entry(start_of(edges, c)).or_default().push((i, angle_leaving(c)));
    }

    // For each arriving edge, leave by the nearest edge clockwise from its reverse.
    let mut next = vec![usize::MAX; half.len()];
    let mut claimed = vec![false; half.len()];
    for (i, &c) in half.iter().enumerate() {
        let v = end_of(edges, c);
        let back = angle_leaving(c.flipped());
        let Some(options) = leaving.get(&v) else {
            return Err(Error::Internal(format!("edge {} ends at a vertex with no way out", c.edge)));
        };
        let mut best = None;
        let mut best_turn = f64::INFINITY;
        for &(j, angle) in options {
            let twin = half[j].edge == c.edge;
            if twin && options.len() > 1 {
                continue;
            }
            let mut turn = (back - angle).rem_euclid(TAU);
            if turn <= 1e-12 {
                turn = TAU;
            }
            if turn < best_turn {
                best_turn = turn;
                best = Some(j);
            }
        }
        let j = best.unwrap();
        if claimed[j] {
            let p = vertices[v];
            let angles: Vec<String> = options.iter().map(|&(k, a)| format!("{}{:+.4}", half[k].edge, a)).collect();
            return Err(Error::Internal(format!(
                "two edges claim the same continuation at ({:.5}, {:.5}, {:.5}) on this {}: arriving {} back angle {:.4}, options {:?}",
                p.x, p.y, p.z, fc.surface.name(), c.edge, back, angles
            )));
        }
        claimed[j] = true;
        next[i] = j;
    }

    // Follow `next` into cycles.
    let mut visited = vec![false; half.len()];
    let mut outers: Vec<(Vec<Coedge>, Vec<Vec2>, f64)> = Vec::new();
    let mut holes: Vec<(Vec<Coedge>, Vec<Vec2>)> = Vec::new();
    for start in 0..half.len() {
        if visited[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut i = start;
        while !visited[i] {
            visited[i] = true;
            cycle.push(half[i]);
            i = next[i];
        }
        let poly = polygon_of(fc, edges, &cycle);
        let area = polygon_area(&poly);
        if area > 0.0 {
            outers.push((cycle, poly, area));
        } else {
            holes.push((cycle, poly));
        }
    }

    outers.sort_by(|a, b| a.2.total_cmp(&b.2));
    let mut faces: Vec<Vec<Vec<Coedge>>> = outers.iter().map(|o| vec![o.0.clone()]).collect();
    for (cycle, _) in holes {
        // Test from the middle of an edge: holes may touch outer loops at vertices.
        let c = cycle[0];
        let e = &edges[c.edge];
        let probe = fc.view(e.curve.eval((e.t0 + e.t1) / 2.0));
        // A region holding the other side of this edge is inside the hole, and the
        // probe lies exactly on its boundary, so the winding test says nothing there.
        let home = outers.iter().position(|(loop_edges, poly, _)| {
            !loop_edges.iter().any(|o| o.edge == c.edge) && winding_of(poly, probe).abs() > 0.5
        });
        match home {
            Some(f) => faces[f].push(cycle),
            None => {
                // A clockwise cycle with no region around it is the outside of a
                // slit or sliver with no area. Anything larger is a real defect.
                let poly = polygon_of(fc, edges, &cycle);
                if polygon_area(&poly).abs() > 1e-9 {
                    return Err(Error::Internal(format!(
                        "a hole on this {} has no enclosing boundary",
                        fc.surface.name()
                    )));
                }
            }
        }
    }
    Ok(faces)
}

/// Points inside a set of loops, in chart coordinates, spread out and well away from
/// the boundary. The boundary here is a polygon that cuts corners on curved edges, so
/// a point close to it could be on the wrong side of the true curve. Candidates come
/// from scanning across the region, and the ones with the most clearance win.
pub fn interior_points(
    surface: &Surface,
    chart: &Chart,
    edges: &[Edge],
    loops: &[Vec<Coedge>],
    wanted: usize,
) -> Vec<Vec2> {
    let fc = FaceChart { surface, chart, flipped: false };
    let polys: Vec<Vec<Vec2>> = loops.iter().map(|l| polygon_of(&fc, edges, l)).collect();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &p in polys.iter().flatten() {
        lo = lo.min(p.y);
        hi = hi.max(p.y);
    }
    let clearance = |q: Vec2| {
        let mut best = f64::INFINITY;
        for poly in &polys {
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                let d = b - a;
                let len2 = d.dot(d);
                let t = if len2 > 0.0 { ((q - a).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
                best = best.min(q.dist(a + d * t));
            }
        }
        best
    };
    let mut candidates: Vec<(f64, Vec2)> = Vec::new();
    let scans = 15;
    for k in 0..scans {
        // The offset keeps scan lines off vertices and symmetry planes.
        let y = lo + (hi - lo) * ((k as f64 + 0.5) / scans as f64) + (hi - lo) * 1.37e-5;
        let mut xs = Vec::new();
        for poly in &polys {
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                if (a.y > y) != (b.y > y) {
                    xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
        }
        xs.sort_by(f64::total_cmp);
        for pair in xs.as_chunks::<2>().0 {
            // Not only span middles: a dangling contact edge can run right along a
            // line of symmetry, where every middle would land on it.
            for along in [0.5, 0.381966, 0.618034, 0.271, 0.729] {
                let q = v2(pair[0] + (pair[1] - pair[0]) * along, y);
                candidates.push((clearance(q), q));
            }
        }
    }
    candidates.sort_by(|a, b| b.0.total_cmp(&a.0));
    let Some(&(best, _)) = candidates.first() else {
        return Vec::new();
    };
    let mut picked: Vec<Vec2> = Vec::with_capacity(wanted);
    for &(c, q) in &candidates {
        if picked.len() == wanted || c < best * 0.15 {
            break;
        }
        if picked.iter().all(|p| p.dist(q) > best * 0.5) {
            picked.push(q);
        }
    }
    // A narrow region may not have room for spread-out points. Use what exists.
    for &(_, q) in &candidates {
        if picked.len() == wanted {
            break;
        }
        if !picked.contains(&q) {
            picked.push(q);
        }
    }
    picked
}
