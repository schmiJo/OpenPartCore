//! Cleans up a Boolean result: neighbouring faces on one surface become one face,
//! and edges split by a vertex nothing else uses become one edge. Without this,
//! each operation leaves seams behind and models get slower with every feature.
//!
//! Curved faces only merge while they stay within 180 degrees, which is the
//! invariant that keeps seams and poles away from all the algorithms.

use super::ssi::coincident;
use super::trace::{trace_faces, FaceChart};
use crate::curve::Curve;
use crate::error::Result;
use crate::math::*;
use crate::surface::Surface;
use crate::topo::{Body, Coedge, Edge, Face};
use crate::hash::{HashMap, HashSet};

pub fn simplify(body: Body) -> Result<Body> {
    let body = merge_faces(body)?;
    let body = merge_edges(body);
    Ok(compact(body))
}

fn boundary_points(body: &Body, face: &Face) -> Vec<Vec3> {
    let mut pts = Vec::new();
    for lp in &face.loops {
        for &c in lp {
            for i in 0..8 {
                pts.push(body.coedge_point(c, i as f64 / 8.0));
            }
        }
    }
    pts
}

/// Whether these points, all on one surface, fit within a single face's limits.
pub(crate) fn within_half_turn(surface: &Surface, points: &[Vec3]) -> bool {
    let span = |angles: &[f64]| {
        let centre = angular_center(angles);
        angles.iter().map(|a| wrap_pi(a - centre).abs()).fold(0.0, f64::max)
    };
    let limit = PI / 2.0 + 1e-6;
    let frame = surface.frame();
    match *surface {
        Surface::Plane(_) | Surface::Extruded { .. } | Surface::Free { .. } => true,
        Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Revolved { .. } => {
            let us: Vec<f64> = points
                .iter()
                .filter_map(|&p| {
                    let l = frame.to_local(p);
                    (l.x.hypot(l.y) > 1e-9).then(|| l.y.atan2(l.x))
                })
                .collect();
            span(&us) <= limit
        }
        Surface::Torus { major, .. } => {
            let us: Vec<f64> = points
                .iter()
                .map(|&p| {
                    let l = frame.to_local(p);
                    l.y.atan2(l.x)
                })
                .collect();
            let vs: Vec<f64> = points
                .iter()
                .map(|&p| {
                    let l = frame.to_local(p);
                    l.z.atan2(l.x.hypot(l.y) - major)
                })
                .collect();
            span(&us) <= limit && span(&vs) <= limit
        }
        Surface::Sphere { .. } => {
            let mut mean = Vec3::ZERO;
            for &p in points {
                mean += (p - frame.origin).normalized();
            }
            if mean.norm() < 1e-6 {
                return false;
            }
            let c = mean.normalized();
            points.iter().all(|&p| (p - frame.origin).dot(c) >= -1e-6)
        }
    }
}

fn merge_faces(body: Body) -> Result<Body> {
    let size = body.aabb().diagonal();
    let n = body.faces.len();
    let uses = body.edge_uses();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while p[r] != r {
            p[r] = p[p[r]];
            r = p[r];
        }
        r
    }
    let mut points: Vec<Vec<Vec3>> = body.faces.iter().map(|f| boundary_points(&body, f)).collect();
    for (ei, list) in uses.iter().enumerate() {
        let [a, b] = list.as_slice() else {
            continue;
        };
        let (ra, rb) = (find(&mut parent, a.0), find(&mut parent, b.0));
        if ra == rb {
            continue;
        }
        let (fa, fb) = (&body.faces[a.0], &body.faces[b.0]);
        if coincident(&fa.surface, &fb.surface, size).is_none() || !one_surface_fits_both(&fa.surface, &fb.surface) {
            continue;
        }
        let mid = body.edges[ei].eval_mid();
        if fa.outward_normal(mid).dot(fb.outward_normal(mid)) <= 0.0 {
            continue;
        }
        let mut joined = points[ra].clone();
        joined.extend_from_slice(&points[rb]);
        if !within_half_turn(&fa.surface, &joined) {
            continue;
        }
        parent[ra] = rb;
        points[rb] = joined;
    }

    let mut groups: HashMap<usize, Vec<usize>> = HashMap::default();
    for f in 0..n {
        let r = find(&mut parent, f);
        groups.entry(r).or_default().push(f);
    }
    if groups.len() == n {
        return Ok(body);
    }

    let mut roots: Vec<usize> = groups.keys().copied().collect();
    roots.sort_unstable();
    let mut out = Body {
        vertices: body.vertices.clone(),
        edges: body.edges.clone(),
        faces: Vec::with_capacity(groups.len()),
        shells: Vec::new(),
        tolerance: body.tolerance,
        sheet: body.sheet,
    };
    for root in roots {
        let group = &groups[&root];
        if let [only] = group.as_slice() {
            out.faces.push(body.faces[*only].clone());
            continue;
        }
        // Edges used in both directions inside the group are now interior.
        let mut half: Vec<Coedge> = Vec::new();
        for &f in group {
            half.extend(body.faces[f].loops.iter().flatten().copied());
        }
        let present: HashSet<(usize, bool)> = half.iter().map(|c| (c.edge, c.reversed)).collect();
        half.retain(|c| !present.contains(&(c.edge, !c.reversed)));

        let lead = &body.faces[group[0]];
        // The smallest tag names the merged face, so names stay stable, and its origin goes with it.
        let (tag, origin) = group.iter().map(|&f| (body.faces[f].tag, &body.faces[f].origin)).min().unwrap_or((lead.tag, &lead.origin));
        // A chart centred on the whole group, not on any one of its faces.
        let all_loops: Vec<Vec<Coedge>> = group.iter().flat_map(|&f| body.faces[f].loops.iter().cloned()).collect();
        let chart = body.fit_face_chart(&lead.surface, lead.flipped, &all_loops);
        let fc = FaceChart { surface: &lead.surface, chart: &chart, flipped: lead.flipped };
        for loops in trace_faces(&fc, &body.edges, &body.vertices, &half)? {
            let i = out.add_face(lead.surface.clone(), lead.flipped, loops, tag);
            out.faces[i].origin = origin.clone();
        }
    }
    out.compute_shells();
    Ok(out)
}

/// A swept curve exists only along its curve. So a piece of one can coincide with a true cylinder, or with
/// another piece, without being able to describe the other's face. Merged faces need the same curve.
fn one_surface_fits_both(a: &Surface, b: &Surface) -> bool {
    match (a, b) {
        (Surface::Extruded { curve: c, .. }, Surface::Extruded { curve: d, .. }) | (Surface::Revolved { curve: c, .. }, Surface::Revolved { curve: d, .. }) => c == d,
        (Surface::Free { patch: p, .. }, Surface::Free { patch: q, .. }) => p == q,
        (Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. }, _) | (_, Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. }) => false,
        _ => true,
    }
}

/// Whether two edges lie on the same curve, so they can become one.
fn same_curve(a: &Edge, b: &Edge) -> bool {
    match (&a.curve, &b.curve) {
        (Curve::Spline(x), Curve::Spline(y)) => x == y,
        (Curve::Line { origin, dir }, Curve::Line { origin: o2, dir: d2 }) => {
            let d = *o2 - *origin;
            dir.cross(*d2).norm() < 1e-9 && (d - *dir * d.dot(*dir)).norm() <= TOL
        }
        (Curve::Circle { frame: f, radius: r }, Curve::Circle { frame: g, radius: s }) => {
            (r - s).abs() <= TOL && f.origin.dist(g.origin) <= TOL && f.z.cross(g.z).norm() < 1e-9
        }
        (Curve::Ellipse { frame: f, major: m, minor: n }, Curve::Ellipse { frame: g, major: p, minor: q }) => {
            (m - p).abs() <= TOL
                && (n - q).abs() <= TOL
                && f.origin.dist(g.origin) <= TOL
                && f.z.cross(g.z).norm() < 1e-9
                && f.x.cross(g.x).norm() < 1e-9
        }
        _ => false,
    }
}

fn merge_edges(mut body: Body) -> Body {
    loop {
        // Vertices with exactly two edges.
        // Only edges still used by a loop count: merged ones stay in the array until
        // `compact` removes them, and must not be picked up again.
        let live: HashSet<usize> = body.faces.iter().flat_map(|f| f.loops.iter().flatten().map(|c| c.edge)).collect();
        let mut at_vertex: HashMap<usize, Vec<usize>> = HashMap::default();
        for &i in &live {
            let e = &body.edges[i];
            at_vertex.entry(e.v0).or_default().push(i);
            at_vertex.entry(e.v1).or_default().push(i);
        }
        // A vertex that was separated at a pinch has a twin at the same position.
        // Merging across it would leave that twin sitting on the middle of an edge,
        // and the two sides of the pinch must keep a common point in every mesh.
        let mut at_position: HashMap<[u64; 3], usize> = HashMap::default();
        for &v in at_vertex.keys() {
            let p = body.vertices[v];
            *at_position.entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).or_insert(0) += 1;
        }
        let vertices = &body.vertices;
        let pinched = |v: usize| {
            let p = vertices[v];
            at_position[&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]] > 1
        };
        let mut candidates: Vec<(usize, usize, usize)> = at_vertex
            .iter()
            .filter(|(&v, list)| list.len() == 2 && list[0] != list[1] && !pinched(v))
            .map(|(&v, list)| (v, list[0], list[1]))
            .collect();
        candidates.sort_unstable();

        let mut changed = false;
        'next: for (w, ea, eb) in candidates {
            let (a, b) = (body.edges[ea].clone(), body.edges[eb].clone());
            if !same_curve(&a, &b) {
                continue;
            }
            // Far ends. An arc must not close on itself.
            let far_a = if a.v0 == w { a.v1 } else { a.v0 };
            let far_b = if b.v0 == w { b.v1 } else { b.v0 };
            if far_a == far_b {
                continue;
            }
            // Parameter on a's curve at far_a, then extend past w by b's span.
            let (t_far, t_w) = if a.v0 == w { (a.t1, a.t0) } else { (a.t0, a.t1) };
            let span = b.t1 - b.t0;
            let t_end = if t_w > t_far { t_w + span } else { t_w - span };
            let new = {
                let (t0, t1, v0, v1) =
                    if t_far < t_end { (t_far, t_end, far_a, far_b) } else { (t_end, t_far, far_b, far_a) };
                Edge { curve: a.curve.clone(), t0, t1, v0, v1 }
            };
            // Both edges must be consecutive in every loop that uses them.
            let id = body.edges.len();
            let mut updated = Vec::new();
            for (fi, face) in body.faces.iter().enumerate() {
                for (li, lp) in face.loops.iter().enumerate() {
                    let m = lp.len();
                    let used: Vec<usize> = (0..m).filter(|&k| lp[k].edge == ea || lp[k].edge == eb).collect();
                    if used.is_empty() {
                        continue;
                    }
                    if used.len() != 2 {
                        continue 'next;
                    }
                    // The first of the pair is the one that arrives at `w`.
                    let Some(&first) = used.iter().find(|&&k| {
                        let n = lp[(k + 1) % m];
                        n.edge != lp[k].edge && (n.edge == ea || n.edge == eb) && start_vertex(&body.edges, n) == w
                    }) else {
                        continue 'next;
                    };
                    let second = (first + 1) % m;
                    if m < 3 {
                        continue 'next;
                    }
                    let start = start_vertex(&body.edges, lp[first]);
                    let merged = Coedge { edge: id, reversed: new.v0 != start };
                    let mut out = Vec::with_capacity(m - 1);
                    for k in 0..m {
                        if k == first {
                            out.push(merged);
                        } else if k != second {
                            out.push(lp[k]);
                        }
                    }
                    updated.push((fi, li, out));
                }
            }
            for (fi, li, out) in updated {
                body.faces[fi].loops[li] = out;
            }
            body.edges.push(new);
            changed = true;
            break;
        }
        if !changed {
            return body;
        }
    }
}

fn start_vertex(edges: &[Edge], c: Coedge) -> usize {
    if c.reversed {
        edges[c.edge].v1
    } else {
        edges[c.edge].v0
    }
}

/// Drops unused vertices and edges, and renumbers.
fn compact(mut body: Body) -> Body {
    let mut edge_map: HashMap<usize, usize> = HashMap::default();
    let mut vertex_map: HashMap<usize, usize> = HashMap::default();
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    for face in &mut body.faces {
        for lp in &mut face.loops {
            for c in lp.iter_mut() {
                let new_id = *edge_map.entry(c.edge).or_insert_with(|| {
                    let mut e = body.edges[c.edge].clone();
                    for v in [&mut e.v0, &mut e.v1] {
                        *v = *vertex_map.entry(*v).or_insert_with(|| {
                            vertices.push(body.vertices[*v]);
                            vertices.len() - 1
                        });
                    }
                    edges.push(e);
                    edges.len() - 1
                });
                c.edge = new_id;
            }
        }
    }
    body.edges = edges;
    body.vertices = vertices;
    body.compute_shells();
    body
}
