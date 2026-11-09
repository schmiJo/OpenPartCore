//! A solid from faces: the space between them and a copy moved along their normals, closed around the edge by walls
//! that follow those normals.
//!
//! The walls and the far faces are exact wherever they have a form of their own: a straight edge whose normal doesn't
//! change gives a plane, and a round edge whose normal keeps its angle to the axis gives a plane, cylinder or cone. Far
//! faces are always exact, including on free-form faces, as those stay a surface moved by a distance. Anywhere else a wall
//! is fitted, ruled along the true normals, through stations added until it's within `FIT` of where it should be.

use crate::build::{SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::{interpolation_knots, Collocation, Patch};
use crate::rolling::FIT;
use crate::spline::Hom;
use crate::surface::Surface;
use crate::tess::{tessellate, TessParams};
use crate::topo::{Body, Coedge, Edge, Face, FaceOrigin, FaceTag};
use crate::tweak::{boundary_samples, moved_surface};
use crate::validate::validate;
use std::collections::HashMap;
use std::sync::Arc;

const MAX_STATIONS: usize = 1024;

fn refuse(what: String) -> Error {
    Error::InvalidInput(format!("thickening these faces: {what}"))
}

/// An edge lifted along its normals: where it ends up, and the surface it sweeps, if that has a form of its own.
struct Lifted {
    top: (Curve, f64, f64),
    wall: Wall,
}

enum Wall {
    Exact(Surface),
    Fitted(Arc<Patch>),
}

/// A surface swept by a straight piece going around an axis, from radius `r` at height 0 by `rho` out and `up` along it.
fn around_axis(frame: &Frame, r: f64, rho: f64, up: f64) -> Surface {
    if rho.abs() <= 1e-9 * up.abs() {
        Surface::Cylinder { frame: *frame, radius: r }
    } else if up.abs() <= 1e-9 * rho.abs() {
        Surface::Plane(*frame)
    } else {
        let flip = Frame { origin: frame.origin, x: frame.x, y: -frame.y, z: -frame.z };
        // It widens along its own z, which is whichever way this piece goes as it gets further from the axis.
        let f = if rho * up > 0.0 { *frame } else { flip };
        Surface::Cone { frame: f, radius: r, half_angle: (rho / up).abs().atan() }
    }
}

/// A patch ruled along an edge from `from(t)` to `to(t)`, as a cubic through stations, with `t` over `(t0, t1)`.
/// Stops at `breaks`, where the edge's curve has a knot, as a cubic can't fit across those. Stations double until
/// it's within `FIT` halfway between them.
fn ruled(t0: f64, t1: f64, breaks: &[f64], from: &dyn Fn(f64) -> Vec3, to: &dyn Fn(f64) -> Vec3) -> Result<Patch> {
    let mut stops: Vec<f64> = vec![t0];
    stops.extend(breaks.iter().copied().filter(|b| *b > t0 + 1e-9 * (t1 - t0) && *b < t1 - 1e-9 * (t1 - t0)));
    stops.push(t1);
    let mut per = 4usize;
    loop {
        let mut knots_v: Vec<f64> = Vec::new();
        let mut near: Vec<Hom> = Vec::new();
        let mut far: Vec<Hom> = Vec::new();
        for w in stops.windows(2) {
            let params: Vec<f64> = (0..=per).map(|i| w[0] + (w[1] - w[0]) * i as f64 / per as f64).collect();
            let (degree, knots) = interpolation_knots(&params);
            let collocation = Collocation::new(&params, degree, &knots).ok_or_else(|| Error::NearDegenerate("a wall's stations can't be interpolated".into()))?;
            let solve = |f: &dyn Fn(f64) -> Vec3| collocation.solve(&params.iter().map(|t| Hom::of(f(*t), 1.0)).collect::<Vec<_>>());
            let (a, b) = (solve(from), solve(to));
            let skip = usize::from(!near.is_empty());
            near.extend_from_slice(&a[skip..]);
            far.extend_from_slice(&b[skip..]);
            if knots_v.is_empty() {
                knots_v = knots;
            } else {
                knots_v.pop();
                knots_v.extend_from_slice(&knots[degree + 1..]);
            }
        }
        let count = near.len();
        let mut points = Vec::with_capacity(2 * count);
        let mut weights = Vec::with_capacity(2 * count);
        for line in [&near, &far] {
            for q in line.iter() {
                points.push(q.point());
                weights.push(q.w);
            }
        }
        let patch = Patch::new((1, 3), (vec![0.0, 0.0, 1.0, 1.0], knots_v), (2, count), points, weights)?;
        let mut worst = 0.0f64;
        for w in stops.windows(2) {
            for i in 0..per {
                let t = w[0] + (w[1] - w[0]) * (i as f64 + 0.5) / per as f64;
                worst = worst.max(patch.eval(0.0, t).dist(from(t))).max(patch.eval(1.0, t).dist(to(t)));
            }
        }
        if worst <= FIT {
            return Ok(patch);
        }
        per *= 2;
        if per * (stops.len() - 1) > MAX_STATIONS {
            return Err(Error::Unsupported(format!("a wall along this edge is still {worst:.1e} off at {MAX_STATIONS} stations, which is more than the tolerance of {FIT:.1e}")));
        }
    }
}

/// Where an edge goes, moved `d` along the normals `n`.
fn lift(edge: &Edge, n: &dyn Fn(Vec3) -> Vec3, d: f64, outward: Vec3) -> Result<Lifted> {
    let same = 1e-9;
    match &edge.curve {
        Curve::Line { origin, dir } => {
            let at = |t: f64| *origin + *dir * t;
            let first = n(at(edge.t0));
            if (0..=4).all(|i| n(at(edge.t0 + (edge.t1 - edge.t0) * i as f64 / 4.0)).dist(first) < same) {
                let shift = first * d;
                let top = Curve::Line { origin: *origin + shift, dir: *dir };
                let wall = Surface::Plane(Frame::new(at(edge.t0), outward, *dir));
                return Ok(Lifted { top: (top, edge.t0, edge.t1), wall: Wall::Exact(wall) });
            }
        }
        Curve::Circle { frame, radius } => {
            // Out from the axis and along it. If those stay the same all the way round, it's a shape of revolution.
            let parts = |t: f64| {
                let p = edge.curve.eval(t);
                let m = n(p);
                let radial = (p - frame.origin - frame.z * (p - frame.origin).dot(frame.z)).normalized();
                (m.dot(radial), m.dot(frame.z), m.dot(frame.z.cross(radial)))
            };
            let (rho, up, turn) = parts(edge.t0);
            let steady = (0..=4).all(|i| {
                let q = parts(edge.t0 + (edge.t1 - edge.t0) * i as f64 / 4.0);
                (q.0 - rho).abs() < same && (q.1 - up).abs() < same && q.2.abs() < same
            });
            if steady && turn.abs() < same {
                let r = radius + d * rho;
                if !(r > MIN_FEATURE) {
                    return Err(refuse(format!("an edge of radius {radius} would shrink away to nothing")));
                }
                let moved = Frame { origin: frame.origin + frame.z * (d * up), ..*frame };
                return Ok(Lifted { top: (Curve::Circle { frame: moved, radius: r }, edge.t0, edge.t1), wall: Wall::Exact(around_axis(frame, *radius, rho, up)) });
            }
        }
        _ => {}
    }
    // Nothing simpler fits, so it's fitted. It breaks at the curve's own knots.
    let breaks: Vec<f64> = match &edge.curve {
        Curve::Spline(c) => c.bezier_spans().iter().map(|s| s.1).collect(),
        _ => Vec::new(),
    };
    let from = |t: f64| edge.curve.eval(t);
    let to = |t: f64| {
        let p = edge.curve.eval(t);
        p + n(p) * d
    };
    let patch = Arc::new(ruled(edge.t0, edge.t1, &breaks, &from, &to)?);
    let top = patch.iso_curve(1.0, true)?;
    let (a, b) = top.domain();
    Ok(Lifted { top: (Curve::Spline(Arc::new(top)), a, b), wall: Wall::Fitted(patch) })
}

/// Makes a solid from the faces given, reaching `thickness` along their outward normals, or inward if that's negative. Its
/// near faces are named as this feature's start cap and its far faces as the end cap, each remembering the face it came
/// from, and its walls are named in order around the edge, each remembering the two faces that edge was between.
pub fn thicken(body: &Body, faces: &[usize], thickness: f64, feature: u32) -> Result<Body> {
    if !(thickness.is_finite() && thickness.abs() >= MIN_FEATURE) {
        return Err(refuse(format!("a thickness of {thickness} is too small")));
    }
    if faces.is_empty() {
        return Err(refuse("it takes at least one face".into()));
    }
    if let Some(&bad) = faces.iter().find(|&&f| f >= body.faces.len()) {
        return Err(refuse(format!("there is no face {bad}")));
    }
    let mut chosen = vec![false; body.faces.len()];
    for &f in faces {
        chosen[f] = true;
    }
    let uses = body.edge_uses();
    let d = thickness;
    // Inward, the original surface faces out of the new solid. Outward, it faces in, and the moved copy faces out.
    let outward_thick = d > 0.0;

    // Every vertex of those faces has one normal, or they meet at a sharp edge.
    let mut normal_at: HashMap<usize, Vec3> = HashMap::new();
    let mut point_face: HashMap<usize, usize> = HashMap::new();
    for &f in faces {
        let face = &body.faces[f];
        for c in face.loops.iter().flatten() {
            for v in [body.start_vertex(*c), body.end_vertex(*c)] {
                let n = normal_of(face, body.vertices[v]);
                if let Some(old) = normal_at.get(&v) {
                    if old.dist(n) > 1e-6 {
                        return Err(refuse("two of them meet at a sharp edge or corner. Thicken them one at a time, or a smooth set".into()));
                    }
                } else {
                    normal_at.insert(v, n);
                    point_face.insert(v, f);
                }
            }
        }
    }

    let mut out = Body { tolerance: body.tolerance, ..Body::default() };
    let mut near_vertex: HashMap<usize, usize> = HashMap::new();
    let mut far_vertex: HashMap<usize, usize> = HashMap::new();
    let mut list: Vec<usize> = normal_at.keys().copied().collect();
    list.sort_unstable();
    for v in list {
        let p = body.vertices[v];
        near_vertex.insert(v, out.add_vertex(p));
        far_vertex.insert(v, out.add_vertex(p + normal_at[&v] * d));
    }

    // Near and far edges, with what's needed to build a wall on those around the edge of the selection.
    let mut near_edge: HashMap<usize, usize> = HashMap::new();
    let mut far_edge: HashMap<usize, usize> = HashMap::new();
    let mut walls: HashMap<usize, (Wall, usize)> = HashMap::new();
    for &f in faces {
        let face = &body.faces[f];
        for lp in &face.loops {
            for c in lp {
                let e = c.edge;
                if near_edge.contains_key(&e) {
                    continue;
                }
                let edge = &body.edges[e];
                let (a, b) = (near_vertex[&edge.v0], near_vertex[&edge.v1]);
                out.edges.push(Edge { curve: edge.curve.clone(), t0: edge.t0, t1: edge.t1, v0: a, v1: b });
                near_edge.insert(e, out.edges.len() - 1);
                let chosen_uses: Vec<usize> = uses[e].iter().map(|u| u.0).filter(|g| chosen[*g]).collect();
                let on_boundary = chosen_uses.len() == 1;
                if !on_boundary {
                    // Between two of them: they have to meet smoothly all along it.
                    let (g, h) = (&body.faces[chosen_uses[0]], &body.faces[chosen_uses[1]]);
                    for i in 0..=4 {
                        let p = edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / 4.0);
                        if normal_of(g, p).dist(normal_of(h, p)) > 1e-6 {
                            return Err(refuse("two of them meet at a sharp edge. Thicken them one at a time, or a smooth set".into()));
                        }
                    }
                }
                let normal = |p: Vec3| normal_of(face, p);
                // The wall's own outward direction: away from the face, to the right of the way the loop runs.
                let mid = edge.eval_mid();
                let tangent = edge.curve.derivative((edge.t0 + edge.t1) / 2.0) * if c.reversed { -1.0 } else { 1.0 };
                let out_of_face = tangent.cross(normal(mid)).normalized();
                let lifted = lift(edge, &normal, d, out_of_face)?;
                let (va, vb) = (far_vertex[&edge.v0], far_vertex[&edge.v1]);
                let (curve, t0, t1) = lifted.top;
                let (pa, pb) = (out.vertices[va], out.vertices[vb]);
                if curve.eval(t0).dist(pa) > 10.0 * TOL.max(body.tolerance) || curve.eval(t1).dist(pb) > 10.0 * TOL.max(body.tolerance) {
                    return Err(Error::Internal(format!("a lifted edge misses its corners by {:.1e}", curve.eval(t0).dist(pa).max(curve.eval(t1).dist(pb)))));
                }
                out.edges.push(Edge { curve, t0, t1, v0: va, v1: vb });
                far_edge.insert(e, out.edges.len() - 1);
                if on_boundary {
                    walls.insert(e, (lifted.wall, f));
                }
            }
        }
    }

    // A straight edge up from each corner where a wall starts.
    let mut riser: HashMap<usize, usize> = HashMap::new();
    let mut boundary: Vec<usize> = walls.keys().copied().collect();
    boundary.sort_unstable();
    for &e in &boundary {
        for v in [body.edges[e].v0, body.edges[e].v1] {
            if !riser.contains_key(&v) {
                let (a, b) = (near_vertex[&v], far_vertex[&v]);
                let c = out.add_edge(Curve::line_through(out.vertices[a], out.vertices[b]), 0.0, d.abs(), a, b);
                riser.insert(v, c.edge);
            }
        }
    }

    let origin_of = |tags: Vec<FaceTag>| {
        let mut between = tags;
        between.sort();
        between.dedup();
        FaceOrigin { copies: Vec::new(), between }
    };
    let tag = |slot| FaceTag { feature, slot };
    let reversed = |lp: &[Coedge], map: &HashMap<usize, usize>| -> Vec<Coedge> { lp.iter().rev().map(|c| Coedge { edge: map[&c.edge], reversed: !c.reversed }).collect() };
    let same = |lp: &[Coedge], map: &HashMap<usize, usize>| -> Vec<Coedge> { lp.iter().map(|c| Coedge { edge: map[&c.edge], reversed: c.reversed }).collect() };
    let mut order = 0u32;
    for &f in faces {
        let face = &body.faces[f];
        let near_loops: Vec<Vec<Coedge>> = face.loops.iter().map(|lp| if outward_thick { reversed(lp, &near_edge) } else { same(lp, &near_edge) }).collect();
        let far_loops: Vec<Vec<Coedge>> = face.loops.iter().map(|lp| if outward_thick { same(lp, &far_edge) } else { reversed(lp, &far_edge) }).collect();
        let reach = boundary_samples(body, f);
        let moved = moved_surface(&face.surface, if face.flipped { -d } else { d }, &reach).map_err(|e| refuse(e.to_string()))?;
        let i = out.add_face(face.surface.clone(), face.flipped != outward_thick, near_loops, tag(SLOT_START_CAP));
        out.faces[i].origin = origin_of(vec![face.tag]);
        let i = out.add_face(moved, face.flipped == outward_thick, far_loops, tag(SLOT_END_CAP));
        out.faces[i].origin = origin_of(vec![face.tag]);
        for lp in &face.loops {
            for c in lp {
                let Some((wall, _)) = walls.remove(&c.edge) else { continue };
                let edge = &body.edges[c.edge];
                let (start, end) = if c.reversed { (edge.v1, edge.v0) } else { (edge.v0, edge.v1) };
                let up = |v: usize, rev: bool| Coedge { edge: riser[&v], reversed: rev };
                let near = near_edge[&c.edge];
                let far = far_edge[&c.edge];
                let looped = if outward_thick {
                    vec![Coedge { edge: near, reversed: c.reversed }, up(end, false), Coedge { edge: far, reversed: !c.reversed }, up(start, true)]
                } else {
                    vec![Coedge { edge: near, reversed: !c.reversed }, up(start, false), Coedge { edge: far, reversed: c.reversed }, up(end, true)]
                };
                let surface = match wall {
                    Wall::Exact(s) => s,
                    Wall::Fitted(patch) => Surface::Free { frame: Frame::WORLD, patch },
                };
                // Which way it faces: out of the face, at a point partway up the wall.
                let m = edge.eval_mid();
                let tangent = edge.curve.derivative((edge.t0 + edge.t1) / 2.0) * if c.reversed { -1.0 } else { 1.0 };
                let n = normal_of(face, m);
                let outward = tangent.cross(n).normalized();
                let probe = m + n * (d / 2.0);
                let flipped = surface.normal(probe).dot(outward) < 0.0;
                let neighbour: Vec<FaceTag> = uses[c.edge].iter().map(|u| body.faces[u.0].tag).collect();
                let i = out.add_face(surface, flipped, vec![looped], tag(SLOT_SEG_BASE + order));
                out.faces[i].origin = origin_of(neighbour);
                order += 1;
            }
        }
    }
    out.compute_shells();
    validate(&out).map_err(|why| refuse(format!("the result isn't a valid solid: {}", why.join("; "))))?;
    let mesh = tessellate(&out, &TessParams::draft_for_size(out.aabb().diagonal()))?;
    if mesh.open_edge_count() > 0 {
        return Err(refuse("the result doesn't mesh into a closed surface".into()));
    }
    let volume = crate::mass::mass_properties(&out).volume;
    if !(volume > 0.0) {
        return Err(refuse(format!("the result has a volume of {volume}, so it has turned inside out")));
    }
    Ok(out)
}

fn normal_of(face: &Face, p: Vec3) -> Vec3 {
    face.outward_normal(p)
}
