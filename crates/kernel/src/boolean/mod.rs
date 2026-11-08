//! Boolean operations.
//!
//! The principle: every contact between the two bodies becomes one shared vertex
//! or one shared edge, created once and used by both sides. Faces are then cut
//! along those shared edges, the pieces are classified, and the kept pieces close
//! up because they literally refer to the same edges. Nothing is stitched back
//! together by tolerance at the end, and any failure to close is reported.
//!
//! The steps:
//!
//! 1. Pool the vertices of both bodies, merging coincident ones.
//! 2. Find where each edge meets the other body: its vertices and its faces.
//! 3. Cut every edge at those points.
//! 4. Intersect overlapping face pairs. Curve pieces lying in both faces become
//!    new edges. On coincident faces, each side's edges imprint onto the other.
//! 5. Trace each face's new regions.
//! 6. Classify each region against the other body and keep what the operation asks.
//! 7. Assemble shells, separate non-manifold contacts, simplify and validate.

pub mod classify;
pub mod roots;
pub mod simplify;
pub mod ssi;
pub mod trace;

use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::surface::{Chart, Surface};
use crate::topo::{Body, Coedge, Edge, Face, FaceOrigin, FaceTag};
use crate::validate::validate;
use classify::{BodyLoc, FaceLoc, Index};
use roots::{curve_surface_contact, Contact};
use ssi::{Ssi, SsiCurve};
use crate::hash::{HashMap, HashSet};
use trace::{interior_points, trace_faces, FaceChart};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Subtract,
    Intersect,
}

pub fn union(a: &Body, b: &Body) -> Result<Body> {
    boolean(a, b, BoolOp::Union)
}

pub fn subtract(a: &Body, b: &Body) -> Result<Body> {
    boolean(a, b, BoolOp::Subtract)
}

pub fn intersect(a: &Body, b: &Body) -> Result<Body> {
    boolean(a, b, BoolOp::Intersect)
}

/// Points this close are one vertex.
const MERGE: f64 = TOL * 1.5;

/// The shared vertices and edges of the arrangement.
#[derive(Default)]
struct Arrangement {
    vertices: Vec<Vec3>,
    grid: HashMap<(i64, i64, i64), Vec<usize>>,
    edges: Vec<Edge>,
    by_ends: HashMap<(usize, usize), Vec<usize>>,
}

impl Arrangement {
    fn cell(p: Vec3) -> (i64, i64, i64) {
        let s = 4.0 * MERGE;
        ((p.x / s).floor() as i64, (p.y / s).floor() as i64, (p.z / s).floor() as i64)
    }

    fn vertex(&mut self, p: Vec3) -> usize {
        let (x, y, z) = Self::cell(p);
        for i in x - 1..=x + 1 {
            for j in y - 1..=y + 1 {
                for k in z - 1..=z + 1 {
                    if let Some(list) = self.grid.get(&(i, j, k)) {
                        for &v in list {
                            if self.vertices[v].dist(p) <= MERGE {
                                return v;
                            }
                        }
                    }
                }
            }
        }
        self.vertices.push(p);
        let id = self.vertices.len() - 1;
        self.grid.entry((x, y, z)).or_default().push(id);
        id
    }

    /// The edge running from `a` at `ta` to `b` at `tb`. If this piece of geometry
    /// already exists between these vertices, returns the existing edge.
    fn edge(&mut self, curve: &Curve, ta: f64, tb: f64, a: usize, b: usize) -> Coedge {
        let key = (a.min(b), a.max(b));
        let probe = [0.25, 0.5, 0.8].map(|f| curve.eval(ta + (tb - ta) * f));
        if let Some(list) = self.by_ends.get(&key) {
            for &id in list {
                let e = &self.edges[id];
                if probe.iter().all(|&p| e.curve.closest(p, e.t0, e.t1).1 <= 8.0 * TOL) {
                    return Coedge { edge: id, reversed: e.v0 != a };
                }
            }
        }
        let reversed = ta > tb;
        let (t0, t1, v0, v1) = if reversed { (tb, ta, b, a) } else { (ta, tb, a, b) };
        self.edges.push(Edge { curve: curve.clone(), t0, t1, v0, v1 });
        let id = self.edges.len() - 1;
        self.by_ends.entry(key).or_default().push(id);
        Coedge { edge: id, reversed }
    }
}

/// A region of an input face after cutting.
struct Piece {
    source: usize, // 0 or 1
    face: usize,
    loops: Vec<Vec<Coedge>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Class {
    In,
    Out,
    OnSame,
    OnOpposite,
}

/// How much a Boolean verifies its own result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Checks {
    /// Everything: topology, a watertight mesh and point membership.
    Full,
    /// For a step inside a longer operation whose final result gets `Full`: topology
    /// and a smaller membership sample, but no mesh.
    Interim,
}

pub fn boolean(a: &Body, b: &Body, op: BoolOp) -> Result<Body> {
    boolean_with(a, b, op, Checks::Full)
}

pub fn boolean_with(a: &Body, b: &Body, op: BoolOp, checks: Checks) -> Result<Body> {
    for p in a.vertices.iter().chain(&b.vertices) {
        // Written so that a NaN fails too.
        if !(p.x.abs().max(p.y.abs()).max(p.z.abs()) <= MAX_COORD) {
            return Err(Error::InvalidInput(format!("a body reaches beyond the {MAX_COORD} mm this kernel works within")));
        }
    }
    let index = [Index::new(a), Index::new(b)];
    let bodies = [a, b];

    let mut arr = Arrangement::default();

    // Step 1: pool the vertices.
    let mut vertex_map: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    for s in 0..2 {
        vertex_map[s] = bodies[s].vertices.iter().map(|&p| arr.vertex(p)).collect();
    }

    // Step 2: where each edge meets the other body.
    let mut splits: [Vec<Vec<(f64, usize)>>; 2] = [Vec::new(), Vec::new()];
    for s in 0..2 {
        splits[s] =
            bodies[s].edges.iter().map(|e| vec![(e.t0, vertex_map[s][e.v0]), (e.t1, vertex_map[s][e.v1])]).collect();
    }
    for s in 0..2 {
        let other = 1 - s;
        for (i, e) in bodies[s].edges.iter().enumerate() {
            let bx = index[s].edge_boxes[i].inflated(2.0 * TOL);
            for g in 0..bodies[other].faces.len() {
                if !bx.overlaps(&index[other].face_boxes[g]) {
                    continue;
                }
                let surf = &bodies[other].faces[g].surface;
                let Contact::Roots(found) = curve_surface_contact(&e.curve, e.t0, e.t1, surf, TOL) else {
                    continue;
                };
                for t in found {
                    let p = e.curve.eval(t);
                    if index[other].locate_on_face(g, p, 2.0 * TOL) == FaceLoc::Outside {
                        continue;
                    }
                    // A shallow crossing can land a little way from a split that
                    // already represents this contact. Don't make a second vertex.
                    let speed = e.curve.speed_bound();
                    let merged = splits[s][i].iter().any(|&(u, _)| {
                        (u - t).abs() * speed < 1e4 * TOL
                            && surf.sdf(e.curve.eval(u)).abs() <= TOL
                            && surf.sdf(e.curve.eval((u + t) / 2.0)).abs() <= TOL
                    });
                    if !merged {
                        splits[s][i].push((t, arr.vertex(p)));
                    }
                }
            }
        }
    }

    // A new vertex on one edge may lie on other edges too, of either body.
    for v in 0..arr.vertices.len() {
        let p = arr.vertices[v];
        for s in 0..2 {
            for (i, e) in bodies[s].edges.iter().enumerate() {
                if index[s].edge_boxes[i].inflated(2.0 * MERGE).contains(p) {
                    let (t, d) = e.curve.closest(p, e.t0, e.t1);
                    if d <= MERGE {
                        splits[s][i].push((t, v));
                    }
                }
            }
        }
    }

    // Step 3: cut every edge.
    let mut sub_edges: [Vec<Vec<Coedge>>; 2] = [Vec::new(), Vec::new()];
    for s in 0..2 {
        for (i, e) in bodies[s].edges.iter().enumerate() {
            let list = &mut splits[s][i];
            list.sort_by(|x, y| x.0.total_cmp(&y.0));
            // Keep each vertex once. The two end vertices keep their ends.
            let (first, last) = (vertex_map[s][e.v0], vertex_map[s][e.v1]);
            let mut order: Vec<(f64, usize)> = vec![(e.t0, first)];
            for &(t, v) in list.iter() {
                if v != first && v != last && !order.iter().any(|o| o.1 == v) {
                    order.push((t, v));
                }
            }
            if first == last {
                return Err(Error::NearDegenerate(format!(
                    "an edge of length {:e} collapsed to a point in the other body's tolerance",
                    (e.t1 - e.t0) * e.curve.speed_bound()
                )));
            }
            order.push((e.t1, last));
            let mut pieces = Vec::with_capacity(order.len() - 1);
            for w in order.windows(2) {
                pieces.push(arr.edge(&e.curve, w[0].0, w[1].0, w[0].1, w[1].1));
            }
            sub_edges[s].push(pieces);
        }
    }

    // The cut boundary of each face, loop by loop, in loop order.
    let mut boundaries: [Vec<Vec<Vec<Coedge>>>; 2] = [Vec::new(), Vec::new()];
    for s in 0..2 {
        for face in &bodies[s].faces {
            let mut loops = Vec::with_capacity(face.loops.len());
            for lp in &face.loops {
                let mut out = Vec::new();
                for c in lp {
                    let pieces = &sub_edges[s][c.edge];
                    if c.reversed {
                        out.extend(pieces.iter().rev().map(|x| x.flipped()));
                    } else {
                        out.extend(pieces.iter().copied());
                    }
                }
                loops.push(out);
            }
            boundaries[s].push(loops);
        }
    }

    // Step 4: intersect faces.
    let mut extras: [Vec<Vec<usize>>; 2] = [vec![Vec::new(); a.faces.len()], vec![Vec::new(); b.faces.len()]];
    for fa in 0..a.faces.len() {
        for fb in 0..b.faces.len() {
            let (ba, bb) = (&index[0].face_boxes[fa], &index[1].face_boxes[fb]);
            if !ba.inflated(2.0 * TOL).overlaps(&bb.inflated(2.0 * TOL)) {
                continue;
            }
            intersect_faces(a, b, &index, fa, fb, &mut arr, &boundaries, &mut extras)?;
        }
    }

    // Step 5: cut each face into regions.
    let mut pieces: Vec<Piece> = Vec::new();
    for s in 0..2 {
        for f in 0..bodies[s].faces.len() {
            let face = &bodies[s].faces[f];
            let boundary = &boundaries[s][f];
            let on_boundary: HashSet<usize> = boundary.iter().flatten().map(|c| c.edge).collect();
            let inner: Vec<usize> = {
                let mut seen = HashSet::default();
                extras[s][f].iter().copied().filter(|e| !on_boundary.contains(e) && seen.insert(*e)).collect()
            };
            if inner.is_empty() {
                pieces.push(Piece { source: s, face: f, loops: boundary.clone() });
                continue;
            }
            let mut half: Vec<Coedge> = boundary.iter().flatten().copied().collect();
            for &e in &inner {
                half.push(Coedge { edge: e, reversed: false });
                half.push(Coedge { edge: e, reversed: true });
            }
            let fc = FaceChart { surface: &face.surface, chart: &face.chart, flipped: face.flipped };
            for loops in trace_faces(&fc, &arr.edges, &arr.vertices, &half)? {
                pieces.push(Piece { source: s, face: f, loops });
            }
        }
    }

    // Step 6: classify and select.
    let mut selected: Vec<(usize, bool)> = Vec::new(); // (piece, flip)
    for (i, piece) in pieces.iter().enumerate() {
        let class = classify_piece(piece, bodies, &index, &arr)?;
        let keep = match (op, piece.source, class) {
            (BoolOp::Union, 0, Class::Out | Class::OnSame) => Some(false),
            (BoolOp::Union, 1, Class::Out) => Some(false),
            (BoolOp::Intersect, 0, Class::In | Class::OnSame) => Some(false),
            (BoolOp::Intersect, 1, Class::In) => Some(false),
            (BoolOp::Subtract, 0, Class::Out | Class::OnOpposite) => Some(false),
            (BoolOp::Subtract, 1, Class::In) => Some(true),
            _ => None,
        };
        if let Some(flip) = keep {
            selected.push((i, flip));
        }
    }

    // Step 7: assemble.
    let body = assemble(&pieces, &selected, bodies, &arr)?;
    let body = simplify::simplify(body)?;
    validate(&body).map_err(Error::InvalidBody)?;
    match checks {
        Checks::Full => {
            verify_meshable(&body, bodies)?;
            verify_membership(&index, &body, op, 40)?;
        }
        Checks::Interim => verify_membership(&index, &body, op, 12)?,
    }
    Ok(body)
}

/// Identifies an edge by its geometry, so that an edge can be recognised across a
/// Boolean that renumbered it.
fn edge_signature(body: &Body, e: usize) -> u64 {
    let edge = &body.edges[e];
    let kind: u64 = match edge.curve {
        Curve::Line { .. } => 1,
        Curve::Circle { .. } => 2,
        Curve::Ellipse { .. } => 3,
        Curve::Sampled(_) => 4,
        Curve::Spline(_) => 5,
    };
    let (p, q) = (body.vertices[edge.v0], body.vertices[edge.v1]);
    let mut h: u64 = 0xcbf29ce484222325 ^ kind;
    for x in [p.x, p.y, p.z, q.x, q.y, q.z, edge.t1 - edge.t0] {
        h = (h ^ x.to_bits()).wrapping_mul(0x100000001b3);
        h ^= h >> 29;
    }
    h
}

/// A result can pass every topological check and still hold two edges that run
/// within tolerance of each other, for instance an intersection curve tangent to
/// a face boundary. Such a body can't be meshed or machined reliably, so it is
/// refused here rather than discovered later.
///
/// Meshing the whole body after every operation made long chains quadratic. A face with
/// no new edges was checked when it was made, so this triangulates only the faces that
/// have new ones. Only if one of those fails does it mesh the whole body for the final
/// verdict, since the mesher can often resolve a conflict with denser sampling.
fn verify_meshable(result: &Body, inputs: [&Body; 2]) -> Result<()> {
    if result.faces.is_empty() {
        return Ok(());
    }
    let mut known = HashSet::default();
    for body in inputs {
        for e in 0..body.edges.len() {
            known.insert(edge_signature(body, e));
        }
    }
    let is_new: Vec<bool> = (0..result.edges.len()).map(|e| !known.contains(&edge_signature(result, e))).collect();
    let mut bounds = Aabb::EMPTY;
    for &p in &result.vertices {
        bounds.add(p);
    }
    let params = crate::tess::TessParams::for_size(bounds.diagonal().max(1e-9));
    let changed_faces_mesh = (0..result.faces.len()).all(|f| {
        !result.faces[f].loops.iter().flatten().any(|c| is_new[c.edge])
            || crate::tess::face_meshes_cleanly(result, f, &is_new, &params)
    });
    if !changed_faces_mesh {
        verify_by_meshing(result, &params)?;
    }
    Ok(())
}

fn verify_by_meshing(body: &Body, params: &crate::tess::TessParams) -> Result<()> {
    let mesh = crate::tess::tessellate(body, params)
        .map_err(|e| Error::NearDegenerate(format!("the result has edges too close to tell apart: {e}")))?;
    let open = mesh.open_edge_count();
    if open != 0 {
        let (p, _) = mesh.open_edges()[0];
        return Err(Error::NearDegenerate(format!(
            "the result has edges too close to tell apart near ({:.4}, {:.4}, {:.4}) ({open} mesh gaps)",
            p.x, p.y, p.z
        )));
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn intersect_faces(
    a: &Body,
    b: &Body,
    index: &[Index; 2],
    fa: usize,
    fb: usize,
    arr: &mut Arrangement,
    boundaries: &[Vec<Vec<Vec<Coedge>>>; 2],
    extras: &mut [Vec<Vec<usize>>; 2],
) -> Result<()> {
    let (face_a, face_b) = (&a.faces[fa], &b.faces[fb]);
    let (ba, bb) = (&index[0].face_boxes[fa], &index[1].face_boxes[fb]);
    let mut region = Aabb {
        min: v3(ba.min.x.max(bb.min.x), ba.min.y.max(bb.min.y), ba.min.z.max(bb.min.z)),
        max: v3(ba.max.x.min(bb.max.x), ba.max.y.min(bb.max.y), ba.max.z.min(bb.max.z)),
    };
    // A touching overlap can come out empty on some axis; give it a little depth.
    for (lo, hi) in [
        (&mut region.min.x, &mut region.max.x),
        (&mut region.min.y, &mut region.max.y),
        (&mut region.min.z, &mut region.max.z),
    ] {
        if *lo > *hi {
            let m = (*lo + *hi) / 2.0;
            *lo = m;
            *hi = m;
        }
    }
    let region = region.inflated(1e-6);

    let result = ssi::intersect(&face_a.surface, &face_b.surface, &region);
    let curves: Vec<SsiCurve>;
    let crossings: Vec<Vec3>;
    match result {
        Ssi::PartlyCoincident { at } => {
            // Only a problem where both faces actually are.
            let there = |s: usize, f: usize, face: &Face| index[s].locate_on_face(f, face.surface.project(at), 4.0 * TOL) != FaceLoc::Outside;
            if there(0, fa, face_a) && there(1, fb, face_b) {
                return Err(Error::NearDegenerate(format!(
                    "a {} and a {} run together over part of their width near ({:.4}, {:.4}, {:.4}), which isn't handled yet",
                    face_a.surface.name(),
                    face_b.surface.name(),
                    at.x,
                    at.y,
                    at.z
                )));
            }
            return Ok(());
        }
        Ssi::Coincident { .. } => {
            // Coincident surfaces: each face's edges imprint onto the other.
            for (s, f, other, g) in [(0usize, fa, 1usize, fb), (1, fb, 0, fa)] {
                for &c in boundaries[s][f].iter().flatten() {
                    let e = &arr.edges[c.edge];
                    let mid = e.curve.eval((e.t0 + e.t1) / 2.0);
                    if index[other].locate_on_face(g, mid, 2.0 * TOL) == FaceLoc::Inside {
                        extras[other][g].push(c.edge);
                    }
                }
            }
            return Ok(());
        }
        Ssi::Curves { curves: c, crossings: x } => {
            curves = c;
            crossings = x;
        }
        Ssi::General => {
            // Trace over whichever face has the smaller chart window.
            let (la, ha) = a.face_chart_bounds(fa);
            let (lb, hb) = b.face_chart_bounds(fb);
            let area_a = (ha.x - la.x) * (ha.y - la.y);
            let area_b = (hb.x - lb.x) * (hb.y - lb.y);
            // A point matters if it is on or near both faces. The tolerance here is
            // generous because a suspect point need not lie exactly on the surfaces.
            let relevant = |p: Vec3| {
                let reach = (region.diagonal() * 1e-3).clamp(1e-4, 1e-2);
                let near = reach / 2.0;
                let qa = face_a.surface.project(p);
                let qb = face_b.surface.project(p);
                let (la, lb) = (index[0].locate_on_face(fa, qa, near), index[1].locate_on_face(fb, qb, near));
                if la == FaceLoc::Outside || lb == FaceLoc::Outside {
                    return false;
                }
                if la == FaceLoc::Inside || lb == FaceLoc::Inside {
                    return true;
                }
                // On the boundary of both. If the faces lie on opposite sides there,
                // they only touch at this point and no curve runs between them. Look
                // around the point for anywhere that is inside both.
                let n = face_a.surface.normal(qa);
                let e1 = n.any_perpendicular();
                let e2 = n.cross(e1);
                (0..24).any(|k| {
                    let (s, c) = (k as f64 * TAU / 24.0).sin_cos();
                    let q = qa + (e1 * c + e2 * s) * reach;
                    index[0].locate_on_face(fa, face_a.surface.project(q), 2.0 * TOL) == FaceLoc::Inside
                        && index[1].locate_on_face(fb, face_b.surface.project(q), 2.0 * TOL) == FaceLoc::Inside
                })
            };
            curves = if area_a <= area_b {
                ssi::contour(&face_a.surface, &face_a.chart, la, ha, &face_b.surface, &region, &relevant)?
            } else {
                ssi::contour(&face_b.surface, &face_b.chart, lb, hb, &face_a.surface, &region, &relevant)?
            };
            crossings = Vec::new();
        }
    }
    if curves.is_empty() {
        return Ok(());
    }

    // Vertices on either face's boundary that lie on both surfaces.
    let mut candidates: Vec<usize> = Vec::new();
    for &(s, f) in &[(0usize, fa), (1usize, fb)] {
        for c in boundaries[s][f].iter().flatten() {
            for v in [arr.edges[c.edge].v0, arr.edges[c.edge].v1] {
                let p = arr.vertices[v];
                if face_a.surface.sdf(p).abs() <= 2.0 * MERGE
                    && face_b.surface.sdf(p).abs() <= 2.0 * MERGE
                    && !candidates.contains(&v)
                {
                    candidates.push(v);
                }
            }
        }
    }
    for x in crossings {
        if index[0].locate_on_face(fa, x, 2.0 * TOL) != FaceLoc::Outside
            && index[1].locate_on_face(fb, x, 2.0 * TOL) != FaceLoc::Outside
        {
            let v = arr.vertex(x);
            if !candidates.contains(&v) {
                candidates.push(v);
            }
        }
    }

    for sc in &curves {
        let mut on_curve: Vec<(f64, usize)> = Vec::new();
        for &v in &candidates {
            let (t, d) = sc.curve.closest(arr.vertices[v], sc.t0, sc.t1);
            if d <= 3.0 * MERGE {
                on_curve.push((t, v));
            }
        }
        on_curve.sort_by(|x, y| x.0.total_cmp(&y.0));
        on_curve.dedup_by_key(|x| x.1);

        let pieces = curve_pieces(sc, &on_curve, arr, face_a, face_b)?;
        for (curve, ta, tb, va, vb) in pieces {
            if va == vb {
                continue;
            }
            // Keep the piece only where it lies in both faces.
            let inside = |f: f64| {
                let p = curve.eval(ta + (tb - ta) * f);
                (index[0].locate_on_face(fa, p, 2.0 * TOL), index[1].locate_on_face(fb, p, 2.0 * TOL))
            };
            let (la, lb) = inside(0.5);
            if la == FaceLoc::Outside || lb == FaceLoc::Outside {
                continue;
            }
            let c = arr.edge(&curve, ta, tb, va, vb);
            extras[0][fa].push(c.edge);
            extras[1][fb].push(c.edge);
        }
    }
    Ok(())
}

type RawPiece = (Curve, f64, f64, usize, usize);

/// Splits an intersection curve at the vertices on it.
fn curve_pieces(
    sc: &SsiCurve,
    on_curve: &[(f64, usize)],
    arr: &mut Arrangement,
    fa: &Face,
    fb: &Face,
) -> Result<Vec<RawPiece>> {
    if let Curve::Sampled(s) = &sc.curve {
        return sampled_pieces(s, sc.closed, on_curve, arr, fa, fb);
    }
    let mut out = Vec::new();
    if sc.closed {
        let mut stops: Vec<(f64, usize)> = on_curve.to_vec();
        match stops.len() {
            0 => {
                // A loop that crosses no boundary. It needs vertices of its own,
                // but only if it actually lies in both faces, which the caller
                // tests per piece. Two pieces, because no edge is a closed loop.
                let (p, q) = (sc.curve.eval(0.0), sc.curve.eval(PI));
                let (v, w) = (arr.vertex(p), arr.vertex(q));
                stops.push((0.0, v));
                stops.push((PI, w));
            }
            1 => {
                let t = stops[0].0;
                let q = sc.curve.eval(t + PI);
                let w = arr.vertex(q);
                stops.push((t + PI, w));
            }
            _ => {}
        }
        for i in 0..stops.len() {
            let (ta, va) = stops[i];
            let (tb, vb) = if i + 1 < stops.len() { stops[i + 1] } else { (stops[0].0 + TAU, stops[0].1) };
            out.push((sc.curve.clone(), ta, tb, va, vb));
        }
    } else {
        for w in on_curve.windows(2) {
            out.push((sc.curve.clone(), w[0].0, w[1].0, w[0].1, w[1].1));
        }
    }
    Ok(out)
}

/// Cuts a sampled curve at vertices. Each piece becomes its own sampled curve that
/// starts and ends exactly at its vertices.
fn sampled_pieces(
    s: &crate::curve::SampledCurve,
    closed: bool,
    on_curve: &[(f64, usize)],
    arr: &mut Arrangement,
    fa: &Face,
    fb: &Face,
) -> Result<Vec<RawPiece>> {
    let n = s.points.len();
    let mut stops: Vec<(f64, usize)> = on_curve.to_vec();
    if closed {
        if stops.is_empty() {
            let (v, w) = (arr.vertex(s.points[0]), arr.vertex(s.points[n / 2]));
            stops.push((0.0, v));
            stops.push(((n / 2) as f64, w));
        } else if stops.len() == 1 {
            let t = (stops[0].0 + (n as f64 - 1.0) / 2.0).rem_euclid(n as f64 - 1.0);
            let w = arr.vertex(s.eval(t));
            stops.push((t, w));
            stops.sort_by(|x, y| x.0.total_cmp(&y.0));
        }
    }
    let build = |ta: f64, tb: f64, va: usize, vb: usize, arr: &Arrangement| -> Result<Curve> {
        let mut pts = vec![arr.vertices[va]];
        // Walk samples from ta to tb, wrapping past the end on a closed curve.
        let period = (n - 1) as f64;
        let end = if tb >= ta { tb } else { tb + period };
        let mut k = ta.floor() + 1.0;
        while k < end - 1e-6 {
            if k > ta + 1e-6 {
                let idx = (k as usize) % (n - 1).max(1);
                pts.push(s.points[if closed { idx } else { k as usize }]);
            }
            k += 1.0;
        }
        pts.push(arr.vertices[vb]);
        // Drop near-duplicate neighbours from snapping onto vertices.
        let mut clean: Vec<Vec3> = Vec::with_capacity(pts.len());
        for (i, p) in pts.iter().enumerate() {
            let last = i + 1 == pts.len();
            if clean.last().is_some_and(|q: &Vec3| q.dist(*p) < 4.0 * TOL) {
                if last {
                    *clean.last_mut().unwrap() = *p;
                }
                continue;
            }
            clean.push(*p);
        }
        if clean.len() == 2 {
            let m = crate::surface::refine_on_two(&fa.surface, &fb.surface, clean[0].lerp(clean[1], 0.5))
                .unwrap_or_else(|| clean[0].lerp(clean[1], 0.5));
            clean.insert(1, m);
        }
        ssi::sampled_from_points(clean, &s.surfaces.0, &s.surfaces.1)
    };
    let mut out = Vec::new();
    let count = stops.len();
    let pairs = if closed { count } else { count.saturating_sub(1) };
    for i in 0..pairs {
        let (ta, va) = stops[i];
        let (tb, vb) = stops[(i + 1) % count];
        if va == vb {
            continue;
        }
        let curve = build(ta, tb, va, vb, arr)?;
        let last = match &curve {
            Curve::Sampled(c) => (c.points.len() - 1) as f64,
            _ => 1.0,
        };
        out.push((curve, 0.0, last, va, vb));
    }
    Ok(out)
}

fn classify_piece(piece: &Piece, bodies: [&Body; 2], index: &[Index; 2], arr: &Arrangement) -> Result<Class> {
    let face = &bodies[piece.source].faces[piece.face];
    let other = 1 - piece.source;
    // A face entirely outside the other body's bounding box needs no testing. In a
    // chain of local operations this is most faces.
    if !index[piece.source].face_boxes[piece.face].inflated(4.0 * TOL).overlaps(&index[other].bounds) {
        return Ok(Class::Out);
    }
    let size = index[0].bounds.diagonal().max(index[1].bounds.diagonal());
    // A few well-placed points settle most regions. If they don't all agree, or there are too few, the
    // thorough search decides, so choosing quickly can't cause a refusal the thorough way wouldn't.
    let quick = interior_points(&face.surface, &face.chart, &arr.edges, &piece.loops, 4, true);
    if quick.len() >= 4 {
        if let Some(class) = unanimous(&quick, face, bodies, index, other, size) {
            return Ok(class);
        }
    }
    let mut answer: Option<Class> = None;
    let mut clean_samples = 0;
    let mut touching = None;
    // Every clean sample must agree. A region that is partly inside and partly
    // outside means an intersection curve was missed, and the region should have
    // been cut, so that is an error however plausible the rest looks.
    let samples = interior_points(&face.surface, &face.chart, &arr.edges, &piece.loops, 8, false);
    if samples.is_empty() {
        return Err(Error::Internal("a face region has no interior point".into()));
    }
    for uv in samples {
        let p = face.surface.from_chart(&face.chart, uv);
        let class = match index[other].locate_in_body(p) {
            BodyLoc::Inside => Class::In,
            BodyLoc::Outside => Class::Out,
            BodyLoc::OnFace(g) => {
                let of = &bodies[other].faces[g];
                if ssi::coincident(&face.surface, &of.surface, size).is_none() {
                    // A point of contact, not the region's real relation.
                    touching = Some((p, g));
                    continue;
                }
                if face.outward_normal(p).dot(of.outward_normal(p)) > 0.0 {
                    Class::OnSame
                } else {
                    Class::OnOpposite
                }
            }
        };
        match answer {
            None => answer = Some(class),
            Some(a) if a != class => {
                return Err(Error::NearDegenerate(format!(
                    "a region of the {} crosses the other body but was not cut there, near ({:.4}, {:.4}, {:.4}). The surfaces are probably tangent",
                    face.surface.name(),
                    p.x,
                    p.y,
                    p.z
                )));
            }
            Some(_) => {}
        }
        clean_samples += 1;
        if clean_samples == 4 {
            break;
        }
    }
    if let Some(a) = answer {
        return Ok(a);
    }
    let (p, g) = touching.unwrap_or((Vec3::ZERO, 0));
    Err(Error::NearDegenerate(format!(
        "a region of {} {} ({}) touches {} of the other body at ({:.6}, {:.6}, {:.6}) wherever it was sampled",
        if piece.source == 0 { "A" } else { "B" },
        piece.face,
        face.surface.name(),
        g,
        p.x,
        p.y,
        p.z,
    )))
}

/// Checks the result against its definition at sampled points: a point is in A+B
/// exactly when it is in A or in B, and so on. This doesn't depend on anything the
/// Boolean computed, so it catches errors the topology checks can't see. Half the
/// points go where the two bodies overlap, since that is where mistakes happen.
/// The class all of these chart points give, if they all give one cleanly.
fn unanimous(samples: &[Vec2], face: &Face, bodies: [&Body; 2], index: &[Index; 2], other: usize, size: f64) -> Option<Class> {
    let mut answer = None;
    for &uv in samples {
        let p = face.surface.from_chart(&face.chart, uv);
        let class = match index[other].locate_in_body(p) {
            BodyLoc::Inside => Class::In,
            BodyLoc::Outside => Class::Out,
            BodyLoc::OnFace(g) => {
                let of = &bodies[other].faces[g];
                ssi::coincident(&face.surface, &of.surface, size)?;
                if face.outward_normal(p).dot(of.outward_normal(p)) > 0.0 {
                    Class::OnSame
                } else {
                    Class::OnOpposite
                }
            }
        };
        match answer {
            None => answer = Some(class),
            Some(a) if a != class => return None,
            Some(_) => {}
        }
    }
    answer
}

fn verify_membership(index: &[Index; 2], result: &Body, op: BoolOp, points: usize) -> Result<()> {
    let out = Index::new(result);
    let mut all = index[0].bounds;
    all.union(&index[1].bounds);
    let (a, b) = (&index[0].bounds, &index[1].bounds);
    let overlap = Aabb {
        min: v3(a.min.x.max(b.min.x), a.min.y.max(b.min.y), a.min.z.max(b.min.z)),
        max: v3(a.max.x.min(b.max.x), a.max.y.min(b.max.y), a.max.z.min(b.max.z)),
    };
    let usable = overlap.min.x < overlap.max.x && overlap.min.y < overlap.max.y && overlap.min.z < overlap.max.z;
    let mut rng = Rng::new(0xC0FFEE);
    for i in 0..points {
        let bx = if usable && i % 2 == 0 { &overlap } else { &all };
        let p = v3(rng.range(bx.min.x, bx.max.x), rng.range(bx.min.y, bx.max.y), rng.range(bx.min.z, bx.max.z));
        let inside = |loc: BodyLoc| match loc {
            BodyLoc::Inside => Some(true),
            BodyLoc::Outside => Some(false),
            BodyLoc::OnFace(_) => None,
        };
        let (Some(ina), Some(inb), Some(got)) =
            (inside(index[0].locate_in_body(p)), inside(index[1].locate_in_body(p)), inside(out.locate_in_body(p)))
        else {
            continue;
        };
        let want = match op {
            BoolOp::Union => ina || inb,
            BoolOp::Subtract => ina && !inb,
            BoolOp::Intersect => ina && inb,
        };
        if got != want {
            return Err(Error::NearDegenerate(format!(
                "the result is wrong at ({:.4}, {:.4}, {:.4}): an intersection was probably missed at a tangency",
                p.x, p.y, p.z
            )));
        }
    }
    Ok(())
}

/// Builds the result body from the selected regions.
fn assemble(pieces: &[Piece], selected: &[(usize, bool)], bodies: [&Body; 2], arr: &Arrangement) -> Result<Body> {
    // Final loops per selected region.
    struct Out {
        surface: Surface,
        flipped: bool,
        chart: Chart,
        tag: FaceTag,
        origin: FaceOrigin,
        loops: Vec<Vec<Coedge>>,
    }
    let mut faces: Vec<Out> = Vec::with_capacity(selected.len());
    for &(i, flip) in selected {
        let piece = &pieces[i];
        let f = &bodies[piece.source].faces[piece.face];
        let loops = if flip {
            piece.loops.iter().map(|l| l.iter().rev().map(|c| c.flipped()).collect()).collect()
        } else {
            piece.loops.clone()
        };
        faces.push(Out { surface: f.surface.clone(), flipped: f.flipped != flip, chart: f.chart, tag: f.tag, origin: f.origin.clone(), loops });
    }

    // Each use of an arrangement edge: (face, loop, position, reversed).
    let mut uses: HashMap<usize, Vec<(usize, usize, usize, bool)>> = HashMap::default();
    for (fi, f) in faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            for (k, c) in l.iter().enumerate() {
                uses.entry(c.edge).or_default().push((fi, li, k, c.reversed));
            }
        }
    }

    let mut body = Body::default();
    // Build edges. Normally one per arrangement edge. Where several faces meet at
    // an edge, pair them around it so each wedge of material gets its own copy.
    let mut assign: HashMap<(usize, usize, usize), usize> = HashMap::default();
    let mut edge_ids: Vec<usize> = uses.keys().copied().collect();
    edge_ids.sort_unstable();
    for ae in edge_ids {
        let list = &uses[&ae];
        let e = &arr.edges[ae];
        let pairs: Vec<Vec<(usize, usize, usize, bool)>> = if list.len() == 2 {
            vec![list.clone()]
        } else if list.len().is_multiple_of(2) && list.len() >= 4 {
            let mid = e.curve.eval((e.t0 + e.t1) / 2.0);
            let tangent = e.curve.derivative((e.t0 + e.t1) / 2.0).normalized();
            let u = tangent.any_perpendicular();
            let v = tangent.cross(u);
            let mut around: Vec<(f64, bool, (usize, usize, usize, bool))> = Vec::with_capacity(list.len());
            for &entry in list {
                let f = &faces[entry.0];
                let n = if f.flipped { -f.surface.normal(mid) } else { f.surface.normal(mid) };
                let dir = if entry.3 { -tangent } else { tangent };
                // The face lies to the left of its loop direction, seen from outside.
                let into = n.cross(dir);
                // Sort by where the face actually goes a short way from the edge, so
                // that faces tangent along the edge still sort by their curvature.
                let reach = (e.t1 - e.t0) * e.curve.speed_bound() * 1e-3;
                let away = f.surface.project(mid + into * reach) - mid;
                let angle = away.dot(v).atan2(away.dot(u));
                // Does material lie on the increasing-angle side of this face?
                let material_ahead = tangent.cross(into).dot(-n) > 0.0;
                around.push((angle, material_ahead, entry));
            }
            around.sort_by(|x, y| x.0.total_cmp(&y.0));
            let n = around.len();
            let start = around
                .iter()
                .position(|x| x.1)
                .ok_or_else(|| Error::Internal("non-manifold edge with no material between its faces".into()))?;
            let mut groups = Vec::new();
            for k in 0..n / 2 {
                let x = around[(start + 2 * k) % n];
                let y = around[(start + 2 * k + 1) % n];
                if !x.1 || y.1 {
                    return Err(Error::Internal("faces around a non-manifold edge do not alternate".into()));
                }
                groups.push(vec![x.2, y.2]);
            }
            groups
        } else {
            return Err(Error::Internal(format!("the result does not close: an edge is used by {} faces", list.len())));
        };
        for group in pairs {
            // Vertices are placeholders here; they are resolved below.
            body.edges.push(e.clone());
            let id = body.edges.len() - 1;
            for entry in group {
                assign.insert((entry.0, entry.1, entry.2), id);
            }
        }
    }

    // Separate pinches. Each edge end belongs to a fan of faces at its vertex, and
    // consecutive coedges of a loop share a fan. A vertex with several fans becomes
    // several vertices, one per fan.
    let mut parent: Vec<usize> = (0..body.edges.len() * 2).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while p[r] != r {
            p[r] = p[p[r]];
            r = p[r];
        }
        r
    }
    let end_node = |edge: usize, c_rev: bool, at_end: bool| -> usize {
        // Node 2e is the edge's v0 end, node 2e + 1 its v1 end.
        let is_v1 = c_rev != at_end;
        edge * 2 + usize::from(is_v1)
    };
    for (fi, f) in faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            for k in 0..l.len() {
                let k2 = (k + 1) % l.len();
                let (e1, e2) = (assign[&(fi, li, k)], assign[&(fi, li, k2)]);
                let n1 = end_node(e1, l[k].reversed, true);
                let n2 = end_node(e2, l[k2].reversed, false);
                let (r1, r2) = (find(&mut parent, n1), find(&mut parent, n2));
                if r1 != r2 {
                    parent[r1] = r2;
                }
            }
        }
    }
    let mut vertex_of_fan: HashMap<usize, usize> = HashMap::default();
    for e in 0..body.edges.len() {
        for (end, node) in [(0, e * 2), (1, e * 2 + 1)] {
            let r = find(&mut parent, node);
            let av = if end == 0 { body.edges[e].v0 } else { body.edges[e].v1 };
            let id = *vertex_of_fan.entry(r).or_insert_with(|| {
                body.vertices.push(arr.vertices[av]);
                body.vertices.len() - 1
            });
            if end == 0 {
                body.edges[e].v0 = id;
            } else {
                body.edges[e].v1 = id;
            }
        }
    }

    for (fi, f) in faces.into_iter().enumerate() {
        let loops = f
            .loops
            .iter()
            .enumerate()
            .map(|(li, l)| {
                l.iter().enumerate().map(|(k, c)| Coedge { edge: assign[&(fi, li, k)], reversed: c.reversed }).collect()
            })
            .collect();
        body.faces.push(Face { surface: f.surface, flipped: f.flipped, chart: f.chart, loops, tag: f.tag, origin: f.origin });
    }
    body.compute_shells();
    Ok(body)
}
