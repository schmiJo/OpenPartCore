//! Fillets and chamfers on any edge, including those whose true surface has no exact form: traced intersection
//! curves, ellipses, and spline or free-form faces.
//!
//! The other blends in `blend.rs` are exact, and they cut with a tool in a Boolean. Here that can't work: a fillet is
//! tangent to its neighbours all along them, and the Boolean rightly refuses to trace a tangency. So this works on the
//! faces directly, with the geometry coming from the true definition of the blend:
//!
//! * A ball of the fillet's radius touching both faces has its center where both offset surfaces meet, found by Newton's
//!   method at stations along the edge. The contact points are the center's closest points, and the arc between them
//!   around the center is exactly the section of the true surface, as a rational quadratic.
//! * Those sections go through a cubic in homogeneous form, the same as a loft's, which gives a patch. Between stations
//!   it's a fit, so stations are doubled until its two long sides are within `FIT` of the faces they lie on, as measured.
//! * The neighbours are trimmed back to those sides. A seam the sides cross gets shortened. An open end gets cut by the
//!   third face there.
//!
//! So unlike nearly everything else in the kernel, the result is a stated approximation and not exact.
//!
//! Where this can't work it refuses by name: a vertex that needs a corner blend, or an end that would take reshaping
//! the neighbouring faces.

use crate::blend::Kind;
use crate::boolean::classify::{BodyLoc, FaceLoc, Index};
use crate::boolean::ssi::sampled_from_points;
use crate::build::SLOT_SEG_BASE;
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::patch::{interpolation_knots, Collocation, Patch};
use crate::spline::{Hom, Spline};
use crate::surface::{refine_on_two, Surface};
use crate::tess::{tessellate, TessParams};
use crate::topo::{Body, Coedge, Edge, Face, FaceOrigin, FaceTag};
use crate::validate::validate;
use std::sync::Arc;

/// How far the patch's long sides may be from the faces they lie on. The validator allows 5e-7.
pub const FIT: f64 = 1.5e-7;
const MAX_STATIONS: usize = 1024;

fn sign(face: &Face) -> f64 {
    if face.flipped {
        -1.0
    } else {
        1.0
    }
}

fn out_sdf(face: &Face, p: Vec3) -> f64 {
    face.surface.sdf(p) * sign(face)
}

fn out_normal(face: &Face, p: Vec3) -> Vec3 {
    face.surface.normal(p) * sign(face)
}

/// Solves `x . a = p`, `x . b = q`, `x . c = r`.
fn solve3(a: Vec3, b: Vec3, c: Vec3, p: f64, q: f64, r: f64) -> Option<Vec3> {
    let (bc, ca, ab) = (b.cross(c), c.cross(a), a.cross(b));
    let det = a.dot(bc);
    if det.abs() < 1e-12 {
        return None;
    }
    Some((bc * p + ca * q + ab * r) / det)
}

/// One edge of a chain, walked in the chain's direction.
#[derive(Clone, Copy)]
struct Element {
    edge: usize,
    forward: bool,
    /// The face on the left of travel seen from outside, and the one on the right.
    left: usize,
    right: usize,
    start: usize,
    end: usize,
}

struct Chain {
    elements: Vec<Element>,
    closed: bool,
}

impl Element {
    fn point(&self, body: &Body, s: f64) -> Vec3 {
        let e = &body.edges[self.edge];
        e.curve.eval(if self.forward { e.t0 + (e.t1 - e.t0) * s } else { e.t1 - (e.t1 - e.t0) * s })
    }

    fn tangent(&self, body: &Body, s: f64) -> Vec3 {
        let e = &body.edges[self.edge];
        let t = if self.forward { e.t0 + (e.t1 - e.t0) * s } else { e.t1 - (e.t1 - e.t0) * s };
        (e.curve.derivative(t) * if self.forward { 1.0 } else { -1.0 }).normalized()
    }
}

/// The edge's direction leaving a vertex.
fn leaving(body: &Body, e: usize, v: usize) -> Vec3 {
    let edge = &body.edges[e];
    if edge.v0 == v {
        edge.curve.derivative(edge.t0).normalized()
    } else {
        -edge.curve.derivative(edge.t1).normalized()
    }
}

fn is_sharp(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], e: usize) -> bool {
    let [a, b] = uses[e].as_slice() else {
        return false;
    };
    let p = body.edge_midpoint(e);
    out_normal(&body.faces[a.0], p).cross(out_normal(&body.faces[b.0], p)).norm() > 1e-4
}

/// Whether an edge is sharp at one of its ends: a rail between two sides can be sharp along its length and smooth at
/// the end that matters here.
fn is_sharp_at(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], e: usize, v: usize) -> bool {
    let [a, b] = uses[e].as_slice() else {
        return false;
    };
    let p = body.vertices[v];
    out_normal(&body.faces[a.0], p).cross(out_normal(&body.faces[b.0], p)).norm() > 1e-3
}

/// Sorts the selected edges into chains, adding what continues them smoothly, so picking part of a rim means the rim.
fn chains_of(body: &Body, selected: &[usize], uses: &[Vec<(usize, usize, usize, bool)>], at_vertex: &[Vec<usize>]) -> Result<Vec<Chain>> {
    let continuing = |e: usize, v: usize| -> Option<usize> {
        let out = -leaving(body, e, v);
        at_vertex[v].iter().copied().find(|&g| g != e && is_sharp(body, uses, g) && leaving(body, g, v).dot(out) > 1.0 - 1e-4)
    };
    let mut done = vec![false; body.edges.len()];
    let mut chains = Vec::new();
    for &first in selected {
        if first >= body.edges.len() {
            return Err(Error::InvalidInput(format!("there is no edge {first}")));
        }
        if done[first] {
            continue;
        }
        if uses[first].len() != 2 {
            return Err(Error::InvalidBody(vec![format!("edge {first} is not used by two faces")]));
        }
        if !is_sharp(body, uses, first) {
            return Err(Error::InvalidInput("this edge is smooth, with its two faces tangent along it, so there's nothing to blend".into()));
        }
        // Back to where the chain starts, or all the way round to here.
        let (mut e, mut v) = (first, body.edges[first].v0);
        let mut closed = false;
        let mut steps = 0;
        while let Some(g) = continuing(e, v) {
            if g == first {
                closed = true;
                break;
            }
            v = if body.edges[g].v0 == v { body.edges[g].v1 } else { body.edges[g].v0 };
            e = g;
            steps += 1;
            if steps > body.edges.len() {
                return Err(Error::Internal("a chain of edges doesn't end".into()));
            }
        }
        // Walk it forward from there.
        let (start_edge, start_vertex) = if closed { (first, body.edges[first].v0) } else { (e, v) };
        let mut elements = Vec::new();
        let (mut e, mut v) = (start_edge, start_vertex);
        loop {
            let forward = body.edges[e].v0 == v;
            let end = if forward { body.edges[e].v1 } else { body.edges[e].v0 };
            let [p, q] = uses[e].as_slice() else {
                return Err(Error::InvalidBody(vec![format!("edge {e} is not used by two faces")]));
            };
            // A loop that runs along the chain has its face on the left.
            let (left, right) = if p.3 != forward { (p.0, q.0) } else { (q.0, p.0) };
            elements.push(Element { edge: e, forward, left, right, start: v, end });
            done[e] = true;
            match continuing(e, end) {
                Some(g) if g == start_edge => break,
                Some(g) => {
                    e = g;
                    v = end;
                }
                None => break,
            }
            if elements.len() > body.edges.len() {
                return Err(Error::Internal("a chain of edges doesn't end".into()));
            }
        }
        chains.push(Chain { elements, closed });
    }
    // Two chains can't meet: that would take a corner blend.
    for (i, a) in chains.iter().enumerate() {
        for b in &chains[i + 1..] {
            let ends = |c: &Chain| -> Vec<usize> { if c.closed { Vec::new() } else { vec![c.elements[0].start, c.elements[c.elements.len() - 1].end] } };
            if ends(a).iter().any(|v| ends(b).contains(v)) {
                return Err(Error::Unsupported("two blended edges meeting at an angle need a corner blend, which isn't available on these edges yet".into()));
            }
        }
    }
    Ok(chains)
}

/// A cross section of the blend at one station.
#[derive(Clone)]
struct Station {
    section: Vec<Hom>,
    /// The middle of the section, which parameters along the blend are measured through.
    mid: Vec3,
    /// A ball's center, which the next station starts from.
    center: Vec3,
}

struct Solver<'a> {
    /// The faces along each side of the whole chain, since a ball by a joint can be touching the next one along.
    lefts: Vec<usize>,
    rights: Vec<usize>,
    kind: Kind,
    /// Which way a ball sits: -1 inside the body on a convex edge, +1 outside on a concave one.
    side: f64,
    body: &'a Body,
    index: &'a Index<'a>,
}

impl<'a> Solver<'a> {
    /// The one of these faces that really has the nearest point to `p`. A seam needn't be square to the edge, so a contact can
    /// still be on the face before while the edge has gone on to the next, and all that says which is where the foot lands.
    fn holding_of(&self, list: &[usize], p: Vec3) -> usize {
        if let [only] = list {
            return *only;
        }
        let faces = &self.body.faces;
        let within = |f: &&usize| faces[**f].surface.foot_within(p);
        // The nearest that holds it: the foot on a wall across the way can be on that wall too.
        let on_it = list
            .iter()
            .filter(within)
            .filter(|f| self.index.locate_on_face(**f, faces[**f].surface.project(p), 1e-6) != FaceLoc::Outside)
            .min_by(|a, b| faces[**a].surface.sdf(p).abs().total_cmp(&faces[**b].surface.sdf(p).abs()));
        on_it.or_else(|| list.iter().find(within)).copied().unwrap_or(list[0])
    }

    fn holding(&self, list: &[usize], p: Vec3) -> &'a Face {
        &self.body.faces[self.holding_of(list, p)]
    }
    /// The section in the plane through `p` with normal `n`. `near` is a center to start from, if there is one.
    fn station(&self, p: Vec3, n: Vec3, near: Option<Vec3>) -> Option<Station> {
        self.station_as(self.kind, p, n, near)
    }

    /// The same, for a radius other than the usual one.
    fn station_as(&self, kind: Kind, p: Vec3, n: Vec3, near: Option<Vec3>) -> Option<Station> {
        match kind {
            Kind::Fillet(r) => self.fillet_station(p, n, r, near),
            Kind::Chamfer(d) => self.chamfer_station(p, n, d),
        }
    }

    fn fillet_station(&self, p: Vec3, n: Vec3, r: f64, near: Option<Vec3>) -> Option<Station> {
        let target = self.side * r;
        let mut c = match near {
            Some(c) => c + n * n.dot(p - c),
            None => {
                let (na, nb) = (out_normal(self.holding(&self.lefts, p), p), out_normal(self.holding(&self.rights, p), p));
                p + solve3(na, nb, n, target, target, 0.0)?
            }
        };
        let mut converged = false;
        for _ in 0..24 {
            let (left, right) = (self.holding(&self.lefts, c), self.holding(&self.rights, c));
            let (fa, fb, fc) = (out_sdf(left, c) - target, out_sdf(right, c) - target, n.dot(c - p));
            if fa.abs().max(fb.abs()).max(fc.abs()) < 1e-11 {
                converged = true;
                break;
            }
            c -= solve3(out_normal(left, c), out_normal(right, c), n, fa, fb, fc)?;
            if !c.is_finite() || c.dist(p) > 8.0 * r + 8.0 * p.dist(near.unwrap_or(p)) {
                return None;
            }
        }
        if !converged {
            return None;
        }
        let (pa, pb) = (c - out_normal(self.holding(&self.lefts, c), c) * target, c - out_normal(self.holding(&self.rights, c), c) * target);
        let (a, b) = ((pa - c).normalized(), (pb - c).normalized());
        let half = 0.5 * a.dot(b).clamp(-1.0, 1.0).acos();
        // Close to a half turn, the faces nearly face each other and there's no sensible arc.
        if half > 87f64.to_radians() || half < 1e-3 {
            return None;
        }
        let w = half.cos();
        let across = ((pa + pb) * 0.5 - c).normalized();
        let peak = c + across * (r / w);
        Some(Station { section: vec![Hom::of(pa, 1.0), Hom::of(peak, w), Hom::of(pb, 1.0)], mid: c + across * r, center: c })
    }

    fn chamfer_station(&self, p: Vec3, n: Vec3, d: f64) -> Option<Station> {
        // At this distance from the edge in this plane, going into each face.
        let reach = |list: &[usize], into: Vec3| -> Option<Vec3> {
            let mut q = p + into * d;
            for _ in 0..24 {
                let face = self.holding(list, q);
                let (f0, f1, f2) = (out_sdf(face, q), n.dot(q - p), ((q - p).norm2() - d * d) * 0.5);
                if f0.abs().max(f1.abs()).max(f2.abs()) < 1e-11 {
                    return Some(q);
                }
                q -= solve3(out_normal(face, q), n, q - p, f0, f1, f2)?;
            }
            None
        };
        let (na, nb) = (out_normal(self.holding(&self.lefts, p), p), out_normal(self.holding(&self.rights, p), p));
        // Left of travel seen from outside, and right of it.
        let pa = reach(&self.lefts, na.cross(n).normalized())?;
        let pb = reach(&self.rights, n.cross(nb).normalized())?;
        let mid = (pa + pb) * 0.5;
        Some(Station { section: vec![Hom::of(pa, 1.0), Hom::of(pb, 1.0)], mid, center: mid })
    }
}

/// Where along an element a blend's side crosses a knot line of a free-form face, since a surface can have a real kink
/// there, which a fit should stop at instead of trying to bend through. As fractions along the element.
fn knot_crossings(body: &Body, e: &Element, solver: &Solver, kind_at: &dyn Fn(f64) -> Kind) -> Vec<f64> {
    const SAMPLES: usize = 48;
    let mut out = Vec::new();
    let contact = |s: f64| -> Option<(Vec3, Vec3)> {
        let st = solver.station_as(kind_at(s), e.point(body, s), e.tangent(body, s), None)?;
        Some((st.section[0].point(), st.section[st.section.len() - 1].point()))
    };
    // Where a side goes from one face to the next away from the joint, as across a slanted seam, the surface it's fitted to
    // changes, and so does how it bends.
    for (which, list) in [(0usize, &solver.lefts), (1, &solver.rights)] {
        if list.len() < 2 {
            continue;
        }
        let on = |s: f64| -> Option<usize> {
            let (a, b) = contact(s)?;
            Some(solver.holding_of(list, if which == 0 { a } else { b }))
        };
        let mut before = on(0.0);
        for k in 1..=SAMPLES {
            let (lo, hi) = ((k - 1) as f64 / SAMPLES as f64, k as f64 / SAMPLES as f64);
            let here = on(hi);
            if let (Some(a), Some(b)) = (before, here) {
                if a != b {
                    let (mut x, mut y) = (lo, hi);
                    for _ in 0..30 {
                        let m = (x + y) / 2.0;
                        if on(m) == Some(a) {
                            x = m;
                        } else {
                            y = m;
                        }
                    }
                    let at = (x + y) / 2.0;
                    if at > 1e-6 && at < 1.0 - 1e-6 {
                        out.push(at);
                    }
                }
            }
            before = here;
        }
    }
    let (left, right) = (&body.faces[e.left], &body.faces[e.right]);
    for (which, face) in [(0usize, left), (1, right)] {
        let Surface::Free { frame, patch } = &face.surface else { continue };
        let knots = patch.interior_knots();
        let at = |s: f64| -> Option<[f64; 2]> {
            let (a, b) = contact(s)?;
            let foot = patch.foot(frame.to_local(if which == 0 { a } else { b }));
            Some([foot.u, foot.v])
        };
        let mut before = at(0.0);
        for k in 1..=SAMPLES {
            let (lo, hi) = ((k - 1) as f64 / SAMPLES as f64, k as f64 / SAMPLES as f64);
            let here = at(hi);
            if let (Some(a), Some(b)) = (before, here) {
                for (direction, list) in knots.iter().enumerate() {
                    for &knot in list.iter() {
                        if (a[direction] - knot) * (b[direction] - knot) >= 0.0 {
                            continue;
                        }
                        let (mut x, mut y) = (lo, hi);
                        let below = a[direction] < knot;
                        for _ in 0..30 {
                            let m = (x + y) / 2.0;
                            match at(m) {
                                Some(q) if (q[direction] < knot) == below => x = m,
                                _ => y = m,
                            }
                        }
                        out.push((x + y) / 2.0);
                    }
                }
            }
            before = here;
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    out
}

/// A patch through stations, with `params` along it. A fit only converges quickly where the faces are smooth, and a
/// face's curvature can jump where one element gives way to the next, or at an open end, where a surface may stop being
/// itself and carry on along its tangents. So each stretch between `breaks` is fitted on its own, sharing only the
/// section where they meet, and they join with a knot of full multiplicity.
fn patch_through(stations: &[Station], params: &[f64], breaks: &[usize]) -> Result<Patch> {
    const DEGREE: usize = 3;
    let stuck = || Error::NearDegenerate("a blend's stations can't be interpolated".into());
    let across = stations[0].section.len();
    let mut knots_v: Vec<f64> = Vec::new();
    let mut columns: Vec<Vec<Hom>> = vec![Vec::new(); across];
    // Stretches of at least four stations, which a cubic needs.
    let mut stops = vec![0usize];
    for &b in breaks {
        if b >= stops[stops.len() - 1] + 3 && b + 3 < stations.len() {
            stops.push(b);
        }
    }
    stops.push(stations.len() - 1);
    for w in stops.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (degree, knots) = interpolation_knots(&params[a..=b]);
        if degree != DEGREE {
            return Err(stuck());
        }
        let collocation = Collocation::new(&params[a..=b], degree, &knots).ok_or_else(stuck)?;
        for (i, column) in columns.iter_mut().enumerate() {
            let values: Vec<Hom> = stations[a..=b].iter().map(|s| s.section[i]).collect();
            let solved = collocation.solve(&values);
            let skip = usize::from(!column.is_empty());
            column.extend_from_slice(&solved[skip..]);
        }
        if knots_v.is_empty() {
            knots_v = knots;
        } else {
            // The last value already appears `degree + 1` times, and an inner knot takes `degree`.
            knots_v.pop();
            knots_v.extend_from_slice(&knots[DEGREE + 1..]);
        }
    }
    let degree_u = across - 1;
    let mut knots_u = vec![0.0; across];
    knots_u.extend(vec![1.0; across]);
    let count = columns[0].len();
    let mut points = Vec::with_capacity(across * count);
    let mut weights = Vec::with_capacity(across * count);
    for column in &columns {
        for q in column {
            if !(q.w > 1e-6) {
                return Err(Error::NearDegenerate("a blend's weights went out of range between stations".into()));
            }
            points.push(q.point());
            weights.push(q.w);
        }
    }
    Patch::new((degree_u, DEGREE), (knots_u, knots_v), (across, count), points, weights)
}

/// Where a run of the chain on one side ends: a seam that the side's curve crosses there.
struct Crossing {
    v: f64,
    seam: usize,
    junction: usize,
}

/// What the chain's end has to be cut by.
struct End {
    vertex: usize,
    /// The edges along the left and right faces, and the face between them.
    left_edge: usize,
    right_edge: usize,
    third: usize,
}

fn end_at(at_vertex: &[Vec<usize>], uses: &[Vec<(usize, usize, usize, bool)>], vertex: usize, e: &Element) -> Result<End> {
    let others: Vec<usize> = at_vertex[vertex].iter().copied().filter(|&g| g != e.edge).collect();
    let [x, y] = others.as_slice() else {
        return Err(Error::Unsupported(format!(
            "a blended edge ending where {} edges meet needs a corner blend, which isn't available on these edges yet",
            others.len() + 1
        )));
    };
    let touches = |g: usize, f: usize| uses[g].iter().any(|u| u.0 == f);
    let (left_edge, right_edge) = if touches(*x, e.left) && touches(*y, e.right) {
        (*x, *y)
    } else if touches(*y, e.left) && touches(*x, e.right) {
        (*y, *x)
    } else {
        return Err(Error::Unsupported("a blended edge's end doesn't have one edge on each side, so it needs a corner blend".into()));
    };
    let third = uses[left_edge].iter().map(|u| u.0).find(|&f| f != e.left).ok_or_else(|| Error::Internal("an edge with one face".into()))?;
    if !touches(right_edge, third) {
        return Err(Error::Unsupported("a blended edge's end has more than one face beyond it, so it needs a corner blend".into()));
    }
    Ok(End { vertex, left_edge, right_edge, third })
}

fn too_big(kind: Kind) -> String {
    match kind {
        Kind::Fillet(r) => format!("a fillet of radius {r} doesn't fit on this edge"),
        Kind::Chamfer(d) => format!("a chamfer of {d} doesn't fit on this edge"),
    }
}

/// Moves one end of an edge to a new vertex at the point on it closest to `p`.
fn trim_edge(body: &mut Body, original: &Body, edge: usize, at: usize, new_vertex: usize, p: Vec3) -> Result<()> {
    let o = &original.edges[edge];
    let (t, off) = o.curve.closest(p, o.t0, o.t1);
    if off > 4e-7 {
        return Err(Error::InvalidInput(format!("a blend's side ends {off:.1e} past where a neighbouring edge does, which would take reshaping the neighbours")));
    }
    let e = &mut body.edges[edge];
    if o.v0 == at {
        e.t0 = t;
        e.v0 = new_vertex;
    } else if o.v1 == at {
        e.t1 = t;
        e.v1 = new_vertex;
    } else {
        return Err(Error::Internal("trimming an edge at a vertex it doesn't have".into()));
    }
    if !(e.t1 > e.t0) {
        return Err(Error::InvalidInput("a blend of this size uses up the whole of a neighbouring edge".into()));
    }
    Ok(())
}

/// Puts `pieces` in a face's loop in place of the coedges of `gone`, which have to run consecutively in it.
fn replace_in_loop(face: &mut Face, gone: &[usize], pieces: &[Coedge]) -> Result<()> {
    for lp in &mut face.loops {
        if !lp.iter().any(|c| gone.contains(&c.edge)) {
            continue;
        }
        let n = lp.len();
        // Start just after a coedge that stays, if there is one.
        let start = (0..n).find(|&i| !gone.contains(&lp[i].edge) && gone.contains(&lp[(i + 1) % n].edge)).map_or(0, |i| (i + 1) % n);
        let rest: Vec<Coedge> = (0..n).map(|k| lp[(start + k) % n]).skip_while(|c| gone.contains(&c.edge)).collect();
        if rest.iter().any(|c| gone.contains(&c.edge)) {
            return Err(Error::Unsupported("a blended chain that touches one face in separate places isn't available yet".into()));
        }
        let mut out = pieces.to_vec();
        out.extend(rest);
        *lp = out;
        return Ok(());
    }
    Err(Error::Internal("a blended edge isn't in the face beside it".into()))
}

/// How a fillet's radius goes along a chain of edges, where it isn't the same all along.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Varying {
    /// By how far along the chain, from 0 to 1, in straight lines between these, measured from the end nearest `start`, or
    /// without one, from the end with the lowest x, then y, then z.
    Along { stations: Vec<(f64, f64)>, start: Option<Vec3> },
    /// By edge. Edges in the chain with none take it smoothly from the radius before them to the one after.
    ByEdge(Vec<(usize, f64)>),
}

/// A radius worked out for each element of one chain.
struct Law {
    /// For each element, its radius at each end, and whether it goes between them smoothly or in a straight line.
    spans: Vec<Span>,
    /// For each element, where along it the radius changes direction, as a fit has to stop there.
    kinks: Vec<Vec<f64>>,
}

enum Span {
    /// In straight lines between these fractions along the element.
    Lines(Vec<(f64, f64)>),
    /// From one radius to another, level at both ends, between these fractions of the stretch it's in.
    Smooth { from: f64, to: f64, start: f64, end: f64 },
}

impl Law {
    fn new(varying: &Varying, chain: &Chain, body: &Body, lengths: &[f64]) -> Result<Law> {
        let els = &chain.elements;
        let total: f64 = lengths.iter().sum();
        let mut before = Vec::with_capacity(els.len());
        let mut sum = 0.0;
        for l in lengths {
            before.push(sum);
            sum += l;
        }
        match varying {
            Varying::Along { stations, start } => {
                if chain.closed {
                    return Err(Error::Unsupported("a radius that varies around a closed chain of edges isn't available yet: there's no end for it to start from".into()));
                }
                // Which end is 0.
                let (a, z) = (els[0].point(body, 0.0), els[els.len() - 1].point(body, 1.0));
                let flip = match start {
                    Some(p) => z.dist(*p) < a.dist(*p),
                    None => {
                        let key = |q: Vec3| [q.x, q.y, q.z];
                        let (ka, kz) = (key(a), key(z));
                        let first = ka.iter().zip(&kz).find(|(x, y)| (**x - **y).abs() > 1e-9);
                        first.is_some_and(|(x, y)| y < x)
                    }
                };
                let at_chain = |x: f64| -> f64 {
                    let x = x.clamp(0.0, 1.0);
                    let x = if flip { 1.0 - x } else { x };
                    if x <= stations[0].0 {
                        return stations[0].1;
                    }
                    for w in stations.windows(2) {
                        if x <= w[1].0 {
                            return w[0].1 + (w[1].1 - w[0].1) * (x - w[0].0) / (w[1].0 - w[0].0);
                        }
                    }
                    stations[stations.len() - 1].1
                };
                let mut spans = Vec::with_capacity(els.len());
                let mut kinks = Vec::with_capacity(els.len());
                for (i, l) in lengths.iter().enumerate() {
                    // Fractions along this element where a station falls.
                    let mut here: Vec<f64> = stations
                        .iter()
                        .map(|st| if flip { 1.0 - st.0 } else { st.0 })
                        .map(|x| (x * total - before[i]) / l)
                        .filter(|s| *s > 1e-9 && *s < 1.0 - 1e-9)
                        .collect();
                    here.sort_by(f64::total_cmp);
                    let mut points = vec![0.0];
                    points.extend(here.iter().copied());
                    points.push(1.0);
                    spans.push(Span::Lines(points.iter().map(|s| (*s, at_chain((before[i] + s * l) / total))).collect()));
                    kinks.push(here);
                }
                Ok(Law { spans, kinks })
            }
            Varying::ByEdge(list) => {
                let of = |e: usize| list.iter().find(|x| x.0 == e).map(|x| x.1);
                let n = els.len();
                // Straight from one radius to another, with nothing between them to go from one to the other, there's no
                // ball that rolls along it.
                let pairs = if chain.closed { n } else { n - 1 };
                for k in 0..pairs {
                    if let (Some(a), Some(b)) = (of(els[k].edge), of(els[(k + 1) % n].edge)) {
                        if (a - b).abs() > 1e-9 * a.max(b) {
                            return Err(Error::Unsupported(format!(
                                "two edges running smoothly into each other with radii {a} and {b} need an edge between them for the radius to go from one to the other"
                            )));
                        }
                    }
                }
                let mut spans = Vec::with_capacity(n);
                for i in 0..n {
                    if let Some(r) = of(els[i].edge) {
                        spans.push(Span::Lines(vec![(0.0, r), (1.0, r)]));
                        continue;
                    }
                    // How far back and on to edges that have a radius.
                    let step = |k: usize, forward: bool| -> Option<usize> {
                        if forward {
                            (k + 1 < n).then_some(k + 1).or(chain.closed.then_some(0))
                        } else {
                            (k > 0).then(|| k - 1).or(chain.closed.then_some(n - 1))
                        }
                    };
                    let walk = |forward: bool| -> (Option<f64>, f64) {
                        let (mut k, mut gone) = (i, 0.0);
                        for _ in 0..n {
                            let Some(next) = step(k, forward) else {
                                return (None, gone);
                            };
                            if let Some(r) = of(els[next].edge) {
                                return (Some(r), gone);
                            }
                            gone += lengths[next];
                            k = next;
                        }
                        (None, gone)
                    };
                    let ((from, back), (to, on)) = (walk(false), walk(true));
                    let (from, to) = match (from, to) {
                        (Some(a), Some(b)) => (a, b),
                        (Some(a), None) => (a, a),
                        (None, Some(b)) => (b, b),
                        (None, None) => return Err(Error::InvalidInput("none of this chain's edges has a radius".into())),
                    };
                    let stretch = back + lengths[i] + on;
                    spans.push(Span::Smooth { from, to, start: back / stretch, end: (back + lengths[i]) / stretch });
                }
                Ok(Law { spans, kinks: vec![Vec::new(); n] })
            }
        }
    }

    /// The radius a fraction of the way along an element.
    fn radius(&self, i: usize, s: f64) -> f64 {
        match &self.spans[i] {
            Span::Lines(points) => {
                for w in points.windows(2) {
                    if s <= w[1].0 {
                        return w[0].1 + (w[1].1 - w[0].1) * (s - w[0].0) / (w[1].0 - w[0].0);
                    }
                }
                points[points.len() - 1].1
            }
            Span::Smooth { from, to, start, end } => {
                let x = start + (end - start) * s;
                from + (to - from) * x * x * (3.0 - 2.0 * x)
            }
        }
    }
}

/// Fillets edges with a radius that varies along them.
pub(crate) fn blend_varying(body: &Body, edges: &[usize], varying: &Varying, feature: u32) -> Result<Body> {
    let largest = match varying {
        Varying::Along { stations, .. } => stations.iter().map(|s| s.1).fold(0.0, f64::max),
        Varying::ByEdge(list) => list.iter().map(|x| x.1).fold(0.0, f64::max),
    };
    blend_with(body, edges, Kind::Fillet(largest), Some(varying), feature)
}

/// Blends edges by working on the faces directly: see the notes at the top of this module.
pub(crate) fn blend(body: &Body, edges: &[usize], kind: Kind, feature: u32) -> Result<Body> {
    blend_with(body, edges, kind, None, feature)
}

fn blend_with(body: &Body, edges: &[usize], kind: Kind, varying: Option<&Varying>, feature: u32) -> Result<Body> {
    let uses = body.edge_uses();
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for (i, e) in body.edges.iter().enumerate() {
        at_vertex[e.v0].push(i);
        at_vertex[e.v1].push(i);
    }
    let chains = chains_of(body, edges, &uses, &at_vertex)?;
    let index = Index::new(body);
    let mut work = body.clone();
    let mut new_faces: Vec<Face> = Vec::new();
    let mut placed: Vec<Placed> = Vec::new();
    for chain in &chains {
        blend_chain(body, &mut work, &index, chain, kind, varying, feature, &uses, &at_vertex, &mut new_faces, &mut placed)?;
    }
    let out = finish(work, new_faces, body, &index, &placed, kind)?;
    // Along each edge, whether it's convex, as the chain says.
    let mut along = Vec::new();
    for chain in &chains {
        for e in &chain.elements {
            for k in 1..8 {
                let s = k as f64 / 8.0;
                let (p, t) = (e.point(body, s), e.tangent(body, s));
                let turn = out_normal(&body.faces[e.left], p).cross(out_normal(&body.faces[e.right], p)).dot(t);
                along.push((p, turn > 0.0, false));
            }
        }
    }
    let name = match kind {
        Kind::Fillet(_) => "fillet",
        Kind::Chamfer(_) => "chamfer",
    };
    crate::blend::check_blend(body, &out, &along, name)?;
    Ok(out)
}

/// A stretch of a new face, to check that it's where a blend belongs.
struct Placed {
    patch: Arc<Patch>,
    from: f64,
    to: f64,
    side: f64,
}

/// Fillets edges by this route even where an exact one exists, which is how the exact ones can check it.
pub fn fillet_fitted(body: &Body, edges: &[usize], radius: f64, feature: u32) -> Result<Body> {
    blend(body, edges, Kind::Fillet(radius), feature)
}

/// Chamfers edges by this route even where an exact one exists.
pub fn chamfer_fitted(body: &Body, edges: &[usize], distance: f64, feature: u32) -> Result<Body> {
    blend(body, edges, Kind::Chamfer(distance), feature)
}

#[allow(clippy::too_many_arguments)]
fn blend_chain(
    body: &Body,
    work: &mut Body,
    index: &Index,
    chain: &Chain,
    kind: Kind,
    varying: Option<&Varying>,
    feature: u32,
    uses: &[Vec<(usize, usize, usize, bool)>],
    at_vertex: &[Vec<usize>],
    new_faces: &mut Vec<Face>,
    placed: &mut Vec<Placed>,
) -> Result<()> {
    let els = &chain.elements;
    let size = match kind {
        Kind::Fillet(r) | Kind::Chamfer(r) => r,
    };
    // Convex or concave, the same all the way along.
    let mut side = 0.0;
    for e in els {
        let p = e.point(body, 0.5);
        let t = e.tangent(body, 0.5);
        let turn = out_normal(&body.faces[e.left], p).cross(out_normal(&body.faces[e.right], p)).dot(t);
        let here = if turn > 0.0 { -1.0 } else { 1.0 };
        if side != 0.0 && here != side {
            return Err(Error::Unsupported("a chain of edges that goes from convex to concave along its length needs a blend this kernel doesn't have".into()));
        }
        side = here;
    }

    let (start_end, finish_end) = if chain.closed {
        (None, None)
    } else {
        let (a, z) = (&els[0], &els[els.len() - 1]);
        (Some(end_at(at_vertex, uses, a.start, a)?), Some(end_at(at_vertex, uses, z.end, z)?))
    };
    // Vertices along the way may only have seams besides the chain's own edges.
    let in_chain = |g: usize| els.iter().any(|e| e.edge == g);
    let joints: Vec<usize> = if chain.closed { (0..els.len()).collect() } else { (1..els.len()).collect() };
    let crease = || Error::Unsupported("a blended edge passing a vertex where a sharp edge meets it needs a corner blend, which isn't available on these edges yet".into());
    let reach = match kind {
        Kind::Fillet(r) | Kind::Chamfer(r) => r,
    };
    for &j in &joints {
        let v = els[j].start;
        for &g in &at_vertex[v] {
            if in_chain(g) {
                continue;
            }
            if is_sharp_at(body, uses, g, v) {
                return Err(crease());
            }
            // Smooth at the vertex isn't enough: the ball touches a little way along, and rolling over a crease there
            // is a corner blend too, as on the rails of a loft that are only smooth right at the end.
            let e = &body.edges[g];
            let length = (e.t1 - e.t0) * e.curve.speed_bound();
            let along = (2.0 * reach / length.max(1e-12)).min(1.0);
            let t = if e.v0 == v { e.t0 + (e.t1 - e.t0) * along } else { e.t1 - (e.t1 - e.t0) * along };
            let p = e.curve.eval(t);
            if let [x, y] = uses[g].as_slice() {
                if out_normal(&body.faces[x.0], p).cross(out_normal(&body.faces[y.0], p)).norm() > 2e-3 {
                    return Err(crease());
                }
            }
        }
    }

    // Stations, doubling until the fit holds.
    let lengths: Vec<f64> = els.iter().map(|e| (0..8).map(|k| e.point(body, k as f64 / 8.0).dist(e.point(body, (k + 1) as f64 / 8.0))).sum::<f64>()).collect();
    let along = |left: bool| -> Vec<usize> {
        let mut list: Vec<usize> = Vec::new();
        for e in els {
            let f = if left { e.left } else { e.right };
            if !list.contains(&f) {
                list.push(f);
            }
        }
        list
    };
    // The current element's face first, for where a point is equally on two.
    let solver_at = |e: &Element| {
        let mut lefts = along(true);
        let mut rights = along(false);
        lefts.sort_by_key(|f| *f != e.left);
        rights.sort_by_key(|f| *f != e.right);
        Solver { lefts, rights, kind, side, body, index }
    };
    // Where it isn't the same all along, what it is at each place.
    let law = varying.map(|v| Law::new(v, chain, body, &lengths)).transpose()?;
    let kind_at = |i: usize, s: f64| -> Kind { law.as_ref().map_or(kind, |l| Kind::Fillet(l.radius(i, s))) };
    // A closed chain's patches join partway along elements, and each part needs enough for a cubic.
    let least = if chain.closed { 8 } else { 4 };
    let mut per_element: Vec<usize> = lengths.iter().map(|l| ((l / (0.75 * size)).ceil() as usize).clamp(least, 64)).collect();
    let crossings: Vec<Vec<f64>> = els
        .iter()
        .enumerate()
        .map(|(i, e)| {
            let mut list = knot_crossings(body, e, &solver_at(e), &|s| kind_at(i, s));
            if let Some(l) = &law {
                list.extend(l.kinks[i].iter().copied());
                list.sort_by(f64::total_cmp);
                list.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
            }
            list
        })
        .collect();
    let mut last_error = f64::INFINITY;
    loop {
        let total: usize = per_element.iter().sum();
        if total > MAX_STATIONS {
            return Err(Error::NearDegenerate(format!("a blend on these surfaces only came within {last_error:.1e} of them with {MAX_STATIONS} stations")));
        }
        let mut stations: Vec<Station> = Vec::with_capacity(total + 1);
        let mut near = None;
        // Which station each element starts at, and which element each stretch between stations belongs to.
        let mut joints: Vec<usize> = Vec::with_capacity(els.len() + 1);
        let mut owner: Vec<usize> = Vec::with_capacity(total);
        // Stations where a fit starts afresh: at joints, and where a side crosses a knot line.
        let mut stops: Vec<usize> = Vec::new();
        for (i, e) in els.iter().enumerate() {
            let solver = solver_at(e);
            joints.push(stations.len() - usize::from(i > 0));
            // Evenly spaced between the crossings, with enough in each stretch for a cubic.
            let mut fractions = vec![0.0];
            let edges: Vec<f64> = crossings[i].iter().copied().chain([1.0]).collect();
            let mut from = 0.0f64;
            for (k, &to) in edges.iter().enumerate() {
                let count = ((per_element[i] as f64 * (to - from)).ceil() as usize).max(3);
                for j in 1..=count {
                    fractions.push(from + (to - from) * j as f64 / count as f64);
                }
                if k + 1 < edges.len() {
                    stops.push(stations.len() + fractions.len() - 1 - usize::from(i > 0));
                }
                from = to;
            }
            for &s in &fractions[usize::from(i > 0)..] {
                let st = solver.station_as(kind_at(i, s), e.point(body, s), e.tangent(body, s), near).ok_or_else(|| Error::InvalidInput(too_big(kind_at(i, s))))?;
                near = Some(st.center);
                stations.push(st);
            }
            owner.extend(std::iter::repeat_n(i, fractions.len() - 1));
        }
        joints.push(stations.len() - 1);
        if chain.closed {
            // Exactly closed, and starting partway along an element, so that where the patches join is never at a
            // joint, which is where the sides cross seams and the two would land on one point.
            let last = stations.len() - 1;
            let turn = (joints[1] - joints[0]) / 2;
            stations = (0..=last).map(|k| stations[(k + turn) % last].clone()).collect();
            owner = (0..last).map(|k| owner[(k + turn) % last]).collect();
            for j in joints.iter_mut().chain(stops.iter_mut()) {
                *j = (*j + last - turn) % last;
            }
        }
        // Past open ends, far enough that the end's third face cuts the whole section.
        let mut before: Vec<Station> = Vec::new();
        let mut after: Vec<Station> = Vec::new();
        if let (Some(a), Some(z)) = (&start_end, &finish_end) {
            let step = lengths[0] / per_element[0] as f64;
            let solver = Solver { kind: kind_at(0, 0.0), ..solver_at(&els[0]) };
            before = extend(body, &solver, els[0].point(body, 0.0), els[0].tangent(body, 0.0), -step.min(size * 0.5).max(size * 0.05), a.third, stations[0].clone(), els[0].point(body, 0.5), kind)?;
            let l = els.len() - 1;
            let step = lengths[l] / per_element[l] as f64;
            let solver = Solver { kind: kind_at(l, 1.0), ..solver_at(&els[l]) };
            after = extend(body, &solver, els[l].point(body, 1.0), els[l].tangent(body, 1.0), step.min(size * 0.5).max(size * 0.05), z.third, stations[stations.len() - 1].clone(), els[l].point(body, 0.5), kind)?;
        }
        let lead = before.len();
        let mut all: Vec<Station> = before.into_iter().rev().collect();
        all.extend(stations.iter().cloned());
        all.extend(after);
        let mut params = vec![0.0f64];
        for w in all.windows(2) {
            let gap = w[0].mid.dist(w[1].mid);
            if !(gap > 1e-9) {
                return Err(Error::InvalidInput(too_big(kind)));
            }
            params.push(params[params.len() - 1] + gap);
        }
        // A closed chain is two patches, since a face can't wrap all the way round on one.
        let cuts: Vec<(usize, usize)> = if chain.closed {
            // Partway along an element, for the same reason as the start.
            let k = els.len() / 2;
            let span = (joints[(k + 1) % els.len()] + all.len() - 1 - joints[k]) % (all.len() - 1);
            let m = (joints[k] + span / 2) % (all.len() - 1);
            vec![(0, m), (m, all.len() - 1)]
        } else {
            vec![(0, all.len() - 1)]
        };
        let mut patches = Vec::with_capacity(cuts.len());
        let mut worst = 0.0f64;
        for &(a, b) in &cuts {
            let mut breaks: Vec<usize> = joints.iter().chain(stops.iter()).map(|j| lead + j).filter(|j| *j > a && *j < b).map(|j| j - a).collect();
            breaks.sort_unstable();
            breaks.dedup();
            let patch = patch_through(&all[a..=b], &params[a..=b], &breaks).map_err(|e| match e {
                Error::InvalidInput(why) if why.contains("fold") || why.contains("pole") => Error::InvalidInput(too_big(kind)),
                other => other,
            })?;
            // Halfway between stations, where a fit is at its worst, measured on the faces themselves. What's past
            // an open end gets cut off, so it doesn't count.
            for j in a.max(lead)..b.min(lead + stations.len() - 1) {
                let v = (params[j] + params[j + 1]) / 2.0;
                let (pa, pb) = (patch.eval(0.0, v), patch.eval(1.0, v));
                // By a joint it can be on the next element's face, whose surface may not be this one's.
                let solver = solver_at(&els[owner[j - lead]]);
                let on_left = solver.holding(&solver.lefts, pa).surface.sdf(pa).abs();
                let on_right = solver.holding(&solver.rights, pb).surface.sdf(pb).abs();
                worst = worst.max(on_left).max(on_right);
            }
            patches.push(patch);
        }
        last_error = worst;
        if worst > FIT {
            for n in &mut per_element {
                *n *= 2;
            }
            continue;
        }
        let frame = Frame::WORLD;
        let surfaces: Vec<Surface> = patches.into_iter().map(|p| Surface::Free { frame, patch: Arc::new(p) }).collect();
        let shape = Shape { surfaces: &surfaces, params: &params, cuts: &cuts, joints: &joints, lead, side };
        return assemble(body, work, index, chain, kind, feature, uses, at_vertex, new_faces, placed, &shape, start_end, finish_end);
    }
}

/// The fitted surfaces of a chain, and how they lie along it.
struct Shape<'a> {
    surfaces: &'a [Surface],
    params: &'a [f64],
    cuts: &'a [(usize, usize)],
    joints: &'a [usize],
    lead: usize,
    side: f64,
}

/// Stations beyond an open end, until the third face has been passed by every point of the section. `inside` is a
/// point of the edge away from its end, which says which side of that face is the near one. A section's plane keeps
/// the direction of travel, which says which face is on which side, also going backward from a chain's start, where
/// the step is negative.
#[allow(clippy::too_many_arguments)]
fn extend(body: &Body, solver: &Solver, from: Vec3, travel: Vec3, step: f64, third: usize, first: Station, inside: Vec3, kind: Kind) -> Result<Vec<Station>> {
    let c = &body.faces[third];
    let near_side = c.surface.sdf(inside);
    let points = |s: &Station| -> Vec<Vec3> { s.section.iter().map(|q| q.point()).chain([s.mid]).collect() };
    let mut out = Vec::new();
    let mut near = Some(first.center);
    let mut passed = 0;
    for k in 1..=200 {
        let st = solver.station(from + travel * (step * k as f64), travel, near).ok_or_else(|| Error::InvalidInput(too_big(kind)))?;
        near = Some(st.center);
        let beyond = points(&st).iter().all(|p| c.surface.sdf(*p) * near_side < 0.0);
        out.push(st);
        if beyond {
            passed += 1;
            if passed >= 2 && out.len() >= 3 {
                return Ok(out);
            }
        }
    }
    Err(Error::NearDegenerate("a blend's end never gets past the face that should cut it".into()))
}

/// The curve where a blend's patch meets a third face, across from `u = 0` to `u = 1`, or the other way. It looks
/// between two parameters along the patch.
fn cut_across(surface: &Surface, patch: &Patch, third: &Surface, inside: f64, outside: f64, forward: bool, kind: Kind) -> Result<(Curve, f64, f64, Vec3, Vec3, f64, f64)> {
    const ACROSS: usize = 16;
    let mut points = Vec::with_capacity(ACROSS + 1);
    let mut v_at_ends = (0.0, 0.0);
    for k in 0..=ACROSS {
        let u = if forward { k as f64 / ACROSS as f64 } else { 1.0 - k as f64 / ACROSS as f64 };
        let (mut lo, mut hi) = (inside, outside);
        let mut flo = third.sdf(patch.eval(u, lo));
        let fhi = third.sdf(patch.eval(u, hi));
        if flo * fhi > 0.0 {
            return Err(Error::InvalidInput(too_big(kind)));
        }
        for _ in 0..60 {
            let m = (lo + hi) / 2.0;
            let f = third.sdf(patch.eval(u, m));
            if f * flo > 0.0 {
                lo = m;
                flo = f;
            } else {
                hi = m;
            }
        }
        let v = (lo + hi) / 2.0;
        // The two ends stay on the patch's edges, so they stay on the neighbouring faces.
        let p = if k == 0 || k == ACROSS {
            if k == 0 {
                v_at_ends.0 = v;
            } else {
                v_at_ends.1 = v;
            }
            patch.eval(u, v)
        } else {
            refine_on_two(surface, third, patch.eval(u, v)).ok_or_else(|| Error::NearDegenerate(format!("a blend meets a {} too close to tangent to cut it", third.name())))?
        };
        points.push(p);
    }
    let (a, b) = (points[0], points[ACROSS]);
    let curve = sampled_from_points(points, surface, third)?;
    let last = match &curve {
        Curve::Sampled(c) => (c.points.len() - 1) as f64,
        _ => ACROSS as f64,
    };
    Ok((curve, 0.0, last, a, b, v_at_ends.0, v_at_ends.1))
}

#[allow(clippy::too_many_arguments)]
fn assemble(
    body: &Body,
    work: &mut Body,
    index: &Index,
    chain: &Chain,
    kind: Kind,
    feature: u32,
    uses: &[Vec<(usize, usize, usize, bool)>],
    at_vertex: &[Vec<usize>],
    new_faces: &mut Vec<Face>,
    placed: &mut Vec<Placed>,
    shape: &Shape,
    start_end: Option<End>,
    finish_end: Option<End>,
) -> Result<()> {
    let els = &chain.elements;
    let Shape { surfaces, params, cuts, joints, lead, side } = *shape;
    let patch_of = |s: &Surface| -> Arc<Patch> {
        match s {
            Surface::Free { patch, .. } => patch.clone(),
            _ => unreachable!(),
        }
    };
    let seg_of = |v: f64| -> usize { cuts.iter().position(|&(_, b)| v <= params[b]).unwrap_or(cuts.len() - 1) };
    let iso = |seg: usize, u: f64| -> Result<Spline> { patch_of(&surfaces[seg]).iso_curve(u, true) };
    let point_at = |side_u: f64, v: f64| patch_of(&surfaces[seg_of(v)]).eval(side_u, v);

    // The first station from an open end whose whole section is on the edge's side of the third face.
    let clear_of = |third: &Surface, seg: usize, from: usize, step: i64, near: Vec3| -> usize {
        let patch = patch_of(&surfaces[seg]);
        let reference = third.sdf(near);
        let mut j = from as i64;
        while j >= 0 && (j as usize) < params.len() {
            let v = params[j as usize];
            if v >= params[cuts[seg].0] && v <= params[cuts[seg].1] && [0.0, 0.25, 0.5, 0.75, 1.0].iter().all(|u| third.sdf(patch.eval(*u, v)) * reference > 0.0) {
                return j as usize;
            }
            j += step;
        }
        from
    };

    // Open ends first: where the patch meets the third face, which fixes the two ends of each side.
    let (mut range_start, mut range_end) = (params[0], params[params.len() - 1]);
    let mut start_trace = None;
    let mut finish_trace = None;
    if let Some(end) = &start_end {
        let p = patch_of(&surfaces[0]);
        let third = &body.faces[end.third].surface;
        let inside = clear_of(third, 0, lead, 1, els[0].point(body, 0.5));
        let (curve, t0, t1, a, b, va, vb) = cut_across(&surfaces[0], &p, third, params[inside], params[0], true, kind)?;
        range_start = va.max(vb);
        start_trace = Some((curve, t0, t1, a, b, va, vb));
    }
    if let Some(end) = &finish_end {
        let last = surfaces.len() - 1;
        let p = patch_of(&surfaces[last]);
        let third = &body.faces[end.third].surface;
        let z = &els[els.len() - 1];
        let inside = clear_of(third, last, lead + joints[joints.len() - 1], -1, z.point(body, 0.5));
        let (curve, t0, t1, b, a, vb, va) = cut_across(&surfaces[last], &p, third, params[inside], params[params.len() - 1], false, kind)?;
        range_end = va.min(vb);
        finish_trace = Some((curve, t0, t1, a, b, va, vb));
    }

    // One side at a time: where it crosses seams, and which face each stretch is on.
    let mut vertex_at_cuts: Vec<[usize; 2]> = Vec::new();
    for &(a, _) in cuts {
        let v = params[a];
        if chain.closed {
            let left = work.add_vertex(point_at(0.0, v));
            let right = work.add_vertex(point_at(1.0, v));
            vertex_at_cuts.push([left, right]);
        }
    }
    let mut piece_edges: [Vec<(f64, f64, Coedge)>; 2] = [Vec::new(), Vec::new()];
    for (which, side_u) in [(0usize, 0.0f64), (1, 1.0)] {
        let face_of = |e: &Element| if which == 0 { e.left } else { e.right };
        // Crossings of seams, at each joint where the side changes face.
        let mut crossings: Vec<Crossing> = Vec::new();
        let joint_list: Vec<usize> = if chain.closed { (0..els.len()).collect() } else { (1..els.len()).collect() };
        for &j in &joint_list {
            let before = &els[if j == 0 { els.len() - 1 } else { j - 1 }];
            let this = &els[j];
            if face_of(before) == face_of(this) {
                continue;
            }
            let (fa, fb) = (face_of(before), face_of(this));
            let vertex = this.start;
            let seam = at_vertex[vertex]
                .iter()
                .copied()
                .find(|&g| g != before.edge && g != this.edge && uses[g].iter().any(|u| u.0 == fa) && uses[g].iter().any(|u| u.0 == fb))
                .ok_or_else(|| Error::Unsupported("a blend passing between two faces with no seam between them needs a corner blend".into()))?;
            let center = params[lead + joints[j]];
            let s = &body.edges[seam];
            let measure = |v: f64| -> f64 {
                let q = wrapped_point(surfaces, params, cuts, side_u, v, chain.closed);
                let (t, _) = s.curve.closest(q, s.t0, s.t1);
                let on = s.curve.eval(t);
                let n = out_normal(&body.faces[fa], on);
                (q - on).dot(n.cross(s.curve.derivative(t)))
            };
            let total = params[params.len() - 1] - params[0];
            let mut reach = (params[(lead + joints[j] + 1).min(params.len() - 1)] - center).abs().max(1e-6);
            let mut bracket = None;
            for _ in 0..12 {
                let (x, y) = (center - reach, center + reach);
                if measure(x) * measure(y) <= 0.0 {
                    bracket = Some((x, y));
                    break;
                }
                reach *= 2.0;
                if reach > total / 2.0 {
                    break;
                }
            }
            let (mut x, mut y) = bracket.ok_or_else(|| Error::InvalidInput(too_big(kind)))?;
            let fx = measure(x);
            for _ in 0..60 {
                let m = (x + y) / 2.0;
                if measure(m) * fx > 0.0 {
                    x = m;
                } else {
                    y = m;
                }
            }
            let mut v = (x + y) / 2.0;
            if chain.closed {
                let (lo, hi) = (params[0], params[params.len() - 1]);
                if v < lo {
                    v += hi - lo;
                } else if v >= hi {
                    v -= hi - lo;
                }
            }
            crossings.push(Crossing { v, seam, junction: vertex });
        }
        crossings.sort_by(|a, b| a.v.total_cmp(&b.v));

        // Vertices along this side, in order.
        let mut stops: Vec<(f64, usize)> = Vec::new();
        if chain.closed {
            for (k, &(a, _)) in cuts.iter().enumerate() {
                stops.push((params[a], vertex_at_cuts[k][which]));
            }
        } else {
            let (first_v, last_v) = (
                if which == 0 { start_trace.as_ref().map(|t| t.5) } else { start_trace.as_ref().map(|t| t.6) },
                if which == 0 { finish_trace.as_ref().map(|t| t.5) } else { finish_trace.as_ref().map(|t| t.6) },
            );
            let (first_v, last_v) = (first_v.unwrap_or(range_start), last_v.unwrap_or(range_end));
            let (a, z) = (work.add_vertex(point_at(side_u, first_v)), work.add_vertex(point_at(side_u, last_v)));
            stops.push((first_v, a));
            stops.push((last_v, z));
        }
        for c in &crossings {
            let p = wrapped_point(surfaces, params, cuts, side_u, c.v, chain.closed);
            let id = work.add_vertex(p);
            trim_edge(work, body, c.seam, c.junction, id, p)?;
            stops.push((c.v, id));
        }
        stops.sort_by(|a, b| a.0.total_cmp(&b.0));
        if chain.closed {
            let hi = params[params.len() - 1];
            let first = stops[0].1;
            stops.push((hi, first));
        }
        // Contact edges between the stops.
        for w in stops.windows(2) {
            let ((a, va), (b, vb)) = (w[0], w[1]);
            if !(b > a) {
                return Err(Error::InvalidInput(too_big(kind)));
            }
            let seg = seg_of((a + b) / 2.0);
            let curve = Curve::Spline(Arc::new(iso(seg, side_u)?));
            let c = work.add_edge(curve, a, b, va, vb);
            piece_edges[which].push((a, b, c));
        }
        // Which face each piece is on, and that it stays on it.
        let candidates: Vec<usize> = {
            let mut list: Vec<usize> = els.iter().map(|e| face_of(e)).collect();
            list.dedup();
            list
        };
        let mut pieces_of_face: Vec<(usize, Vec<Coedge>)> = Vec::new();
        for &(a, b, c) in &piece_edges[which] {
            // Which face this stretch is on, and that every point of it is on one of them.
            let mut votes = vec![0usize; candidates.len()];
            for k in 1..16 {
                let q = point_at(side_u, a + (b - a) * k as f64 / 16.0);
                let mut any = false;
                for (n, &f) in candidates.iter().enumerate() {
                    match index.locate_on_face(f, q, 5e-7) {
                        FaceLoc::Inside => {
                            votes[n] += 2;
                            any = true;
                        }
                        FaceLoc::Boundary => {
                            votes[n] += 1;
                            any = true;
                        }
                        FaceLoc::Outside => {}
                    }
                }
                if !any {
                    return Err(Error::InvalidInput(format!("{}: it runs off a neighbouring face", too_big(kind))));
                }
            }
            let f = candidates[(0..candidates.len()).max_by_key(|&n| votes[n]).unwrap_or(0)];
            let coedge = if which == 0 { c } else { c.flipped() };
            match pieces_of_face.last_mut() {
                Some((g, list)) if *g == f => list.push(coedge),
                _ => pieces_of_face.push((f, vec![coedge])),
            }
        }
        if chain.closed && pieces_of_face.len() > 1 && pieces_of_face[0].0 == pieces_of_face[pieces_of_face.len() - 1].0 {
            let (_, tail) = pieces_of_face.pop().unwrap();
            let mut joined = tail;
            joined.extend(pieces_of_face[0].1.iter().copied());
            pieces_of_face[0].1 = joined;
        }
        for (f, mut list) in pieces_of_face {
            if which == 1 {
                list.reverse();
            }
            let gone: Vec<usize> = els.iter().filter(|e| face_of(e) == f).map(|e| e.edge).collect();
            replace_in_loop(&mut work.faces[f], &gone, &list)?;
        }
    }

    // Open ends: shorten the two edges along the sides, and add the cut to the third face.
    let add_end = |work: &mut Body, end: &End, trace: &(Curve, f64, f64, Vec3, Vec3, f64, f64), at_start: bool, piece_edges: &[Vec<(f64, f64, Coedge)>; 2]| -> Result<Coedge> {
        let (curve, t0, t1, ..) = trace;
        let side_vertex = |which: usize| -> usize {
            let (_, _, c) = if at_start { piece_edges[which][0] } else { piece_edges[which][piece_edges[which].len() - 1] };
            let e = &work.edges[c.edge];
            if at_start {
                e.v0
            } else {
                e.v1
            }
        };
        let (left, right) = (side_vertex(0), side_vertex(1));
        let (pl, pr) = (work.vertices[left], work.vertices[right]);
        trim_edge(work, body, end.left_edge, end.vertex, left, pl)?;
        trim_edge(work, body, end.right_edge, end.vertex, right, pr)?;
        // The cut runs left to right at the start of the chain, and right to left at its end, as the patch's loop has it.
        let (from, to) = if at_start { (left, right) } else { (right, left) };
        let c = work.add_edge(curve.clone(), *t0, *t1, from, to);
        // In the third face's loop it goes the other way, between the two shortened edges.
        let third = &mut work.faces[end.third];
        let (first, second) = if at_start { (end.right_edge, end.left_edge) } else { (end.left_edge, end.right_edge) };
        let mut placed = false;
        for lp in &mut third.loops {
            for i in 0..lp.len() {
                let j = (i + 1) % lp.len();
                if lp[i].edge == first && lp[j].edge == second {
                    lp.insert(i + 1, c.flipped());
                    placed = true;
                    break;
                }
            }
            if placed {
                break;
            }
        }
        if !placed {
            return Err(Error::Internal("a blend's end isn't where the third face's loop says".into()));
        }
        Ok(c)
    };
    let start_coedge = match (&start_end, &start_trace) {
        (Some(end), Some(trace)) => Some(add_end(work, end, trace, true, &piece_edges)?),
        _ => None,
    };
    let finish_coedge = match (&finish_end, &finish_trace) {
        (Some(end), Some(trace)) => Some(add_end(work, end, trace, false, &piece_edges)?),
        _ => None,
    };

    // The blend's own faces.
    let first = &els[0];
    let mut between = vec![body.faces[first.left].tag, body.faces[first.right].tag];
    between.sort_unstable();
    between.dedup();
    let copies = [first.left, first.right].iter().map(|&f| &body.faces[f].origin.copies).max().cloned().unwrap_or_default();
    let origin = FaceOrigin { copies, between };
    let tag = FaceTag { feature, slot: SLOT_SEG_BASE + 1 };
    // Between a closed chain's patches, one edge across, which both of them have since it's the same station.
    let mut arcs: Vec<Coedge> = Vec::new();
    if chain.closed {
        for (k, &(a, _)) in cuts.iter().enumerate() {
            let curve = patch_of(&surfaces[k]).iso_curve(params[a], false)?;
            let (t0, t1) = curve.domain();
            arcs.push(work.add_edge(Curve::Spline(Arc::new(curve)), t0, t1, vertex_at_cuts[k][0], vertex_at_cuts[k][1]));
        }
    }
    for (k, &(a, b)) in cuts.iter().enumerate() {
        let (lo, hi) = (params[a], params[b]);
        let inside = |p: &(f64, f64, Coedge)| p.0 >= lo - 1e-12 && p.1 <= hi + 1e-12;
        // Across the start, left to right. Then up the right side, across the end, and back down the left.
        let mut lp = Vec::new();
        lp.push(if chain.closed { arcs[k] } else { start_coedge.ok_or_else(|| Error::Internal("an open chain with no start".into()))? });
        lp.extend(piece_edges[1].iter().filter(|p| inside(p)).map(|p| p.2));
        lp.push(if chain.closed { arcs[(k + 1) % cuts.len()].flipped() } else { finish_coedge.ok_or_else(|| Error::Internal("an open chain with no end".into()))? });
        let mut back: Vec<Coedge> = piece_edges[0].iter().filter(|p| inside(p)).map(|p| p.2.flipped()).collect();
        back.reverse();
        lp.extend(back);
        let chart = work.fit_face_chart(&surfaces[k], false, std::slice::from_ref(&lp));
        new_faces.push(Face { surface: surfaces[k].clone(), flipped: false, chart, loops: vec![lp], tag, origin: origin.clone() });
        placed.push(Placed { patch: patch_of(&surfaces[k]), from: lo.max(range_start), to: hi.min(range_end), side });
    }
    Ok(())
}

/// A point on one side of the blend, with `v` allowed past either end of a closed chain.
fn wrapped_point(surfaces: &[Surface], params: &[f64], cuts: &[(usize, usize)], u: f64, v: f64, closed: bool) -> Vec3 {
    let (lo, hi) = (params[0], params[params.len() - 1]);
    let mut v = v;
    if closed {
        if v < lo {
            v += hi - lo;
        } else if v > hi {
            v -= hi - lo;
        }
    }
    let seg = cuts.iter().position(|&(_, b)| v <= params[b]).unwrap_or(cuts.len() - 1);
    match &surfaces[seg] {
        Surface::Free { patch, .. } => patch.eval(u, v.clamp(params[cuts[seg].0], params[cuts[seg].1])),
        _ => unreachable!(),
    }
}

/// Takes out everything no loop uses any more, then checks the whole result.
fn finish(mut work: Body, new_faces: Vec<Face>, _original: &Body, index: &Index, placed: &[Placed], kind: Kind) -> Result<Body> {
    work.faces.extend(new_faces);
    let mut edge_map = vec![usize::MAX; work.edges.len()];
    let mut vertex_map = vec![usize::MAX; work.vertices.len()];
    let mut edges: Vec<Edge> = Vec::new();
    let mut vertices: Vec<Vec3> = Vec::new();
    for face in &mut work.faces {
        for lp in &mut face.loops {
            for c in lp.iter_mut() {
                if edge_map[c.edge] == usize::MAX {
                    edge_map[c.edge] = edges.len();
                    let mut e = work.edges[c.edge].clone();
                    for v in [&mut e.v0, &mut e.v1] {
                        if vertex_map[*v] == usize::MAX {
                            vertex_map[*v] = vertices.len();
                            vertices.push(work.vertices[*v]);
                        }
                        *v = vertex_map[*v];
                    }
                    edges.push(e);
                }
                c.edge = edge_map[c.edge];
            }
        }
    }
    let mut out = Body { vertices, edges, faces: Vec::with_capacity(work.faces.len()), shells: Vec::new(), tolerance: work.tolerance, sheet: false };
    for f in std::mem::take(&mut work.faces) {
        let i = out.add_face(f.surface, f.flipped, f.loops, f.tag);
        out.faces[i].origin = f.origin;
    }
    out.compute_shells();
    validate(&out).map_err(|defects| Error::NearDegenerate(format!("a blend didn't come out valid: {}", defects.join("; "))))?;
    let mesh = tessellate(&out, &TessParams::for_size(out.aabb().diagonal()))?;
    if mesh.open_edge_count() != 0 {
        return Err(Error::NearDegenerate("a blend didn't mesh closed".into()));
    }
    // A blend belongs inside the part on a convex edge and outside it on a concave one. Anywhere else, it has run
    // through some other part of the body, which nothing above would notice.
    for p in placed {
        for k in 1..16 {
            let q = p.patch.eval(0.5, p.from + (p.to - p.from) * k as f64 / 16.0);
            let loc = index.locate_in_body(q);
            let wrong = if p.side < 0.0 { loc == BodyLoc::Outside } else { loc == BodyLoc::Inside };
            if wrong {
                return Err(Error::InvalidInput(format!("{}: it runs through another part of the body", too_big(kind))));
            }
        }
    }
    Ok(out)
}
