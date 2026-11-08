//! A solid through sections: one free-form patch for each stretch of outline, running from section to section.
//!
//! Each outline piece becomes rational Bezier pieces in 3D, and matching pieces of all the sections are brought
//! to one degree and one set of breaks, so that their control points correspond. Those then go through the
//! sections: straight between two, and as a cubic through three or more. A line stays a line and a circle stays a
//! circle, exactly, because interpolating control points in homogeneous form does that.

use crate::error::{Error, Result};
use crate::math::*;
use crate::curve::Curve;
use crate::patch::{interpolate, interpolation_knots, Patch};
use crate::profile::{Profile, Seg, SegGeom};
use crate::spline::{Hom, Spline};
use crate::surface::Surface;
use crate::build::{SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::topo::{Body, Coedge, FaceTag};
use crate::validate::validate;
use std::sync::Arc;

/// One outline piece in 3D, as rational Bezier pieces over 0 to 1.
#[derive(Clone)]
struct Chain {
    spans: Vec<Vec<Hom>>,
    breaks: Vec<f64>,
    id: u32,
}

fn split_span(h: &[Hom], s: f64) -> (Vec<Hom>, Vec<Hom>) {
    let n = h.len();
    let mut work = h.to_vec();
    let (mut left, mut right) = (vec![Hom::ZERO; n], vec![Hom::ZERO; n]);
    for level in 0..n {
        left[level] = work[0];
        right[n - 1 - level] = work[n - 1 - level];
        for i in 0..n - 1 - level {
            work[i] = work[i].lerp(work[i + 1], s);
        }
    }
    (left, right)
}

fn elevate(h: &[Hom]) -> Vec<Hom> {
    let p = h.len() - 1;
    let mut out = Vec::with_capacity(p + 2);
    out.push(h[0]);
    for i in 1..=p {
        out.push(h[i - 1].lerp(h[i], 1.0 - i as f64 / (p + 1) as f64));
    }
    out.push(h[p]);
    out
}

impl Chain {
    fn of(seg: &Seg, frame: &Frame) -> Result<Chain> {
        let lift = |q: Vec2, w: f64| Hom::of(frame.point(q.x, q.y, 0.0), w);
        let id = seg.id;
        match seg.geom {
            SegGeom::Line { a, b } => Ok(Chain { spans: vec![vec![lift(a, 1.0), lift(b, 1.0)]], breaks: vec![0.0, 1.0], id }),
            SegGeom::Arc { center, radius, a0, a1 } => {
                let n = (((a1 - a0).abs() / (PI / 2.0)) - 1e-9).ceil().max(1.0) as usize;
                let step = (a1 - a0) / n as f64;
                let on = |a: f64, r: f64| center + v2(a.cos(), a.sin()) * r;
                let w = (step / 2.0).cos();
                let spans = (0..n)
                    .map(|i| {
                        let a = a0 + step * i as f64;
                        vec![lift(on(a, radius), 1.0), lift(on(a + step / 2.0, radius / w), w), lift(on(a + step, radius), 1.0)]
                    })
                    .collect();
                Ok(Chain { spans, breaks: (0..=n).map(|i| i as f64 / n as f64).collect(), id })
            }
            SegGeom::Spline(ref c) => {
                let lifted = c.mapped(|q| frame.point(q.x, q.y, 0.0));
                let (t0, t1) = lifted.domain();
                let pieces = lifted.bezier_spans();
                let first = pieces[0].2[0].w;
                let last = pieces[pieces.len() - 1].2.last().map_or(first, |q| q.w);
                if (first - last).abs() > 1e-9 * first {
                    return Err(Error::Unsupported("a loft can't take a rational spline whose two end weights differ yet".into()));
                }
                let mut breaks = vec![0.0];
                let mut spans = Vec::with_capacity(pieces.len());
                for (_, b, h) in pieces {
                    breaks.push((b - t0) / (t1 - t0));
                    spans.push(h.iter().map(|q| Hom { a: q.a / first, w: q.w / first }).collect());
                }
                Ok(Chain { spans, breaks, id })
            }
        }
    }

    fn start(&self) -> Vec3 {
        self.spans[0][0].point()
    }

    fn degree(&self) -> usize {
        self.spans[0].len() - 1
    }

    fn length(&self) -> f64 {
        self.spans.iter().flat_map(|h| h.windows(2)).map(|w| w[0].point().dist(w[1].point())).sum()
    }

    fn reversed(&self) -> Chain {
        let spans = self.spans.iter().rev().map(|h| h.iter().rev().copied().collect()).collect();
        let breaks = self.breaks.iter().rev().map(|b| 1.0 - b).collect();
        Chain { spans, breaks, id: self.id }
    }

    /// Its two halves by parameter.
    fn halves(&self) -> (Chain, Chain) {
        let k = self.breaks.partition_point(|b| *b <= 0.5).clamp(1, self.spans.len()) - 1;
        let (a, b) = (self.breaks[k], self.breaks[k + 1]);
        let (mut first, mut second) = (self.spans[..k].to_vec(), self.spans[k + 1..].to_vec());
        let (mut stops_a, mut stops_b): (Vec<f64>, Vec<f64>) = (self.breaks[..=k].iter().map(|x| x * 2.0).collect(), Vec::new());
        if (a - 0.5).abs() < 1e-12 {
            second.insert(0, self.spans[k].clone());
            stops_a.truncate(k + 1);
            stops_b.extend(self.breaks[k..].iter().map(|x| (x - 0.5) * 2.0));
        } else {
            let (l, r) = split_span(&self.spans[k], (0.5 - a) / (b - a));
            first.push(l);
            second.insert(0, r);
            stops_a.push(1.0);
            stops_b.push(0.0);
            stops_b.extend(self.breaks[k + 1..].iter().map(|x| (x - 0.5) * 2.0));
        }
        (Chain { spans: first, breaks: stops_a, id: self.id }, Chain { spans: second, breaks: stops_b, id: self.id })
    }

    /// Control points at this degree, cut at these breaks, with each shared end kept once.
    fn control_points(&self, degree: usize, breaks: &[f64]) -> Vec<Hom> {
        let mut out: Vec<Hom> = Vec::new();
        for (k, span) in self.spans.iter().enumerate() {
            let mut h = span.clone();
            while h.len() - 1 < degree {
                h = elevate(&h);
            }
            let (a, b) = (self.breaks[k], self.breaks[k + 1]);
            let mut from = a;
            let mut rest = h;
            let mut pieces = Vec::new();
            for &x in breaks.iter().filter(|x| **x > a + 1e-12 && **x < b - 1e-12) {
                let (l, r) = split_span(&rest, (x - from) / (b - from));
                pieces.push(l);
                rest = r;
                from = x;
            }
            pieces.push(rest);
            for piece in pieces {
                let skip = usize::from(!out.is_empty());
                out.extend_from_slice(&piece[skip..]);
            }
        }
        out
    }
}

fn wire_centroid(chains: &[Chain]) -> Vec3 {
    let mut sum = Vec3::ZERO;
    for c in chains {
        sum += c.start();
    }
    sum / chains.len() as f64
}

/// The vector area of an outline, whose direction says which way around it runs.
fn area_vector(chains: &[Chain]) -> Vec3 {
    let mut points = Vec::new();
    for c in chains {
        for h in &c.spans {
            for i in 0..8 {
                let s = i as f64 / 8.0;
                let mut work = h.clone();
                for level in (1..work.len()).rev() {
                    for k in 0..level {
                        work[k] = work[k].lerp(work[k + 1], s);
                    }
                }
                points.push(work[0].point());
            }
        }
    }
    let o = points[0];
    let mut sum = Vec3::ZERO;
    for w in 1..points.len() {
        sum += (points[w - 1] - o).cross(points[w] - o) * 0.5;
    }
    sum
}

/// A curve for an edge: a line if it's straight, since that's exact and much cheaper.
fn edge_curve(degree: usize, knots: &[f64], line: &[Hom]) -> Result<(Curve, f64, f64)> {
    let points: Vec<Vec3> = line.iter().map(|q| q.point()).collect();
    let (a, b) = (points[0], points[points.len() - 1]);
    let length = a.dist(b);
    if length > MIN_FEATURE {
        let d = (b - a) / length;
        let mut progress = 0.0f64;
        let straight = points.iter().all(|q| {
            let along = (*q - a).dot(d);
            let on = (*q - a - d * along).norm() <= 1e-9 * length && along >= progress - 1e-12;
            progress = along;
            on
        }) && line.iter().all(|q| (q.w - line[0].w).abs() <= 1e-12);
        if straight {
            return Ok((Curve::line_through(a, b), 0.0, length));
        }
    }
    let weights: Vec<f64> = line.iter().map(|q| q.w).collect();
    let spline = Spline::new(degree, knots.to_vec(), points, weights)?;
    let (t0, t1) = spline.domain();
    Ok((Curve::Spline(Arc::new(spline)), t0, t1))
}

/// Lofts through sections in order: each a profile on a frame's XY plane, each with the same number of holes.
/// The sides are named from the first section's segments, and the caps are the start and end caps.
pub fn loft(sections: &[(&Profile, Frame)], feature: u32) -> Result<Body> {
    let bad = |why: String| Err(Error::InvalidProfile(format!("loft: {why}")));
    if sections.len() < 2 {
        return bad(format!("it takes at least two sections, not {}", sections.len()));
    }
    let holes = sections[0].0.holes.len();
    if let Some((k, _)) = sections.iter().enumerate().find(|(_, s)| s.0.holes.len() != holes) {
        return bad(format!("section {k} has a different number of holes from the first"));
    }
    for (p, _) in sections {
        p.validate()?;
    }
    // Each section's wires, as chains.
    let mut wires: Vec<Vec<Vec<Chain>>> = Vec::with_capacity(sections.len());
    for (p, f) in sections {
        let mut list = Vec::with_capacity(holes + 1);
        for w in p.wires_for_faces() {
            list.push(w.segs.iter().map(|s| Chain::of(s, f)).collect::<Result<Vec<_>>>()?);
        }
        wires.push(list);
    }

    // Parameters along the loft, by distance between sections.
    let centers: Vec<Vec3> = wires.iter().map(|w| wire_centroid(&w[0])).collect();
    let mut params = vec![0.0];
    for w in centers.windows(2) {
        let gap = w[0].dist(w[1]);
        if !(gap > MIN_FEATURE) {
            return bad("two neighbouring sections are in one place".into());
        }
        params.push(params[params.len() - 1] + gap);
    }
    let total = params[params.len() - 1];
    for p in &mut params {
        *p /= total;
    }
    let (degree_v, knots_v) = interpolation_knots(&params);

    // All outlines run counter-clockwise seen from the direction of travel, so that outward is to their right.
    let direction = |k: usize| if k + 1 < centers.len() { centers[k + 1] - centers[k] } else { centers[k] - centers[k - 1] };
    for (k, section) in wires.iter_mut().enumerate() {
        if area_vector(&section[0]).dot(direction(k)) < 0.0 {
            for wire in section.iter_mut() {
                wire.reverse();
                for c in wire.iter_mut() {
                    *c = c.reversed();
                }
            }
        }
    }

    let tag = |slot| FaceTag { feature, slot };
    let mut body = Body::default();
    let (mut start_loops, mut end_loops) = (Vec::new(), Vec::new());
    for w in 0..=holes {
        // The same number of pieces in every section, by cutting the longest.
        let count = wires.iter().map(|s| s[w].len()).max().unwrap_or(0);
        for section in wires.iter_mut() {
            let wire = &mut section[w];
            while wire.len() < count {
                let longest = (0..wire.len()).max_by(|&a, &b| wire[a].length().total_cmp(&wire[b].length())).unwrap();
                let (a, b) = wire[longest].halves();
                wire[longest] = a;
                wire.insert(longest + 1, b);
            }
        }
        // Each section starts at the piece closest to where the last one started, so the sides don't twist.
        for k in 1..wires.len() {
            let (before, after) = wires.split_at_mut(k);
            let (last, this) = (&before[k - 1][w], &mut after[0][w]);
            let (ca, cb) = (wire_centroid(last), wire_centroid(this));
            let cost = |r: usize| -> f64 { (0..count).map(|i| ((this[(i + r) % count].start() - cb) - (last[i].start() - ca)).norm2()).sum() };
            let best = (0..count).min_by(|&a, &b| cost(a).total_cmp(&cost(b))).unwrap_or(0);
            this.rotate_left(best);
        }

        let first = &wires[0][w];
        let bottom_vertex: Vec<usize> = first.iter().map(|c| body.add_vertex(c.start())).collect();
        let top_vertex: Vec<usize> = wires[wires.len() - 1][w].iter().map(|c| body.add_vertex(c.start())).collect();
        // Control points of every piece, by section and then along the outline.
        struct Piece {
            degree: usize,
            knots: Vec<f64>,
            grid: Vec<Vec<Hom>>,
            id: u32,
        }
        let mut pieces = Vec::with_capacity(count);
        for i in 0..count {
            let degree = wires.iter().map(|s| s[w][i].degree()).max().unwrap();
            let mut breaks: Vec<f64> = wires.iter().flat_map(|s| s[w][i].breaks.iter().copied()).collect();
            breaks.sort_by(f64::total_cmp);
            breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            let lines: Vec<Vec<Hom>> = wires.iter().map(|s| s[w][i].control_points(degree, &breaks)).collect();
            let mut knots = vec![0.0; degree + 1];
            for b in &breaks[1..breaks.len() - 1] {
                knots.extend(vec![*b; degree]);
            }
            knots.extend(vec![1.0; degree + 1]);
            // Across the sections, a line of control points at a time.
            let mut grid = Vec::with_capacity(lines[0].len());
            for j in 0..lines[0].len() {
                let values: Vec<Hom> = lines.iter().map(|l| l[j]).collect();
                let column = if values.len() == 2 {
                    values
                } else {
                    match interpolate(&params, degree_v, &knots_v, &values) {
                        Some(c) => c,
                        None => return bad("these sections can't be interpolated".into()),
                    }
                };
                if column.iter().any(|q| !(q.w > 1e-6)) {
                    return bad("going through these sections would take a weight below zero, so they're too different".into());
                }
                grid.push(column);
            }
            pieces.push(Piece { degree, knots, grid, id: wires[0][w][i].id });
        }
        let last_row = pieces[0].grid[0].len() - 1;
        let rails: Vec<Coedge> = (0..count)
            .map(|i| -> Result<Coedge> {
                let (curve, a, b) = edge_curve(degree_v, &knots_v, &pieces[i].grid[0])?;
                Ok(body.add_edge(curve, a, b, bottom_vertex[i], top_vertex[i]))
            })
            .collect::<Result<_>>()?;
        let (mut bottom, mut top) = (Vec::with_capacity(count), Vec::with_capacity(count));
        for i in 0..count {
            let j = (i + 1) % count;
            let piece = &pieces[i];
            let at_row = |row: usize| piece.grid.iter().map(|c| c[row]).collect::<Vec<Hom>>();
            let (curve, a, b) = edge_curve(piece.degree, &piece.knots, &at_row(0))?;
            let low = body.add_edge(curve, a, b, bottom_vertex[i], bottom_vertex[j]);
            let (curve, a, b) = edge_curve(piece.degree, &piece.knots, &at_row(last_row))?;
            let high = body.add_edge(curve, a, b, top_vertex[i], top_vertex[j]);
            bottom.push(low);
            top.push(high);

            let (nu, nv) = (piece.grid.len(), last_row + 1);
            let mut points = Vec::with_capacity(nu * nv);
            let mut weights = Vec::with_capacity(nu * nv);
            for column in &piece.grid {
                for q in column {
                    points.push(q.point());
                    weights.push(q.w);
                }
            }
            let patch = Patch::new((piece.degree, degree_v), (piece.knots.clone(), knots_v.clone()), (nu, nv), points, weights)?;
            // A flat piece is a plane, which everything downstream handles better and more exactly.
            let surface = flat_frame(&patch).map_or_else(|| Surface::Free { frame: Frame::WORLD, patch: Arc::new(patch) }, Surface::Plane);
            body.add_face(surface, false, vec![vec![low, rails[j], high.flipped(), rails[i].flipped()]], tag(SLOT_SEG_BASE + piece.id));
        }
        start_loops.push(bottom.iter().rev().map(|c| c.flipped()).collect::<Vec<_>>());
        end_loops.push(top);
    }

    // The caps, with their normals along the direction of travel, then the start's flipped to face out.
    let plane_at = |k: usize| {
        let f = sections[k].1;
        if f.z.dot(direction(k)) >= 0.0 {
            f
        } else {
            Frame { origin: f.origin, x: f.x, y: -f.y, z: -f.z }
        }
    };
    body.add_face(Surface::Plane(plane_at(0)), true, start_loops, tag(SLOT_START_CAP));
    body.add_face(Surface::Plane(plane_at(sections.len() - 1)), false, end_loops, tag(SLOT_END_CAP));
    body.compute_shells();
    validate(&body).map_err(Error::InvalidBody)?;
    Ok(body)
}

/// A frame for a patch that's flat, whose normal is the patch's.
fn flat_frame(patch: &Patch) -> Option<Frame> {
    if !patch.is_polynomial() {
        return None;
    }
    let ((u0, u1), (v0, v1)) = patch.domain();
    let (origin, du, dv) = patch.derivs((u0 + u1) / 2.0, (v0 + v1) / 2.0);
    let z = du.cross(dv).normalized();
    let (lo, hi) = patch.bounds();
    let size = lo.dist(hi);
    if patch.points().iter().any(|p| (*p - origin).dot(z).abs() > 1e-10 * size) {
        return None;
    }
    let x = du.normalized();
    Some(Frame { origin, x, y: z.cross(x), z })
}
