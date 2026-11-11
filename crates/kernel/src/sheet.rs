//! Sheet bodies: open surfaces that are bodies of their own, with no inside. See docs/SHEET_BODIES_DESIGN.md.
//!
//! A sheet takes its geometry from what solids are made of: the faces of an extrude or a revolve, without its caps. So a
//! surface here is exactly what the same curve would make on a solid, by the same code.

use crate::boolean::simplify::simplify;
use crate::build::{extrude_faces, revolve_faces, SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::tess::{open_only_at_boundary, tessellate, TessParams};
use crate::topo::{Body, Coedge, Edge};
use crate::validate::validate;
use std::collections::HashMap;

/// The id of the segment that closes an open curve so it can go through a solid's code. Its face is thrown away.
const CHORD: u32 = u32::MAX - 16;

fn refuse(what: String) -> Error {
    Error::InvalidInput(format!("making a sheet: {what}"))
}

/// Checks a finished sheet: valid, and with its mesh torn nowhere but its boundary.
fn finished(body: Body) -> Result<Body> {
    validate(&body).map_err(|why| Error::NearDegenerate(format!("the sheet didn't come out valid: {}", why.join("; "))))?;
    let mesh = tessellate(&body, &TessParams::draft_for_size(body.aabb().diagonal()))?;
    if !open_only_at_boundary(&body, &mesh) {
        return Err(Error::NearDegenerate("the sheet's mesh is torn somewhere other than its boundary".to_string()));
    }
    Ok(body)
}

/// Gives a body the vertices its faces' fans call for, keeping only what the faces use. Going from one coedge of a loop to the
/// next stays in one fan, so does going across an edge, and each fan gets a vertex of its own. So two parts that only meet at a
/// point, as they can once what joined them has gone, don't pinch. An edge's `v0` and `v1` only say where its ends are.
fn with_vertices_by_fan(mut body: Body) -> Body {
    let mut new_edge: HashMap<usize, usize> = HashMap::new();
    let mut edges: Vec<Edge> = Vec::new();
    for c in body.faces.iter_mut().flat_map(|f| f.loops.iter_mut()).flatten() {
        c.edge = *new_edge.entry(c.edge).or_insert_with(|| {
            edges.push(body.edges[c.edge].clone());
            edges.len() - 1
        });
    }
    fn root(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let mut parent: Vec<usize> = (0..edges.len() * 2).collect();
    for lp in body.faces.iter().flat_map(|f| f.loops.iter()) {
        for i in 0..lp.len() {
            let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
            let arriving = a.edge * 2 + usize::from(!a.reversed);
            let leaving = b.edge * 2 + usize::from(b.reversed);
            let (x, y) = (root(&mut parent, arriving), root(&mut parent, leaving));
            parent[x] = y;
        }
    }
    let mut vertices = Vec::new();
    let mut of_fan: HashMap<usize, usize> = HashMap::new();
    for (e, edge) in edges.iter_mut().enumerate() {
        for (node, end) in [(e * 2, &mut edge.v0), (e * 2 + 1, &mut edge.v1)] {
            let r = root(&mut parent, node);
            let place = body.vertices[*end];
            *end = *of_fan.entry(r).or_insert_with(|| {
                vertices.push(place);
                vertices.len() - 1
            });
        }
    }
    body.edges = edges;
    body.vertices = vertices;
    body.compute_shells();
    body
}

/// Turns a shell's faces to face the other way.
fn flip_faces(body: &mut Body, faces: &[usize]) {
    for &f in faces {
        let face = &mut body.faces[f];
        face.flipped = !face.flipped;
        for lp in &mut face.loops {
            lp.reverse();
            for c in lp.iter_mut() {
                c.reversed = !c.reversed;
            }
        }
    }
}

/// A copy of some of a body's faces, as a sheet. The body stays as it is. Each face keeps its name and the way it faces.
pub fn sheet_from_faces(body: &Body, faces: &[usize]) -> Result<Body> {
    if faces.is_empty() {
        return Err(refuse("it takes at least one face".to_string()));
    }
    if let Some(bad) = faces.iter().find(|f| **f >= body.faces.len()) {
        return Err(refuse(format!("there is no face {bad}")));
    }
    let mut list = faces.to_vec();
    list.sort_unstable();
    list.dedup();
    let out = Body {
        vertices: body.vertices.clone(),
        edges: body.edges.clone(),
        faces: list.iter().map(|f| body.faces[*f].clone()).collect(),
        shells: Vec::new(),
        tolerance: body.tolerance,
        sheet: true,
    };
    finished(with_vertices_by_fan(out))
}

/// The sides of what a sweep made, without its caps: a tube, from something extruded or swept.
pub fn sides_of(solid: &Body) -> Result<Body> {
    let sides: Vec<usize> = (0..solid.faces.len()).filter(|&f| solid.faces[f].tag.slot >= SLOT_SEG_BASE).collect();
    if sides.is_empty() {
        return Err(refuse("this has no sides, only ends".to_string()));
    }
    sheet_from_faces(solid, &sides)
}

/// A flat sheet filling a sketch's region, facing along the frame's z. It's named as an end cap.
pub fn fill(profile: &Profile, frame: &Frame, feature: u32) -> Result<Body> {
    profile.validate()?;
    let block = extrude_faces(profile, frame, 1.0, feature)?;
    let start = (0..block.faces.len()).find(|&f| block.faces[f].tag.slot == SLOT_START_CAP).ok_or_else(|| Error::Internal("an extrude has no start cap".to_string()))?;
    let mut out = Body { vertices: block.vertices.clone(), edges: block.edges.clone(), faces: vec![block.faces[start].clone()], sheet: true, ..Body::default() };
    flip_faces(&mut out, &[0]);
    out.faces[0].tag.slot = SLOT_END_CAP;
    finished(with_vertices_by_fan(out))
}

/// An open chain as a closed wire, for a solid's code to work through, with a line back to its start that is thrown away.
/// The chain goes in as it is, with no turning round to wind one way, so a sheet faces to the right of the way it was drawn.
fn closed_by_chord(path: &Wire) -> Result<Wire> {
    path.check_open()?;
    let mut segs = path.segs.clone();
    if !path.is_closed() {
        let (a, b) = (segs[segs.len() - 1].end(), segs[0].start());
        segs.push(Seg { geom: SegGeom::Line { a, b }, id: CHORD });
    }
    Ok(Wire { segs })
}

fn without_chord_and_caps(made: &Body) -> Result<Body> {
    let keep: Vec<usize> = (0..made.faces.len()).filter(|&f| made.faces[f].tag.slot >= SLOT_SEG_BASE && made.faces[f].tag.slot != SLOT_SEG_BASE + CHORD).collect();
    if keep.is_empty() {
        return Err(refuse("this curve sweeps nothing, as it lies along the axis".to_string()));
    }
    sheet_from_faces(made, &keep)
}

/// A sheet swept by a chain of segments along the frame's z, facing to the right of the way the chain runs. It has a face
/// for each segment, named for it as on a solid, and it can close into a tube.
pub fn extrude_open(path: &Wire, frame: &Frame, height: f64, feature: u32) -> Result<Body> {
    let wire = closed_by_chord(path)?;
    let made = extrude_faces(&Profile { outer: wire, holes: Vec::new() }, frame, height, feature)?;
    without_chord_and_caps(&made)
}

/// A sheet swept by a chain of segments around the frame's z, with x the distance from the axis and y the height along it, as
/// a solid's revolve has them. A chain with both ends on the axis and a whole turn makes a closed sheet.
pub fn revolve_open(path: &Wire, frame: &Frame, angle: f64, feature: u32) -> Result<Body> {
    let wire = closed_by_chord(path)?;
    let made = revolve_faces(&Profile { outer: wire, holes: Vec::new() }, frame, angle, feature)?;
    without_chord_and_caps(&made)
}

// ----- Sewing -----

/// Points on a curve piece that say whether another one is the same.
fn probes(curve: &Curve, ta: f64, tb: f64) -> [Vec3; 3] {
    [0.25, 0.5, 0.8].map(|f| curve.eval(ta + (tb - ta) * f))
}

/// Joins sheets along the boundary edges they have in common, wherever those run together to within `tolerance`, and
/// turns over whichever it has to so that they all face one way. What's left is a sheet, or a solid if nothing is left open.
/// A boundary running along part of another one is cut there to fit. It doesn't move or extend anything.
pub fn sew(sheets: &[&Body], tolerance: f64) -> Result<Body> {
    if sheets.is_empty() {
        return Err(refuse("sewing takes at least one sheet".to_string()));
    }
    if !(tolerance >= TOL && tolerance <= 1.0) {
        return Err(refuse(format!("a sewing tolerance of {tolerance} isn't sensible: it can be from {TOL} to 1")));
    }
    if sheets.iter().any(|s| !s.sheet) {
        return Err(refuse("sewing joins sheets, and one of these is a solid".to_string()));
    }
    let mut all = Body::side_by_side(sheets);
    all.sheet = true;
    all.compute_shells();
    let shell_count = all.shells.len();
    let mut shell_of = vec![0usize; all.faces.len()];
    for (s, shell) in all.shells.iter().enumerate() {
        for &f in shell {
            shell_of[f] = s;
        }
    }

    // One point for each place that boundaries meet.
    let open = all.boundary_edges();
    let mut places: Vec<Vec3> = Vec::new();
    let mut gap = 0.0f64;
    let mut ends: HashMap<usize, (usize, usize)> = HashMap::new();
    for &e in &open {
        let edge = &all.edges[e];
        let mut place_of = |p: Vec3| -> usize {
            match places.iter().position(|q| q.dist(p) <= tolerance) {
                Some(i) => {
                    gap = gap.max(places[i].dist(p));
                    i
                }
                None => {
                    places.push(p);
                    places.len() - 1
                }
            }
        };
        let (a, b) = (place_of(all.vertices[edge.v0]), place_of(all.vertices[edge.v1]));
        if a == b {
            return Err(refuse(format!("a boundary edge is no longer than the sewing tolerance of {tolerance}")));
        }
        ends.insert(e, (a, b));
    }

    // Each boundary edge cut wherever another's end lands on it, into pieces in its own direction.
    struct Piece {
        curve: Curve,
        t0: f64,
        t1: f64,
        a: usize,
        b: usize,
        /// The boundary edge it's part of.
        edge: usize,
    }
    let mut pieces: Vec<Piece> = Vec::new();
    let mut cut: HashMap<usize, Vec<usize>> = HashMap::new();
    for &e in &open {
        let edge = &all.edges[e];
        let (first, last) = ends[&e];
        let mut stops: Vec<(f64, usize)> = Vec::new();
        for (i, q) in places.iter().enumerate() {
            if i != first && i != last && edge.curve.could_be_within(*q, edge.t0, edge.t1, tolerance) {
                let (t, d) = edge.curve.closest(*q, edge.t0, edge.t1);
                if d <= tolerance {
                    gap = gap.max(d);
                    stops.push((t, i));
                }
            }
        }
        stops.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut list = Vec::with_capacity(stops.len() + 1);
        let (mut t, mut at) = (edge.t0, first);
        for (u, place) in stops {
            pieces.push(Piece { curve: edge.curve.clone(), t0: t, t1: u, a: at, b: place, edge: e });
            list.push(pieces.len() - 1);
            (t, at) = (u, place);
        }
        pieces.push(Piece { curve: edge.curve.clone(), t0: t, t1: edge.t1, a: at, b: last, edge: e });
        list.push(pieces.len() - 1);
        cut.insert(e, list);
    }

    // Which pieces run together. Each has at most one partner, as an edge has at most two faces.
    let mut partner: Vec<Option<usize>> = vec![None; pieces.len()];
    let mut by_ends: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (i, p) in pieces.iter().enumerate() {
        by_ends.entry((p.a.min(p.b), p.a.max(p.b))).or_default().push(i);
    }
    let mut keys: Vec<(usize, usize)> = by_ends.keys().copied().collect();
    keys.sort_unstable();
    for key in keys {
        let list = &by_ends[&key];
        for (k, &i) in list.iter().enumerate() {
            let p = &pieces[i];
            let same: Vec<usize> = list[k + 1..]
                .iter()
                .copied()
                .filter(|&j| {
                    let q = &pieces[j];
                    let reach = probes(&p.curve, p.t0, p.t1).iter().map(|m| q.curve.closest(*m, q.t0, q.t1).1).fold(0.0, f64::max);
                    reach <= tolerance
                })
                .collect();
            if same.is_empty() || partner[i].is_some() {
                continue;
            }
            if same.len() > 1 {
                let m = pieces[i].curve.eval((pieces[i].t0 + pieces[i].t1) / 2.0);
                return Err(refuse(format!("{} sheets meet along one edge near ({:.4}, {:.4}, {:.4}), and an edge has at most two faces", same.len() + 1, m.x, m.y, m.z)));
            }
            let j = same[0];
            partner[i] = Some(j);
            partner[j] = Some(i);
            let q = &pieces[j];
            let reach = probes(&p.curve, p.t0, p.t1).iter().map(|m| q.curve.closest(*m, q.t0, q.t1).1).fold(0.0, f64::max);
            gap = gap.max(reach);
        }
    }
    if partner.iter().all(Option::is_none) {
        return Err(refuse("no boundary runs along another, so there's nothing to sew".to_string()));
    }

    // Who uses each piece, and which way round: the face, and whether its loop runs against the edge it came from.
    let uses = all.edge_uses();
    let user = |piece: usize| -> (usize, bool) {
        let e = pieces[piece].edge;
        (uses[e][0].0, uses[e][0].3)
    };
    // Facing one way: every piece and its partner have to run opposite ways round their loops. Whichever shell is reached second
    // turns over if they don't, and if it's already been decided otherwise, this surface has only one side.
    let mut turned: Vec<Option<bool>> = vec![None; shell_count];
    for start in 0..shell_count {
        if turned[start].is_some() {
            continue;
        }
        turned[start] = Some(false);
        let mut stack = vec![start];
        while let Some(s) = stack.pop() {
            for i in 0..pieces.len() {
                let Some(j) = partner[i] else { continue };
                let ((fi, ri), (fj, rj)) = (user(i), user(j));
                if shell_of[fi] != s {
                    continue;
                }
                let same_dir = pieces[i].curve.eval((pieces[i].t0 + pieces[i].t1) / 2.0);
                let along_i = pieces[i].curve.derivative((pieces[i].t0 + pieces[i].t1) / 2.0);
                let (t, _) = pieces[j].curve.closest(same_dir, pieces[j].t0, pieces[j].t1);
                let along_j = pieces[j].curve.derivative(t);
                // Which way each loop goes along the geometry.
                let way_i = if ri { -along_i } else { along_i };
                let way_j = if rj { -along_j } else { along_j };
                let opposite = way_i.dot(way_j) < 0.0;
                let other = shell_of[fj];
                // Running opposite ways, the two go as one. Otherwise one of them turns over.
                let should = turned[s].unwrap_or(false) == opposite;
                match turned[other] {
                    None => {
                        turned[other] = Some(should);
                        stack.push(other);
                    }
                    Some(b) if b != should => {
                        return Err(refuse(format!("sewing these would make a surface with only one side, near ({:.4}, {:.4}, {:.4})", same_dir.x, same_dir.y, same_dir.z)));
                    }
                    Some(_) => {}
                }
            }
        }
    }
    for s in 0..shell_count {
        if turned[s] == Some(true) {
            let list = all.shells[s].clone();
            flip_faces(&mut all, &list);
        }
    }

    // The edges of the result: all that weren't on the boundary, then a piece for each boundary piece or pair.
    let mut out = Body { tolerance: all.tolerance, sheet: true, ..Body::default() };
    out.vertices = all.vertices.clone();
    let place_base = out.vertices.len();
    out.vertices.extend_from_slice(&places);
    // A vertex on the boundary is its place from here on.
    let mut vertex_at: HashMap<usize, usize> = HashMap::new();
    for &e in &open {
        vertex_at.insert(all.edges[e].v0, place_base + ends[&e].0);
        vertex_at.insert(all.edges[e].v1, place_base + ends[&e].1);
    }
    let mut new_edge: Vec<Option<usize>> = vec![None; all.edges.len()];
    for (e, edge) in all.edges.iter().enumerate() {
        if uses[e].len() == 2 {
            let moved = Edge { curve: edge.curve.clone(), t0: edge.t0, t1: edge.t1, v0: *vertex_at.get(&edge.v0).unwrap_or(&edge.v0), v1: *vertex_at.get(&edge.v1).unwrap_or(&edge.v1) };
            out.edges.push(moved);
            new_edge[e] = Some(out.edges.len() - 1);
        }
    }
    // For a piece: its edge in `out`, and whether that edge runs against the piece.
    let mut piece_edge: Vec<Option<(usize, bool)>> = vec![None; pieces.len()];
    for i in 0..pieces.len() {
        if piece_edge[i].is_some() {
            continue;
        }
        let p = &pieces[i];
        out.edges.push(Edge { curve: p.curve.clone(), t0: p.t0, t1: p.t1, v0: place_base + p.a, v1: place_base + p.b });
        let id = out.edges.len() - 1;
        piece_edge[i] = Some((id, false));
        if let Some(j) = partner[i] {
            let q = &pieces[j];
            // The partner runs from the same place, or the other way round.
            piece_edge[j] = Some((id, q.a != p.a));
        }
    }
    for mut face in all.faces.drain(..) {
        for lp in &mut face.loops {
            let mut now = Vec::with_capacity(lp.len());
            for c in lp.iter() {
                if let Some(id) = new_edge[c.edge] {
                    now.push(Coedge { edge: id, reversed: c.reversed });
                    continue;
                }
                let list = &cut[&c.edge];
                let run: Vec<usize> = if c.reversed { list.iter().rev().copied().collect() } else { list.clone() };
                for piece in run {
                    let (id, against) = piece_edge[piece].ok_or_else(|| Error::Internal("a piece of a sewn edge has no edge".to_string()))?;
                    now.push(Coedge { edge: id, reversed: c.reversed != against });
                }
            }
            *lp = now;
        }
        out.faces.push(face);
    }
    if gap > 5.0 * TOL {
        out.tolerance = out.tolerance.max(gap);
    }
    let out = simplify(with_vertices_by_fan(out))?;
    if !out.boundary_edges().is_empty() {
        return finished(out);
    }
    solid_from_closed(out)
}

/// A sheet with nothing open as a solid, with its outsides out, and all that a solid has to pass.
fn solid_from_closed(mut body: Body) -> Result<Body> {
    body.sheet = false;
    validate(&body).map_err(|why| Error::NearDegenerate(format!("the sewn sheets close, but don't make a valid solid: {}", why.join("; "))))?;
    for s in 0..body.shells.len() {
        if crate::mass::shell_volume(&body, s) < 0.0 {
            let list = body.shells[s].clone();
            flip_faces(&mut body, &list);
        }
    }
    validate(&body).map_err(|why| Error::NearDegenerate(format!("the sewn sheets close, but don't make a valid solid: {}", why.join("; "))))?;
    let mesh = tessellate(&body, &TessParams::draft_for_size(body.aabb().diagonal()))?;
    if mesh.open_edge_count() != 0 {
        return Err(Error::NearDegenerate("the sewn sheets close, but their mesh doesn't".to_string()));
    }
    Ok(body)
}
