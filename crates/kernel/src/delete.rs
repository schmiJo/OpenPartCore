//! Deleting faces, and healing the body around the gap.
//!
//! A face is gone, so whatever was next to it has to close over the place. This handles the shapes of gap where what
//! happens is clear, and refuses the rest by name:
//!
//! * **An island,** where the faces' whole border is inner loops of the faces around them: a hole, pocket or boss. Those
//!   loops just go, and the surrounding faces carry on across.
//! * **A whole shell:** a void inside the body, or one separate piece of several.
//! * **A strip,** with two long sides on faces that can grow until they meet and two short ends: a fillet, a chamfer, a
//!   rabbet along an edge, and chains of those, open or closed. The long sides become one new edge, and each short end
//!   shrinks to a new vertex where the surfaces meet. Corner patches between chains shrink to a vertex as well.
//! * **A corner,** with three sides: a cut-off corner or a corner notch. All of it shrinks to one vertex.
//! * **A slot,** with one plane on both of its long sides: they join up into one face across the gap.
//!
//! Faces only grow here, along surfaces that already exist, so everything new is as exact as they are. What this doesn't
//! check is whether a face that grew now runs into some other, distant part of the body.

use crate::boolean::simplify::simplify;
use crate::boolean::ssi;
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::surface::Surface;
use crate::tess::{tessellate, TessParams};
use crate::topo::{Body, Coedge, Edge};
use crate::tweak::{curve_between, fitted, point_along, point_on_surfaces};
use crate::validate::validate;
use std::collections::{HashMap, HashSet};

type Uses = Vec<Vec<(usize, usize, usize, bool)>>;

fn refuse(what: String) -> Error {
    Error::Unsupported(format!("deleting these faces: {what}"))
}

/// A stretch of a surviving face's loop that borders the faces going.
#[derive(Clone, Debug)]
struct Run {
    owner: usize,
    lp: usize,
    /// Where it starts in the loop.
    start: usize,
    coedges: Vec<Coedge>,
    from: usize,
    to: usize,
    /// The group of faces going that it borders.
    group: usize,
}

impl Run {
    /// A point along it, from 0 to 1 in the owner's direction.
    fn point(&self, body: &Body, s: f64) -> Vec3 {
        let n = self.coedges.len() as f64;
        let i = ((s * n).floor() as usize).min(self.coedges.len() - 1);
        body.coedge_point(self.coedges[i], s * n - i as f64)
    }

    fn length(&self, body: &Body) -> f64 {
        (0..16).map(|i| self.point(body, i as f64 / 16.0).dist(self.point(body, (i + 1) as f64 / 16.0))).sum()
    }

    fn vertices(&self, body: &Body) -> Vec<usize> {
        let mut list: Vec<usize> = self.coedges.iter().map(|c| body.start_vertex(*c)).collect();
        list.push(self.to);
        list
    }
}

fn other_face(uses: &Uses, e: usize, f: usize) -> Option<usize> {
    uses[e].iter().map(|u| u.0).find(|g| *g != f)
}

/// Every stretch of a surviving face that borders a group, the groups being given face by face.
fn border_runs(body: &Body, uses: &Uses, group_of: &[Option<usize>]) -> Vec<Run> {
    let mut out = Vec::new();
    for (f, face) in body.faces.iter().enumerate() {
        if group_of[f].is_some() {
            continue;
        }
        for (l, lp) in face.loops.iter().enumerate() {
            let n = lp.len();
            let key = |i: usize| other_face(uses, lp[i].edge, f).and_then(|g| group_of[g]);
            let keys: Vec<Option<usize>> = (0..n).map(key).collect();
            if keys.iter().all(|k| k.is_none()) {
                continue;
            }
            let starts: Vec<usize> = (0..n).filter(|&i| keys[i].is_some() && keys[i] != keys[(i + n - 1) % n]).collect();
            let starts = if starts.is_empty() { vec![0] } else { starts };
            for &start in &starts {
                let group = keys[start].unwrap();
                let mut coedges = vec![lp[start]];
                let mut i = (start + 1) % n;
                while i != start && keys[i] == Some(group) {
                    coedges.push(lp[i]);
                    i = (i + 1) % n;
                }
                let (from, to) = (body.start_vertex(coedges[0]), body.end_vertex(coedges[coedges.len() - 1]));
                out.push(Run { owner: f, lp: l, start, coedges, from, to, group });
            }
        }
    }
    out
}

/// What happens to the faces around a gap. Every run that borders it is in one of these lists.
#[derive(Default, Clone)]
struct Plan {
    /// Shrink to nothing.
    collapse: Vec<Run>,
    /// A long side on one face, and the other long side, which can be on several in turn, going the way the face being
    /// deleted runs. There's one new edge for each of those.
    long: Vec<(Run, Vec<Run>)>,
    /// Vertices that become one, beyond those on the runs that collapse.
    join: Vec<usize>,
    /// Inner loops that go.
    drop_loops: Vec<(usize, usize)>,
    /// A slot's two long sides, on one plane, and the short ones, each of which becomes a new edge.
    slot: Option<[Run; 4]>,
}

fn find(parent: &mut [usize], x: usize) -> usize {
    let mut r = x;
    while parent[r] != r {
        parent[r] = parent[parent[r]];
        r = parent[r];
    }
    r
}

/// The new curve for an edge whose ends have moved along it, keeping the curve if it reaches them.
fn extended(edge: &Edge, s: &Surface, t: &Surface, a: Vec3, b: Vec3) -> Result<(Curve, f64, f64)> {
    let middle = edge.eval_mid();
    if let Some(found) = fitted(&edge.curve, a, b, middle) {
        return Ok(found);
    }
    // It doesn't reach, so what's there gets followed, and carried on to the new ends.
    let (start, end) = (edge.curve.eval(edge.t0), edge.curve.eval(edge.t1));
    let mut samples = Vec::new();
    let reach = |from: Vec3, to: Vec3, out: &mut Vec<Vec3>| {
        if from.dist(to) > 1e-6 {
            for k in 1..4 {
                out.push(from + (to - from) * (k as f64 / 4.0));
            }
        }
    };
    reach(a, start, &mut samples);
    let n = 24;
    samples.extend((0..=n).map(|i| edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64)));
    reach(end, b, &mut samples);
    samples.retain(|p| p.dist(a) > 1e-6 && p.dist(b) > 1e-6);
    curve_between(s, t, a, b, middle, matches!(edge.curve, Curve::Line { .. }), &samples)
}

/// Carries out a plan. Gives the new body, and for each of its faces the face it came from.
fn apply(body: &Body, gone: &[bool], plan: &Plan) -> Result<(Body, Vec<usize>)> {
    let size = body.aabb().diagonal().max(1.0);
    let mut parent: Vec<usize> = (0..body.vertices.len()).collect();
    for run in &plan.collapse {
        let list = run.vertices(body);
        for w in list.windows(2) {
            let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
            parent[a] = b;
        }
    }
    for w in plan.join.chunks(2) {
        let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
        parent[a] = b;
    }
    // What replaces each run, by where it starts in its owner's loop, and which places in a loop go.
    let mut replaced: HashMap<(usize, usize, usize), Vec<Coedge>> = HashMap::new();
    let mut dropped: HashSet<(usize, usize, usize)> = HashSet::new();
    let note = |run: &Run, dropped: &mut HashSet<(usize, usize, usize)>| {
        let n = body.faces[run.owner].loops[run.lp].len();
        for k in 0..run.coedges.len() {
            dropped.insert((run.owner, run.lp, (run.start + k) % n));
        }
    };
    for run in &plan.collapse {
        note(run, &mut dropped);
        replaced.insert((run.owner, run.lp, run.start), Vec::new());
    }
    // The edges as they'll be, with the new ones after them.
    let mut edges: Vec<Edge> = body.edges.iter().map(|e| Edge { curve: e.curve.clone(), t0: e.t0, t1: e.t1, v0: find(&mut parent, e.v0), v1: find(&mut parent, e.v1) }).collect();
    let old_edges = edges.len();
    // Each new edge: the two surfaces, and where it's expected to run.
    struct Fresh {
        s: Surface,
        t: Surface,
        hints: Vec<Vec3>,
        chord: Vec3,
    }
    let mut fresh: Vec<Fresh> = Vec::new();
    let mut forced: Vec<usize> = Vec::new();
    for (a, others) in &plan.long {
        note(a, &mut dropped);
        for b in others {
            note(b, &mut dropped);
        }
        // Where the new edges run from and to, along the first long side.
        let mut stops = vec![find(&mut parent, a.from)];
        for b in &others[..others.len() - 1] {
            stops.push(find(&mut parent, b.from));
        }
        stops.push(find(&mut parent, a.to));
        forced.extend_from_slice(&stops[1..stops.len() - 1]);
        if stops.windows(2).any(|w| w[0] == w[1]) {
            return Err(refuse("a face's two long sides would make an edge with no length".into()));
        }
        for (k, b) in others.iter().enumerate() {
            if find(&mut parent, b.to) != stops[k] || find(&mut parent, b.from) != stops[k + 1] {
                return Err(refuse("its long sides don't line up end to end".into()));
            }
        }
        let lengths: Vec<f64> = others.iter().map(|b| b.length(body)).collect();
        let total: f64 = lengths.iter().sum();
        let mut done = 0.0;
        let mut pieces = Vec::with_capacity(others.len());
        for (k, b) in others.iter().enumerate() {
            let (sa, sb) = (&body.faces[a.owner].surface, &body.faces[b.owner].surface);
            if a.owner == b.owner || ssi::coincident(sa, sb, size).is_some() {
                return Err(refuse("both long sides lie on one surface, so nothing would grow to close the gap".into()));
            }
            let n = 24;
            let along_a = |t: f64| (done + lengths[k] * t) / total;
            let hints: Vec<Vec3> = (1..n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    (a.point(body, along_a(t)) + b.point(body, 1.0 - t)) * 0.5
                })
                .collect();
            let chord = a.point(body, along_a(1.0)) - a.point(body, along_a(0.0));
            edges.push(Edge { curve: Curve::line_through(body.vertices[a.from], body.vertices[a.from] + Vec3::X), t0: 0.0, t1: 1.0, v0: stops[k], v1: stops[k + 1] });
            let id = edges.len() - 1;
            fresh.push(Fresh { s: sa.clone(), t: sb.clone(), hints, chord });
            pieces.push(Coedge { edge: id, reversed: false });
            replaced.insert((b.owner, b.lp, b.start), vec![Coedge { edge: id, reversed: true }]);
            done += lengths[k];
        }
        replaced.insert((a.owner, a.lp, a.start), pieces);
    }
    let mut merged_away = None;
    if let Some([ra, rc, rb, rd]) = &plan.slot {
        for run in [ra, rc, rb, rd] {
            note(run, &mut dropped);
        }
        let new_edge_for = |run: &Run, onto: &Surface, edges: &mut Vec<Edge>, fresh: &mut Vec<Fresh>, parent: &mut [usize]| -> usize {
            let (v0, v1) = (find(parent, run.from), find(parent, run.to));
            edges.push(Edge { curve: Curve::line_through(body.vertices[run.from], body.vertices[run.from] + Vec3::X), t0: 0.0, t1: 1.0, v0, v1 });
            let (from, to) = (body.vertices[run.from], body.vertices[run.to]);
            let hints = (1..24).map(|i| from + (to - from) * (i as f64 / 24.0)).collect();
            fresh.push(Fresh { s: body.faces[run.owner].surface.clone(), t: onto.clone(), hints, chord: to - from });
            edges.len() - 1
        };
        let plane = body.faces[ra.owner].surface.clone();
        let ec = new_edge_for(rc, &plane, &mut edges, &mut fresh, &mut parent);
        let ed = new_edge_for(rd, &plane, &mut edges, &mut fresh, &mut parent);
        replaced.insert((rc.owner, rc.lp, rc.start), vec![Coedge { edge: ec, reversed: false }]);
        replaced.insert((rd.owner, rd.lp, rd.start), vec![Coedge { edge: ed, reversed: false }]);
        // Around the first plane to the gap, across it, all around the second, and back across.
        let other = &body.faces[rb.owner].loops[rb.lp];
        let n = other.len();
        let mut joined = vec![Coedge { edge: ed, reversed: true }];
        for k in rb.coedges.len()..n {
            joined.push(other[(rb.start + k) % n]);
        }
        joined.push(Coedge { edge: ec, reversed: true });
        replaced.insert((ra.owner, ra.lp, ra.start), joined);
        merged_away = Some((rb.owner, rb.lp, ra.owner));
    }

    // Faces, with their loops rewritten.
    let mut loops_of: Vec<(usize, Vec<Vec<Coedge>>)> = Vec::new();
    for (f, face) in body.faces.iter().enumerate() {
        if gone[f] || merged_away.is_some_and(|m| m.0 == f) {
            continue;
        }
        let mut loops = Vec::with_capacity(face.loops.len());
        for (l, lp) in face.loops.iter().enumerate() {
            if plan.drop_loops.contains(&(f, l)) {
                continue;
            }
            let mut out = Vec::with_capacity(lp.len());
            for (i, c) in lp.iter().enumerate() {
                if let Some(list) = replaced.get(&(f, l, i)) {
                    out.extend_from_slice(list);
                } else if !dropped.contains(&(f, l, i)) {
                    out.push(*c);
                }
            }
            if out.len() < 2 {
                if l == 0 {
                    return Err(refuse("a neighbouring face would have nothing left of its boundary".into()));
                }
                continue;
            }
            loops.push(out);
        }
        if let Some((from, lp, into)) = merged_away {
            if into == f {
                for (l, inner) in body.faces[from].loops.iter().enumerate() {
                    if l != lp {
                        loops.push(inner.clone());
                    }
                }
            }
        }
        loops_of.push((f, loops));
    }

    // Where the vertices that moved are now.
    let mut members: HashMap<usize, Vec<usize>> = HashMap::new();
    for v in 0..body.vertices.len() {
        members.entry(find(&mut parent, v)).or_default().push(v);
    }
    let mut vertices = body.vertices.clone();
    let mut edge_faces: HashMap<usize, Vec<usize>> = HashMap::new();
    for (f, loops) in &loops_of {
        for c in loops.iter().flatten() {
            edge_faces.entry(c.edge).or_default().push(*f);
        }
    }
    if let Some((e, _)) = edge_faces.iter().find(|(_, list)| list.len() != 2) {
        return Err(refuse(format!("edge {e} wouldn't have two faces after this")));
    }
    let mut at_vertex: HashMap<usize, Vec<usize>> = HashMap::new();
    for &e in edge_faces.keys() {
        at_vertex.entry(edges[e].v0).or_default().push(e);
        at_vertex.entry(edges[e].v1).or_default().push(e);
    }
    let tangent_edge = |e: usize, edges: &[Edge]| -> bool {
        if e >= old_edges {
            return false;
        }
        let list = &edge_faces[&e];
        let (f, g) = (&body.faces[list[0]], &body.faces[list[1]]);
        let p = edges[e].eval_mid();
        f.surface.normal(p).cross(g.surface.normal(p)).norm() < 1e-7
    };
    let mut moved = vec![false; body.vertices.len()];
    let limit = (10.0 * TOL).max(2.0 * body.tolerance);
    let mut classes: Vec<usize> = at_vertex.keys().copied().collect();
    classes.sort_unstable();
    for &v in &classes {
        let list = &members[&v];
        if list.len() < 2 && !forced.contains(&v) {
            continue;
        }
        moved[v] = true;
        let start = list.iter().fold(Vec3::ZERO, |sum, m| sum + body.vertices[*m]) * (1.0 / list.len() as f64);
        let mut faces: Vec<usize> = at_vertex[&v].iter().flat_map(|e| edge_faces[e].iter().copied()).collect();
        faces.sort_unstable();
        faces.dedup();
        let surfaces: Vec<Surface> = faces.iter().map(|f| body.faces[*f].surface.clone()).collect();
        // A smooth edge or a seam fixes the vertex to its curve. Surfaces that are tangent to each other can't.
        let along = at_vertex[&v].iter().copied().find(|e| tangent_edge(*e, &edges) && matches!(edges[*e].curve, Curve::Line { .. } | Curve::Circle { .. }));
        let p = match along {
            Some(e) => {
                let pair = &edge_faces[&e];
                let others: Vec<Surface> = faces.iter().filter(|f| !pair.contains(f)).map(|f| body.faces[*f].surface.clone()).collect();
                point_along(&edges[e].curve, start, &others)
            }
            None => point_on_surfaces(&surfaces, start).ok_or_else(|| refuse("the faces around it don't meet at a point".into()))?,
        };
        if surfaces.iter().any(|s| s.sdf(p).abs() > limit) {
            return Err(refuse("the faces around it don't meet at a point".into()));
        }
        if p.dist(start) > 3.0 * size {
            return Err(refuse("the faces around it would only meet far outside the part".into()));
        }
        vertices[v] = p;
    }

    // The edges that end at those vertices, and the new ones.
    let mut live: Vec<usize> = edge_faces.keys().copied().collect();
    live.sort_unstable();
    for &e in &live {
        let (v0, v1) = (edges[e].v0, edges[e].v1);
        if v0 == v1 {
            return Err(refuse("an edge would shrink to nothing".into()));
        }
        let (a, b) = (vertices[v0], vertices[v1]);
        if e >= old_edges {
            let f = &fresh[e - old_edges];
            let middle = f.hints[f.hints.len() / 2];
            let (curve, t0, t1) = curve_between(&f.s, &f.t, a, b, middle, true, &f.hints)?;
            if (curve.eval(t1) - curve.eval(t0)).dot(f.chord) <= 0.0 {
                return Err(refuse("the faces meet on the wrong side of it, so the part would turn inside out".into()));
            }
            edges[e] = Edge { curve, t0, t1, v0, v1 };
        } else if moved[v0] || moved[v1] {
            let list = &edge_faces[&e];
            let (s, t) = (&body.faces[list[0]].surface, &body.faces[list[1]].surface);
            let old = &body.edges[e];
            let (curve, t0, t1) = extended(old, s, t, a, b)?;
            if matches!(old.curve, Curve::Line { .. }) && (old.curve.eval(old.t1) - old.curve.eval(old.t0)).dot(curve.eval(t1) - curve.eval(t0)) <= 0.0 {
                return Err(refuse("a neighbouring edge would turn round, so the part would turn inside out".into()));
            }
            edges[e] = Edge { curve, t0, t1, v0, v1 };
        }
    }

    // Only what's still used.
    let mut new_vertex: HashMap<usize, usize> = HashMap::new();
    let mut new_edge: HashMap<usize, usize> = HashMap::new();
    let mut out = Body { tolerance: body.tolerance, ..Body::default() };
    for &e in &live {
        let edge = &edges[e];
        let mut map = |v: usize| *new_vertex.entry(v).or_insert_with(|| {
            out.vertices.push(vertices[v]);
            out.vertices.len() - 1
        });
        let (v0, v1) = (map(edge.v0), map(edge.v1));
        new_edge.insert(e, out.edges.len());
        out.edges.push(Edge { curve: edge.curve.clone(), t0: edge.t0, t1: edge.t1, v0, v1 });
    }
    let mut kept = Vec::with_capacity(loops_of.len());
    for (f, loops) in loops_of {
        let face = &body.faces[f];
        let loops: Vec<Vec<Coedge>> = loops.iter().map(|lp| lp.iter().map(|c| Coedge { edge: new_edge[&c.edge], reversed: c.reversed }).collect()).collect();
        let i = out.add_face(face.surface.clone(), face.flipped, loops, face.tag);
        out.faces[i].origin = face.origin.clone();
        kept.push(f);
    }
    out.compute_shells();
    validate(&out).map_err(|why| refuse(format!("the result wouldn't be a valid solid: {}", why.join("; "))))?;
    let mesh = tessellate(&out, &TessParams::draft_for_size(out.aabb().diagonal()))?;
    if mesh.open_edge_count() > 0 {
        return Err(refuse("the result wouldn't mesh into a closed surface".into()));
    }
    Ok((out, kept))
}

/// Whether two runs lie on one plane, facing the same way, so that their faces can join.
fn one_plane(body: &Body, a: &Run, b: &Run, size: f64) -> bool {
    let (f, g) = (&body.faces[a.owner], &body.faces[b.owner]);
    matches!(f.surface, Surface::Plane(_)) && a.owner != b.owner && f.flipped == g.flipped && ssi::coincident(&f.surface, &g.surface, size).is_some()
}

/// Plans for a connected group of faces, in the order to try them.
fn plans_for(body: &Body, uses: &Uses, group: &[usize]) -> Result<Vec<Plan>> {
    let size = body.aabb().diagonal().max(1.0);
    let mut in_group = vec![None; body.faces.len()];
    for &f in group {
        in_group[f] = Some(0);
    }
    let runs = border_runs(body, uses, &in_group);
    if runs.is_empty() {
        return Ok(vec![Plan::default()]);
    }
    let mut plans = Vec::new();

    // An island: everything around it is whole inner loops of its neighbours.
    let whole = |r: &Run| r.coedges.len() == body.faces[r.owner].loops[r.lp].len();
    if runs.iter().all(|r| r.lp > 0 && whole(r)) {
        plans.push(Plan { drop_loops: runs.iter().map(|r| (r.owner, r.lp)).collect(), ..Plan::default() });
        return Ok(plans);
    }

    // Strips, face by face, for a fillet or chamfer chain whose sides may have several neighbours.
    if let Some(list) = strips_of(body, uses, group) {
        plans.extend(list);
    }

    // The whole group's border: one way around, and how many neighbours that has.
    let mut next: HashMap<usize, usize> = HashMap::new();
    let mut touching = false;
    for (i, r) in runs.iter().enumerate() {
        touching |= next.insert(r.from, i).is_some();
    }
    if !touching {
        let mut order = vec![0usize];
        let mut at = runs[0].to;
        while let Some(&i) = next.get(&at) {
            if i == 0 || order.len() > runs.len() {
                break;
            }
            order.push(i);
            at = runs[i].to;
        }
        if order.len() == runs.len() {
            let r: Vec<&Run> = order.iter().map(|&i| &runs[i]).collect();
            match r.len() {
                2 => plans.push(Plan { long: vec![(r[0].clone(), vec![r[1].clone()])], ..Plan::default() }),
                3 => plans.push(Plan { collapse: r.iter().map(|x| (*x).clone()).collect(), ..Plan::default() }),
                4 => {
                    let strip = |a: usize| Plan {
                        long: vec![(r[a].clone(), vec![r[a + 2].clone()])],
                        collapse: vec![r[(a + 1) % 4].clone(), r[(a + 3) % 4].clone()],
                        ..Plan::default()
                    };
                    for a in 0..2 {
                        if one_plane(body, r[a], r[a + 2], size) {
                            plans.push(Plan { slot: Some([r[a].clone(), r[(a + 1) % 4].clone(), r[a + 2].clone(), r[(a + 3) % 4].clone()]), ..Plan::default() });
                        }
                    }
                    // The pair with the longer sides first, as a strip is longer than it's wide.
                    let (first, second) = if r[0].length(body) + r[2].length(body) >= r[1].length(body) + r[3].length(body) { (0, 1) } else { (1, 0) };
                    plans.push(strip(first));
                    plans.push(strip(second));
                }
                _ => {}
            }
        }
    }
    if plans.is_empty() {
        let names: Vec<String> = runs.iter().map(|r| format!("{:?}", body.faces[r.owner].tag)).collect();
        return Err(refuse(format!(
            "the gap has {} sides on its border, and only islands, strips, corners, slots and whole shells can be closed. Its neighbours are {}",
            runs.len(),
            names.join(", ")
        )));
    }
    Ok(plans)
}

/// A plan for a chain of strips, read off each face. Each has its short sides across it, either shared with the next
/// face in the chain or on a third face at the chain's end, and its long sides on the two faces that have to meet.
fn strips_of(body: &Body, uses: &Uses, group: &[usize]) -> Option<Vec<Plan>> {
    let mut by_face = vec![None; body.faces.len()];
    for &f in group {
        by_face[f] = Some(f);
    }
    let runs = border_runs(body, uses, &by_face);
    let find_run = |owner: usize, f: usize| -> Option<&Run> {
        let mut found = runs.iter().filter(|r| r.owner == owner && r.group == f);
        let first = found.next()?;
        found.next().is_none().then_some(first)
    };
    #[derive(Clone)]
    struct Side {
        interior: bool,
        owner: usize,
        vertices: Vec<usize>,
    }
    // One choice for each face with a choice to make. Only one face alone can have one.
    let mut plans = vec![Plan::default()];
    for &f in group {
        let face = &body.faces[f];
        if face.loops.len() != 1 {
            return None;
        }
        let lp = &face.loops[0];
        let n = lp.len();
        let kind = |i: usize| -> Option<(bool, usize)> {
            let o = other_face(uses, lp[i].edge, f)?;
            Some((by_face[o].is_some(), if by_face[o].is_some() { usize::MAX } else { o }))
        };
        let kinds: Option<Vec<(bool, usize)>> = (0..n).map(kind).collect();
        let kinds = kinds?;
        let starts: Vec<usize> = (0..n).filter(|&i| kinds[i] != kinds[(i + n - 1) % n]).collect();
        if starts.is_empty() {
            if !kinds[0].0 {
                return None;
            }
            // A corner patch among chains, with only them around it: all of it becomes one vertex.
            let ring: Vec<usize> = lp.iter().map(|c| body.start_vertex(*c)).collect();
            for plan in plans.iter_mut() {
                for w in ring.windows(2) {
                    plan.join.extend_from_slice(w);
                }
            }
            continue;
        }
        let mut sides = Vec::with_capacity(starts.len());
        for (k, &s) in starts.iter().enumerate() {
            let end = starts[(k + 1) % starts.len()];
            let mut vertices = vec![body.start_vertex(lp[s])];
            let mut i = s;
            loop {
                vertices.push(body.end_vertex(lp[i]));
                i = (i + 1) % n;
                if i == end {
                    break;
                }
            }
            sides.push(Side { interior: kinds[s].0, owner: kinds[s].1, vertices });
        }
        let m = sides.len();
        let length = |s: &Side| s.vertices.windows(2).map(|w| body.vertices[w[0]].dist(body.vertices[w[1]])).sum::<f64>();
        // Ways to choose the short sides: one or two, not next to each other, including every side shared with another
        // face in the chain, and leaving two stretches of long sides, of which at least one is on a single face.
        let mut options: Vec<(f64, Vec<usize>, Vec<usize>, Vec<usize>)> = Vec::new();
        let interiors: Vec<usize> = (0..m).filter(|&i| sides[i].interior).collect();
        for a in 0..m {
            for b in a + 2..m {
                if a == 0 && b == m - 1 {
                    continue;
                }
                if !interiors.iter().all(|i| *i == a || *i == b) {
                    continue;
                }
                let one: Vec<usize> = (a + 1..b).collect();
                let two: Vec<usize> = (b + 1..m).chain(0..a).collect();
                if one.len() != 1 && two.len() != 1 {
                    continue;
                }
                let long: f64 = one.iter().chain(&two).map(|i| length(&sides[*i])).sum();
                options.push((long, vec![a, b], one, two));
            }
        }
        // One short side, on a face whose long sides meet at its other end.
        if m == 3 && interiors.len() <= 1 {
            for a in 0..3 {
                if interiors.iter().all(|i| *i == a) {
                    let (x, y) = ((a + 1) % 3, (a + 2) % 3);
                    options.push((length(&sides[x]) + length(&sides[y]), vec![a], vec![x], vec![y]));
                }
            }
        }
        // Two long sides and nothing else.
        if m == 2 && interiors.is_empty() {
            options.push((0.0, vec![], vec![0], vec![1]));
        }
        if options.is_empty() {
            return None;
        }
        options.sort_by(|x, y| y.0.total_cmp(&x.0));
        let mut grown = Vec::with_capacity(plans.len() * options.len());
        for (_, cross, one, two) in &options {
            let (single, chain) = if one.len() == 1 { (one, two) } else { (two, one) };
            for plan in &plans {
                let mut with = plan.clone();
                let Some(first) = find_run(sides[single[0]].owner, f) else { continue };
                let Some(list) = chain.iter().map(|i| find_run(sides[*i].owner, f).cloned()).collect::<Option<Vec<_>>>() else { continue };
                with.long.push((first.clone(), list));
                let mut fine = true;
                for &c in cross {
                    if sides[c].interior {
                        for w in sides[c].vertices.windows(2) {
                            with.join.extend_from_slice(w);
                        }
                    } else if let Some(run) = find_run(sides[c].owner, f) {
                        with.collapse.push(run.clone());
                    } else {
                        fine = false;
                    }
                }
                if fine {
                    grown.push(with);
                }
            }
        }
        if grown.is_empty() {
            return None;
        }
        grown.truncate(8);
        plans = grown;
    }
    Some(plans)
}

/// Deletes faces and closes the gap by letting their neighbours grow, or refuses and says why. Faces that keep their place
/// keep their names, so nothing new gets named, and a face that had a neighbour deleted is still the same face.
pub fn delete_faces(body: &Body, faces: &[usize]) -> Result<Body> {
    if faces.is_empty() {
        return Err(Error::InvalidInput("deleting faces takes at least one face".into()));
    }
    if let Some(&bad) = faces.iter().find(|&&f| f >= body.faces.len()) {
        return Err(Error::InvalidInput(format!("there is no face {bad}")));
    }
    let mut gone = vec![false; body.faces.len()];
    for &f in faces {
        gone[f] = true;
    }
    if gone.iter().all(|g| *g) {
        return Err(Error::InvalidInput("deleting every face would leave nothing".into()));
    }
    let mut current = body.clone();
    loop {
        let uses = current.edge_uses();
        if uses.iter().any(|u| !u.is_empty() && u.len() != 2) {
            return Err(refuse("an edge used by other than two faces can't be healed".into()));
        }
        let Some(start) = gone.iter().position(|g| *g) else { break };
        // The faces joined to this one by edges, among those going.
        let mut group = vec![start];
        let mut seen = vec![false; current.faces.len()];
        seen[start] = true;
        let mut k = 0;
        while k < group.len() {
            let f = group[k];
            k += 1;
            for c in current.faces[f].loops.iter().flatten() {
                for u in &uses[c.edge] {
                    if gone[u.0] && !seen[u.0] {
                        seen[u.0] = true;
                        group.push(u.0);
                    }
                }
            }
        }
        let mut only = vec![false; current.faces.len()];
        for &f in &group {
            only[f] = true;
        }
        let plans = plans_for(&current, &uses, &group)?;
        let mut last = None;
        let mut done = None;
        for plan in &plans {
            match apply(&current, &only, plan) {
                Ok(result) => {
                    done = Some(result);
                    break;
                }
                Err(why) => last = Some(why),
            }
        }
        let Some((next, kept)) = done else {
            return Err(last.unwrap_or_else(|| refuse("nothing worked".into())));
        };
        gone = kept.iter().map(|&f| gone[f]).collect();
        current = next;
    }
    if current.faces.is_empty() {
        return Err(Error::InvalidInput("deleting every face would leave nothing".into()));
    }
    let tidy = simplify(current)?;
    validate(&tidy).map_err(|why| refuse(format!("the result wouldn't be a valid solid: {}", why.join("; "))))?;
    Ok(tidy)
}
