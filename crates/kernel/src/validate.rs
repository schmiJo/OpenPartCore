//! The validity checks every operation's output must pass.
//!
//! Topology: every loop closes, every edge is used by exactly two coedges in
//! opposite directions, every vertex has one fan of faces, and each shell obeys
//! the Euler-Poincare formula with a whole non-negative genus.
//!
//! Geometry: curves match their vertices, edges lie on the surfaces of the faces
//! using them, and loops wind the right way in the face chart.

use crate::math::*;
use crate::topo::Body;
use std::collections::HashMap;

/// Largest gap allowed between geometry that should coincide. Intersection points
/// are merged at `TOL`, so a vertex can be up to a little over `TOL` off a curve.
const GEOM_TOL: f64 = 5.0 * TOL;

/// Counts for a valid body, per shell and in total.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Report {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
    pub shells: usize,
    /// Genus of each shell.
    pub genus: Vec<usize>,
}

/// Checks a body. Returns its counts or one line per defect.
pub fn validate(body: &Body) -> Result<Report, Vec<String>> {
    let mut errors = Vec::new();
    check_loops(body, &mut errors);
    check_edge_use(body, &mut errors);
    if errors.is_empty() {
        check_vertex_fans(body, &mut errors);
    }
    let genus = check_euler(body, &mut errors);
    check_geometry(body, &mut errors);
    if errors.is_empty() {
        check_windings(body, &mut errors);
    }
    if errors.is_empty() {
        Ok(Report {
            vertices: body.vertices.len(),
            edges: body.edges.len(),
            faces: body.faces.len(),
            shells: body.shells.len(),
            genus,
        })
    } else {
        Err(errors)
    }
}

fn check_loops(body: &Body, errors: &mut Vec<String>) {
    for (i, e) in body.edges.iter().enumerate() {
        if e.v0 == e.v1 {
            errors.push(format!("edge {i} is a closed loop"));
        }
        if !(e.t0 < e.t1) {
            errors.push(format!("edge {i} has an empty parameter range"));
        }
    }
    for (f, face) in body.faces.iter().enumerate() {
        if face.loops.is_empty() {
            errors.push(format!("face {f} has no boundary"));
        }
        for (l, lp) in face.loops.iter().enumerate() {
            if lp.len() < 2 {
                errors.push(format!("face {f} loop {l} has {} coedges", lp.len()));
            }
            for i in 0..lp.len() {
                let next = lp[(i + 1) % lp.len()];
                if body.end_vertex(lp[i]) != body.start_vertex(next) {
                    errors.push(format!("face {f} loop {l} is broken after coedge {i}"));
                }
            }
        }
    }
}

fn check_edge_use(body: &Body, errors: &mut Vec<String>) {
    for (i, uses) in body.edge_uses().iter().enumerate() {
        match uses.as_slice() {
            [a, b] => {
                if a.3 == b.3 {
                    errors.push(format!(
                        "edge {i} is used twice in the same direction (faces {} and {})",
                        a.0, b.0
                    ));
                }
            }
            _ => errors.push(format!("edge {i} is used by {} coedges, not 2", uses.len())),
        }
    }
}

/// Each vertex must have a single fan of faces around it. Two fans sharing one
/// vertex would be a pinch, which is not a manifold.
fn check_vertex_fans(body: &Body, errors: &mut Vec<String>) {
    // Arriving at a vertex along one coedge and leaving along the next steps to
    // the neighbouring face. Follow these steps around each vertex.
    let mut outgoing: HashMap<(usize, usize), Vec<(usize, bool)>> = HashMap::new();
    let mut per_vertex: HashMap<usize, usize> = HashMap::new();
    let mut leave_after: HashMap<(usize, bool), (usize, bool)> = HashMap::new();
    for face in &body.faces {
        for lp in &face.loops {
            for i in 0..lp.len() {
                let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
                leave_after.insert((a.edge, a.reversed), (b.edge, b.reversed));
                let v = body.end_vertex(a);
                *per_vertex.entry(v).or_insert(0) += 1;
                outgoing.entry((v, b.edge)).or_default().push((b.edge, b.reversed));
            }
        }
    }
    let mut visited: HashMap<(usize, bool), bool> = HashMap::new();
    let mut fans: HashMap<usize, usize> = HashMap::new();
    for face in &body.faces {
        for lp in &face.loops {
            for &c in lp {
                let key = (c.edge, c.reversed);
                if visited.contains_key(&key) {
                    continue;
                }
                let v = body.end_vertex(c);
                *fans.entry(v).or_insert(0) += 1;
                let mut cur = key;
                loop {
                    visited.insert(cur, true);
                    // Next coedge in this loop leaves v; its twin arrives at v in
                    // the neighbouring face.
                    let Some(&(e, r)) = leave_after.get(&cur) else {
                        break;
                    };
                    let twin = (e, !r);
                    if visited.contains_key(&twin) {
                        break;
                    }
                    cur = twin;
                }
            }
        }
    }
    for (v, n) in fans {
        if n != 1 {
            errors.push(format!("vertex {v} joins {n} separate fans of faces (a pinch)"));
        }
    }
}

fn check_euler(body: &Body, errors: &mut Vec<String>) -> Vec<usize> {
    let mut genus = Vec::new();
    for (s, shell) in body.shells.iter().enumerate() {
        let mut vertices = std::collections::HashSet::new();
        let mut edges = std::collections::HashSet::new();
        let mut holes = 0i64;
        for &f in shell {
            let face = &body.faces[f];
            holes += face.loops.len() as i64 - 1;
            for lp in &face.loops {
                for &c in lp {
                    edges.insert(c.edge);
                    vertices.insert(body.start_vertex(c));
                }
            }
        }
        let (v, e, f) = (vertices.len() as i64, edges.len() as i64, shell.len() as i64);
        // V - E + F - H = 2(1 - G) for one closed orientable shell.
        let chi = v - e + f - holes;
        if chi % 2 != 0 || chi > 2 {
            errors.push(format!(
                "shell {s} fails Euler-Poincare: V={v} E={e} F={f} H={holes}, chi={chi}"
            ));
            genus.push(0);
        } else {
            genus.push(((2 - chi) / 2) as usize);
        }
    }
    let total: usize = body.shells.iter().map(Vec::len).sum();
    if total != body.faces.len() {
        errors.push(format!(
            "shells list {total} faces but the body has {}",
            body.faces.len()
        ));
    }
    genus
}

fn check_geometry(body: &Body, errors: &mut Vec<String>) {
    for (i, e) in body.edges.iter().enumerate() {
        if !body.vertices[e.v0].is_finite() || !body.vertices[e.v1].is_finite() {
            errors.push(format!("edge {i} has a non-finite vertex"));
            continue;
        }
        let d0 = e.curve.eval(e.t0).dist(body.vertices[e.v0]);
        let d1 = e.curve.eval(e.t1).dist(body.vertices[e.v1]);
        if d0 > GEOM_TOL || d1 > GEOM_TOL {
            errors.push(format!("edge {i} ends are off its vertices by {d0:e} and {d1:e}"));
        }
        if body.vertices[e.v0].dist(body.vertices[e.v1]) < TOL && (e.t1 - e.t0) * e.curve.speed_bound() < 2.0 * TOL {
            errors.push(format!("edge {i} has zero length"));
        }
    }
    for (f, face) in body.faces.iter().enumerate() {
        for lp in &face.loops {
            for &c in lp {
                for k in 0..=4 {
                    let p = body.coedge_point(c, k as f64 / 4.0);
                    let d = face.surface.sdf(p).abs();
                    if d > GEOM_TOL {
                        errors.push(format!(
                            "edge {} is {d:e} off the {} of face {f}",
                            c.edge,
                            face.surface.name()
                        ));
                        break;
                    }
                }
            }
        }
    }
}

/// Loops must wind correctly in the chart, and holes must be inside the outer loop.
fn check_windings(body: &Body, errors: &mut Vec<String>) {
    for (f, face) in body.faces.iter().enumerate() {
        let sign = if face.flipped { -1.0 } else { 1.0 };
        let polygons: Vec<Vec<Vec2>> = face
            .loops
            .iter()
            .map(|lp| {
                let mut pts = Vec::new();
                for &c in lp {
                    for k in 0..16 {
                        pts.push(face.surface.to_chart(&face.chart, body.coedge_point(c, k as f64 / 16.0)));
                    }
                }
                pts
            })
            .collect();
        for (l, poly) in polygons.iter().enumerate() {
            let area = polygon_area(poly) * sign;
            if l == 0 && area <= 0.0 {
                errors.push(format!("face {f} outer loop winds the wrong way (area {area:e})"));
            }
            if l > 0 && area >= 0.0 {
                errors.push(format!("face {f} hole {l} winds the wrong way (area {area:e})"));
            }
        }
    }
}

pub(crate) fn polygon_area(pts: &[Vec2]) -> f64 {
    let mut a = 0.0;
    let origin = pts.first().copied().unwrap_or_default();
    for i in 0..pts.len() {
        let (p, q) = (pts[i] - origin, pts[(i + 1) % pts.len()] - origin);
        a += p.cross(q);
    }
    a / 2.0
}
