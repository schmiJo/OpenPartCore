//! Tessellation for display and STL.
//!
//! Each edge is sampled once, and both faces using it share those points exactly,
//! so meshes are watertight without welding by tolerance. Faces are triangulated
//! in their charts, then refined until the mesh is within the chord tolerance.

pub mod cdt;

use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::surface::Surface;
use crate::topo::{Body, Edge};
use cdt::{Cdt, GRID};
use crate::hash::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct TessParams {
    /// Maximum distance between the mesh and the true surface, in model units.
    pub chord: f64,
    /// Maximum turning between neighbouring samples, in radians.
    pub angle: f64,
}

impl TessParams {
    /// Sensible display settings for a model of the given size.
    pub fn for_size(size: f64) -> TessParams {
        TessParams { chord: (size * 4e-4).max(1e-4), angle: 18f64.to_radians() }
    }
}

/// A triangle mesh. Vertices are not shared between faces, so each face keeps its
/// own normals and creases stay sharp.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub positions: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    /// Source face of each triangle.
    pub face_of_triangle: Vec<u32>,
    /// One polyline per B-rep edge, for drawing crisp edges.
    pub edge_lines: Vec<Vec<Vec3>>,
}

impl Mesh {
    /// Signed volume enclosed by the mesh.
    pub fn volume(&self) -> f64 {
        let c = self.bounds().center();
        let mut v = 0.0;
        for t in &self.triangles {
            let (a, b, d) = (
                self.positions[t[0] as usize] - c,
                self.positions[t[1] as usize] - c,
                self.positions[t[2] as usize] - c,
            );
            v += a.dot(b.cross(d));
        }
        v / 6.0
    }

    pub fn area(&self) -> f64 {
        let mut s = 0.0;
        for t in &self.triangles {
            let (a, b, d) =
                (self.positions[t[0] as usize], self.positions[t[1] as usize], self.positions[t[2] as usize]);
            s += (b - a).cross(d - a).norm() / 2.0;
        }
        s
    }

    pub fn bounds(&self) -> Aabb {
        let mut b = Aabb::EMPTY;
        for &p in &self.positions {
            b.add(p);
        }
        b
    }

    /// Mesh edges that do not pair up exactly once in each direction, after
    /// matching vertices by exact coordinates. A watertight mesh has none.
    pub fn open_edge_count(&self) -> usize {
        self.open_edges().len()
    }

    /// The endpoints of every mesh edge that fails to pair up.
    pub fn open_edges(&self) -> Vec<(Vec3, Vec3)> {
        let mut ids: HashMap<[u64; 3], u32> = HashMap::default();
        let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let mut canon = Vec::with_capacity(self.positions.len());
        for &p in &self.positions {
            let n = ids.len() as u32;
            canon.push(*ids.entry(key(p)).or_insert(n));
        }
        let mut count: HashMap<(u32, u32), i64> = HashMap::default();
        for t in &self.triangles {
            for i in 0..3 {
                let (a, b) = (canon[t[i] as usize], canon[t[(i + 1) % 3] as usize]);
                if a == b {
                    continue;
                }
                if a < b {
                    *count.entry((a, b)).or_insert(0) += 1;
                } else {
                    *count.entry((b, a)).or_insert(0) -= 1;
                }
            }
        }
        let mut position = vec![Vec3::ZERO; ids.len()];
        for (i, &c) in canon.iter().enumerate() {
            position[c as usize] = self.positions[i];
        }
        let mut open: Vec<(Vec3, Vec3)> = count
            .iter()
            .filter(|(_, &c)| c != 0)
            .map(|(&(a, b), _)| (position[a as usize], position[b as usize]))
            .collect();
        open.sort_by(|p, q| p.0.x.total_cmp(&q.0.x));
        open
    }
}

/// Samples an edge. The first and last points are exactly its vertices. `density`
/// multiplies the number of samples.
///
/// `graded` adds samples at power-of-two distances from each end. Two curves leaving a
/// vertex tangentially can have chords that cross however dense they are, if their
/// samples fall at different distances. Sampled at the same distances, the chords keep
/// the same order as the curves, and these distances mean the same on every edge.
///
/// No sample lands closer than `min_gap` to either end. Tangent curves separate with the
/// square of the distance from their shared vertex, so below about the square root of
/// the mesher's grid they can't be told apart, and rounding could put them in the wrong
/// order. Leaving those samples out moves the mesh far less than the chord tolerance.
pub fn edge_polyline(
    e: &Edge,
    vertices: &[Vec3],
    params: &TessParams,
    density: usize,
    graded: bool,
    min_gap: f64,
) -> Vec<Vec3> {
    let dt = e.t1 - e.t0;
    let base = match &e.curve {
        Curve::Line { .. } => 1,
        Curve::Circle { radius, .. } => {
            let by_chord = 2.0 * (1.0 - (params.chord / radius).min(1.0)).acos();
            ((dt / by_chord.min(params.angle).max(1e-3)).ceil() as usize).max(2)
        }
        Curve::Ellipse { major, minor, .. } => {
            let by_chord = (8.0 * params.chord / major).sqrt();
            let by_angle = params.angle * minor / major;
            ((dt / by_chord.min(by_angle).max(1e-3)).ceil() as usize).max(2)
        }
        Curve::Sampled(_) => (dt.ceil() as usize).max(2),
    };
    let steps = base * density;
    let mut params_on_curve: Vec<f64> = (1..steps).map(|i| e.t0 + dt * i as f64 / steps as f64).collect();
    if graded {
        let length_guess = dt * e.curve.derivative(e.t0 + dt / 2.0).norm();
        let (speed0, speed1) = (e.curve.derivative(e.t0).norm().max(1e-12), e.curve.derivative(e.t1).norm().max(1e-12));
        let mut distance = 2f64.powi((length_guess / 2.0).max(1e-300).log2().floor() as i32);
        for _ in 0..24 {
            let (a, b) = (e.t0 + distance / speed0, e.t1 - distance / speed1);
            if a > e.t0 && a < e.t1 {
                params_on_curve.push(a);
            }
            if b > e.t0 && b < e.t1 {
                params_on_curve.push(b);
            }
            distance /= 2.0;
        }
        params_on_curve.sort_by(f64::total_cmp);
        params_on_curve.dedup_by(|a, b| (*a - *b).abs() < dt * 1e-9);
    }
    let (first, last) = (vertices[e.v0], vertices[e.v1]);
    let mut pts = Vec::with_capacity(params_on_curve.len() + 2);
    pts.push(first);
    for &t in &params_on_curve {
        let p = e.curve.eval(t);
        if p.dist(first) > min_gap && p.dist(last) > min_gap {
            pts.push(p);
        }
    }
    // A curved edge keeps at least one interior point, so that two arcs between the
    // same vertices never collapse onto one chord.
    if pts.len() == 1 && !params_on_curve.is_empty() {
        pts.push(e.curve.eval(e.t0 + dt / 2.0));
    }
    pts.push(last);
    pts
}

/// The sampling floor for a body of this size. See `edge_polyline`.
pub fn min_sample_gap(size: f64) -> f64 {
    size / 16384.0
}

/// Tessellates a body into a watertight mesh.
///
/// Two edges leaving a vertex at a small angle can have chords that cross even
/// though the curves do not. When that happens the affected face's edges get more
/// samples and everything starts again. Both faces of an edge must use the same
/// points, so a partial redo could tear the mesh.
pub fn tessellate(body: &Body, params: &TessParams) -> Result<Mesh> {
    let mut density = vec![1usize; body.edges.len()];
    let mut graded = vec![false; body.edges.len()];
    let mut bounds = Aabb::EMPTY;
    for &p in &body.vertices {
        bounds.add(p);
    }
    let min_gap = min_sample_gap(bounds.diagonal());
    let mut last_error = None;
    for _attempt in 0..5 {
        let mut polylines: Vec<Vec<Vec3>> = body
            .edges
            .iter()
            .enumerate()
            .map(|(i, e)| edge_polyline(e, &body.vertices, params, density[i], graded[i], min_gap))
            .collect();
        let mut mesh = Mesh::default();
        let mut crossing = None;
        for f in 0..body.faces.len() {
            if let Err(e) = tessellate_face(body, f, &mut polylines, params, &mut mesh) {
                crossing = Some((f, e));
                break;
            }
        }
        match crossing {
            None => {
                mesh.edge_lines = polylines;
                return Ok(mesh);
            }
            Some((f, e)) => {
                last_error = Some(e);
                for lp in &body.faces[f].loops {
                    for c in lp {
                        density[c.edge] *= 2;
                        graded[c.edge] = true;
                    }
                }
            }
        }
    }
    Err(last_error.unwrap())
}

/// Whether the part of a face around its new edges triangulates with its boundary intact:
/// no crossing edges, and every boundary segment appearing as a mesh edge with nothing
/// else left open. This is the part of meshing that can fail, since refinement only adds
/// interior points. Only loops near the new edges can conflict with them, so a plate with
/// hundreds of holes costs no more than its one new hole.
pub fn face_meshes_cleanly(body: &Body, f: usize, is_new: &[bool], params: &TessParams) -> bool {
    let face = &body.faces[f];
    let mut bounds = Aabb::EMPTY;
    for &p in &body.vertices {
        bounds.add(p);
    }
    let min_gap = min_sample_gap(bounds.diagonal());
    let mut polylines: Vec<Vec<Vec3>> = vec![Vec::new(); body.edges.len()];
    for &c in face.loops.iter().flatten() {
        if polylines[c.edge].is_empty() {
            polylines[c.edge] = edge_polyline(&body.edges[c.edge], &body.vertices, params, 1, false, min_gap);
        }
    }
    // Chart bounds of each edge, and of everything new on this face.
    let edge_box = |e: usize| {
        let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &p in &polylines[e] {
            let uv = face.surface.to_chart(&face.chart, p);
            lo = v2(lo.x.min(uv.x), lo.y.min(uv.y));
            hi = v2(hi.x.max(uv.x), hi.y.max(uv.y));
        }
        (lo, hi)
    };
    let (mut new_lo, mut new_hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for &c in face.loops.iter().flatten() {
        if is_new[c.edge] {
            let (lo, hi) = edge_box(c.edge);
            new_lo = v2(new_lo.x.min(lo.x), new_lo.y.min(lo.y));
            new_hi = v2(new_hi.x.max(hi.x), new_hi.y.max(hi.y));
        }
    }
    let margin = (new_hi - new_lo).norm() * 0.05 + 1e-9;
    let chosen: Vec<usize> = (0..face.loops.len())
        .filter(|&l| {
            face.loops[l].iter().any(|c| {
                if is_new[c.edge] {
                    return true;
                }
                let (lo, hi) = edge_box(c.edge);
                lo.x <= new_hi.x + margin
                    && hi.x >= new_lo.x - margin
                    && lo.y <= new_hi.y + margin
                    && hi.y >= new_lo.y - margin
            })
        })
        .collect();

    // Directed segments cancel where a loop runs both ways along a slit. What remains,
    // ignoring direction because a subset of loops can enclose the opposite side, should
    // be exactly the mesh's open edges.
    let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let net = |list: &mut HashMap<[[u64; 3]; 2], i32>, a: [u64; 3], b: [u64; 3]| {
        if a < b {
            *list.entry([a, b]).or_insert(0) += 1;
        } else if b < a {
            *list.entry([b, a]).or_insert(0) -= 1;
        }
    };
    let mut expected: HashMap<[[u64; 3]; 2], i32> = HashMap::default();
    for &l in &chosen {
        for &c in &face.loops[l] {
            for w in polylines[c.edge].windows(2) {
                let (a, b) = if c.reversed { (w[1], w[0]) } else { (w[0], w[1]) };
                net(&mut expected, key(a), key(b));
            }
        }
    }
    expected.retain(|_, v| *v != 0);
    let mut mesh = Mesh::default();
    if build_face(body, f, &polylines, params, &mut mesh, false, Some(&chosen)).is_err() {
        return false;
    }
    let mut open: HashMap<[[u64; 3]; 2], i32> = HashMap::default();
    for t in &mesh.triangles {
        for i in 0..3 {
            net(&mut open, key(mesh.positions[t[i] as usize]), key(mesh.positions[t[(i + 1) % 3] as usize]));
        }
    }
    open.retain(|_, v| *v != 0);
    open.len() == expected.len() && open.keys().all(|k| expected.contains_key(k))
}

struct FaceFrame {
    lo: Vec2,
    scale: f64,
}

impl FaceFrame {
    fn to_grid(&self, uv: Vec2) -> [i64; 2] {
        [((uv.x - self.lo.x) * self.scale).round() as i64, ((uv.y - self.lo.y) * self.scale).round() as i64]
    }

    fn from_grid(&self, q: [i64; 2]) -> Vec2 {
        v2(q[0] as f64 / self.scale + self.lo.x, q[1] as f64 / self.scale + self.lo.y)
    }
}

fn tessellate_face(
    body: &Body,
    f: usize,
    polylines: &mut [Vec<Vec3>],
    params: &TessParams,
    mesh: &mut Mesh,
) -> Result<()> {
    build_face(body, f, polylines, params, mesh, true, None)
}

fn build_face(
    body: &Body,
    f: usize,
    polylines: &[Vec<Vec3>],
    params: &TessParams,
    mesh: &mut Mesh,
    refine_surface: bool,
    only_loops: Option<&[usize]>,
) -> Result<()> {
    let face = &body.faces[f];
    let (surface, chart) = (&face.surface, &face.chart);

    // Collect each loop as a closed chain of 3D points.
    let chains: Vec<Vec<Vec3>> = face
        .loops
        .iter()
        .enumerate()
        .filter(|(l, _)| only_loops.is_none_or(|chosen| chosen.contains(l)))
        .map(|(_, lp)| {
            let mut chain = Vec::new();
            for &c in lp {
                let line = &polylines[c.edge];
                if c.reversed {
                    chain.extend(line.iter().rev().take(line.len() - 1).copied());
                } else {
                    chain.extend(line.iter().take(line.len() - 1).copied());
                }
            }
            chain
        })
        .collect();

    let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for &p in chains.iter().flatten() {
        let uv = surface.to_chart(chart, p);
        lo = v2(lo.x.min(uv.x), lo.y.min(uv.y));
        hi = v2(hi.x.max(uv.x), hi.y.max(uv.y));
    }
    let extent = (hi.x - lo.x).max(hi.y - lo.y).max(1e-9);
    let frame = FaceFrame { lo, scale: (GRID - 2) as f64 / extent };

    let mut cdt = Cdt::new();
    let mut point_of: HashMap<u32, Vec3> = HashMap::default();
    let mut loop_ids = Vec::with_capacity(chains.len());
    for chain in &chains {
        let mut ids = Vec::with_capacity(chain.len());
        for &p in chain {
            let q = frame.to_grid(surface.to_chart(chart, p));
            let id = cdt.insert(q, true).expect("boundary insertion always succeeds");
            point_of.entry(id).or_insert(p);
            ids.push(id);
        }
        loop_ids.push(ids);
    }
    for ids in &loop_ids {
        for i in 0..ids.len() {
            cdt.constrain(ids[i], ids[(i + 1) % ids.len()]).map_err(|_| {
                Error::Internal(format!(
                    "face {f} ({}): boundary crosses itself at tessellation resolution",
                    surface.name()
                ))
            })?;
        }
    }
    cdt.mark_inside(&loop_ids);

    if refine_surface && !matches!(surface, Surface::Plane(_)) {
        refine(&mut cdt, &frame, surface, chart, &mut point_of, params);
    }

    // Emit the triangles.
    let sign = if face.flipped { -1.0 } else { 1.0 };
    let base = mesh.positions.len() as u32;
    let mut remap: HashMap<u32, u32> = HashMap::default();
    let triangles: Vec<[u32; 3]> = cdt.inside_triangles().map(|(v, _)| v).collect();
    for tri in triangles {
        let mut out = [0u32; 3];
        for (k, &v) in tri.iter().enumerate() {
            out[k] = *remap.entry(v).or_insert_with(|| {
                let p = point_of[&v];
                mesh.positions.push(p);
                mesh.normals.push(surface.normal(p) * sign);
                base + (mesh.positions.len() as u32 - 1 - base)
            });
        }
        if face.flipped {
            out.swap(1, 2);
        }
        mesh.triangles.push(out);
        mesh.face_of_triangle.push(f as u32);
    }
    Ok(())
}

/// Adds interior points until the mesh follows the surface.
fn refine(
    cdt: &mut Cdt,
    frame: &FaceFrame,
    surface: &Surface,
    chart: &crate::surface::Chart,
    point_of: &mut HashMap<u32, Vec3>,
    params: &TessParams,
) {
    const MAX_POINTS: usize = 40_000;
    for _pass in 0..24 {
        let tris: Vec<([u32; 3], [bool; 3])> = cdt.inside_triangles().collect();
        let mut wanted: Vec<[i64; 2]> = Vec::new();
        for (v, constrained) in &tris {
            let grid = [cdt.pts[v[0] as usize], cdt.pts[v[1] as usize], cdt.pts[v[2] as usize]];
            let world = [point_of[&v[0]], point_of[&v[1]], point_of[&v[2]]];
            let mut worst = 0.0;
            let mut candidate = None;
            // Interior edges: split one when its midpoint leaves the surface or
            // its normals turn too far.
            for i in 0..3 {
                if constrained[i] {
                    continue;
                }
                let (a, b) = ((i + 1) % 3, (i + 2) % 3);
                let (ga, gb) = (grid[a], grid[b]);
                let m = [(ga[0] + gb[0]) / 2, (ga[1] + gb[1]) / 2];
                if m == ga || m == gb {
                    continue;
                }
                // Distance from the chord to the surface. This is the mesh error;
                // comparing against the chart midpoint would also count sliding
                // along the surface, which is not an error at all.
                let sag = surface.sdf((world[a] + world[b]) * 0.5).abs();
                let turn = surface.normal(world[a]).dot(surface.normal(world[b])).clamp(-1.0, 1.0).acos();
                let score = (sag / params.chord).max(turn / params.angle);
                if score > 1.0 && score > worst {
                    worst = score;
                    candidate = Some(m);
                }
            }
            // The triangle's middle catches bulges with no interior edge to split.
            let c = [(grid[0][0] + grid[1][0] + grid[2][0]) / 3, (grid[0][1] + grid[1][1] + grid[2][1]) / 3];
            if c != grid[0] && c != grid[1] && c != grid[2] {
                let flat = (world[0] + world[1] + world[2]) / 3.0;
                let score = surface.sdf(flat).abs() / params.chord;
                if score > 1.0 && score > worst {
                    candidate = Some(c);
                }
            }
            if let Some(q) = candidate {
                wanted.push(q);
            }
        }
        if wanted.is_empty() || cdt.pts.len() > MAX_POINTS {
            break;
        }
        let mut added = false;
        for q in wanted {
            let before = cdt.pts.len();
            if let Some(id) = cdt.insert(q, false) {
                if cdt.pts.len() > before {
                    point_of.insert(id, surface.from_chart(chart, frame.from_grid(q)));
                    added = true;
                }
            }
        }
        if !added {
            break;
        }
    }
}
