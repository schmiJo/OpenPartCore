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
use std::collections::HashMap;

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
        TessParams {
            chord: (size * 4e-4).max(1e-4),
            angle: 18f64.to_radians(),
        }
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
            let (a, b, d) = (
                self.positions[t[0] as usize],
                self.positions[t[1] as usize],
                self.positions[t[2] as usize],
            );
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
        let mut ids: HashMap<[u64; 3], u32> = HashMap::new();
        let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let mut canon = Vec::with_capacity(self.positions.len());
        for &p in &self.positions {
            let n = ids.len() as u32;
            canon.push(*ids.entry(key(p)).or_insert(n));
        }
        let mut count: HashMap<(u32, u32), i64> = HashMap::new();
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
        count.values().filter(|&&c| c != 0).count()
    }
}

/// Samples an edge. The first and last points are exactly its vertices.
fn edge_polyline(e: &Edge, vertices: &[Vec3], params: &TessParams) -> Vec<Vec3> {
    let dt = e.t1 - e.t0;
    let steps = match &e.curve {
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
    let mut pts = Vec::with_capacity(steps + 1);
    pts.push(vertices[e.v0]);
    for i in 1..steps {
        pts.push(e.curve.eval(e.t0 + dt * i as f64 / steps as f64));
    }
    pts.push(vertices[e.v1]);
    pts
}

/// Tessellates a body into a watertight mesh.
pub fn tessellate(body: &Body, params: &TessParams) -> Result<Mesh> {
    let mut polylines: Vec<Vec<Vec3>> = body
        .edges
        .iter()
        .map(|e| edge_polyline(e, &body.vertices, params))
        .collect();
    let mut mesh = Mesh::default();
    for f in 0..body.faces.len() {
        tessellate_face(body, f, &mut polylines, params, &mut mesh)?;
    }
    mesh.edge_lines = polylines;
    Ok(mesh)
}

struct FaceFrame {
    lo: Vec2,
    scale: f64,
}

impl FaceFrame {
    fn to_grid(&self, uv: Vec2) -> [i64; 2] {
        [
            ((uv.x - self.lo.x) * self.scale).round() as i64,
            ((uv.y - self.lo.y) * self.scale).round() as i64,
        ]
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
    let face = &body.faces[f];
    let (surface, chart) = (&face.surface, &face.chart);

    // Collect each loop as a closed chain of 3D points.
    let chains: Vec<Vec<Vec3>> = face
        .loops
        .iter()
        .map(|lp| {
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

    let (mut lo, mut hi) = (
        v2(f64::INFINITY, f64::INFINITY),
        v2(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for &p in chains.iter().flatten() {
        let uv = surface.to_chart(chart, p);
        lo = v2(lo.x.min(uv.x), lo.y.min(uv.y));
        hi = v2(hi.x.max(uv.x), hi.y.max(uv.y));
    }
    let extent = (hi.x - lo.x).max(hi.y - lo.y).max(1e-9);
    let frame = FaceFrame {
        lo,
        scale: (GRID - 2) as f64 / extent,
    };

    let mut cdt = Cdt::new();
    let mut point_of: HashMap<u32, Vec3> = HashMap::new();
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
    cdt.mark_inside();

    if !matches!(surface, Surface::Plane(_)) {
        refine(&mut cdt, &frame, surface, chart, &mut point_of, params);
    }

    // Emit the triangles.
    let sign = if face.flipped { -1.0 } else { 1.0 };
    let base = mesh.positions.len() as u32;
    let mut remap: HashMap<u32, u32> = HashMap::new();
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
            let c = [
                (grid[0][0] + grid[1][0] + grid[2][0]) / 3,
                (grid[0][1] + grid[1][1] + grid[2][1]) / 3,
            ];
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
