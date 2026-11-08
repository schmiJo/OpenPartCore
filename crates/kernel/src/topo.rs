//! The B-rep data model: vertices, edges, faces and shells in plain arrays.
//!
//! There are no next/prev pointers. A loop is an ordered list of coedges, and
//! adjacency is computed when it is needed. Operations build a new `Body`
//! rather than editing one in place, which removes a large class of dangling
//! reference bugs and makes feature history trivial to cache.

use crate::curve::Curve;
use crate::math::*;
use crate::surface::{Chart, Surface};

/// One use of an edge by a face loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Coedge {
    pub edge: usize,
    /// True when the loop runs from the edge's `v1` to its `v0`.
    pub reversed: bool,
}

impl Coedge {
    pub fn flipped(self) -> Coedge {
        Coedge { edge: self.edge, reversed: !self.reversed }
    }
}

/// An edge is a piece of a curve between two distinct vertices, with `t0 < t1`.
/// It is never a closed loop: a full circle is at least two edges.
#[derive(Clone, Debug)]
pub struct Edge {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
    pub v0: usize,
    pub v1: usize,
}

/// Stable name of a face: the feature that created it and a slot in that
/// feature. Booleans preserve tags, so later features can refer to edges as
/// "between these two named faces" and survive a rebuild.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceTag {
    pub feature: u32,
    pub slot: u32,
}

/// A trimmed piece of a surface.
///
/// `loops[0]` is the outer boundary. Every loop keeps the face on its left when
/// seen from outside the body, so seen from outside the outer loop runs
/// counter-clockwise and holes run clockwise.
#[derive(Clone, Debug)]
pub struct Face {
    pub surface: Surface,
    /// True when the outward normal is opposite to the surface normal.
    pub flipped: bool,
    pub chart: Chart,
    pub loops: Vec<Vec<Coedge>>,
    pub tag: FaceTag,
}

/// A solid: any number of closed shells. Outer shells enclose positive volume
/// and voids enclose negative volume.
#[derive(Clone, Debug, Default)]
pub struct Body {
    pub vertices: Vec<Vec3>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub shells: Vec<Vec<usize>>,
}

impl Edge {
    pub fn eval_mid(&self) -> Vec3 {
        self.curve.eval((self.t0 + self.t1) / 2.0)
    }

    /// Conservative bounding box.
    pub fn aabb(&self, vertices: &[Vec3]) -> Aabb {
        let mut b = Aabb::EMPTY;
        b.add(vertices[self.v0]);
        b.add(vertices[self.v1]);
        if let Curve::Line { .. } = self.curve {
            return b;
        }
        let n = 32;
        let dt = (self.t1 - self.t0) / n as f64;
        for i in 0..=n {
            b.add(self.curve.eval(self.t0 + dt * i as f64));
        }
        b.inflated(self.curve.speed_bound() * dt / 2.0)
    }
}

impl Face {
    /// Outward normal at a point on the face.
    pub fn outward_normal(&self, p: Vec3) -> Vec3 {
        let n = self.surface.normal(p);
        if self.flipped {
            -n
        } else {
            n
        }
    }
}

impl Body {
    pub fn add_vertex(&mut self, p: Vec3) -> usize {
        self.vertices.push(p);
        self.vertices.len() - 1
    }

    /// Adds an edge running from vertex `a` at parameter `ta` to vertex `b` at
    /// parameter `tb`, and returns the coedge pointing from `a` to `b`.
    pub fn add_edge(&mut self, curve: Curve, ta: f64, tb: f64, a: usize, b: usize) -> Coedge {
        let reversed = ta > tb;
        let (t0, t1, v0, v1) = if reversed { (tb, ta, b, a) } else { (ta, tb, a, b) };
        self.edges.push(Edge { curve, t0, t1, v0, v1 });
        Coedge { edge: self.edges.len() - 1, reversed }
    }

    pub fn start_vertex(&self, c: Coedge) -> usize {
        let e = &self.edges[c.edge];
        if c.reversed {
            e.v1
        } else {
            e.v0
        }
    }

    pub fn end_vertex(&self, c: Coedge) -> usize {
        let e = &self.edges[c.edge];
        if c.reversed {
            e.v0
        } else {
            e.v1
        }
    }

    /// Evaluates a coedge at `s` in `[0, 1]`, measured along the loop direction.
    pub fn coedge_point(&self, c: Coedge, s: f64) -> Vec3 {
        let e = &self.edges[c.edge];
        let s = if c.reversed { 1.0 - s } else { s };
        e.curve.eval(e.t0 + (e.t1 - e.t0) * s)
    }

    /// Fits a chart to a face's loops.
    pub fn fit_face_chart(&self, surface: &Surface, flipped: bool, loops: &[Vec<Coedge>]) -> Chart {
        let mut points = Vec::new();
        for l in loops {
            for &c in l {
                for i in 0..=8 {
                    points.push(self.coedge_point(c, i as f64 / 8.0));
                }
            }
        }
        let mut chart = surface.fit_chart(&points);
        if let Surface::Sphere { frame, .. } = surface {
            // A patch's boundary can be a whole great circle, which does not say
            // which side the patch is on. The loop direction does: the vector
            // area of the loops points at the middle of the patch.
            let mut area = Vec3::ZERO;
            for l in loops {
                let mut chain = Vec::new();
                for &c in l {
                    for i in 0..16 {
                        chain.push(self.coedge_point(c, i as f64 / 16.0) - frame.origin);
                    }
                }
                for i in 0..chain.len() {
                    area += chain[i].cross(chain[(i + 1) % chain.len()]) * 0.5;
                }
            }
            let centre = if flipped { -area } else { area };
            if centre.norm() > 1e-12 {
                let z = centre.normalized();
                chart.frame = Frame::new(frame.origin, z, z.any_perpendicular());
            }
        }
        chart
    }

    /// Adds a face and fits its chart to its boundary.
    pub fn add_face(&mut self, surface: Surface, flipped: bool, loops: Vec<Vec<Coedge>>, tag: FaceTag) -> usize {
        let chart = self.fit_face_chart(&surface, flipped, &loops);
        self.faces.push(Face { surface, flipped, chart, loops, tag });
        self.faces.len() - 1
    }

    /// For each edge, the faces using it, as `(face, loop, index, reversed)`.
    pub fn edge_uses(&self) -> Vec<Vec<(usize, usize, usize, bool)>> {
        let mut uses = vec![Vec::new(); self.edges.len()];
        for (f, face) in self.faces.iter().enumerate() {
            for (l, lp) in face.loops.iter().enumerate() {
                for (i, c) in lp.iter().enumerate() {
                    uses[c.edge].push((f, l, i, c.reversed));
                }
            }
        }
        uses
    }

    /// Groups faces into shells by edge connectivity.
    pub fn compute_shells(&mut self) {
        let uses = self.edge_uses();
        let mut seen = vec![false; self.faces.len()];
        let mut shells = Vec::new();
        for start in 0..self.faces.len() {
            if seen[start] {
                continue;
            }
            seen[start] = true;
            let mut stack = vec![start];
            let mut shell = Vec::new();
            while let Some(f) = stack.pop() {
                shell.push(f);
                for lp in &self.faces[f].loops {
                    for c in lp {
                        for u in &uses[c.edge] {
                            if !seen[u.0] {
                                seen[u.0] = true;
                                stack.push(u.0);
                            }
                        }
                    }
                }
            }
            shell.sort_unstable();
            shells.push(shell);
        }
        self.shells = shells;
    }

    /// Number of points where the body touches itself: places a Boolean separated into
    /// duplicate vertices because more than one fan of faces met there. A body with any
    /// is valid and manifold as a model, but it would be a zero-thickness contact as a
    /// physical part, so callers may want to warn about it.
    pub fn contact_vertices(&self) -> usize {
        let mut seen: std::collections::HashMap<[u64; 3], usize> = std::collections::HashMap::new();
        for p in &self.vertices {
            *seen.entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).or_insert(0) += 1;
        }
        seen.values().filter(|&&n| n > 1).count()
    }

    /// Rigidly transforms the body.
    pub fn transformed(&self, t: &Frame) -> Body {
        let mut out = self.clone();
        for p in &mut out.vertices {
            *p = t.to_world(*p);
        }
        for e in &mut out.edges {
            e.curve = e.curve.transformed(t);
        }
        for f in &mut out.faces {
            f.surface = f.surface.transformed(t);
            f.chart.frame = t.compose(&f.chart.frame);
        }
        out
    }

    /// Conservative bounding box of one face.
    pub fn face_aabb(&self, f: usize) -> Aabb {
        let face = &self.faces[f];
        let mut b = Aabb::EMPTY;
        for lp in &face.loops {
            for c in lp {
                b.union(&self.edges[c.edge].aabb(&self.vertices));
            }
        }
        // A ruled surface reaches its extremes on the boundary. Spheres and tori
        // can bulge past it, so bound the interior through the chart: 3D distance
        // never exceeds chart distance.
        if matches!(face.surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
            let (lo, hi) = self.face_chart_bounds(f);
            let n = 8;
            let (dx, dy) = ((hi.x - lo.x) / n as f64, (hi.y - lo.y) / n as f64);
            for i in 0..=n {
                for j in 0..=n {
                    let uv = v2(lo.x + dx * i as f64, lo.y + dy * j as f64);
                    b.add(face.surface.from_chart(&face.chart, uv));
                }
            }
            b = b.inflated(dx.hypot(dy) / 2.0);
        }
        b
    }

    /// Bounds of the face boundary in its chart.
    pub fn face_chart_bounds(&self, f: usize) -> (Vec2, Vec2) {
        let face = &self.faces[f];
        let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for lp in &face.loops {
            for &c in lp {
                for i in 0..=16 {
                    let uv = face.surface.to_chart(&face.chart, self.coedge_point(c, i as f64 / 16.0));
                    lo = v2(lo.x.min(uv.x), lo.y.min(uv.y));
                    hi = v2(hi.x.max(uv.x), hi.y.max(uv.y));
                }
            }
        }
        (lo, hi)
    }

    pub fn aabb(&self) -> Aabb {
        let mut b = Aabb::EMPTY;
        for f in 0..self.faces.len() {
            b.union(&self.face_aabb(f));
        }
        b
    }
}
