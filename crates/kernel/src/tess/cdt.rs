//! Constrained Delaunay triangulation on an integer grid.
//!
//! Coordinates are quantised to a 2^28 grid, which makes the orientation and
//! in-circle predicates exact in `i128`. There are no floating-point predicate
//! failures to work around: the triangulation cannot get confused, only reject
//! truly crossing constraints.

use std::collections::HashMap;

pub const GRID: i64 = 1 << 28;
const NONE: u32 = u32::MAX;

#[derive(Clone, Debug)]
struct Tri {
    v: [u32; 3],
    /// `n[i]` is the neighbour across the edge opposite `v[i]`.
    n: [u32; 3],
    /// `c[i]` marks the edge opposite `v[i]` as a constraint.
    c: [bool; 3],
    inside: bool,
}

pub struct Cdt {
    pub pts: Vec<[i64; 2]>,
    tris: Vec<Tri>,
    last: u32,
    /// Some triangle touching each vertex.
    vertex_tri: Vec<u32>,
    lookup: HashMap<[i64; 2], u32>,
    walk_seed: u32,
}

/// Two constraints cross, so the input polygon is not simple at this resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crossing;

fn orient(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i128 {
    (b[0] - a[0]) as i128 * (c[1] - a[1]) as i128 - (b[1] - a[1]) as i128 * (c[0] - a[0]) as i128
}

/// Positive when `d` lies inside the circumcircle of the counter-clockwise
/// triangle `a b c`. With the grid and super-triangle sizes used here, every
/// intermediate fits in `i128`.
fn incircle(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> i128 {
    let (adx, ady) = ((a[0] - d[0]) as i128, (a[1] - d[1]) as i128);
    let (bdx, bdy) = ((b[0] - d[0]) as i128, (b[1] - d[1]) as i128);
    let (cdx, cdy) = ((c[0] - d[0]) as i128, (c[1] - d[1]) as i128);
    let alift = adx * adx + ady * ady;
    let blift = bdx * bdx + bdy * bdy;
    let clift = cdx * cdx + cdy * cdy;
    alift * (bdx * cdy - cdx * bdy) + blift * (cdx * ady - adx * cdy) + clift * (adx * bdy - bdx * ady)
}

impl Cdt {
    /// Empty triangulation covering `[0, GRID]^2`.
    pub fn new() -> Cdt {
        let s = GRID;
        let pts = vec![[-s, -s], [4 * s, -s], [-s, 4 * s]];
        Cdt {
            pts,
            tris: vec![Tri {
                v: [0, 1, 2],
                n: [NONE; 3],
                c: [false; 3],
                inside: false,
            }],
            last: 0,
            vertex_tri: vec![0, 0, 0],
            lookup: HashMap::new(),
            walk_seed: 12345,
        }
    }

    fn p(&self, v: u32) -> [i64; 2] {
        self.pts[v as usize]
    }

    /// Finds the triangle containing `q`.
    fn locate(&mut self, q: [i64; 2]) -> u32 {
        let mut t = self.last;
        let mut guard = self.tris.len() * 4 + 16;
        'walk: while guard > 0 {
            guard -= 1;
            self.walk_seed = self.walk_seed.wrapping_mul(1103515245).wrapping_add(12345);
            let start = (self.walk_seed >> 16) as usize % 3;
            let tri = &self.tris[t as usize];
            for k in 0..3 {
                let i = (start + k) % 3;
                let a = self.p(tri.v[(i + 1) % 3]);
                let b = self.p(tri.v[(i + 2) % 3]);
                if orient(a, b, q) < 0 && tri.n[i] != NONE {
                    t = tri.n[i];
                    continue 'walk;
                }
            }
            return t;
        }
        // A stochastic walk always terminates in a triangulation, but stay
        // defensive: a linear scan is slow and cannot fail.
        for (i, tri) in self.tris.iter().enumerate() {
            let inside = (0..3).all(|k| {
                orient(self.p(tri.v[(k + 1) % 3]), self.p(tri.v[(k + 2) % 3]), q) >= 0
            });
            if inside {
                return i as u32;
            }
        }
        t
    }

    fn set_back_pointer(&mut self, t: u32, old: u32, new: u32) {
        if t == NONE {
            return;
        }
        let tri = &mut self.tris[t as usize];
        for i in 0..3 {
            if tri.n[i] == old {
                tri.n[i] = new;
                return;
            }
        }
    }

    fn touch(&mut self, t: u32) {
        for &v in &self.tris[t as usize].v {
            self.vertex_tri[v as usize] = t;
        }
        self.last = t;
    }

    /// Inserts a point and returns its vertex id. Returns `None` if the point lies
    /// on a constraint and `allow_on_constraint` is false.
    pub fn insert(&mut self, q: [i64; 2], allow_on_constraint: bool) -> Option<u32> {
        if let Some(&v) = self.lookup.get(&q) {
            return Some(v);
        }
        let t = self.locate(q);
        let tri = self.tris[t as usize].clone();
        let mut on_edge = None;
        for i in 0..3 {
            let a = self.p(tri.v[(i + 1) % 3]);
            let b = self.p(tri.v[(i + 2) % 3]);
            if orient(a, b, q) == 0 {
                on_edge = Some(i);
            }
        }
        if let Some(i) = on_edge {
            if tri.c[i] && !allow_on_constraint {
                return None;
            }
        }
        let p = self.pts.len() as u32;
        self.pts.push(q);
        self.vertex_tri.push(t);
        self.lookup.insert(q, p);

        let mut stack = Vec::new();
        match on_edge {
            None => {
                let [a, b, c] = tri.v;
                let [na, nb, nc] = tri.n;
                let [ca, cb, cc] = tri.c;
                let (t0, t1, t2) = (t, self.tris.len() as u32, self.tris.len() as u32 + 1);
                let inside = tri.inside;
                self.tris[t0 as usize] = Tri {
                    v: [p, b, c],
                    n: [na, t1, t2],
                    c: [ca, false, false],
                    inside,
                };
                self.tris.push(Tri {
                    v: [p, c, a],
                    n: [nb, t2, t0],
                    c: [cb, false, false],
                    inside,
                });
                self.tris.push(Tri {
                    v: [p, a, b],
                    n: [nc, t0, t1],
                    c: [cc, false, false],
                    inside,
                });
                self.set_back_pointer(nb, t, t1);
                self.set_back_pointer(nc, t, t2);
                for x in [t0, t1, t2] {
                    self.touch(x);
                }
                stack.extend([t0, t1, t2]);
            }
            Some(i) => {
                let u = tri.n[i];
                let a = tri.v[i];
                let b = tri.v[(i + 1) % 3];
                let c = tri.v[(i + 2) % 3];
                let nb = tri.n[(i + 1) % 3]; // across c-a
                let nc = tri.n[(i + 2) % 3]; // across a-b
                let cb = tri.c[(i + 1) % 3];
                let cc = tri.c[(i + 2) % 3];
                let c0 = tri.c[i];
                let ut = self.tris[u as usize].clone();
                let j = (0..3).find(|&k| ut.n[k] == t).unwrap();
                let d = ut.v[j];
                // `u` holds d, c, b counter-clockwise.
                let n_bd = ut.n[(j + 1) % 3]; // opposite c: across b-d
                let n_dc = ut.n[(j + 2) % 3]; // opposite b: across d-c
                let c_bd = ut.c[(j + 1) % 3];
                let c_dc = ut.c[(j + 2) % 3];
                let (t1, t2, u1, u2) = (t, self.tris.len() as u32, u, self.tris.len() as u32 + 1);
                let (ti, ui) = (tri.inside, ut.inside);
                self.tris[t1 as usize] = Tri {
                    v: [p, a, b],
                    n: [nc, u2, t2],
                    c: [cc, c0, false],
                    inside: ti,
                };
                self.tris.push(Tri {
                    v: [p, c, a],
                    n: [nb, t1, u1],
                    c: [cb, false, c0],
                    inside: ti,
                });
                self.tris[u1 as usize] = Tri {
                    v: [p, d, c],
                    n: [n_dc, t2, u2],
                    c: [c_dc, c0, false],
                    inside: ui,
                };
                self.tris.push(Tri {
                    v: [p, b, d],
                    n: [n_bd, u1, t1],
                    c: [c_bd, false, c0],
                    inside: ui,
                });
                self.set_back_pointer(nb, t, t2);
                self.set_back_pointer(n_bd, u, u2);
                for x in [t1, t2, u1, u2] {
                    self.touch(x);
                }
                stack.extend([t1, t2, u1, u2]);
            }
        }
        // Lawson flips. Every triangle on the stack has the new point at v[0].
        while let Some(x) = stack.pop() {
            let tx = self.tris[x as usize].clone();
            let y = tx.n[0];
            if y == NONE || tx.c[0] {
                continue;
            }
            let ty = self.tris[y as usize].clone();
            let Some(j) = (0..3).find(|&k| ty.n[k] == x) else {
                continue;
            };
            let d = ty.v[j];
            if incircle(self.p(tx.v[0]), self.p(tx.v[1]), self.p(tx.v[2]), self.p(d)) <= 0 {
                continue;
            }
            let (b, c) = (tx.v[1], tx.v[2]);
            let n_bd = ty.n[(j + 1) % 3];
            let n_dc = ty.n[(j + 2) % 3];
            let c_bd = ty.c[(j + 1) % 3];
            let c_dc = ty.c[(j + 2) % 3];
            self.tris[x as usize] = Tri {
                v: [p, b, d],
                n: [n_bd, y, tx.n[2]],
                c: [c_bd, false, tx.c[2]],
                inside: tx.inside,
            };
            self.tris[y as usize] = Tri {
                v: [p, d, c],
                n: [n_dc, tx.n[1], x],
                c: [c_dc, tx.c[1], false],
                inside: tx.inside,
            };
            self.set_back_pointer(n_bd, y, x);
            self.set_back_pointer(tx.n[1], x, y);
            self.touch(x);
            self.touch(y);
            stack.push(x);
            stack.push(y);
        }
        Some(p)
    }

    /// The triangles around vertex `a`, starting anywhere.
    fn fan(&self, a: u32) -> Vec<u32> {
        let start = self.vertex_tri[a as usize];
        let mut out = vec![start];
        let mut cur = start;
        // Rotate one way, then the other in case the fan is open on the hull.
        loop {
            let tri = &self.tris[cur as usize];
            let i = (0..3).find(|&k| tri.v[k] == a).unwrap();
            let next = tri.n[(i + 1) % 3];
            if next == NONE || next == start {
                if next == NONE {
                    let mut cur = start;
                    loop {
                        let tri = &self.tris[cur as usize];
                        let i = (0..3).find(|&k| tri.v[k] == a).unwrap();
                        let prev = tri.n[(i + 2) % 3];
                        if prev == NONE {
                            break;
                        }
                        out.push(prev);
                        cur = prev;
                    }
                }
                break;
            }
            out.push(next);
            cur = next;
        }
        out
    }

    fn mark_edge(&mut self, a: u32, b: u32) -> bool {
        let mut found = false;
        for t in self.fan(a) {
            let tri = &self.tris[t as usize];
            for i in 0..3 {
                let (x, y) = (tri.v[(i + 1) % 3], tri.v[(i + 2) % 3]);
                if (x == a && y == b) || (x == b && y == a) {
                    self.tris[t as usize].c[i] = true;
                    found = true;
                    break;
                }
            }
        }
        found
    }

    /// Makes the segment between two vertices an edge of the triangulation.
    pub fn constrain(&mut self, a: u32, b: u32) -> Result<(), Crossing> {
        if a == b {
            return Ok(());
        }
        let mut a = a;
        let mut guard = 0;
        while a != b {
            guard += 1;
            if guard > 100_000 {
                return Err(Crossing);
            }
            if self.mark_edge(a, b) {
                return Ok(());
            }
            let (pa, pb) = (self.p(a), self.p(b));
            // Find the triangle at `a` whose far edge the segment passes through,
            // or a vertex lying exactly on the segment.
            let fan = self.fan(a);
            let mut crossing: Option<(u32, u32)> = None;
            let mut through: Option<u32> = None;
            for t in fan {
                let tri = &self.tris[t as usize];
                let i = (0..3).find(|&k| tri.v[k] == a).unwrap();
                let (x, y) = (tri.v[(i + 1) % 3], tri.v[(i + 2) % 3]);
                let (sx, sy) = (orient(pa, pb, self.p(x)), orient(pa, pb, self.p(y)));
                let forward = |q: [i64; 2]| {
                    (q[0] - pa[0]) as i128 * (pb[0] - pa[0]) as i128 + (q[1] - pa[1]) as i128 * (pb[1] - pa[1]) as i128 > 0
                };
                if sx == 0 && forward(self.p(x)) {
                    through = Some(x);
                    break;
                }
                if sy == 0 && forward(self.p(y)) {
                    through = Some(y);
                    break;
                }
                if sx < 0 && sy > 0 {
                    crossing = Some((x, y));
                    break;
                }
            }
            if let Some(v) = through {
                self.mark_edge(a, v);
                a = v;
                continue;
            }
            let Some(first) = crossing else {
                return Err(Crossing);
            };
            // Walk to the far end collecting every crossed edge, stopping early at
            // a vertex that lies exactly on the segment.
            let mut crossed = vec![first];
            let mut stop = b;
            let (mut x, mut y) = first;
            loop {
                let Some(t) = self.tri_with_directed_edge(y, x) else {
                    return Err(Crossing);
                };
                let tri = &self.tris[t as usize];
                let i = (0..3).find(|&k| tri.v[k] != x && tri.v[k] != y).unwrap();
                if tri.c[i] {
                    return Err(Crossing);
                }
                let z = tri.v[i];
                if z == b {
                    break;
                }
                let s = orient(pa, pb, self.p(z));
                if s == 0 {
                    stop = z;
                    break;
                }
                if s < 0 {
                    x = z;
                } else {
                    y = z;
                }
                crossed.push((x, y));
            }
            // Reject constraint crossings before changing anything.
            for &(p, q) in &crossed {
                if self.edge_is_constrained(p, q) {
                    return Err(Crossing);
                }
            }
            self.recover(a, stop, crossed)?;
            self.mark_edge(a, stop);
            a = stop;
        }
        Ok(())
    }

    fn edge_is_constrained(&self, p: u32, q: u32) -> bool {
        for t in self.fan(p) {
            let tri = &self.tris[t as usize];
            for i in 0..3 {
                let (x, y) = (tri.v[(i + 1) % 3], tri.v[(i + 2) % 3]);
                if ((x == p && y == q) || (x == q && y == p)) && tri.c[i] {
                    return true;
                }
            }
        }
        false
    }

    fn tri_with_directed_edge(&self, x: u32, y: u32) -> Option<u32> {
        for t in self.fan(x) {
            let tri = &self.tris[t as usize];
            let i = (0..3).find(|&k| tri.v[k] == x).unwrap();
            if tri.v[(i + 1) % 3] == y {
                return Some(t);
            }
        }
        None
    }

    /// Flips away every edge crossing `a-b` (Sloan's method).
    fn recover(&mut self, a: u32, b: u32, crossed: Vec<(u32, u32)>) -> Result<(), Crossing> {
        let (pa, pb) = (self.p(a), self.p(b));
        let mut queue: std::collections::VecDeque<(u32, u32)> = crossed.into();
        let mut stalled = 0usize;
        let mut new_edges = Vec::new();
        while let Some((u, v)) = queue.pop_front() {
            let (Some(t1), Some(t2)) = (self.tri_with_directed_edge(u, v), self.tri_with_directed_edge(v, u)) else {
                return Err(Crossing);
            };
            let ta = &self.tris[t1 as usize];
            let w = ta.v[(0..3).find(|&k| ta.v[k] != u && ta.v[k] != v).unwrap()];
            let tb = &self.tris[t2 as usize];
            let x = tb.v[(0..3).find(|&k| tb.v[k] != u && tb.v[k] != v).unwrap()];
            // The quad u x v w must be strictly convex to flip.
            let convex = orient(self.p(w), self.p(x), self.p(u)) != 0
                && orient(self.p(w), self.p(x), self.p(v)) != 0
                && (orient(self.p(w), self.p(x), self.p(u)) > 0) != (orient(self.p(w), self.p(x), self.p(v)) > 0);
            if !convex {
                queue.push_back((u, v));
                stalled += 1;
                if stalled > queue.len() * 2 + 8 {
                    return Err(Crossing);
                }
                continue;
            }
            stalled = 0;
            self.flip(t1, t2, u, v);
            // Does the new diagonal still cross a-b?
            let (sw, sx) = (orient(pa, pb, self.p(w)), orient(pa, pb, self.p(x)));
            let touches_end = w == a || w == b || x == a || x == b;
            if !touches_end && (sw > 0) != (sx > 0) && sw != 0 && sx != 0 {
                queue.push_back((w, x));
            } else {
                new_edges.push((w, x));
            }
        }
        // Restore the Delaunay property on the edges that were just created.
        for (p, q) in new_edges {
            if (p == a && q == b) || (p == b && q == a) || self.edge_is_constrained(p, q) {
                continue;
            }
            let (Some(t1), Some(t2)) = (self.tri_with_directed_edge(p, q), self.tri_with_directed_edge(q, p)) else {
                continue;
            };
            let ta = &self.tris[t1 as usize];
            let i = (0..3).find(|&k| ta.v[k] != p && ta.v[k] != q).unwrap();
            let w = ta.v[i];
            let tb = &self.tris[t2 as usize];
            let x = tb.v[(0..3).find(|&k| tb.v[k] != p && tb.v[k] != q).unwrap()];
            let in_circle = incircle(self.p(p), self.p(q), self.p(w), self.p(x)) > 0;
            let (o1, o2) = (orient(self.p(w), self.p(x), self.p(p)), orient(self.p(w), self.p(x), self.p(q)));
            let separates_ab = (w == a && x == b) || (w == b && x == a);
            if in_circle && o1 != 0 && o2 != 0 && (o1 > 0) != (o2 > 0) && !separates_ab {
                self.flip(t1, t2, p, q);
            }
        }
        Ok(())
    }

    /// Flips the edge shared by `t1` (holding u->v) and `t2` (holding v->u).
    fn flip(&mut self, t1: u32, t2: u32, u: u32, v: u32) {
        let a = self.tris[t1 as usize].clone();
        let b = self.tris[t2 as usize].clone();
        let iu = (0..3).find(|&k| a.v[k] == u).unwrap();
        // t1 is u, v, w. Neighbours opposite u (across v-w) and opposite v (across w-u).
        let w = a.v[(iu + 2) % 3];
        let n_vw = a.n[iu];
        let c_vw = a.c[iu];
        let n_wu = a.n[(iu + 1) % 3];
        let c_wu = a.c[(iu + 1) % 3];
        let jv = (0..3).find(|&k| b.v[k] == v).unwrap();
        // t2 is v, u, x. Neighbours opposite v (across u-x) and opposite u (across x-v).
        let x = b.v[(jv + 2) % 3];
        let n_ux = b.n[jv];
        let c_ux = b.c[jv];
        let n_xv = b.n[(jv + 1) % 3];
        let c_xv = b.c[(jv + 1) % 3];
        // New triangles: t1 = w, u, x and t2 = x, v, w.
        self.tris[t1 as usize] = Tri {
            v: [w, u, x],
            n: [n_ux, t2, n_wu],
            c: [c_ux, false, c_wu],
            inside: a.inside,
        };
        self.tris[t2 as usize] = Tri {
            v: [x, v, w],
            n: [n_vw, t1, n_xv],
            c: [c_vw, false, c_xv],
            inside: b.inside,
        };
        self.set_back_pointer(n_ux, t2, t1);
        self.set_back_pointer(n_vw, t1, t2);
        self.touch(t1);
        self.touch(t2);
    }

    /// Marks triangles inside the domain. Constraints divide the triangulation into
    /// regions, and `is_inside` decides each one from a point well inside it. This
    /// asks the polygon itself, so it stays right where a parity count would not:
    /// at loops that touch, or at boundary pieces that coincide at grid resolution.
    pub fn mark_inside_with(&mut self, is_inside: impl Fn([f64; 2]) -> bool) {
        let n = self.tris.len();
        let mut region = vec![u32::MAX; n];
        let mut next_region = 0u32;
        for start in 0..n {
            if region[start] != u32::MAX {
                continue;
            }
            let id = next_region;
            next_region += 1;
            region[start] = id;
            let mut stack = vec![start as u32];
            let mut members = Vec::new();
            let mut touches_outside = false;
            while let Some(t) = stack.pop() {
                members.push(t);
                let tri = &self.tris[t as usize];
                touches_outside |= tri.v.iter().any(|&v| v < 3);
                for i in 0..3 {
                    let m = tri.n[i];
                    if m != NONE && !tri.c[i] && region[m as usize] == u32::MAX {
                        region[m as usize] = id;
                        stack.push(m);
                    }
                }
            }
            let inside = if touches_outside {
                false
            } else {
                // Decide from the centre of the region's largest triangle.
                let largest = members
                    .iter()
                    .copied()
                    .max_by_key(|&t| {
                        let v = self.tris[t as usize].v;
                        orient(self.p(v[0]), self.p(v[1]), self.p(v[2]))
                    })
                    .unwrap();
                let v = self.tris[largest as usize].v;
                let (a, b, c) = (self.p(v[0]), self.p(v[1]), self.p(v[2]));
                is_inside([
                    (a[0] + b[0] + c[0]) as f64 / 3.0,
                    (a[1] + b[1] + c[1]) as f64 / 3.0,
                ])
            };
            for t in members {
                self.tris[t as usize].inside = inside;
            }
        }
    }

    /// Marks triangles inside a set of closed polygons given as vertex ids: a point
    /// is inside when it has a non-zero total winding.
    pub fn mark_inside(&mut self, loops: &[Vec<u32>]) {
        let polys: Vec<Vec<[f64; 2]>> = loops
            .iter()
            .map(|l| l.iter().map(|&v| [self.p(v)[0] as f64, self.p(v)[1] as f64]).collect())
            .collect();
        self.mark_inside_with(|q| {
            let mut w = 0.0;
            for poly in &polys {
                for i in 0..poly.len() {
                    let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                    let (ux, uy, vx, vy) = (a[0] - q[0], a[1] - q[1], b[0] - q[0], b[1] - q[1]);
                    w += (ux * vy - uy * vx).atan2(ux * vx + uy * vy);
                }
            }
            (w / std::f64::consts::TAU).round() != 0.0
        });
    }

    /// Triangles inside the domain as `(vertices, constrained edges)`. Vertex ids
    /// index `pts`; id 3 is the first real point.
    pub fn inside_triangles(&self) -> impl Iterator<Item = ([u32; 3], [bool; 3])> + '_ {
        self.tris.iter().filter(|t| t.inside).map(|t| (t.v, t.c))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rng;

    fn area2(c: &Cdt, v: [u32; 3]) -> i128 {
        orient(c.pts[v[0] as usize], c.pts[v[1] as usize], c.pts[v[2] as usize])
    }

    #[test]
    fn square_with_hole_has_exact_area() {
        let g = GRID / 8;
        let mut c = Cdt::new();
        let outer = [[0, 0], [8 * g, 0], [8 * g, 8 * g], [0, 8 * g]];
        let hole = [[3 * g, 3 * g], [3 * g, 5 * g], [5 * g, 5 * g], [5 * g, 3 * g]];
        let mut loops = Vec::new();
        for poly in [&outer, &hole] {
            let ids: Vec<u32> = poly.iter().map(|&p| c.insert(p, true).unwrap()).collect();
            for i in 0..4 {
                c.constrain(ids[i], ids[(i + 1) % 4]).unwrap();
            }
            loops.push(ids);
        }
        c.mark_inside(&loops);
        let total: i128 = c.inside_triangles().map(|(v, _)| area2(&c, v)).sum();
        assert_eq!(total, 2 * (64 - 4) * (g as i128) * (g as i128));
    }

    #[test]
    fn random_polygon_area_is_preserved_with_steiner_points() {
        let mut rng = Rng::new(3);
        for _ in 0..200 {
            // A star-shaped polygon around the middle of the grid.
            let n = rng.int(3, 40) as usize;
            let mut poly = Vec::new();
            for i in 0..n {
                let a = i as f64 / n as f64 * std::f64::consts::TAU;
                let r = rng.range(0.1, 0.48) * GRID as f64;
                poly.push([
                    (GRID as f64 / 2.0 + r * a.cos()) as i64,
                    (GRID as f64 / 2.0 + r * a.sin()) as i64,
                ]);
            }
            let mut exact = 0i128;
            for i in 0..n {
                let (p, q) = (poly[i], poly[(i + 1) % n]);
                exact += p[0] as i128 * q[1] as i128 - q[0] as i128 * p[1] as i128;
            }
            let mut c = Cdt::new();
            let ids: Vec<u32> = poly.iter().map(|&p| c.insert(p, true).unwrap()).collect();
            for i in 0..n {
                c.constrain(ids[i], ids[(i + 1) % n]).unwrap();
            }
            c.mark_inside(std::slice::from_ref(&ids));
            for _ in 0..50 {
                let q = [(rng.f64() * GRID as f64) as i64, (rng.f64() * GRID as f64) as i64];
                let _ = c.insert(q, false);
            }
            c.mark_inside(std::slice::from_ref(&ids));
            let total: i128 = c.inside_triangles().map(|(v, _)| area2(&c, v)).sum();
            assert_eq!(total, exact);
        }
    }

    #[test]
    fn crossing_constraints_are_reported() {
        let g = GRID / 4;
        let mut c = Cdt::new();
        let a = c.insert([0, 0], true).unwrap();
        let b = c.insert([4 * g, 4 * g], true).unwrap();
        let d = c.insert([0, 4 * g], true).unwrap();
        let e = c.insert([4 * g, 0], true).unwrap();
        c.constrain(a, b).unwrap();
        assert_eq!(c.constrain(d, e), Err(Crossing));
    }
}
