//! A view of a body, as a drawing shows it: each edge projected onto the view's plane and cut into what's seen and what's
//! hidden behind the body, the outlines curved faces show where they turn away, the cut faces of a section, and the
//! circles seen face on and the axes seen side on, for center marks and lines.
//!
//! What hides what is worked out against a fine mesh of the body, in the view's plane, so a point counts as hidden when
//! some triangle is in front of it by more than twice the mesh's own error. The edges themselves are exact, from the
//! body. An outline where a curved face turns away is found from the mesh, then moved onto the exact line on a cylinder,
//! cone or sphere.

use coreshape::math::*;
use coreshape::{tessellate, Body, Curve, Mesh, Surface, TessParams};
use std::collections::HashMap;

/// Which way a view looks: `toward` points at the viewer, and `right` and `up` lie across the sheet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewFrame {
    pub right: Vec3,
    pub up: Vec3,
    pub toward: Vec3,
}

impl ViewFrame {
    /// A view looking along `-toward`, with `up` as near as it can be to the given direction.
    pub fn looking(toward: Vec3, up_hint: Vec3) -> ViewFrame {
        let toward = toward.normalized();
        let mut up = up_hint - toward * up_hint.dot(toward);
        if up.norm() < 1e-9 {
            up = toward.any_perpendicular();
        }
        let up = up.normalized();
        ViewFrame { right: up.cross(toward), up, toward }
    }

    pub fn flat(&self, p: Vec3) -> Vec2 {
        v2(p.dot(self.right), p.dot(self.up))
    }

    fn depth(&self, p: Vec3) -> f64 {
        p.dot(self.toward)
    }
}

/// A circle seen face on: where its center is in the view, its radius, and its center in the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle {
    pub center: Vec2,
    pub radius: f64,
    pub at: Vec3,
    /// How much of the circle the body's edges go round, in radians.
    pub sweep: f64,
}

/// What a view shows, in the model's units, in the view's plane.
#[derive(Clone, Debug, Default)]
pub struct ViewLines {
    pub visible: Vec<Vec<Vec2>>,
    pub hidden: Vec<Vec<Vec2>>,
    /// Edges between faces that meet smoothly, as at a fillet: drawn thin, and only where they're seen.
    pub tangent: Vec<Vec<Vec2>>,
    /// The cut faces of a section, as triangles, for hatching.
    pub cut: Vec<[Vec2; 3]>,
    pub circles: Vec<Circle>,
    /// Axes of round faces seen side on, from end to end.
    pub axes: Vec<(Vec2, Vec2)>,
    pub lo: Vec2,
    pub hi: Vec2,
}

/// The faces a section's cut made, by the tag the cut gives them.
pub const CUT_FEATURE: u32 = u32::MAX - 31;

/// Triangles in the view's plane, bucketed in a grid, with the depth of each corner.
struct Screen {
    tris: Vec<([Vec2; 3], [f64; 3])>,
    lo: Vec2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl Screen {
    fn new(mesh: &Mesh, view: &ViewFrame) -> Screen {
        let mut tris = Vec::with_capacity(mesh.triangles.len());
        let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for t in &mesh.triangles {
            let p = [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]];
            let q = [view.flat(p[0]), view.flat(p[1]), view.flat(p[2])];
            // Seen edge on, a triangle covers nothing.
            let area = (q[1] - q[0]).cross(q[2] - q[0]);
            let size = (q[1] - q[0]).norm().max((q[2] - q[0]).norm());
            if !(area.abs() > 1e-12 * size * size) {
                continue;
            }
            for v in q {
                lo = v2(lo.x.min(v.x), lo.y.min(v.y));
                hi = v2(hi.x.max(v.x), hi.y.max(v.y));
            }
            tris.push((q, [view.depth(p[0]), view.depth(p[1]), view.depth(p[2])]));
        }
        if tris.is_empty() {
            return Screen { tris, lo: v2(0.0, 0.0), cell: 1.0, nx: 1, ny: 1, cells: vec![Vec::new()] };
        }
        let side = ((tris.len() as f64).sqrt() * 1.5).ceil().clamp(1.0, 600.0);
        let cell = ((hi.x - lo.x).max(hi.y - lo.y) / side).max(1e-9);
        let (nx, ny) = (((hi.x - lo.x) / cell).floor() as usize + 1, ((hi.y - lo.y) / cell).floor() as usize + 1);
        let mut cells = vec![Vec::new(); nx * ny];
        for (i, (q, _)) in tris.iter().enumerate() {
            let (mut a, mut b) = (q[0], q[0]);
            for v in &q[1..] {
                a = v2(a.x.min(v.x), a.y.min(v.y));
                b = v2(b.x.max(v.x), b.y.max(v.y));
            }
            let (x0, x1) = (((a.x - lo.x) / cell) as usize, (((b.x - lo.x) / cell) as usize).min(nx - 1));
            let (y0, y1) = (((a.y - lo.y) / cell) as usize, (((b.y - lo.y) / cell) as usize).min(ny - 1));
            for y in y0..=y1 {
                for x in x0..=x1 {
                    cells[y * nx + x].push(i as u32);
                }
            }
        }
        Screen { tris, lo, cell, nx, ny, cells }
    }

    /// Whether some triangle is in front of this point, by more than `margin`.
    fn covered(&self, at: Vec2, depth: f64, margin: f64) -> bool {
        let (x, y) = ((at.x - self.lo.x) / self.cell, (at.y - self.lo.y) / self.cell);
        if x < 0.0 || y < 0.0 {
            return false;
        }
        let (x, y) = (x as usize, y as usize);
        if x >= self.nx || y >= self.ny {
            return false;
        }
        for &i in &self.cells[y * self.nx + x] {
            let (q, d) = &self.tris[i as usize];
            let (e1, e2, r) = (q[1] - q[0], q[2] - q[0], at - q[0]);
            let det = e1.cross(e2);
            let (s, t) = (r.cross(e2) / det, e1.cross(r) / det);
            // On its edges too: a point on a line between two triangles in front of it is behind both. A triangle of the
            // point's own face is at its depth, which the margin leaves alone.
            let rim = 1e-9;
            if s < -rim || t < -rim || s + t > 1.0 + rim {
                continue;
            }
            let there = d[0] + (d[1] - d[0]) * s + (d[2] - d[0]) * t;
            if there > depth + margin {
                return true;
            }
        }
        false
    }
}

/// Whether two faces' surfaces are the same, so that an edge between them is only where a round face was cut in two.
fn same_surface(a: &Surface, b: &Surface) -> bool {
    let near = |x: f64, y: f64| (x - y).abs() <= 1e-9 * x.abs().max(y.abs()).max(1.0);
    let frames = |f: &Frame, g: &Frame| f.origin.dist(g.origin) <= 1e-9 * f.origin.norm().max(1.0) && f.z.cross(g.z).norm() < 1e-9;
    match (a, b) {
        (Surface::Plane(f), Surface::Plane(g)) => f.z.cross(g.z).norm() < 1e-9 && (g.origin - f.origin).dot(f.z).abs() < 1e-9,
        (Surface::Cylinder { frame: f, radius: r }, Surface::Cylinder { frame: g, radius: s }) => {
            near(*r, *s) && f.z.cross(g.z).norm() < 1e-9 && { let d = g.origin - f.origin; (d - f.z * d.dot(f.z)).norm() < 1e-9 * r.max(1.0) }
        }
        (Surface::Sphere { frame: f, radius: r }, Surface::Sphere { frame: g, radius: s }) => near(*r, *s) && f.origin.dist(g.origin) < 1e-9 * r.max(1.0),
        (Surface::Cone { frame: f, radius: r, half_angle: h }, Surface::Cone { frame: g, radius: s, half_angle: k }) => frames(f, g) && near(*r, *s) && near(*h, *k),
        (Surface::Torus { frame: f, major: a1, minor: b1 }, Surface::Torus { frame: g, major: a2, minor: b2 }) => frames(f, g) && near(*a1, *a2) && near(*b1, *b2),
        (Surface::Free { patch: p, .. }, Surface::Free { patch: q, .. }) => std::sync::Arc::ptr_eq(p, q),
        (Surface::Extruded { curve: p, .. }, Surface::Extruded { curve: q, .. }) | (Surface::Revolved { curve: p, .. }, Surface::Revolved { curve: q, .. }) => std::sync::Arc::ptr_eq(p, q),
        _ => false,
    }
}

/// A point on a face's surface moved onto the exact outline that surface shows from this view, where there's a formula.
fn onto_outline(surface: &Surface, p: Vec3, toward: Vec3) -> Vec3 {
    match surface {
        Surface::Cylinder { frame, radius } => {
            let t = (p - frame.origin).dot(frame.z);
            let side = frame.z.cross(toward);
            if side.norm() < 1e-9 {
                return p;
            }
            let side = side.normalized();
            let sign = if (p - frame.origin).dot(side) >= 0.0 { 1.0 } else { -1.0 };
            frame.origin + frame.z * t + side * (sign * radius)
        }
        Surface::Sphere { frame, radius } => {
            let r = p - frame.origin;
            let flat = r - toward * r.dot(toward);
            if flat.norm() < 1e-12 {
                return p;
            }
            frame.origin + flat.normalized() * *radius
        }
        _ => p,
    }
}

/// Splits a run of points along a curve into what's seen and what's hidden, adding each piece to `seen` or `hidden`.
/// Between samples whose answers differ, the change is found by halving. Points closer together than `step` in the
/// view aren't sampled further.
fn split_by_sight(points: &[Vec3], view: &ViewFrame, screen: &Screen, margin: f64, step: f64, seen: &mut Vec<Vec<Vec2>>, hidden: Option<&mut Vec<Vec<Vec2>>>) {
    if points.len() < 2 {
        return;
    }
    let hidden_at = |p: Vec3| screen.covered(view.flat(p), view.depth(p), margin);
    // Every sample, close enough together in the view.
    let mut samples: Vec<Vec3> = vec![points[0]];
    for w in points.windows(2) {
        let n = ((view.flat(w[1]) - view.flat(w[0])).norm() / step).ceil().clamp(1.0, 2000.0) as usize;
        for k in 1..=n {
            samples.push(w[0] + (w[1] - w[0]) * (k as f64 / n as f64));
        }
    }
    let states: Vec<bool> = samples.iter().map(|p| hidden_at(*p)).collect();
    let mut runs: Vec<(bool, Vec<Vec2>)> = Vec::new();
    let mut current: Vec<Vec2> = vec![view.flat(samples[0])];
    let mut state = states[0];
    for i in 1..samples.len() {
        if states[i] == state {
            current.push(view.flat(samples[i]));
            continue;
        }
        // Where it changes, between this sample and the last.
        let (mut a, mut b) = (samples[i - 1], samples[i]);
        for _ in 0..24 {
            let m = (a + b) * 0.5;
            if hidden_at(m) == state {
                a = m;
            } else {
                b = m;
            }
        }
        let at = view.flat((a + b) * 0.5);
        current.push(at);
        runs.push((state, std::mem::take(&mut current)));
        current.push(at);
        current.push(view.flat(samples[i]));
        state = states[i];
    }
    runs.push((state, current));
    let mut hidden = hidden;
    for (is_hidden, run) in runs {
        // Too short to draw.
        let length: f64 = run.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        if run.len() < 2 || length < step * 0.05 {
            continue;
        }
        if is_hidden {
            if let Some(h) = hidden.as_deref_mut() {
                h.push(run);
            }
        } else {
            seen.push(run);
        }
    }
}

/// Whether every point of a run, and the middle of each of its pieces, is within `tol` of one of these lines.
fn lies_on(run: &[Vec2], lines: &[Vec<Vec2>], tol: f64) -> bool {
    let near = |p: Vec2| {
        lines.iter().any(|l| {
            l.windows(2).any(|w| {
                let (a, b) = (w[0], w[1]);
                let d = b - a;
                let len2 = d.dot(d);
                let t = if len2 > 0.0 { ((p - a).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
                p.dist(a + d * t) <= tol
            })
        })
    };
    run.iter().all(|p| near(*p)) && run.windows(2).all(|w| near((w[0] + w[1]) * 0.5))
}

/// How fine a body's mesh is for a drawing of it, in the model's units: a size's ten-thousandth, and no coarser than
/// what's asked.
pub fn mesh_chord(body: &Body, finest: f64) -> f64 {
    (body.aabb().diagonal() * 1e-4).max(finest).max(1e-6)
}

/// A body as a view shows it. `chord` is how far the mesh used for what hides what may be from the surfaces; `hidden`
/// says whether to keep hidden lines at all.
pub fn project(body: &Body, view: &ViewFrame, chord: f64, hidden: bool) -> coreshape::Result<ViewLines> {
    let mesh = tessellate(body, &TessParams { chord, angle: 8f64.to_radians() })?;
    let screen = Screen::new(&mesh, view);
    let size = body.aabb().diagonal().max(1e-9);
    let margin = (2.0 * chord).max(1e-7 * size);
    let mut out = ViewLines::default();
    let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for p in &mesh.positions {
        let q = view.flat(*p);
        lo = v2(lo.x.min(q.x), lo.y.min(q.y));
        hi = v2(hi.x.max(q.x), hi.y.max(q.y));
    }
    out.lo = lo;
    out.hi = hi;
    let step = (hi - lo).norm().max(1e-9) / 600.0;
    // The edges, each as the mesh traced it, which is on the edge itself.
    let uses = body.edge_uses();
    for (e, line) in mesh.edge_lines.iter().enumerate() {
        if e >= body.edges.len() || line.len() < 2 {
            continue;
        }
        let faces: Vec<usize> = uses[e].iter().map(|u| u.0).collect();
        // Smooth, or not an edge at all where a round face was only cut in two.
        let mid = line[line.len() / 2];
        let normal_of = |f: usize| {
            let face = &body.faces[f];
            let n = face.surface.normal(face.surface.project(mid));
            if face.flipped {
                -n
            } else {
                n
            }
        };
        let smooth = faces.len() == 2 && faces[0] != faces[1] && normal_of(faces[0]).dot(normal_of(faces[1])) > 1.0 - 1e-6;
        // A smooth edge the view sees side on all along is an outline, as where a round face was cut in two right where it
        // turns away: drawn as one. Otherwise a smooth edge within one surface isn't drawn, and one between two is thin.
        let outline = smooth && [0.2, 0.5, 0.8].iter().all(|s| {
            let at = line[((line.len() - 1) as f64 * s).round() as usize];
            let face = &body.faces[faces[0]];
            face.surface.normal(face.surface.project(at)).dot(view.toward).abs() < 1e-6
        });
        if smooth && !outline && same_surface(&body.faces[faces[0]].surface, &body.faces[faces[1]].surface) {
            continue;
        }
        if smooth && !outline {
            split_by_sight(line, view, &screen, margin, step, &mut out.tangent, None);
        } else {
            split_by_sight(line, view, &screen, margin, step, &mut out.visible, hidden.then_some(&mut out.hidden));
        }
        // A circle seen face on.
        if let Curve::Circle { frame, radius } = &body.edges[e].curve {
            if frame.z.dot(view.toward).abs() > 1.0 - 1e-9 {
                let center = view.flat(frame.origin);
                let edge = &body.edges[e];
                let sweep = (edge.t1 - edge.t0).abs();
                match out.circles.iter_mut().find(|c| c.center.dist(center) < 1e-9 * size && (c.radius - radius).abs() < 1e-9 * size) {
                    Some(c) => c.sweep += sweep,
                    None => out.circles.push(Circle { center, radius: *radius, at: frame.origin, sweep }),
                }
            }
        }
    }
    // Outlines: where a curved face turns from facing the viewer to facing away, which is where its true normal, at the
    // mesh's points, is square to the view. Across each triangle whose corners differ in that, the line where it's
    // nothing, from edge to edge, as a contour is traced; then joined up by the mesh edges they cross.
    let facing: Vec<f64> = mesh.normals.iter().map(|n| n.dot(view.toward)).collect();
    let mut pieces: Vec<((u32, u32), (u32, u32), usize)> = Vec::new();
    let mut point_on: HashMap<(u32, u32), Vec3> = HashMap::new();
    for (t, tri) in mesh.triangles.iter().enumerate() {
        let face = mesh.face_of_triangle[t] as usize;
        if matches!(body.faces[face].surface, Surface::Plane(_)) {
            continue;
        }
        let ahead = |v: u32| facing[v as usize] >= 0.0;
        if ahead(tri[0]) == ahead(tri[1]) && ahead(tri[1]) == ahead(tri[2]) {
            continue;
        }
        let mut crossed = Vec::with_capacity(2);
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            if ahead(a) != ahead(b) {
                let key = (a.min(b), a.max(b));
                let (fa, fb) = (facing[key.0 as usize], facing[key.1 as usize]);
                let (pa, pb) = (mesh.positions[key.0 as usize], mesh.positions[key.1 as usize]);
                point_on.entry(key).or_insert_with(|| pa + (pb - pa) * (fa / (fa - fb)));
                crossed.push(key);
            }
        }
        if crossed.len() == 2 {
            pieces.push((crossed[0], crossed[1], face));
        }
    }
    // Joined into runs, end to end, on each face.
    let mut next: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, (a, b, _)) in pieces.iter().enumerate() {
        next.entry(*a).or_default().push(i);
        next.entry(*b).or_default().push(i);
    }
    let mut used = vec![false; pieces.len()];
    for start in 0..pieces.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let face = pieces[start].2;
        let mut run: Vec<(u32, u32)> = vec![pieces[start].0, pieces[start].1];
        for forward in [true, false] {
            loop {
                let end = if forward { run[run.len() - 1] } else { run[0] };
                let Some(&k) = next.get(&end).and_then(|list| list.iter().find(|&&k| !used[k] && pieces[k].2 == face)) else { break };
                used[k] = true;
                let other = if pieces[k].0 == end { pieces[k].1 } else { pieces[k].0 };
                if forward {
                    run.push(other);
                } else {
                    run.insert(0, other);
                }
            }
        }
        let surface = &body.faces[face].surface;
        let points: Vec<Vec3> = run.iter().map(|k| onto_outline(surface, point_on[k], view.toward)).collect();
        split_by_sight(&points, view, &screen, margin, step, &mut out.visible, hidden.then_some(&mut out.hidden));
    }
    // A line drawn twice, or a hidden one lying under one that's seen, as a back face's edge behind its front's, is drawn
    // once, as seen.
    let tol = 1e-6 * size;
    let mut kept: Vec<Vec<Vec2>> = Vec::new();
    for run in std::mem::take(&mut out.visible) {
        if !lies_on(&run, &kept, tol) {
            kept.push(run);
        }
    }
    out.visible = kept;
    let mut kept_hidden: Vec<Vec<Vec2>> = Vec::new();
    for run in std::mem::take(&mut out.hidden) {
        if !lies_on(&run, &out.visible, tol) && !lies_on(&run, &kept_hidden, tol) {
            kept_hidden.push(run);
        }
    }
    out.hidden = kept_hidden;
    // Axes of round faces seen side on, from end to end of the face.
    for (f, face) in body.faces.iter().enumerate() {
        let (Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. }) = &face.surface else { continue };
        if frame.z.dot(view.toward).abs() > 1e-9 {
            continue;
        }
        let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
        for c in face.loops.iter().flatten() {
            for k in 0..=8 {
                let t = (body.coedge_point(*c, k as f64 / 8.0) - frame.origin).dot(frame.z);
                t0 = t0.min(t);
                t1 = t1.max(t);
            }
        }
        if !(t1 > t0) {
            continue;
        }
        let (a, b) = (view.flat(frame.origin + frame.z * t0), view.flat(frame.origin + frame.z * t1));
        let fits = |x: &(Vec2, Vec2)| (x.0.dist(a) < 1e-9 * size && x.1.dist(b) < 1e-9 * size) || (x.0.dist(b) < 1e-9 * size && x.1.dist(a) < 1e-9 * size);
        if !out.axes.iter().any(fits) {
            out.axes.push((a, b));
        }
        let _ = f;
    }
    // The cut faces of a section, as the mesh has them.
    for (t, tri) in mesh.triangles.iter().enumerate() {
        let face = mesh.face_of_triangle[t] as usize;
        if body.faces[face].tag.feature == CUT_FEATURE {
            out.cut.push([view.flat(mesh.positions[tri[0] as usize]), view.flat(mesh.positions[tri[1] as usize]), view.flat(mesh.positions[tri[2] as usize])]);
        }
    }
    Ok(out)
}

/// A body cut by a plane through `point`, the half `normal` points to taken away, for a section seen looking back along
/// `normal`. The faces the cut makes carry `CUT_FEATURE` in their tags, to be hatched.
pub fn section(body: &Body, point: Vec3, normal: Vec3) -> coreshape::Result<Body> {
    let n = normal.normalized();
    let size = body.aabb().diagonal().max(1.0) * 4.0;
    let center = body.aabb().center();
    // A box over all of the half that goes, its near face on the plane.
    let along = center - n * (center - point).dot(n);
    let frame = Frame::new(along - n.any_perpendicular() * size - n.cross(n.any_perpendicular()) * size, n, n.any_perpendicular());
    let block = coreshape::box_solid(&frame, 2.0 * size, 2.0 * size, size, CUT_FEATURE)?;
    coreshape::boolean::subtract(body, &block)
}
