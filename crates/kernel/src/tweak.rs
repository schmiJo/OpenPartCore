//! Offset, shell and draft, which all change a body's surfaces and keep its topology.
//!
//! Each face gets a new surface. Then every vertex is found again where its new surfaces meet, and
//! every edge is rebuilt between its new vertices. All five surface types offset to their own kind, and
//! a drafted cylinder is a cone, so the result is as exact as the original. If a change is big enough to
//! alter the topology, as when an offset makes a face vanish, it is refused: the checks below catch an
//! edge that has turned round or a face that no longer closes.

use crate::boolean::classify::{BodyLoc, Index};
use crate::boolean::ssi::{self, Ssi};
use crate::boolean::{subtract};
use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::surface::{refine_on_two, Surface};
use crate::topo::{Body, Edge};
use crate::validate::validate;

/// How much two faces' normals can differ along an edge and still count as tangent.
const TANGENT: f64 = 1e-7;

fn refuse(what: String) -> Error {
    Error::InvalidInput(what)
}

/// A surface moved `d` along its own normal: the one it defines, not the face's outward one.
fn moved_surface(s: &Surface, d: f64) -> Result<Surface> {
    let positive = |r: f64| -> Result<f64> {
        if r > MIN_FEATURE {
            Ok(r)
        } else {
            Err(refuse(format!("offsetting by {d} shrinks a curved face away to nothing")))
        }
    };
    if matches!(s, Surface::Extruded { .. } | Surface::Revolved { .. }) {
        return Err(Error::Unsupported("offset and shell on spline faces aren't available yet: an offset of a spline isn't a spline, and this kernel gives an exact answer or none".into()));
    }
    Ok(match *s {
        Surface::Extruded { .. } | Surface::Revolved { .. } => unreachable!(),
        Surface::Plane(f) => Surface::Plane(Frame { origin: f.origin + f.z * d, ..f }),
        Surface::Cylinder { frame, radius } => Surface::Cylinder { frame, radius: positive(radius + d)? },
        // Moving along a cone's slanted normal shifts it sideways by more than d.
        Surface::Cone { frame, radius, half_angle } => Surface::Cone { frame, radius: radius + d / half_angle.cos(), half_angle },
        Surface::Sphere { frame, radius } => Surface::Sphere { frame, radius: positive(radius + d)? },
        Surface::Torus { frame, major, minor } => Surface::Torus { frame, major, minor: positive(minor + d)? },
    })
}

/// The new position of a vertex: nearest to where it was, on all the surfaces given. Each pass
/// takes the smallest step that meets all of them to first order, skipping any whose normal adds nothing,
/// as a tangent neighbour's doesn't.
fn point_on_surfaces(surfaces: &[Surface], start: Vec3) -> Option<Vec3> {
    let mut p = start;
    for _ in 0..40 {
        // Gram-Schmidt on the normals, carrying the right-hand sides along.
        let mut basis: Vec<(Vec3, f64)> = Vec::with_capacity(3);
        let mut largest = 0.0f64;
        for s in surfaces {
            let f = s.sdf(p);
            largest = largest.max(f.abs());
            let (mut n, mut rhs) = (s.normal(p), -f);
            for (b, r) in &basis {
                let c = n.dot(*b);
                n = n - *b * c;
                rhs -= c * r;
            }
            let len = n.norm();
            if len > 1e-5 {
                basis.push((n * (1.0 / len), rhs / len));
            }
        }
        if largest < 1e-12 {
            return Some(p);
        }
        let step = basis.iter().fold(Vec3::ZERO, |sum, (b, r)| sum + *b * *r);
        p = p + step;
    }
    let worst = surfaces.iter().fold(0.0f64, |m, s| m.max(s.sdf(p).abs()));
    (worst < TOL * 0.1).then_some(p)
}

/// The parameter range taking a curve from `a` to `b` by the way nearest `middle`, with the curve
/// turned round if need be so that the parameter increases along it.
fn fitted(curve: &Curve, a: Vec3, b: Vec3, middle: Vec3) -> Option<(Curve, f64, f64)> {
    let tol = TOL * 10.0;
    match curve {
        Curve::Line { origin, dir } => {
            let (t0, t1) = ((a - *origin).dot(*dir), (b - *origin).dot(*dir));
            let (c, s0, s1) = if t1 > t0 { (curve.clone(), t0, t1) } else { (Curve::Line { origin: *origin, dir: -*dir }, -t0, -t1) };
            (c.eval(s0).dist(a) < tol && c.eval(s1).dist(b) < tol).then_some((c, s0, s1))
        }
        Curve::Circle { .. } | Curve::Ellipse { .. } => {
            let frame = match curve {
                Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => *frame,
                _ => unreachable!(),
            };
            let flipped_frame = Frame { z: -frame.z, y: -frame.y, ..frame };
            let with = |f: Frame| match *curve {
                Curve::Circle { radius, .. } => Curve::Circle { frame: f, radius },
                Curve::Ellipse { major, minor, .. } => Curve::Ellipse { frame: f, major, minor },
                _ => unreachable!(),
            };
            let mut best: Option<(f64, (Curve, f64, f64))> = None;
            for f in [frame, flipped_frame] {
                let c = with(f);
                let (t0, _) = c.closest(a, 0.0, TAU);
                let (t1, _) = c.closest(b, 0.0, TAU);
                let mut end = t1;
                while end <= t0 {
                    end += TAU;
                }
                if c.eval(t0).dist(a) > tol || c.eval(end).dist(b) > tol {
                    continue;
                }
                let off = c.eval((t0 + end) / 2.0).dist(middle);
                if best.as_ref().is_none_or(|x| off < x.0) {
                    best = Some((off, (c, t0, end)));
                }
            }
            best.map(|b| b.1)
        }
        Curve::Sampled(_) | Curve::Spline(_) => None,
    }
}

/// The plane a seam lies in. Seams run along a surface's axis or around it, so a change that keeps the
/// axis keeps this plane too, and it stays true of the new surface.
fn seam_plane(edge: &Edge, on: &Surface) -> Surface {
    let middle = edge.eval_mid();
    let n = match &edge.curve {
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => frame.z,
        _ => (edge.curve.eval(edge.t1) - edge.curve.eval(edge.t0)).cross(on.normal(on.project(middle))).normalized(),
    };
    Surface::Plane(Frame::from_z(middle, n))
}

/// The new curve for an edge between surfaces `s` and `t`, from `a` to `b`.
fn rebuilt_curve(edge: &Edge, s: &Surface, t: &Surface, same_surface: bool, a: Vec3, b: Vec3) -> Result<(Curve, f64, f64)> {
    let middle = edge.eval_mid();
    let on_both = |p: Vec3, tol: f64| s.sdf(p).abs() < tol && t.sdf(p).abs() < tol;
    // A straight edge often stays straight: rulings, and edges between planes. Checking is cheaper
    // than intersecting, and works where the surfaces are tangent, which intersection doesn't.
    if matches!(edge.curve, Curve::Line { .. }) && a.dist(b) > MIN_FEATURE {
        let straight = Curve::line_through(a, b);
        if [0.25, 0.5, 0.75].iter().all(|f| on_both(a + (b - a) * *f, TOL)) {
            return fitted(&straight, a, b, middle).ok_or_else(|| Error::Internal("a straight edge didn't fit its ends".into()));
        }
    }
    // A seam between pieces of one surface lies in the plane of the old curve.
    let cutter = same_surface.then(|| seam_plane(edge, s));
    let other = cutter.as_ref().unwrap_or(t);
    let mut region = Aabb::EMPTY;
    region.add(a);
    region.add(b);
    region.add(middle);
    let region = region.inflated(a.dist(b).max(a.dist(middle)) + 1.0);
    if let Ssi::Curves { curves, .. } = ssi::intersect(s, other, &region) {
        let mut best: Option<(f64, (Curve, f64, f64))> = None;
        for c in &curves {
            if let Some(found) = fitted(&c.curve, a, b, middle) {
                let off = found.0.eval((found.1 + found.2) / 2.0).dist(middle);
                if best.as_ref().is_none_or(|x| off < x.0) {
                    best = Some((off, found));
                }
            }
        }
        if let Some((_, found)) = best {
            return Ok(found);
        }
    }
    // No closed form: follow the old curve, moving samples onto both new surfaces.
    let n = 24;
    let mut points = Vec::with_capacity(n + 1);
    points.push(a);
    for i in 1..n {
        let p = edge.curve.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / n as f64);
        points.push(refine_on_two(s, other, p).ok_or_else(|| refuse("an edge can't be followed onto the new surfaces: they may have stopped meeting there".to_string()))?);
    }
    points.push(b);
    let curve = ssi::sampled_from_points(points, s, other)?;
    let last = match &curve {
        Curve::Sampled(c) => (c.points.len() - 1) as f64,
        _ => unreachable!(),
    };
    Ok((curve, 0.0, last))
}

/// A tangent edge's new curve, from what happens to its points.
fn mapped_tangent_edge(edge: &Edge, map: &mut dyn FnMut(Vec3) -> Vec3) -> Option<Curve> {
    match &edge.curve {
        Curve::Line { origin, dir } => {
            let a = map(*origin + *dir * edge.t0);
            let b = map(*origin + *dir * edge.t1);
            (a.dist(b) > MIN_FEATURE).then(|| Curve::line_through(a, b))
        }
        Curve::Circle { frame, radius } => {
            // The map keeps a rim round its axis, so one point tells where the rim went.
            let p = map(frame.origin + frame.x * *radius);
            let local = p - frame.origin;
            let r = local.dot(frame.x);
            (r > MIN_FEATURE).then(|| Curve::Circle { frame: Frame { origin: frame.origin + frame.z * local.dot(frame.z), ..*frame }, radius: r })
        }
        _ => None,
    }
}

/// How a change treats a tangent edge: its faces stay tangent to each other, so their intersection is
/// ill-conditioned, and the new edge has to come from where the old one's points go.
type TangentMap<'a> = &'a dyn Fn(usize, Vec3) -> Result<Vec3>;

/// The body with each face on a new surface and the same topology.
fn with_surfaces(body: &Body, surfaces: &[Surface], tangent_map: TangentMap<'_>) -> Result<Body> {
    let uses = body.edge_uses();
    // What each edge is between.
    let mut tangent = vec![false; body.edges.len()];
    let mut same = vec![false; body.edges.len()];
    let size = body.aabb().diagonal().max(1.0);
    for (e, edge) in body.edges.iter().enumerate() {
        if uses[e].len() != 2 {
            return Err(Error::Unsupported("an edge used by other than two faces can't be moved".into()));
        }
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        same[e] = ssi::coincident(&body.faces[f].surface, &body.faces[g].surface, size).is_some();
        if !same[e] {
            let p = edge.eval_mid();
            let (a, b) = (body.faces[f].surface.normal(p), body.faces[g].surface.normal(p));
            tangent[e] = a.cross(b).norm() < TANGENT;
        }
    }
    let curve_for_tangent = |e: usize| -> Result<Curve> {
        let edge = &body.edges[e];
        let mut failed = None;
        let mut map = |p: Vec3| match tangent_map(e, p) {
            Ok(q) => q,
            Err(why) => {
                failed.get_or_insert(why);
                p
            }
        };
        let c = mapped_tangent_edge(edge, &mut map);
        if let Some(why) = failed {
            return Err(why);
        }
        c.ok_or_else(|| Error::Unsupported("a smooth edge of this kind can't be moved yet: only straight and circular ones can".into()))
    };
    let mut tangent_curves: Vec<Option<Curve>> = vec![None; body.edges.len()];
    for e in 0..body.edges.len() {
        if tangent[e] {
            tangent_curves[e] = Some(curve_for_tangent(e)?);
        }
    }
    // Vertices.
    let mut at_vertex: Vec<Vec<usize>> = vec![Vec::new(); body.vertices.len()];
    for (e, edge) in body.edges.iter().enumerate() {
        at_vertex[edge.v0].push(e);
        at_vertex[edge.v1].push(e);
    }
    let mut vertices = body.vertices.clone();
    for (v, edges) in at_vertex.iter().enumerate() {
        if edges.is_empty() {
            continue;
        }
        let mut faces: Vec<usize> = edges.iter().flat_map(|&e| uses[e].iter().map(|u| u.0)).collect();
        faces.sort_unstable();
        faces.dedup();
        let mut list: Vec<Surface> = faces.iter().map(|&f| surfaces[f].clone()).collect();
        // A vertex on a seam has to stay in the seam's plane, or the seam couldn't reach it.
        for &e in edges {
            if same[e] {
                list.push(seam_plane(&body.edges[e], &body.faces[uses[e][0].0].surface));
            }
        }
        // A tangent edge fixes the vertex to a curve, leaving only where along it, which the other
        // surfaces settle. Tangent surfaces alone would pin it only to about the square root of the tolerance.
        let along = edges.iter().find_map(|&e| tangent_curves[e].as_ref().map(|c| (e, c)));
        let start = body.vertices[v];
        vertices[v] = match along {
            Some((e, c)) => {
                let others: Vec<Surface> = {
                    let tangent_pair = (uses[e][0].0, uses[e][1].0);
                    faces.iter().filter(|&&f| f != tangent_pair.0 && f != tangent_pair.1).map(|&f| surfaces[f].clone()).collect()
                };
                let (mut t, _) = c.closest(tangent_map(e, start)?, -1e9f64.min(0.0), 1e9);
                if matches!(c, Curve::Circle { .. }) {
                    t = c.closest(tangent_map(e, start)?, 0.0, TAU).0;
                }
                let mut p = c.eval(t);
                for _ in 0..40 {
                    let Some(s) = others.iter().find(|s| {
                        let n = s.normal(p);
                        n.dot(c.derivative(t)).abs() > 1e-6 * c.derivative(t).norm()
                    }) else {
                        break;
                    };
                    let f = s.sdf(p);
                    if f.abs() < 1e-13 {
                        break;
                    }
                    let d = c.derivative(t);
                    t -= f / s.normal(p).dot(d);
                    p = c.eval(t);
                }
                p
            }
            None => point_on_surfaces(&list, start).ok_or_else(|| refuse("the new surfaces no longer meet at a corner".to_string()))?,
        };
        if list.iter().any(|s| s.sdf(vertices[v]).abs() > 10.0 * TOL) {
            return Err(refuse("the new surfaces no longer meet at a corner".to_string()));
        }
    }
    // Edges.
    let mut out = Body { vertices, ..Body::default() };
    for (e, edge) in body.edges.iter().enumerate() {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (a, b) = (out.vertices[edge.v0], out.vertices[edge.v1]);
        if a.dist(b) < MIN_FEATURE {
            return Err(refuse("an edge shrinks away to nothing: the change is too big for the part's shape".to_string()));
        }
        let (curve, t0, t1) = match &tangent_curves[e] {
            Some(c) => fitted(c, a, b, tangent_map(e, edge.eval_mid())?).ok_or_else(|| refuse("a smooth edge didn't fit its new corners".to_string()))?,
            None => rebuilt_curve(edge, &surfaces[f], &surfaces[g], same[e], a, b)?,
        };
        // An edge that has turned round means a face has turned inside out.
        let (old, new) = (edge.curve.eval(edge.t1) - edge.curve.eval(edge.t0), curve.eval(t1) - curve.eval(t0));
        if matches!(edge.curve, Curve::Line { .. }) && old.dot(new) <= 0.0 {
            return Err(refuse("an edge turns round: the change is too big for the part's shape".to_string()));
        }
        out.edges.push(Edge { curve, t0, t1, v0: edge.v0, v1: edge.v1 });
    }
    for (f, face) in body.faces.iter().enumerate() {
        out.add_face(surfaces[f].clone(), face.flipped, face.loops.clone(), face.tag);
    }
    out.shells = body.shells.clone();
    validate(&out).map_err(|defects| refuse(format!("the result isn't a valid solid: {}", defects.join("; "))))?;
    let check = crate::tess::tessellate(&out, &crate::tess::TessParams::for_size(out.aabb().diagonal()))?;
    if check.open_edge_count() > 0 {
        return Err(refuse("the result doesn't mesh into a closed surface: the change is too big for the part's shape".to_string()));
    }
    Ok(out)
}

/// Moves each face outward by its own distance, negative for inward. Sharp edges stay sharp: neighbouring
/// faces simply meet further out or in. Faces that share a smooth edge must move by the same distance.
pub fn offset_faces(body: &Body, distances: &[f64]) -> Result<Body> {
    if distances.len() != body.faces.len() {
        return Err(refuse(format!("{} distances for {} faces", distances.len(), body.faces.len())));
    }
    if distances.iter().any(|d| !d.is_finite() || d.abs() > MAX_COORD) {
        return Err(refuse("an offset distance must be a sensible number".to_string()));
    }
    let surfaces = body
        .faces
        .iter()
        .zip(distances)
        .map(|(f, d)| moved_surface(&f.surface, if f.flipped { -*d } else { *d }))
        .collect::<Result<Vec<Surface>>>()?;
    let uses = body.edge_uses();
    let map = |e: usize, p: Vec3| -> Result<Vec3> {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        if (distances[f] - distances[g]).abs() > TOL {
            return Err(refuse("two faces that meet smoothly can't move by different distances".to_string()));
        }
        Ok(p + body.faces[f].outward_normal(p) * distances[f])
    };
    with_surfaces(body, &surfaces, &map)
}

/// Moves every face outward by `distance`, or inward if negative.
pub fn offset(body: &Body, distance: f64) -> Result<Body> {
    if distance.abs() < MIN_FEATURE {
        return Err(refuse(format!("an offset of {distance} is too small")));
    }
    offset_faces(body, &vec![distance; body.faces.len()])
}

/// Hollows a body to walls of `thickness`, inward from every face, leaving the listed faces open.
/// Inner faces are named for feature `feature`, with the slot of the face they follow.
pub fn shell(body: &Body, open: &[usize], thickness: f64, feature: u32) -> Result<Body> {
    if !(thickness >= MIN_FEATURE) {
        return Err(refuse(format!("a wall thickness of {thickness} is too small")));
    }
    if let Some(&bad) = open.iter().find(|&&f| f >= body.faces.len()) {
        return Err(refuse(format!("there is no face {bad}")));
    }
    // Open faces move out, so that the cavity passes cleanly through them.
    let distances: Vec<f64> = (0..body.faces.len()).map(|f| if open.contains(&f) { thickness } else { -thickness }).collect();
    let mut inner = offset_faces(body, &distances)?;
    // The cavity has to be inside the part wherever it's closed.
    let index = Index::new(body);
    for (v, p) in inner.vertices.iter().enumerate() {
        let touches_open = inner.faces.iter().enumerate().any(|(f, face)| open.contains(&f) && face.loops.iter().flatten().any(|c| {
            let e = &inner.edges[c.edge];
            e.v0 == v || e.v1 == v
        }));
        if !touches_open && index.locate_in_body(*p) != BodyLoc::Inside {
            return Err(refuse(format!("walls of {thickness} don't fit: the cavity would reach outside the part")));
        }
    }
    for f in &mut inner.faces {
        f.tag.feature = feature;
    }
    let result = subtract(body, &inner)?;
    let (whole, cavity, left) = (crate::mass::mass_properties(body).volume, crate::mass::mass_properties(&inner).volume, crate::mass::mass_properties(&result).volume);
    if open.is_empty() && (whole - cavity - left).abs() > 1e-6 * whole.abs().max(1.0) {
        return Err(Error::Internal(format!("the shell's volume doesn't add up: {whole} - {cavity} isn't {left}")));
    }
    Ok(result)
}

/// Tapers faces along a pull direction: planes parallel to it tilt about their line on the neutral plane,
/// and cylinders along it become cones. A positive angle makes the part narrower going along the pull, and
/// a hole wider, so that it releases from a mould drawn that way.
pub fn draft(body: &Body, faces: &[usize], pull: Vec3, neutral: Vec3, angle: f64) -> Result<Body> {
    if !(pull.norm() > 1e-12) {
        return Err(refuse("a pull direction must have a non-zero length".to_string()));
    }
    if !(angle.abs() > 1e-9 && angle.abs() < 80f64.to_radians()) {
        return Err(refuse(format!("a draft of {:.3} degrees is out of range", angle.to_degrees())));
    }
    let pull = pull.normalized();
    let mut surfaces: Vec<Surface> = body.faces.iter().map(|f| f.surface.clone()).collect();
    for &i in faces {
        let face = body.faces.get(i).ok_or_else(|| refuse(format!("there is no face {i}")))?;
        surfaces[i] = match face.surface {
            Surface::Plane(frame) => {
                let out = if face.flipped { -frame.z } else { frame.z };
                if out.dot(pull).abs() > 1e-9 {
                    return Err(refuse("a planar face can only be drafted if it runs along the pull direction".to_string()));
                }
                // On the neutral plane the face stays put: it turns about the line it meets that plane in.
                let on_neutral = frame.origin - pull * (frame.origin - neutral).dot(pull);
                let axis = pull.cross(out);
                // Going along the pull, a positive angle moves the outward-facing plane inward.
                let spin = Frame::rotation(on_neutral, axis, -angle);
                let z = spin.dir_to_world(frame.z);
                Surface::Plane(Frame::new(on_neutral, z, frame.x))
            }
            Surface::Cylinder { frame, radius } => {
                if frame.z.cross(pull).norm() > 1e-9 {
                    return Err(refuse("a cylindrical face can only be drafted along its axis".to_string()));
                }
                let origin = frame.origin + frame.z * (neutral - frame.origin).dot(frame.z);
                // A cone widens along its z, so a part's wall narrowing along the pull has z against it.
                let z = if face.flipped == (angle > 0.0) { pull } else { -pull };
                let half_angle = angle.abs();
                Surface::Cone { frame: Frame::new(origin, z, frame.x), radius, half_angle }
            }
            _ => return Err(Error::Unsupported("only planes and cylinders can be drafted".into())),
        };
    }
    // A smooth edge between drafted faces is a straight ruling. It keeps its point on the neutral plane and
    // tilts the way its faces do.
    let uses = body.edge_uses();
    let map = |e: usize, p: Vec3| -> Result<Vec3> {
        let (f, g) = (uses[e][0].0, uses[e][1].0);
        let (fd, gd) = (faces.contains(&f), faces.contains(&g));
        if fd != gd {
            return Err(refuse("a face can't be drafted while a face it meets smoothly isn't: select them both".to_string()));
        }
        if !fd {
            return Ok(p);
        }
        if !matches!(body.edges[e].curve, Curve::Line { .. }) {
            return Err(Error::Unsupported("a smooth edge between drafted faces must be straight".into()));
        }
        let out = body.faces[f].outward_normal(p);
        let along = (p - neutral).dot(pull);
        Ok(p - out * (along * angle.tan()))
    };
    with_surfaces(body, &surfaces, &map)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn moved_surfaces_are_distance_d_away() {
        let f = Frame::new(v3(1.0, 2.0, 3.0), v3(0.3, 0.2, 1.0), Vec3::X);
        let list = [
            Surface::Plane(f),
            Surface::Cylinder { frame: f, radius: 7.0 },
            Surface::Cone { frame: f, radius: 7.0, half_angle: 0.4 },
            Surface::Sphere { frame: f, radius: 7.0 },
            Surface::Torus { frame: f, major: 12.0, minor: 3.0 },
        ];
        let mut rng = Rng::new(5);
        for s in &list {
            for d in [1.5, -1.5] {
                let t = moved_surface(s, d).unwrap();
                for _ in 0..200 {
                    let raw = f.to_world(v3(rng.range(-9.0, 9.0), rng.range(-9.0, 9.0), rng.range(1.0, 9.0)));
                    let p = s.project(raw);
                    if s.sdf(p).abs() > 1e-9 {
                        continue;
                    }
                    let moved = p + s.normal(p) * d;
                    assert!(t.sdf(moved).abs() < 1e-9, "{s:?} by {d}: {}", t.sdf(moved));
                }
            }
        }
    }
}
