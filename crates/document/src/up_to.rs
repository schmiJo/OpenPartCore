//! Where an extrude ends when it isn't at a length: through everything, at the next thing it meets, or at a face, a part or
//! a vertex, as Onshape offers. An end on a flat face square to the direction, or at a vertex, is just a length, and exact.
//! Any other surface is extended as far as it needs to go, as Onshape does, and a long tool is cut with it.

use super::*;
use coreshape::boolean::classify::{ray_surface, Index};
use coreshape::build::{SLOT_END_CAP, SLOT_START_CAP};
use coreshape::{cone_frustum, intersect, sphere, subtract, tessellate, torus, TessParams};

/// Marks a long tool's far end, which shouldn't survive being cut.
const FAR_END: u32 = u32::MAX;
/// How far along a ray something has to be, not to be the surface the sketch is already on.
const ALREADY_THERE: f64 = 1e-6;

/// What an extrude's end can look at.
pub(super) struct Reach<'a> {
    pub all: &'a [Held],
    pub scope: &'a [(BodyId, Arc<Body>)],
    pub mode: Mode,
    pub env: &'a Env,
    pub tag: u32,
}

/// One profile's tools for an end. `frame` is its sketch's frame, moved by the extrude's start.
pub(super) fn tools_to(profile: &Profile, frame: &Frame, opposite: bool, end: &ExtrudeEnd, reach: &Reach) -> Result<Vec<Body>> {
    let way = Way { frame: *frame, dir: if opposite { -frame.z } else { frame.z }, sign: if opposite { -1.0 } else { 1.0 }, tag: reach.tag };
    match end {
        ExtrudeEnd::Blind => Err(Error::InvalidInput("a length isn't an end to look for".into())),
        ExtrudeEnd::ThroughAll => through_all(profile, &way, reach),
        ExtrudeEnd::UpToVertex(vertex) => {
            let at = vertex_of(vertex, reach)?;
            let h = (at - frame.origin).dot(way.dir);
            way.flat(profile, h, "that vertex")
        }
        ExtrudeEnd::UpToFace(key) => {
            let surface = reach.all.iter().find_map(|h| find_faces(&h.body, key).first().map(|f| h.body.faces[*f].surface.clone()));
            let surface = surface.ok_or_else(|| Error::InvalidInput(format!("the face {} no longer exists in the model", key.text())))?;
            way.to_surface(profile, &surface, &format!("the face {}", key.text()))
        }
        ExtrudeEnd::UpToPart(id) => {
            let part = reach.all.iter().find(|h| h.id == *id).ok_or_else(|| Error::InvalidInput(format!("it goes up to part {}, which isn't there at this point", id.text())))?;
            way.to_nearest(profile, &[part.body.clone()], false, &format!("part {}", id.text()))
        }
        ExtrudeEnd::UpToNext => {
            let bodies: Vec<Arc<Body>> = reach.scope.iter().map(|(_, b)| b.clone()).collect();
            // A cut goes on through the material to the far side of it. Anything else stops at the first thing it runs into.
            let leaving = matches!(reach.mode, Mode::Remove | Mode::Intersect);
            way.to_nearest(profile, &bodies, leaving, "the next face")
        }
    }
}

struct Way {
    frame: Frame,
    dir: Vec3,
    sign: f64,
    tag: u32,
}

/// Points on a profile to look from, well inside it where a ray won't run along a wall, and the corners of its outline.
fn samples_of(profile: &Profile, way: &Way) -> Result<(Vec<Vec3>, Vec<Vec3>)> {
    let probe = extrude(profile, &way.frame, 1.0, way.tag)?;
    let mesh = tessellate(&probe, &TessParams::draft_for_size(probe.aabb().diagonal()))?;
    let (mut inside, mut outline) = (Vec::new(), Vec::new());
    for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
        if probe.faces[f as usize].tag.slot != SLOT_START_CAP {
            continue;
        }
        let corners = [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]];
        let middle = (corners[0] + corners[1] + corners[2]) * (1.0 / 3.0);
        inside.push(middle);
        for c in corners {
            inside.push(c * 0.9 + middle * 0.1);
            outline.push(c);
        }
    }
    Ok((inside, outline))
}

/// The furthest a body gets along a direction. Exact on a vertex or a flat face, and on a curved face to within its mesh.
fn furthest(body: &Body, origin: Vec3, dir: Vec3) -> Result<f64> {
    let mut out = body.vertices.iter().map(|v| (*v - origin).dot(dir)).fold(f64::NEG_INFINITY, f64::max);
    if body.faces.iter().any(|f| !matches!(f.surface, Surface::Plane(_))) {
        let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal()))?;
        for p in &mesh.positions {
            out = out.max((*p - origin).dot(dir));
        }
    }
    Ok(out)
}

fn through_all(profile: &Profile, way: &Way, reach: &Reach) -> Result<Vec<Body>> {
    if reach.scope.is_empty() {
        return Err(Error::InvalidInput("there's nothing for it to go through: through all needs a part in the way".into()));
    }
    // A cut ends clear of everything, so its end leaves no mark. A new solid goes as far as the furthest part does.
    let clear = matches!(reach.mode, Mode::Remove | Mode::Intersect);
    let mut h = f64::NEG_INFINITY;
    for (_, body) in reach.scope {
        h = h.max(furthest(body, way.frame.origin, way.dir)?);
    }
    if h <= MIN_FEATURE {
        return Err(Error::InvalidInput("there's nothing in that direction to go through: \"opposite\" goes the other way".into()));
    }
    let h = if clear { h * 1.01 + 1.0 } else { h };
    Ok(vec![extrude(profile, &way.frame, way.sign * h, way.tag)?])
}

fn vertex_of(vertex: &VertexRef, reach: &Reach) -> Result<Vec3> {
    vertex_at(vertex, reach.all, reach.env)
}

/// Where a vertex is among the bodies there are.
pub(super) fn vertex_at(vertex: &VertexRef, all: &[Held], env: &Env) -> Result<Vec3> {
    let at = |p: &Point3| -> Result<Vec3> { Ok(v3(num(&p[0], env)?, num(&p[1], env)?, num(&p[2], env)?)) };
    let (faces, near) = match vertex {
        VertexRef::Point(p) => return at(p),
        VertexRef::Faces { faces, near } => (faces, near),
    };
    let mut found: Vec<Vec3> = Vec::new();
    for held in all {
        let body = &held.body;
        let mut common: Option<Vec<usize>> = None;
        for key in faces {
            let mut here: Vec<usize> = find_faces(body, key).iter().flat_map(|f| body.faces[*f].loops.iter().flatten().map(|c| body.start_vertex(*c))).collect();
            here.sort_unstable();
            here.dedup();
            common = Some(match common {
                Some(list) => list.into_iter().filter(|v| here.contains(v)).collect(),
                None => here,
            });
        }
        found.extend(common.unwrap_or_default().into_iter().map(|v| body.vertices[v]));
    }
    let names: Vec<String> = faces.iter().map(FaceKey::text).collect();
    match (found.as_slice(), near) {
        ([], _) => Err(Error::InvalidInput(format!("no vertex is on all of {}", names.join(", ")))),
        ([one], _) => Ok(*one),
        (many, Some(near)) => {
            let near = at(near)?;
            Ok(many.iter().copied().fold(many[0], |best, v| if v.dist(near) < best.dist(near) { v } else { best }))
        }
        (many, None) => Err(Error::InvalidInput(format!("{} vertices are on all of {}: name one more face, or say which with \"near\"", many.len(), names.join(", ")))),
    }
}

impl Way {
    /// Which end of an extrude is at the sketch, and which is far from it. Going backward, the kernel names them by where they are.
    fn slots(&self) -> (u32, u32) {
        if self.sign > 0.0 {
            (SLOT_START_CAP, SLOT_END_CAP)
        } else {
            (SLOT_END_CAP, SLOT_START_CAP)
        }
    }

    fn flat(&self, profile: &Profile, h: f64, what: &str) -> Result<Vec<Body>> {
        if h <= MIN_FEATURE {
            return Err(Error::InvalidInput(format!("{what} isn't ahead of the sketch, it's {:.6} behind it: \"opposite\" goes the other way", -h)));
        }
        Ok(vec![extrude(profile, &self.frame, self.sign * h, self.tag)?])
    }

    /// Up to the nearest crossing of these bodies seen from anywhere on the profile, ending on that one surface.
    fn to_nearest(&self, profile: &Profile, bodies: &[Arc<Body>], leaving: bool, what: &str) -> Result<Vec<Body>> {
        let (inside, _) = samples_of(profile, self)?;
        let mut best: Option<(f64, usize, usize)> = None;
        let mut started_within = false;
        for (b, body) in bodies.iter().enumerate() {
            let index = Index::new(body);
            for p in &inside {
                let length = p.dist(index.bounds.center()) + index.bounds.diagonal() + 1.0;
                let Some(hits) = index.ray_hits(*p, self.dir, length) else { continue };
                let mut ahead = hits.iter().filter(|h| h.t > ALREADY_THERE);
                if ahead.clone().next().is_some_and(|h| !h.entering) {
                    started_within = true;
                }
                if let Some(hit) = ahead.find(|h| h.entering != leaving) {
                    if best.is_none_or(|(t, _, _)| hit.t < t) {
                        best = Some((hit.t, b, hit.face));
                    }
                }
            }
        }
        let Some((_, b, f)) = best else {
            return Err(Error::InvalidInput(format!("it never meets {what} in that direction: \"opposite\" goes the other way")));
        };
        let surface = &bodies[b].faces[f].surface;
        let label = format!("the face {}", face_keys(&bodies[b])[f].text());
        let tools = self.to_surface(profile, surface, &label)?;
        // Those rays were only a sample, and the surface is extended. Something the tool now runs into means it wasn't the
        // nearest all over, which a single surface can't deal with. A part it started inside is its own to overlap.
        if !leaving && !started_within {
            for tool in &tools {
                for body in bodies {
                    if !boxes_meet(tool, body) {
                        continue;
                    }
                    let Ok(common) = intersect(tool, body) else { continue };
                    if !common.faces.is_empty() && coreshape::mass_properties(&common).volume > 1e-6 {
                        return Err(Error::InvalidInput(format!("ending on {label} would run into the part beyond it: an extrude's end follows one surface, and this needs several")));
                    }
                }
            }
        }
        Ok(tools)
    }

    /// Up to a surface, extended as far as it needs to go.
    fn to_surface(&self, profile: &Profile, surface: &Surface, label: &str) -> Result<Vec<Body>> {
        if let Surface::Plane(plane) = surface {
            let along = plane.z.dot(self.dir);
            if along.abs() > 1.0 - 1e-12 {
                return self.flat(profile, (plane.origin - self.frame.origin).dot(self.dir), label);
            }
            if along.abs() < 1e-9 {
                return Err(Error::InvalidInput(format!("{label} runs along the direction of the extrude, so it would never get there")));
            }
        }
        if !matches!(surface, Surface::Plane(_) | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }) {
            return Err(Error::Unsupported(format!("{label} is on a spline or free-form surface, and an extrude can only end on a plane, cylinder, cone, sphere or torus")));
        }
        let (inside, outline) = samples_of(profile, self)?;
        // Which side of it the sketch is on.
        let sides: Vec<f64> = inside.iter().map(|p| surface.sdf(*p)).collect();
        let before = sides.iter().all(|s| *s < -TOL);
        if !before && !sides.iter().all(|s| *s > TOL) {
            return Err(Error::InvalidInput(format!("the sketch crosses {label}, which has to be all to one side of it")));
        }
        // Long enough to get past it from anywhere on the profile. If it doesn't, what's left of the far end says so.
        let mut far: f64 = 0.0;
        for p in inside.iter().chain(&outline) {
            for t in ray_surface(surface, *p, self.dir, MAX_COORD) {
                far = far.max(t);
            }
        }
        if far <= MIN_FEATURE {
            return Err(Error::InvalidInput(format!("it never meets {label} in that direction: \"opposite\" goes the other way")));
        }
        let (near_slot, far_slot) = self.slots();
        let mut tool = extrude(profile, &self.frame, self.sign * (far * 1.05 + 1.0), self.tag)?;
        for f in &mut tool.faces {
            if f.tag.slot == far_slot {
                f.tag.slot = FAR_END;
            }
        }
        let mut limit = self.solid_of(surface, &tool, label, before)?;
        for f in &mut limit.faces {
            f.tag = FaceTag { feature: self.tag, slot: far_slot };
        }
        // A plane's solid is what's on the sketch's side of it, and a curved one's is what's inside it.
        let keep_within = before || matches!(surface, Surface::Plane(_));
        let cut = if keep_within { intersect(&tool, &limit) } else { subtract(&tool, &limit) };
        let cut = cut.map_err(|e| Error::InvalidInput(format!("it couldn't end on {label}: {e}")))?;
        let mut out = Vec::new();
        for piece in cut.solids() {
            // A piece away from the sketch is beyond the surface.
            if !piece.faces.iter().any(|f| f.tag.slot == near_slot) {
                continue;
            }
            if piece.faces.iter().any(|f| f.tag.slot == FAR_END) {
                return Err(Error::InvalidInput(format!("part of the sketch never meets {label}, even extended: it goes past it")));
            }
            out.push(piece);
        }
        if out.is_empty() {
            return Err(Error::InvalidInput(format!("it never meets {label} in that direction")));
        }
        Ok(out)
    }

    /// A solid whose surface is this one, all around the tool.
    fn solid_of(&self, surface: &Surface, tool: &Body, label: &str, before: bool) -> Result<Body> {
        let bounds = tool.aabb();
        let span = |axis: Vec3, origin: Vec3| -> (f64, f64) {
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for x in [bounds.min.x, bounds.max.x] {
                for y in [bounds.min.y, bounds.max.y] {
                    for z in [bounds.min.z, bounds.max.z] {
                        let h = (v3(x, y, z) - origin).dot(axis);
                        lo = lo.min(h);
                        hi = hi.max(h);
                    }
                }
            }
            (lo, hi)
        };
        match surface {
            Surface::Plane(plane) => {
                // A box on the sketch's side of the plane.
                let up = if before { -plane.z } else { plane.z };
                let center = bounds.center();
                let on = center - plane.z * (center - plane.origin).dot(plane.z);
                let size = bounds.diagonal() * 2.0 + (center - on).norm() * 2.0 + 1.0;
                let base = Frame::from_z(on, up);
                let corner = Frame { origin: on - base.x * size - base.y * size, ..base };
                box_solid(&corner, size * 2.0, size * 2.0, size, self.tag)
            }
            Surface::Cylinder { frame, radius } => {
                let (lo, hi) = span(frame.z, frame.origin);
                let margin = (hi - lo) * 0.1 + 1.0;
                cylinder(&Frame { origin: frame.origin + frame.z * (lo - margin), ..*frame }, *radius, hi - lo + 2.0 * margin, self.tag)
            }
            Surface::Sphere { frame, radius } => sphere(frame, *radius, self.tag),
            Surface::Torus { frame, major, minor } => torus(frame, *major, *minor, self.tag),
            Surface::Cone { frame, radius, half_angle } => {
                let slope = half_angle.tan();
                let tip = -radius / slope;
                let (lo, hi) = span(frame.z, frame.origin);
                let margin = (hi - lo) * 0.1 + 1.0;
                let top = hi + margin;
                let bottom = (lo - margin).max(tip + (top - tip) * 1e-3);
                if lo <= tip {
                    return Err(Error::InvalidInput(format!("the extrude reaches past the tip of the cone {label} is on")));
                }
                let base = Frame { origin: frame.origin + frame.z * bottom, ..*frame };
                cone_frustum(&base, radius + bottom * slope, radius + top * slope, top - bottom, self.tag)
            }
            _ => Err(Error::Unsupported(format!("{label} is on a surface an extrude can't end on"))),
        }
    }
}
