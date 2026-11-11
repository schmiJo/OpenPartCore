//! A flat face as a profile in its own plane, so that it can be extruded as a sketch's region is.
//!
//! The edges keep their exact form: a line stays a line, a circle an arc, and an ellipse or a spline maps into the
//! plane exactly, as an affine map takes a rational spline's control points to the image's. An intersection curve the
//! kernel only holds as samples has no exact form, so a face bounded by one is refused, saying so.

use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::spline::Spline;
use crate::surface::Surface;
use crate::topo::{Body, Coedge};
use std::sync::Arc;

/// A flat face's outline and holes, in a frame on its plane whose z is the face's outward normal. Each segment's id is
/// its place in the list of coedges returned with it, outline first, so that a wall swept from it can be traced back to
/// its edge.
pub fn flat_face_profile(body: &Body, face: usize) -> Result<(Frame, Profile, Vec<Coedge>)> {
    let f = body.faces.get(face).ok_or_else(|| Error::InvalidInput(format!("there is no face {face}")))?;
    let Surface::Plane(plane) = &f.surface else {
        return Err(Error::InvalidInput("it isn't flat, and only a flat face can be extruded".into()));
    };
    let z = if f.flipped { -plane.z } else { plane.z };
    let frame = Frame { origin: plane.origin, x: plane.x, y: z.cross(plane.x), z };
    let local = |p: Vec3| {
        let q = frame.to_local(p);
        v2(q.x, q.y)
    };
    // How far off the plane an edge can be and still be in it.
    let size = body.face_aabb(face).diagonal().max(1.0);
    let off_plane = 1e-9 * size + 10.0 * body.tolerance;
    let in_plane = |normal: Vec3| (normal.dot(z).abs() - 1.0).abs() <= 1e-9;
    let mut coedges = Vec::new();
    let mut wires = Vec::with_capacity(f.loops.len());
    for lp in &f.loops {
        let mut segs = Vec::with_capacity(lp.len());
        for c in lp {
            let id = coedges.len() as u32;
            coedges.push(*c);
            let edge = &body.edges[c.edge];
            let geom = match &edge.curve {
                Curve::Line { .. } => SegGeom::Line { a: local(body.vertices[body.start_vertex(*c)]), b: local(body.vertices[body.end_vertex(*c)]) },
                Curve::Circle { frame: around, radius } => {
                    if !in_plane(around.z) {
                        return Err(Error::Internal("a circle bounding a flat face isn't in its plane".into()));
                    }
                    // The circle's angle 0 in the face's frame, and which way its angle runs there.
                    let sense = around.z.dot(z).signum();
                    let zero = around.x.dot(frame.y).atan2(around.x.dot(frame.x));
                    let (t0, t1) = if c.reversed { (edge.t1, edge.t0) } else { (edge.t0, edge.t1) };
                    SegGeom::Arc { center: local(around.origin), radius: *radius, a0: zero + sense * t0, a1: zero + sense * t1 }
                }
                Curve::Ellipse { frame: around, major, minor } => {
                    if !in_plane(around.z) {
                        return Err(Error::Internal("an ellipse bounding a flat face isn't in its plane".into()));
                    }
                    let flat = |d: Vec3| v3(d.dot(frame.x), d.dot(frame.y), 0.0);
                    let center = local(around.origin);
                    let arc = Spline::ellipse_arc(v3(center.x, center.y, 0.0), flat(around.x), flat(around.y), *major, *minor, edge.t0, edge.t1)?;
                    SegGeom::Spline(Arc::new(if c.reversed { arc.reversed() } else { arc }))
                }
                Curve::Spline(s) => {
                    if let Some(far) = s.points().iter().map(|p| frame.to_local(*p).z.abs()).find(|h| *h > off_plane) {
                        return Err(Error::Internal(format!("a spline bounding a flat face is {far:.1e} off its plane")));
                    }
                    let flat = s.mapped(|p| {
                        let q = frame.to_local(p);
                        v3(q.x, q.y, 0.0)
                    });
                    let (u0, u1) = flat.domain();
                    let part = if edge.t0 > u0 || edge.t1 < u1 { flat.sub(edge.t0.max(u0), edge.t1.min(u1))? } else { flat };
                    SegGeom::Spline(Arc::new(if c.reversed { part.reversed() } else { part }))
                }
                Curve::Sampled(_) => {
                    return Err(Error::InvalidInput(
                        "an edge of it is an intersection curve the kernel holds as samples, with no exact form to extrude".into(),
                    ))
                }
            };
            segs.push(Seg { geom, id });
        }
        wires.push(Wire { segs });
    }
    let mut wires = wires.into_iter();
    let outer = wires.next().ok_or_else(|| Error::Internal("a face with no loops".into()))?;
    let profile = Profile { outer, holes: wires.collect() };
    // Seen from outside, a face's outline runs counter-clockwise and its holes clockwise, as a profile's do.
    if !(profile.outer.signed_area() > 0.0) || profile.holes.iter().any(|h| !(h.signed_area() < 0.0)) {
        return Err(Error::Internal("a flat face's loops don't run the way its normal says".into()));
    }
    profile.validate()?;
    Ok((frame, profile, coedges))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build::{cylinder, extrude};
    use crate::build::SLOT_END_CAP;
    use crate::mass::mass_properties;

    fn cap(body: &Body) -> usize {
        body.faces.iter().position(|f| f.tag.slot == SLOT_END_CAP).unwrap()
    }

    #[test]
    fn a_cylinder_top_is_a_circle_of_its_radius_facing_up() {
        let c = cylinder(&Frame::WORLD, 3.0, 5.0, 1).unwrap();
        let (frame, profile, coedges) = flat_face_profile(&c, cap(&c)).unwrap();
        assert!(frame.z.dist(Vec3::Z) < 1e-12 && (frame.origin.z - 5.0).abs() < 1e-12);
        assert!((profile.area() - PI * 9.0).abs() < 1e-9, "{}", profile.area());
        assert_eq!(coedges.len(), profile.outer.segs.len());
        // The same cylinder again from it.
        let again = extrude(&profile, &frame, 5.0, 2).unwrap();
        assert!((mass_properties(&again).volume - PI * 9.0 * 5.0).abs() < 1e-6);
    }

    #[test]
    fn a_face_with_a_hole_keeps_it() {
        let ring = Profile::new(Wire::rect_centered(10.0, 6.0)).with_hole(Wire::circle(v2(1.0, 0.5), 2.0));
        let tilted = Frame::new(v3(1.0, 2.0, 3.0), v3(1.0, -2.0, 0.5), v3(0.0, 0.0, 1.0));
        let block = extrude(&ring, &tilted, 4.0, 1).unwrap();
        for slot in [crate::build::SLOT_START_CAP, SLOT_END_CAP] {
            let f = block.faces.iter().position(|f| f.tag.slot == slot).unwrap();
            let (frame, profile, _) = flat_face_profile(&block, f).unwrap();
            assert_eq!(profile.holes.len(), 1);
            assert!((profile.area() - (60.0 - 4.0 * PI)).abs() < 1e-9);
            // Outward: away from the block's middle.
            let middle = tilted.origin + tilted.z * 2.0;
            assert!((frame.origin - middle).dot(frame.z) > 0.0);
        }
    }

    #[test]
    fn a_curved_face_is_refused() {
        let c = cylinder(&Frame::WORLD, 3.0, 5.0, 1).unwrap();
        let side = c.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder { .. })).unwrap();
        let why = flat_face_profile(&c, side).unwrap_err().to_string();
        assert!(why.contains("isn't flat"), "{why}");
    }
}
