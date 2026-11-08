//! Extrude, revolve and the primitives built from them.
//!
//! Face naming: slot 0 and 1 are the start and end caps, and slot `2 + seg.id`
//! is the face swept from that profile segment.

use crate::curve::Curve;
use crate::error::{Error, Result};
use crate::math::*;
use crate::profile::{Profile, Seg, SegGeom, Wire};
use crate::surface::Surface;
use crate::topo::{Body, Coedge, FaceTag};
use crate::validate::validate;
use std::collections::HashMap;

pub const SLOT_START_CAP: u32 = 0;
pub const SLOT_END_CAP: u32 = 1;
pub const SLOT_SEG_BASE: u32 = 2;

/// Lifts a profile segment into 3D. The profile's x and y map to `frame.x` and
/// `frame.y`, offset by `z` along `frame.z`. Returns the curve and the parameters
/// at the segment's start and end.
fn lift_seg(seg: &Seg, frame: &Frame, z: f64) -> (Curve, f64, f64) {
    match seg.geom {
        SegGeom::Line { a, b } => {
            let (pa, pb) = (frame.point(a.x, a.y, z), frame.point(b.x, b.y, z));
            (Curve::line_through(pa, pb), 0.0, pa.dist(pb))
        }
        SegGeom::Arc { center, radius, a0, a1 } => {
            let f = Frame { origin: frame.point(center.x, center.y, z), ..*frame };
            (Curve::Circle { frame: f, radius }, a0, a1)
        }
    }
}

/// Extrudes a profile by `height` along `frame.z`. A negative height extrudes the
/// other way.
pub fn extrude(profile: &Profile, frame: &Frame, height: f64, feature: u32) -> Result<Body> {
    profile.validate()?;
    if !height.is_finite() || height.abs() < MIN_FEATURE {
        return Err(Error::InvalidInput(format!("extrude height {height} is too small")));
    }
    let (frame, height) = if height < 0.0 {
        (Frame { origin: frame.origin + frame.z * height, ..*frame }, -height)
    } else {
        (*frame, height)
    };

    let mut body = Body::default();
    let tag = |slot| FaceTag { feature, slot };
    let mut bottom_loops = Vec::new();
    let mut top_loops = Vec::new();

    for wire in profile.wires_for_faces() {
        let n = wire.segs.len();
        let lower: Vec<usize> = (0..n)
            .map(|i| {
                let p = wire.segs[i].start();
                body.add_vertex(frame.point(p.x, p.y, 0.0))
            })
            .collect();
        let upper: Vec<usize> = (0..n)
            .map(|i| {
                let p = wire.segs[i].start();
                body.add_vertex(frame.point(p.x, p.y, height))
            })
            .collect();
        let rails: Vec<Coedge> = (0..n)
            .map(|i| {
                let p = body.vertices[lower[i]];
                body.add_edge(Curve::Line { origin: p, dir: frame.z }, 0.0, height, lower[i], upper[i])
            })
            .collect();
        let mut bottom = Vec::with_capacity(n);
        let mut top = Vec::with_capacity(n);
        for i in 0..n {
            let j = (i + 1) % n;
            let seg = &wire.segs[i];
            let (curve, ta, tb) = lift_seg(seg, &frame, 0.0);
            let low = body.add_edge(curve, ta, tb, lower[i], lower[j]);
            let (curve, ta, tb) = lift_seg(seg, &frame, height);
            let high = body.add_edge(curve, ta, tb, upper[i], upper[j]);
            bottom.push(low);
            top.push(high);

            let (surface, flipped) = match seg.geom {
                SegGeom::Line { a, b } => {
                    // Outward is to the right of travel, and x * y = normal.
                    let (pa, pb) = (frame.point(a.x, a.y, 0.0), frame.point(b.x, b.y, 0.0));
                    let x = (pb - pa).normalized();
                    let s = Surface::Plane(Frame { origin: pa, x, y: frame.z, z: x.cross(frame.z) });
                    (s, false)
                }
                SegGeom::Arc { center, radius, a0, a1 } => {
                    let f = Frame { origin: frame.point(center.x, center.y, 0.0), ..frame };
                    // Counter-clockwise keeps material toward the axis, so the
                    // outward normal is the radial direction.
                    (Surface::Cylinder { frame: f, radius }, a1 < a0)
                }
            };
            body.add_face(
                surface,
                flipped,
                vec![vec![low, rails[j], high.flipped(), rails[i].flipped()]],
                tag(SLOT_SEG_BASE + seg.id),
            );
        }
        bottom_loops.push(bottom.iter().rev().map(|c| c.flipped()).collect::<Vec<_>>());
        top_loops.push(top);
    }

    body.add_face(Surface::Plane(frame), true, bottom_loops, tag(SLOT_START_CAP));
    let top_frame = Frame { origin: frame.origin + frame.z * height, ..frame };
    body.add_face(Surface::Plane(top_frame), false, top_loops, tag(SLOT_END_CAP));
    body.compute_shells();
    validate(&body).map_err(Error::InvalidBody)?;
    Ok(body)
}

/// Revolves a profile about `frame.z`.
///
/// The profile's x is the distance from the axis along `frame.x`, and its y is
/// the height along `frame.z`. The sweep starts in the XZ plane and turns by
/// `angle` radians following the right-hand rule. An angle of `TAU` makes a
/// closed revolution with no end caps.
pub fn revolve(profile: &Profile, frame: &Frame, angle: f64, feature: u32) -> Result<Body> {
    profile.validate()?;
    if !(angle > 1e-6 && angle <= TAU + 1e-9) {
        return Err(Error::InvalidInput(format!("revolve angle {angle} is out of range")));
    }
    let full = (angle - TAU).abs() < 1e-9;
    let angle = if full { TAU } else { angle };
    let steps: usize = if full || angle > PI + 1e-9 { 2 } else { 1 };
    let f = *frame;
    let wires = profile.wires_for_faces();

    // Every wire must stay on the positive side of the axis.
    for w in &wires {
        for s in &w.segs {
            for i in 0..=32 {
                if s.at(i as f64 / 32.0).x < -TOL {
                    return Err(Error::InvalidProfile("profile crosses the revolve axis".into()));
                }
            }
        }
    }

    let sweep_angle = |k: usize| angle * k as f64 / steps as f64;
    let radial = |phi: f64| f.x * phi.cos() + f.y * phi.sin();
    let wrap = |k: usize| if full { k % steps } else { k };

    let mut body = Body::default();
    let tag = |slot| FaceTag { feature, slot };

    // Vertices: on-axis points exist once, the others once per sweep station.
    let mut vertex_ids: HashMap<(usize, usize, usize), usize> = HashMap::new();
    for (w, wire) in wires.iter().enumerate() {
        for (i, seg) in wire.segs.iter().enumerate() {
            let p = seg.start();
            if p.x < TOL {
                let id = body.add_vertex(f.point(0.0, 0.0, p.y));
                for k in 0..=steps {
                    vertex_ids.insert((w, i, k), id);
                }
            } else {
                for k in 0..=steps {
                    if wrap(k) == k {
                        let r = radial(sweep_angle(k));
                        let id = body.add_vertex(f.origin + r * p.x + f.z * p.y);
                        vertex_ids.insert((w, i, k), id);
                    }
                }
                if full {
                    let first = vertex_ids[&(w, i, 0)];
                    vertex_ids.insert((w, i, steps), first);
                }
            }
        }
    }

    let is_on_axis = |seg: &Seg| matches!(seg.geom, SegGeom::Line { .. }) && seg.start().x < TOL && seg.end().x < TOL;

    // Meridian edges (profile segment at a station) and sweep edges (arc around
    // the axis at a profile vertex) are created on first use.
    let mut meridians: HashMap<(usize, usize, usize), Coedge> = HashMap::new();
    let mut sweeps: HashMap<(usize, usize, usize), Coedge> = HashMap::new();

    fn meridian(
        body: &mut Body,
        cache: &mut HashMap<(usize, usize, usize), Coedge>,
        vertex_ids: &HashMap<(usize, usize, usize), usize>,
        f: &Frame,
        phi: f64,
        key: (usize, usize, usize),
        wire: &Wire,
    ) -> Coedge {
        if let Some(c) = cache.get(&key) {
            return *c;
        }
        let (w, i, k) = key;
        let seg = &wire.segs[i];
        let j = (i + 1) % wire.segs.len();
        let (a, b) = (vertex_ids[&(w, i, k)], vertex_ids[&(w, j, k)]);
        let r = f.x * phi.cos() + f.y * phi.sin();
        let c = match seg.geom {
            SegGeom::Line { .. } => {
                let (pa, pb) = (body.vertices[a], body.vertices[b]);
                body.add_edge(Curve::line_through(pa, pb), 0.0, pa.dist(pb), a, b)
            }
            SegGeom::Arc { center, radius, a0, a1 } => {
                // The profile plane has x = radial and y = axis.
                let frame = Frame { origin: f.origin + r * center.x + f.z * center.y, x: r, y: f.z, z: r.cross(f.z) };
                body.add_edge(Curve::Circle { frame, radius }, a0, a1, a, b)
            }
        };
        cache.insert(key, c);
        c
    }

    let mut caps_start: Vec<Vec<Coedge>> = Vec::new();
    let mut caps_end: Vec<Vec<Coedge>> = Vec::new();
    let on_axis_key = |k: usize, seg: &Seg| if is_on_axis(seg) { 0 } else { k };

    for (w, wire) in wires.iter().enumerate() {
        let n = wire.segs.len();
        for i in 0..n {
            let seg = wire.segs[i];
            if is_on_axis(&seg) {
                continue;
            }
            let j = (i + 1) % n;
            let (pa, pb) = (seg.start(), seg.end());
            let (a_axis, b_axis) = (pa.x < TOL, pb.x < TOL);

            // Sweep edge at a profile vertex between two stations.
            let mut sweep = |body: &mut Body, vert: usize, p: Vec2, k: usize| -> Coedge {
                *sweeps.entry((w, vert, k)).or_insert_with(|| {
                    let circle = Frame { origin: f.origin + f.z * p.y, ..f };
                    let (va, vb) = (vertex_ids[&(w, vert, k)], vertex_ids[&(w, vert, k + 1)]);
                    body.add_edge(
                        Curve::Circle { frame: circle, radius: p.x },
                        sweep_angle(k),
                        sweep_angle(k + 1),
                        va,
                        vb,
                    )
                })
            };

            let surface = surface_of_seg(&seg, &f);
            // Outward lies to the right of travel in the profile plane.
            let mid = seg.at(0.5);
            let out2 = seg.direction(0.5).right();
            let phi_mid = sweep_angle(steps) / (2.0 * steps as f64);
            let r_mid = radial(phi_mid);
            let probe = f.origin + r_mid * mid.x + f.z * mid.y;
            let out3 = r_mid * out2.x + f.z * out2.y;
            let flipped = surface.normal(probe).dot(out3) < 0.0;

            if let Surface::Plane(_) = surface {
                // A flat ring or sector is one face however far it sweeps, which
                // keeps planes free of pointless internal edges.
                let mut along_a = Vec::new();
                let mut along_b = Vec::new();
                for k in 0..steps {
                    if !a_axis {
                        along_a.push(sweep(&mut body, i, pa, k));
                    }
                    if !b_axis {
                        along_b.push(sweep(&mut body, j, pb, k).flipped());
                    }
                }
                along_b.reverse();
                let loops = if full {
                    let (outer, inner) = if pa.x > pb.x { (along_a, along_b) } else { (along_b, along_a) };
                    if inner.is_empty() {
                        vec![outer]
                    } else {
                        vec![outer, inner]
                    }
                } else {
                    let m0 = meridian(&mut body, &mut meridians, &vertex_ids, &f, sweep_angle(0), (w, i, 0), wire);
                    let m1 =
                        meridian(&mut body, &mut meridians, &vertex_ids, &f, sweep_angle(steps), (w, i, steps), wire);
                    let mut lp = along_a;
                    lp.push(m1);
                    lp.extend(along_b);
                    lp.push(m0.flipped());
                    vec![lp]
                };
                body.add_face(surface, flipped, loops, tag(SLOT_SEG_BASE + seg.id));
            } else {
                for k in 0..steps {
                    let mut lp = Vec::new();
                    if !a_axis {
                        lp.push(sweep(&mut body, i, pa, k));
                    }
                    lp.push(meridian(
                        &mut body,
                        &mut meridians,
                        &vertex_ids,
                        &f,
                        sweep_angle(wrap(k + 1)),
                        (w, i, wrap(k + 1)),
                        wire,
                    ));
                    if !b_axis {
                        lp.push(sweep(&mut body, j, pb, k).flipped());
                    }
                    lp.push(
                        meridian(&mut body, &mut meridians, &vertex_ids, &f, sweep_angle(k), (w, i, k), wire).flipped(),
                    );
                    body.add_face(surface, flipped, vec![lp], tag(SLOT_SEG_BASE + seg.id));
                }
            }
        }

        if !full {
            let mut start = Vec::with_capacity(n);
            let mut end = Vec::with_capacity(n);
            for i in 0..n {
                let seg = wire.segs[i];
                let key0 = (w, i, on_axis_key(0, &seg));
                let key1 = (w, i, on_axis_key(steps, &seg));
                let phi1 = if is_on_axis(&seg) { 0.0 } else { sweep_angle(steps) };
                start.push(meridian(&mut body, &mut meridians, &vertex_ids, &f, 0.0, key0, wire));
                end.push(meridian(&mut body, &mut meridians, &vertex_ids, &f, phi1, key1, wire).flipped());
            }
            end.reverse();
            caps_start.push(start);
            caps_end.push(end);
        }
    }

    if !full {
        // Start cap: normal is -Y, with chart x radial and y along the axis, so
        // the profile keeps its own direction.
        let start_frame = Frame { origin: f.origin, x: f.x, y: f.z, z: -f.y };
        body.add_face(Surface::Plane(start_frame), false, caps_start, tag(SLOT_START_CAP));
        let r = radial(angle);
        let end_frame = Frame { origin: f.origin, x: f.z, y: r, z: f.z.cross(r) };
        body.add_face(Surface::Plane(end_frame), false, caps_end, tag(SLOT_END_CAP));
    }

    body.compute_shells();
    validate(&body).map_err(Error::InvalidBody)?;
    Ok(body)
}

/// The surface swept by a profile segment revolving about `f.z`.
fn surface_of_seg(seg: &Seg, f: &Frame) -> Surface {
    match seg.geom {
        SegGeom::Line { a, b } => {
            let (dr, dz) = (b.x - a.x, b.y - a.y);
            if dz.abs() < TOL {
                Surface::Plane(Frame { origin: f.origin + f.z * a.y, ..*f })
            } else if dr.abs() < TOL {
                Surface::Cylinder { frame: *f, radius: a.x }
            } else {
                // Cones are stored widening along their own +z.
                let up = if dr / dz > 0.0 { f.z } else { -f.z };
                Surface::Cone {
                    frame: Frame::new(f.origin + f.z * a.y, up, f.x),
                    radius: a.x,
                    half_angle: (dr / dz).abs().atan(),
                }
            }
        }
        SegGeom::Arc { center, radius, .. } => {
            let frame = Frame { origin: f.origin + f.z * center.y, ..*f };
            if center.x.abs() < TOL {
                Surface::Sphere { frame, radius }
            } else {
                Surface::Torus { frame, major: center.x, minor: radius }
            }
        }
    }
}

// ----- Primitives. All of them are ordinary extrudes and revolves. -----

/// Box with one corner at the origin of `frame`.
pub fn box_solid(frame: &Frame, dx: f64, dy: f64, dz: f64, feature: u32) -> Result<Body> {
    extrude(&Profile::new(Wire::rect(0.0, 0.0, dx, dy)), frame, dz, feature)
}

/// Cylinder with its base centred on the origin of `frame`.
pub fn cylinder(frame: &Frame, radius: f64, height: f64, feature: u32) -> Result<Body> {
    extrude(&Profile::new(Wire::circle(v2(0.0, 0.0), radius)), frame, height, feature)
}

/// Cone frustum about `frame.z`. Either radius may be zero for a true cone.
pub fn cone_frustum(frame: &Frame, r0: f64, r1: f64, height: f64, feature: u32) -> Result<Body> {
    if r0 < 0.0 || r1 < 0.0 || r0 + r1 < MIN_FEATURE || height < MIN_FEATURE {
        return Err(Error::InvalidInput("bad cone dimensions".into()));
    }
    let mut pts = vec![v2(0.0, 0.0)];
    if r0 > TOL {
        pts.push(v2(r0, 0.0));
    }
    if r1 > TOL {
        pts.push(v2(r1, height));
    }
    pts.push(v2(0.0, height));
    if r0 <= TOL {
        // Drop the duplicated apex: the wire starts on the axis at the tip.
        pts.remove(0);
        pts.insert(0, v2(0.0, 0.0));
    }
    revolve(&Profile::new(Wire::polygon(&pts)), frame, TAU, feature)
}

/// Sphere centred on the origin of `frame`.
pub fn sphere(frame: &Frame, radius: f64, feature: u32) -> Result<Body> {
    if radius < MIN_FEATURE {
        return Err(Error::InvalidInput("sphere radius is too small".into()));
    }
    let arc = Wire {
        segs: vec![
            Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius, a0: -PI / 2.0, a1: PI / 2.0 }, id: 0 },
            Seg { geom: SegGeom::Line { a: v2(0.0, radius), b: v2(0.0, -radius) }, id: 1 },
        ],
    };
    revolve(&Profile::new(arc), frame, TAU, feature)
}

/// Torus about `frame.z`, centred on the origin of `frame`.
pub fn torus(frame: &Frame, major: f64, minor: f64, feature: u32) -> Result<Body> {
    if minor < MIN_FEATURE || major < minor + MIN_FEATURE {
        return Err(Error::InvalidInput("torus needs major > minor > 0".into()));
    }
    revolve(&Profile::new(Wire::circle(v2(major, 0.0), minor)), frame, TAU, feature)
}
