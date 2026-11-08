//! Volume and area computed from the B-rep itself, not from a mesh.
//!
//! The divergence theorem turns volume into a flux over the faces. On a face,
//! Green's theorem in the chart then turns the area integral into an integral
//! around the boundary, whose integrand is itself a 1D integral. Everything is
//! smooth on each piece, so Gauss-Legendre reaches near machine precision. Exact
//! geometry gives exact-grade numbers here, which lets tests compare against
//! closed-form answers with tight tolerances.

use crate::curve::Curve;
use crate::math::*;
use crate::surface::Surface;
use crate::topo::{Body, Coedge, Face};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassProps {
    pub volume: f64,
    pub area: f64,
}

/// Number of quadrature pieces for an edge.
fn pieces_for(curve: &Curve, t0: f64, t1: f64) -> usize {
    match curve {
        Curve::Line { .. } => 8,
        Curve::Circle { .. } | Curve::Ellipse { .. } => (((t1 - t0) / (PI / 16.0)).ceil() as usize).max(1),
        Curve::Sampled(_) => (((t1 - t0).ceil() as usize) * 2).max(1),
    }
}

/// Calls `f(point, tangent-with-weight)` at quadrature nodes along a coedge, in
/// the direction of the loop. The tangent is `dp/dt * weight` with the sign of
/// travel, so summing `g(p) * tangent` integrates `g dp`.
fn for_each_boundary_node(body: &Body, c: Coedge, mut f: impl FnMut(Vec3, Vec3)) {
    let e = &body.edges[c.edge];
    let rule = gauss_24();
    let pieces = pieces_for(&e.curve, e.t0, e.t1);
    let dt = (e.t1 - e.t0) / pieces as f64;
    let sign = if c.reversed { -1.0 } else { 1.0 };
    for i in 0..pieces {
        let (a, b) = (e.t0 + dt * i as f64, e.t0 + dt * (i + 1) as f64);
        let (mid, half) = ((a + b) / 2.0, (b - a) / 2.0);
        for &(x, w) in rule {
            let t = mid + half * x;
            f(e.curve.eval(t), e.curve.derivative(t) * (w * half * sign));
        }
    }
}

fn face_integrals(body: &Body, face: &Face, reference: Vec3) -> (f64, f64) {
    let sign = if face.flipped { -1.0 } else { 1.0 };
    let surface = &face.surface;
    let chart = &face.chart;

    match surface {
        Surface::Plane(frame) => {
            // Flux is constant on a plane, so only the boundary area integral is
            // needed: area = integral(x dy) in the plane frame.
            let d = (frame.origin - reference).dot(frame.z);
            let mut area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, |p, dp| {
                        let l = frame.to_local(p);
                        area += l.x * dp.dot(frame.y);
                    });
                }
            }
            // `area` is signed by loop direction: negative on flipped faces.
            (d * area / 3.0, area * sign)
        }
        Surface::Cone { .. } => {
            // Rays from the apex lie in the surface, so the flux about the apex
            // is zero. The flux about the reference then reduces to the face's
            // vector area, which Stokes turns into a boundary integral. This
            // sidesteps the chart's kink at the apex.
            let apex = surface.apex().unwrap();
            let mut vector_area = Vec3::ZERO;
            let mut chart_area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, |p, dp| {
                        vector_area += (p - reference).cross(dp) * 0.5;
                        // The chart is isometric, so area is plain chart area.
                        let (uv, duv) = chart_velocity(surface, chart, p, dp);
                        chart_area += 0.5 * uv.cross(duv);
                    });
                }
            }
            ((apex - reference).dot(vector_area) / 3.0, chart_area * sign)
        }
        _ => {
            let (lo, hi) = body_chart_range(body, face);
            let x_ref = (lo + hi) / 2.0;
            let rule = gauss_24();
            let mut flux = 0.0;
            let mut area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, |p, dp| {
                        let (uv, duv) = chart_velocity(surface, chart, p, dp);
                        if duv.y == 0.0 {
                            return;
                        }
                        // Inner integral along chart x at fixed chart y.
                        let (mid, half) = ((uv.x + x_ref) / 2.0, (uv.x - x_ref) / 2.0);
                        let mut h_flux = 0.0;
                        let mut h_area = 0.0;
                        for &(x, w) in rule {
                            let (q, sx, sy) = surface.chart_derivs(chart, v2(mid + half * x, uv.y));
                            let n = sx.cross(sy);
                            h_flux += w * half * (q - reference).dot(n);
                            h_area += w * half * n.norm();
                        }
                        flux += h_flux * duv.y;
                        area += h_area * duv.y;
                    });
                }
            }
            (flux / 3.0, area * sign)
        }
    }
}

/// Chart coordinates of a point, and the chart displacement for a 3D displacement
/// along the surface.
fn chart_velocity(surface: &Surface, chart: &crate::surface::Chart, p: Vec3, dp: Vec3) -> (Vec2, Vec2) {
    let uv = surface.to_chart(chart, p);
    let (_, sx, sy) = surface.chart_derivs(chart, uv);
    // Solve the 2x2 normal equations for dp = a * sx + b * sy.
    let (e, f, g) = (sx.dot(sx), sx.dot(sy), sy.dot(sy));
    let (r1, r2) = (dp.dot(sx), dp.dot(sy));
    let det = e * g - f * f;
    (uv, v2((r1 * g - r2 * f) / det, (e * r2 - f * r1) / det))
}

fn body_chart_range(body: &Body, face: &Face) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for lp in &face.loops {
        for &c in lp {
            for i in 0..=8 {
                let uv = face.surface.to_chart(&face.chart, body.coedge_point(c, i as f64 / 8.0));
                lo = lo.min(uv.x);
                hi = hi.max(uv.x);
            }
        }
    }
    (lo, hi)
}

/// Volume and surface area of a body.
pub fn mass_properties(body: &Body) -> MassProps {
    let reference = body.aabb().center();
    let mut out = MassProps::default();
    for face in &body.faces {
        let (v, a) = face_integrals(body, face, reference);
        out.volume += v;
        out.area += a;
    }
    out
}

/// Signed volume of one shell. Positive encloses material, negative is a void.
pub fn shell_volume(body: &Body, shell: usize) -> f64 {
    let reference = body.aabb().center();
    body.shells[shell]
        .iter()
        .map(|&f| face_integrals(body, &body.faces[f], reference).0)
        .sum()
}
