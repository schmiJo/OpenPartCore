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
    /// An estimate of how far `volume` could be from the true value because of numerical integration, on
    /// a body with spline faces or traced intersection curves. It's the difference between integrating
    /// with two rules, so it's an estimate and not a proof, but tests hold it against known volumes.
    /// Zero on a body with neither, whose volume is good to rounding.
    pub volume_error: f64,
}

fn gauss_12() -> &'static [(f64, f64)] {
    static RULE: std::sync::OnceLock<Vec<(f64, f64)>> = std::sync::OnceLock::new();
    RULE.get_or_init(|| gauss_legendre(12))
}

/// Where to break an edge's parameter range for quadrature. Each piece must be smooth,
/// so a sampled curve breaks at every sample. Its range can start mid-segment once the
/// edge has been split, so the breaks follow the samples rather than the range.
fn quadrature_breaks(curve: &Curve, t0: f64, t1: f64) -> Vec<f64> {
    match curve {
        Curve::Spline(s) => {
            let mut breaks = vec![t0];
            breaks.extend(s.breaks_between(t0, t1));
            breaks.push(t1);
            breaks
        }
        Curve::Sampled(_) => {
            let mut breaks = vec![t0];
            let mut k = t0.floor() + 1.0;
            while k < t1 - 1e-9 {
                if k > t0 + 1e-9 {
                    breaks.push(k);
                }
                k += 1.0;
            }
            breaks.push(t1);
            breaks
        }
        _ => {
            let pieces = match curve {
                Curve::Line { .. } => 2,
                _ => (((t1 - t0) / (PI / 6.0)).ceil() as usize).max(1),
            };
            (0..=pieces).map(|i| t0 + (t1 - t0) * i as f64 / pieces as f64).collect()
        }
    }
}

/// Calls `f(point, tangent-with-weight)` at quadrature nodes along a coedge, in
/// the direction of the loop. The tangent is `dp/dt * weight` with the sign of
/// travel, so summing `g(p) * tangent` integrates `g dp`.
fn for_each_boundary_node(body: &Body, c: Coedge, rule: &[(f64, f64)], mut f: impl FnMut(Vec3, Vec3)) {
    let e = &body.edges[c.edge];
    let sign = if c.reversed { -1.0 } else { 1.0 };
    // Both faces beside an edge use the same nodes. On a traced curve each one costs many surface distances,
    // so it's worth keeping them for the second face. One edge is kept per thread, since faces rarely share more.
    if matches!(e.curve, Curve::Sampled(_)) {
        let nodes = traced_nodes(body, c.edge, e, rule);
        for &(p, d) in nodes.iter() {
            f(p, d * sign);
        }
        return;
    }
    for w in quadrature_breaks(&e.curve, e.t0, e.t1).windows(2) {
        let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
        for &(x, weight) in rule {
            let t = mid + half * x;
            f(e.curve.eval(t), e.curve.derivative(t) * (weight * half * sign));
        }
    }
}

type TracedNodes = std::sync::Arc<Vec<(Vec3, Vec3)>>;

thread_local! {
    /// Nodes of traced edges for the body being measured, by the edge's address and the rule's length.
    static TRACED: std::cell::RefCell<crate::hash::HashMap<(usize, usize, usize), TracedNodes>> = std::cell::RefCell::new(Default::default());
    /// Whether traced edges only need to be roughly right, as they do for telling which way a shell faces.
    static ROUGHLY: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// How many pieces a traced edge is taken in when it only has to be roughly right, whatever its samples.
const ROUGH_PIECES: usize = 12;

/// Traces edges roughly until dropped, even if what's being measured panics and is caught further up.
struct Roughly;

impl Roughly {
    fn start() -> Roughly {
        forget_traced_nodes();
        ROUGHLY.with(|r| r.set(true));
        Roughly
    }
}

impl Drop for Roughly {
    fn drop(&mut self) {
        ROUGHLY.with(|r| r.set(false));
        forget_traced_nodes();
    }
}

fn traced_nodes(body: &Body, edge: usize, e: &crate::topo::Edge, rule: &[(f64, f64)]) -> TracedNodes {
    let key = (body as *const Body as usize, edge, rule.len());
    if let Some(found) = TRACED.with(|t| t.borrow().get(&key).cloned()) {
        return found;
    }
    let mut nodes = Vec::new();
    if ROUGHLY.with(|r| r.get()) {
        let (a, b) = (e.t0, e.t1);
        for k in 0..ROUGH_PIECES {
            let (from, to) = (a + (b - a) * k as f64 / ROUGH_PIECES as f64, a + (b - a) * (k + 1) as f64 / ROUGH_PIECES as f64);
            let (mid, half) = ((from + to) / 2.0, (to - from) / 2.0);
            for &(x, weight) in gauss_legendre(2).iter() {
                let t = mid + half * x;
                nodes.push((e.curve.eval(t), e.curve.derivative(t) * (weight * half)));
            }
        }
        let nodes = std::sync::Arc::new(nodes);
        TRACED.with(|t| {
            t.borrow_mut().insert(key, nodes.clone());
        });
        return nodes;
    }
    // Between its samples and the knot lines it crosses, a traced curve's pieces are short and smooth, so a third
    // of the points does it, and on a free-form surface each point costs enough to be worth saving. The comparison
    // between the two rules still reports what's left.
    let on_free = matches!(&e.curve, Curve::Sampled(c) if c.touches_free_form());
    let fewer = on_free.then(|| gauss_legendre(rule.len() / 3));
    let rule = fewer.as_deref().unwrap_or(rule);
    for w in traced_breaks(e).windows(2) {
        let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
        for &(x, weight) in rule {
            let t = mid + half * x;
            nodes.push((e.curve.eval(t), e.curve.derivative(t) * (weight * half)));
        }
    }
    let nodes = std::sync::Arc::new(nodes);
    TRACED.with(|t| {
        t.borrow_mut().insert(key, nodes.clone());
    });
    nodes
}

/// Where to break a traced edge for quadrature: at its samples, and wherever it crosses a knot line of a free-form
/// surface it's on. What's integrated has a kink across one of those, since a surface's speed can jump at a knot,
/// and a Gauss rule across a kink loses far more than a few points can buy back.
fn traced_breaks(e: &crate::topo::Edge) -> Vec<f64> {
    let mut breaks = quadrature_breaks(&e.curve, e.t0, e.t1);
    let Curve::Sampled(c) = &e.curve else {
        return breaks;
    };
    for surface in [&c.surfaces.0, &c.surfaces.1] {
        let Surface::Free { frame, patch } = surface else { continue };
        let at = |t: f64| {
            let foot = patch.foot(frame.to_local(e.curve.eval(t)));
            [foot.u, foot.v]
        };
        let knots = patch.interior_knots();
        let mut crossings = Vec::new();
        for w in breaks.windows(2) {
            let (a, b) = (at(w[0]), at(w[1]));
            for (direction, list) in knots.iter().enumerate() {
                for &k in list.iter() {
                    if (a[direction] - k) * (b[direction] - k) >= 0.0 {
                        continue;
                    }
                    let (mut lo, mut hi, mut below) = (w[0], w[1], a[direction] < k);
                    for _ in 0..40 {
                        let m = (lo + hi) / 2.0;
                        if (at(m)[direction] < k) == below {
                            lo = m;
                            below = at(m)[direction] < k;
                        } else {
                            hi = m;
                        }
                    }
                    crossings.push((lo + hi) / 2.0);
                }
            }
        }
        breaks.extend(crossings);
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
    }
    breaks
}

fn forget_traced_nodes() {
    TRACED.with(|t| t.borrow_mut().clear());
}

fn face_integrals(body: &Body, face: &Face, reference: Vec3) -> (f64, f64) {
    face_integrals_with(body, face, reference, gauss_24())
}

fn face_integrals_with(body: &Body, face: &Face, reference: Vec3, along_edges: &[(f64, f64)]) -> (f64, f64) {
    let sign = if face.flipped { -1.0 } else { 1.0 };
    let surface = &face.surface;
    let chart = &face.chart;

    match surface {
        Surface::Extruded { frame, curve } => {
            // In terms of the curve's parameter t and the height z, the flux density is the same at every
            // height: (C - r) . right(C'), since the normal has no part along z. So the inner integral is
            // along z and needs no quadrature, and the boundary integral runs over t. A ruling has dt = 0
            // and adds nothing. Region to boundary is Green's theorem for a counter-clockwise loop in (t, z).
            let r = frame.to_local(reference);
            let (mut flux, mut area) = (0.0, 0.0);
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
                        let across = v2(dp.dot(frame.x), dp.dot(frame.y));
                        if across.dot(across) <= 1e-28 * dp.norm2() {
                            return;
                        }
                        let l = frame.to_local(p);
                        let foot = curve.foot(v2(l.x, l.y));
                        let d = curve.derivative(foot.t);
                        let dt = across.dot(d) / d.dot(d);
                        let height = l.z - r.z;
                        flux -= (foot.at - v2(r.x, r.y)).dot(d.right()) * height * dt;
                        area -= d.norm() * height * dt;
                    });
                }
            }
            (flux / 3.0, area * sign)
        }
        Surface::Revolved { frame, curve } => {
            // In terms of the angle and the curve's parameter t, the flux density is
            // rho ((rho + d . radial) h' - (h + d . z) rho'), for a curve (rho, h) and the frame's origin at d
            // from the reference. That integrates over the angle in closed form, measured from the chart's
            // center, and the boundary integral runs over t. A sweep around the axis has dt = 0 and adds
            // nothing, and a tip adds nothing since rho is zero there.
            let d = frame.to_local(frame.origin) - frame.to_local(reference);
            let u0 = chart.u0;
            let (s0, c0) = u0.sin_cos();
            let (mut flux, mut area) = (0.0, 0.0);
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
                        let l = frame.to_local(p);
                        let rho = l.x.hypot(l.y);
                        if rho < 1e-12 {
                            return;
                        }
                        let (cu, su) = (l.x / rho, l.y / rho);
                        let radial = frame.x * cu + frame.y * su;
                        let along = v2(dp.dot(radial), dp.dot(frame.z));
                        if along.dot(along) <= 1e-28 * dp.norm2() {
                            return;
                        }
                        let foot = curve.foot(v2(rho, l.z));
                        let v = curve.derivative(foot.t);
                        let dt = along.dot(v) / v.dot(v);
                        let (r, h) = (foot.at.x, foot.at.y);
                        let phi = wrap_pi(l.y.atan2(l.x) - u0);
                        let inner = r * ((r * v.y - (h + d.z) * v.x) * phi + v.y * (d.x * (su - s0) - d.y * (cu - c0)));
                        flux += inner * dt;
                        area += r * v.norm() * phi * dt;
                    });
                }
            }
            (flux / 3.0, area * sign)
        }
        Surface::Plane(frame) => {
            // Flux is constant on a plane, so only the boundary area integral is
            // needed: area = integral(x dy) in the plane frame.
            let d = (frame.origin - reference).dot(frame.z);
            let mut area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
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
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
                        vector_area += (p - reference).cross(dp) * 0.5;
                        // The chart is isometric, so area is plain chart area.
                        let (uv, duv) = chart_velocity(surface, chart, p, dp);
                        chart_area += 0.5 * uv.cross(duv);
                    });
                }
            }
            ((apex - reference).dot(vector_area) / 3.0, chart_area * sign)
        }
        Surface::Torus { frame, major, minor } if *major < 0.0 => {
            // The polar chart has a cone-like kink at the tip, which Gauss rules resolve
            // badly. In (u, v) the integrand is smooth and the meridians have du = 0. The
            // tip becomes a whole side of the region that no edge runs along, so the inner
            // integral starts at the tip: it is zero there, and that side contributes nothing.
            let tip = face.chart.v0;
            let (major, minor) = (*major, *minor);
            let rule = gauss_24();
            let mut flux = 0.0;
            let mut area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
                        let l = frame.to_local(p);
                        let rho = l.x.hypot(l.y);
                        if rho < 1e-9 {
                            return;
                        }
                        let u = l.y.atan2(l.x);
                        let (su, cu) = u.sin_cos();
                        let theta = frame.y * cu - frame.x * su;
                        let radial = frame.x * cu + frame.y * su;
                        let du = dp.dot(theta) / rho;
                        let v = l.z.atan2(rho - major);
                        let (mid, half) = ((v + tip) / 2.0, (v - tip) / 2.0);
                        let mut h_flux = 0.0;
                        let mut h_area = 0.0;
                        for &(x, w) in rule {
                            let (sv, cv) = (mid + half * x).sin_cos();
                            let r = major + minor * cv;
                            let q = frame.origin + radial * r + frame.z * (minor * sv);
                            let n = radial * cv + frame.z * sv;
                            let jacobian = r * minor;
                            h_flux += w * half * (q - reference).dot(n) * jacobian;
                            h_area += w * half * jacobian;
                        }
                        // Green's theorem: the integral over the region is minus the loop
                        // integral of H du for a counter-clockwise loop in (u, v).
                        flux -= h_flux * du;
                        area -= h_area * du;
                    });
                }
            }
            (flux / 3.0, area * sign)
        }
        Surface::Free { patch, .. } => {
            // The same as the general case below, with the inner integral stopping at the knots, where the
            // surface is less smooth than a Gauss rule expects.
            let (lo, hi) = body_chart_range(body, face);
            let x_ref = (lo + hi) / 2.0;
            let knots = patch.chart_breaks_x();
            let rule = gauss_legendre(patch.quadrature_points());
            let inner = |at: Vec2| -> (f64, f64) {
                let (from, to) = (x_ref.min(at.x), x_ref.max(at.x));
                let sense = if at.x >= x_ref { 1.0 } else { -1.0 };
                let mut stops = vec![from];
                stops.extend(knots.iter().copied().filter(|k| *k > from && *k < to));
                stops.push(to);
                let (mut h_flux, mut h_area) = (0.0, 0.0);
                for w in stops.windows(2) {
                    let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
                    for &(x, weight) in &rule {
                        let (q, sx, sy) = surface.chart_derivs(chart, v2(mid + half * x, at.y));
                        let n = sx.cross(sy);
                        h_flux += weight * half * (q - reference).dot(n);
                        h_area += weight * half * n.norm();
                    }
                }
                (sense * h_flux, sense * h_area)
            };
            let (mut flux, mut area) = (0.0, 0.0);
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
                        let (uv, duv) = chart_velocity(surface, chart, p, dp);
                        if duv.y == 0.0 {
                            return;
                        }
                        let (h_flux, h_area) = inner(uv);
                        flux += h_flux * duv.y;
                        area += h_area * duv.y;
                    });
                }
                // Where a side of the chart is all one point in space, a tip, no edge runs along that side, but the
                // region's boundary in the chart does all the same, and it's owed its share. Without this a cone-like
                // face was short by whatever lay between that side and the reference: a quarter of it.
                for (i, &c) in lp.iter().enumerate() {
                    let next = lp[(i + 1) % lp.len()];
                    let tip = body.coedge_point(next, 0.0);
                    let at = |c: Coedge, f: f64| surface.to_chart(chart, body.coedge_point(c, f));
                    let (arrive, leave) = (at(c, 1.0 - 1e-5), at(next, 1e-5));
                    let side = surface.to_chart(chart, tip).x;
                    let span = (leave.y - arrive.y).abs();
                    let on_side = (arrive.x - side).abs() < 1e-2 * span.max(1e-9) && (leave.x - side).abs() < 1e-2 * span.max(1e-9);
                    if !on_side || span <= 1e-3 * (arrive.x - leave.x).abs().max(1e-6) {
                        continue;
                    }
                    let all_one_point = (1..8).all(|k| surface.from_chart(chart, v2(side, arrive.y + (leave.y - arrive.y) * k as f64 / 8.0)).dist(tip) <= 1e-5);
                    if !all_one_point {
                        continue;
                    }
                    let (from, to) = (arrive.y.min(leave.y), arrive.y.max(leave.y));
                    let mut stops = vec![from];
                    stops.extend(patch.chart_breaks_y().into_iter().filter(|k| *k > from && *k < to));
                    stops.push(to);
                    let sense = if leave.y > arrive.y { 1.0 } else { -1.0 };
                    for w in stops.windows(2) {
                        let (mid, half) = ((w[0] + w[1]) / 2.0, (w[1] - w[0]) / 2.0);
                        for &(y, weight) in along_edges {
                            let (h_flux, h_area) = inner(v2(side, mid + half * y));
                            flux += sense * h_flux * weight * half;
                            area += sense * h_area * weight * half;
                        }
                    }
                }
            }
            (flux / 3.0, area * sign)
        }
        _ => {
            let (lo, hi) = body_chart_range(body, face);
            let x_ref = (lo + hi) / 2.0;
            let rule = gauss_24();
            let mut flux = 0.0;
            let mut area = 0.0;
            for lp in &face.loops {
                for &c in lp {
                    for_each_boundary_node(body, c, along_edges, |p, dp| {
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
    // A body at this address before may have been a different body.
    forget_traced_nodes();
    let reference = body.aabb().center();
    let mut out = MassProps::default();
    // An open surface encloses nothing, and the flux through one depends on where it's measured from.
    if body.sheet {
        return sheet_properties(body, reference);
    }
    for face in &body.faces {
        let (v, a) = face_integrals(body, face, reference);
        out.volume += v;
        out.area += a;
    }
    let numeric = body.faces.iter().any(|f| matches!(f.surface, Surface::Extruded { .. } | Surface::Revolved { .. } | Surface::Free { .. }))
        || body.edges.iter().any(|e| matches!(e.curve, Curve::Sampled(_)));
    if numeric {
        let coarse: f64 = body.faces.iter().map(|f| face_integrals_with(body, f, reference, gauss_12()).0).sum();
        // What's left at the coarser rule, plus what rounding can leave behind in a sum of this size.
        out.volume_error = (coarse - out.volume).abs() + 1e-13 * out.volume.abs();
    }
    forget_traced_nodes();
    out
}

/// A sheet's area, and the volume of whichever of its shells do close.
fn sheet_properties(body: &Body, reference: Vec3) -> MassProps {
    let uses = body.edge_uses();
    let closed = |shell: &[usize]| shell.iter().all(|&f| body.faces[f].loops.iter().flatten().all(|c| uses[c.edge].len() == 2));
    let mut out = MassProps::default();
    for shell in &body.shells {
        let counts = closed(shell);
        for &f in shell {
            let (v, a) = face_integrals(body, &body.faces[f], reference);
            out.area += a;
            if counts {
                out.volume += v;
            }
        }
    }
    forget_traced_nodes();
    out
}

/// How far the integrated tangent misses closing around a loop. For a closed loop the
/// exact answer is zero, so this measures the quadrature behind every other number here.
pub fn loop_closure_error(body: &Body, lp: &[Coedge]) -> f64 {
    let mut sum = Vec3::ZERO;
    for &c in lp {
        for_each_boundary_node(body, c, gauss_24(), |_, dp| sum += dp);
    }
    sum.norm()
}

/// Flux contribution to the volume, and area, of each face, about a reference point.
pub fn per_face(body: &Body, reference: Vec3) -> Vec<(f64, f64)> {
    forget_traced_nodes();
    let out = body.faces.iter().map(|f| face_integrals(body, f, reference)).collect();
    forget_traced_nodes();
    out
}

/// What a shell's volume comes to with its traced edges taken roughly, and what all its faces' parts of that add up to
/// regardless of sign, which says how much the first can be trusted.
pub fn rough_shell_volume(body: &Body, shell: usize) -> (f64, f64) {
    let _roughly = Roughly::start();
    let reference = body.aabb().center();
    body.shells[shell].iter().fold((0.0, 0.0), |(sum, parts), &f| {
        let v = face_integrals_with(body, &body.faces[f], reference, gauss_12()).0;
        (sum + v, parts + v.abs())
    })
}

/// Which way a shell faces: positive if it encloses material, and negative if it's a void. That's the sign of its volume,
/// which all a reader needs of it, and on a traced curve the exact volume can be most of what reading a file takes. So it
/// goes by a rough one when that's clear, and otherwise works out the exact one.
pub fn shell_facing(body: &Body, shell: usize) -> f64 {
    let (volume, parts) = rough_shell_volume(body, shell);
    if volume.abs() > CLEARLY * parts {
        volume
    } else {
        shell_volume(body, shell)
    }
}

/// How much of a shell's parts have to survive adding up, for a rough volume's sign to be believed.
const CLEARLY: f64 = 0.1;

/// Signed volume of one shell. Positive encloses material, negative is a void.
pub fn shell_volume(body: &Body, shell: usize) -> f64 {
    forget_traced_nodes();
    let reference = body.aabb().center();
    let out = body.shells[shell].iter().map(|&f| face_integrals(body, &body.faces[f], reference).0).sum();
    forget_traced_nodes();
    out
}
