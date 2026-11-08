//! Randomized Boolean testing, shared by the test suite and `examples/fuzz.rs`.
//!
//! Each case is replayable from `(category, seed)`. Every result must pass three
//! independent checks:
//!
//! * validity: the validator, and a watertight mesh;
//! * volume: `V(A+B) + V(A&B) = V(A) + V(B)` and `V(A-B) + V(A&B) = V(A)`;
//! * membership: at random points, each result must agree with what the inputs
//!   say that point should be. This catches a wrong result with a plausible volume.
//!
//! An `Err` from the kernel is an honest refusal and is counted separately. A
//! result that passes validation but fails a check is the serious kind of bug.

use crate::boolean::classify::{BodyLoc, Index};
use crate::boolean::{boolean, BoolOp};
use crate::build::*;
use crate::mass::mass_properties;
use crate::math::*;
use crate::profile::{PathBuilder, Profile, Seg, SegGeom, Wire};
use crate::spline::Spline;
use crate::tess::{tessellate, TessParams};
use crate::topo::Body;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Category {
    /// Boxes on an integer grid: coplanar faces and shared edges everywhere.
    GridBoxes,
    /// Boxes in arbitrary orientations.
    RotatedBoxes,
    /// Prisms and plates with holes on a grid.
    GridPrisms,
    /// Any primitive in an arbitrary position.
    GeneralPrimitives,
    /// Primitives aligned with the axes on a grid: coaxial, flush and tangent.
    AlignedPrimitives,
    /// A free-form spline or ellipse extruded, against a primitive or another of its kind. Last, because
    /// a category's number seeds its cases and the others must keep theirs.
    SplineExtrusions,
    /// A spline profile revolved, against a primitive or another of its kind.
    SplineRevolves,
}

pub const SPLINE_CATEGORIES: [Category; 2] = [Category::SplineExtrusions, Category::SplineRevolves];

pub const ALL_CATEGORIES: [Category; 5] = [
    Category::GridBoxes,
    Category::RotatedBoxes,
    Category::GridPrisms,
    Category::GeneralPrimitives,
    Category::AlignedPrimitives,
];

#[derive(Clone, Debug, Default)]
pub struct Outcome {
    /// Kernel refused with an error: `(op, message)`.
    pub refusals: Vec<(String, String)>,
    /// A returned result was wrong.
    pub wrong: Vec<String>,
}

fn random_frame(rng: &mut Rng, reach: f64) -> Frame {
    let origin = v3(rng.range(-reach, reach), rng.range(-reach, reach), rng.range(-reach, reach));
    Frame::new(origin, rng.unit_vector(), rng.unit_vector())
}

fn grid(rng: &mut Rng, lo: i64, hi: i64) -> f64 {
    rng.int(lo, hi) as f64
}

/// A closed outline with spline sides, around the origin, roughly the given size.
fn spline_outline(rng: &mut Rng, size: f64) -> Option<Wire> {
    let whole = |s: Spline| Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(s)), id: 0 }] };
    match rng.int(0, 2) {
        0 => {
            let (a, b) = (rng.range(0.4, 1.0) * size, rng.range(0.4, 1.0) * size);
            let turn = rng.range(0.0, TAU);
            let (x, y) = (v3(turn.cos(), turn.sin(), 0.0), v3(-turn.sin(), turn.cos(), 0.0));
            Spline::ellipse_arc(Vec3::ZERO, x, y, a, b, 0.0, TAU).ok().map(whole)
        }
        1 => {
            // A flat bottom with a wavy top.
            let (w, h) = (rng.range(0.8, 1.6) * size, rng.range(0.5, 1.2) * size);
            let count = rng.int(3, 6);
            let mut points = vec![v3(w, 0.0, 0.0)];
            for i in 1..count {
                let f = i as f64 / count as f64;
                points.push(v3(w * (1.0 - 2.0 * f), h * rng.range(0.5, 1.2), 0.0));
            }
            points.push(v3(-w, 0.0, 0.0));
            let top = Spline::through(&points).ok()?;
            Some(PathBuilder::new(v2(w, 0.0)).spline_to(top).line_to(v2(w, 0.0)).close())
        }
        _ => {
            // A blob: points all the way round, with a corner where it closes.
            let count = rng.int(4, 8);
            let start = rng.range(0.0, TAU);
            let mut points = Vec::new();
            for i in 0..count {
                let a = start + TAU * i as f64 / count as f64;
                let r = rng.range(0.6, 1.0) * size;
                points.push(v3(r * a.cos(), r * a.sin(), 0.0));
            }
            points.push(points[0]);
            Spline::through(&points).ok().map(whole)
        }
    }
}

/// A spline profile to revolve, on the positive side of the axis.
fn revolve_profile(rng: &mut Rng, size: f64) -> Option<Profile> {
    let count = rng.int(3, 5);
    let h = rng.range(0.8, 2.0) * size;
    let mut points = Vec::new();
    for i in 0..=count {
        let f = i as f64 / count as f64;
        points.push(v3(rng.range(0.4, 1.0) * size, f * h, 0.0));
    }
    let bottom = points[0];
    let top = points[points.len() - 1];
    let curve = Spline::through(&points).ok()?;
    let wire = match rng.int(0, 1) {
        // Solid to the axis.
        0 => PathBuilder::new(v2(bottom.x, 0.0)).spline_to(curve).line_to(v2(0.0, top.y)).line_to(v2(0.0, 0.0)).close(),
        // A sleeve, not reaching the axis.
        _ => {
            let inner = 0.2 * size;
            PathBuilder::new(v2(bottom.x, 0.0)).spline_to(curve).line_to(v2(inner, top.y)).line_to(v2(inner, 0.0)).close()
        }
    };
    Some(Profile::new(wire))
}

/// A shape from a category, built from the given generator.
pub fn make_shape(category: Category, rng: &mut Rng) -> Option<Body> {
    let body = match category {
        Category::SplineExtrusions => {
            let f = random_frame(rng, 3.0);
            let size = rng.range(1.5, 4.0);
            let wire = spline_outline(rng, size)?;
            extrude(&Profile::new(wire), &f, rng.range(1.0, 6.0), 1)
        }
        Category::SplineRevolves => {
            let f = random_frame(rng, 3.0);
            let size = rng.range(1.5, 4.0);
            let profile = revolve_profile(rng, size)?;
            let angle = if rng.int(0, 2) == 0 { rng.range(1.0, 5.5) } else { TAU };
            revolve(&profile, &f, angle, 1)
        }
        Category::GridBoxes => {
            let f = Frame::translation(v3(grid(rng, 0, 3), grid(rng, 0, 3), grid(rng, 0, 3)));
            box_solid(&f, grid(rng, 1, 3), grid(rng, 1, 3), grid(rng, 1, 3), 1)
        }
        Category::RotatedBoxes => {
            let f = random_frame(rng, 3.0);
            box_solid(&f, rng.range(1.0, 6.0), rng.range(1.0, 6.0), rng.range(1.0, 6.0), 1)
        }
        Category::GridPrisms => {
            let f = Frame::translation(v3(grid(rng, 0, 3), grid(rng, 0, 3), grid(rng, 0, 3)));
            let h = grid(rng, 1, 3);
            match rng.int(0, 2) {
                0 => {
                    // An L shape.
                    let (w, d) = (grid(rng, 2, 4), grid(rng, 2, 4));
                    let (a, b) = (grid(rng, 1, w as i64 - 1), grid(rng, 1, d as i64 - 1));
                    let wire = Wire::polygon(&[v2(0.0, 0.0), v2(w, 0.0), v2(w, b), v2(a, b), v2(a, d), v2(0.0, d)]);
                    extrude(&Profile::new(wire), &f, h, 1)
                }
                1 => {
                    // A plate with a square hole.
                    let (w, d) = (grid(rng, 3, 5), grid(rng, 3, 5));
                    let p = Profile::new(Wire::rect(0.0, 0.0, w, d)).with_hole(Wire::rect(1.0, 1.0, w - 2.0, d - 2.0));
                    extrude(&p, &f, h, 1)
                }
                _ => {
                    // A triangle with grid corners.
                    let wire = Wire::polygon(&[v2(0.0, 0.0), v2(grid(rng, 2, 4), 0.0), v2(0.0, grid(rng, 2, 4))]);
                    extrude(&Profile::new(wire), &f, h, 1)
                }
            }
        }
        Category::GeneralPrimitives => {
            let f = random_frame(rng, 3.0);
            match rng.int(0, 4) {
                0 => box_solid(&f, rng.range(1.0, 6.0), rng.range(1.0, 6.0), rng.range(1.0, 6.0), 1),
                1 => cylinder(&f, rng.range(0.5, 3.0), rng.range(1.0, 7.0), 1),
                2 => sphere(&f, rng.range(1.0, 4.0), 1),
                3 => cone_frustum(&f, rng.range(0.5, 3.0), rng.range(0.0, 3.0), rng.range(1.0, 6.0), 1),
                _ => torus(&f, rng.range(2.0, 4.0), rng.range(0.4, 1.5), 1),
            }
        }
        Category::AlignedPrimitives => {
            let f = Frame::translation(v3(grid(rng, -2, 2), grid(rng, -2, 2), grid(rng, -2, 2)));
            match rng.int(0, 4) {
                0 => box_solid(&f, grid(rng, 1, 4), grid(rng, 1, 4), grid(rng, 1, 4), 1),
                1 => cylinder(&f, grid(rng, 1, 3), grid(rng, 1, 4), 1),
                2 => sphere(&f, grid(rng, 1, 3), 1),
                3 => cone_frustum(&f, grid(rng, 1, 3), grid(rng, 0, 2), grid(rng, 1, 4), 1),
                _ => torus(&f, grid(rng, 2, 4), 1.0, 1),
            }
        }
    };
    body.ok()
}

fn op_name(op: BoolOp) -> &'static str {
    match op {
        BoolOp::Union => "union",
        BoolOp::Subtract => "subtract",
        BoolOp::Intersect => "intersect",
    }
}

fn is_inside(loc: BodyLoc) -> Option<bool> {
    match loc {
        BodyLoc::Inside => Some(true),
        BodyLoc::Outside => Some(false),
        BodyLoc::OnFace(_) => None,
    }
}

/// Runs all three operations on a pair of shapes and checks every result.
pub fn check_pair(a: &Body, b: &Body, rng: &mut Rng, membership_points: usize) -> Outcome {
    let mut out = Outcome::default();
    let (va, vb) = (mass_properties(a).volume, mass_properties(b).volume);
    let (ia, ib) = (Index::new(a), Index::new(b));

    let mut volumes = [None; 3];
    let mut bounds = ia.bounds;
    bounds.union(&ib.bounds);
    let bounds = bounds.inflated(0.5);

    let scale = bounds.diagonal();
    for (slot, op) in [BoolOp::Union, BoolOp::Subtract, BoolOp::Intersect].into_iter().enumerate() {
        let result = match boolean(a, b, op) {
            Ok(r) => r,
            Err(e) => {
                out.refusals.push((op_name(op).to_string(), e.to_string()));
                continue;
            }
        };
        volumes[slot] = Some(mass_properties(&result).volume);
        let index = Index::new(&result);
        if !result.faces.is_empty() {
            match tessellate(&result, &TessParams::for_size(scale)) {
                Ok(mesh) => {
                    let open = mesh.open_edge_count();
                    if open != 0 {
                        out.wrong.push(format!("{}: mesh has {open} open edges", op_name(op)));
                    }
                }
                Err(e) => out.wrong.push(format!("{}: tessellation failed: {e}", op_name(op))),
            }
        }
        for _ in 0..membership_points {
            let p = v3(
                rng.range(bounds.min.x, bounds.max.x),
                rng.range(bounds.min.y, bounds.max.y),
                rng.range(bounds.min.z, bounds.max.z),
            );
            let (Some(ina), Some(inb)) = (is_inside(ia.locate_in_body(p)), is_inside(ib.locate_in_body(p))) else {
                continue;
            };
            let Some(got) = is_inside(index.locate_in_body(p)) else {
                continue;
            };
            let want = match op {
                BoolOp::Union => ina || inb,
                BoolOp::Subtract => ina && !inb,
                BoolOp::Intersect => ina && inb,
            };
            if got != want {
                out.wrong.push(format!(
                    "{}: point ({:.4}, {:.4}, {:.4}) is {} but should be {} (in A: {ina}, in B: {inb})",
                    op_name(op),
                    p.x,
                    p.y,
                    p.z,
                    if got { "inside" } else { "outside" },
                    if want { "inside" } else { "outside" },
                ));
                break;
            }
        }
    }
    let tol = 1e-6 * (va + vb).max(1.0);
    if let (Some(u), Some(i)) = (volumes[0], volumes[2]) {
        if (u + i - va - vb).abs() > tol {
            out.wrong.push(format!("volume: union {u} + intersect {i} != A {va} + B {vb}"));
        }
    }
    if let (Some(d), Some(i)) = (volumes[1], volumes[2]) {
        if (d + i - va).abs() > tol {
            out.wrong.push(format!("volume: subtract {d} + intersect {i} != A {va}"));
        }
    }
    out
}

/// One replayable case.
/// The two shapes of a case, and the generator as it stands after them, which the checks carry on from.
/// Either is `None` when the generator couldn't make that shape, and the case then doesn't count.
pub fn make_pair(category: Category, seed: u64) -> (Option<Body>, Option<Body>, Rng) {
    let mut rng = Rng::new(seed ^ ((category as u64 + 1) << 40));
    let a = make_shape(category, &mut rng);
    // A spline shape meets a primitive more often than another spline.
    let other = if SPLINE_CATEGORIES.contains(&category) && rng.int(0, 2) > 0 { Category::GeneralPrimitives } else { category };
    let b = make_shape(other, &mut rng);
    (a, b, rng)
}

pub fn run_case(category: Category, seed: u64, membership_points: usize) -> Outcome {
    let (a, b, mut rng) = make_pair(category, seed);
    let (Some(a), Some(b)) = (a, b) else {
        return Outcome::default();
    };
    check_pair(&a, &b, &mut rng, membership_points)
}
