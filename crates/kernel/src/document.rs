//! The parametric layer: a document holding parameters and an ordered list of features,
//! all of it plain data. A document can be saved, loaded, edited, and driven by a person,
//! a program or an AI agent through the same operations.
//!
//! **Names.** A feature has an id that never changes and is never reused. A face is named
//! by the feature that made it and a slot in that feature, written `f2:end` or `f2:side.3`,
//! and Booleans preserve these names. So "the edge between `f2:end` and `f2:side.0`" still
//! means the same edge after a dimension change, even when the edge has been split or
//! renumbered by features in between.
//!
//! **Rebuilding.** Each step has a cache key: a hash of its own resolved inputs and the keys
//! of what it depends on. A step whose key is unchanged reuses its previous result. Because
//! the key covers everything a result depends on, this one rule is correct for dimension
//! changes, edits, suppression and reordering alike.

use crate::blend::{chamfer, fillet_with, BlendOptions};
use crate::boolean::{boolean_with, BoolOp, Checks};
use crate::build::{
    box_solid, cone_frustum, cylinder, extrude, revolve, sphere, torus, SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP,
};
use crate::error::{Error, Result};
use crate::expr::{Env, Expr};
use crate::json::{self, Value};
use crate::math::*;
use crate::profile::{PathBuilder, Profile, Wire};
use crate::topo::{Body, FaceTag};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

/// The file format's name and the version this code writes.
pub const FORMAT: &str = "coreshape-document";
pub const FORMAT_VERSION: f64 = 1.0;

/// Slot offset per closed region of a sketch. Leaves room for 999 holes per region.
const REGION_SLOTS: u32 = 1_000_000;
/// The most regions one sketch may make, which keeps every slot within a `u32`.
const MAX_REGIONS: usize = 4000;
/// Slot offset per hole within a region, matching `Profile::with_hole`.
const HOLE_SLOTS: u32 = 1000;

// ----- Names -----

/// A feature's permanent id.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FeatureId(pub u32);

/// A stable name for a face, or for all the pieces it was split into.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FaceRef {
    pub feature: FeatureId,
    pub slot: u32,
}

impl FeatureId {
    pub fn start_cap(self) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_START_CAP }
    }

    pub fn end_cap(self) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_END_CAP }
    }

    /// The face swept from a segment of the sketch's outer wire.
    pub fn side(self, segment: u32) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_SEG_BASE + segment }
    }

    /// The face swept from a segment of a hole in the sketch's first region.
    pub fn hole_side(self, hole: u32, segment: u32) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_SEG_BASE + (hole + 1) * HOLE_SLOTS + segment }
    }

    /// The face swept from a segment of the outer wire of one region of the sketch.
    pub fn region_side(self, region: u32, segment: u32) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_SEG_BASE + segment + region * REGION_SLOTS }
    }
}

impl FaceRef {
    pub fn tag(self) -> FaceTag {
        FaceTag { feature: self.feature.0, slot: self.slot }
    }

    pub fn from_tag(tag: FaceTag) -> FaceRef {
        FaceRef { feature: FeatureId(tag.feature), slot: tag.slot }
    }

    /// The name used in files and tools: `f2:end`, `f2:side.3`, `f2:hole.0.side.1`,
    /// `f2:region.1.side.0`.
    pub fn name(self) -> String {
        let f = self.feature.0;
        match self.slot {
            SLOT_START_CAP => format!("f{f}:start"),
            SLOT_END_CAP => format!("f{f}:end"),
            slot => {
                let rest = slot - SLOT_SEG_BASE;
                let region = rest / REGION_SLOTS;
                let hole = (rest % REGION_SLOTS) / HOLE_SLOTS;
                let segment = rest % HOLE_SLOTS;
                let mut out = format!("f{f}:");
                if region > 0 {
                    out.push_str(&format!("region.{region}."));
                }
                if hole > 0 {
                    out.push_str(&format!("hole.{}.", hole - 1));
                }
                out.push_str(&format!("side.{segment}"));
                out
            }
        }
    }

    pub fn parse(text: &str) -> Result<FaceRef> {
        let bad = || Error::InvalidInput(format!("\"{text}\" is not a face name; they look like f2:end or f2:side.3"));
        let (feature, slot) = text.split_once(':').ok_or_else(bad)?;
        let id: u32 = feature.strip_prefix('f').and_then(|n| n.parse().ok()).ok_or_else(bad)?;
        let feature = FeatureId(id);
        match slot {
            "start" => return Ok(feature.start_cap()),
            "end" => return Ok(feature.end_cap()),
            _ => {}
        }
        let parts: Vec<&str> = slot.split('.').collect();
        let (mut region, mut hole, mut segment) = (0u32, 0u32, None);
        let mut i = 0;
        while i + 1 < parts.len() {
            let n: u32 = parts[i + 1].parse().map_err(|_| bad())?;
            match parts[i] {
                "region" => region = n,
                "hole" => hole = n.checked_add(1).ok_or_else(bad)?,
                "side" => segment = Some(n),
                _ => return Err(bad()),
            }
            i += 2;
        }
        let segment = segment.filter(|_| i == parts.len()).ok_or_else(bad)?;
        if segment >= HOLE_SLOTS || hole >= REGION_SLOTS / HOLE_SLOTS || region as usize >= MAX_REGIONS {
            return Err(bad());
        }
        Ok(FaceRef { feature, slot: SLOT_SEG_BASE + region * REGION_SLOTS + hole * HOLE_SLOTS + segment })
    }
}

// ----- Parameters -----

/// A named value. The expression may refer to other parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: Expr,
}

// ----- Sketches -----

pub type Point2 = [Expr; 2];
pub type Point3 = [Expr; 3];

fn p2(x: impl Into<Expr>, y: impl Into<Expr>) -> Point2 {
    [x.into(), y.into()]
}

pub fn point(x: impl Into<Expr>, y: impl Into<Expr>) -> Point2 {
    p2(x, y)
}

pub fn point3(x: impl Into<Expr>, y: impl Into<Expr>, z: impl Into<Expr>) -> Point3 {
    [x.into(), y.into(), z.into()]
}

/// Where a sketch or solid sits in space.
#[derive(Clone, Debug, PartialEq)]
pub struct Placement {
    pub origin: Point3,
    /// Normal of the sketch plane, or the axis of a solid.
    pub z: Point3,
    /// Direction of the sketch's x axis. Made perpendicular to `z`.
    pub x: Point3,
}

impl Placement {
    /// The XY plane, offset along Z.
    pub fn xy(z: impl Into<Expr>) -> Placement {
        Placement { origin: point3(0.0, 0.0, z), z: point3(0.0, 0.0, 1.0), x: point3(1.0, 0.0, 0.0) }
    }

    pub fn at(origin: Point3) -> Placement {
        Placement { origin, z: point3(0.0, 0.0, 1.0), x: point3(1.0, 0.0, 0.0) }
    }

    /// Placed with the given axis and x direction.
    pub fn new(origin: Point3, z: Point3, x: Point3) -> Placement {
        Placement { origin, z, x }
    }

    /// The placement of a frame computed in code.
    pub fn from_frame(f: &Frame) -> Placement {
        Placement {
            origin: point3(f.origin.x, f.origin.y, f.origin.z),
            z: point3(f.z.x, f.z.y, f.z.z),
            x: point3(f.x.x, f.x.y, f.x.z),
        }
    }

    fn resolve(&self, env: &Env) -> Result<Frame> {
        let v = |p: &Point3| -> Result<Vec3> { Ok(v3(num(&p[0], env)?, num(&p[1], env)?, num(&p[2], env)?)) };
        let (o, z, x) = (v(&self.origin)?, v(&self.z)?, v(&self.x)?);
        for d in [z, x] {
            // Written so that a length that overflowed fails too.
            if !(d.norm() >= 1e-12 && d.norm() <= 1e150) {
                return Err(Error::InvalidInput("a placement direction must have a sensible, non-zero length".into()));
            }
        }
        if z.cross(x).norm() < 1e-9 * z.norm() * x.norm() {
            return Err(Error::InvalidInput("a placement's z and x directions are parallel; state an x that isn't".into()));
        }
        Ok(Frame::new(o, z, x))
    }
}

/// One step along a sketch path.
#[derive(Clone, Debug, PartialEq)]
pub enum Move {
    Line { to: Point2 },
    Arc { to: Point2, center: Point2, ccw: bool },
}

/// A closed curve in a sketch. Segments are numbered in order, starting at 0. Rectangles
/// number bottom, right, top, left. A circle's two halves share the number 0.
#[derive(Clone, Debug, PartialEq)]
pub enum Shape {
    Rect { x: Expr, y: Expr, w: Expr, h: Expr },
    RoundedRect { cx: Expr, cy: Expr, w: Expr, h: Expr, r: Expr },
    Circle { cx: Expr, cy: Expr, r: Expr },
    Polygon { points: Vec<Point2> },
    Path { start: Point2, moves: Vec<Move> },
}

impl Shape {
    pub fn rect(x: impl Into<Expr>, y: impl Into<Expr>, w: impl Into<Expr>, h: impl Into<Expr>) -> Shape {
        Shape::Rect { x: x.into(), y: y.into(), w: w.into(), h: h.into() }
    }

    /// A rectangle centred on the sketch origin.
    pub fn centered(w: &str, h: &str) -> Shape {
        Shape::Rect {
            x: format!("-({w}) / 2").as_str().into(),
            y: format!("-({h}) / 2").as_str().into(),
            w: w.into(),
            h: h.into(),
        }
    }

    pub fn circle(cx: impl Into<Expr>, cy: impl Into<Expr>, r: impl Into<Expr>) -> Shape {
        Shape::Circle { cx: cx.into(), cy: cy.into(), r: r.into() }
    }

    pub fn rounded(cx: impl Into<Expr>, cy: impl Into<Expr>, w: impl Into<Expr>, h: impl Into<Expr>, r: impl Into<Expr>) -> Shape {
        Shape::RoundedRect { cx: cx.into(), cy: cy.into(), w: w.into(), h: h.into(), r: r.into() }
    }

    pub fn polygon(points: Vec<Point2>) -> Shape {
        Shape::Polygon { points }
    }

    fn resolve(&self, env: &Env) -> Result<Wire> {
        let at = |p: &Point2| -> Result<Vec2> { Ok(v2(num(&p[0], env)?, num(&p[1], env)?)) };
        Ok(match self {
            Shape::Rect { x, y, w, h } => Wire::rect(num(x, env)?, num(y, env)?, num(w, env)?, num(h, env)?),
            Shape::RoundedRect { cx, cy, w, h, r } => {
                Wire::rounded_rect(num(w, env)?, num(h, env)?, num(r, env)?).translated(v2(num(cx, env)?, num(cy, env)?))
            }
            Shape::Circle { cx, cy, r } => Wire::circle(v2(num(cx, env)?, num(cy, env)?), num(r, env)?),
            Shape::Polygon { points } => {
                if points.len() < 3 {
                    return Err(Error::InvalidProfile("a polygon needs at least three points".into()));
                }
                let pts = points.iter().map(at).collect::<Result<Vec<Vec2>>>()?;
                Wire::polygon(&pts)
            }
            Shape::Path { start, moves } => {
                let mut path = PathBuilder::new(at(start)?);
                for m in moves {
                    path = match m {
                        Move::Line { to } => path.line_to(at(to)?),
                        Move::Arc { to, center, ccw } => path.arc_to(at(to)?, at(center)?, *ccw),
                    };
                }
                path.close()
            }
        })
    }
}

/// Repeats a region within a sketch.
#[derive(Clone, Debug, PartialEq)]
pub enum Repeat {
    /// Copies around a circle about the sketch origin, each moved out to `radius` at evenly
    /// spaced angles. The region is drawn as if centred on the sketch origin.
    Circular { count: Expr, radius: Expr, start_degrees: Expr },
    /// A grid of copies, starting where the region is drawn.
    Grid { nx: Expr, ny: Expr, dx: Expr, dy: Expr },
}

/// A closed area of a sketch: an outer wire, and any holes.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub outer: Shape,
    pub holes: Vec<Shape>,
    pub repeat: Option<Repeat>,
}

impl Region {
    pub fn new(outer: Shape) -> Region {
        Region { outer, holes: Vec::new(), repeat: None }
    }

    pub fn hole(mut self, hole: Shape) -> Region {
        self.holes.push(hole);
        self
    }

    pub fn repeated(mut self, repeat: Repeat) -> Region {
        self.repeat = Some(repeat);
        self
    }
}

fn count_of(e: &Expr, env: &Env, what: &str) -> Result<usize> {
    let n = num(e, env)?.round();
    if !(1.0..=MAX_REGIONS as f64).contains(&n) {
        return Err(Error::InvalidInput(format!("{what} of {n} is out of range: it must be from 1 to {MAX_REGIONS}")));
    }
    Ok(n as usize)
}

fn resolve_regions(regions: &[Region], env: &Env) -> Result<Vec<Profile>> {
    let mut out = Vec::new();
    for region in regions {
        let mut base = Profile::new(region.outer.resolve(env)?);
        for h in &region.holes {
            base = base.with_hole(h.resolve(env)?);
        }
        let shift = |by: Vec2| Profile {
            outer: base.outer.translated(by),
            holes: base.holes.iter().map(|h| h.translated(by)).collect(),
        };
        match &region.repeat {
            None => out.push(base.clone()),
            Some(Repeat::Circular { count, radius, start_degrees }) => {
                let n = count_of(count, env, "a circular count")?;
                let (r, start) = (num(radius, env)?, num(start_degrees, env)?.to_radians());
                for i in 0..n {
                    let a = start + TAU * i as f64 / n as f64;
                    out.push(shift(v2(r * a.cos(), r * a.sin())));
                }
            }
            Some(Repeat::Grid { nx, ny, dx, dy }) => {
                let (nx, ny) = (count_of(nx, env, "a grid count")?, count_of(ny, env, "a grid count")?);
                if nx * ny > MAX_REGIONS {
                    return Err(Error::InvalidInput(format!("a grid of {nx} by {ny} is more than the {MAX_REGIONS} regions a sketch can make")));
                }
                let (dx, dy) = (num(dx, env)?, num(dy, env)?);
                for i in 0..nx {
                    for j in 0..ny {
                        out.push(shift(v2(dx * i as f64, dy * j as f64)));
                    }
                }
            }
        }
        if out.len() > MAX_REGIONS {
            return Err(Error::InvalidInput(format!("a sketch can make at most {MAX_REGIONS} regions")));
        }
    }
    Ok(out)
}

// ----- Edge selection -----

/// Which edges a blend applies to, in terms that survive a rebuild.
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeSelect {
    /// Every edge of the body.
    All,
    /// Edges between two named faces.
    Between(FaceRef, FaceRef),
    /// All edges bounding a named face.
    OnFace(FaceRef),
    /// Edges whose midpoint is within a distance of a point.
    Near { point: Point3, tolerance: Expr },
    /// Edges whose midpoint lies in a box.
    InBox { min: Point3, max: Point3 },
    /// Straight edges parallel to a direction.
    Parallel(Point3),
    /// Circular edges of a radius.
    Radius(Expr),
    /// Edges matching any of these.
    Any(Vec<EdgeSelect>),
    /// Edges matching all of these.
    Both(Vec<EdgeSelect>),
}

impl EdgeSelect {
    fn matches(&self, body: &Body, e: usize, uses: &[Vec<(usize, usize, usize, bool)>], env: &Env) -> Result<bool> {
        let touches = |r: &FaceRef| uses[e].iter().any(|u| body.faces[u.0].tag == r.tag());
        let at = |p: &Point3| -> Result<Vec3> { Ok(v3(num(&p[0], env)?, num(&p[1], env)?, num(&p[2], env)?)) };
        Ok(match self {
            EdgeSelect::All => true,
            EdgeSelect::Between(a, b) => {
                if a == b {
                    uses[e].len() == 2 && uses[e].iter().all(|u| body.faces[u.0].tag == a.tag())
                } else {
                    touches(a) && touches(b)
                }
            }
            EdgeSelect::OnFace(f) => touches(f),
            EdgeSelect::Near { point, tolerance } => body.edge_midpoint(e).dist(at(point)?) <= num(tolerance, env)?,
            EdgeSelect::InBox { min, max } => {
                let (lo, hi, m) = (at(min)?, at(max)?, body.edge_midpoint(e));
                m.x >= lo.x && m.x <= hi.x && m.y >= lo.y && m.y <= hi.y && m.z >= lo.z && m.z <= hi.z
            }
            EdgeSelect::Parallel(d) => {
                let d = at(d)?;
                if d.norm() < 1e-12 {
                    return Err(Error::InvalidInput("an edge direction has zero length".into()));
                }
                body.edge_is_parallel(e, d.normalized())
            }
            EdgeSelect::Radius(r) => body.edge_is_arc_of_radius(e, num(r, env)?),
            EdgeSelect::Any(list) => {
                for q in list {
                    if q.matches(body, e, uses, env)? {
                        return Ok(true);
                    }
                }
                false
            }
            EdgeSelect::Both(list) => {
                for q in list {
                    if !q.matches(body, e, uses, env)? {
                        return Ok(false);
                    }
                }
                true
            }
        })
    }

    /// Features this selection refers to by name.
    fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            EdgeSelect::Between(a, b) => {
                out.push(a.feature);
                out.push(b.feature);
            }
            EdgeSelect::OnFace(f) => out.push(f.feature),
            EdgeSelect::Any(list) | EdgeSelect::Both(list) => {
                for q in list {
                    q.references(out);
                }
            }
            _ => {}
        }
    }

    fn collect_exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        match self {
            EdgeSelect::Near { point, tolerance } => {
                out.extend(point.iter());
                out.push(tolerance);
            }
            EdgeSelect::InBox { min, max } => {
                out.extend(min.iter());
                out.extend(max.iter());
            }
            EdgeSelect::Parallel(d) => out.extend(d.iter()),
            EdgeSelect::Radius(r) => out.push(r),
            EdgeSelect::Any(list) | EdgeSelect::Both(list) => {
                for q in list {
                    q.collect_exprs(out);
                }
            }
            _ => {}
        }
    }
}

// ----- Features -----

/// How a new solid combines with the body so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Add,
    Remove,
    Intersect,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Primitive {
    Box { dx: Expr, dy: Expr, dz: Expr },
    Cylinder { radius: Expr, height: Expr },
    Cone { r0: Expr, r1: Expr, height: Expr },
    Sphere { radius: Expr },
    Torus { major: Expr, minor: Expr },
}

/// What a feature does.
#[derive(Clone, Debug, PartialEq)]
pub enum Op {
    Sketch { plane: Placement, regions: Vec<Region> },
    /// Extrudes a sketch along its normal, from `start` for `length`.
    Extrude { sketch: FeatureId, start: Expr, length: Expr, mode: Mode },
    /// Revolves a sketch about its Z axis. Profile x is the distance from the axis and
    /// profile y the height along it.
    Revolve { sketch: FeatureId, degrees: Expr, mode: Mode },
    Solid { primitive: Primitive, at: Placement, mode: Mode },
    Fillet { edges: EdgeSelect, radius: Expr, propagate: bool },
    Chamfer { edges: EdgeSelect, distance: Expr },
}

impl Op {
    pub fn sketch(plane: Placement, regions: Vec<Region>) -> Op {
        Op::Sketch { plane, regions }
    }

    pub fn extrude(sketch: FeatureId, start: impl Into<Expr>, length: impl Into<Expr>, mode: Mode) -> Op {
        Op::Extrude { sketch, start: start.into(), length: length.into(), mode }
    }

    pub fn revolve(sketch: FeatureId, degrees: impl Into<Expr>, mode: Mode) -> Op {
        Op::Revolve { sketch, degrees: degrees.into(), mode }
    }

    pub fn solid(primitive: Primitive, at: Placement, mode: Mode) -> Op {
        Op::Solid { primitive, at, mode }
    }

    /// A fillet that follows tangent neighbours, the default.
    pub fn fillet(edges: EdgeSelect, radius: impl Into<Expr>) -> Op {
        Op::Fillet { edges, radius: radius.into(), propagate: true }
    }

    pub fn chamfer(edges: EdgeSelect, distance: impl Into<Expr>) -> Op {
        Op::Chamfer { edges, distance: distance.into() }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Op::Sketch { .. } => "sketch",
            Op::Extrude { .. } => "extrude",
            Op::Revolve { .. } => "revolve",
            Op::Solid { .. } => "solid",
            Op::Fillet { .. } => "fillet",
            Op::Chamfer { .. } => "chamfer",
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Op::Sketch { .. } => "Sketch",
            Op::Extrude { .. } => "Extrude",
            Op::Revolve { .. } => "Revolve",
            Op::Solid { .. } => "Solid",
            Op::Fillet { .. } => "Fillet",
            Op::Chamfer { .. } => "Chamfer",
        }
    }

    /// Features this one needs to come after.
    pub fn references(&self) -> Vec<FeatureId> {
        let mut out = Vec::new();
        match self {
            Op::Extrude { sketch, .. } | Op::Revolve { sketch, .. } => out.push(*sketch),
            Op::Fillet { edges, .. } | Op::Chamfer { edges, .. } => edges.references(&mut out),
            _ => {}
        }
        out.sort();
        out.dedup();
        out
    }

    fn exprs(&self) -> Vec<&Expr> {
        let mut out: Vec<&Expr> = Vec::new();
        fn place<'a>(p: &'a Placement, out: &mut Vec<&'a Expr>) {
            out.extend(p.origin.iter());
            out.extend(p.z.iter());
            out.extend(p.x.iter());
        }
        fn shape<'a>(s: &'a Shape, out: &mut Vec<&'a Expr>) {
            match s {
                Shape::Rect { x, y, w, h } => out.extend([x, y, w, h]),
                Shape::RoundedRect { cx, cy, w, h, r } => out.extend([cx, cy, w, h, r]),
                Shape::Circle { cx, cy, r } => out.extend([cx, cy, r]),
                Shape::Polygon { points } => out.extend(points.iter().flatten()),
                Shape::Path { start, moves } => {
                    out.extend(start.iter());
                    for m in moves {
                        match m {
                            Move::Line { to } => out.extend(to.iter()),
                            Move::Arc { to, center, .. } => {
                                out.extend(to.iter());
                                out.extend(center.iter());
                            }
                        }
                    }
                }
            }
        }
        match self {
            Op::Sketch { plane, regions } => {
                place(plane, &mut out);
                for r in regions {
                    shape(&r.outer, &mut out);
                    for h in &r.holes {
                        shape(h, &mut out);
                    }
                    match &r.repeat {
                        Some(Repeat::Circular { count, radius, start_degrees }) => out.extend([count, radius, start_degrees]),
                        Some(Repeat::Grid { nx, ny, dx, dy }) => out.extend([nx, ny, dx, dy]),
                        None => {}
                    }
                }
            }
            Op::Extrude { start, length, .. } => out.extend([start, length]),
            Op::Revolve { degrees, .. } => out.push(degrees),
            Op::Solid { primitive, at, .. } => {
                place(at, &mut out);
                match primitive {
                    Primitive::Box { dx, dy, dz } => out.extend([dx, dy, dz]),
                    Primitive::Cylinder { radius, height } => out.extend([radius, height]),
                    Primitive::Cone { r0, r1, height } => out.extend([r0, r1, height]),
                    Primitive::Sphere { radius } => out.push(radius),
                    Primitive::Torus { major, minor } => out.extend([major, minor]),
                }
            }
            Op::Fillet { edges, radius, .. } => {
                edges.collect_exprs(&mut out);
                out.push(radius);
            }
            Op::Chamfer { edges, distance } => {
                edges.collect_exprs(&mut out);
                out.push(distance);
            }
        }
        out
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub suppressed: bool,
    pub op: Op,
}

// ----- Rebuild results -----

/// What became of one feature in a rebuild.
#[derive(Clone, Debug, PartialEq)]
pub enum StepState {
    /// Computed in this rebuild.
    Rebuilt,
    /// Inputs unchanged, so the previous result was reused.
    Reused,
    /// Turned off by the user.
    Suppressed,
    /// Past the rollback position.
    RolledBack,
    /// This feature failed.
    Failed(String),
    /// An earlier feature failed, so this one did not run.
    Blocked,
}

impl StepState {
    pub fn word(&self) -> &'static str {
        match self {
            StepState::Rebuilt => "rebuilt",
            StepState::Reused => "reused",
            StepState::Suppressed => "suppressed",
            StepState::RolledBack => "rolledback",
            StepState::Failed(_) => "failed",
            StepState::Blocked => "blocked",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Step {
    pub id: FeatureId,
    pub name: String,
    pub kind: &'static str,
    pub state: StepState,
}

/// The outcome of a rebuild.
#[derive(Clone, Debug)]
pub struct Outcome {
    pub steps: Vec<Step>,
    /// The body at the rollback position, or at the last feature that succeeded.
    pub body: Option<Arc<Body>>,
    /// Index in `steps` of the failed feature, if any.
    pub failed: Option<usize>,
}

impl Outcome {
    pub fn rebuilt_count(&self) -> usize {
        self.steps.iter().filter(|s| s.state == StepState::Rebuilt).count()
    }
}

#[derive(Clone)]
struct Cached {
    key: u64,
    /// The body after this feature, for features that make one.
    body: Option<Arc<Body>>,
    /// A sketch's output, kept separately because a sketch doesn't touch the body.
    sketch: Option<(Frame, Arc<Vec<Profile>>)>,
}

// ----- The document -----

/// Parameters and features. Everything saved is in here, and results are cached beside it.
#[derive(Clone, Default)]
pub struct Document {
    params: Vec<Parameter>,
    features: Vec<Feature>,
    next_id: u32,
    cache: HashMap<FeatureId, Cached>,
}

fn num(e: &Expr, env: &Env) -> Result<f64> {
    e.eval(env).map_err(Error::InvalidInput)
}

fn hash_text(seed: u64, text: &str) -> u64 {
    let mut h = seed ^ 0xcbf29ce484222325;
    for b in text.as_bytes() {
        h = (h ^ *b as u64).wrapping_mul(0x100000001b3);
    }
    h ^ (h >> 29)
}

fn valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    chars.next().is_some_and(|c| c.is_alphabetic() || c == '_') && chars.all(|c| c.is_alphanumeric() || c == '_')
}

impl Document {
    pub fn new() -> Document {
        Document { next_id: 1, ..Document::default() }
    }

    pub fn parameters(&self) -> &[Parameter] {
        &self.params
    }

    pub fn features(&self) -> &[Feature] {
        &self.features
    }

    pub fn feature(&self, id: FeatureId) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }

    fn position(&self, id: FeatureId) -> Result<usize> {
        self.features
            .iter()
            .position(|f| f.id == id)
            .ok_or_else(|| Error::InvalidInput(format!("there is no feature f{}", id.0)))
    }

    // ----- Parameters -----

    /// Sets a parameter, creating it if it doesn't exist. The value is a number or an
    /// expression over other parameters. Refused if it would make a cycle.
    pub fn set_parameter(&mut self, name: &str, value: impl Into<Expr>) -> Result<()> {
        if !valid_name(name) {
            return Err(Error::InvalidInput(format!("\"{name}\" can't be a parameter name: use letters, digits and _")));
        }
        let value = value.into();
        let previous = self.params.clone();
        match self.params.iter_mut().find(|p| p.name == name) {
            Some(p) => p.value = value,
            None => self.params.push(Parameter { name: name.to_string(), value }),
        }
        if let Err(e) = self.environment() {
            self.params = previous;
            return Err(e);
        }
        Ok(())
    }

    /// Removes a parameter. Refused while another parameter uses it.
    pub fn remove_parameter(&mut self, name: &str) -> Result<()> {
        let at = self
            .params
            .iter()
            .position(|p| p.name == name)
            .ok_or_else(|| Error::InvalidInput(format!("there is no parameter named \"{name}\"")))?;
        if let Some(user) = self.params.iter().find(|p| p.name != name && p.value.names().iter().any(|n| n == name)) {
            return Err(Error::InvalidInput(format!("parameter \"{}\" uses \"{name}\"", user.name)));
        }
        if let Some(f) = self.features.iter().find(|f| f.op.exprs().iter().any(|e| e.names().iter().any(|n| n == name))) {
            return Err(Error::InvalidInput(format!("feature f{} ({}) uses \"{name}\"", f.id.0, f.name)));
        }
        self.params.remove(at);
        Ok(())
    }

    /// Every parameter's value, resolving references in dependency order.
    pub fn environment(&self) -> Result<Env> {
        let mut env = Env::new();
        let mut remaining: Vec<&Parameter> = self.params.iter().collect();
        while !remaining.is_empty() {
            let before = remaining.len();
            let mut next = Vec::new();
            for p in remaining {
                let ready = p
                    .value
                    .names()
                    .iter()
                    .all(|n| env.contains_key(n) || (n == "pi" && !self.params.iter().any(|q| q.name == "pi")));
                if ready {
                    env.insert(p.name.clone(), num(&p.value, &env)?);
                } else {
                    next.push(p);
                }
            }
            if next.len() == before {
                let p = next[0];
                let unknown = p.value.names().into_iter().find(|n| n != "pi" && !self.params.iter().any(|q| &q.name == n));
                return Err(Error::InvalidInput(match unknown {
                    Some(n) => format!("parameter \"{}\" refers to \"{n}\", which doesn't exist", p.name),
                    None => format!("parameter \"{}\" is part of a cycle", p.name),
                }));
            }
            remaining = next;
        }
        Ok(env)
    }

    // ----- Features -----

    /// Adds a feature at the end of the history and returns its id. The feature must refer
    /// only to features that already exist.
    pub fn add(&mut self, op: Op) -> Result<FeatureId> {
        self.insert(op, None, self.features.len())
    }

    /// Adds a feature with a chosen name, at a position in the history.
    pub fn insert(&mut self, op: Op, name: Option<String>, at: usize) -> Result<FeatureId> {
        let at = at.min(self.features.len());
        for r in op.references() {
            match self.features[..at].iter().find(|f| f.id == r) {
                Some(_) => {}
                None => return Err(Error::InvalidInput(format!("this feature refers to f{}, which doesn't come before it", r.0))),
            }
        }
        Self::check_sketch_refs(&op, &self.features[..at])?;
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        let name = name.unwrap_or_else(|| {
            let label = op.label();
            let n = self.features.iter().filter(|f| f.op.label() == label).count();
            format!("{label} {}", n + 1)
        });
        self.features.insert(at, Feature { id, name, suppressed: false, op });
        Ok(id)
    }

    /// Adds a feature, panicking on a bad reference. For models written in code, where a
    /// bad reference is a programming error.
    pub fn with(&mut self, op: Op) -> FeatureId {
        self.add(op).expect("feature references must already exist")
    }

    fn check_sketch_refs(op: &Op, before: &[Feature]) -> Result<()> {
        if let Op::Extrude { sketch, .. } | Op::Revolve { sketch, .. } = op {
            if let Some(f) = before.iter().find(|f| f.id == *sketch) {
                if !matches!(f.op, Op::Sketch { .. }) {
                    return Err(Error::InvalidInput(format!("f{} is a {}, not a sketch", sketch.0, f.op.kind())));
                }
            }
        }
        Ok(())
    }

    /// Replaces what a feature does, keeping its id, name and position.
    pub fn edit(&mut self, id: FeatureId, op: Op) -> Result<()> {
        let at = self.position(id)?;
        for r in op.references() {
            if !self.features[..at].iter().any(|f| f.id == r) {
                return Err(Error::InvalidInput(format!("this feature refers to f{}, which doesn't come before it", r.0)));
            }
        }
        Self::check_sketch_refs(&op, &self.features[..at])?;
        self.features[at].op = op;
        Ok(())
    }

    pub fn rename(&mut self, id: FeatureId, name: &str) -> Result<()> {
        let at = self.position(id)?;
        self.features[at].name = name.to_string();
        Ok(())
    }

    /// Turns a feature off or on. A feature that needs a suppressed sketch fails with a
    /// clear message when rebuilt; other features simply carry on from the previous body.
    pub fn suppress(&mut self, id: FeatureId, suppressed: bool) -> Result<()> {
        let at = self.position(id)?;
        self.features[at].suppressed = suppressed;
        Ok(())
    }

    /// Features that refer to this one.
    pub fn dependents(&self, id: FeatureId) -> Vec<FeatureId> {
        self.features.iter().filter(|f| f.op.references().contains(&id)).map(|f| f.id).collect()
    }

    /// Deletes a feature. Refused while others refer to it.
    pub fn delete(&mut self, id: FeatureId) -> Result<()> {
        let at = self.position(id)?;
        let users = self.dependents(id);
        if !users.is_empty() {
            let names: Vec<String> = users.iter().map(|u| format!("f{}", u.0)).collect();
            return Err(Error::InvalidInput(format!(
                "f{} can't be deleted while {} refer to it",
                id.0,
                names.join(", ")
            )));
        }
        self.features.remove(at);
        self.cache.remove(&id);
        Ok(())
    }

    /// Moves a feature to a position in the history. Refused if that would put it before
    /// something it refers to, or after something that refers to it.
    pub fn reorder(&mut self, id: FeatureId, to: usize) -> Result<()> {
        let from = self.position(id)?;
        let to = to.min(self.features.len() - 1);
        let mut moved = self.features.clone();
        let f = moved.remove(from);
        moved.insert(to, f);
        for (i, f) in moved.iter().enumerate() {
            for r in f.op.references() {
                if !moved[..i].iter().any(|g| g.id == r) {
                    return Err(Error::InvalidInput(format!(
                        "moving f{} there would put f{} before f{}, which it refers to",
                        id.0, f.id.0, r.0
                    )));
                }
            }
        }
        self.features = moved;
        Ok(())
    }

    // ----- Rebuilding -----

    /// Rebuilds through the feature at index `upto` (all of them when `None`), reusing every
    /// step whose inputs did not change. A failed feature keeps the last good body available
    /// and blocks the features after it.
    pub fn rebuild(&mut self, upto: Option<usize>) -> Outcome {
        let n = self.features.len();
        let last = upto.map_or(n, |u| (u + 1).min(n));
        let mut steps: Vec<Step> = Vec::with_capacity(n);
        let mut body: Option<Arc<Body>> = None;
        let mut failed: Option<usize> = None;
        let mut body_key = 0u64;

        let env = match self.environment() {
            Ok(env) => env,
            Err(e) => {
                let why = e.to_string();
                for f in &self.features {
                    steps.push(Step { id: f.id, name: f.name.clone(), kind: f.op.kind(), state: StepState::Blocked });
                }
                if let Some(first) = steps.first_mut() {
                    first.state = StepState::Failed(why);
                    failed = Some(0);
                }
                return Outcome { steps, body: None, failed };
            }
        };

        let mut sketch_keys: HashMap<FeatureId, u64> = HashMap::new();
        for i in 0..n {
            let feature = self.features[i].clone();
            let kind = feature.op.kind();
            let step = |state: StepState| Step { id: feature.id, name: feature.name.clone(), kind, state };
            if i >= last {
                steps.push(step(StepState::RolledBack));
                continue;
            }
            if feature.suppressed {
                steps.push(step(StepState::Suppressed));
                continue;
            }
            if failed.is_some() {
                steps.push(step(StepState::Blocked));
                continue;
            }
            let own = match Self::signature(&feature.op, &env) {
                Ok(s) => s,
                Err(e) => {
                    failed = Some(i);
                    steps.push(step(StepState::Failed(e.to_string())));
                    continue;
                }
            };
            // A sketch depends only on its own inputs. Everything else also depends on the
            // body so far, and on its sketch when it has one.
            let key = match &feature.op {
                Op::Sketch { .. } => hash_text(1, &own),
                Op::Extrude { sketch, .. } | Op::Revolve { sketch, .. } => {
                    let sketch_key = sketch_keys.get(sketch).copied().unwrap_or(0);
                    hash_text(body_key ^ sketch_key.rotate_left(17), &own)
                }
                _ => hash_text(body_key, &own),
            };
            let cached = self.cache.get(&feature.id).filter(|c| c.key == key).cloned();
            if let Some(c) = cached {
                if matches!(feature.op, Op::Sketch { .. }) {
                    sketch_keys.insert(feature.id, key);
                } else {
                    body = c.body.clone();
                    body_key = key;
                }
                steps.push(step(StepState::Reused));
                continue;
            }
            match self.run(&feature, &env, body.clone()) {
                Ok((new_body, sketch)) => {
                    if sketch.is_some() {
                        sketch_keys.insert(feature.id, key);
                    } else {
                        body = new_body.clone();
                        body_key = key;
                    }
                    self.cache.insert(feature.id, Cached { key, body: new_body, sketch });
                    steps.push(step(StepState::Rebuilt));
                }
                // The previous result stays cached under its own key, so going back to the
                // earlier inputs reuses it.
                Err(e) => {
                    failed = Some(i);
                    steps.push(step(StepState::Failed(e.to_string())));
                }
            }
        }
        Outcome { steps, body, failed }
    }

    /// Text covering everything a feature's result depends on: what it is, and the value
    /// of every expression in it, with exact bits.
    fn signature(op: &Op, env: &Env) -> Result<String> {
        let mut text = format!("{op:?}");
        for e in op.exprs() {
            text.push(' ');
            text.push_str(&format!("{:x}", num(e, env)?.to_bits()));
        }
        Ok(text)
    }

    #[allow(clippy::type_complexity)]
    fn run(&self, feature: &Feature, env: &Env, body: Option<Arc<Body>>) -> Result<(Option<Arc<Body>>, Option<(Frame, Arc<Vec<Profile>>)>)> {
        let tag = feature.id.0;
        let sketch_output = |id: &FeatureId| -> Result<(Frame, Arc<Vec<Profile>>)> {
            if self.feature(*id).is_some_and(|f| f.suppressed) {
                return Err(Error::InvalidInput(format!("its sketch f{} is suppressed", id.0)));
            }
            self.cache
                .get(id)
                .and_then(|c| c.sketch.clone())
                .ok_or_else(|| Error::InvalidInput(format!("its sketch f{} did not build", id.0)))
        };
        match &feature.op {
            Op::Sketch { plane, regions } => {
                let list = resolve_regions(regions, env)?;
                if list.is_empty() {
                    return Err(Error::InvalidProfile("the sketch has no closed regions".into()));
                }
                for p in &list {
                    p.validate()?;
                }
                Ok((None, Some((plane.resolve(env)?, Arc::new(list)))))
            }
            Op::Extrude { sketch, start, length, mode } => {
                let (frame, list) = sketch_output(sketch)?;
                let shifted = Frame { origin: frame.origin + frame.z * num(start, env)?, ..frame };
                let h = num(length, env)?;
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| Ok(with_region(extrude(p, &shifted, h, tag)?, k as u32)))
                    .collect::<Result<Vec<Body>>>()?;
                Ok((Some(Arc::new(combine(body.as_deref(), tools, *mode)?)), None))
            }
            Op::Revolve { sketch, degrees, mode } => {
                let (frame, list) = sketch_output(sketch)?;
                let a = num(degrees, env)?.to_radians();
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| Ok(with_region(revolve(p, &frame, a, tag)?, k as u32)))
                    .collect::<Result<Vec<Body>>>()?;
                Ok((Some(Arc::new(combine(body.as_deref(), tools, *mode)?)), None))
            }
            Op::Solid { primitive, at, mode } => {
                let frame = at.resolve(env)?;
                let solid = match primitive {
                    Primitive::Box { dx, dy, dz } => box_solid(&frame, num(dx, env)?, num(dy, env)?, num(dz, env)?, tag)?,
                    Primitive::Cylinder { radius, height } => cylinder(&frame, num(radius, env)?, num(height, env)?, tag)?,
                    Primitive::Cone { r0, r1, height } => cone_frustum(&frame, num(r0, env)?, num(r1, env)?, num(height, env)?, tag)?,
                    Primitive::Sphere { radius } => sphere(&frame, num(radius, env)?, tag)?,
                    Primitive::Torus { major, minor } => torus(&frame, num(major, env)?, num(minor, env)?, tag)?,
                };
                Ok((Some(Arc::new(combine(body.as_deref(), vec![solid], *mode)?)), None))
            }
            Op::Fillet { edges, radius, propagate } => {
                let current = body.ok_or_else(|| Error::InvalidInput("a fillet needs a body to work on".into()))?;
                let list = select_edges(edges, &current, env)?;
                let options = BlendOptions { propagate: *propagate };
                Ok((Some(Arc::new(fillet_with(&current, &list, num(radius, env)?, options, tag)?)), None))
            }
            Op::Chamfer { edges, distance } => {
                let current = body.ok_or_else(|| Error::InvalidInput("a chamfer needs a body to work on".into()))?;
                let list = select_edges(edges, &current, env)?;
                Ok((Some(Arc::new(chamfer(&current, &list, num(distance, env)?, tag)?)), None))
            }
        }
    }

    // ----- Saving and loading -----

    /// The document as a JSON value.
    pub fn to_json(&self) -> Value {
        Value::obj(vec![
            ("format", Value::str(FORMAT)),
            ("version", Value::Num(FORMAT_VERSION)),
            ("units", Value::str("mm")),
            ("next_id", Value::Num(self.next_id as f64)),
            (
                "parameters",
                Value::Arr(
                    self.params
                        .iter()
                        .map(|p| Value::obj(vec![("name", Value::str(&p.name)), ("value", expr_value(&p.value))]))
                        .collect(),
                ),
            ),
            ("features", Value::Arr(self.features.iter().map(feature_value).collect())),
        ])
    }

    /// The document as the text of a file.
    pub fn save_text(&self) -> String {
        self.to_json().to_pretty()
    }

    /// Reads a document from file text. Rejects anything it doesn't fully understand: an
    /// edit that silently dropped part of someone's model would be worse than an error.
    pub fn load_text(text: &str) -> Result<Document> {
        let v = json::parse(text).map_err(|e| Error::InvalidInput(format!("not valid JSON: {e}")))?;
        Document::from_json(&v)
    }

    pub fn from_json(v: &Value) -> Result<Document> {
        let bad = |what: &str| Error::InvalidInput(what.to_string());
        if v.get("format").and_then(Value::as_str) != Some(FORMAT) {
            return Err(bad("this is not a CoreShape document"));
        }
        let version = v.get("version").and_then(Value::as_f64).ok_or_else(|| bad("the document has no version"))?;
        if version != FORMAT_VERSION {
            return Err(Error::InvalidInput(format!(
                "this document is format version {version}, and this kernel reads version {FORMAT_VERSION}"
            )));
        }
        known_keys(v, &["format", "version", "units", "next_id", "parameters", "features"])?;
        if let Some(units) = v.get("units").and_then(Value::as_str) {
            if units != "mm" {
                return Err(Error::InvalidInput(format!("units \"{units}\" are not supported")));
            }
        }
        let mut doc = Document::new();
        for p in v.get("parameters").and_then(Value::as_array).unwrap_or(&[]) {
            known_keys(p, &["name", "value"])?;
            let name = p.get("name").and_then(Value::as_str).ok_or_else(|| bad("a parameter has no name"))?;
            if !valid_name(name) {
                return Err(Error::InvalidInput(format!("\"{name}\" can't be a parameter name")));
            }
            if doc.params.iter().any(|q| q.name == name) {
                return Err(Error::InvalidInput(format!("parameter \"{name}\" appears twice")));
            }
            let value = read_expr(p.get("value").ok_or_else(|| bad("a parameter has no value"))?)?;
            doc.params.push(Parameter { name: name.to_string(), value });
        }
        doc.environment()?;
        let mut max_id = 0;
        for f in v.get("features").and_then(Value::as_array).unwrap_or(&[]) {
            let feature = read_feature(f)?;
            if feature.id.0 == 0 {
                return Err(bad("feature ids start at 1"));
            }
            if doc.features.iter().any(|g| g.id == feature.id) {
                return Err(Error::InvalidInput(format!("feature id {} appears twice", feature.id.0)));
            }
            for r in feature.op.references() {
                if !doc.features.iter().any(|g| g.id == r) {
                    return Err(Error::InvalidInput(format!(
                        "feature f{} refers to f{}, which doesn't come before it",
                        feature.id.0, r.0
                    )));
                }
            }
            Self::check_sketch_refs(&feature.op, &doc.features)?;
            max_id = max_id.max(feature.id.0);
            doc.features.push(feature);
        }
        let next = v.get("next_id").and_then(Value::as_f64).unwrap_or(0.0);
        // Ids are never reused, so this must stay above every id ever handed out.
        doc.next_id = (max_id + 1).max(if next.is_finite() && next >= 0.0 && next < u32::MAX as f64 { next as u32 } else { 0 });
        Ok(doc)
    }
}

fn with_region(mut body: Body, region: u32) -> Body {
    if region > 0 {
        for f in &mut body.faces {
            if f.tag.slot >= SLOT_SEG_BASE {
                f.tag.slot += region * REGION_SLOTS;
            }
        }
    }
    body
}

fn combine(body: Option<&Body>, tools: Vec<Body>, mode: Mode) -> Result<Body> {
    let mut tools = tools.into_iter();
    let mut current = match body {
        Some(b) => b.clone(),
        None => {
            if mode != Mode::Add {
                return Err(Error::InvalidInput("there is no body yet to work on".into()));
            }
            tools.next().ok_or_else(|| Error::InvalidInput("the sketch has no closed regions".into()))?
        }
    };
    match mode {
        // Several regions apply one by one. Only the last result needs the full checks.
        Mode::Add | Mode::Remove => {
            let op = if mode == Mode::Add { BoolOp::Union } else { BoolOp::Subtract };
            let total = tools.len();
            for (i, t) in tools.enumerate() {
                let checks = if i + 1 == total { Checks::Full } else { Checks::Interim };
                current = boolean_with(&current, &t, op, checks)?;
            }
        }
        Mode::Intersect => {
            // One tool at a time would keep only what is common to all of them. A sketch
            // with several regions means their union.
            let mut tool: Option<Body> = None;
            for t in tools {
                tool = Some(match tool {
                    None => t,
                    Some(existing) => boolean_with(&existing, &t, BoolOp::Union, Checks::Interim)?,
                });
            }
            if let Some(t) = tool {
                current = boolean_with(&current, &t, BoolOp::Intersect, Checks::Full)?;
            }
        }
    }
    Ok(current)
}

fn select_edges(query: &EdgeSelect, body: &Body, env: &Env) -> Result<Vec<usize>> {
    let uses = body.edge_uses();
    let mut list = Vec::new();
    for e in 0..body.edges.len() {
        if query.matches(body, e, &uses, env)? {
            list.push(e);
        }
    }
    if list.is_empty() {
        return Err(Error::InvalidInput("the edges this feature refers to no longer exist in the model".into()));
    }
    Ok(list)
}

/// The stable name of an edge: the faces on either side of it, sorted so that it doesn't
/// matter which side comes first. Two edges can share a name when faces meet more than once.
pub fn edge_name(body: &Body, e: usize, uses: &[Vec<(usize, usize, usize, bool)>]) -> String {
    let mut names: Vec<String> =
        uses[e].iter().map(|u| FaceRef::from_tag(body.faces[u.0].tag).name()).collect();
    names.sort();
    names.join("|")
}

// ----- Reading and writing JSON -----

fn known_keys(v: &Value, allowed: &[&str]) -> Result<()> {
    if let Value::Obj(items) = v {
        for (k, _) in items {
            if !allowed.contains(&k.as_str()) {
                return Err(Error::InvalidInput(format!("unknown field \"{k}\"; expected one of: {}", allowed.join(", "))));
            }
        }
        Ok(())
    } else {
        Err(Error::InvalidInput(format!("expected an object with fields: {}", allowed.join(", "))))
    }
}

fn expr_value(e: &Expr) -> Value {
    match e.as_number() {
        Some(n) => Value::Num(n),
        None => Value::str(e.text()),
    }
}

fn read_expr(v: &Value) -> Result<Expr> {
    match v {
        Value::Num(n) => Ok(Expr::number(*n)),
        Value::Str(s) => Expr::parse(s).map_err(Error::InvalidInput),
        _ => Err(Error::InvalidInput("a dimension must be a number or an expression in quotes".into())),
    }
}

fn field<'a>(v: &'a Value, key: &str, what: &str) -> Result<&'a Value> {
    v.get(key).ok_or_else(|| Error::InvalidInput(format!("{what} needs a \"{key}\"")))
}

fn expr_field(v: &Value, key: &str, what: &str) -> Result<Expr> {
    read_expr(field(v, key, what)?)
}

fn point2_value(p: &Point2) -> Value {
    Value::Arr(p.iter().map(expr_value).collect())
}

fn point3_value(p: &Point3) -> Value {
    Value::Arr(p.iter().map(expr_value).collect())
}

fn read_point2(v: &Value) -> Result<Point2> {
    match v.as_array() {
        Some([a, b]) => Ok([read_expr(a)?, read_expr(b)?]),
        _ => Err(Error::InvalidInput("a 2D point is written [x, y]".into())),
    }
}

fn read_point3(v: &Value) -> Result<Point3> {
    match v.as_array() {
        Some([a, b, c]) => Ok([read_expr(a)?, read_expr(b)?, read_expr(c)?]),
        _ => Err(Error::InvalidInput("a 3D point is written [x, y, z]".into())),
    }
}

fn placement_value(p: &Placement) -> Value {
    Value::obj(vec![("origin", point3_value(&p.origin)), ("z", point3_value(&p.z)), ("x", point3_value(&p.x))])
}

fn read_placement(v: &Value) -> Result<Placement> {
    known_keys(v, &["origin", "z", "x"])?;
    let dir = |key: &str, fallback: [f64; 3]| -> Result<Point3> {
        match v.get(key) {
            Some(p) => read_point3(p),
            None => Ok(point3(fallback[0], fallback[1], fallback[2])),
        }
    };
    let z = dir("z", [0.0, 0.0, 1.0])?;
    // Without a stated x, world X, unless z runs along it.
    let x_default = match z.iter().map(Expr::as_number).collect::<Vec<_>>().as_slice() {
        [Some(_), Some(b), Some(c)] if b.abs() < 1e-9 && c.abs() < 1e-9 => [0.0, 1.0, 0.0],
        _ => [1.0, 0.0, 0.0],
    };
    Ok(Placement { origin: dir("origin", [0.0, 0.0, 0.0])?, z, x: dir("x", x_default)? })
}

fn shape_value(s: &Shape) -> Value {
    match s {
        Shape::Rect { x, y, w, h } => Value::obj(vec![
            ("type", Value::str("rect")),
            ("x", expr_value(x)),
            ("y", expr_value(y)),
            ("w", expr_value(w)),
            ("h", expr_value(h)),
        ]),
        Shape::RoundedRect { cx, cy, w, h, r } => Value::obj(vec![
            ("type", Value::str("rounded_rect")),
            ("cx", expr_value(cx)),
            ("cy", expr_value(cy)),
            ("w", expr_value(w)),
            ("h", expr_value(h)),
            ("r", expr_value(r)),
        ]),
        Shape::Circle { cx, cy, r } => {
            Value::obj(vec![("type", Value::str("circle")), ("cx", expr_value(cx)), ("cy", expr_value(cy)), ("r", expr_value(r))])
        }
        Shape::Polygon { points } => {
            Value::obj(vec![("type", Value::str("polygon")), ("points", Value::Arr(points.iter().map(point2_value).collect()))])
        }
        Shape::Path { start, moves } => Value::obj(vec![
            ("type", Value::str("path")),
            ("start", point2_value(start)),
            (
                "moves",
                Value::Arr(
                    moves
                        .iter()
                        .map(|m| match m {
                            Move::Line { to } => Value::obj(vec![("line_to", point2_value(to))]),
                            Move::Arc { to, center, ccw } => Value::obj(vec![
                                ("arc_to", point2_value(to)),
                                ("center", point2_value(center)),
                                ("ccw", Value::Bool(*ccw)),
                            ]),
                        })
                        .collect(),
                ),
            ),
        ]),
    }
}

fn read_shape(v: &Value) -> Result<Shape> {
    let kind = field(v, "type", "a shape")?.as_str().unwrap_or("");
    let what = format!("a {kind} shape");
    let at_origin = |key: &str| -> Result<Expr> {
        match v.get(key) {
            Some(e) => read_expr(e),
            None => Ok(Expr::number(0.0)),
        }
    };
    match kind {
        "rect" => {
            known_keys(v, &["type", "x", "y", "w", "h"])?;
            Ok(Shape::Rect { x: at_origin("x")?, y: at_origin("y")?, w: expr_field(v, "w", &what)?, h: expr_field(v, "h", &what)? })
        }
        "rounded_rect" => {
            known_keys(v, &["type", "cx", "cy", "w", "h", "r"])?;
            Ok(Shape::RoundedRect {
                cx: at_origin("cx")?,
                cy: at_origin("cy")?,
                w: expr_field(v, "w", &what)?,
                h: expr_field(v, "h", &what)?,
                r: expr_field(v, "r", &what)?,
            })
        }
        "circle" => {
            known_keys(v, &["type", "cx", "cy", "r"])?;
            Ok(Shape::Circle { cx: at_origin("cx")?, cy: at_origin("cy")?, r: expr_field(v, "r", &what)? })
        }
        "polygon" => {
            known_keys(v, &["type", "points"])?;
            let points = field(v, "points", &what)?
                .as_array()
                .ok_or_else(|| Error::InvalidInput("a polygon's points must be a list".into()))?
                .iter()
                .map(read_point2)
                .collect::<Result<Vec<Point2>>>()?;
            Ok(Shape::Polygon { points })
        }
        "path" => {
            known_keys(v, &["type", "start", "moves"])?;
            let mut moves = Vec::new();
            for m in field(v, "moves", &what)?.as_array().ok_or_else(|| Error::InvalidInput("a path's moves must be a list".into()))? {
                if let Some(to) = m.get("line_to") {
                    known_keys(m, &["line_to"])?;
                    moves.push(Move::Line { to: read_point2(to)? });
                } else if let Some(to) = m.get("arc_to") {
                    known_keys(m, &["arc_to", "center", "ccw"])?;
                    moves.push(Move::Arc {
                        to: read_point2(to)?,
                        center: read_point2(field(m, "center", "an arc")?)?,
                        ccw: field(m, "ccw", "an arc")?.as_bool().ok_or_else(|| Error::InvalidInput("\"ccw\" must be true or false".into()))?,
                    });
                } else {
                    return Err(Error::InvalidInput("a path move is {\"line_to\": ...} or {\"arc_to\": ..., \"center\": ..., \"ccw\": ...}".into()));
                }
            }
            Ok(Shape::Path { start: read_point2(field(v, "start", &what)?)?, moves })
        }
        other => Err(Error::InvalidInput(format!(
            "unknown shape \"{other}\"; expected rect, rounded_rect, circle, polygon or path"
        ))),
    }
}

fn region_value(r: &Region) -> Value {
    let mut items = vec![("outer", shape_value(&r.outer))];
    if !r.holes.is_empty() {
        items.push(("holes", Value::Arr(r.holes.iter().map(shape_value).collect())));
    }
    if let Some(rep) = &r.repeat {
        items.push((
            "repeat",
            match rep {
                Repeat::Circular { count, radius, start_degrees } => Value::obj(vec![
                    ("type", Value::str("circular")),
                    ("count", expr_value(count)),
                    ("radius", expr_value(radius)),
                    ("start_degrees", expr_value(start_degrees)),
                ]),
                Repeat::Grid { nx, ny, dx, dy } => Value::obj(vec![
                    ("type", Value::str("grid")),
                    ("nx", expr_value(nx)),
                    ("ny", expr_value(ny)),
                    ("dx", expr_value(dx)),
                    ("dy", expr_value(dy)),
                ]),
            },
        ));
    }
    Value::obj(items)
}

fn read_region(v: &Value) -> Result<Region> {
    known_keys(v, &["outer", "holes", "repeat"])?;
    let outer = read_shape(field(v, "outer", "a region")?)?;
    let mut holes = Vec::new();
    if let Some(list) = v.get("holes") {
        for h in list.as_array().ok_or_else(|| Error::InvalidInput("\"holes\" must be a list".into()))? {
            holes.push(read_shape(h)?);
        }
    }
    let repeat = match v.get("repeat") {
        None | Some(Value::Null) => None,
        Some(r) => match field(r, "type", "a repeat")?.as_str() {
            Some("circular") => {
                known_keys(r, &["type", "count", "radius", "start_degrees"])?;
                Some(Repeat::Circular {
                    count: expr_field(r, "count", "a circular repeat")?,
                    radius: expr_field(r, "radius", "a circular repeat")?,
                    start_degrees: r.get("start_degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(0.0)),
                })
            }
            Some("grid") => {
                known_keys(r, &["type", "nx", "ny", "dx", "dy"])?;
                Some(Repeat::Grid {
                    nx: expr_field(r, "nx", "a grid repeat")?,
                    ny: expr_field(r, "ny", "a grid repeat")?,
                    dx: expr_field(r, "dx", "a grid repeat")?,
                    dy: expr_field(r, "dy", "a grid repeat")?,
                })
            }
            _ => return Err(Error::InvalidInput("a repeat's type is circular or grid".into())),
        },
    };
    Ok(Region { outer, holes, repeat })
}

/// Edge selections as JSON. Exposed for tools that take one as an argument.
pub fn edge_select_value(e: &EdgeSelect) -> Value {
    match e {
        EdgeSelect::All => Value::str("all"),
        EdgeSelect::Between(a, b) => Value::obj(vec![("between", Value::Arr(vec![Value::str(a.name()), Value::str(b.name())]))]),
        EdgeSelect::OnFace(f) => Value::obj(vec![("on_face", Value::str(f.name()))]),
        EdgeSelect::Near { point, tolerance } => {
            Value::obj(vec![("near", point3_value(point)), ("tolerance", expr_value(tolerance))])
        }
        EdgeSelect::InBox { min, max } => Value::obj(vec![("in_box", Value::Arr(vec![point3_value(min), point3_value(max)]))]),
        EdgeSelect::Parallel(d) => Value::obj(vec![("parallel", point3_value(d))]),
        EdgeSelect::Radius(r) => Value::obj(vec![("radius", expr_value(r))]),
        EdgeSelect::Any(list) => Value::obj(vec![("any", Value::Arr(list.iter().map(edge_select_value).collect()))]),
        EdgeSelect::Both(list) => Value::obj(vec![("both", Value::Arr(list.iter().map(edge_select_value).collect()))]),
    }
}

/// Reads an edge selection from JSON.
pub fn read_edge_select(v: &Value) -> Result<EdgeSelect> {
    if v.as_str() == Some("all") {
        return Ok(EdgeSelect::All);
    }
    let names = "\"all\", {\"between\": [a, b]}, {\"on_face\": a}, {\"near\": [x, y, z], \"tolerance\": d}, {\"in_box\": [min, max]}, {\"parallel\": [x, y, z]}, {\"radius\": r}, {\"any\": [...]} or {\"both\": [...]}";
    let list_of = |items: &Value| -> Result<Vec<EdgeSelect>> {
        items
            .as_array()
            .ok_or_else(|| Error::InvalidInput("\"any\" and \"both\" take a list of selections".into()))?
            .iter()
            .map(read_edge_select)
            .collect()
    };
    if let Some(pair) = v.get("between") {
        known_keys(v, &["between"])?;
        return match pair.as_array() {
            Some([a, b]) => {
                let name = |x: &Value| {
                    x.as_str().ok_or_else(|| Error::InvalidInput("\"between\" takes two face names".into())).and_then(FaceRef::parse)
                };
                Ok(EdgeSelect::Between(name(a)?, name(b)?))
            }
            _ => Err(Error::InvalidInput("\"between\" takes two face names".into())),
        };
    }
    if let Some(f) = v.get("on_face") {
        known_keys(v, &["on_face"])?;
        let name = f.as_str().ok_or_else(|| Error::InvalidInput("\"on_face\" takes a face name".into()))?;
        return Ok(EdgeSelect::OnFace(FaceRef::parse(name)?));
    }
    if let Some(p) = v.get("near") {
        known_keys(v, &["near", "tolerance"])?;
        return Ok(EdgeSelect::Near {
            point: read_point3(p)?,
            tolerance: v.get("tolerance").map(read_expr).transpose()?.unwrap_or(Expr::number(1e-3)),
        });
    }
    if let Some(b) = v.get("in_box") {
        known_keys(v, &["in_box"])?;
        return match b.as_array() {
            Some([lo, hi]) => Ok(EdgeSelect::InBox { min: read_point3(lo)?, max: read_point3(hi)? }),
            _ => Err(Error::InvalidInput("\"in_box\" takes [min, max]".into())),
        };
    }
    if let Some(d) = v.get("parallel") {
        known_keys(v, &["parallel"])?;
        return Ok(EdgeSelect::Parallel(read_point3(d)?));
    }
    if let Some(r) = v.get("radius") {
        known_keys(v, &["radius"])?;
        return Ok(EdgeSelect::Radius(read_expr(r)?));
    }
    if let Some(items) = v.get("any") {
        known_keys(v, &["any"])?;
        return Ok(EdgeSelect::Any(list_of(items)?));
    }
    if let Some(items) = v.get("both") {
        known_keys(v, &["both"])?;
        return Ok(EdgeSelect::Both(list_of(items)?));
    }
    Err(Error::InvalidInput(format!("an edge selection is one of: {names}")))
}

fn mode_value(m: Mode) -> Value {
    Value::str(match m {
        Mode::Add => "add",
        Mode::Remove => "remove",
        Mode::Intersect => "intersect",
    })
}

fn read_mode(v: &Value) -> Result<Mode> {
    match v.get("mode").and_then(Value::as_str) {
        None | Some("add") => Ok(Mode::Add),
        Some("remove") => Ok(Mode::Remove),
        Some("intersect") => Ok(Mode::Intersect),
        Some(other) => Err(Error::InvalidInput(format!("unknown mode \"{other}\"; expected add, remove or intersect"))),
    }
}

/// A feature's operation as JSON, with the fields a caller supplies.
pub fn op_value(op: &Op) -> Value {
    match op {
        Op::Sketch { plane, regions } => Value::obj(vec![
            ("type", Value::str("sketch")),
            ("plane", placement_value(plane)),
            ("regions", Value::Arr(regions.iter().map(region_value).collect())),
        ]),
        Op::Extrude { sketch, start, length, mode } => Value::obj(vec![
            ("type", Value::str("extrude")),
            ("sketch", Value::Num(sketch.0 as f64)),
            ("start", expr_value(start)),
            ("length", expr_value(length)),
            ("mode", mode_value(*mode)),
        ]),
        Op::Revolve { sketch, degrees, mode } => Value::obj(vec![
            ("type", Value::str("revolve")),
            ("sketch", Value::Num(sketch.0 as f64)),
            ("degrees", expr_value(degrees)),
            ("mode", mode_value(*mode)),
        ]),
        Op::Solid { primitive, at, mode } => {
            let mut items = vec![("type", Value::str("solid"))];
            match primitive {
                Primitive::Box { dx, dy, dz } => {
                    items.push(("shape", Value::str("box")));
                    items.push(("dx", expr_value(dx)));
                    items.push(("dy", expr_value(dy)));
                    items.push(("dz", expr_value(dz)));
                }
                Primitive::Cylinder { radius, height } => {
                    items.push(("shape", Value::str("cylinder")));
                    items.push(("radius", expr_value(radius)));
                    items.push(("height", expr_value(height)));
                }
                Primitive::Cone { r0, r1, height } => {
                    items.push(("shape", Value::str("cone")));
                    items.push(("r0", expr_value(r0)));
                    items.push(("r1", expr_value(r1)));
                    items.push(("height", expr_value(height)));
                }
                Primitive::Sphere { radius } => {
                    items.push(("shape", Value::str("sphere")));
                    items.push(("radius", expr_value(radius)));
                }
                Primitive::Torus { major, minor } => {
                    items.push(("shape", Value::str("torus")));
                    items.push(("major", expr_value(major)));
                    items.push(("minor", expr_value(minor)));
                }
            }
            items.push(("at", placement_value(at)));
            items.push(("mode", mode_value(*mode)));
            Value::obj(items)
        }
        Op::Fillet { edges, radius, propagate } => Value::obj(vec![
            ("type", Value::str("fillet")),
            ("edges", edge_select_value(edges)),
            ("radius", expr_value(radius)),
            ("propagate", Value::Bool(*propagate)),
        ]),
        Op::Chamfer { edges, distance } => Value::obj(vec![
            ("type", Value::str("chamfer")),
            ("edges", edge_select_value(edges)),
            ("distance", expr_value(distance)),
        ]),
    }
}

fn read_sketch_id(v: &Value) -> Result<FeatureId> {
    let id = field(v, "sketch", "this feature")?
        .as_f64()
        .filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < u32::MAX as f64)
        .ok_or_else(|| Error::InvalidInput("\"sketch\" must be a feature id".into()))?;
    Ok(FeatureId(id as u32))
}

/// Reads a feature's operation from JSON. `extra` lists fields owned by the caller.
pub fn read_op(v: &Value, extra: &[&str]) -> Result<Op> {
    let with = |own: &[&str]| -> Result<()> {
        let mut all: Vec<&str> = own.to_vec();
        all.extend_from_slice(extra);
        known_keys(v, &all)
    };
    let kind = field(v, "type", "a feature")?.as_str().unwrap_or("");
    match kind {
        "sketch" => {
            with(&["type", "plane", "regions"])?;
            let plane = match v.get("plane") {
                Some(p) => read_placement(p)?,
                None => Placement::xy(0.0),
            };
            let regions = field(v, "regions", "a sketch")?
                .as_array()
                .ok_or_else(|| Error::InvalidInput("a sketch's regions must be a list".into()))?
                .iter()
                .map(read_region)
                .collect::<Result<Vec<Region>>>()?;
            Ok(Op::Sketch { plane, regions })
        }
        "extrude" => {
            with(&["type", "sketch", "start", "length", "mode"])?;
            Ok(Op::Extrude {
                sketch: read_sketch_id(v)?,
                start: v.get("start").map(read_expr).transpose()?.unwrap_or(Expr::number(0.0)),
                length: expr_field(v, "length", "an extrude")?,
                mode: read_mode(v)?,
            })
        }
        "revolve" => {
            with(&["type", "sketch", "degrees", "mode"])?;
            Ok(Op::Revolve {
                sketch: read_sketch_id(v)?,
                degrees: v.get("degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(360.0)),
                mode: read_mode(v)?,
            })
        }
        "solid" => {
            let shape = field(v, "shape", "a solid")?.as_str().unwrap_or("");
            let what = format!("a {shape}");
            let primitive = match shape {
                "box" => {
                    with(&["type", "shape", "dx", "dy", "dz", "at", "mode"])?;
                    Primitive::Box { dx: expr_field(v, "dx", &what)?, dy: expr_field(v, "dy", &what)?, dz: expr_field(v, "dz", &what)? }
                }
                "cylinder" => {
                    with(&["type", "shape", "radius", "height", "at", "mode"])?;
                    Primitive::Cylinder { radius: expr_field(v, "radius", &what)?, height: expr_field(v, "height", &what)? }
                }
                "cone" => {
                    with(&["type", "shape", "r0", "r1", "height", "at", "mode"])?;
                    Primitive::Cone { r0: expr_field(v, "r0", &what)?, r1: expr_field(v, "r1", &what)?, height: expr_field(v, "height", &what)? }
                }
                "sphere" => {
                    with(&["type", "shape", "radius", "at", "mode"])?;
                    Primitive::Sphere { radius: expr_field(v, "radius", &what)? }
                }
                "torus" => {
                    with(&["type", "shape", "major", "minor", "at", "mode"])?;
                    Primitive::Torus { major: expr_field(v, "major", &what)?, minor: expr_field(v, "minor", &what)? }
                }
                other => return Err(Error::InvalidInput(format!("unknown solid \"{other}\"; expected box, cylinder, cone, sphere or torus"))),
            };
            let at = match v.get("at") {
                Some(p) => read_placement(p)?,
                None => Placement::xy(0.0),
            };
            Ok(Op::Solid { primitive, at, mode: read_mode(v)? })
        }
        "fillet" => {
            with(&["type", "edges", "radius", "propagate"])?;
            Ok(Op::Fillet {
                edges: read_edge_select(field(v, "edges", "a fillet")?)?,
                radius: expr_field(v, "radius", "a fillet")?,
                propagate: match v.get("propagate") {
                    None => true,
                    Some(p) => p.as_bool().ok_or_else(|| Error::InvalidInput("\"propagate\" must be true or false".into()))?,
                },
            })
        }
        "chamfer" => {
            with(&["type", "edges", "distance"])?;
            Ok(Op::Chamfer { edges: read_edge_select(field(v, "edges", "a chamfer")?)?, distance: expr_field(v, "distance", "a chamfer")? })
        }
        other => Err(Error::InvalidInput(format!(
            "unknown feature type \"{other}\"; expected sketch, extrude, revolve, solid, fillet or chamfer"
        ))),
    }
}

fn feature_value(f: &Feature) -> Value {
    let mut items = vec![("id".to_string(), Value::Num(f.id.0 as f64)), ("name".to_string(), Value::str(&f.name))];
    if f.suppressed {
        items.push(("suppressed".to_string(), Value::Bool(true)));
    }
    if let Value::Obj(op) = op_value(&f.op) {
        items.extend(op);
    }
    Value::Obj(items)
}

fn read_feature(v: &Value) -> Result<Feature> {
    let id = field(v, "id", "a feature")?
        .as_f64()
        .filter(|n| n.fract() == 0.0 && *n >= 0.0 && *n < u32::MAX as f64)
        .ok_or_else(|| Error::InvalidInput("a feature's id must be a whole number".into()))?;
    let name = field(v, "name", "a feature")?.as_str().ok_or_else(|| Error::InvalidInput("a feature's name must be text".into()))?;
    let suppressed = match v.get("suppressed") {
        None => false,
        Some(s) => s.as_bool().ok_or_else(|| Error::InvalidInput("\"suppressed\" must be true or false".into()))?,
    };
    Ok(Feature { id: FeatureId(id as u32), name: name.to_string(), suppressed, op: read_op(v, &["id", "name", "suppressed"])? })
}

/// All distinct face names on a body, for listing and checks.
pub fn face_names(body: &Body) -> Vec<String> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for f in &body.faces {
        let name = FaceRef::from_tag(f.tag).name();
        if seen.insert(name.clone()) {
            out.push(name);
        }
    }
    out
}
