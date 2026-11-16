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

use coreshape::blend::{chamfer_with, fillet_with, BlendOptions};
use coreshape::boolean::{intersect, trim_sheet, trim_sheet_by_plane, union, BoolOp, Keep};
use coreshape::multi::{boolean_many, merged_tools, sweep, sweep_along, PathCurve, PathStep};
use coreshape::build::{
    box_solid, cone_frustum, cylinder, extrude, revolve, sphere, torus, SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP,
};
use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::profile::{PathBuilder, Profile, Seg, SegGeom, Wire};
use coreshape::spline::Spline;
use coreshape::surface::Surface;
use coreshape::curve::Curve;
use coreshape::topo::{Body, FaceTag};
use coreshape::delete::delete_faces;
use coreshape::helix::{helix, Helix};
use coreshape::sheet::{extrude_open, fill, revolve_open, sew, sheet_from_faces, sides_of};
use coreshape::flat::flat_face_profile;
use coreshape::thicken::thicken;
use coreshape::tweak::{draft, offset, shell};
pub use coreshape_sketch::expr::{self, Env, Expr};
pub use coreshape_sketch::json::{self, Value};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

mod keys;
mod planes;
mod sheet_metal;
mod up_to;
pub use sheet_metal::{Alignment, BendPosition, Corner, FlatBend, FlatEdge, FlatPattern, HemKind};
use keys::Keys;
pub use planes::{default_x, line_ref_value, plane_ref_value, point_ref_value, read_line_ref, read_plane_ref, read_point_ref, AlongEdge, LineRef, PlaneDef, PlaneRef, PointRef, TangentAt};
use planes::{Geo, Looker};
pub use keys::{face_keys, find_faces, FaceKey};

/// The file format's name and the version this code writes.
pub const FORMAT: &str = "coreshape-document";
pub const FORMAT_VERSION: f64 = 1.0;

/// Slot offset per closed region of a sketch. Leaves room for 999 holes per region.
const REGION_SLOTS: u32 = 1_000_000;
/// The most regions one sketch may make, which keeps every slot within a `u32`.
const MAX_REGIONS: usize = 4000;
/// Slot offset per hole within a region, matching `Profile::with_hole`.
const HOLE_SLOTS: u32 = 1000;
/// The walls an extrude sweeps from a part's face, `f9:wall.0` for the first face it lists, come after every slot a
/// sketch's regions can use.
const SLOT_WALL_BASE: u32 = SLOT_SEG_BASE + MAX_REGIONS as u32 * REGION_SLOTS;
/// The most faces one extrude may list, which keeps its walls' slots within a `u32`.
const MAX_EXTRUDED_FACES: usize = 100_000;

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

    /// The walls an extrude swept from the edges of the faces its `faces` entry `entry` picked. Each wall's key also
    /// holds the face across the edge it came from: `f9:wall.0[f2:side.3]`.
    pub fn wall(self, entry: u32) -> FaceRef {
        FaceRef { feature: self, slot: SLOT_WALL_BASE + entry }
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
    /// `f2:region.1.side.0`, and `f2:wall.0` for an extruded face's walls.
    pub fn name(self) -> String {
        let f = self.feature.0;
        match self.slot {
            SLOT_START_CAP => format!("f{f}:start"),
            SLOT_END_CAP => format!("f{f}:end"),
            slot if slot >= SLOT_WALL_BASE => format!("f{f}:wall.{}", slot - SLOT_WALL_BASE),
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
        if let Some(entry) = slot.strip_prefix("wall.") {
            let entry: u32 = entry.parse().map_err(|_| bad())?;
            if entry as usize >= MAX_EXTRUDED_FACES {
                return Err(bad());
            }
            return Ok(feature.wall(entry));
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
    /// A flat face this follows, for a sketch: at each rebuild the plane is that face's plane, wherever it's got to. The
    /// origin keeps its place within the plane, as a sketch's position on a face is its own, and the normal keeps the
    /// side of the face it started on.
    pub on_face: Option<FaceKey>,
    /// A plane this sits on: at each rebuild its origin, normal and x are that plane's, wherever it's got to, and the
    /// numbers above aren't used.
    pub on_plane: Option<PlaneRef>,
}

impl Placement {
    /// The XY plane, offset along Z.
    pub fn xy(z: impl Into<Expr>) -> Placement {
        Placement { origin: point3(0.0, 0.0, z), z: point3(0.0, 0.0, 1.0), x: point3(1.0, 0.0, 0.0), on_face: None, on_plane: None }
    }

    pub fn at(origin: Point3) -> Placement {
        Placement { origin, z: point3(0.0, 0.0, 1.0), x: point3(1.0, 0.0, 0.0), on_face: None, on_plane: None }
    }

    /// Placed with the given axis and x direction.
    pub fn new(origin: Point3, z: Point3, x: Point3) -> Placement {
        Placement { origin, z, x, on_face: None, on_plane: None }
    }

    /// The placement of a frame computed in code.
    pub fn from_frame(f: &Frame) -> Placement {
        Placement {
            origin: point3(f.origin.x, f.origin.y, f.origin.z),
            z: point3(f.z.x, f.z.y, f.z.z),
            x: point3(f.x.x, f.x.y, f.x.z),
            on_face: None,
            on_plane: None,
        }
    }

    /// On a plane, following it: Top, Front, Right, a plane feature or a flat face.
    pub fn on_plane(plane: PlaneRef) -> Placement {
        Placement { on_plane: Some(plane), ..Placement::xy(0.0) }
    }

    /// This placement, following a flat face from here on.
    pub fn following(mut self, face: FaceKey) -> Placement {
        self.on_face = Some(face);
        self
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
    /// A smooth curve from where the path is, through these points, ending on the last.
    Spline { through: Vec<Point2> },
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
    /// A chain of moves that isn't closed, for a surface to sweep. It's in a region's `outer` with nothing else in the region.
    OpenPath { start: Point2, moves: Vec<Move> },
    /// An ellipse with radii along the sketch's axes, turned counter-clockwise by `degrees`.
    Ellipse { cx: Expr, cy: Expr, rx: Expr, ry: Expr, degrees: Expr },
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
            Shape::Path { start, moves } | Shape::OpenPath { start, moves } => {
                let mut path = PathBuilder::new(at(start)?);
                for m in moves {
                    path = match m {
                        Move::Line { to } => path.line_to(at(to)?),
                        Move::Arc { to, center, ccw } => path.arc_to(at(to)?, at(center)?, *ccw),
                        Move::Spline { through } => {
                            let here = path.cursor();
                            let mut points = vec![v3(here.x, here.y, 0.0)];
                            for p in through {
                                let q = at(p)?;
                                points.push(v3(q.x, q.y, 0.0));
                            }
                            path.spline_to(Spline::through(&points)?)
                        }
                    };
                }
                if matches!(self, Shape::OpenPath { .. }) {
                    path.open()
                } else {
                    path.close()
                }
            }
            Shape::Ellipse { cx, cy, rx, ry, degrees } => {
                let turn = num(degrees, env)?.to_radians();
                let (x, y) = (v3(turn.cos(), turn.sin(), 0.0), v3(-turn.sin(), turn.cos(), 0.0));
                let center = v3(num(cx, env)?, num(cy, env)?, 0.0);
                let spline = Spline::ellipse_arc(center, x, y, num(rx, env)?, num(ry, env)?, 0.0, TAU)?;
                Wire { segs: vec![Seg { geom: SegGeom::Spline(Arc::new(spline)), id: 0 }] }
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

/// The open paths among a sketch's regions, which no region is made from.
fn resolve_open(regions: &[Region], env: &Env) -> Result<Vec<Wire>> {
    let mut out = Vec::new();
    for region in regions {
        if !matches!(region.outer, Shape::OpenPath { .. }) {
            continue;
        }
        if !region.holes.is_empty() || region.repeat.is_some() {
            return Err(Error::InvalidProfile("an open path can't have holes or be repeated".into()));
        }
        let wire = region.outer.resolve(env)?;
        wire.check_open()?;
        out.push(wire);
    }
    Ok(out)
}

fn resolve_regions(regions: &[Region], env: &Env) -> Result<Vec<Profile>> {
    let mut out = Vec::new();
    for region in regions {
        if matches!(region.outer, Shape::OpenPath { .. }) {
            continue;
        }
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
    /// All edges bounding the faces a key picks, which a plain name can't narrow to one copy.
    OnFaceKey(FaceKey),
    /// Edges between the faces two keys pick.
    BetweenKeys(FaceKey, FaceKey),
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
    fn matches(&self, body: &Body, e: usize, uses: &[Vec<(usize, usize, usize, bool)>], env: &Env, keys: &Keys) -> Result<bool> {
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
            EdgeSelect::OnFaceKey(k) => uses[e].iter().any(|u| keys.matches(body, u.0, k)),
            EdgeSelect::BetweenKeys(a, b) => match uses[e].as_slice() {
                [x, y] => {
                    (keys.matches(body, x.0, a) && keys.matches(body, y.0, b)) || (keys.matches(body, x.0, b) && keys.matches(body, y.0, a))
                }
                _ => false,
            },
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
                    if q.matches(body, e, uses, env, keys)? {
                        return Ok(true);
                    }
                }
                false
            }
            EdgeSelect::Both(list) => {
                for q in list {
                    if !q.matches(body, e, uses, env, keys)? {
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
            EdgeSelect::OnFaceKey(k) => k.references(out),
            EdgeSelect::BetweenKeys(a, b) => {
                a.references(out);
                b.references(out);
            }
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

/// How a new solid combines with the bodies a feature acts on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Add,
    Remove,
    Intersect,
    /// Combines with nothing, and each connected solid becomes a body of its own, so that parts stay parts, and can touch
    /// or overlap. Joining them would lose that, and can fail where they meet.
    NewBody,
    /// Makes a sheet and not a solid: what an extrude, revolve, sweep, loft or helix would have made, without its caps, and
    /// an extrude or revolve can sweep a sketch's open curves as well. It combines with nothing, and each connected piece is a
    /// body of its own.
    Surface,
}

/// What cuts a sheet.
#[derive(Clone, Debug, PartialEq)]
pub enum TrimTool {
    /// A solid body.
    Body(BodyId),
    /// A plane. `Outside` is the side its normal points to.
    Plane(PlaneRef),
    /// A sketch's regions, swept through the sheet along the sketch's normal, as a cookie cutter is.
    Sketch(FeatureId),
}

/// Which part of a cut sheet stays.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrimKeep {
    Outside,
    Inside,
    /// Both, as bodies of their own.
    Both,
}

/// Which body of a document: the feature that made it, and which of that feature's bodies it is. Written `f3` for the
/// first or only one and `f3.2` for the others. A body is one connected solid, with any voids inside it. A cut that splits
/// one leaves the name on the piece that was there before, and the others are named for the feature that split it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BodyId {
    pub feature: FeatureId,
    pub index: u32,
}

impl BodyId {
    pub fn text(self) -> String {
        if self.index == 0 {
            format!("f{}", self.feature.0)
        } else {
            format!("f{}.{}", self.feature.0, self.index)
        }
    }

    pub fn parse(text: &str) -> Result<BodyId> {
        let bad = || Error::InvalidInput(format!("\"{text}\" isn't the name of a body: they look like f3 or f3.2"));
        let rest = text.strip_prefix('f').ok_or_else(bad)?;
        let (feature, index) = rest.split_once('.').unwrap_or((rest, "0"));
        let digits = |t: &str| !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit());
        if !digits(feature) || !digits(index) {
            return Err(bad());
        }
        Ok(BodyId { feature: FeatureId(feature.parse().map_err(|_| bad())?), index: index.parse().map_err(|_| bad())? })
    }
}

/// Some edges and their radius, in a fillet with more than one.
#[derive(Clone, Debug, PartialEq)]
pub struct RadiusSet {
    pub edges: EdgeSelect,
    pub radius: RadiusLaw,
}

/// What radius a set of edges has.
#[derive(Clone, Debug, PartialEq)]
pub enum RadiusLaw {
    Constant(Expr),
    /// By how far along the chain of edges, from 0 to 1, in straight lines between these. It's measured from the end nearest
    /// `start`, or without one, from the end with the lowest x, then y, then z.
    Along { stations: Vec<(Expr, Expr)>, start: Option<Point3> },
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
    /// A sketch drawn with constraints and dimensions, solved at each rebuild. Dimensions may be
    /// expressions over the document's parameters. A face swept from it is named by the entity it
    /// came from, `f3:side.7` for entity 7, so the name doesn't depend on drawing order.
    ConstrainedSketch { plane: Placement, sketch: coreshape_sketch::Sketch },
    /// Extrudes a sketch along its normal, from `start` for `length`.
    /// With `regions` it takes those, from any sketches, and otherwise whatever `sketch` gives. With `faces` it takes those
    /// flat faces of parts too, each along its own outward normal, holes and all. Its walls are named `f9:wall.0` for the
    /// first face listed, with the face across the edge in its key, and its ends are `start` and `end`, with the face they
    /// came from in their key. With an `end` other than `Blind`, `length` isn't used, and is only kept for going back to
    /// it. `opposite` goes the other way.
    Extrude { sketch: Option<FeatureId>, start: Expr, length: Expr, mode: Mode, regions: Vec<RegionPick>, faces: Vec<FaceKey>, end: ExtrudeEnd, opposite: bool },
    /// Revolves a sketch. With an `axis`, about that line in the sketch's plane, as drawn, turning right-handed around its
    /// direction. Without one, about the sketch's Z axis, with profile x the distance from the axis and y the height along it.
    /// With `regions` it takes those, from any sketches, and otherwise whatever `sketch` gives. With an `axis`, a region from
    /// another sketch has to be in the same plane as `sketch`, and without one it goes around its own sketch's Z axis.
    Revolve { sketch: FeatureId, degrees: Expr, mode: Mode, axis: Option<RevolveAxis>, regions: Vec<RegionPick> },
    Solid { primitive: Primitive, at: Placement, mode: Mode },
    /// The solids in a STEP file, kept here in full so that the document stands alone. Their faces are named by their
    /// place in the file: `f3:side.0` is the first. A piece cut from a wide face keeps that face's name.
    ImportStep { text: String, mode: Mode },
    /// Each distinct solid in a STEP file once, as a new body in its own coordinates, for an assembly to use.
    ImportStepParts { text: String },
    Fillet { edges: EdgeSelect, radius: Expr, propagate: bool },
    /// A fillet with more than one radius: on different edges, which can meet at a corner, or varying along a chain.
    FilletRadii { sets: Vec<RadiusSet> },
    Chamfer { edges: EdgeSelect, distance: Expr, propagate: bool },
    /// A solid through sections in order, each a sketch holding one outline or one region of a sketch, with free-form sides.
    /// Straight between two sections, and a smooth cubic through three or more. Sides are named from the first section.
    Loft { sections: Vec<LoftSection>, mode: Mode },
    /// Sweeps a sketch along a smooth path of lines and arcs: steps from the sketch plane, heading along its normal, or a
    /// sketch's curves, or a part's edges. See `SweepPath`.
    Sweep { sketch: FeatureId, path: SweepPath, mode: Mode },
    /// Sweeps a sketch along a helix around the axis through `point`: a spring, a thread or a twisted bar. `pitch` is how
    /// far along the axis one turn goes. The sides are named from the sketch, and the ends are the start and end caps.
    Helix { sketch: FeatureId, point: Point3, axis: Point3, pitch: Expr, turns: Expr, left_handed: bool, mode: Mode },
    /// Deletes faces, picked by name or key, and closes the gap by letting their neighbours grow. It adds no faces, and
    /// what's left keeps its names.
    DeleteFace { faces: Vec<FaceKey> },
    /// A solid between faces and a copy of them moved along their normals, `thickness` outward, or inward if negative.
    /// Named as this feature's start cap for the near faces, its end cap for the far ones, and sides for the walls.
    Thicken { faces: Vec<FaceKey>, thickness: Expr, mode: Mode },
    /// Repeats earlier features, or the whole body if `of` is empty. A copy keeps the names of what
    /// it copied, so a face's name covers the original and all its copies.
    Pattern { of: Vec<FeatureId>, along: PatternPath },
    /// Reflects earlier features, or the whole body if `of` is empty, across a plane, adding them to
    /// the originals. A reflection keeps names too.
    Mirror { of: Vec<FeatureId>, plane: PlaneRef },
    /// Moves every face outward by a distance, or inward if negative. Sharp edges stay sharp.
    Offset { distance: Expr },
    /// Hollows the body to walls of a thickness, leaving named faces open. An inner wall is named
    /// for this feature, with the slot of the face it follows: `f9:side.3` lies behind `f2:side.3`.
    Shell { open: Vec<FaceRef>, thickness: Expr },
    /// Tapers named faces, keeping them in place on the neutral plane, along a pull that is its normal. A positive angle
    /// narrows the part along the pull, and widens a hole.
    Draft { faces: Vec<FaceRef>, neutral: PlaneRef, degrees: Expr },
    /// Moves the bodies it acts on, or with `copy` leaves them and adds moved copies of them, as bodies of their own.
    Transform { motion: Motion, copy: bool },
    /// Deletes the bodies it names.
    DeleteBodies,
    /// A Boolean between bodies. `Union` and `Intersect` take the bodies the feature names along with the `tools`.
    /// `Subtract` cuts the `tools` from the bodies the feature names, its targets. Without `keep_tools` the tools go.
    BooleanBodies { operation: BodyBoolean, tools: Vec<BodyId>, keep_tools: bool },
    /// Flat sheets filling a sketch's regions, facing along its normal, and named as this feature's end cap.
    Fill { sketch: FeatureId },
    /// Copies of faces, as a sheet. The bodies they're on stay as they are, and a copy keeps its face's name.
    CopyFaces { faces: Vec<FaceKey> },
    /// Joins sheets along the boundaries they have in common, those it names or all of them. If that closes them, it's a solid.
    /// `tolerance` is how far apart two boundaries can be and still be one.
    Sew { tolerance: Option<Expr> },
    /// Cuts sheets, those it names or all of them, and keeps a part of each.
    Trim { tool: TrimTool, keep: TrimKeep },
    /// A plane, not a part, worked out again at every rebuild from what it names. `flip` turns its normal round, keeping
    /// its origin and x. Sketches can sit on it, and other features can name it.
    Plane { def: PlaneDef, flip: bool },
    /// Sheet metal, a part of one thickness that can be laid flat: a plate from a sketch's one region, or walls `depth`
    /// deep along its one chain of lines, with a bend of inner radius `bend_radius` (default the thickness) at each corner.
    /// The sheet is on the region's side of the sketch, along its normal, or on the chain's left; `flip` puts it on the
    /// other side. `k_factor` (default 0.45) says where the neutral line is, as a fraction of the thickness from the
    /// inside of a bend, for laying it flat. It's a new part.
    /// The part's defaults for what later features do to it: `minimal_gap` (default a tenth of the thickness) for closed
    /// corners and straight hems, and `corner`, how flanges of one feature meet.
    SheetMetal {
        sketch: FeatureId,
        thickness: Expr,
        bend_radius: Option<Expr>,
        k_factor: Option<Expr>,
        depth: Option<Expr>,
        flip: bool,
        minimal_gap: Option<Expr>,
        corner: Corner,
    },
    /// A bend and a wall along each edge picked, where a sheet metal wall's face meets its side. It bends toward the face
    /// the edge is on, by `degrees` (default 90), or the other way with `flip`; `length` is from the outer virtual sharp;
    /// `alignment` says where the bend goes; `bend_radius` defaults to the part's. Where two of its edges meet at a corner,
    /// `corner` (default the part's) leaves it open or closes it, leaving `gap` (default the part's minimal gap).
    Flange {
        edges: EdgeSelect,
        length: Expr,
        degrees: Option<Expr>,
        alignment: Alignment,
        bend_radius: Option<Expr>,
        flip: bool,
        corner: Option<Corner>,
        gap: Option<Expr>,
    },
    /// An edge folded back over its wall, as `kind` says. `length` is for straight and rounded hems from the outside of
    /// the fold to the end, and for a teardrop the straight part after its fold; a roll has none. A straight hem leaves
    /// `gap` (default the part's minimal gap) between the wall and what comes back over it; the others turn at
    /// `bend_radius` (default the part's), and a teardrop or a roll by `degrees` (default 225 or 270).
    Hem {
        edges: EdgeSelect,
        kind: HemKind,
        length: Option<Expr>,
        gap: Option<Expr>,
        bend_radius: Option<Expr>,
        degrees: Option<Expr>,
        alignment: Alignment,
        flip: bool,
    },
    /// The regions of a sketch on the plane of a sheet metal wall, made the sheet's thickness and joined to it.
    Tab { sketch: FeatureId },
    /// A sheet metal part bent along the one line in a sketch on the plane of one of its walls, by `degrees` (default 90)
    /// at `bend_radius` (default the part's). The part on the line's left moves, seen from along the sketch's normal, or
    /// on its right with `right`; it bends toward the side the sketch's normal points to, or the other way with `flip`.
    /// `position` says where the bend is against the line.
    Bend { sketch: FeatureId, degrees: Option<Expr>, bend_radius: Option<Expr>, position: BendPosition, flip: bool, right: bool },
}

/// A line in a sketch's plane to revolve around.
#[derive(Clone, Debug, PartialEq)]
pub struct RevolveAxis {
    pub point: Point2,
    pub direction: Point2,
}

/// How a `Transform` moves things.
#[derive(Clone, Debug, PartialEq)]
pub enum ExtrudeEnd {
    /// At its length.
    Blind,
    /// As far as the parts it acts on go.
    ThroughAll,
    /// At the next surface in the way: what it would run into, or for a cut, the far side of the material.
    UpToNext,
    /// At a face's surface, extended as far as it needs to be.
    UpToFace(FaceKey),
    /// At the first surface of this part in the way.
    UpToPart(BodyId),
    /// At a plane through a vertex, square to the direction.
    UpToVertex(VertexRef),
    /// At a plane: Top, Front, Right, a plane feature, or one given as numbers.
    UpToPlane(PlaneRef),
}

/// A vertex, as the end of an extrude.
#[derive(Clone, Debug, PartialEq)]
pub enum VertexRef {
    /// The vertex these faces share, so that it follows them. `near` says which, if they share several.
    Faces { faces: Vec<FaceKey>, near: Option<Point3> },
    /// A place.
    Point(Point3),
}

/// A region of a constrained sketch, by its id from `sketch_arrangement`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegionPick {
    pub sketch: FeatureId,
    pub region: coreshape_sketch::RegionId,
}

/// One section of a loft: a sketch, which has to hold one outline, or one region of a constrained sketch, so that a sketch
/// with several outlines can feed it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoftSection {
    pub sketch: FeatureId,
    pub region: Option<coreshape_sketch::RegionId>,
}

impl From<FeatureId> for LoftSection {
    fn from(sketch: FeatureId) -> LoftSection {
        LoftSection { sketch, region: None }
    }
}

impl From<RegionPick> for LoftSection {
    fn from(pick: RegionPick) -> LoftSection {
        LoftSection { sketch: pick.sketch, region: Some(pick.region) }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Motion {
    Translate { by: Point3 },
    Rotate { point: Point3, axis: Point3, degrees: Expr },
    /// Carries one placement onto another, as Onshape's transform by mate connectors does.
    Frames { from: Placement, to: Placement },
}

/// A Boolean between bodies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyBoolean {
    Union,
    Subtract,
    Intersect,
}

/// One step along a sweep's path. See `coreshape::PathStep`.
#[derive(Clone, Debug, PartialEq)]
pub enum SweepStep {
    Line { length: Expr },
    /// Bends through `degrees` around a center `radius` away from the path, in the direction
    /// `toward_degrees` counter-clockwise from the sketch's x axis.
    Arc { radius: Expr, degrees: Expr, toward_degrees: Expr },
}

/// What a sweep goes along.
#[derive(Clone, Debug, PartialEq)]
pub enum SweepPath {
    /// Steps from the sketch plane, heading along its normal, each carrying on from where the last ended.
    Steps(Vec<SweepStep>),
    /// The curves of a sketch, end to end: its open curves, or else the outline of its one region.
    Sketch(FeatureId),
    /// Edges of a part, end to end.
    Edges(EdgeSelect),
}

/// Where a pattern's copies go. `count` includes the original.
#[derive(Clone, Debug, PartialEq)]
pub enum PatternPath {
    /// Along a line's direction. Only its direction counts.
    Linear { direction: LineRef, count: Expr, spacing: Expr },
    /// Around a line. Copies are evenly spaced over `degrees`, right-handed about its direction. A full turn has no copy
    /// at the end, as that would be the original; anything less goes from the original to the end of the angle.
    Circular { axis: LineRef, count: Expr, degrees: Expr },
}

impl Op {
    pub fn sketch(plane: Placement, regions: Vec<Region>) -> Op {
        Op::Sketch { plane, regions }
    }

    pub fn extrude(sketch: FeatureId, start: impl Into<Expr>, length: impl Into<Expr>, mode: Mode) -> Op {
        Op::Extrude { sketch: Some(sketch), start: start.into(), length: length.into(), mode, regions: Vec::new(), faces: Vec::new(), end: ExtrudeEnd::Blind, opposite: false }
    }

    /// Extrudes flat faces of parts, each along its own outward normal.
    pub fn extrude_faces(faces: Vec<FaceKey>, length: impl Into<Expr>, mode: Mode) -> Op {
        Op::Extrude { sketch: None, start: Expr::number(0.0), length: length.into(), mode, regions: Vec::new(), faces, end: ExtrudeEnd::Blind, opposite: false }
    }

    pub fn revolve(sketch: FeatureId, degrees: impl Into<Expr>, mode: Mode) -> Op {
        Op::Revolve { sketch, degrees: degrees.into(), mode, axis: None, regions: Vec::new() }
    }

    pub fn solid(primitive: Primitive, at: Placement, mode: Mode) -> Op {
        Op::Solid { primitive, at, mode }
    }

    /// A fillet that follows tangent neighbours, the default.
    pub fn fillet(edges: EdgeSelect, radius: impl Into<Expr>) -> Op {
        Op::Fillet { edges, radius: radius.into(), propagate: true }
    }

    /// A chamfer that follows tangent neighbours, the default.
    pub fn chamfer(edges: EdgeSelect, distance: impl Into<Expr>) -> Op {
        Op::Chamfer { edges, distance: distance.into(), propagate: true }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Op::Sketch { .. } => "sketch",
            Op::ConstrainedSketch { .. } => "constrained_sketch",
            Op::Extrude { .. } => "extrude",
            Op::Revolve { .. } => "revolve",
            Op::Solid { .. } => "solid",
            Op::ImportStep { .. } => "import_step",
            Op::ImportStepParts { .. } => "import_step_parts",
            Op::Fill { .. } => "fill",
            Op::CopyFaces { .. } => "copy_faces",
            Op::Sew { .. } => "sew",
            Op::Trim { .. } => "trim",
            Op::Fillet { .. } => "fillet",
            Op::FilletRadii { .. } => "fillet_radii",
            Op::Chamfer { .. } => "chamfer",
            Op::Sweep { .. } => "sweep",
            Op::Helix { .. } => "helix",
            Op::DeleteFace { .. } => "delete_face",
            Op::Thicken { .. } => "thicken",
            Op::Loft { .. } => "loft",
            Op::Pattern { .. } => "pattern",
            Op::Mirror { .. } => "mirror",
            Op::Transform { .. } => "transform",
            Op::DeleteBodies => "delete_bodies",
            Op::BooleanBodies { .. } => "boolean",
            Op::Offset { .. } => "offset",
            Op::Shell { .. } => "shell",
            Op::Draft { .. } => "draft",
            Op::Plane { .. } => "plane",
            Op::SheetMetal { .. } => "sheet_metal",
            Op::Flange { .. } => "flange",
            Op::Hem { .. } => "hem",
            Op::Tab { .. } => "tab",
            Op::Bend { .. } => "bend",
        }
    }

    /// Whether this makes profiles for later features, leaving the body alone.
    fn is_sketch(&self) -> bool {
        matches!(self, Op::Sketch { .. } | Op::ConstrainedSketch { .. })
    }

    /// Whether this makes something to build on, a sketch or a plane, and acts on no body.
    fn is_reference(&self) -> bool {
        self.is_sketch() || matches!(self, Op::Plane { .. })
    }

    /// What this places things by: placements that follow a face or a plane, and the planes, points and lines it names.
    fn geo(&self) -> Vec<Geo<'_>> {
        let following = |p: &Placement| p.on_face.is_some() || p.on_plane.is_some();
        match self {
            Op::Sketch { plane, .. } | Op::ConstrainedSketch { plane, .. } | Op::Solid { at: plane, .. } if following(plane) => vec![Geo::Place(plane)],
            Op::Transform { motion: Motion::Frames { from, to }, .. } => [from, to].into_iter().filter(|p| following(p)).map(Geo::Place).collect(),
            Op::Plane { def, flip } => vec![Geo::Def(def, *flip)],
            Op::Mirror { plane, .. } | Op::Draft { neutral: plane, .. } | Op::Trim { tool: TrimTool::Plane(plane), .. } | Op::Extrude { end: ExtrudeEnd::UpToPlane(plane), .. } => vec![Geo::Plane(plane)],
            Op::Pattern { along: PatternPath::Linear { direction: line, .. } | PatternPath::Circular { axis: line, .. }, .. } => vec![Geo::Line(line)],
            _ => Vec::new(),
        }
    }

    /// How this combines with a body, for the features that make a solid.
    pub fn mode(&self) -> Option<Mode> {
        match self {
            Op::Extrude { mode, .. }
            | Op::Revolve { mode, .. }
            | Op::Solid { mode, .. }
            | Op::ImportStep { mode, .. }
            | Op::Loft { mode, .. }
            | Op::Sweep { mode, .. }
            | Op::Helix { mode, .. }
            | Op::Thicken { mode, .. } => Some(*mode),
            Op::ImportStepParts { .. } => Some(Mode::NewBody),
            Op::Fill { .. } => Some(Mode::Surface),
            Op::SheetMetal { .. } => Some(Mode::NewBody),
            _ => None,
        }
    }

    /// Whether a feature with no list of bodies acts on sheets as well as solids. The rest only ever reach solids unless told.
    fn takes_sheets(&self) -> bool {
        matches!(self, Op::Transform { .. } | Op::Thicken { .. } | Op::CopyFaces { .. } | Op::Sew { .. } | Op::Trim { .. })
    }

    /// Whether this uses a sketch's open curves: to make a surface from, or as a sweep's path.
    fn uses_open_curves_of(&self, id: FeatureId) -> bool {
        self.surface_from() == Some(id)
            || matches!(self, Op::Sweep { path: SweepPath::Sketch(along), .. } if *along == id)
            || matches!(self, Op::SheetMetal { sketch, .. } | Op::Bend { sketch, .. } if *sketch == id)
    }

    /// The sketch this makes a surface from, whose open curves it can use.
    fn surface_from(&self) -> Option<FeatureId> {
        match self {
            Op::Extrude { sketch: Some(sketch), mode: Mode::Surface, .. } | Op::Revolve { sketch, mode: Mode::Surface, .. } => Some(*sketch),
            _ => None,
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Op::Sketch { .. } | Op::ConstrainedSketch { .. } => "Sketch",
            Op::Extrude { .. } => "Extrude",
            Op::Revolve { .. } => "Revolve",
            Op::Solid { .. } => "Solid",
            Op::ImportStep { .. } => "Imported STEP",
            Op::ImportStepParts { .. } => "Imported STEP parts",
            Op::Fill { .. } => "Fill",
            Op::CopyFaces { .. } => "Copied faces",
            Op::Sew { .. } => "Sew",
            Op::Trim { .. } => "Trim",
            Op::Fillet { .. } | Op::FilletRadii { .. } => "Fillet",
            Op::Chamfer { .. } => "Chamfer",
            Op::Sweep { .. } => "Sweep",
            Op::Helix { .. } => "Helix",
            Op::DeleteFace { .. } => "Delete face",
            Op::Thicken { .. } => "Thicken",
            Op::Loft { .. } => "Loft",
            Op::Pattern { .. } => "Pattern",
            Op::Mirror { .. } => "Mirror",
            Op::Transform { .. } => "Transform",
            Op::DeleteBodies => "Delete bodies",
            Op::BooleanBodies { .. } => "Boolean",
            Op::Offset { .. } => "Offset",
            Op::Shell { .. } => "Shell",
            Op::Draft { .. } => "Draft",
            Op::Plane { .. } => "Plane",
            Op::SheetMetal { .. } => "Sheet metal",
            Op::Flange { .. } => "Flange",
            Op::Hem { .. } => "Hem",
            Op::Tab { .. } => "Tab",
            Op::Bend { .. } => "Bend",
        }
    }

    /// Features this one needs to come after.
    pub fn references(&self) -> Vec<FeatureId> {
        let mut out = Vec::new();
        match self {
            Op::Extrude { sketch, regions, faces, end, .. } => {
                out.extend(*sketch);
                out.extend(regions.iter().map(|r| r.sketch));
                for key in faces {
                    key.references(&mut out);
                }
                match end {
                    ExtrudeEnd::UpToFace(key) => key.references(&mut out),
                    ExtrudeEnd::UpToPart(part) => out.push(part.feature),
                    ExtrudeEnd::UpToVertex(VertexRef::Faces { faces, .. }) => {
                        for key in faces {
                            key.references(&mut out);
                        }
                    }
                    _ => {}
                }
            }
            Op::Revolve { sketch, regions, .. } => {
                out.push(*sketch);
                out.extend(regions.iter().map(|r| r.sketch));
            }
            Op::Sweep { sketch, path, .. } => {
                out.push(*sketch);
                match path {
                    SweepPath::Steps(_) => {}
                    SweepPath::Sketch(id) => out.push(*id),
                    SweepPath::Edges(edges) => edges.references(&mut out),
                }
            }
            Op::Helix { sketch, .. } | Op::Fill { sketch } | Op::SheetMetal { sketch, .. } | Op::Tab { sketch } | Op::Bend { sketch, .. } => out.push(*sketch),
            Op::Loft { sections, .. } => out.extend(sections.iter().map(|s| s.sketch)),
            Op::BooleanBodies { tools, .. } => out.extend(tools.iter().map(|b| b.feature)),
            Op::Trim { tool: TrimTool::Body(body), .. } => out.push(body.feature),
            Op::Trim { tool: TrimTool::Sketch(sketch), .. } => out.push(*sketch),
            Op::DeleteFace { faces } | Op::Thicken { faces, .. } | Op::CopyFaces { faces } => {
                for key in faces {
                    key.references(&mut out);
                }
            }
            Op::Fillet { edges, .. } | Op::Chamfer { edges, .. } | Op::Flange { edges, .. } | Op::Hem { edges, .. } => edges.references(&mut out),
            Op::FilletRadii { sets } => {
                for set in sets {
                    set.edges.references(&mut out);
                }
            }
            Op::Pattern { of, .. } | Op::Mirror { of, .. } => out.extend_from_slice(of),
            Op::Shell { open: faces, .. } | Op::Draft { faces, .. } => out.extend(faces.iter().map(|f| f.feature)),
            // What its projected entities come from.
            Op::ConstrainedSketch { sketch, .. } => {
                for e in sketch.projected() {
                    if let Some(Ok(from)) = e.projection.as_ref().map(|p| read_projection(&p.from)) {
                        from.references(&mut out);
                    }
                }
            }
            _ => {}
        }
        for g in self.geo() {
            g.references(&mut out);
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
                Shape::Path { start, moves } | Shape::OpenPath { start, moves } => {
                    out.extend(start.iter());
                    for m in moves {
                        match m {
                            Move::Line { to } => out.extend(to.iter()),
                            Move::Arc { to, center, .. } => {
                                out.extend(to.iter());
                                out.extend(center.iter());
                            }
                            Move::Spline { through } => out.extend(through.iter().flatten()),
                        }
                    }
                }
                Shape::Ellipse { cx, cy, rx, ry, degrees } => out.extend([cx, cy, rx, ry, degrees]),
            }
        }
        match self {
            Op::ImportStep { .. } | Op::ImportStepParts { .. } | Op::Fill { .. } | Op::CopyFaces { .. } | Op::Tab { .. } => {}
            Op::Sew { tolerance } => out.extend(tolerance.iter()),
            Op::Trim { .. } => {}
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
            Op::ConstrainedSketch { plane, sketch } => {
                place(plane, &mut out);
                out.extend(sketch.constraints.iter().filter_map(|c| c.rule.value()));
            }
            Op::Extrude { start, length, end, .. } => {
                out.extend([start, length]);
                match end {
                    ExtrudeEnd::UpToVertex(VertexRef::Point(p)) | ExtrudeEnd::UpToVertex(VertexRef::Faces { near: Some(p), .. }) => out.extend(p.iter()),
                    _ => {}
                }
            }
            Op::Revolve { degrees, axis, .. } => {
                out.push(degrees);
                if let Some(axis) = axis {
                    out.extend(axis.point.iter());
                    out.extend(axis.direction.iter());
                }
            }
            Op::Transform { motion, .. } => match motion {
                Motion::Translate { by } => out.extend(by.iter()),
                Motion::Rotate { point, axis, degrees } => {
                    out.extend(point.iter());
                    out.extend(axis.iter());
                    out.push(degrees);
                }
                Motion::Frames { from, to } => {
                    place(from, &mut out);
                    place(to, &mut out);
                }
            },
            Op::DeleteBodies | Op::BooleanBodies { .. } => {}
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
            Op::Loft { .. } | Op::DeleteFace { .. } => {}
            Op::Helix { point, axis, pitch, turns, .. } => {
                out.extend(point.iter());
                out.extend(axis.iter());
                out.extend([pitch, turns]);
            }
            Op::Thicken { thickness, .. } => out.push(thickness),
            Op::Sweep { path, .. } => match path {
                SweepPath::Steps(steps) => {
                    for step in steps {
                        match step {
                            SweepStep::Line { length } => out.push(length),
                            SweepStep::Arc { radius, degrees, toward_degrees } => out.extend([radius, degrees, toward_degrees]),
                        }
                    }
                }
                SweepPath::Sketch(_) => {}
                SweepPath::Edges(edges) => edges.collect_exprs(&mut out),
            },
            Op::Pattern { along, .. } => match along {
                PatternPath::Linear { count, spacing, .. } => out.extend([count, spacing]),
                PatternPath::Circular { count, degrees, .. } => out.extend([count, degrees]),
            },
            Op::Mirror { .. } => {}
            Op::Offset { distance } => out.push(distance),
            Op::Shell { thickness, .. } => out.push(thickness),
            Op::Draft { degrees, .. } => out.push(degrees),
            Op::Fillet { edges, radius, .. } => {
                edges.collect_exprs(&mut out);
                out.push(radius);
            }
            Op::SheetMetal { thickness, bend_radius, k_factor, depth, minimal_gap, .. } => {
                out.push(thickness);
                out.extend(bend_radius.iter().chain(k_factor).chain(depth).chain(minimal_gap));
            }
            Op::Flange { edges, length, degrees, bend_radius, gap, .. } => {
                edges.collect_exprs(&mut out);
                out.push(length);
                out.extend(degrees.iter().chain(bend_radius).chain(gap));
            }
            Op::Hem { edges, length, gap, bend_radius, degrees, .. } => {
                edges.collect_exprs(&mut out);
                out.extend(length.iter().chain(gap).chain(bend_radius).chain(degrees));
            }
            Op::Bend { degrees, bend_radius, .. } => out.extend(degrees.iter().chain(bend_radius)),
            Op::FilletRadii { sets } => {
                for set in sets {
                    set.edges.collect_exprs(&mut out);
                    match &set.radius {
                        RadiusLaw::Constant(r) => out.push(r),
                        RadiusLaw::Along { stations, start } => {
                            for (at, r) in stations {
                                out.push(at);
                                out.push(r);
                            }
                            if let Some(p) = start {
                                out.extend(p.iter());
                            }
                        }
                    }
                }
            }
            Op::Chamfer { edges, distance, .. } => {
                edges.collect_exprs(&mut out);
                out.push(distance);
            }
            Op::Plane { .. } => {}
        }
        for g in self.geo() {
            g.exprs(&mut out);
        }
        out
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub suppressed: bool,
    /// The bodies this acts on, which Onshape calls its merge scope. Empty means all of them, and a feature leaves alone
    /// any it doesn't reach.
    pub scope: Vec<BodyId>,
    /// Whatever an app wants kept with this feature, such as where a sketch's dimension labels sit. It's kept as it is and
    /// changes nothing in the model.
    pub app: Option<Value>,
    pub op: Op,
}

impl Feature {
    /// The features this one refers to.
    fn references(&self) -> Vec<FeatureId> {
        let mut out = self.op.references();
        out.extend(self.scope.iter().map(|b| b.feature));
        out
    }
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
    /// Everything there is at the rollback position, or at the last feature that succeeded. With one body, that's the
    /// body. With several, they're side by side in one `Body`, good for showing, measuring and exporting, where a volume
    /// counts twice wherever parts overlap. Its faces and edges come body by body: see `NamedBody`.
    pub body: Option<Arc<Body>>,
    /// Each body on its own, oldest first.
    pub bodies: Vec<NamedBody>,
    /// Index in `steps` of the failed feature, if any.
    pub failed: Option<usize>,
    /// Each plane feature that built, where it is now, in history order.
    pub planes: Vec<PlaneAt>,
}

/// A plane feature, as a rebuild left it.
#[derive(Clone, Debug)]
pub struct PlaneAt {
    pub id: FeatureId,
    pub frame: Frame,
}

/// One of a document's bodies, as a rebuild left it.
#[derive(Clone, Debug)]
pub struct NamedBody {
    pub id: BodyId,
    pub body: Arc<Body>,
    /// Where its faces start in the outcome's `body`.
    pub first_face: usize,
    /// Where its edges start in the outcome's `body`.
    pub first_edge: usize,
}

impl Outcome {
    /// The body a face of the outcome's `body` belongs to.
    pub fn body_of_face(&self, face: usize) -> Option<&NamedBody> {
        let after = self.bodies.partition_point(|b| b.first_face <= face);
        self.bodies.get(after.checked_sub(1)?).filter(|b| face - b.first_face < b.body.faces.len())
    }

    /// The body an edge of the outcome's `body` belongs to.
    pub fn body_of_edge(&self, edge: usize) -> Option<&NamedBody> {
        let after = self.bodies.partition_point(|b| b.first_edge <= edge);
        self.bodies.get(after.checked_sub(1)?).filter(|b| edge - b.first_edge < b.body.edges.len())
    }

    /// A body by name.
    pub fn find_body(&self, id: BodyId) -> Option<&NamedBody> {
        self.bodies.iter().find(|b| b.id == id)
    }

    /// Every body in one STEP file, each as its own solids.
    pub fn step_text(&self, name: &str) -> Option<String> {
        if self.bodies.is_empty() {
            return None;
        }
        let list: Vec<&Body> = self.bodies.iter().map(|b| b.body.as_ref()).collect();
        Some(coreshape::export::step::write_step_bodies(&list, name))
    }

    pub fn rebuilt_count(&self) -> usize {
        self.steps.iter().filter(|s| s.state == StepState::Rebuilt).count()
    }
}

#[derive(Clone)]
struct Cached {
    key: u64,
    /// What became of the bodies it acted on: what each is now, or nothing if it's gone.
    changed: Vec<(BodyId, Option<Arc<Body>>)>,
    /// The bodies it made of their own.
    added: Vec<Arc<Body>>,
    /// A sketch's output, kept separately because a sketch doesn't touch the body.
    sketch: Option<SketchOutput>,
    /// The solids this feature applied, and how, for a pattern or mirror to use again.
    tools: Option<(Arc<Vec<Body>>, Mode)>,
    /// A plane feature's plane.
    plane: Option<Frame>,
}

/// What running one feature produced.
struct RunResult {
    changed: Vec<(BodyId, Option<Arc<Body>>)>,
    added: Vec<Arc<Body>>,
    sketch: Option<SketchOutput>,
    tools: Option<(Arc<Vec<Body>>, Mode)>,
    plane: Option<Frame>,
}

/// A body during a rebuild. Its key covers everything it depends on, so a change to one body doesn't rebuild another.
struct Held {
    id: BodyId,
    body: Arc<Body>,
    key: u64,
}

/// A feature's results, applied to the bodies so far. A body keeps its place in the list however it changes.
fn take_result(held: &mut Vec<Held>, feature: FeatureId, key: u64, changed: &[(BodyId, Option<Arc<Body>>)], added: &[Arc<Body>]) {
    for (id, now) in changed {
        let Some(at) = held.iter().position(|h| h.id == *id) else { continue };
        match now {
            Some(body) => {
                held[at].body = body.clone();
                held[at].key = hash_text(key, &id.text());
            }
            None => {
                held.remove(at);
            }
        }
    }
    for (k, body) in added.iter().enumerate() {
        held.push(Held { id: BodyId { feature, index: k as u32 }, body: body.clone(), key: hash_text(key, &format!("+{k}")) });
    }
}

/// Everything there is as one body, for finding faces and edges by name, whose counting takes in all of them.
fn whole_of(all: &[Held]) -> Option<Arc<Body>> {
    match all {
        [] => None,
        [only] => Some(only.body.clone()),
        many => {
            let list: Vec<&Body> = many.iter().map(|h| h.body.as_ref()).collect();
            Some(Arc::new(Body::side_by_side(&list)))
        }
    }
}

/// What a sketch's projected entity comes from.
enum ProjectFrom {
    Vertex(VertexRef),
    Edge(EdgeSelect),
}

impl ProjectFrom {
    fn references(&self, out: &mut Vec<FeatureId>) {
        match self {
            ProjectFrom::Vertex(VertexRef::Faces { faces, .. }) => {
                for key in faces {
                    key.references(out);
                }
            }
            ProjectFrom::Vertex(VertexRef::Point(_)) => {}
            ProjectFrom::Edge(edges) => edges.references(out),
        }
    }
}

/// `{"vertex": <vertex>}`, as an extrude's up_to_vertex takes it, or `{"edge": <edge selection>}` picking one edge.
fn read_projection(from: &Value) -> Result<ProjectFrom> {
    if let Some(v) = from.get("vertex") {
        known_keys(from, &["vertex"])?;
        Ok(ProjectFrom::Vertex(read_vertex_ref(v)?))
    } else if let Some(e) = from.get("edge") {
        known_keys(from, &["edge"])?;
        Ok(ProjectFrom::Edge(read_edge_select(e)?))
    } else {
        Err(Error::InvalidInput("a projection is from {\"vertex\": {\"faces\": [...], \"near\": [x, y, z]}} or {\"edge\": <edge selection>}".into()))
    }
}

/// A sketch with each projected entity put where the parts now put it, seen along the sketch's normal from `frame`.
fn placed_sketch(sketch: &coreshape_sketch::Sketch, frame: &Frame, all: &[Held], env: &Env) -> Result<coreshape_sketch::Sketch> {
    let mut out = sketch.clone();
    let mut everything: Option<Arc<Body>> = None;
    for e in sketch.projected() {
        let from = &e.projection.as_ref().expect("projected").from;
        let fail = |why: String| Error::InvalidProfile(format!("entity {} is projected from {}, and {why}", e.id.0, from.to_text()));
        let geometry = match read_projection(from).map_err(|why| fail(why.to_string()))? {
            ProjectFrom::Vertex(vertex) => {
                let at = up_to::vertex_at(&vertex, all, env).map_err(|why| fail(why.to_string()))?;
                let q = frame.to_local(at);
                coreshape_sketch::Geometry::Point { at: [q.x, q.y] }
            }
            ProjectFrom::Edge(query) => {
                if everything.is_none() {
                    everything = whole_of(all);
                }
                let body = everything.as_ref().ok_or_else(|| fail("there's no part before the sketch".into()))?;
                let edges = select_edges(&query, body, env).map_err(|why| fail(why.to_string()))?;
                let [one] = edges.as_slice() else {
                    return Err(fail(format!("that picks {} edges, where a projection takes one", edges.len())));
                };
                project_edge(body, *one, frame).map_err(|why| fail(why.to_string()))?
            }
        };
        out.place_projected(e.id, geometry).map_err(Error::InvalidInput)?;
    }
    Ok(out)
}

/// An edge seen along a plane's normal, in the plane's coordinates: a line, arc, circle, ellipse, elliptical arc or
/// spline, or a point if it's seen end on, or a line if a curve is seen edge on.
fn project_edge(body: &Body, e: usize, frame: &Frame) -> Result<coreshape_sketch::Geometry> {
    use coreshape_sketch::Geometry as G;
    let edge = &body.edges[e];
    let flat = |p: Vec3| {
        let q = frame.to_local(p);
        [q.x, q.y]
    };
    let flat_dir = |d: Vec3| v2(d.dot(frame.x), d.dot(frame.y));
    let (a, b) = (body.vertices[edge.v0], body.vertices[edge.v1]);
    let tiny = 1e-9 * body.aabb().diagonal().max(1.0);
    let gap = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]);
    // The ends are where the vertices are, so that they meet projected vertices exactly.
    let (start, end) = (flat(a), flat(b));
    // A circle or ellipse: center plus u cos t plus v sin t, from t0 to t1.
    let conic = |center: Vec3, u: Vec3, v: Vec3| -> Result<G> {
        let c = flat(center);
        let (pu, pv) = (flat_dir(u), flat_dir(v));
        // Its own axes: the longest and shortest of pu cos t + pv sin t, at t = t_major and a quarter turn on.
        let t_major = 0.5 * (2.0 * pu.dot(pv)).atan2(pu.dot(pu) - pv.dot(pv));
        let (s, k) = t_major.sin_cos();
        let major = pu * k + pv * s;
        let minor = pv * k - pu * s;
        let along = |t: f64| major.dot(major).sqrt() * (t - t_major).cos();
        let whole = edge.t1 - edge.t0 >= TAU - 1e-9;
        if minor.norm() <= tiny {
            // Seen edge on: a line over as far as it reaches either way.
            if major.norm() <= tiny {
                return Ok(G::Point { at: c });
            }
            let mut reach = vec![along(edge.t0), along(edge.t1)];
            for k in -2..=3 {
                let t = t_major + k as f64 * PI;
                if t > edge.t0 && t < edge.t1 {
                    reach.push(along(t));
                }
            }
            let (lo, hi) = reach.iter().fold((f64::MAX, f64::MIN), |(lo, hi), x| (lo.min(*x), hi.max(*x)));
            let d = major * (1.0 / major.norm());
            let (p, q) = ([c[0] + d.x * lo, c[1] + d.y * lo], [c[0] + d.x * hi, c[1] + d.y * hi]);
            return Ok(if gap(p, q) <= tiny { G::Point { at: p } } else { G::Line { start: p, end: q } });
        }
        // Counter-clockwise in the sketch as t grows, or the other way.
        let ccw = pu.x * pv.y - pu.y * pv.x > 0.0;
        let (from, to) = if ccw { (start, end) } else { (end, start) };
        if (major.norm() - minor.norm()).abs() <= 1e-9 * major.norm() {
            let radius = major.norm();
            return Ok(if whole { G::Circle { center: c, radius } } else { G::Arc { center: c, radius, start: from, end: to } });
        }
        let tip = [c[0] + major.x, c[1] + major.y];
        Ok(if whole { G::Ellipse { center: c, major: tip, minor: minor.norm() } } else { G::EllipticalArc { center: c, major: tip, minor: minor.norm(), start: from, end: to } })
    };
    match &edge.curve {
        Curve::Line { .. } => Ok(if gap(start, end) <= tiny { G::Point { at: start } } else { G::Line { start, end } }),
        Curve::Circle { frame: around, radius } => conic(around.origin, around.x * *radius, around.y * *radius),
        Curve::Ellipse { frame: around, major, minor } => conic(around.origin, around.x * *major, around.y * *minor),
        Curve::Spline(s) => {
            let (u0, u1) = s.domain();
            let part = if edge.t0 > u0 || edge.t1 < u1 { s.sub(edge.t0.max(u0), edge.t1.min(u1))? } else { (**s).clone() };
            let points: Vec<[f64; 2]> = part.points().iter().map(|p| flat(*p)).collect();
            if points.iter().all(|p| gap(*p, points[0]) <= tiny) {
                return Ok(G::Point { at: points[0] });
            }
            let shape = coreshape_sketch::SplineShape::new(part.degree(), part.knots().to_vec(), points, part.weights().to_vec())
                .map_err(|why| Error::InvalidInput(format!("its projection isn't a spline a sketch can hold: {why}")))?;
            Ok(G::Spline(Arc::new(shape)))
        }
        Curve::Sampled(_) => Err(Error::InvalidInput("it's an intersection curve the kernel holds as samples, with no exact form to project".into())),
    }
}

/// Indices into everything side by side, as a list for each body in scope that has any. Those outside it are dropped.
fn by_body(all: &[Held], scope: &[(BodyId, Arc<Body>)], picks: &[usize], size: impl Fn(&Body) -> usize) -> Vec<(usize, Vec<usize>)> {
    let mut out = Vec::new();
    let mut first = 0;
    for h in all {
        let n = size(&h.body);
        let mine: Vec<usize> = picks.iter().filter(|&&p| p >= first && p < first + n).map(|p| p - first).collect();
        if !mine.is_empty() {
            if let Some(at) = scope.iter().position(|(id, _)| *id == h.id) {
                out.push((at, mine));
            }
        }
        first += n;
    }
    out
}

/// A result as what stays under a body's name and any further pieces of it. The piece that keeps the name is the one with
/// the most faces from before this feature, and the larger of those if it comes to that.
fn pieces_of(made: Body, feature: u32) -> (Arc<Body>, Vec<Arc<Body>>) {
    if made.shells.len() < 2 {
        return (Arc::new(made), Vec::new());
    }
    let mut solids = made.solids();
    if solids.len() < 2 {
        return (Arc::new(solids.remove(0)), Vec::new());
    }
    let older = |b: &Body| b.faces.iter().filter(|f| f.tag.feature != feature && !f.origin.copies.iter().any(|c| c.feature == feature)).count();
    let volumes: Vec<f64> = solids.iter().map(|b| coreshape::mass_properties(b).volume).collect();
    let keep = (0..solids.len()).max_by(|&a, &b| older(&solids[a]).cmp(&older(&solids[b])).then(volumes[a].total_cmp(&volumes[b]))).unwrap_or(0);
    let first = solids.remove(keep);
    (Arc::new(first), solids.into_iter().map(Arc::new).collect())
}

/// Says which body a failure was on, when there are several it could have been.
fn on_body(e: Error, id: BodyId, several: bool) -> Error {
    if several {
        Error::InvalidInput(format!("on body {}: {e}", id.text()))
    } else {
        e
    }
}

/// What a feature did to bodies.
#[derive(Default)]
struct Changes {
    changed: Vec<(BodyId, Option<Arc<Body>>)>,
    added: Vec<Arc<Body>>,
}

impl Changes {
    /// A body is now this. It may have gone altogether, or come apart.
    fn settle(&mut self, id: BodyId, made: Body, feature: u32) {
        if made.faces.is_empty() {
            self.changed.push((id, None));
        } else {
            let (first, rest) = pieces_of(made, feature);
            self.changed.push((id, Some(first)));
            self.added.extend(rest);
        }
    }

    fn done(self, tools: Option<(Arc<Vec<Body>>, Mode)>) -> RunResult {
        RunResult { changed: self.changed, added: self.added, sketch: None, tools, plane: None }
    }
}

fn boxes_meet(a: &Body, b: &Body) -> bool {
    let (a, b) = (a.aabb().inflated(4.0 * TOL), b.aabb());
    a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z
}

/// New solids combined with the bodies in scope, as a mode says. A body the solids don't reach is left alone. With no
/// bodies yet, adding is all that makes sense, and what it makes are the first bodies.
fn combined(scope: &[(BodyId, Arc<Body>)], tools: Vec<Body>, mode: Mode, feature: u32) -> Result<Changes> {
    if tools.is_empty() {
        return Err(Error::InvalidInput("the sketch has no closed regions".into()));
    }
    let mut out = Changes::default();
    if mode == Mode::NewBody || (mode == Mode::Add && scope.is_empty()) {
        out.added = merged_tools(&tools)?.solids().into_iter().map(Arc::new).collect();
        return Ok(out);
    }
    if scope.is_empty() {
        return Err(Error::InvalidInput("there is no body yet to work on".into()));
    }
    match mode {
        Mode::Add if scope.len() == 1 => {
            let (id, current) = &scope[0];
            out.settle(*id, boolean_many(current, &tools, BoolOp::Union)?, feature);
        }
        Mode::Add => {
            // Each new solid joins whatever it touches, and those all become one, under the first one's name.
            let mut current: Vec<Option<Arc<Body>>> = scope.iter().map(|(_, b)| Some(b.clone())).collect();
            let mut touched = vec![false; scope.len()];
            for solid in merged_tools(&tools)?.solids() {
                let mut piece = solid;
                let mut home = None;
                for i in 0..current.len() {
                    let Some(body) = current[i].clone() else { continue };
                    if !boxes_meet(&body, &piece) {
                        continue;
                    }
                    let made = union(&body, &piece)?;
                    if made.solid_count() > 1 {
                        continue;
                    }
                    piece = made;
                    touched[i] = true;
                    if home.is_none() {
                        home = Some(i);
                    } else {
                        current[i] = None;
                    }
                }
                match home {
                    Some(i) => current[i] = Some(Arc::new(piece)),
                    None => out.added.push(Arc::new(piece)),
                }
            }
            for (i, (id, _)) in scope.iter().enumerate() {
                if touched[i] {
                    out.changed.push((*id, current[i].clone()));
                }
            }
        }
        Mode::Remove | Mode::Intersect => {
            let op = if mode == Mode::Remove { BoolOp::Subtract } else { BoolOp::Intersect };
            for (id, body) in scope {
                let near: Vec<Body> = tools.iter().filter(|t| boxes_meet(body, t)).cloned().collect();
                if near.is_empty() {
                    continue;
                }
                let made = boolean_many(body, &near, op)?;
                // A body that's outside what it's intersected with is one this didn't reach.
                if mode == Mode::Intersect && made.faces.is_empty() {
                    continue;
                }
                out.settle(*id, made, feature);
            }
        }
        Mode::NewBody => unreachable!("handled above"),
        Mode::Surface => unreachable!("a surface doesn't combine with anything"),
    }
    Ok(out)
}

/// What's left of a sheet after a cut: the part outside, and the part inside, as asked.
fn trim_parts(
    sheet: &Body,
    tool: &TrimTool,
    keep: TrimKeep,
    all: &[Held],
    plane_at: &dyn Fn(&PlaneRef) -> Result<Frame>,
    sketch_output: &dyn Fn(&FeatureId) -> Result<SketchOutput>,
) -> Result<(Option<Body>, Option<Body>)> {
    let (want_out, want_in) = (keep != TrimKeep::Inside, keep != TrimKeep::Outside);
    if !sheet.sheet {
        return Err(Error::InvalidInput("a trim cuts a sheet, and this is a solid. A Boolean cuts a solid".into()));
    }
    match tool {
        TrimTool::Body(id) => {
            let solid = all.iter().find(|h| h.id == *id).ok_or_else(|| Error::InvalidInput(format!("it cuts with body {}, which isn't there at this point", id.text())))?;
            let outside = want_out.then(|| trim_sheet(sheet, &solid.body, Keep::Outside)).transpose()?;
            let inside = want_in.then(|| trim_sheet(sheet, &solid.body, Keep::Inside)).transpose()?;
            Ok((outside, inside))
        }
        TrimTool::Plane(plane) => {
            let f = plane_at(plane)?;
            let (p, n) = (f.origin, f.z);
            let outside = want_out.then(|| trim_sheet_by_plane(sheet, p, n, Keep::Outside)).transpose()?;
            let inside = want_in.then(|| trim_sheet_by_plane(sheet, p, n, Keep::Inside)).transpose()?;
            Ok((outside, inside))
        }
        TrimTool::Sketch(id) => {
            let out = sketch_output(id)?;
            let (frame, profiles) = (out.frame, out.closed(*id, "cutting with a sketch")?);
            // Each region swept far enough to go right through the sheet both ways.
            let bounds = sheet.aabb();
            let reach = bounds.diagonal() + (bounds.center() - frame.origin).norm() + 10.0;
            let below = Frame { origin: frame.origin - frame.z * reach, ..frame };
            let cutters = profiles.iter().map(|p| extrude(p, &below, 2.0 * reach, 0)).collect::<Result<Vec<Body>>>()?;
            let outside = if want_out {
                let mut left = sheet.clone();
                for c in &cutters {
                    if !left.faces.is_empty() {
                        left = trim_sheet(&left, c, Keep::Outside)?;
                    }
                }
                Some(left)
            } else {
                None
            };
            let inside = if want_in {
                let parts = cutters.iter().map(|c| trim_sheet(sheet, c, Keep::Inside)).collect::<Result<Vec<Body>>>()?;
                let list: Vec<&Body> = parts.iter().collect();
                let mut joined = Body::side_by_side(&list);
                joined.sheet = true;
                Some(joined)
            } else {
                None
            };
            Ok((outside, inside))
        }
    }
}

/// Profiles and a frame for revolving about a line in a sketch's plane, in the terms the kernel's revolve takes: distance
/// from its frame's Z axis, and height along it. That's the same plane turned, with the profile on the +x side, so the
/// axis has to point whichever way leaves it there. Going the other way round needs starting from the far end.
/// Profiles drawn in `from`'s plane, in `to`'s coordinates. The planes have to be the same, facing either way.
fn into_plane(list: &[Profile], from: &Frame, to: &Frame) -> std::result::Result<Vec<Profile>, String> {
    let along = from.z.dot(to.z);
    let gap = (from.origin - to.origin).dot(to.z);
    let size = list.iter().flat_map(|p| p.outer.segs.iter().map(|s| s.start())).fold(1.0f64, |m, q| m.max(q.x.abs()).max(q.y.abs()));
    if (along.abs() - 1.0).abs() > 1e-9 || gap.abs() > 1e-9 * (size + (from.origin - to.origin).norm()) {
        return Err(format!("isn't in the plane ({:.3} degrees and {gap:.3e} off it)", along.abs().min(1.0).acos().to_degrees()));
    }
    let o = to.to_local(from.origin);
    let m = [[from.x.dot(to.x), from.y.dot(to.x)], [from.x.dot(to.y), from.y.dot(to.y)]];
    Ok(list.iter().map(|p| coreshape::multi::mapped_profile(p, m, v2(o.x, o.y))).collect())
}

fn revolve_frame(axis: &RevolveAxis, frame: &Frame, list: &[Profile], angle: f64, env: &Env) -> Result<(Frame, Vec<Profile>)> {
    let (point, along) = (v2(num(&axis.point[0], env)?, num(&axis.point[1], env)?), v2(num(&axis.direction[0], env)?, num(&axis.direction[1], env)?));
    let length = along.x.hypot(along.y);
    if !(length >= 1e-12 && length <= 1e150) {
        return Err(Error::InvalidInput("a revolve's axis must have a sensible, non-zero direction".into()));
    }
    let d = v2(along.x / length, along.y / length);
    // Each side of it, by distance along its left.
    let (mut lowest, mut highest) = (f64::INFINITY, f64::NEG_INFINITY);
    for profile in list {
        for wire in std::iter::once(&profile.outer).chain(&profile.holes) {
            for seg in &wire.segs {
                for i in 0..=32 {
                    let q = seg.at(i as f64 / 32.0);
                    let side = -(q.x - point.x) * d.y + (q.y - point.y) * d.x;
                    lowest = lowest.min(side);
                    highest = highest.max(side);
                }
            }
        }
    }
    if lowest < -TOL && highest > TOL {
        return Err(Error::InvalidProfile("profile crosses the revolve axis".into()));
    }
    let on_left = highest > TOL;
    let up = if on_left { v2(-d.x, -d.y) } else { d };
    let turn = up.x.atan2(up.y);
    let (s, c) = turn.sin_cos();
    let by = v2(-(c * point.x - s * point.y), -(s * point.x + c * point.y));
    let moved: Vec<Profile> = list
        .iter()
        .map(|p| Profile { outer: p.outer.turned_and_moved(turn, by), holes: p.holes.iter().map(|h| h.turned_and_moved(turn, by)).collect() })
        .collect();
    let z = frame.x * up.x + frame.y * up.y;
    let x = frame.x * up.y - frame.y * up.x;
    let origin = frame.origin + frame.x * point.x + frame.y * point.y;
    let full = (angle - TAU).abs() < 1e-9;
    let start = if on_left && !full { -angle } else { 0.0 };
    let (ss, cs) = start.sin_cos();
    Ok((Frame::new(origin, z, x * cs + z.cross(x) * ss), moved))
}

/// What a rebuild hands back.
fn finished(steps: Vec<Step>, held: &[Held], failed: Option<usize>, planes: Vec<PlaneAt>) -> Outcome {
    let (mut first_face, mut first_edge) = (0, 0);
    let bodies: Vec<NamedBody> = held
        .iter()
        .map(|h| {
            let named = NamedBody { id: h.id, body: h.body.clone(), first_face, first_edge };
            first_face += h.body.faces.len();
            first_edge += h.body.edges.len();
            named
        })
        .collect();
    let body = match held {
        [] => None,
        [only] => Some(only.body.clone()),
        many => {
            let list: Vec<&Body> = many.iter().map(|h| h.body.as_ref()).collect();
            Some(Arc::new(Body::side_by_side(&list)))
        }
    };
    Outcome { steps, body, bodies, failed, planes }
}

/// What a sketch hands to the features that use it.
#[derive(Clone)]
struct SketchOutput {
    frame: Frame,
    profiles: Arc<Vec<Profile>>,
    /// Faces are named by entity, which is unique across the sketch, so regions need no offset.
    by_entity: bool,
    /// The solved sketch, if it's a constrained one, for picking any of its regions.
    solved: Option<Arc<coreshape_sketch::Sketch>>,
    /// Chains of curves that close nothing, for a surface to sweep.
    open: Arc<Vec<Wire>>,
    /// Why it has no closed regions, when it has none. Such a sketch still builds, so that its points and lines can be
    /// named, and only what needs a region from it fails.
    no_regions: Option<String>,
}

impl SketchOutput {
    /// Its closed regions, for a feature that needs one. With none, an error saying so, and why.
    fn closed(&self, id: FeatureId, what: &str) -> Result<Arc<Vec<Profile>>> {
        if self.profiles.is_empty() {
            let why = self.no_regions.as_deref().unwrap_or("it has none");
            return Err(Error::InvalidInput(format!("{what} needs a closed region, and the sketch f{} has none: {why}", id.0)));
        }
        Ok(self.profiles.clone())
    }
}

/// New sheets as what a feature made: a body for each connected piece of each, which a pattern can copy.
fn sheets(made: Vec<Body>) -> Result<RunResult> {
    if made.is_empty() {
        return Err(Error::InvalidInput("the sketch has no closed regions, and no open curves".into()));
    }
    let pieces: Vec<Body> = made.iter().flat_map(Body::solids).collect();
    let added = pieces.iter().map(|b| Arc::new(b.clone())).collect();
    Ok(RunResult { changed: Vec::new(), added, sketch: None, tools: Some((Arc::new(pieces), Mode::NewBody)), plane: None })
}

/// What regions and open curves made, each kept apart by its place in the sketch, as regions' faces are when they're not named
/// by entity. An open curve comes after every region.
fn named_by_region(closed: Vec<Body>, curves: Vec<Body>, by_entity: bool) -> Vec<Body> {
    let first_curve = closed.len() as u32;
    let closed = closed.into_iter().enumerate().map(|(k, b)| with_region(b, if by_entity { 0 } else { k as u32 }));
    let curves = curves.into_iter().enumerate().map(|(k, b)| with_region(b, if by_entity { 0 } else { first_curve + k as u32 }));
    closed.chain(curves).collect()
}

/// Refuses what was picked if it's all on sheets, which a feature needing a solid has nothing to do with. Picking everything
/// takes in sheets too, and simply leaves them out, as long as there's a solid among it.
fn only_on_sheets(all: &[Held], picks: &[usize], size: impl Fn(&Body) -> usize, what: &str) -> Result<()> {
    let mut first = 0;
    let mut on_sheet = None;
    for h in all {
        let n = size(&h.body);
        if picks.iter().any(|&p| p >= first && p < first + n) {
            if !h.body.sheet {
                return Ok(());
            }
            on_sheet = on_sheet.or(Some(h.id));
        }
        first += n;
    }
    match on_sheet {
        Some(id) => Err(Error::InvalidInput(format!("{what} needs a solid, and what it picks is on {}, which is a sheet", id.text()))),
        None => Ok(()),
    }
}

/// Chains of a sketch's curves that close nothing, each running the way its first curve does. A chain stops where curves
/// branch, so that each of its joins has just two faces.
fn open_chains(found: &coreshape_sketch::Arrangement) -> Vec<Wire> {
    use coreshape_sketch::{Seg as Piece, WhyIgnored};
    const JOIN: f64 = 1e-6;
    let close = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) <= JOIN;
    let mut left: Vec<Piece> = found.ignored.iter().filter(|i| matches!(i.why, WhyIgnored::FreeEnd { .. })).flat_map(|i| i.pieces.iter().cloned()).collect();
    let everything = left.clone();
    let meeting = |p: [f64; 2]| everything.iter().filter(|s| close(s.start(), p) || close(s.end(), p)).count();
    let mut out = Vec::new();
    while !left.is_empty() {
        let mut chain = vec![left.remove(0)];
        loop {
            let end = chain[chain.len() - 1].end();
            if meeting(end) != 2 {
                break;
            }
            let Some(k) = left.iter().position(|s| close(s.start(), end) || close(s.end(), end)) else { break };
            let next = left.remove(k);
            chain.push(if close(next.start(), end) { next } else { next.reversed() });
        }
        loop {
            let start = chain[0].start();
            if meeting(start) != 2 {
                break;
            }
            let Some(k) = left.iter().position(|s| close(s.start(), start) || close(s.end(), start)) else { break };
            let next = left.remove(k);
            chain.insert(0, if close(next.end(), start) { next } else { next.reversed() });
        }
        out.push(wire_from_segs(&chain, false));
    }
    out
}

// ----- The document -----

/// Parameters and features. Everything saved is in here, and results are cached beside it.
#[derive(Clone, Default)]
pub struct Document {
    params: Vec<Parameter>,
    features: Vec<Feature>,
    next_id: u32,
    /// Whatever an app wants kept with the document. It changes nothing in the model.
    app: Option<Value>,
    cache: HashMap<FeatureId, Cached>,
}

/// A sheet's thickness, bend radius (default the thickness) and k-factor (default 0.45), checked.
fn sheet_sizes(thickness: &Expr, bend_radius: Option<&Expr>, k_factor: Option<&Expr>, env: &Env) -> Result<(f64, f64, f64)> {
    let t = num(thickness, env)?;
    if !(t >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("a sheet's thickness has to be more than 0, not {t}")));
    }
    let r = bend_radius.map_or(Ok(t), |e| num(e, env))?;
    if !(r >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("a bend radius has to be more than 0, not {r}")));
    }
    let k = k_factor.map_or(Ok(0.45), |e| num(e, env))?;
    if !(0.0..=1.0).contains(&k) {
        return Err(Error::InvalidInput(format!("a k-factor is from 0 to 1, not {k}")));
    }
    Ok((t, r, k))
}

/// A sheet metal part's sizes and the defaults later features take, from the feature that made it.
#[derive(Clone, Copy, Debug)]
struct Sheet {
    t: f64,
    r: f64,
    k: f64,
    gap: f64,
    corner: Corner,
    feature: FeatureId,
}

/// The sizes and defaults a sheet metal feature sets, checked.
fn sheet_settings(op: &Op, feature: FeatureId, env: &Env) -> Result<Sheet> {
    let Op::SheetMetal { thickness, bend_radius, k_factor, minimal_gap, corner, .. } = op else {
        return Err(Error::InvalidInput("this isn't a sheet metal feature".into()));
    };
    let (t, r, k) = sheet_sizes(thickness, bend_radius.as_ref(), k_factor.as_ref(), env)?;
    let gap = minimal_gap.as_ref().map_or(Ok(t / 10.0), |e| num(e, env))?;
    if !(gap >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("a minimal gap has to be more than 0, not {gap}")));
    }
    Ok(Sheet { t, r, k, gap, corner: *corner, feature })
}

/// Which fields a hem of each kind takes, as a reader and a rebuild both check.
fn check_hem_fields(kind: HemKind, length: bool, gap: bool, bend_radius: bool, degrees: bool) -> Result<()> {
    let name = kind.word();
    let (wants_length, takes_gap, takes_radius, takes_degrees) = match kind {
        HemKind::Straight => (true, true, false, false),
        HemKind::Rounded => (true, false, true, false),
        HemKind::Teardrop => (true, false, true, true),
        HemKind::Rolled => (false, false, true, true),
    };
    for (has, takes, field) in [(length, wants_length, "length"), (gap, takes_gap, "gap"), (bend_radius, takes_radius, "bend_radius"), (degrees, takes_degrees, "degrees")] {
        if has && !takes {
            return Err(Error::InvalidInput(format!("a {name} hem has no \"{field}\"")));
        }
    }
    if wants_length && !length {
        return Err(Error::InvalidInput(format!("a {name} hem needs a \"length\"")));
    }
    Ok(())
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

    /// Every closed region a constrained sketch feature makes at its solved position, and the curves
    /// that make none. `Op::ConstrainedSketch` builds from the picked regions, or by default from every
    /// other level of nesting, and leaves the rest out without failing, so this is where to look.
    pub fn sketch_arrangement(&self, id: FeatureId) -> Result<coreshape_sketch::Arrangement> {
        let Some(Op::ConstrainedSketch { sketch, .. }) = self.feature(id).map(|f| &f.op) else {
            return Err(Error::InvalidInput(format!("f{} isn't a constrained sketch", id.0)));
        };
        let env = self.environment()?;
        let solved = self.as_last_placed(id, sketch).solve_with(&env).map_err(|e| Error::InvalidProfile(e.message))?;
        Ok(solved.sketch.arrangement())
    }

    /// A sketch with its projected entities where the last rebuild put them, if it got that far.
    fn as_last_placed(&self, id: FeatureId, sketch: &coreshape_sketch::Sketch) -> coreshape_sketch::Sketch {
        let mut out = sketch.clone();
        let last = self.cache.get(&id).and_then(|c| c.sketch.as_ref()).and_then(|s| s.solved.clone());
        if let Some(last) = last {
            for e in sketch.projected() {
                if let Some(there) = last.entity(e.id).filter(|t| t.projection.as_ref().is_some_and(|p| p.placed)) {
                    let _ = out.place_projected(e.id, there.geometry.clone());
                }
            }
        }
        out
    }

    /// A sketch with its projected entities put where the parts put them now, as the sketch tools need it to solve: on
    /// `plane`, and at `position` in the history, which is where the sketch is or will go. The parts are those the
    /// features before that position make.
    pub fn sketch_in_context(&mut self, sketch: &coreshape_sketch::Sketch, plane: &Placement, position: usize) -> Result<coreshape_sketch::Sketch> {
        let env = self.environment()?;
        let held: Vec<Held> = if position == 0 {
            Vec::new()
        } else {
            let o = self.rebuild(Some(position - 1));
            o.bodies.iter().map(|b| Held { id: b.id, body: b.body.clone(), key: 0 }).collect()
        };
        let frame = Looker::new(self, &env, &held).placement(plane)?;
        placed_sketch(sketch, &frame, &held, &env)
    }

    /// Where a plane feature's plane is, or a sketch's, as the last rebuild that reached it left it.
    pub fn frame_of(&self, id: FeatureId) -> Option<Frame> {
        let c = self.cache.get(&id)?;
        c.plane.or_else(|| c.sketch.as_ref().map(|s| s.frame))
    }

    /// A plane feature's plane, for what names it.
    fn plane_output(&self, id: FeatureId) -> Result<Frame> {
        if self.feature(id).is_some_and(|f| f.suppressed) {
            return Err(Error::InvalidInput(format!("plane f{} is suppressed", id.0)));
        }
        self.cache.get(&id).and_then(|c| c.plane).ok_or_else(|| Error::InvalidInput(format!("plane f{} did not build", id.0)))
    }

    /// What a sketch hands on, for what uses it.
    fn sketch_output(&self, id: FeatureId) -> Result<SketchOutput> {
        if self.feature(id).is_some_and(|f| f.suppressed) {
            return Err(Error::InvalidInput(format!("its sketch f{} is suppressed", id.0)));
        }
        self.cache.get(&id).and_then(|c| c.sketch.clone()).ok_or_else(|| Error::InvalidInput(format!("its sketch f{} did not build", id.0)))
    }

    /// A constrained sketch as solved, with its frame, for picking its points and lines.
    fn solved_sketch(&self, id: FeatureId) -> Result<(Frame, Arc<coreshape_sketch::Sketch>)> {
        let out = self.sketch_output(id)?;
        let solved = out.solved.ok_or_else(|| Error::InvalidInput(format!("points and lines are picked from a constrained sketch, and f{} is a sketch of shapes", id.0)))?;
        Ok((out.frame, solved))
    }

    /// Where everything a feature places things by now is, as text for its key, so that it's rebuilt when any of it moves.
    fn resolved_refs(&self, op: &Op, env: &Env, held: &[Held]) -> Result<String> {
        let geo = op.geo();
        if geo.is_empty() {
            return Ok(String::new());
        }
        let look = Looker::new(self, env, held);
        let mut out = String::new();
        for g in geo {
            let text = match g {
                Geo::Place(p) => format!("{:?}", look.placement(p)?),
                Geo::Plane(p) => format!("{:?}", look.plane(p)?),
                Geo::Line(l) => format!("{:?}", look.line(l)?),
                Geo::Def(d, flip) => format!("{:?}", look.def(d, flip)?),
            };
            out.push(' ');
            out.push_str(&text);
        }
        Ok(out)
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
        self.features.insert(at, Feature { id, name, suppressed: false, scope: Vec::new(), app: None, op });
        Ok(id)
    }

    /// Chooses the bodies a feature acts on. Empty means all of them.
    pub fn set_scope(&mut self, id: FeatureId, scope: Vec<BodyId>) -> Result<()> {
        let at = self.position(id)?;
        if !scope.is_empty() && self.features[at].op.is_reference() {
            return Err(Error::InvalidInput(format!("a {} doesn't act on a body", self.features[at].op.label().to_lowercase())));
        }
        for b in &scope {
            if !self.features[..at].iter().any(|f| f.id == b.feature) {
                return Err(Error::InvalidInput(format!("this feature acts on body {}, whose feature doesn't come before it", b.text())));
            }
        }
        self.features[at].scope = scope;
        Ok(())
    }

    /// Keeps something for an app with a feature, or with the document if no feature is given. It changes nothing in the model.
    pub fn set_app(&mut self, id: Option<FeatureId>, app: Option<Value>) -> Result<()> {
        match id {
            Some(id) => {
                let at = self.position(id)?;
                self.features[at].app = app;
            }
            None => self.app = app,
        }
        Ok(())
    }

    /// What an app has kept with the document.
    pub fn app(&self) -> Option<&Value> {
        self.app.as_ref()
    }

    /// Adds a feature, panicking on a bad reference. For models written in code, where a
    /// bad reference is a programming error.
    pub fn with(&mut self, op: Op) -> FeatureId {
        self.add(op).expect("feature references must already exist")
    }

    fn check_sketch_refs(op: &Op, before: &[Feature]) -> Result<()> {
        let list: Vec<FeatureId> = match op {
            Op::Extrude { sketch, regions, .. } => sketch.iter().copied().chain(regions.iter().map(|r| r.sketch)).collect(),
            Op::Revolve { sketch, regions, .. } => std::iter::once(*sketch).chain(regions.iter().map(|r| r.sketch)).collect(),
            Op::Sweep { sketch, path: SweepPath::Sketch(along), .. } => vec![*sketch, *along],
            Op::Sweep { sketch, .. }
            | Op::Helix { sketch, .. }
            | Op::Fill { sketch }
            | Op::Trim { tool: TrimTool::Sketch(sketch), .. }
            | Op::SheetMetal { sketch, .. }
            | Op::Tab { sketch }
            | Op::Bend { sketch, .. } => vec![*sketch],
            Op::Loft { sections, .. } => sections.iter().map(|s| s.sketch).collect(),
            _ => Vec::new(),
        };
        for sketch in &list {
            if let Some(f) = before.iter().find(|f| f.id == *sketch) {
                if !f.op.is_sketch() {
                    return Err(Error::InvalidInput(format!("f{} is a {}, not a sketch", sketch.0, f.op.kind())));
                }
            }
        }
        let (mut planes, mut sketches) = (Vec::new(), Vec::new());
        for g in op.geo() {
            g.planes(&mut planes);
            g.sketches(&mut sketches);
        }
        for id in planes {
            if let Some(f) = before.iter().find(|f| f.id == id) {
                if !matches!(f.op, Op::Plane { .. }) {
                    return Err(Error::InvalidInput(format!("f{} is a {}, not a plane", id.0, f.op.kind())));
                }
            }
        }
        for id in sketches {
            if let Some(f) = before.iter().find(|f| f.id == id) {
                if !matches!(f.op, Op::ConstrainedSketch { .. }) {
                    return Err(Error::InvalidInput(format!("points and lines are picked from a constrained sketch, and f{} is a {}", id.0, f.op.kind())));
                }
            }
        }
        Ok(())
    }

    /// The sheet metal feature that made a part, the first if several did, with its sizes and defaults.
    fn sheet_of(&self, body: &Body, env: &Env) -> Result<Sheet> {
        let mut first: Option<usize> = None;
        for face in &body.faces {
            if let Some(at) = self.features.iter().position(|f| f.id.0 == face.tag.feature && matches!(f.op, Op::SheetMetal { .. })) {
                first = Some(first.map_or(at, |was| was.min(at)));
            }
        }
        let feature = &self.features[first.ok_or_else(|| Error::InvalidInput("this part isn't sheet metal: no sheet metal feature made it".into()))?];
        sheet_settings(&feature.op, feature.id, env)
    }

    /// A sheet metal part laid out flat, as `outcome` has it: the one named, or the only sheet metal part there is.
    /// It's unfolded from the first wall's face on the sketch's side, which stays where it is.
    pub fn flat_pattern(&self, outcome: &Outcome, body: Option<BodyId>) -> Result<FlatPattern> {
        let env = self.environment()?;
        let named = match body {
            Some(id) => outcome.find_body(id).ok_or_else(|| Error::InvalidInput(format!("there's no body {}", id.text())))?,
            None => {
                let sheet: Vec<&NamedBody> = outcome.bodies.iter().filter(|b| self.sheet_of(&b.body, &env).is_ok()).collect();
                match sheet.as_slice() {
                    [only] => *only,
                    [] => return Err(Error::InvalidInput("there's no sheet metal part to lay flat".into())),
                    many => {
                        let names: Vec<String> = many.iter().map(|b| b.id.text()).collect();
                        return Err(Error::InvalidInput(format!("there are several sheet metal parts, so say which: {}", names.join(", "))));
                    }
                }
            }
        };
        let Sheet { t, k, feature: id, .. } = self.sheet_of(&named.body, &env)?;
        let Some(Op::SheetMetal { sketch, .. }) = self.feature(id).map(|f| &f.op) else { unreachable!("found as sheet metal") };
        // A plate starts from the face on its sketch, a chain from its first wall's face on the lines' side.
        let plate = !self.sketch_output(*sketch)?.profiles.is_empty();
        let start = FaceRef { feature: id, slot: if plate { SLOT_START_CAP } else { SLOT_SEG_BASE } };
        let seed = named.body.faces.iter().position(|f| f.tag == start.tag()).ok_or_else(|| {
            Error::InvalidInput(format!("the face this part is laid flat from, {}, has been cut away", start.name()))
        })?;
        sheet_metal::flat_pattern(&named.body, seed, t, k, id.0)
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
        self.features.iter().filter(|f| f.references().contains(&id)).map(|f| f.id).collect()
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
            for r in f.references() {
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
        let mut held: Vec<Held> = Vec::new();
        let mut failed: Option<usize> = None;
        let mut planes: Vec<PlaneAt> = Vec::new();

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
                return finished(steps, &held, failed, Vec::new());
            }
        };

        // Each feature's key, for what depends on it.
        let mut keys: HashMap<FeatureId, u64> = HashMap::new();
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
            let mut own = match Self::signature(&feature.op, &env) {
                Ok(s) => format!("{s} on {:?}", feature.scope),
                Err(e) => {
                    failed = Some(i);
                    steps.push(step(StepState::Failed(e.to_string())));
                    continue;
                }
            };
            // Following a face or a plane, or naming planes, points or lines, it depends on where they've got to, and nothing
            // else about what they're on.
            match self.resolved_refs(&feature.op, &env, &held) {
                Ok(text) => own.push_str(&text),
                Err(e) => {
                    failed = Some(i);
                    steps.push(step(StepState::Failed(e.to_string())));
                    continue;
                }
            }
            // Geometry projected from parts depends on where those parts are now, and on nothing else about the bodies.
            if let Op::ConstrainedSketch { plane, sketch } = &feature.op {
                if sketch.projected().next().is_some() {
                    match Looker::new(self, &env, &held).placement(plane).and_then(|frame| placed_sketch(sketch, &frame, &held, &env)) {
                        Ok(placed) => {
                            for e in placed.projected() {
                                own.push_str(&format!(" {:?}", e.geometry));
                            }
                        }
                        Err(e) => {
                            failed = Some(i);
                            steps.push(step(StepState::Failed(e.to_string())));
                            continue;
                        }
                    }
                }
            }
            // A constrained sketch's open curves are refused unless a surface is what they're for.
            if matches!(feature.op, Op::ConstrainedSketch { .. }) && self.features.iter().any(|f| f.op.uses_open_curves_of(feature.id)) {
                own.push_str(" for a surface");
            }
            // The bodies it acts on: those it names, or all of them. One that isn't there is for it to say so.
            let missing = feature.scope.iter().find(|id| !held.iter().any(|h| h.id == **id));
            if let (Some(id), false) = (missing, feature.op.is_reference()) {
                failed = Some(i);
                steps.push(step(StepState::Failed(format!("it acts on body {}, which isn't there at this point", id.text()))));
                continue;
            }
            let scope: Vec<(BodyId, Arc<Body>)> = if feature.op.is_reference() {
                Vec::new()
            } else if feature.scope.is_empty() {
                // A sheet has no inside to cut or add to, and one lying around mustn't get in the way of what does.
                held.iter().filter(|h| feature.op.takes_sheets() || !h.body.sheet).map(|h| (h.id, h.body.clone())).collect()
            } else {
                feature.scope.iter().filter_map(|id| held.iter().find(|h| h.id == *id).map(|h| (h.id, h.body.clone()))).collect()
            };
            // A sketch depends only on its own inputs. Everything else also depends on the bodies it acts on, and on its
            // sketch or what it copies when it has them. A new body needs nothing from the others, unless it's made
            // from their faces.
            let makes_its_own = |f: &FeatureId| self.feature(*f).is_some_and(|g| matches!(g.op.mode(), Some(Mode::NewBody | Mode::Surface)));
            // So does one made from faces or edges, which can be on any part.
            let looks_around = matches!(&feature.op, Op::Extrude { end, faces, .. } if *end != ExtrudeEnd::Blind || !faces.is_empty())
                || matches!(&feature.op, Op::Sweep { path: SweepPath::Edges(_), .. });
            let independent = match &feature.op {
                Op::Thicken { .. } => false,
                _ if looks_around => false,
                Op::Pattern { of, .. } | Op::Mirror { of, .. } => !of.is_empty() && of.iter().all(makes_its_own),
                op => matches!(op.mode(), Some(Mode::NewBody | Mode::Surface)),
            };
            let key_of = |id: &BodyId| held.iter().find(|h| h.id == *id).map_or(0, |h| h.key);
            let mut body_key = 0u64;
            if !independent {
                // Where it ends can be on any part, in the scope or not.
                if looks_around {
                    for h in &held {
                        body_key ^= h.key;
                    }
                } else {
                    for (id, _) in &scope {
                        body_key ^= key_of(id);
                    }
                }
                if let Op::BooleanBodies { tools, .. } = &feature.op {
                    for id in tools {
                        body_key ^= key_of(id).rotate_left(29);
                    }
                }
                if let Op::Trim { tool: TrimTool::Body(id), .. } = &feature.op {
                    body_key ^= key_of(id).rotate_left(29);
                }
            }
            let mixed = |list: &[FeatureId]| list.iter().enumerate().fold(0u64, |sum, (k, id)| sum ^ keys.get(id).copied().unwrap_or(0).rotate_left(17 + k as u32));
            let key = match &feature.op {
                Op::Sketch { .. } | Op::ConstrainedSketch { .. } | Op::Plane { .. } => hash_text(1, &own),
                Op::Extrude { sketch, regions, .. } => {
                    let sketches: Vec<FeatureId> = sketch.iter().copied().chain(regions.iter().map(|r| r.sketch)).collect();
                    hash_text(body_key ^ mixed(&sketches), &own)
                }
                Op::Revolve { sketch, regions, .. } => {
                    let sketches: Vec<FeatureId> = std::iter::once(*sketch).chain(regions.iter().map(|r| r.sketch)).collect();
                    hash_text(body_key ^ mixed(&sketches), &own)
                }
                Op::Sweep { sketch, path: SweepPath::Sketch(along), .. } => hash_text(body_key ^ mixed(&[*sketch, *along]), &own),
                Op::Sweep { sketch, .. }
                | Op::Helix { sketch, .. }
                | Op::Fill { sketch }
                | Op::Trim { tool: TrimTool::Sketch(sketch), .. }
                | Op::SheetMetal { sketch, .. }
                | Op::Tab { sketch }
                | Op::Bend { sketch, .. } => hash_text(body_key ^ mixed(&[*sketch]), &own),
                Op::Loft { sections, .. } => hash_text(body_key ^ mixed(&sections.iter().map(|s| s.sketch).collect::<Vec<_>>()), &own),
                Op::Pattern { of, .. } | Op::Mirror { of, .. } => hash_text(body_key ^ mixed(of), &own),
                _ => hash_text(body_key, &own),
            };
            let cached = self.cache.get(&feature.id).filter(|c| c.key == key).cloned();
            if let Some(c) = cached {
                keys.insert(feature.id, key);
                if !feature.op.is_sketch() {
                    take_result(&mut held, feature.id, key, &c.changed, &c.added);
                }
                if let Some(frame) = c.plane {
                    planes.push(PlaneAt { id: feature.id, frame });
                }
                steps.push(step(StepState::Reused));
                continue;
            }
            match self.run(&feature, &env, &scope, &held) {
                Ok(result) => {
                    keys.insert(feature.id, key);
                    if result.sketch.is_none() {
                        take_result(&mut held, feature.id, key, &result.changed, &result.added);
                    }
                    if let Some(frame) = result.plane {
                        planes.push(PlaneAt { id: feature.id, frame });
                    }
                    self.cache.insert(feature.id, Cached { key, changed: result.changed, added: result.added, sketch: result.sketch, tools: result.tools, plane: result.plane });
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
        finished(steps, &held, failed, planes)
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

    fn run(&self, feature: &Feature, env: &Env, scope: &[(BodyId, Arc<Body>)], all: &[Held]) -> Result<RunResult> {
        let tag = feature.id.0;
        let sketched = |output: SketchOutput| RunResult { changed: Vec::new(), added: Vec::new(), sketch: Some(output), tools: None, plane: None };
        let applied = |tools: Vec<Body>, mode: Mode| -> Result<RunResult> {
            if mode == Mode::Surface {
                if matches!(feature.op, Op::Solid { .. } | Op::Thicken { .. }) {
                    return Err(Error::InvalidInput(format!("the mode surface is for what sweeps a sketch, and a {} doesn't", feature.op.kind())));
                }
                return sheets(tools.iter().map(sides_of).collect::<Result<Vec<Body>>>()?);
            }
            let done = combined(scope, tools.clone(), mode, tag)?;
            Ok(done.done(Some((Arc::new(tools), mode))))
        };
        // A feature that works on what's there, one body at a time.
        let each = |what: &str, work: &dyn Fn(&Body) -> Result<Body>| -> Result<RunResult> {
            if scope.is_empty() {
                return Err(Error::InvalidInput(format!("{what} needs a body to work on")));
            }
            let mut out = Changes::default();
            for (id, body) in scope {
                out.settle(*id, work(body).map_err(|e| on_body(e, *id, scope.len() > 1))?, tag);
            }
            Ok(out.done(None))
        };
        let whole = || -> Option<Arc<Body>> { whole_of(all) };
        let at_point = |p: &Point3| -> Result<Vec3> { Ok(v3(num(&p[0], env)?, num(&p[1], env)?, num(&p[2], env)?)) };
        let sketch_output = |id: &FeatureId| -> Result<SketchOutput> { self.sketch_output(*id) };
        let look = Looker::new(self, env, all);
        match &feature.op {
            Op::Sketch { plane, regions } => {
                let list = resolve_regions(regions, env)?;
                let open = resolve_open(regions, env)?;
                for p in &list {
                    p.validate()?;
                }
                let no_regions = list.is_empty().then(|| "the sketch has no closed regions".to_string());
                Ok(sketched(SketchOutput { frame: look.placement(plane)?, profiles: Arc::new(list), by_entity: false, solved: None, open: Arc::new(open), no_regions }))
            }
            Op::ConstrainedSketch { plane, sketch } => {
                let frame = look.placement(plane)?;
                let placed;
                let sketch = if sketch.projected().next().is_some() {
                    placed = placed_sketch(sketch, &frame, all, env)?;
                    &placed
                } else {
                    sketch
                };
                let solved = sketch.solve_with(env).map_err(|e| Error::InvalidProfile(e.message))?;
                // Curves that close nothing are refused, unless a surface is going to sweep them.
                let open = if self.features.iter().any(|f| f.op.uses_open_curves_of(feature.id)) { open_chains(&solved.sketch.arrangement()) } else { Vec::new() };
                for wire in &open {
                    wire.check_open()?;
                }
                let (regions, no_regions) = match solved.sketch.regions() {
                    Ok(regions) => (regions, None),
                    Err(e) if solved.sketch.picked_regions().is_empty() => (Vec::new(), Some(e.message)),
                    Err(e) => return Err(Error::InvalidProfile(e.message)),
                };
                let list = profiles_from_regions(&regions);
                for p in &list {
                    p.validate()?;
                }
                Ok(sketched(SketchOutput { frame, profiles: Arc::new(list), by_entity: true, solved: Some(Arc::new(solved.sketch)), open: Arc::new(open), no_regions }))
            }
            Op::Extrude { sketch, start, length, mode, regions, faces, end, opposite } => {
                // Each sketch it takes from, in the order they first come up.
                let mut sketches: Vec<FeatureId> = Vec::new();
                for id in regions.iter().map(|r| r.sketch) {
                    if !sketches.contains(&id) {
                        sketches.push(id);
                    }
                }
                if sketches.is_empty() {
                    sketches.extend(*sketch);
                }
                if sketches.is_empty() && faces.is_empty() {
                    return Err(Error::InvalidInput("an extrude needs a sketch, regions or faces".into()));
                }
                let surface = *mode == Mode::Surface;
                if surface && *end != ExtrudeEnd::Blind {
                    return Err(Error::InvalidInput("a surface is extruded to a length: through all and up to something are only built for solids".into()));
                }
                let plane = match end {
                    ExtrudeEnd::UpToPlane(p) => Some(look.plane(p)?),
                    _ => None,
                };
                let reach = up_to::Reach { all, scope, mode: *mode, env, tag, plane };
                let offset = num(start, env)?;
                // What one profile makes, going from a frame already moved by the start.
                let made_from = |p: &Profile, shifted: &Frame| -> Result<Vec<Body>> {
                    match end {
                        ExtrudeEnd::Blind => {
                            let h = num(length, env)?;
                            Ok(vec![extrude(p, shifted, if *opposite { -h } else { h }, tag)?])
                        }
                        other => up_to::tools_to(p, shifted, *opposite, other, &reach),
                    }
                };
                let mut tools = Vec::new();
                for (order, id) in sketches.iter().enumerate() {
                    let out = sketch_output(id)?;
                    let list = match (regions.is_empty(), surface) {
                        (true, true) => out.profiles.clone(),
                        (true, false) => out.closed(*id, "an extrude")?,
                        (false, _) => Arc::new(picked_profiles(*id, &out, regions)?),
                    };
                    let shifted = Frame { origin: out.frame.origin + out.frame.z * offset, ..out.frame };
                    if surface && regions.is_empty() {
                        // A whole sketch as a surface: its regions' walls, and its open curves after them.
                        let h = num(length, env)?;
                        let h = if *opposite { -h } else { h };
                        let closed = list.iter().map(|p| extrude(p, &shifted, h, tag).and_then(|b| sides_of(&b))).collect::<Result<Vec<Body>>>()?;
                        let curves = out.open.iter().map(|w| extrude_open(w, &shifted, h, tag)).collect::<Result<Vec<Body>>>()?;
                        tools.extend(named_by_region(closed, curves, out.by_entity));
                        continue;
                    }
                    for (k, p) in list.iter().enumerate() {
                        // Entities are unique within a sketch, so another sketch's faces need telling apart from these.
                        let region = if out.by_entity { order as u32 } else { k as u32 };
                        for b in made_from(p, &shifted)? {
                            tools.push(with_region(if surface { sides_of(&b)? } else { b }, region));
                        }
                    }
                }
                if !faces.is_empty() {
                    if faces.len() > MAX_EXTRUDED_FACES {
                        return Err(Error::InvalidInput(format!("an extrude can take at most {MAX_EXTRUDED_FACES} faces")));
                    }
                    let everything = whole().ok_or_else(|| Error::InvalidInput("extruding faces needs a part to take them from".into()))?;
                    let uses = everything.edge_uses();
                    let mut taken = vec![false; everything.faces.len()];
                    for (entry, key) in faces.iter().enumerate() {
                        let found = find_faces(&everything, key);
                        if found.is_empty() {
                            return Err(Error::InvalidInput(format!("the face {} no longer exists in the model", key.text())));
                        }
                        for f in found {
                            // A face two entries pick goes once, named for the first.
                            if std::mem::replace(&mut taken[f], true) {
                                continue;
                            }
                            let (frame, profile, coedges) = flat_face_profile(&everything, f).map_err(|e| Error::InvalidInput(format!("the face {} can't be extruded: {e}", key.text())))?;
                            let shifted = Frame { origin: frame.origin + frame.z * offset, ..frame };
                            for b in made_from(&profile, &shifted)? {
                                let b = named_for_face(b, tag, entry as u32, everything.faces[f].tag, |id| {
                                    let edge = coedges[id as usize].edge;
                                    uses[edge].iter().filter(|u| u.0 != f).map(|u| everything.faces[u.0].tag).collect()
                                });
                                tools.push(if surface { sides_of(&b)? } else { b });
                            }
                        }
                    }
                }
                if surface {
                    return sheets(tools);
                }
                applied(tools, *mode)
            }
            Op::Revolve { sketch, degrees, mode, axis, regions } => {
                let a = num(degrees, env)?.to_radians();
                let surface = *mode == Mode::Surface;
                // Each sketch it takes from, in the order they first come up.
                let mut sketches: Vec<FeatureId> = Vec::new();
                for id in regions.iter().map(|r| r.sketch) {
                    if !sketches.contains(&id) {
                        sketches.push(id);
                    }
                }
                if sketches.is_empty() {
                    sketches.push(*sketch);
                }
                // An axis is drawn in the revolve's own sketch.
                let own = sketch_output(sketch)?;
                let mut tools = Vec::new();
                for (order, id) in sketches.iter().enumerate() {
                    let out = sketch_output(id)?;
                    let list: Vec<Profile> = match (regions.is_empty(), surface) {
                        (true, true) => out.profiles.to_vec(),
                        (true, false) => out.closed(*id, "a revolve")?.to_vec(),
                        (false, _) => picked_profiles(*id, &out, regions)?,
                    };
                    // A whole sketch as a surface takes its open curves too, which go around as a region's outline would.
                    let whole_surface = surface && regions.is_empty();
                    let mut all_curves = list;
                    let closed_count = all_curves.len();
                    if whole_surface {
                        all_curves.extend(out.open.iter().map(|w| Profile { outer: w.clone(), holes: Vec::new() }));
                    }
                    let (frame, moved): (Frame, Vec<Profile>) = match axis {
                        Some(axis) => {
                            let here = if *id == *sketch { all_curves } else { into_plane(&all_curves, &out.frame, &own.frame).map_err(|why| Error::InvalidInput(format!("a region from sketch f{} {why} of f{}, whose axis it turns around", id.0, sketch.0)))? };
                            revolve_frame(axis, &own.frame, &here, a, env)?
                        }
                        None => (out.frame, all_curves),
                    };
                    let (closed, curves) = moved.split_at(closed_count);
                    if whole_surface {
                        let closed = closed.iter().map(|p| revolve(p, &frame, a, tag).and_then(|b| sides_of(&b))).collect::<Result<Vec<Body>>>()?;
                        let curves = curves.iter().map(|p| revolve_open(&p.outer, &frame, a, tag)).collect::<Result<Vec<Body>>>()?;
                        tools.extend(named_by_region(closed, curves, out.by_entity));
                        continue;
                    }
                    for (k, p) in closed.iter().enumerate() {
                        // Entities are unique within a sketch, so another sketch's faces need telling apart from these.
                        let region = if out.by_entity { order as u32 } else { k as u32 };
                        let made = revolve(p, &frame, a, tag)?;
                        tools.push(with_region(if surface { sides_of(&made)? } else { made }, region));
                    }
                }
                if surface {
                    return sheets(tools);
                }
                applied(tools, *mode)
            }
            Op::Loft { sections, mode } => {
                let mut list: Vec<(Profile, Frame)> = Vec::with_capacity(sections.len());
                for section in sections {
                    let out = sketch_output(&section.sketch)?;
                    let profile = match section.region {
                        Some(region) => picked_profiles(section.sketch, &out, &[RegionPick { sketch: section.sketch, region }])?.remove(0),
                        None => match out.closed(section.sketch, "a loft's section")?.as_slice() {
                            [one] => one.clone(),
                            many => {
                                return Err(Error::InvalidInput(format!(
                                    "a loft's section f{} must hold one outline, not {}, or name one of its regions: {{\"sketch\": {}, \"region\": \"r...\"}}",
                                    section.sketch.0,
                                    many.len(),
                                    section.sketch.0
                                )))
                            }
                        },
                    };
                    list.push((profile, out.frame));
                }
                let refs: Vec<(&Profile, Frame)> = list.iter().map(|(p, f)| (p, *f)).collect();
                applied(vec![coreshape::loft::loft(&refs, tag)?], *mode)
            }
            Op::Sweep { sketch, path, mode } => {
                let out = sketch_output(sketch)?;
                let list = if *mode == Mode::Surface { out.profiles.clone() } else { out.closed(*sketch, "a sweep")? };
                let SketchOutput { frame, by_entity, .. } = out;
                let along: Option<Vec<PathCurve>> = match path {
                    SweepPath::Steps(_) => None,
                    SweepPath::Sketch(id) => Some(path_from_sketch(*id, &sketch_output(id)?)?),
                    SweepPath::Edges(query) => {
                        let everything = whole().ok_or_else(|| Error::InvalidInput("a sweep along edges needs a part to take them from".into()))?;
                        Some(path_from_edges(&everything, &select_edges(query, &everything, env)?)?)
                    }
                };
                let no_steps = Vec::new();
                let steps = match path {
                    SweepPath::Steps(steps) => steps,
                    _ => &no_steps,
                };
                let steps = steps
                    .iter()
                    .map(|step| {
                        Ok(match step {
                            SweepStep::Line { length } => PathStep::Line { length: num(length, env)? },
                            SweepStep::Arc { radius, degrees, toward_degrees } => PathStep::Arc {
                                radius: num(radius, env)?,
                                angle: num(degrees, env)?.to_radians(),
                                toward: num(toward_degrees, env)?.to_radians(),
                            },
                        })
                    })
                    .collect::<Result<Vec<PathStep>>>()?;
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| {
                        let made = match &along {
                            Some(pieces) => sweep_along(p, &frame, pieces, tag)?,
                            None => sweep(p, &frame, &steps, tag)?,
                        };
                        Ok(with_region(made, if by_entity { 0 } else { k as u32 }))
                    })
                    .collect::<Result<Vec<Body>>>()?;
                applied(tools, *mode)
            }
            Op::Helix { sketch, point, axis, pitch, turns, left_handed, mode } => {
                let out = sketch_output(sketch)?;
                let list = if *mode == Mode::Surface { out.profiles.clone() } else { out.closed(*sketch, "a helix")? };
                let SketchOutput { frame, by_entity, .. } = out;
                let motion = Helix { origin: at_point(point)?, axis: at_point(axis)?, pitch: num(pitch, env)?, turns: num(turns, env)?, left_handed: *left_handed };
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| Ok(with_region(helix(p, &frame, &motion, tag)?, if by_entity { 0 } else { k as u32 })))
                    .collect::<Result<Vec<Body>>>()?;
                applied(tools, *mode)
            }
            Op::DeleteFace { faces } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("deleting faces needs a body to work on".into()))?;
                let picked = keyed_faces(&everything, faces)?;
                only_on_sheets(all, &picked, |b| b.faces.len(), "deleting a face")?;
                let mut out = Changes::default();
                for (at, list) in by_body(all, scope, &picked, |b| b.faces.len()) {
                    out.settle(scope[at].0, delete_faces(&scope[at].1, &list).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::Thicken { faces, thickness, mode } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("thickening needs a body to take faces from".into()))?;
                let t = num(thickness, env)?;
                let mut tools = Vec::new();
                if faces.is_empty() && !feature.scope.is_empty() {
                    // The whole of each sheet it names.
                    for (_, body) in scope {
                        if !body.sheet {
                            return Err(Error::InvalidInput("thickening has to name the faces of a solid. It can only take all the faces of a sheet".into()));
                        }
                        let list: Vec<usize> = (0..body.faces.len()).collect();
                        tools.push(thicken(body, &list, t, tag)?);
                    }
                    return applied(tools, *mode);
                }
                let picked = keyed_faces(&everything, faces)?;
                for (at, list) in by_body(all, scope, &picked, |b| b.faces.len()) {
                    tools.push(thicken(&scope[at].1, &list, t, tag)?);
                }
                if tools.is_empty() {
                    return Err(Error::InvalidInput("the faces to thicken aren't on the bodies this acts on".into()));
                }
                applied(tools, *mode)
            }
            Op::Pattern { of, along } => {
                let moves = pattern_moves(along, env, &look)?;
                self.copied(scope, of, tag, |b| moves.iter().map(|m| b.transformed(m)).collect())
            }
            Op::Mirror { of, plane } => {
                let f = look.plane(plane)?;
                let m = Mirror::new(f.origin, f.z);
                self.copied(scope, of, tag, |b| vec![b.mirrored(&m)])
            }
            Op::ImportStep { text, mode: Mode::Surface } => sheets(coreshape::import::step::read_step_sheets(text, tag)?),
            Op::ImportStep { text, mode } => {
                let solids = coreshape::import::step::read_step(text, tag)?;
                // A file holding nothing but surfaces brings them in as they are, as there's nothing to merge them into.
                if solids.iter().all(|b| b.sheet) {
                    return sheets(solids);
                }
                // A file's solids are parts, and each of them can be a body. Merging is only for when that's asked for.
                if *mode == Mode::NewBody {
                    let added = solids.iter().map(|b| Arc::new(b.clone())).collect();
                    return Ok(RunResult { changed: Vec::new(), added, sketch: None, tools: Some((Arc::new(solids), *mode)), plane: None });
                }
                let count = solids.len();
                applied(solids, *mode).map_err(|e| {
                    if count > 1 {
                        Error::InvalidInput(format!("this file has {count} solids, and merging them into one body failed: {e}. With the mode new_body each stays a body of its own"))
                    } else {
                        e
                    }
                })
            }
            Op::ImportStepParts { text } => {
                let solids = coreshape::import::step::read_step_parts(text, tag)?;
                // Each is one part, whether it's a solid or a sheet.
                let added = solids.iter().map(|b| Arc::new(b.clone())).collect();
                Ok(RunResult { changed: Vec::new(), added, sketch: None, tools: Some((Arc::new(solids), Mode::NewBody)), plane: None })
            }
            Op::Solid { primitive, at, mode } => {
                let frame = look.placement(at)?;
                let made = match primitive {
                    Primitive::Box { dx, dy, dz } => box_solid(&frame, num(dx, env)?, num(dy, env)?, num(dz, env)?, tag)?,
                    Primitive::Cylinder { radius, height } => cylinder(&frame, num(radius, env)?, num(height, env)?, tag)?,
                    Primitive::Cone { r0, r1, height } => cone_frustum(&frame, num(r0, env)?, num(r1, env)?, num(height, env)?, tag)?,
                    Primitive::Sphere { radius } => sphere(&frame, num(radius, env)?, tag)?,
                    Primitive::Torus { major, minor } => torus(&frame, num(major, env)?, num(minor, env)?, tag)?,
                };
                applied(vec![made], *mode)
            }
            Op::Offset { distance } => {
                let d = num(distance, env)?;
                each("an offset", &|b| offset(b, d))
            }
            Op::Shell { open, thickness } => {
                let t = num(thickness, env)?;
                let everything = whole().ok_or_else(|| Error::InvalidInput("a shell needs a body to work on".into()))?;
                if open.is_empty() {
                    return each("a shell", &|b| shell(b, &[], t, tag));
                }
                let picked = named_faces(&everything, open)?;
                only_on_sheets(all, &picked, |b| b.faces.len(), "a shell")?;
                let mut out = Changes::default();
                for (at, list) in by_body(all, scope, &picked, |b| b.faces.len()) {
                    out.settle(scope[at].0, shell(&scope[at].1, &list, t, tag).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::Draft { faces, neutral, degrees } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("a draft needs a body to work on".into()))?;
                let picked = named_faces(&everything, faces)?;
                if picked.is_empty() {
                    return Err(Error::InvalidInput("a draft needs at least one face".into()));
                }
                only_on_sheets(all, &picked, |b| b.faces.len(), "a draft")?;
                let plane = look.plane(neutral)?;
                let (p, n, a) = (plane.z, plane.origin, num(degrees, env)?.to_radians());
                let mut out = Changes::default();
                for (at, list) in by_body(all, scope, &picked, |b| b.faces.len()) {
                    out.settle(scope[at].0, draft(&scope[at].1, &list, p, n, a).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::Fillet { edges, radius, propagate } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("a fillet needs a body to work on".into()))?;
                let picked = select_edges(edges, &everything, env)?;
                only_on_sheets(all, &picked, |b| b.edges.len(), "a fillet")?;
                let options = BlendOptions { propagate: *propagate };
                let r = num(radius, env)?;
                let mut out = Changes::default();
                let lists = by_body(all, scope, &picked, |b| b.edges.len());
                // Nothing picked is for the blend to say what it says about that.
                if lists.is_empty() {
                    let first = scope.first().ok_or_else(|| Error::InvalidInput("a fillet needs a body to work on".into()))?;
                    out.settle(first.0, fillet_with(&first.1, &[], r, options, tag)?, tag);
                }
                for (at, list) in lists {
                    out.settle(scope[at].0, fillet_with(&scope[at].1, &list, r, options, tag).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::FilletRadii { sets } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("a fillet needs a body to work on".into()))?;
                // Each body's edges from each set, with the set's radius.
                let mut for_body: Vec<(usize, Vec<(Vec<usize>, coreshape::Radius)>)> = Vec::new();
                for set in sets {
                    let picked = select_edges(&set.edges, &everything, env)?;
                    if picked.is_empty() {
                        return Err(Error::InvalidInput("one of this fillet's sets of edges has nothing in it".into()));
                    }
                    only_on_sheets(all, &picked, |b| b.edges.len(), "a fillet")?;
                    let radius = match &set.radius {
                        RadiusLaw::Constant(r) => coreshape::Radius::Constant(num(r, env)?),
                        RadiusLaw::Along { stations, start } => {
                            let mut list = Vec::with_capacity(stations.len());
                            for (at, r) in stations {
                                list.push((num(at, env)?, num(r, env)?));
                            }
                            let start = match start {
                                Some(p) => Some(at_point(p)?),
                                None => None,
                            };
                            coreshape::Radius::Along { stations: list, start }
                        }
                    };
                    for (at, list) in by_body(all, scope, &picked, |b| b.edges.len()) {
                        match for_body.iter_mut().find(|x| x.0 == at) {
                            Some(entry) => entry.1.push((list, radius.clone())),
                            None => for_body.push((at, vec![(list, radius.clone())])),
                        }
                    }
                }
                let mut out = Changes::default();
                for (at, list) in for_body {
                    out.settle(scope[at].0, coreshape::fillet_radii(&scope[at].1, &list, tag).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::Chamfer { edges, distance, propagate } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("a chamfer needs a body to work on".into()))?;
                let picked = select_edges(edges, &everything, env)?;
                only_on_sheets(all, &picked, |b| b.edges.len(), "a chamfer")?;
                let options = BlendOptions { propagate: *propagate };
                let d = num(distance, env)?;
                let mut out = Changes::default();
                let lists = by_body(all, scope, &picked, |b| b.edges.len());
                if lists.is_empty() {
                    let first = scope.first().ok_or_else(|| Error::InvalidInput("a chamfer needs a body to work on".into()))?;
                    out.settle(first.0, chamfer_with(&first.1, &[], d, options, tag)?, tag);
                }
                for (at, list) in lists {
                    out.settle(scope[at].0, chamfer_with(&scope[at].1, &list, d, options, tag).map_err(|e| on_body(e, scope[at].0, scope.len() > 1))?, tag);
                }
                Ok(out.done(None))
            }
            Op::Transform { motion, copy } => {
                if scope.is_empty() {
                    return Err(Error::InvalidInput("there is no body to move".into()));
                }
                let m = match motion {
                    Motion::Translate { by } => Frame::translation(at_point(by)?),
                    Motion::Rotate { point, axis, degrees } => {
                        let a = at_point(axis)?;
                        if !(a.norm() >= 1e-12 && a.norm() <= 1e150) {
                            return Err(Error::InvalidInput("a rotation's axis must have a sensible, non-zero length".into()));
                        }
                        Frame::rotation(at_point(point)?, a.normalized(), num(degrees, env)?.to_radians())
                    }
                    Motion::Frames { from, to } => look.placement(to)?.compose(&look.placement(from)?.inverse()),
                };
                let mut out = Changes::default();
                for (k, (id, body)) in scope.iter().enumerate() {
                    let mut moved = body.transformed(&m);
                    if *copy {
                        moved.mark_copy(tag, k as u32 + 1);
                        out.added.push(Arc::new(moved));
                    } else {
                        out.changed.push((*id, Some(Arc::new(moved))));
                    }
                }
                Ok(out.done(None))
            }
            Op::DeleteBodies => {
                if feature.scope.is_empty() {
                    return Err(Error::InvalidInput("deleting bodies has to name them".into()));
                }
                let mut out = Changes::default();
                out.changed = scope.iter().map(|(id, _)| (*id, None)).collect();
                Ok(out.done(None))
            }
            Op::BooleanBodies { operation, tools, keep_tools } => self.boolean_of_bodies(feature, scope, all, *operation, tools, *keep_tools),
            Op::Fill { sketch } => {
                let out = sketch_output(sketch)?;
                let profiles = out.closed(*sketch, "a fill")?;
                let made = profiles.iter().map(|p| fill(p, &out.frame, tag)).collect::<Result<Vec<Body>>>()?;
                let by_entity = out.by_entity;
                sheets(named_by_region(made, Vec::new(), by_entity))
            }
            Op::CopyFaces { faces } => {
                let everything = whole().ok_or_else(|| Error::InvalidInput("copying faces needs a body to take them from".into()))?;
                let picked = keyed_faces(&everything, faces)?;
                let mut made = Vec::new();
                for (at, list) in by_body(all, scope, &picked, |b| b.faces.len()) {
                    let mut copy = sheet_from_faces(&scope[at].1, &list)?;
                    // A face's name covers its copies, and its key tells them apart.
                    copy.mark_copy(tag, 1);
                    made.push(copy);
                }
                sheets(made)
            }
            Op::Sew { tolerance } => {
                let tolerance = tolerance.as_ref().map_or(Ok(1e-6), |t| num(t, env))?;
                let list: Vec<&(BodyId, Arc<Body>)> = scope.iter().filter(|(_, b)| !feature.scope.is_empty() || b.sheet).collect();
                if list.is_empty() {
                    return Err(Error::InvalidInput("there's no sheet here to sew".into()));
                }
                let bodies: Vec<&Body> = list.iter().map(|(_, b)| b.as_ref()).collect();
                let made = sew(&bodies, tolerance)?;
                let mut out = Changes::default();
                out.settle(list[0].0, made, tag);
                for (id, _) in &list[1..] {
                    out.changed.push((*id, None));
                }
                Ok(out.done(None))
            }
            Op::Trim { tool, keep } => {
                let list: Vec<&(BodyId, Arc<Body>)> = scope.iter().filter(|(_, b)| !feature.scope.is_empty() || b.sheet).collect();
                if list.is_empty() {
                    return Err(Error::InvalidInput("there's no sheet here to trim".into()));
                }
                let mut out = Changes::default();
                for (id, sheet) in list {
                    let (outside, inside) = trim_parts(sheet, tool, *keep, all, &|r| look.plane(r), &sketch_output).map_err(|e| on_body(e, *id, scope.len() > 1))?;
                    match (outside, inside) {
                        (Some(a), Some(b)) => {
                            out.settle(*id, a, tag);
                            out.added.extend(b.solids().into_iter().filter(|p| !p.faces.is_empty()).map(Arc::new));
                        }
                        (Some(a), None) | (None, Some(a)) => out.settle(*id, a, tag),
                        (None, None) => {}
                    }
                }
                Ok(out.done(None))
            }
            Op::Plane { def, flip } => Ok(RunResult { changed: Vec::new(), added: Vec::new(), sketch: None, tools: None, plane: Some(look.def(def, *flip)?) }),
            Op::SheetMetal { sketch, depth, flip, .. } => {
                let out = sketch_output(sketch)?;
                let Sheet { t, r, .. } = sheet_settings(&feature.op, feature.id, env)?;
                let body = match (out.profiles.as_slice(), out.open.as_slice()) {
                    ([region], []) => extrude(region, &out.frame, if *flip { -t } else { t }, tag)?,
                    ([], [chain]) => {
                        let depth = depth.as_ref().ok_or_else(|| {
                            Error::InvalidInput("sheet metal from a chain of lines needs a depth: how far its walls go along the sketch's normal".into())
                        })?;
                        let d = num(depth, env)?;
                        if !(d.abs() >= MIN_FEATURE) {
                            return Err(Error::InvalidInput(format!("a sheet metal chain's depth can't be {d}")));
                        }
                        let mut points = sheet_metal::chain_points(chain)?;
                        if *flip {
                            points.reverse();
                        }
                        extrude(&sheet_metal::chain_profile(&points, t, r)?, &out.frame, d, tag)?
                    }
                    ([], []) => return Err(Error::InvalidInput("the sketch has nothing to make sheet metal from".into())),
                    (regions, chains) => {
                        return Err(Error::InvalidInput(format!(
                            "sheet metal comes from one region or one chain of lines, and this sketch has {} regions and {} chains",
                            regions.len(),
                            chains.len()
                        )))
                    }
                };
                applied(vec![body], Mode::NewBody)
            }
            Op::Flange { edges, length, degrees, alignment, bend_radius, flip, corner, gap } => {
                let length = num(length, env)?;
                let angle = degrees.as_ref().map_or(Ok(90.0), |e| num(e, env))?.to_radians();
                self.fold_edges(edges, all, scope, env, tag, "flange", &|sheet| {
                    let radius = bend_radius.as_ref().map_or(Ok(sheet.r), |e| num(e, env))?;
                    let fold = sheet_metal::FlangeSize { length, angle, radius, thickness: sheet.t, alignment: *alignment }.fold()?;
                    let gap = gap.as_ref().map_or(Ok(sheet.gap), |e| num(e, env))?;
                    Ok((fold, *flip, corner.unwrap_or(sheet.corner), gap))
                })
            }
            Op::Hem { edges, kind, length, gap, bend_radius, degrees, alignment, flip } => {
                check_hem_fields(*kind, length.is_some(), gap.is_some(), bend_radius.is_some(), degrees.is_some())?;
                let length = length.as_ref().map_or(Ok(0.0), |e| num(e, env))?;
                self.fold_edges(edges, all, scope, env, tag, "hem", &|sheet| {
                    let radius = match kind {
                        HemKind::Straight => gap.as_ref().map_or(Ok(sheet.gap), |e| num(e, env))? / 2.0,
                        _ => bend_radius.as_ref().map_or(Ok(sheet.r), |e| num(e, env))?,
                    };
                    let angle = match kind {
                        HemKind::Straight | HemKind::Rounded => PI,
                        HemKind::Teardrop => degrees.as_ref().map_or(Ok(225.0), |e| num(e, env))?.to_radians(),
                        HemKind::Rolled => degrees.as_ref().map_or(Ok(270.0), |e| num(e, env))?.to_radians(),
                    };
                    let fold = sheet_metal::HemSize { kind: *kind, length, radius, angle, thickness: sheet.t, alignment: *alignment }.fold()?;
                    Ok((fold, *flip, Corner::Open, sheet.gap))
                })
            }
            Op::Tab { sketch } => {
                let drawn = sketch_output(sketch)?;
                let regions = drawn.closed(*sketch, "a tab")?;
                let several = scope.len() > 1;
                let mut parts: Vec<(BodyId, Body, Sheet, bool)> =
                    scope.iter().filter_map(|(id, b)| self.sheet_of(b, env).ok().map(|s| (*id, (**b).clone(), s, false))).collect();
                if parts.is_empty() {
                    return Err(Error::InvalidInput("a tab needs a sheet metal part to go on".into()));
                }
                for (i, region) in regions.iter().enumerate() {
                    let mut took = false;
                    for (id, body, sheet, touched) in parts.iter_mut() {
                        let joined = sheet_metal::tab(body, region, &drawn.frame, sheet.t, tag, &|b| with_region(b, i as u32)).map_err(|e| on_body(e, *id, several))?;
                        if let Some(joined) = joined {
                            (*body, *touched, took) = (joined, true, true);
                            break;
                        }
                    }
                    if !took {
                        return Err(Error::InvalidInput(format!(
                            "region {i} of the tab doesn't reach a sheet metal wall in its plane: draw it on the plane of a wall's face, overlapping or meeting the wall"
                        )));
                    }
                }
                let mut out = Changes::default();
                for (id, body, _, touched) in parts {
                    if touched {
                        out.settle(id, body, tag);
                    }
                }
                Ok(out.done(None))
            }
            Op::Bend { sketch, degrees, bend_radius, position, flip, right } => {
                let drawn = sketch_output(sketch)?;
                let line = match (drawn.profiles.as_slice(), drawn.open.as_slice()) {
                    ([], [wire]) if wire.segs.len() == 1 => match wire.segs[0].geom {
                        SegGeom::Line { a, b } => Some((a, b)),
                        _ => None,
                    },
                    _ => None,
                };
                let Some((a, b)) = line else {
                    return Err(Error::InvalidInput(format!(
                        "a bend goes along the one line in its sketch, and this sketch has {} regions and {} chains of curves",
                        drawn.profiles.len(),
                        drawn.open.len()
                    )));
                };
                let angle = degrees.as_ref().map_or(Ok(90.0), |e| num(e, env))?.to_radians();
                let several = scope.len() > 1;
                let middle = drawn.frame.to_world(v3((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, 0.0));
                let mut out = Changes::default();
                for (id, body) in scope {
                    let Ok(sheet) = self.sheet_of(body, env) else { continue };
                    if !body.aabb().inflated(4.0 * TOL).contains(middle) {
                        continue;
                    }
                    let radius = bend_radius.as_ref().map_or(Ok(sheet.r), |e| num(e, env))?;
                    let size = sheet_metal::LineBend { angle, radius, thickness: sheet.t, k_factor: sheet.k, position: *position, flip: *flip, right: *right };
                    if let Some(bent) = sheet_metal::bend_along(body, &drawn.frame, a, b, &size, tag).map_err(|e| on_body(e, *id, several))? {
                        out.settle(*id, bent, tag);
                    }
                }
                if out.changed.is_empty() {
                    return Err(Error::InvalidInput("the line to bend along isn't on the plane of a sheet metal wall".into()));
                }
                Ok(out.done(None))
            }
        }
    }

    /// Folds along edges of sheet metal parts, as a flange or a hem: `fold` says how, for each part, from its sizes and
    /// defaults.
    fn fold_edges(
        &self,
        edges: &EdgeSelect,
        all: &[Held],
        scope: &[(BodyId, Arc<Body>)],
        env: &Env,
        tag: u32,
        what: &str,
        fold: &dyn Fn(&Sheet) -> Result<(sheet_metal::Fold, bool, Corner, f64)>,
    ) -> Result<RunResult> {
        let everything = whole_of(all).ok_or_else(|| Error::InvalidInput(format!("a {what} needs a sheet metal part to go on")))?;
        let picked = select_edges(edges, &everything, env)?;
        if picked.is_empty() {
            return Err(Error::InvalidInput(format!("a {what} needs an edge to go on, and none is picked")));
        }
        let mut out = Changes::default();
        let mut region = 0u32;
        for (at, list) in by_body(all, scope, &picked, |b| b.edges.len()) {
            let (id, body) = &scope[at];
            let several = scope.len() > 1;
            let sheet = self.sheet_of(body, env).map_err(|e| on_body(e, *id, several))?;
            let (shape, flip, corner, gap) = fold(&sheet).map_err(|e| on_body(e, *id, several))?;
            let made = sheet_metal::folds(body, &list, &shape, flip, corner, gap, &format!("a {what}"), tag).map_err(|e| on_body(e, *id, several))?;
            let made: Vec<(Body, Option<Body>)> = made
                .into_iter()
                .map(|(piece, trim)| {
                    region += 1;
                    (with_region(piece, region - 1), trim)
                })
                .collect();
            out.settle(*id, sheet_metal::add_folds(body, &made, what).map_err(|e| on_body(e, *id, several))?, tag);
        }
        Ok(out.done(None))
    }

    /// A Boolean between bodies. As in Onshape, bodies that don't meet are refused, and without `keep_tools` the tools go.
    fn boolean_of_bodies(&self, feature: &Feature, scope: &[(BodyId, Arc<Body>)], all: &[Held], operation: BodyBoolean, tools: &[BodyId], keep_tools: bool) -> Result<RunResult> {
        let tag = feature.id.0;
        let mut tool_bodies: Vec<(BodyId, Arc<Body>)> = Vec::with_capacity(tools.len());
        for id in tools {
            let h = all.iter().find(|h| h.id == *id).ok_or_else(|| Error::InvalidInput(format!("it uses body {}, which isn't there at this point", id.text())))?;
            tool_bodies.push((h.id, h.body.clone()));
        }
        let mut out = Changes::default();
        match operation {
            BodyBoolean::Subtract => {
                if feature.scope.is_empty() {
                    return Err(Error::InvalidInput("subtracting has to name its targets".into()));
                }
                if tool_bodies.is_empty() {
                    return Err(Error::InvalidInput("subtracting needs at least one tool".into()));
                }
                let mut reached = false;
                for (id, target) in scope {
                    let near: Vec<Body> = tool_bodies.iter().filter(|(t, b)| t != id && boxes_meet(target, b)).map(|(_, b)| (**b).clone()).collect();
                    if near.is_empty() {
                        continue;
                    }
                    let made = boolean_many(target, &near, BoolOp::Subtract)?;
                    let before = coreshape::mass_properties(target).volume;
                    if made.faces.is_empty() || (coreshape::mass_properties(&made).volume - before).abs() > 1e-9 * before.abs().max(1.0) {
                        reached = true;
                        out.settle(*id, made, tag);
                    }
                }
                if !reached {
                    return Err(Error::InvalidInput("the tools don't reach the targets, so there's nothing to subtract".into()));
                }
                if !keep_tools {
                    for (id, _) in &tool_bodies {
                        if !out.changed.iter().any(|(c, _)| c == id) {
                            out.changed.push((*id, None));
                        }
                    }
                }
            }
            BodyBoolean::Union | BodyBoolean::Intersect => {
                let mut list: Vec<(BodyId, Arc<Body>)> = scope.to_vec();
                for t in &tool_bodies {
                    if !list.iter().any(|(id, _)| *id == t.0) {
                        list.push(t.clone());
                    }
                }
                if list.len() < 2 {
                    return Err(Error::InvalidInput("a Boolean needs at least two bodies".into()));
                }
                let mut made = (*list[0].1).clone();
                for (_, next) in &list[1..] {
                    made = match operation {
                        BodyBoolean::Union => union(&made, next)?,
                        _ => intersect(&made, next)?,
                    };
                }
                match operation {
                    BodyBoolean::Union if made.solid_count() >= list.len() => return Err(Error::InvalidInput("these bodies don't touch, so there's nothing to join".into())),
                    BodyBoolean::Intersect if made.faces.is_empty() => return Err(Error::InvalidInput("these bodies don't overlap, so nothing is left of them".into())),
                    _ => {}
                }
                if keep_tools {
                    out.added = made.solids().into_iter().map(Arc::new).collect();
                } else {
                    out.settle(list[0].0, made, tag);
                    for (id, _) in &list[1..] {
                        out.changed.push((*id, None));
                    }
                }
            }
        }
        Ok(out.done(None))
    }

    /// Copies added: of listed features' solids, applied as they were, or of each body acted on if none are listed.
    /// `copies` makes the images of one solid. A copy that doesn't touch what it was copied from is a body of its own.
    fn copied(&self, scope: &[(BodyId, Arc<Body>)], of: &[FeatureId], feature: u32, copies: impl Fn(&Body) -> Vec<Body>) -> Result<RunResult> {
        // Copies of sheets come out as sheets, whatever else is there or isn't.
        let only_sheets = !of.is_empty() && of.iter().all(|id| self.cache.get(id).and_then(|c| c.tools.as_ref()).is_some_and(|(tools, _)| tools.iter().all(|b| b.sheet)));
        if scope.is_empty() && !only_sheets {
            return Err(Error::InvalidInput("there is no body yet to copy".into()));
        }
        // Numbered from 1, since the original is copy 0 and has no mark.
        let copies = |b: &Body| -> Vec<Body> {
            let mut made = copies(b);
            for (k, copy) in made.iter_mut().enumerate() {
                copy.mark_copy(feature, k as u32 + 1);
            }
            made
        };
        let mut out = Changes::default();
        if of.is_empty() {
            for (id, body) in scope {
                out.settle(*id, boolean_many(body, &copies(body), BoolOp::Union)?, feature);
            }
            return Ok(out.done(None));
        }
        // What each body is by now, as one listed feature's copies can land on what another's did.
        let mut current: Vec<(BodyId, Arc<Body>)> = scope.to_vec();
        let mut gone: Vec<BodyId> = Vec::new();
        let mut touched: Vec<BodyId> = Vec::new();
        let (mut modes, mut all_made, mut mixed): (Vec<Mode>, Vec<Body>, bool) = (Vec::new(), Vec::new(), false);
        for id in of {
            if self.feature(*id).is_some_and(|f| f.suppressed) {
                return Err(Error::InvalidInput(format!("f{} is suppressed, so it can't be copied", id.0)));
            }
            let (tools, mode) = self
                .cache
                .get(id)
                .and_then(|c| c.tools.clone())
                .ok_or_else(|| Error::InvalidInput(format!("f{} makes no solid to copy: it must be an extrude, revolve, sweep, solid, or a pattern or mirror of those", id.0)))?;
            let made: Vec<Body> = tools.iter().flat_map(&copies).collect();
            // Another pattern can copy these in turn, each doing what the feature it came from did.
            if modes.iter().all(|m| *m == mode) {
                modes.push(mode);
                all_made.extend(made.iter().cloned());
            } else {
                mixed = true;
            }
            if mode == Mode::NewBody {
                out.added.extend(made.into_iter().map(Arc::new));
                continue;
            }
            let step = combined(&current, made, mode, feature)?;
            for (changed, now) in step.changed {
                match now {
                    Some(body) => {
                        if let Some(entry) = current.iter_mut().find(|(c, _)| *c == changed) {
                            entry.1 = body;
                        }
                        if !touched.contains(&changed) {
                            touched.push(changed);
                        }
                    }
                    None => {
                        current.retain(|(c, _)| *c != changed);
                        gone.push(changed);
                    }
                }
            }
            out.added.extend(step.added);
        }
        for id in touched {
            if let Some((_, body)) = current.iter().find(|(c, _)| *c == id) {
                out.changed.push((id, Some(body.clone())));
            }
        }
        out.changed.extend(gone.into_iter().map(|id| (id, None)));
        // Patterns of a cut and an add together can't be copied as one, and that's refused when something tries.
        let tools = modes.first().filter(|_| !mixed).map(|mode| (Arc::new(all_made), *mode));
        Ok(out.done(tools))
    }

    // ----- Saving and loading -----

    /// The document as a JSON value.
    pub fn to_json(&self) -> Value {
        let doc = Value::obj(vec![
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
            ("features", Value::Arr(self.features.iter().map(|f| feature_value(&self.as_saved(f))).collect())),
        ]);
        with_app(doc, self.app.as_ref())
    }

    /// A feature as it's saved: a sketch's projected entities are where they last were.
    fn as_saved(&self, f: &Feature) -> Feature {
        let mut out = f.clone();
        if let Op::ConstrainedSketch { sketch, .. } = &mut out.op {
            if sketch.projected().next().is_some() {
                *sketch = self.as_last_placed(f.id, sketch);
            }
        }
        out
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
        known_keys(v, &["format", "version", "units", "next_id", "parameters", "features", "app"])?;
        if let Some(units) = v.get("units").and_then(Value::as_str) {
            if units != "mm" {
                return Err(Error::InvalidInput(format!("units \"{units}\" are not supported")));
            }
        }
        let mut doc = Document::new();
        doc.app = v.get("app").cloned();
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
            if !feature.scope.is_empty() && feature.op.is_reference() {
                return Err(Error::InvalidInput(format!("feature f{} is a {}, which doesn't act on a body", feature.id.0, feature.op.kind())));
            }
            for r in feature.references() {
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

/// Kernel profiles from a solved sketch's closed regions. Each segment's id is its entity's id.
pub fn profiles_from_regions(regions: &[coreshape_sketch::Region]) -> Vec<Profile> {
    regions
        .iter()
        .map(|r| Profile { outer: wire_from_loop(&r.outer), holes: r.holes.iter().map(wire_from_loop).collect() })
        .collect()
}

/// The solver meets its equations to 1e-9 mm, so neighbouring ends can differ by that much. Here
/// each joint gets one position, taken from an arc where there is one, since an arc's ends follow
/// from its angles and can't be moved.
fn wire_from_loop(l: &coreshape_sketch::Loop) -> Wire {
    wire_from_segs(&l.segs, true)
}

/// A chain of a sketch's pieces as a kernel wire. Open, its two ends are where its pieces say.
fn wire_from_segs(pieces: &[coreshape_sketch::Seg], closed: bool) -> Wire {
    use coreshape_sketch::SegKind;
    let n = pieces.len();
    let joint = |i: usize| -> Vec2 {
        let (s, next) = (&pieces[i], &pieces[(i + 1) % n]);
        let p = match (&s.kind, &next.kind) {
            (SegKind::Arc { .. } | SegKind::Ellipse { .. } | SegKind::Spline { .. }, _) => s.end(),
            (_, SegKind::Arc { .. } | SegKind::Ellipse { .. } | SegKind::Spline { .. }) => next.start(),
            _ => s.end(),
        };
        v2(p[0], p[1])
    };
    let mut segs = Vec::with_capacity(n);
    for (i, s) in pieces.iter().enumerate() {
        let id = s.entity.0;
        match s.kind {
            SegKind::Line { .. } => {
                let a = if !closed && i == 0 { v2(s.start()[0], s.start()[1]) } else { joint((i + n - 1) % n) };
                let b = if !closed && i + 1 == n { v2(s.end()[0], s.end()[1]) } else { joint(i) };
                segs.push(Seg { geom: SegGeom::Line { a, b }, id });
            }
            SegKind::Arc { center, radius, a0, a1 } => {
                let center = v2(center[0], center[1]);
                if (a1 - a0).abs() >= TAU - 1e-9 {
                    // A whole circle is two halves, as the kernel draws one.
                    let middle = (a0 + a1) / 2.0;
                    segs.push(Seg { geom: SegGeom::Arc { center, radius, a0, a1: middle }, id });
                    segs.push(Seg { geom: SegGeom::Arc { center, radius, a0: middle, a1 }, id });
                } else {
                    segs.push(Seg { geom: SegGeom::Arc { center, radius, a0, a1 }, id });
                }
            }
            SegKind::Spline { ref shape, t0, t1 } => {
                let points = shape.points.iter().map(|q| v3(q[0], q[1], 0.0)).collect();
                let whole = Spline::new(shape.degree, shape.knots.clone(), points, shape.weights.clone());
                let part = whole.and_then(|c| if t1 >= t0 { c.sub(t0, t1) } else { c.sub(t1, t0).map(|p| p.reversed()) });
                match part {
                    Ok(part) => segs.push(Seg { geom: SegGeom::Spline(Arc::new(part)), id }),
                    // A spline whose control points have come together. It's refused with a reason when it's used.
                    Err(_) => {
                        let (a, b) = (s.start(), s.end());
                        segs.push(Seg { geom: SegGeom::Line { a: v2(a[0], a[1]), b: v2(b[0], b[1]) }, id })
                    }
                }
            }
            // An ellipse is exactly a rational spline, which is how the kernel sweeps one.
            SegKind::Ellipse { center, x, major, minor, a0, a1 } => {
                let (at, along, across) = (v3(center[0], center[1], 0.0), v3(x[0], x[1], 0.0), v3(-x[1], x[0], 0.0));
                match Spline::ellipse_arc(at, along, across, major, minor, a0, a1) {
                    Ok(spline) => segs.push(Seg { geom: SegGeom::Spline(Arc::new(spline)), id }),
                    // Its numbers have already been through the solver, so this means something absurd
                    // like an overflow. An empty segment gets refused with a reason when it's used.
                    Err(_) => segs.push(Seg { geom: SegGeom::Line { a: v2(center[0], center[1]), b: v2(center[0], center[1]) }, id }),
                }
            }
        }
    }
    Wire { segs }
}

/// The profiles of the regions picked from one sketch. One that has gone is refused, saying what's there.
fn picked_profiles(id: FeatureId, out: &SketchOutput, regions: &[RegionPick]) -> Result<Vec<Profile>> {
    let solved = out.solved.as_ref().ok_or_else(|| Error::InvalidInput(format!("regions can only be picked from a constrained sketch, and f{} isn't one", id.0)))?;
    let found = solved.arrangement();
    let mut list = Vec::new();
    for pick in regions.iter().filter(|r| r.sketch == id) {
        match found.region(pick.region) {
            Some(region) => list.push(region.clone()),
            None => {
                let there: Vec<String> = found.regions.iter().map(|r| r.id.name()).collect();
                let names = if there.is_empty() { "none".to_string() } else { there.join(", ") };
                return Err(Error::InvalidProfile(format!("picked region {} isn't in sketch f{} any more. It has: {names}", pick.region.name(), id.0)));
            }
        }
    }
    let list = profiles_from_regions(&list);
    for p in &list {
        p.validate()?;
    }
    Ok(list)
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

/// A sketch's curves as a sweep's path, in space: its open curves, or else the outline of its one region.
fn path_from_sketch(id: FeatureId, out: &SketchOutput) -> Result<Vec<PathCurve>> {
    let outlines: usize = out.profiles.iter().map(|p| 1 + p.holes.len()).sum();
    if !out.open.is_empty() && outlines > 0 {
        return Err(Error::InvalidInput(format!("a sweep's path is one chain of curves, and f{} has {outlines} closed outline(s) as well as open curves", id.0)));
    }
    if out.open.is_empty() && outlines != 1 {
        return Err(Error::InvalidInput(format!("a sweep's path is one chain of curves, and f{} has {outlines} closed outlines", id.0)));
    }
    let wires: Vec<&Wire> = if out.open.is_empty() { vec![&out.profiles[0].outer] } else { out.open.iter().collect() };
    let f = &out.frame;
    let mut pieces = Vec::new();
    for seg in wires.iter().flat_map(|w| &w.segs) {
        pieces.push(match seg.geom {
            SegGeom::Line { a, b } => PathCurve::Line { from: f.point(a.x, a.y, 0.0), to: f.point(b.x, b.y, 0.0) },
            SegGeom::Arc { center, radius, a0, a1 } => PathCurve::Arc {
                center: f.point(center.x, center.y, 0.0),
                axis: if a1 > a0 { f.z } else { -f.z },
                from: f.point(center.x + radius * a0.cos(), center.y + radius * a0.sin(), 0.0),
                angle: (a1 - a0).abs(),
            },
            SegGeom::Spline(_) => {
                return Err(Error::InvalidInput(format!("a sweep's path can be lines and arcs for now, and f{} has a spline or an ellipse in it", id.0)))
            }
        });
    }
    Ok(pieces)
}

/// A part's edges as a sweep's path.
fn path_from_edges(body: &Body, edges: &[usize]) -> Result<Vec<PathCurve>> {
    edges
        .iter()
        .map(|&e| {
            let edge = &body.edges[e];
            let (a, b) = (body.vertices[edge.v0], body.vertices[edge.v1]);
            match &edge.curve {
                Curve::Line { .. } => Ok(PathCurve::Line { from: a, to: b }),
                Curve::Circle { frame, .. } => Ok(PathCurve::Arc { center: frame.origin, axis: frame.z, from: a, angle: edge.t1 - edge.t0 }),
                other => {
                    let m = edge.eval_mid();
                    let kind = match other {
                        Curve::Ellipse { .. } => "an ellipse",
                        Curve::Spline(_) => "a spline",
                        _ => "an intersection curve",
                    };
                    Err(Error::InvalidInput(format!("a sweep's path can be lines and arcs for now, and the edge through ({:.3}, {:.3}, {:.3}) is {kind}", m.x, m.y, m.z)))
                }
            }
        })
        .collect()
}

/// An extruded face's walls and ends, named for it. A wall swept from segment `id` is `wall.entry`, holding the faces
/// `across(id)` gives, those on the other side of the edge it came from, and the ends hold the face they came from.
fn named_for_face(mut body: Body, feature: u32, entry: u32, from: FaceTag, across: impl Fn(u32) -> Vec<FaceTag>) -> Body {
    for f in &mut body.faces {
        if f.tag.feature != feature {
            continue;
        }
        if f.tag.slot >= SLOT_SEG_BASE {
            let mut between = across(f.tag.slot - SLOT_SEG_BASE);
            between.sort();
            between.dedup();
            f.origin.between = between;
            f.tag.slot = SLOT_WALL_BASE + entry;
        } else {
            f.origin.between = vec![from];
        }
    }
    body
}

/// The rigid moves that take the original to each of a pattern's copies.
fn pattern_moves(along: &PatternPath, env: &Env, look: &Looker) -> Result<Vec<Frame>> {
    Ok(match along {
        PatternPath::Linear { direction, count, spacing } => {
            let (d, n, step) = (look.line(direction)?.1, count_of(count, env, "a pattern's count")?, num(spacing, env)?);
            (1..n).map(|k| Frame::translation(d * (step * k as f64))).collect()
        }
        PatternPath::Circular { axis, count, degrees } => {
            let ((c, a), n) = (look.line(axis)?, count_of(count, env, "a pattern's count")?);
            let total = num(degrees, env)?;
            let full = (total.abs() - 360.0).abs() < 1e-9;
            if !full && total.abs() > 360.0 {
                return Err(Error::InvalidInput(format!("a pattern's angle of {total} is over a full turn")));
            }
            let gaps = if full { n } else { n.saturating_sub(1).max(1) };
            let step = total.to_radians() / gaps as f64;
            (1..n).map(|k| Frame::rotation(c, a, step * k as f64)).collect()
        }
    })
}

/// The faces of a body that go by these names. A name with no faces is an error, since a change that
/// removed a face a later feature needs should say so.
fn named_faces(body: &Body, names: &[FaceRef]) -> Result<Vec<usize>> {
    let mut out = Vec::new();
    for name in names {
        let before = out.len();
        out.extend(body.faces.iter().enumerate().filter(|(_, f)| f.tag == name.tag()).map(|(i, _)| i));
        if out.len() == before {
            return Err(Error::InvalidInput(format!("the face {} no longer exists in the model", name.name())));
        }
    }
    Ok(out)
}

/// The faces some keys pick out, each once, in face order.
fn keyed_faces(body: &Body, keys: &[FaceKey]) -> Result<Vec<usize>> {
    if keys.is_empty() {
        return Err(Error::InvalidInput("this needs at least one face".into()));
    }
    let mut out = Vec::new();
    for key in keys {
        let found = find_faces(body, key);
        if found.is_empty() {
            return Err(Error::InvalidInput(format!("the face {} no longer exists in the model", key.text())));
        }
        out.extend(found);
    }
    out.sort_unstable();
    out.dedup();
    Ok(out)
}

/// The edges of a body a selection picks out.
pub fn select_edges(query: &EdgeSelect, body: &Body, env: &Env) -> Result<Vec<usize>> {
    let uses = body.edge_uses();
    let keys = Keys::default();
    let mut list = Vec::new();
    for e in 0..body.edges.len() {
        if query.matches(body, e, &uses, env, &keys)? {
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

fn read_alignment(v: &Value, what: &str) -> Result<Alignment> {
    match v.get("alignment") {
        None => Ok(Alignment::Outer),
        Some(a) => a
            .as_str()
            .and_then(Alignment::from_word)
            .ok_or_else(|| Error::InvalidInput(format!("{what}'s \"alignment\" is \"outer\", \"inner\" or \"bend\""))),
    }
}

fn read_corner(v: &Value, what: &str) -> Result<Option<Corner>> {
    v.get("corner")
        .map(|c| c.as_str().and_then(Corner::from_word).ok_or_else(|| Error::InvalidInput(format!("{what}'s \"corner\" is \"open\" or \"closed\""))))
        .transpose()
}

fn read_flip(v: &Value) -> Result<bool> {
    match v.get("flip") {
        None => Ok(false),
        Some(b) => b.as_bool().ok_or_else(|| Error::InvalidInput("\"flip\" must be true or false".into())),
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

fn extrude_end_value(end: &ExtrudeEnd) -> Value {
    match end {
        ExtrudeEnd::Blind => Value::str("blind"),
        ExtrudeEnd::ThroughAll => Value::str("through_all"),
        ExtrudeEnd::UpToNext => Value::str("up_to_next"),
        ExtrudeEnd::UpToFace(key) => Value::obj(vec![("type", Value::str("up_to_face")), ("face", Value::str(key.text()))]),
        ExtrudeEnd::UpToPlane(plane) => Value::obj(vec![("type", Value::str("up_to_face")), ("face", plane_ref_value(plane))]),
        ExtrudeEnd::UpToPart(part) => Value::obj(vec![("type", Value::str("up_to_part")), ("part", Value::str(part.text()))]),
        ExtrudeEnd::UpToVertex(vertex) => {
            let on = match vertex {
                VertexRef::Point(p) => Value::obj(vec![("point", point3_value(p))]),
                VertexRef::Faces { faces, near } => {
                    let mut items = vec![("faces", Value::Arr(faces.iter().map(|k| Value::str(k.text())).collect()))];
                    if let Some(p) = near {
                        items.push(("near", point3_value(p)));
                    }
                    Value::obj(items)
                }
            };
            Value::obj(vec![("type", Value::str("up_to_vertex")), ("vertex", on)])
        }
    }
}

fn read_extrude_end(v: &Value) -> Result<ExtrudeEnd> {
    let Some(end) = v.get("end") else { return Ok(ExtrudeEnd::Blind) };
    let kind = end.as_str().or_else(|| end.get("type").and_then(Value::as_str)).unwrap_or("");
    let text = |key: &str| -> Result<&str> { end.get(key).and_then(Value::as_str).ok_or_else(|| Error::InvalidInput(format!("an extrude's {kind} end needs \"{key}\""))) };
    Ok(match kind {
        "blind" => ExtrudeEnd::Blind,
        "through_all" => ExtrudeEnd::ThroughAll,
        "up_to_next" => ExtrudeEnd::UpToNext,
        // A face's name or key, or a plane: Top, Front, Right, a plane feature's id, or one given.
        "up_to_face" => {
            let face = end.get("face").ok_or_else(|| Error::InvalidInput("an extrude's up_to_face end needs \"face\"".into()))?;
            match face.as_str() {
                Some(name) if !matches!(name, "top" | "front" | "right") => ExtrudeEnd::UpToFace(FaceKey::parse(name)?),
                _ => ExtrudeEnd::UpToPlane(read_plane_ref(face)?),
            }
        }
        "up_to_part" => ExtrudeEnd::UpToPart(BodyId::parse(text("part")?)?),
        "up_to_vertex" => {
            let on = end.get("vertex").ok_or_else(|| Error::InvalidInput("an extrude's up_to_vertex end needs \"vertex\"".into()))?;
            ExtrudeEnd::UpToVertex(read_vertex_ref(on)?)
        }
        other => return Err(Error::InvalidInput(format!("unknown extrude end \"{other}\"; expected blind, through_all, up_to_next, up_to_face, up_to_part or up_to_vertex"))),
    })
}

/// A vertex: `{"faces": [...], "near": [x, y, z]}`, the one those faces share, nearest `near` if they share several, or
/// `{"point": [x, y, z]}`.
fn read_vertex_ref(on: &Value) -> Result<VertexRef> {
    if let Some(point) = on.get("point") {
        known_keys(on, &["point"])?;
        return Ok(VertexRef::Point(read_point3(point)?));
    }
    known_keys(on, &["faces", "near"])?;
    let list = on.get("faces").and_then(Value::as_array).filter(|l| !l.is_empty()).ok_or_else(|| Error::InvalidInput("a vertex is {\"faces\": [...]} with the faces it's on, or {\"point\": [x, y, z]}".into()))?;
    let faces = list.iter().map(|k| FaceKey::parse(k.as_str().ok_or_else(|| Error::InvalidInput("a vertex's \"faces\" are the names of faces".into()))?)).collect::<Result<Vec<_>>>()?;
    Ok(VertexRef::Faces { faces, near: on.get("near").map(read_point3).transpose()? })
}

/// An extrude's or a revolve's picks: each a region's id, from `sketch`, or `{"sketch": 4, "region": "r..."}`. `what` is
/// the feature, "an extrude".
fn read_region_picks(v: &Value, sketch: Option<FeatureId>, what: &str) -> Result<Vec<RegionPick>> {
    let Some(list) = v.get("regions") else { return Ok(Vec::new()) };
    let list = list.as_array().ok_or_else(|| Error::InvalidInput(format!("{what}'s \"regions\" must be a list of region ids")))?;
    let mut out = Vec::with_capacity(list.len());
    for item in list {
        out.push(match item.as_str() {
            Some(id) => {
                let sketch = sketch.ok_or_else(|| Error::InvalidInput(format!("a region named by its id alone needs {what}'s \"sketch\". From several sketches, each is {{\"sketch\": 4, \"region\": \"r...\"}}")))?;
                RegionPick { sketch, region: coreshape_sketch::RegionId::parse(id).map_err(Error::InvalidInput)? }
            }
            None => read_region_pick(item)?,
        });
    }
    Ok(out)
}

/// `{"sketch": 4, "region": "r..."}`.
fn read_region_pick(item: &Value) -> Result<RegionPick> {
    known_keys(item, &["sketch", "region"])?;
    let id = item.get("region").and_then(Value::as_str).ok_or_else(|| Error::InvalidInput("a picked region is an id, or {\"sketch\": 4, \"region\": \"r...\"}".into()))?;
    Ok(RegionPick { sketch: read_sketch_id(item)?, region: coreshape_sketch::RegionId::parse(id).map_err(Error::InvalidInput)? })
}

/// Picked regions, each as its id alone when it's from `sketch`.
fn region_picks_value(regions: &[RegionPick], sketch: Option<FeatureId>) -> Value {
    Value::Arr(
        regions
            .iter()
            .map(|r| {
                if Some(r.sketch) == sketch {
                    Value::str(r.region.name())
                } else {
                    Value::obj(vec![("sketch", Value::Num(r.sketch.0 as f64)), ("region", Value::str(r.region.name()))])
                }
            })
            .collect(),
    )
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
    if let Some(plane) = &p.on_plane {
        return Value::obj(vec![("on_plane", plane_ref_value(plane))]);
    }
    let mut items = vec![("origin", point3_value(&p.origin)), ("z", point3_value(&p.z)), ("x", point3_value(&p.x))];
    if let Some(key) = &p.on_face {
        items.push(("on_face", Value::str(key.text())));
    }
    Value::obj(items)
}

fn with_app(v: Value, app: Option<&Value>) -> Value {
    match (v, app) {
        (Value::Obj(mut items), Some(a)) => {
            items.push(("app".to_string(), a.clone()));
            Value::Obj(items)
        }
        (v, _) => v,
    }
}

fn body_list_value(list: &[BodyId]) -> Value {
    Value::Arr(list.iter().map(|b| Value::str(b.text())).collect())
}

/// A list of bodies by name, from a field that may be missing.
pub fn read_body_list(v: &Value, key: &str) -> Result<Vec<BodyId>> {
    let bad = || Error::InvalidInput(format!("\"{key}\" must be a list of bodies, like [\"f3\", \"f3.2\"]"));
    match v.get(key) {
        None => Ok(Vec::new()),
        Some(list) => list.as_array().ok_or_else(bad)?.iter().map(|b| BodyId::parse(b.as_str().ok_or_else(bad)?)).collect(),
    }
}

/// A placement as files write it: `{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}`, each part with a default.
pub fn read_placement(v: &Value) -> Result<Placement> {
    if let Some(plane) = v.get("on_plane") {
        known_keys(v, &["on_plane"]).map_err(|_| Error::InvalidInput("a placement on a plane takes its origin, normal and x from the plane, so it has \"on_plane\" alone".into()))?;
        return Ok(Placement::on_plane(read_plane_ref(plane)?));
    }
    known_keys(v, &["origin", "z", "x", "on_face", "on_plane"])?;
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
    let on_face = match v.get("on_face") {
        None => None,
        Some(k) => Some(FaceKey::parse(k.as_str().ok_or_else(|| Error::InvalidInput("\"on_face\" must be the name of a face".into()))?)?),
    };
    Ok(Placement { origin: dir("origin", [0.0, 0.0, 0.0])?, z, x: dir("x", x_default)?, on_face, on_plane: None })
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
        Shape::Path { start, moves } | Shape::OpenPath { start, moves } => Value::obj(vec![
            ("type", Value::str(if matches!(s, Shape::OpenPath { .. }) { "open_path" } else { "path" })),
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
                            Move::Spline { through } => Value::obj(vec![("spline_through", Value::Arr(through.iter().map(point2_value).collect()))]),
                        })
                        .collect(),
                ),
            ),
        ]),
        Shape::Ellipse { cx, cy, rx, ry, degrees } => Value::obj(vec![
            ("type", Value::str("ellipse")),
            ("cx", expr_value(cx)),
            ("cy", expr_value(cy)),
            ("rx", expr_value(rx)),
            ("ry", expr_value(ry)),
            ("degrees", expr_value(degrees)),
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
        "ellipse" => {
            known_keys(v, &["type", "cx", "cy", "rx", "ry", "degrees"])?;
            Ok(Shape::Ellipse {
                cx: at_origin("cx")?,
                cy: at_origin("cy")?,
                rx: expr_field(v, "rx", &what)?,
                ry: expr_field(v, "ry", &what)?,
                degrees: at_origin("degrees")?,
            })
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
        "path" | "open_path" => {
            let open = kind == "open_path";
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
                } else if let Some(through) = m.get("spline_through") {
                    known_keys(m, &["spline_through"])?;
                    let through = through
                        .as_array()
                        .ok_or_else(|| Error::InvalidInput("\"spline_through\" must be a list of points".into()))?
                        .iter()
                        .map(read_point2)
                        .collect::<Result<Vec<Point2>>>()?;
                    moves.push(Move::Spline { through });
                } else {
                    return Err(Error::InvalidInput("a path move is {\"line_to\": ...}, {\"spline_through\": ...} or {\"arc_to\": ..., \"center\": ..., \"ccw\": ...}".into()));
                }
            }
            if open {
                return Ok(Shape::OpenPath { start: read_point2(field(v, "start", &what)?)?, moves });
            }
            Ok(Shape::Path { start: read_point2(field(v, "start", &what)?)?, moves })
        }
        other => Err(Error::InvalidInput(format!(
            "unknown shape \"{other}\"; expected rect, rounded_rect, circle, polygon, path or open_path"
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
        EdgeSelect::OnFaceKey(k) => Value::obj(vec![("on_face", Value::str(k.text()))]),
        EdgeSelect::BetweenKeys(a, b) => Value::obj(vec![("between", Value::Arr(vec![Value::str(a.text()), Value::str(b.text())]))]),
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
                let key = |x: &Value| {
                    x.as_str().ok_or_else(|| Error::InvalidInput("\"between\" takes two face names".into())).and_then(FaceKey::parse)
                };
                let (a, b) = (key(a)?, key(b)?);
                Ok(if a.is_plain() && b.is_plain() { EdgeSelect::Between(a.face, b.face) } else { EdgeSelect::BetweenKeys(a, b) })
            }
            _ => Err(Error::InvalidInput("\"between\" takes two face names".into())),
        };
    }
    if let Some(f) = v.get("on_face") {
        known_keys(v, &["on_face"])?;
        let name = f.as_str().ok_or_else(|| Error::InvalidInput("\"on_face\" takes a face name".into()))?;
        let key = FaceKey::parse(name)?;
        return Ok(if key.is_plain() { EdgeSelect::OnFace(key.face) } else { EdgeSelect::OnFaceKey(key) });
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
        Mode::NewBody => "new_body",
        Mode::Surface => "surface",
    })
}

fn read_mode(v: &Value) -> Result<Mode> {
    match v.get("mode").and_then(Value::as_str) {
        None | Some("add") => Ok(Mode::Add),
        Some("remove") => Ok(Mode::Remove),
        Some("intersect") => Ok(Mode::Intersect),
        Some("new_body") => Ok(Mode::NewBody),
        Some("surface") => Ok(Mode::Surface),
        Some(other) => Err(Error::InvalidInput(format!("unknown mode \"{other}\"; expected add, remove, intersect, new_body or surface"))),

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
        Op::ConstrainedSketch { plane, sketch } => Value::obj(vec![
            ("type", Value::str("constrained_sketch")),
            ("plane", placement_value(plane)),
            ("sketch", sketch.to_json()),
        ]),
        Op::Extrude { sketch, start, length, mode, regions, faces, end, opposite } => {
            let mut items = vec![("type", Value::str("extrude"))];
            if let Some(sketch) = sketch {
                items.push(("sketch", Value::Num(sketch.0 as f64)));
            }
            items.push(("start", expr_value(start)));
            if *end == ExtrudeEnd::Blind || *length != Expr::number(0.0) {
                items.push(("length", expr_value(length)));
            }
            items.push(("mode", mode_value(*mode)));
            if !regions.is_empty() {
                items.push(("regions", region_picks_value(regions, *sketch)));
            }
            if !faces.is_empty() {
                items.push(("faces", key_list_value(faces)));
            }
            if *end != ExtrudeEnd::Blind {
                items.push(("end", extrude_end_value(end)));
            }
            if *opposite {
                items.push(("opposite", Value::Bool(true)));
            }
            Value::obj(items)
        }
        Op::Revolve { sketch, degrees, mode, axis, regions } => {
            let mut items = vec![("type", Value::str("revolve")), ("sketch", Value::Num(sketch.0 as f64)), ("degrees", expr_value(degrees)), ("mode", mode_value(*mode))];
            if let Some(axis) = axis {
                items.push(("axis", Value::obj(vec![("point", point2_value(&axis.point)), ("direction", point2_value(&axis.direction))])));
            }
            if !regions.is_empty() {
                items.push(("regions", region_picks_value(regions, Some(*sketch))));
            }
            Value::obj(items)
        }
        Op::Transform { motion, copy } => {
            let how = match motion {
                Motion::Translate { by } => Value::obj(vec![("type", Value::str("translate")), ("by", point3_value(by))]),
                Motion::Rotate { point, axis, degrees } => Value::obj(vec![("type", Value::str("rotate")), ("point", point3_value(point)), ("axis", point3_value(axis)), ("degrees", expr_value(degrees))]),
                Motion::Frames { from, to } => Value::obj(vec![("type", Value::str("frames")), ("from", placement_value(from)), ("to", placement_value(to))]),
            };
            Value::obj(vec![("type", Value::str("transform")), ("motion", how), ("copy", Value::Bool(*copy))])
        }
        Op::DeleteBodies => Value::obj(vec![("type", Value::str("delete_bodies"))]),
        Op::BooleanBodies { operation, tools, keep_tools } => Value::obj(vec![
            ("type", Value::str("boolean")),
            (
                "operation",
                Value::str(match operation {
                    BodyBoolean::Union => "union",
                    BodyBoolean::Subtract => "subtract",
                    BodyBoolean::Intersect => "intersect",
                }),
            ),
            ("tools", body_list_value(tools)),
            ("keep_tools", Value::Bool(*keep_tools)),
        ]),
        Op::ImportStep { text, mode } => Value::obj(vec![("type", Value::str("import_step")), ("text", Value::str(text)), ("mode", mode_value(*mode))]),
        Op::ImportStepParts { text } => Value::obj(vec![("type", Value::str("import_step_parts")), ("text", Value::str(text))]),
        Op::Fill { sketch } => Value::obj(vec![("type", Value::str("fill")), ("sketch", Value::Num(sketch.0 as f64))]),
        Op::SheetMetal { sketch, thickness, bend_radius, k_factor, depth, flip, minimal_gap, corner } => {
            let mut items = vec![("type", Value::str("sheet_metal")), ("sketch", Value::Num(sketch.0 as f64)), ("thickness", expr_value(thickness))];
            for (key, e) in [("bend_radius", bend_radius), ("k_factor", k_factor), ("depth", depth), ("minimal_gap", minimal_gap)] {
                if let Some(e) = e {
                    items.push((key, expr_value(e)));
                }
            }
            if *flip {
                items.push(("flip", Value::Bool(true)));
            }
            if *corner != Corner::Open {
                items.push(("corner", Value::str(corner.word())));
            }
            Value::obj(items)
        }
        Op::Flange { edges, length, degrees, alignment, bend_radius, flip, corner, gap } => {
            let mut items = vec![("type", Value::str("flange")), ("edges", edge_select_value(edges)), ("length", expr_value(length))];
            if let Some(e) = degrees {
                items.push(("degrees", expr_value(e)));
            }
            if *alignment != Alignment::Outer {
                items.push(("alignment", Value::str(alignment.word())));
            }
            if let Some(e) = bend_radius {
                items.push(("bend_radius", expr_value(e)));
            }
            if *flip {
                items.push(("flip", Value::Bool(true)));
            }
            if let Some(c) = corner {
                items.push(("corner", Value::str(c.word())));
            }
            if let Some(e) = gap {
                items.push(("gap", expr_value(e)));
            }
            Value::obj(items)
        }
        Op::Hem { edges, kind, length, gap, bend_radius, degrees, alignment, flip } => {
            let mut items = vec![("type", Value::str("hem")), ("edges", edge_select_value(edges)), ("kind", Value::str(kind.word()))];
            for (key, e) in [("length", length), ("gap", gap), ("bend_radius", bend_radius), ("degrees", degrees)] {
                if let Some(e) = e {
                    items.push((key, expr_value(e)));
                }
            }
            if *alignment != Alignment::Outer {
                items.push(("alignment", Value::str(alignment.word())));
            }
            if *flip {
                items.push(("flip", Value::Bool(true)));
            }
            Value::obj(items)
        }
        Op::Tab { sketch } => Value::obj(vec![("type", Value::str("tab")), ("sketch", Value::Num(sketch.0 as f64))]),
        Op::Bend { sketch, degrees, bend_radius, position, flip, right } => {
            let mut items = vec![("type", Value::str("bend")), ("sketch", Value::Num(sketch.0 as f64))];
            for (key, e) in [("degrees", degrees), ("bend_radius", bend_radius)] {
                if let Some(e) = e {
                    items.push((key, expr_value(e)));
                }
            }
            if *position != BendPosition::Center {
                items.push(("position", Value::str(position.word())));
            }
            if *flip {
                items.push(("flip", Value::Bool(true)));
            }
            if *right {
                items.push(("moves", Value::str("right")));
            }
            Value::obj(items)
        }
        Op::CopyFaces { faces } => Value::obj(vec![("type", Value::str("copy_faces")), ("faces", key_list_value(faces))]),
        Op::Sew { tolerance } => {
            let mut items = vec![("type", Value::str("sew"))];
            if let Some(t) = tolerance {
                items.push(("tolerance", expr_value(t)));
            }
            Value::obj(items)
        }
        Op::Trim { tool, keep } => {
            let mut items = vec![("type", Value::str("trim"))];
            match tool {
                TrimTool::Body(id) => items.push(("tool", Value::str(id.text()))),
                TrimTool::Plane(PlaneRef::Given { point, normal }) => {
                    items.push(("point", point3_value(point)));
                    items.push(("normal", point3_value(normal)));
                }
                TrimTool::Plane(plane) => items.push(("plane", plane_ref_value(plane))),
                TrimTool::Sketch(id) => items.push(("sketch", Value::Num(id.0 as f64))),
            }
            items.push((
                "keep",
                Value::str(match keep {
                    TrimKeep::Outside => "outside",
                    TrimKeep::Inside => "inside",
                    TrimKeep::Both => "both",
                }),
            ));
            Value::obj(items)
        }
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
        Op::Loft { sections, mode } => Value::obj(vec![
            ("type", Value::str("loft")),
            (
                "sections",
                Value::Arr(
                    sections
                        .iter()
                        .map(|s| match s.region {
                            None => Value::Num(s.sketch.0 as f64),
                            Some(region) => Value::obj(vec![("sketch", Value::Num(s.sketch.0 as f64)), ("region", Value::str(region.name()))]),
                        })
                        .collect(),
                ),
            ),
            ("mode", mode_value(*mode)),
        ]),
        Op::Sweep { sketch, path, mode } => Value::obj(vec![
            ("type", Value::str("sweep")),
            ("sketch", Value::Num(sketch.0 as f64)),
            ("path", sweep_path_value(path)),
            ("mode", mode_value(*mode)),
        ]),
        Op::Pattern { of, along } => {
            let mut items = vec![("type", Value::str("pattern")), ("of", ids_value(of))];
            match along {
                PatternPath::Linear { direction, count, spacing } => {
                    items.push(("direction", match direction {
                        LineRef::Given { direction, .. } => point3_value(direction),
                        line => line_ref_value(line),
                    }));
                    items.push(("count", expr_value(count)));
                    items.push(("spacing", expr_value(spacing)));
                }
                PatternPath::Circular { axis, count, degrees } => {
                    match axis {
                        LineRef::Given { point, direction } => {
                            items.push(("center", point3_value(point)));
                            items.push(("axis", point3_value(direction)));
                        }
                        line => items.push(("axis", line_ref_value(line))),
                    }
                    items.push(("count", expr_value(count)));
                    items.push(("degrees", expr_value(degrees)));
                }
            }
            Value::obj(items)
        }
        Op::Mirror { of, plane } => {
            let mut items = vec![("type", Value::str("mirror")), ("of", ids_value(of))];
            match plane {
                PlaneRef::Given { point, normal } => {
                    items.push(("point", point3_value(point)));
                    items.push(("normal", point3_value(normal)));
                }
                plane => items.push(("plane", plane_ref_value(plane))),
            }
            Value::obj(items)
        }
        Op::Helix { sketch, point, axis, pitch, turns, left_handed, mode } => Value::obj(vec![
            ("type", Value::str("helix")),
            ("sketch", Value::Num(sketch.0 as f64)),
            ("point", point3_value(point)),
            ("axis", point3_value(axis)),
            ("pitch", expr_value(pitch)),
            ("turns", expr_value(turns)),
            ("left_handed", Value::Bool(*left_handed)),
            ("mode", mode_value(*mode)),
        ]),
        Op::DeleteFace { faces } => Value::obj(vec![("type", Value::str("delete_face")), ("faces", key_list_value(faces))]),
        Op::Thicken { faces, thickness, mode } => Value::obj(vec![
            ("type", Value::str("thicken")),
            ("faces", key_list_value(faces)),
            ("thickness", expr_value(thickness)),
            ("mode", mode_value(*mode)),
        ]),
        Op::Offset { distance } => Value::obj(vec![("type", Value::str("offset")), ("distance", expr_value(distance))]),
        Op::Shell { open, thickness } => Value::obj(vec![
            ("type", Value::str("shell")),
            ("open", face_list_value(open)),
            ("thickness", expr_value(thickness)),
        ]),
        Op::Draft { faces, neutral, degrees } => {
            let mut items = vec![("type", Value::str("draft")), ("faces", face_list_value(faces))];
            match neutral {
                PlaneRef::Given { point, normal } => {
                    items.push(("pull", point3_value(normal)));
                    items.push(("neutral", point3_value(point)));
                }
                plane => items.push(("neutral", plane_ref_value(plane))),
            }
            items.push(("degrees", expr_value(degrees)));
            Value::obj(items)
        }
        Op::Fillet { edges, radius, propagate } => Value::obj(vec![
            ("type", Value::str("fillet")),
            ("edges", edge_select_value(edges)),
            ("radius", expr_value(radius)),
            ("propagate", Value::Bool(*propagate)),
        ]),
        Op::FilletRadii { sets } => Value::obj(vec![
            ("type", Value::str("fillet_radii")),
            (
                "sets",
                Value::Arr(
                    sets.iter()
                        .map(|set| {
                            let mut items = vec![("edges", edge_select_value(&set.edges))];
                            match &set.radius {
                                RadiusLaw::Constant(r) => items.push(("radius", expr_value(r))),
                                RadiusLaw::Along { stations, start } => {
                                    let list = stations.iter().map(|(at, r)| Value::obj(vec![("at", expr_value(at)), ("radius", expr_value(r))])).collect();
                                    items.push(("along", Value::Arr(list)));
                                    if let Some(p) = start {
                                        items.push(("start", point3_value(p)));
                                    }
                                }
                            }
                            Value::obj(items)
                        })
                        .collect(),
                ),
            ),
        ]),
        Op::Chamfer { edges, distance, propagate } => Value::obj(vec![
            ("type", Value::str("chamfer")),
            ("edges", edge_select_value(edges)),
            ("distance", expr_value(distance)),
            ("propagate", Value::Bool(*propagate)),
        ]),
        Op::Plane { def, flip } => planes::plane_value(def, *flip),
    }
}

fn face_list_value(list: &[FaceRef]) -> Value {
    Value::Arr(list.iter().map(|f| Value::str(f.name())).collect())
}

/// A sweep's path: a list of steps, `{"sketch": 5}` or `{"edges": selection}`.
fn sweep_path_value(path: &SweepPath) -> Value {
    match path {
        SweepPath::Steps(steps) => Value::Arr(
            steps
                .iter()
                .map(|step| match step {
                    SweepStep::Line { length } => Value::obj(vec![("line", expr_value(length))]),
                    SweepStep::Arc { radius, degrees, toward_degrees } => {
                        Value::obj(vec![("arc_radius", expr_value(radius)), ("degrees", expr_value(degrees)), ("toward_degrees", expr_value(toward_degrees))])
                    }
                })
                .collect(),
        ),
        SweepPath::Sketch(id) => Value::obj(vec![("sketch", Value::Num(id.0 as f64))]),
        SweepPath::Edges(edges) => Value::obj(vec![("edges", edge_select_value(edges))]),
    }
}

fn key_list_value(list: &[FaceKey]) -> Value {
    Value::Arr(list.iter().map(|f| Value::str(f.text())).collect())
}

fn read_key_list(v: &Value, key: &str) -> Result<Vec<FaceKey>> {
    let bad = || Error::InvalidInput(format!("\"{key}\" must be a list of face names or keys"));
    match v.get(key) {
        None => Ok(Vec::new()),
        Some(list) => list.as_array().ok_or_else(bad)?.iter().map(|n| FaceKey::parse(n.as_str().ok_or_else(bad)?)).collect(),
    }
}

fn read_face_list(v: &Value, key: &str) -> Result<Vec<FaceRef>> {
    let bad = || Error::InvalidInput(format!("\"{key}\" must be a list of face names"));
    match v.get(key) {
        None => Ok(Vec::new()),
        Some(list) => list
            .as_array()
            .ok_or_else(bad)?
            .iter()
            .map(|n| {
                let name = n.as_str().ok_or_else(bad)?;
                if name.contains(['@', '[', '#']) {
                    return Err(Error::InvalidInput(format!("\"{key}\" takes plain face names for now, so it can't take the key {name}")));
                }
                FaceRef::parse(name)
            })
            .collect(),
    }
}

fn ids_value(ids: &[FeatureId]) -> Value {
    Value::Arr(ids.iter().map(|f| Value::Num(f.0 as f64)).collect())
}

fn read_ids(v: &Value) -> Result<Vec<FeatureId>> {
    let bad = || Error::InvalidInput("\"of\" must be a list of feature ids; leave it out to copy the whole body".to_string());
    match v.get("of") {
        None => Ok(Vec::new()),
        Some(list) => list
            .as_array()
            .ok_or_else(bad)?
            .iter()
            .map(|n| n.as_f64().filter(|x| x.fract() == 0.0 && *x >= 1.0 && *x < 4e9).map(|x| FeatureId(x as u32)).ok_or_else(bad))
            .collect(),
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
        "constrained_sketch" => {
            with(&["type", "plane", "sketch"])?;
            let plane = match v.get("plane") {
                Some(p) => read_placement(p)?,
                None => Placement::xy(0.0),
            };
            let sketch = coreshape_sketch::Sketch::from_json(field(v, "sketch", "a constrained sketch")?)
                .map_err(|e| Error::InvalidInput(e.message))?;
            for e in sketch.projected() {
                read_projection(&e.projection.as_ref().expect("projected").from).map_err(|why| Error::InvalidInput(format!("projected entity {}: {why}", e.id.0)))?;
            }
            Ok(Op::ConstrainedSketch { plane, sketch })
        }
        "extrude" => {
            with(&["type", "sketch", "start", "length", "mode", "regions", "faces", "end", "opposite"])?;
            let named = v.get("sketch").map(|_| read_sketch_id(v)).transpose()?;
            let regions = read_region_picks(v, named, "an extrude")?;
            let faces = read_key_list(v, "faces")?;
            let sketch = match (named, regions.first()) {
                (Some(id), _) => Some(id),
                (None, Some(first)) => Some(first.sketch),
                (None, None) if !faces.is_empty() => None,
                (None, None) => return Err(Error::InvalidInput("an extrude needs a \"sketch\", \"regions\" or \"faces\"".into())),
            };
            let end = read_extrude_end(v)?;
            let length = match (&end, v.get("length")) {
                (ExtrudeEnd::Blind, _) => expr_field(v, "length", "an extrude")?,
                (_, Some(length)) => read_expr(length)?,
                (_, None) => Expr::number(0.0),
            };
            let opposite = match v.get("opposite") {
                None => false,
                Some(b) => b.as_bool().ok_or_else(|| Error::InvalidInput("an extrude's \"opposite\" must be true or false".into()))?,
            };
            Ok(Op::Extrude { sketch, start: v.get("start").map(read_expr).transpose()?.unwrap_or(Expr::number(0.0)), length, mode: read_mode(v)?, regions, faces, end, opposite })
        }
        "revolve" => {
            with(&["type", "sketch", "degrees", "mode", "axis", "regions"])?;
            let named = v.get("sketch").map(|_| read_sketch_id(v)).transpose()?;
            let regions = read_region_picks(v, named, "a revolve")?;
            let sketch = match (named, regions.first()) {
                (Some(id), _) => id,
                (None, Some(first)) => first.sketch,
                (None, None) => read_sketch_id(v)?,
            };
            let axis = match v.get("axis") {
                None => None,
                Some(a) => {
                    known_keys(a, &["point", "direction"])?;
                    let need = |key: &str| a.get(key).ok_or_else(|| Error::InvalidInput(format!("a revolve's axis needs a \"{key}\"")));
                    Some(RevolveAxis { point: read_point2(need("point")?)?, direction: read_point2(need("direction")?)? })
                }
            };
            Ok(Op::Revolve { sketch, degrees: v.get("degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(360.0)), mode: read_mode(v)?, axis, regions })
        }
        "transform" => {
            with(&["type", "motion", "copy"])?;
            let m = field(v, "motion", "a transform")?;
            let at = |key: &str, what: &str| -> Result<Point3> { read_point3(field(m, key, what)?) };
            let motion = match field(m, "type", "a transform's motion")?.as_str().unwrap_or("") {
                "translate" => {
                    known_keys(m, &["type", "by"])?;
                    Motion::Translate { by: at("by", "a translation")? }
                }
                "rotate" => {
                    known_keys(m, &["type", "point", "axis", "degrees"])?;
                    Motion::Rotate { point: at("point", "a rotation")?, axis: at("axis", "a rotation")?, degrees: expr_field(m, "degrees", "a rotation")? }
                }
                "frames" => {
                    known_keys(m, &["type", "from", "to"])?;
                    Motion::Frames { from: read_placement(field(m, "from", "a transform by frames")?)?, to: read_placement(field(m, "to", "a transform by frames")?)? }
                }
                other => return Err(Error::InvalidInput(format!("unknown motion \"{other}\"; expected translate, rotate or frames"))),
            };
            let copy = match v.get("copy") {
                None => false,
                Some(c) => c.as_bool().ok_or_else(|| Error::InvalidInput("\"copy\" must be true or false".into()))?,
            };
            Ok(Op::Transform { motion, copy })
        }
        "delete_bodies" => {
            with(&["type"])?;
            Ok(Op::DeleteBodies)
        }
        "boolean" => {
            with(&["type", "operation", "tools", "keep_tools"])?;
            let operation = match field(v, "operation", "a boolean")?.as_str().unwrap_or("") {
                "union" => BodyBoolean::Union,
                "subtract" => BodyBoolean::Subtract,
                "intersect" => BodyBoolean::Intersect,
                other => return Err(Error::InvalidInput(format!("unknown Boolean \"{other}\"; expected union, subtract or intersect"))),
            };
            let keep_tools = match v.get("keep_tools") {
                None => false,
                Some(c) => c.as_bool().ok_or_else(|| Error::InvalidInput("\"keep_tools\" must be true or false".into()))?,
            };
            Ok(Op::BooleanBodies { operation, tools: read_body_list(v, "tools")?, keep_tools })
        }
        "import_step" => {
            with(&["type", "text", "mode"])?;
            let text = field(v, "text", "an imported file")?
                .as_str()
                .ok_or_else(|| Error::InvalidInput("an imported file's \"text\" must be the STEP file as text".into()))?;
            Ok(Op::ImportStep { text: text.to_string(), mode: read_mode(v)? })
        }
        "import_step_parts" => {
            with(&["type", "text"])?;
            let text = field(v, "text", "imported parts")?
                .as_str()
                .ok_or_else(|| Error::InvalidInput("imported parts' \"text\" must be the STEP file as text".into()))?;
            Ok(Op::ImportStepParts { text: text.to_string() })
        }
        "fill" => {
            with(&["type", "sketch"])?;
            Ok(Op::Fill { sketch: read_sketch_id(v)? })
        }
        "copy_faces" => {
            with(&["type", "faces"])?;
            Ok(Op::CopyFaces { faces: read_key_list(v, "faces")? })
        }
        "sew" => {
            with(&["type", "tolerance"])?;
            Ok(Op::Sew { tolerance: v.get("tolerance").map(read_expr).transpose()? })
        }
        "trim" => {
            with(&["type", "tool", "point", "normal", "plane", "sketch", "keep"])?;
            let tool = match (v.get("tool"), v.get("point"), v.get("sketch"), v.get("plane")) {
                (Some(t), None, None, None) => {
                    let name = t.as_str().ok_or_else(|| Error::InvalidInput("a trim's \"tool\" must be the name of a body, like f3 or f3.2".into()))?;
                    TrimTool::Body(BodyId::parse(name)?)
                }
                (None, Some(p), None, None) => TrimTool::Plane(PlaneRef::Given { point: read_point3(p)?, normal: read_point3(field(v, "normal", "a trim by a plane")?)? }),
                (None, None, Some(_), None) => TrimTool::Sketch(read_sketch_id(v)?),
                (None, None, None, Some(p)) if v.get("normal").is_none() => TrimTool::Plane(read_plane_ref(p)?),
                _ => return Err(Error::InvalidInput("a trim takes one of \"tool\", a body, \"plane\", or \"point\" and \"normal\", a plane, or \"sketch\"".into())),
            };
            let keep = match v.get("keep").and_then(Value::as_str) {
                None | Some("outside") => TrimKeep::Outside,
                Some("inside") => TrimKeep::Inside,
                Some("both") => TrimKeep::Both,
                Some(other) => return Err(Error::InvalidInput(format!("unknown \"keep\" of \"{other}\"; expected outside, inside or both"))),
            };
            Ok(Op::Trim { tool, keep })
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
        "loft" => {
            with(&["type", "sections", "mode"])?;
            let bad = || Error::InvalidInput("\"sections\" must be a list of sketch ids, or of {\"sketch\": 4, \"region\": \"r...\"}".into());
            let sections = field(v, "sections", "a loft")?
                .as_array()
                .ok_or_else(bad)?
                .iter()
                .map(|x| match x.as_f64() {
                    Some(n) => Some(n).filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < u32::MAX as f64).map(|n| LoftSection::from(FeatureId(n as u32))).ok_or_else(bad),
                    None if x.get("region").is_some() => read_region_pick(x).map(LoftSection::from),
                    None => Err(bad()),
                })
                .collect::<Result<Vec<_>>>()?;
            Ok(Op::Loft { sections, mode: read_mode(v)? })
        }
        "sweep" => {
            with(&["type", "sketch", "path", "mode"])?;
            let given = field(v, "path", "a sweep")?;
            if given.as_array().is_none() {
                let path = if given.get("sketch").is_some() {
                    known_keys(given, &["sketch"])?;
                    SweepPath::Sketch(read_sketch_id(given)?)
                } else if let Some(edges) = given.get("edges") {
                    known_keys(given, &["edges"])?;
                    SweepPath::Edges(read_edge_select(edges)?)
                } else {
                    return Err(Error::InvalidInput("a sweep's path is a list of steps, {\"sketch\": 5} or {\"edges\": selection}".into()));
                };
                return Ok(Op::Sweep { sketch: read_sketch_id(v)?, path, mode: read_mode(v)? });
            }
            let mut path = Vec::new();
            for step in given.as_array().unwrap_or_default() {
                if let Some(length) = step.get("line") {
                    known_keys(step, &["line"])?;
                    path.push(SweepStep::Line { length: read_expr(length)? });
                } else if step.get("arc_radius").is_some() {
                    known_keys(step, &["arc_radius", "degrees", "toward_degrees"])?;
                    path.push(SweepStep::Arc {
                        radius: expr_field(step, "arc_radius", "a bend")?,
                        degrees: expr_field(step, "degrees", "a bend")?,
                        toward_degrees: step.get("toward_degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(0.0)),
                    });
                } else {
                    return Err(Error::InvalidInput("a sweep step is {\"line\": length} or {\"arc_radius\": r, \"degrees\": a, \"toward_degrees\": d}".into()));
                }
            }
            Ok(Op::Sweep { sketch: read_sketch_id(v)?, path: SweepPath::Steps(path), mode: read_mode(v)? })
        }
        "pattern" => {
            let of = read_ids(v)?;
            let along = if v.get("direction").is_some() {
                with(&["type", "of", "direction", "count", "spacing"])?;
                let direction = field(v, "direction", "a linear pattern")?;
                PatternPath::Linear {
                    direction: if direction.as_array().is_some() { LineRef::along(read_point3(direction)?) } else { read_line_ref(direction)? },
                    count: expr_field(v, "count", "a pattern")?,
                    spacing: expr_field(v, "spacing", "a linear pattern")?,
                }
            } else {
                with(&["type", "of", "center", "axis", "count", "degrees"])?;
                let axis = match v.get("axis") {
                    Some(line) if line.as_array().is_none() => {
                        if v.get("center").is_some() {
                            return Err(Error::InvalidInput("a circular pattern round a line takes its center from the line, so it has no \"center\"".into()));
                        }
                        read_line_ref(line)?
                    }
                    axis => LineRef::Given {
                        point: v.get("center").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
                        direction: axis.map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 1.0)),
                    },
                };
                PatternPath::Circular { axis, count: expr_field(v, "count", "a pattern")?, degrees: v.get("degrees").map(read_expr).transpose()?.unwrap_or(Expr::number(360.0)) }
            };
            Ok(Op::Pattern { of, along })
        }
        "mirror" => {
            with(&["type", "of", "point", "normal", "plane"])?;
            let plane = match v.get("plane") {
                Some(_) if v.get("point").is_some() || v.get("normal").is_some() => {
                    return Err(Error::InvalidInput("a mirror takes \"plane\", or \"point\" and \"normal\", not both".into()))
                }
                Some(plane) => read_plane_ref(plane)?,
                None => PlaneRef::Given {
                    point: v.get("point").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
                    normal: read_point3(field(v, "normal", "a mirror")?)?,
                },
            };
            Ok(Op::Mirror { of: read_ids(v)?, plane })
        }
        "helix" => {
            with(&["type", "sketch", "point", "axis", "pitch", "turns", "left_handed", "mode"])?;
            let sketch = field(v, "sketch", "a helix")?
                .as_f64()
                .filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < u32::MAX as f64)
                .map(|n| FeatureId(n as u32))
                .ok_or_else(|| Error::InvalidInput("\"sketch\" must be a feature id".into()))?;
            Ok(Op::Helix {
                sketch,
                point: v.get("point").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
                axis: v.get("axis").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 1.0)),
                pitch: expr_field(v, "pitch", "a helix")?,
                turns: expr_field(v, "turns", "a helix")?,
                left_handed: match v.get("left_handed") {
                    None => false,
                    Some(b) => b.as_bool().ok_or_else(|| Error::InvalidInput("\"left_handed\" must be true or false".into()))?,
                },
                mode: read_mode(v)?,
            })
        }
        "delete_face" => {
            with(&["type", "faces"])?;
            Ok(Op::DeleteFace { faces: read_key_list(v, "faces")? })
        }
        "thicken" => {
            with(&["type", "faces", "thickness", "mode"])?;
            Ok(Op::Thicken { faces: read_key_list(v, "faces")?, thickness: expr_field(v, "thickness", "a thicken")?, mode: read_mode(v)? })
        }
        "offset" => {
            with(&["type", "distance"])?;
            Ok(Op::Offset { distance: expr_field(v, "distance", "an offset")? })
        }
        "shell" => {
            with(&["type", "open", "thickness"])?;
            Ok(Op::Shell { open: read_face_list(v, "open")?, thickness: expr_field(v, "thickness", "a shell")? })
        }
        "draft" => {
            with(&["type", "faces", "pull", "neutral", "degrees"])?;
            // A neutral plane, whose normal is the pull, or a point on it and the pull as numbers.
            let neutral = match v.get("neutral") {
                Some(plane) if plane.as_array().is_none() => {
                    if v.get("pull").is_some() {
                        return Err(Error::InvalidInput("with a neutral plane, the pull is its normal, so a draft has no \"pull\" as well. A negative angle drafts the other way".into()));
                    }
                    read_plane_ref(plane)?
                }
                point => PlaneRef::Given {
                    point: point.map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 0.0)),
                    normal: v.get("pull").map(read_point3).transpose()?.unwrap_or(point3(0.0, 0.0, 1.0)),
                },
            };
            Ok(Op::Draft { faces: read_face_list(v, "faces")?, neutral, degrees: expr_field(v, "degrees", "a draft")? })
        }
        "sheet_metal" => {
            with(&["type", "sketch", "thickness", "bend_radius", "k_factor", "depth", "flip", "minimal_gap", "corner"])?;
            Ok(Op::SheetMetal {
                sketch: read_sketch_id(v)?,
                thickness: expr_field(v, "thickness", "sheet metal")?,
                bend_radius: v.get("bend_radius").map(read_expr).transpose()?,
                k_factor: v.get("k_factor").map(read_expr).transpose()?,
                depth: v.get("depth").map(read_expr).transpose()?,
                flip: read_flip(v)?,
                minimal_gap: v.get("minimal_gap").map(read_expr).transpose()?,
                corner: read_corner(v, "sheet metal")?.unwrap_or(Corner::Open),
            })
        }
        "flange" => {
            with(&["type", "edges", "length", "degrees", "alignment", "bend_radius", "flip", "corner", "gap"])?;
            Ok(Op::Flange {
                edges: read_edge_select(field(v, "edges", "a flange")?)?,
                length: expr_field(v, "length", "a flange")?,
                degrees: v.get("degrees").map(read_expr).transpose()?,
                alignment: read_alignment(v, "a flange")?,
                bend_radius: v.get("bend_radius").map(read_expr).transpose()?,
                flip: read_flip(v)?,
                corner: read_corner(v, "a flange")?,
                gap: v.get("gap").map(read_expr).transpose()?,
            })
        }
        "hem" => {
            with(&["type", "edges", "kind", "length", "gap", "bend_radius", "degrees", "alignment", "flip"])?;
            let kind = field(v, "kind", "a hem")?
                .as_str()
                .and_then(HemKind::from_word)
                .ok_or_else(|| Error::InvalidInput("a hem's \"kind\" is \"straight\", \"rounded\", \"teardrop\" or \"rolled\"".into()))?;
            let op = Op::Hem {
                edges: read_edge_select(field(v, "edges", "a hem")?)?,
                kind,
                length: v.get("length").map(read_expr).transpose()?,
                gap: v.get("gap").map(read_expr).transpose()?,
                bend_radius: v.get("bend_radius").map(read_expr).transpose()?,
                degrees: v.get("degrees").map(read_expr).transpose()?,
                alignment: read_alignment(v, "a hem")?,
                flip: read_flip(v)?,
            };
            if let Op::Hem { kind, length, gap, bend_radius, degrees, .. } = &op {
                check_hem_fields(*kind, length.is_some(), gap.is_some(), bend_radius.is_some(), degrees.is_some())?;
            }
            Ok(op)
        }
        "tab" => {
            with(&["type", "sketch"])?;
            Ok(Op::Tab { sketch: read_sketch_id(v)? })
        }
        "bend" => {
            with(&["type", "sketch", "degrees", "bend_radius", "position", "flip", "moves"])?;
            let position = match v.get("position") {
                None => BendPosition::Center,
                Some(p) => p
                    .as_str()
                    .and_then(BendPosition::from_word)
                    .ok_or_else(|| Error::InvalidInput("a bend's \"position\" is \"center\" or \"start\"".into()))?,
            };
            let right = match v.get("moves").map(|m| m.as_str()) {
                None | Some(Some("left")) => false,
                Some(Some("right")) => true,
                Some(_) => return Err(Error::InvalidInput("what a bend \"moves\" is \"left\" or \"right\" of its line".into())),
            };
            Ok(Op::Bend {
                sketch: read_sketch_id(v)?,
                degrees: v.get("degrees").map(read_expr).transpose()?,
                bend_radius: v.get("bend_radius").map(read_expr).transpose()?,
                position,
                flip: read_flip(v)?,
                right,
            })
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
        "fillet_radii" => {
            with(&["type", "sets"])?;
            let list = field(v, "sets", "a fillet with several radii")?.as_array().ok_or_else(|| Error::InvalidInput("\"sets\" must be a list of edges and their radii".into()))?;
            if list.is_empty() {
                return Err(Error::InvalidInput("a fillet with several radii needs at least one set of edges".into()));
            }
            let mut sets = Vec::with_capacity(list.len());
            for set in list {
                let edges = read_edge_select(field(set, "edges", "a set of edges")?)?;
                let radius = match (set.get("radius"), set.get("along")) {
                    (Some(_), Some(_)) => return Err(Error::InvalidInput("a set of edges has a \"radius\" or an \"along\", not both".into())),
                    (Some(r), None) => RadiusLaw::Constant(read_expr(r)?),
                    (None, Some(along)) => {
                        let items = along.as_array().ok_or_else(|| Error::InvalidInput("\"along\" must be a list of places and radii".into()))?;
                        let mut stations = Vec::with_capacity(items.len());
                        for item in items {
                            stations.push((expr_field(item, "at", "a place along some edges")?, expr_field(item, "radius", "a place along some edges")?));
                        }
                        let start = set.get("start").map(read_point3).transpose()?;
                        RadiusLaw::Along { stations, start }
                    }
                    (None, None) => return Err(Error::InvalidInput("a set of edges needs a \"radius\" or an \"along\"".into())),
                };
                sets.push(RadiusSet { edges, radius });
            }
            Ok(Op::FilletRadii { sets })
        }
        "chamfer" => {
            with(&["type", "edges", "distance", "propagate"])?;
            Ok(Op::Chamfer {
                edges: read_edge_select(field(v, "edges", "a chamfer")?)?,
                distance: expr_field(v, "distance", "a chamfer")?,
                propagate: match v.get("propagate") {
                    None => true,
                    Some(p) => p.as_bool().ok_or_else(|| Error::InvalidInput("\"propagate\" must be true or false".into()))?,
                },
            })
        }
        "plane" => planes::read_plane(v, &with),
        other => Err(Error::InvalidInput(format!(
            "unknown feature type \"{other}\"; expected sketch, constrained_sketch, plane, extrude, revolve, sweep, solid, pattern, mirror, transform, delete_bodies, boolean, fillet, chamfer, offset, shell or draft"
        ))),
    }
}

fn feature_value(f: &Feature) -> Value {
    let mut items = vec![("id".to_string(), Value::Num(f.id.0 as f64)), ("name".to_string(), Value::str(&f.name))];
    if f.suppressed {
        items.push(("suppressed".to_string(), Value::Bool(true)));
    }
    if !f.scope.is_empty() {
        items.push(("bodies".to_string(), body_list_value(&f.scope)));
    }
    if let Value::Obj(op) = op_value(&f.op) {
        items.extend(op);
    }
    with_app(Value::Obj(items), f.app.as_ref())
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
    Ok(Feature { id: FeatureId(id as u32), name: name.to_string(), suppressed, scope: read_body_list(v, "bodies")?, app: v.get("app").cloned(), op: read_op(v, &["id", "name", "suppressed", "bodies", "app"])? })
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
