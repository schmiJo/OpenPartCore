//! Holes, as Onshape's hole feature makes them: at points of a sketch or of a plane, simple, counterbored or
//! countersunk, blind with a drill point, through everything, or through the first wall they meet, and tapped with a
//! thread that's noted and not modelled. A standard metric size fills in what isn't given.
//!
//! Each hole is a turned profile cut from the parts the feature acts on. It starts where its axis first meets them,
//! going in, and its top carries on back from there by its top radius, so it opens cleanly on a curved or slanted face.
//! With `from_sketch` it starts at the sketch's plane instead, and goes no further back.

use super::*;
use coreshape::boolean::classify::Index;
use coreshape_sketch::{Geometry, Ref};

/// Slots per hole: one for each face it can have.
pub(crate) const HOLE_PARTS: u32 = 8;
/// Holes are numbered below this, which keeps every hole's slots within a `u32`.
pub(crate) const MAX_HOLE_NUMBER: u32 = 1_000_000;

/// Which face of a hole, as its name says: `f9:bore.3` is the bore of hole 3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HoleFace {
    /// What's left of the hole's top, carried on back past where it starts, which only shows where something stands
    /// over the hole.
    Top,
    /// A counterbore's wall.
    Cbore,
    /// A counterbore's flat bottom.
    CboreFloor,
    /// A countersink's cone.
    Csink,
    /// The hole's wall at its own diameter.
    Bore,
    /// A blind hole's drill point.
    Tip,
    /// A blind hole's flat bottom, with a tip angle of 180.
    Floor,
}

impl HoleFace {
    pub const ALL: [HoleFace; 7] = [HoleFace::Top, HoleFace::Cbore, HoleFace::CboreFloor, HoleFace::Csink, HoleFace::Bore, HoleFace::Tip, HoleFace::Floor];

    pub fn name(self) -> &'static str {
        match self {
            HoleFace::Top => "top",
            HoleFace::Cbore => "cbore",
            HoleFace::CboreFloor => "cbore_floor",
            HoleFace::Csink => "csink",
            HoleFace::Bore => "bore",
            HoleFace::Tip => "tip",
            HoleFace::Floor => "floor",
        }
    }

    pub(crate) fn part(self) -> u32 {
        self as u32
    }

    pub(crate) fn from_part(part: u32) -> Option<HoleFace> {
        HoleFace::ALL.get(part as usize).copied()
    }

    pub(crate) fn parse(text: &str) -> Option<HoleFace> {
        HoleFace::ALL.into_iter().find(|f| f.name() == text)
    }
}

/// The profile's segment on the axis, which makes no face.
const AXIS: u32 = 7;

/// Where a hole feature's holes go.
#[derive(Clone, Debug, PartialEq)]
pub enum HoleAt {
    /// Points of a constrained sketch: those listed, as constraints name them (`7`, `9.center`, `4.start`), or, with
    /// none listed, every point entity in it. The holes go in against the sketch's normal. Holes are numbered by their
    /// point's entity id when it takes every point, so that one going doesn't renumber the rest, and otherwise by their
    /// place in the list.
    Sketch { sketch: FeatureId, points: Vec<Ref> },
    /// Points on a plane, in its coordinates. The holes go in against its normal: into the part, for a face's plane.
    /// They're numbered by their place in the list.
    Plane { plane: PlaneRef, points: Vec<Point2> },
}

/// What's at a hole's top.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HoleStyle {
    Simple,
    Counterbore,
    Countersink,
}

/// How much room a clearance hole leaves round its screw: ISO 273's fine, medium and coarse series.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fit {
    Close,
    Normal,
    Loose,
}

/// A tapped hole's thread, which is noted and not modelled: the hole is drilled at the tap drill's diameter.
#[derive(Clone, Debug, PartialEq)]
pub struct Tapped {
    /// The thread's pitch, by default the size's coarse pitch.
    pub pitch: Option<Expr>,
    /// How deep the thread goes, by default all the way: the hole's depth, or through.
    pub depth: Option<Expr>,
}

/// How deep holes go.
#[derive(Clone, Debug, PartialEq)]
pub enum HoleEnd {
    /// To a depth from where they start, to the end of the full diameter: a drill point goes on past it.
    Blind(Expr),
    /// Through everything the feature acts on.
    ThroughAll,
    /// Through the first wall they meet, and no further.
    UpToNext,
}

/// What a hole is like. A `size` such as "M6" fills in the dimensions not given, from the tables below; without one,
/// `diameter` and a counterbore's or countersink's dimensions have to be given.
#[derive(Clone, Debug, PartialEq)]
pub struct HoleSpec {
    pub style: HoleStyle,
    /// A metric screw size, `M1.6` to `M36`.
    pub size: Option<String>,
    /// For a clearance hole of a size, how loose.
    pub fit: Fit,
    pub tapped: Option<Tapped>,
    pub diameter: Option<Expr>,
    pub cbore_diameter: Option<Expr>,
    pub cbore_depth: Option<Expr>,
    pub csink_diameter: Option<Expr>,
    /// The countersink's included angle, by default 90 degrees.
    pub csink_angle: Option<Expr>,
    /// A blind hole's drill point, as its included angle: by default 118 degrees, and 180 for a flat bottom.
    pub tip_angle: Option<Expr>,
}

impl HoleSpec {
    /// A plain hole of a diameter.
    pub fn simple(diameter: impl Into<Expr>) -> HoleSpec {
        HoleSpec { diameter: Some(diameter.into()), ..HoleSpec::sized(HoleStyle::Simple, "") }
    }

    /// A clearance hole for a metric screw, at a normal fit, with what's at its top from the size's tables.
    pub fn sized(style: HoleStyle, size: &str) -> HoleSpec {
        HoleSpec {
            style,
            size: (!size.is_empty()).then(|| size.to_string()),
            fit: Fit::Normal,
            tapped: None,
            diameter: None,
            cbore_diameter: None,
            cbore_depth: None,
            csink_diameter: None,
            csink_angle: None,
            tip_angle: None,
        }
    }

    /// A tapped hole for a metric thread at its coarse pitch, threaded all the way.
    pub fn tapped(size: &str) -> HoleSpec {
        HoleSpec { tapped: Some(Tapped { pitch: None, depth: None }), ..HoleSpec::sized(HoleStyle::Simple, size) }
    }

    pub(crate) fn exprs<'a>(&'a self, out: &mut Vec<&'a Expr>) {
        out.extend(self.diameter.iter());
        out.extend(self.cbore_diameter.iter());
        out.extend(self.cbore_depth.iter());
        out.extend(self.csink_diameter.iter());
        out.extend(self.csink_angle.iter());
        out.extend(self.tip_angle.iter());
        if let Some(t) = &self.tapped {
            out.extend(t.pitch.iter());
            out.extend(t.depth.iter());
        }
    }
}

/// A metric screw size's numbers, in millimetres.
struct Size {
    name: &'static str,
    major: f64,
    /// The coarse pitch, ISO 261.
    pitch: f64,
    /// The usual tap drill for the coarse pitch, about the major diameter less the pitch.
    tap_drill: f64,
    /// Clearance holes, ISO 273: fine, medium and coarse.
    clearance: [f64; 3],
    /// A counterbore for an ISO 4762 socket head cap screw, as DIN 974-1 gives it. Its depth is the head's height,
    /// which for ISO 4762 is the major diameter, so the head sits flush.
    cbore: f64,
    /// A 90 degree countersink for an ISO 10642 countersunk socket head, at the head's theoretical diameter, so the head
    /// sits flush.
    csink: Option<f64>,
}

const fn size(name: &'static str, major: f64, pitch: f64, tap_drill: f64, clearance: [f64; 3], cbore: f64, csink: Option<f64>) -> Size {
    Size { name, major, pitch, tap_drill, clearance, cbore, csink }
}

const SIZES: [Size; 16] = [
    size("M1.6", 1.6, 0.35, 1.25, [1.7, 1.8, 2.0], 3.5, None),
    size("M2", 2.0, 0.4, 1.6, [2.2, 2.4, 2.6], 4.4, None),
    size("M2.5", 2.5, 0.45, 2.05, [2.7, 2.9, 3.1], 5.5, None),
    size("M3", 3.0, 0.5, 2.5, [3.2, 3.4, 3.6], 6.5, Some(6.72)),
    size("M4", 4.0, 0.7, 3.3, [4.3, 4.5, 4.8], 8.0, Some(8.96)),
    size("M5", 5.0, 0.8, 4.2, [5.3, 5.5, 5.8], 10.0, Some(11.2)),
    size("M6", 6.0, 1.0, 5.0, [6.4, 6.6, 7.0], 11.0, Some(13.44)),
    size("M8", 8.0, 1.25, 6.8, [8.4, 9.0, 10.0], 15.0, Some(17.92)),
    size("M10", 10.0, 1.5, 8.5, [10.5, 11.0, 12.0], 18.0, Some(22.4)),
    size("M12", 12.0, 1.75, 10.2, [13.0, 13.5, 14.5], 20.0, Some(26.88)),
    size("M14", 14.0, 2.0, 12.0, [15.0, 15.5, 16.5], 24.0, None),
    size("M16", 16.0, 2.0, 14.0, [17.0, 17.5, 18.5], 26.0, Some(33.6)),
    size("M20", 20.0, 2.5, 17.5, [21.0, 22.0, 24.0], 33.0, Some(40.32)),
    size("M24", 24.0, 3.0, 21.0, [25.0, 26.0, 28.0], 40.0, None),
    size("M30", 30.0, 3.5, 26.5, [31.0, 33.0, 35.0], 50.0, None),
    size("M36", 36.0, 4.0, 32.0, [37.0, 39.0, 42.0], 58.0, None),
];

/// The sizes the tables know, smallest first.
pub fn hole_sizes() -> Vec<&'static str> {
    SIZES.iter().map(|s| s.name).collect()
}

fn find_size(name: &str) -> Result<&'static Size> {
    let wanted = name.trim();
    SIZES.iter().find(|s| s.name.eq_ignore_ascii_case(wanted)).ok_or_else(|| {
        Error::InvalidInput(format!("\"{name}\" isn't a size the hole tables know: they're {}", hole_sizes().join(", ")))
    })
}

/// A hole's thread, as noted.
#[derive(Clone, Debug, PartialEq)]
pub struct ThreadCallout {
    pub size: String,
    pub major: f64,
    pub pitch: f64,
    /// How deep it goes from where the hole starts, or `None` all the way through.
    pub depth: Option<f64>,
}

/// One hole as made, for showing its callout and its thread.
#[derive(Clone, Debug, PartialEq)]
pub struct HoleCallout {
    pub feature: FeatureId,
    pub number: u32,
    /// Where it starts, on its axis.
    pub start: Vec3,
    /// The way it goes in.
    pub direction: Vec3,
    /// What a drawing would say, such as `⌀6.6 through, counterbore ⌀11 6 deep`.
    pub text: String,
    pub thread: Option<ThreadCallout>,
}

/// What's at a hole's top, worked out.
#[derive(Clone, Copy, Debug)]
enum Top {
    Plain,
    Cbore { radius: f64, depth: f64 },
    /// `half` is half the included angle.
    Csink { radius: f64, half: f64 },
}

/// A hole spec as numbers.
struct Dims {
    radius: f64,
    top: Top,
    /// Half the drill point's included angle, or `None` for a flat bottom.
    tip: Option<f64>,
    thread: Option<(String, f64, f64, Option<f64>)>,
    /// How the callout starts, before the depth.
    words: String,
}

fn positive(e: &Option<Expr>, what: &str, env: &Env) -> Result<Option<f64>> {
    let Some(e) = e else { return Ok(None) };
    let x = num(e, env)?;
    if !(x >= MIN_FEATURE && x <= MAX_COORD) {
        return Err(Error::InvalidInput(format!("a hole's {what} has to be at least {MIN_FEATURE}, and {x} isn't")));
    }
    Ok(Some(x))
}

fn angle(e: &Option<Expr>, default: f64, what: &str, top: f64, env: &Env) -> Result<f64> {
    let a = e.as_ref().map(|e| num(e, env)).transpose()?.unwrap_or(default);
    if !(a > 1.0 && a <= top) {
        return Err(Error::InvalidInput(format!("a hole's {what} is an angle in degrees above 1 and at most {top}, and {a} isn't")));
    }
    Ok(a)
}

impl HoleSpec {
    fn dims(&self, env: &Env) -> Result<Dims> {
        let size = self.size.as_deref().filter(|s| !s.trim().is_empty()).map(find_size).transpose()?;
        if self.style != HoleStyle::Counterbore && (self.cbore_diameter.is_some() || self.cbore_depth.is_some()) {
            return Err(Error::InvalidInput("cbore_diameter and cbore_depth are for a counterbored hole".into()));
        }
        if self.style != HoleStyle::Countersink && (self.csink_diameter.is_some() || self.csink_angle.is_some()) {
            return Err(Error::InvalidInput("csink_diameter and csink_angle are for a countersunk hole".into()));
        }
        let mut thread = None;
        let given = positive(&self.diameter, "diameter", env)?;
        let mut words = String::new();
        let diameter = match (&self.tapped, size) {
            (Some(tapped), Some(s)) => {
                let pitch = match &tapped.pitch {
                    Some(p) => num(p, env)?,
                    None => s.pitch,
                };
                if !(pitch >= 0.1 && pitch < s.major / 2.0) {
                    return Err(Error::InvalidInput(format!("{}'s thread can't have a pitch of {pitch}", s.name)));
                }
                // A fine pitch takes a bigger drill: the major diameter less the pitch.
                let drill = given.unwrap_or(if tapped.pitch.is_none() { s.tap_drill } else { s.major - pitch });
                if drill >= s.major {
                    return Err(Error::InvalidInput(format!("a tapped {} hole has to be drilled under {}, and {drill} isn't", s.name, up_to::short(s.major))));
                }
                let depth = positive(&tapped.depth, "thread depth", env)?;
                thread = Some((s.name.to_string(), s.major, pitch, depth));
                words.push_str(&format!("{}×{} tapped", s.name, up_to::short(pitch)));
                drill
            }
            (Some(_), None) => return Err(Error::InvalidInput("a tapped hole needs a size, like M6".into())),
            (None, Some(s)) => {
                let fit = match self.fit {
                    Fit::Close => 0,
                    Fit::Normal => 1,
                    Fit::Loose => 2,
                };
                let d = given.unwrap_or(s.clearance[fit]);
                words.push_str(&format!("{} clearance, ", s.name));
                d
            }
            (None, None) => given.ok_or_else(|| Error::InvalidInput("a hole needs a diameter or a size, like M6".into()))?,
        };
        let radius = diameter / 2.0;
        let top = match self.style {
            HoleStyle::Simple => Top::Plain,
            HoleStyle::Counterbore => {
                let d = positive(&self.cbore_diameter, "counterbore diameter", env)?
                    .or(size.map(|s| s.cbore))
                    .ok_or_else(|| Error::InvalidInput("a counterbore needs cbore_diameter and cbore_depth, or a size".into()))?;
                let depth = positive(&self.cbore_depth, "counterbore depth", env)?
                    .or(size.map(|s| s.major))
                    .ok_or_else(|| Error::InvalidInput("a counterbore needs cbore_diameter and cbore_depth, or a size".into()))?;
                if d / 2.0 <= radius + MIN_FEATURE {
                    return Err(Error::InvalidInput(format!("a counterbore has to be wider than its hole, and ⌀{} isn't wider than ⌀{}", up_to::short(d), up_to::short(diameter))));
                }
                Top::Cbore { radius: d / 2.0, depth }
            }
            HoleStyle::Countersink => {
                let d = match positive(&self.csink_diameter, "countersink diameter", env)? {
                    Some(d) => d,
                    None => match size {
                        Some(s) => s.csink.ok_or_else(|| Error::InvalidInput(format!("the tables have no countersink for {}: give csink_diameter", s.name)))?,
                        None => return Err(Error::InvalidInput("a countersink needs csink_diameter, or a size".into())),
                    },
                };
                let a = angle(&self.csink_angle, 90.0, "countersink angle", 179.0, env)?;
                if d / 2.0 <= radius + MIN_FEATURE {
                    return Err(Error::InvalidInput(format!("a countersink has to be wider than its hole, and ⌀{} isn't wider than ⌀{}", up_to::short(d), up_to::short(diameter))));
                }
                Top::Csink { radius: d / 2.0, half: (a / 2.0).to_radians() }
            }
        };
        let tip = angle(&self.tip_angle, 118.0, "tip angle", 180.0, env)?;
        let tip = (tip < 180.0 - 1e-9).then(|| (tip / 2.0).to_radians());
        Ok(Dims { radius, top, tip, thread, words })
    }
}

/// How far a ray has to go before a crossing counts as ahead of where it starts.
const AHEAD: f64 = 1e-6;

/// Where the line through `q` along `d` is inside any of these bodies, as spans of distance from `q`, which can be
/// behind it. `None` if that can't be told, as when the line runs along a free-form face.
fn spans(indexes: &[Index], q: Vec3, d: Vec3, reach: f64) -> Option<Vec<(f64, f64)>> {
    let mut all: Vec<(f64, f64)> = Vec::new();
    for index in indexes {
        let from = q - d * reach;
        let hits = index.ray_hits(from, d, reach * 2.0)?;
        // In and out in turn. Two crossings at one place, as on an edge between faces, count once.
        let mut inside: Option<f64> = None;
        for h in hits {
            let t = h.t - reach;
            match (h.entering, inside) {
                (true, None) => inside = Some(t),
                (false, Some(a)) => {
                    all.push((a, t));
                    inside = None;
                }
                _ => {}
            }
        }
    }
    all.sort_by(|a, b| a.0.total_cmp(&b.0));
    // Bodies that overlap make one span.
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (a, b) in all {
        match out.last_mut() {
            Some(last) if a <= last.1 + TOL => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    Some(out)
}

/// Where to say a hole is.
fn place(p: Vec3) -> String {
    format!("({}, {}, {})", up_to::short(p.x), up_to::short(p.y), up_to::short(p.z))
}

/// The cutters for a hole feature's holes, and what each hole is.
pub(super) struct Drilled {
    pub tools: Vec<Body>,
    pub callouts: Vec<HoleCallout>,
    pub notes: Vec<String>,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn drill(
    doc: &Document,
    look: &Looker,
    at: &HoleAt,
    spec: &HoleSpec,
    end: &HoleEnd,
    from_sketch: bool,
    opposite: bool,
    scope: &[(BodyId, Arc<Body>)],
    env: &Env,
    feature: FeatureId,
) -> Result<Drilled> {
    if scope.is_empty() {
        return Err(Error::InvalidInput("a hole needs a part to go in".into()));
    }
    let dims = spec.dims(env)?;
    let (frame, points): (Frame, Vec<(u32, [f64; 2])>) = match at {
        HoleAt::Sketch { sketch, points } => {
            let (frame, solved) = doc.solved_sketch(*sketch)?;
            let list: Vec<(u32, [f64; 2])> = if points.is_empty() {
                let mut all: Vec<(u32, [f64; 2])> = solved
                    .entities
                    .iter()
                    .filter_map(|e| match e.geometry {
                        Geometry::Point { at } => Some((e.id.0, at)),
                        _ => None,
                    })
                    .collect();
                all.sort_by_key(|(id, _)| *id);
                if all.is_empty() {
                    return Err(Error::InvalidInput(format!("sketch f{} has no points for holes to go at: draw some, or name the points to use", sketch.0)));
                }
                all
            } else {
                points
                    .iter()
                    .enumerate()
                    .map(|(k, r)| {
                        planes::sketch_point(&solved, *r)
                            .map(|q| (k as u32, q))
                            .map_err(|why| Error::InvalidInput(format!("sketch f{}'s point {}: {why}", sketch.0, r.name())))
                    })
                    .collect::<Result<_>>()?
            };
            (frame, list)
        }
        HoleAt::Plane { plane, points } => {
            if points.is_empty() {
                return Err(Error::InvalidInput("holes on a plane need points to go at".into()));
            }
            let frame = look.plane(plane)?;
            let list = points.iter().enumerate().map(|(k, p)| Ok((k as u32, [num(&p[0], env)?, num(&p[1], env)?]))).collect::<Result<_>>()?;
            (frame, list)
        }
    };
    if let Some((n, _)) = points.iter().find(|(n, _)| *n >= MAX_HOLE_NUMBER) {
        return Err(Error::InvalidInput(format!("holes are numbered below {MAX_HOLE_NUMBER}, and point {n} isn't")));
    }
    let d = if opposite { frame.z } else { -frame.z };
    let indexes: Vec<Index> = scope.iter().map(|(_, b)| Index::new(b)).collect();
    let mut bounds = scope[0].1.aabb();
    for (_, b) in &scope[1..] {
        bounds.union(&b.aabb());
    }
    let top_radius = match dims.top {
        Top::Plain => dims.radius,
        Top::Cbore { radius, .. } | Top::Csink { radius, .. } => radius,
    };
    let mut out = Drilled { tools: Vec::new(), callouts: Vec::new(), notes: Vec::new() };
    let mut done: Vec<(u32, Vec3)> = Vec::new();
    for (n, xy) in points {
        let q = frame.point(xy[0], xy[1], 0.0);
        if let Some((m, _)) = done.iter().find(|(_, p)| p.dist(q) <= TOL) {
            out.notes.push(format!("holes {m} and {n} are at the same place, {}, so there's one hole there", place(q)));
            continue;
        }
        done.push((n, q));
        let reach = q.dist(bounds.center()) + bounds.diagonal() + 1.0;
        let unclear = || Error::InvalidInput(format!("hole {n} at {}: it can't be told where its axis goes in and out of the part", place(q)));
        let line = spans(&indexes, q, d, reach).ok_or_else(unclear)?;
        // Where it starts: on the sketch, or where its axis goes into the part, or comes out of it behind the sketch.
        let start = if from_sketch {
            0.0
        } else {
            match line.iter().find(|(_, b)| *b > AHEAD) {
                Some((a, _)) => *a,
                None => {
                    return Err(Error::InvalidInput(format!("hole {n} at {} never meets the part going that way: \"opposite\" goes the other way", place(q))));
                }
            }
        };
        let s = q + d * start;
        let back = if from_sketch { 0.0 } else { top_radius };
        // How deep its full diameter goes from `s`.
        let (depth, through) = match end {
            HoleEnd::Blind(e) => {
                let h = num(e, env)?;
                if !(h >= MIN_FEATURE && h <= MAX_COORD) {
                    return Err(Error::InvalidInput(format!("a hole's depth has to be at least {MIN_FEATURE}, and {h} isn't")));
                }
                (h, false)
            }
            HoleEnd::ThroughAll => {
                let mut far = f64::NEG_INFINITY;
                for (_, body) in scope {
                    far = far.max(up_to::furthest(body, s, d)?);
                }
                if far <= MIN_FEATURE {
                    return Err(Error::InvalidInput(format!("hole {n} at {} has nothing ahead of it to go through: \"opposite\" goes the other way", place(s))));
                }
                (far * 1.01 + 1.0, true)
            }
            HoleEnd::UpToNext => (through_first_wall(&indexes, s, d, dims.radius, reach, n)?, true),
        };
        let tip_length = match (through, dims.tip) {
            (false, Some(half)) => dims.radius / half.tan(),
            _ => 0.0,
        };
        // A hole starting at the sketch has to reach the part somewhere along its length.
        if from_sketch && !line.iter().any(|(a, b)| *b > AHEAD && *a < depth + tip_length - AHEAD) {
            return Err(Error::InvalidInput(format!("hole {n} at {} doesn't reach the part from the sketch", place(q))));
        }
        let pick = match dims.top {
            Top::Plain => None,
            Top::Cbore { depth: c, .. } => Some((c, "counterbore")),
            Top::Csink { radius, half } => Some(((radius - dims.radius) / half.tan(), "countersink")),
        };
        if let Some((c, what)) = pick {
            if c >= depth - MIN_FEATURE {
                return Err(Error::InvalidInput(format!("hole {n}'s {what} goes {}, as deep as the hole or deeper", up_to::short(c))));
            }
        }
        let thread = match &dims.thread {
            Some((size, major, pitch, given)) => {
                if let (Some(t), false) = (given, through) {
                    if *t > depth + TOL {
                        return Err(Error::InvalidInput(format!("hole {n}'s thread goes {}, deeper than the hole's {}", up_to::short(*t), up_to::short(depth))));
                    }
                }
                let depth = match (given, through) {
                    (Some(t), _) => Some(*t),
                    (None, true) => None,
                    (None, false) => Some(depth),
                };
                Some(ThreadCallout { size: size.clone(), major: *major, pitch: *pitch, depth })
            }
            None => None,
        };
        let tool = cutter(&dims, s, d, back, depth, tip_length, n, feature.0)?;
        out.tools.push(tool);
        out.callouts.push(HoleCallout { feature, number: n, start: s, direction: d, text: callout(&dims, depth, through), thread });
    }
    Ok(out)
}

/// How deep a hole from `s` has to go to get through the first wall it meets, from rays down its middle and round its
/// edge. It ends half way to whatever's beyond, and no further than its radius past the wall.
fn through_first_wall(indexes: &[Index], s: Vec3, d: Vec3, radius: f64, reach: f64, n: u32) -> Result<f64> {
    let across = Frame::from_z(s, d);
    let mut exit: Option<f64> = None;
    let mut next = f64::INFINITY;
    let rays = 12;
    for k in 0..=rays {
        let p = if k == 0 {
            s
        } else {
            let a = TAU * k as f64 / rays as f64;
            across.point(0.98 * radius * a.cos(), 0.98 * radius * a.sin(), 0.0)
        };
        let line = spans(indexes, p, d, reach).ok_or_else(|| Error::InvalidInput(format!("hole {n} at {}: it can't be told where the wall it goes through ends", place(s))))?;
        let Some(i) = line.iter().position(|(_, b)| *b > AHEAD) else { continue };
        exit = Some(exit.map_or(line[i].1, |e: f64| e.max(line[i].1)));
        if let Some((a, _)) = line.get(i + 1) {
            next = next.min(*a);
        }
    }
    let exit = exit.ok_or_else(|| Error::InvalidInput(format!("hole {n} at {} meets no wall going that way: \"opposite\" goes the other way", place(s))))?;
    let gap = next - exit;
    if gap <= MIN_FEATURE * 10.0 {
        return Err(Error::InvalidInput(format!(
            "hole {n} at {}: it can't stop between the first wall it goes through and what's behind it, as across the hole's width they touch or overlap: give it a depth",
            place(s)
        )));
    }
    Ok(exit + (gap / 2.0).min(radius.max(1.0)))
}

/// The solid a hole cuts: its profile turned round its axis, from `back` behind `s` to `depth` past it, and its drill
/// point. Its faces are named for hole `n`.
#[allow(clippy::too_many_arguments)]
fn cutter(dims: &Dims, s: Vec3, d: Vec3, back: f64, depth: f64, tip_length: f64, n: u32, feature: u32) -> Result<Body> {
    let r = dims.radius;
    // Corners down one side of the profile, each with the face that starts at it.
    let mut corners: Vec<(Vec2, u32)> = vec![(v2(0.0, -back), HoleFace::Top.part())];
    match dims.top {
        Top::Plain => corners.push((v2(r, -back), HoleFace::Bore.part())),
        Top::Cbore { radius, depth: c } => {
            corners.push((v2(radius, -back), HoleFace::Cbore.part()));
            corners.push((v2(radius, c), HoleFace::CboreFloor.part()));
            corners.push((v2(r, c), HoleFace::Bore.part()));
        }
        Top::Csink { radius, half } => {
            // The cone carries on back, as the cutter's would.
            corners.push((v2(radius + back * half.tan(), -back), HoleFace::Csink.part()));
            corners.push((v2(r, (radius - r) / half.tan()), HoleFace::Bore.part()));
        }
    }
    if tip_length > 0.0 {
        corners.push((v2(r, depth), HoleFace::Tip.part()));
        corners.push((v2(0.0, depth + tip_length), AXIS));
    } else {
        corners.push((v2(r, depth), HoleFace::Floor.part()));
        corners.push((v2(0.0, depth), AXIS));
    }
    let segs = (0..corners.len())
        .map(|i| {
            let (a, id) = corners[i];
            let b = corners[(i + 1) % corners.len()].0;
            Seg { geom: SegGeom::Line { a, b }, id }
        })
        .collect();
    let profile = Profile::new(Wire { segs });
    let mut body = revolve(&profile, &Frame::from_z(s, d), TAU, feature)?;
    for f in &mut body.faces {
        let part = f.tag.slot - SLOT_SEG_BASE;
        f.tag.slot = SLOT_HOLE_BASE + n * HOLE_PARTS + part;
    }
    Ok(body)
}

fn callout(dims: &Dims, depth: f64, through: bool) -> String {
    let mm = up_to::short;
    let mut text = dims.words.clone();
    let deep = |x: f64| if through { "through".to_string() } else { format!("{} deep", mm(x)) };
    match &dims.thread {
        Some((_, _, _, given)) => {
            let thread_depth = match (given, through) {
                (Some(t), _) => format!("{} deep", mm(*t)),
                (None, true) => "through".into(),
                (None, false) => format!("{} deep", mm(depth)),
            };
            text = format!("{text} {thread_depth}, drill ⌀{} {}", mm(2.0 * dims.radius), deep(depth));
        }
        None => text.push_str(&format!("⌀{} {}", mm(2.0 * dims.radius), deep(depth))),
    }
    match dims.top {
        Top::Plain => {}
        Top::Cbore { radius, depth: c } => text.push_str(&format!(", counterbore ⌀{} {} deep", mm(2.0 * radius), mm(c))),
        Top::Csink { radius, half } => text.push_str(&format!(", countersink ⌀{} × {}°", mm(2.0 * radius), mm((2.0 * half).to_degrees()))),
    }
    if !through {
        match dims.tip {
            Some(half) => text.push_str(&format!(", {}° point", mm((2.0 * half).to_degrees()))),
            None => text.push_str(", flat bottom"),
        }
    }
    text
}
