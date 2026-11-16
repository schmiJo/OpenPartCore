//! Sheet metal, as Onshape has it: a part of one thickness, made of flat walls and the bends between them, that can be
//! laid out flat.
//!
//! A part comes from a sketch, as a plate from one region or as walls along a chain of lines with a bend at each corner.
//! A flange adds a bend and a wall along an edge, and a hem folds an edge back over its wall; both are a fold, an outline
//! across the edge extruded along it. A tab adds a region in a wall's plane, and a bend along a line folds a part along
//! a line drawn on one of its walls, turning one side about the bend's axis. Each is an ordinary solid, made by extruding
//! an outline, so the Booleans only ever put pieces together face to face.
//!
//! The flat pattern is worked out from the part as it is, so a hole cut through a wall since comes with it. Starting from
//! a face on one side of the sheet, it goes across each bend to the wall beyond, and lays that wall down where it would
//! be with the bend undone: turned back about the bend's axis, and moved on by the length of the bend's neutral line,
//! its angle times (r + k t), where r is its inner radius, t the thickness and k the k-factor. Each wall is its face on
//! that side, extruded through the thickness, and each bend a strip that long. Then the flat has to come to the part's
//! own volume, plus what the neutral line adds, or it's refused: that catches anything not square through the sheet.

use coreshape::boolean::{intersect, subtract, union};
use coreshape::build::{box_solid, extrude, SLOT_SEG_BASE};
use coreshape::curve::Curve;
use coreshape::error::{Error, Result};
use coreshape::flat::flat_face_profile;
use coreshape::mass::mass_properties;
use coreshape::math::*;
use coreshape::profile::{PathBuilder, Profile, SegGeom, Wire};
use coreshape::surface::Surface;
use coreshape::topo::{Body, Coedge};
use std::collections::HashSet;

/// Where a flange's bend goes, measured at the edge it's on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alignment {
    /// The flange's outside face goes through the edge: its outer virtual sharp is there, and the part's outside size
    /// stays what it was. The wall is cut back to make room for the bend.
    Outer,
    /// The flange's inside face goes through the edge: its inner virtual sharp is there.
    Inner,
    /// The bend starts at the edge, and the wall stays as it is.
    Bend,
}

impl Alignment {
    pub fn word(self) -> &'static str {
        match self {
            Alignment::Outer => "outer",
            Alignment::Inner => "inner",
            Alignment::Bend => "bend",
        }
    }

    pub fn from_word(word: &str) -> Option<Alignment> {
        match word {
            "outer" => Some(Alignment::Outer),
            "inner" => Some(Alignment::Inner),
            "bend" => Some(Alignment::Bend),
            _ => None,
        }
    }
}

fn at(p: Vec2) -> String {
    format!("({:.4}, {:.4})", p.x, p.y)
}

fn at3(p: Vec3) -> String {
    format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z)
}

/// The corners of an open sketch chain, if it's all lines.
pub(crate) fn chain_points(wire: &Wire) -> Result<Vec<Vec2>> {
    let mut points = Vec::with_capacity(wire.segs.len() + 1);
    for seg in &wire.segs {
        let SegGeom::Line { a, b } = seg.geom else {
            return Err(Error::Unsupported(
                "a sheet metal chain is lines only, with a bend at each corner: arcs and curves in it aren't built yet".into(),
            ));
        };
        if points.is_empty() {
            points.push(a);
        }
        points.push(b);
    }
    Ok(points)
}

/// The outline of a sheet `t` thick along a chain of lines, on their left, with a bend of inner radius `r` at each
/// corner. Its pieces go along the lines' side first, a wall and then a bend at each corner, then across the far end,
/// back along the other side, and across the near end. So the first wall's face on the lines' side is `side.0`.
pub(crate) fn chain_profile(points: &[Vec2], t: f64, r: f64) -> Result<Profile> {
    let mut p: Vec<Vec2> = Vec::with_capacity(points.len());
    for &q in points {
        if p.last().is_some_and(|last: &Vec2| last.dist(q) < 1e-9) {
            return Err(Error::InvalidProfile(format!("a line of the chain at {} has no length", at(q))));
        }
        p.push(q);
    }
    // A corner it goes straight on through is no corner.
    let mut i = 1;
    while i + 1 < p.len() {
        let (a, b) = ((p[i] - p[i - 1]).normalized(), (p[i + 1] - p[i]).normalized());
        if a.cross(b).abs() < 1e-12 && a.dot(b) > 0.0 {
            p.remove(i);
        } else {
            i += 1;
        }
    }
    if p.len() < 2 {
        return Err(Error::InvalidProfile("a sheet metal chain needs at least one line".into()));
    }
    let n = p.len() - 1;
    let dir: Vec<Vec2> = (0..n).map(|k| (p[k + 1] - p[k]).normalized()).collect();
    let left = |d: Vec2| v2(-d.y, d.x);
    // At each corner: which way it turns, how far back along each line its bend starts on the lines' side, and where the
    // bend's axis is. Turning left, toward the sheet, the lines' side is the bend's outside.
    let mut turns = Vec::with_capacity(n.saturating_sub(1));
    for k in 1..n {
        let (a, b) = (dir[k - 1], dir[k]);
        let turn = a.cross(b).atan2(a.dot(b));
        if turn.abs() > PI - 1e-6 {
            return Err(Error::Unsupported(format!(
                "the chain turns right back on itself at {}: a bend of 180 degrees is a hem, which isn't built yet",
                at(p[k])
            )));
        }
        let radius = if turn > 0.0 { r + t } else { r };
        let back = radius * (turn.abs() / 2.0).tan();
        let toward = if turn > 0.0 { left(a) } else { -left(a) };
        turns.push((turn, back, p[k] - a * back + toward * radius));
    }
    // Each line keeps a straight part between its bends.
    for k in 0..n {
        let used = if k > 0 { turns[k - 1].1 } else { 0.0 } + if k + 1 < n { turns[k].1 } else { 0.0 };
        if used >= p[k].dist(p[k + 1]) - 1e-9 {
            return Err(Error::InvalidProfile(format!(
                "the line from {} to {} is too short for its bends at this radius and thickness: they take {:.4} of its {:.4}",
                at(p[k]),
                at(p[k + 1]),
                used,
                p[k].dist(p[k + 1])
            )));
        }
    }
    let starts = |k: usize| if k == 0 { p[0] } else { p[k] + dir[k] * turns[k - 1].1 };
    let ends = |k: usize| if k + 1 == n { p[n] } else { p[k + 1] - dir[k] * turns[k].1 };
    let off = |k: usize, q: Vec2| q + left(dir[k]) * t;
    let mut path = PathBuilder::new(p[0]);
    for k in 0..n {
        path = path.line_to(ends(k));
        if k + 1 < n {
            let (turn, _, center) = turns[k];
            path = path.arc_to(starts(k + 1), center, turn > 0.0);
        }
    }
    path = path.line_to(off(n - 1, p[n]));
    for k in (0..n).rev() {
        path = path.line_to(off(k, starts(k)));
        if k > 0 {
            let (turn, _, center) = turns[k - 1];
            path = path.arc_to(off(k - 1, ends(k - 1)), center, turn < 0.0);
        }
    }
    let profile = Profile::new(path.close());
    profile.validate()?;
    Ok(profile)
}

/// How the folds of one feature meet where two of their edges share a corner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Corner {
    /// Each stops short of the corner by as far as it reaches back over the wall, leaving the corner open with a square
    /// cut out of it.
    Open,
    /// The walls of two flanges at 90 degrees run on into the corner, one over the other's end, with a gap between them.
    /// The bends still stop short, leaving a square hole where they would meet.
    Closed,
}

impl Corner {
    pub fn word(self) -> &'static str {
        match self {
            Corner::Open => "open",
            Corner::Closed => "closed",
        }
    }

    pub fn from_word(word: &str) -> Option<Corner> {
        match word {
            "open" => Some(Corner::Open),
            "closed" => Some(Corner::Closed),
            _ => None,
        }
    }
}

/// How a hem folds an edge back over its wall.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HemKind {
    /// Half a turn with a gap between the wall and what comes back over it, and a straight part after.
    Straight,
    /// Half a turn at a bend radius, and a straight part after.
    Rounded,
    /// More than half a turn, and a straight part after that leans back toward the wall.
    Teardrop,
    /// A roll: a turn of more than half, with nothing after it.
    Rolled,
}

impl HemKind {
    pub fn word(self) -> &'static str {
        match self {
            HemKind::Straight => "straight",
            HemKind::Rounded => "rounded",
            HemKind::Teardrop => "teardrop",
            HemKind::Rolled => "rolled",
        }
    }

    pub fn from_word(word: &str) -> Option<HemKind> {
        match word {
            "straight" => Some(HemKind::Straight),
            "rounded" => Some(HemKind::Rounded),
            "teardrop" => Some(HemKind::Teardrop),
            "rolled" => Some(HemKind::Rolled),
            _ => None,
        }
    }
}

/// What's folded along an edge, across it: a bend from x = `start`, where x runs out of the wall and the wall is from
/// y = -t to 0, of inner radius `radius`, turning by `angle` toward +y, then a straight wall `wall` long, or none.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Fold {
    pub start: f64,
    pub radius: f64,
    pub angle: f64,
    pub wall: f64,
    pub thickness: f64,
}

impl Fold {
    fn ends(&self) -> (Vec2, Vec2, Vec2, Vec2) {
        let (r, t, a) = (self.radius, self.thickness, self.angle);
        let c = v2(self.start, r);
        let radial = v2(a.sin(), -a.cos());
        let d = v2(a.cos(), a.sin());
        let (outside, inside) = (c + radial * (r + t), c + radial * r);
        (outside, inside, outside + d * self.wall, inside + d * self.wall)
    }

    /// Its outline across the edge. The pieces go: the bend's outside, the wall's outside, its end, its inside, the bend's
    /// inside, and back across where it meets the wall, so they're named `side.0` to `side.5`. With no wall, its end
    /// comes straight after the bend's outside.
    fn profile(&self) -> Profile {
        let c = v2(self.start, self.radius);
        let (outside, inside, far_out, far_in) = self.ends();
        let mut path = PathBuilder::new(v2(self.start, -self.thickness)).arc_to(outside, c, true);
        if self.wall > 0.0 {
            path = path.line_to(far_out).line_to(far_in);
        }
        Profile::new(path.line_to(inside).arc_to(v2(self.start, 0.0), c, false).close())
    }

    /// How far it reaches back over the wall from the edge: least x of its outline, which is never more than `start`.
    fn least_x(&self) -> f64 {
        let (outside, inside, far_out, far_in) = self.ends();
        let mut x = self.start.min(outside.x).min(inside.x).min(far_out.x).min(far_in.x);
        if self.angle > 1.5 * PI {
            x = x.min(self.start - self.radius - self.thickness);
        }
        x
    }

    /// Most x of its outline, at start 0.
    fn most_x(&self) -> f64 {
        let at_zero = Fold { start: 0.0, ..*self };
        let (outside, inside, far_out, far_in) = at_zero.ends();
        let mut x = outside.x.max(inside.x).max(far_out.x).max(far_in.x);
        if self.angle > 0.5 * PI {
            x = x.max(self.radius + self.thickness);
        }
        x
    }

    /// The lowest it comes, past half a turn, where it heads back down toward the wall's plane.
    fn lowest_after_half(&self) -> Option<f64> {
        if self.angle <= PI + 1e-9 {
            return None;
        }
        let (outside, inside, far_out, far_in) = self.ends();
        Some(outside.y.min(inside.y).min(far_out.y).min(far_in.y))
    }
}

/// How big a flange is, and how it sits.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FlangeSize {
    /// From the outer virtual sharp to the flange's far end, along its outside face.
    pub length: f64,
    /// How far it bends, in radians.
    pub angle: f64,
    /// The bend's inner radius.
    pub radius: f64,
    pub thickness: f64,
    pub alignment: Alignment,
}

impl FlangeSize {
    pub(crate) fn fold(&self) -> Result<Fold> {
        let (t, r, angle) = (self.thickness, self.radius, self.angle);
        if !(angle > 1e-9 && angle < PI - 1e-9) {
            return Err(Error::InvalidInput(format!(
                "a flange bends by more than 0 and less than 180 degrees, not {:.4}",
                angle.to_degrees()
            )));
        }
        if !(r >= MIN_FEATURE) {
            return Err(Error::InvalidInput(format!("a flange's bend radius has to be more than 0, not {r}")));
        }
        let half = (angle / 2.0).tan();
        let start = match self.alignment {
            Alignment::Outer => -(r + t) * half,
            Alignment::Inner => -r * half,
            Alignment::Bend => 0.0,
        };
        let wall = self.length - (r + t) * half;
        if !(wall > MIN_FEATURE) {
            return Err(Error::InvalidInput(format!(
                "a flange {:.4} long ends inside its own bend, which takes {:.4} of it at this angle, radius and thickness",
                self.length,
                (r + t) * half
            )));
        }
        Ok(Fold { start, radius: r, angle, wall, thickness: t })
    }
}

/// How big a hem is, and how it sits.
#[derive(Clone, Copy, Debug)]
pub(crate) struct HemSize {
    pub kind: HemKind,
    /// For a straight or rounded hem, from the outside of its fold to its end; for a teardrop, the straight part after
    /// the fold. A roll has none.
    pub length: f64,
    /// The fold's inner radius. A straight hem's is half its gap.
    pub radius: f64,
    /// How far it turns, in radians: half a turn for straight and rounded hems.
    pub angle: f64,
    pub thickness: f64,
    /// `outer` puts the outside of the fold where the edge was, so the part's outside size stays the same; `inner` puts
    /// the inside of the fold there, and `bend` starts the fold at the edge.
    pub alignment: Alignment,
}

impl HemSize {
    pub(crate) fn fold(&self) -> Result<Fold> {
        let (t, r, angle) = (self.thickness, self.radius, self.angle);
        let name = self.kind.word();
        if !(r >= MIN_FEATURE / 2.0) {
            return Err(Error::InvalidInput(match self.kind {
                HemKind::Straight => format!("a straight hem's gap has to be more than 0, not {}", 2.0 * r),
                _ => format!("a {name} hem's bend radius has to be more than 0, not {r}"),
            }));
        }
        match self.kind {
            HemKind::Straight | HemKind::Rounded => {}
            HemKind::Teardrop | HemKind::Rolled => {
                if !(angle > PI + 1e-9 && angle < 2.0 * PI - 1e-9) {
                    return Err(Error::InvalidInput(format!(
                        "a {name} hem turns by more than 180 and less than 360 degrees, not {:.4}",
                        angle.to_degrees()
                    )));
                }
            }
        }
        let wall = match self.kind {
            HemKind::Straight | HemKind::Rounded => {
                let wall = self.length - (r + t);
                if !(wall > MIN_FEATURE) {
                    return Err(Error::InvalidInput(format!(
                        "a {name} hem {:.4} long ends inside its own fold, which takes {:.4} of it at this radius and thickness",
                        self.length,
                        r + t
                    )));
                }
                wall
            }
            HemKind::Teardrop => {
                if !(self.length > MIN_FEATURE) {
                    return Err(Error::InvalidInput(format!("a teardrop hem's length has to be more than 0, not {}", self.length)));
                }
                self.length
            }
            HemKind::Rolled => 0.0,
        };
        let fold = Fold { start: 0.0, radius: r, angle, wall, thickness: t };
        if let Some(low) = fold.lowest_after_half() {
            if low <= 1e-9 {
                return Err(Error::InvalidInput(format!(
                    "a {name} hem turning {:.4} degrees comes back down onto its own wall: turn it less{}",
                    angle.to_degrees(),
                    if self.kind == HemKind::Teardrop { " or make it shorter" } else { "" }
                )));
            }
        }
        let start = match self.alignment {
            Alignment::Outer => -fold.most_x(),
            Alignment::Inner => -r,
            Alignment::Bend => 0.0,
        };
        Ok(Fold { start, ..fold })
    }
}

/// Where a fold along an edge goes: across the edge, x runs out of the wall and y toward the side it bends to, with the
/// wall from y = -t to 0, and z runs along it.
struct Seat {
    along: Vec3,
    len: f64,
    /// The edge's first vertex, moved onto the bend's inside, the face it bends toward.
    inner: Vec3,
    up: Vec3,
    /// Out of the wall, square to the edge.
    out: Vec3,
    /// The edge's vertices.
    v: [usize; 2],
    where_: String,
}

impl Seat {
    /// The frame for a piece from `from` to `to` along the edge, from its first vertex: x and y as above, and z from 0 to
    /// `to - from`.
    fn frame(&self, from: f64, to: f64) -> Frame {
        let z = self.out.cross(self.up);
        let origin = if z.dot(self.along) > 0.0 { self.inner + self.along * from } else { self.inner + self.along * to };
        Frame { origin, x: self.out, y: self.up, z }
    }
}

/// Where a fold goes on edge `e`, which has to be where a wall's face meets its side, the face that's the sheet's
/// thickness across. It bends toward the face the edge is on, or away from it with `flip`.
fn seat(body: &Body, uses: &[Vec<(usize, usize, usize, bool)>], e: usize, t: f64, flip: bool, what: &str) -> Result<Seat> {
    let edge = &body.edges[e];
    let (p0, p1) = (body.vertices[edge.v0], body.vertices[edge.v1]);
    let where_ = at3(p0.lerp(p1, 0.5));
    if !matches!(edge.curve, Curve::Line { .. }) {
        return Err(Error::InvalidInput(format!("{what} goes on a straight edge, and the one at {where_} isn't")));
    }
    let [(f1, ..), (f2, ..)] = uses[e].as_slice() else {
        return Err(Error::InvalidInput(format!("the edge at {where_} isn't between two faces")));
    };
    let (f1, f2) = (*f1, *f2);
    let along = (p1 - p0).normalized();
    let tol = 1e-7 * body.aabb().diagonal().max(1.0);
    // How far a flat face reaches across the edge, square to it.
    let across = |f: usize| -> Option<(Vec3, f64)> {
        let Surface::Plane(_) = body.faces[f].surface else { return None };
        let n = body.faces[f].outward_normal(p0);
        let into = n.cross(along).normalized();
        let reach = body.faces[f].loops.iter().flatten().map(|c| (body.vertices[body.start_vertex(*c)] - p0).dot(into).abs()).fold(0.0, f64::max);
        Some((n, reach))
    };
    let (Some((n1, r1)), Some((n2, r2))) = (across(f1), across(f2)) else {
        return Err(Error::InvalidInput(format!("{what} goes on an edge between two flat faces, and the one at {where_} isn't")));
    };
    let thick = |reach: f64| (reach - t).abs() <= tol;
    let (n_face, n_side) = match (thick(r1), thick(r2)) {
        (true, false) => (n2, n1),
        (false, true) => (n1, n2),
        (false, false) => {
            return Err(Error::InvalidInput(format!(
                "the edge at {where_} isn't along the side of a wall of this sheet: neither face beside it is just the sheet's thickness, {t}, across (flanges that meet at a corner go in one feature, and so do hems)"
            )))
        }
        (true, true) => {
            return Err(Error::InvalidInput(format!(
                "at the edge at {where_}, both faces beside it are the sheet's thickness across, so which is the wall isn't clear"
            )))
        }
    };
    if n_face.dot(n_side).abs() > 1e-9 || n_side.dot(along).abs() > 1e-9 {
        return Err(Error::InvalidInput(format!("the side of the wall at {where_} isn't square to it, so {what} can't start there")));
    }
    let up = if flip { -n_face } else { n_face };
    let inner = if flip { p0 - n_face * t } else { p0 };
    Ok(Seat { along, len: p0.dist(p1), inner, up, out: n_side, v: [edge.v0, edge.v1], where_ })
}

/// How far each end of a fold stops short of its edge's end, and the stretch of its wall that runs on past that to close
/// a corner, if it does: for the edge's first and last vertex, (stop, wall's end), as distances from that vertex along
/// the edge, inward.
type Ends = [(f64, Option<f64>); 2];

/// The ends of the fold on `seats[i]`, where the others of the same feature meet it.
fn ends_of(seats: &[Seat], i: usize, fold: &Fold, corner: Corner, gap: f64) -> Result<Ends> {
    let s = &seats[i];
    // A fold that reaches back over the wall past where its bend starts, as a hem or a flange of more than 90 degrees
    // does, stops the gap further short again, so that two at a corner don't touch.
    let reach = -fold.least_x();
    let stop = if reach > -fold.start + 1e-9 { reach + gap } else { reach };
    let mut ends: Ends = [(0.0, None); 2];
    for (end, v) in s.v.iter().enumerate() {
        let Some(o) = seats.iter().enumerate().find(|(j, o)| *j != i && o.v.contains(v)).map(|(_, o)| o) else { continue };
        ends[end].0 = stop;
        if corner == Corner::Open {
            continue;
        }
        let at = &s.where_;
        if (fold.angle - PI / 2.0).abs() > 1e-9 || fold.wall <= 0.0 {
            return Err(Error::Unsupported(format!(
                "closing the corner at the edge at {at} is built for flanges of 90 degrees, and this is a fold of {:.4}",
                fold.angle.to_degrees()
            )));
        }
        if s.up.dot(o.up) < 1.0 - 1e-9 {
            return Err(Error::Unsupported(format!("at the corner by the edge at {at}, the flanges bend different ways, so it can't be closed")));
        }
        // Along each edge from the corner, inward; at an outside corner where the edges meet square, that's straight back
        // against the other's way out.
        let inward = |q: &Seat| if q.v[0] == *v { q.along } else { -q.along };
        if inward(s).dot(o.out) > -1.0 + 1e-9 || inward(o).dot(s.out) > -1.0 + 1e-9 {
            return Err(Error::Unsupported(format!(
                "closing the corner by the edge at {at} is built where two edges meet square at an outside corner of the wall"
            )));
        }
        let (r, t) = (fold.radius, fold.thickness);
        if !(gap > 1e-9 && gap < r) {
            return Err(Error::InvalidInput(format!(
                "a closed corner's gap is more than 0 and less than the bend radius, {r}, which leaves it as closed as it gets: not {gap}"
            )));
        }
        // Going round the corner one way, the first flange's wall runs over the other's end, out to its outside; the
        // other's runs up to that one's inside, less the gap.
        let over = s.up.dot(s.out.cross(o.out)) > 0.0;
        ends[end].1 = Some(if over { -(fold.start + r + t) } else { -(fold.start + r) + gap });
    }
    Ok(ends)
}

/// The stretch of a 90-degree flange's wall from `z0` to `z1` along its edge, to close a corner. Its faces are named as
/// the flange's, with its underside, above the hole the bends leave, as `side.5`.
fn wall_piece(s: &Seat, fold: &Fold, z0: f64, z1: f64, tag: u32) -> Result<Body> {
    let (x, r, t, wall) = (fold.start + fold.radius, fold.radius, fold.thickness, fold.wall);
    let wire = PathBuilder::new(v2(x + t, r)).line_to(v2(x + t, r + wall)).line_to(v2(x, r + wall)).line_to(v2(x, r)).close();
    let mut piece = extrude(&Profile::new(wire), &s.frame(z0, z1), z1 - z0, tag)?;
    for face in &mut piece.faces {
        if face.tag.slot >= SLOT_SEG_BASE {
            face.tag.slot = SLOT_SEG_BASE + [1, 2, 3, 5][(face.tag.slot - SLOT_SEG_BASE) as usize];
        }
    }
    Ok(piece)
}

/// What one feature's folds on edges `list` of a part add, each with the strip of wall it takes away first to make room
/// for its bend, if any. Where two of them meet at a corner, they leave it open or close it, as `corner` says.
pub(crate) fn folds(body: &Body, list: &[usize], fold: &Fold, flip: bool, corner: Corner, gap: f64, what: &str, tag: u32) -> Result<Vec<(Body, Option<Body>)>> {
    let uses = body.edge_uses();
    let t = fold.thickness;
    let seats: Vec<Seat> = list.iter().map(|&e| seat(body, &uses, e, t, flip, what)).collect::<Result<_>>()?;
    let size_of = body.aabb().diagonal().max(1.0);
    let mut out = Vec::with_capacity(seats.len());
    for (i, s) in seats.iter().enumerate() {
        let ends = ends_of(&seats, i, fold, corner, gap)?;
        let (from, to) = (ends[0].0, s.len - ends[1].0);
        if to - from <= 1e-9 * size_of {
            return Err(Error::InvalidInput(format!(
                "the edge at {} is too short for {what} that has to stop {:.4} short of each corner it shares with another",
                s.where_,
                -fold.least_x()
            )));
        }
        let mut piece = extrude(&fold.profile(), &s.frame(from, to), to - from, tag)?;
        if let (stop, Some(reach)) = ends[0] {
            piece = union(&piece, &wall_piece(s, fold, reach, stop, tag)?)?;
        }
        if let (stop, Some(reach)) = ends[1] {
            piece = union(&piece, &wall_piece(s, fold, s.len - stop, s.len - reach, tag)?)?;
        }
        // The wall is cut back all along the edge.
        let trim = if fold.start < 0.0 {
            let margin = 0.1 * t;
            let whole = s.frame(0.0, s.len);
            let corner = Frame { origin: whole.to_world(v3(fold.start, -t - margin, 0.0)), ..whole };
            Some(box_solid(&corner, -fold.start + margin, t + 2.0 * margin, s.len, tag)?)
        } else {
            None
        };
        out.push((piece, trim));
    }
    Ok(out)
}

/// Whether flat face `q` is just the end of a bend beyond edge `e`: one loop, the sheet's thickness across.
fn ends_bend(body: &Body, q: usize, e: usize, t: f64, tol: f64) -> bool {
    let edge = &body.edges[e];
    let (p0, p1) = (body.vertices[edge.v0], body.vertices[edge.v1]);
    let along = (p1 - p0).normalized();
    let into = body.faces[q].outward_normal(p0).cross(along).normalized();
    let reach = body.faces[q].loops.iter().flatten().map(|c| (body.vertices[body.start_vertex(*c)] - p0).dot(into).abs()).fold(0.0, f64::max);
    body.faces[q].loops.len() == 1 && (reach - t).abs() <= tol
}

/// Puts folds on a part: takes away what each needs taken away, then adds them, and makes sure none runs into the part or
/// into another.
pub(crate) fn add_folds(body: &Body, made: &[(Body, Option<Body>)], what: &str) -> Result<Body> {
    let mut out = body.clone();
    for (_, trim) in made {
        if let Some(trim) = trim {
            out = subtract(&out, trim)?;
        }
    }
    for (piece, _) in made {
        let before = mass_properties(&out).volume;
        let added = mass_properties(piece).volume;
        let next = union(&out, piece)?;
        let after = mass_properties(&next).volume;
        if (after - (before + added)).abs() > 1e-7 * (before + added).abs().max(1.0) {
            let b = piece.aabb();
            return Err(Error::InvalidInput(format!(
                "the {what} around {} runs into the part: {:.4} of it is where the part already is",
                at3(b.min.lerp(b.max, 0.5)),
                before + added - after
            )));
        }
        out = next;
    }
    Ok(out)
}

/// Which way into the sheet a part's walls on the plane z = 0 of `frame` go: +1 along its z, -1 against it, or None if
/// no wall of the part is on that plane.
fn wall_on(body: &Body, frame: &Frame, what: &str) -> Result<Option<f64>> {
    let tol = 1e-7 * body.aabb().diagonal().max(1.0);
    let mut way = None;
    for f in &body.faces {
        let Surface::Plane(pl) = f.surface else { continue };
        if pl.z.cross(frame.z).norm() > 1e-9 || (pl.origin - frame.origin).dot(frame.z).abs() > tol {
            continue;
        }
        let w = if f.outward_normal(pl.origin).dot(frame.z) > 0.0 { -1.0 } else { 1.0 };
        match way {
            None => way = Some(w),
            Some(was) if was != w => {
                return Err(Error::InvalidInput(format!(
                    "the part has walls on both sides of the sketch's plane, so which one {what} goes with isn't clear"
                )))
            }
            Some(_) => {}
        }
    }
    Ok(way)
}

/// A tab: region `profile`, drawn on the plane of one of a part's walls, the sheet's thickness through, joined to the
/// part. Where it overlaps the part, that has to be flat wall, square through the sheet. None if no wall of the part is
/// on the sketch's plane, or the tab doesn't reach the part. `name` names its faces.
pub(crate) fn tab(body: &Body, profile: &Profile, frame: &Frame, t: f64, tag: u32, name: &dyn Fn(Body) -> Body) -> Result<Option<Body>> {
    let Some(way) = wall_on(body, frame, "a tab")? else { return Ok(None) };
    let slab = name(extrude(profile, frame, way * t, tag)?);
    let overlap = intersect(body, &slab)?;
    for f in &overlap.faces {
        let square = match &f.surface {
            Surface::Plane(pl) => pl.z.cross(frame.z).norm() < 1e-9 || pl.z.dot(frame.z).abs() < 1e-9,
            Surface::Cylinder { frame: axis, .. } => axis.z.cross(frame.z).norm() < 1e-9,
            _ => false,
        };
        if !square {
            let corners: Vec<Vec3> = f.loops.iter().flatten().map(|c| overlap.vertices[overlap.start_vertex(*c)]).collect();
            let middle = corners.iter().fold(v3(0.0, 0.0, 0.0), |s, p| s + *p) * (1.0 / corners.len().max(1) as f64);
            return Err(Error::InvalidInput(format!(
                "the tab runs into something around {} that isn't flat wall square through the sheet, such as a bend: a tab only joins walls in its own plane",
                at3(middle)
            )));
        }
    }
    let joined = union(body, &slab)?;
    if joined.solids().len() > body.solids().len() {
        return Ok(None);
    }
    let (before, after) = (mass_properties(body).volume, mass_properties(&joined).volume);
    if after - before <= 1e-9 * before.max(1.0) {
        return Err(Error::InvalidInput("the tab is all where the part already is, so it adds nothing".into()));
    }
    Ok(Some(joined))
}

/// Where a bend along a line goes, against the line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BendPosition {
    /// The bend's strip in the flat is centred on the line.
    Center,
    /// The bend starts at the line, on the side that moves.
    Start,
}

impl BendPosition {
    pub fn word(self) -> &'static str {
        match self {
            BendPosition::Center => "center",
            BendPosition::Start => "start",
        }
    }

    pub fn from_word(word: &str) -> Option<BendPosition> {
        match word {
            "center" => Some(BendPosition::Center),
            "start" => Some(BendPosition::Start),
            _ => None,
        }
    }
}

/// How a part is bent along a line drawn on one of its walls.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LineBend {
    pub angle: f64,
    pub radius: f64,
    pub thickness: f64,
    pub k_factor: f64,
    pub position: BendPosition,
    /// Bend away from the side the sketch's normal points to, rather than toward it.
    pub flip: bool,
    /// The part on the line's right moves, rather than the part on its left, seen from along the sketch's normal.
    pub right: bool,
}

/// A part bent along the line from `a` to `b` in `frame`, which has to be on the plane of one of its walls and go right
/// across it. The strip of wall the bend's neutral line takes in the flat becomes the bend, and what's past it on the
/// side that moves turns about the bend's axis, so the part's flat pattern stays as it was. None if no wall of the part
/// is on the sketch's plane.
pub(crate) fn bend_along(body: &Body, frame: &Frame, a: Vec2, b: Vec2, size: &LineBend, tag: u32) -> Result<Option<Body>> {
    let (t, r, angle) = (size.thickness, size.radius, size.angle);
    if !(angle > 1e-9 && angle < PI - 1e-9) {
        return Err(Error::InvalidInput(format!("a bend turns by more than 0 and less than 180 degrees, not {:.4}", angle.to_degrees())));
    }
    if !(r >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("a bend radius has to be more than 0, not {r}")));
    }
    let Some(way) = wall_on(body, frame, "a bend")? else { return Ok(None) };
    let (pa, pb) = (frame.to_world(v3(a.x, a.y, 0.0)), frame.to_world(v3(b.x, b.y, 0.0)));
    let len = pa.dist(pb);
    if len <= MIN_FEATURE {
        return Err(Error::InvalidInput("the line to bend along has no length".into()));
    }
    let dir = (pb - pa).normalized();
    let left = frame.z.cross(dir);
    // x toward the side that moves, y toward the side it bends to, with the wall from y = -t to 0.
    let x = if size.right { -left } else { left };
    let up = if size.flip { -frame.z } else { frame.z };
    let into = frame.z * way;
    let inner = if up.dot(into) > 0.0 { pa + into * t } else { pa };
    let z = x.cross(up);
    let origin = if z.dot(dir) > 0.0 { inner } else { inner + dir * len };
    let bf = Frame { origin, x, y: up, z };
    let neutral = angle * (r + size.k_factor * t);
    let x1 = match size.position {
        BendPosition::Center => -neutral / 2.0,
        BendPosition::Start => 0.0,
    };
    let where_ = at3(pa.lerp(pb, 0.5));
    // The strip that becomes the bend, through the wall and a little either side of it.
    let margin = 0.1 * t;
    let strip = box_solid(&Frame { origin: bf.to_world(v3(x1, -t - margin, 0.0)), ..bf }, neutral, t + 2.0 * margin, len, tag)?;
    let flat = intersect(body, &strip)?;
    let span = flat.vertices.iter().map(|p| bf.to_local(*p).z).fold((f64::MAX, f64::MIN), |(lo, hi), z| (lo.min(z), hi.max(z)));
    let width = span.1 - span.0;
    let plain = !flat.faces.is_empty()
        && flat.solids().len() == 1
        && width > MIN_FEATURE
        && (mass_properties(&flat).volume - neutral * t * width).abs() <= 1e-7 * (neutral * t * width).max(1.0);
    if !plain {
        return Err(Error::InvalidInput(format!(
            "along the line at {where_}, the strip that becomes the bend, {neutral:.4} wide, has to be plain flat wall right across, with nothing cut into it or standing on it"
        )));
    }
    // What's left either side.
    let rest = subtract(body, &strip)?;
    let tol = 1e-7 * body.aabb().diagonal().max(1.0);
    let cut_at = |piece: &Body, at: f64, facing: f64| {
        piece.faces.iter().any(|f| match f.surface {
            Surface::Plane(pl) => f.outward_normal(pl.origin).dot(x) * facing > 1.0 - 1e-9 && (bf.to_local(pl.origin).x - at).abs() <= tol,
            _ => false,
        })
    };
    let (mut stay, mut moving) = (Vec::new(), Vec::new());
    for piece in rest.solids() {
        match (cut_at(&piece, x1, 1.0), cut_at(&piece, x1 + neutral, -1.0)) {
            (true, false) => stay.push(piece),
            (false, true) => moving.push(piece),
            _ => {
                return Err(Error::InvalidInput(format!(
                    "the line at {where_} doesn't cut the part in two: it has to go right across the wall, with nothing of the part coming round to the other side of it"
                )))
            }
        }
    }
    if stay.is_empty() || moving.is_empty() {
        return Err(Error::InvalidInput(format!("the line at {where_} doesn't go right across the wall, so it can't bend along it")));
    }
    let fold = Fold { start: x1, radius: r, angle, wall: 0.0, thickness: t };
    let bend = extrude(&fold.profile(), &Frame { origin: bf.to_world(v3(0.0, 0.0, span.0)), ..bf }, width, tag)?;
    // Back by the strip, then round the bend's axis.
    let motion = Frame::rotation(bf.to_world(v3(x1, r, 0.0)), z, angle).compose(&Frame::translation(-x * neutral));
    let mut volume = mass_properties(&bend).volume;
    let mut out = bend;
    for piece in &stay {
        volume += mass_properties(piece).volume;
        out = union(&out, piece)?;
    }
    for piece in &moving {
        let moved = piece.transformed(&motion);
        volume += mass_properties(&moved).volume;
        out = union(&out, &moved)?;
    }
    let got = mass_properties(&out).volume;
    if (got - volume).abs() > 1e-7 * volume.max(1.0) {
        return Err(Error::InvalidInput(format!(
            "bent along the line at {where_}, the side that moves runs into the rest of the part: {:.4} of it overlaps",
            volume - got
        )));
    }
    Ok(Some(out))
}

/// A sheet metal part laid out flat.
#[derive(Clone, Debug)]
pub struct FlatPattern {
    /// The flat part. It lies in `frame`, from z = 0 to the thickness.
    pub body: Body,
    /// Where the flat lies: on the plane of the face it was unfolded from, which stays where it is, with z into the sheet.
    pub frame: Frame,
    /// Its outline and holes on the side at z = 0, in `frame`'s x and y.
    pub outline: Vec<FlatEdge>,
    /// Where each bend is.
    pub bends: Vec<FlatBend>,
    pub thickness: f64,
    pub k_factor: f64,
}

/// A piece of a flat pattern's outline, in its frame's x and y.
#[derive(Clone, Debug, PartialEq)]
pub enum FlatEdge {
    Line { a: Vec2, b: Vec2 },
    /// From `a` to `b` about `center`, counter-clockwise seen from +z if `ccw`.
    Arc { center: Vec2, radius: f64, a: Vec2, b: Vec2, ccw: bool },
    /// Any other curve, as points along it.
    Points(Vec<Vec2>),
}

/// One bend in a flat pattern: its middle line, from `a` to `b` in the frame's x and y.
#[derive(Clone, Debug, PartialEq)]
pub struct FlatBend {
    pub a: Vec2,
    pub b: Vec2,
    /// How far it bends.
    pub degrees: f64,
    /// Its inner radius.
    pub radius: f64,
    /// Whether what's beyond it goes up, toward +z, when it's bent.
    pub up: bool,
    /// How wide its strip is in the flat: the length of its neutral line.
    pub width: f64,
}

/// Lays a sheet metal part out flat, from `seed`, a flat face on one side of its sheet, which stays where it is. Bends are
/// faces round an axis, each with another round the same axis on the sheet's other side, and the part is everything
/// reached from `seed` across them.
pub fn flat_pattern(body: &Body, seed: usize, t: f64, k: f64, tag: u32) -> Result<FlatPattern> {
    if !(t > 0.0) {
        return Err(Error::InvalidInput(format!("a sheet's thickness has to be more than 0, not {t}")));
    }
    let uses = body.edge_uses();
    let size = body.aabb().diagonal().max(1.0);
    let tol = 1e-7 * size;
    let other = |e: usize, f: usize| uses[e].iter().map(|u| u.0).find(|&g| g != f);
    let smooth = |e: usize, f: usize, g: usize| {
        let m = body.edges[e].eval_mid();
        body.faces[f].outward_normal(m).dot(body.faces[g].outward_normal(m)) > 1.0 - 1e-9
    };
    let ends = |c: &Coedge| (body.vertices[body.start_vertex(*c)], body.vertices[body.end_vertex(*c)]);
    if !matches!(body.faces.get(seed).map(|f| &f.surface), Some(Surface::Plane(_))) {
        return Err(Error::InvalidInput("a flat pattern starts from a flat face".into()));
    }
    // Each flat face on this side of the sheet, and the motion that lays it down.
    let mut placed: Vec<(usize, Frame)> = vec![(seed, Frame::WORLD)];
    let mut queue = vec![seed];
    let mut done_bends: HashSet<usize> = HashSet::new();
    let mut strips: Vec<Body> = Vec::new();
    let mut bends: Vec<(Vec3, Vec3, FlatBend)> = Vec::new();
    // What the neutral lines add to the volume, on top of the part's own.
    let mut extra = 0.0;
    while let Some(f) = queue.pop() {
        let here = placed.iter().find(|p| p.0 == f).expect("placed before queued").1;
        let lists: Vec<Coedge> = body.faces[f].loops.iter().flatten().copied().collect();
        for c in lists {
            let Some(g) = other(c.edge, f) else { continue };
            if !smooth(c.edge, f, g) {
                continue;
            }
            match &body.faces[g].surface {
                Surface::Plane(_) => {
                    // The same wall going on: it lies where this one does.
                    if !placed.iter().any(|p| p.0 == g) {
                        placed.push((g, here));
                        queue.push(g);
                    }
                }
                Surface::Cylinder { frame: axis, radius } => {
                    if !done_bends.insert(g) {
                        continue;
                    }
                    let (o, a) = (axis.origin, axis.z);
                    let bend = &body.faces[g];
                    let mid = body.edges[c.edge].eval_mid();
                    // A bend is a strip of the cylinder: two lines along the axis, where walls meet it, and two arcs. One of
                    // more than half a turn comes as several such strips one after another, round the same axis.
                    let strip_of = |h: usize| -> Result<(Vec<Coedge>, usize)> {
                        let rim: Vec<Coedge> = body.faces[h].loops.iter().flatten().copied().collect();
                        let lines: Vec<Coedge> = rim.iter().copied().filter(|x| matches!(body.edges[x.edge].curve, Curve::Line { .. })).collect();
                        if body.faces[h].loops.len() != 1 || rim.len() != 4 || lines.len() != 2 {
                            return Err(Error::Unsupported(format!(
                                "the bend at {} has something cut across it or into it, and a flat pattern only takes bends that are whole strips so far",
                                at3(mid)
                            )));
                        }
                        let arc = rim.iter().find(|x| !matches!(body.edges[x.edge].curve, Curve::Line { .. })).expect("two arcs").edge;
                        Ok((lines, arc))
                    };
                    let (lines, mut arc) = strip_of(g)?;
                    let mut far = lines.iter().find(|x| x.edge != c.edge).expect("two lines").edge;
                    let mut angle = (body.edges[arc].t1 - body.edges[arc].t0).abs();
                    let mut last = g;
                    while let Some(h) = other(far, last) {
                        let same = match body.faces[h].surface {
                            Surface::Cylinder { frame: ax, radius: rr } => {
                                (rr - radius).abs() <= tol && ax.z.cross(a).norm() < 1e-9 && (ax.origin - o).cross(a).norm() <= tol && smooth(far, last, h)
                            }
                            _ => false,
                        };
                        if !same || !done_bends.insert(h) {
                            break;
                        }
                        let (lines, next_arc) = strip_of(h)?;
                        far = lines.iter().find(|x| x.edge != far).expect("two lines").edge;
                        angle += (body.edges[next_arc].t1 - body.edges[next_arc].t0).abs();
                        (arc, last) = (next_arc, h);
                    }
                    let (near_a, near_b) = ends(&c);
                    let (far_a, far_b) = (body.vertices[body.edges[far].v0], body.vertices[body.edges[far].v1]);
                    if (near_a.dist(near_b) - far_a.dist(far_b)).abs() > tol {
                        return Err(Error::Unsupported(format!("the bend at {} isn't as long on one side as on the other", at3(mid))));
                    }
                    // Its other side, round the same axis a thickness further in or out.
                    let convex = !bend.flipped;
                    let partner = if convex { radius - t } else { radius + t };
                    let paired = body.faces.iter().any(|h| match h.surface {
                        Surface::Cylinder { frame: ax, radius: rr } => {
                            (rr - partner).abs() <= tol && ax.z.cross(a).norm() < 1e-9 && (ax.origin - o).cross(a).norm() <= tol
                        }
                        _ => false,
                    });
                    if !paired {
                        return Err(Error::Unsupported(format!(
                            "the round face at {} isn't a bend, with the sheet's other side round the same axis: rounded edges can't be laid flat",
                            at3(mid)
                        )));
                    }
                    let inner = if convex { radius - t } else { *radius };
                    // Turning about the axis from the far line to the near one undoes the bend. How far is what its arcs
                    // span together, which can be half a turn or more, as in a hem; which way is the way from the far line
                    // to the middle of the last arc.
                    let radial = |p: Vec3| {
                        let d = p - o;
                        (d - a * d.dot(a)).normalized()
                    };
                    let from = radial(body.edges[far].eval_mid());
                    let turn = if from.cross(radial(body.edges[arc].eval_mid())).dot(a) > 0.0 { angle } else { -angle };
                    let neutral = angle * (inner + k * t);
                    // In this wall's plane, square to the bend's line, away from the wall.
                    let n = body.faces[f].outward_normal(mid);
                    let out = -n.cross((near_b - near_a).normalized()).normalized();
                    let unbend = Frame::translation(out * neutral).compose(&Frame::rotation(o, a, turn));
                    let beyond = here.compose(&unbend);
                    let Some(q) = other(far, last) else {
                        return Err(Error::InvalidBody(vec![format!("the bend at {} has an open side", at3(mid))]));
                    };
                    // Past it, a flat wall going on from it, or its own end, as a roll has.
                    let plane = matches!(body.faces[q].surface, Surface::Plane(_));
                    let goes_on = plane && smooth(far, last, q);
                    if !plane || (!goes_on && !ends_bend(body, q, far, t, tol)) {
                        return Err(Error::Unsupported(format!(
                            "past the bend at {} there's no flat wall going on from it, which a flat pattern needs so far",
                            at3(mid)
                        )));
                    }
                    match placed.iter().find(|p| p.0 == q) {
                        _ if !goes_on => {}
                        Some((_, already)) => {
                            let probe = [v3(0.0, 0.0, 0.0), v3(size, 0.0, 0.0), v3(0.0, size, 0.0), v3(0.0, 0.0, size)];
                            if probe.iter().any(|p| already.to_world(*p).dist(beyond.to_world(*p)) > 1e-6 * size) {
                                return Err(Error::Unsupported(
                                    "this sheet closes on itself, so laying it flat would need a rip somewhere, which isn't built yet".into(),
                                ));
                            }
                        }
                        None => {
                            placed.push((q, beyond));
                            queue.push(q);
                        }
                    }
                    // The strip, on this wall's plane past the bend's line, through the thickness.
                    let width = near_a.dist(near_b);
                    let line = (near_b - near_a).normalized();
                    let (origin, y) = if out.cross(line).dot(-n) > 0.0 { (near_a, line) } else { (near_b, -line) };
                    let strip = box_solid(&Frame { origin, x: out, y, z: out.cross(y) }, neutral, width, t, tag)?;
                    strips.push(strip.transformed(&here));
                    extra += width * t * angle * (k - 0.5) * t;
                    let middle = |p: Vec3| here.to_world(p + out * (neutral / 2.0));
                    bends.push((
                        middle(near_a),
                        middle(near_b),
                        FlatBend { a: v2(0.0, 0.0), b: v2(0.0, 0.0), degrees: angle.to_degrees(), radius: inner, up: convex, width: neutral },
                    ));
                }
                _ => {
                    return Err(Error::Unsupported(format!(
                        "the face at {} goes on smoothly from a wall but isn't flat or a bend, and only those can be laid flat so far",
                        at3(body.edges[c.edge].eval_mid())
                    )))
                }
            }
        }
    }
    // Each wall, from its face on this side through the thickness, where it's laid down.
    let mut pieces = Vec::with_capacity(placed.len() + strips.len());
    for (f, motion) in &placed {
        let (frame, profile, _) = flat_face_profile(body, *f)?;
        let through = Frame { origin: frame.origin - frame.z * t, ..frame };
        pieces.push(extrude(&profile, &motion.compose(&through), t, tag)?);
    }
    pieces.extend(strips);
    let mut flat = pieces[0].clone();
    for piece in &pieces[1..] {
        flat = union(&flat, piece)?;
    }
    let want = mass_properties(body).volume + extra;
    let got = mass_properties(&flat).volume;
    if (got - want).abs() > 1e-6 * want.abs().max(1.0) {
        return Err(Error::Unsupported(format!(
            "laid flat this part comes to {got:.4}, where {want:.4} was expected: either not all of it is reached across bends from its first wall, or something isn't square through the sheet"
        )));
    }
    // The flat's frame: on the starting face's plane, z into the sheet.
    let (start, _, _) = flat_face_profile(body, seed)?;
    let frame = Frame { origin: start.origin, x: start.x, y: -start.y, z: -start.z };
    let local = |p: Vec3| {
        let l = frame.to_local(p);
        v2(l.x, l.y)
    };
    let flat_uses = flat.edge_uses();
    let on_bottom = |f: usize| match flat.faces[f].surface {
        Surface::Plane(pl) => pl.z.cross(frame.z).norm() < 1e-9 && (pl.origin - frame.origin).dot(frame.z).abs() <= tol,
        _ => false,
    };
    let mut outline = Vec::new();
    for (e, list) in flat_uses.iter().enumerate() {
        if list.iter().filter(|u| on_bottom(u.0)).count() != 1 {
            continue;
        }
        let edge = &flat.edges[e];
        let (a, b) = (local(flat.vertices[edge.v0]), local(flat.vertices[edge.v1]));
        outline.push(match &edge.curve {
            Curve::Line { .. } => FlatEdge::Line { a, b },
            Curve::Circle { frame: around, radius } => FlatEdge::Arc { center: local(around.origin), radius: *radius, a, b, ccw: around.z.dot(frame.z) > 0.0 },
            other => FlatEdge::Points((0..=32).map(|i| local(other.eval(edge.t0 + (edge.t1 - edge.t0) * i as f64 / 32.0))).collect()),
        });
    }
    let bends = bends.into_iter().map(|(a, b, bend)| FlatBend { a: local(a), b: local(b), ..bend }).collect();
    Ok(FlatPattern { body: flat, frame, outline, bends, thickness: t, k_factor: k })
}
