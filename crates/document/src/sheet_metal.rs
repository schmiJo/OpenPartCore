//! Sheet metal, as Onshape has it: a part of one thickness, made of flat walls and the bends between them, that can be
//! laid out flat.
//!
//! A part comes from a sketch, as a plate from one region or as walls along a chain of lines with a bend at each corner,
//! and a flange adds a bend and a wall along an edge. Each is an ordinary solid, made by extruding an outline, so the
//! Booleans only ever put pieces together face to face.
//!
//! The flat pattern is worked out from the part as it is, so a hole cut through a wall since comes with it. Starting from
//! a face on one side of the sheet, it goes across each bend to the wall beyond, and lays that wall down where it would
//! be with the bend undone: turned back about the bend's axis, and moved on by the length of the bend's neutral line,
//! its angle times (r + k t), where r is its inner radius, t the thickness and k the k-factor. Each wall is its face on
//! that side, extruded through the thickness, and each bend a strip that long. Then the flat has to come to the part's
//! own volume, plus what the neutral line adds, or it's refused: that catches anything not square through the sheet.

use coreshape::boolean::{subtract, union};
use coreshape::build::{box_solid, extrude};
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
    /// Bend away from the face the edge is on, rather than toward it.
    pub flip: bool,
}

/// What a flange on edge `e` adds, and the strip of wall it takes away first to make room for its bend, if any. The edge
/// is where a wall's face meets its side, which is the sheet's thickness across. The flange bends toward the face the
/// edge is on, or away from it with `flip`. At an end of the edge where another flange meets it, `corners` for the
/// edge's first and last vertex, it stops short by as much as the wall is cut back, so that the two leave the corner
/// open, with a square cut out of it, rather than run into each other.
pub(crate) fn flange(
    body: &Body,
    uses: &[Vec<(usize, usize, usize, bool)>],
    e: usize,
    size: &FlangeSize,
    corners: [bool; 2],
    tag: u32,
) -> Result<(Body, Option<Body>)> {
    let (t, r, angle) = (size.thickness, size.radius, size.angle);
    let edge = &body.edges[e];
    let (p0, p1) = (body.vertices[edge.v0], body.vertices[edge.v1]);
    let where_ = at3(p0.lerp(p1, 0.5));
    if !matches!(edge.curve, Curve::Line { .. }) {
        return Err(Error::InvalidInput(format!("a flange goes on a straight edge, and the one at {where_} isn't")));
    }
    if !(angle > 1e-9 && angle < PI - 1e-9) {
        return Err(Error::InvalidInput(format!(
            "a flange bends by more than 0 and less than 180 degrees, not {:.4}",
            angle.to_degrees()
        )));
    }
    if !(r >= MIN_FEATURE) {
        return Err(Error::InvalidInput(format!("a flange's bend radius has to be more than 0, not {r}")));
    }
    let [(f1, ..), (f2, ..)] = uses[e].as_slice() else {
        return Err(Error::InvalidInput(format!("the edge at {where_} isn't between two faces")));
    };
    let (f1, f2) = (*f1, *f2);
    let along = (p1 - p0).normalized();
    let len = p0.dist(p1);
    let size_of = body.aabb().diagonal().max(1.0);
    let tol = 1e-7 * size_of;
    // How far a flat face reaches across the edge, square to it.
    let across = |f: usize| -> Option<(Vec3, f64)> {
        let Surface::Plane(_) = body.faces[f].surface else { return None };
        let n = body.faces[f].outward_normal(p0);
        let into = n.cross(along).normalized();
        let reach = body.faces[f].loops.iter().flatten().map(|c| (body.vertices[body.start_vertex(*c)] - p0).dot(into).abs()).fold(0.0, f64::max);
        Some((n, reach))
    };
    let (Some((n1, r1)), Some((n2, r2))) = (across(f1), across(f2)) else {
        return Err(Error::InvalidInput(format!("a flange goes on an edge between two flat faces, and the one at {where_} isn't")));
    };
    let thick = |reach: f64| (reach - t).abs() <= tol;
    let (face, side) = match (thick(r1), thick(r2)) {
        (true, false) => ((f2, n2), (f1, n1)),
        (false, true) => ((f1, n1), (f2, n2)),
        (false, false) => {
            return Err(Error::InvalidInput(format!(
                "the edge at {where_} isn't along the side of a wall of this sheet: neither face beside it is just the sheet's thickness, {t}, across (flanges that meet at a corner go in one feature)"
            )))
        }
        (true, true) => {
            return Err(Error::InvalidInput(format!(
                "at the edge at {where_}, both faces beside it are the sheet's thickness across, so which is the wall isn't clear"
            )))
        }
    };
    let (n_face, n_side) = (face.1, side.1);
    if n_face.dot(n_side).abs() > 1e-9 || n_side.dot(along).abs() > 1e-9 {
        return Err(Error::InvalidInput(format!("the side of the wall at {where_} isn't square to it, so a flange can't start there")));
    }
    // Across the edge: x out of the wall, y toward the side it bends to, with the wall from y = -t to 0.
    let up = if size.flip { -n_face } else { n_face };
    let inner = if size.flip { p0 - n_face * t } else { p0 };
    let half = (angle / 2.0).tan();
    let start = match size.alignment {
        Alignment::Outer => -(r + t) * half,
        Alignment::Inner => -r * half,
        Alignment::Bend => 0.0,
    };
    // Where along the edge the flange runs, short of corners it shares.
    let (from, to) = (if corners[0] { -start } else { 0.0 }, len - if corners[1] { -start } else { 0.0 });
    if to - from <= 1e-9 * size_of {
        return Err(Error::InvalidInput(format!(
            "the edge at {where_} is too short for a flange that has to stop {:.4} short of each corner it shares with another",
            -start
        )));
    }
    let z = n_side.cross(up);
    let forward = z.dot(along) > 0.0;
    let frame = Frame { origin: if forward { inner + along * from } else { inner + along * to }, x: n_side, y: up, z };
    // The wall is cut back all along the edge.
    let whole = Frame { origin: if forward { inner } else { inner + along * len }, ..frame };
    let wall = size.length - (r + t) * half;
    if wall <= 1e-9 * size_of {
        return Err(Error::InvalidInput(format!(
            "a flange {:.4} long ends inside its own bend, which takes {:.4} of it at this angle, radius and thickness",
            size.length,
            (r + t) * half
        )));
    }
    let c = v2(start, r);
    let radial = v2(angle.sin(), -angle.cos());
    let d = v2(angle.cos(), angle.sin());
    let outside = c + radial * (r + t);
    let inside = c + radial * r;
    let wire = PathBuilder::new(v2(start, -t))
        .arc_to(outside, c, true)
        .line_to(outside + d * wall)
        .line_to(inside + d * wall)
        .line_to(inside)
        .arc_to(v2(start, 0.0), c, false)
        .close();
    let piece = extrude(&Profile::new(wire), &frame, to - from, tag)?;
    let trim = if start < 0.0 {
        let margin = 0.1 * t;
        let corner = Frame { origin: whole.to_world(v3(start, -t - margin, 0.0)), ..whole };
        Some(box_solid(&corner, -start + margin, t + 2.0 * margin, len, tag)?)
    } else {
        None
    };
    Ok((piece, trim))
}

/// Puts flanges on a part: takes away what each needs taken away, then adds them, and makes sure none runs into the part
/// or into another.
pub(crate) fn add_flanges(body: &Body, made: &[(Body, Option<Body>)]) -> Result<Body> {
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
                "the flange around {} runs into the part: {:.4} of it is where the part already is",
                at3(b.min.lerp(b.max, 0.5)),
                before + added - after
            )));
        }
        out = next;
    }
    Ok(out)
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
                    // A bend is a strip of the cylinder: two lines along the axis, where walls meet it, and two arcs.
                    let rim: Vec<Coedge> = bend.loops.iter().flatten().copied().collect();
                    let lines: Vec<Coedge> = rim.iter().copied().filter(|x| matches!(body.edges[x.edge].curve, Curve::Line { .. })).collect();
                    if bend.loops.len() != 1 || rim.len() != 4 || lines.len() != 2 {
                        return Err(Error::Unsupported(format!(
                            "the bend at {} has something cut across it or into it, and a flat pattern only takes bends that are whole strips so far",
                            at3(mid)
                        )));
                    }
                    let far = lines.iter().find(|x| x.edge != c.edge).expect("two lines").edge;
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
                    // Turning about the axis from the far line to the near one undoes the bend.
                    let radial = |p: Vec3| {
                        let d = p - o;
                        (d - a * d.dot(a)).normalized()
                    };
                    let (to, from) = (radial(mid), radial(body.edges[far].eval_mid()));
                    let turn = from.cross(to).dot(a).atan2(from.dot(to));
                    let angle = turn.abs();
                    let neutral = angle * (inner + k * t);
                    // In this wall's plane, square to the bend's line, away from the wall.
                    let n = body.faces[f].outward_normal(mid);
                    let out = -n.cross((near_b - near_a).normalized()).normalized();
                    let unbend = Frame::translation(out * neutral).compose(&Frame::rotation(o, a, turn));
                    let beyond = here.compose(&unbend);
                    let Some(q) = other(far, g) else {
                        return Err(Error::InvalidBody(vec![format!("the bend at {} has an open side", at3(mid))]));
                    };
                    if !matches!(body.faces[q].surface, Surface::Plane(_)) || !smooth(far, g, q) {
                        return Err(Error::Unsupported(format!(
                            "past the bend at {} there's no flat wall going on from it, which a flat pattern needs so far",
                            at3(mid)
                        )));
                    }
                    match placed.iter().find(|p| p.0 == q) {
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
