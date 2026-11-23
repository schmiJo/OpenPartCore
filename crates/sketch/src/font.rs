//! Fonts for text in sketches: a reader of TrueType outlines, written here so that no library is needed, and the font
//! this crate comes with, a part of Noto Sans.
//!
//! A TrueType glyph is closed contours of points, each on the curve or off it. Two points off the curve in a row have one
//! on it half way between them, so a run of points off the curve between two on it is a uniform quadratic B-spline with
//! those points as its control points, which is how each run is handed on: see `Piece`. Composite glyphs, such as an
//! accented letter made of a letter and an accent, are flattened into one list of contours when read.
//!
//! Only outlines, advances and the height of capitals are read. Hinting, kerning, ligatures and anything beyond a glyph per
//! character aren't: each character is set at its advance from the last.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

/// A point in font units.
pub type FontPoint = [f64; 2];

/// One character's outline, in font units with its origin on the baseline where the pen is.
#[derive(Clone, Debug, PartialEq)]
pub struct Glyph {
    /// How far the pen moves on after it.
    pub advance: f64,
    /// Closed contours of points, each on the curve or not, as TrueType has them.
    pub contours: Vec<Vec<(FontPoint, bool)>>,
}

/// A font's glyphs for the characters it has, and what's needed to set them.
pub struct Font {
    pub units_per_em: f64,
    /// The height of a capital letter above the baseline, which a text's height is: H's top, as ISO 3098 measures text.
    pub cap_height: f64,
    /// From one baseline to the next, for text of more than one line.
    pub line_spacing: f64,
    pub glyphs: HashMap<char, Glyph>,
    /// Each glyph's pieces, worked out the first time it's set: see `outline_pieces`.
    pieces: std::sync::Mutex<HashMap<char, Arc<Vec<Vec<Piece>>>>>,
}

impl Clone for Font {
    fn clone(&self) -> Font {
        Font::new(self.units_per_em, self.cap_height, self.line_spacing, self.glyphs.clone())
    }
}

impl PartialEq for Font {
    fn eq(&self, other: &Font) -> bool {
        (self.units_per_em, self.cap_height, self.line_spacing) == (other.units_per_em, other.cap_height, other.line_spacing) && self.glyphs == other.glyphs
    }
}

impl std::fmt::Debug for Font {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Font {{ {} glyphs, {} units per em }}", self.glyphs.len(), self.units_per_em)
    }
}

/// A piece of a contour of a glyph as set: a straight line, or a uniform quadratic B-spline through these control points,
/// starting and ending on the first and last.
#[derive(Clone, Debug, PartialEq)]
pub enum Piece {
    Line(FontPoint, FontPoint),
    Quadratic(Vec<FontPoint>),
}

/// Text as set: each contour's pieces, end to end round it, in font units with the first character's pen at the origin. A
/// glyph whose contours cross comes as the pieces of the outline they fill together, in no order: see `outline_pieces`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Set {
    pub contours: Vec<Vec<Piece>>,
    /// Characters the font hasn't got, which are left out, with a space's room left for each.
    pub missing: Vec<char>,
}

impl Font {
    pub fn new(units_per_em: f64, cap_height: f64, line_spacing: f64, glyphs: HashMap<char, Glyph>) -> Font {
        Font { units_per_em, cap_height, line_spacing, glyphs, pieces: Default::default() }
    }

    /// A glyph's outline as pieces that don't cross each other: its contours' pieces as they are, unless some cross or
    /// overlap, as an accent laid over a letter can. Then they're the outline of what the contours fill together, by
    /// TrueType's rule, which fills wherever they wind round a point at all, where the sketch's regions would fill every
    /// other level of nesting and leave the overlap empty. Worked out once for each glyph.
    pub fn outline_pieces(&self, ch: char, glyph: &Glyph) -> Arc<Vec<Vec<Piece>>> {
        if let Some(found) = self.pieces.lock().unwrap_or_else(|e| e.into_inner()).get(&ch) {
            return found.clone();
        }
        let contours: Vec<Vec<Piece>> = glyph.contours.iter().map(|c| pieces_of(c)).filter(|p| !p.is_empty()).collect();
        let made = Arc::new(filled_outline(&contours).unwrap_or(contours));
        self.pieces.lock().unwrap_or_else(|e| e.into_inner()).insert(ch, made.clone());
        made
    }

    /// The font this crate comes with: Noto Sans Regular's printable ASCII and Latin-1 characters and a few more, under
    /// the SIL Open Font License, which `fonts/OFL.txt` holds.
    pub fn builtin() -> &'static Arc<Font> {
        static FONT: OnceLock<Arc<Font>> = OnceLock::new();
        FONT.get_or_init(|| Arc::new(Font::from_compact(include_bytes!("../fonts/noto_sans.font")).expect("the built-in font reads")))
    }

    /// The names of the fonts there are: only the built-in one, for now.
    pub fn named(name: &str) -> Option<&'static Arc<Font>> {
        match name {
            "" | "sans" | "noto_sans" => Some(Font::builtin()),
            _ => None,
        }
    }

    /// How far the pen goes setting a text, in font units: its longest line's width, for placing it, such as centring it.
    pub fn advance(&self, text: &str) -> f64 {
        let space = self.glyphs.get(&' ').map_or(self.units_per_em / 4.0, |g| g.advance);
        let width = |line: &str| -> f64 {
            line.chars().map(|ch| self.glyphs.get(&ch).or_else(|| if ch == '\u{2300}' { self.glyphs.get(&'\u{D8}') } else { None }).map_or(space, |g| g.advance)).sum()
        };
        text.split('\n').map(width).fold(0.0, f64::max)
    }

    /// Sets a line of text, or lines separated by `\n`, each one `line_spacing` below the last.
    pub fn set(&self, text: &str) -> Set {
        let mut out = Set::default();
        let space = self.glyphs.get(&' ').map_or(self.units_per_em / 4.0, |g| g.advance);
        let (mut pen, mut line) = (0.0, 0.0);
        for ch in text.chars() {
            if ch == '\n' {
                pen = 0.0;
                line -= self.line_spacing;
                continue;
            }
            // The diameter sign, which few fonts have, as the slashed O drawings use for it.
            let Some(glyph) = self.glyphs.get(&ch).or_else(|| if ch == '\u{2300}' { self.glyphs.get(&'\u{D8}') } else { None }) else {
                if !ch.is_whitespace() && !out.missing.contains(&ch) {
                    out.missing.push(ch);
                }
                pen += space;
                continue;
            };
            let shown = if self.glyphs.contains_key(&ch) { ch } else { '\u{D8}' };
            let moved = |p: &FontPoint| [p[0] + pen, p[1] + line];
            for contour in self.outline_pieces(shown, glyph).iter() {
                out.contours.push(
                    contour
                        .iter()
                        .map(|piece| match piece {
                            Piece::Line(a, b) => Piece::Line(moved(a), moved(b)),
                            Piece::Quadratic(c) => Piece::Quadratic(c.iter().map(moved).collect()),
                        })
                        .collect(),
                );
            }
            pen += glyph.advance;
        }
        out
    }

    /// Reads a TrueType font, keeping the glyphs of the characters `wanted` picks out of those it maps.
    pub fn from_truetype(bytes: &[u8], wanted: impl Fn(char) -> bool) -> Result<Font, String> {
        let file = TrueType::new(bytes)?;
        let head = file.table(b"head")?;
        let units_per_em = f64::from(head.u16(18)?);
        let long_offsets = head.i16(50)? != 0;
        let glyph_count = usize::from(file.table(b"maxp")?.u16(4)?);
        let hhea = file.table(b"hhea")?;
        let (ascender, descender, gap) = (f64::from(hhea.i16(4)?), f64::from(hhea.i16(6)?), f64::from(hhea.i16(8)?));
        let metrics = usize::from(hhea.u16(34)?);
        let hmtx = file.table(b"hmtx")?;
        let advance = |g: usize| -> Result<f64, String> { Ok(f64::from(hmtx.u16(4 * g.min(metrics.max(1) - 1))?)) };
        let loca = file.table(b"loca")?;
        let glyf = file.table(b"glyf")?;
        let at = |g: usize| -> Result<(usize, usize), String> {
            if g >= glyph_count {
                return Err(format!("glyph {g} is past the font's {glyph_count}"));
            }
            Ok(if long_offsets {
                (loca.u32(4 * g)? as usize, loca.u32(4 * g + 4)? as usize)
            } else {
                (2 * usize::from(loca.u16(2 * g)?), 2 * usize::from(loca.u16(2 * g + 2)?))
            })
        };
        let map = file.character_map()?;
        let mut glyphs = HashMap::new();
        for (&ch, &g) in &map {
            if !wanted(ch) {
                continue;
            }
            let contours = outline(&glyf, &at, g, 0)?;
            glyphs.insert(ch, Glyph { advance: advance(g)?, contours });
        }
        // The height of capitals: the OS/2 table's, from its second version on, or else H's top.
        let os2_cap = file.table(b"OS/2").ok().and_then(|t| (t.u16(0).ok()? >= 2).then(|| t.i16(88).ok()).flatten()).map(f64::from).filter(|c| *c > 0.0);
        let h_top = glyphs.get(&'H').and_then(|g| g.contours.iter().flatten().map(|(p, _)| p[1]).reduce(f64::max));
        let cap_height = os2_cap.or(h_top).unwrap_or(0.7 * units_per_em);
        Ok(Font::new(units_per_em, cap_height, ascender - descender + gap, glyphs))
    }

    /// Writes the font in this crate's own compact form: what `from_compact` reads.
    pub fn to_compact(&self) -> Vec<u8> {
        let mut out = b"CSFONT1\0".to_vec();
        for v in [self.units_per_em, self.cap_height, self.line_spacing] {
            out.extend_from_slice(&(v as f32).to_le_bytes());
        }
        let mut chars: Vec<&char> = self.glyphs.keys().collect();
        chars.sort();
        out.extend_from_slice(&(chars.len() as u32).to_le_bytes());
        for ch in chars {
            let g = &self.glyphs[ch];
            out.extend_from_slice(&(*ch as u32).to_le_bytes());
            out.extend_from_slice(&(g.advance as f32).to_le_bytes());
            out.extend_from_slice(&(g.contours.len() as u16).to_le_bytes());
            for c in &g.contours {
                out.extend_from_slice(&(c.len() as u16).to_le_bytes());
                for (p, on) in c {
                    out.extend_from_slice(&(p[0] as f32).to_le_bytes());
                    out.extend_from_slice(&(p[1] as f32).to_le_bytes());
                    out.push(u8::from(*on));
                }
            }
        }
        out
    }

    /// Reads what `to_compact` writes.
    pub fn from_compact(bytes: &[u8]) -> Result<Font, String> {
        let mut r = Reader { bytes, at: 0 };
        if r.take(8)? != b"CSFONT1\0" {
            return Err("that isn't a font in this crate's compact form".into());
        }
        let (units_per_em, cap_height, line_spacing) = (r.f32()?, r.f32()?, r.f32()?);
        let count = r.u32()?;
        let mut glyphs = HashMap::new();
        for _ in 0..count {
            let ch = char::from_u32(r.u32()?).ok_or("a glyph for something that isn't a character")?;
            let advance = r.f32()?;
            let contours = (0..r.u16()?)
                .map(|_| (0..r.u16()?).map(|_| Ok(([r.f32()?, r.f32()?], r.byte()? != 0))).collect::<Result<Vec<_>, String>>())
                .collect::<Result<Vec<_>, String>>()?;
            glyphs.insert(ch, Glyph { advance, contours });
        }
        Ok(Font::new(units_per_em, cap_height, line_spacing, glyphs))
    }
}

/// The quadratic Beziers a contour's pieces are, lines as ones with their middle point off the curve.
fn beziers(contours: &[Vec<Piece>]) -> Vec<[FontPoint; 3]> {
    let mid = |a: FontPoint, b: FontPoint| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    let mut out = Vec::new();
    for piece in contours.iter().flatten() {
        match piece {
            Piece::Line(a, b) => out.push([*a, mid(*a, *b), *b]),
            Piece::Quadratic(c) => {
                let n = c.len();
                for k in 1..n - 1 {
                    let a = if k == 1 { c[0] } else { mid(c[k - 1], c[k]) };
                    let b = if k == n - 2 { c[n - 1] } else { mid(c[k], c[k + 1]) };
                    out.push([a, c[k], b]);
                }
            }
        }
    }
    out
}

/// How many times these closed curves wind round a point, counter-clockwise positive: their crossings of the ray from it
/// toward +x, each counted by which way the curve goes there, the crossing at a curve's start counted and at its end not.
fn winding(curves: &[[FontPoint; 3]], p: FontPoint) -> i32 {
    let mut n = 0;
    for [a, o, b] in curves {
        let (qa, qb, qc) = (a[1] - 2.0 * o[1] + b[1], 2.0 * (o[1] - a[1]), a[1] - p[1]);
        let mut roots = Vec::with_capacity(2);
        if qa.abs() < 1e-12 * (qb.abs() + qc.abs() + 1.0) {
            if qb != 0.0 {
                roots.push(-qc / qb);
            }
        } else {
            let disc = qb * qb - 4.0 * qa * qc;
            if disc >= 0.0 {
                let r = disc.sqrt();
                roots.push((-qb - r) / (2.0 * qa));
                roots.push((-qb + r) / (2.0 * qa));
            }
        }
        for t in roots {
            if !(0.0..1.0).contains(&t) {
                continue;
            }
            let x = (1.0 - t) * (1.0 - t) * a[0] + 2.0 * t * (1.0 - t) * o[0] + t * t * b[0];
            let dy = 2.0 * qa * t + qb;
            if x > p[0] && dy != 0.0 {
                n += if dy > 0.0 { 1 } else { -1 };
            }
        }
    }
    n
}

/// The outline of what crossing contours fill together, by TrueType's rule: the pieces between the faces they divide
/// the plane into that are filled and those that aren't. None if they cross nowhere, as every other level of nesting
/// is then what they fill, which is what a sketch's regions take, and no two filled faces meet.
fn filled_outline(contours: &[Vec<Piece>]) -> Option<Vec<Vec<Piece>>> {
    use crate::model::{Geometry, Sketch};
    use crate::regions::SegKind;
    let curves = beziers(contours);
    let mut sketch = Sketch::new();
    for [a, o, b] in &curves {
        let shape = crate::spline::SplineShape::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![*a, *o, *b], vec![1.0; 3]).ok()?;
        sketch.add(Geometry::Spline(Arc::new(shape)));
    }
    let found = sketch.arrangement();
    let size = curves.iter().flatten().fold(1.0f64, |m, p| m.max(p[0].abs()).max(p[1].abs()));
    // Whether each face is filled: by a point just inside it, a little to the left of its outline's first piece as it
    // runs round anticlockwise.
    let filled: Vec<bool> = found
        .regions
        .iter()
        .map(|r| {
            let seg = &r.outer.segs[0];
            let m = seg.middle();
            let t = match seg.kind {
                SegKind::Line { a, b } => [b[0] - a[0], b[1] - a[1]],
                SegKind::Spline { t0, t1, .. } => {
                    let v = crate::regions::velocity_at(&seg.kind, (t0 + t1) / 2.0);
                    if t1 >= t0 { v } else { [-v[0], -v[1]] }
                }
                _ => [1.0, 0.0],
            };
            let length = t[0].hypot(t[1]).max(1e-300);
            let side = if r.outer.signed_area() >= 0.0 { 1.0 } else { -1.0 };
            let step = 1e-6 * size * side;
            winding(&curves, [m[0] - t[1] / length * step, m[1] + t[0] / length * step]) != 0
        })
        .collect();
    // A piece between two filled faces is in both their outlines; one on the outline of what's filled is in one.
    let key = |p: [f64; 2]| ((p[0] * 1e6).round() as i64, (p[1] * 1e6).round() as i64);
    let mut count: HashMap<(i64, i64, i64, i64, i64, i64), (usize, crate::regions::Seg)> = HashMap::new();
    for (r, _) in found.regions.iter().zip(&filled).filter(|(_, f)| **f) {
        for seg in std::iter::once(&r.outer).chain(&r.holes).flat_map(|l| &l.segs) {
            let (a, b) = (key(seg.start()), key(seg.end()));
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            let m = key(seg.middle());
            count.entry((a.0, a.1, b.0, b.1, m.0, m.1)).or_insert((0, seg.clone())).0 += 1;
        }
    }
    // As they are, if every other level of nesting is what they fill and no two filled faces meet, as where contours
    // only nest. Two that meet would be regions of their own, touching, where one is wanted.
    let inside_edges = count.values().any(|(n, _)| *n > 1);
    if !inside_edges && found.regions.iter().zip(&filled).all(|(r, f)| (r.depth % 2 == 0) == *f) {
        return None;
    }
    let mut pieces = Vec::new();
    let mut kept: Vec<_> = count.into_values().filter(|(n, _)| *n == 1).map(|(_, seg)| seg).collect();
    kept.sort_by(|a, b| a.start().partial_cmp(&b.start()).unwrap_or(std::cmp::Ordering::Equal));
    for seg in kept {
        match &seg.kind {
            SegKind::Line { a, b } => pieces.push(Piece::Line(*a, *b)),
            SegKind::Spline { shape, t0, t1 } => {
                // The part of a Bezier from t0 to t1 is one too, its point off the curve where the tangents at its ends
                // meet: the blossom at (t0, t1).
                let c = &shape.points;
                let blossom = |u: f64, v: f64| {
                    let (w0, w1, w2) = ((1.0 - u) * (1.0 - v), (1.0 - u) * v + u * (1.0 - v), u * v);
                    [w0 * c[0][0] + w1 * c[1][0] + w2 * c[2][0], w0 * c[0][1] + w1 * c[1][1] + w2 * c[2][1]]
                };
                pieces.push(Piece::Quadratic(vec![blossom(*t0, *t0), blossom(*t0, *t1), blossom(*t1, *t1)]));
            }
            _ => return None,
        }
    }
    // Each piece's ends are where its own curve is at the crossings, which can be a hair from where its neighbour's is.
    // Ends that close are made one, so the outline closes exactly: a glyph's corners are a unit apart at the least.
    let mut ends: Vec<FontPoint> = Vec::new();
    let mut snap = |p: FontPoint| -> FontPoint {
        match ends.iter().find(|q| (q[0] - p[0]).hypot(q[1] - p[1]) <= 1e-3) {
            Some(q) => *q,
            None => {
                ends.push(p);
                p
            }
        }
    };
    for piece in &mut pieces {
        match piece {
            Piece::Line(a, b) => {
                *a = snap(*a);
                *b = snap(*b);
            }
            Piece::Quadratic(c) => {
                let n = c.len();
                c[0] = snap(c[0]);
                c[n - 1] = snap(c[n - 1]);
            }
        }
    }
    // A piece whose ends came together, such as a sliver where two curves just touch, is nothing.
    pieces.retain(|p| match p {
        Piece::Line(a, b) => a != b,
        Piece::Quadratic(c) => c[0] != c[c.len() - 1],
    });
    Some(vec![pieces])
}

/// A contour's pieces: lines between points on the curve next to each other, and each run of points off it, with the
/// points on it at either end, as a quadratic B-spline. Lines of no length are left out.
pub fn pieces_of(contour: &[(FontPoint, bool)]) -> Vec<Piece> {
    let n = contour.len();
    if n < 2 {
        return Vec::new();
    }
    // Start on the curve: at a point on it, or, with none, half way between the last point and the first.
    let mut points: Vec<(FontPoint, bool)> = match contour.iter().position(|p| p.1) {
        Some(k) => contour[k..].iter().chain(&contour[..k]).copied().collect(),
        None => {
            let (a, b) = (contour[n - 1].0, contour[0].0);
            std::iter::once(([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0], true)).chain(contour.iter().copied()).collect()
        }
    };
    points.push(points[0]);
    let mut out = Vec::new();
    let mut i = 0;
    while i + 1 < points.len() {
        let start = points[i].0;
        let mut j = i + 1;
        let mut controls = vec![start];
        while !points[j].1 {
            controls.push(points[j].0);
            j += 1;
        }
        let end = points[j].0;
        if controls.len() == 1 {
            if start != end {
                out.push(Piece::Line(start, end));
            }
        } else {
            controls.push(end);
            out.push(Piece::Quadratic(controls));
        }
        i = j;
    }
    out
}

/// A glyph's contours, with a composite's parts placed and flattened in.
fn outline(glyf: &Table, at: &impl Fn(usize) -> Result<(usize, usize), String>, g: usize, depth: usize) -> Result<Vec<Vec<(FontPoint, bool)>>, String> {
    if depth > 8 {
        return Err("a composite glyph nests too deep".into());
    }
    let (start, end) = at(g)?;
    if end <= start {
        return Ok(Vec::new());
    }
    let d = glyf.slice(start, end)?;
    let contours = d.i16(0)?;
    if contours >= 0 {
        let contours = contours as usize;
        let ends: Vec<usize> = (0..contours).map(|k| d.u16(10 + 2 * k).map(usize::from)).collect::<Result<_, _>>()?;
        let count = ends.last().map_or(0, |e| e + 1);
        let instructions = usize::from(d.u16(10 + 2 * contours)?);
        let mut p = 12 + 2 * contours + instructions;
        let mut flags = Vec::with_capacity(count);
        while flags.len() < count {
            let f = d.u8(p)?;
            p += 1;
            flags.push(f);
            if f & 8 != 0 {
                let repeat = d.u8(p)?;
                p += 1;
                for _ in 0..repeat {
                    flags.push(f);
                }
            }
        }
        flags.truncate(count);
        let mut coordinates = |short: u8, same: u8| -> Result<Vec<f64>, String> {
            let mut v = 0i32;
            let mut out = Vec::with_capacity(count);
            for &f in &flags {
                if f & short != 0 {
                    let delta = i32::from(d.u8(p)?);
                    p += 1;
                    v += if f & same != 0 { delta } else { -delta };
                } else if f & same == 0 {
                    v += i32::from(d.i16(p)?);
                    p += 2;
                }
                out.push(f64::from(v));
            }
            Ok(out)
        };
        let xs = coordinates(2, 16)?;
        let ys = coordinates(4, 32)?;
        let mut out = Vec::with_capacity(contours);
        let mut first = 0;
        for &last in &ends {
            if last < first || last >= count {
                return Err("a glyph's contour ends out of order".into());
            }
            out.push((first..=last).map(|k| ([xs[k], ys[k]], flags[k] & 1 != 0)).collect());
            first = last + 1;
        }
        return Ok(out);
    }
    // A composite: parts, each a glyph moved, and maybe scaled.
    let mut out = Vec::new();
    let mut p = 10;
    loop {
        let flags = d.u16(p)?;
        let part = usize::from(d.u16(p + 2)?);
        p += 4;
        let (dx, dy) = if flags & 1 != 0 {
            let r = (f64::from(d.i16(p)?), f64::from(d.i16(p + 2)?));
            p += 4;
            r
        } else {
            let r = (f64::from(d.u8(p)? as i8), f64::from(d.u8(p + 1)? as i8));
            p += 2;
            r
        };
        if flags & 2 == 0 {
            return Err("a composite glyph placed by matching points, which isn't read".into());
        }
        let f2dot14 = |p: usize| -> Result<f64, String> { Ok(f64::from(d.i16(p)?) / 16384.0) };
        let (a, b, c, e) = if flags & 8 != 0 {
            let s = f2dot14(p)?;
            p += 2;
            (s, 0.0, 0.0, s)
        } else if flags & 0x40 != 0 {
            let r = (f2dot14(p)?, 0.0, 0.0, f2dot14(p + 2)?);
            p += 4;
            r
        } else if flags & 0x80 != 0 {
            let r = (f2dot14(p)?, f2dot14(p + 2)?, f2dot14(p + 4)?, f2dot14(p + 6)?);
            p += 8;
            r
        } else {
            (1.0, 0.0, 0.0, 1.0)
        };
        for contour in outline(glyf, at, part, depth + 1)? {
            out.push(contour.into_iter().map(|([x, y], on)| ([a * x + c * y + dx, b * x + e * y + dy], on)).collect());
        }
        if flags & 0x20 == 0 {
            break;
        }
    }
    Ok(out)
}

/// A TrueType file's tables.
struct TrueType<'a> {
    bytes: &'a [u8],
    tables: HashMap<[u8; 4], (usize, usize)>,
}

impl<'a> TrueType<'a> {
    fn new(bytes: &'a [u8]) -> Result<TrueType<'a>, String> {
        let all = Table { bytes };
        let version = all.u32(0)?;
        if version != 0x0001_0000 && version != u32::from_be_bytes(*b"true") {
            return Err("that isn't a TrueType font: its outlines would be TrueType's, and an OpenType font with PostScript outlines isn't read".into());
        }
        let count = usize::from(all.u16(4)?);
        let mut tables = HashMap::new();
        for k in 0..count {
            let at = 12 + 16 * k;
            let tag = all.slice(at, at + 4)?.bytes;
            let (offset, length) = (all.u32(at + 8)? as usize, all.u32(at + 12)? as usize);
            if offset.checked_add(length).is_none_or(|e| e > bytes.len()) {
                return Err("a table runs past the end of the font".into());
            }
            tables.insert([tag[0], tag[1], tag[2], tag[3]], (offset, length));
        }
        Ok(TrueType { bytes, tables })
    }

    fn table(&self, tag: &[u8; 4]) -> Result<Table<'a>, String> {
        let (offset, length) = self.tables.get(tag).ok_or_else(|| format!("the font has no {} table", String::from_utf8_lossy(tag)))?;
        Ok(Table { bytes: &self.bytes[*offset..offset + length] })
    }

    /// Characters to glyphs: from a Unicode subtable, format 12 for all of Unicode, or format 4 for its first plane.
    fn character_map(&self) -> Result<HashMap<char, usize>, String> {
        let cmap = self.table(b"cmap")?;
        let count = usize::from(cmap.u16(2)?);
        let mut best: Option<(u32, usize)> = None;
        for k in 0..count {
            let (platform, encoding) = (cmap.u16(4 + 8 * k)?, cmap.u16(6 + 8 * k)?);
            let offset = cmap.u32(8 + 8 * k)? as usize;
            let format = cmap.u16(offset)?;
            let unicode = platform == 0 || (platform == 3 && (encoding == 1 || encoding == 10));
            let rank = match format {
                12 => 2,
                4 => 1,
                _ => 0,
            };
            if unicode && rank > 0 && best.is_none_or(|b| rank > b.0) {
                best = Some((rank, offset));
            }
        }
        let (rank, offset) = best.ok_or("the font maps no Unicode characters in a way that's read")?;
        let mut out = HashMap::new();
        if rank == 2 {
            let groups = cmap.u32(offset + 12)? as usize;
            for k in 0..groups {
                let at = offset + 16 + 12 * k;
                let (first, last, glyph) = (cmap.u32(at)?, cmap.u32(at + 4)?, cmap.u32(at + 8)?);
                for c in first..=last.min(first.saturating_add(0x1_0000)) {
                    if let Some(ch) = char::from_u32(c) {
                        out.insert(ch, (glyph + (c - first)) as usize);
                    }
                }
            }
        } else {
            let segments = usize::from(cmap.u16(offset + 6)? / 2);
            let ends = offset + 14;
            let starts = ends + 2 * segments + 2;
            let deltas = starts + 2 * segments;
            let ranges = deltas + 2 * segments;
            for s in 0..segments {
                let (end, start) = (cmap.u16(ends + 2 * s)?, cmap.u16(starts + 2 * s)?);
                let delta = cmap.u16(deltas + 2 * s)?;
                let range = usize::from(cmap.u16(ranges + 2 * s)?);
                for c in start..=end {
                    if c == 0xFFFF {
                        break;
                    }
                    let glyph = if range == 0 {
                        c.wrapping_add(delta)
                    } else {
                        let at = ranges + 2 * s + range + 2 * usize::from(c - start);
                        match cmap.u16(at)? {
                            0 => 0,
                            g => g.wrapping_add(delta),
                        }
                    };
                    if glyph != 0 {
                        if let Some(ch) = char::from_u32(u32::from(c)) {
                            out.insert(ch, usize::from(glyph));
                        }
                    }
                }
            }
        }
        Ok(out)
    }
}

/// Part of a font file, read big-endian, refusing to read past its end.
#[derive(Clone, Copy)]
struct Table<'a> {
    bytes: &'a [u8],
}

impl<'a> Table<'a> {
    fn slice(&self, a: usize, b: usize) -> Result<Table<'a>, String> {
        self.bytes.get(a..b).map(|bytes| Table { bytes }).ok_or_else(|| "the font ends in the middle of something".to_string())
    }
    fn u8(&self, at: usize) -> Result<u8, String> {
        self.bytes.get(at).copied().ok_or_else(|| "the font ends in the middle of something".to_string())
    }
    fn u16(&self, at: usize) -> Result<u16, String> {
        Ok(u16::from_be_bytes([self.u8(at)?, self.u8(at + 1)?]))
    }
    fn i16(&self, at: usize) -> Result<i16, String> {
        Ok(self.u16(at)? as i16)
    }
    fn u32(&self, at: usize) -> Result<u32, String> {
        Ok(u32::from_be_bytes([self.u8(at)?, self.u8(at + 1)?, self.u8(at + 2)?, self.u8(at + 3)?]))
    }
}

/// Reads this crate's compact form, little-endian.
struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        let out = self.bytes.get(self.at..self.at + n).ok_or("the font ends early")?;
        self.at += n;
        Ok(out)
    }
    fn byte(&mut self) -> Result<u8, String> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, String> {
        let b = self.take(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }
    fn u32(&mut self) -> Result<u32, String> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    fn f32(&mut self) -> Result<f64, String> {
        let b = self.take(4)?;
        Ok(f64::from(f32::from_le_bytes([b[0], b[1], b[2], b[3]])))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runs_off_the_curve_are_quadratic_splines() {
        // A square with one corner rounded by two points off the curve.
        let contour = [([0.0, 0.0], true), ([10.0, 0.0], true), ([10.0, 5.0], false), ([5.0, 10.0], false), ([0.0, 10.0], true)];
        let pieces = pieces_of(&contour);
        assert_eq!(
            pieces,
            vec![
                Piece::Line([0.0, 0.0], [10.0, 0.0]),
                Piece::Quadratic(vec![[10.0, 0.0], [10.0, 5.0], [5.0, 10.0], [0.0, 10.0]]),
                Piece::Line([0.0, 10.0], [0.0, 0.0]),
            ]
        );
        // All off the curve: it starts half way between the last and the first.
        let pieces = pieces_of(&[([0.0, 0.0], false), ([10.0, 0.0], false), ([10.0, 10.0], false), ([0.0, 10.0], false)]);
        assert_eq!(pieces, vec![Piece::Quadratic(vec![[0.0, 5.0], [0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [0.0, 5.0]])]);
    }

    #[test]
    fn the_builtin_font_sets_text() {
        let font = Font::builtin();
        assert!(font.glyphs.len() > 190, "{}", font.glyphs.len());
        for ch in ['A', 'g', '0', '?', 'é', 'Ø', '°', '±'] {
            assert!(font.glyphs.contains_key(&ch), "{ch}");
        }
        let set = font.set("Ho");
        // H has one contour, o two.
        assert_eq!(set.contours.len(), 3);
        assert!(set.missing.is_empty());
        assert_eq!(font.set("a☃b").missing, vec!['☃']);
        // The compact form reads back as it was.
        assert_eq!(&Font::from_compact(&font.to_compact()).unwrap(), font.as_ref());
    }
}
