//! Engineering drawings: views of a part or an assembly projected onto a sheet, with hidden lines, sections, center
//! marks and lines, dimensions, a border and a title block, written as SVG, PDF or DXF, or drawn as a PNG. The views
//! come from the body each time a drawing is made, so a drawing follows its part. Everything here is written here:
//! the projection, the layout and every file format.
//!
//! A drawing is plain data, `Drawing`, which reads and writes as JSON. `page` turns it and a body into a `Page` of lines,
//! filled shapes and text, in millimetres on the sheet, and `out` writes a page.

pub mod hlr;
pub mod out;

use coreshape::error::{Error, Result};
use coreshape::math::*;
use coreshape::Body;
use coreshape_document::json::Value;
use hlr::{ViewFrame, ViewLines};

/// Standard sheets, landscape, in millimetres.
pub const SHEETS: &[(&str, f64, f64)] = &[
    ("A4", 297.0, 210.0),
    ("A3", 420.0, 297.0),
    ("A2", 594.0, 420.0),
    ("A1", 841.0, 594.0),
    ("A0", 1189.0, 841.0),
    ("ANSI A", 279.4, 215.9),
    ("ANSI B", 431.8, 279.4),
    ("ANSI C", 558.8, 431.8),
    ("ANSI D", 863.6, 558.8),
    ("ANSI E", 1117.6, 863.6),
];

/// Scales a drawing picks from when it's left to choose, largest first.
pub const SCALES: &[f64] = &[50.0, 20.0, 10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001];

/// Which way round views are laid out about the front view, as ISO 128 has them: first angle in Europe, where the top
/// view goes below the front and the view from the right to its left, or third angle in America, the other way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Projection {
    FirstAngle,
    ThirdAngle,
}

/// What a view looks at.
#[derive(Clone, Debug, PartialEq)]
pub enum ViewKind {
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Iso,
    /// The part cut by a plane through `point` square to `normal`, with the half `normal` points to taken away, seen
    /// from that side. Its cut faces are hatched, and it's labelled with its letter.
    Section { point: Vec3, normal: Vec3, letter: String },
}

impl ViewKind {
    pub fn frame(&self) -> ViewFrame {
        match self {
            ViewKind::Front => ViewFrame::looking(-Vec3::Y, Vec3::Z),
            ViewKind::Back => ViewFrame::looking(Vec3::Y, Vec3::Z),
            ViewKind::Top => ViewFrame::looking(Vec3::Z, Vec3::Y),
            ViewKind::Bottom => ViewFrame::looking(-Vec3::Z, -Vec3::Y),
            ViewKind::Right => ViewFrame::looking(Vec3::X, Vec3::Z),
            ViewKind::Left => ViewFrame::looking(-Vec3::X, Vec3::Z),
            ViewKind::Iso => ViewFrame::looking(v3(1.0, -1.0, 1.0), Vec3::Z),
            ViewKind::Section { normal, .. } => ViewFrame::looking(*normal, if normal.cross(Vec3::Z).norm() > 1e-9 { Vec3::Z } else { Vec3::Y }),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            ViewKind::Front => "front",
            ViewKind::Back => "back",
            ViewKind::Top => "top",
            ViewKind::Bottom => "bottom",
            ViewKind::Left => "left",
            ViewKind::Right => "right",
            ViewKind::Iso => "iso",
            ViewKind::Section { .. } => "section",
        }
    }

    /// Where it goes about the front view, by column and row, rows going down, for the projections that say.
    fn cell(&self, projection: Projection) -> Option<(i32, i32)> {
        let first = projection == Projection::FirstAngle;
        Some(match self {
            ViewKind::Front => (0, 0),
            ViewKind::Top => (0, if first { 1 } else { -1 }),
            ViewKind::Bottom => (0, if first { -1 } else { 1 }),
            ViewKind::Right => (if first { -1 } else { 1 }, 0),
            ViewKind::Left => (if first { 1 } else { -1 }, 0),
            ViewKind::Back => (2, 0),
            _ => return None,
        })
    }
}

/// A view on the sheet.
#[derive(Clone, Debug, PartialEq)]
pub struct View {
    pub id: u32,
    pub kind: ViewKind,
    /// Its middle on the sheet, in millimetres from the bottom left, or left to the layout.
    pub at: Option<[f64; 2]>,
    /// Its own scale, or the sheet's.
    pub scale: Option<f64>,
    /// Hidden lines, dashed: by default for the six views, not for iso and sections.
    pub hidden: Option<bool>,
}

impl View {
    fn shows_hidden(&self) -> bool {
        self.hidden.unwrap_or(!matches!(self.kind, ViewKind::Iso | ViewKind::Section { .. }))
    }
}

/// A point a dimension measures from: a place in the model, or something the caller looks up, such as a sketch's point
/// or a part's corner, kept as it was given so that it's looked up again each time.
#[derive(Clone, Debug, PartialEq)]
pub enum Spot {
    At(Vec3),
    Ref(Value),
}

/// Which way a distance is measured on the sheet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Along {
    Horizontal,
    Vertical,
    Aligned,
}

#[derive(Clone, Debug, PartialEq)]
pub enum DimKind {
    /// Between two points, as the view shows them.
    Distance { a: Spot, b: Spot, along: Along },
    /// A circle seen face on in the view, the one whose rim or center is nearest the point.
    Diameter { near: Spot },
    Radius { near: Spot },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Dimension {
    pub id: u32,
    pub view: u32,
    pub kind: DimKind,
    /// How far the dimension line is from what it measures, in millimetres on the sheet. For a diameter or radius, how
    /// far out its text goes.
    pub offset: f64,
    /// Text in place of the measured value: `<>` in it stands for the value.
    pub text: Option<String>,
}

/// What the title block says. Empty fields are left blank.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Title {
    pub title: String,
    pub number: String,
    pub part: String,
    pub material: String,
    pub author: String,
    pub date: String,
    pub company: String,
    pub revision: String,
}

/// A drawing: a sheet, its views and dimensions, and what its title block says.
#[derive(Clone, Debug, PartialEq)]
pub struct Drawing {
    pub sheet: String,
    pub projection: Projection,
    /// The sheet's scale, as a drawing at 1:2 has 0.5, or left to pick the largest standard one that fits.
    pub scale: Option<f64>,
    pub views: Vec<View>,
    pub dimensions: Vec<Dimension>,
    /// Each of the six views gets its overall width and height dimensioned, and each circle seen face on its diameter.
    pub auto_dimensions: bool,
    pub title: Title,
    pub next_id: u32,
}

impl Default for Drawing {
    fn default() -> Drawing {
        Drawing { sheet: "A3".into(), projection: Projection::FirstAngle, scale: None, views: Vec::new(), dimensions: Vec::new(), auto_dimensions: true, title: Title::default(), next_id: 1 }
    }
}

impl Drawing {
    /// The usual three views and an iso one.
    pub fn standard() -> Drawing {
        let mut d = Drawing::default();
        for kind in [ViewKind::Front, ViewKind::Top, ViewKind::Right, ViewKind::Iso] {
            d.add_view(kind, None);
        }
        d
    }

    fn fresh(&mut self) -> u32 {
        let id = self.next_id.max(1);
        self.next_id = id + 1;
        id
    }

    pub fn add_view(&mut self, kind: ViewKind, at: Option<[f64; 2]>) -> u32 {
        let id = self.fresh();
        self.views.push(View { id, kind, at, scale: None, hidden: None });
        id
    }

    pub fn add_dimension(&mut self, view: u32, kind: DimKind, offset: f64, text: Option<String>) -> Result<u32> {
        if !self.views.iter().any(|v| v.id == view) {
            return Err(Error::InvalidInput(format!("this drawing has no view {view}")));
        }
        let id = self.fresh();
        self.dimensions.push(Dimension { id, view, kind, offset, text });
        Ok(id)
    }

    /// The sheet's size, in millimetres.
    pub fn size(&self) -> Result<(f64, f64)> {
        sheet_size(&self.sheet).ok_or_else(|| Error::InvalidInput(format!("unknown sheet \"{}\"; expected {}", self.sheet, SHEETS.iter().map(|s| s.0).collect::<Vec<_>>().join(", "))))
    }
}

pub fn sheet_size(name: &str) -> Option<(f64, f64)> {
    let n = name.trim().to_ascii_uppercase().replace('_', " ");
    SHEETS.iter().find(|s| s.0 == n || s.0.replace(' ', "") == n.replace(' ', "")).map(|s| (s.1, s.2))
}

// ----- The page -----

/// How a line is drawn, as ISO 128 has them: thick for what's seen, thin for dimensions, hatching and smooth edges,
/// dashed for what's hidden, and long and short dashes for centers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pen {
    Thick,
    Thin,
    Hidden,
    Center,
    Frame,
}

impl Pen {
    /// Its width in millimetres.
    pub fn width(self) -> f64 {
        match self {
            Pen::Thick => 0.5,
            Pen::Thin | Pen::Center => 0.25,
            Pen::Hidden => 0.35,
            Pen::Frame => 0.7,
        }
    }

    /// Its dashes, as lengths on and off, in millimetres.
    pub fn dashes(self) -> &'static [f64] {
        match self {
            Pen::Hidden => &[3.0, 1.5],
            Pen::Center => &[12.0, 1.5, 1.0, 1.5],
            _ => &[],
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Pen::Thick => "VISIBLE",
            Pen::Thin => "THIN",
            Pen::Hidden => "HIDDEN",
            Pen::Center => "CENTER",
            Pen::Frame => "FRAME",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Align {
    Left,
    Center,
    Right,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Text {
    pub at: Vec2,
    /// Capital height, in millimetres.
    pub height: f64,
    pub text: String,
    pub align: Align,
    /// Turned anticlockwise, in degrees.
    pub angle: f64,
}

/// A drawing as it goes on paper: lines, filled shapes and text, in millimetres from the sheet's bottom left.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Page {
    pub width: f64,
    pub height: f64,
    pub lines: Vec<(Pen, Vec<Vec2>)>,
    pub fills: Vec<Vec<Vec2>>,
    pub texts: Vec<Text>,
    /// What couldn't be drawn, and why, such as a dimension whose point has gone.
    pub notes: Vec<String>,
    /// The scale the sheet is at.
    pub scale: f64,
}

impl Page {
    fn line(&mut self, pen: Pen, points: Vec<Vec2>) {
        if points.len() >= 2 {
            self.lines.push((pen, points));
        }
    }

    fn text(&mut self, at: Vec2, height: f64, text: impl Into<String>, align: Align, angle: f64) {
        self.texts.push(Text { at, height, text: text.into(), align, angle });
    }

    /// An arrowhead with its tip at `tip`, pointing along `dir`, as ISO 129 draws one: 3 long and 1 wide, filled.
    fn arrow(&mut self, tip: Vec2, dir: Vec2) {
        let d = dir.normalized();
        let side = v2(-d.y, d.x);
        let back = tip - d * 3.0;
        self.fills.push(vec![tip, back + side * 0.5, back - side * 0.5]);
    }
}

/// A view placed on the sheet: how its model coordinates map onto it.
struct Placed<'a> {
    view: &'a View,
    frame: ViewFrame,
    lines: ViewLines,
    /// The model point, in the view's plane, at the view's middle, and where that goes on the sheet.
    middle: Vec2,
    at: Vec2,
    scale: f64,
}

impl Placed<'_> {
    fn sheet(&self, q: Vec2) -> Vec2 {
        self.at + (q - self.middle) * self.scale
    }

    fn of(&self, p: Vec3) -> Vec2 {
        self.sheet(self.frame.flat(p))
    }

    /// Its extent on the sheet.
    fn bounds(&self) -> (Vec2, Vec2) {
        (self.sheet(self.lines.lo), self.sheet(self.lines.hi))
    }
}

/// Formats a length as drawings write it: to two places, without trailing zeros.
pub fn format_length(x: f64) -> String {
    let s = format!("{:.2}", x);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

/// The scale as drawings write it: 1:2, 1:1, 5:1.
pub fn format_scale(s: f64) -> String {
    if s >= 1.0 {
        format!("{}:1", format_length(s))
    } else {
        format!("1:{}", format_length(1.0 / s))
    }
}

/// The margins and title block of a sheet, as ISO 5457 and 7200 have them: a border 20 in from the left, for filing, and
/// 10 in elsewhere, and a title block 180 wide in the bottom right corner.
const LEFT: f64 = 20.0;
const EDGE: f64 = 10.0;
const BLOCK_W: f64 = 180.0;
const BLOCK_H: f64 = 40.0;
/// Room between views, and around each for its dimensions.
const GAP: f64 = 22.0;

/// Lays a drawing out on its sheet, from the body it shows. `locate` looks up a dimension's points given as references.
pub fn page(drawing: &Drawing, body: &Body, locate: &dyn Fn(&Value) -> Result<Vec3>) -> Result<Page> {
    let (w, h) = drawing.size()?;
    let mut page = Page { width: w, height: h, ..Page::default() };
    if drawing.views.is_empty() {
        return Err(Error::InvalidInput("a drawing needs a view".into()));
    }
    // Each view's lines, in the model's units.
    let size = body.aabb().diagonal().max(1e-9);
    // Fine enough for the largest scale it might be drawn at: a twentieth of a millimetre on the sheet.
    let largest = drawing.scale.unwrap_or(SCALES[0]).max(drawing.views.iter().filter_map(|v| v.scale).fold(0.0, f64::max));
    let chord = hlr::mesh_chord(body, 0.05 / largest.max(1e-9)).min(size * 2e-3);
    let mut views: Vec<(&View, ViewFrame, ViewLines)> = Vec::with_capacity(drawing.views.len());
    for v in &drawing.views {
        let frame = v.kind.frame();
        let lines = match &v.kind {
            ViewKind::Section { point, normal, letter } => {
                let cut = hlr::section(body, *point, *normal).map_err(|e| Error::InvalidInput(format!("section {letter}: {e}")))?;
                if cut.faces.is_empty() {
                    return Err(Error::InvalidInput(format!("section {letter}'s plane misses the part")));
                }
                hlr::project(&cut, &frame, chord, v.shows_hidden())?
            }
            _ => hlr::project(body, &frame, chord, v.shows_hidden())?,
        };
        views.push((v, frame, lines));
    }
    // The scale: the largest standard one at which the layout fits the space above the title block, or as given.
    let area = (v2(LEFT, EDGE + BLOCK_H), v2(w - EDGE, h - EDGE));
    let cells = layout_cells(&views, drawing.projection);
    let fits = |s: f64| -> bool {
        let (cw, ch) = grid_size(&views, &cells, s);
        cw <= area.1.x - area.0.x && ch <= area.1.y - area.0.y
    };
    let scale = match drawing.scale {
        Some(s) if s.is_finite() && s > 0.0 => s,
        Some(s) => return Err(Error::InvalidInput(format!("a drawing's scale must be above 0, not {s}"))),
        None => SCALES.iter().copied().find(|s| fits(*s)).unwrap_or(SCALES[SCALES.len() - 1]),
    };
    page.scale = scale;
    // Where each view goes: on the grid, centered in the space, or where it's told.
    let positions = grid_positions(&views, &cells, scale, area);
    let placed: Vec<Placed> = views
        .into_iter()
        .zip(positions)
        .map(|((view, frame, lines), at)| {
            let s = scale * view.scale.unwrap_or(1.0);
            let middle = (lines.lo + lines.hi) * 0.5;
            let at = view.at.map_or(at, |p| v2(p[0], p[1]));
            Placed { view, frame, lines, middle, at, scale: s }
        })
        .collect();
    for p in &placed {
        draw_view(&mut page, p);
    }
    // Where each section is cut, on the first of the six views that sees the plane edge on.
    for p in &placed {
        if let ViewKind::Section { point, normal, letter } = &p.view.kind {
            if let Some(on) = placed.iter().find(|q| q.view.kind.cell(drawing.projection).is_some() && q.frame.toward.dot(*normal).abs() < 1e-9) {
                cutting_line(&mut page, on, *point, *normal, letter);
            }
        }
    }
    // Dimensions. A circle given one here isn't given another automatically.
    let mut circles_done: Vec<(u32, Vec2, f64)> = Vec::new();
    for d in &drawing.dimensions {
        let Some(p) = placed.iter().find(|p| p.view.id == d.view) else {
            page.notes.push(format!("dimension {}: there's no view {}", d.id, d.view));
            continue;
        };
        match dimension(&mut page, p, d, locate) {
            Ok(Some((center, radius))) => circles_done.push((p.view.id, center, radius)),
            Ok(None) => {}
            Err(why) => page.notes.push(format!("dimension {}: {why}", d.id)),
        }
    }
    if drawing.auto_dimensions {
        for p in placed.iter().filter(|p| p.view.kind.cell(drawing.projection).is_some()) {
            let done: Vec<(Vec2, f64)> = circles_done.iter().filter(|c| c.0 == p.view.id).map(|c| (c.1, c.2)).collect();
            auto_dimensions(&mut page, p, done);
        }
    }
    border_and_title(&mut page, drawing, scale);
    Ok(page)
}

/// Each view's cell: the six by projection, then iso and sections in the cells to the right, top first.
fn layout_cells(views: &[(&View, ViewFrame, ViewLines)], projection: Projection) -> Vec<(i32, i32)> {
    let wanted: Vec<Option<(i32, i32)>> = views.iter().map(|(v, ..)| v.kind.cell(projection)).collect();
    let right = wanted.iter().flatten().map(|c| c.0).max().unwrap_or(0);
    let mut out: Vec<(i32, i32)> = Vec::with_capacity(views.len());
    let mut spare = (0..).flat_map(|k: i32| [(right + 1 + k, -1), (right + 1 + k, 0), (right + 1 + k, 1)]);
    for w in wanted {
        let mut c = match w {
            Some(c) => c,
            None => spare.next().expect("there are always more cells"),
        };
        // One of the six asked for twice goes on along its row.
        while out.contains(&c) {
            c = match w {
                Some(_) => (c.0 + 1, c.1),
                None => spare.next().expect("there are always more cells"),
            };
        }
        out.push(c);
    }
    out
}

/// The widths of the grid's columns and the heights of its rows at a scale, with the gaps.
fn grid_extents(views: &[(&View, ViewFrame, ViewLines)], cells: &[(i32, i32)], scale: f64) -> (Vec<(i32, f64)>, Vec<(i32, f64)>) {
    let (mut cols, mut rows): (Vec<(i32, f64)>, Vec<(i32, f64)>) = (Vec::new(), Vec::new());
    for ((v, _, lines), &(c, r)) in views.iter().zip(cells) {
        let s = scale * v.scale.unwrap_or(1.0);
        let (wide, tall) = ((lines.hi.x - lines.lo.x) * s + GAP, (lines.hi.y - lines.lo.y) * s + GAP);
        match cols.iter_mut().find(|x| x.0 == c) {
            Some(x) => x.1 = x.1.max(wide),
            None => cols.push((c, wide)),
        }
        match rows.iter_mut().find(|x| x.0 == r) {
            Some(x) => x.1 = x.1.max(tall),
            None => rows.push((r, tall)),
        }
    }
    cols.sort_by_key(|x| x.0);
    rows.sort_by_key(|x| x.0);
    (cols, rows)
}

fn grid_size(views: &[(&View, ViewFrame, ViewLines)], cells: &[(i32, i32)], scale: f64) -> (f64, f64) {
    let (cols, rows) = grid_extents(views, cells, scale);
    (cols.iter().map(|c| c.1).sum(), rows.iter().map(|r| r.1).sum())
}

/// The middle of each view's cell, with the grid centered in the area.
fn grid_positions(views: &[(&View, ViewFrame, ViewLines)], cells: &[(i32, i32)], scale: f64, area: (Vec2, Vec2)) -> Vec<Vec2> {
    let (cols, rows) = grid_extents(views, cells, scale);
    let (total_w, total_h) = (cols.iter().map(|c| c.1).sum::<f64>(), rows.iter().map(|r| r.1).sum::<f64>());
    let left = (area.0.x + area.1.x - total_w) / 2.0;
    let top = (area.0.y + area.1.y + total_h) / 2.0;
    cells
        .iter()
        .map(|&(c, r)| {
            let mut x = left;
            for col in &cols {
                if col.0 == c {
                    x += col.1 / 2.0;
                    break;
                }
                x += col.1;
            }
            let mut y = top;
            for row in &rows {
                if row.0 == r {
                    y -= row.1 / 2.0;
                    break;
                }
                y -= row.1;
            }
            v2(x, y)
        })
        .collect()
}

fn draw_view(page: &mut Page, p: &Placed) {
    let map = |run: &Vec<Vec2>| run.iter().map(|q| p.sheet(*q)).collect::<Vec<_>>();
    for run in &p.lines.hidden {
        page.line(Pen::Hidden, map(run));
    }
    for run in &p.lines.tangent {
        page.line(Pen::Thin, map(run));
    }
    for run in &p.lines.visible {
        page.line(Pen::Thick, map(run));
    }
    // Center marks on circles seen face on, at least half round, and center lines along axes seen side on, each 3 past.
    for c in &p.lines.circles {
        if c.sweep < PI - 1e-6 {
            continue;
        }
        let (o, r) = (p.sheet(c.center), c.radius * p.scale + 3.0);
        page.line(Pen::Center, vec![o - v2(r, 0.0), o + v2(r, 0.0)]);
        page.line(Pen::Center, vec![o - v2(0.0, r), o + v2(0.0, r)]);
    }
    for (a, b) in &p.lines.axes {
        let (a, b) = (p.sheet(*a), p.sheet(*b));
        let d = (b - a).normalized();
        page.line(Pen::Center, vec![a - d * 3.0, b + d * 3.0]);
    }
    // Hatching on cut faces: thin lines at 45 degrees, 3 apart.
    if !p.lines.cut.is_empty() {
        let tris: Vec<[Vec2; 3]> = p.lines.cut.iter().map(|t| [p.sheet(t[0]), p.sheet(t[1]), p.sheet(t[2])]).collect();
        hatch(page, &tris, 3.0);
    }
    if let ViewKind::Section { letter, .. } = &p.view.kind {
        let (lo, _) = p.bounds();
        let middle = p.sheet((p.lines.lo + p.lines.hi) * 0.5);
        page.text(v2(middle.x, lo.y - 10.0), 5.0, format!("SECTION {letter}-{letter}"), Align::Center, 0.0);
    }
}

/// Hatches the area the triangles cover, with lines at 45 degrees `spacing` apart.
fn hatch(page: &mut Page, tris: &[[Vec2; 3]], spacing: f64) {
    // Along u = (1, 1) / sqrt 2, lines at v = (-1, 1) / sqrt 2 steps.
    let (u, v) = (v2(1.0, 1.0) * std::f64::consts::FRAC_1_SQRT_2, v2(-1.0, 1.0) * std::f64::consts::FRAC_1_SQRT_2);
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for t in tris {
        for p in t {
            lo = lo.min(p.dot(v));
            hi = hi.max(p.dot(v));
        }
    }
    let mut k = (lo / spacing).ceil();
    while k * spacing <= hi {
        let at = k * spacing;
        // Where the line crosses each triangle, as a stretch along u, joined where they touch.
        let mut spans: Vec<(f64, f64)> = Vec::new();
        for t in tris {
            let mut hits = Vec::new();
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                let (da, db) = (a.dot(v) - at, b.dot(v) - at);
                if (da <= 0.0 && db > 0.0) || (db <= 0.0 && da > 0.0) {
                    let s = da / (da - db);
                    hits.push((a + (b - a) * s).dot(u));
                }
            }
            if hits.len() == 2 {
                spans.push((hits[0].min(hits[1]), hits[0].max(hits[1])));
            }
        }
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for s in spans {
            match merged.last_mut() {
                Some(m) if s.0 <= m.1 + 1e-6 => m.1 = m.1.max(s.1),
                _ => merged.push(s),
            }
        }
        for (a, b) in merged {
            if b - a > 1e-3 {
                page.line(Pen::Thin, vec![u * a + v * at, u * b + v * at]);
            }
        }
        k += 1.0;
    }
}

/// The line a section is cut along, on a view that sees its plane edge on: long and short dashes, thick at the ends, with
/// arrows showing which way the section looks, and its letter.
fn cutting_line(page: &mut Page, on: &Placed, point: Vec3, normal: Vec3, letter: &str) {
    let along = on.frame.toward.cross(normal).normalized();
    let reach = (on.lines.hi - on.lines.lo).norm();
    let (a, b) = (on.of(point - along * reach), on.of(point + along * reach));
    let (lo, hi) = on.bounds();
    // Clipped to a little past the view.
    let clip = |p: Vec2| v2(p.x.clamp(lo.x - 5.0, hi.x + 5.0), p.y.clamp(lo.y - 5.0, hi.y + 5.0));
    let (a, b) = (clip(a), clip(b));
    page.line(Pen::Center, vec![a, b]);
    let d = (b - a).normalized();
    page.line(Pen::Frame, vec![a, a + d * 5.0]);
    page.line(Pen::Frame, vec![b - d * 5.0, b]);
    // The arrows point the way the section looks, against its normal.
    let look = on.frame.flat(-normal).normalized();
    for end in [a, b] {
        let foot = end + d * if end == a { 2.5 } else { -2.5 };
        page.line(Pen::Thin, vec![foot, foot + look * 8.0]);
        page.arrow(foot + look * 8.0, look);
        page.text(foot + look * 12.0 - v2(0.0, 2.5), 5.0, letter, Align::Center, 0.0);
    }
}

fn spot(s: &Spot, locate: &dyn Fn(&Value) -> Result<Vec3>) -> Result<Vec3> {
    match s {
        Spot::At(p) => Ok(*p),
        Spot::Ref(v) => locate(v),
    }
}

fn shown(text: &Option<String>, value: String) -> String {
    match text {
        Some(t) => t.replace("<>", &value),
        None => value,
    }
}

/// Draws a dimension, giving the circle it's on, if it's a diameter or radius.
fn dimension(page: &mut Page, p: &Placed, d: &Dimension, locate: &dyn Fn(&Value) -> Result<Vec3>) -> Result<Option<(Vec2, f64)>> {
    match &d.kind {
        DimKind::Distance { a, b, along } => {
            let (pa, pb) = (spot(a, locate)?, spot(b, locate)?);
            let (fa, fb) = (p.frame.flat(pa), p.frame.flat(pb));
            let dir = match along {
                Along::Horizontal => v2(1.0, 0.0),
                Along::Vertical => v2(0.0, 1.0),
                Along::Aligned => {
                    if fa.dist(fb) < 1e-12 {
                        return Err(Error::InvalidInput("its two points are at one place in this view".into()));
                    }
                    (fb - fa).normalized()
                }
            };
            let value = (fb - fa).dot(dir).abs();
            linear(page, p.sheet(fa), p.sheet(fb), dir, d.offset, &shown(&d.text, format_length(value)));
            Ok(None)
        }
        DimKind::Diameter { near } | DimKind::Radius { near } => {
            let at = p.frame.flat(spot(near, locate)?);
            let circle = p
                .lines
                .circles
                .iter()
                .min_by(|x, y| (x.center.dist(at) - x.radius).abs().min(x.center.dist(at)).total_cmp(&(y.center.dist(at) - y.radius).abs().min(y.center.dist(at))))
                .ok_or_else(|| Error::InvalidInput("this view sees no circle face on".into()))?;
            let diameter = matches!(d.kind, DimKind::Diameter { .. });
            let value = if diameter { format!("\u{D8}{}", format_length(2.0 * circle.radius)) } else { format!("R{}", format_length(circle.radius)) };
            leader(page, p.sheet(circle.center), circle.radius * p.scale, d.offset.max(4.0), diameter, &shown(&d.text, value), 45.0);
            Ok(Some((circle.center, circle.radius)))
        }
    }
}

/// A linear dimension between two points on the sheet, measured along `dir`. Its line is `offset` past the farther point
/// to the left of `dir`, or with a negative offset, past the nearer one to the right.
fn linear(page: &mut Page, a: Vec2, b: Vec2, dir: Vec2, offset: f64, text: &str) {
    let normal = v2(-dir.y, dir.x);
    let line_at = if offset >= 0.0 { a.dot(normal).max(b.dot(normal)) + offset } else { a.dot(normal).min(b.dot(normal)) + offset };
    let foot = |p: Vec2| p + normal * (line_at - p.dot(normal));
    let (fa, fb) = (foot(a), foot(b));
    // Extension lines, from a millimetre off what's measured to two past the dimension line.
    for (p, f) in [(a, fa), (b, fb)] {
        let reach = f - p;
        if reach.norm() > 1.0 {
            let d = reach.normalized();
            page.line(Pen::Thin, vec![p + d, f + d * 2.0]);
        }
    }
    page.line(Pen::Thin, vec![fa, fb]);
    if fa.dist(fb) > 1e-9 {
        let d = (fb - fa).normalized();
        page.arrow(fa, -d);
        page.arrow(fb, d);
    }
    let middle = (fa + fb) * 0.5;
    let mut angle = dir.y.atan2(dir.x).to_degrees();
    // Read from the bottom or the right, as ISO 129 has it.
    if angle > 90.0 + 1e-9 {
        angle -= 180.0;
    } else if angle <= -90.0 + 1e-9 {
        angle += 180.0;
    }
    let up = v2(-(angle.to_radians().sin()), angle.to_radians().cos());
    page.text(middle + up * 1.2, 3.5, text, Align::Center, angle);
}

/// A diameter or radius on a circle seen face on: a leader through the center or from it, at `degrees` from the right,
/// with an arrow on the rim and the text out past it, on a short line going the leader's way.
fn leader(page: &mut Page, center: Vec2, radius: f64, out: f64, diameter: bool, text: &str, degrees: f64) {
    let d = v2(degrees.to_radians().cos(), degrees.to_radians().sin());
    let rim = center + d * radius;
    let end = rim + d * out;
    let start = if diameter { center - d * radius } else { center };
    let width = out::text_width(text, 3.5) + 2.0;
    let leftward = d.x < 0.0;
    let shelf = end + v2(if leftward { -width } else { width }, 0.0);
    page.line(Pen::Thin, vec![start, end, shelf]);
    page.arrow(rim, d);
    if diameter {
        page.arrow(center - d * radius, -d);
    }
    if leftward {
        page.text(end + v2(-1.0, 1.2), 3.5, text, Align::Right, 0.0);
    } else {
        page.text(end + v2(1.0, 1.2), 3.5, text, Align::Left, 0.0);
    }
}

/// A view's overall width below it and height to its right, and each circle's diameter once.
fn auto_dimensions(page: &mut Page, p: &Placed, mut done: Vec<(Vec2, f64)>) {
    let (lo, hi) = (p.lines.lo, p.lines.hi);
    let (a, b) = (v2(lo.x, lo.y), v2(hi.x, lo.y));
    let width = hi.x - lo.x;
    let height = hi.y - lo.y;
    if width * p.scale > 2.0 {
        linear(page, p.sheet(a), p.sheet(b), v2(1.0, 0.0), -8.0, &format_length(width));
    }
    if height * p.scale > 2.0 {
        // To the right, measured along y, with the line off to the right: left of a downward direction. Out past where
        // a section's cutting line ends.
        linear(page, p.sheet(v2(hi.x, hi.y)), p.sheet(v2(hi.x, lo.y)), v2(0.0, -1.0), 12.0, &format_length(height));
    }
    // Circles round one center each get a leader going a different way: up right, up left, down left, down right.
    let ways = [45.0, 135.0, 225.0, 315.0];
    let tol = 1e-9 * (hi - lo).norm().max(1.0);
    for c in &p.lines.circles {
        if c.sweep < 2.0 * PI - 1e-6 || done.iter().any(|(o, r)| o.dist(c.center) < tol && (r - c.radius).abs() < tol) {
            continue;
        }
        let around = done.iter().filter(|(o, _)| o.dist(c.center) < tol).count();
        done.push((c.center, c.radius));
        if c.radius * p.scale >= 1.5 {
            leader(page, p.sheet(c.center), c.radius * p.scale, 5.0, true, &format!("\u{D8}{}", format_length(2.0 * c.radius)), ways[around % 4]);
        }
    }
}

/// The border, and the title block in the bottom right corner with the projection's symbol.
fn border_and_title(page: &mut Page, drawing: &Drawing, scale: f64) {
    let (w, h) = (page.width, page.height);
    page.line(Pen::Frame, vec![v2(LEFT, EDGE), v2(w - EDGE, EDGE), v2(w - EDGE, h - EDGE), v2(LEFT, h - EDGE), v2(LEFT, EDGE)]);
    let (x0, y0) = (w - EDGE - BLOCK_W, EDGE);
    let (x1, y1) = (w - EDGE, EDGE + BLOCK_H);
    page.line(Pen::Frame, vec![v2(x0, y0), v2(x1, y0), v2(x1, y1), v2(x0, y1), v2(x0, y0)]);
    // Rows of 10, 10, 10 and 10; the top row is the title.
    for k in 1..4 {
        let y = y0 + 10.0 * k as f64;
        page.line(Pen::Thin, vec![v2(x0, y), v2(x1, y)]);
    }
    let col = |x: f64, from: f64, to: f64, page: &mut Page| page.line(Pen::Thin, vec![v2(x0 + x, y0 + from), v2(x0 + x, y0 + to)]);
    for x in [45.0, 90.0, 135.0] {
        col(x, 0.0, 30.0, page);
    }
    let t = &drawing.title;
    let field = |page: &mut Page, cx: f64, row: usize, label: &str, value: &str| {
        let y = y0 + 10.0 * row as f64;
        page.text(v2(x0 + cx + 1.5, y + 7.0), 1.8, label, Align::Left, 0.0);
        page.text(v2(x0 + cx + 1.5, y + 1.8), 3.0, value, Align::Left, 0.0);
    };
    let sheet = drawing.sheet.to_ascii_uppercase();
    field(page, 0.0, 0, "DRAWN BY", &t.author);
    field(page, 45.0, 0, "DATE", &t.date);
    field(page, 90.0, 0, "SHEET", &sheet);
    field(page, 135.0, 0, "REVISION", &t.revision);
    field(page, 0.0, 1, "PART", &t.part);
    field(page, 45.0, 1, "MATERIAL", &t.material);
    field(page, 90.0, 1, "SCALE", &format_scale(scale));
    field(page, 135.0, 1, "UNITS", "mm");
    field(page, 0.0, 2, "DRAWING NUMBER", &t.number);
    field(page, 90.0, 2, "COMPANY", &t.company);
    page.text(v2(x0 + 3.0, y0 + 32.5), 5.0, if t.title.is_empty() { &t.part } else { &t.title }, Align::Left, 0.0);
    // The projection symbol, right of the title: a cone's side seen as a trapezoid, its narrow end left, and its end seen as
    // two circles, to the right of it in first angle and to the left in third.
    let first = drawing.projection == Projection::FirstAngle;
    let (sx, sy) = (x1 - 34.0, y0 + 35.0);
    let (cone, circles) = if first { (sx, sx + 22.0) } else { (sx + 12.0, sx + 3.0) };
    page.line(Pen::Thin, vec![v2(cone, sy - 1.5), v2(cone + 8.0, sy - 3.0), v2(cone + 8.0, sy + 3.0), v2(cone, sy + 1.5), v2(cone, sy - 1.5)]);
    for r in [1.5, 3.0] {
        let ring: Vec<Vec2> = (0..=32).map(|k| v2(circles, sy) + v2((k as f64 / 32.0 * TAU).cos(), (k as f64 / 32.0 * TAU).sin()) * r).collect();
        page.line(Pen::Thin, ring);
    }
}

// ----- JSON -----

fn bad(what: &str) -> Error {
    Error::InvalidInput(format!("a drawing's {what} is missing or wrong"))
}

fn num(v: f64) -> Value {
    Value::Num(v)
}

fn vec3_value(v: Vec3) -> Value {
    Value::Arr(vec![num(v.x), num(v.y), num(v.z)])
}

fn read_vec3(v: &Value) -> Result<Vec3> {
    match v.as_array() {
        Some([x, y, z]) => match (x.as_f64(), y.as_f64(), z.as_f64()) {
            (Some(x), Some(y), Some(z)) if x.is_finite() && y.is_finite() && z.is_finite() => Ok(v3(x, y, z)),
            _ => Err(Error::InvalidInput("a point must be three numbers".into())),
        },
        _ => Err(Error::InvalidInput("a point must be three numbers".into())),
    }
}

fn spot_value(s: &Spot) -> Value {
    match s {
        Spot::At(p) => vec3_value(*p),
        Spot::Ref(v) => v.clone(),
    }
}

/// A point as three numbers, or anything else to be looked up.
pub fn read_spot(v: &Value) -> Result<Spot> {
    match v.as_array() {
        Some(list) if list.len() == 3 && list.iter().all(|x| x.as_f64().is_some()) => Ok(Spot::At(read_vec3(v)?)),
        _ => Ok(Spot::Ref(v.clone())),
    }
}

pub fn read_view_kind(v: &Value) -> Result<ViewKind> {
    let name = v.get("type").and_then(Value::as_str).ok_or_else(|| bad("view type"))?;
    Ok(match name {
        "front" => ViewKind::Front,
        "back" => ViewKind::Back,
        "top" => ViewKind::Top,
        "bottom" => ViewKind::Bottom,
        "left" => ViewKind::Left,
        "right" => ViewKind::Right,
        "iso" => ViewKind::Iso,
        "section" => {
            let point = v.get("point").map_or(Ok(Vec3::ZERO), read_vec3)?;
            let normal = read_vec3(v.get("normal").ok_or_else(|| Error::InvalidInput("a section needs a \"normal\"".into()))?)?;
            if !(normal.norm() > 1e-12) {
                return Err(Error::InvalidInput("a section's normal can't be nothing".into()));
            }
            ViewKind::Section { point, normal: normal.normalized(), letter: v.get("letter").and_then(Value::as_str).unwrap_or("A").to_string() }
        }
        other => return Err(Error::InvalidInput(format!("unknown view \"{other}\"; expected front, back, top, bottom, left, right, iso or section"))),
    })
}

fn view_kind_items(kind: &ViewKind) -> Vec<(&'static str, Value)> {
    let mut items = vec![("type", Value::str(kind.name()))];
    if let ViewKind::Section { point, normal, letter } = kind {
        items.push(("point", vec3_value(*point)));
        items.push(("normal", vec3_value(*normal)));
        items.push(("letter", Value::str(letter)));
    }
    items
}

pub fn read_dim_kind(v: &Value) -> Result<DimKind> {
    let name = v.get("type").and_then(Value::as_str).unwrap_or("distance");
    let need = |key: &str| v.get(key).ok_or_else(|| Error::InvalidInput(format!("a {name} dimension needs \"{key}\"")));
    Ok(match name {
        "distance" => {
            let along = match v.get("along").and_then(Value::as_str).unwrap_or("aligned") {
                "horizontal" => Along::Horizontal,
                "vertical" => Along::Vertical,
                "aligned" => Along::Aligned,
                other => return Err(Error::InvalidInput(format!("unknown \"along\" \"{other}\"; expected horizontal, vertical or aligned"))),
            };
            DimKind::Distance { a: read_spot(need("a")?)?, b: read_spot(need("b")?)?, along }
        }
        "diameter" => DimKind::Diameter { near: read_spot(need("near")?)? },
        "radius" => DimKind::Radius { near: read_spot(need("near")?)? },
        other => return Err(Error::InvalidInput(format!("unknown dimension \"{other}\"; expected distance, diameter or radius"))),
    })
}

impl Drawing {
    pub fn to_json(&self) -> Value {
        let views = self.views.iter().map(|v| {
            let mut items = vec![("id", num(v.id as f64))];
            items.extend(view_kind_items(&v.kind));
            if let Some([x, y]) = v.at {
                items.push(("at", Value::Arr(vec![num(x), num(y)])));
            }
            if let Some(s) = v.scale {
                items.push(("scale", num(s)));
            }
            if let Some(hidden) = v.hidden {
                items.push(("hidden", Value::Bool(hidden)));
            }
            Value::obj(items)
        });
        let dims = self.dimensions.iter().map(|d| {
            let mut items = vec![("id", num(d.id as f64)), ("view", num(d.view as f64))];
            match &d.kind {
                DimKind::Distance { a, b, along } => {
                    let along = match along {
                        Along::Horizontal => "horizontal",
                        Along::Vertical => "vertical",
                        Along::Aligned => "aligned",
                    };
                    items.extend([("type", Value::str("distance")), ("a", spot_value(a)), ("b", spot_value(b)), ("along", Value::str(along))]);
                }
                DimKind::Diameter { near } => items.extend([("type", Value::str("diameter")), ("near", spot_value(near))]),
                DimKind::Radius { near } => items.extend([("type", Value::str("radius")), ("near", spot_value(near))]),
            }
            items.push(("offset", num(d.offset)));
            if let Some(t) = &d.text {
                items.push(("text", Value::str(t)));
            }
            Value::obj(items)
        });
        let t = &self.title;
        let title = Value::obj(
            [("title", &t.title), ("number", &t.number), ("part", &t.part), ("material", &t.material), ("author", &t.author), ("date", &t.date), ("company", &t.company), ("revision", &t.revision)]
                .into_iter()
                .filter(|(_, v)| !v.is_empty())
                .map(|(k, v)| (k, Value::str(v)))
                .collect(),
        );
        let mut items = vec![
            ("sheet", Value::str(&self.sheet)),
            ("projection", Value::str(if self.projection == Projection::FirstAngle { "first_angle" } else { "third_angle" })),
            ("views", Value::Arr(views.collect())),
            ("dimensions", Value::Arr(dims.collect())),
            ("auto_dimensions", Value::Bool(self.auto_dimensions)),
            ("title", title),
            ("next_id", num(self.next_id as f64)),
        ];
        if let Some(s) = self.scale {
            items.push(("scale", num(s)));
        }
        Value::obj(items)
    }

    pub fn from_json(v: &Value) -> Result<Drawing> {
        let mut d = Drawing { sheet: v.get("sheet").and_then(Value::as_str).unwrap_or("A3").to_string(), ..Drawing::default() };
        d.size()?;
        d.projection = match v.get("projection").and_then(Value::as_str).unwrap_or("first_angle") {
            "first_angle" | "first" => Projection::FirstAngle,
            "third_angle" | "third" => Projection::ThirdAngle,
            other => return Err(Error::InvalidInput(format!("unknown projection \"{other}\"; expected first_angle or third_angle"))),
        };
        d.scale = v.get("scale").and_then(Value::as_f64);
        d.auto_dimensions = v.get("auto_dimensions").and_then(Value::as_bool).unwrap_or(true);
        let id_of = |x: &Value, what: &str| x.get("id").and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 1.0 && *n < 4e9).map(|n| n as u32).ok_or_else(|| bad(what));
        for x in v.get("views").and_then(Value::as_array).unwrap_or(&[]) {
            let at = match x.get("at").and_then(Value::as_array) {
                Some([a, b]) => Some([a.as_f64().ok_or_else(|| bad("view place"))?, b.as_f64().ok_or_else(|| bad("view place"))?]),
                Some(_) => return Err(bad("view place")),
                None => None,
            };
            d.views.push(View { id: id_of(x, "view id")?, kind: read_view_kind(x)?, at, scale: x.get("scale").and_then(Value::as_f64), hidden: x.get("hidden").and_then(Value::as_bool) });
        }
        for x in v.get("dimensions").and_then(Value::as_array).unwrap_or(&[]) {
            let view = x.get("view").and_then(Value::as_f64).filter(|n| n.fract() == 0.0 && *n >= 1.0).ok_or_else(|| bad("dimension's view"))? as u32;
            d.dimensions.push(Dimension {
                id: id_of(x, "dimension id")?,
                view,
                kind: read_dim_kind(x)?,
                offset: x.get("offset").and_then(Value::as_f64).unwrap_or(10.0),
                text: x.get("text").and_then(Value::as_str).map(str::to_string),
            });
        }
        if let Some(t) = v.get("title") {
            let s = |k: &str| t.get(k).and_then(Value::as_str).unwrap_or("").to_string();
            d.title = Title { title: s("title"), number: s("number"), part: s("part"), material: s("material"), author: s("author"), date: s("date"), company: s("company"), revision: s("revision") };
        }
        let top = d.views.iter().map(|v| v.id).chain(d.dimensions.iter().map(|x| x.id)).max().unwrap_or(0);
        let saved = v.get("next_id").and_then(Value::as_f64).filter(|n| *n >= 0.0 && *n < 4e9).unwrap_or(0.0) as u32;
        d.next_id = (top + 1).max(saved).max(1);
        Ok(d)
    }
}
