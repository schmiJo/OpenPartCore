//! Offscreen rendering: a body to an image, with no GPU and no display.
//!
//! This is how an agent working through the MCP server sees what it built, so an image has to be
//! readable as a drawing: edges drawn crisply, silhouettes outlined, what's hidden hidden, and faces
//! labelled with the same names the tools use. It works the same on every machine, to the byte.

pub mod png;

mod font;

use coreshape::{tessellate, Aabb, Body, Mesh, Result, TessParams, Vec3};

/// Where to look from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum View {
    Iso,
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    /// Degrees around Z from the front, and up from the XY plane.
    Orbit { azimuth: f64, elevation: f64 },
}

impl View {
    pub fn parse(text: &str) -> Option<View> {
        Some(match text {
            "iso" => View::Iso,
            "front" => View::Front,
            "back" => View::Back,
            "left" => View::Left,
            "right" => View::Right,
            "top" => View::Top,
            "bottom" => View::Bottom,
            _ => return None,
        })
    }

    fn angles(self) -> (f64, f64) {
        match self {
            View::Iso => (-35.0, 28.0),
            View::Front => (0.0, 0.0),
            View::Back => (180.0, 0.0),
            View::Left => (-90.0, 0.0),
            View::Right => (90.0, 0.0),
            View::Top => (0.0, 90.0),
            View::Bottom => (0.0, -90.0),
            View::Orbit { azimuth, elevation } => (azimuth, elevation),
        }
    }
}

/// How to colour faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Colors {
    /// One colour, which reads best as a shape.
    Plain,
    /// A different tint for each named face, which shows how the body is divided.
    ByFace,
    /// A different tint for each group of `face_groups`, which tells a document's bodies apart.
    ByGroup,
}

/// A label to draw at a place on the picture.
#[derive(Clone, Debug, PartialEq)]
pub struct Label {
    pub text: String,
    /// Where the label sits, in image pixels.
    pub at: [i32; 2],
}

#[derive(Clone, Debug)]
pub struct Options {
    pub width: u32,
    pub height: u32,
    pub view: View,
    pub colors: Colors,
    /// Faces to pick out, by index in the body.
    pub highlight_faces: Vec<usize>,
    /// Edges to pick out, by index in the body.
    pub highlight_edges: Vec<usize>,
    /// A name for each face, to label those that can be seen. Empty for no labels.
    pub face_names: Vec<String>,
    /// The group of each face, for `Colors::ByGroup`.
    pub face_groups: Vec<u32>,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            width: 800,
            height: 600,
            view: View::Iso,
            colors: Colors::Plain,
            highlight_faces: Vec::new(),
            highlight_edges: Vec::new(),
            face_names: Vec::new(),
            face_groups: Vec::new(),
        }
    }
}

/// A rendered picture.
#[derive(Clone, Debug)]
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
    /// Faces that can be seen, by index, with how many pixels of each show.
    pub visible_faces: Vec<(usize, usize)>,
    /// Labels drawn on the picture. Faces too small to label are visible but not listed here.
    pub labels: Vec<Label>,
    /// How many millimetres one pixel covers.
    pub mm_per_pixel: f64,
}

impl Image {
    pub fn to_png(&self) -> Vec<u8> {
        png::encode_rgb(self.width, self.height, &self.rgb)
    }
}

const SAMPLES: usize = 2;
const NO_FACE: u32 = u32::MAX;
const BACKGROUND: [f64; 3] = [0.965, 0.965, 0.95];
const BODY: [f64; 3] = [0.62, 0.68, 0.74];
const HIGHLIGHT: [f64; 3] = [0.93, 0.45, 0.12];
const EDGE: [f64; 3] = [0.10, 0.12, 0.15];

struct Camera {
    right: Vec3,
    up: Vec3,
    toward: Vec3,
    center: Vec3,
    scale: f64,
    offset: [f64; 2],
}

impl Camera {
    fn new(bounds: &Aabb, view: View, width: usize, height: usize) -> Camera {
        let (az, el) = view.angles();
        let (az, el) = (az.to_radians(), el.to_radians());
        // Looking from here toward the middle of the part.
        let toward = Vec3 { x: el.cos() * az.sin(), y: -el.cos() * az.cos(), z: el.sin() };
        let z = Vec3 { x: 0.0, y: 0.0, z: 1.0 };
        let right = if toward.cross(z).norm() < 1e-9 {
            Vec3 { x: 1.0, y: 0.0, z: 0.0 }
        } else {
            z.cross(toward).normalized()
        };
        let up = toward.cross(right).normalized();
        let center = (bounds.min + bounds.max) * 0.5;
        // Fit the box's corners, with a margin.
        let (mut reach_x, mut reach_y) = (1e-9f64, 1e-9f64);
        for i in 0..8 {
            let p = Vec3 {
                x: if i & 1 == 0 { bounds.min.x } else { bounds.max.x },
                y: if i & 2 == 0 { bounds.min.y } else { bounds.max.y },
                z: if i & 4 == 0 { bounds.min.z } else { bounds.max.z },
            } - center;
            reach_x = reach_x.max(p.dot(right).abs());
            reach_y = reach_y.max(p.dot(up).abs());
        }
        let margin = 0.88;
        let scale = (width as f64 * margin / (2.0 * reach_x)).min(height as f64 * margin / (2.0 * reach_y));
        Camera { right, up, toward, center, scale, offset: [width as f64 / 2.0, height as f64 / 2.0] }
    }

    /// Screen x, screen y, and height toward the viewer.
    fn project(&self, p: Vec3) -> [f64; 3] {
        let d = p - self.center;
        [self.offset[0] + d.dot(self.right) * self.scale, self.offset[1] - d.dot(self.up) * self.scale, d.dot(self.toward)]
    }
}

fn tint(index: usize) -> [f64; 3] {
    // Evenly spread hues by the golden angle, kept pale so that shading and edges still read.
    let hue = (index as f64 * 0.618_033_988_75).fract() * 6.0;
    let x = 1.0 - (hue % 2.0 - 1.0).abs();
    let (r, g, b) = match hue as u32 {
        0 => (1.0, x, 0.0),
        1 => (x, 1.0, 0.0),
        2 => (0.0, 1.0, x),
        3 => (0.0, x, 1.0),
        4 => (x, 0.0, 1.0),
        _ => (1.0, 0.0, x),
    };
    [0.55 + 0.4 * r, 0.55 + 0.4 * g, 0.55 + 0.4 * b]
}

struct Buffers {
    w: usize,
    h: usize,
    depth: Vec<f64>,
    face: Vec<u32>,
    color: Vec<[f64; 3]>,
}

fn fill_triangles(b: &mut Buffers, mesh: &Mesh, cam: &Camera, base: &[[f64; 3]]) {
    let light = (cam.toward * 0.8 + cam.up * 0.5 - cam.right * 0.3).normalized();
    let screen: Vec<[f64; 3]> = mesh.positions.iter().map(|&p| cam.project(p)).collect();
    for (t, &f) in mesh.triangles.iter().zip(&mesh.face_of_triangle) {
        let [i0, i1, i2] = [t[0] as usize, t[1] as usize, t[2] as usize];
        let (p0, p1, p2) = (screen[i0], screen[i1], screen[i2]);
        let area = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
        if area.abs() < 1e-12 {
            continue;
        }
        let min_x = p0[0].min(p1[0]).min(p2[0]).floor().max(0.0) as usize;
        let max_x = (p0[0].max(p1[0]).max(p2[0]).ceil() as i64).min(b.w as i64 - 1);
        let min_y = p0[1].min(p1[1]).min(p2[1]).floor().max(0.0) as usize;
        let max_y = (p0[1].max(p1[1]).max(p2[1]).ceil() as i64).min(b.h as i64 - 1);
        if max_x < 0 || max_y < 0 {
            continue;
        }
        let (n0, n1, n2) = (mesh.normals[i0], mesh.normals[i1], mesh.normals[i2]);
        for y in min_y..=max_y as usize {
            for x in min_x..=max_x as usize {
                let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
                let w0 = ((p1[0] - px) * (p2[1] - py) - (p1[1] - py) * (p2[0] - px)) / area;
                let w1 = ((p2[0] - px) * (p0[1] - py) - (p2[1] - py) * (p0[0] - px)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * p0[2] + w1 * p1[2] + w2 * p2[2];
                let at = y * b.w + x;
                if z > b.depth[at] {
                    b.depth[at] = z;
                    b.face[at] = f;
                    let n = (n0 * w0 + n1 * w1 + n2 * w2).normalized();
                    // Faces seen from behind can't happen on a closed solid, but a sliver can round to it.
                    let facing = n.dot(cam.toward).abs();
                    let diffuse = n.dot(light).max(0.0);
                    let shade = 0.34 + 0.42 * diffuse + 0.24 * facing;
                    let c = base[f as usize];
                    b.color[at] = [c[0] * shade, c[1] * shade, c[2] * shade];
                }
            }
        }
    }
}

fn plot(b: &mut Buffers, x: f64, y: f64, z: f64, color: [f64; 3], bias: f64, wide: i32) {
    for dy in -wide..=wide {
        for dx in -wide..=wide {
            let (ix, iy) = (x as i64 + dx as i64, y as i64 + dy as i64);
            if ix < 0 || iy < 0 || ix >= b.w as i64 || iy >= b.h as i64 {
                continue;
            }
            let at = iy as usize * b.w + ix as usize;
            if z + bias >= b.depth[at] {
                b.color[at] = color;
            }
        }
    }
}

fn draw_polyline(b: &mut Buffers, line: &[Vec3], cam: &Camera, color: [f64; 3], bias: f64, wide: i32) {
    for w in line.windows(2) {
        let (p, q) = (cam.project(w[0]), cam.project(w[1]));
        let steps = ((q[0] - p[0]).abs().max((q[1] - p[1]).abs()).ceil() as usize).max(1);
        for k in 0..=steps {
            let f = k as f64 / steps as f64;
            plot(b, p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f, p[2] + (q[2] - p[2]) * f, color, bias, wide);
        }
    }
}

/// Outlines where depth jumps between neighbours: the silhouettes of curved faces, which no B-rep
/// edge marks.
fn draw_silhouettes(b: &mut Buffers, jump: f64) {
    let (w, h) = (b.w, b.h);
    let mut marks = Vec::new();
    for y in 0..h {
        for x in 0..w {
            let at = y * w + x;
            let here = b.depth[at];
            if b.face[at] == NO_FACE {
                continue;
            }
            let mut outline = false;
            for (nx, ny) in [(x.wrapping_sub(1), y), (x + 1, y), (x, y.wrapping_sub(1)), (x, y + 1)] {
                if nx >= w || ny >= h {
                    continue;
                }
                let other = ny * w + nx;
                if b.face[other] == NO_FACE || here - b.depth[other] > jump {
                    outline = true;
                }
            }
            if outline {
                marks.push(at);
            }
        }
    }
    for at in marks {
        b.color[at] = EDGE;
    }
}

fn downsample(b: &Buffers, width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);
    for y in 0..height {
        for x in 0..width {
            let mut sum = [0.0; 3];
            for sy in 0..SAMPLES {
                for sx in 0..SAMPLES {
                    let c = b.color[(y * SAMPLES + sy) * b.w + x * SAMPLES + sx];
                    sum[0] += c[0];
                    sum[1] += c[1];
                    sum[2] += c[2];
                }
            }
            let n = (SAMPLES * SAMPLES) as f64;
            for s in sum {
                rgb.push(((s / n).clamp(0.0, 1.0) * 255.0 + 0.5) as u8);
            }
        }
    }
    rgb
}

fn put(rgb: &mut [u8], width: usize, height: usize, x: i32, y: i32, c: [u8; 3]) {
    if x >= 0 && y >= 0 && (x as usize) < width && (y as usize) < height {
        let at = (y as usize * width + x as usize) * 3;
        rgb[at..at + 3].copy_from_slice(&c);
    }
}

fn text_size(text: &str, scale: i32) -> (i32, i32) {
    (text.chars().count() as i32 * font::ADVANCE * scale, font::HEIGHT * scale)
}

fn draw_text(rgb: &mut [u8], width: usize, height: usize, x: i32, y: i32, text: &str, scale: i32, color: [u8; 3]) {
    for (i, c) in text.chars().enumerate() {
        let Some(rows) = font::glyph(c) else { continue };
        let left = x + i as i32 * font::ADVANCE * scale;
        for (row, bits) in rows.iter().enumerate() {
            for col in 0..5 {
                if bits >> (4 - col) & 1 == 1 {
                    for sy in 0..scale {
                        for sx in 0..scale {
                            put(rgb, width, height, left + col * scale + sx, y + row as i32 * scale + sy, color);
                        }
                    }
                }
            }
        }
    }
}

/// Places a label on each visible named face: at the visible pixel farthest from the face's edge,
/// so it sits inside the face, skipping any that wouldn't fit or would cover another.
fn place_labels(rgb: &mut [u8], width: usize, height: usize, ids: &[u32], names: &[String], visible: &[(usize, usize)]) -> Vec<Label> {
    let mut placed: Vec<[i32; 4]> = Vec::new();
    let mut out = Vec::new();
    // Largest first, since they have the most room to give.
    let mut order: Vec<(usize, usize)> = visible.to_vec();
    order.sort_by_key(|v| std::cmp::Reverse(v.1));
    // Distance from each pixel to the nearest pixel of another face, in two sweeps.
    let mut clearance = vec![0u16; width * height];
    for y in 0..height {
        for x in 0..width {
            let at = y * width + x;
            if ids[at] == NO_FACE {
                continue;
            }
            let same = |o: Option<usize>| o.is_some_and(|o| ids[o] == ids[at]);
            let (l, u) = ((x > 0).then(|| at - 1), (y > 0).then(|| at - width));
            let a = if same(l) { clearance[l.unwrap()] } else { 0 };
            let b = if same(u) { clearance[u.unwrap()] } else { 0 };
            clearance[at] = a.min(b) + 1;
        }
    }
    for y in (0..height).rev() {
        for x in (0..width).rev() {
            let at = y * width + x;
            if ids[at] == NO_FACE {
                continue;
            }
            let same = |o: Option<usize>| o.is_some_and(|o| ids[o] == ids[at]);
            let (r, d) = ((x + 1 < width).then(|| at + 1), (y + 1 < height).then(|| at + width));
            let a = if same(r) { clearance[r.unwrap()] + 1 } else { 1 };
            let b = if same(d) { clearance[d.unwrap()] + 1 } else { 1 };
            clearance[at] = clearance[at].min(a).min(b);
        }
    }
    let mut best: Vec<(u16, i32, i32)> = vec![(0, 0, 0); names.len()];
    for y in 0..height {
        for x in 0..width {
            let at = y * width + x;
            let f = ids[at];
            if f != NO_FACE && (f as usize) < best.len() && clearance[at] > best[f as usize].0 {
                best[f as usize] = (clearance[at], x as i32, y as i32);
            }
        }
    }
    for &(f, _) in &order {
        let Some(name) = names.get(f) else { continue };
        let (room, cx, cy) = best[f];
        let (tw, th) = text_size(name, 1);
        // The label needs some of the face around it, or it can't be told what it names.
        if (room as i32) < th / 2 + 2 {
            continue;
        }
        let (x, y) = ((cx - tw / 2).clamp(1, width as i32 - tw - 1), (cy - th / 2).clamp(1, height as i32 - th - 1));
        let rect = [x - 2, y - 2, x + tw + 1, y + th + 1];
        if placed.iter().any(|r| rect[0] <= r[2] && r[0] <= rect[2] && rect[1] <= r[3] && r[1] <= rect[3]) {
            continue;
        }
        for py in rect[1]..=rect[3] {
            for px in rect[0]..=rect[2] {
                put(rgb, width, height, px, py, [255, 255, 250]);
            }
        }
        draw_text(rgb, width, height, x, y, name, 1, [20, 24, 30]);
        placed.push(rect);
        out.push(Label { text: name.clone(), at: [cx, cy] });
    }
    out
}

fn draw_axes(rgb: &mut [u8], width: usize, height: usize, cam: &Camera) {
    let origin = [34.0, height as f64 - 34.0];
    let axes = [
        (Vec3 { x: 1.0, y: 0.0, z: 0.0 }, "X", [200u8, 50, 40]),
        (Vec3 { x: 0.0, y: 1.0, z: 0.0 }, "Y", [40, 150, 60]),
        (Vec3 { x: 0.0, y: 0.0, z: 1.0 }, "Z", [40, 80, 200]),
    ];
    for (axis, name, color) in axes {
        let (dx, dy) = (axis.dot(cam.right) * 22.0, -axis.dot(cam.up) * 22.0);
        for k in 0..=22 {
            let f = k as f64 / 22.0;
            put(rgb, width, height, (origin[0] + dx * f) as i32, (origin[1] + dy * f) as i32, color);
        }
        // An axis pointing at the viewer has no line to label.
        if dx.abs() + dy.abs() > 4.0 {
            draw_text(rgb, width, height, (origin[0] + dx * 1.25) as i32 - 2, (origin[1] + dy * 1.25) as i32 - 3, name, 1, color);
        }
    }
}

/// Renders a body.
pub fn render(body: &Body, options: &Options) -> Result<Image> {
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal()))?;
    // The mesh's bounds, since the B-rep's box is a safe overestimate around curves.
    let bounds = mesh.bounds();
    Ok(render_mesh(body, &mesh, &bounds, options))
}

fn render_mesh(body: &Body, mesh: &Mesh, bounds: &Aabb, options: &Options) -> Image {
    let (width, height) = (options.width.clamp(16, 4096) as usize, options.height.clamp(16, 4096) as usize);
    let (w, h) = (width * SAMPLES, height * SAMPLES);
    let cam = Camera::new(bounds, options.view, w, h);
    let mut buffers = Buffers { w, h, depth: vec![f64::NEG_INFINITY; w * h], face: vec![NO_FACE; w * h], color: vec![BACKGROUND; w * h] };
    let base: Vec<[f64; 3]> = (0..body.faces.len())
        .map(|f| {
            if options.highlight_faces.contains(&f) {
                HIGHLIGHT
            } else {
                match options.colors {
                    Colors::Plain => BODY,
                    Colors::ByFace => tint(f),
                    Colors::ByGroup => tint(options.face_groups.get(f).map_or(0, |g| *g as usize)),
                }
            }
        })
        .collect();
    fill_triangles(&mut buffers, mesh, &cam, &base);
    let size = bounds.diagonal().max(1e-9);
    draw_silhouettes(&mut buffers, size * 0.02);
    // Edges win over the face they lie on, by a small bias toward the viewer.
    let bias = size * 2e-3;
    for (e, line) in mesh.edge_lines.iter().enumerate() {
        if !options.highlight_edges.contains(&e) {
            draw_polyline(&mut buffers, line, &cam, EDGE, bias, 0);
        }
    }
    for &e in &options.highlight_edges {
        if let Some(line) = mesh.edge_lines.get(e) {
            draw_polyline(&mut buffers, line, &cam, HIGHLIGHT, bias, 1);
        }
    }
    let mut rgb = downsample(&buffers, width, height);
    // What shows, at the output's resolution.
    let mut ids = vec![NO_FACE; width * height];
    let mut counts = vec![0usize; body.faces.len()];
    for y in 0..height {
        for x in 0..width {
            let f = buffers.face[(y * SAMPLES + SAMPLES / 2) * w + x * SAMPLES + SAMPLES / 2];
            ids[y * width + x] = f;
            if f != NO_FACE {
                counts[f as usize] += 1;
            }
        }
    }
    let visible_faces: Vec<(usize, usize)> = counts.iter().enumerate().filter(|(_, &n)| n > 0).map(|(f, &n)| (f, n)).collect();
    let labels = if options.face_names.is_empty() {
        Vec::new()
    } else {
        place_labels(&mut rgb, width, height, &ids, &options.face_names, &visible_faces)
    };
    draw_axes(&mut rgb, width, height, &cam);
    Image { width: width as u32, height: height as u32, rgb, visible_faces, labels, mm_per_pixel: SAMPLES as f64 / cam.scale }
}

/// Four views in one picture: front, right, top and iso, as on a drawing.
pub fn render_four(body: &Body, options: &Options) -> Result<Image> {
    let mesh = tessellate(body, &TessParams::for_size(body.aabb().diagonal()))?;
    let bounds = mesh.bounds();
    let (cell_w, cell_h) = ((options.width / 2).max(16), (options.height / 2).max(16));
    let (w, h) = (cell_w as usize * 2, cell_h as usize * 2);
    let mut rgb = vec![255u8; w * h * 3];
    let mut labels = Vec::new();
    let mut counts = vec![0usize; body.faces.len()];
    for (i, (view, name)) in [(View::Top, "top"), (View::Iso, "iso"), (View::Front, "front"), (View::Right, "right")].iter().enumerate() {
        let cell = render_mesh(body, &mesh, &bounds, &Options { width: cell_w, height: cell_h, view: *view, ..options.clone() });
        let (ox, oy) = ((i % 2) * cell_w as usize, (i / 2) * cell_h as usize);
        for y in 0..cell_h as usize {
            let from = y * cell_w as usize * 3;
            let to = ((y + oy) * w + ox) * 3;
            rgb[to..to + cell_w as usize * 3].copy_from_slice(&cell.rgb[from..from + cell_w as usize * 3]);
        }
        draw_text(&mut rgb, w, h, ox as i32 + 6, oy as i32 + 5, name, 2, [90, 95, 105]);
        for (f, n) in cell.visible_faces {
            counts[f] += n;
        }
        for mut l in cell.labels {
            l.at = [l.at[0] + ox as i32, l.at[1] + oy as i32];
            labels.push(l);
        }
    }
    // Lines between the cells.
    for y in 0..h {
        put(&mut rgb, w, h, cell_w as i32, y as i32, [170, 170, 170]);
    }
    for x in 0..w {
        put(&mut rgb, w, h, x as i32, cell_h as i32, [170, 170, 170]);
    }
    let visible_faces = counts.iter().enumerate().filter(|(_, &n)| n > 0).map(|(f, &n)| (f, n)).collect();
    let mm_per_pixel = Camera::new(&bounds, View::Iso, cell_w as usize, cell_h as usize).scale.recip();
    Ok(Image { width: w as u32, height: h as u32, rgb, visible_faces, labels, mm_per_pixel })
}
