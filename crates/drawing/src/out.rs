//! A page written out: SVG, PDF and DXF, each by hand, and a PNG drawn here. Text is the reader's own sans serif in the
//! three vector formats, so it can be edited, and the built-in Noto Sans, filled, in the PNG.

use crate::{Align, Page, Pen, Text};
use coreshape::math::*;
use coreshape_sketch::font::{Font, Piece};

/// How wide a text is on the sheet, in millimetres, by the built-in font's spacing, which is near enough a sans serif's.
pub fn text_width(text: &str, height: f64) -> f64 {
    let font = Font::builtin();
    font.advance(text) * height / font.cap_height
}

/// Where a text's left end goes, for its alignment.
fn left_of(t: &Text) -> Vec2 {
    let w = text_width(&t.text, t.height);
    let shift = match t.align {
        Align::Left => 0.0,
        Align::Center => w / 2.0,
        Align::Right => w,
    };
    let a = t.angle.to_radians();
    t.at - v2(a.cos(), a.sin()) * shift
}

fn number(x: f64) -> String {
    let s = format!("{:.3}", x);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// The page as SVG, in millimetres, the right way up.
pub fn svg(page: &Page) -> String {
    let (w, h) = (page.width, page.height);
    let mut out = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}mm\" height=\"{}mm\" viewBox=\"0 0 {} {}\">\n<rect width=\"{}\" height=\"{}\" fill=\"white\"/>\n",
        number(w),
        number(h),
        number(w),
        number(h),
        number(w),
        number(h)
    );
    let y = |v: f64| h - v;
    out.push_str("<g fill=\"none\" stroke=\"black\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n");
    for (pen, points) in &page.lines {
        let path: Vec<String> = points.iter().map(|p| format!("{},{}", number(p.x), number(y(p.y)))).collect();
        let dash = if pen.dashes().is_empty() { String::new() } else { format!(" stroke-dasharray=\"{}\"", pen.dashes().iter().map(|d| number(*d)).collect::<Vec<_>>().join(" ")) };
        out.push_str(&format!("<polyline points=\"{}\" stroke-width=\"{}\"{dash}/>\n", path.join(" "), number(pen.width())));
    }
    out.push_str("</g>\n<g fill=\"black\" stroke=\"none\">\n");
    for f in &page.fills {
        let path: Vec<String> = f.iter().map(|p| format!("{},{}", number(p.x), number(y(p.y)))).collect();
        out.push_str(&format!("<polygon points=\"{}\"/>\n", path.join(" ")));
    }
    out.push_str("</g>\n<g font-family=\"Helvetica, Arial, sans-serif\" fill=\"black\">\n");
    for t in &page.texts {
        let at = left_of(t);
        // A font's capitals are about 0.72 of its size.
        let size = t.height / 0.72;
        out.push_str(&format!(
            "<text x=\"{}\" y=\"{}\" font-size=\"{}\" transform=\"rotate({} {} {})\">{}</text>\n",
            number(at.x),
            number(y(at.y)),
            number(size),
            number(-t.angle),
            number(at.x),
            number(y(at.y)),
            escape_xml(&t.text)
        ));
    }
    out.push_str("</g>\n</svg>\n");
    out
}

/// A text in PDF's WinAnsi encoding, as a string literal's contents.
fn pdf_string(s: &str) -> String {
    let mut out = String::new();
    for ch in s.chars() {
        let code = match ch {
            '(' | ')' | '\\' => {
                out.push('\\');
                out.push(ch);
                continue;
            }
            '\u{2300}' => 0xD8,
            c if (c as u32) < 0x80 && (c as u32) >= 0x20 => {
                out.push(c);
                continue;
            }
            c if (0xA0..=0xFF).contains(&(c as u32)) => c as u32,
            _ => b'?' as u32,
        };
        out.push_str(&format!("\\{:03o}", code));
    }
    out
}

/// The page as a PDF: one page, its lines as strokes, and its text in Helvetica, which every reader has.
pub fn pdf(page: &Page) -> Vec<u8> {
    let pt = 72.0 / 25.4;
    let mut content = String::new();
    content.push_str("1 J 1 j 0 G 0 g\n");
    for (pen, points) in &page.lines {
        content.push_str(&format!("{} w ", number(pen.width() * pt)));
        if pen.dashes().is_empty() {
            content.push_str("[] 0 d\n");
        } else {
            content.push_str(&format!("[{}] 0 d\n", pen.dashes().iter().map(|d| number(d * pt)).collect::<Vec<_>>().join(" ")));
        }
        for (i, p) in points.iter().enumerate() {
            content.push_str(&format!("{} {} {}\n", number(p.x * pt), number(p.y * pt), if i == 0 { "m" } else { "l" }));
        }
        content.push_str("S\n");
    }
    for f in &page.fills {
        for (i, p) in f.iter().enumerate() {
            content.push_str(&format!("{} {} {}\n", number(p.x * pt), number(p.y * pt), if i == 0 { "m" } else { "l" }));
        }
        content.push_str("f\n");
    }
    for t in &page.texts {
        let at = left_of(t);
        let (c, s) = (t.angle.to_radians().cos(), t.angle.to_radians().sin());
        let size = t.height / 0.72 * pt;
        content.push_str(&format!(
            "BT /F1 {} Tf {} {} {} {} {} {} Tm ({}) Tj ET\n",
            number(size),
            number(c),
            number(s),
            number(-s),
            number(c),
            number(at.x * pt),
            number(at.y * pt),
            pdf_string(&t.text)
        ));
    }
    let objects = [
        "<< /Type /Catalog /Pages 2 0 R >>".to_string(),
        "<< /Type /Pages /Kids [3 0 R] /Count 1 >>".to_string(),
        format!(
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {} {}] /Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>",
            number(page.width * pt),
            number(page.height * pt)
        ),
        format!("<< /Length {} >>\nstream\n{}endstream", content.len(), content),
        "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>".to_string(),
    ];
    let mut out = String::from("%PDF-1.4\n");
    let mut offsets = Vec::with_capacity(objects.len());
    for (i, o) in objects.iter().enumerate() {
        offsets.push(out.len());
        out.push_str(&format!("{} 0 obj\n{}\nendobj\n", i + 1, o));
    }
    let xref = out.len();
    out.push_str(&format!("xref\n0 {}\n0000000000 65535 f \n", objects.len() + 1));
    for o in offsets {
        out.push_str(&format!("{:010} 00000 n \n", o));
    }
    out.push_str(&format!("trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{}\n%%EOF\n", objects.len() + 1, xref));
    out.into_bytes()
}

/// A text as DXF writes it, with its own codes for the diameter, degree and plus-minus signs.
fn dxf_string(s: &str) -> String {
    s.chars()
        .map(|c| match c {
            '\u{D8}' | '\u{2300}' => "%%c".to_string(),
            '\u{B0}' => "%%d".to_string(),
            '\u{B1}' => "%%p".to_string(),
            c if c.is_ascii() => c.to_string(),
            _ => "?".to_string(),
        })
        .collect()
}

/// The page as an R12 DXF, in millimetres: a layer for each pen with its line type, lines as LINEs, arrowheads as SOLIDs
/// and text as TEXT.
pub fn dxf(page: &Page) -> String {
    let mut out = String::new();
    let mut pair = |code: i32, value: &str| {
        out.push_str(&format!("{code}\n{value}\n"));
    };
    pair(0, "SECTION");
    pair(2, "HEADER");
    pair(9, "$ACADVER");
    pair(1, "AC1009");
    pair(9, "$MEASUREMENT");
    pair(70, "1");
    pair(9, "$EXTMIN");
    pair(10, "0");
    pair(20, "0");
    pair(9, "$EXTMAX");
    pair(10, &number(page.width));
    pair(20, &number(page.height));
    pair(0, "ENDSEC");
    pair(0, "SECTION");
    pair(2, "TABLES");
    pair(0, "TABLE");
    pair(2, "LTYPE");
    pair(70, "3");
    for (name, dashes) in [("CONTINUOUS", &[][..]), ("DASHED", Pen::Hidden.dashes()), ("CENTER", Pen::Center.dashes())] {
        pair(0, "LTYPE");
        pair(2, name);
        pair(70, "0");
        pair(3, name);
        pair(72, "65");
        pair(73, &dashes.len().to_string());
        pair(40, &number(dashes.iter().sum::<f64>()));
        for (i, d) in dashes.iter().enumerate() {
            // On is positive and off negative.
            pair(49, &number(if i % 2 == 0 { *d } else { -*d }));
        }
    }
    pair(0, "ENDTAB");
    pair(0, "TABLE");
    pair(2, "LAYER");
    pair(70, "6");
    for (name, ltype) in [("VISIBLE", "CONTINUOUS"), ("THIN", "CONTINUOUS"), ("HIDDEN", "DASHED"), ("CENTER", "CENTER"), ("FRAME", "CONTINUOUS"), ("TEXT", "CONTINUOUS")] {
        pair(0, "LAYER");
        pair(2, name);
        pair(70, "0");
        pair(62, "7");
        pair(6, ltype);
    }
    pair(0, "ENDTAB");
    pair(0, "ENDSEC");
    pair(0, "SECTION");
    pair(2, "ENTITIES");
    for (pen, points) in &page.lines {
        for w in points.windows(2) {
            pair(0, "LINE");
            pair(8, pen.name());
            pair(10, &number(w[0].x));
            pair(20, &number(w[0].y));
            pair(30, "0");
            pair(11, &number(w[1].x));
            pair(21, &number(w[1].y));
            pair(31, "0");
        }
    }
    for f in &page.fills {
        if f.len() < 3 {
            continue;
        }
        pair(0, "SOLID");
        pair(8, "THIN");
        // A triangle's fourth corner is its third again.
        let corners = [f[0], f[1], f[2], *f.get(3).unwrap_or(&f[2])];
        for (k, c) in corners.iter().enumerate() {
            pair(10 + k as i32, &number(c.x));
            pair(20 + k as i32, &number(c.y));
            pair(30 + k as i32, "0");
        }
    }
    for t in &page.texts {
        let at = left_of(t);
        pair(0, "TEXT");
        pair(8, "TEXT");
        pair(10, &number(at.x));
        pair(20, &number(at.y));
        pair(30, "0");
        pair(40, &number(t.height));
        pair(1, &dxf_string(&t.text));
        if t.angle != 0.0 {
            pair(50, &number(t.angle));
        }
    }
    pair(0, "ENDSEC");
    pair(0, "EOF");
    out
}

/// Splits a polyline into its dashes, for drawing it as pixels.
fn dashed(points: &[Vec2], pattern: &[f64]) -> Vec<Vec<Vec2>> {
    if pattern.is_empty() {
        return vec![points.to_vec()];
    }
    let mut out = Vec::new();
    let mut current: Vec<Vec2> = Vec::new();
    let (mut k, mut left) = (0usize, pattern[0]);
    for w in points.windows(2) {
        let (mut a, b) = (w[0], w[1]);
        let mut rest = a.dist(b);
        while rest > 1e-12 {
            let step = left.min(rest);
            let next = a + (b - a) * (step / rest);
            if k % 2 == 0 {
                if current.is_empty() {
                    current.push(a);
                }
                current.push(next);
            }
            rest -= step;
            left -= step;
            a = next;
            if left <= 1e-12 {
                if k % 2 == 0 && current.len() >= 2 {
                    out.push(std::mem::take(&mut current));
                }
                current.clear();
                k = (k + 1) % pattern.len();
                left = pattern[k];
            }
        }
    }
    if current.len() >= 2 {
        out.push(current);
    }
    out
}

/// Pixels, grey levels from white to black.
struct Canvas {
    w: usize,
    h: usize,
    ink: Vec<f32>,
}

impl Canvas {
    fn put(&mut self, x: usize, y: usize, cover: f64) {
        let i = y * self.w + x;
        self.ink[i] = self.ink[i].max(cover.clamp(0.0, 1.0) as f32);
    }

    /// A line of a width, by how far each pixel's middle is from it.
    fn segment(&mut self, a: Vec2, b: Vec2, width: f64) {
        let half = width / 2.0;
        let (x0, x1) = ((a.x.min(b.x) - half - 1.0).floor().max(0.0) as usize, ((a.x.max(b.x) + half + 1.0).ceil() as usize).min(self.w.saturating_sub(1)));
        let (y0, y1) = ((a.y.min(b.y) - half - 1.0).floor().max(0.0) as usize, ((a.y.max(b.y) + half + 1.0).ceil() as usize).min(self.h.saturating_sub(1)));
        let d = b - a;
        let len2 = d.dot(d);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let p = v2(x as f64 + 0.5, y as f64 + 0.5);
                let t = if len2 > 0.0 { ((p - a).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
                let dist = p.dist(a + d * t);
                self.put(x, y, half + 0.5 - dist);
            }
        }
    }

    /// A filled shape of one or more contours, by the even-odd rule, with four by four samples a pixel.
    fn fill(&mut self, contours: &[Vec<Vec2>]) {
        let (mut lo, mut hi) = (v2(f64::INFINITY, f64::INFINITY), v2(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for p in contours.iter().flatten() {
            lo = v2(lo.x.min(p.x), lo.y.min(p.y));
            hi = v2(hi.x.max(p.x), hi.y.max(p.y));
        }
        if !lo.x.is_finite() {
            return;
        }
        let (x0, x1) = (lo.x.floor().max(0.0) as usize, (hi.x.ceil() as usize).min(self.w.saturating_sub(1)));
        let (y0, y1) = (lo.y.floor().max(0.0) as usize, (hi.y.ceil() as usize).min(self.h.saturating_sub(1)));
        for y in y0..=y1 {
            // Crossings of each of four rows of samples through this pixel row.
            let rows: Vec<Vec<f64>> = (0..4)
                .map(|k| {
                    let sy = y as f64 + (k as f64 + 0.5) / 4.0;
                    let mut xs = Vec::new();
                    for c in contours {
                        for i in 0..c.len() {
                            let (a, b) = (c[i], c[(i + 1) % c.len()]);
                            if (a.y <= sy) != (b.y <= sy) {
                                xs.push(a.x + (sy - a.y) / (b.y - a.y) * (b.x - a.x));
                            }
                        }
                    }
                    xs.sort_by(f64::total_cmp);
                    xs
                })
                .collect();
            for x in x0..=x1 {
                let mut hits = 0;
                for xs in &rows {
                    for k in 0..4 {
                        let sx = x as f64 + (k as f64 + 0.5) / 4.0;
                        if xs.iter().filter(|c| **c < sx).count() % 2 == 1 {
                            hits += 1;
                        }
                    }
                }
                if hits > 0 {
                    self.put(x, y, hits as f64 / 16.0);
                }
            }
        }
    }
}

/// A text's glyphs as filled contours, in sheet millimetres.
fn glyph_contours(t: &Text) -> Vec<Vec<Vec2>> {
    let font = Font::builtin();
    let set = font.set(&t.text);
    let k = t.height / font.cap_height;
    let at = left_of(t);
    let (c, s) = (t.angle.to_radians().cos(), t.angle.to_radians().sin());
    let place = |p: [f64; 2]| at + v2(p[0] * k * c - p[1] * k * s, p[0] * k * s + p[1] * k * c);
    let mut out = Vec::new();
    for contour in &set.contours {
        let mut points: Vec<Vec2> = Vec::new();
        for piece in contour {
            match piece {
                Piece::Line(a, b) => {
                    if points.is_empty() {
                        points.push(place(*a));
                    }
                    points.push(place(*b));
                }
                Piece::Quadratic(cs) => {
                    let n = cs.len();
                    if points.is_empty() {
                        points.push(place(cs[0]));
                    }
                    // A run of points off the curve, with the points on it halfway between each two.
                    for i in 1..n - 1 {
                        let start = if i == 1 { cs[0] } else { [(cs[i - 1][0] + cs[i][0]) / 2.0, (cs[i - 1][1] + cs[i][1]) / 2.0] };
                        let end = if i == n - 2 { cs[n - 1] } else { [(cs[i][0] + cs[i + 1][0]) / 2.0, (cs[i][1] + cs[i + 1][1]) / 2.0] };
                        for j in 1..=6 {
                            let u = j as f64 / 6.0;
                            let q = [
                                (1.0 - u) * (1.0 - u) * start[0] + 2.0 * u * (1.0 - u) * cs[i][0] + u * u * end[0],
                                (1.0 - u) * (1.0 - u) * start[1] + 2.0 * u * (1.0 - u) * cs[i][1] + u * u * end[1],
                            ];
                            points.push(place(q));
                        }
                    }
                }
            }
        }
        if points.len() >= 3 {
            out.push(points);
        }
    }
    out
}

/// The page drawn as a PNG, `per_mm` pixels to the millimetre: black on white.
pub fn png(page: &Page, per_mm: f64) -> Vec<u8> {
    let (w, h) = ((page.width * per_mm).ceil().max(1.0) as usize, (page.height * per_mm).ceil().max(1.0) as usize);
    let mut canvas = Canvas { w, h, ink: vec![0.0; w * h] };
    let to_px = |p: Vec2| v2(p.x * per_mm, (page.height - p.y) * per_mm);
    for (pen, points) in &page.lines {
        // At least a pixel wide, so that thin lines show at small sizes.
        let width = (pen.width() * per_mm).max(1.0);
        for run in dashed(points, pen.dashes()) {
            for s in run.windows(2) {
                canvas.segment(to_px(s[0]), to_px(s[1]), width);
            }
        }
    }
    for f in &page.fills {
        canvas.fill(&[f.iter().map(|p| to_px(*p)).collect()]);
    }
    for t in &page.texts {
        let contours: Vec<Vec<Vec2>> = glyph_contours(t).into_iter().map(|c| c.into_iter().map(to_px).collect()).collect();
        canvas.fill(&contours);
    }
    let rgb: Vec<u8> = canvas.ink.iter().flat_map(|ink| {
        let v = (255.0 * (1.0 - *ink as f64)).round() as u8;
        [v, v, v]
    }).collect();
    coreshape_render::png::encode_rgb(w as u32, h as u32, &rgb)
}
