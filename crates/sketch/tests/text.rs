//! Text in sketches: each character's outline becomes curves of the sketch, which make regions as any curves do. Each area
//! is held against one worked out straight from the font's points, by the area of the polygon through the points on the
//! curve and two thirds of each parabola's triangle, which shares nothing with the arrangement's walk round its regions.

use coreshape_sketch::font::{Font, Piece};
use coreshape_sketch::*;

/// The area a contour's pieces enclose, positive anticlockwise: each line's and each quadratic B-spline's, as the sum of
/// its Bezier pieces, each the chord's part of the polygon and two thirds of its triangle.
fn piece_area(pieces: &[Piece]) -> f64 {
    let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
    let mut area = 0.0;
    for p in pieces {
        match p {
            Piece::Line(a, b) => area += cross(*a, *b) / 2.0,
            Piece::Quadratic(c) => {
                // Its Bezier pieces: on the curve at the ends and half way between each pair of points off it.
                let n = c.len();
                let mid = |i: usize| [(c[i][0] + c[i + 1][0]) / 2.0, (c[i][1] + c[i + 1][1]) / 2.0];
                for k in 1..n - 1 {
                    let a = if k == 1 { c[0] } else { mid(k - 1) };
                    let b = if k == n - 2 { c[n - 1] } else { mid(k) };
                    let o = c[k];
                    let triangle = ((o[0] - a[0]) * (b[1] - a[1]) - (o[1] - a[1]) * (b[0] - a[0])) / 2.0;
                    area += cross(a, b) / 2.0 + 2.0 / 3.0 * triangle;
                }
            }
        }
    }
    area
}

/// The area the font's outlines of `text` enclose, in its units: material less holes, whichever way the font winds.
fn font_area(text: &str) -> f64 {
    Font::builtin().set(text).contours.iter().map(|c| piece_area(c)).sum::<f64>().abs()
}

fn region_area(s: &Sketch) -> f64 {
    s.regions().unwrap().iter().map(Region::area).sum::<f64>().abs()
}

#[test]
fn text_makes_regions_with_its_letters_holes() {
    let mut s = Sketch::new();
    let t = s.text("Ho", [10.0, 5.0], 7.14);
    let regions = s.regions().unwrap();
    // H, and o with a hole.
    assert_eq!(regions.len(), 2, "{regions:?}");
    assert_eq!(regions.iter().map(|r| r.holes.len()).sum::<usize>(), 1);
    // A capital is 714 units high in the font, so 7.14 high is a scale of 0.01.
    let want = font_area("Ho") * 0.01 * 0.01;
    assert!((region_area(&s) - want).abs() < 1e-9 * want, "{} against {want}", region_area(&s));
    // Its curves are the text's, by its id.
    assert!(regions.iter().all(|r| r.outer.segs.iter().all(|g| g.entity == t)));
    // From x = 10 on its baseline: H stands on it, and o goes a little below, as round letters do.
    let lowest = |r: &Region| r.outer.segs.iter().map(|g| g.start()[1]).fold(f64::INFINITY, f64::min);
    let h = regions.iter().find(|r| r.holes.is_empty()).unwrap();
    assert!((lowest(h) - 5.0).abs() < 1e-9, "{}", lowest(h));
    let left = h.outer.segs.iter().map(|g| g.start()[0]).fold(f64::INFINITY, f64::min);
    assert!(left > 10.0 && left < 11.0, "{left}");
}

#[test]
fn text_turns_and_sizes_with_its_points() {
    // Its points constrained: at the origin, up along -x at a distance of 20. It reads upward, rotated a quarter turn.
    let mut s = Sketch::new();
    let t = s.text("E", [3.0, 4.0], 5.0);
    let (at, up) = (s.text_of(t).unwrap().at, s.text_of(t).unwrap().up);
    s.constrain(Rule::Coincident { a: Ref { entity: at, part: Part::Whole }, b: Ref { entity: ORIGIN, part: Part::Whole } });
    s.constrain(Rule::Horizontal { a: Ref { entity: at, part: Part::Whole }, b: Some(Ref { entity: up, part: Part::Whole }) });
    s.constrain(Rule::HorizontalDistance { a: Ref { entity: at, part: Part::Whole }, b: Ref { entity: up, part: Part::Whole }, value: (-20.0).into() });
    let solved = s.solve().unwrap();
    let regions = solved.sketch.regions().unwrap();
    let want = font_area("E") * (20.0 / Font::builtin().cap_height).powi(2);
    assert!((region_area(&solved.sketch) - want).abs() < 1e-9 * want);
    // Up is -x, so the letter's height runs from x = 0 to -20, and it goes along +y.
    let xs: Vec<f64> = regions.iter().flat_map(|r| r.outer.segs.iter()).map(|g| g.start()[0]).collect();
    let (lo, hi) = (xs.iter().copied().fold(f64::INFINITY, f64::min), xs.iter().copied().fold(f64::NEG_INFINITY, f64::max));
    assert!(lo > -20.01 && lo < -19.99 && hi.abs() < 0.01, "{lo} {hi}");
}

#[test]
fn text_saves_loads_and_goes_with_its_points() {
    let mut s = Sketch::new();
    let t = s.text("A b\nC", [0.0, 0.0], 10.0);
    let back = Sketch::from_json(&s.to_json()).unwrap();
    assert_eq!(back, s);
    assert_eq!(back.regions().unwrap().len(), s.regions().unwrap().len());
    // Two lines: C is a line's height lower.
    let lowest = s.regions().unwrap().iter().flat_map(|r| r.outer.segs.iter()).map(|g| g.start()[1]).fold(f64::INFINITY, f64::min);
    assert!(lowest < -10.0, "{lowest}");
    // Removing a point it's placed by removes it, and the other point.
    let at = s.text_of(t).unwrap().at;
    let mut gone = s.clone();
    gone.remove_entity(at).unwrap();
    assert!(gone.texts.is_empty() && gone.entities.is_empty());
    let mut gone = s.clone();
    gone.remove_entity(t).unwrap();
    assert!(gone.texts.is_empty() && gone.entities.is_empty());
    // A character the font hasn't got is left out, saying so, and a font there isn't is refused.
    let mut odd = Sketch::new();
    let t = odd.text("a\u{2603}", [0.0, 0.0], 5.0);
    let found = odd.arrangement();
    assert!(found.ignored.iter().any(|i| i.entity == t && i.why == WhyIgnored::NotInFont { ch: '\u{2603}' }), "{:?}", found.ignored);
    assert_eq!(found.regions.len(), 2);
    let mut json = s.to_json().to_text();
    json = json.replace("\"font\":\"sans\"", "\"font\":\"comic\"");
    let why = Sketch::load_text(&json).unwrap_err().message;
    assert!(why.contains("isn't one there is"), "{why}");
}

/// What a glyph's own contours fill by TrueType's rule, wherever they wind round a point at all, by a raster: each cell's
/// middle tested against the contours drawn as polylines of 64 lines a parabola. Nothing shared with how the kernel does it.
fn raster_area(ch: char) -> f64 {
    let glyph = &Font::builtin().glyphs[&ch];
    let mut lines: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for contour in &glyph.contours {
        for piece in coreshape_sketch::font::pieces_of(contour) {
            match piece {
                Piece::Line(a, b) => lines.push((a, b)),
                Piece::Quadratic(c) => {
                    let n = c.len();
                    let mid = |i: usize| [(c[i][0] + c[i + 1][0]) / 2.0, (c[i][1] + c[i + 1][1]) / 2.0];
                    for k in 1..n - 1 {
                        let a = if k == 1 { c[0] } else { mid(k - 1) };
                        let b = if k == n - 2 { c[n - 1] } else { mid(k) };
                        let at = |t: f64| [(1.0 - t) * (1.0 - t) * a[0] + 2.0 * t * (1.0 - t) * c[k][0] + t * t * b[0], (1.0 - t) * (1.0 - t) * a[1] + 2.0 * t * (1.0 - t) * c[k][1] + t * t * b[1]];
                        for j in 0..64 {
                            lines.push((at(j as f64 / 64.0), at((j + 1) as f64 / 64.0)));
                        }
                    }
                }
            }
        }
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (a, _) in &lines {
        lo = [lo[0].min(a[0]), lo[1].min(a[1])];
        hi = [hi[0].max(a[0]), hi[1].max(a[1])];
    }
    let n = 800;
    let (dx, dy) = ((hi[0] - lo[0]) / n as f64, (hi[1] - lo[1]) / n as f64);
    let mut filled = 0usize;
    for i in 0..n {
        for j in 0..n {
            let p = [lo[0] + (i as f64 + 0.5) * dx, lo[1] + (j as f64 + 0.37) * dy];
            let mut w = 0;
            for (a, b) in &lines {
                if (a[1] <= p[1]) != (b[1] <= p[1]) {
                    let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                    if x > p[0] {
                        w += if b[1] > a[1] { 1 } else { -1 };
                    }
                }
            }
            filled += usize::from(w != 0);
        }
    }
    filled as f64 * dx * dy
}

#[test]
fn every_character_of_the_font_makes_regions() {
    // Each character on its own: where its contours don't cross, its regions' area is the font's to the last digits.
    // Where they do, as an accent laid over a letter can, its regions are what the contours fill together, and they're
    // held to a raster of that.
    let font = Font::builtin();
    let mut chars: Vec<char> = font.glyphs.keys().copied().filter(|c| !c.is_whitespace()).collect();
    chars.sort();
    let mut bad = Vec::new();
    let mut crossing = Vec::new();
    for ch in chars {
        let mut s = Sketch::new();
        s.text(&ch.to_string(), [0.0, 0.0], font.cap_height);
        let got = s.regions().map(|r| r.iter().map(Region::area).sum::<f64>().abs()).unwrap_or(-1.0);
        let own: f64 = font.glyphs[&ch].contours.iter().map(|c| piece_area(&coreshape_sketch::font::pieces_of(c))).sum::<f64>().abs();
        if (got - own).abs() <= 1e-7 * own.max(1.0) {
            continue;
        }
        crossing.push(ch);
        let raster = raster_area(ch);
        if (got - raster).abs() > 0.004 * raster {
            bad.push(format!("{ch:?}: {got} against a raster's {raster}, and {own} by its contours"));
        }
    }
    assert!(bad.is_empty(), "{} of them: {bad:?}", bad.len());
    assert_eq!(crossing, vec!['Å', 'Ç', 'ç'], "the glyphs whose contours cross");
}
