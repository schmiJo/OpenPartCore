//! Drawings against what they have to show: where a hole's hidden lines are, where a cylinder's outline is, how much a
//! section hatches, which scale is picked, and that each file format is well formed.

use coreshape::boolean::subtract;
use coreshape::*;
use coreshape_document::json::Value;
use coreshape_drawing::hlr::{project, section, ViewFrame, CUT_FEATURE};
use coreshape_drawing::*;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: v3(x, y, z), ..Frame::WORLD }
}

/// A 30 x 50 x 30 block with a hole of radius 4 down through it at (15, 12).
fn holed_block() -> Body {
    let block = box_solid(&at(0.0, 0.0, 0.0), 30.0, 50.0, 30.0, 1).unwrap();
    subtract(&block, &cylinder(&at(15.0, 12.0, -5.0), 4.0, 40.0, 2).unwrap()).unwrap()
}

fn no_refs(_: &Value) -> Result<Vec3> {
    Err(Error::InvalidInput("none here".into()))
}

/// The vertical runs among these, by where they are across and how long.
fn verticals(runs: &[Vec<Vec2>]) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = runs
        .iter()
        .filter(|r| (r[0].x - r[r.len() - 1].x).abs() < 1e-9 && (r[0].y - r[r.len() - 1].y).abs() > 1e-6)
        .map(|r| (r[0].x, (r[0].y - r[r.len() - 1].y).abs()))
        .collect();
    out.sort_by(|a, b| a.partial_cmp(b).unwrap());
    out
}

fn near(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol, "{what}: {a} isn't {b}");
}

#[test]
fn a_hole_shows_as_hidden_lines_exactly_where_its_sides_are() {
    let body = holed_block();
    // From the front, x across: the hole's sides at 11 and 19, all the way down.
    let front = project(&body, &ViewFrame::looking(-Vec3::Y, Vec3::Z), 0.01, true).unwrap();
    let hidden = verticals(&front.hidden);
    assert_eq!(hidden.len(), 2, "{:?}", front.hidden);
    near(hidden[0].0, 11.0, 1e-9, "left side");
    near(hidden[1].0, 19.0, 1e-9, "right side");
    near(hidden[0].1, 30.0, 1e-9, "full height");
    // From the right, y across: at 8 and 16, in the middle of the hole's two half faces, from the outline traced there.
    let right = project(&body, &ViewFrame::looking(Vec3::X, Vec3::Z), 0.01, true).unwrap();
    let hidden = verticals(&right.hidden);
    assert_eq!(hidden.len(), 2, "{:?}", right.hidden);
    near(hidden[0].0, 8.0, 1e-9, "near side");
    near(hidden[1].0, 16.0, 1e-9, "far side");
    // The block's outline, and nothing hidden under it twice.
    assert_eq!(front.visible.len(), 4, "{:?}", front.visible);
    // From the top, the hole is a whole circle seen face on, round twice over: its top and bottom rims.
    let top = project(&body, &ViewFrame::looking(Vec3::Z, Vec3::Y), 0.01, true).unwrap();
    assert_eq!(top.circles.len(), 1);
    near(top.circles[0].radius, 4.0, 1e-12, "radius");
    assert!(top.circles[0].sweep > 2.0 * PI - 1e-9);
    assert!(top.hidden.is_empty(), "{:?}", top.hidden);
}

#[test]
fn a_cylinder_seen_side_on_has_its_outline_and_axis() {
    // Lying along x, radius 6, from x = 0 to 40, seen from the front: outline at z = 6 and -6 from end to end.
    let rod = cylinder(&Frame::new(Vec3::ZERO, Vec3::X, Vec3::Y), 6.0, 40.0, 1).unwrap();
    let front = project(&rod, &ViewFrame::looking(-Vec3::Y, Vec3::Z), 0.01, true).unwrap();
    let mut levels: Vec<f64> = front.visible.iter().filter(|r| (r[0].y - r[r.len() - 1].y).abs() < 1e-9 && (r[0].x - r[r.len() - 1].x).abs() > 39.0).map(|r| r[0].y).collect();
    levels.sort_by(f64::total_cmp);
    assert_eq!(levels.len(), 2, "{:?}", front.visible);
    near(levels[0], -6.0, 1e-9, "bottom outline");
    near(levels[1], 6.0, 1e-9, "top outline");
    assert_eq!(front.axes.len(), 1);
    let (a, b) = front.axes[0];
    near(a.y, 0.0, 1e-9, "axis");
    near((b.x - a.x).abs(), 40.0, 1e-9, "axis length");
    // Seen end on, it's a circle, and nothing else round.
    let end = project(&rod, &ViewFrame::looking(Vec3::X, Vec3::Z), 0.01, true).unwrap();
    assert_eq!(end.circles.len(), 1);
    assert!(end.axes.is_empty());
}

#[test]
fn a_section_hatches_exactly_what_it_cuts() {
    let body = holed_block();
    // Cut through the hole's axis, square to y, the front half taken away.
    let cut = section(&body, v3(0.0, 12.0, 0.0), -Vec3::Y).unwrap();
    validate(&cut).unwrap();
    near(mass_properties(&cut).volume, 30.0 * 38.0 * 30.0 - PI * 16.0 * 30.0 / 2.0, 1e-6, "what's left");
    let cut_faces = cut.faces.iter().filter(|f| f.tag.feature == CUT_FEATURE).count();
    assert_eq!(cut_faces, 2, "either side of the hole");
    let view = project(&cut, &ViewFrame::looking(-Vec3::Y, Vec3::Z), 0.01, false).unwrap();
    let area: f64 = view.cut.iter().map(|t| ((t[1] - t[0]).cross(t[2] - t[0])).abs() / 2.0).sum();
    near(area, 30.0 * 30.0 - 8.0 * 30.0, 1e-6, "the cut's area");
}

#[test]
fn a_drawing_picks_the_largest_standard_scale_that_fits() {
    let body = holed_block();
    let mut d = Drawing::standard();
    d.sheet = "A4".into();
    let made = page(&d, &body, &no_refs).unwrap();
    assert!(SCALES.contains(&made.scale), "{}", made.scale);
    // Moved far off, it's the same drawing at the same scale.
    let far = holed_block().transformed(&Frame::translation(v3(1000.0, 0.0, 0.0)));
    let page_far = page(&d, &far, &no_refs).unwrap();
    assert_eq!(page_far.scale, made.scale, "moving it doesn't change its scale");
    // A fixed scale is kept, and written in the title block.
    d.scale = Some(0.5);
    let fixed = page(&d, &body, &no_refs).unwrap();
    assert_eq!(fixed.scale, 0.5);
    assert!(fixed.texts.iter().any(|t| t.text == "1:2"));
    assert_eq!(format_scale(2.0), "2:1");
    assert_eq!(format_scale(0.2), "1:5");
}

#[test]
fn dimensions_measure_what_the_view_shows() {
    let body = holed_block();
    let mut d = Drawing::default();
    d.auto_dimensions = false;
    let front = d.add_view(ViewKind::Front, None);
    let top = d.add_view(ViewKind::Top, None);
    d.add_dimension(front, DimKind::Distance { a: Spot::At(v3(0.0, 0.0, 0.0)), b: Spot::At(v3(30.0, 0.0, 30.0)), along: Along::Horizontal }, 10.0, None).unwrap();
    d.add_dimension(front, DimKind::Distance { a: Spot::At(v3(0.0, 0.0, 0.0)), b: Spot::At(v3(30.0, 0.0, 30.0)), along: Along::Vertical }, 10.0, None).unwrap();
    d.add_dimension(front, DimKind::Distance { a: Spot::At(v3(0.0, 0.0, 0.0)), b: Spot::At(v3(30.0, 0.0, 40.0)), along: Along::Aligned }, 10.0, Some("<> mm".into())).unwrap();
    d.add_dimension(top, DimKind::Diameter { near: Spot::At(v3(19.0, 12.0, 30.0)) }, 6.0, None).unwrap();
    d.add_dimension(top, DimKind::Radius { near: Spot::At(v3(15.0, 12.0, 30.0)) }, 6.0, None).unwrap();
    // Asked for a circle where there's none seen face on, it says so and draws the rest.
    d.add_dimension(front, DimKind::Diameter { near: Spot::At(v3(15.0, 12.0, 30.0)) }, 6.0, None).unwrap();
    let made = page(&d, &body, &no_refs).unwrap();
    let texts: Vec<&str> = made.texts.iter().map(|t| t.text.as_str()).collect();
    for want in ["30", "50 mm", "\u{D8}8", "R4"] {
        assert!(texts.contains(&want), "{want} in {texts:?}");
    }
    assert_eq!(made.notes.len(), 1, "{:?}", made.notes);
    assert!(made.notes[0].contains("no circle"), "{:?}", made.notes);
    // A point given by reference is looked up, and one that can't be says why.
    d.dimensions.clear();
    d.add_dimension(front, DimKind::Distance { a: Spot::Ref(Value::str("corner")), b: Spot::At(v3(30.0, 0.0, 0.0)), along: Along::Horizontal }, 10.0, None).unwrap();
    let found = page(&d, &body, &|v: &Value| if v.as_str() == Some("corner") { Ok(v3(10.0, 0.0, 0.0)) } else { Err(Error::InvalidInput("?".into())) }).unwrap();
    assert!(found.texts.iter().any(|t| t.text == "20"), "{:?}", found.texts.iter().map(|t| &t.text).collect::<Vec<_>>());
    let missing = page(&d, &body, &no_refs).unwrap();
    assert!(missing.notes[0].contains("none here"));
}

#[test]
fn every_format_is_well_formed() {
    let body = holed_block();
    let mut d = Drawing::standard();
    d.add_view(ViewKind::Section { point: v3(0.0, 12.0, 0.0), normal: -Vec3::Y, letter: "B".into() }, None);
    d.title.title = "Block (holed) \\ test".into();
    d.title.material = "\u{D8} 6 \u{B1}0.1".into();
    let made = page(&d, &body, &no_refs).unwrap();
    assert!(made.texts.iter().any(|t| t.text == "SECTION B-B"));
    // SVG: one polyline for each line, and the special characters escaped.
    let svg = out::svg(&made);
    assert_eq!(svg.matches("<polyline").count(), made.lines.len());
    assert!(svg.starts_with("<svg") && svg.trim_end().ends_with("</svg>"));
    // PDF: each object where the cross-reference table says it is.
    let pdf = out::pdf(&made);
    let text = String::from_utf8(pdf.clone()).expect("plain text");
    assert!(text.starts_with("%PDF-1.4") && text.ends_with("%%EOF\n"));
    let xref_at: usize = text.lines().rev().nth(1).unwrap().parse().unwrap();
    assert!(text[xref_at..].starts_with("xref"));
    for (k, line) in text[xref_at..].lines().skip(3).take(5).enumerate() {
        let offset: usize = line[..10].parse().unwrap();
        assert!(text[offset..].starts_with(&format!("{} 0 obj", k + 1)), "object {}", k + 1);
    }
    let length: usize = text.split("/Length ").nth(1).unwrap().split(' ').next().unwrap().parse().unwrap();
    let start = text.find("stream\n").unwrap() + "stream\n".len();
    assert_eq!(&text[start + length..start + length + 9], "endstream");
    assert!(text.contains("(Block \\(holed\\) \\\\ test)"));
    assert!(text.contains("\\330 6 \\2610.1"), "WinAnsi codes for the diameter and plus-minus signs");
    // DXF: a LINE for each piece of each line, ending properly, with its own codes for the special signs.
    let dxf = out::dxf(&made);
    let pieces: usize = made.lines.iter().map(|(_, p)| p.len() - 1).sum();
    assert_eq!(dxf.matches("\nLINE\n").count(), pieces);
    assert!(dxf.ends_with("0\nEOF\n"));
    assert!(dxf.contains("%%c 6 %%p0.1"));
    // PNG: the right size for the sheet.
    let png = out::png(&made, 2.0);
    assert_eq!(&png[1..4], b"PNG");
    let width = u32::from_be_bytes(png[16..20].try_into().unwrap());
    let height = u32::from_be_bytes(png[20..24].try_into().unwrap());
    assert_eq!((width, height), (840, 594));
}

#[test]
fn a_drawing_reads_back_as_it_was_written() {
    let mut d = Drawing::standard();
    d.sheet = "A2".into();
    d.projection = Projection::ThirdAngle;
    d.scale = Some(0.2);
    let s = d.add_view(ViewKind::Section { point: v3(1.0, 2.0, 3.0), normal: Vec3::X, letter: "C".into() }, Some([100.0, 80.0]));
    d.add_dimension(s, DimKind::Distance { a: Spot::At(v3(0.0, 0.0, 0.0)), b: Spot::Ref(Value::obj(vec![("faces", Value::Arr(vec![Value::str("f1:end")]))])), along: Along::Vertical }, -6.0, Some("<> max".into())).unwrap();
    d.add_dimension(s, DimKind::Radius { near: Spot::At(v3(1.0, 1.0, 1.0)) }, 4.0, None).unwrap();
    d.title.material = "Steel".into();
    let back = Drawing::from_json(&d.to_json()).unwrap();
    assert_eq!(back, d);
    assert!(Drawing::from_json(&coreshape_document::json::parse(r#"{"sheet": "B7"}"#).unwrap()).is_err());
    assert_eq!(sheet_size("a4"), Some((297.0, 210.0)));
    assert_eq!(sheet_size("ANSI_B"), Some((431.8, 279.4)));
}

#[test]
fn third_angle_puts_the_top_view_above_and_the_right_view_right() {
    let body = holed_block();
    let mut d = Drawing::default();
    d.auto_dimensions = false;
    for kind in [ViewKind::Front, ViewKind::Top, ViewKind::Right] {
        d.add_view(kind, None);
    }
    let first = page(&d, &body, &no_refs).unwrap();
    d.projection = Projection::ThirdAngle;
    let third = page(&d, &body, &no_refs).unwrap();
    // Each view's thick lines, by which side of the front view they're on: the front view is 30 wide and 30 tall, the
    // top 30 by 50, and the right 50 by 30.
    let bounds = |page: &Page| -> Vec<(f64, f64, f64, f64)> {
        let mut boxes: Vec<(f64, f64, f64, f64)> = Vec::new();
        for (pen, line) in &page.lines {
            if *pen != Pen::Thick {
                continue;
            }
            let (x0, x1) = line.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.x), b.max(p.x)));
            let (y0, y1) = line.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.y), b.max(p.y)));
            match boxes.iter_mut().find(|b| x0 <= b.1 + 1.0 && x1 >= b.0 - 1.0 && y0 <= b.3 + 1.0 && y1 >= b.2 - 1.0) {
                Some(b) => *b = (b.0.min(x0), b.1.max(x1), b.2.min(y0), b.3.max(y1)),
                None => boxes.push((x0, x1, y0, y1)),
            }
        }
        boxes
    };
    let (f, t) = (bounds(&first), bounds(&third));
    assert_eq!(f.len(), 3, "{f:?}");
    assert_eq!(t.len(), 3, "{t:?}");
    // The view 50 tall on the sheet's scale is the top view: below the others in first angle, above in third.
    let tallest = |b: &[(f64, f64, f64, f64)]| *b.iter().max_by(|x, y| (x.3 - x.2).total_cmp(&(y.3 - y.2))).unwrap();
    let widest = |b: &[(f64, f64, f64, f64)]| *b.iter().max_by(|x, y| (x.1 - x.0).total_cmp(&(y.1 - y.0))).unwrap();
    let others_y = |b: &[(f64, f64, f64, f64)], skip: (f64, f64, f64, f64)| b.iter().filter(|x| **x != skip).map(|x| x.2).fold(f64::MAX, f64::min);
    let others_x = |b: &[(f64, f64, f64, f64)], skip: (f64, f64, f64, f64)| b.iter().filter(|x| **x != skip).map(|x| x.0).fold(f64::MAX, f64::min);
    assert!(tallest(&f).3 < others_y(&f, tallest(&f)), "first angle: top view below");
    assert!(tallest(&t).2 > others_y(&t, tallest(&t)), "third angle: top view above");
    assert!(widest(&f).1 < others_x(&f, widest(&f)) + 1e-9, "first angle: right view on the left");
    assert!(widest(&t).0 > others_x(&t, widest(&t)), "third angle: right view on the right");
}
