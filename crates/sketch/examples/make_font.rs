//! Makes the font this crate comes with, from a TrueType font: its printable ASCII and Latin-1 characters, and a few more
//! that drawings use, in the crate's compact form, with Å's ring lowered into its A. It was made from Noto Sans Regular, under the SIL Open Font License,
//! as `fonts/OFL.txt` says:
//!
//! cargo run -p coreshape-sketch --example make_font -- /usr/share/fonts/truetype/noto/NotoSans-Regular.ttf crates/sketch/fonts/noto_sans.font

use coreshape_sketch::font::Font;

/// What's kept: printable ASCII and Latin-1, and dashes, quotes, the euro, degrees, a diameter sign and a few Greek letters
/// and maths signs, where the font has them.
fn wanted(ch: char) -> bool {
    let c = ch as u32;
    (0x20..=0x7E).contains(&c)
        || (0xA0..=0xFF).contains(&c)
        || "–—‘’‚“”„•…‰€™Ω≈≠≤≥√∞∅⌀ΔΣΦπαβγδεθλμσφω←→↑↓".contains(ch)
}

fn main() {
    let from = std::env::args().nth(1).expect("a TrueType font to read");
    let to = std::env::args().nth(2).expect("where to write the compact font");
    let mut font = Font::from_truetype(&std::fs::read(&from).unwrap(), wanted).unwrap();
    // Noto Sans sets Å's ring on the A's flat top, touching it at one point. A point where two parts of a region meet
    // can't be made a solid's corner, so the ring is lowered 8 units into the A, where it overlaps it as Ç's cedilla does
    // its C, and the two are filled as one: see `Font::outline_pieces`.
    let mut glyphs = font.glyphs.clone();
    if let Some(ring) = glyphs.get_mut(&'\u{C5}') {
        for contour in &mut ring.contours {
            if contour.iter().all(|(p, _)| p[1] > font.cap_height) {
                for (p, _) in contour.iter_mut() {
                    p[1] -= 8.0;
                }
            }
        }
    }
    font = Font::new(font.units_per_em, font.cap_height, font.line_spacing, glyphs);
    let bytes = font.to_compact();
    std::fs::write(&to, &bytes).unwrap();
    let mut missing: Vec<char> = (0x20u32..=0x2400).filter_map(char::from_u32).filter(|c| wanted(*c) && !font.glyphs.contains_key(c)).collect();
    missing.retain(|c| *c != '\u{AD}');
    println!("{} glyphs, {} bytes, cap height {} of {} units; not in it: {missing:?}", font.glyphs.len(), bytes.len(), font.cap_height, font.units_per_em);
}
