//! Randomized sheets cut by randomized solids and planes, each checked against an identity that doesn't depend on how the cut was
//! found: the part outside and the part inside have to add up to the area of the whole. Each part is valid, and meshes torn only
//! at its boundary. Sewing the two back together has to give the area of the whole too, which is the same as saying that the two
//! cut edges agree. Usage: fuzz_sheets [seeds]. With ALIGNED=1, everything's on a grid and along the axes, which is where
//! surfaces lie on each other and touch, and where refusals come from.

use coreshape::tess::open_only_at_boundary;
use coreshape::*;
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};
use std::time::Instant;

fn line(a: Vec2, b: Vec2, id: u32) -> Seg {
    Seg { geom: SegGeom::Line { a, b }, id }
}

fn arc(radius: f64, a0: f64, a1: f64) -> Wire {
    Wire { segs: vec![Seg { geom: SegGeom::Arc { center: v2(0.0, 0.0), radius, a0, a1 }, id: 0 }] }
}

fn aligned() -> bool {
    std::env::var("ALIGNED").is_ok()
}

/// A length, which is a whole number on a grid.
fn size(r: &mut Rng, lo: f64, hi: f64) -> f64 {
    let x = r.range(lo, hi);
    if aligned() {
        x.round().max(1.0)
    } else {
        x
    }
}

/// Somewhere near the origin, turned any way, or along the axes on a grid.
fn place(r: &mut Rng) -> Frame {
    if aligned() {
        let origin = v3(r.range(-3.0, 3.0).round(), r.range(-3.0, 3.0).round(), r.range(-3.0, 3.0).round());
        let axes = [Vec3::X, Vec3::Y, Vec3::Z];
        let k = r.range(0.0, 3.0) as usize % 3;
        let sign = if r.range(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 };
        return Frame::new(origin, axes[k] * sign, axes[(k + 1) % 3]);
    }
    let origin = v3(r.range(-3.0, 3.0), r.range(-3.0, 3.0), r.range(-3.0, 3.0));
    Frame::new(origin, r.unit_vector(), r.unit_vector())
}

fn sheet(r: &mut Rng) -> Result<(Body, &'static str)> {
    let f = place(r);
    let s = size(r, 4.0, 10.0);
    Ok(match (r.range(0.0, 8.0)) as u32 {
        0 => (fill(&Profile::new(Wire::rect(-s, -s, 2.0 * s, 2.0 * s)), &f, 1)?, "flat"),
        1 => (fill(&Profile::new(Wire::circle(v2(0.0, 0.0), s)).with_hole(Wire::circle(v2(0.0, 0.0), s / 3.0).reversed()), &f, 1)?, "flat with a hole"),
        2 => (extrude_open(&Wire::circle(v2(0.0, 0.0), s), &f, 2.0 * s, 1)?, "tube"),
        3 => (extrude_open(&arc(s, 0.0, r.range(0.5, 3.0)), &f, 2.0 * s, 1)?, "curved panel"),
        4 => (revolve_open(&arc(s, r.range(-1.0, 1.0), PI / 2.0), &f, TAU, 1)?, "dome"),
        5 => (revolve_open(&Wire { segs: vec![line(v2(0.1, 0.0), v2(s, s), 0)] }, &f, TAU, 1)?, "cone"),
        6 => {
            let curve = Spline::through(&[v3(-s, 0.0, 0.0), v3(-s / 3.0, s / 2.0, 0.0), v3(s / 3.0, -s / 2.0, 0.0), v3(s, 0.0, 0.0)])?;
            (extrude_open(&Wire { segs: vec![Seg { geom: SegGeom::Spline(std::sync::Arc::new(curve)), id: 0 }] }, &f, 2.0 * s, 1)?, "spline panel")
        }
        _ => {
            let path = Wire { segs: vec![line(v2(-s, 0.0), v2(0.0, 0.0), 0), line(v2(0.0, 0.0), v2(0.0, s), 1), line(v2(0.0, s), v2(s, s), 2)] };
            (extrude_open(&path, &f, 2.0 * s, 1)?, "panels")
        }
    })
}

fn cutter(r: &mut Rng) -> Result<(Body, &'static str)> {
    let f = place(r);
    let s = size(r, 2.0, 7.0);
    Ok(match (r.range(0.0, 5.0)) as u32 {
        0 => {
            let (dy, dz) = (size(r, 2.0, 7.0), size(r, 2.0, 7.0));
            (box_solid(&f, s, dy, dz, 2)?, "box")
        }
        1 => {
            let h = size(r, 4.0, 20.0);
            (cylinder(&f, s, h, 2)?, "cylinder")
        }
        2 => (sphere(&f, s, 2)?, "sphere"),
        3 => {
            let h = size(r, 3.0, 12.0);
            (cone_frustum(&f, s, s / 2.0, h, 2)?, "cone")
        }
        _ => (torus(&f, s + 1.0, r.range(0.3, 1.0), 2)?, "torus"),
    })
}

fn checked(b: &Body) -> Option<String> {
    if b.faces.is_empty() {
        return None;
    }
    if let Err(why) = validate(b) {
        return Some(format!("invalid: {}", why.join("; ")));
    }
    let m = match tessellate(b, &TessParams::for_size(b.aabb().diagonal())) {
        Ok(m) => m,
        Err(e) => return Some(format!("doesn't mesh: {e}")),
    };
    if !open_only_at_boundary(b, &m) {
        return Some("its mesh is torn away from its boundary".to_string());
    }
    None
}

fn area(b: &Body) -> f64 {
    mass_properties(b).area
}

fn main() {
    let seeds: u64 = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(500);
    let start = Instant::now();
    let (mut right, mut wrong, mut untouched) = (0, 0, 0);
    let mut refusals: BTreeMap<String, (usize, u64)> = BTreeMap::new();
    let (mut sewn, mut sew_refused, mut sew_wrong) = (0, 0, 0);
    let mut refuse = |why: String, seed: u64| {
        let short: String = why.chars().filter(|c| !c.is_ascii_digit()).take(58).collect();
        let e = refusals.entry(short).or_insert((0, seed));
        e.0 += 1;
    };
    for seed in 0..seeds {
        let mut r = Rng::new(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x51ED_270B);
        let (s, what) = match sheet(&mut r) {
            Ok(x) => x,
            Err(e) => {
                println!("seed {seed}: couldn't make a sheet: {e}");
                wrong += 1;
                continue;
            }
        };
        if let Some(why) = checked(&s) {
            println!("seed {seed}: WRONG: {what} on its own: {why}");
            wrong += 1;
            continue;
        }
        // One seed in four is a plane.
        let by_plane = r.range(0.0, 4.0) < 1.0;
        let (outside, inside, tool) = if by_plane {
            let f = place(&mut r);
            let (p, n) = (f.origin, f.z);
            (trim_sheet_by_plane(&s, p, n, Keep::Outside), trim_sheet_by_plane(&s, p, n, Keep::Inside), "plane")
        } else {
            let (c, name) = match cutter(&mut r) {
                Ok(x) => x,
                Err(e) => {
                    println!("seed {seed}: couldn't make a cutter: {e}");
                    wrong += 1;
                    continue;
                }
            };
            (trim_sheet(&s, &c, Keep::Outside), trim_sheet(&s, &c, Keep::Inside), name)
        };
        let (outside, inside) = match (outside, inside) {
            (Ok(a), Ok(b)) => (a, b),
            (Err(e), _) | (_, Err(e)) => {
                refuse(e.to_string(), seed);
                continue;
            }
        };
        if let Some(why) = checked(&outside).or_else(|| checked(&inside)) {
            println!("seed {seed}: WRONG: {what} by a {tool}: {why}");
            wrong += 1;
            continue;
        }
        let whole = area(&s);
        let total = area(&outside) + area(&inside);
        if (total - whole).abs() > 1e-5 * whole {
            println!("seed {seed}: WRONG: {what} by a {tool}: the parts come to {total} of {whole}");
            wrong += 1;
            continue;
        }
        if outside.faces.is_empty() || inside.faces.is_empty() {
            untouched += 1;
            right += 1;
            continue;
        }
        right += 1;
        match sew(&[&outside, &inside], 1e-6) {
            Ok(back) => {
                if (area(&back) - whole).abs() > 1e-5 * whole {
                    println!("seed {seed}: WRONG: {what} by a {tool}: sewn back it's {} of {whole}", area(&back));
                    sew_wrong += 1;
                } else if let Err(why) = validate(&back) {
                    println!("seed {seed}: WRONG: {what} by a {tool}: sewn back it's invalid: {}", why.join("; "));
                    sew_wrong += 1;
                } else {
                    sewn += 1;
                }
            }
            Err(e) => {
                sew_refused += 1;
                if sew_refused <= 5 {
                    println!("seed {seed}: sew refused: {what} by a {tool}: {e}");
                }
            }
        }
    }
    let refused: usize = refusals.values().map(|v| v.0).sum();
    println!("{seeds} seeds: {right} right ({untouched} not reached), {refused} refused, {wrong} wrong. Sewn back: {sewn} right, {sew_refused} refused, {sew_wrong} wrong. {:.1} s", start.elapsed().as_secs_f64());
    for (why, (n, seed)) in refusals {
        println!("{n:6} x {why} (first seed {seed})");
    }
    std::process::exit(if wrong + sew_wrong > 0 { 1 } else { 0 });
}
