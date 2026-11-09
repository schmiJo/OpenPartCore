//! Random parts through delete face, helix and thicken, against answers known from elsewhere: taking away what a
//! feature added gives the part before it, a helix's volume has a closed form, and a thick layer's mesh has to agree with
//! its exact volume. A refusal isn't a failure, and gets counted. A wrong answer is.
//!
//! `cargo run --release --example fuzz_features -- <count> [delete|helix|thicken|offset|curved]`, or `-- one <kind> <seed>` to look at one.

use coreshape::boolean::{subtract, union};
use coreshape::profile::{Profile, Wire};
use coreshape::*;
use std::f64::consts::{PI, TAU};
use std::time::Instant;

const BASE: u32 = 1;
const TOOL: u32 = 2;

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame { origin: v3(x, y, z), ..Frame::WORLD }
}

/// A part to start from, and what its bounding box is, to place features on.
fn base(rng: &mut Rng) -> Result<(Body, f64, f64, f64)> {
    let (w, d, h) = (rng.range(20.0, 60.0), rng.range(20.0, 60.0), rng.range(8.0, 30.0));
    let block = box_solid(&Frame::WORLD, w, d, h, BASE)?;
    Ok(match rng.int(0, 2) {
        0 => (block, w, d, h),
        1 => {
            let r = rng.range(2.0, w.min(d) * 0.2);
            (subtract(&block, &cylinder(&at(w * 0.25, d * 0.25, -1.0), r, h + 2.0, BASE)?)?, w, d, h)
        }
        _ => {
            let e = (0..block.edges.len()).find(|&e| (block.edges[e].eval_mid().z - h).abs() < 1e-9).unwrap();
            (fillet(&block, &[e], rng.range(0.5, 2.0), BASE)?, w, d, h)
        }
    })
}

/// A feature on the part, whose faces are all tagged `TOOL`.
fn feature(b: &Body, w: f64, d: f64, h: f64, rng: &mut Rng) -> Result<Body> {
    let (x, y) = (rng.range(w * 0.45, w * 0.8), rng.range(d * 0.45, d * 0.8));
    let room = (w - x).min(d - y).min(x).min(y) * 0.6;
    match rng.int(0, 8) {
        0 => subtract(b, &cylinder(&at(x, y, -1.0), rng.range(1.0, room), h + 2.0, TOOL)?),
        1 => subtract(b, &cylinder(&at(x, y, h * rng.range(0.3, 0.8)), rng.range(1.0, room), h, TOOL)?),
        2 => subtract(b, &box_solid(&at(x - room / 2.0, y - room / 2.0, h * rng.range(0.3, 0.8)), room, room * rng.range(0.5, 1.0), h, TOOL)?),
        3 => union(b, &cylinder(&at(x, y, h * 0.5), rng.range(1.0, room), h, TOOL)?),
        4 => subtract(b, &sphere(&at(x, y, h), rng.range(1.0, room), TOOL)?),
        // A slot across, a rabbet along an edge, and a notch out of a corner.
        5 => subtract(b, &box_solid(&at(x - room / 2.0, -1.0, h * rng.range(0.3, 0.8)), room, d + 2.0, h, TOOL)?),
        6 => subtract(b, &box_solid(&at(-1.0, d - room, h * rng.range(0.3, 0.8)), w + 2.0, room + 1.0, h, TOOL)?),
        7 => subtract(b, &box_solid(&at(w - room, d - room, h * rng.range(0.3, 0.8)), room + 1.0, room + 1.0, h, TOOL)?),
        _ => {
            // A blend on edges of the top that are still there.
            let list: Vec<usize> = (0..b.edges.len()).filter(|&e| (b.edges[e].eval_mid().z - h).abs() < 1e-9 && matches!(b.edges[e].curve, Curve::Line { .. })).collect();
            let picked: Vec<usize> = list.iter().copied().filter(|_| rng.int(0, 1) == 0).collect();
            if picked.is_empty() {
                return Err(Error::InvalidInput("no edges".into()));
            }
            if rng.int(0, 1) == 0 {
                fillet(b, &picked, rng.range(0.4, 2.0), TOOL)
            } else {
                chamfer(b, &picked, rng.range(0.4, 2.0), TOOL)
            }
        }
    }
}

enum Outcome {
    Ok,
    Skipped,
    Refused(String),
    Wrong(String),
}

fn delete_case(seed: u64) -> Outcome {
    let mut rng = Rng::new(seed);
    let Ok((b, w, d, h)) = base(&mut rng) else { return Outcome::Skipped };
    let Ok(with) = feature(&b, w, d, h, &mut rng) else { return Outcome::Skipped };
    let faces: Vec<usize> = (0..with.faces.len()).filter(|&f| with.faces[f].tag.feature == TOOL).collect();
    // A feature that changed nothing, or swallowed part of the base's faces, has nothing to answer to.
    let (before, after) = (mass_properties(&b), mass_properties(&with));
    if faces.is_empty() || (before.volume - after.volume).abs() < 1e-6 {
        return Outcome::Skipped;
    }
    let healed = match delete_faces(&with, &faces) {
        Ok(x) => x,
        Err(e) => return Outcome::Refused(e.to_string()),
    };
    if let Err(why) = validate(&healed) {
        return Outcome::Wrong(format!("invalid: {}", why.join("; ")));
    }
    let m = mass_properties(&healed);
    // A blend or cut can swallow a neighbouring feature of the base, and those don't come back.
    let swallowed = b.faces.iter().filter(|f| f.tag.feature == BASE).any(|f| !with.faces.iter().any(|g| g.tag == f.tag));
    if swallowed {
        return Outcome::Ok;
    }
    if (m.volume - before.volume).abs() > 1e-5 * before.volume || (m.area - before.area).abs() > 1e-5 * before.area {
        return Outcome::Wrong(format!("volume {} area {} against {} {}", m.volume, m.area, before.volume, before.area));
    }
    Outcome::Ok
}

fn helix_case(seed: u64) -> Outcome {
    let mut rng = Rng::new(seed);
    let radius = rng.range(3.0, 60.0);
    let pitch = rng.range(0.5, 20.0);
    let turns = rng.range(0.2, 6.0);
    let upright = Frame { origin: v3(0.0, 0.0, rng.range(-5.0, 5.0)), x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
    let tall = pitch * rng.range(0.2, 1.3);
    let wide = (radius * 0.9).min(rng.range(0.3, 8.0));
    let left_handed = rng.int(0, 1) == 0;
    let h = Helix { origin: Vec3::ZERO, axis: Vec3::Z, pitch, turns, left_handed };
    let (profile, expected, height) = match rng.int(0, 3) {
        0 => (
            Profile::new(Wire::polygon(&[v2(radius, 0.0), v2(radius + wide, 0.0), v2(radius + wide, tall), v2(radius, tall)])),
            turns * TAU * tall * ((radius + wide) * (radius + wide) - radius * radius) / 2.0,
            tall,
        ),
        1 => {
            let r = (tall / 2.0).min(wide / 2.0).min(radius * 0.9);
            (Profile::new(Wire::circle(v2(radius, 0.0), r)), turns * TAU * PI * r * r * radius, 2.0 * r)
        }
        2 => (
            // A triangle: the integral of r over it is its area times its centroid's radius.
            Profile::new(Wire::polygon(&[v2(radius, 0.0), v2(radius + wide, tall / 2.0), v2(radius, tall)])),
            turns * TAU * (wide * tall / 2.0) * (radius + wide / 3.0),
            tall,
        ),
        _ => {
            // Across the axis: its area times the height, at any twist.
            let (a, b) = (rng.range(1.0, 20.0), rng.range(1.0, 20.0));
            let square = Profile::new(Wire::polygon(&[v2(-a, -b), v2(a, -b), v2(a, b), v2(-a, b)]));
            let across = Frame::WORLD;
            return match helix(&square, &across, &h, 1) {
                Ok(body) => check_helix(&body, 4.0 * a * b * pitch * turns),
                Err(e) => Outcome::Refused(e.to_string()),
            };
        }
    };
    match helix(&profile, &upright, &h, 1) {
        Ok(body) => {
            if height >= pitch && turns >= 1.0 {
                return Outcome::Wrong(format!("the turns run into each other, with height {height} and pitch {pitch}, and it wasn't refused"));
            }
            check_helix(&body, expected)
        }
        Err(e) => {
            if height >= pitch * 0.999 {
                Outcome::Skipped
            } else {
                Outcome::Refused(e.to_string())
            }
        }
    }
}

fn check_helix(body: &Body, expected: f64) -> Outcome {
    if let Err(why) = validate(body) {
        return Outcome::Wrong(format!("invalid: {}", why.join("; ")));
    }
    let v = mass_properties(body).volume;
    if (v - expected).abs() > 1e-5 * expected {
        return Outcome::Wrong(format!("volume {v} against {expected}"));
    }
    Outcome::Ok
}

fn thicken_case(seed: u64) -> Outcome {
    let mut rng = Rng::new(seed);
    let (b, w, d, h) = match rng.int(0, 4) {
        0 | 1 => {
            let Ok(x) = base(&mut rng) else { return Outcome::Skipped };
            x
        }
        2 => {
            let (r, t) = (rng.range(5.0, 30.0), rng.range(5.0, 40.0));
            let Ok(c) = cylinder(&Frame::WORLD, r, t, BASE) else { return Outcome::Skipped };
            (c, r, r, t)
        }
        3 => {
            let r = rng.range(5.0, 30.0);
            let Ok(c) = sphere(&Frame::WORLD, r, BASE) else { return Outcome::Skipped };
            (c, r, r, r)
        }
        _ => {
            let p = Profile::new(Wire::circle(v2(0.0, 0.0), rng.range(5.0, 20.0)));
            let q = Profile::new(Wire::circle(v2(0.0, 0.0), rng.range(5.0, 20.0)));
            let r = Profile::new(Wire::circle(v2(0.0, 0.0), rng.range(5.0, 20.0)));
            let (a, c) = (rng.range(8.0, 20.0), rng.range(8.0, 20.0));
            let Ok(l) = loft(&[(&p, at(0.0, 0.0, 0.0)), (&q, at(0.0, 0.0, a)), (&r, at(0.0, 0.0, a + c))], BASE) else { return Outcome::Skipped };
            (l, 20.0, 20.0, a + c)
        }
    };
    let _ = (w, d, h);
    let f = rng.int(0, b.faces.len() as i64 - 1) as usize;
    // Its smooth neighbours too, as those have to go together when they're one surface split in two.
    let mut faces = vec![f];
    let uses = b.edge_uses();
    for c in b.faces[f].loops.iter().flatten() {
        for u in &uses[c.edge] {
            let g = u.0;
            let p = b.edges[c.edge].eval_mid();
            if g != f && !faces.contains(&g) && b.faces[f].outward_normal(p).dist(b.faces[g].outward_normal(p)) < 1e-9 {
                faces.push(g);
            }
        }
    }
    let t = rng.range(0.2, 3.0) * if rng.int(0, 1) == 0 { 1.0 } else { -1.0 };
    let thick = match thicken(&b, &faces, t, 5) {
        Ok(x) => x,
        Err(e) => return Outcome::Refused(e.to_string()),
    };
    if let Err(why) = validate(&thick) {
        return Outcome::Wrong(format!("invalid: {}", why.join("; ")));
    }
    let mesh = match tessellate(&thick, &TessParams::for_size(thick.aabb().diagonal())) {
        Ok(m) => m,
        Err(e) => return Outcome::Wrong(format!("doesn't mesh: {e}")),
    };
    if mesh.open_edge_count() > 0 {
        return Outcome::Wrong("mesh isn't closed".into());
    }
    let m = mass_properties(&thick);
    if !(m.volume > 0.0) || (mesh.volume() - m.volume).abs() > 0.02 * m.volume {
        return Outcome::Wrong(format!("mesh {} against {}", mesh.volume(), m.volume));
    }
    Outcome::Ok
}

/// Offset, on parts with features small enough to vanish. Whatever gave way, a point closer to the part than the distance
/// has to be in the result of moving out, and every point in the result of moving in has to be at least that far inside.
fn offset_case(seed: u64) -> Outcome {
    let mut rng = Rng::new(seed);
    let Ok((b, w, d, h)) = base(&mut rng) else { return Outcome::Skipped };
    offset_check(b, w, d, h, rng)
}

/// The same on round and free-form parts.
fn offset_curved_case(seed: u64) -> Outcome {
    let mut rng = Rng::new(seed ^ 0xC0FFEE);
    let (w, d, h) = (rng.range(24.0, 50.0), rng.range(24.0, 50.0), rng.range(10.0, 30.0));
    let (cx, cy) = (w / 2.0, d / 2.0);
    let r = w.min(d) / 2.0;
    let made = match rng.int(0, 2) {
        0 => cylinder(&at(cx, cy, 0.0), r, h, BASE),
        1 => {
            let (a, b) = (r * rng.range(0.7, 1.0), r * rng.range(0.6, 1.0));
            let (p, q) = (Profile::new(Wire::circle(v2(cx, cy), r)), Profile::new(Wire::circle(v2(cx, cy), a)));
            let top = Profile::new(Wire::circle(v2(cx, cy), b));
            loft(&[(&p, at(0.0, 0.0, 0.0)), (&q, at(0.0, 0.0, h * 0.5)), (&top, at(0.0, 0.0, h))], BASE)
        }
        _ => {
            let c = coreshape::spline::Spline::ellipse_arc(v3(cx, cy, 0.0), Vec3::X, Vec3::Y, r, r * rng.range(0.6, 0.95), 0.0, TAU);
            match c {
                Ok(c) => extrude(&Profile::new(coreshape::profile::PathBuilder::new(v2(cx + r, cy)).spline_to(c).close()), &Frame::WORLD, h, BASE),
                Err(e) => Err(e),
            }
        }
    };
    let Ok(b) = made else { return Outcome::Skipped };
    offset_check(b, w, d, h, rng)
}

fn offset_check(b: Body, w: f64, d: f64, h: f64, mut rng: Rng) -> Outcome {
    use coreshape::boolean::classify::{BodyLoc, Index};
    let Ok(part) = feature(&b, w, d, h, &mut rng) else { return Outcome::Skipped };
    let distance = rng.range(0.3, (w.min(d).min(h)) * 0.35) * if rng.int(0, 1) == 0 { 1.0 } else { -1.0 };
    let moved = match offset(&part, distance) {
        Ok(x) => x,
        Err(e) => return Outcome::Refused(e.to_string()),
    };
    if let Err(why) = validate(&moved) {
        return Outcome::Wrong(format!("invalid: {}", why.join("; ")));
    }
    let mesh_of_moved = match tessellate(&moved, &TessParams::for_size(moved.aabb().diagonal())) {
        Ok(m) => m,
        Err(e) => return Outcome::Wrong(format!("doesn't mesh: {e}")),
    };
    if mesh_of_moved.open_edge_count() > 0 {
        return Outcome::Wrong("mesh isn't closed".into());
    }
    let m = mass_properties(&moved);
    if !(m.volume > 0.0) || (mesh_of_moved.volume() - m.volume).abs() > 0.02 * m.volume {
        return Outcome::Wrong(format!("mesh {} against {}", mesh_of_moved.volume(), m.volume));
    }
    // The distance from a point to the part's own surface, by its mesh, which is good to the mesh's own chord.
    let mesh = match tessellate(&part, &TessParams::for_size(part.aabb().diagonal())) {
        Ok(m) => m,
        Err(e) => return Outcome::Wrong(format!("the part doesn't mesh: {e}")),
    };
    let to_surface = |p: Vec3| -> f64 {
        mesh.triangles.iter().map(|t| triangle_distance(p, mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize])).fold(f64::INFINITY, f64::min)
    };
    let (of_part, of_moved) = (Index::new(&part), Index::new(&moved));
    let bounds = part.aabb().inflated(distance.abs() * 1.2);
    let slack = 2e-3 * part.aabb().diagonal();
    for _ in 0..120 {
        let p = v3(rng.range(bounds.min.x, bounds.max.x), rng.range(bounds.min.y, bounds.max.y), rng.range(bounds.min.z, bounds.max.z));
        let (in_part, in_moved) = (of_part.locate_in_body(p), of_moved.locate_in_body(p));
        // Sharp corners make the result reach further than the distance, by up to the root of 3 at a corner of three square
        // faces, so anything well past that has to be on the other side, as well as anything nearer than the distance.
        const REACH: f64 = 2.0;
        if distance > 0.0 {
            // Moving out: anything in the part, or closer to it than the distance, has to be in the result.
            if in_moved == BodyLoc::Outside && (in_part != BodyLoc::Outside || to_surface(p) < distance - slack) {
                return Outcome::Wrong(format!("{p:?} is {} from the part and left out, moving out by {distance}", to_surface(p)));
            }
            // And anything far outside the part stays out of it.
            if in_moved == BodyLoc::Inside && in_part == BodyLoc::Outside && to_surface(p) > REACH * distance + slack {
                return Outcome::Wrong(format!("{p:?} is {} outside the part, but in the result of moving out by {distance}", to_surface(p)));
            }
        } else {
            // Moving in: anything in the result has to be in the part, and at least the distance inside it.
            if in_moved == BodyLoc::Inside && (in_part != BodyLoc::Inside || to_surface(p) < -distance - slack) {
                return Outcome::Wrong(format!("{p:?} is in the result, but {} from the part's surface, moving in by {distance}", to_surface(p)));
            }
            // And anything deep inside the part stays in it.
            if in_moved == BodyLoc::Outside && in_part == BodyLoc::Inside && to_surface(p) > -REACH * distance + slack {
                return Outcome::Wrong(format!("{p:?} is {} inside the part, but left out of the result of moving in by {distance}", to_surface(p)));
            }
        }
    }
    Outcome::Ok
}

fn triangle_distance(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let (ab, ac, ap) = (b - a, c - a, p - a);
    let (d1, d2) = (ab.dot(ap), ac.dot(ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.norm();
    }
    let bp = p - b;
    let (d3, d4) = (ab.dot(bp), ac.dot(bp));
    if d3 >= 0.0 && d4 <= d3 {
        return bp.norm();
    }
    let cp = p - c;
    let (d5, d6) = (ab.dot(cp), ac.dot(cp));
    if d6 >= 0.0 && d5 <= d6 {
        return cp.norm();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return (ap - ab * (d1 / (d1 - d3))).norm();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return (ap - ac * (d2 / (d2 - d6))).norm();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return (bp - (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))).norm();
    }
    let denom = 1.0 / (va + vb + vc);
    (ap - ab * (vb * denom) - ac * (vc * denom)).norm()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("one") {
        let seed: u64 = args[3].parse().unwrap();
        let out = match args[2].as_str() {
            "delete" => delete_case(seed),
            "helix" => helix_case(seed),
            "offset" => offset_case(seed),
            "curved" => offset_curved_case(seed),
            _ => thicken_case(seed),
        };
        match out {
            Outcome::Ok => println!("fine"),
            Outcome::Skipped => println!("skipped"),
            Outcome::Refused(why) => println!("refused: {why}"),
            Outcome::Wrong(why) => println!("WRONG: {why}"),
        }
        return;
    }
    let count: u64 = args.get(1).and_then(|c| c.parse().ok()).unwrap_or(200);
    let only = args.get(2).cloned();
    let kinds: Vec<(&str, fn(u64) -> Outcome)> = vec![("delete", delete_case), ("helix", helix_case), ("thicken", thicken_case), ("offset", offset_case), ("curved", offset_curved_case)];
    for (name, case) in kinds {
        if only.as_deref().is_some_and(|o| o != name) {
            continue;
        }
        let start = Instant::now();
        let (mut fine, mut skipped, mut refused, mut wrong) = (0, 0, 0, 0);
        let mut why: std::collections::BTreeMap<String, (usize, u64)> = Default::default();
        for seed in 0..count {
            match case(seed) {
                Outcome::Ok => fine += 1,
                Outcome::Skipped => skipped += 1,
                Outcome::Refused(r) => {
                    refused += 1;
                    // Without what differs from one to the next.
                    let short: String = r.split(" near (").next().unwrap_or(&r).chars().take(70).collect();
                    why.entry(short).or_insert((0, seed)).0 += 1;
                }
                Outcome::Wrong(r) => {
                    wrong += 1;
                    println!("{name} seed {seed} WRONG: {r}");
                }
            }
        }
        println!("{name}: {fine} right, {refused} refused, {wrong} wrong, {skipped} skipped, {:.1} s", start.elapsed().as_secs_f64());
        for (r, (n, seed)) in why {
            println!("   {n} x {r} (seed {seed})");
        }
    }
}
