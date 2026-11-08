//! Compares this solver with FreeCAD's PlaneGCS on random sketches, as an independent check.
//!
//!   oracle export <dir> <cases per category> [first seed]   writes cases.jsonl with this solver's answers
//!   scripts/sketch_oracle/run.sh <dir>                      adds PlaneGCS's answers from the FreeCAD build
//!   oracle compare <dir>                                    prints the report
//!
//! The two solvers can land on different valid solutions, so positions aren't compared. Each solver's
//! answer is given to the other: an answer is right if the other accepts it without moving it.

use coreshape_sketch::fuzz::*;
use coreshape_sketch::json::{self, Value};
use coreshape_sketch::rng::Rng;
use coreshape_sketch::*;
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::time::Instant;

fn nums(list: &[ConstraintId]) -> Value {
    Value::Arr(list.iter().map(|c| Value::Num(c.0 as f64)).collect())
}

fn expect_value(e: &Expect) -> Value {
    match e {
        Expect::Solvable => Value::str("solvable"),
        Expect::Impossible => Value::str("impossible"),
        Expect::Redundant(ids) => Value::obj(vec![("redundant", nums(ids))]),
        Expect::Conflict(ids) => Value::obj(vec![("conflict", nums(ids))]),
    }
}

/// A point to drag: something still free, moved a little.
fn pick_drag(s: &Sketch, free: &[Ref], rng: &mut Rng) -> Option<(Ref, P2)> {
    let points: Vec<Ref> = free.iter().copied().filter(|r| r.part != Part::Radius).collect();
    if points.is_empty() {
        return None;
    }
    let r = points[rng.int(0, points.len() as i64 - 1) as usize];
    let at = match (s.entity(r.entity)?.geometry, r.part) {
        (Geometry::Point { at }, _) => at,
        (Geometry::Line { start, .. }, Part::Start) => start,
        (Geometry::Line { end, .. }, Part::End) => end,
        (Geometry::Circle { center, .. } | Geometry::Arc { center, .. }, Part::Center) => center,
        (Geometry::Arc { start, .. }, Part::Start) => start,
        (Geometry::Arc { end, .. }, Part::End) => end,
        _ => return None,
    };
    let reach = rng.range(1.0, 15.0);
    let turn = rng.range(0.0, std::f64::consts::TAU);
    Some((r, [at[0] + reach * turn.cos(), at[1] + reach * turn.sin()]))
}

fn export(dir: &str, cases: u64, first: u64) {
    std::fs::create_dir_all(dir).unwrap();
    let mut text = String::new();
    for category in ALL_CATEGORIES {
        // PlaneGCS takes seconds on each of these, so they get fewer.
        let count = if category == Category::Large { (cases / 20).max(1) } else { cases };
        for seed in first..first + count {
            let case = generate(category, seed);
            let t = Instant::now();
            let result = case.start.solve();
            let micros = t.elapsed().as_secs_f64() * 1e6;
            let mut items = vec![
                ("category", Value::str(category.name())),
                ("seed", Value::Num(seed as f64)),
                ("expect", expect_value(&case.expect)),
                ("start", case.start.to_json()),
            ];
            match &result {
                Ok(solved) => {
                    let mut rng = Rng::new(seed ^ 0xD3A6);
                    let mut mine = vec![
                        ("ok", Value::Bool(true)),
                        ("dof", Value::Num(solved.analysis.dof as f64)),
                        ("redundant", nums(&solved.analysis.redundant.iter().map(|d| d.constraint).collect::<Vec<_>>())),
                        ("solved", solved.sketch.to_json()),
                        ("micros", Value::Num(micros)),
                    ];
                    if let Some((grab, to)) = pick_drag(&solved.sketch, &solved.analysis.free, &mut rng) {
                        if let Ok((dragged, report)) = solved.sketch.drag(Grab::Point(grab), to) {
                            items.push((
                                "drag",
                                Value::obj(vec![
                                    ("grab", Value::str(grab.name())),
                                    ("to", Value::Arr(vec![Value::Num(to[0]), Value::Num(to[1])])),
                                    ("reached", Value::Bool(report.reached)),
                                    ("miss", Value::Num(report.distance)),
                                    ("moved", Value::Num(moved_others(&solved.sketch, &dragged, grab))),
                                    ("sketch", dragged.to_json()),
                                ]),
                            ));
                        }
                    }
                    mine.shrink_to_fit();
                    items.push(("mine", Value::obj(mine)));
                }
                Err(e) => {
                    items.push((
                        "mine",
                        Value::obj(vec![
                            ("ok", Value::Bool(false)),
                            ("kind", e.to_json().get("kind").cloned().unwrap_or(Value::Null)),
                            ("message", Value::str(&e.message)),
                            ("constraints", nums(&e.constraints)),
                            ("micros", Value::Num(micros)),
                        ]),
                    ));
                }
            }
            text.push_str(&Value::obj(items).to_text());
            text.push('\n');
        }
    }
    std::fs::write(format!("{dir}/cases.jsonl"), text).unwrap();
}

/// How far everything other than the grabbed part moved: the root of the summed squares.
fn moved_others(a: &Sketch, b: &Sketch, grab: Ref) -> f64 {
    let parts = |g: &Geometry| -> Vec<(Part, P2)> {
        match *g {
            Geometry::Point { at } => vec![(Part::Whole, at)],
            Geometry::Line { start, end } => vec![(Part::Start, start), (Part::End, end)],
            Geometry::Circle { center, radius } => vec![(Part::Center, center), (Part::Radius, [radius, 0.0])],
            Geometry::Arc { center, radius, start, end } => {
                vec![(Part::Center, center), (Part::Radius, [radius, 0.0]), (Part::Start, start), (Part::End, end)]
            }
        }
    };
    let mut sum = 0.0;
    for (x, y) in a.entities.iter().zip(&b.entities) {
        for ((part, p), (_, q)) in parts(&x.geometry).into_iter().zip(parts(&y.geometry)) {
            if x.id == grab.entity && part == grab.part {
                continue;
            }
            sum += (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
        }
    }
    sum.sqrt()
}

/// `s` without the constraint PlaneGCS has no form of, for judging what PlaneGCS did: it was never
/// asked to meet that one.
fn without_unmapped(s: &Sketch) -> Sketch {
    let mut out = s.clone();
    let curve = |id: EntityId| matches!(s.entity(id).map(|e| e.geometry), Some(Geometry::Circle { .. } | Geometry::Arc { .. }));
    out.constraints.retain(|c| !matches!(c.rule, Rule::Normal { a, b } if curve(a) && curve(b)));
    out
}

/// A copy of `s` with geometry from PlaneGCS's flat lists.
fn with_geometry(s: &Sketch, list: &[Value]) -> Option<Sketch> {
    let mut out = s.clone();
    for (e, values) in out.entities.iter_mut().zip(list) {
        let v: Vec<f64> = values.as_array()?.iter().filter_map(Value::as_f64).collect();
        e.geometry = match e.geometry {
            Geometry::Point { .. } => Geometry::Point { at: [v[0], v[1]] },
            Geometry::Line { .. } => Geometry::Line { start: [v[0], v[1]], end: [v[2], v[3]] },
            Geometry::Circle { .. } => Geometry::Circle { center: [v[0], v[1]], radius: v[2] },
            Geometry::Arc { .. } => Geometry::Arc { center: [v[0], v[1]], radius: v[2], start: [v[3], v[4]], end: [v[5], v[6]] },
        };
    }
    Some(out)
}

#[derive(Default)]
struct Row {
    cases: usize,
    both: usize,
    agree: usize,
    dof_differ: usize,
    my_refusals: usize,
    their_refusals: usize,
    both_refuse: usize,
    wrong: usize,
    my_time: f64,
    their_time: f64,
    drags: usize,
    drag_ok: usize,
    drag_their_ok: usize,
    my_moved: f64,
    their_moved: f64,
    my_miss: usize,
    their_miss: usize,
    unmapped: usize,
    red_differ: usize,
    named_differently: usize,
    fair_drags: usize,
    my_fair_moved: f64,
    their_fair_moved: f64,
    their_collapses: usize,
    their_wrong: usize,
    their_drag_broken: usize,
    my_short: usize,
}

fn compare(dir: &str) {
    let cases = std::fs::read_to_string(format!("{dir}/cases.jsonl")).unwrap();
    let answers = std::fs::read_to_string(format!("{dir}/planegcs.jsonl")).unwrap();
    let mut rows: BTreeMap<String, Row> = BTreeMap::new();
    let mut order: Vec<String> = Vec::new();
    let mut log = String::new();
    let mut reasons: BTreeMap<String, (usize, String)> = BTreeMap::new();
    let mut note = |log: &mut String, kind: &str, category: &str, seed: u64, detail: String| {
        let _ = writeln!(log, "{kind} {category} {seed}: {detail}");
        let key: String = format!("{kind} {category}: {}", detail.split(|c: char| c.is_ascii_digit()).collect::<Vec<_>>().join("#").chars().take(100).collect::<String>());
        let e = reasons.entry(key).or_insert((0, format!("{seed}")));
        e.0 += 1;
    };
    let mut answer_lines = answers.lines();
    for line in cases.lines() {
        let case = json::parse(line).unwrap();
        let Some(answer_line) = answer_lines.next() else { break };
        let gcs = json::parse(answer_line).unwrap();
        let category = case.get("category").and_then(Value::as_str).unwrap().to_string();
        let seed = case.get("seed").and_then(Value::as_f64).unwrap() as u64;
        if !rows.contains_key(&category) {
            order.push(category.clone());
        }
        let row = rows.entry(category.clone()).or_default();
        row.cases += 1;
        let start = Sketch::from_json(case.get("start").unwrap()).unwrap();
        let mine = case.get("mine").unwrap();
        let mine_ok = mine.get("ok").and_then(Value::as_bool).unwrap();
        let status = gcs.get("status").and_then(Value::as_f64).unwrap() as i64;
        let their_ok = status == 0 || status == -2;
        row.my_time += mine.get("micros").and_then(Value::as_f64).unwrap_or(0.0);
        row.their_time += gcs.get("micros").and_then(Value::as_f64).unwrap_or(0.0);
        let expect_solvable = !matches!(case.get("expect"), Some(Value::Str(s)) if s == "impossible") && case.get("expect").and_then(|e| e.get("conflict")).is_none();
        match (mine_ok, their_ok) {
            (true, true) => row.both += 1,
            (false, true) => {
                if expect_solvable {
                    row.my_refusals += 1;
                    note(&mut log, "I REFUSE, IT SOLVES", &category, seed, mine.get("message").and_then(Value::as_str).unwrap_or("").to_string());
                } else {
                    row.their_refusals += 1;
                    note(&mut log, "NO SOLUTION, IT CLAIMS ONE", &category, seed, format!("status {status}"));
                }
            }
            (true, false) => {
                if expect_solvable {
                    row.their_refusals += 1;
                    note(&mut log, "IT REFUSES, I SOLVE", &category, seed, format!("status {status}"));
                } else {
                    row.wrong += 1;
                    note(&mut log, "WRONG: I SOLVED THE UNSOLVABLE", &category, seed, String::new());
                }
            }
            (false, false) => {
                row.both_refuse += 1;
                if expect_solvable {
                    note(&mut log, "BOTH REFUSE A SOLVABLE SKETCH", &category, seed, mine.get("message").and_then(Value::as_str).unwrap_or("").to_string());
                }
            }
        }
        if !mine_ok {
            continue;
        }
        // Give my answer to PlaneGCS.
        if let Some(accept) = gcs.get("accept") {
            let s = accept.get("status").and_then(Value::as_f64).unwrap() as i64;
            let moved = accept.get("moved").and_then(Value::as_f64).unwrap_or(0.0);
            // A constraint PlaneGCS has no form of leaves it with that much more freedom.
            let unmapped = gcs.get("unmapped").and_then(Value::as_f64).unwrap_or(0.0);
            row.unmapped += (unmapped > 0.0) as usize;
            let dof = accept.get("dof").and_then(Value::as_f64).unwrap_or(-1.0) - unmapped;
            let my_dof = mine.get("dof").and_then(Value::as_f64).unwrap();
            let ids = |v: Option<&Value>| -> Vec<u32> {
                v.and_then(Value::as_array).unwrap_or(&[]).iter().filter_map(Value::as_f64).map(|n| n as u32).collect()
            };
            // Which of a redundant group gets the name is a choice, and a fix is two equations in
            // PlaneGCS, so the names can differ. Whether there is any redundancy can't.
            let (my_red, their_red) = (ids(mine.get("redundant")), ids(accept.get("redundant")));
            if my_red != their_red {
                row.named_differently += 1;
            }
            if my_red.is_empty() != their_red.is_empty() && ids(accept.get("partial")).is_empty() {
                row.red_differ += 1;
                note(&mut log, "ONLY ONE FOUND REDUNDANCY", &category, seed, format!("mine {my_red:?}, PlaneGCS {their_red:?}"));
            }
            let fine = (s == 0 || s == -2) && moved <= 1e-6;
            if fine {
                row.agree += 1;
            } else {
                row.wrong += 1;
                note(&mut log, "WRONG: PLANEGCS REJECTS MY ANSWER", &category, seed, format!("status {s}, moved {moved:.3e}"));
            }
            if dof >= 0.0 && dof != my_dof {
                row.dof_differ += 1;
                note(&mut log, "FREEDOM DIFFERS", &category, seed, format!("mine {my_dof}, PlaneGCS {dof}"));
            }
        }
        // Give PlaneGCS's answer to mine.
        if status == 0 {
            if let Some(list) = gcs.get("geometry").and_then(Value::as_array) {
                let theirs = without_unmapped(&with_geometry(&start, list).unwrap());
                let gone = theirs.collapsed_entities();
                let bad = violations(&theirs, 1e-6);
                if !gone.is_empty() {
                    row.their_collapses += 1;
                    note(&mut log, "PLANEGCS SAID SOLVED BY COLLAPSING GEOMETRY", &category, seed, format!("{} entities", gone.len()));
                } else if !bad.is_empty() {
                    row.their_wrong += 1;
                    note(&mut log, "PLANEGCS SAID SOLVED BUT FAILS THE CHECK", &category, seed, bad.join("; "));
                }
            }
        }
        // Dragging.
        if let (Some(d), Some(t)) = (case.get("drag"), gcs.get("drag")) {
            row.drags += 1;
            row.my_moved += d.get("moved").and_then(Value::as_f64).unwrap_or(0.0);
            let dragged = Sketch::from_json(d.get("sketch").unwrap()).unwrap();
            if violations(&dragged, CHECK_TOLERANCE).is_empty() {
                row.drag_ok += 1;
            } else {
                note(&mut log, "WRONG: MY DRAG BROKE A CONSTRAINT", &category, seed, violations(&dragged, CHECK_TOLERANCE).join("; "));
                row.wrong += 1;
            }
            let i_reached = d.get("reached").and_then(Value::as_bool).unwrap_or(false);
            if !i_reached {
                row.my_miss += 1;
            }
            if let Some(list) = t.get("geometry").and_then(Value::as_array) {
                let solved = without_unmapped(&Sketch::from_json(mine.get("solved").unwrap()).unwrap());
                let theirs = with_geometry(&solved, list).unwrap();
                let their_drag_fine = violations(&theirs, 1e-6).is_empty();
                if their_drag_fine {
                    row.drag_their_ok += 1;
                } else {
                    row.their_drag_broken += 1;
                    note(&mut log, "PLANEGCS'S DRAG BROKE A CONSTRAINT", &category, seed, violations(&theirs, 1e-6).join("; "));
                }
                let grab = Ref::parse(d.get("grab").and_then(Value::as_str).unwrap()).unwrap();
                row.their_moved += moved_others(&solved, &theirs, grab);
                let same_problem = gcs.get("unmapped").and_then(Value::as_f64).unwrap_or(0.0) == 0.0;
                let to = d.get("to").and_then(Value::as_array).unwrap();
                let at = t.get("at").and_then(Value::as_array).map(|a| [a[0].as_f64().unwrap(), a[1].as_f64().unwrap()]);
                if let Some(at) = at {
                    let miss = (at[0] - to[0].as_f64().unwrap()).hypot(at[1] - to[1].as_f64().unwrap());
                    let my_miss = d.get("miss").and_then(Value::as_f64).unwrap_or(0.0);
                    if miss > 1e-4 {
                        row.their_miss += 1;
                    } else if my_miss > 1e-3 && same_problem {
                        row.my_short += 1;
                        note(&mut log, "DRAG: IT REACHES, I STOP SHORT", &category, seed, format!("I miss by {my_miss:.3e}"));
                    }
                    // How far each disturbed the rest, where both got there and kept every constraint.
                    if miss <= 1e-4 && my_miss <= 1e-3 && their_drag_fine && same_problem {
                        row.fair_drags += 1;
                        row.my_fair_moved += d.get("moved").and_then(Value::as_f64).unwrap_or(0.0);
                        row.their_fair_moved += moved_others(&solved, &theirs, grab);
                    }
                }
            }
        }
    }
    println!("{:<13} {:>6} {:>6} {:>7} {:>5} {:>7} {:>7} {:>6} {:>6} {:>9} {:>9}", "category", "cases", "both", "agreed", "dof≠", "I-ref", "it-ref", "both-r", "wrong", "mine µs", "gcs µs");
    println!("(both: both solved. agreed: PlaneGCS accepted my answer unmoved. I-ref: I refused and it solved. it-ref: it refused and I solved. both-r: neither has an answer.)");
    for c in &order {
        let r = &rows[c];
        let n = r.cases.max(1) as f64;
        println!(
            "{:<13} {:>6} {:>6} {:>7} {:>5} {:>7} {:>7} {:>6} {:>6} {:>9.0} {:>9.0}",
            c, r.cases, r.both, r.agree, r.dof_differ, r.my_refusals, r.their_refusals, r.both_refuse, r.wrong, r.my_time / n, r.their_time / n
        );
    }
    println!();
    println!("{:<13} {:>6} {:>9} {:>9} {:>9} {:>9} {:>7} {:>10} {:>10}", "drag", "drags", "gcs broke", "my miss", "gcs miss", "I'm short", "both", "my moved", "gcs moved");
    println!("(both: drags both finished with every constraint kept. moved: how far everything else went, for those.)");
    for c in &order {
        let r = &rows[c];
        if r.drags > 0 {
            let n = r.fair_drags.max(1) as f64;
            println!(
                "{:<13} {:>6} {:>9} {:>9} {:>9} {:>9} {:>7} {:>10.2} {:>10.2}",
                c, r.drags, r.their_drag_broken, r.my_miss, r.their_miss, r.my_short, r.fair_drags, r.my_fair_moved / n, r.their_fair_moved / n
            );
        }
    }
    println!();
    println!("PlaneGCS said solved: {} by collapsing geometry, {} failing the independent check.", rows.values().map(|r| r.their_collapses).sum::<usize>(), rows.values().map(|r| r.their_wrong).sum::<usize>());
    println!("{} cases named different constraints as the redundant ones, which is a free choice.", rows.values().map(|r| r.named_differently).sum::<usize>());
    let (unmapped, red): (usize, usize) = rows.values().fold((0, 0), |a, r| (a.0 + r.unmapped, a.1 + r.red_differ));
    println!("{unmapped} cases had a constraint PlaneGCS has no form of (normal between two curves with no shared point); those were compared without it.");
    println!("{red} cases where only one of the two found any redundancy.");
    let mut list: Vec<_> = reasons.into_iter().collect();
    list.sort_by_key(|r| std::cmp::Reverse(r.1 .0));
    for (why, (n, seed)) in list {
        println!("{n:>6} x {why}   (first seed {seed})");
    }
    std::fs::write(format!("{dir}/disagreements.txt"), log).unwrap();
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("export") => export(&args[2], args[3].parse().unwrap(), args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0)),
        Some("compare") => compare(&args[2]),
        _ => eprintln!("usage: oracle export <dir> <cases> [first seed] | oracle compare <dir>"),
    }
}
