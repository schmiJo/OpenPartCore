//! The regions of a sketch, split at crossings and T-junctions. Besides the worked cases, random
//! sketches are checked against a raster: draw the curves on a fine grid, flood-fill from outside, and
//! every pocket left must be one region of the same size. That shares no code or method with the walk
//! around the arrangement, so a mistake in one doesn't hide in the other.

use coreshape_sketch::rng::Rng;
use coreshape_sketch::*;
use std::f64::consts::{PI, TAU};

fn rect(s: &mut Sketch, x: f64, y: f64, w: f64, h: f64) -> [EntityId; 4] {
    [
        s.line([x, y], [x + w, y]),
        s.line([x + w, y], [x + w, y + h]),
        s.line([x + w, y + h], [x, y + h]),
        s.line([x, y + h], [x, y]),
    ]
}

fn areas(a: &Arrangement) -> Vec<f64> {
    let mut v: Vec<f64> = a.regions.iter().map(Region::area).collect();
    v.sort_by(f64::total_cmp);
    v
}

fn close(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6)
}

#[test]
fn a_line_across_a_rectangle_makes_two_regions() {
    let mut s = Sketch::new();
    rect(&mut s, 0.0, 0.0, 30.0, 12.0);
    let spur = s.line([10.0, 0.0], [10.0, 12.0]);
    let a = s.arrangement();
    assert!(close(&areas(&a), &[120.0, 240.0]), "{:?}", areas(&a));
    assert!(a.ignored.is_empty(), "{:?}", a.ignored);
    assert_eq!(s.regions().unwrap().len(), 2);
    // As construction geometry, it's as if it weren't there.
    s.set_construction(spur, true).unwrap();
    let a = s.arrangement();
    assert!(close(&areas(&a), &[360.0]));
    assert!(a.ignored.is_empty());
}

#[test]
fn lines_that_cross_split_at_the_crossing() {
    // A bow tie.
    let mut t = Sketch::new();
    t.line([0.0, 0.0], [10.0, 10.0]);
    t.line([10.0, 10.0], [10.0, 0.0]);
    t.line([10.0, 0.0], [0.0, 10.0]);
    t.line([0.0, 10.0], [0.0, 0.0]);
    let a = t.arrangement();
    assert!(close(&areas(&a), &[25.0, 25.0]), "{:?}", areas(&a));
    // Noughts and crosses inside a square: nine cells.
    let mut g = Sketch::new();
    rect(&mut g, 0.0, 0.0, 9.0, 9.0);
    g.line([3.0, -1.0], [3.0, 10.0]);
    g.line([6.0, -1.0], [6.0, 10.0]);
    g.line([-1.0, 3.0], [10.0, 3.0]);
    g.line([-1.0, 6.0], [10.0, 6.0]);
    let a = g.arrangement();
    assert!(close(&areas(&a), &[9.0; 9]), "{:?}", areas(&a));
    // The four lines stick out at both ends, and only those stubs are left over.
    assert_eq!(a.ignored.len(), 8, "{:?}", a.ignored);
    assert!(a.ignored.iter().all(|i| !i.whole && matches!(i.why, WhyIgnored::FreeEnd { .. })));
}

#[test]
fn open_curves_and_spurs_are_reported_and_regions_still_come_back() {
    let mut s = Sketch::new();
    let edges = rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    let inside = s.line([5.0, 5.0], [5.0, 10.0]);
    let away = s.line([50.0, 0.0], [60.0, 0.0]);
    let chain = s.arc([0.0, 40.0], 3.0, 0.0, 90.0);
    let a = s.arrangement();
    assert!(close(&areas(&a), &[100.0]), "{:?}", areas(&a));
    let of = |e: EntityId| a.ignored.iter().find(|i| i.entity == e).unwrap_or_else(|| panic!("{e:?} in {:?}", a.ignored));
    assert!(of(inside).whole && matches!(of(inside).why, WhyIgnored::FreeEnd { at } if at == [5.0, 5.0]), "{:?}", of(inside));
    assert!(of(away).whole);
    assert!(of(chain).whole);
    assert!(a.ignored.iter().all(|i| !edges.contains(&i.entity)));
    // A spur doesn't rename the region it hangs off.
    let mut bare = Sketch::new();
    rect(&mut bare, 0.0, 0.0, 10.0, 10.0);
    assert_eq!(bare.arrangement().regions[0].id, a.regions[0].id);
}

#[test]
fn a_bridge_between_two_loops_encloses_nothing() {
    let mut s = Sketch::new();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    rect(&mut s, 20.0, 0.0, 10.0, 10.0);
    let bridge = s.line([10.0, 5.0], [20.0, 5.0]);
    let a = s.arrangement();
    assert!(close(&areas(&a), &[100.0, 100.0]), "{:?}", areas(&a));
    assert_eq!(a.ignored.len(), 1);
    assert_eq!(a.ignored[0].entity, bridge);
    assert!(a.ignored[0].whole && a.ignored[0].why == WhyIgnored::Bridge, "{:?}", a.ignored);
    // A spoke from a hole to the wall around it, too: the ring is still a ring.
    let mut r = Sketch::new();
    rect(&mut r, 0.0, 0.0, 10.0, 10.0);
    let hole = r.circle([5.0, 5.0], 2.0);
    let spoke = r.line([0.0, 5.0], [3.0, 5.0]);
    let a = r.arrangement();
    assert_eq!(a.ignored.len(), 1, "{:?}", a.ignored);
    assert_eq!(a.ignored[0].entity, spoke);
    assert!(close(&areas(&a), &[4.0 * PI, 100.0 - 4.0 * PI]), "{:?}", areas(&a));
    let ring = a.regions.iter().find(|x| x.holes.len() == 1).unwrap();
    assert!(ring.holes[0].segs.iter().all(|g| g.entity == hole));
}

#[test]
fn overlapping_circles_make_three_regions_with_three_ids() {
    let mut s = Sketch::new();
    s.circle([0.0, 0.0], 5.0);
    s.circle([6.0, 0.0], 5.0);
    let a = s.arrangement();
    assert_eq!(a.regions.len(), 3);
    let lens = 2.0 * 25.0 * (3.0f64 / 5.0).acos() - 3.0 * 8.0;
    let side = 25.0 * PI - lens;
    assert!(close(&areas(&a), &[lens, side, side]), "{:?} {lens} {side}", areas(&a));
    let mut ids: Vec<RegionId> = a.regions.iter().map(|r| r.id).collect();
    ids.sort();
    ids.dedup();
    assert_eq!(ids.len(), 3);
    // By default they're all material, since they're one connected group.
    assert!(a.regions.iter().all(|r| r.depth == 0));
    assert_eq!(s.regions().unwrap().len(), 3);
    // Picking by clicking: the middle is the lens.
    let picked = a.region_at([3.0, 0.0]).unwrap();
    assert!((picked.area() - lens).abs() < 1e-6);
    assert!(a.region_at([100.0, 0.0]).is_none());
}

#[test]
fn circles_touching_at_a_point_are_two_regions() {
    let mut u = Sketch::new();
    u.circle([0.0, 0.0], 5.0);
    u.circle([10.0, 0.0], 5.0);
    let a = u.arrangement();
    assert!(close(&areas(&a), &[25.0 * PI, 25.0 * PI]), "{:?}", areas(&a));
    // And a circle inside a circle, touching it: a disc and a crescent.
    let mut v = Sketch::new();
    v.circle([0.0, 0.0], 5.0);
    v.circle([3.0, 0.0], 2.0);
    let a = v.arrangement();
    assert!(close(&areas(&a), &[4.0 * PI, 21.0 * PI]), "{:?}", areas(&a));
    // A line tangent to a circle cuts nothing off.
    let mut w = Sketch::new();
    w.circle([0.0, 0.0], 5.0);
    let tangent = w.line([-9.0, 5.0], [9.0, 5.0]);
    let a = w.arrangement();
    assert!(close(&areas(&a), &[25.0 * PI]), "{:?}", areas(&a));
    assert!(a.ignored.iter().all(|i| i.entity == tangent && i.whole), "{:?}", a.ignored);
}

#[test]
fn a_t_junction_on_a_circle_splits_it() {
    let mut s = Sketch::new();
    s.circle([0.0, 0.0], 5.0);
    s.line([-5.0, 0.0], [5.0, 0.0]);
    let a = s.arrangement();
    assert!(close(&areas(&a), &[12.5 * PI, 12.5 * PI]), "{:?}", areas(&a));
    assert!(a.ignored.is_empty());
}

#[test]
fn lines_drawn_twice_count_once() {
    let mut s = Sketch::new();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    let copy = s.line([10.0, 0.0], [0.0, 0.0]);
    // Overlapping part of a side, and sticking out past it.
    let partial = s.line([2.0, 10.0], [14.0, 10.0]);
    let a = s.arrangement();
    assert!(close(&areas(&a), &[100.0]), "{:?}", areas(&a));
    let of = |e: EntityId| a.ignored.iter().filter(|i| i.entity == e).collect::<Vec<_>>();
    assert!(matches!(of(copy)[0].why, WhyIgnored::Overlap { .. }) && of(copy)[0].whole, "{:?}", of(copy));
    assert_eq!(of(partial).len(), 2, "{:?}", of(partial));
}

#[test]
fn nesting_still_means_holes_by_default() {
    let mut s = Sketch::new();
    rect(&mut s, 0.0, 0.0, 30.0, 30.0);
    let hole = s.circle([15.0, 15.0], 10.0);
    let island = s.circle([15.0, 15.0], 4.0);
    let a = s.arrangement();
    // Everything enclosed is a region: the plate, the ring in the hole, and the island.
    assert_eq!(a.regions.len(), 3);
    let depths = |d: u32| a.regions.iter().filter(|r| r.depth == d).count();
    assert_eq!((depths(0), depths(1), depths(2)), (1, 1, 1));
    let used = s.regions().unwrap();
    assert_eq!(used.len(), 2);
    let plate = used.iter().find(|r| r.depth == 0).unwrap();
    assert_eq!(plate.holes.len(), 1);
    assert_eq!(plate.holes[0].segs[0].entity, hole);
    assert!(plate.holes[0].signed_area() < 0.0, "holes run clockwise");
    let inner = used.iter().find(|r| r.depth == 2).unwrap();
    assert_eq!(inner.outer.segs[0].entity, island);
}

#[test]
fn picked_regions_are_used_saved_and_refused_when_they_go() {
    let mut s = Sketch::new();
    let edges = rect(&mut s, 0.0, 0.0, 30.0, 12.0);
    let divider = s.line([10.0, 0.0], [10.0, 12.0]);
    let a = s.arrangement();
    let left = a.region_at([5.0, 6.0]).unwrap().id;
    s.pick_regions(&[left]);
    let used = s.regions().unwrap();
    assert_eq!(used.len(), 1);
    assert!((used[0].area() - 120.0).abs() < 1e-9);

    let text = s.save_text();
    assert!(text.contains(&left.name()));
    let back = Sketch::load_text(&text).unwrap();
    assert_eq!(back.picked_regions(), &[left]);
    assert_eq!(back, s);

    // Files with nothing picked don't mention it, so the format is unchanged for them.
    let mut plain = s.clone();
    plain.pick_regions(&[]);
    assert!(!plain.save_text().contains("picked_regions"));

    // The picked area goes away with the line that made it: refused, not silently something else.
    s.remove_entity(divider).unwrap();
    let e = s.regions().unwrap_err();
    assert_eq!(e.kind, ErrorKind::Regions);
    assert!(e.message.contains(&left.name()), "{}", e.message);
    let _ = edges;
}

#[test]
fn ids_survive_solving_dragging_and_edits_elsewhere() {
    let mut s = Sketch::new();
    let e = rect(&mut s, 0.0, 0.0, 30.0, 12.0);
    let divider = s.line([10.0, 0.0], [10.0, 12.0]);
    for i in 0..4 {
        s.constrain(Rule::Coincident { a: e[i].end(), b: e[(i + 1) % 4].start() });
    }
    s.constrain(Rule::Horizontal { a: e[0].whole(), b: None });
    s.constrain(Rule::Vertical { a: e[1].whole(), b: None });
    s.constrain(Rule::Horizontal { a: e[2].whole(), b: None });
    s.constrain(Rule::Vertical { a: e[3].whole(), b: None });
    s.constrain(Rule::Coincident { a: e[0].start(), b: ORIGIN.whole() });
    s.constrain(Rule::Vertical { a: divider.whole(), b: None });
    s.constrain(Rule::Coincident { a: divider.start(), b: e[0].whole() });
    s.constrain(Rule::Coincident { a: divider.end(), b: e[2].whole() });
    let width = s.constrain(Rule::Distance { a: e[0].whole(), b: None, value: 30.0.into() });
    let ids = |k: &Sketch| {
        let mut v: Vec<RegionId> = k.arrangement().regions.iter().map(|r| r.id).collect();
        v.sort();
        v
    };
    let solved = s.solve().unwrap().sketch;
    let before = ids(&solved);
    assert_eq!(before.len(), 2);

    // A new dimension moves everything and renames nothing.
    let mut wide = s.clone();
    wide.set_value(width, 80.0).unwrap();
    assert_eq!(ids(&wide.solve().unwrap().sketch), before);

    // Nor does a drag.
    let dragged = solved.drag(Grab::Point(divider.start()), [20.0, 0.0]).unwrap();
    assert_eq!(ids(&dragged.0), before);

    // A hole in one region, and something new elsewhere, leave both names alone.
    let mut more = solved.clone();
    more.circle([20.0, 6.0], 2.0);
    more.circle([100.0, 100.0], 3.0);
    let after = ids(&more);
    assert_eq!(after.len(), 4);
    assert!(before.iter().all(|id| after.contains(id)), "{before:?} {after:?}");

    // Redrawing in a different order keeps them too: an id follows entity ids, not position in the list.
    let mut reversed = solved.clone();
    reversed.entities.reverse();
    assert_eq!(ids(&reversed), before);
}

#[test]
fn an_end_short_of_a_curve_is_a_gap_and_within_tolerance_it_joins() {
    let build = |gap: f64| {
        let mut s = Sketch::new();
        rect(&mut s, 0.0, 0.0, 10.0, 10.0);
        s.line([5.0, 0.0], [5.0, 10.0 - gap]);
        s.arrangement().regions.len()
    };
    assert_eq!(build(0.0), 2);
    assert_eq!(build(1e-9), 2, "within tolerance it lands on the curve");
    assert_eq!(build(1e-3), 1, "a real gap leaves the spur a spur");
    assert_eq!(build(-1e-9), 2);
    assert_eq!(build(-1e-3), 2, "past the curve it's a crossing, with a small stub left over");
}

#[test]
fn an_empty_or_degenerate_sketch_is_not_a_panic() {
    assert!(Sketch::new().arrangement().regions.is_empty());
    let mut s = Sketch::new();
    let point_line = s.line([1.0, 1.0], [1.0, 1.0]);
    let no_radius = s.circle([0.0, 0.0], 0.0);
    s.line([f64::NAN, 0.0], [1.0, 1.0]);
    s.circle([f64::INFINITY, 0.0], 3.0);
    s.arc([0.0, 0.0], f64::NAN, 0.0, 90.0);
    let a = s.arrangement();
    assert!(a.regions.is_empty());
    assert!(a.ignored.iter().any(|i| i.entity == point_line && i.why == WhyIgnored::NoLength));
    assert!(a.ignored.iter().any(|i| i.entity == no_radius && i.why == WhyIgnored::NoLength));
    assert_eq!(a.ignored.len(), 5, "{:?}", a.ignored);
    assert!(s.regions().is_err());
}

#[test]
fn the_arrangement_as_json_names_regions_and_what_it_left_out() {
    let mut s = Sketch::new();
    rect(&mut s, 0.0, 0.0, 10.0, 10.0);
    let spur = s.line([5.0, 5.0], [5.0, 10.0]);
    let text = s.arrangement().to_json().to_text();
    let v = json::parse(&text).unwrap();
    let regions = v.get("regions").and_then(json::Value::as_array).unwrap();
    assert_eq!(regions.len(), 1);
    let id = regions[0].get("id").and_then(json::Value::as_str).unwrap();
    assert_eq!(RegionId::parse(id).unwrap(), s.arrangement().regions[0].id);
    assert_eq!(regions[0].get("depth").and_then(json::Value::as_f64), Some(0.0));
    assert_eq!(regions[0].get("area").and_then(json::Value::as_f64), Some(100.0));
    let ignored = v.get("ignored").and_then(json::Value::as_array).unwrap();
    assert_eq!(ignored.len(), 1);
    assert_eq!(ignored[0].get("entity").and_then(json::Value::as_f64), Some(spur.0 as f64));
    assert_eq!(ignored[0].get("why").and_then(json::Value::as_str), Some("free_end"));
    assert!(RegionId::parse("r12").is_err() && RegionId::parse("x0123456789ab").is_err());
}

#[test]
fn hundreds_of_curves_stay_quick() {
    // A grid of 200 lines over a square: 10,000 cells.
    let mut s = Sketch::new();
    for i in 0..=100 {
        let t = i as f64;
        s.line([t, 0.0], [t, 100.0]);
        s.line([0.0, t], [100.0, t]);
    }
    let start = std::time::Instant::now();
    let a = s.arrangement();
    let took = start.elapsed();
    assert_eq!(a.regions.len(), 10_000);
    println!("10,000 regions from 202 lines in {took:?}");
    assert!(took.as_secs_f64() < 60.0, "{took:?}");
}

// ----- Against a raster -----

struct Grid {
    n: usize,
    lo: [f64; 2],
    cell: f64,
    wall: Vec<bool>,
}

/// A pocket: its size in cells, a point in it, and its bounds.
struct Pocket {
    cells: usize,
    at: [f64; 2],
    min: [f64; 2],
    max: [f64; 2],
}

impl Grid {
    fn new(n: usize, lo: [f64; 2], size: f64) -> Grid {
        Grid { n, lo, cell: size / n as f64, wall: vec![false; n * n] }
    }

    fn mark(&mut self, p: [f64; 2]) {
        let (x, y) = (((p[0] - self.lo[0]) / self.cell).floor(), ((p[1] - self.lo[1]) / self.cell).floor());
        if x >= 0.0 && y >= 0.0 && (x as usize) < self.n && (y as usize) < self.n {
            let i = y as usize * self.n + x as usize;
            self.wall[i] = true;
        }
    }

    /// Marks a curve as a wall: fine steps, so that every cell it passes through is marked and a move
    /// across it is impossible.
    fn draw(&mut self, at: impl Fn(f64) -> [f64; 2], length: f64) {
        let steps = ((length / self.cell) * 4.0).ceil().max(2.0) as usize;
        for i in 0..=steps {
            self.mark(at(i as f64 / steps as f64));
        }
    }

    fn draw_sketch(&mut self, s: &Sketch) {
        for e in &s.entities {
            match e.geometry {
                Geometry::Line { start, end } => {
                    let len = (end[0] - start[0]).hypot(end[1] - start[1]);
                    self.draw(|t| [start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t], len);
                }
                Geometry::Circle { center, radius } => {
                    self.draw(|t| [center[0] + radius * (t * TAU).cos(), center[1] + radius * (t * TAU).sin()], TAU * radius);
                }
                Geometry::Arc { center, radius, start, end } => {
                    let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
                    let mut a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
                    while a1 <= a0 {
                        a1 += TAU;
                    }
                    self.draw(|t| { let u = a0 + (a1 - a0) * t; [center[0] + radius * u.cos(), center[1] + radius * u.sin()] }, (a1 - a0) * radius);
                }
                Geometry::Point { .. } => {}
            }
        }
    }

    /// What a flood from the grid's border can't reach.
    fn pockets(&self) -> Vec<Pocket> {
        let n = self.n;
        let mut seen = self.wall.clone();
        let mut stack: Vec<usize> = Vec::new();
        for k in 0..n {
            stack.extend([k, n * n - 1 - k, k * n, k * n + n - 1]);
        }
        while let Some(i) = stack.pop() {
            if seen[i] {
                continue;
            }
            seen[i] = true;
            let (x, y) = (i % n, i / n);
            if x > 0 {
                stack.push(i - 1);
            }
            if x + 1 < n {
                stack.push(i + 1);
            }
            if y > 0 {
                stack.push(i - n);
            }
            if y + 1 < n {
                stack.push(i + n);
            }
        }
        let mut out = Vec::new();
        for start in 0..n * n {
            if seen[start] {
                continue;
            }
            let mut cells = 0;
            let (mut lo, mut hi) = ([n, n], [0, 0]);
            seen[start] = true;
            stack.push(start);
            while let Some(i) = stack.pop() {
                cells += 1;
                let (x, y) = (i % n, i / n);
                lo = [lo[0].min(x), lo[1].min(y)];
                hi = [hi[0].max(x), hi[1].max(y)];
                for (ok, j) in [(x > 0, i.wrapping_sub(1)), (x + 1 < n, i + 1), (y > 0, i.wrapping_sub(n)), (y + 1 < n, i + n)] {
                    if ok && !seen[j] {
                        seen[j] = true;
                        stack.push(j);
                    }
                }
            }
            let place = |c: usize, axis: usize| self.lo[axis] + c as f64 * self.cell;
            out.push(Pocket {
                cells,
                at: [place(start % n, 0) + self.cell / 2.0, place(start / n, 1) + self.cell / 2.0],
                min: [place(lo[0], 0), place(lo[1], 1)],
                max: [place(hi[0] + 1, 0), place(hi[1] + 1, 1)],
            });
        }
        out
    }
}

/// Walls as wide as a cell close any gap narrower than a cell, which makes a pocket the curves don't
/// really enclose. So look again with a much finer grid around it: if a gap appears, the pocket spills
/// out over its old bounds and wasn't real.
fn really_a_pocket(s: &Sketch, p: &Pocket, cell: f64) -> bool {
    let size = (p.max[0] - p.min[0]).max(p.max[1] - p.min[1]) + 4.0 * cell;
    let n = ((size / (cell / 40.0)).ceil() as usize).clamp(200, 4000);
    let mut fine = Grid::new(n, [p.min[0] - 2.0 * cell, p.min[1] - 2.0 * cell], size);
    fine.draw_sketch(s);
    // Is the place where the pocket was in any pocket now?
    let (x, y) = ((p.at[0] - fine.lo[0]) / fine.cell, (p.at[1] - fine.lo[1]) / fine.cell);
    fine.pockets().iter().any(|q| {
        let (qx, qy) = ((q.min[0] - fine.lo[0]) / fine.cell, (q.min[1] - fine.lo[1]) / fine.cell);
        let (hx, hy) = ((q.max[0] - fine.lo[0]) / fine.cell, (q.max[1] - fine.lo[1]) / fine.cell);
        x >= qx && x <= hx && y >= qy && y <= hy && q.cells as f64 * fine.cell * fine.cell > 0.2 * p.cells as f64 * cell * cell
    })
}

/// Whether a curve's end stops just short of another curve. A raster can't tell that from touching, so
/// these are left to the worked cases.
fn has_near_miss(s: &Sketch) -> bool {
    let gap = |g: &Geometry, p: [f64; 2]| match *g {
        Geometry::Line { start: a, end: b } => {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let f = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            (a[0] + f * dx - p[0]).hypot(a[1] + f * dy - p[1])
        }
        Geometry::Circle { center, radius } | Geometry::Arc { center, radius, .. } => ((p[0] - center[0]).hypot(p[1] - center[1]) - radius).abs(),
        Geometry::Point { .. } => f64::INFINITY,
    };
    s.entities.iter().any(|e| {
        let ends: &[[f64; 2]] = match &e.geometry {
            Geometry::Line { start, end } | Geometry::Arc { start, end, .. } => &[*start, *end],
            _ => &[],
        };
        ends.iter().any(|&p| s.entities.iter().any(|o| o.id != e.id && (1e-6..0.4).contains(&gap(&o.geometry, p))))
    })
}

/// A random sketch of lines, arcs and circles, some of them ending exactly on others.
fn random_sketch(rng: &mut Rng) -> Sketch {
    let mut s = Sketch::new();
    let count = rng.int(3, 14);
    let span = 40.0;
    let mut spots: Vec<[f64; 2]> = Vec::new();
    for _ in 0..count {
        let pick = |rng: &mut Rng, spots: &Vec<[f64; 2]>| -> [f64; 2] {
            if !spots.is_empty() && rng.range(0.0, 1.0) < 0.5 {
                spots[rng.int(0, spots.len() as i64 - 1) as usize]
            } else {
                [rng.range(-span, span), rng.range(-span, span)]
            }
        };
        match rng.int(0, 5) {
            0..=2 => {
                let (a, b) = (pick(rng, &spots), pick(rng, &spots));
                if (a[0] - b[0]).hypot(a[1] - b[1]) > 1.0 {
                    s.line(a, b);
                    spots.push(a);
                    spots.push(b);
                    let f = rng.range(0.2, 0.8);
                    spots.push([a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]);
                }
            }
            3 => {
                let c = pick(rng, &spots);
                let r = rng.range(3.0, 25.0);
                s.circle(c, r);
                let t = rng.range(0.0, 6.28);
                spots.push([c[0] + r * t.cos(), c[1] + r * t.sin()]);
            }
            _ => {
                let c = [rng.range(-span, span), rng.range(-span, span)];
                let r = rng.range(3.0, 25.0);
                let (d0, d1) = (rng.range(0.0, 360.0), rng.range(0.0, 360.0));
                if (d0 - d1).abs() > 10.0 {
                    let id = s.arc(c, r, d0, d1);
                    if let Some(Entity { geometry: Geometry::Arc { start, end, .. }, .. }) = s.entity(id) {
                        spots.push(*start);
                        spots.push(*end);
                    }
                }
            }
        }
    }
    s
}

#[test]
fn every_region_matches_a_pocket_of_a_raster() {
    let cell: f64 = 0.1;
    let cell_area = cell * cell;
    let mut rng = Rng::new(0x5EC7);
    let (mut checked, mut pockets_seen, mut regions_seen, mut false_pockets) = (0, 0, 0, 0);
    let cases = if cfg!(debug_assertions) { 120 } else { 2500 };
    for case in 0..cases {
        let s = random_sketch(&mut rng);
        if has_near_miss(&s) {
            continue;
        }
        let a = s.arrangement();
        // The grid has to reach past everything, or the flood would start inside a curve.
        let reach = s.entities.iter().fold(1.0f64, |far, e| {
            far.max(match e.geometry {
                Geometry::Point { at } => at[0].abs().max(at[1].abs()),
                Geometry::Line { start, end } => start[0].abs().max(start[1].abs()).max(end[0].abs()).max(end[1].abs()),
                Geometry::Circle { center, radius } | Geometry::Arc { center, radius, .. } => center[0].abs().max(center[1].abs()) + radius,
            })
        }) + 2.0;
        let n = (2.0 * reach / cell).ceil() as usize;
        let mut grid = Grid::new(n, [-reach, -reach], n as f64 * cell);
        grid.draw_sketch(&s);
        let pockets = grid.pockets();
        // Walls a cell wide can pinch a sharp corner off into a pocket of its own, so a region can hold
        // several pockets. Every pocket must lie in a region, and together they must fill it.
        let mut filled = vec![0.0; a.regions.len()];
        for p in &pockets {
            let area = p.cells as f64 * cell_area;
            let Some(k) = a.regions.iter().position(|r| r.contains(p.at)) else {
                // The centre of a tiny pocket can lie just outside the true curve's region.
                if area >= 0.05 {
                    assert!(!really_a_pocket(&s, p, cell), "case {case}: a pocket of area {area} at {:?} isn't in any region\n{}", p.at, s.save_text());
                    false_pockets += 1;
                }
                continue;
            };
            filled[k] += area;
            pockets_seen += 1;
        }
        for (r, &f) in a.regions.iter().zip(&filled) {
            // The walls eat about a cell's width all round, so the gap scales with the outline's length.
            let outline: f64 = std::iter::once(&r.outer).chain(&r.holes).map(|l| l.segs.iter().map(seg_length).sum::<f64>()).sum();
            let slack = outline * 2.5 * cell + 0.05;
            assert!(
                r.area() + 1e-9 >= f && r.area() <= f + slack,
                "case {case}: region {} of area {} holds pockets of area {f} (slack {slack})\n{}",
                r.id.name(),
                r.area(),
                s.save_text()
            );
            regions_seen += 1;
        }
        // Regions don't overlap and don't double-count: ids are unique and areas are positive.
        let mut ids: Vec<RegionId> = a.regions.iter().map(|r| r.id).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), a.regions.len(), "case {case}: ids aren't unique\n{}", s.save_text());
        assert!(a.regions.iter().all(|r| r.area() > 0.0), "case {case}");
        // The same sketch gives the same answer.
        assert_eq!(s.arrangement(), a, "case {case}: not deterministic");
        checked += 1;
    }
    println!("{checked} sketches, {pockets_seen} pockets each in a region, {regions_seen} regions each filled by their pockets, {false_pockets} pockets that a finer grid showed were gaps too narrow to see");
}

fn seg_length(s: &Seg) -> f64 {
    match s.kind {
        SegKind::Line { a, b } => (a[0] - b[0]).hypot(a[1] - b[1]),
        SegKind::Arc { radius, a0, a1, .. } => radius * (a1 - a0).abs(),
    }
}
