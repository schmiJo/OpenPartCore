//! Sketches of a chosen size for measuring speed, and the measurement itself, shared by the native
//! and WebAssembly benchmarks.

use crate::drag::{DragSession, Grab};
use crate::fuzz::{perturb, Category};
use crate::model::*;
use crate::rng::Rng;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Workload {
    /// A strip of rectangles, each sharing a corner with the next: long and thin.
    Rectangles,
    /// Every line parallel to one line and as long as it: one line used by every constraint.
    SharedLine,
    /// Lines and arcs tangent end to end: a long chain of curved constraints.
    TangentChain,
    /// Slots, polygons, circles and rounded outlines, joined by dimensions.
    Blocks,
}

pub const ALL_WORKLOADS: [Workload; 4] = [Workload::Rectangles, Workload::SharedLine, Workload::TangentChain, Workload::Blocks];

impl Workload {
    pub fn name(self) -> &'static str {
        match self {
            Workload::Rectangles => "rectangles",
            Workload::SharedLine => "shared line",
            Workload::TangentChain => "tangent chain",
            Workload::Blocks => "mixed blocks",
        }
    }
}

/// A sketch of about `entities` entities, and what to grab in it.
pub fn build(w: Workload, entities: usize) -> (Sketch, Ref) {
    let mut s = Sketch::new();
    match w {
        Workload::Rectangles => {
            let count = entities / 4;
            let mut previous: Option<EntityId> = None;
            let mut last = None;
            for i in 0..count {
                let (x, y) = (i as f64 * 12.0, i as f64 * 6.0);
                let (w, h) = (12.0, 6.0);
                let a = s.line([x, y], [x + w, y]);
                let b = s.line([x + w, y], [x + w, y + h]);
                let c = s.line([x + w, y + h], [x, y + h]);
                let d = s.line([x, y + h], [x, y]);
                for (p, q) in [(a, b), (b, c), (c, d), (d, a)] {
                    s.constrain(Rule::Coincident { a: p.end(), b: q.start() });
                }
                s.constrain(Rule::Horizontal { a: a.whole(), b: None });
                s.constrain(Rule::Vertical { a: b.whole(), b: None });
                s.constrain(Rule::Horizontal { a: c.whole(), b: None });
                s.constrain(Rule::Vertical { a: d.whole(), b: None });
                if i % 2 == 0 {
                    s.constrain(Rule::Distance { a: a.whole(), b: None, value: w.into() });
                }
                match previous {
                    // Each rectangle's first corner sits on the far corner of the last.
                    Some(p) => {
                        s.constrain(Rule::Coincident { a: a.start(), b: p.end() });
                    }
                    None => {
                        s.fix(a.start()).unwrap();
                    }
                }
                previous = Some(b);
                last = Some(b);
            }
            (s, last.unwrap().end())
        }
        Workload::SharedLine => {
            let shared = s.line([0.0, 0.0], [20.0, 0.0]);
            s.fix(shared.start()).unwrap();
            for i in 1..entities {
                let y = i as f64 * 3.0;
                let l = s.line([0.0, y], [20.0, y]);
                s.constrain(Rule::Parallel { a: shared, b: l });
                s.constrain(Rule::Equal { a: shared, b: l });
            }
            (s, shared.end())
        }
        Workload::TangentChain => {
            // Steps that go right, turn up through a quarter circle, go up, and turn back right.
            let mut x = 0.0;
            let mut y = 0.0;
            let r = 3.0;
            let mut last_end: Option<Ref> = None;
            let mut previous_line: Option<EntityId> = None;
            let mut count = 0;
            let mut up = false;
            let mut last = None;
            while count + 2 <= entities {
                let (line, arc);
                if !up {
                    line = s.line([x, y], [x + 10.0, y]);
                    arc = s.arc([x + 10.0, y + r], r, -90.0, 0.0);
                    x += 10.0 + r;
                    y += r;
                } else {
                    line = s.line([x, y], [x, y + 10.0]);
                    // Clockwise in travel, so the arc's own start is where the travel leaves it.
                    arc = s.arc([x + r, y + 10.0], r, 90.0, 180.0);
                    x += r;
                    y += 10.0 + r;
                }
                match (last_end, previous_line) {
                    (Some(e), _) => {
                        s.constrain(Rule::Coincident { a: line.start(), b: e });
                    }
                    (None, _) => {
                        s.fix(line.start()).unwrap();
                    }
                }
                let joint = if up { arc.end() } else { arc.start() };
                s.constrain(Rule::Coincident { a: line.end(), b: joint });
                s.constrain(Rule::Tangent { a: line, b: arc });
                s.constrain(Rule::Radius { of: arc, value: r.into() });
                if let Some(p) = previous_line {
                    s.constrain(Rule::Parallel { a: p, b: p });
                    let _ = p;
                }
                last_end = Some(if up { arc.start() } else { arc.end() });
                previous_line = Some(line);
                last = last_end;
                up = !up;
                count += 2;
            }
            // Drop the placeholder rules: a line parallel to itself says nothing.
            s.constraints.retain(|c| !matches!(c.rule, Rule::Parallel { a, b } if a == b));
            (s, last.unwrap())
        }
        Workload::Blocks => {
            let case = crate::fuzz::generate_sized(Category::Large, 7, entities);
            let solved = case.truth;
            let grab = solved
                .entities
                .iter()
                .rev()
                .find_map(|e| matches!(e.geometry, Geometry::Line { .. }).then(|| e.id.end()))
                .unwrap_or(ORIGIN.whole());
            (solved, grab)
        }
    }
}

/// Times for one sketch, in milliseconds.
#[derive(Clone, Debug)]
pub struct Timing {
    pub workload: &'static str,
    pub entities: usize,
    pub constraints: usize,
    pub dof: usize,
    /// Solving from a start nudged by 0.1 mm, with the analysis.
    pub solve: f64,
    /// The analysis alone, at the solved position.
    pub analyze: f64,
    /// Starting a drag.
    pub begin: f64,
    /// One pointer move: the median, the 95th percentile and the slowest of a scripted path.
    pub move_median: f64,
    pub move_p95: f64,
    pub move_worst: f64,
    /// Entities that moved at all in the last frame, as a sanity check that the drag did something.
    pub moved: bool,
}

/// Measures one workload. `clock` returns milliseconds.
pub fn measure(w: Workload, entities: usize, clock: &dyn Fn() -> f64) -> Timing {
    let (sketch, grab) = build(w, entities);
    let mut rng = Rng::new(11);
    let nudged = perturb(&sketch, &mut rng, 0.1);
    let t = clock();
    let solved = nudged.solve();
    let solve = clock() - t;
    let solved = solved.expect("a workload always solves");
    let t = clock();
    let analysis = solved.sketch.analyze().unwrap();
    let analyze = clock() - t;
    // Something that can actually move.
    let grab = if analysis.free.contains(&grab) {
        grab
    } else {
        analysis.free.iter().rev().copied().find(|r| r.part != Part::Radius).unwrap_or(grab)
    };
    let t = clock();
    let mut session = DragSession::begin(&solved.sketch, Grab::Point(grab)).expect("a workload can be dragged");
    let begin = clock() - t;
    let start = session.pointer();
    let mut times = Vec::with_capacity(240);
    let mut moved = false;
    for frame in 0..240 {
        let a = frame as f64 / 240.0 * std::f64::consts::TAU;
        let to = [start[0] + 4.0 * (a.cos() - 1.0), start[1] + 4.0 * a.sin()];
        let t = clock();
        let report = session.move_to(to);
        times.push(clock() - t);
        moved |= report.iterations > 0;
    }
    times.sort_by(f64::total_cmp);
    Timing {
        workload: w.name(),
        entities: sketch.entities.len(),
        constraints: sketch.constraints.len(),
        dof: analysis.dof,
        solve,
        analyze,
        begin,
        move_median: times[times.len() / 2],
        move_p95: times[times.len() * 95 / 100],
        move_worst: times[times.len() - 1],
        moved,
    }
}
