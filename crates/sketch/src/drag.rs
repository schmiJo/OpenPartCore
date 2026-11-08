//! Dragging: the grabbed thing follows the pointer, everything else stays as near as it can to
//! where it was when the drag began.
//!
//! Each step solves a small quadratic problem: stay close to the starting position, come close
//! to the pointer, and keep every constraint. The constraints are hard and the pointer is soft,
//! so if the pointer goes somewhere the grabbed point can't follow, the point stops at the
//! nearest place it can reach and the sketch stays valid. Measuring disturbance from where the
//! drag began, not from the previous frame, means nothing creeps: bring the pointer back and the
//! sketch comes back.

use crate::expr::Env;
use crate::model::*;
use crate::solve::*;
use crate::system::*;

/// What the user took hold of.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Grab {
    /// A point, or an end or center of an entity.
    Point(Ref),
    /// An edge, at the place it was grabbed. A line moves with the pointer, keeping its direction
    /// and length if the constraints allow. A circle or arc changes radius to reach the pointer.
    Edge { entity: EntityId, at: P2 },
}

/// How one pointer move went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DragReport {
    /// How far the grabbed thing ended up from the pointer.
    pub distance: f64,
    /// Whether it reached the pointer, to within a millionth of the sketch's size.
    pub reached: bool,
    pub iterations: usize,
}

enum Target {
    /// Variable to a fixed value.
    Value(f64),
    /// A radius variable to the distance from its center (x, y variables) to the pointer.
    RadiusTo { cx: u32, cy: u32 },
}

/// A drag in progress. The sketch it began from is untouched, and `sketch` gives the current state
/// at any point, always with its constraints met.
pub struct DragSession {
    base: Sketch,
    sys: System,
    compiled: Compiled,
    x: Vec<f64>,
    anchor: Vec<f64>,
    soft_vars: Vec<u32>,
    targets: Vec<Target>,
    goals: Vec<f64>,
    grab: Grab,
    pointer: P2,
    trial: Vec<f64>,
    tol: f64,
}

/// How much the pointer gives way to the constraints and to staying put. Small, so that a grabbed
/// point that can reach the pointer lands within about 1e-9 of the distance others had to move.
const SOFTNESS: f64 = 1e-9;
/// Damping on the constraints during a drag step, kept below the softness so that they win.
const HARD_DAMPING: f64 = 1e-12;
/// How closely the constraints are met when a move ends.
const POLISH: f64 = 1e-12;
const OFF: f64 = 1e12;
const MOST_STEPS: usize = 24;

impl DragSession {
    pub fn begin(sketch: &Sketch, grab: Grab) -> Result<DragSession, SketchError> {
        DragSession::begin_with(sketch, grab, &Env::new())
    }

    /// Starts a drag, solving the sketch first if it needs it.
    pub fn begin_with(sketch: &Sketch, grab: Grab, env: &Env) -> Result<DragSession, SketchError> {
        let solved = sketch.solve_with(env)?;
        let base = solved.sketch;
        let sys = System::build(&base, env)?;
        let bad = |message: String| SketchError { kind: ErrorKind::Invalid, message, constraints: Vec::new(), entities: Vec::new() };
        let base_var = |id: EntityId| sys.base.get(&id).copied().ok_or_else(|| bad(format!("there is no entity {}", id.whole().name())));
        let mut soft_vars: Vec<u32> = Vec::new();
        let mut targets: Vec<Target> = Vec::new();
        let x = sys.x0.clone();
        let point_vars = |r: Ref| -> Result<[u32; 2], SketchError> {
            let b = base_var(r.entity)?;
            let e = base.entity(r.entity).ok_or_else(|| bad(format!("{} can't be moved", r.name())))?;
            let offset = match (e.geometry.clone(), r.part) {
                (Geometry::Spline(_), Part::Start) => 0,
                (Geometry::Spline(shape), Part::End) => 2 * (shape.points.len() as u32 - 1),
                (Geometry::Spline(shape), Part::Control(i)) if (i as usize) < shape.points.len() => 2 * i,
                (Geometry::Point { .. }, Part::Whole) => 0,
                (Geometry::Line { .. }, Part::Start) => 0,
                (Geometry::Line { .. }, Part::End) => 2,
                (Geometry::Circle { .. } | Geometry::Arc { .. }, Part::Center) => 0,
                (Geometry::Arc { .. }, Part::Start) => 3,
                (Geometry::Arc { .. }, Part::End) => 5,
                (Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. }, Part::Center) => 0,
                (Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. }, Part::Major) => 2,
                (Geometry::EllipticalArc { .. }, Part::Start) => 5,
                (Geometry::EllipticalArc { .. }, Part::End) => 7,
                _ => return Err(bad(format!("{} is not a point that can be grabbed", r.name()))),
            };
            Ok([b + offset, b + offset + 1])
        };
        match grab {
            Grab::Point(r) => {
                let p = point_vars(r)?;
                for k in 0..2 {
                    soft_vars.push(p[k]);
                    targets.push(Target::Value(x[p[k] as usize]));
                }
            }
            Grab::Edge { entity, at: _ } => {
                let b = base_var(entity)?;
                let e = base.entity(entity).ok_or_else(|| bad(format!("{} can't be moved", entity.whole().name())))?;
                match e.geometry {
                    Geometry::Line { .. } => {
                        for k in 0..4 {
                            soft_vars.push(b + k);
                            targets.push(Target::Value(x[(b + k) as usize]));
                        }
                    }
                    Geometry::Circle { .. } | Geometry::Arc { .. } => {
                        soft_vars.push(b + 2);
                        targets.push(Target::RadiusTo { cx: b, cy: b + 1 });
                    }
                    // Grabbing a spline by its edge moves the whole thing too.
                    Geometry::Spline(ref shape) => {
                        for k in 0..2 * shape.points.len() as u32 {
                            soft_vars.push(b + k);
                            targets.push(Target::Value(x[(b + k) as usize]));
                        }
                    }
                    // Grabbing an ellipse by its edge moves the whole thing, like grabbing a line.
                    Geometry::Ellipse { .. } | Geometry::EllipticalArc { .. } => {
                        for k in 0..4 {
                            soft_vars.push(b + k);
                            targets.push(Target::Value(x[(b + k) as usize]));
                        }
                    }
                    Geometry::Point { .. } => {
                        soft_vars.push(b);
                        targets.push(Target::Value(x[b as usize]));
                        soft_vars.push(b + 1);
                        targets.push(Target::Value(x[b as usize + 1]));
                    }
                }
            }
        }
        // Everything connected to what was grabbed. The rest of the sketch can't move at all.
        let comps = sys.components();
        let mut vars: Vec<u32> = Vec::new();
        let mut rows: Vec<usize> = Vec::new();
        for comp in &comps {
            if comp.vars.iter().any(|v| soft_vars.contains(v)) {
                vars.extend_from_slice(&comp.vars);
                rows.extend_from_slice(&comp.rows);
            }
        }
        for &v in &soft_vars {
            if !vars.contains(&v) {
                vars.push(v);
            }
        }
        vars.sort_unstable();
        rows.sort_unstable();
        let compiled = Compiled::new(&sys, &vars, &rows, &soft_vars);
        let tol = tolerance_for(&sys);
        let goals = vec![0.0; soft_vars.len()];
        let pointer = match grab {
            Grab::Point(r) => {
                let p = point_vars(r)?;
                [x[p[0] as usize], x[p[1] as usize]]
            }
            Grab::Edge { at, .. } => at,
        };
        Ok(DragSession {
            base,
            anchor: x.clone(),
            trial: x.clone(),
            x,
            sys,
            compiled,
            soft_vars,
            targets,
            goals,
            grab,
            pointer,
            tol,
        })
    }

    /// Where the pointer was when the drag began.
    pub fn pointer(&self) -> P2 {
        self.pointer
    }

    /// The sketch as it stands now.
    pub fn sketch(&self) -> Sketch {
        self.sys.write_back(&self.base, &self.x)
    }

    fn update_goals(&mut self, x: &[f64], to: P2) {
        let grabbed_at = self.pointer;
        for (k, t) in self.targets.iter().enumerate() {
            self.goals[k] = match *t {
                Target::Value(start) => {
                    // Coordinates alternate x, y.
                    let axis = k % 2;
                    match self.grab {
                        Grab::Point(_) => to[axis],
                        Grab::Edge { .. } => start + to[axis] - grabbed_at[axis],
                    }
                }
                Target::RadiusTo { cx, cy } => (to[0] - x[cx as usize]).hypot(to[1] - x[cy as usize]).max(1e-6),
            };
        }
    }

    fn merit(&self, x: &[f64]) -> f64 {
        let mut sum = 0.0;
        for &v in &self.compiled.vars {
            let d = x[v as usize] - self.anchor[v as usize];
            sum += d * d;
        }
        for (k, &v) in self.soft_vars.iter().enumerate() {
            let d = x[v as usize] - self.goals[k];
            sum += d * d / SOFTNESS;
        }
        sum / 2.0
    }

    /// Pulls `trial` back onto the constraints. False if it can't get there quickly.
    fn project(&mut self, tol: f64) -> bool {
        let n = self.compiled.vars.len();
        let m = self.compiled.hard.len();
        for _ in 0..8 {
            self.compiled.eval(&self.sys, &self.trial);
            if self.compiled.f.iter().fold(0.0f64, |a, v| a.max(v.abs())) <= tol {
                return true;
            }
            self.compiled.factor(1e-10, OFF);
            self.compiled.rhs.fill(0.0);
            for r in 0..m {
                self.compiled.rhs[n + r] = -self.compiled.f[r];
            }
            let mut rhs = std::mem::take(&mut self.compiled.rhs);
            self.compiled.ldl.solve(&mut rhs);
            let alpha = step_limit(&self.compiled, &self.trial, &rhs[..n]);
            for (col, &v) in self.compiled.vars.iter().enumerate() {
                self.trial[v as usize] += alpha * rhs[col];
            }
            self.compiled.rhs = rhs;
        }
        false
    }

    /// Moves toward the pointer. The sketch stays valid however this goes.
    pub fn move_to(&mut self, to: P2) -> DragReport {
        let n = self.compiled.vars.len();
        let m = self.compiled.hard.len();
        let s = self.soft_vars.len();
        let mut steps = 0;
        let x_now = std::mem::take(&mut self.x);
        self.update_goals(&x_now, to);
        self.x = x_now;
        let mut phi = self.merit(&self.x);
        for _ in 0..MOST_STEPS {
            steps += 1;
            self.compiled.eval(&self.sys, &self.x);
            self.compiled.factor(HARD_DAMPING, SOFTNESS);
            for (col, &v) in self.compiled.vars.iter().enumerate() {
                self.compiled.rhs[col] = self.compiled.weights[col] * (self.anchor[v as usize] - self.x[v as usize]);
            }
            for r in 0..m {
                self.compiled.rhs[n + r] = -self.compiled.f[r];
            }
            for k in 0..s {
                self.compiled.rhs[n + m + k] = self.goals[k] - self.x[self.soft_vars[k] as usize];
            }
            let mut rhs = std::mem::take(&mut self.compiled.rhs);
            self.compiled.ldl.solve(&mut rhs);
            let dx: Vec<f64> = rhs[..n].to_vec();
            self.compiled.rhs = rhs;
            if dx.iter().fold(0.0f64, |a, v| a.max(v.abs())) < 1e-10 * self.sys.size {
                break;
            }
            let mut alpha = step_limit(&self.compiled, &self.x, &dx);
            let mut accepted = None;
            for _ in 0..10 {
                self.trial.copy_from_slice(&self.x);
                for (col, &v) in self.compiled.vars.iter().enumerate() {
                    self.trial[v as usize] += alpha * dx[col];
                }
                // A step that collapses a line or a radius is refused like any other that fails:
                // the sketch must stay one that can be solved again.
                if self.project(self.tol) && !has_collapsed(&self.sys, &self.compiled.vars, &self.trial) {
                    let t = std::mem::take(&mut self.trial);
                    let value = self.merit(&t);
                    self.trial = t;
                    if value < phi {
                        accepted = Some(value);
                        break;
                    }
                }
                alpha /= 2.0;
            }
            let Some(value) = accepted else { break };
            self.x.copy_from_slice(&self.trial);
            let x_now = std::mem::take(&mut self.x);
            self.update_goals(&x_now, to);
            self.x = x_now;
            let done = phi - value <= 1e-11 * phi.max(1e-6);
            phi = self.merit(&self.x);
            if done {
                break;
            }
        }
        // One last pull onto the constraints. It moves things by less than the tolerance, and leaves
        // what's fixed exactly where it was, to rounding.
        self.trial.copy_from_slice(&self.x);
        if self.project(POLISH * self.sys.size.max(1.0)) {
            self.x.copy_from_slice(&self.trial);
        }
        let mut worst = 0.0f64;
        for (k, &v) in self.soft_vars.iter().enumerate() {
            worst = worst.max((self.x[v as usize] - self.goals[k]).abs());
        }
        DragReport { distance: worst, reached: worst <= 1e-6 * self.sys.size.max(1.0), iterations: steps }
    }
}

impl Sketch {
    /// One drag from here to `to`, as a single request, returning the new sketch. Long moves go in
    /// stages, so that the geometry follows the path instead of jumping to a different solution.
    pub fn drag(&self, grab: Grab, to: P2) -> Result<(Sketch, DragReport), SketchError> {
        self.drag_with(grab, to, &Env::new())
    }

    pub fn drag_with(&self, grab: Grab, to: P2, env: &Env) -> Result<(Sketch, DragReport), SketchError> {
        let mut session = DragSession::begin_with(self, grab, env)?;
        let from = session.pointer;
        let distance = (to[0] - from[0]).hypot(to[1] - from[1]);
        let stages = ((distance / (session.sys.size / 8.0)).ceil() as usize).clamp(1, 32);
        let mut report = DragReport { distance: 0.0, reached: true, iterations: 0 };
        for i in 1..=stages {
            let f = i as f64 / stages as f64;
            report = session.move_to([from[0] + (to[0] - from[0]) * f, from[1] + (to[1] - from[1]) * f]);
        }
        Ok((session.sketch(), report))
    }
}
