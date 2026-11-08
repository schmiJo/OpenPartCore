//! Solving: a damped Newton iteration that takes the smallest step satisfying the linearized
//! equations, so geometry the constraints don't require to move stays where it is.

use crate::expr::Env;
use crate::ldl::Ldl;
use crate::model::*;
use crate::rng::Rng;
use crate::system::*;
use std::fmt;

/// Equations are solved to this many millimetres, well inside the kernel's 1e-7.
pub(crate) const TOLERANCE: f64 = 1e-9;
/// Shorter than this, a line or radius counts as collapsed. The kernel's smallest feature.
pub(crate) const COLLAPSED: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    /// The sketch can't be understood: a bad reference, or a wrong kind of entity.
    Invalid,
    /// The named constraints can't all hold.
    Conflict,
    /// A solution may exist, but none was found from this position.
    NoSolution,
    /// The constraints are met only by collapsing an entity to nothing.
    Collapsed,
    /// The sketch has no usable closed regions.
    Regions,
}

/// Why something failed. The sketch it was asked of is unchanged.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchError {
    pub kind: ErrorKind,
    pub message: String,
    /// The constraints involved. For a conflict this is a smallest set: removing any one of
    /// them resolves it.
    pub constraints: Vec<ConstraintId>,
    pub entities: Vec<EntityId>,
}

impl fmt::Display for SketchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for SketchError {}

/// One constraint that adds nothing, because others already imply it.
#[derive(Clone, Debug, PartialEq)]
pub struct Redundancy {
    pub constraint: ConstraintId,
    /// Part of it still does something, such as a coincidence with only one direction left.
    pub partial: bool,
    /// The constraints that together imply it.
    pub with: Vec<ConstraintId>,
}

/// What a solved sketch has left.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Analysis {
    /// Remaining degrees of freedom.
    pub dof: usize,
    /// Parts that can still move. A radius is reported as `Part::Radius`.
    pub free: Vec<Ref>,
    pub redundant: Vec<Redundancy>,
}

/// The result of solving.
#[derive(Clone, Debug, PartialEq)]
pub struct Solved {
    pub sketch: Sketch,
    pub analysis: Analysis,
    pub iterations: usize,
}

/// One component's matrix, ready to factor repeatedly.
pub(crate) struct Compiled {
    pub vars: Vec<u32>,
    pub hard: Vec<usize>,
    /// Single-variable soft equations, for dragging: the column of each.
    pub soft: Vec<u32>,
    /// Where each position of each hard row writes in `vals`. `u32::MAX` for reference geometry.
    slots: Vec<[u32; MAX_VARS]>,
    /// The matrix's values: variable diagonals, row diagonals, then the Jacobian.
    pub vals: Vec<f64>,
    pub ldl: Ldl,
    pub f: Vec<f64>,
    pub rhs: Vec<f64>,
    /// Columns that are radii.
    pub radii: Vec<u32>,
    /// What each radius started as.
    radius_start: Vec<f64>,
    /// How expensive each variable is to move. One, apart from radii that have shrunk.
    pub weights: Vec<f64>,
}

impl Compiled {
    pub fn new(sys: &System, vars: &[u32], hard: &[usize], soft_vars: &[u32]) -> Compiled {
        let n = vars.len();
        let total = n + hard.len() + soft_vars.len();
        let mut col_of = std::collections::HashMap::with_capacity(n);
        for (c, &v) in vars.iter().enumerate() {
            col_of.insert(v, c as u32);
        }
        let mut triplets: Vec<(u32, u32)> = (0..total as u32).map(|i| (i, i)).collect();
        let mut slots = Vec::with_capacity(hard.len());
        for (r, &ri) in hard.iter().enumerate() {
            let row = &sys.rows[ri];
            let mut s = [u32::MAX; MAX_VARS];
            for i in 0..row.count as usize {
                let v = row.vars[i];
                if sys.frozen[v as usize] {
                    continue;
                }
                // The same variable twice in one row shares one entry.
                let earlier = (0..i).find(|&k| row.vars[k] == v);
                s[i] = match earlier {
                    Some(k) => s[k],
                    None => {
                        triplets.push((col_of[&v], (n + r) as u32));
                        triplets.len() as u32 - 1
                    }
                };
            }
            slots.push(s);
        }
        let first_soft = (n + hard.len()) as u32;
        let mut soft = Vec::with_capacity(soft_vars.len());
        for (k, v) in soft_vars.iter().enumerate() {
            let c = col_of[v];
            soft.push(c);
            triplets.push((c, first_soft + k as u32));
        }
        let is_row = (0..total).map(|v| v >= n).collect();
        let ldl = Ldl::new(total, &triplets, is_row);
        let mut vals = vec![0.0; triplets.len()];
        for k in 0..soft.len() {
            vals[triplets.len() - soft.len() + k] = 1.0;
        }
        let radii: Vec<u32> = sys.radii.iter().filter_map(|v| col_of.get(v).copied()).collect();
        let radius_start = radii.iter().map(|&c| sys.x0[vars[c as usize] as usize]).collect();
        Compiled {
            vars: vars.to_vec(),
            hard: hard.to_vec(),
            soft,
            slots,
            vals,
            ldl,
            f: vec![0.0; hard.len()],
            rhs: vec![0.0; total],
            radii,
            radius_start,
            weights: vec![1.0; n],
        }
    }

    /// Evaluates the equations and writes the Jacobian into the matrix.
    pub fn eval(&mut self, sys: &System, x: &[f64]) -> f64 {
        // As a radius shrinks it gets dearer to change, in proportion to the square of how far it
        // has come. Each step then takes less from it, and more from what else can give, so a radius
        // that isn't truly forced to zero never gets there.
        for (&col, &start) in self.radii.iter().zip(&self.radius_start) {
            let r = x[self.vars[col as usize] as usize].max(1e-12);
            self.weights[col as usize] = (start / r).powi(2).max(1.0).min(1e12);
        }
        let mut g = [0.0; MAX_VARS];
        let mut cost = 0.0;
        for (r, &ri) in self.hard.iter().enumerate() {
            let row = &sys.rows[ri];
            let s = &self.slots[r];
            for &slot in s.iter().take(row.count as usize) {
                if slot != u32::MAX {
                    self.vals[slot as usize] = 0.0;
                }
            }
            let v = row.eval(x, &mut g);
            for i in 0..row.count as usize {
                if s[i] != u32::MAX {
                    self.vals[s[i] as usize] += g[i];
                }
            }
            self.f[r] = v;
            cost += v * v;
        }
        cost
    }

    /// Sets the diagonals and factors. `damping` applies to hard rows and `softness` to soft ones.
    pub fn factor(&mut self, damping: f64, softness: f64) {
        let n = self.vars.len();
        self.vals[..n].copy_from_slice(&self.weights);
        let m = self.hard.len();
        for v in &mut self.vals[n..n + m] {
            *v = -damping;
        }
        for v in &mut self.vals[n + m..n + m + self.soft.len()] {
            *v = -softness;
        }
        self.ldl.factor(&self.vals);
    }
}

/// The value of the equations at `x`, without touching the matrix.
pub(crate) fn cost_at(sys: &System, hard: &[usize], x: &[f64]) -> (f64, f64) {
    let mut g = [0.0; MAX_VARS];
    let (mut sum, mut largest) = (0.0, 0.0f64);
    for &ri in hard {
        let v = sys.rows[ri].eval(x, &mut g);
        sum += v * v;
        largest = largest.max(v.abs());
    }
    (sum, largest)
}

/// How far along a step radii stay positive.
pub(crate) fn step_limit(c: &Compiled, x: &[f64], dx: &[f64]) -> f64 {
    let mut alpha: f64 = 1.0;
    for &col in &c.radii {
        let (r, d) = (x[c.vars[col as usize] as usize], dx[col as usize]);
        if d < 0.0 {
            alpha = alpha.min(-0.9 * r / d);
        }
    }
    alpha
}

pub(crate) fn tolerance_for(sys: &System) -> f64 {
    TOLERANCE * (sys.size / 1000.0).max(1.0)
}

/// Drives the equations to `target` (zero when `None`) from `x`, updating it. Gives the iterations
/// used; on `Err`, it stalled, with `x` left at the best point reached.
fn newton_to(sys: &System, c: &mut Compiled, x: &mut [f64], tol: f64, limit: usize, target: Option<&[f64]>) -> Result<usize, usize> {
    let n = c.vars.len();
    let goal = |r: usize| target.map_or(0.0, |t| t[r]);
    let cost_of = |x: &[f64], hard: &[usize]| -> f64 {
        let mut g = [0.0; MAX_VARS];
        let mut sum = 0.0;
        for (r, &ri) in hard.iter().enumerate() {
            let v = sys.rows[ri].eval(x, &mut g) - goal(r);
            sum += v * v;
        }
        sum
    };
    c.eval(sys, x);
    let mut cost = cost_of(x, &c.hard);
    let mut damping = 1e-10;
    let mut trial = x.to_vec();
    let mut slow = 0;
    let largest_of = |f: &[f64]| f.iter().enumerate().fold(0.0f64, |m, (r, v)| m.max((v - goal(r)).abs()));
    for iteration in 0..limit {
        let largest = largest_of(&c.f);
        if largest <= tol {
            // One more step costs little and lands far inside the tolerance. Accepted only if it
            // helps, so a solved sketch stays bit for bit where it is.
            if largest > 1e-3 * tol && iteration > 0 {
                c.factor(1e-12, 1.0);
                c.rhs[..n].fill(0.0);
                for (r, v) in c.f.iter().enumerate() {
                    c.rhs[n + r] = goal(r) - v;
                }
                let mut rhs = std::mem::take(&mut c.rhs);
                c.ldl.solve(&mut rhs);
                trial.copy_from_slice(x);
                for (col, &v) in c.vars.iter().enumerate() {
                    trial[v as usize] += rhs[col];
                }
                c.rhs = rhs;
                if cost_of(&trial, &c.hard) < cost {
                    x.copy_from_slice(&trial);
                }
            }
            return Ok(iteration);
        }
        let accepted = loop {
            c.factor(damping, 1.0);
            c.rhs[..n].fill(0.0);
            for (r, v) in c.f.iter().enumerate() {
                c.rhs[n + r] = goal(r) - v;
            }
            let mut rhs = std::mem::take(&mut c.rhs);
            c.ldl.solve(&mut rhs);
            let alpha = step_limit(c, x, &rhs[..n]);
            trial.copy_from_slice(x);
            for (col, &v) in c.vars.iter().enumerate() {
                trial[v as usize] += alpha * rhs[col];
            }
            let step: f64 = rhs[..n].iter().fold(0.0, |m, v| m.max(v.abs()));
            c.rhs = rhs;
            let next = cost_of(&trial, &c.hard);
            if next.is_finite() && next < cost {
                break Some(next);
            }
            if damping < 1e-6 {
                damping = 1e-6;
            } else {
                damping *= 16.0;
            }
            if damping > 1e10 || step < 1e-15 * sys.size {
                break None;
            }
        };
        let Some(next) = accepted else {
            return Err(iteration);
        };
        x.copy_from_slice(&trial);
        slow = if cost - next < 1e-10 * cost { slow + 1 } else { 0 };
        c.eval(sys, x);
        if slow >= 8 {
            return if largest_of(&c.f) <= tol { Ok(iteration) } else { Err(iteration) };
        }
        damping = (damping / 8.0).max(1e-12);
        cost = next;
    }
    if largest_of(&c.f) <= tol {
        Ok(limit)
    } else {
        Err(limit)
    }
}

/// Newton toward zero, from wherever `x` is.
pub(crate) fn newton(sys: &System, c: &mut Compiled, x: &mut [f64], tol: f64, limit: usize) -> Result<usize, usize> {
    newton_to(sys, c, x, tol, limit, None)
}

/// Follows a path instead of going straight: shrinks the equations' errors in stages, from what
/// they are now to zero, solving at each. Slower, and it gets through many starts where Newton
/// alone stalls in a dip that isn't a solution.
fn follow_path(sys: &System, c: &mut Compiled, x: &mut [f64], tol: f64, stages: usize) -> Result<usize, usize> {
    c.eval(sys, x);
    let start: Vec<f64> = c.f.clone();
    let mut target = vec![0.0; start.len()];
    let mut total = 0;
    let loose = (tol * 1e3).max(1e-6);
    for stage in 1..stages {
        let left = 1.0 - stage as f64 / stages as f64;
        for (t, s) in target.iter_mut().zip(&start) {
            *t = s * left;
        }
        match newton_to(sys, c, x, loose, 30, Some(&target)) {
            Ok(i) => total += i,
            Err(i) => return Err(total + i),
        }
    }
    match newton_to(sys, c, x, tol, 100, None) {
        Ok(i) => Ok(total + i),
        Err(i) => Err(total + i),
    }
}

/// Whether a line, arc or radius in these variables has collapsed.
pub(crate) fn has_collapsed(sys: &System, vars: &[u32], x: &[f64]) -> bool {
    let inside = |v: u32| vars.binary_search(&v).is_ok();
    sys.radii.iter().any(|&r| inside(r) && x[r as usize] < COLLAPSED)
        || sys.spans.iter().any(|s| {
            s.iter().any(|&v| inside(v)) && (x[s[2] as usize] - x[s[0] as usize]).hypot(x[s[3] as usize] - x[s[1] as usize]) < COLLAPSED
        })
}

/// The typical size of the geometry in these variables: the median length or radius. Nudges are
/// scaled by this and not by the sketch's extent, since a part 10 mm across can sit a metre from the
/// origin, or from the rest of a large sketch.
fn feature_size(sys: &System, vars: &[u32], x: &[f64]) -> f64 {
    let inside = |v: u32| vars.binary_search(&v).is_ok();
    let mut sizes: Vec<f64> = sys.radii.iter().filter(|&&r| inside(r)).map(|&r| x[r as usize].abs()).collect();
    for s in &sys.spans {
        if s.iter().any(|&v| inside(v)) {
            sizes.push((x[s[2] as usize] - x[s[0] as usize]).hypot(x[s[3] as usize] - x[s[1] as usize]));
        }
    }
    sizes.retain(|v| *v > COLLAPSED);
    if sizes.is_empty() {
        return 1.0;
    }
    sizes.sort_by(f64::total_cmp);
    sizes[sizes.len() / 2]
}

/// The variables in equations that aren't met, or in anything that has collapsed: where a stalled
/// solve is actually stuck.
fn trouble_vars(sys: &System, vars: &[u32], rows: &[usize], x: &[f64], tol: f64) -> Vec<u32> {
    let mut g = [0.0; MAX_VARS];
    let mut out: Vec<u32> = Vec::new();
    for &ri in rows {
        let row = &sys.rows[ri];
        if row.eval(x, &mut g).abs() > tol {
            out.extend(row.vars[..row.count as usize].iter().copied().filter(|&v| !sys.frozen[v as usize]));
        }
    }
    let inside = |v: u32| vars.binary_search(&v).is_ok();
    for s in &sys.spans {
        if s.iter().any(|&v| inside(v)) && (x[s[2] as usize] - x[s[0] as usize]).hypot(x[s[3] as usize] - x[s[1] as usize]) < COLLAPSED {
            out.extend(s.iter().copied().filter(|&v| inside(v)));
        }
    }
    for &r in &sys.radii {
        if inside(r) && x[r as usize] < COLLAPSED {
            out.push(r);
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// The variables within two steps of `trouble`, going through equations, and the equations entirely
/// among them.
fn neighbourhood(sys: &System, rows: &[usize], trouble: &[u32]) -> (Vec<u32>, Vec<usize>) {
    let moving = |r: &Row| -> Vec<u32> { r.vars[..r.count as usize].iter().copied().filter(|&v| !sys.frozen[v as usize]).collect() };
    let mut near: std::collections::BTreeSet<u32> = trouble.iter().copied().collect();
    for _ in 0..2 {
        let mut grown = near.clone();
        for &ri in rows {
            let list = moving(&sys.rows[ri]);
            if list.iter().any(|v| near.contains(v)) {
                grown.extend(list);
            }
        }
        near = grown;
    }
    let inside: Vec<usize> = rows.iter().copied().filter(|&ri| moving(&sys.rows[ri]).iter().all(|v| near.contains(v))).collect();
    (near.into_iter().collect(), inside)
}

/// Solves a set of rows from `x0`, trying harder each time the last way stalls or only finds an
/// answer that collapses something. `out` gets the result, or the last point reached.
///
/// In a large sketch, Newton solves almost everything and sticks on one small part. So the later
/// tries start from where it stuck and disturb only that part, which leaves the rest alone. A true
/// conflict stalls every time.
pub(crate) fn solve_rows(sys: &System, vars: &[u32], rows: &[usize], x0: &[f64], out: &mut Vec<f64>, seed: u64) -> Result<usize, usize> {
    let tol = tolerance_for(sys);
    let mut c = Compiled::new(sys, vars, rows, &[]);
    let mut total = 0;
    let mut collapsed_answer: Option<Vec<f64>> = None;
    let mut rng = Rng::new(seed ^ 0x51ED_270B);
    let mut finish = |result: Result<usize, usize>, out: &Vec<f64>, total: &mut usize| -> bool {
        match result {
            Ok(i) => {
                *total += i;
                if !has_collapsed(sys, vars, out) {
                    return true;
                }
                if collapsed_answer.is_none() {
                    collapsed_answer = Some(out.clone());
                }
                false
            }
            Err(i) => {
                *total += i;
                false
            }
        }
    };
    out.clear();
    out.extend_from_slice(x0);
    let result = newton(sys, &mut c, out, tol, 200);
    if finish(result, out, &mut total) {
        return Ok(total);
    }
    let stuck = out.clone();
    out.copy_from_slice(x0);
    let result = follow_path(sys, &mut c, out, tol, 16);
    if finish(result, out, &mut total) {
        return Ok(total);
    }
    let trouble = trouble_vars(sys, vars, rows, &stuck, tol);
    let feature = feature_size(sys, vars, x0);
    // The stuck part alone: what's wrong, and what's near it, as a small problem of its own.
    let (near_vars, near_rows) = neighbourhood(sys, rows, &trouble);
    let local_differs = near_rows.len() < rows.len();
    let mut local = local_differs.then(|| Compiled::new(sys, &near_vars, &near_rows, &[]));
    for (magnitude, from_start, path) in [
        (0.0, true, false),
        (0.0, true, true),
        (1e-3, false, false),
        (3e-2, false, false),
        (0.1, false, false),
        (0.1, true, false),
        (0.1, true, true),
        (0.3, false, false),
        (0.3, true, true),
    ] {
        out.copy_from_slice(&stuck);
        for &v in &trouble {
            let base = if from_start { x0[v as usize] } else { stuck[v as usize] };
            out[v as usize] = base + rng.range(-1.0, 1.0) * magnitude * feature;
        }
        for &v in &sys.radii {
            if out[v as usize] < COLLAPSED {
                out[v as usize] = x0[v as usize].max(feature * 0.1);
            }
        }
        // Settle the stuck part first, if that's smaller than the whole, then the whole.
        if let Some(l) = &mut local {
            let settled = if path { follow_path(sys, l, out, tol, 16) } else { newton(sys, l, out, tol, 200) };
            match settled {
                Ok(i) => total += i,
                Err(i) => {
                    total += i;
                    continue;
                }
            }
        }
        let result = if path && local.is_none() { follow_path(sys, &mut c, out, tol, 16) } else { newton(sys, &mut c, out, tol, 200) };
        if finish(result, out, &mut total) {
            return Ok(total);
        }
    }
    // Nothing better than an answer that collapses something. The caller reports it as that.
    if let Some(answer) = collapsed_answer {
        *out = answer;
        return Ok(total);
    }
    Err(total)
}

/// The rank of a component's equations at `x`, by elimination with rows taken in order, so that
/// the constraint added last is the one that gets called redundant.
pub(crate) struct Rank {
    pub rank: usize,
    /// Pivot rows, as (pivot column, entries).
    pub pivots: Vec<(u32, Vec<(u32, f64)>)>,
    /// Rows that came out dependent, with the combination of rows proving it.
    pub dependent: Vec<(usize, Vec<(usize, f64)>)>,
}

/// A row reduced below this, relative to its original size, depends on the others.
const RANK_TOLERANCE: f64 = 1e-8;

pub(crate) fn rank_of(sys: &System, vars: &[u32], rows: &[usize], x: &[f64]) -> Rank {
    let n = vars.len();
    let mut col_of = std::collections::HashMap::with_capacity(n);
    for (c, &v) in vars.iter().enumerate() {
        col_of.insert(v, c as u32);
    }
    let mut work = vec![0.0f64; n];
    let mut touched: Vec<u32> = Vec::new();
    let mut is_touched = vec![false; n];
    let mut combo = vec![0.0f64; rows.len()];
    let mut combo_touched: Vec<u32> = Vec::new();
    let mut in_combo = vec![false; rows.len()];
    // Each pivot: its column, value, entries and proof.
    struct Pivot {
        col: u32,
        value: f64,
        entries: Vec<(u32, f64)>,
        combo: Vec<(u32, f64)>,
    }
    let mut pivots: Vec<Pivot> = Vec::new();
    let mut dependent = Vec::new();
    let mut g = [0.0; MAX_VARS];
    for (r, &ri) in rows.iter().enumerate() {
        let row = &sys.rows[ri];
        row.eval(x, &mut g);
        let mut norm = 0.0;
        for i in 0..row.count as usize {
            let v = row.vars[i];
            if sys.frozen[v as usize] {
                continue;
            }
            let c = col_of[&v];
            if !is_touched[c as usize] {
                is_touched[c as usize] = true;
                touched.push(c);
            }
            work[c as usize] += g[i];
        }
        for &c in &touched {
            norm += work[c as usize] * work[c as usize];
        }
        let norm = norm.sqrt();
        if norm > 1e-14 {
            for &c in &touched {
                work[c as usize] /= norm;
            }
        }
        combo[r] = if norm > 1e-14 { 1.0 / norm } else { 1.0 };
        in_combo[r] = true;
        combo_touched.push(r as u32);
        for p in &pivots {
            let v = work[p.col as usize];
            if v == 0.0 {
                continue;
            }
            let factor = v / p.value;
            for &(c, a) in &p.entries {
                if !is_touched[c as usize] {
                    is_touched[c as usize] = true;
                    touched.push(c);
                }
                work[c as usize] -= factor * a;
            }
            work[p.col as usize] = 0.0;
            for &(k, a) in &p.combo {
                if !in_combo[k as usize] {
                    in_combo[k as usize] = true;
                    combo_touched.push(k);
                }
                combo[k as usize] -= factor * a;
            }
        }
        let (mut best, mut best_col) = (0.0f64, None);
        for &c in &touched {
            let v = work[c as usize].abs();
            if v > best {
                best = v;
                best_col = Some(c);
            }
        }
        let proof: Vec<(u32, f64)> = combo_touched.iter().map(|&k| (k, combo[k as usize])).filter(|e| e.1 != 0.0).collect();
        if best >= RANK_TOLERANCE {
            let col = best_col.unwrap();
            let entries: Vec<(u32, f64)> = touched.iter().map(|&c| (c, work[c as usize])).filter(|e| e.1.abs() > 1e-15).collect();
            pivots.push(Pivot { col, value: work[col as usize], entries, combo: proof });
        } else {
            dependent.push((r, proof.iter().map(|&(k, a)| (k as usize, a)).collect()));
        }
        for &c in &touched {
            work[c as usize] = 0.0;
            is_touched[c as usize] = false;
        }
        touched.clear();
        for &k in &combo_touched {
            combo[k as usize] = 0.0;
            in_combo[k as usize] = false;
        }
        combo_touched.clear();
    }
    Rank { rank: pivots.len(), pivots: pivots.into_iter().map(|p| (p.col, p.entries)).collect(), dependent }
}

/// Which columns can still move: those with an entry in some null-space vector.
fn free_columns(rank: &Rank, n: usize) -> Vec<bool> {
    let mut pivot_col = vec![false; n];
    for (c, _) in &rank.pivots {
        pivot_col[*c as usize] = true;
    }
    let mut free = vec![false; n];
    let mut v = vec![0.0f64; n];
    for j in 0..n {
        if pivot_col[j] {
            continue;
        }
        free[j] = true;
        v.fill(0.0);
        v[j] = 1.0;
        for (c, entries) in rank.pivots.iter().rev() {
            let (mut sum, mut pivot) = (0.0, 1.0);
            for &(k, a) in entries {
                if k == *c {
                    pivot = a;
                } else {
                    sum += a * v[k as usize];
                }
            }
            v[*c as usize] = -sum / pivot;
            if v[*c as usize].abs() > 1e-7 {
                free[*c as usize] = true;
            }
        }
    }
    free
}

fn analyze_at(sketch: &Sketch, sys: &System, comps: &[Component], x: &[f64]) -> Analysis {
    let mut analysis = Analysis::default();
    let locate = sys.locate(sketch);
    let mut in_component = vec![false; sys.x0.len()];
    let mut free_vars = vec![false; sys.x0.len()];
    let mut per_constraint: std::collections::BTreeMap<usize, (usize, Vec<ConstraintId>)> = Default::default();
    for comp in comps {
        for &v in &comp.vars {
            in_component[v as usize] = true;
        }
        let rank = rank_of(sys, &comp.vars, &comp.rows, x);
        analysis.dof += comp.vars.len() - rank.rank;
        for (c, &f) in free_columns(&rank, comp.vars.len()).iter().enumerate() {
            if f {
                free_vars[comp.vars[c] as usize] = true;
            }
        }
        for (r, proof) in &rank.dependent {
            let Some(owner) = sys.rows[comp.rows[*r]].owner else { continue };
            let entry = per_constraint.entry(owner).or_default();
            entry.0 += 1;
            for &(k, a) in proof {
                if a.abs() < 1e-7 {
                    continue;
                }
                if let Some(o) = sys.rows[comp.rows[k]].owner {
                    let id = sys.constraint_ids[o];
                    if o != owner && !entry.1.contains(&id) {
                        entry.1.push(id);
                    }
                }
            }
        }
    }
    // Rows on reference geometry alone say nothing.
    for &ri in &sys.constant_rows() {
        if let Some(owner) = sys.rows[ri].owner {
            per_constraint.entry(owner).or_default().0 += 1;
        }
    }
    for (owner, (dependent_rows, mut with)) in per_constraint {
        let total = sys.rows.iter().filter(|r| r.owner == Some(owner)).count();
        with.sort();
        analysis.redundant.push(Redundancy { constraint: sys.constraint_ids[owner], partial: dependent_rows < total, with });
    }
    // Everything no equation touches is free too.
    for v in 0..sys.x0.len() {
        if !sys.frozen[v] && !in_component[v] {
            analysis.dof += 1;
            free_vars[v] = true;
        }
    }
    let mut free: Vec<Ref> = Vec::new();
    for v in 0..sys.x0.len() {
        if free_vars[v] {
            if let Some(r) = locate[v] {
                if !free.contains(&r) {
                    free.push(r);
                }
            }
        }
    }
    analysis.free = free;
    analysis
}

/// Names a constraint for a message.
pub(crate) fn describe(sketch: &Sketch, id: ConstraintId) -> String {
    match sketch.constraint(id) {
        Some(c) => match c.rule.value() {
            Some(v) => format!("c{} ({} = {})", id.0, c.rule.kind(), v),
            None => format!("c{} ({})", id.0, c.rule.kind()),
        },
        None => format!("c{}", id.0),
    }
}

/// How many scattered starts to try before calling a set of constraints a conflict.
const STARTS_TO_TRY: usize = 24;
/// The most constraints to shrink down to a smallest conflicting set: each one tried costs a solve.
const MOST_TO_SHRINK: usize = 64;

/// Works out why a component failed: finds the constraints that can't all hold, and shrinks
/// them to a smallest set.
fn explain(sketch: &Sketch, sys: &System, comp: &Component, stuck: &[f64], gone: &[EntityId]) -> SketchError {
    let tol = tolerance_for(sys);
    // Dependent rows whose values don't cancel are a proof of conflict at this point. At a stall,
    // the leftover error is itself such a combination, so its support is a fallback.
    let rank = rank_of(sys, &comp.vars, &comp.rows, stuck);
    let mut g = [0.0; MAX_VARS];
    let values: Vec<f64> = comp.rows.iter().map(|&ri| sys.rows[ri].eval(stuck, &mut g)).collect();
    let mut suspects: Vec<usize> = Vec::new();
    let add = |suspects: &mut Vec<usize>, r: usize| {
        if let Some(o) = sys.rows[comp.rows[r]].owner {
            if !suspects.contains(&o) {
                suspects.push(o);
            }
        }
    };
    for (_, proof) in &rank.dependent {
        let total: f64 = proof.iter().map(|&(k, a)| a * values[k]).sum();
        if total.abs() > 10.0 * tol {
            for &(k, a) in proof {
                if a.abs() > 1e-7 {
                    add(&mut suspects, k);
                }
            }
        }
    }
    if suspects.is_empty() {
        for (r, v) in values.iter().enumerate() {
            if v.abs() > tol {
                add(&mut suspects, r);
            }
        }
    }
    let mut out = Vec::new();
    let feasible = |keep: &dyn Fn(usize) -> bool, out: &mut Vec<f64>| -> bool {
        let rows: Vec<usize> = comp
            .rows
            .iter()
            .copied()
            .filter(|&ri| match sys.rows[ri].owner {
                None => true,
                Some(o) => keep(o),
            })
            .collect();
        solve_rows(sys, &comp.vars, &rows, &sys.x0, out, 0).is_ok() && !has_collapsed(sys, &comp.vars, out)
    };
    // The suspects alone may be solvable, if the conflict also runs through others.
    let all: Vec<usize> = {
        let mut v: Vec<usize> = comp.rows.iter().filter_map(|&ri| sys.rows[ri].owner).collect();
        v.dedup();
        v.sort_unstable();
        v.dedup();
        v
    };
    let conflict_exists = !feasible(&|_| false, &mut out) || !feasible(&|o| all.contains(&o), &mut out);
    let mut set = if !suspects.is_empty() && !feasible(&|o| suspects.contains(&o), &mut out) { suspects.clone() } else { all.clone() };
    set.sort_unstable();
    let gone_names = gone.iter().map(|e| e.0.to_string()).collect::<Vec<_>>().join(", ");
    // Too many to shrink to a smallest set in reasonable time. Say where it stuck, and no more.
    if set.len() > MOST_TO_SHRINK {
        if !gone.is_empty() {
            return SketchError {
                kind: ErrorKind::Collapsed,
                message: format!("the only solution found collapses entity {gone_names} to nothing"),
                constraints: Vec::new(),
                entities: gone.to_vec(),
            };
        }
        let mut stuck_on: Vec<usize> = values
            .iter()
            .enumerate()
            .filter(|(_, v)| v.abs() > tol)
            .filter_map(|(r, _)| sys.rows[comp.rows[r]].owner)
            .collect();
        stuck_on.dedup();
        stuck_on.truncate(12);
        let ids: Vec<ConstraintId> = stuck_on.iter().map(|&o| sys.constraint_ids[o]).collect();
        let names: Vec<String> = ids.iter().map(|&id| describe(sketch, id)).collect();
        return SketchError {
            kind: ErrorKind::NoSolution,
            message: format!(
                "no solution was found from this position. It stalled with these unmet, and may have more: {}. Moving the geometry closer to what the constraints ask may help.",
                names.join(", ")
            ),
            constraints: ids,
            entities: Vec::new(),
        };
    }
    // Deletion filter: drop whatever isn't needed for the failure. What's left is a smallest set.
    if conflict_exists {
        let mut i = set.len();
        while i > 0 {
            i -= 1;
            let without: Vec<usize> = set.iter().copied().filter(|&o| o != set[i]).collect();
            if !feasible(&|o| without.contains(&o), &mut out) {
                set = without;
            }
        }
    }
    let ids: Vec<ConstraintId> = set.iter().map(|&o| sys.constraint_ids[o]).collect();
    let names: Vec<String> = ids.iter().map(|&id| describe(sketch, id)).collect();
    // A stall alone doesn't prove a conflict: it may only be a dip near this position. So try these
    // constraints from starting positions scattered over the whole sketch. If any of them works, an
    // arrangement exists, and this is a failure to find it, which is a different thing to report.
    let exists_elsewhere = !conflict_exists || {
        let rows: Vec<usize> = comp
            .rows
            .iter()
            .copied()
            .filter(|&ri| match sys.rows[ri].owner {
                None => true,
                Some(o) => set.contains(&o),
            })
            .collect();
        let mut c = Compiled::new(sys, &comp.vars, &rows, &[]);
        let mut rng = Rng::new(0xC0FF1C7);
        // Only what these constraints touch moves, scattered over the size of those parts.
        let mut involved: Vec<u32> = Vec::new();
        for &ri in &rows {
            let r = &sys.rows[ri];
            involved.extend(r.vars[..r.count as usize].iter().copied().filter(|&v| !sys.frozen[v as usize]));
        }
        involved.sort_unstable();
        involved.dedup();
        let (lo, hi) = involved.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &v| (a.min(sys.x0[v as usize]), b.max(sys.x0[v as usize])));
        let span = (hi - lo).max(feature_size(sys, &involved, &sys.x0)).max(1e-3);
        (0..STARTS_TO_TRY).any(|k| {
            out.clear();
            out.extend_from_slice(&sys.x0);
            let reach = if k % 2 == 0 { span } else { span / 4.0 };
            for &v in &involved {
                out[v as usize] += rng.range(-1.0, 1.0) * reach;
            }
            for &v in &sys.radii {
                if out[v as usize] < COLLAPSED {
                    out[v as usize] = rng.range(0.05, 1.0) * span;
                }
            }
            newton(sys, &mut c, &mut out[..], tol, 200).is_ok() && !has_collapsed(sys, &comp.vars, &out[..])
        })
    };
    // Linear equations that don't fit are a proof. For anything else, not finding an arrangement is
    // strong evidence but no more, and the message says only what was done.
    let proven = comp.rows.iter().all(|&ri| match (sys.rows[ri].owner, sys.rows[ri].eq) {
        (_, Eq::Linear { .. }) => true,
        (Some(o), _) => !set.contains(&o),
        (None, _) => true,
    });
    let (kind, message) = if ids.is_empty() {
        (ErrorKind::NoSolution, "no solution was found from this position".to_string())
    } else if exists_elsewhere {
        (ErrorKind::NoSolution, format!("no solution was found near this position, though one exists elsewhere for: {}. Moving the geometry closer to what the constraints ask may help.", names.join(", ")))
    } else if !gone.is_empty() {
        (ErrorKind::Collapsed, format!("these constraints can only be met by collapsing entity {gone_names} to nothing: {}", names.join(", ")))
    } else if proven {
        (ErrorKind::Conflict, format!("these constraints can't all hold: {}", names.join(", ")))
    } else {
        (ErrorKind::Conflict, format!("these constraints conflict: no arrangement meeting all of them was found, from here or from {STARTS_TO_TRY} other starting positions: {}", names.join(", ")))
    };
    SketchError { kind, message, constraints: ids, entities: gone.to_vec() }
}

/// Lines or radii that a solution has collapsed.
pub(crate) fn collapsed(sketch: &Sketch) -> Vec<EntityId> {
    let mut out = Vec::new();
    for e in &sketch.entities {
        let gone = match e.geometry {
            Geometry::Spline(ref shape) => {
                let (a, b) = (shape.points[0], shape.points[shape.points.len() - 1]);
                shape.points.iter().all(|q| (q[0] - a[0]).hypot(q[1] - a[1]) < COLLAPSED) && (a[0] - b[0]).hypot(a[1] - b[1]) < COLLAPSED
            }
            Geometry::Point { .. } => false,
            Geometry::Line { start, end } => (end[0] - start[0]).hypot(end[1] - start[1]) < COLLAPSED,
            Geometry::Circle { radius, .. } => radius < COLLAPSED,
            Geometry::Arc { radius, start, end, .. } => radius < COLLAPSED || (end[0] - start[0]).hypot(end[1] - start[1]) < COLLAPSED,
            Geometry::Ellipse { center, major, minor } => minor < COLLAPSED || (major[0] - center[0]).hypot(major[1] - center[1]) < COLLAPSED,
            Geometry::EllipticalArc { center, major, minor, start, end } => {
                minor < COLLAPSED || (major[0] - center[0]).hypot(major[1] - center[1]) < COLLAPSED || (end[0] - start[0]).hypot(end[1] - start[1]) < COLLAPSED
            }
        };
        if gone {
            out.push(e.id);
        }
    }
    out
}

impl Sketch {
    /// Lines, circles and arcs that have collapsed to nothing, as they stand now.
    pub fn collapsed_entities(&self) -> Vec<EntityId> {
        collapsed(self)
    }

    /// Solves with no parameters.
    pub fn solve(&self) -> Result<Solved, SketchError> {
        self.solve_with(&Env::new())
    }

    /// Solves, with dimension expressions reading from `env`. Returns a new sketch: this one is
    /// unchanged, whatever happens.
    pub fn solve_with(&self, env: &Env) -> Result<Solved, SketchError> {
        let sys = System::build(self, env)?;
        let comps = sys.components();
        let tol = tolerance_for(&sys);
        let mut g = [0.0; MAX_VARS];
        for &ri in &sys.constant_rows() {
            let v = sys.rows[ri].eval(&sys.x0, &mut g);
            if v.abs() > tol {
                if let Some(o) = sys.rows[ri].owner {
                    let id = sys.constraint_ids[o];
                    return Err(SketchError {
                        kind: ErrorKind::Conflict,
                        message: format!("this constraint can't be met: {}", describe(self, id)),
                        constraints: vec![id],
                        entities: Vec::new(),
                    });
                }
            }
        }
        let mut x = sys.x0.clone();
        let mut iterations = 0;
        let mut out = Vec::new();
        for (index, comp) in comps.iter().enumerate() {
            let (_, largest) = cost_at(&sys, &comp.rows, &x);
            if largest <= tol {
                continue;
            }
            match solve_rows(&sys, &comp.vars, &comp.rows, &x, &mut out, index as u64) {
                Ok(i) => {
                    iterations += i;
                    for &v in &comp.vars {
                        x[v as usize] = out[v as usize];
                    }
                }
                Err(_) => return Err(explain(self, &sys, comp, &out, &[])),
            }
            if has_collapsed(&sys, &comp.vars, &x) {
                let gone = collapsed(&sys.write_back(self, &x));
                return Err(explain(self, &sys, comp, &x, &gone));
            }
        }
        let sketch = sys.write_back(self, &x);
        let gone = collapsed(&sketch);
        if !gone.is_empty() {
            let names: Vec<String> = gone.iter().map(|e| e.0.to_string()).collect();
            return Err(SketchError {
                kind: ErrorKind::Collapsed,
                message: format!("the only solution found collapses entity {} to nothing", names.join(", ")),
                constraints: Vec::new(),
                entities: gone,
            });
        }
        let analysis = analyze_at(self, &sys, &comps, &x);
        Ok(Solved { sketch, analysis, iterations })
    }

    /// Reports degrees of freedom and redundancy at the current geometry, without solving.
    pub fn analyze(&self) -> Result<Analysis, SketchError> {
        self.analyze_with(&Env::new())
    }

    pub fn analyze_with(&self, env: &Env) -> Result<Analysis, SketchError> {
        let sys = System::build(self, env)?;
        let comps = sys.components();
        Ok(analyze_at(self, &sys, &comps, &sys.x0))
    }
}
