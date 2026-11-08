//! A sparse LDLᵀ factorization of the symmetric systems the solver produces, with no pivoting
//! at solve time and a fill-reducing order chosen once.
//!
//! The matrix is `[W Jᵀ; J -Λ]`, with `W` and `Λ` positive diagonals: variables first, equations
//! after. Such a matrix has an LDLᵀ factorization for any order, with positive pivots on
//! variables and negative ones on equations. Working on this form, and not on `J Jᵀ`, matters
//! when one line is used by hundreds of constraints: its column would make `J Jᵀ` completely
//! dense, while here it is simply eliminated last.
//!
//! An equation's own diagonal is tiny, so an equation only becomes a good pivot once one of its
//! variables has been eliminated. The order respects that.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub struct Ldl {
    n: usize,
    /// New position to old node.
    perm: Vec<u32>,
    ap: Vec<u32>,
    ai: Vec<u32>,
    /// Entry of `ax` for each input triplet.
    slot: Vec<u32>,
    ax: Vec<f64>,
    parent: Vec<i32>,
    lp: Vec<u32>,
    li: Vec<u32>,
    lx: Vec<f64>,
    d: Vec<f64>,
    is_row: Vec<bool>,
    // Work space, kept so that a factorization allocates nothing.
    lnz: Vec<u32>,
    y: Vec<f64>,
    pattern: Vec<u32>,
    flag: Vec<i32>,
    b: Vec<f64>,
}

/// A minimum-degree order on the graph of the matrix, with equations held back until a variable
/// next to them has been eliminated.
fn order(n: usize, mut adj: Vec<Vec<u32>>, is_row: &[bool]) -> Vec<u32> {
    let mut eligible: Vec<bool> = is_row.iter().map(|r| !r).collect();
    let mut done = vec![false; n];
    let mut heap = BinaryHeap::with_capacity(n);
    for (v, list) in adj.iter_mut().enumerate() {
        list.sort_unstable();
        list.dedup();
        if eligible[v] {
            heap.push(Reverse((list.len() as u32, v as u32)));
        }
    }
    let mut out = Vec::with_capacity(n);
    let mut merged: Vec<u32> = Vec::new();
    while let Some(Reverse((degree, v))) = heap.pop() {
        let v = v as usize;
        if done[v] || !eligible[v] || degree as usize != adj[v].len() {
            continue;
        }
        done[v] = true;
        out.push(v as u32);
        let around = std::mem::take(&mut adj[v]);
        for &u in &around {
            let u = u as usize;
            // Merge the neighbour's list with `around`, leaving out itself and `v`.
            merged.clear();
            let (a, b) = (&adj[u], &around);
            let (mut i, mut j) = (0, 0);
            while i < a.len() || j < b.len() {
                let next = if j >= b.len() || (i < a.len() && a[i] <= b[j]) {
                    let x = a[i];
                    if j < b.len() && b[j] == x {
                        j += 1;
                    }
                    i += 1;
                    x
                } else {
                    let x = b[j];
                    j += 1;
                    x
                };
                if next as usize != u && next as usize != v {
                    merged.push(next);
                }
            }
            std::mem::swap(&mut adj[u], &mut merged);
            if !is_row[v] && is_row[u] {
                eligible[u] = true;
            }
            if eligible[u] {
                heap.push(Reverse((adj[u].len() as u32, u as u32)));
            }
        }
    }
    // Every node is reached: variables are always eligible, and each equation has a variable.
    // An equation with none would be a caller's bug, so it is still placed, at the end.
    for v in 0..n {
        if !done[v] {
            out.push(v as u32);
        }
    }
    out
}

impl Ldl {
    /// `triplets` lists the matrix's entries, each once, on or above the diagonal in either order.
    pub fn new(n: usize, triplets: &[(u32, u32)], is_row: Vec<bool>) -> Ldl {
        let mut adj = vec![Vec::new(); n];
        for &(i, j) in triplets {
            if i != j {
                adj[i as usize].push(j);
                adj[j as usize].push(i);
            }
        }
        let perm = order(n, adj, &is_row);
        let mut pinv = vec![0u32; n];
        for (new, &old) in perm.iter().enumerate() {
            pinv[old as usize] = new as u32;
        }
        // Compressed columns, upper triangle.
        let mut counts = vec![0u32; n + 1];
        let moved: Vec<(u32, u32)> = triplets
            .iter()
            .map(|&(i, j)| {
                let (a, b) = (pinv[i as usize], pinv[j as usize]);
                (a.min(b), a.max(b))
            })
            .collect();
        for &(_, col) in &moved {
            counts[col as usize + 1] += 1;
        }
        for k in 0..n {
            counts[k + 1] += counts[k];
        }
        let ap = counts.clone();
        let mut ai = vec![0u32; triplets.len()];
        let mut slot = vec![0u32; triplets.len()];
        for (t, &(row, col)) in moved.iter().enumerate() {
            let at = counts[col as usize];
            counts[col as usize] += 1;
            ai[at as usize] = row;
            slot[t] = at;
        }
        // The elimination tree and the size of each column of L.
        let mut parent = vec![-1i32; n];
        let mut flag = vec![-1i32; n];
        let mut lnz = vec![0u32; n];
        for k in 0..n {
            flag[k] = k as i32;
            for p in ap[k]..ap[k + 1] {
                let mut i = ai[p as usize] as usize;
                if i < k {
                    while flag[i] != k as i32 {
                        if parent[i] == -1 {
                            parent[i] = k as i32;
                        }
                        lnz[i] += 1;
                        flag[i] = k as i32;
                        i = parent[i] as usize;
                    }
                }
            }
        }
        let mut lp = vec![0u32; n + 1];
        for k in 0..n {
            lp[k + 1] = lp[k] + lnz[k];
        }
        let total = lp[n] as usize;
        let row_of_new: Vec<bool> = perm.iter().map(|&old| is_row[old as usize]).collect();
        Ldl {
            n,
            perm,
            ap,
            ai,
            slot,
            ax: vec![0.0; triplets.len()],
            parent,
            lp,
            li: vec![0; total],
            lx: vec![0.0; total],
            d: vec![0.0; n],
            is_row: row_of_new,
            lnz,
            y: vec![0.0; n],
            pattern: vec![0; n],
            flag,
            b: vec![0.0; n],
        }
    }

    /// Entries in the factor, a measure of how sparse this stayed.
    #[cfg(test)]
    pub fn factor_size(&self) -> usize {
        self.lp[self.n] as usize
    }

    /// Factors the matrix with these values, given in the order of the triplets. A pivot that is
    /// too small or has the wrong sign belongs to an equation dependent on others; it is replaced,
    /// which amounts to leaving that equation out of this step.
    pub fn factor(&mut self, values: &[f64]) {
        for (t, &v) in values.iter().enumerate() {
            self.ax[self.slot[t] as usize] = v;
        }
        const SMALLEST: f64 = 1e-13;
        let n = self.n;
        for k in 0..n {
            self.y[k] = 0.0;
            let mut top = n;
            self.flag[k] = k as i32;
            self.lnz[k] = 0;
            for p in self.ap[k]..self.ap[k + 1] {
                let mut i = self.ai[p as usize] as usize;
                self.y[i] += self.ax[p as usize];
                let mut len = 0;
                while self.flag[i] != k as i32 {
                    self.pattern[len] = i as u32;
                    len += 1;
                    self.flag[i] = k as i32;
                    i = self.parent[i] as usize;
                }
                while len > 0 {
                    top -= 1;
                    len -= 1;
                    self.pattern[top] = self.pattern[len];
                }
            }
            let mut dk = self.y[k];
            self.y[k] = 0.0;
            while top < n {
                let i = self.pattern[top] as usize;
                top += 1;
                let yi = self.y[i];
                self.y[i] = 0.0;
                let (start, count) = (self.lp[i] as usize, self.lnz[i] as usize);
                for p in start..start + count {
                    self.y[self.li[p] as usize] -= self.lx[p] * yi;
                }
                let l_ki = yi / self.d[i];
                dk -= l_ki * yi;
                self.li[start + count] = k as u32;
                self.lx[start + count] = l_ki;
                self.lnz[i] += 1;
            }
            self.d[k] = if self.is_row[k] {
                if dk > -SMALLEST {
                    -SMALLEST
                } else {
                    dk
                }
            } else if dk < SMALLEST {
                SMALLEST
            } else {
                dk
            };
        }
    }

    /// Solves in place, with `rhs` in the original node order.
    pub fn solve(&mut self, rhs: &mut [f64]) {
        let n = self.n;
        for k in 0..n {
            self.b[k] = rhs[self.perm[k] as usize];
        }
        for k in 0..n {
            let v = self.b[k];
            for p in self.lp[k] as usize..(self.lp[k] + self.lnz[k]) as usize {
                self.b[self.li[p] as usize] -= self.lx[p] * v;
            }
        }
        for k in 0..n {
            self.b[k] /= self.d[k];
        }
        for k in (0..n).rev() {
            let mut v = self.b[k];
            for p in self.lp[k] as usize..(self.lp[k] + self.lnz[k]) as usize {
                v -= self.lx[p] * self.b[self.li[p] as usize];
            }
            self.b[k] = v;
        }
        for k in 0..n {
            rhs[self.perm[k] as usize] = self.b[k];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    #[test]
    fn solves_random_quasi_definite_systems() {
        let mut rng = Rng::new(7);
        for _ in 0..200 {
            let vars = rng.int(1, 30) as usize;
            let rows = rng.int(1, vars as i64) as usize;
            let n = vars + rows;
            let mut trip = Vec::new();
            let mut vals = Vec::new();
            for v in 0..n {
                trip.push((v as u32, v as u32));
                vals.push(if v < vars { rng.range(0.5, 2.0) } else { -rng.range(1e-9, 1e-3) });
            }
            let mut dense = vec![vec![0.0; n]; n];
            for r in 0..rows {
                let mut seen = Vec::new();
                for _ in 0..rng.int(1, 5) {
                    let c = rng.int(0, vars as i64 - 1) as usize;
                    if seen.contains(&c) {
                        continue;
                    }
                    seen.push(c);
                    let value = rng.range(-1.0, 1.0);
                    trip.push((c as u32, (vars + r) as u32));
                    vals.push(value);
                    dense[c][vars + r] = value;
                    dense[vars + r][c] = value;
                }
            }
            for v in 0..n {
                dense[v][v] = vals[v];
            }
            let is_row = (0..n).map(|v| v >= vars).collect();
            let mut ldl = Ldl::new(n, &trip, is_row);
            ldl.factor(&vals);
            let truth: Vec<f64> = (0..n).map(|_| rng.range(-1.0, 1.0)).collect();
            let mut b: Vec<f64> = (0..n).map(|i| (0..n).map(|j| dense[i][j] * truth[j]).sum()).collect();
            let original = b.clone();
            ldl.solve(&mut b);
            // Compare residuals, since these matrices can be ill conditioned.
            let residual = (0..n)
                .map(|i| ((0..n).map(|j| dense[i][j] * b[j]).sum::<f64>() - original[i]).abs())
                .fold(0.0, f64::max);
            assert!(residual < 1e-7, "residual {residual}");
        }
    }

    #[test]
    fn a_shared_line_stays_sparse() {
        // One line's four variables used by 500 equations, each with four variables of its own.
        let shared = 4;
        let eqs = 500;
        let vars = shared + 4 * eqs;
        let n = vars + eqs;
        let mut trip = Vec::new();
        for v in 0..n {
            trip.push((v as u32, v as u32));
        }
        for e in 0..eqs {
            let row = (vars + e) as u32;
            for s in 0..shared {
                trip.push((s as u32, row));
            }
            for o in 0..4 {
                trip.push(((shared + 4 * e + o) as u32, row));
            }
        }
        let is_row = (0..n).map(|v| v >= vars).collect();
        let ldl = Ldl::new(n, &trip, is_row);
        // With J Jᵀ this would be a dense triangle of about 125,000 entries.
        assert!(ldl.factor_size() < 40_000, "{} entries", ldl.factor_size());
    }
}
