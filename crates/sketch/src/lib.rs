//! coreshape-sketch: a 2D sketch constraint solver with no dependencies.
//!
//! A [`Sketch`] is plain data: entities, constraints and dimensions. Solving takes a sketch and
//! returns a new one, so a failure can't leave anything half done, and a reason comes back that
//! names the constraints involved.
//!
//! ```
//! use coreshape_sketch::*;
//!
//! let mut s = Sketch::new();
//! let a = s.line([0.0, 0.0], [10.0, 1.0]);
//! s.constrain(Rule::Horizontal { a: a.whole(), b: None });
//! s.constrain(Rule::Distance { a: a.whole(), b: None, value: 25.0.into() });
//! s.constrain(Rule::Coincident { a: a.start(), b: ORIGIN.whole() });
//! let solved = s.solve().unwrap();
//! assert_eq!(solved.analysis.dof, 0);
//! ```

pub mod data;
pub mod drag;
pub mod expr;
pub mod fuzz;
pub mod json;
pub mod measure;
pub mod model;
pub mod regions;
pub mod rng;
pub mod solve;
pub mod spline;
pub mod workloads;

mod dual;
mod ldl;
mod system;

pub use data::*;
pub use drag::{DragReport, DragSession, Grab};
pub use model::*;
pub use regions::{Arrangement, Ignored, Loop, Region, RegionId, Seg, SegKind, WhyIgnored};
pub use solve::{Analysis, ErrorKind, Redundancy, SketchError, Solved};
pub use spline::SplineShape;
