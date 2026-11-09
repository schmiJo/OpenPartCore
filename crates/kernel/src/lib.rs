//! coreshape: a small exact B-rep CAD kernel with no dependencies.
//!
//! The rules that keep it small and robust:
//!
//! * Bodies are plain values. Every operation takes bodies and returns a new one.
//! * Closed surfaces are always split into faces of at most 180 degrees, and no
//!   edge is a closed loop, so seams and poles never reach an algorithm.
//! * Every operation validates what it returns. A failure is an `Err`, never a
//!   silently broken solid.

pub mod blend;
pub mod boolean;
pub mod build;
pub mod curve;
pub mod delete;
pub mod error;
pub mod export;
pub mod fuzz;
pub mod hash;
pub mod helix;
pub mod import;
pub mod loft;
pub mod mass;
pub mod math;
pub mod multi;
pub mod patch;
pub mod profile;
pub mod rolling;
pub mod spline;
pub mod surface;
pub mod tess;
pub mod thicken;
pub mod topo;
pub mod tweak;
pub mod validate;

pub use blend::{chamfer, chamfer_with, fillet, fillet_with, BlendOptions};
pub use boolean::{intersect, subtract, union, BoolOp};
pub use build::{box_solid, cone_frustum, cylinder, extrude, revolve, sphere, torus};
pub use curve::Curve;
pub use delete::delete_faces;
pub use error::{Error, Result};
pub use helix::{helix, Helix};
pub use loft::loft;
pub use mass::{mass_properties, MassProps};
pub use math::{v2, v3, Aabb, Frame, Mirror, Rng, Vec2, Vec3, MAX_COORD, MIN_FEATURE, PI, TAU, TOL};
pub use profile::{PathBuilder, Profile, Seg, SegGeom, Wire};
pub use multi::{boolean_many, compound, merged_tools, sweep, PathStep};
pub use spline::{FacePiece, PlaneCurve, Spline};
pub use surface::{Chart, Surface};
pub use tess::{tessellate, Mesh, TessParams};
pub use thicken::thicken;
pub use topo::{Body, Coedge, Edge, Face, FaceTag};
pub use tweak::{draft, offset, offset_faces, shell};
pub use validate::{validate, Report};
