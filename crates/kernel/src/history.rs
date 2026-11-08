//! The parametric layer: a feature history that rebuilds when a dimension changes.
//!
//! Each feature's inputs are functions of the parameter list, so any dimension can
//! depend on any parameter. Features refer to earlier geometry by name, never by
//! index: a face is named by the feature that made it and a slot in that feature, and
//! Booleans preserve those names. So "the edge between the top cap of Extrude 1 and
//! side 3 of Extrude 1" still means the same edge after a dimension change, even when
//! the edge has been split or renumbered by features in between.
//!
//! Rebuilding reuses every step whose inputs did not change, so dragging the last
//! feature's slider doesn't recompute the rest.

use crate::blend::{chamfer, fillet};
use crate::boolean::{boolean_with, intersect, union, BoolOp, Checks};
use crate::build::{extrude, revolve, SLOT_END_CAP, SLOT_SEG_BASE, SLOT_START_CAP};
use crate::error::{Error, Result};
use crate::math::Frame;
use crate::profile::Profile;
use crate::topo::{Body, FaceTag};
use std::sync::Arc;

type Num = Arc<dyn Fn(&[f64]) -> f64 + Send + Sync>;
type FrameFn = Arc<dyn Fn(&[f64]) -> Frame + Send + Sync>;
type ProfilesFn = Arc<dyn Fn(&[f64]) -> Result<Vec<Profile>> + Send + Sync>;
type EdgeFilter = Arc<dyn Fn(&Body, usize, &[f64]) -> bool + Send + Sync>;

/// How a new solid combines with the body so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Add,
    Remove,
    Intersect,
}

/// A stable name for a face: a feature in the history and a slot in that feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceRef {
    pub feature: FeatureId,
    pub slot: u32,
}

impl FaceRef {
    fn tag(self) -> FaceTag {
        FaceTag {
            feature: self.feature.0 as u32 + 1,
            slot: self.slot,
        }
    }
}

/// Handle to a feature, returned when it is added.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FeatureId(pub usize);

/// Slot offset per closed region of a sketch.
const REGION_SLOTS: u32 = 100_000;

impl FeatureId {
    /// The cap an extrude or revolve starts from.
    pub fn start_cap(self) -> FaceRef {
        FaceRef {
            feature: self,
            slot: SLOT_START_CAP,
        }
    }

    /// The cap an extrude or revolve ends at.
    pub fn end_cap(self) -> FaceRef {
        FaceRef {
            feature: self,
            slot: SLOT_END_CAP,
        }
    }

    /// The face swept from a segment of the sketch.
    pub fn side(self, segment: u32) -> FaceRef {
        FaceRef {
            feature: self,
            slot: SLOT_SEG_BASE + segment,
        }
    }

    /// The face swept from a segment of one region of a sketch with several regions.
    pub fn side_in_region(self, region: u32, segment: u32) -> FaceRef {
        FaceRef {
            feature: self,
            slot: SLOT_SEG_BASE + segment + region * REGION_SLOTS,
        }
    }
}

/// Which edges a blend applies to, in terms that survive a rebuild.
#[derive(Clone)]
pub enum EdgeQuery {
    /// Edges between two named faces.
    Between(FaceRef, FaceRef),
    /// All edges bounding a named face.
    OnFace(FaceRef),
    /// Edges passing a geometric test against the current body.
    Where(EdgeFilter),
    /// The union of several queries.
    Any(Vec<EdgeQuery>),
}

impl EdgeQuery {
    pub fn filter(f: impl Fn(&Body, usize, &[f64]) -> bool + Send + Sync + 'static) -> EdgeQuery {
        EdgeQuery::Where(Arc::new(f))
    }

    fn resolve(&self, body: &Body, params: &[f64], out: &mut Vec<usize>) {
        match self {
            EdgeQuery::Between(a, b) => out.extend(body.edges_between_tags(a.tag(), b.tag())),
            EdgeQuery::OnFace(f) => {
                let tag = f.tag();
                for face in &body.faces {
                    if face.tag == tag {
                        out.extend(face.loops.iter().flatten().map(|c| c.edge));
                    }
                }
            }
            EdgeQuery::Where(pred) => {
                out.extend((0..body.edges.len()).filter(|&e| pred(body, e, params)));
            }
            EdgeQuery::Any(list) => {
                for q in list {
                    q.resolve(body, params, out);
                }
            }
        }
    }
}

#[derive(Clone)]
enum Op {
    Sketch {
        frame: FrameFn,
        profiles: ProfilesFn,
    },
    Extrude {
        sketch: FeatureId,
        start: Num,
        length: Num,
        mode: Mode,
    },
    Revolve {
        sketch: FeatureId,
        angle: Num,
        mode: Mode,
    },
    Fillet {
        edges: EdgeQuery,
        radius: Num,
    },
    Chamfer {
        edges: EdgeQuery,
        distance: Num,
    },
}

#[derive(Clone)]
struct Feature {
    name: String,
    op: Op,
}

/// What became of one feature in a rebuild.
#[derive(Clone, Debug, PartialEq)]
pub enum StepState {
    /// Computed in this rebuild.
    Rebuilt,
    /// Inputs unchanged, so the previous result was reused.
    Reused,
    /// Past the rollback position.
    RolledBack,
    /// This feature failed.
    Failed(String),
    /// An earlier feature failed, so this one did not run.
    Blocked,
}

#[derive(Clone, Debug)]
pub struct Step {
    pub name: String,
    pub kind: &'static str,
    pub state: StepState,
}

/// The outcome of a rebuild.
#[derive(Clone, Debug)]
pub struct Outcome {
    pub steps: Vec<Step>,
    /// The body at the rollback position, or at the last feature that succeeded.
    pub body: Option<Arc<Body>>,
    /// Index of the failed feature, if any.
    pub failed: Option<usize>,
}

/// What a feature produced last time, and the inputs that produced it.
#[derive(Clone)]
struct Cached {
    /// Exact text of this feature's resolved inputs.
    signature: String,
    /// Body after this feature.
    body: Option<Arc<Body>>,
    /// A sketch's output; empty for others.
    sketch: Option<(Frame, Arc<Vec<Profile>>)>,
}

/// An ordered list of features, with results cached between rebuilds.
#[derive(Clone, Default)]
pub struct History {
    features: Vec<Feature>,
    cache: Vec<Option<Cached>>,
}

fn num(f: impl Fn(&[f64]) -> f64 + Send + Sync + 'static) -> Num {
    Arc::new(f)
}

impl History {
    pub fn new() -> History {
        History::default()
    }

    fn push(&mut self, kind: &str, op: Op) -> FeatureId {
        // Named the way people read them: Sketch 1, Extrude 2, Fillet 1.
        let count = self.features.iter().filter(|f| f.name.starts_with(kind)).count();
        self.features.push(Feature {
            name: format!("{kind} {}", count + 1),
            op,
        });
        self.cache.push(None);
        FeatureId(self.features.len() - 1)
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }

    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// A sketch on a plane. It may hold several closed regions.
    pub fn sketch(
        &mut self,
        frame: impl Fn(&[f64]) -> Frame + Send + Sync + 'static,
        profiles: impl Fn(&[f64]) -> Result<Vec<Profile>> + Send + Sync + 'static,
    ) -> FeatureId {
        self.push(
            "Sketch",
            Op::Sketch {
                frame: Arc::new(frame),
                profiles: Arc::new(profiles),
            },
        )
    }

    /// Extrudes a sketch along its normal, starting `start` from the sketch plane and
    /// running for `length`.
    pub fn extrude(
        &mut self,
        sketch: FeatureId,
        start: impl Fn(&[f64]) -> f64 + Send + Sync + 'static,
        length: impl Fn(&[f64]) -> f64 + Send + Sync + 'static,
        mode: Mode,
    ) -> FeatureId {
        self.push(
            "Extrude",
            Op::Extrude {
                sketch,
                start: num(start),
                length: num(length),
                mode,
            },
        )
    }

    /// Revolves a sketch about its frame's Z axis. Profile x is the distance from the
    /// axis and profile y the height along it.
    pub fn revolve(&mut self, sketch: FeatureId, angle: impl Fn(&[f64]) -> f64 + Send + Sync + 'static, mode: Mode) -> FeatureId {
        self.push(
            "Revolve",
            Op::Revolve {
                sketch,
                angle: num(angle),
                mode,
            },
        )
    }

    pub fn fillet(&mut self, edges: EdgeQuery, radius: impl Fn(&[f64]) -> f64 + Send + Sync + 'static) -> FeatureId {
        self.push(
            "Fillet",
            Op::Fillet {
                edges,
                radius: num(radius),
            },
        )
    }

    pub fn chamfer(&mut self, edges: EdgeQuery, distance: impl Fn(&[f64]) -> f64 + Send + Sync + 'static) -> FeatureId {
        self.push(
            "Chamfer",
            Op::Chamfer {
                edges,
                distance: num(distance),
            },
        )
    }

    /// Rebuilds through feature `upto` (all of them when `None`), reusing every step
    /// whose inputs did not change. A failed feature keeps the last good body available
    /// and blocks the features after it.
    pub fn rebuild(&mut self, params: &[f64], upto: Option<usize>) -> Outcome {
        let n = self.features.len();
        let last = upto.map_or(n, |u| (u + 1).min(n));
        let mut steps: Vec<Step> = Vec::with_capacity(n);
        let mut body: Option<Arc<Body>> = None;
        let mut failed: Option<usize> = None;
        // Once the body changes, everything after it depends on the change.
        let mut body_changed = false;
        let mut sketch_changed = vec![false; n];

        for i in 0..n {
            let feature = self.features[i].clone();
            let kind = kind_of(&feature.op);
            if i >= last {
                steps.push(Step {
                    name: feature.name,
                    kind,
                    state: StepState::RolledBack,
                });
                continue;
            }
            if failed.is_some() {
                self.cache[i] = None;
                steps.push(Step {
                    name: feature.name,
                    kind,
                    state: StepState::Blocked,
                });
                continue;
            }
            let signature = signature_of(&feature.op, params);
            let upstream_sketch = match &feature.op {
                Op::Extrude { sketch, .. } | Op::Revolve { sketch, .. } => sketch_changed[sketch.0],
                _ => false,
            };
            // A sketch depends only on the parameters, never on the body.
            let is_sketch = matches!(feature.op, Op::Sketch { .. });
            let reusable = (is_sketch || !body_changed)
                && !upstream_sketch
                && self.cache[i].as_ref().is_some_and(|c| c.signature == signature);
            if reusable {
                if !is_sketch {
                    body = self.cache[i].as_ref().unwrap().body.clone();
                }
                steps.push(Step {
                    name: feature.name,
                    kind,
                    state: StepState::Reused,
                });
                continue;
            }
            match self.run_step(i, &feature.op, params, body.clone()) {
                Ok((new_body, sketch)) => {
                    match &feature.op {
                        Op::Sketch { .. } => sketch_changed[i] = true,
                        _ => {
                            body = new_body.clone();
                            body_changed = true;
                        }
                    }
                    self.cache[i] = Some(Cached {
                        signature,
                        body: body.clone(),
                        sketch,
                    });
                    steps.push(Step {
                        name: feature.name,
                        kind,
                        state: StepState::Rebuilt,
                    });
                }
                Err(e) => {
                    self.cache[i] = None;
                    failed = Some(i);
                    steps.push(Step {
                        name: feature.name,
                        kind,
                        state: StepState::Failed(e.to_string()),
                    });
                }
            }
        }
        Outcome { steps, body, failed }
    }

    #[allow(clippy::type_complexity)]
    fn run_step(
        &self,
        index: usize,
        op: &Op,
        params: &[f64],
        body: Option<Arc<Body>>,
    ) -> Result<(Option<Arc<Body>>, Option<(Frame, Arc<Vec<Profile>>)>)> {
        let feature = index as u32 + 1;
        let sketch_output = |id: &FeatureId| -> Result<(Frame, Arc<Vec<Profile>>)> {
            self.cache[id.0]
                .as_ref()
                .and_then(|c| c.sketch.clone())
                .ok_or_else(|| Error::InvalidInput("this feature's sketch did not build".into()))
        };
        match op {
            Op::Sketch { frame, profiles } => {
                let list = profiles(params)?;
                for p in &list {
                    p.validate()?;
                }
                Ok((body, Some((frame(params), Arc::new(list)))))
            }
            Op::Extrude {
                sketch,
                start,
                length,
                mode,
            } => {
                let (frame, list) = sketch_output(sketch)?;
                let shifted = Frame {
                    origin: frame.origin + frame.z * start(params),
                    ..frame
                };
                let h = length(params);
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| Ok(with_region(extrude(p, &shifted, h, feature)?, k as u32)))
                    .collect::<Result<Vec<Body>>>()?;
                Ok((Some(Arc::new(combine(body.as_deref(), tools, *mode)?)), None))
            }
            Op::Revolve { sketch, angle, mode } => {
                let (frame, list) = sketch_output(sketch)?;
                let a = angle(params);
                let tools = list
                    .iter()
                    .enumerate()
                    .map(|(k, p)| Ok(with_region(revolve(p, &frame, a, feature)?, k as u32)))
                    .collect::<Result<Vec<Body>>>()?;
                Ok((Some(Arc::new(combine(body.as_deref(), tools, *mode)?)), None))
            }
            Op::Fillet { edges, radius } => {
                let current = body.ok_or_else(|| Error::InvalidInput("a fillet needs a body to work on".into()))?;
                let list = resolve_edges(edges, &current, params)?;
                Ok((Some(Arc::new(fillet(&current, &list, radius(params), feature)?)), None))
            }
            Op::Chamfer { edges, distance } => {
                let current = body.ok_or_else(|| Error::InvalidInput("a chamfer needs a body to work on".into()))?;
                let list = resolve_edges(edges, &current, params)?;
                Ok((Some(Arc::new(chamfer(&current, &list, distance(params), feature)?)), None))
            }
        }
    }
}

fn kind_of(op: &Op) -> &'static str {
    match op {
        Op::Sketch { .. } => "sketch",
        Op::Extrude { .. } => "extrude",
        Op::Revolve { .. } => "revolve",
        Op::Fillet { .. } => "fillet",
        Op::Chamfer { .. } => "chamfer",
    }
}

/// The feature's resolved numeric inputs, as text with exact float bits.
fn signature_of(op: &Op, params: &[f64]) -> String {
    match op {
        Op::Sketch { frame, profiles } => format!("{:?} {:?}", frame(params), profiles(params)),
        Op::Extrude { start, length, .. } => format!("{:x} {:x}", start(params).to_bits(), length(params).to_bits()),
        Op::Revolve { angle, .. } => format!("{:x}", angle(params).to_bits()),
        Op::Fillet { radius, .. } => format!("{:x}", radius(params).to_bits()),
        Op::Chamfer { distance, .. } => format!("{:x}", distance(params).to_bits()),
    }
}

fn with_region(mut body: Body, region: u32) -> Body {
    if region > 0 {
        for f in &mut body.faces {
            if f.tag.slot >= SLOT_SEG_BASE {
                f.tag.slot += region * REGION_SLOTS;
            }
        }
    }
    body
}

fn combine(body: Option<&Body>, tools: Vec<Body>, mode: Mode) -> Result<Body> {
    let mut tools = tools.into_iter();
    let mut current = match body {
        Some(b) => b.clone(),
        None => {
            if mode != Mode::Add {
                return Err(Error::InvalidInput("there is no body yet to cut from".into()));
            }
            tools
                .next()
                .ok_or_else(|| Error::InvalidInput("the sketch has no closed regions".into()))?
        }
    };
    match mode {
        // Several regions apply one by one. Only the last result needs the full checks.
        Mode::Add | Mode::Remove => {
            let op = if mode == Mode::Add { BoolOp::Union } else { BoolOp::Subtract };
            let total = tools.len();
            for (i, t) in tools.enumerate() {
                let checks = if i + 1 == total { Checks::Full } else { Checks::Interim };
                current = boolean_with(&current, &t, op, checks)?;
            }
        }
        Mode::Intersect => {
            // One tool at a time would keep only what is common to all of them. A
            // sketch with several regions means their union.
            let mut tool: Option<Body> = None;
            for t in tools {
                tool = Some(match tool {
                    None => t,
                    Some(existing) => union(&existing, &t)?,
                });
            }
            if let Some(t) = tool {
                current = intersect(&current, &t)?;
            }
        }
    }
    Ok(current)
}

fn resolve_edges(query: &EdgeQuery, body: &Body, params: &[f64]) -> Result<Vec<usize>> {
    let mut list = Vec::new();
    query.resolve(body, params, &mut list);
    list.sort_unstable();
    list.dedup();
    if list.is_empty() {
        return Err(Error::InvalidInput(
            "the edges this feature refers to no longer exist in the model".into(),
        ));
    }
    Ok(list)
}
