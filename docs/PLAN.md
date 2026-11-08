# Plan and status

## Choices

**Language: Rust.** One codebase for native and WebAssembly, with no C++ toolchain.

**Representation: exact B-rep.** It is the only option that keeps exact dimensions, real STEP
and true fillet surfaces, and it is the hardest to make robust. "Exact" means the geometry
definitions are exact. Arithmetic is `f64` with a 1e-7 mm tolerance, and unequal-cylinder
intersections are stored as refined samples, as other kernels also do.

**Reuse.** The kernel is our own, with no runtime dependencies. Ideas came from Manifold
(derive every topological decision from one consistent set of predicates) and from classic
kernels (closed-form intersection tables). OpenCASCADE is used only as an independent test
oracle, through a neighbouring FreeCAD build, read-only.

| Project | Why not |
|---|---|
| Fornjot | Archived June 2026; its README says its goals were never reached |
| truck | Closest match, but using it would mean the kernel isn't ours |
| opencascade-rs | C++ dependency, LGPL-2.1, no WebAssembly claim |
| Manifold, csgrs | Meshes: no exact geometry and no real STEP |

## Scope changes from the brief

- Fillets are exact on straight and circular edges, with sphere patches at convex three-edge corners, and fitted to a stated tolerance on any other edge.
  A radius that would swallow a neighbouring face is an error.
- The `sketch` feature takes explicitly dimensioned profiles, and `constrained_sketch` adds the constraint solver, which is a crate of its own.
- STEP import reads analytic surfaces, swept curves and free-form surfaces without poles. Assemblies are refused.
- Two bodies touching only along an edge or at a point become a valid manifold result with
  duplicated topology. `Body::contact_vertices` reports it.

## Milestones

| # | Work | Status |
|---|---|---|
| M1 | Geometry, topology, validator, extrude, revolve, mass properties, mesh, STL, viewer | Done |
| M2 | Booleans on planes, with coplanar and touching contacts, and fuzzing | Done |
| M3 | Booleans on cylinders, cones, spheres and tori | Done |
| M4 | STEP export, checked in OpenCASCADE | Done |
| M5 | Chamfers and fillets | Done, with the limits listed in `KNOWN_LIMITS.md` |
| M6 | Feature history with names that survive rebuilds, as saveable data | Done |
| M7 | Published demos and a known-failures list | Done. Used on a real iPhone, but not measured there |
| M8 | Sketch constraint solver, checked against PlaneGCS | Done, see `SKETCH_SOLVER.md` |
| M9 | MCP server and offscreen rendering | Done, except connecting a real MCP client, see `MCP.md` |
| M10 | Multi-tool Booleans, patterns, mirror, sweep, offset, shell, draft | Done, with the limits in `KNOWN_LIMITS.md` |
| M11 | Sketch regions, distances to curves, reference dimensions, ellipses | Done, see `SKETCH_SOLVER.md` |
| M12 | Splines in sketches, extruded and revolved (phases 1 and 2 of `NURBS_DESIGN.md`) | Done, with the limits in `KNOWN_LIMITS.md`. Phase 3 isn't decided |
| M13 | STEP import of analytic surfaces and swept curves | Done, checked on OpenCASCADE's files only |
| M14 | Free-form surfaces and lofts, read from and written to STEP (phase 3 of `NURBS_DESIGN.md`) | Done, with the limits in `KNOWN_LIMITS.md` |
| M15 | Fitted fillets and chamfers on traced, elliptical, spline and free-form edges | Done, with the limits in `KNOWN_LIMITS.md` |
| M16 | Tolerant STEP import, surfaces with poles, checked on NIST's test files | Done: 30 of 33 read, with the limits in `KNOWN_LIMITS.md` |

## For code building on this repository

**Getting a known-good state.** The tag `openshape-ok` marks the newest commit whose workspace tests all pass, and it moves
forward. `main`, and so the main working copy, only advances to such commits. Work in progress happens on the `next` branch in
`.worktrees/next`, so the main working copy never holds half-finished edits.

**Kept stable:** the sketch JSON format, `profiles_from_regions`, `FaceTag` slot numbering, and the `Body` edge and face accessors.

**Breaking changes,** newest first:

| Commit | Change | What to do |
|---|---|---|
| Tolerant import | `coreshape::topo::Body` gained `tolerance` | Reading is unaffected, and zero means the kernel's own. Code building a `Body` by hand without `..Body::default()` has to set it |
| Free-form surfaces | `Surface` gained `Free`, and `coreshape_document::Op` gained `Loft`. `Surface::name` gives `free-form surface`. `spline::MAX_DEGREE` went from 7 to 16 in the kernel, and stays 7 in the sketch solver | An exhaustive `match` needs the new arms. Nothing else changes for code that doesn't use them |
| Face keys | `coreshape::topo::Face` gained `origin`, and `EdgeSelect` gained `OnFaceKey` and `BetweenKeys`. Names, tags, slots, `FaceRef` and files are unchanged | Reading is unaffected. Code building a `Face` by hand has to set `origin`, and an exhaustive `match` on `EdgeSelect` needs the new arms |
| STEP import | `coreshape_document::Op` gained `ImportStep` | An exhaustive `match` needs the new arm |
| Document shapes | `coreshape_document::Shape` gained `Ellipse` and `Move` gained `Spline` | An exhaustive `match` needs the new arms |
| Sketch splines | `Geometry`, `SegKind` and `coreshape_sketch::Seg` are no longer `Copy`. `Geometry` gained `Spline`, `SegKind` gained `Spline`, and `Part` gained `Control` | Clone them, or match on a reference. An exhaustive `match` needs the new arms. Existing sketch JSON is unchanged |
| Ellipses | `Geometry` gained `Ellipse` and `EllipticalArc`, `SegKind` gained `Ellipse`, `Part` gained `Major`, and `Rule` gained `MajorRadius` and `MinorRadius` | An exhaustive `match` needs the new arms. Existing sketch JSON is unchanged. A region's loop can now hold an ellipse, which `profiles_from_regions` handles |
| Spline surfaces | `Surface` is no longer `Copy`, and gained `Extruded` and `Revolved` | Use `face.surface.clone()` or borrow it. An exhaustive `match` needs the new arms. `Surface::name` gives `extrusion` and `revolution` |
| Spline surfaces | `Curve` gained `Spline`. `SegGeom` gained `Spline`, and it and `Seg` are no longer `Copy` | An exhaustive `match` needs the new arm |
| Spline surfaces | `MassProps` gained `volume_error` | Reading it is unaffected. It's zero unless the body has spline faces or traced edges |
| Solver dimensions | `coreshape_sketch::Constraint` gained `reference` | Reading it is unaffected. `Sketch::constrain` is still the way to add one. Code building a `Constraint` by hand has to set it. Sketch JSON is unchanged unless a dimension is a reference |
| Sketch regions | `coreshape_sketch::Region` gained `id` and `depth` | Reading it is unaffected. Code that builds a `Region` by hand has to set them |
| Sketch regions | `Sketch::regions` no longer refuses open curves, branches or crossings. It refuses only if no region results, or a picked one has gone | A sketch that used to fail may now build. `Sketch::arrangement` and `Document::sketch_arrangement` report what was left out |
| 9d13746 | Chamfers follow tangent edges by default, like fillets, including in saved files. `"propagate": false` turns it off | A saved chamfer on a smooth chain may now cover more edges |

## Not planned yet

Variable-radius fillets, general vertex blends, assemblies, and drawings.

## Next, in order

From OpenShape's list, in its order:

1. Sketch regions: done, see [SKETCH_SOLVER.md](SKETCH_SOLVER.md).
2. Solver dimensions: done, see [SKETCH_SOLVER.md](SKETCH_SOLVER.md).
3. Ellipses and elliptical arcs in the solver, regions and kernel profiles: done, see [SKETCH_SOLVER.md](SKETCH_SOLVER.md).
4. Splines in sketches, extruded and revolved, then lofts and free-form surfaces from imported files: done, see [NURBS_DESIGN.md](NURBS_DESIGN.md).
5. STEP import of analytic surfaces and swept curves: done, see [KNOWN_LIMITS.md](KNOWN_LIMITS.md). It refuses anything else, naming the surface type. Assemblies come next there.
6. Modelling: offset, shell and draft that survive topology changes, helix, delete face and thicken.
