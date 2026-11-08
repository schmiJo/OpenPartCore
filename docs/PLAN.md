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

- Fillets cover straight and circular edges, plus sphere patches at convex three-edge corners.
  A radius that would swallow a neighbouring face is an error.
- Sketches are explicitly dimensioned profiles. There is no constraint solver.
- STEP is export only.
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
| M6 | Feature history with names that survive rebuilds | Done |
| M7 | Published demos and a known-failures list | Done, except testing on a real iPhone |

## Not planned yet

NURBS surfaces and Booleans on them, STEP import, variable-radius fillets, general vertex
blends, shelling and offsetting, a sketch constraint solver, assemblies, and drawings.
