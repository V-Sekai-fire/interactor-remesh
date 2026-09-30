# interactor-remesh

meshoptimizer's voxel remesher and simplifier as a godot-sandbox guest, the remesh stage of RFD 2284's mobile avatar variant.

`remesh.elf` takes positions and indices and returns arrays; it has no
filesystem, sockets or host objects. It runs unmodified in Godot and in the
sandbox host of any other game engine's editor.

| Function | Returns |
|---|---|
| `version()` | meshoptimizer version and manifest pin |
| `remesh(positions, indices, resolution, options, target_triangles)` | `[positions, indices, error]`: voxel remesh, weld, simplify; positions only |
| `simplify(positions, indices, attributes, weights, target_index_count, target_error, options)` | `[indices, error]` into the original vertices |
| `closest(src_positions, src_indices, query)` | `[triangles, barycentrics]`: the closest source surface point per query, for transferring attributes back |
| `alpha_cull(uvs, indices, alpha, width, height, threshold)` | the indices of triangles whose texture is not mostly transparent |
| `lod_chain(positions, normals, indices, deformable)` | `[indices, ends, errors_m]`: Godot's LOD chain, levels back to back with each level's error in metres |
| `avatar_budget(positions, normals, indices, ends, target_triangles)` | `[indices, ends, errors_m, threshold_m, triangles]`: one error threshold in metres across every submesh of every mesh |
| `avatar_remesh(vertices, bones, shapes, indices, ends, resolution, target_triangles)` | the remeshed mesh with normals, UVs, skin weights and blendshapes carried back |
| `avatar_compact(indices, vertex_count)` | `[remap, indices]` dropping unreferenced vertices |
| `avatar_atlas(pixels, sizes, size)` | `[rgba, rects, scale_down]`: RGBA8 textures shelf-packed into one atlas |
| `avatar_atlas_mesh(uv, indices, ends, texture_of_submesh, rects)` | `[picks, uv, indices, wrapped]`: UVs moved into the atlas, submeshes merged |
| `avatar_hide_shape(positions, bones, weights, bone_origins)` | deltas collapsing each vertex onto its heaviest bone |
| `gate`, `gate_transfer`, `gate_alpha`, `gate_lod`, `gate_avatar` | each gate's verdict line |

The `avatar_*` calls make every decision for a host that builds a mobile
avatar variant: the host only turns its meshes and textures into flat
arrays and back. A vertex travels as 12 floats (position, normal, uv, four
bone weights) with four bone indices beside it; blendshapes are position
deltas, shape-major.

`meshopt_remesh` is experimental in meshoptimizer 1.3 and writes positions
only: the host transfers attributes, skin weights and blendshapes back.

`lod_chain` follows Godot's `ImporterMesh::generate_lods` (godot `97dab7a638`):
normals as attributes at weight 1, `SPARSE | LOCK_BORDER | PRUNE`, plus
`REGULARIZE` for deformable meshes; each level halves the last, the error is
kept monotonic with `max(error * 1.5, step)`, and it stops past a relative
error of 1.0 or when a step removes less than a quarter. `meshopt_simplifyScale`
turns the relative error into metres, so one threshold compares any meshes.
Unlike Godot it does not pre-merge vertices by normal angle; meshoptimizer
already treats co-located vertices as seams.

## Gates

Measured on macOS arm64 through `sandbox_host.dylib`. Each planted defect
must fail.

| Gate | Clean | Planted |
|---|---|---|
| Sphere remesh, 0.1 m at resolution 64, tolerance 1.562 mm (half a voxel) | PASS, worst 0.067 mm | 2 mm shift: FAIL, 2.015 mm |
| Surface transfer, 659 samples on a sphere mesh, tolerance 0.01 mm | PASS, worst 0.0000 mm | 0.5 mm tangential shift: FAIL, 0.4987 mm |
| Alpha cull, a 32 x 32 plane transparent for u < 0.5 | PASS, 1024 of 2048 kept | opaque texture: FAIL, 2048 kept |
| LOD chain, 0.1 m sphere, 18432 tris in 6 levels to 576; sag within 2x the reported error | PASS, e.g. 2304 tris reported 0.457 mm, measured 0.857 mm | errors reported 10x low: FAIL |
| Atlas, three solid textures in 64 px: rects apart, each colour at its centre | PASS | rect moved onto another: FAIL |
| Remesh transfer, 0.1 m sphere with a 1 cm normal blendshape and height-split weights | PASS, shape 0.012 mm, weights exact | deltas doubled: FAIL, 9.98 mm |
| Shared budget, spheres of 0.1 m and 0.05 m in 4000 tris | PASS, 2048 tris at one 4.18 mm threshold | threshold halved: FAIL, 8192 tris |

`tests/remesh/build.sh` builds the guest and runs all fourteen.

## Pins

| Dependency | Revision |
|---|---|
| `contract-guest-runtime` (`vendor/sandbox-api`) | `22cdad11236c28fadb30c856271e2efb8a774395` (tree `5ec3b43904`) |
| `meshoptimizer` | `9e1f07b159d3cb777f1c67ed31fc11fd117986f4` |
