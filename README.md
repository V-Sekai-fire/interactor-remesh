# interactor-remesh

meshoptimizer's voxel remesher and simplifier as a godot-sandbox guest, the remesh stage of RFD 2284's Quest variant.

`remesh.elf` takes positions and indices and returns arrays; it has no
filesystem, sockets or host objects. It runs unmodified in Godot and in the
Unity Editor's sandbox host.

| Function | Returns |
|---|---|
| `version()` | meshoptimizer version and manifest pin |
| `remesh(positions, indices, resolution, options, target_triangles)` | `[positions, indices, error]`: voxel remesh, weld, simplify; positions only |
| `simplify(positions, indices, attributes, weights, target_index_count, target_error, options)` | `[indices, error]` into the original vertices |
| `closest(src_positions, src_indices, query)` | `[triangles, barycentrics]`: the closest source surface point per query, for transferring attributes back |
| `alpha_cull(uvs, indices, alpha, width, height, threshold)` | the indices of triangles whose texture is not mostly transparent |
| `lod_chain(positions, normals, indices, deformable)` | `[indices, error_m, ...]`: Godot's LOD chain, each level with its error in metres |
| `gate`, `gate_transfer`, `gate_alpha`, `gate_lod` | each gate's verdict line |

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

`tests/remesh/build.sh` builds the guest and runs all eight.

## Pins

| Dependency | Revision |
|---|---|
| `contract-guest-runtime` (`vendor/sandbox-api`) | `22cdad11236c28fadb30c856271e2efb8a774395` (tree `5ec3b43904`) |
| `meshoptimizer` | `9e1f07b159d3cb777f1c67ed31fc11fd117986f4` |
