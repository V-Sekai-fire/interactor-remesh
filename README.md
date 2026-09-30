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
| `gate(planted_um)` | the sphere gate's verdict line |

`meshopt_remesh` is experimental in meshoptimizer 1.3 and writes positions
only: the host transfers attributes, skin weights and blendshapes back.

## Gate

A 0.1 m sphere at resolution 64 must come back within half a voxel
(1.562 mm). Measured on macOS arm64 through `sandbox_host.dylib`:

    PASS remesh 20588 tris worst 0.067 mm; simplify 18432 -> 2304 tris worst 0.000 mm
    FAIL ... worst 2.015 mm ... planted 2000 um

`tests/remesh/build.sh` builds the guest and runs both lines.
