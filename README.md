# interactor-remesh

A remesher, simplifier and level-of-detail builder for avatar meshes that runs as a sandbox guest and works on flat arrays.

## What it is for

`remesh.elf` takes positions, indices and textures as flat arrays and returns new ones: a voxel remesh, a simplified mesh, a level-of-detail chain with its error in metres, a texture atlas, and the skin weights and blendshapes carried back onto the new surface. It has no filesystem, sockets or host objects, so any editor with the sandbox host runs it unchanged. It is the remesh stage of RFD 2284's mobile avatar variant, and it runs meshoptimizer at the revision `guest/remesh/CITATION.cff` pins.

## Build and test

The build script compiles the guest with the riscv64 toolchain and, given the sandbox host's probe, runs each gate clean and against its planted defect. Its header names the paths it reads from the environment.

```sh
pixi exec -s clangxx -s lld -s cmake -s ninja -- tests/remesh/build.sh
```

## Licence

MIT; see LICENSE.
