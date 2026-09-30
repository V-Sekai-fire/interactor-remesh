// A Godot-style LOD chain (ImporterMesh::generate_lods at godot 97dab7a638):
// each level halves the previous one with meshopt_simplifyWithAttributes on the
// normals, and carries meshoptimizer's error, made monotonic and converted to
// metres with meshopt_simplifyScale. Plain C++ with no sandbox API.
#pragma once
#include <vector>

namespace rm {

struct Lod {
	std::vector<unsigned> indices; // into the original vertices
	float error;                   // relative to the mesh's extent
	float error_m;                 // error * meshopt_simplifyScale, in metres
};

// Level 0 is the input itself, with zero error.
std::vector<Lod> lod_chain(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, bool deformable);

// Levels for choosing one error across many meshes. Godot's metric (the same
// attributes and options, result_error times meshopt_simplifyScale), but every
// level is simplified from the input rather than from the previous level, and
// the target steps by `ratio`: each error is then the true error against the
// source, not Godot's 1.5x-per-step bound meant for monotonic LOD switching.
std::vector<Lod> lod_levels(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, bool deformable, float ratio = 0.8f);

} // namespace rm
