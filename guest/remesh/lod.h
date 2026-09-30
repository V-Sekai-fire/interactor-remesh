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

} // namespace rm
