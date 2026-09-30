// A mobile avatar variant's mesh and texture work (RFD 2284), host-neutral: a
// host only turns its meshes and textures into these arrays and back. Plain
// C++ with no sandbox API, so it runs in any godot-sandbox host or natively.
#pragma once
#include <cstdint>
#include <vector>

namespace rm {

// Per vertex, interleaved: position 3, normal 3, uv 2, bone weights 4.
constexpr int kStride = 12;

// A skinned mesh as flat arrays. shapes holds every blendshape's position
// deltas, shape-major: shapes[(s * vertex_count + v) * 3 + k].
struct Stream {
	std::vector<float> vertices;  // kStride per vertex
	std::vector<int32_t> bones;   // 4 per vertex, matching the weights
	std::vector<float> shapes;
	std::vector<unsigned> indices;
	std::vector<int32_t> ends;    // submesh s spans [ends[s-1], ends[s])
	size_t vertex_count() const { return vertices.size() / kStride; }
	size_t shape_count() const { return vertex_count() ? shapes.size() / (vertex_count() * 3) : 0; }
};

// Godot's quality metric across several meshes at once: each submesh's LOD
// chain (lod.h) and the smallest error in metres at which all of them fit
// target_triangles. Vertices are global across the meshes; the result keeps
// the submesh layout. errors_m holds each submesh's chosen level's error.
struct Budget {
	std::vector<unsigned> indices;
	std::vector<int32_t> ends;
	std::vector<float> errors_m;
	float threshold_m = 0;
	size_t triangles = 0;
};
Budget budget(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, const std::vector<int32_t> &ends, size_t target_triangles);

// Voxel remesh a whole mesh and carry every attribute back: normals, skin
// weights and blendshapes blend from each corner's closest source point; UVs
// come from the one source triangle under the new triangle's centroid, so no
// triangle straddles two UV islands (its vertices are therefore not shared).
// Each new triangle keeps the submesh of that source triangle.
Stream remesh_stream(const Stream &in, int resolution, size_t target_triangles, int *flipped = nullptr);

// Keep only referenced vertices: remap[old] is the new index or -1.
std::vector<int32_t> compact_remap(std::vector<unsigned> &indices, size_t vertex_count);

// Pack RGBA8 textures into one size x size atlas (shelf packing, halving
// every texture until they fit, 4 px padding with edges extended). rects
// holds x, y, w, h per texture in 0..1 atlas UV, row 0 at v = 0.
struct Atlas {
	std::vector<uint8_t> rgba;
	std::vector<float> rects;
	int scale_down = 0; // times every texture was halved to fit
};
Atlas atlas(const std::vector<uint8_t> &pixels, const std::vector<int32_t> &sizes, int size);

// Move each submesh's UVs into its texture's rect (tiled UVs wrap) and merge
// the submeshes into one. A vertex used by two submeshes is duplicated:
// picks[new] is the source vertex.
struct AtlasMesh {
	std::vector<int32_t> picks;
	std::vector<float> uv;
	std::vector<unsigned> indices;
	size_t wrapped = 0;
};
AtlasMesh atlas_mesh(const std::vector<float> &uv, const std::vector<unsigned> &indices, const std::vector<int32_t> &ends, const std::vector<int32_t> &texture_of_submesh, const std::vector<float> &rects);

// A blendshape that moves each vertex onto its heaviest bone's origin, inside
// the body, so a merged-away part can still be hidden.
std::vector<float> hide_shape(const std::vector<float> &positions, const std::vector<int32_t> &bones, const std::vector<float> &weights, const std::vector<float> &bone_origins);

} // namespace rm
