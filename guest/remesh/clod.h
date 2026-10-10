// Cluster level of detail (meshoptimizer's demo/clusterlod.h at the manifest pin): a DAG of
// clusters, each group simplified from the group below with an error in mesh units. Any cut that
// follows the selection rule is free of cracks, so a host can draw near clusters fine and far ones
// coarse within one mesh.
#pragma once
#include <cstdint>
#include <vector>

namespace rm {

struct ClodFlags {
	enum : int64_t {
		Dilate = 1, // dilate open borders of simplified clusters (foliage); positions are copied per cluster
		Protect = 2, // permissive simplification protects every attribute seam
		NoSloppy = 4, // no sloppy fallback when regular simplification gets stuck
		NoPermissive = 8, // regular (not permissive) simplification
	};
};

struct Clod {
	// Every cluster's indices back to back. Without Dilate they index the input vertices; with it,
	// they index `positions`, whose vertex k came from input vertex `source[k]`.
	std::vector<int32_t> indices;
	// Per cluster: group, refined (-1 for input geometry), index start, index count.
	std::vector<int32_t> clusters;
	// Per cluster: bounds sphere centre xyz, radius (culling only).
	std::vector<float> cluster_bounds;
	// Per group: depth, simplified sphere centre xyz, radius, error (-1 for terminal groups).
	std::vector<float> groups;
	std::vector<float> positions;
	std::vector<int32_t> source;
};

Clod cluster_lod(const std::vector<float> &positions, const std::vector<float> &attributes, int attribute_count,
		const std::vector<unsigned> &indices, int max_triangles, int64_t flags);

// The cut for one world-space error threshold, ignoring distance: cluster c is drawn when its
// group's error is over the threshold (terminal groups always are) and it is input geometry or
// the group it was refined from is at or under the threshold. error_scale multiplies every group
// error first (1 for a true cut). Returns the drawn triangles' indices.
std::vector<int32_t> clod_cut(const Clod &d, float threshold, float error_scale);

} // namespace rm
