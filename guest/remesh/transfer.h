// Closest-surface transfer and alpha culling for remesh.elf. Plain C++ with no
// sandbox API, so the host can gate it natively as well as in the guest.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rm {

// For each query point, the source triangle holding its closest surface point
// and that point's barycentric coordinates in it.
struct Closest {
	std::vector<int32_t> triangle;
	std::vector<float> bary; // three per query
};

Closest closest_points(const std::vector<float> &src_pos, const std::vector<unsigned> &src_idx, const std::vector<float> &query);

// Keep the triangles whose alpha, sampled at the three corners and the
// centroid with UV wrapping, averages at least threshold (0..255).
std::vector<unsigned> alpha_cull(const std::vector<float> &uv, const std::vector<unsigned> &idx, const std::vector<uint8_t> &alpha, int width, int height, int threshold);

} // namespace rm
