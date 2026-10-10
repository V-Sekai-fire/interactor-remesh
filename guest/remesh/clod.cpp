#include "clod.h"
#include <meshoptimizer.h>
#define CLUSTERLOD_IMPLEMENTATION
#include <clusterlod.h>
#include <algorithm>
#include <cfloat>

namespace rm {

Clod cluster_lod(const std::vector<float> &positions, const std::vector<float> &attributes, int attribute_count,
		const std::vector<unsigned> &indices, int max_triangles, int64_t flags) {
	Clod d;
	const size_t vcount = positions.size() / 3;
	if (vcount == 0 || indices.size() < 3)
		return d;
	// clusterlod mutates positions in place when it dilates borders, so it gets its own copy.
	std::vector<float> pos = positions;
	clodConfig config = clodDefaultConfig(size_t(std::max(4, std::min(256, max_triangles))));
	config.optimize_bounds = true;
	config.simplify_dilate_borders = (flags & ClodFlags::Dilate) != 0;
	if (flags & ClodFlags::NoSloppy) config.simplify_fallback_sloppy = false;
	if (flags & ClodFlags::NoPermissive) config.simplify_permissive = false;
	std::vector<float> weights(size_t(std::max(0, attribute_count)), 1.0f);
	clodMesh mesh = {};
	mesh.indices = indices.data();
	mesh.index_count = indices.size();
	mesh.vertex_count = vcount;
	mesh.vertex_positions = pos.data();
	mesh.vertex_positions_stride = 12;
	if (attribute_count > 0 && attributes.size() == vcount * size_t(attribute_count)) {
		mesh.vertex_attributes = attributes.data();
		mesh.vertex_attributes_stride = size_t(attribute_count) * sizeof(float);
		mesh.attribute_weights = weights.data();
		mesh.attribute_count = size_t(attribute_count);
		if (flags & ClodFlags::Protect)
			mesh.attribute_protect_mask = (1u << attribute_count) - 1;
	}
	const bool copy = config.simplify_dilate_borders;
	clodBuild(config, mesh, [&](clodGroup group, const clodCluster *clusters, size_t count) -> int {
		const int id = int(d.groups.size() / 6);
		const float err = group.simplified.error == FLT_MAX ? -1.0f : group.simplified.error;
		d.groups.insert(d.groups.end(), { float(group.depth), group.simplified.center[0], group.simplified.center[1],
				group.simplified.center[2], group.simplified.radius, err });
		for (size_t c = 0; c < count; ++c) {
			const clodCluster &cl = clusters[c];
			d.clusters.insert(d.clusters.end(), { id, cl.refined, int32_t(d.indices.size()), int32_t(cl.index_count) });
			d.cluster_bounds.insert(d.cluster_bounds.end(), { cl.bounds.center[0], cl.bounds.center[1], cl.bounds.center[2], cl.bounds.radius });
			if (!copy) {
				for (size_t i = 0; i < cl.index_count; ++i)
					d.indices.push_back(int32_t(cl.indices[i]));
				continue;
			}
			// The positions as they are now: later levels may dilate these vertices.
			std::vector<int32_t> local(cl.index_count);
			std::vector<std::pair<unsigned, int32_t>> seen;
			for (size_t i = 0; i < cl.index_count; ++i) {
				const unsigned v = cl.indices[i];
				int32_t k = -1;
				for (const auto &s : seen)
					if (s.first == v) { k = s.second; break; }
				if (k < 0) {
					k = int32_t(d.source.size());
					seen.push_back({ v, k });
					d.source.push_back(int32_t(v));
					d.positions.insert(d.positions.end(), { pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2] });
				}
				d.indices.push_back(k);
			}
		}
		return id;
	});
	return d;
}

std::vector<int32_t> clod_cut(const Clod &d, float threshold, float error_scale) {
	std::vector<int32_t> out;
	auto over = [&](int g) {
		const float e = d.groups[size_t(g) * 6 + 5];
		return e < 0 || e * error_scale > threshold;
	};
	for (size_t c = 0; c < d.clusters.size() / 4; ++c) {
		const int g = d.clusters[c * 4], refined = d.clusters[c * 4 + 1];
		if (!over(g) || (refined >= 0 && over(refined)))
			continue;
		const int32_t s = d.clusters[c * 4 + 2], n = d.clusters[c * 4 + 3];
		out.insert(out.end(), d.indices.begin() + s, d.indices.begin() + s + n);
	}
	return out;
}

} // namespace rm
