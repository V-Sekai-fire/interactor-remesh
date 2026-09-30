#include "lod.h"

#include <algorithm>
#include <meshoptimizer.h>

namespace rm {

std::vector<Lod> lod_chain(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, bool deformable) {
	const size_t vcount = positions.size() / 3;
	const float scale = meshopt_simplifyScale(positions.data(), vcount, 12);
	std::vector<Lod> chain;
	chain.push_back({ indices, 0.0f, 0.0f });
	if (indices.empty()) return chain;

	// Godot's weights: the normal, 1.0 per component. UV seams need no
	// attribute: meshoptimizer keeps vertices that share a position as seams.
	std::vector<float> attribs(vcount * 3, 0.0f);
	if (normals.size() == positions.size()) attribs = normals;
	const float weights[3] = { 1.0f, 1.0f, 1.0f };

	const float max_mesh_error = 1.0f;
	const unsigned min_target_indices = 12;
	bool allow_prune = true;
	std::vector<unsigned> current = indices;
	float current_error = 0.0f;
	while (current.size() > min_target_indices * 2) {
		const size_t count = current.size();
		const size_t target = std::max<size_t>(((count / 3) / 2) * 3, min_target_indices);
		unsigned options = meshopt_SimplifySparse | meshopt_SimplifyLockBorder;
		if (allow_prune) options |= meshopt_SimplifyPrune;
		if (deformable) options |= meshopt_SimplifyRegularize;
		std::vector<unsigned> next(count);
		float step_error = 0.0f;
		const size_t n = meshopt_simplifyWithAttributes(next.data(), current.data(), count, positions.data(), vcount, 12,
			attribs.data(), 12, weights, 3, nullptr, target, max_mesh_error, options, &step_error);
		if (n == 0 && allow_prune) { allow_prune = false; continue; }
		current_error = std::max(current_error * 1.5f, step_error);
		next.resize(n);
		current = next;
		if (n == 0 || n >= count * 0.75f) break;
		if (current_error > max_mesh_error) break;
		chain.push_back({ current, current_error, current_error * scale });
	}
	return chain;
}

std::vector<Lod> lod_levels(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, bool deformable, float ratio) {
	const size_t vcount = positions.size() / 3;
	const float scale = meshopt_simplifyScale(positions.data(), vcount, 12);
	std::vector<Lod> levels;
	levels.push_back({ indices, 0.0f, 0.0f });
	if (indices.empty()) return levels;
	std::vector<float> attribs(vcount * 3, 0.0f);
	if (normals.size() == positions.size()) attribs = normals;
	const float weights[3] = { 1.0f, 1.0f, 1.0f };
	unsigned options = meshopt_SimplifySparse | meshopt_SimplifyLockBorder | meshopt_SimplifyPrune;
	if (deformable) options |= meshopt_SimplifyRegularize;
	size_t target = indices.size();
	float worst = 0.0f;
	while (true) {
		target = std::max<size_t>(size_t(float(target / 3) * ratio) * 3, 12);
		std::vector<unsigned> out(indices.size());
		float error = 0.0f;
		const size_t n = meshopt_simplifyWithAttributes(out.data(), indices.data(), indices.size(), positions.data(), vcount, 12,
			attribs.data(), 12, weights, 3, nullptr, target, 1.0f, options, &error);
		out.resize(n);
		if (n == 0 || n >= levels.back().indices.size()) break;
		worst = std::max(worst, error);
		levels.push_back({ out, worst, worst * scale });
		if (target <= 12 || worst >= 1.0f) break;
	}
	return levels;
}

} // namespace rm
