#include "avatar.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <meshoptimizer.h>
#include <numeric>

#include "lod.h"
#include "transfer.h"

namespace rm {
namespace {

std::vector<float> positions_of(const Stream &s) {
	std::vector<float> p(s.vertex_count() * 3);
	for (size_t v = 0; v < s.vertex_count(); ++v)
		for (int k = 0; k < 3; ++k) p[v * 3 + k] = s.vertices[v * kStride + k];
	return p;
}

void barycentric(const float *p, const float *a, const float *b, const float *c, float out[3]) {
	float v0[3], v1[3], v2[3];
	for (int k = 0; k < 3; ++k) { v0[k] = b[k] - a[k]; v1[k] = c[k] - a[k]; v2[k] = p[k] - a[k]; }
	auto dot = [](const float *x, const float *y) { return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; };
	const float d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1), d20 = dot(v2, v0), d21 = dot(v2, v1);
	const float den = d00 * d11 - d01 * d01;
	if (std::fabs(den) < 1e-20f) { out[0] = 1; out[1] = 0; out[2] = 0; return; }
	const float v = (d11 * d20 - d01 * d21) / den, w = (d00 * d21 - d01 * d20) / den;
	out[0] = 1 - v - w; out[1] = v; out[2] = w;
}

} // namespace

Budget budget(const std::vector<float> &positions, const std::vector<float> &normals, const std::vector<unsigned> &indices, const std::vector<int32_t> &ends, size_t target_triangles) {
	std::vector<std::vector<Lod>> chains;
	for (size_t s = 0, start = 0; s < ends.size(); start = size_t(ends[s]), ++s) {
		std::vector<unsigned> sub(indices.begin() + start, indices.begin() + ends[s]);
		// Skinned: regularize, as Godot does for deformable meshes.
		chains.push_back(lod_chain(positions, normals, sub, true));
	}
	auto level_at = [](const std::vector<Lod> &c, float eps) -> const Lod & {
		size_t k = 0;
		while (k + 1 < c.size() && c[k + 1].error_m <= eps) ++k;
		return c[k];
	};
	auto triangles_at = [&](float eps) {
		size_t n = 0;
		for (const auto &c : chains) n += level_at(c, eps).indices.size() / 3;
		return n;
	};
	std::vector<float> candidates{ 0.0f };
	for (const auto &c : chains)
		for (const Lod &l : c) candidates.push_back(l.error_m);
	std::sort(candidates.begin(), candidates.end());
	Budget out;
	out.threshold_m = candidates.back();
	for (float e : candidates)
		if (triangles_at(e) <= target_triangles) { out.threshold_m = e; break; }
	for (const auto &c : chains) {
		const Lod &l = level_at(c, out.threshold_m);
		out.indices.insert(out.indices.end(), l.indices.begin(), l.indices.end());
		out.ends.push_back(int32_t(out.indices.size()));
		out.errors_m.push_back(l.error_m);
	}
	out.triangles = out.indices.size() / 3;
	return out;
}

Stream remesh_stream(const Stream &in, int resolution, size_t target_triangles, int *flipped_out) {
	const size_t nv = in.vertex_count(), ns = in.shape_count();
	const std::vector<float> pos = positions_of(in);
	std::vector<int32_t> sub_of(in.indices.size() / 3);
	for (size_t s = 0, start = 0; s < in.ends.size(); start = size_t(in.ends[s]), ++s)
		for (size_t t = start / 3; t < size_t(in.ends[s]) / 3; ++t) sub_of[t] = int32_t(s);

	// Remesh, weld, simplify to the target.
	const size_t bound = meshopt_remesh(nullptr, 0, in.indices.data(), in.indices.size(), pos.data(), nv, 12, resolution, meshopt_RemeshSolve);
	std::vector<float> soup(bound * 9);
	const size_t tris = meshopt_remesh(soup.data(), bound, in.indices.data(), in.indices.size(), pos.data(), nv, 12, resolution, meshopt_RemeshSolve);
	std::vector<unsigned> remap(tris * 3);
	const size_t unique = meshopt_generateVertexRemap(remap.data(), nullptr, tris * 3, soup.data(), tris * 3, 12);
	std::vector<float> rp(unique * 3);
	std::vector<unsigned> ri(tris * 3);
	meshopt_remapVertexBuffer(rp.data(), soup.data(), tris * 3, 12, remap.data());
	meshopt_remapIndexBuffer(ri.data(), nullptr, tris * 3, remap.data());
	if (target_triangles > 0 && target_triangles * 3 < ri.size()) {
		std::vector<unsigned> simple(ri.size());
		simple.resize(meshopt_simplify(simple.data(), ri.data(), ri.size(), rp.data(), unique, 12, target_triangles * 3, 1.0f, meshopt_SimplifyPrune, nullptr));
		ri.swap(simple);
	}
	const size_t nt = ri.size() / 3, rv = rp.size() / 3;

	// Closest source point for every new vertex and every new centroid.
	std::vector<float> query(rp);
	for (size_t t = 0; t < nt; ++t)
		for (int k = 0; k < 3; ++k) query.push_back((rp[ri[t * 3] * 3 + k] + rp[ri[t * 3 + 1] * 3 + k] + rp[ri[t * 3 + 2] * 3 + k]) / 3);
	const Closest c = closest_points(pos, in.indices, query);

	Stream out;
	out.vertices.assign(nt * 3 * kStride, 0.0f);
	out.bones.assign(nt * 3 * 4, 0);
	out.shapes.assign(ns * nt * 3 * 3, 0.0f);
	std::vector<std::vector<unsigned>> per_sub(in.ends.size());
	size_t disagree = 0;
	for (size_t t = 0; t < nt; ++t) {
		const int32_t S = c.triangle[rv + t];
		const unsigned sa = in.indices[S * 3], sb = in.indices[S * 3 + 1], sc = in.indices[S * 3 + 2];
		float face_n[3], src_n[3] = { 0, 0, 0 };
		for (int corner = 0; corner < 3; ++corner) {
			const unsigned rvx = ri[t * 3 + corner];
			const size_t o = t * 3 + corner;
			const int32_t T = c.triangle[rvx];
			const unsigned ta[3] = { in.indices[T * 3], in.indices[T * 3 + 1], in.indices[T * 3 + 2] };
			const float *w = &c.bary[rvx * 3];
			float *dst = &out.vertices[o * kStride];
			for (int k = 0; k < 3; ++k) dst[k] = rp[rvx * 3 + k];
			// Normal: blended from the closest source point.
			float n[3] = { 0, 0, 0 };
			for (int j = 0; j < 3; ++j)
				for (int k = 0; k < 3; ++k) n[k] += w[j] * in.vertices[ta[j] * kStride + 3 + k];
			const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
			for (int k = 0; k < 3; ++k) dst[3 + k] = len > 0 ? n[k] / len : 0;
			// UV: projected into the centroid's source triangle, clamped.
			float l[3];
			barycentric(&rp[rvx * 3], &pos[sa * 3], &pos[sb * 3], &pos[sc * 3], l);
			float sum = 0;
			for (float &x : l) { x = std::min(1.5f, std::max(-0.5f, x)); sum += x; }
			for (float &x : l) x /= std::max(1e-6f, sum);
			for (int k = 0; k < 2; ++k)
				dst[6 + k] = l[0] * in.vertices[sa * kStride + 6 + k] + l[1] * in.vertices[sb * kStride + 6 + k] + l[2] * in.vertices[sc * kStride + 6 + k];
			// Skin: the corners' influences blended, the four largest kept.
			std::map<int32_t, float> acc;
			for (int j = 0; j < 3; ++j)
				for (int k = 0; k < 4; ++k) {
					const float wt = in.vertices[ta[j] * kStride + 8 + k] * w[j];
					if (wt > 0) acc[in.bones[ta[j] * 4 + k]] += wt;
				}
			std::vector<std::pair<float, int32_t>> top;
			for (const auto &kv : acc) top.push_back({ kv.second, kv.first });
			std::sort(top.rbegin(), top.rend());
			if (top.size() > 4) top.resize(4);
			float total = 0;
			for (const auto &p : top) total += p.first;
			for (size_t k = 0; k < 4; ++k) {
				out.bones[o * 4 + k] = k < top.size() ? top[k].second : (top.empty() ? in.bones[ta[0] * 4] : top[0].second);
				dst[8 + k] = k < top.size() && total > 0 ? top[k].first / total : (k == 0 && top.empty() ? 1.0f : 0.0f);
			}
			// Blendshapes: blended like the normal.
			for (size_t s = 0; s < ns; ++s)
				for (int k = 0; k < 3; ++k) {
					float d = 0;
					for (int j = 0; j < 3; ++j) d += w[j] * in.shapes[(s * nv + ta[j]) * 3 + k];
					out.shapes[(s * nt * 3 + o) * 3 + k] = d;
				}
			for (int k = 0; k < 3; ++k) src_n[k] += n[k];
		}
		const float *p0 = &out.vertices[(t * 3) * kStride], *p1 = &out.vertices[(t * 3 + 1) * kStride], *p2 = &out.vertices[(t * 3 + 2) * kStride];
		const float e1[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] }, e2[3] = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
		face_n[0] = e1[1] * e2[2] - e1[2] * e2[1]; face_n[1] = e1[2] * e2[0] - e1[0] * e2[2]; face_n[2] = e1[0] * e2[1] - e1[1] * e2[0];
		if (face_n[0] * src_n[0] + face_n[1] * src_n[1] + face_n[2] * src_n[2] < 0) ++disagree;
		per_sub[size_t(sub_of[S])].insert(per_sub[size_t(sub_of[S])].end(), { unsigned(t * 3), unsigned(t * 3 + 1), unsigned(t * 3 + 2) });
	}
	// The remesher winds its closed surface consistently; two-sided sources
	// disagree locally, so only a majority flips, and it flips everything.
	const bool flip = disagree * 2 > nt;
	for (auto &sub : per_sub) {
		if (flip)
			for (size_t k = 0; k + 2 < sub.size(); k += 3) std::swap(sub[k + 1], sub[k + 2]);
		out.indices.insert(out.indices.end(), sub.begin(), sub.end());
		out.ends.push_back(int32_t(out.indices.size()));
	}
	if (flipped_out) *flipped_out = flip ? 1 : 0;
	return out;
}

std::vector<int32_t> compact_remap(std::vector<unsigned> &indices, size_t vertex_count) {
	std::vector<int32_t> remap(vertex_count, -1);
	int32_t next = 0;
	for (unsigned &i : indices) {
		if (remap[i] < 0) remap[i] = next++;
		i = unsigned(remap[i]);
	}
	return remap;
}

Atlas atlas(const std::vector<uint8_t> &pixels, const std::vector<int32_t> &sizes, int size) {
	const size_t n = sizes.size() / 2;
	std::vector<size_t> offset(n, 0);
	for (size_t i = 1; i < n; ++i) offset[i] = offset[i - 1] + size_t(sizes[(i - 1) * 2]) * sizes[(i - 1) * 2 + 1] * 4;
	const int pad = 4;
	Atlas out;
	std::vector<int> x(n), y(n), w(n), h(n);
	// Shelf packing, tallest first; halve everything until it fits.
	for (out.scale_down = 0;; ++out.scale_down) {
		for (size_t i = 0; i < n; ++i) { w[i] = std::max(1, sizes[i * 2] >> out.scale_down); h[i] = std::max(1, sizes[i * 2 + 1] >> out.scale_down); }
		std::vector<size_t> order(n);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return h[a] > h[b]; });
		int cx = 0, cy = 0, shelf = 0;
		bool fits = true;
		for (size_t i : order) {
			if (cx + w[i] + pad > size) { cx = 0; cy += shelf; shelf = 0; }
			if (w[i] + pad > size || cy + h[i] + pad > size) { fits = false; break; }
			x[i] = cx + pad / 2; y[i] = cy + pad / 2;
			cx += w[i] + pad;
			shelf = std::max(shelf, h[i] + pad);
		}
		if (fits) break;
	}
	out.rgba.assign(size_t(size) * size * 4, 0);
	for (size_t i = 0; i < n; ++i) {
		const int sw = sizes[i * 2], sh = sizes[i * 2 + 1];
		// Nearest-texel downscale, with the padding filled by the edge texels.
		for (int yy = -pad / 2; yy < h[i] + pad / 2; ++yy)
			for (int xx = -pad / 2; xx < w[i] + pad / 2; ++xx) {
				const int tx = x[i] + xx, ty = y[i] + yy;
				if (tx < 0 || ty < 0 || tx >= size || ty >= size) continue;
				const int cxs = std::min(w[i] - 1, std::max(0, xx)), cys = std::min(h[i] - 1, std::max(0, yy));
				const int sx = std::min(sw - 1, cxs * sw / w[i]), sy = std::min(sh - 1, cys * sh / h[i]);
				const uint8_t *src = &pixels[offset[i] + (size_t(sy) * sw + sx) * 4];
				uint8_t *dst = &out.rgba[(size_t(ty) * size + tx) * 4];
				dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]; dst[3] = 255;
			}
		out.rects.insert(out.rects.end(), { float(x[i]) / size, float(y[i]) / size, float(w[i]) / size, float(h[i]) / size });
	}
	return out;
}

AtlasMesh atlas_mesh(const std::vector<float> &uv, const std::vector<unsigned> &indices, const std::vector<int32_t> &ends, const std::vector<int32_t> &texture_of_submesh, const std::vector<float> &rects) {
	AtlasMesh out;
	std::map<std::pair<size_t, unsigned>, unsigned> seen;
	for (size_t s = 0, start = 0; s < ends.size(); start = size_t(ends[s]), ++s) {
		const float *r = &rects[size_t(std::max(0, texture_of_submesh[s])) * 4];
		for (size_t k = start; k < size_t(ends[s]); ++k) {
			const unsigned v = indices[k];
			auto it = seen.find({ s, v });
			if (it == seen.end()) {
				float u = uv[v * 2], w = uv[v * 2 + 1];
				if (u < 0 || u > 1 || w < 0 || w > 1) ++out.wrapped;
				if (u < 0 || u > 1) u -= std::floor(u);
				if (w < 0 || w > 1) w -= std::floor(w);
				it = seen.insert({ { s, v }, unsigned(out.picks.size()) }).first;
				out.picks.push_back(int32_t(v));
				out.uv.push_back(r[0] + u * r[2]);
				out.uv.push_back(r[1] + w * r[3]);
			}
			out.indices.push_back(it->second);
		}
	}
	return out;
}

std::vector<float> hide_shape(const std::vector<float> &positions, const std::vector<int32_t> &bones, const std::vector<float> &weights, const std::vector<float> &bone_origins) {
	const size_t nv = positions.size() / 3;
	std::vector<float> d(nv * 3, 0.0f);
	for (size_t v = 0; v < nv; ++v) {
		size_t best = 0;
		for (size_t k = 1; k < 4; ++k)
			if (weights[v * 4 + k] > weights[v * 4 + best]) best = k;
		const int32_t b = bones[v * 4 + best];
		for (int k = 0; k < 3; ++k) d[v * 3 + k] = bone_origins[b * 3 + k] - positions[v * 3 + k];
	}
	return d;
}

} // namespace rm
