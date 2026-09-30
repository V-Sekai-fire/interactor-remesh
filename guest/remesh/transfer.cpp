#include "transfer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm {
namespace {

struct V3 { float x, y, z; };
V3 at(const std::vector<float> &p, size_t i) { return { p[i * 3], p[i * 3 + 1], p[i * 3 + 2] }; }
V3 sub(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Ericson, Real-Time Collision Detection, 5.1.5: closest point on a triangle,
// returned as barycentric coordinates (u, v, w) for (a, b, c).
void closest_on_triangle(V3 p, V3 a, V3 b, V3 c, float &u, float &v, float &w) {
	const V3 ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
	const float d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return; }
	const V3 bp = sub(p, b);
	const float d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return; }
	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0) { const float t = d1 / (d1 - d3); u = 1 - t; v = t; w = 0; return; }
	const V3 cp = sub(p, c);
	const float d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return; }
	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0) { const float t = d2 / (d2 - d6); u = 1 - t; v = 0; w = t; return; }
	const float va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) { const float t = (d4 - d3) / ((d4 - d3) + (d5 - d6)); u = 0; v = 1 - t; w = t; return; }
	const float den = 1.0f / (va + vb + vc);
	v = vb * den; w = vc * den; u = 1 - v - w;
}

// A uniform grid over triangle bounding boxes; queries search outward in
// shells until the next shell cannot hold anything closer.
struct Grid {
	float lo[3], cell;
	int n[3];
	std::vector<std::vector<int32_t>> cells;
	int clampi(int v, int k) const { return std::max(0, std::min(n[k] - 1, v)); }
	int key(int i, int j, int k) const { return (k * n[1] + j) * n[0] + i; }
};

} // namespace

Closest closest_points(const std::vector<float> &src_pos, const std::vector<unsigned> &src_idx, const std::vector<float> &query) {
	const size_t tris = src_idx.size() / 3, nq = query.size() / 3;
	Closest out;
	out.triangle.assign(nq, -1);
	out.bary.assign(nq * 3, 0.0f);
	if (tris == 0) return out;

	Grid g;
	float hi[3];
	for (int k = 0; k < 3; ++k) { g.lo[k] = std::numeric_limits<float>::max(); hi[k] = -g.lo[k]; }
	for (size_t i = 0; i < src_pos.size() / 3; ++i)
		for (int k = 0; k < 3; ++k) { g.lo[k] = std::min(g.lo[k], src_pos[i * 3 + k]); hi[k] = std::max(hi[k], src_pos[i * 3 + k]); }
	const float extent = std::max({ hi[0] - g.lo[0], hi[1] - g.lo[1], hi[2] - g.lo[2], 1e-6f });
	const int per_axis = std::max(1, std::min(64, int(std::cbrt(double(tris)) * 2)));
	g.cell = extent / per_axis;
	for (int k = 0; k < 3; ++k) g.n[k] = std::max(1, int(std::ceil((hi[k] - g.lo[k]) / g.cell)) + 1);
	g.cells.resize(size_t(g.n[0]) * g.n[1] * g.n[2]);
	for (size_t t = 0; t < tris; ++t) {
		int a[3], b[3];
		for (int k = 0; k < 3; ++k) {
			float mn = std::numeric_limits<float>::max(), mx = -mn;
			for (int c = 0; c < 3; ++c) { const float v = src_pos[src_idx[t * 3 + c] * 3 + k]; mn = std::min(mn, v); mx = std::max(mx, v); }
			a[k] = g.clampi(int((mn - g.lo[k]) / g.cell), k);
			b[k] = g.clampi(int((mx - g.lo[k]) / g.cell), k);
		}
		for (int k = a[2]; k <= b[2]; ++k)
			for (int j = a[1]; j <= b[1]; ++j)
				for (int i = a[0]; i <= b[0]; ++i) g.cells[g.key(i, j, k)].push_back(int32_t(t));
	}

	std::vector<uint32_t> seen(tris, 0);
	uint32_t stamp = 0;
	const int max_ring = std::max({ g.n[0], g.n[1], g.n[2] });
	for (size_t q = 0; q < nq; ++q) {
		const V3 p = at(query, q);
		int c[3] = { g.clampi(int((p.x - g.lo[0]) / g.cell), 0), g.clampi(int((p.y - g.lo[1]) / g.cell), 1), g.clampi(int((p.z - g.lo[2]) / g.cell), 2) };
		float best = std::numeric_limits<float>::max();
		++stamp;
		for (int r = 0; r <= max_ring; ++r) {
			// Everything outside ring r is at least (r - 1) cells away from p.
			if (r > 0 && best < (float(r - 1) * g.cell) * (float(r - 1) * g.cell)) break;
			for (int k = c[2] - r; k <= c[2] + r; ++k)
				for (int j = c[1] - r; j <= c[1] + r; ++j)
					for (int i = c[0] - r; i <= c[0] + r; ++i) {
						if (std::max({ std::abs(i - c[0]), std::abs(j - c[1]), std::abs(k - c[2]) }) != r) continue;
						if (i < 0 || j < 0 || k < 0 || i >= g.n[0] || j >= g.n[1] || k >= g.n[2]) continue;
						for (int32_t t : g.cells[g.key(i, j, k)]) {
							if (seen[t] == stamp) continue;
							seen[t] = stamp;
							const V3 a = at(src_pos, src_idx[t * 3]), b = at(src_pos, src_idx[t * 3 + 1]), cc = at(src_pos, src_idx[t * 3 + 2]);
							float u, v, w;
							closest_on_triangle(p, a, b, cc, u, v, w);
							const V3 s = { u * a.x + v * b.x + w * cc.x, u * a.y + v * b.y + w * cc.y, u * a.z + v * b.z + w * cc.z };
							const V3 d = sub(p, s);
							const float dd = dot(d, d);
							if (dd < best) {
								best = dd;
								out.triangle[q] = t;
								out.bary[q * 3] = u; out.bary[q * 3 + 1] = v; out.bary[q * 3 + 2] = w;
							}
						}
					}
		}
	}
	return out;
}

std::vector<unsigned> alpha_cull(const std::vector<float> &uv, const std::vector<unsigned> &idx, const std::vector<uint8_t> &alpha, int width, int height, int threshold) {
	auto sample = [&](float u, float v) {
		// Wrap tiled UVs, but keep 1.0 on the last texel rather than the first.
		if (u < 0.0f || u > 1.0f) u -= std::floor(u);
		if (v < 0.0f || v > 1.0f) v -= std::floor(v);
		const int x = std::min(width - 1, int(u * width)), y = std::min(height - 1, int(v * height));
		return int(alpha[size_t(y) * width + x]);
	};
	std::vector<unsigned> kept;
	for (size_t t = 0; t + 2 < idx.size(); t += 3) {
		float cu = 0, cv = 0;
		int sum = 0;
		for (int c = 0; c < 3; ++c) {
			const float u = uv[idx[t + c] * 2], v = uv[idx[t + c] * 2 + 1];
			sum += sample(u, v);
			cu += u / 3; cv += v / 3;
		}
		sum += sample(cu, cv);
		if (sum >= threshold * 4) kept.insert(kept.end(), { idx[t], idx[t + 1], idx[t + 2] });
	}
	return kept;
}

} // namespace rm
