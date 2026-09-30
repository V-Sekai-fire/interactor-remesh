// remesh.elf: meshoptimizer 9e1f07b1's voxel remesher and simplifier for the
// avatar build (RFD 2284). The guest reads only its arguments and returns
// arrays; it has no file, socket or host-object access.
#include <api.hpp>
#include <meshoptimizer.h>
#include "avatar.h"
#include "lod.h"
#include "transfer.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

static std::vector<float> flat(const PackedVector3Array &p) {
	std::vector<float> out;
	for (const Vector3 &v : p.fetch()) {
		out.push_back(v.x);
		out.push_back(v.y);
		out.push_back(v.z);
	}
	return out;
}

static std::vector<unsigned> indices_of(const PackedInt32Array &a) {
	std::vector<unsigned> out;
	for (int32_t i : a.fetch())
		out.push_back(unsigned(i));
	return out;
}

static PackedInt32Array packed(const std::vector<unsigned> &ix, size_t n) {
	std::vector<int32_t> out(ix.begin(), ix.begin() + n);
	return PackedInt32Array(out);
}

static Variant version() {
	return Variant(String("meshoptimizer " + std::to_string(MESHOPTIMIZER_VERSION) + " 9e1f07b1"));
}

// Voxel remesh, then weld the triangle soup into an indexed mesh and simplify
// it to target_triangles (0 keeps every remeshed triangle). Positions only:
// the host transfers attributes, skin weights and blendshapes back.
static Variant remesh(PackedVector3Array positions, PackedInt32Array indices, int64_t resolution, int64_t options, int64_t target_triangles) {
	const std::vector<float> pos = flat(positions);
	const std::vector<unsigned> ix = indices_of(indices);
	const size_t vcount = pos.size() / 3;
	const size_t bound = meshopt_remesh(nullptr, 0, ix.data(), ix.size(), pos.data(), vcount, 12, int(resolution), unsigned(options));
	std::vector<float> soup(bound * 9);
	const size_t tris = meshopt_remesh(soup.data(), bound, ix.data(), ix.size(), pos.data(), vcount, 12, int(resolution), unsigned(options));

	std::vector<unsigned> remap(tris * 3);
	const size_t unique = meshopt_generateVertexRemap(remap.data(), nullptr, tris * 3, soup.data(), tris * 3, 12);
	std::vector<float> verts(unique * 3);
	std::vector<unsigned> out(tris * 3);
	meshopt_remapVertexBuffer(verts.data(), soup.data(), tris * 3, 12, remap.data());
	meshopt_remapIndexBuffer(out.data(), nullptr, tris * 3, remap.data());

	size_t count = out.size();
	float error = 0.0f;
	if (target_triangles > 0 && size_t(target_triangles) * 3 < count) {
		std::vector<unsigned> simple(count);
		count = meshopt_simplify(simple.data(), out.data(), out.size(), verts.data(), unique, 12, size_t(target_triangles) * 3, 1.0f, meshopt_SimplifyPrune, &error);
		out.swap(simple);
	}

	// Drop the vertices the simplifier left unreferenced.
	std::vector<unsigned> used(unique);
	const size_t kept = meshopt_optimizeVertexFetchRemap(used.data(), out.data(), count, unique);
	std::vector<float> compact(kept * 3);
	meshopt_remapVertexBuffer(compact.data(), verts.data(), unique, 12, used.data());
	meshopt_remapIndexBuffer(out.data(), out.data(), count, used.data());

	std::vector<Vector3> pv(kept);
	for (size_t i = 0; i < kept; ++i)
		pv[i] = Vector3(compact[i * 3], compact[i * 3 + 1], compact[i * 3 + 2]);
	return Variant(Array::make(Variant(PackedVector3Array(pv)), Variant(packed(out, count)), Variant(double(error))));
}

// Attribute-aware simplification that keeps the original vertices: the result
// indexes into positions, so the host's skin weights, UVs and blendshapes stay
// valid. attributes holds weights.size() floats per vertex.
static Variant simplify(PackedVector3Array positions, PackedInt32Array indices, PackedFloat32Array attributes, PackedFloat32Array weights, int64_t target_index_count, double target_error, int64_t options) {
	const std::vector<float> pos = flat(positions);
	const std::vector<unsigned> ix = indices_of(indices);
	const std::vector<float> attr = attributes.fetch();
	const std::vector<float> w = weights.fetch();
	const size_t vcount = pos.size() / 3;
	std::vector<unsigned> out(ix.size());
	float error = 0.0f;
	size_t count;
	if (w.empty())
		count = meshopt_simplify(out.data(), ix.data(), ix.size(), pos.data(), vcount, 12, size_t(target_index_count), float(target_error), unsigned(options), &error);
	else
		count = meshopt_simplifyWithAttributes(out.data(), ix.data(), ix.size(), pos.data(), vcount, 12, attr.data(), w.size() * sizeof(float), w.data(), w.size(), nullptr, size_t(target_index_count), float(target_error), unsigned(options), &error);
	return Variant(Array::make(Variant(packed(out, count)), Variant(double(error))));
}


// The gate: a UV sphere of radius 0.1 m (head-sized) has an analytic surface,
// so every output vertex's distance from it is the error. planted_um moves the
// output outward by that many micrometres before measuring; a planted defect
// above the tolerance must fail. Tolerance: half a voxel at resolution 64.
static void sphere(std::vector<float> &pos, std::vector<unsigned> &ix, float r, int seg) {
	for (int i = 0; i <= seg; ++i)
		for (int j = 0; j <= seg; ++j) {
			const float th = 3.14159265f * i / seg, ph = 6.2831853f * j / seg;
			pos.push_back(r * std::sin(th) * std::cos(ph));
			pos.push_back(r * std::cos(th));
			pos.push_back(r * std::sin(th) * std::sin(ph));
		}
	for (int i = 0; i < seg; ++i)
		for (int j = 0; j < seg; ++j) {
			const unsigned a = i * (seg + 1) + j, b = a + seg + 1;
			ix.insert(ix.end(), { a, b, a + 1, a + 1, b, b + 1 });
		}
}

static float worst_mm(const std::vector<float> &pos, const std::vector<unsigned> &ix, size_t count, float r, float planted_m) {
	float worst = 0.0f;
	for (size_t k = 0; k < count; ++k) {
		const float *v = &pos[ix[k] * 3];
		const float d = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + planted_m - r;
		worst = std::max(worst, std::fabs(d));
	}
	return worst * 1000.0f;
}

static Variant gate(int64_t planted_um) {
	const float r = 0.1f, planted = float(planted_um) * 1e-6f;
	const int res = 64;
	const float tol_mm = 0.5f * (2.0f * r / res) * 1000.0f;
	std::vector<float> pos;
	std::vector<unsigned> ix;
	sphere(pos, ix, r, 96);
	const size_t vcount = pos.size() / 3;

	const size_t bound = meshopt_remesh(nullptr, 0, ix.data(), ix.size(), pos.data(), vcount, 12, res, meshopt_RemeshSolve);
	std::vector<float> soup(bound * 9);
	const size_t tris = meshopt_remesh(soup.data(), bound, ix.data(), ix.size(), pos.data(), vcount, 12, res, meshopt_RemeshSolve);
	std::vector<unsigned> seq(tris * 3);
	for (size_t k = 0; k < seq.size(); ++k)
		seq[k] = unsigned(k);
	const float remesh_mm = worst_mm(soup, seq, seq.size(), r, planted);

	std::vector<unsigned> simple(ix.size());
	const size_t sc = meshopt_simplify(simple.data(), ix.data(), ix.size(), pos.data(), vcount, 12, ix.size() / 8, 1.0f, 0, nullptr);
	const float simplify_mm = worst_mm(pos, simple, sc, r, planted);

	const bool pass = tris > 0 && sc > 0 && remesh_mm <= tol_mm && simplify_mm <= tol_mm;
	char line[256];
	std::snprintf(line, sizeof line, "%s remesh %zu tris worst %.3f mm; simplify %zu -> %zu tris worst %.3f mm; tolerance %.3f mm; planted %lld um",
		pass ? "PASS" : "FAIL", tris, remesh_mm, ix.size() / 3, sc / 3, simplify_mm, tol_mm, (long long)planted_um);
	return Variant(String(line));
}

// For each query point, the source triangle holding its closest surface point
// and its barycentrics there; the host interpolates UVs, skin weights and
// blendshapes with them. Returns [triangles, barycentrics (three per query)].
static Variant closest(PackedVector3Array src_positions, PackedInt32Array src_indices, PackedVector3Array query) {
	const rm::Closest c = rm::closest_points(flat(src_positions), indices_of(src_indices), flat(query));
	return Variant(Array::make(Variant(PackedInt32Array(c.triangle)), Variant(PackedFloat32Array(c.bary))));
}

// Drop the triangles whose texture alpha is mostly transparent; the kept
// indices. alpha holds width * height bytes, row 0 at v = 0.
static Variant alpha_cull(PackedVector2Array uvs, PackedInt32Array indices, PackedByteArray alpha, int64_t width, int64_t height, int64_t threshold) {
	std::vector<float> uv;
	for (const Vector2 &v : uvs.fetch()) { uv.push_back(v.x); uv.push_back(v.y); }
	const std::vector<unsigned> kept = rm::alpha_cull(uv, indices_of(indices), alpha.fetch(), int(width), int(height), int(threshold));
	return Variant(packed(kept, kept.size()));
}

// Transfer gate: points sampled on a sphere mesh must come back onto
// themselves within 0.01 mm. planted_um shifts each query along the surface
// tangent, so the recovered point misses its sample by that much.
static Variant gate_transfer(int64_t planted_um) {
	std::vector<float> pos;
	std::vector<unsigned> ix;
	sphere(pos, ix, 0.1f, 48);
	std::vector<float> query, truth;
	for (size_t t = 0; t < ix.size() / 3; t += 7) {
		const float w[3] = { 0.2f, 0.3f, 0.5f };
		float p[3] = { 0, 0, 0 };
		for (int c = 0; c < 3; ++c)
			for (int k = 0; k < 3; ++k) p[k] += w[c] * pos[ix[t * 3 + c] * 3 + k];
		truth.insert(truth.end(), p, p + 3);
		const float shift = float(planted_um) * 1e-6f;
		query.insert(query.end(), { p[0] + shift * -p[2] / 0.1f, p[1], p[2] + shift * p[0] / 0.1f });
	}
	const rm::Closest c = rm::closest_points(pos, ix, query);
	float worst = 0;
	for (size_t q = 0; q < truth.size() / 3; ++q) {
		const int t = c.triangle[q];
		float r[3] = { 0, 0, 0 };
		for (int k = 0; k < 3; ++k)
			for (int v = 0; v < 3; ++v) r[k] += c.bary[q * 3 + v] * pos[ix[t * 3 + v] * 3 + k];
		const float d = std::sqrt((r[0] - truth[q * 3]) * (r[0] - truth[q * 3]) + (r[1] - truth[q * 3 + 1]) * (r[1] - truth[q * 3 + 1]) + (r[2] - truth[q * 3 + 2]) * (r[2] - truth[q * 3 + 2]));
		worst = std::max(worst, d);
	}
	const float tol_mm = 0.01f;
	char line[200];
	std::snprintf(line, sizeof line, "%s transfer %zu queries worst %.4f mm; tolerance %.2f mm; planted %lld um",
		worst * 1000.0f <= tol_mm ? "PASS" : "FAIL", truth.size() / 3, worst * 1000.0f, tol_mm, (long long)planted_um);
	return Variant(String(line));
}

// Alpha gate: a 32 x 32 unit plane whose texture is transparent for u < 0.5
// must keep exactly half its triangles. planted != 0 makes the texture opaque.
static Variant gate_alpha(int64_t planted) {
	const int n = 32;
	std::vector<float> uv;
	std::vector<unsigned> ix;
	for (int j = 0; j <= n; ++j)
		for (int i = 0; i <= n; ++i) { uv.push_back(float(i) / n); uv.push_back(float(j) / n); }
	for (int j = 0; j < n; ++j)
		for (int i = 0; i < n; ++i) {
			const unsigned a = j * (n + 1) + i, b = a + n + 1;
			ix.insert(ix.end(), { a, b, a + 1, a + 1, b, b + 1 });
		}
	const int w = 64, h = 64;
	std::vector<uint8_t> alpha(w * h);
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x) alpha[y * w + x] = (planted || x >= w / 2) ? 255 : 0;
	const size_t kept = rm::alpha_cull(uv, ix, alpha, w, h, 128).size() / 3, want = size_t(n * n);
	char line[160];
	std::snprintf(line, sizeof line, "%s alpha kept %zu of %zu triangles; want %zu; planted %lld",
		kept == want ? "PASS" : "FAIL", kept, ix.size() / 3, want, (long long)planted);
	return Variant(String(line));
}

// Godot's LOD chain (see lod.h): [indices, ends, errors_m]. indices holds every
// level back to back, level k spanning [ends[k-1], ends[k]); level 0 is the
// input with zero error, and the errors are in metres and non-decreasing.
// Three packed arrays through Array::make, the path every host implements.
static Variant lod_chain(PackedVector3Array positions, PackedVector3Array normals, PackedInt32Array indices, bool deformable) {
	const std::vector<rm::Lod> chain = rm::lod_chain(flat(positions), flat(normals), indices_of(indices), deformable);
	std::vector<int32_t> all, ends;
	std::vector<float> errors;
	for (const rm::Lod &l : chain) {
		all.insert(all.end(), l.indices.begin(), l.indices.end());
		ends.push_back(int32_t(all.size()));
		errors.push_back(l.error_m);
	}
	return Variant(Array::make(Variant(PackedInt32Array(all)), Variant(PackedInt32Array(ends)), Variant(PackedFloat32Array(errors))));
}

// LOD gate: on a 0.1 m sphere the chain's errors must not decrease, and each
// level's measured worst deviation from the sphere (triangle centroids, where
// a chord sags most) must stay within 2x the reported error plus the level-0
// sag. planted_scale divides every reported error, so under-reporting fails.
static Variant gate_lod(int64_t planted_scale) {
	std::vector<float> pos;
	std::vector<unsigned> ix;
	const float r = 0.1f;
	sphere(pos, ix, r, 96);
	std::vector<float> nrm(pos.size());
	for (size_t i = 0; i < pos.size(); ++i) nrm[i] = pos[i] / r;
	const std::vector<rm::Lod> chain = rm::lod_chain(pos, nrm, ix, false);
	auto sag = [&](const std::vector<unsigned> &t) {
		float worst = 0;
		for (size_t k = 0; k + 2 < t.size(); k += 3) {
			float c[3] = { 0, 0, 0 };
			for (int v = 0; v < 3; ++v)
				for (int a = 0; a < 3; ++a) c[a] += pos[t[k + v] * 3 + a] / 3;
			worst = std::max(worst, r - std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]));
		}
		return worst;
	};
	const float base = sag(chain[0].indices);
	const float div = planted_scale > 0 ? float(planted_scale) : 1.0f;
	bool pass = chain.size() >= 4;
	std::string lines;
	float prev = 0;
	for (size_t l = 0; l < chain.size(); ++l) {
		const float reported = chain[l].error_m / div, measured = sag(chain[l].indices);
		const bool ok = chain[l].error_m >= prev && measured <= 2.0f * reported + base + 1e-6f;
		pass = pass && ok;
		prev = chain[l].error_m;
		char line[160];
		std::snprintf(line, sizeof line, "  lod %zu: %zu tris, reported %.3f mm, measured %.3f mm%s\n", l, chain[l].indices.size() / 3, reported * 1000, measured * 1000, ok ? "" : "  <-- over");
		lines += line;
	}
	char head[120];
	std::snprintf(head, sizeof head, "%s lod chain %zu levels; planted scale %lld\n", pass ? "PASS" : "FAIL", chain.size(), (long long)planted_scale);
	return Variant(String(std::string(head) + lines));
}

// The avatar pipeline (avatar.h). A host passes its meshes and textures as flat
// arrays and writes the results back; every decision is made here.

static std::vector<int32_t> ints_of(const PackedInt32Array &a) { return a.fetch(); }

static rm::Stream stream_of(PackedFloat32Array vertices, PackedInt32Array bones, PackedFloat32Array shapes, PackedInt32Array indices, PackedInt32Array ends) {
	rm::Stream s;
	s.vertices = vertices.fetch();
	s.bones = bones.fetch();
	s.shapes = shapes.fetch();
	s.indices = indices_of(indices);
	s.ends = ends.fetch();
	return s;
}

// One error threshold in metres for every submesh of every mesh (Godot's
// metric); vertices global across meshes. [indices, ends, errors_m, threshold_m, triangles]
static Variant avatar_budget(PackedVector3Array positions, PackedVector3Array normals, PackedInt32Array indices, PackedInt32Array ends, int64_t target_triangles) {
	const rm::Budget b = rm::budget(flat(positions), flat(normals), indices_of(indices), ints_of(ends), size_t(target_triangles));
	return Variant(Array::make(Variant(packed(b.indices, b.indices.size())), Variant(PackedInt32Array(b.ends)), Variant(PackedFloat32Array(b.errors_m)), Variant(double(b.threshold_m)), Variant(int64_t(b.triangles))));
}

// Voxel remesh with every attribute carried back. [vertices, bones, shapes, indices, ends]
static Variant avatar_remesh(PackedFloat32Array vertices, PackedInt32Array bones, PackedFloat32Array shapes, PackedInt32Array indices, PackedInt32Array ends, int64_t resolution, int64_t target_triangles) {
	const rm::Stream o = rm::remesh_stream(stream_of(vertices, bones, shapes, indices, ends), int(resolution), size_t(target_triangles));
	return Variant(Array::make(Variant(PackedFloat32Array(o.vertices)), Variant(PackedInt32Array(o.bones)), Variant(PackedFloat32Array(o.shapes)), Variant(packed(o.indices, o.indices.size())), Variant(PackedInt32Array(o.ends))));
}

// [remap old -> new or -1, indices]
static Variant avatar_compact(PackedInt32Array indices, int64_t vertex_count) {
	std::vector<unsigned> ix = indices_of(indices);
	const std::vector<int32_t> remap = rm::compact_remap(ix, size_t(vertex_count));
	return Variant(Array::make(Variant(PackedInt32Array(remap)), Variant(packed(ix, ix.size()))));
}

// RGBA8 textures back to back, sizes as w, h pairs. [rgba, rects, scale_down]
static Variant avatar_atlas(PackedByteArray pixels, PackedInt32Array sizes, int64_t size) {
	const rm::Atlas a = rm::atlas(pixels.fetch(), ints_of(sizes), int(size));
	return Variant(Array::make(Variant(PackedArray<uint8_t>(a.rgba)), Variant(PackedFloat32Array(a.rects)), Variant(int64_t(a.scale_down))));
}

// uv as u, v pairs. [picks, uv, indices, wrapped]
static Variant avatar_atlas_mesh(PackedFloat32Array uv, PackedInt32Array indices, PackedInt32Array ends, PackedInt32Array texture_of_submesh, PackedFloat32Array rects) {
	const rm::AtlasMesh m = rm::atlas_mesh(uv.fetch(), indices_of(indices), ints_of(ends), ints_of(texture_of_submesh), rects.fetch());
	return Variant(Array::make(Variant(PackedInt32Array(m.picks)), Variant(PackedFloat32Array(m.uv)), Variant(packed(m.indices, m.indices.size())), Variant(int64_t(m.wrapped))));
}

static Variant avatar_hide_shape(PackedVector3Array positions, PackedInt32Array bones, PackedFloat32Array weights, PackedVector3Array bone_origins) {
	const std::vector<float> d = rm::hide_shape(flat(positions), ints_of(bones), weights.fetch(), flat(bone_origins));
	std::vector<Vector3> v(d.size() / 3);
	for (size_t i = 0; i < v.size(); ++i) v[i] = Vector3(d[i * 3], d[i * 3 + 1], d[i * 3 + 2]);
	return Variant(PackedVector3Array(v));
}

// Avatar gate, clean at planted 0. Each planted defect must FAIL:
// 1 moves atlas rect 1 onto rect 0; 2 doubles the remeshed blendshape deltas;
// 3 halves the budget threshold, so the result no longer fits.
static Variant gate_avatar(int64_t planted) {
	std::string report;
	bool pass = true;
	auto note = [&](bool ok, const std::string &what) { pass = pass && ok; report += (ok ? "  ok   " : "  FAIL ") + what + "\n"; };
	char buf[200];

	// Atlas: three solid textures must land apart, each with its own colour.
	{
		const int sz[3][2] = { { 32, 32 }, { 16, 48 }, { 24, 8 } };
		const uint8_t col[3][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 } };
		std::vector<uint8_t> px;
		std::vector<int32_t> sizes;
		for (int i = 0; i < 3; ++i) {
			sizes.push_back(sz[i][0]); sizes.push_back(sz[i][1]);
			for (int k = 0; k < sz[i][0] * sz[i][1]; ++k) px.insert(px.end(), { col[i][0], col[i][1], col[i][2], 255 });
		}
		rm::Atlas a = rm::atlas(px, sizes, 64);
		if (planted == 1) { a.rects[4] = a.rects[0]; a.rects[5] = a.rects[1]; }
		bool apart = true, colours = true;
		for (int i = 0; i < 3; ++i) {
			const float *r = &a.rects[i * 4];
			for (int j = 0; j < i; ++j) {
				const float *q = &a.rects[j * 4];
				if (r[0] < q[0] + q[2] && q[0] < r[0] + r[2] && r[1] < q[1] + q[3] && q[1] < r[1] + r[3]) apart = false;
			}
			const int cx = int((r[0] + r[2] / 2) * 64), cy = int((r[1] + r[3] / 2) * 64);
			const uint8_t *p = &a.rgba[(size_t(cy) * 64 + cx) * 4];
			if (p[0] != col[i][0] || p[1] != col[i][1] || p[2] != col[i][2]) colours = false;
		}
		std::snprintf(buf, sizeof buf, "atlas: 3 textures in 64 px, halved %d times, rects apart %d, colours at centres %d", a.scale_down, apart, colours);
		note(apart && colours, buf);
	}

	// Remesh: a 0.1 m sphere whose one blendshape pushes out 1 cm along the
	// normal and whose weights split by height; both must survive transfer.
	{
		std::vector<float> pos;
		std::vector<unsigned> ix;
		sphere(pos, ix, 0.1f, 48);
		rm::Stream s;
		const size_t nv = pos.size() / 3;
		s.vertices.assign(nv * rm::kStride, 0);
		s.bones.assign(nv * 4, 0);
		s.shapes.assign(nv * 3, 0);
		for (size_t v = 0; v < nv; ++v) {
			float *d = &s.vertices[v * rm::kStride];
			for (int k = 0; k < 3; ++k) { d[k] = pos[v * 3 + k]; d[3 + k] = pos[v * 3 + k] / 0.1f; s.shapes[v * 3 + k] = pos[v * 3 + k] / 0.1f * 0.01f; }
			d[6] = 0.5f; d[7] = 0.5f;
			const float up = 0.5f + pos[v * 3 + 1] / 0.2f;
			d[8] = up; d[9] = 1 - up;
			s.bones[v * 4] = 0; s.bones[v * 4 + 1] = 1;
		}
		s.indices = ix;
		s.ends = { int32_t(ix.size()) };
		rm::Stream o = rm::remesh_stream(s, 64, 1500);
		if (planted == 2) for (float &x : o.shapes) x *= 2;
		const size_t on = o.vertex_count();
		float worst_shape = 0, worst_sum = 0, worst_w = 0;
		for (size_t v = 0; v < on; ++v) {
			const float *d = &o.vertices[v * rm::kStride];
			const float r = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
			for (int k = 0; k < 3; ++k) worst_shape = std::max(worst_shape, std::fabs(o.shapes[v * 3 + k] - d[k] / r * 0.01f));
			float sum = 0, w_up = 0;
			for (int k = 0; k < 4; ++k) { sum += d[8 + k]; if (o.bones[v * 4 + k] == 0) w_up += d[8 + k]; }
			worst_sum = std::max(worst_sum, std::fabs(sum - 1));
			worst_w = std::max(worst_w, std::fabs(w_up - (0.5f + d[1] / 0.2f)));
		}
		std::snprintf(buf, sizeof buf, "remesh: %zu tris; shape error %.3f mm (tol 0.5), weight sum error %.5f, weight error %.3f (tol 0.05)", o.indices.size() / 3, worst_shape * 1000, worst_sum, worst_w);
		note(o.indices.size() > 0 && worst_shape <= 0.0005f && worst_sum <= 1e-4f && worst_w <= 0.05f, buf);
	}

	// Budget: two spheres, 0.1 m and 0.05 m, share 4000 triangles; every
	// submesh's chosen error is at or under one threshold, and it fits.
	{
		std::vector<float> a, b, pos, nrm;
		std::vector<unsigned> ia, ib, ix;
		sphere(a, ia, 0.1f, 64);
		sphere(b, ib, 0.05f, 64);
		pos = a;
		pos.insert(pos.end(), b.begin(), b.end());
		const unsigned off = unsigned(a.size() / 3);
		ix = ia;
		for (unsigned i : ib) ix.push_back(i + off);
		nrm.resize(pos.size());
		for (size_t v = 0; v < pos.size() / 3; ++v) {
			const float r = v < off ? 0.1f : 0.05f;
			for (int k = 0; k < 3; ++k) nrm[v * 3 + k] = pos[v * 3 + k] / r;
		}
		const std::vector<int32_t> ends = { int32_t(ia.size()), int32_t(ix.size()) };
		rm::Budget bu = rm::budget(pos, nrm, ix, ends, 4000);
		if (planted == 3) {
			// Recount at half the threshold, as if the search had stopped short.
			bu.triangles = 0;
			for (size_t sub = 0, st = 0; sub < ends.size(); st = size_t(ends[sub]), ++sub) {
				const auto chain = rm::lod_chain(pos, nrm, std::vector<unsigned>(ix.begin() + st, ix.begin() + ends[sub]), true);
				size_t k = 0;
				while (k + 1 < chain.size() && chain[k + 1].error_m <= bu.threshold_m / 2) ++k;
				bu.triangles += chain[k].indices.size() / 3;
			}
		}
		bool under = true;
		for (float e : bu.errors_m) under = under && e <= bu.threshold_m + 1e-9f;
		std::snprintf(buf, sizeof buf, "budget: %zu tris of 4000 at %.3f mm; submesh errors %.3f and %.3f mm", bu.triangles, bu.threshold_m * 1000, bu.errors_m[0] * 1000, bu.errors_m[1] * 1000);
		note(bu.triangles <= 4000 && under, buf);
	}

	std::snprintf(buf, sizeof buf, "%s avatar gate; planted %lld\n", pass ? "PASS" : "FAIL", (long long)planted);
	return Variant(String(std::string(buf) + report));
}

int main() {
	ADD_API_FUNCTION(version, "String", "", "meshoptimizer version and manifest pin");
	ADD_API_FUNCTION(remesh, "Array", "PackedVector3Array positions, PackedInt32Array indices, int resolution, int options, int target_triangles", "voxel remesh, weld, simplify; returns [positions, indices, error]");
	ADD_API_FUNCTION(simplify, "Array", "PackedVector3Array positions, PackedInt32Array indices, PackedFloat32Array attributes, PackedFloat32Array weights, int target_index_count, float target_error, int options", "attribute-aware simplify; returns [indices, error]");
	ADD_API_FUNCTION(gate, "String", "int planted_um", "sphere gate; planted_um > tolerance must FAIL");
	ADD_API_FUNCTION(closest, "Array", "PackedVector3Array src_positions, PackedInt32Array src_indices, PackedVector3Array query", "closest source triangle and barycentrics per query; returns [triangles, barycentrics]");
	ADD_API_FUNCTION(alpha_cull, "PackedInt32Array", "PackedVector2Array uvs, PackedInt32Array indices, PackedByteArray alpha, int width, int height, int threshold", "drop mostly transparent triangles; the kept indices");
	ADD_API_FUNCTION(gate_transfer, "String", "int planted_um", "transfer gate; a planted tangential shift must FAIL");
	ADD_API_FUNCTION(gate_alpha, "String", "int planted", "alpha gate; planted != 0 must FAIL");
	ADD_API_FUNCTION(lod_chain, "Array", "PackedVector3Array positions, PackedVector3Array normals, PackedInt32Array indices, bool deformable", "Godot-style LOD chain; [indices, ends, errors_m]");
	ADD_API_FUNCTION(gate_lod, "String", "int planted_scale", "LOD gate; planted_scale > 1 under-reports and must FAIL");
	ADD_API_FUNCTION(avatar_budget, "Array", "PackedVector3Array positions, PackedVector3Array normals, PackedInt32Array indices, PackedInt32Array ends, int target_triangles", "one error threshold across meshes; [indices, ends, errors_m, threshold_m, triangles]");
	ADD_API_FUNCTION(avatar_remesh, "Array", "PackedFloat32Array vertices, PackedInt32Array bones, PackedFloat32Array shapes, PackedInt32Array indices, PackedInt32Array ends, int resolution, int target_triangles", "voxel remesh with attributes; [vertices, bones, shapes, indices, ends]");
	ADD_API_FUNCTION(avatar_compact, "Array", "PackedInt32Array indices, int vertex_count", "[remap, indices]");
	ADD_API_FUNCTION(avatar_atlas, "Array", "PackedByteArray pixels, PackedInt32Array sizes, int size", "[rgba, rects, scale_down]");
	ADD_API_FUNCTION(avatar_atlas_mesh, "Array", "PackedFloat32Array uv, PackedInt32Array indices, PackedInt32Array ends, PackedInt32Array texture_of_submesh, PackedFloat32Array rects", "[picks, uv, indices, wrapped]");
	ADD_API_FUNCTION(avatar_hide_shape, "PackedVector3Array", "PackedVector3Array positions, PackedInt32Array bones, PackedFloat32Array weights, PackedVector3Array bone_origins", "deltas onto each vertex's heaviest bone");
	ADD_API_FUNCTION(gate_avatar, "String", "int planted", "avatar gate; planted 1..3 must FAIL");
	halt();
}
