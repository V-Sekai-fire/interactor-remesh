// remesh.elf: meshoptimizer 9e1f07b1's voxel remesher and simplifier for the
// avatar build (RFD 2284). The guest reads only its arguments and returns
// arrays; it has no file, socket or host-object access.
#include <api.hpp>
#include <meshoptimizer.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
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

int main() {
	ADD_API_FUNCTION(version, "String", "", "meshoptimizer version and manifest pin");
	ADD_API_FUNCTION(remesh, "Array", "PackedVector3Array positions, PackedInt32Array indices, int resolution, int options, int target_triangles", "voxel remesh, weld, simplify; returns [positions, indices, error]");
	ADD_API_FUNCTION(simplify, "Array", "PackedVector3Array positions, PackedInt32Array indices, PackedFloat32Array attributes, PackedFloat32Array weights, int target_index_count, float target_error, int options", "attribute-aware simplify; returns [indices, error]");
	ADD_API_FUNCTION(gate, "String", "int planted_um", "sphere gate; planted_um > tolerance must FAIL");
	halt();
}
