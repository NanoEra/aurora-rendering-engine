/*
Cornell Box: triangle-driven rasterization + Z-buffer + recursive planar mirror reflection (render-to-texture)

Constraints satisfied:
- NO "main camera per-pixel ray cast & triangle intersection".
- Visibility is from projection + triangle rasterization + Z-test only.
- Mirror reflection uses recursive offscreen rendering from a mirrored camera, then samples that image.

Key fixes vs the broken/dark versions:
- Add tone mapping + gamma encoding on output (prevents "too dark" or "washed out / flat").
- Use point light + ambient with face-forward normals for diffuse.
- Mirror cache keyed by plane (two triangles on a face share one reflection render).
- When rendering the mirror pass, exclude all triangles coplanar with that mirror plane to avoid self-occlusion.

Build:
  g++ -O2 -std=c++17 cornell_raster_reflect.cpp -o cornell

Run:
  ./cornell

Output:
  output.ppm (P3)
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

static constexpr float kEps = 1e-6f;
static constexpr float kInf = std::numeric_limits<float>::infinity();

struct Vec2 {
	float x = 0, y = 0;
	Vec2() = default;
	Vec2(float _x, float _y) : x(_x), y(_y) {
	}
	Vec2 operator+(const Vec2 &o) const {
		return { x + o.x, y + o.y };
	}
	Vec2 operator-(const Vec2 &o) const {
		return { x - o.x, y - o.y };
	}
	Vec2 operator*(float s) const {
		return { x * s, y * s };
	}
};

struct Vec3 {
	float x = 0, y = 0, z = 0;
	Vec3() = default;
	Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {
	}
	Vec3 operator+(const Vec3 &o) const {
		return { x + o.x, y + o.y, z + o.z };
	}
	Vec3 operator-(const Vec3 &o) const {
		return { x - o.x, y - o.y, z - o.z };
	}
	Vec3 operator*(float s) const {
		return { x * s, y * s, z * s };
	}
	Vec3 operator/(float s) const {
		return { x / s, y / s, z / s };
	}
	Vec3 &operator+=(const Vec3 &o) {
		x += o.x;
		y += o.y;
		z += o.z;
		return *this;
	}
};

static inline Vec3 operator*(float s, const Vec3 &v) {
	return v * s;
}

static inline float dot(const Vec3 &a, const Vec3 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

static inline float length(const Vec3 &v) {
	return std::sqrt(dot(v, v));
}

static inline Vec3 normalize(const Vec3 &v) {
	float len = length(v);
	if (len < kEps)
		return { 0, 0, 0 };
	return v / len;
}

static inline Vec3 clamp01(const Vec3 &c) {
	return {
		std::clamp(c.x, 0.0f, 1.0f),
		std::clamp(c.y, 0.0f, 1.0f),
		std::clamp(c.z, 0.0f, 1.0f),
	};
}

static inline Vec3 hadamard(const Vec3 &a, const Vec3 &b) {
	return { a.x * b.x, a.y * b.y, a.z * b.z };
}

static inline Vec3 reinhardTonemap(const Vec3 &c) {
	// Per-channel Reinhard; simple and stable for this demo.
	return {
		c.x / (1.0f + c.x),
		c.y / (1.0f + c.y),
		c.z / (1.0f + c.z),
	};
}

static inline float gammaEncode(float linear, float gamma = 2.2f) {
	linear = std::max(0.0f, linear);
	return std::pow(linear, 1.0f / gamma);
}

static inline Vec3 gammaEncode(const Vec3 &linear, float gamma = 2.2f) {
	return { gammaEncode(linear.x, gamma), gammaEncode(linear.y, gamma), gammaEncode(linear.z, gamma) };
}

enum class MaterialType {
	Diffuse = 0,
	Metal = 1
};

struct Material {
	MaterialType type = MaterialType::Diffuse;
	Vec3 albedo = { 1, 1, 1 }; // Diffuse base color
	float reflectance = 1.0f; // Metal reflectance [0,1]
};

struct Triangle {
	Vec3 p0, p1, p2;
	Material mat;
	Vec3 normal; // flat normal (unit)
};

static inline Vec3 computeFlatNormal(const Triangle &t) {
	return normalize(cross(t.p1 - t.p0, t.p2 - t.p0));
}

struct Plane {
	Vec3 p; // point on plane
	Vec3 n; // unit normal (oriented)
};

static inline float signedDistance(const Plane &pl, const Vec3 &x) {
	return dot(pl.n, x - pl.p);
}

static inline Vec3 reflectPointAboutPlane(const Vec3 &x, const Plane &pl) {
	float d = signedDistance(pl, x);
	return x - (2.0f * d) * pl.n;
}

static inline Vec3 reflectVectorAboutPlane(const Vec3 &v, const Plane &pl) {
	return v - (2.0f * dot(v, pl.n)) * pl.n;
}

struct Image {
	int w = 0, h = 0;
	std::vector<Vec3> pix; // linear HDR-ish buffer

	void reset(int width, int height, Vec3 clear = { 0, 0, 0 }) {
		w = width;
		h = height;
		pix.assign((size_t)w * (size_t)h, clear);
	}

	Vec3 get(int x, int y) const {
		if (x < 0 || x >= w || y < 0 || y >= h)
			return { 0, 0, 0 };
		return pix[(size_t)y * (size_t)w + (size_t)x];
	}

	void set(int x, int y, const Vec3 &c) {
		if (x < 0 || x >= w || y < 0 || y >= h)
			return;
		pix[(size_t)y * (size_t)w + (size_t)x] = c;
	}

	Vec3 sampleNearest(float u, float v) const {
		if (w <= 0 || h <= 0)
			return { 0, 0, 0 };
		int x = (int)std::floor(u * (float)w);
		int y = (int)std::floor(v * (float)h);
		x = std::clamp(x, 0, w - 1);
		y = std::clamp(y, 0, h - 1);
		return get(x, y);
	}

	void writePPM_P3(const std::string &path) const {
		std::ofstream out(path, std::ios::out);
		if (!out) {
			std::cerr << "Failed to open: " << path << "\n";
			return;
		}
		out << "P3\n"
			<< w << " " << h << "\n255\n";
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				Vec3 c = get(x, y);

				// Fix: tone mapping + gamma for display
				c = reinhardTonemap(c);
				c = gammaEncode(c, 2.2f);
				c = clamp01(c);

				int r = (int)std::round(c.x * 255.0f);
				int g = (int)std::round(c.y * 255.0f);
				int b = (int)std::round(c.z * 255.0f);
				out << r << " " << g << " " << b << "\n";
			}
		}
	}
};

struct DepthBuffer {
	int w = 0, h = 0;
	std::vector<float> z; // camera-space z, smaller=closer

	void reset(int width, int height) {
		w = width;
		h = height;
		z.assign((size_t)w * (size_t)h, kInf);
	}

	float get(int x, int y) const {
		return z[(size_t)y * (size_t)w + (size_t)x];
	}

	void set(int x, int y, float v) {
		z[(size_t)y * (size_t)w + (size_t)x] = v;
	}
};

struct RenderTarget {
	Image color;
	DepthBuffer depth;
};

struct Camera {
	Vec3 origin;
	Vec3 forward;
	Vec3 right;
	Vec3 up;

	float fovY_deg = 60.0f;
	float nearZ = 0.05f;
	float farZ = 50.0f;

	int imgW = 640;
	int imgH = 480;

	static Camera lookAt(const Vec3 &eye, const Vec3 &target, const Vec3 &upHint,
		float fovY_deg, float nearZ, float farZ,
		int w, int h) {
		Camera c;
		c.origin = eye;
		c.forward = normalize(target - eye);
		c.right = normalize(cross(c.forward, upHint));
		c.up = normalize(cross(c.right, c.forward));
		c.fovY_deg = fovY_deg;
		c.nearZ = nearZ;
		c.farZ = farZ;
		c.imgW = w;
		c.imgH = h;
		return c;
	}

	Vec3 worldToCamera(const Vec3 &p) const {
		Vec3 d = p - origin;
		return { dot(d, right), dot(d, up), dot(d, forward) }; // z>0 in front
	}

	bool projectToScreen(const Vec3 &worldP, Vec2 &outXY, float &outInvZ, float &outCamZ) const {
		Vec3 pc = worldToCamera(worldP);
		float z = pc.z;
		outCamZ = z;
		if (z <= nearZ)
			return false;

		float aspect = (float)imgW / (float)imgH;
		float fovY = fovY_deg * (3.1415926535f / 180.0f);
		float tanHalf = std::tan(fovY * 0.5f);

		// Perspective projection with vertical fov
		float ndcX = (pc.x / (z * tanHalf)) / aspect;
		float ndcY = (pc.y / (z * tanHalf));

		float px = (ndcX * 0.5f + 0.5f) * (float)imgW;
		float py = (1.0f - (ndcY * 0.5f + 0.5f)) * (float)imgH;

		outXY = { px, py };
		outInvZ = 1.0f / z;
		return true;
	}
};

struct ScreenVertex {
	Vec2 s;
	float invZ;
	Vec3 worldP;
};

static inline float edgeFunction(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

static inline float triangleAreaPixels(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	return std::abs(edgeFunction(a, b, c)) * 0.5f;
}

struct RenderConfig {
	int maxDepth = 2;
	float minMirrorAreaPixels = 6.0f;
	Vec3 clearColor = { 0.0f, 0.0f, 0.0f }; // linear clear
};

struct RenderContext {
	const std::vector<Triangle> *tris = nullptr;
	RenderConfig cfg;

	// Simple point light (no shadows)
	Vec3 lightPos = { 0.0f, 1.95f, 0.0f };
	Vec3 lightColor = { 1.0f, 1.0f, 1.0f };
	float lightIntensity = 25.0f;

	Vec3 ambient = { 0.06f, 0.06f, 0.06f };
};

static inline Vec3 faceForwardToView(Vec3 n, const Vec3 &worldP, const Camera &cam) {
	Vec3 V = normalize(cam.origin - worldP);
	if (dot(n, V) < 0.0f)
		n = n * -1.0f;
	return n;
}

static inline Vec3 shadeDiffuse(const Triangle &t,
	const Vec3 &worldP,
	const Vec3 &worldN,
	const Camera &cam,
	const RenderContext &ctx) {
	Vec3 N = faceForwardToView(worldN, worldP, cam);

	Vec3 Lvec = ctx.lightPos - worldP;
	float dist2 = std::max(kEps, dot(Lvec, Lvec));
	Vec3 L = normalize(Lvec);

	float ndotl = std::max(0.0f, dot(N, L));
	float atten = 1.0f / dist2;

	Vec3 direct = (ctx.lightIntensity * ndotl * atten) * ctx.lightColor;
	Vec3 lit = ctx.ambient + direct;

	return hadamard(t.mat.albedo, lit);
}

static inline Plane orientedPlaneTowardCamera(const Triangle &t, const Camera &cam) {
	Plane pl;
	pl.p = t.p0;
	pl.n = normalize(t.normal);
	// Ensure camera is on the positive side (for consistent mirroring orientation)
	float s = dot(pl.n, cam.origin - pl.p);
	if (s < 0.0f)
		pl.n = pl.n * -1.0f;
	return pl;
}

static inline Camera mirrorCameraAcrossPlane(const Camera &cam, const Plane &pl) {
	Camera mc = cam;
	mc.origin = reflectPointAboutPlane(cam.origin, pl);
	mc.forward = normalize(reflectVectorAboutPlane(cam.forward, pl));
	mc.up = normalize(reflectVectorAboutPlane(cam.up, pl));
	mc.right = normalize(cross(mc.forward, mc.up));
	mc.up = normalize(cross(mc.right, mc.forward));
	return mc;
}

static inline bool triangleCoplanarWithPlane(const Triangle &t, const Plane &pl) {
	// Exclude triangles on the same plane (both triangles of a quad face)
	// Use both parallel-normal test and vertex distance test.
	float parallel = std::abs(dot(normalize(t.normal), pl.n));
	if (parallel < 0.999f)
		return false;

	float d0 = std::abs(signedDistance(pl, t.p0));
	float d1 = std::abs(signedDistance(pl, t.p1));
	float d2 = std::abs(signedDistance(pl, t.p2));
	return (d0 < 1e-4f && d1 < 1e-4f && d2 < 1e-4f);
}

struct PlaneKey {
	int nx = 0, ny = 0, nz = 0;
	int d = 0; // plane equation: n·x + d = 0 (quantized)
	bool operator==(const PlaneKey &o) const {
		return nx == o.nx && ny == o.ny && nz == o.nz && d == o.d;
	}
};

struct PlaneKeyHash {
	size_t operator()(const PlaneKey &k) const noexcept {
		// Simple hash combine
		size_t h = 1469598103934665603ull;
		auto mix = [&](uint64_t v) {
			h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
		};
		mix((uint64_t)(uint32_t)k.nx);
		mix((uint64_t)(uint32_t)k.ny);
		mix((uint64_t)(uint32_t)k.nz);
		mix((uint64_t)(uint32_t)k.d);
		return h;
	}
};

static inline PlaneKey makePlaneKey(const Plane &pl) {
	// Quantize plane normal and offset for caching; oriented plane already.
	// Plane equation: n·x + d = 0 where d = -n·p
	float d = -dot(pl.n, pl.p);
	const float sN = 10000.0f;
	const float sD = 10000.0f;

	PlaneKey k;
	k.nx = (int)std::round(pl.n.x * sN);
	k.ny = (int)std::round(pl.n.y * sN);
	k.nz = (int)std::round(pl.n.z * sN);
	k.d = (int)std::round(d * sD);
	return k;
}

struct MirrorCacheEntry {
	Plane plane;
	Camera mirrorCam;
	RenderTarget rt;
	bool ready = false;
};

static RenderTarget renderPass(const Camera &cam,
	const RenderContext &ctx,
	int recursionDepth,
	const Plane *excludeCoplanarPlaneOrNull);

static RenderTarget renderPass(const Camera &cam,
	const RenderContext &ctx,
	int recursionDepth,
	const Plane *excludeCoplanarPlaneOrNull) {
	RenderTarget rt;
	rt.color.reset(cam.imgW, cam.imgH, ctx.cfg.clearColor);
	rt.depth.reset(cam.imgW, cam.imgH);

	// Cache planar reflections per plane (per pass)
	std::unordered_map<PlaneKey, MirrorCacheEntry, PlaneKeyHash> mirrorCache;
	mirrorCache.reserve(64);

	for (int ti = 0; ti < (int)ctx.tris->size(); ++ti) {
		const Triangle &t = (*ctx.tris)[ti];

		if (excludeCoplanarPlaneOrNull && triangleCoplanarWithPlane(t, *excludeCoplanarPlaneOrNull)) {
			continue;
		}

		Vec2 s0, s1, s2;
		float invZ0, invZ1, invZ2;
		float z0, z1, z2;

		bool ok0 = cam.projectToScreen(t.p0, s0, invZ0, z0);
		bool ok1 = cam.projectToScreen(t.p1, s1, invZ1, z1);
		bool ok2 = cam.projectToScreen(t.p2, s2, invZ2, z2);

		// Minimal clipping: if any vertex behind near, skip triangle
		if (!(ok0 && ok1 && ok2))
			continue;

		ScreenVertex sv[3];
		sv[0] = { s0, invZ0, t.p0 };
		sv[1] = { s1, invZ1, t.p1 };
		sv[2] = { s2, invZ2, t.p2 };

		float areaPx = triangleAreaPixels(sv[0].s, sv[1].s, sv[2].s);

		float minXf = std::floor(std::min({ sv[0].s.x, sv[1].s.x, sv[2].s.x }));
		float maxXf = std::ceil(std::max({ sv[0].s.x, sv[1].s.x, sv[2].s.x }));
		float minYf = std::floor(std::min({ sv[0].s.y, sv[1].s.y, sv[2].s.y }));
		float maxYf = std::ceil(std::max({ sv[0].s.y, sv[1].s.y, sv[2].s.y }));

		int minX = std::clamp((int)minXf, 0, cam.imgW - 1);
		int maxX = std::clamp((int)maxXf, 0, cam.imgW - 1);
		int minY = std::clamp((int)minYf, 0, cam.imgH - 1);
		int maxY = std::clamp((int)maxYf, 0, cam.imgH - 1);

		float triArea = edgeFunction(sv[0].s, sv[1].s, sv[2].s);
		if (std::abs(triArea) < kEps)
			continue;
		float invTriArea = 1.0f / triArea;

		for (int y = minY; y <= maxY; ++y) {
			for (int x = minX; x <= maxX; ++x) {
				Vec2 p = { (float)x + 0.5f, (float)y + 0.5f };

				float w0 = edgeFunction(sv[1].s, sv[2].s, p) * invTriArea;
				float w1 = edgeFunction(sv[2].s, sv[0].s, p) * invTriArea;
				float w2 = edgeFunction(sv[0].s, sv[1].s, p) * invTriArea;

				if (w0 < -kEps || w1 < -kEps || w2 < -kEps)
					continue;

				// Perspective-correct interpolation: invZ at pixel
				float invZ = w0 * sv[0].invZ + w1 * sv[1].invZ + w2 * sv[2].invZ;
				if (invZ <= kEps)
					continue;
				float camZ = 1.0f / invZ;

				float oldZ = rt.depth.get(x, y);
				if (camZ >= oldZ)
					continue;

				// Interpolate world position perspective-correctly
				Vec3 worldP = (w0 * sv[0].worldP * sv[0].invZ + w1 * sv[1].worldP * sv[1].invZ + w2 * sv[2].worldP * sv[2].invZ) / invZ;

				rt.depth.set(x, y, camZ);

				Vec3 outLinear { 0, 0, 0 };

				if (t.mat.type == MaterialType::Diffuse) {
					outLinear = shadeDiffuse(t, worldP, t.normal, cam, ctx);
				} else {
					float refl = std::clamp(t.mat.reflectance, 0.0f, 1.0f);

					bool canRecurse = (refl > 0.0f) && (recursionDepth < ctx.cfg.maxDepth) && (areaPx >= ctx.cfg.minMirrorAreaPixels);

					Vec3 base = shadeDiffuse(t, worldP, t.normal, cam, ctx);

					if (!canRecurse) {
						outLinear = base;
					} else {
						Plane pl = orientedPlaneTowardCamera(t, cam);
						PlaneKey key = makePlaneKey(pl);

						auto it = mirrorCache.find(key);
						if (it == mirrorCache.end()) {
							MirrorCacheEntry entry;
							entry.plane = pl;
							entry.mirrorCam = mirrorCameraAcrossPlane(cam, pl);

							// Render mirrored pass, excluding this mirror plane (avoid self)
							entry.rt = renderPass(entry.mirrorCam, ctx, recursionDepth + 1, &entry.plane);
							entry.ready = true;

							it = mirrorCache.emplace(key, std::move(entry)).first;
						}

						const MirrorCacheEntry &entry = it->second;

						Vec2 rxy;
						float rinvZ, rcamZ;
						if (entry.ready && entry.mirrorCam.projectToScreen(worldP, rxy, rinvZ, rcamZ)) {
							float u = rxy.x / (float)entry.rt.color.w;
							float v = rxy.y / (float)entry.rt.color.h;

							if (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f) {
								Vec3 sample = entry.rt.color.sampleNearest(u, v);
								// Ideal mirror: mostly reflection; keep tiny base to avoid "reflection outside screen => black"
								float baseKeep = 0.08f;
								outLinear = sample * refl + base * (1.0f - refl);
								outLinear = outLinear + baseKeep * base;
							} else {
								outLinear = base;
							}
						} else {
							outLinear = base;
						}
					}
				}

				rt.color.set(x, y, outLinear);
			}
		}
	}

	return rt;
}

// --- Scene building helpers ---

static void addQuad(std::vector<Triangle> &tris,
	const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d,
	const Material &mat,
	bool ccw = true) {
	Triangle t1, t2;
	if (ccw) {
		t1.p0 = a;
		t1.p1 = b;
		t1.p2 = c;
		t2.p0 = a;
		t2.p1 = c;
		t2.p2 = d;
	} else {
		t1.p0 = a;
		t1.p1 = c;
		t1.p2 = b;
		t2.p0 = a;
		t2.p1 = d;
		t2.p2 = c;
	}
	t1.mat = mat;
	t2.mat = mat;
	t1.normal = computeFlatNormal(t1);
	t2.normal = computeFlatNormal(t2);
	tris.push_back(t1);
	tris.push_back(t2);
}

static void addBox(std::vector<Triangle> &tris,
	const Vec3 &mn, const Vec3 &mx,
	const Material &mat) {
	Vec3 p000 { mn.x, mn.y, mn.z };
	Vec3 p001 { mn.x, mn.y, mx.z };
	Vec3 p010 { mn.x, mx.y, mn.z };
	Vec3 p011 { mn.x, mx.y, mx.z };
	Vec3 p100 { mx.x, mn.y, mn.z };
	Vec3 p101 { mx.x, mn.y, mx.z };
	Vec3 p110 { mx.x, mx.y, mn.z };
	Vec3 p111 { mx.x, mx.y, mx.z };

	addQuad(tris, p000, p001, p011, p010, mat, true); // -X
	addQuad(tris, p100, p110, p111, p101, mat, true); // +X
	addQuad(tris, p000, p100, p101, p001, mat, true); // -Y
	addQuad(tris, p010, p011, p111, p110, mat, true); // +Y
	addQuad(tris, p000, p010, p110, p100, mat, true); // -Z
	addQuad(tris, p001, p101, p111, p011, mat, true); // +Z
}

static std::vector<Triangle> buildCornellBoxScene() {
	std::vector<Triangle> tris;
	tris.reserve(256);

	// Room: x in [-1,1], y in [0,2], z in [-1,1]; front is open (no wall at z=+1)
	float xmin = -1.0f, xmax = 1.0f;
	float ymin = 0.0f, ymax = 2.0f;
	float zmin = -1.0f, zmax = 1.0f;

	Material white { MaterialType::Diffuse, { 0.85f, 0.85f, 0.85f }, 0.0f };
	Material red { MaterialType::Diffuse, { 0.85f, 0.20f, 0.20f }, 0.0f };
	Material green { MaterialType::Diffuse, { 0.20f, 0.85f, 0.20f }, 0.0f };

	// Floor (y=ymin)
	addQuad(tris,
		{ xmin, ymin, zmin }, { xmax, ymin, zmin }, { xmax, ymin, zmax }, { xmin, ymin, zmax },
		white, true);

	// Ceiling (y=ymax)
	addQuad(tris,
		{ xmin, ymax, zmax }, { xmax, ymax, zmax }, { xmax, ymax, zmin }, { xmin, ymax, zmin },
		white, true);

	// Back wall (z=zmin)
	addQuad(tris,
		{ xmin, ymin, zmin }, { xmin, ymax, zmin }, { xmax, ymax, zmin }, { xmax, ymin, zmin },
		white, true);

	// Left wall (x=xmin)
	addQuad(tris,
		{ xmin, ymin, zmax }, { xmin, ymax, zmax }, { xmin, ymax, zmin }, { xmin, ymin, zmin },
		red, true);

	// Right wall (x=xmax)
	addQuad(tris,
		{ xmax, ymin, zmin }, { xmax, ymax, zmin }, { xmax, ymax, zmax }, { xmax, ymin, zmax },
		green, true);

	// Two metal boxes
	Material blueMetal { MaterialType::Metal, { 0.10f, 0.20f, 0.95f }, 1.0f };
	Material yellowMetal { MaterialType::Metal, { 0.95f, 0.85f, 0.10f }, 1.0f };

	addBox(tris,
		Vec3 { -0.75f, 0.0f, -0.20f },
		Vec3 { -0.25f, 0.65f, 0.35f },
		blueMetal);

	addBox(tris,
		Vec3 { 0.20f, 0.0f, -0.70f },
		Vec3 { 0.75f, 1.05f, -0.10f },
		yellowMetal);

	for (auto &t : tris)
		t.normal = computeFlatNormal(t);
	return tris;
}

int main() {
	const int W = 520;
	const int H = 520;

	std::vector<Triangle> scene = buildCornellBoxScene();

	// Keep camera outside-ish but close; perspective should be obvious
	Camera cam = Camera::lookAt(
		/*eye*/ Vec3 { 0.0f, 1.0f, 2.7f },
		/*target*/ Vec3 { 0.0f, 1.0f, 0.0f },
		/*up*/ Vec3 { 0.0f, 1.0f, 0.0f },
		/*fov*/ 55.0f,
		/*near*/ 0.05f,
		/*far*/ 50.0f,
		/*w,h*/ W, H);

	RenderContext ctx;
	ctx.tris = &scene;

	ctx.cfg.maxDepth = 2;
	ctx.cfg.minMirrorAreaPixels = 8.0f;
	ctx.cfg.clearColor = { 0.0f, 0.0f, 0.0f };

	// Light near ceiling center
	ctx.lightPos = { 0.0f, 1.95f, -0.1f };
	ctx.lightColor = { 1.0f, 1.0f, 1.0f };
	ctx.lightIntensity = 30.0f;
	ctx.ambient = { 0.06f, 0.06f, 0.06f };

	RenderTarget rt = renderPass(cam, ctx, /*recDepth*/ 0, /*excludePlane*/ nullptr);
	rt.color.writePPM_P3("output.ppm");

	std::cout << "Wrote output.ppm (" << W << "x" << H << ")\n";
	std::cout << "Convert if needed:\n";
	std::cout << "  magick output.ppm output.png\n";
	return 0;
}
