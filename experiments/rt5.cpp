// cornell_patchstep_fixed.cpp
// g++ -std=c++17 -O2 cornell_patchstep_fixed.cpp -o render && ./render
// 输出：out.ppm
//
// 修复点（对应你的反馈）：
// - 之前递归时“下一级 viewport 在最终图片中占哪块区域”的信息虽然隐含在 vm.screenUV/clip 中，
//   但逻辑上不够显式，容易在扩展/改动时导致下级按“全屏”比例写入。
// - 现在为 renderTriangleWithTriangle / renderViewport 显式增加参数：
//   `viewportRegion` = 当前 viewport 在最终图片中的屏幕UV三角形(3个二维坐标)。
// - 所有“投影到屏幕”的计算都通过 viewportRegion 完成；所有写入也强制 clip 在 viewportRegion 内。
// - 向下递归时，会根据“当前三角形在当前 viewport 上的投影 triScreenUV”计算出下一级 viewportRegion 并传入。

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

static constexpr double kEps = 1e-9;

static inline double clamp01(double x) {
	return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
}
static inline int clampi(int x, int lo, int hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}

struct Vec2 {
	double x = 0, y = 0;
	Vec2() = default;
	Vec2(double xx, double yy) : x(xx), y(yy) {
	}
	Vec2 operator+(const Vec2 &o) const {
		return Vec2 { x + o.x, y + o.y };
	}
	Vec2 operator-(const Vec2 &o) const {
		return Vec2 { x - o.x, y - o.y };
	}
	Vec2 operator*(double s) const {
		return Vec2 { x * s, y * s };
	}
};

static inline double dot(const Vec2 &a, const Vec2 &b) {
	return a.x * b.x + a.y * b.y;
}
static inline double cross2(const Vec2 &a, const Vec2 &b) {
	return a.x * b.y - a.y * b.x;
}

struct Vec3 {
	double x = 0, y = 0, z = 0;
	Vec3() = default;
	Vec3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {
	}
	Vec3 operator+(const Vec3 &o) const {
		return Vec3 { x + o.x, y + o.y, z + o.z };
	}
	Vec3 operator-(const Vec3 &o) const {
		return Vec3 { x - o.x, y - o.y, z - o.z };
	}
	Vec3 operator*(double s) const {
		return Vec3 { x * s, y * s, z * s };
	}
	Vec3 operator/(double s) const {
		return Vec3 { x / s, y / s, z / s };
	}
};

static inline Vec3 operator*(double s, const Vec3 &v) {
	return v * s;
}
static inline double dot(const Vec3 &a, const Vec3 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
	return Vec3 {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x,
	};
}
static inline double length(const Vec3 &v) {
	return std::sqrt(dot(v, v));
}
static inline Vec3 normalize(const Vec3 &v) {
	double len = length(v);
	if (len < kEps)
		return Vec3 { 0, 0, 0 };
	return v / len;
}

struct Plane {
	Vec3 n; // unit normal
	double d; // n·x + d = 0
};

static inline Plane planeFromTriangle(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
	Vec3 n = normalize(cross(b - a, c - a));
	double d = -dot(n, a);
	return Plane { n, d };
}

static inline double planeSignedDistance(const Plane &p, const Vec3 &x) {
	return dot(p.n, x) + p.d;
}

static inline Vec3 reflectPointAcrossPlane(const Vec3 &point, const Plane &p) {
	double dist = planeSignedDistance(p, point);
	return point - 2.0 * dist * p.n;
}

struct Basis2DOnPlane {
	Vec3 origin;
	Vec3 ux;
	Vec3 uy;
	Vec3 n;
};

static inline Basis2DOnPlane makePlaneBasis(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
	Vec3 n = normalize(cross(b - a, c - a));
	Vec3 ux = normalize(b - a);
	Vec3 uy = normalize(cross(n, ux));
	return Basis2DOnPlane { a, ux, uy, n };
}

static inline Vec2 to2D(const Basis2DOnPlane &basis, const Vec3 &p) {
	Vec3 v = p - basis.origin;
	return Vec2 { dot(v, basis.ux), dot(v, basis.uy) };
}

static inline Vec3 barycentric2D(const Vec2 &p, const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	Vec2 v0 = b - a;
	Vec2 v1 = c - a;
	Vec2 v2 = p - a;
	double den = cross2(v0, v1);
	if (std::fabs(den) < kEps)
		return Vec3 { -1, -1, -1 };
	double v = cross2(v2, v1) / den;
	double w = cross2(v0, v2) / den;
	double u = 1.0 - v - w;
	return Vec3 { u, v, w };
}

static inline bool pointInTri2D(const Vec2 &p, const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	Vec3 bc = barycentric2D(p, a, b, c);
	return bc.x >= -1e-12 && bc.y >= -1e-12 && bc.z >= -1e-12;
}

static inline bool segIntersect2D(const Vec2 &a, const Vec2 &b, const Vec2 &c, const Vec2 &d) {
	auto orient = [](const Vec2 &p, const Vec2 &q, const Vec2 &r) {
		return cross2(q - p, r - p);
	};
	double o1 = orient(a, b, c);
	double o2 = orient(a, b, d);
	double o3 = orient(c, d, a);
	double o4 = orient(c, d, b);

	auto onSeg = [](const Vec2 &p, const Vec2 &q, const Vec2 &r) {
		return std::min(p.x, r.x) - 1e-12 <= q.x && q.x <= std::max(p.x, r.x) + 1e-12 && std::min(p.y, r.y) - 1e-12 <= q.y && q.y <= std::max(p.y, r.y) + 1e-12;
	};

	if ((o1 * o2 < 0) && (o3 * o4 < 0))
		return true;
	if (std::fabs(o1) < 1e-12 && onSeg(a, c, b))
		return true;
	if (std::fabs(o2) < 1e-12 && onSeg(a, d, b))
		return true;
	if (std::fabs(o3) < 1e-12 && onSeg(c, a, d))
		return true;
	if (std::fabs(o4) < 1e-12 && onSeg(c, b, d))
		return true;
	return false;
}

static inline bool triOverlap2D(const std::array<Vec2, 3> &t0, const std::array<Vec2, 3> &t1) {
	for (int i = 0; i < 3; i++) {
		if (pointInTri2D(t0[i], t1[0], t1[1], t1[2]))
			return true;
		if (pointInTri2D(t1[i], t0[0], t0[1], t0[2]))
			return true;
	}
	for (int i = 0; i < 3; i++) {
		Vec2 a0 = t0[i], b0 = t0[(i + 1) % 3];
		for (int j = 0; j < 3; j++) {
			Vec2 a1 = t1[j], b1 = t1[(j + 1) % 3];
			if (segIntersect2D(a0, b0, a1, b1))
				return true;
		}
	}
	return false;
}

// 3D 三角形相交（满足你“Triangle类及相关运算”的要求；渲染主流程不依赖它）
static inline void triAABB(const Vec3 &a, const Vec3 &b, const Vec3 &c, Vec3 &mn, Vec3 &mx) {
	mn = Vec3 { std::min({ a.x, b.x, c.x }), std::min({ a.y, b.y, c.y }), std::min({ a.z, b.z, c.z }) };
	mx = Vec3 { std::max({ a.x, b.x, c.x }), std::max({ a.y, b.y, c.y }), std::max({ a.z, b.z, c.z }) };
}

static inline bool aabbOverlap(const Vec3 &mn0, const Vec3 &mx0, const Vec3 &mn1, const Vec3 &mx1) {
	return (mn0.x <= mx1.x + 1e-12 && mx0.x + 1e-12 >= mn1.x) && (mn0.y <= mx1.y + 1e-12 && mx0.y + 1e-12 >= mn1.y) && (mn0.z <= mx1.z + 1e-12 && mx0.z + 1e-12 >= mn1.z);
}

static inline bool segmentTriangleIntersect(const Vec3 &p0, const Vec3 &p1,
	const Vec3 &a, const Vec3 &b, const Vec3 &c,
	double &tOut) {
	Vec3 dir = p1 - p0;
	Vec3 e1 = b - a;
	Vec3 e2 = c - a;
	Vec3 pvec = cross(dir, e2);
	double det = dot(e1, pvec);
	if (std::fabs(det) < 1e-12)
		return false;
	double invDet = 1.0 / det;

	Vec3 tvec = p0 - a;
	double u = dot(tvec, pvec) * invDet;
	if (u < -1e-12 || u > 1.0 + 1e-12)
		return false;

	Vec3 qvec = cross(tvec, e1);
	double v = dot(dir, qvec) * invDet;
	if (v < -1e-12 || u + v > 1.0 + 1e-12)
		return false;

	double t = dot(e2, qvec) * invDet;
	if (t < -1e-12 || t > 1.0 + 1e-12)
		return false;
	tOut = t;
	return true;
}

static inline bool pointInTriangle3D(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
	Vec3 n = cross(b - a, c - a);
	Vec3 n0 = cross(b - a, p - a);
	Vec3 n1 = cross(c - b, p - b);
	Vec3 n2 = cross(a - c, p - c);
	if (dot(n, n0) < -1e-12)
		return false;
	if (dot(n, n1) < -1e-12)
		return false;
	if (dot(n, n2) < -1e-12)
		return false;
	return true;
}

static inline bool triTriIntersect3D(const Vec3 &a0, const Vec3 &b0, const Vec3 &c0,
	const Vec3 &a1, const Vec3 &b1, const Vec3 &c1) {
	Vec3 mn0, mx0, mn1, mx1;
	triAABB(a0, b0, c0, mn0, mx0);
	triAABB(a1, b1, c1, mn1, mx1);
	if (!aabbOverlap(mn0, mx0, mn1, mx1))
		return false;

	Plane p0 = planeFromTriangle(a0, b0, c0);
	Plane p1 = planeFromTriangle(a1, b1, c1);

	if (std::fabs(dot(p0.n, p1.n)) > 1.0 - 1e-7 && std::fabs(planeSignedDistance(p0, a1)) < 1e-7) {
		Vec3 nn = p0.n;
		int drop = 0;
		double ax = std::fabs(nn.x), ay = std::fabs(nn.y), az = std::fabs(nn.z);
		if (ay > ax && ay > az)
			drop = 1;
		else if (az > ax && az > ay)
			drop = 2;

		auto proj = [&](const Vec3 &v) -> Vec2 {
			if (drop == 0)
				return Vec2 { v.y, v.z };
			if (drop == 1)
				return Vec2 { v.x, v.z };
			return Vec2 { v.x, v.y };
		};

		std::array<Vec2, 3> t0 { proj(a0), proj(b0), proj(c0) };
		std::array<Vec2, 3> t1 { proj(a1), proj(b1), proj(c1) };
		return triOverlap2D(t0, t1);
	}

	const Vec3 t0v[3] = { a0, b0, c0 };
	const Vec3 t1v[3] = { a1, b1, c1 };
	for (int i = 0; i < 3; i++) {
		double t = 0;
		if (segmentTriangleIntersect(t0v[i], t0v[(i + 1) % 3], a1, b1, c1, t))
			return true;
		if (segmentTriangleIntersect(t1v[i], t1v[(i + 1) % 3], a0, b0, c0, t))
			return true;
	}

	if (pointInTriangle3D(a0, a1, b1, c1))
		return true;
	if (pointInTriangle3D(a1, a0, b0, c0))
		return true;
	return false;
}

struct Image {
	int w = 0, h = 0;
	std::vector<Vec3> pix;

	Image(int ww, int hh) : w(ww), h(hh), pix((size_t)ww * (size_t)hh, Vec3 { 0, 0, 0 }) {
	}

	void blendOver(int x, int y, const Vec3 &src, double alpha) {
		if (x < 0 || x >= w || y < 0 || y >= h)
			return;
		alpha = clamp01(alpha);
		Vec3 dst = pix[(size_t)y * (size_t)w + (size_t)x];
		Vec3 out = dst * (1.0 - alpha) + src * alpha;
		pix[(size_t)y * (size_t)w + (size_t)x] = out;
	}

	void add(int x, int y, const Vec3 &src) {
		if (x < 0 || x >= w || y < 0 || y >= h)
			return;
		Vec3 dst = pix[(size_t)y * (size_t)w + (size_t)x];
		pix[(size_t)y * (size_t)w + (size_t)x] = Vec3 { dst.x + src.x, dst.y + src.y, dst.z + src.z };
	}

	void writePPM(const std::string &path) const {
		std::ofstream out(path, std::ios::binary);
		if (!out) {
			std::cerr << "Failed to open output file: " << path << "\n";
			return;
		}
		out << "P6\n"
			<< w << " " << h << "\n255\n";
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				Vec3 c = pix[(size_t)y * (size_t)w + (size_t)x];
				c.x = clamp01(c.x);
				c.y = clamp01(c.y);
				c.z = clamp01(c.z);
				auto toByte = [](double v) -> unsigned char {
					v = std::pow(clamp01(v), 1.0 / 2.2);
					int iv = (int)std::lround(v * 255.0);
					return (unsigned char)clampi(iv, 0, 255);
				};
				unsigned char rgb[3] = { toByte(c.x), toByte(c.y), toByte(c.z) };
				out.write((char *)rgb, 3);
			}
		}
	}
};

enum class MaterialType {
	Curtain,
	Diffuse,
	Metal
};

struct Triangle {
	int id = -1;
	std::string name;

	Vec3 p[3];
	Vec2 uv[3];

	MaterialType mat = MaterialType::Diffuse;
	Vec3 baseColor { 1, 1, 1 };

	Vec3 normal() const {
		return normalize(cross(p[1] - p[0], p[2] - p[0]));
	}
	Vec3 centroid() const {
		return (p[0] + p[1] + p[2]) / 3.0;
	}
};

struct ProjectedVertex {
	bool ok = false;
	Vec3 onPlane;
	double t = 0.0;
};

static inline ProjectedVertex projectPointToPlaneAlongOrigin(const Vec3 &origin, const Vec3 &v, const Plane &plane) {
	Vec3 dir = v - origin;
	double denom = dot(plane.n, dir);
	if (std::fabs(denom) < 1e-12)
		return ProjectedVertex { false, Vec3 {}, 0 };
	double t = -(dot(plane.n, origin) + plane.d) / denom;
	if (t <= 1e-9)
		return ProjectedVertex { false, Vec3 {}, t };
	Vec3 p = origin + dir * t;
	return ProjectedVertex { true, p, t };
}

// 显式的“当前 viewport 在最终图片中的范围”
// 用屏幕UV三角形表达（更贴合你描述的“两个三角形幕布”与递归中“面片->面片”的映射）
using ScreenTri = std::array<Vec2, 3>;

static inline bool pointInScreenTri(const ScreenTri &tri, const Vec2 &uv) {
	return pointInTri2D(uv, tri[0], tri[1], tri[2]);
}

static inline Vec2 uvFromPixelCenter(int x, int y, int w, int h) {
	double u = ((double)x + 0.5) / (double)w;
	double v = 1.0 - (((double)y + 0.5) / (double)h);
	return Vec2 { u, v };
}

static inline std::array<Vec2, 3> projectedTriangleToViewport2D(const Basis2DOnPlane &basis,
	const std::array<Vec3, 3> &proj) {
	return { to2D(basis, proj[0]), to2D(basis, proj[1]), to2D(basis, proj[2]) };
}

static inline std::array<Vec2, 3> viewportTriangleTo2D(const Basis2DOnPlane &basis, const Triangle &viewport) {
	return {
		to2D(basis, viewport.p[0]),
		to2D(basis, viewport.p[1]),
		to2D(basis, viewport.p[2]),
	};
}

static inline bool projectTriangleToViewport(const Triangle &tri,
	const Vec3 &origin,
	const Triangle &viewport,
	const Basis2DOnPlane &viewportBasis,
	const ScreenTri &viewportRegion,
	std::array<Vec3, 3> &projOnViewportPlane,
	ScreenTri &triScreenUV) {
	Plane plane = planeFromTriangle(viewport.p[0], viewport.p[1], viewport.p[2]);

	ProjectedVertex pv[3];
	for (int i = 0; i < 3; i++) {
		pv[i] = projectPointToPlaneAlongOrigin(origin, tri.p[i], plane);
		if (!pv[i].ok)
			return false;
		projOnViewportPlane[i] = pv[i].onPlane;
	}

	auto proj2D = projectedTriangleToViewport2D(viewportBasis, projOnViewportPlane);
	auto vp2D = viewportTriangleTo2D(viewportBasis, viewport);
	if (!triOverlap2D(proj2D, vp2D))
		return false;

	// 用 viewport 顶点 <-> viewportRegion 的三点对应，建立“viewport平面 -> 屏幕UV”的仿射映射
	Vec2 A2 = vp2D[0], B2 = vp2D[1], C2 = vp2D[2];
	for (int i = 0; i < 3; i++) {
		Vec3 bc = barycentric2D(proj2D[i], A2, B2, C2);
		Vec2 uv = viewportRegion[0] * bc.x + viewportRegion[1] * bc.y + viewportRegion[2] * bc.z;
		triScreenUV[i] = uv;
	}
	return true;
}

static inline double triAreaPixels(const ScreenTri &screenUV, int w, int h) {
	Vec2 a = Vec2 { screenUV[0].x * w, screenUV[0].y * h };
	Vec2 b = Vec2 { screenUV[1].x * w, screenUV[1].y * h };
	Vec2 c = Vec2 { screenUV[2].x * w, screenUV[2].y * h };
	double area2 = std::fabs(cross2(b - a, c - a));
	return area2 * 0.5;
}

static inline Vec3 sampleProceduralTexture(const Triangle &tri, const Vec2 &uv) {
	double u = uv.x, v = uv.y;
	double s = 10.0;
	int iu = (int)std::floor(u * s);
	int iv = (int)std::floor(v * s);
	bool checker = ((iu + iv) & 1) == 0;

	Vec3 base = tri.baseColor;
	if (tri.mat == MaterialType::Metal) {
		double stripe = 0.6 + 0.4 * std::sin((u * 40.0 + v * 15.0) * 3.14159);
		base = base * stripe;
	} else {
		double k = checker ? 1.0 : 0.85;
		base = base * k;
	}
	return base;
}

static inline Vec3 shadeLambert(const Vec3 &color, const Vec3 &n) {
	Vec3 lightDir = normalize(Vec3 { -0.35, 0.85, 0.25 });
	double ndl = std::max(0.0, dot(n, lightDir));
	double amb = 0.20;
	double diff = 0.90 * ndl;
	return color * (amb + diff);
}

static inline void rasterizeTriangleToImage(Image &img,
	const ScreenTri &triScreenUV,
	const Triangle &tri,
	const ScreenTri &viewportRegion,
	const Vec3 &throughput,
	double alpha,
	bool additive) {
	// 强制只写入 viewportRegion 覆盖的屏幕区域
	// triScreenUV 是“当前 tri 投影到当前 viewport 后，再映射到最终屏幕”的三角形

	double minU = std::min({ triScreenUV[0].x, triScreenUV[1].x, triScreenUV[2].x });
	double maxU = std::max({ triScreenUV[0].x, triScreenUV[1].x, triScreenUV[2].x });
	double minV = std::min({ triScreenUV[0].y, triScreenUV[1].y, triScreenUV[2].y });
	double maxV = std::max({ triScreenUV[0].y, triScreenUV[1].y, triScreenUV[2].y });

	int x0 = clampi((int)std::floor(minU * img.w) - 1, 0, img.w - 1);
	int x1 = clampi((int)std::ceil(maxU * img.w) + 1, 0, img.w - 1);
	int y0 = clampi((int)std::floor((1.0 - maxV) * img.h) - 1, 0, img.h - 1);
	int y1 = clampi((int)std::ceil((1.0 - minV) * img.h) + 1, 0, img.h - 1);

	Vec2 a = triScreenUV[0], b = triScreenUV[1], c = triScreenUV[2];
	Vec3 n = tri.normal();

	for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			Vec2 suv = uvFromPixelCenter(x, y, img.w, img.h);

			// 关键：限制写入区域为当前 viewportRegion
			if (!pointInScreenTri(viewportRegion, suv))
				continue;

			Vec3 bc = barycentric2D(suv, a, b, c);
			if (bc.x < -1e-12 || bc.y < -1e-12 || bc.z < -1e-12)
				continue;

			Vec2 uv = tri.uv[0] * bc.x + tri.uv[1] * bc.y + tri.uv[2] * bc.z;

			Vec3 tex = sampleProceduralTexture(tri, uv);
			Vec3 lit = shadeLambert(tex, n);
			Vec3 src = Vec3 { lit.x * throughput.x, lit.y * throughput.y, lit.z * throughput.z };

			if (additive)
				img.add(x, y, src * alpha);
			else
				img.blendOver(x, y, src, alpha);
		}
	}
}

struct RenderConfig {
	int maxDepth = 3;
	double minAreaPixels = 3.0;
};

static void renderViewport(const std::vector<Triangle> &scene,
	const Vec3 &origin,
	const Triangle &viewport,
	const Basis2DOnPlane &viewportBasis,
	const ScreenTri &viewportRegion,
	Image &out,
	const RenderConfig &cfg,
	int depth,
	const Vec3 &throughput,
	int excludeId);

static inline double fresnelSchlick(double cosTheta, double f0) {
	cosTheta = clamp01(cosTheta);
	return f0 + (1.0 - f0) * std::pow(1.0 - cosTheta, 5.0);
}

static void renderTriangleWithTriangle(const std::vector<Triangle> &scene,
	const Vec3 &origin,
	const Triangle &viewport,
	const Basis2DOnPlane &viewportBasis,
	const ScreenTri &viewportRegion,
	const Triangle &tri,
	Image &out,
	const RenderConfig &cfg,
	int depth,
	const Vec3 &throughput,
	int excludeId) {
	if (tri.id == excludeId)
		return;
	if (tri.mat == MaterialType::Curtain)
		return;

	std::array<Vec3, 3> projOnPlane {};
	ScreenTri triScreenUV {};
	if (!projectTriangleToViewport(tri, origin, viewport, viewportBasis, viewportRegion, projOnPlane, triScreenUV))
		return;

	double areaPix = triAreaPixels(triScreenUV, out.w, out.h);
	if (areaPix < 0.25)
		return;

	if (tri.mat == MaterialType::Diffuse) {
		// 漫反射：直接贴到最终屏幕，但写入范围必须受 viewportRegion 限制（已在 rasterize 内处理）
		rasterizeTriangleToImage(out, triScreenUV, tri, viewportRegion, throughput, 1.0, false);
		return;
	}

	if (tri.mat == MaterialType::Metal) {
		// 金属：先贴自身底色，再递归把反射结果“写到它在最终屏幕上的 triScreenUV 区域里”
		Vec3 n = tri.normal();
		Vec3 viewDir = normalize(origin - tri.centroid());
		double cosTheta = std::fabs(dot(n, viewDir));
		double reflectK = 0.75;
		double f0 = 0.90;
		double F = fresnelSchlick(cosTheta, f0);
		double refl = clamp01(reflectK * F);

		rasterizeTriangleToImage(out, triScreenUV, tri, viewportRegion, throughput, 1.0, false);

		if (depth >= cfg.maxDepth)
			return;
		if (areaPix < cfg.minAreaPixels)
			return;

		Plane triPlane = planeFromTriangle(tri.p[0], tri.p[1], tri.p[2]);
		Vec3 reflectedOrigin = reflectPointAcrossPlane(origin, triPlane);

		// 关键：下一级 viewport 就是 tri（世界空间），而它在最终图片中的区域就是 triScreenUV
		const Triangle &childViewport = tri;
		Basis2DOnPlane childBasis = makePlaneBasis(childViewport.p[0], childViewport.p[1], childViewport.p[2]);
		ScreenTri childRegion = triScreenUV;

		Vec3 throughputChild = Vec3 { throughput.x * refl, throughput.y * refl, throughput.z * refl };

		renderViewport(scene, reflectedOrigin, childViewport, childBasis, childRegion, out, cfg, depth + 1, throughputChild, tri.id);
		return;
	}
}

static void renderViewport(const std::vector<Triangle> &scene,
	const Vec3 &origin,
	const Triangle &viewport,
	const Basis2DOnPlane &viewportBasis,
	const ScreenTri &viewportRegion,
	Image &out,
	const RenderConfig &cfg,
	int depth,
	const Vec3 &throughput,
	int excludeId) {
	struct Cand {
		const Triangle *t;
		double dist2;
	};
	std::vector<Cand> cands;
	cands.reserve(scene.size());

	for (const Triangle &tri : scene) {
		if (tri.id == excludeId)
			continue;
		if (tri.mat == MaterialType::Curtain)
			continue;

		std::array<Vec3, 3> projOnPlane {};
		ScreenTri triScreenUV {};
		if (!projectTriangleToViewport(tri, origin, viewport, viewportBasis, viewportRegion, projOnPlane, triScreenUV))
			continue;

		Vec3 c = tri.centroid();
		Vec3 d = c - origin;
		double dist2 = dot(d, d);
		cands.push_back(Cand { &tri, dist2 });
	}

	std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) {
		return a.dist2 > b.dist2;
	});

	for (const Cand &c : cands) {
		renderTriangleWithTriangle(scene, origin, viewport, viewportBasis, viewportRegion, *c.t, out, cfg, depth, throughput, excludeId);
	}
}

struct Camera {
	Vec3 origin;
	Triangle curtainA;
	Triangle curtainB;

	void render(const std::vector<Triangle> &scene, Image &out) const {
		RenderConfig cfg;
		cfg.maxDepth = 3;
		cfg.minAreaPixels = 3.0;

		// Curtain A：viewportRegion 就是它本身的 uv（三角形覆盖屏幕的一半）
		{
			const Triangle &viewport = curtainA;
			Basis2DOnPlane basis = makePlaneBasis(viewport.p[0], viewport.p[1], viewport.p[2]);
			ScreenTri region = { viewport.uv[0], viewport.uv[1], viewport.uv[2] };
			renderViewport(scene, origin, viewport, basis, region, out, cfg, 0, Vec3 { 1, 1, 1 }, -1);
		}

		// Curtain B
		{
			const Triangle &viewport = curtainB;
			Basis2DOnPlane basis = makePlaneBasis(viewport.p[0], viewport.p[1], viewport.p[2]);
			ScreenTri region = { viewport.uv[0], viewport.uv[1], viewport.uv[2] };
			renderViewport(scene, origin, viewport, basis, region, out, cfg, 0, Vec3 { 1, 1, 1 }, -1);
		}
	}
};

static inline Triangle makeTri(int id, const std::string &name,
	const Vec3 &a, const Vec3 &b, const Vec3 &c,
	const Vec2 &ua, const Vec2 &ub, const Vec2 &uc,
	MaterialType mat, const Vec3 &color) {
	Triangle t;
	t.id = id;
	t.name = name;
	t.p[0] = a;
	t.p[1] = b;
	t.p[2] = c;
	t.uv[0] = ua;
	t.uv[1] = ub;
	t.uv[2] = uc;
	t.mat = mat;
	t.baseColor = color;
	return t;
}

static inline void addQuad(std::vector<Triangle> &out, int &nextId, const std::string &name,
	const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d,
	const Vec2 &ua, const Vec2 &ub, const Vec2 &uc, const Vec2 &ud,
	MaterialType mat, const Vec3 &color) {
	out.push_back(makeTri(nextId++, name + "_t0", a, b, c, ua, ub, uc, mat, color));
	out.push_back(makeTri(nextId++, name + "_t1", a, c, d, ua, uc, ud, mat, color));
}

static inline void addBox(std::vector<Triangle> &out, int &nextId, const std::string &name,
	const Vec3 &mn, const Vec3 &mx,
	MaterialType mat, const Vec3 &color) {
	Vec3 p000 { mn.x, mn.y, mn.z };
	Vec3 p001 { mn.x, mn.y, mx.z };
	Vec3 p010 { mn.x, mx.y, mn.z };
	Vec3 p011 { mn.x, mx.y, mx.z };
	Vec3 p100 { mx.x, mn.y, mn.z };
	Vec3 p101 { mx.x, mn.y, mx.z };
	Vec3 p110 { mx.x, mx.y, mn.z };
	Vec3 p111 { mx.x, mx.y, mx.z };

	Vec2 uv00 { 0, 0 }, uv10 { 1, 0 }, uv11 { 1, 1 }, uv01 { 0, 1 };

	addQuad(out, nextId, name + "_front", p001, p101, p111, p011, uv00, uv10, uv11, uv01, mat, color);
	addQuad(out, nextId, name + "_back", p100, p000, p010, p110, uv00, uv10, uv11, uv01, mat, color);
	addQuad(out, nextId, name + "_left", p000, p001, p011, p010, uv00, uv10, uv11, uv01, mat, color);
	addQuad(out, nextId, name + "_right", p101, p100, p110, p111, uv00, uv10, uv11, uv01, mat, color);
	addQuad(out, nextId, name + "_top", p010, p011, p111, p110, uv00, uv10, uv11, uv01, mat, color);
	addQuad(out, nextId, name + "_bottom", p000, p100, p101, p001, uv00, uv10, uv11, uv01, mat, color);
}

static std::vector<Triangle> buildCornellScene() {
	std::vector<Triangle> scene;
	int nextId = 0;

	Vec3 mn { -1, -1, 0 };
	Vec3 mx { 1, 1, 2 };

	Vec2 uv00 { 0, 0 }, uv10 { 1, 0 }, uv11 { 1, 1 }, uv01 { 0, 1 };

	addQuad(scene, nextId, "wall_back",
		Vec3 { mn.x, mn.y, mn.z }, Vec3 { mx.x, mn.y, mn.z }, Vec3 { mx.x, mx.y, mn.z }, Vec3 { mn.x, mx.y, mn.z },
		uv00, uv10, uv11, uv01, MaterialType::Diffuse, Vec3 { 0.95, 0.95, 0.95 });

	addQuad(scene, nextId, "wall_left",
		Vec3 { mn.x, mn.y, mx.z }, Vec3 { mn.x, mn.y, mn.z }, Vec3 { mn.x, mx.y, mn.z }, Vec3 { mn.x, mx.y, mx.z },
		uv00, uv10, uv11, uv01, MaterialType::Diffuse, Vec3 { 0.90, 0.25, 0.25 });

	addQuad(scene, nextId, "wall_right",
		Vec3 { mx.x, mn.y, mn.z }, Vec3 { mx.x, mn.y, mx.z }, Vec3 { mx.x, mx.y, mx.z }, Vec3 { mx.x, mx.y, mn.z },
		uv00, uv10, uv11, uv01, MaterialType::Diffuse, Vec3 { 0.25, 0.90, 0.25 });

	addQuad(scene, nextId, "floor",
		Vec3 { mn.x, mn.y, mx.z }, Vec3 { mx.x, mn.y, mx.z }, Vec3 { mx.x, mn.y, mn.z }, Vec3 { mn.x, mn.y, mn.z },
		uv00, uv10, uv11, uv01, MaterialType::Diffuse, Vec3 { 0.95, 0.95, 0.95 });

	addQuad(scene, nextId, "ceiling",
		Vec3 { mn.x, mx.y, mn.z }, Vec3 { mx.x, mx.y, mn.z }, Vec3 { mx.x, mx.y, mx.z }, Vec3 { mn.x, mx.y, mx.z },
		uv00, uv10, uv11, uv01, MaterialType::Diffuse, Vec3 { 0.95, 0.95, 0.95 });

	// 蓝金属盒
	{
		Vec3 bmn { -0.75, -1.0, 0.75 };
		Vec3 bmx { -0.15, -0.40, 1.35 };
		addBox(scene, nextId, "box_blue_metal", bmn, bmx, MaterialType::Metal, Vec3 { 0.20, 0.40, 0.95 });
	}

	// 黄金属盒
	{
		Vec3 ymn { 0.15, -1.0, 1.10 };
		Vec3 ymx { 0.70, -0.20, 1.70 };
		addBox(scene, nextId, "box_yellow_metal", ymn, ymx, MaterialType::Metal, Vec3 { 0.95, 0.85, 0.20 });
	}

	// 小漫反射柱（辅助反射观感）
	{
		Vec3 dmn { -0.10, -1.0, 1.55 };
		Vec3 dmx { 0.10, -0.70, 1.75 };
		addBox(scene, nextId, "diffuse_pillar", dmn, dmx, MaterialType::Diffuse, Vec3 { 0.85, 0.85, 0.90 });
	}

	return scene;
}

int main() {
	const int W = 800;
	const int H = 800;

	Image img(W, H);

	Camera cam;
	cam.origin = Vec3 { 0.0, 0.0, 3.0 };

	// Curtain（两个三角形）
	Vec3 s0 { -1.1, -1.1, 2.5 };
	Vec3 s1 { 1.1, -1.1, 2.5 };
	Vec3 s2 { 1.1, 1.1, 2.5 };
	Vec3 s3 { -1.1, 1.1, 2.5 };

	cam.curtainA = makeTri(-100, "curtainA",
		s0, s1, s2,
		Vec2 { 0, 0 }, Vec2 { 1, 0 }, Vec2 { 1, 1 },
		MaterialType::Curtain, Vec3 { 0, 0, 0 });
	cam.curtainB = makeTri(-101, "curtainB",
		s0, s2, s3,
		Vec2 { 0, 0 }, Vec2 { 1, 1 }, Vec2 { 0, 1 },
		MaterialType::Curtain, Vec3 { 0, 0, 0 });

	auto scene = buildCornellScene();

	// 只是证明 tri-tri 相交存在，不影响渲染
	if (scene.size() >= 2) {
		(void)triTriIntersect3D(scene[0].p[0], scene[0].p[1], scene[0].p[2],
			scene[1].p[0], scene[1].p[1], scene[1].p[2]);
	}

	cam.render(scene, img);
	img.writePPM("out.ppm");

	std::cout << "Wrote out.ppm (" << W << "x" << H << ")\n";
	return 0;
}
