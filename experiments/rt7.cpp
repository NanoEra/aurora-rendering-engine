// main.cpp
// 一个“以面片为处理单位”的步进式递归渲染器（非逐像素光线追踪），输出 PPM。
// 思路要点：
// - 相机 viewport 被拆成 2 个三角形 Curtain；渲染时遍历场景三角形，把三角形“投影变形”到当前 viewport 三角形平面上
// - 采用“由远到近”排序的 painter 覆盖关系（O(n log n)）
// - Diffuse：只贴纹理/颜色（不递归）
// - Metal：先贴自身，再把相机原点关于该金属三角形平面对称，令该金属三角形成为下一层 viewport，递归渲染反射并合成到金属区域
//
// 编译：g++ -O2 -std=c++17 main.cpp -o render
// 运行：./render
// 输出：out.ppm

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
#include <tuple>
#include <utility>
#include <vector>

static constexpr double kEps = 1e-9;

static inline double clamp01(double x) {
	return x < 0 ? 0 : (x > 1 ? 1 : x);
}

struct Vec2 {
	double x = 0, y = 0;
	Vec2() = default;
	Vec2(double xx, double yy) : x(xx), y(yy) {
	}
	Vec2 operator+(const Vec2 &r) const {
		return { x + r.x, y + r.y };
	}
	Vec2 operator-(const Vec2 &r) const {
		return { x - r.x, y - r.y };
	}
	Vec2 operator*(double s) const {
		return { x * s, y * s };
	}
};

static inline double cross2(const Vec2 &a, const Vec2 &b) {
	return a.x * b.y - a.y * b.x;
}
static inline double dot2(const Vec2 &a, const Vec2 &b) {
	return a.x * b.x + a.y * b.y;
}

struct Vec3 {
	double x = 0, y = 0, z = 0;
	Vec3() = default;
	Vec3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {
	}

	Vec3 operator+(const Vec3 &r) const {
		return { x + r.x, y + r.y, z + r.z };
	}
	Vec3 operator-(const Vec3 &r) const {
		return { x - r.x, y - r.y, z - r.z };
	}
	Vec3 operator*(double s) const {
		return { x * s, y * s, z * s };
	}
	Vec3 operator/(double s) const {
		return { x / s, y / s, z / s };
	}

	Vec3 &operator+=(const Vec3 &r) {
		x += r.x;
		y += r.y;
		z += r.z;
		return *this;
	}
};

static inline double dot3(const Vec3 &a, const Vec3 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vec3 cross3(const Vec3 &a, const Vec3 &b) {
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

static inline double length3(const Vec3 &v) {
	return std::sqrt(dot3(v, v));
}

static inline Vec3 normalize3(const Vec3 &v) {
	double len = length3(v);
	if (len < kEps)
		return { 0, 0, 0 };
	return v / len;
}

static inline Vec3 hadamard(const Vec3 &a, const Vec3 &b) {
	return { a.x * b.x, a.y * b.y, a.z * b.z };
}

struct ScreenTri {
	Vec2 p0, p1, p2;
};

static inline double triArea2D(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	return 0.5 * std::abs(cross2(b - a, c - a));
}

static inline double triArea2D(const ScreenTri &t) {
	return triArea2D(t.p0, t.p1, t.p2);
}

static inline std::array<double, 3> barycentric2D(const Vec2 &p, const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	// 返回 (w0, w1, w2)，满足 p = w0*a + w1*b + w2*c，且 w0+w1+w2=1（在非退化情况下）
	Vec2 v0 = b - a;
	Vec2 v1 = c - a;
	Vec2 v2 = p - a;
	double den = cross2(v0, v1);
	if (std::abs(den) < kEps)
		return { -1, -1, -1 };
	double w1 = cross2(v2, v1) / den;
	double w2 = cross2(v0, v2) / den;
	double w0 = 1.0 - w1 - w2;
	return { w0, w1, w2 };
}

static inline bool pointInTri2D(const Vec2 &p, const Vec2 &a, const Vec2 &b, const Vec2 &c, double eps = 1e-8) {
	auto w = barycentric2D(p, a, b, c);
	return w[0] >= -eps && w[1] >= -eps && w[2] >= -eps;
}

static inline int orient(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	double v = cross2(b - a, c - a);
	if (v > 1e-12)
		return 1;
	if (v < -1e-12)
		return -1;
	return 0;
}

static inline bool onSegment(const Vec2 &a, const Vec2 &b, const Vec2 &p) {
	return std::min(a.x, b.x) - 1e-12 <= p.x && p.x <= std::max(a.x, b.x) + 1e-12 && std::min(a.y, b.y) - 1e-12 <= p.y && p.y <= std::max(a.y, b.y) + 1e-12 && std::abs(cross2(b - a, p - a)) < 1e-10;
}

static inline bool segIntersects(const Vec2 &a, const Vec2 &b, const Vec2 &c, const Vec2 &d) {
	int o1 = orient(a, b, c);
	int o2 = orient(a, b, d);
	int o3 = orient(c, d, a);
	int o4 = orient(c, d, b);

	if (o1 != o2 && o3 != o4)
		return true;
	if (o1 == 0 && onSegment(a, b, c))
		return true;
	if (o2 == 0 && onSegment(a, b, d))
		return true;
	if (o3 == 0 && onSegment(c, d, a))
		return true;
	if (o4 == 0 && onSegment(c, d, b))
		return true;
	return false;
}

static inline bool triOverlap2D(const ScreenTri &A, const ScreenTri &B) {
	// 顶点包含测试
	if (pointInTri2D(A.p0, B.p0, B.p1, B.p2) || pointInTri2D(A.p1, B.p0, B.p1, B.p2) || pointInTri2D(A.p2, B.p0, B.p1, B.p2))
		return true;
	if (pointInTri2D(B.p0, A.p0, A.p1, A.p2) || pointInTri2D(B.p1, A.p0, A.p1, A.p2) || pointInTri2D(B.p2, A.p0, A.p1, A.p2))
		return true;

	// 边相交测试
	std::array<Vec2, 3> a = { A.p0, A.p1, A.p2 };
	std::array<Vec2, 3> b = { B.p0, B.p1, B.p2 };
	for (int i = 0; i < 3; i++) {
		Vec2 a0 = a[i], a1 = a[(i + 1) % 3];
		for (int j = 0; j < 3; j++) {
			Vec2 b0 = b[j], b1 = b[(j + 1) % 3];
			if (segIntersects(a0, a1, b0, b1))
				return true;
		}
	}
	return false;
}

enum class MaterialType {
	Curtain,
	Diffuse,
	Metal
};

struct Material {
	MaterialType type = MaterialType::Diffuse;
	Vec3 baseColor { 1, 1, 1 };
	double reflectivity = 0.85; // 仅 Metal 使用
	bool checker = false;
	double checkerScale = 8.0;
};

struct Triangle {
	int id = -1;
	Vec3 v0, v1, v2;
	Vec2 uv0 { 0, 0 }, uv1 { 1, 0 }, uv2 { 0, 1 };
	Material mat;

	Vec3 normal() const {
		return normalize3(cross3(v1 - v0, v2 - v0));
	}
	Vec3 centroid() const {
		return (v0 + v1 + v2) / 3.0;
	}
};

struct Image {
	int w = 0, h = 0;
	int ox = 0, oy = 0; // 全局坐标偏移（用于子 buffer）
	bool masked = false; // true：未写入像素默认“无效”
	std::vector<Vec3> pix;
	std::vector<uint8_t> mask; // masked=true 时使用

	Image() = default;
	Image(int ww, int hh, int offx = 0, int offy = 0, bool useMask = false)
		: w(ww), h(hh), ox(offx), oy(offy), masked(useMask), pix((size_t)ww * (size_t)hh, { 0, 0, 0 }) {
		if (masked)
			mask.assign((size_t)ww * (size_t)hh, 0);
	}

	bool inBoundsGlobal(int x, int y) const {
		return x >= ox && x < ox + w && y >= oy && y < oy + h;
	}

	size_t idxGlobal(int x, int y) const {
		return (size_t)(y - oy) * (size_t)w + (size_t)(x - ox);
	}

	Vec3 get(int x, int y) const {
		if (!inBoundsGlobal(x, y))
			return { 0, 0, 0 };
		size_t i = idxGlobal(x, y);
		if (masked && mask[i] == 0)
			return { 0, 0, 0 };
		return pix[i];
	}

	bool has(int x, int y) const {
		if (!inBoundsGlobal(x, y))
			return false;
		if (!masked)
			return true;
		return mask[idxGlobal(x, y)] != 0;
	}

	void set(int x, int y, const Vec3 &c) {
		if (!inBoundsGlobal(x, y))
			return;
		size_t i = idxGlobal(x, y);
		pix[i] = c;
		if (masked)
			mask[i] = 1;
	}

	void writePPM(const std::string &path) const {
		// 仅支持 ox=oy=0 且 masked=false 的整图输出
		std::ofstream out(path, std::ios::binary);
		out << "P6\n"
			<< w << " " << h << "\n255\n";
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				Vec3 c = pix[(size_t)y * (size_t)w + (size_t)x];
				// gamma 2.2
				auto toByte = [](double v) -> uint8_t {
					v = clamp01(v);
					v = std::pow(v, 1.0 / 2.2);
					int b = (int)std::lround(v * 255.0);
					if (b < 0)
						b = 0;
					if (b > 255)
						b = 255;
					return (uint8_t)b;
				};
				uint8_t r = toByte(c.x);
				uint8_t g = toByte(c.y);
				uint8_t b = toByte(c.z);
				out.write((const char *)&r, 1);
				out.write((const char *)&g, 1);
				out.write((const char *)&b, 1);
			}
		}
	}
};

struct ViewportFrame {
	Vec3 p0; // viewport v0
	Vec3 n; // plane normal (unit)
	Vec3 axisX; // in-plane basis
	Vec3 axisY; // in-plane basis
	Vec2 v0_2 { 0, 0 };
	Vec2 v1_2 { 0, 0 };
	Vec2 v2_2 { 0, 0 };
};

static inline ViewportFrame makeViewportFrame(const Triangle &viewport) {
	ViewportFrame f;
	f.p0 = viewport.v0;
	Vec3 e01 = viewport.v1 - viewport.v0;
	Vec3 e02 = viewport.v2 - viewport.v0;
	f.n = normalize3(cross3(e01, e02));
	f.axisX = normalize3(e01);
	f.axisY = normalize3(cross3(f.n, f.axisX));

	f.v0_2 = { 0, 0 };
	f.v1_2 = { dot3(e01, f.axisX), dot3(e01, f.axisY) };
	f.v2_2 = { dot3(e02, f.axisX), dot3(e02, f.axisY) };
	return f;
}

static inline Vec2 toViewport2D(const ViewportFrame &f, const Vec3 &p) {
	Vec3 d = p - f.p0;
	return { dot3(d, f.axisX), dot3(d, f.axisY) };
}

static inline Vec3 reflectPointAcrossPlane(const Vec3 &p, const Vec3 &planePoint, const Vec3 &planeNormalUnit) {
	// 反射点：p' = p - 2*dot(n, (p-planePoint))*n
	double d = dot3(planeNormalUnit, p - planePoint);
	return p - planeNormalUnit * (2.0 * d);
}

struct ProjectedTri {
	ScreenTri screen; // 投影到最终输出（全局像素坐标）的三角形
	std::array<double, 3> invW; // “投影深度倒数”等价量（用于透视矫正插值）
};

static inline std::optional<ProjectedTri> projectTriangleOntoViewport(
	const Triangle &tri,
	const Vec3 &origin,
	const Triangle &viewportWorld,
	const ScreenTri &viewportScreen,
	const ViewportFrame &vf) {
	// 把 tri 的三个顶点沿着 origin->vertex 的直线投影到 viewportWorld 的平面上，
	// 再通过 viewportWorld 的重心坐标映射到 viewportScreen（像素三角形）上。
	Vec3 n = vf.n;
	Vec3 planeP = vf.p0;

	auto projectPoint = [&](const Vec3 &P, double &outT, Vec3 &outProj) -> bool {
		Vec3 dir = P - origin;
		double denom = dot3(n, dir);
		if (std::abs(denom) < 1e-12)
			return false;
		double t = dot3(n, planeP - origin) / denom;

		// 关键：t>1 表示平面在 P 的“更远处”，即 P 位于 origin 与平面之间（不应被该 viewport 看见）
		// t<=0 表示交点在 origin 反方向
		if (!(t > 1e-9 && t <= 1.0 + 1e-7))
			return false;

		outT = t;
		outProj = origin + dir * t;
		return true;
	};

	std::array<Vec3, 3> P = { tri.v0, tri.v1, tri.v2 };
	std::array<Vec3, 3> Q;
	std::array<double, 3> t;

	for (int i = 0; i < 3; i++) {
		if (!projectPoint(P[i], t[i], Q[i]))
			return std::nullopt;
	}

	// 投影点 Q 都在 viewport 平面上：计算它们相对 viewportWorld 的重心坐标
	auto screenFromQ = [&](const Vec3 &q) -> std::optional<Vec2> {
		Vec2 q2 = toViewport2D(vf, q);
		auto w = barycentric2D(q2, vf.v0_2, vf.v1_2, vf.v2_2);
		if (w[0] < -1e-6 || w[1] < -1e-6 || w[2] < -1e-6) {
			// 允许在 viewport 外（仍可能与 viewport 相交），所以这里不做强制 inside
			// 只要不是退化即可
		}
		if (!std::isfinite(w[0]) || !std::isfinite(w[1]) || !std::isfinite(w[2]))
			return std::nullopt;

		Vec2 s = viewportScreen.p0 * w[0] + viewportScreen.p1 * w[1] + viewportScreen.p2 * w[2];
		return s;
	};

	std::array<Vec2, 3> S;
	for (int i = 0; i < 3; i++) {
		auto s = screenFromQ(Q[i]);
		if (!s)
			return std::nullopt;
		S[i] = *s;
	}

	ProjectedTri out;
	out.screen = { S[0], S[1], S[2] };

	// 透视矫正：在“固定 origin + 固定 viewport 平面”下，t 等价于 1/depth（类比 plane z=1 时 t=1/z）
	out.invW = { t[0], t[1], t[2] };
	return out;
}

static inline Vec3 sampleMaterial(const Triangle &tri, const Vec2 &uv) {
	// 简易纹理：checker 或纯色
	Vec3 c = tri.mat.baseColor;
	if (tri.mat.checker) {
		double u = uv.x * tri.mat.checkerScale;
		double v = uv.y * tri.mat.checkerScale;
		int iu = (int)std::floor(u);
		int iv = (int)std::floor(v);
		bool odd = ((iu + iv) & 1) != 0;
		double k = odd ? 0.55 : 1.0;
		c = c * k;
	}
	return c;
}

static inline Vec3 shadeSimple(const Triangle &tri, const Vec3 &base) {
	// 简易定向光 + 环境光，避免因法线翻转导致全黑：使用 abs(dot)
	Vec3 n = tri.normal();
	Vec3 lightDir = normalize3(Vec3 { -0.3, 0.9, -0.2 });
	double ndl = std::abs(dot3(n, lightDir));
	double ambient = 0.18;
	double diff = ambient + (1.0 - ambient) * ndl;
	return base * diff;
}

struct RasterMask {
	int w = 0, h = 0;
	int ox = 0, oy = 0;
	std::vector<uint8_t> m;
	RasterMask() = default;
	RasterMask(int ww, int hh, int offx, int offy) : w(ww), h(hh), ox(offx), oy(offy), m((size_t)ww * (size_t)hh, 0) {
	}

	bool inBoundsGlobal(int x, int y) const {
		return x >= ox && x < ox + w && y >= oy && y < oy + h;
	}
	size_t idxGlobal(int x, int y) const {
		return (size_t)(y - oy) * (size_t)w + (size_t)(x - ox);
	}

	void set(int x, int y) {
		if (!inBoundsGlobal(x, y))
			return;
		m[idxGlobal(x, y)] = 1;
	}
	bool get(int x, int y) const {
		if (!inBoundsGlobal(x, y))
			return false;
		return m[idxGlobal(x, y)] != 0;
	}
};

static inline void rasterizeProjectedTriangle(
	Image &target,
	const Triangle &tri,
	const ProjectedTri &proj,
	const ScreenTri &viewportScreen,
	RasterMask *outMaskOrNull) {
	// 在目标 target 上对 proj.screen 做三角形光栅化，并且裁剪在 viewportScreen 内（防止越界 spill）
	ScreenTri st = proj.screen;

	double minx = std::floor(std::min({ st.p0.x, st.p1.x, st.p2.x }));
	double maxx = std::ceil(std::max({ st.p0.x, st.p1.x, st.p2.x }));
	double miny = std::floor(std::min({ st.p0.y, st.p1.y, st.p2.y }));
	double maxy = std::ceil(std::max({ st.p0.y, st.p1.y, st.p2.y }));

	int ix0 = (int)minx;
	int ix1 = (int)maxx;
	int iy0 = (int)miny;
	int iy1 = (int)maxy;

	for (int y = iy0; y <= iy1; y++) {
		for (int x = ix0; x <= ix1; x++) {
			Vec2 p { (double)x + 0.5, (double)y + 0.5 };
			if (!pointInTri2D(p, st.p0, st.p1, st.p2))
				continue;
			if (!pointInTri2D(p, viewportScreen.p0, viewportScreen.p1, viewportScreen.p2))
				continue;
			if (!target.inBoundsGlobal(x, y))
				continue;

			auto w = barycentric2D(p, st.p0, st.p1, st.p2);
			if (w[0] < -1e-8 || w[1] < -1e-8 || w[2] < -1e-8)
				continue;

			// 透视矫正插值 UV
			double iw0 = proj.invW[0], iw1 = proj.invW[1], iw2 = proj.invW[2];
			double denom = w[0] * iw0 + w[1] * iw1 + w[2] * iw2;
			if (std::abs(denom) < 1e-12)
				continue;

			Vec2 uv = (tri.uv0 * (w[0] * iw0) + tri.uv1 * (w[1] * iw1) + tri.uv2 * (w[2] * iw2)) * (1.0 / denom);

			Vec3 base = sampleMaterial(tri, uv);
			Vec3 col = shadeSimple(tri, base);

			target.set(x, y, col);
			if (outMaskOrNull)
				outMaskOrNull->set(x, y);
		}
	}
}

// ---------（可选）三角形相交：这里给一个基础实现（不一定用于主流程）---------

struct RayHit {
	double t = 0;
	Vec3 p;
	Vec3 n;
};

static inline std::optional<RayHit> intersectRayTriangle(const Vec3 &ro, const Vec3 &rd, const Triangle &tri) {
	// Möller–Trumbore（用于几何工具，主算法不按逐像素 ray casting）
	Vec3 e1 = tri.v1 - tri.v0;
	Vec3 e2 = tri.v2 - tri.v0;
	Vec3 pvec = cross3(rd, e2);
	double det = dot3(e1, pvec);
	if (std::abs(det) < 1e-12)
		return std::nullopt;
	double invDet = 1.0 / det;

	Vec3 tvec = ro - tri.v0;
	double u = dot3(tvec, pvec) * invDet;
	if (u < 0 || u > 1)
		return std::nullopt;

	Vec3 qvec = cross3(tvec, e1);
	double v = dot3(rd, qvec) * invDet;
	if (v < 0 || u + v > 1)
		return std::nullopt;

	double t = dot3(e2, qvec) * invDet;
	if (t < 0)
		return std::nullopt;

	RayHit hit;
	hit.t = t;
	hit.p = ro + rd * t;
	hit.n = tri.normal();
	return hit;
}

static inline bool triangleTriangleIntersectRough(const Triangle &a, const Triangle &b) {
	// 粗略：边与对方三角形的射线相交 + 顶点包含（足够作为“相关运算”示意）
	auto testEdge = [&](const Vec3 &p0, const Vec3 &p1, const Triangle &tri) -> bool {
		Vec3 d = p1 - p0;
		double len = length3(d);
		if (len < 1e-12)
			return false;
		Vec3 rd = d / len;
		auto hit = intersectRayTriangle(p0, rd, tri);
		if (!hit)
			return false;
		return hit->t >= 0 && hit->t <= len;
	};

	std::array<Vec3, 3> av = { a.v0, a.v1, a.v2 };
	std::array<Vec3, 3> bv = { b.v0, b.v1, b.v2 };

	for (int i = 0; i < 3; i++) {
		if (testEdge(av[i], av[(i + 1) % 3], b))
			return true;
		if (testEdge(bv[i], bv[(i + 1) % 3], a))
			return true;
	}
	return false;
}

// --------- 核心递归渲染：renderTriangleWithTriangle ---------

struct RenderSettings {
	int maxDepth = 4;
	double minViewportAreaPx = 4.0; // 1~5 像素级阈值，这里取 4
};

struct Candidate {
	const Triangle *tri = nullptr;
	ProjectedTri proj;
	double dist = 0;
};

static void renderTriangleWithTriangle(
	const std::vector<Triangle> &scene,
	const Vec3 &cameraOrigin,
	const Triangle &curtainTriWorld, // 保留形参以贴合你的描述；本实现不强依赖它
	const Triangle &currentViewportWorld, // 当前 viewport 面片（世界空间三角形）
	const ScreenTri &viewportInImage, // 当前 viewport 面片在最终输出图上的三角形区域（像素三点）
	Image &outTarget,
	const RenderSettings &settings,
	int depth) {
	(void)curtainTriWorld;

	if (depth > settings.maxDepth)
		return;
	if (triArea2D(viewportInImage) < settings.minViewportAreaPx)
		return;

	ViewportFrame vf = makeViewportFrame(currentViewportWorld);

	std::vector<Candidate> candidates;
	candidates.reserve(scene.size());

	for (const Triangle &tri : scene) {
		if (tri.id == currentViewportWorld.id && tri.id != -1)
			continue; // viewport 自身不参与渲染，避免递归自反馈

		auto projOpt = projectTriangleOntoViewport(tri, cameraOrigin, currentViewportWorld, viewportInImage, vf);
		if (!projOpt)
			continue;

		// 投影三角形必须与 viewport 区域有重叠，否则不处理
		if (!triOverlap2D(projOpt->screen, viewportInImage))
			continue;

		Candidate c;
		c.tri = &tri;
		c.proj = *projOpt;
		c.dist = length3(tri.centroid() - cameraOrigin);
		candidates.push_back(c);
	}

	// 覆盖关系：由远到近 painter（O(n log n)）
	std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
		return a.dist > b.dist;
	});

	for (const Candidate &c : candidates) {
		const Triangle &tri = *c.tri;

		if (tri.mat.type == MaterialType::Diffuse) {
			rasterizeProjectedTriangle(outTarget, tri, c.proj, viewportInImage, nullptr);
			continue;
		}

		if (tri.mat.type == MaterialType::Metal) {
			// 1) 先贴金属自身底色（并记录金属可见区域 mask）
			ScreenTri metalScreen = c.proj.screen;

			int bbMinX = (int)std::floor(std::min({ metalScreen.p0.x, metalScreen.p1.x, metalScreen.p2.x }));
			int bbMaxX = (int)std::ceil(std::max({ metalScreen.p0.x, metalScreen.p1.x, metalScreen.p2.x }));
			int bbMinY = (int)std::floor(std::min({ metalScreen.p0.y, metalScreen.p1.y, metalScreen.p2.y }));
			int bbMaxY = (int)std::ceil(std::max({ metalScreen.p0.y, metalScreen.p1.y, metalScreen.p2.y }));

			int bbW = std::max(1, bbMaxX - bbMinX + 1);
			int bbH = std::max(1, bbMaxY - bbMinY + 1);

			RasterMask metalMask(bbW, bbH, bbMinX, bbMinY);

			rasterizeProjectedTriangle(outTarget, tri, c.proj, viewportInImage, &metalMask);

			// 2) 计算“关于当前金属面片”的相机原点对称点，作为下一层递归的 origin
			Vec3 n = tri.normal();
			Vec3 originRef = reflectPointAcrossPlane(cameraOrigin, tri.v0, n);

			// 3) 递归渲染反射：让当前金属面片 tri 自身成为下一层 viewport
			//    注意：反射结果渲染到一个子 buffer（避免直接 blend 导致 painter 合成错误）
			Image reflBuf(bbW, bbH, bbMinX, bbMinY, true /*masked*/);

			// 递归终止也会受 metalScreen 面积限制（通过 viewportInImage 面积判断）
			renderTriangleWithTriangle(
				scene,
				originRef,
				curtainTriWorld,
				tri, // 下一层 viewport = 当前金属三角形
				metalScreen, // 下一层 viewport 在最终图片中的区域 = 当前金属区域（投影三角形）
				reflBuf,
				settings,
				depth + 1);

			// 4) 合成：只在 metalMask 覆盖到的像素上，把 reflBuf 以 reflectivity 混合到 outTarget
			double a = clamp01(tri.mat.reflectivity);
			for (int y = bbMinY; y <= bbMaxY; y++) {
				for (int x = bbMinX; x <= bbMaxX; x++) {
					if (!metalMask.get(x, y))
						continue; // 只在“金属可见区域”合成
					if (!reflBuf.has(x, y))
						continue; // 反射缓冲没写入则不改变底色

					Vec3 base = outTarget.get(x, y);
					Vec3 refl = reflBuf.get(x, y);
					Vec3 out = base * (1.0 - a) + refl * a;
					outTarget.set(x, y, out);
				}
			}

			continue;
		}

		// Curtain 类型一般不在 scene 里；若出现，忽略
	}
}

// --------- Scene：Cornell Box + 两个金属盒子（蓝/黄）---------

static int gNextTriId = 1;

static inline Triangle makeTri(const Vec3 &a, const Vec3 &b, const Vec3 &c,
	const Vec2 &uva, const Vec2 &uvb, const Vec2 &uvc,
	const Material &m) {
	Triangle t;
	t.id = gNextTriId++;
	t.v0 = a;
	t.v1 = b;
	t.v2 = c;
	t.uv0 = uva;
	t.uv1 = uvb;
	t.uv2 = uvc;
	t.mat = m;
	return t;
}

static inline void addQuad(std::vector<Triangle> &out,
	const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d,
	const Material &m) {
	// (a,b,c) + (a,c,d)
	out.push_back(makeTri(a, b, c, { 0, 0 }, { 1, 0 }, { 1, 1 }, m));
	out.push_back(makeTri(a, c, d, { 0, 0 }, { 1, 1 }, { 0, 1 }, m));
}

static inline Vec3 rotateY(const Vec3 &p, double rad) {
	double cs = std::cos(rad), sn = std::sin(rad);
	return { p.x * cs + p.z * sn, p.y, -p.x * sn + p.z * cs };
}

static inline void addBox(std::vector<Triangle> &out,
	const Vec3 &center,
	const Vec3 &size, // (sx, sy, sz)
	double rotYRad,
	const Material &m) {
	Vec3 h = size * 0.5;

	// 8 corners in local space
	std::array<Vec3, 8> v = {
		Vec3 { -h.x, -h.y, -h.z }, // 0
		Vec3 { h.x, -h.y, -h.z }, // 1
		Vec3 { h.x, h.y, -h.z }, // 2
		Vec3 { -h.x, h.y, -h.z }, // 3
		Vec3 { -h.x, -h.y, h.z }, // 4
		Vec3 { h.x, -h.y, h.z }, // 5
		Vec3 { h.x, h.y, h.z }, // 6
		Vec3 { -h.x, h.y, h.z } // 7
	};

	for (auto &p : v)
		p = rotateY(p, rotYRad) + center;

	// faces: each as quad (a,b,c,d) with consistent uv
	// Front (-z): 0,1,2,3
	addQuad(out, v[0], v[1], v[2], v[3], m);
	// Back (+z): 5,4,7,6 (swap order to vary orientation; shading使用abs不敏感)
	addQuad(out, v[5], v[4], v[7], v[6], m);
	// Left (-x): 4,0,3,7
	addQuad(out, v[4], v[0], v[3], v[7], m);
	// Right (+x): 1,5,6,2
	addQuad(out, v[1], v[5], v[6], v[2], m);
	// Bottom (-y): 4,5,1,0
	addQuad(out, v[4], v[5], v[1], v[0], m);
	// Top (+y): 3,2,6,7
	addQuad(out, v[3], v[2], v[6], v[7], m);
}

static inline std::vector<Triangle> buildCornellScene() {
	std::vector<Triangle> tris;
	tris.reserve(128);

	// Cornell box 尺寸（前方开口，便于相机观察）
	// x in [-1,1], y in [0,2], z in [0,2]
	Vec3 L { -1, 0, 0 }, R { 1, 0, 0 };
	(void)L;
	(void)R;

	Material floorM;
	floorM.type = MaterialType::Diffuse;
	floorM.baseColor = { 0.9, 0.9, 0.9 };
	floorM.checker = true;
	floorM.checkerScale = 6;
	Material leftM;
	leftM.type = MaterialType::Diffuse;
	leftM.baseColor = { 0.9, 0.2, 0.2 };
	Material rightM;
	rightM.type = MaterialType::Diffuse;
	rightM.baseColor = { 0.2, 0.9, 0.2 };
	Material backM;
	backM.type = MaterialType::Diffuse;
	backM.baseColor = { 0.95, 0.95, 0.95 };

	// Floor y=0
	addQuad(tris,
		Vec3 { -1, 0, 0 }, Vec3 { 1, 0, 0 }, Vec3 { 1, 0, 2 }, Vec3 { -1, 0, 2 },
		floorM);

	// Left wall x=-1
	addQuad(tris,
		Vec3 { -1, 0, 0 }, Vec3 { -1, 0, 2 }, Vec3 { -1, 2, 2 }, Vec3 { -1, 2, 0 },
		leftM);

	// Right wall x=1
	addQuad(tris,
		Vec3 { 1, 0, 0 }, Vec3 { 1, 2, 0 }, Vec3 { 1, 2, 2 }, Vec3 { 1, 0, 2 },
		rightM);

	// Back wall z=2
	addQuad(tris,
		Vec3 { -1, 0, 2 }, Vec3 { 1, 0, 2 }, Vec3 { 1, 2, 2 }, Vec3 { -1, 2, 2 },
		backM);

	// 两个金属盒子（蓝/黄），放在盒子内部
	Material blueMetal;
	blueMetal.type = MaterialType::Metal;
	blueMetal.baseColor = { 0.15, 0.25, 0.95 };
	blueMetal.reflectivity = 0.85;
	blueMetal.checker = false;
	Material yellowMetal;
	yellowMetal.type = MaterialType::Metal;
	yellowMetal.baseColor = { 0.95, 0.85, 0.15 };
	yellowMetal.reflectivity = 0.80;
	yellowMetal.checker = false;

	addBox(tris, Vec3 { -0.45, 0.50, 1.05 }, Vec3 { 0.55, 1.00, 0.55 }, 0.30, blueMetal);
	addBox(tris, Vec3 { 0.45, 0.35, 1.45 }, Vec3 { 0.65, 0.70, 0.65 }, -0.45, /*yellowMetal*/rightM);

	return tris;
}

// --------- Camera：viewport 两三角形 Curtain，调用两次渲染并输出 PPM ---------

struct Camera {
	Vec3 origin { 0, 1, -3 };
	Vec3 lookAt { 0, 1, 1 };
	Vec3 up { 0, 1, 0 };

	int width = 800;
	int height = 600;
	double fovYDeg = 50.0;
	double focalDist = 1.0;

	Triangle curtainA; // world
	Triangle curtainB; // world
	ScreenTri screenA; // image region
	ScreenTri screenB; // image region

	void buildCurtain() {
		Vec3 forward = normalize3(lookAt - origin);
		Vec3 right = normalize3(cross3(forward, up));
		Vec3 up2 = normalize3(cross3(right, forward));

		double aspect = (double)width / (double)height;
		double fovYRad = fovYDeg * M_PI / 180.0;
		double halfH = std::tan(fovYRad * 0.5) * focalDist;
		double halfW = halfH * aspect;

		Vec3 center = origin + forward * focalDist;
		Vec3 tl = center + up2 * halfH - right * halfW;
		Vec3 tr = center + up2 * halfH + right * halfW;
		Vec3 bl = center - up2 * halfH - right * halfW;
		Vec3 br = center - up2 * halfH + right * halfW;

		Material curtainM;
		curtainM.type = MaterialType::Curtain;
		curtainM.baseColor = { 0, 0, 0 };

		curtainA.id = -1001;
		curtainA.v0 = tl;
		curtainA.v1 = tr;
		curtainA.v2 = bl;
		curtainA.mat = curtainM;

		curtainB.id = -1002;
		curtainB.v0 = br;
		curtainB.v1 = bl;
		curtainB.v2 = tr;
		curtainB.mat = curtainM;

		// 像素三角形：覆盖整个图像矩形（对角线划分）
		// 注意：图像坐标原点在左上，y 向下
		screenA = { Vec2 { 0, 0 }, Vec2 { (double)(width - 1), 0 }, Vec2 { 0, (double)(height - 1) } };
		screenB = { Vec2 { (double)(width - 1), (double)(height - 1) }, Vec2 { 0, (double)(height - 1) }, Vec2 { (double)(width - 1), 0 } };
	}

	void render(const std::vector<Triangle> &scene, const std::string &outPath) {
		buildCurtain();

		Image img(width, height, 0, 0, false);
		// 背景（默认黑）
		for (auto &p : img.pix)
			p = Vec3 { 0.0, 0.0, 0.0 };

		RenderSettings settings;
		settings.maxDepth = 4;
		settings.minViewportAreaPx = 4.0;

		// CurtainA
		renderTriangleWithTriangle(
			scene,
			origin,
			curtainA,
			curtainA,
			screenA,
			img,
			settings,
			0);

		// CurtainB
		renderTriangleWithTriangle(
			scene,
			origin,
			curtainB,
			curtainB,
			screenB,
			img,
			settings,
			0);

		img.writePPM(outPath);
	}
};

int main() {
	std::vector<Triangle> scene = buildCornellScene();

	Camera cam;
	cam.width = 900;
	cam.height = 700;
	cam.origin = Vec3 { 0.0, 1.0, -3.2 };
	cam.lookAt = Vec3 { 0.0, 1.0, 1.0 };
	cam.fovYDeg = 52.0;
	cam.focalDist = 1.0;

	cam.render(scene, "out.ppm");

	std::cerr << "Wrote out.ppm\n";
	return 0;
}
