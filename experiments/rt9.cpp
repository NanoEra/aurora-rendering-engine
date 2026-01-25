#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

// ============================================================================
// 基础数学结构
// ============================================================================

struct Vec3 {
	double x, y, z;

	Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {
	}

	Vec3 operator+(const Vec3 &v) const {
		return Vec3(x + v.x, y + v.y, z + v.z);
	}
	Vec3 operator-(const Vec3 &v) const {
		return Vec3(x - v.x, y - v.y, z - v.z);
	}
	Vec3 operator*(double t) const {
		return Vec3(x * t, y * t, z * t);
	}
	Vec3 operator/(double t) const {
		return Vec3(x / t, y / t, z / t);
	}
	Vec3 operator-() const {
		return Vec3(-x, -y, -z);
	}

	Vec3 &operator+=(const Vec3 &v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	Vec3 &operator*=(double t) {
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}

	double dot(const Vec3 &v) const {
		return x * v.x + y * v.y + z * v.z;
	}
	Vec3 cross(const Vec3 &v) const {
		return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	double length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	double lengthSquared() const {
		return x * x + y * y + z * z;
	}
	Vec3 normalize() const {
		double l = length();
		return l > 1e-10 ? *this / l : Vec3(0, 0, 0);
	}
	// 关于平面反射点（平面由点planePoint和法向量planeNormal定义）
	// 在 Vec3 类中，确保 reflect 函数正确
	Vec3 reflect(const Vec3 &planePoint, const Vec3 &planeNormal) const {
		Vec3 n = planeNormal.normalize();
		double d = (*this - planePoint).dot(n);
		return *this - n * (2.0 * d);
	}
};

inline Vec3 operator*(double t, const Vec3 &v) {
	return v * t;
}

// ============================================================================
// 颜色类
// ============================================================================

struct Color {
	double r, g, b;

	Color(double r = 0, double g = 0, double b = 0) : r(r), g(g), b(b) {
	}

	Color operator+(const Color &c) const {
		return Color(r + c.r, g + c.g, b + c.b);
	}
	Color operator-(const Color &c) const {
		return Color(r - c.r, g - c.g, b - c.b);
	}
	Color operator*(double t) const {
		return Color(r * t, g * t, b * t);
	}
	Color operator*(const Color &c) const {
		return Color(r * c.r, g * c.g, b * c.b);
	}
	Color operator/(double t) const {
		return Color(r / t, g / t, b / t);
	}

	Color &operator+=(const Color &c) {
		r += c.r;
		g += c.g;
		b += c.b;
		return *this;
	}

	Color clamp() const {
		return Color(
			std::max(0.0, std::min(1.0, r)),
			std::max(0.0, std::min(1.0, g)),
			std::max(0.0, std::min(1.0, b)));
	}
	// Gamma校正
	Color gammaCorrect(double gamma = 2.2) const {
		double invGamma = 1.0 / gamma;
		return Color(std::pow(r, invGamma), std::pow(g, invGamma), std::pow(b, invGamma));
	}
};

// ============================================================================
// 纹理类
// ============================================================================

// ============================================================================
// 三角形光栅化辅助函数
// ============================================================================

// 2D点结构
struct Point2D {
	double x, y;
	Point2D(double x = 0, double y = 0) : x(x), y(y) {
	}
};

// 计算2D三角形的有符号面积的两倍
double signedArea2D(const Point2D &a, const Point2D &b, const Point2D &c) {
	return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

// 计算2D重心坐标
bool barycentric2D(const Point2D &p, const Point2D &a, const Point2D &b, const Point2D &c, double &u, double &v, double &w) {
	double area = signedArea2D(a, b, c);
	if (std::abs(area) < 1e-10)
		return false;

	u = signedArea2D(p, b, c) / area;
	v = signedArea2D(a, p, c) / area;
	w = signedArea2D(a, b, p) / area;

	return true;
}

class Texture {
public:
	int width, height;
	std::vector<Color> pixels;
	std::vector<double> depths; // 深度缓冲

	Texture(int w = 1, int h = 1, const Color &c = Color(0, 0, 0))
		: width(w), height(h), pixels(w * h, c), depths(w * h, 1e30) {
	}

	Color getPixel(int x, int y) const {
		if (x < 0 || x >= width || y < 0 || y >= height)
			return Color(0, 0, 0);
		return pixels[y * width + x];
	}

	void setPixel(int x, int y, const Color &c) {
		if (x >= 0 && x < width && y >= 0 && y < height)
			pixels[y * width + x] = c;
	}

	double getDepth(int x, int y) const {
		if (x < 0 || x >= width || y < 0 || y >= height)
			return 1e30;
		return depths[y * width + x];
	}

	void setDepth(int x, int y, double d) {
		if (x >= 0 && x < width && y >= 0 && y < height)
			depths[y * width + x] = d;
	}

	void setPixelWithDepth(int x, int y, const Color &c, double depth) {
		if (x >= 0 && x < width && y >= 0 && y < height) {
			int idx = y * width + x;
			if (depth < depths[idx]) {
				depths[idx] = depth;
				pixels[idx] = c;
			}
		}
	}

	// 双线性插值采样 (u, v 在 [0, 1] 范围内)
	Color sample(double u, double v) const {
		u = std::max(0.0, std::min(1.0, u));
		v = std::max(0.0, std::min(1.0, v));
		double fx = u * (width - 1);
		double fy = v * (height - 1);
		int x0 = (int)fx, y0 = (int)fy;
		int x1 = std::min(x0 + 1, width - 1);
		int y1 = std::min(y0 + 1, height - 1);
		double tx = fx - x0, ty = fy - y0;

		Color c00 = getPixel(x0, y0);
		Color c10 = getPixel(x1, y0);
		Color c01 = getPixel(x0, y1);
		Color c11 = getPixel(x1, y1);

		Color c0 = c00 * (1 - tx) + c10 * tx;
		Color c1 = c01 * (1 - tx) + c11 * tx;
		return c0 * (1 - ty) + c1 * ty;
	}

	void clear(const Color &c = Color(0, 0, 0)) {
		std::fill(pixels.begin(), pixels.end(), c);
		std::fill(depths.begin(), depths.end(), 1e30);
	}
	// 写入PPM文件
	void writePPM(const std::string &filename) const {
		std::ofstream ofs(filename);
		ofs << "P3\n"
			<< width << " " << height << "\n255\n";
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				Color c = getPixel(x, y).clamp().gammaCorrect();
				int ir = static_cast<int>(255.99 * c.r);
				int ig = static_cast<int>(255.99 * c.g);
				int ib = static_cast<int>(255.99 * c.b);
				ofs << ir << " " << ig << " " << ib << "\n";
			}
		}
		ofs.close();
	}
	// 新增：将源纹理拉伸到目标三角形区域并混合粘贴
	// srcTri: 源纹理的三角形UV坐标 (通常是 (0,0), (1,0), (0,1))
	// dstTri: 目标纹理上的三角形像素坐标
	// tint: 颜色染色（用于金属反射）
	// alpha: 混合透明度
	void blendTriangleTexture(
		const Texture &src,
		const Point2D &dst0, const Point2D &dst1, const Point2D &dst2,
		const Color &tint = Color(1, 1, 1),
		double alpha = 1.0) {
		// 计算包围盒
		double minX = std::min({ dst0.x, dst1.x, dst2.x });
		double maxX = std::max({ dst0.x, dst1.x, dst2.x });
		double minY = std::min({ dst0.y, dst1.y, dst2.y });
		double maxY = std::max({ dst0.y, dst1.y, dst2.y });

		int startX = std::max(0, (int)std::floor(minX));
		int endX = std::min(width - 1, (int)std::ceil(maxX));
		int startY = std::max(0, (int)std::floor(minY));
		int endY = std::min(height - 1, (int)std::ceil(maxY));

		// 检查三角形面积，避免退化三角形
		double area = signedArea2D(dst0, dst1, dst2);
		if (std::abs(area) < 1e-6)
			return;

		for (int py = startY; py <= endY; py++) {
			for (int px = startX; px <= endX; px++) {
				Point2D p(px + 0.5, py + 0.5);

				double u, v, w;
				if (!barycentric2D(p, dst0, dst1, dst2, u, v, w))
					continue;

				// 放宽边界检查
				const double eps = -0.01;
				if (u < eps || v < eps || w < eps)
					continue;
				if (u > 1.01 || v > 1.01 || w > 1.01)
					continue;

				// 限制重心坐标到有效范围
				u = std::max(0.0, std::min(1.0, u));
				v = std::max(0.0, std::min(1.0, v));
				w = std::max(0.0, std::min(1.0, w));

				// 源纹理采样 - 使用整个纹理范围
				// 重心坐标直接映射到纹理UV
				double srcU = v; // 对应dst1方向
				double srcV = w; // 对应dst2方向

				// 确保在[0,1]范围内
				srcU = std::max(0.0, std::min(1.0, srcU));
				srcV = std::max(0.0, std::min(1.0, srcV));

				Color srcColor = src.sample(srcU, srcV);
				Color finalColor = srcColor * tint;

				if (alpha >= 1.0) {
					setPixel(px, py, finalColor);
				} else {
					Color dstColor = getPixel(px, py);
					setPixel(px, py, finalColor * alpha + dstColor * (1.0 - alpha));
				}
			}
		}
	}

	// 新增：使用UV坐标版本（UV范围0-1）
	void blendTriangleTextureUV(
		const Texture &src,
		const Point2D &uv0, const Point2D &uv1, const Point2D &uv2,
		const Color &tint = Color(1, 1, 1),
		double alpha = 1.0) {
		// 将UV坐标转换为像素坐标（允许负值和超出范围的值）
		Point2D dst0(uv0.x * width, uv0.y * height);
		Point2D dst1(uv1.x * width, uv1.y * height);
		Point2D dst2(uv2.x * width, uv2.y * height);

		// 计算包围盒并裁剪到纹理范围
		double minX = std::min({ dst0.x, dst1.x, dst2.x });
		double maxX = std::max({ dst0.x, dst1.x, dst2.x });
		double minY = std::min({ dst0.y, dst1.y, dst2.y });
		double maxY = std::max({ dst0.y, dst1.y, dst2.y });

		// 裁剪到有效范围
		int startX = std::max(0, (int)std::floor(minX));
		int endX = std::min(width - 1, (int)std::ceil(maxX));
		int startY = std::max(0, (int)std::floor(minY));
		int endY = std::min(height - 1, (int)std::ceil(maxY));

		// 如果完全在范围外，直接返回
		if (startX > endX || startY > endY)
			return;

		double area = signedArea2D(dst0, dst1, dst2);
		if (std::abs(area) < 0.5)
			return; // 三角形太小

		for (int py = startY; py <= endY; py++) {
			for (int px = startX; px <= endX; px++) {
				Point2D p(px + 0.5, py + 0.5);

				double u, v, w;
				if (!barycentric2D(p, dst0, dst1, dst2, u, v, w))
					continue;

				const double eps = -0.001;
				if (u < eps || v < eps || w < eps)
					continue;

				double srcU = std::max(0.0, std::min(1.0, v));
				double srcV = std::max(0.0, std::min(1.0, w));

				Color srcColor = src.sample(srcU, srcV);
				Color finalColor = srcColor * tint;

				setPixel(px, py, finalColor);
			}
		}
	}
};

// ============================================================================
// 材质类型枚举
// ============================================================================

enum MaterialType {
	DIFFUSE,
	REFLECTIVE,
	EMISSIVE // 光源
};

// ============================================================================
// 材质类
// ============================================================================

struct Material {
	MaterialType type;
	Color baseColor;
	Color emissionColor;
	double reflectivity; // 反射率(0-1)
	Material(MaterialType t = DIFFUSE, const Color &color = Color(0.8, 0.8, 0.8))
		: type(t), baseColor(color), emissionColor(0, 0, 0), reflectivity(0.9) {
	}

	static Material Diffuse(const Color &color) {
		return Material(DIFFUSE, color);
	}

	static Material Reflective(const Color &color, double refl = 0.95) {
		Material m(REFLECTIVE, color);
		m.reflectivity = refl;
		return m;
	}

	static Material Emissive(const Color &color, const Color &emission) {
		Material m(EMISSIVE, color);
		m.emissionColor = emission;
		return m;
	}
};

// ============================================================================
// 三角形类
// ============================================================================

class Triangle {
public:
	Vec3 v0, v1, v2; // 三个顶点
	Vec3 normal;
	Material material;
	int id; // 唯一标识符，用于避免自相交

	Triangle() : id(-1) {
	}

	Triangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2,
		const Material &mat = Material(), int id = -1)
		: v0(v0), v1(v1), v2(v2), material(mat), id(id) {
		computeNormal();
	}

	void computeNormal() {
		normal = (v1 - v0).cross(v2 - v0).normalize();
	}

	// 获取三角形中心
	Vec3 center() const {
		return (v0 + v1 + v2) / 3.0;
	}

	// 获取三角形面积
	double area() const {
		return (v1 - v0).cross(v2 - v0).length() / 2.0;
	}

	// 计算重心坐标
	bool getBarycentricCoords(const Vec3 &p, double &u, double &v, double &w) const {
		Vec3 v0v1 = v1 - v0;
		Vec3 v0v2 = v2 - v0;
		Vec3 v0p = p - v0;

		double d00 = v0v1.dot(v0v1);
		double d01 = v0v1.dot(v0v2);
		double d11 = v0v2.dot(v0v2);
		double d20 = v0p.dot(v0v1);
		double d21 = v0p.dot(v0v2);

		double denom = d00 * d11 - d01 * d01;
		if (std::abs(denom) < 1e-10)
			return false;

		v = (d11 * d20 - d01 * d21) / denom;
		w = (d00 * d21 - d01 * d20) / denom;
		u = 1.0 - v - w;

		return true;
	}

	// 检查点是否在三角形内部（使用重心坐标）
	bool containsPoint(const Vec3 &p, double epsilon = 1e-6) const {
		double u, v, w;
		if (!getBarycentricCoords(p, u, v, w))
			return false;
		return u >= -epsilon && v >= -epsilon && w >= -epsilon;
	}

	// 射线与三角形相交 (Möller–Trumbore算法)
	bool intersectRay(const Vec3 &origin, const Vec3 &dir, double &t, double &u, double &v) const {
		const double EPSILON = 1e-10;
		Vec3 edge1 = v1 - v0;
		Vec3 edge2 = v2 - v0;
		Vec3 h = dir.cross(edge2);
		double a = edge1.dot(h);

		if (std::abs(a) < EPSILON)
			return false;

		double f = 1.0 / a;
		Vec3 s = origin - v0;
		u = f * s.dot(h);
		if (u < 0.0 || u > 1.0)
			return false;

		Vec3 q = s.cross(edge1);
		v = f * dir.dot(q);
		if (v < 0.0 || u + v > 1.0)
			return false;

		t = f * edge2.dot(q);
		return t > EPSILON;
	}

	// 获取平面方程 (ax + by + cz + d = 0)
	void getPlaneEquation(double &a, double &b, double &c, double &d) const {
		a = normal.x;
		b = normal.y;
		c = normal.z;
		d = -normal.dot(v0);
	}

	// 射线与三角形所在平面的交点
	bool intersectPlane(const Vec3 &origin, const Vec3 &dir, Vec3 &intersection) const {
		double denom = normal.dot(dir);
		if (std::abs(denom) < 1e-10)
			return false;

		double t = normal.dot(v0 - origin) / denom;
		if (t < 0)
			return false;

		intersection = origin + dir * t;
		return true;
	}

	// 将世界坐标点转换为三角形的UV坐标
	// 在 Triangle类中，修改 worldToUV 函数
	bool worldToUV(const Vec3 &worldPoint, double &u, double &v) const {
		// 首先将点投影到三角形平面上
		Vec3 pointOnPlane = worldPoint - normal * (worldPoint - v0).dot(normal);

		// 计算重心坐标
		Vec3 v0v1 = v1 - v0;
		Vec3 v0v2 = v2 - v0;
		Vec3 v0p = pointOnPlane - v0;

		double d00 = v0v1.dot(v0v1);
		double d01 = v0v1.dot(v0v2);
		double d11 = v0v2.dot(v0v2);
		double d20 = v0p.dot(v0v1);
		double d21 = v0p.dot(v0v2);

		double denom = d00 * d11 - d01 * d01;
		if (std::abs(denom) < 1e-10)
			return false;

		double bv = (d11 * d20 - d01 * d21) / denom; // v1的权重
		double bw = (d00 * d21 - d01 * d20) / denom; // v2的权重
		// bu = 1 - bv - bw  // v0的权重

		// UV坐标：v0=(0,0), v1=(1,0), v2=(0,1)
		u = bv;
		v = bw;
		return true;
	}

	// 将UV坐标转换为世界坐标
	Vec3 uvToWorld(double u, double v) const {
		return v0 * (1.0 - u - v) + v1 * u + v2 * v;
	}

	// 获取AABB包围盒
	void getAABB(Vec3 &minP, Vec3 &maxP) const {
		minP.x = std::min({ v0.x, v1.x, v2.x });
		minP.y = std::min({ v0.y, v1.y, v2.y });
		minP.z = std::min({ v0.z, v1.z, v2.z });
		maxP.x = std::max({ v0.x, v1.x, v2.x });
		maxP.y = std::max({ v0.y, v1.y, v2.y });
		maxP.z = std::max({ v0.z, v1.z, v2.z });
	}
};

// ============================================================================
// 视锥体类（由apex和底面三角形定义的四面体）
// ============================================================================

class Frustum {
public:
	Vec3 apex;
	Triangle base; // 底面三角形
	Vec3 planes[4]; // 4个侧面的法向量（指向内部）
	double planeDs[4]; // 平面方程的d值
	Frustum(const Vec3 &apex, const Triangle &base) : apex(apex), base(base) {
		computePlanes();
	}

	void computePlanes() {
		// 计算4个平面（3个侧面 + 1个底面）
		// 侧面由apex和底面的边构成
		Vec3 edges[3][2] = {
			{ base.v0, base.v1 },
			{ base.v1, base.v2 },
			{ base.v2, base.v0 }
		};

		Vec3 baseCenter = base.center();

		for (int i = 0; i < 3; i++) {
			Vec3 edge = edges[i][1] - edges[i][0];
			Vec3 toApex = apex - edges[i][0];
			planes[i] = edge.cross(toApex).normalize();
			// 确保法向量指向视锥体内部
			Vec3 toCenter = baseCenter - edges[i][0];
			if (planes[i].dot(toCenter) < 0) {
				planes[i] = -planes[i];
			}
			planeDs[i] = -planes[i].dot(edges[i][0]);
		}

		// 底面（远平面）
		planes[3] = base.normal;
		Vec3 apexToBase = baseCenter - apex;
		if (planes[3].dot(apexToBase) < 0) {
			planes[3] = -planes[3];
		}
		planeDs[3] = -planes[3].dot(base.v0);
	}

	// 检查点是否在视锥体内
	bool containsPoint(const Vec3 &p) const {
		// 检查点是否在apex的前方（相对于底面）
		Vec3 apexToP = p - apex;
		Vec3 apexToBase = base.center() - apex;
		if (apexToP.dot(apexToBase) < 0)
			return false;

		// 检查是否在所有平面的正确一侧
		for (int i = 0; i < 4; i++) {
			double dist = planes[i].dot(p) + planeDs[i];
			if (dist < -1e-6)
				return false;
		}
		return true;
	}

	// 检查三角形是否与视锥体相交或在其内部
	bool intersectsOrContainsTriangle(const Triangle &tri) const {
		// 快速检测：如果三角形的任意顶点在视锥体内
		if (containsPoint(tri.v0) || containsPoint(tri.v1) || containsPoint(tri.v2))
			return true;

		// 检查视锥体的边是否与三角形相交
		Vec3 frustumVerts[4] = { apex, base.v0, base.v1, base.v2 };
		int edges[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 2, 3 }, { 3, 1 } };

		for (int i = 0; i < 6; i++) {
			Vec3 p1 = frustumVerts[edges[i][0]];
			Vec3 p2 = frustumVerts[edges[i][1]];
			Vec3 dir = (p2 - p1).normalize();
			double maxT = (p2 - p1).length();

			double t, u, v;
			if (tri.intersectRay(p1, dir, t, u, v) && t < maxT) {
				return true;
			}
		}

		// 检查三角形的边是否与视锥体的面相交
		Vec3 triVerts[3] = { tri.v0, tri.v1, tri.v2 };
		Triangle frustumFaces[4] = {
			Triangle(apex, base.v0, base.v1),
			Triangle(apex, base.v1, base.v2),
			Triangle(apex, base.v2, base.v0),
			base
		};

		for (int i = 0; i < 3; i++) {
			Vec3 p1 = triVerts[i];
			Vec3 p2 = triVerts[(i + 1) % 3];
			Vec3 dir = (p2 - p1).normalize();
			double maxT = (p2 - p1).length();

			for (int j = 0; j < 4; j++) {
				double t, u, v;
				if (frustumFaces[j].intersectRay(p1, dir, t, u, v) && t < maxT) {
					return true;
				}
			}
		}

		return false;
	}
};

//============================================================================
// 简化的可见性检测类
// ============================================================================

class VisibilityChecker {
public:
	Vec3 cameraOrigin;
	Triangle viewport;
	Vec3 viewDir; // 视线方向

	VisibilityChecker(const Vec3 &origin, const Triangle &vp)
		: cameraOrigin(origin), viewport(vp) {
		viewDir = (viewport.center() - cameraOrigin).normalize();
	}

	// 检查三角形是否可能在视野内
	// 简化方法：检查三角形是否在相机前方，且投影到viewport平面上有交集
	bool isTriangleVisible(const Triangle &tri) const {
		// 检查三角形中心是否在相机前方
		Vec3 toTri = tri.center() - cameraOrigin;
		if (toTri.dot(viewDir) <= 0.01)
			return false;

		// 检查三角形的任意顶点投影是否可能落在viewport附近
		Vec3 verts[3] = { tri.v0, tri.v1, tri.v2 };
		int validCount = 0;

		for (int i = 0; i < 3; i++) {
			Vec3 dir = (verts[i] - cameraOrigin).normalize();
			// 检查方向是否大致朝向viewport
			if (dir.dot(viewDir) > 0.01) {
				validCount++;
			}
		}

		return validCount > 0;
	}

	// 计算三角形顶点在viewport平面上的投影
	// 返回是否所有顶点都能成功投影
	bool projectTriangle(const Triangle &tri, Vec3 projected[3]) const {
		Vec3 verts[3] = { tri.v0, tri.v1, tri.v2 };

		for (int i = 0; i < 3; i++) {
			Vec3 dir = (verts[i] - cameraOrigin);
			double dirLen = dir.length();
			if (dirLen < 1e-10)
				return false;
			dir = dir / dirLen;

			// 与viewport平面求交
			double denom = viewport.normal.dot(dir);
			if (std::abs(denom) < 1e-10)
				return false;

			double t = viewport.normal.dot(viewport.v0 - cameraOrigin) / denom;
			if (t <= 0)
				return false; // 在相机后方

			projected[i] = cameraOrigin + dir * t;
		}
		return true;
	}
};

// ============================================================================
// 前向声明
// ============================================================================

class Scene;

// ============================================================================
// 渲染函数声明
// ============================================================================

// 新增一个辅助结构来传递viewport信息
struct ViewportInfo {
	bool isRectViewport; // 是否是矩形viewport（相机viewport）
	Vec3 topLeft;
	Vec3 right, down;
	double width, height;

	ViewportInfo() : isRectViewport(false) {
	}

	ViewportInfo(const Vec3 &tl, const Vec3 &r, const Vec3 &d, double w, double h)
		: isRectViewport(true), topLeft(tl), right(r), down(d), width(w), height(h) {
	}

	// 将世界坐标转换为UV
	bool worldToUV(const Vec3 &worldPoint, double &u, double &v) const {
		Vec3 toPoint = worldPoint - topLeft;
		u = toPoint.dot(right) / width;
		v = toPoint.dot(down) / height;
		return true;
	}
};

Texture renderTriangleWithTriangle(
	const std::vector<Triangle *> &triangles,
	const Vec3 &cameraOrigin,
	const Triangle &viewport,
	int estimatedWidth,
	int estimatedHeight,
	int maxDepth,
	int currentDepth,
	int excludeId,
	const ViewportInfo *vpInfo = nullptr // 新增参数
);

// ============================================================================
// 场景类
// ============================================================================

class Scene {
public:
	std::vector<Triangle> triangles;
	int nextId = 0;

	void addTriangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Material &mat) {
		triangles.emplace_back(v0, v1, v2, mat, nextId++);
	}

	// 添加一个四边形（由两个三角形组成）
	void addQuad(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3, const Material &mat) {
		addTriangle(v0, v1, v2, mat);
		addTriangle(v0, v2, v3, mat);
	}

	// 添加一个盒子
	void addBox(const Vec3 &minP, const Vec3 &maxP, const Material &mat) {
		Vec3 v[8] = {
			Vec3(minP.x, minP.y, minP.z),
			Vec3(maxP.x, minP.y, minP.z),
			Vec3(maxP.x, maxP.y, minP.z),
			Vec3(minP.x, maxP.y, minP.z),
			Vec3(minP.x, minP.y, maxP.z),
			Vec3(maxP.x, minP.y, maxP.z),
			Vec3(maxP.x, maxP.y, maxP.z),
			Vec3(minP.x, maxP.y, maxP.z)
		};

		// 前面 (z = maxP.z)
		addQuad(v[4], v[5], v[6], v[7], mat);
		// 后面 (z = minP.z)
		addQuad(v[1], v[0], v[3], v[2], mat);
		// 左面 (x = minP.x)
		addQuad(v[0], v[4], v[7], v[3], mat);
		// 右面 (x = maxP.x)
		addQuad(v[5], v[1], v[2], v[6], mat);
		// 顶面 (y = maxP.y)
		addQuad(v[7], v[6], v[2], v[3], mat);
		// 底面 (y = minP.y)
		addQuad(v[0], v[1], v[5], v[4], mat);
	}

	std::vector<Triangle *> getTrianglePtrs() {
		std::vector<Triangle *> ptrs;
		for (auto &tri : triangles) {
			ptrs.push_back(&tri);
		}
		return ptrs;
	}
};
Texture renderTriangleWithTriangle(
	const std::vector<Triangle *> &triangles,
	const Vec3 &cameraOrigin,
	const Triangle &viewport,
	int estimatedWidth,
	int estimatedHeight,
	int maxDepth,
	int currentDepth,
	int excludeId,
	const ViewportInfo *vpInfo) {
	Texture result(estimatedWidth, estimatedHeight);

	// 终止条件
	if (currentDepth >= maxDepth || estimatedWidth < 4 || estimatedHeight < 4) {
		if (viewport.material.type == EMISSIVE) {
			result.clear(viewport.material.emissionColor);
		} else {
			result.clear(viewport.material.baseColor * 0.3);
		}
		return result;
	}

	// 漫反射材质：直接返回着色后的基础颜色
	if (viewport.material.type == DIFFUSE) {
		Vec3 lightDir = Vec3(0, 1, 0.5).normalize();
		double intensity = std::max(0.3, std::abs(viewport.normal.dot(lightDir)));
		result.clear(viewport.material.baseColor * intensity);
		return result;
	}

	// 发光材质
	if (viewport.material.type == EMISSIVE) {
		result.clear(viewport.material.emissionColor);
		return result;
	}

	// ========== 反射材质处理 ==========

	// 初始化为暗色（无反射时的基础色）
	result.clear(viewport.material.baseColor * 0.1);

	// 创建可见性检测器
	VisibilityChecker checker(cameraOrigin, viewport);

	// 收集可见三角形并按距离排序
	struct TriangleDist {
		Triangle *tri;
		double distance;
	};
	std::vector<TriangleDist> visibleTriangles;

	for (Triangle *tri : triangles) {
		if (tri->id == viewport.id || tri->id == excludeId)
			continue;

		if (checker.isTriangleVisible(*tri)) {
			double dist = (tri->center() - cameraOrigin).length();
			visibleTriangles.push_back({ tri, dist });
		}
	}

	// 按距离排序（由远到近）
	std::sort(visibleTriangles.begin(), visibleTriangles.end(), [](const TriangleDist &a, const TriangleDist &b) {
		return a.distance > b.distance;
	});

	std::cout << "Depth " << currentDepth << ": Found " << visibleTriangles.size()
			  << " visible triangles" << std::endl;

	for (const auto &td : visibleTriangles) {
		Triangle *tri = td.tri;

		Vec3 projected[3];
		if (!checker.projectTriangle(*tri, projected))
			continue;

		// ===== 修改UV计算部分 =====
		Point2D uvCoords[3];
		bool uvValid = true;

		for (int i = 0; i < 3; i++) {
			double u, v;
			if (vpInfo && vpInfo->isRectViewport) {
				// 使用矩形viewport的UV坐标系
				vpInfo->worldToUV(projected[i], u, v);
			} else {
				// 使用三角形的UV坐标系
				if (!viewport.worldToUV(projected[i], u, v)) {
					uvValid = false;
					break;
				}
			}
			uvCoords[i] = Point2D(u, v);
		}
		if (!uvValid)
			continue;

		// ===== 添加调试输出 =====
		if (currentDepth == 0 && tri->material.type == REFLECTIVE) {
			std::cout << "Metal tri " << tri->id << " UV: ("
					  << uvCoords[0].x << "," << uvCoords[0].y << ") ("
					  << uvCoords[1].x << "," << uvCoords[1].y << ") ("
					  << uvCoords[2].x << "," << uvCoords[2].y << ")" << std::endl;
		}

		// 检查投影是否在viewport范围内
		double minU = std::min({ uvCoords[0].x, uvCoords[1].x, uvCoords[2].x });
		double maxU = std::max({ uvCoords[0].x, uvCoords[1].x, uvCoords[2].x });
		double minV = std::min({ uvCoords[0].y, uvCoords[1].y, uvCoords[2].y });
		double maxV = std::max({ uvCoords[0].y, uvCoords[1].y, uvCoords[2].y });

		// 如果完全在viewport外，跳过
		if (maxU < 0 || minU > 1 || maxV < 0 || minV > 1)
			continue;

		// 获取目标三角形的纹理
		Texture subTexture;
		if (tri->material.type == REFLECTIVE) {
			// 对于反射材质，递归计算
			// 将相机关于目标三角形对称
			Vec3 newOrigin = cameraOrigin.reflect(tri->center(), tri->normal);

			double projArea = std::abs(signedArea2D(uvCoords[0], uvCoords[1], uvCoords[2]));
			int subW = std::max(8, (int)(estimatedWidth * std::sqrt(projArea) * 0.8));
			int subH = std::max(8, (int)(estimatedHeight * std::sqrt(projArea) * 0.8));
			subW = std::min(subW, estimatedWidth);
			subH = std::min(subH, estimatedHeight);

			subTexture = renderTriangleWithTriangle(
				triangles, newOrigin, *tri,
				subW, subH, maxDepth, currentDepth + 1, viewport.id);
			// 在处理反射材质时，添加测试
			// if (viewport.material.type == REFLECTIVE && currentDepth > 0) {
			// 	// 测试：直接返回一个明显的颜色，验证反射路径是否被执行
			// 	result.clear(Color(1, 0, 1)); // 洋红色
			// 	return result;
			// }
		} else if (tri->material.type == EMISSIVE) {
			// 发光材质
			subTexture = Texture(16, 16, tri->material.emissionColor);
		} else {
			// 漫反射材质
			Vec3 lightDir = Vec3(0, 1, 0.5).normalize();
			double intensity = std::max(0.3, std::abs(tri->normal.dot(lightDir)));
			subTexture = Texture(16, 16, tri->material.baseColor * intensity);
		}

		// 计算染色（金属会给反射染色）
		Color tint = viewport.material.baseColor * viewport.material.reflectivity;

		// 粘贴纹理
		result.blendTriangleTextureUV(subTexture, uvCoords[0], uvCoords[1], uvCoords[2], tint, 1.0);
	}

	return result;
}

// // ============================================================================
// // 核心渲染函数
// // ============================================================================
// Texture renderTriangleWithTriangle(
//     const std::vector<Triangle*>& triangles,
//     const Vec3& cameraOrigin,
//     const Triangle& viewport,
//     int estimatedWidth,
//     int estimatedHeight,
//     int maxDepth,
//     int currentDepth,
//     int excludeId)
// {
//     // 创建当前视口的纹理
//     Texture result(estimatedWidth, estimatedHeight, viewport.material.baseColor);
//
//     // 终止条件：达到最大深度
//     if (currentDepth >= maxDepth) {
//         if (viewport.material.type == EMISSIVE) {
//             result.clear(viewport.material.emissionColor);
//         }
//         return result;
//     }
//
//     // 漫反射材质：直接返回基础颜色
//     if (viewport.material.type == DIFFUSE) {
//         Vec3 lightDir = Vec3(0.5, 1, 0.3).normalize();
//         double intensity = std::max(0.2, std::abs(viewport.normal.dot(lightDir)));
//         Color shadedColor = viewport.material.baseColor * intensity;
//         result.clear(shadedColor);
//         return result;
//     }
//
//     // 发光材质
//     if (viewport.material.type == EMISSIVE) {
//         result.clear(viewport.material.emissionColor);
//         return result;
//     }
//
//     //========== 反射材质处理 ==========
//
//     // 用较暗的基础色初始化（环境光贡献）
//     Color ambientColor = viewport.material.baseColor * 0.05;
//     result.clear(ambientColor);
//
//     // 构建视锥体
//     Frustum frustum(cameraOrigin, viewport);
//
//     // 收集与视锥体相交的三角形，按距离排序（由远到近）
//     struct TriangleDist {
//         Triangle* tri;
//         double distance;
//         bool operator>(const TriangleDist& other) const {
//             return distance > other.distance;
//         }
//     };
//
//     std::vector<TriangleDist> visibleTriangles;
//
//     for (Triangle* tri : triangles) {
//         if (tri->id == viewport.id || tri->id == excludeId) continue;
//
//         Vec3 triCenter = tri->center();
//         Vec3 toTri = triCenter - cameraOrigin;
//         Vec3 viewDir = viewport.center() - cameraOrigin;
//         if (toTri.dot(viewDir) <= 0) continue;
//
//         if (frustum.intersectsOrContainsTriangle(*tri)) {
//             double dist = toTri.length();
//             visibleTriangles.push_back({tri, dist});
//         }
//     }
//
//     // 按距离排序（由远到近）
//     std::sort(visibleTriangles.begin(), visibleTriangles.end(),[](const TriangleDist& a, const TriangleDist& b) {
//             return a.distance > b.distance;
//         });
//
//     // ========== 遍历并粘贴纹理 ==========
//
//     for (const auto& td : visibleTriangles) {
//         Triangle* tri = td.tri;
//
//         // 计算目标三角形三个顶点投影到当前viewport平面上的交点
//         Vec3 triVerts[3] = {tri->v0, tri->v1, tri->v2};
//         Vec3 projectedVerts[3];
//         bool allValid = true;
//
//         for (int i = 0; i < 3; i++) {
//             Vec3 dir = (triVerts[i] - cameraOrigin).normalize();
//             Vec3 intersection;
//
//             if (viewport.intersectPlane(cameraOrigin, dir, intersection)) {
//                 projectedVerts[i] = intersection;
//             } else {
//                 allValid = false;
//                 break;
//             }
//         }
//
//         if (!allValid) continue;
//
//         // 将投影点转换为viewport的UV坐标
//         Point2D uvCoords[3];
//         bool uvValid = true;
//         for (int i = 0; i < 3; i++) {
//             double u, v;
//             if (viewport.worldToUV(projectedVerts[i], u, v)) {
//                 uvCoords[i] = Point2D(u, v);
//             } else {
//                 uvValid = false;
//                 break;
//             }
//         }
//
//         if (!uvValid) continue;
//
//         // 递归获取目标三角形的纹理
//         Vec3 newCameraOrigin = cameraOrigin.reflect(tri->center(), tri->normal);
//         // 估算子纹理大小
//         double projArea = std::abs(signedArea2D(uvCoords[0], uvCoords[1], uvCoords[2]));
//         int subWidth = std::max(8, (int)(estimatedWidth * std::sqrt(projArea)));
//         int subHeight = std::max(8, (int)(estimatedHeight * std::sqrt(projArea)));
//         subWidth = std::min(subWidth, estimatedWidth);
//         subHeight = std::min(subHeight, estimatedHeight);
//         Texture subTexture = renderTriangleWithTriangle(
//             triangles, newCameraOrigin, *tri,
//             subWidth, subHeight,
//             maxDepth, currentDepth + 1, viewport.id
//         );
//
//         // 计算金属染色和反射率
//         Color metalTint = viewport.material.baseColor * viewport.material.reflectivity;
//         // 直接将子纹理拉伸粘贴到当前纹理
//         result.blendTriangleTextureUV(subTexture, uvCoords[0], uvCoords[1], uvCoords[2], metalTint, 1.0);
//     }
//
//     return result;
// }
//
// Texture renderTriangleWithTriangle(
// 	const std::vector<Triangle *> &triangles,
// 	const Vec3 &cameraOrigin,
// 	const Triangle &viewport,
// 	int estimatedWidth,
// 	int estimatedHeight,
// 	int maxDepth,
// 	int currentDepth,
// 	int excludeId) {
// 	// 创建当前视口的纹理
// 	Texture result(estimatedWidth, estimatedHeight, viewport.material.baseColor);
//
// 	// 终止条件：达到最大深度或面积太小
// 	if (currentDepth >= maxDepth) {
// 		// 对于发光材质，返回发光颜色
// 		if (viewport.material.type == EMISSIVE) {
// 			result.clear(viewport.material.emissionColor);
// 		}
// 		return result;
// 	}
//
// 	// 如果是漫反射材质，直接返回基础颜色
// 	if (viewport.material.type == DIFFUSE) {
// 		// 添加简单的着色（基于法线方向）
// 		Vec3 lightDir = Vec3(0.5, 1, 0.3).normalize();
// 		double intensity = std::max(0.2, std::abs(viewport.normal.dot(lightDir)));
// 		Color shadedColor = viewport.material.baseColor * intensity;
// 		result.clear(shadedColor);
// 		return result;
// 	}
//
// 	// 如果是发光材质
// 	if (viewport.material.type == EMISSIVE) {
// 		result.clear(viewport.material.emissionColor);
// 		return result;
// 	}
//
// 	// 对于反射材质，需要递归处理
// 	// 首先用基础颜色初始化
// 	result.clear(viewport.material.baseColor * 0.05); // 微弱的基础色
//
// 	// 构建视锥体
// 	Frustum frustum(cameraOrigin, viewport);
//
// 	// 收集与视锥体相交的三角形，并按距离排序
// 	struct TriangleDist {
// 		Triangle *tri;
// 		double distance;
// 		bool operator>(const TriangleDist &other) const {
// 			return distance > other.distance; // 远的优先（用于由远到近渲染）
// 		}
// 	};
//
// 	std::priority_queue<TriangleDist, std::vector<TriangleDist>, std::greater<TriangleDist>> pq;
//
// 	for (Triangle *tri : triangles) {
// 		// 跳过自身和被排除的三角形
// 		if (tri->id == viewport.id || tri->id == excludeId)
// 			continue;
//
// 		// 检查三角形是否在相机前方
// 		Vec3 triCenter = tri->center();
// 		Vec3 toTri = triCenter - cameraOrigin;
// 		Vec3 viewDir = viewport.center() - cameraOrigin;
// 		if (toTri.dot(viewDir) <= 0)
// 			continue;
//
// 		// 检查三角形是否与视锥体相交
// 		if (frustum.intersectsOrContainsTriangle(*tri)) {
// 			double dist = (triCenter - cameraOrigin).length();
// 			pq.push({ tri, dist });
// 		}
// 	}
//
// 	// 将优先队列转换为向量（从远到近）
// 	std::vector<Triangle *> sortedTriangles;
// 	while (!pq.empty()) {
// 		sortedTriangles.push_back(pq.top().tri);
// 		pq.pop();
// 	}
// 	std::reverse(sortedTriangles.begin(), sortedTriangles.end()); // 变成由远到近
//
// 	// 遍历每个三角形，将其投影到当前视口上
// 	for (Triangle *tri : sortedTriangles) {
// 		// 计算目标三角形顶点在当前视口上的投影位置
// 		Vec3 projectedVerts[3];
// 		bool allValid = true;
//
// 		Vec3 triVerts[3] = { tri->v0, tri->v1, tri->v2 };
//
// 		for (int i = 0; i < 3; i++) {
// 			Vec3 dir = (triVerts[i] - cameraOrigin).normalize();
// 			Vec3 intersection;
//
// 			if (viewport.intersectPlane(cameraOrigin, dir, intersection)) {
// 				// 检查交点是否在视口三角形内（或附近）
// 				projectedVerts[i] = intersection;
// 			} else {
// 				allValid = false;
// 				break;
// 			}
// 		}
//
// 		if (!allValid)
// 			continue;
//
// 		// 将投影的3D坐标转换为视口的2D UV坐标
// 		Point2D uvCoords[3];
// 		for (int i = 0; i < 3; i++) {
// 			double u, v;
// 			viewport.worldToUV(projectedVerts[i], u, v);
// 			uvCoords[i] = Point2D(u, v);
// 		}
//
// 		// 递归获取目标三角形应显示的纹理
// 		// 对于反射，将相机原点关于目标三角形平面对称
// 		Vec3 newCameraOrigin = cameraOrigin.reflect(tri->center(), tri->normal);
//
// 		// 估算子纹理大小（基于投影面积）
// 		double projArea = std::abs(signedArea2D(uvCoords[0], uvCoords[1], uvCoords[2]));
// 		int subWidth = std::max(4, (int)(estimatedWidth * std::sqrt(projArea) * 0.5));
// 		int subHeight = std::max(4, (int)(estimatedHeight * std::sqrt(projArea) * 0.5));
//
// 		// 限制子纹理大小
// 		subWidth = std::min(subWidth, estimatedWidth);
// 		subHeight = std::min(subHeight, estimatedHeight);
// 		Texture subTexture = renderTriangleWithTriangle(
// 			triangles, newCameraOrigin, *tri,
// 			subWidth, subHeight,
// 			maxDepth, currentDepth + 1, viewport.id);
//
// 		// 将子纹理变换并绘制到当前纹理上
// 		// 使用扫描线算法进行三角形光栅化
//
// 		// 计算包围盒
// 		double minU = std::min({ uvCoords[0].x, uvCoords[1].x, uvCoords[2].x });
// 		double maxU = std::max({ uvCoords[0].x, uvCoords[1].x, uvCoords[2].x });
// 		double minV = std::min({ uvCoords[0].y, uvCoords[1].y, uvCoords[2].y });
// 		double maxV = std::max({ uvCoords[0].y, uvCoords[1].y, uvCoords[2].y });
//
// 		// 转换为像素坐标
// 		int startX = std::max(0, (int)(minU * estimatedWidth) - 1);
// 		int endX = std::min(estimatedWidth - 1, (int)(maxU * estimatedWidth) + 1);
// 		int startY = std::max(0, (int)(minV * estimatedHeight) - 1);
// 		int endY = std::min(estimatedHeight - 1, (int)(maxV * estimatedHeight) + 1);
//
// 		// 计算到目标三角形的距离（用于深度测试）
// 		double triDist = (tri->center() - cameraOrigin).length();
//
// 		// 光栅化三角形
// 		for (int py = startY; py <= endY; py++) {
// 			for (int px = startX; px <= endX; px++) {
// 				// 当前像素的UV坐标
// 				double u = (px + 0.5) / estimatedWidth;
// 				double v = (py + 0.5) / estimatedHeight;
// 				Point2D p(u, v);
//
// 				// 计算相对于投影三角形的重心坐标
// 				double bu, bv, bw;
// 				if (!barycentric2D(p, uvCoords[0], uvCoords[1], uvCoords[2], bu, bv, bw))
// 					continue;
//
// 				// 检查是否在三角形内
// 				const double eps = -0.001; // 稍微放宽以避免间隙
// 				if (bu < eps || bv < eps || bw < eps)
// 					continue;
//
// 				// 使用重心坐标在子纹理中采样
// 				// 子纹理的UV对应于目标三角形的顶点
// 				// 在源三角形中，顶点UV为 (0,0), (1,0), (0,1)
// 				double srcU = bv; // 对应v1
// 				double srcV = bw; // 对应v2
//
// 				Color sampledColor = subTexture.sample(srcU, srcV);
//
// 				// // 应用反射率衰减
// 				// sampledColor = sampledColor * viewport.material.reflectivity;
// 				//
// 				// // 混合到结果纹理（使用深度测试）
// 				// result.setPixelWithDepth(px, py, sampledColor, triDist);
//
// 				// 应用反射率衰减，并与金属基础色混合
// 				Color metalTint = viewport.material.baseColor;  // 金属染色
// 				sampledColor = sampledColor * metalTint * viewport.material.reflectivity;
//
// 				// 可选：添加非反射部分的基础色贡献
// 				Color finalColor = sampledColor + metalTint * (1.0 - viewport.material.reflectivity) * 0.1;
//
// 				result.setPixelWithDepth(px, py, sampledColor, triDist);
// 			}
// 		}
// 	}
//
// 	return result;
// }

// ============================================================================
// 相机类
// ============================================================================

class Camera {
public:
	Vec3 origin;
	Vec3 lookAt;
	Vec3 up;
	double fov; // 垂直视场角（度）
	int width, height;

	// 两个视口三角形
	Triangle viewportTri1, viewportTri2;

	// 新增：viewport矩形的四个角和坐标轴
	Vec3 vpTopLeft, vpTopRight, vpBottomLeft, vpBottomRight;
	Vec3 vpRight, vpDown; // viewport的水平和垂直方向
	Vec3 vpNormal;
	double vpWidth, vpHeight;

	Camera(const Vec3 &origin, const Vec3 &lookAt, const Vec3 &up,
		double fov, int width, int height)
		: origin(origin), lookAt(lookAt), up(up), fov(fov), width(width), height(height) {
		computeViewportTriangles();
	}

	void computeViewportTriangles() {
		Vec3 forward = (lookAt - origin).normalize();
		Vec3 right = forward.cross(up).normalize();
		Vec3 camUp = right.cross(forward).normalize();

		double aspectRatio = (double)width / height;
		double fovRad = fov * M_PI / 180.0;
		double viewportDist = 1.0;
		double viewportHeight = 2.0 * viewportDist * std::tan(fovRad / 2.0);
		double viewportWidth = viewportHeight * aspectRatio;

		Vec3 viewportCenter = origin + forward * viewportDist;

		Vec3 halfRight = right * (viewportWidth / 2.0);
		Vec3 halfUp = camUp * (viewportHeight / 2.0);

		// 保存viewport矩形信息
		vpTopLeft = viewportCenter - halfRight + halfUp;
		vpTopRight = viewportCenter + halfRight + halfUp;
		vpBottomLeft = viewportCenter - halfRight - halfUp;
		vpBottomRight = viewportCenter + halfRight - halfUp;

		vpRight = right;
		vpDown = -camUp; // 向下为正（图像坐标系）
		vpNormal = forward;
		vpWidth = viewportWidth;
		vpHeight = viewportHeight;

		Material viewportMat = Material::Reflective(Color(1, 1, 1), 1.0);
		viewportTri1 = Triangle(vpBottomLeft, vpBottomRight, vpTopLeft, viewportMat, -100);
		viewportTri2 = Triangle(vpBottomRight, vpTopRight, vpTopLeft, viewportMat, -101);
	}
	// 新增：将世界坐标转换为viewport的UV坐标（相对于整个矩形）
	bool worldToViewportUV(const Vec3 &worldPoint, double &u, double &v) const {
		// 将点投影到viewport平面
		Vec3 toPoint = worldPoint - vpTopLeft;

		// 计算在viewport坐标系中的位置
		u = toPoint.dot(vpRight) / vpWidth;
		v = toPoint.dot(vpDown) / vpHeight;

		return true;
	}

	Texture render(Scene &scene, int maxDepth = 3) {
		Texture finalImage(width, height);

		auto trianglePtrs = scene.getTrianglePtrs();

		// 创建viewport信息
		ViewportInfo vpInfo(vpTopLeft, vpRight, vpDown, vpWidth, vpHeight);

		std::cout << "Rendering viewport..." << std::endl;

		// 只渲染一次，使用矩形viewport
		// 创建一个虚拟的viewport三角形（实际上覆盖整个矩形）
		Material viewportMat = Material::Reflective(Color(1, 1, 1), 1.0);
		Triangle fullViewport(vpTopLeft, vpBottomRight, vpTopRight, viewportMat, -100);
		fullViewport.normal = vpNormal;
		Texture tex = renderTriangleWithTriangle(
			trianglePtrs, origin, fullViewport,
			width, height, maxDepth, 0, -1, &vpInfo);

		// 直接复制纹理到最终图像
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				double u = (x + 0.5) / width;
				double v = (y + 0.5) / height;
				finalImage.setPixel(x, y, tex.sample(u, v));
			}
		}

		return finalImage;
	}
};

// ============================================================================
// 创建康奈尔盒子场景
// ============================================================================

void createCornellBoxScene(Scene &scene) {
	// 康奈尔盒子尺寸
	double size = 5.0;
	double halfSize = size / 2.0;

	// 材质定义
	Material whiteDiffuse = Material::Diffuse(Color(0.73, 0.73, 0.73));
	Material redDiffuse = Material::Diffuse(Color(0.65, 0.05, 0.05));
	Material greenDiffuse = Material::Diffuse(Color(0.12, 0.45, 0.15));
	Material blueMetal = Material::Reflective(Color(0.1, 0.2, 0.6), 0.85);
	Material yellowMetal = Material::Reflective(Color(0.8, 0.7, 0.1), 0.85);
	Material lightMat = Material::Emissive(Color(1, 1, 1), Color(15, 15, 15));

	// 地板 (白色)
	scene.addQuad(
		Vec3(-halfSize, -halfSize, -halfSize),
		Vec3(halfSize, -halfSize, -halfSize),
		Vec3(halfSize, -halfSize, halfSize),
		Vec3(-halfSize, -halfSize, halfSize),
		whiteDiffuse);

	// 天花板 (白色)
	scene.addQuad(
		Vec3(-halfSize, halfSize, halfSize),
		Vec3(halfSize, halfSize, halfSize),
		Vec3(halfSize, halfSize, -halfSize),
		Vec3(-halfSize, halfSize, -halfSize),
		whiteDiffuse);

	// 后墙 (白色)
	scene.addQuad(
		Vec3(-halfSize, -halfSize, -halfSize),
		Vec3(-halfSize, halfSize, -halfSize),
		Vec3(halfSize, halfSize, -halfSize),
		Vec3(halfSize, -halfSize, -halfSize),
		whiteDiffuse);

	// 左墙 (红色)
	scene.addQuad(
		Vec3(-halfSize, -halfSize, halfSize),
		Vec3(-halfSize, halfSize, halfSize),
		Vec3(-halfSize, halfSize, -halfSize),
		Vec3(-halfSize, -halfSize, -halfSize),
		redDiffuse);

	// 右墙 (绿色)
	scene.addQuad(
		Vec3(halfSize, -halfSize, -halfSize),
		Vec3(halfSize, halfSize, -halfSize),
		Vec3(halfSize, halfSize, halfSize),
		Vec3(halfSize, -halfSize, halfSize),
		greenDiffuse);

	// 光源(天花板中央的小矩形)
	// double lightSize = 1.0;
	// scene.addQuad(
	// 	Vec3(-lightSize, halfSize - 0.01, -lightSize),
	// 	Vec3(-lightSize, halfSize - 0.01, lightSize),
	// 	Vec3(lightSize, halfSize - 0.01, lightSize),
	// 	Vec3(lightSize, halfSize - 0.01, -lightSize),
	// 	lightMat);

	// 蓝色金属盒子 (左侧)
	double box1Size = 1.2;
	Vec3 box1Min(-halfSize + 0.8, -halfSize, -0.5);
	Vec3 box1Max = box1Min + Vec3(box1Size, box1Size * 2, box1Size);
	scene.addBox(box1Min, box1Max, blueMetal);

	// 黄色金属盒子 (右侧)
	double box2Size = 1.2;
	Vec3 box2Min(halfSize - 0.8 - box2Size, -halfSize, 0.3);
	Vec3 box2Max = box2Min + Vec3(box2Size, box2Size * 1.5, box2Size);
	scene.addBox(box2Min, box2Max, yellowMetal);
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
	std::cout << "=== 面片变换渲染器 - 康奈尔盒子 ===" << std::endl;

	// 创建场景
	Scene scene;
	createCornellBoxScene(scene);

	std::cout << "场景创建完成，共" << scene.triangles.size() << " 个三角形" << std::endl;

	// 设置相机
	int width = 1024;
	int height = 1024;
	Vec3 cameraPos(0, 0, 12);
	Vec3 lookAt(0, 0, 0);
	Vec3 up(0, 1, 0);
	double fov = 45.0;

	Camera camera(cameraPos, lookAt, up, fov, width, height);

	std::cout << "开始渲染..." << std::endl;

	// 渲染
	int maxReflectionDepth = 3; // 最大反射深度
	Texture image = camera.render(scene, maxReflectionDepth);

	// 保存图像
	std::string filename = "cornell_box_rt9.ppm";
	image.writePPM(filename);

	std::cout << "渲染完成！图像已保存到 " << filename << std::endl;

	return 0;
}
