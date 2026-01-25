#define _USE_MATH_DEFINES
/*
 * 基于三角形面片的步进式纹理变换康奈尔盒渲染器（无递归，无AO/反射，仅面片遮挡关系）
 * 编译: g++ cornell_box_tri_step_transform.cpp -std=c++11 -O2 -o cornell_box_tri
 * 运行: ./cornell_box_tri
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

constexpr float EPS = 1e-6f;
constexpr float INF = 1e30f;

struct Vec3 {
	float x, y, z;
	Vec3(float xx = 0, float yy = 0, float zz = 0) : x(xx), y(yy), z(zz) {
	}
	Vec3 operator+(const Vec3 &b) const {
		return Vec3(x + b.x, y + b.y, z + b.z);
	}
	Vec3 operator-(const Vec3 &b) const {
		return Vec3(x - b.x, y - b.y, z - b.z);
	}
	Vec3 operator*(float s) const {
		return Vec3(x * s, y * s, z * s);
	}
	float dot(const Vec3 &b) const {
		return x * b.x + y * b.y + z * b.z;
	}
	Vec3 cross(const Vec3 &b) const {
		return Vec3(
			y * b.z - z * b.y,
			z * b.x - x * b.z,
			x * b.y - y * b.x);
	}
	float length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	Vec3 normalized() const {
		float l = length();
		return (*this) * (1.0f / l);
	}
};

struct Color {
	float r, g, b;
	Color(float rr = 0, float gg = 0, float bb = 0) : r(rr), g(gg), b(bb) {
	}
	Color operator*(float s) const {
		return Color(r * s, g * s, b * s);
	}
	Color operator+(const Color &c) const {
		return Color(r + c.r, g + c.g, b + c.b);
	}
	void clamp() {
		r = fmaxf(0.0f, fminf(1.0f, r));
		g = fmaxf(0.0f, fminf(1.0f, g));
		b = fmaxf(0.0f, fminf(1.0f, b));
	}
};

struct Texture {
	virtual Color at(float u, float v) const = 0;
	virtual ~Texture() {
	}
};
struct SolidTexture : Texture {
	Color c;
	SolidTexture(Color cc) : c(cc) {
	}
	Color at(float, float) const override {
		return c;
	}
};
struct CheckerTexture : Texture {
	Color a, b;
	float scale;
	CheckerTexture(Color c1, Color c2, float s = 10) : a(c1), b(c2), scale(s) {
	}
	Color at(float u, float v) const override {
		int xx = int(floor(u * scale)), yy = int(floor(v * scale));
		return ((xx + yy) % 2 == 0) ? a : b;
	}
};
struct MetalTexture : Texture {
	Color tint;
	MetalTexture(Color tt) : tint(tt) {
	}
	Color at(float, float) const override {
		return tint;
	}
};

struct Tri {
	Vec3 v0, v1, v2;
	Vec3 n; // 法线
	Vec3 uv0, uv1, uv2; // uv.xy
	Texture *tex;
	Tri(Vec3 a, Vec3 b, Vec3 c, Vec3 t0, Vec3 t1, Vec3 t2, Texture *tx)
		: v0(a), v1(b), v2(c), uv0(t0), uv1(t1), uv2(t2), tex(tx) {
		n = (v1 - v0).cross(v2 - v0).normalized();
	}
	// 射线相交
	bool intersect(const Vec3 &o, const Vec3 &d, float &t, float &u, float &v) const {
		// Moller-Trumbore
		Vec3 e1 = v1 - v0, e2 = v2 - v0, h = d.cross(e2), s = o - v0;
		float a = e1.dot(h);
		if (fabs(a) < EPS)
			return false;
		float f = 1.f / a;
		u = f * s.dot(h);
		if (u < 0 || u > 1)
			return false;
		Vec3 q = s.cross(e1);
		v = f * d.dot(q);
		if (v < 0 || u + v > 1)
			return false;
		t = f * e2.dot(q);
		return t > EPS;
	}
	// barycentric -> uv
	void bary2uv(float b0, float b1, float b2, float &u, float &v) const {
		u = uv0.x * b0 + uv1.x * b1 + uv2.x * b2;
		v = uv0.y * b0 + uv1.y * b1 + uv2.y * b2;
	}
	// 在射线上与面片交点对应的纹理坐标
	bool texcoord_at_hit(const Vec3 &o, const Vec3 &d, float &t, float &outu, float &outv) const {
		float uu, vv;
		if (!intersect(o, d, t, uu, vv))
			return false;
		float w = 1 - uu - vv;
		bary2uv(w, uu, vv, outu, outv);
		return true;
	}
};

struct Camera {
	Vec3 pos, target, up;
	float fov;
	Camera(Vec3 p, Vec3 t, Vec3 u, float f) : pos(p), target(t), up(u), fov(f) {
	}
};

// Cornell box三角分割，每个面（矩形）分两个三角
struct Scene {
	std::vector<Tri> tris;
	std::vector<Texture *> textures; // 管理资源释放
	~Scene() {
		for (Texture *t : textures)
			delete t;
	}
	Scene() {
		// 盒体(-1~1)
		Texture *red = new SolidTexture(Color(0.8, 0.15, 0.15)),
				*green = new SolidTexture(Color(0.15, 0.8, 0.15)),
				*white = new SolidTexture(Color(0.8, 0.8, 0.8)),
				*checker = new CheckerTexture(Color(0.9, 0.9, 0.9), Color(0.1, 0.1, 0.1), 8);
		textures = { red, green, white, checker };

		// --- 左壁(红)
		pushQuad(-1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, red);
		// --- 右壁(绿)
		pushQuad(1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, green);
		// --- 地板(棋盘)
		pushQuad(-1, -1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, checker);
		// --- 顶
		pushQuad(-1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, white);
		// --- 背面(白)
		pushQuad(-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, white);

		// 添加一个立方体（中心(-0.4,-0.5,0.2)，尺寸0.4）
		pushBox(-0.4f, -0.7f, 0.2f, 0.20f, white);
	}
	// 面片顶点uv为(0,0),(1,0),(1,1),(0,1)
	void pushQuad(float x0, float y0, float z0, float x1, float y1, float z1,
		float x2, float y2, float z2, float x3, float y3, float z3, Texture *t) {
		tris.emplace_back(Vec3(x0, y0, z0), Vec3(x1, y1, z1), Vec3(x2, y2, z2),
			Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0), t);
		tris.emplace_back(Vec3(x0, y0, z0), Vec3(x2, y2, z2), Vec3(x3, y3, z3),
			Vec3(0, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0), t);
	}
	// 六面体
	void pushBox(float cx, float cy, float cz, float r, Texture *t) {
		float x = cx, y = cy, z = cz, s = r;
		float X[2] = { x - s, x + s }, Y[2] = { y - s, y + s }, Z[2] = { z - s, z + s };
		// 六面六个pushQuad
		pushQuad(X[0], Y[0], Z[0], X[1], Y[0], Z[0], X[1], Y[1], Z[0], X[0], Y[1], Z[0], t); // 后
		pushQuad(X[0], Y[0], Z[1], X[1], Y[0], Z[1], X[1], Y[1], Z[1], X[0], Y[1], Z[1], t); // 前
		pushQuad(X[0], Y[0], Z[0], X[0], Y[1], Z[0], X[0], Y[1], Z[1], X[0], Y[0], Z[1], t); // 左
		pushQuad(X[1], Y[0], Z[0], X[1], Y[1], Z[0], X[1], Y[1], Z[1], X[1], Y[0], Z[1], t); // 右
		pushQuad(X[0], Y[1], Z[0], X[1], Y[1], Z[0], X[1], Y[1], Z[1], X[0], Y[1], Z[1], t); // 上
		pushQuad(X[0], Y[0], Z[0], X[1], Y[0], Z[0], X[1], Y[0], Z[1], X[0], Y[0], Z[1], t); // 下
	}
};

struct Viewport {
	int w, h;
	std::vector<Color> pixels;
	Viewport(int W, int H) : w(W), h(H), pixels(W * H) {
	}
	void set(int x, int y, const Color &c) {
		if (x >= 0 && y >= 0 && x < w && y < h)
			pixels[y * w + x] = c;
	}
	const Color &get(int x, int y) const {
		return pixels[y * w + x];
	}
};

struct Cand {
	float t; // 距离
	Color c;
	Cand(float _t, const Color &_c) : t(_t), c(_c) {
	}
};
bool dist_cmp(const Cand &a, const Cand &b) {
	return a.t > b.t;
}

// 主渲染器
void render(const Scene &scene, const Camera &cam, Viewport &vp) {
	// 视锥参数
	Vec3 forward = (cam.target - cam.pos).normalized();
	Vec3 right = forward.cross(cam.up).normalized();
	Vec3 up = right.cross(forward);
	float aspect = 1.0 * vp.w / vp.h;
	float imagePlaneDist = 1.0;
	float scale = tanf(cam.fov * 0.5f * M_PI / 180.f) * imagePlaneDist;

	for (int y = 0; y < vp.h; ++y)
		for (int x = 0; x < vp.w; ++x) {
			float fx = (2 * (x + 0.5f) / vp.w - 1) * aspect * scale;
			float fy = (1 - 2 * (y + 0.5f) / vp.h) * scale;
			Vec3 dir = (forward * imagePlaneDist + right * fx + up * fy).normalized();

			std::vector<Cand> cands;
			for (const auto &tri : scene.tris) {
				float t, u, v;
				if (tri.intersect(cam.pos, dir, t, u, v)) {
					float w = 1 - u - v, uu, vv;
					tri.bary2uv(w, u, v, uu, vv);
					Color col = tri.tex->at(uu, vv);
					cands.emplace_back(t, col);
				}
			}
			if (cands.empty()) {
				vp.set(x, y, Color(0, 0, 0));
				continue;
			}
			std::sort(cands.begin(), cands.end(), dist_cmp);
			Color col = cands.front().c;
			col.clamp();
			vp.set(x, y, col);
		}
	std::cerr << "Rendered." << std::endl;
}

void writePPM(const Viewport &vp, const char *fn) {
	FILE *f = fopen(fn, "wb");
	fprintf(f, "P6\n%d %d\n255\n", vp.w, vp.h);
	for (int y = 0; y < vp.h; ++y)
		for (int x = 0; x < vp.w; ++x) {
			Color c = vp.get(x, y);
			c.clamp();
			unsigned char rr = (unsigned char)(pow(c.r, 1 / 2.2f) * 255);
			unsigned char gg = (unsigned char)(pow(c.g, 1 / 2.2f) * 255);
			unsigned char bb = (unsigned char)(pow(c.b, 1 / 2.2f) * 255);
			fwrite(&rr, 1, 1, f);
			fwrite(&gg, 1, 1, f);
			fwrite(&bb, 1, 1, f);
		}
	fclose(f);
}

int main() {
	int width = 512, height = 512;
	Scene scene;
	Camera cam(
		Vec3(0, 0, 4),
		Vec3(0, 0, 0),
		Vec3(0, 1, 0),
		50.f);
	Viewport vp(width, height);
	render(scene, cam, vp);
	writePPM(vp, "out.ppm");
	std::cout << "Saved to out.ppm\n";
	return 0;
}
