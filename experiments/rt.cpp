#define _USE_MATH_DEFINES
/*
 *  Cornell Box —— 步进式三角渲染 + 金属反射 + AO + 漫反射染色
 *  g++ cornell_box_tri_diffuse.cpp -std=c++11 -O2 -o cornell_box_tri_diffuse
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

constexpr float EPS = 1e-5f;
constexpr float INF = 1e30f;

// ---------------------- 基础代数 ----------------------
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
	Vec3 operator/(float s) const {
		return Vec3(x / s, y / s, z / s);
	}
	float dot(const Vec3 &b) const {
		return x * b.x + y * b.y + z * b.z;
	}
	Vec3 cross(const Vec3 &b) const {
		return Vec3(y * b.z - z * b.y,
			z * b.x - x * b.z,
			x * b.y - y * b.x);
	}
	float length() const {
		return std::sqrt(x * x + y * y + z * z);
	}
	Vec3 normalized() const {
		return (*this) * (1.0f / length());
	}
	// 生成局部正交系，把(0,0,1)转到normal方向
	Vec3 rotateToHemisphere(const Vec3 &normal, float u, float v) const {
		Vec3 up = fabs(normal.z) < 0.999f ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
		Vec3 tangent = normal.cross(up).normalized();
		Vec3 bitangent = normal.cross(tangent);
		return tangent * u + bitangent * v + normal * std::sqrt(std::max(0.f, 1 - u * u - v * v));
	}
};

// ---------------------- 颜色 ----------------------
struct Color {
	float r, g, b;
	Color(float rr = 0, float gg = 0, float bb = 0) : r(rr), g(gg), b(bb) {
	}
	Color operator*(float s) const {
		return Color(r * s, g * s, b * s);
	}
	Color operator*(const Color &c) const {
		return Color(r * c.r, g * c.g, b * c.b);
	}
	Color operator+(const Color &c) const {
		return Color(r + c.r, g + c.g, b + c.b);
	}
	void clamp() {
		r = std::fmax(0.0f, std::fmin(1.0f, r));
		g = std::fmax(0.0f, std::fmin(1.0f, g));
		b = std::fmax(0.0f, std::fmin(1.0f, b));
	}
};

// ---------------------- 纹理 ----------------------
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
		int xx = int(std::floor(u * scale)), yy = int(std::floor(v * scale));
		return ((xx + yy) % 2 == 0) ? a : b;
	}
};

enum MaterialType {
	MAT_DIFFUSE,
	MAT_METAL
};

// ---------------------- 三角形 ----------------------
struct Tri {
	Vec3 v0, v1, v2;
	Vec3 n;
	Vec3 uv0, uv1, uv2;
	Texture *tex;
	MaterialType mat;
	float reflect_ratio;
	Color metal_tint;
	Tri(Vec3 a, Vec3 b, Vec3 c, Vec3 t0, Vec3 t1, Vec3 t2, Texture *tx,
		MaterialType m = MAT_DIFFUSE, float reflr = 0.f, Color tint = Color())
		: v0(a), v1(b), v2(c), uv0(t0), uv1(t1), uv2(t2), tex(tx), mat(m), reflect_ratio(reflr), metal_tint(tint) {
		n = (v1 - v0).cross(v2 - v0).normalized();
	}
	bool intersect(const Vec3 &o, const Vec3 &d, float &t, float &u, float &v) const {
		Vec3 e1 = v1 - v0, e2 = v2 - v0, h = d.cross(e2), s = o - v0;
		float a = e1.dot(h);
		if (std::fabs(a) < EPS)
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
	void bary2uv(float b0, float b1, float b2, float &u, float &v) const {
		u = uv0.x * b0 + uv1.x * b1 + uv2.x * b2;
		v = uv0.y * b0 + uv1.y * b1 + uv2.y * b2;
	}
};

// ---------------------- 场景 ----------------------
struct Scene {
	std::vector<Tri> tris;
	std::vector<Texture *> textures;
	~Scene() {
		for (Texture *t : textures)
			delete t;
	}
	Scene() {
		Texture *red = new SolidTexture(Color(0.8, 0.15, 0.15)),
				*green = new SolidTexture(Color(0.15, 0.8, 0.15)),
				*white = new SolidTexture(Color(0.8, 0.8, 0.8)),
				*checker = new CheckerTexture(Color(0.9, 0.9, 0.9), Color(0.1, 0.1, 0.1), 8),
				*meta = new SolidTexture(Color(0.93, 0.95, 1.0));
		textures = { red, green, white, checker, meta };
		pushQuad(-1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, red, MAT_METAL, 0.90f);
		pushQuad(1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, green, MAT_METAL, 0.90f);
		pushQuad(-1, -1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, checker, MAT_METAL, 0.90f);
		pushQuad(-1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, white, MAT_METAL, 0.90f);
		pushQuad(-1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, white, MAT_METAL, 0.90f);
		pushBox(-0.5f, -0.9f, 0.4f, 0.15f, white, MAT_METAL, 0.90f, Color(0.92, 0.94, 1.0));
		pushBox(0.0f, -0.9f, 0.4f, 0.10f, meta, MAT_METAL, 0.90f, Color(0.92, 0.94, 1.0));
	}
	void pushQuad(float x0, float y0, float z0, float x1, float y1, float z1,
		float x2, float y2, float z2, float x3, float y3, float z3,
		Texture *t, MaterialType mat = MAT_DIFFUSE, float rrat = 0.0f, Color tint = Color()) {
		tris.emplace_back(Vec3(x0, y0, z0), Vec3(x1, y1, z1), Vec3(x2, y2, z2),
			Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0), t, mat, rrat, tint);
		tris.emplace_back(Vec3(x0, y0, z0), Vec3(x2, y2, z2), Vec3(x3, y3, z3),
			Vec3(0, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0), t, mat, rrat, tint);
	}
	void pushBox(float cx, float cy, float cz, float r, Texture *t,
		MaterialType mat = MAT_DIFFUSE, float rrat = 0.0f, Color tint = Color()) {
		float X[2] = { cx - r, cx + r }, Y[2] = { cy - r, cy + r }, Z[2] = { cz - r, cz + r };
		pushQuad(X[0], Y[0], Z[0], X[1], Y[0], Z[0], X[1], Y[1], Z[0], X[0], Y[1], Z[0], t, mat, rrat, tint);
		pushQuad(X[0], Y[0], Z[1], X[1], Y[0], Z[1], X[1], Y[1], Z[1], X[0], Y[1], Z[1], t, mat, rrat, tint);
		pushQuad(X[0], Y[0], Z[0], X[0], Y[1], Z[0], X[0], Y[1], Z[1], X[0], Y[0], Z[1], t, mat, rrat, tint);
		pushQuad(X[1], Y[0], Z[0], X[1], Y[1], Z[0], X[1], Y[1], Z[1], X[1], Y[0], Z[1], t, mat, rrat, tint);
		pushQuad(X[0], Y[1], Z[0], X[1], Y[1], Z[0], X[1], Y[1], Z[1], X[0], Y[1], Z[1], t, mat, rrat, tint);
		pushQuad(X[0], Y[0], Z[0], X[1], Y[0], Z[0], X[1], Y[0], Z[1], X[0], Y[0], Z[1], t, mat, rrat, tint);
	}
};

// ---------------------- 视口 ----------------------
struct Viewport {
	int w, h;
	std::vector<Color> pixels;
	Viewport(int W, int H) : w(W), h(H), pixels(W * H) {
	}
	void set(int x, int y, const Color &c) {
		if (x >= 0 && y >= 0 && x < w && y < h)
			pixels[y * w + x] = c;
	}
	Color get(int x, int y) const {
		return pixels[y * w + x];
	}
};

struct HitRec {
	int tri = -1;
	float t = INF, u = 0, v = 0;
};

// ---------------------- 相交查询 ----------------------
HitRec intersect(const Scene &scene, const Vec3 &o, const Vec3 &d) {
	HitRec ret;
	for (int i = 0; i < scene.tris.size(); ++i) {
		float t, u, v;
		if (scene.tris[i].intersect(o, d, t, u, v) && t < ret.t) {
			ret = { i, t, u, v };
		}
	}
	return ret;
}

// ---------------------- AO ----------------------
float computeAO(const Scene &scene, const Vec3 &p, const Vec3 &n, std::mt19937 &rng) {
	const int N = 32;
	std::uniform_real_distribution<float> dist(0, 1);
	int unoccluded = 0;
	for (int i = 0; i < N; ++i) {
		float u = dist(rng), v = dist(rng);
		float theta = 2 * M_PI * u;
		float phi = std::acos(1 - 2 * v);
		float x = std::sin(phi) * std::cos(theta);
		float y = std::sin(phi) * std::sin(theta);
		float z = std::cos(phi);
		if (z < 0)
			z = -z;
		Vec3 hemi(x, y, z);
		Vec3 axis = Vec3(0, 0, 1).cross(n);
		float sa = axis.length(), ca = Vec3(0, 0, 1).dot(n);
		Vec3 d = hemi;
		if (sa > EPS) {
			axis = axis.normalized();
			float ang = std::acos(ca);
			d = d * std::cos(ang) + axis.cross(d) * std::sin(ang) + axis * (axis.dot(d)) * (1 - std::cos(ang));
		}
		d = d.normalized();
		if (intersect(scene, p + n * EPS, d).tri == -1)
			unoccluded++;
	}
	return 0.25f + 0.75f * (unoccluded / float(N));
}

// ---------------------- 路径追踪 ----------------------
Color trace(const Scene &scene, const Vec3 &o, const Vec3 &d,
	std::mt19937 &rng, int depth = 0) {
	HitRec rec = intersect(scene, o, d);
	if (rec.tri == -1)
		return Color(0.06, 0.09, 0.14);

	const Tri &tri = scene.tris[rec.tri];
	Vec3 p = tri.v0 * (1 - rec.u - rec.v) + tri.v1 * rec.u + tri.v2 * rec.v;
	float uu, vv;
	tri.bary2uv(1 - rec.u - rec.v, rec.u, rec.v, uu, vv);
	Color albedo = tri.tex->at(uu, vv);

	// AO
	float ao = computeAO(scene, p, tri.n, rng);

	// 金属反射
	if (tri.mat == MAT_METAL && depth == 0) {
		Vec3 view = (o - p).normalized();
		Vec3 refl = view - tri.n * 2 * view.dot(tri.n);
		Color reflected = trace(scene, p + tri.n * EPS, refl, rng, depth + 1);
		Color ret = albedo * (1 - tri.reflect_ratio) + reflected * tri.reflect_ratio * tri.metal_tint;
		ret = ret * ao;
		ret.clamp();
		return ret;
	}

	// 漫反射
	if (tri.mat == MAT_DIFFUSE) {
		const int samples = 32;
		const int max_bounce = 3;
		Color accum;
		std::uniform_real_distribution<float> dist(0, 1);
		for (int s = 0; s < samples; ++s) {
			// 余弦重要性采样
			float r1 = dist(rng), r2 = dist(rng);
			float phi = 2 * M_PI * r1;
			float r2s = std::sqrt(r2);
			Vec3 local(r2s * std::cos(phi), r2s * std::sin(phi), std::sqrt(1 - r2));
			Vec3 world = Vec3(0, 0, 1).rotateToHemisphere(tri.n, local.x, local.y);
			Vec3 newOrigin = p + tri.n * EPS;
			Color throughput = albedo;
			int b = 0;
			Vec3 dir = world;
			while (b < max_bounce) {
				HitRec bRec = intersect(scene, newOrigin, dir);
				if (bRec.tri == -1)
					break;
				const Tri &bTri = scene.tris[bRec.tri];
				Vec3 bP = bTri.v0 * (1 - bRec.u - bRec.v) + bTri.v1 * bRec.u + bTri.v2 * bRec.v;
				float bu, bv;
				bTri.bary2uv(1 - bRec.u - bRec.v, bRec.u, bRec.v, bu, bv);
				Color bAlbedo = bTri.tex->at(bu, bv);
				throughput = throughput * bAlbedo;
				// Russian Roulette
				float p = std::fmax(throughput.r, std::fmax(throughput.g, throughput.b));
				if (dist(rng) > p)
					break;
				throughput = throughput * (1 / p);
				// 继续
				Vec3 newDir;
				if (bTri.mat == MAT_METAL) {
					Vec3 view = (Vec3() - dir).normalized();
					newDir = view - bTri.n * 2 * view.dot(bTri.n);
				} else {
					// 继续余弦采样
					float nr1 = dist(rng), nr2 = dist(rng);
					float nphi = 2 * M_PI * nr1;
					float nr2s = std::sqrt(nr2);
					Vec3 nlocal(nr2s * std::cos(nphi), nr2s * std::sin(nphi), std::sqrt(1 - nr2));
					newDir = Vec3(0, 0, 1).rotateToHemisphere(bTri.n, nlocal.x, nlocal.y);
				}
				newOrigin = bP + bTri.n * EPS;
				dir = newDir;
				b++;
			}
			accum = accum + throughput;
		}
		albedo = albedo + accum * (1.0f / samples);
	}

	albedo = albedo * ao;
	albedo.clamp();
	return albedo;
}

// ---------------------- 渲染 ----------------------
void render(const Scene &scene, const Vec3 &camPos, const Vec3 &camTarget,
	const Vec3 &camUp, float fov, Viewport &vp) {
	Vec3 forward = (camTarget - camPos).normalized();
	Vec3 right = forward.cross(camUp).normalized();
	Vec3 up = right.cross(forward);
	float aspect = float(vp.w) / vp.h;
	float scale = std::tan(fov * 0.5f * M_PI / 180.f);

	auto t0 = std::chrono::high_resolution_clock::now();
	uint64_t totalRays = 0;

	std::random_device rd;
	std::vector<std::mt19937> rngs(vp.h);
	for (int y = 0; y < vp.h; ++y)
		rngs[y] = std::mt19937(rd() + y);

	for (int y = 0; y < vp.h; ++y) {
		if (y % 16 == 0) {
			auto t1 = std::chrono::high_resolution_clock::now();
			float elapsed = std::chrono::duration<float>(t1 - t0).count();
			float mrays = (totalRays / 1e6f) / (elapsed + 0.001f);
			std::cerr << "\rProgress: " << std::setw(3) << y << "/" << vp.h
					  << " rows  (" << std::fixed << std::setprecision(1)
					  << (100.f * y / vp.h) << "%)  "
					  << mrays << " MRays/s" << std::flush;
		}
		for (int x = 0; x < vp.w; ++x) {
			float fx = (2 * (x + 0.5f) / vp.w - 1) * aspect * scale;
			float fy = (1 - 2 * (y + 0.5f) / vp.h) * scale;
			Vec3 dir = (forward + right * fx + up * fy).normalized();
			Color col = trace(scene, camPos, dir, rngs[y], 0);
			col.clamp();
			vp.set(x, y, col);
			totalRays += 32 + 32; // AO samples + diffuse samples
		}
	}
	std::cerr << "\nRender complete.\n";
}

// ---------------------- 输出 ----------------------
void writePPM(const Viewport &vp, const char *fn) {
	FILE *f = fopen(fn, "wb");
	fprintf(f, "P6\n%d %d\n255\n", vp.w, vp.h);
	for (int y = 0; y < vp.h; ++y)
		for (int x = 0; x < vp.w; ++x) {
			Color c = vp.get(x, y);
			c.clamp();
			unsigned char r = (unsigned char)(std::pow(c.r, 1 / 2.2f) * 255);
			unsigned char g = (unsigned char)(std::pow(c.g, 1 / 2.2f) * 255);
			unsigned char b = (unsigned char)(std::pow(c.b, 1 / 2.2f) * 255);
			fwrite(&r, 1, 1, f);
			fwrite(&g, 1, 1, f);
			fwrite(&b, 1, 1, f);
		}
	fclose(f);
}

// ---------------------- main ----------------------
int main() {
	int w = 512, h = 512;
	Scene scene;
	Viewport vp(w, h);
	render(scene, Vec3(0, 0, 4), Vec3(0, 0, 0), Vec3(0, 1, 0), 50.f, vp);
	writePPM(vp, "out_diffuse.ppm");
	std::cout << "Saved to out_diffuse.ppm\n";
	return 0;
}
