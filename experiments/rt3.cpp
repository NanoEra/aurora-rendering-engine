// main.cpp
// 一个“面片 + 纹理变换 + 递归反射贴图”的简易 Cornell Box 渲染器（PPM 输出）
// 关键点：
// - 不从 Camera 对每个像素发射 Ray 求交取色（避免传统 Ray Tracing 思路）
// - 以“Viewport(屏幕/面片) = 场景中的三角形面片集合”的方式做投影与贴图（类似光栅化/投影贴图）
// - 对镜面材质：把该三角形当作 current viewport，构造“反射后的虚拟观察点”，递归渲染并把结果作为该三角形的 current 纹理
//
// 编译运行：
//   g++ -std=c++17 -O2 -o render main.cpp
//   ./render
// 输出：output.ppm (P6)
//
// 说明：
// - 这里先忽略 AO、采样漫反射等；漫反射仅用固有颜色/棋盘纹理。
// - 覆盖关系：用“远到近”排序 + painter 覆盖（O(n log n)），符合你描述的临时方案。
// - 反射：对每个镜面三角形，递归渲染“该三角形所在平面作为 viewport”的纹理图，再贴回上一级 viewport。

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

static constexpr double kEps = 1e-9;

struct Vec2 {
  double x = 0, y = 0;
  Vec2() = default;
  Vec2(double _x, double _y) : x(_x), y(_y) {}
  Vec2 operator+(const Vec2& r) const { return {x + r.x, y + r.y}; }
  Vec2 operator-(const Vec2& r) const { return {x - r.x, y - r.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
};

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
  Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
  Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
};

static inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
static inline Vec3 normalize(const Vec3& v) {
  double len = length(v);
  if (len < kEps) return {0, 0, 0};
  return v / len;
}
static inline Vec3 clamp01(const Vec3& c) {
  auto cl = [](double v) { return std::max(0.0, std::min(1.0, v)); };
  return {cl(c.x), cl(c.y), cl(c.z)};
}
static inline Vec3 hadamard(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }

struct Plane {
  Vec3 n;   // normalized
  double d; // plane eq: dot(n, p) + d = 0
};

static inline Plane makePlaneFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c) {
  Vec3 n = normalize(cross(b - a, c - a));
  double d = -dot(n, a);
  return {n, d};
}

static inline double signedDistance(const Plane& pl, const Vec3& p) { return dot(pl.n, p) + pl.d; }

static inline Vec3 reflectPointAcrossPlane(const Vec3& p, const Plane& pl) {
  // p' = p - 2 * dist * n
  double dist = signedDistance(pl, p);
  return p - pl.n * (2.0 * dist);
}

static inline Vec3 reflectVectorAcrossPlane(const Vec3& v, const Plane& pl) {
  // reflect direction v about plane normal: v' = v - 2*dot(v,n)*n
  return v - pl.n * (2.0 * dot(v, pl.n));
}

struct Image {
  int w = 0, h = 0;
  std::vector<Vec3> pix; // linear RGB [0..1]
  Image() = default;
  Image(int _w, int _h, Vec3 clear = {0, 0, 0}) : w(_w), h(_h), pix(size_t(_w) * size_t(_h), clear) {}
  Vec3& at(int x, int y) { return pix[size_t(y) * size_t(w) + size_t(x)]; }
  const Vec3& at(int x, int y) const { return pix[size_t(y) * size_t(w) + size_t(x)]; }
};

static inline uint8_t toSRGB8(double linear) {
  linear = std::max(0.0, std::min(1.0, linear));
  // 简单 gamma 2.2
  double srgb = std::pow(linear, 1.0 / 2.2);
  int v = int(srgb * 255.0 + 0.5);
  v = std::max(0, std::min(255, v));
  return uint8_t(v);
}

static void writePPM_P6(const std::string& path, const Image& img) {
  std::ofstream out(path, std::ios::binary);
  out << "P6\n" << img.w << " " << img.h << "\n255\n";
  for (int y = 0; y < img.h; ++y) {
    for (int x = 0; x < img.w; ++x) {
      Vec3 c = clamp01(img.at(x, y));
      uint8_t r = toSRGB8(c.x);
      uint8_t g = toSRGB8(c.y);
      uint8_t b = toSRGB8(c.z);
      out.write(reinterpret_cast<const char*>(&r), 1);
      out.write(reinterpret_cast<const char*>(&g), 1);
      out.write(reinterpret_cast<const char*>(&b), 1);
    }
  }
}

static inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a * (1.0 - t) + b * t; }

struct Material {
  enum class Type { Diffuse, Mirror } type = Type::Diffuse;
  Vec3 baseColor = {1, 1, 1}; // 对 Diffuse：固有色；对 Mirror：金属 tint
  double mirrorStrength = 0.95; // Mirror 时反射占比
  bool checker = false;
};

struct Triangle {
  int id = -1;
  Vec3 p0, p1, p2; // world
  Vec2 uv0, uv1, uv2; // local texture coords
  Material mat;
};

static inline Vec3 triangleNormal(const Triangle& t) {
  return normalize(cross(t.p1 - t.p0, t.p2 - t.p0));
}

static inline Vec3 triangleCentroid(const Triangle& t) {
  return (t.p0 + t.p1 + t.p2) / 3.0;
}

// 2D barycentric in screen space
static inline bool barycentric2D(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p,
                                 double& w0, double& w1, double& w2) {
  Vec2 v0 = b - a;
  Vec2 v1 = c - a;
  Vec2 v2 = p - a;
  double den = v0.x * v1.y - v0.y * v1.x;
  if (std::abs(den) < kEps) return false;
  double inv = 1.0 / den;
  w1 = (v2.x * v1.y - v2.y * v1.x) * inv;
  w2 = (v0.x * v2.y - v0.y * v2.x) * inv;
  w0 = 1.0 - w1 - w2;
  return true;
}

// 3D barycentric on triangle plane (using areas / cross with normal)
static inline bool barycentric3D(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p,
                                 double& w0, double& w1, double& w2) {
  Vec3 n = cross(b - a, c - a);
  double area2 = length(n);
  if (area2 < kEps) return false;
  // use oriented areas along normal
  double areaPBC = dot(cross(b - p, c - p), n);
  double areaPCA = dot(cross(c - p, a - p), n);
  double areaPAB = dot(cross(a - p, b - p), n);
  w0 = areaPBC / dot(n, n);
  w1 = areaPCA / dot(n, n);
  w2 = areaPAB / dot(n, n);
  // 这里 w0,w1,w2 的数值尺度已经合理（因为 dot(n,n)=|n|^2）
  // 但数值误差会略大，后续用范围判断即可。
  return true;
}

struct Viewport {
  enum class Kind { Rect, TriangleUV } kind = Kind::Rect;

  Vec3 origin;      // “第一人称”视点（Camera 或 反射后的虚拟点）
  Plane plane;      // viewport 平面
  Vec3 planeCenter; // 用于排序/参考
  Vec3 viewDir;     // normalize(planeCenter - origin)

  // 输出图像大小
  int outW = 0;
  int outH = 0;

  // Rect: 通过 plane 坐标系映射到像素
  Vec3 right; // in plane
  Vec3 up;    // in plane
  double halfW = 1;
  double halfH = 1;

  // TriangleUV: 通过 world 三角形的 UV 映射到像素
  Vec3 tv0, tv1, tv2; // viewport triangle in world (coplanar with plane)
  Vec2 tuv0, tuv1, tuv2;

  static Viewport makeCameraRect(const Vec3& origin, const Vec3& center, const Vec3& right, const Vec3& up,
                                 double halfW, double halfH, int outW, int outH) {
    Viewport vp;
    vp.kind = Kind::Rect;
    vp.origin = origin;
    vp.planeCenter = center;
    vp.right = normalize(right);
    vp.up = normalize(up);
    vp.halfW = halfW;
    vp.halfH = halfH;
    Vec3 n = normalize(cross(vp.right, vp.up)); // right x up = normal
    // 让法线朝向 origin（不强制，但有助于一致性）
    if (dot(n, origin - center) < 0) n = n * -1.0;
    vp.plane = {n, -dot(n, center)};
    vp.viewDir = normalize(center - origin);
    vp.outW = outW;
    vp.outH = outH;
    return vp;
  }

  static Viewport makeTriangleUV(const Vec3& origin, const Triangle& tri, int outW, int outH) {
    Viewport vp;
    vp.kind = Kind::TriangleUV;
    vp.origin = origin;
    vp.tv0 = tri.p0; vp.tv1 = tri.p1; vp.tv2 = tri.p2;
    vp.tuv0 = tri.uv0; vp.tuv1 = tri.uv1; vp.tuv2 = tri.uv2;
    vp.planeCenter = triangleCentroid(tri);
    vp.plane = makePlaneFromTriangle(tri.p0, tri.p1, tri.p2);
    // 让平面法线尽量朝向 origin，便于稳定
    if (dot(vp.plane.n, origin - vp.planeCenter) < 0) {
      vp.plane.n = vp.plane.n * -1.0;
      vp.plane.d = -vp.plane.d;
    }
    vp.viewDir = normalize(vp.planeCenter - origin);
    vp.outW = outW;
    vp.outH = outH;
    return vp;
  }

  // 将世界点 x（不一定在 plane 上）沿 origin->x 投影到 viewport 平面，并映射到像素坐标
  // 返回 false 表示无交/在背面/落在 viewport 有效区域外
  bool projectWorldPointToPixel(const Vec3& x, Vec2& outPix, double& outT) const {
    Vec3 dir = x - origin;
    double denom = dot(plane.n, dir);
    if (std::abs(denom) < kEps) return false;
    // intersection t where origin + t*dir hits plane
    double t = -(dot(plane.n, origin) + plane.d) / denom;
    if (t <= kEps) return false; // 只接受向前交
    Vec3 q = origin + dir * t;   // on plane
    outT = t;

    if (kind == Kind::Rect) {
      Vec3 rel = q - planeCenter;
      double u = dot(rel, right) / halfW; // [-1,1] ideally
      double v = dot(rel, up) / halfH;
      double px = (u * 0.5 + 0.5) * double(outW - 1);
      double py = (1.0 - (v * 0.5 + 0.5)) * double(outH - 1);
      if (px < -0.5 || px > outW - 0.5 || py < -0.5 || py > outH - 0.5) return false;
      outPix = {px, py};
      return true;
    }

    // TriangleUV: 要求 q 落在 viewport 三角形内（用 3D 重心判断）
    double w0, w1, w2;
    if (!barycentric3D(tv0, tv1, tv2, q, w0, w1, w2)) return false;
    if (w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4) return false;

    Vec2 uv = tuv0 * w0 + tuv1 * w1 + tuv2 * w2;
    if (uv.x < -1e-4 || uv.x > 1.0 + 1e-4 || uv.y < -1e-4 || uv.y > 1.0 + 1e-4) return false;

    double px = uv.x * double(outW - 1);
    double py = (1.0 - uv.y) * double(outH - 1);
    outPix = {px, py};
    return true;
  }
};

static Vec3 sampleDiffuseTex(const Material& mat, const Vec2& uv) {
  if (!mat.checker) return mat.baseColor;
  // 纯 UV 棋盘
  int cx = int(std::floor(uv.x * 10.0));
  int cy = int(std::floor(uv.y * 10.0));
  bool odd = ((cx + cy) & 1) != 0;
  Vec3 c0 = mat.baseColor;
  Vec3 c1 = lerp(mat.baseColor, Vec3{1, 1, 1}, 0.35);
  return odd ? c1 : c0;
}

// Forward declaration
static Image renderViewport(const Viewport& vp, const std::vector<Triangle>& scene, int depth, int maxDepth,
                            int excludeTriId);

// 为镜面三角形生成“当前纹理”（通过递归把它当 viewport 渲染）
static Image renderMirrorTextureForTriangle(const Triangle& tri, const Vec3& incidentOrigin,
                                           const std::vector<Triangle>& scene, int depth, int maxDepth,
                                           int texW, int texH) {
  Plane pl = makePlaneFromTriangle(tri.p0, tri.p1, tri.p2);

  // 虚拟观察点：把上一级 origin 关于镜面平面反射
  Vec3 virtualOrigin = reflectPointAcrossPlane(incidentOrigin, pl);

  // 以该镜面三角形为 viewport，把场景渲染到它的 UV 纹理图里
  Viewport triVP = Viewport::makeTriangleUV(virtualOrigin, tri, texW, texH);

  // 排除自己，避免最直接的反馈
  Image reflection = renderViewport(triVP, scene, depth + 1, maxDepth, tri.id);

  return reflection;
}

struct ProjectedTri {
  const Triangle* tri = nullptr;
  Vec2 s0, s1, s2; // projected to output pixels
  double sortKey = 0; // larger=nearer or farther? We will sort far->near
};

// 简易 painter：远->近（sortKey 越大越“近”），所以按 sortKey 升序绘制（远先画）
static Image renderViewport(const Viewport& vp, const std::vector<Triangle>& scene, int depth, int maxDepth,
                            int excludeTriId) {
  Image out(vp.outW, vp.outH, Vec3{0.0, 0.0, 0.0});

  // 为本次渲染维护一个“镜面三角形 -> 纹理图”的临时缓存（只对当前 viewport 有效）
  struct MirrorCacheItem { int triId; Image tex; };
  std::vector<MirrorCacheItem> mirrorTexCache;
  mirrorTexCache.reserve(64);

  auto getMirrorTex = [&](const Triangle& t) -> const Image* {
    for (auto& it : mirrorTexCache) {
      if (it.triId == t.id) return &it.tex;
    }
    // 纹理分辨率可根据需要调大/调小
    int texW = 256, texH = 256;
    if (depth >= maxDepth) return nullptr;
    Image tex = renderMirrorTextureForTriangle(t, vp.origin, scene, depth, maxDepth, texW, texH);
    mirrorTexCache.push_back({t.id, std::move(tex)});
    return &mirrorTexCache.back().tex;
  };

  // 1) 收集可投影到 viewport 的三角形，并计算排序 key
  std::vector<ProjectedTri> list;
  list.reserve(scene.size());

  for (const auto& t : scene) {
    if (t.id == excludeTriId) continue;

    Vec2 p0, p1, p2;
    double tt0, tt1, tt2;
    if (!vp.projectWorldPointToPixel(t.p0, p0, tt0)) continue;
    if (!vp.projectWorldPointToPixel(t.p1, p1, tt1)) continue;
    if (!vp.projectWorldPointToPixel(t.p2, p2, tt2)) continue;

    Vec3 c = triangleCentroid(t);
    double key = dot(vp.viewDir, c - vp.origin); // 视线方向上的“深度”
    list.push_back(ProjectedTri{&t, p0, p1, p2, key});
  }

  std::sort(list.begin(), list.end(), [](const ProjectedTri& a, const ProjectedTri& b) {
    return a.sortKey < b.sortKey; // far -> near
  });

  // 2) 按远到近绘制（覆盖关系）
  for (const auto& pt : list) {
    const Triangle& tri = *pt.tri;

    // 镜面材质：先生成该三角形对当前 origin 的“反射纹理”
    const Image* mirrorTex = nullptr;
    if (tri.mat.type == Material::Type::Mirror && depth < maxDepth) {
      mirrorTex = getMirrorTex(tri);
    }

    // 三角形光栅化到 out（注意：这里是把 tri 的“纹理”投影变换后贴到 viewport 上）
    double minX = std::floor(std::min({pt.s0.x, pt.s1.x, pt.s2.x}));
    double maxX = std::ceil (std::max({pt.s0.x, pt.s1.x, pt.s2.x}));
    double minY = std::floor(std::min({pt.s0.y, pt.s1.y, pt.s2.y}));
    double maxY = std::ceil (std::max({pt.s0.y, pt.s1.y, pt.s2.y}));

    int x0 = std::max(0, int(minX));
    int x1 = std::min(vp.outW - 1, int(maxX));
    int y0 = std::max(0, int(minY));
    int y1 = std::min(vp.outH - 1, int(maxY));

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        Vec2 p{double(x) + 0.5, double(y) + 0.5};
        double w0, w1, w2;
        if (!barycentric2D(pt.s0, pt.s1, pt.s2, p, w0, w1, w2)) continue;
        if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;

        // screen-space 仿射插值 tri 的 UV（这里不做 perspective-correct，符合“变换粘贴”的感觉）
        Vec2 uv = tri.uv0 * w0 + tri.uv1 * w1 + tri.uv2 * w2;

        Vec3 base = sampleDiffuseTex(tri.mat, uv);

        if (tri.mat.type == Material::Type::Mirror && mirrorTex) {
          // 镜面：把递归得到的 reflectionTex 作为 tri 的 current 纹理，再叠加 tint
          // mirrorTex 的坐标域就是 tri 的 UV（由 Viewport::TriangleUV 保证）
          int sx = std::max(0, std::min(mirrorTex->w - 1, int(uv.x * (mirrorTex->w - 1) + 0.5)));
          int sy = std::max(0, std::min(mirrorTex->h - 1, int((1.0 - uv.y) * (mirrorTex->h - 1) + 0.5)));
          Vec3 refl = mirrorTex->at(sx, sy);

          // 简单金属混合：反射为主，baseColor 做 tint
          Vec3 tintedRefl = hadamard(refl, tri.mat.baseColor);
          Vec3 outC = lerp(base, tintedRefl, tri.mat.mirrorStrength);

          out.at(x, y) = outC;
        } else {
          // 漫反射（当前回合忽略采样/光照，仅贴固有纹理）
          out.at(x, y) = base;
        }
      }
    }
  }

  return out;
}

static int gNextTriId = 1;

static Triangle makeTri(const Vec3& a, const Vec3& b, const Vec3& c, const Vec2& uva, const Vec2& uvb, const Vec2& uvc,
                        const Material& m) {
  Triangle t;
  t.id = gNextTriId++;
  t.p0 = a; t.p1 = b; t.p2 = c;
  t.uv0 = uva; t.uv1 = uvb; t.uv2 = uvc;
  t.mat = m;
  return t;
}

static void addQuadAsTwoTris(std::vector<Triangle>& out,
                             const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d,
                             const Material& m,
                             const Vec2& uva = {0,0}, const Vec2& uvb = {1,0}, const Vec2& uvc = {1,1}, const Vec2& uvd = {0,1}) {
  // quad (a,b,c,d) in CCW; split to (a,b,c) and (a,c,d)
  out.push_back(makeTri(a, b, c, uva, uvb, uvc, m));
  out.push_back(makeTri(a, c, d, uva, uvc, uvd, m));
}

static void addBox(std::vector<Triangle>& out, const Vec3& center, const Vec3& halfSize, const Material& m) {
  // axis-aligned box, 6 faces, each 2 tris
  Vec3 minP = center - halfSize;
  Vec3 maxP = center + halfSize;

  Vec3 p000{minP.x, minP.y, minP.z};
  Vec3 p001{minP.x, minP.y, maxP.z};
  Vec3 p010{minP.x, maxP.y, minP.z};
  Vec3 p011{minP.x, maxP.y, maxP.z};
  Vec3 p100{maxP.x, minP.y, minP.z};
  Vec3 p101{maxP.x, minP.y, maxP.z};
  Vec3 p110{maxP.x, maxP.y, minP.z};
  Vec3 p111{maxP.x, maxP.y, maxP.z};

  // +X (right): p100 p101 p111 p110
  addQuadAsTwoTris(out, p100, p101, p111, p110, m);
  // -X (left): p000 p010 p011 p001
  addQuadAsTwoTris(out, p000, p010, p011, p001, m);
  // +Y (top): p010 p110 p111 p011
  addQuadAsTwoTris(out, p010, p110, p111, p011, m);
  // -Y (bottom): p000 p001 p101 p100
  addQuadAsTwoTris(out, p000, p001, p101, p100, m);
  // +Z (front): p001 p011 p111 p101
  addQuadAsTwoTris(out, p001, p011, p111, p101, m);
  // -Z (back): p000 p100 p110 p010
  addQuadAsTwoTris(out, p000, p100, p110, p010, m);
}

int main() {
  // 输出分辨率
  const int W = 800;
  const int H = 600;

  // 构造 Cornell Box 场景（用三角形面片）
  std::vector<Triangle> scene;
  scene.reserve(1024);

  // Cornell box bounds
  // x: [-1,1], y: [-1,1], z: [-2,-6]
  const double x0 = -1.0, x1 = 1.0;
  const double y0 = -1.0, y1 = 1.0;
  const double zNear = -2.0, zBack = -6.0;

  Material leftWall;  leftWall.type = Material::Type::Diffuse; leftWall.baseColor = {0.95, 0.20, 0.20}; leftWall.checker = false;
  Material rightWall; rightWall.type = Material::Type::Diffuse; rightWall.baseColor = {0.20, 0.95, 0.20}; rightWall.checker = false;
  Material backWall;  backWall.type = Material::Type::Diffuse; backWall.baseColor = {0.90, 0.90, 0.90}; backWall.checker = false;
  Material floorWall; floorWall.type = Material::Type::Diffuse; floorWall.baseColor = {0.90, 0.90, 0.90}; floorWall.checker = true;
  Material ceilWall;  ceilWall.type = Material::Type::Diffuse; ceilWall.baseColor = {0.85, 0.85, 0.85}; ceilWall.checker = false;

  // 四壁（这里额外加了天花板，便于反射更丰富；如需严格“四壁”，可注释 ceiling）
  // Left wall (x=x0), CCW from inside
  addQuadAsTwoTris(scene,
    Vec3{x0, y0, zNear}, Vec3{x0, y0, zBack}, Vec3{x0, y1, zBack}, Vec3{x0, y1, zNear},
    leftWall);

  // Right wall (x=x1)
  addQuadAsTwoTris(scene,
    Vec3{x1, y0, zBack}, Vec3{x1, y0, zNear}, Vec3{x1, y1, zNear}, Vec3{x1, y1, zBack},
    rightWall);

  // Back wall (z=zBack)
  addQuadAsTwoTris(scene,
    Vec3{x0, y0, zBack}, Vec3{x1, y0, zBack}, Vec3{x1, y1, zBack}, Vec3{x0, y1, zBack},
    backWall);

  // Floor (y=y0)
  addQuadAsTwoTris(scene,
    Vec3{x0, y0, zBack}, Vec3{x1, y0, zBack}, Vec3{x1, y0, zNear}, Vec3{x0, y0, zNear},
    floorWall);

  // Ceiling (y=y1)
  addQuadAsTwoTris(scene,
    Vec3{x0, y1, zNear}, Vec3{x1, y1, zNear}, Vec3{x1, y1, zBack}, Vec3{x0, y1, zBack},
    ceilWall);

  // 两个金属盒子（镜面反射）
  Material blueMetal;
  blueMetal.type = Material::Type::Mirror;
  blueMetal.baseColor = {0.35, 0.55, 1.00}; // 蓝色金属 tint
  blueMetal.mirrorStrength = 0.95;

  Material yellowMetal;
  yellowMetal.type = Material::Type::Mirror;
  yellowMetal.baseColor = {1.00, 0.90, 0.25}; // 黄色金属 tint
  yellowMetal.mirrorStrength = 0.95;

  // 放在地面上（y0=-1），盒子底面稍微抬一点避免穿插
  addBox(scene, Vec3{-0.45, -0.55, -4.7}, Vec3{0.25, 0.45, 0.25}, blueMetal);
  addBox(scene, Vec3{ 0.45, -0.65, -3.8}, Vec3{0.30, 0.35, 0.30}, yellowMetal);

  // Camera viewport 抽象为场景中的一个矩形平面（内部用两个三角形覆盖，这里用 Rect viewport 简化映射）
  Vec3 camOrigin{0.0, 0.0, 0.0};
  Vec3 vpCenter{0.0, 0.0, -1.0};

  // 视场决定 viewport 大小：halfW/halfH
  // 设定垂直 FOV ~ 55°
  double fovY = 55.0 * M_PI / 180.0;
  double halfH = std::tan(fovY * 0.5) * 1.0;          // plane 距离为 1
  double halfW = halfH * (double(W) / double(H));

  Vec3 vpRight{1, 0, 0};
  Vec3 vpUp{0, 1, 0};

  Viewport cameraVP = Viewport::makeCameraRect(camOrigin, vpCenter, vpRight, vpUp, halfW, halfH, W, H);

  // 递归深度（镜面反射级数）
  const int maxDepth = 2;

  std::cerr << "Rendering...\n";
  Image img = renderViewport(cameraVP, scene, /*depth=*/0, maxDepth, /*excludeTriId=*/-1);

  // 输出
  writePPM_P6("output.ppm", img);
  std::cerr << "Done. Wrote output.ppm\n";
  return 0;
}
