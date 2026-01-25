#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

static constexpr double kEps = 1e-9;

static inline double clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }
static inline int clampInt(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

struct Vec2 {
  double x = 0.0, y = 0.0;

  Vec2() = default;
  Vec2(double xx, double yy) : x(xx), y(yy) {}

  Vec2 operator+(const Vec2& r) const { return Vec2(x + r.x, y + r.y); }
  Vec2 operator-(const Vec2& r) const { return Vec2(x - r.x, y - r.y); }
  Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
  Vec2 operator/(double s) const { return Vec2(x / s, y / s); }
};

static inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

static inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
static inline double cross2(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
static inline double orient2D(const Vec2& a, const Vec2& b, const Vec2& c) { return cross2(b - a, c - a); }

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;

  Vec3() = default;
  Vec3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}

  Vec3 operator+(const Vec3& r) const { return Vec3(x + r.x, y + r.y, z + r.z); }
  Vec3 operator-(const Vec3& r) const { return Vec3(x - r.x, y - r.y, z - r.z); }
  Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
  Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }

  Vec3 operator*(const Vec3& r) const { return Vec3(x * r.x, y * r.y, z * r.z); }
  Vec3 operator/(const Vec3& r) const { return Vec3(x / r.x, y / r.y, z / r.z); }
};

static inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

static inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3(a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}
static inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
static inline Vec3 normalize(const Vec3& v) {
  double len = length(v);
  if (len < kEps) return Vec3(0, 0, 0);
  return v / len;
}
static inline Vec3 clamp01(const Vec3& c) { return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z)); }

struct Triangle2D {
  Vec2 a, b, c;

  double areaAbs() const { return std::abs(orient2D(a, b, c)) * 0.5; }

  Vec2 minXY() const {
    return Vec2(std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}));
  }
  Vec2 maxXY() const {
    return Vec2(std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}));
  }
};

static inline bool barycentric2D(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c, Vec3& wOut) {
  double denom = orient2D(a, b, c);
  if (std::abs(denom) < kEps) return false;
  double w0 = orient2D(p, b, c) / denom;
  double w1 = orient2D(a, p, c) / denom;
  double w2 = 1.0 - w0 - w1;
  wOut = Vec3(w0, w1, w2);
  return true;
}

static inline bool pointInTri2D(const Vec2& p, const Triangle2D& t, double eps = 1e-8) {
  Vec3 w;
  if (!barycentric2D(p, t.a, t.b, t.c, w)) return false;
  return (w.x >= -eps && w.y >= -eps && w.z >= -eps);
}

static inline bool onSegment(const Vec2& a, const Vec2& b, const Vec2& p, double eps = 1e-9) {
  if (std::abs(orient2D(a, b, p)) > eps) return false;
  return (std::min(a.x, b.x) - eps <= p.x && p.x <= std::max(a.x, b.x) + eps &&
          std::min(a.y, b.y) - eps <= p.y && p.y <= std::max(a.y, b.y) + eps);
}

static inline bool segmentsIntersect2D(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2) {
  double o1 = orient2D(p1, p2, q1);
  double o2 = orient2D(p1, p2, q2);
  double o3 = orient2D(q1, q2, p1);
  double o4 = orient2D(q1, q2, p2);

  auto sgn = [](double v) -> int {
    if (v > 1e-12) return 1;
    if (v < -1e-12) return -1;
    return 0;
  };

  int s1 = sgn(o1), s2 = sgn(o2), s3 = sgn(o3), s4 = sgn(o4);

  if (s1 * s2 < 0 && s3 * s4 < 0) return true;

  if (s1 == 0 && onSegment(p1, p2, q1)) return true;
  if (s2 == 0 && onSegment(p1, p2, q2)) return true;
  if (s3 == 0 && onSegment(q1, q2, p1)) return true;
  if (s4 == 0 && onSegment(q1, q2, p2)) return true;

  return false;
}

static inline bool trianglesOverlap2D(const Triangle2D& t1, const Triangle2D& t2) {
  Vec2 min1 = t1.minXY(), max1 = t1.maxXY();
  Vec2 min2 = t2.minXY(), max2 = t2.maxXY();
  if (max1.x < min2.x || max2.x < min1.x || max1.y < min2.y || max2.y < min1.y) return false;

  if (pointInTri2D(t1.a, t2) || pointInTri2D(t1.b, t2) || pointInTri2D(t1.c, t2)) return true;
  if (pointInTri2D(t2.a, t1) || pointInTri2D(t2.b, t1) || pointInTri2D(t2.c, t1)) return true;

  std::array<Vec2, 3> e1a{t1.a, t1.b, t1.c};
  std::array<Vec2, 3> e1b{t1.b, t1.c, t1.a};
  std::array<Vec2, 3> e2a{t2.a, t2.b, t2.c};
  std::array<Vec2, 3> e2b{t2.b, t2.c, t2.a};

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (segmentsIntersect2D(e1a[i], e1b[i], e2a[j], e2b[j])) return true;
    }
  }
  return false;
}

enum class MaterialType { Curtain, Diffuse, Metal };

struct Material {
  MaterialType type = MaterialType::Diffuse;
  Vec3 baseColor = Vec3(1, 1, 1);
  double reflectance = 0.85;
};

struct Triangle {
  int id = -1;
  Vec3 a, b, c;
  Vec2 uvA, uvB, uvC;
  Material mat;

  Vec3 normal() const { return normalize(cross(b - a, c - a)); }
  Vec3 centroid() const { return (a + b + c) / 3.0; }
};

struct Image {
  int w = 0, h = 0;
  std::vector<Vec3> px;

  Image(int width, int height) : w(width), h(height), px((size_t)width * (size_t)height, Vec3(0, 0, 0)) {}

  void clear(const Vec3& c) { std::fill(px.begin(), px.end(), c); }

  Vec3 get(int x, int y) const { return px[(size_t)y * (size_t)w + (size_t)x]; }

  void blendPixel(int x, int y, const Vec3& src, double alpha) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    alpha = clamp01(alpha);
    Vec3& dst = px[(size_t)y * (size_t)w + (size_t)x];
    dst = dst * (1.0 - alpha) + src * alpha;
  }

  void writePPM(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
      std::cerr << "Cannot open output file: " << path << "\n";
      return;
    }
    out << "P6\n" << w << " " << h << "\n255\n";
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        Vec3 c = clamp01(get(x, y));
        auto toSRGB8 = [](double v) -> unsigned char {
          v = clamp01(v);
          v = std::pow(v, 1.0 / 2.2);
          int iv = (int)std::lround(v * 255.0);
          iv = clampInt(iv, 0, 255);
          return (unsigned char)iv;
        };
        unsigned char r = toSRGB8(c.x);
        unsigned char g = toSRGB8(c.y);
        unsigned char b = toSRGB8(c.z);
        out.write((char*)&r, 1);
        out.write((char*)&g, 1);
        out.write((char*)&b, 1);
      }
    }
  }
};

static inline Vec3 reflectPointAcrossPlane(const Vec3& p, const Vec3& planePoint, const Vec3& planeNormalUnit) {
  double d = dot(p - planePoint, planeNormalUnit);
  return p - planeNormalUnit * (2.0 * d);
}

static inline bool projectPointToPlaneFromOrigin(const Vec3& origin,
                                                const Vec3& point,
                                                const Vec3& planePoint,
                                                const Vec3& planeNormalUnit,
                                                Vec3& hit,
                                                double& tOut) {
  Vec3 dir = point - origin;
  double denom = dot(dir, planeNormalUnit);
  if (denom <= 1e-12) return false;
  double t = dot(planePoint - origin, planeNormalUnit) / denom;
  if (t <= 1e-12) return false;
  hit = origin + dir * t;
  tOut = t;
  return true;
}

static inline double wrap01(double u) {
  u = u - std::floor(u);
  if (u < 0) u += 1.0;
  return u;
}

static inline Vec3 sampleCheckerUV(const Vec2& uv, const Vec3& base) {
  double u = wrap01(uv.x);
  double v = wrap01(uv.y);

  const double scale = 10.0;
  int iu = (int)std::floor(u * scale);
  int iv = (int)std::floor(v * scale);

  double checker = ((iu + iv) & 1) ? 0.82 : 1.0;

  double fu = u * scale - std::floor(u * scale);
  double fv = v * scale - std::floor(v * scale);
  double line = (fu < 0.05 || fv < 0.05) ? 0.55 : 1.0;

  return base * (checker * line);
}

struct RenderSettings {
  int maxDepth = 3;
  double minViewportAreaPx = 3.0;
  double minReflectAreaPx = 8.0;
};

struct RenderLayer {
  Vec3 tint = Vec3(1, 1, 1);
  double alpha = 1.0;
};

struct Candidate {
  const Triangle* tri = nullptr;
  Triangle2D projPlane;
  Triangle2D projScreen;
  double depthMetric = 0.0;
};

static inline Vec2 mapPlaneToScreen(const Vec2& pPlane, const Triangle2D& viewportPlaneTri, const Triangle2D& viewportScreenTri) {
  Vec3 w;
  if (!barycentric2D(pPlane, viewportPlaneTri.a, viewportPlaneTri.b, viewportPlaneTri.c, w)) return viewportScreenTri.a;
  return viewportScreenTri.a * w.x + viewportScreenTri.b * w.y + viewportScreenTri.c * w.z;
}

static Vec3 shadeTriangle(const Triangle& tri, const Vec2& uv, const Vec3& viewOrigin) {
  Vec3 n = tri.normal();
  Vec3 lightDir = normalize(Vec3(-0.35, 0.9, -0.25)); // 方向光：从上方略偏前
  double ndotl = std::max(0.0, dot(n, lightDir));

  Vec3 albedo = tri.mat.baseColor;
  if (tri.mat.type == MaterialType::Diffuse) {
    albedo = sampleCheckerUV(uv, albedo);
    double ambient = 0.22;
    double diff = ambient + (1.0 - ambient) * ndotl;
    return albedo * diff;
  }

  if (tri.mat.type == MaterialType::Metal) {
    double ambient = 0.05;
    double diff = ambient + (1.0 - ambient) * std::pow(ndotl, 0.35);
    return albedo * diff;
  }

  return albedo;
}

static void rasterizeTriangleToImage(const Triangle& tri,
                                     const Triangle2D& triScreen,
                                     const Triangle2D& clipScreen,
                                     Image& img,
                                     const Vec3& viewOrigin,
                                     const RenderLayer& layer) {
  Vec2 bbMin = triScreen.minXY();
  Vec2 bbMax = triScreen.maxXY();

  int minX = clampInt((int)std::floor(bbMin.x), 0, img.w - 1);
  int maxX = clampInt((int)std::ceil(bbMax.x), 0, img.w - 1);
  int minY = clampInt((int)std::floor(bbMin.y), 0, img.h - 1);
  int maxY = clampInt((int)std::ceil(bbMax.y), 0, img.h - 1);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      Vec2 p((double)x + 0.5, (double)y + 0.5);

      if (!pointInTri2D(p, clipScreen)) continue;

      Vec3 w;
      if (!barycentric2D(p, triScreen.a, triScreen.b, triScreen.c, w)) continue;
      if (w.x < -1e-8 || w.y < -1e-8 || w.z < -1e-8) continue;

      Vec2 uv = tri.uvA * w.x + tri.uvB * w.y + tri.uvC * w.z;

      Vec3 c = shadeTriangle(tri, uv, viewOrigin);
      c = c * layer.tint;
      img.blendPixel(x, y, c, layer.alpha);
    }
  }
}

static void renderTriangleWithTriangle(const std::vector<Triangle>& scene,
                                      const Vec3& origin,
                                      const Triangle& viewportWorld,
                                      const Triangle2D& viewportScreen,
                                      Image& img,
                                      int depth,
                                      const RenderSettings& settings,
                                      const RenderLayer& layer,
                                      int ignoreTriId) {
  if (depth > settings.maxDepth) return;

  double viewportArea = viewportScreen.areaAbs();
  if (viewportArea < settings.minViewportAreaPx) return;

  Vec3 planePoint = viewportWorld.a;
  Vec3 n = normalize(cross(viewportWorld.b - viewportWorld.a, viewportWorld.c - viewportWorld.a));
  if (length(n) < 1e-12) return;

  if (dot(planePoint - origin, n) < 0.0) n = n * -1.0;

  double planeDist = dot(planePoint - origin, n);
  if (planeDist <= 1e-9) return;

  Vec3 u3 = normalize(viewportWorld.b - viewportWorld.a);
  if (length(u3) < 1e-12) return;
  Vec3 v3 = normalize(cross(n, u3));
  if (length(v3) < 1e-12) return;

  Triangle2D viewportPlane;
  viewportPlane.a = Vec2(0.0, 0.0);
  viewportPlane.b = Vec2(dot(viewportWorld.b - viewportWorld.a, u3), dot(viewportWorld.b - viewportWorld.a, v3));
  viewportPlane.c = Vec2(dot(viewportWorld.c - viewportWorld.a, u3), dot(viewportWorld.c - viewportWorld.a, v3));

  std::vector<Candidate> candidates;
  candidates.reserve(scene.size());

  for (const Triangle& tri : scene) {
    if (tri.id == ignoreTriId) continue;
    if (tri.mat.type == MaterialType::Curtain) continue;

    std::array<Vec3, 3> vtx{tri.a, tri.b, tri.c};
    std::array<Vec2, 3> proj2;
    std::array<double, 3> denom;
    bool ok = true;

    for (int i = 0; i < 3; ++i) {
      Vec3 hit;
      double t;
      if (!projectPointToPlaneFromOrigin(origin, vtx[i], planePoint, n, hit, t)) { ok = false; break; }

      if (t >= 1.0 - 1e-6) { ok = false; break; }

      proj2[i] = Vec2(dot(hit - planePoint, u3), dot(hit - planePoint, v3));
      denom[i] = dot(vtx[i] - origin, n);
      if (denom[i] <= 1e-9) { ok = false; break; }
    }
    if (!ok) continue;

    Triangle2D projPlane{proj2[0], proj2[1], proj2[2]};
    if (!trianglesOverlap2D(projPlane, viewportPlane)) continue;

    Triangle2D projScreen;
    projScreen.a = mapPlaneToScreen(projPlane.a, viewportPlane, viewportScreen);
    projScreen.b = mapPlaneToScreen(projPlane.b, viewportPlane, viewportScreen);
    projScreen.c = mapPlaneToScreen(projPlane.c, viewportPlane, viewportScreen);

    double depthMetric = (denom[0] + denom[1] + denom[2]) / 3.0;

    candidates.push_back(Candidate{&tri, projPlane, projScreen, depthMetric});
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) { return lhs.depthMetric > rhs.depthMetric; });

  bool enableCull = (depth == 0);

  for (const Candidate& cand : candidates) {
    const Triangle& tri = *cand.tri;

    if (enableCull) {
      Vec3 nn = tri.normal();
      Vec3 toOrigin = origin - tri.centroid();
      if (dot(nn, toOrigin) <= 0.0) continue;
    }

    rasterizeTriangleToImage(tri, cand.projScreen, viewportScreen, img, origin, layer);

    if (tri.mat.type == MaterialType::Metal) {
      double triAreaPx = cand.projScreen.areaAbs();
      if (triAreaPx < settings.minReflectAreaPx) continue;
      if (depth >= settings.maxDepth) continue;

      Vec3 mirrorN = tri.normal();
      if (length(mirrorN) < 1e-12) continue;

      Vec3 mirroredOrigin = reflectPointAcrossPlane(origin, tri.a, mirrorN);

      RenderLayer childLayer;
      childLayer.tint = layer.tint * tri.mat.baseColor;
      childLayer.alpha = layer.alpha * clamp01(tri.mat.reflectance);

      if (childLayer.alpha < 1e-4) continue;

      renderTriangleWithTriangle(scene,
                                 mirroredOrigin,
                                 tri,
                                 cand.projScreen,
                                 img,
                                 depth + 1,
                                 settings,
                                 childLayer,
                                 tri.id);
    }
  }
}

static Triangle makeTri(int& idCounter,
                        const Vec3& a, const Vec3& b, const Vec3& c,
                        const Vec2& uvA, const Vec2& uvB, const Vec2& uvC,
                        const Material& mat) {
  Triangle t;
  t.id = idCounter++;
  t.a = a; t.b = b; t.c = c;
  t.uvA = uvA; t.uvB = uvB; t.uvC = uvC;
  t.mat = mat;
  return t;
}

static void addQuad(std::vector<Triangle>& out, int& idCounter,
                    const Vec3& p00, const Vec3& p10, const Vec3& p11, const Vec3& p01,
                    const Material& mat,
                    bool windingForGivenNormal = true) {
  Vec2 uv00(0, 0), uv10(1, 0), uv11(1, 1), uv01(0, 1);

  if (windingForGivenNormal) {
    out.push_back(makeTri(idCounter, p00, p11, p10, uv00, uv11, uv10, mat));
    out.push_back(makeTri(idCounter, p00, p01, p11, uv00, uv01, uv11, mat));
  } else {
    out.push_back(makeTri(idCounter, p00, p10, p11, uv00, uv10, uv11, mat));
    out.push_back(makeTri(idCounter, p00, p11, p01, uv00, uv11, uv01, mat));
  }
}

static void addAxisAlignedBox(std::vector<Triangle>& out, int& idCounter,
                             const Vec3& bmin, const Vec3& bmax,
                             const Material& mat) {
  Vec3 p000(bmin.x, bmin.y, bmin.z);
  Vec3 p100(bmax.x, bmin.y, bmin.z);
  Vec3 p110(bmax.x, bmax.y, bmin.z);
  Vec3 p010(bmin.x, bmax.y, bmin.z);

  Vec3 p001(bmin.x, bmin.y, bmax.z);
  Vec3 p101(bmax.x, bmin.y, bmax.z);
  Vec3 p111(bmax.x, bmax.y, bmax.z);
  Vec3 p011(bmin.x, bmax.y, bmax.z);

  addQuad(out, idCounter, p001, p101, p111, p011, mat, true);   // front  +z
  addQuad(out, idCounter, p100, p000, p010, p110, mat, true);   // back   -z
  addQuad(out, idCounter, p000, p001, p011, p010, mat, true);   // left   -x
  addQuad(out, idCounter, p101, p100, p110, p111, mat, true);   // right  +x
  addQuad(out, idCounter, p010, p011, p111, p110, mat, true);   // top    +y
  addQuad(out, idCounter, p000, p100, p101, p001, mat, true);   // bottom -y
}

struct Camera {
  Vec3 origin;
  Triangle viewportWorld0;
  Triangle viewportWorld1;
  Triangle2D viewportScreen0;
  Triangle2D viewportScreen1;

  void render(const std::vector<Triangle>& scene, Image& img, const RenderSettings& settings) const {
    RenderLayer rootLayer;
    rootLayer.tint = Vec3(1, 1, 1);
    rootLayer.alpha = 1.0;

    renderTriangleWithTriangle(scene, origin, viewportWorld0, viewportScreen0, img, 0, settings, rootLayer, -1);
    renderTriangleWithTriangle(scene, origin, viewportWorld1, viewportScreen1, img, 0, settings, rootLayer, -1);
  }
};

int main() {
  const int W = 800;
  const int H = 600;

  Image img(W, H);
  img.clear(Vec3(0.02, 0.02, 0.025));

  RenderSettings settings;
  settings.maxDepth = 3;
  settings.minViewportAreaPx = 2.0;
  settings.minReflectAreaPx = 14.0;

  std::vector<Triangle> scene;
  scene.reserve(128);
  int idCounter = 0;

  Material redWall;    redWall.type = MaterialType::Diffuse; redWall.baseColor = Vec3(0.78, 0.16, 0.16);
  Material greenWall;  greenWall.type = MaterialType::Diffuse; greenWall.baseColor = Vec3(0.16, 0.78, 0.18);
  Material whiteWall;  whiteWall.type = MaterialType::Diffuse; whiteWall.baseColor = Vec3(0.82, 0.82, 0.82);

  Material metalBlue;   metalBlue.type = MaterialType::Metal; metalBlue.baseColor = Vec3(0.12, 0.35, 1.00); metalBlue.reflectance = 0.85;
  Material metalYellow; metalYellow.type = MaterialType::Metal; metalYellow.baseColor = Vec3(1.00, 0.86, 0.18); metalYellow.reflectance = 0.85;

  // Cornell Box: x [-1,1], y [0,2], z [0,2]
  // floor (y=0), normal +y
  addQuad(scene, idCounter,
          Vec3(-1, 0, 0), Vec3( 1, 0, 0), Vec3( 1, 0, 2), Vec3(-1, 0, 2),
          whiteWall, true);

  // ceiling (y=2), normal -y：用反 winding
  addQuad(scene, idCounter,
          Vec3(-1, 2, 2), Vec3( 1, 2, 2), Vec3( 1, 2, 0), Vec3(-1, 2, 0),
          whiteWall, true);

  // back wall (z=2), normal -z
  addQuad(scene, idCounter,
          Vec3(-1, 0, 2), Vec3( 1, 0, 2), Vec3( 1, 2, 2), Vec3(-1, 2, 2),
          whiteWall, false);

  // left wall (x=-1), normal +x
  addQuad(scene, idCounter,
          Vec3(-1, 0, 0), Vec3(-1, 0, 2), Vec3(-1, 2, 2), Vec3(-1, 2, 0),
          redWall, true);

  // right wall (x=1), normal -x
  addQuad(scene, idCounter,
          Vec3( 1, 0, 2), Vec3( 1, 0, 0), Vec3( 1, 2, 0), Vec3( 1, 2, 2),
          greenWall, true);

  // Two metallic boxes (axis-aligned for simplicity)
  addAxisAlignedBox(scene, idCounter, Vec3(-0.65, 0.0, 0.55), Vec3(-0.10, 0.70, 1.10), metalBlue);
  addAxisAlignedBox(scene, idCounter, Vec3( 0.10, 0.0, 1.05), Vec3( 0.70, 0.95, 1.70), metalYellow);

  // Camera + viewport (two triangles)
  Camera cam;
  cam.origin = Vec3(0.0, 1.0, -3.0);

  Material curtain; curtain.type = MaterialType::Curtain; curtain.baseColor = Vec3(0, 0, 0);

  Vec3 bl(-1.0, 0.0, -1.0);
  Vec3 br( 1.0, 0.0, -1.0);
  Vec3 tr( 1.0, 2.0, -1.0);
  Vec3 tl(-1.0, 2.0, -1.0);

  {
    Triangle t0;
    t0.id = -100;
    t0.a = bl; t0.b = br; t0.c = tl;
    t0.uvA = Vec2(0, 1); t0.uvB = Vec2(1, 1); t0.uvC = Vec2(0, 0);
    t0.mat = curtain;
    cam.viewportWorld0 = t0;

    Triangle t1;
    t1.id = -101;
    t1.a = br; t1.b = tr; t1.c = tl;
    t1.uvA = Vec2(1, 1); t1.uvB = Vec2(1, 0); t1.uvC = Vec2(0, 0);
    t1.mat = curtain;
    cam.viewportWorld1 = t1;
  }

  // Screen triangles cover whole image rectangle
  Vec2 sbl(0.0, (double)H - 1.0);
  Vec2 sbr((double)W - 1.0, (double)H - 1.0);
  Vec2 str((double)W - 1.0, 0.0);
  Vec2 stl(0.0, 0.0);

  cam.viewportScreen0 = Triangle2D{sbl, sbr, stl};
  cam.viewportScreen1 = Triangle2D{sbr, str, stl};

  cam.render(scene, img, settings);

  img.writePPM("output.ppm");
  std::cerr << "Done. Wrote output.ppm\n";
  return 0;
}
