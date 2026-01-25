// main.cpp
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

struct Vec2 {
  double x = 0, y = 0;
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
};

static inline double cross2(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
};

static inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline double norm2(const Vec3& v) { return dot(v, v); }
static inline double norm(const Vec3& v) { return std::sqrt(norm2(v)); }

static inline Vec3 normalize(const Vec3& v) {
  const double n = norm(v);
  if (n < kEps) return v;
  return v / n;
}

struct Color {
  double r = 0, g = 0, b = 0;
  Color() = default;
  Color(double r_, double g_, double b_) : r(r_), g(g_), b(b_) {}
  Color operator+(const Color& o) const { return {r + o.r, g + o.g, b + o.b}; }
  Color operator*(double s) const { return {r * s, g * s, b * s}; }
  Color operator*(const Color& o) const { return {r * o.r, g * o.g, b * o.b}; }
};

static inline Color clamp01(const Color& c) {
  auto cl = [](double v) { return std::max(0.0, std::min(1.0, v)); };
  return {cl(c.r), cl(c.g), cl(c.b)};
}

struct Plane {
  Vec3 n; // normalized
  double d = 0; // n·X + d = 0

  double sd(const Vec3& p) const { return dot(n, p) + d; }
};

static inline Plane planeFromPointNormal(const Vec3& p, const Vec3& n_unit) {
  Plane pl;
  pl.n = n_unit;
  pl.d = -dot(pl.n, p);
  return pl;
}

static inline Plane planeThrough3(const Vec3& a, const Vec3& b, const Vec3& c) {
  Vec3 n = normalize(cross(b - a, c - a));
  return planeFromPointNormal(a, n);
}

struct Mat3 {
  double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  static Mat3 identity() {
    Mat3 I;
    I.m[0][0] = I.m[1][1] = I.m[2][2] = 1;
    return I;
  }
};

static inline Mat3 mul(const Mat3& A, const Mat3& B) {
  Mat3 C;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0;
      for (int k = 0; k < 3; ++k) s += A.m[i][k] * B.m[k][j];
      C.m[i][j] = s;
    }
  }
  return C;
}

static inline Vec3 mul_h(const Mat3& H, const Vec3& p) {
  Vec3 r;
  r.x = H.m[0][0] * p.x + H.m[0][1] * p.y + H.m[0][2] * p.z;
  r.y = H.m[1][0] * p.x + H.m[1][1] * p.y + H.m[1][2] * p.z;
  r.z = H.m[2][0] * p.x + H.m[2][1] * p.y + H.m[2][2] * p.z;
  return r;
}

static inline bool invertMat3(const Mat3& A, Mat3& invOut) {
  const double a00 = A.m[0][0], a01 = A.m[0][1], a02 = A.m[0][2];
  const double a10 = A.m[1][0], a11 = A.m[1][1], a12 = A.m[1][2];
  const double a20 = A.m[2][0], a21 = A.m[2][1], a22 = A.m[2][2];

  const double c00 = a11 * a22 - a12 * a21;
  const double c01 = -(a10 * a22 - a12 * a20);
  const double c02 = a10 * a21 - a11 * a20;

  const double c10 = -(a01 * a22 - a02 * a21);
  const double c11 = a00 * a22 - a02 * a20;
  const double c12 = -(a00 * a21 - a01 * a20);

  const double c20 = a01 * a12 - a02 * a11;
  const double c21 = -(a00 * a12 - a02 * a10);
  const double c22 = a00 * a11 - a01 * a10;

  const double det = a00 * c00 + a01 * c01 + a02 * c02;
  if (std::abs(det) < 1e-18) return false;
  const double invDet = 1.0 / det;

  invOut.m[0][0] = c00 * invDet;
  invOut.m[0][1] = c10 * invDet;
  invOut.m[0][2] = c20 * invDet;

  invOut.m[1][0] = c01 * invDet;
  invOut.m[1][1] = c11 * invDet;
  invOut.m[1][2] = c21 * invDet;

  invOut.m[2][0] = c02 * invDet;
  invOut.m[2][1] = c12 * invDet;
  invOut.m[2][2] = c22 * invDet;
  return true;
}

// Solve 8x8 linear system via Gaussian elimination (small, fixed size)
static inline bool solve8(double A[8][8], double b[8], double x[8]) {
  for (int col = 0; col < 8; ++col) {
    int piv = col;
    double best = std::abs(A[col][col]);
    for (int r = col + 1; r < 8; ++r) {
      double v = std::abs(A[r][col]);
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-18) return false;
    if (piv != col) {
      for (int c = col; c < 8; ++c) std::swap(A[piv][c], A[col][c]);
      std::swap(b[piv], b[col]);
    }
    const double diag = A[col][col];
    for (int c = col; c < 8; ++c) A[col][c] /= diag;
    b[col] /= diag;

    for (int r = 0; r < 8; ++r) {
      if (r == col) continue;
      const double f = A[r][col];
      if (std::abs(f) < 1e-18) continue;
      for (int c = col; c < 8; ++c) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }
  for (int i = 0; i < 8; ++i) x[i] = b[i];
  return true;
}

// Homography from 4 point correspondences: (x,y)->(u,v), with h33=1
static inline bool homography4pt(const std::array<Vec2, 4>& src, const std::array<Vec2, 4>& dst, Mat3& H, Mat3& Hinv) {
  double A[8][8];
  double b[8];
  for (int i = 0; i < 8; ++i) {
    b[i] = 0;
    for (int j = 0; j < 8; ++j) A[i][j] = 0;
  }

  for (int i = 0; i < 4; ++i) {
    const double x = src[i].x, y = src[i].y;
    const double u = dst[i].x, v = dst[i].y;

    // row for u
    A[2 * i + 0][0] = x;
    A[2 * i + 0][1] = y;
    A[2 * i + 0][2] = 1;
    A[2 * i + 0][6] = -u * x;
    A[2 * i + 0][7] = -u * y;
    b[2 * i + 0] = u;

    // row for v
    A[2 * i + 1][3] = x;
    A[2 * i + 1][4] = y;
    A[2 * i + 1][5] = 1;
    A[2 * i + 1][6] = -v * x;
    A[2 * i + 1][7] = -v * y;
    b[2 * i + 1] = v;
  }

  double xsol[8];
  if (!solve8(A, b, xsol)) return false;

  // fill H with h33=1
  H = Mat3::identity();
  H.m[0][0] = xsol[0]; H.m[0][1] = xsol[1]; H.m[0][2] = xsol[2];
  H.m[1][0] = xsol[3]; H.m[1][1] = xsol[4]; H.m[1][2] = xsol[5];
  H.m[2][0] = xsol[6]; H.m[2][1] = xsol[7]; H.m[2][2] = 1;

  if (!invertMat3(H, Hinv)) return false;
  return true;
}

enum class MaterialType { Curtain, Diffuse, Metal };

struct Material {
  MaterialType type = MaterialType::Diffuse;
  Color albedo = {1, 1, 1};      // Diffuse base or Metal reflectance
  double roughness = 0.0;        // unused in ideal mirror
};

struct Triangle {
  Vec3 p0, p1, p2;
  Material mat;

  Plane plane() const { return planeThrough3(p0, p1, p2); }

  Vec3 normalUnit() const { return normalize(cross(p1 - p0, p2 - p0)); }

  Vec3 pointFromUV(const Vec2& uv) const { return p0 + (p1 - p0) * uv.x + (p2 - p0) * uv.y; }

  // Solve X = p0 + u*(p1-p0) + v*(p2-p0) in least-square on plane basis via Gram 2x2.
  Vec2 uvFromPoint(const Vec3& X) const {
    const Vec3 e1 = p1 - p0;
    const Vec3 e2 = p2 - p0;
    const Vec3 w = X - p0;

    const double a = dot(e1, e1);
    const double b = dot(e1, e2);
    const double c = dot(e2, e2);
    const double d = dot(e1, w);
    const double e = dot(e2, w);

    const double det = a * c - b * b;
    if (std::abs(det) < 1e-18) return {0, 0};
    const double invDet = 1.0 / det;
    const double u = (d * c - b * e) * invDet;
    const double v = (a * e - b * d) * invDet;
    return {u, v};
  }

  bool containsUV(const Vec2& uv) const {
    return uv.x >= -1e-9 && uv.y >= -1e-9 && (uv.x + uv.y) <= 1.0 + 1e-9;
  }
};

struct Image {
  int w = 0, h = 0;
  std::vector<Color> color;
  std::vector<double> z;

  Image(int w_, int h_) : w(w_), h(h_), color(w_ * h_), z(w_ * h_, std::numeric_limits<double>::infinity()) {}

  void clear(const Color& c) {
    std::fill(color.begin(), color.end(), c);
    std::fill(z.begin(), z.end(), std::numeric_limits<double>::infinity());
  }

  void setIfCloser(int x, int y, double depth, const Color& c) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    const int idx = y * w + x;
    if (depth < z[idx]) {
      z[idx] = depth;
      color[idx] = c;
    }
  }

  void writePPM(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for (int i = 0; i < w * h; ++i) {
      Color c = clamp01(color[i]);
      uint8_t r = (uint8_t)std::lround(c.r * 255.0);
      uint8_t g = (uint8_t)std::lround(c.g * 255.0);
      uint8_t b = (uint8_t)std::lround(c.b * 255.0);
      f.write((char*)&r, 1);
      f.write((char*)&g, 1);
      f.write((char*)&b, 1);
    }
  }
};

struct Frustum {
  std::array<Plane, 5> planes; // 3 side + near + far(optional as large)
  int count = 0;
};

static inline std::optional<Vec3> intersectRayPlane(const Vec3& E, const Vec3& dir_unit, const Plane& pl, double& tOut) {
  const double denom = dot(pl.n, dir_unit);
  if (std::abs(denom) < 1e-12) return std::nullopt;
  const double t = -(dot(pl.n, E) + pl.d) / denom;
  tOut = t;
  return E + dir_unit * t;
}

static inline std::vector<Vec3> clipPolyAgainstPlane(const std::vector<Vec3>& poly, const Plane& pl) {
  std::vector<Vec3> out;
  if (poly.empty()) return out;
  auto inside = [&](const Vec3& p) { return pl.sd(p) >= -1e-9; };

  for (size_t i = 0; i < poly.size(); ++i) {
    const Vec3 S = poly[i];
    const Vec3 E = poly[(i + 1) % poly.size()];
    const double ds = pl.sd(S);
    const double de = pl.sd(E);
    const bool inS = ds >= -1e-9;
    const bool inE = de >= -1e-9;

    if (inS && inE) {
      out.push_back(E);
    } else if (inS && !inE) {
      const double t = ds / (ds - de);
      out.push_back(S + (E - S) * t);
    } else if (!inS && inE) {
      const double t = ds / (ds - de);
      out.push_back(S + (E - S) * t);
      out.push_back(E);
    }
  }
  return out;
}

static inline std::vector<Triangle> triangulateFan(const std::vector<Vec3>& poly, const Material& mat) {
  std::vector<Triangle> tris;
  if (poly.size() < 3) return tris;
  for (size_t i = 1; i + 1 < poly.size(); ++i) {
    Triangle t;
    t.p0 = poly[0];
    t.p1 = poly[i];
    t.p2 = poly[i + 1];
    t.mat = mat;
    tris.push_back(t);
  }
  return tris;
}

static inline Frustum buildFrustum(const Vec3& E, const Triangle& Tv, double near_eps, std::optional<double> far_opt) {
  Frustum fr;
  fr.count = 0;

  // Ensure Tv normal points away from eye along view direction
  Vec3 nTv = Tv.normalUnit();
  Vec3 cTv = (Tv.p0 + Tv.p1 + Tv.p2) / 3.0;
  Vec3 viewDir = normalize(cTv - E);
  if (dot(nTv, viewDir) < 0) nTv = nTv * -1.0;

  // Side planes: each passes through E and an edge of Tv; inward means Tv's opposite vertex is inside.
  auto addSide = [&](const Vec3& A, const Vec3& B, const Vec3& C_opp) {
    Vec3 v0 = A - E;
    Vec3 v1 = B - E;
    Vec3 n = normalize(cross(v0, v1)); // orientation depends
    Plane pl = planeFromPointNormal(E, n);
    if (pl.sd(C_opp) < 0) {
      pl.n = pl.n * -1.0;
      pl.d = -pl.d;
    }
    fr.planes[fr.count++] = pl;
  };
  addSide(Tv.p0, Tv.p1, Tv.p2);
  addSide(Tv.p1, Tv.p2, Tv.p0);
  addSide(Tv.p2, Tv.p0, Tv.p1);

  // Near plane: perpendicular to viewDir (use nTv) at E + nTv * near_eps
  fr.planes[fr.count++] = planeFromPointNormal(E + nTv * near_eps, nTv);

  // Far plane if provided: at E + nTv * far, normal facing back (-nTv) to keep inside between near/far
  if (far_opt.has_value()) {
    fr.planes[fr.count++] = planeFromPointNormal(E + nTv * (*far_opt), nTv * -1.0);
  }
  return fr;
}

static inline std::vector<Triangle> clipTriangleToFrustum(const Triangle& tri, const Frustum& fr) {
  std::vector<Vec3> poly = {tri.p0, tri.p1, tri.p2};
  for (int i = 0; i < fr.count; ++i) {
    poly = clipPolyAgainstPlane(poly, fr.planes[i]);
    if (poly.size() < 3) return {};
  }
  return triangulateFan(poly, tri.mat);
}

// 2D Sutherland–Hodgman: clip polygon P by triangle T (given in 2D)
static inline std::vector<Vec2> clipPolyByTri2D(const std::vector<Vec2>& poly, const std::array<Vec2, 3>& tri) {
  std::vector<Vec2> out = poly;
  auto clipByEdge = [&](const Vec2& A, const Vec2& B, const Vec2& insideRef) {
    std::vector<Vec2> res;
    if (out.empty()) return res;

    Vec2 e = B - A;
    double sign = cross2(e, insideRef - A);
    if (std::abs(sign) < 1e-18) sign = 1.0;

    auto inside = [&](const Vec2& p) { return cross2(e, p - A) * sign >= -1e-9; };

    for (size_t i = 0; i < out.size(); ++i) {
      Vec2 S = out[i];
      Vec2 E = out[(i + 1) % out.size()];
      bool inS = inside(S);
      bool inE = inside(E);

      auto intersect = [&]() -> Vec2 {
        Vec2 d = E - S;
        double num = cross2(e, A - S);
        double den = cross2(e, d);
        if (std::abs(den) < 1e-18) return S;
        double t = num / den;
        return S + d * t;
      };

      if (inS && inE) {
        res.push_back(E);
      } else if (inS && !inE) {
        res.push_back(intersect());
      } else if (!inS && inE) {
        res.push_back(intersect());
        res.push_back(E);
      }
    }
    return res;
  };

  // clip in order by each edge, using opposite vertex as inside reference
  out = clipByEdge(tri[0], tri[1], tri[2]);
  out = clipByEdge(tri[1], tri[2], tri[0]);
  out = clipByEdge(tri[2], tri[0], tri[1]);
  return out;
}

static inline double triArea2(const Vec2& a, const Vec2& b, const Vec2& c) {
  return std::abs(cross2(b - a, c - a)) * 0.5;
}

static inline bool barycentric2D(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c, double& w0, double& w1, double& w2) {
  const Vec2 v0 = b - a;
  const Vec2 v1 = c - a;
  const Vec2 v2 = p - a;
  const double den = cross2(v0, v1);
  if (std::abs(den) < 1e-18) return false;
  w1 = cross2(v2, v1) / den;
  w2 = cross2(v0, v2) / den;
  w0 = 1.0 - w1 - w2;
  return true;
}

struct Screen {
  // Root "curtain" is a rectangle in 3D: O + u*U + v*V, u,v in [0,1]
  Vec3 O, U, V;
  int W = 0, H = 0;

  Vec3 pointFromUV01(const Vec2& uv01) const { return O + U * uv01.x + V * uv01.y; }

  Vec2 uv01FromPixelCenter(int x, int y) const {
    // y down
    double u = (x + 0.5) / (double)W;
    double v = (y + 0.5) / (double)H;
    return {u, v};
  }
};

struct DepthOverride {
  Plane writePlane; // first mirror plane in root view
  bool valid = false;
};

struct RenderConfig {
  double near_eps = 1e-4;
  std::optional<double> far = std::nullopt;
  int L_max = 4;
  double A_min_pixels = 2.0;
};

struct RenderContext {
  const std::vector<Triangle>* scene = nullptr;
  Image* img = nullptr;
  Screen screen;
  Vec3 rootEye;
  RenderConfig cfg;
};

static inline Vec2 applyH(const Mat3& H, const Vec2& p) {
  Vec3 hp = mul_h(H, Vec3{p.x, p.y, 1.0});
  if (std::abs(hp.z) < 1e-18) return {hp.x, hp.y};
  return {hp.x / hp.z, hp.y / hp.z};
}

static inline std::optional<std::pair<double, Vec3>> intersectRayTrianglePlane(const Vec3& E, const Vec3& dir_unit, const Triangle& tri) {
  Plane pl = tri.plane();
  double t = 0;
  auto Xopt = intersectRayPlane(E, dir_unit, pl, t);
  if (!Xopt.has_value()) return std::nullopt;
  return std::make_pair(t, *Xopt);
}

static inline Vec3 reflectPointAboutPlane(const Vec3& P, const Plane& pl) {
  // pl is normalized
  const double s = pl.sd(P);
  return P - pl.n * (2.0 * s);
}

static inline double estimateTrianglePixelArea(const Mat3& H_toRoot, const Triangle& Tv_local, const Screen& screen) {
  // Tv_local canonical coords: (0,0),(1,0),(0,1)
  Vec2 r0 = applyH(H_toRoot, Vec2{0, 0});
  Vec2 r1 = applyH(H_toRoot, Vec2{1, 0});
  Vec2 r2 = applyH(H_toRoot, Vec2{0, 1});
  Vec2 p0 = {r0.x * screen.W, r0.y * screen.H};
  Vec2 p1 = {r1.x * screen.W, r1.y * screen.H};
  Vec2 p2 = {r2.x * screen.W, r2.y * screen.H};
  return triArea2(p0, p1, p2);
}

static inline Color shadeDiffuseSimple(const Triangle& tri, const Vec3& X, const Vec3& dir_unit) {
  Vec3 n = tri.normalUnit();
  // Simple directional light + ambient
  Vec3 L = normalize(Vec3{-0.3, -1.0, -0.2}); // light coming from above
  double ndotl = std::max(0.0, dot(n, L * -1.0));
  double ambient = 0.08;
  double intensity = ambient + 0.92 * ndotl;

  // optional: face-forward
  if (dot(n, dir_unit) > 0) n = n * -1.0;

  return tri.mat.albedo * intensity;
}

static void renderTriangleWithTriangle(
    RenderContext& ctx,
    const Vec3& E_cur,
    const Triangle& Tv_local,
    const Mat3& H_accum, const Mat3& H_accum_inv,
    const Color& throughput,
    const DepthOverride& depthOverride,
    int depthLevel);

// Diffuse branch: clip piece already in frustum, project to Tv plane, rasterize in root pixels
static void renderDiffusePiece(
    RenderContext& ctx,
    const Vec3& E_cur,
    const Triangle& Tv_local,
    const Triangle& triPiece,
    const Mat3& H_accum, const Mat3& H_accum_inv,
    const Color& throughput,
    const DepthOverride& depthOverride) {

  // Project triPiece vertices to Tv plane via rays from E_cur
  Plane tvPl = Tv_local.plane();
  std::array<Vec2, 3> projUV;
  std::array<Vec3, 3> projP;

  std::array<Vec3, 3> V = {triPiece.p0, triPiece.p1, triPiece.p2};
  for (int i = 0; i < 3; ++i) {
    Vec3 dir = normalize(V[i] - E_cur);
    double t = 0;
    auto Xopt = intersectRayPlane(E_cur, dir, tvPl, t);
    if (!Xopt.has_value() || t <= ctx.cfg.near_eps) return;
    projP[i] = *Xopt;
    projUV[i] = Tv_local.uvFromPoint(projP[i]); // in Tv_local uv-space (not necessarily inside Tv triangle, but should)
  }

  // Map projected triangle into root uv01 via H_accum
  std::array<Vec2, 3> rootUV01 = {
      applyH(H_accum, projUV[0]),
      applyH(H_accum, projUV[1]),
      applyH(H_accum, projUV[2]),
  };

  // Pixel bbox in root image
  auto toPix = [&](const Vec2& uv01) -> Vec2 {
    return Vec2{uv01.x * ctx.screen.W, uv01.y * ctx.screen.H};
  };
  Vec2 p0 = toPix(rootUV01[0]);
  Vec2 p1 = toPix(rootUV01[1]);
  Vec2 p2 = toPix(rootUV01[2]);

  int xmin = (int)std::floor(std::min({p0.x, p1.x, p2.x}));
  int xmax = (int)std::ceil (std::max({p0.x, p1.x, p2.x}));
  int ymin = (int)std::floor(std::min({p0.y, p1.y, p2.y}));
  int ymax = (int)std::ceil (std::max({p0.y, p1.y, p2.y}));
  xmin = std::max(xmin, 0); ymin = std::max(ymin, 0);
  xmax = std::min(xmax, ctx.img->w - 1); ymax = std::min(ymax, ctx.img->h - 1);

  // For each pixel center: root uv01 -> tv uv via inverse H_accum -> inside test -> ray intersection
  for (int y = ymin; y <= ymax; ++y) {
    for (int x = xmin; x <= xmax; ++x) {
      Vec2 uv01 = ctx.screen.uv01FromPixelCenter(x, y);
      Vec2 uv_tv = applyH(H_accum_inv, uv01);

      double w0, w1, w2;
      if (!barycentric2D(uv_tv, projUV[0], projUV[1], projUV[2], w0, w1, w2)) continue;
      if (w0 < -1e-8 || w1 < -1e-8 || w2 < -1e-8) continue;

      // Build point on Tv plane for this uv_tv
      Vec3 P_on_tv = Tv_local.pointFromUV(uv_tv);
      Vec3 dir = normalize(P_on_tv - E_cur);

      auto hit = intersectRayTrianglePlane(E_cur, dir, triPiece);
      if (!hit.has_value()) continue;
      double t_virtual = hit->first;
      Vec3 X = hit->second;
      if (t_virtual <= ctx.cfg.near_eps) continue;

      // Ensure inside triPiece in its own uv
      Vec2 uv_piece = triPiece.uvFromPoint(X);
      if (!triPiece.containsUV(uv_piece)) continue;

      // Depth for z-buffer: either real hit depth, or overridden mirror depth in root view
      double depth = t_virtual;
      if (depthOverride.valid) {
        Vec3 P_root = ctx.screen.pointFromUV01(uv01);
        Vec3 dir_root = normalize(P_root - ctx.rootEye);
        double t_override = 0;
        auto Xo = intersectRayPlane(ctx.rootEye, dir_root, depthOverride.writePlane, t_override);
        if (!Xo.has_value() || t_override <= ctx.cfg.near_eps) continue;
        depth = t_override;
      }

      Color base = shadeDiffuseSimple(triPiece, X, dir);
      Color out = base * throughput;
      ctx.img->setIfCloser(x, y, depth, out);
    }
  }
}

static void renderMetalTriangle(
    RenderContext& ctx,
    const Vec3& E_cur,
    const Triangle& Tv_local, // parent curtain viewport
    const Triangle& triMetal, // unclipped original metal tri OR clipped piece; both ok if plane same
    const Mat3& H_accum, const Mat3& H_accum_inv,
    const Color& throughput,
    const DepthOverride& depthOverride,
    int depthLevel) {

  if (depthLevel >= ctx.cfg.L_max) return;

  Plane metalPl = triMetal.plane(); // normalized
  // Footprint of parent frustum on metal plane: intersect 3 boundary rays (E_cur -> Tv_local vertices)
  std::array<Vec3, 3> Q;
  std::array<Vec3, 3> Sv = {Tv_local.p0, Tv_local.p1, Tv_local.p2};
  for (int i = 0; i < 3; ++i) {
    Vec3 dir = normalize(Sv[i] - E_cur);
    double t = 0;
    auto Xopt = intersectRayPlane(E_cur, dir, metalPl, t);
    if (!Xopt.has_value() || t <= ctx.cfg.near_eps) return;
    Q[i] = *Xopt;
  }

  // Clip footprint triangle Q by the metal triangle in-plane (2D)
  // Use metal triangle local 2D coords in its own basis
  auto to2D_inMetal = [&](const Vec3& X) { return triMetal.uvFromPoint(X); };
  std::array<Vec2, 3> metalTri2D = {Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}};
  std::vector<Vec2> poly2D = {to2D_inMetal(Q[0]), to2D_inMetal(Q[1]), to2D_inMetal(Q[2])};
  poly2D = clipPolyByTri2D(poly2D, metalTri2D);
  if (poly2D.size() < 3) return;

  // Triangulate child viewport polygon on metal plane
  std::vector<Triangle> childViewports;
  for (size_t i = 1; i + 1 < poly2D.size(); ++i) {
    Triangle tv;
    tv.p0 = triMetal.pointFromUV(poly2D[0]);
    tv.p1 = triMetal.pointFromUV(poly2D[i]);
    tv.p2 = triMetal.pointFromUV(poly2D[i + 1]);
    tv.mat.type = MaterialType::Curtain; // IMPORTANT: child is a viewport, not a drawable
    tv.mat.albedo = {0, 0, 0};
    childViewports.push_back(tv);
  }

  // Mirror camera
  Vec3 E_mirror = reflectPointAboutPlane(E_cur, metalPl);

  // Decide depth override: lock to the first mirror plane in root view
  DepthOverride nextOverride = depthOverride;
  if (!nextOverride.valid) {
    nextOverride.valid = true;
    nextOverride.writePlane = metalPl;
  }

  // Recurse for each child viewport
  for (const Triangle& Tv_child : childViewports) {
    // Stop condition by projected pixel area in root
    // Need H_step: child uv -> parent uv, induced by central projection from E_cur onto parent plane
    Plane parentPl = Tv_local.plane();

    auto projToParent = [&](const Vec3& X_on_child) -> Vec3 {
      Vec3 dir = normalize(X_on_child - E_cur);
      double t = 0;
      auto Xp = intersectRayPlane(E_cur, dir, parentPl, t);
      if (!Xp.has_value()) return Tv_local.p0;
      return *Xp;
    };

    // 4 correspondences: 3 vertices + centroid
    std::array<Vec2, 4> src_uv_child = {
        Vec2{0, 0},
        Vec2{1, 0},
        Vec2{0, 1},
        Vec2{1.0 / 3.0, 1.0 / 3.0},
    };
    Vec3 C3 = (Tv_child.p0 + Tv_child.p1 + Tv_child.p2) / 3.0;
    std::array<Vec3, 4> child3 = {Tv_child.p0, Tv_child.p1, Tv_child.p2, C3};
    std::array<Vec2, 4> dst_uv_parent;
    for (int i = 0; i < 4; ++i) {
      Vec3 Xp = projToParent(child3[i]);
      dst_uv_parent[i] = Tv_local.uvFromPoint(Xp);
    }

    Mat3 H_step, H_step_inv;
    if (!homography4pt(src_uv_child, dst_uv_parent, H_step, H_step_inv)) continue;

    Mat3 H_child_toRoot = mul(H_accum, H_step);
    Mat3 H_child_toRoot_inv;
    if (!invertMat3(H_child_toRoot, H_child_toRoot_inv)) continue;

    double areaPx = estimateTrianglePixelArea(H_child_toRoot, Tv_child, ctx.screen);
    if (areaPx < ctx.cfg.A_min_pixels) continue;

    Color nextThroughput = throughput * triMetal.mat.albedo; // metal reflectance tint
    renderTriangleWithTriangle(ctx, E_mirror, Tv_child, H_child_toRoot, H_child_toRoot_inv, nextThroughput, nextOverride, depthLevel + 1);
  }
}

static void renderTriangleWithTriangle(
    RenderContext& ctx,
    const Vec3& E_cur,
    const Triangle& Tv_local,
    const Mat3& H_accum, const Mat3& H_accum_inv,
    const Color& throughput,
    const DepthOverride& depthOverride,
    int depthLevel) {

  if (Tv_local.mat.type != MaterialType::Curtain) {
    // In this design, only Curtain triangles are viewports.
    // (Metal recursion creates Tv_child as Curtain.)
    return;
  }

  // Build frustum from E_cur and current viewport triangle
  Frustum fr = buildFrustum(E_cur, Tv_local, ctx.cfg.near_eps, ctx.cfg.far);

  // Collect candidates: clip each scene triangle against frustum
  for (const Triangle& tri : *ctx.scene) {
    if (tri.mat.type == MaterialType::Curtain) continue; // do not render curtain geometry
    std::vector<Triangle> pieces = clipTriangleToFrustum(tri, fr);
    for (const Triangle& piece : pieces) {
      if (piece.mat.type == MaterialType::Diffuse) {
        renderDiffusePiece(ctx, E_cur, Tv_local, piece, H_accum, H_accum_inv, throughput, depthOverride);
      } else if (piece.mat.type == MaterialType::Metal) {
        // Metal is recursive, no direct color; use parent frustum footprint method
        renderMetalTriangle(ctx, E_cur, Tv_local, piece, H_accum, H_accum_inv, throughput, depthOverride, depthLevel);
      }
    }
  }
}

// Build Cornell Box (5 faces) + 2 metal boxes
static std::vector<Triangle> buildScene() {
  std::vector<Triangle> s;

  auto addQuad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, const Material& m) {
    // (a,b,c) and (a,c,d)
    Triangle t1{a, b, c, m};
    Triangle t2{a, c, d, m};
    s.push_back(t1);
    s.push_back(t2);
  };

  // Cornell dimensions
  // Coordinate system: x right, y up, z forward (into the box)
  const double W = 2.0;
  const double H = 2.0;
  const double D = 2.5;

  Vec3 A{-W/2, 0, 0};
  Vec3 B{ W/2, 0, 0};
  Vec3 C{ W/2, H, 0};
  Vec3 Dd{-W/2, H, 0};

  Vec3 A2{-W/2, 0, D};
  Vec3 B2{ W/2, 0, D};
  Vec3 C2{ W/2, H, D};
  Vec3 D2{-W/2, H, D};

  Material white{MaterialType::Diffuse, {0.75, 0.75, 0.75}};
  Material red{MaterialType::Diffuse, {0.75, 0.15, 0.15}};
  Material green{MaterialType::Diffuse, {0.15, 0.75, 0.15}};

  // Floor: A..B..B2..A2
  addQuad(A, B, B2, A2, white);
  // Ceiling: Dd..C..C2..D2
  addQuad(Dd, C, C2, D2, white);
  // Back wall: A2..B2..C2..D2
  addQuad(A2, B2, C2, D2, white);
  // Left wall: A..A2..D2..Dd
  addQuad(A, A2, D2, Dd, red);
  // Right wall: B..C..C2..B2
  addQuad(B, C, C2, B2, green);

  auto addBox = [&](Vec3 minP, Vec3 maxP, const Material& m) {
    Vec3 p000{minP.x, minP.y, minP.z};
    Vec3 p100{maxP.x, minP.y, minP.z};
    Vec3 p110{maxP.x, maxP.y, minP.z};
    Vec3 p010{minP.x, maxP.y, minP.z};

    Vec3 p001{minP.x, minP.y, maxP.z};
    Vec3 p101{maxP.x, minP.y, maxP.z};
    Vec3 p111{maxP.x, maxP.y, maxP.z};
    Vec3 p011{minP.x, maxP.y, maxP.z};

    // 6 faces
    addQuad(p000, p100, p110, p010, m); // front (z=min)
    addQuad(p001, p011, p111, p101, m); // back  (z=max)
    addQuad(p000, p010, p011, p001, m); // left
    addQuad(p100, p101, p111, p110, m); // right
    addQuad(p010, p110, p111, p011, m); // top
    addQuad(p000, p001, p101, p100, m); // bottom
  };

  Material metalBlue{MaterialType::Metal, {0.35, 0.45, 0.95}};
  Material metalYellow{MaterialType::Metal, {0.95, 0.85, 0.25}};

  // Two boxes inside (raise above floor a bit)
  addBox(Vec3{-0.6, 0.0, 0.7}, Vec3{-0.1, 0.9, 1.2}, metalBlue);
  addBox(Vec3{ 0.1, 0.0, 1.2}, Vec3{ 0.7, 1.4, 1.9}, metalYellow);

  return s;
}

struct Camera {
  Vec3 E;
  Triangle Tv0, Tv1; // Curtain triangles covering screen
  Screen screen;

  static Camera makeDefault(int W, int H) {
    Camera cam;
    cam.E = Vec3{0, 1.0, -3.2};

    // Screen rectangle in world: put in front of eye
    // Define 4 corners of screen quad in world (z=-1.0 plane, spanning [-1,1]x[0,2])
    Vec3 a{-1.0, 0.0, -1.0};
    Vec3 b{ 1.0, 0.0, -1.0};
    Vec3 c{ 1.0, 2.0, -1.0};
    Vec3 d{-1.0, 2.0, -1.0};

    Material curtain{MaterialType::Curtain, {0, 0, 0}};
    cam.Tv0 = Triangle{a, b, c, curtain};
    cam.Tv1 = Triangle{a, c, d, curtain};

    cam.screen.O = a;
    cam.screen.U = (b - a);
    cam.screen.V = (d - a);
    cam.screen.W = W;
    cam.screen.H = H;

    return cam;
  }

  void render(const std::vector<Triangle>& scene, Image& img, const RenderConfig& cfg) const {
    RenderContext ctx;
    ctx.scene = &scene;
    ctx.img = &img;
    ctx.screen = screen;
    ctx.rootEye = E;
    ctx.cfg = cfg;

    Mat3 H0 = Mat3::identity();
    Mat3 H0inv = Mat3::identity();
    DepthOverride none;
    none.valid = false;

    Color throughput{1, 1, 1};

    renderTriangleWithTriangle(ctx, E, Tv0, H0, H0inv, throughput, none, 0);
    renderTriangleWithTriangle(ctx, E, Tv1, H0, H0inv, throughput, none, 0);
  }
};

int main(int argc, char** argv) {
  std::string out = "out.ppm";
  if (argc >= 2) out = argv[1];

  int W = 640, H = 480;
  Image img(W, H);
  img.clear(Color{0.0, 0.0, 0.0});

  std::vector<Triangle> scene = buildScene();
  Camera cam = Camera::makeDefault(W, H);

  RenderConfig cfg;
  cfg.near_eps = 1e-4;
  cfg.far = std::nullopt; // or 10.0
  cfg.L_max = 4;
  cfg.A_min_pixels = 2.0;

  cam.render(scene, img, cfg);
  img.writePPM(out);

  std::cerr << "Wrote " << out << "\n";
  return 0;
}
