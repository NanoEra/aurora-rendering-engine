#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

static constexpr double kEps = 1e-9;

static inline double clampd(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}
static inline int clampi(int x, int lo, int hi) {
    return std::max(lo, std::min(hi, x));
}

struct Vec2 {
    double x = 0, y = 0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& r) const { return {x + r.x, y + r.y}; }
    Vec2 operator-(const Vec2& r) const { return {x - r.x, y - r.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
};

static inline double cross2(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
    Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& r) { x += r.x; y += r.y; z += r.z; return *this; }
};

static inline Vec3 mul(const Vec3& a, const Vec3& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}
static inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
static inline double length(const Vec3& v) {
    return std::sqrt(dot(v, v));
}
static inline Vec3 normalize(const Vec3& v) {
    double len = length(v);
    if (len < kEps) return {0, 0, 0};
    return v / len;
}
static inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return a * (1.0 - t) + b * t;
}

struct Image {
    int w = 0, h = 0;
    std::vector<Vec3> pix;

    Image() = default;
    Image(int w_, int h_, Vec3 fill = {0,0,0}) : w(w_), h(h_), pix((size_t)w_ * (size_t)h_, fill) {}

    Vec3 get(int x, int y) const {
        x = clampi(x, 0, w - 1);
        y = clampi(y, 0, h - 1);
        return pix[(size_t)y * (size_t)w + (size_t)x];
    }
    void set(int x, int y, const Vec3& c) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        pix[(size_t)y * (size_t)w + (size_t)x] = c;
    }

    // UV: [0,1]x[0,1], v向下（0为顶端），便于图像坐标。
    Vec3 sampleBilinear(double u, double v) const {
        u = clampd(u, 0.0, 1.0);
        v = clampd(v, 0.0, 1.0);
        double fx = u * (w - 1);
        double fy = v * (h - 1);
        int x0 = (int)std::floor(fx);
        int y0 = (int)std::floor(fy);
        int x1 = std::min(x0 + 1, w - 1);
        int y1 = std::min(y0 + 1, h - 1);
        double tx = fx - x0;
        double ty = fy - y0;

        Vec3 c00 = get(x0, y0);
        Vec3 c10 = get(x1, y0);
        Vec3 c01 = get(x0, y1);
        Vec3 c11 = get(x1, y1);

        Vec3 cx0 = lerp(c00, c10, tx);
        Vec3 cx1 = lerp(c01, c11, tx);
        return lerp(cx0, cx1, ty);
    }

    void writePPM(const std::string& path, double gamma = 2.2) const {
        std::ofstream out(path, std::ios::binary);
        if (!out) {
            throw std::runtime_error("Failed to open output file: " + path);
        }
        out << "P6\n" << w << " " << h << "\n255\n";
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                Vec3 c = pix[(size_t)y * (size_t)w + (size_t)x];
                c.x = clampd(c.x, 0.0, 1.0);
                c.y = clampd(c.y, 0.0, 1.0);
                c.z = clampd(c.z, 0.0, 1.0);
                // gamma
                c.x = std::pow(c.x, 1.0 / gamma);
                c.y = std::pow(c.y, 1.0 / gamma);
                c.z = std::pow(c.z, 1.0 / gamma);

                unsigned char rgb[3] = {
                    (unsigned char)clampi((int)std::lround(c.x * 255.0), 0, 255),
                    (unsigned char)clampi((int)std::lround(c.y * 255.0), 0, 255),
                    (unsigned char)clampi((int)std::lround(c.z * 255.0), 0, 255),
                };
                out.write((char*)rgb, 3);
            }
        }
    }
};

enum class MaterialType {
    Diffuse,
    Reflective,
};

struct Material {
    MaterialType type = MaterialType::Diffuse;
    Vec3 albedo = {1,1,1};     // 基色/金属色
    double metalness = 0.0;    // Reflective时用于混合反射与基色（0..1）
};

struct Triangle {
    int id = -1;
    Vec3 p[3];
    Vec2 uv[3];
    Material* mat = nullptr;

    Vec3 normal() const {
        return normalize(cross(p[1] - p[0], p[2] - p[0]));
    }
    Vec3 centroid() const {
        return (p[0] + p[1] + p[2]) / 3.0;
    }
};

// ---------------------------
// 3D barycentric (点在三角形平面上时使用)
// ---------------------------
static inline std::array<double,3> barycentric3D(const Vec3& P, const Triangle& T) {
    Vec3 v0 = T.p[1] - T.p[0];
    Vec3 v1 = T.p[2] - T.p[0];
    Vec3 v2 = P - T.p[0];

    double d00 = dot(v0, v0);
    double d01 = dot(v0, v1);
    double d11 = dot(v1, v1);
    double d20 = dot(v2, v0);
    double d21 = dot(v2, v1);

    double denom = d00 * d11 - d01 * d01;
    if (std::fabs(denom) < kEps) return { -1, -1, -1 };

    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return {u, v, w};
}

static inline bool pointInTriangle3D(const Vec3& P, const Triangle& T, double eps = 1e-8) {
    auto bc = barycentric3D(P, T);
    return bc[0] >= -eps && bc[1] >= -eps && bc[2] >= -eps;
}

// ---------------------------
// 2D point-in-triangle (UV空间)
// ---------------------------
static inline bool pointInTriangle2D(const Vec2& P, const Vec2& A, const Vec2& B, const Vec2& C, double eps = 1e-10) {
    // 允许任意绕序：用同向性判断
    Vec2 v0 = B - A;
    Vec2 v1 = C - B;
    Vec2 v2 = A - C;
    Vec2 w0 = P - A;
    Vec2 w1 = P - B;
    Vec2 w2 = P - C;

    double c0 = cross2(v0, w0);
    double c1 = cross2(v1, w1);
    double c2 = cross2(v2, w2);

    bool hasNeg = (c0 < -eps) || (c1 < -eps) || (c2 < -eps);
    bool hasPos = (c0 > eps) || (c1 > eps) || (c2 > eps);
    return !(hasNeg && hasPos);
}

// ---------------------------
// 线段-三角形相交（Moller-Trumbore，作为Triangle运算的一部分）
// 注意：本项目核心渲染不依赖每像素Ray，但几何工具函数仍可提供。
// ---------------------------
static inline bool segmentIntersectsTriangle(const Vec3& A, const Vec3& B, const Triangle& T, double* outT = nullptr) {
    Vec3 dir = B - A;
    Vec3 e1 = T.p[1] - T.p[0];
    Vec3 e2 = T.p[2] - T.p[0];
    Vec3 pvec = cross(dir, e2);
    double det = dot(e1, pvec);

    if (std::fabs(det) < kEps) return false;
    double invDet = 1.0 / det;

    Vec3 tvec = A - T.p[0];
    double u = dot(tvec, pvec) * invDet;
    if (u < -kEps || u > 1.0 + kEps) return false;

    Vec3 qvec = cross(tvec, e1);
    double v = dot(dir, qvec) * invDet;
    if (v < -kEps || u + v > 1.0 + kEps) return false;

    double t = dot(e2, qvec) * invDet;
    if (t < -kEps || t > 1.0 + kEps) return false; // segment: t in [0,1]
    if (outT) *outT = t;
    return true;
}

static inline bool trianglesIntersectSimple(const Triangle& A, const Triangle& B) {
    // 简化版：只做边-面检测（对一般非共面情况足够）
    for (int i = 0; i < 3; ++i) {
        Vec3 a0 = A.p[i];
        Vec3 a1 = A.p[(i+1)%3];
        if (segmentIntersectsTriangle(a0, a1, B)) return true;
    }
    for (int i = 0; i < 3; ++i) {
        Vec3 b0 = B.p[i];
        Vec3 b1 = B.p[(i+1)%3];
        if (segmentIntersectsTriangle(b0, b1, A)) return true;
    }
    // 顶点包含（共面等情况粗略兜底）
    if (pointInTriangle3D(A.p[0], B) || pointInTriangle3D(B.p[0], A)) return true;
    return false;
}

// ---------------------------
// 把点关于三角形所在平面镜像（对称）
// ---------------------------
static inline Vec3 reflectPointAcrossTrianglePlane(const Vec3& P, const Triangle& T) {
    Vec3 n = T.normal();
    // 平面点选T.p[0]
    double dist = dot(n, P - T.p[0]); // 有符号距离（n已归一）
    return P - n * (2.0 * dist);
}

// ---------------------------
// 以origin->point的射线与viewport三角形平面相交，计算交点，并得到对应的viewport UV（允许在UV外）
// 要求：交点必须在 origin 到 point 之间（t in (0,1)），即 viewport 平面在两者之间。
// ---------------------------
static inline bool projectPointToViewportUV(
    const Vec3& origin,
    const Vec3& point,
    const Triangle& viewport,
    Vec2& outViewportUV,
    Vec3* outWorldHit = nullptr,
    double* outLineT = nullptr
) {
    Vec3 n = viewport.normal();
    Vec3 dir = point - origin;
    double denom = dot(n, dir);
    if (std::fabs(denom) < kEps) return false;

    double t = dot(n, viewport.p[0] - origin) / denom;
    // 关键：只接受平面位于 origin->point 之间的情况
    if (!(t > 1e-7 && t < 1.0 - 1e-7)) return false;

    Vec3 hit = origin + dir * t;
    auto bc = barycentric3D(hit, viewport);
    if (bc[0] < -1e6 || bc[1] < -1e6 || bc[2] < -1e6) return false;

    // UV使用线性插值（即使点在三角形外也能给出UV，用于后续裁剪）
    outViewportUV = viewport.uv[0] * bc[0] + viewport.uv[1] * bc[1] + viewport.uv[2] * bc[2];

    if (outWorldHit) *outWorldHit = hit;
    if (outLineT) *outLineT = t;
    return true;
}

struct UVPairVertex {
    Vec2 dstUV; // 在“当前viewport”的UV
    Vec2 srcUV; // 在“被投影面片纹理”的UV
};

static inline double polygonAreaUV(const std::vector<UVPairVertex>& poly) {
    // poly为凸多边形（裁剪结果），用shoelace
    double a = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec2& p = poly[i].dstUV;
        const Vec2& q = poly[(i + 1) % poly.size()].dstUV;
        a += p.x * q.y - p.y * q.x;
    }
    return 0.5 * a;
}

static inline bool isInsideEdgeLeft(const Vec2& A, const Vec2& B, const Vec2& P, double eps = 1e-12) {
    // 以A->B为边，左侧为内
    return cross2(B - A, P - A) >= -eps;
}

static inline UVPairVertex intersectSegmentWithLine(
    const UVPairVertex& S,
    const UVPairVertex& E,
    const Vec2& A,
    const Vec2& B
) {
    // 使用有符号距离插值求交点：
    // dist(P)=cross(B-A, P-A). dist=0在直线上
    double dS = cross2(B - A, S.dstUV - A);
    double dE = cross2(B - A, E.dstUV - A);
    double t = dS / (dS - dE + 1e-30);
    t = clampd(t, 0.0, 1.0);
    UVPairVertex I;
    I.dstUV = S.dstUV + (E.dstUV - S.dstUV) * t;
    I.srcUV = S.srcUV + (E.srcUV - S.srcUV) * t;
    return I;
}

// 将“投影后的目标三角形(带srcUV)”裁剪到viewport三角形(仅dstUV空间)内部
static inline std::vector<UVPairVertex> clipPolygonToTriangle(
    std::vector<UVPairVertex> poly,
    const Vec2& C0,
    const Vec2& C1,
    const Vec2& C2
) {
    // 确保裁剪三角形绕序为CCW，使“左侧为内”成立
    double triArea = cross2(C1 - C0, C2 - C0);
    Vec2 A0 = C0, A1 = C1, A2 = C2;
    if (triArea < 0.0) std::swap(A1, A2);

    auto clipAgainstEdge = [&](const Vec2& A, const Vec2& B, std::vector<UVPairVertex> inPoly) {
        std::vector<UVPairVertex> out;
        if (inPoly.empty()) return out;
        UVPairVertex S = inPoly.back();
        bool S_inside = isInsideEdgeLeft(A, B, S.dstUV);
        for (const auto& E : inPoly) {
            bool E_inside = isInsideEdgeLeft(A, B, E.dstUV);
            if (E_inside) {
                if (!S_inside) out.push_back(intersectSegmentWithLine(S, E, A, B));
                out.push_back(E);
            } else if (S_inside) {
                out.push_back(intersectSegmentWithLine(S, E, A, B));
            }
            S = E;
            S_inside = E_inside;
        }
        return out;
    };

    poly = clipAgainstEdge(A0, A1, std::move(poly));
    poly = clipAgainstEdge(A1, A2, std::move(poly));
    poly = clipAgainstEdge(A2, A0, std::move(poly));
    return poly;
}

static inline void rasterizeWarpTriangle(
    Image& dst,
    const Vec2& d0, const Vec2& d1, const Vec2& d2,
    const Image& src,
    const Vec2& s0, const Vec2& s1, const Vec2& s2,
    bool overwrite = true
) {
    // dstUV/srcUV均为[0,1]附近（dstUV经过裁剪保证在dst三角形内）

    auto uvToPixel = [&](const Vec2& uv) -> Vec2 {
        return Vec2(uv.x * (dst.w - 1), uv.y * (dst.h - 1));
    };

    Vec2 p0 = uvToPixel(d0);
    Vec2 p1 = uvToPixel(d1);
    Vec2 p2 = uvToPixel(d2);

    double minx = std::floor(std::min({p0.x, p1.x, p2.x}));
    double maxx = std::ceil (std::max({p0.x, p1.x, p2.x}));
    double miny = std::floor(std::min({p0.y, p1.y, p2.y}));
    double maxy = std::ceil (std::max({p0.y, p1.y, p2.y}));

    int x0 = clampi((int)minx, 0, dst.w - 1);
    int x1 = clampi((int)maxx, 0, dst.w - 1);
    int y0 = clampi((int)miny, 0, dst.h - 1);
    int y1 = clampi((int)maxy, 0, dst.h - 1);

    double area = cross2(p1 - p0, p2 - p0);
    if (std::fabs(area) < 1e-12) return;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            Vec2 P((double)x + 0.5, (double)y + 0.5);
            // barycentric in pixel-space
            double w0 = cross2(p1 - P, p2 - P) / area;
            double w1 = cross2(p2 - P, p0 - P) / area;
            double w2 = 1.0 - w0 - w1;

            if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;

            Vec2 suv = s0 * w0 + s1 * w1 + s2 * w2;
            Vec3 sc = src.sampleBilinear(suv.x, suv.y);

            if (overwrite) {
                dst.set(x, y, sc);
            } else {
                Vec3 dc = dst.get(x, y);
                dst.set(x, y, lerp(dc, sc, 0.5));
            }
        }
    }
}

static inline void rasterizeWarpPolygonFan(
    Image& dst,
    const std::vector<UVPairVertex>& poly,
    const Image& src,
    bool overwrite = true
) {
    if (poly.size() < 3) return;
    const auto& v0 = poly[0];
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        const auto& v1 = poly[i];
        const auto& v2 = poly[i + 1];
        rasterizeWarpTriangle(
            dst,
            v0.dstUV, v1.dstUV, v2.dstUV,
            src,
            v0.srcUV, v1.srcUV, v2.srcUV,
            overwrite
        );
    }
}

// ---------------------------
// 收集“可能投射到viewport三角形”的面片列表（指针），用于后续按距离排序（远到近）
// 这里只用“任一顶点能投到viewport内部”作为候选判定（快），实际绘制时会做裁剪。
// ---------------------------
static inline std::vector<const Triangle*> collectCandidateTriangles(
    const std::vector<Triangle>& scene,
    const Vec3& origin,
    const Triangle& viewport
) {
    std::vector<const Triangle*> out;
    out.reserve(scene.size());

    for (const auto& tri : scene) {
        bool anyInside = false;
        for (int i = 0; i < 3; ++i) {
            Vec2 uv;
            Vec3 hit;
            if (!projectPointToViewportUV(origin, tri.p[i], viewport, uv, &hit, nullptr)) continue;
            // 只有当交点在viewport三角形内部才算可能投射
            if (pointInTriangle3D(hit, viewport, 1e-8)) {
                anyInside = true;
                break;
            }
        }
        if (anyInside) out.push_back(&tri);
    }
    return out;
}

// ---------------------------
// 将 targetTri 的纹理投影到 viewportTri 对应的 dstImage 上：
// 1) 把targetTri三个顶点投到viewport平面，得到dstUV三点（允许在viewport外）
// 2) 用Sutherland-Hodgman把投影三角形裁剪到viewport三角形内部
// 3) 将裁剪结果多边形扇形三角化后，进行纹理拉伸/扭曲并写入dst
// 返回：裁剪后投影区域的像素面积估计（用于递归终止与子纹理分辨率估计）
// ---------------------------
static inline double projectAndCompositeTriangleTexture(
    Image& dstImage,
    const Triangle& viewportTri,
    const Vec3& origin,
    const Triangle& targetTri,
    const Image& targetTexture,
    bool overwrite = true
) {
    Vec2 dstUV[3];
    for (int i = 0; i < 3; ++i) {
        Vec2 uv;
        if (!projectPointToViewportUV(origin, targetTri.p[i], viewportTri, uv, nullptr, nullptr)) {
            return 0.0; // 无法投影（平行或平面不在origin->point之间等）
        }
        dstUV[i] = uv;
    }

    std::vector<UVPairVertex> poly(3);
    for (int i = 0; i < 3; ++i) {
        poly[i].dstUV = dstUV[i];
        poly[i].srcUV = targetTri.uv[i];
    }

    // 裁剪到viewport三角形UV边界内
    std::vector<UVPairVertex> clipped = clipPolygonToTriangle(
        std::move(poly),
        viewportTri.uv[0], viewportTri.uv[1], viewportTri.uv[2]
    );
    if (clipped.size() < 3) return 0.0;

    double areaUV = std::fabs(polygonAreaUV(clipped));
    double areaPx = areaUV * (double)dstImage.w * (double)dstImage.h;

    rasterizeWarpPolygonFan(dstImage, clipped, targetTexture, overwrite);
    return areaPx;
}

// ---------------------------
// 核心递归：renderTriangleWithTriangle
// - 传入：scene、观察点origin、当前面片currentTri
// - Diffuse：直接返回(填充)材质颜色纹理
// - Reflective：
//   1) origin关于currentTri平面镜像 -> mirroredOrigin
//   2) 以currentTri作为viewport，从mirroredOrigin遍历其它面片，排序远->近
//   3) 递归渲染其它面片得到其纹理
//   4) 将其它面片纹理投影/裁剪/拉伸贴到currentTri纹理上
//   5) 与基色按metalness混合并返回
// 终止：深度上限 & 当前估计像素面积阈值 & 循环依赖保护（栈内id）
// ---------------------------
struct RenderConfig {
    int maxDepth = 4;
    double minAreaPxToRecurse = 4.0;   // 小于该像素面积就不再递归
    int maxTriTexRes = 256;
    int minTriTexRes = 16;
    Vec3 envColor = {0.08, 0.08, 0.10};
};

static Image renderTriangleWithTriangle(
    const std::vector<Triangle>& scene,
    const Vec3& origin,
    const Triangle& currentTri,
    int texW,
    int texH,
    int depth,
    double estimatedAreaPx,
    std::vector<int>& recursionStack,
    const RenderConfig& cfg
) {
    texW = clampi(texW, cfg.minTriTexRes, cfg.maxTriTexRes);
    texH = clampi(texH, cfg.minTriTexRes, cfg.maxTriTexRes);

    Vec3 baseColor = currentTri.mat ? currentTri.mat->albedo : Vec3(1,1,1);
    Image base(texW, texH, baseColor);

    if (!currentTri.mat || currentTri.mat->type == MaterialType::Diffuse) {
        return base;
    }

    // 递归终止条件
    if (depth >= cfg.maxDepth) return base;
    if (estimatedAreaPx > 0.0 && estimatedAreaPx < cfg.minAreaPxToRecurse) return base;

    // 防止循环递归（镜面对镜面等）
    for (int id : recursionStack) {
        if (id == currentTri.id) return base;
    }
    recursionStack.push_back(currentTri.id);

    Vec3 mirroredOrigin = reflectPointAcrossTrianglePlane(origin, currentTri);

    // 反射缓冲：初始为环境色（避免未命中区域全黑）
    Image reflection(texW, texH, cfg.envColor);

    // 找出可能投射到currentTri(viewport)的面片
    std::vector<const Triangle*> candidates = collectCandidateTriangles(scene, mirroredOrigin, currentTri);

    // 排序：远 -> 近
    std::sort(candidates.begin(), candidates.end(), [&](const Triangle* a, const Triangle* b) {
        double da = length(a->centroid() - mirroredOrigin);
        double db = length(b->centroid() - mirroredOrigin);
        return da > db;
    });

    for (const Triangle* tptr : candidates) {
        if (!tptr) continue;
        if (tptr->id == currentTri.id) continue;

        // 先用投影+裁剪估面积，再决定子纹理分辨率与是否递归
        // 为了获得“面积估计”，我们需要先进行一次“空投影裁剪”
        // 这里复用projectAndCompositeTriangleTexture的思路，但不写入，只算面积：
        Vec2 dstUV[3];
        bool ok = true;
        for (int i = 0; i < 3; ++i) {
            Vec2 uv;
            if (!projectPointToViewportUV(mirroredOrigin, tptr->p[i], currentTri, uv, nullptr, nullptr)) {
                ok = false;
                break;
            }
            dstUV[i] = uv;
        }
        if (!ok) continue;

        std::vector<UVPairVertex> poly(3);
        for (int i = 0; i < 3; ++i) {
            poly[i].dstUV = dstUV[i];
            poly[i].srcUV = tptr->uv[i];
        }
        std::vector<UVPairVertex> clipped = clipPolygonToTriangle(
            std::move(poly),
            currentTri.uv[0], currentTri.uv[1], currentTri.uv[2]
        );
        if (clipped.size() < 3) continue;

        double areaUV = std::fabs(polygonAreaUV(clipped));
        double areaPx = areaUV * (double)texW * (double)texH;
        if (areaPx < 0.5) continue;

        int childRes = (int)std::lround(std::sqrt(areaPx) * 1.2);
        childRes = clampi(childRes, cfg.minTriTexRes, cfg.maxTriTexRes);

        Image childTex = renderTriangleWithTriangle(
            scene,
            mirroredOrigin,
            *tptr,
            childRes, childRes,
            depth + 1,
            areaPx,
            recursionStack,
            cfg
        );

        // 真正写入反射缓冲（近处覆盖远处）
        // 注意：我们已经有clipped了，可直接用它进行扇形三角化绘制
        rasterizeWarpPolygonFan(reflection, clipped, childTex, /*overwrite=*/true);
    }

    recursionStack.pop_back();

    // 将反射与基色混合：金属色会对反射进行“染色”
    double m = clampd(currentTri.mat->metalness, 0.0, 1.0);
    Image out(texW, texH, {0,0,0});
    for (int y = 0; y < texH; ++y) {
        for (int x = 0; x < texW; ++x) {
            Vec3 r = reflection.get(x, y);
            Vec3 tinted = mul(r, baseColor);
            Vec3 c = baseColor * (1.0 - m) + tinted * m;
            out.set(x, y, c);
        }
    }
    return out;
}

// ---------------------------
// Camera：viewport由两个三角形组成
// render()：分别渲染两块viewport三角形纹理，再合成输出到PPM
// ---------------------------
struct Camera {
    Vec3 origin;
    Triangle vpA; // viewport triangle A
    Triangle vpB; // viewport triangle B
    int width = 800;
    int height = 600;

    Image renderViewportTriangle(
        const std::vector<Triangle>& scene,
        const Triangle& viewportTri,
        const RenderConfig& cfg
    ) const {
        Image img(width, height, cfg.envColor);

        // 先收集候选面片（可能投射到该viewport三角形）
        std::vector<const Triangle*> candidates = collectCandidateTriangles(scene, origin, viewportTri);

        std::sort(candidates.begin(), candidates.end(), [&](const Triangle* a, const Triangle* b) {
            double da = length(a->centroid() - origin);
            double db = length(b->centroid() - origin);
            return da > db; // 远->近
        });

        std::vector<int> stack;
        for (const Triangle* tptr : candidates) {
            if (!tptr) continue;

            // 估计该面片投影到viewport的像素面积：通过一次“投影+裁剪”的面积计算获得
            Vec2 dstUV[3];
            bool ok = true;
            for (int i = 0; i < 3; ++i) {
                Vec2 uv;
                if (!projectPointToViewportUV(origin, tptr->p[i], viewportTri, uv, nullptr, nullptr)) {
                    ok = false;
                    break;
                }
                dstUV[i] = uv;
            }
            if (!ok) continue;

            std::vector<UVPairVertex> poly(3);
            for (int i = 0; i < 3; ++i) {
                poly[i].dstUV = dstUV[i];
                poly[i].srcUV = tptr->uv[i];
            }
            std::vector<UVPairVertex> clipped = clipPolygonToTriangle(
                std::move(poly),
                viewportTri.uv[0], viewportTri.uv[1], viewportTri.uv[2]
            );
            if (clipped.size() < 3) continue;

            double areaUV = std::fabs(polygonAreaUV(clipped));
            double areaPx = areaUV * (double)width * (double)height;
            if (areaPx < 0.5) continue;

            int triRes = (int)std::lround(std::sqrt(areaPx) * 1.0);
            triRes = clampi(triRes, cfg.minTriTexRes, cfg.maxTriTexRes);

            Image triTex = renderTriangleWithTriangle(
                scene,
                origin,
                *tptr,
                triRes, triRes,
                /*depth=*/0,
                /*estimatedAreaPx=*/areaPx,
                stack,
                cfg
            );

            // 投影并写入viewport图像（近处覆盖远处）
            rasterizeWarpPolygonFan(img, clipped, triTex, /*overwrite=*/true);
        }

        // 将viewport三角形外的像素清为黑（严格按“两个三角形viewport”）
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Vec2 uv((double)x / (width - 1), (double)y / (height - 1));
                bool inside = pointInTriangle2D(uv, viewportTri.uv[0], viewportTri.uv[1], viewportTri.uv[2], 1e-10);
                if (!inside) img.set(x, y, {0,0,0});
            }
        }

        return img;
    }

    Image render(const std::vector<Triangle>& scene, const RenderConfig& cfg) const {
        Image a = renderViewportTriangle(scene, vpA, cfg);
        Image b = renderViewportTriangle(scene, vpB, cfg);

        // 合成：由于a/b在各自三角形外为黑，直接相加即可得到完整画面
        Image out(width, height, {0,0,0});
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Vec3 c = a.get(x, y) + b.get(x, y);
                // clamp
                c.x = clampd(c.x, 0.0, 1.0);
                c.y = clampd(c.y, 0.0, 1.0);
                c.z = clampd(c.z, 0.0, 1.0);
                out.set(x, y, c);
            }
        }
        return out;
    }
};

// ---------------------------
// 场景构建：用四边形拆成两个三角形
// UV采用(0,0)-(1,1)映射（每个quad面独立）
// ---------------------------
static inline void addQuad(
    std::vector<Triangle>& scene,
    int& nextId,
    const Vec3& p00, const Vec3& p10, const Vec3& p11, const Vec3& p01,
    Material* mat
) {
    Triangle t0;
    t0.id = nextId++;
    t0.p[0] = p00; t0.p[1] = p10; t0.p[2] = p01;
    t0.uv[0] = {0,0}; t0.uv[1] = {1,0}; t0.uv[2] = {0,1};
    t0.mat = mat;

    Triangle t1;
    t1.id = nextId++;
    t1.p[0] = p10; t1.p[1] = p11; t1.p[2] = p01;
    t1.uv[0] = {1,0}; t1.uv[1] = {1,1}; t1.uv[2] = {0,1};
    t1.mat = mat;

    scene.push_back(t0);
    scene.push_back(t1);
}

static inline void addBox(
    std::vector<Triangle>& scene,
    int& nextId,
    const Vec3& bmin,
    const Vec3& bmax,
    Material* mat
) {
    // 6 faces: +X -X +Y -Y +Z -Z
    Vec3 p000(bmin.x, bmin.y, bmin.z);
    Vec3 p001(bmin.x, bmin.y, bmax.z);
    Vec3 p010(bmin.x, bmax.y, bmin.z);
    Vec3 p011(bmin.x, bmax.y, bmax.z);
    Vec3 p100(bmax.x, bmin.y, bmin.z);
    Vec3 p101(bmax.x, bmin.y, bmax.z);
    Vec3 p110(bmax.x, bmax.y, bmin.z);
    Vec3 p111(bmax.x, bmax.y, bmax.z);

    // -X face: p000 p001 p011 p010
    addQuad(scene, nextId, p000, p001, p011, p010, mat);
    // +X face: p100 p110 p111 p101
    addQuad(scene, nextId, p100, p110, p111, p101, mat);
    // -Y face (bottom): p000 p100 p101 p001
    addQuad(scene, nextId, p000, p100, p101, p001, mat);
    // +Y face (top): p010 p011 p111 p110
    addQuad(scene, nextId, p010, p011, p111, p110, mat);
    // -Z face: p000 p010 p110 p100
    addQuad(scene, nextId, p000, p010, p110, p100, mat);
    // +Z face: p001 p101 p111 p011
    addQuad(scene, nextId, p001, p101, p111, p011, mat);
}

int main() {
    try {
        // ---------------------------
        // 材质
        // ---------------------------
        Material matWhite; matWhite.type = MaterialType::Diffuse; matWhite.albedo = {0.85, 0.85, 0.85};
        Material matRed;   matRed.type   = MaterialType::Diffuse; matRed.albedo   = {0.85, 0.25, 0.25};
        Material matGreen; matGreen.type = MaterialType::Diffuse; matGreen.albedo = {0.25, 0.85, 0.25};

        Material matBlueMetal;   matBlueMetal.type   = MaterialType::Reflective; matBlueMetal.albedo   = {0.25, 0.45, 1.0};  matBlueMetal.metalness   = 0.90;
        Material matYellowMetal; matYellowMetal.type = MaterialType::Reflective; matYellowMetal.albedo = {1.0, 0.92, 0.20}; matYellowMetal.metalness = 0.92;

        // ---------------------------
        // Cornell Box 场景（前方开口）
        // 坐标：房间x∈[-1,1], y∈[0,2], z∈[0,2]
        // 相机在 z<0 处朝 +z 看
        // ---------------------------
        std::vector<Triangle> scene;
        int nextId = 1;

        // floor y=0
        addQuad(scene, nextId,
            Vec3(-1,0,0), Vec3( 1,0,0), Vec3( 1,0,2), Vec3(-1,0,2),
            &matWhite
        );
        // ceiling y=2
        addQuad(scene, nextId,
            Vec3(-1,2,0), Vec3(-1,2,2), Vec3( 1,2,2), Vec3( 1,2,0),
            &matWhite
        );
        // back wall z=2
        addQuad(scene, nextId,
            Vec3(-1,0,2), Vec3( 1,0,2), Vec3( 1,2,2), Vec3(-1,2,2),
            &matWhite
        );
        // left wall x=-1 (red)
        addQuad(scene, nextId,
            Vec3(-1,0,0), Vec3(-1,0,2), Vec3(-1,2,2), Vec3(-1,2,0),
            &matRed
        );
        // right wall x=1 (green)
        addQuad(scene, nextId,
            Vec3( 1,0,0), Vec3( 1,2,0), Vec3( 1,2,2), Vec3( 1,0,2),
            &matGreen
        );

        // ---------------------------
        // 两个金属盒子（蓝、黄）
        // ---------------------------
        addBox(scene, nextId, Vec3(-0.70, 0.0, 0.80), Vec3(-0.15, 0.60, 1.30), &matBlueMetal);
        addBox(scene, nextId, Vec3( 0.15, 0.0, 1.00), Vec3( 0.70, 1.10, 1.65), &matYellowMetal);

        // ---------------------------
        // Camera + viewport(两个三角形)
        // viewport平面取 z = -2.0，矩形中心 y=1.0
        // ---------------------------
        Camera cam;
        cam.width = 900;
        cam.height = 650;
        cam.origin = Vec3(0.0, 1.0, -3.0);

        double vpZ = -2.0;
        double aspect = (double)cam.width / (double)cam.height;
        double vpH = 1.6;              // viewport世界高度
        double vpW = vpH * aspect;     // 对应宽度

        Vec3 center(0.0, 1.0, vpZ);
        Vec3 TL(center.x - vpW * 0.5, center.y - vpH * 0.5, vpZ);
        Vec3 TR(center.x + vpW * 0.5, center.y - vpH * 0.5, vpZ);
        Vec3 BL(center.x - vpW * 0.5, center.y + vpH * 0.5, vpZ);
        Vec3 BR(center.x + vpW * 0.5, center.y + vpH * 0.5, vpZ);

        // viewport UV：u右 v下（0为上）
        // 三角A：TL(0,0), TR(1,0), BL(0,1)
        cam.vpA.id = -100;
        cam.vpA.p[0] = TL; cam.vpA.p[1] = TR; cam.vpA.p[2] = BL;
        cam.vpA.uv[0] = {0,0}; cam.vpA.uv[1] = {1,0}; cam.vpA.uv[2] = {0,1};
        cam.vpA.mat = nullptr; // viewport不是场景面片，不参与材质

        // 三角B：TR(1,0), BR(1,1), BL(0,1)
        cam.vpB.id = -101;
        cam.vpB.p[0] = TR; cam.vpB.p[1] = BR; cam.vpB.p[2] = BL;
        cam.vpB.uv[0] = {1,0}; cam.vpB.uv[1] = {1,1}; cam.vpB.uv[2] = {0,1};
        cam.vpB.mat = nullptr;

        RenderConfig cfg;
        cfg.maxDepth = 4;
        cfg.minAreaPxToRecurse = 6.0;
        cfg.maxTriTexRes = 256;
        cfg.minTriTexRes = 16;
        cfg.envColor = {0.06, 0.07, 0.09};

        Image img = cam.render(scene, cfg);
        img.writePPM("output_rt10.ppm");

        std::cerr << "Done. Wrote output_rt10.ppm\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}
