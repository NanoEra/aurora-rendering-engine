#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <algorithm>

// ===================== 数学工具 =========================

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double v) : x(v), y(v), z(v) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double v) : x(v), y(v) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

inline Vec3 operator-(const Vec3& v) {
	return Vec3(-v.x, -v.y, -v.z);
}

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline Vec3 operator*(const Vec3& a, double s) {
    return Vec3(a.x * s, a.y * s, a.z * s);
}
inline Vec3 operator*(double s, const Vec3& a) {
    return Vec3(a.x * s, a.y * s, a.z * s);
}
inline Vec3 operator/(const Vec3& a, double s) {
    return Vec3(a.x / s, a.y / s, a.z / s);
}
inline Vec3 operator*(const Vec3& a, const Vec3& b) {
    return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline Vec3& operator+=(Vec3& a, const Vec3& b) {
    a.x += b.x; a.y += b.y; a.z += b.z; return a;
}

inline Vec2 operator+(const Vec2& a, const Vec2& b) {
    return Vec2(a.x + b.x, a.y + b.y);
}
inline Vec2 operator-(const Vec2& a, const Vec2& b) {
    return Vec2(a.x - b.x, a.y - b.y);
}
inline Vec2 operator*(const Vec2& a, double s) {
    return Vec2(a.x * s, a.y * s);
}
inline Vec2 operator*(double s, const Vec2& a) {
    return Vec2(a.x * s, a.y * s);
}
inline Vec2 operator/(const Vec2& a, double s) {
    return Vec2(a.x / s, a.y / s);
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}
inline double length(const Vec3& v) {
    return std::sqrt(dot(v, v));
}
inline Vec3 normalize(const Vec3& v) {
    double len = length(v);
    if (len <= 1e-12) return Vec3(0);
    return v / len;
}

// 反射方向：v - 2*(v·n)*n，v 为指向表面的入射方向
inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - n * (2.0 * dot(v, n));
}

inline Vec3 clamp01(const Vec3& c) {
    return Vec3(
        std::min(1.0, std::max(0.0, c.x)),
        std::min(1.0, std::max(0.0, c.y)),
        std::min(1.0, std::max(0.0, c.z))
    );
}

// ===================== 材质 & 三角形 =========================

struct Material {
    Vec3 baseColor;
    bool mirror;
    double reflectivity; // [0,1]
};

struct Triangle {
    Vec3 v[3];   // 顶点
    Vec3 normal; // 面法线
    Vec2 uv[3];  // 顶点局部纹理坐标
    const Material* material;

    Triangle() : material(nullptr) {
        v[0] = v[1] = v[2] = Vec3(0);
        uv[0] = uv[1] = uv[2] = Vec2(0);
        normal = Vec3(0,1,0);
    }

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Material* m) {
        v[0] = a; v[1] = b; v[2] = c;
        material = m;
        normal = normalize(cross(v[1] - v[0], v[2] - v[0]));
        // 简单局部 UV：三角形自身局部坐标
        uv[0] = Vec2(0.0, 0.0);
        uv[1] = Vec2(1.0, 0.0);
        uv[2] = Vec2(0.0, 1.0);
    }
};

// ===================== 相机 =========================

struct Camera {
    Vec3 pos;
    Vec3 target;
    Vec3 up;
    double fov;

    Vec3 forward;
    Vec3 right;
    Vec3 camUp;
    double invTanHalfFov;

    Camera(const Vec3& p, const Vec3& t, const Vec3& u, double fovDeg)
        : pos(p), target(t), up(u)
    {
        forward = normalize(target - pos);
        right = normalize(cross(forward, up));
        camUp = cross(right, forward);
        fov = fovDeg * M_PI / 180.0;
        invTanHalfFov = 1.0 / std::tan(fov * 0.5);
    }

    Vec3 worldToCamera(const Vec3& p) const {
        Vec3 v = p - pos;
        return Vec3(
            dot(v, right),
            dot(v, camUp),
            dot(v, forward)
        );
    }

    bool projectToScreen(
        const Vec3& pWorld,
        int width, int height,
        double& outX, double& outY, double& outZ
    ) const {
        Vec3 pc = worldToCamera(pWorld);
        if (pc.z <= 1e-6) return false;

        double x_ndc = (pc.x * invTanHalfFov) / pc.z;
        double y_ndc = (pc.y * invTanHalfFov) / pc.z;

        outX = (x_ndc * 0.5 + 0.5) * (double)(width);
        outY = (1.0 - (y_ndc * 0.5 + 0.5)) * (double)(height);
        outZ = pc.z;
        return true;
    }
};

// ===================== 场景 =========================

struct Scene {
    std::vector<Triangle> tris;
    std::vector<Material> materials;
};

static const double INF_D = std::numeric_limits<double>::infinity();

// ========== 世界空间三角形重心坐标 ==========

bool barycentricWorld(
    const Vec3& p,
    const Triangle& tri,
    double& w0, double& w1, double& w2
) {
    Vec3 v0 = tri.v[0];
    Vec3 v1 = tri.v[1];
    Vec3 v2 = tri.v[2];
    Vec3 v0v1 = v1 - v0;
    Vec3 v0v2 = v2 - v0;
    Vec3 v0p  = p  - v0;

    double d00 = dot(v0v1, v0v1);
    double d01 = dot(v0v1, v0v2);
    double d11 = dot(v0v2, v0v2);
    double d20 = dot(v0p,  v0v1);
    double d21 = dot(v0p,  v0v2);

    double denom = d00 * d11 - d01 * d01;
    if (std::fabs(denom) < 1e-12) {
        w0 = w1 = w2 = 1.0/3.0;
        return false;
    }

    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    w0 = u; w1 = v; w2 = w;
    return true;
}

// ========== 2D 屏幕三角形重心坐标 ==========

bool barycentric2D(
    double px, double py,
    double x0, double y0,
    double x1, double y1,
    double x2, double y2,
    double& u, double& v, double& w
) {
    double den = ( (y1 - y2)*(x0 - x2) + (x2 - x1)*(y0 - y2) );
    if (std::fabs(den) < 1e-12) return false;
    u = ((y1 - y2)*(px - x2) + (x2 - x1)*(py - y2)) / den;
    v = ((y2 - y0)*(px - x2) + (x0 - x2)*(py - y2)) / den;
    w = 1.0 - u - v;
    return (u >= 0.0 && v >= 0.0 && w >= 0.0);
}

// ========== 射线与三角形相交（用于反射） ==========

bool rayIntersectTriangle(
    const Vec3& orig,
    const Vec3& dir,
    const Triangle& tri,
    double& t, double& u, double& v
) {
    const double EPSILON = 1e-8;
    Vec3 v0v1 = tri.v[1] - tri.v[0];
    Vec3 v0v2 = tri.v[2] - tri.v[0];
    Vec3 pvec = cross(dir, v0v2);
    double det = dot(v0v1, pvec);

    if (std::fabs(det) < EPSILON) return false;
    double invDet = 1.0 / det;

    Vec3 tvec = orig - tri.v[0];
    u = dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    Vec3 qvec = cross(tvec, v0v1);
    v = dot(dir, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;

    t = dot(v0v2, qvec) * invDet;
    if (t < EPSILON) return false;

    return true;
}

bool traceScene(
    const Scene& scene,
    const Vec3& orig,
    const Vec3& dir,
    int& outTriId,
    double& outT,
    double& outU,
    double& outV
) {
    outTriId = -1;
    outT = INF_D;
    double u, v;
    for (size_t i = 0; i < scene.tris.size(); ++i) {
        double tTmp, uTmp, vTmp;
        if (rayIntersectTriangle(orig, dir, scene.tris[i], tTmp, uTmp, vTmp)) {
            if (tTmp < outT) {
                outT = tTmp;
                outTriId = (int)i;
                outU = uTmp;
                outV = vTmp;
            }
        }
    }
    return (outTriId >= 0);
}

// ===================== Texture2D（离屏纹理） =========================

struct Texture2D {
    int w, h;
    std::vector<Vec3> data;

    Texture2D() : w(0), h(0) {}
    Texture2D(int w_, int h_) : w(w_), h(h_), data(w_*h_, Vec3(0)) {}

    void resize(int w_, int h_) {
        w = w_; h = h_;
        data.assign(w*h, Vec3(0));
    }

    void clear(const Vec3& c) {
        std::fill(data.begin(), data.end(), c);
    }

    Vec3 get(int x, int y) const {
        if (w <= 0 || h <= 0) return Vec3(0);
        x = std::max(0, std::min(w-1, x));
        y = std::max(0, std::min(h-1, y));
        return data[y * w + x];
    }

    void set(int x, int y, const Vec3& c) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        data[y * w + x] = c;
    }

    Vec3 sampleUV(const Vec2& uv) const {
        if (w <= 0 || h <= 0) return Vec3(0);
        double u = std::min(1.0, std::max(0.0, uv.x));
        double v = std::min(1.0, std::max(0.0, uv.y));
        int x = (int)(u * (w - 1) + 0.5);
        int y = (int)(v * (h - 1) + 0.5);
        return get(x, y);
    }

    void writeUV(const Vec2& uv, const Vec3& c) {
        if (w <= 0 || h <= 0) return;
        double u = uv.x;
        double v = uv.y;
        // 略宽松判断，允许轻微的数值误差
        if (u < -0.01 || u > 1.01 || v < -0.01 || v > 1.01) return;
        u = std::min(1.0, std::max(0.0, u));
        v = std::min(1.0, std::max(0.0, v));
        int x = (int)(u * (w - 1) + 0.5);
        int y = (int)(v * (h - 1) + 0.5);
        set(x, y, c);
    }
};

// ===================== 2D 仿射变换（UV 映射） =========================

struct Affine2D {
    // [ uA ]   [m00 m01 m02] [ uB ]
    // [ vA ] = [m10 m11 m12] [ vB ]
    //                         [ 1  ]
    double m00, m01, m02;
    double m10, m11, m12;
};

Vec2 applyAffine(const Affine2D& A, const Vec2& uvB) {
    double uA = A.m00 * uvB.x + A.m01 * uvB.y + A.m02;
    double vA = A.m10 * uvB.x + A.m11 * uvB.y + A.m12;
    return Vec2(uA, vA);
}

// 解 3x3 线性方程组 M * x = b
bool solve3x3(double M[3][3], double b[3], double x[3]) {
    double A[3][4];
    for (int i = 0; i < 3; ++i) {
        A[i][0] = M[i][0];
        A[i][1] = M[i][1];
        A[i][2] = M[i][2];
        A[i][3] = b[i];
    }

    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int r = col+1; r < 3; ++r) {
            if (std::fabs(A[r][col]) > std::fabs(A[pivot][col]))
                pivot = r;
        }
        if (std::fabs(A[pivot][col]) < 1e-12) return false;
        if (pivot != col) {
            for (int c = 0; c < 4; ++c) std::swap(A[pivot][c], A[col][c]);
        }

        double div = A[col][col];
        for (int c = col; c < 4; ++c) A[col][c] /= div;

        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            double factor = A[r][col];
            for (int c = col; c < 4; ++c) {
                A[r][c] -= factor * A[col][c];
            }
        }
    }

    for (int i = 0; i < 3; ++i) x[i] = A[i][3];
    return true;
}

// 计算 B 的 UV -> A 的 UV 的仿射矩阵
Affine2D computeUVMappingAffine(
    const Triangle& triA,
    const Triangle& triB
) {
    Affine2D A;

    Vec2 uvB[3];
    Vec2 uvA[3];

    Vec3 nA = triA.normal;
    Vec3 p0A = triA.v[0];

    for (int j = 0; j < 3; ++j) {
        Vec3 pb = triB.v[j];
        Vec3 diff = pb - p0A;
        double dist = dot(diff, nA);
        Vec3 pref = pb - 2.0 * dist * nA; // pb 关于 A 平面镜像后的点

        double w0, w1, w2;
        barycentricWorld(pref, triA, w0, w1, w2);
        Vec2 uvOnA = triA.uv[0] * w0 + triA.uv[1] * w1 + triA.uv[2] * w2;

        uvA[j] = uvOnA;
        uvB[j] = triB.uv[j];
    }

    double MB[3][3];
    double U[3], V[3];
    for (int j = 0; j < 3; ++j) {
        MB[j][0] = uvB[j].x;
        MB[j][1] = uvB[j].y;
        MB[j][2] = 1.0;
        U[j] = uvA[j].x;
        V[j] = uvA[j].y;
    }

    double solU[3], solV[3];
    if (!solve3x3(MB, U, solU)) {
        A.m00 = 1; A.m01 = 0; A.m02 = 0;
        A.m10 = 0; A.m11 = 1; A.m12 = 0;
        return A;
    }
    if (!solve3x3(MB, V, solV)) {
        A.m00 = 1; A.m01 = 0; A.m02 = 0;
        A.m10 = 0; A.m11 = 1; A.m12 = 0;
        return A;
    }

    A.m00 = solU[0]; A.m01 = solU[1]; A.m02 = solU[2];
    A.m10 = solV[0]; A.m11 = solV[1]; A.m12 = solV[2];
    return A;
}

// ===================== 递归反射着色 =========================

// 语义：
//  - currentTriId: 当前点所在三角形 ID（A）
//  - uvCurrent:    当前点在 A 上的 UV
//  - P:            当前点世界坐标
//  - n:            当前点法线
//  - mat:          当前材质
//  - viewDir:      从 P 指向“观察者（上一 bounce 或相机）”的方向
Vec3 shadeRecursive(
    const Scene& scene,
    const Camera& cam,
    std::vector<Texture2D>& triTextures,
    int currentTriId,
    const Vec2& uvCurrent,
    const Vec3& P,
    const Vec3& n,
    const Material& mat,
    const Vec3& viewDir,
    int depth,
    int maxDepth
) {
    // 先从当前三角形纹理采样，再融合基础颜色
    Vec3 texColor = mat.baseColor;
    if (currentTriId >= 0 && currentTriId < (int)triTextures.size()) {
        Vec3 sampled = triTextures[currentTriId].sampleUV(uvCurrent);
        texColor = sampled * 0.7 + mat.baseColor * 0.3;
    }

    if (!mat.mirror || depth >= maxDepth) {
        return texColor;
    }

    // 镜面反射：inDir 是指向表面的方向，viewDir 指向观察者
    Vec3 inDir = -viewDir;
    Vec3 reflDir = normalize(reflect(inDir, n));

    int hitTriId;
    double tHit, uBary, vBary;
    if (!traceScene(scene, P + n * 1e-4, reflDir, hitTriId, tHit, uBary, vBary)) {
        // 环境（看不到任何三角形）
        Vec3 envColor(0.0, 0.0, 0.0);
        return texColor * (1.0 - mat.reflectivity) + envColor * mat.reflectivity;
    }

    const Triangle& triB = scene.tris[hitTriId];
    const Material& matB = *triB.material;

    Vec3 hitPos = P + reflDir * tHit;
    Vec3 nB = triB.normal;
    Vec3 viewDirNext = -reflDir; // 在 B 看 A 的视线方向

    // 计算 B 上的 UV
    double w0B, w1B, w2B;
    barycentricWorld(hitPos, triB, w0B, w1B, w2B);
    Vec2 uvB = triB.uv[0] * w0B + triB.uv[1] * w1B + triB.uv[2] * w2B;

    // 递归计算 B 的颜色（在其自身纹理空间继续反射）
    Vec3 colorB = shadeRecursive(
        scene, cam, triTextures,
        hitTriId, uvB,
        hitPos, nB, matB,
        viewDirNext,
        depth + 1, maxDepth
    );

    // 利用 uvA_fromB 把 B 的颜色写入 A 的纹理
    if (currentTriId >= 0 && currentTriId < (int)scene.tris.size()) {
        const Triangle& triA = scene.tris[currentTriId];
        Affine2D mapBA = computeUVMappingAffine(triA, triB);
        Vec2 uvA_fromB = applyAffine(mapBA, uvB);
        triTextures[currentTriId].writeUV(uvA_fromB, colorB);
    }

    // 当前点最终颜色：自身贴图/基色 + 镜面反射
    Vec3 finalColor =
        texColor * (1.0 - mat.reflectivity) +
        colorB   * mat.reflectivity;

    return finalColor;
}

// ===================== 屏幕三角形 & 相机 viewport 映射 =========================

struct ScreenTriangle {
    double x0, y0, z0;
    double x1, y1, z1;
    double x2, y2, z2;
    int triId;
};

// 将屏幕坐标 (px,py) 映射到相机 viewport 的两个三角形之一，得到 UV 和 camIdx
bool mapScreenToCamUV(
    double px, double py,
    int width, int height,
    Vec2& uvCam,
    int& camIdx
) {
    double u, v, w;

    // TriCam0: (0,0)-(W,0)-(0,H)，UV: (0,0),(1,0),(0,1)
    if (barycentric2D(px, py,
                      0.0, 0.0,
                      (double)width, 0.0,
                      0.0, (double)height,
                      u, v, w)) {
        Vec2 uv0(0.0, 0.0);
        Vec2 uv1(1.0, 0.0);
        Vec2 uv2(0.0, 1.0);
        uvCam = uv0 * u + uv1 * v + uv2 * w;
        camIdx = 0;
        return true;
    }

    // TriCam1: (W,0)-(W,H)-(0,H)，UV: (1,0),(1,1),(0,1)
    if (barycentric2D(px, py,
                      (double)width, 0.0,
                      (double)width, (double)height,
                      0.0,           (double)height,
                      u, v, w)) {
        Vec2 uv0(1.0, 0.0);
        Vec2 uv1(1.0, 1.0);
        Vec2 uv2(0.0, 1.0);
        uvCam = uv0 * u + uv1 * v + uv2 * w;
        camIdx = 1;
        return true;
    }

    return false;
}

// ===================== 场景 → CameraTexture 渲染 =========================

void renderSceneToCameraTextures(
    const Scene& scene,
    const Camera& cam,
    int width,
    int height,
    std::vector<Texture2D>& triTextures,
    Texture2D camTextures[2]
) {
    std::vector<double> depthBuffer(width * height, INF_D);

    // 1. 投影所有三角形为屏幕三角形
    std::vector<ScreenTriangle> screenTris;
    screenTris.reserve(scene.tris.size());

    for (size_t i = 0; i < scene.tris.size(); ++i) {
        const Triangle& tri = scene.tris[i];

        double x0,y0,z0;
        double x1,y1,z1;
        double x2,y2,z2;
        bool ok0 = cam.projectToScreen(tri.v[0], width, height, x0, y0, z0);
        bool ok1 = cam.projectToScreen(tri.v[1], width, height, x1, y1, z1);
        bool ok2 = cam.projectToScreen(tri.v[2], width, height, x2, y2, z2);

        if (!ok0 && !ok1 && !ok2) continue;
        int behind = (!ok0) + (!ok1) + (!ok2);
        if (behind >= 2) continue;

        ScreenTriangle st;
        st.x0 = x0; st.y0 = y0; st.z0 = z0;
        st.x1 = x1; st.y1 = y1; st.z1 = z1;
        st.x2 = x2; st.y2 = y2; st.z2 = z2;
        st.triId = (int)i;
        screenTris.push_back(st);
    }

    // 2. 按平均深度从远到近排序（覆盖关系）
    std::sort(screenTris.begin(), screenTris.end(),
        [](const ScreenTriangle& a, const ScreenTriangle& b) {
            double za = (a.z0 + a.z1 + a.z2) / 3.0;
            double zb = (b.z0 + b.z1 + b.z2) / 3.0;
            return za > zb; // 大 z 在远处
        }
    );

    const int maxReflectionDepth = 3;

    // 3. 遍历每个屏幕三角形，进行光栅化和递归反射着色，写入 CameraTexture
    for (const ScreenTriangle& st : screenTris) {
        const Triangle& tri = scene.tris[st.triId];
        const Material& mat = *tri.material;

        int minX = (int)std::floor(std::min({ st.x0, st.x1, st.x2 }));
        int maxX = (int)std::ceil (std::max({ st.x0, st.x1, st.x2 }));
        int minY = (int)std::floor(std::min({ st.y0, st.y1, st.y2 }));
        int maxY = (int)std::ceil (std::max({ st.y0, st.y1, st.y2 }));

        minX = std::max(minX, 0);
        maxX = std::min(maxX, width  - 1);
        minY = std::max(minY, 0);
        maxY = std::min(maxY, height - 1);

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                double px = x + 0.5;
                double py = y + 0.5;

                double u,v,w;
                if (!barycentric2D(
                        px, py,
                        st.x0, st.y0,
                        st.x1, st.y1,
                        st.x2, st.y2,
                        u,v,w)) {
                    continue;
                }

                double z = st.z0 * u + st.z1 * v + st.z2 * w;
                int idx = y * width + x;
                if (z >= depthBuffer[idx]) continue;
                depthBuffer[idx] = z;

                // 世界坐标、法线、当前三角形 UV
                Vec3 P =
                    tri.v[0] * u +
                    tri.v[1] * v +
                    tri.v[2] * w;

                Vec3 n = tri.normal;

                Vec2 uvCurrent =
                    tri.uv[0] * u +
                    tri.uv[1] * v +
                    tri.uv[2] * w;

                // 从 P 看向相机的方向
                Vec3 viewDir0 = normalize(cam.pos - P);

                // 递归着色（以当前三角形为 viewport）
                Vec3 color = shadeRecursive(
                    scene, cam, triTextures,
                    st.triId, uvCurrent,
                    P, n, mat,
                    viewDir0,
                    0, maxReflectionDepth
                );

                // 将颜色写入相机 viewport 的两个三角形纹理
                Vec2 uvCam;
                int camIdx;
                if (mapScreenToCamUV(px, py, width, height, uvCam, camIdx)) {
                    camTextures[camIdx].writeUV(uvCam, color);
                }
            }
        }
    }
}

// ===================== 构建 Cornell Box 场景 =========================

void buildCornellBox(Scene& scene) {
    scene.materials.clear();
    scene.tris.clear();

    Material redWall;
    redWall.baseColor = Vec3(0.75, 0.15, 0.15);
    redWall.mirror = false;
    redWall.reflectivity = 0.0;

    Material greenWall;
    greenWall.baseColor = Vec3(0.15, 0.75, 0.15);
    greenWall.mirror = false;
    greenWall.reflectivity = 0.0;

    Material whiteWall;
    whiteWall.baseColor = Vec3(0.75, 0.75, 0.75);
    whiteWall.mirror = false;
    whiteWall.reflectivity = 0.0;

    Material blueMetal;
    blueMetal.baseColor = Vec3(0.2, 0.4, 0.9);
    blueMetal.mirror = true;
    blueMetal.reflectivity = 0.9;

    Material yellowMetal;
    yellowMetal.baseColor = Vec3(0.9, 0.8, 0.2);
    yellowMetal.mirror = true;
    yellowMetal.reflectivity = 0.9;

    scene.materials.push_back(redWall);    // 0
    scene.materials.push_back(greenWall);  // 1
    scene.materials.push_back(whiteWall);  // 2
    scene.materials.push_back(blueMetal);  // 3
    scene.materials.push_back(yellowMetal);// 4

    const Material* mRed    = &scene.materials[0];
    const Material* mGreen  = &scene.materials[1];
    const Material* mWhite  = &scene.materials[2];
    const Material* mBlue   = &scene.materials[3];
    const Material* mYellow = &scene.materials[4];

    double roomSize = 2.0;
    double half = roomSize * 0.5;
    double floorY = 0.0;
    double ceilY  = roomSize;
    double backZ  = -roomSize;
    double frontZ = 0.0;

    // 地板（white）
    {
        Vec3 v0(-half, floorY, frontZ);
        Vec3 v1( half, floorY, frontZ);
        Vec3 v2( half, floorY, backZ);
        Vec3 v3(-half, floorY, backZ);
        scene.tris.emplace_back(v0, v1, v2, mWhite);
        scene.tris.emplace_back(v0, v2, v3, mWhite);
    }

    // 天花板（white）
    {
        Vec3 v0(-half, ceilY, backZ);
        Vec3 v1( half, ceilY, backZ);
        Vec3 v2( half, ceilY, frontZ);
        Vec3 v3(-half, ceilY, frontZ);
        scene.tris.emplace_back(v0, v1, v2, mWhite);
        scene.tris.emplace_back(v0, v2, v3, mWhite);
    }

    // 左墙（red）
    {
        Vec3 v0(-half, floorY, backZ);
        Vec3 v1(-half, floorY, frontZ);
        Vec3 v2(-half, ceilY, frontZ);
        Vec3 v3(-half, ceilY, backZ);
        scene.tris.emplace_back(v0, v1, v2, mRed);
        scene.tris.emplace_back(v0, v2, v3, mRed);
    }

    // 右墙（green）
    {
        Vec3 v0(half, floorY, frontZ);
        Vec3 v1(half, floorY, backZ);
        Vec3 v2(half, ceilY, backZ);
        Vec3 v3(half, ceilY, frontZ);
        scene.tris.emplace_back(v0, v1, v2, mGreen);
        scene.tris.emplace_back(v0, v2, v3, mGreen);
    }

    // 后墙（white）
    {
        Vec3 v0(-half, floorY, backZ);
        Vec3 v1( half, floorY, backZ);
        Vec3 v2( half, ceilY,  backZ);
        Vec3 v3(-half, ceilY,  backZ);
        scene.tris.emplace_back(v0, v1, v2, mWhite);
        scene.tris.emplace_back(v0, v2, v3, mWhite);
    }

    // 蓝色金属盒（左）——矮盒
    {
        double bx0 = -0.6;
        double bx1 = -0.2;
        double bz0 = -1.0;
        double bz1 = -0.4;
        double h   = 0.7;

        Vec3 t0(bx0, floorY + h, bz0);
        Vec3 t1(bx1, floorY + h, bz0);
        Vec3 t2(bx1, floorY + h, bz1);
        Vec3 t3(bx0, floorY + h, bz1);
        scene.tris.emplace_back(t0, t1, t2, mBlue);
        scene.tris.emplace_back(t0, t2, t3, mBlue);

        Vec3 f0(bx0, floorY,     bz1);
        Vec3 f1(bx1, floorY,     bz1);
        Vec3 f2(bx1, floorY + h, bz1);
        Vec3 f3(bx0, floorY + h, bz1);
        scene.tris.emplace_back(f0, f1, f2, mBlue);
        scene.tris.emplace_back(f0, f2, f3, mBlue);

        Vec3 bk0(bx1, floorY,     bz0);
        Vec3 bk1(bx0, floorY,     bz0);
        Vec3 bk2(bx0, floorY + h, bz0);
        Vec3 bk3(bx1, floorY + h, bz0);
        scene.tris.emplace_back(bk0, bk1, bk2, mBlue);
        scene.tris.emplace_back(bk0, bk2, bk3, mBlue);

        Vec3 l0(bx0, floorY,     bz0);
        Vec3 l1(bx0, floorY,     bz1);
        Vec3 l2(bx0, floorY + h, bz1);
        Vec3 l3(bx0, floorY + h, bz0);
        scene.tris.emplace_back(l0, l1, l2, mBlue);
        scene.tris.emplace_back(l0, l2, l3, mBlue);

        Vec3 r0(bx1, floorY,     bz1);
        Vec3 r1(bx1, floorY,     bz0);
        Vec3 r2(bx1, floorY + h, bz0);
        Vec3 r3(bx1, floorY + h, bz1);
        scene.tris.emplace_back(r0, r1, r2, mBlue);
        scene.tris.emplace_back(r0, r2, r3, mBlue);
    }

    // 黄色金属盒（右）——高盒
    {
        double bx0 = 0.1;
        double bx1 = 0.6;
        double bz0 = -0.8;
        double bz1 = -0.1;
        double h   = 1.2;

        Vec3 t0(bx0, floorY + h, bz0);
        Vec3 t1(bx1, floorY + h, bz0);
        Vec3 t2(bx1, floorY + h, bz1);
        Vec3 t3(bx0, floorY + h, bz1);
        scene.tris.emplace_back(t0, t1, t2, mYellow);
        scene.tris.emplace_back(t0, t2, t3, mYellow);

        Vec3 f0(bx0, floorY,     bz1);
        Vec3 f1(bx1, floorY,     bz1);
        Vec3 f2(bx1, floorY + h, bz1);
        Vec3 f3(bx0, floorY + h, bz1);
        scene.tris.emplace_back(f0, f1, f2, mYellow);
        scene.tris.emplace_back(f0, f2, f3, mYellow);

        Vec3 bk0(bx1, floorY,     bz0);
        Vec3 bk1(bx0, floorY,     bz0);
        Vec3 bk2(bx0, floorY + h, bz0);
        Vec3 bk3(bx1, floorY + h, bz0);
        scene.tris.emplace_back(bk0, bk1, bk2, mYellow);
        scene.tris.emplace_back(bk0, bk2, bk3, mYellow);

        Vec3 l0(bx0, floorY,     bz0);
        Vec3 l1(bx0, floorY,     bz1);
        Vec3 l2(bx0, floorY + h, bz1);
        Vec3 l3(bx0, floorY + h, bz0);
        scene.tris.emplace_back(l0, l1, l2, mYellow);
        scene.tris.emplace_back(l0, l2, l3, mYellow);

        Vec3 r0(bx1, floorY,     bz1);
        Vec3 r1(bx1, floorY,     bz0);
        Vec3 r2(bx1, floorY + h, bz0);
        Vec3 r3(bx1, floorY + h, bz1);
        scene.tris.emplace_back(r0, r1, r2, mYellow);
        scene.tris.emplace_back(r0, r2, r3, mYellow);
    }
}

// ===================== 写 PPM =========================

void writePPM(
    const std::string& filename,
    const std::vector<Vec3>& colorBuffer,
    int width,
    int height
) {
    std::ofstream ofs(filename);
    ofs << "P3\n" << width << " " << height << "\n255\n";
    for (int i = 0; i < width*height; ++i) {
        Vec3 c = clamp01(colorBuffer[i]);
        int r = (int)(std::pow(c.x, 1.0/2.2) * 255.0 + 0.5);
        int g = (int)(std::pow(c.y, 1.0/2.2) * 255.0 + 0.5);
        int b = (int)(std::pow(c.z, 1.0/2.2) * 255.0 + 0.5);
        r = std::max(0, std::min(255, r));
        g = std::max(0, std::min(255, g));
        b = std::max(0, std::min(255, b));
        ofs << r << " " << g << " " << b << "\n";
    }
    ofs.close();
}

// ===================== main =========================

int main() {
    int width  = 800;
    int height = 800;

    Scene scene;
    buildCornellBox(scene);

    Camera cam(
        Vec3(0.0, 1.0, 2.5),  // 相机位置
        Vec3(0.0, 1.0, -1.0), // 观察目标
        Vec3(0.0, 1.0, 0.0),  // 上方向
        45.0                  // 垂直 FOV
    );

    // 为每个三角形分配离屏纹理
    const int TRI_TEX_RES = 256;
    std::vector<Texture2D> triTextures(scene.tris.size());
    for (size_t i = 0; i < scene.tris.size(); ++i) {
        triTextures[i].resize(TRI_TEX_RES, TRI_TEX_RES);
        const Material* m = scene.tris[i].material;
        Vec3 baseColor = m ? m->baseColor : Vec3(0.5, 0.5, 0.5);
        triTextures[i].clear(baseColor);
    }

    // 相机 viewport 的两个三角形各自的离屏纹理
    Texture2D camTextures[2];
    camTextures[0].resize(width, height);
    camTextures[1].resize(width, height);
    camTextures[0].clear(Vec3(0.0, 0.0, 0.0));
    camTextures[1].clear(Vec3(0.0, 0.0, 0.0));

    // 第一步：场景 → 相机 viewport 纹理（递归贴图）
    renderSceneToCameraTextures(scene, cam, width, height, triTextures, camTextures);

    // 第二步：相机 viewport 纹理 → 最终 PPM
    std::vector<Vec3> colorBuffer(width * height, Vec3(0.0, 0.0, 0.0));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double px = x + 0.5;
            double py = y + 0.5;

            Vec2 uvCam;
            int camIdx;
            if (!mapScreenToCamUV(px, py, width, height, uvCam, camIdx)) {
                continue;
            }
            Vec3 c = camTextures[camIdx].sampleUV(uvCam);
            colorBuffer[y * width + x] = c;
        }
    }

    writePPM("output.ppm", colorBuffer, width, height);
    std::cerr << "Render finished. Saved to output.ppm\n";
    return 0;
}
