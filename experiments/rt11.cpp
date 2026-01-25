#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>
#include <array>

// ==================== 基础数学结构 ====================

struct Vec3 {
    double x, y, z;
    
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator*=(double t) { x *= t; y *= t; z *= t; return *this; }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    double lengthSquared() const { return x * x + y * y + z * z; }
    Vec3 normalized() const {
        double len = length();
        if (len < 1e-10) return Vec3(0, 0, 0);
        return *this / len;
    }
    
    // 关于平面对称点
    static Vec3 reflect(const Vec3& point, const Vec3& planePoint, const Vec3& planeNormal) {
        Vec3 n = planeNormal.normalized();
        double d = (point - planePoint).dot(n);
        return point - n * (2 * d);
    }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// 颜色结构（RGB，0-1范围）
struct Color {
    double r, g, b;
    
    Color() : r(0), g(0), b(0) {}
    Color(double r, double g, double b) : r(r), g(g), b(b) {}
    
    Color operator+(const Color& c) const { return Color(r + c.r, g + c.g, b + c.b); }
    Color operator*(double t) const { return Color(r * t, g * t, b * t); }
    Color operator*(const Color& c) const { return Color(r * c.r, g * c.g, b * c.b); }
    Color& operator+=(const Color& c) { r += c.r; g += c.g; b += c.b; return *this; }
    
    Color clamped() const {
        return Color(
            std::max(0.0, std::min(1.0, r)),
            std::max(0.0, std::min(1.0, g)),
            std::max(0.0, std::min(1.0, b))
        );
    }
    
    static Color lerp(const Color& a, const Color& b, double t) {
        return Color(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t);
    }
};

// ==================== 纹理类 ====================

class Texture {
public:
    int width, height;
    std::vector<Color> pixels;
    
    Texture() : width(0), height(0) {}
    Texture(int w, int h) : width(w), height(h), pixels(w * h) {}
    Texture(int w, int h, const Color& fillColor) : width(w), height(h), pixels(w * h, fillColor) {}
    
    Color getPixel(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return Color(0, 0, 0);
        return pixels[y * width + x];
    }
    
    void setPixel(int x, int y, const Color& c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        pixels[y * width + x] = c;
    }
    
    // 双线性插值采样（UV坐标0-1）
    Color sample(double u, double v) const {
        if (width == 0 || height == 0) return Color(0, 0, 0);
        
        u = std::max(0.0, std::min(1.0, u));
        v = std::max(0.0, std::min(1.0, v));
        
        double fx = u * (width - 1);
        double fy = v * (height - 1);
        
        int x0 = (int)fx;
        int y0 = (int)fy;
        int x1 = std::min(x0 + 1, width - 1);
        int y1 = std::min(y0 + 1, height - 1);
        
        double tx = fx - x0;
        double ty = fy - y0;
        
        Color c00 = getPixel(x0, y0);
        Color c10 = getPixel(x1, y0);
        Color c01 = getPixel(x0, y1);
        Color c11 = getPixel(x1, y1);
        
        Color c0 = Color::lerp(c00, c10, tx);
        Color c1 = Color::lerp(c01, c11, tx);
        
        return Color::lerp(c0, c1, ty);
    }
    
    // 在指定像素位置混合颜色（用于叠加）
    void blendPixel(int x, int y, const Color& c, double alpha = 1.0) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        Color existing = getPixel(x, y);
        Color blended = Color(
            existing.r * (1 - alpha) + c.r * alpha,
            existing.g * (1 - alpha) + c.g * alpha,
            existing.b * (1 - alpha) + c.b * alpha
        );
        setPixel(x, y, blended);
    }
};

// ==================== 材质类型 ====================

enum class MaterialType {
    Diffuse,      // 漫反射
    Reflective,   // 镜面反射
    Emissive      // 发光体
};

struct Material {
    MaterialType type;
    Color baseColor;
    double reflectivity;  // 0-1，反射率
    
    Material() : type(MaterialType::Diffuse), baseColor(0.8, 0.8, 0.8), reflectivity(0) {}
    Material(MaterialType t, const Color& c, double refl = 0.9) 
        : type(t), baseColor(c), reflectivity(refl) {}
};

// ==================== 三角形类 ====================

class Triangle {
public:
    Vec3 v0, v1, v2;  // 三个顶点
    Vec3 normal;       // 法向量
    Material material;
    Texture* texture;  // 可选的纹理
    
    // UV坐标
    Vec3 uv0, uv1, uv2;
    
    Triangle() : texture(nullptr) {
        uv0 = Vec3(0, 0, 0);
        uv1 = Vec3(1, 0, 0);
        uv2 = Vec3(0, 1, 0);
    }
    
    Triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Material& mat = Material())
        : v0(a), v1(b), v2(c), material(mat), texture(nullptr) {
        normal = (v1 - v0).cross(v2 - v0).normalized();
        uv0 = Vec3(0, 0, 0);
        uv1 = Vec3(1, 0, 0);
        uv2 = Vec3(0, 1, 0);
    }
    
    void setUV(const Vec3& u0, const Vec3& u1, const Vec3& u2) {
        uv0 = u0; uv1 = u1; uv2 = u2;
    }
    
    // 获取三角形中心
    Vec3 center() const {
        return (v0 + v1 + v2) / 3.0;
    }
    
    // 获取三角形面积
    double area() const {
        return (v1 - v0).cross(v2 - v0).length() * 0.5;
    }
    
    // 计算重心坐标
    bool barycentricCoords(const Vec3& p, double& u, double& v, double& w) const {
        Vec3 v0v1 = v1 - v0;
        Vec3 v0v2 = v2 - v0;
        Vec3 v0p = p - v0;
        
        double d00 = v0v1.dot(v0v1);
        double d01 = v0v1.dot(v0v2);
        double d11 = v0v2.dot(v0v2);
        double d20 = v0p.dot(v0v1);
        double d21 = v0p.dot(v0v2);
        
        double denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < 1e-10) return false;
        
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1.0 - v - w;
        
        return true;
    }
    
    // 从重心坐标获取UV
    Vec3 getUVFromBarycentric(double u, double v, double w) const {
        return Vec3(
            u * uv0.x + v * uv1.x + w * uv2.x,
            u * uv0.y + v * uv1.y + w * uv2.y,
            0
        );
    }
    
    // 射线与三角形相交测试（Möller–Trumbore算法）
    bool rayIntersect(const Vec3& origin, const Vec3& dir, double& t, double& u, double& v) const {
        const double EPSILON = 1e-8;
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        Vec3 h = dir.cross(edge2);
        double a = edge1.dot(h);
        
        if (std::abs(a) < EPSILON) return false;
        
        double f = 1.0 / a;
        Vec3 s = origin - v0;
        u = f * s.dot(h);
        
        if (u < 0.0 || u > 1.0) return false;
        
        Vec3 q = s.cross(edge1);
        v = f * dir.dot(q);
        
        if (v < 0.0 || u + v > 1.0) return false;
        
        t = f * edge2.dot(q);
        
        return t > EPSILON;
    }
    
    // 获取指定UV处的颜色
    Color getColorAtUV(double u, double v) const {
        if (texture != nullptr) {
            return texture->sample(u, v);
        }
        return material.baseColor;
    }
};

// ==================== 视锥体类（由原点和三角形viewport组成） ====================

class Frustum {
public:
    Vec3 apex;           // 视锥体顶点（相机原点）
    Triangle viewport;   // 底面三角形
    
    // 四个平面（侧面3个 + 底面1个）
    // 每个平面由法向量和平面上的一点定义
    std::array<Vec3, 4> planeNormals;
    std::array<Vec3, 4> planePoints;
    
    Frustum() {}
    
    Frustum(const Vec3& origin, const Triangle& vp) : apex(origin), viewport(vp) {
        buildPlanes();
    }
    
    void buildPlanes() {
        // 三个侧面
        Vec3 edge0 = viewport.v1 - viewport.v0;
        Vec3 edge1 = viewport.v2 - viewport.v1;
        Vec3 edge2 = viewport.v0 - viewport.v2;
        
        Vec3 toApex0 = apex - viewport.v0;
        Vec3 toApex1 = apex - viewport.v1;
        Vec3 toApex2 = apex - viewport.v2;
        
        // 侧面0: v0-v1-apex
        planeNormals[0] = edge0.cross(toApex0).normalized();
        planePoints[0] = viewport.v0;
        
        // 侧面1: v1-v2-apex
        planeNormals[1] = edge1.cross(toApex1).normalized();
        planePoints[1] = viewport.v1;
        
        // 侧面2: v2-v0-apex
        planeNormals[2] = edge2.cross(toApex2).normalized();
        planePoints[2] = viewport.v2;
        
        // 底面（viewport平面）
        planeNormals[3] = viewport.normal;
        planePoints[3] = viewport.v0;
        
        // 确保法向量朝向视锥体内部
        Vec3 center = (viewport.v0 + viewport.v1 + viewport.v2) / 3.0;
        Vec3 frustumCenter = (center + apex) / 2.0;
        
        for (int i = 0; i < 3; ++i) {
            Vec3 toCenter = frustumCenter - planePoints[i];
            if (planeNormals[i].dot(toCenter) < 0) {
                planeNormals[i] = planeNormals[i] * (-1);
            }
        }
        
        // 底面法向量应该指向apex方向
        Vec3 toApex = apex - planePoints[3];
        if (planeNormals[3].dot(toApex) < 0) {
            planeNormals[3] = planeNormals[3] * (-1);
        }
    }
    
    // 检测点是否在视锥体内
    bool containsPoint(const Vec3& p) const {
        for (int i = 0; i < 4; ++i) {
            Vec3 toPoint = p - planePoints[i];
            if (planeNormals[i].dot(toPoint) < -1e-6) {
                return false;
            }
        }
        return true;
    }
    
    // 检测三角形是否与视锥体相交或被包含
    bool intersectsTriangle(const Triangle& tri) const {
        // 简化检测：检查三角形的任意顶点是否在视锥体内，
        // 或者视锥体的任意边是否与三角形相交
        
        // 检查三角形顶点
        if (containsPoint(tri.v0) || containsPoint(tri.v1) || containsPoint(tri.v2)) {
            return true;
        }
        
        // 检查视锥体边与三角形的相交
        std::vector<std::pair<Vec3, Vec3>> frustumEdges = {
            {apex, viewport.v0},
            {apex, viewport.v1},
            {apex, viewport.v2},
            {viewport.v0, viewport.v1},
            {viewport.v1, viewport.v2},
            {viewport.v2, viewport.v0}
        };
        
        for (const auto& edge : frustumEdges) {
            Vec3 dir = edge.second - edge.first;
            double t, u, v;
            if (tri.rayIntersect(edge.first, dir.normalized(), t, u, v)) {
                if (t >= 0 && t <= dir.length()) {
                    return true;
                }
            }
        }
        
        // 检查三角形边与视锥体平面的相交
        std::vector<std::pair<Vec3, Vec3>> triEdges = {
            {tri.v0, tri.v1},
            {tri.v1, tri.v2},
            {tri.v2, tri.v0}
        };
        
        for (const auto& edge : triEdges) {
            Vec3 dir = edge.second - edge.first;
            double t, u, v;
            if (viewport.rayIntersect(edge.first, dir.normalized(), t, u, v)) {
                if (t >= 0 && t <= dir.length()) {
                    return true;
                }
            }
        }
        
        return false;
    }
};

// ==================== 前向声明 ====================

class Scene;
Texture renderTriangleWithTriangle(
    const std::vector<Triangle*>& triangles,
    const Vec3& cameraOrigin,
    const Triangle& currentTriangle,
    int estimatedWidth,
    int estimatedHeight,
    int depth,
    int maxDepth,
    const Scene& scene
);

// ==================== 场景类 ====================

class Scene {
public:
    std::vector<Triangle> triangles;
    Color ambientLight;
    Vec3 lightPosition;
    Color lightColor;
    
    Scene() : ambientLight(0.1, 0.1, 0.1), lightPosition(0, 1.9, 0), lightColor(1, 1, 1) {}
    
    void addTriangle(const Triangle& tri) {
        triangles.push_back(tri);
    }
    
    // 添加四边形（分解为两个三角形）
    void addQuad(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3, const Material& mat) {
        Triangle t1(v0, v1, v2, mat);
        Triangle t2(v0, v2, v3, mat);
        
        // 设置UV坐标
        t1.setUV(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0));
        t2.setUV(Vec3(0, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0));
        
        triangles.push_back(t1);
        triangles.push_back(t2);
    }
    
    // 添加盒子
    void addBox(const Vec3& minCorner, const Vec3& maxCorner, const Material& mat) {
        Vec3 v000 = minCorner;
        Vec3 v111 = maxCorner;
        Vec3 v100 = Vec3(maxCorner.x, minCorner.y, minCorner.z);
        Vec3 v010 = Vec3(minCorner.x, maxCorner.y, minCorner.z);
        Vec3 v001 = Vec3(minCorner.x, minCorner.y, maxCorner.z);
        Vec3 v110 = Vec3(maxCorner.x, maxCorner.y, minCorner.z);
        Vec3 v101 = Vec3(maxCorner.x, minCorner.y, maxCorner.z);
        Vec3 v011 = Vec3(minCorner.x, maxCorner.y, maxCorner.z);
        
        // 前面 (z = max)
        addQuad(v001, v101, v111, v011, mat);
        // 后面 (z = min)
        addQuad(v100, v000, v010, v110, mat);
        // 左面 (x = min)
        addQuad(v000, v001, v011, v010, mat);
        // 右面 (x = max)
        addQuad(v101, v100, v110, v111, mat);
        // 顶面 (y = max)
        addQuad(v010, v011, v111, v110, mat);
        // 底面 (y = min)
        addQuad(v000, v100, v101, v001, mat);
    }
    
    std::vector<Triangle*> getTrianglePointers() {
        std::vector<Triangle*> ptrs;
        for (auto& tri : triangles) {
            ptrs.push_back(&tri);
        }
        return ptrs;
    }
};

// ==================== 工具函数 ====================

// 将点从相机原点投影到目标平面上
Vec3 projectPointToPlane(const Vec3& origin, const Vec3& point, const Vec3& planePoint, const Vec3& planeNormal) {
    Vec3 dir = (point - origin).normalized();
    double denom = dir.dot(planeNormal);
    if (std::abs(denom) < 1e-10) return point; // 平行
    
    double t = (planePoint - origin).dot(planeNormal) / denom;
    return origin + dir * t;
}

// 将三角形的纹理变换并绘制到目标纹理上
void drawTransformedTriangle(
    Texture& destTexture,
    const Triangle& destTriangle,  // 目标三角形（viewport）
    const Triangle& srcTriangle,   // 源三角形
    const Texture& srcTexture,     // 源纹理
    const Vec3& cameraOrigin       // 相机原点
) {
    // 计算源三角形三个顶点投影到目标三角形平面上的位置
    Vec3 proj0 = projectPointToPlane(cameraOrigin, srcTriangle.v0, destTriangle.v0, destTriangle.normal);
    Vec3 proj1 = projectPointToPlane(cameraOrigin, srcTriangle.v1, destTriangle.v0, destTriangle.normal);
    Vec3 proj2 = projectPointToPlane(cameraOrigin, srcTriangle.v2, destTriangle.v0, destTriangle.normal);
    
    // 将投影点转换为目标三角形的重心坐标
    double u0, v0, w0, u1, v1, w1, u2, v2, w2;
    destTriangle.barycentricCoords(proj0, u0, v0, w0);
    destTriangle.barycentricCoords(proj1, u1, v1, w1);
    destTriangle.barycentricCoords(proj2, u2, v2, w2);
    
    // 转换为纹理坐标
    Vec3 texCoord0 = destTriangle.getUVFromBarycentric(u0, v0, w0);
    Vec3 texCoord1 = destTriangle.getUVFromBarycentric(u1, v1, w1);
    Vec3 texCoord2 = destTriangle.getUVFromBarycentric(u2, v2, w2);
    
    // 在目标纹理上栅格化源三角形
    // 计算边界框
    double minU = std::min({texCoord0.x, texCoord1.x, texCoord2.x});
    double maxU = std::max({texCoord0.x, texCoord1.x, texCoord2.x});
    double minV = std::min({texCoord0.y, texCoord1.y, texCoord2.y});
    double maxV = std::max({texCoord0.y, texCoord1.y, texCoord2.y});
    
    int startX = std::max(0, (int)(minU * destTexture.width) - 1);
    int endX = std::min(destTexture.width - 1, (int)(maxU * destTexture.width) + 1);
    int startY = std::max(0, (int)(minV * destTexture.height) - 1);
    int endY = std::min(destTexture.height - 1, (int)(maxV * destTexture.height) + 1);
    
    // 使用重心坐标进行纹理映射
    for (int py = startY; py <= endY; ++py) {
        for (int px = startX; px <= endX; ++px) {
            double u = (px + 0.5) / destTexture.width;
            double v = (py + 0.5) / destTexture.height;
            
            // 检查点是否在投影三角形内（使用2D重心坐标）
            Vec3 p(u, v, 0);
            Vec3 a(texCoord0.x, texCoord0.y, 0);
            Vec3 b(texCoord1.x, texCoord1.y, 0);
            Vec3 c(texCoord2.x, texCoord2.y, 0);
            
            Vec3 v0v = b - a;
            Vec3 v1v = c - a;
            Vec3 v2v = p - a;
            
            double d00 = v0v.x * v0v.x + v0v.y * v0v.y;
            double d01 = v0v.x * v1v.x + v0v.y * v1v.y;
            double d11 = v1v.x * v1v.x + v1v.y * v1v.y;
            double d20 = v2v.x * v0v.x + v2v.y * v0v.y;
            double d21 = v2v.x * v1v.x + v2v.y * v1v.y;
            
            double denom = d00 * d11 - d01 * d01;
            if (std::abs(denom) < 1e-10) continue;
            
            double baryV = (d11 * d20 - d01 * d21) / denom;
            double baryW = (d00 * d21 - d01 * d20) / denom;
            double baryU = 1.0 - baryV - baryW;
            
            // 检查是否在三角形内
            if (baryU >= -0.001 && baryV >= -0.001 && baryW >= -0.001) {
                // 计算源纹理的UV坐标
                double srcU = baryU * srcTriangle.uv0.x + baryV * srcTriangle.uv1.x + baryW * srcTriangle.uv2.x;
                double srcV = baryU * srcTriangle.uv0.y + baryV * srcTriangle.uv1.y + baryW * srcTriangle.uv2.y;
                
                Color srcColor = srcTexture.sample(srcU, srcV);
                destTexture.setPixel(px, py, srcColor);
            }
        }
    }
}

// 计算三角形投影到viewport上的近似像素面积
double estimateProjectedArea(const Triangle& tri, const Vec3& cameraOrigin, const Triangle& viewport, int viewportWidth, int viewportHeight) {
    Vec3 proj0 = projectPointToPlane(cameraOrigin, tri.v0, viewport.v0, viewport.normal);
    Vec3 proj1 = projectPointToPlane(cameraOrigin, tri.v1, viewport.v0, viewport.normal);
    Vec3 proj2 = projectPointToPlane(cameraOrigin, tri.v2, viewport.v0, viewport.normal);
    
    double u0, v0, w0, u1, v1, w1, u2, v2, w2;
    viewport.barycentricCoords(proj0, u0, v0, w0);
    viewport.barycentricCoords(proj1, u1, v1, w1);
    viewport.barycentricCoords(proj2, u2, v2, w2);
    
    Vec3 tc0 = viewport.getUVFromBarycentric(u0, v0, w0);
    Vec3 tc1 = viewport.getUVFromBarycentric(u1, v1, w1);
    Vec3 tc2 = viewport.getUVFromBarycentric(u2, v2, w2);
    
    // 计算2D三角形面积
    double area = 0.5 * std::abs(
        (tc1.x - tc0.x) * (tc2.y - tc0.y) - (tc2.x - tc0.x) * (tc1.y - tc0.y)
    );
    
    return area * viewportWidth * viewportHeight;
}

// ==================== 核心渲染函数 ====================

Texture renderTriangleWithTriangle(
    const std::vector<Triangle*>& triangles,
    const Vec3& cameraOrigin,
    const Triangle& currentTriangle,
    int estimatedWidth,
    int estimatedHeight,
    int depth,
    int maxDepth,
    const Scene& scene
) {
    // 检查终止条件
    if (depth >= maxDepth || estimatedWidth < 2 || estimatedHeight < 2) {
        // 返回基础颜色纹理
        Texture result(std::max(1, estimatedWidth), std::max(1, estimatedHeight), currentTriangle.material.baseColor);
        return result;
    }
    
    // 如果是漫反射材质，直接返回基础颜色
    if (currentTriangle.material.type == MaterialType::Diffuse) {
        Texture result(estimatedWidth, estimatedHeight, currentTriangle.material.baseColor);
        
        // 添加简单的光照计算
        Vec3 toLight = (scene.lightPosition - currentTriangle.center()).normalized();
        double diffuse = std::max(0.0, currentTriangle.normal.dot(toLight));
        
        Color litColor = currentTriangle.material.baseColor * (0.3 + 0.7 * diffuse);
        for (int y = 0; y < estimatedHeight; ++y) {
            for (int x = 0; x < estimatedWidth; ++x) {
                result.setPixel(x, y, litColor);
            }
        }
        
        return result;
    }
    
    // 如果是发光材质，返回发光颜色
    if (currentTriangle.material.type == MaterialType::Emissive) {
        Texture result(estimatedWidth, estimatedHeight, currentTriangle.material.baseColor);
        return result;
    }
    
    // 镜面反射材质
    // 初始化当前面片的纹理（使用基础颜色）
    Texture currentTexture(estimatedWidth, estimatedHeight, currentTriangle.material.baseColor * 0.1);
    
    // 计算反射后的相机原点（关于当前面片对称）
    Vec3 reflectedOrigin = Vec3::reflect(cameraOrigin, currentTriangle.center(), currentTriangle.normal);
    
    // 构建视锥体
    Frustum frustum(reflectedOrigin, currentTriangle);
    
    // 收集与视锥体相交的三角形
    struct TriangleDistance {
        Triangle* tri;
        double distance;
        
        bool operator<(const TriangleDistance& other) const {
            return distance > other.distance; // 由远到近
        }
    };
    
    std::vector<TriangleDistance> intersectedTriangles;
    
    for (Triangle* tri : triangles) {
        // 跳过当前三角形
        if (tri->v0.x == currentTriangle.v0.x && tri->v0.y == currentTriangle.v0.y && tri->v0.z == currentTriangle.v0.z &&
            tri->v1.x == currentTriangle.v1.x && tri->v1.y == currentTriangle.v1.y && tri->v1.z == currentTriangle.v1.z &&
            tri->v2.x == currentTriangle.v2.x && tri->v2.y == currentTriangle.v2.y && tri->v2.z == currentTriangle.v2.z) {
            continue;
        }
        
        // 检查三角形是否在视锥体的"前方"（从反射原点看去）
        Vec3 toTriCenter = tri->center() - reflectedOrigin;
        Vec3 viewDir = (currentTriangle.center() - reflectedOrigin).normalized();
        
        if (toTriCenter.dot(viewDir) <= 0) {
            continue; // 三角形在视锥体后方
        }
        
        if (frustum.intersectsTriangle(*tri)) {
            double dist = (tri->center() - reflectedOrigin).length();
            intersectedTriangles.push_back({tri, dist});
        }
    }
    
    // 按距离排序（由远到近）
    std::sort(intersectedTriangles.begin(), intersectedTriangles.end());
    
    // 遍历并渲染每个相交的三角形
    for (const auto& td : intersectedTriangles) {
        Triangle* tri = td.tri;
        
        // 估计该三角形投影到当前面片上的像素大小
        double projectedArea = estimateProjectedArea(*tri, reflectedOrigin, currentTriangle, estimatedWidth, estimatedHeight);
        int subWidth = std::max(2, (int)std::sqrt(projectedArea));
        int subHeight = subWidth;
        
        // 递归获取该三角形的纹理
        Texture subTexture = renderTriangleWithTriangle(
            triangles, reflectedOrigin, *tri, subWidth, subHeight, depth + 1, maxDepth, scene
        );
        
        // 创建临时三角形用于绘制（带有获取的纹理）
        Triangle tempTri = *tri;
        tempTri.texture = &subTexture;
        
        // 将该三角形的纹理变换并绘制到当前纹理上
        drawTransformedTriangle(currentTexture, currentTriangle, *tri, subTexture, reflectedOrigin);
    }
    
    // 混合基础颜色和反射结果
    double refl = currentTriangle.material.reflectivity;
    for (int y = 0; y < estimatedHeight; ++y) {
        for (int x = 0; x < estimatedWidth; ++x) {
            Color reflected = currentTexture.getPixel(x, y);
            Color base = currentTriangle.material.baseColor * 0.1;
            Color mixed = Color(
                base.r * (1 - refl) + reflected.r * refl,
                base.g * (1 - refl) + reflected.g * refl,
                base.b * (1 - refl) + reflected.b * refl
            );
            currentTexture.setPixel(x, y, mixed);
        }
    }
    
    return currentTexture;
}

// ==================== 相机类 ====================

class Camera {
public:
    Vec3 position;
    Vec3 lookAt;
    Vec3 up;
    double fov;
    int width, height;
    
    // Viewport三角形
    Triangle viewportTri1, viewportTri2;
    
    Camera(const Vec3& pos, const Vec3& look, const Vec3& upDir, double fieldOfView, int w, int h)
        : position(pos), lookAt(look), up(upDir), fov(fieldOfView), width(w), height(h) {
        setupViewport();
    }
    
    void setupViewport() {
        Vec3 forward = (lookAt - position).normalized();
        Vec3 right = forward.cross(up).normalized();
        Vec3 upVec = right.cross(forward).normalized();
        
        double aspectRatio = (double)width / height;
        double halfHeight = std::tan(fov * 0.5 * M_PI / 180.0);
        double halfWidth = aspectRatio * halfHeight;
        
        // Viewport在相机前方单位距离处
        Vec3 center = position + forward;
        
        Vec3 bottomLeft = center - right * halfWidth - upVec * halfHeight;
        Vec3 bottomRight = center + right * halfWidth - upVec * halfHeight;
        Vec3 topLeft = center - right * halfWidth + upVec * halfHeight;
        Vec3 topRight = center + right * halfWidth + upVec * halfHeight;
        
        // 创建两个三角形组成viewport
        Material viewportMat(MaterialType::Diffuse, Color(0, 0, 0));
        viewportTri1 = Triangle(bottomLeft, bottomRight, topRight, viewportMat);
        viewportTri2 = Triangle(bottomLeft, topRight, topLeft, viewportMat);
        
        // 设置UV坐标（用于纹理映射到最终图像）
        viewportTri1.setUV(Vec3(0, 1, 0), Vec3(1, 1, 0), Vec3(1, 0, 0));  // 注意Y轴翻转
        viewportTri2.setUV(Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0, 0, 0));
    }
    
    void render(Scene& scene, const std::string& filename) {
        std::vector<Triangle*> triPtrs = scene.getTrianglePointers();
        int maxDepth = 4;
        
        std::cout << "Rendering viewport triangle 1..." << std::endl;
        Texture tex1 = renderTriangleWithTriangle(triPtrs, position, viewportTri1, width, height, 0, maxDepth, scene);
        
        std::cout << "Rendering viewport triangle 2..." << std::endl;
        Texture tex2 = renderTriangleWithTriangle(triPtrs, position, viewportTri2, width, height, 0, maxDepth, scene);
        
        // 合并两个三角形的纹理到最终图像
        Texture finalImage(width, height);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double u = (x + 0.5) / width;
                double v = (y + 0.5) / height;
                
                // 计算在viewport平面上的3D位置
                Vec3 forward = (lookAt - position).normalized();
                Vec3 right = forward.cross(up).normalized();
                Vec3 upVec = right.cross(forward).normalized();
                
                double aspectRatio = (double)width / height;
                double halfHeight = std::tan(fov * 0.5 * M_PI / 180.0);
                double halfWidth = aspectRatio * halfHeight;
                
                Vec3 center = position + forward;
                Vec3 point = center + right * (u * 2 - 1) * halfWidth + upVec * (1 - v * 2) * halfHeight;
                
                // 检查点在哪个三角形内
                double bu, bv, bw;
                if (viewportTri1.barycentricCoords(point, bu, bv, bw) && bu >= 0 && bv >= 0 && bw >= 0) {
                    Vec3 uv = viewportTri1.getUVFromBarycentric(bu, bv, bw);
                    finalImage.setPixel(x, y, tex1.sample(uv.x, uv.y));
                } else if (viewportTri2.barycentricCoords(point, bu, bv, bw) && bu >= 0 && bv >= 0 && bw >= 0) {
                    Vec3 uv = viewportTri2.getUVFromBarycentric(bu, bv, bw);
                    finalImage.setPixel(x, y, tex2.sample(uv.x, uv.y));
                }
            }
        }
        
        // 写入PPM文件
        writePPM(filename, finalImage);
        std::cout << "Image saved to " << filename << std::endl;
    }
    
    void writePPM(const std::string& filename, const Texture& image) {
        std::ofstream file(filename);
        file << "P3\n" << image.width << " " << image.height << "\n255\n";
        
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                Color c = image.getPixel(x, y).clamped();
                int r = (int)(c.r * 255);
                int g = (int)(c.g * 255);
                int b = (int)(c.b * 255);
                file << r << " " << g << " " << b << "\n";
            }
        }
        
        file.close();
    }
};

// ==================== 构建Cornell Box场景 ====================

void buildCornellBox(Scene& scene) {
    // Cornell Box尺寸：大约 [-1, 1] x [0, 2] x [-1, 1]
    double boxSize = 1.0;
    
    // 材质定义
    Material whiteDiffuse(MaterialType::Diffuse, Color(0.75, 0.75, 0.75));
    Material redDiffuse(MaterialType::Diffuse, Color(0.75, 0.15, 0.15));
    Material greenDiffuse(MaterialType::Diffuse, Color(0.15, 0.75, 0.15));
    Material blueReflective(MaterialType::Reflective, Color(0.2, 0.4, 0.8), 0.85);
    Material yellowReflective(MaterialType::Reflective, Color(0.8, 0.7, 0.2), 0.85);
    Material lightEmissive(MaterialType::Emissive, Color(1.0, 0.95, 0.9));
    
    // 地板（白色）
    scene.addQuad(
        Vec3(-boxSize, 0, -boxSize),
        Vec3(boxSize, 0, -boxSize),
        Vec3(boxSize, 0, boxSize),
        Vec3(-boxSize, 0, boxSize),
        whiteDiffuse
    );
    
    // 天花板（白色）
    scene.addQuad(
        Vec3(-boxSize, 2 * boxSize, boxSize),
        Vec3(boxSize, 2 * boxSize, boxSize),
        Vec3(boxSize, 2 * boxSize, -boxSize),
        Vec3(-boxSize, 2 * boxSize, -boxSize),
        whiteDiffuse
    );
    
    // 后墙（白色）
    scene.addQuad(
        Vec3(-boxSize, 0, -boxSize),
        Vec3(-boxSize, 2 * boxSize, -boxSize),
        Vec3(boxSize, 2 * boxSize, -boxSize),
        Vec3(boxSize, 0, -boxSize),
        whiteDiffuse
    );
    
    // 左墙（红色）
    scene.addQuad(
        Vec3(-boxSize, 0, boxSize),
        Vec3(-boxSize, 2 * boxSize, boxSize),
        Vec3(-boxSize, 2 * boxSize, -boxSize),
        Vec3(-boxSize, 0, -boxSize),
        redDiffuse
    );
    
    // 右墙（绿色）
    scene.addQuad(
        Vec3(boxSize, 0, -boxSize),
        Vec3(boxSize, 2 * boxSize, -boxSize),
        Vec3(boxSize, 2 * boxSize, boxSize),
        Vec3(boxSize, 0, boxSize),
        greenDiffuse
    );
    
    // 顶部灯光（发光面板）
    scene.addQuad(
        Vec3(-0.25, 1.99, -0.25),
        Vec3(0.25, 1.99, -0.25),
        Vec3(0.25, 1.99, 0.25),
        Vec3(-0.25, 1.99, 0.25),
        lightEmissive
    );
    
    // 蓝色金属盒子（左侧，稍微旋转）
    double boxHeight1 = 0.6;
    double boxWidth1 = 0.35;
    Vec3 box1Center(-0.4, boxHeight1 / 2, -0.2);
    
    // 简化：使用轴对齐的盒子
    scene.addBox(
        Vec3(box1Center.x - boxWidth1/2, 0, box1Center.z - boxWidth1/2),
        Vec3(box1Center.x + boxWidth1/2, boxHeight1, box1Center.z + boxWidth1/2),
        blueReflective
    );
    
    // 黄色金属盒子（右侧，更高一些）
    double boxHeight2 = 0.9;
    double boxWidth2 = 0.35;
    Vec3 box2Center(0.35, boxHeight2 / 2, 0.15);
    
    scene.addBox(
        Vec3(box2Center.x - boxWidth2/2, 0, box2Center.z - boxWidth2/2),
        Vec3(box2Center.x + boxWidth2/2, boxHeight2, box2Center.z + boxWidth2/2),
        yellowReflective
    );
    
    // 设置光源
    scene.lightPosition = Vec3(0, 1.95, 0);
    scene.lightColor = Color(1, 1, 1);
    scene.ambientLight = Color(0.05, 0.05, 0.05);
}

// ==================== 主函数 ====================

int main() {
    std::cout << "=== Panel-based Cornell Box Renderer ===" << std::endl;
    std::cout << "Building scene..." << std::endl;
    
    Scene scene;
    buildCornellBox(scene);
    
    std::cout << "Scene contains " << scene.triangles.size() << " triangles." << std::endl;
    
    // 设置相机
    Vec3 cameraPos(0, 1, 2.5);
    Vec3 lookAt(0, 1, 0);
    Vec3 up(0, 1, 0);
    
    int imageWidth = 400;
    int imageHeight = 400;
    double fov = 50.0;
    
    Camera camera(cameraPos, lookAt, up, fov, imageWidth, imageHeight);
    
    std::cout << "Starting render at " << imageWidth << "x" << imageHeight << "..." << std::endl;
    
    camera.render(scene, "cornell_box.ppm");
    
    std::cout << "Render complete!" << std::endl;
    
    return 0;
}
