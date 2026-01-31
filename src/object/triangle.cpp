#include <object/triangle.h>
#include <basic/math.h>
#include <stdexcept>
#include <cmath>

namespace are {

// 构造函数实现
Triangle::Triangle(const Point3 &Q, const Vec3 &u, const Vec3 &v, Material *material, Texture *texture)
    : material_(material), texture_(texture), Q(Q), u(u), v(v) {
    
    // 检查 material 指针的合法性
    if (material_ == nullptr) {
        throw std::invalid_argument("Material pointer cannot be null");
    }
    
    // 检查 texture 指针的合法性
    if (texture_ == nullptr) {
        throw std::invalid_argument("Texture pointer cannot be null");
    }
    
    // 检查向量 u 是否为零向量
    if (u.near_zero()) {
        throw std::invalid_argument("Edge vector u cannot be zero vector");
    }
    
    // 检查向量 v 是否为零向量
    if (v.near_zero()) {
        throw std::invalid_argument("Edge vector v cannot be zero vector");
    }
    
    // 检查 u 和 v 是否共线（叉积为零向量则共线，无法构成三角形）
    Vec3 cross_product = u.cross(v);
    if (cross_product.near_zero()) {
        throw std::invalid_argument("Edge vectors u and v cannot be collinear");
    }
    
    // 生成三角形的三个顶点
    // 顶点1: Q
    // 顶点2: Q + u
    // 顶点3: Q + v
    vertices_.reserve(3);
    vertices_.push_back(Q);
    vertices_.push_back(Q + u);
    vertices_.push_back(Q + v);
}

// 检查点是否在三角形内部
bool Triangle::point_in(const Point3 &point) const {
    // 使用重心坐标法判断点是否在三角形内
    // 计算向量
    Vec3 v0 = v;
    Vec3 v1 = u;
    Vec3 v2 = point - Q;
    
    // 计算点积
    double dot00 = v0.dot(v0);
    double dot01 = v0.dot(v1);
    double dot02 = v0.dot(v2);
    double dot11 = v1.dot(v1);
    double dot12 = v1.dot(v2);
    
    // 计算重心坐标
    double inv_denom = dot00 * dot11 - dot01 * dot01;
    
    // 检查分母是否为零（退化三角形）
    if (std::abs(inv_denom) < GEOMETRY_EPSILON) {
        return false;
    }
    
    inv_denom = 1.0 / inv_denom;
    double alpha = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    double beta = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    
    // 检查点是否在三角形内（包括边界）
    return (alpha >= -GEOMETRY_EPSILON) && 
           (beta >= -GEOMETRY_EPSILON) && 
           (alpha + beta <= 1.0 + GEOMETRY_EPSILON);
}

// 光线与三角形相交检测
bool Triangle::intersect_ray(const Ray &ray, Point3 &hit_point) const {
    // 使用 Möller–Trumbore 算法
    Vec3 edge1 = u;
    Vec3 edge2 = v;
    Vec3 h = ray.D.cross(edge2);
    double a = edge1.dot(h);
    
    // 如果 a 接近 0，光线与三角形平行
    if (std::abs(a) < GEOMETRY_EPSILON) {
        return false;
    }
    
    double f = 1.0 / a;
    Vec3 s = ray.Q - Q;
    double alpha = f * s.dot(h);
    
    // 检查 alpha 是否在有效范围内
    if (alpha < -GEOMETRY_EPSILON || alpha > 1.0 + GEOMETRY_EPSILON) {
        return false;
    }
    
    Vec3 q = s.cross(edge1);
    double beta = f * ray.D.dot(q);
    
    // 检查 beta 和 alpha + beta 是否在有效范围内
    if (beta < -GEOMETRY_EPSILON || alpha + beta > 1.0 + GEOMETRY_EPSILON) {
        return false;
    }
    
    // 计算 t 值（光线参数）
    double t = f * edge2.dot(q);
    
    // 检查交点是否在光线正方向上
    if (t > GEOMETRY_EPSILON) {
        hit_point = ray.Q + t * ray.D;
        return true;
    }
    
    return false;
}

// 获取三角形顶点
const std::vector<Point3> &Triangle::get_vertices() const {
    return vertices_;
}

}
