/**
 * @file triangle.cpp
 * @brief Implementation of triangle primitive
 */

#include <are/geometry/triangle.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>
#include <glm/geometric.hpp>

namespace are {

Triangle::Triangle()
    : v0_()
    , v1_()
    , v2_()
    , material_(are_invalid_handle) {
}

Triangle::Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2,
                  MaterialHandle material)
    : v0_(v0)
    , v1_(v1)
    , v2_(v2)
    , material_(material) {
}

Vec3 Triangle::centroid() const {
    return (v0_.position_ + v1_.position_ + v2_.position_) / 3.0f;
}

Vec3 Triangle::normal() const {
    Vec3 edge1 = v1_.position_ - v0_.position_;
    Vec3 edge2 = v2_.position_ - v0_.position_;
    return glm::normalize(glm::cross(edge1, edge2));
}

Real Triangle::area() const {
    Vec3 edge1 = v1_.position_ - v0_.position_;
    Vec3 edge2 = v2_.position_ - v0_.position_;
    return glm::length(glm::cross(edge1, edge2)) * 0.5f;
}

AABB Triangle::compute_aabb() const {
    AABB box(v0_.position_);
    box.expand(v1_.position_);
    box.expand(v2_.position_);
    return box;
}

bool Triangle::intersect(const Ray& ray, HitRecord& hit) const {
    // Möller-Trumbore ray-triangle intersection algorithm
    const Vec3& v0 = v0_.position_;
    const Vec3& v1 = v1_.position_;
    const Vec3& v2 = v2_.position_;
    
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    
    Vec3 h = glm::cross(ray.direction_, edge2);
    Real a = glm::dot(edge1, h);
    
    // Check if ray is parallel to triangle
    if (std::abs(a) < are_epsilon) {
        return false;
    }
    
    Real f = 1.0f / a;
    Vec3 s = ray.origin_ - v0;
    Real u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    Vec3 q = glm::cross(s, edge1);
    Real v = f * glm::dot(ray.direction_, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    Real t = f * glm::dot(edge2, q);
    
    if (!ray.is_valid_t(t) || t >= hit.t_) {
        return false;
    }
    
    // Compute hit record
    hit.t_ = t;
    hit.position_ = ray.at(t);
    
    // Interpolate vertex attributes using barycentric coordinates
    Real w = 1.0f - u - v;
    hit.normal_ = glm::normalize(w * v0_.normal_ + u * v1_.normal_ + v * v2_.normal_);
    hit.texcoord_ = w * v0_.texcoord_ + u * v1_.texcoord_ + v * v2_.texcoord_;
    hit.tangent_ = glm::normalize(w * v0_.tangent_ + u * v1_.tangent_ + v * v2_.tangent_);
    
    hit.material_ = material_;
    hit.set_face_normal(ray.direction_, hit.normal_);
    
    return true;
}

bool Triangle::intersect_fast(const Ray& ray, Real t_max) const {
    // Fast ray-triangle intersection (no hit record)
    const Vec3& v0 = v0_.position_;
    const Vec3& v1 = v1_.position_;
    const Vec3& v2 = v2_.position_;
    
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    
    Vec3 h = glm::cross(ray.direction_, edge2);
    Real a = glm::dot(edge1, h);
    
    if (std::abs(a) < are_epsilon) {
        return false;
    }
    
    Real f = 1.0f / a;
    Vec3 s = ray.origin_ - v0;
    Real u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    Vec3 q = glm::cross(s, edge1);
    Real v = f * glm::dot(ray.direction_, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    Real t = f * glm::dot(edge2, q);
    
    return ray.is_valid_t(t) && t < t_max;
}

} // namespace are
