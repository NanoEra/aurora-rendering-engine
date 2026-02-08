/**
 * @file aabb.cpp
 * @brief Implementation of Axis-Aligned Bounding Box
 */

#include <are/geometry/aabb.h>
#include <are/raytracer/ray.h>
#include <are/core/logger.h>
#include <glm/glm.hpp>
#include <algorithm>
#include <limits>

namespace are {

AABB::AABB()
    : min_(std::numeric_limits<float>::max())
    , max_(std::numeric_limits<float>::lowest()) {
}

AABB::AABB(const Vec3& min, const Vec3& max)
    : min_(min)
    , max_(max) {
}

AABB::AABB(const Vec3& point)
    : min_(point)
    , max_(point) {
}

bool AABB::is_valid() const {
    return min_.x <= max_.x && min_.y <= max_.y && min_.z <= max_.z;
}

Vec3 AABB::center() const {
    return (min_ + max_) * 0.5f;
}

Vec3 AABB::size() const {
    return max_ - min_;
}

Real AABB::surface_area() const {
    Vec3 d = size();
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

Real AABB::volume() const {
    Vec3 d = size();
    return d.x * d.y * d.z;
}

int AABB::longest_axis() const {
    Vec3 d = size();
    if (d.x > d.y && d.x > d.z) {
        return 0;
    } else if (d.y > d.z) {
        return 1;
    } else {
        return 2;
    }
}

void AABB::expand(const Vec3& point) {
    min_ = glm::min(min_, point);
    max_ = glm::max(max_, point);
}

void AABB::expand(const AABB& other) {
    if (other.is_valid()) {
        min_ = glm::min(min_, other.min_);
        max_ = glm::max(max_, other.max_);
    }
}

bool AABB::contains(const Vec3& point) const {
    return point.x >= min_.x && point.x <= max_.x &&
           point.y >= min_.y && point.y <= max_.y &&
           point.z >= min_.z && point.z <= max_.z;
}

bool AABB::intersects(const AABB& other) const {
    return min_.x <= other.max_.x && max_.x >= other.min_.x &&
           min_.y <= other.max_.y && max_.y >= other.min_.y &&
           min_.z <= other.max_.z && max_.z >= other.min_.z;
}

bool AABB::intersect_ray(const Ray& ray, Real& t_min_out, Real& t_max_out) const {
    // Slab method for ray-AABB intersection
    // Reference: "An Efficient and Robust Ray-Box Intersection Algorithm" by Williams et al.
    
    Real t_min = ray.t_min_;
    Real t_max = ray.t_max_;

    for (int i = 0; i < 3; ++i) {
        Real inv_d = 1.0f / ray.direction_[i];
        Real t0 = (min_[i] - ray.origin_[i]) * inv_d;
        Real t1 = (max_[i] - ray.origin_[i]) * inv_d;

        if (inv_d < 0.0f) {
            std::swap(t0, t1);
        }

        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;

        if (t_max < t_min) {
            return false;
        }
    }

    t_min_out = t_min;
    t_max_out = t_max;
    return true;
}

AABB AABB::merge(const AABB& a, const AABB& b) {
    if (!a.is_valid()) return b;
    if (!b.is_valid()) return a;
    
    return AABB(
        glm::min(a.min_, b.min_),
        glm::max(a.max_, b.max_)
    );
}

AABB AABB::invalid() {
    return AABB();
}

} // namespace are
