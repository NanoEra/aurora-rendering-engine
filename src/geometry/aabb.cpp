/**
 * @file aabb.cpp
 * @brief Implementation of axis-aligned bounding box
 */

#include <are/geometry/aabb.h>
#include <are/raytracer/ray.h>
#include <limits>
#include <algorithm>

namespace are {

AABB::AABB() 
    : min_(std::numeric_limits<Real>::max())
    , max_(std::numeric_limits<Real>::lowest()) {
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
    min_ = glm::min(min_, other.min_);
    max_ = glm::max(max_, other.max_);
}

bool AABB::contains(const Vec3& point) const {
    return point.x >= min_.x && point.x <= max_.x &&
           point.y >= min_.y && point.y <= max_.y &&
           point.z >= min_.z && point.z <= max_.z;
}

bool AABB::intersects(const AABB& other) const {
    return (min_.x <= other.max_.x && max_.x >= other.min_.x) &&
           (min_.y <= other.max_.y && max_.y >= other.min_.y) &&
           (min_.z <= other.max_.z && max_.z >= other.min_.z);
}

bool AABB::intersect_ray(const Ray& ray, Real& t_min, Real& t_max) const {
    // Optimized ray-AABB intersection (slab method)
    Vec3 inv_dir = 1.0f / ray.direction_;
    Vec3 t0 = (min_ - ray.origin_) * inv_dir;
    Vec3 t1 = (max_ - ray.origin_) * inv_dir;
    
    Vec3 t_near = glm::min(t0, t1);
    Vec3 t_far = glm::max(t0, t1);
    
    t_min = glm::max(glm::max(t_near.x, t_near.y), t_near.z);
    t_max = glm::min(glm::min(t_far.x, t_far.y), t_far.z);
    
    return t_max >= t_min && t_max >= ray.t_min_;
}

AABB AABB::merge(const AABB& a, const AABB& b) {
    return AABB(
        glm::min(a.min_, b.min_),
        glm::max(a.max_, b.max_)
    );
}

AABB AABB::invalid() {
    return AABB();
}

} // namespace are
