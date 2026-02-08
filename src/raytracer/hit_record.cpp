/**
 * @file hit_record.cpp
 * @brief Implementation of HitRecord structure
 */

#include <are/raytracer/hit_record.h>
#include <glm/glm.hpp>
#include <limits>

namespace are {

HitRecord::HitRecord()
    : position_(0.0f)
    , normal_(0.0f, 1.0f, 0.0f)
    , texcoord_(0.0f)
    , tangent_(1.0f, 0.0f, 0.0f)
    , t_(std::numeric_limits<Real>::max())
    , material_(are_invalid_handle)
    , triangle_index_(0)
    , front_face_(true) {
}

void HitRecord::set_face_normal(const Vec3& ray_direction, const Vec3& outward_normal) {
    front_face_ = glm::dot(ray_direction, outward_normal) < 0.0f;
    normal_ = front_face_ ? outward_normal : -outward_normal;
}

bool HitRecord::is_valid() const {
    return t_ > 0.0f && t_ < std::numeric_limits<Real>::max();
}

} // namespace are
