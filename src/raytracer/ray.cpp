/**
 * @file ray.cpp
 * @brief Implementation of Ray structure
 */

#include <are/raytracer/ray.h>
#include <are/core/logger.h>
#include <glm/glm.hpp>

namespace are {

Ray::Ray()
    : origin_(0.0f)
    , direction_(0.0f, 0.0f, 1.0f)
    , t_min_(are_epsilon)
    , t_max_(1e30f) {
}

Ray::Ray(const Vec3& origin, const Vec3& direction, Real t_min, Real t_max)
    : origin_(origin)
    , t_min_(t_min)
    , t_max_(t_max) {
    // Normalize direction vector
    Real length = glm::length(direction);
    if (length < are_epsilon) {
        ARE_LOG_WARN("Ray: Direction vector has zero length, using default (0,0,1)");
        direction_ = Vec3(0.0f, 0.0f, 1.0f);
    } else {
        direction_ = direction / length;
    }
}

Vec3 Ray::at(Real t) const {
    return origin_ + direction_ * t;
}

bool Ray::is_valid_t(Real t) const {
    return t >= t_min_ && t <= t_max_;
}

} // namespace are
