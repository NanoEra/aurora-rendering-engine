/**
 * @file light.cpp
 * @brief Implementation of base Light class
 */

#include <are/scene/light.h>
#include <are/core/logger.h>
#include <are/utils/math_utils.h>

namespace are {

Light::Light(LightType type)
    : type_(type)
    , color_(1.0f, 1.0f, 1.0f)
    , intensity_(1.0f)
    , cast_shadows_(true) {
}

void Light::set_color(const Vec3& color) {
    // Clamp color to valid range [0, 1]
    color_.x = clamp(color.x, 0.0f, 1.0f);
    color_.y = clamp(color.y, 0.0f, 1.0f);
    color_.z = clamp(color.z, 0.0f, 1.0f);
}

void Light::set_intensity(Real intensity) {
    // Intensity can be HDR, so only clamp to non-negative
    intensity_ = std::max(0.0f, intensity);
}

void Light::set_cast_shadows(bool cast) {
    cast_shadows_ = cast;
}

} // namespace are
