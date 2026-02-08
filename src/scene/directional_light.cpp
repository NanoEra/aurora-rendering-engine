/**
 * @file directional_light.cpp
 * @brief Implementation of DirectionalLight class
 */

#include <are/scene/directional_light.h>
#include <are/core/logger.h>
#include <glm/glm.hpp>

namespace are {

DirectionalLight::DirectionalLight()
    : Light(LightType::ARE_LIGHT_DIRECTIONAL)
    , direction_(0.0f, -1.0f, 0.0f) {
}

DirectionalLight::DirectionalLight(const Vec3& direction, const Vec3& color, Real intensity)
    : Light(LightType::ARE_LIGHT_DIRECTIONAL)
    , direction_(glm::normalize(direction)) {
    set_color(color);
    set_intensity(intensity);
}

void DirectionalLight::set_direction(const Vec3& direction) {
    Real length = glm::length(direction);
    if (length < are_epsilon) {
        ARE_LOG_WARN("DirectionalLight: Invalid direction vector (zero length), using default");
        direction_ = Vec3(0.0f, -1.0f, 0.0f);
    } else {
        direction_ = direction / length;
    }
}

LightData DirectionalLight::pack() const {
    LightData data;
    
    // position_type_: xyz unused for directional, w = light type
    data.position_type_ = Vec4(0.0f, 0.0f, 0.0f, 
                static_cast<float>(LightType::ARE_LIGHT_DIRECTIONAL));
    
    // direction_range_: xyz = direction, w = range (unused for directional)
    data.direction_range_ = Vec4(direction_, 0.0f);
    
    // color_intensity_: xyz = color, w = intensity
    data.color_intensity_ = Vec4(color_, intensity_);
    
    // params_: x = cast_shadows
    data.params_ = Vec4(cast_shadows_ ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
    
    return data;
}

bool DirectionalLight::affects_point(const Vec3& point) const {
    // Directional light affects all points
    (void)point; // Suppress unused parameter warning
    return true;
}

} // namespace are
