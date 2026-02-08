/**
 * @file point_light.cpp
 * @brief Implementation of PointLight class
 */

#include <are/scene/point_light.h>
#include <are/core/logger.h>
#include <glm/glm.hpp>

namespace are {

PointLight::PointLight()
    : Light(LightType::ARE_LIGHT_POINT)
    , position_(0.0f)
    , range_(10.0f)
    , attenuation_constant_(1.0f)
    , attenuation_linear_(0.09f)
    , attenuation_quadratic_(0.032f) {
}

PointLight::PointLight(const Vec3& position, const Vec3& color, Real intensity, Real range)
    : Light(LightType::ARE_LIGHT_POINT)
    , position_(position)
    , range_(range)
    , attenuation_constant_(1.0f)
    , attenuation_linear_(0.09f)
    , attenuation_quadratic_(0.032f) {
    set_color(color);
    set_intensity(intensity);
    set_range(range);
}

void PointLight::set_position(const Vec3& position) {
    position_ = position;
}

void PointLight::set_range(Real range) {
    if (range <= 0.0f) {
        ARE_LOG_WARN("PointLight: Invalid range (must be positive), using default");
        range_ = 10.0f;
    } else {
        range_ = range;
    }
}

void PointLight::set_attenuation(Real constant, Real linear, Real quadratic) {
    attenuation_constant_ = std::max(0.0f, constant);
    attenuation_linear_ = std::max(0.0f, linear);
    attenuation_quadratic_ = std::max(0.0f, quadratic);
    
    // Ensure at least some attenuation to avoid division issues
    if (attenuation_constant_ < are_epsilon && 
        attenuation_linear_ < are_epsilon && 
        attenuation_quadratic_ < are_epsilon) {
        ARE_LOG_WARN("PointLight: All attenuation factors near zero, setting constant to 1.0");
        attenuation_constant_ = 1.0f;
    }
}

Real PointLight::calculate_attenuation(Real distance) const {
    // Standard attenuation formula: 1 / (constant + linear*d + quadratic*d^2)
    Real attenuation = attenuation_constant_ + 
                       attenuation_linear_ * distance + 
                       attenuation_quadratic_ * distance * distance;
    return 1.0f / std::max(attenuation, are_epsilon);
}

LightData PointLight::pack() const {
    LightData data;
    
    // position_type_: xyz = position, w = light type
    data.position_type_ = Vec4(position_, 
                               static_cast<float>(LightType::ARE_LIGHT_POINT));
    
    // direction_range_: xyz unused for point light, w = range
    data.direction_range_ = Vec4(0.0f, 0.0f, 0.0f, range_);
    
    // color_intensity_: xyz = color, w = intensity
    data.color_intensity_ = Vec4(color_, intensity_);
    
    // params_: x = cast_shadows, y = constant, z = linear, w = quadratic
    data.params_ = Vec4(
        cast_shadows_ ? 1.0f : 0.0f,
        attenuation_constant_,
        attenuation_linear_,
        attenuation_quadratic_
    );
    
    return data;
}

bool PointLight::affects_point(const Vec3& point) const {
    Real distance = glm::length(point - position_);
    return distance <= range_;
}

} // namespace are
