/**
 * @file spot_light.cpp
 * @brief Implementation of SpotLight class
 */

#include <are/scene/spot_light.h>
#include <are/core/logger.h>
#include <are/utils/math_utils.h>
#include <glm/glm.hpp>

namespace are {

SpotLight::SpotLight()
    : Light(LightType::ARE_LIGHT_SPOT)
    , position_(0.0f)
    , direction_(0.0f, -1.0f, 0.0f)
    , inner_angle_(30.0f)
    , outer_angle_(45.0f)
    , range_(10.0f)
    , cos_inner_(std::cos(degrees_to_radians(30.0f)))
    , cos_outer_(std::cos(degrees_to_radians(45.0f))) {
}

SpotLight::SpotLight(const Vec3& position, const Vec3& direction,
                     Real inner_angle, Real outer_angle,
                     const Vec3& color, Real intensity)
    : Light(LightType::ARE_LIGHT_SPOT)
    , position_(position)
    , range_(10.0f) {
    set_direction(direction);
    set_inner_angle(inner_angle);
    set_outer_angle(outer_angle);
    set_color(color);
    set_intensity(intensity);
}

void SpotLight::set_position(const Vec3& position) {
    position_ = position;
}

void SpotLight::set_direction(const Vec3& direction) {
    Real length = glm::length(direction);
    if (length < are_epsilon) {
        ARE_LOG_WARN("SpotLight: Invalid direction vector (zero length), using default");
        direction_ = Vec3(0.0f, -1.0f, 0.0f);
    } else {
        direction_ = direction / length;
    }
}

void SpotLight::set_inner_angle(Real angle) {
    // Clamp to valid range [0, 90] degrees
    inner_angle_ = clamp(angle, 0.0f, 90.0f);
	cos_inner_ = std::cos(degrees_to_radians(inner_angle_));
    
    // Ensure inner angle is not larger than outer angle
    if (inner_angle_ > outer_angle_) {
        ARE_LOG_WARN("SpotLight: Inner angle larger than outer angle, adjusting outer angle");
        outer_angle_ = inner_angle_;
        cos_outer_ = cos_inner_;
    }
}

void SpotLight::set_outer_angle(Real angle) {
    // Clamp to valid range [0, 90] degrees
    outer_angle_ = clamp(angle, 0.0f, 90.0f);
    cos_outer_ = std::cos(degrees_to_radians(outer_angle_));
    
    // Ensure outer angle is not smaller than inner angle
    if (outer_angle_ < inner_angle_) {
        ARE_LOG_WARN("SpotLight: Outer angle smaller than inner angle, adjusting inner angle");
        inner_angle_ = outer_angle_;
        cos_inner_ = cos_outer_;
    }
}

void SpotLight::set_range(Real range) {
    if (range <= 0.0f) {
        ARE_LOG_WARN("SpotLight: Invalid range (must be positive), using default");
        range_ = 10.0f;
    } else {
        range_ = range;
    }
}

Real SpotLight::calculate_spot_factor(const Vec3& to_point) const {
    // Calculate angle between light direction and direction to point
    Real cos_angle = glm::dot(direction_, glm::normalize(to_point));
    
    // Outside outer cone
    if (cos_angle < cos_outer_) {
        return 0.0f;
    }
    
    // Inside inner cone
    if (cos_angle > cos_inner_) {
        return 1.0f;
    }
    
    // Smooth transition between inner and outer cone
    Real delta = cos_inner_ - cos_outer_;
    if (delta < are_epsilon) {
        return 1.0f;
    }
    
    return (cos_angle - cos_outer_) / delta;
}

LightData SpotLight::pack() const {
    LightData data;
    
    // position_type_: xyz = position, w = light type
    data.position_type_ = Vec4(position_, 
                               static_cast<float>(LightType::ARE_LIGHT_SPOT));
    
    // direction_range_: xyz = direction, w = range
    data.direction_range_ = Vec4(direction_, range_);
    
    // color_intensity_: xyz = color, w = intensity
    data.color_intensity_ = Vec4(color_, intensity_);
    
    // params_: x = cast_shadows, y = cos_inner, z = cos_outer, w unused
    data.params_ = Vec4(
        cast_shadows_ ? 1.0f : 0.0f,
        cos_inner_,
        cos_outer_,
        0.0f
    );
    
    return data;
}

bool SpotLight::affects_point(const Vec3& point) const {
    // Check if point is within range
    Vec3 to_point = point - position_;
    Real distance = glm::length(to_point);
    
    if (distance > range_) {
        return false;
    }
    
    // Check if point is within spotlight cone
    if (distance > are_epsilon) {
        to_point /= distance;
        Real cos_angle = glm::dot(direction_, to_point);
        return cos_angle >= cos_outer_;
    }
    
    return true;
}

} // namespace are
