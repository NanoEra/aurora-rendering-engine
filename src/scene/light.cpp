#include "scene/light.h"
#include <glm/gtc/constants.hpp>

namespace are {

Light::Light()
    : type_(LightType::POINT)
    , position_(0.0f, 5.0f, 0.0f)
    , direction_(0.0f, -1.0f, 0.0f)
    , color_(1.0f, 1.0f, 1.0f)
    , intensity_(1.0f)
    , range_(10.0f)
    , inner_angle_(glm::radians(30.0f))
    , outer_angle_(glm::radians(45.0f)) {
}

Light::~Light() {
}

void Light::set_type(LightType type) {
    type_ = type;
}

void Light::set_position(const Vec3& position) {
    position_ = position;
}

void Light::set_direction(const Vec3& direction) {
    direction_ = glm::normalize(direction);
}

void Light::set_color(const Vec3& color) {
    color_ = color;
}

void Light::set_intensity(float intensity) {
    intensity_ = intensity;
}

void Light::set_range(float range) {
    range_ = range;
}

void Light::set_spot_angles(float inner_angle, float outer_angle) {
    inner_angle_ = glm::radians(inner_angle);
    outer_angle_ = glm::radians(outer_angle);
}

} // namespace are
