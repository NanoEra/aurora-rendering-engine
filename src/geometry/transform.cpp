/**
 * @file transform.cpp
 * @brief Implementation of transformation utilities
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <are/geometry/transform.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

namespace are {

Transform::Transform()
    : position_(0.0f)
    , rotation_(0.0f)
    , scale_(1.0f)
    , matrix_(1.0f)
    , inverse_matrix_(1.0f)
    , dirty_(false) {
}

Transform::Transform(const Vec3& position, const Vec3& rotation, const Vec3& scale)
    : position_(position)
    , rotation_(rotation)
    , scale_(scale)
    , matrix_(1.0f)
    , inverse_matrix_(1.0f)
    , dirty_(true) {
}

void Transform::set_position(const Vec3& position) {
    position_ = position;
    mark_dirty();
}

void Transform::set_rotation(const Vec3& rotation) {
    rotation_ = rotation;
    mark_dirty();
}

void Transform::set_scale(const Vec3& scale) {
    scale_ = scale;
    mark_dirty();
}

void Transform::set_scale(Real uniform_scale) {
    scale_ = Vec3(uniform_scale);
    mark_dirty();
}

Mat4 Transform::get_matrix() const {
    if (dirty_) {
        update_matrix();
    }
    return matrix_;
}

Mat4 Transform::get_inverse_matrix() const {
    if (dirty_) {
        update_matrix();
    }
    return inverse_matrix_;
}

Mat3 Transform::get_normal_matrix() const {
    if (dirty_) {
        update_matrix();
    }
    return glm::transpose(glm::inverse(Mat3(matrix_)));
}

Vec3 Transform::transform_point(const Vec3& point) const {
    Vec4 result = get_matrix() * Vec4(point, 1.0f);
    return Vec3(result) / result.w;
}

Vec3 Transform::transform_direction(const Vec3& direction) const {
    return Vec3(get_matrix() * Vec4(direction, 0.0f));
}

Vec3 Transform::transform_normal(const Vec3& normal) const {
    return glm::normalize(get_normal_matrix() * normal);
}

Transform Transform::operator*(const Transform& other) const {
    Transform result;
    result.matrix_ = get_matrix() * other.get_matrix();
    result.inverse_matrix_ = other.get_inverse_matrix() * get_inverse_matrix();
    result.dirty_ = false;
    return result;
}

Transform Transform::identity() {
    return Transform();
}

Transform Transform::translate(const Vec3& translation) {
    return Transform(translation, Vec3(0.0f), Vec3(1.0f));
}

Transform Transform::rotate(const Vec3& rotation) {
    return Transform(Vec3(0.0f), rotation, Vec3(1.0f));
}

Transform Transform::scale(const Vec3& scale) {
    return Transform(Vec3(0.0f), Vec3(0.0f), scale);
}

void Transform::mark_dirty() {
    dirty_ = true;
}

void Transform::update_matrix() const {
    // Build transformation matrix: T * R * S
    Mat4 translation = glm::translate(Mat4(1.0f), position_);
    Mat4 rotation = glm::eulerAngleYXZ(rotation_.y, rotation_.x, rotation_.z);
    Mat4 scale = glm::scale(Mat4(1.0f), scale_);
    
    matrix_ = translation * rotation * scale;
    inverse_matrix_ = glm::inverse(matrix_);
    
    dirty_ = false;
}

} // namespace are
