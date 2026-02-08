/**
 * @file transform.cpp
 * @brief Implementation of Transform class
 */

#define GLM_ENABLE_EXPERIMENTAL
#include <are/geometry/transform.h>
#include <are/core/profiler.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

namespace are {

Transform::Transform()
    : position_(0.0f)
    , rotation_(0.0f)
    , scale_(1.0f)
    , matrix_(1.0f)
    , inverse_matrix_(1.0f)
    , dirty_(true) {
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
    // Normal matrix is the transpose of the inverse of the upper-left 3x3
    return glm::transpose(glm::inverse(Mat3(matrix_)));
}

Vec3 Transform::transform_point(const Vec3& point) const {
    if (dirty_) {
        update_matrix();
    }
    Vec4 result = matrix_ * Vec4(point, 1.0f);
    return Vec3(result);
}

Vec3 Transform::transform_direction(const Vec3& direction) const {
    if (dirty_) {
        update_matrix();
    }
    Vec4 result = matrix_ * Vec4(direction, 0.0f);
    return Vec3(result);
}

Vec3 Transform::transform_normal(const Vec3& normal) const {
    Mat3 normal_matrix = get_normal_matrix();
    return glm::normalize(normal_matrix * normal);
}

Transform Transform::operator*(const Transform& other) const {
    ARE_PROFILE_FUNCTION();
    
    // Combine transforms by multiplying matrices
    // Note: This is an approximation; for exact results, 
    // we would need to decompose the combined matrix
    Transform result;
    
    Mat4 combined = get_matrix() * other.get_matrix();
    
    // Extract translation
    result.position_ = Vec3(combined[3]);
    
    // Extract scale (approximate)
    result.scale_.x = glm::length(Vec3(combined[0]));
    result.scale_.y = glm::length(Vec3(combined[1]));
    result.scale_.z = glm::length(Vec3(combined[2]));
    
    // Remove scale from matrix to extract rotation
    Mat3 rotation_matrix;
    rotation_matrix[0] = Vec3(combined[0]) / result.scale_.x;
    rotation_matrix[1] = Vec3(combined[1]) / result.scale_.y;
    rotation_matrix[2] = Vec3(combined[2]) / result.scale_.z;
    
    // Extract Euler angles (approximate, may have gimbal lock issues)
    result.rotation_.x = std::atan2(rotation_matrix[2][1], rotation_matrix[2][2]);
    result.rotation_.y = std::atan2(-rotation_matrix[2][0],std::sqrt(rotation_matrix[2][1] * rotation_matrix[2][1] + 
                  rotation_matrix[2][2] * rotation_matrix[2][2]));
    result.rotation_.z = std::atan2(rotation_matrix[1][0], rotation_matrix[0][0]);
    
    result.dirty_ = true;
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
    ARE_PROFILE_FUNCTION();
    
    // Build transformation matrix: T * R * S
    // Translation
    Mat4 translation_matrix = glm::translate(Mat4(1.0f), position_);
    
    // Rotation (using Euler angles: YXZ order for typical camera/object rotation)
    Mat4 rotation_matrix = glm::eulerAngleYXZ(rotation_.y, rotation_.x, rotation_.z);
    
    // Scale
    Mat4 scale_matrix = glm::scale(Mat4(1.0f), scale_);
    
    // Combine: T * R * S
    matrix_ = translation_matrix * rotation_matrix * scale_matrix;
    
    // Compute inverse
    inverse_matrix_ = glm::inverse(matrix_);
    
    dirty_ = false;
}

} // namespace are
