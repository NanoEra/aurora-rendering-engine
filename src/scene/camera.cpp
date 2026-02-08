/**
 * @file camera.cpp
 * @brief Implementation of camera system
 */

#include <are/scene/camera.h>
#include <glm/gtc/matrix_transform.hpp>

namespace are {

Camera::Camera()
    : position_(0.0f, 0.0f, 5.0f)
    , target_(0.0f, 0.0f, 0.0f)
    , up_(0.0f, 1.0f, 0.0f)
    , fov_(45.0f)
    , aspect_ratio_(16.0f / 9.0f)
    , near_plane_(0.1f)
    , far_plane_(1000.0f)
    , view_matrix_(1.0f)
    , projection_matrix_(1.0f)
    , view_dirty_(true)
    , projection_dirty_(true)
    , dirty_(true) {
}

Camera::Camera(const Vec3& position, const Vec3& target, const Vec3& up)
    : position_(position)
    , target_(target)
    , up_(up)
    , fov_(45.0f)
    , aspect_ratio_(16.0f / 9.0f)
    , near_plane_(0.1f)
    , far_plane_(1000.0f)
    , view_matrix_(1.0f)
    , projection_matrix_(1.0f)
    , view_dirty_(true)
    , projection_dirty_(true)
    , dirty_(true) {
}

void Camera::set_position(const Vec3& position) {
    position_ = position;
    view_dirty_ = true;
    dirty_ = true;
}

void Camera::set_target(const Vec3& target) {
    target_ = target;
    view_dirty_ = true;
    dirty_ = true;
}

void Camera::set_up(const Vec3& up) {
    up_ = up;
    view_dirty_ = true;
    dirty_ = true;
}

void Camera::look_at(const Vec3& position, const Vec3& target, const Vec3& up) {
    position_ = position;
    target_ = target;
    up_ = up;
    view_dirty_ = true;
    dirty_ = true;
}

void Camera::set_fov(Real fov_degrees) {
    fov_ = fov_degrees;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_aspect_ratio(Real aspect) {
    aspect_ratio_ = aspect;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_near_plane(Real near) {
    near_plane_ = near;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_far_plane(Real far) {
    far_plane_ = far;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_perspective(Real fov_degrees, Real aspect, Real near, Real far) {
    fov_ = fov_degrees;
    aspect_ratio_ = aspect;
    near_plane_ = near;
    far_plane_ = far;
    projection_dirty_ = true;
    dirty_ = true;
}

Vec3 Camera::get_forward() const {
    return glm::normalize(target_ - position_);
}

Vec3 Camera::get_right() const {
    return glm::normalize(glm::cross(get_forward(), up_));
}

const Mat4& Camera::get_view_matrix() const {
    if (view_dirty_) {
        update_view_matrix();
    }
    return view_matrix_;
}

const Mat4& Camera::get_projection_matrix() const {
    if (projection_dirty_) {
        update_projection_matrix();
    }
    return projection_matrix_;
}

Mat4 Camera::get_view_projection_matrix() const {
    return get_projection_matrix() * get_view_matrix();
}

void Camera::generate_ray(Real u, Real v, Vec3& origin, Vec3& direction) const {
    // Convert from [0,1] to [-1,1]
    Real x = 2.0f * u - 1.0f;
    Real y = 1.0f - 2.0f * v;  // Flip Y
    
    // Compute ray direction in camera space
    Real tan_half_fov = std::tan(glm::radians(fov_ * 0.5f));
    Real camera_x = x * aspect_ratio_ * tan_half_fov;
    Real camera_y = y * tan_half_fov;
    
    // Transform to world space
    Vec3 forward = get_forward();
    Vec3 right = get_right();
    Vec3 up = glm::normalize(glm::cross(right, forward));
    
    origin = position_;
    direction = glm::normalize(forward + camera_x * right + camera_y * up);
}

void Camera::update_view_matrix() const {
    view_matrix_ = glm::lookAt(position_, target_, up_);
    view_dirty_ = false;
}

void Camera::update_projection_matrix() const {
    projection_matrix_ = glm::perspective(
        glm::radians(fov_),
        aspect_ratio_,
        near_plane_,
        far_plane_
    );
    projection_dirty_ = false;
}

} // namespace are
