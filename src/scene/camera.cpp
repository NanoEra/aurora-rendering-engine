/**
 * @file camera.cpp
 * @brief Implementation of Camera class
 */

#include <are/scene/camera.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/utils/math_utils.h>
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
    , up_(glm::normalize(up))
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
    Real length = glm::length(up);
    if (length < are_epsilon) {
        ARE_LOG_WARN("Camera: Invalid up vector (zero length), using default");
        up_ = Vec3(0.0f, 1.0f, 0.0f);
    } else {
        up_ = up / length;
    }
    view_dirty_ = true;
    dirty_ = true;
}

void Camera::look_at(const Vec3& position, const Vec3& target, const Vec3& up) {
    position_ = position;
    target_ = target;
    set_up(up);
}

void Camera::set_fov(Real fov_degrees) {
    // Clamp FOV to reasonable range
    fov_ = clamp(fov_degrees, 1.0f, 179.0f);
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_aspect_ratio(Real aspect) {
    if (aspect <= 0.0f) {
        ARE_LOG_ERROR("Camera: Invalid aspect ratio (must be positive)");
        return;
    }
    aspect_ratio_ = aspect;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_near_plane(Real near) {
    if (near <= 0.0f) {
        ARE_LOG_ERROR("Camera: Invalid near plane (must be positive)");
        return;
    }
    near_plane_ = near;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_far_plane(Real far) {
    if (far <= near_plane_) {
        ARE_LOG_ERROR("Camera: Invalid far plane (must be greater than near plane)");
        return;
    }
    far_plane_ = far;
    projection_dirty_ = true;
    dirty_ = true;
}

void Camera::set_perspective(Real fov_degrees, Real aspect, Real near, Real far) {
    set_fov(fov_degrees);
    set_aspect_ratio(aspect);
    set_near_plane(near);
    set_far_plane(far);
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
    ARE_PROFILE_FUNCTION();
    
    // Ray origin is camera position
    origin = position_;
    
    // Calculate ray direction in camera space
    // u, v are in [0, 1], convert to [-1, 1]
    Real x = 2.0f * u - 1.0f;
    Real y = 1.0f - 2.0f * v; // Flip y for screen coordinates
    
    // Calculate direction based on FOV and aspect ratio
    Real tan_half_fov = std::tan(degrees_to_radians(fov_ * 0.5f));
    Real viewport_height = 2.0f * tan_half_fov;
    Real viewport_width = viewport_height * aspect_ratio_;
    
    // Get camera basis vectors
    Vec3 forward = get_forward();
    Vec3 right = get_right();
    Vec3 up = glm::normalize(glm::cross(right, forward));
    
    // Calculate ray direction
    direction = glm::normalize(
        forward + 
        right * (x * viewport_width * 0.5f) + 
        up * (y * viewport_height * 0.5f)
    );
}

void Camera::update_view_matrix() const {
    ARE_PROFILE_FUNCTION();
    
    // Check if camera is looking at itself
    if (glm::length(target_ - position_) < are_epsilon) {
        ARE_LOG_WARN("Camera: Position and target are too close, using default view");
        view_matrix_ = Mat4(1.0f);
    } else {
        view_matrix_ = glm::lookAt(position_, target_, up_);
    }
    
    view_dirty_ = false;
}

void Camera::update_projection_matrix() const {
    ARE_PROFILE_FUNCTION();
    
    projection_matrix_ = glm::perspective(
        degrees_to_radians(fov_),
        aspect_ratio_,
        near_plane_,
        far_plane_
    );
    
    projection_dirty_ = false;
}

} // namespace are
