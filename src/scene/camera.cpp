#include "scene/camera.h"
#include "basic/math.h"
#include <glm/gtc/matrix_transform.hpp>

namespace are {

Camera::Camera()
    : position_(0.0f, 0.0f, 5.0f)
    , target_(0.0f, 0.0f, 0.0f)
    , up_(0.0f, 1.0f, 0.0f)
    , projection_type_(ProjectionType::PERSPECTIVE)
    , fov_(glm::radians(45.0f))
    , aspect_(16.0f / 9.0f)
    , left_(-1.0f)
    , right_(1.0f)
    , bottom_(-1.0f)
    , top_(1.0f)
    , near_(0.1f)
    , far_(100.0f)
    , view_dirty_(true)
    , projection_dirty_(true) {
}

Camera::~Camera() {
}

void Camera::set_perspective(float fov, float aspect, float near, float far) {
    projection_type_ = ProjectionType::PERSPECTIVE;
    fov_ = glm::radians(fov);
    aspect_ = aspect;
    near_ = near;
    far_ = far;
    projection_dirty_ = true;
}

void Camera::set_orthographic(float left, float right, float bottom, float top, float near, float far) {
    projection_type_ = ProjectionType::ORTHOGRAPHIC;
    left_ = left;
    right_ = right;
    bottom_ = bottom;
    top_ = top;
    near_ = near;
    far_ = far;
    projection_dirty_ = true;
}

void Camera::set_position(const Vec3& position) {
    position_ = position;
    view_dirty_ = true;
}

void Camera::set_target(const Vec3& target) {
    target_ = target;
    view_dirty_ = true;
}

void Camera::set_up(const Vec3& up) {
    up_ = up;
    view_dirty_ = true;
}

Mat4 Camera::get_view_matrix() const {
    if (view_dirty_) {
        view_matrix_ = MathUtils::look_at(position_, target_, up_);
        view_dirty_ = false;
    }
    return view_matrix_;
}

Mat4 Camera::get_projection_matrix() const {
    if (projection_dirty_) {
        if (projection_type_ == ProjectionType::PERSPECTIVE) {
            projection_matrix_ = MathUtils::perspective(fov_, aspect_, near_, far_);
        } else {
            projection_matrix_ = glm::ortho(left_, right_, bottom_, top_, near_, far_);
        }
        projection_dirty_ = false;
    }
    return projection_matrix_;
}

Mat4 Camera::get_view_projection_matrix() const {
    return get_projection_matrix() * get_view_matrix();
}

Vec3 Camera::get_forward() const {
    return MathUtils::normalize(target_ - position_);
}

Vec3 Camera::get_right() const {
    Vec3 forward = get_forward();
    return MathUtils::normalize(MathUtils::cross(forward, up_));
}

Vec3 Camera::get_up() const {
    Vec3 forward = get_forward();
    Vec3 right = get_right();
    return MathUtils::cross(right, forward);
}

} // namespace are
