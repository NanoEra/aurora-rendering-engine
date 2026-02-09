#include "basic/math.h"

namespace are {

Mat4 MathUtils::perspective(float fov, float aspect, float near, float far) {
    return glm::perspective(fov, aspect, near, far);
}

Mat4 MathUtils::look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
    return glm::lookAt(eye, center, up);
}

Vec3 MathUtils::normalize(const Vec3& v) {
    return glm::normalize(v);
}

float MathUtils::dot(const Vec3& a, const Vec3& b) {
    return glm::dot(a, b);
}

Vec3 MathUtils::cross(const Vec3& a, const Vec3& b) {
    return glm::cross(a, b);
}

Vec3 MathUtils::reflect(const Vec3& incident, const Vec3& normal) {
    return glm::reflect(incident, normal);
}

const float* MathUtils::value_ptr(const Mat4& mat) {
    return glm::value_ptr(mat);
}

} // namespace are
