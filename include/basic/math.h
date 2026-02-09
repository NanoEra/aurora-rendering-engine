#ifndef ARE_INCLUDE_BASIC_MATH_UTILS_H
#define ARE_INCLUDE_BASIC_MATH_UTILS_H

#include "types.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace are {

/// @brief Math utility functions wrapping GLM
class MathUtils {
public:
    /// @brief Create perspective projection matrix
    /// @param fov Field of view in radians
    /// @param aspect Aspect ratio
    /// @param near Near plane distance
    /// @param far Far plane distance
    /// @return Projection matrix
    static Mat4 perspective(float fov, float aspect, float near, float far);
    
    /// @brief Create look-at view matrix
    /// @param eye Camera position
    /// @param center Look-at target
    /// @param up Up vector
    /// @return View matrix
    static Mat4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up);
    
    /// @brief Normalize a vector
    /// @param v Input vector
    /// @return Normalized vector
    static Vec3 normalize(const Vec3& v);
    
    /// @brief Calculate dot product
    /// @param a First vector
    /// @param b Second vector
    /// @return Dot product
    static float dot(const Vec3& a, const Vec3& b);
    
    /// @brief Calculate cross product
    /// @param a First vector
    /// @param b Second vector
    /// @return Cross product
    static Vec3 cross(const Vec3& a, const Vec3& b);
    
    /// @brief Reflect vector around normal
    /// @param incident Incident vector
    /// @param normal Surface normal
    /// @return Reflected vector
    static Vec3 reflect(const Vec3& incident, const Vec3& normal);
    
    /// @brief Get pointer to matrix data (for OpenGL)
    /// @param mat Input matrix
    /// @return Pointer to matrix data
    static const float* value_ptr(const Mat4& mat);
};

} // namespace are

#endif // ARE_INCLUDE_BASIC_MATH_UTILS_H
