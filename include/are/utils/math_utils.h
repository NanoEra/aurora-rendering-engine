/**
 * @file math_utils.h
 * @brief Mathematical utility functions
 */

#ifndef ARE_INCLUDE_UTILS_MATH_UTILS_H
#define ARE_INCLUDE_UTILS_MATH_UTILS_H

#include <are/core/types.h>
#include <algorithm>

namespace are {

/**
 * @brief Clamp value to range [min, max]
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
template<typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor [0, 1]
 * @return Interpolated value
 */
template<typename T>
inline T lerp(T a, T b, Real t) {
    return a + (b - a) * t;
}

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline Real degrees_to_radians(Real degrees) {
    return degrees * are_pi / 180.0f;
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline Real radians_to_degrees(Real radians) {
    return radians * 180.0f / are_pi;
}

/**
 * @brief Check if two floating point values are approximately equal
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance
 * @return true if approximately equal
 */
inline bool approx_equal(Real a, Real b, Real epsilon = are_epsilon) {
    return std::abs(a - b) < epsilon;
}

/**
 * @brief Compute barycentric coordinates
 * @param p Point
 * @param a Triangle vertex A
 * @param b Triangle vertex B
 * @param c Triangle vertex C
 * @param u Output barycentric coordinate u
 * @param v Output barycentric coordinate v
 * @param w Output barycentric coordinate w
 */
void compute_barycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                        Real& u, Real& v, Real& w);

/**
 * @brief Reflect vector around normal
 * @param incident Incident vector
 * @param normal Surface normal (normalized)
 * @return Reflected vector
 */
Vec3 reflect(const Vec3& incident, const Vec3& normal);

/**
 * @brief Refract vector through surface
 * @param incident Incident vector (normalized)
 * @param normal Surface normal (normalized)
 * @param eta Ratio of refractive indices
 * @param refracted Output refracted vector
 * @return true if refraction occurred (false for total internal reflection)
 */
bool refract(const Vec3& incident, const Vec3& normal, Real eta, Vec3& refracted);

/**
 * @brief Compute Fresnel reflectance (Schlick approximation)
 * @param cos_theta Cosine of angle between view and normal
 * @param f0 Reflectance at normal incidence
 * @return Fresnel reflectance
 */
Real fresnel_schlick(Real cos_theta, Real f0);

/**
 * @brief Create orthonormal basis from normal
 * @param normal Normal vector (normalized)
 * @param tangent Output tangent vector
 * @param bitangent Output bitangent vector
 */
void create_orthonormal_basis(const Vec3& normal, Vec3& tangent, Vec3& bitangent);

/**
 * @brief Transform direction from tangent space to world space
 * @param tangent_dir Direction in tangent space
 * @param normal Surface normal
 * @param tangent Surface tangent
 * @param bitangent Surface bitangent
 * @return Direction in world space
 */
Vec3 tangent_to_world(const Vec3& tangent_dir, const Vec3& normal, 
                     const Vec3& tangent, const Vec3& bitangent);

} // namespace are

#endif // ARE_INCLUDE_UTILS_MATH_UTILS_H
