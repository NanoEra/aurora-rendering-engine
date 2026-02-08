/**
 * @file ray.h
 * @brief Ray structure for ray tracing
 */

#ifndef ARE_INCLUDE_RAYTRACER_RAY_H
#define ARE_INCLUDE_RAYTRACER_RAY_H

#include <are/core/types.h>

namespace are {

/**
 * @struct Ray
 * @brief Ray representation for ray tracing
 */
struct Ray {
    Vec3 origin_;                         ///< Ray origin
    Vec3 direction_;                      ///< Ray direction (normalized)
    Real t_min_;                          ///< Minimum t value
    Real t_max_;                          ///< Maximum t value

    /**
     * @brief Default constructor
     */
    Ray();

    /**
     * @brief Construct ray with origin and direction
     * @param origin Ray origin
     * @param direction Ray direction (will be normalized)
     * @param t_min Minimum t value
     * @param t_max Maximum t value
     */
    Ray(const Vec3& origin, const Vec3& direction, 
        Real t_min = are_epsilon, Real t_max = 1e30f);

    /**
     * @brief Evaluate ray at parameter t
     * @param t Parameter value
     * @return Point on ray
     */
    Vec3 at(Real t) const;

    /**
     * @brief Check if t is within valid range
     * @param t Parameter value
     * @return true if t is valid
     */
    bool is_valid_t(Real t) const;
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_RAY_H
