/**
 * @file aabb.h
 * @brief Axis-Aligned Bounding Box implementation
 */

#ifndef ARE_INCLUDE_GEOMETRY_AABB_H
#define ARE_INCLUDE_GEOMETRY_AABB_H

#include <are/core/types.h>

namespace are {

// Forward declaration
struct Ray;

/**
 * @class AABB
 * @brief Axis-Aligned Bounding Box for spatial queries
 * 
 * Used for BVH construction and ray intersection acceleration.
 */
class AABB {
public:
    Vec3 min_;                            ///< Minimum corner
    Vec3 max_;                            ///< Maximum corner

    /**
     * @brief Default constructor - creates invalid AABB
     */
    AABB();

    /**
     * @brief Construct AABB from min and max corners
     * @param min Minimum corner
     * @param max Maximum corner
     */
    AABB(const Vec3& min, const Vec3& max);

    /**
     * @brief Construct AABB containing a single point
     * @param point Point to contain
     */
    explicit AABB(const Vec3& point);

    /**
     * @brief Check if AABB is valid (min <= max)
     * @return true if valid
     */
    bool is_valid() const;

    /**
     * @brief Get center of AABB
     * @return Center point
     */
    Vec3 center() const;

    /**
     * @brief Get size (extent) of AABB
     * @return Size vector
     */
    Vec3 size() const;

    /**
     * @brief Get surface area of AABB
     * @return Surface area
     */
    Real surface_area() const;

    /**
     * @brief Get volume of AABB
     * @return Volume
     */
    Real volume() const;

    /**
     * @brief Get longest axis index (0=x, 1=y, 2=z)
     * @return Axis index
     */
    int longest_axis() const;

    /**
     * @brief Expand AABB to include a point
     * @param point Point to include
     */
    void expand(const Vec3& point);

    /**
     * @brief Expand AABB to include another AABB
     * @param other AABB to include
     */
    void expand(const AABB& other);

    /**
     * @brief Check if AABB contains a point
     * @param point Point to check
     * @return true if point is inside AABB
     */
    bool contains(const Vec3& point) const;

    /**
     * @brief Check if AABB intersects another AABB
     * @param other AABB to check
     * @return true if AABBs intersect
     */
    bool intersects(const AABB& other) const;

    /**
     * @brief Ray-AABB intersection test
     * @param ray Ray to test
     * @param t_min Minimum t value (output)
     * @param t_max Maximum t value (output)
     * @return true if ray intersects AABB
     */
    bool intersect_ray(const Ray& ray, Real& t_min, Real& t_max) const;

    /**
     * @brief Merge two AABBs
     * @param a First AABB
     * @param b Second AABB
     * @return Merged AABB containing both
     */
    static AABB merge(const AABB& a, const AABB& b);

    /**
     * @brief Create invalid AABB (for initialization)
     * @return Invalid AABB
     */
    static AABB invalid();
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_AABB_H
