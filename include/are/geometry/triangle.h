/**
 * @file triangle.h
 * @brief Triangle primitive definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRIANGLE_H
#define ARE_INCLUDE_GEOMETRY_TRIANGLE_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>

namespace are {

// Forward declaration
struct Ray;
struct HitRecord;

/**
 * @struct Triangle
 * @brief Triangle primitive for ray tracing
 * 
 * Stores three vertices and provides intersection testing.
 */
struct Triangle {
    Vertex v0_;                           ///< First vertex
    Vertex v1_;                           ///< Second vertex
    Vertex v2_;                           ///< Third vertex
    MaterialHandle material_;             ///< Material handle

    /**
     * @brief Default constructor
     */
    Triangle();

    /**
     * @brief Construct triangle from three vertices
     * @param v0 First vertex
     * @param v1 Second vertex
     * @param v2 Third vertex
     * @param material Material handle
     */
    Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2,
            MaterialHandle material = are_invalid_handle);

    /**
     * @brief Get triangle centroid
     * @return Centroid position
     */
    Vec3 centroid() const;

    /**
     * @brief Get triangle normal (geometric normal)
     * @return Normal vector (normalized)
     */
    Vec3 normal() const;

    /**
     * @brief Get triangle area
     * @return Area
     */
    Real area() const;

    /**
     * @brief Compute axis-aligned bounding box
     * @return AABB containing the triangle
     */
    AABB compute_aabb() const;

    /**
     * @brief Ray-triangle intersection test (Möller-Trumbore algorithm)
     * @param ray Ray to test
     * @param hit Output hit record
     * @return true if intersection occurred
     */
    bool intersect(const Ray& ray, HitRecord& hit) const;

    /**
     * @brief Fast ray-triangle intersection test (no hit record)
     * @param ray Ray to test
     * @param t_max Maximum t value
     * @return true if intersection occurred
     */
    bool intersect_fast(const Ray& ray, Real t_max) const;
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRIANGLE_H
