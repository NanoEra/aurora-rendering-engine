/**
 * @file hit_record.h
 * @brief Ray-surface intersection record
 */

#ifndef ARE_INCLUDE_RAYTRACER_HIT_RECORD_H
#define ARE_INCLUDE_RAYTRACER_HIT_RECORD_H

#include <are/core/types.h>

namespace are {

/**
 * @struct HitRecord
 * @brief Information about ray-surface intersection
 */
struct HitRecord {
    Vec3 position_;                       ///< Hit position in world space
    Vec3 normal_;                         ///< Surface normal at hit point
    Vec2 texcoord_;                       ///< Texture coordinates at hit point
    Vec3 tangent_;                        ///< Tangent vector at hit point
    Real t_;                              ///< Ray parameter at hit point
    MaterialHandle material_;             ///< Material at hit point
    uint32_t triangle_index_;             ///< Triangle index that was hit
    bool front_face_;                     ///< Whether ray hit front face

    /**
     * @brief Default constructor
     */
    HitRecord();

    /**
     * @brief Set face normal based on ray direction
     * @param ray_direction Ray direction
     * @param outward_normal Outward-facing normal
     */
    void set_face_normal(const Vec3& ray_direction, const Vec3& outward_normal);

    /**
     * @brief Check if hit record is valid
     * @return true if hit occurred
     */
    bool is_valid() const;
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_HIT_RECORD_H
