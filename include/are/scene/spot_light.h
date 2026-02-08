/**
 * @file spot_light.h
 * @brief Spot light implementation
 */

#ifndef ARE_INCLUDE_SCENE_SPOT_LIGHT_H
#define ARE_INCLUDE_SCENE_SPOT_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class SpotLight
 * @brief Spot light source
 * 
 * Emits light in a cone from a single point.
 */
class SpotLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    SpotLight();

    /**
     * @brief Construct with position, direction, and angles
     * @param position Light position
     * @param direction Light direction
     * @param inner_angle Inner cone angle in degrees
     * @param outer_angle Outer cone angle in degrees
     * @param color Light color
     * @param intensity Light intensity
     */
    SpotLight(const Vec3& position, const Vec3& direction,Real inner_angle, Real outer_angle,
             const Vec3& color = Vec3(1.0f), Real intensity = 1.0f);

    // Position and direction
    void set_position(const Vec3& position);
    void set_direction(const Vec3& direction);
    const Vec3& get_position() const { return position_; }
    const Vec3& get_direction() const { return direction_; }

    // Cone angles (in degrees)
    void set_inner_angle(Real angle);
    void set_outer_angle(Real angle);
    Real get_inner_angle() const { return inner_angle_; }
    Real get_outer_angle() const { return outer_angle_; }

    // Range
    void set_range(Real range);
    Real get_range() const { return range_; }

    /**
     * @brief Calculate spotlight intensity at given direction
     * @param to_point Direction from light to point (normalized)
     * @return Spotlight factor [0, 1]
     */
    Real calculate_spot_factor(const Vec3& to_point) const;

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 position_;                       ///< Light position
    Vec3 direction_;                      ///< Light direction (normalized)
    Real inner_angle_;                    ///< Inner cone angle (degrees)
    Real outer_angle_;                    ///< Outer cone angle (degrees)
    Real range_;                          ///< Light range
    Real cos_inner_;                      ///< Cosine of inner angle (cache
	Real cos_outer_;                      ///< Cosine of outer angle (cached)
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SPOT_LIGHT_H
