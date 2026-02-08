/**
 * @file point_light.h
 * @brief Point light implementation
 */

#ifndef ARE_INCLUDE_SCENE_POINT_LIGHT_H
#define ARE_INCLUDE_SCENE_POINT_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class PointLight
 * @brief Point light source
 * 
 * Emits light equally in all directions from a single point.
 */
class PointLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    PointLight();

    /**
     * @brief Construct with position and color
     * @param position Light position
     * @param color Light color
     * @param intensity Light intensity
     * @param range Light range (attenuation distance)
     */
    PointLight(const Vec3& position, const Vec3& color = Vec3(1.0f),
              Real intensity = 1.0f, Real range = 10.0f);

    // Position
    void set_position(const Vec3& position);
    const Vec3& get_position() const { return position_; }

    // Range (attenuation)
    void set_range(Real range);
    Real get_range() const { return range_; }

    // Attenuation parameters
    void set_attenuation(Real constant, Real linear, Real quadratic);
    Real get_constant_attenuation() const { return attenuation_constant_; }
    Real get_linear_attenuation() const { return attenuation_linear_; }
    Real get_quadratic_attenuation() const { return attenuation_quadratic_; }

    /**
     * @brief Calculate attenuation at given distance
     * @param distance Distance from light
     * @return Attenuation factor [0, 1]
     */
    Real calculate_attenuation(Real distance) const;

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 position_;                       ///< Light position
    Real range_;                          ///< Light range
    Real attenuation_constant_;           ///< Constant attenuation factor
    Real attenuation_linear_;             ///< Linear attenuation factor
    Real attenuation_quadratic_;          ///< Quadratic attenuation factor
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_POINT_LIGHT_H
