/**
 * @file directional_light.h
 * @brief Directional light implementation
 */

#ifndef ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H
#define ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class DirectionalLight
 * @brief Directional light source (sun-like)
 * 
 * Represents an infinitely distant light source with parallel rays.
 */
class DirectionalLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    DirectionalLight();

    /**
     * @brief Construct with direction and color
     * @param direction Light direction (will be normalized)
     * @param color Light color
     * @param intensity Light intensity
     */
    DirectionalLight(const Vec3& direction, const Vec3& color = Vec3(1.0f), 
                    Real intensity = 1.0f);

    // Direction
    void set_direction(const Vec3& direction);
    const Vec3& get_direction() const { return direction_; }

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 direction_;                      ///< Light direction (normalized)
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H
