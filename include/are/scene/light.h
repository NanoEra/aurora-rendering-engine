/**
 * @file light.h
 * @brief Base light class and common light utilities
 */

#ifndef ARE_INCLUDE_SCENE_LIGHT_H
#define ARE_INCLUDE_SCENE_LIGHT_H

#include <are/core/types.h>

namespace are {

/**
 * @enum LightType
 * @brief Types of light sources
 */
enum class LightType {
    ARE_LIGHT_DIRECTIONAL,
    ARE_LIGHT_POINT,
    ARE_LIGHT_SPOT
};

/**
 * @struct LightData
 * @brief Packed light data for GPU transfer
 * 
 * This structure is designed to be efficiently transferred to GPU
 * via SSBO or UBO.
 */
struct LightData {
    Vec4 position_type_;                  ///< xyz: position, w: light type
    Vec4 direction_range_;                ///< xyz: direction, w: range
    Vec4 color_intensity_;                ///< xyz: color, w: intensity
    Vec4 params_;                         ///< Light-specific parameters
};

/**
 * @class Light
 * @brief Base class for all light types
 */
class Light {
public:
    /**
     * @brief Constructor
     * @param type Light type
     */
    explicit Light(LightType type);

    virtual ~Light() = default;

    // Common properties
    void set_color(const Vec3& color);
    void set_intensity(Real intensity);
    void set_cast_shadows(bool cast);

    const Vec3& get_color() const { return color_; }
    Real get_intensity() const { return intensity_; }
    bool get_cast_shadows() const { return cast_shadows_; }
    LightType get_type() const { return type_; }

    /**
     * @brief Pack light data for GPU transfer
     * @return Packed light data
     */
    virtual LightData pack() const = 0;

    /**
     * @brief Check if light affects a point
     * @param point World position
     * @return true if light can affect the point
     */
    virtual bool affects_point(const Vec3& point) const = 0;

protected:
    LightType type_;                      ///< Light type
    Vec3 color_;                          ///< Light color (RGB)
    Real intensity_;                      ///< Light intensity
    bool cast_shadows_;                   ///< Whether light casts shadows
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_LIGHT_H
