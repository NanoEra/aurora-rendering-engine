你对代码的观察非常敏锐！我对你的实现计划表示认同，下面是你要求到的和可能用到的头文件：
### 文件：include/are/raytracer/ray.h
```cpp
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
```

### 文件：include/are/raytracer/hit_record.h
```cpp
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
```

### 文件：include/are/scene/directional_light.h
```cpp
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
```

### 文件：include/are/scene/point_light.h
```cpp
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
```

### 文件：include/are/scene/spot_light.h
```cpp
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
```

还有缺失的头文件需要补充吗？如果没有的话，我们就可以开始实现Phase 2了。
