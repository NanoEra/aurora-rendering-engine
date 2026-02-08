/**
 * @file raytracer.h
 * @brief Ray tracing interface
 */

#ifndef ARE_INCLUDE_RAYTRACER_RAYTRACER_H
#define ARE_INCLUDE_RAYTRACER_RAYTRACER_H

#include <are/core/types.h>
#include <are/core/config.h>

namespace are {

// Forward declarations
class SceneManager;
class Camera;
class GBuffer;
class BVH;

/**
 * @class RayTracer
 * @brief Abstract ray tracing interface
 * 
 * Base class for CPU and GPU ray tracing implementations.
 */
class RayTracer {
public:
    /**
     * @brief Constructor
     * @param config Ray tracing configuration
     */
    explicit RayTracer(const RayTracingConfig& config);

    /**
     * @brief Virtual destructor
     */
    virtual ~RayTracer() = default;

    /**
     * @brief Render scene using ray tracing
     * @param scene Scene manager
     * @param camera Camera
     * @param gbuffer G-Buffer (optional, for hybrid rendering)
     * @param output Output texture ID
     */
    virtual void render(const SceneManager& scene, 
                       const Camera& camera,
                       const GBuffer* gbuffer,
                       uint32_t output_texture) = 0;

    /**
     * @brief Update BVH
     * @param bvh BVH reference
     */
    virtual void update_bvh(const BVH& bvh) = 0;

    /**
     * @brief Set configuration
     * @param config New configuration
     */
    virtual void set_config(const RayTracingConfig& config);

    /**
     * @brief Get configuration
     * @return Current configuration
     */
    const RayTracingConfig& get_config() const { return config_; }

protected:
    RayTracingConfig config_;             ///< Ray tracing configuration
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_RAYTRACER_H
