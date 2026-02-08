/**
 * @file compute_raytracer.h
 * @brief GPU compute shader ray tracing implementation
 */

#ifndef ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H
#define ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H

#include <are/raytracer/raytracer.h>
#include <memory>

namespace are {

// Forward declarations
class ShaderProgram;

/**
 * @class ComputeRayTracer
 * @brief GPU-based ray tracing using compute shaders
 */
class ComputeRayTracer : public RayTracer {
public:
    /**
     * @brief Constructor
     * @param config Ray tracing configuration
     */
    explicit ComputeRayTracer(const RayTracingConfig& config);

    /**
     * @brief Destructor
     */
    ~ComputeRayTracer() override;

    /**
     * @brief Render scene using compute shader ray tracing
     * @param scene Scene manager
     * @param camera Camera
     * @param gbuffer G-Buffer (optional)
     * @param output Output texture ID
     */
    void render(const SceneManager& scene, 
               const Camera& camera,
               const GBuffer* gbuffer,
               uint32_t output_texture) override;

    /**
     * @brief Update BVH
     * @param bvh BVH reference
     */
    void update_bvh(const BVH& bvh) override;

private:
    void initialize_compute_shader(const std::string& shader_dir);
    void upload_scene_data(const SceneManager& scene);
    void upload_bvh_data(const BVH& bvh);
    void upload_camera_data(const Camera& camera);

    std::unique_ptr<ShaderProgram> compute_shader_; ///< Ray tracing compute shader

    // GPU buffers (SSBOs)
    uint32_t bvh_buffer_;                 ///< BVH nodes buffer
    uint32_t triangle_buffer_;            ///< Triangle data buffer
    uint32_t material_buffer_;            ///< Material data buffer
    uint32_t light_buffer_;               ///< Light data buffer

    bool buffers_initialized_;            ///< Buffer initialization flag
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H
