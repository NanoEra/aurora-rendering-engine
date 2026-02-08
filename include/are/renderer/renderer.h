/**
 * @file renderer.h
 * @brief Main renderer interface
 */

#ifndef ARE_INCLUDE_RENDERER_RENDERER_H
#define ARE_INCLUDE_RENDERER_RENDERER_H

#include <are/core/config.h>
#include <are/core/types.h>
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <are/renderer/render_stats.h>
#include <memory>
#include <vector>

namespace are {

// Forward declarations
class Window;
class SceneManager;
class Rasterizer;
class RayTracer;
class TextureManager;
class BVH;

/**
 * @class Renderer
 * @brief Main rendering interface for Aurora Rendering Engine
 * 
 * This class provides the primary API for rendering scenes using
 * hybrid rasterization and ray tracing techniques.
 */
class Renderer {
public:
    /**
     * @brief Constructor
     * @param config Rendering configuration
     */
    explicit Renderer(const AreConfig& config);

    /**
     * @brief Destructor
     */
    ~Renderer();

    // Configuration
    void set_config(const AreConfig& config);
    const AreConfig& get_config() const;

    // Camera management
    void set_camera(const Camera& camera);
    Camera& get_camera();
    const Camera& get_camera() const;

    // Scene management
    MeshHandle add_mesh(const Mesh& mesh);
    MaterialHandle add_material(const Material& material);
    LightHandle add_light(const std::shared_ptr<Light>& light);

    void remove_mesh(MeshHandle handle);
    void remove_material(MaterialHandle handle);
    void remove_light(LightHandle handle);

    void update_mesh(MeshHandle handle, const Mesh& mesh);
    void update_material(MaterialHandle handle, const Material& material);

    void clear_scene();

    // Ray tracing backend control
    void set_ray_tracing_backend(RayTracingBackend backend);
    RayTracingBackend get_ray_tracing_backend() const;

    // Rendering
    void begin_frame();
    void render();
    void end_frame();
    void present();

    // Frame capture
    void capture_frame_ldr(uint8_t** pixels, int* width, int* height);
    void capture_frame_hdr(float** pixels, int* width, int* height);
    void save_frame(const std::string& filename, const uint8_t* pixels, 
                   int width, int height);

    // Window control
    bool should_close() const;
    void set_should_close(bool should_close);

    // Statistics
    const RenderStats& get_stats() const;
    void reset_stats();

    // Debug visualization
    void set_gbuffer_visualization_mode(GBufferVisualizationMode mode);
    GBufferVisualizationMode get_gbuffer_visualization_mode() const;

private:
    void initialize_subsystems();
    void shutdown_subsystems();
    void rebuild_bvh_if_needed();
    void check_scene_dirty();

    AreConfig config_;                    ///< Current configuration
    
    std::unique_ptr<Window> window_;      ///< Window management
    std::unique_ptr<SceneManager> scene_manager_; ///< Scene data management
    std::unique_ptr<Rasterizer> rasterizer_; ///< Rasterization pipeline
    std::unique_ptr<RayTracer> raytracer_; ///< Ray tracing pipeline
    std::unique_ptr<TextureManager> texture_manager_; ///< Texture management
    std::unique_ptr<BVH> bvh_;            ///< BVH acceleration structure

    Camera camera_;                       ///< Active camera
    RenderStats stats_;                   ///< Rendering statistics
    
    bool scene_dirty_;                    ///< Scene needs BVH rebuild
    bool initialized_;                    ///< Initialization flag
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDERER_H
