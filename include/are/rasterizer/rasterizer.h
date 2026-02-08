/**
 * @file rasterizer.h
 * @brief Rasterization pipeline for G-Buffer generation
 */

#ifndef ARE_INCLUDE_RASTERIZER_RASTERIZER_H
#define ARE_INCLUDE_RASTERIZER_RASTERIZER_H

#include <are/core/types.h>
#include <are/core/config.h>
#include <memory>

namespace are {

// Forward declarations
class GBuffer;
class ShaderProgram;
class SceneManager;
class Camera;
class Mesh;

/**
 * @class Rasterizer
 * @brief OpenGL rasterization pipeline
 * 
 * Renders scene geometry to G-Buffer using traditional rasterization.
 */
class Rasterizer {
public:
    /**
     * @brief Constructor
     * @param width Framebuffer width
     * @param height Framebuffer height
     */
    Rasterizer(int width, int height);

    /**
     * @brief Destructor
     */
    ~Rasterizer();

    /**
     * @brief Resize framebuffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Render scene to G-Buffer
     * @param scene Scene manager
     * @param camera Camera
     */
    void render_gbuffer(const SceneManager& scene, const Camera& camera);

    /**
     * @brief Get G-Buffer
     * @return G-Buffer reference
     */
    GBuffer& get_gbuffer();
    const GBuffer& get_gbuffer() const;

    /**
     * @brief Upload mesh data to GPU
     * @param mesh Mesh to upload
     */
    void upload_mesh(Mesh& mesh);

    /**
     * @brief Delete mesh GPU resources
     * @param mesh Mesh to delete
     */
    void delete_mesh(Mesh& mesh);

private:
    void initialize_shaders(const std::string& shader_dir);
    void setup_mesh_buffers(Mesh& mesh);

    std::unique_ptr<GBuffer> gbuffer_;    ///< G-Buffer
    std::unique_ptr<ShaderProgram> gbuffer_shader_; ///< G-Buffer shader
    
    int width_;                           ///< Framebuffer width
    int height_;                          ///< Framebuffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_RASTERIZER_H
