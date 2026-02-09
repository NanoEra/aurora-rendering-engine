/**
 * @file rasterizer.h
 * @brief Rasterization pipeline for G-Buffer generation
 */

#ifndef ARE_INCLUDE_RASTERIZER_RASTERIZER_H
#define ARE_INCLUDE_RASTERIZER_RASTERIZER_H

#include <are/core/types.h>
#include <memory>
#include <functional>
#include <string>

namespace are {

class GBuffer;
class ShaderProgram;
class SceneManager;
class Camera;
class Mesh;

/**
 * @struct RasterizerState
 * @brief Rasterizer fixed-function state (configurable)
 */
struct RasterizerState {
    bool enable_depth_test = true;
    bool enable_cull_face = false;
    uint32_t cull_face_mode = 0x0405; // GL_BACK
    uint32_t front_face = 0x0901;     // GL_CCW
};

class Rasterizer {
public:
    Rasterizer(int width, int height);
    ~Rasterizer();

    void resize(int width, int height);

    void render_gbuffer(const SceneManager& scene, const Camera& camera);

    GBuffer& get_gbuffer();
    const GBuffer& get_gbuffer() const;

    void upload_mesh(Mesh& mesh);
    void delete_mesh(Mesh& mesh);

    void initialize_shaders(const std::string& shader_dir);

    void set_triangle_base_provider(std::function<uint32_t(size_t)> provider);

    /**
     * @brief Set rasterizer fixed-function state
     * @param state State
     */
    void set_state(const RasterizerState& state);

private:
    void setup_mesh_buffers(Mesh& mesh);

    std::unique_ptr<GBuffer> gbuffer_;
    std::unique_ptr<ShaderProgram> gbuffer_shader_;

    std::function<uint32_t(size_t)> triangle_base_provider_;
    RasterizerState state_;

    int width_;
    int height_;
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_RASTERIZER_H
