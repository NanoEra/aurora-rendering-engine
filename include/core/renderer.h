#ifndef ARE_INCLUDE_CORE_RENDERER_H
#define ARE_INCLUDE_CORE_RENDERER_H

#include "basic/types.h"
#include "core/denoiser.h"
#include "core/gbuffer.h"
#include "core/raytracer.h"
#include "core/screen_blit.h"
#include "core/shader_manager.h"
#include "scene/scene.h"
#include <memory>

namespace are {

// Main renderer configuration
struct RendererConfig {
	uint width_;
	uint height_;
	uint samples_per_pixel_;
	uint max_ray_depth_;
	bool enable_denoising_;
	bool enable_accumulation_;
};

// Main rendering engine interface
class Renderer {
public:
	/*
	 * @brief Constructor
	 * @param config Renderer configuration
	 */
	Renderer(const RendererConfig &config);

	// Destructor
	~Renderer();

	/*
	 * @brief Initialize renderer (OpenGL context must be current)
	 * @return True if initialization succeeded
	 */
	bool initialize();

	// Shutdown renderer and release resources
	void shutdown();

	/*
	 * @brief Render a frame
	 * @param scene Scene to render
	 * @param output_texture Output texture handle (0 for default framebuffer)
	 * @return Rendering statistics
	 */
	RenderStats render(const Scene &scene, TextureHandle output_texture = 0);

	/*
	 * @brief Resize render targets
	 * @param width New width
	 * @param height New height
	 */
	void resize(uint width, uint height);

	/*
	 * @brief Get current configuration
	 * @return Current configuration
	 */
	const RendererConfig &get_config() const {
		return config_;
	}

	/*
	 * @brief Update configuration
	 * @param config New configuration
	 */
	void set_config(const RendererConfig &config);

	// Notify scene changed to rebuild acceleration
	void notify_scene_changed(const Scene &scene);

private:
	RendererConfig config_;
	std::unique_ptr<GBuffer> gbuffer_;
	std::unique_ptr<RayTracer> raytracer_;
	std::unique_ptr<ShaderManager> shader_manager_;
	std::unique_ptr<ScreenBlit> screen_blit_;
	std::unique_ptr<Denoiser> denoiser_;

	bool initialized_;
	uint frame_count_;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RENDERER_H
