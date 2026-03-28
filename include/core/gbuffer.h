#ifndef ARE_INCLUDE_CORE_GBUFFER_H
#define ARE_INCLUDE_CORE_GBUFFER_H

#include "basic/constants.h"
#include "basic/types.h"
#include "resource/shader.h"
#include "scene/scene.h"

namespace are {

// G-Buffer manager for deferred rendering
class GBuffer {
public:
	/*
	 * @brief Constructor
	 * @param width Buffer width
	 * @param height Buffer height
	 */
	GBuffer(uint width, uint height);

	// Destructor
	~GBuffer();

	/*
	 * @brief Initialize G-Buffer (create framebuffer and textures)
	 * @return True if initialization succeeded
	 */
	bool initialize();

	// Release G-Buffer resources
	void release();

	/*
	 * @brief Render scene to G-Buffer
	 * @param scene Scene to render
	 * @param shader Shader program for G-Buffer pass
	 */
	void render(const Scene &scene, const Shader &shader);

	/*
	 * @brief Resize G-Buffer
	 * @param width New width
	 * @param height New height
	 */
	void resize(uint width, uint height);

	/*
	 * @brief Get texture handle for specific buffer
	 * @param index Buffer index (GBUFFER_POSITION, GBUFFER_NORMAL, etc.)
	 * @return Texture handle
	 */
	TextureHandle get_texture(int index) const;

	/*
	 * @brief Get framebuffer handle
	 * @return Framebuffer handle
	 */
	FramebufferHandle get_framebuffer() const {
		return fbo_;
	}

	/*
	 * @brief Get buffer dimensions
	 * @param width Output width
	 * @param height Output height
	 */
	void get_dimensions(uint &width, uint &height) const;

private:
	uint width_;
	uint height_;
	FramebufferHandle fbo_;
	TextureHandle textures_[GBUFFER_COUNT];
	TextureHandle depth_texture_;

	bool initialized_;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_GBUFFER_H
