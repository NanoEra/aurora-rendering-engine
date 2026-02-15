#ifndef ARE_INCLUDE_CORE_DENOISER_H
#define ARE_INCLUDE_CORE_DENOISER_H

#include "basic/types.h"
#include "resource/shader.h"
#include <memory>

namespace are {

// Mean filter denoiser using compute shader
class Denoiser {
public:
	/**
	 * @brief Construct denoiser
	 * @param width Output width
	 * @param height Output height
	 */
	Denoiser(uint width, uint height);

	/**
	 * @brief Destroy denoiser
	 */
	~Denoiser();

	/**
	 * @brief Initialize GPU resources
	 * @param shader Denoise compute shader (managed by ShaderManager)
	 * @return True on success
	 */
	bool initialize(const std::shared_ptr<Shader> &shader);

	/**
	 * @brief Release GPU resources
	 */
	void release();

	/**
	 * @brief Resize internal targets
	 * @param width New width
	 * @param height New height
	 */
	void resize(uint width, uint height);

	/**
	 * @brief Apply mean filter
	 * @param input_texture RGBA32F input texture
	 * @param radius Filter radius (1 => 3x3)
	 * @return Output texture handle (internal)
	 */
	TextureHandle denoise(TextureHandle input_texture, int radius);

private:
	uint width_;
	uint height_;
	std::shared_ptr<Shader> shader_;
	TextureHandle output_texture_;
	bool initialized_;

	// Create output texture
	void create_output_texture_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_DENOISER_H
