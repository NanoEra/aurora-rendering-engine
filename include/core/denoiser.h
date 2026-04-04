#ifndef ARE_INCLUDE_CORE_DENOISER_H
#define ARE_INCLUDE_CORE_DENOISER_H

#include "basic/types.h"
#include "resource/shader.h"
#include <memory>

namespace are {

// Bilateral filter denoiser with temporal accumulation
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
	 * @brief Apply bilateral filter with optional temporal accumulation
	 * @param input_texture RGBA32F input texture
	 * @param radius Filter radius (1 => 3x3)
	 * @param temporal_weight Weight for temporal blending (0 = no temporal, 1 = full history)
	 * @return Output texture handle (internal)
	 */
	TextureHandle denoise(TextureHandle input_texture, int radius, float temporal_weight = 0.0f);

	/**
	 * @brief Reset temporal history (call on scene change)
	 */
	void reset_history();

private:
	uint width_;
	uint height_;
	std::shared_ptr<Shader> shader_;
	TextureHandle output_texture_;
	TextureHandle history_texture_;  // Previous frame for temporal accumulation
	bool history_valid_;             // Whether history contains valid data
	bool initialized_;

	// Create output texture
	void create_output_texture_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_DENOISER_H
