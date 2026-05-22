#ifndef ARE_INCLUDE_CORE_SUPER_RESOLUTION_H
#define ARE_INCLUDE_CORE_SUPER_RESOLUTION_H

#include "basic/types.h"
#include "resource/shader.h"
#include "utils/config.h"
#include <memory>

namespace are {

// Super resolution sparse ray tracing system
// Renders a subset of pixels each frame via a jitter pattern and
// accumulates across multiple frames inside the RT compute shader.
class SuperResolution {
public:
	/*
	 * @brief Constructor
	 * @param full_width  Full-resolution output width
	 * @param full_height Full-resolution output height
	 * @param config      Super resolution configuration (enabled, scaling)
	 */
	SuperResolution(uint full_width, uint full_height, const SuperResolutionConfig &config);

	// Destructor
	~SuperResolution();

	/*
	 * @brief Allocate textures and bind the upscale compute shader
	 * @param shader Super-resolution compute shader (super_resolution.comp)
	 * @return True if successful
	 */
	bool initialize(const std::shared_ptr<Shader> &shader);

	// Release all GPU resources
	void release();

	/*
	 * @brief Low-resolution texture written by RayTracer each frame.
	 *        Dimensions are W/scaling × H/scaling.
	 * @return Texture handle
	 */
	TextureHandle get_low_res_rt_texture() const {
		return low_res_rt_texture_;
	}

	/*
	 * @brief Full-resolution accumulation buffer (binding 4 in the RT shader).
	 *        RGB = running average colour, A = 1.0 once sampled.
	 * @return Texture handle
	 */
	TextureHandle get_accumulated_rt_texture() const {
		return accumulated_rt_texture_;
	}

	/*
	 * @brief Run the upscale compute pass: tonemap accumulated RT, output final image.
	 *        Unrendered pixels (alpha == 0) appear black.
	 * @return Upscaled output texture handle (full resolution)
	 */
	TextureHandle upscale();

	/*
	 * @brief Advance to the next jitter frame within the current cycle.
	 *        Cycles from 0 to scaling-1, wrapping back to 0.
	 */
	void advance_jitter_frame();

	/*
	 * @brief Reset jitter frame to 0 and clear the accumulation texture to black.
	 *        Called on scene changes and initialisation.
	 */
	void reset_accumulation();

	/*
	 * @brief Resize all internal textures for a new output resolution
	 * @param full_width  New full-resolution width
	 * @param full_height New full-resolution height
	 */
	void resize(uint full_width, uint full_height);

	/*
	 * @brief Current jitter frame index (0 .. scaling-1)
	 * @return Jitter frame index
	 */
	uint get_current_jitter_frame() const {
		return current_jitter_frame_;
	}

	/*
	 * @brief Read-only access to the active configuration
	 * @return Current SuperResolutionConfig
	 */
	const SuperResolutionConfig &get_config() const {
		return config_;
	}

	/*
	 * @brief Low-resolution dispatch width (full_width / sqrt(scaling))
	 * @return Width in pixels
	 */
	uint get_low_res_width() const {
		return low_res_w_;
	}

	/*
	 * @brief Low-resolution dispatch height (full_height / sqrt(scaling))
	 * @return Height in pixels
	 */
	uint get_low_res_height() const {
		return low_res_h_;
	}

private:
	uint full_width_, full_height_, low_res_w_, low_res_h_;
	SuperResolutionConfig config_;
	uint current_jitter_frame_;

	TextureHandle low_res_rt_texture_; // W/block × H/block – per-frame RT output
	TextureHandle accumulated_rt_texture_; // W × H – running average (binding 4)
	TextureHandle upscaled_texture_; // W × H – final tonemapped output

	std::shared_ptr<Shader> compute_shader_;
	bool initialized_ = false;

	// Create / recreate all internal textures
	void create_textures_();

	// sqrt(scaling) – the block side length in pixels
	uint compute_block_size_() const;

	// Clears accumulated_rt_texture_ to (0,0,0,0) via FBO colour clear
	void clear_accumulation_texture_() const;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SUPER_RESOLUTION_H
