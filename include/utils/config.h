#ifndef ARE_INCLUDE_UTILS_CONFIG_H
#define ARE_INCLUDE_UTILS_CONFIG_H

#include <string>
#include <unordered_map>

namespace are {

// Configuration struct for renderer
struct RendererConfig {
	uint width_;
	uint height_;
	uint samples_per_pixel_;
	uint max_ray_depth_;
	bool enable_denoising_;
	bool enable_accumulation_;
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_CONFIG_H
