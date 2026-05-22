#ifndef ARE_INCLUDE_UTILS_CONFIG_H
#define ARE_INCLUDE_UTILS_CONFIG_H

#include "basic/types.h"
#include <string>
#include <unordered_map>

namespace are {

// Ray tracing configuration
struct RayTracerConfig {
	uint samples_per_pixel = 1;
	uint max_depth = 4;
	bool enable_shadows = true;
	bool enable_reflections = true;
	bool enable_accumulation = true;
	bool use_bvh = true;
};

// Super resolution configuration
struct SuperResolutionConfig {
	bool enabled = false; // Enable the super resolution mode
	uint scaling = 4; // Pixel ratio: how many times fewer pixels rendered per frame
					  // scaling=4 → 1/4 pixels, 2×2 blocks, 4 jitter positions
};

// Configuration struct for renderer
struct RendererConfig {
	uint output_width;
	uint output_height;
	RayTracerConfig rt_config;
	bool enable_denoising;
	SuperResolutionConfig sr_config;
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_CONFIG_H
