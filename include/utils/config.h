#ifndef ARE_INCLUDE_UTILS_CONFIG_H
#define ARE_INCLUDE_UTILS_CONFIG_H

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


// Configuration struct for renderer
struct RendererConfig {
	uint output_width;
	uint output_height;
	RayTracerConfig rt_config;
	bool enable_denoising;
	bool enable_sr; // Enable the super resolution mode
	double sr_scaling; // The magnification of super-resolution
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_CONFIG_H
