/**
 * @file config.h
 * @brief Configuration system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_CONFIG_H
#define ARE_INCLUDE_CORE_CONFIG_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @struct WindowConfig
 * @brief Configuration for window creation
 */
struct WindowConfig {
	int width = 1280; ///< Window width in pixels
	int height = 720; ///< Window height in pixels
	std::string title = "Aurora Rendering Engine"; ///< Window title
	bool resizable = true; ///< Whether window is resizable
	bool vsync = true; ///< Enable vertical synchronization
	int samples = 1; ///< MSAA samples (1 = disabled)
};

/**
 * @struct RayTracingConfig
 * @brief Configuration for ray tracing
 */
struct RayTracingConfig {
	RayTracingBackend backend = RayTracingBackend::ARE_RT_BACKEND_COMPUTE_SHADER;
	int spp = 64; ///< Samples per pixel
	int max_depth = 8; ///< Maximum ray bounce depth
	bool enable_gi = true; ///< Enable global illumination
	bool enable_ao = true; ///< Enable ambient occlusion
	bool enable_soft_shadows = false; ///< Enable soft shadows
	int ao_samples = 16; ///< AO sample count
	Real ao_radius = 1.0f; ///< AO sampling radius
};

/**
 * @struct RenderConfig
 * @brief General rendering configuration
 */
struct RenderConfig {
	ToneMappingOperator tonemap_op = ToneMappingOperator::ARE_TONEMAP_ACES;
	Real exposure = 1.0f; ///< Exposure value for tone mapping
	bool use_hdr = true; ///< Use HDR rendering pipeline
	GBufferVisualizationMode gbuffer_vis_mode = GBufferVisualizationMode::ARE_GBUFFER_VIS_NONE;
};

/**
 * @struct PerformanceConfig
 * @brief Performance-related configuration
 */
struct PerformanceConfig {
	int num_threads = 0; ///< Number of threads (0 = auto-detect)
	bool enable_bvh_multithreading = true; ///< Use multithreading for BVH construction
	bool enable_profiling = false; ///< Enable performance profiling
};

/**
 * @struct PathConfig
 * @brief File path configuration
 */
struct PathConfig {
	std::string shader_dir = "shaders/"; ///< Directory containing shader files
	std::string texture_dir = "textures/"; ///< Default texture directory
	std::string output_dir = "output/"; ///< Default output directory
};

/**
 * @class AreConfig
 * @brief Main configuration class for Aurora Rendering Engine
 */
class AreConfig {
public:
	WindowConfig window; ///< Window configuration
	RayTracingConfig ray_tracing; ///< Ray tracing configuration
	RenderConfig render; ///< Rendering configuration
	PerformanceConfig performance; ///< Performance configuration
	PathConfig paths; ///< Path configuration

	/**
	 * @brief Constructor with default values
	 */
	AreConfig() = default;

	/**
	 * @brief Validate configuration values
	 * @return true if configuration is valid, false otherwise
	 */
	bool validate() const;

	/**
	 * @brief Print configuration to console
	 */
	void print() const;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_CONFIG_H
