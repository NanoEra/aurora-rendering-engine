#ifndef ARE_INCLUDE_CORE_RAYTRACER_H
#define ARE_INCLUDE_CORE_RAYTRACER_H

#include "basic/types.h"
#include "core/bvh.h" // 添加
#include "core/gbuffer.h"
#include "resource/buffer.h"
#include "resource/shader.h"
#include "scene/scene.h"
#include <memory>

namespace are {

/// @brief Ray tracing configuration
struct RayTracerConfig {
	uint samples_per_pixel_;
	uint max_depth_;
	bool enable_shadows_;
	bool enable_reflections_;
	bool enable_accumulation_;
	bool use_bvh_; // 添加BVH开关
};

/// @brief Compute shader based ray tracer
class RayTracer {
public:
	/// @brief Constructor
	/// @param width Output width
	/// @param height Output height
	/// @param config Ray tracer configuration
	RayTracer(uint width, uint height, const RayTracerConfig &config);

	/// @brief Destructor
	~RayTracer();

	/// @brief Initialize ray tracer
	/// @return True if initialization succeeded
	bool initialize(const std::shared_ptr<Shader> &shader);

	/// @brief Release resources
	void release();

	/// @brief Trace rays using G-Buffer as input
	/// @param scene Scene data
	/// @param gbuffer G-Buffer containing geometry information
	/// @param output_texture Output texture for ray traced result
	void trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_texture);

	/// @brief Resize output
	/// @param width New width
	/// @param height New height
	void resize(uint width, uint height);

	/// @brief Reset accumulation buffer
	void reset_accumulation();

	/// @brief Get current configuration
	/// @return Current configuration
	const RayTracerConfig &get_config() const {
		return config_;
	}

	/// @brief Update configuration
	/// @param config New configuration
	void set_config(const RayTracerConfig &config);

	/// @brief Rebuild BVH from scene
	/// @param scene Scene to build BVH from
	/// @return True if build succeeded
	bool rebuild_bvh(const Scene &scene);

private:
	uint width_;
	uint height_;
	RayTracerConfig config_;

	std::shared_ptr<Shader> compute_shader_;
	TextureHandle accumulation_texture_;
	BufferHandle scene_buffer_;
	BufferHandle material_buffer_;
	BufferHandle light_buffer_;

	// BVH related
	std::unique_ptr<BVH> bvh_; // 添加
	Buffer bvh_node_buffer_; // 添加
	Buffer bvh_triangle_buffer_; // 添加
	bool bvh_built_; // 添加

	uint materials_hash_;
	uint lights_hash_;

	uint frame_count_;
	bool initialized_;

	/// @brief Upload scene data to GPU buffers
	/// @param scene Scene to upload
	void upload_scene_data_(const Scene &scene);

	/// @brief Bind G-Buffer textures to compute shader
	/// @param gbuffer G-Buffer to bind
	void bind_gbuffer_(const GBuffer &gbuffer);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RAYTRACER_H
