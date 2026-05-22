#ifndef ARE_INCLUDE_CORE_RAYTRACER_H
#define ARE_INCLUDE_CORE_RAYTRACER_H

#include "basic/types.h"
#include "core/bvh.h"
#include "core/gbuffer.h"
#include "resource/buffer.h"
#include "resource/shader.h"
#include "scene/scene.h"
#include "utils/config.h"
#include <glad/glad.h>
#include <memory>

namespace are {

// Compute shader based ray tracer
class RayTracer {
public:
	RayTracer(uint width, uint height, const RayTracerConfig &config);
	~RayTracer();

	bool initialize(const std::shared_ptr<Shader> &shader);
	void release();

	/*
	 * @brief Trace rays using G-Buffer as input
	 * @param scene        Scene data
	 * @param gbuffer      G-Buffer containing geometry information
	 * @param output_image Low-resolution output (W/block × H/block in SR mode)
	 * @param sr_scaling   Pixel ratio for super resolution (1 = disabled, e.g. 4 = 4× fewer pixels)
	 * @param sr_jitter    Jitter frame index 0..scaling-1
	 * @param sr_accum     Full-resolution accumulation texture (SR mode only, 0 = disabled)
	 */
	void trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_image,
		uint sr_scaling = 1, uint sr_jitter = 0, TextureHandle sr_accum = 0);

	void resize(uint width, uint height);
	void reset_accumulation();

	const RayTracerConfig &get_config() const {
		return config_;
	}
	void set_config(const RayTracerConfig &config);
	bool rebuild_bvh(const Scene &scene);

private:
	uint width_, height_;
	RayTracerConfig config_;

	uint materials_hash_;
	uint lights_hash_;

	GLuint texture_arrays_[6];
	uint texture_array_sizes_[6];
	uint texture_config_hash_;
	uint texture_slot_hashes_[6];
	bool texture_arrays_dirty_;

	std::shared_ptr<Shader> compute_shader_;
	TextureHandle accumulation_texture_;
	BufferHandle material_buffer_;
	BufferHandle light_buffer_;

	std::unique_ptr<BVH> bvh_;
	Buffer bvh_node_buffer_;
	Buffer bvh_triangle_buffer_;
	Buffer bvh_attr_buffer_;
	bool bvh_built_;

	uint frame_count_;
	bool initialized_;

	void upload_scene_data_(const Scene &scene);
	void bind_gbuffer_(const GBuffer &gbuffer);
	void build_texture_arrays_(const Scene &scene);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RAYTRACER_H
