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
	/*
	 * @brief Constructor
	 * @param width Output width
	 * @param height Output height
	 * @param config Ray tracer configuration
	 */
	RayTracer(uint width, uint height, const RayTracerConfig &config);

	// Destructor
	~RayTracer();

	/*
	 * @brief Initialize ray tracer
	 * @return True if initialization succeeded
	 */
	bool initialize(const std::shared_ptr<Shader> &shader);

	// Release resources
	void release();

	/*
	 * @brief Trace rays using G-Buffer as input
	 * @param scene Scene data
	 * @param gbuffer G-Buffer containing geometry information
	 * @param output_texture Output texture for ray traced result
	 */
	void trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_texture);

	/*
	 * @brief Resize output
	 * @param width New width
	 * @param height New height
	 */
	void resize(uint width, uint height);

	// Reset accumulation buffer
	void reset_accumulation();

	/*
	 * @brief Get current configuration
	 * @return Current configuration
	 */
	const RayTracerConfig &get_config() const {
		return config_;
	}

	/*
	 * @brief Update configuration
	 * @param config New configuration
	 */
	void set_config(const RayTracerConfig &config);

	/*
	 * @brief Rebuild BVH from scene
	 * @param scene Scene to build BVH from
	 * @return True if build succeeded
	 */
	bool rebuild_bvh(const Scene &scene);

private:
	uint width_;
	uint height_;
	RayTracerConfig config_;

	// Scene data hash for change detection
	uint materials_hash_;
	uint lights_hash_;

	// Texture arrays for PBR materials
	GLuint texture_arrays_[6]; // albedo, normal, metallic, roughness, ao, emission
	uint texture_array_sizes_[6]; // Number of textures in each array

	// Texture array caching (content hash based)
	uint texture_config_hash_; // Hash of entire texture configuration
	uint texture_slot_hashes_[6]; // Hash per slot for incremental rebuild
	bool texture_arrays_dirty_; // Dirty flag for texture arrays

	std::shared_ptr<Shader> compute_shader_;
	TextureHandle accumulation_texture_;
	BufferHandle material_buffer_;
	BufferHandle light_buffer_;

	// BVH related
	std::unique_ptr<BVH> bvh_;
	Buffer bvh_node_buffer_;
	Buffer bvh_triangle_buffer_; ///< Compact triangle data (intersection only)
	Buffer bvh_attr_buffer_; ///< Triangle attributes (fetched on hit)
	bool bvh_built_;

	uint frame_count_;
	bool initialized_;

	/*
	 * @brief Upload scene data to GPU buffers
	 * @param scene Scene to upload
	 */
	void upload_scene_data_(const Scene &scene);

	/*
	 * @brief Bind G-Buffer textures to compute shader
	 * @param gbuffer G-Buffer to bind
	 */
	void bind_gbuffer_(const GBuffer &gbuffer);

	/*
	 * @brief Build texture arrays from scene materials
	 * @param scene Scene containing materials
	 */
	void build_texture_arrays_(const Scene &scene);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RAYTRACER_H
