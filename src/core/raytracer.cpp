#include "core/raytracer.h"
#include "basic/constants.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

namespace {
	uint fnv1a_hash_bytes(const void *data, size_t size) {
		const uint8_t *bytes = static_cast<const uint8_t *>(data);
		uint h = 2166136261u;
		for (size_t i = 0; i < size; ++i) {
			h ^= bytes[i];
			h *= 16777619u;
		}
		return h;
	}

	// Compute hash of texture pointers for a specific slot
	uint compute_slot_texture_hash(const std::vector<std::shared_ptr<Texture>> &textures) {
		if (textures.empty())
			return 0u;
		// Hash the raw pointers to detect texture set changes
		std::vector<const void *> ptrs;
		ptrs.reserve(textures.size());
		for (const auto &t : textures) {
			ptrs.push_back(t.get());
		}
		return fnv1a_hash_bytes(ptrs.data(), ptrs.size() * sizeof(void *));
	}
} // namespace

RayTracer::RayTracer(uint width, uint height, const RayTracerConfig &config)
	: width_(width)
	, height_(height)
	, config_(config)
	, materials_hash_(0u)
	, lights_hash_(0u)
	, texture_config_hash_(0u)
	, texture_arrays_dirty_(true)
	, accumulation_texture_(INVALID_HANDLE)
	, material_buffer_(INVALID_HANDLE)
	, light_buffer_(INVALID_HANDLE)
	, bvh_(nullptr)
	, bvh_built_(false)
	, frame_count_(0)
	, initialized_(false) {
	for (int i = 0; i < 6; ++i) {
		texture_slot_hashes_[i] = 0u;
	}
}

RayTracer::~RayTracer() {
	release();
}

bool RayTracer::initialize(const std::shared_ptr<Shader> &shader) {
	if (initialized_) {
		ARE_LOG_WARN("RayTracer already initialized");
		return true;
	}

	ResourceManager &rm = ResourceManager::instance();

	compute_shader_ = shader;

	// Create accumulation texture
	accumulation_texture_ = rm.create_texture(width_, height_, TextureFormat::RGBA32F);

	// Create shader storage buffers
	BufferDescription ssbo_desc;
	ssbo_desc.type = BufferType::SHADER_STORAGE_BUFFER;
	ssbo_desc.usage = BufferUsage::DYNAMIC_DRAW;
	ssbo_desc.size = 1;
	ssbo_desc.data = nullptr;
	material_buffer_ = rm.create_buffer(ssbo_desc);
	light_buffer_ = rm.create_buffer(ssbo_desc);

	// Initialize texture arrays (empty for now)
	for (int i = 0; i < 6; i++) {
		texture_arrays_[i] = 0;
		texture_array_sizes_[i] = 0;
	}

	// Initialize BVH if enabled
	if (config_.use_bvh_) {
		bvh_ = std::make_unique<BVH>();
	}

	initialized_ = true;
	ARE_LOG_INFO("RayTracer initialized successfully");
	return true;
}

void RayTracer::release() {
	if (!initialized_)
		return;

	ResourceManager &rm = ResourceManager::instance();

	if (accumulation_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(accumulation_texture_);
		accumulation_texture_ = INVALID_HANDLE;
	}

	// Release texture arrays
	for (int i = 0; i < 6; i++) {
		if (texture_arrays_[i] != 0) {
			rm.destroy_texture_array(texture_arrays_[i]);
			texture_arrays_[i] = 0;
		}
	}

	if (material_buffer_ != INVALID_HANDLE) {
		rm.destroy_buffer(material_buffer_);
		material_buffer_ = INVALID_HANDLE;
	}

	if (light_buffer_ != INVALID_HANDLE) {
		rm.destroy_buffer(light_buffer_);
		light_buffer_ = INVALID_HANDLE;
	}

	bvh_node_buffer_.release();
	bvh_triangle_buffer_.release();
	bvh_attr_buffer_.release();

	bvh_.reset();
	bvh_built_ = false;

	initialized_ = false;
	ARE_LOG_INFO("RayTracer released");
}

bool RayTracer::rebuild_bvh(const Scene &scene) {
	if (!config_.use_bvh_) {
		ARE_LOG_WARN("BVH is disabled in configuration");
		return false;
	}

	if (!bvh_) {
		bvh_ = std::make_unique<BVH>();
	}

	ARE_LOG_INFO("Building BVH for ray tracing...");

	if (!bvh_->build(scene.get_meshes())) {
		ARE_LOG_ERROR("Failed to build BVH");
		return false;
	}

	if (!bvh_->upload_to_gpu(bvh_node_buffer_, bvh_triangle_buffer_, bvh_attr_buffer_)) {
		ARE_LOG_ERROR("Failed to upload BVH to GPU");
		return false;
	}

	bvh_built_ = true;
	reset_accumulation();
	ARE_LOG_INFO("BVH built and uploaded successfully");
	return true;
}

void RayTracer::trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_texture) {
	if (!initialized_) {
		ARE_LOG_ERROR("RayTracer not initialized");
		return;
	}

	if (!compute_shader_->is_valid()) {
		ARE_LOG_ERROR("Ray tracing compute shader not loaded");
		return;
	}

	// Build BVH if enabled and not built yet
	if (config_.use_bvh_ && !bvh_built_) {
		rebuild_bvh(scene);
	}

	// Build texture arrays BEFORE uploading materials (so indices are available)
	const auto &materials = scene.get_materials();
	bool has_textures = false;
	for (const auto &mat : materials) {
		if (mat->has_texture(TextureSlot::ALBEDO) || mat->has_texture(TextureSlot::NORMAL) || mat->has_texture(TextureSlot::METALLIC) || mat->has_texture(TextureSlot::ROUGHNESS) || mat->has_texture(TextureSlot::AO) || mat->has_texture(TextureSlot::EMISSION)) {
			has_textures = true;
			break;
		}
	}
	if (has_textures) {
		build_texture_arrays_(scene);

		// Bind texture arrays
		glActiveTexture(GL_TEXTURE10);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[0]);
		glActiveTexture(GL_TEXTURE11);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[1]);
		glActiveTexture(GL_TEXTURE12);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[2]);
		glActiveTexture(GL_TEXTURE13);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[3]);
		glActiveTexture(GL_TEXTURE14);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[4]);
		glActiveTexture(GL_TEXTURE15);
		glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[5]);
	}

	// Upload scene data (materials now have correct texture indices)
	upload_scene_data_(scene);

	// Use compute shader
	if (!compute_shader_ || !compute_shader_->is_valid()) {
		ARE_LOG_ERROR("Ray tracing compute shader not set or invalid");
		return;
	}
	compute_shader_->use();

	// Bind G-Buffer textures
	bind_gbuffer_(gbuffer);

	// Bind output and accumulation textures
	glBindImageTexture(3, output_texture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
	glBindImageTexture(4, accumulation_texture_, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);

	// Bind BVH buffers if enabled
	if (config_.use_bvh_ && bvh_built_) {
		bvh_node_buffer_.bind_base(2);
		bvh_triangle_buffer_.bind_base(3);
		bvh_attr_buffer_.bind_base(4);
		compute_shader_->set_bool("u_use_bvh", true);
		compute_shader_->set_uint("u_bvh_node_count", bvh_->get_node_count());
	} else {
		compute_shader_->set_bool("u_use_bvh", false);
	}

	// Set uniforms
	compute_shader_->set_uint("u_frame_count", frame_count_);
	compute_shader_->set_uint("u_samples_per_pixel", config_.samples_per_pixel_);
	compute_shader_->set_uint("u_max_depth", config_.max_depth_);
	compute_shader_->set_uint("u_light_count", static_cast<uint>(scene.get_lights().size()));
	compute_shader_->set_bool("u_enable_accumulation", config_.enable_accumulation_);

	// Enable/disable textures based on material usage
	compute_shader_->set_bool("u_enable_textures", has_textures);

	// Set camera data
	const Camera &camera = scene.get_camera();
	Mat4 inv_vp = glm::inverse(camera.get_view_projection_matrix());
	compute_shader_->set_mat4("u_inv_view_projection", inv_vp);

	// Dispatch compute shader
	uint num_groups_x = (width_ + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
	uint num_groups_y = (height_ + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;

	glDispatchCompute(num_groups_x, num_groups_y, 1);

	// Memory barrier
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	// Increment frame count for accumulation
	if (config_.enable_accumulation_) {
		frame_count_++;
	}
}

void RayTracer::resize(uint width, uint height) {
	if (width == width_ && height == height_)
		return;

	width_ = width;
	height_ = height;

	if (initialized_) {
		ResourceManager &rm = ResourceManager::instance();

		// Recreate accumulation texture
		if (accumulation_texture_ != INVALID_HANDLE) {
			rm.destroy_texture(accumulation_texture_);
		}

		accumulation_texture_ = rm.create_texture(width_, height_, TextureFormat::RGBA32F);

		reset_accumulation();
	}
}

void RayTracer::reset_accumulation() {
	frame_count_ = 0;
}

void RayTracer::set_config(const RayTracerConfig &config) {
	bool bvh_changed = (config.use_bvh_ != config_.use_bvh_);

	config_ = config;
	reset_accumulation();

	if (bvh_changed) {
		if (config_.use_bvh_ && !bvh_) {
			bvh_ = std::make_unique<BVH>();
			bvh_built_ = false;
		} else if (!config_.use_bvh_) {
			bvh_.reset();
			bvh_built_ = false;
		}
	}
}

void RayTracer::upload_scene_data_(const Scene &scene) {
	// Upload materials (on change only)
	const auto &materials = scene.get_materials();
	if (!materials.empty()) {
		// Aligned to match GLSL std430 layout (vec3 = vec4 = 16 bytes)
		struct MaterialData {
			alignas(16) Vec3 albedo;
			alignas(16) Vec3 emission;
			float metallic;
			float roughness;
			int type;
			float ior;
			float ao;
			float padding1;
			uint texture_handles[6];
		};

		std::vector<MaterialData> material_data;
		material_data.reserve(materials.size());

		for (const auto &mat : materials) {
			MaterialData data {};
			data.albedo = mat->get_albedo();
			data.metallic = mat->get_metallic();
			data.emission = mat->get_emission();
			data.roughness = mat->get_roughness();
			data.type = static_cast<int>(mat->get_type());
			data.ior = mat->get_ior();
			data.ao = 1.0f; // default: no AO

			// Texture array indices (0 = no texture, 1+ = index into array)
			data.texture_handles[0] = mat->get_texture_index(TextureSlot::ALBEDO);
			data.texture_handles[1] = mat->get_texture_index(TextureSlot::NORMAL);
			data.texture_handles[2] = mat->get_texture_index(TextureSlot::METALLIC);
			data.texture_handles[3] = mat->get_texture_index(TextureSlot::ROUGHNESS);
			data.texture_handles[4] = mat->get_texture_index(TextureSlot::AO);
			data.texture_handles[5] = mat->get_texture_index(TextureSlot::EMISSION);

			material_data.push_back(data);
		}

		uint h = fnv1a_hash_bytes(material_data.data(), material_data.size() * sizeof(MaterialData));
		if (h != materials_hash_) {
			materials_hash_ = h;

			glBindBuffer(GL_SHADER_STORAGE_BUFFER, material_buffer_);
			glBufferData(GL_SHADER_STORAGE_BUFFER,
				material_data.size() * sizeof(MaterialData),
				material_data.data(), GL_DYNAMIC_DRAW);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);

			reset_accumulation(); // materials changed => invalidate accumulation
		} else {
			// Still ensure bound (in case other code changed bindings)
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);
		}
	} else {
		materials_hash_ = 0u;
	}

	// Upload lights (on change only)
	const auto &lights = scene.get_lights();
	if (!lights.empty()) {
		struct LightData {
			Vec3 position;
			int type;
			Vec3 direction;
			float intensity;
			Vec3 color;
			float range;
			Vec2 spot_angles;
			Vec2 padding;
		};

		std::vector<LightData> light_data;
		light_data.reserve(lights.size());

		for (const auto &light : lights) {
			LightData data {};
			data.position = light->get_position();
			data.type = static_cast<int>(light->get_type());
			data.direction = light->get_direction();
			data.intensity = light->get_intensity();
			data.color = light->get_color();
			data.range = light->get_range();
			data.spot_angles = Vec2(light->get_inner_angle(), light->get_outer_angle());
			light_data.push_back(data);
		}

		uint h = fnv1a_hash_bytes(light_data.data(), light_data.size() * sizeof(LightData));
		if (h != lights_hash_) {
			lights_hash_ = h;

			glBindBuffer(GL_SHADER_STORAGE_BUFFER, light_buffer_);
			glBufferData(GL_SHADER_STORAGE_BUFFER,
				light_data.size() * sizeof(LightData),
				light_data.data(), GL_DYNAMIC_DRAW);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);

			reset_accumulation(); // lights changed => invalidate accumulation
		} else {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);
		}
	} else {
		lights_hash_ = 0u;
	}
}

void RayTracer::bind_gbuffer_(const GBuffer &gbuffer) {
	glBindImageTexture(0, gbuffer.get_texture(GBUFFER_POSITION), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(1, gbuffer.get_texture(GBUFFER_NORMAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RG32F); // Octahedral encoded

	glBindImageTexture(5, gbuffer.get_texture(GBUFFER_MATERIAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(6, gbuffer.get_texture(GBUFFER_MATERIAL_ID), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32UI);

	// Texcoord
	glBindImageTexture(7, gbuffer.get_texture(GBUFFER_TEXCOORD), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);

	// Tangent
	glBindImageTexture(8, gbuffer.get_texture(GBUFFER_TANGENT), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
}

void RayTracer::build_texture_arrays_(const Scene &scene) {
	const auto &materials = scene.get_materials();

	// Collect all textures for each slot
	std::vector<std::shared_ptr<Texture>> textures[6];

	for (const auto &mat : materials) {
		for (int slot = 0; slot < 6; slot++) {
			auto tex = mat->get_texture(static_cast<TextureSlot>(slot));
			if (tex && tex->is_valid()) {
				// Check if texture already added (use set for O(1) lookup)
				bool found = false;
				for (const auto &t : textures[slot]) {
					if (t.get() == tex.get()) {
						found = true;
						break;
					}
				}
				if (!found) {
					textures[slot].push_back(tex);
				}
			}
		}
	}

	// Compute hash for each slot and check if rebuild is needed
	bool any_slot_dirty = false;
	uint new_slot_hashes[6];
	for (int slot = 0; slot < 6; slot++) {
		new_slot_hashes[slot] = compute_slot_texture_hash(textures[slot]);
		if (new_slot_hashes[slot] != texture_slot_hashes_[slot]) {
			any_slot_dirty = true;
		}
	}

	// If no slots changed, skip entire rebuild
	if (!any_slot_dirty && !texture_arrays_dirty_) {
		// Still need to bind existing arrays
		for (int slot = 0; slot < 6; slot++) {
			if (texture_arrays_[slot] != 0) {
				glActiveTexture(GL_TEXTURE10 + slot);
				glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
			}
		}
		return;
	}

	ResourceManager &rm = ResourceManager::instance();

	// Build arrays only for dirty slots
	for (int slot = 0; slot < 6; slot++) {
		// Skip if this slot hasn't changed
		if (new_slot_hashes[slot] == texture_slot_hashes_[slot] && !texture_arrays_dirty_) {
			// Bind existing array
			if (texture_arrays_[slot] != 0) {
				glActiveTexture(GL_TEXTURE10 + slot);
				glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
			}
			continue;
		}

		// Destroy previous texture array if exists
		if (texture_arrays_[slot] != 0) {
			rm.destroy_texture_array(texture_arrays_[slot]);
			texture_arrays_[slot] = 0;
		}

		if (textures[slot].empty()) {
			texture_array_sizes_[slot] = 0;
			texture_slot_hashes_[slot] = 0u;
			continue;
		}

		texture_array_sizes_[slot] = static_cast<uint>(textures[slot].size());

		// Create texture array using ResourceManager
		TextureArrayDescription desc;
		desc.textures = textures[slot];
		desc.filter = TextureFilter::LINEAR;
		desc.wrap = TextureWrap::REPEAT;

		texture_arrays_[slot] = rm.create_texture_array(desc);

		// Set texture index on all materials using each texture
		for (size_t i = 0; i < textures[slot].size(); i++) {
			// Index is i+1 because 0 means "no texture" in the shader
			uint32_t array_index = static_cast<uint32_t>(i) + 1;
			for (const auto &mat : materials) {
				if (mat->get_texture(static_cast<TextureSlot>(slot)).get() == textures[slot][i].get()) {
					mat->set_texture_index(static_cast<TextureSlot>(slot), array_index);
				}
			}
		}

		// Update slot hash
		texture_slot_hashes_[slot] = new_slot_hashes[slot];

		// Bind the newly created array
		if (texture_arrays_[slot] != 0) {
			glActiveTexture(GL_TEXTURE10 + slot);
			glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
		}
	}

	texture_arrays_dirty_ = false;
}

} // namespace are
