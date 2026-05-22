#include "core/raytracer.h"
#include "basic/constants.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <cmath>
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

	uint compute_slot_texture_hash(const std::vector<std::shared_ptr<Texture>> &textures) {
		if (textures.empty())
			return 0u;
		std::vector<const void *> ptrs;
		ptrs.reserve(textures.size());
		for (const auto &t : textures)
			ptrs.push_back(t.get());
		return fnv1a_hash_bytes(ptrs.data(), ptrs.size() * sizeof(void *));
	}
} // namespace

RayTracer::RayTracer(uint width, uint height, const RayTracerConfig &config)
	: width_(width), height_(height), config_(config), materials_hash_(0u), lights_hash_(0u), texture_config_hash_(0u), texture_arrays_dirty_(true), accumulation_texture_(INVALID_HANDLE), material_buffer_(INVALID_HANDLE), light_buffer_(INVALID_HANDLE), bvh_(nullptr), bvh_built_(false), frame_count_(0), initialized_(false) {
	for (int i = 0; i < 6; ++i)
		texture_slot_hashes_[i] = 0u;
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
	accumulation_texture_ = rm.create_texture(width_, height_, TextureFormat::RGBA32F);
	BufferDescription ssbo_desc;
	ssbo_desc.type = BufferType::SHADER_STORAGE_BUFFER;
	ssbo_desc.usage = BufferUsage::DYNAMIC_DRAW;
	ssbo_desc.size = 1;
	ssbo_desc.data = nullptr;
	material_buffer_ = rm.create_buffer(ssbo_desc);
	light_buffer_ = rm.create_buffer(ssbo_desc);
	for (int i = 0; i < 6; i++) {
		texture_arrays_[i] = 0;
		texture_array_sizes_[i] = 0;
	}
	if (config_.use_bvh)
		bvh_ = std::make_unique<BVH>();
	initialized_ = true;
	ARE_LOG_INFO("RayTracer initialized (" + std::to_string(width_) + "x" + std::to_string(height_) + ")");
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
	if (!config_.use_bvh)
		return false;
	if (!bvh_)
		bvh_ = std::make_unique<BVH>();
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
	return true;
}

void RayTracer::trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_image,
	uint sr_scaling, uint sr_jitter, TextureHandle sr_accum) {
	if (!initialized_) {
		ARE_LOG_ERROR("RayTracer not initialized");
		return;
	}
	if (!compute_shader_->is_valid()) {
		ARE_LOG_ERROR("Compute shader not loaded");
		return;
	}
	if (config_.use_bvh && !bvh_built_)
		rebuild_bvh(scene);

	uint sr_enabled = (sr_scaling > 1) ? 1u : 0u;
	uint sr_block = sr_enabled ? static_cast<uint>(std::sqrt(static_cast<double>(sr_scaling))) : 1u;
	uint dispatch_w = sr_enabled ? (width_ / sr_block) : width_;
	uint dispatch_h = sr_enabled ? (height_ / sr_block) : height_;

	// ── texture arrays / scene data ────────────────────────────────────────
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
		for (int slot = 0; slot < 6; slot++) {
			glActiveTexture(GL_TEXTURE10 + slot);
			glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
		}
	}
	upload_scene_data_(scene);

	compute_shader_->use();

	// ── G‑buffer images ────────────────────────────────────────────────────
	bind_gbuffer_(gbuffer);

	// ── output image (binding 3) ──────────────────────────────────────────
	glBindImageTexture(3, output_image, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	// ── accumulation image (binding 4) ────────────────────────────────────
	// SR: caller provides the full‑res accumulation texture
	// non‑SR: use ray‑tracer's own accumulation texture
	TextureHandle accum_tex = sr_enabled ? sr_accum : accumulation_texture_;
	glBindImageTexture(4, accum_tex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);

	// ── BVH ────────────────────────────────────────────────────────────────
	if (config_.use_bvh && bvh_built_) {
		bvh_node_buffer_.bind_base(2);
		bvh_triangle_buffer_.bind_base(3);
		bvh_attr_buffer_.bind_base(4);
		compute_shader_->set_bool("u_use_bvh", true);
		compute_shader_->set_uint("u_bvh_node_count", bvh_->get_node_count());
	} else {
		compute_shader_->set_bool("u_use_bvh", false);
	}

	// ── uniforms ───────────────────────────────────────────────────────────
	compute_shader_->set_uint("u_frame_count", frame_count_);
	compute_shader_->set_uint("u_samples_per_pixel", config_.samples_per_pixel);
	compute_shader_->set_uint("u_max_depth", config_.max_depth);
	compute_shader_->set_uint("u_light_count", static_cast<uint>(scene.get_lights().size()));
	compute_shader_->set_bool("u_enable_accumulation", sr_enabled ? false : config_.enable_accumulation);
	compute_shader_->set_bool("u_enable_textures", has_textures);

	const Camera &camera = scene.get_camera();
	compute_shader_->set_mat4("u_inv_view_projection", glm::inverse(camera.get_view_projection_matrix()));

	compute_shader_->set_uint("u_sr_enabled", sr_enabled);
	compute_shader_->set_uint("u_sr_scaling", sr_scaling);
	compute_shader_->set_uint("u_sr_block", sr_block);
	compute_shader_->set_uint("u_sr_jitter", sr_jitter);
	compute_shader_->set_uint("u_sr_full_width", width_);
	compute_shader_->set_uint("u_sr_full_height", height_);

	// ── dispatch ───────────────────────────────────────────────────────────
	uint num_groups_x = (dispatch_w + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
	uint num_groups_y = (dispatch_h + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;
	glDispatchCompute(num_groups_x, num_groups_y, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	if (config_.enable_accumulation || sr_enabled)
		frame_count_++;
}

void RayTracer::resize(uint width, uint height) {
	if (width == width_ && height == height_)
		return;
	ARE_LOG_DEBUG("RayTracer resize: " + std::to_string(width_) + "x" + std::to_string(height_) + " -> " + std::to_string(width) + "x" + std::to_string(height));
	width_ = width;
	height_ = height;
	if (!initialized_)
		return;
	ResourceManager &rm = ResourceManager::instance();
	if (accumulation_texture_ != INVALID_HANDLE)
		rm.destroy_texture(accumulation_texture_);
	accumulation_texture_ = rm.create_texture(width_, height_, TextureFormat::RGBA32F);
	reset_accumulation();
}

void RayTracer::reset_accumulation() {
	frame_count_ = 0;
}

void RayTracer::set_config(const RayTracerConfig &config) {
	bool bvh_changed = (config.use_bvh != config_.use_bvh);
	config_ = config;
	reset_accumulation();
	if (bvh_changed) {
		if (config_.use_bvh && !bvh_) {
			bvh_ = std::make_unique<BVH>();
			bvh_built_ = false;
		} else if (!config_.use_bvh) {
			bvh_.reset();
			bvh_built_ = false;
		}
	}
}

void RayTracer::upload_scene_data_(const Scene &scene) {
	const auto &materials = scene.get_materials();
	if (!materials.empty()) {
		struct MaterialData {
			alignas(16) Vec3 albedo;
			alignas(16) Vec3 emission;
			float metallic, roughness;
			int type;
			float ior, ao, padding1;
			uint texture_handles[6];
		};
		std::vector<MaterialData> md;
		md.reserve(materials.size());
		for (const auto &mat : materials) {
			MaterialData d {};
			d.albedo = mat->get_albedo();
			d.metallic = mat->get_metallic();
			d.emission = mat->get_emission();
			d.roughness = mat->get_roughness();
			d.type = static_cast<int>(mat->get_type());
			d.ior = mat->get_ior();
			d.ao = 1.0f;
			d.texture_handles[0] = mat->get_texture_index(TextureSlot::ALBEDO);
			d.texture_handles[1] = mat->get_texture_index(TextureSlot::NORMAL);
			d.texture_handles[2] = mat->get_texture_index(TextureSlot::METALLIC);
			d.texture_handles[3] = mat->get_texture_index(TextureSlot::ROUGHNESS);
			d.texture_handles[4] = mat->get_texture_index(TextureSlot::AO);
			d.texture_handles[5] = mat->get_texture_index(TextureSlot::EMISSION);
			md.push_back(d);
		}
		uint h = fnv1a_hash_bytes(md.data(), md.size() * sizeof(MaterialData));
		if (h != materials_hash_) {
			materials_hash_ = h;
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, material_buffer_);
			glBufferData(GL_SHADER_STORAGE_BUFFER, md.size() * sizeof(MaterialData), md.data(), GL_DYNAMIC_DRAW);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);
			reset_accumulation();
		} else {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);
		}
	} else {
		materials_hash_ = 0u;
	}

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
		std::vector<LightData> ld;
		ld.reserve(lights.size());
		for (const auto &l : lights) {
			LightData d {};
			d.position = l->get_position();
			d.type = static_cast<int>(l->get_type());
			d.direction = l->get_direction();
			d.intensity = l->get_intensity();
			d.color = l->get_color();
			d.range = l->get_range();
			d.spot_angles = Vec2(l->get_inner_angle(), l->get_outer_angle());
			ld.push_back(d);
		}
		uint h = fnv1a_hash_bytes(ld.data(), ld.size() * sizeof(LightData));
		if (h != lights_hash_) {
			lights_hash_ = h;
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, light_buffer_);
			glBufferData(GL_SHADER_STORAGE_BUFFER, ld.size() * sizeof(LightData), ld.data(), GL_DYNAMIC_DRAW);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);
			reset_accumulation();
		} else {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);
		}
	} else {
		lights_hash_ = 0u;
	}
}

void RayTracer::bind_gbuffer_(const GBuffer &gbuffer) {
	glBindImageTexture(0, gbuffer.get_texture(GBUFFER_POSITION), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(1, gbuffer.get_texture(GBUFFER_NORMAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RG32F);
	glBindImageTexture(5, gbuffer.get_texture(GBUFFER_MATERIAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(6, gbuffer.get_texture(GBUFFER_MATERIAL_ID), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32UI);
	glBindImageTexture(2, gbuffer.get_texture(GBUFFER_TEXCOORD), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(7, gbuffer.get_texture(GBUFFER_TANGENT), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
}

void RayTracer::build_texture_arrays_(const Scene &scene) {
	const auto &materials = scene.get_materials();
	std::vector<std::shared_ptr<Texture>> textures[6];
	for (const auto &mat : materials) {
		for (int slot = 0; slot < 6; slot++) {
			auto tex = mat->get_texture(static_cast<TextureSlot>(slot));
			if (tex && tex->is_valid()) {
				bool found = false;
				for (const auto &t : textures[slot]) {
					if (t.get() == tex.get()) {
						found = true;
						break;
					}
				}
				if (!found)
					textures[slot].push_back(tex);
			}
		}
	}
	bool any_slot_dirty = false;
	uint new_slot_hashes[6];
	for (int slot = 0; slot < 6; slot++) {
		new_slot_hashes[slot] = compute_slot_texture_hash(textures[slot]);
		if (new_slot_hashes[slot] != texture_slot_hashes_[slot])
			any_slot_dirty = true;
	}
	if (!any_slot_dirty && !texture_arrays_dirty_) {
		for (int slot = 0; slot < 6; slot++)
			if (texture_arrays_[slot] != 0) {
				glActiveTexture(GL_TEXTURE10 + slot);
				glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
			}
		return;
	}
	ResourceManager &rm = ResourceManager::instance();
	for (int slot = 0; slot < 6; slot++) {
		if (new_slot_hashes[slot] == texture_slot_hashes_[slot] && !texture_arrays_dirty_) {
			if (texture_arrays_[slot] != 0) {
				glActiveTexture(GL_TEXTURE10 + slot);
				glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
			}
			continue;
		}
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
		TextureArrayDescription desc;
		desc.textures = textures[slot];
		desc.filter = TextureFilter::LINEAR;
		desc.wrap = TextureWrap::REPEAT;
		texture_arrays_[slot] = rm.create_texture_array(desc);
		for (size_t i = 0; i < textures[slot].size(); i++) {
			uint32_t array_index = static_cast<uint32_t>(i) + 1;
			for (const auto &mat : materials)
				if (mat->get_texture(static_cast<TextureSlot>(slot)).get() == textures[slot][i].get())
					mat->set_texture_index(static_cast<TextureSlot>(slot), array_index);
		}
		texture_slot_hashes_[slot] = new_slot_hashes[slot];
		if (texture_arrays_[slot] != 0) {
			glActiveTexture(GL_TEXTURE10 + slot);
			glBindTexture(GL_TEXTURE_2D_ARRAY, texture_arrays_[slot]);
		}
	}
	texture_arrays_dirty_ = false;
}

} // namespace are
