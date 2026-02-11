#include "core/raytracer.h"
#include "basic/constants.h"
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
} // namespace

RayTracer::RayTracer(uint width, uint height, const RayTracerConfig &config)
	: width_(width)
	, height_(height)
	, config_(config)
	, accumulation_texture_(INVALID_HANDLE)
	, scene_buffer_(INVALID_HANDLE)
	, material_buffer_(INVALID_HANDLE)
	, light_buffer_(INVALID_HANDLE)
	, bvh_(nullptr)
	, bvh_built_(false)
	, materials_hash_(0u)
	, lights_hash_(0u)
	, frame_count_(0)
	, initialized_(false) {
}

RayTracer::~RayTracer() {
	release();
}

bool RayTracer::initialize() {
	if (initialized_) {
		Logger::warning("RayTracer already initialized");
		return true;
	}

	// Create accumulation texture
	glGenTextures(1, &accumulation_texture_);
	glBindTexture(GL_TEXTURE_2D, accumulation_texture_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// Create shader storage buffers
	glGenBuffers(1, &material_buffer_);
	glGenBuffers(1, &light_buffer_);

	// Initialize BVH if enabled
	if (config_.use_bvh_) {
		bvh_ = std::make_unique<BVH>();
	}

	initialized_ = true;
	Logger::info("RayTracer initialized successfully");
	return true;
}

void RayTracer::release() {
	if (!initialized_)
		return;

	if (accumulation_texture_ != INVALID_HANDLE) {
		glDeleteTextures(1, &accumulation_texture_);
		accumulation_texture_ = INVALID_HANDLE;
	}

	if (material_buffer_ != INVALID_HANDLE) {
		glDeleteBuffers(1, &material_buffer_);
		material_buffer_ = INVALID_HANDLE;
	}

	if (light_buffer_ != INVALID_HANDLE) {
		glDeleteBuffers(1, &light_buffer_);
		light_buffer_ = INVALID_HANDLE;
	}

	bvh_node_buffer_.release();
	bvh_triangle_buffer_.release();

	bvh_.reset();
	bvh_built_ = false;

	initialized_ = false;
	Logger::info("RayTracer released");
}

bool RayTracer::rebuild_bvh(const Scene &scene) {
	if (!config_.use_bvh_) {
		Logger::warning("BVH is disabled in configuration");
		return false;
	}

	if (!bvh_) {
		bvh_ = std::make_unique<BVH>();
	}

	Logger::info("Building BVH for ray tracing...");

	if (!bvh_->build(scene.get_meshes())) {
		Logger::error("Failed to build BVH");
		return false;
	}

	if (!bvh_->upload_to_gpu(bvh_node_buffer_, bvh_triangle_buffer_)) {
		Logger::error("Failed to upload BVH to GPU");
		return false;
	}

	bvh_built_ = true;
	reset_accumulation();
	Logger::info("BVH built and uploaded successfully");
	return true;
}

void RayTracer::trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_texture) {
	if (!initialized_) {
		Logger::error("RayTracer not initialized");
		return;
	}

	if (!compute_shader_->is_valid()) {
		Logger::error("Ray tracing compute shader not loaded");
		return;
	}

	// Build BVH if enabled and not built yet
	if (config_.use_bvh_ && !bvh_built_) {
		rebuild_bvh(scene);
	}

	// Upload scene data
	upload_scene_data_(scene);

	// Use compute shader
	if (!compute_shader_ || !compute_shader_->is_valid()) {
		Logger::error("Ray tracing compute shader not set or invalid");
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

	// Set camera data
	const Camera &camera = scene.get_camera();
	compute_shader_->set_vec3("u_camera_position", camera.get_position());

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
		// Recreate accumulation texture
		if (accumulation_texture_ != INVALID_HANDLE) {
			glDeleteTextures(1, &accumulation_texture_);
		}

		glGenTextures(1, &accumulation_texture_);
		glBindTexture(GL_TEXTURE_2D, accumulation_texture_);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

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
		struct MaterialData {
			Vec3 albedo;
			float metallic;
			Vec3 emission;
			float roughness;
			int type;
			float ior;
			Vec2 padding;
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
	glBindImageTexture(1, gbuffer.get_texture(GBUFFER_NORMAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(2, gbuffer.get_texture(GBUFFER_ALBEDO), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA8);

	glBindImageTexture(5, gbuffer.get_texture(GBUFFER_MATERIAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(6, gbuffer.get_texture(GBUFFER_MATERIAL_ID), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32UI);
}

void RayTracer::set_compute_shader(const std::shared_ptr<Shader> &shader) {
	compute_shader_ = shader;
	Logger::info("Compute shader set for RayTracer");
}

} // namespace are
