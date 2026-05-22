#include "core/renderer.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <algorithm>
#include <chrono>
#include <glad/glad.h>

namespace are {

Renderer::Renderer(const RendererConfig &config)
	: config_(config)
	, rt_output_texture_(INVALID_HANDLE)
	, initialized_(false) {
}

Renderer::~Renderer() {
	shutdown();
}

bool Renderer::initialize() {
	if (initialized_) {
		ARE_LOG_WARN("Renderer already initialized");
		return true;
	}

	ARE_LOG_INFO("Initializing Aurora Rendering Engine...");

	// The parameter check for super resolution module
	if (config_.sr_config.enabled) {
		// Parameters checking
		if (config_.sr_config.scaling <= 1) {
			ARE_LOG_WARN("Super resolution disabled: scaling must be > 1 (got " + std::to_string(config_.sr_config.scaling) + ")");
			config_.sr_config.enabled = false;
			config_.sr_config.scaling = 1;
		}

		// The super resolution module does not support nouveau driver
		std::string vendor = std::string(reinterpret_cast<const char *>(glGetString(GL_VENDOR)));
		std::transform(vendor.begin(), vendor.end(), vendor.begin(), ::tolower);
		std::string renderer = std::string(reinterpret_cast<const char *>(glGetString(GL_RENDERER)));
		std::transform(renderer.begin(), renderer.end(), renderer.begin(), ::tolower);
		if (vendor.find("mesa") != std::string::npos && renderer.find("nv") != std::string::npos) {
			ARE_LOG_WARN("Super resolution disabled: The super resolution module on nouveau may produce artifacts. Please switch to the NVIDIA proprietary driver if possible.");
			config_.sr_config.enabled = false;
			config_.sr_config.scaling = 1;
		}
	}

	// Initialize shader manager
	shader_manager_ = std::make_unique<ShaderManager>();
	if (!shader_manager_->initialize()) {
		ARE_LOG_ERROR("Failed to initialize shader manager");
		return false;
	}

	// Initialize G-Buffer
	gbuffer_ = std::make_unique<GBuffer>(config_.output_width, config_.output_height);
	if (!gbuffer_->initialize()) {
		ARE_LOG_ERROR("Failed to initialize G-Buffer");
		return false;
	}

	// Initialize ray tracer
	raytracer_ = std::make_unique<RayTracer>(config_.output_width, config_.output_height, config_.rt_config);
	const auto &rt_shader = shader_manager_->get_raytracing_shader();
	if (!raytracer_->initialize(rt_shader)) {
		ARE_LOG_ERROR("Failed to initialize ray tracer");
		return false;
	}

	// Initialize screen blit
	screen_blit_ = std::make_unique<ScreenBlit>();
	const auto &screen_blit_shader = shader_manager_->get_screen_blit_shader();
	if (!screen_blit_->initialize(screen_blit_shader)) {
		ARE_LOG_ERROR("Failed to initialize screen blit");
		return false;
	}

	denoiser_ = std::make_unique<Denoiser>(config_.output_width, config_.output_height);
	const auto &denoise_shader = shader_manager_->get_denoise_shader();
	if (!denoiser_->initialize(denoise_shader)) {
		ARE_LOG_ERROR("Failed to initialize denoiser");
		return false;
	}

	// Initialize super resolution if enabled
	if (config_.sr_config.enabled) {
		super_resolution_ = std::make_unique<SuperResolution>(config_.output_width, config_.output_height, config_.sr_config);
		const auto &sr_shader = shader_manager_->get_super_resolution_shader();
		if (!super_resolution_->initialize(sr_shader)) {
			ARE_LOG_ERROR("Failed to initialize super resolution");
			return false;
		}
	}

	// Create ray tracing output texture (reused every frame)
	ResourceManager &rm = ResourceManager::instance();
	rt_output_texture_ = rm.create_texture(config_.output_width, config_.output_height, TextureFormat::RGBA32F);

	initialized_ = true;
	ARE_LOG_INFO("Aurora Rendering Engine initialized successfully");
	return true;
}

void Renderer::shutdown() {
	if (!initialized_)
		return;

	ARE_LOG_INFO("Shutting down Aurora Rendering Engine...");

	ResourceManager &rm = ResourceManager::instance();

	if (rt_output_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(rt_output_texture_);
		rt_output_texture_ = INVALID_HANDLE;
	}

	screen_blit_.reset();
	raytracer_.reset();
	gbuffer_.reset();
	shader_manager_.reset();
	denoiser_.reset();
	super_resolution_.reset();

	initialized_ = false;
	ARE_LOG_INFO("Aurora Rendering Engine shut down");
}

RenderStats Renderer::render(const Scene &scene, TextureHandle output_texture) {
	RenderStats stats = {};

	if (!initialized_) {
		ARE_LOG_ERROR("Renderer not initialized");
		return stats;
	}

	// Start timing
	auto start_time = std::chrono::high_resolution_clock::now();

	// Phase 1: G-Buffer pass
	auto gbuffer_start = std::chrono::high_resolution_clock::now();

	const auto &gbuffer_shader = shader_manager_->get_gbuffer_shader();
	if (!gbuffer_shader || !gbuffer_shader->is_valid()) {
		ARE_LOG_ERROR("G-Buffer shader is invalid");
		return stats;
	}
	gbuffer_->render(scene, *gbuffer_shader);

	auto gbuffer_end = std::chrono::high_resolution_clock::now();
	stats.gbuffer_time_ms_ = std::chrono::duration<float, std::milli>(gbuffer_end - gbuffer_start).count();

	// Phase 2: Ray tracing pass
	auto raytrace_start = std::chrono::high_resolution_clock::now();

	TextureHandle rt_output;
	if (config_.sr_config.enabled && super_resolution_) {
		auto &sr = *super_resolution_;
		uint jitt = sr.get_current_jitter_frame();
		rt_output = sr.get_low_res_rt_texture();

		raytracer_->trace(scene, *gbuffer_, rt_output,
			config_.sr_config.scaling, jitt,
			sr.get_accumulated_rt_texture());
	} else {
		rt_output = (output_texture != 0) ? output_texture : rt_output_texture_;
		raytracer_->trace(scene, *gbuffer_, rt_output);
	}

	auto raytrace_end = std::chrono::high_resolution_clock::now();
	stats.raytrace_time_ms_ = std::chrono::duration<float, std::milli>(raytrace_end - raytrace_start).count();

	// Phase 3: Post-processing and output
	TextureHandle final_output;
	if (config_.sr_config.enabled && super_resolution_) {
		// Denoising intentionally skipped — cross‑cycle accumulation provides temporal smoothing
		auto &sr = *super_resolution_;
		final_output = sr.upscale();
		sr.advance_jitter_frame();
	} else {
		final_output = rt_output;
		if (config_.enable_denoising && denoiser_) {
			float temporal_weight = 0.1f;
			final_output = denoiser_->denoise(final_output, 1, temporal_weight);
		}
	}

	// Phase 4: Blit to screen if output is default framebuffer
	if (output_texture == 0) {
		screen_blit_->blit_fullscreen(final_output);
	}

	// Calculate total frame time
	auto end_time = std::chrono::high_resolution_clock::now();
	stats.frame_time_ms_ = std::chrono::duration<float, std::milli>(end_time - start_time).count();

	// Count triangles
	const auto &meshes = scene.get_meshes();
	for (const auto &mesh : meshes) {
		stats.triangle_count_ += mesh->get_indices().size() / 3;
	}

	return stats;
}

void Renderer::resize(uint width, uint height) {
	if (width == config_.output_width && height == config_.output_height)
		return;

	config_.output_width = width;
	config_.output_height = height;

	if (initialized_) {
		ResourceManager &rm = ResourceManager::instance();

		// Recreate ray tracing output texture
		if (rt_output_texture_ != INVALID_HANDLE) {
			rm.destroy_texture(rt_output_texture_);
		}
		rt_output_texture_ = rm.create_texture(width, height, TextureFormat::RGBA32F);

		gbuffer_->resize(width, height);
		raytracer_->resize(width, height);
		denoiser_->resize(width, height);

		if (super_resolution_) {
			super_resolution_->resize(width, height);
		}

		ARE_LOG_INFO("Renderer resized to " + std::to_string(width) + "x" + std::to_string(height));
	}
}

void Renderer::set_config(const RendererConfig &config) {
	bool size_changed = (config.output_width != config_.output_width || config.output_height != config_.output_height);

	config_ = config;

	if (initialized_) {
		if (size_changed) {
			resize(config_.output_width, config_.output_height);
		}

		// Update ray tracer config
		raytracer_->set_config(config_.rt_config);

		// Handle SR enable/disable
		if (config_.sr_config.enabled && !super_resolution_) {
			super_resolution_ = std::make_unique<SuperResolution>(config_.output_width, config_.output_height, config_.sr_config);
			const auto &sr_shader = shader_manager_->get_super_resolution_shader();
			super_resolution_->initialize(sr_shader);
		} else if (!config_.sr_config.enabled && super_resolution_) {
			super_resolution_.reset();
		}
	}
}

void Renderer::notify_scene_changed(const Scene &scene) {
	raytracer_->reset_accumulation();
	raytracer_->rebuild_bvh(scene);

	// Reset denoiser temporal history on scene change
	if (denoiser_) {
		denoiser_->reset_history();
	}

	// Reset super resolution accumulation on scene change
	if (super_resolution_) {
		super_resolution_->reset_accumulation();
	}
}

} // namespace are
