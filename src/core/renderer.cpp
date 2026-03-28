#include "core/renderer.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <chrono>
#include <glad/glad.h>

namespace are {

Renderer::Renderer(const RendererConfig &config)
	: config_(config)
	, rt_output_texture_(INVALID_HANDLE)
	, initialized_(false)
	, frame_count_(0) {
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
    
    // Initialize shader manager
    shader_manager_ = std::make_unique<ShaderManager>();
    if (!shader_manager_->initialize()) {
        ARE_LOG_ERROR("Failed to initialize shader manager");
        return false;
    }
    
    // Initialize G-Buffer
    gbuffer_ = std::make_unique<GBuffer>(config_.width_, config_.height_);
    if (!gbuffer_->initialize()) {
        ARE_LOG_ERROR("Failed to initialize G-Buffer");
        return false;
    }
    
    // Initialize ray tracer
    RayTracerConfig rt_config;
    rt_config.samples_per_pixel_ = config_.samples_per_pixel_;
    rt_config.max_depth_ = config_.max_ray_depth_;
    rt_config.enable_shadows_ = true;
    rt_config.enable_reflections_ = true;
    rt_config.enable_accumulation_ = config_.enable_accumulation_;
    rt_config.use_bvh_ = true;
    
	// Initialize ray tracer
    raytracer_ = std::make_unique<RayTracer>(config_.width_, config_.height_, rt_config);
	const auto& rt_shader = shader_manager_->get_raytracing_shader();
    if (!raytracer_->initialize(rt_shader)) {
        ARE_LOG_ERROR("Failed to initialize ray tracer");
        return false;
    }

    // Initialize screen blit
    screen_blit_ = std::make_unique<ScreenBlit>();
	const auto& screen_blit_shader = shader_manager_->get_screen_blit_shader();
    if (!screen_blit_->initialize(screen_blit_shader)) {
        ARE_LOG_ERROR("Failed to initialize screen blit");
        return false;
    }

	denoiser_ = std::make_unique<Denoiser>(config_.width_, config_.height_);
	const auto& denoise_shader = shader_manager_->get_denoise_shader();
	if (!denoiser_->initialize(denoise_shader)) {
		ARE_LOG_ERROR("Failed to initialize denoiser");
		return false;
	}

	// Create ray tracing output texture (reused every frame)
	ResourceManager &rm = ResourceManager::instance();
	rt_output_texture_ = rm.create_texture(config_.width_, config_.height_, TextureFormat::RGBA32F);

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

	initialized_ = false;
	ARE_LOG_INFO("Aurora Rendering Engine shut down");
}

RenderStats Renderer::render(const Scene& scene, TextureHandle output_texture) {
    RenderStats stats = {};
    
    if (!initialized_) {
        ARE_LOG_ERROR("Renderer not initialized");
        return stats;
    }
    
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Phase 1: G-Buffer pass
    auto gbuffer_start = std::chrono::high_resolution_clock::now();
    
	const auto& gbuffer_shader = shader_manager_->get_gbuffer_shader();
	if (!gbuffer_shader || !gbuffer_shader->is_valid()) {
		ARE_LOG_ERROR("G-Buffer shader is invalid");
		return stats;
	}
	gbuffer_->render(scene, *gbuffer_shader);
    
    auto gbuffer_end = std::chrono::high_resolution_clock::now();
    stats.gbuffer_time_ms_ = std::chrono::duration<float, std::milli>(gbuffer_end - gbuffer_start).count();
    
    // Phase 2: Ray tracing pass
    auto raytrace_start = std::chrono::high_resolution_clock::now();
    
    // Use output texture if provided, otherwise use internal texture
    TextureHandle rt_output = (output_texture != 0) ? output_texture : rt_output_texture_;
    
    raytracer_->trace(scene, *gbuffer_, rt_output);
    
    auto raytrace_end = std::chrono::high_resolution_clock::now();
    stats.raytrace_time_ms_ = std::chrono::duration<float, std::milli>(raytrace_end - raytrace_start).count();
    
	// Phase 3: Denoise texture
	TextureHandle final_output = rt_output;

	if (config_.enable_denoising_ && denoiser_) {
		final_output = denoiser_->denoise(rt_output, 1); // radius=1 => 3x3 mean
	}

    // Phase 4: Blit to screen if output is default framebuffer
	if (output_texture == 0) {
		screen_blit_->blit_fullscreen(final_output);
	}
    
    // Calculate total frame time
    auto end_time = std::chrono::high_resolution_clock::now();
    stats.frame_time_ms_ = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    
    // Count triangles
    const auto& meshes = scene.get_meshes();
    for (const auto& mesh : meshes) {
        stats.triangle_count_ += mesh->get_indices().size() / 3;
    }
    
    // Estimate ray count (very rough)
    stats.ray_count_ = config_.width_ * config_.height_ * config_.samples_per_pixel_ * config_.max_ray_depth_;
    
    frame_count_++;
    
    return stats;
}

void Renderer::resize(uint width, uint height) {
	if (width == config_.width_ && height == config_.height_)
		return;

	config_.width_ = width;
	config_.height_ = height;

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

		ARE_LOG_INFO("Renderer resized to " + std::to_string(width) + "x" + std::to_string(height));
	}
}

void Renderer::set_config(const RendererConfig &config) {
	bool size_changed = (config.width_ != config_.width_ || config.height_ != config_.height_);

	config_ = config;

	if (initialized_) {
		if (size_changed) {
			resize(config_.width_, config_.height_);
		}

		// Update ray tracer config
		RayTracerConfig rt_config = raytracer_->get_config();
		rt_config.samples_per_pixel_ = config_.samples_per_pixel_;
		rt_config.max_depth_ = config_.max_ray_depth_;
		rt_config.enable_accumulation_ = config_.enable_accumulation_;
		raytracer_->set_config(rt_config);
	}
}

void Renderer::notify_scene_changed(const Scene &scene) {
	raytracer_->reset_accumulation();
	raytracer_->rebuild_bvh(scene);
}

} // namespace are
