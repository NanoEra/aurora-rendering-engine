#include "core/shader_manager.h"
#include "utils/logger.h"

namespace are {

ShaderManager::ShaderManager()
    : initialized_(false) {
}

ShaderManager::~ShaderManager() {
    release();
}

bool ShaderManager::initialize() {
    if (initialized_) {
        ARE_LOG_WARN("ShaderManager already initialized");
        return true;
    }
    
    ARE_LOG_INFO("Loading built-in shaders...");
    
    if (!load_builtin_shaders_()) {
        ARE_LOG_ERROR("Failed to load built-in shaders");
        return false;
    }
    
    initialized_ = true;
    ARE_LOG_INFO("ShaderManager initialized successfully");
    return true;
}

void ShaderManager::release() {
    if (!initialized_) return;

    for (auto& pair : shader_cache_) {
        if (pair.second) pair.second->release();
    }
    shader_cache_.clear();

    gbuffer_shader_.reset();
	screen_blit_shader_.reset();
    raytracing_shader_.reset();
	denoise_shader_.reset();

    initialized_ = false;
    ARE_LOG_INFO("ShaderManager released");
}

std::shared_ptr<Shader> ShaderManager::load_shader(const std::string& name,
                                                   const std::string& vertex_path,
                                                   const std::string& fragment_path) {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        ARE_LOG_INFO("Shader '" + name + "' loaded from cache");
        return it->second;
    }

    auto shader = std::make_shared<Shader>();
    if (!shader->load(vertex_path, fragment_path)) {
        ARE_LOG_ERROR("Failed to load shader '" + name + "'");
        return nullptr;
    }

    shader_cache_[name] = shader;
    ARE_LOG_INFO("Shader '" + name + "' loaded successfully");
    return shader;
}

std::shared_ptr<Shader> ShaderManager::load_compute_shader(const std::string& name,
                                                           const std::string& compute_path) {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        ARE_LOG_INFO("Compute shader '" + name + "' loaded from cache");
        return it->second;
    }

    auto shader = std::make_shared<Shader>();
    if (!shader->load_compute(compute_path)) {
        ARE_LOG_ERROR("Failed to load compute shader '" + name + "'");
        return nullptr;
    }

    shader_cache_[name] = shader;
    ARE_LOG_INFO("Compute shader '" + name + "' loaded successfully");
    return shader;
}

std::shared_ptr<Shader> ShaderManager::get_shader(const std::string& name) const {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) return it->second;

    ARE_LOG_WARN("Shader '" + name + "' not found in cache");
    return nullptr;
}

bool ShaderManager::load_builtin_shaders_() {
	// Load G-buffer shader
	ARE_LOG_INFO("Loading G-buffer shaders..");
    gbuffer_shader_ = std::make_shared<Shader>();
    if (!gbuffer_shader_->load("shaders/gbuffer.vert", "shaders/gbuffer.frag")) {
        ARE_LOG_ERROR("Failed to load G-Buffer shader");
        return false;
    }
    shader_cache_["gbuffer"] = gbuffer_shader_;

	// Load screen bliting shader
	ARE_LOG_INFO("Loading screen blit shaders...");
    screen_blit_shader_ = std::make_shared<Shader>();
    if (!screen_blit_shader_->load("shaders/screen_blit.vert", "shaders/screen_blit.frag")) {
        ARE_LOG_ERROR("Failed to load screen blit shader");
        return false;
    }
    shader_cache_["screen_blit"] = denoise_shader_;
    ARE_LOG_INFO("Screen blit shader loaded successfully");

	// Load ray tracing shader
    ARE_LOG_INFO("Loading ray tracing compute shader...");
    raytracing_shader_ = std::make_shared<Shader>();
    if (!raytracing_shader_->load_compute("shaders/raytracing.comp")) {
        ARE_LOG_ERROR("Failed to load ray tracing shader");
        return false;
    }
    shader_cache_["raytracing"] = raytracing_shader_;
    ARE_LOG_INFO("Ray tracing shader loaded successfully");

	// Load denoising shader
	ARE_LOG_INFO("Loading denoise compute shader...");
    denoise_shader_ = std::make_shared<Shader>();
    if (!denoise_shader_->load_compute("shaders/denoiser.comp")) {
        ARE_LOG_ERROR("Failed to load denoise shader");
        return false;
    }
    shader_cache_["denoise"] = denoise_shader_;
    ARE_LOG_INFO("Denoise shader loaded successfully");

    return true;
}

} // namespace are
