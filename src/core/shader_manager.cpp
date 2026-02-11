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
        Logger::warning("ShaderManager already initialized");
        return true;
    }
    
    Logger::info("Loading built-in shaders...");
    
    if (!load_builtin_shaders_()) {
        Logger::error("Failed to load built-in shaders");
        return false;
    }
    
    initialized_ = true;
    Logger::info("ShaderManager initialized successfully");
    return true;
}

void ShaderManager::release() {
    if (!initialized_) return;

    for (auto& pair : shader_cache_) {
        if (pair.second) pair.second->release();
    }
    shader_cache_.clear();

    gbuffer_shader_.reset();
    raytracing_shader_.reset();
	denoise_shader_.reset();

    initialized_ = false;
    Logger::info("ShaderManager released");
}

std::shared_ptr<Shader> ShaderManager::load_shader(const std::string& name,
                                                   const std::string& vertex_path,
                                                   const std::string& fragment_path) {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Shader '" + name + "' loaded from cache");
        return it->second;
    }

    auto shader = std::make_shared<Shader>();
    if (!shader->load(vertex_path, fragment_path)) {
        Logger::error("Failed to load shader '" + name + "'");
        return nullptr;
    }

    shader_cache_[name] = shader;
    Logger::info("Shader '" + name + "' loaded successfully");
    return shader;
}

std::shared_ptr<Shader> ShaderManager::load_compute_shader(const std::string& name,
                                                           const std::string& compute_path) {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Compute shader '" + name + "' loaded from cache");
        return it->second;
    }

    auto shader = std::make_shared<Shader>();
    if (!shader->load_compute(compute_path)) {
        Logger::error("Failed to load compute shader '" + name + "'");
        return nullptr;
    }

    shader_cache_[name] = shader;
    Logger::info("Compute shader '" + name + "' loaded successfully");
    return shader;
}

std::shared_ptr<Shader> ShaderManager::get_shader(const std::string& name) const {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) return it->second;

    Logger::warning("Shader '" + name + "' not found in cache");
    return nullptr;
}

bool ShaderManager::load_builtin_shaders_() {
    gbuffer_shader_ = std::make_shared<Shader>();
    if (!gbuffer_shader_->load("shaders/gbuffer.vert", "shaders/gbuffer.frag")) {
        Logger::error("Failed to load G-Buffer shader");
        return false;
    }
    shader_cache_["gbuffer"] = gbuffer_shader_;

    Logger::info("Loading ray tracing compute shader...");
    raytracing_shader_ = std::make_shared<Shader>();
    if (!raytracing_shader_->load_compute("shaders/raytracing.comp")) {
        Logger::error("Failed to load ray tracing shader");
        return false;
    }
    shader_cache_["raytracing"] = raytracing_shader_;
    Logger::info("Ray tracing shader loaded successfully");

	Logger::info("Loading denoise compute shader...");
    denoise_shader_ = std::make_shared<Shader>();
    if (!denoise_shader_->load_compute("shaders/denoiser.comp")) {
        Logger::error("Failed to load denoise shader");
        return false;
    }
    shader_cache_["denoise"] = denoise_shader_;
    Logger::info("Denoise shader loaded successfully");

    return true;
}

} // namespace are
