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
    
    gbuffer_shader_.release();
    raytracing_shader_.release();
    
    for (auto& pair : shader_cache_) {
        pair.second.release();
    }
    shader_cache_.clear();
    
    initialized_ = false;
    Logger::info("ShaderManager released");
}

Shader ShaderManager::load_shader(const std::string& name,
                                  const std::string& vertex_path,
                                  const std::string& fragment_path) {
    // Check cache
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Shader '" + name + "' loaded from cache");
        return it->second;
    }
    
    // Load shader
    Shader shader;
    if (!shader.load(vertex_path, fragment_path)) {
        Logger::error("Failed to load shader '" + name + "'");
        return Shader();
    }
    
    shader_cache_[name] = shader;
    Logger::info("Shader '" + name + "' loaded successfully");
    return shader;
}

Shader ShaderManager::load_compute_shader(const std::string& name,
                                         const std::string& compute_path) {
    // Check cache
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Compute shader '" + name + "' loaded from cache");
        return it->second;
    }
    
    // Load shader
    Shader shader;
    if (!shader.load_compute(compute_path)) {
        Logger::error("Failed to load compute shader '" + name + "'");
        return Shader();
    }
    
    shader_cache_[name] = shader;
    Logger::info("Compute shader '" + name + "' loaded successfully");
    return shader;
}

Shader ShaderManager::get_shader(const std::string& name) const {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        return it->second;
    }
    
    Logger::warning("Shader '" + name + "' not found in cache");
    return Shader();
}

bool ShaderManager::load_builtin_shaders_() {
    // Load G-Buffer shader
    if (!gbuffer_shader_.load("shaders/gbuffer.vert", "shaders/gbuffer.frag")) {
        Logger::error("Failed to load G-Buffer shader");
        return false;
    }
    shader_cache_["gbuffer"] = gbuffer_shader_;

    // Load ray tracing compute shader
    Logger::info("Loading ray tracing compute shader...");
    if (!raytracing_shader_.load_compute("shaders/raytracing.comp")) {
        Logger::error("Failed to load ray tracing shader");
        return false;
    }
    shader_cache_["raytracing"] = raytracing_shader_;
    Logger::info("Ray tracing shader loaded successfully");

    return true;
}

} // namespace are
