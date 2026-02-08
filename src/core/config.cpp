/**
 * @file config.cpp
 * @brief Implementation of configuration system
 */

#include <are/core/config.h>
#include <are/core/logger.h>
#include <thread>

namespace are {

bool AreConfig::validate() const {
    bool valid = true;

    // Validate window config
    if (window.width <= 0 || window.height <= 0) {
        ARE_LOG_ERROR("Invalid window dimensions: " + 
                     std::to_string(window.width) + "x" + 
                     std::to_string(window.height));
        valid = false;
    }

    if (window.samples < 1) {
        ARE_LOG_ERROR("Invalid MSAA samples: " + std::to_string(window.samples));
        valid = false;
    }

    // Validate ray tracing config
    if (ray_tracing.spp <= 0) {
        ARE_LOG_ERROR("Invalid SPP value: " + std::to_string(ray_tracing.spp));
        valid = false;
    }

    if (ray_tracing.max_depth <= 0) {
        ARE_LOG_ERROR("Invalid max ray depth: " + std::to_string(ray_tracing.max_depth));
        valid = false;
    }

    if (ray_tracing.ao_samples < 0) {
        ARE_LOG_ERROR("Invalid AO samples: " + std::to_string(ray_tracing.ao_samples));
        valid = false;
    }

    if (ray_tracing.ao_radius <= 0.0f) {
        ARE_LOG_ERROR("Invalid AO radius: " + std::to_string(ray_tracing.ao_radius));
        valid = false;
    }

    // Validate render config
    if (render.exposure <= 0.0f) {
        ARE_LOG_ERROR("Invalid exposure: " + std::to_string(render.exposure));
        valid = false;
    }

    // Validate performance config
    if (performance.num_threads < 0) {
        ARE_LOG_ERROR("Invalid thread count: " + std::to_string(performance.num_threads));
        valid = false;
    }

    return valid;
}

void AreConfig::print() const {
    ARE_LOG_INFO("=== Aurora Rendering Engine Configuration ===");
    
    // Window configuration
    ARE_LOG_INFO("Window:");
    ARE_LOG_INFO("  Size: " + std::to_string(window.width) + "x" + 
                 std::to_string(window.height));
    ARE_LOG_INFO("  Title: " + window.title);
    ARE_LOG_INFO("  Resizable: " + std::string(window.resizable ? "yes" : "no"));
    ARE_LOG_INFO("  VSync: " + std::string(window.vsync ? "enabled" : "disabled"));
    ARE_LOG_INFO("  MSAA: " + std::to_string(window.samples) + "x");

    // Ray tracing configuration
    ARE_LOG_INFO("Ray Tracing:");
    std::string backend_str = (ray_tracing.backend == RayTracingBackend::ARE_RT_BACKEND_CPU) 
                             ? "CPU" : "Compute Shader";
    ARE_LOG_INFO("  Backend: " + backend_str);
    ARE_LOG_INFO("  SPP: " + std::to_string(ray_tracing.spp));
    ARE_LOG_INFO("  Max Depth: " + std::to_string(ray_tracing.max_depth));
    ARE_LOG_INFO("  Global Illumination: " + std::string(ray_tracing.enable_gi ? "enabled" : "disabled"));
    ARE_LOG_INFO("  Ambient Occlusion: " + std::string(ray_tracing.enable_ao ? "enabled" : "disabled"));
    ARE_LOG_INFO("  Soft Shadows: " + std::string(ray_tracing.enable_soft_shadows ? "enabled" : "disabled"));
    
    if (ray_tracing.enable_ao) {
        ARE_LOG_INFO("  AO Samples: " + std::to_string(ray_tracing.ao_samples));
        ARE_LOG_INFO("  AO Radius: " + std::to_string(ray_tracing.ao_radius));
    }

    // Render configuration
    ARE_LOG_INFO("Rendering:");
    std::string tonemap_str;
    switch (render.tonemap_op) {
        case ToneMappingOperator::ARE_TONEMAP_NONE:     tonemap_str = "None"; break;
        case ToneMappingOperator::ARE_TONEMAP_REINHARD: tonemap_str = "Reinhard"; break;
        case ToneMappingOperator::ARE_TONEMAP_ACES:     tonemap_str = "ACES"; break;
    }
    ARE_LOG_INFO("  Tone Mapping: " + tonemap_str);
    ARE_LOG_INFO("  Exposure: " + std::to_string(render.exposure));
    ARE_LOG_INFO("  HDR: " + std::string(render.use_hdr ? "enabled" : "disabled"));

    // Performance configuration
    ARE_LOG_INFO("Performance:");
    int num_threads = performance.num_threads == 0 ? 
        static_cast<int>(std::thread::hardware_concurrency()) : performance.num_threads;
    ARE_LOG_INFO("  Threads: " + std::to_string(num_threads));
    ARE_LOG_INFO("  BVH Multithreading: " + 
                 std::string(performance.enable_bvh_multithreading ? "enabled" : "disabled"));
    ARE_LOG_INFO("  Profiling: " + 
                 std::string(performance.enable_profiling ? "enabled" : "disabled"));

    // Path configuration
    ARE_LOG_INFO("Paths:");
    ARE_LOG_INFO("  Shaders: " + paths.shader_dir);
    ARE_LOG_INFO("  Textures: " + paths.texture_dir);
    ARE_LOG_INFO("  Output: " + paths.output_dir);
    
    ARE_LOG_INFO("=============================================");
}

} // namespace are
