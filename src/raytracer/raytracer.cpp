/**
 * @file raytracer.cpp
 * @brief Implementation of RayTracer base class
 */

#include <are/raytracer/raytracer.h>
#include <are/core/logger.h>

namespace are {

RayTracer::RayTracer(const RayTracingConfig& config)
    : config_(config) {
    ARE_LOG_INFO("RayTracer: Created with max depth " + 
                 std::to_string(config_.max_depth));
}

void RayTracer::set_config(const RayTracingConfig& config) {
    config_ = config;
    ARE_LOG_INFO("RayTracer: Configuration updated");
}

} // namespace are
