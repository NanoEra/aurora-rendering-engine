/**
 * @file raytracer.cpp
 * @brief Implementation of RayTracer interface
 */

#include <are/raytracer/raytracer.h>

namespace are {

RayTracer::RayTracer(const RayTracingConfig& config)
    : config_(config) {
}

void RayTracer::set_config(const RayTracingConfig& config) {
    config_ = config;
}

} // namespace are
