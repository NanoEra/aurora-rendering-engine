/**
 * @file cpu_raytracer.cpp
 * @brief CPU ray tracing implementation
 */

#include <are/raytracer/cpu_raytracer.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>
#include <are/acceleration/bvh.h>
#include <are/scene/scene_manager.h>
#include <are/scene/camera.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <are/scene/directional_light.h>
#include <are/scene/point_light.h>
#include <are/scene/spot_light.h>
#include <are/rasterizer/gbuffer.h>
#include <are/utils/random.h>
#include <are/utils/math_utils.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

#ifdef ARE_USE_OPENMP
#include <omp.h>
#endif

namespace are {

CPURayTracer::CPURayTracer(const RayTracingConfig& config)
    : RayTracer(config)
    , bvh_(nullptr)
    , scene_(nullptr)
    , width_(0)
    , height_(0)
{
    ARE_LOG_INFO("CPU ray tracer initialized");
}

CPURayTracer::~CPURayTracer() {
    ARE_LOG_INFO("CPU ray tracer destroyed");
}

void CPURayTracer::update_bvh(const BVH& bvh) {
    bvh_ = &bvh;
    ARE_LOG_INFO("BVH updated for CPU ray tracer (" + 
                 std::to_string(bvh.get_nodes().size()) + " nodes)");
}

void CPURayTracer::render(const SceneManager& scene, 
                         const Camera& camera,
                         const GBuffer* gbuffer,
                         uint32_t output_texture) {
    ARE_PROFILE_FUNCTION();
    
    if (!bvh_ || !bvh_->is_built()) {
        ARE_LOG_ERROR("BVH not built, cannot render");
        return;
    }
    
    scene_ = &scene;
    
    // Get framebuffer size from output texture
    glBindTexture(GL_TEXTURE_2D, output_texture);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width_);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height_);
    glBindTexture(GL_TEXTURE_2D, 0);
    
    if (width_ <= 0 || height_ <= 0) {
        ARE_LOG_ERROR("Invalid output texture dimensions");
        return;
    }
    
    // Resize framebuffer if needed
    size_t pixel_count = width_ * height_;
    if (framebuffer_.size() != pixel_count) {
        framebuffer_.resize(pixel_count);
        ARE_LOG_INFO("Framebuffer resized to " + std::to_string(width_) + "x" + std::to_string(height_));
    }
    
    // Render using ray tracing
    ARE_LOG_INFO("Starting CPU ray tracing (" + std::to_string(config_.spp) + " spp)");
    
    const int spp = config_.spp;
    const Real inv_spp = 1.0f / static_cast<Real>(spp);
    
    #ifdef ARE_USE_OPENMP
    #pragma omp parallel for schedule(dynamic, 16)
    #endif
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Vec3 color(0.0f);
            
            // Multi-sampling
            for (int s = 0; s < spp; ++s) {
                // Jittered sampling
                Real u = (x + random_float()) / static_cast<Real>(width_);
                Real v = (y + random_float()) / static_cast<Real>(height_);
                
                // Generate ray
                Vec3 ray_origin, ray_direction;
                camera.generate_ray(u, v, ray_origin, ray_direction);
                
                Ray ray(ray_origin, ray_direction);
                
                // Trace ray
                color += trace_ray(ray, 0);
            }
            
            // Average samples
            color *= inv_spp;
            
            // Store in framebuffer
            size_t index = y * width_ + x;
            framebuffer_[index] = color;
        }
        
        // Progress logging (every 10%)
        if (y % (height_ / 10) == 0) {
            Real progress = 100.0f * y / height_;
            ARE_LOG_INFO("Ray tracing progress: " + std::to_string(static_cast<int>(progress)) + "%");
        }
    }
    
    ARE_LOG_INFO("Ray tracing complete, uploading to GPU");
    
    // Upload to GPU texture
    glBindTexture(GL_TEXTURE_2D, output_texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, 
                    GL_RGB, GL_FLOAT, framebuffer_.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

Vec3 CPURayTracer::trace_ray(const Ray& ray, int depth) {
    // Russian roulette termination
    if (depth >= config_.max_depth) {
        return Vec3(0.0f);
    }
    
    // Intersect with scene
    HitRecord hit;
    if (!bvh_->intersect(ray, hit)) {
        // Sky color (simple gradient)
        Real t = 0.5f * (ray.direction_.y + 1.0f);
        return glm::mix(Vec3(1.0f), Vec3(0.5f, 0.7f, 1.0f), t);
    }
    
    // Shade hit point
    return shade(hit, ray, depth);
}

Vec3 CPURayTracer::shade(const HitRecord& hit, const Ray& ray, int depth) {
	// Random real generator
    thread_local RandomGenerator generator;

    // Get material
    const Material* material = scene_->get_material(hit.material_);
    if (!material) {
        return Vec3(1.0f, 0.0f, 1.0f); // Magenta for missing material
    }
    
    // Get material properties
    Vec3 albedo = material->get_albedo();
    Real metallic = material->get_metallic();
    Real roughness = material->get_roughness();
    Vec3 emissive = material->get_emissive();
    
    // Emissive materials
    if (material->is_emissive()) {
        return emissive;
    }
    
    // Compute direct lighting
    Vec3 direct_lighting = compute_direct_lighting(hit);
    
    // Compute ambient occlusion
    Real ao = 1.0f;
    if (config_.enable_ao) {
        ao = compute_ambient_occlusion(hit);
    }
    
    // Simple diffuse shading
    Vec3 color = albedo * direct_lighting * ao;
    
    // Global illumination (indirect lighting)
    if (config_.enable_gi && depth < config_.max_depth - 1) {
        // Generate random direction in hemisphere
        Vec3 scatter_direction = generator.random_cosine_direction(hit.normal_);
        
        // Trace secondary ray
        Ray scatter_ray(hit.position_ + hit.normal_ * are_epsilon, scatter_direction);
        Vec3 indirect = trace_ray(scatter_ray, depth + 1);
        
        // Add indirect lighting (weighted by albedo)
        color += albedo * indirect * 0.5f;
    }
    
    return color;
}

Vec3 CPURayTracer::compute_direct_lighting(const HitRecord& hit) {
    Vec3 lighting(0.0f);
    
    const auto& lights = scene_->get_all_lights();
    
    for (const auto& light : lights) {
        if (!light) continue;
        
        // Check if light affects this point
        if (!light->affects_point(hit.position_)) {
            continue;
        }
        
        Vec3 light_dir;
        Vec3 light_color = light->get_color() * light->get_intensity();
        Real light_distance = 1e30f;
        
        // Compute light direction based on type
        if (light->get_type() == LightType::ARE_LIGHT_DIRECTIONAL) {
            auto* dir_light = static_cast<const DirectionalLight*>(light.get());
            light_dir = -dir_light->get_direction();
            light_distance = 1e30f; // Infinite distance
        }
        else if (light->get_type() == LightType::ARE_LIGHT_POINT) {
            auto* point_light = static_cast<const PointLight*>(light.get());
            Vec3 to_light = point_light->get_position() - hit.position_;
            light_distance = glm::length(to_light);
            light_dir = to_light / light_distance;
            
            // Apply attenuation
            Real attenuation = point_light->calculate_attenuation(light_distance);
            light_color *= attenuation;
        }
        else if (light->get_type() == LightType::ARE_LIGHT_SPOT) {
            auto* spot_light = static_cast<const SpotLight*>(light.get());
            Vec3 to_light = spot_light->get_position() - hit.position_;
            light_distance = glm::length(to_light);
            light_dir = to_light / light_distance;
            
            // Apply spotlight factor
            Real spot_factor = spot_light->calculate_spot_factor(-light_dir);
            if (spot_factor <= 0.0f) {
                continue; // Outside spotlight cone
            }
            
            light_color *= spot_factor;
        }
        
        // Shadow test
        bool in_shadow = false;
        if (light->get_cast_shadows()) {
            in_shadow = is_in_shadow(hit.position_ + hit.normal_ * are_epsilon, 
                                    light_dir, light_distance);
        }
        
        if (!in_shadow) {
            // Lambertian diffuse
            Real n_dot_l = glm::max(glm::dot(hit.normal_, light_dir), 0.0f);
            lighting += light_color * n_dot_l;
        }
    }
    
    // Add ambient term
    lighting += Vec3(0.03f);
    
    return lighting;
}

Real CPURayTracer::compute_ambient_occlusion(const HitRecord& hit) {
	// Random real generator
    thread_local RandomGenerator generator;

    const int num_samples = config_.ao_samples;
    const Real radius = config_.ao_radius;
    
    int occluded_count = 0;
    
    for (int i = 0; i < num_samples; ++i) {
        // Generate random direction in hemisphere
        Vec3 sample_dir = generator.random_in_hemisphere(hit.normal_);
        
        // Cast AO ray
        Ray ao_ray(hit.position_ + hit.normal_ * are_epsilon, sample_dir, 
                   are_epsilon, radius);
        
        // Check if ray hits anything within radius
        if (bvh_->intersect_any(ao_ray, radius)) {
            occluded_count++;
        }
    }
    
    // AO factor (1.0 = no occlusion, 0.0 = full occlusion)
    Real ao = 1.0f - (static_cast<Real>(occluded_count) / num_samples);
    return ao;
}

bool CPURayTracer::is_in_shadow(const Vec3& origin, const Vec3& direction, Real max_distance) {
    Ray shadow_ray(origin, direction, are_epsilon, max_distance - are_epsilon);
    return bvh_->intersect_any(shadow_ray, max_distance);
}

} // namespace are
