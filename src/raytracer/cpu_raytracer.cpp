/**
 * @file cpu_raytracer.cpp
 * @brief Implementation of CPURayTracer
 */

#include <are/raytracer/cpu_raytracer.h>

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
#include <limits>
#include <stdexcept>

// #ifdef ARE_USE_OPENMP
// #include <omp.h>
// #endif

namespace are {

namespace {

/**
 * @brief Apply simple Reinhard tonemapping.
 * @param c HDR color
 * @param exposure Exposure value
 * @return LDR color in [0,1]
 */
inline Vec3 tonemap_reinhard(const Vec3& c, Real exposure) {
    Vec3 x = c * exposure;
    return x / (Vec3(1.0f) + x);
}

/**
 * @brief Offset ray origin to reduce self-intersection.
 * @param p Hit position
 * @param n Shading normal
 * @return Offset position
 */
inline Vec3 offset_ray_origin(const Vec3& p, const Vec3& n) {
    return p + n * (are_epsilon * 10.0f);
}

} // namespace

CPURayTracer::CPURayTracer(const RayTracingConfig& config)
    : RayTracer(config)
    , bvh_(nullptr)
    , scene_(nullptr)
    , framebuffer_()
    , width_(0)
    , height_(0) {
}

CPURayTracer::~CPURayTracer() = default;

void CPURayTracer::update_bvh(const BVH& bvh) {
    bvh_ = &bvh;
}

void CPURayTracer::render(const SceneManager& scene,
                          const Camera& camera,
                          const GBuffer* gbuffer,
                          uint32_t output_texture) {
    ARE_PROFILE_FUNCTION();

    if (!bvh_ || !bvh_->is_built()) {
        ARE_LOG_ERROR("CPURayTracer: BVH is null or not built");
        return;
    }

    if (!gbuffer) {
        ARE_LOG_CRITICAL("CPURayTracer: GBuffer is null, cannot infer render resolution");
        throw std::runtime_error("CPURayTracer requires a valid GBuffer for resolution");
    }

    if (output_texture == 0) {
        ARE_LOG_ERROR("CPURayTracer: output_texture is 0");
        return;
    }

    scene_ = &scene;
    width_ = gbuffer->get_width();
    height_ = gbuffer->get_height();

    if (width_ <= 0 || height_ <= 0) {
        ARE_LOG_ERROR("CPURayTracer: Invalid render resolution");
        return;
    }

    const int spp = std::max(1, config_.spp);
    const int max_depth = std::max(1, config_.max_depth);

    framebuffer_.assign(static_cast<size_t>(width_ * height_), Vec4(0.0f));

// #ifdef ARE_USE_OPENMP
//     #pragma omp parallel for schedule(dynamic, 1)
// #endif
    for (int y = 0; y < height_; ++y) {
        RandomGenerator& rng = get_thread_random();

        for (int x = 0; x < width_; ++x) {
            Vec3 hdr(0.0f);

            for (int s = 0; s < spp; ++s) {
                Real u = (static_cast<Real>(x) + rng.random_float()) / static_cast<Real>(width_);
                Real v = (static_cast<Real>(y) + rng.random_float()) / static_cast<Real>(height_);

                Vec3 origin;
                Vec3 direction;
                camera.generate_ray(u, v, origin, direction);

                Ray ray(origin, direction, are_epsilon, 1e30f);
                hdr += trace_ray(ray, max_depth);
            }

            hdr /= static_cast<Real>(spp);

            // Phase 5: tonemap in tracer for standalone output
            Vec3 ldr = tonemap_reinhard(hdr, 1.0f);
            framebuffer_[static_cast<size_t>(y * width_ + x)] = Vec4(ldr, 1.0f);
        }
    }

    // Upload to output texture (recommended internal format: GL_RGBA16F)
    glBindTexture(GL_TEXTURE_2D, output_texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGBA, GL_FLOAT, framebuffer_.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

Vec3 CPURayTracer::trace_ray(const Ray& ray, int depth) {
    ARE_PROFILE_FUNCTION();

    if (depth <= 0) {
        return Vec3(0.0f);
    }

    HitRecord hit;
    if (!bvh_->intersect(ray, hit)) {
        // Simple sky
        Vec3 unit_dir = glm::normalize(ray.direction_);
        Real t = 0.5f * (unit_dir.y + 1.0f);
        return lerp(Vec3(1.0f), Vec3(0.5f, 0.7f, 1.0f), t);
    }

    return shade(hit, ray, depth);
}

Vec3 CPURayTracer::shade(const HitRecord& hit, const Ray& ray, int depth) {
    ARE_PROFILE_FUNCTION();

    Vec3 albedo(0.8f);
    Real metallic = 0.0f;
    Real roughness = 0.5f;

    if (scene_) {
        const Material* mat = scene_->get_material(hit.material_);
        if (mat) {
            albedo = mat->get_albedo();
            metallic = mat->get_metallic();
            roughness = mat->get_roughness();
        }
    }

    Vec3 direct = compute_direct_lighting(hit);
    Real ao = 1.0f;

    if (config_.enable_ao) {
        ao = compute_ambient_occlusion(hit);
    }

    Vec3 result = direct * ao;

    if (config_.enable_gi && depth > 1) {
        RandomGenerator& rng = get_thread_random();
        Vec3 bounce_dir = rng.random_cosine_direction(hit.normal_);
        Ray bounce_ray(offset_ray_origin(hit.position_, hit.normal_), bounce_dir, are_epsilon, 1e30f);

        Vec3 bounced = trace_ray(bounce_ray, depth - 1);
        result += albedo * bounced * 0.5f;
    }

    // Phase 5: Lambert base
    result *= albedo;

    (void)ray;
    (void)metallic;
    (void)roughness;

    return result;
}

Vec3 CPURayTracer::compute_direct_lighting(const HitRecord& hit) {
    ARE_PROFILE_FUNCTION();

    Vec3 lighting(0.0f);
    if (!scene_) {
        return lighting;
    }

    const auto& lights = scene_->get_all_lights();
    for (const auto& light_ptr : lights) {
        if (!light_ptr) {
            continue;
        }

        Vec3 L(0.0f);
        Real max_distance = 1e30f;
        Real attenuation = 1.0f;

        const LightType type = light_ptr->get_type();

        if (type == LightType::ARE_LIGHT_DIRECTIONAL) {
            const auto* dl = static_cast<const DirectionalLight*>(light_ptr.get());
            L = -glm::normalize(dl->get_direction());
        } else if (type == LightType::ARE_LIGHT_POINT) {
            const auto* pl = static_cast<const PointLight*>(light_ptr.get());
            Vec3 to_light = pl->get_position() - hit.position_;
            Real dist = glm::length(to_light);
            if (dist < are_epsilon) {
                continue;
            }
            if (!pl->affects_point(hit.position_)) {
                continue;
            }
            L = to_light / dist;
            max_distance = dist;
            attenuation = pl->calculate_attenuation(dist);
        } else if (type == LightType::ARE_LIGHT_SPOT) {
            const auto* sl = static_cast<const SpotLight*>(light_ptr.get());
            Vec3 to_light = sl->get_position() - hit.position_;
            Real dist = glm::length(to_light);
            if (dist < are_epsilon) {
                continue;
            }
            if (!sl->affects_point(hit.position_)) {
                continue;
            }
            L = to_light / dist;
            max_distance = dist;

            Vec3 light_to_point = glm::normalize(hit.position_ - sl->get_position());
            Real spot = sl->calculate_spot_factor(light_to_point);
            attenuation *= spot;
        } else {
            continue;
        }

        if (light_ptr->get_cast_shadows()) {
            if (is_in_shadow(offset_ray_origin(hit.position_, hit.normal_), L, max_distance)) {
                continue;
            }
        }

        Real n_dot_l = std::max(0.0f, glm::dot(hit.normal_, L));
        if (n_dot_l <= 0.0f) {
            continue;
        }

        Vec3 radiance = light_ptr->get_color() * light_ptr->get_intensity();
        lighting += radiance * n_dot_l * attenuation;
    }

    return lighting;
}

Real CPURayTracer::compute_ambient_occlusion(const HitRecord& hit) {
    ARE_PROFILE_FUNCTION();

    if (!bvh_) {
        return 1.0f;
    }

    const int ao_samples = std::max(1, config_.ao_samples);
    const Real radius = std::max(are_epsilon, config_.ao_radius);

    RandomGenerator& rng = get_thread_random();

    int occluded = 0;
    for (int i = 0; i < ao_samples; ++i) {
        Vec3 dir = rng.random_in_hemisphere(hit.normal_);
        Ray ao_ray(offset_ray_origin(hit.position_, hit.normal_), dir, are_epsilon, radius);

        if (bvh_->intersect_any(ao_ray, radius)) {
            occluded++;
        }
    }

    Real occ = static_cast<Real>(occluded) / static_cast<Real>(ao_samples);
    return 1.0f - occ;
}

bool CPURayTracer::is_in_shadow(const Vec3& origin, const Vec3& direction, Real max_distance) {
    ARE_PROFILE_FUNCTION();

    if (!bvh_) {
        return false;
    }

    Real t_max = (max_distance > 0.0f) ? max_distance : 1e30f;
    Ray shadow_ray(origin, direction, are_epsilon, t_max);
    return bvh_->intersect_any(shadow_ray, t_max);
}

} // namespace are
