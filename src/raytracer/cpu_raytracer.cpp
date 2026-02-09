/**
 * @file cpu_raytracer.cpp
 * @brief CPU hybrid ray tracer (GBuffer-driven) with geometric normal offset
 */

#include <are/raytracer/cpu_raytracer.h>

#include <are/acceleration/bvh.h>
#include <are/scene/scene_manager.h>
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
#include <vector>

namespace are {

namespace {

inline Real compute_ray_epsilon(const Vec3& p) {
    Real s = std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z), 1.0f});
    return 1e-4f * s;
}

inline Vec3 offset_ray_origin(const Vec3& p, const Vec3& ng) {
    Real eps = compute_ray_epsilon(p);
    return p + ng * (eps * 4.0f);
}

inline Vec3 tonemap_reinhard(const Vec3& c, Real exposure) {
    Vec3 x = c * exposure;
    return x / (Vec3(1.0f) + x);
}

inline Vec3 decode_albedo_from_rgba8(uint8_t r, uint8_t g, uint8_t b) {
    return Vec3(r, g, b) / 255.0f;
}

inline Real decode_01_from_u8(uint8_t v) {
    return static_cast<Real>(v) / 255.0f;
}

inline bool finite_vec3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
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
    (void)camera;

    if (!bvh_ || !bvh_->is_built()) {
        ARE_LOG_ERROR("CPURayTracer: BVH is null or not built");
        return;
    }

    if (!gbuffer) {
        ARE_LOG_CRITICAL("CPURayTracer: GBuffer is null (hybrid requires it)");
        throw std::runtime_error("CPURayTracer requires GBuffer in hybrid mode");
    }

    if (output_texture == 0) {
        ARE_LOG_ERROR("CPURayTracer: output_texture is 0");
        return;
    }

    scene_ = &scene;
    width_ = gbuffer->get_width();
    height_ = gbuffer->get_height();

    if (width_ <= 0 || height_ <= 0) {
        ARE_LOG_ERROR("CPURayTracer: Invalid resolution");
        return;
    }

    std::vector<Vec3> pos(static_cast<size_t>(width_ * height_));
    std::vector<Vec3> nrm(static_cast<size_t>(width_ * height_));
    std::vector<uint8_t> albedo_metallic(static_cast<size_t>(width_ * height_ * 4));
    std::vector<uint8_t> rough_ao(static_cast<size_t>(width_ * height_ * 2));
    std::vector<Real> depth(static_cast<size_t>(width_ * height_));
    std::vector<uint32_t> prim_id(static_cast<size_t>(width_ * height_));

    const_cast<GBuffer *>(gbuffer)->read_pixels(0, pos.data());
    const_cast<GBuffer *>(gbuffer)->read_pixels(1, nrm.data());
    const_cast<GBuffer *>(gbuffer)->read_pixels(2, albedo_metallic.data());
    const_cast<GBuffer *>(gbuffer)->read_pixels(3, rough_ao.data());
    const_cast<GBuffer *>(gbuffer)->read_pixels(4, depth.data());
    const_cast<GBuffer *>(gbuffer)->read_pixels(5, prim_id.data());

    framebuffer_.assign(static_cast<size_t>(width_ * height_), Vec3(0.0f));

    const int spp = std::max(1, config_.spp);
    const int max_depth = std::max(1, config_.max_depth);
    const auto& triangles = bvh_->get_triangles();

    for (int y = 0; y < height_; ++y) {
        RandomGenerator& rng = get_thread_random();

        for (int x = 0; x < width_; ++x) {
            const size_t idx = static_cast<size_t>(y * width_ + x);

            // Depth validity
            if (!(depth[idx] > 0.0f && depth[idx] < 0.999999f)) {
                framebuffer_[idx] = Vec3(0.0f);
                continue;
            }

            Vec3 P = pos[idx];
            Vec3 Ns = glm::normalize(nrm[idx]);

            if (!finite_vec3(P) || !finite_vec3(Ns) || glm::length(Ns) < 0.1f) {
                framebuffer_[idx] = Vec3(0.0f);
                continue;
            }

            // Geometric normal from primitive id
            Vec3 Ng = Ns;
            uint32_t pid = prim_id[idx];
            if (pid < triangles.size()) {
                Ng = triangles[pid].normal();
            }

            const uint8_t* am = &albedo_metallic[idx * 4];
            Vec3 albedo = decode_albedo_from_rgba8(am[0], am[1], am[2]);
            (void)decode_01_from_u8(am[3]);

            const uint8_t* ra = &rough_ao[idx * 2];
            (void)decode_01_from_u8(ra[0]);
            Real ao_gbuffer = decode_01_from_u8(ra[1]);

            Vec3 accum(0.0f);

            for (int s = 0; s < spp; ++s) {
                HitRecord surf;
                surf.position_ = P;
                surf.normal_ = Ns;
                surf.t_ = 1.0f;
                surf.material_ = are_invalid_handle;

                // Direct lighting (shadow uses robust epsilon)
                Vec3 direct = compute_direct_lighting(surf);

                // AO
                Real ao = 1.0f;
                if (config_.enable_ao) {
                    ao = compute_ambient_occlusion(surf);
                }
                ao *= ao_gbuffer;

                // GI (simplified)
                Vec3 gi(0.0f);
                if (config_.enable_gi && max_depth > 1) {
                    Vec3 bounce_dir = rng.random_cosine_direction(Ns);
                    Vec3 origin = offset_ray_origin(P, Ng);
                    Real eps = compute_ray_epsilon(P);
                    Ray bounce(origin, bounce_dir, eps * 4.0f, 1e30f);
                    gi = trace_ray(bounce, max_depth - 1);
                }

                Vec3 c = albedo * direct * ao + albedo * gi;
                accum += c;
            }

            Vec3 hdr = accum / static_cast<Real>(spp);
            framebuffer_[idx] = tonemap_reinhard(hdr, 1.0f);
        }
    }

    std::vector<Vec4> rgba(static_cast<size_t>(width_ * height_));
    for (size_t i = 0; i < rgba.size(); ++i) {
        rgba[i] = Vec4(framebuffer_[i], 1.0f);
    }

    glBindTexture(GL_TEXTURE_2D, output_texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGBA, GL_FLOAT, rgba.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

Vec3 CPURayTracer::trace_ray(const Ray& ray, int depth) {
    if (depth <= 0) {
        return Vec3(0.0f);
    }

    HitRecord hit;
    if (!bvh_->intersect(ray, hit)) {
        return Vec3(0.0f);
    }

    Vec3 Ng = hit.normal_;
    if (hit.triangle_index_ < bvh_->get_triangles().size()) {
        Ng = bvh_->get_triangles()[hit.triangle_index_].normal();
    }

    RandomGenerator& rng = get_thread_random();
    Vec3 dir = rng.random_cosine_direction(hit.normal_);
    Vec3 origin = offset_ray_origin(hit.position_, Ng);
    Real eps = compute_ray_epsilon(hit.position_);

    Ray bounce(origin, dir, eps * 4.0f, 1e30f);
    return trace_ray(bounce, depth - 1);
}

Vec3 CPURayTracer::compute_direct_lighting(const HitRecord& hit) {
    Vec3 lighting(0.0f);
    if (!scene_) {
        return lighting;
    }

    const auto& lights = scene_->get_all_lights();
    for (const auto& light_ptr : lights) {
        if (!light_ptr) continue;

        Vec3 L(0.0f);
        Real max_distance = 1e30f;
        Real attenuation = 1.0f;

        LightType type = light_ptr->get_type();

        if (type == LightType::ARE_LIGHT_DIRECTIONAL) {
            const auto* dl = static_cast<const DirectionalLight*>(light_ptr.get());
            L = -glm::normalize(dl->get_direction());
        } else if (type == LightType::ARE_LIGHT_POINT) {
            const auto* pl = static_cast<const PointLight*>(light_ptr.get());
            Vec3 to_light = pl->get_position() - hit.position_;
            Real dist = glm::length(to_light);
            if (dist < are_epsilon) continue;
            if (!pl->affects_point(hit.position_)) continue;
            L = to_light / dist;
            max_distance = dist;
            attenuation = pl->calculate_attenuation(dist);
        } else if (type == LightType::ARE_LIGHT_SPOT) {
            const auto* sl = static_cast<const SpotLight*>(light_ptr.get());
            Vec3 to_light = sl->get_position() - hit.position_;
            Real dist = glm::length(to_light);
            if (dist < are_epsilon) continue;
            if (!sl->affects_point(hit.position_)) continue;
            L = to_light / dist;
            max_distance = dist;
            Vec3 light_to_point = glm::normalize(hit.position_ - sl->get_position());
            attenuation *= sl->calculate_spot_factor(light_to_point);
        } else {
            continue;
        }

        if (light_ptr->get_cast_shadows()) {
            Real eps = compute_ray_epsilon(hit.position_);
            Vec3 origin = hit.position_ + hit.normal_ * (eps * 4.0f);
            Ray shadow(origin, L, eps * 4.0f, max_distance);
            if (bvh_ && bvh_->intersect_any(shadow, max_distance)) {
                continue;
            }
        }

        Real n_dot_l = std::max(0.0f, glm::dot(hit.normal_, L));
        if (n_dot_l <= 0.0f) continue;

        Vec3 radiance = light_ptr->get_color() * light_ptr->get_intensity();
        lighting += radiance * n_dot_l * attenuation;
    }

    return lighting;
}

Real CPURayTracer::compute_ambient_occlusion(const HitRecord& hit) {
    if (!bvh_) {
        return 1.0f;
    }

    const int ao_samples = std::max(1, config_.ao_samples);
    const Real radius = std::max(are_epsilon, config_.ao_radius);

    RandomGenerator& rng = get_thread_random();

    int occluded = 0;
    for (int i = 0; i < ao_samples; ++i) {
        Vec3 dir = rng.random_in_hemisphere(hit.normal_);
        Real eps = compute_ray_epsilon(hit.position_);
        Vec3 origin = hit.position_ + hit.normal_ * (eps * 4.0f);
        Ray ao_ray(origin, dir, eps * 4.0f, radius);
        if (bvh_->intersect_any(ao_ray, radius)) {
            occluded++;
        }
    }

    Real occ = static_cast<Real>(occluded) / static_cast<Real>(ao_samples);
    return 1.0f - occ;
}

bool CPURayTracer::is_in_shadow(const Vec3& origin, const Vec3& direction, Real max_distance, uint32_t ignore_triangle) {
    if (!bvh_) return false;

    Real t_max = (max_distance > 0.0f) ? max_distance : 1e30f;
    Real eps = compute_ray_epsilon(origin);
    Ray shadow(origin, direction, eps * 4.0f, t_max);
    return bvh_->intersect_any(shadow, t_max, ignore_triangle);
}

} // namespace are
