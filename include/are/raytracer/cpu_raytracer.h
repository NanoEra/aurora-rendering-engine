/**
 * @file cpu_raytracer.h
 * @brief CPU-based ray tracing implementation
 */

#ifndef ARE_INCLUDE_RAYTRACER_CPU_RAYTRACER_H
#define ARE_INCLUDE_RAYTRACER_CPU_RAYTRACER_H

#include <are/raytracer/raytracer.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>
#include <vector>

namespace are {

/**
 * @class CPURayTracer
 * @brief CPU-based ray tracing implementation
 * 
 * Uses multithreading for parallel ray tracing on CPU.
 */
class CPURayTracer : public RayTracer {
public:
    /**
     * @brief Constructor
     * @param config Ray tracing configuration
     */
    explicit CPURayTracer(const RayTracingConfig& config);

    /**
     * @brief Destructor
     */
    ~CPURayTracer() override;

    /**
     * @brief Render scene using CPU ray tracing
     * @param scene Scene manager
     * @param camera Camera
     * @param gbuffer G-Buffer (optional)
     * @param output Output texture ID
     */
    void render(const SceneManager& scene, 
               const Camera& camera,
               const GBuffer* gbuffer,
               uint32_t output_texture) override;

    /**
     * @brief Update BVH
     * @param bvh BVH reference
     */
    void update_bvh(const BVH& bvh) override;

private:
    /**
     * @brief Trace a single ray
     * @param ray Ray to trace
     * @param depth Current recursion depth
     * @return Ray color
     */
    Vec3 trace_ray(const Ray& ray, int depth);

    /**
     * @brief Shade hit point
     * @param hit Hit record
     * @param ray Incident ray
     * @param depth Current recursion depth
     * @return Shaded color
     */
    Vec3 shade(const HitRecord& hit, const Ray& ray, int depth);

    /**
     * @brief Compute direct lighting
     * @param hit Hit record
     * @return Direct lighting contribution
     */
    Vec3 compute_direct_lighting(const HitRecord& hit);

    /**
     * @brief Compute ambient occlusion
     * @param hit Hit record
     * @return AO factor [0, 1]
     */
    Real compute_ambient_occlusion(const HitRecord& hit);

    /**
     * @brief Check shadow ray
     * @param origin Shadow ray origin
     * @param direction Shadow ray direction
     * @param max_distance Maximum distance
     * @return true if in shadow
     */
	bool is_in_shadow(const Vec3& origin, const Vec3& direction, Real max_distance, uint32_t ignore_triangle);

    const BVH* bvh_;                      ///< BVH reference
    const SceneManager* scene_;           ///< Scene reference
    std::vector<Vec3> framebuffer_;       ///< CPU framebuffer (HDR)
    int width_;                           ///< Framebuffer width
    int height_;                          ///< Framebuffer height
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_CPU_RAYTRACER_H
