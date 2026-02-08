很好！现在让我们开始Phase 5的实现吧！如下是Phase 5我已经写好的头文件与依赖文件（注：ray.h/cpp&hit_record.h/cpp在前面的代码中你已经实现了，现在只需要再检查一遍之前的实现是否正确即可）：
### 文件：include/are/raytracer/cpu_raytracer.h
```cpp
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
    bool is_in_shadow(const Vec3& origin, const Vec3& direction, Real max_distance);

    const BVH* bvh_;                      ///< BVH reference
    const SceneManager* scene_;           ///< Scene reference
    std::vector<Vec3> framebuffer_;       ///< CPU framebuffer (HDR)
    int width_;                           ///< Framebuffer width
    int height_;                          ///< Framebuffer height
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_CPU_RAYTRACER_H
```

### 文件：include/are/raytracer/raytracer.h
```cpp
/**
 * @file raytracer.h
 * @brief Ray tracing interface
 */

#ifndef ARE_INCLUDE_RAYTRACER_RAYTRACER_H
#define ARE_INCLUDE_RAYTRACER_RAYTRACER_H

#include <are/core/types.h>
#include <are/core/config.h>

namespace are {

// Forward declarations
class SceneManager;
class Camera;
class GBuffer;
class BVH;

/**
 * @class RayTracer
 * @brief Abstract ray tracing interface
 * 
 * Base class for CPU and GPU ray tracing implementations.
 */
class RayTracer {
public:
    /**
     * @brief Constructor
     * @param config Ray tracing configuration
     */
    explicit RayTracer(const RayTracingConfig& config);

    /**
     * @brief Virtual destructor
     */
    virtual ~RayTracer() = default;

    /**
     * @brief Render scene using ray tracing
     * @param scene Scene manager
     * @param camera Camera
     * @param gbuffer G-Buffer (optional, for hybrid rendering)
     * @param output Output texture ID
     */
    virtual void render(const SceneManager& scene, 
                       const Camera& camera,
                       const GBuffer* gbuffer,
                       uint32_t output_texture) = 0;

    /**
     * @brief Update BVH
     * @param bvh BVH reference
     */
    virtual void update_bvh(const BVH& bvh) = 0;

    /**
     * @brief Set configuration
     * @param config New configuration
     */
    virtual void set_config(const RayTracingConfig& config);

    /**
     * @brief Get configuration
     * @return Current configuration
     */
    const RayTracingConfig& get_config() const { return config_; }

protected:
    RayTracingConfig config_;             ///< Ray tracing configuration
};

} // namespace are
#endif // ARE_INCLUDE_RAYTRACER_RAYTRACER_H
```
此外，你可能需要随机数模块。
### 文件：include/utils/random.h
```cpp
/**
 * @file random.h
 * @brief Random number generation utilities
 */

#ifndef ARE_INCLUDE_UTILS_RANDOM_H
#define ARE_INCLUDE_UTILS_RANDOM_H

#include <are/core/types.h>
#include <random>

namespace are {

/**
 * @class RandomGenerator
 * @brief Thread-safe random number generator
 *
 * Uses PCG (Permuted Congruential Generator) for high-quality random numbers.
 */
class RandomGenerator {
public:
	/**
	 * @brief Constructor with optional seed
	 * @param seed Random seed (0 = use random device)
	 */
	explicit RandomGenerator(uint64_t seed = 0);

	/**
	 * @brief Generate random float in [0, 1)
	 * @return Random float
	 */
	Real random_float();

	/**
	 * @brief Generate random float in [min, max)
	 * @param min Minimum value
	 * @param max Maximum value
	 * @return Random float
	 */
	Real random_float(Real min, Real max);

	/**
	 * @brief Generate random integer in [min, max]
	 * @param min Minimum value
	 * @param max Maximum value
	 * @return Random integer
	 */
	int random_int(int min, int max);

	/**
	 * @brief Generate random point in unit disk
	 * @return Random point (z = 0)
	 */
	Vec3 random_in_unit_disk();

	/**
	 * @brief Generate random point in unit sphere
	 * @return Random point
	 */
	Vec3 random_in_unit_sphere();

	/**
	 * @brief Generate random unit vector
	 * @return Random unit vector
	 */
	Vec3 random_unit_vector();

	/**
	 * @brief Generate random vector in hemisphere
	 * @param normal Hemisphere normal
	 * @return Random vector in hemisphere
	 */
	Vec3 random_in_hemisphere(const Vec3 &normal);

	/**
	 * @brief Generate random cosine-weighted direction
	 * @param normal Surface normal
	 * @return Random direction (cosine-weighted)
	 */
	Vec3 random_cosine_direction(const Vec3 &normal);

	/**
	 * @brief Set seed for reproducible results
	 * @param seed Random seed
	 */
	void set_seed(uint64_t seed);

private:
	std::mt19937_64 rng_; ///< Random number generator
	std::uniform_real_distribution<Real> dist_; ///< Uniform distribution [0, 1)
};

/**
 * @brief Get thread-local random generator
 * @return Reference to thread-local generator
 */
RandomGenerator &get_thread_random();

/**
 * @brief Generate random float in [0, 1) using thread-local generator
 * @return Random float
 */
inline Real random_float() {
	return get_thread_random().random_float();
}

/**
 * @brief Generate random float in [min, max) using thread-local generator
 * @param min Minimum value
 * @param max Maximum value
 * @return Random float
 */
inline Real random_float(Real min, Real max) {
	return get_thread_random().random_float(min, max);
}

} // namespace are

#endif // ARE_INCLUDE_UTILS_RANDOM_H
```

同样地，如果有依赖或者需要给你的头文件/实现文件还没有给你，欢迎提出；如果已有的数学或者随机数模块需要补充，请分别给出函数声明与实现。
