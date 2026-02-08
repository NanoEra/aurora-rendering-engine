我其实已经实现了所有头文件了，包括所有模块的所有接口以及使用方法都在里面，所以让我直接给你所有头文件的完整源码吧：

### 文件：include/are/are.h

```cpp
/**
 * @file are.h
 * @brief Aurora Rendering Engine - Main header file
 * 
 * This header includes all public interfaces of the Aurora Rendering Engine.
 * Users only need to include this single header to access all functionality.
 * 
 * @author Ternary_Operator
 * @version 1.0
 */

#ifndef ARE_INCLUDE_ARE_H
#define ARE_INCLUDE_ARE_H

// Core modules
#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/core/types.h>
#include <are/core/profiler.h>

// Platform modules
#include <are/platform/window.h>
#include <are/platform/gl_context.h>

// Geometry modules
#include <are/geometry/vertex.h>
#include <are/geometry/triangle.h>
#include <are/geometry/aabb.h>
#include <are/geometry/transform.h>

// Scene modules
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <are/scene/directional_light.h>
#include <are/scene/point_light.h>
#include <are/scene/spot_light.h>
#include <are/scene/scene_manager.h>

// Acceleration modules
#include <are/acceleration/bvh.h>

// Renderer modules
#include <are/renderer/renderer.h>
#include <are/renderer/render_stats.h>

// Utility modules
#include <are/utils/image_io.h>
#include <are/utils/math_utils.h>

/**
 * @namespace are
 * @brief Main namespace for Aurora Rendering Engine
 */
namespace are {

/**
 * @brief Get the version string of the engine
 * @return Version string in format "major.minor.patch"
 */
const char* get_version();

/**
 * @brief Initialize the Aurora Rendering Engine
 * @return true if initialization succeeded, false otherwise
 */
bool initialize();

/**
 * @brief Shutdown the Aurora Rendering Engine
 */
void shutdown();

} // namespace are

#endif // ARE_INCLUDE_ARE_H
```

### 文件：include/are/core/types.h

```cpp
/**
 * @file types.h
 * @brief Basic type definitions and constants
 */

#ifndef ARE_INCLUDE_CORE_TYPES_H
#define ARE_INCLUDE_CORE_TYPES_H

#include <cstdint>
#include <glm/glm.hpp>

namespace are {

// Floating point precision
using Real = float;

// Common vector types
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;

using Vec2i = glm::ivec2;
using Vec3i = glm::ivec3;
using Vec4i = glm::ivec4;

// Matrix types
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;

// Color type (RGBA)
using Color = Vec4;

// Handle types for resource management
using MeshHandle = uint32_t;
using MaterialHandle = uint32_t;
using LightHandle = uint32_t;
using TextureHandle = uint32_t;

// Invalid handle constant
constexpr uint32_t are_invalid_handle = 0xFFFFFFFF;

// Mathematical constants
constexpr Real are_pi = 3.14159265358979323846f;
constexpr Real are_two_pi = 6.28318530717958647692f;
constexpr Real are_inv_pi = 0.31830988618379067154f;
constexpr Real are_inv_two_pi = 0.15915494309189533577f;
constexpr Real are_epsilon = 1e-6f;

// Ray tracing backend types
enum class RayTracingBackend {
    ARE_RT_BACKEND_CPU,
    ARE_RT_BACKEND_COMPUTE_SHADER
};

// Tone mapping operators
enum class ToneMappingOperator {
    ARE_TONEMAP_NONE,
    ARE_TONEMAP_REINHARD,
    ARE_TONEMAP_ACES
};

// G-Buffer visualization modes
enum class GBufferVisualizationMode {
    ARE_GBUFFER_VIS_NONE,
    ARE_GBUFFER_VIS_POSITION,
    ARE_GBUFFER_VIS_NORMAL,
    ARE_GBUFFER_VIS_ALBEDO,
    ARE_GBUFFER_VIS_METALLIC,
    ARE_GBUFFER_VIS_ROUGHNESS,
    ARE_GBUFFER_VIS_DEPTH
};

} // namespace are

#endif // ARE_INCLUDE_CORE_TYPES_H
```

### 文件：include/are/core/config.h

```cpp
/**
 * @file config.h
 * @brief Configuration system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_CONFIG_H
#define ARE_INCLUDE_CORE_CONFIG_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @struct WindowConfig
 * @brief Configuration for window creation
 */
struct WindowConfig {
	int width = 1280; ///< Window width in pixels
	int height = 720; ///< Window height in pixels
	std::string title = "Aurora Rendering Engine"; ///< Window title
	bool resizable = true; ///< Whether window is resizable
	bool vsync = true; ///< Enable vertical synchronization
	int samples = 1; ///< MSAA samples (1 = disabled)
};

/**
 * @struct RayTracingConfig
 * @brief Configuration for ray tracing
 */
struct RayTracingConfig {
	RayTracingBackend backend = RayTracingBackend::ARE_RT_BACKEND_COMPUTE_SHADER;
	int spp = 64; ///< Samples per pixel
	int max_depth = 8; ///< Maximum ray bounce depth
	bool enable_gi = true; ///< Enable global illumination
	bool enable_ao = true; ///< Enable ambient occlusion
	bool enable_soft_shadows = false; ///< Enable soft shadows
	int ao_samples = 16; ///< AO sample count
	Real ao_radius = 1.0f; ///< AO sampling radius
};

/**
 * @struct RenderConfig
 * @brief General rendering configuration
 */
struct RenderConfig {
	ToneMappingOperator tonemap_op = ToneMappingOperator::ARE_TONEMAP_ACES;
	Real exposure = 1.0f; ///< Exposure value for tone mapping
	bool use_hdr = true; ///< Use HDR rendering pipeline
	GBufferVisualizationMode gbuffer_vis_mode = GBufferVisualizationMode::ARE_GBUFFER_VIS_NONE;
};

/**
 * @struct PerformanceConfig
 * @brief Performance-related configuration
 */
struct PerformanceConfig {
	int num_threads = 0; ///< Number of threads (0 = auto-detect)
	bool enable_bvh_multithreading = true; ///< Use multithreading for BVH construction
	bool enable_profiling = false; ///< Enable performance profiling
};

/**
 * @struct PathConfig
 * @brief File path configuration
 */
struct PathConfig {
	std::string shader_dir = "shaders/"; ///< Directory containing shader files
	std::string texture_dir = "textures/"; ///< Default texture directory
	std::string output_dir = "output/"; ///< Default output directory
};

/**
 * @class AreConfig
 * @brief Main configuration class for Aurora Rendering Engine
 */
class AreConfig {
public:
	WindowConfig window; ///< Window configuration
	RayTracingConfig ray_tracing; ///< Ray tracing configuration
	RenderConfig render; ///< Rendering configuration
	PerformanceConfig performance; ///< Performance configuration
	PathConfig paths; ///< Path configuration

	/**
	 * @brief Constructor with default values
	 */
	AreConfig() = default;

	/**
	 * @brief Validate configuration values
	 * @return true if configuration is valid, false otherwise
	 */
	bool validate() const;

	/**
	 * @brief Print configuration to console
	 */
	void print() const;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_CONFIG_H
```

### 文件：include/are/core/logger.h

```cpp
/**
 * @file logger.h
 * @brief Logging system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_LOGGER_H
#define ARE_INCLUDE_CORE_LOGGER_H

#include <string>
#include <memory>

namespace are {

/**
 * @enum LogLevel
 * @brief Logging severity levels
 */
enum class LogLevel {
    ARE_LOG_TRACE,
    ARE_LOG_DEBUG,
    ARE_LOG_INFO,
    ARE_LOG_WARN,
    ARE_LOG_ERROR,
    ARE_LOG_CRITICAL
};

/**
 * @class Logger
 * @brief Thread-safe logging system
 * 
 * This class provides a simple interface for logging messages with different
 * severity levels. It wraps spdlog for actual logging functionality.
 */
class Logger {
public:
    /**
     * @brief Initialize the logging system
     * @param min_level Minimum log level to display
     */
    static void init(LogLevel min_level = LogLevel::ARE_LOG_INFO);

    /**
     * @brief Shutdown the logging system
     */
    static void shutdown();

    /**
     * @brief Log a message with file/function/line information
     * @param level Log severity level
     * @param file Source file name
     * @param func Function name
     * @param line Line number
     * @param message Log message
     */
    static void log(LogLevel level, const char* file, const char* func, 
                   int line, const std::string& message);

    /**
     * @brief Set minimum log level
     * @param level Minimum log level to display
     */
    static void set_level(LogLevel level);

private:
    static std::shared_ptr<void> logger_impl_; ///< Internal logger implementation
    static bool initialized_;                   ///< Initialization flag
};

} // namespace are

// Logging macros
#define ARE_LOG_TRACE(msg)    are::Logger::log(are::LogLevel::ARE_LOG_TRACE, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_DEBUG(msg)    are::Logger::log(are::LogLevel::ARE_LOG_DEBUG, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_INFO(msg)     are::Logger::log(are::LogLevel::ARE_LOG_INFO, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_WARN(msg)     are::Logger::log(are::LogLevel::ARE_LOG_WARN, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_ERROR(msg)    are::Logger::log(are::LogLevel::ARE_LOG_ERROR, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_CRITICAL(msg) are::Logger::log(are::LogLevel::ARE_LOG_CRITICAL, __FILE__, __func__, __LINE__, msg)

#endif // ARE_INCLUDE_CORE_LOGGER_H
```

### 文件：include/are/core/profiler.h

```cpp
/**
 * @file profiler.h
 * @brief Performance profiling utilities
 */

#ifndef ARE_INCLUDE_CORE_PROFILER_H
#define ARE_INCLUDE_CORE_PROFILER_H

#include <are/core/types.h>
#include <string>
#include <chrono>
#include <unordered_map>

namespace are {

/**
 * @struct ProfileResult
 * @brief Result of a profiling measurement
 */
struct ProfileResult {
    std::string name_;                    ///< Profile section name
    double duration_ms_;                  ///< Duration in milliseconds
    uint64_t call_count_;                 ///< Number of times called
    double avg_duration_ms_;              ///< Average duration per call
};

/**
 * @class Profiler
 * @brief Simple performance profiler
 * 
 * This class provides basic timing functionality for performance analysis.
 * It is only active when ARE_ENABLE_PROFILING is defined.
 */
class Profiler {
public:
    /**
     * @brief Initialize the profiler
     */
    static void init();

    /**
     * @brief Shutdown the profiler and print results
     */
    static void shutdown();

    /**
     * @brief Begin a profiling section
     * @param name Section name
     */
    static void begin(const std::string& name);

	/**
     * @brief End a profiling section
     * @param name Section name
     */
    static void end(const std::string& name);

    /**
     * @brief Get profiling results
     * @return Map of section names to profile results
     */
    static const std::unordered_map<std::string, ProfileResult>& get_results();

    /**
     * @brief Reset all profiling data
     */
    static void reset();

    /**
     * @brief Print profiling results to console
     */
    static void print_results();

private:
    struct SectionData {
        std::chrono::high_resolution_clock::time_point start_time_;
        double total_duration_ms_ = 0.0;
        uint64_t call_count_ = 0;
    };

    static std::unordered_map<std::string, SectionData> sections_;
    static std::unordered_map<std::string, ProfileResult> results_;
    static bool enabled_;
};

/**
 * @class ScopedProfiler
 * @brief RAII-style profiler for automatic timing
 */
class ScopedProfiler {
public:
    /**
     * @brief Constructor - begins profiling
     * @param name Section name
     */
    explicit ScopedProfiler(const std::string& name);

    /**
     * @brief Destructor - ends profiling
     */
    ~ScopedProfiler();

private:
    std::string name_;
};

} // namespace are

// Profiling macros
#ifdef ARE_ENABLE_PROFILING
	#define ARE_PROFILE_BEGIN(name) are::Profiler::begin(name)
    #define ARE_PROFILE_END(name)   are::Profiler::end(name)
    #define ARE_PROFILE_SCOPE(name) are::ScopedProfiler are_profiler_##__LINE__(name)
    #define ARE_PROFILE_FUNCTION()  ARE_PROFILE_SCOPE(__func__)
#else
    #define ARE_PROFILE_BEGIN(name) ((void)0)
    #define ARE_PROFILE_END(name)   ((void)0)
    #define ARE_PROFILE_SCOPE(name) ((void)0)
    #define ARE_PROFILE_FUNCTION()  ((void)0)
#endif

#endif // ARE_INCLUDE_CORE_PROFILER_H
```

### 文件：include/are/geometry/vertex.h

```cpp
/**
 * @file vertex.h
 * @brief Vertex data structure definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_VERTEX_H
#define ARE_INCLUDE_GEOMETRY_VERTEX_H

#include <are/core/types.h>

namespace are {

/**
 * @struct Vertex
 * @brief Standard vertex structure for mesh data
 * 
 * Contains position, normal, texture coordinates, and tangent information
 * for PBR rendering pipeline.
 */
struct Vertex {
    Vec3 position_;///< Vertex position in object space
    Vec3 normal_;                         ///< Vertex normal (normalized)
    Vec2 texcoord_;                       ///< Texture coordinates (UV)
    Vec3 tangent_;                        ///< Tangent vector for normal mapping

    Vertex() = default;

    /**
     * @brief Construct vertex with position only
     * @param pos Position
     */
    explicit Vertex(const Vec3& pos);

    /**
     * @brief Construct vertex with position and normal
     * @param pos Position
     * @param norm Normal
     */
    Vertex(const Vec3& pos, const Vec3& norm);

    /**
     * @brief Construct vertex with position, normal, and texcoord
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv);

    /**
     * @brief Construct vertex with all attributes
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     * @param tan Tangent
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv, const Vec3& tan);

    /**
     * @brief Interpolate between two vertices
     * @param a First vertex
     * @param b Second vertex
     * @param t Interpolation factor [0, 1]
     * @return Interpolated vertex
     */
    static Vertex lerp(const Vertex& a, const Vertex& b, Real t);
};

/**
 * @brief Get vertex attribute stride for OpenGL
 * @return Size of Vertex structure in bytes
 */
constexpr size_t get_vertex_stride() {
    return sizeof(Vertex);
}

/**
 * @brief Get offset of position attribute
 * @return Offset in bytes
 */
constexpr size_t get_position_offset() {
    return offsetof(Vertex, position_);
}

/**
 * @brief Get offset of normal attribute
 * @return Offset in bytes
 */
constexpr size_t get_normal_offset() {
    return offsetof(Vertex, normal_);
}

/**
 * @brief Get offset of texcoord attribute
 * @return Offset in bytes
 */
constexpr size_t get_texcoord_offset() {
    return offsetof(Vertex, texcoord_);
}

/**
 * @brief Get offset of tangent attribute
 * @return Offset in bytes
 */
constexpr size_t get_tangent_offset() {
    return offsetof(Vertex, tangent_);
}

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_VERTEX_H
```

### 文件：include/are/geometry/aabb.h

```cpp
/**
 * @file aabb.h
 * @brief Axis-Aligned Bounding Box implementation
 */

#ifndef ARE_INCLUDE_GEOMETRY_AABB_H
#define ARE_INCLUDE_GEOMETRY_AABB_H

#include <are/core/types.h>

namespace are {

// Forward declaration
struct Ray;

/**
 * @class AABB
 * @brief Axis-Aligned Bounding Box for spatial queries
 * 
 * Used for BVH construction and ray intersection acceleration.
 */
class AABB {
public:
    Vec3 min_;                            ///< Minimum corner
    Vec3 max_;                            ///< Maximum corner

    /**
     * @brief Default constructor - creates invalid AABB
     */
    AABB();

    /**
     * @brief Construct AABB from min and max corners
     * @param min Minimum corner
     * @param max Maximum corner
     */
    AABB(const Vec3& min, const Vec3& max);

    /**
     * @brief Construct AABB containing a single point
     * @param point Point to contain
     */
    explicit AABB(const Vec3& point);

    /**
     * @brief Check if AABB is valid (min <= max)
     * @return true if valid
     */
    bool is_valid() const;

    /**
     * @brief Get center of AABB
     * @return Center point
     */
    Vec3 center() const;

    /**
     * @brief Get size (extent) of AABB
     * @return Size vector
     */
    Vec3 size() const;

    /**
     * @brief Get surface area of AABB
     * @return Surface area
     */
    Real surface_area() const;

    /**
     * @brief Get volume of AABB
     * @return Volume
     */
    Real volume() const;

    /**
     * @brief Get longest axis index (0=x, 1=y, 2=z)
     * @return Axis index
     */
    int longest_axis() const;

    /**
     * @brief Expand AABB to include a point
     * @param point Point to include
     */
    void expand(const Vec3& point);

    /**
     * @brief Expand AABB to include another AABB
     * @param other AABB to include
     */
    void expand(const AABB& other);

    /**
     * @brief Check if AABB contains a point
     * @param point Point to check
     * @return true if point is inside AABB
     */
    bool contains(const Vec3& point) const;

    /**
     * @brief Check if AABB intersects another AABB
     * @param other AABB to check
     * @return true if AABBs intersect
     */
    bool intersects(const AABB& other) const;

    /**
     * @brief Ray-AABB intersection test
     * @param ray Ray to test
     * @param t_min Minimum t value (output)
     * @param t_max Maximum t value (output)
     * @return true if ray intersects AABB
     */
    bool intersect_ray(const Ray& ray, Real& t_min, Real& t_max) const;

    /**
     * @brief Merge two AABBs
     * @param a First AABB
     * @param b Second AABB
     * @return Merged AABB containing both
     */
    static AABB merge(const AABB& a, const AABB& b);

    /**
     * @brief Create invalid AABB (for initialization)
     * @return Invalid AABB
     */
    static AABB invalid();
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_AABB_H
```

### 文件：include/are/geometry/triangle.h

```cpp
/**
 * @file triangle.h
 * @brief Triangle primitive definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRIANGLE_H
#define ARE_INCLUDE_GEOMETRY_TRIANGLE_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>

namespace are {

// Forward declaration
struct Ray;
struct HitRecord;

/**
 * @struct Triangle
 * @brief Triangle primitive for ray tracing
 * 
 * Stores three vertices and provides intersection testing.
 */
struct Triangle {
    Vertex v0_;                           ///< First vertex
    Vertex v1_;                           ///< Second vertex
    Vertex v2_;                           ///< Third vertex
    MaterialHandle material_;             ///< Material handle

    /**
     * @brief Default constructor
     */
    Triangle();

    /**
     * @brief Construct triangle from three vertices
     * @param v0 First vertex
     * @param v1 Second vertex
     * @param v2 Third vertex
     * @param material Material handle
     */
    Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2,
            MaterialHandle material = are_invalid_handle);

    /**
     * @brief Get triangle centroid
     * @return Centroid position
     */
    Vec3 centroid() const;

    /**
     * @brief Get triangle normal (geometric normal)
     * @return Normal vector (normalized)
     */
    Vec3 normal() const;

    /**
     * @brief Get triangle area
     * @return Area
     */
    Real area() const;

    /**
     * @brief Compute axis-aligned bounding box
     * @return AABB containing the triangle
     */
    AABB compute_aabb() const;

    /**
     * @brief Ray-triangle intersection test (Möller-Trumbore algorithm)
     * @param ray Ray to test
     * @param hit Output hit record
     * @return true if intersection occurred
     */
    bool intersect(const Ray& ray, HitRecord& hit) const;

    /**
     * @brief Fast ray-triangle intersection test (no hit record)
     * @param ray Ray to test
     * @param t_max Maximum t value
     * @return true if intersection occurred
     */
    bool intersect_fast(const Ray& ray, Real t_max) const;
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRIANGLE_H
```

### 文件：include/are/geometry/transform.h

```cpp
/**
 * @file transform.h
 * @brief Transformation matrix utilities
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRANSFORM_H
#define ARE_INCLUDE_GEOMETRY_TRANSFORM_H

#include <are/core/types.h>

namespace are {

/**
 * @class Transform
 * @brief 3D transformation (position, rotation, scale)
 * 
 * Provides convenient interface for building transformation matrices.
 */
class Transform {
public:
    /**
     * @brief Default constructor (identity transform)
     */
    Transform();

    /**
     * @brief Construct from position, rotation, and scale
     * @param position Translation
     * @param rotation Rotation (Euler angles in radians)
     * @param scale Scale factors
     */
    Transform(const Vec3& position, const Vec3& rotation, const Vec3& scale);

    // Setters
    void set_position(const Vec3& position);
    void set_rotation(const Vec3& rotation);
    void set_scale(const Vec3& scale);
    void set_scale(Real uniform_scale);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_rotation() const { return rotation_; }
    const Vec3& get_scale() const { return scale_; }

    // Matrix operations
    Mat4 get_matrix() const;
    Mat4 get_inverse_matrix() const;
    Mat3 get_normal_matrix() const;

    /**
     * @brief Transform a point
     * @param point Point to transform
     * @return Transformed point
     */
    Vec3 transform_point(const Vec3& point) const;

    /**
     * @brief Transform a direction (ignores translation)
     * @param direction Direction to transform
     * @return Transformed direction
     */
    Vec3 transform_direction(const Vec3& direction) const;

    /**
     * @brief Transform a normal (uses inverse transpose)
     * @param normal Normal to transform
     * @return Transformed normal
     */
    Vec3 transform_normal(const Vec3& normal) const;

    /**
     * @brief Combine two transforms
     * @param other Other transform
     * @return Combined transform
     */
    Transform operator*(const Transform& other) const;

    /**
     * @brief Create identity transform
     * @return Identity transform
     */
    static Transform identity();

    /**
     * @brief Create translation transform
     * @param translation Translation vector
     * @return Translation transform
     */
    static Transform translate(const Vec3& translation);

    /**
     * @brief Create rotation transform
     * @param rotation Rotation (Euler angles in radians)
     * @return Rotation transform
     */
    static Transform rotate(const Vec3& rotation);

    /**
     * @brief Create scale transform
     * @param scale Scale factors
     * @return Scale transform
     */
    static Transform scale(const Vec3& scale);

private:
    void mark_dirty();
    void update_matrix() const;

    Vec3 position_;                       ///< Translation
    Vec3 rotation_;                       ///< Rotation (Euler angles)
    Vec3 scale_;                          ///< Scale factors

    mutable Mat4 matrix_;                 ///< Cached transformation matrix
    mutable Mat4 inverse_matrix_;         ///< Cached inverse matrix
    mutable bool dirty_;                  ///< Matrix needs update
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRANSFORM_H
```

### 文件：include/are/scene/camera.h

```cpp
/**
 * @file camera.h
 * @brief Camera class for view and projection management
 */

#ifndef ARE_INCLUDE_SCENE_CAMERA_H
#define ARE_INCLUDE_SCENE_CAMERA_H

#include <are/core/types.h>

namespace are {

/**
 * @class Camera
 * @brief Perspective camera for rendering
 * 
 * Manages view and projection matrices, and provides ray generation
 * for ray tracing.
 */
class Camera {
public:
    /**
     * @brief Default constructor
     */
    Camera();

    /**
     * @brief Construct camera with position and target
     * @param position Camera position
     * @param target Look-at target
     * @param up Up vector
     */
    Camera(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Position and orientation setters
    void set_position(const Vec3& position);
    void set_target(const Vec3& target);
    void set_up(const Vec3& up);
    void look_at(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Projection setters
    void set_fov(Real fov_degrees);
    void set_aspect_ratio(Real aspect);
    void set_near_plane(Real near);
    void set_far_plane(Real far);
    void set_perspective(Real fov_degrees, Real aspect, Real near, Real far);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_target() const { return target_; }
    const Vec3& get_up() const { return up_; }
    Real get_fov() const { return fov_; }
    Real get_aspect_ratio() const { return aspect_ratio_; }
    Real get_near_plane() const { return near_plane_; }
    Real get_far_plane() const { return far_plane_; }

    // Direction vectors
    Vec3 get_forward() const;
    Vec3 get_right() const;

    // Matrix getters
    const Mat4& get_view_matrix() const;
    const Mat4& get_projection_matrix() const;
    Mat4 get_view_projection_matrix() const;

    /**
     * @brief Generate ray for given pixel coordinates
     * @param u Normalized x coordinate [0, 1]
     * @param v Normalized y coordinate [0, 1]
     * @param origin Output ray origin
     * @param direction Output ray direction (normalized)
     */
    void generate_ray(Real u, Real v, Vec3& origin, Vec3& direction) const;

    /**
     * @brief Check if camera parameters have changed
     * @return true if matrices need recalculation
     */
    bool is_dirty() const { return dirty_; }

    /**
     * @brief Mark camera as clean after matrix update
     */
    void clear_dirty() { dirty_ = false; }

private:
    void update_view_matrix() const;
    void update_projection_matrix() const;

    Vec3 position_;                       ///< Camera position
    Vec3 target_;                         ///< Look-at target
    Vec3 up_;                             ///< Up vector

    Real fov_;                            ///< Field of view in degrees
    Real aspect_ratio_;                   ///< Aspect ratio (width/height)
    Real near_plane_;                     ///< Near clipping plane
    Real far_plane_;                      ///< Far clipping plane

    mutable Mat4 view_matrix_;            ///< Cached view matrix
    mutable Mat4 projection_matrix_;      ///< Cached projection matrix
    mutable bool view_dirty_;             ///< View matrix needs update
    mutable bool projection_dirty_;       ///< Projection matrix needs update
    bool dirty_;                          ///< Any parameter changed
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_CAMERA_H
```

### 文件：include/are/scene/material.h

```cpp
/**
 * @file material.h
 * @brief PBR material definition
 */

#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @class Material
 * @brief Physically-based rendering material
 * 
 * Supports standard PBR workflow with metallic-roughness model.
 */
class Material {
public:
    /**
     * @brief Default constructor - creates default white material
     */
    Material();

    // Albedo (base color)
    void set_albedo(const Vec3& albedo);
    void set_albedo_map(const std::string& path);
    const Vec3& get_albedo() const { return albedo_; }
    const std::string& get_albedo_map() const { return albedo_map_; }
    bool has_albedo_map() const { return !albedo_map_.empty(); }

    // Metallic
    void set_metallic(Real metallic);
    void set_metallic_map(const std::string& path);
    Real get_metallic() const { return metallic_; }
    const std::string& get_metallic_map() const { return metallic_map_; }
    bool has_metallic_map() const { return !metallic_map_.empty(); }

    // Roughness
    void set_roughness(Real roughness);
    void set_roughness_map(const std::string& path);
    Real get_roughness() const { return roughness_; }
    const std::string& get_roughness_map() const { return roughness_map_; }
    bool has_roughness_map() const { return !roughness_map_.empty(); }

    // Normal map
    void set_normal_map(const std::string& path);
    const std::string& get_normal_map() const { return normal_map_; }
    bool has_normal_map() const { return !normal_map_.empty(); }

    // Ambient occlusion
    void set_ao_map(const std::string& path);
    const std::string& get_ao_map() const { return ao_map_; }
    bool has_ao_map() const { return !ao_map_.empty(); }

    // Emissive
    void set_emissive(const Vec3& emissive);
    void set_emissive_map(const std::string& path);
    const Vec3& get_emissive() const { return emissive_; }
    const std::string& get_emissive_map() const { return emissive_map_; }
    bool has_emissive_map() const { return !emissive_map_.empty(); }
    bool is_emissive() const;

    // Texture handles (set by TextureManager)
    void set_albedo_texture_handle(TextureHandle handle) { albedo_tex_handle_ = handle; }
    void set_metallic_texture_handle(TextureHandle handle) { metallic_tex_handle_ = handle; }
    void set_roughness_texture_handle(TextureHandle handle) { roughness_tex_handle_ = handle; }
    void set_normal_texture_handle(TextureHandle handle) { normal_tex_handle_ = handle; }
    void set_ao_texture_handle(TextureHandle handle) { ao_tex_handle_ = handle; }
    void set_emissive_texture_handle(TextureHandle handle) { emissive_tex_handle_ = handle; }

    TextureHandle get_albedo_texture_handle() const { return albedo_tex_handle_; }
    TextureHandle get_metallic_texture_handle() const { return metallic_tex_handle_; }
    TextureHandle get_roughness_texture_handle() const { return roughness_tex_handle_; }
    TextureHandle get_normal_texture_handle() const { return normal_tex_handle_; }
    TextureHandle get_ao_texture_handle() const { return ao_tex_handle_; }
    TextureHandle get_emissive_texture_handle() const { return emissive_tex_handle_; }

private:
    // Base values
    Vec3 albedo_;///< Base color (RGB)
    Real metallic_;                       ///< Metallic factor [0, 1]
    Real roughness_;                      ///< Roughness factor [0, 1]
    Vec3 emissive_;                       ///< Emissive color (RGB)

    // Texture paths
    std::string albedo_map_;
    std::string metallic_map_;
    std::string roughness_map_;
    std::string normal_map_;
    std::string ao_map_;
    std::string emissive_map_;

    // Texture handles (GPU resources)
    TextureHandle albedo_tex_handle_;
    TextureHandle metallic_tex_handle_;
    TextureHandle roughness_tex_handle_;
    TextureHandle normal_tex_handle_;
    TextureHandle ao_tex_handle_;
    TextureHandle emissive_tex_handle_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MATERIAL_H
```

### 文件：include/are/scene/mesh.h

```cpp
/**
 * @file mesh.h
 * @brief Mesh class for geometry storage
 */

#ifndef ARE_INCLUDE_SCENE_MESH_H
#define ARE_INCLUDE_SCENE_MESH_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>
#include <vector>

namespace are {

/**
 * @class Mesh
 * @brief Triangle mesh container
 * 
 * Stores vertex and index data for a triangle mesh.
 * Supports automatic AABB computation.
 */
class Mesh {
public:
    /**
     * @brief Default constructor - creates empty mesh
     */
    Mesh();

    /**
     * @brief Construct mesh from vertex and index data
     * @param vertices Vertex array
     * @param indices Index array (triangles)
     * @param material_id Material handle
     */
    Mesh(const std::vector<Vertex>& vertices, 
         const std::vector<uint32_t>& indices,
         MaterialHandle material_id = are_invalid_handle);

    /**
     * @brief Construct mesh from raw arrays
     * @param vertices Vertex array pointer
     * @param vertex_count Number of vertices
     * @param indices Index array pointer
     * @param index_count Number of indices
     * @param material_id Material handle
     */
    Mesh(const Vertex* vertices, size_t vertex_count,
         const uint32_t* indices, size_t index_count,
         MaterialHandle material_id = are_invalid_handle);

    // Data setters
    void set_vertices(const std::vector<Vertex>& vertices);
    void set_indices(const std::vector<uint32_t>& indices);
    void set_material(MaterialHandle material_id);

    // Data getters
    const std::vector<Vertex>& get_vertices() const { return vertices_; }
    const std::vector<uint32_t>& get_indices() const { return indices_; }
    MaterialHandle get_material() const { return material_id_; }

    // Geometry queries
    size_t get_vertex_count() const { return vertices_.size(); }
    size_t get_index_count() const { return indices_.size(); }
    size_t get_triangle_count() const { return indices_.size() / 3; }
    bool is_empty() const { return vertices_.empty() || indices_.empty(); }

    // AABB
    const AABB& get_aabb() const { return aabb_; }
    void compute_aabb();

    /**
     * @brief Compute tangent vectors for normal mapping
     */
    void compute_tangents();/**
     * @brief Get triangle vertices by index
     * @param triangle_index Triangle index
     * @param v0 Output first vertex
     * @param v1 Output second vertex
     * @param v2 Output third vertex
     * @return true if triangle exists
     */
    bool get_triangle(size_t triangle_index, Vertex& v0, Vertex& v1, Vertex& v2) const;

    // GPU resource handles (set by Renderer)
    void set_vao(uint32_t vao) { vao_ = vao; }
    void set_vbo(uint32_t vbo) { vbo_ = vbo; }
    void set_ebo(uint32_t ebo) { ebo_ = ebo; }
    uint32_t get_vao() const { return vao_; }
    uint32_t get_vbo() const { return vbo_; }
    uint32_t get_ebo() const { return ebo_; }
    bool has_gpu_resources() const { return vao_ != 0; }

private:
    std::vector<Vertex> vertices_;        ///< Vertex data
    std::vector<uint32_t> indices_;       ///< Index data (triangles)
    MaterialHandle material_id_;          ///< Associated material
    AABB aabb_;                           ///< Bounding box

    // GPU resources
    uint32_t vao_;                         ///< Vertex Array Object
    uint32_t vbo_;                         ///< Vertex Buffer Object
    uint32_t ebo_;                         ///< Element Buffer Object
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MESH_H
```

### 文件：include/are/scene/light.h

```cpp
/**
 * @file light.h
 * @brief Base light class and common light utilities
 */

#ifndef ARE_INCLUDE_SCENE_LIGHT_H
#define ARE_INCLUDE_SCENE_LIGHT_H

#include <are/core/types.h>

namespace are {

/**
 * @enum LightType
 * @brief Types of light sources
 */
enum class LightType {
    ARE_LIGHT_DIRECTIONAL,
    ARE_LIGHT_POINT,
    ARE_LIGHT_SPOT
};

/**
 * @struct LightData
 * @brief Packed light data for GPU transfer
 * 
 * This structure is designed to be efficiently transferred to GPU
 * via SSBO or UBO.
 */
struct LightData {
    Vec4 position_type_;                  ///< xyz: position, w: light type
    Vec4 direction_range_;                ///< xyz: direction, w: range
    Vec4 color_intensity_;                ///< xyz: color, w: intensity
    Vec4 params_;                         ///< Light-specific parameters
};

/**
 * @class Light
 * @brief Base class for all light types
 */
class Light {
public:
    /**
     * @brief Constructor
     * @param type Light type
     */
    explicit Light(LightType type);

    virtual ~Light() = default;

    // Common properties
    void set_color(const Vec3& color);
    void set_intensity(Real intensity);
    void set_cast_shadows(bool cast);

    const Vec3& get_color() const { return color_; }
    Real get_intensity() const { return intensity_; }
    bool get_cast_shadows() const { return cast_shadows_; }
    LightType get_type() const { return type_; }

    /**
     * @brief Pack light data for GPU transfer
     * @return Packed light data
     */
    virtual LightData pack() const = 0;

    /**
     * @brief Check if light affects a point
     * @param point World position
     * @return true if light can affect the point
     */
    virtual bool affects_point(const Vec3& point) const = 0;

protected:
    LightType type_;                      ///< Light type
    Vec3 color_;                          ///< Light color (RGB)
    Real intensity_;                      ///< Light intensity
    bool cast_shadows_;                   ///< Whether light casts shadows
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_LIGHT_H
```

### 文件：include/are/scene/directional_light.h

```cpp
/**
 * @file directional_light.h
 * @brief Directional light implementation
 */

#ifndef ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H
#define ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class DirectionalLight
 * @brief Directional light source (sun-like)
 * 
 * Represents an infinitely distant light source with parallel rays.
 */
class DirectionalLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    DirectionalLight();

    /**
     * @brief Construct with direction and color
     * @param direction Light direction (will be normalized)
     * @param color Light color
     * @param intensity Light intensity
     */
    DirectionalLight(const Vec3& direction, const Vec3& color = Vec3(1.0f), 
                    Real intensity = 1.0f);

    // Direction
    void set_direction(const Vec3& direction);
    const Vec3& get_direction() const { return direction_; }

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 direction_;                      ///< Light direction (normalized)
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_DIRECTIONAL_LIGHT_H
```

### 文件：include/are/scene/point_light.h

```cpp
/**
 * @file point_light.h
 * @brief Point light implementation
 */

#ifndef ARE_INCLUDE_SCENE_POINT_LIGHT_H
#define ARE_INCLUDE_SCENE_POINT_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class PointLight
 * @brief Point light source
 * 
 * Emits light equally in all directions from a single point.
 */
class PointLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    PointLight();

    /**
     * @brief Construct with position and color
     * @param position Light position
     * @param color Light color
     * @param intensity Light intensity
     * @param range Light range (attenuation distance)
     */
    PointLight(const Vec3& position, const Vec3& color = Vec3(1.0f),
              Real intensity = 1.0f, Real range = 10.0f);

    // Position
    void set_position(const Vec3& position);
    const Vec3& get_position() const { return position_; }

    // Range (attenuation)
    void set_range(Real range);
    Real get_range() const { return range_; }

    // Attenuation parameters
    void set_attenuation(Real constant, Real linear, Real quadratic);
    Real get_constant_attenuation() const { return attenuation_constant_; }
    Real get_linear_attenuation() const { return attenuation_linear_; }
    Real get_quadratic_attenuation() const { return attenuation_quadratic_; }

    /**
     * @brief Calculate attenuation at given distance
     * @param distance Distance from light
     * @return Attenuation factor [0, 1]
     */
    Real calculate_attenuation(Real distance) const;

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 position_;                       ///< Light position
    Real range_;                          ///< Light range
    Real attenuation_constant_;           ///< Constant attenuation factor
    Real attenuation_linear_;             ///< Linear attenuation factor
    Real attenuation_quadratic_;          ///< Quadratic attenuation factor
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_POINT_LIGHT_H
```

### 文件：include/are/scene/spot_light.h

```cpp
/**
 * @file spot_light.h
 * @brief Spot light implementation
 */

#ifndef ARE_INCLUDE_SCENE_SPOT_LIGHT_H
#define ARE_INCLUDE_SCENE_SPOT_LIGHT_H

#include <are/scene/light.h>

namespace are {

/**
 * @class SpotLight
 * @brief Spot light source
 * 
 * Emits light in a cone from a single point.
 */
class SpotLight : public Light {
public:
    /**
     * @brief Default constructor
     */
    SpotLight();

    /**
     * @brief Construct with position, direction, and angles
     * @param position Light position
     * @param direction Light direction
     * @param inner_angle Inner cone angle in degrees
     * @param outer_angle Outer cone angle in degrees
     * @param color Light color
     * @param intensity Light intensity
     */
    SpotLight(const Vec3& position, const Vec3& direction,Real inner_angle, Real outer_angle,
             const Vec3& color = Vec3(1.0f), Real intensity = 1.0f);

    // Position and direction
    void set_position(const Vec3& position);
    void set_direction(const Vec3& direction);
    const Vec3& get_position() const { return position_; }
    const Vec3& get_direction() const { return direction_; }

    // Cone angles (in degrees)
    void set_inner_angle(Real angle);
    void set_outer_angle(Real angle);
    Real get_inner_angle() const { return inner_angle_; }
    Real get_outer_angle() const { return outer_angle_; }

    // Range
    void set_range(Real range);
    Real get_range() const { return range_; }

    /**
     * @brief Calculate spotlight intensity at given direction
     * @param to_point Direction from light to point (normalized)
     * @return Spotlight factor [0, 1]
     */
    Real calculate_spot_factor(const Vec3& to_point) const;

    // Light interface
    LightData pack() const override;
    bool affects_point(const Vec3& point) const override;

private:
    Vec3 position_;                       ///< Light position
    Vec3 direction_;                      ///< Light direction (normalized)
    Real inner_angle_;                    ///< Inner cone angle (degrees)
    Real outer_angle_;                    ///< Outer cone angle (degrees)
    Real range_;                          ///< Light range
    Real cos_inner_;                      ///< Cosine of inner angle (cache
	Real cos_outer_;                      ///< Cosine of outer angle (cached)
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SPOT_LIGHT_H
```

### 文件：include/are/scene/scene_manager.h

```cpp
/**
 * @file scene_manager.h
 * @brief Scene data management
 */

#ifndef ARE_INCLUDE_SCENE_SCENE_MANAGER_H
#define ARE_INCLUDE_SCENE_SCENE_MANAGER_H

#include <are/core/types.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <vector>
#include <memory>
#include <unordered_map>

namespace are {

/**
 * @class SceneManager
 * @brief Manages all scene objects (meshes, materials, lights)
 * 
 * Provides handle-based access to scene resources and tracks
 * scene modifications for BVH rebuilding.
 */
class SceneManager {
public:
    /**
     * @brief Constructor
     */
    SceneManager();

    /**
     * @brief Destructor
     */
    ~SceneManager();

    // Mesh management
    MeshHandle add_mesh(const Mesh& mesh);
    void remove_mesh(MeshHandle handle);
    void update_mesh(MeshHandle handle, const Mesh& mesh);
    Mesh* get_mesh(MeshHandle handle);
    const Mesh* get_mesh(MeshHandle handle) const;
    const std::vector<Mesh>& get_all_meshes() const { return meshes_; }

    // Material management
    MaterialHandle add_material(const Material& material);
    void remove_material(MaterialHandle handle);
    void update_material(MaterialHandle handle, const Material& material);
    Material* get_material(MaterialHandle handle);
    const Material* get_material(MaterialHandle handle) const;
    const std::vector<Material>& get_all_materials() const { return materials_; }

    // Light management
    LightHandle add_light(const std::shared_ptr<Light>& light);
    void remove_light(LightHandle handle);
    std::shared_ptr<Light> get_light(LightHandle handle);
    const std::vector<std::shared_ptr<Light>>& get_all_lights() const { return lights_; }

    // Scene queries
    size_t get_mesh_count() const { return meshes_.size(); }
    size_t get_material_count() const { return materials_.size(); }
    size_t get_light_count() const { return lights_.size(); }
    size_t get_total_triangle_count() const;

    // Scene state
    bool is_dirty() const { return dirty_; }
    void mark_dirty() { dirty_ = true; }
    void clear_dirty() { dirty_ = false; }

    /**
     * @brief Clear all scene data
     */
    void clear();

    /**
     * @brief Validate all handles and remove invalid entries
     */
    void compact();

private:
    std::vector<Mesh> meshes_;            ///< Mesh storage
    std::vector<Material> materials_;     ///< Material storage
    std::vector<std::shared_ptr<Light>> lights_; ///< Light storage

    std::unordered_map<MeshHandle, size_t> mesh_handle_map_;
    std::unordered_map<MaterialHandle, size_t> material_handle_map_;
    std::unordered_map<LightHandle, size_t> light_handle_map_;

    MeshHandle next_mesh_handle_;
    MaterialHandle next_material_handle_;
    LightHandle next_light_handle_;

    bool dirty_;                          ///< Scene modified flag
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SCENE_MANAGER_H
```

### 文件：include/are/renderer/renderer.h

```cpp
/**
 * @file renderer.h
 * @brief Main renderer interface
 */

#ifndef ARE_INCLUDE_RENDERER_RENDERER_H
#define ARE_INCLUDE_RENDERER_RENDERER_H

#include <are/core/config.h>
#include <are/core/types.h>
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <are/renderer/render_stats.h>
#include <memory>
#include <vector>

namespace are {

// Forward declarations
class Window;
class SceneManager;
class Rasterizer;
class RayTracer;
class TextureManager;

/**
 * @class Renderer
 * @brief Main rendering interface for Aurora Rendering Engine
 * 
 * This class provides the primary API for rendering scenes using
 * hybrid rasterization and ray tracing techniques.
 */
class Renderer {
public:
    /**
     * @brief Constructor
     * @param config Rendering configuration
     */
    explicit Renderer(const AreConfig& config);

    /**
     * @brief Destructor
     */
    ~Renderer();

    // Configuration
    void set_config(const AreConfig& config);
    const AreConfig& get_config() const;

    // Camera management
    void set_camera(const Camera& camera);
    Camera& get_camera();
    const Camera& get_camera() const;

    // Scene management
    MeshHandle add_mesh(const Mesh& mesh);
    MaterialHandle add_material(const Material& material);
    LightHandle add_light(const std::shared_ptr<Light>& light);

    void remove_mesh(MeshHandle handle);
    void remove_material(MaterialHandle handle);
    void remove_light(LightHandle handle);

    void update_mesh(MeshHandle handle, const Mesh& mesh);
    void update_material(MaterialHandle handle, const Material& material);

    void clear_scene();

    // Ray tracing backend control
    void set_ray_tracing_backend(RayTracingBackend backend);
    RayTracingBackend get_ray_tracing_backend() const;

    // Rendering
    void begin_frame();
    void render();
    void end_frame();
    void present();

    // Frame capture
    void capture_frame_ldr(uint8_t** pixels, int* width, int* height);
    void capture_frame_hdr(float** pixels, int* width, int* height);
    void save_frame(const std::string& filename, const uint8_t* pixels, 
                   int width, int height);

    // Window control
    bool should_close() const;
    void set_should_close(bool should_close);

    // Statistics
    const RenderStats& get_stats() const;
    void reset_stats();

    // Debug visualization
    void set_gbuffer_visualization_mode(GBufferVisualizationMode mode);
    GBufferVisualizationMode get_gbuffer_visualization_mode() const;

private:
    void initialize_subsystems();
    void shutdown_subsystems();
    void rebuild_bvh_if_needed();
    void check_scene_dirty();

    AreConfig config_;                    ///< Current configuration
    
    std::unique_ptr<Window> window_;      ///< Window management
    std::unique_ptr<SceneManager> scene_manager_; ///< Scene data management
    std::unique_ptr<Rasterizer> rasterizer_; ///< Rasterization pipeline
    std::unique_ptr<RayTracer> raytracer_; ///< Ray tracing pipeline
    std::unique_ptr<TextureManager> texture_manager_; ///< Texture management

    Camera camera_;                       ///< Active camera
    RenderStats stats_;                   ///< Rendering statistics
    
    bool scene_dirty_;                    ///< Scene needs BVH rebuild
    bool initialized_;                    ///< Initialization flag
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDERER_H
```

### 文件：include/are/renderer/render_stats.h

```cpp
/**
 * @file render_stats.h
 * @brief Rendering statistics tracking
 */

#ifndef ARE_INCLUDE_RENDERER_RENDER_STATS_H
#define ARE_INCLUDE_RENDERER_RENDER_STATS_H

#include <are/core/types.h>
#include <cstdint>

namespace are {

/**
 * @struct RenderStats
 * @brief Statistics for rendering performance analysis
 */
struct RenderStats {
    // Frame timing
    double frame_time_ms_;                ///< Total frame time in milliseconds
    double rasterization_time_ms_;        ///< Rasterization time
    double ray_tracing_time_ms_;          ///< Ray tracing time
    double bvh_build_time_ms_;            ///< BVH construction time
    double present_time_ms_;              ///< Present/swap time

    // Scene statistics
    uint32_t mesh_count_;                 ///< Number of meshes
    uint32_t triangle_count_;             ///< Total triangle count
    uint32_t light_count_;                ///< Number of lights
    uint32_t material_count_;             ///< Number of materials

    // Ray tracing statistics
    uint64_t primary_rays_;               ///< Number of primary rays
    uint64_t secondary_rays_;             ///< Number of secondary rays
    uint64_t shadow_rays_;                ///< Number of shadow rays
    uint64_t bvh_traversals_;             ///< BVH traversal count
    uint64_t triangle_tests_;             ///< Triangle intersection tests

    // Memory statistics
    size_t vertex_memory_bytes_;          ///< Vertex buffer memory
    size_t index_memory_bytes_;           ///< Index buffer memory
    size_t texture_memory_bytes_;         ///< Texture memory
    size_t bvh_memory_bytes_;             ///< BVH memory

    // FPS
    double fps_;                          ///< Frames per second

    /**
     * @brief Reset all statistics to zero
     */
    void reset();

    /**
     * @brief Print statistics to console
     */
    void print() const;

    /**
     * @brief Update FPS based on frame time
     */
    void update_fps();
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDER_STATS_H
```

### 文件：include/are/renderer/render_context.h

```cpp
/**
 * @file render_context.h
 * @brief Rendering context and state management
 */

#ifndef ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H
#define ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H

#include <are/core/types.h>
#include <are/core/config.h>

namespace are {

/**
 * @struct RenderContext
 * @brief Rendering context information
 * 
 * Contains current rendering state and frame information.
 */
struct RenderContext {
    int frame_number_;                    ///< Current frame number
    double time_;                         ///< Total elapsed time in seconds
    double delta_time_;                   ///< Time since last frame
    
    int viewport_width_;                  ///< Viewport width
    int viewport_height_;                 ///< Viewport height
    
    RayTracingBackend current_backend_;   ///< Current ray tracing backend
    bool scene_dirty_;                    ///< Scene needs BVH rebuild
    bool camera_moved_;                   ///< Camera moved this frame

    /**
     * @brief Constructor
     */
    RenderContext();

    /**
     * @brief Reset context
     */
    void reset();

    /**
     * @brief Update frame timing
     * @param current_time Current time in seconds
     */
    void update_timing(double current_time);
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H
```

### 文件：include/are/raytracer/ray.h

```cpp
/**
 * @file ray.h
 * @brief Ray structure for ray tracing
 */

#ifndef ARE_INCLUDE_RAYTRACER_RAY_H
#define ARE_INCLUDE_RAYTRACER_RAY_H

#include <are/core/types.h>

namespace are {

/**
 * @struct Ray
 * @brief Ray representation for ray tracing
 */
struct Ray {
    Vec3 origin_;                         ///< Ray origin
    Vec3 direction_;                      ///< Ray direction (normalized)
    Real t_min_;                          ///< Minimum t value
    Real t_max_;                          ///< Maximum t value

    /**
     * @brief Default constructor
     */
    Ray();

    /**
     * @brief Construct ray with origin and direction
     * @param origin Ray origin
     * @param direction Ray direction (will be normalized)
     * @param t_min Minimum t value
     * @param t_max Maximum t value
     */
    Ray(const Vec3& origin, const Vec3& direction, 
        Real t_min = are_epsilon, Real t_max = 1e30f);

    /**
     * @brief Evaluate ray at parameter t
     * @param t Parameter value
     * @return Point on ray
     */
    Vec3 at(Real t) const;

    /**
     * @brief Check if t is within valid range
     * @param t Parameter value
     * @return true if t is valid
     */
    bool is_valid_t(Real t) const;
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_RAY_H
```

### 文件：include/are/raytracer/hit_record.h

```cpp
/**
 * @file hit_record.h
 * @brief Ray-surface intersection record
 */

#ifndef ARE_INCLUDE_RAYTRACER_HIT_RECORD_H
#define ARE_INCLUDE_RAYTRACER_HIT_RECORD_H

#include <are/core/types.h>

namespace are {

/**
 * @struct HitRecord
 * @brief Information about ray-surface intersection
 */
struct HitRecord {
    Vec3 position_;                       ///< Hit position in world space
    Vec3 normal_;                         ///< Surface normal at hit point
    Vec2 texcoord_;                       ///< Texture coordinates at hit point
    Vec3 tangent_;                        ///< Tangent vector at hit point
    Real t_;                              ///< Ray parameter at hit point
    MaterialHandle material_;             ///< Material at hit point
    uint32_t triangle_index_;             ///< Triangle index that was hit
    bool front_face_;                     ///< Whether ray hit front face

    /**
     * @brief Default constructor
     */
    HitRecord();

    /**
     * @brief Set face normal based on ray direction
     * @param ray_direction Ray direction
     * @param outward_normal Outward-facing normal
     */
    void set_face_normal(const Vec3& ray_direction, const Vec3& outward_normal);

    /**
     * @brief Check if hit record is valid
     * @return true if hit occurred
     */
    bool is_valid() const;
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_HIT_RECORD_H
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

### 文件：include/are/raytracer/compute_raytracer.h

```cpp
/**
 * @file compute_raytracer.h
 * @brief GPU compute shader ray tracing implementation
 */

#ifndef ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H
#define ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H

#include <are/raytracer/raytracer.h>
#include <memory>

namespace are {

// Forward declarations
class ShaderProgram;

/**
 * @class ComputeRayTracer
 * @brief GPU-based ray tracing using compute shaders
 */
class ComputeRayTracer : public RayTracer {
public:
    /**
     * @brief Constructor
     * @param config Ray tracing configuration
     */
    explicit ComputeRayTracer(const RayTracingConfig& config);

    /**
     * @brief Destructor
     */
    ~ComputeRayTracer() override;

    /**
     * @brief Render scene using compute shader ray tracing
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
    void initialize_compute_shader(const std::string& shader_dir);
    void upload_scene_data(const SceneManager& scene);
    void upload_bvh_data(const BVH& bvh);
    void upload_camera_data(const Camera& camera);

    std::unique_ptr<ShaderProgram> compute_shader_; ///< Ray tracing compute shader

    // GPU buffers (SSBOs)
    uint32_t bvh_buffer_;                 ///< BVH nodes buffer
    uint32_t triangle_buffer_;            ///< Triangle data buffer
    uint32_t material_buffer_;            ///< Material data buffer
    uint32_t light_buffer_;               ///< Light data buffer

    bool buffers_initialized_;            ///< Buffer initialization flag
};

} // namespace are

#endif // ARE_INCLUDE_RAYTRACER_COMPUTE_RAYTRACER_H
```

### 文件：include/are/utils/image_io.h

```cpp
/**
 * @file image_io.h
 * @brief Image loading and saving utilities
 */

#ifndef ARE_INCLUDE_UTILS_IMAGE_IO_H
#define ARE_INCLUDE_UTILS_IMAGE_IO_H

#include <are/core/types.h>
#include <string>
#include <vector>

namespace are {

/**
 * @enum ImageFormat
 * @brief Supported image formats
 */
enum class ImageFormat {
    ARE_IMAGE_FORMAT_PPM,
    ARE_IMAGE_FORMAT_BMP,
    ARE_IMAGE_FORMAT_PNG,
    ARE_IMAGE_FORMAT_JPG
};

/**
 * @struct ImageData
 * @brief Container for image data
 */
struct ImageData {
    int width_;                           ///< Image width
    int height_;                          ///< Image height
    int channels_;                        ///< Number of channels (3=RGB, 4=RGBA)
    std::vector<uint8_t> data_;           ///< Pixel data (row-major)

    /**
     * @brief Check if image data is valid
     * @return true if valid
     */
    bool is_valid() const;

    /**
     * @brief Get pixel at (x, y)
     * @param x X coordinate
     * @param y Y coordinate
     * @return Pointer to pixel data (RGB or RGBA)
     */
    const uint8_t* get_pixel(int x, int y) const;

    /**
     * @brief Set pixel at (x, y)
     * @param x X coordinate
     * @param y Y coordinate
     * @param r Red channel
     * @param g Green channel
     * @param b Blue channel
     * @param a Alpha channel (optional)
     */
    void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
};

/**
 * @brief Load image from file
 * @param filename Image file path
 * @param flip_vertically Whether to flip image vertically
 * @return Image data (empty if failed)
 */
ImageData load_image(const std::string& filename, bool flip_vertically = false);

/**
 * @brief Save image to file
 * @param filename Output file path
 * @param data Image data
 * @param format Output format (auto-detected from extension if not specified)
 * @return true if save succeeded
 */
bool save_image(const std::string& filename, const ImageData& data, 
               ImageFormat format = ImageFormat::ARE_IMAGE_FORMAT_PNG);

/**
 * @brief Save raw pixel data to file
 * @param filename Output file path
 * @param pixels Pixel data pointer
 * @param width Image width
 * @param height Image height
 * @param channels Number of channels
 * @param format Output format
 * @return true if save succeeded
 */
bool save_image(const std::string& filename, const uint8_t* pixels,
               int width, int height, int channels,
               ImageFormat format = ImageFormat::ARE_IMAGE_FORMAT_PNG);

/**
 * @brief Detect image format from file extension
 * @param filename File path
 * @return Detected format
 */
ImageFormat detect_format(const std::string& filename);

} // namespace are

#endif // ARE_INCLUDE_UTILS_IMAGE_IO_H
```

### 文件：include/are/utils/math_utils.h

```cpp
/**
 * @file math_utils.h
 * @brief Mathematical utility functions
 */

#ifndef ARE_INCLUDE_UTILS_MATH_UTILS_H
#define ARE_INCLUDE_UTILS_MATH_UTILS_H

#include <are/core/types.h>
#include <algorithm>

namespace are {

/**
 * @brief Clamp value to range [min, max]
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
template<typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor [0, 1]
 * @return Interpolated value
 */
template<typename T>
inline T lerp(T a, T b, Real t) {
    return a + (b - a) * t;
}

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline Real degrees_to_radians(Real degrees) {
    return degrees * are_pi / 180.0f;
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline Real radians_to_degrees(Real radians) {
    return radians * 180.0f / are_pi;
}

/**
 * @brief Check if two floating point values are approximately equal
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance
 * @return true if approximately equal
 */
inline bool approx_equal(Real a, Real b, Real epsilon = are_epsilon) {
    return std::abs(a - b) < epsilon;
}

/**
 * @brief Compute barycentric coordinates
 * @param p Point
 * @param a Triangle vertex A
 * @param b Triangle vertex B
 * @param c Triangle vertex C
 * @param u Output barycentric coordinate u
 * @param v Output barycentric coordinate v
 * @param w Output barycentric coordinate w
 */
void compute_barycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                        Real& u, Real& v, Real& w);

/**
 * @brief Reflect vector around normal
 * @param incident Incident vector
 * @param normal Surface normal (normalized)
 * @return Reflected vector
 */
Vec3 reflect(const Vec3& incident, const Vec3& normal);

/**
 * @brief Refract vector through surface
 * @param incident Incident vector (normalized)
 * @param normal Surface normal (normalized)
 * @param eta Ratio of refractive indices
 * @param refracted Output refracted vector
 * @return true if refraction occurred (false for total internal reflection)
 */
bool refract(const Vec3& incident, const Vec3& normal, Real eta, Vec3& refracted);

/**
 * @brief Compute Fresnel reflectance (Schlick approximation)
 * @param cos_theta Cosine of angle between view and normal
 * @param f0 Reflectance at normal incidence
 * @return Fresnel reflectance
 */
Real fresnel_schlick(Real cos_theta, Real f0);

/**
 * @brief Create orthonormal basis from normal
 * @param normal Normal vector (normalized)
 * @param tangent Output tangent vector
 * @param bitangent Output bitangent vector
 */
void create_orthonormal_basis(const Vec3& normal, Vec3& tangent, Vec3& bitangent);

/**
 * @brief Transform direction from tangent space to world space
 * @param tangent_dir Direction in tangent space
 * @param normal Surface normal
 * @param tangent Surface tangent
 * @param bitangent Surface bitangent
 * @return Direction in world space
 */
Vec3 tangent_to_world(const Vec3& tangent_dir, const Vec3& normal, 
                     const Vec3& tangent, const Vec3& bitangent);

} // namespace are

#endif // ARE_INCLUDE_UTILS_MATH_UTILS_H
```

### 文件：include/are/utils/random.h

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

### 文件：include/are/utils/file_utils.h

```cpp
/**
 * @file file_utils.h
 * @brief File system utilities
 */

#ifndef ARE_INCLUDE_UTILS_FILE_UTILS_H
#define ARE_INCLUDE_UTILS_FILE_UTILS_H

#include <string>
#include <vector>
#include <cstdint>

namespace are {

/**
 * @brief Read entire file into string
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::string read_file_to_string(const std::string& filepath);

/**
 * @brief Read entire file into byte array
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::vector<uint8_t> read_file_to_bytes(const std::string& filepath);

/**
 * @brief Write string to file
 * @param filepath File path
 * @param content Content to write
 * @return true if write succeeded
 */
bool write_string_to_file(const std::string& filepath, const std::string& content);

/**
 * @brief Write bytes to file
 * @param filepath File path
 * @param data Data pointer
 * @param size Data size in bytes
 * @return true if write succeeded
 */
bool write_bytes_to_file(const std::string& filepath, const void* data, size_t size);

/**
 * @brief Check if file exists
 * @param filepath File path
 * @return true if file exists
 */
bool file_exists(const std::string& filepath);

/**
 * @brief Check if path is directory
 * @param path Directory path
 * @return true if directory exists
 */
bool is_directory(const std::string& path);

/**
 * @brief Create directory (including parent directories)
 * @param path Directory path
 * @return true if creation succeeded
 */
bool create_directory(const std::string& path);

/**
 * @brief Get file extension
 * @param filepath File path
 * @return Extension (lowercase, without dot)
 */
std::string get_file_extension(const std::string& filepath);

/**
 * @brief Get filename from path
 * @param filepath File path
 * @return Filename (without directory)
 */
std::string get_filename(const std::string& filepath);

/**
 * @brief Get directory from path
 * @param filepath File path
 * @return Directory path
 */
std::string get_directory(const std::string& filepath);

/**
 * @brief Join path components
 * @param parts Path components
 * @return Joined path
 */
std::string join_path(const std::vector<std::string>& parts);

/**
 * @brief Normalize path (resolve .. and .)
 * @param path Path to normalize
 * @return Normalized path
 */
std::string normalize_path(const std::string& path);

} // namespace are

#endif // ARE_INCLUDE_UTILS_FILE_UTILS_H
```

### 文件：include/are/platform/window.h

```cpp
/**
 * @file window.h
 * @brief Window management using GLFW
 */

#ifndef ARE_INCLUDE_PLATFORM_WINDOW_H
#define ARE_INCLUDE_PLATFORM_WINDOW_H

#include <are/core/config.h>
#include <are/core/types.h>
#include <string>

// Forward declare GLFW types to avoid including GLFW in header
struct GLFWwindow;

namespace are {

/**
 * @class Window
 * @brief GLFW window wrapper
 * 
 * Manages window creation, input handling, and OpenGL context.
 */
class Window {
public:
    /**
     * @brief Constructor
     * @param config Window configuration
     */
    explicit Window(const WindowConfig& config);

    /**
     * @brief Destructor
     */
    ~Window();

    // Window control
    bool should_close() const;
    void set_should_close(bool should_close);
    void swap_buffers();
    void poll_events();

    // Window properties
    int get_width() const;
    int get_height() const;
    Real get_aspect_ratio() const;
    const std::string& get_title() const;

    void set_title(const std::string& title);
    void set_size(int width, int height);

    // Framebuffer size (may differ from window size on high-DPI displays)
    void get_framebuffer_size(int& width, int& height) const;

    // VSync control
    void set_vsync(bool enabled);
    bool get_vsync() const;

    // Input queries (basic support)
    bool is_key_pressed(int key) const;
    bool is_mouse_button_pressed(int button) const;
    void get_cursor_pos(double& x, double& y) const;

    // Internal
    GLFWwindow* get_native_window() const { return window_; }

private:
    void initialize_glfw();
    void create_window();
    void setup_callbacks();

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void error_callback(int error, const char* description);

    GLFWwindow* window_;                  ///< GLFW window handle
    WindowConfig config_;                 ///< Window configuration
    bool vsync_enabled_;                  ///< VSync state

    static int instance_count_;           ///< Number of Window instances
};

} // namespace are

#endif // ARE_INCLUDE_PLATFORM_WINDOW_H
```

### 文件：include/are/platform/gl_context.h

```cpp
/**
 * @file gl_context.h
 * @brief OpenGL context management
 */

#ifndef ARE_INCLUDE_PLATFORM_GL_CONTEXT_H
#define ARE_INCLUDE_PLATFORM_GL_CONTEXT_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @class GLContext
 * @brief OpenGL context initialization and management
 * 
 * Handles GLAD initialization and provides OpenGL utility functions.
 */
class GLContext {
public:
    /**
     * @brief Initialize OpenGL context (load function pointers)
     * @return true if initialization succeeded
     */
    static bool initialize();

    /**
     * @brief Check if context is initialized
     * @return true if initialized
     */
    static bool is_initialized();

    /**
     * @brief Get OpenGL version string
     * @return Version string
     */
    static std::string get_version();

    /**
     * @brief Get OpenGL renderer string
     * @return Renderer string
     */
    static std::string get_renderer();

    /**
     * @brief Get OpenGL vendor string
     * @return Vendor string
     */
    static std::string get_vendor();

    /**
     * @brief Check if OpenGL extension is supported
     * @param extension Extension name
     * @return true if supported
     */
    static bool is_extension_supported(const std::string& extension);

    /**
     * @brief Print OpenGL information to console
     */
    static void print_info();

    /**
     * @brief Check for OpenGL errors
     * @param file Source file
     * @param line Line number
     * @return true if error occurred
     */
    static bool check_error(const char* file, int line);

    /**
     * @brief Clear all OpenGL errors
     */
    static void clear_errors();

private:
    static bool initialized_;             ///< Initialization flag
};

} // namespace are

// OpenGL error checking macro
#ifdef ARE_ENABLE_DEBUG_VIS
    #define ARE_GL_CHECK() are::GLContext::check_error(__FILE__, __LINE__)
#else
    #define ARE_GL_CHECK() ((void)0)
#endif

#endif // ARE_INCLUDE_PLATFORM_GL_CONTEXT_H
```

### 文件：include/are/acceleration/bvh_node.h

```cpp
/**
 * @file bvh_node.h
 * @brief BVH node structure
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_NODE_H
#define ARE_INCLUDE_ACCELERATION_BVH_NODE_H

#include <are/core/types.h>
#include <are/geometry/aabb.h>

namespace are {

/**
 * @struct BVHNode
 * @brief Node in Bounding Volume Hierarchy
 * 
 * Uses a compact representation for efficient GPU transfer.
 */
struct BVHNode {
    AABB bounds_;                         ///< Node bounding box
    
    union {
        uint32_t left_child_;             ///< Left child index (internal node)
        uint32_t first_primitive_;        ///< First primitive index (leaf node)
    };
    
    union {
        uint32_t right_child_;            ///< Right child index (internal node)
        uint32_t primitive_count_;        ///< Number of primitives (leaf node)
    };

    /**
     * @brief Check if node is a leaf
     * @return true if leaf node
     */
    bool is_leaf() const {
        return primitive_count_ > 0;
    }

    /**
     * @brief Get node surface area (for SAH)
     * @return Surface area
     */
    Real surface_area() const {
        return bounds_.surface_area();
    }
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_NODE_H
```

### 文件：include/are/acceleration/bvh_builder.h

```cpp
/**
 * @file bvh_builder.h
 * @brief BVH construction algorithms
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H
#define ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H

#include <are/core/types.h>
#include <are/acceleration/bvh_node.h>
#include <are/geometry/triangle.h>
#include <vector>

namespace are {

/**
 * @enum BVHSplitMethod
 * @brief BVH splitting strategies
 */
enum class BVHSplitMethod {
    ARE_BVH_SPLIT_MIDDLE,                 ///< Split at midpoint
    ARE_BVH_SPLIT_SAH                     ///< Surface Area Heuristic
};

/**
 * @struct BVHBuildConfig
 * @brief Configuration for BVH construction
 */
struct BVHBuildConfig {
    BVHSplitMethod split_method_ = BVHSplitMethod::ARE_BVH_SPLIT_SAH;
    int max_leaf_size_ = 4;               ///< Maximum triangles per leaf
    int max_depth_ = 64;                  ///< Maximum tree depth
    bool use_multithreading_ = true;      ///< Use parallel construction
};

/**
 * @class BVHBuilder
 * @brief Constructs BVH from triangle list
 */
class BVHBuilder {
public:
    /**
     * @brief Constructor
     * @param config Build configuration
     */
    explicit BVHBuilder(const BVHBuildConfig& config = BVHBuildConfig());

    /**
     * @brief Build BVH from triangles
     * @param triangles Triangle list
     * @param nodes Output node list
     * @param primitive_indices Output primitive index list
     * @return Root node index
     */
    uint32_t build(const std::vector<Triangle>& triangles,
                  std::vector<BVHNode>& nodes,
                  std::vector<uint32_t>& primitive_indices);

    /**
     * @brief Get build statistics
     * @param node_count Output node count
     * @param leaf_count Output leaf count
     * @param max_depth Output maximum depth reached
     */
    void get_stats(size_t& node_count, size_t& leaf_count, int& max_depth) const;

private:
    struct BuildEntry {
        uint32_t parent_;
        uint32_t start_;
        uint32_t end_;
        int depth_;
    };

    uint32_t build_recursive(const std::vector<Triangle>& triangles,
                            std::vector<BVHNode>& nodes,
                            std::vector<uint32_t>& primitive_indices,
                            uint32_t start, uint32_t end, int depth);

    int find_best_split_axis(const std::vector<Triangle>& triangles,
                            const std::vector<uint32_t>& indices,
                            uint32_t start, uint32_t end);

    Real compute_sah_cost(const AABB& bounds, uint32_t count);

    BVHBuildConfig config_;
    size_t node_count_;
    size_t leaf_count_;
    int max_depth_reached_;
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H
```

### 文件：include/are/acceleration/bvh.h

```cpp
/**
 * @file bvh.h
 * @brief BVH interface and traversal
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_H
#define ARE_INCLUDE_ACCELERATION_BVH_H

#include <are/core/types.h>
#include <are/acceleration/bvh_node.h>
#include <are/acceleration/bvh_builder.h>
#include <are/geometry/triangle.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>
#include <vector>

namespace are {

/**
 * @class BVH
 * @brief Bounding Volume Hierarchy for ray tracing acceleration
 */
class BVH {
public:
    /**
     * @brief Constructor
     */
    BVH();

    /**
     * @brief Destructor
     */
    ~BVH();

    /**
     * @brief Build BVH from triangle list
     * @param triangles Triangle list
     * @param config Build configuration
     * @return true if build succeeded
     */
    bool build(const std::vector<Triangle>& triangles,
              const BVHBuildConfig& config = BVHBuildConfig());

    /**
     * @brief Traverse BVH and find closest intersection
     * @param ray Ray to trace
     * @param hit Output hit record
     * @return true if intersection found
     */
    bool intersect(const Ray& ray, HitRecord& hit) const;

    /**
     * @brief Fast occlusion test (any hit)
     * @param ray Ray to trace
     * @param t_max Maximum t value
     * @return true if any intersection found
     */
    bool intersect_any(const Ray& ray, Real t_max) const;

    /**
     * @brief Check if BVH is built
     * @return true if built
     */
    bool is_built() const { return !nodes_.empty(); }

    /**
     * @brief Get BVH nodes (for GPU upload)
     * @return Node array
     */
    const std::vector<BVHNode>& get_nodes() const { return nodes_; }

    /**
     * @brief Get primitive indices
     * @return Index array
     */
    const std::vector<uint32_t>& get_primitive_indices() const { 
        return primitive_indices_; 
    }

    /**
     * @brief Get triangles
     * @return Triangle array
     */
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

    /**
     * @brief Get memory usage in bytes
     * @return Memory usage
     */
    size_t get_memory_usage() const;

    /**
     * @brief Clear BVH data
     */
    void clear();

private:
    // Recursive traversal (kept for reference)
    bool intersect_recursive(uint32_t node_index, const Ray& ray, HitRecord& hit) const;
    bool intersect_any_recursive(uint32_t node_index, const Ray& ray, Real t_max) const;
    
    // Optimized iterative traversal
    bool intersect_iterative(const Ray& ray, HitRecord& hit) const;
    bool intersect_any_iterative(const Ray& ray, Real t_max) const;
    
    // Fast intersection helpers
    inline bool intersect_aabb_fast(const AABB& bounds, const Ray& ray,
                                    const Vec3& inv_dir, Real t_max,
                                    Real& t_min_out, Real& t_max_out) const;
    inline bool intersect_triangle_fast(const Triangle& triangle, const Ray& ray,
                                       Real t_max, HitRecord& hit) const;

    std::vector<BVHNode> nodes_;          ///< BVH nodes
    std::vector<uint32_t> primitive_indices_; ///< Primitive index array
    std::vector<Triangle> triangles_;     ///< Triangle data
    uint32_t root_index_;                 ///< Root node index
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_H
```

### 文件：include/are/rasterizer/rasterizer.h

```cpp
/**
 * @file rasterizer.h
 * @brief Rasterization pipeline for G-Buffer generation
 */

#ifndef ARE_INCLUDE_RASTERIZER_RASTERIZER_H
#define ARE_INCLUDE_RASTERIZER_RASTERIZER_H

#include <are/core/types.h>
#include <are/core/config.h>
#include <memory>

namespace are {

// Forward declarations
class GBuffer;
class ShaderProgram;
class SceneManager;
class Camera;
class Mesh;

/**
 * @class Rasterizer
 * @brief OpenGL rasterization pipeline
 * 
 * Renders scene geometry to G-Buffer using traditional rasterization.
 */
class Rasterizer {
public:
    /**
     * @brief Constructor
     * @param width Framebuffer width
     * @param height Framebuffer height
     */
    Rasterizer(int width, int height);

    /**
     * @brief Destructor
     */
    ~Rasterizer();

    /**
     * @brief Resize framebuffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Render scene to G-Buffer
     * @param scene Scene manager
     * @param camera Camera
     */
    void render_gbuffer(const SceneManager& scene, const Camera& camera);

    /**
     * @brief Get G-Buffer
     * @return G-Buffer reference
     */
    GBuffer& get_gbuffer();
    const GBuffer& get_gbuffer() const;

    /**
     * @brief Upload mesh data to GPU
     * @param mesh Mesh to upload
     */
    void upload_mesh(Mesh& mesh);

    /**
     * @brief Delete mesh GPU resources
     * @param mesh Mesh to delete
     */
    void delete_mesh(Mesh& mesh);

private:
    void initialize_shaders(const std::string& shader_dir);
    void setup_mesh_buffers(Mesh& mesh);

    std::unique_ptr<GBuffer> gbuffer_;    ///< G-Buffer
    std::unique_ptr<ShaderProgram> gbuffer_shader_; ///< G-Buffer shader
    
    int width_;                           ///< Framebuffer width
    int height_;                          ///< Framebuffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_RASTERIZER_H
```

### 文件：include/are/rasterizer/shader_program.h

```cpp
/**
 * @file shader_program.h
 * @brief OpenGL shader program wrapper
 */

#ifndef ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
#define ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H

#include <are/core/types.h>
#include <string>
#include <unordered_map>

namespace are {

/**
 * @enum ShaderType
 * @brief Shader stage types
 */
enum class ShaderType {
    ARE_SHADER_VERTEX,
    ARE_SHADER_FRAGMENT,
    ARE_SHADER_COMPUTE
};

/**
 * @class ShaderProgram
 * @brief OpenGL shader program management
 */
class ShaderProgram {
public:
    /**
     * @brief Constructor
     */
    ShaderProgram();

    /**
     * @brief Destructor
     */
    ~ShaderProgram();

    /**
     * @brief Load and compile shader from file
     * @param type Shader type
     * @param filepath Shader file path
     * @return true if compilation succeeded
     */
    bool load_shader(ShaderType type, const std::string& filepath);

    /**
     * @brief Compile shader from source string
     * @param type Shader type
     * @param source Shader source code
     * @return true if compilation succeeded
     */
    bool compile_shader(ShaderType type, const std::string& source);

    /**
     * @brief Link shader program
     * @return true if linking succeeded
     */
    bool link();

    /**
     * @brief Use this shader program
     */
    void use() const;

    /**
     * @brief Check if program is valid
     * @return true if valid
     */
    bool is_valid() const { return program_ != 0 && linked_; }

    /**
     * @brief Get OpenGL program ID
     * @return Program ID
     */
    uint32_t get_program() const { return program_; }

    // Uniform setters
    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, const Vec2& value);
    void set_uniform(const std::string& name, const Vec3& value);
    void set_uniform(const std::string& name, const Vec4& value);
    void set_uniform(const std::string& name, const Mat3& value);
    void set_uniform(const std::string& name, const Mat4& value);

    /**
     * @brief Get uniform location (cached)
     * @param name Uniform name
     * @return Uniform location (-1 if not found)
     */
    int get_uniform_location(const std::string& name);

private:
    bool check_compile_errors(uint32_t shader, ShaderType type);
    bool check_link_errors();

    uint32_t program_;                    ///< OpenGL program ID
    uint32_t vertex_shader_;              ///< Vertex shader ID
    uint32_t fragment_shader_;            ///< Fragment shader ID
    uint32_t compute_shader_;             ///< Compute shader ID
    
    bool linked_;                         ///< Link status
    std::unordered_map<std::string, int> uniform_cache_; ///< Uniform location cache
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
```

### 文件：include/are/rasterizer/gbuffer.h

```cpp
/**
 * @file gbuffer.h
 * @brief G-Buffer management for deferred rendering
 */

#ifndef ARE_INCLUDE_RASTERIZER_GBUFFER_H
#define ARE_INCLUDE_RASTERIZER_GBUFFER_H

#include <are/core/types.h>
#include <cstdint>

namespace are {

/**
 * @class GBuffer
 * @brief G-Buffer for deferred rendering
 * 
 * Contains multiple render targets for position, normal, albedo, etc.
 */
class GBuffer {
public:
    /**
     * @brief Constructor
     * @param width Buffer width
     * @param height Buffer height
     */
    GBuffer(int width, int height);

    /**
     * @brief Destructor
     */
    ~GBuffer();

    /**
     * @brief Resize G-Buffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Bind G-Buffer for rendering
     */
    void bind();

    /**
     * @brief Unbind G-Buffer
     */
    void unbind();

    /**
     * @brief Clear all buffers
     */
    void clear();

    /**
     * @brief Bind texture for reading
     * @param index Texture index (0=position, 1=normal, 2=albedo, etc.)
     * @param texture_unit Texture unit to bind to
     */
    void bind_texture(int index, int texture_unit);

    // Texture getters
    uint32_t get_position_texture() const { return position_texture_; }
    uint32_t get_normal_texture() const { return normal_texture_; }
    uint32_t get_albedo_texture() const { return albedo_texture_; }
    uint32_t get_material_texture() const { return material_texture_; }
    uint32_t get_depth_texture() const { return depth_texture_; }

    // Dimensions
    int get_width() const { return width_; }
    int get_height() const { return height_; }

    /**
     * @brief Read pixel data from G-Buffer
     * @param index Buffer index
     * @param data Output data pointer
     */
    void read_pixels(int index, void* data);

private:
    void create_textures();
    void delete_textures();
    void create_framebuffer();

    uint32_t fbo_;                        ///< Framebuffer object
    uint32_t rbo_depth_;                  ///< Depth renderbuffer

    // G-Buffer textures
    uint32_t position_texture_;           ///< World position (RGB16F)
    uint32_t normal_texture_;             ///< World normal (RGB16F)
    uint32_t albedo_texture_;             ///< Albedo + Metallic (RGBA8)
    uint32_t material_texture_;           ///< Roughness + AO (RG8)
    uint32_t depth_texture_;              ///< Depth (R32F)

    int width_;                           ///< Buffer width
    int height_;                          ///< Buffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_GBUFFER_H
```

### 文件：include/are/texture/texture.h

```cpp
/**
 * @file texture.h
 * @brief Texture resource management
 */

#ifndef ARE_INCLUDE_TEXTURE_TEXTURE_H
#define ARE_INCLUDE_TEXTURE_TEXTURE_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @enum TextureFormat
 * @brief Texture internal formats
 */
enum class TextureFormat {
    ARE_TEXTURE_R8,
    ARE_TEXTURE_RG8,
    ARE_TEXTURE_RGB8,
    ARE_TEXTURE_RGBA8,
    ARE_TEXTURE_R16F,
    ARE_TEXTURE_RG16F,
    ARE_TEXTURE_RGB16F,
    ARE_TEXTURE_RGBA16F,
    ARE_TEXTURE_R32F,
    ARE_TEXTURE_RG32F,
    ARE_TEXTURE_RGB32F,
    ARE_TEXTURE_RGBA32F
};

/**
 * @enum TextureFilter
 * @brief Texture filtering modes
 */
enum class TextureFilter {
    ARE_TEXTURE_FILTER_NEAREST,
    ARE_TEXTURE_FILTER_LINEAR,
    ARE_TEXTURE_FILTER_NEAREST_MIPMAP_NEAREST,
    ARE_TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST,
    ARE_TEXTURE_FILTER_NEAREST_MIPMAP_LINEAR,
    ARE_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR
};

/**
 * @enum TextureWrap
 * @brief Texture wrapping modes
 */
enum class TextureWrap {
    ARE_TEXTURE_WRAP_REPEAT,
    ARE_TEXTURE_WRAP_CLAMP_TO_EDGE,
    ARE_TEXTURE_WRAP_CLAMP_TO_BORDER,
    ARE_TEXTURE_WRAP_MIRRORED_REPEAT
};

/**
 * @class Texture
 * @brief OpenGL texture wrapper
 */
class Texture {
public:
    /**
     * @brief Constructor
     */
    Texture();

    /**
     * @brief Destructor
     */
    ~Texture();

    /**
     * @brief Load texture from file
     * @param filepath Image file path
     * @param format Desired texture format
     * @param generate_mipmaps Generate mipmaps
     * @return true if load succeeded
     */
    bool load_from_file(const std::string& filepath, 
                       TextureFormat format = TextureFormat::ARE_TEXTURE_RGBA8,
                       bool generate_mipmaps = true);

    /**
     * @brief Create texture from raw data
     * @param width Texture width
     * @param height Texture height
     * @param format Texture format
     * @param data Pixel data
     * @param generate_mipmaps Generate mipmaps
     * @return true if creation succeeded
     */
	bool create_from_data(int width, int height, 
                         TextureFormat format,
                         const void* data,
                         bool generate_mipmaps = true);

    /**
     * @brief Bind texture to texture unit
     * @param unit Texture unit (0-31)
     */
    void bind(int unit = 0) const;

    /**
     * @brief Unbind texture
     */
    void unbind() const;

    /**
     * @brief Set texture filtering
     * @param min_filter Minification filter
     * @param mag_filter Magnification filter
     */
    void set_filter(TextureFilter min_filter, TextureFilter mag_filter);

    /**
     * @brief Set texture wrapping
     * @param wrap_s S-axis wrapping
     * @param wrap_t T-axis wrapping
     */
    void set_wrap(TextureWrap wrap_s, TextureWrap wrap_t);

    /**
     * @brief Generate mipmaps
     */
    void generate_mipmaps();

    // Getters
    uint32_t get_id() const { return texture_id_; }
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    TextureFormat get_format() const { return format_; }
    bool is_valid() const { return texture_id_ != 0; }

    /**
     * @brief Delete texture
     */
    void destroy();

private:
    uint32_t texture_id_;                 ///< OpenGL texture ID
    int width_;                           ///< Texture width
    int height_;                          ///< Texture height
    TextureFormat format_;                ///< Texture format
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_TEXTURE_H
```

### 文件：include/are/texture/texture_manager.h

```cpp
/**
 * @file texture_manager.h
 * @brief Texture resource management and caching
 */

#ifndef ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H
#define ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H

#include <are/core/types.h>
#include <are/texture/texture.h>
#include <string>
#include <unordered_map>
#include <memory>

namespace are {

/**
 * @class TextureManager
 * @brief Manages texture loading and caching
 * 
 * Automatically handles texture deduplication and lifetime management.
 */
class TextureManager {
public:
    /**
     * @brief Constructor
     */
    TextureManager();

    /**
     * @brief Destructor
     */
    ~TextureManager();

    /**
     * @brief Load texture from file (with caching)
     * @param filepath Texture file path
     * @param format Desired texture format
     * @param generate_mipmaps Generate mipmaps
     * @return Texture handle (are_invalid_handle if failed)
     */
    TextureHandle load_texture(const std::string& filepath,
                              TextureFormat format = TextureFormat::ARE_TEXTURE_RGBA8,
                              bool generate_mipmaps = true);

    /**
     * @brief Create texture from raw data
     * @param name Texture name (for caching)
     * @param width Texture width
     * @param height Texture height
     * @param format Texture format
     * @param data Pixel data
     * @param generate_mipmaps Generate mipmaps
     * @return Texture handle
     */
    TextureHandle create_texture(const std::string& name,
                                int width, int height,
                                TextureFormat format,
                                const void* data,
                                bool generate_mipmaps = true);

    /**
     * @brief Get texture by handle
     * @param handle Texture handle
     * @return Texture pointer (nullptr if not found)
     */
    Texture* get_texture(TextureHandle handle);
    const Texture* get_texture(TextureHandle handle) const;

    /**
     * @brief Unload texture
     * @param handle Texture handle
     */
    void unload_texture(TextureHandle handle);

    /**
     * @brief Clear all textures
     */
    void clear();

    /**
     * @brief Get total texture memory usage
     * @return Memory usage in bytes
     */
    size_t get_memory_usage() const;

    /**
     * @brief Get number of loaded textures
     * @return Texture count
     */
    size_t get_texture_count() const { return textures_.size(); }

private:
    std::unordered_map<std::string, TextureHandle> path_to_handle_;
    std::unordered_map<TextureHandle, std::unique_ptr<Texture>> textures_;
    TextureHandle next_handle_;
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H
```

### 文件：include/are/texture/sampler.h

```cpp
/**
 * @file sampler.h
 * @brief Texture sampling utilities
 */

#ifndef ARE_INCLUDE_TEXTURE_SAMPLER_H
#define ARE_INCLUDE_TEXTURE_SAMPLER_H

#include <are/core/types.h>

namespace are {

// Forward declaration
class Texture;
class TextureWrap;

/**
 * @class Sampler
 * @brief Texture sampling utilities for CPU ray tracing
 * 
 * Provides bilinear filtering and wrapping modes for CPU-side texture access.
 */
class Sampler {
public:
    /**
     * @brief Sample texture at UV coordinates
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample(const Texture& texture, const Vec2& uv);

    /**
     * @brief Sample texture with bilinear filtering
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample_bilinear(const Texture& texture, const Vec2& uv);

    /**
     * @brief Sample texture at nearest pixel
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample_nearest(const Texture& texture, const Vec2& uv);

private:
    static Vec2 apply_wrap(const Vec2& uv, TextureWrap wrap);
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_SAMPLER_H
```

接下来我再给你一个项目中的.cpp文件，让你熟悉错误处理和日志输出等信息：
### 文件：src/utils/image_io.cpp
```cpp
/**
 * @file image_io.cpp
 * @brief Implementation of image I/O utilities
 */

#include <are/utils/image_io.h>
#include <are/core/logger.h>
#include <cstring>
#include <algorithm>
#include <string>
#include <fstream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace are {

bool ImageData::is_valid() const {
    return width_ > 0 && height_ > 0 && channels_ > 0 && !data_.empty();
}

const uint8_t* ImageData::get_pixel(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return nullptr;
    }
    
    size_t index = (y * width_ + x) * channels_;
    return &data_[index];
}

void ImageData::set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return;
    }
    
    size_t index = (y * width_ + x) * channels_;
    
    data_[index + 0] = r;
    data_[index + 1] = g;
    data_[index + 2] = b;
    
    if (channels_ == 4) {
        data_[index + 3] = a;
    }
}

ImageData load_image(const std::string& filename, bool flip_vertically) {
    ImageData image;
    
    // Set flip flag
    stbi_set_flip_vertically_on_load(flip_vertically ? 1 : 0);
    
    // Load image
    int width, height, channels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        ARE_LOG_ERROR("Failed to load image: " + filename + " - " + stbi_failure_reason());
        return image;
    }
    
    // Copy data to ImageData
    image.width_ = width;
    image.height_ = height;
    image.channels_ = channels;
    
    size_t data_size = width * height * channels;
    image.data_.resize(data_size);
    std::memcpy(image.data_.data(), data, data_size);
    
    // Free stb_image data
    stbi_image_free(data);
    
    ARE_LOG_INFO("Loaded image: " + filename + " (" + 
                 std::to_string(width) + "x" + std::to_string(height) + 
                 ", " + std::to_string(channels) + " channels)");
    
    return image;
}

ImageFormat detect_format(const std::string& filename) {
    std::string ext;
    size_t dot_pos = filename.find_last_of('.');
    
    if (dot_pos != std::string::npos) {
        ext = filename.substr(dot_pos + 1);
        
        // Convert to lowercase
        std::transform(ext.begin(), ext.end(), ext.begin(),
                      [](unsigned char c) { return std::tolower(c); });
    }
    
    if (ext == "ppm") return ImageFormat::ARE_IMAGE_FORMAT_PPM;
    if (ext == "bmp") return ImageFormat::ARE_IMAGE_FORMAT_BMP;
    if (ext == "png") return ImageFormat::ARE_IMAGE_FORMAT_PNG;
    if (ext == "jpg" || ext == "jpeg") return ImageFormat::ARE_IMAGE_FORMAT_JPG;
    
    // Default to PNG
    return ImageFormat::ARE_IMAGE_FORMAT_PNG;
}

bool save_image(const std::string& filename, const ImageData& data, ImageFormat format) {
    if (!data.is_valid()) {
        ARE_LOG_ERROR("Cannot save invalid image data");
        return false;
    }
    
    // Auto-detect format if not specified
    if (format == ImageFormat::ARE_IMAGE_FORMAT_PNG) {
        format = detect_format(filename);
    }
    
    int result = 0;
    
    switch (format) {
        case ImageFormat::ARE_IMAGE_FORMAT_PPM: {
            // Write PPM manually (stb doesn't support it)
            std::ofstream file(filename, std::ios::binary);
            if (!file.is_open()) {
                ARE_LOG_ERROR("Failed to open file for writing: " + filename);
                return false;
            }
            
            file << "P6\n" << data.width_ << " " << data.height_ << "\n255\n";
            
            for (int i = 0; i < data.width_ * data.height_; ++i) {
                int idx = i * data.channels_;
                file.put(data.data_[idx + 0]); // R
                file.put(data.data_[idx + 1]); // G
                file.put(data.data_[idx + 2]); // B
            }
            
            file.close();
            result = 1;
            break;
        }
        
        case ImageFormat::ARE_IMAGE_FORMAT_BMP:
            result = stbi_write_bmp(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data());
            break;
        
        case ImageFormat::ARE_IMAGE_FORMAT_PNG:
            result = stbi_write_png(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data(), 
                                   data.width_ * data.channels_);
            break;
        
        case ImageFormat::ARE_IMAGE_FORMAT_JPG:
            result = stbi_write_jpg(filename.c_str(), data.width_, data.height_, 
                                   data.channels_, data.data_.data(), 90);
            break;
    }
    
    if (result == 0) {
        ARE_LOG_ERROR("Failed to save image: " + filename);
        return false;
    }
    
    ARE_LOG_INFO("Saved image: " + filename);
    return true;
}

bool save_image(const std::string& filename, const uint8_t* pixels,
               int width, int height, int channels, ImageFormat format) {
    ImageData data;
    data.width_ = width;
    data.height_ = height;
    data.channels_ = channels;
    
    size_t data_size = width * height * channels;
    data.data_.resize(data_size);
    std::memcpy(data.data_.data(), pixels, data_size);
    
    return save_image(filename, data, format);
}

} // namespace are
```
哦对了，这是我们现在的CMakeLists.txt，你可以在编写完这个模块之后编写cornell_box的测试代码和独属于Phase 5的测试代码（在CPU光追效率太低的时候使用）：
```CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(AuroraRenderingEngine VERSION 0.1.0 LANGUAGES CXX C)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set C standard for GLAD
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build options
option(ARE_BUILD_SHARED "Build shared library" OFF)
option(ARE_BUILD_EXAMPLES "Build example programs" ON)
option(ARE_ENABLE_PROFILING "Enable performance profiling" ON)
option(ARE_ENABLE_DEBUG_VIS "Enable debug visualization" ON)

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Compiler flags
if(MSVC)
    add_compile_options(/W4)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    # Disable specific warnings that are too strict
    add_compile_options(/wd4100)  # unreferenced formal parameter
else()
    add_compile_options(-Wall -Wextra -pedantic)
    # Disable specific warnings
    add_compile_options(-Wno-unused-parameter)
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Try to find spdlog (system installation)
find_package(spdlog QUIET)

if(NOT spdlog_FOUND)
    message(STATUS "spdlog not found in system, using local version")
    # Use local spdlog
    set(SPDLOG_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/spdlog/include")
    if(NOT EXISTS ${SPDLOG_INCLUDE_DIR})
        message(FATAL_ERROR "spdlog not found. Please install spdlog or place it in lib/spdlog/")
    endif()
    add_library(spdlog INTERFACE)
    target_include_directories(spdlog INTERFACE ${SPDLOG_INCLUDE_DIR})
    # spdlog compile definitions
    target_compile_definitions(spdlog INTERFACE SPDLOG_COMPILED_LIB)
endif()

# Find OpenMP (optional)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(ARE_USE_OPENMP ON)
    add_compile_definitions(ARE_USE_OPENMP)
    message(STATUS "OpenMP found and enabled")
else()
    message(STATUS "OpenMP not found, multithreading will use std::thread")
endif()

# GLAD library path
set(GLAD_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/glad")
set(GLAD_SOURCE_DIR "${CMAKE_SOURCE_DIR}/lib/glad")

if(NOT EXISTS ${GLAD_INCLUDE_DIR})
    message(FATAL_ERROR "GLAD include directory not found: ${GLAD_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${GLAD_SOURCE_DIR})
    message(FATAL_ERROR "GLAD source directory not found: ${GLAD_SOURCE_DIR}")
endif()

# STB library path
set(STB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/stb")

if(NOT EXISTS ${STB_INCLUDE_DIR})
    message(FATAL_ERROR "STB include directory not found: ${STB_INCLUDE_DIR}")
endif()

# Collect all source files from src/
file(GLOB_RECURSE ARE_SOURCES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)

# Collect all GLAD source files
file(GLOB GLAD_SOURCES 
    "${GLAD_SOURCE_DIR}/*.c"
)

# Add GLAD sources to ARE sources
list(APPEND ARE_SOURCES ${GLAD_SOURCES})

# Collect all header files
file(GLOB_RECURSE ARE_HEADERS 
    "${CMAKE_SOURCE_DIR}/include/*.h"
    "${CMAKE_SOURCE_DIR}/include/*.hpp"
)

# Create library
if(ARE_BUILD_SHARED)
    add_library(are SHARED ${ARE_SOURCES} ${ARE_HEADERS})
    target_compile_definitions(are PRIVATE ARE_BUILD_SHARED)
    message(STATUS "Building shared library")
else()
    add_library(are STATIC ${ARE_SOURCES} ${ARE_HEADERS})
    message(STATUS "Building static library")
endif()

# Set target properties
set_target_properties(are PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${ARE_HEADERS}"
)

# Include directories
target_include_directories(are
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${GLAD_INCLUDE_DIR}
        ${STB_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(are
    PUBLIC
        OpenGL::GL
        glfw
        glm::glm
)

# Link spdlog
if(spdlog_FOUND)
    target_link_libraries(are PUBLIC spdlog::spdlog)
else()
    target_link_libraries(are PUBLIC spdlog)
    target_include_directories(are PRIVATE ${SPDLOG_INCLUDE_DIR})
endif()

# Link OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(are PUBLIC OpenMP::OpenMP_CXX)
endif()

# Platform-specific libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(are PUBLIC dl pthread)
endif()

# Compile definitions
if(ARE_ENABLE_PROFILING)
    target_compile_definitions(are PUBLIC ARE_ENABLE_PROFILING)
    message(STATUS "Profiling enabled")
endif()

if(ARE_ENABLE_DEBUG_VIS)
    target_compile_definitions(are PUBLIC ARE_ENABLE_DEBUG_VIS)
    message(STATUS "Debug visualization enabled")
endif()

# Build examples
if(ARE_BUILD_EXAMPLES)
    # Define helper function for creating examples
    function(add_are_example EXAMPLE_NAME)
        add_executable(${EXAMPLE_NAME} ${ARGN})
        target_link_libraries(${EXAMPLE_NAME} PRIVATE are)
        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    endfunction()
    
    # Add example subdirectories
    add_subdirectory(examples/00_phase1_test)
	add_subdirectory(examples/01_phase2_test)
	add_subdirectory(examples/02_visual_test)
	add_subdirectory(examples/02_phase3_test)
	add_subdirectory(examples/03_phase4_test)
    
    message(STATUS "Examples will be built")
endif()

# Installation rules
install(TARGETS are
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/are
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/are
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/shaders
    DESTINATION share/are/shaders
)

# Print configuration summary
message(STATUS "")
message(STATUS "========================================")
message(STATUS "Aurora Rendering Engine Configuration")
message(STATUS "========================================")
message(STATUS "  Version:              ${PROJECT_VERSION}")
message(STATUS "  Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "  Library type:         ${ARE_BUILD_SHARED}")
message(STATUS "  C++ standard:         ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler:             ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Features:")
message(STATUS "  OpenMP support:       ${ARE_USE_OPENMP}")
message(STATUS "  Build examples:       ${ARE_BUILD_EXAMPLES}")
message(STATUS "  Enable profiling:     ${ARE_ENABLE_PROFILING}")
message(STATUS "  Enable debug vis:     ${ARE_ENABLE_DEBUG_VIS}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  OpenGL:               ${OPENGL_LIBRARIES}")
message(STATUS "  GLFW:                 Found")
message(STATUS "  GLM:                  Found")
if(spdlog_FOUND)
    message(STATUS "  spdlog:               Found (system)")
else()
    message(STATUS "  spdlog:               Found (local)")
endif()
message(STATUS "  GLAD:                 ${GLAD_INCLUDE_DIR}")
message(STATUS "  STB:                  ${STB_INCLUDE_DIR}")
message(STATUS "")
message(STATUS "Output directories:")
message(STATUS "  Executables:          ${CMAKE_BINARY_DIR}/bin")
message(STATUS "  Libraries:            ${CMAKE_BINARY_DIR}/lib")
message(STATUS "========================================")
message(STATUS "")
```

目前你只需要实现Phase 5所要求的文件即可，其他大部分模块已经实现。
