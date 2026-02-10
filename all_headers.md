### 文件：include/basic/types.h

```cpp
#ifndef ARE_INCLUDE_BASIC_TYPES_H
#define ARE_INCLUDE_BASIC_TYPES_H

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace are {

/// @brief Basic vector types using GLM
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;

/// @brief Basic matrix types using GLM
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;

/// @brief Basic integer types
using uint = uint32_t;
using uchar = uint8_t;

/// @brief Handle types for GPU resources
using TextureHandle = uint;
using BufferHandle = uint;
using ShaderHandle = uint;
using FramebufferHandle = uint;

/// @brief Invalid handle constant
constexpr uint INVALID_HANDLE = 0;

/// @brief Vertex structure for mesh data
struct Vertex {
    Vec3 position_;
    Vec3 normal_;
    Vec2 texcoord_;
    Vec3 tangent_;
};

/// @brief Ray structure for ray tracing
struct Ray {
    Vec3 origin_;
    Vec3 direction_;
    float t_min_;
    float t_max_;
};

/// @brief Hit information for ray-surface intersection
struct HitInfo {
    bool hit_;
    float t_;
    Vec3 position_;
    Vec3 normal_;
    Vec2 texcoord_;
    uint material_id_;
};

/// @brief Rendering statistics
struct RenderStats {
    float frame_time_ms_;
    uint triangle_count_;
    uint ray_count_;
    float gbuffer_time_ms_;
    float raytrace_time_ms_;
};

} // namespace are

#endif // ARE_INCLUDE_BASIC_TYPES_H
```

### 文件：include/basic/constants.h

```cpp
#ifndef ARE_INCLUDE_BASIC_CONSTANTS_H
#define ARE_INCLUDE_BASIC_CONSTANTS_H

namespace are {

/// @brief Maximum number of lights in scene
constexpr int MAX_LIGHTS = 16;

/// @brief Maximum ray tracing depth
constexpr int MAX_RAY_DEPTH = 8;

/// @brief Default samples per pixel for ray tracing
constexpr int DEFAULT_SPP = 1;

/// @brief G-Buffer attachment indices
constexpr int GBUFFER_POSITION = 0;
constexpr int GBUFFER_NORMAL = 1;
constexpr int GBUFFER_ALBEDO = 2;
constexpr int GBUFFER_COUNT = 3;

/// @brief Compute shader work group size
constexpr int COMPUTE_GROUP_SIZE_X = 16;
constexpr int COMPUTE_GROUP_SIZE_Y = 16;

/// @brief Mathematical constants
constexpr float PI = 3.14159265359f;
constexpr float INV_PI = 0.31830988618f;
constexpr float EPSILON = 1e-4f;

} // namespace are

#endif // ARE_INCLUDE_BASIC_CONSTANTS_H
```

### 文件：include/basic/math.h

```cpp
#ifndef ARE_INCLUDE_BASIC_MATH_UTILS_H
#define ARE_INCLUDE_BASIC_MATH_UTILS_H

#include "types.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace are {

/// @brief Math utility functions wrapping GLM
class MathUtils {
public:
    /// @brief Create perspective projection matrix
    /// @param fov Field of view in radians
    /// @param aspect Aspect ratio
    /// @param near Near plane distance
    /// @param far Far plane distance
    /// @return Projection matrix
    static Mat4 perspective(float fov, float aspect, float near, float far);
    
    /// @brief Create look-at view matrix
    /// @param eye Camera position
    /// @param center Look-at target
    /// @param up Up vector
    /// @return View matrix
    static Mat4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up);
    
    /// @brief Normalize a vector
    /// @param v Input vector
    /// @return Normalized vector
    static Vec3 normalize(const Vec3& v);
    
    /// @brief Calculate dot product
    /// @param a First vector
    /// @param b Second vector
    /// @return Dot product
    static float dot(const Vec3& a, const Vec3& b);
    
    /// @brief Calculate cross product
    /// @param a First vector
    /// @param b Second vector
    /// @return Cross product
    static Vec3 cross(const Vec3& a, const Vec3& b);
    
    /// @brief Reflect vector around normal
    /// @param incident Incident vector
    /// @param normal Surface normal
    /// @return Reflected vector
    static Vec3 reflect(const Vec3& incident, const Vec3& normal);
    
    /// @brief Get pointer to matrix data (for OpenGL)
    /// @param mat Input matrix
    /// @return Pointer to matrix data
    static const float* value_ptr(const Mat4& mat);
};

} // namespace are

#endif // ARE_INCLUDE_BASIC_MATH_UTILS_H
```

### 文件：include/core/bvh.h

```cpp
#ifndef ARE_INCLUDE_CORE_BVH_H
#define ARE_INCLUDE_CORE_BVH_H

#include "basic/types.h"
#include "scene/mesh.h"
#include "resource/buffer.h"
#include <vector>
#include <memory>

namespace are {

/// @brief Axis-aligned bounding box
struct AABB {
    Vec3 min_;
    Vec3 max_;
    
    /// @brief Construct AABB from min and max points
    AABB(const Vec3& min = Vec3(0.0f), const Vec3& max = Vec3(0.0f))
        : min_(min), max_(max) {}
    
    /// @brief Expand AABB to include point
    void expand(const Vec3& point);
    
    /// @brief Expand AABB to include another AABB
    void expand(const AABB& other);
    
    /// @brief Get center of AABB
    Vec3 center() const { return (min_ + max_) * 0.5f; }
    
    /// @brief Get surface area of AABB
    float surface_area() const;
    
    /// @brief Check if AABB is valid
    bool is_valid() const;
};

/// @brief Triangle primitive for BVH
struct Triangle {
    Vec3 v0_, v1_, v2_;
    Vec3 n0_, n1_, n2_;
    Vec2 uv0_, uv1_, uv2_;
    uint material_id_;
    
    /// @brief Get bounding box of triangle
    AABB get_bounds() const;
    
    /// @brief Get centroid of triangle
    Vec3 get_centroid() const;
};

/// @brief BVH node for GPU
struct BVHNode {
    Vec3 aabb_min_;
    uint left_first_;  // Left child index or first primitive index
    Vec3 aabb_max_;
    uint count_;       // 0 for interior node, >0 for leaf node
};

/// @brief Bounding Volume Hierarchy for ray tracing acceleration
class BVH {
public:
    /// @brief Constructor
    BVH();
    
    /// @brief Destructor
    ~BVH();
    
    /// @brief Build BVH from meshes
    /// @param meshes Mesh list
    /// @return True if build succeeded
    bool build(const std::vector<std::shared_ptr<Mesh>>& meshes);
    
    /// @brief Upload BVH to GPU
    /// @param node_buffer Buffer for BVH nodes
    /// @param triangle_buffer Buffer for triangles
    /// @return True if upload succeeded
    bool upload_to_gpu(Buffer& node_buffer, Buffer& triangle_buffer);
    
    /// @brief Get total node count
    /// @return Node count
    uint get_node_count() const { return static_cast<uint>(nodes_.size()); }
    
    /// @brief Get total triangle count
    /// @return Triangle count
    uint get_triangle_count() const { return static_cast<uint>(triangles_.size()); }
    
    /// @brief Clear BVH data
    void clear();

private:
    std::vector<BVHNode> nodes_;
    std::vector<Triangle> triangles_;
    std::vector<uint> triangle_indices_;
    
    /// @brief Recursively build BVH
    /// @param node_idx Current node index
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    void build_recursive_(uint node_idx, uint first_prim, uint prim_count);
    
    /// @brief Find best split using SAH
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @param axis Split axis (output)
    /// @param split_pos Split position (output)
    /// @return Split cost
    float find_best_split_(uint first_prim, uint prim_count, int& axis, float& split_pos);
    
    /// @brief Calculate node bounds
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @return Bounding box
    AABB calculate_bounds_(uint first_prim, uint prim_count);
    
    /// @brief Calculate centroid bounds
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @return Centroid bounding box
    AABB calculate_centroid_bounds_(uint first_prim, uint prim_count);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_BVH_H
```

### 文件：include/core/gbuffer.h

```cpp
#ifndef ARE_INCLUDE_CORE_GBUFFER_H
#define ARE_INCLUDE_CORE_GBUFFER_H

#include "basic/types.h"
#include "basic/constants.h"
#include "scene/scene.h"
#include "resource/shader.h"

namespace are {

/// @brief G-Buffer manager for deferred rendering
class GBuffer {
public:
    /// @brief Constructor
    /// @param width Buffer width
    /// @param height Buffer height
    GBuffer(uint width, uint height);
    
    /// @brief Destructor
    ~GBuffer();
    
    /// @brief Initialize G-Buffer (create framebuffer and textures)
    /// @return True if initialization succeeded
    bool initialize();
    
    /// @brief Release G-Buffer resources
    void release();
    
    /// @brief Render scene to G-Buffer
    /// @param scene Scene to render
    /// @param shader Shader program for G-Buffer pass
    void render(const Scene& scene, const Shader& shader);
    
    /// @brief Resize G-Buffer
    /// @param width New width
    /// @param height New height
    void resize(uint width, uint height);
    
    /// @brief Get texture handle for specific buffer
    /// @param index Buffer index (GBUFFER_POSITION, GBUFFER_NORMAL, etc.)
    /// @return Texture handle
    TextureHandle get_texture(int index) const;
    
    /// @brief Get framebuffer handle
    /// @return Framebuffer handle
    FramebufferHandle get_framebuffer() const { return fbo_; }
    
    /// @brief Get buffer dimensions
    /// @param width Output width
    /// @param height Output height
    void get_dimensions(uint& width, uint& height) const;

private:
    uint width_;
    uint height_;
    FramebufferHandle fbo_;
    TextureHandle textures_[GBUFFER_COUNT];
    TextureHandle depth_texture_;
    
    bool initialized_;
    
    /// @brief Create texture for G-Buffer attachment
    /// @param internal_format OpenGL internal format
    /// @param format OpenGL format
    /// @param type OpenGL type
    /// @return Texture handle
    TextureHandle create_texture_(uint internal_format, uint format, uint type);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_GBUFFER_H
```

### 文件：include/core/raytracer.h

```cpp
#ifndef ARE_INCLUDE_CORE_RAYTRACER_H
#define ARE_INCLUDE_CORE_RAYTRACER_H

#include "basic/types.h"
#include "core/bvh.h" // 添加
#include "core/gbuffer.h"
#include "resource/buffer.h"
#include "resource/shader.h"
#include "scene/scene.h"

namespace are {

/// @brief Ray tracing configuration
struct RayTracerConfig {
	uint samples_per_pixel_;
	uint max_depth_;
	bool enable_shadows_;
	bool enable_reflections_;
	bool enable_accumulation_;
	bool use_bvh_; // 添加BVH开关
};

/// @brief Compute shader based ray tracer
class RayTracer {
public:
	/// @brief Constructor
	/// @param width Output width
	/// @param height Output height
	/// @param config Ray tracer configuration
	RayTracer(uint width, uint height, const RayTracerConfig &config);

	/// @brief Destructor
	~RayTracer();

	/// @brief Initialize ray tracer
	/// @return True if initialization succeeded
	bool initialize();

	/// @brief Release resources
	void release();

	/// @brief Trace rays using G-Buffer as input
	/// @param scene Scene data
	/// @param gbuffer G-Buffer containing geometry information
	/// @param output_texture Output texture for ray traced result
	void trace(const Scene &scene, const GBuffer &gbuffer, TextureHandle output_texture);

	/// @brief Resize output
	/// @param width New width
	/// @param height New height
	void resize(uint width, uint height);

	/// @brief Reset accumulation buffer
	void reset_accumulation();

	/// @brief Get current configuration
	/// @return Current configuration
	const RayTracerConfig &get_config() const {
		return config_;
	}

	/// @brief Update configuration
	/// @param config New configuration
	void set_config(const RayTracerConfig &config);

	/// @brief Rebuild BVH from scene
	/// @param scene Scene to build BVH from
	/// @return True if build succeeded
	bool rebuild_bvh(const Scene &scene);

	/// @brief Set compute shader (called by renderer)
	/// @param shader Compute shader
	void set_compute_shader(const Shader &shader);

private:
	uint width_;
	uint height_;
	RayTracerConfig config_;

	Shader compute_shader_;
	TextureHandle accumulation_texture_;
	BufferHandle scene_buffer_;
	BufferHandle material_buffer_;
	BufferHandle light_buffer_;

	// BVH related
	std::unique_ptr<BVH> bvh_; // 添加
	Buffer bvh_node_buffer_; // 添加
	Buffer bvh_triangle_buffer_; // 添加
	bool bvh_built_; // 添加

	uint frame_count_;
	bool initialized_;

	/// @brief Upload scene data to GPU buffers
	/// @param scene Scene to upload
	void upload_scene_data_(const Scene &scene);

	/// @brief Bind G-Buffer textures to compute shader
	/// @param gbuffer G-Buffer to bind
	void bind_gbuffer_(const GBuffer &gbuffer);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RAYTRACER_H
```

### 文件：include/core/renderer.h

```cpp
#ifndef ARE_INCLUDE_CORE_RENDERER_H
#define ARE_INCLUDE_CORE_RENDERER_H

#include "basic/types.h"
#include "scene/scene.h"
#include "core/gbuffer.h"
#include "core/raytracer.h"
#include "core/screen_blit.h"
#include "core/shader_manager.h"
#include <memory>

namespace are {

/// @brief Main renderer configuration
struct RendererConfig {
    uint width_;
    uint height_;
    uint samples_per_pixel_;
    uint max_ray_depth_;
    bool enable_denoising_;
    bool enable_accumulation_;
};

/// @brief Main rendering engine interface
class Renderer {
public:
    /// @brief Constructor
    /// @param config Renderer configuration
    Renderer(const RendererConfig& config);
    
    /// @brief Destructor
    ~Renderer();
    
    /// @brief Initialize renderer (OpenGL context must be current)
    /// @return True if initialization succeeded
    bool initialize();
    
    /// @brief Shutdown renderer and release resources
    void shutdown();
    
    /// @brief Render a frame
    /// @param scene Scene to render
    /// @param output_texture Output texture handle (0 for default framebuffer)
    /// @return Rendering statistics
    RenderStats render(const Scene& scene, TextureHandle output_texture = 0);
    
    /// @brief Resize render targets
    /// @param width New width
    /// @param height New height
    void resize(uint width, uint height);
    
    /// @brief Get current configuration
    /// @return Current configuration
    const RendererConfig& get_config() const { return config_; }
    
    /// @brief Update configuration
    /// @param config New configuration
    void set_config(const RendererConfig& config);

private:
    RendererConfig config_;
    std::unique_ptr<GBuffer> gbuffer_;
    std::unique_ptr<RayTracer> raytracer_;
    std::unique_ptr<ShaderManager> shader_manager_;
	std::unique_ptr<ScreenBlit> screen_blit_;
    
    bool initialized_;
    uint frame_count_;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_RENDERER_H
```

### 文件：include/core/screen_blit.h

```cpp
#ifndef ARE_INCLUDE_CORE_SCREEN_BLIT_H
#define ARE_INCLUDE_CORE_SCREEN_BLIT_H

#include "basic/types.h"
#include "resource/shader.h"

namespace are {

/// @brief Screen blit utility for rendering texture to screen
class ScreenBlit {
public:
    /// @brief Constructor
    ScreenBlit();
    
    /// @brief Destructor
    ~ScreenBlit();
    
    /// @brief Initialize screen blit
    /// @return True if initialization succeeded
    bool initialize();
    
    /// @brief Release resources
    void release();
    
    /// @brief Blit texture to screen
    /// @param texture Texture to blit
    /// @param x Screen X position
    /// @param y Screen Y position
    /// @param width Blit width
    /// @param height Blit height
    void blit(TextureHandle texture, int x, int y, uint width, uint height);
    
    /// @brief Blit texture to full screen
    /// @param texture Texture to blit
    void blit_fullscreen(TextureHandle texture);

private:
    Shader shader_;
    uint vao_;
    uint vbo_;
    bool initialized_;
    
    /// @brief Create fullscreen quad
    void create_quad_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SCREEN_BLIT_H
```

### 文件：include/core/shader_manager.h

```cpp
#ifndef ARE_INCLUDE_CORE_SHADER_MANAGER_H
#define ARE_INCLUDE_CORE_SHADER_MANAGER_H

#include "basic/types.h"
#include "resource/shader.h"
#include <unordered_map>
#include <string>

namespace are {

/// @brief Shader manager for loading and caching shaders
class ShaderManager {
public:
    /// @brief Constructor
    ShaderManager();
    
    /// @brief Destructor
    ~ShaderManager();
    
    /// @brief Initialize shader manager and load built-in shaders
    /// @return True if initialization succeeded
    bool initialize();
    
    /// @brief Release all shaders
    void release();
    
    /// @brief Load shader from files
    /// @param name Shader name for caching
    /// @param vertex_path Vertex shader file path
    /// @param fragment_path Fragment shader file path
    /// @return Shader object
    Shader load_shader(const std::string& name, 
                      const std::string& vertex_path,
                      const std::string& fragment_path);
    
    /// @brief Load compute shader from file
    /// @param name Shader name for caching
    /// @param compute_path Compute shader file path
    /// @return Shader object
    Shader load_compute_shader(const std::string& name,
                              const std::string& compute_path);
    
    /// @brief Get cached shader by name
    /// @param name Shader name
    /// @return Shader object (invalid if not found)
    Shader get_shader(const std::string& name) const;
    
    /// @brief Get G-Buffer shader
    /// @return G-Buffer shader
    const Shader& get_gbuffer_shader() const { return gbuffer_shader_; }
    
    /// @brief Get ray tracing compute shader
    /// @return Ray tracing shader
    const Shader& get_raytracing_shader() const { return raytracing_shader_; }

private:
    std::unordered_map<std::string, Shader> shader_cache_;
    Shader gbuffer_shader_;
    Shader raytracing_shader_;
    
    bool initialized_;
    
    /// @brief Load built-in shaders
    /// @return True if loading succeeded
    bool load_builtin_shaders_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SHADER_MANAGER_H
```

### 文件：include/scene/camera.h

```cpp
#ifndef ARE_INCLUDE_SCENE_CAMERA_H
#define ARE_INCLUDE_SCENE_CAMERA_H

#include "basic/types.h"

namespace are {

/// @brief Camera projection type
enum class ProjectionType {
    PERSPECTIVE,
    ORTHOGRAPHIC
};

/// @brief Camera for rendering
class Camera {
public:
    /// @brief Constructor
    Camera();
    
    /// @brief Destructor
    ~Camera();
    
    /// @brief Set perspective projection
    /// @param fov Field of view in degrees
    /// @param aspect Aspect ratio
    /// @param near Near plane
    /// @param far Far plane
    void set_perspective(float fov, float aspect, float near, float far);
    
    /// @brief Set orthographic projection
    /// @param left Left plane
    /// @param right Right plane
    /// @param bottom Bottom plane
    /// @param top Top plane
    /// @param near Near plane
    /// @param far Far plane
    void set_orthographic(float left, float right, float bottom, float top, float near, float far);
    
    /// @brief Set camera position
    /// @param position Position
    void set_position(const Vec3& position);
    
    /// @brief Set camera target
    /// @param target Target position
    void set_target(const Vec3& target);
    
    /// @brief Set camera up vector
    /// @param up Up vector
    void set_up(const Vec3& up);
    
    /// @brief Get view matrix
    /// @return View matrix
    Mat4 get_view_matrix() const;
    
    /// @brief Get projection matrix
    /// @return Projection matrix
    Mat4 get_projection_matrix() const;
    
    /// @brief Get view-projection matrix
    /// @return View-projection matrix
    Mat4 get_view_projection_matrix() const;
    
    /// @brief Get camera position
    /// @return Position
    const Vec3& get_position() const { return position_; }
    
    /// @brief Get camera forward direction
    /// @return Forward direction
    Vec3 get_forward() const;
    
    /// @brief Get camera right direction
    /// @return Right direction
    Vec3 get_right() const;
    
    /// @brief Get camera up direction
    /// @return Up direction
    Vec3 get_up() const;

private:
    Vec3 position_;
    Vec3 target_;
    Vec3 up_;
    
    ProjectionType projection_type_;
    
    // Perspective parameters
    float fov_;
    float aspect_;
    
    // Orthographic parameters
    float left_, right_, bottom_, top_;
    
    // Common parameters
    float near_;
    float far_;
    
    mutable Mat4 view_matrix_;
    mutable Mat4 projection_matrix_;
    mutable bool view_dirty_;
    mutable bool projection_dirty_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_CAMERA_H
```

### 文件：include/scene/material.h

```cpp
#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include "basic/types.h"
#include "resource/texture.h"
#include <memory>

namespace are {

/// @brief Material type enumeration
enum class MaterialType {
    DIFFUSE = 0,
    METAL = 1,
    DIELECTRIC = 2,
    EMISSIVE = 3
};

/// @brief Material properties
class Material {
public:
    /// @brief Constructor
    Material();
    
    /// @brief Destructor
    ~Material();
    
    /// @brief Set albedo color
    /// @param albedo Albedo color
    void set_albedo(const Vec3& albedo);
    
    /// @brief Set emission color
    /// @param emission Emission color
    void set_emission(const Vec3& emission);
    
    /// @brief Set metallic value
    /// @param metallic Metallic (0-1)
    void set_metallic(float metallic);
    
    /// @brief Set roughness value
    /// @param roughness Roughness (0-1)
    void set_roughness(float roughness);
    
    /// @brief Set index of refraction
    /// @param ior Index of refraction
    void set_ior(float ior);
    
    /// @brief Set material type
    /// @param type Material type
    void set_type(MaterialType type);
    
    /// @brief Set albedo texture
    /// @param texture Albedo texture
    void set_albedo_texture(std::shared_ptr<Texture> texture);
    
    /// @brief Set normal map
    /// @param texture Normal map texture
    void set_normal_texture(std::shared_ptr<Texture> texture);
    
    /// @brief Get albedo color
    /// @return Albedo color
    const Vec3& get_albedo() const { return albedo_; }
    
    /// @brief Get emission color
    /// @return Emission color
    const Vec3& get_emission() const { return emission_; }
    
    /// @brief Get metallic value
    /// @return Metallic
    float get_metallic() const { return metallic_; }
    
    /// @brief Get roughness value
    /// @return Roughness
    float get_roughness() const { return roughness_; }
    
    /// @brief Get index of refraction
    /// @return IOR
    float get_ior() const { return ior_; }
    
    /// @brief Get material type
    /// @return Material type
    MaterialType get_type() const { return type_; }
    
    /// @brief Get albedo texture
    /// @return Albedo texture (nullptr if none)
    std::shared_ptr<Texture> get_albedo_texture() const { return albedo_texture_; }
    
    /// @brief Get normal texture
    /// @return Normal texture (nullptr if none)
    std::shared_ptr<Texture> get_normal_texture() const { return normal_texture_; }

private:
    Vec3 albedo_;
    Vec3 emission_;
    float metallic_;
    float roughness_;
    float ior_;
    MaterialType type_;
    
    std::shared_ptr<Texture> albedo_texture_;
    std::shared_ptr<Texture> normal_texture_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MATERIAL_H
```

### 文件：include/scene/light.h

```cpp
#ifndef ARE_INCLUDE_SCENE_LIGHT_H
#define ARE_INCLUDE_SCENE_LIGHT_H

#include "basic/types.h"

namespace are {

/// @brief Light type enumeration
enum class LightType {
	DIRECTIONAL = 0,
	POINT = 1,
	SPOT = 2
};

/// @brief Light source
class Light {
public:
	/// @brief Constructor
	Light();

	/// @brief Destructor
	~Light();

	/// @brief Set light type
	/// @param type Light type
	void set_type(LightType type);

	/// @brief Set light position (for point and spot lights)
	/// @param position Light position
	void set_position(const Vec3 &position);

	/// @brief Set light direction (for directional and spot lights)
	/// @param direction Light direction
	void set_direction(const Vec3 &direction);

	/// @brief Set light color
	/// @param color Light color
	void set_color(const Vec3 &color);

	/// @brief Set light intensity
	/// @param intensity Light intensity
	void set_intensity(float intensity);

	/// @brief Set light range (for point and spot lights)
	/// @param range Light range
	void set_range(float range);

	/// @brief Set spot light angles
	/// @param inner_angle Inner cone angle in degrees
	/// @param outer_angle Outer cone angle in degrees
	void set_spot_angles(float inner_angle, float outer_angle);

	/// @brief Get light type
	/// @return Light type
	LightType get_type() const {
		return type_;
	}

	/// @brief Get light position
	/// @return Light position
	const Vec3 &get_position() const {
		return position_;
	}

	/// @brief Get light direction
	/// @return Light direction
	const Vec3 &get_direction() const {
		return direction_;
	}

	/// @brief Get light color
	/// @return Light color
	const Vec3 &get_color() const {
		return color_;
	}

	/// @brief Get light intensity
	/// @return Light intensity
	float get_intensity() const {
		return intensity_;
	}

	/// @brief Get light range
	/// @return Light range
	float get_range() const {
		return range_;
	}

	/// @brief Get spot light inner angle
	/// @return Inner angle in radians
	float get_inner_angle() const {
		return inner_angle_;
	}

	/// @brief Get spot light outer angle
	/// @return Outer angle in radians
	float get_outer_angle() const {
		return outer_angle_;
	}

private:
	LightType type_;
	Vec3 position_;
	Vec3 direction_;
	Vec3 color_;
	float intensity_;
	float range_;
	float inner_angle_;
	float outer_angle_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_LIGHT_H
```

### 文件：include/scene/scene.h

```cpp
#ifndef ARE_INCLUDE_SCENE_SCENE_H
#define ARE_INCLUDE_SCENE_SCENE_H

#include "basic/types.h"
#include "scene/camera.h"
#include "scene/mesh.h"
#include "scene/material.h"
#include "scene/light.h"
#include <vector>
#include <memory>

namespace are {

/// @brief Scene container holding all scene objects
class Scene {
public:
    /// @brief Constructor
    Scene();
    
    /// @brief Destructor
    ~Scene();
    
    /// @brief Add mesh to scene
    /// @param mesh Mesh to add
    /// @return Mesh index
    uint add_mesh(std::shared_ptr<Mesh> mesh);
    
    /// @brief Add material to scene
    /// @param material Material to add
    /// @return Material index
    uint add_material(std::shared_ptr<Material> material);
    
    /// @brief Add light to scene
    /// @param light Light to add
    /// @return Light index
    uint add_light(std::shared_ptr<Light> light);
    
    /// @brief Set active camera
    /// @param camera Camera to set
    void set_camera(std::shared_ptr<Camera> camera);
    
    /// @brief Get active camera
    /// @return Active camera
    const Camera& get_camera() const { return *camera_; }
    
    /// @brief Get all meshes
    /// @return Mesh list
    const std::vector<std::shared_ptr<Mesh>>& get_meshes() const { return meshes_; }
    
    /// @brief Get all materials
    /// @return Material list
    const std::vector<std::shared_ptr<Material>>& get_materials() const { return materials_; }
    
    /// @brief Get all lights
    /// @return Light list
    const std::vector<std::shared_ptr<Light>>& get_lights() const { return lights_; }
    
    /// @brief Clear all scene objects
    void clear();
    
    /// @brief Update scene (animations, transforms, etc.)
    /// @param delta_time Time since last update
    void update(float delta_time);

private:
    std::shared_ptr<Camera> camera_;
    std::vector<std::shared_ptr<Mesh>> meshes_;
    std::vector<std::shared_ptr<Material>> materials_;
    std::vector<std::shared_ptr<Light>> lights_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SCENE_H
```

### 文件：include/scene/mesh.h

```cpp
#ifndef ARE_INCLUDE_SCENE_MESH_H
#define ARE_INCLUDE_SCENE_MESH_H

#include "basic/types.h"
#include <vector>

namespace are {

/// @brief Mesh data container
class Mesh {
public:
    /// @brief Constructor
    Mesh();
    
    /// @brief Destructor
    ~Mesh();
    
    /// @brief Set vertex data
    /// @param vertices Vertex array
    void set_vertices(const std::vector<Vertex>& vertices);
    
    /// @brief Set index data
    /// @param indices Index array
    void set_indices(const std::vector<uint>& indices);
    
    /// @brief Set material index
    /// @param material_id Material index
    void set_material(uint material_id);
    
    /// @brief Set transform matrix
    /// @param transform Transform matrix
    void set_transform(const Mat4& transform);
    
    /// @brief Get vertices
    /// @return Vertex array
    const std::vector<Vertex>& get_vertices() const { return vertices_; }
    
    /// @brief Get indices
    /// @return Index array
    const std::vector<uint>& get_indices() const { return indices_; }
    
    /// @brief Get material index
    /// @return Material index
    uint get_material() const { return material_id_; }
    
    /// @brief Get transform matrix
    /// @return Transform matrix
    const Mat4& get_transform() const { return transform_; }
    
    /// @brief Upload mesh data to GPU
    /// @return True if upload succeeded
    bool upload_to_gpu();
    
    /// @brief Release GPU resources
    void release_gpu_resources();
    
    /// @brief Get VAO handle
    /// @return VAO handle
    uint get_vao() const { return vao_; }
    
    /// @brief Check if mesh is uploaded to GPU
    /// @return True if uploaded
    bool is_uploaded() const { return uploaded_; }

private:
    std::vector<Vertex> vertices_;
    std::vector<uint> indices_;
    uint material_id_;
    Mat4 transform_;
    
    uint vao_;
    uint vbo_;
    uint ebo_;
    bool uploaded_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MESH_H
```

### 文件：include/resource/buffer.h

```cpp
#ifndef ARE_INCLUDE_RESOURCE_BUFFER_H
#define ARE_INCLUDE_RESOURCE_BUFFER_H

#include "basic/types.h"

namespace are {

/// @brief Buffer usage hint
enum class BufferUsage {
    STATIC_DRAW,
    DYNAMIC_DRAW,
    STREAM_DRAW
};

/// @brief Buffer type
enum class BufferType {
    VERTEX_BUFFER,
    INDEX_BUFFER,
    UNIFORM_BUFFER,
    SHADER_STORAGE_BUFFER
};

/// @brief GPU buffer resource
class Buffer {
public:
    /// @brief Constructor
    Buffer();
    
    /// @brief Destructor
    ~Buffer();
    
    /// @brief Create buffer
    /// @param type Buffer type
    /// @param size Buffer size in bytes
    /// @param data Initial data (nullptr for empty buffer)
    /// @param usage Usage hint
    /// @return True if creation succeeded
    bool create(BufferType type, size_t size, const void* data, BufferUsage usage);
    
    /// @brief Update buffer data
    /// @param offset Offset in bytes
    /// @param size Size in bytes
    /// @param data Data to upload
    void update(size_t offset, size_t size, const void* data);
    
    /// @brief Bind buffer
    void bind() const;
    
    /// @brief Bind buffer to binding point (for UBO/SSBO)
    /// @param binding_point Binding point index
    void bind_base(uint binding_point) const;
    
    /// @brief Unbind buffer
    void unbind() const;
    
    /// @brief Release buffer resources
    void release();
    
    /// @brief Get buffer handle
    /// @return Buffer handle
    BufferHandle get_handle() const { return handle_; }
    
    /// @brief Get buffer size
    /// @return Size in bytes
    size_t get_size() const { return size_; }
    
    /// @brief Get buffer type
    /// @return Buffer type
    BufferType get_type() const { return type_; }
    
    /// @brief Check if buffer is valid
    /// @return True if valid
    bool is_valid() const { return handle_ != INVALID_HANDLE; }

private:
    BufferHandle handle_;
    BufferType type_;
    size_t size_;
    BufferUsage usage_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_BUFFER_H
```

### 文件：include/resource/model_loader.h

```cpp
#ifndef ARE_INCLUDE_RESOURCE_MODEL_LOADER_H
#define ARE_INCLUDE_RESOURCE_MODEL_LOADER_H

#include "basic/types.h"
#include "scene/mesh.h"
#include "scene/material.h"
#include <string>
#include <vector>
#include <memory>

namespace are {

/// @brief Model loader using Assimp
class ModelLoader {
public:
    /// @brief Load model from file
    /// @param path Model file path
    /// @param meshes Output mesh list
    /// @param materials Output material list
    /// @param flip_uvs Flip UV coordinates vertically
    /// @return True if loading succeeded
    static bool load(const std::string& path,
                    std::vector<std::shared_ptr<Mesh>>& meshes,
                    std::vector<std::shared_ptr<Material>>& materials,
                    bool flip_uvs = true);
    
    /// @brief Load model and automatically upload to GPU
    /// @param path Model file path
    /// @param meshes Output mesh list
    /// @param materials Output material list
    /// @param flip_uvs Flip UV coordinates vertically
    /// @return True if loading succeeded
    static bool load_and_upload(const std::string& path,
                               std::vector<std::shared_ptr<Mesh>>& meshes,
                               std::vector<std::shared_ptr<Material>>& materials,
                               bool flip_uvs = true);

private:
    /// @brief Process Assimp node recursively
    static void process_node_(void* node, void* scene,
                             std::vector<std::shared_ptr<Mesh>>& meshes,
                             std::vector<std::shared_ptr<Material>>& materials,
                             const std::string& directory);
    
    /// @brief Process Assimp mesh
    static std::shared_ptr<Mesh> process_mesh_(void* mesh, void* scene,
                                              std::vector<std::shared_ptr<Material>>& materials,
                                              const std::string& directory);
    
    /// @brief Load material textures
    static void load_material_textures_(void* material, int type,
                                       std::shared_ptr<Material>& mat,
                                       const std::string& directory);
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_MODEL_LOADER_H
```

### 文件：include/resource/shader.h

```cpp
#ifndef ARE_INCLUDE_RESOURCE_SHADER_H
#define ARE_INCLUDE_RESOURCE_SHADER_H

#include "basic/types.h"
#include <string>
#include <unordered_map>

namespace are {

/// @brief Shader program resource
class Shader {
public:
    /// @brief Constructor
    Shader();
    
    /// @brief Destructor
    ~Shader();
    
    /// @brief Load and compile shader from files
    /// @param vertex_path Vertex shader path
    /// @param fragment_path Fragment shader path
    /// @return True if compilation succeeded
    bool load(const std::string& vertex_path, const std::string& fragment_path);
    
    /// @brief Load and compile compute shader
    /// @param compute_path Compute shader path
    /// @return True if compilation succeeded
    bool load_compute(const std::string& compute_path);
    
    /// @brief Compile shader from source strings
    /// @param vertex_source Vertex shader source
    /// @param fragment_source Fragment shader source
    /// @return True if compilation succeeded
    bool compile(const std::string& vertex_source, const std::string& fragment_source);
    
    /// @brief Compile compute shader from source
    /// @param compute_source Compute shader source
    /// @return True if compilation succeeded
    bool compile_compute(const std::string& compute_source);
    
    /// @brief Use/activate shader program
    void use() const;  // 改为const
    
    /// @brief Release shader resources
    void release();
    
    /// @brief Set uniform boolean
    /// @param name Uniform name
    /// @param value Value
    void set_bool(const std::string& name, bool value) const;  // 新增，const
    
    /// @brief Set uniform integer
    /// @param name Uniform name
    /// @param value Value
    void set_int(const std::string& name, int value) const;  // 改为const
    
    /// @brief Set uniform unsigned integer
    /// @param name Uniform name
    /// @param value Value
    void set_uint(const std::string& name, uint value) const;  // 改为const
    
    /// @brief Set uniform float
    /// @param name Uniform name
    /// @param value Value
    void set_float(const std::string& name, float value) const;  // 改为const
    
    /// @brief Set uniform vec2
    /// @param name Uniform name
    /// @param value Value
    void set_vec2(const std::string& name, const Vec2& value) const;  // 改为const
    
    /// @brief Set uniform vec3
    /// @param name Uniform name
    /// @param value Value
    void set_vec3(const std::string& name, const Vec3& value) const;  // 改为const
    
    /// @brief Set uniform vec4
    /// @param name Uniform name
    /// @param value Value
    void set_vec4(const std::string& name, const Vec4& value) const;  // 改为const
    
    /// @brief Set uniform mat3
    /// @param name Uniform name
    /// @param value Value
    void set_mat3(const std::string& name, const Mat3& value) const;  // 改为const
    
    /// @brief Set uniform mat4
    /// @param name Uniform name
    /// @param value Value
    void set_mat4(const std::string& name, const Mat4& value) const;  // 改为const
    
    /// @brief Get shader program handle
    /// @return Shader handle
    ShaderHandle get_handle() const { return handle_; }
    
    /// @brief Check if shader is valid
    /// @return True if valid
    bool is_valid() const { return handle_ != INVALID_HANDLE; }

private:
    ShaderHandle handle_;
    mutable std::unordered_map<std::string, int> uniform_cache_;  // 改为mutable
    
    /// @brief Get uniform location (with caching)
    /// @param name Uniform name
    /// @return Uniform location
    int get_uniform_location_(const std::string& name) const;  // 改为const
    
    /// @brief Compile shader stage
    /// @param source Shader source code
    /// @param type Shader type (GL_VERTEX_SHADER, etc.)
    /// @return Shader object handle (0 on failure)
    uint compile_shader_(const std::string& source, uint type);
    
    /// @brief Link shader program
    /// @param shaders Array of shader object handles
    /// @param count Number of shaders
    /// @return True if linking succeeded
    bool link_program_(const uint* shaders, uint count);
    
    /// @brief Read file content
    /// @param path File path
    /// @return File content
    std::string read_file_(const std::string& path);
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_SHADER_H
```

### 文件：include/resource/texture.h

```cpp
#ifndef ARE_INCLUDE_RESOURCE_TEXTURE_H
#define ARE_INCLUDE_RESOURCE_TEXTURE_H

#include "basic/types.h"
#include <string>

namespace are {

/// @brief Texture format enumeration
enum class TextureFormat {
    R8,
    RG8,
    RGB8,
    RGBA8,
    R16F,
    RG16F,
    RGB16F,
    RGBA16F,
    R32F,
    RG32F,
    RGB32F,
    RGBA32F,
    DEPTH24_STENCIL8
};

/// @brief Texture filter mode
enum class TextureFilter {
    NEAREST,
    LINEAR,
    NEAREST_MIPMAP_NEAREST,
    LINEAR_MIPMAP_NEAREST,
    NEAREST_MIPMAP_LINEAR,
    LINEAR_MIPMAP_LINEAR
};

/// @brief Texture wrap mode
enum class TextureWrap {
    REPEAT,
    MIRRORED_REPEAT,
    CLAMP_TO_EDGE,
    CLAMP_TO_BORDER
};

/// @brief Texture resource
class Texture {
public:
    /// @brief Constructor
    Texture();
    
    /// @brief Destructor
    ~Texture();
    
    /// @brief Load texture from file
    /// @param path File path
    /// @param generate_mipmaps Generate mipmaps
    /// @return True if loading succeeded
    bool load_from_file(const std::string& path, bool generate_mipmaps = true);
    
    /// @brief Create empty texture
    /// @param width Texture width
    /// @param height Texture height
    /// @param format Texture format
    /// @return True if creation succeeded
    bool create(uint width, uint height, TextureFormat format);
    
    /// @brief Upload data to texture
    /// @param data Pixel data
    /// @param width Data width
    /// @param height Data height
    /// @param format Data format
    /// @return True if upload succeeded
    bool upload(const void* data, uint width, uint height, TextureFormat format);
    
    /// @brief Set texture filter mode
    /// @param min_filter Minification filter
    /// @param mag_filter Magnification filter
    void set_filter(TextureFilter min_filter, TextureFilter mag_filter);
    
    /// @brief Set texture wrap mode
    /// @param wrap_s Wrap mode for S coordinate
    /// @param wrap_t Wrap mode for T coordinate
    void set_wrap(TextureWrap wrap_s, TextureWrap wrap_t);
    
    /// @brief Generate mipmaps
    void generate_mipmaps();
    
    /// @brief Bind texture to texture unit
    /// @param unit Texture unit
    void bind(uint unit) const;
    
    /// @brief Unbind texture
    void unbind() const;
    
    /// @brief Release texture resources
    void release();
    
    /// @brief Get texture handle
    /// @return Texture handle
    TextureHandle get_handle() const { return handle_; }
    
    /// @brief Get texture width
    /// @return Width
    uint get_width() const { return width_; }
    
    /// @brief Get texture height
    /// @return Height
    uint get_height() const { return height_; }
    
    /// @brief Get texture format
    /// @return Format
    TextureFormat get_format() const { return format_; }
    
    /// @brief Check if texture is valid
    /// @return True if valid
    bool is_valid() const { return handle_ != INVALID_HANDLE; }

private:
    TextureHandle handle_;
    uint width_;
    uint height_;
    TextureFormat format_;
    bool has_mipmaps_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_TEXTURE_H
```

### 文件：include/utils/config.h

```cpp
#ifndef ARE_INCLUDE_UTILS_CONFIG_H
#define ARE_INCLUDE_UTILS_CONFIG_H

#include <string>
#include <unordered_map>

namespace are {

/// @brief Configuration manager for loading engine settings
/// @note This module should be implemented by the user
class Config {
public:
    /// @brief Load configuration from file
    /// @param path Configuration file path
    /// @return True if loading succeeded
    static bool load(const std::string& path);
    
    /// @brief Save configuration to file
    /// @param path Configuration file path
    /// @return True if saving succeeded
    static bool save(const std::string& path);
    
    /// @brief Get string value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static std::string get_string(const std::string& key, const std::string& default_value = "");
    
    /// @brief Get integer value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static int get_int(const std::string& key, int default_value = 0);
    
    /// @brief Get float value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static float get_float(const std::string& key, float default_value = 0.0f);
    
    /// @brief Get boolean value
    /// @param key Configuration key
    /// @param default_value Default value if key not found
    /// @return Configuration value
    static bool get_bool(const std::string& key, bool default_value = false);
    
    /// @brief Set string value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_string(const std::string& key, const std::string& value);
    
    /// @brief Set integer value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_int(const std::string& key, int value);
    
    /// @brief Set float value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_float(const std::string& key, float value);
    
    /// @brief Set boolean value
    /// @param key Configuration key
    /// @param value Configuration value
    static void set_bool(const std::string& key, bool value);
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_CONFIG_H
```

### 文件：include/utils/logger.h

```cpp
#ifndef ARE_INCLUDE_UTILS_LOGGER_H
#define ARE_INCLUDE_UTILS_LOGGER_H

#include <string>

namespace are {

/// @brief Log level enumeration
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

/// @brief Logger interface for engine logging
/// @note This module should be implemented by the user
class Logger {
public:
    /// @brief Initialize logger
    /// @param log_file Log file path (empty for console only)
    /// @return True if initialization succeeded
    static bool initialize(const std::string& log_file = "");
    
    /// @brief Shutdown logger
    static void shutdown();
    
    /// @brief Log message
    /// @param level Log level
    /// @param message Message content
    static void log(LogLevel level, const std::string& message);
    
    /// @brief Log debug message
    /// @param message Message content
    static void debug(const std::string& message);
    
    /// @brief Log info message
    /// @param message Message content
    static void info(const std::string& message);
    
    /// @brief Log warning message
    /// @param message Message content
    static void warning(const std::string& message);
    
    /// @brief Log error message
    /// @param message Message content
    static void error(const std::string& message);
    
    /// @brief Log fatal message
    /// @param message Message content
    static void fatal(const std::string& message);
    
    /// @brief Set minimum log level
    /// @param level Minimum level to log
    static void set_level(LogLevel level);
};

} // namespace are

#endif // ARE_INCLUDE_UTILS_LOGGER_H
```

