#ifndef ARE_INCLUDE_BASIC_TYPES_H
#define ARE_INCLUDE_BASIC_TYPES_H

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace are {

// Basic vector types using GLM
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;

// Basic matrix types using GLM
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;

// Basic integer types
using uint = uint32_t;
using uchar = uint8_t;

// Handle types for GPU resources
using TextureHandle = uint;
using BufferHandle = uint;
using ShaderHandle = uint;
using FramebufferHandle = uint;
using VertexArrayHandle = uint;

// Invalid handle constant
constexpr uint INVALID_HANDLE = 0;

// Vertex structure for mesh data
struct Vertex {
	Vec3 position_;
	Vec3 normal_;
	Vec2 texcoord_;
	Vec3 tangent_;
};

// Ray structure for ray tracing
struct Ray {
	Vec3 origin_;
	Vec3 direction_;
	float t_min_;
	float t_max_;
};

// Hit information for ray-surface intersection
struct HitInfo {
	bool hit_;
	float t_;
	Vec3 position_;
	Vec3 normal_;
	Vec2 texcoord_;
	uint material_id_;
};

// Rendering statistics
struct RenderStats {
	float frame_time_ms_;
	uint triangle_count_;
	uint ray_count_;
	float gbuffer_time_ms_;
	float raytrace_time_ms_;
};

} // namespace are

#endif // ARE_INCLUDE_BASIC_TYPES_H
