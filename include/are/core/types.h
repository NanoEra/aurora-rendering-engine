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
