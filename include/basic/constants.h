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
constexpr int GBUFFER_MATERIAL = 3;
constexpr int GBUFFER_MATERIAL_ID = 4;
constexpr int GBUFFER_COUNT = 5;

/// @brief Compute shader work group size
constexpr int COMPUTE_GROUP_SIZE_X = 16;
constexpr int COMPUTE_GROUP_SIZE_Y = 16;

/// @brief Mathematical constants
constexpr float PI = 3.14159265359f;
constexpr float INV_PI = 0.31830988618f;
constexpr float EPSILON = 1e-4f;

} // namespace are

#endif // ARE_INCLUDE_BASIC_CONSTANTS_H
