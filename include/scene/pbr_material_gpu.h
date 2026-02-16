#ifndef ARE_INCLUDE_SCENE_PBR_MATERIAL_GPU_H
#define ARE_INCLUDE_SCENE_PBR_MATERIAL_GPU_H

#include "basic/types.h"

namespace are {

/**
 * @brief PBR material feature flags
 */
enum class PbrMaterialFlags : uint {
	NONE = 0u,
	HAS_BASE_COLOR_TEX = 1u << 0,
	HAS_NORMAL_TEX = 1u << 1,
	HAS_METAL_ROUGH_TEX = 1u << 2,
	HAS_EMISSIVE_TEX = 1u << 3,
	DOUBLE_SIDED = 1u << 4,
	ALPHA_MASK = 1u << 5,
	ALPHA_BLEND = 1u << 6
};

inline PbrMaterialFlags operator|(PbrMaterialFlags a, PbrMaterialFlags b) {
	return static_cast<PbrMaterialFlags>(static_cast<uint>(a) | static_cast<uint>(b));
}

inline uint to_uint(PbrMaterialFlags f) {
	return static_cast<uint>(f);
}

/**
 * @brief GPU-friendly PBR material (std430 aligned by vec4/uvec4)
 * @note All fields are designed to be consumed by GLSL std430 without padding issues.
 */
struct PbrMaterialGpu {
	Vec4 base_color_factor_; ///< rgb = baseColor, a = alpha
	Vec4 emissive_factor_; ///< rgb = emissive, a = unused
	Vec4 mr_normal_flags_; ///< x=metallic, y=roughness, z=normal_scale, w=flags (uint bits in float)

	// texture layer indices (per texture array). TEX_INVALID if none.
	glm::uvec4 tex0_; ///< x=baseColor, y=normal, z=metalRough, w=emissive
	glm::uvec4 tex1_; ///< reserved
};

constexpr uint TEX_INVALID = 0xFFFFFFFFu;

} // namespace are

#endif // ARE_INCLUDE_SCENE_PBR_MATERIAL_GPU_H
