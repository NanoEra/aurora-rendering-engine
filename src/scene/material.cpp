/**
 * @file material.cpp
 * @brief Implementation of PBR Material class
 */

#include <are/scene/material.h>
#include <are/core/logger.h>
#include <are/utils/math_utils.h>

namespace are {

Material::Material()
    : albedo_(1.0f, 1.0f, 1.0f)
    , metallic_(0.0f)
    , roughness_(0.5f)
    , emissive_(0.0f, 0.0f, 0.0f)
    , albedo_tex_handle_(are_invalid_handle)
    , metallic_tex_handle_(are_invalid_handle)
    , roughness_tex_handle_(are_invalid_handle)
    , normal_tex_handle_(are_invalid_handle)
    , ao_tex_handle_(are_invalid_handle)
    , emissive_tex_handle_(are_invalid_handle) {
}

void Material::set_albedo(const Vec3& albedo) {
    // Clamp albedo to valid range [0, 1]
    albedo_.x = clamp(albedo.x, 0.0f, 1.0f);
    albedo_.y = clamp(albedo.y, 0.0f, 1.0f);
    albedo_.z = clamp(albedo.z, 0.0f, 1.0f);
}

void Material::set_albedo_map(const std::string& path) {
    albedo_map_ = path;
}

void Material::set_metallic(Real metallic) {
    metallic_ = clamp(metallic, 0.0f, 1.0f);
}

void Material::set_metallic_map(const std::string& path) {
    metallic_map_ = path;
}

void Material::set_roughness(Real roughness) {
    // Clamp roughness to avoid division by zero in BRDF calculations
    roughness_ = clamp(roughness, 0.04f, 1.0f);
}

void Material::set_roughness_map(const std::string& path) {
    roughness_map_ = path;
}

void Material::set_normal_map(const std::string& path) {
    normal_map_ = path;
}

void Material::set_ao_map(const std::string& path) {
    ao_map_ = path;
}

void Material::set_emissive(const Vec3& emissive) {
    // Emissive can be HDR, so no upper clamp
    emissive_.x = std::max(0.0f, emissive.x);
    emissive_.y = std::max(0.0f, emissive.y);
    emissive_.z = std::max(0.0f, emissive.z);
}

void Material::set_emissive_map(const std::string& path) {
    emissive_map_ = path;
}

bool Material::is_emissive() const {
    // Check if material has significant emission
    const Real threshold = 0.001f;
    return (emissive_.x > threshold || 
            emissive_.y > threshold || 
            emissive_.z > threshold) ||
           has_emissive_map();
}

} // namespace are
