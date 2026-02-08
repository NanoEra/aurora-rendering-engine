/**
 * @file material.cpp
 * @brief Implementation of PBR material
 */

#include <are/scene/material.h>

namespace are {

Material::Material()
    : albedo_(0.8f, 0.8f, 0.8f)
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
    albedo_ = albedo;
}

void Material::set_albedo_map(const std::string& path) {
    albedo_map_ = path;
}

void Material::set_metallic(Real metallic) {
    metallic_ = glm::clamp(metallic, 0.0f, 1.0f);
}

void Material::set_metallic_map(const std::string& path) {
    metallic_map_ = path;
}

void Material::set_roughness(Real roughness) {
    roughness_ = glm::clamp(roughness, 0.0f, 1.0f);
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
    emissive_ = emissive;
}

void Material::set_emissive_map(const std::string& path) {
    emissive_map_ = path;
}

bool Material::is_emissive() const {
    return glm::length(emissive_) > are_epsilon || !emissive_map_.empty();
}

} // namespace are
