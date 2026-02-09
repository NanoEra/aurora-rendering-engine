#include "scene/material.h"

namespace are {

Material::Material()
    : albedo_(1.0f, 1.0f, 1.0f)
    , emission_(0.0f, 0.0f, 0.0f)
    , metallic_(0.0f)
    , roughness_(0.5f)
    , ior_(1.5f)
    , type_(MaterialType::DIFFUSE)
    , albedo_texture_(nullptr)
    , normal_texture_(nullptr) {
}

Material::~Material() {
}

void Material::set_albedo(const Vec3& albedo) {
    albedo_ = albedo;
}

void Material::set_emission(const Vec3& emission) {
    emission_ = emission;
}

void Material::set_metallic(float metallic) {
    metallic_ = glm::clamp(metallic, 0.0f, 1.0f);
}

void Material::set_roughness(float roughness) {
    roughness_ = glm::clamp(roughness, 0.0f, 1.0f);
}

void Material::set_ior(float ior) {
    ior_ = ior;
}

void Material::set_type(MaterialType type) {
    type_ = type;
}

void Material::set_albedo_texture(std::shared_ptr<Texture> texture) {
    albedo_texture_ = texture;
}

void Material::set_normal_texture(std::shared_ptr<Texture> texture) {
    normal_texture_ = texture;
}

} // namespace are
