#include "scene/material.h"
#include <array>

namespace are {

Material::Material()
	: albedo_(1.0f, 1.0f, 1.0f)
	, emission_(0.0f, 0.0f, 0.0f)
	, metallic_(0.0f)
	, roughness_(0.5f)
	, ior_(1.5f)
	, type_(MaterialType::DIFFUSE)
	, textures_() {
}

Material::~Material() {
}

void Material::set_albedo(const Vec3 &albedo) {
	albedo_ = albedo;
}

void Material::set_emission(const Vec3 &emission) {
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
	textures_[static_cast<int>(TextureSlot::ALBEDO)] = texture;
}

void Material::set_normal_texture(std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(TextureSlot::NORMAL)] = texture;
}

void Material::set_metallic_texture(std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(TextureSlot::METALLIC)] = texture;
}

void Material::set_roughness_texture(std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(TextureSlot::ROUGHNESS)] = texture;
}

void Material::set_ao_texture(std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(TextureSlot::AO)] = texture;
}

void Material::set_emission_texture(std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(TextureSlot::EMISSION)] = texture;
}

void Material::set_texture(TextureSlot slot, std::shared_ptr<Texture> texture) {
	textures_[static_cast<int>(slot)] = texture;
}

} // namespace are
