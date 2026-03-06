#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include "basic/types.h"
#include "resource/texture.h"
#include <array>
#include <memory>

namespace are {

// Texture slot enumeration for PBR materials
enum class TextureSlot {
	ALBEDO = 0,
	NORMAL = 1,
	METALLIC = 2,
	ROUGHNESS = 3,
	AO = 4,
	EMISSION = 5,
	COUNT = 6
};

// Material type enumeration
enum class MaterialType {
	DIFFUSE = 0,
	METAL = 1,
	DIELECTRIC = 2,
	EMISSIVE = 3
};

// Material properties
class Material {
public:
	// Constructor
	Material();

	// Destructor
	~Material();

	/*
	 * @brief Set albedo color
	 * @param albedo Albedo color
	 */
	void set_albedo(const Vec3 &albedo);

	/*
	 * @brief Set emission color
	 * @param emission Emission color
	 */
	void set_emission(const Vec3 &emission);

	/*
	 * @brief Set metallic value
	 * @param metallic Metallic (0-1)
	 */
	void set_metallic(float metallic);

	/*
	 * @brief Set roughness value
	 * @param roughness Roughness (0-1)
	 */
	void set_roughness(float roughness);

	/*
	 * @brief Set index of refraction
	 * @param ior Index of refraction
	 */
	void set_ior(float ior);

	/*
	 * @brief Set material type
	 * @param type Material type
	 */
	void set_type(MaterialType type);

	/*
	 * @brief Set albedo texture
	 * @param texture Albedo texture
	 */
	void set_albedo_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set normal map
	 * @param texture Normal map texture
	 */
	void set_normal_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set metallic map
	 * @param texture Metallic texture (R channel)
	 */
	void set_metallic_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set roughness map
	 * @param texture Roughness texture (G channel)
	 */
	void set_roughness_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set ambient occlusion map
	 * @param texture AO texture (R channel)
	 */
	void set_ao_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set emission map
	 * @param texture Emission texture (RGB)
	 */
	void set_emission_texture(std::shared_ptr<Texture> texture);

	/*
	 * @brief Set texture for a specific slot
	 * @param slot Texture slot
	 * @param texture Texture to set
	 */
	void set_texture(TextureSlot slot, std::shared_ptr<Texture> texture);

	/*
	 * @brief Get albedo color
	 * @return Albedo color
	 */
	const Vec3 &get_albedo() const {
		return albedo_;
	}

	/*
	 * @brief Get emission color
	 * @return Emission color
	 */
	const Vec3 &get_emission() const {
		return emission_;
	}

	/*
	 * @brief Get metallic value
	 * @return Metallic
	 */
	float get_metallic() const {
		return metallic_;
	}

	/*
	 * @brief Get roughness value
	 * @return Roughness
	 */
	float get_roughness() const {
		return roughness_;
	}

	/*
	 * @brief Get index of refraction
	 * @return IOR
	 */
	float get_ior() const {
		return ior_;
	}

	/*
	 * @brief Get material type
	 * @return Material type
	 */
	MaterialType get_type() const {
		return type_;
	}

	/*
	 * @brief Get albedo texture
	 * @return Albedo texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_albedo_texture() const {
		return textures_[static_cast<int>(TextureSlot::ALBEDO)];
	}

	/*
	 * @brief Get normal texture
	 * @return Normal texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_normal_texture() const {
		return textures_[static_cast<int>(TextureSlot::NORMAL)];
	}

	/*
	 * @brief Get metallic texture
	 * @return Metallic texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_metallic_texture() const {
		return textures_[static_cast<int>(TextureSlot::METALLIC)];
	}

	/*
	 * @brief Get roughness texture
	 * @return Roughness texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_roughness_texture() const {
		return textures_[static_cast<int>(TextureSlot::ROUGHNESS)];
	}

	/*
	 * @brief Get AO texture
	 * @return AO texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_ao_texture() const {
		return textures_[static_cast<int>(TextureSlot::AO)];
	}

	/*
	 * @brief Get emission texture
	 * @return Emission texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_emission_texture() const {
		return textures_[static_cast<int>(TextureSlot::EMISSION)];
	}

	/*
	 * @brief Get texture for a specific slot
	 * @param slot Texture slot
	 * @return Texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_texture(TextureSlot slot) const {
		return textures_[static_cast<int>(slot)];
	}

	/*
	 * @brief Check if material has texture for slot
	 * @param slot Texture slot
	 * @return True if texture exists
	 */
	bool has_texture(TextureSlot slot) const {
		return textures_[static_cast<int>(slot)] != nullptr;
	}

	/*
	 * @brief Set texture array index for a slot (used by RayTracer)
	 * @param slot Texture slot
	 * @param index Index into the texture array
	 */
	void set_texture_index(TextureSlot slot, uint32_t index) {
		texture_indices_[static_cast<int>(slot)] = index;
	}

	/*
	 * @brief Get texture array index for a slot
	 * @param slot Texture slot
	 * @return Index into the texture array (0 = no texture)
	 */
	uint32_t get_texture_index(TextureSlot slot) const {
		return texture_indices_[static_cast<int>(slot)];
	}

private:
	Vec3 albedo_;
	Vec3 emission_;
	float metallic_;
	float roughness_;
	float ior_;
	MaterialType type_;

	std::array<std::shared_ptr<Texture>, static_cast<int>(TextureSlot::COUNT)> textures_;
	std::array<uint32_t, static_cast<int>(TextureSlot::COUNT)> texture_indices_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MATERIAL_H
