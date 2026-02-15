#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include "basic/types.h"
#include "resource/texture.h"
#include <memory>

namespace are {

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
		return albedo_texture_;
	}

	/*
	 * @brief Get normal texture
	 * @return Normal texture (nullptr if none)
	 */
	std::shared_ptr<Texture> get_normal_texture() const {
		return normal_texture_;
	}

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
