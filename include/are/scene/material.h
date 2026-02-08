/**
 * @file material.h
 * @brief PBR material definition
 */

#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @class Material
 * @brief Physically-based rendering material
 * 
 * Supports standard PBR workflow with metallic-roughness model.
 */
class Material {
public:
    /**
     * @brief Default constructor - creates default white material
     */
    Material();

    // Albedo (base color)
    void set_albedo(const Vec3& albedo);
    void set_albedo_map(const std::string& path);
    const Vec3& get_albedo() const { return albedo_; }
    const std::string& get_albedo_map() const { return albedo_map_; }
    bool has_albedo_map() const { return !albedo_map_.empty(); }

    // Metallic
    void set_metallic(Real metallic);
    void set_metallic_map(const std::string& path);
    Real get_metallic() const { return metallic_; }
    const std::string& get_metallic_map() const { return metallic_map_; }
    bool has_metallic_map() const { return !metallic_map_.empty(); }

    // Roughness
    void set_roughness(Real roughness);
    void set_roughness_map(const std::string& path);
    Real get_roughness() const { return roughness_; }
    const std::string& get_roughness_map() const { return roughness_map_; }
    bool has_roughness_map() const { return !roughness_map_.empty(); }

    // Normal map
    void set_normal_map(const std::string& path);
    const std::string& get_normal_map() const { return normal_map_; }
    bool has_normal_map() const { return !normal_map_.empty(); }

    // Ambient occlusion
    void set_ao_map(const std::string& path);
    const std::string& get_ao_map() const { return ao_map_; }
    bool has_ao_map() const { return !ao_map_.empty(); }

    // Emissive
    void set_emissive(const Vec3& emissive);
    void set_emissive_map(const std::string& path);
    const Vec3& get_emissive() const { return emissive_; }
    const std::string& get_emissive_map() const { return emissive_map_; }
    bool has_emissive_map() const { return !emissive_map_.empty(); }
    bool is_emissive() const;

    // Texture handles (set by TextureManager)
    void set_albedo_texture_handle(TextureHandle handle) { albedo_tex_handle_ = handle; }
    void set_metallic_texture_handle(TextureHandle handle) { metallic_tex_handle_ = handle; }
    void set_roughness_texture_handle(TextureHandle handle) { roughness_tex_handle_ = handle; }
    void set_normal_texture_handle(TextureHandle handle) { normal_tex_handle_ = handle; }
    void set_ao_texture_handle(TextureHandle handle) { ao_tex_handle_ = handle; }
    void set_emissive_texture_handle(TextureHandle handle) { emissive_tex_handle_ = handle; }

    TextureHandle get_albedo_texture_handle() const { return albedo_tex_handle_; }
    TextureHandle get_metallic_texture_handle() const { return metallic_tex_handle_; }
    TextureHandle get_roughness_texture_handle() const { return roughness_tex_handle_; }
    TextureHandle get_normal_texture_handle() const { return normal_tex_handle_; }
    TextureHandle get_ao_texture_handle() const { return ao_tex_handle_; }
    TextureHandle get_emissive_texture_handle() const { return emissive_tex_handle_; }

private:
    // Base values
    Vec3 albedo_;///< Base color (RGB)
    Real metallic_;                       ///< Metallic factor [0, 1]
    Real roughness_;                      ///< Roughness factor [0, 1]
    Vec3 emissive_;                       ///< Emissive color (RGB)

    // Texture paths
    std::string albedo_map_;
    std::string metallic_map_;
    std::string roughness_map_;
    std::string normal_map_;
    std::string ao_map_;
    std::string emissive_map_;

    // Texture handles (GPU resources)
    TextureHandle albedo_tex_handle_;
    TextureHandle metallic_tex_handle_;
    TextureHandle roughness_tex_handle_;
    TextureHandle normal_tex_handle_;
    TextureHandle ao_tex_handle_;
    TextureHandle emissive_tex_handle_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MATERIAL_H
