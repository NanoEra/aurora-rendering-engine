/**
 * @file scene_manager.h
 * @brief Scene data management
 */

#ifndef ARE_INCLUDE_SCENE_SCENE_MANAGER_H
#define ARE_INCLUDE_SCENE_SCENE_MANAGER_H

#include <are/core/types.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <vector>
#include <memory>
#include <unordered_map>

namespace are {

/**
 * @class SceneManager
 * @brief Manages all scene objects (meshes, materials, lights)
 * 
 * Provides handle-based access to scene resources and tracks
 * scene modifications for BVH rebuilding.
 */
class SceneManager {
public:
    /**
     * @brief Constructor
     */
    SceneManager();

    /**
     * @brief Destructor
     */
    ~SceneManager();

    // Mesh management
    MeshHandle add_mesh(const Mesh& mesh);
    void remove_mesh(MeshHandle handle);
    void update_mesh(MeshHandle handle, const Mesh& mesh);
    Mesh* get_mesh(MeshHandle handle);
    const Mesh* get_mesh(MeshHandle handle) const;
    const std::vector<Mesh>& get_all_meshes() const { return meshes_; }

    // Material management
    MaterialHandle add_material(const Material& material);
    void remove_material(MaterialHandle handle);
    void update_material(MaterialHandle handle, const Material& material);
    Material* get_material(MaterialHandle handle);
    const Material* get_material(MaterialHandle handle) const;
    const std::vector<Material>& get_all_materials() const { return materials_; }

    // Light management
    LightHandle add_light(const std::shared_ptr<Light>& light);
    void remove_light(LightHandle handle);
    std::shared_ptr<Light> get_light(LightHandle handle);
    const std::vector<std::shared_ptr<Light>>& get_all_lights() const { return lights_; }

    // Scene queries
    size_t get_mesh_count() const { return meshes_.size(); }
    size_t get_material_count() const { return materials_.size(); }
    size_t get_light_count() const { return lights_.size(); }
    size_t get_total_triangle_count() const;

    // Scene state
    bool is_dirty() const { return dirty_; }
    void mark_dirty() { dirty_ = true; }
    void clear_dirty() { dirty_ = false; }

    /**
     * @brief Clear all scene data
     */
    void clear();

    /**
     * @brief Validate all handles and remove invalid entries
     */
    void compact();

private:
    std::vector<Mesh> meshes_;            ///< Mesh storage
    std::vector<Material> materials_;     ///< Material storage
    std::vector<std::shared_ptr<Light>> lights_; ///< Light storage

    std::unordered_map<MeshHandle, size_t> mesh_handle_map_;
    std::unordered_map<MaterialHandle, size_t> material_handle_map_;
    std::unordered_map<LightHandle, size_t> light_handle_map_;

    MeshHandle next_mesh_handle_;
    MaterialHandle next_material_handle_;
    LightHandle next_light_handle_;

    bool dirty_;                          ///< Scene modified flag
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SCENE_MANAGER_H
