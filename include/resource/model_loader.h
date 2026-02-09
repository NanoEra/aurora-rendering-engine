#ifndef ARE_INCLUDE_RESOURCE_MODEL_LOADER_H
#define ARE_INCLUDE_RESOURCE_MODEL_LOADER_H

#include "basic/types.h"
#include "scene/mesh.h"
#include "scene/material.h"
#include <string>
#include <vector>
#include <memory>

namespace are {

/// @brief Model loader using Assimp
class ModelLoader {
public:
    /// @brief Load model from file
    /// @param path Model file path
    /// @param meshes Output mesh list
    /// @param materials Output material list
    /// @param flip_uvs Flip UV coordinates vertically
    /// @return True if loading succeeded
    static bool load(const std::string& path,
                    std::vector<std::shared_ptr<Mesh>>& meshes,
                    std::vector<std::shared_ptr<Material>>& materials,
                    bool flip_uvs = true);
    
    /// @brief Load model and automatically upload to GPU
    /// @param path Model file path
    /// @param meshes Output mesh list
    /// @param materials Output material list
    /// @param flip_uvs Flip UV coordinates vertically
    /// @return True if loading succeeded
    static bool load_and_upload(const std::string& path,
                               std::vector<std::shared_ptr<Mesh>>& meshes,
                               std::vector<std::shared_ptr<Material>>& materials,
                               bool flip_uvs = true);

private:
    /// @brief Process Assimp node recursively
    static void process_node_(void* node, void* scene,
                             std::vector<std::shared_ptr<Mesh>>& meshes,
                             std::vector<std::shared_ptr<Material>>& materials,
                             const std::string& directory);
    
    /// @brief Process Assimp mesh
    static std::shared_ptr<Mesh> process_mesh_(void* mesh, void* scene,
                                              std::vector<std::shared_ptr<Material>>& materials,
                                              const std::string& directory);
    
    /// @brief Load material textures
    static void load_material_textures_(void* material, int type,
                                       std::shared_ptr<Material>& mat,
                                       const std::string& directory);
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_MODEL_LOADER_H
