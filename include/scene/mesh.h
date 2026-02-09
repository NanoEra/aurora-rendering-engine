#ifndef ARE_INCLUDE_SCENE_MESH_H
#define ARE_INCLUDE_SCENE_MESH_H

#include "basic/types.h"
#include <vector>

namespace are {

/// @brief Mesh data container
class Mesh {
public:
    /// @brief Constructor
    Mesh();
    
    /// @brief Destructor
    ~Mesh();
    
    /// @brief Set vertex data
    /// @param vertices Vertex array
    void set_vertices(const std::vector<Vertex>& vertices);
    
    /// @brief Set index data
    /// @param indices Index array
    void set_indices(const std::vector<uint>& indices);
    
    /// @brief Set material index
    /// @param material_id Material index
    void set_material(uint material_id);
    
    /// @brief Set transform matrix
    /// @param transform Transform matrix
    void set_transform(const Mat4& transform);
    
    /// @brief Get vertices
    /// @return Vertex array
    const std::vector<Vertex>& get_vertices() const { return vertices_; }
    
    /// @brief Get indices
    /// @return Index array
    const std::vector<uint>& get_indices() const { return indices_; }
    
    /// @brief Get material index
    /// @return Material index
    uint get_material() const { return material_id_; }
    
    /// @brief Get transform matrix
    /// @return Transform matrix
    const Mat4& get_transform() const { return transform_; }
    
    /// @brief Upload mesh data to GPU
    /// @return True if upload succeeded
    bool upload_to_gpu();
    
    /// @brief Release GPU resources
    void release_gpu_resources();
    
    /// @brief Get VAO handle
    /// @return VAO handle
    uint get_vao() const { return vao_; }
    
    /// @brief Check if mesh is uploaded to GPU
    /// @return True if uploaded
    bool is_uploaded() const { return uploaded_; }

private:
    std::vector<Vertex> vertices_;
    std::vector<uint> indices_;
    uint material_id_;
    Mat4 transform_;
    
    uint vao_;
    uint vbo_;
    uint ebo_;
    bool uploaded_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MESH_H
