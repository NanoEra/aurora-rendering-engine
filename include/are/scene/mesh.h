/**
 * @file mesh.h
 * @brief Mesh class for geometry storage
 */

#ifndef ARE_INCLUDE_SCENE_MESH_H
#define ARE_INCLUDE_SCENE_MESH_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>
#include <vector>

namespace are {

/**
 * @class Mesh
 * @brief Triangle mesh container
 * 
 * Stores vertex and index data for a triangle mesh.
 * Supports automatic AABB computation.
 */
class Mesh {
public:
    /**
     * @brief Default constructor - creates empty mesh
     */
    Mesh();

    /**
     * @brief Construct mesh from vertex and index data
     * @param vertices Vertex array
     * @param indices Index array (triangles)
     * @param material_id Material handle
     */
    Mesh(const std::vector<Vertex>& vertices, 
         const std::vector<uint32_t>& indices,
         MaterialHandle material_id = are_invalid_handle);

    /**
     * @brief Construct mesh from raw arrays
     * @param vertices Vertex array pointer
     * @param vertex_count Number of vertices
     * @param indices Index array pointer
     * @param index_count Number of indices
     * @param material_id Material handle
     */
    Mesh(const Vertex* vertices, size_t vertex_count,
         const uint32_t* indices, size_t index_count,
         MaterialHandle material_id = are_invalid_handle);

    // Data setters
    void set_vertices(const std::vector<Vertex>& vertices);
    void set_indices(const std::vector<uint32_t>& indices);
    void set_material(MaterialHandle material_id);

    // Data getters
    const std::vector<Vertex>& get_vertices() const { return vertices_; }
    const std::vector<uint32_t>& get_indices() const { return indices_; }
    MaterialHandle get_material() const { return material_id_; }

    // Geometry queries
    size_t get_vertex_count() const { return vertices_.size(); }
    size_t get_index_count() const { return indices_.size(); }
    size_t get_triangle_count() const { return indices_.size() / 3; }
    bool is_empty() const { return vertices_.empty() || indices_.empty(); }

    // AABB
    const AABB& get_aabb() const { return aabb_; }
    void compute_aabb();

    /**
     * @brief Compute tangent vectors for normal mapping
     */
    void compute_tangents();/**
     * @brief Get triangle vertices by index
     * @param triangle_index Triangle index
     * @param v0 Output first vertex
     * @param v1 Output second vertex
     * @param v2 Output third vertex
     * @return true if triangle exists
     */
    bool get_triangle(size_t triangle_index, Vertex& v0, Vertex& v1, Vertex& v2) const;

    // GPU resource handles (set by Renderer)
    void set_vao(uint32_t vao) { vao_ = vao; }
    void set_vbo(uint32_t vbo) { vbo_ = vbo; }
    void set_ebo(uint32_t ebo) { ebo_ = ebo; }
    uint32_t get_vao() const { return vao_; }
    uint32_t get_vbo() const { return vbo_; }
    uint32_t get_ebo() const { return ebo_; }
    bool has_gpu_resources() const { return vao_ != 0; }

private:
    std::vector<Vertex> vertices_;        ///< Vertex data
    std::vector<uint32_t> indices_;       ///< Index data (triangles)
    MaterialHandle material_id_;          ///< Associated material
    AABB aabb_;                           ///< Bounding box

    // GPU resources
    uint32_t vao_;                         ///< Vertex Array Object
    uint32_t vbo_;                         ///< Vertex Buffer Object
    uint32_t ebo_;                         ///< Element Buffer Object
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MESH_H
