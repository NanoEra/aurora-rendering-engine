/**
 * @file mesh.cpp
 * @brief Implementation of Mesh class
 */

#include <are/scene/mesh.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/utils/math_utils.h>
#include <glm/glm.hpp>

namespace are {

Mesh::Mesh()
    : material_id_(are_invalid_handle)
    , vao_(0)
    , vbo_(0)
    , ebo_(0) {
}

Mesh::Mesh(const std::vector<Vertex>& vertices, 
           const std::vector<uint32_t>& indices,
           MaterialHandle material_id)
    : vertices_(vertices)
    , indices_(indices)
    , material_id_(material_id)
    , vao_(0)
    , vbo_(0)
    , ebo_(0) {
    compute_aabb();
}

Mesh::Mesh(const Vertex* vertices, size_t vertex_count,
           const uint32_t* indices, size_t index_count,
           MaterialHandle material_id)
    : material_id_(material_id)
    , vao_(0)
    , vbo_(0)
    , ebo_(0) {
    if (vertices && vertex_count > 0) {
        vertices_.assign(vertices, vertices + vertex_count);
    }
    
    if (indices && index_count > 0) {
        indices_.assign(indices, indices + index_count);
    }
    
    compute_aabb();
}

void Mesh::set_vertices(const std::vector<Vertex>& vertices) {
    vertices_ = vertices;
    compute_aabb();
}

void Mesh::set_indices(const std::vector<uint32_t>& indices) {
    indices_ = indices;
}

void Mesh::set_material(MaterialHandle material_id) {
    material_id_ = material_id;
}

void Mesh::compute_aabb() {
    ARE_PROFILE_FUNCTION();
    
    if (vertices_.empty()) {
        aabb_ = AABB::invalid();
        return;
    }
    
    aabb_ = AABB(vertices_[0].position_);
    
    for (size_t i = 1; i < vertices_.size(); ++i) {
        aabb_.expand(vertices_[i].position_);
    }
}

void Mesh::compute_tangents() {
    ARE_PROFILE_FUNCTION();
    
    if (vertices_.empty() || indices_.empty()) {
        ARE_LOG_WARN("Mesh: Cannot compute tangents for empty mesh");
        return;
    }
    
    if (indices_.size() % 3 != 0) {
        ARE_LOG_ERROR("Mesh: Index count is not a multiple of 3");
        return;
    }
    
    // Initialize tangents to zero
    std::vector<Vec3> tangents(vertices_.size(), Vec3(0.0f));
    std::vector<Vec3> bitangents(vertices_.size(), Vec3(0.0f));
    
    // Calculate tangents for each triangle
    for (size_t i = 0; i < indices_.size(); i += 3) {
        uint32_t i0 = indices_[i];
        uint32_t i1 = indices_[i + 1];
        uint32_t i2 = indices_[i + 2];
        
        if (i0 >= vertices_.size() || i1 >= vertices_.size() || i2 >= vertices_.size()) {
            ARE_LOG_ERROR("Mesh: Invalid index in compute_tangents");
            continue;
        }
        
        const Vertex& v0 = vertices_[i0];
        const Vertex& v1 = vertices_[i1];
        const Vertex& v2 = vertices_[i2];
        
        // Calculate edges
        Vec3 edge1 = v1.position_ - v0.position_;
        Vec3 edge2 = v2.position_ - v0.position_;
        
        Vec2 delta_uv1 = v1.texcoord_ - v0.texcoord_;
        Vec2 delta_uv2 = v2.texcoord_ - v0.texcoord_;
        
        // Calculate tangent and bitangent
        Real f = delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y;
        
        if (std::abs(f) < are_epsilon) {
            // Degenerate UV coordinates, use arbitrary tangent
            Vec3 tangent, bitangent;
            create_orthonormal_basis(v0.normal_, tangent, bitangent);
            
            tangents[i0] += tangent;
            tangents[i1] += tangent;
            tangents[i2] += tangent;
            continue;
        }
        
        f = 1.0f / f;
        
        Vec3 tangent;
        tangent.x = f * (delta_uv2.y * edge1.x - delta_uv1.y * edge2.x);
        tangent.y = f * (delta_uv2.y * edge1.y - delta_uv1.y * edge2.y);
        tangent.z = f * (delta_uv2.y * edge1.z - delta_uv1.y * edge2.z);
        
        Vec3 bitangent;
        bitangent.x = f * (-delta_uv2.x * edge1.x + delta_uv1.x * edge2.x);
        bitangent.y = f * (-delta_uv2.x * edge1.y + delta_uv1.x * edge2.y);
        bitangent.z = f * (-delta_uv2.x * edge1.z + delta_uv1.x * edge2.z);
        
        // Accumulate tangents for each vertex
        tangents[i0] += tangent;
        tangents[i1] += tangent;
        tangents[i2] += tangent;
        
        bitangents[i0] += bitangent;
        bitangents[i1] += bitangent;
        bitangents[i2] += bitangent;
    }
    
    // Orthogonalize and normalize tangents (Gram-Schmidt)
    for (size_t i = 0; i < vertices_.size(); ++i) {
        const Vec3& n = vertices_[i].normal_;
        const Vec3& t = tangents[i];
        
        // Gram-Schmidt orthogonalize
        Vec3 tangent = t - n * glm::dot(n, t);
        
        Real length = glm::length(tangent);
        if (length < are_epsilon) {
            // If tangent is parallel to normal, create arbitrary tangent
            create_orthonormal_basis(n, tangent, bitangents[i]);
        } else {
            tangent /= length;
        }
        
        // Check handedness
        Real handedness = glm::dot(glm::cross(n, t), bitangents[i]);
        if (handedness < 0.0f) {
            tangent = -tangent;
        }
        
        vertices_[i].tangent_ = tangent;
    }
}

bool Mesh::get_triangle(size_t triangle_index, Vertex& v0, Vertex& v1, Vertex& v2) const {
    if (triangle_index >= get_triangle_count()) {
        return false;
    }
    
    size_t base_index = triangle_index * 3;
    uint32_t i0 = indices_[base_index];
    uint32_t i1 = indices_[base_index + 1];
    uint32_t i2 = indices_[base_index + 2];
    
    if (i0 >= vertices_.size() || i1 >= vertices_.size() || i2 >= vertices_.size()) {
        ARE_LOG_ERROR("Mesh: Invalid indices in get_triangle");
        return false;
    }
    
    v0 = vertices_[i0];
    v1 = vertices_[i1];
    v2 = vertices_[i2];
    
    return true;
}

} // namespace are
