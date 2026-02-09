#include "scene/mesh.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

Mesh::Mesh()
    : material_id_(0)
    , transform_(1.0f)
    , vao_(0)
    , vbo_(0)
    , ebo_(0)
    , uploaded_(false) {
}

Mesh::~Mesh() {
    release_gpu_resources();
}

void Mesh::set_vertices(const std::vector<Vertex>& vertices) {
    vertices_ = vertices;
    uploaded_ = false;
}

void Mesh::set_indices(const std::vector<uint>& indices) {
    indices_ = indices;
    uploaded_ = false;
}

void Mesh::set_material(uint material_id) {
    material_id_ = material_id;
}

void Mesh::set_transform(const Mat4& transform) {
    transform_ = transform;
}

bool Mesh::upload_to_gpu() {
    if (uploaded_) {
        Logger::warning("Mesh already uploaded to GPU");
        return true;
    }
    
    if (vertices_.empty()) {
        Logger::error("Cannot upload mesh: no vertices");
        return false;
    }
    
    if (indices_.empty()) {
        Logger::error("Cannot upload mesh: no indices");
        return false;
    }
    
    // Generate VAO
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    
    // Generate and upload VBO
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), 
                vertices_.data(), GL_STATIC_DRAW);
    
    // Generate and upload EBO
    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint),
                indices_.data(), GL_STATIC_DRAW);
    
    // Set vertex attributes
    // Location 0: Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, position_));
    
    // Location 1: Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, normal_));
    
    // Location 2: TexCoord
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, texcoord_));
    
    // Location 3: Tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, tangent_));
    
    glBindVertexArray(0);
    
    uploaded_ = true;
    Logger::info("Mesh uploaded to GPU successfully");
    return true;
}

void Mesh::release_gpu_resources() {
    if (!uploaded_) return;
    
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    
    if (ebo_ != 0) {
        glDeleteBuffers(1, &ebo_);
        ebo_ = 0;
    }
    
    uploaded_ = false;
}

} // namespace are
