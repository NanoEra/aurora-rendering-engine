/**
 * @file rasterizer.cpp
 * @brief Implementation of Rasterizer class
 */

#include <are/rasterizer/rasterizer.h>
#include <are/rasterizer/gbuffer.h>
#include <are/rasterizer/shader_program.h>
#include <are/scene/scene_manager.h>
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/geometry/vertex.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/platform/gl_context.h>
#include <glad/glad.h>
#include <glm/gtc/matrix_inverse.hpp>

namespace are {

Rasterizer::Rasterizer(int width, int height)
    : width_(width)
    , height_(height) {
    ARE_PROFILE_FUNCTION();
    
    // Create G-Buffer
    gbuffer_ = std::make_unique<GBuffer>(width, height);
    
    // Create shader program
    gbuffer_shader_ = std::make_unique<ShaderProgram>();
    
    ARE_LOG_INFO("Rasterizer: Created " + std::to_string(width) + "x" + std::to_string(height));
}

Rasterizer::~Rasterizer() {
    ARE_LOG_INFO("Rasterizer: Destroyed");
}

void Rasterizer::resize(int width, int height) {
    ARE_PROFILE_FUNCTION();
    
    if (width == width_ && height == height_) {
        return;
    }
    
    width_ = width;
    height_ = height;
    
    if (gbuffer_) {
        gbuffer_->resize(width, height);
    }
    
    ARE_LOG_INFO("Rasterizer: Resized to " + std::to_string(width) + "x" + std::to_string(height));
}

void Rasterizer::render_gbuffer(const SceneManager& scene, const Camera& camera) {
    ARE_PROFILE_FUNCTION();
    
    if (!gbuffer_shader_ || !gbuffer_shader_->is_valid()) {
        ARE_LOG_ERROR("Rasterizer: G-Buffer shader is not valid");
        return;
    }
    
    // Bind G-Buffer for rendering
    gbuffer_->bind();
    
    // Clear buffers
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Enable face culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    
    // Use G-Buffer shader
    gbuffer_shader_->use();
    
    // Set view and projection matrices
    gbuffer_shader_->set_uniform("u_view", camera.get_view_matrix());
    gbuffer_shader_->set_uniform("u_projection", camera.get_projection_matrix());
    
    // Render all meshes
    const auto& meshes = scene.get_all_meshes();
    const auto& materials = scene.get_all_materials();
    
    for (const auto& mesh : meshes) {
        if (mesh.is_empty() || !mesh.has_gpu_resources()) {
            continue;
        }
        
        // Set model matrix (identity for now, can be extended with Transform)
        Mat4 model_matrix = Mat4(1.0f);
        gbuffer_shader_->set_uniform("u_model", model_matrix);
        
        // Calculate normal matrix
        Mat3 normal_matrix = glm::transpose(glm::inverse(Mat3(model_matrix)));
        gbuffer_shader_->set_uniform("u_normal_matrix", normal_matrix);
        
        // Set material properties
        MaterialHandle mat_handle = mesh.get_material();
        if (mat_handle != are_invalid_handle && mat_handle <= materials.size()) {
            const Material& material = materials[mat_handle - 1]; // Handle is 1-based
            gbuffer_shader_->set_uniform("u_albedo", material.get_albedo());
            gbuffer_shader_->set_uniform("u_metallic", material.get_metallic());
            gbuffer_shader_->set_uniform("u_roughness", material.get_roughness());
        } else {
            // Default material
            gbuffer_shader_->set_uniform("u_albedo", Vec3(0.8f, 0.8f, 0.8f));
            gbuffer_shader_->set_uniform("u_metallic", 0.0f);
            gbuffer_shader_->set_uniform("u_roughness", 0.5f);
        }
        
        // Draw mesh
        glBindVertexArray(mesh.get_vao());
        glDrawElements(GL_TRIANGLES, 
                       static_cast<GLsizei>(mesh.get_index_count()), 
                       GL_UNSIGNED_INT, 
                       nullptr);
        glBindVertexArray(0);
    }
    
    // Disable states
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    
    // Unbind G-Buffer
    gbuffer_->unbind();
    
    ARE_GL_CHECK();
}

GBuffer& Rasterizer::get_gbuffer() {
    return *gbuffer_;
}

const GBuffer& Rasterizer::get_gbuffer() const {
    return *gbuffer_;
}

void Rasterizer::upload_mesh(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();
    
    if (mesh.is_empty()) {
        ARE_LOG_WARN("Rasterizer: Attempting to upload empty mesh");
        return;
    }
    
    // Delete existing GPU resources if any
    if (mesh.has_gpu_resources()) {
        delete_mesh(mesh);
    }
    
    setup_mesh_buffers(mesh);
    
    ARE_LOG_DEBUG("Rasterizer: Uploaded mesh with " + 
                  std::to_string(mesh.get_vertex_count()) + " vertices, " +
                  std::to_string(mesh.get_triangle_count()) + " triangles");
}

void Rasterizer::delete_mesh(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();
    
    uint32_t vao = mesh.get_vao();
    uint32_t vbo = mesh.get_vbo();
    uint32_t ebo = mesh.get_ebo();
    
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
    }
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
    }
    if (ebo != 0) {
        glDeleteBuffers(1, &ebo);
    }
    
    mesh.set_vao(0);
    mesh.set_vbo(0);
    mesh.set_ebo(0);
}

void Rasterizer::initialize_shaders(const std::string& shader_dir) {
    ARE_PROFILE_FUNCTION();
    
    if (!gbuffer_shader_) {
        gbuffer_shader_ = std::make_unique<ShaderProgram>();
    }
    
    std::string vert_path = shader_dir + "gbuffer/gbuffer.vert";
    std::string frag_path = shader_dir + "gbuffer/gbuffer.frag";
    
    bool success = true;
    
    if (!gbuffer_shader_->load_shader(ShaderType::ARE_SHADER_VERTEX, vert_path)) {
        ARE_LOG_ERROR("Rasterizer: Failed to load vertex shader: " + vert_path);
        success = false;
    }
    
    if (!gbuffer_shader_->load_shader(ShaderType::ARE_SHADER_FRAGMENT, frag_path)) {
        ARE_LOG_ERROR("Rasterizer: Failed to load fragment shader: " + frag_path);
        success = false;
    }
    
    if (success && !gbuffer_shader_->link()) {
        ARE_LOG_ERROR("Rasterizer: Failed to link G-Buffer shader program");
        success = false;
    }
    
    if (success) {
        ARE_LOG_INFO("Rasterizer: Shaders initialized successfully");
    }
}

void Rasterizer::setup_mesh_buffers(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();
    
    uint32_t vao, vbo, ebo;
    
    // Create VAO
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    
    // Create VBO
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 
                 mesh.get_vertex_count() * sizeof(Vertex),
                 mesh.get_vertices().data(),
                 GL_STATIC_DRAW);
    
    // Create EBO
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 mesh.get_index_count() * sizeof(uint32_t),
                 mesh.get_indices().data(),
                 GL_STATIC_DRAW);
    
    // Setup vertex attributes
    // Position (location = 0)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                          sizeof(Vertex), 
                          reinterpret_cast<void*>(get_position_offset()));
    
    // Normal (location = 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 
                          sizeof(Vertex), 
                          reinterpret_cast<void*>(get_normal_offset()));
    
    // Texcoord (location = 2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 
                          sizeof(Vertex), 
                          reinterpret_cast<void*>(get_texcoord_offset()));
    
    // Tangent (location = 3)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 
                          sizeof(Vertex), 
                          reinterpret_cast<void*>(get_tangent_offset()));
    
    // Unbind VAO
    glBindVertexArray(0);
    
    // Store handles in mesh
    mesh.set_vao(vao);
    mesh.set_vbo(vbo);
    mesh.set_ebo(ebo);
    
    ARE_GL_CHECK();
}

} // namespace are
