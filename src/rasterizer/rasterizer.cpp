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
    : gbuffer_(std::make_unique<GBuffer>(width, height))
    , gbuffer_shader_(std::make_unique<ShaderProgram>())
    , triangle_base_provider_(nullptr)
    , state_()
    , width_(width)
    , height_(height) {
}

Rasterizer::~Rasterizer() = default;

void Rasterizer::set_state(const RasterizerState& state) {
    state_ = state;
}

void Rasterizer::set_triangle_base_provider(std::function<uint32_t(size_t)> provider) {
    triangle_base_provider_ = std::move(provider);
}

void Rasterizer::resize(int width, int height) {
    ARE_PROFILE_FUNCTION();
    if (width == width_ && height == height_) return;
    width_ = width;
    height_ = height;
    gbuffer_->resize(width_, height_);
}

void Rasterizer::render_gbuffer(const SceneManager& scene, const Camera& camera) {
    ARE_PROFILE_FUNCTION();

    if (!gbuffer_shader_ || !gbuffer_shader_->is_valid()) {
        ARE_LOG_ERROR("Rasterizer: gbuffer shader not ready");
        return;
    }

    gbuffer_->bind();

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (state_.enable_depth_test) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
    } else {
        glDisable(GL_DEPTH_TEST);
    }

    if (state_.enable_cull_face) {
        glEnable(GL_CULL_FACE);
        glCullFace(static_cast<GLenum>(state_.cull_face_mode));
        glFrontFace(static_cast<GLenum>(state_.front_face));
    } else {
        glDisable(GL_CULL_FACE);
    }

    gbuffer_shader_->use();
    gbuffer_shader_->set_uniform("u_view", camera.get_view_matrix());
    gbuffer_shader_->set_uniform("u_projection", camera.get_projection_matrix());

    const auto& meshes = scene.get_all_meshes();

    for (size_t mi = 0; mi < meshes.size(); ++mi) {
        const auto& mesh = meshes[mi];
        if (mesh.is_empty() || !mesh.has_gpu_resources()) continue;

        Mat4 model = Mat4(1.0f);
        gbuffer_shader_->set_uniform("u_model", model);

        Mat3 normal_matrix = glm::inverseTranspose(Mat3(model));
        gbuffer_shader_->set_uniform("u_normal_matrix", normal_matrix);

        uint32_t tri_base = triangle_base_provider_ ? triangle_base_provider_(mi) : 0u;
        // IMPORTANT: u_triangle_id_base is uint in GLSL, must use glUniform1ui
        gbuffer_shader_->set_uniform("u_triangle_id_base", tri_base);

        const Material* mat = scene.get_material(mesh.get_material());
        if (mat) {
            gbuffer_shader_->set_uniform("u_albedo", mat->get_albedo());
            gbuffer_shader_->set_uniform("u_metallic", mat->get_metallic());
            gbuffer_shader_->set_uniform("u_roughness", mat->get_roughness());
        } else {
            gbuffer_shader_->set_uniform("u_albedo", Vec3(0.8f));
            gbuffer_shader_->set_uniform("u_metallic", 0.0f);
            gbuffer_shader_->set_uniform("u_roughness", 0.5f);
        }

        glBindVertexArray(mesh.get_vao());
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.get_index_count()), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    gbuffer_->unbind();
    ARE_GL_CHECK();
}

GBuffer& Rasterizer::get_gbuffer() { return *gbuffer_; }
const GBuffer& Rasterizer::get_gbuffer() const { return *gbuffer_; }

void Rasterizer::upload_mesh(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();
    if (mesh.is_empty()) {
        ARE_LOG_WARN("Rasterizer: upload_mesh on empty mesh");
        return;
    }
    if (mesh.has_gpu_resources()) delete_mesh(mesh);
    setup_mesh_buffers(mesh);
}

void Rasterizer::delete_mesh(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();

    uint32_t vao = mesh.get_vao();
    uint32_t vbo = mesh.get_vbo();
    uint32_t ebo = mesh.get_ebo();

    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (ebo) glDeleteBuffers(1, &ebo);

    mesh.set_vao(0);
    mesh.set_vbo(0);
    mesh.set_ebo(0);
}

void Rasterizer::initialize_shaders(const std::string& shader_dir) {
    ARE_PROFILE_FUNCTION();

    bool ok = true;
    ok &= gbuffer_shader_->load_shader(ShaderType::ARE_SHADER_VERTEX, shader_dir + "gbuffer/gbuffer.vert");
    ok &= gbuffer_shader_->load_shader(ShaderType::ARE_SHADER_FRAGMENT, shader_dir + "gbuffer/gbuffer.frag");
    ok &= gbuffer_shader_->link();

    if (!ok) {
        ARE_LOG_ERROR("Rasterizer: Failed to init gbuffer shaders");
    }
}

void Rasterizer::setup_mesh_buffers(Mesh& mesh) {
    ARE_PROFILE_FUNCTION();

    uint32_t vao = 0, vbo = 0, ebo = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, mesh.get_vertex_count() * sizeof(Vertex), mesh.get_vertices().data(), GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.get_index_count() * sizeof(uint32_t), mesh.get_indices().data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(get_position_offset()));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(get_normal_offset()));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(get_texcoord_offset()));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(get_tangent_offset()));

    glBindVertexArray(0);

    mesh.set_vao(vao);
    mesh.set_vbo(vbo);
    mesh.set_ebo(ebo);

    ARE_GL_CHECK();
}

} // namespace are
