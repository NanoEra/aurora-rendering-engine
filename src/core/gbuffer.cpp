#include "core/gbuffer.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

GBuffer::GBuffer(uint width, uint height)
    : width_(width)
    , height_(height)
    , fbo_(INVALID_HANDLE)
    , depth_texture_(INVALID_HANDLE)
    , initialized_(false) {
    for (int i = 0; i < GBUFFER_COUNT; ++i) {
        textures_[i] = INVALID_HANDLE;
    }
}

GBuffer::~GBuffer() {
    release();
}

bool GBuffer::initialize() {
    if (initialized_) {
        ARE_LOG_WARN("GBuffer already initialized");
        return true;
    }

    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    textures_[GBUFFER_POSITION] = create_texture_(GL_RGBA32F, GL_RGBA, GL_FLOAT);
    textures_[GBUFFER_NORMAL]   = create_texture_(GL_RGBA32F, GL_RGBA, GL_FLOAT);
    textures_[GBUFFER_ALBEDO]   = create_texture_(GL_RGBA8,   GL_RGBA, GL_UNSIGNED_BYTE);

    // New: material params (metallic, roughness, ior, type)
    textures_[GBUFFER_MATERIAL] = create_texture_(GL_RGBA32F, GL_RGBA, GL_FLOAT);

    // New: material id (integer)
    textures_[GBUFFER_MATERIAL_ID] = create_texture_(GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_POSITION,
                           GL_TEXTURE_2D, textures_[GBUFFER_POSITION], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_NORMAL,
                           GL_TEXTURE_2D, textures_[GBUFFER_NORMAL], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_ALBEDO,
                           GL_TEXTURE_2D, textures_[GBUFFER_ALBEDO], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_MATERIAL,
                           GL_TEXTURE_2D, textures_[GBUFFER_MATERIAL], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_MATERIAL_ID,
                           GL_TEXTURE_2D, textures_[GBUFFER_MATERIAL_ID], 0);

    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, width_, height_, 0,
                 GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                           GL_TEXTURE_2D, depth_texture_, 0);

    GLenum draw_buffers[GBUFFER_COUNT] = {
        GL_COLOR_ATTACHMENT0 + GBUFFER_POSITION,
        GL_COLOR_ATTACHMENT0 + GBUFFER_NORMAL,
        GL_COLOR_ATTACHMENT0 + GBUFFER_ALBEDO,
        GL_COLOR_ATTACHMENT0 + GBUFFER_MATERIAL,
        GL_COLOR_ATTACHMENT0 + GBUFFER_MATERIAL_ID
    };
    glDrawBuffers(GBUFFER_COUNT, draw_buffers);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        ARE_LOG_ERROR("GBuffer framebuffer is not complete");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    initialized_ = true;
    ARE_LOG_INFO("GBuffer initialized successfully");
    return true;
}

void GBuffer::release() {
    if (!initialized_) return;

    if (fbo_ != INVALID_HANDLE) {
        glDeleteFramebuffers(1, &fbo_);
        fbo_ = INVALID_HANDLE;
    }

    for (int i = 0; i < GBUFFER_COUNT; ++i) {
        if (textures_[i] != INVALID_HANDLE) {
            glDeleteTextures(1, &textures_[i]);
            textures_[i] = INVALID_HANDLE;
        }
    }

    if (depth_texture_ != INVALID_HANDLE) {
        glDeleteTextures(1, &depth_texture_);
        depth_texture_ = INVALID_HANDLE;
    }

    initialized_ = false;
    ARE_LOG_INFO("GBuffer released");
}

TextureHandle GBuffer::create_texture_(uint internal_format, uint format, uint type) {
    TextureHandle texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width_, height_, 0, format, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return texture;
}

void GBuffer::render(const Scene& scene, const Shader& shader) {
    if (!initialized_) {
        ARE_LOG_ERROR("GBuffer not initialized");
        return;
    }
    
    // Bind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, width_, height_);
    
    // Clear buffers
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Use shader
    shader.use();
    
    // Set camera matrices
    const Camera& camera = scene.get_camera();
    Mat4 view = camera.get_view_matrix();
    Mat4 projection = camera.get_projection_matrix();
    
    shader.set_mat4("u_view", view);
    shader.set_mat4("u_projection", projection);
    
    // Render all meshes
    const auto& meshes = scene.get_meshes();
    const auto& materials = scene.get_materials();
    
    for (const auto& mesh : meshes) {
        if (!mesh->is_uploaded()) {
            ARE_LOG_WARN("Mesh not uploaded to GPU, skipping");
            continue;
        }
        
        // Set model matrix
        Mat4 model = mesh->get_transform();
        shader.set_mat4("u_model", model);
        
        // Set material properties
        uint material_id = mesh->get_material();
        if (material_id < materials.size()) {
            const auto& material = materials[material_id];
            
            shader.set_vec3("u_albedo", material->get_albedo());
            shader.set_float("u_metallic", material->get_metallic());
            shader.set_float("u_roughness", material->get_roughness());
            shader.set_uint("u_material_id", material_id);
			shader.set_float("u_ior", material->get_ior());
			shader.set_vec3("u_emission", material->get_emission());
			shader.set_uint("u_material_type", static_cast<uint>(material->get_type()));
            
            // Bind textures
            auto albedo_tex = material->get_albedo_texture();
            if (albedo_tex && albedo_tex->is_valid()) {
                albedo_tex->bind(0);
                shader.set_int("u_albedo_map", 0);
                shader.set_int("u_has_albedo_map", 1);
            } else {
                shader.set_int("u_has_albedo_map", 0);
            }
            
            auto normal_tex = material->get_normal_texture();
            if (normal_tex && normal_tex->is_valid()) {
                normal_tex->bind(1);
                shader.set_int("u_normal_map", 1);
                shader.set_int("u_has_normal_map", 1);
            } else {
                shader.set_int("u_has_normal_map", 0);
            }
        }
        
        // Draw mesh
        glBindVertexArray(mesh->get_vao());
        glDrawElements(GL_TRIANGLES, mesh->get_indices().size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    
    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::resize(uint width, uint height) {
    if (width == width_ && height == height_) return;
    
    width_ = width;
    height_ = height;
    
    if (initialized_) {
        release();
        initialize();
    }
}

TextureHandle GBuffer::get_texture(int index) const {
    if (index < 0 || index >= GBUFFER_COUNT) {
        ARE_LOG_ERROR("Invalid G-Buffer texture index");
        return INVALID_HANDLE;
    }
    return textures_[index];
}

void GBuffer::get_dimensions(uint& width, uint& height) const {
    width = width_;
    height = height_;
}

} // namespace are
