#include "core/screen_blit.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

ScreenBlit::ScreenBlit()
    : vao_(0)
    , vbo_(0)
    , initialized_(false) {
}

ScreenBlit::~ScreenBlit() {
    release();
}

bool ScreenBlit::initialize(const std::shared_ptr<Shader> &screen_blit_shader) {
    if (initialized_) {
        ARE_LOG_WARN("ScreenBlit already initialized");
        return true;
    }

	shader_ = screen_blit_shader;
    
    // Create fullscreen quad
    create_quad_();
    
    initialized_ = true;
    ARE_LOG_INFO("ScreenBlit initialized successfully");
    return true;
}

void ScreenBlit::release() {
    if (!initialized_) return;
    
	shader_.reset();
    
    ResourceManager &rm = ResourceManager::instance();
    
    if (vao_ != 0) {
        rm.destroy_vertex_array(vao_);
        vao_ = 0;
    }
    
    if (vbo_ != 0) {
        rm.destroy_buffer(vbo_);
        vbo_ = 0;
    }
    
    initialized_ = false;
}

void ScreenBlit::blit(TextureHandle texture, int x, int y, uint width, uint height) {
    if (!initialized_) {
        ARE_LOG_ERROR("ScreenBlit not initialized");
        return;
    }
    
    // Set viewport
    glViewport(x, y, width, height);
    
    // Disable depth test
    glDisable(GL_DEPTH_TEST);
    
    // Use shader
    shader_->use();
    shader_->set_int("u_texture", 0);
    
    // Bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    
    // Draw quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    // Re-enable depth test
    glEnable(GL_DEPTH_TEST);
}

void ScreenBlit::blit_fullscreen(TextureHandle texture) {
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    blit(texture, viewport[0], viewport[1], viewport[2], viewport[3]);
}

void ScreenBlit::create_quad_() {
    // Fullscreen quad vertices (position + texcoord)
    float vertices[] = {
        // Position    // TexCoord
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        -1.0f,  1.0f,  0.0f, 1.0f
    };
    
    ResourceManager &rm = ResourceManager::instance();
    
    vao_ = rm.create_vertex_array();
    glBindVertexArray(vao_);
    
    BufferDescription vbo_desc;
    vbo_desc.type = BufferType::VERTEX_BUFFER;
    vbo_desc.usage = BufferUsage::STATIC_DRAW;
    vbo_desc.size = sizeof(vertices);
    vbo_desc.data = vertices;
    vbo_ = rm.create_buffer(vbo_desc);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    
    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    
    // TexCoord attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    
    glBindVertexArray(0);
}

} // namespace are
