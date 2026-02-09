#include "core/screen_blit.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

namespace {
    const char* VERTEX_SHADER_SOURCE = R"(
        #version 430 core
        layout(location = 0) in vec2 a_position;
        layout(location = 1) in vec2 a_texcoord;
        
        out vec2 v_texcoord;
        
        void main() {
            v_texcoord = a_texcoord;
            gl_Position = vec4(a_position, 0.0, 1.0);
        }
    )";
    
    const char* FRAGMENT_SHADER_SOURCE = R"(
        #version 430 core
        in vec2 v_texcoord;
        out vec4 frag_color;
        
        uniform sampler2D u_texture;
        
        void main() {
            frag_color = texture(u_texture, v_texcoord);
        }
    )";
}

ScreenBlit::ScreenBlit()
    : vao_(0)
    , vbo_(0)
    , initialized_(false) {
}

ScreenBlit::~ScreenBlit() {
    release();
}

bool ScreenBlit::initialize() {
    if (initialized_) {
        Logger::warning("ScreenBlit already initialized");
        return true;
    }
    
    // Compile shader
    if (!shader_.compile(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE)) {
        Logger::error("Failed to compile screen blit shader");
        return false;
    }
    
    // Create fullscreen quad
    create_quad_();
    
    initialized_ = true;
    Logger::info("ScreenBlit initialized successfully");
    return true;
}

void ScreenBlit::release() {
    if (!initialized_) return;
    
    shader_.release();
    
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    
    initialized_ = false;
}

void ScreenBlit::blit(TextureHandle texture, int x, int y, uint width, uint height) {
    if (!initialized_) {
        Logger::error("ScreenBlit not initialized");
        return;
    }
    
    // Set viewport
    glViewport(x, y, width, height);
    
    // Disable depth test
    glDisable(GL_DEPTH_TEST);
    
    // Use shader
    shader_.use();
    shader_.set_int("u_texture", 0);
    
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
    
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    
    // TexCoord attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    
    glBindVertexArray(0);
}

} // namespace are
