/**
 * @file gbuffer.cpp
 * @brief Implementation of GBuffer class
 */

#include <are/rasterizer/gbuffer.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <glad/glad.h>

namespace are {

GBuffer::GBuffer(int width, int height)
    : fbo_(0)
    , rbo_depth_(0)
    , position_texture_(0)
    , normal_texture_(0)
    , albedo_texture_(0)
    , material_texture_(0)
    , depth_texture_(0)
    , width_(width)
    , height_(height) {
    
    create_textures();
    create_framebuffer();
    
    ARE_LOG_INFO("GBuffer: Created " + std::to_string(width) + "x" + std::to_string(height));
}

GBuffer::~GBuffer() {
    delete_textures();
    
    if (rbo_depth_ != 0) {
        glDeleteRenderbuffers(1, &rbo_depth_);
    }
    if (fbo_ != 0) {
        glDeleteFramebuffers(1, &fbo_);
    }
}

void GBuffer::resize(int width, int height) {
    ARE_PROFILE_FUNCTION();
    
    if (width == width_ && height == height_) {
        return;
    }
    
    width_ = width;
    height_ = height;
    
    // Recreate textures and framebuffer
    delete_textures();
    if (rbo_depth_ != 0) {
        glDeleteRenderbuffers(1, &rbo_depth_);
        rbo_depth_ = 0;
    }
    
    create_textures();
    create_framebuffer();
    
    ARE_LOG_INFO("GBuffer: Resized to " + std::to_string(width) + "x" + std::to_string(height));
}

void GBuffer::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, width_, height_);
}

void GBuffer::unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::clear() {
    bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    unbind();
}

void GBuffer::bind_texture(int index, int texture_unit) {
    glActiveTexture(GL_TEXTURE0 + texture_unit);
    
    switch (index) {
        case 0: glBindTexture(GL_TEXTURE_2D, position_texture_); break;
        case 1: glBindTexture(GL_TEXTURE_2D, normal_texture_); break;
        case 2: glBindTexture(GL_TEXTURE_2D, albedo_texture_); break;
        case 3: glBindTexture(GL_TEXTURE_2D, material_texture_); break;
        case 4: glBindTexture(GL_TEXTURE_2D, depth_texture_); break;
        default:
            ARE_LOG_WARN("GBuffer: Invalid texture index " + std::to_string(index));
            break;
    }
}

void GBuffer::read_pixels(int index, void* data) {
    ARE_PROFILE_FUNCTION();
    
    bind();
    
    GLenum attachment;
    GLenum format;
    GLenum type;
    
    switch (index) {
        case 0: // Position
            attachment = GL_COLOR_ATTACHMENT0;
            format = GL_RGB;
            type = GL_FLOAT;
            break;
        case 1: // Normal
            attachment = GL_COLOR_ATTACHMENT1;
            format = GL_RGB;
            type = GL_FLOAT;
            break;
        case 2: // Albedo
            attachment = GL_COLOR_ATTACHMENT2;
            format = GL_RGBA;
            type = GL_UNSIGNED_BYTE;
            break;
        case 3: // Material
            attachment = GL_COLOR_ATTACHMENT3;
            format = GL_RG;
            type = GL_UNSIGNED_BYTE;
            break;
        case 4: // Depth
            attachment = GL_DEPTH_ATTACHMENT;
            format = GL_DEPTH_COMPONENT;
            type = GL_FLOAT;
            break;
        default:
            ARE_LOG_ERROR("GBuffer: Invalid buffer index for read_pixels");
            unbind();
            return;
    }
    
    glReadBuffer(attachment);
    glReadPixels(0, 0, width_, height_, format, type, data);
    
    unbind();
}

void GBuffer::create_textures() {
    ARE_PROFILE_FUNCTION();
    
    // Position texture (RGB16F)
    glGenTextures(1, &position_texture_);
    glBindTexture(GL_TEXTURE_2D, position_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width_, height_, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Normal texture (RGB16F)
    glGenTextures(1, &normal_texture_);
    glBindTexture(GL_TEXTURE_2D, normal_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width_, height_, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Albedo + Metallic texture (RGBA8)
    glGenTextures(1, &albedo_texture_);
    glBindTexture(GL_TEXTURE_2D, albedo_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Roughness + AO texture (RG8)
    glGenTextures(1, &material_texture_);
    glBindTexture(GL_TEXTURE_2D, material_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, width_, height_, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Depth texture (R32F)
    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width_, height_, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glBindTexture(GL_TEXTURE_2D, 0);
}

void GBuffer::delete_textures() {
    if (position_texture_ != 0) {
        glDeleteTextures(1, &position_texture_);
        position_texture_ = 0;
    }
    if (normal_texture_ != 0) {
        glDeleteTextures(1, &normal_texture_);
        normal_texture_ = 0;
    }
    if (albedo_texture_ != 0) {
        glDeleteTextures(1, &albedo_texture_);
        albedo_texture_ = 0;
    }
    if (material_texture_ != 0) {
        glDeleteTextures(1, &material_texture_);
        material_texture_ = 0;
    }
    if (depth_texture_ != 0) {
        glDeleteTextures(1, &depth_texture_);
        depth_texture_ = 0;
    }
}

void GBuffer::create_framebuffer() {
    ARE_PROFILE_FUNCTION();
    
    // Create framebuffer
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    
    // Attach textures
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, position_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, normal_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, albedo_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, material_texture_, 0);
    
    // Specify draw buffers
    GLenum draw_buffers[] = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3
    };
    glDrawBuffers(4, draw_buffers);
    
    // Create depth renderbuffer
    glGenRenderbuffers(1, &rbo_depth_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_depth_);
    
    // Check framebuffer completeness
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        ARE_LOG_ERROR("GBuffer: Framebuffer is not complete! Status: " + std::to_string(status));
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace are
