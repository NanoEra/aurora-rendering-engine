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
    , primitive_id_texture_(0)
    , width_(width)
    , height_(height) {

    create_textures();
    create_framebuffer();
}

GBuffer::~GBuffer() {
    delete_textures();

    if (rbo_depth_ != 0) {
        glDeleteRenderbuffers(1, &rbo_depth_);
        rbo_depth_ = 0;
    }

    if (fbo_ != 0) {
        glDeleteFramebuffers(1, &fbo_);
        fbo_ = 0;
    }
}

void GBuffer::resize(int width, int height) {
    ARE_PROFILE_FUNCTION();

    if (width == width_ && height == height_) {
        return;
    }

    width_ = width;
    height_ = height;

    delete_textures();

    if (rbo_depth_ != 0) {
        glDeleteRenderbuffers(1, &rbo_depth_);
        rbo_depth_ = 0;
    }

    if (fbo_ != 0) {
        glDeleteFramebuffers(1, &fbo_);
        fbo_ = 0;
    }

    create_textures();
    create_framebuffer();
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
        case 5: glBindTexture(GL_TEXTURE_2D, primitive_id_texture_); break;
        default:
            ARE_LOG_WARN("GBuffer: Invalid texture index " + std::to_string(index));
            break;
    }
}

void GBuffer::read_pixels(int index, void* data) {
    ARE_PROFILE_FUNCTION();

    // Robust: read from texture object (not from FBO read buffer)
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);

    uint32_t tex = 0;
    GLenum format = GL_RGBA;
    GLenum type = GL_UNSIGNED_BYTE;

    switch (index) {
        case 0:
            tex = position_texture_;
            format = GL_RGB;
            type = GL_FLOAT;
            break;
        case 1:
            tex = normal_texture_;
            format = GL_RGB;
            type = GL_FLOAT;
            break;
        case 2:
            tex = albedo_texture_;
            format = GL_RGBA;
            type = GL_UNSIGNED_BYTE;
            break;
        case 3:
            tex = material_texture_;
            format = GL_RG;
            type = GL_UNSIGNED_BYTE;
            break;
        case 4:
            tex = depth_texture_;
            format = GL_DEPTH_COMPONENT;
            type = GL_FLOAT;
            break;
        case 5:
            tex = primitive_id_texture_;
            format = GL_RED_INTEGER;
            type = GL_UNSIGNED_INT;
            break;
        default:
            ARE_LOG_ERROR("GBuffer: Invalid buffer index for read_pixels");
            return;
    }

    glBindTexture(GL_TEXTURE_2D, tex);
    glGetTexImage(GL_TEXTURE_2D, 0, format, type, data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void GBuffer::create_textures() {
    ARE_PROFILE_FUNCTION();

    glGenTextures(1, &position_texture_);
    glBindTexture(GL_TEXTURE_2D, position_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width_, height_, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &normal_texture_);
    glBindTexture(GL_TEXTURE_2D, normal_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width_, height_, 0, GL_RGB, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &albedo_texture_);
    glBindTexture(GL_TEXTURE_2D, albedo_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &material_texture_);
    glBindTexture(GL_TEXTURE_2D, material_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, width_, height_, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &primitive_id_texture_);
    glBindTexture(GL_TEXTURE_2D, primitive_id_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, width_, height_, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void GBuffer::delete_textures() {
    if (position_texture_ != 0) glDeleteTextures(1, &position_texture_);
    if (normal_texture_ != 0) glDeleteTextures(1, &normal_texture_);
    if (albedo_texture_ != 0) glDeleteTextures(1, &albedo_texture_);
    if (material_texture_ != 0) glDeleteTextures(1, &material_texture_);
    if (depth_texture_ != 0) glDeleteTextures(1, &depth_texture_);
    if (primitive_id_texture_ != 0) glDeleteTextures(1, &primitive_id_texture_);

    position_texture_ = 0;
    normal_texture_ = 0;
    albedo_texture_ = 0;
    material_texture_ = 0;
    depth_texture_ = 0;
    primitive_id_texture_ = 0;
}

void GBuffer::create_framebuffer() {
    ARE_PROFILE_FUNCTION();

    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, position_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, normal_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, albedo_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, material_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_2D, primitive_id_texture_, 0);

    GLenum draw_buffers[] = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3,
        GL_COLOR_ATTACHMENT4
    };
    glDrawBuffers(5, draw_buffers);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        ARE_LOG_ERROR("GBuffer: Framebuffer incomplete. Status=" + std::to_string(status));
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace are
