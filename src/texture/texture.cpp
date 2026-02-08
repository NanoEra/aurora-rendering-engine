/**
 * @file texture.cpp
 * @brief Implementation of texture class
 */

#include <are/texture/texture.h>
#include <are/utils/image_io.h>
#include <are/core/logger.h>
#include <glad/glad.h>

namespace are {

// Helper function to convert TextureFormat to OpenGL format
static GLenum get_gl_internal_format(TextureFormat format) {
    switch (format) {
        case TextureFormat::ARE_TEXTURE_R8:       return GL_R8;
        case TextureFormat::ARE_TEXTURE_RG8:      return GL_RG8;
        case TextureFormat::ARE_TEXTURE_RGB8:     return GL_RGB8;
        case TextureFormat::ARE_TEXTURE_RGBA8:    return GL_RGBA8;
        case TextureFormat::ARE_TEXTURE_R16F:     return GL_R16F;
        case TextureFormat::ARE_TEXTURE_RG16F:    return GL_RG16F;
        case TextureFormat::ARE_TEXTURE_RGB16F:   return GL_RGB16F;
        case TextureFormat::ARE_TEXTURE_RGBA16F:  return GL_RGBA16F;
        case TextureFormat::ARE_TEXTURE_R32F:     return GL_R32F;
        case TextureFormat::ARE_TEXTURE_RG32F:    return GL_RG32F;
        case TextureFormat::ARE_TEXTURE_RGB32F:   return GL_RGB32F;
        case TextureFormat::ARE_TEXTURE_RGBA32F:  return GL_RGBA32F;
        default: return GL_RGBA8;
    }
}

static GLenum get_gl_format(int channels) {
    switch (channels) {
        case 1: return GL_RED;
        case 2: return GL_RG;
        case 3: return GL_RGB;
        case 4: return GL_RGBA;
        default: return GL_RGBA;
    }
}

static GLenum get_gl_filter(TextureFilter filter) {
    switch (filter) {
        case TextureFilter::ARE_TEXTURE_FILTER_NEAREST:
            return GL_NEAREST;
        case TextureFilter::ARE_TEXTURE_FILTER_LINEAR:
            return GL_LINEAR;
        case TextureFilter::ARE_TEXTURE_FILTER_NEAREST_MIPMAP_NEAREST:
            return GL_NEAREST_MIPMAP_NEAREST;
        case TextureFilter::ARE_TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST:
            return GL_LINEAR_MIPMAP_NEAREST;
        case TextureFilter::ARE_TEXTURE_FILTER_NEAREST_MIPMAP_LINEAR:
            return GL_NEAREST_MIPMAP_LINEAR;
        case TextureFilter::ARE_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR:
            return GL_LINEAR_MIPMAP_LINEAR;
        default:
            return GL_LINEAR;
    }
}

static GLenum get_gl_wrap(TextureWrap wrap) {
    switch (wrap) {
        case TextureWrap::ARE_TEXTURE_WRAP_REPEAT:
            return GL_REPEAT;
        case TextureWrap::ARE_TEXTURE_WRAP_CLAMP_TO_EDGE:
            return GL_CLAMP_TO_EDGE;
        case TextureWrap::ARE_TEXTURE_WRAP_CLAMP_TO_BORDER:
            return GL_CLAMP_TO_BORDER;
        case TextureWrap::ARE_TEXTURE_WRAP_MIRRORED_REPEAT:
            return GL_MIRRORED_REPEAT;
        default:
            return GL_REPEAT;
    }
}

Texture::Texture()
    : texture_id_(0)
    , width_(0)
    , height_(0)
    , format_(TextureFormat::ARE_TEXTURE_RGBA8)
{
}

Texture::~Texture() {
    destroy();
}

bool Texture::load_from_file(const std::string& filepath, 
                             TextureFormat format,
                             bool generate_mipmaps) {
    // Load image data
    ImageData image = load_image(filepath, true); // Flip vertically for OpenGL
    
    if (!image.is_valid()) {
        ARE_LOG_ERROR("Failed to load image: " + filepath);
        return false;
    }
    
    return create_from_data(image.width_, image.height_, format, 
                           image.data_.data(), generate_mipmaps);
}

bool Texture::create_from_data(int width, int height, 
                               TextureFormat format,
                               const void* data,
                               bool generate_mipmaps) {
    if (width <= 0 || height <= 0) {
        ARE_LOG_ERROR("Invalid texture dimensions");
        return false;
    }
    
    // Delete old texture if exists
    if (texture_id_ != 0) {
        destroy();
    }
    
    width_ = width;
    height_ = height;
    format_ = format;
    
    // Create OpenGL texture
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                   generate_mipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload texture data
    GLenum internal_format = get_gl_internal_format(format);
    GLenum data_format = GL_RGBA; // Assume RGBA input
    
    // Determine data format from input (assume 4 channels for now)
    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0,
                    data_format, GL_UNSIGNED_BYTE, data);
    } else {
        // Create empty texture
        glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0,
                    data_format, GL_UNSIGNED_BYTE, nullptr);
    }
    
    // Generate mipmaps
    if (generate_mipmaps) {
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    
    glBindTexture(GL_TEXTURE_2D, 0);
    
    return true;
}

void Texture::bind(int unit) const {
    if (texture_id_ == 0) {
        ARE_LOG_WARN("Attempting to bind invalid texture");
        return;
    }
    
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
}

void Texture::unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::set_filter(TextureFilter min_filter, TextureFilter mag_filter) {
    if (texture_id_ == 0) return;
    
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, get_gl_filter(min_filter));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, get_gl_filter(mag_filter));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::set_wrap(TextureWrap wrap_s, TextureWrap wrap_t) {
    if (texture_id_ == 0) return;
    
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, get_gl_wrap(wrap_s));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, get_gl_wrap(wrap_t));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::generate_mipmaps() {
    if (texture_id_ == 0) return;
    
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::destroy() {
    if (texture_id_ != 0) {
        glDeleteTextures(1, &texture_id_);
        texture_id_ = 0;
        width_ = 0;
        height_ = 0;
    }
}

} // namespace are
