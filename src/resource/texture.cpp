#include "resource/texture.h"
#include "utils/logger.h"
#include <glad/glad.h>
#include <stb_image.h>

namespace are {

namespace {
    GLenum get_gl_internal_format(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8: return GL_R8;
            case TextureFormat::RG8: return GL_RG8;
            case TextureFormat::RGB8: return GL_RGB8;
            case TextureFormat::RGBA8: return GL_RGBA8;
            case TextureFormat::R16F: return GL_R16F;
            case TextureFormat::RG16F: return GL_RG16F;
            case TextureFormat::RGB16F: return GL_RGB16F;
            case TextureFormat::RGBA16F: return GL_RGBA16F;
            case TextureFormat::R32F: return GL_R32F;
            case TextureFormat::RG32F: return GL_RG32F;
            case TextureFormat::RGB32F: return GL_RGB32F;
            case TextureFormat::RGBA32F: return GL_RGBA32F;
            case TextureFormat::DEPTH24_STENCIL8: return GL_DEPTH24_STENCIL8;
            default: return GL_RGBA8;
        }
    }
    
    GLenum get_gl_format(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8:
            case TextureFormat::R16F:
            case TextureFormat::R32F:
                return GL_RED;
            case TextureFormat::RG8:
            case TextureFormat::RG16F:
            case TextureFormat::RG32F:
                return GL_RG;
            case TextureFormat::RGB8:
            case TextureFormat::RGB16F:
            case TextureFormat::RGB32F:
                return GL_RGB;
            case TextureFormat::RGBA8:
            case TextureFormat::RGBA16F:
            case TextureFormat::RGBA32F:
                return GL_RGBA;
            case TextureFormat::DEPTH24_STENCIL8:
                return GL_DEPTH_STENCIL;
            default:
                return GL_RGBA;
        }
    }
    
    GLenum get_gl_type(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8:
            case TextureFormat::RG8:
            case TextureFormat::RGB8:
            case TextureFormat::RGBA8:
                return GL_UNSIGNED_BYTE;
            case TextureFormat::R16F:
            case TextureFormat::RG16F:
            case TextureFormat::RGB16F:
            case TextureFormat::RGBA16F:
            case TextureFormat::R32F:
            case TextureFormat::RG32F:
            case TextureFormat::RGB32F:
            case TextureFormat::RGBA32F:
                return GL_FLOAT;
            case TextureFormat::DEPTH24_STENCIL8:
                return GL_UNSIGNED_INT_24_8;
            default:
                return GL_UNSIGNED_BYTE;
        }
    }
    
    GLenum get_gl_filter(TextureFilter filter) {
        switch (filter) {
            case TextureFilter::NEAREST: return GL_NEAREST;
            case TextureFilter::LINEAR: return GL_LINEAR;
            case TextureFilter::NEAREST_MIPMAP_NEAREST: return GL_NEAREST_MIPMAP_NEAREST;
            case TextureFilter::LINEAR_MIPMAP_NEAREST: return GL_LINEAR_MIPMAP_NEAREST;
            case TextureFilter::NEAREST_MIPMAP_LINEAR: return GL_NEAREST_MIPMAP_LINEAR;
            case TextureFilter::LINEAR_MIPMAP_LINEAR: return GL_LINEAR_MIPMAP_LINEAR;
            default: return GL_LINEAR;
        }
    }
    
    GLenum get_gl_wrap(TextureWrap wrap) {
        switch (wrap) {
            case TextureWrap::REPEAT: return GL_REPEAT;
            case TextureWrap::MIRRORED_REPEAT: return GL_MIRRORED_REPEAT;
            case TextureWrap::CLAMP_TO_EDGE: return GL_CLAMP_TO_EDGE;
            case TextureWrap::CLAMP_TO_BORDER: return GL_CLAMP_TO_BORDER;
            default: return GL_REPEAT;
        }
    }
}

Texture::Texture()
    : handle_(INVALID_HANDLE)
    , width_(0)
    , height_(0)
    , format_(TextureFormat::RGBA8)
    , has_mipmaps_(false) {
}

Texture::Texture(Texture&& other) noexcept
    : handle_(other.handle_)
    , width_(other.width_)
    , height_(other.height_)
    , format_(other.format_)
    , has_mipmaps_(other.has_mipmaps_) {
    other.handle_ = INVALID_HANDLE;
    other.width_ = 0;
    other.height_ = 0;
    other.has_mipmaps_ = false;
}

Texture& Texture::operator=(Texture&& other) noexcept {
    if (this == &other) return *this;

    release();
    handle_ = other.handle_;
    width_ = other.width_;
    height_ = other.height_;
    format_ = other.format_;
    has_mipmaps_ = other.has_mipmaps_;

    other.handle_ = INVALID_HANDLE;
    other.width_ = 0;
    other.height_ = 0;
    other.has_mipmaps_ = false;
    return *this;
}

Texture::~Texture() {
	release();
}

bool Texture::load_from_file(const std::string& path, bool generate_mipmaps) {
    // Load image using stb_image
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        ARE_LOG_ERROR("Failed to load texture: " + path);
        return false;
    }
    
    // Determine format based on channels
    TextureFormat format;
    switch (channels) {
        case 1: format = TextureFormat::R8; break;
        case 2: format = TextureFormat::RG8; break;
        case 3: format = TextureFormat::RGB8; break;
        case 4: format = TextureFormat::RGBA8; break;
        default:
            ARE_LOG_ERROR("Unsupported channel count: " + std::to_string(channels));
            stbi_image_free(data);
            return false;
    }
    
    // Create texture
    bool success = create(width, height, format);
    if (!success) {
        stbi_image_free(data);
        return false;
    }
    
    // Upload data
    success = upload(data, width, height, format);
    stbi_image_free(data);
    
    if (!success) {
        return false;
    }
    
    // Generate mipmaps if requested
    if (generate_mipmaps) {
        this->generate_mipmaps();
    }
    
    ARE_LOG_INFO("Texture loaded successfully: " + path);
    return true;
}

bool Texture::create(uint width, uint height, TextureFormat format) {
    if (handle_ != INVALID_HANDLE) {
        ARE_LOG_WARN("Texture already created, releasing old texture");
        release();
    }
    
    width_ = width;
    height_ = height;
    format_ = format;
    
    glGenTextures(1, &handle_);
    glBindTexture(GL_TEXTURE_2D, handle_);
    
    GLenum internal_format = get_gl_internal_format(format);
    GLenum gl_format = get_gl_format(format);
    GLenum type = get_gl_type(format);
    
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, gl_format, type, nullptr);
    
    // Set default parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    
    return true;
}

bool Texture::upload(const void* data, uint width, uint height, TextureFormat format) {
    if (handle_ == INVALID_HANDLE) {
        ARE_LOG_ERROR("Cannot upload to invalid texture");
        return false;
    }
    
    if (width != width_ || height != height_ || format != format_) {
        ARE_LOG_WARN("Upload parameters differ from texture creation, recreating texture");
        create(width, height, format);
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    
    GLenum gl_format = get_gl_format(format);
    GLenum type = get_gl_type(format);
    
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, gl_format, type, data);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    
    return true;
}

void Texture::set_filter(TextureFilter min_filter, TextureFilter mag_filter) {
    if (handle_ == INVALID_HANDLE) {
        ARE_LOG_ERROR("Cannot set filter on invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, get_gl_filter(min_filter));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, get_gl_filter(mag_filter));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::set_wrap(TextureWrap wrap_s, TextureWrap wrap_t) {
    if (handle_ == INVALID_HANDLE) {
        ARE_LOG_ERROR("Cannot set wrap mode on invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, get_gl_wrap(wrap_s));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, get_gl_wrap(wrap_t));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::generate_mipmaps() {
    if (handle_ == INVALID_HANDLE) {
        ARE_LOG_ERROR("Cannot generate mipmaps for invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    
    has_mipmaps_ = true;
}

void Texture::bind(uint unit) const {
    if (handle_ == INVALID_HANDLE) {
        ARE_LOG_WARN("Attempting to bind invalid texture");
        return;
    }
    
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, handle_);
}

void Texture::unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteTextures(1, &handle_);
        handle_ = INVALID_HANDLE;
    }
    
    width_ = 0;
    height_ = 0;
    has_mipmaps_ = false;
}

} // namespace are
