#include "resource/texture_array.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

TextureArray::TextureArray()
    : handle_(INVALID_HANDLE)
    , width_(0)
    , height_(0)
    , layers_(0)
    , internal_format_(0) {
}

TextureArray::~TextureArray() {
    release();
}

TextureArray::TextureArray(TextureArray&& other) noexcept
    : handle_(other.handle_)
    , width_(other.width_)
    , height_(other.height_)
    , layers_(other.layers_)
    , internal_format_(other.internal_format_) {
    other.handle_ = INVALID_HANDLE;
    other.width_ = 0;
    other.height_ = 0;
    other.layers_ = 0;
    other.internal_format_ = 0;
}

TextureArray& TextureArray::operator=(TextureArray&& other) noexcept {
    if (this == &other) return *this;
    release();
    handle_ = other.handle_;
    width_ = other.width_;
    height_ = other.height_;
    layers_ = other.layers_;
    internal_format_ = other.internal_format_;
    other.handle_ = INVALID_HANDLE;
    other.width_ = 0;
    other.height_ = 0;
    other.layers_ = 0;
    other.internal_format_ = 0;
    return *this;
}

bool TextureArray::create(uint width, uint height, uint layers, uint internal_format) {
    release();

    width_ = width;
    height_ = height;
    layers_ = layers;
    internal_format_ = internal_format;

    glGenTextures(1, &handle_);
    glBindTexture(GL_TEXTURE_2D_ARRAY, handle_);

    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, internal_format_, width_, height_, layers_);

    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
    return true;
}

bool TextureArray::upload_rgba8(uint layer, const void* data, uint width, uint height) {
    if (!is_valid()) {
        ARE_LOG_ERROR("TextureArray upload on invalid handle");
        return false;
    }
    if (layer >= layers_) {
        ARE_LOG_ERROR("TextureArray layer out of range");
        return false;
    }
    if (width != width_ || height != height_) {
        ARE_LOG_WARN("TextureArray upload size mismatch (resizing not implemented yet)");
        return false;
    }

    glBindTexture(GL_TEXTURE_2D_ARRAY, handle_);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY,
                    0, 0, 0, static_cast<int>(layer),
                    width_, height_, 1,
                    GL_RGBA, GL_UNSIGNED_BYTE, data);
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
    return true;
}

void TextureArray::bind(uint unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D_ARRAY, handle_);
}

void TextureArray::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteTextures(1, &handle_);
        handle_ = INVALID_HANDLE;
    }
    width_ = height_ = layers_ = 0;
    internal_format_ = 0;
}

} // namespace are
