#ifndef ARE_INCLUDE_RESOURCE_TEXTURE_ARRAY_H
#define ARE_INCLUDE_RESOURCE_TEXTURE_ARRAY_H

#include "basic/types.h"
#include <string>

namespace are {

/**
 * @brief 2D texture array wrapper for PBR textures
 */
class TextureArray {
public:
    /**
     * @brief Construct texture array
     */
    TextureArray();

    /**
     * @brief Destroy texture array
     */
    ~TextureArray();

    TextureArray(const TextureArray&) = delete;
    TextureArray& operator=(const TextureArray&) = delete;

    TextureArray(TextureArray&& other) noexcept;
    TextureArray& operator=(TextureArray&& other) noexcept;

    /**
     * @brief Create empty texture array storage
     * @param width Layer width
     * @param height Layer height
     * @param layers Layer count
     * @param internal_format OpenGL internal format (e.g. GL_RGBA8, GL_RGBA16F)
     * @param srgb True if texture should be treated as sRGB (use GL_SRGB8_ALPHA8)
     * @return True on success
     */
    bool create(uint width, uint height, uint layers, uint internal_format);

    /**
     * @brief Upload one layer (expects RGBA8 data)
     * @param layer Layer index
     * @param data Pixel data pointer
     * @param width Data width
     * @param height Data height
     */
    bool upload_rgba8(uint layer, const void* data, uint width, uint height);

    /**
     * @brief Bind to texture unit
     */
    void bind(uint unit) const;

    /**
     * @brief Release OpenGL resources
     */
    void release();

    /**
     * @brief Get OpenGL handle
     */
    TextureHandle get_handle() const { return handle_; }

    uint get_width() const { return width_; }
    uint get_height() const { return height_; }
    uint get_layers() const { return layers_; }

    bool is_valid() const { return handle_ != INVALID_HANDLE; }

private:
    TextureHandle handle_;
    uint width_;
    uint height_;
    uint layers_;
    uint internal_format_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_TEXTURE_ARRAY_H
