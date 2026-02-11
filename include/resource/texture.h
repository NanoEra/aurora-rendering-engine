#ifndef ARE_INCLUDE_RESOURCE_TEXTURE_H
#define ARE_INCLUDE_RESOURCE_TEXTURE_H

#include "basic/types.h"
#include <string>

namespace are {

/// @brief Texture format enumeration
enum class TextureFormat {
    R8,
    RG8,
    RGB8,
    RGBA8,
    R16F,
    RG16F,
    RGB16F,
    RGBA16F,
    R32F,
    RG32F,
    RGB32F,
    RGBA32F,
    DEPTH24_STENCIL8
};

/// @brief Texture filter mode
enum class TextureFilter {
    NEAREST,
    LINEAR,
    NEAREST_MIPMAP_NEAREST,
    LINEAR_MIPMAP_NEAREST,
    NEAREST_MIPMAP_LINEAR,
    LINEAR_MIPMAP_LINEAR
};

/// @brief Texture wrap mode
enum class TextureWrap {
    REPEAT,
    MIRRORED_REPEAT,
    CLAMP_TO_EDGE,
    CLAMP_TO_BORDER
};

/// @brief Texture resource
class Texture {
public:
    /// @brief Constructor
    Texture();

	Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;
    
    /// @brief Destructor
    ~Texture();
    
    /// @brief Load texture from file
    /// @param path File path
    /// @param generate_mipmaps Generate mipmaps
    /// @return True if loading succeeded
    bool load_from_file(const std::string& path, bool generate_mipmaps = true);
    
    /// @brief Create empty texture
    /// @param width Texture width
    /// @param height Texture height
    /// @param format Texture format
    /// @return True if creation succeeded
    bool create(uint width, uint height, TextureFormat format);
    
    /// @brief Upload data to texture
    /// @param data Pixel data
    /// @param width Data width
    /// @param height Data height
    /// @param format Data format
    /// @return True if upload succeeded
    bool upload(const void* data, uint width, uint height, TextureFormat format);
    
    /// @brief Set texture filter mode
    /// @param min_filter Minification filter
    /// @param mag_filter Magnification filter
    void set_filter(TextureFilter min_filter, TextureFilter mag_filter);
    
    /// @brief Set texture wrap mode
    /// @param wrap_s Wrap mode for S coordinate
    /// @param wrap_t Wrap mode for T coordinate
    void set_wrap(TextureWrap wrap_s, TextureWrap wrap_t);
    
    /// @brief Generate mipmaps
    void generate_mipmaps();
    
    /// @brief Bind texture to texture unit
    /// @param unit Texture unit
    void bind(uint unit) const;
    
    /// @brief Unbind texture
    void unbind() const;
    
    /// @brief Release texture resources
    void release();
    
    /// @brief Get texture handle
    /// @return Texture handle
    TextureHandle get_handle() const { return handle_; }
    
    /// @brief Get texture width
    /// @return Width
    uint get_width() const { return width_; }
    
    /// @brief Get texture height
    /// @return Height
    uint get_height() const { return height_; }
    
    /// @brief Get texture format
    /// @return Format
    TextureFormat get_format() const { return format_; }
    
    /// @brief Check if texture is valid
    /// @return True if valid
    bool is_valid() const { return handle_ != INVALID_HANDLE; }

private:
    TextureHandle handle_;
    uint width_;
    uint height_;
    TextureFormat format_;
    bool has_mipmaps_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_TEXTURE_H
