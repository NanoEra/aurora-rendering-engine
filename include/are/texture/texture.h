/**
 * @file texture.h
 * @brief Texture resource management
 */

#ifndef ARE_INCLUDE_TEXTURE_TEXTURE_H
#define ARE_INCLUDE_TEXTURE_TEXTURE_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @enum TextureFormat
 * @brief Texture internal formats
 */
enum class TextureFormat {
    ARE_TEXTURE_R8,
    ARE_TEXTURE_RG8,
    ARE_TEXTURE_RGB8,
    ARE_TEXTURE_RGBA8,
    ARE_TEXTURE_R16F,
    ARE_TEXTURE_RG16F,
    ARE_TEXTURE_RGB16F,
    ARE_TEXTURE_RGBA16F,
    ARE_TEXTURE_R32F,
    ARE_TEXTURE_RG32F,
    ARE_TEXTURE_RGB32F,
    ARE_TEXTURE_RGBA32F
};

/**
 * @enum TextureFilter
 * @brief Texture filtering modes
 */
enum class TextureFilter {
    ARE_TEXTURE_FILTER_NEAREST,
    ARE_TEXTURE_FILTER_LINEAR,
    ARE_TEXTURE_FILTER_NEAREST_MIPMAP_NEAREST,
    ARE_TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST,
    ARE_TEXTURE_FILTER_NEAREST_MIPMAP_LINEAR,
    ARE_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR
};

/**
 * @enum TextureWrap
 * @brief Texture wrapping modes
 */
enum class TextureWrap {
    ARE_TEXTURE_WRAP_REPEAT,
    ARE_TEXTURE_WRAP_CLAMP_TO_EDGE,
    ARE_TEXTURE_WRAP_CLAMP_TO_BORDER,
    ARE_TEXTURE_WRAP_MIRRORED_REPEAT
};

/**
 * @class Texture
 * @brief OpenGL texture wrapper
 */
class Texture {
public:
    /**
     * @brief Constructor
     */
    Texture();

    /**
     * @brief Destructor
     */
    ~Texture();

    /**
     * @brief Load texture from file
     * @param filepath Image file path
     * @param format Desired texture format
     * @param generate_mipmaps Generate mipmaps
     * @return true if load succeeded
     */
    bool load_from_file(const std::string& filepath, 
                       TextureFormat format = TextureFormat::ARE_TEXTURE_RGBA8,
                       bool generate_mipmaps = true);

    /**
     * @brief Create texture from raw data
     * @param width Texture width
     * @param height Texture height
     * @param format Texture format
     * @param data Pixel data
     * @param generate_mipmaps Generate mipmaps
     * @return true if creation succeeded
     */
	bool create_from_data(int width, int height, 
                         TextureFormat format,
                         const void* data,
                         bool generate_mipmaps = true);

    /**
     * @brief Bind texture to texture unit
     * @param unit Texture unit (0-31)
     */
    void bind(int unit = 0) const;

    /**
     * @brief Unbind texture
     */
    void unbind() const;

    /**
     * @brief Set texture filtering
     * @param min_filter Minification filter
     * @param mag_filter Magnification filter
     */
    void set_filter(TextureFilter min_filter, TextureFilter mag_filter);

    /**
     * @brief Set texture wrapping
     * @param wrap_s S-axis wrapping
     * @param wrap_t T-axis wrapping
     */
    void set_wrap(TextureWrap wrap_s, TextureWrap wrap_t);

    /**
     * @brief Generate mipmaps
     */
    void generate_mipmaps();

    // Getters
    uint32_t get_id() const { return texture_id_; }
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    TextureFormat get_format() const { return format_; }
    bool is_valid() const { return texture_id_ != 0; }

    /**
     * @brief Delete texture
     */
    void destroy();

private:
    uint32_t texture_id_;                 ///< OpenGL texture ID
    int width_;                           ///< Texture width
    int height_;                          ///< Texture height
    TextureFormat format_;                ///< Texture format
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_TEXTURE_H
