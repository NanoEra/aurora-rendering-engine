/**
 * @file texture_manager.h
 * @brief Texture resource management and caching
 */

#ifndef ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H
#define ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H

#include <are/core/types.h>
#include <are/texture/texture.h>
#include <string>
#include <unordered_map>
#include <memory>

namespace are {

/**
 * @class TextureManager
 * @brief Manages texture loading and caching
 * 
 * Automatically handles texture deduplication and lifetime management.
 */
class TextureManager {
public:
    /**
     * @brief Constructor
     */
    TextureManager();

    /**
     * @brief Destructor
     */
    ~TextureManager();

    /**
     * @brief Load texture from file (with caching)
     * @param filepath Texture file path
     * @param format Desired texture format
     * @param generate_mipmaps Generate mipmaps
     * @return Texture handle (are_invalid_handle if failed)
     */
    TextureHandle load_texture(const std::string& filepath,
                              TextureFormat format = TextureFormat::ARE_TEXTURE_RGBA8,
                              bool generate_mipmaps = true);

    /**
     * @brief Create texture from raw data
     * @param name Texture name (for caching)
     * @param width Texture width
     * @param height Texture height
     * @param format Texture format
     * @param data Pixel data
     * @param generate_mipmaps Generate mipmaps
     * @return Texture handle
     */
    TextureHandle create_texture(const std::string& name,
                                int width, int height,
                                TextureFormat format,
                                const void* data,
                                bool generate_mipmaps = true);

    /**
     * @brief Get texture by handle
     * @param handle Texture handle
     * @return Texture pointer (nullptr if not found)
     */
    Texture* get_texture(TextureHandle handle);
    const Texture* get_texture(TextureHandle handle) const;

    /**
     * @brief Unload texture
     * @param handle Texture handle
     */
    void unload_texture(TextureHandle handle);

    /**
     * @brief Clear all textures
     */
    void clear();

    /**
     * @brief Get total texture memory usage
     * @return Memory usage in bytes
     */
    size_t get_memory_usage() const;

    /**
     * @brief Get number of loaded textures
     * @return Texture count
     */
    size_t get_texture_count() const { return textures_.size(); }

private:
    std::unordered_map<std::string, TextureHandle> path_to_handle_;
    std::unordered_map<TextureHandle, std::unique_ptr<Texture>> textures_;
    TextureHandle next_handle_;
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_TEXTURE_MANAGER_H
