/**
 * @file texture_manager.cpp
 * @brief Implementation of texture manager
 */

#include <are/texture/texture_manager.h>
#include <are/core/logger.h>

namespace are {

TextureManager::TextureManager()
    : next_handle_(1)
{
    ARE_LOG_INFO("Texture manager initialized");
}

TextureManager::~TextureManager() {
    clear();
    ARE_LOG_INFO("Texture manager destroyed");
}

TextureHandle TextureManager::load_texture(const std::string& filepath,
                                          TextureFormat format,
                                          bool generate_mipmaps) {
    // Check if texture already loaded
    auto it = path_to_handle_.find(filepath);
    if (it != path_to_handle_.end()) {
        ARE_LOG_INFO("Texture already loaded: " + filepath);
        return it->second;
    }
    
    // Create new texture
    auto texture = std::make_unique<Texture>();
    
    if (!texture->load_from_file(filepath, format, generate_mipmaps)) {
        ARE_LOG_ERROR("Failed to load texture: " + filepath);
        return are_invalid_handle;
    }
    
    // Assign handle
    TextureHandle handle = next_handle_++;
    
    // Store texture
    textures_[handle] = std::move(texture);
    path_to_handle_[filepath] = handle;
    
    ARE_LOG_INFO("Texture loaded: " + filepath + " (handle: " + std::to_string(handle) + ")");
    
    return handle;
}

TextureHandle TextureManager::create_texture(const std::string& name,
                                            int width, int height,
                                            TextureFormat format,
                                            const void* data,
                                            bool generate_mipmaps) {
    // Check if texture with this name already exists
    auto it = path_to_handle_.find(name);
    if (it != path_to_handle_.end()) {
        ARE_LOG_WARN("Texture with name already exists: " + name);
        return it->second;
    }
    
    // Create new texture
    auto texture = std::make_unique<Texture>();
    
    if (!texture->create_from_data(width, height, format, data, generate_mipmaps)) {
        ARE_LOG_ERROR("Failed to create texture: " + name);
        return are_invalid_handle;
    }
    
    // Assign handle
    TextureHandle handle = next_handle_++;
    
    // Store texture
    textures_[handle] = std::move(texture);
    path_to_handle_[name] = handle;
    
    ARE_LOG_INFO("Texture created: " + name + " (handle: " + std::to_string(handle) + ")");
    
    return handle;
}

Texture* TextureManager::get_texture(TextureHandle handle) {
    auto it = textures_.find(handle);
    if (it != textures_.end()) {
        return it->second.get();
    }
    return nullptr;
}

const Texture* TextureManager::get_texture(TextureHandle handle) const {
    auto it = textures_.find(handle);
    if (it != textures_.end()) {
        return it->second.get();
    }
    return nullptr;
}

void TextureManager::unload_texture(TextureHandle handle) {
    auto it = textures_.find(handle);
    if (it == textures_.end()) {
        return;
    }
    
    // Remove from path map
    for (auto path_it = path_to_handle_.begin(); path_it != path_to_handle_.end(); ++path_it) {
        if (path_it->second == handle) {
            path_to_handle_.erase(path_it);
            break;
        }
    }
    
    // Remove texture
    textures_.erase(it);
    
    ARE_LOG_INFO("Texture unloaded (handle: " + std::to_string(handle) + ")");
}

void TextureManager::clear() {
    textures_.clear();
    path_to_handle_.clear();
    next_handle_ = 1;
    
    ARE_LOG_INFO("All textures cleared");
}

size_t TextureManager::get_memory_usage() const {
    size_t total = 0;
    
    for (const auto& [handle, texture] : textures_) {
        if (texture && texture->is_valid()) {
            // Estimate memory usage (width * height * bytes_per_pixel)
            int width = texture->get_width();
            int height = texture->get_height();
            
            // Estimate bytes per pixel based on format
            int bytes_per_pixel = 4; // Default RGBA8
            
            switch (texture->get_format()) {
                case TextureFormat::ARE_TEXTURE_R8:
                    bytes_per_pixel = 1;
                    break;
                case TextureFormat::ARE_TEXTURE_RG8:
                    bytes_per_pixel = 2;
                    break;
                case TextureFormat::ARE_TEXTURE_RGB8:
                    bytes_per_pixel = 3;
                    break;
                case TextureFormat::ARE_TEXTURE_RGBA8:
                    bytes_per_pixel = 4;
                    break;
                case TextureFormat::ARE_TEXTURE_R16F:
                    bytes_per_pixel = 2;
                    break;
                case TextureFormat::ARE_TEXTURE_RG16F:
                    bytes_per_pixel = 4;
                    break;
                case TextureFormat::ARE_TEXTURE_RGB16F:
                    bytes_per_pixel = 6;
                    break;
                case TextureFormat::ARE_TEXTURE_RGBA16F:
                    bytes_per_pixel = 8;
                    break;
                case TextureFormat::ARE_TEXTURE_R32F:
                    bytes_per_pixel = 4;
                    break;
                case TextureFormat::ARE_TEXTURE_RG32F:
                    bytes_per_pixel = 8;
                    break;
                case TextureFormat::ARE_TEXTURE_RGB32F:
                    bytes_per_pixel = 12;
                    break;
                case TextureFormat::ARE_TEXTURE_RGBA32F:
                    bytes_per_pixel = 16;
                    break;
            }
            
            size_t texture_size = width * height * bytes_per_pixel;
            
            // Account for mipmaps (approximately 1.33x base size)
            texture_size = static_cast<size_t>(texture_size * 1.33);
            
            total += texture_size;
        }
    }
    
    return total;
}

} // namespace are
