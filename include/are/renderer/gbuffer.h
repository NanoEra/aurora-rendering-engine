/**
 * @file gbuffer.h
 * @brief G-Buffer management for deferred rendering
 */

#ifndef ARE_INCLUDE_RASTERIZER_GBUFFER_H
#define ARE_INCLUDE_RASTERIZER_GBUFFER_H

#include <are/core/types.h>
#include <cstdint>

namespace are {

/**
 * @class GBuffer
 * @brief G-Buffer for deferred rendering
 * 
 * Contains multiple render targets for position, normal, albedo, etc.
 */
class GBuffer {
public:
    /**
     * @brief Constructor
     * @param width Buffer width
     * @param height Buffer height
     */
    GBuffer(int width, int height);

    /**
     * @brief Destructor
     */
    ~GBuffer();

    /**
     * @brief Resize G-Buffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Bind G-Buffer for rendering
     */
    void bind();

    /**
     * @brief Unbind G-Buffer
     */
    void unbind();

    /**
     * @brief Clear all buffers
     */
    void clear();

    /**
     * @brief Bind texture for reading
     * @param index Texture index (0=position, 1=normal, 2=albedo, etc.)
     * @param texture_unit Texture unit to bind to
     */
    void bind_texture(int index, int texture_unit);

    // Texture getters
    uint32_t get_position_texture() const { return position_texture_; }
    uint32_t get_normal_texture() const { return normal_texture_; }
    uint32_t get_albedo_texture() const { return albedo_texture_; }
    uint32_t get_material_texture() const { return material_texture_; }
    uint32_t get_depth_texture() const { return depth_texture_; }

    // Dimensions
    int get_width() const { return width_; }
    int get_height() const { return height_; }

    /**
     * @brief Read pixel data from G-Buffer
     * @param index Buffer index
     * @param data Output data pointer
     */
    void read_pixels(int index, void* data);

private:
    void create_textures();
    void delete_textures();
    void create_framebuffer();

    uint32_t fbo_;                        ///< Framebuffer object
    uint32_t rbo_depth_;                  ///< Depth renderbuffer

    // G-Buffer textures
    uint32_t position_texture_;           ///< World position (RGB16F)
    uint32_t normal_texture_;             ///< World normal (RGB16F)
    uint32_t albedo_texture_;             ///< Albedo + Metallic (RGBA8)
    uint32_t material_texture_;           ///< Roughness + AO (RG8)
    uint32_t depth_texture_;              ///< Depth (R32F)

    int width_;                           ///< Buffer width
    int height_;                          ///< Buffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_GBUFFER_H
