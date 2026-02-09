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
 * Attachment layout:
 * 0: position (RGB16F)
 * 1: normal (RGB16F)
 * 2: albedo+metallic (RGBA8)
 * 3: roughness+ao (RG8)
 * 4: primitive id (R32UI)
 * Depth: depth texture (DEPTH_COMPONENT24)
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
     * @param index Texture index
     * @param texture_unit Texture unit to bind to
     */
    void bind_texture(int index, int texture_unit);

    // Texture getters
    uint32_t get_position_texture() const { return position_texture_; }
    uint32_t get_normal_texture() const { return normal_texture_; }
    uint32_t get_albedo_texture() const { return albedo_texture_; }
    uint32_t get_material_texture() const { return material_texture_; }
    uint32_t get_depth_texture() const { return depth_texture_; }
    uint32_t get_primitive_id_texture() const { return primitive_id_texture_; }

    // Dimensions
    int get_width() const { return width_; }
    int get_height() const { return height_; }

    /**
     * @brief Read pixel data from G-Buffer
     *
     * Index mapping:
     * - 0: position (RGB16F) -> GL_RGB/GL_FLOAT
     * - 1: normal (RGB16F) -> GL_RGB/GL_FLOAT
     * - 2: albedo_metallic (RGBA8) -> GL_RGBA/GL_UNSIGNED_BYTE
     * - 3: material (RG8) -> GL_RG/GL_UNSIGNED_BYTE
     * - 4: depth (DEPTH_COMPONENT24) -> GL_DEPTH_COMPONENT/GL_FLOAT
     * - 5: primitive id (R32UI) -> GL_RED_INTEGER/GL_UNSIGNED_INT
     *
     * @param index Buffer index
     * @param data Output data pointer
     */
    void read_pixels(int index, void* data);

private:
    void create_textures();
    void delete_textures();
    void create_framebuffer();

    uint32_t fbo_;                        ///< Framebuffer object
    uint32_t rbo_depth_;                  ///< Legacy depth renderbuffer (unused)

    uint32_t position_texture_;
    uint32_t normal_texture_;
    uint32_t albedo_texture_;
    uint32_t material_texture_;
    uint32_t depth_texture_;
    uint32_t primitive_id_texture_;

    int width_;
    int height_;
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_GBUFFER_H
