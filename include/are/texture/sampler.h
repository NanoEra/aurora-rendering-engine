/**
 * @file sampler.h
 * @brief Texture sampling utilities
 */

#ifndef ARE_INCLUDE_TEXTURE_SAMPLER_H
#define ARE_INCLUDE_TEXTURE_SAMPLER_H

#include <are/core/types.h>

namespace are {

// Forward declaration
class Texture;
class TextureWrap;

/**
 * @class Sampler
 * @brief Texture sampling utilities for CPU ray tracing
 * 
 * Provides bilinear filtering and wrapping modes for CPU-side texture access.
 */
class Sampler {
public:
    /**
     * @brief Sample texture at UV coordinates
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample(const Texture& texture, const Vec2& uv);

    /**
     * @brief Sample texture with bilinear filtering
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample_bilinear(const Texture& texture, const Vec2& uv);

    /**
     * @brief Sample texture at nearest pixel
     * @param texture Texture to sample
     * @param uv UV coordinates
     * @return Sampled color (RGBA)
     */
    static Vec4 sample_nearest(const Texture& texture, const Vec2& uv);

private:
    static Vec2 apply_wrap(const Vec2& uv, TextureWrap wrap);
};

} // namespace are

#endif // ARE_INCLUDE_TEXTURE_SAMPLER_H
