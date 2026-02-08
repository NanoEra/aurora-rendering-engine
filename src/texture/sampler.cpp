/**
 * @file sampler.cpp
 * @brief Texture sampling utilities implementation
 */

#include <are/texture/texture.h>
#include <are/texture/sampler.h>
#include <are/core/logger.h>

namespace are {

Vec4 Sampler::sample(const Texture& texture, const Vec2& uv) {
    // Default to bilinear sampling
    return sample_bilinear(texture, uv);
}

Vec4 Sampler::sample_bilinear(const Texture& texture, const Vec2& uv) {
    // TODO: Implement CPU-side bilinear sampling
    // For now, return white
    ARE_LOG_WARN("CPU texture sampling not implemented");
    return Vec4(1.0f);
}

Vec4 Sampler::sample_nearest(const Texture& texture, const Vec2& uv) {
    // TODO: Implement CPU-side nearest sampling
    ARE_LOG_WARN("CPU texture sampling not implemented");
    return Vec4(1.0f);
}

Vec2 Sampler::apply_wrap(const Vec2& uv, TextureWrap wrap) {
    // TODO: Implement wrapping modes
    return uv;
}

} // namespace are
