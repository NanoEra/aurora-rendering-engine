#include "core/denoiser.h"
#include "basic/constants.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

Denoiser::Denoiser(uint width, uint height)
    : width_(width)
    , height_(height)
    , output_texture_(INVALID_HANDLE)
    , history_texture_(INVALID_HANDLE)
    , history_valid_(false)
    , initialized_(false) {
}

Denoiser::~Denoiser() {
    release();
}

bool Denoiser::initialize(const std::shared_ptr<Shader>& shader) {
    if (initialized_) return true;

    if (!shader || !shader->is_valid()) {
        ARE_LOG_ERROR("Invalid denoise shader");
        return false;
    }

    shader_ = shader;
    create_output_texture_();

    initialized_ = true;
    ARE_LOG_INFO("Denoiser initialized");
    return true;
}

void Denoiser::release() {
    if (!initialized_) return;

    shader_.reset();

    if (output_texture_ != INVALID_HANDLE) {
        ResourceManager::instance().destroy_texture(output_texture_);
        output_texture_ = INVALID_HANDLE;
    }

    if (history_texture_ != INVALID_HANDLE) {
        ResourceManager::instance().destroy_texture(history_texture_);
        history_texture_ = INVALID_HANDLE;
    }

    history_valid_ = false;
    initialized_ = false;
}

void Denoiser::resize(uint width, uint height) {
    if (width == width_ && height == height_) return;
    width_ = width;
    height_ = height;

    if (!initialized_) return;

    ResourceManager &rm = ResourceManager::instance();

    if (output_texture_ != INVALID_HANDLE) {
        rm.destroy_texture(output_texture_);
        output_texture_ = INVALID_HANDLE;
    }

    if (history_texture_ != INVALID_HANDLE) {
        rm.destroy_texture(history_texture_);
        history_texture_ = INVALID_HANDLE;
    }

    history_valid_ = false;
    create_output_texture_();
}

TextureHandle Denoiser::denoise(TextureHandle input_texture, int radius, float temporal_weight) {
    if (!initialized_) return input_texture;

    radius = (radius < 0) ? 0 : radius;
    temporal_weight = (temporal_weight < 0.0f) ? 0.0f : ((temporal_weight > 1.0f) ? 1.0f : temporal_weight);

    shader_->use();

    glBindImageTexture(0, input_texture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(1, output_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    shader_->set_int("u_radius", radius);
    shader_->set_float("u_temporal_weight", temporal_weight);
    shader_->set_bool("u_has_history", history_valid_ && temporal_weight > 0.0f);

    // Bind history texture if available and temporal accumulation is enabled
    if (history_valid_ && temporal_weight > 0.0f) {
        glBindImageTexture(2, history_texture_, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    }

    uint groups_x = (width_ + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
    uint groups_y = (height_ + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;
    glDispatchCompute(groups_x, groups_y, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // Copy output to history for next frame (if temporal accumulation is enabled)
    if (temporal_weight > 0.0f) {
        // Create history texture if it doesn't exist
        if (history_texture_ == INVALID_HANDLE) {
            ResourceManager &rm = ResourceManager::instance();
            TextureDescription desc;
            desc.width = width_;
            desc.height = height_;
            desc.format = TextureFormat::RGBA32F;
            desc.filter = TextureFilter::NEAREST;
            desc.wrap = TextureWrap::CLAMP_TO_EDGE;
            history_texture_ = rm.create_texture(desc);
        }

        // Copy output to history using GPU (blit or compute)
        // For simplicity, we'll just bind output as history for next frame
        // This requires double buffering - let's swap the textures
        std::swap(output_texture_, history_texture_);
        history_valid_ = true;

        // Return the new output (which was history before swap)
        return output_texture_;
    }

    return output_texture_;
}

void Denoiser::reset_history() {
    history_valid_ = false;
}

void Denoiser::create_output_texture_() {
    ResourceManager &rm = ResourceManager::instance();
    TextureDescription desc;
    desc.width = width_;
    desc.height = height_;
    desc.format = TextureFormat::RGBA32F;
    desc.filter = TextureFilter::NEAREST;
    desc.wrap = TextureWrap::CLAMP_TO_EDGE;
    output_texture_ = rm.create_texture(desc);
}

} // namespace are
