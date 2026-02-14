#include "core/denoiser.h"
#include "basic/constants.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

Denoiser::Denoiser(uint width, uint height)
    : width_(width)
    , height_(height)
    , output_texture_(INVALID_HANDLE)
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
        glDeleteTextures(1, &output_texture_);
        output_texture_ = INVALID_HANDLE;
    }

    initialized_ = false;
}

void Denoiser::resize(uint width, uint height) {
    if (width == width_ && height == height_) return;
    width_ = width;
    height_ = height;

    if (!initialized_) return;

    if (output_texture_ != INVALID_HANDLE) {
        glDeleteTextures(1, &output_texture_);
        output_texture_ = INVALID_HANDLE;
    }
    create_output_texture_();
}

TextureHandle Denoiser::denoise(TextureHandle input_texture, int radius) {
    if (!initialized_) return input_texture;

    radius = (radius < 0) ? 0 : radius;

    shader_->use();

    glBindImageTexture(0, input_texture, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(1, output_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    shader_->set_int("u_radius", radius);

    uint groups_x = (width_ + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
    uint groups_y = (height_ + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;
    glDispatchCompute(groups_x, groups_y, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    return output_texture_;
}

void Denoiser::create_output_texture_() {
    glGenTextures(1, &output_texture_);
    glBindTexture(GL_TEXTURE_2D, output_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

} // namespace are
