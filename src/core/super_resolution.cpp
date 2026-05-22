#include "core/super_resolution.h"
#include "basic/constants.h"
#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <cmath>
#include <glad/glad.h>
#include <string>

namespace are {

uint SuperResolution::compute_block_size_() const {
	return static_cast<uint>(std::sqrt(static_cast<double>(config_.scaling)));
}

SuperResolution::SuperResolution(uint full_width, uint full_height, const SuperResolutionConfig &config)
	: full_width_(full_width), full_height_(full_height), config_(config), current_jitter_frame_(0), low_res_rt_texture_(INVALID_HANDLE), accumulated_rt_texture_(INVALID_HANDLE), upscaled_texture_(INVALID_HANDLE) {
	uint block = compute_block_size_();
	low_res_w_ = full_width_ / block;
	low_res_h_ = full_height_ / block;
}

SuperResolution::~SuperResolution() {
	release();
}

bool SuperResolution::initialize(const std::shared_ptr<Shader> &shader) {
	if (initialized_) {
		ARE_LOG_WARN("Super resolution already initialized");
		return true;
	}

	if (!shader || !shader->is_valid()) {
		ARE_LOG_ERROR("Invalid shader");
		return false;
	}

	compute_shader_ = shader;
	create_textures_();
	initialized_ = true;
	ARE_LOG_INFO("Super resolution initialized: " + std::to_string(full_width_) + "x" + std::to_string(full_height_) + " scaling_px=" + std::to_string(config_.scaling) + " lowres=" + std::to_string(low_res_w_) + "x" + std::to_string(low_res_h_));
	return true;
}

void SuperResolution::release() {
	if (!initialized_) {
		return;
	}
	ResourceManager &rm = ResourceManager::instance();

	if (low_res_rt_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(low_res_rt_texture_);
		low_res_rt_texture_ = INVALID_HANDLE;
	}

	if (accumulated_rt_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(accumulated_rt_texture_);
		accumulated_rt_texture_ = INVALID_HANDLE;
	}

	if (upscaled_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(upscaled_texture_);
		upscaled_texture_ = INVALID_HANDLE;
	}

	initialized_ = false;
	ARE_LOG_INFO("SuperResolution released");
}

TextureHandle SuperResolution::upscale() {
	if (!initialized_ || !compute_shader_) {
		return INVALID_HANDLE;
	}

	compute_shader_->use();
	glBindImageTexture(1, accumulated_rt_texture_, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
	glBindImageTexture(2, upscaled_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	uint gx = (full_width_ + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
	uint gy = (full_height_ + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;
	glDispatchCompute(gx, gy, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	return upscaled_texture_;
}

void SuperResolution::advance_jitter_frame() {
	current_jitter_frame_ = (current_jitter_frame_ + 1) % config_.scaling;
}

void SuperResolution::reset_accumulation() {
	current_jitter_frame_ = 0;
	clear_accumulation_texture_();
	ARE_LOG_DEBUG("SuperResolution accumulation reset (frame " + std::to_string(current_jitter_frame_) + ")");
}

void SuperResolution::resize(uint full_width, uint full_height) {
	if (full_width == full_width_ && full_height == full_height_) {
		return;
	}
	full_width_ = full_width;
	full_height_ = full_height;
	uint block = compute_block_size_();
	low_res_w_ = full_width_ / block;
	low_res_h_ = full_height_ / block;
	if (!initialized_) {
		return;
	}

	ResourceManager &rm = ResourceManager::instance();
	if (low_res_rt_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(low_res_rt_texture_);
	}

	if (accumulated_rt_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(accumulated_rt_texture_);
	}

	if (upscaled_texture_ != INVALID_HANDLE) {
		rm.destroy_texture(upscaled_texture_);
	}

	create_textures_();
	ARE_LOG_INFO("SuperResolution resized to " + std::to_string(full_width) + "x" + std::to_string(full_height));
}

void SuperResolution::create_textures_() {
	ResourceManager &rm = ResourceManager::instance();
	low_res_rt_texture_ = rm.create_texture(low_res_w_, low_res_h_, TextureFormat::RGBA32F);
	accumulated_rt_texture_ = rm.create_texture(full_width_, full_height_, TextureFormat::RGBA32F);
	upscaled_texture_ = rm.create_texture(full_width_, full_height_, TextureFormat::RGBA32F);
	reset_accumulation();
}

void SuperResolution::clear_accumulation_texture_() const {
	// FBO-based clear — far more efficient than a full compute dispatch.
	// The texture is attached as a colour attachment and cleared to (0,0,0,0).
	GLuint fbo;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, accumulated_rt_texture_, 0);
	const GLfloat clear_color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glClearBufferfv(GL_COLOR, 0, clear_color);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &fbo);
}

} // namespace are
