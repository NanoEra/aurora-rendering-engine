#include "resource/resource_manager.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

ResourceManager::ResourceManager()
	: initialized_(false) {
}

ResourceManager::~ResourceManager() {
	release();
}

ResourceManager &ResourceManager::instance() {
	static ResourceManager manager;
	return manager;
}

bool ResourceManager::initialize() {
	if (initialized_) {
		ARE_LOG_WARN("ResourceManager already initialized");
		return true;
	}

	initialized_ = true;
	ARE_LOG_INFO("ResourceManager initialized successfully");
	return true;
}

void ResourceManager::release() {
	if (!initialized_)
		return;

	ARE_LOG_INFO("Releasing ResourceManager resources...");

	// Release all textures
	for (auto &[handle, resource] : textures_) {
		glDeleteTextures(1, &resource.gl_handle);
	}
	textures_.clear();

	// Release all buffers
	for (auto &[handle, resource] : buffers_) {
		glDeleteBuffers(1, &resource.gl_handle);
	}
	buffers_.clear();

	// Release all framebuffers and their attachments
	for (auto &[handle, resource] : framebuffers_) {
		// Attachments are owned by textures_ map, already deleted
		glDeleteFramebuffers(1, &resource.gl_handle);
	}
	framebuffers_.clear();

	initialized_ = false;
	ARE_LOG_INFO("ResourceManager released");
}

// === Texture Management ===

TextureHandle ResourceManager::create_texture(const TextureDescription &desc) {
	TextureHandle handle;
	glGenTextures(1, &handle);
	glBindTexture(GL_TEXTURE_2D, handle);

	// Convert format to GL types
	GLenum gl_internal_format = GL_RGBA8;
	GLenum gl_format = GL_RGBA;
	GLenum gl_type = GL_UNSIGNED_BYTE;

	switch (desc.format) {
	case TextureFormat::R8:
		gl_internal_format = GL_R8;
		gl_format = GL_RED;
		gl_type = GL_UNSIGNED_BYTE;
		break;
	case TextureFormat::RG8:
		gl_internal_format = GL_RG8;
		gl_format = GL_RG;
		gl_type = GL_UNSIGNED_BYTE;
		break;
	case TextureFormat::RGB8:
		gl_internal_format = GL_RGB8;
		gl_format = GL_RGB;
		gl_type = GL_UNSIGNED_BYTE;
		break;
	case TextureFormat::RGBA8:
		gl_internal_format = GL_RGBA8;
		gl_format = GL_RGBA;
		gl_type = GL_UNSIGNED_BYTE;
		break;
	case TextureFormat::RGBA32F:
		gl_internal_format = GL_RGBA32F;
		gl_format = GL_RGBA;
		gl_type = GL_FLOAT;
		break;
	case TextureFormat::R32F:
		gl_internal_format = GL_R32F;
		gl_format = GL_RED;
		gl_type = GL_FLOAT;
		break;
	case TextureFormat::RG32F:
		gl_internal_format = GL_RG32F;
		gl_format = GL_RG;
		gl_type = GL_FLOAT;
		break;
	case TextureFormat::DEPTH24_STENCIL8:
		gl_internal_format = GL_DEPTH24_STENCIL8;
		gl_format = GL_DEPTH_STENCIL;
		gl_type = GL_UNSIGNED_INT_24_8;
		break;
	default:
		gl_internal_format = GL_RGBA8;
		gl_format = GL_RGBA;
		gl_type = GL_UNSIGNED_BYTE;
		break;
	}

	glTexImage2D(GL_TEXTURE_2D, 0, gl_internal_format, desc.width, desc.height,
		0, gl_format, gl_type, nullptr);

	// Set filter
	GLenum gl_min_filter = GL_LINEAR;
	GLenum gl_mag_filter = GL_LINEAR;

	switch (desc.filter) {
	case TextureFilter::NEAREST:
		gl_min_filter = GL_NEAREST;
		gl_mag_filter = GL_NEAREST;
		break;
	case TextureFilter::LINEAR:
		gl_min_filter = GL_LINEAR;
		gl_mag_filter = GL_LINEAR;
		break;
	case TextureFilter::NEAREST_MIPMAP_NEAREST:
		gl_min_filter = GL_NEAREST_MIPMAP_NEAREST;
		gl_mag_filter = GL_NEAREST;
		break;
	case TextureFilter::LINEAR_MIPMAP_NEAREST:
		gl_min_filter = GL_LINEAR_MIPMAP_NEAREST;
		gl_mag_filter = GL_LINEAR;
		break;
	case TextureFilter::NEAREST_MIPMAP_LINEAR:
		gl_min_filter = GL_NEAREST_MIPMAP_LINEAR;
		gl_mag_filter = GL_NEAREST;
		break;
	case TextureFilter::LINEAR_MIPMAP_LINEAR:
		gl_min_filter = GL_LINEAR_MIPMAP_LINEAR;
		gl_mag_filter = GL_LINEAR;
		break;
	default:
		gl_min_filter = GL_LINEAR;
		gl_mag_filter = GL_LINEAR;
		break;
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, gl_min_filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, gl_mag_filter);

	// Set wrap
	GLenum gl_wrap = GL_REPEAT;
	switch (desc.wrap) {
	case TextureWrap::REPEAT:
		gl_wrap = GL_REPEAT;
		break;
	case TextureWrap::MIRRORED_REPEAT:
		gl_wrap = GL_MIRRORED_REPEAT;
		break;
	case TextureWrap::CLAMP_TO_EDGE:
		gl_wrap = GL_CLAMP_TO_EDGE;
		break;
	case TextureWrap::CLAMP_TO_BORDER:
		gl_wrap = GL_CLAMP_TO_BORDER;
		break;
	default:
		gl_wrap = GL_REPEAT;
		break;
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, gl_wrap);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, gl_wrap);

	// Generate mipmaps if requested
	if (desc.generate_mipmaps) {
		glGenerateMipmap(GL_TEXTURE_2D);
	}

	glBindTexture(GL_TEXTURE_2D, 0);

	TextureResource resource;
	resource.gl_handle = handle;
	resource.width = desc.width;
	resource.height = desc.height;
	resource.format = desc.format;
	textures_[handle] = resource;

	ARE_LOG_DEBUG("Texture created: " + std::to_string(handle));
	return handle;
}

TextureHandle ResourceManager::create_texture(uint width, uint height, TextureFormat format) {
	TextureDescription desc;
	desc.width = width;
	desc.height = height;
	desc.format = format;
	return create_texture(desc);
}

TextureHandle ResourceManager::create_texture(const std::string &path) {
	// Use Texture class for file loading (stb_image)
	auto tex = std::make_shared<Texture>();
	if (!tex->load_from_file(path, true)) {
		ARE_LOG_ERROR("Failed to load texture: " + path);
		return INVALID_HANDLE;
	}

	TextureHandle handle = tex->get_handle();

	// Register in manager (but don't own - Texture class manages lifetime)
	TextureResource resource;
	resource.gl_handle = handle;
	resource.width = tex->get_width();
	resource.height = tex->get_height();
	resource.format = tex->get_format();
	textures_[handle] = resource;

	return handle;
}

void ResourceManager::destroy_texture(TextureHandle handle) {
	auto it = textures_.find(handle);
	if (it == textures_.end()) {
		ARE_LOG_WARN("Attempting to destroy unknown texture: " + std::to_string(handle));
		return;
	}

	glDeleteTextures(1, &it->second.gl_handle);
	textures_.erase(it);
	ARE_LOG_DEBUG("Texture destroyed: " + std::to_string(handle));
}

// === Buffer Management ===

BufferHandle ResourceManager::create_buffer(const BufferDescription &desc) {
	BufferHandle handle;
	glGenBuffers(1, &handle);

	GLenum gl_type = GL_ARRAY_BUFFER;
	switch (desc.type) {
	case BufferType::VERTEX_BUFFER:
		gl_type = GL_ARRAY_BUFFER;
		break;
	case BufferType::INDEX_BUFFER:
		gl_type = GL_ELEMENT_ARRAY_BUFFER;
		break;
	case BufferType::UNIFORM_BUFFER:
		gl_type = GL_UNIFORM_BUFFER;
		break;
	case BufferType::SHADER_STORAGE_BUFFER:
		gl_type = GL_SHADER_STORAGE_BUFFER;
		break;
	default:
		gl_type = GL_ARRAY_BUFFER;
		break;
	}

	GLenum gl_usage = GL_STATIC_DRAW;
	switch (desc.usage) {
	case BufferUsage::STATIC_DRAW:
		gl_usage = GL_STATIC_DRAW;
		break;
	case BufferUsage::DYNAMIC_DRAW:
		gl_usage = GL_DYNAMIC_DRAW;
		break;
	case BufferUsage::STREAM_DRAW:
		gl_usage = GL_STREAM_DRAW;
		break;
	default:
		gl_usage = GL_STATIC_DRAW;
		break;
	}

	glBindBuffer(gl_type, handle);
	glBufferData(gl_type, desc.size, desc.data, gl_usage);
	glBindBuffer(gl_type, 0);

	BufferResource resource;
	resource.gl_handle = handle;
	resource.type = desc.type;
	resource.usage = desc.usage;
	resource.size = desc.size;
	buffers_[handle] = resource;

	ARE_LOG_DEBUG("Buffer created: " + std::to_string(handle) + " size: " + std::to_string(desc.size));
	return handle;
}

void ResourceManager::update_buffer(BufferHandle handle, size_t offset, size_t size, const void *data) {
	auto it = buffers_.find(handle);
	if (it == buffers_.end()) {
		ARE_LOG_ERROR("Attempting to update unknown buffer: " + std::to_string(handle));
		return;
	}

	GLenum gl_type = GL_ARRAY_BUFFER;
	switch (it->second.type) {
	case BufferType::VERTEX_BUFFER:
		gl_type = GL_ARRAY_BUFFER;
		break;
	case BufferType::INDEX_BUFFER:
		gl_type = GL_ELEMENT_ARRAY_BUFFER;
		break;
	case BufferType::UNIFORM_BUFFER:
		gl_type = GL_UNIFORM_BUFFER;
		break;
	case BufferType::SHADER_STORAGE_BUFFER:
		gl_type = GL_SHADER_STORAGE_BUFFER;
		break;
	default:
		gl_type = GL_ARRAY_BUFFER;
		break;
	}

	glBindBuffer(gl_type, handle);
	glBufferSubData(gl_type, offset, size, data);
	glBindBuffer(gl_type, 0);
}

void ResourceManager::destroy_buffer(BufferHandle handle) {
	auto it = buffers_.find(handle);
	if (it == buffers_.end()) {
		ARE_LOG_WARN("Attempting to destroy unknown buffer: " + std::to_string(handle));
		return;
	}

	glDeleteBuffers(1, &it->second.gl_handle);
	buffers_.erase(it);
	ARE_LOG_DEBUG("Buffer destroyed: " + std::to_string(handle));
}

// === Framebuffer Management ===

FramebufferHandle ResourceManager::create_framebuffer(const FramebufferDescription &desc) {
	FramebufferHandle fbo;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	FramebufferResource resource;
	resource.gl_handle = fbo;
	resource.width = desc.width;
	resource.height = desc.height;

	// Create color attachments
	for (uint i = 0; i < desc.color_attachment_count; ++i) {
		TextureDescription tex_desc;
		tex_desc.width = desc.width;
		tex_desc.height = desc.height;
		tex_desc.format = TextureFormat::RGBA32F;
		tex_desc.filter = TextureFilter::NEAREST;
		tex_desc.wrap = TextureWrap::CLAMP_TO_EDGE;

		TextureHandle color_tex = create_texture(tex_desc);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i,
			GL_TEXTURE_2D, color_tex, 0);

		resource.color_attachments.push_back(color_tex);
	}

	// Create depth attachment if requested
	if (desc.create_depth) {
		TextureDescription depth_desc;
		depth_desc.width = desc.width;
		depth_desc.height = desc.height;
		depth_desc.format = TextureFormat::DEPTH24_STENCIL8;
		depth_desc.filter = TextureFilter::NEAREST;
		depth_desc.wrap = TextureWrap::CLAMP_TO_EDGE;

		TextureHandle depth_tex = create_texture(depth_desc);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
			GL_TEXTURE_2D, depth_tex, 0);

		resource.depth_attachment = depth_tex;
	}

	// Set draw buffers if color attachments exist
	if (desc.color_attachment_count > 0) {
		std::vector<GLenum> draw_buffers(desc.color_attachment_count);
		for (uint i = 0; i < desc.color_attachment_count; ++i) {
			draw_buffers[i] = GL_COLOR_ATTACHMENT0 + i;
		}
		glDrawBuffers(desc.color_attachment_count, draw_buffers.data());
	}

	// Check completeness
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		ARE_LOG_ERROR("Framebuffer is not complete!");
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		destroy_framebuffer(fbo);
		return INVALID_HANDLE;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	framebuffers_[fbo] = resource;

	ARE_LOG_DEBUG("Framebuffer created: " + std::to_string(fbo));
	return fbo;
}

TextureHandle ResourceManager::get_framebuffer_color_attachment(FramebufferHandle fbo, uint index) {
	auto it = framebuffers_.find(fbo);
	if (it == framebuffers_.end()) {
		ARE_LOG_ERROR("Invalid framebuffer handle: " + std::to_string(fbo));
		return INVALID_HANDLE;
	}

	if (index >= it->second.color_attachments.size()) {
		ARE_LOG_ERROR("Color attachment index out of range: " + std::to_string(index));
		return INVALID_HANDLE;
	}

	return it->second.color_attachments[index];
}

TextureHandle ResourceManager::get_framebuffer_depth_attachment(FramebufferHandle fbo) {
	auto it = framebuffers_.find(fbo);
	if (it == framebuffers_.end()) {
		ARE_LOG_ERROR("Invalid framebuffer handle: " + std::to_string(fbo));
		return INVALID_HANDLE;
	}

	return it->second.depth_attachment;
}

void ResourceManager::destroy_framebuffer(FramebufferHandle fbo) {
	auto it = framebuffers_.find(fbo);
	if (it == framebuffers_.end()) {
		ARE_LOG_WARN("Attempting to destroy unknown framebuffer: " + std::to_string(fbo));
		return;
	}

	// Destroy color attachments
	for (auto &color_tex : it->second.color_attachments) {
		destroy_texture(color_tex);
	}

	// Destroy depth attachment
	if (it->second.depth_attachment != INVALID_HANDLE) {
		destroy_texture(it->second.depth_attachment);
	}

	glDeleteFramebuffers(1, &it->second.gl_handle);
	framebuffers_.erase(it);
	ARE_LOG_DEBUG("Framebuffer destroyed: " + std::to_string(fbo));
}

// === Texture Array Management ===

TextureHandle ResourceManager::create_texture_array(const TextureArrayDescription &desc) {
	if (desc.textures.empty()) {
		ARE_LOG_WARN("Creating empty texture array");
		return INVALID_HANDLE;
	}

	// Get dimensions from first texture
	int tex_width = desc.textures[0]->get_width();
	int tex_height = desc.textures[0]->get_height();

	// Create texture array
	GLuint tex_array;
	glGenTextures(1, &tex_array);
	glBindTexture(GL_TEXTURE_2D_ARRAY, tex_array);

	// Allocate storage
	glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, tex_width, tex_height,
		static_cast<int>(desc.textures.size()), 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

	// Copy each texture to array layer
	for (size_t i = 0; i < desc.textures.size(); ++i) {
		auto &tex = desc.textures[i];
		GLuint tex_handle = tex->get_handle();

		if (tex_handle == 0)
			continue;

		TextureFormat orig_format = tex->get_format();
		int orig_width = tex->get_width();
		int orig_height = tex->get_height();

		// Get format for reading
		GLenum orig_gl_format = GL_RGBA;
		int channels = 4;
		switch (orig_format) {
		case TextureFormat::R8:
			orig_gl_format = GL_RED;
			channels = 1;
			break;
		case TextureFormat::RG8:
			orig_gl_format = GL_RG;
			channels = 2;
			break;
		case TextureFormat::RGB8:
			orig_gl_format = GL_RGB;
			channels = 3;
			break;
		case TextureFormat::RGBA8:
			orig_gl_format = GL_RGBA;
			channels = 4;
			break;
		default:
			orig_gl_format = GL_RGBA;
			channels = 4;
			break;
		}

		// Read texture data
		std::vector<uint8_t> orig_pixels(orig_width * orig_height * channels);
		glBindTexture(GL_TEXTURE_2D, tex_handle);
		glGetTexImage(GL_TEXTURE_2D, 0, orig_gl_format, GL_UNSIGNED_BYTE, orig_pixels.data());
		glBindTexture(GL_TEXTURE_2D, 0);

		// Convert to RGBA
		std::vector<uint8_t> pixels(orig_width * orig_height * 4, 255);
		for (int y = 0; y < orig_height; ++y) {
			for (int x = 0; x < orig_width; ++x) {
				int src_idx = (y * orig_width + x) * channels;
				int dst_idx = (y * orig_width + x) * 4;
				pixels[dst_idx + 0] = orig_pixels[src_idx + 0];
				pixels[dst_idx + 1] = (channels >= 2) ? orig_pixels[src_idx + 1] : orig_pixels[src_idx + 0];
				pixels[dst_idx + 2] = (channels >= 3) ? orig_pixels[src_idx + 2] : orig_pixels[src_idx + 0];
				pixels[dst_idx + 3] = (channels >= 4) ? orig_pixels[src_idx + 3] : 255;
			}
		}

		// Upload to array layer
		glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, static_cast<int>(i),
			tex_width, tex_height, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
	}

	// Set filter
	GLenum gl_min_filter = GL_LINEAR;
	GLenum gl_mag_filter = GL_LINEAR;
	switch (desc.filter) {
	case TextureFilter::NEAREST:
		gl_min_filter = GL_NEAREST;
		gl_mag_filter = GL_NEAREST;
		break;
	case TextureFilter::LINEAR:
		gl_min_filter = GL_LINEAR;
		gl_mag_filter = GL_LINEAR;
		break;
	default:
		gl_min_filter = GL_LINEAR;
		gl_mag_filter = GL_LINEAR;
		break;
	}

	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, gl_min_filter);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, gl_mag_filter);

	// Set wrap
	GLenum gl_wrap = GL_REPEAT;
	switch (desc.wrap) {
	case TextureWrap::REPEAT:
		gl_wrap = GL_REPEAT;
		break;
	case TextureWrap::CLAMP_TO_EDGE:
		gl_wrap = GL_CLAMP_TO_EDGE;
		break;
	default:
		gl_wrap = GL_REPEAT;
		break;
	}

	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, gl_wrap);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, gl_wrap);

	glGenerateMipmap(GL_TEXTURE_2D_ARRAY);

	glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

	TextureResource resource;
	resource.gl_handle = tex_array;
	resource.width = tex_width;
	resource.height = tex_height;
	resource.format = TextureFormat::RGBA8;
	textures_[tex_array] = resource;

	ARE_LOG_DEBUG("Texture array created: " + std::to_string(tex_array) + " layers: " + std::to_string(desc.textures.size()));
	return tex_array;
}

void ResourceManager::destroy_texture_array(TextureHandle handle) {
	destroy_texture(handle);
}

// === VAO Management ===

VertexArrayHandle ResourceManager::create_vertex_array() {
	VertexArrayHandle vao;
	glGenVertexArrays(1, &vao);
	ARE_LOG_DEBUG("Vertex array created: " + std::to_string(vao));
	return vao;
}

void ResourceManager::destroy_vertex_array(VertexArrayHandle vao) {
	glDeleteVertexArrays(1, &vao);
	ARE_LOG_DEBUG("Vertex array destroyed: " + std::to_string(vao));
}

// === Binding Management ===

void ResourceManager::bind_buffer(BufferHandle buffer, uint binding_point) {
	auto it = buffers_.find(buffer);
	if (it == buffers_.end()) {
		ARE_LOG_ERROR("Attempting to bind unknown buffer: " + std::to_string(buffer));
		return;
	}

	GLenum gl_type = GL_ARRAY_BUFFER;
	switch (it->second.type) {
	case BufferType::UNIFORM_BUFFER:
		gl_type = GL_UNIFORM_BUFFER;
		break;
	case BufferType::SHADER_STORAGE_BUFFER:
		gl_type = GL_SHADER_STORAGE_BUFFER;
		break;
	default:
		ARE_LOG_ERROR("Buffer type not supported for bind_buffer");
		return;
	}

	glBindBufferBase(gl_type, binding_point, buffer);
}

void ResourceManager::bind_image_texture(TextureHandle texture, uint binding, bool read, bool write) {
	GLenum access = GL_READ_ONLY;
	if (read && write) {
		access = GL_READ_WRITE;
	} else if (write) {
		access = GL_WRITE_ONLY;
	}

	glBindImageTexture(binding, texture, 0, GL_FALSE, 0, access, GL_RGBA32F);
}

void ResourceManager::bind_texture_to_unit(TextureHandle texture, uint unit) {
	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, texture);
}

// === Query ===

bool ResourceManager::is_texture_valid(TextureHandle handle) const {
	return textures_.find(handle) != textures_.end();
}

bool ResourceManager::is_buffer_valid(BufferHandle handle) const {
	return buffers_.find(handle) != buffers_.end();
}

bool ResourceManager::is_framebuffer_valid(FramebufferHandle handle) const {
	return framebuffers_.find(handle) != framebuffers_.end();
}

} // namespace are
