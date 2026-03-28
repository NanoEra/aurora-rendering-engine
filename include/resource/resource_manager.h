#ifndef ARE_INCLUDE_RESOURCE_RESOURCE_MANAGER_H
#define ARE_INCLUDE_RESOURCE_RESOURCE_MANAGER_H

#include "basic/types.h"
#include "resource/buffer.h"
#include "resource/texture.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace are {

// Texture creation parameters
struct TextureDescription {
	uint width = 1;
	uint height = 1;
	TextureFormat format = TextureFormat::RGBA8;
	TextureFilter filter = TextureFilter::LINEAR;
	TextureWrap wrap = TextureWrap::CLAMP_TO_EDGE;
	bool generate_mipmaps = false;
};

// Buffer creation parameters
struct BufferDescription {
	BufferType type = BufferType::VERTEX_BUFFER;
	BufferUsage usage = BufferUsage::STATIC_DRAW;
	size_t size = 0;
	const void *data = nullptr;
};

// Texture array creation parameters
struct TextureArrayDescription {
	uint width = 1;
	uint height = 1;
	TextureFormat format = TextureFormat::RGBA8;
	TextureFilter filter = TextureFilter::LINEAR;
	TextureWrap wrap = TextureWrap::REPEAT;
	std::vector<std::shared_ptr<Texture>> textures;
};

// Framebuffer creation parameters
struct FramebufferDescription {
	uint width = 1;
	uint height = 1;
	bool create_depth = true;
	uint color_attachment_count = 0;
};

// GPU resource manager - centralized resource creation and lifecycle
class ResourceManager {
public:
	static ResourceManager &instance();

	// Initialization and release
	bool initialize();
	void release();

	// === Texture Management ===
	TextureHandle create_texture(const TextureDescription &desc);
	TextureHandle create_texture(uint width, uint height, TextureFormat format);
	TextureHandle create_texture(const std::string &path);
	void destroy_texture(TextureHandle handle);

	// === Buffer Management ===
	BufferHandle create_buffer(const BufferDescription &desc);
	void update_buffer(BufferHandle handle, size_t offset, size_t size, const void *data);
	void destroy_buffer(BufferHandle handle);

	// === Framebuffer Management ===
	FramebufferHandle create_framebuffer(const FramebufferDescription &desc);
	TextureHandle get_framebuffer_color_attachment(FramebufferHandle fbo, uint index);
	TextureHandle get_framebuffer_depth_attachment(FramebufferHandle fbo);
	void destroy_framebuffer(FramebufferHandle fbo);

	// === Texture Array Management ===
	TextureHandle create_texture_array(const TextureArrayDescription &desc);
	void destroy_texture_array(TextureHandle handle);

	// === VAO Management ===
	VertexArrayHandle create_vertex_array();
	void destroy_vertex_array(VertexArrayHandle vao);

	// === Binding Management ===
	void bind_buffer(BufferHandle buffer, uint binding_point);
	void bind_image_texture(TextureHandle texture, uint binding, bool read, bool write);
	void bind_texture_to_unit(TextureHandle texture, uint unit);

	// === Query ===
	bool is_texture_valid(TextureHandle handle) const;
	bool is_buffer_valid(BufferHandle handle) const;
	bool is_framebuffer_valid(FramebufferHandle handle) const;

private:
	ResourceManager();
	~ResourceManager();

	struct TextureResource {
		TextureHandle gl_handle;
		uint width;
		uint height;
		TextureFormat format;
	};

	struct BufferResource {
		BufferHandle gl_handle;
		BufferType type;
		BufferUsage usage;
		size_t size;
	};

	struct FramebufferResource {
		FramebufferHandle gl_handle;
		uint width;
		uint height;
		std::vector<TextureHandle> color_attachments;
		TextureHandle depth_attachment;
	};

	std::unordered_map<TextureHandle, TextureResource> textures_;
	std::unordered_map<BufferHandle, BufferResource> buffers_;
	std::unordered_map<FramebufferHandle, FramebufferResource> framebuffers_;

	bool initialized_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_RESOURCE_MANAGER_H
