#ifndef ARE_INCLUDE_RESOURCE_BUFFER_H
#define ARE_INCLUDE_RESOURCE_BUFFER_H

#include "basic/types.h"

namespace are {

// Buffer usage hint
enum class BufferUsage {
	STATIC_DRAW,
	DYNAMIC_DRAW,
	STREAM_DRAW
};

// Buffer type
enum class BufferType {
	VERTEX_BUFFER,
	INDEX_BUFFER,
	UNIFORM_BUFFER,
	SHADER_STORAGE_BUFFER
};

// GPU buffer resource
class Buffer {
public:
	// Constructor
	Buffer();

	Buffer(const Buffer &) = delete;
	Buffer &operator=(const Buffer &) = delete;

	Buffer(Buffer &&other) noexcept;
	Buffer &operator=(Buffer &&other) noexcept;

	// Destructor
	~Buffer();

	/*
	 * @brief Create buffer
	 * @param type Buffer type
	 * @param size Buffer size in bytes
	 * @param data Initial data (nullptr for empty buffer)
	 * @param usage Usage hint
	 * @return True if creation succeeded
	 */
	bool create(BufferType type, size_t size, const void *data, BufferUsage usage);

	/*
	 * @brief Update buffer data
	 * @param offset Offset in bytes
	 * @param size Size in bytes
	 * @param data Data to upload
	 */
	void update(size_t offset, size_t size, const void *data);

	// Bind buffer
	void bind() const;

	/*
	 * @brief Bind buffer to binding point (for UBO/SSBO)
	 * @param binding_point Binding point index
	 */
	void bind_base(uint binding_point) const;

	// Unbind buffer
	void unbind() const;

	// Release buffer resources
	void release();

	/*
	 * @brief Get buffer handle
	 * @return Buffer handle
	 */
	BufferHandle get_handle() const {
		return handle_;
	}

	/*
	 * @brief Get buffer size
	 * @return Size in bytes
	 */
	size_t get_size() const {
		return size_;
	}

	/*
	 * @brief Get buffer type
	 * @return Buffer type
	 */
	BufferType get_type() const {
		return type_;
	}

	/*
	 * @brief Check if buffer is valid
	 * @return True if valid
	 */
	bool is_valid() const {
		return handle_ != INVALID_HANDLE;
	}

private:
	BufferHandle handle_;
	BufferType type_;
	size_t size_;
	BufferUsage usage_;
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_BUFFER_H
