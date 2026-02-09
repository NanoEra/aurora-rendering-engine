#include "resource/buffer.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

namespace {
    GLenum get_gl_buffer_type(BufferType type) {
        switch (type) {
            case BufferType::VERTEX_BUFFER: return GL_ARRAY_BUFFER;
            case BufferType::INDEX_BUFFER: return GL_ELEMENT_ARRAY_BUFFER;
            case BufferType::UNIFORM_BUFFER: return GL_UNIFORM_BUFFER;
            case BufferType::SHADER_STORAGE_BUFFER: return GL_SHADER_STORAGE_BUFFER;
            default: return GL_ARRAY_BUFFER;
        }
    }
    
    GLenum get_gl_usage(BufferUsage usage) {
        switch (usage) {
            case BufferUsage::STATIC_DRAW: return GL_STATIC_DRAW;
            case BufferUsage::DYNAMIC_DRAW: return GL_DYNAMIC_DRAW;
            case BufferUsage::STREAM_DRAW: return GL_STREAM_DRAW;
            default: return GL_STATIC_DRAW;
        }
    }
}

Buffer::Buffer()
    : handle_(INVALID_HANDLE)
    , type_(BufferType::VERTEX_BUFFER)
    , size_(0)
    , usage_(BufferUsage::STATIC_DRAW) {
}

Buffer::~Buffer() {
    // Don't auto-release, let user control lifetime
}

bool Buffer::create(BufferType type, size_t size, const void* data, BufferUsage usage) {
    if (handle_ != INVALID_HANDLE) {
        Logger::warning("Buffer already created, releasing old buffer");
        release();
    }
    
    type_ = type;
    size_ = size;
    usage_ = usage;
    
    glGenBuffers(1, &handle_);
    
    GLenum gl_type = get_gl_buffer_type(type);
    GLenum gl_usage = get_gl_usage(usage);
    
    glBindBuffer(gl_type, handle_);
    glBufferData(gl_type, size, data, gl_usage);
    glBindBuffer(gl_type, 0);
    
    Logger::info("Buffer created successfully");
    return true;
}

void Buffer::update(size_t offset, size_t size, const void* data) {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot update invalid buffer");
        return;
    }
    
    if (offset + size > size_) {
        Logger::error("Buffer update out of bounds");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    
    glBindBuffer(gl_type, handle_);
    glBufferSubData(gl_type, offset, size, data);
    glBindBuffer(gl_type, 0);
}

void Buffer::bind() const {
    if (handle_ == INVALID_HANDLE) {
        Logger::warning("Attempting to bind invalid buffer");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBuffer(gl_type, handle_);
}

void Buffer::bind_base(uint binding_point) const {
    if (handle_ == INVALID_HANDLE) {
        Logger::warning("Attempting to bind invalid buffer");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBufferBase(gl_type, binding_point, handle_);
}

void Buffer::unbind() const {
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBuffer(gl_type, 0);
}

void Buffer::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteBuffers(1, &handle_);
        handle_ = INVALID_HANDLE;
    }
    
    size_ = 0;
}

} // namespace are
