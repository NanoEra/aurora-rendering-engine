/**
 * @file gl_context.cpp
 * @brief Implementation of OpenGL context management
 */

#include <are/platform/gl_context.h>
#include <are/core/logger.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <sstream>

namespace are {

bool GLContext::initialized_ = false;

bool GLContext::initialize() {
    if (initialized_) {
        ARE_LOG_WARN("OpenGL context already initialized");
        return true;
    }

    // Load OpenGL function pointers using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        ARE_LOG_CRITICAL("Failed to initialize GLAD");
        return false;
    }

    initialized_ = true;
    
    ARE_LOG_INFO("OpenGL context initialized successfully");
    print_info();
    
    return true;
}

bool GLContext::is_initialized() {
    return initialized_;
}

std::string GLContext::get_version() {
    if (!initialized_) {
        return "Not initialized";
    }
    
    const GLubyte* version = glGetString(GL_VERSION);
    return version ? std::string(reinterpret_cast<const char*>(version)) : "Unknown";
}

std::string GLContext::get_renderer() {
    if (!initialized_) {
        return "Not initialized";
    }
    
    const GLubyte* renderer = glGetString(GL_RENDERER);
    return renderer ? std::string(reinterpret_cast<const char*>(renderer)) : "Unknown";
}

std::string GLContext::get_vendor() {
    if (!initialized_) {
        return "Not initialized";
    }
    
    const GLubyte* vendor = glGetString(GL_VENDOR);
    return vendor ? std::string(reinterpret_cast<const char*>(vendor)) : "Unknown";
}

bool GLContext::is_extension_supported(const std::string& extension) {
    if (!initialized_) {
        return false;
    }

    GLint num_extensions = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &num_extensions);

    for (GLint i = 0; i < num_extensions; ++i) {
        const GLubyte* ext = glGetStringi(GL_EXTENSIONS, i);
        if (ext && extension == reinterpret_cast<const char*>(ext)) {
            return true;
        }
    }

    return false;
}

void GLContext::print_info() {
    if (!initialized_) {
        ARE_LOG_WARN("Cannot print OpenGL info: context not initialized");
        return;
    }

    ARE_LOG_INFO("=== OpenGL Information ===");
    ARE_LOG_INFO("Version: " + get_version());
    ARE_LOG_INFO("Renderer: " + get_renderer());
    ARE_LOG_INFO("Vendor: " + get_vendor());
    
    // Get GLSL version
    const GLubyte* glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION);
    if (glsl_version) {
        ARE_LOG_INFO("GLSL Version: " + std::string(reinterpret_cast<const char*>(glsl_version)));
    }

    // Get max texture size
    GLint max_texture_size = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
    ARE_LOG_INFO("Max Texture Size: " + std::to_string(max_texture_size));

    // Get max compute work group size
    GLint max_compute_work_group_invocations = 0;
    glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &max_compute_work_group_invocations);
    ARE_LOG_INFO("Max Compute Work Group Invocations: " + 
                 std::to_string(max_compute_work_group_invocations));

    ARE_LOG_INFO("==========================");
}

bool GLContext::check_error(const char* file, int line) {
    GLenum error = glGetError();
    
    if (error == GL_NO_ERROR) {
        return false;
    }

    std::string error_string;
    switch (error) {
        case GL_INVALID_ENUM:
            error_string = "GL_INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            error_string = "GL_INVALID_VALUE";
            break;
        case GL_INVALID_OPERATION:
            error_string = "GL_INVALID_OPERATION";
            break;
        case GL_OUT_OF_MEMORY:
            error_string = "GL_OUT_OF_MEMORY";
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            error_string = "GL_INVALID_FRAMEBUFFER_OPERATION";
            break;
        default:
            error_string = "Unknown error " + std::to_string(error);
            break;
    }

    // Extract filename from path
    const char* filename = file;
    for (const char* p = file; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            filename = p + 1;
        }
    }

    ARE_LOG_ERROR("OpenGL Error: " + error_string + " at " + 
                  filename + ":" + std::to_string(line));
    
    return true;
}

void GLContext::clear_errors() {
    while (glGetError() != GL_NO_ERROR) {
        // Clear all errors
    }
}

} // namespace are
