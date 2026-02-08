/**
 * @file gl_context.h
 * @brief OpenGL context management
 */

#ifndef ARE_INCLUDE_PLATFORM_GL_CONTEXT_H
#define ARE_INCLUDE_PLATFORM_GL_CONTEXT_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @class GLContext
 * @brief OpenGL context initialization and management
 * 
 * Handles GLAD initialization and provides OpenGL utility functions.
 */
class GLContext {
public:
    /**
     * @brief Initialize OpenGL context (load function pointers)
     * @return true if initialization succeeded
     */
    static bool initialize();

    /**
     * @brief Check if context is initialized
     * @return true if initialized
     */
    static bool is_initialized();

    /**
     * @brief Get OpenGL version string
     * @return Version string
     */
    static std::string get_version();

    /**
     * @brief Get OpenGL renderer string
     * @return Renderer string
     */
    static std::string get_renderer();

    /**
     * @brief Get OpenGL vendor string
     * @return Vendor string
     */
    static std::string get_vendor();

    /**
     * @brief Check if OpenGL extension is supported
     * @param extension Extension name
     * @return true if supported
     */
    static bool is_extension_supported(const std::string& extension);

    /**
     * @brief Print OpenGL information to console
     */
    static void print_info();

    /**
     * @brief Check for OpenGL errors
     * @param file Source file
     * @param line Line number
     * @return true if error occurred
     */
    static bool check_error(const char* file, int line);

    /**
     * @brief Clear all OpenGL errors
     */
    static void clear_errors();

private:
    static bool initialized_;             ///< Initialization flag
};

} // namespace are

// OpenGL error checking macro
#ifdef ARE_ENABLE_DEBUG_VIS
    #define ARE_GL_CHECK() are::GLContext::check_error(__FILE__, __LINE__)
#else
    #define ARE_GL_CHECK() ((void)0)
#endif

#endif // ARE_INCLUDE_PLATFORM_GL_CONTEXT_H
