### 文件：include/are/platform/window.h
```cpp
/**
 * @file window.h
 * @brief Window management using GLFW
 */

#ifndef ARE_INCLUDE_PLATFORM_WINDOW_H
#define ARE_INCLUDE_PLATFORM_WINDOW_H

#include <are/core/config.h>
#include <are/core/types.h>
#include <string>

// Forward declare GLFW types to avoid including GLFW in header
struct GLFWwindow;

namespace are {

/**
 * @class Window
 * @brief GLFW window wrapper
 * 
 * Manages window creation, input handling, and OpenGL context.
 */
class Window {
public:
    /**
     * @brief Constructor
     * @param config Window configuration
     */
    explicit Window(const WindowConfig& config);

    /**
     * @brief Destructor
     */
    ~Window();

    // Window control
    bool should_close() const;
    void set_should_close(bool should_close);
    void swap_buffers();
    void poll_events();

    // Window properties
    int get_width() const;
    int get_height() const;
    Real get_aspect_ratio() const;
    const std::string& get_title() const;

    void set_title(const std::string& title);
    void set_size(int width, int height);

    // Framebuffer size (may differ from window size on high-DPI displays)
    void get_framebuffer_size(int& width, int& height) const;

    // VSync control
    void set_vsync(bool enabled);
    bool get_vsync() const;

    // Input queries (basic support)
    bool is_key_pressed(int key) const;
    bool is_mouse_button_pressed(int button) const;
    void get_cursor_pos(double& x, double& y) const;

    // Internal
    GLFWwindow* get_native_window() const { return window_; }

private:
    void initialize_glfw();
    void create_window();
    void setup_callbacks();

    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void error_callback(int error, const char* description);

    GLFWwindow* window_;                  ///< GLFW window handle
    WindowConfig config_;                 ///< Window configuration
    bool vsync_enabled_;                  ///< VSync state

    static int instance_count_;           ///< Number of Window instances
};

} // namespace are

#endif // ARE_INCLUDE_PLATFORM_WINDOW_H
```

### 文件：include/are/platform/gl_context.h
```cpp
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
```

### 文件：include/are/utils/file_utils.h
```cpp
/**
 * @file file_utils.h
 * @brief File system utilities
 */

#ifndef ARE_INCLUDE_UTILS_FILE_UTILS_H
#define ARE_INCLUDE_UTILS_FILE_UTILS_H

#include <string>
#include <vector>
#include <cstdint>

namespace are {

/**
 * @brief Read entire file into string
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::string read_file_to_string(const std::string& filepath);

/**
 * @brief Read entire file into byte array
 * @param filepath File path
 * @return File contents (empty if failed)
 */
std::vector<uint8_t> read_file_to_bytes(const std::string& filepath);

/**
 * @brief Write string to file
 * @param filepath File path
 * @param content Content to write
 * @return true if write succeeded
 */
bool write_string_to_file(const std::string& filepath, const std::string& content);

/**
 * @brief Write bytes to file
 * @param filepath File path
 * @param data Data pointer
 * @param size Data size in bytes
 * @return true if write succeeded
 */
bool write_bytes_to_file(const std::string& filepath, const void* data, size_t size);

/**
 * @brief Check if file exists
 * @param filepath File path
 * @return true if file exists
 */
bool file_exists(const std::string& filepath);

/**
 * @brief Check if path is directory
 * @param path Directory path
 * @return true if directory exists
 */
bool is_directory(const std::string& path);

/**
 * @brief Create directory (including parent directories)
 * @param path Directory path
 * @return true if creation succeeded
 */
bool create_directory(const std::string& path);

/**
 * @brief Get file extension
 * @param filepath File path
 * @return Extension (lowercase, without dot)
 */
std::string get_file_extension(const std::string& filepath);

/**
 * @brief Get filename from path
 * @param filepath File path
 * @return Filename (without directory)
 */
std::string get_filename(const std::string& filepath);

/**
 * @brief Get directory from path
 * @param filepath File path
 * @return Directory path
 */
std::string get_directory(const std::string& filepath);

/**
 * @brief Join path components
 * @param parts Path components
 * @return Joined path
 */
std::string join_path(const std::vector<std::string>& parts);

/**
 * @brief Normalize path (resolve .. and .)
 * @param path Path to normalize
 * @return Normalized path
 */
std::string normalize_path(const std::string& path);

} // namespace are

#endif // ARE_INCLUDE_UTILS_FILE_UTILS_H
```

这是你所要求的三个头文件，如果还需要更多头文件的代码，请随时向我提出。同时，平台层着三个函数的代码我也已经实现，你只需要专心考虑渲染管线即可。此外，渲染管线的头文件我也实现了，你只需要负责实现渲染管线的shader以及代码实现即可。
### 文件：include/are/rasterizer/shader_program.h
```cpp
/**
 * @file shader_program.h
 * @brief OpenGL shader program wrapper
 */

#ifndef ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
#define ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H

#include <are/core/types.h>
#include <string>
#include <unordered_map>

namespace are {

/**
 * @enum ShaderType
 * @brief Shader stage types
 */
enum class ShaderType {
    ARE_SHADER_VERTEX,
    ARE_SHADER_FRAGMENT,
    ARE_SHADER_COMPUTE
};

/**
 * @class ShaderProgram
 * @brief OpenGL shader program management
 */
class ShaderProgram {
public:
    /**
     * @brief Constructor
     */
    ShaderProgram();

    /**
     * @brief Destructor
     */
    ~ShaderProgram();

    /**
     * @brief Load and compile shader from file
     * @param type Shader type
     * @param filepath Shader file path
     * @return true if compilation succeeded
     */
    bool load_shader(ShaderType type, const std::string& filepath);

    /**
     * @brief Compile shader from source string
     * @param type Shader type
     * @param source Shader source code
     * @return true if compilation succeeded
     */
    bool compile_shader(ShaderType type, const std::string& source);

    /**
     * @brief Link shader program
     * @return true if linking succeeded
     */
    bool link();

    /**
     * @brief Use this shader program
     */
    void use() const;

    /**
     * @brief Check if program is valid
     * @return true if valid
     */
    bool is_valid() const { return program_ != 0 && linked_; }

    /**
     * @brief Get OpenGL program ID
     * @return Program ID
     */
    uint32_t get_program() const { return program_; }

    // Uniform setters
    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, const Vec2& value);
    void set_uniform(const std::string& name, const Vec3& value);
    void set_uniform(const std::string& name, const Vec4& value);
    void set_uniform(const std::string& name, const Mat3& value);
    void set_uniform(const std::string& name, const Mat4& value);

    /**
     * @brief Get uniform location (cached)
     * @param name Uniform name
     * @return Uniform location (-1 if not found)
     */
    int get_uniform_location(const std::string& name);

private:
    bool check_compile_errors(uint32_t shader, ShaderType type);
    bool check_link_errors();

    uint32_t program_;                    ///< OpenGL program ID
    uint32_t vertex_shader_;              ///< Vertex shader ID
    uint32_t fragment_shader_;            ///< Fragment shader ID
    uint32_t compute_shader_;             ///< Compute shader ID
    
    bool linked_;                         ///< Link status
    std::unordered_map<std::string, int> uniform_cache_; ///< Uniform location cache
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
```

### 文件：include/are/rasterizer/gbuffer.h
```cpp
/**
 * @file gbuffer.h
 * @brief G-Buffer management for deferred rendering
 */

#ifndef ARE_INCLUDE_RASTERIZER_GBUFFER_H
#define ARE_INCLUDE_RASTERIZER_GBUFFER_H

#include <are/core/types.h>
#include <cstdint>

namespace are {

/**
 * @class GBuffer
 * @brief G-Buffer for deferred rendering
 * 
 * Contains multiple render targets for position, normal, albedo, etc.
 */
class GBuffer {
public:
    /**
     * @brief Constructor
     * @param width Buffer width
     * @param height Buffer height
     */
    GBuffer(int width, int height);

    /**
     * @brief Destructor
     */
    ~GBuffer();

    /**
     * @brief Resize G-Buffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Bind G-Buffer for rendering
     */
    void bind();

    /**
     * @brief Unbind G-Buffer
     */
    void unbind();

    /**
     * @brief Clear all buffers
     */
    void clear();

    /**
     * @brief Bind texture for reading
     * @param index Texture index (0=position, 1=normal, 2=albedo, etc.)
     * @param texture_unit Texture unit to bind to
     */
    void bind_texture(int index, int texture_unit);

    // Texture getters
    uint32_t get_position_texture() const { return position_texture_; }
    uint32_t get_normal_texture() const { return normal_texture_; }
    uint32_t get_albedo_texture() const { return albedo_texture_; }
    uint32_t get_material_texture() const { return material_texture_; }
    uint32_t get_depth_texture() const { return depth_texture_; }

    // Dimensions
    int get_width() const { return width_; }
    int get_height() const { return height_; }

    /**
     * @brief Read pixel data from G-Buffer
     * @param index Buffer index
     * @param data Output data pointer
     */
    void read_pixels(int index, void* data);

private:
    void create_textures();
    void delete_textures();
    void create_framebuffer();

    uint32_t fbo_;                        ///< Framebuffer object
    uint32_t rbo_depth_;                  ///< Depth renderbuffer

    // G-Buffer textures
    uint32_t position_texture_;           ///< World position (RGB16F)
    uint32_t normal_texture_;             ///< World normal (RGB16F)
    uint32_t albedo_texture_;             ///< Albedo + Metallic (RGBA8)
    uint32_t material_texture_;           ///< Roughness + AO (RG8)
    uint32_t depth_texture_;              ///< Depth (R32F)

    int width_;                           ///< Buffer width
    int height_;                          ///< Buffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_GBUFFER_H
```

### 文件：include/are/rasterizer/rasterizer.h
```cpp
/**
 * @file rasterizer.h
 * @brief Rasterization pipeline for G-Buffer generation
 */

#ifndef ARE_INCLUDE_RASTERIZER_RASTERIZER_H
#define ARE_INCLUDE_RASTERIZER_RASTERIZER_H

#include <are/core/types.h>
#include <are/core/config.h>
#include <memory>

namespace are {

// Forward declarations
class GBuffer;
class ShaderProgram;
class SceneManager;
class Camera;
class Mesh;

/**
 * @class Rasterizer
 * @brief OpenGL rasterization pipeline
 * 
 * Renders scene geometry to G-Buffer using traditional rasterization.
 */
class Rasterizer {
public:
    /**
     * @brief Constructor
     * @param width Framebuffer width
     * @param height Framebuffer height
     */
    Rasterizer(int width, int height);

    /**
     * @brief Destructor
     */
    ~Rasterizer();

    /**
     * @brief Resize framebuffer
     * @param width New width
     * @param height New height
     */
    void resize(int width, int height);

    /**
     * @brief Render scene to G-Buffer
     * @param scene Scene manager
     * @param camera Camera
     */
    void render_gbuffer(const SceneManager& scene, const Camera& camera);

    /**
     * @brief Get G-Buffer
     * @return G-Buffer reference
     */
    GBuffer& get_gbuffer();
    const GBuffer& get_gbuffer() const;

    /**
     * @brief Upload mesh data to GPU
     * @param mesh Mesh to upload
     */
    void upload_mesh(Mesh& mesh);

    /**
     * @brief Delete mesh GPU resources
     * @param mesh Mesh to delete
     */
    void delete_mesh(Mesh& mesh);

private:
    void initialize_shaders(const std::string& shader_dir);
    void setup_mesh_buffers(Mesh& mesh);

    std::unique_ptr<GBuffer> gbuffer_;    ///< G-Buffer
    std::unique_ptr<ShaderProgram> gbuffer_shader_; ///< G-Buffer shader
    
    int width_;                           ///< Framebuffer width
    int height_;                          ///< Framebuffer height
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_RASTERIZER_H
```

如果没有还需要我给出的内容的话，你就可以开始实现了。
