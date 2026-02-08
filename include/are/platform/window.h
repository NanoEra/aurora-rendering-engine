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
