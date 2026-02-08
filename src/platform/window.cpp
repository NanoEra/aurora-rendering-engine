/**
 * @file window.cpp
 * @brief Implementation of GLFW window wrapper
 */

#include <are/platform/window.h>
#include <are/core/logger.h>
#include <GLFW/glfw3.h>
#include <stdexcept>

namespace are {

int Window::instance_count_ = 0;

Window::Window(const WindowConfig& config) 
    : window_(nullptr)
    , config_(config)
    , vsync_enabled_(config.vsync) {
    
    initialize_glfw();
    create_window();
    setup_callbacks();
}

Window::~Window() {
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    instance_count_--;
    if (instance_count_ == 0) {
        glfwTerminate();
        ARE_LOG_INFO("GLFW terminated");
    }
}

void Window::initialize_glfw() {
    if (instance_count_ == 0) {
        glfwSetErrorCallback(error_callback);
        
        if (!glfwInit()) {
            ARE_LOG_CRITICAL("Failed to initialize GLFW");
            throw std::runtime_error("GLFW initialization failed");
        }
        
        ARE_LOG_INFO("GLFW initialized successfully");
    }
    
    instance_count_++;
}

void Window::create_window() {
    // Set OpenGL version hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Set window hints
    glfwWindowHint(GLFW_RESIZABLE, config_.resizable ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_SAMPLES, config_.samples);

    // Create window
    window_ = glfwCreateWindow(
        config_.width,
        config_.height,
        config_.title.c_str(),
        nullptr,
        nullptr
    );

    if (!window_) {
        ARE_LOG_CRITICAL("Failed to create GLFW window");
        throw std::runtime_error("Window creation failed");
    }

    // Make context current
    glfwMakeContextCurrent(window_);

    // Set VSync
    glfwSwapInterval(vsync_enabled_ ? 1 : 0);

    ARE_LOG_INFO("Window created: " + std::to_string(config_.width) + "x" + 
                 std::to_string(config_.height) + " - " + config_.title);
}

void Window::setup_callbacks() {
    // Store this pointer in window user pointer
    glfwSetWindowUserPointer(window_, this);

    // Set framebuffer size callback
    glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
}

void Window::framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    Window* win = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (win) {
        win->config_.width = width;
        win->config_.height = height;
        ARE_LOG_DEBUG("Framebuffer resized: " + std::to_string(width) + "x" + std::to_string(height));
    }
}

void Window::error_callback(int error, const char* description) {
    ARE_LOG_ERROR("GLFW Error " + std::to_string(error) + ": " + description);
}

bool Window::should_close() const {
    return glfwWindowShouldClose(window_);
}

void Window::set_should_close(bool should_close) {
    glfwSetWindowShouldClose(window_, should_close ? GLFW_TRUE : GLFW_FALSE);
}

void Window::swap_buffers() {
    glfwSwapBuffers(window_);
}

void Window::poll_events() {
    glfwPollEvents();
}

int Window::get_width() const {
    return config_.width;
}

int Window::get_height() const {
    return config_.height;
}

Real Window::get_aspect_ratio() const {
    return static_cast<Real>(config_.width) / static_cast<Real>(config_.height);
}

const std::string& Window::get_title() const {
    return config_.title;
}

void Window::set_title(const std::string& title) {
    config_.title = title;
    glfwSetWindowTitle(window_, title.c_str());
}

void Window::set_size(int width, int height) {
    config_.width = width;
    config_.height = height;
    glfwSetWindowSize(window_, width, height);
}

void Window::get_framebuffer_size(int& width, int& height) const {
    glfwGetFramebufferSize(window_, &width, &height);
}

void Window::set_vsync(bool enabled) {
    vsync_enabled_ = enabled;
    glfwSwapInterval(enabled ? 1 : 0);
}

bool Window::get_vsync() const {
    return vsync_enabled_;
}

bool Window::is_key_pressed(int key) const {
    return glfwGetKey(window_, key) == GLFW_PRESS;
}

bool Window::is_mouse_button_pressed(int button) const {
    return glfwGetMouseButton(window_, button) == GLFW_PRESS;
}

void Window::get_cursor_pos(double& x, double& y) const {
    glfwGetCursorPos(window_, &x, &y);
}

} // namespace are
