#include <core/renderer.h>
#include <scene/scene.h>
#include <scene/camera.h>
#include <scene/mesh.h>
#include <scene/material.h>
#include <scene/light.h>
#include <utils/logger.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

using namespace are;

// Window dimensions
const uint WINDOW_WIDTH = 800;
const uint WINDOW_HEIGHT = 800;

// Global state
GLFWwindow* g_window = nullptr;
std::unique_ptr<Renderer> g_renderer = nullptr;
std::unique_ptr<Scene> g_scene = nullptr;

// GLFW error callback
void glfw_error_callback(int error, const char* description) {
    Logger::error("GLFW Error " + std::to_string(error) + ": " + std::string(description));
}

/// @brief Create a quad mesh
std::shared_ptr<Mesh> create_quad(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Vec3& v3, 
                                  const Vec3& normal, uint material_id) {
    auto mesh = std::make_shared<Mesh>();
    
    std::vector<Vertex> vertices = {
        {v0, normal, Vec2(0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f)},
        {v1, normal, Vec2(1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f)},
        {v2, normal, Vec2(1.0f, 1.0f), Vec3(1.0f, 0.0f, 0.0f)},
        {v3, normal, Vec2(0.0f, 1.0f), Vec3(1.0f, 0.0f, 0.0f)}
    };
    
    std::vector<uint> indices = {0, 1, 2, 0, 2, 3};
    
    mesh->set_vertices(vertices);
    mesh->set_indices(indices);
    mesh->set_material(material_id);
    
    return mesh;
}

/// @brief Create a box mesh
std::shared_ptr<Mesh> create_box(const Vec3& min, const Vec3& max, uint material_id) {
    auto mesh = std::make_shared<Mesh>();
    
    std::vector<Vertex> vertices = {
        // Front face
        {{min.x, min.y, max.z}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, min.y, max.z}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, max.y, max.z}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        {{min.x, max.y, max.z}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        
        // Back face
        {{max.x, min.y, min.z}, {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}},
        {{min.x, min.y, min.z}, {0.0f, 0.0f, -1.0f}, {1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}},
        {{min.x, max.y, min.z}, {0.0f, 0.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}},
        {{max.x, max.y, min.z}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}},
        
        // Top face
        {{min.x, max.y, max.z}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, max.y, max.z}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, max.y, min.z}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        {{min.x, max.y, min.z}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        
        // Bottom face
        {{min.x, min.y, min.z}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, min.y, min.z}, {0.0f, -1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{max.x, min.y, max.z}, {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        {{min.x, min.y, max.z}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        
        // Right face
        {{max.x, min.y, max.z}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
        {{max.x, min.y, min.z}, {1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
        {{max.x, max.y, min.z}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 0.0f, -1.0f}},
        {{max.x, max.y, max.z}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}},
        
        // Left face
        {{min.x, min.y, min.z}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{min.x, min.y, max.z}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{min.x, max.y, max.z}, {-1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}},
        {{min.x, max.y, min.z}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}}
    };
    
    std::vector<uint> indices = {
        0, 1, 2, 0, 2, 3,       // Front
        4, 5, 6, 4, 6, 7,       // Back
        8, 9, 10, 8, 10, 11,    // Top
        12, 13, 14, 12, 14, 15, // Bottom
        16, 17, 18, 16, 18, 19, // Right
        20, 21, 22, 20, 22, 23  // Left
    };
    
    mesh->set_vertices(vertices);
    mesh->set_indices(indices);
    mesh->set_material(material_id);
    
    return mesh;
}

/// @brief Setup Cornell Box scene
void setup_cornell_box() {
    g_scene = std::make_unique<Scene>();
    
    // Create materials
    // 0: White diffuse
    auto white_material = std::make_shared<Material>();
    white_material->set_albedo(Vec3(0.73f, 0.73f, 0.73f));
    white_material->set_type(MaterialType::DIFFUSE);
    uint white_id = g_scene->add_material(white_material);
    
    // 1: Red diffuse (left wall)
    auto red_material = std::make_shared<Material>();
    red_material->set_albedo(Vec3(0.65f, 0.05f, 0.05f));
    red_material->set_type(MaterialType::DIFFUSE);
    uint red_id = g_scene->add_material(red_material);
    
    // 2: Green diffuse (right wall)
    auto green_material = std::make_shared<Material>();
    green_material->set_albedo(Vec3(0.12f, 0.45f, 0.15f));
    green_material->set_type(MaterialType::DIFFUSE);
    uint green_id = g_scene->add_material(green_material);
    
    // 3: Light emissive
    auto light_material = std::make_shared<Material>();
    light_material->set_albedo(Vec3(1.0f, 1.0f, 1.0f));
    light_material->set_emission(Vec3(15.0f, 15.0f, 15.0f));
    light_material->set_type(MaterialType::EMISSIVE);
    uint light_id = g_scene->add_material(light_material);
    
    // 4: Metal (for one box)
    auto metal_material = std::make_shared<Material>();
    metal_material->set_albedo(Vec3(0.95f, 0.93f, 0.88f));
    metal_material->set_metallic(1.0f);
    metal_material->set_roughness(0.1f);
    metal_material->set_type(MaterialType::METAL);
    uint metal_id = g_scene->add_material(metal_material);
    
    // Create room (Cornell Box)
    float room_size = 2.0f;
    
    // Floor (white)
    auto floor = create_quad(
        Vec3(-room_size, -room_size, -room_size),
        Vec3(room_size, -room_size, -room_size),
        Vec3(room_size, -room_size, room_size),
        Vec3(-room_size, -room_size, room_size),
        Vec3(0.0f, 1.0f, 0.0f),
        white_id
    );
    floor->upload_to_gpu();
    g_scene->add_mesh(floor);
    
    // Ceiling (white)
    auto ceiling = create_quad(
        Vec3(-room_size, room_size, room_size),
        Vec3(room_size, room_size, room_size),
        Vec3(room_size, room_size, -room_size),
        Vec3(-room_size, room_size, -room_size),
        Vec3(0.0f, -1.0f, 0.0f),
        white_id
    );
    ceiling->upload_to_gpu();
    g_scene->add_mesh(ceiling);
    
    // Back wall (white)
    auto back_wall = create_quad(
        Vec3(-room_size, -room_size, -room_size),
        Vec3(-room_size, room_size, -room_size),
        Vec3(room_size, room_size, -room_size),
        Vec3(room_size, -room_size, -room_size),
        Vec3(0.0f, 0.0f, 1.0f),
        white_id
    );
    back_wall->upload_to_gpu();
    g_scene->add_mesh(back_wall);
    
    // Left wall (red)
    auto left_wall = create_quad(
        Vec3(-room_size, -room_size, room_size),
        Vec3(-room_size, room_size, room_size),
        Vec3(-room_size, room_size, -room_size),
        Vec3(-room_size, -room_size, -room_size),
        Vec3(1.0f, 0.0f, 0.0f),
        red_id
    );
    left_wall->upload_to_gpu();
    g_scene->add_mesh(left_wall);
    
    // Right wall (green)
    auto right_wall = create_quad(
        Vec3(room_size, -room_size, -room_size),
        Vec3(room_size, room_size, -room_size),
        Vec3(room_size, room_size, room_size),
        Vec3(room_size, -room_size, room_size),
        Vec3(-1.0f, 0.0f, 0.0f),
        green_id
    );
    right_wall->upload_to_gpu();
    g_scene->add_mesh(right_wall);
    
    // Area light on ceiling
    float light_size = 0.5f;
    auto area_light = create_quad(
        Vec3(-light_size, room_size - 0.01f, -light_size),
        Vec3(light_size, room_size - 0.01f, -light_size),
        Vec3(light_size, room_size - 0.01f, light_size),
        Vec3(-light_size, room_size - 0.01f, light_size),
        Vec3(0.0f, -1.0f, 0.0f),
        light_id
    );
    area_light->upload_to_gpu();
    g_scene->add_mesh(area_light);
    
    // Tall box (white, left side)
    auto tall_box = create_box(Vec3(-0.7f, -room_size, -0.7f), Vec3(-0.2f, 0.6f, -0.2f), white_id);
    tall_box->upload_to_gpu();
    g_scene->add_mesh(tall_box);
    
    // Short box (metal, right side)
    auto short_box = create_box(Vec3(0.2f, -room_size, 0.2f), Vec3(0.9f, -0.4f, 0.9f), /*metal_id*/white_id);
    short_box->upload_to_gpu();
    g_scene->add_mesh(short_box);
    
    // Setup camera
    auto camera = std::make_shared<Camera>();
    camera->set_position(Vec3(0.0f, 0.0f, 4.5f));
    camera->set_target(Vec3(0.0f, 0.0f, 0.0f));
    camera->set_up(Vec3(0.0f, 1.0f, 0.0f));
    camera->set_perspective(45.0f, static_cast<float>(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.0f);
    g_scene->set_camera(camera);
    
    // Add point light
    auto light = std::make_shared<Light>();
    light->set_type(LightType::POINT);
    light->set_position(Vec3(0.0f, 1.8f, 0.0f));
    light->set_color(Vec3(1.0f, 1.0f, 1.0f));
    light->set_intensity(10.0f);
    light->set_range(10.0f);
    g_scene->add_light(light);
    
    Logger::info("Cornell Box scene created");
}

/// @brief Initialize GLFW and create window
bool init_window() {
    // Set error callback before init
    glfwSetErrorCallback(glfw_error_callback);
    
    if (!glfwInit()) {
        Logger::error("Failed to initialize GLFW");
        return false;
    }
    
    Logger::info("GLFW initialized successfully");
    
    // Request OpenGL 4.5 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
    // Additional hints for better compatibility
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 0);
    
    Logger::info("Creating window...");
    g_window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Aurora - Cornell Box", nullptr, nullptr);
    
    if (!g_window) {
        Logger::error("Failed to create GLFW window");
        Logger::error("Possible reasons:");
        Logger::error("  1. OpenGL 4.5 not supported by your GPU/driver");
        Logger::error("  2. No display server running (X11/Wayland)");
        Logger::error("  3. Insufficient GPU resources");
        
        // Try to get more info
        int major, minor, rev;
        glfwGetVersion(&major, &minor, &rev);
        Logger::info("GLFW version: " + std::to_string(major) + "." + 
                    std::to_string(minor) + "." + std::to_string(rev));
        
        glfwTerminate();
        return false;
    }
    
    Logger::info("Window created successfully");
    
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1); // Enable vsync
    
    // Load OpenGL functions
    Logger::info("Loading OpenGL functions...");
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        Logger::error("Failed to initialize GLAD");
        return false;
    }
    
    // Print OpenGL info
    const char* vendor = (const char*)glGetString(GL_VENDOR);
    const char* renderer = (const char*)glGetString(GL_RENDERER);
    const char* version = (const char*)glGetString(GL_VERSION);
    const char* glsl_version = (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION);
    
    Logger::info("OpenGL Vendor: " + std::string(vendor ? vendor : "Unknown"));
    Logger::info("OpenGL Renderer: " + std::string(renderer ? renderer : "Unknown"));
    Logger::info("OpenGL Version: " + std::string(version ? version : "Unknown"));
    Logger::info("GLSL Version: " + std::string(glsl_version ? glsl_version : "Unknown"));
    
    // Check OpenGL version
    GLint major_ver, minor_ver;
    glGetIntegerv(GL_MAJOR_VERSION, &major_ver);
    glGetIntegerv(GL_MINOR_VERSION, &minor_ver);
    
    Logger::info("OpenGL Context: " + std::to_string(major_ver) + "." + std::to_string(minor_ver));
    
    // if (major_ver < 4 || (major_ver == 4 && minor_ver < 5)) {
    //     Logger::error("OpenGL 4.5 or higher is required!");
    //     Logger::error("Your system supports: OpenGL " + std::to_string(major_ver) + "." + std::to_string(minor_ver));
    //     return false;
    // }
    
    // Check compute shader support
    GLint max_compute_work_group_invocations;
    glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &max_compute_work_group_invocations);
    Logger::info("Max compute work group invocations: " + std::to_string(max_compute_work_group_invocations));
    
    return true;
}

/// @brief Main render loop
void render_loop() {
    Logger::info("Entering render loop...");
    
    int frame_count = 0;
    double last_time = glfwGetTime();
    double fps_time = last_time;
    
    while (!glfwWindowShouldClose(g_window)) {
        // Render
        RenderStats stats = g_renderer->render(*g_scene);
        
        // Swap buffers
        glfwSwapBuffers(g_window);
        glfwPollEvents();
        
        // Calculate FPS
        frame_count++;
        double current_time = glfwGetTime();
        double delta = current_time - fps_time;
        
        if (delta >= 1.0) {
            double fps = frame_count / delta;
            std::string title = "Aurora - Cornell Box | FPS: " + std::to_string((int)fps) + 
                              " | Frame: " + std::to_string((int)stats.frame_time_ms_) + "ms";
            glfwSetWindowTitle(g_window, title.c_str());
            
            frame_count = 0;
            fps_time = current_time;
        }
        
        // Print detailed stats every 60 frames
        static int stat_frame_count = 0;
        if (++stat_frame_count % 60 == 0) {
            Logger::info("Frame time: " + std::to_string(stats.frame_time_ms_) + " ms (" +
                        std::to_string(1000.0f / stats.frame_time_ms_) + " FPS)");
            Logger::info("  G-Buffer: " + std::to_string(stats.gbuffer_time_ms_) + " ms");
            Logger::info("  Ray trace: " + std::to_string(stats.raytrace_time_ms_) + " ms");
            Logger::info("  Triangles: " + std::to_string(stats.triangle_count_));
        }
        
        // ESC to exit
        if (glfwGetKey(g_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(g_window, true);
        }
    }
    
    Logger::info("Exiting render loop");
}

/// @brief Cleanup
void cleanup() {
    Logger::info("Cleaning up...");
    
    if (g_renderer) {
        g_renderer->shutdown();
        g_renderer.reset();
    }
    
    g_scene.reset();
    
    if (g_window) {
        glfwDestroyWindow(g_window);
        glfwTerminate();
    }
    
    Logger::info("Cleanup complete");
}

int main() {
    // Initialize logger
    Logger::initialize();
    Logger::info("===========================================");
    Logger::info("Aurora Rendering Engine - Cornell Box Demo");
    Logger::info("===========================================");
    
    // Check environment
    const char* display = getenv("DISPLAY");
    const char* wayland_display = getenv("WAYLAND_DISPLAY");
    
    if (!display && !wayland_display) {
        Logger::error("No display server detected!");
        Logger::error("Make sure you're running in a graphical environment (X11 or Wayland)");
        Logger::error("DISPLAY=" + std::string(display ? display : "not set"));
        Logger::error("WAYLAND_DISPLAY=" + std::string(wayland_display ? wayland_display : "not set"));
        return -1;
    }
    
    Logger::info("Display server: " + std::string(display ? display : wayland_display));
    
    // Initialize window
    if (!init_window()) {
        cleanup();
        Logger::error("Failed to initialize window");
        Logger::shutdown();
        return -1;
    }
    
    // Setup scene
    Logger::info("Setting up Cornell Box scene...");
    setup_cornell_box();
    
    // Initialize renderer
    Logger::info("Initializing renderer...");
    RendererConfig config;
    config.width_ = WINDOW_WIDTH;
    config.height_ = WINDOW_HEIGHT;
    config.samples_per_pixel_ = 1;
    config.max_ray_depth_ = 4;
    config.enable_accumulation_ = true;
    config.enable_denoising_ = true;
    
    g_renderer = std::make_unique<Renderer>(config);
    if (!g_renderer->initialize()) {
        Logger::error("Failed to initialize renderer");
        cleanup();
        Logger::shutdown();
        return -1;
    }
    
    Logger::info("===========================================");
    Logger::info("Renderer initialized successfully!");
    Logger::info("Press ESC to exit");
    Logger::info("===========================================");
    
    // Main loop
    render_loop();
    
    // Cleanup
    cleanup();
    
    Logger::info("Cornell Box demo finished");
    Logger::shutdown();
    
    return 0;
}
