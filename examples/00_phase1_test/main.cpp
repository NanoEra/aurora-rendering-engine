/**
 * @file main.cpp
 * @brief Phase 1 verification program
 * 
 * Tests core, platform, and utils modules.
 */

#include <are/are.h>
#include <are/utils/random.h>
#include <are/utils/file_utils.h>
#include <iostream>
#include <GLFW/glfw3.h>

void test_core_modules() {
    std::cout << "\n=== Testing Core Modules ===" << std::endl;
    
    // Test config
    are::AreConfig config;
    config.window.width = 800;
    config.window.height = 600;
    config.window.title = "Phase 1 Test";
    
    if (config.validate()) {
        ARE_LOG_INFO("Config validation passed");
        config.print();
    } else {
        ARE_LOG_ERROR("Config validation failed");
    }
    
    // Test profiler
    ARE_PROFILE_BEGIN("test_section");
    
    // Simulate some work
    double sum = 0.0;
    for (int i = 0; i < 1000000; ++i) {
        sum += i * 0.001;
    }
    
    ARE_PROFILE_END("test_section");
    
    ARE_LOG_INFO("Profiler test completed (sum = " + std::to_string(sum) + ")");
}

void test_platform_modules() {
    std::cout << "\n=== Testing Platform Modules ===" << std::endl;
    
    try {
        // Create window
        are::WindowConfig win_config;
        win_config.width = 800;
        win_config.height = 600;
        win_config.title = "Phase 1 Platform Test";
        
        are::Window window(win_config);
        ARE_LOG_INFO("Window created successfully");
        
        // Initialize OpenGL context
        if (are::GLContext::initialize()) {
            ARE_LOG_INFO("OpenGL context initialized");
            are::GLContext::print_info();
        } else {
            ARE_LOG_ERROR("Failed to initialize OpenGL context");
            return;
        }
        
        // Clear screen to blue
        glClearColor(0.2f, 0.3f, 0.8f, 1.0f);
        
        // Simple render loop (5 seconds)
        ARE_LOG_INFO("Running render loop for 5 seconds...");
        
        double start_time = glfwGetTime();
        int frame_count = 0;
        
        while (!window.should_close() && (glfwGetTime() - start_time) < 5.0) {
            glClear(GL_COLOR_BUFFER_BIT);
            
            window.swap_buffers();
            window.poll_events();
            
            frame_count++;
        }
        
        double elapsed = glfwGetTime() - start_time;
        double fps = frame_count / elapsed;
        
        ARE_LOG_INFO("Rendered " + std::to_string(frame_count) + " frames in " + 
                     std::to_string(elapsed) + " seconds");
        ARE_LOG_INFO("Average FPS: " + std::to_string(fps));
        
    } catch (const std::exception& e) {
        ARE_LOG_ERROR("Platform test failed: " + std::string(e.what()));
    }
}

void test_utils_modules() {
    std::cout << "\n=== Testing Utils Modules ===" << std::endl;
    
    // Test math utils
    are::Vec3 a(1, 0, 0);
    are::Vec3 b(0, 1, 0);
    are::Vec3 c(0, 0, 1);
    are::Vec3 p(0.3f, 0.3f, 0.4f);
    
    are::Real u, v, w;
    are::compute_barycentric(p, a, b, c, u, v, w);
    
    ARE_LOG_INFO("Barycentric coordinates: u=" + std::to_string(u) + 
                 ", v=" + std::to_string(v) + ", w=" + std::to_string(w));
    
    // Test random
    are::RandomGenerator rng(12345);
    
    ARE_LOG_INFO("Random float [0,1): " + std::to_string(rng.random_float()));
    ARE_LOG_INFO("Random float [10,20): " + std::to_string(rng.random_float(10.0f, 20.0f)));
    ARE_LOG_INFO("Random int [1,100]: " + std::to_string(rng.random_int(1, 100)));
    
    are::Vec3 random_vec = rng.random_unit_vector();
    ARE_LOG_INFO("Random unit vector: (" + std::to_string(random_vec.x) + ", " + 
                 std::to_string(random_vec.y) + ", " + std::to_string(random_vec.z) + ")");
    
    // Test file utils
    std::string test_dir = "test_output";
    if (are::create_directory(test_dir)) {
        ARE_LOG_INFO("Created directory: " + test_dir);
        
        std::string test_file = test_dir + "/test.txt";
        std::string content = "Hello, Aurora Rendering Engine!";
        
        if (are::write_string_to_file(test_file, content)) {
            ARE_LOG_INFO("Wrote test file: " + test_file);
            
            std::string read_content = are::read_file_to_string(test_file);
            if (read_content == content) {
                ARE_LOG_INFO("File read/write test passed");
            } else {
                ARE_LOG_ERROR("File content mismatch");
            }
        }
    }
    
    // Test image I/O (create a simple test image)
    are::ImageData test_image;
    test_image.width_ = 256;
    test_image.height_ = 256;
    test_image.channels_ = 3;
    test_image.data_.resize(256 * 256 * 3);
    
    // Create gradient
    for (int y = 0; y < 256; ++y) {
        for (int x = 0; x < 256; ++x) {
            int idx = (y * 256 + x) * 3;
            test_image.data_[idx + 0] = static_cast<uint8_t>(x);
            test_image.data_[idx + 1] = static_cast<uint8_t>(y);
            test_image.data_[idx + 2] = static_cast<uint8_t>((x + y) / 2);
        }
    }
    
    std::string image_file = test_dir + "/gradient.png";
    if (are::save_image(image_file, test_image)) {
        ARE_LOG_INFO("Saved test image: " + image_file);
    }
}

int main() {
	// Initialize engine
    if (!are::initialize()) {
        std::cerr << "Failed to initialize Aurora Rendering Engine" << std::endl;
        return -1;
    }
    
    ARE_LOG_INFO("Starting Phase 1 verification tests...");
    
    // Run tests
    test_core_modules();
    test_utils_modules();
    test_platform_modules();
    
    // Print profiler results
    are::Profiler::print_results();
    
    ARE_LOG_INFO("All Phase 1 tests completed!");
    
    // Shutdown engine
    are::shutdown();
    
    return 0;
}
