<div align="center">
  <a href="https://github.com/NanoEraStudio/AuroraRenderingEngine">
    <img src="https://are.nanoera.top/res/logo/logo.png" alt="Aurora Logo" width="120" height="120">
  </a>
  
  <h1 align="center">Aurora Rendering Engine</h1>
  
  <p align="center">
    <strong>A.R.E.</strong> - A High-Performance Path Tracing Library in C++
    <br/>
    <br/>
    English | <a href="README-zh.md">中文</a>
  </p>
  
  <p align="center">
    <a href="https://github.com/NanoEraStudio/AuroraRenderingEngine/blob/main/LICENSE">
      <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License">
    </a>
    <a href="https://github.com/NanoEraStudio/AuroraRenderingEngine/releases">
      <img src="https://img.shields.io/badge/version-1.0.0-green.svg" alt="Version">
    </a>
    <a href="https://github.com/NanoEraStudio/AuroraRenderingEngine/issues">
      <img src="https://img.shields.io/github/issues/NanoEraStudio/AuroraRenderingEngine.svg" alt="Issues">
    </a>
  </p>
</div>
---
## 📖 Overview
**Aurora Rendering Engine (A.R.E.)** is a high-performance path tracing library developed in C++ by **NanoEra Studio**. Leveraging OpenGL 4.3 compute shaders for GPU-accelerated path tracing, A.R.E. provides a clean and intuitive API suitable for learning and researching ray tracing techniques.
## ✨ Features
- 🚀 **GPU-Accelerated Path Tracing** - Powered by OpenGL 4.3 Compute Shaders
- 🎨 **PBR Material System** - Diffuse, Metal, Dielectric materials
- 💡 **Multiple Light Types** - Point lights, Area lights, Environmental lighting
- 📦 **Static Library** - Easy integration into existing projects
- 🔧 **CMake Build System** - Cross-platform support
## 🛠️ Dependencies
A.R.E. depends on the following external libraries:
- **OpenGL 4.3** - Graphics API
- **GLFW** - Window and input management
- **GLAD** - OpenGL loader
- **GLM** - Mathematics library
- **stb-image** - Image loading
- **spdlog** - Logging system
## 📦 Quick Start
### Clone Repository
```bash
git clone https://github.com/NanoEraStudio/AuroraRenderingEngine.git
cd AuroraRenderingEngine
```
### Build Project
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
## 🎮 Minimal Example: Cornell Box
The following code demonstrates how to render a classic Cornell Box scene using A.R.E.:
```cpp
#include <core/renderer.h>
#include <scene/scene.h>
#include <scene/camera.h>
#include <scene/mesh.h>
#include <scene/material.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
using namespace are;
int main() {
    // 1. Initialize window
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(800, 800, "Aurora - Cornell Box", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    
    // 2. Configure renderer
    RendererConfig config;
    config.width_ = 800;
    config.height_ = 800;
    config.samples_per_pixel_ = 1;
    config.max_ray_depth_ = 4;
    
    auto renderer = std::make_unique<Renderer>(config);
    renderer->initialize();
    
    // 3. Create scene
    auto scene = std::make_unique<Scene>();
    
    // Create materials
    auto white_mat = std::make_shared<Material>();
    white_mat->set_albedo(Vec3(0.73f, 0.73f, 0.73f));
    white_mat->set_type(MaterialType::DIFFUSE);
    uint white_id = scene->add_material(white_mat);
    
    auto red_mat = std::make_shared<Material>();
    red_mat->set_albedo(Vec3(0.65f, 0.05f, 0.05f));
    red_mat->set_type(MaterialType::DIFFUSE);
    uint red_id = scene->add_material(red_mat);
    
    // Create floor mesh (example: a simple quad)
    auto floor = std::make_shared<Mesh>();
    std::vector<Vertex> vertices = {
        {{-2.0f, -2.0f, -2.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{ 2.0f, -2.0f, -2.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
        {{ 2.0f, -2.0f,  2.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
        {{-2.0f, -2.0f,  2.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}}
    };
    std::vector<uint> indices = {0, 1, 2, 0, 2, 3};
    floor->set_vertices(vertices);
    floor->set_indices(indices);
    floor->set_material(white_id);
    floor->upload_to_gpu();
    scene->add_mesh(floor);
    
    // Setup camera
    auto camera = std::make_shared<Camera>();
    camera->set_position(Vec3(0.0f, 0.0f, 4.5f));
    camera->set_target(Vec3(0.0f, 0.0f, 0.0f));
    camera->set_perspective(45.0f, 1.0f, 0.1f, 100.0f);
    scene->set_camera(camera);
    
    // 4. Render loop
    while (!glfwWindowShouldClose(window)) {
        renderer->render(*scene);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    return 0;
}
```
## 📚 Documentation
For detailed documentation, visit: [A.R.E. Documentation](https://are.nanoera.top/docs)
## 🤝 Contributing
We welcome all forms of contributions! Please check [CONTRIBUTING.md](CONTRIBUTING.md) for details.
## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
## 🙏 Acknowledgments
Thanks to all the developers and contributors of the dependency libraries.
---
<div align="center">
  <sub>Built with ❤️ by <a href="https://github.com/NanoEraStudio">NanoEra Studio</a></sub>
</div>
