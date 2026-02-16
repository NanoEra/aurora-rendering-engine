<div align="center">
  <a href="https://github.com/NanoEra/aurora-rendering-engine">
    <img src="https://are.nanoera.top/res/logo/logo.png" alt="Aurora Logo" width="120" height="120">
  </a>
  
  <h1 align="center">Aurora Rendering Engine</h1>
  
  <p align="center">
    <strong>ARE</strong> - A High-Performance Path Tracing Library in C++
    <br/>
    <br/>
    English | <a href="README_zh.md">中文</a>
  </p>
  
  <p align="center">
    <a href="https://github.com/NanoEra/aurora-rendering-engine/blob/main/LICENSE">
      <img src="https://img.shields.io/badge/license-LGPL-blue.svg" alt="License">
    </a>
    <a href="https://github.com/NanoEra/aurora-rendering-engine/releases">
      <img src="https://img.shields.io/badge/version-1.0.0-green.svg" alt="Version">
    </a>
    <a href="https://github.com/NanoEra/aurora-rendering-engine/issues">
      <img src="https://img.shields.io/github/issues/NanoEra/aurora-rendering-engine.svg" alt="Issues">
    </a>
  </p>
</div>

---
## Overview
**Aurora Rendering Engine (ARE)** is a high-performance path tracing library developed in C++ by **NanoEra Studio**.
## Dependencies
- **OpenGL 4.3**
- **GLFW**
- **GLAD**
- **GLM**
- **stb-image**
- **spdlog**
## Quick Start
### Clone Repository
```bash
git clone https://github.com/NanoEra/aurora-rendering-engine.git
cd aurora-rendering-engine
```
### Build Project
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
## Example
Here is a cornell box demo using ARE:
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
---

<div align="center">
  <sub>Built by <a href="https://www.nanoera.top/">NanoEra Studio</a></sub>
</div>
