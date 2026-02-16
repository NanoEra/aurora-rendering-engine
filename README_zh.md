<div align="center">
  <a href="https://github.com/NanoEra/aurora-rendering-engine">
    <img src="https://are.nanoera.top/res/logo/logo.png" alt="Aurora Logo" width="120" height="120">
  </a>
  
  <h1 align="center">曙光渲染引擎</h1>
  
  <p align="center">
    <strong>Aurora Rendering Engine (ARE)</strong> - 一个基于 C++ 的高性能路径追踪渲染库
    <br/>
    <br/>
    <a href="README.md">English</a> | 中文
  </p>
  
  <p align="center">
    <a href="https://github.com/NanoEra/aurora-rendering-engine/blob/main/LICENSE">
      <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="License">
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
## 📖 简介
曙光渲染引擎（Aurora Rendering Engine，简称 ARE）是一个基于 C++ 开发的路径追踪渲染库，由 **NanoEra Studio** 开发和维护。
## 依赖
- **OpenGL 4.3**
- **GLAD**
- **GLM**
- **stb-image**
- **spdlog**
## 快速开始
### 克隆仓库
```bash
git clone https://github.com/NanoEra/aurora-rendering-engine.git
cd aurora-rendering-engine
```
### 编译项目
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
## 示例
如下是一个Cornell Box Demo：
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
    // 1. 初始化窗口
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(800, 800, "Aurora - Cornell Box", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    
    // 2. 配置渲染器
    RendererConfig config;
    config.width_ = 800;
    config.height_ = 800;
    config.samples_per_pixel_ = 1;
    config.max_ray_depth_ = 4;
    
    auto renderer = std::make_unique<Renderer>(config);
    renderer->initialize();
    
    // 3. 创建场景
    auto scene = std::make_unique<Scene>();
    
    // 创建材质
    auto white_mat = std::make_shared<Material>();
    white_mat->set_albedo(Vec3(0.73f, 0.73f, 0.73f));
    white_mat->set_type(MaterialType::DIFFUSE);
    uint white_id = scene->add_material(white_mat);
    
    auto red_mat = std::make_shared<Material>();
    red_mat->set_albedo(Vec3(0.65f, 0.05f, 0.05f));
    red_mat->set_type(MaterialType::DIFFUSE);
    uint red_id = scene->add_material(red_mat);
    
    // 创建地板 (示例：一个简单的四边形)
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
    
    // 设置相机
    auto camera = std::make_shared<Camera>();
    camera->set_position(Vec3(0.0f, 0.0f, 4.5f));
    camera->set_target(Vec3(0.0f, 0.0f, 0.0f));
    camera->set_perspective(45.0f, 1.0f, 0.1f, 100.0f);
    scene->set_camera(camera);
    
    // 4. 渲染循环
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

