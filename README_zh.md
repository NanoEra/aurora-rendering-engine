<div align="center">
  <a href="https://github.com/NanoEraStudio/AuroraRenderingEngine">
    <img src="https://are.nanoera.top/res/logo/logo.png" alt="Aurora Logo" width="120" height="120">
  </a>
  
  <h1 align="center">曙光渲染引擎</h1>
  
  <p align="center">
    <strong>Aurora Rendering Engine (A.R.E.)</strong><br/>
    一个基于 C++ 的高性能路径追踪渲染库
    <br/>
    <br/>
    <a href="README_EN.md">English</a> | 中文
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
## 📖 简介
曙光渲染引擎（Aurora Rendering Engine，简称 A.R.E.）是一个基于 C++ 开发的路径追踪渲染库，由 **NanoEra Studio** 开发和维护。A.R.E. 利用 OpenGL 4.3 计算着色器实现高效的 GPU 路径追踪，提供了简洁易用的 API，适合学习和研究光线追踪技术。
## ✨ 特性
- 🚀 **GPU 加速路径追踪** - 基于 OpenGL 4.3 Compute Shader
- 🎨 **PBR 材质系统** - 支持漫反射、金属、电介质等材质
- 💡 **多种光源支持** - 点光源、面光源、环境光
- 📦 **静态链接库** - 轻松集成到现有项目
- 🔧 **CMake 构建** - 跨平台支持
## 🛠️ 依赖项
A.R.E. 依赖以下外部库：
- **OpenGL 4.3** - 图形 API
- **GLFW** - 窗口和输入管理
- **GLAD** - OpenGL 加载器
- **GLM** - 数学库
- **stb-image** - 图像加载
- **spdlog** - 日志系统
## 📦 快速开始
### 克隆仓库
```bash
git clone https://github.com/NanoEraStudio/AuroraRenderingEngine.git
cd AuroraRenderingEngine
```
### 编译项目
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
## 🎮 极简示例：康奈尔盒子
以下代码展示了如何使用 A.R.E. 渲染一个经典的康奈尔盒子场景：
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
## 📚 文档
详细文档请访问：[A.R.E. Documentation](https://are.nanoera.top/docs)
## 🤝 贡献指南
我们欢迎所有形式的贡献！请查看 [CONTRIBUTING.md](CONTRIBUTING.md) 了解详情。
## 📄 许可证
本项目基于 MIT 许可证开源，详见 [LICENSE](LICENSE) 文件。
## 🙏 致谢
感谢所有依赖库的开发者和贡献者。
---
<div align="center">
  <sub>Built with ❤️ by <a href="https://github.com/NanoEraStudio">NanoEra Studio</a></sub>
</div>

