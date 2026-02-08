我正在开发 Aurora Rendering Engine (ARE)，一个 C++17 跨平台混合渲染引擎。

**核心特性**：
- 混合渲染：光栅化（G-Buffer）+ 光线追踪（GI/AO/软阴影）
- 双后端：CPU 和 Compute Shader 光线追踪，可运行时切换
- BVH 加速结构
- PBR 材质系统
- 工业级可扩展性

**技术栈**：
- C++17, OpenGL 4.3+, GLFW, GLAD, GLM, spdlog, stb_image
- 平台：Windows + Linux
- 构建：CMake 3.15+

**编码规范（NanoEra）**：
- 命名：函数/变量 `snake_case`，类 `PascalCase`，类成员后缀 `_`（但用户直接使用的结构体成员不加下划线）
- 全局宏前缀 `are_`，头文件保护 `ARE_INCLUDE_[路径]_H`
- 注释：全英文，重要接口用 Doxygen
- 禁用虚函数（性能优先，但不影响性能时可用）

**重要设计决策**：
1. 配置系统：纯代码配置，不支持文件加载
2. 错误处理：简化设计，直接输出，无堆栈累积
3. Shader 管理：外部文件，用户在 config 中指定路径
4. 场景数据：渲染器拷贝数据（用户友好）
5. BVH 构建：延迟构建（在 begin_frame 时检测并构建）
6. 材质系统：Mesh 持有 Material ID，支持复用

**文件目录树**：
```
aurora-rendering-engine/
├── include/                          # 头文件目录
│   ├── are/                          # 主命名空间目录
│   │   ├── are.h                     # 总头文件（包含所有公共接口）
│   │   ├── core/                     # 核心模块
│   │   │   ├── config.h              # 配置系统
│   │   │   ├── logger.h              # 日志系统
│   │   │   ├── types.h               # 基础类型定义
│   │   │   └── profiler.h            # 性能分析器
│   │   ├── renderer/                 # 渲染器模块
│   │   │   ├── renderer.h            # 主渲染器接口
│   │   │   ├── render_context.h      # 渲染上下文
│   │   │   └── render_stats.h        # 渲染统计信息
│   │   ├── scene/                    # 场景管理模块
│   │   │   ├── camera.h              # 相机
│   │   │   ├── mesh.h                # 网格
│   │   │   ├── material.h            # 材质
│   │   │   ├── light.h               # 光源（基类）
│   │   │   ├── directional_light.h   # 平行光
│   │   │   ├── point_light.h         # 点光源
│   │   │   ├── spot_light.h          # 聚光灯
│   │   │   └── scene_manager.h       # 场景管理器
│   │   ├── geometry/                 # 几何处理模块
│   │   │   ├── vertex.h              # 顶点结构
│   │   │   ├── triangle.h            # 三角形
│   │   │   ├── aabb.h                # 轴对齐包围盒
│   │   │   └── transform.h           # 变换矩阵
│   │   ├── acceleration/             # 加速结构模块
│   │   │   ├── bvh.h                 # BVH 接口
│   │   │   ├── bvh_node.h            # BVH 节点
│   │   │   └── bvh_builder.h         # BVH 构建器
│   │   ├── rasterizer/               # 光栅化模块
│   │   │   ├── rasterizer.h          # 光栅化器接口
│   │   │   ├── gbuffer.h             # G-Buffer 管理
│   │   │   └── shader_program.h      # Shader 程序封装
│   │   ├── raytracer/                # 光线追踪模块
│   │   │   ├── raytracer.h           # 光线追踪器接口
│   │   │   ├── ray.h                 # 光线结构
│   │   │   ├── hit_record.h          # 碰撞记录
│   │   │   ├── cpu_raytracer.h       # CPU 光线追踪实现
│   │   │   └── compute_raytracer.h   # Compute Shader 实现
│   │   ├── texture/                  # 纹理模块
│   │   │   ├── texture.h             # 纹理接口
│   │   │   ├── texture_manager.h     # 纹理管理器
│   │   │   └── sampler.h             # 采样器
│   │   ├── utils/                    # 工具模块
│   │   │   ├── image_io.h            # 图像读写
│   │   │   ├── random.h              # 随机数生成
│   │   │   ├── math_utils.h          # 数学工具函数
│   │   │   └── file_utils.h          # 文件工具函数
│   │   └── platform/                 # 平台相关模块
│   │       ├── window.h              # 窗口管理
│   │       └── gl_context.h          # OpenGL 上下文
│   └── extended_folders.list         # 模块分类文件夹列表
│
├── src/                              # 源代码目录
│   ├── core/
│   │   ├── config.cpp
│   │   ├── logger.cpp
│   │   └── profiler.cpp
│   ├── renderer/
│   │   ├── renderer.cpp
│   │   ├── render_context.cpp
│   │   └── render_stats.cpp
│   ├── scene/
│   │   ├── camera.cpp
│   │   ├── mesh.cpp
│   │   ├── material.cpp
│   │   ├── light.cpp
│   │   ├── directional_light.cpp
│   │   ├── point_light.cpp
│   │   ├── spot_light.cpp
│   │   └── scene_manager.cpp
│   ├── geometry/
│   │   ├── vertex.cpp
│   │   ├── triangle.cpp
│   │   ├── aabb.cpp
│   │   └── transform.cpp
│   ├── acceleration/
│   │   ├── bvh.cpp
│   │   ├── bvh_node.cpp
│   │   └── bvh_builder.cpp
│   ├── rasterizer/
│   │   ├── rasterizer.cpp
│   │   ├── gbuffer.cpp
│   │   └── shader_program.cpp
│   ├── raytracer/
│   │   ├── raytracer.cpp
│   │   ├── ray.cpp
│   │   ├── hit_record.cpp
│   │   ├── cpu_raytracer.cpp
│   │   └── compute_raytracer.cpp
│   ├── texture/
│   │   ├── texture.cpp
│   │   ├── texture_manager.cpp
│   │   └── sampler.cpp
│   ├── utils/
│   │   ├── image_io.cpp
│   │   ├── random.cpp
│   │   ├── math_utils.cpp
│   │   └── file_utils.cpp
│   └── platform/
│       ├── window.cpp
│       └── gl_context.cpp
│
├── shaders/                          # Shader 文件目录
│   ├── gbuffer/
│   │   ├── gbuffer.vert              # G-Buffer 顶点着色器
│   │   └── gbuffer.frag              # G-Buffer 片段着色器
│   ├── raytracing/
│   │   └── raytracing.comp           # 光线追踪计算着色器
│   ├── post_process/
│   │   ├── tonemap.vert              # Tone Mapping 顶点着色器
│   │   ├── tonemap.frag              # Tone Mapping 片段着色器
│   │   └── fullscreen_quad.vert      # 全屏四边形顶点着色器
│   └── debug/
│       ├── visualize_gbuffer.frag    # G-Buffer 可视化
│       └── visualize_bvh.frag        # BVH 可视化
│
├── lib/                              # 第三方库目录（用户自行安装）
│   ├── glfw/                         # GLFW（用户提供）
│   ├── glad/                         # GLAD（用户提供）
│   ├── glm/                          # GLM（用户提供）
│   ├── spdlog/                       # spdlog（用户提供）
│   └── stb/                          # stb 库（项目包含）
│       ├── stb_image.h
│       └── stb_image_write.h
│
├── examples/                         # 示例程序目录
│   ├── 01_hello_triangle/
│   │   ├── main.cpp                  # 基础三角形示例
│   │   └── CMakeLists.txt
│   ├── 02_cornell_box/
│   │   ├── main.cpp                  # Cornell Box 示例
│   │   └── CMakeLists.txt
│   ├── 03_material_showcase/
│   │   ├── main.cpp                  # 材质展示示例
│   │   └── CMakeLists.txt
│   └── 04_performance_test/
│       ├── main.cpp                  # 性能测试示例
│       └── CMakeLists.txt
│
├── experiments/                      # 实验和测试记录目录
│   └── .gitkeep
│
├── docs/                             # 文档目录
│   ├── API.md                        # API 文档
│   ├── ARCHITECTURE.md               # 架构设计文档
│   ├── BUILD.md                      # 构建指南
│   └── CODING_STYLE.md               # 编码规范
│
├── scripts/                          # 脚本目录
│   ├── export_modules_requirements.sh  # 导出模块依赖
│   ├── check_code.sh                   # 代码规范检查
│   └── setup_dependencies.sh           # 依赖配置脚本（您后续编写）
│
├── res/                              # 资源文件目录
│   └── icon.png                      # 项目图标
│
├── CMakeLists.txt                    # 主 CMake 配置文件
├── .gitignore                        # Git 忽略文件
├── .gitmodules                       # Git 子模块配置（如果使用）
├── LICENSE                           # 许可证
└── README.md                         # 项目说明
```
如下是模块间依赖关系示意图：
```
┌─────────────────────────────────────────────────────────────┐
│                         Renderer                            │
│                    (主渲染器接口)                            │
└────────────┬────────────────────────────────┬───────────────┘
             │                                │
             ▼                                ▼
    ┌────────────────┐              ┌─────────────────┐
    │  Rasterizer    │              │   RayTracer     │
    │  (光栅化器)     │              │  (光线追踪器)    │
    └────────┬───────┘              └────────┬────────┘
             │                               │
             ▼                               ▼
    ┌────────────────┐              ┌─────────────────┐
    │    GBuffer     │              │  CPU/Compute    │
    │  (G-Buffer)    │              │  (双实现)        │
    └────────────────┘              └────────┬────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │       BVH       │
                                    │   (加速结构)     │
                                    └────────┬────────┘
                                             │
             ┌───────────────────────────────┴───────────────┐
             │                                               │
             ▼                                               ▼
    ┌────────────────┐                            ┌─────────────────┐
    │ SceneManager   │                            │    Geometry     │
    │  (场景管理)     │                            │   (几何数据)     │
    └────────┬───────┘                            └─────────────────┘
             │
             ├─────► Camera (相机)
             ├─────► Mesh (网格)
             ├─────► Material (材质)
             └─────► Light (光源)
                        │
                        ├─────► DirectionalLight
                        ├─────► PointLight
                        └─────► SpotLight

┌─────────────────────────────────────────────────────────────┐
│                      支撑模块 (底层)                          │
├─────────────────────────────────────────────────────────────┤
│  Core: Config, Logger, Profiler, Types                     │
│  Texture: TextureManager, Sampler                          │
│  Utils: ImageIO, Random, MathUtils, FileUtils              │
│  Platform: Window, GLContext                               │
└─────────────────────────────────────────────────────────────┘
```

关于项目实现架构，我有以下步骤：
### Phase 1: 基础设施搭建（Week 1-2）
**目标**：建立项目骨架，确保编译通过

#### Step 1.1: 核心模块
```
优先级: ★★★★★
文件:
  - include/are/core/types.h          # 基础类型定义
  - include/are/core/config.h         # 配置系统
  - include/are/core/logger.h         # 日志系统
  - src/core/config.cpp
  - src/core/logger.cpp

依赖: spdlog

验证: 能够创建 Config 对象并输出日志
```

#### Step 1.2: 平台层
```
优先级: ★★★★★
文件:
  - include/are/platform/window.h     # 窗口管理
  - include/are/platform/gl_context.h # OpenGL 上下文
  - src/platform/window.cpp
  - src/platform/gl_context.cpp

依赖: GLFW, GLAD

验证: 能够创建窗口并初始化 OpenGL 上下文
```

#### Step 1.3: 工具模块
```
优先级: ★★★★☆
文件:
  - include/are/utils/math_utils.h    # 数学工具
  - include/are/utils/file_utils.h    # 文件工具
  - include/are/utils/random.h        # 随机数生成
  - src/utils/*.cpp

依赖: GLM

验证: 基础工具函数可用
```

---

### Phase 2: 几何与场景系统（Week 2-3）

**目标**：能够构建和管理场景数据

#### Step 2.1: 几何数据结构
```
优先级: ★★★★★
文件:
  - include/are/geometry/vertex.h     # 顶点结构
  - include/are/geometry/triangle.h   # 三角形
  - include/are/geometry/aabb.h       # 包围盒
  - include/are/geometry/transform.h  # 变换
  - src/geometry/*.cpp

依赖: GLM

验证: 能够创建和操作几何数据
```

#### Step 2.2: 场景对象
```
优先级: ★★★★★
文件:
  - include/are/scene/camera.h        # 相机
  - include/are/scene/mesh.h          # 网格
  - include/are/scene/material.h      # 材质
  - include/are/scene/light.h         # 光源基类
  - include/are/scene/directional_light.h
  - include/are/scene/point_light.h
  - include/are/scene/spot_light.h
  - src/scene/*.cpp

依赖: geometry

验证: 能够创建场景对象
```

#### Step 2.3: 场景管理器
```
优先级: ★★★★☆
文件:
  - include/are/scene/scene_manager.h
  - src/scene/scene_manager.cpp

依赖: scene/*

验证: 能够添加/删除/查询场景对象
```

---

### Phase 3: 光栅化管线（Week 3-4）

**目标**：实现 G-Buffer 生成

#### Step 3.1: Shader 管理
```
优先级: ★★★★★
文件:
  - include/are/rasterizer/shader_program.h
  - src/rasterizer/shader_program.cpp
  - shaders/gbuffer/gbuffer.vert
  - shaders/gbuffer/gbuffer.frag

依赖: platform/gl_context

验证: 能够加载和编译 Shader
```

#### Step 3.2: G-Buffer
```
优先级: ★★★★★
文件:
  - include/are/rasterizer/gbuffer.h
  - src/rasterizer/gbuffer.cpp

依赖: shader_program

验证: 能够创建 G-Buffer 纹理
```

#### Step 3.3: 光栅化器
```
优先级: ★★★★★
文件:
  - include/are/rasterizer/rasterizer.h
  - src/rasterizer/rasterizer.cpp

依赖: gbuffer, scene_manager

验证: 能够渲染场景到 G-Buffer
```

---

### Phase 4: 加速结构（Week 4-5）

**目标**：实现 BVH 构建

#### Step 4.1: BVH 节点
```
优先级: ★★★★★
文件:
  - include/are/acceleration/bvh_node.h
  - src/acceleration/bvh_node.cpp

依赖: geometry/aabb

验证: 能够创建 BVH 节点
```

#### Step 4.2: BVH 构建器
```
优先级: ★★★★★
文件:
  - include/are/acceleration/bvh_builder.h
  - src/acceleration/bvh_builder.cpp

依赖: bvh_node, scene/mesh

验证: 能够从三角形列表构建 BVH（使用 SAH 或中点分割）
```

#### Step 4.3: BVH 接口
```
优先级: ★★★★☆
文件:
  - include/are/acceleration/bvh.h
  - src/acceleration/bvh.cpp

依赖: bvh_builder

验证: 提供统一的 BVH 查询接口
```

---

### Phase 5: CPU 光线追踪（Week 5-6）

**目标**：实现基础的 CPU 光线追踪

#### Step 5.1: 光线与碰撞
```
优先级: ★★★★★
文件:
  - include/are/raytracer/ray.h
  - include/are/raytracer/hit_record.h
  - src/raytracer/ray.cpp
  - src/raytracer/hit_record.cpp

依赖: geometry

验证: 能够进行光线-三角形相交测试
```

#### Step 5.2: CPU 光线追踪器
```
优先级: ★★★★★
文件:
  - include/are/raytracer/cpu_raytracer.h
  - src/raytracer/cpu_raytracer.cpp

依赖: ray, hit_record, bvh, scene_manager

验证: 能够追踪光线并计算基础着色（漫反射）
```

#### Step 5.3: 多线程优化
```
优先级: ★★★★☆
修改: src/raytracer/cpu_raytracer.cpp

依赖: OpenMP 或 std::thread

验证: 多线程加速光线追踪
```

---

### Phase 6: Compute Shader 光线追踪（Week 6-7）

**目标**：实现 GPU 光线追踪

#### Step 6.1: Compute Shader 编写
```
优先级: ★★★★★
文件:
  - shaders/raytracing/raytracing.comp

依赖: OpenGL 4.3+

验证: Shader 能够编译通过
```

#### Step 6.2: GPU 数据传输
```
优先级: ★★★★★
文件:
  - include/are/raytracer/compute_raytracer.h
  - src/raytracer/compute_raytracer.cpp

依赖: rasterizer/shader_program, bvh

验证: 能够将 BVH 和场景数据上传到 GPU（SSBO）
```

#### Step 6.3: GPU 光线追踪实现
```
优先级: ★★★★★
修改: shaders/raytracing/raytracing.comp
      src/raytracer/compute_raytracer.cpp

验证: GPU 光线追踪能够产生正确结果
```

---

### Phase 7: 主渲染器集成（Week 7-8）

**目标**：整合所有模块

#### Step 7.1: 渲染上下文
```
优先级: ★★★★★
文件:
  - include/are/renderer/render_context.h
  - include/are/renderer/render_stats.h
  - src/renderer/render_context.cpp
  - src/renderer/render_stats.cpp

依赖: 所有模块

验证: 能够管理渲染状态
```

#### Step 7.2: 主渲染器
```
优先级: ★★★★★
文件:
  - include/are/renderer/renderer.h
  - src/renderer/renderer.cpp

依赖: rasterizer, raytracer, scene_manager

验证: 能够完成完整的混合渲染流程
```

#### Step 7.3: 后处理（Tone Mapping）
```
优先级: ★★★★☆
文件:
  - shaders/post_process/tonemap.vert
  - shaders/post_process/tonemap.frag

修改: src/renderer/renderer.cpp

验证: HDR 到 LDR 的转换
```

---

### Phase 8: 纹理系统（Week 8-9）

**目标**：支持纹理贴图

#### Step 8.1: 图像 I/O
```
优先级: ★★★★☆
文件:
  - include/are/utils/image_io.h
  - src/utils/image_io.cpp

依赖: stb_image, stb_image_write

验证: 能够加载和保存图像
```

#### Step 8.2: 纹理管理
```
优先级: ★★★★☆
文件:
  - include/are/texture/texture.h
  - include/are/texture/texture_manager.h
  - include/are/texture/sampler.h
  - src/texture/*.cpp

依赖: image_io

验证: 能够加载纹理并在材质中使用
```

#### Step 8.3: 集成到渲染管线
```
优先级: ★★★★☆
修改: src/rasterizer/rasterizer.cpp
      src/raytracer/cpu_raytracer.cpp
      shaders/raytracing/raytracing.comp

验证: 纹理能够正确显示
```

---

### Phase 9: 高级光线追踪特性（Week 9-11）

**目标**：实现高质量渲染特性

#### Step 9.1: 环境光遮蔽（AO）
```
优先级: ★★★★☆
修改: src/raytracer/cpu_raytracer.cpp
      shaders/raytracing/raytracing.comp

验证: AO 效果正确
```

#### Step 9.2: 软阴影
```
优先级: ★★★☆☆
修改: 光线追踪器实现

验证: 软阴影效果
```

#### Step 9.3: 镜面反射
```
优先级: ★★★☆☆
修改: 光线追踪器实现

验证: 镜面反射效果
```

#### Step 9.4: 折射/透明
```
优先级: ★★☆☆☆
修改: 光线追踪器实现

验证: 透明材质效果
```

---

### Phase 10: 调试与性能分析（Week 11-12）

**目标**：提供调试工具和性能优化

#### Step 10.1: 性能分析器
```
优先级: ★★★★☆
文件:
  - include/are/core/profiler.h
  - src/core/profiler.cpp

依赖: logger

验证: 能够测量各阶段耗时
```

#### Step 10.2: 调试可视化
```
优先级: ★★★☆☆
文件:
  - shaders/debug/visualize_gbuffer.frag
  - shaders/debug/visualize_bvh.frag

修改: src/renderer/renderer.cpp

验证: 能够可视化 G-Buffer 各通道和 BVH 包围盒
```

#### Step 10.3: 帧捕获功能
```
优先级: ★★★★☆
修改: src/renderer/renderer.cpp
      src/utils/image_io.cpp

验证: 能够保存渲染结果到文件
```

---

### Phase 11: 示例程序（Week 12-13）

**目标**：提供完整的使用示例

#### Step 11.1: Hello Triangle
```
优先级: ★★★★★
文件:
  - examples/01_hello_triangle/main.cpp
  - examples/01_hello_triangle/CMakeLists.txt

验证: 最简单的三角形渲染
```

#### Step 11.2: Cornell Box
```
优先级: ★★★★☆
文件:
  - examples/02_cornell_box/main.cpp
  - examples/02_cornell_box/CMakeLists.txt

验证: 经典的 Cornell Box 场景，展示全局光照
```

#### Step 11.3: Material Showcase
```
优先级: ★★★☆☆
文件:
  - examples/03_material_showcase/main.cpp
  - examples/03_material_showcase/CMakeLists.txt

验证: 展示不同材质参数的效果
```

#### Step 11.4: Performance Test
```
优先级: ★★★☆☆
文件:
  - examples/04_performance_test/main.cpp
  - examples/04_performance_test/CMakeLists.txt

验证: CPU vs GPU 性能对比
```

---

接下来是开发策略说明

### 开发原则

1. **自底向上构建**：
   - 先实现底层模块（core, utils, platform）
   - 再实现数据层（geometry, scene）
   - 最后实现算法层和接口层

2. **增量验证**：
   - 每个 Phase 完成后都有明确的验证目标
   - 通过简单的测试程序验证功能正确性
   - 避免大量代码堆积后才发现问题

3. **并行开发可能性**：
   - Phase 3（光栅化）和 Phase 4（BVH）可以并行开发
   - Phase 5（CPU 光追）和 Phase 6（GPU 光追）可以并行开发
   - 但建议先完成 CPU 版本，再实现 GPU 版本（便于调试）

4. **性能优先**：
   - 初期实现注重正确性
   - 后期通过 Profiler 识别瓶颈
   - 针对性优化热点代码

### 关键里程碑

**Milestone 1（Week 4）**：能够渲染静态场景到 G-Buffer
```cpp
// 验证代码
are::Renderer renderer(config);
renderer.add_mesh(triangle_mesh);
renderer.begin_frame();
renderer.render_gbuffer();  // 只渲染 G-Buffer
renderer.end_frame();
renderer.save_frame("gbuffer.png");
```

**Milestone 2（Week 6）**：CPU 光线追踪能够产生正确结果
```cpp
// 验证代码
config.ray_tracing.backend = are::ARE_RT_BACKEND_CPU;
config.ray_tracing.spp = 16;
renderer.render();  // 完整的混合渲染
renderer.save_frame("cpu_result.png");
```

**Milestone 3（Week 8）**：GPU 光线追踪能够产生正确结果
```cpp
// 验证代码
config.ray_tracing.backend = are::ARE_RT_BACKEND_COMPUTE_SHADER;
renderer.render();
renderer.save_frame("gpu_result.png");
// 对比 CPU 和 GPU 结果，误差应在可接受范围内
```

**Milestone 4（Week 10）**：支持纹理和多种光源
```cpp
// 验证代码
material.set_albedo_map("brick.png");
renderer.add_light(directional_light);
renderer.add_light(point_light);
renderer.render();
```

**Milestone 5（Week 14）**：项目完成，所有示例可运行

### 风险控制

**潜在风险**：
1. **BVH 构建性能**：如果场景复杂，构建时间可能过长
   - 缓解：使用多线程构建，考虑增量更新
   
2. **GPU 内存限制**：大场景可能超出 GPU 内存
   - 缓解：实现场景分块，按需加载

3. **Compute Shader 调试困难**：GPU 代码难以调试
   - 缓解：先在 CPU 上验证算法正确性，再移植到 GPU

4. **跨平台兼容性**：Windows 和 Linux 的差异
   - 缓解：使用 CMake 和标准库，避免平台特定代码

接下来是一些小总结

### 项目规模估算
- **总文件数**：约 80-100 个文件（头文件 + 源文件 + Shader）
- **代码量**：约 15,000-20,000 行（不含注释和空行）
- **开发周期**：14 周（约 3.5 个月）
- **核心模块**：10 个（core, platform, utils, geometry, scene, acceleration, rasterizer, raytracer, texture, renderer）

### 技术栈
- **语言**：C++17
- **图形 API**：OpenGL 4.3+
- **依赖库**：GLFW, GLAD, GLM, spdlog, stb
- **构建系统**：CMake 3.15+
- **并行**：OpenMP / std::thread

### 设计亮点
1. **模块化设计**：清晰的层次结构，低耦合高内聚
2. **双后端支持**：CPU 和 GPU 光线追踪可运行时切换
3. **工业级可扩展性**：预留接口，便于后期添加新特性
4. **性能优先**：BVH 加速、多线程、GPU 计算
5. **易于调试**：完善的日志系统、性能分析、可视化工具

---

此外，还有一些额外要求：
### ⚠️ 关键修改记录
1. **config.h 成员变量已去除下划线**
   - 原因：用户直接使用的结构体成员不加下划线
   - 影响：所有访问 config 的代码都使用无下划线版本
   - 示例：`config.window.width` 而不是 `config.window.width_`

### 📌 API 一致性检查清单
在实现时应该：
- [ ] 检查头文件中声明的所有函数是否都实现
- [ ] 不创造头文件中未声明的公共函数
- [ ] 成员变量命名符合规范（类成员变量后缀 `_`，用户结构体，即public成员变量不加）
- [ ] 所有公共 API 都有 Doxygen 注释
- [ ] 错误处理使用 `ARE_LOG_ERROR`
- [ ] 性能关键函数使用 `ARE_PROFILE_FUNCTION()`

### 🎯 Phase 2 成功标准
- [ ] 所有源文件编译通过（0 错误 0 警告）
- [ ] 验证程序能够创建场景对象
- [ ] 相机能够生成光线
- [ ] 网格能够计算 AABB
- [ ] 光源能够打包数据
- [ ] 场景管理器能够管理对象

对于目前的进度，我们已经实现了Phase 1，正在实现Phase 2，请你帮我们进行Phase 2的实现。
但是因为我还没有给你我们已有的代码，因此你需要哪些头文件的代码？请在下一轮向我询问，然后我们会开始Phase 2的开发。
