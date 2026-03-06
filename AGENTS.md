# AGENTS.md - Aurora Rendering Engine

## Project Overview
Aurora Rendering Engine (ARE) is a high-performance path tracing library in C++ developed by NanoEra Studio. It provides a rendering framework using OpenGL 4.3 with support for GPU-accelerated ray tracing, BVH acceleration, and denoising.

## Build Commands

### Standard Build
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### Build Types
- **Debug**: `cmake -DCMAKE_BUILD_TYPE=Debug ..`
- **Release**: `cmake -DCMAKE_BUILD_TYPE=Release ..` (default)

### Running Examples
The main example is the Cornell Box demo:
```bash
./examples/cornell_box
```

### Testing
This project currently has **no built-in unit tests**. Testing is performed manually by running the example executables. There are no test-specific build targets or test frameworks configured.

### Linting/Code Formatting
- **Format**: Use `.clang-format` configuration in project root
  ```bash
  clang-format -i src/*.cpp include/**/*.h
  ```
- **Clangd**: IDE integration via `.clangd` file (C++20, includes paths configured)

## Code Style Guidelines

### General Conventions
- **C++ Standard**: C++17 (project CMake), C++20 (clangd for IDE)
- **Indentation**: Tabs (see `.clang-format`: `UseTab: Always`)
- **Line Length**: Unlimited (`ColumnLimit: 0`)
- **Brace Style**: Attach (see `.clang-format`: `BreakBeforeBraces: Attach`)

### File Organization
- **Headers**: `include/` - Public API
- **Source**: `src/` - Implementation
- **Include format**: `#include "path/to/header.h"` (quotes for project headers)

### Naming Conventions
- **Classes**: `PascalCase` (e.g., `Renderer`, `Scene`)
- **Functions**: `PascalCase` (e.g., `initialize()`, `render()`)
- **Member variables**: `snake_case_` with trailing underscore (e.g., `config_`, `frame_count_`)
- **Types (aliases)**: `PascalCase` (e.g., `Vec3`, `Mat4`, `TextureHandle`)
- **Enums**: `PascalCase` with `k` prefix for values (e.g., `LogLevel::ARE_LOG_INFO`)

### Header Guards
```cpp
#ifndef ARE_INCLUDE_PATH_TO_FILENAME_H
#define ARE_INCLUDE_PATH_TO_FILENAME_H
// ... content ...
#endif // ARE_INCLUDE_PATH_TO_FILENAME_H
```

### Namespace
- All code lives in `are` namespace
- Namespace closing comment: `} // namespace are`

### Imports/Includes Order
1. Corresponding header (for .cpp files)
2. Project headers (alphabetically within group)
3. External library headers (e.g., `<glm/glm.hpp>`, `<glad/glad.h>`)
4. Standard library headers

### Comments
- Use Doxygen-style `/** */` or `/* */` for function documentation
- Brief descriptions in header files, implementation details in .cpp
- Avoid unnecessary inline comments

### Error Handling
- Use `ARE_LOG_ERROR()`, `ARE_LOG_WARN()`, `ARE_LOG_CRITICAL()` macros
- Return `false` on failure, `true` on success
- Check initialization states before operations
- Log with context: `"Failed to initialize X"`

### Smart Pointers
- Use `std::unique_ptr` for exclusive ownership
- Use `std::shared_ptr` for shared ownership
- Avoid raw `new`/`delete`

### GLM Types
Use GLM for all math types:
```cpp
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;
```

### OpenGL Resources
- Follow OpenGL best practices
- Check OpenGL errors during development (debug builds)
- Clean up resources in destructors or `shutdown()` methods

## Project Structure
```
ARE/
├── include/          # Public headers
│   ├── basic/        # Types, constants, math
│   ├── core/         # Renderer, raytracer, BVH
│   ├── scene/        # Scene objects (mesh, material, camera)
│   ├── resource/     # GPU resources (shader, texture, buffer)
│   └── utils/        # Utilities (logger, config)
├── src/              # Implementation files
├── examples/         # Example applications
├── shaders/          # GLSL shaders
├── lib/              # External libraries (glad, stb, spdlog)
└── build/            # Build output (generated)
```

## Key Classes
- `Renderer` - Main rendering engine interface
- `Scene` - Scene container (meshes, materials, lights, camera)
- `RayTracer` - GPU ray tracing implementation
- `BVH` - Acceleration structure
- `GBuffer` - Geometry buffer for deferred rendering

## Dependencies
- OpenGL 4.3
- GLFW 3
- GLAD (OpenGL loader)
- GLM (math library)
- stb-image (image loading)
- spdlog (logging)
