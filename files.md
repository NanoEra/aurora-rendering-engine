### 文件：include/are/core/types.h
```cpp
/**
 * @file types.h
 * @brief Basic type definitions and constants
 */

#ifndef ARE_INCLUDE_CORE_TYPES_H
#define ARE_INCLUDE_CORE_TYPES_H

#include <cstdint>
#include <glm/glm.hpp>

namespace are {

// Floating point precision
using Real = float;

// Common vector types
using Vec2 = glm::vec2;
using Vec3 = glm::vec3;
using Vec4 = glm::vec4;

using Vec2i = glm::ivec2;
using Vec3i = glm::ivec3;
using Vec4i = glm::ivec4;

// Matrix types
using Mat3 = glm::mat3;
using Mat4 = glm::mat4;

// Color type (RGBA)
using Color = Vec4;

// Handle types for resource management
using MeshHandle = uint32_t;
using MaterialHandle = uint32_t;
using LightHandle = uint32_t;
using TextureHandle = uint32_t;

// Invalid handle constant
constexpr uint32_t are_invalid_handle = 0xFFFFFFFF;

// Mathematical constants
constexpr Real are_pi = 3.14159265358979323846f;
constexpr Real are_two_pi = 6.28318530717958647692f;
constexpr Real are_inv_pi = 0.31830988618379067154f;
constexpr Real are_inv_two_pi = 0.15915494309189533577f;
constexpr Real are_epsilon = 1e-6f;

// Ray tracing backend types
enum class RayTracingBackend {
    ARE_RT_BACKEND_CPU,
    ARE_RT_BACKEND_COMPUTE_SHADER
};

// Tone mapping operators
enum class ToneMappingOperator {
    ARE_TONEMAP_NONE,
    ARE_TONEMAP_REINHARD,
    ARE_TONEMAP_ACES
};

// G-Buffer visualization modes
enum class GBufferVisualizationMode {
    ARE_GBUFFER_VIS_NONE,
    ARE_GBUFFER_VIS_POSITION,
    ARE_GBUFFER_VIS_NORMAL,
    ARE_GBUFFER_VIS_ALBEDO,
    ARE_GBUFFER_VIS_METALLIC,
    ARE_GBUFFER_VIS_ROUGHNESS,
    ARE_GBUFFER_VIS_DEPTH
};

} // namespace are

#endif // ARE_INCLUDE_CORE_TYPES_H
```

### 文件：include/are/core/logger.h
```cpp
/**
 * @file logger.h
 * @brief Logging system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_LOGGER_H
#define ARE_INCLUDE_CORE_LOGGER_H

#include <string>
#include <memory>

namespace are {

/**
 * @enum LogLevel
 * @brief Logging severity levels
 */
enum class LogLevel {
    ARE_LOG_TRACE,
    ARE_LOG_DEBUG,
    ARE_LOG_INFO,
    ARE_LOG_WARN,
    ARE_LOG_ERROR,
    ARE_LOG_CRITICAL
};

/**
 * @class Logger
 * @brief Thread-safe logging system
 * 
 * This class provides a simple interface for logging messages with different
 * severity levels. It wraps spdlog for actual logging functionality.
 */
class Logger {
public:
    /**
     * @brief Initialize the logging system
     * @param min_level Minimum log level to display
     */
    static void init(LogLevel min_level = LogLevel::ARE_LOG_INFO);

    /**
     * @brief Shutdown the logging system
     */
    static void shutdown();

    /**
     * @brief Log a message with file/function/line information
     * @param level Log severity level
     * @param file Source file name
     * @param func Function name
     * @param line Line number
     * @param message Log message
     */
    static void log(LogLevel level, const char* file, const char* func, 
                   int line, const std::string& message);

    /**
     * @brief Set minimum log level
     * @param level Minimum log level to display
     */
    static void set_level(LogLevel level);

private:
    static std::shared_ptr<void> logger_impl_; ///< Internal logger implementation
    static bool initialized_;                   ///< Initialization flag
};

} // namespace are

// Logging macros
#define ARE_LOG_TRACE(msg)    are::Logger::log(are::LogLevel::ARE_LOG_TRACE, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_DEBUG(msg)    are::Logger::log(are::LogLevel::ARE_LOG_DEBUG, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_INFO(msg)     are::Logger::log(are::LogLevel::ARE_LOG_INFO, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_WARN(msg)     are::Logger::log(are::LogLevel::ARE_LOG_WARN, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_ERROR(msg)    are::Logger::log(are::LogLevel::ARE_LOG_ERROR, __FILE__, __func__, __LINE__, msg)
#define ARE_LOG_CRITICAL(msg) are::Logger::log(are::LogLevel::ARE_LOG_CRITICAL, __FILE__, __func__, __LINE__, msg)

#endif // ARE_INCLUDE_CORE_LOGGER_H
```

### 文件：include/are/core/config.h
```cpp
/**
 * @file config.h
 * @brief Configuration system for the rendering engine
 */

#ifndef ARE_INCLUDE_CORE_CONFIG_H
#define ARE_INCLUDE_CORE_CONFIG_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @struct WindowConfig
 * @brief Configuration for window creation
 */
struct WindowConfig {
	int width = 1280; ///< Window width in pixels
	int height = 720; ///< Window height in pixels
	std::string title = "Aurora Rendering Engine"; ///< Window title
	bool resizable = true; ///< Whether window is resizable
	bool vsync = true; ///< Enable vertical synchronization
	int samples = 1; ///< MSAA samples (1 = disabled)
};

/**
 * @struct RayTracingConfig
 * @brief Configuration for ray tracing
 */
struct RayTracingConfig {
	RayTracingBackend backend = RayTracingBackend::ARE_RT_BACKEND_COMPUTE_SHADER;
	int spp = 64; ///< Samples per pixel
	int max_depth = 8; ///< Maximum ray bounce depth
	bool enable_gi = true; ///< Enable global illumination
	bool enable_ao = true; ///< Enable ambient occlusion
	bool enable_soft_shadows = false; ///< Enable soft shadows
	int ao_samples = 16; ///< AO sample count
	Real ao_radius = 1.0f; ///< AO sampling radius
};

/**
 * @struct RenderConfig
 * @brief General rendering configuration
 */
struct RenderConfig {
	ToneMappingOperator tonemap_op = ToneMappingOperator::ARE_TONEMAP_ACES;
	Real exposure = 1.0f; ///< Exposure value for tone mapping
	bool use_hdr = true; ///< Use HDR rendering pipeline
	GBufferVisualizationMode gbuffer_vis_mode = GBufferVisualizationMode::ARE_GBUFFER_VIS_NONE;
};

/**
 * @struct PerformanceConfig
 * @brief Performance-related configuration
 */
struct PerformanceConfig {
	int num_threads = 0; ///< Number of threads (0 = auto-detect)
	bool enable_bvh_multithreading = true; ///< Use multithreading for BVH construction
	bool enable_profiling = false; ///< Enable performance profiling
};

/**
 * @struct PathConfig
 * @brief File path configuration
 */
struct PathConfig {
	std::string shader_dir = "shaders/"; ///< Directory containing shader files
	std::string texture_dir = "textures/"; ///< Default texture directory
	std::string output_dir = "output/"; ///< Default output directory
};

/**
 * @class AreConfig
 * @brief Main configuration class for Aurora Rendering Engine
 */
class AreConfig {
public:
	WindowConfig window; ///< Window configuration
	RayTracingConfig ray_tracing; ///< Ray tracing configuration
	RenderConfig render; ///< Rendering configuration
	PerformanceConfig performance; ///< Performance configuration
	PathConfig paths; ///< Path configuration

	/**
	 * @brief Constructor with default values
	 */
	AreConfig() = default;

	/**
	 * @brief Validate configuration values
	 * @return true if configuration is valid, false otherwise
	 */
	bool validate() const;

	/**
	 * @brief Print configuration to console
	 */
	void print() const;
};

} // namespace are

#endif // ARE_INCLUDE_CORE_CONFIG_H
```

### 文件：include/are/utils/math_utils.h
```cpp
/**
 * @file math_utils.h
 * @brief Mathematical utility functions
 */

#ifndef ARE_INCLUDE_UTILS_MATH_UTILS_H
#define ARE_INCLUDE_UTILS_MATH_UTILS_H

#include <are/core/types.h>
#include <algorithm>

namespace are {

/**
 * @brief Clamp value to range [min, max]
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
template<typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor [0, 1]
 * @return Interpolated value
 */
template<typename T>
inline T lerp(T a, T b, Real t) {
    return a + (b - a) * t;
}

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline Real degrees_to_radians(Real degrees) {
    return degrees * are_pi / 180.0f;
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline Real radians_to_degrees(Real radians) {
    return radians * 180.0f / are_pi;
}

/**
 * @brief Check if two floating point values are approximately equal
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance
 * @return true if approximately equal
 */
inline bool approx_equal(Real a, Real b, Real epsilon = are_epsilon) {
    return std::abs(a - b) < epsilon;
}

/**
 * @brief Compute barycentric coordinates
 * @param p Point
 * @param a Triangle vertex A
 * @param b Triangle vertex B
 * @param c Triangle vertex C
 * @param u Output barycentric coordinate u
 * @param v Output barycentric coordinate v
 * @param w Output barycentric coordinate w
 */
void compute_barycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                        Real& u, Real& v, Real& w);

/**
 * @brief Reflect vector around normal
 * @param incident Incident vector
 * @param normal Surface normal (normalized)
 * @return Reflected vector
 */
Vec3 reflect(const Vec3& incident, const Vec3& normal);

/**
 * @brief Refract vector through surface
 * @param incident Incident vector (normalized)
 * @param normal Surface normal (normalized)
 * @param eta Ratio of refractive indices
 * @param refracted Output refracted vector
 * @return true if refraction occurred (false for total internal reflection)
 */
bool refract(const Vec3& incident, const Vec3& normal, Real eta, Vec3& refracted);

/**
 * @brief Compute Fresnel reflectance (Schlick approximation)
 * @param cos_theta Cosine of angle between view and normal
 * @param f0 Reflectance at normal incidence
 * @return Fresnel reflectance
 */
Real fresnel_schlick(Real cos_theta, Real f0);

/**
 * @brief Create orthonormal basis from normal
 * @param normal Normal vector (normalized)
 * @param tangent Output tangent vector
 * @param bitangent Output bitangent vector
 */
void create_orthonormal_basis(const Vec3& normal, Vec3& tangent, Vec3& bitangent);

/**
 * @brief Transform direction from tangent space to world space
 * @param tangent_dir Direction in tangent space
 * @param normal Surface normal
 * @param tangent Surface tangent
 * @param bitangent Surface bitangent
 * @return Direction in world space
 */
Vec3 tangent_to_world(const Vec3& tangent_dir, const Vec3& normal, 
                     const Vec3& tangent, const Vec3& bitangent);

} // namespace are

#endif // ARE_INCLUDE_UTILS_MATH_UTILS_H
```

### 文件：include/are/core/profiler.h
```cpp
/**
 * @file profiler.h
 * @brief Performance profiling utilities
 */

#ifndef ARE_INCLUDE_CORE_PROFILER_H
#define ARE_INCLUDE_CORE_PROFILER_H

#include <are/core/types.h>
#include <string>
#include <chrono>
#include <unordered_map>

namespace are {

/**
 * @struct ProfileResult
 * @brief Result of a profiling measurement
 */
struct ProfileResult {
    std::string name_;                    ///< Profile section name
    double duration_ms_;                  ///< Duration in milliseconds
    uint64_t call_count_;                 ///< Number of times called
    double avg_duration_ms_;              ///< Average duration per call
};

/**
 * @class Profiler
 * @brief Simple performance profiler
 * 
 * This class provides basic timing functionality for performance analysis.
 * It is only active when ARE_ENABLE_PROFILING is defined.
 */
class Profiler {
public:
    /**
     * @brief Initialize the profiler
     */
    static void init();

    /**
     * @brief Shutdown the profiler and print results
     */
    static void shutdown();

    /**
     * @brief Begin a profiling section
     * @param name Section name
     */
    static void begin(const std::string& name);

	/**
     * @brief End a profiling section
     * @param name Section name
     */
    static void end(const std::string& name);

    /**
     * @brief Get profiling results
     * @return Map of section names to profile results
     */
    static const std::unordered_map<std::string, ProfileResult>& get_results();

    /**
     * @brief Reset all profiling data
     */
    static void reset();

    /**
     * @brief Print profiling results to console
     */
    static void print_results();

private:
    struct SectionData {
        std::chrono::high_resolution_clock::time_point start_time_;
        double total_duration_ms_ = 0.0;
        uint64_t call_count_ = 0;
    };

    static std::unordered_map<std::string, SectionData> sections_;
    static std::unordered_map<std::string, ProfileResult> results_;
    static bool enabled_;
};

/**
 * @class ScopedProfiler
 * @brief RAII-style profiler for automatic timing
 */
class ScopedProfiler {
public:
    /**
     * @brief Constructor - begins profiling
     * @param name Section name
     */
    explicit ScopedProfiler(const std::string& name);

    /**
     * @brief Destructor - ends profiling
     */
    ~ScopedProfiler();

private:
    std::string name_;
};

} // namespace are

// Profiling macros
#ifdef ARE_ENABLE_PROFILING
	#define ARE_PROFILE_BEGIN(name) are::Profiler::begin(name)
    #define ARE_PROFILE_END(name)   are::Profiler::end(name)
    #define ARE_PROFILE_SCOPE(name) are::ScopedProfiler are_profiler_##__LINE__(name)
    #define ARE_PROFILE_FUNCTION()  ARE_PROFILE_SCOPE(__func__)
#else
    #define ARE_PROFILE_BEGIN(name) ((void)0)
    #define ARE_PROFILE_END(name)   ((void)0)
    #define ARE_PROFILE_SCOPE(name) ((void)0)
    #define ARE_PROFILE_FUNCTION()  ((void)0)
#endif

#endif // ARE_INCLUDE_CORE_PROFILER_H
```

### 文件：src/core/logger.cpp
```cpp
/**
 * @file logger.cpp
 * @brief Implementation of logging system
 */

#include <are/core/logger.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <mutex>
#include <cstring>

namespace are {

std::shared_ptr<void> Logger::logger_impl_ = nullptr;
bool Logger::initialized_ = false;

void Logger::init(LogLevel min_level) {
    if (initialized_) {
        return;
    }

    try {
        // Create console sink with color support
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

        // Create logger with console sink
        auto logger = std::make_shared<spdlog::logger>("are", console_sink);
        
        // Set log level
        switch (min_level) {
            case LogLevel::ARE_LOG_TRACE:    
                logger->set_level(spdlog::level::trace); 
                break;
            case LogLevel::ARE_LOG_DEBUG:    
                logger->set_level(spdlog::level::debug); 
                break;
            case LogLevel::ARE_LOG_INFO:     
                logger->set_level(spdlog::level::info); 
                break;
            case LogLevel::ARE_LOG_WARN:     
                logger->set_level(spdlog::level::warn); 
                break;
            case LogLevel::ARE_LOG_ERROR:    
                logger->set_level(spdlog::level::err); 
                break;
            case LogLevel::ARE_LOG_CRITICAL: 
                logger->set_level(spdlog::level::critical); 
                break;
        }

        // Flush on error or higher
        logger->flush_on(spdlog::level::err);

        // Set as default logger
        spdlog::set_default_logger(logger);
        logger_impl_ = logger;
        initialized_ = true;

    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Failed to initialize logger: %s\n", e.what());
    }
}

void Logger::shutdown() {
    if (!initialized_) {
        return;
    }

    try {
        spdlog::shutdown();
        logger_impl_.reset();
        initialized_ = false;
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Error during logger shutdown: %s\n", e.what());
    }
}

void Logger::log(LogLevel level, const char* file, const char* func, 
                int line, const std::string& message) {
    if (!initialized_) {
        init();
    }

    // Extract filename from full path
    const char* filename = file;
    const char* last_slash = nullptr;
    
    for (const char* p = file; *p; ++p) {
        if (*p == '/' || *p == '\\') {
            last_slash = p;
        }
    }
    
    if (last_slash) {
        filename = last_slash + 1;
    }

    // Format message with location information
    std::string formatted = message + " (" + filename + ":" + std::to_string(line) + ")";

    try {
        auto logger = std::static_pointer_cast<spdlog::logger>(logger_impl_);
        
        switch (level) {
            case LogLevel::ARE_LOG_TRACE:
                logger->trace(formatted);
                break;
            case LogLevel::ARE_LOG_DEBUG:
                logger->debug(formatted);
                break;
            case LogLevel::ARE_LOG_INFO:
                logger->info(formatted);
                break;
            case LogLevel::ARE_LOG_WARN:
                logger->warn(formatted);
                break;
            case LogLevel::ARE_LOG_ERROR:
                logger->error(formatted);
                break;
            case LogLevel::ARE_LOG_CRITICAL:
                logger->critical(formatted);
                break;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Logging error: %s\n", e.what());
    }
}

void Logger::set_level(LogLevel level) {
    if (!initialized_) {
        return;
    }

    try {
        auto logger = std::static_pointer_cast<spdlog::logger>(logger_impl_);
        
        switch (level) {
            case LogLevel::ARE_LOG_TRACE:    
                logger->set_level(spdlog::level::trace); 
                break;
            case LogLevel::ARE_LOG_DEBUG:    
                logger->set_level(spdlog::level::debug); 
                break;
            case LogLevel::ARE_LOG_INFO:     
                logger->set_level(spdlog::level::info); 
                break;
            case LogLevel::ARE_LOG_WARN:     
                logger->set_level(spdlog::level::warn); 
                break;
            case LogLevel::ARE_LOG_ERROR:    
                logger->set_level(spdlog::level::err); 
                break;
            case LogLevel::ARE_LOG_CRITICAL: 
                logger->set_level(spdlog::level::critical); 
                break;
        }
    } catch (const std::exception& e) {
        fprintf(stderr, "[ARE] Error setting log level: %s\n", e.what());
    }
}

} // namespace are
```

### 文件：CMakeLists.txt
```CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(AuroraRenderingEngine VERSION 0.1.0 LANGUAGES CXX C)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set C standard for GLAD
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build options
option(ARE_BUILD_SHARED "Build shared library" OFF)
option(ARE_BUILD_EXAMPLES "Build example programs" ON)
option(ARE_ENABLE_PROFILING "Enable performance profiling" ON)
option(ARE_ENABLE_DEBUG_VIS "Enable debug visualization" ON)

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Compiler flags
if(MSVC)
    add_compile_options(/W4)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    # Disable specific warnings that are too strict
    add_compile_options(/wd4100)  # unreferenced formal parameter
else()
    add_compile_options(-Wall -Wextra -pedantic)
    # Disable specific warnings
    add_compile_options(-Wno-unused-parameter)
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Try to find spdlog (system installation)
find_package(spdlog QUIET)

if(NOT spdlog_FOUND)
    message(STATUS "spdlog not found in system, using local version")
    # Use local spdlog
    set(SPDLOG_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/spdlog/include")
    if(NOT EXISTS ${SPDLOG_INCLUDE_DIR})
        message(FATAL_ERROR "spdlog not found. Please install spdlog or place it in lib/spdlog/")
    endif()
    add_library(spdlog INTERFACE)
    target_include_directories(spdlog INTERFACE ${SPDLOG_INCLUDE_DIR})
    # spdlog compile definitions
    target_compile_definitions(spdlog INTERFACE SPDLOG_COMPILED_LIB)
endif()

# Find OpenMP (optional)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(ARE_USE_OPENMP ON)
    add_compile_definitions(ARE_USE_OPENMP)
    message(STATUS "OpenMP found and enabled")
else()
    message(STATUS "OpenMP not found, multithreading will use std::thread")
endif()

# GLAD library path
set(GLAD_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/glad")
set(GLAD_SOURCE_DIR "${CMAKE_SOURCE_DIR}/lib/glad")

if(NOT EXISTS ${GLAD_INCLUDE_DIR})
    message(FATAL_ERROR "GLAD include directory not found: ${GLAD_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${GLAD_SOURCE_DIR})
    message(FATAL_ERROR "GLAD source directory not found: ${GLAD_SOURCE_DIR}")
endif()

# STB library path
set(STB_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lib/stb")

if(NOT EXISTS ${STB_INCLUDE_DIR})
    message(FATAL_ERROR "STB include directory not found: ${STB_INCLUDE_DIR}")
endif()

# Collect all source files from src/
file(GLOB_RECURSE ARE_SOURCES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)

# Collect all GLAD source files
file(GLOB GLAD_SOURCES 
    "${GLAD_SOURCE_DIR}/*.c"
)

# Add GLAD sources to ARE sources
list(APPEND ARE_SOURCES ${GLAD_SOURCES})

# Collect all header files
file(GLOB_RECURSE ARE_HEADERS 
    "${CMAKE_SOURCE_DIR}/include/*.h"
    "${CMAKE_SOURCE_DIR}/include/*.hpp"
)

# Create library
if(ARE_BUILD_SHARED)
    add_library(are SHARED ${ARE_SOURCES} ${ARE_HEADERS})
    target_compile_definitions(are PRIVATE ARE_BUILD_SHARED)
    message(STATUS "Building shared library")
else()
    add_library(are STATIC ${ARE_SOURCES} ${ARE_HEADERS})
    message(STATUS "Building static library")
endif()

# Set target properties
set_target_properties(are PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "${ARE_HEADERS}"
)

# Include directories
target_include_directories(are
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${GLAD_INCLUDE_DIR}
        ${STB_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(are
    PUBLIC
        OpenGL::GL
        glfw
        glm::glm
)

# Link spdlog
if(spdlog_FOUND)
    target_link_libraries(are PUBLIC spdlog::spdlog)
else()
    target_link_libraries(are PUBLIC spdlog)
    target_include_directories(are PRIVATE ${SPDLOG_INCLUDE_DIR})
endif()

# Link OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(are PUBLIC OpenMP::OpenMP_CXX)
endif()

# Platform-specific libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(are PUBLIC dl pthread)
endif()

# Compile definitions
if(ARE_ENABLE_PROFILING)
    target_compile_definitions(are PUBLIC ARE_ENABLE_PROFILING)
    message(STATUS "Profiling enabled")
endif()

if(ARE_ENABLE_DEBUG_VIS)
    target_compile_definitions(are PUBLIC ARE_ENABLE_DEBUG_VIS)
    message(STATUS "Debug visualization enabled")
endif()

# Build examples
if(ARE_BUILD_EXAMPLES)
    # Define helper function for creating examples
    function(add_are_example EXAMPLE_NAME)
        add_executable(${EXAMPLE_NAME} ${ARGN})
        target_link_libraries(${EXAMPLE_NAME} PRIVATE are)
        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    endfunction()
    
    # Add example subdirectories
    add_subdirectory(examples/00_phase1_test)
    
    message(STATUS "Examples will be built")
endif()

# Installation rules
install(TARGETS are
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/are
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/are
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/shaders
    DESTINATION share/are/shaders
)

# Print configuration summary
message(STATUS "")
message(STATUS "========================================")
message(STATUS "Aurora Rendering Engine Configuration")
message(STATUS "========================================")
message(STATUS "  Version:              ${PROJECT_VERSION}")
message(STATUS "  Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "  Library type:         ${ARE_BUILD_SHARED}")
message(STATUS "  C++ standard:         ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler:             ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Features:")
message(STATUS "  OpenMP support:       ${ARE_USE_OPENMP}")
message(STATUS "  Build examples:       ${ARE_BUILD_EXAMPLES}")
message(STATUS "  Enable profiling:     ${ARE_ENABLE_PROFILING}")
message(STATUS "  Enable debug vis:     ${ARE_ENABLE_DEBUG_VIS}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  OpenGL:               ${OPENGL_LIBRARIES}")
message(STATUS "  GLFW:                 Found")
message(STATUS "  GLM:                  Found")
if(spdlog_FOUND)
    message(STATUS "  spdlog:               Found (system)")
else()
    message(STATUS "  spdlog:               Found (local)")
endif()
message(STATUS "  GLAD:                 ${GLAD_INCLUDE_DIR}")
message(STATUS "  STB:                  ${STB_INCLUDE_DIR}")
message(STATUS "")
message(STATUS "Output directories:")
message(STATUS "  Executables:          ${CMAKE_BINARY_DIR}/bin")
message(STATUS "  Libraries:            ${CMAKE_BINARY_DIR}/lib")
message(STATUS "========================================")
message(STATUS "")
```

接下来，其实我已经实现了所有头文件，下面是Phase 2的头文件：
### 文件：vertex.h
```cpp
/**
 * @file vertex.h
 * @brief Vertex data structure definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_VERTEX_H
#define ARE_INCLUDE_GEOMETRY_VERTEX_H

#include <are/core/types.h>

namespace are {

/**
 * @struct Vertex
 * @brief Standard vertex structure for mesh data
 * 
 * Contains position, normal, texture coordinates, and tangent information
 * for PBR rendering pipeline.
 */
struct Vertex {
    Vec3 position_;///< Vertex position in object space
    Vec3 normal_;                         ///< Vertex normal (normalized)
    Vec2 texcoord_;                       ///< Texture coordinates (UV)
    Vec3 tangent_;                        ///< Tangent vector for normal mapping

    Vertex() = default;

    /**
     * @brief Construct vertex with position only
     * @param pos Position
     */
    explicit Vertex(const Vec3& pos);

    /**
     * @brief Construct vertex with position and normal
     * @param pos Position
     * @param norm Normal
     */
    Vertex(const Vec3& pos, const Vec3& norm);

    /**
     * @brief Construct vertex with position, normal, and texcoord
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv);

    /**
     * @brief Construct vertex with all attributes
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     * @param tan Tangent
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv, const Vec3& tan);

    /**
     * @brief Interpolate between two vertices
     * @param a First vertex
     * @param b Second vertex
     * @param t Interpolation factor [0, 1]
     * @return Interpolated vertex
     */
    static Vertex lerp(const Vertex& a, const Vertex& b, Real t);
};

/**
 * @brief Get vertex attribute stride for OpenGL
 * @return Size of Vertex structure in bytes
 */
constexpr size_t get_vertex_stride() {
    return sizeof(Vertex);
}

/**
 * @brief Get offset of position attribute
 * @return Offset in bytes
 */
constexpr size_t get_position_offset() {
    return offsetof(Vertex, position_);
}

/**
 * @brief Get offset of normal attribute
 * @return Offset in bytes
 */
constexpr size_t get_normal_offset() {
    return offsetof(Vertex, normal_);
}

/**
 * @brief Get offset of texcoord attribute
 * @return Offset in bytes
 */
constexpr size_t get_texcoord_offset() {
    return offsetof(Vertex, texcoord_);
}

/**
 * @brief Get offset of tangent attribute
 * @return Offset in bytes
 */
constexpr size_t get_tangent_offset() {
    return offsetof(Vertex, tangent_);
}

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_VERTEX_H
```

### 文件：triangle.h
```cpp
/**
 * @file triangle.h
 * @brief Triangle primitive definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRIANGLE_H
#define ARE_INCLUDE_GEOMETRY_TRIANGLE_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>

namespace are {

// Forward declaration
struct Ray;
struct HitRecord;

/**
 * @struct Triangle
 * @brief Triangle primitive for ray tracing
 * 
 * Stores three vertices and provides intersection testing.
 */
struct Triangle {
    Vertex v0_;                           ///< First vertex
    Vertex v1_;                           ///< Second vertex
    Vertex v2_;                           ///< Third vertex
    MaterialHandle material_;             ///< Material handle

    /**
     * @brief Default constructor
     */
    Triangle();

    /**
     * @brief Construct triangle from three vertices
     * @param v0 First vertex
     * @param v1 Second vertex
     * @param v2 Third vertex
     * @param material Material handle
     */
    Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2,
            MaterialHandle material = are_invalid_handle);

    /**
     * @brief Get triangle centroid
     * @return Centroid position
     */
    Vec3 centroid() const;

    /**
     * @brief Get triangle normal (geometric normal)
     * @return Normal vector (normalized)
     */
    Vec3 normal() const;

    /**
     * @brief Get triangle area
     * @return Area
     */
    Real area() const;

    /**
     * @brief Compute axis-aligned bounding box
     * @return AABB containing the triangle
     */
    AABB compute_aabb() const;

    /**
     * @brief Ray-triangle intersection test (Möller-Trumbore algorithm)
     * @param ray Ray to test
     * @param hit Output hit record
     * @return true if intersection occurred
     */
    bool intersect(const Ray& ray, HitRecord& hit) const;

    /**
     * @brief Fast ray-triangle intersection test (no hit record)
     * @param ray Ray to test
     * @param t_max Maximum t value
     * @return true if intersection occurred
     */
    bool intersect_fast(const Ray& ray, Real t_max) const;
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRIANGLE_H
```

### 文件：aabb.h
```cpp
/**
 * @file aabb.h
 * @brief Axis-Aligned Bounding Box implementation
 */

#ifndef ARE_INCLUDE_GEOMETRY_AABB_H
#define ARE_INCLUDE_GEOMETRY_AABB_H

#include <are/core/types.h>

namespace are {

// Forward declaration
struct Ray;

/**
 * @class AABB
 * @brief Axis-Aligned Bounding Box for spatial queries
 * 
 * Used for BVH construction and ray intersection acceleration.
 */
class AABB {
public:
    Vec3 min_;                            ///< Minimum corner
    Vec3 max_;                            ///< Maximum corner

    /**
     * @brief Default constructor - creates invalid AABB
     */
    AABB();

    /**
     * @brief Construct AABB from min and max corners
     * @param min Minimum corner
     * @param max Maximum corner
     */
    AABB(const Vec3& min, const Vec3& max);

    /**
     * @brief Construct AABB containing a single point
     * @param point Point to contain
     */
    explicit AABB(const Vec3& point);

    /**
     * @brief Check if AABB is valid (min <= max)
     * @return true if valid
     */
    bool is_valid() const;

    /**
     * @brief Get center of AABB
     * @return Center point
     */
    Vec3 center() const;

    /**
     * @brief Get size (extent) of AABB
     * @return Size vector
     */
    Vec3 size() const;

    /**
     * @brief Get surface area of AABB
     * @return Surface area
     */
    Real surface_area() const;

    /**
     * @brief Get volume of AABB
     * @return Volume
     */
    Real volume() const;

    /**
     * @brief Get longest axis index (0=x, 1=y, 2=z)
     * @return Axis index
     */
    int longest_axis() const;

    /**
     * @brief Expand AABB to include a point
     * @param point Point to include
     */
    void expand(const Vec3& point);

    /**
     * @brief Expand AABB to include another AABB
     * @param other AABB to include
     */
    void expand(const AABB& other);

    /**
     * @brief Check if AABB contains a point
     * @param point Point to check
     * @return true if point is inside AABB
     */
    bool contains(const Vec3& point) const;

    /**
     * @brief Check if AABB intersects another AABB
     * @param other AABB to check
     * @return true if AABBs intersect
     */
    bool intersects(const AABB& other) const;

    /**
     * @brief Ray-AABB intersection test
     * @param ray Ray to test
     * @param t_min Minimum t value (output)
     * @param t_max Maximum t value (output)
     * @return true if ray intersects AABB
     */
    bool intersect_ray(const Ray& ray, Real& t_min, Real& t_max) const;

    /**
     * @brief Merge two AABBs
     * @param a First AABB
     * @param b Second AABB
     * @return Merged AABB containing both
     */
    static AABB merge(const AABB& a, const AABB& b);

    /**
     * @brief Create invalid AABB (for initialization)
     * @return Invalid AABB
     */
    static AABB invalid();
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_AABB_H
```

### 文件：transform.h
```cpp
/**
 * @file transform.h
 * @brief Transformation matrix utilities
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRANSFORM_H
#define ARE_INCLUDE_GEOMETRY_TRANSFORM_H

#include <are/core/types.h>

namespace are {

/**
 * @class Transform
 * @brief 3D transformation (position, rotation, scale)
 * 
 * Provides convenient interface for building transformation matrices.
 */
class Transform {
public:
    /**
     * @brief Default constructor (identity transform)
     */
    Transform();

    /**
     * @brief Construct from position, rotation, and scale
     * @param position Translation
     * @param rotation Rotation (Euler angles in radians)
     * @param scale Scale factors
     */
    Transform(const Vec3& position, const Vec3& rotation, const Vec3& scale);

    // Setters
    void set_position(const Vec3& position);
    void set_rotation(const Vec3& rotation);
    void set_scale(const Vec3& scale);
    void set_scale(Real uniform_scale);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_rotation() const { return rotation_; }
    const Vec3& get_scale() const { return scale_; }

    // Matrix operations
    Mat4 get_matrix() const;
    Mat4 get_inverse_matrix() const;
    Mat3 get_normal_matrix() const;

    /**
     * @brief Transform a point
     * @param point Point to transform
     * @return Transformed point
     */
    Vec3 transform_point(const Vec3& point) const;

    /**
     * @brief Transform a direction (ignores translation)
     * @param direction Direction to transform
     * @return Transformed direction
     */
    Vec3 transform_direction(const Vec3& direction) const;

    /**
     * @brief Transform a normal (uses inverse transpose)
     * @param normal Normal to transform
     * @return Transformed normal
     */
    Vec3 transform_normal(const Vec3& normal) const;

    /**
     * @brief Combine two transforms
     * @param other Other transform
     * @return Combined transform
     */
    Transform operator*(const Transform& other) const;

    /**
     * @brief Create identity transform
     * @return Identity transform
     */
    static Transform identity();

    /**
     * @brief Create translation transform
     * @param translation Translation vector
     * @return Translation transform
     */
    static Transform translate(const Vec3& translation);

    /**
     * @brief Create rotation transform
     * @param rotation Rotation (Euler angles in radians)
     * @return Rotation transform
     */
    static Transform rotate(const Vec3& rotation);

    /**
     * @brief Create scale transform
     * @param scale Scale factors
     * @return Scale transform
     */
    static Transform scale(const Vec3& scale);

private:
    void mark_dirty();
    void update_matrix() const;

    Vec3 position_;                       ///< Translation
    Vec3 rotation_;                       ///< Rotation (Euler angles)
    Vec3 scale_;                          ///< Scale factors

    mutable Mat4 matrix_;                 ///< Cached transformation matrix
    mutable Mat4 inverse_matrix_;         ///< Cached inverse matrix
    mutable bool dirty_;                  ///< Matrix needs update
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRANSFORM_H
```

### 文件：camera.h
```cpp
/**
 * @file camera.h
 * @brief Camera class for view and projection management
 */

#ifndef ARE_INCLUDE_SCENE_CAMERA_H
#define ARE_INCLUDE_SCENE_CAMERA_H

#include <are/core/types.h>

namespace are {

/**
 * @class Camera
 * @brief Perspective camera for rendering
 * 
 * Manages view and projection matrices, and provides ray generation
 * for ray tracing.
 */
class Camera {
public:
    /**
     * @brief Default constructor
     */
    Camera();

    /**
     * @brief Construct camera with position and target
     * @param position Camera position
     * @param target Look-at target
     * @param up Up vector
     */
    Camera(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Position and orientation setters
    void set_position(const Vec3& position);
    void set_target(const Vec3& target);
    void set_up(const Vec3& up);
    void look_at(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Projection setters
    void set_fov(Real fov_degrees);
    void set_aspect_ratio(Real aspect);
    void set_near_plane(Real near);
    void set_far_plane(Real far);
    void set_perspective(Real fov_degrees, Real aspect, Real near, Real far);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_target() const { return target_; }
    const Vec3& get_up() const { return up_; }
    Real get_fov() const { return fov_; }
    Real get_aspect_ratio() const { return aspect_ratio_; }
    Real get_near_plane() const { return near_plane_; }
    Real get_far_plane() const { return far_plane_; }

    // Direction vectors
    Vec3 get_forward() const;
    Vec3 get_right() const;

    // Matrix getters
    const Mat4& get_view_matrix() const;
    const Mat4& get_projection_matrix() const;
    Mat4 get_view_projection_matrix() const;

    /**
     * @brief Generate ray for given pixel coordinates
     * @param u Normalized x coordinate [0, 1]
     * @param v Normalized y coordinate [0, 1]
     * @param origin Output ray origin
     * @param direction Output ray direction (normalized)
     */
    void generate_ray(Real u, Real v, Vec3& origin, Vec3& direction) const;

    /**
     * @brief Check if camera parameters have changed
     * @return true if matrices need recalculation
     */
    bool is_dirty() const { return dirty_; }

    /**
     * @brief Mark camera as clean after matrix update
     */
    void clear_dirty() { dirty_ = false; }

private:
    void update_view_matrix() const;
    void update_projection_matrix() const;

    Vec3 position_;                       ///< Camera position
    Vec3 target_;                         ///< Look-at target
    Vec3 up_;                             ///< Up vector

    Real fov_;                            ///< Field of view in degrees
    Real aspect_ratio_;                   ///< Aspect ratio (width/height)
    Real near_plane_;                     ///< Near clipping plane
    Real far_plane_;                      ///< Far clipping plane

    mutable Mat4 view_matrix_;            ///< Cached view matrix
    mutable Mat4 projection_matrix_;      ///< Cached projection matrix
    mutable bool view_dirty_;             ///< View matrix needs update
    mutable bool projection_dirty_;       ///< Projection matrix needs update
    bool dirty_;                          ///< Any parameter changed
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_CAMERA_H
```

### 文件：mesh.h
```cpp
/**
 * @file mesh.h
 * @brief Mesh class for geometry storage
 */

#ifndef ARE_INCLUDE_SCENE_MESH_H
#define ARE_INCLUDE_SCENE_MESH_H

#include <are/core/types.h>
#include <are/geometry/vertex.h>
#include <are/geometry/aabb.h>
#include <vector>

namespace are {

/**
 * @class Mesh
 * @brief Triangle mesh container
 * 
 * Stores vertex and index data for a triangle mesh.
 * Supports automatic AABB computation.
 */
class Mesh {
public:
    /**
     * @brief Default constructor - creates empty mesh
     */
    Mesh();

    /**
     * @brief Construct mesh from vertex and index data
     * @param vertices Vertex array
     * @param indices Index array (triangles)
     * @param material_id Material handle
     */
    Mesh(const std::vector<Vertex>& vertices, 
         const std::vector<uint32_t>& indices,
         MaterialHandle material_id = are_invalid_handle);

    /**
     * @brief Construct mesh from raw arrays
     * @param vertices Vertex array pointer
     * @param vertex_count Number of vertices
     * @param indices Index array pointer
     * @param index_count Number of indices
     * @param material_id Material handle
     */
    Mesh(const Vertex* vertices, size_t vertex_count,
         const uint32_t* indices, size_t index_count,
         MaterialHandle material_id = are_invalid_handle);

    // Data setters
    void set_vertices(const std::vector<Vertex>& vertices);
    void set_indices(const std::vector<uint32_t>& indices);
    void set_material(MaterialHandle material_id);

    // Data getters
    const std::vector<Vertex>& get_vertices() const { return vertices_; }
    const std::vector<uint32_t>& get_indices() const { return indices_; }
    MaterialHandle get_material() const { return material_id_; }

    // Geometry queries
    size_t get_vertex_count() const { return vertices_.size(); }
    size_t get_index_count() const { return indices_.size(); }
    size_t get_triangle_count() const { return indices_.size() / 3; }
    bool is_empty() const { return vertices_.empty() || indices_.empty(); }

    // AABB
    const AABB& get_aabb() const { return aabb_; }
    void compute_aabb();

    /**
     * @brief Compute tangent vectors for normal mapping
     */
    void compute_tangents();/**
     * @brief Get triangle vertices by index
     * @param triangle_index Triangle index
     * @param v0 Output first vertex
     * @param v1 Output second vertex
     * @param v2 Output third vertex
     * @return true if triangle exists
     */
    bool get_triangle(size_t triangle_index, Vertex& v0, Vertex& v1, Vertex& v2) const;

    // GPU resource handles (set by Renderer)
    void set_vao(uint32_t vao) { vao_ = vao; }
    void set_vbo(uint32_t vbo) { vbo_ = vbo; }
    void set_ebo(uint32_t ebo) { ebo_ = ebo; }
    uint32_t get_vao() const { return vao_; }
    uint32_t get_vbo() const { return vbo_; }
    uint32_t get_ebo() const { return ebo_; }
    bool has_gpu_resources() const { return vao_ != 0; }

private:
    std::vector<Vertex> vertices_;        ///< Vertex data
    std::vector<uint32_t> indices_;       ///< Index data (triangles)
    MaterialHandle material_id_;          ///< Associated material
    AABB aabb_;                           ///< Bounding box

    // GPU resources
    uint32_t vao_;                         ///< Vertex Array Object
    uint32_t vbo_;                         ///< Vertex Buffer Object
    uint32_t ebo_;                         ///< Element Buffer Object
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MESH_H
```

### 文件：material.h
```cpp
/**
 * @file material.h
 * @brief PBR material definition
 */

#ifndef ARE_INCLUDE_SCENE_MATERIAL_H
#define ARE_INCLUDE_SCENE_MATERIAL_H

#include <are/core/types.h>
#include <string>

namespace are {

/**
 * @class Material
 * @brief Physically-based rendering material
 * 
 * Supports standard PBR workflow with metallic-roughness model.
 */
class Material {
public:
    /**
     * @brief Default constructor - creates default white material
     */
    Material();

    // Albedo (base color)
    void set_albedo(const Vec3& albedo);
    void set_albedo_map(const std::string& path);
    const Vec3& get_albedo() const { return albedo_; }
    const std::string& get_albedo_map() const { return albedo_map_; }
    bool has_albedo_map() const { return !albedo_map_.empty(); }

    // Metallic
    void set_metallic(Real metallic);
    void set_metallic_map(const std::string& path);
    Real get_metallic() const { return metallic_; }
    const std::string& get_metallic_map() const { return metallic_map_; }
    bool has_metallic_map() const { return !metallic_map_.empty(); }

    // Roughness
    void set_roughness(Real roughness);
    void set_roughness_map(const std::string& path);
    Real get_roughness() const { return roughness_; }
    const std::string& get_roughness_map() const { return roughness_map_; }
    bool has_roughness_map() const { return !roughness_map_.empty(); }

    // Normal map
    void set_normal_map(const std::string& path);
    const std::string& get_normal_map() const { return normal_map_; }
    bool has_normal_map() const { return !normal_map_.empty(); }

    // Ambient occlusion
    void set_ao_map(const std::string& path);
    const std::string& get_ao_map() const { return ao_map_; }
    bool has_ao_map() const { return !ao_map_.empty(); }

    // Emissive
    void set_emissive(const Vec3& emissive);
    void set_emissive_map(const std::string& path);
    const Vec3& get_emissive() const { return emissive_; }
    const std::string& get_emissive_map() const { return emissive_map_; }
    bool has_emissive_map() const { return !emissive_map_.empty(); }
    bool is_emissive() const;

    // Texture handles (set by TextureManager)
    void set_albedo_texture_handle(TextureHandle handle) { albedo_tex_handle_ = handle; }
    void set_metallic_texture_handle(TextureHandle handle) { metallic_tex_handle_ = handle; }
    void set_roughness_texture_handle(TextureHandle handle) { roughness_tex_handle_ = handle; }
    void set_normal_texture_handle(TextureHandle handle) { normal_tex_handle_ = handle; }
    void set_ao_texture_handle(TextureHandle handle) { ao_tex_handle_ = handle; }
    void set_emissive_texture_handle(TextureHandle handle) { emissive_tex_handle_ = handle; }

    TextureHandle get_albedo_texture_handle() const { return albedo_tex_handle_; }
    TextureHandle get_metallic_texture_handle() const { return metallic_tex_handle_; }
    TextureHandle get_roughness_texture_handle() const { return roughness_tex_handle_; }
    TextureHandle get_normal_texture_handle() const { return normal_tex_handle_; }
    TextureHandle get_ao_texture_handle() const { return ao_tex_handle_; }
    TextureHandle get_emissive_texture_handle() const { return emissive_tex_handle_; }

private:
    // Base values
    Vec3 albedo_;///< Base color (RGB)
    Real metallic_;                       ///< Metallic factor [0, 1]
    Real roughness_;                      ///< Roughness factor [0, 1]
    Vec3 emissive_;                       ///< Emissive color (RGB)

    // Texture paths
    std::string albedo_map_;
    std::string metallic_map_;
    std::string roughness_map_;
    std::string normal_map_;
    std::string ao_map_;
    std::string emissive_map_;

    // Texture handles (GPU resources)
    TextureHandle albedo_tex_handle_;
    TextureHandle metallic_tex_handle_;
    TextureHandle roughness_tex_handle_;
    TextureHandle normal_tex_handle_;
    TextureHandle ao_tex_handle_;
    TextureHandle emissive_tex_handle_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_MATERIAL_H
```

### 文件：light.h
```cpp
/**
 * @file light.h
 * @brief Base light class and common light utilities
 */

#ifndef ARE_INCLUDE_SCENE_LIGHT_H
#define ARE_INCLUDE_SCENE_LIGHT_H

#include <are/core/types.h>

namespace are {

/**
 * @enum LightType
 * @brief Types of light sources
 */
enum class LightType {
    ARE_LIGHT_DIRECTIONAL,
    ARE_LIGHT_POINT,
    ARE_LIGHT_SPOT
};

/**
 * @struct LightData
 * @brief Packed light data for GPU transfer
 * 
 * This structure is designed to be efficiently transferred to GPU
 * via SSBO or UBO.
 */
struct LightData {
    Vec4 position_type_;                  ///< xyz: position, w: light type
    Vec4 direction_range_;                ///< xyz: direction, w: range
    Vec4 color_intensity_;                ///< xyz: color, w: intensity
    Vec4 params_;                         ///< Light-specific parameters
};

/**
 * @class Light
 * @brief Base class for all light types
 */
class Light {
public:
    /**
     * @brief Constructor
     * @param type Light type
     */
    explicit Light(LightType type);

    virtual ~Light() = default;

    // Common properties
    void set_color(const Vec3& color);
    void set_intensity(Real intensity);
    void set_cast_shadows(bool cast);

    const Vec3& get_color() const { return color_; }
    Real get_intensity() const { return intensity_; }
    bool get_cast_shadows() const { return cast_shadows_; }
    LightType get_type() const { return type_; }

    /**
     * @brief Pack light data for GPU transfer
     * @return Packed light data
     */
    virtual LightData pack() const = 0;

    /**
     * @brief Check if light affects a point
     * @param point World position
     * @return true if light can affect the point
     */
    virtual bool affects_point(const Vec3& point) const = 0;

protected:
    LightType type_;                      ///< Light type
    Vec3 color_;                          ///< Light color (RGB)
    Real intensity_;                      ///< Light intensity
    bool cast_shadows_;                   ///< Whether light casts shadows
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_LIGHT_H
```

### 文件：scene_manager.h
```cpp
/**
 * @file scene_manager.h
 * @brief Scene data management
 */

#ifndef ARE_INCLUDE_SCENE_SCENE_MANAGER_H
#define ARE_INCLUDE_SCENE_SCENE_MANAGER_H

#include <are/core/types.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <vector>
#include <memory>
#include <unordered_map>

namespace are {

/**
 * @class SceneManager
 * @brief Manages all scene objects (meshes, materials, lights)
 * 
 * Provides handle-based access to scene resources and tracks
 * scene modifications for BVH rebuilding.
 */
class SceneManager {
public:
    /**
     * @brief Constructor
     */
    SceneManager();

    /**
     * @brief Destructor
     */
    ~SceneManager();

    // Mesh management
    MeshHandle add_mesh(const Mesh& mesh);
    void remove_mesh(MeshHandle handle);
    void update_mesh(MeshHandle handle, const Mesh& mesh);
    Mesh* get_mesh(MeshHandle handle);
    const Mesh* get_mesh(MeshHandle handle) const;
    const std::vector<Mesh>& get_all_meshes() const { return meshes_; }

    // Material management
    MaterialHandle add_material(const Material& material);
    void remove_material(MaterialHandle handle);
    void update_material(MaterialHandle handle, const Material& material);
    Material* get_material(MaterialHandle handle);
    const Material* get_material(MaterialHandle handle) const;
    const std::vector<Material>& get_all_materials() const { return materials_; }

    // Light management
    LightHandle add_light(const std::shared_ptr<Light>& light);
    void remove_light(LightHandle handle);
    std::shared_ptr<Light> get_light(LightHandle handle);
    const std::vector<std::shared_ptr<Light>>& get_all_lights() const { return lights_; }

    // Scene queries
    size_t get_mesh_count() const { return meshes_.size(); }
    size_t get_material_count() const { return materials_.size(); }
    size_t get_light_count() const { return lights_.size(); }
    size_t get_total_triangle_count() const;

    // Scene state
    bool is_dirty() const { return dirty_; }
    void mark_dirty() { dirty_ = true; }
    void clear_dirty() { dirty_ = false; }

    /**
     * @brief Clear all scene data
     */
    void clear();

    /**
     * @brief Validate all handles and remove invalid entries
     */
    void compact();

private:
    std::vector<Mesh> meshes_;            ///< Mesh storage
    std::vector<Material> materials_;     ///< Material storage
    std::vector<std::shared_ptr<Light>> lights_; ///< Light storage

    std::unordered_map<MeshHandle, size_t> mesh_handle_map_;
    std::unordered_map<MaterialHandle, size_t> material_handle_map_;
    std::unordered_map<LightHandle, size_t> light_handle_map_;

    MeshHandle next_mesh_handle_;
    MaterialHandle next_material_handle_;
    LightHandle next_light_handle_;

    bool dirty_;                          ///< Scene modified flag
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SCENE_MANAGER_H
```
关于异常部分，你直接ARE_LOG_CRITICAL然后返回或者throw std::runtime_error即可；GLM已经有了自定义别名，且已经启用GLM扩展。
关于命名空间的话，就像你刚才看到的一样，代码包裹在are命名空间中。
还有需要我提供的东西吗？没有的话，你可以开始分步骤实现Phase 2。
