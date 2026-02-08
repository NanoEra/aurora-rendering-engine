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
	add_subdirectory(examples/01_phase2_test)
	add_subdirectory(examples/02_visual_test)
	add_subdirectory(examples/02_phase3_test)
	add_subdirectory(examples/03_phase4_test)
	add_subdirectory(examples/04_phase5_test)
    
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
