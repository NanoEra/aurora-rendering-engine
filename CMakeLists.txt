# =========================================
# Basic Configuration
# =========================================

cmake_minimum_required(VERSION 3.16)

project(are LANGUAGES C CXX)

# Policy for FetchContent compatibility with older dependencies
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================
# Build Configuration & Flags
# =========================================

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Print Configuration Info
message(STATUS "")
message(STATUS "====== Build Configuration ======")
message(STATUS "Project Name:    ${PROJECT_NAME}")
message(STATUS "Build Type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:    ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler ID:     ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Compiler:        ${CMAKE_CXX_COMPILER}")
message(STATUS "================================")
message(STATUS "")

# Set compiler flags after target is defined to avoid affecting subprojects

# =========================================
# Dependencies
# =========================================

# Find required packages
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

# =========================================
# Source Files
# =========================================

# Collect source files from src/
file(GLOB_RECURSE SRC_FILES 
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Add specific library source files
list(APPEND SRC_FILES 
    "${CMAKE_SOURCE_DIR}/lib/glad/glad.c"
    "${CMAKE_SOURCE_DIR}/lib/stb/stb_image.cpp"
)

# =========================================
# Include Directories
# =========================================

# Collect all subdirectories in lib/ for include paths
file(GLOB LIB_DIRS LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/lib/*")

# Filter to keep only directories
set(LIB_INCLUDE_DIRS "")
foreach(DIR ${LIB_DIRS})
    if(IS_DIRECTORY ${DIR})
        list(APPEND LIB_INCLUDE_DIRS ${DIR})
    endif()
endforeach()

# =========================================
# Target Definition
# =========================================

# Create static library
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

# Add include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include  # Public headers
    PRIVATE
        ${LIB_INCLUDE_DIRS}          # Internal lib headers (glad, stb, etc.)
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        OpenGL::GL
        Threads::Threads
)

# Set compiler flags based on build type and compiler (target-specific)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra)
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(${PROJECT_NAME} PRIVATE -Og -Wall -g)
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /utf-8)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/O2>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/Od>)
endif()

# =========================================
# GLFW - Auto-download if not in lib/
# =========================================

set(GLFW_LIB_DIR "${CMAKE_SOURCE_DIR}/lib/glfw")
set(ARE_USE_SYSTEM_GLFW FALSE)

# Check if GLFW is in lib/ or system - prefer system first for better compatibility
if(EXISTS "${GLFW_LIB_DIR}")
    message(STATUS "Using GLFW from lib/")
    add_subdirectory(${GLFW_LIB_DIR} glfw_system)
    target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
else()
    find_package(glfw3 QUIET)
    if(glfw3_FOUND)
        message(STATUS "Using system GLFW")
        target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
        set(ARE_USE_SYSTEM_GLFW TRUE)
    else()
        include(FetchContent)
        message(STATUS "Downloading GLFW...")
        FetchContent_Declare(
            glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 3.4
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(glfw)
        target_link_libraries(${PROJECT_NAME} PUBLIC glfw)
    endif()
endif()