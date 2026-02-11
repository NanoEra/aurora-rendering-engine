
# =========================================
# Basic Configuration
# =========================================

cmake_minimum_required(VERSION 3.16)

project(are LANGUAGES C CXX)

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

# Set compiler flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -Wall -Wextra)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og -Wall -g)
endif()

# =========================================
# Dependencies
# =========================================

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
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
        glfw
        Threads::Threads
)
