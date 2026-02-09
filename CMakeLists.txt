cmake_minimum_required(VERSION 3.15)
project(AuroraRenderingEngine VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(ARE_BUILD_TESTS "Build tests" ON)
option(ARE_BUILD_EXAMPLES "Build examples" ON)

# Find dependencies
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)

# GLAD library
add_library(glad STATIC
    lib/glad/src/glad.c
)
target_include_directories(glad PUBLIC lib/glad/include)

# stb_image library
set(STB_IMAGE_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/lib/stb/stb_image.cpp)
if(NOT EXISTS ${STB_IMAGE_SOURCE})
    file(WRITE ${STB_IMAGE_SOURCE} 
"#define STB_IMAGE_IMPLEMENTATION
#include \"stb_image.h\"
")
endif()

add_library(stb_image STATIC ${STB_IMAGE_SOURCE})
target_include_directories(stb_image PUBLIC lib/stb)

# Collect all source files automatically
file(GLOB_RECURSE ARE_SOURCES 
    "src/*.cpp"
    "src/*.c"
)

# Collect all header files
file(GLOB_RECURSE ARE_HEADERS 
    "include/*.h"
    "include/*.hpp"
)

# Aurora Rendering Engine library
add_library(are STATIC ${ARE_SOURCES} ${ARE_HEADERS})

target_include_directories(are
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(are
    PUBLIC
        OpenGL::GL
        glm::glm
    PRIVATE
        glad
        stb_image
)

# Copy shaders to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Examples
if(ARE_BUILD_EXAMPLES)
    # Cornell Box example
    add_executable(cornell_box examples/cornell_box.cpp)
    target_link_libraries(cornell_box
        PRIVATE
            are
            glfw
    )
    
    # Copy shaders to example directory
    add_custom_command(TARGET cornell_box POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:cornell_box>/shaders
    )
endif()

# Tests
if(ARE_BUILD_TESTS)
    enable_testing()
    
    # Basic test
    add_executable(test_basic tests/test_basic.cpp)
    target_link_libraries(test_basic PRIVATE are)
    add_test(NAME BasicTest COMMAND test_basic)
endif()

# Install
install(TARGETS are
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY shaders/
    DESTINATION share/are/shaders
)

# Print configuration
message(STATUS "Aurora Rendering Engine Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build examples: ${ARE_BUILD_EXAMPLES}")
message(STATUS "  Build tests: ${ARE_BUILD_TESTS}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
