/**
 * @file are.h
 * @brief Aurora Rendering Engine - Main header file
 * 
 * This header includes all public interfaces of the Aurora Rendering Engine.
 * Users only need to include this single header to access all functionality.
 * 
 * @author Ternary_Operator
 * @version 1.0
 */

#ifndef ARE_INCLUDE_ARE_H
#define ARE_INCLUDE_ARE_H

// Core modules
#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/core/types.h>
#include <are/core/profiler.h>

// Platform modules
#include <are/platform/window.h>
#include <are/platform/gl_context.h>

// Geometry modules
#include <are/geometry/vertex.h>
#include <are/geometry/triangle.h>
#include <are/geometry/aabb.h>
#include <are/geometry/transform.h>

// Scene modules
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/light.h>
#include <are/scene/directional_light.h>
#include <are/scene/point_light.h>
#include <are/scene/spot_light.h>
#include <are/scene/scene_manager.h>

// Acceleration modules
#include <are/acceleration/bvh.h>

// Renderer modules
#include <are/renderer/renderer.h>
#include <are/renderer/render_stats.h>

// Utility modules
#include <are/utils/image_io.h>
#include <are/utils/math_utils.h>

/**
 * @namespace are
 * @brief Main namespace for Aurora Rendering Engine
 */
namespace are {

/**
 * @brief Get the version string of the engine
 * @return Version string in format "major.minor.patch"
 */
const char* get_version();

/**
 * @brief Initialize the Aurora Rendering Engine
 * @return true if initialization succeeded, false otherwise
 */
bool initialize();

/**
 * @brief Shutdown the Aurora Rendering Engine
 */
void shutdown();

} // namespace are

#endif // ARE_INCLUDE_ARE_H
