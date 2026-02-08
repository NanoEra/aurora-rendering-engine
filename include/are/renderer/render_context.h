/**
 * @file render_context.h
 * @brief Rendering context and state management
 */

#ifndef ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H
#define ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H

#include <are/core/types.h>
#include <are/core/config.h>

namespace are {

/**
 * @struct RenderContext
 * @brief Rendering context information
 * 
 * Contains current rendering state and frame information.
 */
struct RenderContext {
    int frame_number_;                    ///< Current frame number
    double time_;                         ///< Total elapsed time in seconds
    double delta_time_;                   ///< Time since last frame
    
    int viewport_width_;                  ///< Viewport width
    int viewport_height_;                 ///< Viewport height
    
    RayTracingBackend current_backend_;   ///< Current ray tracing backend
    bool scene_dirty_;                    ///< Scene needs BVH rebuild
    bool camera_moved_;                   ///< Camera moved this frame

    /**
     * @brief Constructor
     */
    RenderContext();

    /**
     * @brief Reset context
     */
    void reset();

    /**
     * @brief Update frame timing
     * @param current_time Current time in seconds
     */
    void update_timing(double current_time);
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDER_CONTEXT_H
