/**
 * @file render_stats.h
 * @brief Rendering statistics tracking
 */

#ifndef ARE_INCLUDE_RENDERER_RENDER_STATS_H
#define ARE_INCLUDE_RENDERER_RENDER_STATS_H

#include <are/core/types.h>
#include <cstdint>

namespace are {

/**
 * @struct RenderStats
 * @brief Statistics for rendering performance analysis
 */
struct RenderStats {
    // Frame timing
    double frame_time_ms_;                ///< Total frame time in milliseconds
    double rasterization_time_ms_;        ///< Rasterization time
    double ray_tracing_time_ms_;          ///< Ray tracing time
    double bvh_build_time_ms_;            ///< BVH construction time
    double present_time_ms_;              ///< Present/swap time

    // Scene statistics
    uint32_t mesh_count_;                 ///< Number of meshes
    uint32_t triangle_count_;             ///< Total triangle count
    uint32_t light_count_;                ///< Number of lights
    uint32_t material_count_;             ///< Number of materials

    // Ray tracing statistics
    uint64_t primary_rays_;               ///< Number of primary rays
    uint64_t secondary_rays_;             ///< Number of secondary rays
    uint64_t shadow_rays_;                ///< Number of shadow rays
    uint64_t bvh_traversals_;             ///< BVH traversal count
    uint64_t triangle_tests_;             ///< Triangle intersection tests

    // Memory statistics
    size_t vertex_memory_bytes_;          ///< Vertex buffer memory
    size_t index_memory_bytes_;           ///< Index buffer memory
    size_t texture_memory_bytes_;         ///< Texture memory
    size_t bvh_memory_bytes_;             ///< BVH memory

    // FPS
    double fps_;                          ///< Frames per second

    /**
     * @brief Reset all statistics to zero
     */
    void reset();

    /**
     * @brief Print statistics to console
     */
    void print() const;

    /**
     * @brief Update FPS based on frame time
     */
    void update_fps();
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_RENDER_STATS_H
