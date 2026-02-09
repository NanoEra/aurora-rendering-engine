/**
 * @file geometry_cache.h
 * @brief Frame-level geometry cache for consistent BVH and GBuffer primitive ids
 */

#ifndef ARE_INCLUDE_RENDERER_GEOMETRY_CACHE_H
#define ARE_INCLUDE_RENDERER_GEOMETRY_CACHE_H

#include <are/core/types.h>
#include <are/geometry/triangle.h>
#include <are/acceleration/bvh.h>
#include <are/scene/scene_manager.h>
#include <vector>

namespace are {

/**
 * @class GeometryCache
 * @brief Builds a global triangle list and BVH from a SceneManager snapshot.
 *
 * Provides a single source of truth for:
 * - Global triangle array layout (used by BVH and RayTracer)
 * - Mesh -> triangle base mapping (used by Rasterizer for primitive id output)
 */
class GeometryCache {
public:
    /**
     * @brief Build cache from scene
     * @param scene Scene manager
     * @param bvh_config BVH build config
     * @return true if succeeded
     */
    bool build_from_scene(const SceneManager& scene,
                          const BVHBuildConfig& bvh_config = BVHBuildConfig());

    /**
     * @brief Get BVH reference
     * @return BVH
     */
    const BVH& get_bvh() const { return bvh_; }

    /**
     * @brief Get global triangles
     * @return Triangle array
     */
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

    /**
     * @brief Get triangle base for mesh by index into scene.get_all_meshes()
     * @param mesh_index Mesh index in scene.get_all_meshes()
     * @return Base triangle id
     */
    uint32_t get_mesh_triangle_base(size_t mesh_index) const;

private:
    std::vector<Triangle> triangles_;
    std::vector<uint32_t> mesh_triangle_base_;
    BVH bvh_;
};

} // namespace are

#endif // ARE_INCLUDE_RENDERER_GEOMETRY_CACHE_H
