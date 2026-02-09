/**
 * @file geometry_cache.cpp
 * @brief Implementation of GeometryCache
 */

#include <are/renderer/geometry_cache.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>

namespace are {

static std::vector<Triangle> mesh_to_triangles(const Mesh& mesh) {
    std::vector<Triangle> tris;
    if (mesh.is_empty()) {
        return tris;
    }

    const auto& v = mesh.get_vertices();
    const auto& idx = mesh.get_indices();
    if (idx.size() % 3 != 0) {
        ARE_LOG_ERROR("GeometryCache: Mesh index count is not multiple of 3");
        return tris;
    }

    MaterialHandle material = mesh.get_material();
    tris.reserve(idx.size() / 3);

    for (size_t i = 0; i < idx.size(); i += 3) {
        uint32_t i0 = idx[i + 0];
        uint32_t i1 = idx[i + 1];
        uint32_t i2 = idx[i + 2];

        if (i0 >= v.size() || i1 >= v.size() || i2 >= v.size()) {
            continue;
        }

        tris.emplace_back(v[i0], v[i1], v[i2], material);
    }

    return tris;
}

bool GeometryCache::build_from_scene(const SceneManager& scene, const BVHBuildConfig& bvh_config) {
    ARE_PROFILE_FUNCTION();

    triangles_.clear();
    mesh_triangle_base_.clear();

    const auto& meshes = scene.get_all_meshes();
    mesh_triangle_base_.reserve(meshes.size());

    uint32_t base = 0;

    for (size_t mi = 0; mi < meshes.size(); ++mi) {
        mesh_triangle_base_.push_back(base);

        auto tris = mesh_to_triangles(meshes[mi]);
        base += static_cast<uint32_t>(tris.size());

        triangles_.insert(triangles_.end(), tris.begin(), tris.end());
    }

    if (triangles_.empty()) {
        ARE_LOG_WARN("GeometryCache: No triangles in scene");
        return false;
    }

    if (!bvh_.build(triangles_, bvh_config)) {
        ARE_LOG_ERROR("GeometryCache: BVH build failed");
        return false;
    }

    ARE_LOG_INFO("GeometryCache: Built triangles=" + std::to_string(triangles_.size()) +
                 ", meshes=" + std::to_string(meshes.size()));
    return true;
}

uint32_t GeometryCache::get_mesh_triangle_base(size_t mesh_index) const {
    if (mesh_index >= mesh_triangle_base_.size()) {
        return 0;
    }
    return mesh_triangle_base_[mesh_index];
}

} // namespace are
