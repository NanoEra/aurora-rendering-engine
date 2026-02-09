#ifndef ARE_INCLUDE_CORE_BVH_H
#define ARE_INCLUDE_CORE_BVH_H

#include "basic/types.h"
#include "scene/mesh.h"
#include "resource/buffer.h"
#include <vector>
#include <memory>

namespace are {

/// @brief Axis-aligned bounding box
struct AABB {
    Vec3 min_;
    Vec3 max_;
    
    /// @brief Construct AABB from min and max points
    AABB(const Vec3& min = Vec3(0.0f), const Vec3& max = Vec3(0.0f))
        : min_(min), max_(max) {}
    
    /// @brief Expand AABB to include point
    void expand(const Vec3& point);
    
    /// @brief Expand AABB to include another AABB
    void expand(const AABB& other);
    
    /// @brief Get center of AABB
    Vec3 center() const { return (min_ + max_) * 0.5f; }
    
    /// @brief Get surface area of AABB
    float surface_area() const;
    
    /// @brief Check if AABB is valid
    bool is_valid() const;
};

/// @brief Triangle primitive for BVH
struct Triangle {
    Vec3 v0_, v1_, v2_;
    Vec3 n0_, n1_, n2_;
    Vec2 uv0_, uv1_, uv2_;
    uint material_id_;
    
    /// @brief Get bounding box of triangle
    AABB get_bounds() const;
    
    /// @brief Get centroid of triangle
    Vec3 get_centroid() const;
};

/// @brief BVH node for GPU
struct BVHNode {
    Vec3 aabb_min_;
    uint left_first_;  // Left child index or first primitive index
    Vec3 aabb_max_;
    uint count_;       // 0 for interior node, >0 for leaf node
};

/// @brief Bounding Volume Hierarchy for ray tracing acceleration
class BVH {
public:
    /// @brief Constructor
    BVH();
    
    /// @brief Destructor
    ~BVH();
    
    /// @brief Build BVH from meshes
    /// @param meshes Mesh list
    /// @return True if build succeeded
    bool build(const std::vector<std::shared_ptr<Mesh>>& meshes);
    
    /// @brief Upload BVH to GPU
    /// @param node_buffer Buffer for BVH nodes
    /// @param triangle_buffer Buffer for triangles
    /// @return True if upload succeeded
    bool upload_to_gpu(Buffer& node_buffer, Buffer& triangle_buffer);
    
    /// @brief Get total node count
    /// @return Node count
    uint get_node_count() const { return static_cast<uint>(nodes_.size()); }
    
    /// @brief Get total triangle count
    /// @return Triangle count
    uint get_triangle_count() const { return static_cast<uint>(triangles_.size()); }
    
    /// @brief Clear BVH data
    void clear();

private:
    std::vector<BVHNode> nodes_;
    std::vector<Triangle> triangles_;
    std::vector<uint> triangle_indices_;
    
    /// @brief Recursively build BVH
    /// @param node_idx Current node index
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    void build_recursive_(uint node_idx, uint first_prim, uint prim_count);
    
    /// @brief Find best split using SAH
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @param axis Split axis (output)
    /// @param split_pos Split position (output)
    /// @return Split cost
    float find_best_split_(uint first_prim, uint prim_count, int& axis, float& split_pos);
    
    /// @brief Calculate node bounds
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @return Bounding box
    AABB calculate_bounds_(uint first_prim, uint prim_count);
    
    /// @brief Calculate centroid bounds
    /// @param first_prim First primitive index
    /// @param prim_count Primitive count
    /// @return Centroid bounding box
    AABB calculate_centroid_bounds_(uint first_prim, uint prim_count);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_BVH_H
