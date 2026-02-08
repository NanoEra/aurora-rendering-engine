/**
 * @file bvh_node.h
 * @brief BVH node structure
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_NODE_H
#define ARE_INCLUDE_ACCELERATION_BVH_NODE_H

#include <are/core/types.h>
#include <are/geometry/aabb.h>

namespace are {

/**
 * @struct BVHNode
 * @brief Node in Bounding Volume Hierarchy
 * 
 * Uses a compact representation for efficient GPU transfer.
 */
struct BVHNode {
    AABB bounds_;                         ///< Node bounding box
    
    union {
        uint32_t left_child_;             ///< Left child index (internal node)
        uint32_t first_primitive_;        ///< First primitive index (leaf node)
    };
    
    union {
        uint32_t right_child_;            ///< Right child index (internal node)
        uint32_t primitive_count_;        ///< Number of primitives (leaf node)
    };

    /**
     * @brief Check if node is a leaf
     * @return true if leaf node
     */
    bool is_leaf() const {
        return primitive_count_ > 0;
    }

    /**
     * @brief Get node surface area (for SAH)
     * @return Surface area
     */
    Real surface_area() const {
        return bounds_.surface_area();
    }
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_NODE_H
