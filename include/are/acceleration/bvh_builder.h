/**
 * @file bvh_builder.h
 * @brief BVH construction algorithms
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H
#define ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H

#include <are/core/types.h>
#include <are/acceleration/bvh_node.h>
#include <are/geometry/triangle.h>
#include <vector>

namespace are {

/**
 * @enum BVHSplitMethod
 * @brief BVH splitting strategies
 */
enum class BVHSplitMethod {
    ARE_BVH_SPLIT_MIDDLE,                 ///< Split at midpoint
    ARE_BVH_SPLIT_SAH                     ///< Surface Area Heuristic
};

/**
 * @struct BVHBuildConfig
 * @brief Configuration for BVH construction
 */
struct BVHBuildConfig {
    BVHSplitMethod split_method_ = BVHSplitMethod::ARE_BVH_SPLIT_SAH;
    int max_leaf_size_ = 4;               ///< Maximum triangles per leaf
    int max_depth_ = 64;                  ///< Maximum tree depth
    bool use_multithreading_ = true;      ///< Use parallel construction
};

/**
 * @class BVHBuilder
 * @brief Constructs BVH from triangle list
 */
class BVHBuilder {
public:
    /**
     * @brief Constructor
     * @param config Build configuration
     */
    explicit BVHBuilder(const BVHBuildConfig& config = BVHBuildConfig());

    /**
     * @brief Build BVH from triangles
     * @param triangles Triangle list
     * @param nodes Output node list
     * @param primitive_indices Output primitive index list
     * @return Root node index
     */
    uint32_t build(const std::vector<Triangle>& triangles,
                  std::vector<BVHNode>& nodes,
                  std::vector<uint32_t>& primitive_indices);

    /**
     * @brief Get build statistics
     * @param node_count Output node count
     * @param leaf_count Output leaf count
     * @param max_depth Output maximum depth reached
     */
    void get_stats(size_t& node_count, size_t& leaf_count, int& max_depth) const;

private:
    struct BuildEntry {
        uint32_t parent_;
        uint32_t start_;
        uint32_t end_;
        int depth_;
    };

    uint32_t build_recursive(const std::vector<Triangle>& triangles,
                            std::vector<BVHNode>& nodes,
                            std::vector<uint32_t>& primitive_indices,
                            uint32_t start, uint32_t end, int depth);

    int find_best_split_axis(const std::vector<Triangle>& triangles,
                            const std::vector<uint32_t>& indices,
                            uint32_t start, uint32_t end);

    Real compute_sah_cost(const AABB& bounds, uint32_t count);

    BVHBuildConfig config_;
    size_t node_count_;
    size_t leaf_count_;
    int max_depth_reached_;
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_BUILDER_H
