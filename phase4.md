很好！让我们来实现Phase 4吧！
如下是我已经实现的Phase 4头文件：
### 文件：include/are/acceleration/bvh_node.h
```cpp
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
```

### 文件：include/are/acceleration/bvh_builder.h
```cpp
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
```

### 文件：include/are/acceleration/bvh.h
```cpp
/**
 * @file bvh.h
 * @brief BVH interface and traversal
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_H
#define ARE_INCLUDE_ACCELERATION_BVH_H

#include <are/core/types.h>
#include <are/acceleration/bvh_node.h>
#include <are/acceleration/bvh_builder.h>
#include <are/geometry/triangle.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>
#include <vector>

namespace are {

/**
 * @class BVH
 * @brief Bounding Volume Hierarchy for ray tracing acceleration
 */
class BVH {
public:
    /**
     * @brief Constructor
     */
    BVH();

    /**
     * @brief Destructor
     */
    ~BVH();

    /**
     * @brief Build BVH from triangle list
     * @param triangles Triangle list
     * @param config Build configuration
     * @return true if build succeeded
     */
    bool build(const std::vector<Triangle>& triangles,
              const BVHBuildConfig& config = BVHBuildConfig());

    /**
     * @brief Traverse BVH and find closest intersection
     * @param ray Ray to trace
     * @param hit Output hit record
     * @return true if intersection found
     */
    bool intersect(const Ray& ray, HitRecord& hit) const;

    /**
     * @brief Fast occlusion test (any hit)
     * @param ray Ray to trace
     * @param t_max Maximum t value
     * @return true if any intersection found
     */
    bool intersect_any(const Ray& ray, Real t_max) const;

    /**
     * @brief Check if BVH is built
     * @return true if built
     */
    bool is_built() const { return !nodes_.empty(); }

    /**
     * @brief Get BVH nodes (for GPU upload)
     * @return Node array
     */
    const std::vector<BVHNode>& get_nodes() const { return nodes_; }

    /**
     * @brief Get primitive indices
     * @return Index array
     */
    const std::vector<uint32_t>& get_primitive_indices() const { 
        return primitive_indices_; 
    }

    /**
     * @brief Get triangles
     * @return Triangle array
     */
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

    /**
     * @brief Get memory usage in bytes
     * @return Memory usage
     */
    size_t get_memory_usage() const;

    /**
     * @brief Clear BVH data
     */
    void clear();

private:
    bool intersect_recursive(uint32_t node_index, const Ray& ray, HitRecord& hit) const;
    bool intersect_any_recursive(uint32_t node_index, const Ray& ray, Real t_max) const;

    std::vector<BVHNode> nodes_;          ///< BVH nodes
    std::vector<uint32_t> primitive_indices_; ///< Primitive index array
    std::vector<Triangle> triangles_;     ///< Triangle data
    uint32_t root_index_;                 ///< Root node index
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_H
```

如果有依赖或者需要给你的头文件或实现文件我还没有给你，欢迎提出！
