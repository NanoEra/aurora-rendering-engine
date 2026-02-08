/**
 * @file bvh_builder.cpp
 * @brief Implementation of BVH construction algorithms
 */

#include <are/acceleration/bvh_builder.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/utils/math_utils.h>
#include <algorithm>
#include <limits>
#include <stack>

#ifdef ARE_USE_OPENMP
#include <omp.h>
#endif

namespace are {

BVHBuilder::BVHBuilder(const BVHBuildConfig& config)
    : config_(config)
    , node_count_(0)
    , leaf_count_(0)
    , max_depth_reached_(0) {
}

uint32_t BVHBuilder::build(const std::vector<Triangle>& triangles,
                           std::vector<BVHNode>& nodes,
                           std::vector<uint32_t>& primitive_indices) {
    ARE_PROFILE_FUNCTION();
    
    if (triangles.empty()) {
        ARE_LOG_WARN("BVHBuilder: Cannot build BVH from empty triangle list");
        return 0;
    }
    
    ARE_LOG_INFO("BVHBuilder: Building BVH for " + std::to_string(triangles.size()) + " triangles");
    
    // Reset statistics
    node_count_ = 0;
    leaf_count_ = 0;
    max_depth_reached_ = 0;
    
    // Initialize primitive indices
    primitive_indices.resize(triangles.size());
    for (size_t i = 0; i < triangles.size(); ++i) {
        primitive_indices[i] = static_cast<uint32_t>(i);
    }
    
    // Reserve space for nodes (estimate: 2 * num_triangles)
    nodes.clear();
    nodes.reserve(triangles.size() * 2);
    
    // Build BVH recursively
    uint32_t root_index = build_recursive(triangles, nodes, primitive_indices, 
                                         0, static_cast<uint32_t>(triangles.size()), 0);
    
    ARE_LOG_INFO("BVHBuilder: Built BVH with " + std::to_string(node_count_) + " nodes, " +
                 std::to_string(leaf_count_) + " leaves, max depth " + 
                 std::to_string(max_depth_reached_));
    
    return root_index;
}

void BVHBuilder::get_stats(size_t& node_count, size_t& leaf_count, int& max_depth) const {
    node_count = node_count_;
    leaf_count = leaf_count_;
    max_depth = max_depth_reached_;
}

uint32_t BVHBuilder::build_recursive(const std::vector<Triangle>& triangles,
                                     std::vector<BVHNode>& nodes,
                                     std::vector<uint32_t>& primitive_indices,
                                     uint32_t start, uint32_t end, int depth) {
    ARE_PROFILE_FUNCTION();
    
    // Update statistics
    max_depth_reached_ = std::max(max_depth_reached_, depth);
    
    // Create new node
    uint32_t node_index = static_cast<uint32_t>(nodes.size());
    nodes.emplace_back();
    BVHNode& node = nodes[node_index];
    node_count_++;
    
    // Compute bounding box for all primitives in range
    node.bounds_ = AABB::invalid();
    for (uint32_t i = start; i < end; ++i) {
        uint32_t prim_idx = primitive_indices[i];
        node.bounds_.expand(triangles[prim_idx].compute_aabb());
    }
    
    uint32_t count = end - start;
    
    // Check if we should create a leaf
    bool should_create_leaf = (count <= static_cast<uint32_t>(config_.max_leaf_size_)) ||
                              (depth >= config_.max_depth_);
    
    if (should_create_leaf) {
        // Create leaf node
        node.first_primitive_ = start;
        node.primitive_count_ = count;
        leaf_count_++;
        return node_index;
    }
    
    // Find best split axis
    int split_axis = find_best_split_axis(triangles, primitive_indices, start, end);
    
    // Sort primitives along split axis
    std::sort(primitive_indices.begin() + start, 
              primitive_indices.begin() + end,
              [&](uint32_t a, uint32_t b) {
                  return triangles[a].centroid()[split_axis] < 
                         triangles[b].centroid()[split_axis];
              });
    
    // Find split position
    uint32_t mid = start + count / 2;
    
    // Use SAH if enabled
    if (config_.split_method_ == BVHSplitMethod::ARE_BVH_SPLIT_SAH) {
        Real best_cost = std::numeric_limits<Real>::max();
        uint32_t best_split = mid;
        
        // Try different split positions
        const int num_buckets = 12;
        for (int i = 1; i < num_buckets; ++i) {
            uint32_t test_split = start + (count * i) / num_buckets;
            
            // Compute bounding boxes for left and right
            AABB left_bounds = AABB::invalid();
            AABB right_bounds = AABB::invalid();
            
            for (uint32_t j = start; j < test_split; ++j) {
                left_bounds.expand(triangles[primitive_indices[j]].compute_aabb());
            }
            for (uint32_t j = test_split; j < end; ++j) {
                right_bounds.expand(triangles[primitive_indices[j]].compute_aabb());
            }
            
            // Compute SAH cost
            Real left_cost = compute_sah_cost(left_bounds, test_split - start);
            Real right_cost = compute_sah_cost(right_bounds, end - test_split);
            Real cost = left_cost + right_cost;
            
            if (cost < best_cost) {
                best_cost = cost;
                best_split = test_split;
            }
        }
        
        mid = best_split;
    }
    
    // Ensure we don't create empty children
    if (mid == start || mid == end) {
        mid = start + count / 2;
    }
    
    // Create internal node
    node.primitive_count_ = 0; // Mark as internal node
    
    // Build left and right children
    uint32_t left_child = build_recursive(triangles, nodes, primitive_indices, 
                                         start, mid, depth + 1);
    uint32_t right_child = build_recursive(triangles, nodes, primitive_indices, 
                                          mid, end, depth + 1);
    
    // Update node (it may have been reallocated)
    nodes[node_index].left_child_ = left_child;
    nodes[node_index].right_child_ = right_child;
    
    return node_index;
}

int BVHBuilder::find_best_split_axis(const std::vector<Triangle>& triangles,
                                     const std::vector<uint32_t>& indices,
                                     uint32_t start, uint32_t end) {
    ARE_PROFILE_FUNCTION();
    
    // Compute centroid bounds
    AABB centroid_bounds = AABB::invalid();
    for (uint32_t i = start; i < end; ++i) {
        centroid_bounds.expand(triangles[indices[i]].centroid());
    }
    
    // Return longest axis
    return centroid_bounds.longest_axis();
}

Real BVHBuilder::compute_sah_cost(const AABB& bounds, uint32_t count) {
    // SAH cost = surface_area * primitive_count
    // This is a simplified version; full SAH includes traversal cost
    return bounds.surface_area() * static_cast<Real>(count);
}

} // namespace are
