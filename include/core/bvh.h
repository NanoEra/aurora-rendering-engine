#ifndef ARE_INCLUDE_CORE_BVH_H
#define ARE_INCLUDE_CORE_BVH_H

#include "basic/types.h"
#include "resource/buffer.h"
#include "scene/mesh.h"
#include <memory>
#include <vector>

namespace are {

// Axis-aligned bounding box
struct AABB {
	Vec3 min_;
	Vec3 max_;

	// Construct AABB from min and max points
	AABB(const Vec3 &min = Vec3(0.0f), const Vec3 &max = Vec3(0.0f))
		: min_(min), max_(max) {
	}

	// Expand AABB to include point
	void expand(const Vec3 &point);

	// Expand AABB to include another AABB
	void expand(const AABB &other);

	// Get center of AABB
	Vec3 center() const {
		return (min_ + max_) * 0.5f;
	}

	// Get surface area of AABB
	float surface_area() const;

	// Check if AABB is valid
	bool is_valid() const;
};

// Triangle primitive for BVH
struct Triangle {
	Vec3 v0_, v1_, v2_;
	Vec3 n0_, n1_, n2_;
	Vec2 uv0_, uv1_, uv2_;
	Vec3 t0_, t1_, t2_; // Tangents for each vertex
	uint material_id_;

	// Get bounding box of triangle
	AABB get_bounds() const;

	// Get centroid of triangle
	Vec3 get_centroid() const;
};

// BVH node for GPU
// Internal node: left_first_ = left child index, count_ = 0 (right child = left_first_ + 1)
// Leaf node: left_first_ = triangle offset in sorted array, count_ = triangle count
struct BVHNode {
	Vec3 aabb_min_;
	uint left_first_; // Left child index (internal) or first primitive index (leaf)
	Vec3 aabb_max_;
	uint count_; // 0 for internal node, >0 for leaf (triangle count)
};

// GPU-friendly BVH node layout (std430 aligned)
struct BVHNodeGpu {
	Vec4 aabb_min_left_first_; ///< xyz = aabb min, w = left_first (uint)
	Vec4 aabb_max_count_; ///< xyz = aabb max, w = count (uint, 0 for internal)
};

// GPU-friendly triangle layout (std430 aligned)
struct TriangleGpu {
	Vec4 v0_material_; ///< xyz = v0, w = material_id (uint)
	Vec4 v1_; ///< xyz = v1, w = reserved
	Vec4 v2_; ///< xyz = v2, w = reserved
	Vec4 n0_; ///< xyz = n0, w = reserved
	Vec4 n1_; ///< xyz = n1, w = reserved
	Vec4 n2_; ///< xyz = n2, w = reserved
	Vec4 uv0_uv1_; ///< xy = uv0, zw = uv1
	Vec4 uv2_; ///< xy = uv2, zw = reserved
	Vec4 t0_; ///< xyz = t0 (tangent at v0), w = reserved
	Vec4 t1_; ///< xyz = t1 (tangent at v1), w = reserved
};

/*
 * @brief Bounding Volume Hierarchy using top-down SAH construction
 *
 * Algorithm:
 * 1. Extract triangles from meshes and transform to world space
 * 2. Sort triangles by Morton code for spatial coherence
 * 3. Build BVH top-down using SAH (Surface Area Heuristic) with 16-bin evaluation
 * 4. Node layout ensures children are at consecutive indices for GPU efficiency
 *
 * Node layout (GPU-friendly):
 * - Internal nodes: left_first_ = left child index, right = left_first_ + 1
 * - Leaf nodes: left_first_ = triangle offset, count_ = triangle count
 *
 * Time complexity: O(n log n) average with SAH binning
 * Space complexity: O(n)
 */
class BVH {
public:
	// Constructor
	BVH();

	// Destructor
	~BVH();

	/*
	 * @brief Build BVH from meshes
	 * @param meshes Mesh list
	 * @return True if build succeeded
	 */
	bool build(const std::vector<std::shared_ptr<Mesh>> &meshes);

	/*
	 * @brief Upload BVH to GPU
	 * @param node_buffer Buffer for BVH nodes
	 * @param triangle_buffer Buffer for triangles
	 * @return True if upload succeeded
	 */
	bool upload_to_gpu(Buffer &node_buffer, Buffer &triangle_buffer);

	/*
	 * @brief Get total node count
	 * @return Node count
	 */
	uint get_node_count() const {
		return static_cast<uint>(nodes_.size());
	}

	/*
	 * @brief Get total triangle count
	 * @return Triangle count
	 */
	uint get_triangle_count() const {
		return static_cast<uint>(triangles_.size());
	}

	// Clear BVH data
	void clear();

private:
	std::vector<BVHNode> nodes_;
	std::vector<Triangle> triangles_;
	std::vector<uint> triangle_indices_; // Indirection array for partitioning

	/*
	 * @brief Extract triangles from meshes and transform to world space
	 */
	void extract_triangles_(const std::vector<std::shared_ptr<Mesh>> &meshes);

	/*
	 * @brief Sort triangles by Morton code for spatial coherence
	 */
	void sort_triangles_by_morton_();

	/*
	 * @brief Recursively build BVH using SAH
	 * @param node_idx Current node index to fill
	 * @param first_prim First primitive index in triangle_indices_
	 * @param prim_count Number of primitives
	 */
	void build_recursive_(uint node_idx, uint first_prim, uint prim_count);

	/*
	 * @brief Find best split using SAH with binning
	 * @param first_prim First primitive index
	 * @param prim_count Primitive count
	 * @param axis Best split axis (output)
	 * @param split_pos Best split position (output)
	 * @return SAH cost of best split
	 */
	float find_best_split_(uint first_prim, uint prim_count, int &axis, float &split_pos);

	/*
	 * @brief Calculate node bounds
	 */
	AABB calculate_bounds_(uint first_prim, uint prim_count);

	/*
	 * @brief Calculate centroid bounds
	 */
	AABB calculate_centroid_bounds_(uint first_prim, uint prim_count);
};

} // namespace are

#endif // ARE_INCLUDE_CORE_BVH_H
