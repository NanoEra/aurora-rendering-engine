/**
 * @file bvh.h
 * @brief BVH interface and traversal
 */

#ifndef ARE_INCLUDE_ACCELERATION_BVH_H
#define ARE_INCLUDE_ACCELERATION_BVH_H

#include <are/acceleration/bvh_builder.h>
#include <are/acceleration/bvh_node.h>
#include <are/core/types.h>
#include <are/geometry/triangle.h>
#include <are/raytracer/hit_record.h>
#include <are/raytracer/ray.h>
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
	bool build(const std::vector<Triangle> &triangles,
		const BVHBuildConfig &config = BVHBuildConfig());

	/**
	 * @brief Traverse BVH and find closest intersection
	 * @param ray Ray to trace
	 * @param hit Output hit record
	 * @return true if intersection found
	 */
	bool intersect(const Ray &ray, HitRecord &hit) const;

	/**
	 * @brief Fast occlusion test (any hit)
	 * @param ray Ray to trace
	 * @param t_max Maximum t value
	 * @return true if any intersection found
	 */
	bool intersect_any(const Ray &ray, Real t_max) const;
	/**
	 * @brief Fast occlusion test (any hit), with ignored triangle id
	 * @param ray Ray to trace
	 * @param t_max Maximum t value
	 * @param ignore_triangle_index Triangle index to ignore (e.g. self primitive id)
	 * @return true if any intersection found (excluding ignored triangle)
	 */
	bool intersect_any(const Ray &ray, Real t_max, uint32_t ignore_triangle_index) const;

	/**
	 * @brief Check if BVH is built
	 * @return true if built
	 */
	bool is_built() const {
		return !nodes_.empty();
	}

	/**
	 * @brief Get BVH nodes (for GPU upload)
	 * @return Node array
	 */
	const std::vector<BVHNode> &get_nodes() const {
		return nodes_;
	}

	/**
	 * @brief Get primitive indices
	 * @return Index array
	 */
	const std::vector<uint32_t> &get_primitive_indices() const {
		return primitive_indices_;
	}

	/**
	 * @brief Get triangles
	 * @return Triangle array
	 */
	const std::vector<Triangle> &get_triangles() const {
		return triangles_;
	}

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
	// Recursive traversal (kept for reference)
	bool intersect_recursive(uint32_t node_index, const Ray &ray, HitRecord &hit) const;
	bool intersect_any_recursive(uint32_t node_index, const Ray &ray, Real t_max) const;

	// Optimized iterative traversal
	bool intersect_iterative(const Ray &ray, HitRecord &hit) const;
	bool intersect_any_iterative(const Ray &ray, Real t_max) const;

	// Fast intersection helpers
	inline bool intersect_aabb_fast(const AABB &bounds, const Ray &ray,
		const Vec3 &inv_dir, Real t_max,
		Real &t_min_out, Real &t_max_out) const;
	inline bool intersect_triangle_fast(const Triangle &triangle, const Ray &ray,
		Real t_max, HitRecord &hit) const;

	std::vector<BVHNode> nodes_; ///< BVH nodes
	std::vector<uint32_t> primitive_indices_; ///< Primitive index array
	std::vector<Triangle> triangles_; ///< Triangle data
	uint32_t root_index_; ///< Root node index
};

} // namespace are

#endif // ARE_INCLUDE_ACCELERATION_BVH_H
