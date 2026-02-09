/**
 * @file bvh.cpp
 * @brief Implementation of BVH class (optimized version)
 */

#include <stack>
#include <algorithm>
#include <are/acceleration/bvh.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>

namespace are {

BVH::BVH()
	: root_index_(0) {
}

BVH::~BVH() {
	clear();
}

bool BVH::build(const std::vector<Triangle> &triangles, const BVHBuildConfig &config) {
	ARE_PROFILE_FUNCTION();

	if (triangles.empty()) {
		ARE_LOG_WARN("BVH: Cannot build from empty triangle list");
		return false;
	}

	// Clear existing data
	clear();

	// Copy triangles
	triangles_ = triangles;

	// Build BVH
	BVHBuilder builder(config);
	root_index_ = builder.build(triangles_, nodes_, primitive_indices_);

	// Get statistics
	size_t node_count, leaf_count;
	int max_depth;
	builder.get_stats(node_count, leaf_count, max_depth);

	ARE_LOG_INFO("BVH: Built successfully");
	ARE_LOG_INFO("  Triangles: " + std::to_string(triangles_.size()));
	ARE_LOG_INFO("  Nodes: " + std::to_string(node_count));
	ARE_LOG_INFO("  Leaves: " + std::to_string(leaf_count));
	ARE_LOG_INFO("  Max depth: " + std::to_string(max_depth));
	ARE_LOG_INFO("  Memory: " + std::to_string(get_memory_usage() / 1024) + " KB");

	return true;
}

bool BVH::intersect(const Ray &ray, HitRecord &hit) const {
	// Note: No profiling here - this is a hot path

	if (!is_built()) {
		return false;
	}

	// Use iterative traversal with stack for better performance
	return intersect_iterative(ray, hit);
}

bool BVH::intersect_any(const Ray &ray, Real t_max) const {
	// Note: No profiling here - this is a hot path

	if (!is_built()) {
		return false;
	}

	return intersect_any_iterative(ray, t_max);
}

bool BVH::intersect_any(const Ray& ray, Real t_max, uint32_t ignore_triangle_index) const {
    ARE_PROFILE_FUNCTION();

    if (!is_built()) {
        return false;
    }

    // iterative traversal is safer than recursion for deep trees, but keep style consistent
    std::stack<uint32_t> stack;
    stack.push(root_index_);

    while (!stack.empty()) {
        uint32_t node_index = stack.top();
        stack.pop();

        const BVHNode& node = nodes_[node_index];

        Real t0, t1;
        if (!node.bounds_.intersect_ray(ray, t0, t1)) {
            continue;
        }
        if (t0 > t_max) {
            continue;
        }

        if (node.is_leaf()) {
            for (uint32_t i = 0; i < node.primitive_count_; ++i) {
                uint32_t prim_idx = primitive_indices_[node.first_primitive_ + i];
                if (prim_idx == ignore_triangle_index) {
                    continue;
                }
                if (prim_idx >= triangles_.size()) {
                    continue;
                }
                if (triangles_[prim_idx].intersect_fast(ray, t_max)) {
                    return true;
                }
            }
        } else {
            stack.push(node.left_child_);
            stack.push(node.right_child_);
        }
    }

    return false;
}

size_t BVH::get_memory_usage() const {
	size_t total = 0;
	total += nodes_.size() * sizeof(BVHNode);
	total += primitive_indices_.size() * sizeof(uint32_t);
	total += triangles_.size() * sizeof(Triangle);
	return total;
}

void BVH::clear() {
	nodes_.clear();
	primitive_indices_.clear();
	triangles_.clear();
	root_index_ = 0;
}

bool BVH::intersect_iterative(const Ray &ray, HitRecord &hit) const {
	// Precompute inverse direction for faster AABB tests
	Vec3 inv_dir(
		1.0f / ray.direction_.x,
		1.0f / ray.direction_.y,
		1.0f / ray.direction_.z);

	// Stack-based traversal (64 levels is enough for most scenes)
	uint32_t stack[64];
	int stack_ptr = 0;
	stack[stack_ptr++] = root_index_;

	bool hit_anything = false;
	Real closest_t = ray.t_max_;

	while (stack_ptr > 0) {
		uint32_t node_index = stack[--stack_ptr];

		if (node_index >= nodes_.size()) {
			continue;
		}

		const BVHNode &node = nodes_[node_index];

		// Fast AABB test with precomputed inverse direction
		Real t_min, t_max;
		if (!intersect_aabb_fast(node.bounds_, ray, inv_dir, closest_t, t_min, t_max)) {
			continue;
		}

		if (node.is_leaf()) {
			// Test all primitives in leaf
			for (uint32_t i = 0; i < node.primitive_count_; ++i) {
				uint32_t prim_idx = primitive_indices_[node.first_primitive_ + i];

				if (prim_idx >= triangles_.size()) {
					continue;
				}

				const Triangle &triangle = triangles_[prim_idx];

				HitRecord temp_hit;
				if (intersect_triangle_fast(triangle, ray, closest_t, temp_hit)) {
					closest_t = temp_hit.t_;
					hit = temp_hit;
					hit.triangle_index_ = prim_idx;
					hit_anything = true;
				}
			}
		} else {
			// Push children to stack (far child first, so near child is processed first)
			if (node.left_child_ >= nodes_.size() || node.right_child_ >= nodes_.size()) {
				continue;
			}

			const BVHNode &left = nodes_[node.left_child_];
			const BVHNode &right = nodes_[node.right_child_];

			Real t_left_min, t_left_max;
			Real t_right_min, t_right_max;

			bool hit_left = intersect_aabb_fast(left.bounds_, ray, inv_dir, closest_t,
				t_left_min, t_left_max);
			bool hit_right = intersect_aabb_fast(right.bounds_, ray, inv_dir, closest_t,
				t_right_min, t_right_max);

			if (hit_left && hit_right) {
				// Push far child first (so near child is popped first)
				if (t_left_min < t_right_min) {
					if (stack_ptr < 64)
						stack[stack_ptr++] = node.right_child_;
					if (stack_ptr < 64)
						stack[stack_ptr++] = node.left_child_;
				} else {
					if (stack_ptr < 64)
						stack[stack_ptr++] = node.left_child_;
					if (stack_ptr < 64)
						stack[stack_ptr++] = node.right_child_;
				}
			} else if (hit_left) {
				if (stack_ptr < 64)
					stack[stack_ptr++] = node.left_child_;
			} else if (hit_right) {
				if (stack_ptr < 64)
					stack[stack_ptr++] = node.right_child_;
			}
		}
	}

	return hit_anything;
}

bool BVH::intersect_any_iterative(const Ray &ray, Real t_max) const {
	// Precompute inverse direction
	Vec3 inv_dir(
		1.0f / ray.direction_.x,
		1.0f / ray.direction_.y,
		1.0f / ray.direction_.z);

	// Stack-based traversal
	uint32_t stack[64];
	int stack_ptr = 0;
	stack[stack_ptr++] = root_index_;

	while (stack_ptr > 0) {
		uint32_t node_index = stack[--stack_ptr];

		if (node_index >= nodes_.size()) {
			continue;
		}

		const BVHNode &node = nodes_[node_index];

		// Fast AABB test
		Real t_min, t_max_box;
		if (!intersect_aabb_fast(node.bounds_, ray, inv_dir, t_max, t_min, t_max_box)) {
			continue;
		}

		if (node.is_leaf()) {
			// Test all primitives in leaf
			for (uint32_t i = 0; i < node.primitive_count_; ++i) {
				uint32_t prim_idx = primitive_indices_[node.first_primitive_ + i];

				if (prim_idx >= triangles_.size()) {
					continue;
				}

				const Triangle &triangle = triangles_[prim_idx];

				if (triangle.intersect_fast(ray, t_max)) {
					return true; // Early exit on first hit
				}
			}
		} else {
			// Push both children
			if (node.left_child_ < nodes_.size() && stack_ptr < 64) {
				stack[stack_ptr++] = node.left_child_;
			}
			if (node.right_child_ < nodes_.size() && stack_ptr < 64) {
				stack[stack_ptr++] = node.right_child_;
			}
		}
	}

	return false;
}

inline bool BVH::intersect_aabb_fast(const AABB &bounds, const Ray &ray,
	const Vec3 &inv_dir, Real t_max,
	Real &t_min_out, Real &t_max_out) const {
	// Optimized slab method with precomputed inverse direction
	Real t_min = ray.t_min_;
	Real t_max_local = t_max;

	// X axis
	{
		Real t0 = (bounds.min_.x - ray.origin_.x) * inv_dir.x;
		Real t1 = (bounds.max_.x - ray.origin_.x) * inv_dir.x;

		if (inv_dir.x < 0.0f) {
			Real temp = t0;
			t0 = t1;
			t1 = temp;
		}

		t_min = std::max(t_min, t0);
		t_max_local = std::min(t_max_local, t1);

		if (t_max_local < t_min) {
			return false;
		}
	}

	// Y axis
	{
		Real t0 = (bounds.min_.y - ray.origin_.y) * inv_dir.y;
		Real t1 = (bounds.max_.y - ray.origin_.y) * inv_dir.y;

		if (inv_dir.y < 0.0f) {
			Real temp = t0;
			t0 = t1;
			t1 = temp;
		}

		t_min = std::max(t_min, t0);
		t_max_local = std::min(t_max_local, t1);

		if (t_max_local < t_min) {
			return false;
		}
	}

	// Z axis
	{
		Real t0 = (bounds.min_.z - ray.origin_.z) * inv_dir.z;
		Real t1 = (bounds.max_.z - ray.origin_.z) * inv_dir.z;

		if (inv_dir.z < 0.0f) {
			Real temp = t0;
			t0 = t1;
			t1 = temp;
		}

		t_min = std::max(t_min, t0);
		t_max_local = std::min(t_max_local, t1);

		if (t_max_local < t_min) {
			return false;
		}
	}

	t_min_out = t_min;
	t_max_out = t_max_local;
	return true;
}

inline bool BVH::intersect_triangle_fast(const Triangle &triangle, const Ray &ray,
	Real t_max, HitRecord &hit) const {
	// Möller-Trumbore algorithm (inlined for performance)
	const Vec3 &v0 = triangle.v0_.position_;
	const Vec3 &v1 = triangle.v1_.position_;
	const Vec3 &v2 = triangle.v2_.position_;

	const Vec3 edge1 = v1 - v0;
	const Vec3 edge2 = v2 - v0;

	const Vec3 h = glm::cross(ray.direction_, edge2);
	const Real a = glm::dot(edge1, h);

	// Check if ray is parallel to triangle
	if (a > -are_epsilon && a < are_epsilon) {
		return false;
	}

	const Real f = 1.0f / a;
	const Vec3 s = ray.origin_ - v0;
	const Real u = f * glm::dot(s, h);

	if (u < 0.0f || u > 1.0f) {
		return false;
	}

	const Vec3 q = glm::cross(s, edge1);
	const Real v = f * glm::dot(ray.direction_, q);

	if (v < 0.0f || u + v > 1.0f) {
		return false;
	}

	const Real t = f * glm::dot(edge2, q);

	if (t < ray.t_min_ || t >= t_max) {
		return false;
	}

	// Fill hit record
	const Real w = 1.0f - u - v;

	hit.t_ = t;
	hit.position_ = ray.origin_ + ray.direction_ * t;
	hit.material_ = triangle.material_;

	// Interpolate vertex attributes
	hit.normal_ = glm::normalize(
		w * triangle.v0_.normal_ + u * triangle.v1_.normal_ + v * triangle.v2_.normal_);

	hit.texcoord_ = w * triangle.v0_.texcoord_ + u * triangle.v1_.texcoord_ + v * triangle.v2_.texcoord_;

	hit.tangent_ = glm::normalize(
		w * triangle.v0_.tangent_ + u * triangle.v1_.tangent_ + v * triangle.v2_.tangent_);

	// Determine front face
	hit.set_face_normal(ray.direction_, hit.normal_);

	return true;
}

// Keep recursive versions for reference/debugging
bool BVH::intersect_recursive(uint32_t node_index, const Ray &ray, HitRecord &hit) const {
	if (node_index >= nodes_.size()) {
		return false;
	}

	const BVHNode &node = nodes_[node_index];

	Real t_min, t_max;
	if (!node.bounds_.intersect_ray(ray, t_min, t_max)) {
		return false;
	}

	if (t_min > hit.t_) {
		return false;
	}

	bool hit_anything = false;

	if (node.is_leaf()) {
		for (uint32_t i = 0; i < node.primitive_count_; ++i) {
			uint32_t prim_idx = primitive_indices_[node.first_primitive_ + i];

			if (prim_idx >= triangles_.size()) {
				continue;
			}

			const Triangle &triangle = triangles_[prim_idx];
			HitRecord temp_hit;

			if (triangle.intersect(ray, temp_hit) && temp_hit.t_ < hit.t_) {
				hit = temp_hit;
				hit.triangle_index_ = prim_idx;
				hit_anything = true;
			}
		}
	} else {
		Real t_left_min, t_left_max;
		Real t_right_min, t_right_max;

		bool hit_left = nodes_[node.left_child_].bounds_.intersect_ray(ray, t_left_min, t_left_max);
		bool hit_right = nodes_[node.right_child_].bounds_.intersect_ray(ray, t_right_min, t_right_max);

		// Traverse closer child first
		if (hit_left && hit_right) {
			if (t_left_min < t_right_min) {
				hit_anything |= intersect_recursive(node.left_child_, ray, hit);
				hit_anything |= intersect_recursive(node.right_child_, ray, hit);
			} else {
				hit_anything |= intersect_recursive(node.right_child_, ray, hit);
				hit_anything |= intersect_recursive(node.left_child_, ray, hit);
			}
		} else if (hit_left) {
			hit_anything |= intersect_recursive(node.left_child_, ray, hit);
		} else if (hit_right) {
			hit_anything |= intersect_recursive(node.right_child_, ray, hit);
		}
	}

	return hit_anything;
}

bool BVH::intersect_any_recursive(uint32_t node_index, const Ray &ray, Real t_max) const {
	if (node_index >= nodes_.size()) {
		return false;
	}

	const BVHNode &node = nodes_[node_index];

	Real t_min, t_max_box;
	if (!node.bounds_.intersect_ray(ray, t_min, t_max_box)) {
		return false;
	}

	if (t_min > t_max) {
		return false;
	}

	if (node.is_leaf()) {
		for (uint32_t i = 0; i < node.primitive_count_; ++i) {
			uint32_t prim_idx = primitive_indices_[node.first_primitive_ + i];

			if (prim_idx >= triangles_.size()) {
				continue;
			}

			if (triangles_[prim_idx].intersect_fast(ray, t_max)) {
				return true;
			}
		}
		return false;
	} else {
		return intersect_any_recursive(node.left_child_, ray, t_max) || intersect_any_recursive(node.right_child_, ray, t_max);
	}
}

} // namespace are
