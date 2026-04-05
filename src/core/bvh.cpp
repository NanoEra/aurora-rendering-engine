#include "core/bvh.h"
#include "basic/constants.h"
#include "utils/logger.h"
#include <algorithm>
#include <limits>

namespace are {

// AABB implementation
void AABB::expand(const Vec3 &point) {
	min_ = glm::min(min_, point);
	max_ = glm::max(max_, point);
}

void AABB::expand(const AABB &other) {
	min_ = glm::min(min_, other.min_);
	max_ = glm::max(max_, other.max_);
}

float AABB::surface_area() const {
	Vec3 extent = max_ - min_;
	return 2.0f * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

bool AABB::is_valid() const {
	return min_.x <= max_.x && min_.y <= max_.y && min_.z <= max_.z;
}

// Triangle implementation
AABB Triangle::get_bounds() const {
	AABB bounds(v0_, v0_);
	bounds.expand(v1_);
	bounds.expand(v2_);
	return bounds;
}

Vec3 Triangle::get_centroid() const {
	return (v0_ + v1_ + v2_) / 3.0f;
}

// BVH implementation
BVH::BVH() {
}

BVH::~BVH() {
	clear();
}

void BVH::clear() {
	nodes_.clear();
	triangles_.clear();
	triangle_indices_.clear();
}

bool BVH::build(const std::vector<std::shared_ptr<Mesh>> &meshes) {
	clear();

	ARE_LOG_INFO("Building BVH...");

	// Step 1: Extract triangles from meshes
	extract_triangles_(meshes);

	if (triangles_.empty()) {
		ARE_LOG_WARN("No triangles to build BVH");
		return false;
	}

	// Step 2: Sort triangles by Morton code for spatial coherence
	sort_triangles_by_morton_();

	// Step 3: Initialize triangle indices (identity mapping after Morton sort)
	uint n = static_cast<uint>(triangles_.size());
	triangle_indices_.resize(n);
	for (uint i = 0; i < n; ++i) {
		triangle_indices_[i] = i;
	}

	// Step 4: Build BVH top-down using SAH
	// Reserve space: worst case 2n-1 nodes for binary tree with 1 tri per leaf
	nodes_.reserve(2 * n - 1);

	// Create root node
	nodes_.emplace_back();

	// Build recursively
	build_recursive_(0, 0, n);

	ARE_LOG_INFO("BVH built: " + std::to_string(nodes_.size()) + " nodes, " +
		std::to_string(triangles_.size()) + " triangles");

	return true;
}

void BVH::extract_triangles_(const std::vector<std::shared_ptr<Mesh>> &meshes) {
	for (const auto &mesh : meshes) {
		const auto &vertices = mesh->get_vertices();
		const auto &indices = mesh->get_indices();
		uint material_id = mesh->get_material();
		Mat4 transform = mesh->get_transform();

		for (size_t i = 0; i < indices.size(); i += 3) {
			Triangle tri;

			Vec4 v0 = transform * Vec4(vertices[indices[i]].position_, 1.0f);
			Vec4 v1 = transform * Vec4(vertices[indices[i + 1]].position_, 1.0f);
			Vec4 v2 = transform * Vec4(vertices[indices[i + 2]].position_, 1.0f);

			tri.v0_ = Vec3(v0) / v0.w;
			tri.v1_ = Vec3(v1) / v1.w;
			tri.v2_ = Vec3(v2) / v2.w;

			Mat3 normal_matrix = glm::transpose(glm::inverse(Mat3(transform)));
			tri.n0_ = glm::normalize(normal_matrix * vertices[indices[i]].normal_);
			tri.n1_ = glm::normalize(normal_matrix * vertices[indices[i + 1]].normal_);
			tri.n2_ = glm::normalize(normal_matrix * vertices[indices[i + 2]].normal_);

			tri.t0_ = glm::normalize(normal_matrix * vertices[indices[i]].tangent_);
			tri.t1_ = glm::normalize(normal_matrix * vertices[indices[i + 1]].tangent_);
			tri.t2_ = glm::normalize(normal_matrix * vertices[indices[i + 2]].tangent_);

			tri.uv0_ = vertices[indices[i]].texcoord_;
			tri.uv1_ = vertices[indices[i + 1]].texcoord_;
			tri.uv2_ = vertices[indices[i + 2]].texcoord_;

			tri.material_id_ = material_id;

			triangles_.push_back(tri);
		}
	}
}

// Morton code helper: interleave bits
static uint32_t part1by2(uint32_t x) {
	x &= 0x000003ffu;
	x = (x ^ (x << 16)) & 0xff0000ffu;
	x = (x ^ (x << 8)) & 0x0300f00fu;
	x = (x ^ (x << 4)) & 0x030c30c3u;
	x = (x ^ (x << 2)) & 0x09249249u;
	return x;
}

static uint32_t compute_morton_code(const Vec3 &p, const Vec3 &min, const Vec3 &max) {
	Vec3 scale = Vec3(1023.0f) / (max - min + Vec3(1e-6f));
	Vec3 v = (p - min) * scale;

	uint32_t ix = glm::clamp(static_cast<int>(v.x), 0, 1023);
	uint32_t iy = glm::clamp(static_cast<int>(v.y), 0, 1023);
	uint32_t iz = glm::clamp(static_cast<int>(v.z), 0, 1023);

	return (part1by2(iz) << 2) | (part1by2(iy) << 1) | part1by2(ix);
}

void BVH::sort_triangles_by_morton_() {
	if (triangles_.empty())
		return;

	// Compute scene bounds
	AABB scene_bounds;
	for (const auto &tri : triangles_) {
		scene_bounds.expand(tri.get_bounds());
	}

	// Expand bounds slightly
	Vec3 padding = (scene_bounds.max_ - scene_bounds.min_) * 0.001f;
	scene_bounds.min_ -= padding;
	scene_bounds.max_ += padding;

	// Compute Morton codes with indices
	struct MortonEntry {
		uint32_t code;
		size_t original_index;
	};

	std::vector<MortonEntry> entries;
	entries.reserve(triangles_.size());

	for (size_t i = 0; i < triangles_.size(); ++i) {
		uint32_t code = compute_morton_code(triangles_[i].get_centroid(),
			scene_bounds.min_, scene_bounds.max_);
		entries.push_back({ code, i });
	}

	// Sort by Morton code
	std::sort(entries.begin(), entries.end(),
		[](const MortonEntry &a, const MortonEntry &b) {
			return a.code < b.code;
		});

	// Reorder triangles
	std::vector<Triangle> sorted_triangles;
	sorted_triangles.reserve(triangles_.size());

	for (const auto &entry : entries) {
		sorted_triangles.push_back(triangles_[entry.original_index]);
	}

	triangles_ = std::move(sorted_triangles);
}

void BVH::build_recursive_(uint node_idx, uint first_prim, uint prim_count) {
	BVHNode &node = nodes_[node_idx];

	// Calculate bounds
	AABB bounds = calculate_bounds_(first_prim, prim_count);
	node.aabb_min_ = bounds.min_;
	node.aabb_max_ = bounds.max_;

	// Leaf node: 1 triangle per leaf for optimal GPU traversal
	if (prim_count <= 1) {
		node.left_first_ = first_prim;
		node.count_ = 1;
		return;
	}

	// Find best split using SAH
	int axis = 0;
	float split_pos = 0.0f;
	float split_cost = find_best_split_(first_prim, prim_count, axis, split_pos);

	// If SAH says no split is beneficial, force median split
	// For GPU ray tracing, deeper trees with 1 tri per leaf are preferred
	if (split_cost == std::numeric_limits<float>::max() || split_cost >= static_cast<float>(prim_count)) {
		AABB cb = calculate_centroid_bounds_(first_prim, prim_count);
		for (int a = 0; a < 3; ++a) {
			float extent = cb.max_[a] - cb.min_[a];
			if (extent > EPSILON) {
				axis = a;
				split_pos = (cb.min_[a] + cb.max_[a]) * 0.5f;
				break;
			}
		}
	}

	// Partition primitives
	uint mid = first_prim;
	for (uint i = first_prim; i < first_prim + prim_count; ++i) {
		Triangle &tri = triangles_[triangle_indices_[i]];
		float centroid = tri.get_centroid()[axis];

		if (centroid < split_pos) {
			std::swap(triangle_indices_[i], triangle_indices_[mid]);
			mid++;
		}
	}

	// Ensure split produces non-empty partitions
	if (mid == first_prim || mid == first_prim + prim_count) {
		mid = first_prim + prim_count / 2;
	}

	// Create interior node
	uint left_count = mid - first_prim;
	uint right_count = prim_count - left_count;

	// Store left child index (children will be at left_first_ and left_first_ + 1)
	node.left_first_ = static_cast<uint>(nodes_.size());
	node.count_ = 0; // Internal node

	// Create child nodes (contiguous indices)
	nodes_.emplace_back();
	nodes_.emplace_back();

	// Recursively build children
	build_recursive_(node.left_first_, first_prim, left_count);
	build_recursive_(node.left_first_ + 1, mid, right_count);
}

float BVH::find_best_split_(uint first_prim, uint prim_count, int &axis, float &split_pos) {
	float best_cost = std::numeric_limits<float>::max();
	axis = 0;
	split_pos = 0.0f;

	AABB centroid_bounds = calculate_centroid_bounds_(first_prim, prim_count);
	AABB parent_bounds = calculate_bounds_(first_prim, prim_count);
	float parent_sa = parent_bounds.surface_area();

	// Try each axis
	for (int a = 0; a < 3; ++a) {
		float extent = centroid_bounds.max_[a] - centroid_bounds.min_[a];
		if (extent < EPSILON)
			continue;

		// 16-bin SAH
		const int NUM_BINS = 16;
		for (int i = 1; i < NUM_BINS; ++i) {
			float t = static_cast<float>(i) / NUM_BINS;
			float pos = centroid_bounds.min_[a] + t * extent;

			AABB left_bounds, right_bounds;
			uint left_count = 0, right_count = 0;

			for (uint j = first_prim; j < first_prim + prim_count; ++j) {
				Triangle &tri = triangles_[triangle_indices_[j]];
				float centroid = tri.get_centroid()[a];

				if (centroid < pos) {
					left_bounds.expand(tri.get_bounds());
					left_count++;
				} else {
					right_bounds.expand(tri.get_bounds());
					right_count++;
				}
			}

			if (left_count == 0 || right_count == 0)
				continue;

			// SAH cost: C_split = C_trav + (N_left * SA_left + N_right * SA_right) / SA_parent
			float cost = 1.0f;
			if (parent_sa > 0.0f) {
				cost += (left_count * left_bounds.surface_area() +
					right_count * right_bounds.surface_area()) / parent_sa;
			}

			if (cost < best_cost) {
				best_cost = cost;
				axis = a;
				split_pos = pos;
			}
		}
	}

	return best_cost;
}

AABB BVH::calculate_bounds_(uint first_prim, uint prim_count) {
	AABB bounds { Vec3(std::numeric_limits<float>::max()),
		Vec3(std::numeric_limits<float>::lowest()) };

	for (uint i = first_prim; i < first_prim + prim_count; ++i) {
		Triangle &tri = triangles_[triangle_indices_[i]];
		bounds.expand(tri.get_bounds());
	}

	return bounds;
}

AABB BVH::calculate_centroid_bounds_(uint first_prim, uint prim_count) {
	AABB bounds { Vec3(std::numeric_limits<float>::max()),
		Vec3(std::numeric_limits<float>::lowest()) };

	for (uint i = first_prim; i < first_prim + prim_count; ++i) {
		Triangle &tri = triangles_[triangle_indices_[i]];
		bounds.expand(tri.get_centroid());
	}

	return bounds;
}

bool BVH::upload_to_gpu(Buffer &node_buffer, Buffer &triangle_buffer) {
	if (nodes_.empty() || triangles_.empty()) {
		ARE_LOG_ERROR("Cannot upload empty BVH to GPU");
		return false;
	}

	// Reorder triangles according to BVH layout
	std::vector<Triangle> ordered_triangles;
	ordered_triangles.reserve(triangles_.size());
	for (uint idx : triangle_indices_) {
		ordered_triangles.push_back(triangles_[idx]);
	}

	// Pack nodes to GPU layout
	std::vector<BVHNodeGpu> node_gpu;
	node_gpu.resize(nodes_.size());
	for (size_t i = 0; i < nodes_.size(); ++i) {
		const BVHNode &n = nodes_[i];
		BVHNodeGpu g;
		g.aabb_min_left_first_ = Vec4(n.aabb_min_, glm::uintBitsToFloat(n.left_first_));
		g.aabb_max_count_ = Vec4(n.aabb_max_, glm::uintBitsToFloat(n.count_));
		node_gpu[i] = g;
	}

	// Pack triangles to GPU layout
	std::vector<TriangleGpu> tri_gpu;
	tri_gpu.resize(ordered_triangles.size());
	for (size_t i = 0; i < ordered_triangles.size(); ++i) {
		const Triangle &t = ordered_triangles[i];

		TriangleGpu g {};
		g.v0_material_ = Vec4(t.v0_, glm::uintBitsToFloat(t.material_id_));
		g.v1_ = Vec4(t.v1_, 0.0f);
		g.v2_ = Vec4(t.v2_, 0.0f);

		g.n0_ = Vec4(t.n0_, 0.0f);
		g.n1_ = Vec4(t.n1_, 0.0f);
		g.n2_ = Vec4(t.n2_, 0.0f);

		g.uv0_uv1_ = Vec4(t.uv0_.x, t.uv0_.y, t.uv1_.x, t.uv1_.y);
		g.uv2_ = Vec4(t.uv2_.x, t.uv2_.y, 0.0f, 0.0f);

		g.t0_ = Vec4(t.t0_, 0.0f);
		g.t1_ = Vec4(t.t1_, 0.0f);

		tri_gpu[i] = g;
	}

	if (!node_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
			node_gpu.size() * sizeof(BVHNodeGpu),
			node_gpu.data(),
			BufferUsage::STATIC_DRAW)) {
		ARE_LOG_ERROR("Failed to upload BVH nodes to GPU");
		return false;
	}

	if (!triangle_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
			tri_gpu.size() * sizeof(TriangleGpu),
			tri_gpu.data(),
			BufferUsage::STATIC_DRAW)) {
		ARE_LOG_ERROR("Failed to upload BVH triangles to GPU");
		return false;
	}

	ARE_LOG_INFO("BVH uploaded to GPU successfully");
	return true;
}

} // namespace are
