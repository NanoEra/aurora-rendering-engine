#include "core/bvh.h"
#include "utils/logger.h"
#include "basic/constants.h"
#include <algorithm>
#include <limits>

namespace are {

// AABB implementation
void AABB::expand(const Vec3& point) {
    min_ = glm::min(min_, point);
    max_ = glm::max(max_, point);
}

void AABB::expand(const AABB& other) {
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

bool BVH::build(const std::vector<std::shared_ptr<Mesh>>& meshes) {
    clear();
    
    Logger::info("Building BVH...");
    
    // Extract all triangles from meshes
    for (const auto& mesh : meshes) {
        const auto& vertices = mesh->get_vertices();
        const auto& indices = mesh->get_indices();
        uint material_id = mesh->get_material();
        Mat4 transform = mesh->get_transform();
        
        for (size_t i = 0; i < indices.size(); i += 3) {
            Triangle tri;
            
            // Transform vertices
            Vec4 v0 = transform * Vec4(vertices[indices[i]].position_, 1.0f);
            Vec4 v1 = transform * Vec4(vertices[indices[i + 1]].position_, 1.0f);
            Vec4 v2 = transform * Vec4(vertices[indices[i + 2]].position_, 1.0f);
            
            tri.v0_ = Vec3(v0) / v0.w;
            tri.v1_ = Vec3(v1) / v1.w;
            tri.v2_ = Vec3(v2) / v2.w;
            
            // Transform normals
            Mat3 normal_matrix = glm::transpose(glm::inverse(Mat3(transform)));
            tri.n0_ = glm::normalize(normal_matrix * vertices[indices[i]].normal_);
            tri.n1_ = glm::normalize(normal_matrix * vertices[indices[i + 1]].normal_);
            tri.n2_ = glm::normalize(normal_matrix * vertices[indices[i + 2]].normal_);
            
            // Copy UVs
            tri.uv0_ = vertices[indices[i]].texcoord_;
            tri.uv1_ = vertices[indices[i + 1]].texcoord_;
            tri.uv2_ = vertices[indices[i + 2]].texcoord_;
            
            tri.material_id_ = material_id;
            
            triangles_.push_back(tri);
        }
    }
    
    if (triangles_.empty()) {
        Logger::warning("No triangles to build BVH");
        return false;
    }
    
    // Initialize triangle indices
    triangle_indices_.resize(triangles_.size());
    for (size_t i = 0; i < triangles_.size(); ++i) {
        triangle_indices_[i] = static_cast<uint>(i);
    }
    
    // Reserve space for nodes (estimate)
    nodes_.reserve(triangles_.size() * 2);
    
    // Create root node
    nodes_.emplace_back();
    
    // Build BVH recursively
    build_recursive_(0, 0, static_cast<uint>(triangles_.size()));
    
    Logger::info("BVH built: " + std::to_string(nodes_.size()) + " nodes, " +
                std::to_string(triangles_.size()) + " triangles");
    
    return true;
}

void BVH::build_recursive_(uint node_idx, uint first_prim, uint prim_count) {
    BVHNode& node = nodes_[node_idx];
    
    // Calculate bounds
    AABB bounds = calculate_bounds_(first_prim, prim_count);
    node.aabb_min_ = bounds.min_;
    node.aabb_max_ = bounds.max_;
    
    // Leaf node threshold
    const uint LEAF_SIZE = 4;
    
    if (prim_count <= LEAF_SIZE) {
        // Create leaf node
        node.left_first_ = first_prim;
        node.count_ = prim_count;
        return;
    }
    
    // Find best split
    int axis;
    float split_pos;
    float split_cost = find_best_split_(first_prim, prim_count, axis, split_pos);
	if(split_cost == std::numeric_limits<float>::max()) {
		node.left_first_ = first_prim;
		node.count_ = prim_count;
		return;
	}
    
    // Check if split is beneficial
    float no_split_cost = prim_count * bounds.surface_area();
    if (split_cost >= no_split_cost) {
        // Create leaf node
        node.left_first_ = first_prim;
        node.count_ = prim_count;
        return;
    }
    
    // Partition primitives
    uint mid = first_prim;
    for (uint i = first_prim; i < first_prim + prim_count; ++i) {
        Triangle& tri = triangles_[triangle_indices_[i]];
        float centroid = tri.get_centroid()[axis];
        
        if (centroid < split_pos) {
            std::swap(triangle_indices_[i], triangle_indices_[mid]);
            mid++;
        }
    }
    
    // Ensure we have primitives on both sides
    if (mid == first_prim || mid == first_prim + prim_count) {
        mid = first_prim + prim_count / 2;
    }
    
    // Create interior node
    uint left_count = mid - first_prim;
    uint right_count = prim_count - left_count;
    
    node.left_first_ = static_cast<uint>(nodes_.size());
    node.count_ = 0;
    
    // Create child nodes
    nodes_.emplace_back();
    nodes_.emplace_back();
    
    // Recursively build children
    build_recursive_(node.left_first_, first_prim, left_count);
    build_recursive_(node.left_first_ + 1, mid, right_count);
}

float BVH::find_best_split_(uint first_prim, uint prim_count, int& axis, float& split_pos) {
    float best_cost = std::numeric_limits<float>::max();
	axis = 0, split_pos = 0.0f;
    
    AABB centroid_bounds = calculate_centroid_bounds_(first_prim, prim_count);
    
    // Try each axis
    for (int a = 0; a < 3; ++a) {
        float extent = centroid_bounds.max_[a] - centroid_bounds.min_[a];
        if (extent < EPSILON) continue;
        
        // Try multiple split positions
        const int NUM_BINS = 16;
        for (int i = 1; i < NUM_BINS; ++i) {
            float t = static_cast<float>(i) / NUM_BINS;
            float pos = centroid_bounds.min_[a] + t * extent;
            
            // Count primitives and calculate bounds for each side
            AABB left_bounds, right_bounds;
            uint left_count = 0, right_count = 0;
            
            for (uint j = first_prim; j < first_prim + prim_count; ++j) {
                Triangle& tri = triangles_[triangle_indices_[j]];
                float centroid = tri.get_centroid()[a];
                
                if (centroid < pos) {
                    left_bounds.expand(tri.get_bounds());
                    left_count++;
                } else {
                    right_bounds.expand(tri.get_bounds());
                    right_count++;
                }
            }
            
            // Calculate SAH cost
            if (left_count == 0 || right_count == 0) continue;
            
            float cost = left_count * left_bounds.surface_area() +
                        right_count * right_bounds.surface_area();
            
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
    AABB bounds{Vec3(std::numeric_limits<float>::max()),
               Vec3(std::numeric_limits<float>::lowest())};
    
    for (uint i = first_prim; i < first_prim + prim_count; ++i) {
        Triangle& tri = triangles_[triangle_indices_[i]];
        bounds.expand(tri.get_bounds());
    }
    
    return bounds;
}

AABB BVH::calculate_centroid_bounds_(uint first_prim, uint prim_count) {
    AABB bounds{Vec3(std::numeric_limits<float>::max()),
               Vec3(std::numeric_limits<float>::lowest())};
    
    for (uint i = first_prim; i < first_prim + prim_count; ++i) {
        Triangle& tri = triangles_[triangle_indices_[i]];
        bounds.expand(tri.get_centroid());
    }
    
    return bounds;
}

bool BVH::upload_to_gpu(Buffer& node_buffer, Buffer& triangle_buffer) {
    if (nodes_.empty() || triangles_.empty()) {
        Logger::error("Cannot upload empty BVH to GPU");
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
        const BVHNode& n = nodes_[i];
        BVHNodeGpu g;
        g.aabb_min_left_first_ = Vec4(n.aabb_min_, glm::uintBitsToFloat(n.left_first_));
        g.aabb_max_count_      = Vec4(n.aabb_max_, glm::uintBitsToFloat(n.count_));
        node_gpu[i] = g;
    }

    // Pack triangles to GPU layout
    std::vector<TriangleGpu> tri_gpu;
    tri_gpu.resize(ordered_triangles.size());
    for (size_t i = 0; i < ordered_triangles.size(); ++i) {
        const Triangle& t = ordered_triangles[i];

        TriangleGpu g{};
        g.v0_material_ = Vec4(t.v0_, glm::uintBitsToFloat(t.material_id_));
        g.v1_          = Vec4(t.v1_, 0.0f);
        g.v2_          = Vec4(t.v2_, 0.0f);

        g.n0_          = Vec4(t.n0_, 0.0f);
        g.n1_          = Vec4(t.n1_, 0.0f);
        g.n2_          = Vec4(t.n2_, 0.0f);

        g.uv0_uv1_     = Vec4(t.uv0_.x, t.uv0_.y, t.uv1_.x, t.uv1_.y);
        g.uv2_         = Vec4(t.uv2_.x, t.uv2_.y, 0.0f, 0.0f);

        tri_gpu[i] = g;
    }

    if (!node_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
                            node_gpu.size() * sizeof(BVHNodeGpu),
                            node_gpu.data(),
                            BufferUsage::STATIC_DRAW)) {
        Logger::error("Failed to upload BVH nodes to GPU");
        return false;
    }

    if (!triangle_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
                                tri_gpu.size() * sizeof(TriangleGpu),
                                tri_gpu.data(),
                                BufferUsage::STATIC_DRAW)) {
        Logger::error("Failed to upload BVH triangles to GPU");
        return false;
    }

    Logger::info("BVH uploaded to GPU successfully");
    return true;
}

void BVH::clear() {
    nodes_.clear();
    triangles_.clear();
    triangle_indices_.clear();
}

} // namespace are
