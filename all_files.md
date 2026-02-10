### 文件：src/basic/math.cpp

```cpp
#include "basic/math.h"

namespace are {

Mat4 MathUtils::perspective(float fov, float aspect, float near, float far) {
    return glm::perspective(fov, aspect, near, far);
}

Mat4 MathUtils::look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
    return glm::lookAt(eye, center, up);
}

Vec3 MathUtils::normalize(const Vec3& v) {
    return glm::normalize(v);
}

float MathUtils::dot(const Vec3& a, const Vec3& b) {
    return glm::dot(a, b);
}

Vec3 MathUtils::cross(const Vec3& a, const Vec3& b) {
    return glm::cross(a, b);
}

Vec3 MathUtils::reflect(const Vec3& incident, const Vec3& normal) {
    return glm::reflect(incident, normal);
}

const float* MathUtils::value_ptr(const Mat4& mat) {
    return glm::value_ptr(mat);
}

} // namespace are
```

### 文件：src/core/bvh.cpp

```cpp
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
    
    // Upload nodes
    if (!node_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
                           nodes_.size() * sizeof(BVHNode),
                           nodes_.data(),
                           BufferUsage::STATIC_DRAW)) {
        Logger::error("Failed to upload BVH nodes to GPU");
        return false;
    }
    
    // Upload triangles
    if (!triangle_buffer.create(BufferType::SHADER_STORAGE_BUFFER,
                               ordered_triangles.size() * sizeof(Triangle),
                               ordered_triangles.data(),
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
```

### 文件：src/core/gbuffer.cpp

```cpp
#include "core/gbuffer.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

GBuffer::GBuffer(uint width, uint height)
    : width_(width)
    , height_(height)
    , fbo_(INVALID_HANDLE)
    , depth_texture_(INVALID_HANDLE)
    , initialized_(false) {
    for (int i = 0; i < GBUFFER_COUNT; ++i) {
        textures_[i] = INVALID_HANDLE;
    }
}

GBuffer::~GBuffer() {
    release();
}

bool GBuffer::initialize() {
    if (initialized_) {
        Logger::warning("GBuffer already initialized");
        return true;
    }
    
    // Create framebuffer
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    
    // Create G-Buffer textures
    textures_[GBUFFER_POSITION] = create_texture_(GL_RGBA32F, GL_RGBA, GL_FLOAT);
    textures_[GBUFFER_NORMAL] = create_texture_(GL_RGBA32F, GL_RGBA, GL_FLOAT);
    textures_[GBUFFER_ALBEDO] = create_texture_(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    
    // Attach textures to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_POSITION,
                          GL_TEXTURE_2D, textures_[GBUFFER_POSITION], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_NORMAL,
                          GL_TEXTURE_2D, textures_[GBUFFER_NORMAL], 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + GBUFFER_ALBEDO,
                          GL_TEXTURE_2D, textures_[GBUFFER_ALBEDO], 0);
    
    // Create depth texture
    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, width_, height_, 0,
                GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                          GL_TEXTURE_2D, depth_texture_, 0);
    
    // Set draw buffers
    GLenum draw_buffers[GBUFFER_COUNT] = {
        GL_COLOR_ATTACHMENT0 + GBUFFER_POSITION,
        GL_COLOR_ATTACHMENT0 + GBUFFER_NORMAL,
        GL_COLOR_ATTACHMENT0 + GBUFFER_ALBEDO
    };
    glDrawBuffers(GBUFFER_COUNT, draw_buffers);
    
    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        Logger::error("GBuffer framebuffer is not complete");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    initialized_ = true;
    Logger::info("GBuffer initialized successfully");
    return true;
}

void GBuffer::release() {
    if (!initialized_) return;
    
    if (fbo_ != INVALID_HANDLE) {
        glDeleteFramebuffers(1, &fbo_);
        fbo_ = INVALID_HANDLE;
    }
    
    for (int i = 0; i < GBUFFER_COUNT; ++i) {
        if (textures_[i] != INVALID_HANDLE) {
            glDeleteTextures(1, &textures_[i]);
            textures_[i] = INVALID_HANDLE;
        }
    }
    
    if (depth_texture_ != INVALID_HANDLE) {
        glDeleteTextures(1, &depth_texture_);
        depth_texture_ = INVALID_HANDLE;
    }
    
    initialized_ = false;
    Logger::info("GBuffer released");
}

void GBuffer::render(const Scene& scene, const Shader& shader) {
    if (!initialized_) {
        Logger::error("GBuffer not initialized");
        return;
    }
    
    // Bind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, width_, height_);
    
    // Clear buffers
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Use shader
    shader.use();
    
    // Set camera matrices
    const Camera& camera = scene.get_camera();
    Mat4 view = camera.get_view_matrix();
    Mat4 projection = camera.get_projection_matrix();
    
    shader.set_mat4("u_view", view);
    shader.set_mat4("u_projection", projection);
    
    // Render all meshes
    const auto& meshes = scene.get_meshes();
    const auto& materials = scene.get_materials();
    
    for (const auto& mesh : meshes) {
        if (!mesh->is_uploaded()) {
            Logger::warning("Mesh not uploaded to GPU, skipping");
            continue;
        }
        
        // Set model matrix
        Mat4 model = mesh->get_transform();
        shader.set_mat4("u_model", model);
        
        // Set material properties
        uint material_id = mesh->get_material();
        if (material_id < materials.size()) {
            const auto& material = materials[material_id];
            
            shader.set_vec3("u_albedo", material->get_albedo());
            shader.set_float("u_metallic", material->get_metallic());
            shader.set_float("u_roughness", material->get_roughness());
            shader.set_uint("u_material_id", material_id);
            
            // Bind textures
            auto albedo_tex = material->get_albedo_texture();
            if (albedo_tex && albedo_tex->is_valid()) {
                albedo_tex->bind(0);
                shader.set_int("u_albedo_map", 0);
                shader.set_int("u_has_albedo_map", 1);
            } else {
                shader.set_int("u_has_albedo_map", 0);
            }
            
            auto normal_tex = material->get_normal_texture();
            if (normal_tex && normal_tex->is_valid()) {
                normal_tex->bind(1);
                shader.set_int("u_normal_map", 1);
                shader.set_int("u_has_normal_map", 1);
            } else {
                shader.set_int("u_has_normal_map", 0);
            }
        }
        
        // Draw mesh
        glBindVertexArray(mesh->get_vao());
        glDrawElements(GL_TRIANGLES, mesh->get_indices().size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    
    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GBuffer::resize(uint width, uint height) {
    if (width == width_ && height == height_) return;
    
    width_ = width;
    height_ = height;
    
    if (initialized_) {
        release();
        initialize();
    }
}

TextureHandle GBuffer::get_texture(int index) const {
    if (index < 0 || index >= GBUFFER_COUNT) {
        Logger::error("Invalid G-Buffer texture index");
        return INVALID_HANDLE;
    }
    return textures_[index];
}

void GBuffer::get_dimensions(uint& width, uint& height) const {
    width = width_;
    height = height_;
}

TextureHandle GBuffer::create_texture_(uint internal_format, uint format, uint type) {
    TextureHandle texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width_, height_, 0, format, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return texture;
}

} // namespace are
```

### 文件：src/core/raytracer.cpp

```cpp
#include "core/raytracer.h"
#include "utils/logger.h"
#include "basic/constants.h"
#include <glad/glad.h>

namespace are {

RayTracer::RayTracer(uint width, uint height, const RayTracerConfig& config)
    : width_(width)
    , height_(height)
    , config_(config)
    , accumulation_texture_(INVALID_HANDLE)
    , scene_buffer_(INVALID_HANDLE)
    , material_buffer_(INVALID_HANDLE)
    , light_buffer_(INVALID_HANDLE)
    , bvh_(nullptr)
    , bvh_built_(false)
    , frame_count_(0)
    , initialized_(false) {
}

RayTracer::~RayTracer() {
    release();
}

bool RayTracer::initialize() {
    if (initialized_) {
        Logger::warning("RayTracer already initialized");
        return true;
    }
    
    // Create accumulation texture
    glGenTextures(1, &accumulation_texture_);
    glBindTexture(GL_TEXTURE_2D, accumulation_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Create shader storage buffers
    glGenBuffers(1, &material_buffer_);
    glGenBuffers(1, &light_buffer_);
    
    // Load compute shader
    Logger::info("Loading ray tracing compute shader in RayTracer...");
    if (!compute_shader_.load_compute("shaders/raytracing.comp")) {
        Logger::error("Failed to load ray tracing compute shader in RayTracer");
        return false;
    }
    Logger::info("Ray tracing compute shader loaded in RayTracer");
    
    // Initialize BVH if enabled
    if (config_.use_bvh_) {
        bvh_ = std::make_unique<BVH>();
    }
    
    initialized_ = true;
    Logger::info("RayTracer initialized successfully");
    return true;
}

void RayTracer::release() {
    if (!initialized_) return;
    
    if (accumulation_texture_ != INVALID_HANDLE) {
        glDeleteTextures(1, &accumulation_texture_);
        accumulation_texture_ = INVALID_HANDLE;
    }
    
    if (material_buffer_ != INVALID_HANDLE) {
        glDeleteBuffers(1, &material_buffer_);
        material_buffer_ = INVALID_HANDLE;
    }
    
    if (light_buffer_ != INVALID_HANDLE) {
        glDeleteBuffers(1, &light_buffer_);
        light_buffer_ = INVALID_HANDLE;
    }
    
    bvh_node_buffer_.release();
    bvh_triangle_buffer_.release();
    
    compute_shader_.release();
    
    bvh_.reset();
    bvh_built_ = false;
    
    initialized_ = false;
    Logger::info("RayTracer released");
}

bool RayTracer::rebuild_bvh(const Scene& scene) {
    if (!config_.use_bvh_) {
        Logger::warning("BVH is disabled in configuration");
        return false;
    }
    
    if (!bvh_) {
        bvh_ = std::make_unique<BVH>();
    }
    
    Logger::info("Building BVH for ray tracing...");
    
    if (!bvh_->build(scene.get_meshes())) {
        Logger::error("Failed to build BVH");
        return false;
    }
    
    if (!bvh_->upload_to_gpu(bvh_node_buffer_, bvh_triangle_buffer_)) {
        Logger::error("Failed to upload BVH to GPU");
        return false;
    }
    
    bvh_built_ = true;
    Logger::info("BVH built and uploaded successfully");
    return true;
}

void RayTracer::trace(const Scene& scene, const GBuffer& gbuffer, TextureHandle output_texture) {
    if (!initialized_) {
        Logger::error("RayTracer not initialized");
        return;
    }
    
    if (!compute_shader_.is_valid()) {
        Logger::error("Ray tracing compute shader not loaded");
        return;
    }
    
    // Build BVH if enabled and not built yet
    if (config_.use_bvh_ && !bvh_built_) {
        rebuild_bvh(scene);
    }
    
    // Upload scene data
    upload_scene_data_(scene);
    
    // Use compute shader
    compute_shader_.use();
    
    // Bind G-Buffer textures
    bind_gbuffer_(gbuffer);
    
    // Bind output and accumulation textures
    glBindImageTexture(3, output_texture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(4, accumulation_texture_, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    
    // Bind BVH buffers if enabled
    if (config_.use_bvh_ && bvh_built_) {
        bvh_node_buffer_.bind_base(2);
        bvh_triangle_buffer_.bind_base(3);
        compute_shader_.set_bool("u_use_bvh", true);
        compute_shader_.set_uint("u_bvh_node_count", bvh_->get_node_count());
    } else {
        compute_shader_.set_bool("u_use_bvh", false);
    }
    
    // Set uniforms
    compute_shader_.set_uint("u_frame_count", frame_count_);
    compute_shader_.set_uint("u_samples_per_pixel", config_.samples_per_pixel_);
    compute_shader_.set_uint("u_max_depth", config_.max_depth_);
    compute_shader_.set_uint("u_light_count", static_cast<uint>(scene.get_lights().size()));
    compute_shader_.set_bool("u_enable_accumulation", config_.enable_accumulation_);
    
    // Set camera data
    const Camera& camera = scene.get_camera();
    compute_shader_.set_vec3("u_camera_position", camera.get_position());
    
    Mat4 inv_vp = glm::inverse(camera.get_view_projection_matrix());
    compute_shader_.set_mat4("u_inv_view_projection", inv_vp);
    
    // Dispatch compute shader
    uint num_groups_x = (width_ + COMPUTE_GROUP_SIZE_X - 1) / COMPUTE_GROUP_SIZE_X;
    uint num_groups_y = (height_ + COMPUTE_GROUP_SIZE_Y - 1) / COMPUTE_GROUP_SIZE_Y;
    
    glDispatchCompute(num_groups_x, num_groups_y, 1);
    
    // Memory barrier
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    
    // Increment frame count for accumulation
    if (config_.enable_accumulation_) {
        frame_count_++;
    }
}

void RayTracer::resize(uint width, uint height) {
    if (width == width_ && height == height_) return;
    
    width_ = width;
    height_ = height;
    
    if (initialized_) {
        // Recreate accumulation texture
        if (accumulation_texture_ != INVALID_HANDLE) {
            glDeleteTextures(1, &accumulation_texture_);
        }
        
        glGenTextures(1, &accumulation_texture_);
        glBindTexture(GL_TEXTURE_2D, accumulation_texture_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width_, height_, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
        reset_accumulation();
    }
}

void RayTracer::reset_accumulation() {
    frame_count_ = 0;
}

void RayTracer::set_config(const RayTracerConfig& config) {
    bool bvh_changed = (config.use_bvh_ != config_.use_bvh_);
    
    config_ = config;
    reset_accumulation();
    
    if (bvh_changed) {
        if (config_.use_bvh_ && !bvh_) {
            bvh_ = std::make_unique<BVH>();
            bvh_built_ = false;
        } else if (!config_.use_bvh_) {
            bvh_.reset();
            bvh_built_ = false;
        }
    }
}

void RayTracer::upload_scene_data_(const Scene& scene) {
    // Upload materials
    const auto& materials = scene.get_materials();
    if (!materials.empty()) {
        struct MaterialData {
            Vec3 albedo;
            float metallic;
            Vec3 emission;
            float roughness;
            int type;
            float ior;
            Vec2 padding;
        };
        
        std::vector<MaterialData> material_data;
        material_data.reserve(materials.size());
        
        for (const auto& mat : materials) {
            MaterialData data;
            data.albedo = mat->get_albedo();
            data.metallic = mat->get_metallic();
            data.emission = mat->get_emission();
            data.roughness = mat->get_roughness();
            data.type = static_cast<int>(mat->get_type());
            data.ior = mat->get_ior();
            material_data.push_back(data);
        }
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, material_buffer_);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 
                    material_data.size() * sizeof(MaterialData),
                    material_data.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);
    }
    
    // Upload lights
    const auto& lights = scene.get_lights();
    if (!lights.empty()) {
        struct LightData {
            Vec3 position;
            int type;
            Vec3 direction;
            float intensity;
            Vec3 color;
            float range;
            Vec2 spot_angles;
            Vec2 padding;
        };
        
        std::vector<LightData> light_data;
        light_data.reserve(lights.size());
        
        for (const auto& light : lights) {
            LightData data;
            data.position = light->get_position();
            data.type = static_cast<int>(light->get_type());
            data.direction = light->get_direction();
            data.intensity = light->get_intensity();
            data.color = light->get_color();
            data.range = light->get_range();
            data.spot_angles = Vec2(light->get_inner_angle(), light->get_outer_angle());
            light_data.push_back(data);
        }
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, light_buffer_);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                    light_data.size() * sizeof(LightData),
                    light_data.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);
    }
}

void RayTracer::bind_gbuffer_(const GBuffer& gbuffer) {
    glBindImageTexture(0, gbuffer.get_texture(GBUFFER_POSITION), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(1, gbuffer.get_texture(GBUFFER_NORMAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(2, gbuffer.get_texture(GBUFFER_ALBEDO), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA8);
}

void RayTracer::set_compute_shader(const Shader& shader) {
    compute_shader_ = shader;
    Logger::info("Compute shader set for RayTracer");
}

} // namespace are
```

### 文件：src/core/renderer.cpp

```cpp
#include "core/renderer.h"
#include "utils/logger.h"
#include <chrono>
#include <glad/glad.h>

namespace are {

Renderer::Renderer(const RendererConfig &config)
	: config_(config)
	, initialized_(false)
	, frame_count_(0) {
}

Renderer::~Renderer() {
	shutdown();
}

bool Renderer::initialize() {
    if (initialized_) {
        Logger::warning("Renderer already initialized");
        return true;
    }
    
    Logger::info("Initializing Aurora Rendering Engine...");
    
    // Initialize shader manager
    shader_manager_ = std::make_unique<ShaderManager>();
    if (!shader_manager_->initialize()) {
        Logger::error("Failed to initialize shader manager");
        return false;
    }
    
    // Initialize G-Buffer
    gbuffer_ = std::make_unique<GBuffer>(config_.width_, config_.height_);
    if (!gbuffer_->initialize()) {
        Logger::error("Failed to initialize G-Buffer");
        return false;
    }
    
    // Initialize ray tracer
    RayTracerConfig rt_config;
    rt_config.samples_per_pixel_ = config_.samples_per_pixel_;
    rt_config.max_depth_ = config_.max_ray_depth_;
    rt_config.enable_shadows_ = true;
    rt_config.enable_reflections_ = true;
    rt_config.enable_accumulation_ = config_.enable_accumulation_;
    rt_config.use_bvh_ = true;
    
    raytracer_ = std::make_unique<RayTracer>(config_.width_, config_.height_, rt_config);
    if (!raytracer_->initialize()) {
        Logger::error("Failed to initialize ray tracer");
        return false;
    }
    
    // Pass compute shader to ray tracer
    const Shader& rt_shader = shader_manager_->get_raytracing_shader();
    if (!rt_shader.is_valid()) {
        Logger::error("Ray tracing shader is invalid");
        return false;
    }
    raytracer_->set_compute_shader(rt_shader);
    
    // Initialize screen blit
    screen_blit_ = std::make_unique<ScreenBlit>();
    if (!screen_blit_->initialize()) {
        Logger::error("Failed to initialize screen blit");
        return false;
    }
    
    initialized_ = true;
    Logger::info("Aurora Rendering Engine initialized successfully");
    return true;
}

void Renderer::shutdown() {
	if (!initialized_)
		return;

	Logger::info("Shutting down Aurora Rendering Engine...");

	if (screen_blit_) {
        screen_blit_->release();
        screen_blit_.reset();
    }

	if (raytracer_) {
		raytracer_->release();
		raytracer_.reset();
	}

	if (gbuffer_) {
		gbuffer_->release();
		gbuffer_.reset();
	}

	if (shader_manager_) {
		shader_manager_->release();
		shader_manager_.reset();
	}

	initialized_ = false;
	Logger::info("Aurora Rendering Engine shut down");
}

RenderStats Renderer::render(const Scene& scene, TextureHandle output_texture) {
    RenderStats stats = {};
    
    if (!initialized_) {
        Logger::error("Renderer not initialized");
        return stats;
    }
    
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Phase 1: G-Buffer pass
    auto gbuffer_start = std::chrono::high_resolution_clock::now();
    
    const Shader& gbuffer_shader = shader_manager_->get_gbuffer_shader();
    gbuffer_->render(scene, gbuffer_shader);
    
    auto gbuffer_end = std::chrono::high_resolution_clock::now();
    stats.gbuffer_time_ms_ = std::chrono::duration<float, std::milli>(gbuffer_end - gbuffer_start).count();
    
    // Phase 2: Ray tracing pass
    auto raytrace_start = std::chrono::high_resolution_clock::now();
    
    // Create output texture if not provided
    TextureHandle rt_output = output_texture;
    bool created_temp_texture = false;
    
    if (rt_output == 0) {
        glGenTextures(1, &rt_output);
        glBindTexture(GL_TEXTURE_2D, rt_output);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, config_.width_, config_.height_, 
                    0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        created_temp_texture = true;
    }
    
    raytracer_->trace(scene, *gbuffer_, rt_output);
    
    auto raytrace_end = std::chrono::high_resolution_clock::now();
    stats.raytrace_time_ms_ = std::chrono::duration<float, std::milli>(raytrace_end - raytrace_start).count();
    
    // Phase 3: Blit to screen if output is default framebuffer
    if (created_temp_texture && output_texture == 0) {
        screen_blit_->blit_fullscreen(rt_output);
        glDeleteTextures(1, &rt_output);
    }
    
    // Calculate total frame time
    auto end_time = std::chrono::high_resolution_clock::now();
    stats.frame_time_ms_ = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    
    // Count triangles
    const auto& meshes = scene.get_meshes();
    for (const auto& mesh : meshes) {
        stats.triangle_count_ += mesh->get_indices().size() / 3;
    }
    
    // Estimate ray count (very rough)
    stats.ray_count_ = config_.width_ * config_.height_ * config_.samples_per_pixel_ * config_.max_ray_depth_;
    
    frame_count_++;
    
    return stats;
}

void Renderer::resize(uint width, uint height) {
	if (width == config_.width_ && height == config_.height_)
		return;

	config_.width_ = width;
	config_.height_ = height;

	if (initialized_) {
		gbuffer_->resize(width, height);
		raytracer_->resize(width, height);

		Logger::info("Renderer resized to " + std::to_string(width) + "x" + std::to_string(height));
	}
}

void Renderer::set_config(const RendererConfig &config) {
	bool size_changed = (config.width_ != config_.width_ || config.height_ != config_.height_);

	config_ = config;

	if (initialized_) {
		if (size_changed) {
			resize(config_.width_, config_.height_);
		}

		// Update ray tracer config
		RayTracerConfig rt_config = raytracer_->get_config();
		rt_config.samples_per_pixel_ = config_.samples_per_pixel_;
		rt_config.max_depth_ = config_.max_ray_depth_;
		rt_config.enable_accumulation_ = config_.enable_accumulation_;
		raytracer_->set_config(rt_config);
	}
}

} // namespace are
```

### 文件：src/core/screen_blit.cpp

```cpp
#include "core/screen_blit.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

namespace {
    const char* VERTEX_SHADER_SOURCE = R"(
        #version 430 core
        layout(location = 0) in vec2 a_position;
        layout(location = 1) in vec2 a_texcoord;
        
        out vec2 v_texcoord;
        
        void main() {
            v_texcoord = a_texcoord;
            gl_Position = vec4(a_position, 0.0, 1.0);
        }
    )";
    
    const char* FRAGMENT_SHADER_SOURCE = R"(
        #version 430 core
        in vec2 v_texcoord;
        out vec4 frag_color;
        
        uniform sampler2D u_texture;
        
        void main() {
            frag_color = texture(u_texture, v_texcoord);
        }
    )";
}

ScreenBlit::ScreenBlit()
    : vao_(0)
    , vbo_(0)
    , initialized_(false) {
}

ScreenBlit::~ScreenBlit() {
    release();
}

bool ScreenBlit::initialize() {
    if (initialized_) {
        Logger::warning("ScreenBlit already initialized");
        return true;
    }
    
    // Compile shader
    if (!shader_.compile(VERTEX_SHADER_SOURCE, FRAGMENT_SHADER_SOURCE)) {
        Logger::error("Failed to compile screen blit shader");
        return false;
    }
    
    // Create fullscreen quad
    create_quad_();
    
    initialized_ = true;
    Logger::info("ScreenBlit initialized successfully");
    return true;
}

void ScreenBlit::release() {
    if (!initialized_) return;
    
    shader_.release();
    
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    
    initialized_ = false;
}

void ScreenBlit::blit(TextureHandle texture, int x, int y, uint width, uint height) {
    if (!initialized_) {
        Logger::error("ScreenBlit not initialized");
        return;
    }
    
    // Set viewport
    glViewport(x, y, width, height);
    
    // Disable depth test
    glDisable(GL_DEPTH_TEST);
    
    // Use shader
    shader_.use();
    shader_.set_int("u_texture", 0);
    
    // Bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    
    // Draw quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    // Re-enable depth test
    glEnable(GL_DEPTH_TEST);
}

void ScreenBlit::blit_fullscreen(TextureHandle texture) {
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    blit(texture, viewport[0], viewport[1], viewport[2], viewport[3]);
}

void ScreenBlit::create_quad_() {
    // Fullscreen quad vertices (position + texcoord)
    float vertices[] = {
        // Position    // TexCoord
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        -1.0f,  1.0f,  0.0f, 1.0f
    };
    
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    
    // TexCoord attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    
    glBindVertexArray(0);
}

} // namespace are
```

### 文件：src/core/shader_manager.cpp

```cpp
#include "core/shader_manager.h"
#include "utils/logger.h"

namespace are {

ShaderManager::ShaderManager()
    : initialized_(false) {
}

ShaderManager::~ShaderManager() {
    release();
}

bool ShaderManager::initialize() {
    if (initialized_) {
        Logger::warning("ShaderManager already initialized");
        return true;
    }
    
    Logger::info("Loading built-in shaders...");
    
    if (!load_builtin_shaders_()) {
        Logger::error("Failed to load built-in shaders");
        return false;
    }
    
    initialized_ = true;
    Logger::info("ShaderManager initialized successfully");
    return true;
}

void ShaderManager::release() {
    if (!initialized_) return;
    
    gbuffer_shader_.release();
    raytracing_shader_.release();
    
    for (auto& pair : shader_cache_) {
        pair.second.release();
    }
    shader_cache_.clear();
    
    initialized_ = false;
    Logger::info("ShaderManager released");
}

Shader ShaderManager::load_shader(const std::string& name,
                                  const std::string& vertex_path,
                                  const std::string& fragment_path) {
    // Check cache
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Shader '" + name + "' loaded from cache");
        return it->second;
    }
    
    // Load shader
    Shader shader;
    if (!shader.load(vertex_path, fragment_path)) {
        Logger::error("Failed to load shader '" + name + "'");
        return Shader();
    }
    
    shader_cache_[name] = shader;
    Logger::info("Shader '" + name + "' loaded successfully");
    return shader;
}

Shader ShaderManager::load_compute_shader(const std::string& name,
                                         const std::string& compute_path) {
    // Check cache
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        Logger::info("Compute shader '" + name + "' loaded from cache");
        return it->second;
    }
    
    // Load shader
    Shader shader;
    if (!shader.load_compute(compute_path)) {
        Logger::error("Failed to load compute shader '" + name + "'");
        return Shader();
    }
    
    shader_cache_[name] = shader;
    Logger::info("Compute shader '" + name + "' loaded successfully");
    return shader;
}

Shader ShaderManager::get_shader(const std::string& name) const {
    auto it = shader_cache_.find(name);
    if (it != shader_cache_.end()) {
        return it->second;
    }
    
    Logger::warning("Shader '" + name + "' not found in cache");
    return Shader();
}

bool ShaderManager::load_builtin_shaders_() {
    // Load G-Buffer shader
    if (!gbuffer_shader_.load("shaders/gbuffer.vert", "shaders/gbuffer.frag")) {
        Logger::error("Failed to load G-Buffer shader");
        return false;
    }
    shader_cache_["gbuffer"] = gbuffer_shader_;
    
    // Load ray tracing compute shader
    if (!raytracing_shader_.load_compute("shaders/raytracing.comp")) {
        Logger::error("Failed to load ray tracing shader");
        return false;
    }
    shader_cache_["raytracing"] = raytracing_shader_;

	// Load ray tracing compute shader
    Logger::info("Loading ray tracing compute shader...");
    if (!raytracing_shader_.load_compute("shaders/raytracing.comp")) {
        Logger::error("Failed to load ray tracing shader");
        return false;
    }
    shader_cache_["raytracing"] = raytracing_shader_;
    Logger::info("Ray tracing shader loaded successfully");
    
    return true;
}

} // namespace are
```

### 文件：src/scene/camera.cpp

```cpp
#include "scene/camera.h"
#include "basic/math.h"
#include <glm/gtc/matrix_transform.hpp>

namespace are {

Camera::Camera()
    : position_(0.0f, 0.0f, 5.0f)
    , target_(0.0f, 0.0f, 0.0f)
    , up_(0.0f, 1.0f, 0.0f)
    , projection_type_(ProjectionType::PERSPECTIVE)
    , fov_(glm::radians(45.0f))
    , aspect_(16.0f / 9.0f)
    , left_(-1.0f)
    , right_(1.0f)
    , bottom_(-1.0f)
    , top_(1.0f)
    , near_(0.1f)
    , far_(100.0f)
    , view_dirty_(true)
    , projection_dirty_(true) {
}

Camera::~Camera() {
}

void Camera::set_perspective(float fov, float aspect, float near, float far) {
    projection_type_ = ProjectionType::PERSPECTIVE;
    fov_ = glm::radians(fov);
    aspect_ = aspect;
    near_ = near;
    far_ = far;
    projection_dirty_ = true;
}

void Camera::set_orthographic(float left, float right, float bottom, float top, float near, float far) {
    projection_type_ = ProjectionType::ORTHOGRAPHIC;
    left_ = left;
    right_ = right;
    bottom_ = bottom;
    top_ = top;
    near_ = near;
    far_ = far;
    projection_dirty_ = true;
}

void Camera::set_position(const Vec3& position) {
    position_ = position;
    view_dirty_ = true;
}

void Camera::set_target(const Vec3& target) {
    target_ = target;
    view_dirty_ = true;
}

void Camera::set_up(const Vec3& up) {
    up_ = up;
    view_dirty_ = true;
}

Mat4 Camera::get_view_matrix() const {
    if (view_dirty_) {
        view_matrix_ = MathUtils::look_at(position_, target_, up_);
        view_dirty_ = false;
    }
    return view_matrix_;
}

Mat4 Camera::get_projection_matrix() const {
    if (projection_dirty_) {
        if (projection_type_ == ProjectionType::PERSPECTIVE) {
            projection_matrix_ = MathUtils::perspective(fov_, aspect_, near_, far_);
        } else {
            projection_matrix_ = glm::ortho(left_, right_, bottom_, top_, near_, far_);
        }
        projection_dirty_ = false;
    }
    return projection_matrix_;
}

Mat4 Camera::get_view_projection_matrix() const {
    return get_projection_matrix() * get_view_matrix();
}

Vec3 Camera::get_forward() const {
    return MathUtils::normalize(target_ - position_);
}

Vec3 Camera::get_right() const {
    Vec3 forward = get_forward();
    return MathUtils::normalize(MathUtils::cross(forward, up_));
}

Vec3 Camera::get_up() const {
    Vec3 forward = get_forward();
    Vec3 right = get_right();
    return MathUtils::cross(right, forward);
}

} // namespace are
```

### 文件：src/scene/material.cpp

```cpp
#include "scene/material.h"

namespace are {

Material::Material()
    : albedo_(1.0f, 1.0f, 1.0f)
    , emission_(0.0f, 0.0f, 0.0f)
    , metallic_(0.0f)
    , roughness_(0.5f)
    , ior_(1.5f)
    , type_(MaterialType::DIFFUSE)
    , albedo_texture_(nullptr)
    , normal_texture_(nullptr) {
}

Material::~Material() {
}

void Material::set_albedo(const Vec3& albedo) {
    albedo_ = albedo;
}

void Material::set_emission(const Vec3& emission) {
    emission_ = emission;
}

void Material::set_metallic(float metallic) {
    metallic_ = glm::clamp(metallic, 0.0f, 1.0f);
}

void Material::set_roughness(float roughness) {
    roughness_ = glm::clamp(roughness, 0.0f, 1.0f);
}

void Material::set_ior(float ior) {
    ior_ = ior;
}

void Material::set_type(MaterialType type) {
    type_ = type;
}

void Material::set_albedo_texture(std::shared_ptr<Texture> texture) {
    albedo_texture_ = texture;
}

void Material::set_normal_texture(std::shared_ptr<Texture> texture) {
    normal_texture_ = texture;
}

} // namespace are
```

### 文件：src/scene/light.cpp

```cpp
#include "scene/light.h"
#include <glm/gtc/constants.hpp>

namespace are {

Light::Light()
    : type_(LightType::POINT)
    , position_(0.0f, 5.0f, 0.0f)
    , direction_(0.0f, -1.0f, 0.0f)
    , color_(1.0f, 1.0f, 1.0f)
    , intensity_(1.0f)
    , range_(10.0f)
    , inner_angle_(glm::radians(30.0f))
    , outer_angle_(glm::radians(45.0f)) {
}

Light::~Light() {
}

void Light::set_type(LightType type) {
    type_ = type;
}

void Light::set_position(const Vec3& position) {
    position_ = position;
}

void Light::set_direction(const Vec3& direction) {
    direction_ = glm::normalize(direction);
}

void Light::set_color(const Vec3& color) {
    color_ = color;
}

void Light::set_intensity(float intensity) {
    intensity_ = intensity;
}

void Light::set_range(float range) {
    range_ = range;
}

void Light::set_spot_angles(float inner_angle, float outer_angle) {
    inner_angle_ = glm::radians(inner_angle);
    outer_angle_ = glm::radians(outer_angle);
}

} // namespace are
```

### 文件：src/scene/scene.cpp

```cpp
#include "scene/scene.h"

namespace are {

Scene::Scene() {
    // Create default camera
    camera_ = std::make_shared<Camera>();
}

Scene::~Scene() {
    clear();
}

uint Scene::add_mesh(std::shared_ptr<Mesh> mesh) {
    meshes_.push_back(mesh);
    return static_cast<uint>(meshes_.size() - 1);
}

uint Scene::add_material(std::shared_ptr<Material> material) {
    materials_.push_back(material);
    return static_cast<uint>(materials_.size() - 1);
}

uint Scene::add_light(std::shared_ptr<Light> light) {
    lights_.push_back(light);
    return static_cast<uint>(lights_.size() - 1);
}

void Scene::set_camera(std::shared_ptr<Camera> camera) {
    camera_ = camera;
}

void Scene::clear() {
    meshes_.clear();
    materials_.clear();
    lights_.clear();
}

void Scene::update(float delta_time) {
    // Reserved for future animation/physics updates
    (void)delta_time; // Suppress unused parameter warning
}

} // namespace are
```

### 文件：src/scene/mesh.cpp

```cpp
#include "scene/mesh.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

Mesh::Mesh()
    : material_id_(0)
    , transform_(1.0f)
    , vao_(0)
    , vbo_(0)
    , ebo_(0)
    , uploaded_(false) {
}

Mesh::~Mesh() {
    release_gpu_resources();
}

void Mesh::set_vertices(const std::vector<Vertex>& vertices) {
    vertices_ = vertices;
    uploaded_ = false;
}

void Mesh::set_indices(const std::vector<uint>& indices) {
    indices_ = indices;
    uploaded_ = false;
}

void Mesh::set_material(uint material_id) {
    material_id_ = material_id;
}

void Mesh::set_transform(const Mat4& transform) {
    transform_ = transform;
}

bool Mesh::upload_to_gpu() {
    if (uploaded_) {
        Logger::warning("Mesh already uploaded to GPU");
        return true;
    }
    
    if (vertices_.empty()) {
        Logger::error("Cannot upload mesh: no vertices");
        return false;
    }
    
    if (indices_.empty()) {
        Logger::error("Cannot upload mesh: no indices");
        return false;
    }
    
    // Generate VAO
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    
    // Generate and upload VBO
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), 
                vertices_.data(), GL_STATIC_DRAW);
    
    // Generate and upload EBO
    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint),
                indices_.data(), GL_STATIC_DRAW);
    
    // Set vertex attributes
    // Location 0: Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, position_));
    
    // Location 1: Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, normal_));
    
    // Location 2: TexCoord
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, texcoord_));
    
    // Location 3: Tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                         (void*)offsetof(Vertex, tangent_));
    
    glBindVertexArray(0);
    
    uploaded_ = true;
    Logger::info("Mesh uploaded to GPU successfully");
    return true;
}

void Mesh::release_gpu_resources() {
    if (!uploaded_) return;
    
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    
    if (ebo_ != 0) {
        glDeleteBuffers(1, &ebo_);
        ebo_ = 0;
    }
    
    uploaded_ = false;
}

} // namespace are
```

### 文件：src/resource/buffer.cpp

```cpp
#include "resource/buffer.h"
#include "utils/logger.h"
#include <glad/glad.h>

namespace are {

namespace {
    GLenum get_gl_buffer_type(BufferType type) {
        switch (type) {
            case BufferType::VERTEX_BUFFER: return GL_ARRAY_BUFFER;
            case BufferType::INDEX_BUFFER: return GL_ELEMENT_ARRAY_BUFFER;
            case BufferType::UNIFORM_BUFFER: return GL_UNIFORM_BUFFER;
            case BufferType::SHADER_STORAGE_BUFFER: return GL_SHADER_STORAGE_BUFFER;
            default: return GL_ARRAY_BUFFER;
        }
    }
    
    GLenum get_gl_usage(BufferUsage usage) {
        switch (usage) {
            case BufferUsage::STATIC_DRAW: return GL_STATIC_DRAW;
            case BufferUsage::DYNAMIC_DRAW: return GL_DYNAMIC_DRAW;
            case BufferUsage::STREAM_DRAW: return GL_STREAM_DRAW;
            default: return GL_STATIC_DRAW;
        }
    }
}

Buffer::Buffer()
    : handle_(INVALID_HANDLE)
    , type_(BufferType::VERTEX_BUFFER)
    , size_(0)
    , usage_(BufferUsage::STATIC_DRAW) {
}

Buffer::~Buffer() {
    // Don't auto-release, let user control lifetime
}

bool Buffer::create(BufferType type, size_t size, const void* data, BufferUsage usage) {
    if (handle_ != INVALID_HANDLE) {
        Logger::warning("Buffer already created, releasing old buffer");
        release();
    }
    
    type_ = type;
    size_ = size;
    usage_ = usage;
    
    glGenBuffers(1, &handle_);
    
    GLenum gl_type = get_gl_buffer_type(type);
    GLenum gl_usage = get_gl_usage(usage);
    
    glBindBuffer(gl_type, handle_);
    glBufferData(gl_type, size, data, gl_usage);
    glBindBuffer(gl_type, 0);
    
    Logger::info("Buffer created successfully");
    return true;
}

void Buffer::update(size_t offset, size_t size, const void* data) {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot update invalid buffer");
        return;
    }
    
    if (offset + size > size_) {
        Logger::error("Buffer update out of bounds");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    
    glBindBuffer(gl_type, handle_);
    glBufferSubData(gl_type, offset, size, data);
    glBindBuffer(gl_type, 0);
}

void Buffer::bind() const {
    if (handle_ == INVALID_HANDLE) {
        Logger::warning("Attempting to bind invalid buffer");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBuffer(gl_type, handle_);
}

void Buffer::bind_base(uint binding_point) const {
    if (handle_ == INVALID_HANDLE) {
        Logger::warning("Attempting to bind invalid buffer");
        return;
    }
    
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBufferBase(gl_type, binding_point, handle_);
}

void Buffer::unbind() const {
    GLenum gl_type = get_gl_buffer_type(type_);
    glBindBuffer(gl_type, 0);
}

void Buffer::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteBuffers(1, &handle_);
        handle_ = INVALID_HANDLE;
    }
    
    size_ = 0;
}

} // namespace are
```

### 文件：src/resource/model_loader.cpp

```cpp
#include "resource/model_loader.h"
#include "utils/logger.h"
#include "resource/texture.h"

// Note: This is a simplified implementation without Assimp
// For full implementation, include Assimp and implement properly

namespace are {

bool ModelLoader::load(const std::string& path,
                      std::vector<std::shared_ptr<Mesh>>& meshes,
                      std::vector<std::shared_ptr<Material>>& materials,
                      bool flip_uvs) {
    Logger::error("ModelLoader requires Assimp library (not implemented in this version)");
    Logger::info("To implement: include <assimp/Importer.hpp>, <assimp/scene.h>, <assimp/postprocess.h>");
    
    // Placeholder implementation
    // TODO: Implement with Assimp
    /*
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, 
        aiProcess_Triangulate | 
        aiProcess_GenNormals |
        aiProcess_CalcTangentSpace |
        (flip_uvs ? aiProcess_FlipUVs : 0));
    
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        Logger::error("Failed to load model: " + std::string(importer.GetErrorString()));
        return false;
    }
    
    std::string directory = path.substr(0, path.find_last_of('/'));
    process_node_(scene->mRootNode, scene, meshes, materials, directory);
    
    Logger::info("Model loaded: " + path);
    return true;
    */
    
    return false;
}

bool ModelLoader::load_and_upload(const std::string& path,
                                 std::vector<std::shared_ptr<Mesh>>& meshes,
                                 std::vector<std::shared_ptr<Material>>& materials,
                                 bool flip_uvs) {
    if (!load(path, meshes, materials, flip_uvs)) {
        return false;
    }
    
    // Upload all meshes to GPU
    for (auto& mesh : meshes) {
        if (!mesh->upload_to_gpu()) {
            Logger::error("Failed to upload mesh to GPU");
            return false;
        }
    }
    
    return true;
}

void ModelLoader::process_node_(void* node, void* scene,
                               std::vector<std::shared_ptr<Mesh>>& meshes,
                               std::vector<std::shared_ptr<Material>>& materials,
                               const std::string& directory) {
    // TODO: Implement with Assimp
    /*
    aiNode* ai_node = static_cast<aiNode*>(node);
    const aiScene* ai_scene = static_cast<const aiScene*>(scene);
    
    // Process all meshes in this node
    for (uint i = 0; i < ai_node->mNumMeshes; ++i) {
        aiMesh* ai_mesh = ai_scene->mMeshes[ai_node->mMeshes[i]];
        meshes.push_back(process_mesh_(ai_mesh, ai_scene, materials, directory));
    }
    
    // Process children recursively
    for (uint i = 0; i < ai_node->mNumChildren; ++i) {
        process_node_(ai_node->mChildren[i], ai_scene, meshes, materials, directory);
    }
    */
}

std::shared_ptr<Mesh> ModelLoader::process_mesh_(void* mesh, void* scene,
                                                std::vector<std::shared_ptr<Material>>& materials,
                                                const std::string& directory) {
    // TODO: Implement with Assimp
    /*
    aiMesh* ai_mesh = static_cast<aiMesh*>(mesh);
    const aiScene* ai_scene = static_cast<const aiScene*>(scene);
    
    std::vector<Vertex> vertices;
    std::vector<uint> indices;
    
    // Process vertices
    for (uint i = 0; i < ai_mesh->mNumVertices; ++i) {
        Vertex vertex;
        
        vertex.position_ = Vec3(ai_mesh->mVertices[i].x,
                               ai_mesh->mVertices[i].y,
                               ai_mesh->mVertices[i].z);
        
        if (ai_mesh->HasNormals()) {
            vertex.normal_ = Vec3(ai_mesh->mNormals[i].x,
                                 ai_mesh->mNormals[i].y,
                                 ai_mesh->mNormals[i].z);
        }
        
        if (ai_mesh->mTextureCoords[0]) {
            vertex.texcoord_ = Vec2(ai_mesh->mTextureCoords[0][i].x,
                                   ai_mesh->mTextureCoords[0][i].y);
        }
        
        if (ai_mesh->HasTangentsAndBitangents()) {
            vertex.tangent_ = Vec3(ai_mesh->mTangents[i].x,
                                  ai_mesh->mTangents[i].y,
                                  ai_mesh->mTangents[i].z);
        }
        
        vertices.push_back(vertex);
    }
    
    // Process indices
    for (uint i = 0; i < ai_mesh->mNumFaces; ++i) {
        aiFace face = ai_mesh->mFaces[i];
        for (uint j = 0; j < face.mNumIndices; ++j) {
            indices.push_back(face.mIndices[j]);
        }
    }
    
    // Process material
    uint material_id = materials.size();
    if (ai_mesh->mMaterialIndex >= 0) {
        aiMaterial* ai_material = ai_scene->mMaterials[ai_mesh->mMaterialIndex];
        
        auto material = std::make_shared<Material>();
        
        // Load diffuse color
        aiColor3D color;
        if (ai_material->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS) {
            material->set_albedo(Vec3(color.r, color.g, color.b));
        }
        
        // Load textures
        load_material_textures_(ai_material, aiTextureType_DIFFUSE, material, directory);
        load_material_textures_(ai_material, aiTextureType_NORMALS, material, directory);
        
        materials.push_back(material);
    }
    
    auto mesh_obj = std::make_shared<Mesh>();
    mesh_obj->set_vertices(vertices);
    mesh_obj->set_indices(indices);
    mesh_obj->set_material(material_id);
    
    return mesh_obj;
    */
    
    return nullptr;
}

void ModelLoader::load_material_textures_(void* material, int type,
                                         std::shared_ptr<Material>& mat,
                                         const std::string& directory) {
    // TODO: Implement with Assimp
    /*
    aiMaterial* ai_material = static_cast<aiMaterial*>(material);
    aiTextureType ai_type = static_cast<aiTextureType>(type);
    
    for (uint i = 0; i < ai_material->GetTextureCount(ai_type); ++i) {
        aiString str;
        ai_material->GetTexture(ai_type, i, &str);
        
        std::string filename = directory + "/" + std::string(str.C_Str());
        
        auto texture = std::make_shared<Texture>();
        if (texture->load_from_file(filename)) {
            if (ai_type == aiTextureType_DIFFUSE) {
                mat->set_albedo_texture(texture);
            } else if (ai_type == aiTextureType_NORMALS) {
                mat->set_normal_texture(texture);
            }
        }
    }
    */
}

} // namespace are
```

### 文件：src/resource/shader.cpp

```cpp
#include "resource/shader.h"
#include "utils/logger.h"
#include "basic/math.h"  // 修改为math.h
#include <glad/glad.h>
#include <fstream>
#include <sstream>

namespace are {

Shader::Shader()
    : handle_(INVALID_HANDLE) {
}

Shader::~Shader() {
    // Don't auto-release, let user control lifetime
}

bool Shader::load(const std::string& vertex_path, const std::string& fragment_path) {
    std::string vertex_source = read_file_(vertex_path);
    std::string fragment_source = read_file_(fragment_path);
    
    if (vertex_source.empty() || fragment_source.empty()) {
        Logger::error("Failed to read shader files");
        return false;
    }
    
    return compile(vertex_source, fragment_source);
}

bool Shader::load_compute(const std::string& compute_path) {
    std::string compute_source = read_file_(compute_path);
    
    if (compute_source.empty()) {
        Logger::error("Failed to read compute shader file");
        return false;
    }
    
    return compile_compute(compute_source);
}

bool Shader::compile(const std::string& vertex_source, const std::string& fragment_source) {
    uint vertex_shader = compile_shader_(vertex_source, GL_VERTEX_SHADER);
    if (vertex_shader == 0) return false;
    
    uint fragment_shader = compile_shader_(fragment_source, GL_FRAGMENT_SHADER);
    if (fragment_shader == 0) {
        glDeleteShader(vertex_shader);
        return false;
    }
    
    uint shaders[] = { vertex_shader, fragment_shader };
    bool success = link_program_(shaders, 2);
    
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    return success;
}

bool Shader::compile_compute(const std::string& compute_source) {
    uint compute_shader = compile_shader_(compute_source, GL_COMPUTE_SHADER);
    if (compute_shader == 0) return false;
    
    uint shaders[] = { compute_shader };
    bool success = link_program_(shaders, 1);
    
    glDeleteShader(compute_shader);
    
    return success;
}

void Shader::use() const {  // 改为const
    if (handle_ != INVALID_HANDLE) {
        glUseProgram(handle_);
    }
}

void Shader::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteProgram(handle_);
        handle_ = INVALID_HANDLE;
    }
    uniform_cache_.clear();
}

void Shader::set_bool(const std::string& name, bool value) const {  // 新增
    glUniform1i(get_uniform_location_(name), static_cast<int>(value));
}

void Shader::set_int(const std::string& name, int value) const {  // 改为const
    glUniform1i(get_uniform_location_(name), value);
}

void Shader::set_uint(const std::string& name, uint value) const {  // 改为const
    glUniform1ui(get_uniform_location_(name), value);
}

void Shader::set_float(const std::string& name, float value) const {  // 改为const
    glUniform1f(get_uniform_location_(name), value);
}

void Shader::set_vec2(const std::string& name, const Vec2& value) const {  // 改为const
    glUniform2fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_vec3(const std::string& name, const Vec3& value) const {  // 改为const
    glUniform3fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_vec4(const std::string& name, const Vec4& value) const {  // 改为const
    glUniform4fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_mat3(const std::string& name, const Mat3& value) const {  // 改为const
    glUniformMatrix3fv(get_uniform_location_(name), 1, GL_FALSE, &value[0][0]);
}

void Shader::set_mat4(const std::string& name, const Mat4& value) const {  // 改为const
    glUniformMatrix4fv(get_uniform_location_(name), 1, GL_FALSE, MathUtils::value_ptr(value));
}

int Shader::get_uniform_location_(const std::string& name) const {  // 改为const
    auto it = uniform_cache_.find(name);
    if (it != uniform_cache_.end()) {
        return it->second;
    }
    
    int location = glGetUniformLocation(handle_, name.c_str());
    uniform_cache_[name] = location;  // mutable允许修改
    
    if (location == -1) {
        Logger::warning("Uniform '" + name + "' not found in shader");
    }
    
    return location;
}

uint Shader::compile_shader_(const std::string& source, uint type) {
    uint shader = glCreateShader(type);
    const char* source_cstr = source.c_str();
    glShaderSource(shader, 1, &source_cstr, nullptr);
    glCompileShader(shader);
    
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(shader, 512, nullptr, info_log);
        
        std::string type_str = (type == GL_VERTEX_SHADER) ? "VERTEX" :
                              (type == GL_FRAGMENT_SHADER) ? "FRAGMENT" : "COMPUTE";
        Logger::error("Shader compilation failed (" + type_str + "): " + std::string(info_log));
        
        glDeleteShader(shader);
        return 0;
    }
    
    return shader;
}

bool Shader::link_program_(const uint* shaders, uint count) {
    handle_ = glCreateProgram();
    
    for (uint i = 0; i < count; ++i) {
        glAttachShader(handle_, shaders[i]);
    }
    
    glLinkProgram(handle_);
    
    int success;
    glGetProgramiv(handle_, GL_LINK_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetProgramInfoLog(handle_, 512, nullptr, info_log);
        Logger::error("Shader linking failed: " + std::string(info_log));
        
        glDeleteProgram(handle_);
        handle_ = INVALID_HANDLE;
        return false;
    }
    
    return true;
}

std::string Shader::read_file_(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        Logger::error("Failed to open file: " + path);
        return "";
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

} // namespace are
```

### 文件：src/resource/texture.cpp

```cpp
#include "resource/texture.h"
#include "utils/logger.h"
#include <glad/glad.h>
#include <stb_image.h>

namespace are {

namespace {
    GLenum get_gl_internal_format(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8: return GL_R8;
            case TextureFormat::RG8: return GL_RG8;
            case TextureFormat::RGB8: return GL_RGB8;
            case TextureFormat::RGBA8: return GL_RGBA8;
            case TextureFormat::R16F: return GL_R16F;
            case TextureFormat::RG16F: return GL_RG16F;
            case TextureFormat::RGB16F: return GL_RGB16F;
            case TextureFormat::RGBA16F: return GL_RGBA16F;
            case TextureFormat::R32F: return GL_R32F;
            case TextureFormat::RG32F: return GL_RG32F;
            case TextureFormat::RGB32F: return GL_RGB32F;
            case TextureFormat::RGBA32F: return GL_RGBA32F;
            case TextureFormat::DEPTH24_STENCIL8: return GL_DEPTH24_STENCIL8;
            default: return GL_RGBA8;
        }
    }
    
    GLenum get_gl_format(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8:
            case TextureFormat::R16F:
            case TextureFormat::R32F:
                return GL_RED;
            case TextureFormat::RG8:
            case TextureFormat::RG16F:
            case TextureFormat::RG32F:
                return GL_RG;
            case TextureFormat::RGB8:
            case TextureFormat::RGB16F:
            case TextureFormat::RGB32F:
                return GL_RGB;
            case TextureFormat::RGBA8:
            case TextureFormat::RGBA16F:
            case TextureFormat::RGBA32F:
                return GL_RGBA;
            case TextureFormat::DEPTH24_STENCIL8:
                return GL_DEPTH_STENCIL;
            default:
                return GL_RGBA;
        }
    }
    
    GLenum get_gl_type(TextureFormat format) {
        switch (format) {
            case TextureFormat::R8:
            case TextureFormat::RG8:
            case TextureFormat::RGB8:
            case TextureFormat::RGBA8:
                return GL_UNSIGNED_BYTE;
            case TextureFormat::R16F:
            case TextureFormat::RG16F:
            case TextureFormat::RGB16F:
            case TextureFormat::RGBA16F:
            case TextureFormat::R32F:
            case TextureFormat::RG32F:
            case TextureFormat::RGB32F:
            case TextureFormat::RGBA32F:
                return GL_FLOAT;
            case TextureFormat::DEPTH24_STENCIL8:
                return GL_UNSIGNED_INT_24_8;
            default:
                return GL_UNSIGNED_BYTE;
        }
    }
    
    GLenum get_gl_filter(TextureFilter filter) {
        switch (filter) {
            case TextureFilter::NEAREST: return GL_NEAREST;
            case TextureFilter::LINEAR: return GL_LINEAR;
            case TextureFilter::NEAREST_MIPMAP_NEAREST: return GL_NEAREST_MIPMAP_NEAREST;
            case TextureFilter::LINEAR_MIPMAP_NEAREST: return GL_LINEAR_MIPMAP_NEAREST;
            case TextureFilter::NEAREST_MIPMAP_LINEAR: return GL_NEAREST_MIPMAP_LINEAR;
            case TextureFilter::LINEAR_MIPMAP_LINEAR: return GL_LINEAR_MIPMAP_LINEAR;
            default: return GL_LINEAR;
        }
    }
    
    GLenum get_gl_wrap(TextureWrap wrap) {
        switch (wrap) {
            case TextureWrap::REPEAT: return GL_REPEAT;
            case TextureWrap::MIRRORED_REPEAT: return GL_MIRRORED_REPEAT;
            case TextureWrap::CLAMP_TO_EDGE: return GL_CLAMP_TO_EDGE;
            case TextureWrap::CLAMP_TO_BORDER: return GL_CLAMP_TO_BORDER;
            default: return GL_REPEAT;
        }
    }
}

Texture::Texture()
    : handle_(INVALID_HANDLE)
    , width_(0)
    , height_(0)
    , format_(TextureFormat::RGBA8)
    , has_mipmaps_(false) {
}

Texture::~Texture() {
    // Don't auto-release, let user control lifetime
}

bool Texture::load_from_file(const std::string& path, bool generate_mipmaps) {
    // Load image using stb_image
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        Logger::error("Failed to load texture: " + path);
        return false;
    }
    
    // Determine format based on channels
    TextureFormat format;
    switch (channels) {
        case 1: format = TextureFormat::R8; break;
        case 2: format = TextureFormat::RG8; break;
        case 3: format = TextureFormat::RGB8; break;
        case 4: format = TextureFormat::RGBA8; break;
        default:
            Logger::error("Unsupported channel count: " + std::to_string(channels));
            stbi_image_free(data);
            return false;
    }
    
    // Create texture
    bool success = create(width, height, format);
    if (!success) {
        stbi_image_free(data);
        return false;
    }
    
    // Upload data
    success = upload(data, width, height, format);
    stbi_image_free(data);
    
    if (!success) {
        return false;
    }
    
    // Generate mipmaps if requested
    if (generate_mipmaps) {
        this->generate_mipmaps();
    }
    
    Logger::info("Texture loaded successfully: " + path);
    return true;
}

bool Texture::create(uint width, uint height, TextureFormat format) {
    if (handle_ != INVALID_HANDLE) {
        Logger::warning("Texture already created, releasing old texture");
        release();
    }
    
    width_ = width;
    height_ = height;
    format_ = format;
    
    glGenTextures(1, &handle_);
    glBindTexture(GL_TEXTURE_2D, handle_);
    
    GLenum internal_format = get_gl_internal_format(format);
    GLenum gl_format = get_gl_format(format);
    GLenum type = get_gl_type(format);
    
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, gl_format, type, nullptr);
    
    // Set default parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    
    return true;
}

bool Texture::upload(const void* data, uint width, uint height, TextureFormat format) {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot upload to invalid texture");
        return false;
    }
    
    if (width != width_ || height != height_ || format != format_) {
        Logger::warning("Upload parameters differ from texture creation, recreating texture");
        create(width, height, format);
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    
    GLenum gl_format = get_gl_format(format);
    GLenum type = get_gl_type(format);
    
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, gl_format, type, data);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    
    return true;
}

void Texture::set_filter(TextureFilter min_filter, TextureFilter mag_filter) {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot set filter on invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, get_gl_filter(min_filter));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, get_gl_filter(mag_filter));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::set_wrap(TextureWrap wrap_s, TextureWrap wrap_t) {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot set wrap mode on invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, get_gl_wrap(wrap_s));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, get_gl_wrap(wrap_t));
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::generate_mipmaps() {
    if (handle_ == INVALID_HANDLE) {
        Logger::error("Cannot generate mipmaps for invalid texture");
        return;
    }
    
    glBindTexture(GL_TEXTURE_2D, handle_);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    
    has_mipmaps_ = true;
}

void Texture::bind(uint unit) const {
    if (handle_ == INVALID_HANDLE) {
        Logger::warning("Attempting to bind invalid texture");
        return;
    }
    
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, handle_);
}

void Texture::unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::release() {
    if (handle_ != INVALID_HANDLE) {
        glDeleteTextures(1, &handle_);
        handle_ = INVALID_HANDLE;
    }
    
    width_ = 0;
    height_ = 0;
    has_mipmaps_ = false;
}

} // namespace are
```

### 文件：src/utils/config.cpp

```cpp
#include "utils/config.h"
#include "utils/logger.h"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace are {

// Static storage
static std::unordered_map<std::string, std::string> g_config_map;

// Helper function to trim whitespace
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

bool Config::load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        Logger::error("Failed to open config file: " + path);
        return false;
    }
    
    g_config_map.clear();
    
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        line = trim(line);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        
        // Section header
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }
        
        // Key-value pair
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            
            // Add section prefix if in a section
            if (!current_section.empty()) {
                key = current_section + "." + key;
            }
            
            g_config_map[key] = value;
        }
    }
    
    Logger::info("Config loaded: " + path + " (" + std::to_string(g_config_map.size()) + " entries)");
    return true;
}

bool Config::save(const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        Logger::error("Failed to open config file for writing: " + path);
        return false;
    }
    
    for (const auto& pair : g_config_map) {
        file << pair.first << "=" << pair.second << std::endl;
    }
    
    Logger::info("Config saved: " + path);
    return true;
}

std::string Config::get_string(const std::string& key, const std::string& default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        return it->second;
    }
    return default_value;
}

int Config::get_int(const std::string& key, int default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        try {
            return std::stoi(it->second);
        } catch (...) {
            Logger::warning("Failed to parse int for key: " + key);
        }
    }
    return default_value;
}

float Config::get_float(const std::string& key, float default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        try {
            return std::stof(it->second);
        } catch (...) {
            Logger::warning("Failed to parse float for key: " + key);
        }
    }
    return default_value;
}

bool Config::get_bool(const std::string& key, bool default_value) {
    auto it = g_config_map.find(key);
    if (it != g_config_map.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        
        if (value == "true" || value == "1" || value == "yes" || value == "on") {
            return true;
        }
        if (value == "false" || value == "0" || value == "no" || value == "off") {
            return false;
        }
    }
    return default_value;
}

void Config::set_string(const std::string& key, const std::string& value) {
    g_config_map[key] = value;
}

void Config::set_int(const std::string& key, int value) {
    g_config_map[key] = std::to_string(value);
}

void Config::set_float(const std::string& key, float value) {
    g_config_map[key] = std::to_string(value);
}

void Config::set_bool(const std::string& key, bool value) {
    g_config_map[key] = value ? "true" : "false";
}

} // namespace are
```

### 文件：src/utils/logger.cpp

```cpp
#include "utils/logger.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace are {

// Static members
static LogLevel g_min_level = LogLevel::DEBUG;
static std::ofstream g_log_file;
static bool g_initialized = false;

bool Logger::initialize(const std::string& log_file) {
    if (g_initialized) {
        return true;
    }
    
    if (!log_file.empty()) {
        g_log_file.open(log_file, std::ios::out | std::ios::app);
        if (!g_log_file.is_open()) {
            std::cerr << "Failed to open log file: " << log_file << std::endl;
            return false;
        }
    }
    
    g_initialized = true;
    return true;
}

void Logger::shutdown() {
    if (g_log_file.is_open()) {
        g_log_file.close();
    }
    g_initialized = false;
}

static std::string get_current_time() {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

static std::string level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    if (level < g_min_level) return;
    
    std::string time_str = get_current_time();
    std::string level_str = level_to_string(level);
    std::string formatted = "[" + time_str + "] [" + level_str + "] " + message;
    
    // Console output
    if (level >= LogLevel::ERROR) {
        std::cerr << formatted << std::endl;
    } else {
        std::cout << formatted << std::endl;
    }
    
    // File output
    if (g_log_file.is_open()) {
        g_log_file << formatted << std::endl;
        g_log_file.flush();
    }
}

void Logger::debug(const std::string& message) {
    log(LogLevel::DEBUG, message);
}

void Logger::info(const std::string& message) {
    log(LogLevel::INFO, message);
}

void Logger::warning(const std::string& message) {
    log(LogLevel::WARNING, message);
}

void Logger::error(const std::string& message) {
    log(LogLevel::ERROR, message);
}

void Logger::fatal(const std::string& message) {
    log(LogLevel::FATAL, message);
}

void Logger::set_level(LogLevel level) {
    g_min_level = level;
}

} // namespace are
```

