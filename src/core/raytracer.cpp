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
    glBindImageTexture(GBUFFER_TEXTURE_COUNT, output_texture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(GBUFFER_TEXTURE_COUNT + 1, accumulation_texture_, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    
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
        // 使用 vec4 确保对齐正确
        struct alignas(16) MaterialData {
            Vec4 albedo_metallic;      // xyz = albedo, w = metallic
            Vec4 emission_roughness;   // xyz = emission, w = roughness
            int type;
            float ior;
            float padding1;
            float padding2;
        };
        
        std::vector<MaterialData> material_data;
        material_data.reserve(materials.size());
        
        for (const auto& mat : materials) {
            MaterialData data;
            data.albedo_metallic = Vec4(mat->get_albedo(), mat->get_metallic());
            data.emission_roughness = Vec4(mat->get_emission(), mat->get_roughness());
            data.type = static_cast<int>(mat->get_type());
            data.ior = mat->get_ior();
            data.padding1 = 0.0f;
            data.padding2 = 0.0f;
            material_data.push_back(data);
        }
        
        // 打印调试信息
        Logger::info("MaterialData size: " + std::to_string(sizeof(MaterialData)) + " bytes");
        Logger::info("Material[0] albedo: (" + 
                    std::to_string(material_data[0].albedo_metallic.x) + ", " +
                    std::to_string(material_data[0].albedo_metallic.y) + ", " +
                    std::to_string(material_data[0].albedo_metallic.z) + ")");
        Logger::info("Material[0] metallic: " + std::to_string(material_data[0].albedo_metallic.w));
        
        // 找到金属材质并打印
        for (size_t i = 0; i < material_data.size(); i++) {
            if (material_data[i].albedo_metallic.w > 0.5f) {
                Logger::info("Material[" + std::to_string(i) + "] is metallic: " + 
                            std::to_string(material_data[i].albedo_metallic.w));
            }
        }
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, material_buffer_);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 
                    material_data.size() * sizeof(MaterialData),
                    material_data.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, material_buffer_);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        
        Logger::info("Uploaded " + std::to_string(material_data.size()) + " materials to GPU");
    } else {
        Logger::warning("No materials to upload");
    }
    
    // Upload lights (保持不变)
    const auto& lights = scene.get_lights();
    if (!lights.empty()) {
        // 同样使用 vec4 确保对齐
        struct alignas(16) LightData {
            Vec4 position_type;      // xyz = position, w = type (as float)
            Vec4 direction_intensity; // xyz = direction, w = intensity
            Vec4 color_range;        // xyz = color, w = range
            Vec4 spot_angles;        // xy = spot angles, zw = padding
        };
        
        std::vector<LightData> light_data;
        light_data.reserve(lights.size());
        
        for (const auto& light : lights) {
            LightData data;
            data.position_type = Vec4(light->get_position(), static_cast<float>(light->get_type()));
            data.direction_intensity = Vec4(light->get_direction(), light->get_intensity());
            data.color_range = Vec4(light->get_color(), light->get_range());
            data.spot_angles = Vec4(light->get_inner_angle(), light->get_outer_angle(), 0.0f, 0.0f);
            light_data.push_back(data);
        }
        
        Logger::info("LightData size: " + std::to_string(sizeof(LightData)) + " bytes");
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, light_buffer_);
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                    light_data.size() * sizeof(LightData),
                    light_data.data(), GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, light_buffer_);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        
        Logger::info("Uploaded " + std::to_string(light_data.size()) + " lights to GPU");
    } else {
        Logger::warning("No lights to upload");
    }
}

void RayTracer::bind_gbuffer_(const GBuffer& gbuffer) {
    glBindImageTexture(GBUFFER_POSITION, gbuffer.get_texture(GBUFFER_POSITION), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(GBUFFER_NORMAL, gbuffer.get_texture(GBUFFER_NORMAL), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
    glBindImageTexture(GBUFFER_ALBEDO, gbuffer.get_texture(GBUFFER_ALBEDO), 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA8);
	glBindImageTexture(GBUFFER_MATERIAL_ID, gbuffer.get_texture(GBUFFER_MATERIAL_ID), 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32UI);
}

void RayTracer::set_compute_shader(const Shader& shader) {
    compute_shader_ = shader;
    Logger::info("Compute shader set for RayTracer");
}

} // namespace are
