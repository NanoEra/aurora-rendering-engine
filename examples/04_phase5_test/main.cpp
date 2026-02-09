/**
 * @file main.cpp
 * @brief Phase 5 test: Hybrid GBuffer-driven CPU raytracing with primitive-id
 */

#include <are/core/logger.h>
#include <are/core/profiler.h>

#include <are/platform/window.h>
#include <are/platform/gl_context.h>

#include <are/rasterizer/rasterizer.h>
#include <are/rasterizer/gbuffer.h>
#include <are/rasterizer/shader_program.h>

#include <are/renderer/geometry_cache.h>

#include <are/scene/scene_manager.h>
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/directional_light.h>
#include <are/scene/point_light.h>

#include <are/raytracer/cpu_raytracer.h>

#include "../lib/glad/glad/glad.h"

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

using namespace are;

namespace {

uint32_t create_fullscreen_quad_vao() {
    float vertices[] = {
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        -1.0f,  1.0f,  0.0f, 1.0f
    };
    uint32_t indices[] = { 0, 1, 2, 2, 3, 0 };

    uint32_t vao = 0, vbo = 0, ebo = 0;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
    return vao;
}

uint32_t create_output_texture_rgba16f(int width, int height) {
    uint32_t tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
}

} // namespace

int main() {
    try {
        Logger::init(LogLevel::ARE_LOG_INFO);
        Profiler::init();

        WindowConfig wc;
        wc.width = 960;
        wc.height = 540;
        wc.title = "ARE Phase5 - Hybrid + PrimitiveID";
        wc.vsync = true;

        Window window(wc);

        if (!GLContext::initialize()) {
            ARE_LOG_CRITICAL("GLContext initialize failed");
            return -1;
        }

        int fb_w = 0, fb_h = 0;
        for (int i = 0; i < 240; ++i) {
            window.poll_events();
            window.get_framebuffer_size(fb_w, fb_h);
            if (fb_w > 0 && fb_h > 0) break;
            window.swap_buffers();
        }
        if (fb_w <= 0 || fb_h <= 0) {
            ARE_LOG_CRITICAL("Invalid framebuffer size");
            return -1;
        }

        Rasterizer rasterizer(fb_w, fb_h);
        rasterizer.initialize_shaders("shaders/");

        uint32_t output_tex = create_output_texture_rgba16f(fb_w, fb_h);
        uint32_t quad_vao = create_fullscreen_quad_vao();

        // Display shader (simple)
        const char* vs = R"(
#version 430 core
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec2 a_uv;
out vec2 v_uv;
void main(){ v_uv=a_uv; gl_Position=vec4(a_pos,0,1); }
)";
        const char* fs = R"(
#version 430 core
in vec2 v_uv;
out vec4 frag_color;
uniform sampler2D u_tex;
void main(){ frag_color = texture(u_tex, v_uv); }
)";

        ShaderProgram display;
        if (!display.compile_shader(ShaderType::ARE_SHADER_VERTEX, vs) ||
            !display.compile_shader(ShaderType::ARE_SHADER_FRAGMENT, fs) ||
            !display.link()) {
            ARE_LOG_CRITICAL("Display shader compile failed");
            return -1;
        }

        // Scene setup
        SceneManager scene;

        Material mat;
        mat.set_albedo(Vec3(0.8f, 0.2f, 0.2f));
        mat.set_roughness(0.6f);
        MaterialHandle mat_h = scene.add_material(mat);

        std::vector<Vertex> ground_v = {
            Vertex(Vec3(-4, 0, -4), Vec3(0, 1, 0), Vec2(0, 0)),
            Vertex(Vec3( 4, 0, -4), Vec3(0, 1, 0), Vec2(1, 0)),
            Vertex(Vec3( 4, 0,  4), Vec3(0, 1, 0), Vec2(1, 1)),
            Vertex(Vec3(-4, 0,  4), Vec3(0, 1, 0), Vec2(0, 1)),
        };
        std::vector<uint32_t> ground_i = { 0, 1, 2, 2, 3, 0 };
        Mesh ground(ground_v, ground_i, mat_h);
        ground.compute_tangents();

        std::vector<Vertex> tri_v = {
            Vertex(Vec3(-0.8f, 0.2f, 0.0f), Vec3(0, 1, 0), Vec2(0, 0)),
            Vertex(Vec3( 0.8f, 0.2f, 0.0f), Vec3(0, 1, 0), Vec2(1, 0)),
            Vertex(Vec3( 0.0f, 1.2f, 0.3f), Vec3(0, 1, 0), Vec2(0.5f, 1)),
        };
        std::vector<uint32_t> tri_i = { 0, 1, 2 };
        Mesh tri_mesh(tri_v, tri_i, mat_h);
        tri_mesh.compute_tangents();

        // Upload meshes BEFORE adding (SceneManager stores copy)
        rasterizer.upload_mesh(ground);
        scene.add_mesh(ground);

        rasterizer.upload_mesh(tri_mesh);
        scene.add_mesh(tri_mesh);

        auto sun = std::make_shared<DirectionalLight>(Vec3(-1, -1, -0.5f), Vec3(1.0f), 2.0f);
        sun->set_cast_shadows(true);
        scene.add_light(sun);

        auto point = std::make_shared<PointLight>(Vec3(0, 2.5f, 1.5f), Vec3(1.0f, 0.95f, 0.8f), 10.0f, 10.0f);
        point->set_cast_shadows(true);
        scene.add_light(point);

        Camera camera(Vec3(0.0f, 1.8f, 4.5f), Vec3(0.0f, 0.6f, 0.0f));
        camera.set_perspective(45.0f, static_cast<Real>(fb_w) / static_cast<Real>(fb_h), 0.1f, 200.0f);

        // Geometry cache: single source of truth
        GeometryCache geom;
        if (!geom.build_from_scene(scene)) {
            ARE_LOG_CRITICAL("GeometryCache build failed");
            return -1;
        }

        // Provide triangle base offsets to rasterizer for primitive id output
        rasterizer.set_triangle_base_provider([&](size_t mesh_index) {
            return geom.get_mesh_triangle_base(mesh_index);
        });

        // CPU ray tracer uses the same BVH (same triangle layout)
        RayTracingConfig rtc;
        rtc.backend = RayTracingBackend::ARE_RT_BACKEND_CPU;
        rtc.spp = 1;
        rtc.max_depth = 3;
        rtc.enable_gi = false;
        rtc.enable_ao = false;
        rtc.ao_samples = 4;
        rtc.ao_radius = 1.0f;

        CPURayTracer tracer(rtc);
        tracer.update_bvh(geom.get_bvh());

        bool request_render = true;

        while (!window.should_close()) {
            window.poll_events();

            if (window.is_key_pressed(256)) {
                window.set_should_close(true);
            }

            int new_fb_w = 0, new_fb_h = 0;
            window.get_framebuffer_size(new_fb_w, new_fb_h);
            if (new_fb_w <= 0 || new_fb_h <= 0) {
                window.swap_buffers();
                continue;
            }

            if (new_fb_w != fb_w || new_fb_h != fb_h) {
                fb_w = new_fb_w;
                fb_h = new_fb_h;

                rasterizer.resize(fb_w, fb_h);
                glDeleteTextures(1, &output_tex);
                output_tex = create_output_texture_rgba16f(fb_w, fb_h);
                camera.set_aspect_ratio(static_cast<Real>(fb_w) / static_cast<Real>(fb_h));

                request_render = true;
            }

            if (request_render) {
                rasterizer.render_gbuffer(scene, camera);
                tracer.render(scene, camera, &rasterizer.get_gbuffer(), output_tex);
                request_render = false;
            }

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, fb_w, fb_h);
            glDisable(GL_DEPTH_TEST);

            display.use();
            display.set_uniform("u_tex", 0);

            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, output_tex);

            glBindVertexArray(quad_vao);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);

            glBindTexture(GL_TEXTURE_2D, 0);
            window.swap_buffers();
        }

        glDeleteTextures(1, &output_tex);
        glDeleteVertexArrays(1, &quad_vao);

        Profiler::shutdown();
        Logger::shutdown();
        return 0;

    } catch (const std::exception& e) {
        Logger::init(LogLevel::ARE_LOG_INFO);
        ARE_LOG_CRITICAL(std::string("Unhandled exception: ") + e.what());
        Logger::shutdown();
        return -1;
    }
}
