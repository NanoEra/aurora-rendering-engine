/**
 * @file main.cpp
 * @brief Phase 5 verification program - CPU ray tracing test (RGBA16F output)
 */

#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>

#include <are/platform/gl_context.h>
#include <are/platform/window.h>

#include <are/rasterizer/gbuffer.h>
#include <are/rasterizer/rasterizer.h>
#include <are/rasterizer/shader_program.h>

#include <are/scene/camera.h>
#include <are/scene/directional_light.h>
#include <are/scene/material.h>
#include <are/scene/mesh.h>
#include <are/scene/point_light.h>
#include <are/scene/scene_manager.h>

#include <are/acceleration/bvh.h>
#include <are/geometry/triangle.h>
#include <are/geometry/vertex.h>

#include <are/raytracer/cpu_raytracer.h>

#include "../lib/glad/glad/glad.h"
#include <glm/glm.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace are;

namespace {

/**
 * @brief Create a fullscreen quad VAO.
 * @return VAO handle
 */
uint32_t create_fullscreen_quad_vao() {
	float vertices[] = {
		// pos      // uv
		-1.0f, -1.0f, 0.0f, 0.0f,
		1.0f, -1.0f, 1.0f, 0.0f,
		1.0f, 1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, 0.0f, 1.0f
	};

	uint32_t indices[] = { 0, 1, 2, 2, 3, 0 };

	uint32_t vao = 0;
	uint32_t vbo = 0;
	uint32_t ebo = 0;

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));

	glBindVertexArray(0);

	// Intentionally leak vbo/ebo for this small test (or store & delete if you prefer).
	return vao;
}

/**
 * @brief Create RGBA16F output texture.
 * @param width Texture width
 * @param height Texture height
 * @return Texture ID
 */
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

/**
 * @brief Build triangles from a mesh (positions only, identity transform).
 * @param mesh Mesh
 * @param material Material handle
 * @return Triangle list
 */
std::vector<Triangle> mesh_to_triangles(const Mesh &mesh, MaterialHandle material) {
	std::vector<Triangle> tris;
	if (mesh.is_empty()) {
		return tris;
	}

	const auto &v = mesh.get_vertices();
	const auto &idx = mesh.get_indices();

	if (idx.size() % 3 != 0) {
		ARE_LOG_ERROR("phase5_test: mesh indices not multiple of 3");
		return tris;
	}

	tris.reserve(idx.size() / 3);
	for (size_t i = 0; i < idx.size(); i += 3) {
		uint32_t i0 = idx[i + 0];
		uint32_t i1 = idx[i + 1];
		uint32_t i2 = idx[i + 2];
		if (i0 >= v.size() || i1 >= v.size() || i2 >= v.size()) {
			continue;
		}
		Triangle t(v[i0], v[i1], v[i2], material);
		tris.push_back(t);
	}
	return tris;
}

} // namespace

int main() {
	Logger::init(LogLevel::ARE_LOG_INFO);
	Profiler::init();

	WindowConfig wc;
	wc.width = 960;
	wc.height = 540;
	wc.title = "ARE Phase 5 - CPU Ray Tracing (RGBA16F)";
	wc.vsync = true;

	Window window(wc);

	if (!GLContext::initialize()) {
		ARE_LOG_CRITICAL("phase5_test: GLContext::initialize failed");
		return -1;
	}
	GLContext::print_info();

	int fb_w = 0;
	int fb_h = 0;
	window.get_framebuffer_size(fb_w, fb_h);
	if (fb_w <= 0 || fb_h <= 0) {
		ARE_LOG_CRITICAL("phase5_test: framebuffer size is invalid");
		return -1;
	}

	// GBuffer only used for resolution / future hybrid path
	Rasterizer rasterizer(fb_w, fb_h);
	GBuffer &gbuffer = rasterizer.get_gbuffer();

	// Output texture (RGBA16F)
	uint32_t output_tex = create_output_texture_rgba16f(fb_w, fb_h);

	// Simple display shader
	const char *fsq_vs = R"(
#version 430 core
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec2 a_uv;
out vec2 v_uv;
void main() {
    v_uv = a_uv;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
)";

	const char *fsq_fs = R"(
#version 430 core
in vec2 v_uv;
out vec4 frag_color;
uniform sampler2D u_tex;
void main() {
    frag_color = texture(u_tex, v_uv);
}
)";

	ShaderProgram display;
	if (!display.compile_shader(ShaderType::ARE_SHADER_VERTEX, fsq_vs) || !display.compile_shader(ShaderType::ARE_SHADER_FRAGMENT, fsq_fs) || !display.link()) {
		ARE_LOG_CRITICAL("phase5_test: failed to compile display shader");
		return -1;
	}

	uint32_t quad_vao = create_fullscreen_quad_vao();

	// Scene setup
	SceneManager scene;

	Material mat;
	mat.set_albedo(Vec3(0.8f, 0.2f, 0.2f));
	mat.set_roughness(0.6f);
	MaterialHandle mat_h = scene.add_material(mat);

	// A ground plane (two triangles)
	std::vector<Vertex> ground_v = {
		Vertex(Vec3(-4, 0, -4), Vec3(0, 1, 0), Vec2(0, 0)),
		Vertex(Vec3(4, 0, -4), Vec3(0, 1, 0), Vec2(1, 0)),
		Vertex(Vec3(4, 0, 4), Vec3(0, 1, 0), Vec2(1, 1)),
		Vertex(Vec3(-4, 0, 4), Vec3(0, 1, 0), Vec2(0, 1)),
	};
	std::vector<uint32_t> ground_i = { 0, 1, 2, 2, 3, 0 };
	Mesh ground(ground_v, ground_i, mat_h);
	ground.compute_tangents();
	MeshHandle ground_mh = scene.add_mesh(ground);
	(void)ground_mh;

	// A tilted triangle
	std::vector<Vertex> tri_v = {
		Vertex(Vec3(-0.8f, 0.2f, 0.0f), Vec3(0, 0.8f, 0.6f), Vec2(0, 0)),
		Vertex(Vec3(0.8f, 0.2f, 0.0f), Vec3(0, 0.8f, 0.6f), Vec2(1, 0)),
		Vertex(Vec3(0.0f, 1.2f, 0.3f), Vec3(0, 0.8f, 0.6f), Vec2(0.5f, 1)),
	};
	std::vector<uint32_t> tri_i = { 0, 1, 2 };
	Mesh tri_mesh(tri_v, tri_i, mat_h);
	tri_mesh.compute_tangents();
	scene.add_mesh(tri_mesh);

	// Lights
	auto sun = std::make_shared<DirectionalLight>(Vec3(-1, -1, -0.5f), Vec3(1.0f), 2.0f);
	scene.add_light(sun);

	auto point = std::make_shared<PointLight>(Vec3(0, 2.5f, 1.5f), Vec3(1.0f, 0.95f, 0.8f), 10.0f, 10.0f);
	point->set_cast_shadows(true);
	scene.add_light(point);

	// Camera
	Camera camera(Vec3(0.0f, 1.8f, 4.5f), Vec3(0.0f, 0.6f, 0.0f));
	camera.set_perspective(45.0f, static_cast<Real>(fb_w) / static_cast<Real>(fb_h), 0.1f, 200.0f);

	// Build BVH from scene meshes
	std::vector<Triangle> triangles;
	for (const auto &m : scene.get_all_meshes()) {
		auto tris = mesh_to_triangles(m, m.get_material());
		triangles.insert(triangles.end(), tris.begin(), tris.end());
	}

	BVH bvh;
	if (!bvh.build(triangles)) {
		ARE_LOG_CRITICAL("phase5_test: BVH build failed");
		return -1;
	}

	// Ray tracer config
	RayTracingConfig rtc;
	rtc.backend = RayTracingBackend::ARE_RT_BACKEND_CPU;
	rtc.spp = 1;
	rtc.max_depth = 4;
	rtc.enable_gi = true;
	rtc.enable_ao = true;
	rtc.ao_samples = 8;
	rtc.ao_radius = 1.0f;

	CPURayTracer tracer(rtc);
	tracer.update_bvh(bvh);

	ARE_LOG_INFO("Controls:");
	ARE_LOG_INFO("  1: spp = 1");
	ARE_LOG_INFO("  2: spp = 16");
	ARE_LOG_INFO("  A: toggle AO");
	ARE_LOG_INFO("  G: toggle GI");
	ARE_LOG_INFO("  ESC: exit");

	bool request_render = true;

	while (!window.should_close()) {
		window.poll_events();

		if (window.is_key_pressed(256)) {
			window.set_should_close(true);
		}

		if (window.is_key_pressed(49)) { // 1
			rtc.spp = 1;
			tracer.set_config(rtc);
			request_render = true;
		}
		if (window.is_key_pressed(50)) { // 2
			rtc.spp = 16;
			tracer.set_config(rtc);
			request_render = true;
		}
		if (window.is_key_pressed(65)) { // A
			rtc.enable_ao = !rtc.enable_ao;
			tracer.set_config(rtc);
			request_render = true;
		}
		if (window.is_key_pressed(71)) { // G
			rtc.enable_gi = !rtc.enable_gi;
			tracer.set_config(rtc);
			request_render = true;
		}

		// Handle resize
		int new_fb_w = 0;
		int new_fb_h = 0;
		window.get_framebuffer_size(new_fb_w, new_fb_h);
		if (new_fb_w != fb_w || new_fb_h != fb_h) {
			fb_w = new_fb_w;
			fb_h = new_fb_h;
			rasterizer.resize(fb_w, fb_h);

			glDeleteTextures(1, &output_tex);
			output_tex = create_output_texture_rgba16f(fb_w, fb_h);

			request_render = true;
		}

		if (request_render) {
			ARE_PROFILE_SCOPE("CPU Ray Trace");
			tracer.render(scene, camera, &gbuffer, output_tex);
			request_render = false;
		}

		// Present
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

	Profiler::print_results();
	Profiler::shutdown();
	Logger::shutdown();
	return 0;
}
