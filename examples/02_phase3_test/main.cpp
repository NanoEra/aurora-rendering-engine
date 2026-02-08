/**
 * @file main.cpp
 * @brief Phase 3 verification program - G-Buffer rendering test
 */

#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/geometry/vertex.h>
#include <are/platform/gl_context.h>
#include <are/platform/window.h>
#include <are/rasterizer/gbuffer.h>
#include <are/rasterizer/rasterizer.h>
#include <are/rasterizer/shader_program.h>
#include <are/scene/camera.h>
#include <are/scene/material.h>
#include <are/scene/mesh.h>
#include <are/scene/scene_manager.h>
#include <are/utils/file_utils.h>

#include <cmath>
#include <iostream>
#include <vector>
#include "../lib/glad/glad/glad.h"
#include <GLFW/glfw3.h>

using namespace are;

/**
 * @brief Create a simple cube mesh
 */
Mesh create_cube_mesh() {
	std::vector<Vertex> vertices = {
		// Front face
		Vertex(Vec3(-0.5f, -0.5f, 0.5f), Vec3(0, 0, 1), Vec2(0, 0)),
		Vertex(Vec3(0.5f, -0.5f, 0.5f), Vec3(0, 0, 1), Vec2(1, 0)),
		Vertex(Vec3(0.5f, 0.5f, 0.5f), Vec3(0, 0, 1), Vec2(1, 1)),
		Vertex(Vec3(-0.5f, 0.5f, 0.5f), Vec3(0, 0, 1), Vec2(0, 1)),

		// Back face
		Vertex(Vec3(0.5f, -0.5f, -0.5f), Vec3(0, 0, -1), Vec2(0, 0)),
		Vertex(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0, 0, -1), Vec2(1, 0)),
		Vertex(Vec3(-0.5f, 0.5f, -0.5f), Vec3(0, 0, -1), Vec2(1, 1)),
		Vertex(Vec3(0.5f, 0.5f, -0.5f), Vec3(0, 0, -1), Vec2(0, 1)),

		// Top face
		Vertex(Vec3(-0.5f, 0.5f, 0.5f), Vec3(0, 1, 0), Vec2(0, 0)),
		Vertex(Vec3(0.5f, 0.5f, 0.5f), Vec3(0, 1, 0), Vec2(1, 0)),
		Vertex(Vec3(0.5f, 0.5f, -0.5f), Vec3(0, 1, 0), Vec2(1, 1)),
		Vertex(Vec3(-0.5f, 0.5f, -0.5f), Vec3(0, 1, 0), Vec2(0, 1)),

		// Bottom face
		Vertex(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0, -1, 0), Vec2(0, 0)),
		Vertex(Vec3(0.5f, -0.5f, -0.5f), Vec3(0, -1, 0), Vec2(1, 0)),
		Vertex(Vec3(0.5f, -0.5f, 0.5f), Vec3(0, -1, 0), Vec2(1, 1)),
		Vertex(Vec3(-0.5f, -0.5f, 0.5f), Vec3(0, -1, 0), Vec2(0, 1)),

		// Right face
		Vertex(Vec3(0.5f, -0.5f, 0.5f), Vec3(1, 0, 0), Vec2(0, 0)),
		Vertex(Vec3(0.5f, -0.5f, -0.5f), Vec3(1, 0, 0), Vec2(1, 0)),
		Vertex(Vec3(0.5f, 0.5f, -0.5f), Vec3(1, 0, 0), Vec2(1, 1)),
		Vertex(Vec3(0.5f, 0.5f, 0.5f), Vec3(1, 0, 0), Vec2(0, 1)),

		// Left face
		Vertex(Vec3(-0.5f, -0.5f, -0.5f), Vec3(-1, 0, 0), Vec2(0, 0)),
		Vertex(Vec3(-0.5f, -0.5f, 0.5f), Vec3(-1, 0, 0), Vec2(1, 0)),
		Vertex(Vec3(-0.5f, 0.5f, 0.5f), Vec3(-1, 0, 0), Vec2(1, 1)),
		Vertex(Vec3(-0.5f, 0.5f, -0.5f), Vec3(-1, 0, 0), Vec2(0, 1))
	};

	std::vector<uint32_t> indices = {
		// Front
		0, 1, 2, 2, 3, 0,
		// Back
		4, 5, 6, 6, 7, 4,
		// Top
		8, 9, 10, 10, 11, 8,
		// Bottom
		12, 13, 14, 14, 15, 12,
		// Right
		16, 17, 18, 18, 19, 16,
		// Left
		20, 21, 22, 22, 23, 20
	};

	return Mesh(vertices, indices);
}

/**
 * @brief Create a simple triangle mesh (positioned in front of cube)
 */
Mesh create_triangle_mesh() {
    std::vector<Vertex> vertices = {
        Vertex(Vec3(-0.5f, -0.5f, 1.0f), Vec3(0, 0, 1), Vec2(0, 0)),  // Z = 1.0
        Vertex(Vec3( 0.5f, -0.5f, 1.0f), Vec3(0, 0, 1), Vec2(1, 0)),  // Z = 1.0
        Vertex(Vec3( 0.0f,  0.5f, 1.0f), Vec3(0, 0, 1), Vec2(0.5f, 1)) // Z = 1.0
    };
    
    std::vector<uint32_t> indices = {0, 1, 2};
    
    return Mesh(vertices, indices);
}

/**
 * @brief Fullscreen quad shader for G-Buffer visualization
 */
const char *fullscreen_vert_source = R"(
#version 430 core
layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_texcoord;
out vec2 v_texcoord;
void main() {
    v_texcoord = a_texcoord;
    gl_Position = vec4(a_position, 0.0, 1.0);
}
)";

const char *visualize_frag_source = R"(
#version 430 core
in vec2 v_texcoord;
out vec4 frag_color;

uniform sampler2D u_texture;
uniform int u_mode; // 0=position, 1=normal, 2=albedo, 3=material

void main() {
    vec4 value = texture(u_texture, v_texcoord);
    
    if (u_mode == 0) {
        // Position: normalize to [0,1] range for visualization
        frag_color = vec4(value.xyz * 0.5 + 0.5, 1.0);
    } else if (u_mode == 1) {
        // Normal: normalize to [0,1] range
        frag_color = vec4(value.xyz * 0.5 + 0.5, 1.0);
    } else if (u_mode == 2) {
        // Albedo: direct output
        frag_color = vec4(value.rgb, 1.0);
    } else if (u_mode == 3) {
        // Material: roughness in R, AO in G
        frag_color = vec4(value.r, value.g, 0.0, 1.0);
    } else {
        frag_color = value;
    }
}
)";

/**
 * @brief Create fullscreen quad VAO
 */
uint32_t create_fullscreen_quad() {
	float vertices[] = {
		// Position    // Texcoord
		-1.0f, -1.0f, 0.0f, 0.0f,
		1.0f, -1.0f, 1.0f, 0.0f,
		1.0f, 1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, 0.0f, 1.0f
	};

	uint32_t indices[] = { 0, 1, 2, 2, 3, 0 };

	uint32_t vao, vbo, ebo;

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

	return vao;
}

int main() {
	// Initialize logger
	Logger::init(LogLevel::ARE_LOG_DEBUG);
	Profiler::init();

	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("Phase 3 Verification Program");
	ARE_LOG_INFO("G-Buffer Rendering Test");
	ARE_LOG_INFO("========================================");

	// Create window configuration
	WindowConfig window_config;
	window_config.width = 800;
	window_config.height = 600;
	window_config.title = "Phase 3 - G-Buffer Test";
	window_config.vsync = true;

	// Create window
	Window window(window_config);

	// Initialize OpenGL
	if (!GLContext::initialize()) {
		ARE_LOG_CRITICAL("Failed to initialize OpenGL context");
		return -1;
	}

	GLContext::print_info();

	// Get shader directory (relative to executable)
	std::string shader_dir = "shaders/";

	// Create rasterizer
	int fb_width, fb_height;
	window.get_framebuffer_size(fb_width, fb_height);
	Rasterizer rasterizer(fb_width, fb_height);

	// Initialize shaders
	// First, create G-Buffer shader manually for testing
	ShaderProgram gbuffer_shader;

	// Try to load from file first
	bool shader_loaded = false;
	if (file_exists(shader_dir + "gbuffer/gbuffer.vert") && file_exists(shader_dir + "gbuffer/gbuffer.frag")) {
		if (gbuffer_shader.load_shader(ShaderType::ARE_SHADER_VERTEX, shader_dir + "gbuffer/gbuffer.vert") && gbuffer_shader.load_shader(ShaderType::ARE_SHADER_FRAGMENT, shader_dir + "gbuffer/gbuffer.frag") && gbuffer_shader.link()) {
			shader_loaded = true;
			ARE_LOG_INFO("Loaded G-Buffer shaders from files");
		}
	}

	// Fallback to embedded shaders
	if (!shader_loaded) {
		ARE_LOG_WARN("Shader files not found, using embedded shaders");

		const char *gbuffer_vert = R"(
#version 430 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_texcoord;
layout(location = 3) in vec3 a_tangent;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;
uniform mat3 u_normal_matrix;

out vec3 v_world_position;
out vec3 v_world_normal;
out vec2 v_texcoord;
out vec3 v_world_tangent;

void main() {
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    v_world_position = world_pos.xyz;
    v_world_normal = normalize(u_normal_matrix * a_normal);
    v_world_tangent = normalize(u_normal_matrix * a_tangent);
    v_texcoord = a_texcoord;
    gl_Position = u_projection * u_view * world_pos;
}
)";

		const char *gbuffer_frag = R"(
#version 430 core
in vec3 v_world_position;
in vec3 v_world_normal;
in vec2 v_texcoord;
in vec3 v_world_tangent;

uniform vec3 u_albedo;
uniform float u_metallic;
uniform float u_roughness;

layout(location = 0) out vec3 g_position;
layout(location = 1) out vec3 g_normal;
layout(location = 2) out vec4 g_albedo_metallic;
layout(location = 3) out vec2 g_roughness_ao;

void main() {
    g_position = v_world_position;
    g_normal = normalize(v_world_normal);
    g_albedo_metallic = vec4(u_albedo, u_metallic);
    g_roughness_ao = vec2(u_roughness, 1.0);
}
)";

		if (!gbuffer_shader.compile_shader(ShaderType::ARE_SHADER_VERTEX, gbuffer_vert) || !gbuffer_shader.compile_shader(ShaderType::ARE_SHADER_FRAGMENT, gbuffer_frag) || !gbuffer_shader.link()) {
			ARE_LOG_CRITICAL("Failed to compile embedded G-Buffer shaders");
			return -1;
		}
	}

	// Create visualization shader
	ShaderProgram vis_shader;
	if (!vis_shader.compile_shader(ShaderType::ARE_SHADER_VERTEX, fullscreen_vert_source) || !vis_shader.compile_shader(ShaderType::ARE_SHADER_FRAGMENT, visualize_frag_source) || !vis_shader.link()) {
		ARE_LOG_CRITICAL("Failed to compile visualization shaders");
		return -1;
	}

	// Create fullscreen quad
	uint32_t fullscreen_quad_vao = create_fullscreen_quad();

	// Create scene
	SceneManager scene;

	// Create materials
	Material red_material;
	red_material.set_albedo(Vec3(0.8f, 0.2f, 0.2f));
	red_material.set_metallic(0.0f);
	red_material.set_roughness(0.5f);
	MaterialHandle red_mat_handle = scene.add_material(red_material);

	Material green_material;
	green_material.set_albedo(Vec3(0.2f, 0.8f, 0.2f));
	green_material.set_metallic(0.5f);
	green_material.set_roughness(0.3f);
	MaterialHandle green_mat_handle = scene.add_material(green_material);

	// Create meshes
	Mesh cube = create_cube_mesh();
	cube.set_material(red_mat_handle);
	cube.compute_tangents();

	Mesh triangle = create_triangle_mesh();
	triangle.set_material(green_mat_handle);
	triangle.compute_tangents();

	// Upload meshes to GPU
	rasterizer.upload_mesh(cube);
	rasterizer.upload_mesh(triangle);

	// Add meshes to scene
	scene.add_mesh(cube);
	scene.add_mesh(triangle);

	// Create camera
	Camera camera(Vec3(0, 0, 3), Vec3(0, 0, 0));
	camera.set_perspective(45.0f, static_cast<float>(fb_width) / fb_height, 0.1f, 100.0f);

	// Visualization mode (0=position, 1=normal, 2=albedo, 3=material)
	int vis_mode = 2; // Start with albedo

	ARE_LOG_INFO("Controls:");
	ARE_LOG_INFO("  1 - View Position buffer");
	ARE_LOG_INFO("  2 - View Normal buffer");
	ARE_LOG_INFO("  3 - View Albedo buffer");
	ARE_LOG_INFO("  4 - View Material buffer (Roughness/AO)");
	ARE_LOG_INFO("  ESC - Exit");

	// Main loop
	float time = 0.0f;
	while (!window.should_close()) {
		ARE_PROFILE_SCOPE("Frame");

		// Poll events
		window.poll_events();

		// Handle input
		if (window.is_key_pressed(256)) { // ESC
			window.set_should_close(true);
		}
		if (window.is_key_pressed(49))
			vis_mode = 0; // 1 - Position
		if (window.is_key_pressed(50))
			vis_mode = 1; // 2 - Normal
		if (window.is_key_pressed(51))
			vis_mode = 2; // 3 - Albedo
		if (window.is_key_pressed(52))
			vis_mode = 3; // 4 - Material

		// Update camera position (orbit around origin)
		time += 0.016f;
		float cam_x = std::sin(time * 0.5f) * 3.0f;
		float cam_z = std::cos(time * 0.5f) * 3.0f;
		camera.set_position(Vec3(cam_x, 1.5f, cam_z));
		camera.set_target(Vec3(0, 0, 0));

		// Render to G-Buffer
		{
			ARE_PROFILE_SCOPE("Render G-Buffer");

			// Manually render since we're using our own shader
			GBuffer &gbuffer = rasterizer.get_gbuffer();
			gbuffer.bind();

			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glEnable(GL_DEPTH_TEST);
			glEnable(GL_CULL_FACE);

			gbuffer_shader.use();
			gbuffer_shader.set_uniform("u_view", camera.get_view_matrix());
			gbuffer_shader.set_uniform("u_projection", camera.get_projection_matrix());

			// Render all meshes
			const auto &meshes = scene.get_all_meshes();
			const auto &materials = scene.get_all_materials();

			for (const auto &mesh : meshes) {
				if (!mesh.has_gpu_resources())
					continue;

				Mat4 model = Mat4(1.0f);
				gbuffer_shader.set_uniform("u_model", model);

				Mat3 normal_matrix = glm::transpose(glm::inverse(Mat3(model)));
				gbuffer_shader.set_uniform("u_normal_matrix", normal_matrix);

				MaterialHandle mat_handle = mesh.get_material();
				if (mat_handle != are_invalid_handle && mat_handle <= materials.size()) {
					const Material &mat = materials[mat_handle - 1];
					gbuffer_shader.set_uniform("u_albedo", mat.get_albedo());
					gbuffer_shader.set_uniform("u_metallic", mat.get_metallic());
					gbuffer_shader.set_uniform("u_roughness", mat.get_roughness());
				} else {
					gbuffer_shader.set_uniform("u_albedo", Vec3(0.8f));
					gbuffer_shader.set_uniform("u_metallic", 0.0f);
					gbuffer_shader.set_uniform("u_roughness", 0.5f);
				}

				glBindVertexArray(mesh.get_vao());
				glDrawElements(GL_TRIANGLES,
					static_cast<GLsizei>(mesh.get_index_count()),
					GL_UNSIGNED_INT,
					nullptr);
			}

			glBindVertexArray(0);
			glDisable(GL_CULL_FACE);
			glDisable(GL_DEPTH_TEST);

			gbuffer.unbind();
		}

		// Visualize G-Buffer
		{
			ARE_PROFILE_SCOPE("Visualize G-Buffer");

			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glViewport(0, 0, fb_width, fb_height);
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			vis_shader.use();
			vis_shader.set_uniform("u_mode", vis_mode);
			vis_shader.set_uniform("u_texture", 0);

			// Bind appropriate G-Buffer texture
			GBuffer &gbuffer = rasterizer.get_gbuffer();
			gbuffer.bind_texture(vis_mode, 0);

			glBindVertexArray(fullscreen_quad_vao);
			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
			glBindVertexArray(0);
		}

		// Swap buffers
		window.swap_buffers();
	}

	// Cleanup
	glDeleteVertexArrays(1, &fullscreen_quad_vao);

	// Print profiling results
	Profiler::print_results();

	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("Phase 3 test completed successfully!");
	ARE_LOG_INFO("========================================");

	Profiler::shutdown();
	Logger::shutdown();

	return 0;
}
