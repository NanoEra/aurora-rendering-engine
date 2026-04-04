#include <core/renderer.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <memory>
#include <scene/camera.h>
#include <scene/light.h>
#include <scene/material.h>
#include <scene/mesh.h>
#include <scene/scene.h>
#include <utils/logger.h>

using namespace are;

// Window dimensions
const uint WINDOW_WIDTH = 800;
const uint WINDOW_HEIGHT = 800;

// Global state
GLFWwindow *g_window = nullptr;
std::unique_ptr<Renderer> g_renderer = nullptr;
std::unique_ptr<Scene> g_scene = nullptr;
std::shared_ptr<Camera> g_camera = nullptr; // Keep a direct reference to camera

// --- Camera Control State ---
Vec3 g_cameraPos = Vec3(0.0f, 0.0f, 4.5f);
Vec3 g_cameraTarget = Vec3(0.0f, 0.0f, 0.0f);
Vec3 g_cameraUp = Vec3(0.0f, 1.0f, 0.0f);
Vec3 g_worldUp = Vec3(0.0f, 1.0f, 0.0f);

// Euler Angles
float g_yaw = -90.0f; // Initialized to look along -Z (standard OpenGL)
float g_pitch = 0.0f;

// Control settings
float g_moveSpeed = 2.5f;
float g_mouseSensitivity = 0.1f;
bool g_firstMouse = true;
double g_lastX = WINDOW_WIDTH / 2.0;
double g_lastY = WINDOW_HEIGHT / 2.0;

// Time
float g_deltaTime = 0.0f;
float g_lastFrame = 0.0f;

// GLFW error callback
void glfw_error_callback(int error, const char *description) {
	ARE_LOG_ERROR("GLFW Error " + std::to_string(error) + ": " + std::string(description));
}

/// @brief Create a quad mesh
std::shared_ptr<Mesh> create_quad(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3,
	const Vec3 &normal, uint material_id) {
	auto mesh = std::make_shared<Mesh>();

	std::vector<Vertex> vertices = {
		{ v0, normal, Vec2(0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f) },
		{ v1, normal, Vec2(1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f) },
		{ v2, normal, Vec2(1.0f, 1.0f), Vec3(1.0f, 0.0f, 0.0f) },
		{ v3, normal, Vec2(0.0f, 1.0f), Vec3(1.0f, 0.0f, 0.0f) }
	};

	std::vector<uint> indices = { 0, 1, 2, 0, 2, 3 };

	mesh->set_vertices(vertices);
	mesh->set_indices(indices);
	mesh->set_material(material_id);

	return mesh;
}

/// @brief Create a box mesh
std::shared_ptr<Mesh> create_box(const Vec3 &min, const Vec3 &max, uint material_id) {
	auto mesh = std::make_shared<Mesh>();

	std::vector<Vertex> vertices = {
		// Front face
		{ { min.x, min.y, max.z }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, min.y, max.z }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, max.y, max.z }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { min.x, max.y, max.z }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },

		// Back face
		{ { max.x, min.y, min.z }, { 0.0f, 0.0f, -1.0f }, { 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } },
		{ { min.x, min.y, min.z }, { 0.0f, 0.0f, -1.0f }, { 1.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } },
		{ { min.x, max.y, min.z }, { 0.0f, 0.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 0.0f, 0.0f } },
		{ { max.x, max.y, min.z }, { 0.0f, 0.0f, -1.0f }, { 0.0f, 1.0f }, { -1.0f, 0.0f, 0.0f } },

		// Top face
		{ { min.x, max.y, max.z }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, max.y, max.z }, { 0.0f, 1.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, max.y, min.z }, { 0.0f, 1.0f, 0.0f }, { 1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { min.x, max.y, min.z }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },

		// Bottom face
		{ { min.x, min.y, min.z }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, min.y, min.z }, { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { max.x, min.y, max.z }, { 0.0f, -1.0f, 0.0f }, { 1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },
		{ { min.x, min.y, max.z }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } },

		// Right face
		{ { max.x, min.y, max.z }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } },
		{ { max.x, min.y, min.z }, { 1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } },
		{ { max.x, max.y, min.z }, { 1.0f, 0.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 0.0f, -1.0f } },
		{ { max.x, max.y, max.z }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f } },

		// Left face
		{ { min.x, min.y, min.z }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
		{ { min.x, min.y, max.z }, { -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } },
		{ { min.x, max.y, max.z }, { -1.0f, 0.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 0.0f, 1.0f } },
		{ { min.x, max.y, min.z }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f }, { 0.0f, 0.0f, 1.0f } }
	};

	std::vector<uint> indices = {
		0, 1, 2, 0, 2, 3, // Front
		4, 5, 6, 4, 6, 7, // Back
		8, 9, 10, 8, 10, 11, // Top
		12, 13, 14, 12, 14, 15, // Bottom
		16, 17, 18, 16, 18, 19, // Right
		20, 21, 22, 20, 22, 23 // Left
	};

	mesh->set_vertices(vertices);
	mesh->set_indices(indices);
	mesh->set_material(material_id);

	return mesh;
}

/// @brief Create a sphere mesh
std::shared_ptr<Mesh> create_sphere(float radius, uint segments, uint rings, uint material_id) {
	auto mesh = std::make_shared<Mesh>();

	std::vector<Vertex> vertices;
	std::vector<uint> indices;

	for (uint ring = 0; ring <= rings; ++ring) {
		float theta = ring * glm::pi<float>() / rings;
		float sin_theta = sin(theta);
		float cos_theta = cos(theta);

		for (uint seg = 0; seg <= segments; ++seg) {
			float phi = seg * 2.0f * glm::pi<float>() / segments;
			float x = cos(phi) * sin_theta;
			float y = cos_theta;
			float z = sin(phi) * sin_theta;

			Vec3 pos = Vec3(x, y, z) * radius;
			Vec3 normal = Vec3(x, y, z);
			Vec2 uv = Vec2((float)seg / segments, (float)ring / rings);
			Vec3 tangent = Vec3(-sin(phi), 0.0f, cos(phi));

			vertices.push_back({ pos, normal, uv, tangent });
		}
	}

	for (uint ring = 0; ring < rings; ++ring) {
		for (uint seg = 0; seg < segments; ++seg) {
			uint current = ring * (segments + 1) + seg;
			indices.push_back(current);
			indices.push_back(current + segments + 1);
			indices.push_back(current + 1);
			indices.push_back(current + 1);
			indices.push_back(current + segments + 1);
			indices.push_back(current + segments + 2);
		}
	}

	mesh->set_vertices(vertices);
	mesh->set_indices(indices);
	mesh->set_material(material_id);
	mesh->compute_tangents();

	return mesh;
}

/// @brief Setup Cornell Box scene with metal sphere
void setup_cornell_box() {
	g_scene = std::make_unique<Scene>();

	// Create materials
	// 0: White diffuse
	auto white_material = std::make_shared<Material>();
	white_material->set_albedo(Vec3(0.73f, 0.73f, 0.73f));
	white_material->set_type(MaterialType::DIFFUSE);
	uint white_id = g_scene->add_material(white_material);

	// 1: Red diffuse (left wall)
	auto red_material = std::make_shared<Material>();
	red_material->set_albedo(Vec3(0.65f, 0.05f, 0.05f));
	red_material->set_type(MaterialType::DIFFUSE);
	uint red_id = g_scene->add_material(red_material);

	// 2: Green diffuse (right wall)
	auto green_material = std::make_shared<Material>();
	green_material->set_albedo(Vec3(0.12f, 0.45f, 0.15f));
	green_material->set_type(MaterialType::DIFFUSE);
	uint green_id = g_scene->add_material(green_material);

	// 3: Light emissive
	auto light_material = std::make_shared<Material>();
	light_material->set_albedo(Vec3(1.0f, 1.0f, 1.0f));
	light_material->set_emission(Vec3(15.0f, 15.0f, 15.0f));
	light_material->set_type(MaterialType::EMISSIVE);
	uint light_id = g_scene->add_material(light_material);

	// 4: Metal (shiny gold-like)
	auto metal_material = std::make_shared<Material>();
	metal_material->set_albedo(Vec3(0.95f, 0.93f, 0.88f));
	metal_material->set_metallic(1.0f);
	metal_material->set_roughness(0.05f);
	metal_material->set_type(MaterialType::METAL);
	uint metal_id = g_scene->add_material(metal_material);

	// 5: Yellow emissive sphere
	auto emissive_sphere_mat = std::make_shared<Material>();
	emissive_sphere_mat->set_albedo(Vec3(1.0f, 0.8f, 0.2f));
	emissive_sphere_mat->set_emission(Vec3(5.0f, 4.0f, 1.0f));
	emissive_sphere_mat->set_type(MaterialType::EMISSIVE);
	uint emissive_sphere_id = g_scene->add_material(emissive_sphere_mat);
	(void)emissive_sphere_id; // Reserved for future use

	// Create room (Cornell Box)
	float room_size = 2.0f;

	// Floor (white)
	auto floor = create_quad(
		Vec3(-room_size, -room_size, -room_size),
		Vec3(room_size, -room_size, -room_size),
		Vec3(room_size, -room_size, room_size),
		Vec3(-room_size, -room_size, room_size),
		Vec3(0.0f, 1.0f, 0.0f),
		white_id);
	floor->upload_to_gpu();
	g_scene->add_mesh(floor);

	// Ceiling (white)
	auto ceiling = create_quad(
		Vec3(-room_size, room_size, room_size),
		Vec3(room_size, room_size, room_size),
		Vec3(room_size, room_size, -room_size),
		Vec3(-room_size, room_size, -room_size),
		Vec3(0.0f, -1.0f, 0.0f),
		white_id);
	ceiling->upload_to_gpu();
	g_scene->add_mesh(ceiling);

	// Back wall (white)
	auto back_wall = create_quad(
		Vec3(-room_size, -room_size, -room_size),
		Vec3(-room_size, room_size, -room_size),
		Vec3(room_size, room_size, -room_size),
		Vec3(room_size, -room_size, -room_size),
		Vec3(0.0f, 0.0f, 1.0f),
		white_id);
	back_wall->upload_to_gpu();
	g_scene->add_mesh(back_wall);

	// Left wall (red)
	auto left_wall = create_quad(
		Vec3(-room_size, -room_size, room_size),
		Vec3(-room_size, room_size, room_size),
		Vec3(-room_size, room_size, -room_size),
		Vec3(-room_size, -room_size, -room_size),
		Vec3(1.0f, 0.0f, 0.0f),
		red_id);
	left_wall->upload_to_gpu();
	g_scene->add_mesh(left_wall);

	// Right wall (green)
	auto right_wall = create_quad(
		Vec3(room_size, -room_size, -room_size),
		Vec3(room_size, room_size, -room_size),
		Vec3(room_size, room_size, room_size),
		Vec3(room_size, -room_size, room_size),
		Vec3(-1.0f, 0.0f, 0.0f),
		green_id);
	right_wall->upload_to_gpu();
	g_scene->add_mesh(right_wall);

	// Area light on ceiling
	float light_size = 0.5f;
	auto area_light = create_quad(
		Vec3(-light_size, room_size - 0.01f, -light_size),
		Vec3(light_size, room_size - 0.01f, -light_size),
		Vec3(light_size, room_size - 0.01f, light_size),
		Vec3(-light_size, room_size - 0.01f, light_size),
		Vec3(0.0f, -1.0f, 0.0f),
		light_id);
	area_light->upload_to_gpu();
	g_scene->add_mesh(area_light);

	// Tall box (white, left side)
	auto tall_box = create_box(Vec3(-0.7f, -room_size, -0.7f), Vec3(-0.2f, 0.6f, -0.2f), white_id);
	tall_box->upload_to_gpu();
	g_scene->add_mesh(tall_box);

	// Metal sphere (replacing the glass box, positioned on the right side)
	auto metal_sphere = create_sphere(0.5f, 16, 8, /*metal_id*/white_id);
	metal_sphere->set_position(Vec3(0.55f, -1.5f, 0.35f));
	metal_sphere->upload_to_gpu();
	g_scene->add_mesh(metal_sphere);

	// Setup camera
	g_camera = std::make_shared<Camera>();
	g_camera->set_position(g_cameraPos);
	g_camera->set_target(g_cameraTarget);
	g_camera->set_up(g_cameraUp);
	g_camera->set_perspective(45.0f, static_cast<float>(WINDOW_WIDTH) / WINDOW_HEIGHT, 0.1f, 100.0f);
	g_scene->set_camera(g_camera);

	// Add point light
	auto light = std::make_shared<Light>();
	light->set_type(LightType::POINT);
	light->set_position(Vec3(0.0f, 1.8f, 0.0f));
	light->set_color(Vec3(1.0f, 1.0f, 1.0f));
	light->set_intensity(10.0f);
	light->set_range(10.0f);
	g_scene->add_light(light);

	ARE_LOG_INFO("Cornell Box with Metal Sphere scene created");
}

/// @brief Initialize GLFW and create window
bool init_window() {
	glfwSetErrorCallback(glfw_error_callback);

	if (!glfwInit()) {
		ARE_LOG_ERROR("Failed to initialize GLFW");
		return false;
	}

	ARE_LOG_INFO("GLFW initialized successfully");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_SAMPLES, 0);

	g_window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Aurora - Cornell Box (Metal Sphere)", nullptr, nullptr);

	if (!g_window) {
		ARE_LOG_ERROR("Failed to create GLFW window");
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(g_window);
	glfwSwapInterval(1);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		ARE_LOG_ERROR("Failed to initialize GLAD");
		return false;
	}

	return true;
}

// --- Input Processing ---
void process_input() {
	// Calculate delta time
	float currentFrame = glfwGetTime();
	g_deltaTime = currentFrame - g_lastFrame;
	g_lastFrame = currentFrame;

	float velocity = g_moveSpeed * g_deltaTime;
	bool camera_changed = false;

	// 1. Mouse Rotation (Left Button Hold)
	if (glfwGetMouseButton(g_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
		double xpos, ypos;
		glfwGetCursorPos(g_window, &xpos, &ypos);

		if (g_firstMouse) {
			g_lastX = xpos;
			g_lastY = ypos;
			g_firstMouse = false;
		}

		float xoffset = xpos - g_lastX;
		float yoffset = g_lastY - ypos; // Reversed since y-coordinates go from bottom to top
		g_lastX = xpos;
		g_lastY = ypos;

		// Only update if mouse actually moved
		if (xoffset != 0.0f || yoffset != 0.0f) {
			xoffset *= g_mouseSensitivity;
			yoffset *= g_mouseSensitivity;

			g_yaw += xoffset;
			g_pitch += yoffset;

			// Constrain pitch
			if (g_pitch > 89.0f)
				g_pitch = 89.0f;
			if (g_pitch < -89.0f)
				g_pitch = -89.0f;

			camera_changed = true;
		}
	} else {
		g_firstMouse = true; // Reset when released
	}

	// 2. Calculate Direction Vectors
	glm::vec3 front;
	front.x = cos(glm::radians(g_yaw)) * cos(glm::radians(g_pitch));
	front.y = sin(glm::radians(g_pitch));
	front.z = sin(glm::radians(g_yaw)) * cos(glm::radians(g_pitch));
	glm::vec3 frontNorm = glm::normalize(front);

	glm::vec3 rightNorm = glm::normalize(glm::cross(frontNorm, glm::vec3(g_worldUp.x, g_worldUp.y, g_worldUp.z)));

	// 3. Keyboard Movement (WASD)
	glm::vec3 pos = glm::vec3(g_cameraPos.x, g_cameraPos.y, g_cameraPos.z);

	if (glfwGetKey(g_window, GLFW_KEY_W) == GLFW_PRESS) {
		pos += frontNorm * velocity;
		camera_changed = true;
	}
	if (glfwGetKey(g_window, GLFW_KEY_S) == GLFW_PRESS) {
		pos -= frontNorm * velocity;
		camera_changed = true;
	}
	if (glfwGetKey(g_window, GLFW_KEY_A) == GLFW_PRESS) {
		pos -= rightNorm * velocity;
		camera_changed = true;
	}
	if (glfwGetKey(g_window, GLFW_KEY_D) == GLFW_PRESS) {
		pos += rightNorm * velocity;
		camera_changed = true;
	}

	// 4. Apply changes to Scene Camera and Notify Renderer
	if (camera_changed) {
		g_cameraPos = Vec3(pos.x, pos.y, pos.z);

		// Target = Position + Front
		Vec3 newTarget = g_cameraPos + Vec3(frontNorm.x, frontNorm.y, frontNorm.z);

		g_camera->set_position(g_cameraPos);
		g_camera->set_target(newTarget);

		// CRITICAL: Notify renderer to reset accumulation
		g_renderer->notify_scene_changed(*g_scene);
	}
}

/// @brief Main render loop
void render_loop() {
	ARE_LOG_INFO("Entering render loop...");

	int frame_count = 0;
	double fps_time = glfwGetTime();
	g_lastFrame = glfwGetTime(); // Initialize for delta time

	while (!glfwWindowShouldClose(g_window)) {
		// Process input at the start of the frame
		process_input();

		// Render
		RenderStats stats = g_renderer->render(*g_scene);

		// Swap buffers
		glfwSwapBuffers(g_window);
		glfwPollEvents();

		// Calculate FPS
		frame_count++;
		double current_time = glfwGetTime();
		double delta = current_time - fps_time;

		if (delta >= 1.0) {
			double fps = frame_count / delta;
			std::string title = "Aurora - Cornell Box (Metal Sphere) | FPS: " + std::to_string((int)fps) + " | Frame: " + std::to_string((int)stats.frame_time_ms_) + "ms";
			glfwSetWindowTitle(g_window, title.c_str());

			frame_count = 0;
			fps_time = current_time;
		}

		// ESC to exit
		if (glfwGetKey(g_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
			glfwSetWindowShouldClose(g_window, true);
		}
	}

	ARE_LOG_INFO("Exiting render loop");
}

/// @brief Cleanup
void cleanup() {
	ARE_LOG_INFO("Cleaning up...");

	if (g_renderer) {
		g_renderer->shutdown();
		g_renderer.reset();
	}

	g_scene.reset();

	if (g_window) {
		glfwDestroyWindow(g_window);
		glfwTerminate();
	}

	ARE_LOG_INFO("Cleanup complete");
}

int main() {
	ARE_LOG_INFO("===========================================");
	ARE_LOG_INFO("Aurora Rendering Engine - Cornell Box with Metal Sphere Demo");
	ARE_LOG_INFO("===========================================");

	if (!init_window()) {
		cleanup();
		ARE_LOG_ERROR("Failed to initialize window");
		Logger::shutdown();
		return -1;
	}

	ARE_LOG_INFO("Setting up Cornell Box with Metal Sphere scene...");
	setup_cornell_box();

	ARE_LOG_INFO("Initializing renderer...");
	RendererConfig config;
	config.width_ = WINDOW_WIDTH;
	config.height_ = WINDOW_HEIGHT;
	config.samples_per_pixel_ = 1;
	config.max_ray_depth_ = 4;
	config.enable_accumulation_ = true;
	config.enable_denoising_ = false;

	g_renderer = std::make_unique<Renderer>(config);
	if (!g_renderer->initialize()) {
		ARE_LOG_ERROR("Failed to initialize renderer");
		cleanup();
		Logger::shutdown();
		return -1;
	}

	ARE_LOG_INFO("===========================================");
	ARE_LOG_INFO("Renderer initialized successfully!");
	ARE_LOG_INFO("Controls:");
	ARE_LOG_INFO("  WASD - Move Camera");
	ARE_LOG_INFO("  Hold Left Mouse Button - Rotate Camera");
	ARE_LOG_INFO("  ESC - Exit");
	ARE_LOG_INFO("===========================================");

	render_loop();
	cleanup();

	ARE_LOG_INFO("Cornell Box with Metal Sphere demo finished");
	Logger::shutdown();

	return 0;
}
