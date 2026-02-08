/**
 * @file main.cpp
 * @brief Visual verification using software rasterization
 */

#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/geometry/triangle.h>
#include <are/geometry/vertex.h>
#include <are/raytracer/hit_record.h>
#include <are/raytracer/ray.h>
#include <are/scene/camera.h>
#include <are/scene/material.h>
#include <are/scene/mesh.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../lib/stb/stb_image_write.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace are;

// Simple framebuffer
struct Framebuffer {
	int width;
	int height;
	std::vector<uint8_t> pixels; // RGB format

	Framebuffer(int w, int h) : width(w), height(h) {
		pixels.resize(w * h * 3, 0);
	}

	void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
		if (x < 0 || x >= width || y < 0 || y >= height)
			return;
		int index = (y * width + x) * 3;
		pixels[index + 0] = r;
		pixels[index + 1] = g;
		pixels[index + 2] = b;
	}

	void set_pixel(int x, int y, const Vec3 &color) {
		uint8_t r = static_cast<uint8_t>(std::min(color.x * 255.0f, 255.0f));
		uint8_t g = static_cast<uint8_t>(std::min(color.y * 255.0f, 255.0f));
		uint8_t b = static_cast<uint8_t>(std::min(color.z * 255.0f, 255.0f));
		set_pixel(x, y, r, g, b);
	}

	bool save(const std::string &filename) {
		return stbi_write_png(filename.c_str(), width, height, 3,
				   pixels.data(), width * 3)
			!= 0;
	}
};

// Simple shading function
Vec3 shade_hit(const HitRecord &hit, const Vec3 &light_dir) {
	// Lambertian shading
	float ndotl = std::max(0.0f, glm::dot(hit.normal_, light_dir));

	// Base color based on normal (for visualization)
	Vec3 base_color = (hit.normal_ + Vec3(1.0f)) * 0.5f;

	// Apply lighting
	Vec3 ambient = base_color * 0.2f;
	Vec3 diffuse = base_color * ndotl * 0.8f;

	return ambient + diffuse;
}

// Render a single triangle
void render_triangle(Framebuffer &fb, const Triangle &tri, Camera &camera) {
	Vec3 light_dir = glm::normalize(Vec3(0.5f, 1.0f, 0.5f));

	for (int y = 0; y < fb.height; ++y) {
		for (int x = 0; x < fb.width; ++x) {
			// Generate ray
			float u = (x + 0.5f) / fb.width;
			float v = (y + 0.5f) / fb.height;

			Vec3 origin, direction;
			camera.generate_ray(u, v, origin, direction);
			Ray ray(origin, direction);

			// Test intersection
			HitRecord hit;
			if (tri.intersect(ray, hit)) {
				Vec3 color = shade_hit(hit, light_dir);
				fb.set_pixel(x, y, color);
			} else {
				// Background gradient
				Vec3 bg_color = Vec3(0.5f, 0.7f, 1.0f) * (1.0f - v) + Vec3(1.0f, 1.0f, 1.0f) * v;
				fb.set_pixel(x, y, bg_color);
			}
		}
	}
}

// Render multiple triangles (mesh)
void render_mesh(Framebuffer &fb, const Mesh &mesh, Camera &camera) {
	Vec3 light_dir = glm::normalize(Vec3(0.5f, 1.0f, 0.5f));

	for (int y = 0; y < fb.height; ++y) {
		for (int x = 0; x < fb.width; ++x) {
			// Generate ray
			float u = (x + 0.5f) / fb.width;
			float v = (y + 0.5f) / fb.height;

			Vec3 origin, direction;
			camera.generate_ray(u, v, origin, direction);
			Ray ray(origin, direction);

			// Test intersection with all triangles
			HitRecord closest_hit;
			closest_hit.t_ = ray.t_max_;
			bool hit_any = false;

			for (size_t i = 0; i < mesh.get_triangle_count(); ++i) {
				Vertex v0, v1, v2;
				if (mesh.get_triangle(i, v0, v1, v2)) {
					Triangle tri(v0, v1, v2);
					HitRecord hit;
					if (tri.intersect(ray, hit) && hit.t_ < closest_hit.t_) {
						closest_hit = hit;
						hit_any = true;
					}
				}
			}

			if (hit_any) {
				Vec3 color = shade_hit(closest_hit, light_dir);
				fb.set_pixel(x, y, color);
			} else {
				// Background gradient
				Vec3 bg_color = Vec3(0.5f, 0.7f, 1.0f) * (1.0f - v) + Vec3(1.0f, 1.0f, 1.0f) * v;
				fb.set_pixel(x, y, bg_color);
			}
		}
	}
}

int main() {
	Logger::init(LogLevel::ARE_LOG_INFO);

	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("Phase 2 Visual Verification");
	ARE_LOG_INFO("========================================");

	const int width = 800;
	const int height = 600;

	// Test 1: Single triangle
	{
		ARE_LOG_INFO("Rendering single triangle...");

		Framebuffer fb(width, height);

		// Create triangle
		Vertex v0(Vec3(-1, -1, 0), Vec3(0, 0, 1));
		Vertex v1(Vec3(1, -1, 0), Vec3(0, 0, 1));
		Vertex v2(Vec3(0, 1, 0), Vec3(0, 0, 1));
		Triangle tri(v0, v1, v2);

		// Setup camera
		Camera camera(Vec3(0, 0, 3), Vec3(0, 0, 0));
		camera.set_perspective(45.0f, (float)width / height, 0.1f, 100.0f);

		// Render
		render_triangle(fb, tri, camera);

		// Save
		if (fb.save("output_triangle.png")) {
			ARE_LOG_INFO("✓ Saved: output_triangle.png");
		} else {
			ARE_LOG_ERROR("✗ Failed to save output_triangle.png");
		}
	}

	// Test 2: Colored triangle (using normals)
	{
		ARE_LOG_INFO("Rendering colored triangle...");

		Framebuffer fb(width, height);

		// Create triangle with different normals for each vertex
		Vertex v0(Vec3(-1, -1, 0), Vec3(1, 0, 0)); // Red
		Vertex v1(Vec3(1, -1, 0), Vec3(0, 1, 0)); // Green
		Vertex v2(Vec3(0, 1, 0), Vec3(0, 0, 1)); // Blue
		Triangle tri(v0, v1, v2);

		Camera camera(Vec3(0, 0, 3), Vec3(0, 0, 0));
		camera.set_perspective(45.0f, (float)width / height, 0.1f, 100.0f);

		render_triangle(fb, tri, camera);

		if (fb.save("output_colored_triangle.png")) {
			ARE_LOG_INFO("✓ Saved: output_colored_triangle.png");
		} else {
			ARE_LOG_ERROR("✗ Failed to save output_colored_triangle.png");
		}
	}

	// Test 3: Cube (mesh with multiple triangles)
	{
		ARE_LOG_INFO("Rendering cube...");

		Framebuffer fb(width, height);

		// Create cube vertices
		std::vector<Vertex> vertices = {
			// Front face
			Vertex(Vec3(-1, -1, 1), Vec3(0, 0, 1)),
			Vertex(Vec3(1, -1, 1), Vec3(0, 0, 1)),
			Vertex(Vec3(1, 1, 1), Vec3(0, 0, 1)),
			Vertex(Vec3(-1, 1, 1), Vec3(0, 0, 1)),
			// Back face
			Vertex(Vec3(-1, -1, -1), Vec3(0, 0, -1)),
			Vertex(Vec3(1, -1, -1), Vec3(0, 0, -1)),
			Vertex(Vec3(1, 1, -1), Vec3(0, 0, -1)),
			Vertex(Vec3(-1, 1, -1), Vec3(0, 0, -1)),
		};

		// Create cube indices
		std::vector<uint32_t> indices = {
			// Front
			0, 1, 2, 2, 3, 0,
			// Right
			1, 5, 6, 6, 2, 1,
			// Back
			5, 4, 7, 7, 6, 5,
			// Left
			4, 0, 3, 3, 7, 4,
			// Top
			3, 2, 6, 6, 7, 3,
			// Bottom
			4, 5, 1, 1, 0, 4
		};

		Mesh cube(vertices, indices);

		// Setup camera (slightly angled view)
		Camera camera(Vec3(3, 2, 4), Vec3(0, 0, 0));
		camera.set_perspective(45.0f, (float)width / height, 0.1f, 100.0f);

		// Render
		render_mesh(fb, cube, camera);

		if (fb.save("output_cube.png")) {
			ARE_LOG_INFO("✓ Saved: output_cube.png");
		} else {
			ARE_LOG_ERROR("✗ Failed to save output_cube.png");
		}
	}

	// Test 4: Cornell Box (corrected)
	{
		ARE_LOG_INFO("Rendering Cornell Box...");

		Framebuffer fb(width, height);

		std::vector<Vertex> vertices;
		std::vector<uint32_t> indices;

		// Helper function to add a quad
		auto add_quad = [&](const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3, const Vec3 &normal) {
			unsigned int base = vertices.size();
			vertices.push_back(Vertex(v0, normal));
			vertices.push_back(Vertex(v1, normal));
			vertices.push_back(Vertex(v2, normal));
			vertices.push_back(Vertex(v3, normal));
			indices.insert(indices.end(), { base + 0, base + 1, base + 2, base + 2, base + 3, base + 0 });
		};

		// Floor (white)
		add_quad(
			Vec3(-2, -2, 2), Vec3(2, -2, 2),
			Vec3(2, -2, -2), Vec3(-2, -2, -2),
			Vec3(0, 1, 0));

		// Ceiling (white)
		add_quad(
			Vec3(-2, 2, -2), Vec3(2, 2, -2),
			Vec3(2, 2, 2), Vec3(-2, 2, 2),
			Vec3(0, -1, 0));

		// Back wall (white)
		add_quad(
			Vec3(-2, -2, -2), Vec3(2, -2, -2),
			Vec3(2, 2, -2), Vec3(-2, 2, -2),
			Vec3(0, 0, 1));

		// Left wall (red)
		add_quad(
			Vec3(-2, -2, 2), Vec3(-2, -2, -2),
			Vec3(-2, 2, -2), Vec3(-2, 2, 2),
			Vec3(1, 0, 0));

		// Right wall (green)
		add_quad(
			Vec3(2, -2, -2), Vec3(2, -2, 2),
			Vec3(2, 2, 2), Vec3(2, 2, -2),
			Vec3(-1, 0, 0));

		Mesh cornell_box(vertices, indices);

		Camera camera(Vec3(0, 0, 5), Vec3(0, 0, 0));
		camera.set_perspective(45.0f, (float)width / height, 0.1f, 100.0f);

		render_mesh(fb, cornell_box, camera);

		if (fb.save("output_cornell_box.png")) {
			ARE_LOG_INFO("✓ Saved: output_cornell_box.png");
		}
	}

	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("✓ All images generated successfully!");
	ARE_LOG_INFO("Check the following files:");
	ARE_LOG_INFO("  - output_triangle.png");
	ARE_LOG_INFO("  - output_colored_triangle.png");
	ARE_LOG_INFO("  - output_cube.png");
	ARE_LOG_INFO("  - output_cornell_box.png");
	ARE_LOG_INFO("========================================");

	Logger::shutdown();
	return 0;
}
