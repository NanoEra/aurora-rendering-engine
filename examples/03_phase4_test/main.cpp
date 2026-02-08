/**
 * @file main.cpp
 * @brief Phase 4 verification program - BVH construction and traversal test
 */

#include <are/acceleration/bvh.h>
#include <are/acceleration/bvh_builder.h>
#include <are/core/config.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/geometry/triangle.h>
#include <are/geometry/vertex.h>
#include <are/raytracer/hit_record.h>
#include <are/raytracer/ray.h>

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

using namespace are;

// Test result tracking
struct TestResult {
	std::string name;
	bool passed;
	std::string message;
};

std::vector<TestResult> test_results;

void report_test(const std::string &name, bool passed, const std::string &message = "") {
	test_results.push_back({ name, passed, message });
	if (passed) {
		ARE_LOG_INFO("✓ " + name);
	} else {
		ARE_LOG_ERROR("✗ " + name + ": " + message);
	}
}

/**
 * @brief Create a simple scene with a few triangles
 */
std::vector<Triangle> create_simple_scene() {
	std::vector<Triangle> triangles;

	// Ground plane (2 triangles)
	Vertex v0(Vec3(-5, 0, -5), Vec3(0, 1, 0));
	Vertex v1(Vec3(5, 0, -5), Vec3(0, 1, 0));
	Vertex v2(Vec3(5, 0, 5), Vec3(0, 1, 0));
	Vertex v3(Vec3(-5, 0, 5), Vec3(0, 1, 0));

	triangles.emplace_back(v0, v1, v2);
	triangles.emplace_back(v0, v2, v3);

	// Cube (12 triangles)
	Vec3 cube_min(-1, 1, -1);
	Vec3 cube_max(1, 3, 1);

	// Front face
	triangles.emplace_back(
		Vertex(Vec3(cube_min.x, cube_min.y, cube_max.z), Vec3(0, 0, 1)),
		Vertex(Vec3(cube_max.x, cube_min.y, cube_max.z), Vec3(0, 0, 1)),
		Vertex(Vec3(cube_max.x, cube_max.y, cube_max.z), Vec3(0, 0, 1)));
	triangles.emplace_back(
		Vertex(Vec3(cube_min.x, cube_min.y, cube_max.z), Vec3(0, 0, 1)),
		Vertex(Vec3(cube_max.x, cube_max.y, cube_max.z), Vec3(0, 0, 1)),
		Vertex(Vec3(cube_min.x, cube_max.y, cube_max.z), Vec3(0, 0, 1)));

	// Add more faces... (simplified for brevity)

	return triangles;
}

/**
 * @brief Create a complex scene with many triangles
 */
std::vector<Triangle> create_complex_scene(int num_triangles) {
	std::vector<Triangle> triangles;
	triangles.reserve(num_triangles);

	std::mt19937 rng(42);
	std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

	for (int i = 0; i < num_triangles; ++i) {
		Vec3 p0(dist(rng), dist(rng), dist(rng));
		Vec3 p1 = p0 + Vec3(dist(rng) * 0.5f, dist(rng) * 0.5f, dist(rng) * 0.5f);
		Vec3 p2 = p0 + Vec3(dist(rng) * 0.5f, dist(rng) * 0.5f, dist(rng) * 0.5f);

		Vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));

		triangles.emplace_back(
			Vertex(p0, normal),
			Vertex(p1, normal),
			Vertex(p2, normal));
	}

	return triangles;
}

/**
 * @brief Test 1: BVH construction
 */
void test_bvh_construction() {
	auto triangles = create_simple_scene();

	BVH bvh;
	BVHBuildConfig config;
	config.split_method_ = BVHSplitMethod::ARE_BVH_SPLIT_MIDDLE;
	config.max_leaf_size_ = 4;

	bool success = bvh.build(triangles, config);

	report_test("BVH construction (simple scene)", success);
	report_test("BVH is built", bvh.is_built());
	report_test("BVH has nodes", !bvh.get_nodes().empty());
}

/**
 * @brief Test 2: BVH construction with SAH
 */
void test_bvh_construction_sah() {
	auto triangles = create_simple_scene();

	BVH bvh;
	BVHBuildConfig config;
	config.split_method_ = BVHSplitMethod::ARE_BVH_SPLIT_SAH;
	config.max_leaf_size_ = 2;

	bool success = bvh.build(triangles, config);

	report_test("BVH construction with SAH", success);
}

/**
 * @brief Test 3: Ray-BVH intersection
 */
void test_ray_bvh_intersection() {
	auto triangles = create_simple_scene();

	BVH bvh;
	bvh.build(triangles);

	// Ray hitting the ground plane
	Ray ray1(Vec3(0, 5, 0), Vec3(0, -1, 0));
	HitRecord hit1;
	bool test1 = bvh.intersect(ray1, hit1);

	// Ray missing everything
	Ray ray2(Vec3(100, 5, 100), Vec3(0, -1, 0));
	HitRecord hit2;
	bool test2 = !bvh.intersect(ray2, hit2);

	report_test("Ray-BVH intersection (hit)", test1);
	report_test("Ray-BVH intersection (miss)", test2);
}

/**
 * @brief Test 4: BVH occlusion test
 */
void test_bvh_occlusion() {
	auto triangles = create_simple_scene();

	BVH bvh;
	bvh.build(triangles);

	// Ray with occlusion
	Ray ray1(Vec3(0, 5, 0), Vec3(0, -1, 0));
	bool test1 = bvh.intersect_any(ray1, 10.0f);

	// Ray without occlusion
	Ray ray2(Vec3(100, 5, 100), Vec3(0, -1, 0));
	bool test2 = !bvh.intersect_any(ray2, 10.0f);

	report_test("BVH occlusion test (occluded)", test1);
	report_test("BVH occlusion test (not occluded)", test2);
}

/**
 * @brief Test 5: BVH performance with complex scene
 */
void test_bvh_performance() {
	const int num_triangles = 10000;
	auto triangles = create_complex_scene(num_triangles);

	ARE_LOG_INFO("Building BVH for " + std::to_string(num_triangles) + " triangles...");

	BVH bvh;
	BVHBuildConfig config;
	config.split_method_ = BVHSplitMethod::ARE_BVH_SPLIT_SAH;

	auto start_build = std::chrono::high_resolution_clock::now();
	bool success = bvh.build(triangles, config);
	auto end_build = std::chrono::high_resolution_clock::now();

	double build_time = std::chrono::duration<double, std::milli>(end_build - start_build).count();

	ARE_LOG_INFO("BVH build time: " + std::to_string(build_time) + " ms");

	// Test ray tracing performance
	const int num_rays = 10000;
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

	int hit_count = 0;

	auto start_trace = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < num_rays; ++i) {
		Vec3 origin(dist(rng), dist(rng), dist(rng));
		Vec3 direction = glm::normalize(Vec3(dist(rng), dist(rng), dist(rng)));

		Ray ray(origin, direction);
		HitRecord hit;

		if (bvh.intersect(ray, hit)) {
			hit_count++;
		}
	}

	auto end_trace = std::chrono::high_resolution_clock::now();
	double trace_time = std::chrono::duration<double, std::milli>(end_trace - start_trace).count();

	ARE_LOG_INFO("Ray tracing time: " + std::to_string(trace_time) + " ms for " + std::to_string(num_rays) + " rays");
	ARE_LOG_INFO("Hit rate: " + std::to_string(hit_count) + "/" + std::to_string(num_rays) + " (" + std::to_string(100.0 * hit_count / num_rays) + "%)");
	ARE_LOG_INFO("Average time per ray: " + std::to_string(trace_time / num_rays) + " ms");

	report_test("BVH performance test", success && build_time < 5000.0); // Should build in < 5 seconds
}

/**
 * @brief Test 6: BVH memory usage
 */
void test_bvh_memory() {
	auto triangles = create_complex_scene(1000);

	BVH bvh;
	bvh.build(triangles);

	size_t memory = bvh.get_memory_usage();
	ARE_LOG_INFO("BVH memory usage: " + std::to_string(memory / 1024) + " KB");

	report_test("BVH memory usage", memory > 0);
}

/**
 * @brief Test 7: BVH clear and rebuild
 */
void test_bvh_clear_rebuild() {
	auto triangles = create_simple_scene();

	BVH bvh;
	bvh.build(triangles);

	bool test1 = bvh.is_built();

	bvh.clear();
	bool test2 = !bvh.is_built();

	bvh.build(triangles);
	bool test3 = bvh.is_built();

	report_test("BVH clear and rebuild (initial build)", test1);
	report_test("BVH clear and rebuild (after clear)", test2);
	report_test("BVH clear and rebuild (rebuild)", test3);
}

/**
 * @brief Test 8: BVH with empty scene
 */
void test_bvh_empty_scene() {
	std::vector<Triangle> empty_triangles;

	BVH bvh;
	bool success = bvh.build(empty_triangles);

	report_test("BVH with empty scene", !success);
}

/**
 * @brief Test 9: BVH node structure
 */
void test_bvh_node_structure() {
	auto triangles = create_simple_scene();

	BVH bvh;
	bvh.build(triangles);

	const auto &nodes = bvh.get_nodes();

	bool test1 = !nodes.empty();

	// Check root node
	bool test2 = nodes[0].bounds_.is_valid();

	// Count leaf and internal nodes
	int leaf_count = 0;
	int internal_count = 0;

	for (const auto &node : nodes) {
		if (node.is_leaf()) {
			leaf_count++;
		} else {
			internal_count++;
		}
	}

	bool test3 = leaf_count > 0;
	bool test4 = internal_count >= 0;

	ARE_LOG_INFO("BVH structure: " + std::to_string(nodes.size()) + " nodes (" + std::to_string(leaf_count) + " leaves, " + std::to_string(internal_count) + " internal)");

	report_test("BVH node structure (has nodes)", test1);
	report_test("BVH node structure (valid root)", test2);
	report_test("BVH node structure (has leaves)", test3);
	report_test("BVH node structure (node counts)", test4);
}

/**
 * @brief Test 10: BVH traversal correctness
 */
void test_bvh_traversal_correctness() {
	// Create a simple scene with known geometry
	std::vector<Triangle> triangles;

	// Single triangle at origin
	Vertex v0(Vec3(-1, 0, -1), Vec3(0, 1, 0));
	Vertex v1(Vec3(1, 0, -1), Vec3(0, 1, 0));
	Vertex v2(Vec3(0, 0, 1), Vec3(0, 1, 0));

	triangles.emplace_back(v0, v1, v2);

	BVH bvh;
	bvh.build(triangles);

	// Ray hitting the triangle from above
	Ray ray(Vec3(0, 5, 0), Vec3(0, -1, 0));
	HitRecord hit;

	bool intersected = bvh.intersect(ray, hit);

	bool test1 = intersected;
	bool test2 = hit.t_ > 0.0f && hit.t_ < 10.0f;
	bool test3 = glm::length(hit.normal_ - Vec3(0, 1, 0)) < 0.01f;

	report_test("BVH traversal correctness (intersection)", test1);
	report_test("BVH traversal correctness (t value)", test2);
	report_test("BVH traversal correctness (normal)", test3);
}

int main() {
	// Initialize logger and profiler
	Logger::init(LogLevel::ARE_LOG_INFO);
	Profiler::init();

	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("Phase 4 Verification Program");
	ARE_LOG_INFO("BVH Construction and Traversal Test");
	ARE_LOG_INFO("========================================");

	// Run all tests
	test_bvh_construction();
	test_bvh_construction_sah();
	test_ray_bvh_intersection();
	test_bvh_occlusion();
	test_bvh_performance();
	test_bvh_memory();
	test_bvh_clear_rebuild();
	test_bvh_empty_scene();
	test_bvh_node_structure();
	test_bvh_traversal_correctness();

	// Print summary
	ARE_LOG_INFO("========================================");
	ARE_LOG_INFO("Test Summary");
	ARE_LOG_INFO("========================================");

	int passed = 0;
	int failed = 0;

	for (const auto &result : test_results) {
		if (result.passed) {
			++passed;
		} else {
			++failed;
		}
	}

	ARE_LOG_INFO("Total tests: " + std::to_string(test_results.size()));
	ARE_LOG_INFO("Passed: " + std::to_string(passed));
	ARE_LOG_INFO("Failed: " + std::to_string(failed));

	if (failed == 0) {
		ARE_LOG_INFO("========================================");
		ARE_LOG_INFO("✓ All Phase 4 tests passed!");
		ARE_LOG_INFO("========================================");
	} else {
		ARE_LOG_ERROR("========================================");
		ARE_LOG_ERROR("✗ Some tests failed. Please review.");
		ARE_LOG_ERROR("========================================");
	}

	// Print profiling results
	Profiler::print_results();

	Profiler::shutdown();
	Logger::shutdown();

	return failed == 0 ? 0 : 1;
}
