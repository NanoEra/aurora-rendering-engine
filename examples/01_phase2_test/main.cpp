/**
 * @file main.cpp
 * @brief Phase 2 verification program
 */

#include <are/core/logger.h>
#include <are/core/config.h>
#include <are/geometry/vertex.h>
#include <are/geometry/triangle.h>
#include <are/geometry/aabb.h>
#include <are/geometry/transform.h>
#include <are/scene/camera.h>
#include <are/scene/mesh.h>
#include <are/scene/material.h>
#include <are/scene/directional_light.h>
#include <are/scene/point_light.h>
#include <are/scene/spot_light.h>
#include <are/scene/scene_manager.h>
#include <are/raytracer/ray.h>
#include <are/raytracer/hit_record.h>

#include <iostream>
#include <vector>

using namespace are;

// Test result tracking
struct TestResult {
    std::string name;
    bool passed;
    std::string message;
};

std::vector<TestResult> test_results;

void report_test(const std::string& name, bool passed, const std::string& message = "") {
    test_results.push_back({name, passed, message});
    if (passed) {
        ARE_LOG_INFO("✓ " + name);
    } else {
        ARE_LOG_ERROR("✗ " + name + ": " + message);
    }
}

// Test 1: Vertex operations
void test_vertex() {
    Vertex v1(Vec3(1, 2, 3));
    Vertex v2(Vec3(4, 5, 6), Vec3(0, 1, 0));
    Vertex v3 = Vertex::lerp(v1, v2, 0.5f);
    
    bool passed = glm::length(v3.position_ - Vec3(2.5f, 3.5f, 4.5f)) < are_epsilon;
    report_test("Vertex interpolation", passed);
}

// Test 2: AABB operations
void test_aabb() {
    AABB aabb1(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    AABB aabb2(Vec3(0, 0, 0), Vec3(2, 2, 2));
    
    bool test1 = aabb1.is_valid();
    bool test2 = aabb1.contains(Vec3(0, 0, 0));
    bool test3 = aabb1.intersects(aabb2);
    bool test4 = aabb1.longest_axis() == 0; // All axes equal
    
    AABB merged = AABB::merge(aabb1, aabb2);
    bool test5 = merged.contains(Vec3(-1, -1, -1)) && merged.contains(Vec3(2, 2, 2));
    
    report_test("AABB validity", test1);
    report_test("AABB contains point", test2);
    report_test("AABB intersection", test3);
    report_test("AABB merge", test5);
}

// Test 3: Triangle operations
void test_triangle() {
    Vertex v0(Vec3(0, 0, 0), Vec3(0, 0, 1));
    Vertex v1(Vec3(1, 0, 0), Vec3(0, 0, 1));
    Vertex v2(Vec3(0, 1, 0), Vec3(0, 0, 1));
    
    Triangle tri(v0, v1, v2);
    
    Vec3 centroid = tri.centroid();
    bool test1 = glm::length(centroid - Vec3(1.0f/3.0f, 1.0f/3.0f, 0.0f)) < are_epsilon;
    
    Vec3 normal = tri.normal();
    bool test2 = glm::length(normal - Vec3(0, 0, 1)) < are_epsilon;
    
    Real area = tri.area();
    bool test3 = std::abs(area - 0.5f) < are_epsilon;
    
    AABB aabb = tri.compute_aabb();
    bool test4 = aabb.contains(Vec3(0, 0, 0)) && aabb.contains(Vec3(1, 0, 0));
    
    report_test("Triangle centroid", test1);
    report_test("Triangle normal", test2);
    report_test("Triangle area", test3);
    report_test("Triangle AABB", test4);
}

// Test 4: Ray-Triangle intersection
void test_ray_triangle_intersection() {
    Vertex v0(Vec3(0, 0, 0), Vec3(0, 0, 1));
    Vertex v1(Vec3(1, 0, 0), Vec3(0, 0, 1));
    Vertex v2(Vec3(0, 1, 0), Vec3(0, 0, 1));
    
    Triangle tri(v0, v1, v2);
    
    // Ray hitting the triangle
    Ray ray1(Vec3(0.25f, 0.25f, -1.0f), Vec3(0, 0, 1));
    HitRecord hit1;
    bool test1 = tri.intersect(ray1, hit1);
    
    // Ray missing the triangle
    Ray ray2(Vec3(2, 2, -1), Vec3(0, 0, 1));
    HitRecord hit2;
    bool test2 = !tri.intersect(ray2, hit2);
    
    report_test("Ray-Triangle hit", test1);
    report_test("Ray-Triangle miss", test2);
}

// Test 5: Transform operations
void test_transform() {
    Transform t1 = Transform::translate(Vec3(1, 2, 3));
    Transform t2 = Transform::rotate(Vec3(0, are_pi / 2, 0));
    Transform t3 = Transform::scale(Vec3(2, 2, 2));
    
    Vec3 point = Vec3(1, 0, 0);
    Vec3 transformed = t1.transform_point(point);
    bool test1 = glm::length(transformed - Vec3(2, 2, 3)) < are_epsilon;
    
    Vec3 scaled = t3.transform_point(point);
    bool test2 = glm::length(scaled - Vec3(2, 0, 0)) < are_epsilon;
    
    report_test("Transform translation", test1);
    report_test("Transform scale", test2);
}

// Test 6: Camera operations
void test_camera() {
    Camera camera(Vec3(0, 0, 5), Vec3(0, 0, 0));
    camera.set_perspective(45.0f, 16.0f / 9.0f, 0.1f, 100.0f);
    
    Vec3 forward = camera.get_forward();
    bool test1 = glm::length(forward - Vec3(0, 0, -1)) < are_epsilon;
    
    Vec3 origin, direction;
    camera.generate_ray(0.5f, 0.5f, origin, direction);
    bool test2 = glm::length(origin - Vec3(0, 0, 5)) < are_epsilon;
    bool test3 = glm::length(direction - Vec3(0, 0, -1)) < 0.1f; // Approximate
    
    report_test("Camera forward vector", test1);
    report_test("Camera ray generation origin", test2);
    report_test("Camera ray generation direction", test3);
}

// Test 7: Mesh operations
void test_mesh() {
    std::vector<Vertex> vertices = {
        Vertex(Vec3(0, 0, 0), Vec3(0, 0, 1)),
        Vertex(Vec3(1, 0, 0), Vec3(0, 0, 1)),
        Vertex(Vec3(0, 1, 0), Vec3(0, 0, 1))
    };
    
    std::vector<uint32_t> indices = {0, 1, 2};
    
    Mesh mesh(vertices, indices);
    
    bool test1 = mesh.get_vertex_count() == 3;
    bool test2 = mesh.get_triangle_count() == 1;
    bool test3 = mesh.get_aabb().is_valid();
    
    Vertex v0, v1, v2;
    bool test4 = mesh.get_triangle(0, v0, v1, v2);
    
    report_test("Mesh vertex count", test1);
    report_test("Mesh triangle count", test2);
    report_test("Mesh AABB", test3);
    report_test("Mesh get triangle", test4);
}

// Test 8: Material operations
void test_material() {
    Material mat;
    mat.set_albedo(Vec3(0.8f, 0.2f, 0.1f));
    mat.set_metallic(0.5f);
    mat.set_roughness(0.3f);
    mat.set_emissive(Vec3(1.0f, 0.5f, 0.0f));
    
    bool test1 = glm::length(mat.get_albedo() - Vec3(0.8f, 0.2f, 0.1f)) < are_epsilon;
    bool test2 = std::abs(mat.get_metallic() - 0.5f) < are_epsilon;
    bool test3 = mat.is_emissive();
    
    mat.set_albedo_map("textures/albedo.png");
    bool test4 = mat.has_albedo_map();
    
    report_test("Material albedo", test1);
    report_test("Material metallic", test2);
    report_test("Material emissive", test3);
    report_test("Material texture map", test4);
}

// Test 9: Light operations
void test_lights() {
    // Directional light
    DirectionalLight dir_light(Vec3(0, -1, 0), Vec3(1, 1, 1), 1.0f);
    bool test1 = dir_light.affects_point(Vec3(100, 100, 100));
    
    // Point light
    PointLight point_light(Vec3(0, 0, 0), Vec3(1, 1, 1), 1.0f, 10.0f);
    bool test2 = point_light.affects_point(Vec3(5, 0, 0));
    bool test3 = !point_light.affects_point(Vec3(20, 0, 0));
    
    // Spot light
    SpotLight spot_light(Vec3(0, 0, 0), Vec3(0, 0, -1), 30.0f, 45.0f);
    bool test4 = spot_light.affects_point(Vec3(0, 0, -5));
    
    report_test("Directional light affects all points", test1);
    report_test("Point light range (inside)", test2);
    report_test("Point light range (outside)", test3);
    report_test("Spot light cone", test4);
}

// Test 10: SceneManager operations
void test_scene_manager() {
    SceneManager scene;
    
    // Add mesh
    std::vector<Vertex> vertices = {
        Vertex(Vec3(0, 0, 0)),
        Vertex(Vec3(1, 0, 0)),
        Vertex(Vec3(0, 1, 0))
    };
    std::vector<uint32_t> indices = {0, 1, 2};
    Mesh mesh(vertices, indices);
    
    MeshHandle mesh_handle = scene.add_mesh(mesh);
    bool test1 = mesh_handle != are_invalid_handle;
    bool test2 = scene.get_mesh_count() == 1;
    
    // Add material
    Material mat;
    MaterialHandle mat_handle = scene.add_material(mat);
    bool test3 = mat_handle != are_invalid_handle;
    bool test4 = scene.get_material_count() == 1;
    
    // Add light
    auto light = std::make_shared<DirectionalLight>();
    LightHandle light_handle = scene.add_light(light);
    bool test5 = light_handle != are_invalid_handle;
    bool test6 = scene.get_light_count() == 1;
    
    // Test dirty flag
    bool test7 = scene.is_dirty();
    scene.clear_dirty();
    bool test8 = !scene.is_dirty();
    
    // Remove mesh
    scene.remove_mesh(mesh_handle);
    bool test9 = scene.get_mesh_count() == 0;
    
    report_test("SceneManager add mesh", test1);
    report_test("SceneManager mesh count", test2);
    report_test("SceneManager add material", test3);
    report_test("SceneManager material count", test4);
    report_test("SceneManager add light", test5);
    report_test("SceneManager light count", test6);
    report_test("SceneManager dirty flag (set)", test7);
    report_test("SceneManager dirty flag (clear)", test8);
    report_test("SceneManager remove mesh", test9);
}

int main() {
    // Initialize logger
    Logger::init(LogLevel::ARE_LOG_INFO);
    
    ARE_LOG_INFO("========================================");
    ARE_LOG_INFO("Phase 2 Verification Program");
    ARE_LOG_INFO("========================================");
    
    // Run all tests
    test_vertex();
    test_aabb();
    test_triangle();
    test_ray_triangle_intersection();
    test_transform();
    test_camera();
    test_mesh();
    test_material();
    test_lights();
    test_scene_manager();
    
    // Print summary
    ARE_LOG_INFO("========================================");
    ARE_LOG_INFO("Test Summary");
    ARE_LOG_INFO("========================================");
    
    int passed = 0;
    int failed = 0;
    
    for (const auto& result : test_results) {
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
        ARE_LOG_INFO("✓ All Phase 2 tests passed!");
        ARE_LOG_INFO("========================================");
    } else {
        ARE_LOG_ERROR("========================================");
        ARE_LOG_ERROR("✗ Some tests failed. Please review.");
        ARE_LOG_ERROR("========================================");
    }
    
    Logger::shutdown();
    
    return failed == 0 ? 0 : 1;
}
