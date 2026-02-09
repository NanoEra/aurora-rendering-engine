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
