#ifndef ARE_INCLUDE_SCENE_SCENE_H
#define ARE_INCLUDE_SCENE_SCENE_H

#include "basic/types.h"
#include "scene/camera.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/mesh.h"
#include <memory>
#include <vector>

namespace are {

// Scene container holding all scene objects
class Scene {
public:
	// Constructor
	Scene();

	// Destructor
	~Scene();

	/*
	 * @brief Add mesh to scene
	 * @param mesh Mesh to add
	 * @return Mesh index
	 */
	uint add_mesh(std::shared_ptr<Mesh> mesh);

	/*
	 * @brief Add material to scene
	 * @param material Material to add
	 * @return Material index
	 */
	uint add_material(std::shared_ptr<Material> material);

	/*
	 * @brief Add light to scene
	 * @param light Light to add
	 * @return Light index
	 */
	uint add_light(std::shared_ptr<Light> light);

	/*
	 * @brief Set active camera
	 * @param camera Camera to set
	 */
	void set_camera(std::shared_ptr<Camera> camera);

	/*
	 * @brief Get active camera
	 * @return Active camera
	 */
	const Camera &get_camera() const {
		return *camera_;
	}

	/*
	 * @brief Get all meshes
	 * @return Mesh list
	 */
	const std::vector<std::shared_ptr<Mesh>> &get_meshes() const {
		return meshes_;
	}

	/*
	 * @brief Get all materials
	 * @return Material list
	 */
	const std::vector<std::shared_ptr<Material>> &get_materials() const {
		return materials_;
	}

	/*
	 * @brief Get all lights
	 * @return Light list
	 */
	const std::vector<std::shared_ptr<Light>> &get_lights() const {
		return lights_;
	}

	// Clear all scene objects
	void clear();

	/*
	 * @brief Update scene (animations, transforms, etc.)
	 * @param delta_time Time since last update
	 */
	void update(float delta_time);

private:
	std::shared_ptr<Camera> camera_;
	std::vector<std::shared_ptr<Mesh>> meshes_;
	std::vector<std::shared_ptr<Material>> materials_;
	std::vector<std::shared_ptr<Light>> lights_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_SCENE_H
