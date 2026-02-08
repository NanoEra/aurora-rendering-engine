/**
 * @file scene_manager.cpp
 * @brief Implementation of SceneManager class
 */

#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/scene/scene_manager.h>

namespace are {

SceneManager::SceneManager()
	: next_mesh_handle_(1)
	, next_material_handle_(1)
	, next_light_handle_(1)
	, dirty_(false) {
}

SceneManager::~SceneManager() {
	clear();
}

MeshHandle SceneManager::add_mesh(const Mesh &mesh) {
	ARE_PROFILE_FUNCTION();

	if (mesh.is_empty()) {
		ARE_LOG_WARN("SceneManager: Attempting to add empty mesh");
		return are_invalid_handle;
	}

	MeshHandle handle = next_mesh_handle_++;
	meshes_.push_back(mesh);
	mesh_handle_map_[handle] = meshes_.size() - 1;

	dirty_ = true;

	ARE_LOG_DEBUG("SceneManager: Added mesh with handle " + std::to_string(handle));
	return handle;
}

void SceneManager::remove_mesh(MeshHandle handle) {
	ARE_PROFILE_FUNCTION();

	auto it = mesh_handle_map_.find(handle);
	if (it == mesh_handle_map_.end()) {
		ARE_LOG_WARN("SceneManager: Attempting to remove invalid mesh handle");
		return;
	}

	size_t index = it->second;

	// Swap with last element and pop
	if (index < meshes_.size() - 1) {
		meshes_[index] = meshes_.back();

		// Update handle map for swapped element
		for (auto &pair : mesh_handle_map_) {
			if (pair.second == meshes_.size() - 1) {
				pair.second = index;
				break;
			}
		}
	}

	meshes_.pop_back();
	mesh_handle_map_.erase(it);

	dirty_ = true;

	ARE_LOG_DEBUG("SceneManager: Removed mesh with handle " + std::to_string(handle));
}

void SceneManager::update_mesh(MeshHandle handle, const Mesh &mesh) {
	ARE_PROFILE_FUNCTION();

	Mesh *existing = get_mesh(handle);
	if (!existing) {
		ARE_LOG_WARN("SceneManager: Attempting to update invalid mesh handle");
		return;
	}

	*existing = mesh;
	dirty_ = true;
}

Mesh *SceneManager::get_mesh(MeshHandle handle) {
	auto it = mesh_handle_map_.find(handle);
	if (it == mesh_handle_map_.end()) {
		return nullptr;
	}

	size_t index = it->second;
	if (index >= meshes_.size()) {
		ARE_LOG_ERROR("SceneManager: Mesh handle map corrupted");
		return nullptr;
	}

	return &meshes_[index];
}

const Mesh *SceneManager::get_mesh(MeshHandle handle) const {
	auto it = mesh_handle_map_.find(handle);
	if (it == mesh_handle_map_.end()) {
		return nullptr;
	}

	size_t index = it->second;
	if (index >= meshes_.size()) {
		ARE_LOG_ERROR("SceneManager: Mesh handle map corrupted");
		return nullptr;
	}

	return &meshes_[index];
}

MaterialHandle SceneManager::add_material(const Material &material) {
	ARE_PROFILE_FUNCTION();

	MaterialHandle handle = next_material_handle_++;
	materials_.push_back(material);
	material_handle_map_[handle] = materials_.size() - 1;

	ARE_LOG_DEBUG("SceneManager: Added material with handle " + std::to_string(handle));
	return handle;
}

void SceneManager::remove_material(MaterialHandle handle) {
	ARE_PROFILE_FUNCTION();

	auto it = material_handle_map_.find(handle);
	if (it == material_handle_map_.end()) {
		ARE_LOG_WARN("SceneManager: Attempting to remove invalid material handle");
		return;
	}

	size_t index = it->second;

	// Swap with last element and pop
	if (index < materials_.size() - 1) {
		materials_[index] = materials_.back();

		// Update handle map for swapped element
		for (auto &pair : material_handle_map_) {
			if (pair.second == materials_.size() - 1) {
				pair.second = index;
				break;
			}
		}
	}

	materials_.pop_back();
	material_handle_map_.erase(it);

	ARE_LOG_DEBUG("SceneManager: Removed material with handle " + std::to_string(handle));
}

void SceneManager::update_material(MaterialHandle handle, const Material &material) {
	ARE_PROFILE_FUNCTION();

	Material *existing = get_material(handle);
	if (!existing) {
		ARE_LOG_WARN("SceneManager: Attempting to update invalid material handle");
		return;
	}

	*existing = material;
}

Material *SceneManager::get_material(MaterialHandle handle) {
	auto it = material_handle_map_.find(handle);
	if (it == material_handle_map_.end()) {
		return nullptr;
	}

	size_t index = it->second;
	if (index >= materials_.size()) {
		ARE_LOG_ERROR("SceneManager: Material handle map corrupted");
		return nullptr;
	}

	return &materials_[index];
}

const Material *SceneManager::get_material(MaterialHandle handle) const {
	auto it = material_handle_map_.find(handle);
	if (it == material_handle_map_.end()) {
		return nullptr;
	}

	size_t index = it->second;
	if (index >= materials_.size()) {
		ARE_LOG_ERROR("SceneManager: Material handle map corrupted");
		return nullptr;
	}

	return &materials_[index];
}

LightHandle SceneManager::add_light(const std::shared_ptr<Light> &light) {
	ARE_PROFILE_FUNCTION();

	if (!light) {
		ARE_LOG_WARN("SceneManager: Attempting to add null light");
		return are_invalid_handle;
	}

	LightHandle handle = next_light_handle_++;
	lights_.push_back(light);
	light_handle_map_[handle] = lights_.size() - 1;

	dirty_ = true;

	ARE_LOG_DEBUG("SceneManager: Added light with handle " + std::to_string(handle));
	return handle;
}

void SceneManager::remove_light(LightHandle handle) {
	ARE_PROFILE_FUNCTION();

	auto it = light_handle_map_.find(handle);
	if (it == light_handle_map_.end()) {
		ARE_LOG_WARN("SceneManager: Attempting to remove invalid light handle");
		return;
	}

	size_t index = it->second;

	// Swap with last element and pop
	if (index < lights_.size() - 1) {
		lights_[index] = lights_.back();

		// Update handle map for swapped element
		for (auto &pair : light_handle_map_) {
			if (pair.second == lights_.size() - 1) {
				pair.second = index;
				break;
			}
		}
	}

	lights_.pop_back();
	light_handle_map_.erase(it);

	dirty_ = true;

	ARE_LOG_DEBUG("SceneManager: Removed light with handle " + std::to_string(handle));
}

std::shared_ptr<Light> SceneManager::get_light(LightHandle handle) {
	auto it = light_handle_map_.find(handle);
	if (it == light_handle_map_.end()) {
		return nullptr;
	}

	size_t index = it->second;
	if (index >= lights_.size()) {
		ARE_LOG_ERROR("SceneManager: Light handle map corrupted");
		return nullptr;
	}

	return lights_[index];
}

size_t SceneManager::get_total_triangle_count() const {
	ARE_PROFILE_FUNCTION();

	size_t total = 0;
	for (const auto &mesh : meshes_) {
		total += mesh.get_triangle_count();
	}
	return total;
}

void SceneManager::clear() {
	ARE_PROFILE_FUNCTION();

	meshes_.clear();
	materials_.clear();
	lights_.clear();

	mesh_handle_map_.clear();
	material_handle_map_.clear();
	light_handle_map_.clear();

	next_mesh_handle_ = 1;
	next_material_handle_ = 1;
	next_light_handle_ = 1;

	dirty_ = true;

	ARE_LOG_INFO("SceneManager: Cleared all scene data");
}

void SceneManager::compact() {
	ARE_PROFILE_FUNCTION();

	// Remove invalid entries (this is a placeholder for future optimization)
	// Currently, the handle-based system ensures no invalid entries exist

	ARE_LOG_DEBUG("SceneManager: Compacted scene data");
}

} // namespace are
