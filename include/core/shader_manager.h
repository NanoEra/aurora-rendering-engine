#ifndef ARE_INCLUDE_CORE_SHADER_MANAGER_H
#define ARE_INCLUDE_CORE_SHADER_MANAGER_H

#include "basic/types.h"
#include "resource/shader.h"
#include <memory>
#include <string>
#include <unordered_map>

namespace are {

// Shader manager for loading and caching shaders
class ShaderManager {
public:
	// Constructor
	ShaderManager();

	// Destructor
	~ShaderManager();

	/*
	 * @brief Initialize shader manager and load built-in shaders
	 * @return True if initialization succeeded
	 */
	bool initialize();

	// Release all shaders
	void release();

	/*
	 * @brief Load shader from files
	 * @param name Shader name for caching
	 * @param vertex_path Vertex shader file path
	 * @param fragment_path Fragment shader file path
	 * @return Shader object
	 */
	std::shared_ptr<Shader> load_shader(const std::string &name,
		const std::string &vertex_path,
		const std::string &fragment_path);

	/*
	 * @brief Load compute shader from file
	 * @param name Shader name for caching
	 * @param compute_path Compute shader file path
	 * @return Shader object
	 */
	std::shared_ptr<Shader> load_compute_shader(const std::string &name,
		const std::string &compute_path);

	/*
	 * @brief Get cached shader by name
	 * @param name Shader name
	 * @return Shader object (invalid if not found)
	 */
	std::shared_ptr<Shader> get_shader(const std::string &name) const;

	/*
	 * @brief Get G-Buffer shader
	 * @return G-Buffer shader
	 */
	const std::shared_ptr<Shader> &get_gbuffer_shader() const {
		return gbuffer_shader_;
	}

	/*
	 * @brief Get ray tracing compute shader
	 * @return Ray tracing shader
	 */
	const std::shared_ptr<Shader> &get_raytracing_shader() const {
		return raytracing_shader_;
	}

	/*
	 * @brief Get mean denoise compute shader
	 * @return Denoise shader (nullptr if not loaded)
	 */
	const std::shared_ptr<Shader> &get_denoise_shader() const {
		return denoise_shader_;
	}

	/*
	 * @brief Get screen bliting shader
	 * @return Screen bliting shader (nullptr if not loaded)
	 */
	const std::shared_ptr<Shader> &get_screen_blit_shader() const {
		return screen_blit_shader_;
	}

private:
	std::unordered_map<std::string, std::shared_ptr<Shader>> shader_cache_;
	std::shared_ptr<Shader> gbuffer_shader_;
	std::shared_ptr<Shader> raytracing_shader_;
	std::shared_ptr<Shader> denoise_shader_;
	std::shared_ptr<Shader> screen_blit_shader_;

	bool initialized_;

	/*
	 * @brief Load built-in shaders
	 * @return True if loading succeeded
	 */
	bool load_builtin_shaders_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SHADER_MANAGER_H
