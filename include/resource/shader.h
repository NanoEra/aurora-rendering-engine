#ifndef ARE_INCLUDE_RESOURCE_SHADER_H
#define ARE_INCLUDE_RESOURCE_SHADER_H

#include "basic/types.h"
#include <string>
#include <unordered_map>

namespace are {

// Shader program resource
class Shader {
public:
	// Constructor
	Shader();

	Shader(const Shader &) = delete;
	Shader &operator=(const Shader &) = delete;

	Shader(Shader &&other) noexcept;
	Shader &operator=(Shader &&other) noexcept;

	// Destructor
	~Shader();

	/*
	 * @brief Load and compile shader from files
	 * @param vertex_path Vertex shader path
	 * @param fragment_path Fragment shader path
	 * @return True if compilation succeeded
	 */
	bool load(const std::string &vertex_path, const std::string &fragment_path);

	/*
	 * @brief Load and compile compute shader
	 * @param compute_path Compute shader path
	 * @return True if compilation succeeded
	 */
	bool load_compute(const std::string &compute_path);

	/*
	 * @brief Compile shader from source strings
	 * @param vertex_source Vertex shader source
	 * @param fragment_source Fragment shader source
	 * @return True if compilation succeeded
	 */
	bool compile(const std::string &vertex_source, const std::string &fragment_source);

	/*
	 * @brief Compile compute shader from source
	 * @param compute_source Compute shader source
	 * @return True if compilation succeeded
	 */
	bool compile_compute(const std::string &compute_source);

	// Use/activate shader program
	void use() const; // 改为const

	// Release shader resources
	void release();

	/*
	 * @brief Set uniform boolean
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_bool(const std::string &name, bool value) const; // 新增，const

	/*
	 * @brief Set uniform integer
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_int(const std::string &name, int value) const; // 改为const

	/*
	 * @brief Set uniform unsigned integer
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_uint(const std::string &name, uint value) const; // 改为const

	/*
	 * @brief Set uniform float
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_float(const std::string &name, float value) const; // 改为const

	/*
	 * @brief Set uniform vec2
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_vec2(const std::string &name, const Vec2 &value) const; // 改为const

	/*
	 * @brief Set uniform vec3
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_vec3(const std::string &name, const Vec3 &value) const; // 改为const

	/*
	 * @brief Set uniform vec4
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_vec4(const std::string &name, const Vec4 &value) const; // 改为const

	/*
	 * @brief Set uniform mat3
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_mat3(const std::string &name, const Mat3 &value) const; // 改为const

	/*
	 * @brief Set uniform mat4
	 * @param name Uniform name
	 * @param value Value
	 */
	void set_mat4(const std::string &name, const Mat4 &value) const; // 改为const

	/*
	 * @brief Get shader program handle
	 * @return Shader handle
	 */
	ShaderHandle get_handle() const {
		return handle_;
	}

	/*
	 * @brief Check if shader is valid
	 * @return True if valid
	 */
	bool is_valid() const {
		return handle_ != INVALID_HANDLE;
	}

private:
	ShaderHandle handle_;
	mutable std::unordered_map<std::string, int> uniform_cache_; // 改为mutable

	/*
	 * @brief Get uniform location (with caching)
	 * @param name Uniform name
	 * @return Uniform location
	 */
	int get_uniform_location_(const std::string &name) const; // 改为const

	/*
	 * @brief Compile shader stage
	 * @param source Shader source code
	 * @param type Shader type (GL_VERTEX_SHADER, etc.)
	 * @return Shader object handle (0 on failure)
	 */
	uint compile_shader_(const std::string &source, uint type);

	/*
	 * @brief Link shader program
	 * @param shaders Array of shader object handles
	 * @param count Number of shaders
	 * @return True if linking succeeded
	 */
	bool link_program_(const uint *shaders, uint count);

	/*
	 * @brief Read file content
	 * @param path File path
	 * @return File content
	 */
	std::string read_file_(const std::string &path);
};

} // namespace are

#endif // ARE_INCLUDE_RESOURCE_SHADER_H
