#include "resource/shader.h"
#include "basic/math.h"
#include "utils/logger.h"
#include <fstream>
#include <glad/glad.h>
#include <sstream>

namespace are {

Shader::Shader()
	: handle_(INVALID_HANDLE) {
}

Shader::Shader(Shader &&other) noexcept
	: handle_(other.handle_)
	, uniform_cache_(std::move(other.uniform_cache_)) {
	other.handle_ = INVALID_HANDLE;
	other.uniform_cache_.clear();
}

Shader::~Shader() {
	release();
}

Shader &Shader::operator=(Shader &&other) noexcept {
	if (this == &other)
		return *this;

	release();
	handle_ = other.handle_;
	uniform_cache_ = std::move(other.uniform_cache_);

	other.handle_ = INVALID_HANDLE;
	other.uniform_cache_.clear();
	return *this;
}

bool Shader::load(const std::string &vertex_path, const std::string &fragment_path) {
	std::string vertex_source = read_file_(vertex_path);
	std::string fragment_source = read_file_(fragment_path);

	if (vertex_source.empty() || fragment_source.empty()) {
		ARE_LOG_ERROR("Failed to read shader files");
		return false;
	}

	// Process #include directives
	std::string vertex_dir = vertex_path.substr(0, vertex_path.find_last_of("/\\"));
	std::string fragment_dir = fragment_path.substr(0, fragment_path.find_last_of("/\\"));
	vertex_source = process_includes_(vertex_source, vertex_dir);
	fragment_source = process_includes_(fragment_source, fragment_dir);

	return compile(vertex_source, fragment_source);
}

bool Shader::load_compute(const std::string &compute_path) {
	std::string compute_source = read_file_(compute_path);

	if (compute_source.empty()) {
		ARE_LOG_ERROR("Failed to read compute shader file");
		return false;
	}

	// Process #include directives
	std::string compute_dir = compute_path.substr(0, compute_path.find_last_of("/\\"));
	compute_source = process_includes_(compute_source, compute_dir);

	return compile_compute(compute_source);
}

bool Shader::compile(const std::string &vertex_source, const std::string &fragment_source) {
	uint vertex_shader = compile_shader_(vertex_source, GL_VERTEX_SHADER);
	if (vertex_shader == 0)
		return false;

	uint fragment_shader = compile_shader_(fragment_source, GL_FRAGMENT_SHADER);
	if (fragment_shader == 0) {
		glDeleteShader(vertex_shader);
		return false;
	}

	uint shaders[] = { vertex_shader, fragment_shader };
	bool success = link_program_(shaders, 2);

	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);

	return success;
}

bool Shader::compile_compute(const std::string &compute_source) {
	uint compute_shader = compile_shader_(compute_source, GL_COMPUTE_SHADER);
	if (compute_shader == 0)
		return false;

	uint shaders[] = { compute_shader };
	bool success = link_program_(shaders, 1);

	glDeleteShader(compute_shader);

	return success;
}

void Shader::use() const {
	if (handle_ != INVALID_HANDLE) {
		glUseProgram(handle_);
	}
}

void Shader::release() {
	if (handle_ != INVALID_HANDLE) {
		glDeleteProgram(handle_);
		handle_ = INVALID_HANDLE;
	}
	uniform_cache_.clear();
}

void Shader::set_bool(const std::string &name, bool value) const {
	glUniform1i(get_uniform_location_(name), static_cast<int>(value));
}

void Shader::set_int(const std::string &name, int value) const {
	glUniform1i(get_uniform_location_(name), value);
}

void Shader::set_uint(const std::string &name, uint value) const {
	glUniform1ui(get_uniform_location_(name), value);
}

void Shader::set_float(const std::string &name, float value) const {
	glUniform1f(get_uniform_location_(name), value);
}

void Shader::set_vec2(const std::string &name, const Vec2 &value) const {
	glUniform2fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_vec3(const std::string &name, const Vec3 &value) const {
	glUniform3fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_vec4(const std::string &name, const Vec4 &value) const {
	glUniform4fv(get_uniform_location_(name), 1, &value[0]);
}

void Shader::set_mat3(const std::string &name, const Mat3 &value) const {
	glUniformMatrix3fv(get_uniform_location_(name), 1, GL_FALSE, &value[0][0]);
}

void Shader::set_mat4(const std::string &name, const Mat4 &value) const {
	glUniformMatrix4fv(get_uniform_location_(name), 1, GL_FALSE, MathUtils::value_ptr(value));
}

int Shader::get_uniform_location_(const std::string &name) const {
	auto it = uniform_cache_.find(name);
	if (it != uniform_cache_.end()) {
		return it->second;
	}

	int location = glGetUniformLocation(handle_, name.c_str());
	uniform_cache_[name] = location;

	if (location == -1) {
		ARE_LOG_WARN("Uniform '" + name + "' not found in shader");
	}

	return location;
}

uint Shader::compile_shader_(const std::string &source, uint type) {
	uint shader = glCreateShader(type);
	const char *source_cstr = source.c_str();
	glShaderSource(shader, 1, &source_cstr, nullptr);
	glCompileShader(shader);

	int success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		char info_log[512];
		glGetShaderInfoLog(shader, 512, nullptr, info_log);

		std::string type_str = (type == GL_VERTEX_SHADER) ? "VERTEX" : (type == GL_FRAGMENT_SHADER) ? "FRAGMENT"
																									: "COMPUTE";
		ARE_LOG_ERROR("Shader compilation failed (" + type_str + "): " + std::string(info_log));

		glDeleteShader(shader);
		return 0;
	}

	return shader;
}

bool Shader::link_program_(const uint *shaders, uint count) {
	handle_ = glCreateProgram();

	for (uint i = 0; i < count; ++i) {
		glAttachShader(handle_, shaders[i]);
	}

	glLinkProgram(handle_);

	int success;
	glGetProgramiv(handle_, GL_LINK_STATUS, &success);
	if (!success) {
		char info_log[512];
		glGetProgramInfoLog(handle_, 512, nullptr, info_log);
		ARE_LOG_ERROR("Shader linking failed: " + std::string(info_log));

		glDeleteProgram(handle_);
		handle_ = INVALID_HANDLE;
		return false;
	}

	return true;
}

std::string Shader::read_file_(const std::string &path) {
	std::ifstream file(path);
	if (!file.is_open()) {
		ARE_LOG_ERROR("Failed to open file: " + path);
		return "";
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	return buffer.str();
}

std::string Shader::process_includes_(const std::string &source, const std::string &base_dir) {
	std::string result;
	std::istringstream stream(source);
	std::string line;

	while (std::getline(stream, line)) {
		// Check if line starts with #include
		std::string trimmed = line;
		// Trim leading whitespace
		size_t start = trimmed.find_first_not_of(" \t");
		if (start != std::string::npos) {
			trimmed = trimmed.substr(start);
		}

		if (trimmed.find("#include") == 0) {
			// Extract path: #include "path" or #include <path>
			size_t first_quote = line.find('"');
			size_t last_quote = line.rfind('"');

			if (first_quote != std::string::npos && last_quote != std::string::npos && first_quote != last_quote) {
				std::string include_path = line.substr(first_quote + 1, last_quote - first_quote - 1);
				std::string full_path = base_dir + "/" + include_path;

				// Read included file
				std::string included_content = read_file_(full_path);
				if (!included_content.empty()) {
					// Get directory of included file for nested includes
					std::string included_dir = full_path.substr(0, full_path.find_last_of("/\\"));

					// Recursively process includes
					result += process_includes_(included_content, included_dir) + "\n";
				} else {
					ARE_LOG_WARN("Include file not found or empty: " + full_path);
				}
			} else {
				// Invalid include syntax, keep original line
				result += line + "\n";
			}
		} else {
			result += line + "\n";
		}
	}

	return result;
}

} // namespace are
