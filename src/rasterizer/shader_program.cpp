/**
 * @file shader_program.cpp
 * @brief Implementation of ShaderProgram class
 */

#include <are/rasterizer/shader_program.h>
#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/utils/file_utils.h>
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

namespace are {

ShaderProgram::ShaderProgram()
    : program_(0)
    , vertex_shader_(0)
    , fragment_shader_(0)
    , compute_shader_(0)
    , linked_(false) {
    program_ = glCreateProgram();
    if (program_ == 0) {
        ARE_LOG_ERROR("ShaderProgram: Failed to create OpenGL program");
    }
}

ShaderProgram::~ShaderProgram() {
    if (vertex_shader_ != 0) {
        glDeleteShader(vertex_shader_);
    }
    if (fragment_shader_ != 0) {
        glDeleteShader(fragment_shader_);
    }
    if (compute_shader_ != 0) {
        glDeleteShader(compute_shader_);
    }
    if (program_ != 0) {
        glDeleteProgram(program_);
    }
}

bool ShaderProgram::load_shader(ShaderType type, const std::string& filepath) {
    ARE_PROFILE_FUNCTION();
    
    // Read shader source from file
    std::string source = read_file_to_string(filepath);
    if (source.empty()) {
        ARE_LOG_ERROR("ShaderProgram: Failed to read shader file: " + filepath);
        return false;
    }
    
    ARE_LOG_INFO("ShaderProgram: Loaded shader from " + filepath);
    return compile_shader(type, source);
}

bool ShaderProgram::compile_shader(ShaderType type, const std::string& source) {
    ARE_PROFILE_FUNCTION();
    
    GLenum gl_type;
    uint32_t* shader_id;
    std::string type_name;
    
    switch (type) {
        case ShaderType::ARE_SHADER_VERTEX:
            gl_type = GL_VERTEX_SHADER;
            shader_id = &vertex_shader_;
            type_name = "vertex";
            break;
        case ShaderType::ARE_SHADER_FRAGMENT:
            gl_type = GL_FRAGMENT_SHADER;
            shader_id = &fragment_shader_;
            type_name = "fragment";
            break;
        case ShaderType::ARE_SHADER_COMPUTE:
            gl_type = GL_COMPUTE_SHADER;
            shader_id = &compute_shader_;
            type_name = "compute";
            break;
        default:
            ARE_LOG_ERROR("ShaderProgram: Unknown shader type");
            return false;
    }
    
    // Delete existing shader if any
    if (*shader_id != 0) {
        glDeleteShader(*shader_id);
    }
    
    // Create and compile shader
    *shader_id = glCreateShader(gl_type);
    const char* source_cstr = source.c_str();
    glShaderSource(*shader_id, 1, &source_cstr, nullptr);
    glCompileShader(*shader_id);
    
    // Check compilation errors
    if (!check_compile_errors(*shader_id, type)) {
        ARE_LOG_ERROR("ShaderProgram: Failed to compile " + type_name + " shader");
        glDeleteShader(*shader_id);
        *shader_id = 0;
        return false;
    }
    
    // Attach shader to program
    glAttachShader(program_, *shader_id);
    
    ARE_LOG_INFO("ShaderProgram: Compiled " + type_name + " shader successfully");
    return true;
}

bool ShaderProgram::link() {
    ARE_PROFILE_FUNCTION();
    
    if (program_ == 0) {
        ARE_LOG_ERROR("ShaderProgram: Cannot link invalid program");
        return false;
    }
    
    // Link program
    glLinkProgram(program_);
    
    // Check link errors
    if (!check_link_errors()) {
        ARE_LOG_ERROR("ShaderProgram: Failed to link shader program");
        linked_ = false;
        return false;
    }
    
    // Detach and delete shaders after successful link
    if (vertex_shader_ != 0) {
        glDetachShader(program_, vertex_shader_);
        glDeleteShader(vertex_shader_);
        vertex_shader_ = 0;
    }
    if (fragment_shader_ != 0) {
        glDetachShader(program_, fragment_shader_);
        glDeleteShader(fragment_shader_);
        fragment_shader_ = 0;
    }
    if (compute_shader_ != 0) {
        glDetachShader(program_, compute_shader_);
        glDeleteShader(compute_shader_);
        compute_shader_ = 0;
    }
    
    linked_ = true;
    ARE_LOG_INFO("ShaderProgram: Linked shader program successfully");
    return true;
}

void ShaderProgram::use() const {
    if (is_valid()) {
        glUseProgram(program_);
    } else {
        ARE_LOG_WARN("ShaderProgram: Attempting to use invalid program");
    }
}

void ShaderProgram::set_uniform(const std::string& name, int value) {
    glUniform1i(get_uniform_location(name), value);
}

void ShaderProgram::set_uniform(const std::string& name, float value) {
    glUniform1f(get_uniform_location(name), value);
}

void ShaderProgram::set_uniform(const std::string& name, const Vec2& value) {
    glUniform2fv(get_uniform_location(name), 1, glm::value_ptr(value));
}

void ShaderProgram::set_uniform(const std::string& name, const Vec3& value) {
    glUniform3fv(get_uniform_location(name), 1, glm::value_ptr(value));
}

void ShaderProgram::set_uniform(const std::string& name, const Vec4& value) {
    glUniform4fv(get_uniform_location(name), 1, glm::value_ptr(value));
}

void ShaderProgram::set_uniform(const std::string& name, const Mat3& value) {
    glUniformMatrix3fv(get_uniform_location(name), 1, GL_FALSE, glm::value_ptr(value));
}

void ShaderProgram::set_uniform(const std::string& name, const Mat4& value) {
    glUniformMatrix4fv(get_uniform_location(name), 1, GL_FALSE, glm::value_ptr(value));
}

int ShaderProgram::get_uniform_location(const std::string& name) {
    // Check cache first
    auto it = uniform_cache_.find(name);
    if (it != uniform_cache_.end()) {
        return it->second;
    }
    
    // Query OpenGL
    int location = glGetUniformLocation(program_, name.c_str());
    if (location == -1) {
        ARE_LOG_WARN("ShaderProgram: Uniform '" + name + "' not found");
    }
    
    // Cache the location
    uniform_cache_[name] = location;
    return location;
}

bool ShaderProgram::check_compile_errors(uint32_t shader, ShaderType type) {
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    
    if (!success) {
        GLint log_length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        
        std::string info_log;
        info_log.resize(log_length);
        glGetShaderInfoLog(shader, log_length, nullptr, &info_log[0]);
        
        std::string type_name;
        switch (type) {
            case ShaderType::ARE_SHADER_VERTEX:   type_name = "VERTEX"; break;
            case ShaderType::ARE_SHADER_FRAGMENT: type_name = "FRAGMENT"; break;
            case ShaderType::ARE_SHADER_COMPUTE:  type_name = "COMPUTE"; break;
        }
        
        ARE_LOG_ERROR("ShaderProgram: " + type_name + " shader compilation failed:");
        ARE_LOG_ERROR(info_log);
        return false;
    }
    
    return true;
}

bool ShaderProgram::check_link_errors() {
    GLint success;
    glGetProgramiv(program_, GL_LINK_STATUS, &success);
    
    if (!success) {
        GLint log_length;
        glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &log_length);
        
        std::string info_log;
        info_log.resize(log_length);
        glGetProgramInfoLog(program_, log_length, nullptr, &info_log[0]);
        
        ARE_LOG_ERROR("ShaderProgram: Program linking failed:");
        ARE_LOG_ERROR(info_log);
        return false;
    }
    
    return true;
}

} // namespace are
