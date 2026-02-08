/**
 * @file shader_program.h
 * @brief OpenGL shader program wrapper
 */

#ifndef ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
#define ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H

#include <are/core/types.h>
#include <string>
#include <unordered_map>

namespace are {

/**
 * @enum ShaderType
 * @brief Shader stage types
 */
enum class ShaderType {
    ARE_SHADER_VERTEX,
    ARE_SHADER_FRAGMENT,
    ARE_SHADER_COMPUTE
};

/**
 * @class ShaderProgram
 * @brief OpenGL shader program management
 */
class ShaderProgram {
public:
    /**
     * @brief Constructor
     */
    ShaderProgram();

    /**
     * @brief Destructor
     */
    ~ShaderProgram();

    /**
     * @brief Load and compile shader from file
     * @param type Shader type
     * @param filepath Shader file path
     * @return true if compilation succeeded
     */
    bool load_shader(ShaderType type, const std::string& filepath);

    /**
     * @brief Compile shader from source string
     * @param type Shader type
     * @param source Shader source code
     * @return true if compilation succeeded
     */
    bool compile_shader(ShaderType type, const std::string& source);

    /**
     * @brief Link shader program
     * @return true if linking succeeded
     */
    bool link();

    /**
     * @brief Use this shader program
     */
    void use() const;

    /**
     * @brief Check if program is valid
     * @return true if valid
     */
    bool is_valid() const { return program_ != 0 && linked_; }

    /**
     * @brief Get OpenGL program ID
     * @return Program ID
     */
    uint32_t get_program() const { return program_; }

    // Uniform setters
    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, const Vec2& value);
    void set_uniform(const std::string& name, const Vec3& value);
    void set_uniform(const std::string& name, const Vec4& value);
    void set_uniform(const std::string& name, const Mat3& value);
    void set_uniform(const std::string& name, const Mat4& value);

    /**
     * @brief Get uniform location (cached)
     * @param name Uniform name
     * @return Uniform location (-1 if not found)
     */
    int get_uniform_location(const std::string& name);

private:
    bool check_compile_errors(uint32_t shader, ShaderType type);
    bool check_link_errors();

    uint32_t program_;                    ///< OpenGL program ID
    uint32_t vertex_shader_;              ///< Vertex shader ID
    uint32_t fragment_shader_;            ///< Fragment shader ID
    uint32_t compute_shader_;             ///< Compute shader ID
    
    bool linked_;                         ///< Link status
    std::unordered_map<std::string, int> uniform_cache_; ///< Uniform location cache
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
