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

enum class ShaderType {
    ARE_SHADER_VERTEX,
    ARE_SHADER_FRAGMENT,
    ARE_SHADER_COMPUTE
};

class ShaderProgram {
public:
    ShaderProgram();
    ~ShaderProgram();

    bool load_shader(ShaderType type, const std::string& filepath);
    bool compile_shader(ShaderType type, const std::string& source);
    bool link();

    void use() const;

    bool is_valid() const { return program_ != 0 && linked_; }
    uint32_t get_program() const { return program_; }

    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, uint32_t value);   ///< NEW
    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, const Vec2& value);
    void set_uniform(const std::string& name, const Vec3& value);
    void set_uniform(const std::string& name, const Vec4& value);
    void set_uniform(const std::string& name, const Mat3& value);
    void set_uniform(const std::string& name, const Mat4& value);

    int get_uniform_location(const std::string& name);

private:
    bool check_compile_errors(uint32_t shader, ShaderType type);
    bool check_link_errors();

    uint32_t program_;
    uint32_t vertex_shader_;
    uint32_t fragment_shader_;
    uint32_t compute_shader_;
    bool linked_;
    std::unordered_map<std::string, int> uniform_cache_;
};

} // namespace are

#endif // ARE_INCLUDE_RASTERIZER_SHADER_PROGRAM_H
