#ifndef ARE_INCLUDE_CORE_SHADER_MANAGER_H
#define ARE_INCLUDE_CORE_SHADER_MANAGER_H

#include "basic/types.h"
#include "resource/shader.h"
#include <unordered_map>
#include <string>

namespace are {

/// @brief Shader manager for loading and caching shaders
class ShaderManager {
public:
    /// @brief Constructor
    ShaderManager();
    
    /// @brief Destructor
    ~ShaderManager();
    
    /// @brief Initialize shader manager and load built-in shaders
    /// @return True if initialization succeeded
    bool initialize();
    
    /// @brief Release all shaders
    void release();
    
    /// @brief Load shader from files
    /// @param name Shader name for caching
    /// @param vertex_path Vertex shader file path
    /// @param fragment_path Fragment shader file path
    /// @return Shader object
    Shader load_shader(const std::string& name, 
                      const std::string& vertex_path,
                      const std::string& fragment_path);
    
    /// @brief Load compute shader from file
    /// @param name Shader name for caching
    /// @param compute_path Compute shader file path
    /// @return Shader object
    Shader load_compute_shader(const std::string& name,
                              const std::string& compute_path);
    
    /// @brief Get cached shader by name
    /// @param name Shader name
    /// @return Shader object (invalid if not found)
    Shader get_shader(const std::string& name) const;
    
    /// @brief Get G-Buffer shader
    /// @return G-Buffer shader
    const Shader& get_gbuffer_shader() const { return gbuffer_shader_; }
    
    /// @brief Get ray tracing compute shader
    /// @return Ray tracing shader
    const Shader& get_raytracing_shader() const { return raytracing_shader_; }

private:
    std::unordered_map<std::string, Shader> shader_cache_;
    Shader gbuffer_shader_;
    Shader raytracing_shader_;
    
    bool initialized_;
    
    /// @brief Load built-in shaders
    /// @return True if loading succeeded
    bool load_builtin_shaders_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SHADER_MANAGER_H
