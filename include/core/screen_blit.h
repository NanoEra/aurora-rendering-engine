#ifndef ARE_INCLUDE_CORE_SCREEN_BLIT_H
#define ARE_INCLUDE_CORE_SCREEN_BLIT_H

#include "basic/types.h"
#include "resource/shader.h"
#include <memory>

namespace are {

/// @brief Screen blit utility for rendering texture to screen
class ScreenBlit {
public:
    /// @brief Constructor
    ScreenBlit();
    
    /// @brief Destructor
    ~ScreenBlit();
    
    /// @brief Initialize screen blit
    /// @return True if initialization succeeded
    bool initialize(const std::shared_ptr<Shader> &screen_blit_shader);
    
    /// @brief Release resources
    void release();
    
    /// @brief Blit texture to screen
    /// @param texture Texture to blit
    /// @param x Screen X position
    /// @param y Screen Y position
    /// @param width Blit width
    /// @param height Blit height
    void blit(TextureHandle texture, int x, int y, uint width, uint height);
    
    /// @brief Blit texture to full screen
    /// @param texture Texture to blit
    void blit_fullscreen(TextureHandle texture);

private:
	std::shared_ptr<Shader> shader_;
    uint vao_;
    uint vbo_;
    bool initialized_;
    
    /// @brief Create fullscreen quad
    void create_quad_();
};

} // namespace are

#endif // ARE_INCLUDE_CORE_SCREEN_BLIT_H
