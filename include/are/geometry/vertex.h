/**
 * @file vertex.h
 * @brief Vertex data structure definition
 */

#ifndef ARE_INCLUDE_GEOMETRY_VERTEX_H
#define ARE_INCLUDE_GEOMETRY_VERTEX_H

#include <are/core/types.h>

namespace are {

/**
 * @struct Vertex
 * @brief Standard vertex structure for mesh data
 * 
 * Contains position, normal, texture coordinates, and tangent information
 * for PBR rendering pipeline.
 */
struct Vertex {
    Vec3 position_;///< Vertex position in object space
    Vec3 normal_;                         ///< Vertex normal (normalized)
    Vec2 texcoord_;                       ///< Texture coordinates (UV)
    Vec3 tangent_;                        ///< Tangent vector for normal mapping

    Vertex() = default;

    /**
     * @brief Construct vertex with position only
     * @param pos Position
     */
    explicit Vertex(const Vec3& pos);

    /**
     * @brief Construct vertex with position and normal
     * @param pos Position
     * @param norm Normal
     */
    Vertex(const Vec3& pos, const Vec3& norm);

    /**
     * @brief Construct vertex with position, normal, and texcoord
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv);

    /**
     * @brief Construct vertex with all attributes
     * @param pos Position
     * @param norm Normal
     * @param uv Texture coordinates
     * @param tan Tangent
     */
    Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv, const Vec3& tan);

    /**
     * @brief Interpolate between two vertices
     * @param a First vertex
     * @param b Second vertex
     * @param t Interpolation factor [0, 1]
     * @return Interpolated vertex
     */
    static Vertex lerp(const Vertex& a, const Vertex& b, Real t);
};

/**
 * @brief Get vertex attribute stride for OpenGL
 * @return Size of Vertex structure in bytes
 */
constexpr size_t get_vertex_stride() {
    return sizeof(Vertex);
}

/**
 * @brief Get offset of position attribute
 * @return Offset in bytes
 */
constexpr size_t get_position_offset() {
    return offsetof(Vertex, position_);
}

/**
 * @brief Get offset of normal attribute
 * @return Offset in bytes
 */
constexpr size_t get_normal_offset() {
    return offsetof(Vertex, normal_);
}

/**
 * @brief Get offset of texcoord attribute
 * @return Offset in bytes
 */
constexpr size_t get_texcoord_offset() {
    return offsetof(Vertex, texcoord_);
}

/**
 * @brief Get offset of tangent attribute
 * @return Offset in bytes
 */
constexpr size_t get_tangent_offset() {
    return offsetof(Vertex, tangent_);
}

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_VERTEX_H
