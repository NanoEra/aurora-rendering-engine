/**
 * @file vertex.cpp
 * @brief Implementation of vertex structure
 */

#include <are/geometry/vertex.h>

namespace are {

Vertex::Vertex(const Vec3& pos) 
    : position_(pos)
    , normal_(0.0f, 1.0f, 0.0f)
    , texcoord_(0.0f, 0.0f)
    , tangent_(1.0f, 0.0f, 0.0f) {
}

Vertex::Vertex(const Vec3& pos, const Vec3& norm)
    : position_(pos)
    , normal_(norm)
    , texcoord_(0.0f, 0.0f)
    , tangent_(1.0f, 0.0f, 0.0f) {
}

Vertex::Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv)
    : position_(pos)
    , normal_(norm)
    , texcoord_(uv)
    , tangent_(1.0f, 0.0f, 0.0f) {
}

Vertex::Vertex(const Vec3& pos, const Vec3& norm, const Vec2& uv, const Vec3& tan)
    : position_(pos)
    , normal_(norm)
    , texcoord_(uv)
    , tangent_(tan) {
}

Vertex Vertex::lerp(const Vertex& a, const Vertex& b, Real t) {
    Vertex result;
    result.position_ = glm::mix(a.position_, b.position_, t);
    result.normal_ = glm::normalize(glm::mix(a.normal_, b.normal_, t));
    result.texcoord_ = glm::mix(a.texcoord_, b.texcoord_, t);
    result.tangent_ = glm::normalize(glm::mix(a.tangent_, b.tangent_, t));
    return result;
}

} // namespace are
