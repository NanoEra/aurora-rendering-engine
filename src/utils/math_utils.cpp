/**
 * @file math_utils.cpp
 * @brief Implementation of mathematical utility functions
 */

#include <are/utils/math_utils.h>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

namespace are {

void compute_barycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c,
                        Real& u, Real& v, Real& w) {
    Vec3 v0 = b - a;
    Vec3 v1 = c - a;
    Vec3 v2 = p - a;
    
    Real d00 = glm::dot(v0, v0);
    Real d01 = glm::dot(v0, v1);
    Real d11 = glm::dot(v1, v1);
    Real d20 = glm::dot(v2, v0);
    Real d21 = glm::dot(v2, v1);
    
    Real denom = d00 * d11 - d01 * d01;
    
    if (std::abs(denom) < are_epsilon) {
        // Degenerate triangle
        u = v = w = 1.0f / 3.0f;
        return;
    }
    
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

Vec3 reflect(const Vec3& incident, const Vec3& normal) {
    return incident - 2.0f * glm::dot(incident, normal) * normal;
}

bool refract(const Vec3& incident, const Vec3& normal, Real eta, Vec3& refracted) {
    Real cos_theta = glm::dot(-incident, normal);
    Real sin_theta_sq = 1.0f - cos_theta * cos_theta;
    Real sin_theta_t_sq = eta * eta * sin_theta_sq;
    
    // Total internal reflection
    if (sin_theta_t_sq > 1.0f) {
        return false;
    }
    
    Real cos_theta_t = std::sqrt(1.0f - sin_theta_t_sq);
    refracted = eta * incident + (eta * cos_theta - cos_theta_t) * normal;
    
    return true;
}

Real fresnel_schlick(Real cos_theta, Real f0) {
    Real f = 1.0f - cos_theta;
    Real f2 = f * f;
    Real f5 = f2 * f2 * f;
    return f0 + (1.0f - f0) * f5;
}

void create_orthonormal_basis(const Vec3& normal, Vec3& tangent, Vec3& bitangent) {
    // Choose a vector not parallel to normal
    Vec3 up = (std::abs(normal.y) < 0.999f) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    
    tangent = glm::normalize(glm::cross(up, normal));
    bitangent = glm::cross(normal, tangent);
}

Vec3 tangent_to_world(const Vec3& tangent_dir, const Vec3& normal, 
                     const Vec3& tangent, const Vec3& bitangent) {
    return tangent_dir.x * tangent + 
           tangent_dir.y * bitangent + 
           tangent_dir.z * normal;
}

} // namespace are
