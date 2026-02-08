/**
 * @file random.cpp
 * @brief Implementation of random number generation
 */

#include <are/utils/random.h>
#include <glm/gtc/constants.hpp>
#include <random>
#include <cmath>

namespace are {

RandomGenerator::RandomGenerator(uint64_t seed) 
    : dist_(0.0f, 1.0f) {
    if (seed == 0) {
        std::random_device rd;
        seed = rd();
    }
    rng_.seed(seed);
}

Real RandomGenerator::random_float() {
    return dist_(rng_);
}

Real RandomGenerator::random_float(Real min, Real max) {
    return min + (max - min) * random_float();
}

int RandomGenerator::random_int(int min, int max) {
    std::uniform_int_distribution<int> int_dist(min, max);
    return int_dist(rng_);
}

Vec3 RandomGenerator::random_in_unit_disk() {
    while (true) {
        Vec3 p(random_float(-1.0f, 1.0f), random_float(-1.0f, 1.0f), 0.0f);
        if (glm::dot(p, p) < 1.0f) {
            return p;
        }
    }
}

Vec3 RandomGenerator::random_in_unit_sphere() {
    while (true) {
        Vec3 p(random_float(-1.0f, 1.0f), 
               random_float(-1.0f, 1.0f), 
               random_float(-1.0f, 1.0f));
        if (glm::dot(p, p) < 1.0f) {
            return p;
        }
    }
}

Vec3 RandomGenerator::random_unit_vector() {
    return glm::normalize(random_in_unit_sphere());
}

Vec3 RandomGenerator::random_in_hemisphere(const Vec3& normal) {
    Vec3 in_unit_sphere = random_in_unit_sphere();
    if (glm::dot(in_unit_sphere, normal) > 0.0f) {
        return in_unit_sphere;
    } else {
        return -in_unit_sphere;
    }
}

Vec3 RandomGenerator::random_cosine_direction(const Vec3& normal) {
    // Generate random direction with cosine-weighted distribution
    Real r1 = random_float();
    Real r2 = random_float();
    
    Real phi = 2.0f * glm::pi<Real>() * r1;
    Real cos_theta = std::sqrt(r2);
    Real sin_theta = std::sqrt(1.0f - r2);
    
    Vec3 local_dir(
        std::cos(phi) * sin_theta,
        std::sin(phi) * sin_theta,
        cos_theta
    );
    
    // Transform to world space
    Vec3 tangent, bitangent;
    Vec3 up = (std::abs(normal.y) < 0.999f) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    tangent = glm::normalize(glm::cross(up, normal));
    bitangent = glm::cross(normal, tangent);
    
    return local_dir.x * tangent + local_dir.y * bitangent + local_dir.z * normal;
}

void RandomGenerator::set_seed(uint64_t seed) {
    rng_.seed(seed);
}

// Thread-local random generator
RandomGenerator& get_thread_random() {
    thread_local RandomGenerator generator;
    return generator;
}

} // namespace are
