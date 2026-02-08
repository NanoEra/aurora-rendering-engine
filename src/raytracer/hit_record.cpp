/**
 * @file hit_record.cpp
 * @brief Implementation of hit record
 */

#include <are/raytracer/hit_record.h>
#include <glm/glm.hpp>

namespace are {

HitRecord::HitRecord()
	: position_(0.0f)
	, normal_(0.0f, 1.0f, 0.0f)
	, texcoord_(0.0f)
	, tangent_(1.0f, 0.0f, 0.0f)
	, t_(-1.0f)
	, material_(are_invalid_handle)
	, triangle_index_(0)
	, front_face_(true) {
}

void HitRecord::set_face_normal(const Vec3 &ray_direction, const Vec3 &outward_normal) {
	// Determine if ray hit front face or back face
	front_face_ = glm::dot(ray_direction, outward_normal) < 0.0f;

	// Normal always points against ray direction
	normal_ = front_face_ ? outward_normal : -outward_normal;
}

bool HitRecord::is_valid() const {
	return t_ >= 0.0f && material_ != are_invalid_handle;
}

} // namespace are
