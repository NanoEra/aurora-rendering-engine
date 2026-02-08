/**
 * @file triangle.cpp
 * @brief Implementation of Triangle primitive
 */

#include <are/core/logger.h>
#include <are/core/profiler.h>
#include <are/geometry/triangle.h>
#include <are/raytracer/hit_record.h>
#include <are/raytracer/ray.h>
#include <glm/glm.hpp>

namespace are {

Triangle::Triangle()
	: material_(are_invalid_handle) {
}

Triangle::Triangle(const Vertex &v0, const Vertex &v1, const Vertex &v2, MaterialHandle material)
	: v0_(v0)
	, v1_(v1)
	, v2_(v2)
	, material_(material) {
}

Vec3 Triangle::centroid() const {
	return (v0_.position_ + v1_.position_ + v2_.position_) / 3.0f;
}

Vec3 Triangle::normal() const {
	Vec3 edge1 = v1_.position_ - v0_.position_;
	Vec3 edge2 = v2_.position_ - v0_.position_;
	return glm::normalize(glm::cross(edge1, edge2));
}

Real Triangle::area() const {
	Vec3 edge1 = v1_.position_ - v0_.position_;
	Vec3 edge2 = v2_.position_ - v0_.position_;
	return 0.5f * glm::length(glm::cross(edge1, edge2));
}

AABB Triangle::compute_aabb() const {
	AABB aabb(v0_.position_);
	aabb.expand(v1_.position_);
	aabb.expand(v2_.position_);
	return aabb;
}

bool Triangle::intersect(const Ray &ray, HitRecord &hit) const {
	ARE_PROFILE_FUNCTION();

	// Möller-Trumbore algorithm
	// Reference: "Fast, Minimum Storage Ray/Triangle Intersection"

	const Vec3 edge1 = v1_.position_ - v0_.position_;
	const Vec3 edge2 = v2_.position_ - v0_.position_;

	const Vec3 h = glm::cross(ray.direction_, edge2);
	const Real a = glm::dot(edge1, h);

	// Check if ray is parallel to triangle
	if (a > -are_epsilon && a < are_epsilon) {
		return false;
	}

	const Real f = 1.0f / a;
	const Vec3 s = ray.origin_ - v0_.position_;
	const Real u = f * glm::dot(s, h);

	// Check barycentric coordinate u
	if (u < 0.0f || u > 1.0f) {
		return false;
	}

	const Vec3 q = glm::cross(s, edge1);
	const Real v = f * glm::dot(ray.direction_, q);

	// Check barycentric coordinate v
	if (v < 0.0f || u + v > 1.0f) {
		return false;
	}

	// Calculate t parameter
	const Real t = f * glm::dot(edge2, q);

	// Check if intersection is within ray bounds
	if (!ray.is_valid_t(t)) {
		return false;
	}

	// Fill hit record
	const Real w = 1.0f - u - v;

	hit.t_ = t;
	hit.position_ = ray.at(t);
	hit.material_ = material_;

	// Interpolate vertex attributes using barycentric coordinates
	hit.normal_ = glm::normalize(
		w * v0_.normal_ + u * v1_.normal_ + v * v2_.normal_);
	hit.texcoord_ = w * v0_.texcoord_ + u * v1_.texcoord_ + v * v2_.texcoord_;
	hit.tangent_ = glm::normalize(
		w * v0_.tangent_ + u * v1_.tangent_ + v * v2_.tangent_);

	// Determine front face
	hit.set_face_normal(ray.direction_, hit.normal_);

	return true;
}

bool Triangle::intersect_fast(const Ray &ray, Real t_max) const {
	ARE_PROFILE_FUNCTION();

	// Simplified Möller-Trumbore without hit record computation
	const Vec3 edge1 = v1_.position_ - v0_.position_;
	const Vec3 edge2 = v2_.position_ - v0_.position_;

	const Vec3 h = glm::cross(ray.direction_, edge2);
	const Real a = glm::dot(edge1, h);

	if (a > -are_epsilon && a < are_epsilon) {
		return false;
	}

	const Real f = 1.0f / a;
	const Vec3 s = ray.origin_ - v0_.position_;
	const Real u = f * glm::dot(s, h);

	if (u < 0.0f || u > 1.0f) {
		return false;
	}

	const Vec3 q = glm::cross(s, edge1);
	const Real v = f * glm::dot(ray.direction_, q);

	if (v < 0.0f || u + v > 1.0f) {
		return false;
	}

	const Real t = f * glm::dot(edge2, q);

	return t > ray.t_min_ && t < t_max;
}

} // namespace are
