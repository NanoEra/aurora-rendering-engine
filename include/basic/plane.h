#ifndef ARE_INCLUDE_BASIC_PLANE_H
#define ARE_INCLUDE_BASIC_PLANE_H

#include <basic/vec3.h>
#include <basic/ray.h>

namespace are {

// Plane represented in the form: normal * P + d = 0
struct Plane {
	Vec3 normal;
	double d;

	// Constructors
	Plane() = default;
	Plane(Vec3 normal_, double d_);
	Plane(const Point3 &p, const Vec3 &n);

	// Function to check ray-plane intersection.
	bool intersect_ray(const Ray &ray, Point3 &intersection) const;
};

}

#endif
