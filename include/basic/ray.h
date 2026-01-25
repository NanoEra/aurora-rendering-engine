#ifndef ARE_INCLUDE_BASIC_RAY_H
#define ARE_INCLUDE_BASIC_RAY_H

#include <basic/vec3.h>

namespace are {

struct Ray {
	Point3 Q;
	Vec3 D;

	// Constructors
	Ray() = default;
	Ray(const Point3 &origin, const Vec3 &direction); // direction should be normalized

	// Get point at geometric distance t along the ray
	Point3 at(double t);
};

}

#endif
