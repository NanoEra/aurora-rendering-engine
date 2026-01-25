#include <basic/math.h>
#include <basic/plane.h>
#include <cstdlib>

namespace are {

Plane::Plane(Vec3 normal_, double d_) : normal(normal_), d(d_) {
}

Plane::Plane(const Point3 &p, const Vec3 &n) : normal(n.normalized()), d(-normal.dot(p)) {
}

bool Plane::intersect_ray(const Ray &ray, Point3 &intersection) const {
	double denom = normal.dot(ray.D);
	if (std::abs(denom) < GEOMETRY_EPSILON) {
		return false;
	}

	double t = -(normal.dot(ray.Q) + d) / denom;
	if (t < GEOMETRY_EPSILON) {
		return false;
	}

	intersection = ray.Q + t * ray.D;

	return true;
}

}
