#include <basic/ray.h>

namespace are {

Ray::Ray(const Point3 &origin, const Vec3 &direction) : Q(origin), D(direction.normalized()) {
}

Point3 Ray::at(double t) {
	return Q + D.normalized() * t;
}

}
