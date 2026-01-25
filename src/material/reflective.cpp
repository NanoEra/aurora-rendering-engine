#include <basic/math.h>
#include <material/reflective.h>

namespace are {

Reflective::Reflective(double reflectivity) : reflectivity_(reflectivity) {
}

bool Reflective::reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const {
	// Calculate the denominator
    double denom = viewport_plane.normal.length_squared();
    if (denom < GEOMETRY_EPSILON) { // Illegal plane
        return false; 
    }

	// Calculate the numerator
    double numer = viewport_plane.normal.dot(viewport_origin) + viewport_plane.d;

	// Calculate the k value
    double k = 2.0 * numer / denom;

    // New viewport origin point
    new_viewport_origin = viewport_origin - k * viewport_plane.normal;

    return true;
}

}
