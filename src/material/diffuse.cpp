#include <material/diffuse.h>

namespace are {

bool Diffuse::reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const {
	return false;
}

}
