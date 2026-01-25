#include <material/material.h>

namespace are {

Material::~Material() = default;

bool Material::reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const {
	return false;
}

}
