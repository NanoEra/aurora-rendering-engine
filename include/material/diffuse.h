#ifndef ARE_INCLUDE_MATERIAL_DIFFUSE_H
#define ARE_INCLUDE_MATERIAL_DIFFUSE_H

#include <basic/vec3.h>
#include <basic/plane.h>
#include <material/material.h>

namespace are {

// Diffuse material class, derived from class Material.
class Diffuse : public Material {
public:
	// Constructor to initialize a Diffuse object.
	Diffuse() = default;

	// Destructors
	~Diffuse() override = default;

	// Calculate the new viewport origin after reflection.
	bool reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const override;
};

}

#endif
