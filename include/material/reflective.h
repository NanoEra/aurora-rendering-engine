#ifndef ARE_INCLUDE_MATERIAL_REFLECTIVE_H
#define ARE_INCLUDE_MATERIAL_REFLECTIVE_H

#include <basic/vec3.h>
#include <basic/plane.h>
#include <material/material.h>

namespace are {

// Reflective material class, derived from class Material.
class Reflective : public Material {
public:
	// Constructor to initialize a Reflective object.
	Reflective() = delete;
	Reflective(double reflectivity);

	// Destructors
	~Reflective() override = default;

	// Calculate the new viewport origin after reflection.
	bool reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const override;

	double reflectivity_;
private:
};

}

#endif
