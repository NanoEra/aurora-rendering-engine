#ifndef ARE_INCLUDE_MATERIAL_MATERIAL_H
#define ARE_INCLUDE_MATERIAL_MATERIAL_H

#include <basic/plane.h>
#include <basic/vec3.h>

namespace are {

// Material base cass
class Material {
protected:
	// Declare the constructor in protected section.
	Material() = default;

public:
	// Constructors
	Material(const Material &) = delete; // Delete the copy constructor
	Material &operator=(const Material &) = delete;
	Material(Material &) = delete; // Delete the move constructor
	Material &operator=(Material &) = delete;

	// Destructor
	virtual ~Material();

	// Virtual function interface declaration

	// Calculate the new viewport origin after reflection.
	virtual bool reflect(const Plane &viewport_plane, const Point3 &viewport_origin, Point3 &new_viewport_origin) const;
};

}

#endif
