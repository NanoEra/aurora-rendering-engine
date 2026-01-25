#ifndef ARE_INCLUDE_OBJECT_OBJECT_H
#define ARE_INCLUDE_OBJECT_OBJECT_H

#include <basic/ray.h>
#include <basic/vec3.h>
#include <texture.h>

namespace are {

// The declaration of ObjectSet to avoid circular dependency(forward declaration), please see object_set.h for the detailed definition.
struct ObjectSet;

// Object base class
class Object {
protected:
	// Declare the constructor in protected section.
	Object() = default;

public:
	// Constructors
	Object(const Object &) = delete; // Delete the copy constructor
	Object &operator=(const Object &) = delete;
	Object(Object &) = delete; // Delete the move constructor
	Object &operator=(Object &) = delete;

	// Destructor
	virtual ~Object();

	// Virtual function interface declaration

	// Function to check if a point is inside the polygon.
	virtual bool point_in(const Point3 &point) const;

	// Function to check ray-quad intersection.
	virtual bool intersect_ray(const Ray &ray, Point3 &hit_point) const;

	// Function to trace texture for polygon objects.
	virtual Texture trace_texture(const ObjectSet &object_set, const Point3 &viewport_origin_point) const;
};

}

#endif
