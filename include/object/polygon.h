#ifndef ARE_INCLUDE_OBJECT_POLYGON_H
#define ARE_INCLUDE_OBJECT_POLYGON_H

#include <basic/plane.h>
#include <basic/ray.h>
#include <basic/vec3.h>
#include <material/material.h>
#include <object/object.h>
#include <texture.h>
#include <vector>

namespace are {

// Polygon object class, derived from class Object.
class Polygon : public Object {
public:
	// Constructor to initialize a Polygon object.
	Polygon() = delete;
	Polygon(const std::vector<Point3> &vertices, Material *material, Texture *texture);

	// Destructors
	~Polygon() override = default;

	// Function to check if a point is inside the polygon.
	bool point_in(const Point3 &point) const override;

	// Function to check ray-polygon intersection.
	bool intersect_ray(const Ray &ray, Point3 &hit_point) const override;

	// Function to trace texture for polygon objects.
	Texture trace_texture(const ObjectSet &object_set, const Point3 &viewport_origin_point) const override;

	// Object properties
	std::vector<Point3> vertices_; // Vertices of the polygon
	Plane plane_; // The plane that based on

private:
	// Graphic properties
	Material *material_;
	Texture *texture_;
};

}

#endif
