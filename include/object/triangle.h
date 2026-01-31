#ifndef ARE_INCLUDE_OBJECT_TRIANGLE_H
#define ARE_INCLUDE_OBJECT_TRIANGLE_H

#include <basic/plane.h>
#include <basic/ray.h>
#include <basic/vec3.h>
#include <material/material.h>
#include <object/object.h>
#include <texture.h>
#include <vector>

namespace are {

// Triangle object class, derived from class Object.
class Triangle : public Object {
public:
	// Constructor to initialize a Triangle object.
	Triangle() = delete;
	Triangle(const Point3 &Q, const Vec3 &u, const Vec3 &v, Material *material, Texture *texture);

	// Destructors
	~Triangle() override = default;

	// Function to check if a point is inside the triangle.
	bool point_in(const Point3 &point) const override;

	// Function to check ray-triangle intersection.
	bool intersect_ray(const Ray &ray, Point3 &hit_point) const override;

	// Function to trace texture for triangle objects.
	Texture trace_texture(const ObjectSet &object_set, const Point3 &viewport_origin_point) const override;

	// Function to get the vertices of the triangle.
	const std::vector<Point3> &get_vertices() const;

private:
	// Graphic properties
	Material *material_;
	Texture *texture_;

	// Object properties
	Point3 Q; // One vertex of the triangle.
	Vec3 u, v; // Edge vectors from vertex Q.

	std::vector<Point3> vertices_; // Vertices of the triangle.
};

}

#endif
