#include <object/object.h>

namespace are {

Object::~Object() = default;

bool Object::point_in(const Point3 &point) const {
	return false;
}

bool Object::intersect_ray(const Ray &ray, Point3 &hit_point) const {
	return false;
}

Texture Object::trace_texture(const ObjectSet &object_set, const Point3 &viewport_origin_point) const {
	return Texture();
}

}
