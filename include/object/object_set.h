#ifndef ARE_INCLUDE_OBJECT_OBJECT_SET_H
#define ARE_INCLUDE_OBJECT_OBJECT_SET_H

#include <object/object.h>
#include <object/triangle.h>

namespace are {

// Unified container for all objects.
struct ObjectSet {
	std::vector<Triangle *> triangles;
};

}

#endif
