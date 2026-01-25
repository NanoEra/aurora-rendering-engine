#ifndef ARE_INCLUDE_OBJECT_OBJECT_SET_H
#define ARE_INCLUDE_OBJECT_OBJECT_SET_H

#include <object/object.h>
#include <object/polygon.h>

namespace are {

// Unified container for all objects.
struct ObjectSet {
	std::vector<Polygon *> object_set;
};

}

#endif
