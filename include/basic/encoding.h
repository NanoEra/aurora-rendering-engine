#ifndef ARE_INCLUDE_BASIC_ENCODING_H
#define ARE_INCLUDE_BASIC_ENCODING_H

#include "basic/types.h"
#include <cmath>

namespace are {

/*
 * @brief Octahedral encode a unit vector to 2D coordinates
 * @param n Normalized 3D vector
 * @return 2D encoded coordinates in [0, 1] range
 */
inline Vec2 oct_encode(Vec3 n) {
	float sum = std::abs(n.x) + std::abs(n.y) + std::abs(n.z);
	n /= sum;

	if (n.z < 0.0f) {
		float x = n.x;
		float y = n.y;
		n.x = (1.0f - std::abs(y)) * (x >= 0.0f ? 1.0f : -1.0f);
		n.y = (1.0f - std::abs(x)) * (y >= 0.0f ? 1.0f : -1.0f);
	}

	return Vec2(n.x, n.y) * 0.5f + 0.5f;
}

/*
 * @brief Octahedral decode 2D coordinates to unit vector
 * @param f 2D encoded coordinates in [0, 1] range
 * @return Normalized 3D vector
 */
inline Vec3 oct_decode(Vec2 f) {
	f = f * 2.0f - 1.0f;

	Vec3 n = Vec3(f.x, f.y, 1.0f - std::abs(f.x) - std::abs(f.y));
	float t = std::max(-n.z, 0.0f);
	n.x += n.x >= 0.0f ? -t : t;
	n.y += n.y >= 0.0f ? -t : t;

	return glm::normalize(n);
}

} // namespace are

#endif // ARE_INCLUDE_BASIC_ENCODING_H
