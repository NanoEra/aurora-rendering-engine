#ifndef ARE_INCLUDE_SCENE_LIGHT_H
#define ARE_INCLUDE_SCENE_LIGHT_H

#include "basic/types.h"

namespace are {

// Light type enumeration
enum class LightType {
	DIRECTIONAL = 0,
	POINT = 1,
	SPOT = 2
};

// Light source
class Light {
public:
	// Constructor
	Light();

	// Destructor
	~Light();

	/*
	 * @brief Set light type
	 * @param type Light type
	 */
	void set_type(LightType type);

	/*
	 * @brief Set light position (for point and spot lights)
	 * @param position Light position
	 */
	void set_position(const Vec3 &position);

	/*
	 * @brief Set light direction (for directional and spot lights)
	 * @param direction Light direction
	 */
	void set_direction(const Vec3 &direction);

	/*
	 * @brief Set light color
	 * @param color Light color
	 */
	void set_color(const Vec3 &color);

	/*
	 * @brief Set light intensity
	 * @param intensity Light intensity
	 */
	void set_intensity(float intensity);

	/*
	 * @brief Set light range (for point and spot lights)
	 * @param range Light range
	 */
	void set_range(float range);

	/*
	 * @brief Set spot light angles
	 * @param inner_angle Inner cone angle in degrees
	 * @param outer_angle Outer cone angle in degrees
	 */
	void set_spot_angles(float inner_angle, float outer_angle);

	/*
	 * @brief Get light type
	 * @return Light type
	 */
	LightType get_type() const {
		return type_;
	}

	/*
	 * @brief Get light position
	 * @return Light position
	 */
	const Vec3 &get_position() const {
		return position_;
	}

	/*
	 * @brief Get light direction
	 * @return Light direction
	 */
	const Vec3 &get_direction() const {
		return direction_;
	}

	/*
	 * @brief Get light color
	 * @return Light color
	 */
	const Vec3 &get_color() const {
		return color_;
	}

	/*
	 * @brief Get light intensity
	 * @return Light intensity
	 */
	float get_intensity() const {
		return intensity_;
	}

	/*
	 * @brief Get light range
	 * @return Light range
	 */
	float get_range() const {
		return range_;
	}

	/*
	 * @brief Get spot light inner angle
	 * @return Inner angle in radians
	 */
	float get_inner_angle() const {
		return inner_angle_;
	}

	/*
	 * @brief Get spot light outer angle
	 * @return Outer angle in radians
	 */
	float get_outer_angle() const {
		return outer_angle_;
	}

private:
	LightType type_;
	Vec3 position_;
	Vec3 direction_;
	Vec3 color_;
	float intensity_;
	float range_;
	float inner_angle_;
	float outer_angle_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_LIGHT_H
