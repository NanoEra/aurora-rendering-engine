#ifndef ARE_INCLUDE_SCENE_CAMERA_H
#define ARE_INCLUDE_SCENE_CAMERA_H

#include "basic/types.h"

namespace are {

// Camera projection type
enum class ProjectionType {
	PERSPECTIVE,
	ORTHOGRAPHIC
};

// Camera for rendering
class Camera {
public:
	// Constructor
	Camera();

	// Destructor
	~Camera();

	/*
	 * @brief Set perspective projection
	 * @param fov Field of view in degrees
	 * @param aspect Aspect ratio
	 * @param near Near plane
	 * @param far Far plane
	 */
	void set_perspective(float fov, float aspect, float near, float far);

	/*
	 * @brief Set orthographic projection
	 * @param left Left plane
	 * @param right Right plane
	 * @param bottom Bottom plane
	 * @param top Top plane
	 * @param near Near plane
	 * @param far Far plane
	 */
	void set_orthographic(float left, float right, float bottom, float top, float near, float far);

	/*
	 * @brief Set camera position
	 * @param position Position
	 */
	void set_position(const Vec3 &position);

	/*
	 * @brief Set camera target
	 * @param target Target position
	 */
	void set_target(const Vec3 &target);

	/*
	 * @brief Set camera up vector
	 * @param up Up vector
	 */
	void set_up(const Vec3 &up);

	/*
	 * @brief Get view matrix
	 * @return View matrix
	 */
	Mat4 get_view_matrix() const;

	/*
	 * @brief Get projection matrix
	 * @return Projection matrix
	 */
	Mat4 get_projection_matrix() const;

	/*
	 * @brief Get view-projection matrix
	 * @return View-projection matrix
	 */
	Mat4 get_view_projection_matrix() const;

	/*
	 * @brief Get camera position
	 * @return Position
	 */
	const Vec3 &get_position() const {
		return position_;
	}

	/*
	 * @brief Get camera forward direction
	 * @return Forward direction
	 */
	Vec3 get_forward() const;

	/*
	 * @brief Get camera right direction
	 * @return Right direction
	 */
	Vec3 get_right() const;

	/*
	 * @brief Get camera up direction
	 * @return Up direction
	 */
	Vec3 get_up() const;

private:
	Vec3 position_;
	Vec3 target_;
	Vec3 up_;

	ProjectionType projection_type_;

	// Perspective parameters
	float fov_;
	float aspect_;

	// Orthographic parameters
	float left_, right_, bottom_, top_;

	// Common parameters
	float near_;
	float far_;

	mutable Mat4 view_matrix_;
	mutable Mat4 projection_matrix_;
	mutable bool view_dirty_;
	mutable bool projection_dirty_;
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_CAMERA_H
