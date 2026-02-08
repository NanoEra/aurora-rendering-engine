/**
 * @file camera.h
 * @brief Camera class for view and projection management
 */

#ifndef ARE_INCLUDE_SCENE_CAMERA_H
#define ARE_INCLUDE_SCENE_CAMERA_H

#include <are/core/types.h>

namespace are {

/**
 * @class Camera
 * @brief Perspective camera for rendering
 * 
 * Manages view and projection matrices, and provides ray generation
 * for ray tracing.
 */
class Camera {
public:
    /**
     * @brief Default constructor
     */
    Camera();

    /**
     * @brief Construct camera with position and target
     * @param position Camera position
     * @param target Look-at target
     * @param up Up vector
     */
    Camera(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Position and orientation setters
    void set_position(const Vec3& position);
    void set_target(const Vec3& target);
    void set_up(const Vec3& up);
    void look_at(const Vec3& position, const Vec3& target, const Vec3& up = Vec3(0, 1, 0));

    // Projection setters
    void set_fov(Real fov_degrees);
    void set_aspect_ratio(Real aspect);
    void set_near_plane(Real near);
    void set_far_plane(Real far);
    void set_perspective(Real fov_degrees, Real aspect, Real near, Real far);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_target() const { return target_; }
    const Vec3& get_up() const { return up_; }
    Real get_fov() const { return fov_; }
    Real get_aspect_ratio() const { return aspect_ratio_; }
    Real get_near_plane() const { return near_plane_; }
    Real get_far_plane() const { return far_plane_; }

    // Direction vectors
    Vec3 get_forward() const;
    Vec3 get_right() const;

    // Matrix getters
    const Mat4& get_view_matrix() const;
    const Mat4& get_projection_matrix() const;
    Mat4 get_view_projection_matrix() const;

    /**
     * @brief Generate ray for given pixel coordinates
     * @param u Normalized x coordinate [0, 1]
     * @param v Normalized y coordinate [0, 1]
     * @param origin Output ray origin
     * @param direction Output ray direction (normalized)
     */
    void generate_ray(Real u, Real v, Vec3& origin, Vec3& direction) const;

    /**
     * @brief Check if camera parameters have changed
     * @return true if matrices need recalculation
     */
    bool is_dirty() const { return dirty_; }

    /**
     * @brief Mark camera as clean after matrix update
     */
    void clear_dirty() { dirty_ = false; }

private:
    void update_view_matrix() const;
    void update_projection_matrix() const;

    Vec3 position_;                       ///< Camera position
    Vec3 target_;                         ///< Look-at target
    Vec3 up_;                             ///< Up vector

    Real fov_;                            ///< Field of view in degrees
    Real aspect_ratio_;                   ///< Aspect ratio (width/height)
    Real near_plane_;                     ///< Near clipping plane
    Real far_plane_;                      ///< Far clipping plane

    mutable Mat4 view_matrix_;            ///< Cached view matrix
    mutable Mat4 projection_matrix_;      ///< Cached projection matrix
    mutable bool view_dirty_;             ///< View matrix needs update
    mutable bool projection_dirty_;       ///< Projection matrix needs update
    bool dirty_;                          ///< Any parameter changed
};

} // namespace are

#endif // ARE_INCLUDE_SCENE_CAMERA_H
