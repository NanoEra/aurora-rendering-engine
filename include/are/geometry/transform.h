/**
 * @file transform.h
 * @brief Transformation matrix utilities
 */

#ifndef ARE_INCLUDE_GEOMETRY_TRANSFORM_H
#define ARE_INCLUDE_GEOMETRY_TRANSFORM_H

#include <are/core/types.h>

namespace are {

/**
 * @class Transform
 * @brief 3D transformation (position, rotation, scale)
 * 
 * Provides convenient interface for building transformation matrices.
 */
class Transform {
public:
    /**
     * @brief Default constructor (identity transform)
     */
    Transform();

    /**
     * @brief Construct from position, rotation, and scale
     * @param position Translation
     * @param rotation Rotation (Euler angles in radians)
     * @param scale Scale factors
     */
    Transform(const Vec3& position, const Vec3& rotation, const Vec3& scale);

    // Setters
    void set_position(const Vec3& position);
    void set_rotation(const Vec3& rotation);
    void set_scale(const Vec3& scale);
    void set_scale(Real uniform_scale);

    // Getters
    const Vec3& get_position() const { return position_; }
    const Vec3& get_rotation() const { return rotation_; }
    const Vec3& get_scale() const { return scale_; }

    // Matrix operations
    Mat4 get_matrix() const;
    Mat4 get_inverse_matrix() const;
    Mat3 get_normal_matrix() const;

    /**
     * @brief Transform a point
     * @param point Point to transform
     * @return Transformed point
     */
    Vec3 transform_point(const Vec3& point) const;

    /**
     * @brief Transform a direction (ignores translation)
     * @param direction Direction to transform
     * @return Transformed direction
     */
    Vec3 transform_direction(const Vec3& direction) const;

    /**
     * @brief Transform a normal (uses inverse transpose)
     * @param normal Normal to transform
     * @return Transformed normal
     */
    Vec3 transform_normal(const Vec3& normal) const;

    /**
     * @brief Combine two transforms
     * @param other Other transform
     * @return Combined transform
     */
    Transform operator*(const Transform& other) const;

    /**
     * @brief Create identity transform
     * @return Identity transform
     */
    static Transform identity();

    /**
     * @brief Create translation transform
     * @param translation Translation vector
     * @return Translation transform
     */
    static Transform translate(const Vec3& translation);

    /**
     * @brief Create rotation transform
     * @param rotation Rotation (Euler angles in radians)
     * @return Rotation transform
     */
    static Transform rotate(const Vec3& rotation);

    /**
     * @brief Create scale transform
     * @param scale Scale factors
     * @return Scale transform
     */
    static Transform scale(const Vec3& scale);

private:
    void mark_dirty();
    void update_matrix() const;

    Vec3 position_;                       ///< Translation
    Vec3 rotation_;                       ///< Rotation (Euler angles)
    Vec3 scale_;                          ///< Scale factors

    mutable Mat4 matrix_;                 ///< Cached transformation matrix
    mutable Mat4 inverse_matrix_;         ///< Cached inverse matrix
    mutable bool dirty_;                  ///< Matrix needs update
};

} // namespace are

#endif // ARE_INCLUDE_GEOMETRY_TRANSFORM_H
