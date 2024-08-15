#pragma once

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace amcl::math
{

/**
 * A rigid 2D trasformation
 *
 * \tparam T Scalar type
 *
 */
template <typename T>
class Transform2D
{
  private:
    Eigen::Matrix<T, 2, 1> m_translation; /**< Translation vector */
    Eigen::Rotation2D<T> m_rotation;      /**< Rotation */

  public:
    /**
     * Constructs the identity transformation.
     */
    Transform2D();

    /** Copy constructor.
     */
    Transform2D(const Transform2D& t_transform);

    /** Copy assignment.
     */
    Transform2D& operator=(const Transform2D& t_transform);

    /** Destructor.
     */
    ~Transform2D();

    /**
     * Construct a transformation from an Eigen vector and a rotation.
     *
     * \param t_translation an Eigen object containing a translation as a column vector
     * \param t_rotation An Eigen rotation object
     *
     */
    explicit Transform2D(const Eigen::Matrix<T, 2, 1>& t_translation, const Eigen::Rotation2D<T>& t_rotation);

    /**
     * Construct a transformation from an homogeneous matrix.
     *
     * \param[in] t_matrix Homogeneous matrix of the transformation.
     *
     * Assumes that the given matrix corresponds to an isometry Transformation.
     */
    explicit Transform2D(const Eigen::Matrix<T, 3, 3>& t_matrix);

    /**
     * Obtain an homogeneous matrix for the transformation.
     *
     * \return homogeneous matrix of the transformation.
     */
    Eigen::Matrix<T, 3, 3> homogeneous() const;

    /**
     * Get mutable rotation component.
     *
     * \return rotation component as a reference to a Rotation2D.
     */
    Eigen::Rotation2D<T>& rotation();

    /**
     * Get immutable rotation component.
     *
     * \return rotation component as a Rotation2D.
     */
    const Eigen::Rotation2D<T>& rotation() const;

    /**
     * Get mutable translation component.
     *
     * \return Translation component as a reference to a column vector.
     */
    Eigen::Matrix<T, 2, 1>& translation();

    /**
     * Get immutable translation component.
     *
     * \return Translation component as a column vector.
     */
    const Eigen::Matrix<T, 2, 1>& translation() const;

    bool operator==(const Transform2D& t_transform) const;

    bool operator!=(const Transform2D& t_transform) const;

    Transform2D operator*(const Transform2D& t_transform) const;
};

/** Apply a 2D transform to N vectors arranged as a 2xN matrix
 */
template <typename T, int N>
Eigen::Matrix<T, 2, N> operator*(const Transform2D<T>& t_transform, const Eigen::Matrix<T, 2, N>& t_matrix);

/** Commodity function to explicitly return an identity transform
 */
template <typename T>
Transform2D<T> identityTransform2D();

template <typename T>
Transform2D<T> inverse(const Transform2D<T>& t_transform);

/** Read a transformation from a stream
 */
template <typename T>
std::istream& operator>>(std::istream& t_stream, Transform2D<T>& t_transform);

/** Write a transformation to a stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& t_stream, const Transform2D<T>& t_transform);

}  // namespace amcl::math

#include "amcl/math.hpp"