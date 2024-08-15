#pragma once

#include "include/amcl/math.h"

template <typename T>
amcl::math::Transform2D<T>::Transform2D()
    : m_translation{Eigen::Matrix<T, 2, 1>::Zero()}, m_rotation{Eigen::Rotation2D<T>{0.}}
{
}

template <typename T>
amcl::math::Transform2D<T>::Transform2D(const Transform2D& t_transform) = default;

template <typename T>
amcl::math::Transform2D<T>& amcl::math::Transform2D<T>::operator=(const Transform2D& t_transform) = default;

template <typename T>
amcl::math::Transform2D<T>::~Transform2D() = default;

template <typename T>
amcl::math::Transform2D<T>::Transform2D(const Eigen::Matrix<T, 2, 1>& t_translation,
                                        const Eigen::Rotation2D<T>& t_rotation)
    : m_translation{t_translation}, m_rotation{t_rotation}
{
}

template <typename T>
amcl::math::Transform2D<T>::Transform2D(const Eigen::Matrix<T, 3, 3>& t_matrix)
    : m_translation{t_matrix.template block<2, 1>(0, 2)}, m_rotation{t_matrix.template block<2, 2>(0, 0)}
{
}

template <typename T>
Eigen::Matrix<T, 3, 3> amcl::math::Transform2D<T>::homogeneous() const
{
    Eigen::Matrix<T, 3, 3> mat = Eigen::Matrix<T, 3, 3>::Identity();
    mat.template block<2, 2>(0, 0) = m_rotation.toRotationMatrix();
    mat.template block<3, 1>(0, 2) = m_translation.homogeneous();
    return mat;
}

template <typename T>
Eigen::Rotation2D<T>& amcl::math::Transform2D<T>::rotation()
{
    return this->m_rotation;
}

template <typename T>
const Eigen::Rotation2D<T>& amcl::math::Transform2D<T>::rotation() const
{
    return this->m_rotation;
}

template <typename T>
Eigen::Matrix<T, 2, 1>& amcl::math::Transform2D<T>::translation()
{
    return this->m_translation;
}

template <typename T>
const Eigen::Matrix<T, 2, 1>& amcl::math::Transform2D<T>::translation() const
{
    return this->m_translation;
}

template <typename T>
bool amcl::math::Transform2D<T>::operator==(const Transform2D& t_transform) const
{
    return (this->m_translation == t_transform.m_translation) &&
           (this->m_rotation.angle() == t_transform.m_rotation.angle());
}

template <typename T>
bool amcl::math::Transform2D<T>::operator!=(const Transform2D& t_transform) const
{
    return (this->m_translation != t_transform.m_translation) ||
           (this->m_rotation.angle() != t_transform.m_rotation.angle());
}

template <typename T>
amcl::math::Transform2D<T> amcl::math::Transform2D<T>::operator*(const amcl::math::Transform2D<T>& t_transform) const
{
    return Transform2D<T>{this->m_rotation * t_transform.m_translation + this->m_translation,
                          (this->m_rotation * t_transform.m_rotation)};
}

template <typename T, int N>
Eigen::Matrix<T, 2, N> amcl::math::operator*(const amcl::math::Transform2D<T>& t_transform,
                                             const Eigen::Matrix<T, 2, N>& t_matrix)
{
    return (t_transform.rotation().toRotationMatrix() * t_matrix).colwise() + t_transform.translation();
}

template <typename T>
amcl::math::Transform2D<T> amcl::math::identityTransform2D()
{
    return Transform2D<T>{Eigen::Matrix<T, 2, 1>::Zero(), Eigen::Rotation2D<T>{0.}};
}

template <typename T>
amcl::math::Transform2D<T> amcl::math::inverse(const amcl::math::Transform2D<T>& t_transform)
{
    return Transform2D<T>{-(t_transform.rotation().inverse() * t_transform.translation()),
                          t_transform.rotation().inverse()};
}

/** Read a transform from a stream
 */
template <typename T>
std::istream& amcl::math::operator>>(std::istream& t_stream, amcl::math::Transform2D<T>& t_transform)
{
    Eigen::Matrix<T, 3, 3> mat;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            t_stream >> mat(i, j);
        }
    }

    t_transform = Transform2D<T>(mat);
    return t_stream;
}

/** Write a transform to a stream
 */
template <typename T>
std::ostream& amcl::math::operator<<(std::ostream& t_stream, const amcl::math::Transform2D<T>& t_transform)
{
    auto eigenFmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "", "", "\n", "", "");
    t_stream << t_transform.homogeneous().format(eigenFmt);
    return t_stream;
}

/** Conversion from amcl::math::Transform2D<S> to amcl::math::Transform2D<T>
 */
template <typename S, typename T>
struct cc::util::ToImpl<amcl::math::Transform2D<S>, amcl::math::Transform2D<T>,
                        typename std::enable_if<std::is_convertible<S, T>::value>::type>
{
    static inline void to(const amcl::math::Transform2D<S>& src, amcl::math::Transform2D<T>& tgt)
    {
        Eigen::Matrix<T, 2, 1> tran = src.translation().template cast<T>();
        Eigen::Rotation2D<T> rot = src.rotation().template cast<T>();

        tgt = amcl::math::Transform2D<T>{tran, rot};
    }
};
