#pragma once

#include <Eigen/Eigen>

/// Useful conversion variable
constexpr static double deg2rad = M_PI / 180.0;
constexpr static double rad2deg = 180.0 / M_PI;


/// Compute the angle between two vector
template <typename T>
inline T vector_angle(const Eigen::Matrix<T, 3, 1>& vec_a, const Eigen::Matrix<T, 3, 1>& vec_b,
                      bool force_sharp_angle = true)
{
    T vec_a_norm = vec_a.norm(), vec_b_norm = vec_b.norm();
    if (vec_a_norm == 0 || vec_b_norm == 0)
    {
        // zero vector is parallel to any vector.
        return T(0);
    }
    else
    {
        if (force_sharp_angle)
        {
            /// Add `abs` to force in sharp angle
            return acos(abs(vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
        }
        else
        {
            return acos((vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
        }
    }
}


/// Compute eigenvalues and eigenvectors for a series of points
template <typename T, int DIM>
inline void solve_eigen(const std::vector<Eigen::Matrix<T, DIM, 1>>& pts,
                        Eigen::Matrix<T, 1, DIM>& eigen_value, Eigen::Matrix<T, DIM, DIM>& eigen_vector)
{

    using PointType = Eigen::Matrix<T, DIM, 1>;
    using MatType = Eigen::Matrix<T, DIM, DIM>;

    // compute center
    PointType center = PointType::Zero();
    for (auto& p: pts)
        center += p;
    center /= static_cast<double>(pts.size());

    // compute cov
    MatType cov_mat = MatType::Zero();
    for (size_t i = 0; i < pts.size(); ++i)
    {
        PointType zero_mean = pts[i] - center;
        cov_mat += zero_mean * zero_mean.transpose();
    }

    // compute eigenvalue and eigen vector
    Eigen::SelfAdjointEigenSolver<MatType> esolver(cov_mat);
    eigen_value = esolver.eigenvalues().real();
    eigen_vector = esolver.eigenvectors().real();
}


/// Get quaternion from rotation vector
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> delta_Q(const Eigen::MatrixBase<Derived>& theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}


/// Basic Interpolation
template <typename TimeType, typename InterpolateType>
struct LinearInterpolation
{
    void operator()(const TimeType& t1, const InterpolateType& x1,
                    const TimeType& t2, const InterpolateType& x2,
                    const TimeType& t_interpolated, InterpolateType& x_interpolated)
    {
        CHECK_LT(t1, t2);

        const double theta = (t_interpolated - t1) / static_cast<double>(t2 - t1);

        x_interpolated = t1 + theta * (t2 - t1);
    }
};