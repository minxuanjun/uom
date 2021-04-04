#pragma once

#include "uom/utils/common_types.hpp"
#include <ceres/ceres.h>

struct PointToEdgeFactor
{
    PointToEdgeFactor(Vector3d curr_point, Vector6d line_coeffs)
    : curr_point_(std::move(curr_point)), line_coeffs_(std::move(line_coeffs))
    {
        // make an unit-vector
        line_coeffs_.head<3>().normalize();
    }

    template <typename T>
    bool operator ()(const T* q, const T* t, T* residual) const
    {
        // pay attention to the internal memory layout
        Eigen::Quaternion<T> q_inc{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_inc{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
        Eigen::Matrix<T, 3, 1> point_w = q_inc * cp + t_inc;

        Eigen::Matrix<T, 3, 1> line_dir = line_coeffs_.head<3>().cast<T>();
        Eigen::Matrix<T, 3, 1> line_pt = line_coeffs_.tail<3>().cast<T>();

        residual[0] = (line_dir.cross(line_pt - point_w)).norm();

        return true;
    }

    static ceres::CostFunction* create(const Vector3d& curr_point,
                                       const Vector6d& line_coeffs)
    {
        return (new ceres::AutoDiffCostFunction<PointToEdgeFactor, 1, 4, 3>(
            new PointToEdgeFactor(curr_point, line_coeffs)));
    }

    // point in lidar frame
    Vector3d curr_point_;

    // v^{T} p + o = 0
    Vector6d line_coeffs_;
};


struct PointToPlaneFactor
{
    PointToPlaneFactor(Vector3d curr_point, Vector4d plane_coeffs)
        : curr_point_(std::move(curr_point)), plane_coeffs_(std::move(plane_coeffs)) {}

    template <typename T>
    bool operator ()(const T* q, const T* t, T* residual) const
    {
        // pay attention to the internal memory layout
        Eigen::Quaternion<T> q_inc{q[3], q[0], q[1], q[2]};

        Eigen::Matrix<T, 3, 1> t_inc{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
        Eigen::Matrix<T, 4, 1> point_w;
        point_w << q_inc * cp + t_inc, T(1);

        residual[0] = plane_coeffs_.cast<T>().dot(point_w);
        return true;
    }

    static ceres::CostFunction* create(const Vector3d& curr_point,
                                       const Vector4d& plane_coeffs)
    {
        return (new ceres::AutoDiffCostFunction<PointToPlaneFactor, 1, 4, 3>(
            new PointToPlaneFactor(curr_point, plane_coeffs)));
    }

    // point in lidar frame
    Vector3d curr_point_;

    // n^{T} p + d = 0
    Vector4d plane_coeffs_;
};