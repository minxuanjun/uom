#pragma once

#include "uom/utils/common_types.hpp"
#include <ceres/ceres.h>


struct LidarPlaneNormIncreFactor
{
    LidarPlaneNormIncreFactor(
        Vector3d curr_point, Vector3d plane_unit_norm, double negative_OA_dot_norm)
        : curr_point_(curr_point), plane_unit_norm_(plane_unit_norm), negative_OA_dot_norm_(negative_OA_dot_norm) {}

    template <typename T>
    bool operator ()(const T* q, const T* t, T* residual) const
    {
        Eigen::Quaternion<T> q_inc{q[0], q[1], q[2], q[3]};
        Eigen::Matrix<T, 3, 1> t_inc{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp = curr_point_.cast<T>();
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_inc * cp + t_inc;

        Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm_.x()), T(plane_unit_norm_.y()), T(plane_unit_norm_.z()));
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm_);
        return true;
    }

    static ceres::CostFunction* create(const Vector3d curr_point,
                                       const Eigen::Vector4d plane_coeffs)
    {
        return (new ceres::AutoDiffCostFunction<LidarPlaneNormIncreFactor, 1, 4, 3>(
            new LidarPlaneNormIncreFactor(curr_point, plane_coeffs.head<3>(), plane_coeffs[3])));
    }

    Vector3d curr_point_;

    Vector3d plane_unit_norm_;

    double negative_OA_dot_norm_;
};