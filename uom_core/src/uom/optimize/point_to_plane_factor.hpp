#pragma once

#include "uom/utils/common_types.hpp"
#include <ceres/ceres.h>


struct PointToPlaneFactorAutodiff
{
    PointToPlaneFactorAutodiff(Vector3d curr_point, Vector4d plane_coeffs) : curr_point_(std::move(curr_point))
    {
        W_ = plane_coeffs.head<3>() * plane_coeffs.head<3>().transpose();
        dw_ = plane_coeffs[3] * plane_coeffs.head<3>();
    }

    template <typename T>
    bool operator ()(const T* p, T* r) const
    {
        Eigen::Map<Sophus::SE3<T> const> const pose(p);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(r);

        Eigen::Matrix<T, 3, 1> p_w = pose * curr_point_.cast<T>();

        residuals = W_.cast<T>() * p_w + dw_.cast<T>();
        return true;
    }

    static ceres::CostFunction* create(const Vector3d& curr_point,
                                       const Vector4d& plane_coeffs)
    {
        return (new ceres::AutoDiffCostFunction<PointToPlaneFactorAutodiff, 3, 7>(
            new PointToPlaneFactorAutodiff(curr_point, plane_coeffs)));
    }

    // point in lidar frame
    Vector3d curr_point_;

    Matrix3d W_;

    Vector3d dw_;
};