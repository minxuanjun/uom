#pragma once

#include "uom/utils/common_types.hpp"
#include <ceres/ceres.h>


struct PointToEdgeFactorAutoDiff
{
    PointToEdgeFactorAutoDiff(Vector3d curr_point, Vector6d line_coeffs)
        : curr_point_(std::move(curr_point)), line_coeffs_(std::move(line_coeffs))
    {
        // make an unit-vector
        line_coeffs_.head<3>().normalize();
    }

    template <typename T>
    bool operator ()(const T* p, T* r) const
    {
        Eigen::Map<Sophus::SE3<T> const> const pose(p);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(r);

        Eigen::Matrix<T, 3, 1> p_w = pose * curr_point_.cast<T>();

        Eigen::Matrix<T, 3, 1> line_dir = line_coeffs_.head<3>().cast<T>();
        Eigen::Matrix<T, 3, 1> line_pt = line_coeffs_.tail<3>().cast<T>();

        residuals = line_dir.cross(line_pt - p_w);

        return true;
    }

    static ceres::CostFunction* create(const Vector3d& curr_point,
                                       const Vector6d& line_coeffs)
    {
        return (new ceres::AutoDiffCostFunction<PointToEdgeFactorAutoDiff, 3, 7>(
            new PointToEdgeFactorAutoDiff(curr_point, line_coeffs)));
    }

    // point in lidar frame
    Vector3d curr_point_;

    // v^{T} p + o = 0
    Vector6d line_coeffs_;
};