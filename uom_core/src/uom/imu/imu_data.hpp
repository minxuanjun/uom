#pragma once

#include "uom/common_types.hpp"


struct ImuData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Receive time
    double t = 0;

    /// Raw Measurement of acceleration
    Vector3d acc = Vector3d::Zero();

    /// Raw Measurement of gyro
    Vector3d gyr = Vector3d::Zero();
};