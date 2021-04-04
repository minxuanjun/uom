#pragma once

#include "uom/imu/imu_data.hpp"
#include "uom/utils/math_utils.hpp"
#include "uom/utils/common_types.hpp"


inline static Quaterniond compute_rotation_from_imu_vector(const ImuDataVector& imu_data_vec)
{
    Quaterniond quat = Quaterniond::Identity();

    if (imu_data_vec.empty())
    {
        return quat;
    }

    // Compute the mean angular velocity in the IMU frame.
    Vector3d mean_ang_vel {0, 0, 0};
    for (auto& data : imu_data_vec)
    {
        mean_ang_vel += data.gyr;
    }
    mean_ang_vel /= static_cast<double>(imu_data_vec.size());

    // Compute rotation by using mean angular
    double dt = imu_data_vec.back().t - imu_data_vec.front().t;
    quat = SO3d::exp(mean_ang_vel * dt).unit_quaternion();
    quat.normalize();

    return quat;
}