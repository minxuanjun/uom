#include <glog/logging.h>

#include "uom/imu/static_initialization.hpp"


void StaticInitializer::add_measurement(const ImuDataVector& imu_data_vec)
{

    // Append to buffer
    imu_buffer_.insert(imu_buffer_.end(), imu_data_vec.begin(), imu_data_vec.end());

    // Dropout readings older then three of our initialization windows (seconds)
    const double& oldest_threshold = imu_buffer_.back().t - 3 * options_.window_length;

    auto it = imu_buffer_.begin();
    while (it != imu_buffer_.end() && it->t < oldest_threshold)
    {
        it = imu_buffer_.erase(it);
    }

}


std::tuple<Vector3d, Vector3d, double> StaticInitializer::compute_avg_var(const ImuDataVector& imu_data_vec)
{
    CHECK(!imu_data_vec.empty());

    // Compute average value
    Vector3d acc_avg = Vector3d::Zero(), omg_avg = Vector3d::Zero();

    for (auto& data : imu_data_vec)
    {
        acc_avg += data.acc;
        omg_avg += data.gyr;
    }

    acc_avg /= static_cast<double>(imu_data_vec.size());
    omg_avg /= static_cast<double>(imu_data_vec.size());

    // Compute variance of acceleration
    double acc_var = 0;

    for (auto& data : imu_data_vec)
    {
        acc_var += (data.acc - acc_avg).dot(data.acc - acc_avg);
    }

    acc_var = std::sqrt(acc_var / static_cast<double>(imu_data_vec.size() - 1));

    return {acc_avg, omg_avg, acc_var};
}


bool StaticInitializer::initialize(State& out_state)
{
    // Clear
    out_state.set_identity();

    // Return if we don't have any measurements
    if (imu_buffer_.empty())
    {
        return false;
    }

    // Newest imu timestamp
    const double& newest_time = imu_buffer_.back().t;


    // Collect two window of IMU readings.
    ImuDataVector window_newest, window_second_new;

    for (auto& data: imu_buffer_)
    {
        if (data.t > newest_time - 1.0 * options_.window_length && data.t <= newest_time - 0.0 * options_.window_length)
        {
            window_newest.emplace_back(data);
        }
        if (data.t > newest_time - 2.0 * options_.window_length && data.t <= newest_time - 1.0 * options_.window_length)
        {
            window_second_new.emplace_back(data);
        }
    }

    // Return if both of these failed
    if (window_newest.empty() || window_second_new.empty())
    {
        return false;
    }

    // Compute average and variance
    auto&&[acc_avg_w1, omg_avg_w1, acc_var_w1] = compute_avg_var(window_newest);
    auto&&[acc_avg_w2, omg_avg_w2, acc_var_w2] = compute_avg_var(window_second_new);

    // If it is below the threshold and we want to wait till we detect a jerk
    if (acc_var_w1 < options_.excite_threshold && options_.wait_for_jerk)
    {
        LOG(WARNING) << "StaticInitializer::initialize(): No enough IMU excitation, below "
                        "threshold " << acc_var_w1 << " < " << options_.excite_threshold;
        return false;
    }

    // If it is above the threshold and we are not waiting for a jerk
    // Then we are not stationary (i.e. moving) so we should wait till we are
    if ((acc_var_w1 > options_.excite_threshold || acc_var_w2 > options_.excite_threshold) && !options_.wait_for_jerk)
    {
        LOG(WARNING) << "StaticInitializer::initialize(): No enough IMU excitation, below "
                        "threshold " << acc_var_w1 << " < " << options_.excite_threshold;

        LOG(WARNING) << "StaticInitializer::initialize(): to much IMU excitation, "
                        "above threshold " << acc_var_w1 << "," << acc_var_w2 << " > " << options_.excite_threshold;
        return false;
    }

    // Get z axis, which aligned with  -g (z_in_G = 0,0,1)
    Vector3d z_axis = acc_avg_w2.normalized();

    // Create an x_axis
    Vector3d e_1(1, 0, 0);

    // Make x_axis perpendicular to z
    Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    x_axis = x_axis / x_axis.norm();

    // Get y from the cross product of these two
    Vector3d y_axis = z_axis.cross(x_axis);

    // From these axes get rotation
    Matrix3d R_W_I;
    R_W_I.row(0) = x_axis;
    R_W_I.row(1) = y_axis;
    R_W_I.row(2) = z_axis;

    out_state.t = newest_time;
    out_state.q = Quaterniond(R_W_I);

    // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
    out_state.ba = acc_avg_w2 + R_W_I.inverse() * n_gravity_;
    out_state.bg = omg_avg_w2;

    return true;
}

