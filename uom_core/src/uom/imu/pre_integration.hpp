#pragma once

#include "uom/imu/imu_params.hpp"
#include "uom/imu/state.hpp"


struct PreIntegration
{

    PreIntegration() = delete;

    PreIntegration(const Vector3d& acc0, const Vector3d& gyr0, ImuParams params);


    void push_back(double dt, const Vector3d& acc, const Vector3d& gyr);


    void re_propagate(const Vector3d& linearized_ba, const Vector3d& linearized_bg);


    void propagate(double dt, const Vector3d& acc1, const Vector3d& gyr1);


    Matrix<double, 15, 1> evaluate(const State& x_i, const State& x_j);


    void mid_point_integration(double dt,
                               const Vector3d& acc0, const Vector3d& gyr0,
                               const Vector3d& acc1, const Vector3d& gyr1,
                               const Vector3d& delta_p, const Quaterniond& delta_q,
                               const Vector3d& delta_v, const Vector3d& linearized_ba,
                               const Vector3d& linearized_bg, Vector3d& result_delta_p,
                               Quaterniond& result_delta_q, Vector3d& result_delta_v,
                               Vector3d& result_linearized_ba, Vector3d& result_linearized_bg,
                               bool update_jacobian);

    /// Noise settings
    ImuParams params_;

    /// Buffers
    std::vector<double> dt_buf_;
    std::vector<Vector3d> acc_buf_;
    std::vector<Vector3d> gyr_buf_;

    /// Delta state
    double sum_dt_;

    Vector3d delta_p_;
    Vector3d delta_v_;
    Quaterniond delta_q_;

    /// Variant readings
    double dt_;               // latest reading timestamp
    Vector3d acc0_, gyr0_;    // previous or current reading
    Vector3d acc1_, gyr1_;    // current reading

    const Vector3d linearized_acc_, linearized_gyr_;

    Matrix<double, 15, 15> jacobian_, covariance_;
    Matrix<double, 18, 18> noise_;

};


