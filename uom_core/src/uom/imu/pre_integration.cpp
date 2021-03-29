#include "uom/imu/pre_integration.hpp"
#include "uom/utils/math_utils.hpp"

PreIntegration::PreIntegration(const Vector3d& acc0, const Vector3d& gyr0, ImuParams params)
    : acc0_{acc0},
      gyr0_{gyr0},
      linearized_acc_{acc0},
      linearized_gyr_{gyr0},
      jacobian_{Matrix<double, 15, 15>::Identity()},
      sum_dt_{0.0},
      delta_p_{Vector3d::Zero()},
      delta_q_{Quaterniond::Identity()},
      delta_v_{Vector3d::Zero()},
      params_{params}
{
    // TODO: add prior cov
    // covariance_ = params_.integration_cov * Matrix<double, 15, 15>::Identity();
    covariance_.setZero();

    noise_.setZero();
    noise_.block<3, 3>(0, 0) = (params_.acc_n * params_.acc_n) * Matrix3d::Identity();
    noise_.block<3, 3>(3, 3) = (params_.gyr_n * params_.gyr_n) * Matrix3d::Identity();
    noise_.block<3, 3>(6, 6) = (params_.acc_n * params_.acc_n) * Matrix3d::Identity();
    noise_.block<3, 3>(9, 9) = (params_.gyr_n * params_.gyr_n) * Matrix3d::Identity();
    noise_.block<3, 3>(12, 12) = (params_.acc_w * params_.acc_w) * Matrix3d::Identity();
    noise_.block<3, 3>(15, 15) = (params_.gyr_w * params_.gyr_w) * Matrix3d::Identity();
}


void PreIntegration::push_back(double dt, const Vector3d& acc, const Vector3d& gyr)
{
    dt_buf_.push_back(dt);
    acc_buf_.push_back(acc);
    gyr_buf_.push_back(gyr);
    propagate(dt, acc, gyr);
}


void PreIntegration::re_propagate(const Vector3d& linearized_ba, const Vector3d& linearized_bg)
{
    // Reset delta variables
    sum_dt_ = 0.0;
    acc0_ = linearized_acc_;
    gyr0_ = linearized_gyr_;
    delta_p_.setZero();
    delta_v_.setZero();
    delta_q_.setIdentity();

    // Reset linearized point
    params_.linearized_ba = linearized_ba;
    params_.linearized_bg = linearized_bg;

    // Reset probability info
    jacobian_.setIdentity();
    covariance_.setZero();

    // Loop through all readings
    for (size_t i = 0; i < dt_buf_.size(); ++i)
        propagate(dt_buf_[i], acc_buf_[i], gyr_buf_[i]);
}


void PreIntegration::propagate(double dt, const Vector3d& acc1, const Vector3d& gyr1)
{
    // Update current reading
    dt_ = dt;
    acc1_ = acc1;
    gyr1_ = gyr1;

    Vector3d result_delta_p;
    Vector3d result_delta_v;
    Quaterniond result_delta_q;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    // Do propagation
    mid_point_integration(dt, acc0_, gyr0_, acc1, gyr1, delta_p_, delta_q_, delta_v_,
                          params_.linearized_ba, params_.linearized_bg,
                          result_delta_p, result_delta_q, result_delta_v,
                          result_linearized_ba, result_linearized_bg, true);

    delta_p_ = result_delta_p;
    delta_q_ = result_delta_q;
    delta_v_ = result_delta_v;
    params_.linearized_ba = result_linearized_ba;
    params_.linearized_bg = result_linearized_bg;
    delta_q_.normalize();
    sum_dt_ += dt_;
    acc0_ = acc1_;
    gyr0_ = gyr1_;

}


void PreIntegration::mid_point_integration(
    double dt,
    const Vector3d& acc0, const Vector3d& gyr0,
    const Vector3d& acc1, const Vector3d& gyr1,
    const Vector3d& delta_p, const Quaterniond& delta_q,
    const Vector3d& delta_v, const Vector3d& linearized_ba,
    const Vector3d& linearized_bg, Vector3d& result_delta_p,
    Quaterniond& result_delta_q, Vector3d& result_delta_v,
    Vector3d& result_linearized_ba, Vector3d& result_linearized_bg,
    bool update_jacobian)
{
    Vector3d un_acc_0 = delta_q * (acc0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (gyr0 + gyr1) - linearized_bg;
    result_delta_q = delta_q * delta_Q(dt * un_gyr);
    Vector3d un_acc_1 = result_delta_q * (acc1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
    result_delta_v = delta_v + un_acc * dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    {
        Vector3d w_x = 0.5 * (gyr0 + gyr1) - linearized_bg;
        Vector3d a_0_x = acc0 - linearized_ba;
        Vector3d a_1_x = acc1 - linearized_ba;

        Matrix3d R_w_x = hat(w_x);
        Matrix3d R_a_0_x = hat(a_0_x);
        Matrix3d R_a_1_x = hat(a_1_x);

        Matrix3d delta_R = delta_q.toRotationMatrix();
        Matrix3d result_delta_R = result_delta_q.toRotationMatrix();

        MatrixXd F = MatrixXd::Zero(15, 15);
        F.block<3, 3>(0, 0) = Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_R * R_a_0_x * dt * dt +
                              -0.25 * result_delta_R * R_a_1_x *
                              (Matrix3d::Identity() - R_w_x * dt) * dt * dt;
        F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_R + result_delta_q.toRotationMatrix()) * dt * dt;
        F.block<3, 3>(0, 12) = -0.1667 * result_delta_R * R_a_1_x * dt * dt * -dt;
        F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * dt;
        F.block<3, 3>(3, 12) = -MatrixXd::Identity(3, 3) * dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_R * R_a_0_x * dt +
                              -0.5 * result_delta_R * R_a_1_x *
                              (Matrix3d::Identity() - R_w_x * dt) * dt;
        F.block<3, 3>(6, 6) = Matrix3d::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_R + result_delta_q.toRotationMatrix()) * dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_R * R_a_1_x * dt * -dt;
        F.block<3, 3>(9, 9) = Matrix3d::Identity();
        F.block<3, 3>(12, 12) = Matrix3d::Identity();

        MatrixXd V = MatrixXd::Zero(15, 18);
        V.block<3, 3>(0, 0) = 0.5 * delta_R * dt * dt;
        V.block<3, 3>(0, 3) = -0.25 * result_delta_R * R_a_1_x * dt * dt * 0.5 * dt;
        V.block<3, 3>(0, 6) = 0.5 * result_delta_R * dt * dt;
        V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(6, 0) = 0.5 * delta_R * dt;
        V.block<3, 3>(6, 3) = 0.5 * -result_delta_R * R_a_1_x * dt * 0.5 * dt;
        V.block<3, 3>(6, 6) = 0.5 * result_delta_R * dt;
        V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * dt;
        V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * dt;

        jacobian_ = F * jacobian_;
        covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
    }
}

Matrix<double, 15, 1> PreIntegration::evaluate(const State& x_i, const State& x_j)
{
    Matrix<double, 15, 1> residuals;

    residuals.setZero();

    Matrix3d dp_dba = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 12);

    Matrix3d dq_dbg = jacobian_.block<3, 3>(3, 12);

    Matrix3d dv_dba = jacobian_.block<3, 3>(6, 9);
    Matrix3d dv_dbg = jacobian_.block<3, 3>(6, 12);

    Vector3d dba = x_i.ba - params_.linearized_ba;
    Vector3d dbg = x_i.bg - params_.linearized_bg; // NOTE: optimized one minus the linearized one

    Quaterniond corrected_delta_q = delta_q_ * delta_Q(dq_dbg * dbg);
    Vector3d corrected_delta_v = delta_v_ + dv_dba * dba + dv_dbg * dbg;
    Vector3d corrected_delta_p = delta_p_ + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(0, 0) = x_i.q.inverse() * (-0.5 * params_.n_gravity * sum_dt_ * sum_dt_ + x_j.p - x_i.p - x_i.v * sum_dt_) - corrected_delta_p;
    residuals.block<3, 1>(3, 0) = 2.0 * (corrected_delta_q.inverse() * (x_i.q.inverse() * x_j.q)).normalized().vec();
    residuals.block<3, 1>(6, 0) = x_i.q.inverse() * (-params_.n_gravity * sum_dt_ + x_j.v - x_i.v) - corrected_delta_v;
    residuals.block<3, 1>(9, 0) = x_j.ba - x_i.ba;
    residuals.block<3, 1>(12, 0) = x_j.bg - x_i.bg;

    return residuals;
}
