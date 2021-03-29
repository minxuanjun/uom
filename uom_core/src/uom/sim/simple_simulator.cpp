#include <fstream>
#include <random>
#include <glog/logging.h>

#include "uom/sim/simple_simulator.hpp"


SimpleSimulator::SimpleSimulator(const std::string& sim_data_path)
{
    std::ifstream f(sim_data_path);

    if (!f.is_open())
    {
        LOG(FATAL) << "Cannot open " << sim_data_path;
        std::exit(-1);
    }

    while (!f.eof())
    {
        // Read values
        Vector4d pt0, pt1;
        f >> pt0.x() >> pt0.y() >> pt0.z();
        pt0.w() = 1;
        f >> pt1.x() >> pt1.y() >> pt1.z();
        pt1.w() = 1;

        points_.emplace(pt0);
        points_.emplace(pt1);
        lines_.emplace_back(pt0, pt1);
    }
}


SimpleSimulator::SimData SimpleSimulator::generate_sim_data(const SimParams& sim_params)
{
    params_ = sim_params;

    SimData data;

    create_imu_data(data);
    create_cam_data(data);

    return data;
}


void SimpleSimulator::add_imu_noise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    // Generate random noise for IMU
    Vector3d n_gyr(noise(generator), noise(generator), noise(generator));
    Vector3d n_acc(noise(generator), noise(generator), noise(generator));
    Vector3d n_ba(noise(generator), noise(generator), noise(generator));
    Vector3d n_bg(noise(generator), noise(generator), noise(generator));

    // \sqrt{ \delta t}
    const double sample_time_sqr = 1.0 / std::sqrt(1.0 * params_.rate);

    // Add gaussian noise to measurement
    Matrix3d acc_cov = params_.acc_n * Matrix3d::Identity();
    Matrix3d gyr_cov = params_.gyr_n * Matrix3d::Identity();

    // \sigma_d = \frac{\sigma_c}{\sqrt{\delta t}}
    data.acc_m += acc_cov * n_acc / sample_time_sqr + state_.ba;
    data.gyr_m += gyr_cov * n_gyr / sample_time_sqr + state_.bg;

    // Update bias, \sigma_bd = \sigma_bc \cdot \sqrt{\delta t}
    state_.bg += params_.gyr_w * sample_time_sqr * n_bg;
    state_.ba += params_.acc_w * sample_time_sqr * n_bg;

    data.ba = state_.ba;
    data.bg = state_.bg;
}


Matrix3d SimpleSimulator::euler_2_rot(const Vector3d& euler_angles)
{
    double roll = euler_angles(0);
    double pitch = euler_angles(1);
    double yaw = euler_angles(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Matrix3d RIb;
    RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
        sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
        -sp, cp * sr, cp * cr;
    return RIb;
}


Matrix3d SimpleSimulator::euler_rates_2_body_rates(const Vector3d& euler_angles)
{
    double roll = euler_angles(0);
    double pitch = euler_angles(1);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);

    Matrix3d R;
    R << 1, 0, -sp,
        0, cr, sr * cp,
        0, -sr, cr * cp;

    return R;
}


SimpleSimulator::MotionData SimpleSimulator::motion_model(double t)
{
    MotionData data;

    double ellipse_x = 20.0;
    double ellipse_y = 25.0;

    double z = 3.0;
    double K1 = 10;
    double K = M_PI / 10;

    // Translation T_WB
    Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);

    // First derivative of position in world frame
    Vector3d dp(-K * ellipse_x * sin(K * t),
                K * ellipse_y * cos(K * t),
                z * K1 * K * cos(K1 * K * t));

    // Second derivative of position
    double K2 = K * K;
    Vector3d ddp(-K2 * ellipse_x * cos(K * t),
                 -K2 * ellipse_y * sin(K * t),
                 -z * K1 * K1 * K2 * sin(K1 * K * t));

    // Rotation
    double k_roll = 0.2;
    double k_pitch = 0.3;
    Vector3d euler_angles(k_roll * cos(t),       // roll ~ [-0.2, 0.2]
                          k_pitch * sin(t),      // pitch ~ [-0.3, 0.3]
                          std::fmod(K * t + M_PI, 2 * M_PI));               // yaw ~ [0,2pi]

    // First derivative of euler angles
    Vector3d euler_angles_rates(-k_roll * sin(t), k_pitch * cos(t), K);

    Matrix3d Rwb = euler_2_rot(euler_angles);

    //  euler rates trans to body gyro
    Vector3d imu_gyro = euler_rates_2_body_rates(euler_angles) * euler_angles_rates;

    //  Rbw * Rwn * gn = gs
    Vector3d imu_acc = Rwb.transpose() * (ddp - params_.n_gravity);

    data.gyr_m = imu_gyro;
    data.acc_m = imu_acc;
    data.q = Eigen::Quaterniond(Rwb);
    data.p = position;
    data.v = dp;
    data.t = t;

    return data;
}


void SimpleSimulator::create_imu_data(SimData& out_sim_data)
{
    out_sim_data.imu_data_noised.clear();
    out_sim_data.imu_data_gt.clear();

    double t = 0;
    while (t < params_.duration)
    {
        MotionData data = motion_model(t);

        out_sim_data.imu_data_gt.push_back(data);

        add_imu_noise(data);
        out_sim_data.imu_data_noised.push_back(data);

        t += 1.0 / static_cast<double>(params_.rate);
    }
}


void SimpleSimulator::create_cam_data(SimData& out_sim_data)
{
    out_sim_data.cam_pose.clear();

    Matrix3d K;
    K << params_.intrinsics[0], 0, params_.intrinsics[2],
        0, params_.intrinsics[1], params_.intrinsics[3],
        0, 0, 1;

    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> noise(0.0, params_.pixel_n);

    double t = 0;
    while (t < params_.duration)
    {
        /// generate data from motion model
        MotionData data = motion_model(t + params_.timeshift_cam_imu);   // t_imu = t_cam + dt

        /// Convert motion to camera frame
        Matrix4d T_WB = Matrix4d::Identity();
        T_WB.block<3, 3>(0, 0) = data.q.toRotationMatrix();
        T_WB.block<3, 1>(0, 3) = data.p;

        Matrix4d T_WC = T_WB * params_.T_imu_cam;

        /// Store cam motion
        State cam_state {
            .t = t,
            .p = T_WC.block<3, 1>(0, 3),
            .q = Quaterniond(T_WC.topLeftCorner<3, 3>())
        };
        out_sim_data.cam_pose.emplace_back(std::move(cam_state));

        Matrix4d T_CW = T_WC.inverse();

        /// Generate point feature
        std::vector<Eigen::Vector2d> feature_points;
        std::vector<Eigen::Vector4d> cam_points;
        for (auto& pt_w : points_)
        {
            // Convert point to camera frame from world frame
            Eigen::Vector4d pt_c = T_WC.inverse() * pt_w;

            // Z-value has larger then 0, which means in front of camera
            if (pt_c(2) < 0)
            {
                continue;
            }

            // Convert point to camera normal coordination plane
            Vector3d pt_n = pt_c.head<3>() / pt_c.z();

            // Convert point to Intrinsic plane
            Vector3d pt_i = K * pt_n;

            if (pt_i.x() < 0 || pt_i.y() < 0 ||
                pt_i.x() >= params_.resolution.width || pt_i.y() >= params_.resolution.height)
            {
                continue;
            }

            cam_points.emplace_back(pt_w);
            feature_points.emplace_back( Vector2d::Constant(noise(generator)) + pt_i.head<2>());
        }

        out_sim_data.features_cam.emplace_back(std::move(feature_points));
        out_sim_data.points_cam.emplace_back(std::move(cam_points));


        /// Generate line feature
        std::vector<Vector4d> lines;
        for (auto& line : lines_)
        {
            // Convert point to camera frame from world frame
            Vector4d p1_c = T_CW * line.first;
            Vector4d p2_c = T_CW * line.second;

            // Z-value has larger then 0, which means in front of camera
            if (p1_c(2) < 0 || p2_c(2) < 0)
            {
                continue;
            }

            Vector3d p1_i = K * (p1_c.head<3>() / p1_c.z());
            Vector3d p2_i = K * (p2_c.head<3>() / p2_c.z());

            bool p1_not_in = p1_i.x() < 0 || p1_i.y() < 0 ||
                             p1_i.x() >= params_.resolution.width || p1_i.y() >= params_.resolution.height;

            bool p2_not_in = p2_i.x() < 0 || p2_i.y() < 0 ||
                             p2_i.x() >= params_.resolution.width || p2_i.y() >= params_.resolution.height;

            if (p1_not_in && p2_not_in)
            {
                continue;
            }

            Vector4d temp;
            temp << p1_i.head<2>(), p2_i.head<2>();
            lines.emplace_back(temp);
        }

        out_sim_data.lines_cam.emplace_back(std::move(lines));

        t += 1.0 / static_cast<double>(params_.cam_rate);
    }
}


