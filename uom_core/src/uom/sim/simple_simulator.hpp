#pragma once

#include "uom/imu/state.hpp"
#include "uom/utils/hash_utils.hpp"
#include "uom/sim/sim_params.hpp"
#include "uom/utils/common_types.hpp"

#include <string>
#include <vector>
#include <unordered_set>

class SimpleSimulator
{
public:

    using Line = std::pair<Vector4d, Vector4d>;
    using Lines = std::vector<Line>;

    struct MotionData : State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Vector3d acc_m;
        Vector3d gyr_m;

    };


    struct SimData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<MotionData> imu_data_gt;
        std::vector<MotionData> imu_data_noised;

        // Camera pose
        std::vector<State>      cam_pose;

        // 3D points in current view with World Position
        std::vector<std::vector<Vector4d>> points_cam;

        // and the corresponding coordination in image plane
        std::vector<std::vector<Vector2d>> features_cam;

        // Line segment in intrisic image plane
        std::vector<std::vector<Vector4d>> lines_cam;
    };

    SimpleSimulator() = delete;

    /// Initial Simulator with data path
    SimpleSimulator(const std::string& sim_data_path);


    SimData generate_sim_data(const SimParams& sim_params);


    Lines get_all_lines() const
    {
        return lines_;
    }


    std::vector<Vector4d> get_all_points() const
    {
        return {points_.cbegin(), points_.cend()};
    }


protected:

    /**
     * @brief Add noise to input MotionData, update internal bias of acc and gyr
     * @param data
     */
    void add_imu_noise(MotionData& data);


    MotionData motion_model(double t);


    Matrix3d euler_rates_2_body_rates(const Vector3d& euler_angles);


    Matrix3d euler_2_rot(const Vector3d& euler_angles);


    void create_imu_data(SimData& out_sim_data);


    void create_cam_data(SimData& out_sim_data);


    SimParams params_;

    State state_;

    /// Contains all feature points (endpoint of line)
    std::unordered_set<Vector4d, hash_eigen<Vector4d>> points_;

    /// Contains all lines
    Lines lines_;

};


