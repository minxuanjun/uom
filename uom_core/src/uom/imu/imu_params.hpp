#pragma once

#include "uom/common_types.hpp"
#include "uom/utils/params_base.hpp"


struct ImuParams : ParamsBase
{
    /// IMU update rate.
    int rate = 400;

    /// accelerometer_noise_density
    double acc_n = 0.00059;

    /// gyroscope_noise_density
    double gyr_n = 0.000061;

    /// accelerometer_random_walk
    double acc_w = 0.000011;

    /// gyroscope_random_walk
    double gyr_w = 0.000001;

    Vector3d linearized_ba = Vector3d::Zero();
    Vector3d linearized_bg = Vector3d::Zero();

    /// Gravity constant and direction
    /// For a Z-down navigation frame, such as NED: gravity points along positive Z-axis,
    /// For a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
    Vector3d n_gravity{0, 0, -9.81};

    double integration_cov = 0.0001;

    void parse_yaml(const std::string& file_path) override
    {
        YamlParser yaml_parser(file_path);

        yaml_parser.get_param("update_rate", rate);
        yaml_parser.get_param("accelerometer_noise_density", acc_n);
        yaml_parser.get_param("gyroscope_noise_density", gyr_n);
        yaml_parser.get_param("accelerometer_random_walk", acc_w);
        yaml_parser.get_param("gyroscope_random_walk", gyr_w);

        std::array<double, 3> linearized_ba_array;
        std::array<double, 3> linearized_bg_array;
        std::array<double, 3> n_gravity_array;

        yaml_parser.get_param("linearized_ba", linearized_ba_array);
        yaml_parser.get_param("linearized_bg", linearized_bg_array);
        yaml_parser.get_param("n_gravity", n_gravity_array);

        linearized_ba = Vector3d(linearized_ba_array.data());
        linearized_bg = Vector3d(linearized_bg_array.data());
        n_gravity = Vector3d(n_gravity_array.data());

        yaml_parser.get_param("integration_cov", integration_cov);
    }

    void print() override
    {
        std::stringstream out;

        ParamsBase::print
            (
                // param name
                "ImuParams",
                // output
                out,
                // input
                "update_rate: ", rate,
                "accelerometer_noise_density: ", acc_n,
                "gyroscope_noise_density: ", gyr_n,
                "accelerometer_random_walk: ", acc_w,
                "gyroscope_random_walk: ", gyr_w,
                "linearized_ba: ", linearized_ba,
                "linearized_bg: ", linearized_bg,
                "n_gravity: ", n_gravity
            );

        LOG(INFO) << out.str();
    }

};