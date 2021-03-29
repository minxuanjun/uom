#pragma once

#include "uom/imu/imu_params.hpp"
#include "uom/vision/camera_params.hpp"

struct SimParams : ImuParams, CameraParams
{
    /// IMU update rate.
    int cam_rate = 30;

    /// camera_pixel_noise
    double pixel_n = 1.0;

    /// Simulation circle
    double duration = 20;

    SimParams()
    {
        // reasonable value
        T_imu_cam <<
            0,  0,  1,  0,
           -1,  0,  0,  0,
            0, -1,  0,  0,
            0,  0,  0,  1;
    }


    void parse_yaml(const std::string& file_path) override
    {
        ImuParams::parse_yaml(file_path);
        CameraParams::parse_yaml("cam0", file_path);

        YamlParser parser(file_path);
        parser.get_param("camera_pixel_noise", pixel_n);
        parser.get_param("cam_rate", cam_rate);
        parser.get_param("duration", duration);
    }

    void print() override
    {
        ImuParams::print();
        CameraParams::print();
    }
};