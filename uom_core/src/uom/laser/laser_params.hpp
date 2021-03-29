#pragma once

#include <cstring>
#include <ostream>

#include "uom/common_types.hpp"
#include "uom/utils/params_base.hpp"


enum class LaserModel
{
    MID40,             // livox mid-40
};


/// For print
std::ostream& operator <<(std::ostream& os, const LaserModel& model);


// For YAML parse
LaserModel string_to_laser_model(const std::string& laser_model);


/// Describing each laser parameters
struct LaserParams : ParamsBase
{
    std::string laser_name = "laser";      // When we read param from `laserchain` file of kalibr, we need a name
    std::string rostopic = "/cloud_raw";   // Should be useful in reading bag file

    /// The transformation which takes a vector frame laser_i to the IMU frame.
    Matrix4d T_imu_laser = Matrix4d::Identity();

    /// Camera model: equidistant, radtan, e.t.
    LaserModel laser_model = LaserModel::MID40;

    /// Ext parameters
    double timeshift_laser_imu = 0; // time shift from laser to imu: [s] (t_imu = t_laser + shift)

    /**
     * @brief CameraParams::parse_yaml with a laser_name is used to read param from a camchain file
     */
    void parse_yaml(const std::string& laser_name, const std::string& file_path)
    {
        CHECK(!laser_name.empty());

        this->laser_name = laser_name;
        this->parse_yaml(file_path);
    }

    void print() override
    {
        std::stringstream out;

        ParamsBase::print
            (
                // param name
                "LaserParams for " + laser_name,
                // output
                out,
                // input
                "Laser ID: ", laser_name,
                "Laser Model: ", laser_model,
                "Time Shift: ", timeshift_laser_imu,
                "T_imu_laser: ", T_imu_laser
            );

        LOG(INFO) << out.str();
    }

private:


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser yaml_parser(file_path);

        /// Laser model
        std::string laser_model_str;
        yaml_parser.get_nested_param(laser_name, "laser_model", laser_model_str);
        laser_model = string_to_laser_model(laser_model_str);

        /// timeshift_cam_imu
        yaml_parser.get_nested_param(laser_name, "timeshift_laser_imu", timeshift_laser_imu);

        /// Camera rostopic
        yaml_parser.get_nested_param(laser_name, "rostopic", rostopic);

        /// Camera pose wrt body
        yaml_parser.get_nested_param(laser_name, "T_imu_laser", T_imu_laser);
    }

};