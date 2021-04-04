#pragma once

#include <cstring>
#include <ostream>
#include <opencv2/opencv.hpp>

#include "uom/utils/common_types.hpp"
#include "uom/utils/params_base.hpp"


enum class DistortionModel
{
    NONE,              // pinhole, plumb_bob
    RADTAN,            // radial-tangential
    EQUIDISTANT,       // equidistant
};


/// For print
std::ostream& operator <<(std::ostream& os, const DistortionModel& model);


// For YAML parse
DistortionModel string_to_distortion_model(const std::string& distortion_model);


/// Describing each camera parameters
struct CameraParams : ParamsBase
{
    std::string camera_name = "cam0";      // When we read param from `camchain` file of kalibr, we need a name
    std::string cam_rostopic = "/image_raw";   // Should be useful in reading bag file
    std::string camera_model = "pinhole";  // Always pinhole in current system

    /// The transformation which takes a vector frame cam_i to the IMU frame.
    Matrix4d T_imu_cam = Matrix4d::Identity();

    /// The transformation which takes a vector frame cam_i to the cam0
    Matrix4d T_cam0_cam = Matrix4d::Identity();

    /// OpenCv style parameters
    cv::Size2i resolution {640, 480};

    std::vector<double> distortion_coeffs {0, 0, 0, 0};

    std::vector<double> intrinsics {450, 450, 320, 240};

    /// Camera model: equidistant, radtan, e.t.
    DistortionModel distortion_model = DistortionModel::NONE;

    /// Ext parameters
    double timeshift_cam_imu = 0; // time shift from cam to imu: [s] (t_imu = t_cam + shift)


    /**
     * @brief CameraParams::parse_yaml with a camera_name is used to read param from a camchain file
     */
    void parse_yaml(const std::string& cam_name, const std::string& file_path)
    {
        CHECK(!camera_name.empty());

        CameraParams::camera_name = cam_name;
        CameraParams::parse_yaml(file_path);
    }


    void print() override
    {
        std::stringstream out;

        ParamsBase::print
        (
            // param name
            "CameraParams for " + camera_name,
            // output
            out,
            // input
            "Camera ID: ", camera_name,
            "Intrinsics: ", "",
            "- fx", intrinsics[0],
            "- fy", intrinsics[1],
            "- cu", intrinsics[2],
            "- cv", intrinsics[3],
            "Image Size: ", "",
            "- width", resolution.width,
            "- height", resolution.height,
            "Distortion: ", "",
            "- k1", distortion_coeffs[0],
            "- k2", distortion_coeffs[1],
            "- p1", distortion_coeffs[2],
            "- p2", distortion_coeffs[3],
            "Dist Model: ", distortion_model,
            "Time Shift: ", timeshift_cam_imu,
            "T_imu_cam: ", T_imu_cam,
            "T_cn_cnm: ", T_cam0_cam
        );

        LOG(INFO) << out.str();
    }


    cv::Matx33d get_camera_matrix() const
    {
        CHECK_EQ(intrinsics.size(), 4);
        const cv::Matx33d K(
            intrinsics[0], 0.0, intrinsics[2],
            0.0, intrinsics[1], intrinsics[3],
            0.0, 0.0, 1.0);
        return K;
    }


    cv::Vec4d get_distortion_matrix() const
    {
        CHECK_EQ(distortion_coeffs.size(), 4);

        cv::Vec4d distortion_coeffs_mat;
        for (int k = 0; k < int(distortion_coeffs.size()); k++)
        {
            distortion_coeffs_mat[k] = distortion_coeffs[k];
        }
        return  distortion_coeffs_mat;
    }


private:


    void parse_yaml(const std::string& file_path) override
    {
        YamlParser yaml_parser(file_path);

        /// Camera distortion_model
        std::string distortion_model_str;
        yaml_parser.get_nested_param(camera_name, "distortion_model", distortion_model_str);
        distortion_model = string_to_distortion_model(distortion_model_str);

        /// Camera distortion_coefficients
        yaml_parser.get_nested_param(camera_name, "distortion_coeffs", distortion_coeffs);
        CHECK_EQ(distortion_coeffs.size(), 4);

        /// Camera resolution
        std::vector<int> resolution_vec;
        yaml_parser.get_nested_param(camera_name, "resolution", resolution_vec);
        resolution = cv::Size2i(resolution_vec[0], resolution_vec[1]);

        /// Camera intrinsics
        yaml_parser.get_nested_param(camera_name, "intrinsics", intrinsics);
        CHECK_EQ(intrinsics.size(), 4);

        /// timeshift_cam_imu
        yaml_parser.get_nested_param(camera_name, "timeshift_cam_imu", timeshift_cam_imu);

        /// Camera rostopic
        yaml_parser.get_nested_param(camera_name, "rostopic", cam_rostopic);

        /// Camera pose wrt body
        // In kalibr file, they use `T_cam_imu`, which is body pose wrt camera
        yaml_parser.get_nested_param(camera_name, "T_cam_imu", T_imu_cam);
        T_imu_cam = T_imu_cam.inverse().eval();

        /// Camera pose wrt cam0
        if (camera_name != "cam0")
        {
            // such as : cam2 => cam_idx=2
            int cam_idx = std::stoi(camera_name.substr(3));
            std::string file_node_name = "T_cn_cnm" + std::to_string(cam_idx);

            yaml_parser.get_nested_param(camera_name, file_node_name, T_cam0_cam);
        }
    }

};
