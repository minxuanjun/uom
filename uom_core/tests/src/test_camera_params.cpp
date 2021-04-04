#include <uom/vision/camera_params.hpp>
#include <gtest/gtest.h>


DECLARE_string(test_data_path);

TEST(CameraParams, ReadParamters)
{
    CameraParams params;
    params.parse_yaml("cam1", FLAGS_test_data_path + "/camchain.yaml");

    Matrix4d T_cam_imu, T_cn_cnm1;
    T_cam_imu << 0.004139434663975622, -0.9999908827771948, -0.0010485433465533545, -0.09738548062492787,
     0.006951684731647589, 0.0010773032686258177, -0.9999752564424078, 0.00018177550233263823,
     0.9999672690443696, 0.004132043096862936, 0.006956080778082796, -0.03128944582133639,
     0.0, 0.0, 0.0, 1.0;
    T_cn_cnm1 << 0.999978089234298, 6.632951085983079e-05, -0.006619414756407041, -0.12018627594459369,
     -6.886652717392447e-05, 0.9999999242681943, -0.00038304178123172, -0.00013794612269259888,
     0.006619388848132824, 0.0003834892445991951, 0.9999780180720352, -0.0005216227453188331,
     0.0, 0.0, 0.0, 1.0;

    EXPECT_EQ(-0.03925697996111361, params.distortion_coeffs[0]);
    EXPECT_EQ(0.001043885332478853, params.distortion_coeffs[1]);
    EXPECT_EQ(7.608873023877205e-05, params.distortion_coeffs[2]);
    EXPECT_EQ(8.50302406845555e-05, params.distortion_coeffs[3]);

    EXPECT_EQ(526.7543819613772, params.intrinsics[0]);
    EXPECT_EQ(527.7829570075781, params.intrinsics[1]);
    EXPECT_EQ(650.6156948131367, params.intrinsics[2]);
    EXPECT_EQ(356.5664323249376, params.intrinsics[3]);

    EXPECT_EQ(1280, params.resolution.width);
    EXPECT_EQ(720, params.resolution.height);
    EXPECT_EQ("/right_raw", params.cam_rostopic);
    EXPECT_EQ(0.0006595341199495037, params.timeshift_cam_imu);

    EXPECT_EQ(DistortionModel::RADTAN, params.distortion_model);

    EXPECT_TRUE(T_cam_imu.inverse().isApprox(params.T_imu_cam));
    EXPECT_TRUE(T_cn_cnm1.isApprox(params.T_cam0_cam));
}