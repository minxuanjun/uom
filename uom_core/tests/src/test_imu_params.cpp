#include <uom/imu/imu_params.hpp>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

TEST(ImuParams, ReadParamters)
{
    ImuParams params;
    params.parse_yaml(FLAGS_test_data_path + "/imu.yaml");

    EXPECT_DOUBLE_EQ(params.acc_n, 0.0015999999595806003);
    EXPECT_DOUBLE_EQ(params.acc_w, 0.0002508999896235764);
    EXPECT_DOUBLE_EQ(params.gyr_n, 0.007000000216066837);
    EXPECT_DOUBLE_EQ(params.gyr_w, 0.0019474000437185168);
    EXPECT_DOUBLE_EQ(params.rate, 400.0);

    ASSERT_TRUE(params.linearized_ba.isApprox(Vector3d{0.01, 0.02, 0.03}));
    ASSERT_TRUE(params.linearized_bg.isApprox(Vector3d{0.03, 0.02, 0.01}));
    ASSERT_TRUE(params.n_gravity.isApprox(Vector3d{0, 0, -9.81}));

    EXPECT_DOUBLE_EQ(params.integration_cov, 0.0001);

    params.print();
}