#pragma once

#include <ceres/ceres.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "uom/utils/common_types.hpp"
#include "uom/imu/imu_params.hpp"
#include "uom/imu/state.hpp"
#include "uom/imu/imu_data.hpp"
#include "uom/imu/pre_integration.hpp"
#include "uom/imu/static_initialization.hpp"
#include "uom/laser/laser_params.hpp"
#include "uom/laser/laser_odometry_params.hpp"
#include "uom/geometry/voxel_filter_larger.hpp"


class LaserOdometry
{
public:

    using PointType = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointType>;


    LaserOdometry(const LaserOdometryParams& odometry_params = LaserOdometryParams(),
                  const LaserParams& laser_params = LaserParams(),
                  const ImuParams& imu_params = ImuParams(),
                  const StaticInitializerParams& static_initializer_params = StaticInitializerParams());

    ~LaserOdometry() = default;


    void process(const PointCloud::Ptr& cloud_full,
                 const PointCloud::Ptr& cloud_corn,
                 const PointCloud::Ptr& cloud_surf,
                 const ImuDataVector& imu_data_vector);


    void get_odom(Quaterniond& q_w_i, Vector3d& t_w_i) const;


    PointCloud::Ptr get_local_map() const;


protected:


    bool initialization_process(const PointCloud::Ptr& cloud_edge,
                                const PointCloud::Ptr& cloud_surf,
                                const ImuDataVector& imu_data_vector);


    void nominal_process(const PointCloud::Ptr& cloud_full,
                         const PointCloud::Ptr& cloud_corn,
                         const PointCloud::Ptr& cloud_surf,
                         const ImuDataVector& imu_data_vector);


    void build_local_map();


    void downsample_cloud();


    void init_pose_guess(const ImuDataVector& imu_vector);


    void update_transformation();


    std::size_t find_surf_corresponding(ceres::Problem& problem);


    std::size_t find_edge_corresponding(ceres::Problem& problem);


    PointCloud::Ptr transform_cloud(const PointCloud::Ptr& cloud_L, const Matrix4d& T);


    void clear_cloud();


    enum class FrontendState
    {
        BOOTSTRAP = 0u,  // Initialize frontend
        NOMINAL   = 1u   // Run frontend
    };

    FrontendState frontend_state_ = FrontendState::BOOTSTRAP;

    /// Configuration for Laser sensor and its odometry
    ImuParams imu_params_;
    LaserParams laser_params_;
    LaserOdometryParams odometry_params_;

    /// Current state, under un-thread-safe
    SE3d state_curr_, state_prev_, state_delta_;

    /// IMU relative
    StaticInitializer inertial_initializer_;

    /// Cloud relative
    PointCloud::Ptr surf_last_;
    PointCloud::Ptr surf_last_ds_;

    PointCloud::Ptr surf_from_map_;
    PointCloud::Ptr surf_from_map_ds_;

    std::vector<PointCloud::Ptr> surf_frames_;       // all cached raw cloud feature
    std::deque<PointCloud::Ptr> recent_surf_frames_; // cloud feature in odom frame

    pcl::VoxelGridLarge<PointType> down_size_filter_surf_;
    pcl::VoxelGridLarge<PointType> down_size_filter_surf_map_;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_surf_last_;


    PointCloud::Ptr edge_last_;
    PointCloud::Ptr edge_last_ds_;

    PointCloud::Ptr edge_from_map_;
    PointCloud::Ptr edge_from_map_ds_;

    std::vector<PointCloud::Ptr> edge_frames_;       // all cached raw cloud feature
    std::deque<PointCloud::Ptr> recent_edge_frames_; // cloud feature in odom frame

    pcl::VoxelGridLarge<PointType> down_size_filter_edge_;
    pcl::VoxelGridLarge<PointType> down_size_filter_edge_map_;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_edge_last_;

};


