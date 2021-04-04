#include "uom/laser/laser_odometry.hpp"
#include "uom/optimize/laser_factor.hpp"
#include "uom/imu/imu_tools.hpp"

#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>

LaserOdometry::LaserOdometry(
    const LaserOdometryParams& odometry_params,
    const LaserParams& laser_params,
    const ImuParams& imu_params,
    const StaticInitializerParams& static_initializer_params) :
    imu_params_(imu_params), laser_params_(laser_params), odometry_params_(odometry_params),
    inertial_initializer_(static_initializer_params)
{
    kd_tree_surf_last_.reset(new pcl::KdTreeFLANN<PointType>());

    surf_last_.reset(new PointCloud);
    surf_last_ds_.reset(new PointCloud);

    surf_from_map_.reset(new PointCloud);
    surf_from_map_ds_.reset(new PointCloud);

    // VoxelGrid filter
    down_size_filter_surf_.setLeafSize(Vector4f::Constant(odometry_params.ds_surf_size));
    down_size_filter_surf_map_.setLeafSize(Vector4f::Constant(odometry_params.ds_surf_map_size));
}


void LaserOdometry::process(const PointCloud::Ptr& cloud_full, const PointCloud::Ptr& cloud_corn,
                            const PointCloud::Ptr& cloud_surf, const ImuDataVector& imu_data_vector)
{
    switch (frontend_state_)
    {
        case FrontendState::BOOTSTRAP:
        {
            if (initialization_process(cloud_surf, imu_data_vector))
            {
                frontend_state_ = FrontendState::NOMINAL;
            }
            break;
        }
        case FrontendState::NOMINAL:
        {
            nominal_process(cloud_full, cloud_corn, cloud_surf, imu_data_vector);
            break;
        }
    }
}


bool LaserOdometry::initialization_process(const PointCloud::Ptr& cloud_surf, const ImuDataVector& imu_data_vector)
{
    inertial_initializer_.add_measurement(imu_data_vector);

    State init_state;
    if (inertial_initializer_.initialize(init_state))
    {
        LOG(WARNING) << std::setfill(' ')
                     << "Success to init inertial: \n\tba=" << init_state.ba.transpose() << "\n\tbg="
                     << init_state.bg.transpose() << "\n\tquat=" << init_state.q.coeffs().transpose();

        // Most recent state will be stored in abs_pose_
        // Matrix4d T_WL = init_state.transformation() * laser_params_.T_imu_laser;
        // q_curr_ = Quaterniond(T_WL.topLeftCorner<3, 3>());
        // t_curr_ = T_WL.block<3, 1>(0, 3);

        // update state array
        q_curr_.setIdentity();
        t_curr_.setZero();

        q_prev_ = q_curr_;
        t_prev_ = t_curr_;

        // update cloud feature array
        surf_frames_.clear();
        surf_frames_.emplace_back(cloud_surf);

        State temp {.p = t_curr_, .q = q_curr_};
        recent_surf_frames_.emplace_back(transform_cloud(cloud_surf, temp));

        // true indicates that inertial system was initialized successfully.
        return true;
    }

    // We need more data
    return false;
}


void LaserOdometry::build_local_map()
{
    surf_from_map_->clear();

    for (auto& frame : recent_surf_frames_)
    {
        *surf_from_map_ += *frame;
    }
    surf_from_map_->header = recent_surf_frames_.back()->header;
}


void LaserOdometry::downsample_cloud()
{
    down_size_filter_surf_map_.setInputCloud(surf_from_map_);
    down_size_filter_surf_map_.filter(*surf_from_map_ds_);

    surf_last_ds_->clear();

    down_size_filter_surf_.setInputCloud(surf_last_);
    down_size_filter_surf_.filter(*surf_last_ds_);

    surf_from_map_ds_->header = surf_from_map_->header;
    surf_last_ds_->header = surf_last_->header;
}


void LaserOdometry::update_transformation()
{
    // Make sure there is enough feature points in the sweep
    if (surf_from_map_ds_->points.size() < 10)
    {
        LOG(WARNING) << "Not enough feature points from the map";
        return;
    }

    kd_tree_surf_last_->setInputCloud(surf_from_map_ds_);

    int match_cnt = odometry_params_.scan_match_cnt;
    if (surf_frames_.size() < 2)
    {
        match_cnt = 8;
    }

    for (int iter_cnt = 0; iter_cnt < match_cnt; iter_cnt++)
    {
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization* eigen_quat_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem problem;
        problem.AddParameterBlock(q_curr_.coeffs().data(), 4, eigen_quat_parameterization);
        problem.AddParameterBlock(t_curr_.data(), 3);

        auto surf_res_cnt = find_corresponding();

        for (std::size_t i = 0; i < surf_res_cnt; ++i)
        {
            ceres::CostFunction* costFunction = PointToPlaneFactor::create(surf_current_pts_[i], surf_plane_coeff_[i]);
            problem.AddResidualBlock(costFunction, loss_function, q_curr_.coeffs().data(), t_curr_.data());
        }

        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::DENSE_QR;
        solverOptions.max_num_iterations = odometry_params_.max_num_iter;
        solverOptions.max_solver_time_in_seconds = 0.02;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;

        ceres::Solver::Summary summary;
        ceres::Solve(solverOptions, &problem, &summary);
    }

    // update relative transformation delta
    t_delta_ = q_prev_.inverse() * (t_curr_ - t_prev_);
    q_delta_ = q_prev_.inverse() * q_curr_;

    // Update cloud array
    PointCloud::Ptr surf_data_copy(new PointCloud);
    pcl::copyPointCloud(*surf_last_ds_, *surf_data_copy);
    surf_frames_.emplace_back(surf_data_copy);

    // Update local map window
    State temp {.p = t_curr_, .q = q_curr_};
    recent_surf_frames_.emplace_back(transform_cloud(surf_frames_.back(), temp));
    if (recent_surf_frames_.size() > odometry_params_.windows_size)
    {
        recent_surf_frames_.pop_front();
    }
}

void LaserOdometry::init_pose_guess(const ImuDataVector& imu_data_vec)
{
    // cache previous
    t_prev_ = t_curr_;
    q_prev_ = q_curr_;

    // Use IMU rotation part
    Matrix3d R_BL = laser_params_.T_imu_laser.topLeftCorner<3, 3>();
    Quaterniond q_b0_b1 = compute_rotation_from_imu_vector(imu_data_vec);
    q_delta_ = Quaterniond { R_BL.transpose() * q_b0_b1.matrix() * R_BL };

    // predict
    t_curr_ = q_curr_ * t_delta_ + t_curr_;
    q_curr_ = q_curr_ * q_delta_;
}


std::size_t LaserOdometry::find_corresponding()
{
    constexpr int search_n = 6;
    const std::size_t pts_size = surf_last_ds_->points.size();

    surf_plane_coeff_.clear();
    surf_current_pts_.clear();
    surf_plane_coeff_.reserve(pts_size);
    surf_current_pts_.reserve(pts_size);

    for (std::size_t i = 0; i < pts_size; ++i)
    {
        Vector3d curr_pt = surf_last_ds_->points[i].getVector3fMap().cast<double>();

        PointType point_sel;
        point_sel.getVector3fMap() = (q_curr_ * curr_pt + t_curr_).cast<float>();

        std::vector<int> point_search_idx;
        std::vector<float> point_search_dists;
        kd_tree_surf_last_->nearestKSearch(point_sel, search_n, point_search_idx, point_search_dists);

        // p \cdot n + d = 1  => p \cdot n' + 1 = 0 =>  P \cdot n' = -1
        Eigen::Matrix<double, search_n, 3> matA0;
        Eigen::Matrix<double, search_n, 1> matB0 = -Eigen::Matrix<double, search_n, 1>::Ones();

        // Not too much far away with searched points.
        if (point_search_dists.back() < 1.0)
        {
            for (int j = 0; j < search_n; ++j)
            {
                matA0.row(j) = surf_from_map_ds_->points[point_search_idx[j]].getVector3fMap().cast<double>();
            }

            // Get the norm of the plane using linear solver based on QR composition
            Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double coeff_d = 1 / norm.norm();
            norm.normalize(); // get the unit norm

            // Compute the centroid of the plane
             Vector3d center;
             center.x() = matA0.col(0).sum() / search_n;
             center.y() = matA0.col(1).sum() / search_n;
             center.z() = matA0.col(2).sum() / search_n;

            // Make sure that the plan is fit
            bool plane_valid = true;
            for (int j = 0; j < search_n; ++j)
            {
                auto p_j = surf_from_map_ds_->points[point_search_idx[j]].getVector3fMap().cast<double>();
                double pd = std::abs(norm.dot(p_j) + coeff_d);

                if (pd > 0.05)
                {
                    plane_valid = false;
                    break;
                }
            }

            // if one eigenvalue is significantly larger than the other two
            if (plane_valid)
            {
                double pd = norm.dot(curr_pt) + coeff_d;
                double weight = 1.0 - 0.9 * std::abs(pd) / std::sqrt(curr_pt.norm());

                if (weight > 0.4)
                {
                    surf_current_pts_.emplace_back(curr_pt);

                    Vector4d plane_coeffs;
                    plane_coeffs << weight * norm, weight * coeff_d;
                    surf_plane_coeff_.emplace_back(std::move(plane_coeffs));
                }
            }
        }
    }

    return surf_current_pts_.size();
}


void LaserOdometry::nominal_process(const PointCloud::Ptr& cloud_full,
                                    const PointCloud::Ptr& cloud_corn,
                                    const PointCloud::Ptr& cloud_surf,
                                    const ImuDataVector& imu_data_vector)
{
    surf_last_ = cloud_surf;

    // Construct local cloud map from recent cloud feature.
    build_local_map();

    // Downsample map and current cloud feature
    downsample_cloud();

    // Predict current motion by using a static motion model.
    init_pose_guess(imu_data_vector);

    // Do data-association, optimize current transformation and update state array and cloud array.
    update_transformation();

    // clear_cloud();
}


void LaserOdometry::get_odom(Quaterniond& q_w_i, Vector3d& t_w_i) const
{
    q_w_i = q_curr_.cast<double>();
    q_w_i.normalize();
    t_w_i = t_curr_.cast<double>();
}


LaserOdometry::PointCloud::Ptr LaserOdometry::get_local_map() const
{
    if (frontend_state_ == FrontendState::BOOTSTRAP)
    {
        return nullptr;
    }

    PointCloud::Ptr ret(new PointCloud);
    pcl::copyPointCloud(*surf_from_map_ds_, *ret);
    return ret;
}


LaserOdometry::PointCloud::Ptr LaserOdometry::transform_cloud(const PointCloud::Ptr& local_cloud, const State& x)
{
    DCHECK(local_cloud != nullptr);

    PointCloud::Ptr ret(new PointCloud);
    pcl::transformPointCloud(*local_cloud,                         // input
                             *ret,                              // output
                             x.transformation().cast<float>(),     // SE3 pose
                             true);                   // need all field
    ret->header.frame_id = "odom";
    return ret;
}


void LaserOdometry::clear_cloud()
{
    surf_last_->clear();
    surf_last_ds_->clear();
    surf_from_map_->clear();
    surf_from_map_ds_->clear();

    // for save memory
    if (surf_frames_.size() > 7)
    {
        surf_frames_[surf_frames_.size() - 8]->clear();
    }
}



