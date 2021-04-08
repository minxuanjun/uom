#include "uom/laser/laser_odometry.hpp"
#include "uom/imu/imu_tools.hpp"
#include "uom/utils/math_utils.hpp"

#include "uom/optimize/parameterization.hpp"
#include "uom/optimize/point_to_edge_factor.hpp"
#include "uom/optimize/point_to_plane_factor.hpp"

#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>


LaserOdometry::LaserOdometry(
    const LaserOdometryParams& odometry_params,
    const LaserParams& laser_params,
    const ImuParams& imu_params,
    const StaticInitializerParams& static_initializer_params) :
    imu_params_(imu_params), laser_params_(laser_params), odometry_params_(odometry_params),
    inertial_initializer_(static_initializer_params)
{
    kd_tree_surf_last_.reset(new pcl::KdTreeFLANN<PointType>());
    kd_tree_edge_last_.reset(new pcl::KdTreeFLANN<PointType>());

    surf_last_.reset(new PointCloud);
    surf_last_ds_.reset(new PointCloud);

    edge_last_.reset(new PointCloud);
    edge_last_ds_.reset(new PointCloud);

    surf_from_map_.reset(new PointCloud);
    surf_from_map_ds_.reset(new PointCloud);

    edge_from_map_.reset(new PointCloud);
    edge_from_map_ds_.reset(new PointCloud);

    // VoxelGrid filter
    down_size_filter_surf_.setLeafSize(Vector4f::Constant(odometry_params.ds_surf_size));
    down_size_filter_surf_map_.setLeafSize(Vector4f::Constant(odometry_params.ds_surf_map_size));

    down_size_filter_edge_.setLeafSize(Vector4f::Constant(odometry_params.ds_edge_size));
    down_size_filter_edge_map_.setLeafSize(Vector4f::Constant(odometry_params.ds_edge_map_size));
}


void LaserOdometry::process(const PointCloud::Ptr& cloud_full, const PointCloud::Ptr& cloud_edge,
                            const PointCloud::Ptr& cloud_surf, const ImuDataVector& imu_data_vector)
{
    switch (frontend_state_)
    {
        case FrontendState::BOOTSTRAP:
        {
            if (initialization_process(cloud_edge, cloud_surf, imu_data_vector))
            {
                frontend_state_ = FrontendState::NOMINAL;
            }
            break;
        }
        case FrontendState::NOMINAL:
        {
            nominal_process(cloud_full, cloud_edge, cloud_surf, imu_data_vector);
            break;
        }
    }
}


bool LaserOdometry::initialization_process(
    const PointCloud::Ptr& cloud_edge, const PointCloud::Ptr& cloud_surf, const ImuDataVector& imu_data_vector)
{
    inertial_initializer_.add_measurement(imu_data_vector);

    /// We assume that the vehicle is static until inertial device was initialized.
    surf_frames_.emplace_back(cloud_surf);
    edge_frames_.emplace_back(cloud_edge);

    State init_state;
    if (inertial_initializer_.initialize(init_state))
    {
        LOG(WARNING) << std::setfill(' ')
                     << "Success to init inertial: \n\tba=" << init_state.ba.transpose() << "\n\tbg="
                     << init_state.bg.transpose() << "\n\tquat=" << init_state.q.coeffs().transpose();

        // Most recent state will be stored in abs_pose_
        Matrix4d T_WL = init_state.transformation() * laser_params_.T_imu_laser;
        state_curr_.setQuaternion(Quaterniond(T_WL.topLeftCorner<3, 3>()).normalized());
        state_curr_.translation() = T_WL.block<3, 1>(0, 3);

        state_prev_ = state_curr_;

        // Transform recent_surf_frames_ and recent_edge_frames_ to correct frame
        const std::size_t buff_size = surf_frames_.size();
        for (std::size_t i = 0; i < buff_size; ++i)
        {
            recent_surf_frames_.emplace_back(transform_cloud(cloud_surf, state_curr_.matrix()));
            recent_edge_frames_.emplace_back(transform_cloud(cloud_edge, state_curr_.matrix()));
        }

        // true indicates that inertial system was initialized successfully.
        return true;
    }

    // We need more data
    return false;
}


void LaserOdometry::build_local_map()
{
    CHECK_EQ(recent_surf_frames_.size(), recent_edge_frames_.size());

    surf_from_map_->clear();
    edge_from_map_->clear();

    for (std::size_t i = 0; i < recent_surf_frames_.size(); ++i)
    {
        *surf_from_map_ += *(recent_surf_frames_[i]);
        *edge_from_map_ += *(recent_edge_frames_[i]);
    }

    surf_from_map_->header = recent_surf_frames_.back()->header;
    edge_from_map_->header = recent_edge_frames_.back()->header;
}


void LaserOdometry::downsample_cloud()
{
    down_size_filter_surf_map_.setInputCloud(surf_from_map_);
    down_size_filter_surf_map_.filter(*surf_from_map_ds_);
    down_size_filter_edge_map_.setInputCloud(edge_from_map_);
    down_size_filter_edge_map_.filter(*edge_from_map_ds_);

    surf_last_ds_->clear();
    edge_last_ds_->clear();

    down_size_filter_surf_.setInputCloud(surf_last_);
    down_size_filter_surf_.filter(*surf_last_ds_);
    down_size_filter_edge_.setInputCloud(edge_last_);
    down_size_filter_edge_.filter(*edge_last_ds_);

    surf_from_map_ds_->header = surf_from_map_->header;
    surf_last_ds_->header = surf_last_->header;
    edge_from_map_ds_->header = edge_from_map_->header;
    edge_last_ds_->header = edge_last_->header;
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
    kd_tree_edge_last_->setInputCloud(edge_from_map_ds_);

    int match_cnt = odometry_params_.scan_match_cnt;
    if (surf_frames_.size() < 2)
    {
        match_cnt = 8;
    }

    for (int iter_cnt = 0; iter_cnt < match_cnt; iter_cnt++)
    {
        ceres::Problem problem;
        problem.AddParameterBlock(state_curr_.data(), SE3d::num_parameters, new LieLocalParameterization<SE3d>);

        auto surf_res_cnt = find_surf_corresponding(problem);
        auto edge_res_cnt = find_edge_corresponding(problem);

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
    state_delta_ = state_prev_.inverse() * state_curr_;

    // Update cloud array
    PointCloud::Ptr surf_data_copy(new PointCloud);
    pcl::copyPointCloud(*surf_last_ds_, *surf_data_copy);
    surf_frames_.emplace_back(surf_data_copy);

    PointCloud::Ptr edge_data_copy(new PointCloud);
    pcl::copyPointCloud(*edge_last_ds_, *edge_data_copy);
    edge_frames_.emplace_back(edge_data_copy);

    // Update local map window
    recent_surf_frames_.emplace_back(transform_cloud(surf_frames_.back(), state_curr_.matrix()));
    recent_edge_frames_.emplace_back(transform_cloud(edge_frames_.back(), state_curr_.matrix()));
    if (recent_surf_frames_.size() > odometry_params_.windows_size)
    {
        recent_surf_frames_.pop_front();
        recent_edge_frames_.pop_front();
    }
}

void LaserOdometry::init_pose_guess(const ImuDataVector& imu_data_vec)
{
    // Cache previous
    state_prev_ = state_curr_;

    // Use IMU rotation part
    Matrix3d R_BL = laser_params_.T_imu_laser.topLeftCorner<3, 3>();
    Quaterniond q_b0_b1 = compute_rotation_from_imu_vector(imu_data_vec);
    state_delta_.setQuaternion(q_b0_b1);

    // Predict
    state_curr_ = state_prev_ * state_delta_;
}


std::size_t LaserOdometry::find_edge_corresponding(ceres::Problem& problem)
{
    constexpr int search_n = 6;
    const std::size_t pts_size = edge_last_ds_->points.size();

    std::size_t residual_cnt = 0;

    ceres::LossFunction* loss_function = new ceres::HuberLoss(4.0);

    for (std::size_t i = 0; i < pts_size; ++i)
    {
        Vector3d curr_pt = edge_last_ds_->points[i].getVector3fMap().cast<double>();
        Vector3d curr_pt_w = state_curr_ * curr_pt;

        PointType point_sel;
        point_sel.getVector3fMap() = curr_pt_w.cast<float>();

        std::vector<int> point_search_idx;
        std::vector<float> point_search_dists;
        kd_tree_edge_last_->nearestKSearch(point_sel, search_n, point_search_idx, point_search_dists);

        // Not too much far away with searched points.
        if (point_search_dists.back() < 1.0)
        {
            std::vector<Vector3d> near_corners;

            Eigen::Matrix<double, 3, 1> center;
            Eigen::Matrix<double, 1, 3> lambda;
            Eigen::Matrix<double, 3, 3> vectors;

            for (auto& idx : point_search_idx)
            {
                near_corners.emplace_back(edge_from_map_ds_->points[idx].getVector3fMap().cast<double>());
            }

            solve_eigen(near_corners, &lambda, &vectors, &center);

            // If this is a valid line
            if (lambda[2] > 4 * lambda[1])
            {
                ++residual_cnt;

                Vector6d line_coeffs;
                line_coeffs << vectors.col(2), center;

                ceres::CostFunction* cost_function = PointToEdgeFactorAutoDiff::create(curr_pt, line_coeffs);
                problem.AddResidualBlock(cost_function, loss_function, state_curr_.data());
            }
        }
    }

    return residual_cnt;
}


std::size_t LaserOdometry::find_surf_corresponding(ceres::Problem& problem)
{
    constexpr int search_n = 6;
    const std::size_t pts_size = surf_last_ds_->points.size();

    // Count how many residual items were added to problem
    std::size_t residual_cnt = 0;

    ceres::LossFunction* loss_function = new ceres::HuberLoss(2.0);

    for (std::size_t i = 0; i < pts_size; ++i)
    {
        Vector3d curr_pt = surf_last_ds_->points[i].getVector3fMap().cast<double>();
        Vector3d curr_pt_w = state_curr_ * curr_pt;

        PointType point_sel;
        point_sel.getVector3fMap() = curr_pt_w.cast<float>();

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

                // If we have kernal function, is it necessary?
                // double weight = 1.0 - 0.9 * std::abs(pd) / std::sqrt(curr_pt.norm());
                // if (weight > 0.4) {}

                ++residual_cnt;

                Vector4d plane_coeffs;
                plane_coeffs << norm, coeff_d;

                ceres::CostFunction* cost_function = PointToPlaneFactorAutodiff::create(curr_pt, plane_coeffs);
                problem.AddResidualBlock(cost_function, loss_function, state_curr_.data());
            }
        }
    }

    return residual_cnt;
}


void LaserOdometry::nominal_process(const PointCloud::Ptr& cloud_full,
                                    const PointCloud::Ptr& cloud_edge,
                                    const PointCloud::Ptr& cloud_surf,
                                    const ImuDataVector& imu_data_vector)
{
    surf_last_ = cloud_surf;
    edge_last_ = cloud_edge;

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
    q_w_i = state_curr_.unit_quaternion();
    t_w_i = state_curr_.translation();
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


LaserOdometry::PointCloud::Ptr LaserOdometry::transform_cloud(const PointCloud::Ptr& local_cloud, const Matrix4d& T)
{
    DCHECK(local_cloud != nullptr);

    PointCloud::Ptr ret(new PointCloud);
    pcl::transformPointCloud(*local_cloud,                         // input
                             *ret,                              // output
                             T.cast<float>(),     // SE3 pose
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

    edge_last_->clear();
    edge_last_ds_->clear();
    edge_from_map_->clear();
    edge_from_map_ds_->clear();

    // for save memory
    if (surf_frames_.size() > 7)
    {
        surf_frames_[surf_frames_.size() - 8]->clear();
    }

    if (edge_frames_.size() > 7)
    {
        edge_frames_[edge_frames_.size() - 8]->clear();
    }
}



