#include "uom/laser/laser_odometry.hpp"
#include "uom/optimize/laser_factor.hpp"

#include <pcl/common/transforms.h>


LaserOdometry::LaserOdometry(
    const LaserOdometryParams& odometry_params,
    const LaserParams& laser_params,
    const ImuParams& imu_params,
    const StaticInitializerParams& static_initializer_params) :
    imu_params_(imu_params), laser_params_(laser_params), odometry_params_(odometry_params),
    inertial_initializer_(static_initializer_params)
{

    latest_frame_idx_ = 0;

    // state
    abs_pose_[0] = 1;
    for (int i = 1; i < 7; ++i)
    {
        abs_pose_[i] = 0;
    }
    quaternion_r.setIdentity();
    transition_r.setZero();

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
            if (initialization_process(imu_data_vector))
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


bool LaserOdometry::initialization_process(const ImuDataVector& imu_data_vector)
{
    inertial_initializer_.add_measurement(imu_data_vector);

    State init_state;
    if (inertial_initializer_.initialize(init_state))
    {
        LOG(WARNING) << "Success to init imu bias: ba=" << init_state.ba << ", bg="
                     << init_state.bg << ", quat=" << init_state.q.coeffs().transpose();

        state_cloud_frame_.clear();
        state_cloud_frame_.emplace_back(std::move(init_state));

        return true;
    }
    return false;
}


void LaserOdometry::build_local_map()
{
    if (!surf_from_map_)
    {
        surf_from_map_.reset(new PointCloud);
    }
    else
    {
        surf_from_map_->clear();
    }

    // Initialization
    if (state_cloud_frame_.size() <= 1)
    {
        *surf_from_map_ += *surf_last_;
        return;
    }


    if (recent_surf_frames_.size() < odometry_params_.windows_size)
    {
        CHECK_EQ(surf_frames_.size(), state_cloud_frame_.size());
        recent_surf_frames_.emplace_back(transform_cloud(surf_frames_.back(), state_cloud_frame_.back()));
    }
    else
    {
        if (latest_frame_idx_ != state_cloud_frame_.size() - 1)
        {
            recent_surf_frames_.pop_front();
            latest_frame_idx_ = state_cloud_frame_.size() - 1;
            recent_surf_frames_.push_back(transform_cloud(surf_frames_.back(), state_cloud_frame_.back()));
        }
    }


    for (auto& frame : recent_surf_frames_)
    {
        *surf_from_map_ += *frame;
    }
}


void LaserOdometry::downsample_cloud()
{
    down_size_filter_surf_map_.setInputCloud(surf_from_map_);
    down_size_filter_surf_map_.filter(*surf_from_map_ds_);

    surf_last_ds_->clear();

    down_size_filter_surf_.setInputCloud(surf_last_);
    down_size_filter_surf_.filter(*surf_last_ds_);
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

    double transformInc[7];
    std::memcpy(transformInc, abs_pose_, 7 * sizeof(double));

    int match_cnt;
    if (state_cloud_frame_.size() < 2)
    {
        match_cnt = 8;
    }
    else
    {
        match_cnt = odometry_params_.scan_match_cnt;
    }

    for (int iter_cnt = 0; iter_cnt < match_cnt; iter_cnt++)
    {
        ceres::LossFunction* lossFunction = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization* quatParameterization = new ceres::QuaternionParameterization();
        ceres::Problem problem;
        problem.AddParameterBlock(transformInc, 4, quatParameterization);
        problem.AddParameterBlock(transformInc + 4, 3);

        auto surf_res_cnt = find_corresponding();

        for (std::size_t i = 0; i < surf_res_cnt; ++i)
        {
            const auto& surf_curr_pt = surf_current_pts_[i];
            const auto& surf_coeffs = surf_plane_coeff_[i];

            ceres::CostFunction* costFunction = LidarPlaneNormIncreFactor::create(surf_curr_pt, surf_coeffs);
            problem.AddResidualBlock(costFunction, lossFunction, transformInc, transformInc + 4);
        }

        ceres::Solver::Options solverOptions;
        solverOptions.linear_solver_type = ceres::DENSE_QR;
        solverOptions.max_num_iterations = odometry_params_.max_num_iter;
        solverOptions.max_solver_time_in_seconds = 0.015;
        solverOptions.minimizer_progress_to_stdout = false;
        solverOptions.check_gradients = false;
        solverOptions.gradient_check_relative_precision = 1e-2;

        ceres::Solver::Summary summary;
        ceres::Solve(solverOptions, &problem, &summary);

        if (transformInc[0] < 0)
        {
            transformInc[0] = -transformInc[0];
            transformInc[1] = -transformInc[1];
            transformInc[2] = -transformInc[2];
            transformInc[3] = -transformInc[3];
        }

        std::memcpy(abs_pose_, transformInc, 7 * sizeof(double));
    }


    // double ratio_u = double(surfResCount) / double(surfLastDS->points.size());
    Vector3d trans_cur(abs_pose_[4], abs_pose_[5], abs_pose_[6]);
    Quaterniond quat_cur(abs_pose_);

    double dis = (trans_cur - trans_last_kf_).norm();
    double ang = 2 * acos((quat_last_kf_.inverse() * quat_cur).w());
    if (((dis > 0.2 || ang > 0.1) && (state_cloud_frame_.size() - kf_num > 1) ||
         (state_cloud_frame_.size() - kf_num > 2)) || state_cloud_frame_.size() <= 1)
    {
        kf = true;
        trans_last_kf_ = trans_cur;
        quat_last_kf_ = quat_cur;
    }
    else
    {
        kf = false;
    }

    // update state vector
    State curr_state {.t = surf_last_->header.stamp / 1.0e6, .p = trans_cur, .q=quat_cur};
    if (!state_cloud_frame_.empty())
    {
        curr_state.ba = state_cloud_frame_.back().ba;
        curr_state.bg = state_cloud_frame_.back().bg;
    }

    PointCloud::Ptr surf_data_copy(new PointCloud);
    pcl::copyPointCloud(*surf_last_ds_, *surf_data_copy);
    surf_frames_.emplace_back(surf_data_copy);
}


void LaserOdometry::update_relative()
{
    Quaterniond quaternion1;
    Vector3d transition1;
    if (state_cloud_frame_.empty())
    {
        quaternion1.setIdentity();
        transition1.setZero();
    }
    else
    {
        int max_idx = state_cloud_frame_.size();
        quaternion1 = state_cloud_frame_[max_idx - 2].q;
        transition1 = state_cloud_frame_[max_idx - 2].p;
    }

    Quaterniond quaternion2(abs_pose_);
    Vector3d transition2(abs_pose_[4], abs_pose_[5], abs_pose_[6]);

    quaternion_r = quaternion1.inverse() * quaternion2;
    transition_r = quaternion1.inverse() * (transition2 - transition1);
}


void LaserOdometry::init_pose_guess()
{
    // TODO: Use IMU

    // Use laser
    Quaterniond q0(abs_pose_);
    Vector3d t0(abs_pose_[4], abs_pose_[5], abs_pose_[6]);

    t0 = q0 * transition_r + t0;
    q0 = q0 * quaternion_r;

    abs_pose_[0] = q0.w();
    abs_pose_[1] = q0.x();
    abs_pose_[2] = q0.y();
    abs_pose_[3] = q0.z();
    abs_pose_[4] = t0.x();
    abs_pose_[5] = t0.y();
    abs_pose_[6] = t0.z();
}


std::size_t LaserOdometry::find_corresponding()
{
    constexpr int search_n = 5;
    const std::size_t pts_size = surf_last_ds_->points.size();

    surf_plane_coeff_.clear();
    surf_current_pts_.clear();
    surf_plane_coeff_.reserve(pts_size);
    surf_current_pts_.reserve(pts_size);

    Quaterniond quaternion(abs_pose_);
    Vector3d transition(abs_pose_[4], abs_pose_[5], abs_pose_[6]);

    for (std::size_t i = 0; i < pts_size; ++i)
    {
        Vector3d curr_pt = surf_last_ds_->points[i].getVector3fMap().cast<double>();

        PointType point_sel;
        point_sel.getVector3fMap() = quaternion * curr_pt + transition;

        std::vector<int> point_search_idx;
        std::vector<float> point_search_dists;
        kd_tree_surf_last_->nearestKSearch(point_sel, search_n, point_search_idx, point_search_dists);

        Eigen::Matrix<double, search_n, 3> matA0;
        Eigen::Matrix<double, search_n, 1> matB0 = -Eigen::Matrix<double, search_n, 1>::Ones();

        if (point_search_dists.back() < 1.0)
        {
            for (int j = 0; j < search_n; ++j)
            {
                matA0.row(j) = surf_from_map_ds_->points[point_search_idx[j]].getVector3fMap().cast<double>();
            }

            // Get the norm of the plane using linear solver based on QR composition
            Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double norm_inverse = 1 / norm.norm();
            norm.normalize(); // get the unit norm

            // Compute the centroid of the plane
            // Vector3d center;
            // center.x() = matA0.col(0).sum() / search_n;
            // center.y() = matA0.col(1).sum() / search_n;
            // center.z() = matA0.col(2).sum() / search_n;

            // Make sure that the plan is fit
            bool plane_valid = true;
            for (int j = 0; j < search_n; ++j)
            {
                auto p_j = surf_from_map_ds_->points[point_search_idx[j]].getVector3fMap().cast<double>();
                double pd = std::abs(norm.dot(p_j) + norm_inverse);

                if (pd > 0.06)
                {
                    plane_valid = false;
                    break;
                }
            }

            // if one eigenvalue is significantly larger than the other two
            if (plane_valid)
            {
                double pd = norm.dot(curr_pt) + norm_inverse;
                double weight = 1.0 - 0.9 * std::abs(pd) / std::sqrt(curr_pt.norm());

                if (weight > 0.4)
                {
                    surf_current_pts_.emplace_back(curr_pt);

                    Vector4d plane_coeffs;
                    plane_coeffs << weight * norm, weight * norm_inverse;
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

    build_local_map();
    downsample_cloud();
    init_pose_guess();
    update_transformation();
    update_relative();

    // clear_cloud();
}


void LaserOdometry::get_odom(Quaterniond& q_w_i, Vector3d& t_w_i) const
{
    q_w_i = Quaterniond(abs_pose_);
    q_w_i.normalize();
    t_w_i = Vector3d(abs_pose_[4], abs_pose_[5], abs_pose_[6]);
}


LaserOdometry::PointCloud::Ptr LaserOdometry::get_local_map() const
{
    PointCloud::Ptr ret(new PointCloud);
    pcl::copyPointCloud(*surf_last_ds_, *ret);
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
    return ret;
}


void LaserOdometry::clear_cloud()
{
    surf_last_->clear();
    surf_last_ds_->clear();
    surf_from_map_->clear();
    surf_from_map_ds_->clear();

    // for save memory
    if(surf_frames_.size() > 7)
        surf_frames_[surf_frames_.size() - 8]->clear();
}



