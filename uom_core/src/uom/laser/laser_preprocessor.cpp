#include <glog/logging.h>

#include "uom/utils/math_utils.hpp"
#include "uom/laser/laser_preprocessor.hpp"


void CloudFeatureExtractor::process(
    const PointCloud& in_cloud, const std::vector<sensor_msgs::ImuConstPtr>* imu_data_vec,
    PointCloud& out_full, PointCloud& out_corn, PointCloud& out_surf, PointCloud& undistort_points)
{
    /// We will use timestamp information
    CHECK(in_cloud.header.stamp > 0);
    current_time_cloud_ = in_cloud.header.stamp * 1e-9;

    out_full.header = in_cloud.header;
    out_corn.header = in_cloud.header;
    out_surf.header = in_cloud.header;
    undistort_points.header = in_cloud.header;

    /// Prepare contains, clear and resize
    point_info_.clear();
    point_info_.resize(in_cloud.size());


    /// Process IMU If we have
    Quaterniond quat = Quaterniond::Identity();
    if (imu_data_vec)
    {
        quat = process_imu(*imu_data_vec);
        // Matrix3d R_LI = laser_params_.T_imu_laser.inverse().topLeftCorner<3, 3>();
        // quat = R_LI * quat;
    }

    /// Set PointInfo and find splitting scan
    compute_rejection(in_cloud);

    /// Use timestamp and quat to undistort points.
    undistortion(in_cloud, quat);
    for (auto& pt : points_undistort_)
    {
        undistort_points.emplace_back(pt);
    }

    /// Compute critical and get features
    compute_features(in_cloud, out_corn, out_surf);

    /// Construct full pointcloud After computing curvature
    out_full.clear();
    out_full.reserve(in_cloud.size());
    for (auto& idx_vec : point_idx_per_scan_)
    {
        for (auto& idx : idx_vec)
        {
            out_full.emplace_back(in_cloud[idx]);
            out_full.back().intensity = point_info_[idx].curvature;
        }
    }
}

void CloudFeatureExtractor::compute_features(const PointCloud& in_cloud,
                                             PointCloud& pc_corners, PointCloud& pc_surface)
{
    pc_corners.clear();
    pc_surface.clear();
    pc_surface.reserve(in_cloud.size());
    pc_corners.reserve(in_cloud.size() / 2.0);

    /// Compute curvature for every scans
    for (auto& pts_idx_vec : point_idx_per_scan_)
    {
        if (pts_idx_vec.size() < params_.curvature_ssd_size * 2 + 1)
        {
            continue;
        }

        const size_t pts_size = pts_idx_vec.size();
        const size_t end_idx = pts_size - params_.curvature_ssd_size;

        for (size_t i = params_.curvature_ssd_size; i < end_idx; ++i)
        {
            size_t pt_idx = pts_idx_vec[i];
            auto& curr_pt_info = point_info_[pt_idx];
            auto& curr_pt = in_cloud[pt_idx];


            /// Calculate the curvature and reflectivity diff to determine whether it is a corner point or a plane point

            Vector3f neighbor_accumulate = -2.f * params_.curvature_ssd_size * curr_pt.getVector3fMap();
            float reflectivity_accumulate = -2.f * params_.curvature_ssd_size * curr_pt_info.sigma;
            for (size_t j = 1; j <= params_.curvature_ssd_size; ++j)
            {
                const auto& temp1 = pts_idx_vec[i - j];
                const auto& temp2 = pts_idx_vec[i + j];

                neighbor_accumulate += in_cloud[temp1].getVector3fMap() + in_cloud[temp2].getVector3fMap();
                reflectivity_accumulate += point_info_[temp1].sigma + point_info_[temp2].sigma;
            }

            // c = s_x^{2} + s_y^{2} + s_z^{2}
            curr_pt_info.curvature = neighbor_accumulate.norm() / curr_pt.getVector3fMap().norm();
            curr_pt_info.reflectivity_curvature = std::abs(reflectivity_accumulate);

            // Compute plane incident angle
            size_t plane_a_idx = pts_idx_vec[i + params_.curvature_ssd_size];
            size_t plane_c_idx = pts_idx_vec[i - params_.curvature_ssd_size];

            Vector3f plane_a = in_cloud[plane_a_idx].getVector3fMap();
            Vector3f plane_c = in_cloud[plane_c_idx].getVector3fMap();

            Vector3f vec_a(curr_pt.x, curr_pt.y, curr_pt.z);
            Vector3f vec_b = plane_a - plane_c;

            curr_pt_info.incident_angle = vector_angle(vec_a, vec_b, true) * rad2deg;

            //  Only need point in minimum_incident_angle <  curr_pt_info.incident_angle < 180 - minimum_incident_angle
            if (curr_pt_info.incident_angle > params_.min_incident_angle)
            {
                if (curr_pt_info.curvature < params_.threshold_surf_curvature)
                {
                    // If curvature is less then thr_surface_curvature, Mark as e_label_surface
                    curr_pt_info.ft_type |= PointInfo::FT_SURFACE;
                }

                else if (curr_pt_info.curvature > params_.threshold_corn_curvature ||
                         curr_pt_info.reflectivity_curvature > params_.threshold_corn_reflectivity)
                {
                    // If the depth difference is greater than or equal to 10% of its own depth
                    // it is considered a corner.
                    float sq2_diff = 0.1;

                    // Reject points hidden behind an objects.
                    if (std::abs(curr_pt_info.sqr_depth - point_info_[plane_a_idx].sqr_depth) <
                        sq2_diff * curr_pt_info.sqr_depth ||
                        std::abs(curr_pt_info.sqr_depth - point_info_[plane_c_idx].sqr_depth) <
                        sq2_diff * curr_pt_info.sqr_depth)
                    {
                        curr_pt_info.ft_type |= PointInfo::FT_CORNER;
                    }
                }
                // Other points are not ...
            }


            // Reject
            if (curr_pt_info.ft_type & PointInfo::FT_CORNER)
            {
                for (size_t j = 1; j <= params_.curvature_ssd_size; ++j)
                {
                    auto& prev_point_info = point_info_[pts_idx_vec[i - j]];
                    if (curr_pt_info.curvature > prev_point_info.curvature)
                    {
                        prev_point_info.ft_type &= ~(PointInfo::FT_CORNER);
                    }
                    else
                    {
                        curr_pt_info.ft_type &= ~(PointInfo::FT_CORNER);
                        prev_point_info.ft_type |= PointInfo::FT_CORNER;
                    }
                }
            }

            if (curr_pt_info.ft_type & PointInfo::FT_CORNER)
            {
                if (curr_pt_info.sqr_depth < std::pow(30, 2))
                {
                    pc_corners.points.emplace_back(in_cloud[curr_pt_info.index]);
                    pc_corners.points.back().intensity = curr_pt_info.curvature;
                }
            }
            if (curr_pt_info.ft_type & PointInfo::FT_SURFACE)
            {
                if (curr_pt_info.sqr_depth < std::pow(1000, 2))
                {
                    pc_surface.points.emplace_back(in_cloud[curr_pt_info.index]);
                    pc_surface.points.back().intensity = curr_pt_info.curvature;
                }
            }

        }
    }
}


void CloudFeatureExtractor::compute_rejection(const PointCloud& in_cloud)
{
    const size_t pts_size = in_cloud.size();

    point_idx_per_scan_.clear();
    point_idx_per_scan_.emplace_back(); // first scan.

    size_t prev_valid_idx = 0;

    /// Do projection and mark rejection
    for (size_t idx = 0; idx < pts_size; ++idx)
    {
        auto& curr_pt = in_cloud.points[idx];
        PointInfo* curr_pt_info = &point_info_[idx];

        // update information
        curr_pt_info->index = idx;
        curr_pt_info->raw_intensity = curr_pt.intensity;

        // Each packet contains a timestamp indicating the time of the first point in the packet.
        // The time interval of each point in the packet is equal.
        // ref: https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol#32-time-stamp-timestamp
        curr_pt_info->timestamp = current_time_cloud_ + params_.time_interval_pts * idx;

        // reject point with nan value
        if (!std::isfinite(curr_pt.x) || !std::isfinite(curr_pt.y) || !std::isfinite(curr_pt.z))
        {
            curr_pt_info->pt_type |= PointInfo::PT_NAN;
            continue;
        }

        // Mask nearby point
        curr_pt_info->sqr_depth = curr_pt.getVector3fMap().squaredNorm();
        if (curr_pt_info->sqr_depth < params_.min_allow_dist * params_.min_allow_dist || curr_pt.x == 0)
        {
            curr_pt_info->pt_type |= PointInfo::PT_NEAR_ORIGIN;
            continue;
        }

        // Project point onto X=0 plane, like in visual odometry
        curr_pt_info->pt_normalized << curr_pt.y / curr_pt.x, curr_pt.z / curr_pt.x;
        curr_pt_info->polar_sqr_dist = curr_pt_info->pt_normalized.squaredNorm();

        // Mask the point nears the edge of pattern
        if (curr_pt_info->polar_sqr_dist >= params_.max_near_fov * params_.max_near_fov)
        {
            update_mask_of_point(curr_pt_info, PointInfo::PT_NEAR_FRINGE, 2);
        }

        // Mask low reflectivity point
        // TODO: It is diff from eq.(3). Should it be curr_pt_info->raw_intensity / curr_pt_info->sqr_depth;
        curr_pt_info->sigma = curr_pt_info->raw_intensity / curr_pt_info->sqr_depth;
        if (curr_pt_info->sigma < params_.min_sigma)
        {
            curr_pt_info->pt_type |= PointInfo::PT_REFLECTIVITY_LOW;
        }

        if ((curr_pt_info->pt_type & PointInfo::PT_NEAR_FRINGE))
        {
            continue;
        }


        // Compute direction
        if (idx > 0)
        {
            auto& prev_pt_info = point_info_[prev_valid_idx];

            curr_pt_info->move_direction = (curr_pt_info->polar_sqr_dist > prev_pt_info.polar_sqr_dist) ? 1 : -1;

            if (point_info_[idx - 1].pt_type & PointInfo::PT_NEAR_FRINGE ||
                curr_pt_info->move_direction == -prev_pt_info.move_direction)
            {
                // add new scan
                if (point_idx_per_scan_.back().size() > 10)
                {
                    point_idx_per_scan_.emplace_back();
                }
            }
        }

        point_idx_per_scan_.back().emplace_back(idx);

        prev_valid_idx = idx;
    }
}


void CloudFeatureExtractor::undistortion(const PointCloud& in_cloud, Quaterniond quat)
{
    points_undistort_.clear();
    points_undistort_.reserve(in_cloud.size());

    if (quat.isApprox(Quaterniond::Identity()))
    {
        for (auto& pt : in_cloud)
        {
            // copy directly
            points_undistort_.emplace_back(pt);
        }
        return;
    }

    // total time interval
    const double dt = params_.time_interval_pts * in_cloud.size();

    const Quaterniond q0 = Quaterniond::Identity();

    for (size_t i = 0; i < in_cloud.size(); ++i)
    {
        auto& pt_info = point_info_[i];
        auto& pt = in_cloud[i];

        // compute undistorted point
        double ratio_i = (pt_info.timestamp - current_time_cloud_) / dt;
        if (ratio_i >= 1.0) ratio_i = 1.0;
        Quaterniond q_si = q0.slerp(ratio_i, quat);
        Vector3d pt_s = q_si * (pt.getVector3fMap().cast<double>());

        PointType pt_out = pt;
        pt_out.x = pt_s.x();
        pt_out.y = pt_s.y();
        pt_out.z = pt_s.z();
        points_undistort_.emplace_back(pt_out);
    }
}


Quaterniond CloudFeatureExtractor::process_imu(const std::vector<sensor_msgs::ImuConstPtr>& imu_data_vec)
{
    Quaterniond quat = Quaterniond::Identity();

    if (imu_data_vec.empty())
    {
        return quat;
    }

    double t_prev {imu_data_vec.front()->header.stamp.toSec()};
    Vector3d gyr_prev {imu_data_vec.front()->angular_velocity.x,
                       imu_data_vec.front()->angular_velocity.y,
                       imu_data_vec.front()->angular_velocity.z};

    for (auto& data : imu_data_vec)
    {
        const double t_curr = data->header.stamp.toSec();
        const double dt = t_curr - t_prev;

        Vector3d angular_velocity {data->angular_velocity.x,
                                   data->angular_velocity.y,
                                   data->angular_velocity.z};

        // Integration
        Eigen::Vector3d un_gyr = 0.5 * (gyr_prev + angular_velocity);
        quat *= delta_Q(un_gyr * dt);
        gyr_prev = angular_velocity;
    }

    return quat;
}


cv::Mat CloudFeatureExtractor::get_debug_image()
{
    CHECK(point_info_.size() > 0);

    const int image_size = 600;
    const double scale = image_size * 1.5;

    cv::Mat display_image(image_size, image_size, CV_8UC3);
    display_image = 0;


    for (auto& vec : point_idx_per_scan_)
    {
        Vector3d color = Vector3d::Random() * 255;
        if (color.x() < 20)
        {
            color.x() = 40;
        }
        if (color.y() < 20)
        {
            color.y() = 40;
        }
        if (color.z() < 20)
        {
            color.z() = 40;
        }
        cv::Scalar point_color(color.x(), color.y(), color.z());

        for (size_t i = 0; i < vec.size(); ++i)
        {
            auto& pt_info = point_info_[vec[i]];
            cv::Point2i p_i(scale * pt_info.pt_normalized.x() + image_size / 2,
                            scale * pt_info.pt_normalized.y() + image_size / 2);
            cv::circle(display_image, p_i, 1, point_color * (0.5 * double(i) / double(vec.size()) + 0.5), -1,
                       cv::LINE_AA);
        }
    }

    return display_image;
}
