#include <glog/logging.h>

#include "uom/utils/timer.hpp"
#include "uom/imu/imu_tools.hpp"
#include "uom/utils/statistics.hpp"
#include "uom/laser/laser_preprocessor.hpp"


void CloudFeatureExtractor::process(
    const PointCloud& in_cloud, const std::vector<sensor_msgs::ImuConstPtr>* imu_data_vec,
    PointCloud& out_full, PointCloud& out_corn, PointCloud& out_surf)
{
    /// We will use timestamp information
    CHECK(in_cloud.header.stamp > 0);

    out_full.header = in_cloud.header;
    out_corn.header = in_cloud.header;
    out_surf.header = in_cloud.header;

    /// Process IMU If we have
    Quaterniond quat = Quaterniond::Identity();
    if (params_.to_undistort && imu_data_vec)
    {
        quat = process_imu(*imu_data_vec);
    }

    StatsCollector timing_undistortion_stats("undistortion [ms]");
    StatsCollector timing_compute_rejection_stats("compute_rejection [ms]");
    StatsCollector timing_compute_features_stats("compute_features [ms]");

    /// Use timestamp and quat to undistort points.
    timing_undistortion_stats.AddSample(
        Measure<std::chrono::milliseconds>::execution([&]
                                                      {
                                                          undistortion(in_cloud, quat);
                                                      }));

    /// Set PointInfo and find splitting scan. We use Original Cloud to split points, instead of undistort points.
    timing_compute_rejection_stats.AddSample(
        Measure<std::chrono::milliseconds>::execution([&]
                                                      {
                                                          compute_rejection(in_cloud);
                                                      }));

    /// Compute critical and get features
    timing_compute_features_stats.AddSample(
        Measure<std::chrono::milliseconds>::execution([&]
                                                      {
                                                          compute_features(out_corn, out_surf);
                                                      }));

    /// Construct full pointcloud After computing curvature
    out_full.clear();
    out_full.reserve(points_undistort_.size());
    for (auto& idx_vec : point_idx_per_scan_)
    {
        for (auto& idx : idx_vec)
        {
            out_full.emplace_back(points_undistort_[idx]);
        }
    }

    // LOG(INFO) << Statistics::Print();
}

void CloudFeatureExtractor::compute_features(PointCloud& pc_corners, PointCloud& pc_surface)
{
    pc_corners.clear();
    pc_surface.clear();
    pc_surface.reserve(points_undistort_.size());
    pc_corners.reserve(points_undistort_.size() / 2.0);

    /// Compute curvature for every scans
    for (auto& pts_idx_vec : point_idx_per_scan_)
    {
        if (pts_idx_vec.size() < params_.curvature_ssd_size * 2 + 1)
        {
            continue;
        }

        const std::size_t pts_size = pts_idx_vec.size();
        const std::size_t end_idx = pts_size - params_.curvature_ssd_size;

        for (std::size_t i = params_.curvature_ssd_size; i < end_idx; ++i)
        {
            std::size_t pt_idx = pts_idx_vec[i];
            auto& curr_pt_info = point_info_[pt_idx];
            auto& curr_pt = points_undistort_[pt_idx];


            /// Calculate the curvature and reflectivity diff to determine whether it is a corner point or a plane point

            Vector3f neighbor_accumulate = -2.f * params_.curvature_ssd_size * curr_pt.getVector3fMap();
            float reflectivity_accumulate = -2.f * params_.curvature_ssd_size * curr_pt_info.sigma;
            for (std::size_t j = 1; j <= params_.curvature_ssd_size; ++j)
            {
                const auto& temp1 = pts_idx_vec[i - j];
                const auto& temp2 = pts_idx_vec[i + j];

                neighbor_accumulate +=
                    points_undistort_[temp1].getVector3fMap() + points_undistort_[temp2].getVector3fMap();
                reflectivity_accumulate += point_info_[temp1].sigma + point_info_[temp2].sigma;
            }

            // c = s_x^{2} + s_y^{2} + s_z^{2}
            curr_pt_info.curvature = neighbor_accumulate.norm() / curr_pt.getVector3fMap().norm();
            curr_pt_info.reflectivity_curvature = std::abs(reflectivity_accumulate);

            // Compute plane incident angle
            std::size_t plane_a_idx = pts_idx_vec[i + params_.curvature_ssd_size];
            std::size_t plane_c_idx = pts_idx_vec[i - params_.curvature_ssd_size];

            Vector3f plane_a = points_undistort_[plane_a_idx].getVector3fMap();
            Vector3f plane_c = points_undistort_[plane_c_idx].getVector3fMap();

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
                for (std::size_t j = 1; j <= params_.curvature_ssd_size; ++j)
                {
                    auto& prev_point_info = point_info_[pts_idx_vec[i - j]];

                    if ((prev_point_info.ft_type & PointInfo::FT_CORNER))
                    {
                        if (curr_pt_info.curvature > prev_point_info.curvature)
                        {
                            prev_point_info.ft_type &= ~(PointInfo::FT_CORNER);
                        }
                        else
                        {
                            curr_pt_info.ft_type &= ~(PointInfo::FT_CORNER);
                        }
                    }
                }
            }
        }
    }

    for (auto& curr_pt_info: point_info_)
    {
        if (curr_pt_info.ft_type & PointInfo::FT_CORNER)
        {
            if (curr_pt_info.sqr_depth < std::pow(30, 2))
            {
                pc_corners.points.emplace_back(points_undistort_[curr_pt_info.index]);
            }
        }
        if (curr_pt_info.ft_type & PointInfo::FT_SURFACE)
        {
            if (curr_pt_info.sqr_depth < std::pow(1000, 2))
            {
                pc_surface.points.emplace_back(points_undistort_[curr_pt_info.index]);
            }
        }
    }
}


void CloudFeatureExtractor::compute_rejection(const PointCloud& origin_cloud)
{
    const std::size_t pts_size = points_undistort_.size();

    point_info_.clear();
    point_info_.resize(pts_size);

    point_idx_per_scan_.clear();
    point_idx_per_scan_.emplace_back(); // first scan.

    std::size_t prev_valid_idx = 0;

    /// Do projection and mark rejection
    for (std::size_t idx = 0; idx < pts_size; ++idx)
    {
        // We have to use original point cloud instead of undistorted cloud
        auto& curr_pt = origin_cloud[idx];

        PointInfo* curr_pt_info = &point_info_[idx];

        // update information
        curr_pt_info->index = idx;
        curr_pt_info->raw_intensity = curr_pt.intensity;

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
    const double dt = laser_params_.time_interval_pts * in_cloud.size();

    // split num_segment segment. All points in the same segment use same rotation parameter.
    const int num_segment = 200;
    const Quaterniond q0 = Quaterniond::Identity();
    Quaterniond q_table[num_segment + 1]{Quaterniond::Identity()};
    for (int i = 0; i <= num_segment; ++i)
    {
        q_table[i] = q0.slerp(double(i) / num_segment, quat);
    }

    for (std::size_t i = 0; i < in_cloud.size(); ++i)
    {
        auto& pt_info = point_info_[i];
        auto& pt = in_cloud[i];

        // compute undistorted point
        double ratio_i = laser_params_.time_interval_pts * double(i) / dt;
        if (ratio_i >= 1.0)
        {
            ratio_i = 1.0;
        }

        const Quaterniond& q_0_i = q_table[int(num_segment * ratio_i)];
        Vector3d pt_s = q_0_i * (pt.getVector3fMap().cast<double>());

        PointType pt_out = pt;
        pt_out.x = pt_s.x();
        pt_out.y = pt_s.y();
        pt_out.z = pt_s.z();
        points_undistort_.emplace_back(pt_out);
    }
}


Quaterniond CloudFeatureExtractor::process_imu(const std::vector<sensor_msgs::ImuConstPtr>& msgs)
{
    ImuDataVector imu_data_vec;
    imu_data_vec.reserve(msgs.size());
    for (auto& data : msgs)
    {
        ImuData imu_data;
        imu_data.t = data->header.stamp.toSec();
        imu_data.gyr << data->angular_velocity.x, data->angular_velocity.y, data->angular_velocity.z;
        imu_data.acc << data->linear_acceleration.x, data->linear_acceleration.y, data->linear_acceleration.z;
        imu_data_vec.emplace_back(std::move(imu_data));
    }

    Matrix3d R_BL = laser_params_.T_imu_laser.topLeftCorner<3, 3>();
    Quaterniond q_b0_b1 = compute_rotation_from_imu_vector(imu_data_vec);

    return Quaterniond { R_BL.transpose() * q_b0_b1.matrix() * R_BL };
}


cv::Mat CloudFeatureExtractor::get_debug_image()
{
    CHECK(!point_info_.empty());

    const int image_size = 600;
    const int half_image_size = image_size / 2;
    const double scale = image_size * 1.2;

    cv::Mat display_image(image_size, image_size, CV_8UC3);
    display_image = 0;

    for (auto& pt_info : point_info_)
    {
        cv::Point2i p_i(scale * pt_info.pt_normalized.x() + half_image_size,
                        scale * pt_info.pt_normalized.y() + half_image_size);
        cv::circle(display_image, p_i, 2, cv::Scalar(88, 88, 88), -1,
                   cv::LINE_AA);
    }

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

        for (std::size_t i = 0; i < vec.size(); ++i)
        {
            auto& pt_info = point_info_[vec[i]];
            cv::Point2i p_i(scale * pt_info.pt_normalized.x() + half_image_size,
                            scale * pt_info.pt_normalized.y() + half_image_size);
            cv::circle(display_image, p_i, 2, point_color * (0.5 * double(i) / double(vec.size()) + 0.5), -1,
                       cv::LINE_AA);
        }
    }

    return display_image;
}
