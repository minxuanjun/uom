#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include <sensor_msgs/Imu.h>

// For hashing pcl::PointXYZI
#include "uom/utils/hash_utils.hpp"
#include "uom/utils/common_types.hpp"
#include "uom/laser/laser_params.hpp"
#include "uom/laser/laser_preprocessor_params.hpp"


struct PointInfo
{
    /// \brief For point's rejection
    enum PointType
    {
        PT_NORMAL = 0x0000 << 0,              // Normal points
        PT_NEAR_ORIGIN = 0x0001 << 0,         // Points in short distance to [0, 0, 0], we will not compute normalized coor for it
        PT_REFLECTIVITY_LOW = 0x0001 << 1,    // Points with low reflectivity
        PT_NEAR_FRINGE = 0x0001 << 2,         // Points near the edge of circle
        PT_NAN = 0x0001 << 3,                 // Points with infinite value
    };

    /// \brief If and only if normal points can be labeled
    enum FeatureType
    {
        FT_UNLABELED = 0,
        FT_CORNER = 0x0001 << 0,
        FT_SURFACE = 0x0001 << 1,
        FT_HIGH_INTENSITY = 0x0001 << 2
    };

    /// Basic property for point's rejection or feature selection
    int pt_type = PT_NORMAL;

    int ft_type = FT_UNLABELED;

    /// Index of point in original cloud
    std::size_t index = 0;

    /// Value in intensity field of original input point
    float raw_intensity = 0.f;

    /// Yaw angle X-Y plane (degree)
    float polar_angle = 0.f;

    /// Squared projective distance in X-Y plane (m)
    float polar_sqr_dist = 0.f;

    /// Squared distance to sensor origin (m)
    float sqr_depth = 0.f;

    /// curvature of point
    float curvature = 0.0;

    float reflectivity_curvature = 0.0;

    /// Angle between the laser ray and the local plane around the measured point. (degree)
    float incident_angle = 0.0;

    /// eq.(4) ?
    float sigma = 0.f;

    /// Normalized coordination in X=1 plane
    Vector2f pt_normalized;

    /// Move direction, Far away from origin (1) or move to origin (-1)
    int move_direction = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class CloudFeatureExtractor
{
public:

    using PointType = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<PointType>;


    CloudFeatureExtractor(const CloudFeatureExtractorParams& params, const LaserParams& laser_params)
        : params_(params), laser_params_(laser_params){}


    void process(const PointCloud& in_cloud,
                 const std::vector<sensor_msgs::ImuConstPtr>* imu_data_vec,
                 PointCloud& out_full, PointCloud& out_corn, PointCloud& out_surf);


    cv::Mat get_debug_image();


protected:

    /// Use q_imu_ and timestamp in every point to do undistortion
    void undistortion(const PointCloud& in_cloud, Quaterniond quat);


    Quaterniond process_imu(const std::vector<sensor_msgs::ImuConstPtr>& imu_data_vec);


    void compute_rejection(const PointCloud& origin_cloud);


    void compute_features(PointCloud& pc_corners, PointCloud& pc_surface);


    /// \brief Update properties of a point and its neighborhood.
    inline void update_mask_of_point(
        PointInfo* point_info, const PointInfo::PointType& type, std::size_t neighbor_radius = 0)
    {
        DCHECK(neighbor_radius > 0);
        DCHECK(neighbor_radius <= point_info_.size() / 2);

        point_info->pt_type |= type;
        for (std::size_t i = 1; i <= neighbor_radius; ++i)
        {
            if (point_info->index - i < point_info_.size()) point_info_[point_info->index - i].pt_type |= type;
            if (point_info->index - i < point_info_.size()) point_info_[point_info->index - i].pt_type |= type;
        }
    }


    /// All settings
    CloudFeatureExtractorParams params_;

    /// For calibration params
    LaserParams laser_params_;

    /// Same size as input cloud. Contains all temporal result of every point
    std::vector<PointInfo> point_info_;

    /// Only inliers
    std::vector<std::vector<std::size_t>> point_idx_per_scan_;

    /// Undistort pointcloud
    std::vector<PointType> points_undistort_;

};