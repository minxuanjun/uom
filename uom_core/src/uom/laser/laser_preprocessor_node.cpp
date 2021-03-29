#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include "uom_msgs/cloud_feature.h"
#include "uom/laser/laser_preprocessor.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_preprocessor_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string params_path;
    nh_private.getParam("params_path", params_path);

    auto pub_debug_image = nh_private.advertise<sensor_msgs::Image>("debug_image", 10, true);
    auto pub_surf = nh_private.advertise<sensor_msgs::PointCloud2>("surf", 10, true);
    auto pub_corn = nh_private.advertise<sensor_msgs::PointCloud2>("corn", 10, true);
    auto pub_full = nh_private.advertise<sensor_msgs::PointCloud2>("full", 10, true);
    auto pub_feature = nh_private.advertise<sensor_msgs::PointCloud2>("feature", 10, false);

    // Store IMU data in ringbuffer
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh_private, "/zed2_node/imu/data", 1);

    std::shared_ptr<message_filters::Cache<sensor_msgs::Imu>> imu_cache;
    imu_cache.reset(new message_filters::Cache<sensor_msgs::Imu>(sub_imu, 200));

    CloudFeatureExtractorParams params;
    params.parse_yaml(params_path);
    params.print();

    LaserParams laser_params;
    laser_params.parse_yaml("laser0", params_path);
    laser_params.print();

    CloudFeatureExtractor extractor(params, laser_params);

    double last_cloud_time = 0;

    std::mutex mtx_cloud;
    auto cloud_callback = [&](auto&& msg)
    {
        std::unique_lock<std::mutex> lck(mtx_cloud);

        const double curr_cloud_time = msg->header.stamp.toSec();
        if (last_cloud_time > curr_cloud_time)
        {
            // We need to reset imu_cache
            imu_cache.reset(new message_filters::Cache<sensor_msgs::Imu>(sub_imu, 200));
            return;
        }

        CloudFeatureExtractor::PointCloud cloud, full, corn, surf;
        pcl::fromROSMsg(*msg, cloud);

        // get IMU data
        DCHECK_GT(cloud.size(), 5000);
        ros::Time frame_start(curr_cloud_time);
        ros::Time frame_end(curr_cloud_time + params.time_interval_pts * cloud.size());
        auto imu_vector = imu_cache->getInterval(frame_start, frame_end);

        extractor.process(cloud, &imu_vector, full, corn, surf);

        cv::Mat debug_image = extractor.get_debug_image();
        auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
        pub_debug_image.publish(img_msg);

        last_cloud_time = curr_cloud_time;

        if(pub_surf.getNumSubscribers() > 0) pub_surf.publish(surf);
        if(pub_corn.getNumSubscribers() > 0) pub_corn.publish(corn);
        if(pub_full.getNumSubscribers() > 0) pub_full.publish(full);
        if(pub_feature.getNumSubscribers() > 0)
        {
            uom_msgs::cloud_feature::Ptr feat (new uom_msgs::cloud_feature);
            feat->header = msg->header;
            feat->pose.orientation.w = 1;
            pcl::toROSMsg(full, feat->full);
            pcl::toROSMsg(corn, feat->corn);
            pcl::toROSMsg(surf, feat->surf);
            pub_feature.publish(feat);
        }
    };

    auto sub_cloud = nh_private.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, cloud_callback);

    ros::spin();
    return 0;
}