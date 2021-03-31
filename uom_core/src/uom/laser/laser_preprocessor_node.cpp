#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include "uom/utils/threadsafe_queue.hpp"
#include "uom/utils/temporal_buffer.hpp"
#include "uom_msgs/CloudFeature.h"
#include "uom/laser/laser_preprocessor.hpp"


class SimpleSynchronizer
{
public:

    SimpleSynchronizer(const LaserParams& laser_params) :
        laser_params_(laser_params), imu_data_buffer_(s_to_ns), cloud_data_queue_("cloud_data_queue"){}
    ~SimpleSynchronizer() = default;


    void init_ros_env()
    {
        nh_ = ros::NodeHandle();
        nh_private_ = ros::NodeHandle("~");

        sub_cloud_ = nh_private_.subscribe<sensor_msgs::PointCloud2>(
            laser_params_.rostopic, 10, &SimpleSynchronizer::add_cloud, this);

        sub_imu_ = nh_private_.subscribe<sensor_msgs::Imu>(
            "/zed2_node/imu/data", 10, &SimpleSynchronizer::add_imu, this);


        internal_spin_thread_ = std::thread([this]{ internal_spin();});
    }


    void add_callback(std::function<void(CloudFeatureExtractor::PointCloud&,
        std::vector<sensor_msgs::ImuConstPtr>&)> cb)
    {
        callback_ = cb;
    }


protected:


    void add_imu(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_data_buffer_.add_value(msg->header.stamp.toNSec(), msg);
    }

    void add_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        cloud_data_queue_.push(msg);
    }

    void internal_spin()
    {
        while(ros::ok())
        {
            /// Wait for cloud
            sensor_msgs::PointCloud2ConstPtr cloud_msg;

            if (!cloud_data_queue_.pop_blocking_with_timeout(cloud_msg, 10 * ms_to_ns))
            {
                continue;
            }

            Timestamp curr_cloud_timestamp = cloud_msg->header.stamp.toNSec();
            if (last_cloud_time_ > curr_cloud_timestamp)
            {
                imu_data_buffer_.clear();
            }
            last_cloud_time_ = curr_cloud_timestamp;

            if (callback_)
            {
                CloudFeatureExtractor::PointCloud  cloud;
                pcl::fromROSMsg(*cloud_msg, cloud);

                /// Wait for IMU
                const Timestamp s_t = curr_cloud_timestamp;
                const Timestamp e_t = curr_cloud_timestamp +
                    laser_params_.time_interval_pts * cloud.size() * s_to_ns;

                std::vector<sensor_msgs::ImuConstPtr> imu_values;
                auto result = imu_data_buffer_.get_value_between_blocking(s_t, e_t, &imu_values, nullptr, true);

                if (result == TemporalBufferQueryResult::AVAILABLE)
                {
                    ROS_INFO_STREAM("imu_values size = " << imu_values.size());
                    callback_(cloud, imu_values);
                }
                else
                {
                    ROS_INFO_STREAM("Drop cloud, reason = " << result << ", queue size = " << imu_data_buffer_.size());
                }
            }
        }
    }

    // parameters
    LaserParams laser_params_;

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_imu_, sub_cloud_;


    // buffers
    Timestamp last_cloud_time_ = 0;
    ThreadsafeTemporalBuffer<sensor_msgs::ImuConstPtr> imu_data_buffer_;
    ThreadsafeQueue<sensor_msgs::PointCloud2ConstPtr> cloud_data_queue_;

    // callback
    std::function<void(CloudFeatureExtractor::PointCloud&,
                       std::vector<sensor_msgs::ImuConstPtr>&)> callback_;


    std::thread internal_spin_thread_;
};


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
    auto pub_feature = nh_private.advertise<uom_msgs::CloudFeature>("feature", 10, false);


    CloudFeatureExtractorParams params;
    params.parse_yaml(params_path);
    params.print();

    LaserParams laser_params;
    laser_params.parse_yaml("laser0", params_path);
    laser_params.print();

    CloudFeatureExtractor extractor(params, laser_params);


    auto cloud_callback = [&](CloudFeatureExtractor::PointCloud& cloud,
        std::vector<sensor_msgs::ImuConstPtr>& imu_vector)
    {
        CloudFeatureExtractor::PointCloud full, corn, surf;
        extractor.process(cloud, &imu_vector, full, corn, surf);

        cv::Mat debug_image = extractor.get_debug_image();
        std_msgs::Header header;
        header.frame_id = header.frame_id;
        header.seq = header.seq;
        pcl_conversions::fromPCL(cloud.header.stamp, header.stamp);
        auto img_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
        pub_debug_image.publish(img_msg);

        if(pub_surf.getNumSubscribers() > 0) pub_surf.publish(surf);
        if(pub_corn.getNumSubscribers() > 0) pub_corn.publish(corn);
        if(pub_full.getNumSubscribers() > 0) pub_full.publish(full);
        if(pub_feature.getNumSubscribers() > 0)
        {
            uom_msgs::CloudFeature::Ptr feat (new uom_msgs::CloudFeature);
            feat->header = header;
            feat->pose.orientation.w = 1;
            pcl::toROSMsg(full, feat->full);
            pcl::toROSMsg(corn, feat->corn);
            pcl::toROSMsg(surf, feat->surf);
            pub_feature.publish(feat);
        }
    };

    SimpleSynchronizer simple_synchronizer(laser_params);

    simple_synchronizer.add_callback(cloud_callback);
    simple_synchronizer.init_ros_env();

    ros::spin();
    return 0;
}