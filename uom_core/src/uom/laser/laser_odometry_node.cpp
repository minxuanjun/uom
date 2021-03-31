#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include "uom/utils/threadsafe_queue.hpp"
#include "uom/utils/temporal_buffer.hpp"
#include "uom_msgs/CloudFeature.h"
#include "uom/laser/laser_odometry.hpp"


class SimpleSynchronizer
{
public:

    SimpleSynchronizer(): imu_data_buffer_(1 * s_to_ns), feature_cloud_queue_("feature_cloud_queue") {}
    ~SimpleSynchronizer() = default;


    void init_ros_env()
    {
        nh_ = ros::NodeHandle();
        nh_private_ = ros::NodeHandle("~");

        sub_cloud_ = nh_private_.subscribe<uom_msgs::CloudFeature>(
            "/laser_preprocessor_node/feature", 10, &SimpleSynchronizer::add_cloud_feature, this);

        sub_imu_ = nh_private_.subscribe<sensor_msgs::Imu>(
            "/zed2_node/imu/data", 10, &SimpleSynchronizer::add_imu, this);


        internal_spin_thread_ = std::thread([this]{ internal_spin();});
    }


    void add_callback(std::function<void(uom_msgs::CloudFeatureConstPtr &,
                                         std::vector<sensor_msgs::ImuConstPtr>&)> cb)
    {
        callback_ = cb;
    }


protected:


    void add_imu(const sensor_msgs::ImuConstPtr& msg)
    {
        imu_data_buffer_.add_value(msg->header.stamp.toNSec(), msg);
    }

    void add_cloud_feature(const uom_msgs::CloudFeatureConstPtr& msg)
    {
        feature_cloud_queue_.push(msg);
    }

    void internal_spin()
    {
        while(ros::ok())
        {
            /// Wait for cloud
            uom_msgs::CloudFeatureConstPtr feature_msg;

            if (!feature_cloud_queue_.pop_blocking_with_timeout(feature_msg, 10 * ms_to_ns))
            {
                continue;
            }

            Timestamp curr_feature_timestamp = feature_msg->header.stamp.toNSec();
            if (last_feature_time_ > curr_feature_timestamp)
            {
                imu_data_buffer_.clear();
            }

            if (callback_)
            {
                /// Wait for IMU
                std::vector<sensor_msgs::ImuConstPtr> imu_values;
                auto result = imu_data_buffer_.get_value_between_blocking(
                    last_feature_time_, curr_feature_timestamp, &imu_values, nullptr, true);

                if (result == TemporalBufferQueryResult::AVAILABLE)
                {
                    ROS_INFO_STREAM("imu_values size = " << imu_values.size());
                    callback_(feature_msg, imu_values);
                }
                else
                {
                    ROS_INFO_STREAM("Drop feature_msg, reason = " <<
                        result << ", queue size = " << imu_data_buffer_.size());
                }
            }

            last_feature_time_ = curr_feature_timestamp;
        }
    }

    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_imu_, sub_cloud_;


    // buffers
    Timestamp last_feature_time_ = 0;
    ThreadsafeTemporalBuffer<sensor_msgs::ImuConstPtr> imu_data_buffer_;
    ThreadsafeQueue<uom_msgs::CloudFeatureConstPtr> feature_cloud_queue_;

    // callback
    std::function<void(uom_msgs::CloudFeatureConstPtr&,
                       std::vector<sensor_msgs::ImuConstPtr>&)> callback_;


    std::thread internal_spin_thread_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_odometry_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    /// Parsing parameters

    std::string odom_params_path;
    std::string imu_params_path;
    std::string laser_params_path;
    std::string static_initializer_params_path;

    nh_private.getParam("odom_params_path", odom_params_path);
    nh_private.getParam("imu_params_path", imu_params_path);
    nh_private.getParam("laser_params_path", laser_params_path);
    nh_private.getParam("static_initializer_params_path", static_initializer_params_path);

    LaserParams laser_params;
    laser_params.parse_yaml("laser0", laser_params_path);
    laser_params.print();

    LaserOdometryParams odometry_params;
    odometry_params.parse_yaml(odom_params_path);
    odometry_params.print();

    ImuParams imu_params;
    imu_params.parse_yaml(imu_params_path);
    imu_params.print();

    StaticInitializerParams static_initializer_params;
    static_initializer_params.parse_yaml(static_initializer_params_path);
    static_initializer_params.print();

    /// Configure ROS interface
    auto pub_odom = nh_private.advertise<nav_msgs::Odometry>("pose", 10, true);
    auto pub_path = nh_private.advertise<nav_msgs::Odometry>("path", 10, true);

    nav_msgs::Odometry odom;
    nav_msgs::Path path;

    LaserOdometry odometry(odometry_params, laser_params, imu_params, static_initializer_params);


    auto cloud_callback = [&](uom_msgs::CloudFeatureConstPtr& cloud, std::vector<sensor_msgs::ImuConstPtr>& imu_vector)
    {

        /// Convert ros msg
        LaserOdometry::PointCloud::Ptr full, corn, surf;
        full.reset(new LaserOdometry::PointCloud);
        corn.reset(new LaserOdometry::PointCloud);
        surf.reset(new LaserOdometry::PointCloud);

        pcl::fromROSMsg(cloud->full, *full);
        pcl::fromROSMsg(cloud->corn, *corn);
        pcl::fromROSMsg(cloud->surf, *surf);

        ImuDataVector imu_data_vector;
        imu_data_vector.reserve(imu_vector.size());
        for (auto& item : imu_vector)
        {
            imu_data_vector.emplace_back( ImuData{
                .t = item->header.stamp.toSec(),
                .acc = Vector3d(item->linear_acceleration.x, item->linear_acceleration.y, item->linear_acceleration.z),
                .gyr = Vector3d(item->angular_velocity.x, item->angular_velocity.y, item->angular_velocity.z),
            });
        }

        /// Compute odometry
        odometry.process(nullptr, nullptr, surf, imu_data_vector);

        Quaterniond q_w;
        Vector3d t_w;
        odometry.get_odom(q_w, t_w);

        /// Send odometry and TF
        odom.header = cloud->header;
        odom.header.frame_id = "odom";
        odom.child_frame_id = cloud->header.frame_id;
        odom.pose.pose.orientation.w = q_w.w();
        odom.pose.pose.orientation.x = q_w.x();
        odom.pose.pose.orientation.y = q_w.y();
        odom.pose.pose.orientation.z = q_w.z();
        odom.pose.pose.position.x = t_w.x();
        odom.pose.pose.position.y = t_w.y();
        odom.pose.pose.position.z = t_w.z();
        pub_odom.publish(odom);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        path.header = odom.header;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        static tf::TransformBroadcaster transform_broadcaster;

        tf::StampedTransform transform;
        transform.child_frame_id_ = odom.child_frame_id;
        transform.frame_id_ = odom.header.frame_id;
        transform.stamp_ = odom.header.stamp;
        transform.setRotation(tf::Quaternion(q_w.x(),q_w.y(),q_w.z(),q_w.w()));
        transform.setOrigin(tf::Vector3(t_w.x(),t_w.y(),t_w.z()));
        transform_broadcaster.sendTransform(transform);
    };


    SimpleSynchronizer simple_synchronizer;
    simple_synchronizer.add_callback(cloud_callback);
    simple_synchronizer.init_ros_env();

    ros::spin();
    return 0;
}