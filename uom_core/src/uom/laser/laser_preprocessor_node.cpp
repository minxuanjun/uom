#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include "uom/utils/threadsafe_queue.hpp"
#include "uom_msgs/cloud_feature.h"
#include "uom/laser/laser_preprocessor.hpp"


class SimpleSynchronizer
{
public:

    SimpleSynchronizer(): cloud_data_queue_("cloud_data_queue") {}
    ~SimpleSynchronizer() = default;


    void init_ros_env()
    {
        nh_ = ros::NodeHandle();
        nh_private_ = ros::NodeHandle("~");

        sub_cloud_ = nh_private_.subscribe<sensor_msgs::PointCloud2>(
            "/livox/lidar", 10, &SimpleSynchronizer::add_cloud, this);

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
        std::unique_lock<std::mutex> lk(mtx_imu_queue_);
        imu_data_queue_.emplace(msg);
        lk.unlock();
        cv_imu_queue_.notify_one();
    }

    void add_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        cloud_data_queue_.push(msg);
    }


    inline bool is_imu_available(Timestamp t) const
    {
        return !imu_data_queue_.empty() && t <= imu_data_queue_.back()->header.stamp.toNSec();
    }


    bool get_imu_interval(Timestamp t0, Timestamp t1, std::vector<sensor_msgs::ImuConstPtr>& value)
    {
        value.clear();

        if (imu_data_queue_.empty())
        {
            ROS_WARN("not receive imu");
            return false;
        }

        if (t1 <= imu_data_queue_.back()->header.stamp.toNSec())
        {
            while (imu_data_queue_.front()->header.stamp.toNSec() <= t0)
            {
                imu_data_queue_.pop();
            }
            while (imu_data_queue_.front()->header.stamp.toNSec()< t1)
            {
                value.emplace_back(imu_data_queue_.front());
                imu_data_queue_.pop();
            }
            value.emplace_back(imu_data_queue_.front());
        }
        else
        {
            ROS_WARN("wait for imu");
            return false;
        }
        return true;
    }


    void internal_spin()
    {
        while(ros::ok())
        {
            /// Wait for cloud
            sensor_msgs::PointCloud2ConstPtr cloud_msg;
            cloud_data_queue_.pop_blocking(cloud_msg);

            if (last_cloud_time_ > cloud_msg->header.stamp.toNSec())
            {
                mtx_imu_queue_.lock();
                decltype(imu_data_queue_)().swap(imu_data_queue_); // clear all
                last_cloud_time_ = cloud_msg->header.stamp.toNSec();
                return;
            }
            last_cloud_time_ = cloud_msg->header.stamp.toNSec();

            if (callback_)
            {
                CloudFeatureExtractor::PointCloud  cloud;
                pcl::fromROSMsg(*cloud_msg, cloud);


                /// Wait for IMU

                Timestamp s_t = cloud_msg->header.stamp.toNSec();
                Timestamp e_t = cloud_msg->header.stamp.toNSec() + 0.1 * s_to_ns;

                // Try to lock
                std::unique_lock<std::mutex> imu_queue_lk(mtx_imu_queue_);

                // Wait until
                cv_imu_queue_.wait(imu_queue_lk, [&] { return is_imu_available(e_t); });

                std::vector<sensor_msgs::ImuConstPtr> imu_values;
                get_imu_interval(s_t, e_t, imu_values);

                // Unlock before notify.
                imu_queue_lk.unlock();
                cv_imu_queue_.notify_one();


                ROS_INFO_STREAM("imu_values size = " << imu_values.size());
                callback_(cloud, imu_values);
            }
        }
    }



    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_imu_, sub_cloud_;


    // buffers
    Timestamp last_cloud_time_ = 0;

    std::mutex mtx_imu_queue_;
    std::condition_variable cv_imu_queue_;
    std::queue<sensor_msgs::ImuConstPtr> imu_data_queue_;

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
    auto pub_feature = nh_private.advertise<sensor_msgs::PointCloud2>("feature", 10, false);


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
        header.stamp.fromNSec(cloud.header.stamp);
        auto img_msg = cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg();
        pub_debug_image.publish(img_msg);

        if(pub_surf.getNumSubscribers() > 0) pub_surf.publish(surf);
        if(pub_corn.getNumSubscribers() > 0) pub_corn.publish(corn);
        if(pub_full.getNumSubscribers() > 0) pub_full.publish(full);
        if(pub_feature.getNumSubscribers() > 0)
        {
            uom_msgs::cloud_feature::Ptr feat (new uom_msgs::cloud_feature);
            feat->header = header;
            feat->pose.orientation.w = 1;
            pcl::toROSMsg(full, feat->full);
            pcl::toROSMsg(corn, feat->corn);
            pcl::toROSMsg(surf, feat->surf);
            pub_feature.publish(feat);
        }
    };

    SimpleSynchronizer simple_synchronizer;

    simple_synchronizer.add_callback(cloud_callback);
    simple_synchronizer.init_ros_env();

    ros::spin();
    return 0;
}