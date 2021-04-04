#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <thread>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>

#include "uom/utils/math_utils.hpp"
#include "uom/sim/simple_simulator.hpp"
#include "uom/imu/pre_integration.hpp"


void pub_imu_msg(std::vector<SimpleSimulator::MotionData>::const_iterator it, ros::Publisher& pub, bool pub_tf = false)
{
    sensor_msgs::ImuPtr imu_ptr(new sensor_msgs::Imu);
    imu_ptr->header.frame_id = "base_link";
    imu_ptr->header.stamp.fromSec(it->t);
    imu_ptr->orientation.w = it->q.w();
    imu_ptr->orientation.x = it->q.x();
    imu_ptr->orientation.y = it->q.y();
    imu_ptr->orientation.z = it->q.z();
    imu_ptr->angular_velocity.x = it->gyr_m.x();
    imu_ptr->angular_velocity.y = it->gyr_m.y();
    imu_ptr->angular_velocity.z = it->gyr_m.z();
    imu_ptr->linear_acceleration.x = it->acc_m.x();
    imu_ptr->linear_acceleration.y = it->acc_m.y();
    imu_ptr->linear_acceleration.z = it->acc_m.z();

    pub.publish(imu_ptr);

    if (pub_tf)
    {
        tf::StampedTransform transform;
        transform.stamp_.fromSec(it->t);
        transform.frame_id_ = "world";
        transform.child_frame_id_ = "base_link";
        transform.setOrigin({it->p.x(),it->p.y(),it->p.z()});
        transform.setRotation({it->q.x(),it->q.y(),it->q.z(),it->q.w()});

        static tf::TransformBroadcaster transform_broadcaster;
        transform_broadcaster.sendTransform(transform);
    }

}

void pub_odom(const State& x, const std::string& frame_id, ros::Publisher& pub)
{
    nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
    odom->header.stamp.fromSec(x.t);
    odom->header.frame_id = "world";
    odom->child_frame_id = frame_id;
    odom->pose.pose.position.x = x.p.x();
    odom->pose.pose.position.y = x.p.y();
    odom->pose.pose.position.z = x.p.z();
    odom->pose.pose.orientation.x = x.q.x();
    odom->pose.pose.orientation.y = x.q.y();
    odom->pose.pose.orientation.z = x.q.z();
    odom->pose.pose.orientation.w = x.q.w();

    pub.publish(odom);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_simulator_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int integral_method;
    std::string sim_data_path;
    std::string sim_params_path;
    std::string sim_output;

    nh_private.getParam("sim_data", sim_data_path);
    nh_private.getParam("sim_params", sim_params_path);
    nh_private.getParam("sim_output", sim_output);
    nh_private.getParam("integral_method", integral_method);

    SimParams params;
    params.parse_yaml(sim_params_path);
    params.print();

    auto pub_imu_gt = nh_private.advertise<sensor_msgs::Imu>(params.imu_rostopic + "_gt", 400);
    auto pub_imu_noised = nh_private.advertise<sensor_msgs::Imu>(params.imu_rostopic, 400);
    auto pub_cam_pose = nh_private.advertise<nav_msgs::Odometry>("cam_pose", 1, true);
    auto pub_integral_test_pose = nh_private.advertise<nav_msgs::Odometry>("test_pose", 1, true);
    auto pub_image = nh_private.advertise<sensor_msgs::Image>(params.cam_rostopic, 30);
    auto pub_lines = nh_private.advertise<visualization_msgs::Marker>("lines", 1, true);
    auto pub_points = nh_private.advertise<sensor_msgs::PointCloud2>("points", 1, true);

    SimpleSimulator simple_simulator(sim_data_path);
    auto sim_data = simple_simulator.generate_sim_data(params);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto it_imu_data_gt = sim_data.imu_data_gt.cbegin();
    auto it_imu_data_noised = sim_data.imu_data_noised.cbegin();
    auto it_cam_pose = sim_data.cam_pose.cbegin();
    auto it_points_cam = sim_data.points_cam.cbegin();
    auto it_features_cam = sim_data.features_cam.cbegin();
    auto it_lines_cam = sim_data.lines_cam.cbegin();


    // publish all world lines and points first
    auto all_lines = simple_simulator.get_all_lines();
    auto all_points = simple_simulator.get_all_points();

    pcl::PointCloud<pcl::PointXYZ> feature_points;
    feature_points.header.stamp = 0;
    feature_points.header.frame_id = "world";
    for (auto& pt: all_points)
    {
        feature_points.emplace_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
    }
    pub_points.publish(feature_points);

    visualization_msgs::Marker line_list;
    line_list.header.stamp.fromSec(0);
    line_list.header.frame_id = "world";
    line_list.header.seq = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.05;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    for (auto& line : all_lines)
    {
        geometry_msgs::Point p;

        p.x = line.first.x();
        p.y = line.first.y();
        p.z = line.first.z();
        line_list.points.push_back(p);

        p.x = line.second.x();
        p.y = line.second.y();
        p.z = line.second.z();
        line_list.points.push_back(p);
    }
    pub_lines.publish(line_list);

    /// for imu integration
    double dt = 1.0 / static_cast<double>(params.rate);

    State integration_state {
        .p = it_imu_data_gt->p,
        .v = it_imu_data_gt->v,
        .q = it_imu_data_gt->q
    };

    PreIntegration integration(it_imu_data_gt->acc_m,
                               it_imu_data_gt->gyr_m,
                               static_cast<ImuParams>(params));

    std::ofstream f_test(sim_output+"/traj.csv");
    std::ofstream f_gt(sim_output+"/gt.csv");

    f_test << "# timestamp(s) x y z q_x q_y q_z q_w\n";
    f_gt << "# timestamp(s) x y z q_x q_y q_z q_w\n";

    ros::Rate rate(params.rate);
    while (ros::ok())
    {
        ros::spinOnce();

        /// Send
        pub_imu_msg(it_imu_data_gt, pub_imu_gt, true);
        pub_imu_msg(it_imu_data_noised, pub_imu_noised);

        /// If we have camera frame in current stamp.
        if (it_cam_pose->t <= it_imu_data_gt->t)
        {
            cv::Mat image_data(params.resolution, CV_8UC3, cv::Scalar(45, 45, 45));

            for (auto& line : *it_lines_cam)
            {
                cv::line(image_data, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 0, 0), 2,
                         cv::LINE_AA);
            }

            for (auto& pt : *it_features_cam)
            {
                cv::circle(image_data, cv::Point(pt.x(), pt.y()), 4, cv::Scalar(0, 0, 255), -1);
            }

            std_msgs::Header header;
            header.seq = std::distance(it_cam_pose, sim_data.cam_pose.cbegin());
            header.stamp.fromSec(it_cam_pose->t);
            pub_image.publish(cv_bridge::CvImage(header, "bgr8", image_data).toImageMsg());

            pub_odom(*it_cam_pose, "cam", pub_cam_pose);

            ++it_cam_pose;
            ++it_features_cam;
            ++it_lines_cam;
            ++it_points_cam;
            if (it_cam_pose == sim_data.cam_pose.cend())
            {
                it_cam_pose = sim_data.cam_pose.cbegin();
                it_features_cam = sim_data.features_cam.cbegin();
                it_lines_cam = sim_data.lines_cam.cbegin();
                it_points_cam = sim_data.points_cam.cbegin();
            }
        }

        integration.push_back(dt, it_imu_data_gt->acc_m, it_imu_data_gt->gyr_m);


        /// Testing integration

        static Vector3d gyr_w_prev = it_imu_data_gt->gyr_m;
        static Vector3d acc_w_prev = it_imu_data_gt->acc_m;

        integration_state.t = it_imu_data_gt->t;

        Vector3d gyr_w, acc_w;

        if (integral_method == 1)
        {
            acc_w = integration_state.q * acc_w_prev;
            gyr_w = 0.5 * (it_imu_data_gt->gyr_m + gyr_w_prev);
            integration_state.q = integration_state.q * delta_Q(gyr_w * dt);
            acc_w = 0.5 * (acc_w + integration_state.q * it_imu_data_gt->acc_m) + params.n_gravity;
        }
        else
        {
            gyr_w = it_imu_data_gt->gyr_m;
            integration_state.q = integration_state.q * delta_Q(gyr_w * dt);
            acc_w = integration_state.q * it_imu_data_gt->acc_m + params.n_gravity;
        }
        integration_state.p = integration_state.p + integration_state.v * dt + 0.5 * dt * dt * acc_w;
        integration_state.v = integration_state.v + acc_w * dt;

        acc_w_prev = it_imu_data_gt->acc_m;
        gyr_w_prev = it_imu_data_gt->gyr_m;

        /// Save trajectory
        integration_state.print(f_test); f_test << std::endl;
        it_imu_data_gt->print(f_gt); f_gt << std::endl;

        /// Send integration result
        pub_odom(integration_state, "base_link", pub_integral_test_pose);

        ++it_imu_data_gt;
        ++it_imu_data_noised;

        if (it_imu_data_gt == sim_data.imu_data_gt.cend())
        {
            break;
        }

        rate.sleep();
    }

    f_test.close();
    f_gt.close();

    ros::shutdown();
    return 0;
}