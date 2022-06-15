//
// Created by ataparlar on 08.06.2022.
//
#include <memory>
#include <cmath>

#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "applanix_msgs/msg/navigation_solution_gsof49.hpp"
#include "applanix_msgs/msg/navigation_performance_gsof50.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "global_positioning/gnss_odometry.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>


GnssOdometry::GnssOdometry() : Node("gnss_odometry")
{

//    message_filters::Subscriber<applanix_msgs::msg::NavigationSolutionGsof49> lla_subscriber_time(
//            this,
//            "/lvx_client/gsof/ins_solution_49");
//
//
//    message_filters::Subscriber<applanix_msgs::msg::NavigationPerformanceGsof50> rms_subscriber_time(
//            this,
//            "/lvx_client/gsof/ins_solution_rms_50");



//    message_filters::syncApproximate(
//            approximate_policy(10),
//            lla_subscriber_time,
//            rms_subscriber_time);

    message_filters::Synchronizer<GnssOdometry::approximate_policy> syncApproximate(
            approximate_policy(10),
            lla_subscriber_time,
            rms_subscriber_time)
            ;


//    syncApproximate.registerCallback(
//            &GnssOdometry::gnss_pose_callback,
//            this
//    );

    GnssOdometry::lla_subscriber_time.subscribe(this, "/lvx_client/gsof/ins_solution_49");
    GnssOdometry::rms_subscriber_time.subscribe(this, "/lvx_client/gsof/ins_solution_rms_50");

    //GnssOdometry::ptr_subs.

    sync_.reset(
            new Sync(
                    approximate_policy(10),
                    GnssOdometry::lla_subscriber_time,
                    GnssOdometry::rms_subscriber_time));


    sync_->registerCallback(
            std::bind(
                    &GnssOdometry::gnss_pose_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));


//            message_filters::TimeSynchronizer<
//                    applanix_msgs::msg::NavigationSolutionGsof49,
//                    applanix_msgs::msg::NavigationPerformanceGsof50>
//    sync(
//            lla_subscriber_time,
//            rms_subscriber_time,
//            1000);


        gnss_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/gnss/pose",
                10);
        // gnss_path_publisher = this->create_publisher<nav_msgs::msg::Path>(
        //         "/gnss/path",
        //         1);


        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        GeographicLib::LocalCartesian locart(
                0,
                0,
                0,
                earth);

        RCLCPP_INFO(this->get_logger(), "array count '%d'", array_count);

}

void GnssOdometry::gnss_pose_callback(
        const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &msg,
        const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &rms) {

    if(array_count==0)
    {
        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
        locart.Reset(
                msg->lla.latitude,
                msg->lla.longitude,
                msg->lla.altitude
        );

        q.setRPY(M_PI*(msg->roll)/180,
                 M_PI*(msg->pitch)/180,
                 M_PI*(msg->heading)/180);

        RCLCPP_INFO(this->get_logger(), "Initialized Origin --");
        RCLCPP_INFO(this->get_logger(), "Lat: '%f'", msg->lla.latitude);
        RCLCPP_INFO(this->get_logger(), "Long: '%f'", msg->lla.longitude);
        RCLCPP_INFO(this->get_logger(), "Alt: '%f'", msg->lla.altitude);
        RCLCPP_INFO(this->get_logger(), "------");
        RCLCPP_INFO(this->get_logger(), "quaternion x: '%f'", q.getX());
        RCLCPP_INFO(this->get_logger(), "quaternion y: '%f'", q.getY());
        RCLCPP_INFO(this->get_logger(), "quaternion z: '%f'", q.getZ());
        RCLCPP_INFO(this->get_logger(), "quaternion w: '%f'", q.getW());

        array_count++;
    }
    else
    {
        locart.Forward(msg->lla.latitude,
                       msg->lla.longitude,
                       msg->lla.altitude,
                       x,
                       y,
                       z);

        q.setRPY(M_PI*(msg->roll)/180,
                 M_PI*(msg->pitch)/180,
                 M_PI*(msg->heading)/180);


        RCLCPP_INFO(this->get_logger(), "X: '%f'", x );
        RCLCPP_INFO(this->get_logger(), "Y: '%f'", y );
        RCLCPP_INFO(this->get_logger(), "Z: '%f'", z );
        RCLCPP_INFO(this->get_logger(), "quaternion x: '%f'", q.getX());
        RCLCPP_INFO(this->get_logger(), "quaternion y: '%f'", q.getY());
        RCLCPP_INFO(this->get_logger(), "quaternion z: '%f'", q.getZ());
        RCLCPP_INFO(this->get_logger(), "quaternion w: '%f'", q.getW());
        RCLCPP_INFO(this->get_logger(), "------ \n");

        gnss_pose.header.frame_id = "map";
        gnss_pose.header.stamp = this->get_clock()->now();

        gnss_pose.pose.pose.position.x = x;
        gnss_pose.pose.pose.position.y = y;
        gnss_pose.pose.pose.position.z = z;

        gnss_pose.pose.pose.orientation.x = q.getX();
        gnss_pose.pose.pose.orientation.y = q.getY();
        gnss_pose.pose.pose.orientation.z = q.getZ();
        gnss_pose.pose.pose.orientation.w = q.getW();

        gnss_pose.pose.covariance = {
                pow(rms->pos_rms_error.north, 2), 0, 0, 0, 0, 0,
                0, pow(rms->pos_rms_error.east, 2), 0, 0, 0, 0,
                0, 0, pow(rms->pos_rms_error.down, 2), 0, 0, 0,
                0, 0, 0, pow(rms->attitude_rms_error_roll, 2), 0, 0,
                0, 0, 0, 0, pow(rms->attitude_rms_error_pitch, 2), 0,
                0, 0, 0, 0, 0, pow(rms->attitude_rms_error_heading, 2)
        };

        gnss_pose_publisher->publish(gnss_pose);


    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssOdometry>());
    rclcpp::shutdown();
    return 0;
}