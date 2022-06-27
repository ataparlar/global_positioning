//
// Created by ataparlar on 08.06.2022.
//
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "applanix_msgs/msg/navigation_solution_gsof49.hpp"
#include "applanix_msgs/msg/navigation_performance_gsof50.hpp"
#include "global_positioning/gnss_odometry.h"

#include <GeographicLib/UTMUPS.hpp>


GnssOdometry::GnssOdometry() : Node("gnss_odometry")
{
    // subscribers from message_filters initialized
    lla_subscriber_time.subscribe(
            this,
            "/lvx_client/gsof/ins_solution_49");
    rms_subscriber_time.subscribe(
            this,
            "/lvx_client/gsof/ins_solution_rms_50");

    // synchronizer defined here with reset() function.
    // initialized with a new pointer object inside
    // with approximate policy
    sync_.reset(
            new Sync(
                    approximate_policy(10),
                    GnssOdometry::lla_subscriber_time,
                    GnssOdometry::rms_subscriber_time));

    // callback is called here.
    sync_->registerCallback(
            std::bind(
                    &GnssOdometry::gnss_pose_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

    // publisher initialized
    gnss_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/gnss/pose",
            10);
    gnss_pose_utm_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/gnss/pose/utm",
            10);

    // defined the earth model and local cartesian in order to
    // point an origin

}

// defined the callback function out of the constructor
void GnssOdometry::gnss_pose_callback(
        const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &msg,
        const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &rms) {

    // if is_first_msg, initialize again the local cartesian origin
    // with the first message lat long altitude and orientation message
    if(is_first_msg)
    {
        // set the origin of wgs84 local coordinate system
        locart.Reset(
                msg->lla.latitude,
                msg->lla.longitude,
                msg->lla.altitude
        );

        // set the origin of the UTM local coordinate system
        // used Forward because of the local coordinate calculation of UTM
        GeographicLib::UTMUPS::Forward(
                40.81187906,
                29.35810110,
                utm_origin.zone,
                utm_origin.northp,
                utm_origin.X,
                utm_origin.Y,
                utm_origin.gamma,
                utm_origin.k
        );

        q.setRPY(M_PI*(msg->roll)/180,
                 M_PI*(msg->pitch)/180,
                 M_PI*(msg->heading)/180);

        is_first_msg = false;

        RCLCPP_INFO(this->get_logger(), "Initialized Origin --");
    }
    // else, other values will be pushed to topic
    // as a message which contains the position and orientation information
    else
    {
        // local cartesian coordinate conversation
        locart.Forward(msg->lla.latitude,
                       msg->lla.longitude,
                       msg->lla.altitude,
                       wgs_local.x,
                       wgs_local.y,
                       wgs_local.z);


        // UTM global coordinates conversation
        double utm_x;
        double utm_y;
        double utm_local_lat;
        double utm_local_long;
        GeographicLib::UTMUPS::Forward(
                msg->lla.latitude,
                msg->lla.longitude,
                utm_origin.zone,
                utm_origin.northp,
                utm_x,
                utm_y,
                utm_origin.gamma,
                utm_origin.k
        );


//        RCLCPP_INFO(this->get_logger(), "Local Coordinates in UTM");
//        RCLCPP_INFO(this->get_logger(), "UTM - LAT: '%f'", utm_local_lat );
//        RCLCPP_INFO(this->get_logger(), "UTM - LONG: '%f'", utm_local_long );


        q.setRPY(M_PI*(msg->roll)/180,
                 M_PI*(msg->pitch)/180,
                 M_PI*(msg->heading)/180);


//        RCLCPP_INFO(this->get_logger(), "Local Coordinates in WGS84");
//        RCLCPP_INFO(this->get_logger(), "WGS84 - X: '%f'", wgs_local.x );
//        RCLCPP_INFO(this->get_logger(), "WGS84 - Y: '%f'", wgs_local.y );
//        RCLCPP_INFO(this->get_logger(), "WGS84 - Z: '%f'", wgs_local.z );
//        RCLCPP_INFO(this->get_logger(), "------");
//
//        RCLCPP_INFO(this->get_logger(), "Local Coordinates in UTM");
//        RCLCPP_INFO(this->get_logger(), "UTM - X: '%f'", utm_x - utm_origin.X );
//        RCLCPP_INFO(this->get_logger(), "UTM - Y: '%f'", utm_y - utm_origin.Y );
//        RCLCPP_INFO(this->get_logger(), "------");
//
//        RCLCPP_INFO(this->get_logger(), "Orientation");
//        RCLCPP_INFO(this->get_logger(), "quaternion x: '%f'", q.getX());
//        RCLCPP_INFO(this->get_logger(), "quaternion y: '%f'", q.getY());
//        RCLCPP_INFO(this->get_logger(), "quaternion z: '%f'", q.getZ());
//        RCLCPP_INFO(this->get_logger(), "quaternion w: '%f'", q.getW());
//        RCLCPP_INFO(this->get_logger(), "------");
//        RCLCPP_INFO(this->get_logger(), "------");
//        RCLCPP_INFO(this->get_logger(), "------ \n");

        // wgs84 local coordinate system part
        gnss_pose.header.frame_id = "map";
        gnss_pose.header.stamp = this->get_clock()->now();

        gnss_pose.pose.pose.position.x = wgs_local.x;
        gnss_pose.pose.pose.position.y = wgs_local.y;
        gnss_pose.pose.pose.position.z = wgs_local.z;

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

        // ------------------------------------------------
        // ------------------------------------------------

        // utm coordinate_system part
        gnss_pose_utm.header.frame_id = "map";
        gnss_pose_utm.header.stamp = this->get_clock()->now();

        // UTM local coordinate calculation is done here
        gnss_pose_utm.pose.pose.position.x = utm_x - utm_origin.X;
        gnss_pose_utm.pose.pose.position.y = utm_y - utm_origin.Y;

        gnss_pose_utm.pose.pose.orientation.x = q.getX();
        gnss_pose_utm.pose.pose.orientation.y = q.getY();
        gnss_pose_utm.pose.pose.orientation.z = q.getZ();
        gnss_pose_utm.pose.pose.orientation.w = q.getW();

        gnss_pose_utm.pose.covariance = {
                pow(rms->pos_rms_error.north, 2), 0, 0, 0, 0, 0,
                0, pow(rms->pos_rms_error.east, 2), 0, 0, 0, 0,
                0, 0, pow(rms->pos_rms_error.down, 2), 0, 0, 0,
                0, 0, 0, pow(rms->attitude_rms_error_roll, 2), 0, 0,
                0, 0, 0, 0, pow(rms->attitude_rms_error_pitch, 2), 0,
                0, 0, 0, 0, 0, pow(rms->attitude_rms_error_heading, 2)
        };

        gnss_pose_utm_publisher->publish(gnss_pose_utm);

    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssOdometry>());
    rclcpp::shutdown();
    return 0;
}