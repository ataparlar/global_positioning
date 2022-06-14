//
// Created by ataparlar on 10.06.2022.

#ifndef BUILD_GNSS_ODOMETRY_H
#define BUILD_GNSS_ODOMETRY_H
#include <memory>

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

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>



class GnssOdometry : public rclcpp::Node
{

public:
    GnssOdometry();




    typedef message_filters::sync_policies::ApproximateTime<
            applanix_msgs::msg::NavigationSolutionGsof49,
            applanix_msgs::msg::NavigationPerformanceGsof50> approximate_policy;

//    message_filters::Synchronizer<approximate_policy> syncApproximate(
//            approximate_policy,
//            message_filters::Subscriber<applanix_msgs::msg::NavigationSolutionGsof49>,
//            message_filters::Subscriber<applanix_msgs::msg::NavigationPerformanceGsof50>
//            );


    GeographicLib::LocalCartesian locart;
    GeographicLib::Geocentric earth;
    tf2::Quaternion q;
    double x, y, z;
    int array_count;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_publisher;

    geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose;


private:
    void gnss_pose_callback(
            const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &msg,
            const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &rms);

//    message_filters::Subscriber<applanix_msgs::msg::NavigationSolutionGsof49> lla_subscriber_time;
//    message_filters::Subscriber<applanix_msgs::msg::NavigationPerformanceGsof50> rms_subscriber_time;




};



#endif //BUILD_GNSS_ODOMETRY_H