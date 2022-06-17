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
#include <GeographicLib/UTMUPS.hpp>


class GnssOdometry : public rclcpp::Node
{

public:
    GnssOdometry();


private:
    // callback function is declared
    void gnss_pose_callback(
            const applanix_msgs::msg::NavigationSolutionGsof49::ConstSharedPtr &msg,
            const applanix_msgs::msg::NavigationPerformanceGsof50::ConstSharedPtr &rms);

    // approximate policy type defined
    // will be used in synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
            applanix_msgs::msg::NavigationSolutionGsof49,
            applanix_msgs::msg::NavigationPerformanceGsof50> approximate_policy;

    // made a synchronizer type which uses appriximate policy
    typedef message_filters::Synchronizer<approximate_policy> Sync;

    // make this synchronizer a shared_ptr in order to reach the callback function
    // when the node will be run **********
    std::shared_ptr<Sync> sync_;

    // topic subscribers declared from message_filters
    message_filters::Subscriber<applanix_msgs::msg::NavigationSolutionGsof49> lla_subscriber_time;
    message_filters::Subscriber<applanix_msgs::msg::NavigationPerformanceGsof50> rms_subscriber_time;

    struct UTMOrigin{
        double X;
        double Y;
        double gamma;
        double k;
        bool northp;
        int zone;
    };
    UTMOrigin utm_origin;

    struct WGSLocal{
        // some variables for local cartesian calculations
        double x;
        double y;
        double z;
    };
    WGSLocal wgs_local;

    bool is_first_msg = true;


    // earth model for local cartesian is declared
    GeographicLib::Geocentric earth;
    GeographicLib::LocalCartesian locart;

    // quaternion declared for posewithcovariancestamped message
    tf2::Quaternion q;
    // posewithcovariancestamped message declared
    geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose_utm;

    // ROS2 publisher declared
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_utm_publisher;

};



#endif //BUILD_GNSS_ODOMETRY_H