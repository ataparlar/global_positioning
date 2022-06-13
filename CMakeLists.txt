cmake_minimum_required(VERSION 3.8)
project(global_positioning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(applanix_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(message_filters REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(gnss_odometry src/gnss_odometry.cpp)
ament_target_dependencies(gnss_odometry
        applanix_msgs
        tf2
        tf2_ros
        geometry_msgs
        sensor_msgs
        rclcpp
        GeographicLib
        message_filters
        )
target_include_directories(gnss_odometry PUBLIC
        include/
        )

target_link_libraries( gnss_odometry
        ${GeographicLib_LIBRARIES}
        )

install(TARGETS
        gnss_odometry
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
