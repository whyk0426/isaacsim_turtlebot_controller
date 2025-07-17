#ifndef ISAAC_TB3_CONTROLLER_HPP
#define ISAAC_TB3_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include "dynamic_window_approach.hpp"
#include "global_path.hpp"
#include "visualization_msgs/msg/marker.hpp"

class IsaacsimTurtlebotController : public rclcpp::Node
{
public:
    IsaacsimTurtlebotController();

private:
    void tf_timer_callback();
    void cmd_timer_callback();
    // void merge_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void path_marker();

    std::string robot_name;
    double x_offset, y_offset, theta_offset;

    bool tf_flag = false, goal_flag = false, map_flag = false;

    double goal_x = 0.0, goal_y = 0.0;
    double real_x = 0.0, real_y = 0.0, real_th = 0.0;

    DynamicWindowApproach dwa;
    GlobalPath a_star;

    // Subscriber
    //  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr merge_map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    // TF Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    // Timer
    rclcpp::TimerBase::SharedPtr cmd_timer, tf_timer;
};

#endif