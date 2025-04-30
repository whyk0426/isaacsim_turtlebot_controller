#ifndef ISAAC_TB3_CONTROLLER_HPP
#define ISAAC_TB3_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <cmath>
#include "dynamic_window_approach.hpp"

class IsaacsimTurtlebotController : public rclcpp::Node{
    public:
        IsaacsimTurtlebotController();

    private:
        void tf_timer_callback();
        void cmd_timer_callback();
        // void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        bool tf_flag = false, goal_flag = false;

        double goal_x = 0.0, goal_y = 0.0;
        double real_x = 0.0, real_y = 0.0, real_th = 0.0;
        


    DynamicWindowApproach dwa;

    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;
    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    //TF Listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    //Timer
    rclcpp::TimerBase::SharedPtr cmd_timer, tf_timer;
};

#endif