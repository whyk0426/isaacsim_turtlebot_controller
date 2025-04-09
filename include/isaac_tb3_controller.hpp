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
#include <array>

class IsaacsimTurtlebotController : public rclcpp::Node{
    public:
        IsaacsimTurtlebotController();
    
    private:
        void tf_timer_callback();
        void cmd_timer_callback();
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        
        double goal_x = 0.0, goal_y = 0.0;
        double real_x = 0.0, real_y = 0.0, real_th = 0.0;
        double obs_x = 0, obs_y = 0;

        double lidar_distance[360] = {0};

        bool tf_flag = false, goal_flag = false, scan_flag = false, obs_flag = false;

        std::array<double, 2> optimal_velocity();
        double cost_calculator(double v, double w);
        double cost_goal(double x, double y);
        double cost_obs(double x, double y);
        double cost_vel(double v);


        double dt = 0.1;
        double predict_time = 2.0;
        double v_res = 0.01;
        double w_res = 0.1;

        double goal_weight = 1.0;
        double obs_weight = 1.0;
        double vel_weight = 1.0;

        double v_opt = 0.0, w_opt = 0.0;
        double v_min = 0.0, v_max = 0.0;
        double w_min = 0.0, w_max = 0.0;
        double v_acc = 0.4;
        double w_acc = 2.0;


    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
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