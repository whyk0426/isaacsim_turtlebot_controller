#ifndef ISAAC_TB3_CONTROLLER_HPP
#define ISAAC_TB3_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

class IsaacsimTurtlebotController : public rclcpp::Node{
    public:
        IsaacsimTurtlebotController();
    
    private:
        void tf_timer_callback();
        void cmd_timer_callback();
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        

        double goal_x = 0.0, goal_y = 0.0;
        double real_x = 0.0, real_y = 0.0, real_th = 0.0;

        bool tf_flag = false, goal_flag = false;

    //Subscriber
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