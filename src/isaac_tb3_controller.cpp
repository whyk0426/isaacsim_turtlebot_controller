#include "isaac_tb3_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IsaacsimTurtlebotController::IsaacsimTurtlebotController() : Node("isaac_tb3_controller"){
    //goal subscriber
    goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_position", 10, std::bind(&IsaacsimTurtlebotController::goal_callback, this, _1));

    //publisher
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //TF Listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    //Timer 
    tf_timer = this->create_wall_timer(10ms, std::bind(&IsaacsimTurtlebotController::tf_timer_callback, this));
    cmd_timer = this->create_wall_timer(100ms, std::bind(&IsaacsimTurtlebotController::cmd_timer_callback, this));
}

void IsaacsimTurtlebotController::cmd_timer_callback(){
    if ((!tf_flag)||(!goal_flag))
        return;

    geometry_msgs::msg::Twist cmd_vel;


    


}

void IsaacsimTurtlebotController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    goal_flag = true;

    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%2f", goal_x, goal_y);
}

void IsaacsimTurtlebotController::tf_timer_callback(){
    geometry_msgs::msg::TransformStamped t;

    try{
        t = tf_buffer->lookupTransform("odom", "base_scan", tf2::TimePointZero);
    }   catch (const tf2::TransformException &ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
        return;
    }
    double z = t.transform.rotation.z;
    double w = t.transform.rotation.w;

    real_x = t.transform.translation.x;
    real_y = t.transform.translation.y;
    real_th = 2 * atan2(z,w);

    tf_flag = true;
}

