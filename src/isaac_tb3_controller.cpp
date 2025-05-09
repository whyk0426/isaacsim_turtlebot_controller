#include "isaac_tb3_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IsaacsimTurtlebotController::IsaacsimTurtlebotController() : Node("isaac_tb3_controller"){
    //parameter
    this->declare_parameter("robot_name", "robot_name");
    robot_name = this->get_parameter("robot_name").as_string();

    //subscriber
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&IsaacsimTurtlebotController::map_callback, this, _1));
    goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_position", 10, std::bind(&IsaacsimTurtlebotController::goal_callback, this, _1));

    //publisher
    cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

    //TF Listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    //Timer 
    tf_timer = this->create_wall_timer(10ms, std::bind(&IsaacsimTurtlebotController::tf_timer_callback, this));
    cmd_timer = this->create_wall_timer(100ms, std::bind(&IsaacsimTurtlebotController::cmd_timer_callback, this));
}

void IsaacsimTurtlebotController::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    dwa.map = *msg;

    map_flag = true;
}

void IsaacsimTurtlebotController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    goal_flag = true;

    dwa.goal_x = goal_x;
    dwa.goal_y = goal_y;

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";  
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
    marker.pose.position.z = 0.2;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_publisher->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%.2f", goal_x, goal_y);
}

void IsaacsimTurtlebotController::tf_timer_callback(){
    geometry_msgs::msg::TransformStamped t;

    try{
        t = tf_buffer->lookupTransform(robot_name + "_odom", robot_name + "_base_link", tf2::TimePointZero);
    }   catch (const tf2::TransformException &ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
        return;
    }
    double z = t.transform.rotation.z;
    double w = t.transform.rotation.w;

    real_x = t.transform.translation.x;
    real_y = t.transform.translation.y;
    real_th = 2 * atan2(z,w);

    // RCLCPP_INFO(this->get_logger(), "Robot state: x=%.2f y=%.2f", real_x, real_y);

    dwa.real_x = real_x;
    dwa.real_y = real_y;
    dwa.real_th = real_th;

    tf_flag = true;
}

void IsaacsimTurtlebotController::cmd_timer_callback(){
    if ((!tf_flag)||(!goal_flag)||(!map_flag))
        return;

    geometry_msgs::msg::Twist cmd_vel;

    RCLCPP_INFO(this->get_logger(), "Optical velocity: v=%.2f w=%.2f", dwa.optimal_velocity()[0], dwa.optimal_velocity()[1]);

    cmd_vel.linear.x = dwa.optimal_velocity()[0];
    cmd_vel.angular.z = dwa.optimal_velocity()[1];

    double distance_to_goal = sqrt(((goal_x - real_x) * (goal_x - real_x)) + (goal_y - real_y) * (goal_y - real_y));

    if (distance_to_goal < 0.1){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "ARRIVE");
        goal_flag = false;
    }
    cmd_publisher->publish(cmd_vel);
}

