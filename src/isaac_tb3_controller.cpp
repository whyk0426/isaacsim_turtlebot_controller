#include "isaac_tb3_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IsaacsimTurtlebotController::IsaacsimTurtlebotController() : Node("isaac_tb3_controller"){
    //parameter
    this->declare_parameter("robot_name", "robot_name");
    robot_name = this->get_parameter("robot_name").as_string();
    this->declare_parameter("x_offset", 0.0);
    x_offset = this->get_parameter("x_offset").as_double();
    this->declare_parameter("y_offset", 0.0);
    y_offset = this->get_parameter("y_offset").as_double();
    this->declare_parameter("theta_offset", 0.0);
    theta_offset = this->get_parameter("theta_offset").as_double();

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
    a_star.map = *msg;

    map_flag = true;
}

void IsaacsimTurtlebotController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    goal_flag = true;
    
    // dwa.goal_x = goal_x * cos(theta_offset) - goal_y * sin(theta_offset) + x_offset;
    // dwa.goal_y = goal_x * sin(theta_offset) + goal_y * cos(theta_offset) + y_offset;

    a_star.goal_x = goal_x * cos(theta_offset) - goal_y * sin(theta_offset) + x_offset;
    a_star.goal_y = goal_x * sin(theta_offset) + goal_y * cos(theta_offset) + y_offset;
    
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%.2f", goal_x, goal_y);
    // path_marker();
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

    // RCLCPP_INFO(this->get_logger(), "Robot state: theata=%.2f", real_th);

    dwa.real_x = real_x;
    dwa.real_y = real_y;
    dwa.real_th = real_th;

    a_star.robot_x = real_x;
    a_star.robot_y = real_y;
    // if (!goal_flag||dwa.arriving_flag){
    //     a_star.robot_x = real_x;
    //     a_star.robot_y = real_y;
    // }
    tf_flag = true;
}

void IsaacsimTurtlebotController::cmd_timer_callback(){
    if ((!tf_flag)||(!goal_flag)||(!map_flag))
        return;

    geometry_msgs::msg::Twist cmd_vel;

    if (dwa.arriving_flag){
        path_marker();

        dwa.goal_x = a_star.row[0][0];
        dwa.goal_y = a_star.row[1][0];

        dwa.arriving_flag = false;
    }

    RCLCPP_INFO(this->get_logger(), "Optical velocity: v=%.2f w=%.2f", dwa.optimal_velocity()[0], dwa.optimal_velocity()[1]);

    cmd_vel.linear.x = dwa.optimal_velocity()[0];
    cmd_vel.angular.z = dwa.optimal_velocity()[1];

    // double distance_to_goal = sqrt(((goal_x - real_x) * (goal_x - real_x)) + (goal_y - real_y) * (goal_y - real_y));
    // if (distance_to_goal < 0.05) goal_flag = false;

    cmd_publisher->publish(cmd_vel);

    tf_flag = false;
}

void IsaacsimTurtlebotController::path_marker(){
    a_star.wayPoint();

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";  
    marker.header.stamp = this->get_clock()->now();
    marker.ns = robot_name + "_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.01;

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = robot_name + "_goal";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.01;

    std_msgs::msg::ColorRGBA waypoint_color;
    waypoint_color.r = 1.0;
    waypoint_color.g = 1.0;
    waypoint_color.b = 0.0;
    waypoint_color.a = 1.0;

    std_msgs::msg::ColorRGBA goal_color;
    goal_color.r = 1.0;
    goal_color.g = 0.0;
    goal_color.b = 0.0;
    goal_color.a = 1.0;

    std_msgs::msg::ColorRGBA line_color;
    line_color.r = 1.0;
    line_color.g = 1.0;
    line_color.b = 0.0;
    line_color.a = 1.0;
    line_marker.color = line_color;

    size_t n = a_star.row[0].size();
    for(size_t i = 0; i < n; i++){
        geometry_msgs::msg::Point pt;
        pt.x = a_star.row[0][i] * cos(theta_offset) + a_star.row[1][i] * sin(theta_offset) - x_offset;
        pt.y = a_star.row[0][i] * sin(theta_offset) + a_star.row[1][i] * cos(theta_offset) - y_offset;
        pt.z = 0.2;
        marker.points.push_back(pt);

        if ( i == n - 1)
            marker.colors.push_back(goal_color);
        else
            marker.colors.push_back(waypoint_color);
        line_marker.points.push_back(pt);
    }
    marker_publisher->publish(marker);
    marker_publisher->publish(line_marker);
}
