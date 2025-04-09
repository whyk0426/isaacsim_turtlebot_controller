#include "isaac_tb3_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

IsaacsimTurtlebotController::IsaacsimTurtlebotController() : Node("isaac_tb3_controller"){
    //subscriber
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&IsaacsimTurtlebotController::scan_callback, this, _1));
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

void IsaacsimTurtlebotController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    if (!tf_flag)
        return;

    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;
    scan_flag = true;

    size_t num_points = msg->ranges.size();
    // for (size_t i = 0; i < num_points; i++){
    //     double angle_rad = angle_min + i * angle_increment;
    //     int angle_deg = static_cast<int>(std::round(angle_rad * 180.0 / M_PI));

    //     int index = (angle_deg + 360) % 360;
    //     lidar_distance[index] = msg->ranges[i];
    // }
    for (size_t i = 0; i < num_points; i++){
        lidar_distance[i] = msg->ranges[i];
    }
    RCLCPP_INFO(this->get_logger(), "r=%.2f", lidar_distance[0]);
    double closest_range = 1.0; //2초 후에 미래까지 봄. 0.22m/s로 2s 동안 가면 0.44m x2는 1.0m 정도. 다음 스텝까지도 예측에 영향을 안줌/
    int closest_deg = 0;

    for (int j = -90; j < 91; j++){
        if (lidar_distance[j] < closest_range){
            closest_range = lidar_distance[j];
            closest_deg = j;
            obs_flag = true;
        }
    }
    double closest_rad = closest_deg * M_PI / 180.0; 

    // world좌표계에서 장애물 좌표
    obs_x = closest_range * (cos(real_th)*cos(closest_rad) - sin(real_th)*sin(closest_rad)) + real_x;
    obs_y = closest_range * (sin(real_th)*cos(closest_rad) + cos(real_th)*sin(closest_rad)) + real_y;
}

void IsaacsimTurtlebotController::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    goal_flag = true;

    // RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%2f", goal_x, goal_y);
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

void IsaacsimTurtlebotController::cmd_timer_callback(){
    if ((!tf_flag)||(!goal_flag)||(!scan_flag))
        return;

    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = optimal_velocity()[0];
    cmd_vel.angular.z = optimal_velocity()[1];

    cmd_publisher->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "(v,w)=(%.2f, %.2f)", cmd_vel.linear.x, cmd_vel.angular.z);
}

//DWA
std::array<double, 2> IsaacsimTurtlebotController::optimal_velocity(){
    double v0 = v_opt;
    double w0 = w_opt;

    v_min = v0 - v_acc * dt;
    if (v_min < 0) 
        v_min = 0;
    v_max = v0 + v_acc * dt;
    if (v_max > 0.22) 
        v_max = 0.22;
    w_min = w0 - w_acc * dt;
    if (w_min < -1.0)
        w_min = -1.0;
    w_max = w0 + w_acc * dt;
    if (w_max > 1.0)
        w_max = 1.0;

    double min_cost = std::numeric_limits<double>::max();
    for (double w = w_min; w < w_max; w += w_res){
        for (double v = v_min; v < v_max; v += v_res){
            double cost = cost_calculator(v, w);
            if (cost < min_cost){
                min_cost = cost;
                v_opt = v;
                w_opt = w;
            }
        }
    }
    // RCLCPP_INFO(this->get_logger(), "(v,w)=(%.2f, %.2f)", v_opt, w_opt);
    return {v_opt, w_opt};
}

double IsaacsimTurtlebotController::cost_calculator(double v, double w){
    double x = 0.0, y = 0.0, theta = 0.0;
    double x0 = real_x;
    double y0 = real_y;
    double theta0 = real_th;

    double cost_obstacle = 0.0;

    for (double i = dt; i <= predict_time; i += dt){
        x = x0 + (v / w) * (sin(theta0 + w * i) - sin(theta0));
        y = y0 + (v / w) * (cos(theta0 + w * i) - cos(theta0));
        theta = theta0 + w * i;
        if(cost_obs(x, y) > cost_obstacle)
            cost_obstacle = cost_obs(x, y);
        x0 = x;
        y0 = y;
        theta0 = theta;
    }
    double cost_goal_pose = cost_goal(x, y);
    double cost_velocity = cost_vel(v);

    RCLCPP_INFO(this->get_logger(), "(goal, obs, vel)=(%.2f, %.2f, %.2f)", cost_goal_pose, cost_obstacle, cost_velocity);
    return goal_weight * cost_goal_pose + obs_weight * cost_obstacle + vel_weight * cost_velocity;
}

double IsaacsimTurtlebotController::cost_goal(double x, double y){
    double x_diff = x - goal_x;
    double y_diff = y - goal_y;
    
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double IsaacsimTurtlebotController::cost_obs(double x, double y){
    if (!obs_flag)
        return 0;

    double x_diff = x - obs_x;
    double y_diff = y - obs_y;
    double obs_distance = sqrt(x_diff * x_diff + y_diff * y_diff);
    
    if (obs_distance < 0.1)
        return std::numeric_limits<double>::max();

    return 1 / obs_distance;
}

double IsaacsimTurtlebotController::cost_vel(double v){
    return (0.22 - v);
}