#include "dynamic_window_approach.hpp"

DynamicWindowApproach::DynamicWindowApproach(){
    
}

std::array<double, 2> DynamicWindowApproach::optimal_velocity(){
    double v0 = v_opt;
    double w0 = w_opt;

    v_min = v0 - v_acc * dt;
    if (v_min < 0) v_min = 0;
    v_max = v0 + v_acc * dt;
    if (v_max > 0.22) v_max = 0.22;
    w_min = w0 - w_acc * dt;
    if (w_min < -1.0) w_min = -1.0;
    w_max = w0 + w_acc * dt;
    if (w_max > 1.0) w_max = 1.0;

    double min_cost = std::numeric_limits<double>::max();
    for (double w = w_min; w < w_max; w += w_res){
        for (double v = v_min; v < v_max; v += v_res){
            double cost = cost_calculator(v, w);
            if(cost < min_cost){
                min_cost = cost;
                v_opt = v;
                w_opt = w;
            }
        }
    }
    return {v_opt, w_opt};
}

double DynamicWindowApproach::cost_calculator(double v, double w){
    double x0 = real_x;
    double y0 = real_y;
    double theta0 = real_th;

    double x = 0.0, y = 0.0, theta = 0.0;
    double obstacle_cost = 0;

    for (double t = 0.0; t < predict_time; t += dt){
        theta = theta0 + w * t;
        x = (w==0) ? (x0 + v * sin(theta0) * t) : (x0 + (v/w) * (sin(theta0 + w * t) - sin(theta0)));
        y = (w==0) ? (y0 + v * cos(theta0) * t) : (y0 - (v/w) * (cos(theta0 + w * t) - cos(theta0)));

        if (cost_obs(x, y) > obstacle_cost)
            obstacle_cost = cost_obs(x, y);

        x0 = x;
        y0 = y;
        theta0 = theta;
    }
    
    return ((goal_weight * cost_goal(x, y))+(obs_weight * cost_obs(x, y))+(vel_weight * cost_vel(v)));
}

double DynamicWindowApproach::cost_goal(double x, double y){
    double x_diff = (goal_x - x);
    double y_diff = (goal_y - y);

    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double DynamicWindowApproach::cost_obs(double x, double y){
    double x_diff = (obs_x - x);
    double y_diff = (obs_y - y);

    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double DynamicWindowApproach::cost_vel(double vel){
    return (0.22 - vel);
}

// void IsaacsimTurtlebotController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
//     if (!tf_flag)
//         return;

//     double angle_min = msg->angle_min;
//     double angle_increment = msg->angle_increment;
//     scan_flag = true;

//     size_t num_points = msg->ranges.size();
//     // for (size_t i = 0; i < num_points; i++){
//     //     double angle_rad = angle_min + i * angle_increment;
//     //     int angle_deg = static_cast<int>(std::round(angle_rad * 180.0 / M_PI));

//     //     int index = (angle_deg + 360) % 360;
//     //     lidar_distance[index] = msg->ranges[i];
//     // }
//     for (size_t i = 0; i < num_points; i++){
//         lidar_distance[i] = msg->ranges[i];
//     }
//     RCLCPP_INFO(this->get_logger(), "r=%.2f", lidar_distance[0]);
//     double closest_range = 1.0; //2초 후에 미래까지 봄. 0.22m/s로 2s 동안 가면 0.44m x2는 1.0m 정도. 다음 스텝까지도 예측에 영향을 안줌/
//     int closest_deg = 0;

//     for (int j = -90; j < 91; j++){
//         if (lidar_distance[j] < closest_range){
//             closest_range = lidar_distance[j];
//             closest_deg = j;
//             obs_flag = true;
//         }
//     }
//     double closest_rad = closest_deg * M_PI / 180.0; 

//     // world좌표계에서 장애물 좌표
//     obs_x = closest_range * (cos(real_th)*cos(closest_rad) - sin(real_th)*sin(closest_rad)) + real_x;
//     obs_y = closest_range * (sin(real_th)*cos(closest_rad) + cos(real_th)*sin(closest_rad)) + real_y;
// }
