#include "dynamic_window_approach.hpp"

DynamicWindowApproach::DynamicWindowApproach(){
    
}

std::array<double, 2> DynamicWindowApproach::optimal_velocity(){    
    arriving_flag = false;
    double v0 = v_opt;
    double w0 = w_opt;

    v_min = v0 - v_acc * dt;
    if (v_min < 0) v_min = 0;
    v_max = v0 + v_acc * dt;
    if (v_max > max_linear) v_max = max_linear;
    w_min = w0 - w_acc * dt;
    if (w_min < - max_angular) w_min = - max_angular;
    w_max = w0 + w_acc * dt;
    if (w_max > max_angular) w_max = max_angular;

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
    double error_x = goal_x - real_x;
    double error_y = goal_y - real_y;   //0.05
    if(sqrt(error_x*error_x + error_y*error_y)< 0.05){
        v_opt = 0.0;
        w_opt = 0.0;
        arriving_flag = true;
        RCLCPP_INFO(rclcpp::get_logger("DWA"), "ARRIVE");
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
        theta = theta0 + w * dt;
        x = (abs(w) <= EPSILON) ? (x0 + v * sin(theta0) * dt) : (x0 + (v/w) * (sin(theta0 + w * dt) - sin(theta0)));
        y = (abs(w) <= EPSILON) ? (y0 + v * cos(theta0) * dt) : (y0 - (v/w) * (cos(theta0 + w * dt) - cos(theta0)));

        double obs_cost = cost_obs(x, y);
        if (obs_cost > obstacle_cost)
            obstacle_cost = obs_cost;

        x0 = x;
        y0 = y;
        theta0 = theta;
    }
    // RCLCPP_INFO(rclcpp::get_logger("DWA"), "goal: %.3f, obs:%.3f, vel:%.3f angle: %.3f", 
    // (goal_weight * cost_goal(x, y)), (obs_weight * obstacle_cost), (vel_weight * cost_vel(v)), (angle_weight * cost_angle(x, y, theta)));
    // RCLCPP_INFO(rclcpp::get_logger("DWA"), "goal_x, goal_y (%.3f, %.3f)",goal_x, goal_y);

    return ((goal_weight * cost_goal(x, y))+(obs_weight * obstacle_cost)+(vel_weight * cost_vel(v))/*+(angle_weight * cost_angle(x, y, theta))*/);
}

double DynamicWindowApproach::cost_goal(double x, double y){
    double x_diff = (goal_x - x);
    double y_diff = (goal_y - y);

    return (sqrt(x_diff * x_diff + y_diff * y_diff));
}

double DynamicWindowApproach::cost_obs(double x, double y){
    size_t width = map.info.width;
    size_t height = map.info.height;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double resolution = map.info.resolution;

    double closest_obstacle = std::numeric_limits<double>::max();
    for (size_t b = 0; b < height; b++){
        for (size_t a = 0; a < width; a++){
            size_t index = a + b * width;

            if (index >= map.data.size()) continue; //실제 map의 크기가 w*h 보다 작은 경우도 있다고 함.
            if (map.data[index] > 50){
                double x_diff = (origin_x + a * resolution) - x;
                double y_diff = (origin_y + b * resolution) - y;
                double obstacle_distance = sqrt(x_diff * x_diff + y_diff * y_diff);

                if (obstacle_distance < closest_obstacle)
                    closest_obstacle = obstacle_distance;
            }
        }
    } 
    if (closest_obstacle < safety_radius)
        return std::numeric_limits<double>::max();

    return (closest_obstacle < max_obs) ? (1.0 / closest_obstacle) : 0;
}

double DynamicWindowApproach::cost_vel(double vel){
    return (max_linear - vel);
}

// double DynamicWindowApproach::cost_angle(double x, double y, double th){
//     double goal_th = atan2((goal_y - y), (goal_x - x));

//     if(goal_th > 2 * M_PI) goal_th -= 2 * M_PI;
//     else if(goal_th < 0)   goal_th += 2 * M_PI;

//     if(th > 2 * M_PI) th -= 2 * M_PI;
//     else if(th < 0)   th += 2 * M_PI;

//     double error_th = (goal_th - th)/1.57;

//     // RCLCPP_INFO(rclcpp::get_logger("DWA"), "goal_th: %.3f real_th: %.3f error_th: %.3f", goal_th, th, error_th);
//     return error_th * error_th;
// }

