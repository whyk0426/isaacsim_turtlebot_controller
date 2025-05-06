#include "dynamic_window_approach.hpp"

DynamicWindowApproach::DynamicWindowApproach(){
    
}

std::array<double, 2> DynamicWindowApproach::optimal_velocity(){
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

        double obs_cost = cost_obs(x, y);
        if (obs_cost > obstacle_cost)
            obstacle_cost = obs_cost;

        x0 = x;
        y0 = y;
        theta0 = theta;
    }
    // RCLCPP_INFO(rclcpp::get_logger("DWA"), "goal: %.3f, obs:%.3f, vel:%.3f angle: %.3f", (goal_weight * cost_goal(x, y)), (obs_weight * obstacle_cost), (vel_weight * cost_vel(v)), (angle_weight * cost_angle(x, y, theta)));

    return ((goal_weight * cost_goal(x, y))+(obs_weight * obstacle_cost)+(vel_weight * cost_vel(v)));
}

double DynamicWindowApproach::cost_goal(double x, double y){
    double x_diff = (goal_x - x);
    double y_diff = (goal_y - y);

    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double DynamicWindowApproach::cost_obs(double x, double y){
    int width = map.info.width;
    int height = map.info.height;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    double resolution = map.info.resolution;

    double closest_obstacle = std::numeric_limits<double>::max();

    for (int b = 0; b < height; b++){
        for (int a = 0; a < width; a++){
            int index = a + b * width;

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
//     double goal_th = atan2((goal_y - x), (goal_x - y));
//     double error_th = goal_th - th;

//     if (error_th > (2 * M_PI))
//         error_th = error_th - 2 * M_PI;
//     else if (error_th < (- 2 * M_PI))
//         error_th =  error_th + 2 * M_PI;

//     goal_inside = (abs(error_th) < 0.5 * M_PI) ? true : false;

//     return error_th;
// }

