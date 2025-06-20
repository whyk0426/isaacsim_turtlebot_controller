#ifndef DYNAMIC_WINDOW_APPROACH_HPP
#define DYNAMIC_WINDOW_APPROACH_HPP

#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"

class DynamicWindowApproach{
    public:
        DynamicWindowApproach();
        double real_x = 0.0, real_y = 0.0, real_th = 0.0;
        double goal_x = 0.0, goal_y = 0.0; 

        nav_msgs::msg::OccupancyGrid map;

        std::array<double, 2> optimal_velocity();
        bool arriving_flag = true;
    private:
        double cost_calculator(double v, double w);
        double cost_goal(double x, double y);
        double cost_obs(double x, double y);
        double cost_vel(double vel);
        // double cost_angle(double x, double y, double th);

        double goal_weight = 3.0;
        double obs_weight = 0.1; //1.0
        double vel_weight = 0.1; //0.5
        // double angle_weight = 0.1; //0.01

        double max_linear = 0.22;
        double max_angular = 1.0;
        double max_obs = 0.7; // predict_time * max_linear * 2 + safety_radius

        double dt = 0.1;
        double predict_time = 1.5;  //1.5
        double v_res = 0.02;    // 0.01
        double w_res = 0.05;

        double v_opt = 0.0, w_opt = 0.0;
        double v_min = 0.0, w_min = 0.0;
        double v_max = 0.0, w_max = 0.0;
        double v_acc = 0.4;
        double w_acc = 2.0;

        const double safety_radius = 0.25;
        const double EPSILON = 1e-3;
};

#endif 

        // double obs_x = 0, obs_y = 0;

