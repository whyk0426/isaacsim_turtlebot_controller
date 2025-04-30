#ifndef DYNAMIC_WINDOW_APPROACH_HPP
#define DYNAMIC_WINDOW_APPROACH_HPP

#include <array>
#include <cmath>

class DynamicWindowApproach{
    public:
        DynamicWindowApproach();
        double real_x = 0.0, real_y = 0.0, real_th = 0.0;
        double goal_x = 0.0, goal_y = 0.0; 
        double obs_x = 0.0, obs_y = 0.0;
        
        std::array<double, 2> optimal_velocity();

    private:
        double cost_calculator(double v, double w);
        double cost_goal(double x, double y);
        double cost_obs(double x, double y);
        double cost_vel(double vel);

        bool goal_inside = true;
        double goal_weight = 1.0;
        double obs_weight = 1.0;
        double vel_weight = (goal_inside) ? 1.0: 0.0;

        double dt = 0.1;
        double predict_time = 2.0;
        double v_res = 0.01;
        double w_res = 0.1;

        double v_opt = 0.0, w_opt = 0.0;
        double v_min = 0.0, w_min = 0.0;
        double v_max = 0.0, w_max = 0.0;
        double v_acc = 0.4;
        double w_acc = 2.0;
};

#endif 

        // double obs_x = 0, obs_y = 0;

