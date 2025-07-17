#ifndef GLOBAL_PATH_HPP
#define GLOBAL_PATH_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <array>
#include <vector>
#include <queue>
#include <memory>
#include <unordered_set>
#include <utility>
#include <cmath>

struct Node{
    int x, y;
    double g_cost, h_cost;
    std::shared_ptr<Node> parent;

    Node(int x_, int y_, double g, double h, std::shared_ptr<Node> p):
        x(x_), y(y_), g_cost(g), h_cost(h), parent(p) {}

    double f_cost() const {
        return g_cost + h_cost;
    }
    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
};

struct pair_hash{
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};


class GlobalPath{
    public:
        GlobalPath();
        double robot_x = 0.0, robot_y = 0.0;
        double goal_x = 0.0, goal_y = 0.0;
        std::array<std::vector<double>, 2> row;

        nav_msgs::msg::OccupancyGrid map;

        void wayPoint();
    private:
        std::vector<std::vector<int>> grid_map;
        std::array<std::vector<int>, 2> path_();
        int robot_xc = 0, robot_yc = 0, goal_xc = 0, goal_yc = 0;

        double h_cost(int x, int y);

        void RealToGrid();
        std::vector<std::vector<int>> convert_mapTo2D();
        
        std::function<bool(const std::shared_ptr<Node>&, const std::shared_ptr<Node>&)> cmp;
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(cmp)> open_list;
        std::unordered_set<std::pair<int, int>, pair_hash> closed_list;
        std::unordered_map<std::pair<int, int>, double, pair_hash> g_cost_map;
};

#endif