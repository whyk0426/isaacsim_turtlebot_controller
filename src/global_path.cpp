#include "global_path.hpp"

GlobalPath::GlobalPath():
    cmp([](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
        return a->f_cost() > b->f_cost();
    }),
    open_list(cmp)
{}

void GlobalPath::wayPoint(){
    row[0] = {};
    row[1] = {};

    auto path_result = path_();
    size_t path_size = path_result[0].size();
    if (path_size < 7) return;

    
    // bool prev_near_obstacle = false;
    for (size_t i = 5; i < path_size - 1; i++){
        bool near_obstacle = false;

        for (int dy = -1; dy <= 1; dy++){
            for (int dx = -1; dx <= 1; dx++){
                int nx = path_result[0][i] + dx;
                int ny = path_result[1][i] + dy;
                if(nx >= 0 && nx < (int)map.info.width && ny >= 0 && ny < (int)map.info.height) {
                    if (grid_map[ny][nx] > 50){
                        near_obstacle = true;
                        break;
                    }
                }
            }
            if (near_obstacle) break;
        }
        
        int x0 = path_result[0][i-1], y0 = path_result[1][i-1];
        int x1 = path_result[0][i],   y1 = path_result[1][i];
        int x2 = path_result[0][i+1], y2 = path_result[1][i+1];

        double vx1 = x1 - x0, vy1 = y1 - y0;
        double vx2 = x2 - x1, vy2 = y2 - y1;

        double dot = vx1 * vx2 + vy1 * vy2;
        double mag1 = sqrt(vx1*vx2 + vy1*vy1);
        double mag2 = sqrt(vx2*vx2 + vy2*vy2);

        double cos_theta = (mag1 * mag2 > 1e-6) ? dot / (mag1 * mag2) : 1.0;

        bool angle_changed = (cos_theta < cos(M_PI / 6));

        if (near_obstacle && angle_changed){
            row[0].push_back(x1 * map.info.resolution + map.info.origin.position.x);
            row[1].push_back(y1 * map.info.resolution + map.info.origin.position.y);
        }
        //prev_near_obstacle = near_obstacle;
    }
    // row[0].push_back(path_result[0][int(0.5 * (path_size-1))] * map.info.resolution + map.info.origin.position.x);
    // row[1].push_back(path_result[1][int(0.5 * (path_size-1))] * map.info.resolution + map.info.origin.position.y);
    row[0].push_back(path_result[0][path_size-1] * map.info.resolution + map.info.origin.position.x);
    row[1].push_back(path_result[1][path_size-1] * map.info.resolution + map.info.origin.position.y);
}

std::array<std::vector<int> ,2> GlobalPath::path_(){
    grid_map = convert_mapTo2D();
    RealToGrid();

    std::array<std::vector<int>, 2> path;
    path[0] = {};
    path[1] = {};

    open_list = decltype(open_list)(cmp);
    closed_list.clear();
    g_cost_map.clear();

    auto start = std::make_shared<Node>(robot_xc, robot_yc, 0.0, h_cost(robot_xc, robot_yc), nullptr);
    open_list.push(start);

    while(!open_list.empty()){
        auto current = open_list.top();
        open_list.pop();

        if(current->x == goal_xc && current->y == goal_yc){
            while(current){ // current != nullptr 이라는 뜻. 즉 시작 점까지 올라갈때 까지
                path[0].push_back(current->x);
                path[1].push_back(current->y);
                current = current->parent;
            }
            std::reverse(path[0].begin(), path[0].end());
            std::reverse(path[1].begin(), path[1].end());

            return path;
        }
        closed_list.insert({current->x, current->y});

        for (int dx = -1; dx <= 1; dx++){
            for (int dy = -1; dy <= 1; dy++){
                if (dx == 0 && dy == 0) continue;

                int nx = current->x + dx;
                int ny = current->y + dy;

                if (nx < 0 || ny < 0 || nx >= (int)grid_map[0].size() || ny >= (int)grid_map.size()) continue;
                if (grid_map[ny][nx] > 50) continue;
                if (closed_list.find({nx, ny}) != closed_list.end()) continue; //결과가 .end()가 아니면 이미 방문했단 뜻

                double move_cost = (dx != 0 && dy != 0) ? sqrt(2) : 1.0;
                double g = current->g_cost + move_cost;
                double h = h_cost(nx, ny);

                std::pair<int, int> neighbor_pos = {nx, ny};
                if(g_cost_map.find(neighbor_pos) == g_cost_map.end() || g < g_cost_map[neighbor_pos]){
                    g_cost_map[neighbor_pos] = g;
                    auto neighbor = std::make_shared<Node>(nx, ny, g, h, current);
                    open_list.push(neighbor);
                }
            }
        }
    }
    return path;
}

double GlobalPath::h_cost(int x, int y){
    return sqrt((x - goal_xc) * (x - goal_xc) + (y - goal_yc) * (y - goal_yc));
}

std::vector<std::vector<int>> GlobalPath::convert_mapTo2D(){
    int width = map.info.width;
    int height = map.info.height;

    std::vector<std::vector<int>> map2D(height, std::vector<int>(width, 0));

    for (int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){
            int index = y * width + x;
            map2D[y][x] = map.data[index];
        }
    }

    int obstacle_radius = 5;
    std::vector<std::vector<int>> obstacle_map = map2D;

    for (int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){
            if (map2D[y][x] > 50){
                for (int dy = - obstacle_radius; dy <= obstacle_radius; dy++){
                    for (int dx = - obstacle_radius; dx <= obstacle_radius; dx++){
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height){
                            obstacle_map[ny][nx] = 100;
                        }
                    }
                }
            }
        }
    }
    return obstacle_map;
}

void GlobalPath::RealToGrid(){
    robot_xc = static_cast<int>((robot_x - map.info.origin.position.x) / map.info.resolution);
    robot_yc = static_cast<int>((robot_y - map.info.origin.position.y) / map.info.resolution);

    goal_xc = static_cast<int>((goal_x - map.info.origin.position.x) / map.info.resolution);
    goal_yc = static_cast<int>((goal_y - map.info.origin.position.y) / map.info.resolution);
}