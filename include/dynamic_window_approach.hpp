

class DynamicWindowApproach
{
public:
    DynamicWindowApproach(double goal_x, double goal_y);

private:
    double goal_cost();
    double obstacle_cost();
    double velocity_cost();
};