#include "rclcpp/rclcpp.hpp"
#include "isaac_tb3_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IsaacsimTurtlebotController>());
    rclcpp::shutdown();
    return 0;
}
