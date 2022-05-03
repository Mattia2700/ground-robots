#include <cstdio>
#include "../include/g_robot_lifecycle/GroundRobotManager.hpp"
#include "../include/g_robot_lifecycle/GroundRobotService.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    
    std::shared_ptr<GroundRobotManager> node = std::make_shared<GroundRobotManager>("ground_robot_manager");

    exe.add_node(node->get_node_base_interface());
    exe.spin();
    
    return 0;
}
