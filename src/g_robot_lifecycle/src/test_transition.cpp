#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "../include/g_robot_lifecycle/GroundRobotService.hpp"

int main(int argc, char **argv) {
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto grobot_service = std::make_shared<GroundRobotService>("ground_robot_service");
    
    grobot_service->init();
    grobot_service->get_state();
    grobot_service->change_state(1,false);
    grobot_service->get_state();
    grobot_service->change_state(3);
    grobot_service->get_state();
    grobot_service->change_state(4,false);
    grobot_service->get_state();
    grobot_service->change_state(6);
    grobot_service->get_state();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(grobot_service);

    exe.spin();

    rclcpp::shutdown();

    return 0;
}