#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "../include/g_robot_lifecycle/GroundRobotService.hpp"

#include "lifecycle_msgs/msg/transition.hpp"

int main(int argc, char **argv) {
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto grobot_service = std::make_shared<GroundRobotService>("ground_robot_service");

    lifecycle_msgs::msg::Transition transition;
    
    grobot_service->init();
    grobot_service->change_state(transition.TRANSITION_CONFIGURE);
    if(grobot_service->get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        grobot_service->change_state(transition.TRANSITION_ACTIVATE);
    }

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(grobot_service);

    exe.spin();

    rclcpp::shutdown();

    return 0;
}