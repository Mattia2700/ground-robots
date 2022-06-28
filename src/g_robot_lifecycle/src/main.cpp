#include <cstdio>
#include "GroundRobotManager.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    
    std::shared_ptr<GroundRobotManager> node = std::make_shared<GroundRobotManager>("ground_robot_manager");
    auto state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if(state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    // check
    

    exe.add_node(node->get_node_base_interface());
    exe.spin();
    
    return 0;
}
