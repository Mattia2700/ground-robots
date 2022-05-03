#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using callback_return = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GroundRobotManager : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit GroundRobotManager(const std::string &node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
            // Create a publisher for the ground robot state
        }

        virtual ~GroundRobotManager() {
            this->_pub_ground_robot_state.reset();
        }

        callback_return on_configure(const rclcpp_lifecycle::State &) {
            RCLCPP_INFO(this->get_logger(), "Robot is configuring");

            // initialize robot state publisher
            this->_pub_ground_robot_state = this->create_publisher<std_msgs::msg::String>("ground_robot_state", 10);

            // TODO: setup robot devices

            return callback_return::SUCCESS;
        }

        callback_return on_activate(const rclcpp_lifecycle::State &) {
            this->_pub_ground_robot_state->on_activate();
            RCLCPP_INFO(this->get_logger(), "Robot is activating");
            
            // TODO: launch navigation stack
            
            std_msgs::msg::String msg;
            msg.data = "Hi Filippo, I'm alive!";
            this->_pub_ground_robot_state->publish(msg);

            return callback_return::SUCCESS;
        }

        callback_return on_shutdown(const rclcpp_lifecycle::State & ) {
            std_msgs::msg::String msg;
            msg.data = "Bye Filippo, I'm dead!";
            this->_pub_ground_robot_state->publish(msg);

            this->_pub_ground_robot_state.reset();

            RCLCPP_INFO(this->get_logger(), "Robot is shutting down");

            // TODO: Stop nodes and services
            
            return callback_return::SUCCESS;
        }

    private:
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> _pub_ground_robot_state;

};