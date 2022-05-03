#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

static constexpr char const *get_state_topic =  "ground_robot_manager/get_state";
static constexpr char const *change_state_topic =  "ground_robot_manager/change_state";

class GroundRobotService : public rclcpp::Node {
    public:
        explicit GroundRobotService(const std::string &node_name) : Node(node_name) {
            RCLCPP_INFO(this->get_logger(), "GroundRobotService node created");
        }

        void init(){
            this->_srv_get_state = this->create_client<lifecycle_msgs::srv::GetState>(get_state_topic);
            this->_srv_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_topic);
        }

        unsigned int get_state() {
            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

            // wait for the service to being started
            while (!this->_srv_get_state->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            // send request
            auto result = this->_srv_get_state->async_send_request(request);

            //wait for response
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
                auto state = result.get()->current_state;
                RCLCPP_INFO(this->get_logger(), "Current state: %s", state.label.c_str());
                return state.id;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get current state");
                return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }
        }

        bool change_state(std::uint8_t transition, bool wait_before_change = true) {

            if(wait_before_change) wait(5000ms);

            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition;

            while (!this->_srv_change_state->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            // send request
            auto result = this->_srv_change_state->async_send_request(request);

            //wait for response
            if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
                return false;
            }

        }
    
    private:
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> _srv_get_state;
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> _srv_change_state;
        void wait(std::chrono::milliseconds duration) {
            std::this_thread::sleep_for(duration);
        }
};