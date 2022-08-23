#include <memory>
#include <algorithm>
#include <fstream>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "planning_bridge_msgs/srv/start_navigation.hpp"
#include "planning_bridge_msgs/srv/current_pose.hpp"

// utils
#include "json.hpp"

#define GOAL_INDEX 2
#define JSON_NAME "waypoints.json"

using json = nlohmann::json;
using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient {
public:
  MoveAction() : plansys2::ActionExecutorClient("ugv_transporting_uav_move", 250ms) {
    progress_ = 0.0;

    start_navigation_client_ = this->create_client<planning_bridge_msgs::srv::StartNavigation>("start_navigation");

    current_pose_client_ptr_ = create_client<planning_bridge_msgs::srv::CurrentPose>("get_current_pose");

    waypoints_file_path_ = current_file_path_.substr(0,
    current_file_path_.find_last_of("/")); waypoints_file_path_ =
    waypoints_file_path_.substr(0,
    waypoints_file_path_.find_last_of("/")).append("/config/" JSON_NAME);
    waypoints_file_ = std::ifstream(waypoints_file_path_);

    // check if waypoints_file_ exists
    std::cout << "Reading waypoints from " << waypoints_file_path_ << std::endl; 
    if(!waypoints_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "waypoints_dict.json not found");
      return;
    } else {
      labels_ = json::parse(waypoints_file_);
      waypoints_file_.close();
    }
  }

private:
  void do_work() {
    if(!is_initial_distance_set_){
      room_goal_ = current_arguments_[GOAL_INDEX];
      goal_position_ = labels_.at(room_goal_).at("center").get<std::vector<double>>();
      
      auto request = std::make_shared<planning_bridge_msgs::srv::StartNavigation::Request>();
      request->x = goal_position_[0];
      request->y = goal_position_[1];
      auto future_pose = this->start_navigation_client_->async_send_request(request);
      
    }
    get_pose();
  }

  void continue_work(){
    progress_ = 1.0 - (current_distance_ /
    initial_distance_);
    
    if (progress_ < 0.95) {
      send_feedback(progress_, "Move running");
    } else {
      finish(true, 1.0, "Move completed");

      progress_ = 0.0;
      is_initial_distance_set_ = false;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving ... [" << std::min(100.0, progress_ * 100.0) << "%]" << std::flush;
  }

  void get_pose() {
    auto future_pose = this->current_pose_client_ptr_->async_send_request(
        std::make_unique<planning_bridge_msgs::srv::CurrentPose::Request>(),
        [this](rclcpp::Client<planning_bridge_msgs::srv::CurrentPose>::SharedFuture future) {
          auto result = future.get();
          if(!is_initial_distance_set_){
            this->initial_distance_ = std::sqrt(std::pow(result->current_pose.x - goal_position_[0], 2) +
            std::pow(result->current_pose.y - goal_position_[1], 2));
            is_initial_distance_set_ = true;
            this->current_distance_ = this->initial_distance_;
          } else {
            this->current_distance_ = std::sqrt(std::pow(result->current_pose.x - goal_position_[0], 2) +
            std::pow(result->current_pose.y - goal_position_[1], 2));
            if(this->current_distance_ > this->initial_distance_){
              float temp = this->current_distance_;
              this->current_distance_ = this->initial_distance_;
              this->initial_distance_ = temp;
            }
          }
          this->continue_work();
        });
  }

  float progress_;
  json labels_;
  std::string room_goal_;
  std::vector<double> goal_position_;
  float initial_distance_;
  bool is_initial_distance_set_ = false;
  float current_distance_;
  std::string current_file_path_ = __FILE__;
  std::string waypoints_file_path_;
  std::ifstream waypoints_file_;
  rclcpp::Client<planning_bridge_msgs::srv::StartNavigation>::SharedPtr start_navigation_client_;
  rclcpp::Client<planning_bridge_msgs::srv::CurrentPose>::SharedPtr current_pose_client_ptr_;
};  

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "ugv_transporting_uav_move"));
  node->trigger_transition(
  lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}