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

/**
 * @brief A class implementing robot movement to a given goal 
 * 
 */
class MoveAction : public plansys2::ActionExecutorClient {
public:
  /**
   * @brief Reads waypoints from a json file containing the coordinates of each room
   * 
   */
  MoveAction() : plansys2::ActionExecutorClient("ugv_move", 250ms) {
    progress_ = 0.0;

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
  /**
   * @brief Called when an action needs to be executed, it sends a request to the navigation client, requesting the robot to move to the specified goal coordinates
   * 
   */
  void do_work() {
    if(!is_initial_distance_set_){
      room_goal_ = current_arguments_[GOAL_INDEX];
      goal_position_ = labels_.at(room_goal_).at("center").get<std::vector<double>>();
      
      auto request = std::make_shared<planning_bridge_msgs::srv::StartNavigation::Request>();
      request->x = goal_position_[0];
      request->y = goal_position_[1];
      
      start_navigation_client_->async_send_request(request,
      [this](rclcpp::Client<planning_bridge_msgs::srv::StartNavigation>::SharedFuture future){
        auto result = future.get();
        if(result->success) {
          RCLCPP_INFO(get_logger(), "Navigation started");
          get_pose();
        } else {
          RCLCPP_ERROR(get_logger(), "Navigation failed");
        }
      });
      
    } else {
      get_pose();
    }
  }
  /**
   * @brief When the goal has been set, and a new pose has been received, it will calculate a new percentage of completion and send it to the planner
   * 
   */
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

  /**
   * @brief After a goal has been set, it will request the current pose of the robot, in order to calculate the remaining distance to the goal. If it is the first time, it will also set the initial distance, later used to calculate the progress
   * 
   */
  void get_pose() {
    current_pose_client_ptr_->async_send_request(
        std::make_unique<planning_bridge_msgs::srv::CurrentPose::Request>(),
        [this](rclcpp::Client<planning_bridge_msgs::srv::CurrentPose>::SharedFuture future) {
          auto result = future.get();
          if(!is_initial_distance_set_){
            initial_distance_ = std::sqrt(std::pow(result->x - goal_position_[0], 2) +
            std::pow(result->y - goal_position_[1], 2));
            is_initial_distance_set_ = true;
            current_distance_ = initial_distance_;
          } else {
            current_distance_ = std::sqrt(std::pow(result->x - goal_position_[0], 2) +
            std::pow(result->y - goal_position_[1], 2));
            if(current_distance_ > initial_distance_){
              float temp = current_distance_;
              current_distance_ = initial_distance_;
              initial_distance_ = temp;
            }
          }
          continue_work();
        });
  }
  /**
   * @brief Percentage of completion of the action
   */
  float progress_;
  /**
   * @brief Json file containing the coordinates of each room
   */
  json labels_;
  /**
   * @brief Room name of the goal 
   */
  std::string room_goal_;
  /**
   * @brief Coordinates of the goal
   */
  std::vector<double> goal_position_;
  /**
   * @brief Initial distance to the goal
   */
  float initial_distance_;
  /**
   * @brief Whether the initial distance has been set
   */
  bool is_initial_distance_set_ = false;
  /**
   * @brief Current distance to the goal
   */
  float current_distance_;
  /**
   * @brief Current file path
   */
  std::string current_file_path_ = __FILE__;
  /**
   * @brief Json file path
   */
  std::string waypoints_file_path_;
  /**
   * @brief Json file input stream
   */
  std::ifstream waypoints_file_;
  /**
   * @brief Service clients used to start the navigation to the desired goal   * 
   */
  rclcpp::Client<planning_bridge_msgs::srv::StartNavigation>::SharedPtr start_navigation_client_ = create_client<planning_bridge_msgs::srv::StartNavigation>("start_navigation");

  rclcpp::Client<planning_bridge_msgs::srv::CurrentPose>::SharedPtr current_pose_client_ptr_ = create_client<planning_bridge_msgs::srv::CurrentPose>("get_current_pose");
};  

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "ugv_move"));
  node->trigger_transition(
  lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}